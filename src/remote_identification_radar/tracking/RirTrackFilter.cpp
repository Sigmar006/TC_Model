/**
 * @file RirTrackFilter.cpp
 * @brief RIR 轻量跟踪子集的单目标 Kalman 滤波器实现（阶段 2-T T1）。
 */

/**
 * ## 过程：CV 卡尔曼滤波（LRDR 跟踪/识别链 · 第 2 步）
 * - 上游：生命周期命中的量测（位置 + 可选动态 R）与上一周期后验高斯状态。
 * - 本步：6 维恒速状态 [x, vx, y, vy, z, vz]——predict 用 F(dt) 外推均值、
 *   P←F·P·Fᵀ+Q（Q 由过程噪声差异系数缩放）；update 用 H 只取位置三轴，
 *   S = H·P·Hᵀ + R，K = P·Hᵀ·S⁻¹；初始化按"位置有先验、速度无知"给对角协方差。
 * - 下游：回写 `RirTrackState::gaussian_state/position/velocity`，供关联外推、识别运动
 *   特征与验收第 42 项（位置/速度/加速度估计与状态协方差）消费。
 * - 关键参数：`RirTrackFilterConfig::process_noise_diff_coeff`（默认 1.0，下限 0.001
 *   钳制）、`default_measurement_noise_std`（默认 10 m，动态 R 缺省兜底）、
 *   `initial_state_variance`（100 m²，位置先验）、`initial_velocity_std_mps`
 *   （3000 m/s，速度无知先验，方差 9e6 m²/s²）。
 * - 对应验收项：第 42 项 跟踪滤波功能测试（位置/速度/加速度 + 状态协方差 + 估计误差）。
 */

#include "remote_identification_radar/tracking/RirTrackFilter.h"

#include <algorithm>

namespace remote_identification_radar {
namespace tracking {

namespace {

/** @brief 过程噪声差异系数下限（与 AR SignalComponentFactory 工厂钳制同口径）。 */
// 合并版第 8 条：下限改为配置项 RirTrackFilterConfig::min_noise_diff_coeff（原硬编码 0.001）。
/** @brief 量测噪声标准差下限（与 AR SignalComponentFactory 工厂钳制同口径）。 */
constexpr float kMinimumMeasurementNoiseStd = 0.001f;

// 过程：配置 → 公共预测器参数——过程噪声差异系数 q 决定 Q 的幅值，F 由每步 dt 生成。
::oneq::common::estimation::KalmanPredictorConfig MakePredictorConfig(
    const RirTrackFilterConfig& config) {
  ::oneq::common::estimation::KalmanPredictorConfig common_config;
  // 越界配置以非法 q 运行会污染协方差传播：与 AR 同口径下限钳制。
  common_config.noise_diff_coeff = std::max(config.process_noise_diff_coeff, config.min_noise_diff_coeff);
  return common_config;
}

// 过程：配置 → 公共更新器参数——只有缺省量测噪声标准差（动态 R 缺省时兜底）。
::oneq::common::estimation::KalmanUpdaterConfig MakeUpdaterConfig(
    const RirTrackFilterConfig& config) {
  ::oneq::common::estimation::KalmanUpdaterConfig common_config;
  common_config.measurement_noise_std =
      std::max(config.default_measurement_noise_std, kMinimumMeasurementNoiseStd);
  return common_config;
}

}  // namespace

// 过程：滤波器 = 公共 6 维 CV 预测器 + 3 维位置更新器（H 只取 [x, y, z] 三行）；
//       构造时把配置钳制后下发，避免非法 q/R 污染协方差传播。
RirTrackFilter::RirTrackFilter(RirTrackFilterConfig config)
    : config_(config),
      predictor_(MakePredictorConfig(config)),
      updater_(MakeUpdaterConfig(config)) {}

// 过程：以量测位置 + 量测携带速度（可为零向量）建初始高斯状态。
RirGaussianState RirTrackFilter::Initialize(const RirTrackMeasurement& measurement) const {
  return Initialize(measurement.position, measurement.velocity);
}

// 过程：状态序 [x, vx, y, vy, z, vz]，位置/速度按轴交错填均值；
//       协方差只置对角，位置与速度先验互不相关（无交叉项）。
RirGaussianState RirTrackFilter::Initialize(const Eigen::Vector3f& position,
                                            const Eigen::Vector3f& velocity) const {
  RirStateVector mean = RirStateVector::Zero();
  mean(0) = position.x();
  mean(1) = velocity.x();
  mean(2) = position.y();
  mean(3) = velocity.y();
  mean(4) = position.z();
  mean(5) = velocity.z();

  // 诚实初始化（rir-tracking-realism 契约）：位置项=位置先验，速度项=速度无知
  // 先验——去真值种子后速度未知性须显式建模；先验越大建轨期门越宽、收敛后自然
  // 收紧（门自适应来源，无需按航迹年龄特判）。
  RirStateCovariance covariance = RirStateCovariance::Zero();
  const float position_variance = config_.initial_state_variance;
  const float velocity_variance =
      config_.initial_velocity_std_mps * config_.initial_velocity_std_mps;
  covariance(0, 0) = position_variance;
  covariance(2, 2) = position_variance;
  covariance(4, 4) = position_variance;
  covariance(1, 1) = velocity_variance;
  covariance(3, 3) = velocity_variance;
  covariance(5, 5) = velocity_variance;
  return RirGaussianState(mean, covariance);
}

// 过程：CV 预测一步——mean←F·mean（位置 += 速度·dt），P←F·P·Fᵀ+Q(dt, q)。
RirGaussianState RirTrackFilter::Predict(const RirGaussianState& prior, float dt_sec) const {
  return predictor_.Predict(prior, dt_sec);
}

// 过程：以量测自带协方差作动态 R 更新（优先于配置缺省 R）。
RirKalmanUpdateResult RirTrackFilter::Update(const RirGaussianState& predicted,
                                             const RirTrackMeasurement& measurement) const {
  const RirMeasurementVector position = measurement.position;
  return updater_.Update(predicted, position, measurement.measurement_covariance);
}

// 过程：标准 KF 更新——S = H·P·Hᵀ + R，K = P·Hᵀ·S⁻¹，
//       后验均值 = 先验 + K·(z − H·先验)，后验协方差 = (I − K·H)·P。
RirKalmanUpdateResult RirTrackFilter::Update(
    const RirGaussianState& predicted, const Eigen::Vector3f& position,
    const RirMeasurementCovariance& measurement_covariance) const {
  return updater_.Update(predicted, position, measurement_covariance);
}

// 过程：热更新同步配置副本与预测/更新器参数，已存在的航迹状态不受影响。
void RirTrackFilter::UpdateConfig(RirTrackFilterConfig config) {
  config_ = config;
  predictor_.UpdateConfig(MakePredictorConfig(config));
  updater_.UpdateConfig(MakeUpdaterConfig(config));
}

}  // namespace tracking
}  // namespace remote_identification_radar
