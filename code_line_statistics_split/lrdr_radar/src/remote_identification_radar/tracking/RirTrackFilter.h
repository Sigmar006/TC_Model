/**
 * @file RirTrackFilter.h
 * @brief RIR 轻量跟踪子集的单目标 3D 恒速 Kalman 滤波器（阶段 2-T T1）。
 *
 * 副本来源：`src/airborne_radar/signal/tracking/KalmanPredictor.h` /
 * `KalmanUpdater.h`（审计基线 96de367c），底层数值原语为
 * `common/estimation/KalmanPredictor` / `KalmanUpdater` 的 6/3 实例化。
 * 状态布局 [x, vx, y, vy, z, vz]，量测为笛卡尔位置 [x, y, z]。
 * IMM 不迁（轻量边界，见 phase2 计划 D-A4）。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_FILTER_H_
#define REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_FILTER_H_

#include "common/estimation/KalmanPredictor.h"
#include "common/estimation/KalmanUpdater.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace tracking {

/** @brief RIR 单目标 KF 更新结果（6 维状态 / 3 维位置量测）。 */
using RirKalmanUpdateResult = ::oneq::common::estimation::KalmanUpdateResult<6, 3>;

/**
 * @brief RirTrackFilterConfig 单目标 KF 配置。
 */
struct RirTrackFilterConfig {
  /** @brief 连续白噪声加速度扩散系数 q（m/s²）。 */
  float process_noise_diff_coeff{1.0f};
  /** @brief 缺省位置量测噪声标准差（m），仅构造缺省 R 时使用。 */
  float default_measurement_noise_std{10.0f};
  /** @brief 新航迹初始位置协方差对角线（m²）。 */
  float initial_state_variance{100.0f};
  /**
   * @brief 新航迹初始速度先验标准差（m/s，无知先验）。
   * 2026-08-31 去真值化（rir-tracking-realism 契约）：量测不再携带真值速度种子，
   * 建轨速度从零均值 + 本先验起步、由滤波逐拍收敛；取值覆盖目标域典型速度上界
   * （缺省 3000 覆盖飞机~弹道域）。先验越大建轨期门越宽（协方差自适应来源）。
   */
  float initial_velocity_std_mps{3000.0f};
};

/**
 * @brief RirTrackFilter 单目标 Kalman 预测/更新滤波器。
 *
 * 调用契约：
 * - `Initialize` 从量测构造新航迹先验（位置 + 零速均值/速度无知先验协方差，
 *   2026-08-31 起速度种子不再取量测真值）；
 * - `Predict` 执行 CV 时间外推，调用方保证 dt > 0；
 * - `Update` 以动态量测协方差 R 执行标准线性更新；R 非正定导致 LLT 失败时
 *   底层更新器跳过更新并返回预测状态（数值保护）。
 */
class RirTrackFilter {
 public:
  /** @brief 构造滤波器。 */
  explicit RirTrackFilter(RirTrackFilterConfig config = {});

  /**
   * @brief 从量测构造初始高斯状态。
   * @param[in] measurement 新航迹量测（位置；velocity 字段不再作真值种子消费）。
   * @return 初始状态：位置来自量测、速度取量测速度均值（调用方现传零），位置项
   * 协方差 `initial_state_variance`、速度项协方差 `initial_velocity_std_mps`²。
   */
  RirGaussianState Initialize(const RirTrackMeasurement& measurement) const;

  /**
   * @brief 从位置与速度构造初始高斯状态。
   * @param[in] position 初始位置（m）。
   * @param[in] velocity 初始速度均值（m/s；调用方现传零，速度未知性由先验承载）。
   * @return 初始状态：位置项协方差 `initial_state_variance`、速度项协方差
   * `initial_velocity_std_mps`²。
   */
  RirGaussianState Initialize(const Eigen::Vector3f& position,
                              const Eigen::Vector3f& velocity) const;

  /**
   * @brief CV 模型状态预测。
   * @param[in] prior 先验状态。
   * @param[in] dt_sec 预测步长（s），须为正。
   * @return 预测状态（含过程噪声传播后的协方差）。
   */
  RirGaussianState Predict(const RirGaussianState& prior, float dt_sec) const;

  /**
   * @brief 以量测位置与动态 R 执行 Kalman 更新。
   * @param[in] predicted 预测状态。
   * @param[in] measurement 量测（position + measurement_covariance）。
   * @return 更新结果（后验、新息、新息协方差）。
   */
  RirKalmanUpdateResult Update(const RirGaussianState& predicted,
                               const RirTrackMeasurement& measurement) const;

  /**
   * @brief 以量测位置与显式动态 R 执行 Kalman 更新。
   * @param[in] predicted 预测状态。
   * @param[in] position 位置量测（m）。
   * @param[in] measurement_covariance 量测噪声协方差 R（m²）。
   * @return 更新结果。
   */
  RirKalmanUpdateResult Update(const RirGaussianState& predicted, const Eigen::Vector3f& position,
                               const RirMeasurementCovariance& measurement_covariance) const;

  /** @brief 全量更新滤波器配置。 */
  void UpdateConfig(RirTrackFilterConfig config);

 private:
  RirTrackFilterConfig config_{};
  ::oneq::common::estimation::KalmanPredictor<6, 3> predictor_;
  ::oneq::common::estimation::KalmanUpdater<6, 3> updater_;
};

}  // namespace tracking
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_FILTER_H_
