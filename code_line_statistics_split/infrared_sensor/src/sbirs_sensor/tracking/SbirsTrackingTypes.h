/**
 * @file SbirsTrackingTypes.h
 * @brief SBIRS 红外滤波测量跟踪 facade：消费 common/estimation 模板化滤波框架，实例化为 SBIRS
 *        6 维 CV 状态 / 2 维角度量测场景，并提供球坐标角度量测模型与动态 R 矩阵构造。
 *
 * 设计要点（见 docs/sbirs_sensor/algorithms.md 目标状态机 / EKF 滤波跟踪）：
 * - 状态：6 维 ECI 恒速 [x, vx, y, vy, z, vz]，复用 common CV 模型（2026-08 正式变更：
 *   ECI 输出参考系——pipeline 周期入口已把真值旋转到 ECI，滤波状态随之在 ECI 中演化；
 *   CV 模型对 ECI 恒速目标更贴合，且不含 ECEF 的科氏耦合）。
 * - 量测：2 维球坐标角度 [az, el]（被动红外不测距）。h(x) 非线性，走 EKF。
 * - 角度量测模型 SbirsAngleMeasurementModel：h(x) = 目标 ECI 相对卫星位置的 LOS → az/el；
 *   Jacobian 解析求导。卫星 ECI 位置每帧由 pipeline 通过 SetSatellitePosition 注入。
 * - R 矩阵 BuildMeasurementCovariance：从 SbirsErrorModelConfig 的 5 类误差 1-σ 合成 2×2，
 *   随距离/俯仰/角速度动态变化。
 */

#ifndef SBIRS_SENSOR_TRACKING_SBIRS_TRACKING_TYPES_H_
#define SBIRS_SENSOR_TRACKING_SBIRS_TRACKING_TYPES_H_

#include <cmath>

#include "1q/sbirs_sensor/config/SbirsPolicyConfig.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"
#include "common/estimation/EkfFilter.h"
#include "common/estimation/GaussianState.h"
#include "common/estimation/IKalmanPredictor.h"
#include "common/estimation/IKalmanUpdater.h"
#include "common/estimation/ImmFilter.h"
#include "common/numerics/Constants.h"
#include "common/estimation/KalmanPredictor.h"
#include "sbirs_sensor/foundation/SbirsErrorModel.h"
#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace tracking {

/** @brief SBIRS 红外滤波状态维度（3D 恒速 ECI）。 */
static constexpr int kSbirsStateDim = 6;
/** @brief SBIRS 红外量测维度（方位 + 俯仰角度）。 */
static constexpr int kSbirsMeasurementDim = 2;

/** @brief SBIRS 红外高斯状态（6 维状态 / 2 维角度量测）。 */
using SbirsGaussianState = ::oneq::common::estimation::GaussianState<kSbirsStateDim, kSbirsMeasurementDim>;
/** @brief SBIRS EKF 预测器（6/2 实例化，复用 CV 转移模型）。 */
using SbirsEkfPredictor = ::oneq::common::estimation::EkfPredictor<kSbirsStateDim, kSbirsMeasurementDim>;
/** @brief SBIRS EKF 更新器（6/2 实例化）。 */
using SbirsEkfUpdater = ::oneq::common::estimation::EkfUpdater<kSbirsStateDim, kSbirsMeasurementDim>;
/** @brief SBIRS EKF 预测器配置。 */
using SbirsEkfPredictorConfig = ::oneq::common::estimation::EkfPredictorConfig;
/** @brief SBIRS EKF 更新器配置。 */
using SbirsEkfUpdaterConfig = ::oneq::common::estimation::EkfUpdaterConfig;
/** @brief SBIRS 线性 CV 转移模型（6 维状态默认实现）。 */
using SbirsCvTransitionModel = ::oneq::common::estimation::LinearCvTransitionModel<kSbirsStateDim>;
/** @brief SBIRS 更新结果。 */
using SbirsKalmanUpdateResult =
    ::oneq::common::estimation::KalmanUpdateResult<kSbirsStateDim, kSbirsMeasurementDim>;

using SbirsStateVector = SbirsGaussianState::StateVector;
using SbirsStateCovariance = SbirsGaussianState::StateCovariance;
using SbirsMeasurementVector = SbirsGaussianState::MeasurementVector;
using SbirsMeasurementCovariance = SbirsGaussianState::MeasurementCovariance;

/**
 * @brief SBIRS 红外球坐标角度量测模型。
 * @details h(x) = [atan2(los_y, los_x), asin(los_z / |los|)]，其中 los = x.pos - satellite_position。
 *          量测向量单位为弧度（rad），由 pipeline 在比较时与 deg 量测互转。
 *          Jacobian H = ∂h/∂x 解析求导，对位置分量非零、速度分量为零。
 *
 *          卫星位置不是状态的一部分，由 pipeline 在每帧 update 前通过 SetSatellitePosition 注入。
 *          这避免了把卫星位置塞进状态向量，保持状态纯目标运动学。
 */
class SbirsAngleMeasurementModel final
    : public ::oneq::common::estimation::IMeasurementModel<kSbirsStateDim, kSbirsMeasurementDim> {
 public:
  SbirsAngleMeasurementModel() = default;

  /**
   * @brief 设置当前帧卫星 ECI 位置（每帧 update 前调用；与状态同一参考系）。
   * @param[in] satellite_position_eci_m 卫星 ECI 位置（米）。
   */
  void SetSatellitePosition(const session::SbirsVector3M& satellite_position_eci_m) {
    satellite_position_ = satellite_position_eci_m;
  }

  /** @brief h(x) = 目标 ECI 相对卫星的 LOS → [az, el]（弧度，ECI 极坐标）。 */
  SbirsMeasurementVector Function(const SbirsStateVector& state) const override {
    // 状态布局 CV 交错 [x,vx,y,vy,z,vz]：位置在偶数索引 0/2/4。
    const double dx = static_cast<double>(state(0)) - satellite_position_.x;
    const double dy = static_cast<double>(state(2)) - satellite_position_.y;
    const double dz = static_cast<double>(state(4)) - satellite_position_.z;
    const double range = std::sqrt(dx * dx + dy * dy + dz * dz);

    SbirsMeasurementVector z;
    if (range <= 0.0) {
      z << 0.0f, 0.0f;
      return z;
    }
    const double az = std::atan2(dy, dx);
    // 仰角 = asin(dz / range)，钳制到 [-1,1] 防 NaN。
    const double sin_el = dz / range;
    const double clamped = sin_el < -1.0 ? -1.0 : (sin_el > 1.0 ? 1.0 : sin_el);
    const double el = std::asin(clamped);
    z << static_cast<float>(az), static_cast<float>(el);
    return z;
  }

  /** @brief Jacobian H = ∂[az,el]/∂[x,vx,y,vy,z,vz]，速度列为零。 */
  SbirsGaussianState::MeasurementMatrix Jacobian(const SbirsStateVector& state) const override {
    // CV 交错布局：位置 x/y/z 在状态索引 0/2/4。
    const double dx = static_cast<double>(state(0)) - satellite_position_.x;
    const double dy = static_cast<double>(state(2)) - satellite_position_.y;
    const double dz = static_cast<double>(state(4)) - satellite_position_.z;
    const double range_sq = dx * dx + dy * dy + dz * dz;
    const double range = std::sqrt(range_sq);
    const double horiz_sq = dx * dx + dy * dy;
    const double horiz = std::sqrt(horiz_sq);

    SbirsGaussianState::MeasurementMatrix H = SbirsGaussianState::MeasurementMatrix::Zero();
    if (range <= 0.0 || horiz <= 0.0) {
      return H;  // 退化几何，零 Jacobian（update 会回退到预测态）
    }
    // ∂az/∂x = -dy/horiz_sq（x 在状态索引 0）, ∂az/∂y = dx/horiz_sq（y 在索引 2）
    H(0, 0) = static_cast<float>(-dy / horiz_sq);
    H(0, 2) = static_cast<float>(dx / horiz_sq);
    // ∂el/∂x = (-dz·dx)/(range_sq·horiz), ∂el/∂y = (-dz·dy)/(range_sq·horiz)
    H(1, 0) = static_cast<float>(-dz * dx / (range_sq * horiz));
    H(1, 2) = static_cast<float>(-dz * dy / (range_sq * horiz));
    // ∂el/∂z = horiz/range_sq（z 在索引 4）
    H(1, 4) = static_cast<float>(horiz / range_sq);
    return H;
  }

 private:
  session::SbirsVector3M satellite_position_{};
};

/**
 * @brief 由误差模型配置构造量测噪声协方差 R（2×2，单位 rad²）。
 * @details 把 design 2.10 的 5 类角度误差 1-σ（orbit/attitude/fov 高斯 + 折射 + 滞后）RSS 合成，
 *          转为弧度并平方得方差。折射/滞后随距离/俯仰/角速度变化，故 R 动态。
 * @param[in] error_model 误差模型配置（sigma 单位 deg）。
 * @param[in] range_m 目标距离（米，用于折射项）。
 * @param[in] elevation_deg 目标俯仰角（deg，用于折射项）。
 * @param[in] relative_angular_rate_deg_per_sec 相对视线角速度（v_target−v_satellite 推导，
 *                deg/s，用于动态滞后项）。
 * @return 2×2 对角协方差矩阵（rad²）。
 */
inline SbirsMeasurementCovariance BuildMeasurementCovariance(
    const config::SbirsErrorModelConfig& error_model, double range_m, float elevation_deg,
    float relative_angular_rate_deg_per_sec) {
  const double angular_sigma_deg =
      foundation::ResolveEffectiveAngularSigmaDeg(error_model);
  const double gauss_sq = angular_sigma_deg * angular_sigma_deg;

  // 确定性项：折射（随距离/俯仰）与动态滞后（随角速度/带宽）。
  const double refraction_deg = [range_m, elevation_deg]() -> double {
    if (range_m <= 0.0) return 0.0;
    const double cos_beta = std::cos(static_cast<double>(elevation_deg) * 3.14159265358979323846 / 180.0);
    const double denom = range_m * std::fabs(cos_beta);
    return denom <= 0.0 ? 0.0 : 1.5e-6 / denom;
  }();
  const double lag_deg = error_model.detector_bandwidth_hz > 0.0f
                             ? static_cast<double>(relative_angular_rate_deg_per_sec) /
                                   (2.0 * 3.14159265358979323846 *
                                    static_cast<double>(error_model.detector_bandwidth_hz))
                             : 0.0;

  // 方差（deg²），各通道相同（5 类误差对 az/el 对称作用）。
  const double sigma_az_deg = std::sqrt(gauss_sq + refraction_deg * refraction_deg +
                                        lag_deg * lag_deg);
  const double sigma_el_deg = sigma_az_deg;

  // deg² → rad²。
  const double var_az_rad2 =
      oneq::common::numerics::DegToRad(sigma_az_deg) * oneq::common::numerics::DegToRad(sigma_az_deg);
  const double var_el_rad2 =
      oneq::common::numerics::DegToRad(sigma_el_deg) * oneq::common::numerics::DegToRad(sigma_el_deg);

  SbirsMeasurementCovariance R = SbirsMeasurementCovariance::Zero();
  R(0, 0) = static_cast<float>(var_az_rad2);
  R(1, 1) = static_cast<float>(var_el_rad2);
  return R;
}

// IMM facade：6 维状态 / 2 维角度量测
using SbirsImmConfig = ::oneq::common::estimation::ImmConfig;
using SbirsImmFilter = ::oneq::common::estimation::ImmFilter<kSbirsStateDim, kSbirsMeasurementDim>;
using SbirsImmModelState =
    ::oneq::common::estimation::ImmModelState<kSbirsStateDim, kSbirsMeasurementDim>;

/** @brief IMM 滤波状态快照（供 replay/capture 持久化）。 */
struct SbirsImmSnapshot {
  std::vector<SbirsImmModelState> model_states{};  /**< 各模型状态 + 权重 */
  Eigen::VectorXf model_weights{};                  /**< 模型权重向量（冗余，供校验） */
};

}  // namespace tracking
}  // namespace sbirs_sensor

#endif  // SBIRS_SENSOR_TRACKING_SBIRS_TRACKING_TYPES_H_
