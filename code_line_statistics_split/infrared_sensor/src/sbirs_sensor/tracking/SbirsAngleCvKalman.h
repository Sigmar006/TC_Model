/**
 * @file SbirsAngleCvKalman.h
 * @brief SBIRS 实验后端：4 维视线角恒速线性标准卡尔曼滤波（用例 16）。
 *
 * 状态 [az, ω_az, el, ω_el]（弧度 / 弧度每秒），量测 [az, el]。
 * 不复用公共 KalmanPredictor/KalmanUpdater 的 6 维笛卡尔 F/H。
 * 后验不得解释为三维位置或速度。
 */

#ifndef SBIRS_SENSOR_TRACKING_SBIRS_ANGLE_CV_KALMAN_H_
#define SBIRS_SENSOR_TRACKING_SBIRS_ANGLE_CV_KALMAN_H_

#include <cmath>

#include <Eigen/Cholesky>

#include "common/estimation/GaussianState.h"
#include "common/estimation/IKalmanPredictor.h"
#include "common/estimation/IKalmanUpdater.h"
#include "common/logging/ProjectLog.h"
#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace tracking {

static constexpr int kSbirsAngleStateDim = 4;
static constexpr int kSbirsAngleMeasurementDim = 2;

using SbirsAngleCvGaussianState =
    ::oneq::common::estimation::GaussianState<kSbirsAngleStateDim, kSbirsAngleMeasurementDim>;
using SbirsAngleCvUpdateResult =
    ::oneq::common::estimation::KalmanUpdateResult<kSbirsAngleStateDim, kSbirsAngleMeasurementDim>;

/** @brief 把方位折到 (-π, π]，便于最短弧新息。 */
inline float WrapAzimuthRad(float azimuth_rad) {
  return std::atan2(std::sin(azimuth_rad), std::cos(azimuth_rad));
}

/** @brief 方位最短弧差 lhs−rhs，结果在 (-π, π]。 */
inline float ShortestAzimuthDeltaRad(float lhs_rad, float rhs_rad) {
  return WrapAzimuthRad(lhs_rad - rhs_rad);
}

/** @brief 俯仰钳制到 [-π/2, π/2]。 */
inline float ClampElevationRad(float elevation_rad) {
  const float kHalfPi = static_cast<float>(0.5 * 3.14159265358979323846);
  if (elevation_rad < -kHalfPi) return -kHalfPi;
  if (elevation_rad > kHalfPi) return kHalfPi;
  return elevation_rad;
}

/**
 * @brief 用当前角度点迹初始化，变化率置 0；不用三维真值。
 * @details 初始角方差对应 1° 1-σ，变化率方差对应 1°/s 1-σ。
 */
inline SbirsAngleCvGaussianState MakeInitialAngleCvState(float azimuth_rad, float elevation_rad) {
  SbirsAngleCvGaussianState state;
  state.mean(0) = WrapAzimuthRad(azimuth_rad);
  state.mean(1) = 0.0f;
  state.mean(2) = ClampElevationRad(elevation_rad);
  state.mean(3) = 0.0f;
  const float angle_std = oneq::common::numerics::DegToRad(1.0f);
  const float rate_std = oneq::common::numerics::DegToRad(1.0f);
  const float angle_var = angle_std * angle_std;
  const float rate_var = rate_std * rate_std;
  state.covariance = SbirsAngleCvGaussianState::StateCovariance::Zero();
  state.covariance(0, 0) = angle_var;
  state.covariance(1, 1) = rate_var;
  state.covariance(2, 2) = angle_var;
  state.covariance(3, 3) = rate_var;
  return state;
}

/**
 * @brief 两轴角度恒速线性预测器。
 * @details F 为每轴 |1 dt; 0 1| 块对角；Q 为连续白噪声加速度离散化。
 *          q = process_noise_diff_coeff，单位 rad²/s³。
 */
class SbirsAngleCvPredictor final
    : public ::oneq::common::estimation::IKalmanPredictor<kSbirsAngleStateDim,
                                                          kSbirsAngleMeasurementDim> {
 public:
  using GaussianStateT = SbirsAngleCvGaussianState;
  using TransitionMatrix = typename GaussianStateT::TransitionMatrix;
  using ProcessNoiseCovariance = typename GaussianStateT::ProcessNoiseCovariance;

  explicit SbirsAngleCvPredictor(::oneq::common::estimation::KalmanPredictorConfig config = {})
      : config_(config) {}

  GaussianStateT Predict(const GaussianStateT& prior, float dt) const override {
    GaussianStateT predicted = prior;
    if (!(dt > 0.0f) || !std::isfinite(dt)) {
      predicted.mean(0) = WrapAzimuthRad(predicted.mean(0));
      predicted.mean(2) = ClampElevationRad(predicted.mean(2));
      return predicted;
    }
    const TransitionMatrix F = BuildTransitionMatrix(dt);
    const ProcessNoiseCovariance Q = BuildProcessNoise(dt, config_.noise_diff_coeff);
    predicted.mean = F * prior.mean;
    predicted.covariance = F * prior.covariance * F.transpose() + Q;
    predicted.mean(0) = WrapAzimuthRad(predicted.mean(0));
    predicted.mean(2) = ClampElevationRad(predicted.mean(2));
    return predicted;
  }

  void UpdateConfig(::oneq::common::estimation::KalmanPredictorConfig config) override {
    config_ = config;
  }

  static TransitionMatrix BuildTransitionMatrix(float dt) {
    TransitionMatrix F = TransitionMatrix::Identity();
    F(0, 1) = dt;
    F(2, 3) = dt;
    return F;
  }

  static ProcessNoiseCovariance BuildProcessNoise(float dt, float q) {
    ProcessNoiseCovariance Q = ProcessNoiseCovariance::Zero();
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    for (int axis = 0; axis < 2; ++axis) {
      const int base = axis * 2;
      Q(base, base) = dt3 / 3.0f;
      Q(base, base + 1) = dt2 / 2.0f;
      Q(base + 1, base) = dt2 / 2.0f;
      Q(base + 1, base + 1) = dt;
    }
    Q *= q;
    return Q;
  }

 private:
  ::oneq::common::estimation::KalmanPredictorConfig config_{};
};

/**
 * @brief 线性角度量测更新器：H 提取 [az, el]，方位新息走最短弧，协方差 Joseph 形式。
 */
class SbirsAngleCvUpdater final
    : public ::oneq::common::estimation::IKalmanUpdater<kSbirsAngleStateDim,
                                                        kSbirsAngleMeasurementDim> {
 public:
  using GaussianStateT = SbirsAngleCvGaussianState;
  using Result = SbirsAngleCvUpdateResult;
  using MeasurementVector = typename GaussianStateT::MeasurementVector;
  using MeasurementCovariance = typename GaussianStateT::MeasurementCovariance;
  using MeasurementMatrix = typename GaussianStateT::MeasurementMatrix;
  using KalmanGainMatrix = typename GaussianStateT::KalmanGainMatrix;
  using StateCovariance = typename GaussianStateT::StateCovariance;

  explicit SbirsAngleCvUpdater(::oneq::common::estimation::KalmanUpdaterConfig config = {})
      : R_(::oneq::common::estimation::IKalmanUpdater<kSbirsAngleStateDim, kSbirsAngleMeasurementDim>::
               BuildDefaultMeasurementNoise(config.measurement_noise_std)) {
    H_ = MeasurementMatrix::Zero();
    H_(0, 0) = 1.0f;
    H_(1, 2) = 1.0f;
  }

  Result Update(const GaussianStateT& predicted, const MeasurementVector& measurement) const override {
    return Update(predicted, measurement, R_);
  }

  Result Update(const GaussianStateT& predicted, const MeasurementVector& measurement,
                const MeasurementCovariance& dynamic_R) const override {
    Result result;
    MeasurementVector z_pred;
    z_pred(0) = predicted.mean(0);
    z_pred(1) = predicted.mean(2);
    result.innovation(0) = ShortestAzimuthDeltaRad(measurement(0), z_pred(0));
    result.innovation(1) = measurement(1) - z_pred(1);
    result.innovation_covariance = H_ * predicted.covariance * H_.transpose() + dynamic_R;

    const Eigen::LLT<MeasurementCovariance> llt(result.innovation_covariance);
    if (llt.info() != Eigen::Success) {
      // 中译：创新协方差 LLT 分解失败，本次角度 KF 更新被跳过。
      // 标识：数值保护——协方差非正定时后验=预测，防止发散。
      PROJECT_LOG_ERROR(
          "[SbirsAngleCvUpdater] Innovation covariance LLT decomposition failed; update is skipped.");
      result.posterior = predicted;
      return result;
    }
    const KalmanGainMatrix K = llt.solve(H_ * predicted.covariance).transpose();
    result.posterior.mean = predicted.mean + K * result.innovation;
    result.posterior.mean(0) = WrapAzimuthRad(result.posterior.mean(0));
    result.posterior.mean(2) = ClampElevationRad(result.posterior.mean(2));
    const StateCovariance I_KH = StateCovariance::Identity() - K * H_;
    result.posterior.covariance =
        I_KH * predicted.covariance * I_KH.transpose() + K * dynamic_R * K.transpose();
    return result;
  }

 private:
  MeasurementMatrix H_;
  MeasurementCovariance R_;
};

}  // namespace tracking
}  // namespace sbirs_sensor

#endif  // SBIRS_SENSOR_TRACKING_SBIRS_ANGLE_CV_KALMAN_H_
