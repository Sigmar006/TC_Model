/**
 * @file SbirsTrackingCoordinator.h
 * @brief Internal owner of SBIRS estimated-tracking runtime state.
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_TRACKING_COORDINATOR_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_TRACKING_COORDINATOR_H_

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include "1q/sbirs_sensor/config/SbirsPolicyConfig.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"
#include "sbirs_sensor/foundation/SbirsErrorModel.h"
#include "sbirs_sensor/pipeline/SbirsEciScene.h"
#include "sbirs_sensor/tracking/SbirsAngleCvKalman.h"
#include "sbirs_sensor/tracking/SbirsTrackingTypes.h"

namespace sbirs_sensor {
namespace pipeline {

/** @brief Persisted tracking state mapped directly to SbirsPipelineSnapshot fields. */
struct SbirsTrackingRuntimeState {
  std::map<std::uint64_t, tracking::SbirsGaussianState> filter_states{};
  std::map<std::uint64_t, tracking::SbirsAngleCvGaussianState> angle_kf_states{};
  std::map<std::uint64_t, unsigned int> nis_gate_exceeded_counts{};
  bool imm_active{false};
  std::map<std::uint64_t, tracking::SbirsImmSnapshot> imm_snapshots{};
};

/** @brief Result of one estimated-tracking prediction. */
struct SbirsTrackingPredictionResult {
  float output_azimuth_deg{0.0f};
  float output_elevation_deg{0.0f};
};

/** @brief Result of one estimated-tracking measurement correction. */
struct SbirsTrackingUpdateResult {
  bool has_measurement{false};
  float measurement_azimuth_deg{0.0f};
  float measurement_elevation_deg{0.0f};
  tracking::SbirsMeasurementCovariance measurement_covariance{
      tracking::SbirsMeasurementCovariance::Zero()};
  float output_azimuth_deg{0.0f};
  float output_elevation_deg{0.0f};
  bool has_estimation_nis{false};
  float estimation_nis{0.0f};
  bool estimation_nis_gate_exceeded{false};
  bool lost_due_to_estimation_nis{false};
  bool innovation_covariance_singular{false}; /**< 创新协方差非正定 → NIS 无定义（不写 inf）。 */
  bool measurement_discarded{false};          /**< 本拍量测被判离群并丢弃（后验保持预测）。 */
  bool state_diverged{false};                 /**< 后验发散（非有限/量级越界），已丢弃该目标状态。 */
  bool has_angle_rate{false};              /**< kAngleCvKf 后验变化率是否有效 */
  float azimuth_rate_rad_per_s{0.0f};      /**< 方位变化率（rad/s），仅 kAngleCvKf */
  float elevation_rate_rad_per_s{0.0f};    /**< 俯仰变化率（rad/s），仅 kAngleCvKf */
};

/**
 * @brief 由目标/卫星的相对几何解析求出视线角速率（rad/s）。
 *
 * 与 `SbirsAngleCvKalman` 的后验变化率不同，EKF/IMM 的状态里没有角度速率的显式分量；
 * 但后验状态同时带位置与速度，故角速率可由相对位置/速度直接解出，
 * 不需要跨拍差分，也不需要额外的历史状态：
 *
 *   r_rel = r_target − r_sat，v_rel = v_target − v_sat
 *   ω_az = (x·v_y − y·v_x) / (x² + y²)
 *   ρ = √(x² + y²)
 *   ω_el = (ρ·v_z − z·(x·v_x + y·v_y)/ρ) / (ρ² + z²)
 *
 * @note **必须减掉卫星自身速度**：GEO 卫星在 ECEF 静止、在 ECI 约 3 km/s，
 *       否则派生值会被卫星运动主导（实测视线角速率量级 0.004–0.013 deg/s）。
 * @param[in] target_position_eci_m 目标后验位置（ECI，m）。
 * @param[in] target_velocity_eci_m 目标后验速度（ECI，m/s）。
 * @param[in] satellite_position_eci_m 卫星位置（ECI，m）。
 * @param[in] satellite_velocity_eci_m 卫星速度（ECI，m/s）。
 * @param[out] azimuth_rate_rad_per_s 方位角速率（rad/s）；不可用时保持原值。
 * @param[out] elevation_rate_rad_per_s 俯仰角速率（rad/s）；不可用时保持原值。
 * @return 几何退化（|r_rel| 的投影为零、非有限量）时返回 false，调用方应报"不适用"。
 */
bool TryDeriveAngleRateRadPerSec(const session::SbirsVector3M& target_position_eci_m,
                                 const session::SbirsVector3M& target_velocity_eci_m,
                                 const session::SbirsVector3M& satellite_position_eci_m,
                                 const session::SbirsVector3M& satellite_velocity_eci_m,
                                 float* azimuth_rate_rad_per_s,
                                 float* elevation_rate_rad_per_s);

/**
 * @brief Coordinates EKF/IMM/angle-CV-KF update state for targets already locked by SbirsPipeline.
 * @note Scheduling, target-state transitions, output construction, and snapshot schema remain pipeline-owned.
 */
class SbirsTrackingCoordinator {
 public:
  /** @brief 初始化滤波状态。EKF/IMM 用 ECI 真值；kAngleCvKf 只用量测角度，禁止真值三维。 */
  void InitializeTarget(std::uint64_t target_id, const SbirsEciSceneTarget& target,
                        const config::SbirsTrackingConfig& tracking, float measured_azimuth_deg,
                        float measured_elevation_deg);
  SbirsTrackingPredictionResult PredictTarget(
      std::uint64_t target_id, const config::SbirsPolicyConfig& policy, float dt_sec,
      const session::SbirsVector3M& satellite_position_eci_m);
  SbirsTrackingUpdateResult CorrectTarget(
      std::uint64_t target_id, const config::SbirsPolicyConfig& policy,
      foundation::SbirsRandomSource* random_source, float azimuth_deg, float elevation_deg,
      double range_m, float relative_angular_rate_deg_per_sec,
      const session::SbirsVector3M& satellite_position_eci_m,
      const session::SbirsVector3M& satellite_velocity_eci_m, int frame_count = 1);
  void MarkMeasurementUnavailable(std::uint64_t target_id);
  SbirsTrackingUpdateResult Update(std::uint64_t target_id,
                                   const config::SbirsPolicyConfig& policy,
                                   foundation::SbirsRandomSource* random_source,
                                   float azimuth_deg, float elevation_deg, double range_m,
                                   float relative_angular_rate_deg_per_sec, float dt_sec,
                                   const session::SbirsVector3M& satellite_position_eci_m,
                                   const session::SbirsVector3M& satellite_velocity_eci_m);
  void ReleaseTarget(std::uint64_t target_id);
  void ResetNisGateCounts();
  void ClearForStandby();
  SbirsTrackingRuntimeState CaptureRuntimeState() const;
  void RestoreRuntimeState(const SbirsTrackingRuntimeState& state);

 private:
  void InitializeImmComponents(const config::SbirsTrackingConfig& tracking);
  tracking::SbirsImmFilter* CreateImmFilter(
      std::uint64_t target_id, const tracking::SbirsGaussianState& initial_state);
  static SbirsTrackingPredictionResult BuildPredictionResult(
      const tracking::SbirsGaussianState& state,
      const session::SbirsVector3M& satellite_position_eci_m);
  static SbirsTrackingPredictionResult BuildAnglePredictionResult(
      const tracking::SbirsAngleCvGaussianState& state);

  tracking::SbirsCvTransitionModel cv_transition_model_{};
  tracking::SbirsAngleMeasurementModel angle_measurement_model_{};
  std::map<std::uint64_t, tracking::SbirsGaussianState> filter_states_{};
  std::map<std::uint64_t, tracking::SbirsAngleCvGaussianState> angle_kf_states_{};
  std::map<std::uint64_t, unsigned int> nis_gate_exceeded_counts_{};
  /**
   * @brief 连续被离群门丢弃的量测拍数（2026-11-02 定位 A）。
   *
   * 角度 CV 滤波遇到"预测漂移 → 创新变大 → 判离群丢弃 → 只靠预测 → 漂移更大"的正反馈时
   * 会永久锁死；现场表现为某颗星的窄场角度逐拍单调涨到度级。连续丢弃达到阈值即以当前
   * 量测重置滤波器（重捕），打断该正反馈。
   */
  std::map<std::uint64_t, unsigned int> consecutive_outlier_counts_{};
  bool imm_initialized_{false};
  std::vector<std::unique_ptr<tracking::SbirsAngleMeasurementModel>> imm_measurement_models_{};
  std::vector<std::unique_ptr<tracking::SbirsEkfPredictor>> imm_predictors_owned_{};
  std::vector<::oneq::common::estimation::IKalmanPredictor<6, 2>*> imm_predictors_{};
  std::vector<std::unique_ptr<tracking::SbirsEkfUpdater>> imm_updaters_owned_{};
  std::vector<::oneq::common::estimation::IKalmanUpdater<6, 2>*> imm_updaters_{};
  // Predictors and updaters are immutable model plumbing shared by all targets;
  // each target owns its mutable IMM model state.
  std::map<std::uint64_t, std::unique_ptr<tracking::SbirsImmFilter>> imm_filters_by_target_{};
  // Restored snapshots wait here until the target next receives an update.
  std::map<std::uint64_t, tracking::SbirsImmSnapshot> imm_snapshots_{};
};

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_TRACKING_COORDINATOR_H_
