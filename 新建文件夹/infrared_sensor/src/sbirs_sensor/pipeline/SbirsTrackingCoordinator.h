/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsTrackingCoordinator.
 * Model: SBIRS infrared sensor.
 * Layer: pipeline.
 * File role: interface declarations and data contracts.
 * Responsibility: wide-field search, pointing, acquisition, and tracking processing.
 * Upstream boundary: cycle scene, cue, sensor state, and resolved physical parameters.
 * Downstream boundary: detections, estimates, diagnostics, and lifecycle observations.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

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
  bool has_angle_rate{false};              /**< kAngleCvKf 后验变化率是否有效 */
  float azimuth_rate_rad_per_s{0.0f};      /**< 方位变化率（rad/s），仅 kAngleCvKf */
  float elevation_rate_rad_per_s{0.0f};    /**< 俯仰变化率（rad/s），仅 kAngleCvKf */
};

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
      int frame_count = 1);
  void MarkMeasurementUnavailable(std::uint64_t target_id);
  SbirsTrackingUpdateResult Update(std::uint64_t target_id,
                                   const config::SbirsPolicyConfig& policy,
                                   foundation::SbirsRandomSource* random_source,
                                   float azimuth_deg, float elevation_deg, double range_m,
                                   float relative_angular_rate_deg_per_sec, float dt_sec,
                                   const session::SbirsVector3M& satellite_position_eci_m);
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
