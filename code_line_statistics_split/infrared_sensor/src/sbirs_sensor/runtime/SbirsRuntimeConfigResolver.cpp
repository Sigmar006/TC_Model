#include "sbirs_sensor/runtime/SbirsRuntimeConfigResolver.h"

#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"

namespace sbirs_sensor {
namespace runtime {
namespace {

bool MeasurementStatisticsChanged(const config::SbirsSessionConfig& previous,
                                  const config::SbirsSessionConfig& next) {
  const config::SbirsErrorModelConfig& left = previous.policy.error_model;
  const config::SbirsErrorModelConfig& right = next.policy.error_model;
  return left.range_fraction_sigma != right.range_fraction_sigma ||
         left.orbit_sigma_deg != right.orbit_sigma_deg ||
         left.attitude_sigma_deg != right.attitude_sigma_deg ||
         left.fov_sigma_deg != right.fov_sigma_deg ||
         left.detector_bandwidth_hz != right.detector_bandwidth_hz;
}

bool PointingDisturbanceChanged(const config::SbirsSessionConfig& previous,
                                const config::SbirsSessionConfig& next) {
  const config::SbirsPointingDisturbanceConfig& left = previous.policy.pointing_disturbance;
  const config::SbirsPointingDisturbanceConfig& right = next.policy.pointing_disturbance;
  return left.common_attitude_sigma_deg != right.common_attitude_sigma_deg ||
         left.common_attitude_correlation_time_s != right.common_attitude_correlation_time_s ||
         left.channel_pointing_sigma_deg != right.channel_pointing_sigma_deg ||
         left.channel_pointing_correlation_time_s != right.channel_pointing_correlation_time_s ||
         left.channel_vibration_amplitude_deg != right.channel_vibration_amplitude_deg ||
         left.channel_vibration_frequency_hz != right.channel_vibration_frequency_hz;
}

SbirsRuntimeConfigImpact ClassifyImpact(const config::SbirsSessionConfig& previous,
                                        const config::SbirsSessionConfig& next) {
  SbirsRuntimeConfigImpact impact;
  impact.scan_sector_changed =
      previous.mission.scan_start_az_deg != next.mission.scan_start_az_deg ||
      previous.mission.scan_span_deg != next.mission.scan_span_deg ||
      previous.mission.scan_direction != next.mission.scan_direction ||
      // 方位基准切换（eci_absolute ↔ nadir_relative）同样改变有效起点，走扇区重锚。
      previous.mission.scan_azimuth_reference != next.mission.scan_azimuth_reference ||
      previous.mission.scan_el_start_deg != next.mission.scan_el_start_deg ||
      previous.mission.scan_el_span_deg != next.mission.scan_el_span_deg ||
      previous.mission.scan_el_step_deg != next.mission.scan_el_step_deg;
  impact.reset_measurement_random_stream =
      previous.policy.error_model.random_seed != next.policy.error_model.random_seed;
  impact.reset_nis_gate_counts =
      MeasurementStatisticsChanged(previous, next) ||
      previous.policy.tracking.process_noise_diff_coeff !=
          next.policy.tracking.process_noise_diff_coeff ||
      previous.policy.tracking.nis_gate_loss_cycles != next.policy.tracking.nis_gate_loss_cycles;
  impact.reset_nfov_gate_failure_counts =
      previous.policy.detection.narrow_min_snr_linear !=
          next.policy.detection.narrow_min_snr_linear ||
      previous.mission.narrow_field_fov_az_deg != next.mission.narrow_field_fov_az_deg ||
      previous.mission.narrow_field_fov_el_deg != next.mission.narrow_field_fov_el_deg ||
      PointingDisturbanceChanged(previous, next) ||
      previous.policy.tracking.nfov_tracking_gate_loss_cycles !=
          next.policy.tracking.nfov_tracking_gate_loss_cycles;
  impact.restart_pointing_disturbance = previous.policy.pointing_disturbance.random_seed !=
                                        next.policy.pointing_disturbance.random_seed;
  impact.previous_tracking_mode = previous.policy.tracking.tracking_mode;
  impact.next_tracking_mode = next.policy.tracking.tracking_mode;
  const bool previous_estimated =
      impact.previous_tracking_mode == config::SbirsTrackingMode::kEstimated;
  const bool next_estimated =
      impact.next_tracking_mode == config::SbirsTrackingMode::kEstimated;
  impact.release_incompatible_tracks = previous_estimated != next_estimated;
  impact.retag_truth_tracks =
      !previous_estimated && !next_estimated &&
      impact.previous_tracking_mode != impact.next_tracking_mode;
  impact.release_estimated_tracks =
      previous.policy.tracking.estimated_backend != next.policy.tracking.estimated_backend ||
      previous.policy.tracking.imm_model_noise_diff_coeffs !=
          next.policy.tracking.imm_model_noise_diff_coeffs;

  const bool previous_inactive =
      !previous.sensor_enabled || previous.mission.work_mode == config::SbirsWorkMode::kStandby;
  const bool next_inactive =
      !next.sensor_enabled || next.mission.work_mode == config::SbirsWorkMode::kStandby;
  impact.clear_for_inactive = !previous_inactive && next_inactive;
  impact.clear_for_wide_search =
      !next_inactive && next.mission.work_mode == config::SbirsWorkMode::kWideSearch &&
      (previous_inactive || previous.mission.work_mode != config::SbirsWorkMode::kWideSearch);
  return impact;
}

}  // namespace

SbirsRuntimeConfigResolution ResolveSbirsRuntimeConfigPatch(
    const config::SbirsSessionConfig& current_config,
    const config::SbirsRuntimeConfigPatch& patch) {
  SbirsRuntimeConfigResolution resolution;
  resolution.resolved_config = current_config;
  resolution.has_requested_update = patch.has_mission || patch.has_policy ||
                                    patch.has_environment || patch.has_work_mode ||
                                    patch.has_scan_rate_deg_per_sec || patch.has_sensor_enabled;
  if (!resolution.has_requested_update) {
    return resolution;
  }
  if (patch.has_mission) {
    // 电源单源：mission 域无电源字段（COMMON-OQ-4，见 contract.md §电源状态单源契约）。
    resolution.resolved_config.mission = patch.mission;
  }
  if (patch.has_policy) {
    resolution.resolved_config.policy = patch.policy;
  }
  if (patch.has_environment) {
    resolution.resolved_config.environment = patch.environment;
  }
  if (patch.has_work_mode) {
    resolution.resolved_config.mission.work_mode = patch.work_mode;
  }
  if (patch.has_scan_rate_deg_per_sec) {
    resolution.resolved_config.mission.scan_rate_deg_per_sec = patch.scan_rate_deg_per_sec;
  }
  if (patch.has_sensor_enabled) {
    resolution.resolved_config.sensor_enabled = patch.sensor_enabled;
  }
  resolution.is_valid = config::ValidateSbirsSessionConfig(resolution.resolved_config).empty();
  if (resolution.is_valid) {
    resolution.impact = ClassifyImpact(current_config, resolution.resolved_config);
  }
  return resolution;
}

}  // namespace runtime
}  // namespace sbirs_sensor
