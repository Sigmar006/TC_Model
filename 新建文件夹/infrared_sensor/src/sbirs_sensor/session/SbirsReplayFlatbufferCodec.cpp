/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsReplayFlatbufferCodec.
 * Model: SBIRS infrared sensor.
 * Layer: session.
 * File role: executable implementation.
 * Responsibility: public cycle contract, validation, adapters, recording, and replay.
 * Upstream boundary: external cycle data and validated session configuration.
 * Downstream boundary: stable output frames, issues, debug views, and replay records.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

#include "sbirs_sensor/session/SbirsReplayFlatbufferCodec.h"

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "common/replay/ReplayFlatbufferCodecSupport.h"
#include "flatbuffers/flatbuffers.h"
#include "sbirs_sensor/session/generated/sbirs_replay_generated.h"
#include "sbirs_sensor/session/generated/sbirs_session_replay_generated.h"

namespace sbirs_sensor {
namespace session {
namespace {

// session_contract 规则 7：以整数存储的 enum 必须逐值校验，未知值原子拒绝。
bool IsKnownSbirsWorkMode(std::int32_t raw) {
  return raw >= static_cast<std::int32_t>(config::SbirsWorkMode::kStandby) &&
         raw <= static_cast<std::int32_t>(config::SbirsWorkMode::kSearchAndStare);
}

bool IsKnownSbirsObservationStage(std::int32_t raw) {
  return raw >= static_cast<std::int32_t>(output::SbirsObservationStage::kWideFieldSearch) &&
         raw <= static_cast<std::int32_t>(output::SbirsObservationStage::kNarrowFieldTrack);
}

bool IsKnownSbirsCaptureFailureReason(std::int32_t raw) {
  return raw >= static_cast<std::int32_t>(attribution::SbirsCaptureFailureReason::kNone) &&
         raw <= static_cast<std::int32_t>(
                    attribution::SbirsCaptureFailureReason::kNfovTrackingGateLost);
}

bool IsKnownSbirsIssueSeverity(std::int32_t raw) {
  return raw >= static_cast<std::int32_t>(SbirsIssueSeverity::kInfo) &&
         raw <= static_cast<std::int32_t>(SbirsIssueSeverity::kError);
}

bool IsKnownSbirsIssuePhase(std::int32_t raw) {
  return raw >= static_cast<std::int32_t>(SbirsIssuePhase::kInputValidation) &&
         raw <= static_cast<std::int32_t>(SbirsIssuePhase::kOutputContract);
}

bool IsKnownSbirsIssueCause(std::int32_t raw) {
  return raw >= static_cast<std::int32_t>(SbirsIssueCause::kNone) &&
         raw <= static_cast<std::int32_t>(SbirsIssueCause::kUnknown);
}

sbirs::replay::Vec3d ToFbVec3(const SbirsVector3M& v) {
  return sbirs::replay::Vec3d(v.x, v.y, v.z);
}

SbirsVector3M FromFbVec3(const sbirs::replay::Vec3d* fb) {
  SbirsVector3M out;
  if (fb != nullptr) {
    out.x = fb->x();
    out.y = fb->y();
    out.z = fb->z();
  }
  return out;
}

flatbuffers::Offset<sbirs::replay::SbirsEnvironmentConfig> EncodeSessionEnvironmentConfig(
    flatbuffers::FlatBufferBuilder& fbb, const config::SbirsEnvironmentConfig& value) {
  return sbirs::replay::CreateSbirsEnvironmentConfig(
      fbb, static_cast<std::int32_t>(value.weather_type),
      static_cast<std::int32_t>(value.sea_state), value.temperature_c,
      value.relative_humidity_percent, value.visibility_km, value.base_atmospheric_transmittance,
      value.humidity_visibility_interaction_weight, value.rain_humidity_interaction_weight);
}

config::SbirsEnvironmentConfig DecodeSessionEnvironmentConfig(
    const sbirs::replay::SbirsEnvironmentConfig* fb) {
  config::SbirsEnvironmentConfig out;
  if (fb != nullptr) {
    out.weather_type = static_cast<config::SbirsWeatherType>(fb->weather_type());
    out.sea_state = static_cast<config::SbirsSeaState>(fb->sea_state());
    out.temperature_c = fb->temperature_c();
    out.relative_humidity_percent = fb->relative_humidity_percent();
    out.visibility_km = fb->visibility_km();
    out.base_atmospheric_transmittance = fb->base_atmospheric_transmittance();
    out.humidity_visibility_interaction_weight = fb->humidity_visibility_interaction_weight();
    out.rain_humidity_interaction_weight = fb->rain_humidity_interaction_weight();
  }
  return out;
}

flatbuffers::Offset<sbirs::replay::SbirsDetectionRecord> EncodeOneDetection(
    flatbuffers::FlatBufferBuilder& fbb, const output::SbirsDetectionRecord& value) {
  return sbirs::replay::CreateSbirsDetectionRecord(
      fbb, value.detection_id, value.azimuth_rad, value.elevation_rad, value.infrared_snr_linear,
      static_cast<std::int32_t>(value.observation_stage), value.detected,
      value.measurement_time_s);
}

bool DecodeOneDetection(const sbirs::replay::SbirsDetectionRecord& fb,
                        output::SbirsDetectionRecord* out) {
  const std::int32_t observation_stage = fb.observation_stage();
  if (!IsKnownSbirsObservationStage(observation_stage)) {
    return false;
  }
  out->detection_id = fb.detection_id();
  out->measurement_time_s = fb.measurement_time_s();
  out->azimuth_rad = fb.azimuth_rad();
  out->elevation_rad = fb.elevation_rad();
  out->infrared_snr_linear = fb.infrared_snr_linear();
  out->observation_stage = static_cast<output::SbirsObservationStage>(observation_stage);
  out->detected = fb.detected();
  return true;
}

flatbuffers::Offset<sbirs::replay::SbirsOutputFrame> EncodeOutputFrameTable(
    flatbuffers::FlatBufferBuilder& fbb, const SbirsOutputFrame& value) {
  std::vector<flatbuffers::Offset<sbirs::replay::SbirsDetectionRecord>> detections;
  detections.reserve(value.detections.size());
  for (const output::SbirsDetectionRecord& detection : value.detections) {
    detections.push_back(EncodeOneDetection(fbb, detection));
  }
  return sbirs::replay::CreateSbirsOutputFrame(fbb, value.cycle_index, value.scan_azimuth_rad,
                                               value.scan_elevation_rad,
                                               fbb.CreateVector(detections));
}

bool DecodeOutputFrameTable(const sbirs::replay::SbirsOutputFrame* fb, SbirsOutputFrame* out) {
  if (fb == nullptr) {
    return true;
  }
  SbirsOutputFrame decoded;
  decoded.cycle_index = fb->cycle_index();
  decoded.scan_azimuth_rad = fb->scan_azimuth_rad();
  decoded.scan_elevation_rad = fb->scan_elevation_rad();
  if (fb->detections() != nullptr) {
    decoded.detections.reserve(fb->detections()->size());
    for (const sbirs::replay::SbirsDetectionRecord* detection : *fb->detections()) {
      output::SbirsDetectionRecord record;
      if (!DecodeOneDetection(*detection, &record)) {
        return false;
      }
      decoded.detections.push_back(record);
    }
  }
  *out = decoded;
  return true;
}

flatbuffers::Offset<sbirs::replay::SbirsHardwareConfig> EncodeHardwareConfig(
    flatbuffers::FlatBufferBuilder& fbb, const config::SbirsHardwareConfig& value) {
  return sbirs::replay::CreateSbirsHardwareConfig(
      fbb, value.wavelength_lower_um, value.wavelength_upper_um, value.optical_aperture_m,
      value.optical_transmission, value.detector_quantum_efficiency,
      value.integration_time_sec, value.noise_equivalent_power_w, value.background_radiance_w_sr_m2,
      value.detector_temperature_k, value.readout_noise_rms_w, value.focal_length_m,
      value.detector_pixel_pitch_m);
}

void DecodeHardwareConfig(const sbirs::replay::SbirsHardwareConfig* fb,
                          config::SbirsHardwareConfig* out) {
  if (fb == nullptr) {
    return;
  }
  out->wavelength_lower_um = fb->wavelength_lower_um();
  out->wavelength_upper_um = fb->wavelength_upper_um();
  out->optical_aperture_m = fb->optical_aperture_m();
  out->optical_transmission = fb->optical_transmission();
  out->detector_quantum_efficiency = fb->detector_quantum_efficiency();
  out->integration_time_sec = fb->integration_time_sec();
  out->noise_equivalent_power_w = fb->noise_equivalent_power_w();
  out->background_radiance_w_sr_m2 = fb->background_radiance_w_sr_m2();
  out->detector_temperature_k = fb->detector_temperature_k();
  out->readout_noise_rms_w = fb->readout_noise_rms_w();
  out->focal_length_m = fb->focal_length_m();
  out->detector_pixel_pitch_m = fb->detector_pixel_pitch_m();
}

flatbuffers::Offset<sbirs::replay::SbirsMissionConfig> EncodeMissionConfig(
    flatbuffers::FlatBufferBuilder& fbb, const config::SbirsMissionConfig& value) {
  return sbirs::replay::CreateSbirsMissionConfig(
      fbb, static_cast<std::int32_t>(value.work_mode), value.wide_field_fov_az_deg,
      value.wide_field_fov_el_deg, value.narrow_field_fov_az_deg, value.narrow_field_fov_el_deg,
      value.scan_start_az_deg, value.scan_span_deg, static_cast<std::int32_t>(value.scan_direction),
      value.scan_center_el_deg, value.scan_el_start_deg, value.scan_el_span_deg,
      value.scan_el_step_deg, value.scan_rate_deg_per_sec, value.min_range_m, value.max_range_m,
      value.frame_rate_hz, value.narrow_cue_latency_s, value.narrow_pointing_settle_error_deg,
      value.narrow_pointing_max_slew_rate_deg_per_sec, value.narrow_pointing_settle_tolerance_deg,
      static_cast<std::int32_t>(value.scan_azimuth_reference), value.initial_scan_phase_deg);
}

bool DecodeMissionConfig(const sbirs::replay::SbirsMissionConfig* fb,
                         config::SbirsMissionConfig* out) {
  if (fb == nullptr || out == nullptr || !IsKnownSbirsWorkMode(fb->work_mode()) ||
      (fb->scan_direction() !=
           static_cast<std::int32_t>(config::SbirsScanDirection::kIncreasingAzimuth) &&
       fb->scan_direction() !=
           static_cast<std::int32_t>(config::SbirsScanDirection::kDecreasingAzimuth)) ||
      (fb->scan_azimuth_reference() !=
           static_cast<std::int32_t>(config::SbirsScanAzimuthReference::kEciAbsolute) &&
       fb->scan_azimuth_reference() !=
           static_cast<std::int32_t>(config::SbirsScanAzimuthReference::kNadirRelative))) {
    return false;
  }
  config::SbirsMissionConfig decoded;
  decoded.work_mode = static_cast<config::SbirsWorkMode>(fb->work_mode());
  decoded.wide_field_fov_az_deg = fb->wide_field_fov_az_deg();
  decoded.wide_field_fov_el_deg = fb->wide_field_fov_el_deg();
  decoded.narrow_field_fov_az_deg = fb->narrow_field_fov_az_deg();
  decoded.narrow_field_fov_el_deg = fb->narrow_field_fov_el_deg();
  decoded.scan_start_az_deg = fb->scan_start_az_deg();
  decoded.scan_span_deg = fb->scan_span_deg();
  decoded.initial_scan_phase_deg = fb->initial_scan_phase_deg();
  decoded.scan_direction = static_cast<config::SbirsScanDirection>(fb->scan_direction());
  decoded.scan_azimuth_reference =
      static_cast<config::SbirsScanAzimuthReference>(fb->scan_azimuth_reference());
  decoded.scan_center_el_deg = fb->scan_center_el_deg();
  decoded.scan_el_start_deg = fb->scan_el_start_deg();
  decoded.scan_el_span_deg = fb->scan_el_span_deg();
  decoded.scan_el_step_deg = fb->scan_el_step_deg();
  decoded.scan_rate_deg_per_sec = fb->scan_rate_deg_per_sec();
  decoded.min_range_m = fb->min_range_m();
  decoded.max_range_m = fb->max_range_m();
  decoded.frame_rate_hz = fb->frame_rate_hz();
  decoded.narrow_cue_latency_s = fb->narrow_cue_latency_s();
  decoded.narrow_pointing_settle_error_deg = fb->narrow_pointing_settle_error_deg();
  decoded.narrow_pointing_max_slew_rate_deg_per_sec =
      fb->narrow_pointing_max_slew_rate_deg_per_sec();
  decoded.narrow_pointing_settle_tolerance_deg = fb->narrow_pointing_settle_tolerance_deg();
  *out = decoded;
  return true;
}

flatbuffers::Offset<sbirs::replay::SbirsOrientationConfig> EncodeOrientationConfig(
    flatbuffers::FlatBufferBuilder& fbb, const config::SbirsOrientationConfig& value) {
  return sbirs::replay::CreateSbirsOrientationConfig(
      fbb, static_cast<float>(value.mount_angles_deg.yaw_deg),
      static_cast<float>(value.mount_angles_deg.pitch_deg),
      static_cast<float>(value.mount_angles_deg.roll_deg),
      value.sensor_scan_limits_deg.az_min_deg, value.sensor_scan_limits_deg.az_max_deg,
      value.sensor_scan_limits_deg.el_min_deg, value.sensor_scan_limits_deg.el_max_deg,
      static_cast<std::int32_t>(value.stabilization_mode),
      static_cast<float>(value.misalignment.bias_deg.yaw_deg),
      static_cast<float>(value.misalignment.bias_deg.pitch_deg),
      static_cast<float>(value.misalignment.bias_deg.roll_deg),
      value.misalignment.random_sigma_deg, value.misalignment.random_seed);
}

bool DecodeOrientationConfig(const sbirs::replay::SbirsOrientationConfig* fb,
                             config::SbirsOrientationConfig* out) {
  if (fb == nullptr) {
    return true;
  }
  if (fb->stabilization_mode() !=
          static_cast<std::int32_t>(config::SbirsStabilizationMode::kBodyStabilized) &&
      fb->stabilization_mode() !=
          static_cast<std::int32_t>(config::SbirsStabilizationMode::kInertialStabilized)) {
    return false;
  }
  config::SbirsOrientationConfig decoded;
  decoded.mount_angles_deg.yaw_deg = fb->mount_yaw_deg();
  decoded.mount_angles_deg.pitch_deg = fb->mount_pitch_deg();
  decoded.mount_angles_deg.roll_deg = fb->mount_roll_deg();
  decoded.sensor_scan_limits_deg.az_min_deg = fb->sensor_scan_limits_az_min_deg();
  decoded.sensor_scan_limits_deg.az_max_deg = fb->sensor_scan_limits_az_max_deg();
  decoded.sensor_scan_limits_deg.el_min_deg = fb->sensor_scan_limits_el_min_deg();
  decoded.sensor_scan_limits_deg.el_max_deg = fb->sensor_scan_limits_el_max_deg();
  decoded.stabilization_mode =
      static_cast<config::SbirsStabilizationMode>(fb->stabilization_mode());
  decoded.misalignment.bias_deg.yaw_deg = fb->misalignment_yaw_deg();
  decoded.misalignment.bias_deg.pitch_deg = fb->misalignment_pitch_deg();
  decoded.misalignment.bias_deg.roll_deg = fb->misalignment_roll_deg();
  decoded.misalignment.random_sigma_deg = fb->misalignment_random_sigma_deg();
  decoded.misalignment.random_seed = fb->misalignment_random_seed();
  *out = decoded;
  return true;
}

flatbuffers::Offset<sbirs::replay::SbirsPolicyConfig> EncodePolicyConfig(
    flatbuffers::FlatBufferBuilder& fbb, const config::SbirsPolicyConfig& value) {
  const flatbuffers::Offset<sbirs::replay::SbirsDetectionPolicyConfig> detection =
      sbirs::replay::CreateSbirsDetectionPolicyConfig(fbb, value.detection.wide_min_snr_linear,
                                                      value.detection.narrow_min_snr_linear);
  const flatbuffers::Offset<sbirs::replay::SbirsErrorModelConfig> error =
      sbirs::replay::CreateSbirsErrorModelConfig(
          fbb, value.error_model.range_fraction_sigma, value.error_model.random_seed,
          value.error_model.orbit_sigma_deg,
          value.error_model.attitude_sigma_deg, value.error_model.fov_sigma_deg,
          value.error_model.detector_bandwidth_hz);
  const flatbuffers::Offset<sbirs::replay::SbirsPointingDisturbanceConfig> disturbance =
      sbirs::replay::CreateSbirsPointingDisturbanceConfig(
          fbb, value.pointing_disturbance.common_attitude_sigma_deg,
          value.pointing_disturbance.common_attitude_correlation_time_s,
          value.pointing_disturbance.channel_pointing_sigma_deg,
          value.pointing_disturbance.channel_pointing_correlation_time_s,
          value.pointing_disturbance.channel_vibration_amplitude_deg,
          value.pointing_disturbance.channel_vibration_frequency_hz,
          value.pointing_disturbance.random_seed);
  const flatbuffers::Offset<sbirs::replay::SbirsSchedulerConfig> scheduler =
      sbirs::replay::CreateSbirsSchedulerConfig(
          fbb, value.scheduler.wide_to_narrow_required_consecutive_hits);
  const auto imm_coeffs = fbb.CreateVector(value.tracking.imm_model_noise_diff_coeffs);
  sbirs::replay::SbirsTrackingConfigBuilder tracking_builder(fbb);
  tracking_builder.add_tracking_mode(static_cast<std::int32_t>(value.tracking.tracking_mode));
  tracking_builder.add_estimated_backend(
      static_cast<std::int32_t>(value.tracking.estimated_backend));
  tracking_builder.add_process_noise_diff_coeff(value.tracking.process_noise_diff_coeff);
  tracking_builder.add_initial_position_std_m(value.tracking.initial_position_std_m);
  tracking_builder.add_initial_velocity_std_m_per_s(value.tracking.initial_velocity_std_m_per_s);
  tracking_builder.add_nis_gate_loss_cycles(value.tracking.nis_gate_loss_cycles);
  tracking_builder.add_nfov_tracking_gate_loss_cycles(
      value.tracking.nfov_tracking_gate_loss_cycles);
  tracking_builder.add_imm_model_noise_diff_coeffs(imm_coeffs);
  const flatbuffers::Offset<sbirs::replay::SbirsTrackingConfig> tracking =
      tracking_builder.Finish();
  return sbirs::replay::CreateSbirsPolicyConfig(fbb, detection, error, disturbance, scheduler,
                                                tracking);
}

bool DecodePolicyConfig(const sbirs::replay::SbirsPolicyConfig* fb,
                        config::SbirsPolicyConfig* out) {
  if (fb == nullptr) {
    return true;
  }
  if (fb->detection() != nullptr) {
    out->detection.wide_min_snr_linear = fb->detection()->wide_min_snr_linear();
    out->detection.narrow_min_snr_linear = fb->detection()->narrow_min_snr_linear();
  }
  if (fb->error_model() != nullptr) {
    out->error_model.range_fraction_sigma = fb->error_model()->range_fraction_sigma();
    out->error_model.random_seed = fb->error_model()->random_seed();
    out->error_model.orbit_sigma_deg = fb->error_model()->orbit_sigma_deg();
    out->error_model.attitude_sigma_deg = fb->error_model()->attitude_sigma_deg();
    out->error_model.fov_sigma_deg = fb->error_model()->fov_sigma_deg();
    out->error_model.detector_bandwidth_hz = fb->error_model()->detector_bandwidth_hz();
  }
  if (fb->pointing_disturbance() != nullptr) {
    out->pointing_disturbance.common_attitude_sigma_deg =
        fb->pointing_disturbance()->common_attitude_sigma_deg();
    out->pointing_disturbance.common_attitude_correlation_time_s =
        fb->pointing_disturbance()->common_attitude_correlation_time_s();
    out->pointing_disturbance.channel_pointing_sigma_deg =
        fb->pointing_disturbance()->channel_pointing_sigma_deg();
    out->pointing_disturbance.channel_pointing_correlation_time_s =
        fb->pointing_disturbance()->channel_pointing_correlation_time_s();
    out->pointing_disturbance.channel_vibration_amplitude_deg =
        fb->pointing_disturbance()->channel_vibration_amplitude_deg();
    out->pointing_disturbance.channel_vibration_frequency_hz =
        fb->pointing_disturbance()->channel_vibration_frequency_hz();
    out->pointing_disturbance.random_seed = fb->pointing_disturbance()->random_seed();
  }
  if (fb->scheduler() != nullptr) {
    out->scheduler.wide_to_narrow_required_consecutive_hits =
        fb->scheduler()->wide_to_narrow_required_consecutive_hits();
  }
  if (fb->tracking() != nullptr) {
    const std::int32_t tracking_mode = fb->tracking()->tracking_mode();
    const std::int32_t estimated_backend = fb->tracking()->estimated_backend();
    if ((tracking_mode != static_cast<std::int32_t>(config::SbirsTrackingMode::kEstimated) &&
         tracking_mode !=
             static_cast<std::int32_t>(config::SbirsTrackingMode::kStrictTruthAssisted) &&
         tracking_mode !=
             static_cast<std::int32_t>(config::SbirsTrackingMode::kSensorLikeTruthAssisted)) ||
        (estimated_backend !=
             static_cast<std::int32_t>(config::SbirsEstimatedTrackingBackend::kEkf) &&
         estimated_backend !=
             static_cast<std::int32_t>(config::SbirsEstimatedTrackingBackend::kImm) &&
         estimated_backend !=
             static_cast<std::int32_t>(config::SbirsEstimatedTrackingBackend::kAngleCvKf))) {
      return false;
    }
    out->tracking.tracking_mode = static_cast<config::SbirsTrackingMode>(tracking_mode);
    out->tracking.estimated_backend =
        static_cast<config::SbirsEstimatedTrackingBackend>(estimated_backend);
    out->tracking.process_noise_diff_coeff = fb->tracking()->process_noise_diff_coeff();
    out->tracking.initial_position_std_m = fb->tracking()->initial_position_std_m();
    out->tracking.initial_velocity_std_m_per_s = fb->tracking()->initial_velocity_std_m_per_s();
    out->tracking.nis_gate_loss_cycles = fb->tracking()->nis_gate_loss_cycles();
    out->tracking.nfov_tracking_gate_loss_cycles =
        fb->tracking()->nfov_tracking_gate_loss_cycles();
    if (fb->tracking()->imm_model_noise_diff_coeffs() != nullptr) {
      const auto* coeffs = fb->tracking()->imm_model_noise_diff_coeffs();
      out->tracking.imm_model_noise_diff_coeffs.assign(coeffs->begin(), coeffs->end());
    }
  }
  return true;
}

}  // namespace

std::string EncodeSbirsCycleInput(const SbirsCycleInput& value) {
  flatbuffers::FlatBufferBuilder fbb(512);
  std::vector<flatbuffers::Offset<sbirs::replay::SbirsSceneTarget>> targets;
  targets.reserve(value.scene.size());
  for (const SbirsSceneTarget& target : value.scene) {
    const flatbuffers::Offset<flatbuffers::String> name = fbb.CreateString(target.target_name);
    const sbirs::replay::Vec3d position = ToFbVec3(target.position_ecef_m);
    const sbirs::replay::Vec3d velocity = ToFbVec3(target.velocity_ecef_m_per_s);
    targets.push_back(sbirs::replay::CreateSbirsSceneTarget(
        fbb, target.target_id, name, &position, target.radiant_intensity_w_per_sr, &velocity,
        target.has_velocity_ecef_m_per_s, target.active));
  }

  const sbirs::replay::Vec3d satellite = ToFbVec3(value.satellite_position_ecef_m);
  const sbirs::replay::Vec3d satellite_velocity = ToFbVec3(value.satellite_velocity_ecef_m_per_s);
  const sbirs::replay::EulerDeg3d satellite_attitude(
      value.satellite_attitude_eci_body_deg.yaw_deg,
      value.satellite_attitude_eci_body_deg.pitch_deg,
      value.satellite_attitude_eci_body_deg.roll_deg);
  fbb.Finish(sbirs::replay::CreateSbirsCycleInput(
                 fbb, value.cycle_index, value.dt_sec, value.utc_julian_day, &satellite,
                 &satellite_velocity, &satellite_attitude, fbb.CreateVector(targets)),
             kSbirsReplayFileIdentifier);
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
}

bool DecodeSbirsCycleInput(const std::string& bytes, SbirsCycleInput* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (out == nullptr ||
      !verifier.VerifyBuffer<sbirs::replay::SbirsCycleInput>(kSbirsReplayFileIdentifier)) {
    return false;
  }
  const sbirs::replay::SbirsCycleInput* fb =
      flatbuffers::GetRoot<sbirs::replay::SbirsCycleInput>(bytes.data());
  out->cycle_index = fb->cycle_index();
  out->dt_sec = fb->dt_sec();
  out->utc_julian_day = fb->utc_julian_day();
  out->satellite_position_ecef_m = FromFbVec3(fb->satellite_position_ecef_m());
  out->satellite_velocity_ecef_m_per_s = FromFbVec3(fb->satellite_velocity_ecef_m_per_s());
  if (fb->satellite_attitude_eci_body_deg() != nullptr) {
    out->satellite_attitude_eci_body_deg.yaw_deg =
        fb->satellite_attitude_eci_body_deg()->yaw_deg();
    out->satellite_attitude_eci_body_deg.pitch_deg =
        fb->satellite_attitude_eci_body_deg()->pitch_deg();
    out->satellite_attitude_eci_body_deg.roll_deg =
        fb->satellite_attitude_eci_body_deg()->roll_deg();
  }
  out->scene.clear();
  if (fb->scene_targets() != nullptr) {
    for (const sbirs::replay::SbirsSceneTarget* target : *fb->scene_targets()) {
      SbirsSceneTarget item;
      item.target_id = target->target_id();
      item.target_name = target->target_name() ? target->target_name()->str() : std::string();
      item.position_ecef_m = FromFbVec3(target->position_ecef_m());
      item.radiant_intensity_w_per_sr = target->radiant_intensity_w_per_sr();
      item.velocity_ecef_m_per_s = FromFbVec3(target->velocity_ecef_m_per_s());
      item.has_velocity_ecef_m_per_s = target->has_velocity_ecef_m_per_s();
      item.active = target->active();
      out->scene.push_back(item);
    }
  }
  return true;
}

std::string EncodeSbirsOutputFrame(const SbirsOutputFrame& value) {
  flatbuffers::FlatBufferBuilder fbb(256);
  fbb.Finish(EncodeOutputFrameTable(fbb, value), kSbirsReplayFileIdentifier);
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
}

bool DecodeSbirsOutputFrame(const std::string& bytes, SbirsOutputFrame* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (out == nullptr ||
      !verifier.VerifyBuffer<sbirs::replay::SbirsOutputFrame>(kSbirsReplayFileIdentifier)) {
    return false;
  }
  SbirsOutputFrame decoded;
  if (!DecodeOutputFrameTable(
          flatbuffers::GetRoot<sbirs::replay::SbirsOutputFrame>(bytes.data()), &decoded)) {
    return false;
  }
  *out = decoded;
  return true;
}

std::string EncodeSbirsCycleResult(const SbirsCycleResult& value) {
  flatbuffers::FlatBufferBuilder fbb(512);
  const flatbuffers::Offset<sbirs::replay::SbirsOutputFrame> frame =
      EncodeOutputFrameTable(fbb, value.output_frame);

  std::vector<flatbuffers::Offset<sbirs::replay::SbirsDetectionAttributionRecord>> attributions;
  attributions.reserve(value.detection_attributions.size());
  for (const attribution::SbirsDetectionAttributionRecord& attribution :
       value.detection_attributions) {
    const auto target_name = fbb.CreateString(attribution.target_name);
    sbirs::replay::SbirsDetectionAttributionRecordBuilder builder(fbb);
    builder.add_detection_id(attribution.detection_id);
    builder.add_target_id(attribution.target_id);
    builder.add_target_name(target_name);
    builder.add_estimated_range_m(attribution.estimated_range_m);
    builder.add_max_detection_range_m(attribution.max_detection_range_m);
    builder.add_tracking_source(static_cast<std::int32_t>(attribution.tracking_source));
    builder.add_capture_failure_reason(
        static_cast<std::int32_t>(attribution.capture_failure_reason));
    builder.add_has_estimation_nis(attribution.has_estimation_nis);
    builder.add_estimation_nis(attribution.estimation_nis);
    builder.add_estimation_nis_gate_exceeded(attribution.estimation_nis_gate_exceeded);
    builder.add_nfov_channel_id(attribution.nfov_channel_id);
    builder.add_has_nfov_tracking_diagnostics(attribution.has_nfov_tracking_diagnostics);
    builder.add_nfov_pointing_error_deg(attribution.nfov_pointing_error_deg);
    builder.add_nfov_geometry_gate_passed(attribution.nfov_geometry_gate_passed);
    builder.add_nfov_snr_gate_passed(attribution.nfov_snr_gate_passed);
    builder.add_nfov_tracking_gate_failure_count(attribution.nfov_tracking_gate_failure_count);
    builder.add_nfov_tracking_coasting(attribution.nfov_tracking_coasting);
    attributions.push_back(builder.Finish());
  }

  std::vector<flatbuffers::Offset<sbirs::replay::SbirsIssue>> fb_issues;
  fb_issues.reserve(value.issues.size());
  for (const SbirsIssue& issue : value.issues) {
    std::int64_t entity_index = -1;
    if (issue.location.entity_index != std::numeric_limits<std::size_t>::max()) {
      entity_index = static_cast<std::int64_t>(issue.location.entity_index);
    }
    fb_issues.push_back(sbirs::replay::CreateSbirsIssue(
        fbb, static_cast<std::int32_t>(issue.severity), static_cast<std::int32_t>(issue.phase),
        fbb.CreateString(issue.code), fbb.CreateString(issue.message),
        static_cast<std::int32_t>(issue.location.kind), entity_index,
        fbb.CreateString(issue.field), static_cast<std::int32_t>(issue.cause)));
  }

  fbb.Finish(sbirs::replay::CreateSbirsCycleResult(
                  fbb, value.input_cycle_index, frame, fbb.CreateVector(attributions),
                  static_cast<std::int32_t>(value.abort_reason),
                  static_cast<std::uint8_t>(value.status), fbb.CreateVector(fb_issues)),
             kSbirsReplayFileIdentifier);
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
}

bool DecodeSbirsCycleResult(const std::string& bytes, SbirsCycleResult* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (out == nullptr ||
      !verifier.VerifyBuffer<sbirs::replay::SbirsCycleResult>(kSbirsReplayFileIdentifier)) {
    return false;
  }
  const sbirs::replay::SbirsCycleResult* fb =
      flatbuffers::GetRoot<sbirs::replay::SbirsCycleResult>(bytes.data());
  const std::int32_t abort_reason = fb->abort_reason();
  if (abort_reason != static_cast<std::int32_t>(SbirsPipelineAbortReason::kNone) &&
      abort_reason !=
          static_cast<std::int32_t>(SbirsPipelineAbortReason::kValidationRejected) &&
      abort_reason !=
          static_cast<std::int32_t>(SbirsPipelineAbortReason::kSensorPoweredOff)) {
    return false;
  }
  SbirsCycleResult decoded;
  decoded.input_cycle_index = fb->input_cycle_index();
  if (!DecodeOutputFrameTable(fb->output_frame(), &decoded.output_frame)) {
    return false;
  }
  if (fb->detection_attributions() != nullptr) {
    for (const sbirs::replay::SbirsDetectionAttributionRecord* attr :
         *fb->detection_attributions()) {
      attribution::SbirsDetectionAttributionRecord record;
      record.detection_id = attr->detection_id();
      record.target_id = attr->target_id();
      record.target_name = attr->target_name() ? attr->target_name()->str() : std::string();
      record.estimated_range_m = attr->estimated_range_m();
      record.max_detection_range_m = attr->max_detection_range_m();
      const std::int32_t tracking_source = attr->tracking_source();
      if (tracking_source !=
              static_cast<std::int32_t>(attribution::SbirsTrackingSource::kNotApplicable) &&
          tracking_source !=
              static_cast<std::int32_t>(attribution::SbirsTrackingSource::kEstimated) &&
          tracking_source != static_cast<std::int32_t>(
                                 attribution::SbirsTrackingSource::kStrictTruthAssisted) &&
          tracking_source != static_cast<std::int32_t>(
                                 attribution::SbirsTrackingSource::kSensorLikeTruthAssisted)) {
        return false;
      }
      record.tracking_source =
          static_cast<attribution::SbirsTrackingSource>(tracking_source);
      const std::int32_t capture_failure_reason = attr->capture_failure_reason();
      if (!IsKnownSbirsCaptureFailureReason(capture_failure_reason)) {
        return false;
      }
      record.capture_failure_reason =
          static_cast<attribution::SbirsCaptureFailureReason>(capture_failure_reason);
      record.has_estimation_nis = attr->has_estimation_nis();
      record.estimation_nis = attr->estimation_nis();
      record.estimation_nis_gate_exceeded = attr->estimation_nis_gate_exceeded();
      record.nfov_channel_id = attr->nfov_channel_id();
      record.has_nfov_tracking_diagnostics = attr->has_nfov_tracking_diagnostics();
      record.nfov_pointing_error_deg = attr->nfov_pointing_error_deg();
      record.nfov_geometry_gate_passed = attr->nfov_geometry_gate_passed();
      record.nfov_snr_gate_passed = attr->nfov_snr_gate_passed();
      record.nfov_tracking_gate_failure_count = attr->nfov_tracking_gate_failure_count();
      record.nfov_tracking_coasting = attr->nfov_tracking_coasting();
      decoded.detection_attributions.push_back(record);
    }
  }
  if (fb->issues() != nullptr) {
    for (const sbirs::replay::SbirsIssue* issue : *fb->issues()) {
      const std::int32_t severity = issue->severity();
      const std::int32_t phase = issue->phase();
      const std::int32_t cause = issue->cause();
      if (!IsKnownSbirsIssueSeverity(severity) || !IsKnownSbirsIssuePhase(phase) ||
          !IsKnownSbirsIssueCause(cause)) {
        return false;
      }
      SbirsIssue item;
      item.severity = static_cast<SbirsIssueSeverity>(severity);
      item.phase = static_cast<SbirsIssuePhase>(phase);
      item.code = issue->code() ? issue->code()->str() : std::string();
      item.message = issue->message() ? issue->message()->str() : std::string();
      item.location.kind = static_cast<oneq::foundation::ValidationLocationKind>(
          issue->location_kind());
      item.location.entity_index = issue->entity_index() < 0
                                       ? std::numeric_limits<std::size_t>::max()
                                       : static_cast<std::size_t>(issue->entity_index());
      item.field = issue->field() ? issue->field()->str() : std::string();
      item.cause = static_cast<SbirsIssueCause>(cause);
      decoded.issues.push_back(item);
    }
  }
  decoded.abort_reason = static_cast<SbirsPipelineAbortReason>(abort_reason);
  const std::int32_t status = fb->status();
  if (status != static_cast<std::int32_t>(SbirsCycleStatus::kCompleted) &&
      status != static_cast<std::int32_t>(SbirsCycleStatus::kPoweredOff) &&
      status != static_cast<std::int32_t>(SbirsCycleStatus::kRejectedInvalidInput) &&
      status != static_cast<std::int32_t>(SbirsCycleStatus::kRejectedExecution)) {
    return false;
  }
  decoded.status = static_cast<SbirsCycleStatus>(status);
  *out = decoded;
  return true;
}

std::string EncodeSbirsSessionConfig(const config::SbirsSessionConfig& value) {
  flatbuffers::FlatBufferBuilder fbb(512);
  fbb.Finish(sbirs::replay::CreateSbirsSessionConfig(
                  fbb, EncodeHardwareConfig(fbb, value.hardware), EncodeMissionConfig(fbb, value.mission),
                  EncodeOrientationConfig(fbb, value.orientation), EncodePolicyConfig(fbb, value.policy),
                  EncodeSessionEnvironmentConfig(fbb, value.environment), value.sensor_enabled),
             kSbirsReplayFileIdentifier);
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
}

bool DecodeSbirsSessionConfig(const std::string& bytes, config::SbirsSessionConfig* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (out == nullptr ||
      !verifier.VerifyBuffer<sbirs::replay::SbirsSessionConfig>(kSbirsReplayFileIdentifier)) {
    return false;
  }
  const sbirs::replay::SbirsSessionConfig* fb =
      flatbuffers::GetRoot<sbirs::replay::SbirsSessionConfig>(bytes.data());
  config::SbirsSessionConfig decoded = *out;
  if (!DecodeMissionConfig(fb->mission(), &decoded.mission)) {
    return false;
  }
  if (!DecodeOrientationConfig(fb->orientation(), &decoded.orientation)) {
    return false;
  }
  DecodeHardwareConfig(fb->hardware(), &decoded.hardware);
  if (!DecodePolicyConfig(fb->policy(), &decoded.policy)) {
    return false;
  }
  decoded.environment = DecodeSessionEnvironmentConfig(fb->environment());
  decoded.sensor_enabled = fb->sensor_enabled();
  *out = decoded;
  return true;
}

std::string EncodeSbirsRuntimeConfigPatch(const config::SbirsRuntimeConfigPatch& value) {
  flatbuffers::FlatBufferBuilder fbb(512);
  fbb.Finish(sbirs::replay::CreateSbirsRuntimeConfigPatch(
                  fbb, value.has_mission, EncodeMissionConfig(fbb, value.mission), value.has_policy,
                  EncodePolicyConfig(fbb, value.policy), value.has_environment,
                  EncodeSessionEnvironmentConfig(fbb, value.environment), value.has_work_mode,
                  static_cast<std::int32_t>(value.work_mode), value.has_scan_rate_deg_per_sec,
                  value.scan_rate_deg_per_sec, value.has_sensor_enabled, value.sensor_enabled),
             kSbirsReplayFileIdentifier);
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
}

bool DecodeSbirsRuntimeConfigPatch(const std::string& bytes, config::SbirsRuntimeConfigPatch* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (out == nullptr ||
      !verifier.VerifyBuffer<sbirs::replay::SbirsRuntimeConfigPatch>(kSbirsReplayFileIdentifier)) {
    return false;
  }
  const sbirs::replay::SbirsRuntimeConfigPatch* fb =
      flatbuffers::GetRoot<sbirs::replay::SbirsRuntimeConfigPatch>(bytes.data());
  config::SbirsRuntimeConfigPatch decoded = *out;
  if (!DecodeMissionConfig(fb->mission(), &decoded.mission)) {
    return false;
  }
  decoded.has_mission = fb->has_mission();
  decoded.has_policy = fb->has_policy();
  decoded.has_environment = fb->has_environment();
  decoded.has_work_mode = fb->has_work_mode();
  {
    const std::int32_t raw_work_mode = fb->work_mode();
    if (!IsKnownSbirsWorkMode(raw_work_mode)) {
      return false;
    }
    decoded.work_mode = static_cast<config::SbirsWorkMode>(raw_work_mode);
  }
  decoded.has_scan_rate_deg_per_sec = fb->has_scan_rate_deg_per_sec();
  decoded.scan_rate_deg_per_sec = fb->scan_rate_deg_per_sec();
  decoded.has_sensor_enabled = fb->has_sensor_enabled();
  decoded.sensor_enabled = fb->sensor_enabled();
  if (!DecodePolicyConfig(fb->policy(), &decoded.policy)) {
    return false;
  }
  decoded.environment = DecodeSessionEnvironmentConfig(fb->environment());
  *out = decoded;
  return true;
}

std::string EncodeSbirsFailureMarker(const oneq::replay::ReplayTraceFailure& failure) {
  flatbuffers::FlatBufferBuilder fbb(256);
  fbb.Finish(sbirs::replay::CreateFailureMarkerDirect(
      fbb, failure.error_code.c_str(), failure.message.c_str(), failure.location.c_str(),
      failure.has_cycle_index, failure.cycle_index, failure.has_sim_time_sec, failure.sim_time_sec,
      failure.diagnostics_payload.c_str(), false, 0U));
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
}

bool DecodeSbirsFailureMarker(const std::string& bytes, oneq::replay::ReplayTraceFailure* failure,
                              std::string* error) {
  return oneq::common::replay::DecodeFailureMarkerPayload<sbirs::replay::FailureMarker>(
      bytes, failure, error);
}

}  // namespace session
}  // namespace sbirs_sensor
