#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>

#include "1q/replay/ReplayTrace.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"
#include "flatbuffers/flatbuffers.h"
#include "sbirs_sensor/session/SbirsReplayFlatbufferCodec.h"
#include "sbirs_sensor/session/generated/sbirs_replay_generated.h"
#include "sbirs_sensor/session/generated/sbirs_session_replay_generated.h"

namespace sbirs_sensor {
namespace session {
namespace tests {
namespace {

using config::SbirsEnvironmentConfig;
using config::SbirsHardwareConfig;
using config::SbirsMissionConfig;
using config::SbirsPolicyConfig;
using config::SbirsRuntimeConfigPatch;
using config::SbirsScanDirection;
using config::SbirsSeaState;
using config::SbirsSessionConfig;
using config::SbirsWeatherType;
using config::SbirsWorkMode;
using output::SbirsDetectionRecord;
using output::SbirsObservationStage;

SbirsVector3M Vector(double x, double y, double z) {
  SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

std::string EncodeCycleResultWithRawAbortReason(std::int32_t abort_reason) {
  flatbuffers::FlatBufferBuilder builder(128U);
  builder.Finish(sbirs::replay::CreateSbirsCycleResult(
                    builder, 99U, 0, 0, abort_reason, 0, 0),
                  kSbirsReplayFileIdentifier);
  return std::string(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                     builder.GetSize());
}

std::string EncodeCycleResultWithRawStatus(std::int32_t status) {
  flatbuffers::FlatBufferBuilder builder(128U);
  builder.Finish(sbirs::replay::CreateSbirsCycleResult(
                    builder, 99U, 0, 0, 0, status, 0),
                  kSbirsReplayFileIdentifier);
  return std::string(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                     builder.GetSize());
}

std::string EncodeSessionConfigWithRawScanDirection(std::int32_t scan_direction) {
  flatbuffers::FlatBufferBuilder builder(128U);
  const auto mission = sbirs::replay::CreateSbirsMissionConfig(
      builder, 0, 20.0f, 20.0f, 2.0f, 2.0f, -60.0f, 120.0f, scan_direction);
  builder.Finish(sbirs::replay::CreateSbirsSessionConfig(builder, 0, mission, 0, 0),
                  kSbirsReplayFileIdentifier);
  return std::string(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
}

// nadir 方位基准（2026-08-31）：raw 非法基准值走与 scan_direction 同款原子拒绝路径。
// schema 中 scan_azimuth_reference 追加在表尾，位置参数须填满前序字段后落到末位。
std::string EncodeSessionConfigWithRawAzimuthReference(std::int32_t azimuth_reference) {
  flatbuffers::FlatBufferBuilder builder(128U);
  const auto mission = sbirs::replay::CreateSbirsMissionConfig(
      builder, 0, 20.0f, 20.0f, 2.0f, 2.0f, -60.0f, 120.0f, 0, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 30.0f, 0.01f, azimuth_reference);
  builder.Finish(sbirs::replay::CreateSbirsSessionConfig(builder, 0, mission, 0, 0),
                  kSbirsReplayFileIdentifier);
  return std::string(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
}

std::string EncodeSessionConfigWithRawTrackingEnums(std::int32_t tracking_mode,
                                                    std::int32_t estimated_backend) {
  flatbuffers::FlatBufferBuilder builder(128U);
  const auto tracking = sbirs::replay::CreateSbirsTrackingConfig(
      builder, tracking_mode, estimated_backend);
  const auto policy = sbirs::replay::CreateSbirsPolicyConfig(builder, 0, 0, 0, 0, tracking);
  builder.Finish(sbirs::replay::CreateSbirsSessionConfig(builder, 0, 0, 0, policy),
                 kSbirsReplayFileIdentifier);
  return std::string(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
}

std::string EncodeCycleResultWithRawTrackingSource(std::int32_t tracking_source) {
  flatbuffers::FlatBufferBuilder builder(128U);
  // 位置参数须与 schema 字段序一致（max_detection_range_m 在 estimated_range_m 之后）。
  const auto attribution = sbirs::replay::CreateSbirsDetectionAttributionRecord(
      builder, 1U, 2U, 0, 3.0f, 4.0f, tracking_source);
  std::vector<flatbuffers::Offset<sbirs::replay::SbirsDetectionAttributionRecord>> records;
  records.push_back(attribution);
  builder.Finish(sbirs::replay::CreateSbirsCycleResult(
                    builder, 4U, 0, builder.CreateVector(records)),
                  kSbirsReplayFileIdentifier);
  return std::string(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
}

}  // namespace

// --- CycleInput ---

TEST(SbirsReplayCodecRoundtripTest, CycleInputPreservesAllFields) {
  SbirsSceneTarget target;
  target.target_id = 9U;
  target.target_name = "boost";
  target.position_ecef_m = Vector(8000000.0, 2.0, 3.0);
  target.radiant_intensity_w_per_sr = 12345.678;
  target.velocity_ecef_m_per_s = Vector(1000.0, -500.0, 250.0);
  target.has_velocity_ecef_m_per_s = true;
  target.active = false;

  sbirs_sensor::session::SbirsEulerAnglesDeg attitude;
  attitude.yaw_deg = 12.0;
  attitude.pitch_deg = -3.0;
  attitude.roll_deg = 4.0;
  const SbirsCycleInput input = SbirsCycleInputBuilder()
                                    .WithCycleIndex(3U)
                                    .WithDeltaTimeSec(0.25f)
                                    .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                                    .WithSatellitePosition(Vector(7000000.0, 4.0, 5.0))
                                    .WithSatelliteVelocity(Vector(7500.0, -300.0, 120.0))
                                    .WithSatelliteAttitude(attitude)
                                    .AddTarget(target)
                                    .Build();

  const std::string bytes = EncodeSbirsCycleInput(input);
  ASSERT_FALSE(bytes.empty());
  SbirsCycleInput decoded;
  ASSERT_TRUE(DecodeSbirsCycleInput(bytes, &decoded));

  EXPECT_EQ(decoded.cycle_index, 3U);
  EXPECT_FLOAT_EQ(decoded.dt_sec, 0.25f);
  EXPECT_DOUBLE_EQ(decoded.utc_julian_day, 2451544.2230698913);  // ECI 输出参考系（UTC 儒略日）
  EXPECT_DOUBLE_EQ(decoded.satellite_position_ecef_m.x, 7000000.0);
  EXPECT_DOUBLE_EQ(decoded.satellite_velocity_ecef_m_per_s.x, 7500.0);
  EXPECT_DOUBLE_EQ(decoded.satellite_velocity_ecef_m_per_s.y, -300.0);
  EXPECT_DOUBLE_EQ(decoded.satellite_velocity_ecef_m_per_s.z, 120.0);
  EXPECT_DOUBLE_EQ(decoded.satellite_attitude_eci_body_deg.yaw_deg, 12.0);
  EXPECT_DOUBLE_EQ(decoded.satellite_attitude_eci_body_deg.pitch_deg, -3.0);
  EXPECT_DOUBLE_EQ(decoded.satellite_attitude_eci_body_deg.roll_deg, 4.0);
  ASSERT_EQ(decoded.scene.size(), 1U);
  EXPECT_EQ(decoded.scene[0].target_id, 9U);
  EXPECT_EQ(decoded.scene[0].target_name, "boost");
  EXPECT_DOUBLE_EQ(decoded.scene[0].position_ecef_m.y, 2.0);
  EXPECT_DOUBLE_EQ(decoded.scene[0].radiant_intensity_w_per_sr, 12345.678);
  EXPECT_DOUBLE_EQ(decoded.scene[0].velocity_ecef_m_per_s.x, 1000.0);
  EXPECT_DOUBLE_EQ(decoded.scene[0].velocity_ecef_m_per_s.y, -500.0);
  EXPECT_DOUBLE_EQ(decoded.scene[0].velocity_ecef_m_per_s.z, 250.0);
  EXPECT_TRUE(decoded.scene[0].has_velocity_ecef_m_per_s);
  EXPECT_FALSE(decoded.scene[0].active);
}

TEST(SbirsReplayCodecRoundtripTest, CycleInputDecodesEmptyScene) {
  const SbirsCycleInput input = SbirsCycleInputBuilder().WithCycleIndex(1U).Build();
  const std::string bytes = EncodeSbirsCycleInput(input);
  SbirsCycleInput decoded;
  ASSERT_TRUE(DecodeSbirsCycleInput(bytes, &decoded));
  EXPECT_EQ(decoded.cycle_index, 1U);
  EXPECT_TRUE(decoded.scene.empty());
}

// --- OutputFrame ---

TEST(SbirsReplayCodecRoundtripTest, OutputFramePreservesAllFields) {
  SbirsOutputFrame frame;
  frame.cycle_index = 7U;
  frame.scan_azimuth_rad = -15.5f;
  frame.scan_elevation_rad = 0.25f;  // 阶段 4：2-D 栅格当前行中心俯仰（非默认防漏读）

  SbirsDetectionRecord detection;
  detection.detection_id = 33U;
  detection.measurement_time_s = 12.345;
  detection.azimuth_rad = 1.5f;
  detection.elevation_rad = -2.5f;
  detection.infrared_snr_linear = 9.0f;
  detection.observation_stage = SbirsObservationStage::kNarrowFieldTrack;
  detection.detected = true;
  frame.detections.push_back(detection);

  const std::string bytes = EncodeSbirsOutputFrame(frame);
  ASSERT_FALSE(bytes.empty());
  SbirsOutputFrame decoded;
  ASSERT_TRUE(DecodeSbirsOutputFrame(bytes, &decoded));

  EXPECT_EQ(decoded.cycle_index, 7U);
  EXPECT_FLOAT_EQ(decoded.scan_azimuth_rad, -15.5f);
  EXPECT_FLOAT_EQ(decoded.scan_elevation_rad, 0.25f);
  ASSERT_EQ(decoded.detections.size(), 1U);
  EXPECT_EQ(decoded.detections[0].detection_id, 33U);
  EXPECT_DOUBLE_EQ(decoded.detections[0].measurement_time_s, 12.345);
  EXPECT_FLOAT_EQ(decoded.detections[0].azimuth_rad, 1.5f);
  EXPECT_FLOAT_EQ(decoded.detections[0].elevation_rad, -2.5f);
  EXPECT_FLOAT_EQ(decoded.detections[0].infrared_snr_linear, 9.0f);
  EXPECT_EQ(decoded.detections[0].observation_stage, SbirsObservationStage::kNarrowFieldTrack);
  EXPECT_TRUE(decoded.detections[0].detected);
}

// --- CycleResult ---

TEST(SbirsReplayCodecRoundtripTest, CycleResultPreservesOutputAndAttributionFields) {
  SbirsCycleResult result;
  result.input_cycle_index = 5U;
  result.output_frame.cycle_index = 5U;
  result.output_frame.scan_azimuth_rad = 12.0f;
  result.status = SbirsCycleStatus::kCompleted;
  result.abort_reason = SbirsPipelineAbortReason::kValidationRejected;

  SbirsDetectionRecord detection;
  detection.detection_id = 33U;
  detection.observation_stage = SbirsObservationStage::kNarrowFieldTrack;
  detection.detected = true;
  result.output_frame.detections.push_back(detection);

  attribution::SbirsDetectionAttributionRecord attribution;
  attribution.detection_id = 33U;
  attribution.target_id = 44U;
  attribution.target_name = "truth";
  attribution.estimated_range_m = 1234.0f;
  attribution.max_detection_range_m = 2.5e6f;
  attribution.tracking_source =
      attribution::SbirsTrackingSource::kSensorLikeTruthAssisted;
  attribution.capture_failure_reason =
      attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed;
  attribution.has_estimation_nis = true;
  attribution.estimation_nis = 6.25f;
  attribution.estimation_nis_gate_exceeded = true;
  attribution.nfov_channel_id = 1;
  attribution.has_nfov_tracking_diagnostics = true;
  attribution.nfov_pointing_error_deg = 0.75f;
  attribution.nfov_geometry_gate_passed = false;
  attribution.nfov_snr_gate_passed = true;
  attribution.nfov_tracking_gate_failure_count = 1U;
  attribution.nfov_tracking_coasting = true;
  result.detection_attributions.push_back(attribution);

  // 带 entity_index 的 scene 级校验问题
  SbirsIssue scene_issue;
  scene_issue.severity = SbirsIssueSeverity::kWarning;
  scene_issue.phase = SbirsIssuePhase::kInputValidation;
  scene_issue.code = "sbirs.validation.invalid_target_physical";
  scene_issue.location.kind = oneq::foundation::ValidationLocationKind::kSceneEntity;
  scene_issue.location.entity_index = 1U;
  scene_issue.field = "scene";
  scene_issue.message = "warn";
  scene_issue.cause = SbirsIssueCause::kBothAxesOutside;
  result.issues.push_back(scene_issue);

  // kGlobal location（entity_index 哨兵值 size_t::max ↔ int64 -1）
  SbirsIssue global_issue;
  global_issue.severity = SbirsIssueSeverity::kError;
  global_issue.phase = SbirsIssuePhase::kInputValidation;
  global_issue.code = "sbirs.validation.invalid_cycle_delta_time";
  global_issue.location.kind = oneq::foundation::ValidationLocationKind::kGlobal;
  global_issue.location.entity_index = std::numeric_limits<std::size_t>::max();
  global_issue.field = "dt_sec";
  global_issue.message = "global";
  global_issue.cause = SbirsIssueCause::kDistanceLimited;
  result.issues.push_back(global_issue);

  const std::string bytes = EncodeSbirsCycleResult(result);
  SbirsCycleResult decoded;
  ASSERT_TRUE(DecodeSbirsCycleResult(bytes, &decoded));

  EXPECT_EQ(decoded.input_cycle_index, 5U);
  EXPECT_EQ(decoded.status, SbirsCycleStatus::kCompleted);
  EXPECT_EQ(decoded.abort_reason, SbirsPipelineAbortReason::kValidationRejected);
  ASSERT_EQ(decoded.output_frame.detections.size(), 1U);
  EXPECT_EQ(decoded.output_frame.detections[0].observation_stage,
            SbirsObservationStage::kNarrowFieldTrack);
  ASSERT_EQ(decoded.detection_attributions.size(), 1U);
  EXPECT_FLOAT_EQ(decoded.detection_attributions[0].estimated_range_m, 1234.0f);
  EXPECT_FLOAT_EQ(decoded.detection_attributions[0].max_detection_range_m, 2.5e6f);
  EXPECT_EQ(decoded.detection_attributions[0].tracking_source,
            attribution::SbirsTrackingSource::kSensorLikeTruthAssisted);
  EXPECT_EQ(decoded.detection_attributions[0].capture_failure_reason,
            attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed);
  EXPECT_TRUE(decoded.detection_attributions[0].has_estimation_nis);
  EXPECT_FLOAT_EQ(decoded.detection_attributions[0].estimation_nis, 6.25f);
  EXPECT_TRUE(decoded.detection_attributions[0].estimation_nis_gate_exceeded);
  EXPECT_EQ(decoded.detection_attributions[0].nfov_channel_id, 1);
  EXPECT_TRUE(decoded.detection_attributions[0].has_nfov_tracking_diagnostics);
  EXPECT_FLOAT_EQ(decoded.detection_attributions[0].nfov_pointing_error_deg, 0.75f);
  EXPECT_FALSE(decoded.detection_attributions[0].nfov_geometry_gate_passed);
  EXPECT_TRUE(decoded.detection_attributions[0].nfov_snr_gate_passed);
  EXPECT_EQ(decoded.detection_attributions[0].nfov_tracking_gate_failure_count, 1U);
  EXPECT_TRUE(decoded.detection_attributions[0].nfov_tracking_coasting);
  ASSERT_EQ(decoded.issues.size(), 2U);
  EXPECT_EQ(decoded.issues[0].phase, SbirsIssuePhase::kInputValidation);
  EXPECT_EQ(decoded.issues[0].code, "sbirs.validation.invalid_target_physical");
  EXPECT_EQ(decoded.issues[0].field, "scene");
  EXPECT_EQ(decoded.issues[0].location.entity_index, 1U);
  EXPECT_EQ(decoded.issues[0].cause, SbirsIssueCause::kBothAxesOutside);
  // 哨兵值经 size_t::max → int64(-1) → size_t::max 的往返
  EXPECT_EQ(decoded.issues[1].location.kind, oneq::foundation::ValidationLocationKind::kGlobal);
  EXPECT_EQ(decoded.issues[1].location.entity_index,
            std::numeric_limits<std::size_t>::max());
  EXPECT_EQ(decoded.issues[1].cause, SbirsIssueCause::kDistanceLimited);
}

TEST(SbirsReplayCodecRoundtripTest, CycleResultPreservesPoweredOffAbortReason) {
  SbirsCycleResult result;
  result.input_cycle_index = 7U;
  result.output_frame.cycle_index = 7U;
  result.output_frame.scan_azimuth_rad = 3.0f;
  result.abort_reason = SbirsPipelineAbortReason::kSensorPoweredOff;
  result.status = SbirsCycleStatus::kPoweredOff;

  const std::string bytes = EncodeSbirsCycleResult(result);
  SbirsCycleResult decoded;
  ASSERT_TRUE(DecodeSbirsCycleResult(bytes, &decoded));

  EXPECT_EQ(decoded.input_cycle_index, 7U);
  EXPECT_EQ(decoded.abort_reason, SbirsPipelineAbortReason::kSensorPoweredOff);
  EXPECT_EQ(decoded.status, SbirsCycleStatus::kPoweredOff);
}

// --- SessionConfig ---

TEST(SbirsReplayCodecRoundtripTest, SessionConfigPreservesAllDomains) {
  SbirsSessionConfig config;
  config.hardware.wavelength_lower_um = 2.0f;
  config.hardware.wavelength_upper_um = 5.0f;
  config.hardware.optical_aperture_m = 0.9f;
  config.hardware.optical_transmission = 0.75f;
  config.hardware.detector_quantum_efficiency = 0.65f;
  config.hardware.integration_time_sec = 0.04f;
  config.hardware.noise_equivalent_power_w = 2.0e-12f;
  config.hardware.focal_length_m = 1.8f;              // 焦平面几何：非默认值防 decode 漏读
  config.hardware.detector_pixel_pitch_m = 25.0e-6f;  // 焦平面几何：非默认值防 decode 漏读
  config.mission.work_mode = SbirsWorkMode::kWideSearch;
  config.sensor_enabled = false;  // 非默认值防 decode 漏读（COMMON-OQ-4 字段提升）
  config.mission.scan_start_az_deg = 170.0f;
  config.mission.scan_span_deg = 45.0f;
  config.mission.scan_direction = SbirsScanDirection::kDecreasingAzimuth;
  // nadir 方位基准（2026-08-31）：非默认值防 decode 漏读。
  config.mission.scan_azimuth_reference = config::SbirsScanAzimuthReference::kNadirRelative;
  config.mission.scan_center_el_deg = 6.0f;
  config.mission.scan_el_start_deg = -10.0f;  // 阶段 4 俯仰栅格：非默认值防 decode 漏读
  config.mission.scan_el_span_deg = 20.0f;
  config.mission.scan_el_step_deg = 5.0f;
  config.mission.scan_rate_deg_per_sec = 3.0f;
  config.mission.initial_scan_phase_deg = 15.0f;
  config.mission.narrow_cue_latency_s = 0.05f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 17.5f;
  config.mission.narrow_pointing_settle_tolerance_deg = 0.025f;
  config.policy.detection.wide_min_snr_linear = 3.5f;
  config.policy.detection.narrow_min_snr_linear = 7.0f;
  config.policy.error_model.orbit_sigma_deg = 0.02f;
  config.policy.error_model.attitude_sigma_deg = 0.04f;
  config.policy.error_model.fov_sigma_deg = 0.06f;
  config.policy.error_model.range_fraction_sigma = 0.002f;
  config.policy.error_model.random_seed = 42U;
  config.policy.pointing_disturbance.common_attitude_sigma_deg = 0.02f;
  config.policy.pointing_disturbance.common_attitude_correlation_time_s = 2.0f;
  config.policy.pointing_disturbance.channel_pointing_sigma_deg = 0.03f;
  config.policy.pointing_disturbance.channel_pointing_correlation_time_s = 3.0f;
  config.policy.pointing_disturbance.channel_vibration_amplitude_deg = 0.04f;
  config.policy.pointing_disturbance.channel_vibration_frequency_hz = 4.0f;
  config.policy.pointing_disturbance.random_seed = 43U;
  config.policy.scheduler.wide_to_narrow_required_consecutive_hits = 2;  // 宽窄切换前置条件
  config.policy.tracking.tracking_mode = config::SbirsTrackingMode::kStrictTruthAssisted;
  config.policy.tracking.estimated_backend = config::SbirsEstimatedTrackingBackend::kEkf;
  config.policy.tracking.process_noise_diff_coeff = 2.5f;
  config.policy.tracking.initial_position_std_m = 1500.0f;
  config.policy.tracking.initial_velocity_std_m_per_s = 80.0f;
  config.policy.tracking.nis_gate_loss_cycles = 2U;
  config.policy.tracking.nfov_tracking_gate_loss_cycles = 4U;
  config.environment.weather_type = SbirsWeatherType::kRain;
  config.environment.sea_state = SbirsSeaState::kMedium;
  config.environment.temperature_c = 25.0f;
  config.environment.base_atmospheric_transmittance = 0.7f;
  // 阶段 2 安装指向域：非默认值防 decode 漏读。
  config.orientation.mount_angles_deg.yaw_deg = 11.0;
  config.orientation.mount_angles_deg.pitch_deg = -7.0;
  config.orientation.mount_angles_deg.roll_deg = 3.0;
  config.orientation.sensor_scan_limits_deg.az_min_deg = -45.0f;
  config.orientation.sensor_scan_limits_deg.az_max_deg = 60.0f;
  config.orientation.sensor_scan_limits_deg.el_min_deg = -20.0f;
  config.orientation.sensor_scan_limits_deg.el_max_deg = 25.0f;
  config.orientation.stabilization_mode = config::SbirsStabilizationMode::kInertialStabilized;
  // 阶段 3 安装失准域：非默认值防 decode 漏读。
  config.orientation.misalignment.bias_deg.yaw_deg = 4.0;
  config.orientation.misalignment.bias_deg.pitch_deg = -2.0;
  config.orientation.misalignment.bias_deg.roll_deg = 1.0;
  config.orientation.misalignment.random_sigma_deg = 0.5f;
  config.orientation.misalignment.random_seed = 44U;

  SbirsSessionConfig decoded;
  ASSERT_TRUE(DecodeSbirsSessionConfig(EncodeSbirsSessionConfig(config), &decoded));
  EXPECT_FLOAT_EQ(decoded.hardware.optical_aperture_m, 0.9f);
  EXPECT_FLOAT_EQ(decoded.hardware.noise_equivalent_power_w, 2.0e-12f);
  EXPECT_EQ(decoded.mission.work_mode, SbirsWorkMode::kWideSearch);
  EXPECT_FALSE(decoded.sensor_enabled);
  EXPECT_FLOAT_EQ(decoded.mission.scan_start_az_deg, 170.0f);
  EXPECT_FLOAT_EQ(decoded.mission.scan_span_deg, 45.0f);
  EXPECT_EQ(decoded.mission.scan_direction, SbirsScanDirection::kDecreasingAzimuth);
  EXPECT_EQ(decoded.mission.scan_azimuth_reference,
            config::SbirsScanAzimuthReference::kNadirRelative);
  EXPECT_FLOAT_EQ(decoded.mission.scan_center_el_deg, 6.0f);
  EXPECT_FLOAT_EQ(decoded.mission.scan_el_start_deg, -10.0f);
  EXPECT_FLOAT_EQ(decoded.mission.scan_el_span_deg, 20.0f);
  EXPECT_FLOAT_EQ(decoded.mission.scan_el_step_deg, 5.0f);
  EXPECT_FLOAT_EQ(decoded.mission.scan_rate_deg_per_sec, 3.0f);
  EXPECT_FLOAT_EQ(decoded.mission.initial_scan_phase_deg, 15.0f);
  EXPECT_FLOAT_EQ(decoded.mission.narrow_pointing_max_slew_rate_deg_per_sec, 17.5f);
  EXPECT_FLOAT_EQ(decoded.mission.narrow_pointing_settle_tolerance_deg, 0.025f);
  EXPECT_FLOAT_EQ(decoded.policy.detection.narrow_min_snr_linear, 7.0f);
  EXPECT_FLOAT_EQ(decoded.policy.error_model.orbit_sigma_deg, 0.02f);
  EXPECT_FLOAT_EQ(decoded.policy.error_model.attitude_sigma_deg, 0.04f);
  EXPECT_FLOAT_EQ(decoded.policy.error_model.fov_sigma_deg, 0.06f);
  EXPECT_EQ(decoded.policy.error_model.random_seed, 42U);
  EXPECT_FLOAT_EQ(decoded.policy.pointing_disturbance.common_attitude_sigma_deg, 0.02f);
  EXPECT_FLOAT_EQ(decoded.policy.pointing_disturbance.common_attitude_correlation_time_s, 2.0f);
  EXPECT_FLOAT_EQ(decoded.policy.pointing_disturbance.channel_pointing_sigma_deg, 0.03f);
  EXPECT_FLOAT_EQ(decoded.policy.pointing_disturbance.channel_pointing_correlation_time_s, 3.0f);
  EXPECT_FLOAT_EQ(decoded.policy.pointing_disturbance.channel_vibration_amplitude_deg, 0.04f);
  EXPECT_FLOAT_EQ(decoded.policy.pointing_disturbance.channel_vibration_frequency_hz, 4.0f);
  EXPECT_EQ(decoded.policy.pointing_disturbance.random_seed, 43U);
  EXPECT_EQ(decoded.policy.scheduler.wide_to_narrow_required_consecutive_hits, 2);
  EXPECT_FLOAT_EQ(decoded.hardware.focal_length_m, 1.8f);
  EXPECT_FLOAT_EQ(decoded.hardware.detector_pixel_pitch_m, 25.0e-6f);
  EXPECT_EQ(decoded.policy.tracking.tracking_mode,
            config::SbirsTrackingMode::kStrictTruthAssisted);
  EXPECT_EQ(decoded.policy.tracking.estimated_backend,
            config::SbirsEstimatedTrackingBackend::kEkf);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.process_noise_diff_coeff, 2.5f);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.initial_position_std_m, 1500.0f);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.initial_velocity_std_m_per_s, 80.0f);
  EXPECT_EQ(decoded.policy.tracking.nis_gate_loss_cycles, 2U);
  EXPECT_EQ(decoded.policy.tracking.nfov_tracking_gate_loss_cycles, 4U);
  EXPECT_EQ(decoded.environment.weather_type, SbirsWeatherType::kRain);
  EXPECT_EQ(decoded.environment.sea_state, SbirsSeaState::kMedium);
  EXPECT_FLOAT_EQ(decoded.environment.base_atmospheric_transmittance, 0.7f);
  EXPECT_DOUBLE_EQ(decoded.orientation.mount_angles_deg.yaw_deg, 11.0);
  EXPECT_DOUBLE_EQ(decoded.orientation.mount_angles_deg.pitch_deg, -7.0);
  EXPECT_DOUBLE_EQ(decoded.orientation.mount_angles_deg.roll_deg, 3.0);
  EXPECT_FLOAT_EQ(decoded.orientation.sensor_scan_limits_deg.az_min_deg, -45.0f);
  EXPECT_FLOAT_EQ(decoded.orientation.sensor_scan_limits_deg.az_max_deg, 60.0f);
  EXPECT_FLOAT_EQ(decoded.orientation.sensor_scan_limits_deg.el_min_deg, -20.0f);
  EXPECT_FLOAT_EQ(decoded.orientation.sensor_scan_limits_deg.el_max_deg, 25.0f);
  EXPECT_EQ(decoded.orientation.stabilization_mode,
            config::SbirsStabilizationMode::kInertialStabilized);
  EXPECT_DOUBLE_EQ(decoded.orientation.misalignment.bias_deg.yaw_deg, 4.0);
  EXPECT_DOUBLE_EQ(decoded.orientation.misalignment.bias_deg.pitch_deg, -2.0);
  EXPECT_DOUBLE_EQ(decoded.orientation.misalignment.bias_deg.roll_deg, 1.0);
  EXPECT_FLOAT_EQ(decoded.orientation.misalignment.random_sigma_deg, 0.5f);
  EXPECT_EQ(decoded.orientation.misalignment.random_seed, 44U);
}

// --- RuntimeConfigPatch ---

TEST(SbirsReplayCodecRoundtripTest, RuntimeConfigPatchPreservesAllFields) {
  SbirsMissionConfig mission;
  mission.work_mode = SbirsWorkMode::kStandby;
  mission.scan_start_az_deg = 175.0f;
  mission.scan_span_deg = 80.0f;
  mission.scan_direction = SbirsScanDirection::kDecreasingAzimuth;
  mission.initial_scan_phase_deg = 22.5f;
  mission.narrow_pointing_max_slew_rate_deg_per_sec = 12.0f;
  mission.narrow_pointing_settle_tolerance_deg = 0.03f;
  SbirsPolicyConfig policy;
  policy.detection.wide_min_snr_linear = 5.0f;
  policy.tracking.nis_gate_loss_cycles = 3U;
  policy.tracking.nfov_tracking_gate_loss_cycles = 5U;
  policy.pointing_disturbance.common_attitude_sigma_deg = 0.05f;
  policy.pointing_disturbance.random_seed = 44U;
  SbirsEnvironmentConfig environment;
  environment.weather_type = SbirsWeatherType::kCloudy;

  SbirsRuntimeConfigPatch patch;
  patch.has_mission = true;
  patch.mission = mission;
  patch.has_policy = true;
  patch.policy = policy;
  patch.has_environment = true;
  patch.environment = environment;
  patch.has_work_mode = true;
  patch.work_mode = SbirsWorkMode::kWideSearch;
  patch.has_scan_rate_deg_per_sec = true;
  patch.scan_rate_deg_per_sec = 4.0f;
  patch.has_sensor_enabled = true;
  patch.sensor_enabled = false;
  SbirsRuntimeConfigPatch decoded;
  ASSERT_TRUE(DecodeSbirsRuntimeConfigPatch(EncodeSbirsRuntimeConfigPatch(patch), &decoded));

  EXPECT_TRUE(decoded.has_mission);
  EXPECT_EQ(decoded.mission.work_mode, SbirsWorkMode::kStandby);
  EXPECT_FLOAT_EQ(decoded.mission.scan_start_az_deg, 175.0f);
  EXPECT_FLOAT_EQ(decoded.mission.scan_span_deg, 80.0f);
  EXPECT_EQ(decoded.mission.scan_direction, SbirsScanDirection::kDecreasingAzimuth);
  EXPECT_FLOAT_EQ(decoded.mission.initial_scan_phase_deg, 22.5f);
  EXPECT_FLOAT_EQ(decoded.mission.narrow_pointing_max_slew_rate_deg_per_sec, 12.0f);
  EXPECT_FLOAT_EQ(decoded.mission.narrow_pointing_settle_tolerance_deg, 0.03f);
  EXPECT_TRUE(decoded.has_policy);
  EXPECT_FLOAT_EQ(decoded.policy.detection.wide_min_snr_linear, 5.0f);
  EXPECT_EQ(decoded.policy.tracking.nis_gate_loss_cycles, 3U);
  EXPECT_EQ(decoded.policy.tracking.nfov_tracking_gate_loss_cycles, 5U);
  EXPECT_FLOAT_EQ(decoded.policy.pointing_disturbance.common_attitude_sigma_deg, 0.05f);
  EXPECT_EQ(decoded.policy.pointing_disturbance.random_seed, 44U);
  EXPECT_TRUE(decoded.has_environment);
  EXPECT_EQ(decoded.environment.weather_type, SbirsWeatherType::kCloudy);
  EXPECT_TRUE(decoded.has_work_mode);
  EXPECT_EQ(decoded.work_mode, SbirsWorkMode::kWideSearch);
  EXPECT_TRUE(decoded.has_scan_rate_deg_per_sec);
  EXPECT_FLOAT_EQ(decoded.scan_rate_deg_per_sec, 4.0f);
  EXPECT_TRUE(decoded.has_sensor_enabled);
  EXPECT_FALSE(decoded.sensor_enabled);
}

TEST(SbirsReplayCodecRoundtripTest, EmptyRuntimeConfigPatchKeepsAllFlagsFalse) {
  const SbirsRuntimeConfigPatch patch;  // 默认构造：全部 has_* 为 false
  SbirsRuntimeConfigPatch decoded;
  ASSERT_TRUE(DecodeSbirsRuntimeConfigPatch(EncodeSbirsRuntimeConfigPatch(patch), &decoded));
  EXPECT_FALSE(decoded.has_mission);
  EXPECT_FALSE(decoded.has_policy);
  EXPECT_FALSE(decoded.has_environment);
  EXPECT_FALSE(decoded.has_work_mode);
  EXPECT_FALSE(decoded.has_scan_rate_deg_per_sec);
  EXPECT_FALSE(decoded.has_sensor_enabled);
}

// --- FailureMarker ---

TEST(SbirsReplayCodecRoundtripTest, FailureMarkerPreservesAllFields) {
  oneq::replay::ReplayTraceFailure failure;
  failure.error_code = "E_TIMEOUT";
  failure.message = "narrow cue exceeded latency budget";
  failure.location = "SbirsSession::ExecuteCycle";
  failure.has_cycle_index = true;
  failure.cycle_index = 88U;
  failure.has_sim_time_sec = true;
  failure.sim_time_sec = 12.5;
  failure.diagnostics_payload = "{\"retry\":3}";

  const std::string bytes = EncodeSbirsFailureMarker(failure);
  ASSERT_FALSE(bytes.empty());

  oneq::replay::ReplayTraceFailure decoded;
  std::string error;
  ASSERT_TRUE(DecodeSbirsFailureMarker(bytes, &decoded, &error)) << error;

  EXPECT_EQ(decoded.error_code, "E_TIMEOUT");
  EXPECT_EQ(decoded.message, "narrow cue exceeded latency budget");
  EXPECT_EQ(decoded.location, "SbirsSession::ExecuteCycle");
  EXPECT_TRUE(decoded.has_cycle_index);
  EXPECT_EQ(decoded.cycle_index, 88U);
  EXPECT_TRUE(decoded.has_sim_time_sec);
  EXPECT_DOUBLE_EQ(decoded.sim_time_sec, 12.5);
  EXPECT_EQ(decoded.diagnostics_payload, "{\"retry\":3}");
}

// --- 负路径：null out / 空 payload / 损坏 payload ---

TEST(SbirsReplayCodecRoundtripTest, DecodeCycleInputRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeSbirsCycleInput("", nullptr));
  SbirsCycleInput input;
  EXPECT_FALSE(DecodeSbirsCycleInput("corrupt", &input));
}

TEST(SbirsReplayCodecRoundtripTest, DecodeOutputFrameRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeSbirsOutputFrame("", nullptr));
  SbirsOutputFrame frame;
  EXPECT_FALSE(DecodeSbirsOutputFrame("bad", &frame));
}

TEST(SbirsReplayCodecRoundtripTest, DecodeCycleResultRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeSbirsCycleResult("", nullptr));
  SbirsCycleResult result;
  EXPECT_FALSE(DecodeSbirsCycleResult("bad", &result));
}

TEST(SbirsReplayCodecRoundtripTest, DecodeCycleResultRejectsUnknownAbortReasonAtomically) {
  // 值 2 已由 kSensorPoweredOff 占用（合法），从非法集合中移除。
  const std::int32_t invalid_reasons[] = {3, -1, std::numeric_limits<std::int32_t>::max()};
  for (const std::int32_t invalid_reason : invalid_reasons) {
    SbirsCycleResult result;
    result.input_cycle_index = 17U;
    result.output_frame.cycle_index = 18U;
    result.status = SbirsCycleStatus::kCompleted;
    result.abort_reason = SbirsPipelineAbortReason::kValidationRejected;

    EXPECT_FALSE(DecodeSbirsCycleResult(EncodeCycleResultWithRawAbortReason(invalid_reason),
                                       &result));
    EXPECT_EQ(result.input_cycle_index, 17U);
    EXPECT_EQ(result.output_frame.cycle_index, 18U);
    EXPECT_EQ(result.status, SbirsCycleStatus::kCompleted);
    EXPECT_EQ(result.abort_reason, SbirsPipelineAbortReason::kValidationRejected);
  }
}

TEST(SbirsReplayCodecRoundtripTest, DecodeCycleResultRejectsUnknownStatusAtomically) {
  // status 合法值 0..3（kCompleted..kRejectedExecution）；未知值必须 fail closed，
  // 且不得修改调用方传入的输出对象（原子性）。
  const std::int32_t invalid_statuses[] = {4, 255, -1};
  for (const std::int32_t invalid_status : invalid_statuses) {
    SbirsCycleResult result;
    result.input_cycle_index = 17U;
    result.output_frame.cycle_index = 18U;
    result.status = SbirsCycleStatus::kCompleted;

    EXPECT_FALSE(DecodeSbirsCycleResult(EncodeCycleResultWithRawStatus(invalid_status),
                                        &result));
    EXPECT_EQ(result.input_cycle_index, 17U);
    EXPECT_EQ(result.output_frame.cycle_index, 18U);
    EXPECT_EQ(result.status, SbirsCycleStatus::kCompleted);
  }
}

TEST(SbirsReplayCodecRoundtripTest, DecodeSessionConfigRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeSbirsSessionConfig("", nullptr));
  SbirsSessionConfig config;
  EXPECT_FALSE(DecodeSbirsSessionConfig("bad", &config));
}

TEST(SbirsReplayCodecRoundtripTest, DecodeSessionConfigRejectsUnknownScanDirectionAtomically) {
  SbirsSessionConfig config;
  config.mission.scan_start_az_deg = 12.0f;
  config.mission.scan_span_deg = 34.0f;
  EXPECT_FALSE(DecodeSbirsSessionConfig(EncodeSessionConfigWithRawScanDirection(99), &config));
  EXPECT_FLOAT_EQ(config.mission.scan_start_az_deg, 12.0f);
  EXPECT_FLOAT_EQ(config.mission.scan_span_deg, 34.0f);
}

TEST(SbirsReplayCodecRoundtripTest, DecodeSessionConfigRejectsUnknownAzimuthReferenceAtomically) {
  SbirsSessionConfig config;
  config.mission.scan_start_az_deg = 12.0f;
  config.mission.scan_azimuth_reference = config::SbirsScanAzimuthReference::kNadirRelative;
  EXPECT_FALSE(DecodeSbirsSessionConfig(EncodeSessionConfigWithRawAzimuthReference(99), &config));
  EXPECT_FLOAT_EQ(config.mission.scan_start_az_deg, 12.0f);
  EXPECT_EQ(config.mission.scan_azimuth_reference,
            config::SbirsScanAzimuthReference::kNadirRelative);
}

TEST(SbirsReplayCodecRoundtripTest, DecodeSessionConfigRejectsUnknownTrackingEnumsAtomically) {
  SbirsSessionConfig config;
  config.policy.tracking.tracking_mode = config::SbirsTrackingMode::kStrictTruthAssisted;
  EXPECT_FALSE(DecodeSbirsSessionConfig(EncodeSessionConfigWithRawTrackingEnums(99, 0), &config));
  EXPECT_EQ(config.policy.tracking.tracking_mode,
            config::SbirsTrackingMode::kStrictTruthAssisted);
  EXPECT_FALSE(DecodeSbirsSessionConfig(EncodeSessionConfigWithRawTrackingEnums(0, 99), &config));
  EXPECT_EQ(config.policy.tracking.tracking_mode,
            config::SbirsTrackingMode::kStrictTruthAssisted);
}

TEST(SbirsReplayCodecRoundtripTest, DecodeCycleResultRejectsUnknownTrackingSourceAtomically) {
  SbirsCycleResult result;
  result.input_cycle_index = 17U;
  EXPECT_FALSE(DecodeSbirsCycleResult(EncodeCycleResultWithRawTrackingSource(99), &result));
  EXPECT_EQ(result.input_cycle_index, 17U);
  EXPECT_TRUE(result.detection_attributions.empty());
}

TEST(SbirsReplayCodecRoundtripTest, DecodeRuntimeConfigPatchRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeSbirsRuntimeConfigPatch("", nullptr));
  SbirsRuntimeConfigPatch patch;
  EXPECT_FALSE(DecodeSbirsRuntimeConfigPatch("bad", &patch));
}

// --- 版本护栏：v1/异源标识符负载显式拒绝（2026-08 ECI 变更后防静默误解码） ---

TEST(SbirsReplayCodecRoundtripTest, DecodeRejectsLegacyOrForeignIdentifierBuffers) {
  // v1 时代录制从未写入 file_identifier（deg/ECEF 语义）；v2 codec 写入并校验
  // 标识符，标识符不符必须显式拒绝。标识符位于字节 [4,8)（root uoffset 之后）。
  // v3（2026-08-24 去 has_* 槽位变更）起，v2 的 "SBI2" 同样属被拒的旧版本载荷。
  SbirsCycleInput input;
  input.cycle_index = 3U;
  input.dt_sec = 1.0f;
  input.utc_julian_day = 2460310.5;
  const std::string encoded = EncodeSbirsCycleInput(input);
  ASSERT_GE(encoded.size(), 8U);
  EXPECT_TRUE(DecodeSbirsCycleInput(encoded, &input));

  std::string legacy = encoded;
  legacy.erase(4U, 4U);  // 抹掉标识符 = v1 时代录制形态
  SbirsCycleInput legacy_decoded;
  EXPECT_FALSE(DecodeSbirsCycleInput(legacy, &legacy_decoded));

  std::string v1_identifier = encoded;
  v1_identifier.replace(4U, 4U, "SBIC");  // v1 声明过的标识符
  EXPECT_FALSE(DecodeSbirsCycleInput(v1_identifier, &legacy_decoded));

  std::string v2_identifier = encoded;
  v2_identifier.replace(4U, 4U, "SBI2");  // v2 标识符（含 has_* 槽位的旧表）
  EXPECT_FALSE(DecodeSbirsCycleInput(v2_identifier, &legacy_decoded));

  std::string v3_identifier = encoded;
  // v3 标识符（多通道调度配置时代的旧表）；v4（2026-09-02 单镜筒化删
  // max_concurrent_nfov_locks）起 SBI3 同属被拒的旧版本载荷。
  v3_identifier.replace(4U, 4U, "SBI3");
  EXPECT_FALSE(DecodeSbirsCycleInput(v3_identifier, &legacy_decoded));

  std::string v4_identifier = encoded;
  // v4 标识符（无量测时间戳与初始相位字段）；v5（2026-09-07 SBI5）起 SBI4 同属被拒旧版本。
  v4_identifier.replace(4U, 4U, "SBI4");
  EXPECT_FALSE(DecodeSbirsCycleInput(v4_identifier, &legacy_decoded));

  // 其余 sbirs replay 负载同样受护栏保护：篡改标识符后拒绝。
  SbirsOutputFrame frame;
  frame.cycle_index = 3U;
  std::string frame_bytes = EncodeSbirsOutputFrame(frame);
  frame_bytes.replace(4U, 4U, "XXXX");
  SbirsOutputFrame frame_decoded;
  EXPECT_FALSE(DecodeSbirsOutputFrame(frame_bytes, &frame_decoded));

  SbirsCycleResult result;
  result.status = SbirsCycleStatus::kCompleted;
  std::string result_bytes = EncodeSbirsCycleResult(result);
  result_bytes.replace(4U, 4U, "XXXX");
  SbirsCycleResult result_decoded;
  EXPECT_FALSE(DecodeSbirsCycleResult(result_bytes, &result_decoded));

  config::SbirsSessionConfig config;
  std::string config_bytes = EncodeSbirsSessionConfig(config);
  config_bytes.replace(4U, 4U, "XXXX");
  config::SbirsSessionConfig config_decoded;
  EXPECT_FALSE(DecodeSbirsSessionConfig(config_bytes, &config_decoded));

  config::SbirsRuntimeConfigPatch patch;
  std::string patch_bytes = EncodeSbirsRuntimeConfigPatch(patch);
  patch_bytes.replace(4U, 4U, "XXXX");
  config::SbirsRuntimeConfigPatch patch_decoded;
  EXPECT_FALSE(DecodeSbirsRuntimeConfigPatch(patch_bytes, &patch_decoded));
}

TEST(SbirsReplayCodecRoundtripTest, DecodeFailureMarkerRejectsNullAndCorrupted) {
  std::string error;
  // null out
  EXPECT_FALSE(DecodeSbirsFailureMarker("", nullptr, &error));
  // 空 payload
  oneq::replay::ReplayTraceFailure failure;
  EXPECT_FALSE(DecodeSbirsFailureMarker("", &failure, &error));
  // 损坏 payload
  EXPECT_FALSE(DecodeSbirsFailureMarker("bad", &failure, &error));
}

TEST(SbirsReplayCodecRoundtripTest, SessionConfigPreservesImmTrackingFields) {
  SbirsSessionConfig config;
  config.policy.tracking.estimated_backend = config::SbirsEstimatedTrackingBackend::kImm;
  config.policy.tracking.imm_model_noise_diff_coeffs = {0.5f, 80.0f, 200.0f};

  const std::string encoded = EncodeSbirsSessionConfig(config);
  SbirsSessionConfig decoded;
  ASSERT_TRUE(DecodeSbirsSessionConfig(encoded, &decoded));

  EXPECT_EQ(decoded.policy.tracking.estimated_backend,
            config::SbirsEstimatedTrackingBackend::kImm);
  ASSERT_EQ(decoded.policy.tracking.imm_model_noise_diff_coeffs.size(), 3U);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.imm_model_noise_diff_coeffs[0], 0.5f);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.imm_model_noise_diff_coeffs[1], 80.0f);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.imm_model_noise_diff_coeffs[2], 200.0f);
}

TEST(SbirsReplayCodecRoundtripTest, SessionConfigPreservesAngleCvKfBackend) {
  SbirsSessionConfig config;
  config.policy.tracking.estimated_backend = config::SbirsEstimatedTrackingBackend::kAngleCvKf;

  const std::string encoded = EncodeSbirsSessionConfig(config);
  SbirsSessionConfig decoded;
  ASSERT_TRUE(DecodeSbirsSessionConfig(encoded, &decoded));
  EXPECT_EQ(decoded.policy.tracking.estimated_backend,
            config::SbirsEstimatedTrackingBackend::kAngleCvKf);
}

}  // namespace tests
}  // namespace session
}  // namespace sbirs_sensor
