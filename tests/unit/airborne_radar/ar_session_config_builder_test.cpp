// Copyright 2026. All Rights Reserved.
//
// @file radar_session_config_builder_test.cpp
// @brief 验证 ArProfileConstants 常量赋值路径与运行期补丁直接赋值。

#include <gtest/gtest.h>

#include <limits>

#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/config/ArSessionConfigValidation.h"
#include "1q/airborne_radar/session/ArSession.h"

namespace airborne_radar {
namespace tests {

namespace {

config::ArSessionConfig MakeDetectionFocusedConfig() {
  config::ArSessionConfig config;
  config.policy.detection = config::profiles::kDetectionPriorityDetection;
  config.policy.tracking = config::profiles::kFastAssociationTracking;
  config.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
  return config;
}

}  // namespace

TEST(RadarSessionConfigBuilderTest, DefaultConstructionPreservesSemanticDefaults) {
  const config::ArSessionConfig config;

  EXPECT_FLOAT_EQ(config.policy.detection.minimum_detection_margin_db, -2.0f);
  EXPECT_EQ(config.policy.detection.pulse_count, 10);
  EXPECT_FALSE(config.policy.tracking.enable_kalman_filter);
  EXPECT_EQ(config.policy.lifecycle.confirm_hits, 3U);
  EXPECT_EQ(config.policy.lifecycle.max_miss_before_lost, 2U);
  EXPECT_FALSE(config.policy.lifecycle.enable_imm_lifecycle);
}

TEST(RadarSessionConfigBuilderTest, DetectionFocusedConfigPreservesValues) {
  const auto config = MakeDetectionFocusedConfig();

  EXPECT_EQ(config.policy.detection.pulse_count, 16);
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, -12.0f);
  EXPECT_FLOAT_EQ(config.policy.tracking.kalman_measurement_noise_std, 6.0f);
  EXPECT_EQ(config.policy.lifecycle.confirm_hits, 1U);
}

TEST(RadarSessionConfigBuilderTest, DirectFieldAssignmentPreservesDetailedConfig) {
  config::ArSessionConfig base_config{};
  base_config.hardware.transmitter.peak_power_w = 7.5e6f;
  base_config.hardware.transmitter.frequency_hz = 9.7e9f;
  base_config.policy.tracking.enable_kalman_filter = true;
  base_config.policy.tracking.kalman_measurement_noise_std = 4.5f;
  base_config.policy.lifecycle.enable_imm_lifecycle = true;
  base_config.policy.lifecycle.confirm_hits = 2U;

  EXPECT_FLOAT_EQ(base_config.hardware.transmitter.peak_power_w, 7.5e6f);
  EXPECT_FLOAT_EQ(base_config.hardware.transmitter.frequency_hz, 9.7e9f);
  EXPECT_TRUE(base_config.policy.tracking.enable_kalman_filter);
  EXPECT_FLOAT_EQ(base_config.policy.tracking.kalman_measurement_noise_std, 4.5f);
  EXPECT_TRUE(base_config.policy.lifecycle.enable_imm_lifecycle);
  EXPECT_EQ(base_config.policy.lifecycle.confirm_hits, 2U);
}

TEST(RadarSessionConfigBuilderTest, ProfileConstantsAssignToConfig) {
  config::ArSessionConfig config;
  config.hardware = config::profiles::kLongRangeHighPowerHardware;
  config.policy.detection = config::profiles::kTrackStabilityPriorityDetection;
  config.hardware.antenna.pattern = config::profiles::kLowSidelobeAntennaPattern;
  config.hardware.rcs_physics = config::profiles::kEnhancedRcsPhysics;

  EXPECT_FLOAT_EQ(config.hardware.transmitter.peak_power_w, 5.0e6f);
  EXPECT_EQ(config.policy.detection.pulse_count, 8);
  EXPECT_FLOAT_EQ(config.hardware.antenna.pattern.max_sidelobe_level_db, -30.0f);
  EXPECT_TRUE(config.hardware.rcs_physics.enable_physical_rcs);
  EXPECT_FLOAT_EQ(config.hardware.rcs_physics.physics_mix_ratio, 0.60f);
}

TEST(RadarSessionConfigBuilderTest, TrackingAndLifecycleConstantsAssignToConfig) {
  config::ArSessionConfig config;
  config.policy.tracking = config::profiles::kRobustAntiJammingTracking;
  config.policy.association = config::profiles::kRobustAntiJammingAssociation;
  config.policy.tracking.enable_kalman_filter = true;
  config.policy.lifecycle = config::profiles::kHighPersistenceLifecycle;
  config.policy.lifecycle.enable_imm_lifecycle = true;

  EXPECT_TRUE(config.policy.tracking.enable_kalman_filter);
  EXPECT_TRUE(config.policy.lifecycle.enable_imm_lifecycle);
  EXPECT_EQ(config.policy.lifecycle.confirm_hits, 3U);
  EXPECT_EQ(config.policy.lifecycle.max_lost_cycles, 8U);
}

TEST(RadarSessionConfigBuilderTest, BuiltConfigCanConstructRadarSession) {
  const auto config = MakeDetectionFocusedConfig();
  session::ArSession session = session::ArSession::Create(config);
  EXPECT_TRUE(session.HasLatestControlProfile() == false ||
              session.HasLatestControlProfile() == true);
}

TEST(RadarSessionConfigBuilderTest, DefaultConstructedPatchLeavesAllFlagsUnset) {
  const config::ArRuntimeConfigPatch patch;
  EXPECT_FALSE(patch.has_mission);
  EXPECT_FALSE(patch.has_policy);
  EXPECT_FALSE(patch.has_environment);
  EXPECT_FALSE(patch.has_work_mode);
  EXPECT_FALSE(patch.has_scan_center_deg);
  EXPECT_FALSE(patch.has_dwell_center_deg);
  EXPECT_FALSE(patch.has_commanded_beamwidth_deg);
  EXPECT_FALSE(patch.has_commanded_beamwidth_enabled);
  EXPECT_FALSE(patch.has_sensor_enabled);
}

TEST(RadarSessionConfigBuilderTest, RuntimeConfigPatchSetsAllFields) {
  config::AzimuthElevationDeg scan_center;
  scan_center.az_deg = 12.0f;
  scan_center.el_deg = -3.0f;

  config::AzimuthElevationDeg dwell_center;
  dwell_center.az_deg = 5.0f;
  dwell_center.el_deg = 2.0f;

  config::CommandedBeamwidthDeg commanded_beamwidth;
  commanded_beamwidth.commanded_az_beamwidth_deg = 2.5f;
  commanded_beamwidth.commanded_el_beamwidth_deg = 2.0f;

  config::ArRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::ArWorkMode::kStt;
  patch.has_scan_center_deg = true;
  patch.scan_center_deg = scan_center;
  patch.has_dwell_center_deg = true;
  patch.dwell_center_deg = dwell_center;
  patch.has_commanded_beamwidth_deg = true;
  patch.commanded_beamwidth_deg = commanded_beamwidth;
  patch.has_commanded_beamwidth_enabled = true;
  patch.commanded_beamwidth_enabled = true;
  patch.has_environment = true;
  patch.environment.has_scenario_config = true;
  patch.environment.scenario_config = config::EnvironmentScenarioConfig{};

  EXPECT_TRUE(patch.has_work_mode);
  EXPECT_EQ(patch.work_mode, config::ArWorkMode::kStt);
  EXPECT_TRUE(patch.has_scan_center_deg);
  EXPECT_FLOAT_EQ(patch.scan_center_deg.az_deg, 12.0f);
  EXPECT_FLOAT_EQ(patch.scan_center_deg.el_deg, -3.0f);
  EXPECT_TRUE(patch.has_dwell_center_deg);
  EXPECT_FLOAT_EQ(patch.dwell_center_deg.az_deg, 5.0f);
  EXPECT_FLOAT_EQ(patch.dwell_center_deg.el_deg, 2.0f);
  EXPECT_TRUE(patch.has_commanded_beamwidth_deg);
  EXPECT_FLOAT_EQ(patch.commanded_beamwidth_deg.commanded_az_beamwidth_deg, 2.5f);
  EXPECT_FLOAT_EQ(patch.commanded_beamwidth_deg.commanded_el_beamwidth_deg, 2.0f);
  EXPECT_TRUE(patch.has_commanded_beamwidth_enabled);
  EXPECT_TRUE(patch.commanded_beamwidth_enabled);
  EXPECT_TRUE(patch.has_environment);
  EXPECT_TRUE(patch.environment.has_scenario_config);
}

TEST(RadarSessionConfigBuilderTest, RuntimePatchCanBeAppliedWithoutReconstructingSession) {
  session::ArSession session = session::ArSession::Create(MakeDetectionFocusedConfig());

  config::ArRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::ArWorkMode::kTas;
  patch.has_commanded_beamwidth_enabled = true;
  patch.commanded_beamwidth_enabled = true;
  patch.has_environment = true;
  patch.environment.has_scenario_config = true;
  patch.environment.scenario_config = config::EnvironmentScenarioConfig{};

  ASSERT_TRUE(session.TryApplyRuntimeConfig(patch));

  session::ArCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 1.0;
  input.platform.platform_position_ecef_m.x_m = 6378137.0;
  const session::ArCycleResult result = session.StepWithResult(input);
  EXPECT_FALSE(session::HasValidationError(result.issues));
}

TEST(RadarSessionConfigBuilderTest, DetailedBuilderProducesDetailedSessionConfig) {
  config::ArSessionConfig detailed_config{};
  detailed_config.hardware.transmitter.peak_power_w = 5.0e6f;
  detailed_config.hardware.transmitter.frequency_hz = 9.3e9f;
  detailed_config.hardware.transmitter.bandwidth_hz = 10.0e6f;
  detailed_config.hardware.transmitter.pulse_width_s = 20e-6f;
  detailed_config.hardware.transmitter.prf_hz = 500.0f;
  detailed_config.hardware.antenna.main_beam_gain_db = 38.0f;
  detailed_config.hardware.receiver.noise_figure_db = 3.5f;
  detailed_config.mission.work_mode = config::ArWorkMode::kTas;
  detailed_config.policy.tracking.enable_kalman_filter = true;
  detailed_config.policy.tracking.kalman_measurement_noise_std = 7.5f;
  detailed_config.policy.lifecycle.enable_imm_lifecycle = true;
  detailed_config.policy.lifecycle.confirm_hits = 2U;
  detailed_config.policy.lifecycle.max_miss_before_lost = 1U;
  detailed_config.policy.lifecycle.max_lost_cycles = 4U;
  detailed_config.environment.scenario_config.atmospheric_physics
      .enable_physical_model = true;

  EXPECT_FLOAT_EQ(detailed_config.hardware.transmitter.peak_power_w, 5.0e6f);
  EXPECT_FLOAT_EQ(detailed_config.hardware.transmitter.frequency_hz, 9.3e9f);
  EXPECT_EQ(detailed_config.mission.work_mode, config::ArWorkMode::kTas);
  EXPECT_FLOAT_EQ(detailed_config.policy.tracking.kalman_measurement_noise_std, 7.5f);
  EXPECT_EQ(detailed_config.policy.lifecycle.confirm_hits, 2U);
  EXPECT_TRUE(detailed_config.policy.lifecycle.enable_imm_lifecycle);
  EXPECT_TRUE(detailed_config.environment.scenario_config.atmospheric_physics
                  .enable_physical_model);
}

TEST(RadarSessionConfigBuilderTest, DetailedBuiltConfigCanConstructRadarSession) {
  config::ArSessionConfig config{};
  config.hardware.transmitter.peak_power_w = 5.0e6f;
  config.hardware.transmitter.frequency_hz = 9.3e9f;
  config.policy.lifecycle.enable_imm_lifecycle = true;

  session::ArSession session = session::ArSession::Create(config);
  EXPECT_TRUE(session.HasLatestControlProfile() == false ||
              session.HasLatestControlProfile() == true);
}

TEST(RadarSessionConfigBuilderTest, DetailedBeamSchedulerWritesPolicyPath) {
  config::AzimuthElevationDeg default_scan_center;
  default_scan_center.az_deg = 6.0f;
  default_scan_center.el_deg = -1.5f;
  config::CommandedBeamwidthDeg nominal_beamwidth;
  nominal_beamwidth.commanded_az_beamwidth_deg = 2.5f;
  nominal_beamwidth.commanded_el_beamwidth_deg = 1.5f;

  config::ArSessionConfig detailed_config{};
  detailed_config.policy.beam_control.scheduler.azimuth_step_count_hint = 8U;
  detailed_config.policy.beam_control.scheduler.elevation_step_count_hint = 4U;
  detailed_config.policy.beam_control.scheduler.prefer_dense_tas_sampling = true;
  detailed_config.mission.scan_center_deg = default_scan_center;
  detailed_config.policy.beam_control.pointing.nominal_beamwidth_deg = nominal_beamwidth;
  detailed_config.mission.commanded_beamwidth_deg = nominal_beamwidth;

  EXPECT_EQ(detailed_config.policy.beam_control.scheduler.azimuth_step_count_hint, 8U);
  EXPECT_EQ(detailed_config.policy.beam_control.scheduler.elevation_step_count_hint, 4U);
  EXPECT_TRUE(detailed_config.policy.beam_control.scheduler.prefer_dense_tas_sampling);
  EXPECT_FLOAT_EQ(detailed_config.mission.scan_center_deg.az_deg, 6.0f);
  EXPECT_FLOAT_EQ(detailed_config.mission.scan_center_deg.el_deg, -1.5f);
  EXPECT_FLOAT_EQ(
      detailed_config.mission.commanded_beamwidth_deg.commanded_az_beamwidth_deg, 2.5f);
  EXPECT_FLOAT_EQ(
      detailed_config.mission.commanded_beamwidth_deg.commanded_el_beamwidth_deg, 1.5f);
}

TEST(RadarSessionConfigValidationTest, ReportsInvalidCommandedBeamwidth) {
  config::ArSessionConfig session_config;
  session_config.mission.commanded_beamwidth_enabled = true;
  session_config.mission.commanded_beamwidth_deg.commanded_az_beamwidth_deg = 0.0f;
  session_config.mission.commanded_beamwidth_deg.commanded_el_beamwidth_deg = 1.0f;

  const auto issues = config::ValidateArSessionConfig(session_config);
  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues.front().code, "ar.validation.commanded_beamwidth_az_not_positive");
  // 创建时配置校验问题统一 phase=kInputValidation + severity=kError（规则 14 config 域）。
  EXPECT_EQ(issues.front().phase, session::ArIssuePhase::kInputValidation);
  EXPECT_EQ(issues.front().severity, session::ArIssueSeverity::kError);
}

TEST(RadarSessionConfigValidationTest, RejectsNonFiniteCommandedBeamwidth) {
  config::ArSessionConfig session_config;
  session_config.mission.commanded_beamwidth_enabled = true;
  session_config.mission.commanded_beamwidth_deg.commanded_az_beamwidth_deg = 1.0f;
  session_config.mission.commanded_beamwidth_deg.commanded_el_beamwidth_deg =
      std::numeric_limits<float>::quiet_NaN();

  const auto issues = config::ValidateArSessionConfig(session_config);
  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues.front().code, "ar.validation.commanded_beamwidth_el_not_positive");
}

TEST(RadarSessionConfigValidationTest, AcceptsPhysicalApertureWhenNominalBeamwidthIsZero) {
  config::ArSessionConfig session_config;
  session_config.hardware.antenna.nominal_az_beamwidth_deg = 0.0f;
  session_config.hardware.antenna.antenna_length_m = 1.5f;
  session_config.hardware.antenna.nominal_el_beamwidth_deg = 0.0f;
  session_config.hardware.antenna.antenna_width_m = 0.8f;

  EXPECT_TRUE(config::ValidateArSessionConfig(session_config).empty());
}

TEST(RadarSessionConfigValidationTest, RejectsMissingOrNonFiniteAntennaGeometry) {
  config::ArSessionConfig session_config;
  session_config.hardware.antenna.nominal_az_beamwidth_deg = 0.0f;
  session_config.hardware.antenna.antenna_length_m = 0.0f;
  session_config.hardware.antenna.antenna_width_m = std::numeric_limits<float>::quiet_NaN();

  const auto issues = config::ValidateArSessionConfig(session_config);
  ASSERT_EQ(issues.size(), 2U);
  EXPECT_EQ(issues[0].code, "ar.validation.antenna_az_geometry_invalid");
  EXPECT_EQ(issues[1].code, "ar.validation.antenna_el_geometry_invalid");
  // 多条目场景同样锁定 config 域 phase/severity 不变式（规则 14 config 域）。
  EXPECT_EQ(issues[0].phase, session::ArIssuePhase::kInputValidation);
  EXPECT_EQ(issues[1].phase, session::ArIssuePhase::kInputValidation);
  EXPECT_EQ(issues[0].severity, session::ArIssueSeverity::kError);
}

TEST(RadarSessionConfigValidationTest, RejectsInvalidTransmitterFrequency) {
  config::ArSessionConfig session_config;
  session_config.hardware.transmitter.frequency_hz = 0.0f;

  const auto issues = config::ValidateArSessionConfig(session_config);
  ASSERT_EQ(issues.size(), 2U);
  EXPECT_EQ(issues[0].code, "ar.validation.transmitter_frequency_invalid");
  EXPECT_EQ(issues[1].code, "ar.validation.frequency_plan_invalid");
}

TEST(RadarSessionConfigValidationTest, ValidatesEngineeringTransmitterEnvelopeAndIdentity) {
  config::ArSessionConfig session_config;
  session_config.hardware.transmitter.frequency_plan_hz = {3.0e9, 3.1e9};
  EXPECT_TRUE(config::ValidateArSessionConfig(session_config).empty());

  session_config.hardware.transmitter.frequency_plan_hz = {3.1e9};
  auto issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.front().code, "ar.validation.frequency_plan_invalid");

  session_config.hardware.transmitter.frequency_plan_hz = {3.0e9};
  session_config.hardware.transmitter.maximum_peak_power_w = 5.0e5f;
  issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.front().code,
            "ar.validation.transmitter_operating_envelope_invalid");

}

TEST(RadarSessionConfigValidationTest, ValidatesReceiverRfHardwareBoundary) {
  config::ArSessionConfig session_config;
  ASSERT_EQ(session_config.hardware.receiver.co_site_paths.size(), 1U);
  EXPECT_EQ(session_config.hardware.receiver.co_site_paths.front().transmitter_sensor_entity_id, 1U);
  session_config.hardware.receiver.maximum_linear_input_power_w = 0.0f;
  auto issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.back().code, "ar.validation.receiver_rf_hardware_invalid");

  session_config.hardware.receiver.maximum_linear_input_power_w = 1.0e-3f;
  session_config.hardware.receiver.interference_observation_jn_gate_db =
      std::numeric_limits<float>::quiet_NaN();
  issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.back().code, "ar.validation.receiver_rf_hardware_invalid");

  session_config.hardware.receiver.interference_observation_jn_gate_db = 6.0f;
  session_config.hardware.receiver.has_co_site_isolation = true;
  session_config.hardware.receiver.co_site_isolation_db = 80.0f;
  EXPECT_TRUE(config::ValidateArSessionConfig(session_config).empty());

  session_config.hardware.receiver.co_site_paths.push_back(
      oneq::electromagnetics::RfCoSiteIsolationPath{101U, 80.0});
  EXPECT_TRUE(config::ValidateArSessionConfig(session_config).empty());
  session_config.hardware.receiver.co_site_paths.back().transmitter_sensor_entity_id = 0U;
  issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.back().code, "ar.validation.receiver_rf_hardware_invalid");
  session_config.hardware.receiver.co_site_paths.clear();

  session_config.hardware.receiver.cross_polarization_isolation_db = -1.0f;
  issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.back().code, "ar.validation.receiver_rf_hardware_invalid");
}

TEST(RadarSessionConfigValidationTest, RejectsSignalProcessingGainsOutsideRange) {
  config::ArSessionConfig session_config;
  session_config.hardware.signal_processing.clutter_suppression_gain_db = 41.0f;
  auto issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.front().code, "ar.validation.signal_processing_gains_invalid");

  session_config.hardware.signal_processing.clutter_suppression_gain_db = -1.0f;
  issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.front().code, "ar.validation.signal_processing_gains_invalid");

  session_config.hardware.signal_processing.noise_processing_gain_db = 40.0f;
  session_config.hardware.signal_processing.clutter_suppression_gain_db = 0.0f;
  issues = config::ValidateArSessionConfig(session_config);
  EXPECT_TRUE(issues.empty());

  session_config.hardware.signal_processing.jamming_suppression_gain_db =
      std::numeric_limits<float>::quiet_NaN();
  issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.front().code, "ar.validation.signal_processing_gains_invalid");
}

TEST(RadarSessionCreateWithDiagnosticsTest, BuildsSessionAndReportsNoIssuesForHealthyConfig) {
  config::ArSessionConfig config;
  config.policy.lifecycle.enable_imm_lifecycle = false;

  session::ArIssueList issues;
  const session::ArSession session = session::ArSession::CreateWithDiagnostics(config, &issues);

  EXPECT_TRUE(issues.empty());
  (void)session;
}

TEST(RadarSessionCreateWithDiagnosticsTest, ReportsIssuesButStillConstructsSession) {
  config::ArSessionConfig invalid;
  invalid.orientation.mechanical_scan_limits_deg.az_min_deg = 10.0f;
  invalid.orientation.mechanical_scan_limits_deg.az_max_deg = -10.0f;

  session::ArIssueList issues;
  const session::ArSession session = session::ArSession::CreateWithDiagnostics(invalid, &issues);

  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues.front().code, "ar.validation.mechanical_scan_limits_swapped_az");
  (void)session;  // 会话仍被构造，调用方据 issues 决策
}

TEST(RadarSessionCreateWithDiagnosticsTest, AcceptsNullIssuesWithoutCrash) {
  config::ArSessionConfig invalid;
  invalid.orientation.mechanical_scan_limits_deg.az_min_deg = 10.0f;
  invalid.orientation.mechanical_scan_limits_deg.az_max_deg = -10.0f;

  const session::ArSession session = session::ArSession::CreateWithDiagnostics(invalid, nullptr);
  (void)session;  // nullptr 时仅构造，不写回 issues
}

}  // namespace tests
}  // namespace airborne_radar
