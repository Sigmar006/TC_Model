// Copyright 2026. All Rights Reserved.
//
// @file esr_public_api_convenience_test.cpp
// @brief ESR 对外易用性 API 契约测试（高层语义配置版本）。

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "1q/electromagnetics/RfScene.h"
#include "1q/electronic_surveillance_radar/config/EsrProfileConstants.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"

namespace electronic_surveillance_radar {
namespace tests {
namespace {

bool ContainsEsrIssueCode(const session::EsrIssueList& issues, const std::string& code) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].code == code) {
      return true;
    }
  }
  return false;
}

config::EsrSessionConfig MakeSessionConfig() {
  config::EsrSessionConfig config;
  config.mission = config::profiles::kElectronicOrderOfBattleMission;
  // kStandard 灵敏度档位为 no-op（struct 默认即该档位），不再显式赋值。
  config.environment.scenario_config.preset = config::EsrEnvironmentPreset::kStandard;
  config.mission.scan.use_explicit_scan_bounds = true;
  config.mission.scan.scan_start_az_deg = -60.0f;
  config.mission.scan.scan_end_az_deg = 60.0f;
  config.mission.scan.scan_start_el_deg = -20.0f;
  config.mission.scan.scan_end_el_deg = 20.0f;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 40.0f;
  return config;
}

}  // namespace

TEST(EsrPublicApiConvenienceTest, SessionConfigDefaultsMatchDefaultConfig) {
  const config::EsrSessionConfig built;
  const config::EsrSessionConfig defaults;

  EXPECT_EQ(built.mission.work_mode, defaults.mission.work_mode);
  EXPECT_EQ(built.environment.scenario_config.preset, defaults.environment.scenario_config.preset);
}

TEST(EsrPublicApiConvenienceTest, SemanticProfileConstantsProduceExpectedConfig) {
  config::EsrSessionConfig cfg;
  cfg.mission = config::profiles::kThreatWarningMission;
  cfg.policy.detection = config::profiles::kHighSensitivityDetection;
  cfg.environment.scenario_config.preset = config::EsrEnvironmentPreset::kJammed;
  cfg.sensor_enabled = false;

  EXPECT_EQ(cfg.mission.work_mode, config::EsrWorkMode::kRwr);
  EXPECT_FALSE(cfg.sensor_enabled);
  EXPECT_FLOAT_EQ(cfg.mission.scan.scan_rate_hz, 5.0f);
  EXPECT_FLOAT_EQ(cfg.policy.detection.minimum_snr_db, 3.0f);
  EXPECT_EQ(cfg.environment.scenario_config.preset, config::EsrEnvironmentPreset::kJammed);
}

TEST(EsrPublicApiConvenienceTest, DetailedSessionConfigSupportsDetailedDetectionParams) {
  config::EsrSessionConfig details_cfg{};
  details_cfg.policy.detection.minimum_snr_db = 8.0f;
  details_cfg.policy.detection.pfa = 1.0e-5f;
  details_cfg.policy.detection.pulse_count = 16U;
  details_cfg.policy.detection.threshold_scale = 0.9f;
  details_cfg.policy.detection.enable_statistical_detection = true;
  details_cfg.mission.scan.scan_rate_hz = 4.0f;
  details_cfg.mission.scan.use_explicit_scan_bounds = true;
  details_cfg.mission.scan.scan_start_az_deg = -30.0f;
  details_cfg.mission.scan.scan_end_az_deg = 30.0f;
  details_cfg.mission.scan.scan_start_el_deg = -5.0f;
  details_cfg.mission.scan.scan_end_el_deg = 5.0f;
  details_cfg.environment.scenario_config.preset = config::EsrEnvironmentPreset::kLowClutter;

  EXPECT_FLOAT_EQ(details_cfg.policy.detection.minimum_snr_db, 8.0f);
  EXPECT_FLOAT_EQ(details_cfg.policy.detection.pfa, 1.0e-5f);
  EXPECT_EQ(details_cfg.policy.detection.pulse_count, 16U);
  EXPECT_FLOAT_EQ(details_cfg.policy.detection.threshold_scale, 0.9f);
  EXPECT_TRUE(details_cfg.policy.detection.enable_statistical_detection);
  EXPECT_FLOAT_EQ(details_cfg.mission.scan.scan_rate_hz, 4.0f);
  EXPECT_TRUE(details_cfg.mission.scan.use_explicit_scan_bounds);
  EXPECT_EQ(details_cfg.environment.scenario_config.preset,
            config::EsrEnvironmentPreset::kLowClutter);
}

TEST(EsrPublicApiConvenienceTest, SessionConfigCopyPassesThrough) {
  config::EsrSessionConfig baseline;
  baseline.mission.work_mode = config::EsrWorkMode::kHgesm;
  baseline.mission.scan.scan_rate_hz = 0.25f;
  baseline.policy.detection.minimum_snr_db = 12.0f;

  const config::EsrSessionConfig config = baseline;

  // Direct copy preserves fields.
  EXPECT_EQ(config.mission.work_mode, config::EsrWorkMode::kHgesm);
  EXPECT_FLOAT_EQ(config.mission.scan.scan_rate_hz, 0.25f);
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 12.0f);
}

TEST(EsrPublicApiConvenienceTest, SessionConfigValidatorReportsFinalConfigIssues) {
  config::EsrSessionConfig invalid_config;
  invalid_config.mission.scan.scan_rate_hz = 0.0f;
  invalid_config.hardware.receiver_band_lower_hz = 10.0e9;
  invalid_config.hardware.receiver_band_upper_hz = 9.0e9;
  invalid_config.hardware.beam_az_width_deg = 0.0f;
  invalid_config.hardware.beam_el_width_deg = -1.0f;
  invalid_config.mission.scan.use_explicit_scan_bounds = true;
  invalid_config.mission.scan.scan_start_az_deg = 10.0f;
  invalid_config.mission.scan.scan_end_az_deg = 10.0f;
  invalid_config.mission.scan.scan_start_el_deg = 5.0f;
  invalid_config.mission.scan.scan_end_el_deg = 5.0f;

  const session::EsrIssueList issues = config::ValidateEsrSessionConfig(invalid_config);

  ASSERT_EQ(issues.size(), 6U);
  EXPECT_EQ(issues[0].code, "esr.validation.scan_rate_not_positive");
  EXPECT_EQ(issues[1].code, "esr.validation.receiver_band_lower_above_upper");
  EXPECT_EQ(issues[2].code, "esr.validation.beam_az_width_not_positive");
  EXPECT_EQ(issues[3].code, "esr.validation.beam_el_width_not_positive");
  EXPECT_EQ(issues[4].code, "esr.validation.explicit_scan_bounds_az_swapped");
  EXPECT_EQ(issues[5].code, "esr.validation.explicit_scan_bounds_el_swapped");
}

TEST(EsrPublicApiConvenienceTest, RuntimeConfigPatchDefaultsAreUnset) {
  const config::EsrRuntimeConfigPatch patch;

  EXPECT_FALSE(patch.has_mission);
  EXPECT_FALSE(patch.has_policy);
  EXPECT_FALSE(patch.has_environment);
  EXPECT_FALSE(patch.has_sensor_enabled);
  EXPECT_FALSE(patch.has_work_mode);
  EXPECT_FALSE(patch.has_scan_rate_hz);
  EXPECT_FALSE(patch.has_scan_center_az_deg);
  EXPECT_FALSE(patch.has_explicit_scan_bounds);
}

TEST(EsrPublicApiConvenienceTest, RuntimeConfigPatchSetsSemanticFields) {
  config::EsrAtmosphericPhysicsConfig atmospheric_physics;
  atmospheric_physics.enable_physical_model = true;
  atmospheric_physics.relative_humidity = 0.65f;

  config::EsrRuntimeConfigPatch patch;
  patch.has_sensor_enabled = true;
  patch.sensor_enabled = false;
  patch.has_work_mode = true;
  patch.work_mode = config::EsrWorkMode::kHgesm;
  patch.has_scan_rate_hz = true;
  patch.scan_rate_hz = 3.0f;
  patch.has_scan_center_az_deg = true;
  patch.scan_center_az_deg = 15.0f;
  patch.has_scan_center_el_deg = true;
  patch.scan_center_el_deg = 5.0f;
  patch.has_explicit_scan_bounds = true;
  patch.explicit_scan_bounds.enabled = true;
  patch.explicit_scan_bounds.scan_start_az_deg = -30.0f;
  patch.explicit_scan_bounds.scan_end_az_deg = 30.0f;
  patch.explicit_scan_bounds.scan_start_el_deg = -10.0f;
  patch.explicit_scan_bounds.scan_end_el_deg = 10.0f;
  patch.has_environment = true;
  patch.environment.has_atmospheric_physics = true;
  patch.environment.atmospheric_physics = atmospheric_physics;

  EXPECT_TRUE(patch.has_sensor_enabled);
  EXPECT_FALSE(patch.sensor_enabled);
  EXPECT_TRUE(patch.has_work_mode);
  EXPECT_EQ(patch.work_mode, config::EsrWorkMode::kHgesm);
  EXPECT_TRUE(patch.has_scan_rate_hz);
  EXPECT_FLOAT_EQ(patch.scan_rate_hz, 3.0f);
  EXPECT_TRUE(patch.has_scan_center_az_deg);
  EXPECT_TRUE(patch.has_scan_center_el_deg);
  EXPECT_TRUE(patch.has_explicit_scan_bounds);
  EXPECT_TRUE(patch.explicit_scan_bounds.enabled);
  EXPECT_TRUE(patch.has_environment);
  EXPECT_TRUE(patch.environment.has_atmospheric_physics);
  EXPECT_TRUE(patch.environment.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(patch.environment.atmospheric_physics.relative_humidity, 0.65f);
}

TEST(EsrPublicApiConvenienceTest, RuntimeConfigPatchSupportsDomainOverrides) {
  config::EsrMissionConfig mission;
  mission.work_mode = config::EsrWorkMode::kRwr;
  mission.scan.scan_rate_hz = 5.0f;

  config::EsrPolicyConfig policy;
  policy.detection.minimum_snr_db = 5.0f;

  config::EsrRuntimeConfigPatch patch;
  patch.has_mission = true;
  patch.mission = mission;
  patch.has_policy = true;
  patch.policy = policy;

  EXPECT_TRUE(patch.has_mission);
  EXPECT_EQ(patch.mission.work_mode, config::EsrWorkMode::kRwr);
  EXPECT_FLOAT_EQ(patch.mission.scan.scan_rate_hz, 5.0f);
  EXPECT_TRUE(patch.has_policy);
  EXPECT_FLOAT_EQ(patch.policy.detection.minimum_snr_db, 5.0f);
}

TEST(EsrPublicApiConvenienceTest, InputValidationReportsErrorsForBoundaryCases) {
  session::EsrCycleInput input;
  input.dt_sec = 0.0f;
  input.platform_attitude_deg.yaw_deg = std::numeric_limits<double>::infinity();

  input.platform_entity_id = 0U;

  const session::EsrIssueList issues = session::ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsEsrIssueCode(issues, "esr.validation.invalid_cycle_delta_time"));
  EXPECT_TRUE(
      ContainsEsrIssueCode(issues, "esr.validation.non_finite_platform_numeric_field"));
  EXPECT_TRUE(session::HasValidationError(issues));
}

TEST(EsrPublicApiConvenienceTest, SessionStepAndRuntimePatchWorkTogether) {
  session::EsrSession session = session::EsrSession::Create(MakeSessionConfig());

  session::EsrCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 10.0;
  input.dt_sec = 1.0f;
  input.platform_entity_id = 1U;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;

  const session::EsrCycleResult baseline = session.StepWithResult(input);
  EXPECT_EQ(baseline.status, session::EsrCycleExecutionStatus::kCompleted);

  config::EsrRuntimeConfigPatch patch;
  patch.has_sensor_enabled = true;
  patch.sensor_enabled = false;
  (void)session.TryApplyRuntimeConfig(patch);

  const session::EsrCycleResult updated = session.StepWithResult(input);
  EXPECT_TRUE(updated.output_frame.observation_output.observations.empty());
}

TEST(EsrPublicApiConvenienceTest, TryApplyRuntimeConfigExposesRejectFeedback) {
  session::EsrSession session = session::EsrSession::Create(MakeSessionConfig());

  config::EsrRuntimeConfigPatch invalid_patch;
  invalid_patch.has_explicit_scan_bounds = true;
  invalid_patch.explicit_scan_bounds.enabled = true;
  invalid_patch.explicit_scan_bounds.scan_start_az_deg = std::numeric_limits<float>::quiet_NaN();
  invalid_patch.explicit_scan_bounds.scan_end_az_deg = 10.0f;
  invalid_patch.explicit_scan_bounds.scan_start_el_deg = -10.0f;
  invalid_patch.explicit_scan_bounds.scan_end_el_deg = 10.0f;

  const session::EsrRuntimeConfigApplyResult invalid_result =
      session.ApplyRuntimeConfigWithResult(invalid_patch);
  EXPECT_FALSE(invalid_result.applied);
  EXPECT_TRUE(invalid_result.has_requested_update);
  EXPECT_EQ(invalid_result.status,
            session::EsrRuntimeConfigApplyStatus::kRejectedInvalidExplicitScanBounds);
  EXPECT_FALSE(session.TryApplyRuntimeConfig(invalid_patch));

  config::EsrRuntimeConfigPatch valid_patch;
  valid_patch.has_sensor_enabled = true;
  valid_patch.sensor_enabled = false;
  const session::EsrRuntimeConfigApplyResult valid_result =
      session.ApplyRuntimeConfigWithResult(valid_patch);
  EXPECT_TRUE(valid_result.applied);
  EXPECT_TRUE(valid_result.has_requested_update);
  EXPECT_EQ(valid_result.status, session::EsrRuntimeConfigApplyStatus::kApplied);
  EXPECT_TRUE(session.TryApplyRuntimeConfig(valid_patch));
}

TEST(EsrPublicApiConvenienceTest, CreateWithDiagnosticsBuildsSessionAndReportsNoIssuesForHealthyConfig) {
  config::EsrSessionConfig config;

  session::EsrIssueList issues;
  const session::EsrSession session = session::EsrSession::CreateWithDiagnostics(config, &issues);

  EXPECT_TRUE(issues.empty());
  (void)session;
}

TEST(EsrPublicApiConvenienceTest, CreateWithDiagnosticsReportsIssuesButStillConstructsSession) {
  config::EsrSessionConfig invalid;
  invalid.mission.scan.scan_rate_hz = 0.0f;

  session::EsrIssueList issues;
  const session::EsrSession session = session::EsrSession::CreateWithDiagnostics(invalid, &issues);

  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues.front().code, "esr.validation.scan_rate_not_positive");
  (void)session;  // 会话仍被构造，调用方据 issues 决策
}

TEST(EsrPublicApiConvenienceTest, CreateWithDiagnosticsAcceptsNullIssuesWithoutCrash) {
  config::EsrSessionConfig invalid;
  invalid.mission.scan.scan_rate_hz = 0.0f;

  const session::EsrSession session = session::EsrSession::CreateWithDiagnostics(invalid, nullptr);
  (void)session;  // nullptr 时仅构造，不写回 issues
}

// ESR 属于"不复用组"（COMMON-OQ-5）：Step() 在校验失败时返回默认空帧，不复用上一有效帧。
// 这条 guard 锁定该边界——调用方仅凭 Step() 返回值即可判定本轮无新观测。
TEST(EsrPublicApiConvenienceTest, StepReturnsEmptyFrameOnValidationFailure) {
  session::EsrSession session = session::EsrSession::Create(MakeSessionConfig());

  session::EsrCycleInput valid_input;
  valid_input.cycle_index = 1U;
  valid_input.cycle_start_time_s = 10.0;
  valid_input.dt_sec = 1.0f;
  valid_input.platform_entity_id = 1U;
  valid_input.platform_position_ecef_m.x_m = 6378137.0;
  valid_input.rf_emissions.world_cycle_index = valid_input.cycle_index;
  valid_input.rf_emissions.window_start_time_s = valid_input.cycle_start_time_s;
  valid_input.rf_emissions.window_duration_s = valid_input.dt_sec;

  ASSERT_EQ(session.StepWithResult(valid_input).status,
            session::EsrCycleExecutionStatus::kCompleted);

  // 校验失败：dt_sec 非正。
  session::EsrCycleInput invalid_input = valid_input;
  invalid_input.cycle_index = 2U;
  invalid_input.dt_sec = 0.0f;

  const session::EsrCycleResult rejected_result = session.StepWithResult(invalid_input);
  EXPECT_EQ(rejected_result.status, session::EsrCycleExecutionStatus::kRejected);
  EXPECT_TRUE(session::HasValidationError(rejected_result.issues));

  // Step() 与 StepWithResult().output_frame 一致：默认空帧，cycle_index 为 0（非 2），无复用。
  const session::EsrOutputFrame step_frame = session.Step(invalid_input);
  EXPECT_EQ(step_frame.cycle_index, 0U);
  EXPECT_TRUE(step_frame.observation_output.observations.empty());
  EXPECT_TRUE(step_frame.emitter_output.hypotheses.empty());
  EXPECT_EQ(step_frame.cycle_index, rejected_result.output_frame.cycle_index);
}

TEST(EsrPublicApiConvenienceTest, SetSensorEntityIdCanBeSetAndRestored) {
  session::EsrSession session = session::EsrSession::Create(MakeSessionConfig());
  session.SetSensorEntityId(102U);

  session::EsrCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 10.0;
  input.dt_sec = 1.0f;
  input.platform_entity_id = 1U;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = static_cast<double>(input.dt_sec);

  EXPECT_EQ(session.StepWithResult(input).status, session::EsrCycleExecutionStatus::kCompleted);

  // Restore to default sentinel (0)
  session.SetSensorEntityId(0U);
  input.cycle_index = 2U;
  input.cycle_start_time_s = 11.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  EXPECT_EQ(session.StepWithResult(input).status, session::EsrCycleExecutionStatus::kCompleted);
}

}  // namespace tests
}  // namespace electronic_surveillance_radar
