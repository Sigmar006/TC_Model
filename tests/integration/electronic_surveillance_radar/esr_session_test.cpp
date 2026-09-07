#include <gtest/gtest.h>

#include <cmath>

#include "1q/electromagnetics/RfScene.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

config::EsrSessionConfig MakeConfig() {
  config::EsrSessionConfig config;
  config.policy.detection.minimum_snr_db = -100.0f;
  config.policy.detection.enable_statistical_detection = false;
  config.hardware.receiver_band_lower_hz = 9.0e9;
  config.hardware.receiver_band_upper_hz = 11.0e9;
  config.hardware.beam_az_width_deg = 180.0f;
  config.hardware.beam_el_width_deg = 180.0f;
  return config;
}

EsrCycleInput MakeInput() {
  EsrCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 10.0;
  input.dt_sec = 1.0f;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  return input;
}

oneq::electromagnetics::RfSceneEmission MakeEmission(std::uint64_t emission_id, double power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = static_cast<std::uint32_t>(100U + emission_id);
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m.x_m = 6378137.0;
  emission.position_ecef_m.y_m = 1000.0;
  emission.antenna.boresight_ecef.y = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      10.0, 1.0, 10.0e9, 1.0e6, power_w, &emission.waveform));
  return emission;
}

TEST(EsrSessionIntegrationTest, StepWithResultConsumesDirectRfV2Frame) {
  EsrCycleInput input = MakeInput();
  input.rf_emissions.emissions.push_back(MakeEmission(1U, 1.0e6));
  const EsrCycleResult result = EsrSession::Create(MakeConfig()).StepWithResult(input);
  EXPECT_EQ(result.status, EsrCycleExecutionStatus::kCompleted);
  EXPECT_FALSE(HasValidationError(result.issues));
  EXPECT_EQ(result.output_frame.cycle_index, input.cycle_index);
  EXPECT_NE(result.abort_reason, EsrPipelineAbortReason::kRfReceiverRejected);
}

TEST(EsrSessionIntegrationTest, NearFieldLinkWithoutCoSitePathRejectsWithoutOutput) {
  EsrCycleInput input = MakeInput();
  oneq::electromagnetics::RfSceneEmission emission = MakeEmission(1U, 1.0e6);
  emission.position_ecef_m = input.platform_position_ecef_m;
  input.rf_emissions.emissions.push_back(emission);
  const EsrCycleResult result = EsrSession::Create(MakeConfig()).StepWithResult(input);
  EXPECT_EQ(result.status, EsrCycleExecutionStatus::kRejected);
  EXPECT_EQ(result.abort_reason, EsrPipelineAbortReason::kRfReceiverRejected);
  EXPECT_EQ(result.output_frame.cycle_index, 0U);
  // 非执行周期默认空帧契约：scan_azimuth_deg 也必须为 0，不得携带历史方位。
  EXPECT_FLOAT_EQ(result.output_frame.scan_azimuth_deg, 0.0f);
}

TEST(EsrSessionIntegrationTest,
     PolarBearingSingularityDoesNotRejectObservablePeer) {
  EsrCycleInput input = MakeInput();
  oneq::electromagnetics::RfSceneEmission polar = MakeEmission(1U, 1.0e6);
  polar.position_ecef_m.x_m += 1000.0;
  polar.position_ecef_m.y_m = 0.0;
  polar.antenna.boresight_ecef.x = -1.0;
  polar.antenna.boresight_ecef.y = 0.0;
  input.rf_emissions.emissions.push_back(polar);
  input.rf_emissions.emissions.push_back(MakeEmission(2U, 1.0e6));

  const EsrCycleResult result =
      EsrSession::Create(MakeConfig()).StepWithResult(input);
  EXPECT_EQ(result.status, EsrCycleExecutionStatus::kCompleted);
  EXPECT_EQ(result.abort_reason, EsrPipelineAbortReason::kNone);
  ASSERT_EQ(result.output_frame.observation_output.observations.size(), 1U);
  EXPECT_LT(
      std::abs(result.output_frame.observation_output.observations.front().aoa_el_deg),
      30.0);
}

TEST(EsrSessionIntegrationTest, PowerOffProducesNoHistoricalOutput) {
  EsrSession session = EsrSession::Create(MakeConfig());
  EsrCycleInput input = MakeInput();
  input.rf_emissions.emissions.push_back(MakeEmission(1U, 1.0e6));
  ASSERT_EQ(session.StepWithResult(input).status, EsrCycleExecutionStatus::kCompleted);
  config::EsrRuntimeConfigPatch power_off;
  power_off.has_sensor_enabled = true;
  power_off.sensor_enabled = false;
  (void)session.TryApplyRuntimeConfig(power_off);
  const EsrCycleResult powered_off = session.StepWithResult(input);
  EXPECT_EQ(powered_off.status, EsrCycleExecutionStatus::kPoweredOff);
  EXPECT_EQ(powered_off.output_frame.cycle_index, 0U);
  // 关机周期默认空帧契约：scan_azimuth_deg 必须为 0，不得复用上次执行方位。
  EXPECT_FLOAT_EQ(powered_off.output_frame.scan_azimuth_deg, 0.0f);
}

TEST(EsrSessionIntegrationTest, BelowThresholdEmissionCarriesInfoExclusionDiagnostic) {
  EsrCycleInput input = MakeInput();
  input.rf_emissions.emissions.push_back(MakeEmission(3U, 1.0e-20));
  const EsrCycleResult result = EsrSession::Create(MakeConfig()).StepWithResult(input);
  // 正常完成周期：排除原因经 issues 承载，不改变执行状态（规则 13b/13c）。
  EXPECT_EQ(result.status, EsrCycleExecutionStatus::kCompleted);
  bool found = false;
  for (const EsrIssue& issue : result.issues) {
    if (issue.code == "esr.emission_below_threshold") {
      found = true;
      EXPECT_EQ(issue.severity, EsrIssueSeverity::kInfo);
    }
  }
  EXPECT_TRUE(found);
}

}  // namespace
}  // namespace session
}  // namespace electronic_surveillance_radar
