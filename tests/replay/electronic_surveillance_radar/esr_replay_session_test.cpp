#include <gtest/gtest.h>

#include <memory>

#include "1q/electromagnetics/RfScene.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/replay/ReplayTrace.h"
#include "1q/electronic_surveillance_radar/session/EsrReplaySession.h"
#include "1q/electronic_surveillance_radar/session/EsrRecordingSession.h"
#include "electronic_surveillance_radar/session/EsrReplayFlatbufferCodec.h"
#include "support/oneq_test_temp_dir.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

EsrCycleInput MakeWaveformClassInput(
    std::uint32_t cycle_index,
    oneq::electromagnetics::RfSceneWaveformKind waveform_kind) {
  EsrCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = static_cast<double>(cycle_index - 1U);
  input.dt_sec = 1.0f;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = 2U;
  emission.identity.emission_id = cycle_index;
  emission.position_ecef_m.x_m = 6378137.0;
  emission.position_ecef_m.y_m = 1000.0;
  emission.antenna.boresight_ecef.y = -1.0;
  if (waveform_kind ==
      oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain) {
    EXPECT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
        input.cycle_start_time_s, 10.0e9, 1.0e6, 1.0e6, 1.0e-6, 1.0e-3,
        1000U, 0.0, 1U, cycle_index, &emission.waveform));
  } else {
    EXPECT_TRUE(oneq::electromagnetics::TryCreateRfContinuousWaveform(
        input.cycle_start_time_s, input.dt_sec, 10.0e9, 1.0e6, 1.0e6,
        &emission.waveform));
  }
  input.rf_emissions.emissions.push_back(emission);
  return input;
}

EsrCycleInput MakeContinuousInput(std::uint32_t cycle_index,
                                  double center_frequency_hz,
                                  double transmit_power_w) {
  EsrCycleInput input = MakeWaveformClassInput(
      cycle_index, oneq::electromagnetics::RfSceneWaveformKind::kContinuous);
  input.rf_emissions.emissions.clear();
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = 2U;
  emission.identity.emission_id = cycle_index;
  emission.position_ecef_m.x_m = 6378137.0;
  emission.position_ecef_m.y_m = 1000.0;
  emission.antenna.boresight_ecef.y = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfContinuousWaveform(
      input.cycle_start_time_s, input.dt_sec, center_frequency_hz, 1.0e6,
      transmit_power_w, &emission.waveform));
  input.rf_emissions.emissions.push_back(emission);
  return input;
}

std::shared_ptr<oneq::replay::ReplayTraceWriter> MakeWriter(
    const std::string& trace_dir) {
  oneq::replay::ReplayTraceManifest manifest;
  manifest.module = "electronic_surveillance_radar";
  return std::shared_ptr<oneq::replay::ReplayTraceWriter>(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));
}

TEST(EsrReplaySessionTest, ReplaysDirectRfV2Input) {
  const std::string trace_dir = oneq_test::TempDir() + "1q-esr-rf-v2-replay";
  oneq::replay::ReplayTraceManifest manifest;
  manifest.module = "electronic_surveillance_radar";
  std::shared_ptr<oneq::replay::ReplayTraceWriter> writer(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

  config::EsrSessionConfig config;
  config.policy.detection.minimum_snr_db = -100.0f;
  EsrRecordingSessionOptions options;
  options.replay_writer = writer;
  EsrRecordingSession session(config, options);
  EsrCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 10.0;
  input.dt_sec = 1.0f;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = 1U;
  input.rf_emissions.window_start_time_s = 10.0;
  input.rf_emissions.window_duration_s = 1.0;
  ASSERT_EQ(session.StepWithResult(input).status, EsrCycleExecutionStatus::kCompleted);
  writer->Flush();

  const EsrReplaySessionResult replay = ReplayEsrTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
}

TEST(EsrReplaySessionTest, WaveformClassGateContinuesDeterministicallyInReplay) {
  const std::string trace_dir = oneq_test::TempDir() + "1q-esr-waveform-class-replay";
  oneq::replay::ReplayTraceManifest manifest;
  manifest.module = "electronic_surveillance_radar";
  std::shared_ptr<oneq::replay::ReplayTraceWriter> writer(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

  config::EsrSessionConfig config;
  config.hardware.receiver_band_lower_hz = 9.99e9;
  config.hardware.receiver_band_upper_hz = 10.01e9;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 120.0f;
  config.hardware.maximum_linear_input_power_w = 10.0f;
  config.policy.detection.minimum_snr_db = -100.0f;
  config.policy.detection.enable_statistical_detection = false;
  EsrRecordingSessionOptions options;
  options.replay_writer = writer;
  EsrRecordingSession session(config, options);

  const EsrCycleResult pulse_result = session.StepWithResult(MakeWaveformClassInput(
      1U, oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain));
  ASSERT_EQ(pulse_result.status, EsrCycleExecutionStatus::kCompleted);
  ASSERT_EQ(pulse_result.output_frame.emitter_output.hypotheses.size(), 1U);
  const EsrCycleResult continuous_result =
      session.StepWithResult(MakeWaveformClassInput(
          2U, oneq::electromagnetics::RfSceneWaveformKind::kContinuous));
  ASSERT_EQ(continuous_result.status, EsrCycleExecutionStatus::kCompleted);
  ASSERT_EQ(continuous_result.output_frame.emitter_output.hypotheses.size(), 2U);
  EXPECT_NE(continuous_result.output_frame.emitter_output.hypotheses[0].waveform_class,
            continuous_result.output_frame.emitter_output.hypotheses[1].waveform_class);
  writer->Flush();

  const EsrReplaySessionResult replay = ReplayEsrTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.playback.compared_output_count, 2U);
}

TEST(EsrReplaySessionTest,
     RuntimePatchPowerOffAndRecoveryReplayDeterministically) {
  const std::string trace_dir = oneq_test::TempDir() + "1q-esr-runtime-patch-replay";
  const auto writer = MakeWriter(trace_dir);
  config::EsrSessionConfig config;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 120.0f;
  config.policy.detection.enable_statistical_detection = false;
  EsrRecordingSessionOptions options;
  options.replay_writer = writer;
  EsrRecordingSession session(config, options);

  config::EsrRuntimeConfigPatch invalid_policy;
  invalid_policy.has_policy = true;
  invalid_policy.policy = config.policy;
  invalid_policy.policy.detection.pfa = 0.0f;
  const EsrRuntimeConfigApplyResult rejected =
      session.TryApplyRuntimeConfig(invalid_policy);
  ASSERT_FALSE(rejected.applied);
  ASSERT_EQ(rejected.status,
            EsrRuntimeConfigApplyStatus::kRejectedInvalidPolicy);

  config::EsrRuntimeConfigPatch power_off;
  power_off.has_sensor_enabled = true;
  power_off.sensor_enabled = false;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(power_off).applied);
  const EsrCycleResult powered_off =
      session.StepWithResult(MakeContinuousInput(1U, 10.0e9, 1.0e6));
  ASSERT_EQ(powered_off.status, EsrCycleExecutionStatus::kPoweredOff);

  config::EsrRuntimeConfigPatch power_on;
  power_on.has_sensor_enabled = true;
  power_on.sensor_enabled = true;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(power_on).applied);
  const EsrCycleResult recovered =
      session.StepWithResult(MakeContinuousInput(2U, 10.0e9, 1.0e6));
  ASSERT_EQ(recovered.status, EsrCycleExecutionStatus::kCompleted);
  writer->Flush();

  const EsrReplaySessionResult replay = ReplayEsrTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.report.runtime_config_patch_count, 3U);
  EXPECT_EQ(replay.playback.compared_output_count, 2U);
}

TEST(EsrReplaySessionTest, RuntimePatchApplyResultDivergenceFailsReplay) {
  const std::string trace_dir = oneq_test::TempDir() + "1q-esr-runtime-patch-divergence";
  const auto writer = MakeWriter(trace_dir);
  oneq::replay::ReplayTraceEvent config_event;
  config_event.module = "electronic_surveillance_radar";
  config_event.event_type = "session_config";
  config_event.payload_type = "EsrSessionConfig";
  config_event.payload_encoding = "flatbuffers";
  config_event.payload_bytes =
      EncodeEsrSessionConfig(config::EsrSessionConfig{});
  ASSERT_EQ(writer->WriteEvent(config_event),
            oneq::replay::ReplayTraceWriteStatus::kSuccess);

  config::EsrRuntimeConfigPatch invalid_policy;
  invalid_policy.has_policy = true;
  invalid_policy.policy.detection.pfa = 0.0f;
  EsrRuntimeConfigApplyResult incorrect_expected;
  incorrect_expected.status = EsrRuntimeConfigApplyStatus::kApplied;
  incorrect_expected.has_requested_update = true;
  incorrect_expected.applied = true;
  oneq::replay::ReplayTraceEvent patch_event;
  patch_event.module = "electronic_surveillance_radar";
  patch_event.event_type = "runtime_config_patch";
  patch_event.payload_type = "EsrRuntimeConfigPatchEvent";
  patch_event.payload_encoding = "flatbuffers";
  patch_event.payload_bytes =
      EncodeEsrRuntimeConfigPatchEvent(invalid_policy, incorrect_expected);
  ASSERT_EQ(writer->WriteEvent(patch_event),
            oneq::replay::ReplayTraceWriteStatus::kSuccess);
  writer->Flush();

  const EsrReplaySessionResult replay = ReplayEsrTrace(trace_dir);
  EXPECT_FALSE(replay.ok);
  EXPECT_NE(replay.first_error.find("apply-result divergence"),
            std::string::npos);
}

TEST(EsrReplaySessionTest, ReplayEsrTraceContinuesAfterFailureMarker) {
  const std::string trace_dir = oneq_test::TempDir() + "1q-esr-failure-marker-replay";
  const auto writer = MakeWriter(trace_dir);
  EsrRecordingSessionOptions options;
  options.replay_writer = writer;
  EsrRecordingSession session(config::EsrSessionConfig{}, options);

  EsrCycleInput invalid = MakeContinuousInput(1U, 10.0e9, 1.0e6);
  invalid.dt_sec = -1.0f;
  const EsrCycleResult rejected = session.StepWithResult(invalid);
  ASSERT_EQ(rejected.status, EsrCycleExecutionStatus::kRejected);
  ASSERT_TRUE(HasValidationError(rejected.issues));

  const EsrCycleResult recovered =
      session.StepWithResult(MakeContinuousInput(2U, 10.0e9, 1.0e6));
  ASSERT_EQ(recovered.status, EsrCycleExecutionStatus::kCompleted);
  writer->Flush();

  const EsrReplaySessionResult replay = ReplayEsrTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_TRUE(replay.reached_failure_marker);
  EXPECT_EQ(replay.playback.failure_marker_count, 1U);
  EXPECT_EQ(replay.playback.compared_output_count, 2U);
}

TEST(EsrReplaySessionTest,
     SaturationAdvancesTuningPhaseAndReplaysDeterministically) {
  const std::string trace_dir = oneq_test::TempDir() + "1q-esr-saturation-tuning-replay";
  const auto writer = MakeWriter(trace_dir);
  config::EsrSessionConfig config;
  config.hardware.receiver_band_lower_hz = 9.0e9;
  config.hardware.receiver_band_upper_hz = 11.0e9;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 120.0f;
  config.hardware.maximum_linear_input_power_w = 1.0e-8f;
  config.hardware.tuning_plan.push_back(
      config::EsrTuningWindow{9.5e9, 10.0e6, 1U});
  config.hardware.tuning_plan.push_back(
      config::EsrTuningWindow{10.0e9, 10.0e6, 1U});
  config.policy.detection.enable_statistical_detection = false;
  EsrRecordingSessionOptions options;
  options.replay_writer = writer;
  EsrRecordingSession session(config, options);

  const EsrCycleResult saturated =
      session.StepWithResult(MakeContinuousInput(1U, 9.5e9, 1.0e12));
  ASSERT_EQ(saturated.status, EsrCycleExecutionStatus::kCompleted);
  ASSERT_TRUE(saturated.output_frame.observation_output.receiver_saturated);
  ASSERT_DOUBLE_EQ(
      saturated.output_frame.observation_output.receiver_center_frequency_hz,
      9.5e9);

  const EsrCycleResult next =
      session.StepWithResult(MakeContinuousInput(2U, 10.0e9, 1.0e-6));
  ASSERT_EQ(next.status, EsrCycleExecutionStatus::kCompleted);
  ASSERT_DOUBLE_EQ(
      next.output_frame.observation_output.receiver_center_frequency_hz,
      10.0e9);
  writer->Flush();

  const EsrReplaySessionResult replay = ReplayEsrTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.playback.compared_output_count, 2U);
}

TEST(EsrReplaySessionTest,
     DeceptionClassificationReplaysDeterministically) {
  const std::string trace_dir = oneq_test::TempDir() + "1q-esr-deception-replay";
  const auto writer = MakeWriter(trace_dir);
  config::EsrSessionConfig config;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 120.0f;
  config.hardware.receiver_band_lower_hz = 9.99e9;
  config.hardware.receiver_band_upper_hz = 10.01e9;
  config.policy.detection.minimum_snr_db = -100.0f;
  config.policy.detection.enable_statistical_detection = false;
  EsrRecordingSessionOptions options;
  options.replay_writer = writer;
  EsrRecordingSession session(config, options);

  // 构造同一周期内两个正交方位脉冲发射（触发欺骗标注）
  EsrCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 1.0f;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = 1U;
  input.rf_emissions.window_start_time_s = 0.0;
  input.rf_emissions.window_duration_s = 1.0;
  for (std::uint32_t i = 0; i < 2; ++i) {
    oneq::electromagnetics::RfSceneEmission emission;
    emission.identity.sensor_entity_id = 2U;
    emission.identity.emission_id = static_cast<std::uint64_t>(10 + i);
    emission.position_ecef_m.x_m = 6378137.0;
    emission.position_ecef_m.y_m = 1000.0;
    emission.antenna.boresight_ecef.y = -1.0;
    EXPECT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
        input.cycle_start_time_s, 10.0e9, 1.0e6, 1.0e6, 1.0e-6, 1.0e-3,
        1000U, 0.0, 1U, static_cast<std::uint64_t>(10 + i),
        &emission.waveform));
    input.rf_emissions.emissions.push_back(emission);
  }

  const EsrCycleResult result = session.StepWithResult(input);
  ASSERT_EQ(result.status, EsrCycleExecutionStatus::kCompleted);
  // 验证 live session 产生了观测输出（至少一条）
  ASSERT_GE(result.output_frame.observation_output.observations.size(), 1U);
  writer->Flush();

  // 回放验证：deception_class 已纳入比较器，差异应被检出
  const EsrReplaySessionResult replay = ReplayEsrTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_GE(replay.playback.compared_output_count, 1U);
}

// Negative oracle：篡改 trace 中 observation 的 deception_class（kNone → kLikelyFalseTarget），
// 回放时新鲜计算结果（kNone）与篡改后期望值不一致 → 检出 divergence。
// 验证 deception_class 已纳入 replay 比较器且字段级篡改可被发现。
TEST(EsrReplaySessionTest,
     TamperedDeceptionClassCausesReplayDivergence) {
  // 第一步：录制包含 kLikelyFalseTarget 的 trace（与 DeceptionClassificationReplaysDeterministically 相同配置）。
  const std::string src_dir = oneq_test::TempDir() + "1q-esr-deception-tamper-src";
  {
    const auto writer = MakeWriter(src_dir);
    config::EsrSessionConfig config;
    config.hardware.beam_az_width_deg = 120.0f;
    config.hardware.beam_el_width_deg = 120.0f;
    config.hardware.receiver_band_lower_hz = 9.99e9;
    config.hardware.receiver_band_upper_hz = 10.01e9;
    config.policy.detection.minimum_snr_db = -100.0f;
    config.policy.detection.enable_statistical_detection = false;
    EsrRecordingSessionOptions options;
    options.replay_writer = writer;
    EsrRecordingSession session(config, options);

    EsrCycleInput input;
    input.cycle_index = 1U;
    input.cycle_start_time_s = 0.0;
    input.dt_sec = 1.0f;
    input.platform_position_ecef_m.x_m = 6378137.0;
    input.rf_emissions.world_cycle_index = 1U;
    input.rf_emissions.window_start_time_s = 0.0;
    input.rf_emissions.window_duration_s = 1.0;
    for (std::uint32_t i = 0; i < 2; ++i) {
      oneq::electromagnetics::RfSceneEmission emission;
      emission.identity.sensor_entity_id = 2U;
      emission.identity.emission_id = static_cast<std::uint64_t>(10 + i);
      emission.position_ecef_m.x_m = 6378137.0;
      emission.position_ecef_m.y_m = 1000.0;
      emission.antenna.boresight_ecef.y = -1.0;
      EXPECT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
          input.cycle_start_time_s, 10.0e9, 1.0e6, 1.0e6, 1.0e-6, 1.0e-3,
          1000U, 0.0, 1U, static_cast<std::uint64_t>(10 + i),
          &emission.waveform));
      input.rf_emissions.emissions.push_back(emission);
    }
    const EsrCycleResult result = session.StepWithResult(input);
    ASSERT_EQ(result.status, EsrCycleExecutionStatus::kCompleted);
    ASSERT_GE(result.output_frame.observation_output.observations.size(), 1U);
    writer->Flush();
  }

  // 第二步：读取 trace，篡改 cycle_output 中的 deception_class，写入新 trace。
  const std::string tampered_dir = oneq_test::TempDir() + "1q-esr-deception-tamper-modified";
  {
    oneq::replay::ReplayTraceReader reader(src_dir);
    oneq::replay::ReplayTraceManifest manifest;
    manifest.module = "electronic_surveillance_radar";
    oneq::replay::ReplayTraceWriter writer(tampered_dir, manifest, true);

    oneq::replay::ReplayTraceReadEvent read_event;
    while (reader.ReadNextEvent(&read_event) ==
           oneq::replay::ReplayTraceReadStatus::kEvent) {
      oneq::replay::ReplayTraceEvent out_event;
      out_event.module = read_event.module;
      out_event.event_type = read_event.event_type;
      out_event.payload_type = read_event.payload_type;
      out_event.payload_encoding = read_event.payload_encoding;
      out_event.payload_inline = read_event.payload_inline;
      out_event.has_cycle_index = read_event.has_cycle_index;
      out_event.cycle_index = read_event.cycle_index;
      out_event.has_sim_time_sec = read_event.has_sim_time_sec;
      out_event.sim_time_sec = read_event.sim_time_sec;

      if (read_event.event_type == "cycle_output" &&
          read_event.payload_type == "EsrCycleResult") {
        // 篡改：decode → 修改 deception_class → re-encode。
        EsrCycleResult tampered;
        ASSERT_TRUE(DecodeEsrCycleResult(read_event.payload_bytes, &tampered));
        ASSERT_GE(tampered.output_frame.observation_output.observations.size(), 1U);
        // 篡改：将 deception_class 从 kNone 改为 kLikelyFalseTarget，
        // 使回放时新鲜计算结果（kNone）与篡改后期望值不一致 → divergence。
        for (auto& obs :
             tampered.output_frame.observation_output.observations) {
          obs.deception_class = EsrDeceptionClass::kLikelyFalseTarget;
        }
        out_event.payload_bytes = EncodeEsrCycleResult(tampered);
      } else {
        out_event.payload_bytes = read_event.payload_bytes;
      }
      ASSERT_EQ(writer.WriteEvent(out_event),
                oneq::replay::ReplayTraceWriteStatus::kSuccess);
    }
    writer.Flush();
  }

  // 第三步：回放篡改后的 trace，验证 divergence。
  const EsrReplaySessionResult replay = ReplayEsrTrace(tampered_dir);
  EXPECT_FALSE(replay.ok);
  EXPECT_NE(replay.first_error.find("divergence"), std::string::npos)
      << "tampered deception_class should cause replay divergence: "
      << replay.first_error;
}

// Negative oracle：篡改 trace 中 EsrOutputFrame 的 scan_azimuth_deg（+30°），
// 回放时新鲜计算值（原方位）与篡改后期望值不一致 → 检出 divergence。
// 验证 scan_azimuth_deg 已纳入 replay 比较器且字段级篡改可被发现；
// 录制阶段断言方位非零，同时守卫 pipeline 填充链路。
TEST(EsrReplaySessionTest,
     TamperedScanAzimuthCausesReplayDivergence) {
  // 第一步：录制含非零扫描方位的 trace（扫描中心 30° → 首个波束方位 -30°）。
  const std::string src_dir = oneq_test::TempDir() + "1q-esr-scan-az-tamper-src";
  {
    const auto writer = MakeWriter(src_dir);
    config::EsrSessionConfig config;
    config.hardware.beam_az_width_deg = 120.0f;
    config.hardware.beam_el_width_deg = 120.0f;
    config.hardware.receiver_band_lower_hz = 9.99e9;
    config.hardware.receiver_band_upper_hz = 10.01e9;
    config.policy.detection.minimum_snr_db = -100.0f;
    config.policy.detection.enable_statistical_detection = false;
    config.mission.scan.scan_center_az_deg = 30.0f;
    EsrRecordingSessionOptions options;
    options.replay_writer = writer;
    EsrRecordingSession session(config, options);

    EsrCycleInput input;
    input.cycle_index = 1U;
    input.cycle_start_time_s = 0.0;
    input.dt_sec = 1.0f;
    input.platform_position_ecef_m.x_m = 6378137.0;
    input.rf_emissions.world_cycle_index = 1U;
    input.rf_emissions.window_start_time_s = 0.0;
    input.rf_emissions.window_duration_s = 1.0;
    for (std::uint32_t i = 0; i < 2; ++i) {
      oneq::electromagnetics::RfSceneEmission emission;
      emission.identity.sensor_entity_id = 2U;
      emission.identity.emission_id = static_cast<std::uint64_t>(10 + i);
      emission.position_ecef_m.x_m = 6378137.0;
      emission.position_ecef_m.y_m = 1000.0;
      emission.antenna.boresight_ecef.y = -1.0;
      EXPECT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
          input.cycle_start_time_s, 10.0e9, 1.0e6, 1.0e6, 1.0e-6, 1.0e-3,
          1000U, 0.0, 1U, static_cast<std::uint64_t>(10 + i),
          &emission.waveform));
      input.rf_emissions.emissions.push_back(emission);
    }
    const EsrCycleResult result = session.StepWithResult(input);
    ASSERT_EQ(result.status, EsrCycleExecutionStatus::kCompleted);
    ASSERT_NE(result.output_frame.scan_azimuth_deg, 0.0f);
    writer->Flush();
  }

  // 第二步：读取 trace，篡改 cycle_output 中的 scan_azimuth_deg，写入新 trace。
  const std::string tampered_dir = oneq_test::TempDir() + "1q-esr-scan-az-tamper-modified";
  {
    oneq::replay::ReplayTraceReader reader(src_dir);
    oneq::replay::ReplayTraceManifest manifest;
    manifest.module = "electronic_surveillance_radar";
    oneq::replay::ReplayTraceWriter writer(tampered_dir, manifest, true);

    oneq::replay::ReplayTraceReadEvent read_event;
    while (reader.ReadNextEvent(&read_event) ==
           oneq::replay::ReplayTraceReadStatus::kEvent) {
      oneq::replay::ReplayTraceEvent out_event;
      out_event.module = read_event.module;
      out_event.event_type = read_event.event_type;
      out_event.payload_type = read_event.payload_type;
      out_event.payload_encoding = read_event.payload_encoding;
      out_event.payload_inline = read_event.payload_inline;
      out_event.has_cycle_index = read_event.has_cycle_index;
      out_event.cycle_index = read_event.cycle_index;
      out_event.has_sim_time_sec = read_event.has_sim_time_sec;
      out_event.sim_time_sec = read_event.sim_time_sec;

      if (read_event.event_type == "cycle_output" &&
          read_event.payload_type == "EsrCycleResult") {
        // 篡改：decode → scan_azimuth_deg 平移 30° → re-encode。
        EsrCycleResult tampered;
        ASSERT_TRUE(DecodeEsrCycleResult(read_event.payload_bytes, &tampered));
        // 篡改：把扫描方位平移 30°，使回放时新鲜计算值（原方位）与
        // 篡改后期望值不一致 → divergence。
        tampered.output_frame.scan_azimuth_deg += 30.0f;
        out_event.payload_bytes = EncodeEsrCycleResult(tampered);
      } else {
        out_event.payload_bytes = read_event.payload_bytes;
      }
      ASSERT_EQ(writer.WriteEvent(out_event),
                oneq::replay::ReplayTraceWriteStatus::kSuccess);
    }
    writer.Flush();
  }

  // 第三步：回放篡改后的 trace，验证 divergence。
  const EsrReplaySessionResult replay = ReplayEsrTrace(tampered_dir);
  EXPECT_FALSE(replay.ok);
  EXPECT_NE(replay.first_error.find("divergence"), std::string::npos)
      << "tampered scan_azimuth_deg should cause replay divergence: "
      << replay.first_error;
}

TEST(EsrRecordingSessionTest, ValidationFailureUsesInputCycleIndex) {
  const std::string trace_dir = oneq_test::TempDir() + "1q-esr-validation";
  std::shared_ptr<oneq::replay::ReplayTraceWriter> writer = MakeWriter(trace_dir);
  EsrRecordingSessionOptions options;
  options.replay_writer = writer;
  EsrRecordingSession traced(config::EsrSessionConfig{}, options);
  EsrCycleInput input;
  input.cycle_index = 77U;
  input.dt_sec = -1.0f;
  (void)traced.StepWithResult(input);
  ASSERT_EQ(writer->Flush(), oneq::replay::ReplayTraceWriteStatus::kSuccess);

  oneq::replay::ReplayTraceReader reader(trace_dir);
  oneq::replay::ReplayTraceReadEvent event;
  bool saw_rejected_output = false;
  while (reader.ReadNextEvent(&event) == oneq::replay::ReplayTraceReadStatus::kEvent) {
    if (event.event_type != "cycle_output") {
      continue;
    }
    EsrCycleResult result;
    ASSERT_TRUE(DecodeEsrCycleResult(event.payload_bytes, &result));
    if (HasValidationError(result.issues)) {
      saw_rejected_output = true;
      EXPECT_TRUE(event.has_cycle_index);
      EXPECT_EQ(event.cycle_index, 77U);
      EXPECT_EQ(result.input_cycle_index, 77U);
    }
  }
  EXPECT_TRUE(saw_rejected_output);
}

}  // namespace
}  // namespace session
}  // namespace electronic_surveillance_radar
