#include <gtest/gtest.h>

#include "flatbuffers/flatbuffers.h"
#include "1q/electromagnetics/RfScene.h"
#include "electronic_surveillance_radar/session/EsrReplayFlatbufferCodec.h"
#include "electronic_surveillance_radar/session/generated/esr_replay_generated.h"
#include "electronic_surveillance_radar/session/generated/esr_session_replay_generated.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

EsrCycleInput MakeInput() {
  EsrCycleInput input;
  input.cycle_index = 7U;
  input.cycle_start_time_s = 20.0;
  input.dt_sec = 0.5f;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = 2U;
  emission.identity.emission_id = 4U;
  emission.position_ecef_m.x_m = 6378137.0;
  emission.position_ecef_m.y_m = 1000.0;
  emission.antenna.boresight_ecef.y = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      20.0, 10.0e9, 1.0e6, 100.0, 1.0e-6, 1.0e-3, 10U, 0.0, 9U, 1U,
      &emission.waveform));
  input.rf_emissions.emissions.push_back(emission);
  return input;
}

TEST(EsrReplayCodecRoundtripTest, CycleInputPreservesRfV2Frame) {
  const EsrCycleInput input = MakeInput();
  EsrCycleInput decoded;
  ASSERT_TRUE(DecodeEsrCycleInput(EncodeEsrCycleInput(input), &decoded));
  EXPECT_EQ(decoded.cycle_index, input.cycle_index);
  EXPECT_DOUBLE_EQ(decoded.cycle_start_time_s, input.cycle_start_time_s);
  ASSERT_EQ(decoded.rf_emissions.emissions.size(), 1U);
  EXPECT_EQ(decoded.rf_emissions.emissions.front().identity.sensor_entity_id, 2U);
  EXPECT_EQ(decoded.rf_emissions.emissions.front().identity.emission_id, 4U);
  EXPECT_EQ(decoded.rf_emissions.emissions.front().waveform.kind,
            oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain);
}

// 标识符护栏（2026-08-24 起引入，2026-09-06 升位为 ESRD）：encode 写入 "ESRD"，无标识符、
// 异源标识符以及已废弃的 "ESRC" 缓冲必须显式拒绝，不得静默误解码。
TEST(EsrReplayCodecRoundtripTest, DecodeRejectsMissingOrForeignIdentifierBuffers) {
  const std::string encoded = EncodeEsrCycleInput(MakeInput());
  ASSERT_GE(encoded.size(), 8U);
  EsrCycleInput decoded;
  EXPECT_TRUE(DecodeEsrCycleInput(encoded, &decoded));

  std::string legacy = encoded;
  legacy.erase(4U, 4U);  // 抹掉标识符 = 历史无标识符录制形态
  EXPECT_FALSE(DecodeEsrCycleInput(legacy, &decoded));

  std::string foreign = encoded;
  foreign.replace(4U, 4U, "XXXX");
  EXPECT_FALSE(DecodeEsrCycleInput(foreign, &decoded));

  std::string retired_esrc = encoded;
  retired_esrc.replace(4U, 4U, "ESRC");
  EXPECT_FALSE(DecodeEsrCycleInput(retired_esrc, &decoded));
}

TEST(EsrReplayCodecRoundtripTest, CycleResultPreservesExplicitStatus) {
  EsrCycleResult result;
  result.input_cycle_index = 7U;
  result.output_frame.scan_azimuth_deg = -8.5f;
  result.status = EsrCycleExecutionStatus::kPoweredOff;
  result.abort_reason = EsrPipelineAbortReason::kSensorPoweredOff;
  EsrCycleResult decoded;
  ASSERT_TRUE(DecodeEsrCycleResult(EncodeEsrCycleResult(result), &decoded));
  EXPECT_EQ(decoded.status, EsrCycleExecutionStatus::kPoweredOff);
  EXPECT_EQ(decoded.abort_reason, EsrPipelineAbortReason::kSensorPoweredOff);
  EXPECT_FLOAT_EQ(decoded.output_frame.scan_azimuth_deg, -8.5f);
}

TEST(EsrReplayCodecRoundtripTest, CycleResultPreservesIssueFullFields) {
  // Q-1（审查修复）：非空 issues 全字段往返保真——severity/phase/code/message/
  // location.kind/entity_index/field 任一未同步到 schema/codec 时立即失败；
  // kPlatform 定位（校验实际使用形态）验证非 kGlobal kind 不丢失。
  EsrCycleResult result;
  result.input_cycle_index = 7U;
  result.status = EsrCycleExecutionStatus::kRejected;
  result.abort_reason = EsrPipelineAbortReason::kValidationRejected;

  EsrIssue validation_issue;
  validation_issue.severity = EsrIssueSeverity::kError;
  validation_issue.phase = EsrIssuePhase::kInputValidation;
  validation_issue.code = "esr.validation.invalid_cycle_delta_time";
  validation_issue.message = "cycle delta time must be positive";
  validation_issue.location.kind = oneq::foundation::ValidationLocationKind::kPlatform;
  validation_issue.field = "dt_sec";
  validation_issue.cause = EsrIssueCause::kHardGateFailed;
  result.issues.push_back(validation_issue);

  // kSceneEntity + entity_index 哨兵往返（非 kSceneEntity kind 编码为 -1）。
  EsrIssue execution_issue;
  execution_issue.severity = EsrIssueSeverity::kError;
  execution_issue.phase = EsrIssuePhase::kExecution;
  execution_issue.code = "esr.sensor_powered_off";
  execution_issue.message = "ESR cycle aborted.";
  execution_issue.location.kind = oneq::foundation::ValidationLocationKind::kSceneEntity;
  execution_issue.location.entity_index = 3U;
  execution_issue.field = "emitters";
  execution_issue.cause = EsrIssueCause::kStatisticalGateFailed;
  result.issues.push_back(execution_issue);

  EsrCycleResult decoded;
  ASSERT_TRUE(DecodeEsrCycleResult(EncodeEsrCycleResult(result), &decoded));
  ASSERT_EQ(decoded.issues.size(), 2U);

  const EsrIssue& decoded_validation = decoded.issues[0];
  EXPECT_EQ(decoded_validation.severity, EsrIssueSeverity::kError);
  EXPECT_EQ(decoded_validation.phase, EsrIssuePhase::kInputValidation);
  EXPECT_EQ(decoded_validation.code, "esr.validation.invalid_cycle_delta_time");
  EXPECT_EQ(decoded_validation.message, "cycle delta time must be positive");
  EXPECT_EQ(decoded_validation.location.kind,
            oneq::foundation::ValidationLocationKind::kPlatform);
  EXPECT_EQ(decoded_validation.location.entity_index, static_cast<std::size_t>(-1));
  EXPECT_EQ(decoded_validation.field, "dt_sec");
  EXPECT_EQ(decoded_validation.cause, EsrIssueCause::kHardGateFailed);

  const EsrIssue& decoded_execution = decoded.issues[1];
  EXPECT_EQ(decoded_execution.severity, EsrIssueSeverity::kError);
  EXPECT_EQ(decoded_execution.phase, EsrIssuePhase::kExecution);
  EXPECT_EQ(decoded_execution.code, "esr.sensor_powered_off");
  EXPECT_EQ(decoded_execution.message, "ESR cycle aborted.");
  EXPECT_EQ(decoded_execution.location.kind,
            oneq::foundation::ValidationLocationKind::kSceneEntity);
  EXPECT_EQ(decoded_execution.location.entity_index, 3U);
  EXPECT_EQ(decoded_execution.field, "emitters");
  EXPECT_EQ(decoded_execution.cause, EsrIssueCause::kStatisticalGateFailed);
}

TEST(EsrReplayCodecRoundtripTest,
     UnknownWaveformClassRejectsWithoutMutatingDestination) {
  flatbuffers::FlatBufferBuilder builder;
  esr::replay::EmitterObservationBuilder observation_builder(builder);
  observation_builder.add_observation_id(1U);
  observation_builder.add_quality(
      static_cast<std::int32_t>(EsrObservationQuality::kHigh));
  observation_builder.add_waveform_class(999);
  const auto observation = observation_builder.Finish();
  const std::vector<flatbuffers::Offset<esr::replay::EmitterObservation>>
      observations{observation};
  // 向量创建前置：CreateVector 必须在 ObservationOutputBuilder 打开之前（NotNested 约束）。
  const auto observations_fb = builder.CreateVector(observations);
  esr::replay::ObservationOutputBuilder output_builder(builder);
  output_builder.add_observations(observations_fb);
  const auto observation_output = output_builder.Finish();
  const auto root = esr::replay::CreateEsrOutputFrame(
      builder, 1U, 2U, 0.0f, observation_output, 0);
  // 2026-08-24 起 decode 校验标识符：手构负载须带 ESRC，用例才能测到各自目标
  // 语义（未知枚举拒绝 / 缺字段默认），而非被标识符护栏先行拒绝。
  builder.Finish(root, kEsrReplayFileIdentifier);
  const std::string bytes(
      reinterpret_cast<const char*>(builder.GetBufferPointer()),
      builder.GetSize());

  EsrOutputFrame destination;
  destination.cycle_index = 777U;
  EXPECT_FALSE(DecodeEsrOutputFrame(bytes, &destination));
  EXPECT_EQ(destination.cycle_index, 777U);
  EXPECT_TRUE(destination.observation_output.observations.empty());
}

TEST(EsrReplayCodecRoundtripTest,
     UnknownSessionConfigEnumRejectsWithoutMutatingDestination) {
  flatbuffers::FlatBufferBuilder builder;
  const auto mission =
      esr::replay::CreateEsrMissionConfig(builder, 999, 0);
  builder.Finish(
      esr::replay::CreateEsrSessionConfig(builder, 0, mission, 0, 0));
  const std::string bytes(
      reinterpret_cast<const char*>(builder.GetBufferPointer()),
      builder.GetSize());

  config::EsrSessionConfig destination;
  destination.sensor_enabled = false;
  EXPECT_FALSE(DecodeEsrSessionConfig(bytes, &destination));
  EXPECT_FALSE(destination.sensor_enabled);
  EXPECT_EQ(destination.mission.work_mode, config::EsrWorkMode::kEsm);
}

TEST(EsrReplayCodecRoundtripTest, SessionConfigPreservesSensorEnabled) {
  // 正向 session config 往返锚点（COMMON-OQ-4 字段提升）：
  // 非默认值 false 防 decode 漏读。
  config::EsrSessionConfig config;
  config.sensor_enabled = false;
  config.mission.work_mode = config::EsrWorkMode::kRwr;
  config.mission.scan.scan_rate_hz = 1.0f;
  config.hardware.receiver_band_lower_hz = 8.5e9;
  config.hardware.receiver_band_upper_hz = 10.5e9;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 40.0f;
  config.policy.detection.minimum_snr_db = -40.0f;
  config.policy.detection.pfa = 1.0e-6f;
  config.policy.detection.pulse_count = 16U;
  config.policy.detection.threshold_scale = 1.0f;
  config.environment.scenario_config.preset = config::EsrEnvironmentPreset::kStandard;

  const std::string bytes = EncodeEsrSessionConfig(config);
  ASSERT_FALSE(bytes.empty());

  config::EsrSessionConfig decoded;
  ASSERT_TRUE(DecodeEsrSessionConfig(bytes, &decoded));
  EXPECT_FALSE(decoded.sensor_enabled);
  EXPECT_EQ(decoded.mission.work_mode, config::EsrWorkMode::kRwr);
  EXPECT_FLOAT_EQ(decoded.mission.scan.scan_rate_hz, 1.0f);
}

TEST(EsrReplayCodecRoundtripTest, DecodeRejectsRetiredESRSFileIdentifier) {
  config::EsrSessionConfig config;
  const std::string bytes = EncodeEsrSessionConfig(config);
  ASSERT_GE(bytes.size(), 8U);
  config::EsrSessionConfig decoded;
  EXPECT_TRUE(DecodeEsrSessionConfig(bytes, &decoded));

  std::string retired_esrs = bytes;
  retired_esrs.replace(4U, 4U, "ESRS");
  EXPECT_FALSE(DecodeEsrSessionConfig(retired_esrs, &decoded));
}

TEST(EsrReplayCodecRoundtripTest,
     UnknownRuntimePatchEnumRejectsWithoutMutatingDestination) {
  flatbuffers::FlatBufferBuilder builder;
  esr::replay::EsrRuntimeConfigPatchBuilder patch_builder(builder);
  patch_builder.add_has_work_mode(true);
  patch_builder.add_work_mode(999);
  builder.Finish(patch_builder.Finish());
  const std::string bytes(
      reinterpret_cast<const char*>(builder.GetBufferPointer()),
      builder.GetSize());

  config::EsrRuntimeConfigPatch destination;
  destination.has_sensor_enabled = true;
  destination.sensor_enabled = false;
  EXPECT_FALSE(DecodeEsrRuntimeConfigPatch(bytes, &destination));
  EXPECT_TRUE(destination.has_sensor_enabled);
  EXPECT_FALSE(destination.sensor_enabled);
  EXPECT_FALSE(destination.has_work_mode);
}

TEST(EsrReplayCodecRoundtripTest,
     RuntimePatchEventPreservesStructuredApplyResult) {
  config::EsrRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::EsrWorkMode::kRwr;
  EsrRuntimeConfigApplyResult expected;
  expected.status = EsrRuntimeConfigApplyStatus::kApplied;
  expected.has_requested_update = true;
  expected.applied = true;

  config::EsrRuntimeConfigPatch decoded_patch;
  EsrRuntimeConfigApplyResult decoded_result;
  ASSERT_TRUE(DecodeEsrRuntimeConfigPatchEvent(
      EncodeEsrRuntimeConfigPatchEvent(patch, expected), &decoded_patch,
      &decoded_result));
  EXPECT_TRUE(decoded_patch.has_work_mode);
  EXPECT_EQ(decoded_patch.work_mode, config::EsrWorkMode::kRwr);
  EXPECT_EQ(decoded_result.status, expected.status);
  EXPECT_TRUE(decoded_result.has_requested_update);
  EXPECT_TRUE(decoded_result.applied);
}

TEST(EsrReplayCodecRoundtripTest,
     UnknownRuntimeApplyStatusRejectsEventAtomically) {
  const std::string patch_bytes =
      EncodeEsrRuntimeConfigPatch(config::EsrRuntimeConfigPatch{});
  flatbuffers::FlatBufferBuilder builder;
  const auto payload = builder.CreateVector(
      reinterpret_cast<const std::uint8_t*>(patch_bytes.data()),
      patch_bytes.size());
  const auto result =
      esr::replay::CreateEsrRuntimeConfigApplyResult(builder, 999, true, true);
  builder.Finish(esr::replay::CreateEsrRuntimeConfigPatchEvent(
      builder, payload, result));
  const std::string bytes(
      reinterpret_cast<const char*>(builder.GetBufferPointer()),
      builder.GetSize());

  config::EsrRuntimeConfigPatch destination_patch;
  destination_patch.has_sensor_enabled = true;
  EsrRuntimeConfigApplyResult destination_result;
  destination_result.status =
      EsrRuntimeConfigApplyStatus::kRejectedInvalidPolicy;
  EXPECT_FALSE(DecodeEsrRuntimeConfigPatchEvent(
      bytes, &destination_patch, &destination_result));
  EXPECT_TRUE(destination_patch.has_sensor_enabled);
  EXPECT_EQ(destination_result.status,
            EsrRuntimeConfigApplyStatus::kRejectedInvalidPolicy);
}

TEST(EsrReplayCodecRoundtripTest,
     DeceptionClassFieldRoundtripsCorrectly) {
  EsrOutputFrame frame;
  frame.cycle_index = 1U;
  frame.batch_id = 2U;
  EmitterObservation obs;
  obs.observation_id = 10U;
  obs.quality = EsrObservationQuality::kHigh;
  obs.waveform_class = EsrWaveformClass::kPulse;
  obs.deception_class = EsrDeceptionClass::kLikelyFalseTarget;
  frame.observation_output.observations.push_back(obs);
  EsrOutputFrame decoded;
  ASSERT_TRUE(DecodeEsrOutputFrame(EncodeEsrOutputFrame(frame), &decoded));
  ASSERT_EQ(decoded.observation_output.observations.size(), 1U);
  EXPECT_EQ(decoded.observation_output.observations.front().deception_class,
            EsrDeceptionClass::kLikelyFalseTarget);
}

TEST(EsrReplayCodecRoundtripTest,
     OutputFramePreservesScanAzimuth) {
  EsrOutputFrame frame;
  frame.cycle_index = 1U;
  frame.batch_id = 2U;
  frame.scan_azimuth_deg = -15.5f;
  EsrOutputFrame decoded;
  ASSERT_TRUE(DecodeEsrOutputFrame(EncodeEsrOutputFrame(frame), &decoded));
  EXPECT_EQ(decoded.cycle_index, 1U);
  EXPECT_EQ(decoded.batch_id, 2U);
  EXPECT_FLOAT_EQ(decoded.scan_azimuth_deg, -15.5f);
}

TEST(EsrReplayCodecRoundtripTest,
     UnknownDeceptionClassRejectsWithoutMutatingDestination) {
  flatbuffers::FlatBufferBuilder builder;
  esr::replay::EmitterObservationBuilder observation_builder(builder);
  observation_builder.add_observation_id(1U);
  observation_builder.add_quality(
      static_cast<std::int32_t>(EsrObservationQuality::kHigh));
  observation_builder.add_waveform_class(
      static_cast<std::int32_t>(EsrWaveformClass::kPulse));
  observation_builder.add_deception_class(999);
  const auto observation = observation_builder.Finish();
  const std::vector<flatbuffers::Offset<esr::replay::EmitterObservation>>
      observations{observation};
  // 向量创建前置：CreateVector 必须在 ObservationOutputBuilder 打开之前（NotNested 约束）。
  const auto observations_fb = builder.CreateVector(observations);
  esr::replay::ObservationOutputBuilder output_builder(builder);
  output_builder.add_observations(observations_fb);
  const auto observation_output = output_builder.Finish();
  const auto root = esr::replay::CreateEsrOutputFrame(
      builder, 1U, 2U, 0.0f, observation_output, 0);
  // 2026-08-24 起 decode 校验标识符：手构负载须带 ESRC，用例才能测到各自目标
  // 语义（未知枚举拒绝 / 缺字段默认），而非被标识符护栏先行拒绝。
  builder.Finish(root, kEsrReplayFileIdentifier);
  const std::string bytes(
      reinterpret_cast<const char*>(builder.GetBufferPointer()),
      builder.GetSize());

  EsrOutputFrame destination;
  destination.cycle_index = 777U;
  EXPECT_FALSE(DecodeEsrOutputFrame(bytes, &destination));
  EXPECT_EQ(destination.cycle_index, 777U);
  EXPECT_TRUE(destination.observation_output.observations.empty());
}

// 旧 trace 不含 deception_class 字段时默认解码为 kNone
TEST(EsrReplayCodecRoundtripTest,
     MissingDeceptionClassFieldDefaultsToNone) {
  flatbuffers::FlatBufferBuilder builder;
  esr::replay::EmitterObservationBuilder observation_builder(builder);
  observation_builder.add_observation_id(1U);
  observation_builder.add_quality(
      static_cast<std::int32_t>(EsrObservationQuality::kHigh));
  observation_builder.add_waveform_class(
      static_cast<std::int32_t>(EsrWaveformClass::kPulse));
  // 故意不调用 add_deception_class —— 模拟旧 trace
  const auto observation = observation_builder.Finish();
  const std::vector<flatbuffers::Offset<esr::replay::EmitterObservation>>
      observations{observation};
  // 向量创建前置：CreateVector 必须在 ObservationOutputBuilder 打开之前（NotNested 约束）。
  const auto observations_fb = builder.CreateVector(observations);
  esr::replay::ObservationOutputBuilder output_builder(builder);
  output_builder.add_observations(observations_fb);
  const auto observation_output = output_builder.Finish();
  const auto root = esr::replay::CreateEsrOutputFrame(
      builder, 1U, 2U, 0.0f, observation_output, 0);
  // 2026-08-24 起 decode 校验标识符：手构负载须带 ESRC，用例才能测到各自目标
  // 语义（未知枚举拒绝 / 缺字段默认），而非被标识符护栏先行拒绝。
  builder.Finish(root, kEsrReplayFileIdentifier);
  const std::string bytes(
      reinterpret_cast<const char*>(builder.GetBufferPointer()),
      builder.GetSize());

  EsrOutputFrame decoded;
  ASSERT_TRUE(DecodeEsrOutputFrame(bytes, &decoded));
  ASSERT_EQ(decoded.observation_output.observations.size(), 1U);
  EXPECT_EQ(decoded.observation_output.observations.front().deception_class,
            EsrDeceptionClass::kNone);
}

}  // namespace
}  // namespace session
}  // namespace electronic_surveillance_radar
