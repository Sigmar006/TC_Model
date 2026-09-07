/**
 * @file ar_replay_codec_roundtrip_test.cpp
 * @brief 验证 AR replay FlatBuffers codec 各 payload 的 Encode→Decode round-trip 字段精确保真。
 *
 * 每个测试独立覆盖一种 payload 类型，确保新增字段未同步到 schema/codec 时立即被发现。
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "1q/replay/ReplayTrace.h"
#include "airborne_radar/session/ArReplayFlatbufferCodec.h"
#include "airborne_radar/session/ArReplayCycleRecord.h"
#include "flatbuffers/flatbuffers.h"

namespace airborne_radar {
namespace session {
namespace tests {

// ---------------------------------------------------------------------------
// config::ArSessionConfig（含 environment 域）
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, SessionConfigPreservesAllDomains) {
  config::ArSessionConfig config;
  config.hardware.transmitter.maximum_peak_power_w = 2.5e6f;
  config.hardware.transmitter.maximum_duty_cycle = 0.08f;
  config.hardware.transmitter.maximum_pulse_energy_j = 18.0f;
  config.hardware.transmitter.frequency_plan_hz = {8.0e9, 8.2e9, 8.4e9};
  config.hardware.receiver.cross_polarization_isolation_db = 27.0f;
  config.hardware.receiver.minimum_far_field_range_m = 3.0f;
  config.hardware.receiver.has_co_site_isolation = true;
  config.hardware.receiver.co_site_isolation_db = 111.0f;
  config.hardware.receiver.maximum_linear_input_power_w = 2.0e-4f;
  config.hardware.receiver.preselector_bandwidth_hz = 32.0e6f;
  config.hardware.receiver.interference_observation_jn_gate_db = 8.5f;
  config.hardware.receiver.scene_polarization =
      oneq::electromagnetics::RfScenePolarization::kVertical;
  config.hardware.receiver.co_site_paths = {{41U, 117.0}, {43U, 104.0}};
  // hardware
  config.hardware.transmitter.peak_power_w = 50000.0f;
  config.hardware.transmitter.frequency_hz = 9.5e9f;
  config.hardware.antenna.antenna_length_m = 1.75f;
  config.hardware.antenna.antenna_width_m = 0.85f;
  config.hardware.signal_processing.target_processing_gain_db = 4.0f;
  config.hardware.signal_processing.noise_processing_gain_db = 1.5f;
  config.hardware.signal_processing.clutter_suppression_gain_db = 12.0f;
  config.hardware.signal_processing.jamming_suppression_gain_db = 8.0f;
  // mission
  config.mission.scan_center_deg.az_deg = 15.0f;
  config.mission.scan_center_deg.el_deg = -2.0f;
  // policy
  config.policy.lifecycle.confirm_hits = 2U;
  config.policy.detection.minimum_detection_margin_db = -20.0f;
  config.policy.detection.minimum_snr_db = -8.0f;
  config.policy.detection.pfa = 2.0e-6f;
  config.policy.detection.pulse_count = 24;
  config.policy.lifecycle.max_miss_before_lost = 3U;
  config.policy.tracking.enable_kalman_filter = true;
  config.policy.tracking.kalman_measurement_noise_std = 5.5f;
  config.policy.decision_control.lpi_hold_cycles_after_request = 2U;
  config.policy.decision_control.eccm_hold_cycles_after_request = 3U;
  config.policy.decision_control.lpi_cooldown_cycles_after_release = 4U;
  config.policy.decision_control.eccm_cooldown_cycles_after_release = 5U;
  // sensor_enabled 顶层电源字段（COMMON-OQ-4 字段提升）往返锚点：非默认值防 decode 漏读
  config.sensor_enabled = false;
  // natural environment
  config.environment.scenario_config.atmospheric_physics.enable_physical_model = true;
  config.environment.scenario_config.atmospheric_physics.pressure_hpa = 1010.0f;
  config.environment.scenario_config.atmospheric_physics.temperature_k = 290.0f;
  config.environment.scenario_config.atmospheric_physics.relative_humidity = 0.6f;
  config.environment.scenario_config.vegetation_scatter_physics.enable_physical_model = true;
  config.environment.scenario_config.vegetation_scatter_physics.cover_profile =
      config::VegetationCoverProfile::kSparseWoodland;
  const std::string bytes = EncodeSessionConfigFlatbuffer(config);
  ASSERT_FALSE(bytes.empty());

  config::ArSessionConfig decoded;
  std::string error;
  ASSERT_TRUE(DecodeSessionConfigFlatbuffer(bytes, &decoded, &error)) << error;

  EXPECT_FLOAT_EQ(decoded.hardware.transmitter.maximum_peak_power_w, 2.5e6f);
  EXPECT_FLOAT_EQ(decoded.hardware.transmitter.maximum_duty_cycle, 0.08f);
  EXPECT_FLOAT_EQ(decoded.hardware.transmitter.maximum_pulse_energy_j, 18.0f);
  EXPECT_EQ(decoded.hardware.transmitter.frequency_plan_hz,
            config.hardware.transmitter.frequency_plan_hz);
  EXPECT_FLOAT_EQ(decoded.hardware.receiver.maximum_linear_input_power_w, 2.0e-4f);
  EXPECT_FLOAT_EQ(decoded.hardware.receiver.preselector_bandwidth_hz, 32.0e6f);
  EXPECT_EQ(decoded.hardware.receiver.scene_polarization,
            oneq::electromagnetics::RfScenePolarization::kVertical);
  ASSERT_EQ(decoded.hardware.receiver.co_site_paths.size(), 2U);
  EXPECT_EQ(decoded.hardware.receiver.co_site_paths[0].transmitter_sensor_entity_id, 41U);
  EXPECT_DOUBLE_EQ(decoded.hardware.receiver.co_site_paths[0].isolation_db, 117.0);

  // hardware
  EXPECT_FLOAT_EQ(decoded.hardware.transmitter.peak_power_w, 50000.0f);
  EXPECT_FLOAT_EQ(decoded.hardware.transmitter.frequency_hz, 9.5e9f);
  EXPECT_FLOAT_EQ(decoded.hardware.antenna.antenna_length_m, 1.75f);
  EXPECT_FLOAT_EQ(decoded.hardware.antenna.antenna_width_m, 0.85f);
  EXPECT_FLOAT_EQ(decoded.hardware.signal_processing.target_processing_gain_db, 4.0f);
  EXPECT_FLOAT_EQ(decoded.hardware.signal_processing.noise_processing_gain_db, 1.5f);
  EXPECT_FLOAT_EQ(decoded.hardware.signal_processing.clutter_suppression_gain_db, 12.0f);
  EXPECT_FLOAT_EQ(decoded.hardware.signal_processing.jamming_suppression_gain_db, 8.0f);
  // mission
  EXPECT_FLOAT_EQ(decoded.mission.scan_center_deg.az_deg, 15.0f);
  EXPECT_FLOAT_EQ(decoded.mission.scan_center_deg.el_deg, -2.0f);
  // sensor_enabled 往返锚点（COMMON-OQ-4 字段提升）
  EXPECT_FALSE(decoded.sensor_enabled);
  // policy
  EXPECT_EQ(decoded.policy.lifecycle.confirm_hits, 2U);
  EXPECT_FLOAT_EQ(decoded.policy.detection.minimum_detection_margin_db, -20.0f);
  EXPECT_FLOAT_EQ(decoded.policy.detection.minimum_snr_db, -8.0f);
  EXPECT_FLOAT_EQ(decoded.policy.detection.pfa, 2.0e-6f);
  EXPECT_EQ(decoded.policy.detection.pulse_count, 24);
  EXPECT_EQ(decoded.policy.lifecycle.max_miss_before_lost, 3U);
  EXPECT_TRUE(decoded.policy.tracking.enable_kalman_filter);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.kalman_measurement_noise_std, 5.5f);
  EXPECT_EQ(decoded.policy.decision_control.lpi_hold_cycles_after_request, 2U);
  EXPECT_EQ(decoded.policy.decision_control.eccm_hold_cycles_after_request, 3U);
  EXPECT_EQ(decoded.policy.decision_control.lpi_cooldown_cycles_after_release, 4U);
  EXPECT_EQ(decoded.policy.decision_control.eccm_cooldown_cycles_after_release, 5U);
  // natural environment
  EXPECT_TRUE(decoded.environment.scenario_config.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.pressure_hpa, 1010.0f);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.temperature_k, 290.0f);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.relative_humidity, 0.6f);
  EXPECT_TRUE(decoded.environment.scenario_config.vegetation_scatter_physics.enable_physical_model);
  EXPECT_EQ(decoded.environment.scenario_config.vegetation_scatter_physics.cover_profile,
            config::VegetationCoverProfile::kSparseWoodland);
}

// ---------------------------------------------------------------------------
// ArRuntimeConfigPatch
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, RuntimeConfigPatchPreservesAllFields) {
  config::ArRuntimeConfigPatch patch;
  patch.has_policy = true;
  patch.policy.tracking.enable_kalman_filter = true;
  patch.policy.tracking.kalman_measurement_noise_std = 7.5f;
  patch.policy.lifecycle.confirm_hits = 1U;
  patch.policy.decision_control.lpi_hold_cycles_after_request = 6U;
  patch.policy.decision_control.eccm_hold_cycles_after_request = 7U;
  patch.policy.decision_control.lpi_cooldown_cycles_after_release = 8U;
  patch.policy.decision_control.eccm_cooldown_cycles_after_release = 9U;
  patch.has_scan_center_deg = true;
  patch.scan_center_deg.az_deg = 30.0f;
  patch.scan_center_deg.el_deg = -3.0f;
  patch.has_dwell_center_deg = true;
  patch.dwell_center_deg.az_deg = 31.0f;
  patch.dwell_center_deg.el_deg = -3.5f;
  patch.has_designated_target_id = true;
  patch.designated_external_target_id = 9001U;
  patch.has_designation_duration_cycles = true;
  patch.designation_duration_cycles = 6U;
  patch.has_commanded_beamwidth_enabled = true;
  patch.commanded_beamwidth_enabled = true;
  patch.has_environment = true;
  patch.environment.has_scenario_config = true;
  patch.environment.scenario_config.atmospheric_physics.relative_humidity = 0.45f;

  const std::string bytes = EncodeRuntimeConfigPatchFlatbuffer(patch);
  ASSERT_FALSE(bytes.empty());

  config::ArRuntimeConfigPatch decoded;
  std::string error;
  ASSERT_TRUE(DecodeRuntimeConfigPatchFlatbuffer(bytes, &decoded, &error)) << error;

  EXPECT_TRUE(decoded.has_policy);
  EXPECT_TRUE(decoded.policy.tracking.enable_kalman_filter);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.kalman_measurement_noise_std, 7.5f);
  EXPECT_EQ(decoded.policy.lifecycle.confirm_hits, 1U);
  EXPECT_EQ(decoded.policy.decision_control.lpi_hold_cycles_after_request, 6U);
  EXPECT_EQ(decoded.policy.decision_control.eccm_hold_cycles_after_request, 7U);
  EXPECT_EQ(decoded.policy.decision_control.lpi_cooldown_cycles_after_release, 8U);
  EXPECT_EQ(decoded.policy.decision_control.eccm_cooldown_cycles_after_release, 9U);
  EXPECT_TRUE(decoded.has_scan_center_deg);
  EXPECT_FLOAT_EQ(decoded.scan_center_deg.az_deg, 30.0f);
  EXPECT_FLOAT_EQ(decoded.scan_center_deg.el_deg, -3.0f);
  EXPECT_TRUE(decoded.has_dwell_center_deg);
  EXPECT_FLOAT_EQ(decoded.dwell_center_deg.az_deg, 31.0f);
  EXPECT_TRUE(decoded.has_designated_target_id);
  EXPECT_EQ(decoded.designated_external_target_id, 9001U);
  EXPECT_TRUE(decoded.has_designation_duration_cycles);
  EXPECT_EQ(decoded.designation_duration_cycles, 6U);
  EXPECT_TRUE(decoded.has_commanded_beamwidth_enabled);
  EXPECT_TRUE(decoded.commanded_beamwidth_enabled);
  EXPECT_TRUE(decoded.has_environment);
  EXPECT_TRUE(decoded.environment.has_scenario_config);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.relative_humidity, 0.45f);
}

// ---------------------------------------------------------------------------
// FailureMarker
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, FailureMarkerPreservesAllFields) {
  oneq::replay::ReplayTraceFailure failure;
  failure.error_code = "AR_ASSERT";
  failure.message = "track pool overflow";
  failure.location = "ArController::RunOnce";
  failure.has_cycle_index = true;
  failure.cycle_index = 42U;
  failure.has_sim_time_sec = true;
  failure.sim_time_sec = 42.5;
  failure.diagnostics_payload = "{\"track_count\":128}";

  const std::string bytes = EncodeFailureMarkerFlatbuffer(failure, true, 99U);
  ASSERT_FALSE(bytes.empty());

  oneq::replay::ReplayTraceFailure decoded;
  std::string error;
  ASSERT_TRUE(DecodeFailureMarkerFlatbuffer(bytes, &decoded, &error)) << error;

  EXPECT_EQ(decoded.error_code, "AR_ASSERT");
  EXPECT_EQ(decoded.message, "track pool overflow");
  EXPECT_EQ(decoded.location, "ArController::RunOnce");
  EXPECT_TRUE(decoded.has_cycle_index);
  EXPECT_EQ(decoded.cycle_index, 42U);
  EXPECT_TRUE(decoded.has_sim_time_sec);
  EXPECT_DOUBLE_EQ(decoded.sim_time_sec, 42.5);
  EXPECT_EQ(decoded.diagnostics_payload, "{\"track_count\":128}");
}

TEST(ArReplayCodecRoundtripTest, SingleCycleInputPreservesWorldAndRfFacts) {
  ArCycleInput input;
  input.cycle_index = 71U;
  input.cycle_start_time_s = 12.5;
  input.dt_sec = 0.25;
  input.platform.platform_position_ecef_m.x_m = 6378137.0;
  input.platform.platform_velocity_mps.y_mps = 210.0;
  input.platform.platform_attitude_deg.yaw_deg = 17.25;
  ArTargetInput target;
  target.target_id = 123U;
  target.target_name = "replay-target";
  target.position_x = 1200.0f;
  target.position_y = -3400.0f;
  target.position_z = 500.0f;
  target.velocity_x = 12.0f;
  target.velocity_y = -6.0f;
  target.velocity_z = 8.0f;
  target.rcs = 3.5f;
  target.swerling_type = 2;
  input.targets.push_back(target);
  input.interference.world_cycle_index = 71U;
  input.interference.window_start_time_s = 12.5;
  input.interference.window_duration_s = 0.25;
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity = {700U, 33U};
  emission.position_ecef_m.x_m = 6388137.0;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfLinearSweepWaveform(
      12.5, 0.25, 8.0e9, 10.0e9, 1.0e6, 1000.0, 0.1, &emission.waveform));
  input.interference.emissions.push_back(emission);

  ArCycleInput decoded;
  std::string error;
  ASSERT_TRUE(DecodeCycleInputFlatbuffer(EncodeCycleInputFlatbuffer(input), &decoded, &error))
      << error;
  EXPECT_EQ(decoded.cycle_index, 71U);
  EXPECT_DOUBLE_EQ(decoded.cycle_start_time_s, 12.5);
  EXPECT_DOUBLE_EQ(decoded.platform.platform_attitude_deg.yaw_deg, 17.25);
  ASSERT_EQ(decoded.targets.size(), 1U);
  EXPECT_EQ(decoded.targets.front().target_name, "replay-target");
  EXPECT_FLOAT_EQ(decoded.targets.front().position_x, 1200.0f);
  EXPECT_FLOAT_EQ(decoded.targets.front().position_y, -3400.0f);
  EXPECT_FLOAT_EQ(decoded.targets.front().velocity_z, 8.0f);
  ASSERT_EQ(decoded.interference.emissions.size(), 1U);
  EXPECT_EQ(decoded.interference.emissions.front().identity.sensor_entity_id, 700U);
  EXPECT_EQ(decoded.interference.emissions.front().identity.emission_id, 33U);
  EXPECT_EQ(decoded.interference.emissions.front().waveform.kind,
            oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep);
}

TEST(ArReplayCodecRoundtripTest, SingleCycleRecordPreservesResultAndState) {
  ArCycleReplayRecord record;
  record.result.input_cycle_index = 81U;
  record.result.status = ArCycleStatus::kCompleted;
  record.result.output_frame.cycle_index = 81U;
  // 航迹快照字段往返锚点（识别迁出后保留的非默认字段）：uncertainty trace 与决策分类。
  record.result.output_frame.tracks.resize(1U);
  record.result.output_frame.tracks.front().estimation_uncertainty_trace = 1234.5f;
  record.result.output_frame.tracks.front().target_type = "HIGH_THREAT_FIGHTER";
  record.result.emission_frame.world_cycle_index = 81U;
  ArInterferenceObservation observation;
  observation.observation_id = 91U;
  observation.jammer_to_noise_db = 14.0;
  record.result.interference_observations.push_back(observation);
  record.result.receiver_impairment = ArReceiverImpairment::kSaturated;
  record.result.submitted_commands.push_back(
      ArCommand(ArCommandType::SET_AGILITY_FREQ, ArCommandSource::ECCM));
  ArIssue issue;
  issue.severity = ArIssueSeverity::kWarning;
  issue.phase = ArIssuePhase::kInputValidation;
  issue.code = "ar.validation.negative_rcs";
  issue.location.kind = oneq::foundation::ValidationLocationKind::kSceneEntity;
  issue.location.entity_index = 3U;
  issue.field = "rcs";
  issue.message = "negative rcs";
  issue.cause = ArIssueCause::kRcsLimited;
  record.result.issues.push_back(issue);
  record.result.has_control_profile = true;
  record.result.control_profile.version = 4U;
  record.result.association_quality_metrics.matched_count = 5U;
  record.result.association_quality_metrics.match_rate = 0.75f;
  record.result.applied_decision_source = DecisionControlSource::kExternal;
  // STT 指定航迹状态字段往返锚点（防 decode 漏读）。
  record.result.effective_work_mode = config::ArWorkMode::kStt;
  record.result.designation_active = true;
  record.result.designated_target_id = 9001U;
  record.result.designation_reverted_to_tws = false;
  record.result.designation_revert_reason = session::ArDesignationRevertReason::kNone;
  record.session_state.has_world_chronology = true;
  record.session_state.last_world_window_end_s = 21.5;
  record.session_state.next_emission_id = 34U;
  record.session_state.successful_prepare_count = 7U;
  record.session_state.frequency_hop_index = 2U;
  record.session_state.has_pending_runtime_update = true;

  ArCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(DecodeCycleReplayRecordFlatbuffer(EncodeCycleReplayRecordFlatbuffer(record), &decoded,
                                                &error))
      << error;
  EXPECT_EQ(decoded.result.status, ArCycleStatus::kCompleted);
  EXPECT_EQ(decoded.result.receiver_impairment, ArReceiverImpairment::kSaturated);
  ASSERT_EQ(decoded.result.output_frame.tracks.size(), 1U);
  EXPECT_FLOAT_EQ(decoded.result.output_frame.tracks.front().estimation_uncertainty_trace,
                  1234.5f);
  EXPECT_EQ(decoded.result.output_frame.tracks.front().target_type, "HIGH_THREAT_FIGHTER");
  ASSERT_EQ(decoded.result.interference_observations.size(), 1U);
  EXPECT_DOUBLE_EQ(decoded.result.interference_observations.front().jammer_to_noise_db, 14.0);
  ASSERT_EQ(decoded.result.submitted_commands.size(), 1U);
  EXPECT_EQ(decoded.result.submitted_commands.front().type, ArCommandType::SET_AGILITY_FREQ);
  ASSERT_EQ(decoded.result.issues.size(), 1U);
  // 全字段往返保真（Q-1 审查修复）：phase/severity/code/message/location/field/cause
  // 任一字段未同步到 schema/codec 时此处立即失败。
  const ArIssue& decoded_issue = decoded.result.issues.front();
  EXPECT_EQ(decoded_issue.severity, ArIssueSeverity::kWarning);
  EXPECT_EQ(decoded_issue.phase, ArIssuePhase::kInputValidation);
  EXPECT_EQ(decoded_issue.code, "ar.validation.negative_rcs");
  EXPECT_EQ(decoded_issue.message, "negative rcs");
  EXPECT_EQ(decoded_issue.location.kind, oneq::foundation::ValidationLocationKind::kSceneEntity);
  EXPECT_EQ(decoded_issue.location.entity_index, 3U);
  EXPECT_EQ(decoded_issue.field, "rcs");
  EXPECT_EQ(decoded_issue.cause, ArIssueCause::kRcsLimited);
  EXPECT_EQ(decoded.result.association_quality_metrics.matched_count, 5U);
  EXPECT_EQ(decoded.result.effective_work_mode, config::ArWorkMode::kStt);
  EXPECT_TRUE(decoded.result.designation_active);
  EXPECT_EQ(decoded.result.designated_target_id, 9001U);
  EXPECT_FALSE(decoded.result.designation_reverted_to_tws);
  EXPECT_EQ(decoded.result.designation_revert_reason, session::ArDesignationRevertReason::kNone);
  EXPECT_EQ(decoded.session_state.next_emission_id, 34U);
  EXPECT_TRUE(decoded.session_state.has_pending_runtime_update);
}

TEST(ArReplayCodecRoundtripTest, CycleRecordDesignationRevertStateRoundtripPreserved) {
  ArCycleReplayRecord record;
  record.result.input_cycle_index = 82U;
  record.result.status = ArCycleStatus::kCompleted;
  record.result.effective_work_mode = config::ArWorkMode::kTws;
  record.result.designation_active = false;
  record.result.designated_target_id = 9002U;
  record.result.designation_reverted_to_tws = true;
  record.result.designation_revert_reason = session::ArDesignationRevertReason::kTrackLost;

  ArCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(DecodeCycleReplayRecordFlatbuffer(EncodeCycleReplayRecordFlatbuffer(record), &decoded,
                                                &error))
      << error;
  EXPECT_EQ(decoded.result.effective_work_mode, config::ArWorkMode::kTws);
  EXPECT_FALSE(decoded.result.designation_active);
  EXPECT_EQ(decoded.result.designated_target_id, 9002U);
  EXPECT_TRUE(decoded.result.designation_reverted_to_tws);
  EXPECT_EQ(decoded.result.designation_revert_reason,
            session::ArDesignationRevertReason::kTrackLost);
}

// 航迹归属对照表（信封通道，session_contract.md Attribution 挂载表）往返保真：
// 加性字段，encode→decode 后键/真值 ID/名称逐条不丢；含"未提供真值"哨兵记录。
TEST(ArReplayCodecRoundtripTest, TrackAttributionsRoundtripPreserved) {
  ArCycleReplayRecord record;
  record.result.input_cycle_index = 83U;
  record.result.status = ArCycleStatus::kCompleted;
  record.result.track_attributions.resize(2U);
  record.result.track_attributions[0].association_key = 11U;
  record.result.track_attributions[0].external_target_id = 7U;
  record.result.track_attributions[0].target_name = "truth-a";
  record.result.track_attributions[1].association_key = 12U;
  record.result.track_attributions[1].external_target_id = 0U;
  record.result.track_attributions[1].target_name = "";

  ArCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(DecodeCycleReplayRecordFlatbuffer(EncodeCycleReplayRecordFlatbuffer(record), &decoded,
                                                &error))
      << error;
  ASSERT_EQ(decoded.result.track_attributions.size(), 2U);
  EXPECT_EQ(decoded.result.track_attributions[0].association_key, 11U);
  EXPECT_EQ(decoded.result.track_attributions[0].external_target_id, 7U);
  EXPECT_EQ(decoded.result.track_attributions[0].target_name, "truth-a");
  EXPECT_EQ(decoded.result.track_attributions[1].association_key, 12U);
  EXPECT_EQ(decoded.result.track_attributions[1].external_target_id, 0U);
  EXPECT_TRUE(decoded.result.track_attributions[1].target_name.empty());
}

TEST(ArReplayCodecRoundtripTest, AttemptsPreserveRejectedRuntimeConfigResult) {
  config::ArRuntimeConfigPatch patch;
  patch.has_sensor_enabled = true;
  patch.sensor_enabled = false;
  config::ArRuntimeConfigPatch decoded_patch;
  bool accepted = true;
  std::string error;
  ASSERT_TRUE(DecodeRuntimeConfigAttemptFlatbuffer(
      EncodeRuntimeConfigAttemptFlatbuffer(patch, false), &decoded_patch, &accepted, &error))
      << error;
  EXPECT_TRUE(decoded_patch.has_sensor_enabled);
  EXPECT_FALSE(decoded_patch.sensor_enabled);
  EXPECT_FALSE(accepted);
}

// 验证新增的反欺骗 profile 开关在编码途中不丢失（stage 4 / P4 修复的核心断言）。
TEST(ArReplayCodecRoundtripTest, AntiDeceptionProfileFlagsRoundtripPreserved) {
  ArCycleReplayRecord record;
  record.result.status = ArCycleStatus::kCompleted;
  record.result.input_cycle_index = 1U;
  record.result.has_control_profile = true;
  record.result.control_profile.version = 5U;
  record.result.control_profile.enable_anti_rgpo_leading_edge = true;
  record.result.control_profile.enable_anti_vgpo_acceleration_bound = true;
  record.result.control_profile.enable_anti_false_target_discrimination = true;

  ArCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(DecodeCycleReplayRecordFlatbuffer(EncodeCycleReplayRecordFlatbuffer(record), &decoded,
                                                &error))
      << error;
  EXPECT_EQ(decoded.result.control_profile.version, 5U);
  EXPECT_TRUE(decoded.result.control_profile.enable_anti_rgpo_leading_edge);
  EXPECT_TRUE(decoded.result.control_profile.enable_anti_vgpo_acceleration_bound);
  EXPECT_TRUE(decoded.result.control_profile.enable_anti_false_target_discrimination);
}

// 验证新增的 interference observation 几何与欺骗字段在编码途中不丢失。
TEST(ArReplayCodecRoundtripTest, InterferenceObservationNewFieldsRoundtripPreserved) {
  session::ArInterferenceObservation obs;
  obs.observation_id = 99U;
  obs.deception_class = session::DeceptionClass::kLikelyFalseTarget;
  obs.coherent_emission_count = 5U;
  obs.estimated_slant_range_m = 12500.0;
  obs.has_local_bearings = true;
  obs.estimated_bearing_azimuth_local_deg = -30.0;
  obs.estimated_bearing_elevation_local_deg = 15.0;
  obs.estimated_range_rate_mps = -120.0;
  obs.estimated_carrier_offset_hz = 5000.0;    // VGPO 可观测特征
  obs.estimated_first_pulse_delay_s = 1.2e-6;  // RGPO 可观测特征

  ArCycleReplayRecord record;
  record.result.status = ArCycleStatus::kCompleted;
  record.result.input_cycle_index = 1U;
  record.result.interference_observations.push_back(obs);

  ArCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(DecodeCycleReplayRecordFlatbuffer(EncodeCycleReplayRecordFlatbuffer(record), &decoded,
                                                &error))
      << error;
  ASSERT_EQ(decoded.result.interference_observations.size(), 1U);
  const auto& decoded_obs = decoded.result.interference_observations.front();
  EXPECT_EQ(decoded_obs.observation_id, 99U);
  EXPECT_EQ(decoded_obs.deception_class, session::DeceptionClass::kLikelyFalseTarget);
  EXPECT_EQ(decoded_obs.coherent_emission_count, 5U);
  EXPECT_DOUBLE_EQ(decoded_obs.estimated_slant_range_m, 12500.0);
  EXPECT_TRUE(decoded_obs.has_local_bearings);
  EXPECT_DOUBLE_EQ(decoded_obs.estimated_bearing_azimuth_local_deg, -30.0);
  EXPECT_DOUBLE_EQ(decoded_obs.estimated_bearing_elevation_local_deg, 15.0);
  EXPECT_DOUBLE_EQ(decoded_obs.estimated_range_rate_mps, -120.0);
  EXPECT_DOUBLE_EQ(decoded_obs.estimated_carrier_offset_hz, 5000.0);
  EXPECT_DOUBLE_EQ(decoded_obs.estimated_first_pulse_delay_s, 1.2e-6);
}

// fail-closed：deception_class 枚举越界时必须原子拒绝，不能钳制后接受损坏语义。
TEST(ArReplayCodecRoundtripTest,
     InterferenceObservationRejectsOutOfRangeDeceptionClassWithoutMutation) {
  session::ArInterferenceObservation obs;
  obs.observation_id = 1U;
  // 构造合法观测后篡改 deception_class 为越界值。
  obs.deception_class = static_cast<session::DeceptionClass>(99);
  obs.coherent_emission_count = 2U;
  obs.estimated_slant_range_m = 5000.0;

  ArCycleReplayRecord record;
  record.result.status = ArCycleStatus::kCompleted;
  record.result.input_cycle_index = 1U;
  record.result.interference_observations.push_back(obs);

  ArCycleReplayRecord decoded;
  decoded.result.input_cycle_index = 777U;
  decoded.session_state.next_emission_id = 888U;
  std::string error;
  EXPECT_FALSE(DecodeCycleReplayRecordFlatbuffer(EncodeCycleReplayRecordFlatbuffer(record),
                                                 &decoded, &error));
  EXPECT_EQ(decoded.result.input_cycle_index, 777U);
  EXPECT_EQ(decoded.session_state.next_emission_id, 888U);
}

TEST(ArReplayCodecRoundtripTest, InterferenceObservationRejectsUnknownWaveformKindWithoutMutation) {
  session::ArInterferenceObservation obs;
  obs.observation_id = 1U;
  obs.estimated_waveform_kind = static_cast<oneq::electromagnetics::RfSceneWaveformKind>(99);

  ArCycleReplayRecord record;
  record.result.status = ArCycleStatus::kCompleted;
  record.result.input_cycle_index = 1U;
  record.result.interference_observations.push_back(obs);

  ArCycleReplayRecord decoded;
  decoded.result.input_cycle_index = 777U;
  std::string error;
  EXPECT_FALSE(DecodeCycleReplayRecordFlatbuffer(EncodeCycleReplayRecordFlatbuffer(record),
                                                 &decoded, &error));
  EXPECT_EQ(decoded.result.input_cycle_index, 777U);
}

// ===========================================================================
// Decode 失败路径（null output / 空 payload / 损坏 payload）
// ===========================================================================

TEST(ArReplayCodecRoundtripTest, CycleRecordRejectsPayloadWithoutFileIdentifier) {
  // 规则 15f 落地收回了 ArCycleResultV3 中段字段：抹掉 file_identifier "ARC3" 模拟落地前的
  // 旧 record（彼时 record 面未写入标识符），必须显式拒绝而非按前移后的 field ID 静默错读。
  ArCycleReplayRecord record;
  record.session_state.next_emission_id = 34U;
  std::string encoded = EncodeCycleReplayRecordFlatbuffer(record);
  ASSERT_GE(encoded.size(), 8U);
  const char* identifier = flatbuffers::GetBufferIdentifier(encoded.data());
  encoded[static_cast<std::size_t>(identifier - encoded.data())] = 'X';
  encoded[static_cast<std::size_t>(identifier - encoded.data()) + 1U] = 'X';
  encoded[static_cast<std::size_t>(identifier - encoded.data()) + 2U] = 'X';
  encoded[static_cast<std::size_t>(identifier - encoded.data()) + 3U] = 'X';
  ArCycleReplayRecord decoded;
  std::string error;
  EXPECT_FALSE(DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error));
  EXPECT_FALSE(error.empty());
}

TEST(ArReplayCodecRoundtripTest, CycleRecordRejectsRetiredARC3FileIdentifier) {
  ArCycleReplayRecord record;
  record.session_state.next_emission_id = 34U;
  std::string encoded = EncodeCycleReplayRecordFlatbuffer(record);
  ASSERT_GE(encoded.size(), 8U);
  const char* identifier = flatbuffers::GetBufferIdentifier(encoded.data());
  const std::size_t offset = static_cast<std::size_t>(identifier - encoded.data());
  encoded[offset] = 'A';
  encoded[offset + 1U] = 'R';
  encoded[offset + 2U] = 'C';
  encoded[offset + 3U] = '3';
  ArCycleReplayRecord decoded;
  std::string error;
  EXPECT_FALSE(DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error));
  EXPECT_NE(error.find("ARC4"), std::string::npos);
}

TEST(ArReplayCodecRoundtripTest, SessionConfigRejectsRetiredARSCFileIdentifier) {
  config::ArSessionConfig config;
  std::string encoded = EncodeSessionConfigFlatbuffer(config);
  ASSERT_GE(encoded.size(), 8U);
  const char* identifier = flatbuffers::GetBufferIdentifier(encoded.data());
  const std::size_t offset = static_cast<std::size_t>(identifier - encoded.data());
  encoded[offset] = 'A';
  encoded[offset + 1U] = 'R';
  encoded[offset + 2U] = 'S';
  encoded[offset + 3U] = 'C';
  config::ArSessionConfig decoded;
  std::string error;
  EXPECT_FALSE(DecodeSessionConfigFlatbuffer(encoded, &decoded, &error));
  EXPECT_FALSE(error.empty());
}

TEST(ArReplayCodecRoundtripTest, DecodeSessionConfigRejectsNullAndCorrupted) {
  std::string error;
  EXPECT_FALSE(DecodeSessionConfigFlatbuffer("", nullptr, &error));
  config::ArSessionConfig config;
  EXPECT_FALSE(DecodeSessionConfigFlatbuffer("bad", &config, &error));
}

TEST(ArReplayCodecRoundtripTest, DecodeRuntimeConfigPatchRejectsNullAndCorrupted) {
  std::string error;
  EXPECT_FALSE(DecodeRuntimeConfigPatchFlatbuffer("", nullptr, &error));
  config::ArRuntimeConfigPatch patch;
  EXPECT_FALSE(DecodeRuntimeConfigPatchFlatbuffer("bad", &patch, &error));
}

TEST(ArReplayCodecRoundtripTest, DecodeFailureMarkerRejectsNullAndCorrupted) {
  std::string error;
  EXPECT_FALSE(DecodeFailureMarkerFlatbuffer("", nullptr, &error));
  oneq::replay::ReplayTraceFailure failure;
  EXPECT_FALSE(DecodeFailureMarkerFlatbuffer("bad", &failure, &error));
}

TEST(ArReplayCodecRoundtripTest, ArControlProfilePayloadRoundtripPreservesAllFields) {
  session::ArControlProfile profile;
  profile.version = 42U;
  profile.enable_lpi_power_control = true;
  profile.lpi_power_scale = 0.75f;
  profile.enable_lpi_beamforming = true;
  profile.lpi_dwell_scale = 0.5f;
  profile.enable_agility_frequency = true;
  profile.agility_frequency_hop_phase = 2U;
  profile.enable_sidelobe_canceller = true;
  profile.enable_adaptive_beamforming = true;
  profile.enable_eccm_rejitter = true;
  profile.eccm_burnthrough_gain = 1.5f;
  profile.enable_anti_rgpo_leading_edge = true;
  profile.enable_anti_vgpo_acceleration_bound = true;
  profile.enable_anti_false_target_discrimination = true;

  const std::string payload = EncodeArControlProfileFlatbuffer(profile);
  ASSERT_FALSE(payload.empty());

  session::ArControlProfile decoded;
  std::string error;
  ASSERT_TRUE(DecodeArControlProfileFlatbuffer(payload, &decoded, &error))
      << error;

  EXPECT_EQ(decoded.version, 42U);
  EXPECT_TRUE(decoded.enable_lpi_power_control);
  EXPECT_FLOAT_EQ(decoded.lpi_power_scale, 0.75f);
  EXPECT_TRUE(decoded.enable_lpi_beamforming);
  EXPECT_FLOAT_EQ(decoded.lpi_dwell_scale, 0.5f);
  EXPECT_TRUE(decoded.enable_agility_frequency);
  EXPECT_EQ(decoded.agility_frequency_hop_phase, 2U);
  EXPECT_TRUE(decoded.enable_sidelobe_canceller);
  EXPECT_TRUE(decoded.enable_adaptive_beamforming);
  EXPECT_TRUE(decoded.enable_eccm_rejitter);
  EXPECT_FLOAT_EQ(decoded.eccm_burnthrough_gain, 1.5f);
  EXPECT_TRUE(decoded.enable_anti_rgpo_leading_edge);
  EXPECT_TRUE(decoded.enable_anti_vgpo_acceleration_bound);
  EXPECT_TRUE(decoded.enable_anti_false_target_discrimination);
}

TEST(ArReplayCodecRoundtripTest, DecodeArControlProfileFlatbufferRejectsNullEmptyAndCorrupted) {
  std::string error;
  // null output pointer
  EXPECT_FALSE(DecodeArControlProfileFlatbuffer("payload", nullptr, &error));
  // empty payload
  session::ArControlProfile profile;
  EXPECT_FALSE(DecodeArControlProfileFlatbuffer("", &profile, &error));
  // corrupted payload
  EXPECT_FALSE(DecodeArControlProfileFlatbuffer("bad", &profile, &error));
}

}  // namespace tests
}  // namespace session
}  // namespace airborne_radar
