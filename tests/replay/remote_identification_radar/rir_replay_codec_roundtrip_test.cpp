// Copyright 2026. All Rights Reserved.
//
// @file rir_replay_codec_roundtrip_test.cpp
// @brief RIR replay 编解码字节精确往返测试。
//
// 蓝本：ar_replay_codec_roundtrip_test.cpp 的识别字段段（审计基线 96de367c）；
// replay 逐周期比较语义（浮点容差 1e-5f）由调用方承担，本测试验证编码往返
// 无字段丢失（字节精确 + 逐字段断言）。

#include <gtest/gtest.h>

#include <string>

#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "1q/electromagnetics/RfScene.h"
#include "remote_identification_radar/session/RirReplayCycleRecord.h"
#include "remote_identification_radar/session/RirReplayFlatbufferCodec.h"
#include "remote_identification_radar/session/generated/rir_replay_generated.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleAbortReason;
using session::RirCycleReplayRecord;
using session::RirCycleResult;
using session::RirCycleStatus;
using session::RirDesignationRevertReason;
using session::RirRecognitionCategory;
using session::RirRecognitionState;
using session::RirSessionReplayState;
using session::RirTrackRecognitionOutput;

TEST(RirReplayCodecRoundtripTest, RecognitionFieldsRoundtripPreserved) {
  RirCycleReplayRecord record;

  RirTrackRecognitionOutput output;
  output.association_key = 7U;
  output.result.state = RirRecognitionState::kModelConfirmed;
  output.result.target_category = RirRecognitionCategory::kBallistic;
  output.result.target_model = "BALLISTIC_EXAMPLE_A";
  output.result.confidence = 0.91f;
  output.result.best_score = 0.92f;
  output.result.runner_up_score = 0.81f;
  output.result.feature_scores.rcs_similarity = 0.9f;
  output.result.feature_scores.rcs_quality = 0.8f;
  output.result.feature_scores.motion_similarity = 0.7f;
  output.result.feature_scores.motion_quality = 0.6f;
  output.result.feature_scores.polarization_similarity = 0.5f;
  output.result.feature_scores.polarization_quality = 0.4f;
  output.result.feature_scores.range_profile_similarity = 0.3f;
  output.result.feature_scores.range_profile_quality = 0.2f;
  output.result.valid_feature_mask = 0x0BU;
  output.result.observation_count = 6U;
  output.result.accumulation_sec = 4.5f;
  output.result.database_version = "1.0.0";
  output.result.source_cycle_index = 12U;
  output.result.source_batch_id = 34U;
  record.result.output_frame.recognition_outputs.push_back(output);

  record.result.input_cycle_index = 12U;
  record.result.status = RirCycleStatus::kCompleted;
  record.result.abort_reason = RirCycleAbortReason::kNone;
  record.result.has_recognition_summary = true;
  record.result.recognition_summary.participating_track_count = 2U;
  record.result.recognition_summary.model_confirmed_count = 1U;
  record.result.recognition_summary.rcs_availability_rate = 0.5f;
  record.result.recognition_summary.mean_confidence = 0.91f;
  record.result.recognition_summary.has_ground_truth = true;
  record.result.recognition_summary.category_accuracy = 1.0f;
  record.result.recognition_summary.model_accuracy = 1.0f;
  record.session_state.active_database_version = "1.0.0";
  record.session_state.detection_random_seed = 123U;
  record.result.recognition_summary.dwell_budget.scheduled_dwell_count = 2U;
  record.result.recognition_summary.dwell_budget.executed_dwell_count = 1U;
  record.result.recognition_summary.dwell_budget.dwell_budget_sec = 0.1f;
  record.result.recognition_summary.dwell_budget.dwell_consumed_sec = 0.05f;
  record.result.designated_target_id = 9100U;
  record.result.designation_active = true;
  record.result.designation_reverted_to_scan = false;
  record.result.designation_revert_reason = RirDesignationRevertReason::kNone;
  record.result.dwell_center_deg.az_deg = -56.0f;
  record.result.dwell_center_deg.el_deg = 30.0f;

  const std::string encoded = session::EncodeCycleReplayRecordFlatbuffer(record);
  ASSERT_FALSE(encoded.empty());
  RirCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(session::DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error)) << error;
  // 字节精确往返：任何字段丢失都会 divergence。
  EXPECT_EQ(session::EncodeCycleReplayRecordFlatbuffer(decoded), encoded);

  EXPECT_EQ(decoded.result.input_cycle_index, 12U);
  EXPECT_EQ(decoded.result.status, RirCycleStatus::kCompleted);
  EXPECT_EQ(decoded.result.abort_reason, RirCycleAbortReason::kNone);
  EXPECT_TRUE(decoded.result.has_recognition_summary);
  EXPECT_EQ(decoded.result.recognition_summary.participating_track_count, 2U);
  EXPECT_EQ(decoded.result.recognition_summary.model_confirmed_count, 1U);
  EXPECT_FLOAT_EQ(decoded.result.recognition_summary.rcs_availability_rate, 0.5f);
  EXPECT_FLOAT_EQ(decoded.result.recognition_summary.mean_confidence, 0.91f);
  EXPECT_TRUE(decoded.result.recognition_summary.has_ground_truth);
  EXPECT_FLOAT_EQ(decoded.result.recognition_summary.category_accuracy, 1.0f);
  EXPECT_FLOAT_EQ(decoded.result.recognition_summary.model_accuracy, 1.0f);

  ASSERT_EQ(decoded.result.output_frame.recognition_outputs.size(), 1U);
  const auto& decoded_output = decoded.result.output_frame.recognition_outputs.front();
  EXPECT_EQ(decoded_output.association_key, 7U);
  EXPECT_EQ(decoded_output.result.state, RirRecognitionState::kModelConfirmed);
  EXPECT_EQ(decoded_output.result.target_category, RirRecognitionCategory::kBallistic);
  EXPECT_EQ(decoded_output.result.target_model, "BALLISTIC_EXAMPLE_A");
  EXPECT_FLOAT_EQ(decoded_output.result.confidence, 0.91f);
  EXPECT_FLOAT_EQ(decoded_output.result.best_score, 0.92f);
  EXPECT_FLOAT_EQ(decoded_output.result.runner_up_score, 0.81f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.rcs_similarity, 0.9f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.rcs_quality, 0.8f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.motion_similarity, 0.7f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.motion_quality, 0.6f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.polarization_similarity, 0.5f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.polarization_quality, 0.4f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.range_profile_similarity, 0.3f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.range_profile_quality, 0.2f);
  EXPECT_EQ(decoded_output.result.valid_feature_mask, 0x0BU);
  EXPECT_EQ(decoded_output.result.observation_count, 6U);
  EXPECT_FLOAT_EQ(decoded_output.result.accumulation_sec, 4.5f);
  EXPECT_EQ(decoded_output.result.database_version, "1.0.0");
  EXPECT_EQ(decoded_output.result.source_cycle_index, 12U);
  EXPECT_EQ(decoded_output.result.source_batch_id, 34U);
  EXPECT_EQ(decoded.session_state.active_database_version, "1.0.0");
  EXPECT_EQ(decoded.session_state.detection_random_seed, 123U);
  EXPECT_EQ(decoded.result.recognition_summary.dwell_budget.scheduled_dwell_count, 2U);
  EXPECT_EQ(decoded.result.recognition_summary.dwell_budget.executed_dwell_count, 1U);
  EXPECT_FLOAT_EQ(decoded.result.recognition_summary.dwell_budget.dwell_budget_sec, 0.1f);
  EXPECT_FLOAT_EQ(decoded.result.recognition_summary.dwell_budget.dwell_consumed_sec, 0.05f);
  EXPECT_EQ(decoded.result.designated_target_id, 9100U);
  EXPECT_TRUE(decoded.result.designation_active);
  EXPECT_FALSE(decoded.result.designation_reverted_to_scan);
  EXPECT_EQ(decoded.result.designation_revert_reason, RirDesignationRevertReason::kNone);
  EXPECT_FLOAT_EQ(decoded.result.dwell_center_deg.az_deg, -56.0f);
  EXPECT_FLOAT_EQ(decoded.result.dwell_center_deg.el_deg, 30.0f);
}

TEST(RirReplayCodecRoundtripTest, DefaultStateRoundtripsByteExact) {
  RirCycleReplayRecord record;
  const std::string encoded = session::EncodeCycleReplayRecordFlatbuffer(record);
  ASSERT_FALSE(encoded.empty());
  RirCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(session::DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error)) << error;
  EXPECT_EQ(session::EncodeCycleReplayRecordFlatbuffer(decoded), encoded);
  EXPECT_EQ(decoded.result.status, RirCycleStatus::kRejectedInvalidInput);
  EXPECT_FALSE(decoded.result.has_recognition_summary);
  EXPECT_TRUE(decoded.result.output_frame.recognition_outputs.empty());
  EXPECT_TRUE(decoded.session_state.active_database_version.empty());
}

TEST(RirReplayCodecRoundtripTest, RejectsPayloadWithoutCurrentIdentifier) {
  RirCycleReplayRecord record;
  record.session_state.detection_random_seed = 7U;
  std::string encoded = session::EncodeCycleReplayRecordFlatbuffer(record);
  ASSERT_GE(encoded.size(), 8U);
  // 抹掉 file_identifier "RIR4"，模拟旧版录制（RIR1/RIR2/RIR3 等）：必须显式拒绝而非静默误读。
  const char* identifier = flatbuffers::GetBufferIdentifier(encoded.data());
  encoded[static_cast<std::size_t>(identifier - encoded.data())] = 'X';
  encoded[static_cast<std::size_t>(identifier - encoded.data()) + 1U] = 'X';
  encoded[static_cast<std::size_t>(identifier - encoded.data()) + 2U] = 'X';
  encoded[static_cast<std::size_t>(identifier - encoded.data()) + 3U] = 'X';
  RirCycleReplayRecord decoded;
  std::string error;
  EXPECT_FALSE(session::DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error));
  EXPECT_FALSE(error.empty());
}

TEST(RirReplayCodecRoundtripTest, RejectsLegacyRir3Identifier) {
  RirCycleReplayRecord record;
  record.session_state.detection_random_seed = 7U;
  std::string encoded = session::EncodeCycleReplayRecordFlatbuffer(record);
  ASSERT_GE(encoded.size(), 8U);
  const char* identifier = flatbuffers::GetBufferIdentifier(encoded.data());
  std::memcpy(const_cast<char*>(identifier), "RIR3", 4U);
  RirCycleReplayRecord decoded;
  std::string error;
  EXPECT_FALSE(session::DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error));
  EXPECT_FALSE(error.empty());
}

/// @brief 出口①特征量测全字段往返（含平台位置双路径：携带与缺省记录并存）。
TEST(RirReplayCodecRoundtripTest, FeatureMeasurementsRoundtripPreserved) {
  RirCycleReplayRecord record;
  record.result.status = RirCycleStatus::kCompleted;

  session::RirFeatureMeasurementRecord with_origin;
  with_origin.association_key = 11U;
  with_origin.features.rcs.valid = true;
  with_origin.features.rcs.mean_dbsm = -3.5f;
  with_origin.features.rcs.std_db = 0.7f;
  with_origin.features.rcs.azimuth_variation_db = 4.0f;
  with_origin.features.rcs.elevation_variation_db = 2.0f;
  with_origin.features.rcs.peak_to_valley_db = 6.0f;
  with_origin.features.rcs.aspect_coverage_deg = 25.0f;
  with_origin.features.rcs.quality = 0.8f;
  with_origin.features.motion.valid = true;
  with_origin.features.motion.speed_m_per_s = 300.0f;
  with_origin.features.motion.altitude_m = 3000.0f;
  with_origin.features.motion.acceleration_m_per_s2 = 12.0f;
  with_origin.features.motion.turn_radius_m = 5000.0f;
  with_origin.features.motion.is_straight = false;
  with_origin.features.motion.quality = 0.6f;
  with_origin.features.polarization.valid = true;
  with_origin.features.polarization.energy_difference_db = 2.5f;
  with_origin.features.polarization.relative_difference_db = 1.5f;
  with_origin.features.polarization.energy_sum_db = -6.0f;
  with_origin.features.polarization.quality = 0.7f;
  with_origin.features.range_profile.valid = true;
  with_origin.features.range_profile.length_m = 8.0f;
  with_origin.features.range_profile.peak_count = 3U;
  with_origin.features.range_profile.peak_energy_concentration = 0.75f;
  with_origin.features.range_profile.resolution_m = 33.31f;
  with_origin.features.range_profile.quality = 0.9f;
  with_origin.features.polarization_stats.valid = true;
  with_origin.features.polarization_stats.determinant_mean = 1.25f;
  with_origin.features.polarization_stats.determinant_std = 0.15f;
  with_origin.features.polarization_stats.span_mean = 4.5f;
  with_origin.features.polarization_stats.span_std = 0.25f;
  with_origin.features.polarization_stats.depolarization_mean = 0.2f;
  with_origin.features.polarization_stats.depolarization_std = 0.05f;
  with_origin.features.polarization_stats.psi_mean_deg = 45.0f;
  with_origin.features.polarization_stats.psi_std_deg = 2.5f;
  with_origin.features.polarization_stats.tau_mean_deg = -3.0f;
  with_origin.features.polarization_stats.tau_std_deg = 0.75f;
  with_origin.valid_feature_mask = 0x0FU;
  with_origin.look_az_deg = 12.5f;
  with_origin.look_el_deg = -3.25f;
  with_origin.range_m = 5385.16f;
  with_origin.snr_db = 18.0f;
  with_origin.dwell_sec = 0.05f;
  with_origin.bandwidth_hz = 4.5e6f;
  with_origin.has_platform_position = true;
  with_origin.platform_position.x_m = 6378137.0;
  with_origin.platform_position.y_m = -100.5;
  with_origin.platform_position.z_m = 200.25;
  with_origin.cycle_index = 12U;
  with_origin.batch_id = 34U;
  record.result.output_frame.feature_measurements.push_back(with_origin);

  // 第二条记录：缺省平台位置路径（has=false + 默认分量）。
  session::RirFeatureMeasurementRecord without_origin;
  without_origin.association_key = 12U;
  without_origin.valid_feature_mask = 0x01U;
  without_origin.look_az_deg = -170.0f;
  record.result.output_frame.feature_measurements.push_back(without_origin);

  const std::string encoded = session::EncodeCycleReplayRecordFlatbuffer(record);
  ASSERT_FALSE(encoded.empty());
  RirCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(session::DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error)) << error;
  EXPECT_EQ(session::EncodeCycleReplayRecordFlatbuffer(decoded), encoded);

  ASSERT_EQ(decoded.result.output_frame.feature_measurements.size(), 2U);
  const auto& first = decoded.result.output_frame.feature_measurements[0];
  EXPECT_EQ(first.association_key, 11U);
  EXPECT_TRUE(first.features.rcs.valid);
  EXPECT_FLOAT_EQ(first.features.rcs.mean_dbsm, -3.5f);
  EXPECT_FLOAT_EQ(first.features.rcs.std_db, 0.7f);
  EXPECT_FLOAT_EQ(first.features.rcs.azimuth_variation_db, 4.0f);
  EXPECT_FLOAT_EQ(first.features.rcs.elevation_variation_db, 2.0f);
  EXPECT_FLOAT_EQ(first.features.rcs.peak_to_valley_db, 6.0f);
  EXPECT_FLOAT_EQ(first.features.rcs.aspect_coverage_deg, 25.0f);
  EXPECT_FLOAT_EQ(first.features.rcs.quality, 0.8f);
  EXPECT_TRUE(first.features.motion.valid);
  EXPECT_FLOAT_EQ(first.features.motion.speed_m_per_s, 300.0f);
  EXPECT_FLOAT_EQ(first.features.motion.altitude_m, 3000.0f);
  EXPECT_FLOAT_EQ(first.features.motion.acceleration_m_per_s2, 12.0f);
  EXPECT_FLOAT_EQ(first.features.motion.turn_radius_m, 5000.0f);
  EXPECT_FALSE(first.features.motion.is_straight);
  EXPECT_FLOAT_EQ(first.features.motion.quality, 0.6f);
  EXPECT_TRUE(first.features.polarization.valid);
  EXPECT_FLOAT_EQ(first.features.polarization.energy_difference_db, 2.5f);
  EXPECT_FLOAT_EQ(first.features.polarization.relative_difference_db, 1.5f);
  EXPECT_FLOAT_EQ(first.features.polarization.energy_sum_db, -6.0f);
  EXPECT_FLOAT_EQ(first.features.polarization.quality, 0.7f);
  EXPECT_TRUE(first.features.range_profile.valid);
  EXPECT_FLOAT_EQ(first.features.range_profile.length_m, 8.0f);
  EXPECT_EQ(first.features.range_profile.peak_count, 3U);
  EXPECT_FLOAT_EQ(first.features.range_profile.peak_energy_concentration, 0.75f);
  EXPECT_FLOAT_EQ(first.features.range_profile.resolution_m, 33.31f);
  EXPECT_FLOAT_EQ(first.features.range_profile.quality, 0.9f);
  EXPECT_TRUE(first.features.polarization_stats.valid);
  EXPECT_FLOAT_EQ(first.features.polarization_stats.determinant_mean, 1.25f);
  EXPECT_FLOAT_EQ(first.features.polarization_stats.determinant_std, 0.15f);
  EXPECT_FLOAT_EQ(first.features.polarization_stats.span_mean, 4.5f);
  EXPECT_FLOAT_EQ(first.features.polarization_stats.span_std, 0.25f);
  EXPECT_FLOAT_EQ(first.features.polarization_stats.depolarization_mean, 0.2f);
  EXPECT_FLOAT_EQ(first.features.polarization_stats.depolarization_std, 0.05f);
  EXPECT_FLOAT_EQ(first.features.polarization_stats.psi_mean_deg, 45.0f);
  EXPECT_FLOAT_EQ(first.features.polarization_stats.psi_std_deg, 2.5f);
  EXPECT_FLOAT_EQ(first.features.polarization_stats.tau_mean_deg, -3.0f);
  EXPECT_FLOAT_EQ(first.features.polarization_stats.tau_std_deg, 0.75f);
  EXPECT_EQ(first.valid_feature_mask, 0x0FU);
  EXPECT_FLOAT_EQ(first.look_az_deg, 12.5f);
  EXPECT_FLOAT_EQ(first.look_el_deg, -3.25f);
  EXPECT_FLOAT_EQ(first.range_m, 5385.16f);
  EXPECT_FLOAT_EQ(first.snr_db, 18.0f);
  EXPECT_FLOAT_EQ(first.dwell_sec, 0.05f);
  EXPECT_FLOAT_EQ(first.bandwidth_hz, 4.5e6f);
  EXPECT_TRUE(first.has_platform_position);
  EXPECT_DOUBLE_EQ(first.platform_position.x_m, 6378137.0);
  EXPECT_DOUBLE_EQ(first.platform_position.y_m, -100.5);
  EXPECT_DOUBLE_EQ(first.platform_position.z_m, 200.25);
  EXPECT_EQ(first.cycle_index, 12U);
  EXPECT_EQ(first.batch_id, 34U);

  const auto& second = decoded.result.output_frame.feature_measurements[1];
  EXPECT_EQ(second.association_key, 12U);
  EXPECT_EQ(second.valid_feature_mask, 0x01U);
  EXPECT_FLOAT_EQ(second.look_az_deg, -170.0f);
  EXPECT_FALSE(second.has_platform_position);
  EXPECT_DOUBLE_EQ(second.platform_position.x_m, 0.0);
  EXPECT_DOUBLE_EQ(second.platform_position.y_m, 0.0);
  EXPECT_DOUBLE_EQ(second.platform_position.z_m, 0.0);
}

/// @brief 航迹归属视图全字段往返（结果层产品，双记录）。
TEST(RirReplayCodecRoundtripTest, TrackAttributionsRoundtripPreserved) {
  RirCycleReplayRecord record;
  record.result.status = RirCycleStatus::kCompleted;

  session::RirTrackAttributionRecord first;
  first.association_key = 1U;
  first.external_target_id = 7U;
  first.target_name = "truth-a";
  first.track_status = session::RirTrackLifecycleStatus::kConfirmed;
  first.hit_count = 3U;
  first.position_enu_x_m = 5000.5;
  first.position_enu_y_m = -12.25;
  first.position_enu_z_m = 2000.0;
  first.speed_m_per_s = 150.5;
  record.result.track_attributions.push_back(first);

  session::RirTrackAttributionRecord second;
  second.association_key = 2U;
  second.external_target_id = 0U;  // 真值未提供路径。
  second.track_status = session::RirTrackLifecycleStatus::kLost;
  second.hit_count = 1U;
  second.speed_m_per_s = 0.0;
  record.result.track_attributions.push_back(second);

  const std::string encoded = session::EncodeCycleReplayRecordFlatbuffer(record);
  ASSERT_FALSE(encoded.empty());
  RirCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(session::DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error)) << error;
  EXPECT_EQ(session::EncodeCycleReplayRecordFlatbuffer(decoded), encoded);

  ASSERT_EQ(decoded.result.track_attributions.size(), 2U);
  EXPECT_EQ(decoded.result.track_attributions[0].association_key, 1U);
  EXPECT_EQ(decoded.result.track_attributions[0].external_target_id, 7U);
  EXPECT_EQ(decoded.result.track_attributions[0].target_name, "truth-a");
  EXPECT_EQ(decoded.result.track_attributions[0].track_status,
            session::RirTrackLifecycleStatus::kConfirmed);
  EXPECT_EQ(decoded.result.track_attributions[0].hit_count, 3U);
  EXPECT_DOUBLE_EQ(decoded.result.track_attributions[0].position_enu_x_m, 5000.5);
  EXPECT_DOUBLE_EQ(decoded.result.track_attributions[0].position_enu_y_m, -12.25);
  EXPECT_DOUBLE_EQ(decoded.result.track_attributions[0].position_enu_z_m, 2000.0);
  EXPECT_DOUBLE_EQ(decoded.result.track_attributions[0].speed_m_per_s, 150.5);
  EXPECT_EQ(decoded.result.track_attributions[1].association_key, 2U);
  EXPECT_EQ(decoded.result.track_attributions[1].external_target_id, 0U);
  EXPECT_EQ(decoded.result.track_attributions[1].track_status,
            session::RirTrackLifecycleStatus::kLost);
  EXPECT_TRUE(decoded.result.track_attributions[1].target_name.empty());
}

/// @brief emission_frame 全字段往返（加性 V2 扩展；旧记录缺字段解码为空帧）。
///        两条 emission（2026-08-30 核查 9.2：多驻留周期逐驻留记录）验证
///        数组语义无损——无需 bump 回放标识符（emissions 本就是向量字段）。
TEST(RirReplayCodecRoundtripTest, EmissionFrameRoundtripPreserved) {
  RirCycleReplayRecord record;
  record.result.status = RirCycleStatus::kCompleted;
  record.result.emission_frame.world_cycle_index = 12U;
  record.result.emission_frame.window_start_time_s = 5.5;
  record.result.emission_frame.window_duration_s = 0.05;

  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = 2U;
  emission.identity.emission_id = 7U;
  emission.position_ecef_m.x_m = 1000.0;
  emission.position_ecef_m.y_m = 2000.0;
  emission.position_ecef_m.z_m = 3000.0;
  emission.velocity_ecef_mps.x_mps = 10.0;
  emission.antenna.peak_gain_dbi = 35.0;
  emission.antenna.half_power_beamwidth_deg = 4.0;
  emission.polarization = oneq::electromagnetics::RfScenePolarization::kVertical;
  emission.waveform.kind = oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain;
  emission.waveform.activity_start_time_s = 5.5;
  emission.waveform.activity_duration_s = 0.05;
  emission.waveform.center_frequency_hz = 3.0e9;
  emission.waveform.occupied_bandwidth_hz = 4.5e6;
  emission.waveform.transmit_power_w = 1.0e6;
  emission.waveform.pulse_width_s = 1.3e-5;
  emission.waveform.pulse_repetition_interval_s = 1.0 / 300.0;
  record.result.emission_frame.emissions.push_back(emission);
  // 第二条：同发射机第二驻留（唯一 emission_id + 不同指向）。
  emission.identity.emission_id = 9223372036854775811ULL;
  emission.antenna.boresight_ecef.x = 0.6;
  emission.antenna.boresight_ecef.y = 0.0;
  emission.antenna.boresight_ecef.z = 0.8;
  record.result.emission_frame.emissions.push_back(emission);

  const std::string encoded = session::EncodeCycleReplayRecordFlatbuffer(record);
  ASSERT_FALSE(encoded.empty());
  RirCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(session::DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error)) << error;
  EXPECT_EQ(session::EncodeCycleReplayRecordFlatbuffer(decoded), encoded);

  EXPECT_EQ(decoded.result.emission_frame.world_cycle_index, 12U);
  EXPECT_DOUBLE_EQ(decoded.result.emission_frame.window_start_time_s, 5.5);
  EXPECT_DOUBLE_EQ(decoded.result.emission_frame.window_duration_s, 0.05);
  ASSERT_EQ(decoded.result.emission_frame.emissions.size(), 2U);
  EXPECT_EQ(decoded.result.emission_frame.emissions[0].identity.sensor_entity_id, 2U);
  EXPECT_EQ(decoded.result.emission_frame.emissions[0].identity.emission_id, 7U);
  EXPECT_DOUBLE_EQ(decoded.result.emission_frame.emissions[0].position_ecef_m.x_m, 1000.0);
  EXPECT_DOUBLE_EQ(decoded.result.emission_frame.emissions[0].waveform.transmit_power_w,
                   1.0e6);
  EXPECT_EQ(decoded.result.emission_frame.emissions[0].waveform.kind,
            oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain);
  EXPECT_EQ(decoded.result.emission_frame.emissions[1].identity.emission_id,
            9223372036854775811ULL)
      << "逐驻留唯一化的大 emission_id 应无损往返";
  EXPECT_DOUBLE_EQ(decoded.result.emission_frame.emissions[1].antenna.boresight_ecef.z, 0.8);
}

/// @brief 空 emission_frame 往返（加性 V2 扩展默认值）。
TEST(RirReplayCodecRoundtripTest, EmptyEmissionFrameRoundtripPreserved) {
  RirCycleReplayRecord record;
  record.result.status = RirCycleStatus::kCompleted;
  record.result.input_cycle_index = 3U;

  const std::string encoded = session::EncodeCycleReplayRecordFlatbuffer(record);
  RirCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(session::DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error)) << error;
  EXPECT_TRUE(decoded.result.emission_frame.emissions.empty());
  EXPECT_EQ(decoded.result.emission_frame.world_cycle_index, 0U);
}

TEST(RirReplayCodecRoundtripTest, OutsideSteerableVolumeRevertReasonRoundtrips) {
  RirCycleReplayRecord record;
  record.result.status = RirCycleStatus::kCompleted;
  record.result.input_cycle_index = 8U;
  record.result.designated_target_id = 9400U;
  record.result.designation_active = false;
  record.result.designation_reverted_to_scan = true;
  record.result.designation_revert_reason = RirDesignationRevertReason::kOutsideSteerableVolume;

  const std::string encoded = session::EncodeCycleReplayRecordFlatbuffer(record);
  RirCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(session::DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error)) << error;
  EXPECT_EQ(decoded.result.designation_revert_reason,
            RirDesignationRevertReason::kOutsideSteerableVolume);
  EXPECT_EQ(session::EncodeCycleReplayRecordFlatbuffer(decoded), encoded);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
