// Copyright 2026. All Rights Reserved.
//
// @file rir_replay_session_test.cpp
// @brief RIR 录制会话与 trace 回放闭环测试（record → replay 字节一致）。
//
// 蓝本：tests/replay/airborne_radar/ar_rf_trace_session_test.cpp。

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <sstream>
#include <string>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirRecordingSession.h"
#include "1q/remote_identification_radar/session/RirReplaySession.h"
#include "1q/replay/ReplayTrace.h"
#include "remote_identification_radar/session/RirReplayFlatbufferCodec.h"
#include "support/oneq_test_temp_dir.h"

namespace remote_identification_radar {
namespace session {
namespace {

std::string MakeTraceDir(const char* prefix) {
  std::ostringstream stream;
  stream << oneq_test::TempDir() << prefix << "-" << std::time(nullptr) << "-"
         << std::chrono::high_resolution_clock::now().time_since_epoch().count() << "-"
         << std::rand() << ".trace";
  return stream.str();
}

config::RirSessionConfig MakeIdentifyConfig() {
  config::RirSessionConfig config;
  config.mission.work_mode = config::RirWorkMode::kIdentify;
  config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  config.policy.lifecycle.confirm_hits = 1U;
  return config;
}

RirCycleInput MakeCycleInput(std::uint32_t cycle) {
  RirCycleInput input;
  input.input_cycle_index = cycle;
  input.dt_sec = 0.5;
  input.sim_time_sec = static_cast<float>(cycle - 1U) * 0.5f;
  // 平台 ECEF（30°N, 120°E, 1000 m）：必填且模长为正，否则周期被输入校验拒绝。
  oneq::coordinate::LlaPositionDegM lla{};
  lla.latitude_deg = 30.0;
  lla.longitude_deg = 120.0;
  lla.altitude_m = 1000.0;
  oneq::coordinate::TryLlaToEcef(lla, &input.platform_position);

  RirSceneTarget target;
  target.external_target_id = 7U;
  target.target_name = "target-a";
  target.position_x = 5000.0f;
  target.position_z = 2000.0f;
  target.velocity_x = 100.0f;
  target.rcs = 5.0f;
  target.aspect_rcs_samples.push_back(RirAspectRcsSample{});
  target.polarization_rcs_samples.push_back(RirPolarizationRcsSample{});
  target.range_rcs_scatterers.push_back(RirRangeRcsScatterer{});
  input.scene_targets.push_back(target);
  return input;
}

std::shared_ptr<oneq::replay::ReplayTraceWriter> MakeWriter(const std::string& trace_dir) {
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "rir-single-cycle-v2";
  manifest.module = "remote_identification_radar";
  manifest.scenario_id = "rir-single-cycle-replay-test";
  return std::shared_ptr<oneq::replay::ReplayTraceWriter>(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));
}

TEST(RirReplaySessionTest, SingleCycleInputAndOutputReplayExactly) {
  const std::string trace_dir = MakeTraceDir("oneq-rir-cycle-complete");
  {
    const std::shared_ptr<oneq::replay::ReplayTraceWriter> writer = MakeWriter(trace_dir);
    RirRecordingSessionOptions options;
    options.replay_writer = writer;
    RirRecordingSession recorded(MakeIdentifyConfig(), options);

    const RirCycleResult result = recorded.StepWithResult(MakeCycleInput(1U));
    ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
    ASSERT_EQ(writer->Flush(), oneq::replay::ReplayTraceWriteStatus::kSuccess);
  }

  const RirReplaySessionResult replay = ReplayRirTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.playback.applied_input_count, 1U);
  EXPECT_EQ(replay.playback.compared_output_count, 1U);
  EXPECT_FALSE(replay.playback.divergence_found);
}

TEST(RirReplaySessionTest, RuntimePatchAndSecondCycleReplayExactly) {
  const std::string trace_dir = MakeTraceDir("oneq-rir-cycle-patch");
  {
    const std::shared_ptr<oneq::replay::ReplayTraceWriter> writer = MakeWriter(trace_dir);
    RirRecordingSessionOptions options;
    options.replay_writer = writer;
    RirRecordingSession recorded(MakeIdentifyConfig(), options);

    ASSERT_EQ(recorded.StepWithResult(MakeCycleInput(1U)).status, RirCycleStatus::kCompleted);

    config::RirRuntimeConfigPatch patch;
    patch.has_scan_center = true;
    patch.scan_center_deg.az_deg = 4.0f;
    EXPECT_TRUE(recorded.TryApplyRuntimeConfig(patch));

    ASSERT_EQ(recorded.StepWithResult(MakeCycleInput(2U)).status, RirCycleStatus::kCompleted);
    ASSERT_EQ(writer->Flush(), oneq::replay::ReplayTraceWriteStatus::kSuccess);
  }

  const RirReplaySessionResult replay = ReplayRirTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.playback.applied_input_count, 2U);
  EXPECT_EQ(replay.playback.applied_runtime_patch_count, 1U);
  EXPECT_EQ(replay.playback.compared_output_count, 2U);
}

/// @brief 会话配置 / 运行期补丁 / 周期输入编解码字节精确往返（字段丢失即失败）。
TEST(RirReplaySessionTest, ConfigAndInputPayloadsRoundtripByteExact) {
  config::RirSessionConfig config = MakeIdentifyConfig();
  config.platform_id = 9U;
  config.sensor_enabled = false;
  config.hardware.transmitter.equipment_id = 11U;
  config.hardware.transmitter.bandwidth_hz = 9.0e6f;
  config.hardware.antenna.model_type =
      config::hardware::RirAntennaPatternModelType::kSincPattern;
  config.hardware.antenna.boresight_offset_deg.az_deg = 3.5f;
  config.hardware.receiver.scene_polarization =
      oneq::electromagnetics::RfScenePolarization::kVertical;
  config.hardware.rcs_physics.enable_physical_rcs = true;
  config.hardware.signal_processing.target_processing_gain_db = 2.5f;
  config.orientation.az_min_deg = -45.0f;
  config.mission.scan_sequence = oneq::foundation::ScanSequence::kElevationFirst;
  config.mission.scan_center_deg.el_deg = 6.0f;
  config.policy.detection.random_seed = 77U;
  config.policy.tracking.kalman_noise_diff_coeff = 2.0f;
  config.policy.recognition.enabled = true;
  config.policy.recognition.database_path = "features.sqlite";
  config.environment.vegetation_cover_profile =
      config::RirVegetationCoverProfile::kDeciduousForest;
  config.environment.atmospheric_physics.enable_physical_model = true;

  const std::string encoded_config = EncodeRirSessionConfig(config);
  config::RirSessionConfig decoded_config;
  std::string error;
  ASSERT_TRUE(DecodeRirSessionConfig(encoded_config, &decoded_config, &error)) << error;
  EXPECT_EQ(EncodeRirSessionConfig(decoded_config), encoded_config);
  EXPECT_EQ(decoded_config.platform_id, 9U);
  EXPECT_FALSE(decoded_config.sensor_enabled);
  EXPECT_EQ(decoded_config.policy.recognition.database_path, "features.sqlite");

  config::RirRuntimeConfigPatch patch;
  patch.has_mission = true;
  patch.mission.max_range_m = 250000.0f;
  patch.has_work_mode = true;
  patch.work_mode = config::RirWorkMode::kIdentify;
  patch.has_scan_center = true;
  patch.scan_center_deg.az_deg = -12.0f;
  patch.has_policy = true;
  patch.policy.lifecycle.max_lost_cycles = 9U;
  patch.has_environment = true;
  patch.environment.weather_attenuation_db = 1.5f;
  patch.has_sensor_enabled = true;
  patch.sensor_enabled = false;
  patch.has_designated_target_id = true;
  patch.designated_external_target_id = 7U;
  patch.has_designation_duration_cycles = true;
  patch.designation_duration_cycles = 20U;

  const std::string encoded_patch = EncodeRirRuntimeConfigPatch(patch);
  config::RirRuntimeConfigPatch decoded_patch;
  ASSERT_TRUE(DecodeRirRuntimeConfigPatch(encoded_patch, &decoded_patch, &error)) << error;
  EXPECT_EQ(EncodeRirRuntimeConfigPatch(decoded_patch), encoded_patch);
  EXPECT_EQ(decoded_patch.designated_external_target_id, 7U);
  EXPECT_EQ(decoded_patch.designation_duration_cycles, 20U);

  RirCycleInput input = MakeCycleInput(3U);
  input.scene_targets[0].target_swerling_type = RirSwerlingType::kSwerling3;
  input.scene_targets[0].polarization_rcs_samples[0].has_cross_pol = true;
  input.scene_targets[0].polarization_rcs_samples[0].cross_rcs_dbsm = -12.0f;
  input.scene_targets[0].range_rcs_scatterers[0].range_offset_m = 1.5f;

  const std::string encoded_input = EncodeRirCycleInput(input);
  RirCycleInput decoded_input;
  ASSERT_TRUE(DecodeRirCycleInput(encoded_input, &decoded_input, &error)) << error;
  EXPECT_EQ(EncodeRirCycleInput(decoded_input), encoded_input);
  EXPECT_EQ(decoded_input.input_cycle_index, 3U);
  EXPECT_DOUBLE_EQ(decoded_input.dt_sec, 0.5);
  ASSERT_EQ(decoded_input.scene_targets.size(), 1U);
  EXPECT_EQ(decoded_input.scene_targets[0].target_name, "target-a");
  EXPECT_EQ(decoded_input.scene_targets[0].target_swerling_type, RirSwerlingType::kSwerling3);
  EXPECT_DOUBLE_EQ(decoded_input.platform_position.x_m, input.platform_position.x_m);
}

/// @brief 会话配置载荷拒绝旧标识符：v4（2026-09-06）sensor_platform_id 改名
///        platform_id，RIRE 升 RIRF；v3（2026-08-30）tracking 策略表删除
///        全链未消费的 kalman_measurement_noise_std 槽位，RIRD 升 RIRE；
///        v2（2026-08-30）receiver 表槽位前移，RIRC 升 RIRD。旧录制必须
///        显式拒绝而非静默误读。
TEST(RirReplaySessionTest, SessionConfigDecodeRejectsLegacyIdentifier) {
  const std::string encoded = EncodeRirSessionConfig(MakeIdentifyConfig());
  ASSERT_GE(encoded.size(), 8U);

  for (const char* legacy_identifier : {"RIRC", "RIRD", "RIRE"}) {
    std::string relabeled = encoded;
    relabeled.replace(4U, 4U, legacy_identifier);
    config::RirSessionConfig decoded;
    std::string error;
    EXPECT_FALSE(DecodeRirSessionConfig(relabeled, &decoded, &error)) << legacy_identifier;
    EXPECT_FALSE(error.empty()) << legacy_identifier;
  }
}

}  // namespace
}  // namespace session
}  // namespace remote_identification_radar
