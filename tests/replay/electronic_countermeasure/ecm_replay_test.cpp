/**
 * @file ecm_replay_test.cpp
 * @brief 验证 ECM FlatBuffers 字段保真和 ReplayTrace 确定性。
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <sstream>
#include <string>

#include "1q/electronic_countermeasure/EcmReplaySession.h"
#include "1q/electronic_countermeasure/EcmRecordingSession.h"
#include "1q/replay/ReplayTrace.h"
#include "electronic_countermeasure/EcmReplayFlatbufferCodec.h"
#include "flatbuffers/flatbuffers.h"
#include "support/oneq_test_temp_dir.h"

namespace electronic_countermeasure {
namespace session {
namespace {

std::string MakeTempTracePath() {
  static unsigned int counter = 0U;
  std::ostringstream stream;
  stream << oneq_test::TempDir() << "oneq-ecm-replay-" << std::time(nullptr) << "-"
         << std::chrono::high_resolution_clock::now().time_since_epoch().count() << "-"
         << counter++ << ".trace";
  return stream.str();
}

EcmCycleInput MakeInput(std::uint32_t cycle_index, bool fresh) {
  EcmCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = 0.5 * static_cast<double>(cycle_index - 1U);
  input.dt_sec = 0.5;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.transmit_antenna.peak_gain_dbi = 10.0;
  input.has_sensor_observation_frame = fresh;
  if (fresh) {
    input.sensor_observation_frame.source_esr_batch_id = static_cast<std::uint64_t>(cycle_index - 1U);
    EcmSensorObservation observation;
    observation.source_hypothesis_id = 42U;
    observation.estimated_center_frequency_hz = 10.0e9;
    observation.estimated_bandwidth_hz = 20.0e6;
    observation.estimated_pri_s = 1.0e-3;
    observation.estimated_pulse_width_s = 1.0e-6;
    observation.center_frequency_std_hz = 1000.0;
    observation.bandwidth_std_hz = 2000.0;
    observation.bearing_std_deg = 1.0;
    observation.threat_score = 0.9f;
    observation.confidence = 0.8f;
    input.sensor_observation_frame.observations.push_back(observation);
  }
  return input;
}

TEST(EcmReplayCodecTest, InputAndResultPreserveProvenanceAndRfSegments) {
  const EcmCycleInput input = MakeInput(2U, true);
  EcmCycleInput decoded_input;
  ASSERT_TRUE(DecodeEcmCycleInput(EncodeEcmCycleInput(input), &decoded_input));
  EXPECT_EQ(decoded_input.cycle_index, 2U);
  EXPECT_DOUBLE_EQ(decoded_input.cycle_start_time_s, 0.5);
  EXPECT_EQ(decoded_input.sensor_observation_frame.source_esr_batch_id, 1U);
  ASSERT_EQ(decoded_input.sensor_observation_frame.observations.size(), 1U);
  EXPECT_DOUBLE_EQ(decoded_input.sensor_observation_frame.observations.front()
                       .estimated_center_frequency_hz,
                   10.0e9);

  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kSweep;
  EcmSession session = EcmSession::Create(config);
  const EcmCycleResult result = session.StepWithResult(input);
  EcmCycleResult decoded_result;
  ASSERT_TRUE(DecodeEcmCycleResult(EncodeEcmCycleResult(result), &decoded_result));
  EXPECT_EQ(decoded_result.status, EcmCycleStatus::kExecuted);
  EXPECT_EQ(decoded_result.source_esr_batch_id, 1U);
  ASSERT_EQ(decoded_result.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(decoded_result.emission_frame.emissions.front().waveform.kind,
            oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep);
  EXPECT_DOUBLE_EQ(decoded_result.thermal_energy_j, result.thermal_energy_j);
}

TEST(EcmReplayCodecTest, CycleInputRejectsRetiredECMCFileIdentifier) {
  const EcmCycleInput input = MakeInput(2U, true);
  std::string encoded = EncodeEcmCycleInput(input);
  ASSERT_GE(encoded.size(), 8U);
  const char* identifier = flatbuffers::GetBufferIdentifier(encoded.data());
  std::size_t offset = static_cast<std::size_t>(identifier - encoded.data());
  encoded[offset] = 'E';
  encoded[offset + 1U] = 'C';
  encoded[offset + 2U] = 'M';
  encoded[offset + 3U] = 'C';
  EcmCycleInput decoded;
  EXPECT_FALSE(DecodeEcmCycleInput(encoded, &decoded));
}

TEST(EcmReplaySessionTest, MultiCycleTraceReplaysDeterministically) {
  const std::string trace_dir = MakeTempTracePath();
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "ecm-replay-test";
  manifest.module = "electronic_countermeasure";
  manifest.scenario_id = "sensor-driven-glide";
  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));
    config::EcmSessionConfig config;
    config.default_technique = EcmTechnique::kSweep;
    EcmRecordingSessionOptions options;
    options.replay_writer = writer;
    EcmRecordingSession session(config, options);
    EXPECT_EQ(session.StepWithResult(MakeInput(2U, true)).status, EcmCycleStatus::kExecuted);
    EXPECT_EQ(session.StepWithResult(MakeInput(3U, false)).status, EcmCycleStatus::kExecuted);
    config::EcmRuntimeConfigPatch patch;
    patch.has_default_technique = true;
    patch.default_technique = EcmTechnique::kBarrage;
    EXPECT_TRUE(session.ApplyRuntimeConfig(patch).applied);
    EXPECT_EQ(session.StepWithResult(MakeInput(4U, false)).status, EcmCycleStatus::kExecuted);
    EXPECT_EQ(writer->Flush(), oneq::replay::ReplayTraceWriteStatus::kSuccess);
  }

  const EcmReplaySessionResult replay = ReplayEcmTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.playback.applied_input_count, 3U);
  EXPECT_EQ(replay.playback.compared_output_count, 3U);
  EXPECT_EQ(replay.playback.applied_runtime_patch_count, 1U);
  EXPECT_FALSE(replay.playback.divergence_found);
}

}  // namespace
}  // namespace session
}  // namespace electronic_countermeasure
