/**
 * @file ecm_session_test.cpp
 * @brief 验证 ECM 双模式、资源调度、滑行和快照确定性契约。
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <functional>
#include <limits>
#include <utility>

#include "1q/electronic_countermeasure/EcmEsrAdapter.h"
#include "1q/electronic_countermeasure/EcmSession.h"

namespace electronic_countermeasure {
namespace session {
namespace {

EcmSensorObservation MakeObservation(std::uint64_t id, double center_hz, float threat_score) {
  EcmSensorObservation observation;
  observation.source_hypothesis_id = id;
  observation.estimated_center_frequency_hz = center_hz;
  observation.estimated_bandwidth_hz = 10.0e6;
  observation.estimated_pri_s = 1.0e-3;
  observation.estimated_pulse_width_s = 1.0e-6;
  observation.center_frequency_std_hz = 1000.0;
  observation.bandwidth_std_hz = 2000.0;
  observation.bearing_std_deg = 1.0;
  observation.threat_score = threat_score;
  observation.confidence = 0.9f;
  return observation;
}

EcmCycleInput MakeSensorInput(std::uint32_t cycle_index, bool has_frame) {
  EcmCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = static_cast<double>(cycle_index - 1U);
  input.dt_sec = 1.0;
  input.input_mode = EcmInputMode::kSensorDriven;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.has_sensor_observation_frame = has_frame;
  if (has_frame) {
    input.sensor_observation_frame.source_esr_batch_id =
        static_cast<std::uint64_t>(cycle_index - 1U);
    input.sensor_observation_frame.observations.push_back(MakeObservation(10U, 10.0e9, 0.9f));
  }
  return input;
}

EcmCycleInput MakeTruthInput(std::uint32_t cycle_index) {
  EcmCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = static_cast<double>(cycle_index - 1U);
  input.dt_sec = 1.0;
  input.input_mode = EcmInputMode::kTruthAssisted;
  input.platform_position_ecef_m.x_m = 6378137.0;
  EcmTruthThreat threat;
  threat.truth_entity_id = 100U;
  threat.center_frequency_hz = 10.0e9;
  threat.bandwidth_hz = 10.0e6;
  threat.threat_score = 0.9f;
  threat.estimated_pri_s = 1.0e-3;
  threat.estimated_pulse_width_s = 1.0e-6;
  input.truth_threats.push_back(threat);
  return input;
}

TEST(EcmSessionTest, SensorFrameGlidesTwoSuccessfulCyclesThenSafelyStops) {
  EcmSession session = EcmSession::Create();
  const EcmCycleResult fresh = session.StepWithResult(MakeSensorInput(2U, true));
  ASSERT_EQ(fresh.status, EcmCycleStatus::kExecuted);
  ASSERT_EQ(fresh.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(fresh.source_esr_batch_id, 1U);

  const EcmCycleResult glide_one = session.StepWithResult(MakeSensorInput(3U, false));
  EXPECT_EQ(glide_one.status, EcmCycleStatus::kExecuted);
  EXPECT_TRUE(glide_one.used_glided_observation);
  EXPECT_EQ(glide_one.observation_age_successful_ecm_cycles, 1U);

  const EcmCycleResult glide_two = session.StepWithResult(MakeSensorInput(4U, false));
  EXPECT_EQ(glide_two.status, EcmCycleStatus::kExecuted);
  EXPECT_EQ(glide_two.observation_age_successful_ecm_cycles, 2U);

  const EcmCycleResult expired = session.StepWithResult(MakeSensorInput(5U, false));
  EXPECT_EQ(expired.status, EcmCycleStatus::kSafeStopNoFreshObservation);
  EXPECT_TRUE(expired.emission_frame.emissions.empty());
}

TEST(EcmSessionTest, RejectedMixedModeDoesNotAdvanceSuccessfulState) {
  EcmSession session = EcmSession::Create();
  ASSERT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status, EcmCycleStatus::kExecuted);

  EcmCycleInput mixed = MakeSensorInput(3U, false);
  EcmTruthThreat truth;
  truth.truth_entity_id = 77U;
  truth.center_frequency_hz = 9.0e9;
  truth.bandwidth_hz = 5.0e6;
  truth.threat_score = 1.0f;
  mixed.truth_threats.push_back(truth);
  EXPECT_EQ(session.StepWithResult(mixed).status, EcmCycleStatus::kRejectedInvalidInput);

  const EcmCycleResult retry = session.StepWithResult(MakeSensorInput(3U, false));
  EXPECT_EQ(retry.status, EcmCycleStatus::kExecuted);
  EXPECT_EQ(retry.observation_age_successful_ecm_cycles, 1U);
}

TEST(EcmSessionTest, PublishedFrameUsesAbsoluteTimeAndSensorEntityIdentity) {
  config::EcmSessionConfig config;
  EcmSession session = EcmSession::Create(config);
  session.SetSensorEntityId(17U);

  const EcmCycleResult result = session.StepWithResult(MakeSensorInput(2U, true));
  ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
  EXPECT_EQ(result.emission_frame.world_cycle_index, 2U);
  EXPECT_DOUBLE_EQ(result.emission_frame.window_start_time_s, 1.0);
  EXPECT_DOUBLE_EQ(result.emission_frame.window_duration_s, 1.0);
  ASSERT_EQ(result.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(result.emission_frame.emissions.front().identity.sensor_entity_id, 17U);
  EXPECT_DOUBLE_EQ(result.emission_frame.emissions.front().waveform.activity_start_time_s, 1.0);

  // 传 0 恢复缺省 101U
  session.SetSensorEntityId(0U);
  const EcmCycleResult reset_result = session.StepWithResult(MakeSensorInput(3U, true));
  ASSERT_EQ(reset_result.status, EcmCycleStatus::kExecuted);
  ASSERT_EQ(reset_result.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(reset_result.emission_frame.emissions.front().identity.sensor_entity_id, 101U);
}

TEST(EcmSessionTest, PoweredOffAdvancesChronologyWithoutConsumingEmissionId) {
  config::EcmSessionConfig config;
  config.power_on = false;
  EcmSession session = EcmSession::Create(config);
  EXPECT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status, EcmCycleStatus::kPoweredOff);

  config::EcmRuntimeConfigPatch patch;
  patch.has_power_on = true;
  patch.power_on = true;
  ASSERT_TRUE(session.ApplyRuntimeConfig(patch).applied);
  EXPECT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status,
            EcmCycleStatus::kRejectedInvalidInput);
  const EcmCycleResult next = session.StepWithResult(MakeSensorInput(3U, true));
  ASSERT_EQ(next.status, EcmCycleStatus::kExecuted);
  ASSERT_EQ(next.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(next.emission_frame.emissions.front().identity.emission_id, 1U);
}

TEST(EcmSessionTest, ChannelAndPowerBudgetsAreConservedForAllTechniques) {
  for (EcmTechnique technique :
       {EcmTechnique::kSpot, EcmTechnique::kBarrage, EcmTechnique::kSweep}) {
    config::EcmSessionConfig config;
    config.channel_count = 2U;
    config.maximum_total_transmit_power_w = 1000.0;
    config.maximum_channel_transmit_power_w = 600.0;
    config.default_technique = technique;
    EcmSession session = EcmSession::Create(config);
    EcmCycleInput input = MakeSensorInput(2U, true);
    input.sensor_observation_frame.observations.push_back(MakeObservation(11U, 9.0e9, 0.8f));
    input.sensor_observation_frame.observations.push_back(MakeObservation(12U, 8.0e9, 0.7f));

    const EcmCycleResult result = session.StepWithResult(input);
    ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
    ASSERT_EQ(result.emission_frame.emissions.size(), 2U);
    double allocated_power_w = 0.0;
    for (const EcmResourceDecision& decision : result.decisions) {
      EXPECT_LE(decision.allocated_power_w, 600.0);
      allocated_power_w += decision.allocated_power_w;
    }
    EXPECT_LE(allocated_power_w, 1000.0);
    if (technique == EcmTechnique::kSweep) {
      EXPECT_EQ(result.emission_frame.emissions.front().waveform.kind,
                oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep);
    }
  }
}

TEST(EcmSessionTest, TruthAssistedOwnershipIsExplicitAndSeparate) {
  EcmSession session = EcmSession::Create();
  session.SetSensorEntityId(900U);
  EcmCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.input_mode = EcmInputMode::kTruthAssisted;
  EcmTruthThreat truth;
  truth.truth_entity_id = 77U;
  truth.center_frequency_hz = 9.0e9;
  truth.bandwidth_hz = 5.0e6;
  truth.threat_score = 1.0f;
  input.truth_threats.push_back(truth);

  const EcmCycleResult result = session.StepWithResult(input);
  ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
  EXPECT_TRUE(result.truth_assisted);
  ASSERT_EQ(result.decisions.size(), 1U);
  EXPECT_EQ(result.decisions.front().truth_entity_id, 77U);
  EXPECT_EQ(result.emission_frame.emissions.front().identity.sensor_entity_id, 900U);
}

TEST(EcmSessionTest, SweepSnapshotContinuationIsDeterministic) {
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kSweep;
  EcmSession session = EcmSession::Create(config);
  ASSERT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status, EcmCycleStatus::kExecuted);
  const EcmRuntimeState snapshot = session.CaptureRuntimeState();

  const EcmCycleResult first = session.StepWithResult(MakeSensorInput(3U, false));
  ASSERT_TRUE(session.RestoreRuntimeState(snapshot));
  const EcmCycleResult replayed = session.StepWithResult(MakeSensorInput(3U, false));

  ASSERT_EQ(first.emission_frame.emissions.size(), 1U);
  ASSERT_EQ(replayed.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(first.emission_frame.emissions.front().identity.emission_id,
            replayed.emission_frame.emissions.front().identity.emission_id);
  EXPECT_DOUBLE_EQ(first.emission_frame.emissions.front().waveform.sweep_start_frequency_hz,
                   replayed.emission_frame.emissions.front().waveform.sweep_start_frequency_hz);
  EXPECT_DOUBLE_EQ(first.emission_frame.emissions.front().waveform.sweep_stop_frequency_hz,
                   replayed.emission_frame.emissions.front().waveform.sweep_stop_frequency_hz);
}

TEST(EcmSessionTest, SnapshotFollowsImplAcrossFacadeMoveAndRejectsForeignSession) {
  EcmSession session = EcmSession::Create();
  ASSERT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status, EcmCycleStatus::kExecuted);
  const EcmRuntimeState snapshot = session.CaptureRuntimeState();

  EcmSession moved = std::move(session);
  EXPECT_TRUE(moved.RestoreRuntimeState(snapshot));

  EcmSession foreign = EcmSession::Create();
  const EcmRuntimeState foreign_before = foreign.CaptureRuntimeState();
  EXPECT_FALSE(foreign.RestoreRuntimeState(snapshot));
  const EcmRuntimeState foreign_after = foreign.CaptureRuntimeState();
  EXPECT_EQ(foreign_after.next_emission_id, foreign_before.next_emission_id);
  EXPECT_DOUBLE_EQ(foreign_after.thermal_energy_j, foreign_before.thermal_energy_j);
}

// Dirty-snapshot negative tests: each case mutates one field of a captured
// (well-formed) snapshot and asserts RestoreRuntimeState rejects it and leaves
// the session untouched. Covers design §3 "完整校验所有嵌套 observation、重复 ID、
// provenance、模式组合和随机状态，失败不得部分修改". owner_identity still matches,
// so these exercises the deep validation gate, not the ownership check.

// Capture a baseline well-formed snapshot from a session that has executed one
// successful sensor-driven cycle (so has_last_sensor_frame and has_successful_
// cycle are both true with valid nested observations).
EcmRuntimeState CaptureBaselineSensorSnapshot(EcmSession* session) {
  EXPECT_NE(session, nullptr);
  EXPECT_EQ(session->StepWithResult(MakeSensorInput(2U, true)).status, EcmCycleStatus::kExecuted);
  return session->CaptureRuntimeState();
}

TEST(EcmSessionTest, RestoreRejectsDirtySnapshotAndLeavesSessionUntouched) {
  // Each sub-test reuses one session: restore a clean snapshot, confirm it
  // applies, then attempt a dirty variant and confirm rejection + no mutation.
  auto run_dirty_case = [](const std::string& /*name*/, EcmRuntimeState dirty) {
    EcmSession session = EcmSession::Create();
    // Advance the session so it has distinguishable state to detect mutation.
    ASSERT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status, EcmCycleStatus::kExecuted);
    const EcmRuntimeState before = session.CaptureRuntimeState();
    // Replace only the owner token so rejection is decided by nested-state validation,
    // not short-circuited by the helper snapshot's original session identity.
    dirty.owner_identity = before.owner_identity;
    EXPECT_FALSE(session.RestoreRuntimeState(dirty)) << "dirty snapshot should be rejected";
    const EcmRuntimeState after = session.CaptureRuntimeState();
    EXPECT_EQ(after.next_emission_id, before.next_emission_id)
        << "session mutated on rejected restore";
    EXPECT_DOUBLE_EQ(after.thermal_energy_j, before.thermal_energy_j);
    EXPECT_EQ(after.has_successful_cycle, before.has_successful_cycle);
    EXPECT_EQ(after.last_successful_cycle_index, before.last_successful_cycle_index);
  };

  EcmSession baseline_session = EcmSession::Create();
  EcmRuntimeState base = CaptureBaselineSensorSnapshot(&baseline_session);

  // Case 1: invalid nested observation (NaN center frequency).
  {
    EcmRuntimeState dirty = base;
    dirty.last_sensor_frame.observations.front().estimated_center_frequency_hz =
        std::numeric_limits<double>::quiet_NaN();
    run_dirty_case("invalid nested observation", dirty);
  }
  // Case 2: duplicate source_hypothesis_id within the frame.
  {
    EcmRuntimeState dirty = base;
    dirty.last_sensor_frame.observations.push_back(dirty.last_sensor_frame.observations.front());
    run_dirty_case("duplicate hypothesis id", dirty);
  }
  // Case 3: has_successful_cycle=false but last_successful_cycle_index != 0.
  {
    EcmRuntimeState dirty = base;
    dirty.has_successful_cycle = false;
    run_dirty_case("successful flag cleared but index retained", dirty);
  }
  // Case 4: has_successful_cycle=true but last_successful_cycle_index == 0.
  {
    EcmRuntimeState dirty = base;
    dirty.last_successful_cycle_index = 0U;
    run_dirty_case("successful flag set but zero index", dirty);
  }
}

TEST(EcmSessionTest, RestoreRejectsShallowInconsistencyRegression) {
  // has_last_sensor_frame=false must still reject non-empty observations or a
  // non-zero glide age (guards against the shallow check being dropped).
  EcmSession session = EcmSession::Create();
  ASSERT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status, EcmCycleStatus::kExecuted);
  const EcmRuntimeState before = session.CaptureRuntimeState();

  EcmRuntimeState dirty = before;
  dirty.has_last_sensor_frame = false;
  // leave observations populated -> inconsistent with the cleared flag
  EXPECT_FALSE(session.RestoreRuntimeState(dirty));

  const EcmRuntimeState after = session.CaptureRuntimeState();
  EXPECT_EQ(after.has_last_sensor_frame, before.has_last_sensor_frame);
  EXPECT_EQ(after.last_sensor_frame.observations.size(),
            before.last_sensor_frame.observations.size());
}

TEST(EcmSessionTest, EsrAdapterCopiesOnlyDetruthEstimatedFields) {
  electronic_surveillance_radar::session::EmitterHypothesis hypothesis;
  hypothesis.hypothesis_id = 55U;
  hypothesis.estimated_center_frequency_hz = 10.0e9;
  hypothesis.estimated_bandwidth_hz = 5.0e6;
  hypothesis.estimated_pri_s = 1.0e-3;
  hypothesis.estimated_pulse_width_s = 1.0e-6;
  hypothesis.center_frequency_std_hz = 1000.0;
  hypothesis.bandwidth_std_hz = 2000.0;
  hypothesis.bearing_std_deg = 1.0f;
  hypothesis.confidence = 0.8f;
  hypothesis.threat_level = electronic_surveillance_radar::session::EsrThreatLevel::kHigh;
  electronic_surveillance_radar::session::EmitterHypothesisList hypotheses;
  hypotheses.push_back(hypothesis);

  EcmSensorObservationFrame frame;
  ASSERT_TRUE(TryBuildEcmSensorObservationFrame(hypotheses, 700U, &frame));
  ASSERT_EQ(frame.observations.size(), 1U);
  EXPECT_EQ(frame.source_esr_batch_id, 700U);
  EXPECT_EQ(frame.observations.front().source_hypothesis_id, 55U);
  EXPECT_DOUBLE_EQ(frame.observations.front().estimated_center_frequency_hz, 10.0e9);
  EXPECT_FLOAT_EQ(frame.observations.front().threat_score, 0.8f);
}

TEST(EcmSessionTest, TruthAssistedSnapshotValidatesDeceptionStates) {
  // Bug 3b: TruthAssisted sessions cache no sensor frame. SnapshotInternallyConsistent
  // must still validate deception states even when has_last_sensor_frame is false.
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kDeception;
  config.default_deception_mode = EcmDeceptionMode::kRgpo;
  EcmSession session = EcmSession::Create(config);

  // Run TruthAssisted deception cycles to create an engaged deception state.
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    EcmCycleInput input;
    input.cycle_index = cycle;
    input.cycle_start_time_s = static_cast<double>(cycle - 1U);
    input.dt_sec = 1.0;
    input.input_mode = EcmInputMode::kTruthAssisted;
      input.platform_position_ecef_m.x_m = 6378137.0;
    EcmTruthThreat threat;
    threat.truth_entity_id = 100U;
    threat.center_frequency_hz = 10.0e9;
    threat.bandwidth_hz = 10.0e6;
    threat.threat_score = 0.9f;
    threat.estimated_pri_s = 1.0e-3;
    threat.estimated_pulse_width_s = 1.0e-6;
    input.truth_threats.push_back(threat);
    const EcmCycleResult result = session.StepWithResult(input);
    ASSERT_EQ(result.status, EcmCycleStatus::kExecuted) << "cycle " << cycle;
  }

  // Verify snapshot validates deception states
  const EcmRuntimeState before = session.CaptureRuntimeState();
  EXPECT_FALSE(before.deception_states.empty());
  EXPECT_TRUE(before.deception_states.front().engaged);

  // Corrupt a deception state field
  EcmRuntimeState dirty = before;
  dirty.deception_states.front().current_delay_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(session.RestoreRuntimeState(dirty));

  // Session state must be unchanged after rejected restore.
  const EcmRuntimeState after = session.CaptureRuntimeState();
  ASSERT_FALSE(after.deception_states.empty());
  EXPECT_TRUE(std::isfinite(after.deception_states.front().current_delay_s));
}

TEST(EcmSessionTest, SnapshotRejectsModeIrrelevantDeceptionFieldsWithoutMutation) {
  const auto run_case = [](EcmDeceptionMode mode,
                           const std::function<void(EcmDeceptionState*)>& corrupt) {
    config::EcmSessionConfig config;
    config.default_technique = EcmTechnique::kDeception;
    config.default_deception_mode = mode;
    EcmSession session = EcmSession::Create(config);
    ASSERT_EQ(session.StepWithResult(MakeTruthInput(1U)).status, EcmCycleStatus::kExecuted);
    const EcmRuntimeState before = session.CaptureRuntimeState();
    ASSERT_EQ(before.deception_states.size(), 1U);
    EcmRuntimeState dirty = before;
    corrupt(&dirty.deception_states.front());
    EXPECT_FALSE(session.RestoreRuntimeState(dirty));
    const EcmRuntimeState after = session.CaptureRuntimeState();
    EXPECT_DOUBLE_EQ(after.deception_states.front().current_delay_s,
                     before.deception_states.front().current_delay_s);
    EXPECT_DOUBLE_EQ(after.deception_states.front().current_doppler_offset_hz,
                     before.deception_states.front().current_doppler_offset_hz);
  };

  run_case(EcmDeceptionMode::kRgpo,
           [](EcmDeceptionState* state) { state->current_doppler_offset_hz = 1.0; });
  run_case(EcmDeceptionMode::kVgpo,
           [](EcmDeceptionState* state) { state->current_delay_s = 1.0e-9; });
  run_case(EcmDeceptionMode::kFalseTarget,
           [](EcmDeceptionState* state) { state->current_doppler_offset_hz = 1.0; });
}

TEST(EcmSessionTest, RuntimeTechniqueOrModeChangeReleasesDeceptionState) {
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kDeception;
  config.default_deception_mode = EcmDeceptionMode::kRgpo;
  EcmSession session = EcmSession::Create(config);
  ASSERT_EQ(session.StepWithResult(MakeTruthInput(1U)).status, EcmCycleStatus::kExecuted);
  ASSERT_FALSE(session.CaptureRuntimeState().deception_states.empty());

  config::EcmRuntimeConfigPatch patch;
  patch.has_default_deception_mode = true;
  patch.default_deception_mode = EcmDeceptionMode::kVgpo;
  ASSERT_TRUE(session.ApplyRuntimeConfig(patch).applied);
  EXPECT_TRUE(session.CaptureRuntimeState().deception_states.empty());
}

// 快照校验的 P1 加固：验证之前仅检查 engaged 状态有限字段的 SnapshotInternallyConsistent
// 现拒绝非法 mode/phase 枚举、超上限交战数、delay/doppler 超限、重复 threat_id 及
// engaged+phase 非法组合。
TEST(EcmSessionTest, SnapshotRejectsIllegalDeceptionModePhaseEnums) {
  EcmSession session = EcmSession::Create();
  EcmRuntimeState base = CaptureBaselineSensorSnapshot(&session);
  base.active_config.default_technique = EcmTechnique::kDeception;
  // 在基线快照上手工构造非法 deception state 并置入。
  EcmDeceptionState bogus;
  bogus.threat_id = 999U;
  bogus.mode = static_cast<EcmDeceptionMode>(99);  // 非法 mode
  bogus.phase = EcmDeceptionPhase::kTowing;
  bogus.engaged = true;
  bogus.current_delay_s = 0.0;
  bogus.current_doppler_offset_hz = 0.0;
  bogus.phase_elapsed_s = 0.0;
  EcmRuntimeState dirty = base;
  dirty.deception_states.push_back(bogus);
  EXPECT_FALSE(session.RestoreRuntimeState(dirty));

  bogus.mode = EcmDeceptionMode::kRgpo;
  bogus.phase = static_cast<EcmDeceptionPhase>(99);  // 非法 phase
  dirty = base;
  dirty.deception_states.push_back(bogus);
  EXPECT_FALSE(session.RestoreRuntimeState(dirty));
}

TEST(EcmSessionTest, SnapshotRejectsEngagedExceedingMaxActive) {
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kDeception;
  config.default_deception_mode = EcmDeceptionMode::kRgpo;
  config.deception_max_active = 1U;  // 仅允许一个活跃交战
  EcmSession session = EcmSession::Create(config);
  // 运行一个 TruthAssisted 周期生成一个活跃交战。
  EcmCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 1.0;
  input.input_mode = EcmInputMode::kTruthAssisted;
  input.platform_position_ecef_m.x_m = 6378137.0;
  EcmTruthThreat threat;
  threat.truth_entity_id = 100U;
  threat.center_frequency_hz = 10.0e9;
  threat.bandwidth_hz = 10.0e6;
  threat.threat_score = 0.9f;
  input.truth_threats.push_back(threat);
  ASSERT_EQ(session.StepWithResult(input).status, EcmCycleStatus::kExecuted);
  EcmRuntimeState before = session.CaptureRuntimeState();
  // 手工插入第二个活跃交战（同名 threat_id 但不同交战）。
  EcmDeceptionState extra;
  extra.threat_id = 200U;
  extra.mode = EcmDeceptionMode::kRgpo;
  extra.phase = EcmDeceptionPhase::kTowing;
  extra.engaged = true;
  extra.current_delay_s = 0.0;
  extra.current_doppler_offset_hz = 0.0;
  extra.phase_elapsed_s = 0.0;
  before.deception_states.push_back(extra);
  // 当前 active_config.deception_max_active = 1，插入后共 2 个 → 拒绝。
  EXPECT_FALSE(session.RestoreRuntimeState(before));
}

TEST(EcmSessionTest, SnapshotRejectsEngagedWithPhaseIdle) {
  EcmSession session = EcmSession::Create();
  EcmRuntimeState base = CaptureBaselineSensorSnapshot(&session);
  EcmDeceptionState engaged_idle;
  engaged_idle.threat_id = 888U;
  engaged_idle.mode = EcmDeceptionMode::kRgpo;
  engaged_idle.phase = EcmDeceptionPhase::kIdle;  // engaged=true 不允许 kIdle
  engaged_idle.engaged = true;
  engaged_idle.current_delay_s = 0.0;
  engaged_idle.current_doppler_offset_hz = 0.0;
  engaged_idle.phase_elapsed_s = 0.0;
  EcmRuntimeState dirty = base;
  dirty.active_config.default_technique = EcmTechnique::kDeception;
  dirty.deception_states.push_back(engaged_idle);
  EXPECT_FALSE(session.RestoreRuntimeState(dirty));
}

TEST(EcmSessionTest, SnapshotRejectsNonEngagedWithPhaseTowing) {
  EcmSession session = EcmSession::Create();
  EcmRuntimeState base = CaptureBaselineSensorSnapshot(&session);
  EcmDeceptionState towing_not_engaged;
  towing_not_engaged.threat_id = 777U;
  towing_not_engaged.mode = EcmDeceptionMode::kRgpo;
  towing_not_engaged.phase = EcmDeceptionPhase::kTowing;  // kTowing 要求 engaged
  towing_not_engaged.engaged = false;
  EcmRuntimeState dirty = base;
  dirty.active_config.default_technique = EcmTechnique::kDeception;
  dirty.deception_states.push_back(towing_not_engaged);
  EXPECT_FALSE(session.RestoreRuntimeState(dirty));
}

TEST(EcmSessionTest, SnapshotRejectsDelayBeyondMaximum) {
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kDeception;
  config.default_deception_mode = EcmDeceptionMode::kRgpo;
  config.deception_rgpo_max_range_m = 1000.0;  // 最大时延 = 2*1000/c ≈ 6.67 µs
  EcmSession session = EcmSession::Create(config);
  // 运行一个 TruthAssisted 周期。
  EcmCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 1.0;
  input.input_mode = EcmInputMode::kTruthAssisted;
  input.platform_position_ecef_m.x_m = 6378137.0;
  EcmTruthThreat threat;
  threat.truth_entity_id = 100U;
  threat.center_frequency_hz = 10.0e9;
  threat.bandwidth_hz = 10.0e6;
  threat.threat_score = 0.9f;
  input.truth_threats.push_back(threat);
  ASSERT_EQ(session.StepWithResult(input).status, EcmCycleStatus::kExecuted);
  EcmRuntimeState before = session.CaptureRuntimeState();
  // 篡改 delay 为超大值。
  if (!before.deception_states.empty()) {
    before.deception_states.front().current_delay_s = 1.0;  // 远超上限
    EXPECT_FALSE(session.RestoreRuntimeState(before));
  }
}

TEST(EcmSessionTest, SnapshotRejectsDuplicateThreatIdAmongEngaged) {
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kDeception;
  config.default_deception_mode = EcmDeceptionMode::kRgpo;
  config.deception_max_active = 3U;
  EcmSession session = EcmSession::Create(config);
  EcmCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 1.0;
  input.input_mode = EcmInputMode::kTruthAssisted;
  input.platform_position_ecef_m.x_m = 6378137.0;
  EcmTruthThreat threat;
  threat.truth_entity_id = 100U;
  threat.center_frequency_hz = 10.0e9;
  threat.bandwidth_hz = 10.0e6;
  threat.threat_score = 0.9f;
  input.truth_threats.push_back(threat);
  ASSERT_EQ(session.StepWithResult(input).status, EcmCycleStatus::kExecuted);
  EcmRuntimeState before = session.CaptureRuntimeState();
  // 手工克隆已有的 engaged state（相同 threat_id）→ 重复。
  if (!before.deception_states.empty()) {
    EcmDeceptionState dup = before.deception_states.front();
    dup.current_delay_s = 0.5e-6;  // 合法值，仅威胁 id 重复
    before.deception_states.push_back(dup);
    EXPECT_FALSE(session.RestoreRuntimeState(before));
  }
}

// StepWithResult 事务不变量：当波形构造中途失败时，候选状态被丢弃，
// 会话不得部分推进（既不预扣 emission ID / 热量，也不推进欺骗状态机或任何 RNG 流）。
// 触发点：欺骗 PRI 被钳位到 1e-6，dt_sec=2.0 使 pulse_count = 2e6 > 1e6 上限，
// TryCreateRfPulseTrainWaveform 经 IsValidWaveform 拒绝 → TryBuildDeceptionEmission 返回
// false → EcmSession.cpp 中的 RejectInvalidConfig 分支（波形构造失败→丢弃候选）。
TEST(EcmSessionTest, WaveformFailureLeavesSessionStateUnchanged) {
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kDeception;
  config.default_deception_mode = EcmDeceptionMode::kRgpo;
  EcmSession session = EcmSession::Create(config);

  // 先执行一个正常周期，建立可区分的活跃交战与资源状态。
  ASSERT_EQ(session.StepWithResult(MakeTruthInput(1U)).status, EcmCycleStatus::kExecuted);
  const EcmRuntimeState before = session.CaptureRuntimeState();
  ASSERT_EQ(before.next_emission_id, 2U);
  ASSERT_FALSE(before.deception_states.empty());

  // 构造必然使脉冲计数越界的输入：最小 PRI + 2 秒窗口。
  // pulse_count = floor(2.0 / 1e-6) = 2,000,000 > kMaximumPulseCount(1,000,000)。
  // 脉宽设为 1e-9（钳位后 < 0.5*pri），确保失败由计数门决定，而非脉宽门。
  EcmCycleInput input;
  input.cycle_index = 2U;
  input.cycle_start_time_s = 1.0;
  input.dt_sec = 2.0;
  input.input_mode = EcmInputMode::kTruthAssisted;
  input.platform_position_ecef_m.x_m = 6378137.0;
  EcmTruthThreat threat;
  threat.truth_entity_id = 100U;
  threat.center_frequency_hz = 10.0e9;
  threat.bandwidth_hz = 10.0e6;
  threat.threat_score = 0.9f;
  threat.estimated_pri_s = 1.0e-6;
  threat.estimated_pulse_width_s = 1.0e-9;
  input.truth_threats.push_back(threat);

  const EcmCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, EcmCycleStatus::kRejectedInvalidConfig);
  EXPECT_TRUE(result.emission_frame.emissions.empty());
  EXPECT_TRUE(result.decisions.empty());

  // 事务不变量：会话状态逐字段不变。
  const EcmRuntimeState after = session.CaptureRuntimeState();
  EXPECT_EQ(after.next_emission_id, before.next_emission_id)
      << "emission ID 不得在失败时被预留";
  EXPECT_DOUBLE_EQ(after.thermal_energy_j, before.thermal_energy_j)
      << "热预算不得在失败时被累加";
  EXPECT_EQ(after.has_successful_cycle, before.has_successful_cycle);
  EXPECT_EQ(after.last_successful_cycle_index, before.last_successful_cycle_index);
  ASSERT_EQ(after.deception_states.size(), before.deception_states.size());
  EXPECT_EQ(after.deception_states.front().threat_id, before.deception_states.front().threat_id);
  EXPECT_DOUBLE_EQ(after.deception_states.front().current_delay_s,
                   before.deception_states.front().current_delay_s)
      << "欺骗状态机不得在失败时被推进";
  EXPECT_DOUBLE_EQ(after.deception_states.front().cycle_count,
                   before.deception_states.front().cycle_count);
  EXPECT_EQ(after.scheduling_rng_state, before.scheduling_rng_state) << "scheduling RNG 不得被消费";
  EXPECT_EQ(after.tie_break_rng_state, before.tie_break_rng_state) << "tie-break RNG 不得被消费";
  EXPECT_EQ(after.deception_rng_state, before.deception_rng_state) << "deception RNG 不得被消费";

  // 后续合法周期可正常执行——会话未被失败路径破坏。
  const EcmCycleResult recovered = session.StepWithResult(MakeTruthInput(2U));
  EXPECT_EQ(recovered.status, EcmCycleStatus::kExecuted);
}

}  // namespace
}  // namespace session
}  // namespace electronic_countermeasure
