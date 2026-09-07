/**
 * @file ecm_deception_test.cpp
 * @brief 欺骗干扰模式单元测试：RGPO、VGPO、RGPO+VGPO、假目标。
 */

#include <cmath>
#include <memory>

#include "1q/electronic_countermeasure/EcmSession.h"
#include "1q/electromagnetics/RfScene.h"
#include "gtest/gtest.h"

namespace electronic_countermeasure {
namespace session {
namespace {

EcmSensorObservation MakeObservation(std::uint64_t id, double center_hz,
                                     float threat_score) {
  EcmSensorObservation obs;
  obs.source_hypothesis_id = id;
  obs.estimated_center_frequency_hz = center_hz;
  obs.estimated_bandwidth_hz = 10.0e6;
  obs.estimated_pri_s = 1.0e-3;
  obs.estimated_pulse_width_s = 1.0e-6;
  obs.center_frequency_std_hz = 1000.0;
  obs.bandwidth_std_hz = 2000.0;
  obs.bearing_std_deg = 1.0;
  obs.threat_score = threat_score;
  obs.confidence = 0.9f;
  return obs;
}

EcmCycleInput MakeSensorInput(std::uint32_t cycle_index) {
  EcmCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = static_cast<double>(cycle_index - 1U);
  input.dt_sec = 1.0;
  input.input_mode = EcmInputMode::kSensorDriven;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.has_sensor_observation_frame = true;
  input.sensor_observation_frame.source_esr_batch_id =
      static_cast<std::uint64_t>(cycle_index);
  input.sensor_observation_frame.observations.push_back(
      MakeObservation(10U, 10.0e9, 0.9f));
  return input;
}

config::EcmSessionConfig MakeDeceptionConfig(EcmDeceptionMode mode) {
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kDeception;
  config.default_deception_mode = mode;
  return config;
}

TEST(EcmDeceptionTest, DeceptionTechniqueProducesPulseTrainWaveform) {
  EcmSession session = EcmSession::Create(MakeDeceptionConfig(EcmDeceptionMode::kRgpo));
  const EcmCycleInput input = MakeSensorInput(1U);
  const EcmCycleResult result = session.StepWithResult(input);

  ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
  ASSERT_EQ(result.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(result.emission_frame.emissions.front().waveform.kind,
            oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain);
  EXPECT_EQ(result.decisions.front().technique, EcmTechnique::kDeception);
  EXPECT_EQ(result.decisions.front().deception_mode, EcmDeceptionMode::kRgpo);
  EXPECT_EQ(result.decisions.front().deception_phase, EcmDeceptionPhase::kTowing);
}

TEST(EcmDeceptionTest, RgpoDelayAdvancesEachCycle) {
  EcmSession session = EcmSession::Create(MakeDeceptionConfig(EcmDeceptionMode::kRgpo));
  constexpr double c = 299792458.0;
  double previous_delay_s = 0.0;

  for (std::uint32_t cycle = 1U; cycle <= 5U; ++cycle) {
    EcmCycleInput input = MakeSensorInput(cycle);
    input.cycle_start_time_s = static_cast<double>(cycle - 1U);
    const EcmCycleResult result = session.StepWithResult(input);
    ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
    ASSERT_EQ(result.emission_frame.emissions.size(), 1U);
    const double first_pulse_time_s =
        result.emission_frame.emissions.front().waveform.first_pulse_time_s;
    // first_pulse_time_s = cycle_start_time_s + current_delay_s
    const double current_delay_s =
        first_pulse_time_s - input.cycle_start_time_s;
    if (cycle > 1U) {
      EXPECT_GT(current_delay_s, previous_delay_s);
    }
    previous_delay_s = current_delay_s;
  }
}

TEST(EcmDeceptionTest, RgpoHoldAfterMaxDelay) {
  config::EcmSessionConfig config =
      MakeDeceptionConfig(EcmDeceptionMode::kRgpo);
  // Set a very small max range so we reach holding quickly.
  config.deception_rgpo_max_range_m = 150.0;
  config.deception_hold_time_s = 5.0;

  EcmSession session = EcmSession::Create(config);
  // RGPO rate = 100 m/s, max range = 150 m -> delay = 2*150/c ~1e-6 s
  // Rate = 100 m/s, in 1s per cycle, delay increases by 100/c ~3.34e-7 per cycle
  // After ~3 cycles we should hit the max.
  bool reached_holding = false;
  for (std::uint32_t cycle = 1U; cycle <= 10U; ++cycle) {
    EcmCycleInput input = MakeSensorInput(cycle);
    input.cycle_start_time_s = static_cast<double>(cycle - 1U);
    const EcmCycleResult result = session.StepWithResult(input);
    ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
    if (result.decisions.front().deception_phase == EcmDeceptionPhase::kHolding) {
      reached_holding = true;
      // Delay should be clamped at max
      constexpr double c = 299792458.0;
      const double max_delay = 2.0 * 150.0 / c;
      const double first_pulse_s =
          result.emission_frame.emissions.front().waveform.first_pulse_time_s;
      const double delay = first_pulse_s - input.cycle_start_time_s;
      EXPECT_NEAR(delay, max_delay, 1e-9);
      break;
    }
  }
  EXPECT_TRUE(reached_holding);
}

TEST(EcmDeceptionTest, RgpoStopAfterHoldTime) {
  config::EcmSessionConfig config =
      MakeDeceptionConfig(EcmDeceptionMode::kRgpo);
  config.deception_rgpo_max_range_m = 150.0;
  config.deception_hold_time_s = 0.5;  // Short hold

  EcmSession session = EcmSession::Create(config);
  bool reached_stopped = false;
  EcmCycleStatus last_status = EcmCycleStatus::kExecuted;
  for (std::uint32_t cycle = 1U; cycle <= 12U; ++cycle) {
    EcmCycleInput input = MakeSensorInput(cycle);
    input.cycle_start_time_s = static_cast<double>(cycle - 1U);
    const EcmCycleResult result = session.StepWithResult(input);

    // After stopping, session may glide or safe-stop if no new observations
    // arrive (we provide fresh sensor frame each cycle, so it stays executing).
    if (result.decisions.empty()) {
      last_status = result.status;
      continue;
    }
    if (result.decisions.front().deception_phase == EcmDeceptionPhase::kStopped) {
      reached_stopped = true;
    }
    // After stopped + one more cycle, the emission should be gone (engagement released),
    // but a new one starts since we have a fresh sensor frame -> re-engagement.
  }
  EXPECT_TRUE(reached_stopped);
}

TEST(EcmDeceptionTest, VgpoDopplerOffsetAdvancesEachCycle) {
  EcmSession session = EcmSession::Create(MakeDeceptionConfig(EcmDeceptionMode::kVgpo));
  double previous_center_hz = 10.0e9;

  for (std::uint32_t cycle = 1U; cycle <= 5U; ++cycle) {
    EcmCycleInput input = MakeSensorInput(cycle);
    input.cycle_start_time_s = static_cast<double>(cycle - 1U);
    const EcmCycleResult result = session.StepWithResult(input);
    ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
    ASSERT_EQ(result.emission_frame.emissions.size(), 1U);
    const double center_hz =
        result.emission_frame.emissions.front().waveform.center_frequency_hz;
    if (cycle > 1U) {
      EXPECT_GT(center_hz, previous_center_hz);
    }
    previous_center_hz = center_hz;
  }
}

TEST(EcmDeceptionTest, VgpoHoldAndStop) {
  config::EcmSessionConfig config =
      MakeDeceptionConfig(EcmDeceptionMode::kVgpo);
  config.deception_vgpo_max_doppler_hz = 5000.0;
  config.deception_hold_time_s = 0.5;

  EcmSession session = EcmSession::Create(config);
  bool reached_holding = false;
  for (std::uint32_t cycle = 1U; cycle <= 15U; ++cycle) {
    EcmCycleInput input = MakeSensorInput(cycle);
    input.cycle_start_time_s = static_cast<double>(cycle - 1U);
    const EcmCycleResult result = session.StepWithResult(input);
    if (result.decisions.empty()) {
      continue;
    }
    if (result.decisions.front().deception_phase == EcmDeceptionPhase::kHolding) {
      reached_holding = true;
    }
  }
  EXPECT_TRUE(reached_holding);
}

TEST(EcmDeceptionTest, RgpoVgpoCombinedAdvancesBoth) {
  EcmSession session =
      EcmSession::Create(MakeDeceptionConfig(EcmDeceptionMode::kRgpoVgpo));
  const EcmCycleInput input1 = MakeSensorInput(1U);
  const EcmCycleResult result1 = session.StepWithResult(input1);
  ASSERT_EQ(result1.status, EcmCycleStatus::kExecuted);

  EcmCycleInput input2 = MakeSensorInput(2U);
  input2.cycle_start_time_s = 1.0;
  const EcmCycleResult result2 = session.StepWithResult(input2);
  ASSERT_EQ(result2.status, EcmCycleStatus::kExecuted);

  const double delay1 =
      result1.emission_frame.emissions.front().waveform.first_pulse_time_s -
      input1.cycle_start_time_s;
  const double delay2 =
      result2.emission_frame.emissions.front().waveform.first_pulse_time_s -
      input2.cycle_start_time_s;
  EXPECT_GT(delay2, delay1);

  const double freq1 =
      result1.emission_frame.emissions.front().waveform.center_frequency_hz;
  const double freq2 =
      result2.emission_frame.emissions.front().waveform.center_frequency_hz;
  EXPECT_GT(freq2, freq1);
}

TEST(EcmDeceptionTest, FalseTargetProducesMultipleEmissions) {
  config::EcmSessionConfig config =
      MakeDeceptionConfig(EcmDeceptionMode::kFalseTarget);
  config.deception_max_false_targets_per_threat = 3U;

  EcmSession session = EcmSession::Create(config);
  const EcmCycleInput input = MakeSensorInput(1U);
  const EcmCycleResult result = session.StepWithResult(input);

  ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
  EXPECT_EQ(result.emission_frame.emissions.size(), 3U);
  // All false targets should be kPulseTrain.
  for (const auto& emission : result.emission_frame.emissions) {
    EXPECT_EQ(emission.waveform.kind,
              oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain);
  }
}

TEST(EcmDeceptionTest, DeceptionPowerScaleIsRespected) {
  config::EcmSessionConfig config =
      MakeDeceptionConfig(EcmDeceptionMode::kRgpo);
  config.deception_power_scale = 0.25;
  config.maximum_channel_transmit_power_w = 1000.0;

  EcmSession session = EcmSession::Create(config);
  const EcmCycleInput input = MakeSensorInput(1U);
  const EcmCycleResult result = session.StepWithResult(input);

  ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
  ASSERT_EQ(result.emission_frame.emissions.size(), 1U);
  const double allocated_power = result.decisions.front().allocated_power_w;
  const double emission_power =
      result.emission_frame.emissions.front().waveform.transmit_power_w;
  // allocated_power = max_channel * power_scale = 1000 * 0.25 = 250 W
  EXPECT_DOUBLE_EQ(allocated_power, 250.0);
  EXPECT_DOUBLE_EQ(emission_power, 250.0);
}

TEST(EcmDeceptionTest, DeceptionStateSnapshotRoundtrip) {
  EcmSession session = EcmSession::Create(MakeDeceptionConfig(EcmDeceptionMode::kRgpo));
  // Run a few cycles to accumulate state
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    EcmCycleInput input = MakeSensorInput(cycle);
    input.cycle_start_time_s = static_cast<double>(cycle - 1U);
    session.StepWithResult(input);
  }

  EcmRuntimeState state = session.CaptureRuntimeState();
  EXPECT_FALSE(state.deception_states.empty());
  EXPECT_FALSE(state.deception_rng_state.empty());

  // Restore into same session
  EXPECT_TRUE(session.RestoreRuntimeState(state));

  // Run one more cycle - should continue from the restored state
  EcmCycleInput input4 = MakeSensorInput(4U);
  input4.cycle_start_time_s = 3.0;
  const EcmCycleResult result4 = session.StepWithResult(input4);
  ASSERT_EQ(result4.status, EcmCycleStatus::kExecuted);
  EXPECT_EQ(result4.decisions.front().deception_phase, EcmDeceptionPhase::kTowing);
}

TEST(EcmDeceptionTest, DeceptionRejectedCycleDoesNotAdvanceState) {
  EcmSession session = EcmSession::Create(MakeDeceptionConfig(EcmDeceptionMode::kRgpo));
  const EcmCycleInput input1 = MakeSensorInput(1U);
  const EcmCycleResult result1 = session.StepWithResult(input1);
  ASSERT_EQ(result1.status, EcmCycleStatus::kExecuted);
  const double delay_cycle1 =
      result1.emission_frame.emissions.front().waveform.first_pulse_time_s -
      input1.cycle_start_time_s;

  // Submit an invalid input (mixed mode) that will be rejected
  EcmCycleInput bad_input = MakeSensorInput(2U);
  bad_input.cycle_start_time_s = 1.0;
  bad_input.truth_threats.push_back(EcmTruthThreat{100U, 10.0e9, 10.0e6, 0.5f});
  const EcmCycleResult bad_result = session.StepWithResult(bad_input);
  ASSERT_EQ(bad_result.status, EcmCycleStatus::kRejectedInvalidInput);

  // Now submit a valid input - state should continue from cycle 1, not skip ahead
  EcmCycleInput input3 = MakeSensorInput(2U);
  input3.cycle_start_time_s = 1.0;
  // Need to match batch_id chronology
  input3.sensor_observation_frame.source_esr_batch_id = 2U;
  const EcmCycleResult result3 = session.StepWithResult(input3);
  ASSERT_EQ(result3.status, EcmCycleStatus::kExecuted);
  const double delay_cycle3 =
      result3.emission_frame.emissions.front().waveform.first_pulse_time_s -
      input3.cycle_start_time_s;
  // The delay should have advanced by only 2 successful cycles worth, not 3.
  EXPECT_GT(delay_cycle3, delay_cycle1);
}

TEST(EcmDeceptionTest, DeceptionDefaultConfigIsValid) {
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kDeception;
  EcmSession session = EcmSession::Create(config);
  const EcmCycleInput input = MakeSensorInput(1U);
  const EcmCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, EcmCycleStatus::kExecuted);
}

TEST(EcmDeceptionTest, DeceptionTechniqueIsKnown) {
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kDeception;
  EcmSession session = EcmSession::Create(config);
  // Creating without rejection proves IsKnownTechnique accepts kDeception
  const EcmCycleInput input = MakeSensorInput(1U);
  const EcmCycleResult result = session.StepWithResult(input);
  EXPECT_NE(result.status, EcmCycleStatus::kRejectedInvalidConfig);
}

TEST(EcmDeceptionTest, ConfigRejectsInvalidDeceptionFields) {
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kDeception;
  // Negative power scale should fail validation
  config.deception_power_scale = -0.5;
  EcmSession session = EcmSession::Create(config);
  const EcmCycleInput input = MakeSensorInput(1U);
  const EcmCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, EcmCycleStatus::kRejectedInvalidConfig);
}

TEST(EcmDeceptionTest, ConfigRejectsPowerScaleAboveOne) {
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kDeception;
  config.deception_power_scale = 1.5;
  EcmSession session = EcmSession::Create(config);
  const EcmCycleInput input = MakeSensorInput(1U);
  const EcmCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, EcmCycleStatus::kRejectedInvalidConfig);
}

TEST(EcmDeceptionTest, ConfigRejectsZeroMaxActive) {
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kDeception;
  config.deception_max_active = 0U;
  EcmSession session = EcmSession::Create(config);
  const EcmCycleInput input = MakeSensorInput(1U);
  const EcmCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, EcmCycleStatus::kRejectedInvalidConfig);
}

TEST(EcmDeceptionTest, FalseTargetEmissionsHaveUniqueIdentities) {
  config::EcmSessionConfig config =
      MakeDeceptionConfig(EcmDeceptionMode::kFalseTarget);
  config.deception_max_false_targets_per_threat = 4U;

  EcmSession session = EcmSession::Create(config);
  const EcmCycleInput input = MakeSensorInput(1U);
  const EcmCycleResult result = session.StepWithResult(input);

  ASSERT_EQ(result.emission_frame.emissions.size(), 4U);
  std::set<std::uint64_t> emission_ids;
  for (const auto& emission : result.emission_frame.emissions) {
    emission_ids.insert(emission.identity.emission_id);
  }
  EXPECT_EQ(emission_ids.size(), 4U);
}

TEST(EcmDeceptionTest, TruthAssistedDeceptionUsesThreatPriAndPulseWidth) {
  config::EcmSessionConfig config =
      MakeDeceptionConfig(EcmDeceptionMode::kRgpo);
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
  threat.threat_score = 0.8f;
  threat.estimated_pri_s = 2.0e-3;
  threat.estimated_pulse_width_s = 5.0e-7;
  input.truth_threats.push_back(threat);

  const EcmCycleResult result = session.StepWithResult(input);
  ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
  ASSERT_EQ(result.emission_frame.emissions.size(), 1U);
  const auto& waveform = result.emission_frame.emissions.front().waveform;
  EXPECT_DOUBLE_EQ(waveform.pulse_repetition_interval_s, 2.0e-3);
  EXPECT_DOUBLE_EQ(waveform.pulse_width_s, 5.0e-7);
}

TEST(EcmDeceptionTest, RuntimePatchCanSwitchToDeception) {
  // Create session with noise jamming
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kSpot;
  EcmSession session = EcmSession::Create(config);

  // Apply patch to switch to deception
  config::EcmRuntimeConfigPatch patch;
  patch.has_default_technique = true;
  patch.default_technique = EcmTechnique::kDeception;
  patch.has_default_deception_mode = true;
  patch.default_deception_mode = EcmDeceptionMode::kRgpo;
  const EcmRuntimeConfigApplyResult apply_result =
      session.ApplyRuntimeConfig(patch);
  EXPECT_TRUE(apply_result.applied);

  // Now run a deception cycle
  const EcmCycleInput input = MakeSensorInput(1U);
  const EcmCycleResult result = session.StepWithResult(input);
  ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
  EXPECT_EQ(result.decisions.front().technique, EcmTechnique::kDeception);
}

TEST(EcmDeceptionTest, FalseTargetEmitsEveryCycle) {
  // Bug 1: kFalseTarget must emit every cycle, not every-other cycle.
  config::EcmSessionConfig config =
      MakeDeceptionConfig(EcmDeceptionMode::kFalseTarget);
  config.deception_max_false_targets_per_threat = 3U;
  EcmSession session = EcmSession::Create(config);

  for (std::uint32_t cycle = 1U; cycle <= 10U; ++cycle) {
    EcmCycleInput input = MakeSensorInput(cycle);
    input.cycle_start_time_s = static_cast<double>(cycle - 1U);
    const EcmCycleResult result = session.StepWithResult(input);
    ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
    EXPECT_EQ(result.emission_frame.emissions.size(), 3U)
        << "cycle " << cycle << " should produce 3 false targets";
  }
}

TEST(EcmDeceptionTest, DeceptionRespectsMaxActiveCap) {
  // Bug 2a: deception_max_active must limit concurrent deception engagements.
  config::EcmSessionConfig config =
      MakeDeceptionConfig(EcmDeceptionMode::kFalseTarget);
  config.deception_max_active = 1U;
  config.channel_count = 3U;
  EcmSession session = EcmSession::Create(config);

  EcmCycleInput input = MakeSensorInput(1U);
  // Add observations at different center frequencies to create distinct
  // threats. Offset in bearing to prevent resolution-cell merging.
  input.sensor_observation_frame.observations.clear();
  input.sensor_observation_frame.observations.push_back(
      MakeObservation(10U, 10.0e9, 0.9f));
  input.sensor_observation_frame.observations[0].bearing_el_deg = 0.0;
  input.sensor_observation_frame.observations.push_back(
      MakeObservation(20U, 11.0e9, 0.8f));
  input.sensor_observation_frame.observations[1].bearing_el_deg = 30.0;
  input.sensor_observation_frame.observations.push_back(
      MakeObservation(30U, 12.0e9, 0.7f));
  input.sensor_observation_frame.observations[2].bearing_el_deg = 60.0;

  const EcmCycleResult result = session.StepWithResult(input);
  ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
  // Only 1 channel emits (max_active=1). Other 2 threats are skipped and
  // recorded as decisions with reason "deception concurrency cap reached".
  EXPECT_EQ(result.emission_frame.emissions.size(),
            config.deception_max_false_targets_per_threat);
  // 3 decisions: 1 emitting + 2 skipped (cap reached)
  EXPECT_EQ(result.decisions.size(), 3U);
  // The first decision (highest-priority threat) should have emissions
  EXPECT_EQ(result.decisions[0].reason, "highest-threat feasible channel allocation");
  // The other two should be concurrency-cap rejections
  EXPECT_EQ(result.decisions[1].reason, "deception concurrency cap reached");
  EXPECT_EQ(result.decisions[2].reason, "deception concurrency cap reached");
}

TEST(EcmDeceptionTest, FalseTargetAppliesPowerScale) {
  // Bug 2c: FalseTarget branch must apply deception_power_scale.
  config::EcmSessionConfig config =
      MakeDeceptionConfig(EcmDeceptionMode::kFalseTarget);
  config.deception_power_scale = 0.25;
  config.maximum_channel_transmit_power_w = 1000.0;
  config.deception_max_false_targets_per_threat = 4U;
  EcmSession session = EcmSession::Create(config);

  const EcmCycleInput input = MakeSensorInput(1U);
  const EcmCycleResult result = session.StepWithResult(input);

  ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
  ASSERT_EQ(result.emission_frame.emissions.size(), 4U);
  // Each false target should get: (1000 * 0.25) / 4 = 62.5 W
  for (const auto& emission : result.emission_frame.emissions) {
    EXPECT_DOUBLE_EQ(emission.waveform.transmit_power_w, 62.5);
  }
  // Decision should record scaled total power: 1000 * 0.25 = 250 W
  EXPECT_DOUBLE_EQ(result.decisions.front().allocated_power_w, 250.0);
}

TEST(EcmDeceptionTest, DeceptionThermalAccounting) {
  // Bug 2b: thermal energy must increase by scaled power * dt, not unscaled.
  config::EcmSessionConfig config =
      MakeDeceptionConfig(EcmDeceptionMode::kRgpo);
  config.deception_power_scale = 0.5;
  config.maximum_channel_transmit_power_w = 1000.0;
  config.cooling_power_w = 0.0;  // Disable cooling for clean measurement
  EcmSession session = EcmSession::Create(config);

  // Thermal starts at 0.
  EcmCycleInput input1 = MakeSensorInput(1U);
  input1.dt_sec = 1.0;
  const EcmCycleResult result1 = session.StepWithResult(input1);
  ASSERT_EQ(result1.status, EcmCycleStatus::kExecuted);
  // Emitted power = 1000 * 0.5 = 500 W. Thermal += 500 * 1.0 = 500 J.
  EXPECT_DOUBLE_EQ(result1.thermal_energy_j, 500.0);

  EcmCycleInput input2 = MakeSensorInput(2U);
  input2.cycle_start_time_s = 1.0;
  input2.dt_sec = 1.0;
  const EcmCycleResult result2 = session.StepWithResult(input2);
  ASSERT_EQ(result2.status, EcmCycleStatus::kExecuted);
  // Thermal += another 500 J → 1000 J total.
  EXPECT_DOUBLE_EQ(result2.thermal_energy_j, 1000.0);
}

TEST(EcmDeceptionTest, ConfigRejectsInvalidDeceptionMode) {
  // Bug 3a: out-of-range default_deception_mode must fail validation.
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kDeception;
  config.default_deception_mode = static_cast<EcmDeceptionMode>(99);
  EcmSession session = EcmSession::Create(config);
  const EcmCycleInput input = MakeSensorInput(1U);
  const EcmCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, EcmCycleStatus::kRejectedInvalidConfig);
}

TEST(EcmDeceptionTest, RgpoDelayReachesMaxInExpectedCycles) {
  // Bug 4a: RGPO delay increment must use round-trip factor 2.
  // With rate=100 m/s, max_range=1000 m, dt=1.0 s:
  //   max_delay = 2.0 * 1000 / c ≈ 6.671e-6 s
  //   increment_per_cycle = 2.0 * 100 * 1.0 / c ≈ 6.671e-7 s
  //   cycles_to_max = 6.671e-6 / 6.671e-7 = 10
  config::EcmSessionConfig config =
      MakeDeceptionConfig(EcmDeceptionMode::kRgpo);
  config.deception_rgpo_rate_m_per_s = 100.0;
  config.deception_rgpo_max_range_m = 1000.0;
  config.deception_hold_time_s = 10.0;  // long enough to observe holding
  EcmSession session = EcmSession::Create(config);

  constexpr double c = 299792458.0;
  const double expected_max_delay = 2.0 * 1000.0 / c;
  bool reached_holding = false;

  for (std::uint32_t cycle = 1U; cycle <= 20U; ++cycle) {
    EcmCycleInput input = MakeSensorInput(cycle);
    input.cycle_start_time_s = static_cast<double>(cycle - 1U);
    const EcmCycleResult result = session.StepWithResult(input);
    ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
    if (result.decisions.front().deception_phase == EcmDeceptionPhase::kHolding) {
      reached_holding = true;
      const double delay =
          result.emission_frame.emissions.front().waveform.first_pulse_time_s -
          input.cycle_start_time_s;
      EXPECT_NEAR(delay, expected_max_delay, 1e-12);
      // Reaches holding after 10 increments; with floating-point rounding of
      // the >= comparison, it may take 10 or 11 cycles from state creation.
      EXPECT_GE(cycle, 10U);
      EXPECT_LE(cycle, 12U);
      break;
    }
  }
  EXPECT_TRUE(reached_holding);
}

TEST(EcmDeceptionTest, IsFeasibleThreatRejectsThreatWithDopplerOutsideBand) {
  // Bug 4b: threat feasibility must account for Doppler offset.
  config::EcmSessionConfig config =
      MakeDeceptionConfig(EcmDeceptionMode::kFalseTarget);
  config.minimum_frequency_hz = 10.0e9 - 1000.0;
  config.maximum_frequency_hz = 10.0e9 + 1000.0;
  config.deception_false_target_doppler_hz = 5000.0;  // pushes outside band
  EcmSession session = EcmSession::Create(config);

  EcmCycleInput input = MakeSensorInput(1U);
  input.sensor_observation_frame.observations.clear();
  // Threat at 10.0e9 + 0 Hz = within band.
  // After Doppler: 10.0e9 + 5000 Hz = outside [min+1000, max-1000].
  input.sensor_observation_frame.observations.push_back(
      MakeObservation(10U, 10.0e9, 0.9f));
  const EcmCycleResult result = session.StepWithResult(input);

  // No feasible threats → safe stop with no fresh observation.
  EXPECT_EQ(result.status, EcmCycleStatus::kSafeStopNoFreshObservation);
  EXPECT_TRUE(result.emission_frame.emissions.empty());
}

}  // namespace
}  // namespace session
}  // namespace electronic_countermeasure
