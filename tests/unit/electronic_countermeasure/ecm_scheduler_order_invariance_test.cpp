/**
 * @file ecm_scheduler_order_invariance_test.cpp
 * @brief Property-style tests: scheduler randomness must be independent of threat
 *        input order (design §3 hard constraint — separate RNG streams).
 *
 * The codebase has no property-test framework (gtest only), so these tests
 * enumerate the full permutation set by hand, mirroring
 * tests/integration/airborne_radar/ar_session_test.cpp input-reordering idiom.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <map>
#include <vector>

#include "1q/electronic_countermeasure/EcmSession.h"

namespace electronic_countermeasure {
namespace session {
namespace {

// Per-threat sweep result keyed by the threat's center frequency. For a linear
// sweep the waveform leaves center_frequency_hz unset, but the midpoint of the
// sweep span equals the threat center regardless of reverse direction, so we use
// (sweep_start + sweep_stop) / 2 as the disambiguating key per threat.
struct ThreatSweep {
  double threat_center_hz{0.0};
  double sweep_start_frequency_hz{0.0};
  double sweep_stop_frequency_hz{0.0};
};

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

// Build a sensor-driven input carrying the given observations in the supplied order.
EcmCycleInput MakeSensorInput(std::uint32_t cycle_index,
                              const std::vector<EcmSensorObservation>& observations) {
  EcmCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = static_cast<double>(cycle_index - 1U);
  input.dt_sec = 1.0;
  input.input_mode = EcmInputMode::kSensorDriven;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.has_sensor_observation_frame = true;
  input.sensor_observation_frame.source_esr_batch_id = static_cast<std::uint64_t>(cycle_index - 1U);
  input.sensor_observation_frame.observations = observations;
  return input;
}

// Index a cycle result's sweep fields by threat center frequency, derived as the
// midpoint of each emission's sweep span (reverse-independent).
std::map<double, ThreatSweep> IndexSweepByThreatCenter(const EcmCycleResult& result) {
  std::map<double, ThreatSweep> sweeps;
  for (const oneq::electromagnetics::RfSceneEmission& emission :
       result.emission_frame.emissions) {
    ThreatSweep sweep;
    sweep.threat_center_hz = 0.5 * (emission.waveform.sweep_start_frequency_hz +
                                    emission.waveform.sweep_stop_frequency_hz);
    sweep.sweep_start_frequency_hz = emission.waveform.sweep_start_frequency_hz;
    sweep.sweep_stop_frequency_hz = emission.waveform.sweep_stop_frequency_hz;
    sweeps[sweep.threat_center_hz] = sweep;
  }
  return sweeps;
}

// Run one cycle for a given observation ordering and return the per-threat sweep map.
std::map<double, ThreatSweep> RunOneCycle(std::uint32_t random_seed,
                                          const std::vector<EcmSensorObservation>& observations) {
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kSweep;
  config.random_seed = random_seed;
  EcmSession session = EcmSession::Create(config);
  const EcmCycleResult result = session.StepWithResult(MakeSensorInput(2U, observations));
  EXPECT_EQ(result.status, EcmCycleStatus::kExecuted);
  EXPECT_EQ(result.emission_frame.emissions.size(), observations.size());
  return IndexSweepByThreatCenter(result);
}

// (Re)normalize a permutation to a fixed canonical order before enumerating the
// full set via std::next_permutation. Returns all factoral(N) orderings.
std::vector<std::vector<EcmSensorObservation>> AllPermutations(
    std::vector<EcmSensorObservation> observations) {
  std::sort(observations.begin(), observations.end(),
            [](const EcmSensorObservation& lhs, const EcmSensorObservation& rhs) {
              return lhs.source_hypothesis_id < rhs.source_hypothesis_id;
            });
  std::vector<std::vector<EcmSensorObservation>> permutations;
  do {
    permutations.push_back(observations);
  } while (std::next_permutation(
      observations.begin(), observations.end(),
      [](const EcmSensorObservation& lhs, const EcmSensorObservation& rhs) {
        return lhs.source_hypothesis_id < rhs.source_hypothesis_id;
      }));
  return permutations;
}

// Three distinct-score threats. All three are feasible (well inside the default
// hardware band), so the scheduling_rng draws fire once per threat and the
// sweep direction is the observable that must be order-independent.
TEST(EcmSchedulerOrderInvariance, DistinctScoresKeepSweepSequenceAcrossPermutations) {
  std::vector<EcmSensorObservation> observations;
  observations.push_back(MakeObservation(10U, 10.0e9, 0.9f));
  observations.push_back(MakeObservation(11U, 9.0e9, 0.8f));
  observations.push_back(MakeObservation(12U, 8.0e9, 0.7f));

  const std::vector<std::vector<EcmSensorObservation>> permutations =
      AllPermutations(observations);
  ASSERT_EQ(permutations.size(), 6U);

  const std::uint32_t random_seed = 2468U;
  std::map<double, ThreatSweep> baseline =
      RunOneCycle(random_seed, permutations.front());
  ASSERT_EQ(baseline.size(), 3U);

  for (std::size_t i = 1U; i < permutations.size(); ++i) {
    const std::map<double, ThreatSweep> reordered =
        RunOneCycle(random_seed, permutations[i]);
    ASSERT_EQ(reordered.size(), 3U);
    for (const auto& [center_hz, expected] : baseline) {
      const auto it = reordered.find(center_hz);
      ASSERT_NE(it, reordered.end()) << "threat missing in permutation " << i;
      EXPECT_DOUBLE_EQ(it->second.sweep_start_frequency_hz,
                       expected.sweep_start_frequency_hz)
          << "sweep start changed with input order at " << center_hz;
      EXPECT_DOUBLE_EQ(it->second.sweep_stop_frequency_hz,
                       expected.sweep_stop_frequency_hz)
          << "sweep stop changed with input order at " << center_hz;
    }
  }
}

// Three threats sharing the same score but distinct ids. The tie-break RNG alone
// decides their order, so this directly exercises the order-independence of
// AssignTieBreakKeys (the draw order must follow canonical id order, not input).
TEST(EcmSchedulerOrderInvariance, TiedScoresTieBreakIsOrderIndependent) {
  std::vector<EcmSensorObservation> observations;
  observations.push_back(MakeObservation(20U, 10.0e9, 0.85f));
  observations.push_back(MakeObservation(21U, 9.0e9, 0.85f));
  observations.push_back(MakeObservation(22U, 8.0e9, 0.85f));

  const std::vector<std::vector<EcmSensorObservation>> permutations =
      AllPermutations(observations);
  ASSERT_EQ(permutations.size(), 6U);

  const std::uint32_t random_seed = 1357U;
  const std::map<double, ThreatSweep> baseline =
      RunOneCycle(random_seed, permutations.front());
  ASSERT_EQ(baseline.size(), 3U);

  for (std::size_t i = 1U; i < permutations.size(); ++i) {
    const std::map<double, ThreatSweep> reordered =
        RunOneCycle(random_seed, permutations[i]);
    ASSERT_EQ(reordered.size(), 3U);
    for (const auto& [center_hz, expected] : baseline) {
      const auto it = reordered.find(center_hz);
      ASSERT_NE(it, reordered.end()) << "tied threat missing in permutation " << i;
      EXPECT_DOUBLE_EQ(it->second.sweep_start_frequency_hz,
                       expected.sweep_start_frequency_hz)
          << "tie-break sweep start leaked input order at " << center_hz;
      EXPECT_DOUBLE_EQ(it->second.sweep_stop_frequency_hz,
                       expected.sweep_stop_frequency_hz)
          << "tie-break sweep stop leaked input order at " << center_hz;
    }
  }
}

// Negative control: distinct seeds must actually produce a different sweep
// sequence, proving the test above is not trivially passing by coincidence
// (mirrors sar_geometry_model_test GaussianSamplerDifferentSeedsDiverge).
TEST(EcmSchedulerOrderInvariance, DifferentSeedsProduceDifferentSweepSequence) {
  std::vector<EcmSensorObservation> observations;
  observations.push_back(MakeObservation(10U, 10.0e9, 0.9f));
  observations.push_back(MakeObservation(11U, 9.0e9, 0.8f));
  observations.push_back(MakeObservation(12U, 8.0e9, 0.7f));

  const std::map<double, ThreatSweep> first = RunOneCycle(1U, observations);
  const std::map<double, ThreatSweep> second = RunOneCycle(2U, observations);
  ASSERT_EQ(first.size(), 3U);
  ASSERT_EQ(second.size(), 3U);

  bool any_differ = false;
  for (const auto& [center_hz, expected] : first) {
    const auto it = second.find(center_hz);
    ASSERT_NE(it, second.end());
    if (it->second.sweep_start_frequency_hz != expected.sweep_start_frequency_hz ||
        it->second.sweep_stop_frequency_hz != expected.sweep_stop_frequency_hz) {
      any_differ = true;
      break;
    }
  }
  EXPECT_TRUE(any_differ) << "sweep sequence did not depend on seed; RNG not exercised";
}

}  // namespace
}  // namespace session
}  // namespace electronic_countermeasure
