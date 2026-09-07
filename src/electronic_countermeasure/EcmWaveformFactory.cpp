/**
 * @file EcmWaveformFactory.cpp
 * @brief EcmWaveformFactory 实现：压制和欺骗波形构造。
 */

#include "electronic_countermeasure/EcmWaveformFactory.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>

namespace electronic_countermeasure {
namespace session {

bool EcmWaveformFactory::TryBuildEmission(const EcmCycleInput& input,
                                          const config::EcmSessionConfig& config,
                                          const SchedulingThreat& threat,
                                          double allocated_power_w, std::uint32_t channel_index,
                                          std::uint32_t sensor_entity_id,
                                          std::uint64_t emission_id, std::mt19937* scheduling_rng,
                                          oneq::electromagnetics::RfSceneEmission* output) {
  if (output == nullptr) {
    return false;
  }
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = sensor_entity_id;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m = input.platform_position_ecef_m;
  emission.velocity_ecef_mps = input.platform_velocity_ecef_mps;
  emission.antenna = input.transmit_antenna;
  emission.polarization = input.transmit_polarization;
  const EcmTechnique technique = config.default_technique;
  if (technique == EcmTechnique::kSweep) {
    bool reverse_sweep = false;
    if (scheduling_rng != nullptr) {
      std::uniform_int_distribution<std::uint32_t> distribution(0U, 1U);
      reverse_sweep = distribution(*scheduling_rng) != 0U;
    }
    double start_frequency_hz = threat.center_frequency_hz - 0.5 * config.sweep_bandwidth_hz;
    double stop_frequency_hz = threat.center_frequency_hz + 0.5 * config.sweep_bandwidth_hz;
    if (reverse_sweep) {
      std::swap(start_frequency_hz, stop_frequency_hz);
    }
    const double instantaneous_bandwidth_hz =
        config.sweep_bandwidth_hz / static_cast<double>(config.sweep_segment_count);
    if (!oneq::electromagnetics::TryCreateRfLinearSweepWaveform(
            input.cycle_start_time_s, input.dt_sec, start_frequency_hz, stop_frequency_hz,
            instantaneous_bandwidth_hz, allocated_power_w, input.dt_sec, &emission.waveform)) {
      return false;
    }
  } else {
    const double bandwidth_hz = technique == EcmTechnique::kBarrage
                                    ? std::max(config.barrage_bandwidth_hz, threat.bandwidth_hz)
                                    : config.spot_bandwidth_hz;
    if (!oneq::electromagnetics::TryCreateRfNoiseWaveform(input.cycle_start_time_s, input.dt_sec,
                                                          threat.center_frequency_hz, bandwidth_hz,
                                                          allocated_power_w, &emission.waveform)) {
      return false;
    }
  }
  (void)channel_index;
  *output = emission;
  return true;
}

bool EcmWaveformFactory::TryBuildDeceptionEmission(
    const EcmCycleInput& input, const config::EcmSessionConfig& config,
    const SchedulingThreat& threat, const EcmDeceptionState& deception_state,
    double allocated_power_w, std::uint32_t sensor_entity_id,
    std::uint64_t emission_id, std::mt19937* pulse_rng,
    oneq::electromagnetics::RfSceneEmission* output) {
  if (output == nullptr) {
    return false;
  }
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = sensor_entity_id;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m = input.platform_position_ecef_m;
  emission.velocity_ecef_mps = input.platform_velocity_ecef_mps;
  emission.antenna = input.transmit_antenna;
  emission.polarization = input.transmit_polarization;

  double center_frequency_hz =
      threat.center_frequency_hz + deception_state.current_doppler_offset_hz;
  double bandwidth_hz = std::max(1.0e6, threat.bandwidth_hz);
  double pri_s = std::max(1.0e-6, threat.estimated_pri_s);
  double pulse_width_s = std::min(threat.estimated_pulse_width_s, pri_s * 0.45);
  pulse_width_s = std::max(1.0e-9, pulse_width_s);
  double first_pulse_time_s = input.cycle_start_time_s + deception_state.current_delay_s;

  std::uint32_t pulse_count = static_cast<std::uint32_t>(std::floor(input.dt_sec / pri_s));
  if (pulse_count == 0U) {
    pulse_count = 1U;
  }
  double jitter_fraction = 0.05;
  std::uint64_t timing_seed = 0U;
  std::uint64_t timing_epoch = 0U;
  if (pulse_rng != nullptr) {
    std::uniform_int_distribution<std::uint64_t> dist;
    timing_seed = dist(*pulse_rng);
    timing_epoch = dist(*pulse_rng);
  }

  bool ok = oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      first_pulse_time_s, center_frequency_hz, bandwidth_hz, allocated_power_w, pulse_width_s,
      pri_s, pulse_count, jitter_fraction, timing_seed, timing_epoch, &emission.waveform);
  if (ok) {
    *output = emission;
  }
  return ok;
}

bool EcmWaveformFactory::TryBuildFalseTargetEmission(
    const EcmCycleInput& input, const config::EcmSessionConfig& config,
    const SchedulingThreat& threat, double allocated_power_w,
    std::uint32_t sensor_entity_id, std::uint64_t emission_id,
    std::mt19937* pulse_rng, std::uint32_t ft_index,
    oneq::electromagnetics::RfSceneEmission* output) {
  if (output == nullptr) {
    return false;
  }
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = sensor_entity_id;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m = input.platform_position_ecef_m;
  emission.velocity_ecef_mps = input.platform_velocity_ecef_mps;
  emission.antenna = input.transmit_antenna;
  emission.polarization = input.transmit_polarization;

  double center_frequency_hz =
      threat.center_frequency_hz + config.deception_false_target_doppler_hz;
  double bandwidth_hz = std::max(1.0e6, threat.bandwidth_hz);
  double pri_s = std::max(1.0e-6, threat.estimated_pri_s);
  double pulse_width_s = std::min(threat.estimated_pulse_width_s, pri_s * 0.45);
  pulse_width_s = std::max(1.0e-9, pulse_width_s);

  double stagger_offset_s = static_cast<double>(ft_index) * 1.0e-6;
  double first_pulse_time_s =
      input.cycle_start_time_s + config.deception_false_target_delay_s + stagger_offset_s;

  std::uint32_t pulse_count = static_cast<std::uint32_t>(std::floor(input.dt_sec / pri_s));
  if (pulse_count == 0U) {
    pulse_count = 1U;
  }
  double jitter_fraction = 0.05;
  std::uint64_t timing_seed = 0U;
  std::uint64_t timing_epoch = 0U;
  if (pulse_rng != nullptr) {
    std::uniform_int_distribution<std::uint64_t> dist;
    timing_seed = dist(*pulse_rng);
    timing_epoch = dist(*pulse_rng);
  }

  bool ok = oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      first_pulse_time_s, center_frequency_hz, bandwidth_hz, allocated_power_w, pulse_width_s,
      pri_s, pulse_count, jitter_fraction, timing_seed, timing_epoch, &emission.waveform);
  if (ok) {
    *output = emission;
  }
  return ok;
}

}  // namespace session
}  // namespace electronic_countermeasure
