/**
 * @file EcmConfigValidator.cpp
 * @brief EcmConfigValidator 纯校验函数实现。
 */

#include "electronic_countermeasure/EcmConfigValidator.h"

#include <algorithm>
#include <cmath>
#include <set>
#include "common/numerics/Constants.h"

namespace electronic_countermeasure {
namespace session {

bool EcmConfigValidator::IsKnownTechnique(EcmTechnique technique) {
  return technique == EcmTechnique::kSpot || technique == EcmTechnique::kBarrage ||
         technique == EcmTechnique::kSweep || technique == EcmTechnique::kDeception;
}

bool EcmConfigValidator::IsKnownDeceptionMode(EcmDeceptionMode mode) {
  return mode == EcmDeceptionMode::kRgpo || mode == EcmDeceptionMode::kVgpo ||
         mode == EcmDeceptionMode::kRgpoVgpo || mode == EcmDeceptionMode::kFalseTarget;
}

bool EcmConfigValidator::IsValidConfig(const config::EcmSessionConfig& config) {
  return config.channel_count > 0U &&
         std::isfinite(config.minimum_frequency_hz) &&
         std::isfinite(config.maximum_frequency_hz) && config.minimum_frequency_hz > 0.0 &&
         config.maximum_frequency_hz > config.minimum_frequency_hz &&
         std::isfinite(config.maximum_total_transmit_power_w) &&
         config.maximum_total_transmit_power_w > 0.0 &&
         std::isfinite(config.maximum_channel_transmit_power_w) &&
         config.maximum_channel_transmit_power_w > 0.0 &&
         config.maximum_channel_transmit_power_w <= config.maximum_total_transmit_power_w &&
         std::isfinite(config.thermal_capacity_j) && config.thermal_capacity_j > 0.0 &&
         std::isfinite(config.cooling_power_w) && config.cooling_power_w >= 0.0 &&
         std::isfinite(config.spot_bandwidth_hz) && config.spot_bandwidth_hz > 0.0 &&
         std::isfinite(config.barrage_bandwidth_hz) && config.barrage_bandwidth_hz > 0.0 &&
         std::isfinite(config.sweep_bandwidth_hz) && config.sweep_bandwidth_hz > 0.0 &&
         config.sweep_segment_count > 0U && IsKnownTechnique(config.default_technique) &&
         std::isfinite(config.deception_rgpo_rate_m_per_s) &&
         config.deception_rgpo_rate_m_per_s >= 0.0 &&
         std::isfinite(config.deception_rgpo_max_range_m) &&
         config.deception_rgpo_max_range_m >= 0.0 &&
         std::isfinite(config.deception_vgpo_rate_hz_per_s) &&
         config.deception_vgpo_rate_hz_per_s >= 0.0 &&
         std::isfinite(config.deception_vgpo_max_doppler_hz) &&
         config.deception_vgpo_max_doppler_hz >= 0.0 &&
         std::isfinite(config.deception_hold_time_s) && config.deception_hold_time_s >= 0.0 &&
         std::isfinite(config.deception_power_scale) && config.deception_power_scale > 0.0 &&
         config.deception_power_scale <= 1.0 && config.deception_max_active > 0U &&
         std::isfinite(config.deception_false_target_delay_s) &&
         config.deception_false_target_delay_s >= 0.0 &&
         std::isfinite(config.deception_false_target_doppler_hz) &&
         config.deception_max_false_targets_per_threat > 0U &&
         IsKnownDeceptionMode(config.default_deception_mode);
}

bool EcmConfigValidator::IsValidSensorObservation(const EcmSensorObservation& observation) {
  return std::isfinite(observation.estimated_center_frequency_hz) &&
         observation.estimated_center_frequency_hz > 0.0 &&
         std::isfinite(observation.estimated_bandwidth_hz) &&
         observation.estimated_bandwidth_hz > 0.0 &&
         std::isfinite(observation.estimated_pri_s) && observation.estimated_pri_s >= 0.0 &&
         std::isfinite(observation.estimated_pulse_width_s) &&
         observation.estimated_pulse_width_s >= 0.0 &&
         std::isfinite(observation.center_frequency_std_hz) &&
         observation.center_frequency_std_hz >= 0.0 &&
         std::isfinite(observation.bandwidth_std_hz) && observation.bandwidth_std_hz >= 0.0 &&
         std::isfinite(observation.bearing_az_deg) && std::isfinite(observation.bearing_el_deg) &&
         std::isfinite(observation.bearing_std_deg) && observation.bearing_std_deg >= 0.0 &&
         std::isfinite(observation.threat_score) && observation.threat_score >= 0.0f &&
         observation.threat_score <= 1.0f && std::isfinite(observation.confidence) &&
         observation.confidence >= 0.0f && observation.confidence <= 1.0f;
}

bool EcmConfigValidator::IsValidInput(const EcmCycleInput& input) {
  if (input.cycle_index == 0U ||
      !std::isfinite(input.cycle_start_time_s) || input.cycle_start_time_s < 0.0 ||
      !std::isfinite(input.dt_sec) || input.dt_sec <= 0.0 ||
      !std::isfinite(input.platform_position_ecef_m.x_m) ||
      !std::isfinite(input.platform_position_ecef_m.y_m) ||
      !std::isfinite(input.platform_position_ecef_m.z_m) ||
      !std::isfinite(input.platform_velocity_ecef_mps.x_mps) ||
      !std::isfinite(input.platform_velocity_ecef_mps.y_mps) ||
      !std::isfinite(input.platform_velocity_ecef_mps.z_mps)) {
    return false;
  }
  if (input.input_mode == EcmInputMode::kSensorDriven) {
    if (!input.truth_threats.empty() ||
        (!input.has_sensor_observation_frame &&
         (!input.sensor_observation_frame.observations.empty() ||
          input.sensor_observation_frame.source_esr_batch_id != 0U)) ||
        (input.has_sensor_observation_frame &&
         input.sensor_observation_frame.source_esr_batch_id == 0U)) {
      return false;
    }
    std::set<std::uint64_t> ids;
    for (const EcmSensorObservation& observation : input.sensor_observation_frame.observations) {
      if (!IsValidSensorObservation(observation) ||
          !ids.insert(observation.source_hypothesis_id).second) {
        return false;
      }
    }
    return true;
  }
  if (input.input_mode != EcmInputMode::kTruthAssisted || input.has_sensor_observation_frame ||
      !input.sensor_observation_frame.observations.empty()) {
    return false;
  }
  std::set<std::uint64_t> ids;
  for (const EcmTruthThreat& threat : input.truth_threats) {
    if (!ids.insert(threat.truth_entity_id).second || !std::isfinite(threat.center_frequency_hz) ||
        threat.center_frequency_hz <= 0.0 || !std::isfinite(threat.bandwidth_hz) ||
        threat.bandwidth_hz <= 0.0 || !std::isfinite(threat.threat_score) ||
        threat.threat_score < 0.0f || threat.threat_score > 1.0f ||
        !std::isfinite(threat.estimated_pri_s) || threat.estimated_pri_s <= 0.0 ||
        !std::isfinite(threat.estimated_pulse_width_s) || threat.estimated_pulse_width_s < 0.0) {
      return false;
    }
  }
  return true;
}

bool EcmConfigValidator::TryMergePatch(const config::EcmSessionConfig& current,
                                       const config::EcmRuntimeConfigPatch& patch,
                                       config::EcmSessionConfig* candidate) {
  if (candidate == nullptr) {
    return false;
  }
  if (!patch.has_power_on && !patch.has_maximum_total_transmit_power_w &&
      !patch.has_default_technique && !patch.has_default_deception_mode) {
    return false;
  }
  *candidate = current;
  if (patch.has_power_on) {
    candidate->power_on = patch.power_on;
  }
  if (patch.has_maximum_total_transmit_power_w) {
    candidate->maximum_total_transmit_power_w = patch.maximum_total_transmit_power_w;
  }
  if (patch.has_default_technique) {
    candidate->default_technique = patch.default_technique;
  }
  if (patch.has_default_deception_mode) {
    candidate->default_deception_mode = patch.default_deception_mode;
  }
  return IsValidConfig(*candidate);
}

bool EcmConfigValidator::IsSnapshotInternallyConsistent(const EcmRuntimeState& state) {
  if (state.has_successful_cycle == (state.last_successful_cycle_index == 0U)) {
    return false;
  }
  if (!state.has_last_sensor_frame) {
    if (!state.last_sensor_frame.observations.empty() ||
        state.observation_age_successful_ecm_cycles != 0U) {
      return false;
    }
  } else {
    std::set<std::uint64_t> hypothesis_ids;
    for (const EcmSensorObservation& observation : state.last_sensor_frame.observations) {
      if (!IsValidSensorObservation(observation) ||
          !hypothesis_ids.insert(observation.source_hypothesis_id).second) {
        return false;
      }
    }
  }
  if (!state.deception_states.empty() &&
      (!state.active_config.power_on ||
       state.active_config.default_technique != EcmTechnique::kDeception)) {
    return false;
  }
  for (const EcmDeceptionState& deception_state : state.deception_states) {
    if (!IsKnownDeceptionMode(deception_state.mode)) {
      return false;
    }
    if (deception_state.phase != EcmDeceptionPhase::kIdle &&
        deception_state.phase != EcmDeceptionPhase::kTowing &&
        deception_state.phase != EcmDeceptionPhase::kHolding &&
        deception_state.phase != EcmDeceptionPhase::kStopped) {
      return false;
    }
    if (!deception_state.engaged ||
        deception_state.mode != state.active_config.default_deception_mode ||
        deception_state.current_delay_s < 0.0 || deception_state.phase_elapsed_s < 0.0 ||
        !std::isfinite(deception_state.current_delay_s) ||
        !std::isfinite(deception_state.current_doppler_offset_hz) ||
        !std::isfinite(deception_state.phase_elapsed_s) ||
        deception_state.phase == EcmDeceptionPhase::kIdle) {
      return false;
    }
    const double c = oneq::common::numerics::kLightSpeed;
    const double max_delay_s = 2.0 * state.active_config.deception_rgpo_max_range_m / c;
    switch (deception_state.mode) {
      case EcmDeceptionMode::kRgpo:
        if (deception_state.current_delay_s > max_delay_s ||
            deception_state.current_doppler_offset_hz != 0.0) {
          return false;
        }
        break;
      case EcmDeceptionMode::kVgpo:
        if (deception_state.current_delay_s != 0.0 ||
            std::fabs(deception_state.current_doppler_offset_hz) >
                state.active_config.deception_vgpo_max_doppler_hz) {
          return false;
        }
        break;
      case EcmDeceptionMode::kRgpoVgpo:
        if (deception_state.current_delay_s > max_delay_s ||
            std::fabs(deception_state.current_doppler_offset_hz) >
                state.active_config.deception_vgpo_max_doppler_hz) {
          return false;
        }
        break;
      case EcmDeceptionMode::kFalseTarget:
        if (deception_state.phase != EcmDeceptionPhase::kTowing ||
            deception_state.current_delay_s != 0.0 ||
            deception_state.current_doppler_offset_hz != 0.0) {
          return false;
        }
        break;
    }
  }
  {
    const std::size_t engaged_count = static_cast<std::size_t>(
        std::count_if(state.deception_states.begin(), state.deception_states.end(),
                      [](const EcmDeceptionState& s) { return s.engaged; }));
    if (engaged_count > static_cast<std::size_t>(state.active_config.deception_max_active)) {
      return false;
    }
  }
  {
    std::set<std::uint64_t> engaged_ids;
    for (const EcmDeceptionState& deception_state : state.deception_states) {
      if (deception_state.engaged) {
        if (!engaged_ids.insert(deception_state.threat_id).second) {
          return false;
        }
      }
    }
  }
  return true;
}

}  // namespace session
}  // namespace electronic_countermeasure
