// @file ArSessionConfigValidation.cpp
// @brief Session-config validation for airborne radar.

#include "1q/airborne_radar/config/ArSessionConfigValidation.h"

#include <cstdint>
#include <set>

#include "1q/airborne_radar/session/ArIssueCodes.h"
#include "common/validation/ValidationUtils.h"

namespace airborne_radar {
namespace config {

session::ArIssueList ValidateArSessionConfig(const config::ArSessionConfig& config) noexcept {
  session::ArIssueList issues;
  const auto push = [&issues](const char* code, const char* field, const char* msg) {
    session::ArIssue issue;
    issue.severity = session::ArIssueSeverity::kError;
    issue.phase = session::ArIssuePhase::kInputValidation;
    issue.code = code;
    issue.field = field;
    issue.message = msg;
    issues.push_back(issue);
  };
  const config::ArOrientationConfig& orientation = config.orientation;
  const config::ArMissionConfig& mission = config.mission;
  const config::detection::AntennaConfig& antenna = config.hardware.antenna;
  const config::detection::ReceiverConfig& receiver = config.hardware.receiver;
  const config::detection::TransmitterConfig& transmitter = config.hardware.transmitter;
  const float transmitter_frequency_hz = transmitter.frequency_hz;

  if (!oneq::common::validation::IsFinite(transmitter_frequency_hz) ||
      transmitter_frequency_hz <= 0.0f) {
    push(session::codes::kTransmitterFrequencyInvalid, "hardware.transmitter.frequency_hz",
         "Transmitter frequency must be finite and positive.");
  }
  bool frequency_plan_valid = !transmitter.frequency_plan_hz.empty();
  bool contains_initial_frequency = false;
  for (double frequency_hz : transmitter.frequency_plan_hz) {
    frequency_plan_valid = frequency_plan_valid &&
                           oneq::common::validation::IsFinite(frequency_hz) && frequency_hz > 0.0;
    contains_initial_frequency =
        contains_initial_frequency || frequency_hz == static_cast<double>(transmitter_frequency_hz);
  }
  if (!frequency_plan_valid || !contains_initial_frequency) {
    push(session::codes::kFrequencyPlanInvalid, "hardware.transmitter.frequency_plan_hz",
         "Frequency plan must contain finite positive values and the initial carrier.");
  }
  const double duty_cycle =
      static_cast<double>(transmitter.pulse_width_s) * static_cast<double>(transmitter.prf_hz);
  const double pulse_energy_j = static_cast<double>(transmitter.peak_power_w) *
                                static_cast<double>(transmitter.pulse_width_s);
  if (!oneq::common::validation::IsFinite(transmitter.peak_power_w) ||
      !oneq::common::validation::IsFinite(transmitter.maximum_peak_power_w) ||
      !oneq::common::validation::IsFinite(transmitter.maximum_duty_cycle) ||
      !oneq::common::validation::IsFinite(transmitter.maximum_pulse_energy_j) ||
      transmitter.peak_power_w <= 0.0f || transmitter.maximum_peak_power_w <= 0.0f ||
      transmitter.peak_power_w > transmitter.maximum_peak_power_w ||
      transmitter.maximum_duty_cycle <= 0.0f || transmitter.maximum_duty_cycle > 1.0f ||
      duty_cycle <= 0.0 || duty_cycle > transmitter.maximum_duty_cycle ||
      transmitter.maximum_pulse_energy_j <= 0.0f ||
      pulse_energy_j > transmitter.maximum_pulse_energy_j) {
    push(session::codes::kTransmitterOperatingEnvelopeInvalid, "hardware.transmitter",
         "Transmitter power, duty cycle and pulse energy must stay inside hardware limits.");
  }
  if (!oneq::common::validation::IsFinite(receiver.cross_polarization_isolation_db) ||
      receiver.cross_polarization_isolation_db < 0.0f ||
      !oneq::common::validation::IsFinite(receiver.minimum_far_field_range_m) ||
      receiver.minimum_far_field_range_m <= 0.0f ||
      (receiver.has_co_site_isolation &&
       (!oneq::common::validation::IsFinite(receiver.co_site_isolation_db) ||
        receiver.co_site_isolation_db < 0.0f)) ||
      !oneq::common::validation::IsFinite(receiver.maximum_linear_input_power_w) ||
      receiver.maximum_linear_input_power_w <= 0.0f ||
      !oneq::common::validation::IsFinite(receiver.preselector_bandwidth_hz) ||
      receiver.preselector_bandwidth_hz <= 0.0f ||
      !oneq::common::validation::IsFinite(receiver.interference_observation_jn_gate_db)) {
    push(session::codes::kReceiverRfHardwareInvalid, "hardware.receiver",
         "Receiver RF isolation, far-field range and linear input limit must be valid.");
  }
  std::set<std::uint32_t> co_site_tx_ids;
  for (const auto& path : receiver.co_site_paths) {
    if (path.transmitter_sensor_entity_id == 0U ||
        !oneq::common::validation::IsFinite(path.isolation_db) || path.isolation_db < 0.0 ||
        !co_site_tx_ids.insert(path.transmitter_sensor_entity_id).second) {
      push(session::codes::kReceiverRfHardwareInvalid, "hardware.receiver.co_site_paths",
           "Each co-site path must specify a unique, valid non-zero transmitter entity and non-negative isolation.");
      break;
    }
  }
  // 信号处理增益偏置：有限且 [0, 40] dB。
  {
    const config::detection::SignalProcessingConfig& gains = config.hardware.signal_processing;
    if (!oneq::common::validation::IsFinite(gains.target_processing_gain_db) ||
        !oneq::common::validation::IsFinite(gains.noise_processing_gain_db) ||
        !oneq::common::validation::IsFinite(gains.clutter_suppression_gain_db) ||
        !oneq::common::validation::IsFinite(gains.jamming_suppression_gain_db) ||
        gains.target_processing_gain_db < 0.0f ||
        gains.target_processing_gain_db > 40.0f ||
        gains.noise_processing_gain_db < 0.0f || gains.noise_processing_gain_db > 40.0f ||
        gains.clutter_suppression_gain_db < 0.0f ||
        gains.clutter_suppression_gain_db > 40.0f ||
        gains.jamming_suppression_gain_db < 0.0f ||
        gains.jamming_suppression_gain_db > 40.0f) {
      push(session::codes::kSignalProcessingGainsInvalid, "hardware.signal_processing",
           "Signal processing gain offsets must be finite values in [0, 40] dB.");
    }
  }
  const auto axis_geometry_valid = [transmitter_frequency_hz](float nominal_beamwidth_deg,
                                                              float aperture_m,
                                                              bool commanded_override_enabled) {
    if (!oneq::common::validation::IsFinite(nominal_beamwidth_deg) ||
        !oneq::common::validation::IsFinite(aperture_m) || nominal_beamwidth_deg < 0.0f ||
        aperture_m < 0.0f) {
      return false;
    }
    if (commanded_override_enabled || nominal_beamwidth_deg > 0.0f) {
      return true;
    }
    return aperture_m > 0.0f && oneq::common::validation::IsFinite(transmitter_frequency_hz) &&
           transmitter_frequency_hz > 0.0f;
  };

  if (mission.commanded_beamwidth_enabled) {
    if (!oneq::common::validation::IsFinite(
            mission.commanded_beamwidth_deg.commanded_az_beamwidth_deg) ||
        mission.commanded_beamwidth_deg.commanded_az_beamwidth_deg <= 0.0f) {
      push(session::codes::kCommandedBeamwidthAzNotPositive,
           "mission.commanded_beamwidth_deg.commanded_az_beamwidth_deg",
           "Commanded azimuth beamwidth must be finite and positive when enabled.");
    }
    if (!oneq::common::validation::IsFinite(
            mission.commanded_beamwidth_deg.commanded_el_beamwidth_deg) ||
        mission.commanded_beamwidth_deg.commanded_el_beamwidth_deg <= 0.0f) {
      push(session::codes::kCommandedBeamwidthElNotPositive,
           "mission.commanded_beamwidth_deg.commanded_el_beamwidth_deg",
           "Commanded elevation beamwidth must be finite and positive when enabled.");
    }
  }

  if (!axis_geometry_valid(antenna.nominal_az_beamwidth_deg, antenna.antenna_length_m,
                           mission.commanded_beamwidth_enabled)) {
    push(session::codes::kAntennaAzGeometryInvalid,
         "hardware.antenna.nominal_az_beamwidth_deg / antenna_length_m",
         "Azimuth beamwidth requires a positive nominal value or a valid physical aperture.");
  }
  if (!axis_geometry_valid(antenna.nominal_el_beamwidth_deg, antenna.antenna_width_m,
                           mission.commanded_beamwidth_enabled)) {
    push(session::codes::kAntennaElGeometryInvalid,
         "hardware.antenna.nominal_el_beamwidth_deg / antenna_width_m",
         "Elevation beamwidth requires a positive nominal value or a valid physical aperture.");
  }

  if (orientation.mechanical_scan_limits_deg.az_min_deg >
      orientation.mechanical_scan_limits_deg.az_max_deg) {
    push(session::codes::kMechanicalScanLimitsSwappedAz, "orientation.mechanical_scan_limits_deg",
         "Mechanical azimuth scan min exceeds max.");
  }
  if (orientation.mechanical_scan_limits_deg.el_min_deg >
      orientation.mechanical_scan_limits_deg.el_max_deg) {
    push(session::codes::kMechanicalScanLimitsSwappedEl, "orientation.mechanical_scan_limits_deg",
         "Mechanical elevation scan min exceeds max.");
  }

  if (orientation.electronic_scan_limits_deg.az_min_deg >
      orientation.electronic_scan_limits_deg.az_max_deg) {
    push(session::codes::kElectronicScanLimitsSwappedAz, "orientation.electronic_scan_limits_deg",
         "Electronic azimuth scan min exceeds max.");
  }
  if (orientation.electronic_scan_limits_deg.el_min_deg >
      orientation.electronic_scan_limits_deg.el_max_deg) {
    push(session::codes::kElectronicScanLimitsSwappedEl, "orientation.electronic_scan_limits_deg",
         "Electronic elevation scan min exceeds max.");
  }

  return issues;
}

}  // namespace config
}  // namespace airborne_radar
