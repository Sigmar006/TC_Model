/**
 * @file RirDetectionCellResolver.cpp
 * @brief RIR detection cell 薄适配：转调 common DetectionCellResolver（anti_rgpo=false）。
 */

#include "remote_identification_radar/dwell/RirDetectionCellResolver.h"

#include "common/radar/DetectionCellResolver.h"

namespace remote_identification_radar {
namespace dwell {
namespace {

oneq::common::radar::DetectionCellConfig ToCommon(const RirDetectionCellConfig& config) {
  oneq::common::radar::DetectionCellConfig out;
  out.own_transmit_waveform = config.own_transmit_waveform;
  out.receive_window_start_time_s = config.receive_window_start_time_s;
  out.receive_window_duration_s = config.receive_window_duration_s;
  out.matched_filter_bandwidth_hz = config.matched_filter_bandwidth_hz;
  out.one_way_antenna_gain_dbi = config.one_way_antenna_gain_dbi;
  out.receiver_loss_db = config.receiver_loss_db;
  out.receiver_noise_figure_db = config.receiver_noise_figure_db;
  out.reference_temperature_k = config.reference_temperature_k;
  out.enable_anti_rgpo_leading_edge = false;
  out.signal_processing.target_processing_gain_db =
      static_cast<double>(config.signal_processing.target_processing_gain_db);
  out.signal_processing.noise_processing_gain_db =
      static_cast<double>(config.signal_processing.noise_processing_gain_db);
  out.signal_processing.clutter_suppression_gain_db =
      static_cast<double>(config.signal_processing.clutter_suppression_gain_db);
  out.signal_processing.jamming_suppression_gain_db =
      static_cast<double>(config.signal_processing.jamming_suppression_gain_db);
  return out;
}

oneq::common::radar::DetectionCellTarget ToCommon(const RirDetectionCellTarget& target) {
  oneq::common::radar::DetectionCellTarget out;
  out.range_m = target.range_m;
  out.closing_radial_velocity_mps = target.closing_radial_velocity_mps;
  out.rcs_m2 = target.rcs_m2;
  out.two_way_additional_propagation_loss_db = target.two_way_additional_propagation_loss_db;
  out.effective_pulse_count = target.effective_pulse_count;
  return out;
}

void FromCommon(const oneq::common::radar::DetectionCellResult& in, RirDetectionCellResult* out) {
  out->echo_delay_s = in.echo_delay_s;
  out->two_way_doppler_shift_hz = in.two_way_doppler_shift_hz;
  out->echo_power_w = in.echo_power_w;
  out->pulse_compression_gain = in.pulse_compression_gain;
  out->thermal_noise_power_w = in.thermal_noise_power_w;
  out->interference_power_w = in.interference_power_w;
  out->clutter_power_w = in.clutter_power_w;
  out->processed_single_pulse_sinr_linear = in.processed_single_pulse_sinr_linear;
  out->processed_single_pulse_sinr_db = in.processed_single_pulse_sinr_db;
  out->effective_pulse_count = in.effective_pulse_count;
}

}  // namespace

bool TryResolveRirDetectionCell(
    const RirDetectionCellConfig& config, const RirDetectionCellTarget& target,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double clutter_power_w, RirDetectionCellResult* result) {
  if (result == nullptr) {
    return false;
  }
  oneq::common::radar::DetectionCellResult common_result;
  if (!oneq::common::radar::TryResolveDetectionCell(ToCommon(config), ToCommon(target),
                                                    own_emission_identity, incident_links,
                                                    clutter_power_w, &common_result)) {
    return false;
  }
  FromCommon(common_result, result);
  return true;
}

}  // namespace dwell
}  // namespace remote_identification_radar
