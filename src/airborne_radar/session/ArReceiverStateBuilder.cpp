#include "airborne_radar/session/ArReceiverStateBuilder.h"

#include <algorithm>

namespace airborne_radar {
namespace session {

ArReceiverOperatingState ArReceiverStateBuilder::Build(
    const ArPrepareCycleInput& input, const oneq::electromagnetics::RfSceneEmission& emission,
    const config::engineering::DetectionConfig& detection, const ArControlProfile& control_profile,
    double carrier_hz) {
  const config::engineering::ReceiverConfig& receiver = detection.receiver;
  const config::engineering::TransmitterConfig& transmitter = detection.transmitter;

  ArReceiverOperatingState operating_state;
  oneq::electromagnetics::RfSceneReceiverState& receiver_state = operating_state.rf_receiver;
  receiver_state.sensor_entity_id = input.sensor_entity_id;
  receiver_state.position_ecef_m = input.platform_position_ecef_m;
  receiver_state.velocity_ecef_mps = input.platform_velocity_ecef_mps;
  receiver_state.antenna = emission.antenna;
  if (control_profile.enable_sidelobe_canceller) {
    receiver_state.antenna.sidelobe_level_db -= 12.0;
    receiver_state.antenna.backlobe_level_db = std::min(receiver_state.antenna.backlobe_level_db,
                                                        receiver_state.antenna.sidelobe_level_db);
  }
  if (control_profile.enable_adaptive_beamforming) {
    receiver_state.antenna.half_power_beamwidth_deg *= 0.75;
    receiver_state.antenna.sidelobe_level_db -= 6.0;
    receiver_state.antenna.backlobe_level_db = std::min(receiver_state.antenna.backlobe_level_db,
                                                        receiver_state.antenna.sidelobe_level_db);
  }
  receiver_state.polarization = receiver.scene_polarization;
  receiver_state.window_start_time_s = input.window_start_time_s;
  receiver_state.window_duration_s = input.window_duration_s;
  receiver_state.center_frequency_hz = carrier_hz;
  receiver_state.bandwidth_hz = static_cast<double>(receiver.preselector_bandwidth_hz);
  receiver_state.receiver_system_loss_db = static_cast<double>(receiver.receive_loss_db);
  receiver_state.minimum_far_field_range_m =
      static_cast<double>(receiver.minimum_far_field_range_m);
  receiver_state.co_site_paths = receiver.co_site_paths;
  if (input.sensor_entity_id != 1U) {
    for (auto& path : receiver_state.co_site_paths) {
      if (path.transmitter_sensor_entity_id == 1U) {
        path.transmitter_sensor_entity_id = input.sensor_entity_id;
      }
    }
  }
  operating_state.beam_pointing_deg = input.beam_pointing_deg;
  operating_state.matched_filter_bandwidth_hz = static_cast<double>(transmitter.bandwidth_hz);
  operating_state.receiver_noise_figure_db = static_cast<double>(receiver.noise_figure_db);
  operating_state.maximum_linear_input_power_w =
      static_cast<double>(receiver.maximum_linear_input_power_w);
  return operating_state;
}

}  // namespace session
}  // namespace airborne_radar
