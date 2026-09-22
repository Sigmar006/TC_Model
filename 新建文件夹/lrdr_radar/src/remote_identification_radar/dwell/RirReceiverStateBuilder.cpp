/*
 * Supplemental architecture notes for acceptance review.
 * Unit: RirReceiverStateBuilder.
 * Model: LRDR remote identification radar.
 * Layer: dwell.
 * File role: executable implementation.
 * Responsibility: per-dwell emission, receiver, beam, echo, and detection processing.
 * Upstream boundary: dwell plan, RF scene, target geometry, and radar hardware state.
 * Downstream boundary: resolved RF cells, detection decisions, and track measurements.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

#include "remote_identification_radar/dwell/RirReceiverStateBuilder.h"

namespace remote_identification_radar {
namespace dwell {
namespace {

constexpr double kDefaultPreselectorBandwidthHz = 20.0e6;
constexpr double kDefaultMinimumFarFieldRangeM = 1.0;
constexpr double kDefaultCoSiteIsolationDb = 120.0;

}  // namespace

RirReceiverOperatingState RirReceiverStateBuilder::Build(
    const RirRfCycleInput& input, const oneq::electromagnetics::RfSceneEmission& emission,
    const config::RirHardwareConfig& hardware, double carrier_hz) {
  const config::hardware::RirReceiverConfig& receiver = hardware.receiver;
  const config::hardware::RirTransmitterConfig& transmitter = hardware.transmitter;

  RirReceiverOperatingState operating_state;
  oneq::electromagnetics::RfSceneReceiverState& receiver_state = operating_state.rf_receiver;
  receiver_state.sensor_entity_id = input.sensor_entity_id;
  receiver_state.position_ecef_m = input.platform_position_ecef_m;
  receiver_state.velocity_ecef_mps = input.platform_velocity_ecef_mps;
  receiver_state.antenna = emission.antenna;
  receiver_state.polarization = receiver.scene_polarization;
  receiver_state.window_start_time_s = input.window_start_time_s;
  receiver_state.window_duration_s = input.window_duration_s;
  receiver_state.center_frequency_hz = carrier_hz;
  // 预选带宽 / 远场下限 / 同平台隔离：库内默认，不暴露为公开配置。
  receiver_state.bandwidth_hz = kDefaultPreselectorBandwidthHz;
  receiver_state.receiver_system_loss_db = static_cast<double>(receiver.receive_loss_db);
  receiver_state.minimum_far_field_range_m = kDefaultMinimumFarFieldRangeM;
  receiver_state.co_site_paths = {oneq::electromagnetics::RfCoSiteIsolationPath(
      input.sensor_entity_id, kDefaultCoSiteIsolationDb)};
  operating_state.beam_pointing_deg = input.beam_pointing_deg;
  operating_state.matched_filter_bandwidth_hz = static_cast<double>(transmitter.bandwidth_hz);
  operating_state.receiver_noise_figure_db = static_cast<double>(receiver.noise_figure_db);
  operating_state.maximum_linear_input_power_w =
      static_cast<double>(receiver.maximum_linear_input_power_w);
  return operating_state;
}

}  // namespace dwell
}  // namespace remote_identification_radar
