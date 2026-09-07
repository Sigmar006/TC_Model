#include "remote_identification_radar/dwell/RirEmissionFactory.h"

#include <algorithm>
#include <cmath>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "common/numerics/Constants.h"

namespace remote_identification_radar {
namespace dwell {
namespace {

bool TryResolveEcefBoresight(const RirRfCycleInput& input,
                             oneq::electromagnetics::RfSceneDirection* boresight_ecef) {
  if (boresight_ecef == nullptr || !oneq::coordinate::IsFinite(input.radar_frame_attitude_deg) ||
      !std::isfinite(input.beam_pointing_deg.az_deg) ||
      !std::isfinite(input.beam_pointing_deg.el_deg) || input.beam_pointing_deg.az_deg < -180.0f ||
      input.beam_pointing_deg.az_deg > 180.0f || input.beam_pointing_deg.el_deg < -90.0f ||
      input.beam_pointing_deg.el_deg > 90.0f) {
    return false;
  }
  using oneq::common::numerics::kPi;
  const double azimuth_rad = static_cast<double>(input.beam_pointing_deg.az_deg) * kPi / 180.0;
  const double elevation_rad = static_cast<double>(input.beam_pointing_deg.el_deg) * kPi / 180.0;
  const double cos_elevation = std::cos(elevation_rad);
  const oneq::coordinate::Vector3d local_direction{cos_elevation * std::cos(azimuth_rad),
                                                   cos_elevation * std::sin(azimuth_rad),
                                                   std::sin(elevation_rad)};
  const oneq::coordinate::Vector3d enu_direction = oneq::coordinate::RotateLocalToEnu(
      local_direction.x, local_direction.y, local_direction.z, input.radar_frame_attitude_deg);
  oneq::coordinate::LlaPositionDegM platform_lla;
  oneq::coordinate::Vector3d resolved_ecef;
  if (!oneq::coordinate::TryEcefToLla(input.platform_position_ecef_m, &platform_lla) ||
      !oneq::coordinate::TryEnuToEcefDirection(enu_direction, platform_lla, &resolved_ecef)) {
    return false;
  }
  boresight_ecef->x = resolved_ecef.x;
  boresight_ecef->y = resolved_ecef.y;
  boresight_ecef->z = resolved_ecef.z;
  return true;
}

}  // namespace

double RirEmissionFactory::ResolveCarrierHz(
    const config::hardware::RirTransmitterConfig& transmitter, std::uint32_t /*cycle_index*/) {
  return static_cast<double>(transmitter.frequency_hz);
}

bool RirEmissionFactory::TryBuildEmission(
    const RirRfCycleInput& input, const config::RirHardwareConfig& hardware,
    std::uint64_t emission_id, double carrier_hz, double pulse_repetition_interval_s,
    std::uint32_t pulse_count, std::uint64_t timing_seed, std::uint64_t successful_cycle_count,
    oneq::electromagnetics::RfSceneEmission* emission) {
  if (emission == nullptr) {
    return false;
  }
  const config::hardware::RirTransmitterConfig& transmitter = hardware.transmitter;
  const config::hardware::RirReceiverConfig& receiver = hardware.receiver;
  const config::hardware::RirAntennaConfig& antenna = hardware.antenna;

  emission->identity.sensor_entity_id = input.sensor_entity_id;
  emission->identity.emission_id = emission_id;
  emission->position_ecef_m = input.platform_position_ecef_m;
  emission->velocity_ecef_mps = input.platform_velocity_ecef_mps;
  if (!TryResolveEcefBoresight(input, &emission->antenna.boresight_ecef)) {
    return false;
  }
  emission->antenna.peak_gain_dbi = static_cast<double>(antenna.main_beam_gain_db);
  emission->antenna.half_power_beamwidth_deg = static_cast<double>(
      std::max(antenna.nominal_az_beamwidth_deg, antenna.nominal_el_beamwidth_deg));
  emission->antenna.sidelobe_level_db = static_cast<double>(antenna.max_sidelobe_level_db);
  emission->antenna.backlobe_level_db = static_cast<double>(antenna.backlobe_level_db);
  emission->antenna.cross_polarization_isolation_db =
      static_cast<double>(receiver.cross_polarization_isolation_db);
  emission->polarization = receiver.scene_polarization;

  const double radiated_peak_power_w =
      static_cast<double>(transmitter.peak_power_w) *
      std::pow(10.0, -static_cast<double>(transmitter.transmit_loss_db) / 10.0);
  return oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      input.window_start_time_s, carrier_hz, static_cast<double>(transmitter.bandwidth_hz),
      radiated_peak_power_w, static_cast<double>(transmitter.pulse_width_s),
      pulse_repetition_interval_s, pulse_count, 0.0, timing_seed, successful_cycle_count,
      &emission->waveform);
}

}  // namespace dwell
}  // namespace remote_identification_radar
