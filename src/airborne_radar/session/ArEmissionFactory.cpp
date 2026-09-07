#include "airborne_radar/session/ArEmissionFactory.h"

#include <algorithm>
#include <cmath>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "common/numerics/Constants.h"

namespace airborne_radar {
namespace session {
namespace {

// 把 mount-frame 波束指向解析到 ECEF 方向单位向量。失败当姿态或方位/俯仰角越界。
bool TryResolveEcefBoresight(const ArPrepareCycleInput& input,
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

bool ArEmissionFactory::TryBuildEmission(
    const ArPrepareCycleInput& input, const config::engineering::DetectionConfig& detection,
    const ArControlProfile& control_profile, std::uint64_t emission_id, double carrier_hz,
    double pulse_repetition_interval_s, std::uint32_t pulse_count, std::uint64_t timing_seed,
    std::uint64_t successful_prepare_count, oneq::electromagnetics::RfSceneEmission* emission) {
  if (emission == nullptr) {
    return false;
  }
  const config::engineering::TransmitterConfig& transmitter = detection.transmitter;
  const config::engineering::ReceiverConfig& receiver = detection.receiver;

  emission->identity.sensor_entity_id = input.sensor_entity_id;
  emission->identity.emission_id = emission_id;
  emission->position_ecef_m = input.platform_position_ecef_m;
  emission->velocity_ecef_mps = input.platform_velocity_ecef_mps;
  if (!TryResolveEcefBoresight(input, &emission->antenna.boresight_ecef)) {
    return false;
  }
  emission->antenna.peak_gain_dbi = static_cast<double>(detection.antenna.main_beam_gain_db);
  emission->antenna.half_power_beamwidth_deg = static_cast<double>(std::max(
      detection.antenna.nominal_az_beamwidth_deg, detection.antenna.nominal_el_beamwidth_deg));
  emission->antenna.sidelobe_level_db =
      static_cast<double>(detection.antenna.pattern.max_sidelobe_level_db);
  emission->antenna.backlobe_level_db =
      static_cast<double>(detection.antenna.pattern.backlobe_level_db);
  emission->antenna.cross_polarization_isolation_db =
      static_cast<double>(receiver.cross_polarization_isolation_db);
  emission->polarization = receiver.scene_polarization;
  const double emission_control_scale =
      control_profile.enable_lpi_power_control
          ? std::max(0.0, static_cast<double>(control_profile.lpi_power_scale))
          : 1.0;
  const double requested_peak_power_w =
      static_cast<double>(transmitter.peak_power_w) * emission_control_scale *
      std::max(1.0, static_cast<double>(control_profile.eccm_burnthrough_gain));
  const double energy_limited_peak_power_w =
      static_cast<double>(transmitter.maximum_pulse_energy_j) /
      static_cast<double>(transmitter.pulse_width_s);
  const double actual_peak_power_w =
      std::min({requested_peak_power_w, static_cast<double>(transmitter.maximum_peak_power_w),
                energy_limited_peak_power_w});
  const double radiated_peak_power_w =
      actual_peak_power_w *
      std::pow(10.0, -static_cast<double>(transmitter.transmit_loss_db) / 10.0);
  return oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      input.window_start_time_s, carrier_hz, static_cast<double>(transmitter.bandwidth_hz),
      radiated_peak_power_w, static_cast<double>(transmitter.pulse_width_s),
      pulse_repetition_interval_s, pulse_count,
      control_profile.enable_eccm_rejitter ? 0.15 : 0.0, timing_seed, successful_prepare_count,
      &emission->waveform);
}

}  // namespace session
}  // namespace airborne_radar
