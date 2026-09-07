#include "electronic_surveillance_radar/pipeline/EsrRfV2FrontEnd.h"

#include <algorithm>
#include <cmath>

#include "1q/coordinate/position_transform.h"
#include "electronic_surveillance_radar/pipeline/EsrBoresightChain.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace {

bool IsFinite(double value) { return std::isfinite(value) != 0; }

bool IdentityLess(const oneq::electromagnetics::RfIncidentLinkResult& left,
                  const oneq::electromagnetics::RfIncidentLinkResult& right) {
  if (left.identity.sensor_entity_id != right.identity.sensor_entity_id) {
    return left.identity.sensor_entity_id < right.identity.sensor_entity_id;
  }
  return left.identity.emission_id < right.identity.emission_id;
}

bool TryResolveBoresight(const session::EsrCycleInput& input,
                         const config::EsrOrientationConfig& orientation,
                         double beam_az_deg, double beam_el_deg,
                         oneq::electromagnetics::RfSceneDirection* boresight) {
  // beam_az/el 为天线系指向角（扫描图案输出口径），安装偏置由链路复合，不再角度相加。
  if (boresight == nullptr || !IsFinite(beam_az_deg) || !IsFinite(beam_el_deg) ||
      beam_az_deg < -180.0 || beam_az_deg > 180.0 || beam_el_deg < -90.0 ||
      beam_el_deg > 90.0) {
    return false;
  }
  const EsrBoresightChain chain(
      input.platform_attitude_deg, static_cast<double>(orientation.antenna_mount_az_deg),
      static_cast<double>(orientation.antenna_mount_el_deg));
  const oneq::coordinate::Vector3d enu_direction =
      chain.EnuLosOfAntennaPointing(beam_az_deg, beam_el_deg);
  oneq::coordinate::LlaPositionDegM platform_lla;
  oneq::coordinate::Vector3d ecef_direction;
  if (!oneq::coordinate::TryEcefToLla(input.platform_position_ecef_m, &platform_lla) ||
      !oneq::coordinate::TryEnuToEcefDirection(enu_direction, platform_lla, &ecef_direction)) {
    return false;
  }
  boresight->x = ecef_direction.x;
  boresight->y = ecef_direction.y;
  boresight->z = ecef_direction.z;
  return true;
}

bool TryBuildReceiverState(const session::EsrCycleInput& input,
                           std::uint32_t sensor_entity_id,
                           const config::EsrHardwareConfig& hardware,
                           const config::EsrOrientationConfig& orientation,
                           double beam_az_deg, double beam_el_deg,
                           double center_frequency_hz, double bandwidth_hz,
                           oneq::electromagnetics::RfSceneReceiverState* receiver) {
  if (receiver == nullptr) {
    return false;
  }
  receiver->sensor_entity_id = sensor_entity_id;
  receiver->position_ecef_m = input.platform_position_ecef_m;
  receiver->velocity_ecef_mps = input.platform_velocity_ecef_mps;
  receiver->window_start_time_s = input.cycle_start_time_s;
  receiver->window_duration_s = static_cast<double>(input.dt_sec);
  receiver->center_frequency_hz = center_frequency_hz;
  receiver->bandwidth_hz = bandwidth_hz;
  receiver->receiver_system_loss_db = static_cast<double>(hardware.integrated_receive_loss_db);
  receiver->minimum_far_field_range_m = static_cast<double>(hardware.minimum_far_field_range_m);
  receiver->antenna.peak_gain_dbi = static_cast<double>(hardware.antenna_peak_gain_dbi);
  receiver->antenna.half_power_beamwidth_deg =
      std::max(static_cast<double>(hardware.beam_az_width_deg),
               static_cast<double>(hardware.beam_el_width_deg));
  receiver->antenna.sidelobe_level_db = static_cast<double>(hardware.antenna_sidelobe_level_db);
  receiver->antenna.backlobe_level_db = static_cast<double>(hardware.antenna_backlobe_level_db);
  receiver->antenna.cross_polarization_isolation_db =
      static_cast<double>(hardware.cross_polarization_isolation_db);
  receiver->polarization = hardware.polarization;
  receiver->co_site_paths.reserve(hardware.co_site_paths.size());
  for (const config::EsrCoSiteIsolationPath& path : hardware.co_site_paths) {
    oneq::electromagnetics::RfCoSiteIsolationPath resolved_path;
    resolved_path.transmitter_sensor_entity_id = path.transmitter_sensor_entity_id;
    resolved_path.isolation_db = path.isolation_db;
    receiver->co_site_paths.push_back(resolved_path);
  }
  return TryResolveBoresight(input, orientation, beam_az_deg, beam_el_deg,
                             &receiver->antenna.boresight_ecef);
}

}  // namespace

bool TryResolveEsrRfV2FrontEnd(const session::EsrCycleInput& input,
                               const config::EsrHardwareConfig& hardware,
                               const config::EsrOrientationConfig& orientation,
                               double beam_az_deg, double beam_el_deg,
                               double receiver_center_frequency_hz,
                               double receiver_bandwidth_hz,
                               double additional_propagation_loss_db,
                               std::uint32_t sensor_entity_id,
                               EsrRfV2FrontEndResult* result) {
  if (result == nullptr ||
      sensor_entity_id == 0U ||
      !IsFinite(receiver_center_frequency_hz) ||
      receiver_center_frequency_hz <= 0.0 || !IsFinite(receiver_bandwidth_hz) ||
      receiver_bandwidth_hz <= 0.0 || !IsFinite(additional_propagation_loss_db) ||
      additional_propagation_loss_db < 0.0 ||
      !IsFinite(hardware.receiver_band_lower_hz) ||
      !IsFinite(hardware.receiver_band_upper_hz) ||
      hardware.receiver_band_lower_hz <= 0.0 ||
      hardware.receiver_band_upper_hz <= hardware.receiver_band_lower_hz ||
      !oneq::electromagnetics::TryValidateRfSceneFrame(input.rf_emissions) ||
      !oneq::electromagnetics::RfFrameMatchesCycleWindow(
          input.rf_emissions, input.cycle_index, input.cycle_start_time_s,
          static_cast<double>(input.dt_sec))) {
    return false;
  }

  EsrRfV2FrontEndResult candidate;
  const double front_end_bandwidth_hz =
      hardware.receiver_band_upper_hz - hardware.receiver_band_lower_hz;
  const double front_end_center_frequency_hz =
      0.5 * (hardware.receiver_band_lower_hz + hardware.receiver_band_upper_hz);
  if (!TryBuildReceiverState(input, sensor_entity_id, hardware, orientation, beam_az_deg, beam_el_deg,
                             front_end_center_frequency_hz, front_end_bandwidth_hz,
                             &candidate.front_end_receiver) ||
      !TryBuildReceiverState(input, sensor_entity_id, hardware, orientation, beam_az_deg, beam_el_deg,
                             receiver_center_frequency_hz, receiver_bandwidth_hz,
                             &candidate.channel_receiver)) {
    return false;
  }

  oneq::electromagnetics::RfIncidentLinkConfig link_config;
  link_config.additional_propagation_loss_db = additional_propagation_loss_db;
  candidate.front_end_incident_links.reserve(input.rf_emissions.emissions.size());
  candidate.channel_incident_links.reserve(input.rf_emissions.emissions.size());
  for (const oneq::electromagnetics::RfSceneEmission& emission :
       input.rf_emissions.emissions) {
    oneq::electromagnetics::RfIncidentLinkResult front_end_link;
    oneq::electromagnetics::RfIncidentLinkResult channel_link;
    if (!oneq::electromagnetics::TryEvaluateRfIncidentLink(
            emission, candidate.front_end_receiver, link_config, &front_end_link) ||
        !oneq::electromagnetics::TryEvaluateRfIncidentLink(
            emission, candidate.channel_receiver, link_config, &channel_link)) {
      return false;
    }
    candidate.front_end_incident_links.push_back(front_end_link);
    candidate.channel_incident_links.push_back(channel_link);
  }
  std::sort(candidate.front_end_incident_links.begin(), candidate.front_end_incident_links.end(),
            IdentityLess);
  std::sort(candidate.channel_incident_links.begin(), candidate.channel_incident_links.end(),
            IdentityLess);
  if (!oneq::electromagnetics::TryAggregateRfIncidentPower(candidate.front_end_incident_links,
                                                           &candidate.total_incident_power_w)) {
    return false;
  }
  candidate.receiver_saturated =
      candidate.total_incident_power_w >
      static_cast<double>(hardware.maximum_linear_input_power_w);
  *result = candidate;
  return true;
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar
