#include "remote_identification_radar/dwell/RirRfFrontEndResolver.h"

#include <algorithm>
#include <cmath>

namespace remote_identification_radar {
namespace dwell {
namespace {

bool IdentityLess(const oneq::electromagnetics::RfIncidentLinkResult& left,
                  const oneq::electromagnetics::RfIncidentLinkResult& right) {
  if (left.identity.sensor_entity_id != right.identity.sensor_entity_id) {
    return left.identity.sensor_entity_id < right.identity.sensor_entity_id;
  }
  return left.identity.emission_id < right.identity.emission_id;
}

}  // namespace

bool TryResolveRirRfFrontEnd(const oneq::electromagnetics::RfSceneFrame& scene,
                             const oneq::electromagnetics::RfSceneReceiverState& receiver,
                             double maximum_linear_input_power_w,
                             const oneq::electromagnetics::RfIncidentLinkConfig& link_config,
                             RirRfFrontEndResult* result) {
  if (result == nullptr || !std::isfinite(maximum_linear_input_power_w) ||
      maximum_linear_input_power_w <= 0.0 ||
      !oneq::electromagnetics::TryValidateRfSceneFrame(scene) ||
      scene.window_start_time_s != receiver.window_start_time_s ||
      scene.window_duration_s != receiver.window_duration_s) {
    return false;
  }

  RirRfFrontEndResult candidate;
  candidate.incident_links.reserve(scene.emissions.size());
  for (const auto& emission : scene.emissions) {
    oneq::electromagnetics::RfIncidentLinkResult link;
    if (!oneq::electromagnetics::TryEvaluateRfIncidentLink(emission, receiver, link_config,
                                                           &link)) {
      return false;
    }
    candidate.incident_links.push_back(link);
  }
  std::sort(candidate.incident_links.begin(), candidate.incident_links.end(), IdentityLess);
  if (!oneq::electromagnetics::TryAggregateRfIncidentPower(candidate.incident_links,
                                                           &candidate.total_incident_power_w)) {
    return false;
  }
  candidate.receiver_saturated = candidate.total_incident_power_w > maximum_linear_input_power_w;
  *result = candidate;
  return true;
}

}  // namespace dwell
}  // namespace remote_identification_radar
