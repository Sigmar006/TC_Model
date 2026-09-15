#include "sbirs_sensor/pipeline/SbirsNfovAcquisition.h"

#include <cmath>

namespace sbirs_sensor {
namespace pipeline {
namespace {

float NormalizeAzimuth(float azimuth_deg) {
  float result = std::fmod(azimuth_deg + 180.0f, 360.0f);
  if (result < 0.0f) {
    result += 360.0f;
  }
  return result - 180.0f;
}

}  // namespace

bool IsNfovAcquisitionEligible(const SbirsNfovAcquisitionRequest& request) {
  const float cue_azimuth_deg = request.command_azimuth_deg + request.pointing_settle_error_deg;
  const bool in_nfov =
      std::fabs(NormalizeAzimuth(request.delayed_truth_azimuth_deg - cue_azimuth_deg)) <=
          0.5f * request.field_of_view_azimuth_deg &&
      std::fabs(request.delayed_truth_elevation_deg - request.command_elevation_deg) <=
          0.5f * request.field_of_view_elevation_deg;
  return in_nfov && request.snr >= request.minimum_snr_linear;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
