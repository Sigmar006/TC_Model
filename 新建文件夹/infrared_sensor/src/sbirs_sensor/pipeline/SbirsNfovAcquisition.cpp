/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsNfovAcquisition.
 * Model: SBIRS infrared sensor.
 * Layer: pipeline.
 * File role: executable implementation.
 * Responsibility: wide-field search, pointing, acquisition, and tracking processing.
 * Upstream boundary: cycle scene, cue, sensor state, and resolved physical parameters.
 * Downstream boundary: detections, estimates, diagnostics, and lifecycle observations.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

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
