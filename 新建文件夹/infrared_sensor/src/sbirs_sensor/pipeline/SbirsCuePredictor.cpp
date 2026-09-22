/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsCuePredictor.
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

#include "sbirs_sensor/pipeline/SbirsCuePredictor.h"

#include <algorithm>
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

float ClampElevation(float elevation_deg) {
  return std::max(-90.0f, std::min(90.0f, elevation_deg));
}

}  // namespace

SbirsCuePrediction SbirsCuePredictor::Update(std::uint64_t target_id, float measured_azimuth_deg,
                                             float measured_elevation_deg, float dt_sec,
                                             float cue_latency_s) {
  SbirsCuePrediction result;
  result.command_azimuth_deg = NormalizeAzimuth(measured_azimuth_deg);
  result.command_elevation_deg = ClampElevation(measured_elevation_deg);

  const auto previous = targets_.find(target_id);
  const bool valid_timing = std::isfinite(dt_sec) && dt_sec > 0.0f &&
                            std::isfinite(cue_latency_s) && cue_latency_s > 0.0f;
  const bool valid_measurement =
      std::isfinite(measured_azimuth_deg) && std::isfinite(measured_elevation_deg);
  if (previous != targets_.end() && valid_timing && valid_measurement) {
    result.angular_rate_azimuth_deg_per_sec =
        NormalizeAzimuth(measured_azimuth_deg - previous->second.measured_azimuth_deg) / dt_sec;
    result.angular_rate_elevation_deg_per_sec =
        (measured_elevation_deg - previous->second.measured_elevation_deg) / dt_sec;
    result.command_azimuth_deg = NormalizeAzimuth(
        measured_azimuth_deg + result.angular_rate_azimuth_deg_per_sec * cue_latency_s);
    result.command_elevation_deg = ClampElevation(
        measured_elevation_deg + result.angular_rate_elevation_deg_per_sec * cue_latency_s);
    result.used_motion_prediction = true;
  }

  if (valid_measurement) {
    SbirsCuePredictorTargetState state;
    state.measured_azimuth_deg = NormalizeAzimuth(measured_azimuth_deg);
    state.measured_elevation_deg = ClampElevation(measured_elevation_deg);
    targets_[target_id] = state;
  }
  return result;
}

void SbirsCuePredictor::Release(std::uint64_t target_id) { targets_.erase(target_id); }

void SbirsCuePredictor::Clear() { targets_.clear(); }

SbirsCuePredictorSnapshot SbirsCuePredictor::Capture() const {
  SbirsCuePredictorSnapshot snapshot;
  snapshot.targets = targets_;
  return snapshot;
}

void SbirsCuePredictor::Restore(const SbirsCuePredictorSnapshot& snapshot) {
  targets_ = snapshot.targets;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
