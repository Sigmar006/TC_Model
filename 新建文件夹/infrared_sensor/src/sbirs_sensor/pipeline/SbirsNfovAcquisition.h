/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsNfovAcquisition.
 * Model: SBIRS infrared sensor.
 * Layer: pipeline.
 * File role: interface declarations and data contracts.
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

/**
 * @file SbirsNfovAcquisition.h
 * @brief Internal NFOV first-acquisition eligibility evaluation.
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_NFOV_ACQUISITION_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_NFOV_ACQUISITION_H_

namespace sbirs_sensor {
namespace pipeline {

/** @brief Inputs to the pure NFOV first-acquisition decision. */
struct SbirsNfovAcquisitionRequest {
  float delayed_truth_azimuth_deg{0.0f};
  float delayed_truth_elevation_deg{0.0f};
  float command_azimuth_deg{0.0f};
  float command_elevation_deg{0.0f};
  float pointing_settle_error_deg{0.0f};
  float field_of_view_azimuth_deg{0.0f};
  float field_of_view_elevation_deg{0.0f};
  double snr{0.0};
  float minimum_snr_linear{0.0f};
};

/**
 * @brief Returns whether the delayed truth direction is inside the cued NFOV and meets SNR.
 * @note This does not schedule channels, mutate target state, or initialize tracking.
 */
bool IsNfovAcquisitionEligible(const SbirsNfovAcquisitionRequest& request);

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_NFOV_ACQUISITION_H_
