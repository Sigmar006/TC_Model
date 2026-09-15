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
