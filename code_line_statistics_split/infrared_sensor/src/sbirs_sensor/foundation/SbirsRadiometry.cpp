#include "sbirs_sensor/foundation/SbirsRadiometry.h"

#include <algorithm>
#include <cmath>
#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace foundation {
namespace {

using oneq::common::numerics::kPi;

}  // namespace

double ComputeReceivedPowerW(double radiant_intensity_w_per_sr, double range_m,
                             double aperture_m, double optical_transmission,
                             double path_transmittance, double detector_quantum_efficiency) {
  if (radiant_intensity_w_per_sr < 0.0 || range_m <= 0.0 || aperture_m <= 0.0) {
    return 0.0;
  }
  const double aperture_area = kPi * aperture_m * aperture_m * 0.25;
  const double signal = radiant_intensity_w_per_sr * aperture_area;
  return signal * std::max(0.0, optical_transmission) * std::max(0.0, path_transmittance) *
         std::max(0.0, detector_quantum_efficiency) / (range_m * range_m);
}

double ComputeInfraredSnrLinear(double received_power_w,
                                const config::SbirsHardwareConfig& hardware) {
  const double signal_energy =
      std::max(0.0, received_power_w) * std::max(0.0f, hardware.integration_time_sec);
  const double noise = std::max(1.0e-18f, hardware.noise_equivalent_power_w);
  return signal_energy / noise;
}

double ComputeMaxDetectionRangeM(double radiant_intensity_w_per_sr, double aperture_m,
                                 double optical_transmission, double path_transmittance,
                                 double detector_quantum_efficiency,
                                 double integration_time_sec, double effective_noise_w,
                                 double min_snr_linear) {
  if (radiant_intensity_w_per_sr <= 0.0 || aperture_m <= 0.0 || optical_transmission <= 0.0 ||
      path_transmittance <= 0.0 || detector_quantum_efficiency <= 0.0 ||
      integration_time_sec <= 0.0 || effective_noise_w <= 0.0 || min_snr_linear <= 0.0) {
    return 0.0;
  }
  const double aperture_area = kPi * aperture_m * aperture_m * 0.25;
  const double numerator =
      radiant_intensity_w_per_sr * aperture_area * optical_transmission * path_transmittance *
      detector_quantum_efficiency * integration_time_sec;
  return std::sqrt(numerator / (effective_noise_w * min_snr_linear));
}

}  // namespace foundation
}  // namespace sbirs_sensor
