/**
 * @file SbirsPointingDisturbance.h
 * @brief Internal time-correlated attitude and per-channel pointing disturbance model.
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_POINTING_DISTURBANCE_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_POINTING_DISTURBANCE_H_

#include <cstdint>
#include <vector>

#include "sbirs_sensor/foundation/SbirsErrorModel.h"

namespace sbirs_sensor {
namespace pipeline {

struct SbirsPointingDisturbanceParameters {
  double common_attitude_sigma_deg{0.0};
  double common_attitude_correlation_time_s{1.0};
  double channel_pointing_sigma_deg{0.0};
  double channel_pointing_correlation_time_s{1.0};
  double channel_vibration_amplitude_deg{0.0};
  double channel_vibration_frequency_hz{0.0};
};

struct SbirsAngularDisturbance {
  double azimuth_deg{0.0};
  double elevation_deg{0.0};
};

struct SbirsPointingDisturbanceSample {
  SbirsAngularDisturbance common{};
  SbirsAngularDisturbance channel{};
};

struct SbirsGaussMarkovSnapshot {
  double azimuth_deg{0.0};
  double elevation_deg{0.0};
  std::uint32_t random_state{1U};
};

struct SbirsChannelDisturbanceSnapshot {
  SbirsGaussMarkovSnapshot gauss_markov{};
  double elapsed_time_s{0.0};
};

struct SbirsPointingDisturbanceSnapshot {
  std::uint32_t base_seed{1U};
  SbirsGaussMarkovSnapshot common{};
  std::vector<SbirsChannelDisturbanceSnapshot> channels{};
};

class SbirsPointingDisturbance {
 public:
  SbirsPointingDisturbance(int channel_count, std::uint32_t seed);

  bool Advance(double dt_sec, const SbirsPointingDisturbanceParameters& parameters);
  bool Sample(int channel_id, const SbirsPointingDisturbanceParameters& parameters,
              SbirsPointingDisturbanceSample* sample) const;
  SbirsPointingDisturbanceSnapshot Capture() const;
  bool Restore(const SbirsPointingDisturbanceSnapshot& snapshot);

 private:
  struct GaussMarkovRuntime {
    double azimuth_deg{0.0};
    double elevation_deg{0.0};
    foundation::SbirsRandomSource random;

    explicit GaussMarkovRuntime(std::uint32_t seed) : random(seed) {}
  };

  struct ChannelRuntime {
    GaussMarkovRuntime gauss_markov;
    double elapsed_time_s{0.0};

    explicit ChannelRuntime(std::uint32_t seed) : gauss_markov(seed) {}
  };

  static std::uint32_t DeriveSeed(std::uint32_t base_seed, std::uint32_t stream_id);
  static double DerivePhaseRad(std::uint32_t base_seed, std::uint32_t stream_id);
  static bool ValidateParameters(const SbirsPointingDisturbanceParameters& parameters);
  static void AdvanceGaussMarkov(double dt_sec, double sigma_deg, double correlation_time_s,
                                 GaussMarkovRuntime* runtime);

  std::uint32_t base_seed_{1U};
  GaussMarkovRuntime common_;
  std::vector<ChannelRuntime> channels_{};
};

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_POINTING_DISTURBANCE_H_
