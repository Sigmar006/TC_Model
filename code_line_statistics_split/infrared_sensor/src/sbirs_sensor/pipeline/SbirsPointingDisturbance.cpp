#include "sbirs_sensor/pipeline/SbirsPointingDisturbance.h"

#include <cmath>
#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

using oneq::common::numerics::kPi;

bool IsFinite(const SbirsGaussMarkovSnapshot& snapshot) {
  return std::isfinite(snapshot.azimuth_deg) && std::isfinite(snapshot.elevation_deg) &&
         snapshot.random_state != 0U;
}

}  // namespace

std::uint32_t SbirsPointingDisturbance::DeriveSeed(std::uint32_t base_seed,
                                                   std::uint32_t stream_id) {
  std::uint32_t value =
      (base_seed == 0U ? 1U : base_seed) + UINT32_C(0x9e3779b9) * (stream_id + 1U);
  value ^= value >> 16;
  value *= 0x7feb352dU;
  value ^= value >> 15;
  value *= 0x846ca68bU;
  value ^= value >> 16;
  return value == 0U ? 1U : value;
}

double SbirsPointingDisturbance::DerivePhaseRad(std::uint32_t base_seed,
                                                 std::uint32_t stream_id) {
  return 2.0 * kPi * static_cast<double>(DeriveSeed(base_seed, stream_id)) / 4294967296.0;
}

SbirsPointingDisturbance::SbirsPointingDisturbance(int channel_count, std::uint32_t seed)
    : base_seed_(seed == 0U ? 1U : seed), common_(DeriveSeed(base_seed_, 0U)) {
  const int normalized_count = channel_count < 1 ? 1 : channel_count;
  channels_.reserve(static_cast<std::size_t>(normalized_count));
  for (int channel_id = 0; channel_id < normalized_count; ++channel_id) {
    channels_.push_back(ChannelRuntime(
        DeriveSeed(base_seed_, 1U + static_cast<std::uint32_t>(channel_id))));
  }
}

bool SbirsPointingDisturbance::ValidateParameters(
    const SbirsPointingDisturbanceParameters& parameters) {
  return std::isfinite(parameters.common_attitude_sigma_deg) &&
         parameters.common_attitude_sigma_deg >= 0.0 &&
         std::isfinite(parameters.common_attitude_correlation_time_s) &&
         parameters.common_attitude_correlation_time_s > 0.0 &&
         std::isfinite(parameters.channel_pointing_sigma_deg) &&
         parameters.channel_pointing_sigma_deg >= 0.0 &&
         std::isfinite(parameters.channel_pointing_correlation_time_s) &&
         parameters.channel_pointing_correlation_time_s > 0.0 &&
         std::isfinite(parameters.channel_vibration_amplitude_deg) &&
         parameters.channel_vibration_amplitude_deg >= 0.0 &&
         std::isfinite(parameters.channel_vibration_frequency_hz) &&
         parameters.channel_vibration_frequency_hz >= 0.0 &&
         (parameters.channel_vibration_amplitude_deg == 0.0 ||
          parameters.channel_vibration_frequency_hz > 0.0);
}

void SbirsPointingDisturbance::AdvanceGaussMarkov(double dt_sec, double sigma_deg,
                                                   double correlation_time_s,
                                                   GaussMarkovRuntime* runtime) {
  if (sigma_deg == 0.0) {
    runtime->azimuth_deg = 0.0;
    runtime->elevation_deg = 0.0;
    return;
  }
  const double alpha = std::exp(-dt_sec / correlation_time_s);
  const double innovation_scale = sigma_deg * std::sqrt(1.0 - alpha * alpha);
  runtime->azimuth_deg =
      alpha * runtime->azimuth_deg + innovation_scale * runtime->random.NextStandardNormal();
  runtime->elevation_deg =
      alpha * runtime->elevation_deg + innovation_scale * runtime->random.NextStandardNormal();
}

bool SbirsPointingDisturbance::Advance(
    double dt_sec, const SbirsPointingDisturbanceParameters& parameters) {
  if (!std::isfinite(dt_sec) || dt_sec <= 0.0 || !ValidateParameters(parameters)) {
    return false;
  }
  GaussMarkovRuntime next_common = common_;
  std::vector<ChannelRuntime> next_channels = channels_;
  AdvanceGaussMarkov(dt_sec, parameters.common_attitude_sigma_deg,
                     parameters.common_attitude_correlation_time_s, &next_common);
  for (ChannelRuntime& channel : next_channels) {
    AdvanceGaussMarkov(dt_sec, parameters.channel_pointing_sigma_deg,
                       parameters.channel_pointing_correlation_time_s, &channel.gauss_markov);
    channel.elapsed_time_s += dt_sec;
  }
  common_ = next_common;
  channels_ = next_channels;
  return true;
}

bool SbirsPointingDisturbance::Sample(
    int channel_id, const SbirsPointingDisturbanceParameters& parameters,
    SbirsPointingDisturbanceSample* sample) const {
  if (sample == nullptr || channel_id < 0 ||
      static_cast<std::size_t>(channel_id) >= channels_.size() || !ValidateParameters(parameters)) {
    return false;
  }
  const ChannelRuntime& channel = channels_[static_cast<std::size_t>(channel_id)];
  const double angular_frequency = 2.0 * kPi * parameters.channel_vibration_frequency_hz;
  const std::uint32_t stream_base = UINT32_C(0x100) +
                                    2U * static_cast<std::uint32_t>(channel_id);
  SbirsPointingDisturbanceSample next;
  next.common.azimuth_deg = common_.azimuth_deg;
  next.common.elevation_deg = common_.elevation_deg;
  next.channel.azimuth_deg = channel.gauss_markov.azimuth_deg +
                             parameters.channel_vibration_amplitude_deg *
                                 std::sin(angular_frequency * channel.elapsed_time_s +
                                          DerivePhaseRad(base_seed_, stream_base));
  next.channel.elevation_deg = channel.gauss_markov.elevation_deg +
                               parameters.channel_vibration_amplitude_deg *
                                   std::sin(angular_frequency * channel.elapsed_time_s +
                                            DerivePhaseRad(base_seed_, stream_base + 1U));
  *sample = next;
  return true;
}

SbirsPointingDisturbanceSnapshot SbirsPointingDisturbance::Capture() const {
  SbirsPointingDisturbanceSnapshot snapshot;
  snapshot.base_seed = base_seed_;
  snapshot.common.azimuth_deg = common_.azimuth_deg;
  snapshot.common.elevation_deg = common_.elevation_deg;
  snapshot.common.random_state = common_.random.Capture();
  snapshot.channels.reserve(channels_.size());
  for (const ChannelRuntime& channel : channels_) {
    SbirsChannelDisturbanceSnapshot channel_snapshot;
    channel_snapshot.gauss_markov.azimuth_deg = channel.gauss_markov.azimuth_deg;
    channel_snapshot.gauss_markov.elevation_deg = channel.gauss_markov.elevation_deg;
    channel_snapshot.gauss_markov.random_state = channel.gauss_markov.random.Capture();
    channel_snapshot.elapsed_time_s = channel.elapsed_time_s;
    snapshot.channels.push_back(channel_snapshot);
  }
  return snapshot;
}

bool SbirsPointingDisturbance::Restore(const SbirsPointingDisturbanceSnapshot& snapshot) {
  if (snapshot.base_seed == 0U || snapshot.channels.size() != channels_.size() ||
      !IsFinite(snapshot.common)) {
    return false;
  }
  GaussMarkovRuntime next_common(DeriveSeed(snapshot.base_seed, 0U));
  next_common.azimuth_deg = snapshot.common.azimuth_deg;
  next_common.elevation_deg = snapshot.common.elevation_deg;
  next_common.random.Restore(snapshot.common.random_state);
  std::vector<ChannelRuntime> next_channels;
  next_channels.reserve(snapshot.channels.size());
  for (std::size_t i = 0; i < snapshot.channels.size(); ++i) {
    const SbirsChannelDisturbanceSnapshot& source = snapshot.channels[i];
    if (!IsFinite(source.gauss_markov) || !std::isfinite(source.elapsed_time_s) ||
        source.elapsed_time_s < 0.0) {
      return false;
    }
    ChannelRuntime destination(
        DeriveSeed(snapshot.base_seed, 1U + static_cast<std::uint32_t>(i)));
    destination.gauss_markov.azimuth_deg = source.gauss_markov.azimuth_deg;
    destination.gauss_markov.elevation_deg = source.gauss_markov.elevation_deg;
    destination.gauss_markov.random.Restore(source.gauss_markov.random_state);
    destination.elapsed_time_s = source.elapsed_time_s;
    next_channels.push_back(destination);
  }
  base_seed_ = snapshot.base_seed;
  common_ = next_common;
  channels_ = next_channels;
  return true;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
