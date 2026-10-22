/**
 * @file MtiMtdAcceptanceBank.cpp
 * @brief 验收旁路 2 脉冲 MTI + 8 路 DFT MTD（频谱效能，不进 SINR）。
 */

#include "common/radar/MtiMtdAcceptanceBank.h"

#include <algorithm>
#include <cmath>
#include "common/numerics/Constants.h"

namespace oneq {
namespace common {
namespace radar {
namespace {

using oneq::common::numerics::kLightSpeed;
using oneq::common::numerics::kPi;
constexpr double kLinearFloor = 1.0e-300;
constexpr std::size_t kClutterGridCount = 512U;

bool IsFiniteNonNegative(double value) { return std::isfinite(value) && value >= 0.0; }

bool IsFinitePositive(double value) { return std::isfinite(value) && value > 0.0; }

double FoldIntoPrf(double frequency_hz, double prf_hz) {
  const double half = 0.5 * prf_hz;
  double folded = std::fmod(frequency_hz + half, prf_hz);
  if (folded < 0.0) {
    folded += prf_hz;
  }
  return folded - half;
}

double MtiPowerResponse(double frequency_hz, double prf_hz) {
  const double folded = FoldIntoPrf(frequency_hz, prf_hz);
  const double sine = std::sin(kPi * folded / prf_hz);
  return 4.0 * sine * sine;
}

double DftBinPower(double frequency_hz, std::size_t channel, double prf_hz) {
  const double center_hz =
      static_cast<double>(channel) * prf_hz / static_cast<double>(kMtiMtdChannelCount);
  const double delta_hz = FoldIntoPrf(frequency_hz - center_hz, prf_hz);
  const double theta = 2.0 * kPi * delta_hz / prf_hz;
  if (std::abs(theta) < 1.0e-14) {
    return 1.0;
  }
  const double numerator = std::sin(0.5 * static_cast<double>(kMtiMtdChannelCount) * theta);
  const double denominator = std::sin(0.5 * theta);
  if (std::abs(denominator) < 1.0e-18) {
    return 1.0;
  }
  const double magnitude = numerator / (static_cast<double>(kMtiMtdChannelCount) * denominator);
  return magnitude * magnitude;
}

double ToDecibels(double linear) {
  return 10.0 * std::log10(std::max(linear, kLinearFloor));
}

bool AreTonesUsable(const MtiMtdAcceptanceInput& input) {
  if (input.tone_count == 0U) {
    return true;
  }
  if (input.tones == nullptr) {
    return false;
  }
  for (std::size_t i = 0U; i < input.tone_count; ++i) {
    if (!std::isfinite(input.tones[i].doppler_hz) || !IsFiniteNonNegative(input.tones[i].power_w)) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool TryResolveMtiMtdAcceptanceBank(const MtiMtdAcceptanceInput& input,
                                    MtiMtdAcceptanceResult* result) {
  if (result == nullptr || !IsFiniteNonNegative(input.echo_power_w) ||
      !IsFiniteNonNegative(input.thermal_noise_power_w) ||
      !IsFiniteNonNegative(input.clutter_power_w) ||
      !std::isfinite(input.two_way_doppler_shift_hz) || !IsFinitePositive(input.prf_hz) ||
      !IsFinitePositive(input.center_frequency_hz) || !AreTonesUsable(input)) {
    return false;
  }

  MtiMtdAcceptanceResult candidate;
  const double fd_hz = FoldIntoPrf(input.two_way_doppler_shift_hz, input.prf_hz);
  const double mti_target = MtiPowerResponse(fd_hz, input.prf_hz);
  candidate.mti_gain_db = ToDecibels(mti_target);

  double best_target = -1.0;
  for (std::size_t k = 0U; k < kMtiMtdChannelCount; ++k) {
    const double bin = DftBinPower(fd_hz, k, input.prf_hz);
    candidate.target_w[k] = input.echo_power_w * mti_target * bin;
    candidate.noise_w[k] =
        input.thermal_noise_power_w * kMtiNoiseFactor / static_cast<double>(kMtiMtdChannelCount);
    if (candidate.target_w[k] > best_target) {
      best_target = candidate.target_w[k];
      candidate.selected_channel = k;
    }
  }
  // 中译：**MTD 增益口径**（2026-10-22）——真实 MTD 增益 = 相干积累 10·log10(N) +
  //   跨 bin 损耗（≤0）。此前只输出归一化 bin 响应（恒 ≤0），会被误读成"MTD 永远是负增益"；
  //   N=1（不做相干积累）时退化为纯损耗，与旧行为逐位一致。
  // 标识：MTD 真实增益（2026-10-22）。
  const double bin_response = DftBinPower(fd_hz, candidate.selected_channel, input.prf_hz);
  candidate.mtd_bin_loss_db = ToDecibels(bin_response);
  const std::uint32_t coherent_pulses =
      input.coherent_pulse_count >= 2U ? input.coherent_pulse_count : 1U;
  candidate.mtd_coherent_gain_db =
      coherent_pulses >= 2U ? 10.0 * std::log10(static_cast<double>(coherent_pulses)) : 0.0;
  candidate.mtd_gain_db = candidate.mtd_coherent_gain_db + candidate.mtd_bin_loss_db;
  candidate.mtd_equivalent_noise_w = candidate.noise_w[candidate.selected_channel];

  const double wavelength_m = kLightSpeed / input.center_frequency_hz;
  const double sigma_hz = 2.0 * kMtiMtdClutterSigmaVelocityMps / wavelength_m;
  const double df = input.prf_hz / static_cast<double>(kClutterGridCount);
  const double grid_start = -0.5 * input.prf_hz;
  double spectrum_mass = 0.0;
  double mti_weighted_mass = 0.0;
  std::array<double, kMtiMtdChannelCount> clutter_mass{};
  const double inv_sigma = (sigma_hz > 1.0e-12) ? (1.0 / sigma_hz) : 0.0;
  const double gauss_scale =
      (sigma_hz > 1.0e-12) ? (inv_sigma / std::sqrt(2.0 * kPi)) : 0.0;
  for (std::size_t i = 0U; i < kClutterGridCount; ++i) {
    const double f_hz = grid_start + (static_cast<double>(i) + 0.5) * df;
    double density = 0.0;
    if (sigma_hz > 1.0e-12) {
      const double x = f_hz * inv_sigma;
      density = gauss_scale * std::exp(-0.5 * x * x);
    } else if (std::abs(f_hz) < 0.5 * df) {
      density = 1.0 / df;
    }
    const double mass = density * df;
    spectrum_mass += mass;
    const double mti = MtiPowerResponse(f_hz, input.prf_hz);
    mti_weighted_mass += mti * mass;
    for (std::size_t k = 0U; k < kMtiMtdChannelCount; ++k) {
      clutter_mass[k] += mti * DftBinPower(f_hz, k, input.prf_hz) * mass;
    }
  }
  const double inv_mass = (spectrum_mass > kLinearFloor) ? (1.0 / spectrum_mass) : 0.0;
  candidate.mti_residual_clutter_w = input.clutter_power_w * mti_weighted_mass * inv_mass;
  for (std::size_t k = 0U; k < kMtiMtdChannelCount; ++k) {
    candidate.clutter_w[k] = input.clutter_power_w * clutter_mass[k] * inv_mass;
  }

  candidate.has_jam_channels = input.tone_count > 0U;
  if (candidate.has_jam_channels) {
    for (std::size_t t = 0U; t < input.tone_count; ++t) {
      const double fj = FoldIntoPrf(input.tones[t].doppler_hz, input.prf_hz);
      const double mti_j = MtiPowerResponse(fj, input.prf_hz);
      candidate.mti_residual_jam_w += input.tones[t].power_w * mti_j;
      for (std::size_t k = 0U; k < kMtiMtdChannelCount; ++k) {
        candidate.jam_w[k] += input.tones[t].power_w * mti_j * DftBinPower(fj, k, input.prf_hz);
      }
    }
  }

  *result = candidate;
  return true;
}

bool TryAcceptancePowerRatioDb(double input_w, double residual_w, double* ratio_db) {
  if (ratio_db == nullptr || !IsFinitePositive(input_w) || !IsFinitePositive(residual_w)) {
    return false;
  }
  const double ratio = 10.0 * std::log10(input_w / residual_w);
  if (!std::isfinite(ratio)) {
    return false;
  }
  *ratio_db = ratio;
  return true;
}

}  // namespace radar
}  // namespace common
}  // namespace oneq
