/**
 * @file RangeProfileFeatureExtractor.cpp
 * @brief 宽带一维距离像特征提取器实现。
 */

#include "remote_identification_radar/recognition/RangeProfileFeatureExtractor.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>
#include "common/numerics/Constants.h"

namespace remote_identification_radar {
namespace recognition {

namespace {

using oneq::common::numerics::kLightSpeed;
constexpr float kSpeedOfLightFloat = static_cast<float>(kLightSpeed);
/** @brief 能量集中率前 K 峰计数。 */
constexpr std::uint32_t kConcentrationPeakCount = 3U;
/** @brief 最小峰间距（m）：低于该间距的相邻散射中心合并计数。 */
constexpr float kMinimumPeakSeparationM = 0.5f;

float Clamp01(float value) { return std::max(0.0f, std::min(1.0f, value)); }

/** @brief dBsm 转线性散射功率。 */
float ToLinearPower(float rcs_dbsm) { return std::pow(10.0f, rcs_dbsm / 10.0f); }

}  // namespace

RirRangeProfileObservation RirRangeProfileFeatureExtractor::Extract(
    const std::vector<session::RirRangeRcsScatterer>& scatterers, float bandwidth_hz, float snr_db,
    float max_range_resolution_m) {
  RirRangeProfileObservation observation;
  if (scatterers.empty() || !std::isfinite(bandwidth_hz) || bandwidth_hz <= 0.0f ||
      !std::isfinite(snr_db)) {
    return observation;
  }
  const float resolution_m = kSpeedOfLightFloat / (2.0f * bandwidth_hz);
  if (max_range_resolution_m > 0.0f && resolution_m > max_range_resolution_m) {
    return observation;  // 分辨率不满足数据库要求 → 维度无效
  }
  observation.resolution_m = resolution_m;

  // 噪声门限：相对最强散射中心由 SNR 决定。
  float peak_linear_power = 0.0f;
  for (std::size_t i = 0U; i < scatterers.size(); ++i) {
    peak_linear_power = std::max(peak_linear_power, ToLinearPower(scatterers[i].rcs_dbsm));
  }
  const float snr_linear = std::pow(10.0f, snr_db / 10.0f);
  const float noise_gate = peak_linear_power / std::max(1.0e-6f, snr_linear);

  // 峰值判定（散射中心级，满足最小间距）与能量统计。
  std::vector<float> peak_offsets_m;
  std::vector<float> peak_powers;
  peak_offsets_m.reserve(scatterers.size());
  peak_powers.reserve(scatterers.size());
  float total_peak_power = 0.0f;
  for (std::size_t i = 0U; i < scatterers.size(); ++i) {
    const float power = ToLinearPower(scatterers[i].rcs_dbsm);
    if (power < noise_gate) {
      continue;
    }
    bool separated = true;
    for (std::size_t p = 0U; p < peak_offsets_m.size(); ++p) {
      if (std::fabs(peak_offsets_m[p] - scatterers[i].range_offset_m) <
          kMinimumPeakSeparationM) {
        separated = false;
        break;
      }
    }
    if (!separated) {
      continue;
    }
    peak_offsets_m.push_back(scatterers[i].range_offset_m);
    peak_powers.push_back(power);
    total_peak_power += power;
  }
  if (peak_offsets_m.empty() || total_peak_power <= 0.0f) {
    return observation;
  }

  // 目标长度：首末有效峰跨距。
  float min_offset = std::numeric_limits<float>::infinity();
  float max_offset = -std::numeric_limits<float>::infinity();
  for (std::size_t i = 0U; i < peak_offsets_m.size(); ++i) {
    min_offset = std::min(min_offset, peak_offsets_m[i]);
    max_offset = std::max(max_offset, peak_offsets_m[i]);
  }
  observation.length_m = max_offset - min_offset;
  observation.peak_count = static_cast<std::uint32_t>(peak_offsets_m.size());

  // 能量集中率：前 K 峰能量与总有效能量之比。
  std::vector<float> sorted_powers = peak_powers;
  std::sort(sorted_powers.begin(), sorted_powers.end(), std::greater<float>());
  float top_k_power = 0.0f;
  const std::uint32_t top_k = std::min(kConcentrationPeakCount,
                                       static_cast<std::uint32_t>(sorted_powers.size()));
  for (std::uint32_t i = 0U; i < top_k; ++i) {
    top_k_power += sorted_powers[i];
  }
  observation.peak_energy_concentration = top_k_power / total_peak_power;

  observation.valid = true;
  const float snr_factor = Clamp01(snr_db / 30.0f);
  const float resolution_factor = Clamp01(50.0f / std::max(1.0e-3f, resolution_m));
  observation.quality = 0.5f * snr_factor + 0.5f * resolution_factor;
  return observation;
}

}  // namespace recognition
}  // namespace remote_identification_radar
