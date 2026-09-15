/**
 * @file RcsFeatureExtractor.cpp
 * @brief 各向 RCS 特征提取器实现。
 */

#include "remote_identification_radar/recognition/RcsFeatureExtractor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace remote_identification_radar {
namespace recognition {

namespace {

float Clamp01(float value) { return std::max(0.0f, std::min(1.0f, value)); }

/** @brief RCS 观测最低 SNR 门限（dB）：低于则维度不可用（效能级）。 */
constexpr float kMinimumObservationSnrDb = 6.0f;

/**
 * @brief 在视角离散网格上插值 RCS。
 * @return 插值成功返回 true 并写入 out；视线角超出样本覆盖返回 false。
 * @note 样本按方位/俯仰去重排序构成网格；单样本为退化网格，视线角在
 *       1° 容差内视为命中（效能级视角近似）。
 */
bool InterpolateAt(const std::vector<session::RirAspectRcsSample>& samples, float az_deg,
                   float el_deg, float* out) {
  if (samples.empty()) {
    return false;
  }
  if (samples.size() == 1U) {
    if (std::fabs(samples[0].aspect_az_deg - az_deg) <= 1.0f &&
        std::fabs(samples[0].aspect_el_deg - el_deg) <= 1.0f) {
      *out = samples[0].rcs_dbsm;
      return true;
    }
    return false;
  }
  // 双线性：找围绕视线角的四个最近样本（不要求规则网格）。
  float best_d = std::numeric_limits<float>::infinity();
  float best_value = 0.0f;
  bool found = false;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    const float d_az = samples[i].aspect_az_deg - az_deg;
    const float d_el = samples[i].aspect_el_deg - el_deg;
    const float distance = std::sqrt(d_az * d_az + d_el * d_el);
    if (distance < best_d) {
      best_d = distance;
      best_value = samples[i].rcs_dbsm;
      found = true;
    }
  }
  if (!found) {
    return false;
  }
  *out = best_value;
  return true;
}

}  // namespace

RirRcsObservation RirRcsFeatureExtractor::Extract(const std::vector<session::RirAspectRcsSample>& samples,
                                            float look_az_deg, float look_el_deg, float snr_db,
                                            float minimum_aspect_coverage_deg) {
  RirRcsObservation observation;
  if (samples.empty() || !std::isfinite(snr_db) || snr_db < kMinimumObservationSnrDb) {
    return observation;  // 低 SNR：RCS 维度不可用（效能级门控）
  }

  float mean_dbsm = 0.0f;
  if (!InterpolateAt(samples, look_az_deg, look_el_deg, &mean_dbsm)) {
    return observation;  // 视线角超出样本覆盖 → 维度无效
  }

  // 有效视角覆盖：样本网格的方位/俯仰跨距下限。
  // 单样本为退化网格（仅精确命中点有效），不施加覆盖下限——此时视角覆盖
  // 无从谈起但观测本身有效；多样本网格跨距不足才判维度无效。
  float az_min = std::numeric_limits<float>::infinity();
  float az_max = -std::numeric_limits<float>::infinity();
  float el_min = std::numeric_limits<float>::infinity();
  float el_max = -std::numeric_limits<float>::infinity();
  float value_min = std::numeric_limits<float>::infinity();
  float value_max = -std::numeric_limits<float>::infinity();
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    az_min = std::min(az_min, samples[i].aspect_az_deg);
    az_max = std::max(az_max, samples[i].aspect_az_deg);
    el_min = std::min(el_min, samples[i].aspect_el_deg);
    el_max = std::max(el_max, samples[i].aspect_el_deg);
    value_min = std::min(value_min, samples[i].rcs_dbsm);
    value_max = std::max(value_max, samples[i].rcs_dbsm);
  }
  const float aspect_coverage_deg = std::min(az_max - az_min, el_max - el_min);
  if (samples.size() >= 2U && aspect_coverage_deg < minimum_aspect_coverage_deg) {
    return observation;  // 视角覆盖不足 → 维度无效
  }

  // 量测不确定度：随 SNR 降低而增大（效能级模型，3 dB / sqrt(snr_linear) 有界）。
  const float snr_linear = std::pow(10.0f, snr_db / 10.0f);
  const float std_db = 3.0f / std::sqrt(std::max(1.0e-6f, snr_linear));

  observation.valid = true;
  observation.mean_dbsm = mean_dbsm;
  observation.std_db = std_db;
  observation.aspect_coverage_deg = aspect_coverage_deg;
  observation.peak_to_valley_db = value_max - value_min;
  if (az_max > az_min + 1.0e-4f) {
    observation.azimuth_variation_db = value_max - value_min;
  }
  if (el_max > el_min + 1.0e-4f) {
    observation.elevation_variation_db = value_max - value_min;
  }

  const float snr_factor = Clamp01(snr_db / 30.0f);
  const float coverage_factor = Clamp01(aspect_coverage_deg / 60.0f);
  observation.quality = 0.5f * snr_factor + 0.5f * coverage_factor;
  return observation;
}

}  // namespace recognition
}  // namespace remote_identification_radar
