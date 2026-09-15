/**
 * @file PolarizationFeatureExtractor.cpp
 * @brief 双通道极化特征提取器实现。
 */

#include "remote_identification_radar/recognition/PolarizationFeatureExtractor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace remote_identification_radar {
namespace recognition {

namespace {

/** @brief 统一参考距离（m）：能量和换算基准，与数据库
 *  `polarization_energy_reference` 语义对应。 */
constexpr float kReferenceRangeM = 100000.0f;
/** @brief 相对差对数奇异下限（线性）。 */
constexpr float kMinimumRelativeDifferenceFloor = 1.0e-6f;
/** @brief 极化观测最低 SNR 门限（dB）：低于则维度不可用（效能级）。 */
constexpr float kMinimumObservationSnrDb = 6.0f;

float Clamp01(float value) { return std::max(0.0f, std::min(1.0f, value)); }

/** @brief 视线角最近邻插值；返回两通道 RCS 与命中标志。 */
bool InterpolateChannels(const std::vector<session::RirPolSMatrixSample>& samples,
                         float az_deg, float el_deg, float* channel_1, float* channel_2) {
  if (samples.empty()) {
    return false;
  }
  float best_d = std::numeric_limits<float>::infinity();
  bool found = false;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    const float d_az = samples[i].aspect_az_deg - az_deg;
    const float d_el = samples[i].aspect_el_deg - el_deg;
    const float distance = std::sqrt(d_az * d_az + d_el * d_el);
    if (distance < best_d) {
      best_d = distance;
      *channel_1 = samples[i].hh_amp_db;
      *channel_2 = samples[i].vv_amp_db;
      found = true;
    }
  }
  return found;
}

}  // namespace

RirPolarizationObservation RirPolarizationFeatureExtractor::Extract(
    const std::vector<session::RirPolSMatrixSample>& samples, float look_az_deg,
    float look_el_deg, float snr_db, float range_m) {
  RirPolarizationObservation observation;
  if (samples.empty() || !std::isfinite(snr_db) || snr_db < kMinimumObservationSnrDb ||
      !std::isfinite(range_m) || range_m <= 0.0f) {
    return observation;  // 低 SNR 或无效距离：极化维度不可用
  }
  float rcs_1_dbsm = 0.0f;
  float rcs_2_dbsm = 0.0f;
  if (!InterpolateChannels(samples, look_az_deg, look_el_deg, &rcs_1_dbsm, &rcs_2_dbsm)) {
    return observation;
  }

  // 同一雷达方程：E_linear ∝ σ / R⁴（通道增益差异由数据库定义，此处同构）。
  const float sigma_1 = std::pow(10.0f, rcs_1_dbsm / 10.0f);
  const float sigma_2 = std::pow(10.0f, rcs_2_dbsm / 10.0f);
  // 噪声底：相对第一通道信号的 SNR 决定；叠加后进入各度量。
  const float snr_linear = std::pow(10.0f, snr_db / 10.0f);
  const float noise_linear = sigma_1 / std::max(1.0e-6f, snr_linear);
  const float e1 = sigma_1 + noise_linear;
  const float e2 = sigma_2 + noise_linear;

  // 能量差：通道功率比（与距离/发射参数无关）。
  observation.energy_difference_db = 10.0f * std::log10(e1 / std::max(1.0e-12f, e2));

  // 相对差：|E1-E2|/(E1+E2) 对数化，接近相等时以下限保护。
  const float relative = std::fabs(e1 - e2) / std::max(kMinimumRelativeDifferenceFloor, e1 + e2);
  observation.relative_difference_db = 10.0f * std::log10(relative);

  // 能量和：换算到统一参考距离（4 次方距离补偿），得到与体制解耦的强度特征。
  const float range_compensation = std::pow(range_m / kReferenceRangeM, 4.0f);
  const float e1_ref = sigma_1 * range_compensation;
  const float e2_ref = sigma_2 * range_compensation;
  observation.energy_sum_db = 10.0f * std::log10(e1_ref + e2_ref);

  observation.valid = true;
  observation.quality = Clamp01(snr_db / 30.0f);
  return observation;
}

}  // namespace recognition
}  // namespace remote_identification_radar
