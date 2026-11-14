/**
 * @file PolarizationFeatureExtractor.cpp
 * @brief 双通道极化特征提取器实现。
 */

/**
 * ## 过程：双通道极化特征（LRDR 跟踪/识别链 · 第 8 步）
 * - 上游：场景极化散射矩阵样本（HH/VV 幅度 dBsm）、视线角、检测 SNR、斜距。
 * - 本步：视线角最近邻取两通道 RCS → 线性功率 σ；以第一通道 SNR 定噪声底得 E₁/E₂；
 *   输出能量差 10lg(E₁/E₂)、相对差 10lg(|E₁−E₂|/(E₁+E₂))、能量和（按 R⁴ 补偿到 100 km
 *   参考距离），质量 = clamp(SNR/30 dB)；SNR<6 dB 或距离非法则维度不可用。
 * - 下游：`RirMatcher` 极化维；验收第 46 项的双通道比值类子项（散射矩阵行列式/Span/
 *   去极化系数/本征角由 `PolarizationStatsExtractor` 给出）。
 * - 关键参数：`kReferenceRangeM`=100 km（库 `polarization_energy_reference` 语义）、
 *   SNR 门限 6 dB、相对差下限 1e-6、质量归一化 30 dB。
 * - 对应验收项：第 46 项 极化特征解算功能测试。
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
// 合并版第 9 条：观测 SNR 门改为配置项 RirRecognitionPolicy::min_feature_observation_snr_db（原硬编码 6.0）。

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

// 过程：输入门——无样本、SNR<6 dB 或距离非法时极化维判不可用。
RirPolarizationObservation RirPolarizationFeatureExtractor::Extract(
    const std::vector<session::RirPolSMatrixSample>& samples, float look_az_deg,
    float look_el_deg, float snr_db, float range_m, float minimum_observation_snr_db) {
  RirPolarizationObservation observation;
  if (samples.empty() || !std::isfinite(snr_db) || snr_db < minimum_observation_snr_db ||
      !std::isfinite(range_m) || range_m <= 0.0f) {
    return observation;  // 低 SNR 或无效距离：极化维度不可用
  }
  float rcs_1_dbsm = 0.0f;
  float rcs_2_dbsm = 0.0f;
  if (!InterpolateChannels(samples, look_az_deg, look_el_deg, &rcs_1_dbsm, &rcs_2_dbsm)) {
    return observation;
  }

  // 同一雷达方程：E_linear ∝ σ / R⁴（通道增益差异由数据库定义，此处同构）。
  // 过程：σ = 10^(dBsm/10)（线性散射功率），两通道同构换算。
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
  // 过程：E_ref = σ·(R/100 km)⁴——4 次方距离补偿后的强度（与 R⁻⁴ 回波衰减相消）。
  const float range_compensation = std::pow(range_m / kReferenceRangeM, 4.0f);
  const float e1_ref = sigma_1 * range_compensation;
  const float e2_ref = sigma_2 * range_compensation;
  observation.energy_sum_db = 10.0f * std::log10(e1_ref + e2_ref);

  observation.valid = true;
  // 过程：质量 = clamp(SNR/30 dB)——极化比类特征对信噪比敏感，30 dB 视为满质量。
  observation.quality = Clamp01(snr_db / 30.0f);
  return observation;
}

}  // namespace recognition
}  // namespace remote_identification_radar
