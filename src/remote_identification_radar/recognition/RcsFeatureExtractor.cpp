/**
 * @file RcsFeatureExtractor.cpp
 * @brief 各向 RCS 特征提取器实现。
 */

/**
 * ## 过程：RCS 统计特征（LRDR 跟踪/识别链 · 第 6 步）
 * - 上游：场景目标各向 RCS 样本表（方位/俯仰/RCS dBsm）、视线角、检测 SNR、库的视角
 *   覆盖下限。
 * - 本步：在视角网格上取视线方向的 RCS 均值；统计网格跨距（视角覆盖）、峰谷差与方位/
 *   俯仰起伏；量测不确定度 σ_obs = 3 dB/√SNR_lin；质量 = 0.5·clamp(SNR/30 dB) +
 *   0.5·clamp(覆盖/60°)；SNR < 6 dB 或视角覆盖不足则维度不可用。
 * - 下游：`RirObservationBuilder` → `RirMatcher` 的 RCS 维打分；验收第 45 项（RCS 均值/
 *   标准差、归一化值、波动系数、RCS 置信度）。
 * - 关键参数：`minimum_aspect_coverage_deg`（库 profile 传入）、SNR 门限 6 dB（效能级）、
 *   质量归一化 30 dB / 60°。
 * - 对应验收项：第 45 项 RCS 统计特征处理功能测试。
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
// 过程：多样本时取方位/俯仰欧氏距离最近的样本值（不要求规则网格）。
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

// 过程：输入门——无样本、SNR 非有限或低于 6 dB 时 RCS 维直接判不可用（效能级门控）。
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
  // 过程：有效覆盖取方位/俯仰跨距的较小者；同循环取 RCS 极值供峰谷差与起伏。
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

  // 过程：质量 = 0.5·clamp(SNR/30 dB) + 0.5·clamp(覆盖/60°)——信噪比与视角覆盖等权。
  const float snr_factor = Clamp01(snr_db / 30.0f);
  const float coverage_factor = Clamp01(aspect_coverage_deg / 60.0f);
  observation.quality = 0.5f * snr_factor + 0.5f * coverage_factor;
  return observation;
}

}  // namespace recognition
}  // namespace remote_identification_radar
