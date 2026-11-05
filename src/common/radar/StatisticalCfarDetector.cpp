#include "common/radar/StatisticalCfarDetector.h"

/**
 * ## 过程：统计 CFAR 检测判决（雷达探测链 · 公共核心）
 * - 上游：`RirSignalDetector`（回波功率 + 处理后单脉冲 SINR + 起伏模型 + 脉冲数）
 * - 本步：
 *   ① 由 SINR 代入 Marcum Q 模型算检测概率 Pd（门限 T = Q⁻¹(N, Pfa)）；
 *   ② 若 SINR < `min_snr_db` 直接把 Pd 置 0（低于最小可检门限，不参与抽签）；
 *   ③ 以 Pd 与均匀随机数比较得"本拍是否检出"（统计检测，非阈值硬判）；
 *   ④ 检出后再过 `min_detection_margin_db` 余量门，不足则判不检（抑制边缘虚警）。
 * - 下游：`RirDetectionResult{detected, snr_db, detection_prob}` → 量测构造 → 跟踪
 * - 关键参数：`cfar_pfa`（1e-6）、`min_snr_db`（-10 dB）、`min_detection_margin_db`（-2 dB）
 * - 对应验收项：第 36 项（统计检测门限/结果/概率）
 *
 * @note 第 36 项写的 `统计检测门限=` 即本文件的"Pd=50% 所需单脉冲 SNR"，
 *       与同项 `信噪比=` 同轴，可直接比较（2026-11-04 口径）。
 */

#include <algorithm>
#include <cmath>

namespace oneq {
namespace common {
namespace radar {

StatisticalCfarDetector::StatisticalCfarDetector(float thermal_noise_w,
                                                 StatisticalCfarPolicy policy)
    : thermal_noise_w_(thermal_noise_w), policy_(policy), rng_(42u) {}

void StatisticalCfarDetector::Update(float thermal_noise_w, StatisticalCfarPolicy policy) {
  thermal_noise_w_ = thermal_noise_w;
  policy_ = policy;
}

void StatisticalCfarDetector::SetRandomSeed(unsigned int seed) { rng_.seed(seed); }

StatisticalCfarResult StatisticalCfarDetector::DetectFromEchoBudget(
    float echo_power_dbw_after_receive_loss, float clutter_noise_w, float jam_noise_w,
    SwerlingModel swerling, int pulse_count) {
  StatisticalCfarResult result;
  result.echo_power_dbw = echo_power_dbw_after_receive_loss;

  const float echo_power_w = std::pow(10.0f, result.echo_power_dbw / 10.0f);
  const float kNoiseFloorW = 1e-30f;
  const float safe_thermal_noise_w = std::max(thermal_noise_w_, 0.0f);
  const float safe_clutter_noise_w = std::max(clutter_noise_w, 0.0f);
  const float safe_jam_noise_w = std::max(jam_noise_w, 0.0f);
  const float total_noise_w =
      std::max(safe_thermal_noise_w + safe_clutter_noise_w + safe_jam_noise_w, kNoiseFloorW);

  const float base_snr_linear = echo_power_w / total_noise_w;
  result.snr_db = 10.0f * std::log10(base_snr_linear + kNoiseFloorW);

  const int effective_pulse_count = std::max(1, pulse_count);
  result.detection_prob = RadarEquations::ComputeDetectionProbability(
      result.snr_db, policy_.cfar_pfa, swerling, effective_pulse_count);

  if (result.snr_db < policy_.min_snr_db) {
    result.detected = false;
    result.detection_prob = 0.0f;
  } else {
    result.detected = RadarEquations::ThresholdDecision(result.detection_prob, rng_);
  }

  if (result.detected && result.snr_db < policy_.min_detection_margin_db) {
    result.detected = false;
  }
  return result;
}

StatisticalCfarResult StatisticalCfarDetector::DetectResolvedCell(
    float echo_power_w, float processed_single_pulse_sinr_db, SwerlingModel swerling,
    int pulse_count) {
  StatisticalCfarResult result;
  if (!std::isfinite(echo_power_w) || echo_power_w < 0.0f ||
      !std::isfinite(processed_single_pulse_sinr_db) || pulse_count <= 0) {
    return result;
  }
  constexpr double kLinearFloor = 1.0e-300;
  result.echo_power_dbw =
      static_cast<float>(10.0 * std::log10(std::max(static_cast<double>(echo_power_w), kLinearFloor)));
  result.snr_db = processed_single_pulse_sinr_db;
  result.detection_prob = RadarEquations::ComputeDetectionProbability(
      result.snr_db, policy_.cfar_pfa, swerling, pulse_count);
  if (result.snr_db < policy_.min_snr_db) {
    result.detection_prob = 0.0f;
    return result;
  }
  result.detected = RadarEquations::ThresholdDecision(result.detection_prob, rng_);
  if (result.detected && result.snr_db < policy_.min_detection_margin_db) {
    result.detected = false;
  }
  return result;
}

}  // namespace radar
}  // namespace common
}  // namespace oneq
