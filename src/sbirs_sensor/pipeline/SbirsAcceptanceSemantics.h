/**
 * @file SbirsAcceptanceSemantics.h
 * @brief 红外验收行的语义判定小工具：把"视场外/钳位/未建模"与物理量区分开。
 *
 * 背景（2026-10-21 红外日志体检）：现场把三种不同的事实都写成了数值 0，读日志的人无法
 * 区分"探测距离真的是 0 m"和"目标不在本星宽场视场内"：
 *   - 最大探测距离 66% 写 0 m（= 视场外或辐射链路不可解）；
 *   - 6.4% 恰为 mission.max_range_m（= 被上限钳位）；
 *   - 读出噪声 100% 写 0.000J（= 配置未建模）。
 * 本头把这些判定抽成纯函数，既供 pipeline 拼行，也供单测守住语义。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_SEMANTICS_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_SEMANTICS_H_

#include <cmath>
#include <string>

#include "common/platform/OneqPlatformCompat.h"

namespace sbirs_sensor {
namespace pipeline {

/** @brief 最大探测距离的三种事实来源（验收第 6 项）。 */
enum class SbirsDetectionRangeKind {
  kApplicable = 0,  /**< 有物理意义的值：辐射反解与视场内几何都成立。 */
  kOutOfFieldOfView = 1, /**< 本星宽场视场内无地球交点（或辐射链路不可解）→ 不适用。 */
  kClampedToMissionMax = 2 /**< 反解值超过 mission.max_range_m → 被上限钳位。 */
};

/**
 * @brief 判定"相对卫星最大探测距离"应写成数值还是"不适用"。
 * @param[in] radiometric_max_m 辐射链路反解距离（m；<=0 表示链路不可解）。
 * @param[in] fov_max_m 视场内最大可视距离（m；<=0 表示目标不在宽场视场内）。
 * @param[in] mission_max_m 任务距离上限（m；<=0 表示无上限）。
 * @return 语义分类。
 */
inline SbirsDetectionRangeKind ClassifyDetectionRange(double radiometric_max_m, double fov_max_m,
                                                      double mission_max_m) {
  if (!(radiometric_max_m > 0.0) || !(fov_max_m > 0.0)) {
    return SbirsDetectionRangeKind::kOutOfFieldOfView;
  }
  if (mission_max_m > 0.0 && fov_max_m >= mission_max_m) {
    return SbirsDetectionRangeKind::kClampedToMissionMax;
  }
  return SbirsDetectionRangeKind::kApplicable;
}

/**
 * @brief 第 11/68 项"检测标志"：由两道门派生，不得写死。
 * @param[in] snr_gate_passed 宽场 SNR 门（snr >= wide_min_snr_linear）。
 * @param[in] in_fov 目标是否落在本星宽场视场内。
 * @return "是"/"否"。
 */
inline const char* DetectionFlagText(bool snr_gate_passed, bool in_fov) {
  return (snr_gate_passed && in_fov) ? "是" : "否";
}

/**
 * @brief 检测门限的日志文本：低于 k=1 的门限不是物理检测门限，必须显式标注。
 *
 * `*_min_snr_linear` 的语义是"归一化 SNR 倍数 k"（设计值 k≈4），而现场配置写成了 0.001，
 * 使 Pfa=0.5·erfc(k/√2)≈0.4996、探测概率恒 ≈0.5（第 11 项因此失去判据意义）。
 * @param[in] k 门限值（无量纲倍数）。
 * @return 形如 "4.000000" 或 "0.001000(低于k=1,不合物理)"。
 */
inline std::string DetectionThresholdText(float k) {
  char buffer[64];
  const double value = static_cast<double>(k);
  ONEQ_SNPRINTF(buffer, sizeof(buffer), "%.6f", value);
  std::string text(buffer);
  if (!(value >= 1.0)) {
    text += "(低于k=1,不合物理)";
  }
  return text;
}

/**
 * @brief 噪声分量是否建模：未建模时日志必须写"未建模"，不得写 0.000。
 * @param[in] configured_value 对应配置项（≤0 视为未配置）。
 * @return true 表示该分量已建模、可按数值写出。
 */
inline bool IsNoiseComponentModeled(float configured_value) {
  return std::isfinite(static_cast<double>(configured_value)) && configured_value > 0.0f;
}

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_SEMANTICS_H_
