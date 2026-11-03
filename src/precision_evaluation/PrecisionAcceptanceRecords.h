/**
 * @file PrecisionAcceptanceRecords.h
 * @brief 精度评估验收行拼装。
 */

#ifndef ONEQ_SRC_PRECISION_EVALUATION_PRECISION_ACCEPTANCE_RECORDS_H_
#define ONEQ_SRC_PRECISION_EVALUATION_PRECISION_ACCEPTANCE_RECORDS_H_

#include <cstdint>
#include <array>
#include <map>
#include <vector>

#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"

namespace precision_evaluation {

/** @brief 单目标「最新一拍」误差快照（验收判定标准 第26项：误差本身，非 RMSE 统计）。 */
struct TargetKeyErrorSnapshot {
  std::uint32_t last_cycle{0U};  /**< 该目标最近一次误差样本周期。 */
  bool has_ecef{false};          /**< 三轴位置误差 ECEF 可用（双星交会解−真值）。 */
  std::array<double, 3> ecef_error_m = std::array<double, 3>(); /**< 三轴位置误差 ECEF（m）。 */
  bool has_slant_range{false};   /**< 距离误差可用（交会解斜距−真值斜距）。 */
  double slant_range_error_m{0.0}; /**< 距离误差（m）。 */
  bool has_angular{false};       /**< 方位/俯仰误差可用（输出角−真值角）。 */
  double az_error_deg{0.0};      /**< 方位误差（°）。 */
  double el_error_deg{0.0};      /**< 俯仰误差（°）。 */
  bool has_focal{false};         /**< 脱靶量可用（NFOV 焦平面，归属记录透出）。 */
  double focal_x_m{0.0};         /**< 脱靶量 x（m）。 */
  double focal_y_m{0.0};         /**< 脱靶量 y（m）。 */
};

// 验收判定标准 第26项：逐目标一行（目标ID + 三轴位置误差ECEF/距离误差/方位俯仰误差/
// 脱靶量，无则省略）；不写 RMSE/CEP50/CI 等派生统计。
void WritePrecisionKeyMetrics(const std::map<std::uint64_t, TargetKeyErrorSnapshot>& latest);

void WritePrecisionAhp(const PrecisionEvaluationReport& report);

// 无编译期闸门的写出接口（2026-10-21）：现场宿主用"总是写"的路径，避免写盘宏在
// 不同编译单元里求值不一致；是否调用由会话/宿主决定。
void WritePrecisionKeyMetricsUnconditional(
    const std::map<std::uint64_t, TargetKeyErrorSnapshot>& latest);

void WritePrecisionAhpUnconditional(const PrecisionEvaluationReport& report);

/**
 * @brief 只格式化不写盘：第26 项验收行（逐目标一行，无样本目标不产出）。
 *
 * @note 2026-10-22 现场定案：宿主进程里由评估线程调用 `WriteAcceptanceLog` 写出的行
 *       始终不可见（写调用确实执行、sink 指向现场日志、流 healthy、tellp 增长；同目录
 *       另开文件正常、同一 DLL 的推演行正常）。故把"算/格式化"与"写"拆开：库只出字符串，
 *       由集成方在**自己确认可写盘的线程**上落盘（见 PrecisionEvaluationSession::
 *       LastAcceptanceLines 与宿主 `sbirsfusiondp_1q.cpp` 的写出分支）。
 */
std::vector<std::string> BuildPrecisionKeyMetricsLines(
    const std::map<std::uint64_t, TargetKeyErrorSnapshot>& latest);

/**
 * @brief 只格式化不写盘：第27 项验收行（AHP 非法时返回空串）。
 * @note 拆分理由同 BuildPrecisionKeyMetricsLines。
 */
std::string BuildPrecisionAhpLine(const PrecisionEvaluationReport& report);

}  // namespace precision_evaluation

#endif
