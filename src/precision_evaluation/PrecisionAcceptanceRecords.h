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

}  // namespace precision_evaluation

#endif
