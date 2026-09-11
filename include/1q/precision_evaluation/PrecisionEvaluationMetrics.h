/**
 * @file PrecisionEvaluationMetrics.h
 * @brief 误差序列汇总统计与 AHP 归一化评分纯函数（需求 3.2.1.6.3.1 关键精度指标提取）。
 */

#ifndef ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_METRICS_H_
#define ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_METRICS_H_

#include <vector>

#include "1q/api.hpp"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"

namespace precision_evaluation {

/**
 * @brief 汇总一条误差序列（均值/RMSE/P95/最大值）。
 * @param[in] errors 误差样本序列（非负，单位由指标决定）
 * @return 汇总统计；空序列返回全零（count=0）
 * @note P95 取升序排序后的 ceil(0.95·n)−1 下标（最近秩法），序列短时退化为最大值。
 */
ErrorMetricSummary ONEQ_API SummarizeErrorSeries(const std::vector<double>& errors);

/**
 * @brief 误差归一化评分：score = ref/(ref+rmse)。
 * @param[in] rmse 误差均方根
 * @param[in] reference_error 参考误差（rmse=ref 时得 0.5；误差越小分越接近 1）
 * @return 归一化得分 ∈ [0,1]；ref 非正或非有限（NaN/±∞）、rmse<0、或 rmse 非有限
 *         时返回 0。有效输入的取值落在 (0,1]：rmse=0 时恰为 1。
 *         评估侧以 rmse=+∞ 表示零证据指标（count=0），据此得 0 分。
 * @note 平滑单调映射，无断点配置；参考误差语义见 PrecisionEvaluationConfig。
 */
double ONEQ_API NormalizeErrorScore(double rmse, double reference_error);

}  // namespace precision_evaluation

#endif  // ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_METRICS_H_
