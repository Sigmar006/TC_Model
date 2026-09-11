#include "1q/precision_evaluation/PrecisionEvaluationMetrics.h"

#include <algorithm>
#include <cmath>

namespace precision_evaluation {

ErrorMetricSummary SummarizeErrorSeries(const std::vector<double>& errors) {
  ErrorMetricSummary summary;
  summary.count = errors.size();
  if (errors.empty()) {
    return summary;
  }
  double sum = 0.0;
  double squared_sum = 0.0;
  for (const double error : errors) {
    sum += error;
    squared_sum += error * error;
  }
  summary.mean = sum / static_cast<double>(errors.size());
  summary.rmse = std::sqrt(squared_sum / static_cast<double>(errors.size()));
  // P95 最近秩法：升序第 ceil(0.95·n) 个样本（1 基）。
  std::vector<double> sorted_errors = errors;
  std::sort(sorted_errors.begin(), sorted_errors.end());
  const std::size_t p95_index =
      std::min(sorted_errors.size() - 1U,
               static_cast<std::size_t>(
                   std::ceil(0.95 * static_cast<double>(sorted_errors.size()))) - 1U);
  summary.p95 = sorted_errors[p95_index];
  summary.max = sorted_errors.back();
  return summary;
}

double NormalizeErrorScore(double rmse, double reference_error) {
  // 中译：参考误差非有限时必须显式记 0——NaN 与任何数比较都为假，
  //       仅靠 reference_error <= 0.0 拦不住它，会一路除成 NaN 污染综合分。
  // 标识：precision_evaluation 规则 9（不得静默退化）
  if (reference_error <= 0.0 || !std::isfinite(reference_error) || rmse < 0.0 ||
      !std::isfinite(rmse)) {
    return 0.0;
  }
  return reference_error / (reference_error + rmse);
}

}  // namespace precision_evaluation
