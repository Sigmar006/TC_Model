/**
 * @file PrecisionAcceptanceRecords.cpp
 * @brief 精度评估验收行：关键精度指标派生 + AHP 等级/贡献排序。
 */

#include "precision_evaluation/PrecisionAcceptanceRecords.h"

#include <string>
#include <utility>

#include "common/logging/AcceptanceText.h"
#include "precision_evaluation/PrecisionEvaluationLog.h"
#include "target_inference/InferenceAcceptanceRecords.h"

namespace precision_evaluation {
namespace {

using oneq::logging::FormatF;
using oneq::logging::FormatPairDeg;
using oneq::logging::FormatVec3;

const char* GradeOf(double score) {
  if (score >= 0.85) {
    return "A";
  }
  if (score >= 0.70) {
    return "B";
  }
  if (score >= 0.55) {
    return "C";
  }
  return "D";
}

}  // namespace

void WritePrecisionKeyMetricsUnconditional(const std::map<std::uint64_t, TargetKeyErrorSnapshot>& latest) {
  // 验收判定标准 第26项（修改）：逐目标一行，只写误差本身（估计−真值/输出角−真值角/
  // 焦平面脱靶量），不写 RMSE/CEP50/CI 等派生统计；无样本的字段省略。
  for (const auto& entry : latest) {
    const TargetKeyErrorSnapshot& snapshot = entry.second;
    std::string content = "目标ID=" + std::to_string(entry.first);
    if (snapshot.has_ecef) {
      content += " 三轴位置误差ECEF=" +
                 FormatVec3(snapshot.ecef_error_m[0], snapshot.ecef_error_m[1],
                            snapshot.ecef_error_m[2], 1) +
                 "m";
    }
    if (snapshot.has_slant_range) {
      content += " 距离误差=" + FormatF(snapshot.slant_range_error_m, 1) + "m";
    }
    if (snapshot.has_angular) {
      content += " 方位/俯仰误差=" + FormatPairDeg(snapshot.az_error_deg, snapshot.el_error_deg, 4) +
                 "°";
    }
    if (snapshot.has_focal) {
      content += " 脱靶量m=(" + FormatF(snapshot.focal_x_m, 6) + "," +
                 FormatF(snapshot.focal_y_m, 6) + ")";
    }
    // 会话汇总行语义：周期/时间用 0（与文档示例一致；行内误差为最近一拍快照）。
    // 直接写通道：`PRECISION_EVAL_ITEM` 在编译期开关关闭时会展开成空语句，
  // 现场因此出现"会话跑通、样本有、日志 0 行"（2026-10-21）。
  target_inference::WriteSatelliteTargetAcceptanceLine("关键精度指标提取功能测试", content);
  }
}

void WritePrecisionAhpUnconditional(const PrecisionEvaluationReport& report) {
  if (!report.ahp_valid) {
    return;
  }
  std::string weights = "(";
  std::string scores = "(";
  for (std::size_t i = 0; i < kPrecisionMetricCount; ++i) {
    if (i != 0U) {
      weights += ",";
      scores += ",";
    }
    weights += FormatF(report.ahp.weights[i], 3);
    scores += FormatF(report.metric_scores[i], 3);
  }
  weights += ")";
  scores += ")";

  // 验收判定标准 第27项：权重/底层得分/综合分/等级；贡献排序与 CR 为派生项不写。
  std::string content = "准则层权重=" + weights;
  content += " 底层分=" + scores;
  content += " 综合分=" + FormatF(report.composite_score, 3);
  content += std::string(" 等级=") + GradeOf(report.composite_score);
  target_inference::WriteSatelliteTargetAcceptanceLine("层次分析法功能测试", content);
}

void WritePrecisionKeyMetrics(const std::map<std::uint64_t, TargetKeyErrorSnapshot>& latest) {
  if (!PRECISION_EVAL_LOG_ENABLED()) {
    return;
  }
  WritePrecisionKeyMetricsUnconditional(latest);
}

void WritePrecisionAhp(const PrecisionEvaluationReport& report) {
  if (!PRECISION_EVAL_LOG_ENABLED()) {
    return;
  }
  WritePrecisionAhpUnconditional(report);
}

}  // namespace precision_evaluation
