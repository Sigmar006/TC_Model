#include "sbirs_sensor/session/SbirsDiagnosticUtils.h"

#include <utility>

#include "1q/sbirs_sensor/session/SbirsIssueCodes.h"
#include "common/logging/ProjectLog.h"

namespace sbirs_sensor {
namespace session {

const char* AbortReasonToDiagnosticCode(SbirsPipelineAbortReason reason) {
  switch (reason) {
    case SbirsPipelineAbortReason::kNone:
      return "";
    case SbirsPipelineAbortReason::kValidationRejected:
      return "validation_rejected";
    case SbirsPipelineAbortReason::kSensorPoweredOff:
      return "sensor_powered_off";
  }
  // 日志兜底，不进注册表（AbortReasonToDiagnosticCode 仅人读日志映射）。
  return "unknown";
}

void RecordAbort(SbirsCycleResult* result, SbirsPipelineAbortReason reason,
                 const char* detail_code, const std::string& message) {
  result->abort_reason = reason;
  result->status = reason == SbirsPipelineAbortReason::kValidationRejected
                       ? SbirsCycleStatus::kRejectedInvalidInput
                       : SbirsCycleStatus::kRejectedExecution;

  // 结构化诊断（细粒度；统一问题列表模型，规则 14：phase 由中止原因推导；
  // detail_code 为 SbirsIssueCodes.h 完整 code 常量，调用方负责传注册表常量）
  SbirsIssue issue;
  issue.severity = SbirsIssueSeverity::kError;
  issue.phase = reason == SbirsPipelineAbortReason::kValidationRejected
                    ? SbirsIssuePhase::kInputValidation
                    : SbirsIssuePhase::kExecution;
  issue.code = detail_code;
  issue.message = message;
  result->issues.push_back(std::move(issue));

  // 中译：SBIRS 中止记录（原因码 — 细粒度码 — 消息）。
  // 标识：三写之三（人读日志）——标识本周期中止的粗粒度原因与细粒度码，
  //       供排查中止路径；仅用于人读，不用于状态判断（规则 3）。
  PROJECT_LOG_ERROR("SBIRS {}: {} — {}", AbortReasonToDiagnosticCode(reason), detail_code, message);
}

}  // namespace session
}  // namespace sbirs_sensor
