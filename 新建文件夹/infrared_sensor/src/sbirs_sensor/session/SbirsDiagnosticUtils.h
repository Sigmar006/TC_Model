/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsDiagnosticUtils.
 * Model: SBIRS infrared sensor.
 * Layer: session.
 * File role: interface declarations and data contracts.
 * Responsibility: public cycle contract, validation, adapters, recording, and replay.
 * Upstream boundary: external cycle data and validated session configuration.
 * Downstream boundary: stable output frames, issues, debug views, and replay records.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

/**
 * @file SbirsDiagnosticUtils.h
 * @brief SBIRS 单周期诊断条目构造与中止记录工具函数。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_SESSION_SBIRS_DIAGNOSTIC_UTILS_H_
#define ONEQ_SRC_SBIRS_SENSOR_SESSION_SBIRS_DIAGNOSTIC_UTILS_H_

#include <string>

#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief 将 SbirsPipelineAbortReason 粗粒度枚举转换为诊断码字符串。
 */
const char* AbortReasonToDiagnosticCode(SbirsPipelineAbortReason reason);

/**
 * @brief 将本周期标记为中止（三写：abort_reason + issues + 日志）。
 * @param[out] result 单周期结果。
 * @param[in] reason 粗粒度中止原因；phase 由原因推导（kValidationRejected → kInputValidation，
 *            其余 → kExecution）。
 * @param[in] detail_code 细粒度 issue code（SbirsIssueCodes.h 注册表常量，完整字符串），写入 issues。
 * @param[in] message 中止描述。
 */
void RecordAbort(SbirsCycleResult* result, SbirsPipelineAbortReason reason,
                 const char* detail_code, const std::string& message);

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_SESSION_SBIRS_DIAGNOSTIC_UTILS_H_
