#include "sbirs_sensor/runtime/SbirsController.h"

#include "1q/sbirs_sensor/session/SbirsInputValidation.h"
#include "1q/sbirs_sensor/session/SbirsIssueCodes.h"
#include "common/logging/ProjectLog.h"
#include "sbirs_sensor/session/SbirsDiagnosticUtils.h"

namespace sbirs_sensor {
namespace runtime {

SbirsController::SbirsController(const config::SbirsInternalExecutionConfig& config)
    : pipeline_(config), frame_rate_hz_(config.session.mission.frame_rate_hz) {}

void SbirsController::ApplyConfig(const config::SbirsInternalExecutionConfig& config,
                                  const SbirsRuntimeConfigImpact& impact) {
  pipeline_.ApplyConfig(config, impact);
}

session::SbirsCycleResult SbirsController::RunOnce(const session::SbirsCycleInput& input) {
  session::SbirsCycleResult result;
  result.input_cycle_index = input.cycle_index;
  // 统一问题列表（规则 14）：输入校验问题（phase=kInputValidation）在前，执行诊断在后。
  result.issues = session::ValidateSbirsCycleInput(input, frame_rate_hz_);
  if (session::HasValidationError(result.issues)) {
    // 校验拒绝（规则 9/14）：校验问题本身就是 error 级诊断（写二），直接进入统一
    // 问题列表；abort_reason（写一）与日志（写三）在此补齐，不调用 RecordAbort
    // （避免附加与细粒度主诊断重复的粗粒度条目，与 SAR 参考形态一致）。
    result.abort_reason = session::SbirsPipelineAbortReason::kValidationRejected;
    result.status = session::SbirsCycleStatus::kRejectedInvalidInput;
    // 中译：SBIRS 输入校验拒绝（周期号）。
    // 标识：三写之三（人读日志）——调用方输入非法，本周期未执行；
    //       仅用于人读，不用于状态判断（规则 3）。
    PROJECT_LOG_WARN("SBIRS validation rejected for cycle_index={}", input.cycle_index);
    return result;
  }

  const pipeline::SbirsPipelineResult pipeline_result = pipeline_.RunCycle(input);

  if (!pipeline_result.executed) {
    // detail_code 为 SbirsIssueCodes.h 完整 code 常量（"sbirs.sensor_powered_off"）。
    session::RecordAbort(&result, session::SbirsPipelineAbortReason::kSensorPoweredOff,
                         session::codes::kSensorPoweredOff,
                         "SBIRS sensor is powered off or in standby mode.");
    result.status = session::SbirsCycleStatus::kPoweredOff;
    result.output_frame.cycle_index = input.cycle_index;
    result.output_frame.scan_azimuth_rad = pipeline_result.scan_azimuth_rad;
    result.output_frame.scan_elevation_rad = pipeline_result.scan_elevation_rad;
    return result;
  }

  result.output_frame.cycle_index = input.cycle_index;
  result.output_frame.scan_azimuth_rad = pipeline_result.scan_azimuth_rad;
  result.output_frame.scan_elevation_rad = pipeline_result.scan_elevation_rad;
  // 规则 13b：正常执行周期按目标排除的 kInfo 诊断并入统一问题列表（abort 路径不变）。
  result.issues = pipeline_result.issues;
  result.wide_cue_measurements = pipeline_result.wide_cue_measurements;  // cross-cue 外发数据源（修订 7）
  // raw output 仅进 detected==true 的 record；失败诊断 attribution 仍保留进 result 层。
  for (const pipeline::SbirsPipelineDetection& detection : pipeline_result.detections) {
    result.detection_attributions.push_back(detection.attribution);
    if (!detection.record.detected) {
      continue;
    }
    result.output_frame.detections.push_back(detection.record);
  }
  result.status = session::SbirsCycleStatus::kCompleted;
  result.abort_reason = session::SbirsPipelineAbortReason::kNone;
  return result;
}

}  // namespace runtime
}  // namespace sbirs_sensor
