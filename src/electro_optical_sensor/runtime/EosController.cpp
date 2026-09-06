#include "electro_optical_sensor/runtime/EosController.h"

#include <cstddef>
#include <memory>

#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosIssueCodes.h"
#include "common/logging/ProjectLog.h"
#include "electro_optical_sensor/foundation/EosLookAngles.h"
#include "electro_optical_sensor/pipeline/EosPipeline.h"
#include "electro_optical_sensor/session/EosDiagnosticUtils.h"

#include <utility>

namespace electro_optical_sensor {
namespace extension {

namespace {

constexpr std::uint32_t kControllerRuntimeStateSchemaVersion = 1U;

/**
 * @brief 公开 ENU 周期输入 → 管道内部球坐标输入（每周期一次派生）。
 * @note 体系球坐标（斜距/方位/仰角）是库内量测几何，由 ENU 位置 +
 *       platform_attitude_deg（Body->ENU）派生；退化几何在校验层已拒绝，
 *       此处防御性跳过。
 */
signal::pipeline::EosPipelineCycleInput BuildPipelineInput(
    const session::EosCycleInput& input) {
  signal::pipeline::EosPipelineCycleInput pipeline_input;
  pipeline_input.cycle_index = input.cycle_index;
  pipeline_input.dt_sec = input.dt_sec;
  pipeline_input.platform_altitude_m = input.platform_altitude_m;
  pipeline_input.scene.reserve(input.scene.size());
  for (const session::EosSceneTarget& target : input.scene) {
    signal::pipeline::EosPipelineSceneTarget scene_target;
    scene_target.target_id = target.target_id;
    scene_target.target_name = target.target_name;
    scene_target.appearance = target.appearance;
    if (!foundation::TryResolveEosLookAngles(
            target.position_x, target.position_y, target.position_z,
            input.platform_attitude_deg, &scene_target.range_m, &scene_target.azimuth_deg,
            &scene_target.elevation_deg)) {
      continue;
    }
    pipeline_input.scene.push_back(std::move(scene_target));
  }
  return pipeline_input;
}

session::EosCycleStatus DeriveCycleStatus(session::EosPipelineAbortReason reason) {
  switch (reason) {
    case session::EosPipelineAbortReason::kNone:
      return session::EosCycleStatus::kCompleted;
    case session::EosPipelineAbortReason::kSensorPoweredOff:
      return session::EosCycleStatus::kPoweredOff;
    case session::EosPipelineAbortReason::kValidationRejected:
      return session::EosCycleStatus::kRejectedInvalidInput;
    default:
      return session::EosCycleStatus::kRejectedExecution;
  }
}

bool IsCompatibleControllerRuntimeState(const extension::EosControllerRuntimeState& state,
                                        const void* owner_identity) {
  return state.owner_identity == owner_identity &&
         state.schema_version == kControllerRuntimeStateSchemaVersion;
}

}  // namespace

struct EosController::Impl {
  explicit Impl(signal::pipeline::EosPipeline& pipeline_ref) : pipeline(pipeline_ref) {}

  void ResetPerCycleFlags() {
    last_cycle_executed = false;
    last_abort_reason = session::EosPipelineAbortReason::kNone;
  }

  /** @brief 装配单周期聚合结果（COMMON-OQ-9：issues 直通，不经校验缓存）。 */
  session::EosCycleResult AssembleResult(const session::EosCycleInput& input,
                                         const session::EosIssueList& validation_issues) const {
    session::EosCycleResult result;
    result.input_cycle_index = input.cycle_index;
    // 统一问题列表（规则 14）：输入校验问题（phase=kInputValidation）在前，正常执行周期
    // 按目标排除的 kInfo 诊断（规则 13b）在后；abort 路径诊断由 RecordAbort 追加。
    session::EosIssueList issues = validation_issues;
    if (last_cycle_executed && has_latest_output) {
      result.output_frame = latest_output;
      result.detection_attributions = latest_detection_attributions;
      session::EosIssueList execution_issues = latest_issues;
      issues.insert(issues.end(), execution_issues.begin(), execution_issues.end());
    }
    result.issues = std::move(issues);
    result.abort_reason = last_abort_reason;

    // 三写：对所有非 kNone 且非校验拒绝的 abort_reason 写入 issues + 日志。
    // 校验拒绝时，校验问题本身就是 error 级诊断（规则 9 写二由它们承载），
    // 不再重复写入粗粒度条目。
    if (last_abort_reason != session::EosPipelineAbortReason::kNone &&
        last_abort_reason != session::EosPipelineAbortReason::kValidationRejected) {
      // 不可达兜底（值不属注册表；若命中会写入 issue.code）。
      const char* detail_code = "unknown";
      // 校验拒绝（kValidationRejected）不可达：外层 if 已排除，校验问题本身承载写二。
      switch (last_abort_reason) {
        case session::EosPipelineAbortReason::kSensorPoweredOff:
          detail_code = session::codes::kSensorPoweredOff;
          break;
        case session::EosPipelineAbortReason::kOutputContractViolation:
          detail_code = session::codes::kPipelineContractViolation;
          break;
        case session::EosPipelineAbortReason::kRuntimeStateRestoreRejected:
          detail_code = session::codes::kRuntimeStateRestoreRejected;
          break;
        default:
          break;
      }
      session::RecordAbort(&result, last_abort_reason, detail_code, "EOS cycle aborted.");
    }

    // status 由 abort_reason 单一推导（在 RecordAbort 之后，避免其覆盖链造成
    // powered-off 被标成 kRejectedExecution；与 ESR/AR 的 powered-off 语义对齐）。
    result.status = DeriveCycleStatus(last_abort_reason);

    return result;
  }

  signal::pipeline::EosPipeline& pipeline;
  session::EosOutputFrame latest_output{};
  attribution::EosDetectionAttributionRecordList latest_detection_attributions{};
  session::EosIssueList latest_issues{}; /**< 正常周期按目标排除的 kInfo 诊断（规则 13b）。 */
  bool has_latest_output{false};
  bool last_cycle_executed{false};
  session::EosPipelineAbortReason last_abort_reason{session::EosPipelineAbortReason::kNone};
  session::EosCycleResult latest_result{}; /**< 最近一次周期的聚合结果缓存（COMMON-OQ-9 直通装配）。 */
};

EosController::EosController(signal::pipeline::EosPipeline& pipeline) : impl_(new Impl(pipeline)) {}

EosController::~EosController() = default;

namespace {

bool IsEosExecuteResultContractValid(
    const extension::EosPipelineExecuteResult& execute_result,
    const ::electro_optical_sensor::session::EosCycleInput& input) {
  if (!execute_result.executed_this_cycle) {
    return false;
  }
  return execute_result.abort_reason == session::EosPipelineAbortReason::kNone;
}

}  // namespace

void EosController::RunOnce(const ::electro_optical_sensor::session::EosCycleInput& input) {
  const extension::EosPipelineRuntimeState previous_pipeline_state =
      impl_->pipeline.CaptureRuntimeState();
  impl_->ResetPerCycleFlags();

  const session::EosIssueList issues =
      session::ValidateEosCycleInput(input, impl_->pipeline.GetFrameRateHz());

  if (session::HasValidationError(issues)) {
    impl_->last_abort_reason = session::EosPipelineAbortReason::kValidationRejected;
    // 中译：EOS 周期输入校验被拒绝（周期号）。
    // 标识：输入校验失败——本周期不执行、输出为空帧；
    //       排查输入场景/时间字段等校验问题（详见 EosIssueList）。
    PROJECT_LOG_WARN("EOS validation rejected for cycle_index={}", input.cycle_index);
    impl_->latest_result = impl_->AssembleResult(input, issues);
    return;
  }

  const signal::pipeline::EosPipelineCycleInput pipeline_input = BuildPipelineInput(input);
  const extension::EosPipelineExecuteResult execute_result =
      impl_->pipeline.RunCycle(pipeline_input);

  if (!execute_result.executed_this_cycle &&
      execute_result.abort_reason == session::EosPipelineAbortReason::kSensorPoweredOff) {
    impl_->last_cycle_executed = false;
    impl_->last_abort_reason = session::EosPipelineAbortReason::kSensorPoweredOff;
    impl_->latest_result = impl_->AssembleResult(input, issues);
    return;
  }

  if (!IsEosExecuteResultContractValid(execute_result, input)) {
    const bool restore_ok = impl_->pipeline.RestoreRuntimeState(previous_pipeline_state);
    if (!restore_ok) {
      impl_->latest_output = session::EosOutputFrame{};
      impl_->latest_detection_attributions.clear();
      impl_->has_latest_output = false;
      impl_->last_cycle_executed = false;
      impl_->last_abort_reason = session::EosPipelineAbortReason::kRuntimeStateRestoreRejected;
      // 中译：EOS 流水线回滚失败（周期号）。
      // 标识：执行中止后的状态恢复失败——输出被清空、本周期视为未执行，
      //       防止脏状态泄漏到下一周期。
      PROJECT_LOG_ERROR("EOS pipeline rollback failed for cycle_index={}", input.cycle_index);
      impl_->latest_result = impl_->AssembleResult(input, issues);
      return;
    }
    impl_->latest_output = session::EosOutputFrame{};
    impl_->latest_detection_attributions.clear();
    impl_->has_latest_output = false;
    impl_->last_cycle_executed = false;
    impl_->last_abort_reason =
        execute_result.abort_reason == session::EosPipelineAbortReason::kNone
            ? session::EosPipelineAbortReason::kOutputContractViolation
            : execute_result.abort_reason;
    impl_->latest_result = impl_->AssembleResult(input, issues);
    return;
  }

  session::EosOutputFrame assembled_frame;
  assembled_frame.cycle_index = input.cycle_index;
  assembled_frame.scan_azimuth_deg = execute_result.scan_azimuth_deg;
  assembled_frame.detections = std::move(execute_result.detections);
  impl_->latest_output = assembled_frame;
  impl_->latest_detection_attributions = std::move(execute_result.detection_attributions);
  // 规则 13b：正常周期按目标排除的 kInfo 诊断转写（abort 路径不变）。
  impl_->latest_issues = execute_result.issues;
  impl_->has_latest_output = true;
  impl_->last_cycle_executed = true;
  // 中译：本周期已执行（周期号、探测数）。
  // 标识：执行成功摘要——确认周期正常完成并产出了探测记录。
  PROJECT_LOG_DEBUG("[EosController] cycle_index={} executed detections={}", input.cycle_index,
                    assembled_frame.detections.size());
  impl_->latest_result = impl_->AssembleResult(input, issues);
}

void EosController::SetSensorEntityId(std::uint32_t sensor_entity_id) {
  impl_->pipeline.SetSensorEntityId(sensor_entity_id);
}

bool EosController::ExecutedLatestCycle() const { return impl_->last_cycle_executed; }

::electro_optical_sensor::session::EosCycleResult EosController::BuildCycleResult() const {
  // COMMON-OQ-9：装配已在 RunOnce 内完成并缓存（issues 直通），此处仅返回缓存。
  return impl_->latest_result;
}

extension::EosControllerRuntimeState EosController::CaptureRuntimeState() const {
  extension::EosControllerRuntimeState state;
  state.owner_identity = this;
  state.schema_version = kControllerRuntimeStateSchemaVersion;
  state.latest_output = impl_->latest_output;
  state.latest_detection_attributions = impl_->latest_detection_attributions;
  state.has_latest_output = impl_->has_latest_output;
  state.last_cycle_executed = impl_->last_cycle_executed;
  state.last_abort_reason = impl_->last_abort_reason;
  state.pipeline_state = impl_->pipeline.CaptureRuntimeState();
  return state;
}

bool EosController::RestoreRuntimeState(const extension::EosControllerRuntimeState& state) {
  if (!IsCompatibleControllerRuntimeState(state, this)) {
    return false;
  }
  if (!impl_->pipeline.RestoreRuntimeState(state.pipeline_state)) {
    return false;
  }
  impl_->latest_output = state.latest_output;
  impl_->latest_detection_attributions = state.latest_detection_attributions;
  impl_->has_latest_output = state.has_latest_output;
  impl_->last_cycle_executed = state.last_cycle_executed;
  impl_->last_abort_reason = state.last_abort_reason;
  return true;
}

}  // namespace extension
}  // namespace electro_optical_sensor
