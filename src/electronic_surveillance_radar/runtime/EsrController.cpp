#include "electronic_surveillance_radar/runtime/EsrController.h"

#include <memory>

#include "1q/coordinate/position_transform.h"
#include "electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"
#include "electronic_surveillance_radar/session/EsrDiagnosticUtils.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrIssueCodes.h"
#include "common/logging/ProjectLog.h"
#include "1q/foundation/RuntimeCycleExecutor.h"

namespace electronic_surveillance_radar {
namespace extension {

namespace {

session::EsrCycleExecutionStatus DeriveCycleStatus(session::EsrPipelineAbortReason reason) {
  switch (reason) {
    case session::EsrPipelineAbortReason::kNone:
      return session::EsrCycleExecutionStatus::kCompleted;
    case session::EsrPipelineAbortReason::kSensorPoweredOff:
      return session::EsrCycleExecutionStatus::kPoweredOff;
    default:
      return session::EsrCycleExecutionStatus::kRejected;
  }
}

}  // namespace

struct EsrController::Impl {
  Impl(pipeline::InterceptPipeline& pipeline_ref,
       environment::IEsrEnvironmentService& environment_service_ref)
      : pipeline(pipeline_ref), environment_service(environment_service_ref) {}

  /** @brief 装配单周期聚合结果（COMMON-OQ-9：issues 直通，不经校验缓存）。 */
  session::EsrCycleResult AssembleResult(const session::EsrCycleInput& input,
                                         const session::EsrIssueList& validation_issues) const {
    session::EsrCycleResult result;
    result.input_cycle_index = input.cycle_index;
    // 统一问题列表（规则 14）：输入校验问题（phase=kInputValidation）在前，正常执行周期
    // 按发射源排除的 kInfo 诊断（规则 13b）在后；abort 路径诊断由 RecordAbort 追加。
    session::EsrIssueList issues = validation_issues;
    if (last_abort_reason == session::EsrPipelineAbortReason::kNone &&
        runtime_state.has_latest_output) {
      result.output_frame = runtime_state.latest_output;
      session::EsrIssueList execution_issues = latest_issues;
      issues.insert(issues.end(), execution_issues.begin(), execution_issues.end());
    }
    result.issues = std::move(issues);
    result.abort_reason = last_abort_reason;

    // 三写：对所有非 kNone 且非校验拒绝的 abort_reason 写入 issues + 日志。
    // 校验拒绝时，校验问题本身就是 error 级诊断（规则 9 写二由它们承载），
    // 不再重复写入粗粒度条目。
    if (last_abort_reason != session::EsrPipelineAbortReason::kNone &&
        last_abort_reason != session::EsrPipelineAbortReason::kValidationRejected) {
      // 不可达兜底（值不属注册表；若命中会写入 issue.code）。
      const char* detail_code = "unknown";
      // 校验拒绝（kValidationRejected）不可达：外层 if 已排除，校验问题本身承载写二。
      switch (last_abort_reason) {
        case session::EsrPipelineAbortReason::kSensorPoweredOff:
          detail_code = session::codes::kSensorPoweredOff;
          break;
        case session::EsrPipelineAbortReason::kRfReceiverRejected:
          detail_code = session::codes::kRfReceiverRejected;
          break;
        default:
          break;
      }
      session::RecordAbort(&result, last_abort_reason, detail_code, "ESR cycle aborted.");
    }

    // status 由 abort_reason 单一推导（在 RecordAbort 之后，避免其覆盖链造成
    // powered-off 被标成 kRejected；与 EOS 的 DeriveCycleStatus 形态对齐）。
    result.status = DeriveCycleStatus(last_abort_reason);

    return result;
  }

  pipeline::InterceptPipeline& pipeline;
  environment::IEsrEnvironmentService& environment_service;
  oneq::foundation::RuntimeCycleState<session::EsrOutputFrame> runtime_state{};
  session::EsrPipelineAbortReason last_abort_reason{session::EsrPipelineAbortReason::kNone};
  session::EsrIssueList latest_issues{}; /**< 正常周期按发射源排除的 kInfo 诊断（规则 13b）。 */
  session::EsrCycleResult latest_result{}; /**< 最近一次周期的聚合结果缓存（COMMON-OQ-9 直通装配）。 */
};

EsrController::EsrController(pipeline::InterceptPipeline& pipeline,
                             environment::IEsrEnvironmentService& environment_service)
    : impl_(new Impl(pipeline, environment_service)) {}

EsrController::~EsrController() = default;

void EsrController::RunOnce(const session::EsrCycleInput& input) {
  const oneq::foundation::RuntimeCycleStamp stamp =
      oneq::foundation::MakeRuntimeCycleStamp(
          input.cycle_index, impl_->runtime_state.next_batch_id);

  // 校验（COMMON-OQ-9：issues 直通装配，不经校验缓存）
  session::EsrIssueList issues = session::ValidateEsrCycleInput(input);

  if (session::HasValidationError(issues)) {
    impl_->last_abort_reason = session::EsrPipelineAbortReason::kValidationRejected;
    // 中译：ESR 周期输入校验被拒绝（周期号）。
    // 标识：输入校验失败——本周期不执行、输出为空；
    //       排查输入字段校验问题（详见 EsrIssueList）。
    PROJECT_LOG_WARN("ESR validation rejected for cycle_index={}", stamp.cycle_index);
    impl_->latest_result = impl_->AssembleResult(input, issues);
    return;
  }

  // 冻结环境
  {
    oneq::coordinate::LlaPositionDegM platform_lla;
    if (!oneq::coordinate::TryEcefToLla(input.platform_position_ecef_m, &platform_lla)) {
      impl_->last_abort_reason = session::EsrPipelineAbortReason::kValidationRejected;
      impl_->latest_result = impl_->AssembleResult(input, issues);
      return;
    }
    impl_->environment_service.BeginCycle(
        stamp.cycle_index, input.dt_sec, static_cast<float>(platform_lla.altitude_m));
  }

  // 执行
  extension::InterceptPipelineResult pipeline_result =
      impl_->pipeline.RunCycle(input, impl_->environment_service);
  if (pipeline_result.rf_v2_rejected) {
    impl_->last_abort_reason = session::EsrPipelineAbortReason::kRfReceiverRejected;
    // 中译：ESR RF v2 接收机拒绝了本周期（周期号）。
    // 标识：射频接收级拒绝——周期不产出观测；排查 RF 帧与接收机窗口。
    PROJECT_LOG_WARN("ESR RF v2 receiver rejected cycle_index={}", stamp.cycle_index);
    impl_->latest_result = impl_->AssembleResult(input, issues);
    return;
  }
  if (pipeline_result.sensor_powered_off) {
    impl_->last_abort_reason = session::EsrPipelineAbortReason::kSensorPoweredOff;
    impl_->latest_result = impl_->AssembleResult(input, issues);
    return;
  }
  session::EsrOutputFrame output_frame;
  output_frame.cycle_index = stamp.cycle_index;
  output_frame.batch_id = stamp.batch_id;
  output_frame.scan_azimuth_deg = pipeline_result.scan_azimuth_deg;
  output_frame.observation_output = std::move(pipeline_result.observation_output);
  output_frame.emitter_output = std::move(pipeline_result.emitter_output);
  // 规则 13b：正常周期按发射源排除的 kInfo 诊断转写（abort 路径不变）。
  impl_->latest_issues = std::move(pipeline_result.issues);

  impl_->runtime_state.latest_output = std::move(output_frame);
  impl_->runtime_state.has_latest_output = true;
  impl_->last_abort_reason = session::EsrPipelineAbortReason::kNone;
  ++impl_->runtime_state.next_batch_id;
  // 中译：本周期已执行（周期号、观测数、假设数）。
  // 标识：执行成功摘要——确认周期正常完成并产出了观测与假设。
  PROJECT_LOG_DEBUG(
      "[EsrController] cycle_index={} executed obs={} hyp={}",
      stamp.cycle_index,
      impl_->runtime_state.latest_output.observation_output.observations.size(),
      impl_->runtime_state.latest_output.emitter_output.hypotheses.size());
  impl_->latest_result = impl_->AssembleResult(input, issues);
}

bool EsrController::HasLatestInterceptOutputFrame() const { return impl_->runtime_state.has_latest_output; }

const session::EsrOutputFrame& EsrController::GetLatestInterceptOutputFrame() const {
  return impl_->runtime_state.latest_output;
}

void EsrController::SetSensorEntityId(std::uint32_t sensor_entity_id) {
  impl_->pipeline.SetSensorEntityId(sensor_entity_id);
}

session::EsrCycleResult EsrController::BuildCycleResult() const {
  // COMMON-OQ-9：装配已在 RunOnce 内完成并缓存（issues 直通），此处仅返回缓存。
  return impl_->latest_result;
}

environment::IEsrEnvironmentService& EsrController::GetEnvironmentService() {
  return impl_->environment_service;
}

EsrControllerRuntimeState EsrController::CaptureRuntimeState() const {
  EsrControllerRuntimeState state;
  state.owner_identity = this;
  state.schema_version = 2U;
  state.has_latest_output = impl_->runtime_state.has_latest_output;
  state.latest_output = impl_->runtime_state.latest_output;
  state.next_batch_id = impl_->runtime_state.next_batch_id;
  state.last_abort_reason = impl_->last_abort_reason;
  return state;
}

bool EsrController::RestoreRuntimeState(const EsrControllerRuntimeState& state) {
  if (state.owner_identity != this || state.schema_version != 2U) {
    return false;
  }
  impl_->runtime_state.has_latest_output = state.has_latest_output;
  impl_->runtime_state.latest_output = state.latest_output;
  impl_->runtime_state.next_batch_id = state.next_batch_id;
  impl_->last_abort_reason = state.last_abort_reason;
  return true;
}

}  // namespace extension

}  // namespace electronic_surveillance_radar
