#include "1q/electronic_surveillance_radar/session/EsrSession.h"

#include <utility>

#include "electronic_surveillance_radar/config/EsrInternalExecutionConfig.h"
#include "electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"
#include "electronic_surveillance_radar/runtime/EsrController.h"
#include "1q/electronic_surveillance_radar/session/EsrExclusionCauseRecorder.h"
#include "electronic_surveillance_radar/session/EsrRuntimeConfigResolver.h"
#include "electronic_surveillance_radar/session/EsrSessionCompositionRoot.h"

namespace electronic_surveillance_radar {
namespace session {

struct EsrSession::Impl {
  explicit Impl(EsrSessionComposition composition)
      : owned_pipeline(std::move(composition.owned_pipeline)),
        owned_environment_service(std::move(composition.owned_environment_service)),
        owned_controller(std::move(composition.owned_controller)) {
    resolved_config = std::move(composition.execution_config);
  }

  /**
   * @brief 执行单周期并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   * @note COMMON-OQ-9 收敛后装配与校验缓存均在 EsrController 内（issues 直通），
   *       session 仅透传；controller 各 abort 路径均不推进 pipeline 累积状态，
   *       原 session 层周期内快照回滚分支不可达，已随装配一并移除。
   */
  EsrCycleResult RunCycle(const session::EsrCycleInput& input) {
    Controller().RunOnce(input);
    EsrCycleResult result = Controller().BuildCycleResult();
    if (exclusion_cause_recorder != nullptr) {
      exclusion_cause_recorder->Update(input, result);
    }
    return result;
  }

  EsrInternalExecutionConfig resolved_config{};
  std::unique_ptr<pipeline::InterceptPipeline> owned_pipeline;
  std::unique_ptr<environment::IEsrEnvironmentService> owned_environment_service;
  std::unique_ptr<extension::EsrController> owned_controller;
  EsrExclusionCauseRecorder* exclusion_cause_recorder{nullptr};
  std::uint32_t sensor_entity_id{0U};

  void SetSensorEntityId(std::uint32_t id) noexcept {
    sensor_entity_id = id;
    Controller().SetSensorEntityId(id);
  }

  pipeline::InterceptPipeline& Pipeline() const { return *owned_pipeline; }
  environment::IEsrEnvironmentService& EnvironmentService() const {
    return *owned_environment_service;
  }
  extension::EsrController& Controller() const { return *owned_controller; }
};

EsrSession::EsrSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

EsrSession::EsrSession()
    : impl_(new Impl(EsrSessionCompositionRoot::ComposeDefault(config::EsrSessionConfig{}))) {}

EsrSession::~EsrSession() = default;

EsrSession::EsrSession(EsrSession&& other) noexcept = default;

EsrSession& EsrSession::operator=(EsrSession&& other) noexcept = default;

session::EsrOutputFrame EsrSession::Step(const session::EsrCycleInput& input) {
  return impl_->RunCycle(input).output_frame;
}

EsrCycleResult EsrSession::StepWithResult(const session::EsrCycleInput& input) {
  return impl_->RunCycle(input);
}

void EsrSession::AttachExclusionCauseRecorder(EsrExclusionCauseRecorder* recorder) noexcept {
  impl_->exclusion_cause_recorder = recorder;
}

void EsrSession::SetSensorEntityId(std::uint32_t sensor_entity_id) noexcept {
  impl_->SetSensorEntityId(sensor_entity_id);
}

bool EsrSession::TryApplyRuntimeConfig(const config::EsrRuntimeConfigPatch& patch) {
  return ApplyRuntimeConfigWithResult(patch).applied;
}

EsrRuntimeConfigApplyResult EsrSession::ApplyRuntimeConfigWithResult(
    const config::EsrRuntimeConfigPatch& patch) {
  EsrRuntimeConfigApplyResult apply_result;
  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(impl_->resolved_config, patch);
  apply_result.status = resolved.status;
  apply_result.has_requested_update = resolved.has_requested_update;
  if (!resolved.has_requested_update || !resolved.is_valid) {
    apply_result.applied = false;
    return apply_result;
  }

  impl_->resolved_config = resolved.next_config;
  if (resolved.runtime_config_changed || resolved.pipeline_config_changed) {
    // 写路径直接吃 internal config（与 EOS/AR 一致），避免 internal→extension→internal
    // 往返。RunCycle 的 per-cycle MutableEsrContext 投影仍在 RunCycle 内进行。
    impl_->Pipeline().UpdateConfig(impl_->resolved_config);
  }
  if (resolved.environment_model_config_changed) {
    impl_->EnvironmentService().UpdateModelConfig(impl_->resolved_config.environment);
  }
  apply_result.applied = true;
  return apply_result;
}

// ── EsrSession static factory ──────────────────────────────────────────────────────

EsrSession EsrSession::Create(const config::EsrSessionConfig& config) {
  return EsrSession(std::unique_ptr<EsrSession::Impl>(
      new EsrSession::Impl(EsrSessionCompositionRoot::ComposeDefault(config))));
}

EsrSession EsrSession::CreateWithDiagnostics(const config::EsrSessionConfig& config,
                                             EsrIssueList* issues) {
  const EsrIssueList found = config::ValidateEsrSessionConfig(config);
  if (issues != nullptr) {
    *issues = found;
  }
  return Create(config);
}

}  // namespace session

}  // namespace electronic_surveillance_radar
