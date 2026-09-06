#include "1q/electro_optical_sensor/session/EosSession.h"

#include <cstdlib>
#include <utility>

#include "1q/electro_optical_sensor/session/EosDetectionLifecycleRecorder.h"
#include "1q/electro_optical_sensor/session/EosExclusionCauseRecorder.h"
#include "electro_optical_sensor/runtime/EosController.h"
#include "common/logging/ProjectLog.h"
#include "electro_optical_sensor/runtime/EosRuntimeConfigResolver.h"
#include "electro_optical_sensor/session/EosSessionCompositionRoot.h"
#include "electro_optical_sensor/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace session {

namespace {

/**
 * @brief 校验组合根输出的依赖指针非空并返回引用。
 * @param[in] ptr 待校验指针。
 * @param[in] dependency_name 依赖名称，用于日志与故障定位。
 * @return 非空依赖引用。
 * @warning 若组合结果违反约束且传入空指针，将终止进程以避免未定义行为。
 */
template <typename T>
T& RequireCompositionDependency(T* ptr, const char* dependency_name) {
  if (ptr != nullptr) {
    return *ptr;
  }
  // 中译：会话组合依赖「{}」为空指针。
  // 标识：组合装配错误——依赖未注入即被使用，属内部装配缺陷；
  //       随后将终止进程以避免未定义行为（见上方 @warning）。
  PROJECT_LOG_ERROR("[EosSession] Composition dependency '{}' is null.", dependency_name);
  std::abort();
}

}  // namespace

struct EosSession::Impl {
  explicit Impl(EosSessionComposition composition)
      : owned_pipeline(std::move(composition.owned_pipeline)),
        owned_controller(std::move(composition.owned_controller)),
        internal_config_(composition.internal_config) {
    RequireCompositionDependency(owned_controller.get(), "controller");
  }

  std::unique_ptr<signal::pipeline::EosPipeline> owned_pipeline;
  std::unique_ptr<extension::EosController> owned_controller;
  config::execution::EosInternalExecutionConfig internal_config_;
  EosDetectionLifecycleRecorder* lifecycle_recorder{nullptr};
  EosExclusionCauseRecorder* exclusion_cause_recorder{nullptr};
  std::uint32_t sensor_entity_id{0U};

  void SetSensorEntityId(std::uint32_t id) noexcept {
    sensor_entity_id = id;
    controller().SetSensorEntityId(id);
  }

  extension::EosController& controller() const { return *owned_controller; }
};

EosSession::EosSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

EosSession::EosSession()
    : impl_(new Impl(EosSessionCompositionRoot::ComposeDefault(config::EosSessionConfig{}))) {}

EosSession::~EosSession() noexcept = default;
EosSession::EosSession(EosSession&&) noexcept = default;
EosSession& EosSession::operator=(EosSession&&) noexcept = default;

EosSession EosSession::Create(const config::EosSessionConfig& config) {
  return EosSession(std::unique_ptr<EosSession::Impl>(
      new EosSession::Impl(EosSessionCompositionRoot::ComposeDefault(config))));
}

EosSession EosSession::CreateWithDiagnostics(const config::EosSessionConfig& config,
                                             EosIssueList* issues) {
  const EosIssueList found = config::ValidateEosSessionConfig(config);
  if (issues != nullptr) {
    *issues = found;
  }
  return Create(config);
}

session::EosOutputFrame EosSession::Step(const EosCycleInput& input) {
  return StepWithResult(input).output_frame;
}

::electro_optical_sensor::session::EosCycleResult EosSession::StepWithResult(
    const EosCycleInput& input) {
  extension::EosController& controller = impl_->controller();
  controller.RunOnce(input);
  EosCycleResult result = controller.BuildCycleResult();
  if (impl_->lifecycle_recorder != nullptr) {
    impl_->lifecycle_recorder->Update(input, result);
  }
  if (impl_->exclusion_cause_recorder != nullptr) {
    impl_->exclusion_cause_recorder->Update(input, result);
  }
  return result;
}

void EosSession::AttachDetectionLifecycleRecorder(EosDetectionLifecycleRecorder* recorder) noexcept {
  impl_->lifecycle_recorder = recorder;
}

void EosSession::AttachExclusionCauseRecorder(EosExclusionCauseRecorder* recorder) noexcept {
  impl_->exclusion_cause_recorder = recorder;
}

void EosSession::SetSensorEntityId(std::uint32_t sensor_entity_id) noexcept {
  impl_->SetSensorEntityId(sensor_entity_id);
}

bool EosSession::TryApplyRuntimeConfig(const config::EosRuntimeConfigPatch& patch) {
  // 立即提交类（见 docs/common/contract.md「运行期配置提交策略」）：调用即生效、
  // 配置单向落定、不在 session 层回滚。执行期的 pipeline 状态回滚边界在
  // EosController::RunOnce 内（EosController.cpp:68-111），不上升为本方法契约。
  const ::electro_optical_sensor::runtime::session::EosRuntimeConfigResolveResult
      resolved = ::electro_optical_sensor::runtime::session::ResolveEosRuntimeConfigPatch(
          impl_->internal_config_, patch);
  if (!resolved.has_requested_update || !resolved.is_valid) {
    return false;
  }
  impl_->internal_config_ = resolved.next_config;
  if (impl_->owned_pipeline != nullptr) {
    impl_->owned_pipeline->ApplyInternalConfig(
        impl_->internal_config_, resolved.reset_scan_phase);
  }
  return true;
}

}  // namespace session
}  // namespace electro_optical_sensor
