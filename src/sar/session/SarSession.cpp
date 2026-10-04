#include "1q/sar/session/SarSession.h"

#include <memory>
#include <utility>

#include "1q/sar/session/SarProductLifecycleRecorder.h"
#include "sar/runtime/SarController.h"
#include "sar/session/SarSessionCompositionRoot.h"

namespace sar {
namespace session {

struct SarSession::Impl {
  explicit Impl(SarSessionComposition composition)
      : owned_pipeline(std::move(composition.owned_pipeline)),
        owned_controller(std::move(composition.owned_controller)) {}

  std::unique_ptr<pipeline::SarProcessingPipeline> owned_pipeline;
  std::unique_ptr<extension::SarController> owned_controller;
  SarProductLifecycleRecorder* lifecycle_recorder{nullptr};

  extension::SarController& controller() const { return *owned_controller; }
};

SarSession::SarSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

SarSession::SarSession()
    : impl_(new Impl(SarSessionCompositionRoot::ComposeDefault(config::SarSessionConfig{}))) {}

SarSession::~SarSession() noexcept = default;
SarSession::SarSession(SarSession&& other) noexcept : impl_(std::move(other.impl_)) {}
SarSession& SarSession::operator=(SarSession&& other) noexcept { impl_ = std::move(other.impl_); return *this; }

SarSession SarSession::Create(const config::SarSessionConfig& config) {
  return SarSession(std::unique_ptr<SarSession::Impl>(
      new SarSession::Impl(SarSessionCompositionRoot::ComposeDefault(config))));
}

SarSession SarSession::CreateWithDiagnostics(const config::SarSessionConfig& config,
                                             SarIssueList* issues) {
  const SarIssueList found = config::ValidateSarSessionConfig(config);
  if (issues != nullptr) {
    *issues = found;
  }
  return Create(config);
}

SarCycleProduct SarSession::Step(const SarCycleInput& input) {
  // 规则 15c：产品层从同一份周期记录移动取出，非第二份副本。
  SarCycleResult result = StepWithResult(input);
  return std::move(result.product);
}

SarCycleResult SarSession::StepWithResult(const SarCycleInput& input) {
  extension::SarController& controller = impl_->controller();
  controller.RunOnce(input);
  SarCycleResult result = controller.BuildCycleResult();
  if (impl_->lifecycle_recorder != nullptr) {
    impl_->lifecycle_recorder->Update(result);
  }
  return result;
}

void SarSession::AttachProductLifecycleRecorder(SarProductLifecycleRecorder* recorder) noexcept {
  impl_->lifecycle_recorder = recorder;
}

bool SarSession::TryApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch) {
  // 立即提交类（见 docs/common/contract.md「运行期配置提交策略」）：调用即生效、单向
  // 落定、无 session 层回滚；执行期合法性由 controller 在 Step 内 gate。
  return impl_->controller().TryApplyRuntimeConfig(patch);
}

}  // namespace session
}  // namespace sar
