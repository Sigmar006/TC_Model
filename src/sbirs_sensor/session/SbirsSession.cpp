#include "1q/sbirs_sensor/session/SbirsSession.h"

/**
 * ## 过程：红外会话生命周期（红外处理链 · 会话层）
 * - 上游：外部模型每周期传入 `SbirsCycleInput`（卫星 ECEF/速度、姿态、场景真值、UTC）
 * - 本步：① 组装控制器（配置映射 `SbirsPipelineConfigMapper`）→ ② `Step()` 转发控制器 →
 *         ③ 从周期结果抽取探测事件/排除原因（`SbirsDetectionLifecycleRecorder`、
 *            `SbirsExclusionCauseRecorder`）→ ④ 落验收行（`SbirsAcceptanceRecords`）与文件
 * - 下游：`SbirsCycleResult`（输出帧 + issues + 事件）返回给外部
 * - 关键参数：`acceptance_log_enabled`、`satellite_entity_id`（验收行标注）
 * - 对应验收项：第 24/24B 项（事件）、第 26/27 项（精度行由融合侧写出）、第 58/67 项（接口）
 *
 * @note 会话层不改变物理结果，只做编排与旁路记录；同一套处理链同时服务宽场/窄场/联合输出。
 */

#include <memory>
#include <utility>
#include <vector>

#include "1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"
#include "1q/sbirs_sensor/session/SbirsExclusionCauseRecorder.h"
#include "common/logging/AcceptanceFileLog.h"
#include "sbirs_sensor/pipeline/SbirsAcceptanceLog.h"
#include "sbirs_sensor/pipeline/SbirsAcceptanceRecords.h"
#include "sbirs_sensor/runtime/SbirsPipelineConfigMapper.h"
#include "sbirs_sensor/runtime/SbirsRuntimeConfigResolver.h"
#include "sbirs_sensor/session/SbirsSessionCompositionRoot.h"

namespace sbirs_sensor {
namespace session {

struct SbirsSession::Impl {
  config::SbirsSessionConfig config{};
  std::unique_ptr<runtime::SbirsController> controller{};
  SbirsDetectionLifecycleRecorder* lifecycle_recorder{nullptr};
  SbirsExclusionCauseRecorder* exclusion_cause_recorder{nullptr};
  std::uint32_t satellite_entity_id{0U}; /**< 验收行标注用（卫星ID=/相对卫星ID=）。 */
};

SbirsSession::SbirsSession() : impl_(new Impl) {
  impl_->controller = CreateSbirsController(impl_->config);
}

SbirsSession::SbirsSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

SbirsSession::~SbirsSession() noexcept {
  // 会话结束补齐第7项顺延落盘的可探测性行（单星候选待定条目；双星齐备行已即时
  // 写出。会话析构后不再有新候选注册，首个析构会话刷同目录内待定条目）。
  if (impl_ != nullptr && impl_->config.acceptance_log_enabled) {
    pipeline::SbirsScopedAcceptanceLogEnable acceptance_log_scope(true);
    oneq::logging::ScopedAcceptanceLogOutputDirectory directory_scope(
        impl_->config.acceptance_log_output_directory);
    pipeline::FlushSbirsDetectabilityPending();
  }
}
SbirsSession::SbirsSession(SbirsSession&& other) noexcept : impl_(std::move(other.impl_)) {}
SbirsSession& SbirsSession::operator=(SbirsSession&& other) noexcept { impl_ = std::move(other.impl_); return *this; }

SbirsOutputFrame SbirsSession::Step(const SbirsCycleInput& input) {
  return StepWithResult(input).output_frame;
}

SbirsCycleResult SbirsSession::StepWithResult(const SbirsCycleInput& input) {
  pipeline::SbirsScopedAcceptanceLogEnable acceptance_log_scope(
      impl_->config.acceptance_log_enabled);
  oneq::logging::ScopedAcceptanceLogOutputDirectory directory_scope(
      impl_->config.acceptance_log_output_directory);
  SbirsCycleResult result = impl_->controller->RunOnce(input);
  if (impl_->lifecycle_recorder != nullptr) {
    const std::vector<SbirsDetectionLifecycleEvent> events =
        impl_->lifecycle_recorder->Update(input, result);
    if (SBIRS_ACCEPTANCE_LOG_ENABLED()) {
      const float sim_time_sec = static_cast<float>(input.cycle_index) * input.dt_sec;
      pipeline::WriteSbirsLifecycleEvents(impl_->satellite_entity_id, sim_time_sec,
                                          input.cycle_index, events, input);
    }
  }
  if (impl_->exclusion_cause_recorder != nullptr) {
    impl_->exclusion_cause_recorder->Update(input, result);
  }
  return result;
}

void SbirsSession::AttachDetectionLifecycleRecorder(SbirsDetectionLifecycleRecorder* recorder) noexcept {
  impl_->lifecycle_recorder = recorder;
}

void SbirsSession::AttachExclusionCauseRecorder(SbirsExclusionCauseRecorder* recorder) noexcept {
  impl_->exclusion_cause_recorder = recorder;
}

void SbirsSession::SetSatelliteEntityId(std::uint32_t satellite_entity_id) noexcept {
  impl_->satellite_entity_id = satellite_entity_id;
  impl_->controller->SetSatelliteEntityId(satellite_entity_id);
}

bool SbirsSession::TryApplyRuntimeConfig(const config::SbirsRuntimeConfigPatch& patch) {
  const runtime::SbirsRuntimeConfigResolution resolution =
      runtime::ResolveSbirsRuntimeConfigPatch(impl_->config, patch);
  if (!resolution.is_valid || !resolution.has_requested_update) {
    return false;
  }
  impl_->config = resolution.resolved_config;
  impl_->controller->ApplyConfig(runtime::MapSessionToInternal(impl_->config), resolution.impact);
  return true;
}

void SbirsSession::SubmitExternalCue(const SbirsExternalCue& cue) {
  impl_->controller->SubmitExternalCue(cue);
}

SbirsSession SbirsSession::Create(const config::SbirsSessionConfig& config) {
  std::unique_ptr<Impl> impl(new Impl);
  impl->config = config;
  impl->controller = CreateSbirsController(config);
  return SbirsSession(std::move(impl));
}

SbirsSession SbirsSession::CreateWithDiagnostics(const config::SbirsSessionConfig& config,
                                                 SbirsIssueList* issues) {
  if (issues != nullptr) {
    *issues = config::ValidateSbirsSessionConfig(config);
  }
  return Create(config);
}

}  // namespace session
}  // namespace sbirs_sensor
