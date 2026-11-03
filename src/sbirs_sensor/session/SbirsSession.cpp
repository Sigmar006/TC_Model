#include "1q/sbirs_sensor/session/SbirsSession.h"

#include <memory>
#include <utility>
#include <vector>

#include "1q/satellite_orbit/OrbitPropagator.h"
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
namespace {

/**
 * @brief 按配置建立卫星轨道传播器。
 * @param[in] config 会话配置
 * @return 启用且初始化成功时返回传播器；未启用或根数/历元非法时返回 nullptr（fail-closed）
 * @note 非法配置不建立传播状态：`PropagateSatelliteState` 随即返回 false，
 *       卫星状态继续由调用方逐周期提供；配置校验由 `CreateWithDiagnostics` 报告。
 */
std::unique_ptr<satellite_orbit::OrbitPropagator> MakeOrbitPropagator(
    const config::SbirsSessionConfig& config) {
  if (!config.orbit.propagation_enabled) {
    return std::unique_ptr<satellite_orbit::OrbitPropagator>();
  }
  std::unique_ptr<satellite_orbit::OrbitPropagator> propagator(
      new satellite_orbit::OrbitPropagator());
  propagator->SetStepSize(config.orbit.propagation_step_sec);
  propagator->SetJ2Enabled(config.orbit.j2_enabled);
  if (!propagator->Initialize(config.orbit.orbital_elements,
                              config.orbit.epoch_utc_julian_day)) {
    return std::unique_ptr<satellite_orbit::OrbitPropagator>();
  }
  return propagator;
}

}  // namespace

struct SbirsSession::Impl {
  config::SbirsSessionConfig config{};
  std::unique_ptr<runtime::SbirsController> controller{};
  std::unique_ptr<satellite_orbit::OrbitPropagator> orbit_propagator{};
  SbirsDetectionLifecycleRecorder* lifecycle_recorder{nullptr};
  SbirsExclusionCauseRecorder* exclusion_cause_recorder{nullptr};
  std::uint32_t satellite_entity_id{0U}; /**< 验收行标注用（卫星ID=/相对卫星ID=）。 */
};

SbirsSession::SbirsSession() : impl_(new Impl) {
  impl_->controller = CreateSbirsController(impl_->config);
  impl_->orbit_propagator = MakeOrbitPropagator(impl_->config);
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

bool SbirsSession::PropagateSatelliteState(double utc_julian_day, SbirsCycleInput* input) {
  if (impl_ == nullptr || impl_->orbit_propagator == nullptr || input == nullptr) {
    return false;
  }
  satellite_orbit::EcefState ecef;
  if (!impl_->orbit_propagator->PropagateToTime(utc_julian_day, &ecef)) {
    return false;
  }
  input->utc_julian_day = utc_julian_day;
  input->satellite_position_ecef_m =
      SbirsVector3M(ecef.position.x_m, ecef.position.y_m, ecef.position.z_m);
  input->satellite_velocity_ecef_m_per_s =
      SbirsVector3M(ecef.velocity.x_mps, ecef.velocity.y_mps, ecef.velocity.z_mps);
  return true;
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
  impl->orbit_propagator = MakeOrbitPropagator(config);
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
