#include "airborne_radar/runtime/ArController.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>

#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/ArTrackOutput.h"
#include "airborne_radar/decision/ControlReducer.h"
#include "airborne_radar/decision/TacticalCoordinator.h"
#include "airborne_radar/environment/IEnvironmentService.h"
#include "airborne_radar/runtime/ControlCommandMapper.h"
#include "airborne_radar/session/MutableArContext.h"
#include "airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "common/logging/ProjectLog.h"
#include "1q/foundation/RuntimeCycleExecutor.h"

namespace airborne_radar {
namespace extension {

namespace {

bool HasOperationalProfileChanged(const session::ArControlProfile& previous,
                                  const session::ArControlProfile& next) {
  return previous.enable_lpi_power_control != next.enable_lpi_power_control ||
         previous.lpi_power_scale != next.lpi_power_scale ||
         previous.enable_lpi_beamforming != next.enable_lpi_beamforming ||
         previous.lpi_dwell_scale != next.lpi_dwell_scale ||
         previous.enable_agility_frequency != next.enable_agility_frequency ||
         previous.agility_frequency_hop_phase != next.agility_frequency_hop_phase ||
         previous.enable_sidelobe_canceller != next.enable_sidelobe_canceller ||
         previous.enable_adaptive_beamforming != next.enable_adaptive_beamforming ||
         previous.enable_eccm_rejitter != next.enable_eccm_rejitter ||
         previous.eccm_burnthrough_gain != next.eccm_burnthrough_gain ||
         previous.enable_anti_rgpo_leading_edge != next.enable_anti_rgpo_leading_edge ||
         previous.enable_anti_vgpo_acceleration_bound != next.enable_anti_vgpo_acceleration_bound ||
         previous.enable_anti_false_target_discrimination != next.enable_anti_false_target_discrimination;
}

extension::ControlReducerConfig MapDecisionControlConfig(
    const config::DecisionControlConfig& config) {
  extension::ControlReducerConfig mapped;
  mapped.lpi_hold_cycles_after_request = config.lpi_hold_cycles_after_request;
  mapped.eccm_hold_cycles_after_request = config.eccm_hold_cycles_after_request;
  mapped.lpi_cooldown_cycles_after_release = config.lpi_cooldown_cycles_after_release;
  mapped.eccm_cooldown_cycles_after_release = config.eccm_cooldown_cycles_after_release;
  return mapped;
}

}  // namespace

/**
 * @brief 控制器内部自有组件，生命周期由 Impl 管理。
 *
 * 统一封装默认决策子系统。
 */
struct OwnedDecisionComponents {
  std::unique_ptr<decision::TacticalCoordinator> decision_engine;
  std::unique_ptr<session::TacticalStateStore> tactical_state_store;
  std::unique_ptr<decision::ControlReducer> control_reducer;
};

/**
 * @brief ArController 内部实现体，持有所有运行时状态。
 */
struct ArController::Impl {
  // -- 外部引用（生命周期由 Session 管理）
  session::MutableArContext& radar_context;
  signal::ISignalPipeline& signal_pipeline;
  environment::IEnvironmentService& environment_service;

  // -- 决策子系统
  OwnedDecisionComponents owned_decision_components;

  // -- 控制状态
  session::ArControlProfile control_profile{};

  // -- 独立生命周期组件
  std::unique_ptr<extension::ControlCommandMapper> command_mapper;

  // -- 周期运行时状态
  oneq::foundation::RuntimeCycleState<session::TrackOutputFrame> cycle_state{};
  bool last_cycle_executed{false};
  session::SignalCycleAbortReason last_signal_abort_reason{session::SignalCycleAbortReason::kNone};
  session::ArIssueList latest_issues{}; /**< 正常周期按目标排除的 kInfo 诊断（规则 13b）。 */

  bool has_pending_internal_decision{false};
  std::uint32_t pending_internal_cycle_index{0U};
  std::uint64_t pending_internal_batch_id{0U};
  std::vector<session::TacticalProposal> pending_internal_proposals{};
  bool has_pending_external_override{false};
  session::ExternalDecisionOverride pending_external_override{};
  session::DecisionControlSource last_applied_decision_source{
      session::DecisionControlSource::kNone};
  std::uint32_t last_applied_decision_cycle_index{0U};
  std::uint64_t last_applied_decision_batch_id{0U};
  std::vector<session::TacticalProposal> last_applied_decision_proposals{};
  bool control_prepared_for_cycle{false};

  /** @brief 构造使用默认 TacticalCoordinator 的控制器。 */
  Impl(session::MutableArContext& ctx, signal::ISignalPipeline& sig,
       environment::IEnvironmentService& env,
       const config::DecisionControlConfig& decision_control_config)
      : radar_context(ctx),
        signal_pipeline(sig),
        environment_service(env) {
    owned_decision_components.decision_engine.reset(new decision::TacticalCoordinator(nullptr));
    owned_decision_components.tactical_state_store.reset(new session::TacticalStateStore());
    owned_decision_components.control_reducer.reset(
        new decision::ControlReducer(MapDecisionControlConfig(decision_control_config)));
    command_mapper.reset(
        new extension::ControlCommandMapper(*owned_decision_components.control_reducer, ctx));
  }

  /** @brief 重置每周期可变标志位。 */
  void ResetPerCycleFlags(bool preserve_prepared_control_attribution) {
    last_cycle_executed = false;
    last_signal_abort_reason = session::SignalCycleAbortReason::kNone;
    if (!preserve_prepared_control_attribution) {
      last_applied_decision_source = session::DecisionControlSource::kNone;
      last_applied_decision_cycle_index = 0U;
      last_applied_decision_batch_id = 0U;
      last_applied_decision_proposals.clear();
    }
  }

  /** @brief 校验外部覆盖返回的 profile 字段是否在合法范围内。 */
  static bool IsValidOverrideProfile(const session::ArControlProfile& profile) {
    if (profile.enable_lpi_power_control) {
      if (!std::isfinite(profile.lpi_power_scale) || profile.lpi_power_scale <= 0.0f ||
          profile.lpi_power_scale > 1.0f) {
        return false;
      }
    }
    if (profile.lpi_dwell_scale != 1.0f) {
      if (!std::isfinite(profile.lpi_dwell_scale) || profile.lpi_dwell_scale < 0.25f ||
          profile.lpi_dwell_scale > 1.0f) {
        return false;
      }
    }
    if (profile.eccm_burnthrough_gain != 1.0f) {
      if (!std::isfinite(profile.eccm_burnthrough_gain) || profile.eccm_burnthrough_gain <= 1.0f ||
          profile.eccm_burnthrough_gain > 2.0f) {
        return false;
      }
    }
    if (profile.agility_frequency_hop_phase > 1) {
      return false;
    }
    return true;
  }

  /** @brief 将外部覆盖应用到原生 profile 上，返回最终 profile。 */
  static session::ArControlProfile ApplyExternalOverride(
    const session::ArControlProfile& native_profile,
    const session::ExternalDecisionOverride& override) {
  session::ArControlProfile override_profile = override.profile;
  if (!IsValidOverrideProfile(override_profile)) {
    return native_profile;
  }
    override_profile.version = HasOperationalProfileChanged(native_profile, override_profile)
                                   ? native_profile.version + 1U
                                   : native_profile.version;
    return override_profile;
  }

  void ApplyPendingDecisionControl() {
    if (!has_pending_internal_decision) {
      return;
    }

    // 1. 原生归约（始终执行）
    const extension::ControlReductionResult native_result =
        command_mapper->Apply(&control_profile, pending_internal_proposals);

    // 2. 外部覆盖
    if (has_pending_external_override) {
      const session::ArControlProfile override_profile =
          ApplyExternalOverride(control_profile, pending_external_override);
      if (override_profile.version != control_profile.version) {
        control_profile = override_profile;
        radar_context.UpdateRadarControlProfile(control_profile);
        // 对差异字段生成 ArCommand
        const std::vector<session::ControlDirective> diffs =
            command_mapper->DiffProfiles(native_result.profile, control_profile);
        for (std::size_t i = 0; i < diffs.size(); ++i) {
          const session::ArCommand cmd =
              extension::ControlCommandMapper::DirectiveToCommand(diffs[i]);
          if (cmd.type != session::ArCommandType::NONE) {
            radar_context.SubmitControlCommand(cmd);
          }
        }
      }
      last_applied_decision_source = session::DecisionControlSource::kExternal;
    } else {
      last_applied_decision_source = session::DecisionControlSource::kInternal;
    }

    last_applied_decision_cycle_index = pending_internal_cycle_index;
    last_applied_decision_batch_id = pending_internal_batch_id;
    last_applied_decision_proposals = pending_internal_proposals;
    has_pending_internal_decision = false;
    pending_internal_proposals.clear();
    has_pending_external_override = false;
    pending_external_override = session::ExternalDecisionOverride();
  }
};

// -- 构造函数

ArController::ArController(session::MutableArContext& radar_context,
                           signal::ISignalPipeline& signal_pipeline,
                           environment::IEnvironmentService& environment_service,
                           config::DecisionControlConfig decision_control_config)
    : impl_(new Impl(radar_context, signal_pipeline, environment_service, decision_control_config)) {}

ArController::~ArController() = default;

void ArController::UpdateDecisionControlConfig(
    const config::DecisionControlConfig& decision_control_config) {
  impl_->owned_decision_components.control_reducer->UpdateConfig(
      MapDecisionControlConfig(decision_control_config));
}

void ArController::SetSensorEntityId(std::uint32_t sensor_entity_id) {
  impl_->signal_pipeline.SetSensorEntityId(sensor_entity_id);
}

void ArController::RunOnce(const signal::pipeline::SignalCycleInput& cycle_input,
                           session::ArIssueList* validation_issues_out) {
  const bool control_was_prepared = impl_->control_prepared_for_cycle;
  impl_->ResetPerCycleFlags(control_was_prepared);

  const session::ArSceneTargetList& scene_targets = impl_->radar_context.GetSceneTargets();
  const config::PlatformAttitudeDeg platform_attitude = impl_->radar_context.GetPlatformAttitude();
  const float platform_altitude_m = impl_->radar_context.GetPlatformAltitudeM();
  const float cycle_dt_sec = impl_->radar_context.GetCycleDeltaTimeSec();
  const std::uint32_t cycle_index = impl_->radar_context.GetCycleIndex();

  const oneq::foundation::RuntimeCycleStamp stamp =
      oneq::foundation::MakeRuntimeCycleStamp(cycle_index, impl_->cycle_state.next_batch_id);

  // 校验（COMMON-OQ-9：拒绝时明细经出参直通，无校验缓存）
  session::ArIssueList issues = session::ValidateArCycleDeltaTime(cycle_dt_sec);
  {
    const session::ArIssueList target_issues =
        session::ValidateArSceneTargets(scene_targets);
    issues.insert(issues.end(), target_issues.begin(), target_issues.end());
  }

  if (session::HasValidationError(issues)) {
    if (validation_issues_out != nullptr) {
      *validation_issues_out = issues;
    }
    impl_->last_signal_abort_reason = session::SignalCycleAbortReason::kValidationRejected;
    // 中译：AR 运行期周期输入校验被拒绝（周期号）。
    // 标识：运行期路径（RunExecutionCycle 入口）校验失败——本周期不执行、输出为空，
    //       校验明细经出参直通；三写之三由本日志补齐（规则 9c）。
    PROJECT_LOG_WARN("AR validation rejected for cycle_index={}", stamp.cycle_index);
    return;
  }

  // 冻结环境
  session::EnvironmentCycleContext environment_cycle_context;
  environment_cycle_context.cycle_index = stamp.cycle_index;
  environment_cycle_context.dt_sec = cycle_dt_sec;
  impl_->environment_service.BeginCycle(environment_cycle_context);

  // 执行信号流水线与决策引擎
  if (!control_was_prepared) {
    impl_->ApplyPendingDecisionControl();
  }
  impl_->radar_context.UpdateRadarControlProfile(impl_->control_profile);
  impl_->signal_pipeline.SetControlProfile(impl_->control_profile);
  impl_->signal_pipeline.UpdatePlatformAttitude(platform_attitude);
  impl_->signal_pipeline.UpdatePlatformAltitudeM(platform_altitude_m);
  // 周期输入已通过 SignalCycleInput 显式传入，不再依赖 mutable 旁路状态。

  session::SignalCycleResult signal_result =
      impl_->signal_pipeline.RunCycle(cycle_input, impl_->environment_service);
  impl_->control_prepared_for_cycle = false;

  impl_->last_cycle_executed = signal_result.executed_this_cycle;
  impl_->last_signal_abort_reason = signal_result.abort_reason;

  if (!impl_->last_cycle_executed) {
    return;
  }
  // 规则 13b：正常周期按目标排除的 kInfo 诊断转写（abort 路径不变）。
  impl_->latest_issues = std::move(signal_result.issues);

  session::DecisionInputFrame decision_frame = signal_result.decision_frame;
  decision_frame.cycle_index = stamp.cycle_index;
  decision_frame.batch_id = stamp.batch_id;
  decision_frame.interference_observations = cycle_input.interference_observations;

  session::TrackOutputFrame output_frame;
  output_frame.cycle_index = stamp.cycle_index;
  output_frame.batch_id = stamp.batch_id;
  output_frame.tracks = decision_frame.tracks;

  session::TacticalDecisionResult decision_result;
  if (impl_->owned_decision_components.decision_engine != nullptr &&
      impl_->owned_decision_components.tactical_state_store != nullptr) {
    decision_result = impl_->owned_decision_components.decision_engine->Evaluate(
        decision_frame, *impl_->owned_decision_components.tactical_state_store);

    // 将目标分类结果回填到轨迹输出帧
    const auto& classifications = decision_result.target_classification_result;
    auto& output_tracks = output_frame.tracks;
    const std::size_t count = std::min(classifications.size(), output_tracks.size());
    for (std::size_t i = 0; i < count; ++i) {
      output_tracks[i].target_type = classifications[i].target_type;
      output_tracks[i].target_probability = classifications[i].probability;
    }
  }

  signal_result.decision_frame = decision_frame;

  impl_->has_pending_internal_decision = true;
  impl_->pending_internal_cycle_index = stamp.cycle_index;
  impl_->pending_internal_batch_id = stamp.batch_id;
  impl_->pending_internal_proposals = decision_result.proposals;

  impl_->cycle_state.latest_output = output_frame;
  impl_->cycle_state.has_latest_output = true;
  ++impl_->cycle_state.next_batch_id;
}

bool ArController::PrepareEmissionControl() {
  if (impl_->control_prepared_for_cycle) {
    return false;
  }
  impl_->last_applied_decision_source = session::DecisionControlSource::kNone;
  impl_->last_applied_decision_cycle_index = 0U;
  impl_->last_applied_decision_batch_id = 0U;
  impl_->last_applied_decision_proposals.clear();
  impl_->ApplyPendingDecisionControl();
  impl_->radar_context.UpdateRadarControlProfile(impl_->control_profile);
  impl_->signal_pipeline.SetControlProfile(impl_->control_profile);
  impl_->control_prepared_for_cycle = true;
  return true;
}

void ArController::ReleasePreparedEmissionControl() {
  impl_->control_prepared_for_cycle = false;
}

const session::ArControlProfile& ArController::GetControlProfile() const {
  return impl_->control_profile;
}

void ArController::RunCycles(std::size_t cycles) {
  for (std::size_t i = 0; i < cycles; ++i) {
    RunOnce(signal::pipeline::SignalCycleInput{impl_->radar_context.GetSceneTargets()});
  }
}

bool ArController::HasLatestTrackOutputFrame() const {
  return impl_->cycle_state.has_latest_output;
}

const session::TrackOutputFrame& ArController::GetLatestTrackOutputFrame() const {
  return impl_->cycle_state.latest_output;
}

const session::ArIssueList& ArController::GetLatestIssues() const {
  return impl_->latest_issues;
}

bool ArController::ExecutedLatestCycle() const { return impl_->last_cycle_executed; }

session::SignalCycleAbortReason ArController::GetLastSignalCycleAbortReason() const {
  return impl_->last_signal_abort_reason;
}

session::ExternalDecisionSubmitStatus ArController::SubmitExternalDecision(
    session::ExternalDecisionOverride override_decision) {
  // 15f 语义等价收窄：两标志历史上只在成功周期成对置位、观测袋无独立清除路径，
  // has_pending_internal_decision 单条件即原 has_pending && has_observation。
  if (!impl_->has_pending_internal_decision) {
    return session::ExternalDecisionSubmitStatus::kNoPendingObservation;
  }
  if (impl_->has_pending_external_override) {
    return session::ExternalDecisionSubmitStatus::kAlreadySubmitted;
  }
  if (!impl_->IsValidOverrideProfile(override_decision.profile)) {
    return session::ExternalDecisionSubmitStatus::kInvalidProfile;
  }
  impl_->pending_external_override = std::move(override_decision);
  impl_->has_pending_external_override = true;
  return session::ExternalDecisionSubmitStatus::kAccepted;
}

session::DecisionControlSource ArController::GetLastAppliedDecisionSource() const {
  return impl_->last_applied_decision_source;
}

std::uint32_t ArController::GetLastAppliedDecisionCycleIndex() const {
  return impl_->last_applied_decision_cycle_index;
}

std::uint64_t ArController::GetLastAppliedDecisionBatchId() const {
  return impl_->last_applied_decision_batch_id;
}

const std::vector<session::TacticalProposal>& ArController::GetLastAppliedDecisionProposals()
    const {
  return impl_->last_applied_decision_proposals;
}

extension::ArControllerRuntimeState ArController::CaptureRuntimeState() const {
  extension::ArControllerRuntimeState state;
  state.owner_identity = this;
  state.schema_version = 9U;
  state.latest_output = impl_->cycle_state.latest_output;
  state.has_latest_output = impl_->cycle_state.has_latest_output;
  state.next_batch_id = impl_->cycle_state.next_batch_id;
  state.last_cycle_executed = impl_->last_cycle_executed;
  state.last_signal_abort_reason = impl_->last_signal_abort_reason;
  state.control_profile = impl_->control_profile;
  state.control_reducer_config = impl_->owned_decision_components.control_reducer->GetConfig();
  state.control_reducer_state = impl_->owned_decision_components.control_reducer->GetRuntimeState();
  state.has_pending_internal_decision = impl_->has_pending_internal_decision;
  state.pending_internal_cycle_index = impl_->pending_internal_cycle_index;
  state.pending_internal_batch_id = impl_->pending_internal_batch_id;
  state.pending_internal_proposals = impl_->pending_internal_proposals;
  state.has_pending_external_override = impl_->has_pending_external_override;
  state.pending_external_override = impl_->pending_external_override;
  state.last_applied_decision_source = impl_->last_applied_decision_source;
  state.last_applied_decision_cycle_index = impl_->last_applied_decision_cycle_index;
  state.last_applied_decision_batch_id = impl_->last_applied_decision_batch_id;
  state.last_applied_decision_proposals = impl_->last_applied_decision_proposals;
  state.control_prepared_for_cycle = impl_->control_prepared_for_cycle;
  return state;
}

bool ArController::RestoreRuntimeState(const extension::ArControllerRuntimeState& state) {
  if (state.owner_identity != this || state.schema_version != 9U) {
    // 中译：控制器运行状态恢复被拒绝：所有者/结构不匹配。
    // 标识：回滚保护——归属/结构校验失败时拒绝恢复，防止错误状态写入。
    PROJECT_LOG_ERROR(
        "[ArController] controller runtime state restore rejected: "
        "owner/schema mismatch.");
    return false;
  }
  impl_->cycle_state.latest_output = state.latest_output;
  impl_->cycle_state.has_latest_output = state.has_latest_output;
  impl_->cycle_state.next_batch_id = state.next_batch_id;
  impl_->last_cycle_executed = state.last_cycle_executed;
  impl_->last_signal_abort_reason = state.last_signal_abort_reason;
  impl_->control_profile = state.control_profile;
  impl_->owned_decision_components.control_reducer->UpdateConfig(state.control_reducer_config);
  impl_->owned_decision_components.control_reducer->RestoreRuntimeState(
      state.control_reducer_state);
  impl_->has_pending_internal_decision = state.has_pending_internal_decision;
  impl_->pending_internal_cycle_index = state.pending_internal_cycle_index;
  impl_->pending_internal_batch_id = state.pending_internal_batch_id;
  impl_->pending_internal_proposals = state.pending_internal_proposals;
  impl_->has_pending_external_override = state.has_pending_external_override;
  impl_->pending_external_override = state.pending_external_override;
  impl_->last_applied_decision_source = state.last_applied_decision_source;
  impl_->last_applied_decision_cycle_index = state.last_applied_decision_cycle_index;
  impl_->last_applied_decision_batch_id = state.last_applied_decision_batch_id;
  impl_->last_applied_decision_proposals = state.last_applied_decision_proposals;
  impl_->control_prepared_for_cycle = state.control_prepared_for_cycle;
  return true;
}

}  // namespace extension
}  // namespace airborne_radar
