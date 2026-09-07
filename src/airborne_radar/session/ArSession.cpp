#include "1q/airborne_radar/session/ArSession.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "1q/airborne_radar/session/ArIssueCodes.h"
#include "1q/airborne_radar/session/ArRadarFrameTransform.h"
#include "1q/airborne_radar/session/ArExclusionCauseRecorder.h"
#include "1q/airborne_radar/session/ArTrackLifecycleRecorder.h"
#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "airborne_radar/config/mapping/RuntimePatchMapper.h"
#include "airborne_radar/session/ArDiagnosticUtils.h"
#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"
#include "airborne_radar/environment/IEnvironmentService.h"
#include "airborne_radar/runtime/ArController.h"
#include "airborne_radar/session/ArReplayCycleRecord.h"
#include "airborne_radar/session/ArRfCycleState.h"
#include "airborne_radar/session/ArSessionCompositionRoot.h"
#include "airborne_radar/session/MutableArContext.h"
#include "airborne_radar/session/ArEmissionFactory.h"
#include "airborne_radar/session/ArReceiverStateBuilder.h"
#include "airborne_radar/session/PreparedCycleLedger.h"
#include "airborne_radar/signal/detection/ArInterferenceObservationResolver.h"
#include "airborne_radar/signal/detection/ArRfFrontEndResolver.h"
#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "airborne_radar/signal/pipeline/ScanScheduleResolver.h"
#include "airborne_radar/signal/pipeline/SignalCycleInput.h"
#include "airborne_radar/signal/pipeline/SignalCycleInput.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"
#include "common/logging/ProjectLog.h"
#include "common/validation/ValidationUtils.h"
#include "common/numerics/Constants.h"

namespace airborne_radar {
namespace session {
namespace {

bool IsFinitePosition(const oneq::coordinate::EcefPositionM& value) {
  return std::isfinite(value.x_m) && std::isfinite(value.y_m) && std::isfinite(value.z_m);
}

bool IsFiniteVelocity(const oneq::coordinate::EcefVelocityMps& value) {
  return std::isfinite(value.x_mps) && std::isfinite(value.y_mps) && std::isfinite(value.z_mps);
}

bool SameEmissionIdentity(const oneq::electromagnetics::RfEmissionIdentity& left,
                          const oneq::electromagnetics::RfEmissionIdentity& right) {
  return left.sensor_entity_id == right.sensor_entity_id &&
         left.emission_id == right.emission_id;
}

bool SamePreparedEmission(const oneq::electromagnetics::RfSceneEmission& left,
                          const oneq::electromagnetics::RfSceneEmission& right) {
  const auto& left_waveform = left.waveform;
  const auto& right_waveform = right.waveform;
  return SameEmissionIdentity(left.identity, right.identity) &&
         left.position_ecef_m.x_m == right.position_ecef_m.x_m &&
         left.position_ecef_m.y_m == right.position_ecef_m.y_m &&
         left.position_ecef_m.z_m == right.position_ecef_m.z_m &&
         left.velocity_ecef_mps.x_mps == right.velocity_ecef_mps.x_mps &&
         left.velocity_ecef_mps.y_mps == right.velocity_ecef_mps.y_mps &&
         left.velocity_ecef_mps.z_mps == right.velocity_ecef_mps.z_mps &&
         left.antenna.boresight_ecef.x == right.antenna.boresight_ecef.x &&
         left.antenna.boresight_ecef.y == right.antenna.boresight_ecef.y &&
         left.antenna.boresight_ecef.z == right.antenna.boresight_ecef.z &&
         left.antenna.peak_gain_dbi == right.antenna.peak_gain_dbi &&
         left.antenna.half_power_beamwidth_deg == right.antenna.half_power_beamwidth_deg &&
         left.antenna.sidelobe_level_db == right.antenna.sidelobe_level_db &&
         left.antenna.backlobe_level_db == right.antenna.backlobe_level_db &&
         left.antenna.cross_polarization_isolation_db ==
             right.antenna.cross_polarization_isolation_db &&
         left.polarization == right.polarization && left_waveform.kind == right_waveform.kind &&
         left_waveform.activity_start_time_s == right_waveform.activity_start_time_s &&
         left_waveform.activity_duration_s == right_waveform.activity_duration_s &&
         left_waveform.center_frequency_hz == right_waveform.center_frequency_hz &&
         left_waveform.occupied_bandwidth_hz == right_waveform.occupied_bandwidth_hz &&
         left_waveform.transmit_power_w == right_waveform.transmit_power_w &&
         left_waveform.pulse_width_s == right_waveform.pulse_width_s &&
         left_waveform.pulse_repetition_interval_s == right_waveform.pulse_repetition_interval_s &&
         left_waveform.first_pulse_time_s == right_waveform.first_pulse_time_s &&
         left_waveform.pulse_count == right_waveform.pulse_count &&
         left_waveform.pulse_jitter_fraction == right_waveform.pulse_jitter_fraction &&
         left_waveform.timing_seed == right_waveform.timing_seed &&
         left_waveform.timing_epoch == right_waveform.timing_epoch;
}

struct ArExecutionCycleResult {
  bool executed{false};
  session::SignalCycleAbortReason abort_reason{session::SignalCycleAbortReason::kNone};
  TrackOutputFrame output_frame{};
  session::ArIssueList issues{}; /**< 正常执行周期按目标排除的 kInfo 诊断（规则 13b）；
                                      校验拒绝时承载校验明细（COMMON-OQ-9）。 */
};

/**
 * @brief 指定跟踪目标的周期派生状态（无跨周期记忆）。
 *
 * 由已提交 work_mode + 指定目标 ID + 最新航迹帧派生，供 prepare 指向解析与
 * completed 结果回填共用。派生规则（冻结，与 ArCycleResult::effective_work_mode
 * 注释一致）：
 *   - stt_active（航迹跟随生效）= 已提交 STT && 已指定 && 指定航迹 confirmed
 *     && 无显式 dwell 覆盖（显式 dwell 时指向按现状语义，不跟随航迹）；
 *   - reverted_to_tws（回退） = 已提交 STT && 已指定 && 指定航迹未确认/丢失；
 *   - 未指定目标的 STT 保持现状 scan_center 驻留语义（不视为回退）。
 */
struct SttDesignationCycleState {
  bool stt_requested{false};           /**< 已提交 work_mode == kStt。 */
  bool designation_set{false};         /**< 已配置指定目标（ID != 0）。 */
  std::uint64_t designated_target_id{0U}; /**< 指定目标外部 ID。 */
  bool has_track{false};               /**< 指定目标是否存在航迹。 */
  bool track_confirmed{false};         /**< 指定目标存在 confirmed 航迹。 */
  bool track_lost{false};              /**< 指定目标存在 lost 航迹。 */
  config::AzimuthElevationDeg track_pointing_deg{}; /**< confirmed 航迹位置换算的雷达局部指向。 */
  bool stt_active{false};              /**< 本周期 STT 航迹跟随驻留是否生效。 */
  bool reverted_to_tws{false};         /**< 本周期 STT 已请求但回退到 TWS。 */
};

SttDesignationCycleState BuildSttDesignationCycleState(
    config::ArWorkMode committed_work_mode, std::uint64_t designated_target_id,
    bool explicit_dwell_override, const session::TrackOutputFrame& latest_track_frame) {
  SttDesignationCycleState state;
  state.stt_requested = committed_work_mode == config::ArWorkMode::kStt;
  state.designation_set = designated_target_id != 0U;
  state.designated_target_id = designated_target_id;
  if (state.designation_set) {
    const session::TrackStateSnapshotList tracks =
        session::CollectTracksByExternalTargetId(latest_track_frame, designated_target_id);
    for (const session::TrackStateSnapshot& track : tracks) {
      state.has_track = true;
      state.track_lost = state.track_lost || track.status == session::TrackStatus::kLost;
      if (track.status == session::TrackStatus::kConfirmed && !state.track_confirmed) {
        // 多航迹同 ID 时取帧内第一条 confirmed（文档化边界，见 data-flow.md）。
        state.track_confirmed = true;
        config::AzimuthElevationDeg pointing;
        if (signal::pipeline::TryTrackPositionToLookAnglesDeg(
                track.position_x, track.position_y, track.position_z, &pointing)) {
          state.track_pointing_deg = pointing;
        }
      }
    }
  }
  // stt_active 排除显式 dwell 覆盖（此时指向按现状语义，不跟随航迹）；
  // reverted_to_tws 只看航迹状态（显式 dwell 覆盖不构成回退）。
  state.stt_active = state.stt_requested && state.designation_set && state.track_confirmed &&
                     !explicit_dwell_override;
  state.reverted_to_tws = state.stt_requested && state.designation_set && !state.track_confirmed;
  return state;
}

/**
 * @brief 指定指令生命周期推进结果（写回 RuntimeConfigState 的跨周期状态）。
 */
struct DesignationPhaseAdvance {
  config::mapping::DesignationPhase phase{config::mapping::DesignationPhase::kNone};
  std::uint32_t deadline_cycle_index{0U}; /**< 捕获窗口截止周期（0 = 无限期）。 */
  bool expired_edge{false}; /**< 本周期由 kPending 转移至 kExpired（作废沿）。 */
};

/**
 * @brief 推进指定指令生命周期（限时锁定；跨周期状态，见 RuntimeConfigState）。
 *
 * 状态机：kNone → kPending → kAcquired | kExpired（kAcquired/kExpired 为终态）：
 *   - 指令未消费（未指定 / work_mode 非 kStt）：阶段恒为 kNone，窗口不运行；
 *   - kNone（指令生效后首个处理周期）：开窗——deadline = cycle + duration
 *     （duration == 0 表示无限期，deadline 置 0）；
 *   - kPending：上一周期航迹帧 confirmed → kAcquired（此后不再受窗口约束，
 *     后续丢失按既有回退语义，不重新开窗口）；否则 cycle >= deadline 时 →
 *     kExpired（指令作废，回到扫描）；
 *   - kAcquired / kExpired：不再转移。
 * @note 捕获判定与指向同源，使用"上一周期航迹帧"口径（滞后一周期）。
 */
DesignationPhaseAdvance AdvanceDesignationPhase(
    config::mapping::DesignationPhase phase, std::uint32_t deadline_cycle_index,
    std::uint32_t designation_duration_cycles, std::uint32_t cycle_index,
    bool designation_consumed, bool track_confirmed) {
  DesignationPhaseAdvance advance;
  advance.phase = phase;
  advance.deadline_cycle_index = deadline_cycle_index;
  if (!designation_consumed) {
    advance.phase = config::mapping::DesignationPhase::kNone;
    advance.deadline_cycle_index = 0U;
    return advance;
  }
  if (phase == config::mapping::DesignationPhase::kNone) {
    advance.phase = config::mapping::DesignationPhase::kPending;
    if (designation_duration_cycles > 0U) {
      // 饱和加法：周期号接近 2³² 时不回绕为 0（0 = 无限期语义）。
      const std::uint64_t deadline =
          static_cast<std::uint64_t>(cycle_index) + designation_duration_cycles;
      advance.deadline_cycle_index = static_cast<std::uint32_t>(
          std::min<std::uint64_t>(deadline, std::numeric_limits<std::uint32_t>::max()));
    } else {
      advance.deadline_cycle_index = 0U;
    }
    return advance;
  }
  if (phase == config::mapping::DesignationPhase::kPending) {
    if (track_confirmed) {
      advance.phase = config::mapping::DesignationPhase::kAcquired;
    } else if (advance.deadline_cycle_index != 0U &&
               cycle_index >= advance.deadline_cycle_index) {
      advance.phase = config::mapping::DesignationPhase::kExpired;
      // 作废沿 = 转移发生的周期；阶段仅成功周期持久化，故沿恒为
      // "截止后首个成功周期"（失败/关机周期不消耗窗口，不吞掉超时报告）。
      advance.expired_edge = true;
    }
  }
  return advance;
}

/**
 * @brief 指定指令作废状态解析（由已推进的阶段与作废沿派生）。
 */
struct DesignationExpiryState {
  bool expired{false};      /**< 本周期指定指令已作废（kExpired 终态）。 */
  bool expiry_cycle{false}; /**< 本周期是作废沿（报告 kAcquisitionTimeout）。 */
};

DesignationExpiryState ResolveDesignationExpiry(
    config::mapping::DesignationPhase phase, bool expired_edge) {
  DesignationExpiryState expiry;
  expiry.expired = phase == config::mapping::DesignationPhase::kExpired;
  expiry.expiry_cycle = expired_edge;
  return expiry;
}

}  // namespace

struct ArSession::Impl {
  explicit Impl(ArSessionComposition composition)
      : runtime_state(),
        pipeline_config_synced(composition.pipeline_config_synced),
        owned_ar_context(std::move(composition.owned_ar_context)),
        owned_signal_pipeline(std::move(composition.owned_signal_pipeline)),
        owned_environment_service(std::move(composition.owned_environment_service)),
        owned_controller(std::move(composition.owned_controller)) {
    config::ArSessionConfig initial_session_config;
    initial_session_config.hardware = composition.runtime_hardware;
    initial_session_config.mission = composition.runtime_mission;
    initial_session_config.orientation = composition.runtime_orientation;
    initial_session_config.policy = composition.runtime_policy;
    initial_session_config.environment.scenario_config =
        composition.runtime_environment_scenario_config;
    runtime_state.execution_config = config::mapping::MapSessionToExecution(initial_session_config);
    runtime_state.environment_scenario_config = composition.runtime_environment_scenario_config;
    pending_runtime_state = runtime_state;

    concrete_signal_pipeline_ =
        static_cast<signal::pipeline::SignalPipeline*>(owned_signal_pipeline.get());
  }

  ArCycleResult BuildCompletedCycleResult(const ArCycleInput& input,
                                          const ArIssueList& issues,
                                          const ArPrepareCycleResult& prepared,
                                          const ArCompleteCycleResult& completed,
                                          config::mapping::DesignationPhase designation_phase,
                                          bool designation_expired_edge) const {
    ArCycleResult result;
    result.input_cycle_index = input.cycle_index;
    result.status = ArCycleStatus::kCompleted;
    result.output_frame = completed.output_frame;
    // 航迹归属对照表（信封通道，session_contract.md Attribution 挂载表）：与产品导出
    // 航迹逐条对应，键与真值透传自同一快照。产品帧上的 external_target_id/target_name
    // 为 deprecated 遗留（sim-only），权威关联路径是本表。
    result.track_attributions.reserve(completed.output_frame.tracks.size());
    for (const TrackStateSnapshot& track : completed.output_frame.tracks) {
      ArTrackAttributionRecord attribution;
      attribution.association_key = track.association_key;
      attribution.external_target_id = track.external_target_id;
      attribution.target_name = track.target_name;
      result.track_attributions.push_back(attribution);
    }
    // 统一问题列表（规则 14）：输入校验问题（phase=kInputValidation）在前，
    // 正常执行周期按目标排除的 kInfo 诊断（phase=kExecution，规则 13b）在后。
    result.issues = issues;
    result.issues.insert(result.issues.end(), completed.issues.begin(),
                         completed.issues.end());
    FillEmissionFrame(result, input, prepared);
    result.receiver_impairment = completed.receiver_impairment;
    result.interference_observations = completed.interference_observations;
    result.submitted_commands = RadarContext().GetSubmittedCommands();
    result.has_control_profile = RadarContext().HasLatestControlProfile();
    if (result.has_control_profile) {
      result.control_profile = RadarContext().GetLatestControlProfile();
    }
    result.association_quality_metrics = SignalPipeline().GetLastAssociationQualityMetrics();
    FillAppliedDecisionMetadata(result);
    // STT 指定航迹状态回填（派生规则见 BuildSttDesignationCycleState；指令
    // 生命周期阶段为会话级状态，由 RunCycle prepare 推进，见 AdvanceDesignationPhase）：
    // effective_work_mode 反映本周期生效模式（回退/作废时与已提交配置不同）；
    // designation_* 供外部经 L2 结果 / L3 视图 / 生命周期事件观测
    // "自动丢跟踪（回 TWS）"与"限时指令捕获超时（作废）"——
    // designation_reverted_to_tws 为每周期状态指示，跨周期差分由调用方承担；
    // 作废沿周期（cycle == deadline）保留目标 ID 并报告 kAcquisitionTimeout，
    // 其后指定清零、按扫描处理（回到扫描，见边界文档）。
    const config::ArWorkMode committed_work_mode =
        runtime_state.execution_config.detection.orientation.work_mode;
    const SttDesignationCycleState designation_state = BuildSttDesignationCycleState(
        committed_work_mode, runtime_state.designated_external_target_id,
        runtime_state.dwell_center_deg.az_deg != 0.0f ||
            runtime_state.dwell_center_deg.el_deg != 0.0f,
        completed.output_frame);
    const DesignationExpiryState designation_expiry =
        ResolveDesignationExpiry(designation_phase, designation_expired_edge);
    result.effective_work_mode =
        designation_expiry.expired
            ? (committed_work_mode == config::ArWorkMode::kStt ? config::ArWorkMode::kTws
                                                               : committed_work_mode)
            : (designation_state.reverted_to_tws ? config::ArWorkMode::kTws
                                                 : committed_work_mode);
    result.designated_target_id =
        designation_expiry.expired
            ? (designation_expiry.expiry_cycle ? designation_state.designated_target_id : 0U)
            : designation_state.designated_target_id;
    result.designation_active =
        designation_expiry.expired ? false : designation_state.stt_active;
    result.designation_reverted_to_tws =
        designation_expiry.expired ? designation_expiry.expiry_cycle
                                   : designation_state.reverted_to_tws;
    result.designation_revert_reason =
        designation_expiry.expired
            ? (designation_expiry.expiry_cycle
                   ? session::ArDesignationRevertReason::kAcquisitionTimeout
                   : session::ArDesignationRevertReason::kNone)
            : (designation_state.reverted_to_tws
                   ? (designation_state.track_lost
                          ? session::ArDesignationRevertReason::kTrackLost
                          : session::ArDesignationRevertReason::kTrackNotConfirmed)
                   : session::ArDesignationRevertReason::kNone);
    return result;
  }

  ArIssueList ValidateInput(const ArCycleInput& input) const {
    return ValidateArCycleInput(input);
  }

  ArCycleResult BuildValidationErrorResult(const ArCycleInput& input,
                                           const ArIssueList& issues) const {
    ArCycleResult result;
    result.input_cycle_index = input.cycle_index;
    result.status = ArCycleStatus::kRejectedInvalidInput;
    result.abort_reason = session::SignalCycleAbortReason::kValidationRejected;
    // 统一问题列表（规则 14）：校验问题本身就是 error 级诊断（phase=kInputValidation，
    // code 形如 "ar.validation.<snake>"）。校验拒绝路径不再附加粗粒度 abort 条目，
    // 避免 issues 列表中出现重复的 error 级主诊断。
    result.issues = issues;
    // 中译：AR 周期输入校验被拒绝（周期号）。
    // 标识：公共路径入口校验失败——本周期不执行、输出为空，abort_reason 为
    //       kValidationRejected；三写之三由本日志补齐（规则 9c），校验明细见 issues。
    PROJECT_LOG_WARN("AR validation rejected for cycle_index={}", input.cycle_index);
    return result;
  }

  ArCycleResult BuildExecutionAbortResult(const ArCycleInput& input,
                                          session::SignalCycleAbortReason abort_reason,
                                          ArCycleStatus status =
                                              ArCycleStatus::kRejectedExecution) const {
    ArCycleResult result;
    result.input_cycle_index = input.cycle_index;
    result.abort_reason = abort_reason;
    // 不可达兜底（值不属注册表；若命中会写入 issue.code）。
    const char* detail_code = "unknown";
    switch (abort_reason) {
      // 校验拒绝（kValidationRejected）不可经此路径：公共路径与发射后路径均走
      // BuildValidationErrorResult / BuildPostEmissionValidationErrorResult。
      case session::SignalCycleAbortReason::kSensorPoweredOff:
        detail_code = session::codes::kSensorPoweredOff;
        break;
      case session::SignalCycleAbortReason::kLifecycleUnavailable:
        detail_code = session::codes::kLifecycleUnavailable;
        break;
      case session::SignalCycleAbortReason::kInvalidEnvironmentCycle:
        detail_code = session::codes::kInvalidEnvironmentCycle;
        break;
      case session::SignalCycleAbortReason::kRuntimePreparationFailed:
        detail_code = session::codes::kRuntimePreparationFailed;
        break;
      default:
        break;
    }
    // 非校验中止路径才经 RecordAbort 写入粗粒度 abort 条目（phase=kExecution）；
    // 校验拒绝路径的 error 级诊断由校验问题本身承载（见 BuildValidationErrorResult）。
    RecordAbort(&result, abort_reason, detail_code, "AR cycle aborted.");
    // status 由调用点显式声明，在 RecordAbort 之后做最终赋值，
    // 避免"RecordAbort 覆盖 status、调用方再补丁"的双重覆盖链。
    result.status = status;
    return result;
  }

  // 用 prepare 期权威窗口与本周期 AR 发射填充 emission_frame，完成路径与发射后中止路径共用。
  void FillEmissionFrame(ArCycleResult& result, const ArCycleInput& input,
                         const ArPrepareCycleResult& prepared) const {
    result.emission_frame.world_cycle_index = input.cycle_index;
    result.emission_frame.window_start_time_s = input.cycle_start_time_s;
    result.emission_frame.window_duration_s = input.dt_sec;
    if (prepared.has_emission) {
      result.emission_frame.emissions.push_back(prepared.emission);
    }
  }

  // 复制本周期控制器实际采用的控制决策来源元组，完成路径与发射后中止路径共用。
  void FillAppliedDecisionMetadata(ArCycleResult& result) const {
    result.applied_decision_source = Controller().GetLastAppliedDecisionSource();
    result.applied_decision_cycle_index = Controller().GetLastAppliedDecisionCycleIndex();
    result.applied_decision_batch_id = Controller().GetLastAppliedDecisionBatchId();
  }

  ArCycleResult BuildPostEmissionAbortResult(const ArCycleInput& input,
                                             const ArPrepareCycleResult& prepared,
                                             session::SignalCycleAbortReason abort_reason) const {
    ArCycleResult result = BuildExecutionAbortResult(input, abort_reason);
    FillEmissionFrame(result, input, prepared);
    result.has_control_profile = true;
    result.control_profile = Controller().GetControlProfile();
    FillAppliedDecisionMetadata(result);
    return result;
  }

  // 发射后校验拒绝结果（COMMON-OQ-9）：与 BuildValidationErrorResult 同语义（校验问题
  // 本身即 error 级诊断，不附加粗粒度 abort 条目），但保留已发布发射的 emission_frame
  // 与决策元数据（发射后接收侧执行期校验拒绝路径）。
  ArCycleResult BuildPostEmissionValidationErrorResult(const ArCycleInput& input,
                                                       const ArPrepareCycleResult& prepared,
                                                       const ArIssueList& issues) const {
    ArCycleResult result;
    result.input_cycle_index = input.cycle_index;
    result.status = ArCycleStatus::kRejectedInvalidInput;
    result.abort_reason = session::SignalCycleAbortReason::kValidationRejected;
    result.issues = issues;
    FillEmissionFrame(result, input, prepared);
    result.has_control_profile = true;
    result.control_profile = Controller().GetControlProfile();
    FillAppliedDecisionMetadata(result);
    // 中译：AR 发射后接收侧校验被拒绝（周期号）。
    // 标识：发射后执行期校验失败——已发布发射保留在 emission_frame，abort_reason 为
    //       kValidationRejected；三写之三由本日志补齐（规则 9c），校验明细见 issues。
    PROJECT_LOG_WARN("AR validation rejected for cycle_index={}", input.cycle_index);
    return result;
  }

  /**
   * @brief 将已暂存的运行期配置提交到各子系统。
   *
   * 失败时调用方负责恢复已捕获的子系统快照。
   */
  bool CommitPendingRuntimeConfig() {
    const bool should_sync_pipeline =
        !pipeline_config_synced || (has_pending_runtime_update && pending_execution_config_changed);
    const bool should_sync_environment_model =
        has_pending_runtime_update && pending_environment_scenario_config_changed;
    if (!should_sync_pipeline && !should_sync_environment_model) {
      return true;
    }

    if (should_sync_pipeline) {
      const config::mapping::RuntimeConfigState& state_to_commit =
          has_pending_runtime_update ? pending_runtime_state : runtime_state;

      if (concrete_signal_pipeline_ != nullptr) {
        // 内部路径：直接传递 InternalExecutionConfig，避免经过公开类型的 round-trip 信息损失
        config::execution::InternalExecutionConfig exec_config = state_to_commit.execution_config;
        exec_config.detection.orientation.scan_center_deg.az_deg +=
            state_to_commit.dwell_center_deg.az_deg;
        exec_config.detection.orientation.scan_center_deg.el_deg +=
            state_to_commit.dwell_center_deg.el_deg;
        if (!concrete_signal_pipeline_->UpdateExecutionConfig(exec_config)) {
          return false;
        }
      } else {
        // 外部路径：通过公开接口合约传递，由外部 pipeline 自行管理内部配置
        const config::ArSessionConfig pipeline_config =
            config::mapping::MapRuntimeStateToPipelineSession(state_to_commit);
        if (!SignalPipeline().UpdateConfig(pipeline_config)) {
          return false;
        }
      }
      Controller().UpdateDecisionControlConfig(state_to_commit.execution_config.decision_control);
      pipeline_config_synced = true;
    }

    if (should_sync_environment_model) {
      EnvironmentService().UpdateModelConfig(pending_runtime_state.environment_scenario_config);
    }
    return true;
  }

  void FinalizePendingRuntimeConfig() {
    if (!has_pending_runtime_update) {
      return;
    }
    runtime_state = pending_runtime_state;
    has_pending_runtime_update = false;
    pending_execution_config_changed = false;
    pending_environment_scenario_config_changed = false;
  }

  bool RestoreCycleRuntimeState(
      const ArContextRuntimeState& radar_context_state,
      const signal::SignalPipelineRuntimeState& pipeline_state,
      const environment::EnvironmentServiceRuntimeState& environment_state,
      const extension::ArControllerRuntimeState& controller_state) {
    const bool radar_context_restored = RadarContext().RestoreRuntimeState(radar_context_state);
    SignalPipeline().RestoreRuntimeState(pipeline_state);
    EnvironmentService().RestoreRuntimeState(environment_state);
    const bool controller_restored = Controller().RestoreRuntimeState(controller_state);
    const bool runtime_state_restored = radar_context_restored && controller_restored;
    if (!runtime_state_restored) {
      // 中译：周期运行状态恢复被上下文或控制器拒绝。
      // 标识：回滚保护——发射后接收侧失败需恢复状态时，若恢复失败则
      //       本周期作废，防止状态不一致跨周期延续。
      PROJECT_LOG_ERROR(
          "[ArSession] cycle runtime state restore rejected by context or controller.");
    }
    return runtime_state_restored;
  }

  ArExecutionCycleResult RunExecutionCycle(
      std::uint32_t cycle_index, float dt_sec, float platform_altitude_m,
      signal::pipeline::SignalCycleInput cycle_input,
      bool commit_pending_runtime_config) {
    ArExecutionCycleResult result;
    // COMMON-OQ-9：周期输入校验唯一化在 ArController::RunOnce（同一输入的会话层
    // 二次校验已删除）；运行期拒绝的校验明细经 RunOnce 出参直通，不再丢失。
    ArIssueList validation_issues;

    const ArContextRuntimeState radar_context_state = RadarContext().CaptureRuntimeState();
    const signal::SignalPipelineRuntimeState pipeline_state =
        SignalPipeline().CaptureRuntimeState();
    const environment::EnvironmentServiceRuntimeState environment_state =
        EnvironmentService().CaptureRuntimeState();
    const extension::ArControllerRuntimeState controller_state = Controller().CaptureRuntimeState();

    if (commit_pending_runtime_config && !CommitPendingRuntimeConfig()) {
      (void)RestoreCycleRuntimeState(radar_context_state, pipeline_state, environment_state,
                                     controller_state);
      result.abort_reason = session::SignalCycleAbortReason::kRuntimePreparationFailed;
      return result;
    }
    RadarContext().BeginCycle(cycle_input.scene_targets,
                              platform_altitude_m, dt_sec, cycle_index);
    Controller().RunOnce(cycle_input, &validation_issues);

    if (!Controller().ExecutedLatestCycle()) {
      result.abort_reason = Controller().GetLastSignalCycleAbortReason();
      if (result.abort_reason == session::SignalCycleAbortReason::kValidationRejected) {
        // 统一问题列表（规则 14）：校验问题本身就是 error 级诊断（phase=kInputValidation），
        // 明细随结果透传，由上层 CompleteRfCycle 装配进最终周期结果。
        result.issues = validation_issues;
      }
      if (!RestoreCycleRuntimeState(radar_context_state, pipeline_state, environment_state,
                                    controller_state)) {
        result.abort_reason = session::SignalCycleAbortReason::kRuntimePreparationFailed;
        return result;
      }
      if (commit_pending_runtime_config &&
          result.abort_reason == session::SignalCycleAbortReason::kSensorPoweredOff) {
        if (!CommitPendingRuntimeConfig()) {
          (void)RestoreCycleRuntimeState(radar_context_state, pipeline_state, environment_state,
                                         controller_state);
          result.abort_reason = session::SignalCycleAbortReason::kRuntimePreparationFailed;
          return result;
        }
        FinalizePendingRuntimeConfig();
      }
      return result;
    }

    if (commit_pending_runtime_config) {
      FinalizePendingRuntimeConfig();
    }
    result.executed = true;
    result.output_frame = Controller().GetLatestTrackOutputFrame();
    // 规则 13b：正常执行周期按目标排除的 kInfo 诊断转写（abort 路径不变）。
    result.issues = Controller().GetLatestIssues();
    return result;
  }

  ArCycleResult RunCycle(const ArCycleInput& input) {
    const ArIssueList issues = ValidateInput(input);
    if (HasValidationError(issues)) {
      return BuildValidationErrorResult(input, issues);
    }

    oneq::coordinate::LocalFrameReference reference;
    oneq::coordinate::Vector3d radar_local_velocity;
    const config::mapping::RuntimeConfigState& next_operating_state =
        has_pending_runtime_update ? pending_runtime_state : runtime_state;
    const config::ArEffectiveOrientationConfig& orientation_config =
        next_operating_state.execution_config.detection.orientation;
    const oneq::coordinate::EulerAnglesDeg mount_angles_coord{
        orientation_config.mount_angles_deg.yaw_deg,
        orientation_config.mount_angles_deg.pitch_deg,
        orientation_config.mount_angles_deg.roll_deg};
    // 复跑（真实 mount 角）运动学转换是首轮校验（零 mount 角）未覆盖的路径：
    // 失败时必须补 error 级 issue，否则校验拒绝的三写中 issues 一写为空
    //（status=kRejectedInvalidInput 却无任何 error 条目）。
    const auto make_mount_issue = [&input](const char* code,
                                           oneq::foundation::ValidationLocationKind location_kind,
                                           std::size_t entity_index, const std::string& field,
                                           const std::string& message) {
      ArIssue issue = oneq::common::validation::MakeLocatedIssue<
          ArIssue, oneq::foundation::ValidationLocation>(
          ArIssueSeverity::kError, code, location_kind, entity_index, field, message);
      issue.phase = ArIssuePhase::kInputValidation;
      return issue;
    };
    if (!TryMakeArPoseFromPlatform(input.platform, mount_angles_coord,
                                  &reference, &radar_local_velocity)) {
      ArIssueList mount_issues = issues;
      mount_issues.push_back(make_mount_issue(
          codes::kInvalidPlatformInput,
          oneq::foundation::ValidationLocationKind::kPlatform,
          static_cast<std::size_t>(-1), "mount_angles_deg",
          "platform pose conversion failed with configured mount angles"));
      return BuildValidationErrorResult(input, mount_issues);
    }
    ArSceneTargetList local_targets;
    for (std::size_t target_index = 0U; target_index < input.targets.size(); ++target_index) {
      const ArTargetInput& target = input.targets[target_index];
      ArSceneTarget local_target;
      if (!TryMakeArTargetFromEnu(target, reference, radar_local_velocity, &local_target)) {
        ArIssueList mount_issues = issues;
        mount_issues.push_back(make_mount_issue(
            codes::kInvalidTargetInput,
            oneq::foundation::ValidationLocationKind::kSceneEntity, target_index,
            "targets",
            "target ENU scene conversion failed with configured mount angles"));
        return BuildValidationErrorResult(input, mount_issues);
      }
      local_targets.push_back(local_target);
    }

    const ArContextRuntimeState radar_context_state = RadarContext().CaptureRuntimeState();
    const signal::SignalPipelineRuntimeState pipeline_state =
        SignalPipeline().CaptureRuntimeState();
    const environment::EnvironmentServiceRuntimeState environment_state =
        EnvironmentService().CaptureRuntimeState();
    const extension::ArControllerRuntimeState controller_state = Controller().CaptureRuntimeState();
    const config::mapping::RuntimeConfigState saved_runtime_state = runtime_state;
    const config::mapping::RuntimeConfigState saved_pending_runtime_state = pending_runtime_state;
    const bool saved_has_pending_runtime_update = has_pending_runtime_update;
    const bool saved_pending_execution_config_changed = pending_execution_config_changed;
    const bool saved_pending_environment_scenario_config_changed =
        pending_environment_scenario_config_changed;
    const bool saved_pipeline_config_synced = pipeline_config_synced;
    // prepared-cycle 书记（编年史、令牌、计数器、冻结输入/发射/接收状态）整体快照——
    // PrepareRfCycle 失败时单次赋值即可逐字段回滚。
    const PreparedCycleLedger saved_prepared_ledger = prepared_ledger_;

    const auto restore_user_cycle = [&]() {
      const bool restored = RestoreCycleRuntimeState(radar_context_state, pipeline_state,
                                                     environment_state, controller_state);
      runtime_state = saved_runtime_state;
      pending_runtime_state = saved_pending_runtime_state;
      has_pending_runtime_update = saved_has_pending_runtime_update;
      pending_execution_config_changed = saved_pending_execution_config_changed;
      pending_environment_scenario_config_changed =
          saved_pending_environment_scenario_config_changed;
      pipeline_config_synced = saved_pipeline_config_synced;
      prepared_ledger_ = saved_prepared_ledger;
      return restored;
    };

    ArPrepareCycleInput prepare_input;
    prepare_input.world_cycle_index = input.cycle_index;
    prepare_input.window_start_time_s = input.cycle_start_time_s;
    prepare_input.window_duration_s = input.dt_sec;
    prepare_input.sensor_entity_id =
        (sensor_entity_id != 0U) ? sensor_entity_id : 1U;
    prepare_input.platform_position_ecef_m = input.platform.platform_position_ecef_m;
    prepare_input.platform_velocity_ecef_mps = input.platform.platform_velocity_mps;
    prepare_input.radar_frame_attitude_deg = ComposeRadarAttitudeDeg(
        input.platform.platform_attitude_deg, mount_angles_coord);
    config::PlatformAttitudeDeg platform_attitude;
    platform_attitude.yaw_deg = input.platform.platform_attitude_deg.yaw_deg;
    platform_attitude.pitch_deg = input.platform.platform_attitude_deg.pitch_deg;
    platform_attitude.roll_deg = input.platform.platform_attitude_deg.roll_deg;
    // STT 指定航迹跟随（方案 A）：指向来源优先级（冻结，见
    // ResolveSttTrackFollowingPointing 与 ArRuntimeConfigPatch 注释）——
    //   1) 显式 dwell_center_deg 非零：scan_center + dwell（现状语义，最高优先）；
    //   2) STT + 指定目标 confirmed：指向 = 指定航迹位置换算的 az/el（雷达局部系，
    //      外部无需提供目标角度）；
    //   3) 其余（未指定/航迹未确认/丢失）：scan_center（现状行为）。
    // 生效模式派生（无跨周期记忆）：已提交 STT 且指定航迹未确认时回退 TWS；
    // 本周期指向仍按回退分支（scan_center + dwell）解析，与 session 级 TWS 一致。
    const bool explicit_dwell_override =
        next_operating_state.dwell_center_deg.az_deg != 0.0f ||
        next_operating_state.dwell_center_deg.el_deg != 0.0f;
    const SttDesignationCycleState designation_state = BuildSttDesignationCycleState(
        orientation_config.work_mode, next_operating_state.designated_external_target_id,
        explicit_dwell_override, Controller().GetLatestTrackOutputFrame());
    // 指定指令生命周期推进（限时锁定，跨周期状态）：窗口 [start, start+N-1] 内
    // 捕获 confirmed 航迹 → kAcquired（此后不再受窗口约束，丢失按既有回退语义）；
    // 窗口耗尽仍未捕获 → kExpired（指令作废，回到扫描，终态直到外部重新指定）。
    // 推进结果先留在局部，仅在本周期成功完成后落定（见成功路径的写回）——
    // 失败/关机周期不消耗窗口（与 boundaries.md 的"失败周期由事务快照回滚"一致）。
    const config::mapping::RuntimeConfigState& designation_owner =
        has_pending_runtime_update ? pending_runtime_state : runtime_state;
    const DesignationPhaseAdvance phase_advance = AdvanceDesignationPhase(
        designation_owner.designation_phase, designation_owner.designation_deadline_cycle_index,
        designation_owner.designation_duration_cycles, input.cycle_index,
        orientation_config.work_mode == config::ArWorkMode::kStt &&
            designation_owner.designated_external_target_id != 0U,
        designation_state.track_confirmed);
    const DesignationExpiryState designation_expiry = ResolveDesignationExpiry(
        phase_advance.phase, phase_advance.expired_edge);
    // 指令作废后不再消费指定（指向回退分支按扫描推进，不跟随任何目标）。
    const bool designation_consumed =
        designation_state.designation_set && !designation_expiry.expired;
    const signal::pipeline::SttTrackFollowingResolution stt_pointing =
        signal::pipeline::ResolveSttTrackFollowingPointing(
            orientation_config, next_operating_state.dwell_center_deg,
            designation_consumed, designation_state.track_confirmed,
            designation_state.track_pointing_deg);
    // 扫描动画（boundaries.md 已知限制修复）：生效模式为 TWS/TAS 且无显式
    // dwell 覆盖、无航迹跟随时，session 级指向按扫描表逐周期推进，使发射
    // boresight 与 RfV2DetectionContext::beam_pointing_deg 随周期移动；
    // 显式 dwell（优先级 1）与 STT 航迹跟随（优先级 2）保持静态语义，
    // 未指定目标的 STT 驻留（生效模式仍为 kStt）同样保持 scan_center 静态；
    // 指令作废（kExpired）后生效模式按扫描处理（回到扫描）。
    const config::ArWorkMode effective_work_mode =
        designation_expiry.expired
            ? (orientation_config.work_mode == config::ArWorkMode::kStt
                   ? config::ArWorkMode::kTws
                   : orientation_config.work_mode)
            : (designation_state.reverted_to_tws ? config::ArWorkMode::kTws
                                                 : orientation_config.work_mode);
    config::ArEffectiveOrientationConfig effective_orientation = orientation_config;
    effective_orientation.scan_center_deg = stt_pointing.scan_center_deg;
    if (!explicit_dwell_override && !stt_pointing.track_following_active &&
        (effective_work_mode == config::ArWorkMode::kTws ||
         effective_work_mode == config::ArWorkMode::kTas)) {
      effective_orientation.work_mode = effective_work_mode;
      // 生效模式覆盖：已提交配置可能为 kStt（回退/指令作废后按扫描处理），
      // resolver 须按生效模式选波位语义（STT 返回静态扫描中心）。
      effective_orientation.scan_center_deg =
          signal::pipeline::ResolveScheduledBeamPointingFromExecutionConfig(
              next_operating_state.execution_config, input.cycle_index, effective_work_mode);
    }
    prepare_input.beam_pointing_deg =
        signal::detection::BeamControlResolver::ResolveMountFrameBeamPointing(
            effective_orientation, platform_attitude, stt_pointing.dwell_center_deg);
    if (stt_pointing.track_following_active) {
      // 中译：STT 波束已跟随指定航迹（目标外部 ID），本周期指向由该航迹位置换算。
      // 标识：指向来源事件——指定航迹 confirmed 且无显式驻留偏移时自动跟随；
      //       仅人读诊断，不用于状态判断。
      PROJECT_LOG_INFO(
          "[ArSession] STT beam follows designated track (external_target_id={}, "
          "az_deg={}, el_deg={}).",
          designation_state.designated_target_id, stt_pointing.scan_center_deg.az_deg,
          stt_pointing.scan_center_deg.el_deg);
    }

    const ArPrepareCycleResult prepared = PrepareRfCycle(prepare_input);
    if (prepared.status == ArPrepareCycleStatus::kPoweredOff) {
      return BuildExecutionAbortResult(input, session::SignalCycleAbortReason::kSensorPoweredOff,
                                       ArCycleStatus::kPoweredOff);
    }
    if (prepared.status != ArPrepareCycleStatus::kPrepared) {
      (void)restore_user_cycle();
      return BuildExecutionAbortResult(
          input, session::SignalCycleAbortReason::kRuntimePreparationFailed,
          ArCycleStatus::kRejectedInvalidConfig);
    }

    // The complete-phase RF scene is authored from the prepared token's
    // authoritative cycle window, not from input.interference: FrameMatchesCycle()
    // has already guaranteed that any non-empty interference frame matches this
    // window, while an empty (defaulted) interference frame carries zero window
    // fields that must be populated here so the AR emission and any external
    // emissions share one validated window before the front-end link check.
    ArCompleteCycleInput complete_input;
    complete_input.rf_scene = input.interference;
    complete_input.rf_scene.world_cycle_index = input.cycle_index;
    complete_input.rf_scene.window_start_time_s = input.cycle_start_time_s;
    complete_input.rf_scene.window_duration_s = input.dt_sec;
    complete_input.rf_scene.emissions.push_back(prepared.emission);
    complete_input.targets = local_targets;
    const ArContextRuntimeState post_emission_radar_context_state =
        RadarContext().CaptureRuntimeState();
    const signal::SignalPipelineRuntimeState post_emission_pipeline_state =
        SignalPipeline().CaptureRuntimeState();
    const environment::EnvironmentServiceRuntimeState post_emission_environment_state =
        EnvironmentService().CaptureRuntimeState();
    const extension::ArControllerRuntimeState post_emission_controller_state =
        Controller().CaptureRuntimeState();
    const ArCompleteCycleResult completed = CompleteRfCycle(prepared.token, complete_input);
    if (completed.status != ArCompleteCycleStatus::kCompleted) {
      const bool receive_state_restored =
          RestoreCycleRuntimeState(post_emission_radar_context_state, post_emission_pipeline_state,
                                   post_emission_environment_state, post_emission_controller_state);
      const bool emission_finalized =
          AbandonRfCycle(prepared.token) == ArAbandonCycleStatus::kAbandoned;
      if (!receive_state_restored || !emission_finalized) {
        // 中译：发射后的接收侧拒绝未能完成收尾（状态恢复或发射放弃失败）。
        // 标识：事务性提交的失败收尾——发射事实已提交但接收侧回滚不完整，
        //       属内部异常路径，需排查状态恢复与发射账本一致性。
        PROJECT_LOG_ERROR("[ArSession] failed to finalize post-emission receive rejection.");
      }
      // COMMON-OQ-9：按真实 abort_reason 装配——校验拒绝带明细且不再被写死替换
      // （kRuntimePreparationFailed 仅保留给无执行侧原因的接收拒绝）。
      if (completed.abort_reason == session::SignalCycleAbortReason::kValidationRejected) {
        return BuildPostEmissionValidationErrorResult(input, prepared, completed.issues);
      }
      return BuildPostEmissionAbortResult(
          input, prepared,
          completed.abort_reason == session::SignalCycleAbortReason::kNone
              ? session::SignalCycleAbortReason::kRuntimePreparationFailed
              : completed.abort_reason);
    }
    // 成功完成才落定指定指令生命周期阶段（失败/关机周期不消耗窗口；
    // pending 已在 PrepareRfCycle 中 finalize，runtime_state 即生效状态）。
    runtime_state.designation_phase = phase_advance.phase;
    runtime_state.designation_deadline_cycle_index = phase_advance.deadline_cycle_index;
    return BuildCompletedCycleResult(input, issues, prepared, completed, phase_advance.phase,
                                     phase_advance.expired_edge);
  }

  bool TokenMatches(const ArPreparedCycleToken& token) const {
    return prepared_ledger_.TokenMatches(token);
  }

  ArPrepareCycleResult PrepareRfCycle(const ArPrepareCycleInput& input) {
    ArPrepareCycleResult result;
    if (prepared_ledger_.has_prepared_cycle()) {
      result.status = ArPrepareCycleStatus::kBusy;
      return result;
    }
    if (input.sensor_entity_id == 0U || input.world_cycle_index == 0U ||
        !std::isfinite(input.window_start_time_s) || !std::isfinite(input.window_duration_s) ||
        input.window_duration_s <= 0.0 ||
        input.world_cycle_index > std::numeric_limits<std::uint32_t>::max() ||
        !IsFinitePosition(input.platform_position_ecef_m) ||
        !IsFiniteVelocity(input.platform_velocity_ecef_mps) ||
        (prepared_ledger_.has_world_chronology() &&
         input.window_start_time_s < prepared_ledger_.last_world_window_end_s())) {
      result.status = ArPrepareCycleStatus::kRejected;
      return result;
    }
    if (!CommitPendingRuntimeConfig()) {
      result.status = ArPrepareCycleStatus::kRejected;
      return result;
    }
    FinalizePendingRuntimeConfig();
    if (!runtime_state.execution_config.sensor_enabled) {
      prepared_ledger_.AdvanceWorldChronology(input.window_start_time_s, input.window_duration_s);
      result.status = ArPrepareCycleStatus::kPoweredOff;
      return result;
    }

    const config::engineering::DetectionConfig& detection =
        runtime_state.execution_config.detection.engineering;
    const config::engineering::TransmitterConfig& transmitter = detection.transmitter;
    if (transmitter.frequency_plan_hz.empty() || transmitter.prf_hz <= 0.0f) {
      result.status = ArPrepareCycleStatus::kRejected;
      return result;
    }
    const double pulse_repetition_interval_s = 1.0 / static_cast<double>(transmitter.prf_hz);
    const double pulse_count_value =
        std::ceil(input.window_duration_s / pulse_repetition_interval_s);
    if (!std::isfinite(pulse_count_value) || pulse_count_value < 1.0 ||
        pulse_count_value > static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
      result.status = ArPrepareCycleStatus::kRejected;
      return result;
    }
    const extension::ArControllerRuntimeState controller_state_before_prepare =
        Controller().CaptureRuntimeState();
    if (!Controller().PrepareEmissionControl()) {
      result.status = ArPrepareCycleStatus::kRejected;
      return result;
    }
    const session::ArControlProfile& control_profile = Controller().GetControlProfile();
    std::size_t selected_frequency_hop_index = prepared_ledger_.frequency_hop_index();
    if (control_profile.enable_agility_frequency && transmitter.frequency_plan_hz.size() > 1U) {
      selected_frequency_hop_index =
          (prepared_ledger_.frequency_hop_index() + 1U) % transmitter.frequency_plan_hz.size();
    }
    const double carrier_hz = transmitter.frequency_plan_hz[selected_frequency_hop_index];

    oneq::electromagnetics::RfSceneEmission emission;
    if (!ArEmissionFactory::TryBuildEmission(
            input, detection, control_profile, prepared_ledger_.next_emission_id(), carrier_hz,
            pulse_repetition_interval_s, static_cast<std::uint32_t>(pulse_count_value), timing_seed,
            prepared_ledger_.successful_prepare_count(), &emission)) {
      (void)Controller().RestoreRuntimeState(controller_state_before_prepare);
      result.status = ArPrepareCycleStatus::kRejected;
      return result;
    }

    const ArReceiverOperatingState operating_state =
        ArReceiverStateBuilder::Build(input, emission, detection, control_profile, carrier_hz);

    prepared_ledger_.CommitPrepared(input, emission, operating_state, selected_frequency_hop_index);

    result.status = ArPrepareCycleStatus::kPrepared;
    result.token = prepared_ledger_.prepared_token();
    result.has_emission = true;
    result.emission = prepared_ledger_.prepared_emission();
    result.operating_state = prepared_ledger_.prepared_operating_state();
    return result;
  }

  // 非饱和路径的干扰/欺骗观测解析：构造雷达局部坐标系、热噪声基底、去真值化扰动种子，
  // 调用 resolver 填充 interference_observations 与 deception_candidates。失败返回 false
  // （调用方置 kRejected）；饱和路径由调用方跳过本方法、保留空观测。
  bool TryResolveReceiveObservations(
      const oneq::electromagnetics::RfSceneFrame& rf_scene,
      const ArPreparedCycleToken& prepared_token, const ArPrepareCycleInput& prepared_input,
      const oneq::electromagnetics::RfSceneEmission& prepared_emission,
      const ArReceiverOperatingState& prepared_operating_state,
      const signal::detection::ArRfFrontEndResult& front_end,
      std::vector<ArInterferenceObservation>* interference_observations,
      signal::detection::ArDeceptionMeasurementCandidateList* deception_candidates) {
    using oneq::common::numerics::kBoltzmann;
    constexpr double kReferenceTemperatureK = 290.0;
    const config::engineering::DetectionConfig& detection =
        runtime_state.execution_config.detection.engineering;
    const double thermal_noise_power_w =
        kBoltzmann * kReferenceTemperatureK *
        static_cast<double>(detection.transmitter.bandwidth_hz) *
        std::pow(10.0, static_cast<double>(detection.receiver.noise_figure_db) / 10.0);
    // 为干扰观测构造雷达局部坐标系：原点 LLA + 合成姿态（平台姿态+挂架角），
    // 使解析器能把 ECEF 视线转换到与目标 look angle 同系的局部方位。
    oneq::coordinate::LocalFrameReference platform_frame;
    bool platform_frame_resolved = false;
    oneq::coordinate::LlaPositionDegM frame_origin_lla;
    if (oneq::coordinate::TryEcefToLla(prepared_input.platform_position_ecef_m, &frame_origin_lla)) {
      platform_frame.origin_lla = frame_origin_lla;
      platform_frame.frame_attitude_deg = prepared_input.radar_frame_attitude_deg;
      platform_frame_resolved = true;
    }
    if (!platform_frame_resolved) {
      // 中译：无法构建平台坐标框架，干扰方位回退到 ECEF 切平面
      //       （跨帧鉴别能力降级）。
      // 标识：坐标解算失败——平台位置转 LLA 失败时使用回退参考系，
      //       干扰观测精度降级但周期继续执行。
      PROJECT_LOG_WARN(
          "[ArSession] CompleteRfCycle could not build platform frame; interference "
          "bearings fall back to ECEF tangent-plane (cross-frame discrimination degraded).");
    }
    // 去真值化扰动种子：cycle index + receiver sensor entity id 派生，保证 replay（同 cycle
    // 重放）下扰动可复现，同时跨周期/跨设备互不相关（contract.md:348）。
    const std::uint32_t perturbation_seed =
        (prepared_token.world_cycle_index & 0xFFFF'FFFFU) ^
        (prepared_operating_state.rf_receiver.sensor_entity_id * 2654435761U);
    if (!signal::detection::TryResolveArInterferenceObservations(
            rf_scene, prepared_operating_state.rf_receiver, prepared_emission.identity,
            front_end.incident_links, thermal_noise_power_w,
            static_cast<double>(detection.receiver.interference_observation_jn_gate_db),
            platform_frame, perturbation_seed, interference_observations,
            deception_candidates)) {
      return false;
    }
    return true;
  }

  ArCompleteCycleResult CompleteRfCycle(const ArPreparedCycleToken& token,
                                        const ArCompleteCycleInput& input) {
    ArCompleteCycleResult result;
    if (!TokenMatches(token)) {
      result.status = ArCompleteCycleStatus::kTokenMismatch;
      return result;
    }
    // 账本冻结的 prepared-cycle 状态——本方法全程只读消费这些值。
    const ArPreparedCycleToken& prepared_token = prepared_ledger_.prepared_token();
    const ArPrepareCycleInput& prepared_input = prepared_ledger_.prepared_input();
    const oneq::electromagnetics::RfSceneEmission& prepared_emission =
        prepared_ledger_.prepared_emission();
    const ArReceiverOperatingState& prepared_operating_state =
        prepared_ledger_.prepared_operating_state();
    result.world_cycle_index = prepared_token.world_cycle_index;
    if (input.rf_scene.world_cycle_index != prepared_token.world_cycle_index ||
        input.rf_scene.window_start_time_s != prepared_input.window_start_time_s ||
        input.rf_scene.window_duration_s != prepared_input.window_duration_s ||
        !oneq::electromagnetics::TryValidateRfSceneFrame(input.rf_scene)) {
      // 中译：CompleteRfCycle 拒绝了非法或不匹配的 RF 场景。
      // 标识：RF 场景一致性门——周期窗口/世界周期号与预备阶段不符时
      //       整周期拒绝，防止错帧执行。
      PROJECT_LOG_ERROR("[ArSession] CompleteRfCycle rejected an invalid or mismatched RF scene.");
      result.status = ArCompleteCycleStatus::kRejected;
      return result;
    }
    bool found_prepared_emission = false;
    for (const auto& emission : input.rf_scene.emissions) {
      if (SameEmissionIdentity(emission.identity, prepared_emission.identity)) {
        if (!SamePreparedEmission(emission, prepared_emission)) {
          // 中译：CompleteRfCycle 发现预备发射被修改过。
          // 标识：发射一致性门——提交阶段的发射与预备阶段不一致时拒绝，
          //       防止篡改后的发射参数被执行。
          PROJECT_LOG_ERROR("[ArSession] CompleteRfCycle found a modified prepared emission.");
          result.status = ArCompleteCycleStatus::kRejected;
          return result;
        }
        found_prepared_emission = true;
      }
    }
    if (!found_prepared_emission) {
      // 中译：CompleteRfCycle 未找到预备的发射。
      // 标识：发射一致性门——提交场景中缺少预备阶段的发射时拒绝，
      //       防止无发射事实的周期继续。
      PROJECT_LOG_ERROR("[ArSession] CompleteRfCycle did not find the prepared emission.");
      result.status = ArCompleteCycleStatus::kRejected;
      return result;
    }

    signal::detection::ArRfFrontEndResult front_end;
    if (!signal::detection::TryResolveArRfFrontEnd(
            input.rf_scene, prepared_operating_state.rf_receiver,
            prepared_operating_state.maximum_linear_input_power_w,
            oneq::electromagnetics::RfIncidentLinkConfig{}, &front_end)) {
      // 中译：CompleteRfCycle RF 前端解析失败。
      // 标识：接收链路解析失败——前端饱和/链路预算异常时整周期拒绝。
      PROJECT_LOG_ERROR("[ArSession] CompleteRfCycle RF front-end resolution failed.");
      result.status = ArCompleteCycleStatus::kRejected;
      return result;
    }
    result.receiver_impairment = front_end.receiver_saturated ? ArReceiverImpairment::kSaturated
                                                              : ArReceiverImpairment::kNone;
    signal::pipeline::RfV2DetectionContext rf_v2_detection_context;
    rf_v2_detection_context.own_emission_identity = prepared_emission.identity;
    rf_v2_detection_context.own_transmit_waveform = prepared_emission.waveform;
    rf_v2_detection_context.receive_window_start_time_s =
        prepared_operating_state.rf_receiver.window_start_time_s;
    rf_v2_detection_context.receive_window_duration_s =
        prepared_operating_state.rf_receiver.window_duration_s;
    rf_v2_detection_context.beam_pointing_deg = prepared_operating_state.beam_pointing_deg;
    rf_v2_detection_context.incident_links = front_end.incident_links;
    rf_v2_detection_context.enable_anti_rgpo_leading_edge =
        Controller().GetControlProfile().enable_anti_rgpo_leading_edge;
    std::vector<ArInterferenceObservation> interference_observations;
    signal::detection::ArDeceptionMeasurementCandidateList deception_candidates;
    if (!front_end.receiver_saturated) {
      if (!TryResolveReceiveObservations(input.rf_scene, prepared_token, prepared_input,
                                         prepared_emission, prepared_operating_state, front_end,
                                         &interference_observations, &deception_candidates)) {
        // 中译：CompleteRfCycle 干扰观测解析失败。
        // 标识：干扰链路失败——干扰观测/欺骗候选解析异常时整周期拒绝。
        PROJECT_LOG_ERROR(
            "[ArSession] CompleteRfCycle interference observation resolution failed.");
        result.status = ArCompleteCycleStatus::kRejected;
        return result;
      }
    }
    oneq::coordinate::LlaPositionDegM platform_lla;
    if (!oneq::coordinate::TryEcefToLla(prepared_input.platform_position_ecef_m, &platform_lla)) {
      // 中译：CompleteRfCycle 无法将平台 ECEF 坐标转为 LLA。
      // 标识：坐标解算失败——平台位置非法时整周期拒绝。
      PROJECT_LOG_ERROR("[ArSession] CompleteRfCycle could not resolve platform ECEF to LLA.");
      result.status = ArCompleteCycleStatus::kRejected;
      return result;
    }
    // 保存干扰观测到结果（在 move 到 cycle_input 之前）。
    result.interference_observations = interference_observations;
    // 构造显式周期输入：所有接收端数据经 SignalCycleInput 一次性传入 pipeline，
    // 不再通过 mutable setter 旁路写入。
    ArSceneTargetList effective_targets =
        front_end.receiver_saturated ? ArSceneTargetList{} : input.targets;
    signal::pipeline::SignalCycleInput cycle_input{
        std::move(effective_targets), &rf_v2_detection_context,
        std::move(interference_observations), std::move(deception_candidates)};
    const ArExecutionCycleResult execution_result = RunExecutionCycle(
        static_cast<std::uint32_t>(prepared_input.world_cycle_index),
        static_cast<float>(prepared_input.window_duration_s),
        static_cast<float>(platform_lla.altitude_m),
        std::move(cycle_input), false);
    if (!execution_result.executed) {
      // 中译：CompleteRfCycle 信号执行失败（中止原因码）。
      // 标识：执行级失败——探测/跟踪链路中止时整周期拒绝，
      //       细粒度原因见 abort_reason 与诊断列表。
      PROJECT_LOG_ERROR("[ArSession] CompleteRfCycle signal execution failed with abort reason {}.",
                        static_cast<int>(execution_result.abort_reason));
      result.status = ArCompleteCycleStatus::kRejected;
      // COMMON-OQ-9：透传执行侧中止原因与明细（校验拒绝时 issues 为校验明细），
      // 由上层 RunCycle 按真实 abort_reason 装配，不再丢原因或写死替换。
      result.abort_reason = execution_result.abort_reason;
      result.issues = execution_result.issues;
      return result;
    }
    result.status = ArCompleteCycleStatus::kCompleted;
    result.output_frame = execution_result.output_frame;
    // 规则 13b：正常执行周期按目标排除的 kInfo 诊断转写（abort 路径不变）。
    result.issues = execution_result.issues;
    prepared_ledger_.ClearPrepared();
    return result;
  }

  ArAbandonCycleStatus AbandonRfCycle(const ArPreparedCycleToken& token) {
    if (!TokenMatches(token)) {
      return ArAbandonCycleStatus::kTokenMismatch;
    }
    prepared_ledger_.ReleasePrepared();
    Controller().ReleasePreparedEmissionControl();
    return ArAbandonCycleStatus::kAbandoned;
  }

  config::mapping::RuntimeConfigState runtime_state{};
  config::mapping::RuntimeConfigState pending_runtime_state{};
  bool has_pending_runtime_update{false};
  bool pending_execution_config_changed{false};
  bool pending_environment_scenario_config_changed{false};
  bool pipeline_config_synced{true};
  PreparedCycleLedger prepared_ledger_{};
  std::uint64_t timing_seed{0x41525f5052495f31ULL};
  std::unique_ptr<MutableArContext> owned_ar_context;
  std::unique_ptr<signal::ISignalPipeline> owned_signal_pipeline;
  std::unique_ptr<environment::IEnvironmentService> owned_environment_service;
  std::unique_ptr<extension::ArController> owned_controller;
  signal::pipeline::SignalPipeline* concrete_signal_pipeline_{nullptr};
  ArTrackLifecycleRecorder* lifecycle_recorder{nullptr};
  ArExclusionCauseRecorder* exclusion_cause_recorder{nullptr};
  std::uint32_t sensor_entity_id{0U};

  void SetSensorEntityId(std::uint32_t id) noexcept {
    sensor_entity_id = id;
    Controller().SetSensorEntityId(id);
  }

  MutableArContext& RadarContext() const { return *owned_ar_context; }
  signal::ISignalPipeline& SignalPipeline() const { return *owned_signal_pipeline; }
  environment::IEnvironmentService& EnvironmentService() const {
    return *owned_environment_service;
  }
  extension::ArController& Controller() const { return *owned_controller; }
};

ArDecisionReplayState ArSessionReplayAccess::CaptureDecisionState(const ArSession& session) {
  ArDecisionReplayState replay_state;
  const extension::ArControllerRuntimeState controller_state =
      session.impl_->Controller().CaptureRuntimeState();
  replay_state.has_pending_internal_decision = controller_state.has_pending_internal_decision;
  replay_state.pending_internal_cycle_index = controller_state.pending_internal_cycle_index;
  replay_state.pending_internal_batch_id = controller_state.pending_internal_batch_id;
  replay_state.pending_internal_proposals = controller_state.pending_internal_proposals;
  replay_state.applied_decision_source = controller_state.last_applied_decision_source;
  replay_state.applied_decision_cycle_index = controller_state.last_applied_decision_cycle_index;
  replay_state.applied_decision_batch_id = controller_state.last_applied_decision_batch_id;
  replay_state.applied_decision_proposals = controller_state.last_applied_decision_proposals;
  replay_state.reducer_state = controller_state.control_reducer_state;
  return replay_state;
}

ArSessionReplayState ArSessionReplayAccess::CaptureSessionState(const ArSession& session) {
  ArSessionReplayState replay_state;
  const PreparedCycleLedger& ledger = session.impl_->prepared_ledger_;
  replay_state.has_world_chronology = ledger.has_world_chronology();
  replay_state.last_world_window_end_s = ledger.last_world_window_end_s();
  replay_state.next_emission_id = ledger.next_emission_id();
  replay_state.successful_prepare_count = ledger.successful_prepare_count();
  replay_state.timing_seed = session.impl_->timing_seed;
  replay_state.frequency_hop_index = static_cast<std::uint64_t>(ledger.frequency_hop_index());
  replay_state.has_pending_runtime_update = session.impl_->has_pending_runtime_update;
  replay_state.pending_execution_config_changed = session.impl_->pending_execution_config_changed;
  replay_state.pending_environment_scenario_config_changed =
      session.impl_->pending_environment_scenario_config_changed;
  replay_state.decision_state = CaptureDecisionState(session);
  return replay_state;
}

ArSession::ArSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

ArSession::ArSession()
    : impl_(new Impl(ArSessionCompositionRoot::ComposeDefault(config::ArSessionConfig{}))) {}

ArSession::~ArSession() = default;
ArSession::ArSession(ArSession&&) noexcept = default;
ArSession& ArSession::operator=(ArSession&&) noexcept = default;

ArSession ArSession::Create(const config::ArSessionConfig& config) {
  return ArSession(std::unique_ptr<ArSession::Impl>(
      new ArSession::Impl(ArSessionCompositionRoot::ComposeDefault(config))));
}

ArSession ArSession::CreateWithDiagnostics(const config::ArSessionConfig& config,
                                           session::ArIssueList* issues) {
  const session::ArIssueList found = config::ValidateArSessionConfig(config);
  if (issues != nullptr) {
    *issues = found;
  }
  return Create(config);
}

session::TrackOutputFrame ArSession::Step(const ArCycleInput& input) {
  return StepWithResult(input).output_frame;
}

ArCycleResult ArSession::StepWithResult(const ArCycleInput& input) {
  ArCycleResult result = impl_->RunCycle(input);
  if (impl_->lifecycle_recorder != nullptr) {
    impl_->lifecycle_recorder->Update(input.targets, result);
  }
  if (impl_->exclusion_cause_recorder != nullptr) {
    impl_->exclusion_cause_recorder->Update(input.targets, result);
  }
  return result;
}

void ArSession::AttachTrackLifecycleRecorder(ArTrackLifecycleRecorder* recorder) noexcept {
  impl_->lifecycle_recorder = recorder;
}

void ArSession::AttachExclusionCauseRecorder(ArExclusionCauseRecorder* recorder) noexcept {
  impl_->exclusion_cause_recorder = recorder;
}

void ArSession::SetSensorEntityId(std::uint32_t sensor_entity_id) noexcept {
  impl_->SetSensorEntityId(sensor_entity_id);
}

const std::vector<session::ArCommand>& ArSession::GetSubmittedCommands() const {
  return impl_->RadarContext().GetSubmittedCommands();
}

bool ArSession::HasLatestControlProfile() const {
  return impl_->RadarContext().HasLatestControlProfile();
}

const session::ArControlProfile& ArSession::GetLatestControlProfile() const {
  return impl_->RadarContext().GetLatestControlProfile();
}

session::AssociationQualityMetrics ArSession::GetLastAssociationQualityMetrics() const {
  return impl_->SignalPipeline().GetLastAssociationQualityMetrics();
}

bool ArSession::TryApplyRuntimeConfig(const config::ArRuntimeConfigPatch& patch) {
  // 事务性提交类（见 docs/common/contract.md「运行期配置提交策略」）：本方法只写入
  // pending_runtime_state，不触碰 runtime_state；配置延迟到下个 StepWithResult 边界
  // 由 CommitPendingRuntimeConfig 原子提交，失败时 4 子系统 capture/restore 回滚，
  // 执行成功后才由 FinalizePendingRuntimeConfig 落定 pending→runtime。
  const config::mapping::RuntimeConfigState& patch_base_state =
      impl_->has_pending_runtime_update ? impl_->pending_runtime_state : impl_->runtime_state;
  const config::mapping::RuntimeConfigResolveResult resolved =
      config::mapping::ApplyRuntimePatch(patch_base_state, patch);
  if (!resolved.has_requested_update || !resolved.is_valid) {
    return false;
  }
  impl_->pending_runtime_state = resolved.next_state;
  impl_->has_pending_runtime_update = true;
  impl_->pending_execution_config_changed =
      impl_->pending_execution_config_changed || resolved.execution_config_changed;
  impl_->pending_environment_scenario_config_changed =
      impl_->pending_environment_scenario_config_changed ||
      resolved.environment_scenario_config_changed;
  return true;
}

session::ExternalDecisionSubmitStatus ArSession::SubmitExternalDecision(
    session::ExternalDecisionOverride override_decision) {
  return impl_->Controller().SubmitExternalDecision(std::move(override_decision));
}

}  // namespace session
}  // namespace airborne_radar
