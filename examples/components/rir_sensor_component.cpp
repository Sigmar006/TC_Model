/**
 * @file rir_sensor_component.cpp
 * @brief RIR 地基站点传感器组件实现（会话驱动 + 识别/指定任务事件 + 融合量测）。
 *
 * 1. RirCycleInput 直接构造（固定站点海拔/ECEF + 消费方注入的站点局部 ENU
 *    场景目标帧），驱动 RirSession；指定识别任务经运行期补丁接口下发
 *    （外部指令事件 → CommandRouter → TryApplyRuntimeConfig；识别完成或
 *    窗口耗尽由会话状态机自动回扫）；
 * 2. 识别结论进入确认态时发关键事件（逐航迹状态迁移判定，避免每周期重复）；
 *    指定任务回扫沿发事件；每周期视图行汇总归属航迹/驻留中心；
 * 3. 特征量测（出口①）适配为泛型探测记录（fusion::AdaptRirFeatureMeasurements-
 *    ToDetectionRecords，源通道 kRirSourceId=5），FusionComponent 跨实体聚合。
 */

#include "rir_sensor_component.h"

#include <chrono>
#include <cstdio>
#include <cstdint>
#include <string>
#include <unordered_map>

#include "1q/coordinate/position_transform.h"
#include "1q/fusion/SensorAdapters.h"
#include "core/events.h"
#include "core/fusion_detection_bridge.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "logger/acceptance_timing.h"
#include "core/world.h"
#include "logger/logger.h"
#include "logger/logger_i18n.h"
#include "core/rf_world_broker.h"
#include "core/scene_types.h"
#include "components/sensor_utils.h"
#include "common/logging/AcceptanceText.h"
#include "remote_identification_radar/runtime/RirAcceptanceLog.h"

namespace component_attachment {

namespace {

namespace rir = remote_identification_radar::session;

/// 识别状态 → 中文名（人读事件行）。
const char* RecognitionStateName(rir::RirRecognitionState state) {
  switch (state) {
    case rir::RirRecognitionState::kDisabled:
      return "未启用";
    case rir::RirRecognitionState::kAccumulating:
      return "积累中";
    case rir::RirRecognitionState::kCategoryConfirmed:
      return "大类确认";
    case rir::RirRecognitionState::kModelConfirmed:
      return "型号确认";
    case rir::RirRecognitionState::kUnknown:
      return "无候选";
    case rir::RirRecognitionState::kStale:
      return "结论过期";
  }
  return "未知";
}

/// 识别大类 → 中文名（人读事件行）。
const char* RecognitionCategoryName(rir::RirRecognitionCategory category) {
  switch (category) {
    case rir::RirRecognitionCategory::kBallistic:
      return "弹道目标";
    case rir::RirRecognitionCategory::kNearSpace:
      return "临近空间";
    case rir::RirRecognitionCategory::kOther:
      return "其它";
    case rir::RirRecognitionCategory::kUnknown:
      return "未知";
    case rir::RirRecognitionCategory::kFighter:
      return "战斗机";
    case rir::RirRecognitionCategory::kBomber:
      return "轰炸机";
    case rir::RirRecognitionCategory::kMissile:
      return "导弹";
  }
  return "未知";
}

/// 指定任务回退成因 → 中文名（人读事件行）。
const char* DesignationRevertReasonName(
    remote_identification_radar::session::RirDesignationRevertReason reason) {
  switch (reason) {
    case remote_identification_radar::session::RirDesignationRevertReason::kNone:
      return "无";
    case remote_identification_radar::session::RirDesignationRevertReason::kNotRecognized:
      return "目标缺席";
    case remote_identification_radar::session::RirDesignationRevertReason::kAcquisitionTimeout:
      return "窗口超时";
    case remote_identification_radar::session::RirDesignationRevertReason::kOutsideSteerableVolume:
      return "超出可扫描体积";
  }
  return "未知";
}

/// 调试目标状态 → 中文名（人读视图行）。
const char* DebugTargetStatusName(rir::RirDebugTargetStatus status) {
  switch (status) {
    case rir::RirDebugTargetStatus::kConfirmed:
      return "已确认";
    case rir::RirDebugTargetStatus::kTentative:
      return "候选";
    case rir::RirDebugTargetStatus::kLost:
      return "丢失";
    case rir::RirDebugTargetStatus::kNotInOutput:
      return "无航迹";
    case rir::RirDebugTargetStatus::kCycleNotCompleted:
      return "周期未完成";
  }
  return "未知";
}

/// 排除主因 → 中文名（人读事件行）。
const char* ExclusionCauseName(rir::RirIssueCause cause) {
  switch (cause) {
    case rir::RirIssueCause::kNone:
      return "无归因";
    case rir::RirIssueCause::kDistanceLimited:
      return "距离受限";
    case rir::RirIssueCause::kBeamLimited:
      return "波束偏轴";
    case rir::RirIssueCause::kNoiseLimited:
      return "噪声底受限";
    case rir::RirIssueCause::kRcsLimited:
      return "RCS受限";
    case rir::RirIssueCause::kUnknown:
      return "未知主因";
  }
  return "未知主因";
}

/// 航迹生命周期状态 → 中文名（人读事件行）。
const char* TrackStatusName(rir::RirTrackLifecycleStatus status) {
  switch (status) {
    case rir::RirTrackLifecycleStatus::kConfirmed:
      return "已确认";
    case rir::RirTrackLifecycleStatus::kTentative:
      return "候选";
    case rir::RirTrackLifecycleStatus::kLost:
      return "丢失";
  }
  return "未知";
}

/// 本周期检测 SNR 后缀（单脉冲口径）；未检出时返回空串。
std::string FormatCycleSnrSuffix(const rir::RirDebugTargetState& state) {
  if (!state.has_cycle_detection_snr) {
    return std::string();
  }
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), " SNR=%.1fdB", state.cycle_detection_snr_db);
  return buffer;
}

}  // namespace

RirSensorComponent::RirSensorComponent(
    rir::RirSession session, const oneq::coordinate::LlaPositionDegM& site_origin,
    float recognition_dwell_sec,
    std::uint32_t device_id,
    DetectionDeliveryMode detection_delivery)
    : session_(std::move(session)),
      site_origin_(site_origin),
      recognition_dwell_sec_(recognition_dwell_sec),
      device_id_(device_id),
      detection_delivery_(detection_delivery) {
  session_.SetSensorEntityId(device_id_);
  // 站点固定：ECEF 解析一次，逐周期作为特征量测 sensor_origin 提供。
  oneq::coordinate::TryLlaToEcef(site_origin_, &site_ecef_);
  // 生命周期/排除差分记录器：StepWithResult 内部自动喂，组件只读事件。
  session_.AttachTrackLifecycleRecorder(&lifecycle_);
  session_.AttachExclusionCauseRecorder(&exclusion_);
}

bool RirSensorComponent::TryApplyRuntimeConfig(
    const remote_identification_radar::config::RirRuntimeConfigPatch& patch) {
  const bool applied = session_.TryApplyRuntimeConfig(patch);
  if (applied && patch.has_sensor_enabled) {
    powered_on_ = patch.sensor_enabled;  // 电源状态由补丁唯一维护（组件层电源门控）
  }
  return applied;
}

void RirSensorComponent::WriteSbirsCueDetectionAcceptance(
    std::uint32_t cycle, double sim_time_sec, const std::string& source_name,
    std::uint64_t target_id, double bearing_time_sec, double cue_az_deg,
    double cue_el_deg, double radar_range_m, double radar_az_deg,
    double radar_height_m) const {
  std::string content = "目标ID=" + std::to_string(static_cast<unsigned long long>(target_id));
  content += " 传感器信息源名称=" + source_name;
  content += " 卫星测角时刻=" + oneq::logging::FormatF(bearing_time_sec, 3) + "s";
  content += " 卫星引导方位/俯仰=" +
             oneq::logging::FormatPairDeg(cue_az_deg, cue_el_deg, 4) + "°";
  content += " 雷达探测时刻=" + oneq::logging::FormatF(sim_time_sec, 3) + "s";
  content += " 雷达距离=" + oneq::logging::FormatF(radar_range_m, 3) + "m";
  content += " 雷达方位=" + oneq::logging::FormatF(radar_az_deg, 4) + "°";
  content += " 雷达高度=" + oneq::logging::FormatF(radar_height_m, 3) + "m";
  content += " 时序=卫星测角后";
  RIR_ACCEPTANCE_ITEM(static_cast<float>(sim_time_sec), cycle, "落点预测功能测试", content);
}

rir::RirCycleInput RirSensorComponent::BuildCycleInput(const AppSceneState& scene,
                                                       double dt_sec) const {
  rir::RirCycleInput input;
  input.input_cycle_index = static_cast<std::uint32_t>(scene.cycle);
  input.dt_sec = dt_sec;
  input.sim_time_sec = static_cast<float>(scene.t_sec);
  input.platform_position = site_ecef_;
  // 世界 ECEF 真值 → 站点局部 ENU + 识别特征真值铺样的转换写法见
  // scenes/scene_script.cpp 的 MakeRirSceneTargets（app/runner.cpp 每周期注入）。
  input.scene_targets = scene.rir_targets;
  // RF 世界是全装备共享的（含本传感器自己的发射）：会话要的是"外部电磁环境"，
  // 故先剔除自身 device_id 的发射再喂入（自己不观测自己的发射）。
  input.rf_scene = BuildExternalRfScene(scene.rf_world, device_id_, scene.t_sec,
                                        static_cast<double>(recognition_dwell_sec_),
                                        static_cast<std::uint64_t>(scene.cycle));
  return input;
}

void RirSensorComponent::Step(World& world, double dt_sec) {
  if (!powered_on_) {
    // 关机：不驱动会话；视图重置为空快照再走一遍写入（摘要模式写一行空摘要，
    // 模式一/二天然静默）。
    last_debug_view_ = rir::RirOutputDebugView{};
    last_max_detected_slant_range_m_ = 0.0f;
    LogDebugView(world, last_debug_view_);
    return;
  }
  // 共享场景状态（World 只存基类引用，实际类型为 AppSceneState）：本组件从中
  // 读世界真值组输入，也向其写回射频发射，故直接取可变引用。
  auto& scene = static_cast<AppSceneState&>(world.scene_state());

  const rir::RirCycleInput input = BuildCycleInput(scene, dt_sec);

  const std::chrono::steady_clock::time_point step_begin = std::chrono::steady_clock::now();
  const rir::RirCycleResult result = session_.StepWithResult(input);
  if (!step_timing_logged_) {
    app::LogAcceptanceMs(scene.cycle, scene.t_sec, "单步执行时间性能测试", "RIR",
                          app::SteadyElapsedMs(step_begin));
    step_timing_logged_ = true;
  }
  // 调试视图快照每周期都要构建：它是下方视图行的数据源（日志写多少由三密度
  // 模式宏门控，与快照构建无关；被拒绝周期为 kCycleNotCompleted 行）。
  last_debug_view_ = rir::RirOutputDebugViewBuilder::Build(input, result);
  // 库上报本周期「实际有效目标最大斜距」（供可视化区分粗筛门 max_range_m）。
  last_max_detected_slant_range_m_ = result.max_detected_slant_range_m;
  LogDebugView(world, last_debug_view_);
  if (result.status != rir::RirCycleStatus::kCompleted) {
    return;  // 周期被拒绝：本周期无量测/结论
  }
  // 周期成功，按序发布本周期产物：
  PublishEquipmentEmissions(&scene, result.emission_frame);  // 射频发射 → 共享 RF 世界
  PublishRecognitionEvents(world, result);                   // 识别结论确认沿 → 事件日志
  PublishDesignationEvent(world, result);                    // 指定任务终态沿 → 事件日志
  PublishTrackLifecycleEvents(world, result);                // 航迹首确认/丢失/作废沿 → 事件日志
  PublishExclusionEvents(world, result);                     // 排除原因变化沿 → 事件日志
  AdaptDetections(world, scene, result);                            // 特征量测 → 共享探测池或消息
}

void RirSensorComponent::PublishRecognitionEvents(
    World& world, const rir::RirCycleResult& result) {
  // 逐航迹识别结论：与本组件记录的上一周期状态比对，"非确认 → 确认"的迁移沿
  // 写一条关键事件（确认态持续期间不逐周期重复）；任一航迹确认则本周期计入
  // 确认周期数（冒烟下限用）。
  bool any_confirmed = false;
  for (const auto& output : result.output_frame.recognition_outputs) {
    const rir::RirRecognitionState state = output.result.state;
    // 确认态 = 大类确认/型号确认两种终态。
    const bool confirmed =
        state == rir::RirRecognitionState::kCategoryConfirmed ||
        state == rir::RirRecognitionState::kModelConfirmed;
    any_confirmed = any_confirmed || confirmed;
    // 上一周期状态查表：首次进入确认才发事件。
    const auto it = prev_recognition_states_.find(output.association_key);
    const bool was_confirmed =
        it != prev_recognition_states_.end() &&
        (it->second == rir::RirRecognitionState::kCategoryConfirmed ||
         it->second == rir::RirRecognitionState::kModelConfirmed);
    if (confirmed && !was_confirmed) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string rir_recognition_event_log =
          std::string("航迹=") +
          std::to_string(static_cast<unsigned long long>(output.association_key)) +
          " 状态=" +
          (RecognitionStateName(state)) +
          " 大类=" +
          (RecognitionCategoryName(output.result.target_category)) +
          " 型号=" +
          (output.result.target_model.empty() ? "-" : output.result.target_model) +
          " 置信=" +
          std::to_string(output.result.confidence) +
          " 观测数=" +
          std::to_string(output.result.observation_count);
      CA_LOG_EVENT(world, "rir_recognition",
                   "航迹={} 状态={} 大类={} 型号={} 置信={:.2f} 观测数={}",
                   static_cast<unsigned long long>(output.association_key),
                   RecognitionStateName(state),
                   RecognitionCategoryName(output.result.target_category),
                   output.result.target_model.empty() ? "-" : output.result.target_model,
                   output.result.confidence, output.result.observation_count);
    }
    prev_recognition_states_[output.association_key] = state;
  }
  if (any_confirmed) {
    ++confirmed_recognition_outputs_;
  }

}

void RirSensorComponent::PublishDesignationEvent(
    World& world, const rir::RirCycleResult& result) {
  // 指定任务终态沿事件：designated_target_id 从非零归零 = 任务结束。结束语义按
  // 回退成因区分——kAcquisitionTimeout 为窗口耗尽作废；无回退标志为识别达成
  // 完成（成功完成不置 designation_reverted_to_scan，该标志仅缺席/超时使用）。
  const bool designation_assigned = result.designated_target_id != 0U;
  if (!designation_assigned && prev_designated_target_id_ != 0U) {
    const bool timed_out =
        result.designation_revert_reason ==
        remote_identification_radar::session::RirDesignationRevertReason::kAcquisitionTimeout;
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string rir_designation_event_log =
        std::string("指定目标=") +
        std::to_string(static_cast<unsigned long long>(prev_designated_target_id_)) +
        " 类型=" +
        (timed_out ? "任务作废回扫" : "识别达成完成") +
        " 成因=" +
        (DesignationRevertReasonName(result.designation_revert_reason));
    CA_LOG_EVENT(world, "rir_designation",
                 "指定目标={} 类型={} 成因={}",
                 static_cast<unsigned long long>(prev_designated_target_id_),
                 timed_out ? "任务作废回扫" : "识别达成完成",
                 DesignationRevertReasonName(result.designation_revert_reason));
  }
  prev_designated_target_id_ = result.designated_target_id;

}

void RirSensorComponent::PublishTrackLifecycleEvents(
    World& world, const rir::RirCycleResult& result) {
  // 航迹生命周期沿事件（首确认/丢失/指定作废）→ 仅事件日志；kUpdated 为周期性
  // 重复事件、kNotTracked 为诊断事件，均不落盘。
  for (const auto& event : lifecycle_.GetLastEvents()) {
    switch (event.kind) {
      case rir::RirTrackLifecycleEventKind::kFirstConfirmed: {
        // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
        const std::string rir_track_confirmed_log =
            std::string("周期=") + std::to_string(event.world_cycle_index) + " 目标=" +
            std::to_string(static_cast<unsigned long long>(event.external_target_id)) +
            " 航迹=" +
            std::to_string(static_cast<unsigned long long>(event.association_key)) +
            " 状态=已确认 速度=" + std::to_string(event.speed_m_per_s);
        CA_LOG_EVENT(world, "rir_track_confirmed",
                     "周期={} 目标={} 航迹={} 状态=已确认 速度={:.1f}",
                     event.world_cycle_index,
                     static_cast<unsigned long long>(event.external_target_id),
                     static_cast<unsigned long long>(event.association_key),
                     event.speed_m_per_s);
        break;
      }
      case rir::RirTrackLifecycleEventKind::kLost: {
        // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
        const std::string rir_track_lost_log =
            std::string("周期=") + std::to_string(event.world_cycle_index) + " 目标=" +
            std::to_string(static_cast<unsigned long long>(event.external_target_id)) +
            " 航迹=" +
            std::to_string(static_cast<unsigned long long>(event.association_key)) +
            " 状态=丢失";
        CA_LOG_EVENT(world, "rir_track_lost", "周期={} 目标={} 航迹={} 状态=丢失",
                     event.world_cycle_index,
                     static_cast<unsigned long long>(event.external_target_id),
                     static_cast<unsigned long long>(event.association_key));
        break;
      }
      case rir::RirTrackLifecycleEventKind::kDesignationDropped: {
        // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
        const std::string rir_designation_dropped_log =
            std::string("周期=") + std::to_string(event.world_cycle_index) + " 目标=" +
            std::to_string(static_cast<unsigned long long>(event.external_target_id)) +
            " 航迹=" +
            std::to_string(static_cast<unsigned long long>(event.association_key)) +
            " 状态=" + (TrackStatusName(event.track_status)) + " 作废成因=" +
            (DesignationRevertReasonName(event.designation_revert_reason));
        CA_LOG_EVENT(world, "rir_designation_dropped",
                     "周期={} 目标={} 航迹={} 状态={} 作废成因={}",
                     event.world_cycle_index,
                     static_cast<unsigned long long>(event.external_target_id),
                     static_cast<unsigned long long>(event.association_key),
                     TrackStatusName(event.track_status),
                     DesignationRevertReasonName(event.designation_revert_reason));
        break;
      }
      case rir::RirTrackLifecycleEventKind::kUpdated:
      case rir::RirTrackLifecycleEventKind::kNotTracked:
      default:
        break;
    }
  }

}

void RirSensorComponent::PublishExclusionEvents(
    World& world, const rir::RirCycleResult& result) {
  // 排除原因跨周期差分沿（进入/变化/退出）→ 仅事件日志，不发 World 信号；
  // 原因稳定不产事件，故无刷屏。
  for (const auto& event : exclusion_.GetLastEvents()) {
    if (event.kind == rir::RirExclusionCauseEventKind::kEntered) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log =
          std::string("周期=") + std::to_string(event.world_cycle_index) + " 目标=" +
          std::to_string(static_cast<unsigned long long>(event.external_target_id)) +
          " 类型=进入排除 排除码=" + event.current_code + " 主因=" +
          (ExclusionCauseName(event.current_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "周期={} 目标={} 类型=进入排除 排除码={} 主因={}",
                   event.world_cycle_index,
                   static_cast<unsigned long long>(event.external_target_id),
                   event.current_code, ExclusionCauseName(event.current_cause));
    } else if (event.kind == rir::RirExclusionCauseEventKind::kChanged) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log_2 =
          std::string("周期=") + std::to_string(event.world_cycle_index) + " 目标=" +
          std::to_string(static_cast<unsigned long long>(event.external_target_id)) +
          " 类型=原因变化 旧码=" + event.previous_code + " 旧主因=" +
          (ExclusionCauseName(event.previous_cause)) + " 新码=" + event.current_code +
          " 新主因=" + (ExclusionCauseName(event.current_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "周期={} 目标={} 类型=原因变化 旧码={} 旧主因={} 新码={} 新主因={}",
                   event.world_cycle_index,
                   static_cast<unsigned long long>(event.external_target_id),
                   event.previous_code, ExclusionCauseName(event.previous_cause),
                   event.current_code, ExclusionCauseName(event.current_cause));
    } else if (event.kind == rir::RirExclusionCauseEventKind::kExited) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log_3 =
          std::string("周期=") + std::to_string(event.world_cycle_index) + " 目标=" +
          std::to_string(static_cast<unsigned long long>(event.external_target_id)) +
          " 类型=退出排除 旧码=" + event.previous_code + " 旧主因=" +
          (ExclusionCauseName(event.previous_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "周期={} 目标={} 类型=退出排除 旧码={} 旧主因={}",
                   event.world_cycle_index,
                   static_cast<unsigned long long>(event.external_target_id),
                   event.previous_code, ExclusionCauseName(event.previous_cause));
    }
  }

}

void RirSensorComponent::AdaptDetections(
    World& world, AppSceneState& scene, const rir::RirCycleResult& result) {
  // 特征量测 → 统一探测记录（键按归属表从库内键重写为外部目标 ID，与其它源同键
  // 后才会并进同一条融合航迹；无归属记录保留库内键）。
  rir::RirFeatureMeasurementFrame frame;
  frame.input_cycle_index = result.output_frame.input_cycle_index;
  frame.batch_id = result.output_frame.batch_id;
  frame.records = result.output_frame.feature_measurements;
  std::vector<fusion::DetectionRecord> records =
      fusion::AdaptRirFeatureMeasurementsToDetectionRecords(device_id_, frame);
  std::unordered_map<std::uint64_t, std::uint64_t> key_to_external;
  for (const auto& attribution : result.track_attributions) {
    if (attribution.external_target_id != 0U) {
      key_to_external[attribution.association_key] = attribution.external_target_id;
    }
  }
  for (auto& record : records) {
    const auto it = key_to_external.find(record.key);
    if (it != key_to_external.end()) {
      record.key = it->second;
    }
  }
  if (detection_delivery_ == DetectionDeliveryMode::kMessage) {
    // 消息路径：展平为基础类型后发 on_detection_batch_submitted（融合组件
    // 订阅重建库类型；集成方对应把样本推给融合组件的消息）。
    DetectionBatchSubmittedEvent event;
    event.cycle = scene.cycle;
    event.source_id = device_id_;
    for (const auto& record : records) {
      event.records.push_back(ToFusionDetectionSample(record));
    }
    world.signals().on_detection_batch_submitted(event);
    return;
  }
  // 黑板路径：库内适配器 → 共享探测池（FusionComponent 聚合读；集成方对应
  // 把记录消息推给融合组件）。
  scene.detection_pool.insert(scene.detection_pool.end(), records.begin(), records.end());
}

void RirSensorComponent::LogDebugView(World& world, const rir::RirOutputDebugView& view) {
  // 视图行写入：三种密度模式编译期三选一（CMake -DCA_VIEW_LOG_MODE=… 或改
  // logger/logger_modes.h；未选中的分支不参与编译）。纯观测，不影响会话与信号。
  // 各模式输出示例：
  //   模式一 nonnominal（只写异常目标行；全部正常时整周期静默不写，防刷屏）：
  //     周期=5 目标=1002 状态=无航迹 斜距=9000m
  //   模式二 delta（只写状态有变化的目标行；无变化时整周期静默不写，防刷屏）：
  //     周期=5 目标=1001 状态=已确认 SNR=12.3dB
  //   模式三 summary（默认；每周期恰好一行完整摘要）：
  //     周期=1 完成=是 航迹=1 确认=否 指定=无 驻留中心=(-110.0°,85.0°)
  //     目标=[1001 候选(F-16C) 积累中 置信0.00 位置LLA=(…) 速度=250.0 SNR=12.3dB,
  //     1002 无航迹(BGM-109) 斜距=9000m 速度=0.0] 问题=[无]
#if defined(CA_VIEW_LOG_MODE_NONNOMINAL)
  // 模式一：只写非标称目标（非已确认）行；全部正常时本周期静默不写（防刷屏）。
  for (const auto& state : view.targets) {
    if (state.external_target_id == 0U ||
        state.status == rir::RirDebugTargetStatus::kConfirmed) {
      continue;
    }
    oneq::coordinate::LlaPositionDegM lla;
    const bool have_lla =
        state.has_track && TryEnuMetersToLla(state.position_enu_x_m, state.position_enu_y_m,
                                             state.position_enu_z_m, site_origin_, &lla);
    const std::string snr_suffix = FormatCycleSnrSuffix(state);
    if (have_lla) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string rir_view_log =
          std::string("周期=") + std::to_string(view.input_cycle_index) + " 目标=" +
          std::to_string(static_cast<unsigned long long>(state.external_target_id)) +
          " 状态=" + (DebugTargetStatusName(state.status)) + " 位置LLA=有" + " 速度=" +
          std::to_string(state.speed_m_per_s) + snr_suffix;
      CA_LOG_VIEW("rir", "周期={} 目标={} 状态={} 位置LLA=({:.5f},{:.5f},{:.0f}) 速度={:.1f}{}",
                  view.input_cycle_index,
                  static_cast<unsigned long long>(state.external_target_id),
                  DebugTargetStatusName(state.status), lla.latitude_deg, lla.longitude_deg,
                  lla.altitude_m, state.speed_m_per_s, snr_suffix);
    } else {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string rir_view_log =
          std::string("周期=") + std::to_string(view.input_cycle_index) + " 目标=" +
          std::to_string(static_cast<unsigned long long>(state.external_target_id)) +
          " 状态=" + (DebugTargetStatusName(state.status)) + " 斜距=" +
          std::to_string(state.slant_range_m) + snr_suffix;
      CA_LOG_VIEW("rir", "周期={} 目标={} 状态={} 斜距={:.0f}m{}", view.input_cycle_index,
                  static_cast<unsigned long long>(state.external_target_id),
                  DebugTargetStatusName(state.status), state.slant_range_m, snr_suffix);
    }
  }
#elif defined(CA_VIEW_LOG_MODE_DELTA)
  // 模式二：只写状态与上一周期不同的目标行；无变化时本周期静默不写（防刷屏）。
  for (const auto& state : view.targets) {
    if (state.external_target_id == 0U) {
      continue;
    }
    const auto it = prev_target_status_.find(state.external_target_id);
    const bool first_or_changed = it == prev_target_status_.end() || it->second != state.status;
    prev_target_status_[state.external_target_id] = state.status;
    if (!first_or_changed) {
      continue;
    }
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string snr_suffix = FormatCycleSnrSuffix(state);
    const std::string rir_view_log =
        std::string("周期=") + std::to_string(view.input_cycle_index) + " 目标=" +
        std::to_string(static_cast<unsigned long long>(state.external_target_id)) + " 状态=" +
        (DebugTargetStatusName(state.status)) + snr_suffix;
    CA_LOG_VIEW("rir", "周期={} 目标={} 状态={}{}", view.input_cycle_index,
                static_cast<unsigned long long>(state.external_target_id),
                DebugTargetStatusName(state.status), snr_suffix);
  }
#else
  // 模式三：每周期恰好一行完整摘要（含指定任务镜像与排除诊断问题列表）。
  // 逐目标片段累积（"ID 状态(型号) 识别 置信 位置/斜距 速度"），拼进下方摘要行
  // 的 目标=[…] 字段随行写日志——片段拼接与摘要行同为纯 std::string/std::to_string
  // 写法，集成方可整段搬入己方日志组装（数值为 std::to_string 缺省精度）。
  std::string target_parts;
  std::size_t track_count = 0U;
  bool any_confirmed = false;
  for (const auto& state : view.targets) {
    if (state.external_target_id == 0U) {
      continue;
    }
    track_count += state.has_track ? 1U : 0U;
    any_confirmed = any_confirmed || state.status == rir::RirDebugTargetStatus::kConfirmed;
    if (!target_parts.empty()) {
      target_parts += ", ";
    }
    std::string part =
        std::to_string(static_cast<unsigned long long>(state.external_target_id)) +
        std::string(" ") + DebugTargetStatusName(state.status) + "(" +
        (state.target_name.empty() ? "-" : state.target_name) + ")";
    if (state.has_recognition_output) {
      part += std::string(" ") + RecognitionStateName(state.recognition_state) +
              " 置信" + std::to_string(state.confidence);
    }
    oneq::coordinate::LlaPositionDegM lla;
    if (state.has_track &&
        TryEnuMetersToLla(state.position_enu_x_m, state.position_enu_y_m,
                          state.position_enu_z_m, site_origin_, &lla)) {
      part += std::string(" 位置LLA=(") + std::to_string(lla.latitude_deg) + "," +
              std::to_string(lla.longitude_deg) + "," +
              std::to_string(lla.altitude_m) + ")";
    } else {
      part += std::string(" 斜距=") + std::to_string(state.slant_range_m) + "m";
    }
    part += std::string(" 速度=") + std::to_string(state.speed_m_per_s);
    part += FormatCycleSnrSuffix(state);
    target_parts += part;
  }
  const std::string issues_text = app::FormatIssueText(view.issues);
  // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
  const std::string rir_view_log =
      std::string("周期=") + std::to_string(view.input_cycle_index) + " 完成=" +
      (view.executed_this_cycle ? "是" : "否") + " 航迹=" +
      std::to_string(track_count) + " 确认=" + (any_confirmed ? "是" : "否") +
      " 指定=" + (view.designation_active ? "执行中" : "无") + " 驻留中心=(" +
      std::to_string(view.dwell_center_deg.az_deg) + "°," +
      std::to_string(view.dwell_center_deg.el_deg) + "°) 目标=[" +
      (target_parts.empty() ? "无" : target_parts) + "] 问题=[" +
      (issues_text.empty() ? "无" : issues_text) + "]";
  CA_LOG_VIEW("rir",
              "周期={} 完成={} 航迹={} 确认={} 指定={} 驻留中心=({:.1f}°,{:.1f}°) 目标=[{}] 问题=[{}]",
              view.input_cycle_index, view.executed_this_cycle ? "是" : "否",
              track_count, any_confirmed ? "是" : "否",
              view.designation_active ? "执行中" : "无", view.dwell_center_deg.az_deg,
              view.dwell_center_deg.el_deg, target_parts.empty() ? "无" : target_parts.c_str(),
              issues_text.empty() ? "无" : issues_text.c_str());
#endif
}

}  // namespace component_attachment
