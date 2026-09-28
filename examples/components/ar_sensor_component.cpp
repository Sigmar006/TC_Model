/**
 * @file ar_sensor_component.cpp
 * @brief AR 传感器组件实现（会话驱动 + 轨迹生命周期事件转发）。
 *
 * 1. 平台位姿经 host_ 读 FlightComponent（零姿态共享局部系），构建 ArCycleInput
 *    驱动 ArSession，输出适配为泛型探测记录（fusion::AdaptArTracksToDetectionRecords）；
 * 2. 轨迹事件（首确认/失跟）由库内 ArTrackLifecycleRecorder 差分产出，组件仅
 *    映射转发为 World 信号（事件位置从本周期外部帧按关联键回查）；
 * 3. 事件与调试视图直写集成端日志（CA_LOG_EVENT / CA_LOG_VIEW，中文人读行）。
 */

#include "ar_sensor_component.h"

#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleOutputAdapter.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"
#include "1q/fusion/SensorAdapters.h"
#include "core/events.h"
#include "core/fusion_detection_bridge.h"
#include "logger/logger.h"
#include "logger/logger_i18n.h"
#include "flight_component.h"
#include "core/world.h"
#include "core/rf_world_broker.h"
#include "core/scene_types.h"
#include "components/sensor_utils.h"

namespace component_attachment {

namespace {

/// 调试轨迹状态 → 中文名（人读日志）。
const char* ArTrackStatusName(airborne_radar::session::ArDebugTrackStatus status) {
  switch (status) {
    case airborne_radar::session::ArDebugTrackStatus::kConfirmed:
      return "已确认";
    case airborne_radar::session::ArDebugTrackStatus::kTentative:
      return "候选";
    case airborne_radar::session::ArDebugTrackStatus::kLost:
      return "丢失";
    case airborne_radar::session::ArDebugTrackStatus::kNotInOutput:
      return "不在输出";
    case airborne_radar::session::ArDebugTrackStatus::kCycleNotCompleted:
      return "周期未完成";
  }
  return "未知";
}

/// 排除主因 → 中文名（人读事件行）。
const char* ArExclusionCauseName(airborne_radar::session::ArIssueCause cause) {
  switch (cause) {
    case airborne_radar::session::ArIssueCause::kNone:
      return "无归因";
    case airborne_radar::session::ArIssueCause::kDistanceLimited:
      return "距离受限";
    case airborne_radar::session::ArIssueCause::kBeamLimited:
      return "波束偏轴";
    case airborne_radar::session::ArIssueCause::kNoiseLimited:
      return "噪声底受限";
    case airborne_radar::session::ArIssueCause::kRcsLimited:
      return "RCS受限";
    case airborne_radar::session::ArIssueCause::kUnknown:
      return "未知主因";
  }
  return "未知主因";
}

/// 世界 ECEF 目标真值 → 平台锚点 ENU 场景输入（ENU 契约的标准集成侧写法：
/// 每周期以平台 ECEF 求锚点一次，逐目标经公共 TryMakeEnuSceneState 转换后直填）。
std::vector<airborne_radar::session::ArTargetInput> BuildArEnuTargets(
    const std::vector<app::TargetEcefState>& world_targets,
    const oneq::coordinate::EcefPositionM& platform_position_ecef_m) {
  std::vector<airborne_radar::session::ArTargetInput> targets;
  if (world_targets.empty()) {
    return targets;
  }
  oneq::coordinate::LlaPositionDegM anchor_lla;
  if (!oneq::coordinate::TryEcefToLla(platform_position_ecef_m, &anchor_lla)) {
    return targets;
  }
  targets.reserve(world_targets.size());
  for (const auto& state : world_targets) {
    oneq::coordinate::ExternalKinematics kinematics;
    kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    kinematics.position_ecef_m = state.position;
    kinematics.velocity_mps = state.velocity;

    oneq::coordinate::EnuSceneState enu;
    if (!oneq::coordinate::TryMakeEnuSceneState(kinematics, anchor_lla, &enu)) {
      continue;
    }
    airborne_radar::session::ArTargetInput target;
    target.target_id = state.id;
    target.position_x = static_cast<float>(enu.position_enu_m.east_m);
    target.position_y = static_cast<float>(enu.position_enu_m.north_m);
    target.position_z = static_cast<float>(enu.position_enu_m.up_m);
    target.velocity_x = static_cast<float>(enu.velocity_enu_mps.east_mps);
    target.velocity_y = static_cast<float>(enu.velocity_enu_mps.north_mps);
    target.velocity_z = static_cast<float>(enu.velocity_enu_mps.up_mps);
    target.rcs = state.rcs;
    target.swerling_type = 0;
    targets.push_back(target);
  }
  return targets;
}

}  // namespace

ArSensorComponent::ArSensorComponent(airborne_radar::session::ArSession session,
                                     std::uint32_t device_id,
                                     DetectionDeliveryMode detection_delivery)
    : session_(std::move(session)),
      device_id_(device_id),
      detection_delivery_(detection_delivery) {
  // 轨迹生命周期事件由库内 recorder 承担（StepWithResult 内部自动喂）。
  session_.AttachTrackLifecycleRecorder(&lifecycle_);
  // 排除原因跨周期差分事件由库内 recorder 承担（与 lifecycle recorder 独立并列）。
  session_.AttachExclusionCauseRecorder(&exclusion_);
  session_.SetSensorEntityId(device_id_);
}

bool ArSensorComponent::TryApplyRuntimeConfig(
    const airborne_radar::config::ArRuntimeConfigPatch& patch) {
  const bool applied = session_.TryApplyRuntimeConfig(patch);
  if (applied && patch.has_sensor_enabled) {
    // 电源状态由补丁唯一维护（组件层电源门控）：AR 补丁为事务性暂存（会话下个
    // 成功周期边界生效），组件在指令接受时即反映开关机；关机期间组件不驱动
    // Step，暂存补丁不提交，标志与会话状态无失步。
    powered_on_ = patch.sensor_enabled;
  }
  return applied;
}

// 视图行写入：三种密度模式编译期三选一（CMake -DCA_VIEW_LOG_MODE=… 或改
// logger/logger_modes.h；未选中的分支不参与编译）。纯观测，不影响会话与信号。
// 各模式输出示例：
//   模式一 nonnominal（只写异常目标行；全部正常时整周期静默不写，防刷屏）：
//     周期=5 目标=1001 状态=候选 位置LLA=(29.74956,128.13653,4000) 速度=250.0m/s
//   模式二 delta（只写状态有变化的目标行；无变化时整周期静默不写，防刷屏）：
//     周期=5 目标=1001 状态=已确认
//   模式三 summary（默认；每周期恰好一行完整摘要）：
//     周期=400 完成=是 目标=[1001 不在输出(RCS 1.20m^2), 1002 不在输出(RCS 0.10m^2)]
//     问题=[ar.target_snr_below_threshold 目标信噪比低于门限…]
void ArSensorComponent::LogDebugView(
    const airborne_radar::session::ArTrackOutputDebugView& view,
    const oneq::coordinate::LlaPositionDegM* origin_lla) {
#if defined(CA_VIEW_LOG_MODE_NONNOMINAL)
  // 模式一：只写非标称目标（非已确认）行；全部正常时本周期静默不写（防刷屏）。
  for (const auto& track : view.tracks) {
    if (track.status == airborne_radar::session::ArDebugTrackStatus::kConfirmed) {
      continue;
    }
    oneq::coordinate::LlaPositionDegM lla;
    const bool have_lla =
        origin_lla != nullptr &&
        TryEnuMetersToLla(static_cast<double>(track.position_x),
                          static_cast<double>(track.position_y),
                          static_cast<double>(track.position_z), *origin_lla, &lla);
    if (have_lla) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string ar_view_log =
          std::string("周期=") +
          std::to_string(view.world_cycle_index) +
          " 目标=" +
          std::to_string(track.external_target_id) +
          " 状态=" +
          (ArTrackStatusName(track.status)) +
          " 位置LLA=(" +
          std::to_string(lla.latitude_deg) +
          "," +
          std::to_string(lla.longitude_deg) +
          "," +
          std::to_string(lla.altitude_m) +
          ") 速度=" +
          std::to_string(track.speed) +
          "m/s";
      CA_LOG_VIEW("ar", "周期={} 目标={} 状态={} 位置LLA=({:.5f},{:.5f},{:.0f}) 速度={:.1f}m/s",
                  view.world_cycle_index, track.external_target_id,
                  ArTrackStatusName(track.status), lla.latitude_deg, lla.longitude_deg,
                  lla.altitude_m, track.speed);
    } else {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string ar_view_log_2 =
          std::string("周期=") +
          std::to_string(view.world_cycle_index) +
          " 目标=" +
          std::to_string(track.external_target_id) +
          " 状态=" +
          (ArTrackStatusName(track.status)) +
          " 位置LLA=无 速度=" +
          std::to_string(track.speed) +
          "m/s";
      CA_LOG_VIEW("ar", "周期={} 目标={} 状态={} 位置LLA=无 速度={:.1f}m/s",
                  view.world_cycle_index, track.external_target_id,
                  ArTrackStatusName(track.status), track.speed);
    }
  }
#elif defined(CA_VIEW_LOG_MODE_DELTA)
  // 模式二：只写状态与上一周期不同的目标行（上一周期状态表由组件持有，首次
  // 出现视为变化；表只增不减，示例不清理）；无变化时静默不写（防刷屏）。
  for (const auto& track : view.tracks) {
    const auto it = prev_track_status_.find(track.external_target_id);
    if (it == prev_track_status_.end() || it->second != track.status) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string ar_view_log_4 =
          std::string("周期=") +
          std::to_string(view.world_cycle_index) +
          " 目标=" +
          std::to_string(track.external_target_id) +
          " 状态=" +
          (ArTrackStatusName(track.status));
      CA_LOG_VIEW("ar", "周期={} 目标={} 状态={}",
                  view.world_cycle_index, track.external_target_id,
                  ArTrackStatusName(track.status));
    }
    prev_track_status_[track.external_target_id] = track.status;
  }
#else  // CA_VIEW_LOG_MODE_SUMMARY（默认）
  // 模式三（每周期摘要行）：目标状态明细带结构化量值（input 回填 RCS，未
  // 跟踪也可见）+ 问题中文名，一眼可读。
  std::string tracks_text;
  for (const auto& track : view.tracks) {
    if (!tracks_text.empty()) {
      tracks_text += ", ";
    }
    tracks_text += std::to_string(track.external_target_id) +
                   std::string(" ") + ArTrackStatusName(track.status) + "(RCS " +
                   std::to_string(track.rcs) + "m^2)";
  }
  const std::string issues_text = app::FormatIssueText(view.issues);
  // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
  const std::string ar_view_log_6 =
      std::string("周期=") +
      std::to_string(view.world_cycle_index) +
      " 完成=" +
      (view.completed_this_cycle ? "是" : "否") +
      " 目标=[" +
      (tracks_text.empty() ? "无" : tracks_text.c_str()) +
      "] 问题=[" +
      (issues_text.empty() ? "无" : issues_text.c_str()) +
      "]";
  CA_LOG_VIEW("ar", "周期={} 完成={} 目标=[{}] 问题=[{}]",
              view.world_cycle_index, view.completed_this_cycle ? "是" : "否",
              tracks_text.empty() ? "无" : tracks_text.c_str(),
              issues_text.empty() ? "无" : issues_text.c_str());
#endif  // CA_VIEW_LOG_MODE_*
}

airborne_radar::session::ArCycleInput ArSensorComponent::BuildCycleInput(
    const FlightComponent& flight, const AppSceneState& scene, double dt_sec) const {
  airborne_radar::session::ArCycleInput input;
  input.cycle_index = static_cast<std::uint32_t>(scene.cycle);
  input.cycle_start_time_s = scene.t_sec;
  input.dt_sec = dt_sec;
  airborne_radar::session::ArPlatformInput pose;
  ResolvePlatformEcef(flight.position(), flight.heading_deg(), flight.speed_mps(),
                      &pose.platform_position_ecef_m, &pose.platform_velocity_mps);
  input.platform = pose;
  input.targets = BuildArEnuTargets(scene.world_targets, pose.platform_position_ecef_m);
  input.interference =
      BuildArInterferenceFromRfWorld(scene.rf_world, device_id_,
                                     scene.t_sec, dt_sec, static_cast<std::uint64_t>(scene.cycle));
  return input;
}

void ArSensorComponent::Step(World& world, double dt_sec) {
  if (!powered_on_) {
    last_debug_view_ = airborne_radar::session::ArTrackOutputDebugView{};  // 关机：调试视图清零（无有效周期）
    LogDebugView(last_debug_view_, nullptr);
    PublishTrackStateEvents(world, last_debug_view_);  // 空视图 → 无在跟航迹事件（消费方本周期无属性输入）
    return;  // 关机：组件不驱动会话（设备不工作），本周期无探测
  }

  const FlightComponent* flight = host_ != nullptr ? host_->Find<FlightComponent>() : nullptr;
  if (flight == nullptr) {
    return;  // 无平台动力学（空场景）：不产生探测
  }

  // 共享场景状态（World 只存基类引用，实际类型为 AppSceneState）：本组件从中
  // 读世界真值组输入，也向其写回射频发射，故直接取可变引用。
  auto& scene = static_cast<AppSceneState&>(world.scene_state());
  const airborne_radar::session::ArCycleInput input = BuildCycleInput(*flight, scene, dt_sec);

  const airborne_radar::session::ArCycleResult result = session_.StepWithResult(input);
  // 调试视图快照每周期都要构建：它是下方视图行的数据源（日志写多少由三密度
  // 模式宏门控，与快照构建无关；被拒绝周期为 kCycleNotCompleted 行）。
  last_debug_view_ = airborne_radar::session::ArTrackOutputDebugViewBuilder::Build(input, result);
  const oneq::coordinate::LlaPositionDegM origin = flight->position();
  LogDebugView(last_debug_view_, &origin);
  PublishTrackStateEvents(world, last_debug_view_);  // 航迹属性逐周期事件（威胁评估订阅）
  if (result.status != airborne_radar::session::ArCycleStatus::kCompleted) {
    return;  // 周期被拒绝：本周期无探测
  }
  // 航迹归属对照表（指令路由器把融合键翻译为外部目标 ID 的权威来源）。
  last_track_attributions_ = result.track_attributions;
  PublishEquipmentEmissions(&scene, result.emission_frame);  // 射频发射 → 共享 RF 世界
  PublishDesignationEvent(world, result);                    // STT 指定任务沿 → 事件日志

  // 事件转发与探测适配统一用外部轨迹帧（雷达局部坐标 → ECEF 已在
  // ArCycleOutputAdapter 边界转换；内部帧为 TrackStateSnapshot，无 ECEF）。
  airborne_radar::session::ArExternalTrackOutputFrame external_frame;
  if (!airborne_radar::session::ArCycleOutputAdapter::Build(input.platform,
                                                            result.output_frame,
                                                            &external_frame)) {
    return;  // 坐标适配失败：本周期无探测
  }

  PublishTrackLifecycleEvents(world, scene, external_frame);  // 航迹首确认/失跟 → 信号+事件日志
  PublishExclusionEvents(world);                              // 排除原因变化沿 → 事件日志
  AdaptDetections(world, scene, external_frame);             // 已发布轨迹 → 共享探测池或消息
}

void ArSensorComponent::PublishTrackStateEvents(
    World& world, const airborne_radar::session::ArTrackOutputDebugView& view) {
  // AR 航迹逐周期状态事件：逐在跟航迹展平速度/RCS/位置（属性侧输入，威胁
  // 评估订阅缓存；消费方不依赖 AR 组件方法）。association_key 与融合键同键
  // 空间（FusedTarget.key 源自 AR association_key 适配）。
  for (const auto& track : view.tracks) {
    if (!track.has_track || track.association_key == 0U) {
      continue;
    }
    ArTrackStateEvent event;
    event.cycle = view.world_cycle_index;
    event.association_key = track.association_key;
    event.speed_m_per_s = track.speed;
    event.rcs_m2 = track.rcs;
    event.position_x_m = track.position_x;
    event.position_y_m = track.position_y;
    event.position_z_m = track.position_z;
    world.signals().on_ar_track_state(event);
  }
}

void ArSensorComponent::PublishDesignationEvent(
    World& world, const airborne_radar::session::ArCycleResult& result) {
  // STT 指定任务沿事件（与 RIR 组件 rir_designation 同形）：designated_target_id
  // 非零归零沿 = 任务终态——回退标志区分超时作废与正常结束；无标志归零为
  // 指定目标航迹确认后的锁定解除（外部清除/任务完成）。锁定生效沿（0→非零）
  // 证明指令事件 → 运行期补丁 → 会话消费的闭环可见。
  if (result.designated_target_id != 0U && prev_designated_target_id_ == 0U) {
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string ar_designation_log =
        std::string("指定目标=") +
        std::to_string(static_cast<unsigned long long>(result.designated_target_id)) +
        " 类型=STT锁定生效 生效模式=" +
        std::to_string(static_cast<unsigned long long>(
            static_cast<int>(result.effective_work_mode)));
    CA_LOG_EVENT(world, "ar_designation",
                 "指定目标={} 类型=STT锁定生效 生效模式={}",
                 static_cast<unsigned long long>(result.designated_target_id),
                 static_cast<int>(result.effective_work_mode));
  } else if (result.designated_target_id == 0U && prev_designated_target_id_ != 0U) {
    const bool timed_out = result.designation_reverted_to_tws;
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string ar_designation_log_2 =
        std::string("指定目标=") +
        std::to_string(static_cast<unsigned long long>(prev_designated_target_id_)) +
        " 类型=" +
        (timed_out ? "窗口耗尽作废回扫" : "锁定解除回扫") +
        " 回退=" +
        (timed_out ? "是" : "否");
    CA_LOG_EVENT(world, "ar_designation", "指定目标={} 类型={} 回退={}",
                 static_cast<unsigned long long>(prev_designated_target_id_),
                 timed_out ? "窗口耗尽作废回扫" : "锁定解除回扫",
                 timed_out ? "是" : "否");
  }
  prev_designated_target_id_ = result.designated_target_id;

}

void ArSensorComponent::PublishTrackLifecycleEvents(
    World& world, const AppSceneState& scene,
    const airborne_radar::session::ArExternalTrackOutputFrame& external_frame) {
  // 库内 recorder 已按跨周期状态差分产出本周期事件（首确认/失跟），组件仅做
  // 映射转发（kUpdated 不转发——示例 AR 信号只关心边界事件）；掉轨后重捕自动
  // 重新产生 kFirstConfirmed。事件目标 ID 优先外部原始目标标识（1001/1002），
  // 无则回退内部关联键。
  for (const auto& event : lifecycle_.GetLastEvents()) {
    const std::uint64_t event_target_id =
        event.external_target_id != 0U ? event.external_target_id : event.association_key;

    if (event.kind == airborne_radar::session::ArTrackLifecycleEventKind::kFirstConfirmed) {
      TargetConfirmedEvent confirmed;
      confirmed.cycle = scene.cycle;
      confirmed.target_id = event_target_id;
      // recorder 事件无位置字段：从本周期帧内同关联键轨迹取回（事件来自本周期
      // 帧，键必命中）；事件为集成契约，LLA 展平为数值字段。
      for (const auto& track : external_frame.tracks) {
        if (track.association_key == event.association_key) {
          oneq::coordinate::LlaPositionDegM lla;
          if (oneq::coordinate::TryEcefToLla(track.target_position_ecef_m, &lla)) {
            confirmed.latitude_deg = lla.latitude_deg;
            confirmed.longitude_deg = lla.longitude_deg;
            confirmed.altitude_m = lla.altitude_m;
          }
          break;
        }
      }
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string target_confirmed_event_log =
          std::string("目标=") +
          std::to_string(static_cast<unsigned long long>(confirmed.target_id)) +
          " 位置=(" +
          std::to_string(confirmed.latitude_deg) +
          "," +
          std::to_string(confirmed.longitude_deg) +
          ")";
      CA_LOG_EVENT(world, "target_confirmed", "目标={} 位置=({:.5f},{:.5f})",
                   static_cast<unsigned long long>(confirmed.target_id),
                   confirmed.latitude_deg, confirmed.longitude_deg);
      world.signals().on_target_confirmed(confirmed);
    } else if (event.kind == airborne_radar::session::ArTrackLifecycleEventKind::kLost) {
      TargetLostEvent lost;
      lost.cycle = scene.cycle;
      lost.target_id = event_target_id;
      lost.reason = "track_lost";  // recorder 的 kLost 事件 reason 恒为 kNone
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string target_lost_event_log =
          std::string("目标=") +
          std::to_string(static_cast<unsigned long long>(lost.target_id)) +
          " 原因=失跟";
      CA_LOG_EVENT(world, "target_lost", "目标={} 原因=失跟",
                   static_cast<unsigned long long>(lost.target_id));
      world.signals().on_target_lost(lost);
    }
  }

}

void ArSensorComponent::PublishExclusionEvents(World& world) {
  // 排除原因跨周期差分沿（进入/变化/退出）→ 仅事件日志，不发 World 信号；
  // 原因稳定不产事件，故无刷屏。
  for (const auto& event : exclusion_.GetLastEvents()) {
    const std::uint64_t event_target_id = event.external_target_id;
    if (event.kind == airborne_radar::session::ArExclusionCauseEventKind::kEntered) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log =
          std::string("目标=") +
          std::to_string(static_cast<unsigned long long>(event_target_id)) +
          " 类型=进入排除 排除码=" +
          (event.current_code) +
          " 主因=" +
          (ArExclusionCauseName(event.current_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "目标={} 类型=进入排除 排除码={} 主因={}",
                   static_cast<unsigned long long>(event_target_id),
                   event.current_code, ArExclusionCauseName(event.current_cause));
    } else if (event.kind == airborne_radar::session::ArExclusionCauseEventKind::kChanged) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log_2 =
          std::string("目标=") +
          std::to_string(static_cast<unsigned long long>(event_target_id)) +
          " 类型=原因变化 旧码=" +
          (event.previous_code) +
          " 旧主因=" +
          (ArExclusionCauseName(event.previous_cause)) +
          " 新码=" +
          (event.current_code) +
          " 新主因=" +
          (ArExclusionCauseName(event.current_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "目标={} 类型=原因变化 旧码={} 旧主因={} 新码={} 新主因={}",
                   static_cast<unsigned long long>(event_target_id),
                   event.previous_code, ArExclusionCauseName(event.previous_cause),
                   event.current_code, ArExclusionCauseName(event.current_cause));
    } else if (event.kind == airborne_radar::session::ArExclusionCauseEventKind::kExited) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log_3 =
          std::string("目标=") +
          std::to_string(static_cast<unsigned long long>(event_target_id)) +
          " 类型=退出排除 旧码=" +
          (event.previous_code) +
          " 旧主因=" +
          (ArExclusionCauseName(event.previous_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "目标={} 类型=退出排除 旧码={} 旧主因={}",
                   static_cast<unsigned long long>(event_target_id),
                   event.previous_code, ArExclusionCauseName(event.previous_cause));
    }
  }

}

void ArSensorComponent::AdaptDetections(
    World& world, AppSceneState& scene,
    const airborne_radar::session::ArExternalTrackOutputFrame& external_frame) {
  const std::vector<fusion::DetectionRecord> records =
      fusion::AdaptArTracksToDetectionRecords(device_id_, external_frame);
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


}  // namespace component_attachment
