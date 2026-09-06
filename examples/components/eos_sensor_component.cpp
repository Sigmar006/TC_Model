/**
 * @file eos_sensor_component.cpp
 * @brief EOS 传感器组件实现（会话驱动 + 探测生命周期事件转发）。
 *
 * 1. 公共 TryMakeEnuSceneState 直填 EosSceneTarget + 手填 EosCycleInput，驱动
 *    EosSession，输出探测适配为泛型探测记录（fusion::AdaptEosDetectionsToDetectionRecords）；
 * 2. 探测事件（首发现/更新/丢失）由库内 EosDetectionLifecycleRecorder 差分
 *    产出，组件经归属映射关联目标 ID 后发布 EosDetectionEvent；
 * 3. 事件与调试视图直写集成端日志（CA_LOG_EVENT / CA_LOG_VIEW，中文人读行）。
 */

#include "eos_sensor_component.h"

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosSceneTypes.h"
#include "1q/fusion/SensorAdapters.h"
#include "core/events.h"
#include "core/fusion_detection_bridge.h"
#include "logger/logger.h"
#include "logger/logger_i18n.h"
#include "flight_component.h"
#include "core/world.h"
#include "core/scene_types.h"
#include "components/sensor_utils.h"

namespace component_attachment {

namespace {

/// 探测记录 ID → 仿真目标 ID（无归属返回 0）。
std::uint64_t AttributionTargetId(
    std::uint64_t detection_id,
    const electro_optical_sensor::attribution::EosDetectionAttributionRecordList& attributions) {
  for (const auto& attribution : attributions) {
    if (attribution.detection_id == detection_id) {
      return attribution.target_id;
    }
  }
  return 0U;  // 无归属（调试视图缺失）：未知目标
}

/// 生命周期事件类型 → 示例事件类型（kNotDetected 诊断事件不转发）。
EosDetectionEventKind ToAppKind(
    electro_optical_sensor::session::EosDetectionLifecycleEventKind kind) {
  switch (kind) {
    case electro_optical_sensor::session::EosDetectionLifecycleEventKind::kFirstDetected:
      return EosDetectionEventKind::kFirstDetected;
    case electro_optical_sensor::session::EosDetectionLifecycleEventKind::kUpdated:
      return EosDetectionEventKind::kUpdated;
    case electro_optical_sensor::session::EosDetectionLifecycleEventKind::kLost:
      return EosDetectionEventKind::kLost;
    default:
      break;  // kNotDetected：组件未开启诊断事件，调用方显式跳过，不会到达
  }
  return EosDetectionEventKind::kUpdated;
}

/// 示例事件类型 → 中文名（人读日志）。
const char* EosEventKindName(EosDetectionEventKind kind) {
  switch (kind) {
    case EosDetectionEventKind::kFirstDetected:
      return "首发现";
    case EosDetectionEventKind::kUpdated:
      return "更新";
    case EosDetectionEventKind::kLost:
      return "丢失";
  }
  return "未知";
}

/// 世界 ECEF 目标真值 → 平台锚点 ENU 场景输入（与 AR 同契约：锚点一次 + 逐目标
/// TryMakeEnuSceneState 直填 EosSceneTarget）。
std::vector<electro_optical_sensor::session::EosSceneTarget> BuildEosEnuTargets(
    const std::vector<app::TargetEcefState>& world_targets,
    const oneq::coordinate::EcefPositionM& platform_position_ecef_m) {
  std::vector<electro_optical_sensor::session::EosSceneTarget> targets;
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
    electro_optical_sensor::session::EosSceneTarget target;
    target.target_id = state.id;
    target.position_x = static_cast<float>(enu.position_enu_m.east_m);
    target.position_y = static_cast<float>(enu.position_enu_m.north_m);
    target.position_z = static_cast<float>(enu.position_enu_m.up_m);
    target.velocity_x = static_cast<float>(enu.velocity_enu_mps.east_mps);
    target.velocity_y = static_cast<float>(enu.velocity_enu_mps.north_mps);
    target.velocity_z = static_cast<float>(enu.velocity_enu_mps.up_mps);
    target.appearance.apparent_temperature_k = state.temperature_k;
    target.appearance.emissivity = 0.92f;
    target.appearance.reflectance = 0.35f;
    target.appearance.projected_area_m2 = state.projected_area_m2;
    targets.push_back(target);
  }
  return targets;
}

/// 调试目标状态 → 中文名（人读日志）。
const char* EosTargetStatusName(electro_optical_sensor::session::EosDebugTargetStatus status) {
  switch (status) {
    case electro_optical_sensor::session::EosDebugTargetStatus::kDetected:
      return "已检测";
    case electro_optical_sensor::session::EosDebugTargetStatus::kObservedBelowThreshold:
      return "低于门限";
    case electro_optical_sensor::session::EosDebugTargetStatus::kNotInOutput:
      return "不在输出";
    case electro_optical_sensor::session::EosDebugTargetStatus::kCycleNotExecuted:
      return "周期未执行";
  }
  return "未知";
}

/// 排除主因 → 中文名（人读事件行）。
const char* EosExclusionCauseName(electro_optical_sensor::session::EosIssueCause cause) {
  switch (cause) {
    case electro_optical_sensor::session::EosIssueCause::kNone:
      return "无归因";
    case electro_optical_sensor::session::EosIssueCause::kAzOutside:
      return "方位越界";
    case electro_optical_sensor::session::EosIssueCause::kElOutside:
      return "俯仰越界";
    case electro_optical_sensor::session::EosIssueCause::kBothAxesOutside:
      return "双轴越界";
    case electro_optical_sensor::session::EosIssueCause::kUnknown:
      return "未知主因";
  }
  return "未知主因";
}

}  // namespace

EosSensorComponent::EosSensorComponent(electro_optical_sensor::session::EosSession session,
                                       std::uint32_t device_id,
                                       DetectionDeliveryMode detection_delivery)
    : session_(std::move(session)),
      device_id_(device_id),
      detection_delivery_(detection_delivery) {
  session_.SetSensorEntityId(device_id_);
  // 探测生命周期事件由库内 recorder 承担（StepWithResult 内部自动喂）。
  session_.AttachDetectionLifecycleRecorder(&lifecycle_);
  // 排除原因跨周期差分事件由库内 recorder 承担（与 lifecycle recorder 独立并列）。
  session_.AttachExclusionCauseRecorder(&exclusion_);
}

bool EosSensorComponent::TryApplyRuntimeConfig(
    const electro_optical_sensor::config::EosRuntimeConfigPatch& patch) {
  const bool applied = session_.TryApplyRuntimeConfig(patch);
  if (applied && patch.has_sensor_enabled) {
    powered_on_ = patch.sensor_enabled;  // 电源状态由补丁唯一维护（组件层电源门控）
  }
  return applied;
}

// 视图行写入：三种密度模式编译期三选一（CMake -DCA_VIEW_LOG_MODE=… 或改
// logger/logger_modes.h；未选中的分支不参与编译）。纯观测，不影响会话与信号。
// 各模式输出示例：
//   模式一 nonnominal（只写异常目标行；全部正常时整周期静默不写，防刷屏）：
//     周期=5 目标=1001 状态=不在视场 距离=85000.0m 方位=12.0°
//   模式二 delta（只写状态有变化的目标行；无变化时整周期静默不写，防刷屏）：
//     周期=5 目标=1001 状态=已检测
//   模式三 summary（默认；每周期恰好一行完整摘要）：
//     周期=400 执行=是 目标=[1001 不在输出(方位-0.1° 俯仰6.9° 距离86.3km),
//     1002 不在输出(方位4.0° 俯仰-0.0° 距离88.3km)] 问题=[eos.target_out_of_fov …]
void EosSensorComponent::LogDebugView(
    const electro_optical_sensor::session::EosOutputDebugView& view) {
#if defined(CA_VIEW_LOG_MODE_NONNOMINAL)
  // 模式一：只写非标称目标（非已检测）行；全部正常时本周期静默不写（防刷屏）。
  for (const auto& target : view.targets) {
    if (target.status == electro_optical_sensor::session::EosDebugTargetStatus::kDetected) {
      continue;
    }
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string eos_view_log =
        std::string("周期=") +
        std::to_string(view.input_cycle_index) +
        " 目标=" +
        std::to_string(target.target_id) +
        " 状态=" +
        (EosTargetStatusName(target.status)) +
        " 距离=" +
        std::to_string(target.range_m) +
        "m 方位=" +
        std::to_string(target.azimuth_deg) +
        "°";
    CA_LOG_VIEW("eos", "周期={} 目标={} 状态={} 距离={:.1f}m 方位={:.1f}°",
                view.input_cycle_index, target.target_id,
                EosTargetStatusName(target.status), target.range_m, target.azimuth_deg);
  }
#elif defined(CA_VIEW_LOG_MODE_DELTA)
  // 模式二：只写状态与上一周期不同的目标行（上一周期状态表由组件持有，首次
  // 出现视为变化；表只增不减，示例不清理）；无变化时静默不写（防刷屏）。
  for (const auto& target : view.targets) {
    const auto it = prev_target_status_.find(target.target_id);
    if (it == prev_target_status_.end() || it->second != target.status) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string eos_view_log_3 =
          std::string("周期=") +
          std::to_string(view.input_cycle_index) +
          " 目标=" +
          std::to_string(target.target_id) +
          " 状态=" +
          (EosTargetStatusName(target.status));
    CA_LOG_VIEW("eos", "周期={} 目标={} 状态={}",
                view.input_cycle_index, target.target_id,
                EosTargetStatusName(target.status));
    }
    prev_target_status_[target.target_id] = target.status;
  }
#else  // CA_VIEW_LOG_MODE_SUMMARY（默认）
  // 模式三（每周期摘要行）：目标状态明细带结构化量值（input 回填，未检测也
  // 可见目标角度/距离）+ 问题中文名，一眼可读。
  std::string targets_text;
  for (const auto& target : view.targets) {
    if (!targets_text.empty()) {
      targets_text += ", ";
    }
    targets_text += std::to_string(target.target_id) +
                    std::string(" ") + EosTargetStatusName(target.status) + "(方位" +
                    std::to_string(target.azimuth_deg) + "° 俯仰" +
                    std::to_string(target.elevation_deg) + "° 距离" +
                    std::to_string(target.range_m / 1000.0) + "km)";
  }
  const std::string issues_text = app::FormatIssueText(view.issues);
  // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
  const std::string eos_view_log_5 =
      std::string("周期=") +
      std::to_string(view.input_cycle_index) +
      " 执行=" +
      (view.executed_this_cycle ? "是" : "否") +
      " 目标=[" +
      (targets_text.empty() ? "无" : targets_text.c_str()) +
      "] 问题=[" +
      (issues_text.empty() ? "无" : issues_text.c_str()) +
      "]";
  CA_LOG_VIEW("eos", "周期={} 执行={} 目标=[{}] 问题=[{}]",
              view.input_cycle_index, view.executed_this_cycle ? "是" : "否",
              targets_text.empty() ? "无" : targets_text.c_str(),
              issues_text.empty() ? "无" : issues_text.c_str());
#endif  // CA_VIEW_LOG_MODE_*
}

bool EosSensorComponent::BuildCycleInput(const FlightComponent& flight,
                                          const AppSceneState& scene, double dt_sec,
                                          electro_optical_sensor::session::EosCycleInput* input) const {
  // 平台 ECEF（零姿态：与 AR/ESR 共享同一平台局部坐标系）+ ENU 场景目标直填。
  oneq::coordinate::EcefPositionM platform_ecef;
  oneq::coordinate::EcefVelocityMps platform_vel;
  ResolvePlatformEcef(flight.position(), flight.heading_deg(), flight.speed_mps(),
                      &platform_ecef, &platform_vel);
  oneq::coordinate::LlaPositionDegM platform_lla;
  if (!oneq::coordinate::TryEcefToLla(platform_ecef, &platform_lla)) {
    return false;  // 平台锚点失败：本周期不产生探测
  }

  electro_optical_sensor::session::EosCycleInput& assembled = *input;
  assembled.cycle_index = static_cast<std::uint32_t>(scene.cycle);
  assembled.dt_sec = static_cast<float>(dt_sec);
  assembled.platform_altitude_m = static_cast<float>(platform_lla.altitude_m);
  assembled.platform_attitude_deg = {};
  assembled.scene = BuildEosEnuTargets(scene.world_targets, platform_ecef);

  return true;
}

void EosSensorComponent::Step(World& world, double dt_sec) {
  if (!powered_on_) {
    scan_azimuth_deg_ = 0.0f;  // 关机：不驱动会话，角度无有效值（清零）
    last_debug_view_ = electro_optical_sensor::session::EosOutputDebugView{};  // 关机：调试视图清零（无有效周期）
    LogDebugView(last_debug_view_);
    return;
  }

  const FlightComponent* flight = host_ != nullptr ? host_->Find<FlightComponent>() : nullptr;
  if (flight == nullptr) {
    return;  // 无平台动力学（空场景）：不产生探测
  }

  // 共享场景状态（World 只存基类引用，实际类型为 AppSceneState）：读真值组输入，
  // 也向其探测池写适配记录，故直接取可变引用。
  auto& scene = static_cast<AppSceneState&>(world.scene_state());
  electro_optical_sensor::session::EosCycleInput input;
  if (!BuildCycleInput(*flight, scene, dt_sec, &input)) {
    return;  // 平台锚点失败：本周期不产生探测
  }

  const electro_optical_sensor::session::EosCycleResult result = session_.StepWithResult(input);
  scan_azimuth_deg_ = result.output_frame.scan_azimuth_deg;  // 扫描方位随周期结果刷新（拒绝周期为空帧 → 0）
  // 调试视图快照每周期都要构建：它是下方视图行的数据源（日志写多少由三密度
  // 模式宏门控，与快照构建无关；被拒绝周期为 kCycleNotExecuted 行）。
  last_debug_view_ = electro_optical_sensor::session::EosOutputDebugViewBuilder::Build(input, result);
  LogDebugView(last_debug_view_);
  if (result.status != electro_optical_sensor::session::EosCycleStatus::kCompleted) {
    return;  // 周期被拒绝：本周期无探测
  }
  PublishDetectionEvents(world, scene, result);  // 探测生命周期沿 → 信号+事件日志
  PublishExclusionEvents(world);                 // 排除原因变化沿 → 事件日志
  AdaptDetections(world, scene, result);                // 探测记录 → 共享探测池或消息
}

void EosSensorComponent::PublishDetectionEvents(
    World& world, const AppSceneState& scene,
    const electro_optical_sensor::session::EosCycleResult& result) {
  // 库内 recorder 已按跨周期状态差分产出本周期事件（首发现/更新/丢失）。recorder
  // 事件无方位/探测 ID 字段：非丢失事件从本周期输出帧按归属目标回查；丢失事件
  // 目标不在帧内，字段留默认。kNotDetected 诊断事件未开启，显式跳过（防御）。
  const auto& records = result.output_frame.detections;
  for (const auto& event : lifecycle_.GetLastEvents()) {
    if (event.kind == electro_optical_sensor::session::EosDetectionLifecycleEventKind::kNotDetected) {
      continue;  // 诊断事件（未开启 emit_not_detected_events）：不发布
    }
    EosDetectionEvent eos_event;
    eos_event.cycle = scene.cycle;
    eos_event.kind = ToAppKind(event.kind);
    eos_event.target_id = event.target_id;
    eos_event.snr_db = event.fused_snr_db;
    if (event.kind != electro_optical_sensor::session::EosDetectionLifecycleEventKind::kLost) {
      for (const auto& record : records) {
        if (!record.detected) {
          continue;  // 未过探测门限不参与回查
        }
        if (AttributionTargetId(record.detection_id, result.detection_attributions) ==
            event.target_id) {
          eos_event.detection_id = record.detection_id;
          eos_event.az_deg = record.azimuth_deg;
          break;
        }
      }
    }
    if (eos_event.kind == EosDetectionEventKind::kUpdated) {
      // 更新类事件每周期重复：事件模式一下不落盘（信号照常发布）。
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string eos_detection_event_log =
          std::string("类型=") +
          (EosEventKindName(eos_event.kind)) +
          " 探测ID=" +
          std::to_string(static_cast<unsigned long long>(eos_event.detection_id)) +
          " 目标=" +
          std::to_string(static_cast<unsigned long long>(eos_event.target_id)) +
          " 信噪比=" +
          std::to_string(eos_event.snr_db) +
          "dB 方位=" +
          std::to_string(eos_event.az_deg) +
          "°";
      CA_LOG_EVENT_DUP(world, "eos_detection", "类型={} 探测ID={} 目标={} 信噪比={:.1f}dB 方位={:.1f}°",
                       EosEventKindName(eos_event.kind),
                       static_cast<unsigned long long>(eos_event.detection_id),
                       static_cast<unsigned long long>(eos_event.target_id), eos_event.snr_db,
                       eos_event.az_deg);
    } else {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string eos_detection_event_log_2 =
          std::string("类型=") +
          (EosEventKindName(eos_event.kind)) +
          " 探测ID=" +
          std::to_string(static_cast<unsigned long long>(eos_event.detection_id)) +
          " 目标=" +
          std::to_string(static_cast<unsigned long long>(eos_event.target_id)) +
          " 信噪比=" +
          std::to_string(eos_event.snr_db) +
          "dB 方位=" +
          std::to_string(eos_event.az_deg) +
          "°";
      CA_LOG_EVENT(world, "eos_detection", "类型={} 探测ID={} 目标={} 信噪比={:.1f}dB 方位={:.1f}°",
                   EosEventKindName(eos_event.kind),
                   static_cast<unsigned long long>(eos_event.detection_id),
                   static_cast<unsigned long long>(eos_event.target_id), eos_event.snr_db,
                   eos_event.az_deg);
    }
    world.signals().on_eos_detection(eos_event);
  }

}

void EosSensorComponent::PublishExclusionEvents(World& world) {
  // 排除原因跨周期差分沿（进入/变化/退出）→ 仅事件日志，不发 World 信号；
  // 原因稳定不产事件，故无刷屏。EOS 单一视场门排除（eos.target_out_of_fov），
  // A3 由越界轴变化（az/el/both）驱动。
  for (const auto& event : exclusion_.GetLastEvents()) {
    const std::uint64_t event_target_id = event.target_id;
    if (event.kind == electro_optical_sensor::session::EosExclusionCauseEventKind::kEntered) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log =
          std::string("目标=") +
          std::to_string(static_cast<unsigned long long>(event_target_id)) +
          " 类型=进入排除 排除码=" +
          (event.current_code) +
          " 主因=" +
          (EosExclusionCauseName(event.current_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "目标={} 类型=进入排除 排除码={} 主因={}",
                   static_cast<unsigned long long>(event_target_id),
                   event.current_code, EosExclusionCauseName(event.current_cause));
    } else if (event.kind == electro_optical_sensor::session::EosExclusionCauseEventKind::kChanged) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log_2 =
          std::string("目标=") +
          std::to_string(static_cast<unsigned long long>(event_target_id)) +
          " 类型=原因变化 旧码=" +
          (event.previous_code) +
          " 旧主因=" +
          (EosExclusionCauseName(event.previous_cause)) +
          " 新码=" +
          (event.current_code) +
          " 新主因=" +
          (EosExclusionCauseName(event.current_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "目标={} 类型=原因变化 旧码={} 旧主因={} 新码={} 新主因={}",
                   static_cast<unsigned long long>(event_target_id),
                   event.previous_code, EosExclusionCauseName(event.previous_cause),
                   event.current_code, EosExclusionCauseName(event.current_cause));
    } else if (event.kind == electro_optical_sensor::session::EosExclusionCauseEventKind::kExited) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log_3 =
          std::string("目标=") +
          std::to_string(static_cast<unsigned long long>(event_target_id)) +
          " 类型=退出排除 旧码=" +
          (event.previous_code) +
          " 旧主因=" +
          (EosExclusionCauseName(event.previous_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "目标={} 类型=退出排除 旧码={} 旧主因={}",
                   static_cast<unsigned long long>(event_target_id),
                   event.previous_code, EosExclusionCauseName(event.previous_cause));
    }
  }

}

void EosSensorComponent::AdaptDetections(
    World& world, AppSceneState& scene,
    const electro_optical_sensor::session::EosCycleResult& result) {
  const auto& detections = result.output_frame.detections;
  const std::vector<fusion::DetectionRecord> records =
      fusion::AdaptEosDetectionsToDetectionRecords(device_id_, detections);
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
