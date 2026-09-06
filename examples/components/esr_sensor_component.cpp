/**
 * @file esr_sensor_component.cpp
 * @brief ESR 传感器组件实现（会话驱动 + 假设事件发布）。
 *
 * 1. EsrCycleInput 直接构造（平台 ECEF 运动学 + 消费方注入的辐射源真值帧），
 *    驱动 EsrSession，输出假设适配为泛型探测记录（fusion::AdaptEsrHypothesesToDetectionRecords）；
 * 2. 每条假设（键 ≠ 0）经 World 信号发布 EmitterHypothesisEvent；
 * 3. 事件直写集成端日志（CA_LOG_EVENT，中文人读行）。
 */

#include "esr_sensor_component.h"

#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/fusion/SensorAdapters.h"
#include "core/events.h"
#include "core/fusion_detection_bridge.h"
#include "logger/logger.h"
#include "flight_component.h"
#include "core/world.h"
#include "core/scene_types.h"
#include "components/sensor_utils.h"

namespace component_attachment {

namespace {

/// 排除主因 → 中文名（人读事件行）。
const char* EsrExclusionCauseName(electronic_surveillance_radar::session::EsrIssueCause cause) {
  switch (cause) {
    case electronic_surveillance_radar::session::EsrIssueCause::kNone:
      return "无归因";
    case electronic_surveillance_radar::session::EsrIssueCause::kTransmitSilent:
      return "发射静默";
    case electronic_surveillance_radar::session::EsrIssueCause::kOverlapWindow:
      return "时频重叠为零";
    case electronic_surveillance_radar::session::EsrIssueCause::kPropagationLoss:
      return "传播损耗";
    case electronic_surveillance_radar::session::EsrIssueCause::kHardGateFailed:
      return "硬门失败";
    case electronic_surveillance_radar::session::EsrIssueCause::kStatisticalGateFailed:
      return "统计门失败";
    case electronic_surveillance_radar::session::EsrIssueCause::kUnknown:
      return "未知主因";
  }
  return "未知主因";
}

}  // namespace

EsrSensorComponent::EsrSensorComponent(
    electronic_surveillance_radar::session::EsrSession session,
    std::uint32_t device_id,
    DetectionDeliveryMode detection_delivery)
    : session_(std::move(session)),
      device_id_(device_id),
      detection_delivery_(detection_delivery) {
  session_.SetSensorEntityId(device_id_);
  // 排除原因跨周期差分事件由库内 recorder 承担（StepWithResult 内部自动喂）。
  // ESR 无既有 lifecycle recorder，exclusion_ 为首个 recorder 成员。
  session_.AttachExclusionCauseRecorder(&exclusion_);
}

bool EsrSensorComponent::TryApplyRuntimeConfig(
    const electronic_surveillance_radar::config::EsrRuntimeConfigPatch& patch) {
  const bool applied = session_.TryApplyRuntimeConfig(patch);
  if (applied && patch.has_sensor_enabled) {
    powered_on_ = patch.sensor_enabled;  // 电源状态由补丁唯一维护（组件层电源门控）
  }
  return applied;
}

electronic_surveillance_radar::session::EsrRuntimeConfigApplyResult
EsrSensorComponent::ApplyRuntimeConfigWithResult(
    const electronic_surveillance_radar::config::EsrRuntimeConfigPatch& patch) {
  const auto result = session_.ApplyRuntimeConfigWithResult(patch);
  if (result.applied && patch.has_sensor_enabled) {
    powered_on_ = patch.sensor_enabled;  // 电源状态由补丁唯一维护（组件层电源门控）
  }
  return result;
}

electronic_surveillance_radar::session::EsrCycleInput EsrSensorComponent::BuildCycleInput(
    const FlightComponent& flight, const AppSceneState& scene, double dt_sec) const {
  electronic_surveillance_radar::session::EsrCycleInput input;
  input.cycle_index = static_cast<std::uint32_t>(scene.cycle);
  input.cycle_start_time_s = scene.t_sec;
  input.dt_sec = static_cast<float>(dt_sec);
  input.platform_entity_id = 1U;  // 平台实体标识（本示例单平台）
  ResolvePlatformEcef(flight.position(), flight.heading_deg(), flight.speed_mps(),
                      &input.platform_position_ecef_m, &input.platform_velocity_ecef_mps);
  // RF 场景包络必须与本周期权威时间一致（含空帧亦须填齐）。
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  input.rf_emissions.emissions = scene.rf_world.emissions;  // RF-WORLD（脚本源 + 装备发射）

  return input;
}

void EsrSensorComponent::Step(World& world, double dt_sec) {
  if (!powered_on_) {
    scan_azimuth_deg_ = 0.0f;  // 关机：不驱动会话，角度无有效值（清零）
    return;
  }

  const FlightComponent* flight = host_ != nullptr ? host_->Find<FlightComponent>() : nullptr;
  if (flight == nullptr) {
    return;  // 无平台动力学（空场景）：不产生探测
  }

  // 共享场景状态（World 只存基类引用，实际类型为 AppSceneState）：读真值组
  // 输入，也向其探测池写适配记录，故直接取可变引用。
  auto& scene = static_cast<AppSceneState&>(world.scene_state());
  const electronic_surveillance_radar::session::EsrCycleInput input =
      BuildCycleInput(*flight, scene, dt_sec);

  const electronic_surveillance_radar::session::EsrCycleResult result = session_.StepWithResult(input);
  scan_azimuth_deg_ = result.output_frame.scan_azimuth_deg;  // 扫描方位随周期结果刷新（拒绝周期为空帧 → 0）
  if (result.status !=
      electronic_surveillance_radar::session::EsrCycleExecutionStatus::kCompleted) {
    return;  // 周期被拒绝：本周期无假设
  }

  last_hypotheses_ = result.output_frame.emitter_output.hypotheses;
  last_batch_id_ = result.output_frame.batch_id;
  last_completed_cycle_index_ = static_cast<std::uint32_t>(scene.cycle);
  has_last_completed_output_ = true;

  PublishHypothesisEvents(world, scene);  // 辐射源假设沿 → 信号+事件日志（周期性重复）
  PublishScanEvent(world, scene);         // 假设集快照 → 信号（ECM sensor-driven 输入）
  PublishExclusionEvents(world);          // 排除原因变化沿 → 事件日志
  AdaptDetections(world, scene);                 // 假设 → 共享探测池或消息
}

void EsrSensorComponent::PublishHypothesisEvents(World& world,
                                                 const AppSceneState& scene) {
  const auto& hypotheses = last_hypotheses_;
  for (const auto& hypothesis : hypotheses) {
    if (hypothesis.hypothesis_id == 0U) {
      continue;  // 库内键 0 = 无身份，不发布事件
    }
    EmitterHypothesisEvent event;
    event.cycle = scene.cycle;
    event.hypothesis_id = hypothesis.hypothesis_id;
    event.bearing_az_deg = hypothesis.bearing_az_deg;
    event.confidence = hypothesis.confidence;
    event.mode = static_cast<EventEmitterMode>(hypothesis.mode);
    event.threat_level = static_cast<EventThreatLevel>(hypothesis.threat_level);
    // 假设事件每周期重复（目标恒在时）：事件模式一下不落盘（信号照常发布）。
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string emitter_hypothesis_event_log =
        std::string("假设=") +
        std::to_string(static_cast<unsigned long long>(event.hypothesis_id)) +
        " 方位=" +
        std::to_string(event.bearing_az_deg) +
        "° 置信=" +
        std::to_string(event.confidence) +
        " 模式=" +
        std::to_string(static_cast<int>(event.mode)) +
        " 威胁=" +
        std::to_string(static_cast<int>(event.threat_level));
    CA_LOG_EVENT_DUP(world, "emitter_hypothesis",
                     "假设={} 方位={:.1f}° 置信={:.2f} 模式={} 威胁={}",
                     static_cast<unsigned long long>(event.hypothesis_id),
                     event.bearing_az_deg, event.confidence,
                     static_cast<int>(event.mode), static_cast<int>(event.threat_level));
    world.signals().on_emitter_hypothesis(event);
  }

}

void EsrSensorComponent::PublishExclusionEvents(World& world) {
  // 排除原因跨周期差分沿（进入/变化/退出）→ 仅事件日志，不发 World 信号；
  // 原因稳定不产事件，故无刷屏。ESR 无 target_id，以发射源标识
  // （platform/equipment/emission 三元组）为实体关联键。差分键为 (code,cause)
  // 组合对——co-site↔zero-power 切换（kNone→细分 cause）亦产事件。
  for (const auto& event : exclusion_.GetLastEvents()) {
    const auto& id = event.identity;
    if (event.kind == electronic_surveillance_radar::session::EsrExclusionCauseEventKind::kEntered) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log =
          std::string("辐射源=(platform=") +
          std::to_string(static_cast<unsigned long long>(id.platform_id)) +
          ",equipment=" +
          std::to_string(static_cast<unsigned long long>(id.equipment_id)) +
          ",emission=" +
          std::to_string(static_cast<unsigned long long>(id.emission_id)) +
          ") 类型=进入排除 排除码=" +
          (event.current_code) +
          " 主因=" +
          (EsrExclusionCauseName(event.current_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "辐射源=(platform={},equipment={},emission={}) 类型=进入排除 排除码={} 主因={}",
                   static_cast<unsigned long long>(id.platform_id),
                   static_cast<unsigned long long>(id.equipment_id),
                   static_cast<unsigned long long>(id.emission_id),
                   event.current_code, EsrExclusionCauseName(event.current_cause));
    } else if (event.kind == electronic_surveillance_radar::session::EsrExclusionCauseEventKind::kChanged) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log_2 =
          std::string("辐射源=(platform=") +
          std::to_string(static_cast<unsigned long long>(id.platform_id)) +
          ",equipment=" +
          std::to_string(static_cast<unsigned long long>(id.equipment_id)) +
          ",emission=" +
          std::to_string(static_cast<unsigned long long>(id.emission_id)) +
          ") 类型=原因变化 旧码=" +
          (event.previous_code) +
          " 旧主因=" +
          (EsrExclusionCauseName(event.previous_cause)) +
          " 新码=" +
          (event.current_code) +
          " 新主因=" +
          (EsrExclusionCauseName(event.current_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "辐射源=(platform={},equipment={},emission={}) 类型=原因变化 旧码={} 旧主因={} 新码={} 新主因={}",
                   static_cast<unsigned long long>(id.platform_id),
                   static_cast<unsigned long long>(id.equipment_id),
                   static_cast<unsigned long long>(id.emission_id),
                   event.previous_code, EsrExclusionCauseName(event.previous_cause),
                   event.current_code, EsrExclusionCauseName(event.current_cause));
    } else if (event.kind == electronic_surveillance_radar::session::EsrExclusionCauseEventKind::kExited) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log_3 =
          std::string("辐射源=(platform=") +
          std::to_string(static_cast<unsigned long long>(id.platform_id)) +
          ",equipment=" +
          std::to_string(static_cast<unsigned long long>(id.equipment_id)) +
          ",emission=" +
          std::to_string(static_cast<unsigned long long>(id.emission_id)) +
          ") 类型=退出排除 旧码=" +
          (event.previous_code) +
          " 旧主因=" +
          (EsrExclusionCauseName(event.previous_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "辐射源=(platform={},equipment={},emission={}) 类型=退出排除 旧码={} 旧主因={}",
                   static_cast<unsigned long long>(id.platform_id),
                   static_cast<unsigned long long>(id.equipment_id),
                   static_cast<unsigned long long>(id.emission_id),
                   event.previous_code, EsrExclusionCauseName(event.previous_cause));
    }
  }

}

void EsrSensorComponent::PublishScanEvent(World& world, const AppSceneState& scene) {
  // 假设集快照事件（每成功周期全量）：sensor-driven 消费方（ECM）订阅本事件
  // 取当前假设集与批次号，不依赖 ESR 组件方法。库内枚举/结构展平为事件
  // 契约的镜像枚举/数值字段；消费方需要库类型时在本地重建（防腐层归消费方）。
  EsrScanUpdatedEvent event;
  event.cycle = scene.cycle;
  event.batch_id = last_batch_id_;
  event.hypotheses.reserve(last_hypotheses_.size());
  for (const auto& hypothesis : last_hypotheses_) {
    EsrHypothesisData data;
    data.hypothesis_id = hypothesis.hypothesis_id;
    data.candidate_classes = hypothesis.candidate_classes;
    data.mode = static_cast<EventEmitterMode>(hypothesis.mode);
    data.threat_level = static_cast<EventThreatLevel>(hypothesis.threat_level);
    data.bearing_az_deg = hypothesis.bearing_az_deg;
    data.bearing_el_deg = hypothesis.bearing_el_deg;
    data.bearing_std_deg = hypothesis.bearing_std_deg;
    data.estimated_center_frequency_hz = hypothesis.estimated_center_frequency_hz;
    data.estimated_bandwidth_hz = hypothesis.estimated_bandwidth_hz;
    data.estimated_pri_s = hypothesis.estimated_pri_s;
    data.estimated_pulse_width_s = hypothesis.estimated_pulse_width_s;
    data.center_frequency_std_hz = hypothesis.center_frequency_std_hz;
    data.bandwidth_std_hz = hypothesis.bandwidth_std_hz;
    data.pri_std_s = hypothesis.pri_std_s;
    data.pulse_width_std_s = hypothesis.pulse_width_std_s;
    data.confidence = hypothesis.confidence;
    data.last_seen_cycle = hypothesis.last_seen_cycle;
    data.waveform_class = static_cast<std::uint8_t>(hypothesis.waveform_class);
    event.hypotheses.push_back(data);
  }
  world.signals().on_esr_scan_updated(event);
}

void EsrSensorComponent::AdaptDetections(World& world, AppSceneState& scene) {
  const std::vector<fusion::DetectionRecord> records =
      fusion::AdaptEsrHypothesesToDetectionRecords(device_id_, last_hypotheses_);
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
