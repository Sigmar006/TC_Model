/**
 * @file ecm_sensor_component.cpp
 * @brief ECM 组件实现（ESR 假设驱动 + RF-WORLD 发布）。
 *
 * 订阅 ESR 假设集快照事件（on_esr_scan_updated，每成功周期全量去真值化
 * 假设 + batch_id；按 batch_id 防重复消费），本地重建库类型喂 sensor-driven
 * 适配器——不调用 ESR 组件方法（集成方同走事件机制）。调度压制/欺骗干扰，
 * 成功执行时将 emission_frame 发布到共享 RF 世界（供 AR/ESR/RIR 消费）；
 * 须挂载在 ESR 之后、AR 之前（挂载序 = 步进序）。
 */

#include "ecm_sensor_component.h"

#include "1q/electronic_countermeasure/EcmEsrAdapter.h"
#include "1q/electronic_surveillance_radar/session/EmitterObservation.h"
#include "core/world.h"
#include "flight_component.h"
#include "logger/logger.h"
#include "core/rf_world_broker.h"
#include "core/scene_types.h"
#include "components/sensor_utils.h"

namespace component_attachment {

namespace ecm = electronic_countermeasure;

namespace {

const char* EcmCycleStatusName(ecm::session::EcmCycleStatus status) {
  switch (status) {
    case ecm::session::EcmCycleStatus::kExecuted:
      return "已执行";
    case ecm::session::EcmCycleStatus::kSafeStopNoFreshObservation:
      return "无新观测停发";
    case ecm::session::EcmCycleStatus::kPoweredOff:
      return "关机";
    case ecm::session::EcmCycleStatus::kRejectedInvalidInput:
      return "输入拒绝";
    case ecm::session::EcmCycleStatus::kRejectedInvalidConfig:
      return "配置拒绝";
  }
  return "未知";
}

const char* EcmTechniqueName(ecm::EcmTechnique technique) {
  switch (technique) {
    case ecm::EcmTechnique::kSpot:
      return "点频";
    case ecm::EcmTechnique::kBarrage:
      return "阻塞";
    case ecm::EcmTechnique::kSweep:
      return "扫频";
    case ecm::EcmTechnique::kDeception:
      return "欺骗";
  }
  return "未知";
}

}  // namespace

EcmSensorComponent::EcmSensorComponent(ecm::session::EcmSession session,
                                       std::uint32_t device_id)
    : session_(std::move(session)), device_id_(device_id) {
  session_.SetSensorEntityId(device_id_);
}

void EcmSensorComponent::OnEsrScanUpdated(const EsrScanUpdatedEvent& event) {
  esr_scan_ = event;
}

void EcmSensorComponent::EnsureSignalConnections(World& world) {
  if (!esr_connection_.connected()) {
    esr_connection_ = world.signals().on_esr_scan_updated.connect(
        [this](const EsrScanUpdatedEvent& event) { OnEsrScanUpdated(event); });
  }
}

void EcmSensorComponent::Step(World& world, double dt_sec) {
  if (!powered_on_) {
    return;
  }

  EnsureSignalConnections(world);

  const FlightComponent* flight = host_ != nullptr ? host_->Find<FlightComponent>() : nullptr;
  if (flight == nullptr) {
    return;  // 无平台动力学（空场景）：不发射
  }

  // 共享场景状态（World 只存基类引用，实际类型为 AppSceneState）：读真值组输入，
  // 也向其写回射频发射，故直接取可变引用。
  auto& scene = static_cast<AppSceneState&>(world.scene_state());

  ecm::session::EcmCycleInput input;
  input.cycle_index = static_cast<std::uint32_t>(scene.cycle);
  input.cycle_start_time_s = scene.t_sec;
  input.dt_sec = dt_sec;
  input.input_mode = ecm::EcmInputMode::kSensorDriven;
  ResolvePlatformEcef(flight->position(), flight->heading_deg(), flight->speed_mps(),
                      &input.platform_position_ecef_m, &input.platform_velocity_ecef_mps);

  // ESR 假设集事件缓存（ESR 挂载序在前，同周期事件先于本组件 Step 到达）：
  // 本周期新鲜 + 批次递增才消费；事件展平字段在此重建库类型喂适配器
  // （防腐层归消费方）。
  if (esr_scan_.batch_id != 0U &&
      esr_scan_.cycle == scene.cycle &&
      esr_scan_.batch_id > last_submitted_esr_batch_id_) {
    electronic_surveillance_radar::session::EmitterHypothesisList hypotheses;
    hypotheses.reserve(esr_scan_.hypotheses.size());
    for (const auto& data : esr_scan_.hypotheses) {
      electronic_surveillance_radar::session::EmitterHypothesis hypothesis;
      hypothesis.hypothesis_id = data.hypothesis_id;
      hypothesis.candidate_classes = data.candidate_classes;
      hypothesis.mode = static_cast<electronic_surveillance_radar::session::EsrEmitterMode>(
          data.mode);
      hypothesis.threat_level =
          static_cast<electronic_surveillance_radar::session::EsrThreatLevel>(
              data.threat_level);
      hypothesis.bearing_az_deg = data.bearing_az_deg;
      hypothesis.bearing_el_deg = data.bearing_el_deg;
      hypothesis.bearing_std_deg = data.bearing_std_deg;
      hypothesis.estimated_center_frequency_hz = data.estimated_center_frequency_hz;
      hypothesis.estimated_bandwidth_hz = data.estimated_bandwidth_hz;
      hypothesis.estimated_pri_s = data.estimated_pri_s;
      hypothesis.estimated_pulse_width_s = data.estimated_pulse_width_s;
      hypothesis.center_frequency_std_hz = data.center_frequency_std_hz;
      hypothesis.bandwidth_std_hz = data.bandwidth_std_hz;
      hypothesis.pri_std_s = data.pri_std_s;
      hypothesis.pulse_width_std_s = data.pulse_width_std_s;
      hypothesis.confidence = data.confidence;
      hypothesis.last_seen_cycle = data.last_seen_cycle;
      hypothesis.waveform_class =
          static_cast<electronic_surveillance_radar::session::EsrWaveformClass>(
              data.waveform_class);
      hypotheses.push_back(hypothesis);
    }
    ecm::session::EcmSensorObservationFrame sensor_frame;
    if (ecm::session::TryBuildEcmSensorObservationFrame(hypotheses, esr_scan_.batch_id,
                                                        &sensor_frame)) {
      input.has_sensor_observation_frame = true;
      input.sensor_observation_frame = sensor_frame;
      last_submitted_esr_batch_id_ = esr_scan_.batch_id;
    }
  }

  const ecm::session::EcmCycleResult result = session_.StepWithResult(input);
  // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
  const std::string ecm_view_log =
      std::string("周期=") +
      std::to_string(result.input_cycle_index) +
      " 状态=" +
      (EcmCycleStatusName(result.status)) +
      " 发射数=" +
      std::to_string(result.emission_frame.emissions.size()) +
      " ESR批次=" +
      std::to_string(static_cast<unsigned long long>(result.source_esr_batch_id));
  CA_LOG_VIEW("ecm", "周期={} 状态={} 发射数={} ESR批次={}",
              result.input_cycle_index, EcmCycleStatusName(result.status),
              result.emission_frame.emissions.size(),
              static_cast<unsigned long long>(result.source_esr_batch_id));

  if (result.status == ecm::session::EcmCycleStatus::kExecuted) {
    ++executed_cycle_count_;
    if (!result.decisions.empty()) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string ecm_jamming_event_log =
          std::string("技术=") +
          (EcmTechniqueName(result.decisions.front().technique)) +
          " 决策数=" +
          std::to_string(result.decisions.size()) +
          " 功率=" +
          std::to_string(result.decisions.front().allocated_power_w) +
          "W";
      CA_LOG_EVENT(world, "ecm_jamming",
                   "技术={} 决策数={} 功率={:.0f}W",
                   EcmTechniqueName(result.decisions.front().technique),
                   result.decisions.size(), result.decisions.front().allocated_power_w);
    }
    PublishEquipmentEmissions(&scene, result.emission_frame);  // 射频发射 → 共享 RF 世界
  }
}

}  // namespace component_attachment
