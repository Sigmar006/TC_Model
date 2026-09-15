/**
 * @file sbirs_sensor_component.cpp
 * @brief SBIRS 传感器组件实现（会话驱动 + 探测生命周期事件转发）。
 *
 * 1. 从共享场景状态读取卫星 ECEF 位置与红外目标真值，构建 SbirsCycleInput
 *    驱动 SbirsSession（天基平台方位参考系与机载通道不同，见 README 简化声明）；
 * 2. 探测事件（首发现/更新/coasting/丢失）由库内 SbirsDetectionLifecycleRecorder
 *    差分产出，组件经归属映射关联目标 ID 后发布 SbirsDetectionEvent；
 * 3. 事件与调试视图直写集成端日志（CA_LOG_EVENT / CA_LOG_VIEW，中文人读行）。
 */

#include "sbirs_sensor_component.h"

#include <chrono>

#include "1q/fusion/SensorAdapters.h"
#include "logger/acceptance_timing.h"
#include "core/events.h"
#include "core/world.h"
#include "logger/logger.h"
#include "logger/logger_i18n.h"
#include "flight_component.h"
#include "core/scene_types.h"

namespace component_attachment {

namespace {

// 库内角度换算工具（common/numerics/Constants.h）随 src/ PRIVATE 包含，示例层不可达；
// 以本地命名常量替代内联魔法数（库内 rad 输出 → 示例 deg 显示的唯一换算点）。
constexpr float kRadToDeg = 57.29577951308232f;

/// 生命周期事件类型 → 示例事件类型（kNotDetected 诊断事件不转发）。
SbirsDetectionEventKind ToAppKind(
    sbirs_sensor::session::SbirsDetectionLifecycleEventKind kind) {
  switch (kind) {
    case sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kFirstDetected:
      return SbirsDetectionEventKind::kFirstDetected;
    case sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kUpdated:
      return SbirsDetectionEventKind::kUpdated;
    case sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kCoasting:
      return SbirsDetectionEventKind::kCoasting;
    case sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kLost:
      return SbirsDetectionEventKind::kLost;
    default:
      break;  // kNotDetected：组件未开启诊断事件，调用方显式跳过，不会到达
  }
  return SbirsDetectionEventKind::kUpdated;
}

/// 示例事件类型 → 中文名（人读日志）。
const char* SbirsEventKindName(SbirsDetectionEventKind kind) {
  switch (kind) {
    case SbirsDetectionEventKind::kFirstDetected:
      return "首发现";
    case SbirsDetectionEventKind::kUpdated:
      return "更新";
    case SbirsDetectionEventKind::kCoasting:
      return "跟踪锁定";
    case SbirsDetectionEventKind::kLost:
      return "丢失";
  }
  return "未知";
}

/// 库内丢失原因 → 示例事件原因（细分透出：区分目标真消失/扫描间隙/调度跳过/
/// 门失败，避免消费方把扫描间隙误读为目标丢失）。
SbirsDetectionLossReason ToAppLossReason(
    sbirs_sensor::session::SbirsDetectionLifecycleReason reason) {
  switch (reason) {
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kOutOfFieldOfView:
      return SbirsDetectionLossReason::kOutOfFieldOfView;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kBelowSnrThreshold:
      return SbirsDetectionLossReason::kBelowSnrThreshold;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kTargetMissingFromInput:
      return SbirsDetectionLossReason::kTargetMissingFromInput;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kNfovAcquisitionFailed:
      return SbirsDetectionLossReason::kAcquisitionFailed;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kSchedulerSkipped:
      return SbirsDetectionLossReason::kSchedulerSkipped;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kEstimationNisGateLost:
      return SbirsDetectionLossReason::kNisGateLost;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kNfovPointingTimeout:
      return SbirsDetectionLossReason::kPointingTimeout;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kNfovTrackingGateLost:
      return SbirsDetectionLossReason::kTrackingGateLost;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kUnknown:
      return SbirsDetectionLossReason::kUnknown;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kNone:
      break;
  }
  return SbirsDetectionLossReason::kNone;
}

/// 丢失原因 → 中文名（人读日志）。
const char* LossReasonName(SbirsDetectionLossReason reason) {
  switch (reason) {
    case SbirsDetectionLossReason::kOutOfFieldOfView:
      return "视场外";
    case SbirsDetectionLossReason::kBelowSnrThreshold:
      return "低于信噪比门限";
    case SbirsDetectionLossReason::kTargetMissingFromInput:
      return "目标消失";
    case SbirsDetectionLossReason::kAcquisitionFailed:
      return "捕获失败";
    case SbirsDetectionLossReason::kSchedulerSkipped:
      return "调度跳过";
    case SbirsDetectionLossReason::kNisGateLost:
      return "NIS 超限丢锁";
    case SbirsDetectionLossReason::kPointingTimeout:
      return "指向超时";
    case SbirsDetectionLossReason::kTrackingGateLost:
      return "跟踪门连续失败";
    case SbirsDetectionLossReason::kUnknown:
      return "未知";
    case SbirsDetectionLossReason::kNone:
      break;
  }
  return "无";
}

/// 检测记录 ID → 仿真目标 ID（无归属返回 0）。
std::uint64_t AttributionTargetId(
    std::uint64_t detection_id,
    const sbirs_sensor::attribution::SbirsDetectionAttributionRecordList& attributions) {
  for (const auto& attribution : attributions) {
    if (attribution.detection_id == detection_id) {
      return attribution.target_id;
    }
  }
  return 0U;  // 无归属（调试视图缺失）：未知目标
}

/// 调试目标状态 → 中文名（人读日志）。
const char* SbirsTargetStatusName(sbirs_sensor::session::SbirsDebugTargetStatus status) {
  switch (status) {
    case sbirs_sensor::session::SbirsDebugTargetStatus::kDetected:
      return "已检测";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kObservedBelowThreshold:
      return "低于门限";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kCoasting:
      return "跟踪锁定";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kNotInOutput:
      return "不在输出";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kCycleNotExecuted:
      return "周期未执行";
  }
  return "未知";
}

/// 排除主因 → 中文名（人读事件行）。
const char* SbirsExclusionCauseName(sbirs_sensor::session::SbirsIssueCause cause) {
  switch (cause) {
    case sbirs_sensor::session::SbirsIssueCause::kNone:
      return "无归因";
    case sbirs_sensor::session::SbirsIssueCause::kAzOutside:
      return "方位越界";
    case sbirs_sensor::session::SbirsIssueCause::kElOutside:
      return "俯仰越界";
    case sbirs_sensor::session::SbirsIssueCause::kBothAxesOutside:
      return "双轴越界";
    case sbirs_sensor::session::SbirsIssueCause::kDistanceLimited:
      return "距离受限";
    case sbirs_sensor::session::SbirsIssueCause::kAttenuationLimited:
      return "大气衰减";
    case sbirs_sensor::session::SbirsIssueCause::kSignatureLimited:
      return "签名受限";
    case sbirs_sensor::session::SbirsIssueCause::kNoiseLimited:
      return "噪声底受限";
    case sbirs_sensor::session::SbirsIssueCause::kUnknown:
      return "未知主因";
  }
  return "未知主因";
}

/// [0,1] 夹取（质量归一化用；与库内 SensorAdapters 同口径）。
double Clamp01(double value) {
  if (value < 0.0) {
    return 0.0;
  }
  if (value > 1.0) {
    return 1.0;
  }
  return value;
}

/// SBIRS 过门探测 → 示例融合探测样本（消息路径；字段同库内默认适配器）。
FusionDetectionSample ToFusionDetectionSample(
    const sbirs_sensor::output::SbirsDetectionRecord& record) {
  FusionDetectionSample sample;
  sample.key = 0U;
  sample.source_id = fusion::kSbirsSourceId;
  sample.has_bearing = true;
  sample.bearing_az_deg = record.azimuth_rad * kRadToDeg;
  sample.bearing_el_deg = record.elevation_rad * kRadToDeg;
  sample.verdict = 1.0;
  sample.quality = Clamp01(static_cast<double>(record.infrared_snr_linear) / 4.0);
  return sample;
}

}  // namespace

// session：调用方构造后移入；示例见 main 卫星 Attach（scene.config → SbirsSession::Create）。
SbirsSensorComponent::SbirsSensorComponent(
    sbirs_sensor::session::SbirsSession session,
    std::uint32_t device_id,
    DetectionDeliveryMode detection_delivery)
    : session_(std::move(session)),
      device_id_(device_id),
      detection_delivery_(detection_delivery) {
  // 探测生命周期事件由库内 recorder 承担（StepWithResult 内部自动喂）。
  session_.AttachDetectionLifecycleRecorder(&lifecycle_);
  // 排除原因跨周期差分事件由库内 recorder 承担（与 lifecycle recorder 独立并列）。
  session_.AttachExclusionCauseRecorder(&exclusion_);
}

// session / source_id / 星历姿态：main 挂载前从 scene.config 与 ephemeris 段组装后移入
// （见 sbirs_triple_sat_fix_messages/main.cpp 卫星 Attach）。
SbirsSensorComponent::SbirsSensorComponent(
    sbirs_sensor::session::SbirsSession session, std::uint32_t ground_station_source_id,
    sbirs_sensor::session::SbirsVector3M position_ecef_m,
    sbirs_sensor::session::SbirsVector3M velocity_ecef_m_per_s,
    sbirs_sensor::session::SbirsEulerAnglesDeg attitude_eci_body_deg,
    SbirsGroundDeliveryMode ground_delivery)
    : session_(std::move(session)),
      ground_station_source_id_(ground_station_source_id),
      ground_delivery_(ground_delivery),
      use_own_satellite_pose_(true),
      own_position_ecef_m_(position_ecef_m),
      own_velocity_ecef_m_per_s_(velocity_ecef_m_per_s),
      own_attitude_eci_body_deg_(attitude_eci_body_deg) {
  session_.AttachDetectionLifecycleRecorder(&lifecycle_);
  session_.AttachExclusionCauseRecorder(&exclusion_);
  // 验收行卫星标注（验收判定标准 第6/7/8/…项 卫星ID=/相对卫星ID=）：卫星实体用
  // 融合源通道 ID（双星场景互异，如 4 与 104），与探测帧投递地面站的源同号。
  if (ground_station_source_id_ != 0U) {
    session_.SetSatelliteEntityId(ground_station_source_id_);
  }
}

bool SbirsSensorComponent::TryApplyRuntimeConfig(
    const sbirs_sensor::config::SbirsRuntimeConfigPatch& patch) {
  const bool applied = session_.TryApplyRuntimeConfig(patch);
  if (applied && patch.has_sensor_enabled) {
    powered_on_ = patch.sensor_enabled;  // 电源状态由补丁唯一维护（组件层电源门控）
  }
  return applied;
}

void SbirsSensorComponent::OnAttach(Entity& host) {
  host_ = &host;
  // 单站挂载（无融合源 ID）时以 device_id_（默认 fusion::kSbirsSourceId）标注验收行 卫星ID=（卫星变体构造时
  // 已按融合源 ID 标注，此处不覆盖）。
  if (ground_station_source_id_ == 0U) {
    session_.SetSatelliteEntityId(device_id_);
  }
}

// 视图行写入：三种密度模式编译期三选一（CMake -DCA_VIEW_LOG_MODE=… 或改
// logger/logger_modes.h；未选中的分支不参与编译）。纯观测，不影响会话与信号。
// 各模式输出示例：
//   模式一 nonnominal（只写异常目标行；全部正常时整周期静默不写，防刷屏）：
//     周期=5 目标=1001 状态=不在视场 方位(ECI)=209.2° 仰角(ECI)=-89.3°
//   模式二 delta（只写状态有变化的目标行；无变化时整周期静默不写，防刷屏）：
//     周期=5 目标=1001 状态=已检测
//   模式三 summary（默认；每周期恰好一行完整摘要）：
//     周期=400 执行=是 目标=[1001 不在输出(方位209.2° 俯仰-89.3°),
//     1002 不在输出(方位29.2° 俯仰-89.3°)] 问题=[sbirs.target_out_of_wfov …]
void SbirsSensorComponent::LogDebugView(
    const sbirs_sensor::session::SbirsOutputDebugView& view) {
#if defined(CA_VIEW_LOG_MODE_NONNOMINAL)
  // 模式一：只写非标称目标（非已检测）行；全部正常时本周期静默不写（防刷屏）。
  for (const auto& target : view.targets) {
    if (target.status == sbirs_sensor::session::SbirsDebugTargetStatus::kDetected) {
      continue;
    }
    // 2026-08 正式变更：库内方位/俯仰输出为 ECI 极坐标弧度；示例按可读性转
    // 度显示。距离仅存在于库内诊断字段（estimated_range_m，仅归属目标有值），
    // 非标称行不再展示距离（被动红外测距无物理依据，见 docs/common/contract.md）。
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string sbirs_view_log =
        std::string("周期=") +
        std::to_string(view.input_cycle_index) +
        " 目标=" +
        std::to_string(target.target_id) +
        " 状态=" +
        (SbirsTargetStatusName(target.status)) +
        " 方位(ECI)=" +
        std::to_string(target.azimuth_rad * kRadToDeg) +
        "° 仰角(ECI)=" +
        std::to_string(target.elevation_rad * kRadToDeg) +
        "°";
    CA_LOG_VIEW("sbirs", "周期={} 目标={} 状态={} 方位(ECI)={:.1f}° 仰角(ECI)={:.1f}°",
                view.input_cycle_index, target.target_id,
                SbirsTargetStatusName(target.status), target.azimuth_rad * kRadToDeg,
                target.elevation_rad * kRadToDeg);
  }
#elif defined(CA_VIEW_LOG_MODE_DELTA)
  // 模式二：只写状态与上一周期不同的目标行（上一周期状态表由组件持有，首次
  // 出现视为变化；表只增不减，示例不清理）；无变化时静默不写（防刷屏）。
  for (const auto& target : view.targets) {
    const auto it = prev_target_status_.find(target.target_id);
    if (it == prev_target_status_.end() || it->second != target.status) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string sbirs_view_log_3 =
          std::string("周期=") +
          std::to_string(view.input_cycle_index) +
          " 目标=" +
          std::to_string(target.target_id) +
          " 状态=" +
          (SbirsTargetStatusName(target.status));
      CA_LOG_VIEW("sbirs", "周期={} 目标={} 状态={}",
                  view.input_cycle_index, target.target_id,
                  SbirsTargetStatusName(target.status));
    }
    prev_target_status_[target.target_id] = target.status;
  }
#else  // CA_VIEW_LOG_MODE_SUMMARY（默认）
  // 模式三（每周期摘要行）：目标状态明细带结构化量值（input 回填，未检测也
  // 可见目标角度）+ 问题中文名，一眼可读。
  std::string targets_text;
  for (const auto& target : view.targets) {
    if (!targets_text.empty()) {
      targets_text += ", ";
    }
    targets_text += std::to_string(target.target_id) +
                    std::string(" ") + SbirsTargetStatusName(target.status) + "(方位" +
                    std::to_string(target.azimuth_rad * kRadToDeg) + "° 俯仰" +
                    std::to_string(target.elevation_rad * kRadToDeg) + "°)";
  }
  const std::string issues_text = app::FormatIssueText(view.issues);
  // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
  const std::string sbirs_view_log_5 =
      std::string("周期=") +
      std::to_string(view.input_cycle_index) +
      " 执行=" +
      (view.executed_this_cycle ? "是" : "否") +
      " 目标=[" +
      (targets_text.empty() ? "无" : targets_text.c_str()) +
      "] 问题=[" +
      (issues_text.empty() ? "无" : issues_text.c_str()) +
      "]";
  CA_LOG_VIEW("sbirs", "周期={} 执行={} 目标=[{}] 问题=[{}]",
              view.input_cycle_index, view.executed_this_cycle ? "是" : "否",
              targets_text.empty() ? "无" : targets_text.c_str(),
              issues_text.empty() ? "无" : issues_text.c_str());
#endif  // CA_VIEW_LOG_MODE_*
}

sbirs_sensor::session::SbirsCycleInput SbirsSensorComponent::BuildCycleInput(
    const AppSceneState& scene, double dt_sec) const {
  sbirs_sensor::session::SbirsCycleInput input;
  input.cycle_index = static_cast<std::uint32_t>(scene.cycle);
  input.dt_sec = static_cast<float>(dt_sec);
  if (use_own_satellite_pose_) {
    input.satellite_position_ecef_m = own_position_ecef_m_;
    input.satellite_velocity_ecef_m_per_s = own_velocity_ecef_m_per_s_;
    input.satellite_attitude_eci_body_deg = own_attitude_eci_body_deg_;
  } else {
    input.satellite_position_ecef_m = scene.sbirs_satellite_position_ecef_m;
    input.satellite_velocity_ecef_m_per_s = scene.sbirs_satellite_velocity_ecef_m_per_s;
    input.satellite_attitude_eci_body_deg = scene.sbirs_satellite_attitude_eci_body_deg;
  }
  input.utc_julian_day = scene.sbirs_utc_julian_day;  // ECI 输出参考系（UTC 儒略日）
  input.scene = scene.sbirs_targets;

  return input;
}

void SbirsSensorComponent::Step(World& world, double dt_sec) {
  if (!powered_on_) {
    scan_azimuth_deg_ = 0.0f;  // 关机：不驱动会话，角度无有效值（清零）
    last_debug_view_ = sbirs_sensor::session::SbirsOutputDebugView{};  // 关机：调试视图清零（无有效周期）
    LogDebugView(last_debug_view_);
    return;
  }

  const FlightComponent* flight = host_ != nullptr ? host_->Find<FlightComponent>() : nullptr;
  if (!use_own_satellite_pose_ && flight == nullptr) {
    return;  // 机载挂载且无平台动力学：不产生探测；卫星实体自持星历，不走这条
  }

  // 共享场景状态（World 只存基类引用，实际类型为 AppSceneState）：读真值组输入，
  // 也向其探测池写适配记录，故直接取可变引用。
  auto& scene = static_cast<AppSceneState&>(world.scene_state());
  // cross-cue（2026-09-01）：先接线转发信号，再把"上一周期及更早收到"的递话注入
  // 会话（星→地→星固定一周期延迟：本周期收到的要等下一周期才消费）。
  EnsureCrossCueRelayConnection(world);
  if (cross_cue_enabled_) {
    const double kRadToDeg = 57.2957795130823;
    for (const SbirsCrossCueEvent& event : stashed_cue_events_) {
      if (event.cycle >= scene.cycle) {
        continue;  // 本周期才收到的：下一周期再消费
      }
      for (const SbirsCrossCueItem& item : event.cues) {
        sbirs_sensor::session::SbirsExternalCue cue;
        cue.target_id = item.target_id;
        cue.source_satellite_entity_id = event.source_satellite_entity_id;
        cue.source_position_ecef_m.x = event.satellite_ecef_x_m;
        cue.source_position_ecef_m.y = event.satellite_ecef_y_m;
        cue.source_position_ecef_m.z = event.satellite_ecef_z_m;
        cue.azimuth_deg = static_cast<float>(item.azimuth_rad * kRadToDeg);
        cue.elevation_deg = static_cast<float>(item.elevation_rad * kRadToDeg);
        cue.range_m = item.range_m;
        cue.cycle_index = static_cast<std::uint32_t>(event.cycle);
        session_.SubmitExternalCue(cue);
      }
    }
    stashed_cue_events_.clear();
  }
  const sbirs_sensor::session::SbirsCycleInput input = BuildCycleInput(scene, dt_sec);

  const std::chrono::steady_clock::time_point step_begin = std::chrono::steady_clock::now();
  const sbirs_sensor::session::SbirsCycleResult result = session_.StepWithResult(input);
  if (!step_timing_logged_) {
    app::LogAcceptanceMs(scene.cycle, scene.t_sec, "单步执行时间性能测试", "OPIR",
                          app::SteadyElapsedMs(step_begin));
    step_timing_logged_ = true;
  }
  // 扫描方位随周期结果刷新（拒绝周期为空帧 → 0）；库内为 ECI 弧度，组件转度显示。
  scan_azimuth_deg_ = result.output_frame.scan_azimuth_rad * kRadToDeg;
  // 调试视图快照每周期都要构建：它是下方视图行的数据源（日志写多少由三密度
  // 模式宏门控，与快照构建无关；被拒绝周期为 kCycleNotExecuted 行）。
  last_debug_view_ = sbirs_sensor::session::SbirsOutputDebugViewBuilder::Build(input, result);
  LogDebugView(last_debug_view_);
  if (result.status != sbirs_sensor::session::SbirsCycleStatus::kCompleted) {
    return;  // 周期被拒绝：本周期无探测
  }
  PublishDetectionEvents(world, scene, result);  // 探测生命周期沿 → 信号+事件日志
  PublishExclusionEvents(world);                 // 排除原因变化沿 → 事件日志
  if (ground_station_source_id_ != 0U) {
    PublishToGroundStation(world, scene, result);
  } else {
    AdaptDetections(world, scene, result);
  }
  if (cross_cue_enabled_) {
    PublishCrossCue(world, scene, result);
  }
}

void SbirsSensorComponent::EnsureCrossCueRelayConnection(World& world) {
  if (cross_cue_enabled_ && !cross_cue_relay_connection_.connected()) {
    cross_cue_relay_connection_ = world.signals().on_sbirs_cross_cue_relay.connect(
        [this](const SbirsCrossCueEvent& event) {
          if (event.source_satellite_entity_id == ground_station_source_id_) {
            return;  // 自己发的：不回灌
          }
          stashed_cue_events_.push_back(event);
        });
  }
}

void SbirsSensorComponent::PublishCrossCue(
    World& world, const AppSceneState& scene,
    const sbirs_sensor::session::SbirsCycleResult& result) {
  // ECI 测角 + 误差模型距离）；窄场精测不外发（修订 3 条 4：精测刷新不做）。
  SbirsCrossCueEvent event;
  event.cycle = scene.cycle;
  event.source_satellite_entity_id = ground_station_source_id_;
  event.satellite_ecef_x_m = own_position_ecef_m_.x;
  event.satellite_ecef_y_m = own_position_ecef_m_.y;
  event.satellite_ecef_z_m = own_position_ecef_m_.z;
  constexpr double kRadPerDeg = 0.017453292519943295;
  for (const sbirs_sensor::session::SbirsWideCueMeasurement& m :
       result.wide_cue_measurements) {
    if (m.target_id == 0U || !(m.measured_range_m > 0.0)) {
      continue;  // 无归属或无有效距离：不递话
    }
    SbirsCrossCueItem item;
    item.target_id = m.target_id;
    item.azimuth_rad = static_cast<float>(m.azimuth_deg * kRadPerDeg);
    item.elevation_rad = static_cast<float>(m.elevation_deg * kRadPerDeg);
    item.range_m = m.measured_range_m;
    event.cues.push_back(item);
  }
  if (!event.cues.empty()) {
    world.signals().on_sbirs_cross_cue(event);
  }
}

void SbirsSensorComponent::PublishDetectionEvents(
    World& world, const AppSceneState& scene,
    const sbirs_sensor::session::SbirsCycleResult& result) {
  // 库内 recorder 已按跨周期状态差分产出本周期事件（首发现/更新/coasting/丢失）。
  // recorder 事件无方位/探测 ID 字段：非丢失事件从本周期输出帧按归属目标回查；
  // 丢失事件目标不在帧内，字段留默认。kNotDetected 诊断事件未开启，显式跳过。
  const auto& records = result.output_frame.detections;
  for (const auto& event : lifecycle_.GetLastEvents()) {
    if (event.kind == sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kNotDetected) {
      continue;  // 诊断事件（未开启 emit_not_detected_events）：不发布
    }
    SbirsDetectionEvent sbirs_event;
    sbirs_event.cycle = scene.cycle;
    sbirs_event.kind = ToAppKind(event.kind);
    sbirs_event.reason = ToAppLossReason(event.reason);
    sbirs_event.target_id = event.target_id;
    sbirs_event.infrared_snr_linear = event.infrared_snr_linear;
    if (event.kind != sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kLost) {
      for (const auto& record : records) {
        if (!record.detected) {
          continue;  // 未过探测门限不参与回查
        }
        if (AttributionTargetId(record.detection_id, result.detection_attributions) ==
            event.target_id) {
          sbirs_event.detection_id = record.detection_id;
          sbirs_event.az_deg = record.azimuth_rad * kRadToDeg;  // ECI rad → 度（显示用）
          break;
        }
      }
    }
    const float snr_db = (sbirs_event.infrared_snr_linear > 0.0f)
        ? static_cast<float>(10.0 * std::log10(static_cast<double>(sbirs_event.infrared_snr_linear)))
        : -100.0f;
    if (sbirs_event.kind == SbirsDetectionEventKind::kUpdated) {
      // 更新类事件每周期重复：事件模式一下不落盘（信号照常发布）。
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string sbirs_detection_event_log =
          std::string("类型=") +
          (SbirsEventKindName(sbirs_event.kind)) +
          " 探测ID=" +
          std::to_string(static_cast<unsigned long long>(sbirs_event.detection_id)) +
          " 目标=" +
          std::to_string(static_cast<unsigned long long>(sbirs_event.target_id)) +
          " 信噪比(dB)=" +
          std::to_string(snr_db) +
          "dB 方位=" +
          std::to_string(sbirs_event.az_deg) +
          "°";
      CA_LOG_EVENT_DUP(world, "sbirs_detection",
                       "类型={} 探测ID={} 目标={} 信噪比(dB)={:.1f}dB 方位={:.1f}°",
                       SbirsEventKindName(sbirs_event.kind),
                       static_cast<unsigned long long>(sbirs_event.detection_id),
                       static_cast<unsigned long long>(sbirs_event.target_id),
                       snr_db, sbirs_event.az_deg);
    } else if (sbirs_event.kind == SbirsDetectionEventKind::kLost) {
      // 丢失事件携带细分原因（视场外/调度跳过/门失败…），区分扫描间隙与真丢失。
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string sbirs_detection_event_log_2 =
          std::string("类型=丢失(") +
          (LossReasonName(sbirs_event.reason)) +
          ") 探测ID=" +
          std::to_string(static_cast<unsigned long long>(sbirs_event.detection_id)) +
          " 目标=" +
          std::to_string(static_cast<unsigned long long>(sbirs_event.target_id)) +
          " 信噪比(dB)=" +
          std::to_string(snr_db) +
          "dB 方位=" +
          std::to_string(sbirs_event.az_deg) +
          "°";
      CA_LOG_EVENT(world, "sbirs_detection",
                   "类型=丢失({}) 探测ID={} 目标={} 信噪比(dB)={:.1f}dB 方位={:.1f}°",
                   LossReasonName(sbirs_event.reason),
                   static_cast<unsigned long long>(sbirs_event.detection_id),
                   static_cast<unsigned long long>(sbirs_event.target_id),
                   snr_db, sbirs_event.az_deg);
    } else {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string sbirs_detection_event_log_3 =
          std::string("类型=") +
          (SbirsEventKindName(sbirs_event.kind)) +
          " 探测ID=" +
          std::to_string(static_cast<unsigned long long>(sbirs_event.detection_id)) +
          " 目标=" +
          std::to_string(static_cast<unsigned long long>(sbirs_event.target_id)) +
          " 信噪比(dB)=" +
          std::to_string(snr_db) +
          "dB 方位=" +
          std::to_string(sbirs_event.az_deg) +
          "°";
      CA_LOG_EVENT(world, "sbirs_detection",
                   "类型={} 探测ID={} 目标={} 信噪比(dB)={:.1f}dB 方位={:.1f}°",
                   SbirsEventKindName(sbirs_event.kind),
                   static_cast<unsigned long long>(sbirs_event.detection_id),
                   static_cast<unsigned long long>(sbirs_event.target_id),
                   snr_db, sbirs_event.az_deg);
    }
    world.signals().on_sbirs_detection(sbirs_event);
  }

}

void SbirsSensorComponent::PublishExclusionEvents(World& world) {
  // 排除原因跨周期差分沿（进入/变化/退出）→ 仅事件日志，不发 World 信号；
  // 原因稳定不产事件，故无刷屏。SBIRS 排除涵盖遮挡/距离带/视场/SNR 四门，
  // 差分键为 (code,cause) 组合对——遮挡↔距离带切换（同为 kNone、code 不同）亦产事件。
  for (const auto& event : exclusion_.GetLastEvents()) {
    const std::uint64_t event_target_id = event.target_id;
    if (event.kind == sbirs_sensor::session::SbirsExclusionCauseEventKind::kEntered) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log =
          std::string("目标=") +
          std::to_string(static_cast<unsigned long long>(event_target_id)) +
          " 类型=进入排除 排除码=" +
          (event.current_code) +
          " 主因=" +
          (SbirsExclusionCauseName(event.current_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "目标={} 类型=进入排除 排除码={} 主因={}",
                   static_cast<unsigned long long>(event_target_id),
                   event.current_code, SbirsExclusionCauseName(event.current_cause));
    } else if (event.kind == sbirs_sensor::session::SbirsExclusionCauseEventKind::kChanged) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log_2 =
          std::string("目标=") +
          std::to_string(static_cast<unsigned long long>(event_target_id)) +
          " 类型=原因变化 旧码=" +
          (event.previous_code) +
          " 旧主因=" +
          (SbirsExclusionCauseName(event.previous_cause)) +
          " 新码=" +
          (event.current_code) +
          " 新主因=" +
          (SbirsExclusionCauseName(event.current_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "目标={} 类型=原因变化 旧码={} 旧主因={} 新码={} 新主因={}",
                   static_cast<unsigned long long>(event_target_id),
                   event.previous_code, SbirsExclusionCauseName(event.previous_cause),
                   event.current_code, SbirsExclusionCauseName(event.current_cause));
    } else if (event.kind == sbirs_sensor::session::SbirsExclusionCauseEventKind::kExited) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log_3 =
          std::string("目标=") +
          std::to_string(static_cast<unsigned long long>(event_target_id)) +
          " 类型=退出排除 旧码=" +
          (event.previous_code) +
          " 旧主因=" +
          (SbirsExclusionCauseName(event.previous_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "目标={} 类型=退出排除 旧码={} 旧主因={}",
                   static_cast<unsigned long long>(event_target_id),
                   event.previous_code, SbirsExclusionCauseName(event.previous_cause));
    }
  }

}

void SbirsSensorComponent::AdaptDetections(
    World& world, AppSceneState& scene,
    const sbirs_sensor::session::SbirsCycleResult& result) {
  const auto& records = result.output_frame.detections;
  if (detection_delivery_ == DetectionDeliveryMode::kMessage) {
    // 消息路径：展平为基础类型后发 on_detection_batch_submitted（地面站融合
    // 订阅重建库类型；集成方对应把样本推给融合组件的消息）。
    DetectionBatchSubmittedEvent event;
    event.cycle = scene.cycle;
    event.source_id = device_id_;
    for (const auto& record : records) {
      if (!record.detected) {
        continue;
      }
      FusionDetectionSample sample = ToFusionDetectionSample(record);
      sample.source_name = sensor_name_;
      event.records.push_back(sample);
    }
    world.signals().on_detection_batch_submitted(event);
    return;
  }
  // 黑板路径：库内适配器 → 共享探测池（FusionComponent 聚合读；集成方对应
  // 把记录消息推给融合组件）。
  std::vector<fusion::DetectionRecord> adapted =
      fusion::AdaptSbirsDetectionsToDetectionRecords(device_id_, records);
  for (fusion::DetectionRecord& record : adapted) {
    record.source_name = sensor_name_;
  }
  scene.detection_pool.insert(scene.detection_pool.end(), adapted.begin(), adapted.end());
}

void SbirsSensorComponent::PublishToGroundStation(
    World& world, AppSceneState& scene,
    const sbirs_sensor::session::SbirsCycleResult& result) {
  if (ground_delivery_ == SbirsGroundDeliveryMode::kMessage) {
    SbirsFrameSubmittedEvent event;
    event.cycle = scene.cycle;
    event.source_id = ground_station_source_id_;
    event.source_name = sensor_name_;
    event.satellite_ecef_x_m = own_position_ecef_m_.x;
    event.satellite_ecef_y_m = own_position_ecef_m_.y;
    event.satellite_ecef_z_m = own_position_ecef_m_.z;
    for (const auto& record : result.output_frame.detections) {
      if (!record.detected) {
        continue;
      }
      SbirsBearingSample sample;
      sample.detection_id = record.detection_id;
      sample.target_id =
          AttributionTargetId(record.detection_id, result.detection_attributions);
      sample.azimuth_rad = record.azimuth_rad;
      sample.elevation_rad = record.elevation_rad;
      sample.infrared_snr_linear = record.infrared_snr_linear;
      sample.measurement_time_s = record.measurement_time_s;
      // 观测阶段随消息透出（双星定位仅收窄场数据，2026-08-31 建模口径）。
      sample.observation_stage = static_cast<int>(record.observation_stage);
      // 焦平面脱靶量随消息透出（归属记录携带；精度评估层第26项 消费）。
      for (const auto& attribution : result.detection_attributions) {
        if (attribution.detection_id != record.detection_id) {
          continue;
        }
        if (attribution.has_focal_plane_offset) {
          sample.has_focal_plane_offset = true;
          sample.focal_plane_offset_x_m = attribution.focal_plane_offset_x_m;
          sample.focal_plane_offset_y_m = attribution.focal_plane_offset_y_m;
        }
        break;
      }
      event.detections.push_back(sample);
    }
    world.signals().on_sbirs_frame_submitted(event);
    return;
  }
  SbirsGroundStationFrame frame;
  frame.source_id = ground_station_source_id_;
  frame.satellite_position_ecef_m = own_position_ecef_m_;
  frame.result = result;
  scene.sbirs_ground_station_inbox.push_back(frame);
}


}  // namespace component_attachment
