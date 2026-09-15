/**
 * @file RirSession.cpp
 * @brief 远程识别雷达会话门面实现。
 *
 * 周期语义对齐 AR `ArSession`（审计基线 96de367c）：非执行周期不复用上一帧、
 * 校验拒绝返回 `kRejectedInvalidInput` + 明细、关机 `kPoweredOff` 只推进时间、
 * 运行期补丁在下一次成功周期边界提交。
 */

#include "1q/remote_identification_radar/session/RirSession.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/session/RirExclusionCauseRecorder.h"
#include "1q/remote_identification_radar/session/RirInputValidation.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "1q/remote_identification_radar/session/RirTrackLifecycleRecorder.h"
#include "common/logging/ProjectLog.h"
#include "common/numerics/Constants.h"
#include "common/radar/ScanScheduleRuntime.h"
#include "remote_identification_radar/dwell/RirBeamControl.h"
#include "remote_identification_radar/internal/RirScanVolume.h"
#include "remote_identification_radar/runtime/RirAcceptanceLog.h"
#include "remote_identification_radar/runtime/RirAcceptanceRecords.h"
#include "remote_identification_radar/runtime/RirController.h"
#include "remote_identification_radar/session/RirReplayCycleRecord.h"

namespace remote_identification_radar {
namespace session {

namespace {

/**
 * @brief 指定识别任务生命周期阶段（镜像 AR designation 生命周期骨架；
 *        会话级跨周期状态，随 patch 原子提交）。
 */
enum class RirDesignationPhase : std::uint8_t {
  kNone = 0,     /**< 未指定/未初始化（新指令待首个处理周期开窗）。 */
  kPending = 1,  /**< 限时窗口内：驻留对准指定目标等待识别达成。 */
  kAcquired = 2, /**< 识别已达成：任务完成，回到扫描（终态，直到外部重新指定）。 */
  kExpired = 3   /**< 窗口耗尽仍未识别：任务作废，回到扫描（终态，直到外部重新指定）。 */
};

struct RirDesignationPhaseAdvance {
  RirDesignationPhase phase{RirDesignationPhase::kNone};
  std::uint32_t deadline_cycle_index{0U}; /**< 窗口截止周期（0 = 无限期）。 */
  bool expired_edge{false}; /**< 本周期由 kPending 转移至 kExpired（作废沿）。 */
};

struct RirDesignationExpiry {
  bool expired{false};      /**< 任务已作废（kExpired 终态）。 */
  bool expiry_cycle{false}; /**< 本周期是作废沿（报告 kAcquisitionTimeout）。 */
};

/**
 * @brief 推进指定识别任务生命周期（镜像 AR AdvanceDesignationPhase）。
 *
 * 状态机：kNone → kPending → kAcquired | kExpired（kAcquired/kExpired 为终态）：
 *   - 指令未消费（未指定 / work_mode 非 kIdentify）：阶段恒为 kNone，窗口不运行；
 *   - kNone：开窗——deadline = cycle + duration（duration == 0 表示无限期）；
 *   - kPending：上一周期识别达成（kCategoryConfirmed/kModelConfirmed）→ kAcquired
 *     （任务完成回扫描）；否则 cycle >= deadline 时 → kExpired（任务作废回扫描）；
 *   - kAcquired / kExpired：不再转移。
 * @note 识别判定与驻留同源，使用"上一周期航迹快照"口径（滞后一周期）。
 */
RirDesignationPhaseAdvance AdvanceDesignationPhase(
    RirDesignationPhase phase, std::uint32_t deadline_cycle_index,
    std::uint32_t designation_duration_cycles, std::uint32_t cycle_index,
    bool designation_consumed, bool recognized) {
  RirDesignationPhaseAdvance advance;
  advance.phase = phase;
  advance.deadline_cycle_index = deadline_cycle_index;
  if (!designation_consumed) {
    advance.phase = RirDesignationPhase::kNone;
    advance.deadline_cycle_index = 0U;
    return advance;
  }
  if (phase == RirDesignationPhase::kNone) {
    advance.phase = RirDesignationPhase::kPending;
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
  if (phase == RirDesignationPhase::kPending) {
    if (recognized) {
      advance.phase = RirDesignationPhase::kAcquired;
    } else if (advance.deadline_cycle_index != 0U &&
               cycle_index >= advance.deadline_cycle_index) {
      advance.phase = RirDesignationPhase::kExpired;
      advance.expired_edge = true;  // 作废沿 = 转移发生的周期。
    }
  }
  return advance;
}

RirDesignationExpiry ResolveDesignationExpiry(RirDesignationPhase phase, bool expired_edge) {
  RirDesignationExpiry expiry;
  expiry.expired = phase == RirDesignationPhase::kExpired;
  expiry.expiry_cycle = expired_edge;
  return expiry;
}

const session::RirSceneTarget* FindSceneTarget(const session::RirSceneTargetList& targets,
                                               std::uint64_t external_target_id) {
  if (external_target_id == 0U) {
    return nullptr;
  }
  for (const session::RirSceneTarget& target : targets) {
    if (target.external_target_id == external_target_id) {
      return &target;
    }
  }
  return nullptr;
}

/** @brief 目标视线角（与 RirController::ComputeLookAngles 同口径）。 */
config::RirAzimuthElevationDeg TargetLookAngles(const session::RirSceneTarget& target) {
  const float px = target.position_x;
  const float py = target.position_y;
  const float pz = target.position_z;
  const float range_hypot = std::sqrt(px * px + py * py);
  config::RirAzimuthElevationDeg look;
  look.az_deg = oneq::common::numerics::RadToDeg(std::atan2(py, px));
  look.el_deg = oneq::common::numerics::RadToDeg(std::atan2(pz, range_hypot));
  return look;
}

// 可扫描体积判定单源于 internal/RirScanVolume.h（驻留门与检测候选裁剪同口径）。

/** @brief 由相对可扫描体积 + scan_center 构建绝对 ENU 波位序列。 */
std::vector<oneq::common::radar::AzimuthElevationDeg> BuildAbsoluteScanWaves(
    const config::RirSessionConfig& config) {
  // 波束宽度解析波长（λ = c/f）：与 RirController 检测门的 gate_wavelength_m 同源
  // 同口径（λ/L 孔径推导；检测门规划频率优先、兜底发射频率，此处为配置期构建，
  // 直接取发射频率）。不传波长时 nominal 波束宽为 0 的配置会解析出 0 步长，
  // BuildScanPattern 对步长 <= 0 返回空表，扫描静默退化。
  const float scan_wavelength_m =
      config.hardware.transmitter.frequency_hz > 0.0f
          ? static_cast<float>(oneq::common::numerics::kLightSpeed) /
                config.hardware.transmitter.frequency_hz
          : 0.0f;  // 频率无效时传 0：保持既有「无孔径推导」回退。
  const dwell::RirEffectiveBeamwidthDeg beamwidth =
      dwell::RirResolveEffectiveBeamwidth(config.hardware.antenna, scan_wavelength_m);
  const config::RirMissionConfig& mission = config.mission;
  // 实际搜索扇区 = 任务扫描子窗 ∩ 硬件可扫描体积（子窗缺省无界时退化为体积）：
  // 扫描波位仅在该扇区内推进（甲方「任务范围由用户指定」）。
  const config::RirAzimuthElevationLimitsDeg sector = internal::IntersectScanSector(
      mission.scan_window_deg, config.orientation);
  const float az_step = beamwidth.az_beamwidth_deg * mission.step_scale;
  const float el_step = beamwidth.el_beamwidth_deg * mission.step_scale;
  const std::vector<oneq::common::radar::AzimuthElevationDeg> relative_pattern =
      oneq::common::radar::BuildScanPattern(
          sector.az_min_deg, sector.az_max_deg, sector.el_min_deg, sector.el_max_deg, az_step,
          el_step, mission.scan_start_position, mission.scan_sequence);
  std::vector<oneq::common::radar::AzimuthElevationDeg> absolute_pattern;
  absolute_pattern.reserve(relative_pattern.size());
  const config::RirAzimuthElevationDeg& center = config.mission.scan_center_deg;
  for (const oneq::common::radar::AzimuthElevationDeg& wave : relative_pattern) {
    oneq::common::radar::AzimuthElevationDeg absolute;
    absolute.az_deg = oneq::common::radar::NormalizeAzimuthDeg(center.az_deg + wave.az_deg);
    absolute.el_deg = wave.el_deg;
    absolute_pattern.push_back(absolute);
  }
  return absolute_pattern;
}

}  // namespace

struct RirSession::Impl {
  config::RirSessionConfig config{};
  runtime::RirController controller{};
  config::RirRuntimeConfigPatch pending_patch{};
  bool has_pending_patch{false};
  // 指定识别任务状态（会话级，镜像 AR designation；随 patch 原子提交；
  // 任一指定相关字段变更视为新指令，窗口重新起算）。
  std::uint64_t designated_external_target_id{0U};
  std::uint32_t designation_duration_cycles{0U};
  RirDesignationPhase designation_phase{RirDesignationPhase::kNone};
  std::uint32_t designation_deadline_cycle_index{0U};
  std::uint64_t next_batch_id{1U};
  // [RirAccept] 波位排列表已按当前扫描配置输出（mission 配置变更后重置重发）。
  bool acceptance_scan_pattern_logged{false};
  // [TAS] 搜索波位游标（运行态；按实际搜索驻留数推进，mission 配置变更后归零）。
  std::size_t scan_wave_cursor_{0U};
  // 观测投影记录器（规则 10/11）：非拥有裸指针，nullptr = 未注册。
  RirTrackLifecycleRecorder* lifecycle_recorder{nullptr};
  RirExclusionCauseRecorder* exclusion_cause_recorder{nullptr};
  std::uint32_t sensor_entity_id{0U};

  /** @brief 组装单周期聚合结果（早退路径保持非执行语义；不驱动 recorder）。 */
  RirCycleResult RunCycle(const RirCycleInput& input);

  void SetSensorEntityId(std::uint32_t id) {
    sensor_entity_id = id;
    controller.SetSensorEntityId(id);
  }

  explicit Impl(const config::RirSessionConfig& session_config) : config(session_config) {
    controller.SetHardware(config.hardware);
    controller.UpdateRuntime(config.mission, config.policy);
    controller.UpdateEnvironment(config.environment);
  }
};

RirSession::RirSession() : impl_(new Impl(config::RirSessionConfig{})) {}
RirSession::RirSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
RirSession::~RirSession() = default;
RirSession::RirSession(RirSession&&) noexcept = default;
RirSession& RirSession::operator=(RirSession&&) noexcept = default;

RirOutputFrame RirSession::Step(const RirCycleInput& input) {
  return StepWithResult(input).output_frame;
}

RirCycleResult RirSession::StepWithResult(const RirCycleInput& input) {
  // 规则 10：recorder 注册后在 Step/StepWithResult 内部自动驱动（含早退路径，
  // 非执行周期由 recorder 自身空转——空事件、不推进状态）。
  RirCycleResult result = impl_->RunCycle(input);
  if (impl_->lifecycle_recorder != nullptr) {
    impl_->lifecycle_recorder->Update(input.scene_targets, result);
  }
  if (impl_->exclusion_cause_recorder != nullptr) {
    impl_->exclusion_cause_recorder->Update(input.scene_targets, result);
  }
  return result;
}

void RirSession::AttachTrackLifecycleRecorder(RirTrackLifecycleRecorder* recorder) noexcept {
  impl_->lifecycle_recorder = recorder;
}

void RirSession::AttachExclusionCauseRecorder(RirExclusionCauseRecorder* recorder) noexcept {
  impl_->exclusion_cause_recorder = recorder;
}

RirCycleResult RirSession::Impl::RunCycle(const RirCycleInput& input) {
  RirCycleResult result;
  result.input_cycle_index = input.input_cycle_index;
  result.output_frame.input_cycle_index = input.input_cycle_index;
  result.output_frame.batch_id = next_batch_id;

  // 关机：非执行周期，只记录状态，不推进识别状态（tracker 状态不被触碰）。
  if (!config.sensor_enabled) {
    result.status = RirCycleStatus::kPoweredOff;
    result.abort_reason = RirCycleAbortReason::kPoweredOff;
    RirIssue issue;
    issue.severity = RirIssueSeverity::kError;
    issue.phase = RirIssuePhase::kExecution;
    issue.code = codes::kSensorPoweredOff;
    issue.message = "RIR sensor is powered off.";
    result.issues.push_back(std::move(issue));
    // 中译：RIR 传感器关机（非执行周期中止）。
    // 标识：三写之三（人读日志）——电源关闭，本周期未执行；
    //       仅用于人读，不用于状态判断（规则 3）。
    PROJECT_LOG_ERROR("RIR sensor_powered_off: {} — {}", codes::kSensorPoweredOff,
                      "RIR sensor is powered off.");
    return result;
  }

  // 校验拒绝：不执行流水线，问题明细入 issues。
  const RirIssueList validation_issues =
      ValidateRirCycleInput(input, config.mission.recognition_dwell_sec);
  if (HasValidationError(validation_issues)) {
    result.status = RirCycleStatus::kRejectedInvalidInput;
    result.abort_reason = RirCycleAbortReason::kValidationRejected;
    result.issues = validation_issues;
    // 中译：RIR 输入校验拒绝（周期号）。
    // 标识：三写之三（人读日志）——调用方输入非法，本周期未执行；
    //       仅用于人读，不用于状态判断（规则 3）。
    PROJECT_LOG_WARN("RIR validation rejected for cycle_index={}", input.input_cycle_index);
    return result;
  }

  // 补丁提交（下一个成功周期边界）：任务域/电源/识别策略整域；叶子 work_mode 覆盖整域。
  if (has_pending_patch) {
    const config::RirRuntimeConfigPatch& patch = pending_patch;
    if (patch.has_mission) {
      config.mission = patch.mission;
      acceptance_scan_pattern_logged = false;
      scan_wave_cursor_ = 0U;  // 波位表随 mission 变更重建，游标同步归零。
    }
    if (patch.has_work_mode) {
      config.mission.work_mode = patch.work_mode;
    }
    if (patch.has_scan_center) {
      config.mission.scan_center_deg = patch.scan_center_deg;
      acceptance_scan_pattern_logged = false;
      scan_wave_cursor_ = 0U;  // 波位表随转台朝向平移，游标语义重置。
    }
    if (patch.has_policy) {
      config.policy = patch.policy;
    }
    if (patch.has_environment) {
      config.environment = patch.environment;
      controller.UpdateEnvironment(config.environment);
    }
    if (patch.has_sensor_enabled) {
      config.sensor_enabled = patch.sensor_enabled;
    }
    // 指定识别任务：任一指定相关字段变更（含仅改时长）都视为新指令，
    // 生命周期阶段重置，窗口在指令生效后首个周期重新起算。
    if (patch.has_designated_target_id || patch.has_designation_duration_cycles) {
      if (patch.has_designated_target_id) {
        designated_external_target_id = patch.designated_external_target_id;
      }
      if (patch.has_designation_duration_cycles) {
        designation_duration_cycles = patch.designation_duration_cycles;
      }
      designation_phase = RirDesignationPhase::kNone;
      designation_deadline_cycle_index = 0U;
    }
    controller.UpdateRuntime(config.mission, config.policy);
    has_pending_patch = false;
  }

  // 指定识别任务生命周期推进（镜像 AR 限时锁定语义）：
  //   识别达成（上一周期口径）→ 任务完成回到扫描；窗口耗尽仍未识别 →
  //   任务作废（作废沿报告 kAcquisitionTimeout）→ 回到扫描。
  const bool designation_consumed =
      config.mission.work_mode == config::RirWorkMode::kIdentify &&
      designated_external_target_id != 0U;
  const RirDesignationPhaseAdvance advance = AdvanceDesignationPhase(
      designation_phase, designation_deadline_cycle_index, designation_duration_cycles,
      input.input_cycle_index, designation_consumed,
      designation_consumed &&
          controller.IsTargetRecognized(designated_external_target_id));
  designation_phase = advance.phase;
  designation_deadline_cycle_index = advance.deadline_cycle_index;
  const RirDesignationExpiry expiry =
      ResolveDesignationExpiry(advance.phase, advance.expired_edge);

  // 驻留中心（库内驻留调度器）：任务窗口内对准指定目标（在场景且在可扫描体积内）；
  // 其余情况按扫描策略逐周期推进。
  const session::RirSceneTarget* designated_target = FindSceneTarget(
      input.scene_targets, designated_external_target_id);
  const bool target_in_scene = designated_target != nullptr;
  const bool target_in_volume =
      target_in_scene &&
      internal::TargetWithinSteerableVolume(TargetLookAngles(*designated_target),
                                            config.orientation,
                                            config.mission.scan_center_deg);
  const bool dwelling_on_target =
      advance.phase == RirDesignationPhase::kPending && target_in_volume;

  // 周期驻留计划（2026-08-29 TAS 边搜边跟）：① 指定驻留（算子导引，语义不变）
  // → ② 确认航迹跟踪驻留（库内航迹预测；保底 1 个搜索槽）→ ③ 搜索波位按运行态
  // 游标填充。无航迹/无指定周期退化为整周期搜索：游标每周期 +1 ≡ 既有
  // "第 N 周期取第 (N-1)%size 个波位"口径。
  const float dwell_sec = config.mission.recognition_dwell_sec;
  std::size_t max_dwells = 1U;
  if (input.dt_sec > 0.0 && dwell_sec > 0.0f) {
    // 浮点容差取整：0.5/0.05f = 9.9999… 必须入到 10，否则默认配置系统性少排
    // 一个驻留（消耗 0.45s ≠ 周期 0.5s）；非整除组合 floor 语义不变。
    const double dwell_quota =
        std::floor(input.dt_sec / static_cast<double>(dwell_sec) + 1.0e-6);
    max_dwells = std::max<std::size_t>(1U, static_cast<std::size_t>(dwell_quota));
  }
  std::vector<runtime::RirDwellPlan> dwell_plan;
  if (dwelling_on_target) {
    runtime::RirDwellPlan designate_dwell;
    designate_dwell.pointing_deg = TargetLookAngles(*designated_target);
    designate_dwell.kind = runtime::RirDwellKind::kDesignate;
    designate_dwell.external_target_id = designated_external_target_id;
    dwell_plan.push_back(designate_dwell);
  }
  if (max_dwells > dwell_plan.size() + 1U) {
    std::vector<runtime::RirDwellPlan> track_dwells = controller.CollectTrackDwellRequests(
        max_dwells - dwell_plan.size() - 1U, static_cast<float>(input.dt_sec));
    dwell_plan.insert(dwell_plan.end(), track_dwells.begin(), track_dwells.end());
  }
  const std::vector<oneq::common::radar::AzimuthElevationDeg> scan_pattern =
      BuildAbsoluteScanWaves(config);
  while (dwell_plan.size() < max_dwells) {
    runtime::RirDwellPlan search_dwell;
    if (!scan_pattern.empty()) {
      scan_wave_cursor_ %= scan_pattern.size();
      const oneq::common::radar::AzimuthElevationDeg& wave = scan_pattern[scan_wave_cursor_];
      search_dwell.pointing_deg = config::RirAzimuthElevationDeg{wave.az_deg, wave.el_deg};
      search_dwell.scan_pattern_index = scan_wave_cursor_;
      scan_wave_cursor_ = (scan_wave_cursor_ + 1U) % scan_pattern.size();
    } else {
      search_dwell.pointing_deg = config.mission.scan_center_deg;
    }
    dwell_plan.push_back(search_dwell);
  }
  // 结果层主驻留指向：首个非跟踪驻留（指定优先，其次搜索）——与单驻留时代语义对齐。
  config::RirAzimuthElevationDeg dwell_center = config.mission.scan_center_deg;
  for (const runtime::RirDwellPlan& dwell : dwell_plan) {
    if (dwell.kind != runtime::RirDwellKind::kTrack) {
      dwell_center = dwell.pointing_deg;
      break;
    }
  }

  // 验收事件 beam_pattern（3.2.2.4.2.1）：完整波位排列表按当前扫描配置一次性
  // 输出（与逐周期取位同源；mission/orientation 配置变更后重发）。
  if (RIR_ACCEPTANCE_LOG_ENABLED() && !acceptance_scan_pattern_logged) {
    // 摘要波束宽与扫描波位/检测门同口径（λ = c/f，λ/L 孔径推导；见
    // BuildAbsoluteScanWaves 与 RirController gate_wavelength_m）。
    const float summary_wavelength_m =
        config.hardware.transmitter.frequency_hz > 0.0f
            ? static_cast<float>(oneq::common::numerics::kLightSpeed) /
                  config.hardware.transmitter.frequency_hz
            : 0.0f;
    const dwell::RirEffectiveBeamwidthDeg beamwidth = dwell::RirResolveEffectiveBeamwidth(
        config.hardware.antenna, summary_wavelength_m);
    const std::string csv_path = runtime::ResolveRirAntennaPatternCsvPath();
    runtime::TryExportRirAntennaPatternCsv(config.hardware.antenna, csv_path.c_str());
    const std::string scan_csv_path = runtime::ResolveRirScanPatternCsvPath();
    runtime::TryExportRirScanPatternCsv(scan_pattern, scan_csv_path.c_str());
    runtime::WriteRirAntennaPatternSummary(controller.EffectiveSensorEntityId(), input.sim_time_sec,
                                           input.input_cycle_index,
                                           config.hardware.antenna.main_beam_gain_db,
                                           beamwidth.az_beamwidth_deg, beamwidth.el_beamwidth_deg);
    runtime::WriteRirOncePerSession(input.sim_time_sec, input.input_cycle_index);
    acceptance_scan_pattern_logged = true;
  }
  if (RIR_ACCEPTANCE_LOG_ENABLED()) {
    runtime::WriteRirDwellScan(controller.EffectiveSensorEntityId(), input.sim_time_sec,
                               input.input_cycle_index, dwell_plan);
    runtime::WriteRirCycleRunCount(input.sim_time_sec, input.input_cycle_index);
  }

  // 搜索角域随驻留计划一并下发：硬件体积 + 转台朝向 + 任务扫描子窗 + 指定目标 ID；
  // 检测候选集按「子窗 ∩ 体积」裁剪，指定目标与确认航迹目标在体积内豁免子窗
  // （2026-08-22 甲方批注「设定方位俯仰进行扫描」+「任务范围由用户指定」；
  // 2026-08-29 TAS 跟踪连续性豁免）。
  controller.RunCycle(input, &result.output_frame, next_batch_id, dwell_plan,
                      config.orientation, config.mission.scan_center_deg,
                      config.mission.scan_window_deg, designated_external_target_id);
  result.status = RirCycleStatus::kCompleted;
  result.abort_reason = RirCycleAbortReason::kNone;
  ++next_batch_id;
  if (controller.HasLatestSummary()) {
    result.has_recognition_summary = true;
    result.recognition_summary = controller.GetLatestSummary();
  }
  // 航迹归属视图回填（结果层）：仅已执行周期携带，非执行路径早退保持空列表。
  result.track_attributions = controller.LatestTrackAttributions();
  result.emission_frame = controller.LatestEmissionFrame();
  // 执行期按目标排除诊断（规则 13b）：完成周期并入统一问题列表（早退路径
  // 保持校验明细，不携带执行诊断）。
  result.issues = controller.LatestExecutionIssues();

  // 指定识别任务结果回填（镜像 AR designation_* 形状）：
  //   kPending + 目标在体积内 → active；kPending + 目标缺席 → kNotRecognized；
  //   kPending + 目标在场景但越界 → kOutsideSteerableVolume（阶段保持 kPending）；
  //   作废沿 → kAcquisitionTimeout；任务完成/作废后 → 指定清零。
  const bool pending = advance.phase == RirDesignationPhase::kPending;
  result.designated_target_id =
      (pending || expiry.expiry_cycle) ? designated_external_target_id : 0U;
  result.designation_active = pending && target_in_volume;
  result.designation_reverted_to_scan =
      expiry.expired ? expiry.expiry_cycle : (pending && !target_in_volume);
  if (expiry.expiry_cycle) {
    result.designation_revert_reason = session::RirDesignationRevertReason::kAcquisitionTimeout;
  } else if (result.designation_reverted_to_scan) {
    result.designation_revert_reason =
        target_in_scene && !target_in_volume
            ? session::RirDesignationRevertReason::kOutsideSteerableVolume
            : session::RirDesignationRevertReason::kNotRecognized;
  } else {
    result.designation_revert_reason = session::RirDesignationRevertReason::kNone;
  }
  result.dwell_center_deg = dwell_center;
  // 实际有效目标最大斜距（本周期持航迹目标最大输入斜距）：给外部判断实际探测距离，
  // 与 max_range_m 径向粗筛门区分。
  result.max_detected_slant_range_m = controller.LastMaxDetectedSlantRangeM();
  return result;
}

bool RirSession::TryApplyRuntimeConfig(const config::RirRuntimeConfigPatch& patch) {
  impl_->pending_patch = patch;
  impl_->has_pending_patch = true;
  return true;
}

bool RirSession::HasLatestRecognitionSummary() const {
  return impl_->controller.HasLatestSummary();
}

const RirRecognitionCycleSummary& RirSession::GetLatestRecognitionSummary() const {
  return impl_->controller.GetLatestSummary();
}

double RirSession::LastRecognitionDatabaseLoadMs() const {
  return impl_ != nullptr ? impl_->controller.LastDatabaseLoadMs() : 0.0;
}

void RirSession::SetSensorEntityId(std::uint32_t sensor_entity_id) {
  if (impl_ != nullptr) {
    impl_->SetSensorEntityId(sensor_entity_id);
  }
}

RirSession RirSession::Create(const config::RirSessionConfig& config) {
  return RirSession(std::unique_ptr<Impl>(new Impl(config)));
}

RirSession RirSession::CreateWithDiagnostics(const config::RirSessionConfig& config,
                                             RirIssueList* issues) {
  if (issues != nullptr) {
    *issues = config::ValidateRirSessionConfig(config);
  }
  return RirSession(std::unique_ptr<Impl>(new Impl(config)));
}

RirSessionReplayState RirSessionReplayAccess::CaptureSessionState(const RirSession& session) {
  RirSessionReplayState replay_state;
  replay_state.active_database_version = session.impl_->controller.ActiveDatabaseVersion();
  replay_state.detection_random_seed = session.impl_->controller.DetectionRandomSeed();
  return replay_state;
}

}  // namespace session
}  // namespace remote_identification_radar
