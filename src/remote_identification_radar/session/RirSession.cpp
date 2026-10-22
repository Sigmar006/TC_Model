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
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"
#include "1q/coordinate/types.h"
#include "1q/remote_identification_radar/session/RirExclusionCauseRecorder.h"
#include "1q/remote_identification_radar/session/RirInputValidation.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "1q/remote_identification_radar/session/RirTrackLifecycleRecorder.h"
#include "common/logging/ProjectLog.h"
#include "common/logging/AcceptanceFileLog.h"
#include "common/numerics/Constants.h"
#include "common/radar/BeamwidthResolution.h"
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
 * 「待处理」请求哨兵：`SubmitExternalTrack` 在周期**之间**被调用，无从得知它属于
 * 哪一拍，故把「下一拍执行时本请求即为新请求」编码为该哨兵值。周期号从 1 起，
 * 故哨兵不会与任何真实周期号相撞。用独立常量而非 0，避免与「未提交」语义混淆
 * （该混淆曾导致首周期被误判为外推保持）。
 */
constexpr std::uint32_t kExternalTrackRequestPending = 0xFFFFFFFFU;

/**
 * @brief 每周期**保底**留给自主搜索的驻留数（T-04b，2026-10-20）。
 *
 * @details 20 个驻留槽位是硬预算。此前外部引导窗口按「剩余槽位 − 1」铺点，实测吃掉 19/20，
 *   搜索只剩 1 拍；而波位表有 4,750 个波位、目标在窗内只停留约 1,070 周期，
 *   1 拍/周期意味着单帧要 4,750 周期（连 0.23 帧都扫不完）——自主搜索实际上不可能发现目标。
 *   取 4 拍后单帧约 1,188 周期，穿越期内可扫约 0.9 帧，且仍给引导留 13–14 拍。
 */
constexpr std::size_t kMinSearchDwells = 4U;

/** @brief 每周期保底留给**库内航迹**的驻留数（T-04b）：自产航迹需要自己的量测。 */
constexpr std::size_t kMinInternalTrackDwells = 2U;

/**
 * @brief 外部窗口一次构造的最大点数上限（远大于实际 5×5=25，用于取到完整窗口序列）。
 *
 * @details 窗口点数上限只用于截断异常配置；跨周期续扫需要**完整**序列，故给足余量。
 */
constexpr std::size_t kExternalWindowMaxPoints = 128U;

/** @brief cue 合理性门：目标可达的最大速度（m/s），用于判定位置跳变是否可信。 */
constexpr double kCueGateMaxSpeedMps = 10000.0;

/** @brief cue 合理性门：跳变容差（m），覆盖定位刷新间隔内的正常位移。 */
constexpr double kCueGateSlackM = 50000.0;

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

/**
 * @brief 解析扫描/波束宽度推导用波长（λ = c/f），与 BuildAbsoluteScanWaves 同口径。
 * @return 发射频率有效时返回波长（m），否则 0（保持"无孔径推导"回退）。
 */
float ResolveScanWavelengthM(const ::remote_identification_radar::config::RirSessionConfig& cfg) {
  return cfg.hardware.transmitter.frequency_hz > 0.0f
             ? static_cast<float>(oneq::common::numerics::kLightSpeed) /
                   cfg.hardware.transmitter.frequency_hz
             : 0.0f;
}

/**
 * @brief 生成外部跟踪的邻域窗口波位（中心优先、由近到远）。
 *
 * 机理：窗口半宽是「指向误差容限」——中心对应零外推误差，故中心必须排第一；
 * 其余波位按离中心的归一化距离升序展开，覆盖到窗口边缘，从而对位置误差与
 * 目标机动提供容错。波位表**每周期随外推中心重建**（区别于全局扫描波位表
 * 一次建好 + 游标推进：后者在默认体积下目标方向要上千周期才被照到一次）。
 *
 * @param[in] center_deg    外推后的照射中心（雷达局部 ENU，deg）。
 * @param[in] window        邻域半宽（deg，两轴相对中心）。
 * @param[in] beamwidth_deg 有效波束宽度（决定格点步长）。
 * @param[in] step_scale    步长系数（mission.step_scale）。
 * @param[in] max_points    波位点数上限（受本周期可用驻留槽位限制）。
 * @param[out] out          输出波位列表；首元素恒为中心。
 */
void BuildExternalTrackWindowDwells(
    const ::remote_identification_radar::config::RirAzimuthElevationDeg& center_deg,
    const ::remote_identification_radar::session::RirTrackWindowDeg& window,
    const ::remote_identification_radar::dwell::RirEffectiveBeamwidthDeg& beamwidth_deg,
    float step_scale, std::size_t max_points,
                                    std::vector<config::RirAzimuthElevationDeg>* out) {
  out->clear();
  if (max_points == 0U) {
    return;
  }
  out->push_back(center_deg);
  if (max_points == 1U) {
    return;
  }
  const float az_step = beamwidth_deg.az_beamwidth_deg * (step_scale > 0.0f ? step_scale : 1.0f);
  const float el_step = beamwidth_deg.el_beamwidth_deg * (step_scale > 0.0f ? step_scale : 1.0f);
  if (!(az_step > 0.0f) || !(el_step > 0.0f)) {
    return;  // 波束宽度不可解析：只留中心，不假装能铺窗口
  }
  // 两轴格点幅度：0 与 ±k·step，限制在最外层之内。
  std::vector<float> az_offsets;
  std::vector<float> el_offsets;
  az_offsets.push_back(0.0f);
  el_offsets.push_back(0.0f);
  for (float k = 1.0f;; k += 1.0f) {
    const float a = k * az_step;
    if (a > window.az_half_deg + 1.0e-4f) {
      break;
    }
    az_offsets.push_back(a);
    az_offsets.push_back(-a);
    if (az_offsets.size() > 512U) {
      break;  // 防御：窗口异常大时不无界展开
    }
  }
  for (float k = 1.0f;; k += 1.0f) {
    const float e = k * el_step;
    if (e > window.el_half_deg + 1.0e-4f) {
      break;
    }
    el_offsets.push_back(e);
    el_offsets.push_back(-e);
    if (el_offsets.size() > 512U) {
      break;
    }
  }
  if (az_offsets.size() == 1U && el_offsets.size() == 1U) {
    return;  // 窗口小于一个波束：只照中心（合法配置，非错误）
  }
  // 候选点按离中心的归一化距离升序（近的先照——对应较小的外推误差）。
  struct Candidate {
    float az_off;
    float el_off;
    float norm;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(az_offsets.size() * el_offsets.size());
  const float az_half = window.az_half_deg > 0.0f ? window.az_half_deg : 1.0f;
  const float el_half = window.el_half_deg > 0.0f ? window.el_half_deg : 1.0f;
  for (float az_off : az_offsets) {
    for (float el_off : el_offsets) {
      if (az_off == 0.0f && el_off == 0.0f) {
        continue;  // 中心已入列
      }
      Candidate c;
      c.az_off = az_off;
      c.el_off = el_off;
      c.norm = std::sqrt((az_off / az_half) * (az_off / az_half) +
                         (el_off / el_half) * (el_off / el_half));
      candidates.push_back(c);
    }
  }
  std::stable_sort(candidates.begin(), candidates.end(),
                   [](const Candidate& lhs, const Candidate& rhs) { return lhs.norm < rhs.norm; });
  for (const Candidate& c : candidates) {
    if (out->size() >= max_points) {
      break;
    }
    config::RirAzimuthElevationDeg p;
    p.az_deg = oneq::common::radar::NormalizeAzimuthDeg(center_deg.az_deg + c.az_off);
    p.el_deg = center_deg.el_deg + c.el_off;
    out->push_back(p);
  }
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

  // ---- 外部跟踪（中心 + 邻域窗口）状态；见 include/.../RirTrackRequest.h ----
  bool external_track_active{false};
  session::RirExternalTrackRequest external_track_request{};
  RirExternalTrackPhase external_track_phase{RirExternalTrackPhase::kNone};
  /** 请求所属周期号：kExternalTrackRequestPending = 已提交待下一拍消费；
   *  真实周期号 = 已消化的最近一次新请求；0 = 未提交。
   *  `external_track_active` 表示「指令仍在有效期内」，到期后置 false，
   *  但 phase 保持 kExpired 终态直到下一次成功提交。 */
  std::uint32_t external_track_request_cycle{0U};
  std::uint32_t external_track_last_request_cycle{0U};
  std::uint32_t external_track_coast_cycles{0U};
  std::uint32_t external_track_sweep_index{0U};
  std::uint32_t external_track_last_seen_cycle{0U};
  /**
   * 外部窗口**跨周期续扫**游标（T-04b，2026-10-20）。
   * @note 窗口序列按「中心优先、由近到远」排好，每周期从游标处取若干点并推进；
   *       新 cue 到来时归零，保证新数据一到就先照中心。
   */
  std::size_t external_window_cursor{0U};
  /** cue 合理性门：上一次**被接受**的 cue 位置（ECEF）及其周期。 */
  bool cue_gate_has_last{false};
  std::uint32_t cue_gate_last_cycle{0U};
  oneq::coordinate::EcefPositionM cue_gate_last_position{};
  /** 本周期 cue 被判不可信（位置跳变超物理上限）→ 只照中心 1 拍，其余槽位让给搜索。 */
  bool external_cue_degraded_this_cycle{false};
  /** 本周期判定标志：有效窗口到期（沿）——用于结果层发一次到期诊断。 */
  bool external_track_expired_this_cycle{false};
  /** 本周期判定标志：窗口已超期（电平）——用于抑制同周期的 stale 诊断。 */
  bool external_track_window_expired_this_cycle{false};

  /** @brief 解析外部跟踪在当前周期是否应产生驻留，并算出外推后的照射中心（局部 ENU）。
   *  @return 应排外部驻留返回 true，并写出 @p center_deg 与 @p phase。 */
  bool ResolveExternalTrackCenter(std::uint32_t cycle_index, double sim_time_sec,
                                  const oneq::coordinate::EcefPositionM& platform_ecef,
                                  config::RirAzimuthElevationDeg* center_deg,
                                  RirExternalTrackPhase* phase);

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
RirSession::RirSession(RirSession&& other) noexcept : impl_(std::move(other.impl_)) {}
RirSession& RirSession::operator=(RirSession&& other) noexcept { impl_ = std::move(other.impl_); return *this; }

RirOutputFrame RirSession::Step(const RirCycleInput& input) {
  return StepWithResult(input).output_frame;
}

RirCycleResult RirSession::StepWithResult(const RirCycleInput& input) {
  runtime::RirScopedAcceptanceLogEnable acceptance_log_scope(
      impl_->config.acceptance_log_enabled);
  oneq::logging::ScopedAcceptanceLogOutputDirectory acceptance_log_directory_scope(
      impl_->config.acceptance_log_output_directory);
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

bool RirSession::SubmitExternalTrack(const session::RirExternalTrackRequest& request) {
  // 校验：位置/速度有限、位置非退化、窗口为正。非法则**原子拒绝**（不改状态）。
  const oneq::coordinate::EcefPositionM& p = request.position_ecef_m;
  const oneq::coordinate::EcefVelocityMps& v = request.velocity_ecef_mps;
  const bool pos_finite = std::isfinite(p.x_m) && std::isfinite(p.y_m) && std::isfinite(p.z_m);
  const bool vel_finite = std::isfinite(v.x_mps) && std::isfinite(v.y_mps) &&
                          std::isfinite(v.z_mps);
  const double pos_norm = std::sqrt(p.x_m * p.x_m + p.y_m * p.y_m + p.z_m * p.z_m);
  const bool window_ok = request.window.az_half_deg > 0.0f && request.window.el_half_deg > 0.0f &&
                         std::isfinite(request.window.az_half_deg) &&
                         std::isfinite(request.window.el_half_deg);
  if (!pos_finite || !vel_finite || !window_ok || !(pos_norm > 1.0)) {
    return false;
  }
  impl_->external_track_request = request;
  impl_->external_track_active = true;
  impl_->external_track_phase = RirExternalTrackPhase::kFresh;
  impl_->external_track_coast_cycles = 0U;
  impl_->external_track_request_cycle = kExternalTrackRequestPending;
  impl_->external_track_expired_this_cycle = false;
  return true;
}

void RirSession::ClearExternalTrack() {
  impl_->external_track_active = false;
  impl_->external_track_phase = RirExternalTrackPhase::kNone;
  impl_->external_track_coast_cycles = 0U;
  impl_->external_track_sweep_index = 0U;
  impl_->external_track_request_cycle = 0U;  // 0 = 未提交（与哨兵语义分离）
  impl_->external_track_last_request_cycle = 0U;
  impl_->external_track_expired_this_cycle = false;
}

session::RirExternalTrackState RirSession::GetExternalTrackState() const {
  session::RirExternalTrackState state;
  state.phase = impl_->external_track_phase;
  state.last_request_cycle_index = impl_->external_track_last_request_cycle;
  state.coasting_cycles = impl_->external_track_coast_cycles;
  return state;
}

/**
 * @brief 解析外部跟踪在本周期是否产生驻留，并算出外推后的照射中心（局部 ENU）。
 *
 * 外推是本设计的关键：外部给的位置对应 `source_time_s` 时刻，而本周期指向应指向
 * "目标现在在哪"。实测（docs/review/ir_dual_sat_fix_accuracy_measurement_2026-10-04.md）
 * 表明 1827 m/s 目标上 1 个周期的时延就等于 1827 m 位置偏差，大于测角精度贡献，
 * 故不外推则时间偏斜直接变成指向误差。
 */
bool RirSession::Impl::ResolveExternalTrackCenter(
    std::uint32_t cycle_index, double sim_time_sec,
    const oneq::coordinate::EcefPositionM& platform_ecef,
    config::RirAzimuthElevationDeg* center_deg, RirExternalTrackPhase* phase) {
  if (center_deg == nullptr || phase == nullptr) {
    return false;
  }
  // 有效期判定必须先于「已释放」早退：否则到期那一周期会把 kExpired 吞掉，
  // 外部永远看不到释放事件（本函数是该状态的唯一生产者）。
  // 设计语义：kExpired 是**终态**（保持到外部重新提交），不是一次性事件。
  // 故「已释放」由 phase == kExpired 表达，而不复位 active —— 复位会让结果层
  // 退回 kNone，外部就分不清「从未启用」与「曾跟踪但已到期释放」。
  const bool window_expired = external_track_active &&
                              external_track_request.valid_until_cycle_index != 0U &&
                              cycle_index > external_track_request.valid_until_cycle_index;
  if (window_expired) {
    external_track_phase = RirExternalTrackPhase::kExpired;
    external_track_expired_this_cycle = true;
    external_track_window_expired_this_cycle = true;
    *phase = external_track_phase;
    return false;
  }
  *phase = external_track_phase;
  if (!external_track_active || external_track_phase == RirExternalTrackPhase::kExpired) {
    return false;  // 未启用 / 已到期释放：本周期不排外部驻留（phase 字段仍如实携带终态）。
  }

  // 时延外推：位置(source_time) -> 位置(本周期)。
  const double dt_extrapolate = sim_time_sec - external_track_request.source_time_s;
  oneq::coordinate::EcefPositionM position_at_now;
  position_at_now.x_m =
      external_track_request.position_ecef_m.x_m + external_track_request.velocity_ecef_mps.x_mps *
                                                       dt_extrapolate;
  position_at_now.y_m =
      external_track_request.position_ecef_m.y_m + external_track_request.velocity_ecef_mps.y_mps *
                                                       dt_extrapolate;
  position_at_now.z_m =
      external_track_request.position_ecef_m.z_m + external_track_request.velocity_ecef_mps.z_mps *
                                                       dt_extrapolate;

  // 换系：ECEF -> 平台锚点局部 ENU（与场景目标同源一站式入口）。
  oneq::coordinate::LlaPositionDegM anchor_lla;
  if (!oneq::coordinate::TryEcefToLla(platform_ecef, &anchor_lla)) {
    return false;
  }
  oneq::coordinate::ExternalKinematics kinematics;
  kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  kinematics.position_ecef_m = position_at_now;
  kinematics.velocity_mps = external_track_request.velocity_ecef_mps;
  oneq::coordinate::EnuSceneState enu;
  if (!oneq::coordinate::TryMakeEnuSceneState(kinematics, anchor_lla, &enu)) {
    return false;
  }
  const double px = enu.position_enu_m.east_m;
  const double py = enu.position_enu_m.north_m;
  const double pz = enu.position_enu_m.up_m;
  const double range_hypot = std::sqrt(px * px + py * py);
  // 与库内 ComputeLookAngles / TargetLookAngles 同口径：az 自 +x（东）向 +y（北）。
  center_deg->az_deg = static_cast<float>(oneq::common::numerics::RadToDeg(std::atan2(py, px)));
  center_deg->el_deg =
      static_cast<float>(oneq::common::numerics::RadToDeg(std::atan2(pz, range_hypot)));

  // 体积门：越界则本周期不排外部波位（不吸附、如实报告）。
  if (!internal::TargetWithinSteerableVolume(*center_deg, config.orientation,
                                             config.mission.scan_center_deg)) {
    *phase = RirExternalTrackPhase::kCoasting;
    return false;
  }
  external_track_last_seen_cycle = cycle_index;
  *phase = external_track_phase;
  return true;
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
  // 外部窗口是否吃满本周期全部槽位（2026-10-22 外部优先策略）：为 true 时本周期不排
  // 库内跟踪驻留与固定扫描光栅驻留。cue 合理性门降级或窗口无波位时不置位（退回原调度）。
  bool external_monopolizes_budget = false;
  // ⓪ 外部跟踪（位置 + 速度 -> 外推中心 + 邻域窗口）。优先级仅次于人工指定：
  //    外部跟踪是"实时目标指令"，而库内航迹是自产状态，外部指令应更靠前。
  //    中心优先、由近到远铺窗口波位；**每周期随外推中心重建**，不依赖扫描游标。
  //
  // 阶段判定的顺序要求：必须先判「本周期有没有新请求」，再解析外推中心——
  // 否则首周期（last_request_cycle 尚为 0）会被误判为 coasting。
  external_track_window_expired_this_cycle = false;
  if (external_track_active) {
    if (external_track_request_cycle == kExternalTrackRequestPending) {
      // 本拍就是新请求：保持 kFresh，并把请求记账到本拍。
      external_track_request_cycle = input.input_cycle_index;
      external_track_last_request_cycle = input.input_cycle_index;
      external_track_coast_cycles = 0U;
      external_track_phase = RirExternalTrackPhase::kFresh;
    } else if (external_track_request_cycle != input.input_cycle_index) {
      // 请求属于更早的周期、本拍又没有新请求：进入外推保持（窗口继续跟随外推中心）。
      external_track_phase = RirExternalTrackPhase::kCoasting;
      ++external_track_coast_cycles;
    }
  }
  config::RirAzimuthElevationDeg external_center_deg;
  RirExternalTrackPhase external_phase = RirExternalTrackPhase::kNone;
  const bool has_external_track = ResolveExternalTrackCenter(
      input.input_cycle_index, static_cast<double>(input.sim_time_sec),
      input.platform_position, &external_center_deg, &external_phase);
  if (dwelling_on_target) {
    runtime::RirDwellPlan designate_dwell;
    designate_dwell.pointing_deg = TargetLookAngles(*designated_target);
    designate_dwell.kind = runtime::RirDwellKind::kDesignate;
    designate_dwell.external_target_id = designated_external_target_id;
    dwell_plan.push_back(designate_dwell);
  }
  if (has_external_track) {
    // 中译：有外部引导时**整个周期都照外部窗口**（2026-10-22 用户裁定，取代 T-04b 的
    //       "搜索保底 4 + 库内跟踪保底 2"分配）——来袭方向随目标变化，按预设扫描光栅
    //       补扫既浪费槽位、检测效果也不好；外部窗口吃掉全部剩余槽位，固定光栅与库内
    //       跟踪本周期不排。
    // 标识：外部优先预算（2026-10-22）。唯一例外是下方 cue 合理性门降级：
    //       cue 判为不可信时只照中心 1 拍，剩余槽位退回搜索，避免坏引导带偏整周期。
    const std::size_t used_slots = dwell_plan.size();
    const std::size_t budget = (max_dwells > used_slots) ? (max_dwells - used_slots) : 0U;
    const std::size_t external_slots = budget;

    // 中译：cue 合理性门——位置跳变超过"最大速度 × 间隔 + 容差"即判为不可信（融合定位偶发
    //       大幅偏离，实测有 4,000+ km 的跳变）。不可信时本周期只照中心 1 拍，把槽位让给搜索。
    // 标识：cue 合理性门（2026-10-20）。
    external_cue_degraded_this_cycle = false;
    if (cue_gate_has_last) {
      const oneq::coordinate::EcefPositionM& previous = cue_gate_last_position;
      const double dx = external_track_request.position_ecef_m.x_m - previous.x_m;
      const double dy = external_track_request.position_ecef_m.y_m - previous.y_m;
      const double dz = external_track_request.position_ecef_m.z_m - previous.z_m;
      const double jump_m = std::sqrt(dx * dx + dy * dy + dz * dz);
      const double elapsed_cycles =
          (external_track_last_request_cycle > cue_gate_last_cycle)
              ? static_cast<double>(external_track_last_request_cycle - cue_gate_last_cycle)
              : 1.0;
      const double allowed_m = kCueGateMaxSpeedMps * elapsed_cycles + kCueGateSlackM;
      external_cue_degraded_this_cycle = jump_m > allowed_m;
    }
    if (!external_cue_degraded_this_cycle) {
      cue_gate_has_last = true;
      cue_gate_last_cycle = external_track_last_request_cycle;
      cue_gate_last_position = external_track_request.position_ecef_m;
    }

    std::vector<config::RirAzimuthElevationDeg> window_points;
    const dwell::RirEffectiveBeamwidthDeg beamwidth = dwell::RirResolveEffectiveBeamwidth(
        config.hardware.antenna, ResolveScanWavelengthM(config));
    BuildExternalTrackWindowDwells(external_center_deg, external_track_request.window, beamwidth,
                                   config.mission.step_scale, kExternalWindowMaxPoints,
                                   &window_points);
    // 新 cue 到来：窗口序列按新中心重生，游标归零 ⇒ 先照中心。
    if (external_phase == RirExternalTrackPhase::kFresh) {
      external_window_cursor = 0U;
    }
    // 中译：降级仍照常吃满窗口——引导可信度只作标注（写进结果层供现场核对），不再把
    //       槽位退回固定光栅：既然本周期唯一的指向来源就是这条引导，退回光栅既找不回
    //       目标、又白扔 8~9 拍（2026-10-22 用户裁定：有外部信息时全部槽位都给外部）。
    // 标识：外部优先（2026-10-22 修订）。
    const std::size_t take_points = std::min(external_slots, window_points.size());
    // 中译：跨周期续扫——按游标从窗口序列中取连续若干点，而不是每周期都从中心重来；
    //       这样 25 点窗口在 13 槽/周期下 2 个周期即可铺满，外圈点不再永远轮不到。
    // 标识：T-04b 外部窗口续扫。
    for (std::size_t index = 0U; index < take_points; ++index) {
      const std::size_t point_index =
          (external_window_cursor + index) % window_points.size();
      const config::RirAzimuthElevationDeg& pt = window_points[point_index];
      runtime::RirDwellPlan ext_dwell;
      ext_dwell.pointing_deg = pt;
      ext_dwell.kind = runtime::RirDwellKind::kExternalTrack;
      // 中译：外部跟踪驻留必须带上来源目标 ID——否则验收日志只能写"外部跟踪驻留 目标ID=0"，
      //       既无法归因到目标，也无法与库内跟踪驻留区分（此前 99% 的驻留行都是这种）。
      // 标识：T-03（2026-10-20）；外部请求本就带 source_target_id（RirTrackRequest.h:83）。
      ext_dwell.external_target_id = external_track_request.source_target_id;
      dwell_plan.push_back(ext_dwell);
    }
    // 中译：窗口游标无论 cue 是否被判降级都推进——降级只影响标注，不再改变照射范围。
    if (!window_points.empty()) {
      external_window_cursor =
          (external_window_cursor + take_points) % window_points.size();
    }
    // 外部窗口吃满预算（窗口有波位）⇒ 本周期不再排库内跟踪与固定光栅。
    external_monopolizes_budget = !window_points.empty();
    ++external_track_sweep_index;
  }
  if (!external_monopolizes_budget && max_dwells > dwell_plan.size() + kMinSearchDwells) {
    std::vector<runtime::RirDwellPlan> track_dwells = controller.CollectTrackDwellRequests(
        max_dwells - dwell_plan.size() - kMinSearchDwells, static_cast<float>(input.dt_sec));
    dwell_plan.insert(dwell_plan.end(), track_dwells.begin(), track_dwells.end());
  }
  const std::vector<oneq::common::radar::AzimuthElevationDeg> scan_pattern =
      BuildAbsoluteScanWaves(config);
  while (!external_monopolizes_budget && dwell_plan.size() < max_dwells) {
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
  // 结果层驻留种类计数（2026-10-22）：让"外部窗口吃满预算"这一策略在结果层可验证，
  // 同时给验收第 37/66 项一个与计划同源的口径。
  std::size_t search_dwell_count = 0U;
  std::size_t designate_dwell_count = 0U;
  std::size_t internal_track_dwell_count = 0U;
  std::size_t external_track_dwell_count = 0U;
  for (const runtime::RirDwellPlan& dwell : dwell_plan) {
    switch (dwell.kind) {
      case runtime::RirDwellKind::kSearch:
        ++search_dwell_count;
        break;
      case runtime::RirDwellKind::kDesignate:
        ++designate_dwell_count;
        break;
      case runtime::RirDwellKind::kTrack:
        ++internal_track_dwell_count;
        break;
      case runtime::RirDwellKind::kExternalTrack:
        ++external_track_dwell_count;
        break;
      default:
        break;
    }
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
    // 2026-09-28 合并：第48项要求记录方位/俯仰扫描范围（波位表上下限），与 CSV 同源。
    runtime::WriteRirScanRangeSummary(controller.EffectiveSensorEntityId(), input.sim_time_sec,
                                      input.input_cycle_index, scan_pattern, config.mission,
                                      config.orientation);
    // 2026-09-28 按主文档补全：第28项从实际方向图采样反算特征并与配置逐项比对（5%）。
    // 早期的“直接打印配置峰值/宽度”摘要已删除——它与反算校验并存会造成自证，且不满足
    // “只记录校验结果”的边界要求。
    runtime::WriteRirAntennaPatternDerived(controller.EffectiveSensorEntityId(),
                                           input.sim_time_sec, input.input_cycle_index,
                                           config.hardware.antenna);
    acceptance_scan_pattern_logged = true;
  }
  if (RIR_ACCEPTANCE_LOG_ENABLED()) {
    runtime::WriteRirDwellScan(controller.EffectiveSensorEntityId(), input.sim_time_sec,
                               input.input_cycle_index, dwell_plan);
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
  // 指向来源与外部跟踪可观测面（加性字段）：让集成方能核对自己提交的位置/速度
  // 是否被正确消费，并区分「外部跟踪指向」与「扫描基准」这两种不同语义。
  if (has_external_track && !dwelling_on_target) {
    result.pointing_source = RirPointingSource::kExternalTrack;
  } else if (dwelling_on_target) {
    result.pointing_source = RirPointingSource::kDesignation;
  } else {
    result.pointing_source = RirPointingSource::kScan;
  }
  result.external_track_center_deg = external_center_deg;
  result.external_track_phase = external_track_phase;
  result.search_dwell_count = static_cast<std::uint32_t>(search_dwell_count);
  result.designate_dwell_count = static_cast<std::uint32_t>(designate_dwell_count);
  result.internal_track_dwell_count = static_cast<std::uint32_t>(internal_track_dwell_count);
  result.external_track_dwell_count = static_cast<std::uint32_t>(external_track_dwell_count);
  result.external_track_sweep_index =
      has_external_track ? external_track_sweep_index : 0U;
  result.external_cue_degraded = has_external_track && external_cue_degraded_this_cycle;
  // 外部跟踪的如实报告（不吸附、不静默）：到期/保持/越界各自可辨识。
  if (external_track_expired_this_cycle) {
    session::RirIssue expired;
    expired.code = session::codes::kExternalTrackExpired;
    expired.message = "external track valid window expired; slot released";
    result.issues.push_back(expired);
  } else if (external_track_active && !external_track_window_expired_this_cycle &&
             external_track_phase == RirExternalTrackPhase::kCoasting) {
    session::RirIssue stale;
    stale.code = session::codes::kExternalTrackStale;
    stale.message = "external track coasting on last velocity (no new request this cycle)";
    result.issues.push_back(stale);
  }
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

bool RirSession::GetDetectionRngDiagnostics(DetectionRngDiagnostics* out) const {
  // 中译：仅作诊断——把"种子施加次数"与"判决次数"交给宿主诊断日志，用于自证
  //       CFAR 抽签没有被每周期重播种冻结。不参与任何数值计算、不写验收日志。
  // 标识：2026-10-20 选项 C（诊断）。
  if (out == nullptr || impl_ == nullptr) {
    return false;
  }
  out->applied_seed = impl_->controller.DetectionRandomSeed();
  out->seed_apply_count = impl_->controller.DetectionSeedApplyCount();
  out->decision_count = impl_->controller.DetectionDecisionCount();
  out->false_decision_count = impl_->controller.DetectionFalseDecisionCount();
  return true;
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
