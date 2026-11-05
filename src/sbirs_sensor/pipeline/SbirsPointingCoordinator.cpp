/**
 * ## 过程：指向合成与 ATP 决策（红外处理链 · 第 12 步）
 * - 上游：pipeline 窄场轮转段给出的指令光轴（ECI 单位向量，已按扫描限位钳制）、
 *   `mission.narrow_pointing_max_slew_rate_deg_per_sec` 与
 *   `mission.narrow_pointing_settle_tolerance_deg` 组装的执行机构配置、本周期扰动参数。
 * - 本步：把"要指向谁"翻译成"这一拍能指向哪"：
 *   · 舵机按转动限速推进（第 10 步），稳定判据 = 剩余角 ≤ 稳定容差；
 *   · 捕获态逐目标累计等待时长，超时门 = 180°/最大转速（最坏转半圈的兜底上限）；
 *   · 跟踪态不累计等待时长，只透出舵机结果；跟踪门连续失败次数在此记账；
 *   · 共模/逐通道扰动（第 11 步）随周期推进并逐帧采样。
 * - 下游：pipeline 依返回的 current_los/状态/已服务时长决定捕获还是继续转向，并合成实际指向角
 *   （含扰动与静态 settle 误差）；快照进 checkpoint/replay。
 * - 关键参数：`max_slew_rate_deg_per_sec`(30°/s)、`settle_tolerance_deg`、
 *   `pointing_disturbance.*`（默认全 0，即理想指向）、`pointing_disturbance.random_seed`。
 * - 对应验收项：第 13 项 窄视场跟踪探测功能测试；第 14 项 宽窄视场联合探测功能测试（镜头转向
 *   时间、指向稳定判据）；第 76 项 红外宽场候选与窄场调度交互接口测试（指令角/指向状态/剩余角）。
 */

#include "sbirs_sensor/pipeline/SbirsPointingCoordinator.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace sbirs_sensor {
namespace pipeline {

// 过程：构造只持有一个带种子的扰动源（单通道），舵机状态等到首次指向时才初始化。
SbirsPointingCoordinator::SbirsPointingCoordinator(std::uint32_t disturbance_seed)
    : disturbance_(1, disturbance_seed) {}

// 过程：舵机惰性初始化 —— 首次以当前名义扫描光轴为起点（零转角即稳定），之后不再重置，
//       保证转向从扫描中心起算而不是凭空跳变。
bool SbirsPointingCoordinator::EnsureActuatorInitialized(
    const session::SbirsVector3M& initial_los) {
  if (actuator_initialized_) {
    return true;
  }
  if (!actuator_.Initialize(initial_los)) {
    return false;
  }
  actuator_initialized_ = true;
  return true;
}

// 过程：捕获态推进 —— 每拍累加该目标的等待时长，返回舵机状态与本拍已服务时长。
SbirsPointingAdvanceResult SbirsPointingCoordinator::AdvanceAcquisition(
    std::uint64_t target_id, const session::SbirsVector3M& command_los, double dt_sec,
    const SbirsPointingActuatorConfig& config) {
  SbirsPointingAdvanceResult result;
  if (!actuator_initialized_) {
    return result;
  }
  SbirsPointingActuatorResult actuator_result;
  if (!actuator_.Step(command_los, dt_sec, config, &actuator_result)) {
    return result;
  }

  const double next_elapsed = acquisition_wait_sec_[target_id] + dt_sec;
  acquisition_wait_sec_[target_id] = next_elapsed;
  result.current_los = actuator_result.current_los;
  result.remaining_angle_deg = actuator_result.remaining_angle_deg;
  result.settled_duration_sec = actuator_result.settled_duration_sec;
  result.elapsed_wait_sec = next_elapsed;
  if (actuator_result.settled) {
    result.status = SbirsPointingAdvanceStatus::kSettled;
    return result;
  }

  // 过程：超时门取转半圈（180°）所需时间 = 180/最大转速，是与视场无关的最坏情况上限：
  //       真转不到位时宁可报超时让 pipeline 释放目标，也不无限期占用单镜筒。
  const double timeout_sec = 180.0 / config.max_slew_rate_deg_per_sec;
  if (next_elapsed >= timeout_sec) {
    acquisition_wait_sec_.erase(target_id);
    result.status = SbirsPointingAdvanceStatus::kTimedOut;
    return result;
  }
  result.status = SbirsPointingAdvanceStatus::kSlewing;
  return result;
}

// 过程：跟踪态推进 —— 逻辑与捕获态相同，但不累计等待时长（elapsed_wait_sec 恒 0），
//       稳定与否直接由舵机剩余角与容差判定。
SbirsPointingAdvanceResult SbirsPointingCoordinator::AdvanceTracking(
    const session::SbirsVector3M& command_los, double dt_sec,
    const SbirsPointingActuatorConfig& config) {
  SbirsPointingAdvanceResult result;
  if (!actuator_initialized_) {
    return result;
  }
  SbirsPointingActuatorResult actuator_result;
  if (!actuator_.Step(command_los, dt_sec, config, &actuator_result)) {
    return result;
  }
  result.current_los = actuator_result.current_los;
  result.remaining_angle_deg = actuator_result.remaining_angle_deg;
  result.settled_duration_sec = actuator_result.settled_duration_sec;
  result.elapsed_wait_sec = 0.0;
  result.status = actuator_result.settled ? SbirsPointingAdvanceStatus::kSettled
                                          : SbirsPointingAdvanceStatus::kSlewing;
  return result;
}

// 过程：捕获成功转跟踪即清掉等待计时，避免下次重捕沿用旧累计值提前触发超时。
bool SbirsPointingCoordinator::PromoteToTracking(std::uint64_t target_id) {
  acquisition_wait_sec_.erase(target_id);
  return true;
}

// 过程：跟踪门（窄场几何 + SNR）连续失败计数 —— 通过即清零，连续失败达阈值的丢锁判定在 pipeline。
unsigned int SbirsPointingCoordinator::RecordTrackingGateResult(std::uint64_t target_id,
                                                                bool gate_passed) {
  if (gate_passed) {
    tracking_gate_failure_counts_[target_id] = 0U;
  } else {
    ++tracking_gate_failure_counts_[target_id];
  }
  return tracking_gate_failure_counts_[target_id];
}

// 过程：扰动过程按本周期 dt 推进一次；采样必须在其后取，否则用的是上一拍姿态。
bool SbirsPointingCoordinator::AdvanceDisturbance(
    double dt_sec, const SbirsPointingDisturbanceParameters& parameters) {
  return disturbance_.Advance(dt_sec, parameters);
}

// 过程：只读采样当前扰动角（共模 + 通道），不推进状态 —— 同一拍可被多处复用。
bool SbirsPointingCoordinator::DisturbanceSample(
    int channel_id, const SbirsPointingDisturbanceParameters& parameters,
    SbirsPointingDisturbanceSample* sample) const {
  return disturbance_.Sample(channel_id, parameters, sample);
}

// 过程：配置重载/换种子时重建扰动源，共模与各通道随机流全部回到初始状态。
void SbirsPointingCoordinator::RestartDisturbance(std::uint32_t disturbance_seed) {
  disturbance_ = SbirsPointingDisturbance(1, disturbance_seed);
}

// 过程：释放目标时两本账同时清（等待计时与门失败计数），返回是否本来有记录。
bool SbirsPointingCoordinator::ReleaseTarget(std::uint64_t target_id) {
  const bool had_entries =
      acquisition_wait_sec_.erase(target_id) != 0U ||
      tracking_gate_failure_counts_.erase(target_id) != 0U;
  return had_entries;
}

// 过程：只清失败计数、不清滤波器状态（诊断复位语义），供重载配置后重建基线。
void SbirsPointingCoordinator::ResetTrackingGateFailureCounts() {
  tracking_gate_failure_counts_.clear();
}

// 过程：standby 清空全部指向状态，但取出扰动基础种子重新播种，保持随机流可复现。
void SbirsPointingCoordinator::Clear() {
  const std::uint32_t seed = disturbance_.Capture().base_seed;
  actuator_ = SbirsPointingActuator{};
  actuator_initialized_ = false;
  acquisition_wait_sec_.clear();
  tracking_gate_failure_counts_.clear();
  disturbance_ = SbirsPointingDisturbance(1, seed);
}

// 过程：快照 = 舵机状态 + 两本账 + 扰动状态；舵机是否初始化单独记，恢复时用于一致性校验。
SbirsPointingCoordinatorSnapshot SbirsPointingCoordinator::Capture() const {
  SbirsPointingCoordinatorSnapshot snapshot;
  snapshot.actuator_initialized = actuator_initialized_;
  snapshot.actuator = actuator_.Capture();
  snapshot.acquisition_wait_sec = acquisition_wait_sec_;
  snapshot.tracking_gate_failure_counts = tracking_gate_failure_counts_;
  snapshot.disturbance = disturbance_.Capture();
  return snapshot;
}

// 过程：恢复前逐项校验 —— 扰动/舵机快照须自洽（初始化标志与状态一致）、等待时长须有限
//       非负、两本账的目标集合不得重叠；任一不满足即整体拒绝，不留半套状态。
bool SbirsPointingCoordinator::Restore(const SbirsPointingCoordinatorSnapshot& snapshot) {
  SbirsPointingDisturbance restored_disturbance(1, 1U);
  if (!restored_disturbance.Restore(snapshot.disturbance)) {
    return false;
  }
  SbirsPointingActuator restored_actuator;
  if (!restored_actuator.Restore(snapshot.actuator)) {
    return false;
  }
  if (snapshot.actuator_initialized && !snapshot.actuator.initialized) {
    return false;
  }
  std::set<std::uint64_t> bookkeeping_targets;
  for (const auto& entry : snapshot.acquisition_wait_sec) {
    if (!std::isfinite(entry.second) || entry.second < 0.0 ||
        !bookkeeping_targets.insert(entry.first).second) {
      return false;
    }
  }
  for (const auto& entry : snapshot.tracking_gate_failure_counts) {
    if (!bookkeeping_targets.insert(entry.first).second) {
      return false;
    }
  }
  actuator_ = restored_actuator;
  actuator_initialized_ = snapshot.actuator_initialized;
  acquisition_wait_sec_ = snapshot.acquisition_wait_sec;
  tracking_gate_failure_counts_ = snapshot.tracking_gate_failure_counts;
  disturbance_ = restored_disturbance;
  return true;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
