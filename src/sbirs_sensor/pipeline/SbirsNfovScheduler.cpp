/**
 * @file SbirsNfovScheduler.cpp
 * @brief NFOV 锁定集合管理器实现。
 */

/**
 * ## 过程：单镜筒窄场调度（候选排序 → 锁定集合）（红外处理链 · 第 9 步）
 * - 上游：pipeline 的 WFOV 发现阶段填充的 `SbirsCandidate` 列表（含 SNR、真值/量测斜距、
 *   延迟真值角、引导来源星 ID）以及当前锁定集合。
 * - 本步：按 design 2.6 的固定优先级 SNR 降序 → 距离升序 → target_id 升序就地排序，剔除已锁定
 *   目标，给出"可进入首次捕获"的顺序（单镜筒不做并发截断，容量由分时轮转的物理时序涌现）；
 *   同时维护 target_id → 引导来源星 ID 的映射（-1=自星宽场，其它=他星 cross-cue）。
 * - 下游：pipeline 按返回顺序逐个尝试捕获与指向轮转；`IsLocked`/`CueSourceOf` 决定目标是否走
 *   窄场分时量测、以及归属层的引导来源字段；快照进 checkpoint/replay。
 * - 关键参数：本类无独立配置项；宽窄切换前置的连续命中次数由
 *   `policy.scheduler.wide_to_narrow_required_consecutive_hits`(默认 1) 在 pipeline 侧控制，
 *   候选门限来自 `policy.detection.wide_min_snr_linear`(4)。
 * - 对应验收项：第 13 项 窄视场跟踪探测功能测试；第 14 项 宽窄视场联合探测功能测试（调度顺序）；
 *   第 76 项 红外宽场候选与窄场调度交互接口测试（候选状态/当前服务目标）。
 */

#include "sbirs_sensor/pipeline/SbirsNfovScheduler.h"

#include <algorithm>

namespace sbirs_sensor {
namespace pipeline {

// 过程：锁定判据就是映射表里有没有这个 target_id（map 有序，遍历顺序确定）。
bool SbirsNfovScheduler::IsLocked(std::uint64_t target_id) const {
  return target_to_cue_source_.find(target_id) != target_to_cue_source_.end();
}

// 过程：登记锁定并记下引导来源星；重复登记覆盖为同值，故天然幂等。
bool SbirsNfovScheduler::Acquire(std::uint64_t target_id, int cue_source_satellite_entity_id) {
  target_to_cue_source_[target_id] = cue_source_satellite_entity_id;
  return true;
}

// 过程：查引导来源；未锁定返回 -1，与自星宽场同值，调用方靠是否锁定区分两者。
int SbirsNfovScheduler::CueSourceOf(std::uint64_t target_id) const {
  const auto it = target_to_cue_source_.find(target_id);
  return it == target_to_cue_source_.end() ? -1 : it->second;
}

// 过程：单点清除 —— 目标消失、门丢锁、切 standby 都走这里，锁定与引导来源一起失效。
void SbirsNfovScheduler::Release(std::uint64_t target_id) {
  target_to_cue_source_.erase(target_id);
}

void SbirsNfovScheduler::Clear() { target_to_cue_source_.clear(); }

// 过程：按 map 键升序导出锁定目标（确定性顺序，replay 与日志不受容器实现影响）。
std::vector<std::uint64_t> SbirsNfovScheduler::LockedTargetIds() const {
  std::vector<std::uint64_t> ids;
  ids.reserve(target_to_cue_source_.size());
  for (const auto& entry : target_to_cue_source_) {
    ids.push_back(entry.first);
  }
  return ids;
}

std::vector<const SbirsCandidate*> SbirsNfovScheduler::SelectForAcquisition(
    std::vector<SbirsCandidate>& candidates) {
  // design 2.6 优先级：SNR 降序 → 距离升序 → target_id 升序（稳定，保证 replay 可复现）。
  std::sort(candidates.begin(), candidates.end(), [](const SbirsCandidate& lhs, const SbirsCandidate& rhs) {
    if (lhs.snr != rhs.snr) {
      return lhs.snr > rhs.snr;
    }
    if (lhs.range_m != rhs.range_m) {
      return lhs.range_m < rhs.range_m;
    }
    return lhs.target->target_id < rhs.target->target_id;
  });
  // 过程：已锁定目标不再入选（防重复捕获）；单镜筒不做并发截断，能同时跟几条由轮转时序决定。
  std::vector<const SbirsCandidate*> selected;
  for (const SbirsCandidate& candidate : candidates) {
    if (!IsLocked(candidate.target->target_id)) {
      selected.push_back(&candidate);
    }
  }
  return selected;
}

// 过程：快照只含锁定集合关系；滤波/IMM 状态由跟踪协调器各自管理，两者分开持久化。
SbirsNfovSchedulerSnapshot SbirsNfovScheduler::Capture() const {
  SbirsNfovSchedulerSnapshot snapshot;
  snapshot.target_to_cue_source = target_to_cue_source_;
  return snapshot;
}

// 过程：整体替换锁定集合（不做增量合并），保证恢复后的顺序与快照一致。
void SbirsNfovScheduler::Restore(const SbirsNfovSchedulerSnapshot& snapshot) {
  target_to_cue_source_ = snapshot.target_to_cue_source;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
