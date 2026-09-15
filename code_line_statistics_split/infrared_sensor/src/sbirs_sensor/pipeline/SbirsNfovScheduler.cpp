/**
 * @file SbirsNfovScheduler.cpp
 * @brief NFOV 锁定集合管理器实现。
 */

#include "sbirs_sensor/pipeline/SbirsNfovScheduler.h"

#include <algorithm>

namespace sbirs_sensor {
namespace pipeline {

bool SbirsNfovScheduler::IsLocked(std::uint64_t target_id) const {
  return target_to_cue_source_.find(target_id) != target_to_cue_source_.end();
}

bool SbirsNfovScheduler::Acquire(std::uint64_t target_id, int cue_source_satellite_entity_id) {
  target_to_cue_source_[target_id] = cue_source_satellite_entity_id;
  return true;
}

int SbirsNfovScheduler::CueSourceOf(std::uint64_t target_id) const {
  const auto it = target_to_cue_source_.find(target_id);
  return it == target_to_cue_source_.end() ? -1 : it->second;
}

void SbirsNfovScheduler::Release(std::uint64_t target_id) {
  target_to_cue_source_.erase(target_id);
}

void SbirsNfovScheduler::Clear() { target_to_cue_source_.clear(); }

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
  std::vector<const SbirsCandidate*> selected;
  for (const SbirsCandidate& candidate : candidates) {
    if (!IsLocked(candidate.target->target_id)) {
      selected.push_back(&candidate);
    }
  }
  return selected;
}

SbirsNfovSchedulerSnapshot SbirsNfovScheduler::Capture() const {
  SbirsNfovSchedulerSnapshot snapshot;
  snapshot.target_to_cue_source = target_to_cue_source_;
  return snapshot;
}

void SbirsNfovScheduler::Restore(const SbirsNfovSchedulerSnapshot& snapshot) {
  target_to_cue_source_ = snapshot.target_to_cue_source;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
