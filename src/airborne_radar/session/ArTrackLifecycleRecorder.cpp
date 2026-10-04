#include "1q/airborne_radar/session/ArTrackLifecycleRecorder.h"

#include <unordered_map>
#include <utility>

namespace airborne_radar {
namespace session {

namespace {

struct TargetState {
  bool confirmed{false};
  bool designation_active{false}; /**< 上一周期该目标是否为生效的指定跟踪目标。 */
  bool designation_pending{false}; /**< 上一周期该目标是否处于"指定但未生效（回退）"状态。 */
  std::string target_name{};
};

const session::TrackStateSnapshot* FindTrackByExternalTargetId(
    const TrackOutputFrame& frame, std::uint64_t external_target_id) {
  for (const session::TrackStateSnapshot& track : frame.tracks) {
    if (track.external_target_id == external_target_id && external_target_id != 0U) {
      return &track;
    }
  }
  return nullptr;
}

ArTrackLifecycleReason InferReason(const session::TrackStateSnapshot* track) {
  if (track == nullptr) {
    return ArTrackLifecycleReason::kNoTrack;
  }
  return ArTrackLifecycleReason::kUnknown;
}

ArTrackLifecycleEvent MakeBaseEvent(const ArTargetInput& target,
                                    const ArCycleResult& result) {
  ArTrackLifecycleEvent event;
  event.world_cycle_index = result.input_cycle_index;
  event.external_target_id = target.target_id;
  event.target_name = target.target_name;
  return event;
}

}  // namespace

struct ArTrackLifecycleRecorder::Impl {
  ArTrackLifecycleRecorderConfig config;
  std::unordered_map<std::uint64_t, TargetState> states;
  std::vector<ArTrackLifecycleEvent> last_events{};
  Impl() = default;
  Impl(ArTrackLifecycleRecorderConfig config_,
       std::unordered_map<std::uint64_t, TargetState> states_ = {})
      : config(config_), states(std::move(states_)) {}
};

ArTrackLifecycleRecorder::ArTrackLifecycleRecorder(ArTrackLifecycleRecorderConfig config)
    : impl_(new Impl{config, {}}) {}

ArTrackLifecycleRecorder::~ArTrackLifecycleRecorder() = default;

ArTrackLifecycleRecorder::ArTrackLifecycleRecorder(ArTrackLifecycleRecorder&& other) noexcept : impl_(std::move(other.impl_)) {}
ArTrackLifecycleRecorder& ArTrackLifecycleRecorder::operator=(ArTrackLifecycleRecorder&& other) noexcept { impl_ = std::move(other.impl_); return *this; }

std::vector<ArTrackLifecycleEvent> ArTrackLifecycleRecorder::Update(
    const ArTargetInputList& targets, const ArCycleResult& result) {
  std::vector<ArTrackLifecycleEvent> events;
  if (result.status != ArCycleStatus::kCompleted) {
    return events;
  }
  events.reserve(targets.size());
  for (const ArTargetInput& target : targets) {
    // external_target_id 为 0 的输入目标无法按 ID 关联，跳过生命周期记录。
    if (target.target_id == 0U) {
      continue;
    }
    TargetState& state = impl_->states[target.target_id];
    const session::TrackStateSnapshot* track =
        FindTrackByExternalTargetId(result.output_frame, target.target_id);

    const bool designation_active_now =
        result.designated_target_id == target.target_id && result.designation_active;
    const bool designation_pending_now =
        result.designated_target_id == target.target_id && !result.designation_active &&
        result.designation_reverted_to_tws;
    // 指定跟踪回退事件（自动丢跟踪，回 TWS）：仅当该目标为本周期指定的目标、
    // 上一周期跟踪生效且本周期已回退时，在转换沿产生 kDesignationDropped。
    // 成因直接转写 L2 结果字段（ArCycleResult::designation_revert_reason）。
    // 注意：本块必须在 confirmed 分支的 continue 之前执行——confirmed 目标
    // 同样需要推进 designation_active 状态，否则回退沿永远无法触发。
    if (state.designation_active && !designation_active_now &&
        result.designated_target_id == target.target_id &&
        result.designation_reverted_to_tws) {
      ArTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = ArTrackLifecycleEventKind::kDesignationDropped;
      event.reason = ArTrackLifecycleReason::kNone;
      if (track != nullptr) {
        event.association_key = track->association_key;
        event.track_status = track->status;
        event.speed = track->speed;
      }
      event.designation_revert_reason = result.designation_revert_reason;
      events.push_back(event);
    }
    // 限时指令捕获超时事件（窗口耗尽指令作废）：上一周期与本周期的指定目标
    // 均处于"指定但未生效（回退）"状态，且本周期成因变为 kAcquisitionTimeout
    // 时，在作废沿产生 kDesignationDropped（成因 kAcquisitionTimeout）。
    // 跟随后丢失的既有回退沿不重复触发（该路径成因非 kAcquisitionTimeout）。
    if (state.designation_pending && designation_pending_now &&
        result.designation_revert_reason == ArDesignationRevertReason::kAcquisitionTimeout) {
      ArTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = ArTrackLifecycleEventKind::kDesignationDropped;
      event.reason = ArTrackLifecycleReason::kNone;
      if (track != nullptr) {
        event.association_key = track->association_key;
        event.track_status = track->status;
        event.speed = track->speed;
      }
      event.designation_revert_reason = result.designation_revert_reason;
      events.push_back(event);
    }
    state.designation_active = designation_active_now;
    state.designation_pending = designation_pending_now;

    const bool confirmed_now =
        track != nullptr && track->status == session::TrackStatus::kConfirmed;

    if (confirmed_now) {
      ArTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = state.confirmed ? ArTrackLifecycleEventKind::kUpdated
                                   : ArTrackLifecycleEventKind::kFirstConfirmed;
      event.reason = ArTrackLifecycleReason::kNone;
      event.association_key = track->association_key;
      event.track_status = track->status;
      event.speed = track->speed;
      events.push_back(event);
      state.confirmed = true;
      state.target_name = target.target_name;
      continue;
    }

    const bool lost_now = track != nullptr && track->status == session::TrackStatus::kLost;
    if (lost_now && state.confirmed) {
      ArTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = ArTrackLifecycleEventKind::kLost;
      event.reason = ArTrackLifecycleReason::kNone;
      event.association_key = track->association_key;
      event.track_status = track->status;
      event.speed = track->speed;
      events.push_back(event);
    } else if (!state.confirmed && impl_->config.emit_not_tracked_events) {
      ArTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = ArTrackLifecycleEventKind::kNotTracked;
      event.reason = InferReason(track);
      if (track != nullptr) {
        event.association_key = track->association_key;
        event.track_status = track->status;
        event.speed = track->speed;
      }
      events.push_back(event);
    }

    // 进入 lost 后不再视为已确认，后续若无 track 可重新触发首次确认。
    if (lost_now) {
      state.confirmed = false;
    }
    state.target_name = target.target_name;
  }
  impl_->last_events = events;
  return events;
}

void ArTrackLifecycleRecorder::Reset() {
  impl_->states.clear();
  impl_->last_events.clear();
}

const std::vector<ArTrackLifecycleEvent>& ArTrackLifecycleRecorder::GetLastEvents() const noexcept {
  return impl_->last_events;
}

}  // namespace session
}  // namespace airborne_radar
