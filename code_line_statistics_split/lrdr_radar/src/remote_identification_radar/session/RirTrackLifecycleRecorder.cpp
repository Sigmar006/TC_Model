#include "1q/remote_identification_radar/session/RirTrackLifecycleRecorder.h"

#include <unordered_map>
#include <utility>

namespace remote_identification_radar {
namespace session {

namespace {

struct TargetState {
  bool confirmed{false};
  bool designation_active{false}; /**< 上一周期该目标是否为生效的指定识别目标。 */
  bool designation_pending{false}; /**< 上一周期该目标是否处于"指定但未生效（回扫）"状态。 */
  std::string target_name{};
};

const RirTrackAttributionRecord* FindAttributionByExternalTargetId(
    const RirCycleResult& result, std::uint64_t external_target_id) {
  for (const RirTrackAttributionRecord& attribution : result.track_attributions) {
    if (attribution.external_target_id == external_target_id && external_target_id != 0U) {
      return &attribution;
    }
  }
  return nullptr;
}

RirTrackLifecycleReason InferReason(const RirTrackAttributionRecord* attribution) {
  if (attribution == nullptr) {
    return RirTrackLifecycleReason::kNoTrack;
  }
  return RirTrackLifecycleReason::kUnknown;
}

RirTrackLifecycleEvent MakeBaseEvent(const RirSceneTarget& target, const RirCycleResult& result) {
  RirTrackLifecycleEvent event;
  event.world_cycle_index = result.input_cycle_index;
  event.external_target_id = target.external_target_id;
  event.target_name = target.target_name;
  return event;
}

void FillTrackFields(const RirTrackAttributionRecord* attribution, RirTrackLifecycleEvent* event) {
  if (attribution == nullptr) {
    return;
  }
  event->association_key = attribution->association_key;
  event->track_status = attribution->track_status;
  event->speed_m_per_s = attribution->speed_m_per_s;
}

}  // namespace

struct RirTrackLifecycleRecorder::Impl {
  RirTrackLifecycleRecorderConfig config;
  std::unordered_map<std::uint64_t, TargetState> states;
  std::vector<RirTrackLifecycleEvent> last_events{};
  Impl() = default;
  Impl(RirTrackLifecycleRecorderConfig config_,
       std::unordered_map<std::uint64_t, TargetState> states_ = {})
      : config(config_), states(std::move(states_)) {}
};

RirTrackLifecycleRecorder::RirTrackLifecycleRecorder(RirTrackLifecycleRecorderConfig config)
    : impl_(new Impl{config, {}}) {}

RirTrackLifecycleRecorder::~RirTrackLifecycleRecorder() = default;

RirTrackLifecycleRecorder::RirTrackLifecycleRecorder(RirTrackLifecycleRecorder&&) noexcept =
    default;
RirTrackLifecycleRecorder& RirTrackLifecycleRecorder::operator=(
    RirTrackLifecycleRecorder&&) noexcept = default;

std::vector<RirTrackLifecycleEvent> RirTrackLifecycleRecorder::Update(
    const RirSceneTargetList& targets, const RirCycleResult& result) {
  std::vector<RirTrackLifecycleEvent> events;
  if (result.status != RirCycleStatus::kCompleted) {
    return events;
  }
  events.reserve(targets.size());
  for (const RirSceneTarget& target : targets) {
    // external_target_id 为 0 的输入目标无法按 ID 关联，跳过生命周期记录。
    if (target.external_target_id == 0U) {
      continue;
    }
    TargetState& state = impl_->states[target.external_target_id];
    const RirTrackAttributionRecord* attribution =
        FindAttributionByExternalTargetId(result, target.external_target_id);

    const bool designation_active_now =
        result.designated_target_id == target.external_target_id && result.designation_active;
    const bool designation_pending_now =
        result.designated_target_id == target.external_target_id &&
        !result.designation_active && result.designation_reverted_to_scan;
    // 指定识别任务回扫事件（作废/回扫终态）：仅当该目标为本周期指定的目标、
    // 上一周期驻留生效且本周期已回扫时，在转换沿产生 kDesignationDropped。
    // 成因直接转写信封字段（RirCycleResult::designation_revert_reason）。
    // 注意：本块必须在 confirmed 分支的 continue 之前执行——已确认目标
    // 同样需要推进 designation_active 状态，否则回扫沿永远无法触发。
    if (state.designation_active && !designation_active_now &&
        result.designated_target_id == target.external_target_id &&
        result.designation_reverted_to_scan) {
      RirTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = RirTrackLifecycleEventKind::kDesignationDropped;
      event.reason = RirTrackLifecycleReason::kNone;
      FillTrackFields(attribution, &event);
      event.designation_revert_reason = result.designation_revert_reason;
      events.push_back(event);
    }
    // 限时指令窗口耗尽事件（窗口耗尽任务作废）：上一周期与本周期的指定目标
    // 均处于"指定但未生效（回扫）"状态，且本周期成因变为 kAcquisitionTimeout
    // 时，在作废沿产生 kDesignationDropped（成因 kAcquisitionTimeout）。
    // 驻留期内的既有回扫沿不重复触发（该路径成因非 kAcquisitionTimeout）。
    if (state.designation_pending && designation_pending_now &&
        result.designation_revert_reason ==
            RirDesignationRevertReason::kAcquisitionTimeout) {
      RirTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = RirTrackLifecycleEventKind::kDesignationDropped;
      event.reason = RirTrackLifecycleReason::kNone;
      FillTrackFields(attribution, &event);
      event.designation_revert_reason = result.designation_revert_reason;
      events.push_back(event);
    }
    state.designation_active = designation_active_now;
    state.designation_pending = designation_pending_now;

    const bool confirmed_now =
        attribution != nullptr &&
        attribution->track_status == RirTrackLifecycleStatus::kConfirmed;

    if (confirmed_now) {
      RirTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = state.confirmed ? RirTrackLifecycleEventKind::kUpdated
                                   : RirTrackLifecycleEventKind::kFirstConfirmed;
      event.reason = RirTrackLifecycleReason::kNone;
      event.association_key = attribution->association_key;
      event.track_status = attribution->track_status;
      event.speed_m_per_s = attribution->speed_m_per_s;
      events.push_back(event);
      state.confirmed = true;
      state.target_name = target.target_name;
      continue;
    }

    const bool lost_now =
        attribution != nullptr && attribution->track_status == RirTrackLifecycleStatus::kLost;
    if (lost_now && state.confirmed) {
      RirTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = RirTrackLifecycleEventKind::kLost;
      event.reason = RirTrackLifecycleReason::kNone;
      event.association_key = attribution->association_key;
      event.track_status = attribution->track_status;
      event.speed_m_per_s = attribution->speed_m_per_s;
      events.push_back(event);
    } else if (!state.confirmed && impl_->config.emit_not_tracked_events) {
      RirTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = RirTrackLifecycleEventKind::kNotTracked;
      event.reason = InferReason(attribution);
      FillTrackFields(attribution, &event);
      events.push_back(event);
    }

    // 进入 lost 后不再视为已确认，后续若无航迹可重新触发首次确认。
    if (lost_now) {
      state.confirmed = false;
    }
    state.target_name = target.target_name;
  }
  impl_->last_events = events;
  return events;
}

void RirTrackLifecycleRecorder::Reset() {
  impl_->states.clear();
  impl_->last_events.clear();
}

const std::vector<RirTrackLifecycleEvent>& RirTrackLifecycleRecorder::GetLastEvents() const
    noexcept {
  return impl_->last_events;
}

}  // namespace session
}  // namespace remote_identification_radar
