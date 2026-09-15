#include "1q/remote_identification_radar/session/RirExclusionCauseRecorder.h"

#include <cstddef>
#include <unordered_map>
#include <utility>

#include "1q/foundation/validation_types.h"

namespace remote_identification_radar {
namespace session {

namespace {

// 上一执行周期的排除原因快照（差分原料）。无条目 = 上周未被排除。
// VS2015/MSVC19.0 未实现 DR-1467（NSDMI 聚合初始化，C2440），按项目既有
// 惯例为含 NSDMI 的值类型补充用户声明构造函数，调用点零改动。
struct RirExclusionState {
  std::string code{};
  RirIssueCause cause{RirIssueCause::kNone};
  RirExclusionState() = default;
  RirExclusionState(std::string code_, RirIssueCause cause_)
      : code(std::move(code_)), cause(cause_) {}
};

// 本周期按 entity_index 收集的排除诊断命中（单目标单周期取第一条，见 header @note）。
struct RirCurrentExclusion {
  std::string code{};
  RirIssueCause cause{RirIssueCause::kNone};
  bool found{false};
  RirCurrentExclusion() = default;
  RirCurrentExclusion(std::string code_, RirIssueCause cause_, bool found_)
      : code(std::move(code_)), cause(cause_), found(found_) {}
};

// 按 location.entity_index 索引本周期排除诊断。仅消费 kSceneEntity 定位且
// phase=kExecution 的排除 issue（规则 13b 门内归因条目）；输入校验 issues
// （phase=kInputValidation）虽也可能用 kSceneEntity 定位，但属调用方输入问题
// 而非执行期排除，不在此列。一个 entity_index 命中多条时取第一条（RIR 执行链
// 每目标每周期至多落一条排除诊断，假设见 header @note）。
std::unordered_map<std::size_t, RirCurrentExclusion> RirIndexCurrentExclusions(
    const RirCycleResult& result) {
  std::unordered_map<std::size_t, RirCurrentExclusion> by_entity;
  for (const RirIssue& issue : result.issues) {
    if (issue.phase != RirIssuePhase::kExecution ||
        issue.location.kind != oneq::foundation::ValidationLocationKind::kSceneEntity) {
      continue;
    }
    const std::size_t idx = issue.location.entity_index;
    if (by_entity.count(idx) == 0U) {
      by_entity[idx] = RirCurrentExclusion{issue.code, issue.cause, true};
    }
  }
  return by_entity;
}

}  // namespace

struct RirExclusionCauseRecorder::Impl {
  std::unordered_map<std::uint64_t, RirExclusionState> states;
  std::vector<RirExclusionCauseEvent> last_events{};
};

RirExclusionCauseRecorder::RirExclusionCauseRecorder() : impl_(new Impl) {}

RirExclusionCauseRecorder::~RirExclusionCauseRecorder() = default;

RirExclusionCauseRecorder::RirExclusionCauseRecorder(RirExclusionCauseRecorder&&) noexcept =
    default;
RirExclusionCauseRecorder& RirExclusionCauseRecorder::operator=(
    RirExclusionCauseRecorder&&) noexcept = default;

std::vector<RirExclusionCauseEvent> RirExclusionCauseRecorder::Update(
    const RirSceneTargetList& targets, const RirCycleResult& result) {
  std::vector<RirExclusionCauseEvent> events;
  if (result.status != RirCycleStatus::kCompleted) {
    return events;
  }
  const std::unordered_map<std::size_t, RirCurrentExclusion> current_by_entity =
      RirIndexCurrentExclusions(result);
  events.reserve(targets.size());
  for (std::size_t idx = 0; idx < targets.size(); ++idx) {
    const RirSceneTarget& target = targets[idx];
    // external_target_id 为 0 的输入目标无法按 ID 关联，跳过差分记录。
    if (target.external_target_id == 0U) {
      continue;
    }

    typename std::unordered_map<std::size_t, RirCurrentExclusion>::const_iterator current_it =
        current_by_entity.find(idx);
    const bool excluded_now = current_it != current_by_entity.end() && current_it->second.found;
    typename std::unordered_map<std::uint64_t, RirExclusionState>::iterator prev_it =
        impl_->states.find(target.external_target_id);
    const bool excluded_prev = prev_it != impl_->states.end();

    if (excluded_now && !excluded_prev) {
      // A2：未被排除 → 被排除。
      RirExclusionCauseEvent event;
      event.world_cycle_index = result.input_cycle_index;
      event.external_target_id = target.external_target_id;
      event.target_name = target.target_name;
      event.kind = RirExclusionCauseEventKind::kEntered;
      event.current_code = current_it->second.code;
      event.current_cause = current_it->second.cause;
      events.push_back(event);
      impl_->states[target.external_target_id] =
          RirExclusionState{current_it->second.code, current_it->second.cause};
    } else if (excluded_now && excluded_prev) {
      // A3：被排除中 (code,cause) 对变化。A1（相同）不产事件。
      const bool code_changed = prev_it->second.code != current_it->second.code;
      const bool cause_changed = prev_it->second.cause != current_it->second.cause;
      if (code_changed || cause_changed) {
        RirExclusionCauseEvent event;
        event.world_cycle_index = result.input_cycle_index;
        event.external_target_id = target.external_target_id;
        event.target_name = target.target_name;
        event.kind = RirExclusionCauseEventKind::kChanged;
        event.previous_code = prev_it->second.code;
        event.previous_cause = prev_it->second.cause;
        event.current_code = current_it->second.code;
        event.current_cause = current_it->second.cause;
        events.push_back(event);
      }
      prev_it->second.code = current_it->second.code;
      prev_it->second.cause = current_it->second.cause;
    } else if (!excluded_now && excluded_prev) {
      // A4：被排除 → 不再被排除。
      RirExclusionCauseEvent event;
      event.world_cycle_index = result.input_cycle_index;
      event.external_target_id = target.external_target_id;
      event.target_name = target.target_name;
      event.kind = RirExclusionCauseEventKind::kExited;
      event.previous_code = prev_it->second.code;
      event.previous_cause = prev_it->second.cause;
      events.push_back(event);
      impl_->states.erase(prev_it);
    }
    // !excluded_now && !excluded_prev：持续未被排除，不产事件、无状态变化。
  }
  impl_->last_events = events;
  return events;
}

void RirExclusionCauseRecorder::Reset() {
  impl_->states.clear();
  impl_->last_events.clear();
}

const std::vector<RirExclusionCauseEvent>& RirExclusionCauseRecorder::GetLastEvents()
    const noexcept {
  return impl_->last_events;
}

}  // namespace session
}  // namespace remote_identification_radar
