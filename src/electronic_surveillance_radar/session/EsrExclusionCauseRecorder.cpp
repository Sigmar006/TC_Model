#include "1q/electronic_surveillance_radar/session/EsrExclusionCauseRecorder.h"

#include <algorithm>
#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/foundation/validation_types.h"

namespace electronic_surveillance_radar {
namespace session {

namespace {

// 发射源标识二元组（用作 unordered_map 键）。identity 唯一性由帧校验保证。
// VS2015/MSVC19.0 未实现 DR-1467（NSDMI 聚合初始化，C2440），按项目既有
// 惯例为含 NSDMI 的值类型补充用户声明构造函数，调用点零改动。
struct EmissionKey {
  std::uint32_t sensor_entity_id{0U};
  std::uint64_t emission_id{0U};

  EmissionKey() = default;
  EmissionKey(std::uint32_t sensor_entity_id_, std::uint64_t emission_id_)
      : sensor_entity_id(sensor_entity_id_), emission_id(emission_id_) {}

  bool operator==(const EmissionKey& other) const {
    return sensor_entity_id == other.sensor_entity_id &&
           emission_id == other.emission_id;
  }
};

struct EmissionKeyHash {
  std::size_t operator()(const EmissionKey& key) const noexcept {
    // 简单折叠加扰动；identity 唯一性由帧校验保证，哈希碰撞不影响正确性。
    std::uint64_t h = static_cast<std::uint64_t>(key.sensor_entity_id) ^ (key.emission_id << 2U);
    return static_cast<std::size_t>(h ^ (h >> 32U));
  }
};

EmissionKey KeyFromIdentity(const oneq::electromagnetics::RfEmissionIdentity& identity) {
  return EmissionKey{identity.sensor_entity_id, identity.emission_id};
}

// 与 InterceptDetectionExecutor 相同的 identity 排序序（sensor_entity_id → emission）。
// 返回按此序排列的 emissions 指针数组，使 entity_index 与 executor 的排序下标一致。
std::vector<const oneq::electromagnetics::RfSceneEmission*> SortEmissionsByIdentity(
    const std::vector<oneq::electromagnetics::RfSceneEmission>& emissions) {
  std::vector<const oneq::electromagnetics::RfSceneEmission*> sorted;
  sorted.reserve(emissions.size());
  for (const oneq::electromagnetics::RfSceneEmission& emission : emissions) {
    sorted.push_back(&emission);
  }
  std::sort(sorted.begin(), sorted.end(),
            [](const oneq::electromagnetics::RfSceneEmission* a,
               const oneq::electromagnetics::RfSceneEmission* b) {
              return a->identity.sensor_entity_id != b->identity.sensor_entity_id
                         ? a->identity.sensor_entity_id < b->identity.sensor_entity_id
                         : a->identity.emission_id < b->identity.emission_id;
            });
  return sorted;
}

// 上一执行周期的排除原因快照（差分原料）。无条目 = 上周未被排除。
struct ExclusionState {
  std::string code{};
  EsrIssueCause cause{EsrIssueCause::kNone};
  ExclusionState() = default;
  ExclusionState(std::string code_, EsrIssueCause cause_)
      : code(std::move(code_)), cause(cause_) {}
};

// 本周期按 entity_index 收集的排除诊断命中（单源单周期取第一条，见 header @note）。
struct CurrentExclusion {
  std::string code{};
  EsrIssueCause cause{EsrIssueCause::kNone};
  bool found{false};
  CurrentExclusion() = default;
  CurrentExclusion(std::string code_, EsrIssueCause cause_, bool found_)
      : code(std::move(code_)), cause(cause_), found(found_) {}
};

// 按 location.entity_index 索引本周期排除诊断。仅消费 kSceneEntity 定位且 phase=kExecution
// 的排除 issue（规则 13b 门内归因条目）；输入校验 issues（phase=kInputValidation）不在此列。
std::unordered_map<std::size_t, CurrentExclusion> IndexCurrentExclusions(
    const EsrCycleResult& result) {
  std::unordered_map<std::size_t, CurrentExclusion> by_entity;
  for (const EsrIssue& issue : result.issues) {
    if (issue.phase != EsrIssuePhase::kExecution ||
        issue.location.kind != oneq::foundation::ValidationLocationKind::kSceneEntity) {
      continue;
    }
    const std::size_t idx = issue.location.entity_index;
    if (by_entity.count(idx) == 0U) {
      by_entity[idx] = CurrentExclusion{issue.code, issue.cause, true};
    }
  }
  return by_entity;
}

}  // namespace

struct EsrExclusionCauseRecorder::Impl {
  std::unordered_map<EmissionKey, ExclusionState, EmissionKeyHash> states;
  std::vector<EsrExclusionCauseEvent> last_events{};
};

EsrExclusionCauseRecorder::EsrExclusionCauseRecorder() : impl_(new Impl) {}

EsrExclusionCauseRecorder::~EsrExclusionCauseRecorder() = default;

EsrExclusionCauseRecorder::EsrExclusionCauseRecorder(EsrExclusionCauseRecorder&& other) noexcept : impl_(std::move(other.impl_)) {}
EsrExclusionCauseRecorder& EsrExclusionCauseRecorder::operator=(EsrExclusionCauseRecorder&& other) noexcept { impl_ = std::move(other.impl_); return *this; }

std::vector<EsrExclusionCauseEvent> EsrExclusionCauseRecorder::Update(
    const EsrCycleInput& input, const EsrCycleResult& result) {
  std::vector<EsrExclusionCauseEvent> events;
  if (result.status != EsrCycleExecutionStatus::kCompleted) {
    return events;
  }
  // 按 identity 排序 emissions，使 entity_index 与 InterceptDetectionExecutor 的排序下标一致。
  const std::vector<const oneq::electromagnetics::RfSceneEmission*> sorted_emissions =
      SortEmissionsByIdentity(input.rf_emissions.emissions);
  const std::unordered_map<std::size_t, CurrentExclusion> current_by_entity =
      IndexCurrentExclusions(result);

  // 本周期被排除发射源的 identity 集合（用于 A4 退出检测）。
  std::unordered_map<EmissionKey, CurrentExclusion, EmissionKeyHash> current_by_key;
  for (const auto& entity_pair : current_by_entity) {
    const std::size_t idx = entity_pair.first;
    if (idx < sorted_emissions.size()) {
      current_by_key[KeyFromIdentity(sorted_emissions[idx]->identity)] = entity_pair.second;
    }
  }

  // 遍历排序后 emissions：对每个发射源判定 A2/A3（被排除中）。
  events.reserve(sorted_emissions.size());
  for (const oneq::electromagnetics::RfSceneEmission* emission : sorted_emissions) {
    const EmissionKey key = KeyFromIdentity(emission->identity);

    std::unordered_map<EmissionKey, CurrentExclusion, EmissionKeyHash>::const_iterator
        current_it = current_by_key.find(key);
    const bool excluded_now = current_it != current_by_key.end() && current_it->second.found;
    std::unordered_map<EmissionKey, ExclusionState, EmissionKeyHash>::iterator prev_it =
        impl_->states.find(key);
    const bool excluded_prev = prev_it != impl_->states.end();

    if (excluded_now && !excluded_prev) {
      // A2：未被排除 → 被排除。
      EsrExclusionCauseEvent event;
      event.cycle_index = result.input_cycle_index;
      event.identity = emission->identity;
      event.kind = EsrExclusionCauseEventKind::kEntered;
      event.current_code = current_it->second.code;
      event.current_cause = current_it->second.cause;
      events.push_back(event);
      impl_->states[key] = ExclusionState{current_it->second.code, current_it->second.cause};
    } else if (excluded_now && excluded_prev) {
      // A3：被排除中 (code,cause) 对变化。A1（相同）不产事件。
      const bool code_changed = prev_it->second.code != current_it->second.code;
      const bool cause_changed = prev_it->second.cause != current_it->second.cause;
      if (code_changed || cause_changed) {
        EsrExclusionCauseEvent event;
        event.cycle_index = result.input_cycle_index;
        event.identity = emission->identity;
        event.kind = EsrExclusionCauseEventKind::kChanged;
        event.previous_code = prev_it->second.code;
        event.previous_cause = prev_it->second.cause;
        event.current_code = current_it->second.code;
        event.current_cause = current_it->second.cause;
        events.push_back(event);
      }
      prev_it->second.code = current_it->second.code;
      prev_it->second.cause = current_it->second.cause;
    }
    // !excluded_now 的处理（A4 退出）在下面的清理遍历中统一做，因为发射源可能从输入消失。
  }

  // A4 退出检测：上一周期被排除但本周期未出现在 current_by_key 中的发射源。
  // 遍历内部状态，找出本周不再被排除（含从输入消失）的源。
  for (std::unordered_map<EmissionKey, ExclusionState, EmissionKeyHash>::iterator
           state_it = impl_->states.begin();
       state_it != impl_->states.end();) {
    std::unordered_map<EmissionKey, CurrentExclusion, EmissionKeyHash>::const_iterator
        current_it = current_by_key.find(state_it->first);
    const bool excluded_now = current_it != current_by_key.end() && current_it->second.found;
    if (!excluded_now) {
      EsrExclusionCauseEvent event;
      event.cycle_index = result.input_cycle_index;
      event.identity.sensor_entity_id = state_it->first.sensor_entity_id;
      event.identity.emission_id = state_it->first.emission_id;
      event.kind = EsrExclusionCauseEventKind::kExited;
      event.previous_code = state_it->second.code;
      event.previous_cause = state_it->second.cause;
      events.push_back(event);
      state_it = impl_->states.erase(state_it);
    } else {
      ++state_it;
    }
  }

  impl_->last_events = events;
  return events;
}

void EsrExclusionCauseRecorder::Reset() {
  impl_->states.clear();
  impl_->last_events.clear();
}

const std::vector<EsrExclusionCauseEvent>& EsrExclusionCauseRecorder::GetLastEvents()
    const noexcept {
  return impl_->last_events;
}

}  // namespace session
}  // namespace electronic_surveillance_radar
