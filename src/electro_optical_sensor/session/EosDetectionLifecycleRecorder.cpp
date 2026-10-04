#include "1q/electro_optical_sensor/session/EosDetectionLifecycleRecorder.h"

#include <unordered_map>
#include <utility>

#include "1q/electro_optical_sensor/session/EosCycleInput.h"

namespace electro_optical_sensor {
namespace session {

namespace {

struct TargetState {
  bool detected{false};
  std::string target_name{};
};

const output::EosDetectionRecord* FindRecord(std::uint64_t detection_id, const EosOutputFrame& frame) {
  for (const output::EosDetectionRecord& record : frame.detections) {
    if (record.detection_id == detection_id) {
      return &record;
    }
  }
  return nullptr;
}

const attribution::EosDetectionAttributionRecord* FindAttribution(
    std::uint64_t target_id, const attribution::EosDetectionAttributionRecordList& attributions) {
  for (const attribution::EosDetectionAttributionRecord& attribution : attributions) {
    if (attribution.target_id == target_id) {
      return &attribution;
    }
  }
  return nullptr;
}

EosDetectionLifecycleReason InferReason(const output::EosDetectionRecord* record) {
  if (record == nullptr) {
    return EosDetectionLifecycleReason::kOutOfFov;
  }
  if (!record->detected) {
    return EosDetectionLifecycleReason::kBelowSnrThreshold;
  }
  return EosDetectionLifecycleReason::kUnknown;
}

EosDetectionLifecycleEvent MakeBaseEvent(const EosSceneTarget& target, const EosCycleResult& result) {
  EosDetectionLifecycleEvent event;
  event.cycle_index = result.input_cycle_index;
  event.target_id = target.target_id;
  event.target_name = target.target_name;
  return event;
}

}  // namespace

struct EosDetectionLifecycleRecorder::Impl {
  EosDetectionLifecycleRecorderConfig config;
  std::unordered_map<std::uint64_t, TargetState> states;
  std::vector<EosDetectionLifecycleEvent> last_events{};
  Impl() = default;
  Impl(EosDetectionLifecycleRecorderConfig config_,
       std::unordered_map<std::uint64_t, TargetState> states_ = {})
      : config(config_), states(std::move(states_)) {}
};

EosDetectionLifecycleRecorder::EosDetectionLifecycleRecorder(EosDetectionLifecycleRecorderConfig config)
    : impl_(new Impl{config, {}}) {}

EosDetectionLifecycleRecorder::~EosDetectionLifecycleRecorder() = default;

EosDetectionLifecycleRecorder::EosDetectionLifecycleRecorder(EosDetectionLifecycleRecorder&& other) noexcept : impl_(std::move(other.impl_)) {}
EosDetectionLifecycleRecorder& EosDetectionLifecycleRecorder::operator=(EosDetectionLifecycleRecorder&& other) noexcept { impl_ = std::move(other.impl_); return *this; }

std::vector<EosDetectionLifecycleEvent> EosDetectionLifecycleRecorder::Update(
    const EosCycleInput& input, const EosCycleResult& result) {
  std::vector<EosDetectionLifecycleEvent> events;
  if (result.status != EosCycleStatus::kCompleted) {
    return events;
  }
  events.reserve(input.scene.size());
  for (const EosSceneTarget& target : input.scene) {
    TargetState& state = impl_->states[target.target_id];
    const attribution::EosDetectionAttributionRecord* attribution =
        FindAttribution(target.target_id, result.detection_attributions);
    const output::EosDetectionRecord* record =
        attribution == nullptr ? nullptr
                               : FindRecord(attribution->detection_id, result.output_frame);
    const bool detected_now = record != nullptr && record->detected;
    if (detected_now) {
      EosDetectionLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = state.detected ? EosDetectionLifecycleEventKind::kUpdated
                                  : EosDetectionLifecycleEventKind::kFirstDetected;
      event.reason = EosDetectionLifecycleReason::kNone;
      event.fused_snr_db = record->fused_snr_db;
      event.range_m = record->range_m;
      events.push_back(event);
      state.detected = true;
      state.target_name = target.target_name;
      continue;
    }

    const EosDetectionLifecycleReason reason = InferReason(record);
    if (state.detected) {
      EosDetectionLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = EosDetectionLifecycleEventKind::kLost;
      event.reason = reason;
      if (record != nullptr) {
        event.fused_snr_db = record->fused_snr_db;
        event.range_m = record->range_m;
      }
      events.push_back(event);
    } else if (impl_->config.emit_not_detected_events) {
      EosDetectionLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = EosDetectionLifecycleEventKind::kNotDetected;
      event.reason = reason;
      if (record != nullptr) {
        event.fused_snr_db = record->fused_snr_db;
        event.range_m = record->range_m;
      }
      events.push_back(event);
    }
    state.detected = false;
    state.target_name = target.target_name;
  }
  impl_->last_events = events;
  return events;
}

void EosDetectionLifecycleRecorder::Reset() {
  impl_->states.clear();
  impl_->last_events.clear();
}

const std::vector<EosDetectionLifecycleEvent>& EosDetectionLifecycleRecorder::GetLastEvents() const
    noexcept {
  return impl_->last_events;
}

}  // namespace session
}  // namespace electro_optical_sensor
