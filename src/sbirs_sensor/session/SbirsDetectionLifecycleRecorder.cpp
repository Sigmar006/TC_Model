#include "1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"

#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "1q/sbirs_sensor/session/SbirsCycleInput.h"

namespace sbirs_sensor {
namespace session {
namespace {

struct TargetState {
  bool detected{false};
  std::string target_name{};
  // 中译：**预警中断计数**（2026-11-04）——连续未检测拍数 + 是否已报过中断 + 是否曾被
  //   成功预警（`kFirstDetected`/`kUpdated`/`kCoasting` 任一置真）。连续 10 拍未检测即报
  //   "预警中断"（第24B项），已报过不重复报；目标重新被检测到则清零、允许下次再报。
  // 标识：预警中断计数（2026-11-04）。
  std::uint32_t missed_cycles{0U};
  bool interrupted_reported{false};
  bool warned{false};
};

// 预警中断阈值（甲方 2026-11-04：连续 10 拍未检测即判中断）。
constexpr std::uint32_t kEarlyWarningInterruptCycles = 10U;

const output::SbirsDetectionRecord* FindRecord(std::uint64_t detection_id,
                                               const SbirsOutputFrame& frame) {
  for (const output::SbirsDetectionRecord& record : frame.detections) {
    if (record.detection_id == detection_id) {
      return &record;
    }
  }
  return nullptr;
}

const attribution::SbirsDetectionAttributionRecord* FindAttribution(
    std::uint64_t target_id, const attribution::SbirsDetectionAttributionRecordList& attributions) {
  for (const attribution::SbirsDetectionAttributionRecord& attribution : attributions) {
    if (attribution.target_id == target_id) {
      return &attribution;
    }
  }
  return nullptr;
}

output::SbirsObservationStage InferObservationStage(
    const attribution::SbirsDetectionAttributionRecord& attribution) {
  if (attribution.nfov_tracking_coasting || attribution.has_nfov_tracking_diagnostics) {
    return output::SbirsObservationStage::kNarrowFieldTrack;
  }
  switch (attribution.capture_failure_reason) {
    case attribution::SbirsCaptureFailureReason::kEstimationNisGateLost:
    case attribution::SbirsCaptureFailureReason::kNfovTrackingGateLost:
      return output::SbirsObservationStage::kNarrowFieldTrack;
    case attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed:
    case attribution::SbirsCaptureFailureReason::kNfovPointingTimeout:
      return output::SbirsObservationStage::kNarrowFieldAcquisition;
    case attribution::SbirsCaptureFailureReason::kSchedulerSkipped:
    case attribution::SbirsCaptureFailureReason::kNone:
      return output::SbirsObservationStage::kWideFieldSearch;
  }
  return output::SbirsObservationStage::kWideFieldSearch;
}

SbirsDetectionLifecycleReason InferReason(
    const output::SbirsDetectionRecord* record,
    const attribution::SbirsDetectionAttributionRecord* attribution) {
  // 优先消费 attribution 携带的捕获失败原因，确保捕获失败场景不被误归为 FOV 外。
  if (attribution != nullptr) {
    switch (attribution->capture_failure_reason) {
      case attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed:
        return SbirsDetectionLifecycleReason::kNfovAcquisitionFailed;
      case attribution::SbirsCaptureFailureReason::kSchedulerSkipped:
        return SbirsDetectionLifecycleReason::kSchedulerSkipped;
      case attribution::SbirsCaptureFailureReason::kEstimationNisGateLost:
        return SbirsDetectionLifecycleReason::kEstimationNisGateLost;
      case attribution::SbirsCaptureFailureReason::kNfovPointingTimeout:
        return SbirsDetectionLifecycleReason::kNfovPointingTimeout;
      case attribution::SbirsCaptureFailureReason::kNfovTrackingGateLost:
        return SbirsDetectionLifecycleReason::kNfovTrackingGateLost;
      case attribution::SbirsCaptureFailureReason::kNone:
        break;
    }
  }
  if (record == nullptr) {
    return SbirsDetectionLifecycleReason::kOutOfFieldOfView;
  }
  if (!record->detected) {
    return SbirsDetectionLifecycleReason::kBelowSnrThreshold;
  }
  return SbirsDetectionLifecycleReason::kUnknown;
}

SbirsDetectionLifecycleEvent MakeBaseEvent(const SbirsSceneTarget& target,
                                           const SbirsCycleResult& result) {
  SbirsDetectionLifecycleEvent event;
  event.cycle_index = result.input_cycle_index;
  event.target_id = target.target_id;
  event.target_name = target.target_name;
  return event;
}

void FillObservationFields(const output::SbirsDetectionRecord& record,
                           const attribution::SbirsDetectionAttributionRecord& attribution,
                           SbirsDetectionLifecycleEvent* event) {
  event->observation_stage = record.observation_stage;
  event->infrared_snr_linear = record.infrared_snr_linear;
  event->estimated_range_m = attribution.estimated_range_m;
  event->max_detection_range_m = attribution.max_detection_range_m;
  event->tracking_source = attribution.tracking_source;
  event->has_estimation_nis = attribution.has_estimation_nis;
  event->estimation_nis = attribution.estimation_nis;
  event->estimation_nis_gate_exceeded = attribution.estimation_nis_gate_exceeded;
  event->nfov_channel_id = attribution.nfov_channel_id;
  event->has_nfov_tracking_diagnostics = attribution.has_nfov_tracking_diagnostics;
  event->nfov_pointing_error_deg = attribution.nfov_pointing_error_deg;
  event->nfov_geometry_gate_passed = attribution.nfov_geometry_gate_passed;
  event->nfov_snr_gate_passed = attribution.nfov_snr_gate_passed;
  event->nfov_tracking_gate_failure_count = attribution.nfov_tracking_gate_failure_count;
  event->nfov_tracking_coasting = attribution.nfov_tracking_coasting;
}

void FillAttributionFields(const attribution::SbirsDetectionAttributionRecord& attribution,
                           SbirsDetectionLifecycleEvent* event) {
  event->observation_stage = InferObservationStage(attribution);
  event->estimated_range_m = attribution.estimated_range_m;
  event->max_detection_range_m = attribution.max_detection_range_m;
  event->tracking_source = attribution.tracking_source;
  event->has_estimation_nis = attribution.has_estimation_nis;
  event->estimation_nis = attribution.estimation_nis;
  event->estimation_nis_gate_exceeded = attribution.estimation_nis_gate_exceeded;
  event->nfov_channel_id = attribution.nfov_channel_id;
  event->has_nfov_tracking_diagnostics = attribution.has_nfov_tracking_diagnostics;
  event->nfov_pointing_error_deg = attribution.nfov_pointing_error_deg;
  event->nfov_geometry_gate_passed = attribution.nfov_geometry_gate_passed;
  event->nfov_snr_gate_passed = attribution.nfov_snr_gate_passed;
  event->nfov_tracking_gate_failure_count = attribution.nfov_tracking_gate_failure_count;
  event->nfov_tracking_coasting = attribution.nfov_tracking_coasting;
}

}  // namespace

struct SbirsDetectionLifecycleRecorder::Impl {
  SbirsDetectionLifecycleRecorderConfig config;
  std::unordered_map<std::uint64_t, TargetState> states;
  std::vector<SbirsDetectionLifecycleEvent> last_events{};
  Impl() = default;
  Impl(SbirsDetectionLifecycleRecorderConfig config_,
       std::unordered_map<std::uint64_t, TargetState> states_ = {})
      : config(config_), states(std::move(states_)) {}
};

SbirsDetectionLifecycleRecorder::SbirsDetectionLifecycleRecorder(
    SbirsDetectionLifecycleRecorderConfig config)
    : impl_(new Impl{config, {}}) {}

SbirsDetectionLifecycleRecorder::~SbirsDetectionLifecycleRecorder() = default;
SbirsDetectionLifecycleRecorder::SbirsDetectionLifecycleRecorder(SbirsDetectionLifecycleRecorder&& other) noexcept : impl_(std::move(other.impl_)) {}
SbirsDetectionLifecycleRecorder& SbirsDetectionLifecycleRecorder::operator=(SbirsDetectionLifecycleRecorder&& other) noexcept { impl_ = std::move(other.impl_); return *this; }

std::vector<SbirsDetectionLifecycleEvent> SbirsDetectionLifecycleRecorder::Update(
    const SbirsCycleInput& input, const SbirsCycleResult& result) {
  std::vector<SbirsDetectionLifecycleEvent> events;
  if (result.status != SbirsCycleStatus::kCompleted) {
    return events;
  }
  std::unordered_set<std::uint64_t> present_target_ids;
  events.reserve(input.scene.size());

  for (const SbirsSceneTarget& target : input.scene) {
    present_target_ids.insert(target.target_id);
    TargetState& state = impl_->states[target.target_id];
    const attribution::SbirsDetectionAttributionRecord* attribution =
        FindAttribution(target.target_id, result.detection_attributions);
    const output::SbirsDetectionRecord* record =
        attribution == nullptr ? nullptr
                               : FindRecord(attribution->detection_id, result.output_frame);
    if (attribution != nullptr && attribution->nfov_tracking_coasting) {
      SbirsDetectionLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = SbirsDetectionLifecycleEventKind::kCoasting;
      event.reason = SbirsDetectionLifecycleReason::kNone;
      FillAttributionFields(*attribution, &event);
      events.push_back(event);
      state.detected = true;
      state.target_name = target.target_name;
      state.missed_cycles = 0U;
      state.interrupted_reported = false;
      state.warned = true;
      continue;
    }
    const bool detected_now = record != nullptr && record->detected;
    if (detected_now) {
      SbirsDetectionLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = state.detected ? SbirsDetectionLifecycleEventKind::kUpdated
                                  : SbirsDetectionLifecycleEventKind::kFirstDetected;
      event.reason = SbirsDetectionLifecycleReason::kNone;
      FillObservationFields(*record, *attribution, &event);
      events.push_back(event);
      state.detected = true;
      state.target_name = target.target_name;
      state.missed_cycles = 0U;
      state.interrupted_reported = false;
      state.warned = true;
      continue;
    }

    const SbirsDetectionLifecycleReason reason = InferReason(record, attribution);
    if (state.detected) {
      SbirsDetectionLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = SbirsDetectionLifecycleEventKind::kLost;
      event.reason = reason;
      if (record != nullptr && attribution != nullptr) {
        FillObservationFields(*record, *attribution, &event);
      } else if (attribution != nullptr) {
        FillAttributionFields(*attribution, &event);
      }
      events.push_back(event);
    } else if (impl_->config.emit_not_detected_events) {
      SbirsDetectionLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = SbirsDetectionLifecycleEventKind::kNotDetected;
      event.reason = reason;
      if (record != nullptr && attribution != nullptr) {
        FillObservationFields(*record, *attribution, &event);
      } else if (attribution != nullptr) {
        FillAttributionFields(*attribution, &event);
      }
      events.push_back(event);
    }
    state.detected = false;
    state.target_name = target.target_name;
    // 中译：连续未检测计数 + 第 10 拍报"预警中断"（2026-11-04 甲方口径：10 拍）。
    // 标识：预警中断判定（2026-11-04）。
    ++state.missed_cycles;
    if (state.warned && !state.interrupted_reported &&
        state.missed_cycles == kEarlyWarningInterruptCycles) {
      SbirsDetectionLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = SbirsDetectionLifecycleEventKind::kEarlyWarningInterrupted;
      event.reason = reason;
      if (record != nullptr && attribution != nullptr) {
        FillObservationFields(*record, *attribution, &event);
      } else if (attribution != nullptr) {
        FillAttributionFields(*attribution, &event);
      }
      events.push_back(event);
      state.interrupted_reported = true;
    }
  }

  // 中译：目标从输入场景消失时**不再立刻删除状态**（2026-11-04）——删了就无法累计
  //   "连续 10 拍未检测"，第24B项的预警中断永远报不出来。改为：先按 `kLost` 报一次
  //   （保持既有行为），随后与"在场但未检测"走同一套计数，在第 10 拍报预警中断。
  // 标识：目标消失也计预警中断（2026-11-04）。
  for (std::unordered_map<std::uint64_t, TargetState>::iterator it = impl_->states.begin();
       it != impl_->states.end(); ++it) {
    if (present_target_ids.count(it->first) != 0U) {
      continue;
    }
    if (it->second.detected) {
      SbirsDetectionLifecycleEvent event;
      event.cycle_index = result.input_cycle_index;
      event.target_id = it->first;
      event.target_name = it->second.target_name;
      event.kind = SbirsDetectionLifecycleEventKind::kLost;
      event.reason = SbirsDetectionLifecycleReason::kTargetMissingFromInput;
      events.push_back(event);
      it->second.detected = false;
    }
    ++it->second.missed_cycles;
    if (it->second.warned && !it->second.interrupted_reported &&
        it->second.missed_cycles == kEarlyWarningInterruptCycles) {
      SbirsDetectionLifecycleEvent event;
      event.cycle_index = result.input_cycle_index;
      event.target_id = it->first;
      event.target_name = it->second.target_name;
      event.kind = SbirsDetectionLifecycleEventKind::kEarlyWarningInterrupted;
      event.reason = SbirsDetectionLifecycleReason::kTargetMissingFromInput;
      events.push_back(event);
      it->second.interrupted_reported = true;
    }
  }

  impl_->last_events = events;
  return events;
}

void SbirsDetectionLifecycleRecorder::Reset() {
  impl_->states.clear();
  impl_->last_events.clear();
}

const std::vector<SbirsDetectionLifecycleEvent>& SbirsDetectionLifecycleRecorder::GetLastEvents()
    const noexcept {
  return impl_->last_events;
}

}  // namespace session
}  // namespace sbirs_sensor
