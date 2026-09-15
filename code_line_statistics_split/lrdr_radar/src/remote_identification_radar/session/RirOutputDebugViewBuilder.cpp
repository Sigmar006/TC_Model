#include "1q/remote_identification_radar/session/RirOutputDebugView.h"

#include <cmath>

#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "common/numerics/Constants.h"

namespace remote_identification_radar {
namespace session {

namespace {


double RirRadToDeg(double radians) {
  return oneq::common::numerics::RadToDeg(radians);
}

RirDebugTargetStatus ToDebugTargetStatus(RirTrackLifecycleStatus status) {
  switch (status) {
    case RirTrackLifecycleStatus::kConfirmed:
      return RirDebugTargetStatus::kConfirmed;
    case RirTrackLifecycleStatus::kLost:
      return RirDebugTargetStatus::kLost;
    case RirTrackLifecycleStatus::kTentative:
    default:
      return RirDebugTargetStatus::kTentative;
  }
}

const RirTrackAttributionRecord* FindAttributionByExternalTargetId(
    const RirCycleResult& result, std::uint64_t external_target_id) {
  for (const RirTrackAttributionRecord& attribution : result.track_attributions) {
    if (attribution.external_target_id == external_target_id && external_target_id != 0U) {
      return &attribution;
    }
  }
  return nullptr;
}

const RirTrackRecognitionOutput* FindRecognitionOutput(
    const RirCycleResult& result, std::uint64_t association_key) {
  for (const RirTrackRecognitionOutput& output : result.output_frame.recognition_outputs) {
    if (output.association_key == association_key) {
      return &output;
    }
  }
  return nullptr;
}

// 输入几何回填（规则 12）：斜距/视线角自输入 ENU 位置推导，与库内
// RirController::ComputeLookAngles 同口径。
void FillInputGeometry(const RirSceneTarget& target, RirDebugTargetState* state) {
  const double position_x = static_cast<double>(target.position_x);
  const double position_y = static_cast<double>(target.position_y);
  const double position_z = static_cast<double>(target.position_z);
  const double horizontal = std::sqrt(position_x * position_x + position_y * position_y);
  state->look_az_deg = RirRadToDeg(std::atan2(position_y, position_x));
  state->look_el_deg = RirRadToDeg(std::atan2(position_z, horizontal));
  state->slant_range_m =
      std::sqrt(position_x * position_x + position_y * position_y + position_z * position_z);
}

RirDebugTargetState BuildDebugTargetState(const RirSceneTarget& target,
                                          const RirCycleResult& cycle_result) {
  RirDebugTargetState state;
  state.external_target_id = target.external_target_id;
  state.target_name = target.target_name;
  state.present_in_input = true;
  // 输入几何回填（规则 12）：无航迹/未完成周期仍暴露输入侧斜距与视线角。
  FillInputGeometry(target, &state);
  if (cycle_result.status != RirCycleStatus::kCompleted) {
    state.status = RirDebugTargetStatus::kCycleNotCompleted;
    return state;
  }
  const RirTrackAttributionRecord* attribution =
      FindAttributionByExternalTargetId(cycle_result, target.external_target_id);
  if (attribution == nullptr) {
    // external_target_id 为 0（未知）的输入目标无法按 ID 关联到航迹。
    state.status = RirDebugTargetStatus::kNotInOutput;
    return state;
  }
  state.has_track = true;
  state.association_key = attribution->association_key;
  state.status = ToDebugTargetStatus(attribution->track_status);
  state.hit_count = attribution->hit_count;
  state.position_enu_x_m = attribution->position_enu_x_m;
  state.position_enu_y_m = attribution->position_enu_y_m;
  state.position_enu_z_m = attribution->position_enu_z_m;
  state.speed_m_per_s = attribution->speed_m_per_s;
  // 识别诊断（识别结论不构成生命周期状态位，仅作诊断字段透出）。
  const RirTrackRecognitionOutput* recognition =
      FindRecognitionOutput(cycle_result, attribution->association_key);
  if (recognition != nullptr &&
      recognition->result.state != RirRecognitionState::kDisabled) {
    state.has_recognition_output = true;
    state.recognition_state = recognition->result.state;
    state.target_category = recognition->result.target_category;
    state.target_model = recognition->result.target_model;
    state.confidence = recognition->result.confidence;
    state.observation_count = recognition->result.observation_count;
  }
  if (attribution->has_cycle_detection_snr) {
    state.has_cycle_detection_snr = true;
    state.cycle_detection_snr_db = static_cast<double>(attribution->cycle_detection_snr_db);
  }
  return state;
}

}  // namespace

RirOutputDebugView RirOutputDebugViewBuilder::Build(const RirCycleInput& input,
                                                    const RirCycleResult& result) {
  RirOutputDebugView view;
  view.input_cycle_index = result.input_cycle_index;
  view.executed_this_cycle = result.status == RirCycleStatus::kCompleted;
  view.abort_reason = result.abort_reason;
  view.issues = result.issues;
  // 指定任务镜像直接转写信封字段（只读组合，不反向影响链路）。
  view.designated_target_id = result.designated_target_id;
  view.designation_active = result.designation_active;
  view.designation_reverted_to_scan = result.designation_reverted_to_scan;
  view.designation_revert_reason = result.designation_revert_reason;
  view.dwell_center_deg = result.dwell_center_deg;
  view.targets.reserve(input.scene_targets.size());
  for (const RirSceneTarget& target : input.scene_targets) {
    view.targets.push_back(BuildDebugTargetState(target, result));
  }
  return view;
}

}  // namespace session
}  // namespace remote_identification_radar
