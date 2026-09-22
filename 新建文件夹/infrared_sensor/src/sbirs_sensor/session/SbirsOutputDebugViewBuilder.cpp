/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsOutputDebugViewBuilder.
 * Model: SBIRS infrared sensor.
 * Layer: session.
 * File role: executable implementation.
 * Responsibility: public cycle contract, validation, adapters, recording, and replay.
 * Upstream boundary: external cycle data and validated session configuration.
 * Downstream boundary: stable output frames, issues, debug views, and replay records.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsOutputDebugView.h"

#include <algorithm>
#include <cmath>

#include "1q/coordinate/inertial_transform.h"
#include "common/numerics/Constants.h"
#include "sbirs_sensor/foundation/SbirsGeometry.h"

namespace sbirs_sensor {
namespace session {
namespace {

float WrapAzimuthPositive(float azimuth_deg) {
  const float wrapped = std::fmod(azimuth_deg, 360.0f);
  return wrapped < 0.0f ? wrapped + 360.0f : wrapped;
}

float ToEciAzimuthRad(float azimuth_deg) {
  return oneq::common::numerics::DegToRad(WrapAzimuthPositive(azimuth_deg));
}

float ToEciElevationRad(float elevation_deg) {
  const float clamped = std::max(-90.0f, std::min(90.0f, elevation_deg));
  return oneq::common::numerics::DegToRad(clamped);
}

// 由输入实体回填 ECI 极坐标角度（rad）：与检测记录同参考系（2026-08 正式变更）。
// 输入仍为 ECEF（位置/速度），需按本周期 GMST 旋转到 ECI 后计算 az/el。
bool TryComputeEciAnglesRad(const session::SbirsCycleInput& input,
                            const session::SbirsSceneTarget& target, float* azimuth_rad,
                            float* elevation_rad) {
  if (azimuth_rad == nullptr || elevation_rad == nullptr) {
    return false;
  }
  double gmst_rad = 0.0;
  if (!oneq::coordinate::TryComputeGmstRad(input.utc_julian_day, &gmst_rad)) {
    return false;
  }
  const oneq::coordinate::EcefPositionM sat_ecef(input.satellite_position_ecef_m.x,
                                                 input.satellite_position_ecef_m.y,
                                                 input.satellite_position_ecef_m.z);
  oneq::coordinate::EciPositionM sat_eci;
  if (!oneq::coordinate::TryEcefToEci(sat_ecef, gmst_rad, &sat_eci)) {
    return false;
  }
  const oneq::coordinate::EcefPositionM tgt_ecef(target.position_ecef_m.x,
                                                 target.position_ecef_m.y,
                                                 target.position_ecef_m.z);
  oneq::coordinate::EciPositionM tgt_eci;
  if (!oneq::coordinate::TryEcefToEci(tgt_ecef, gmst_rad, &tgt_eci)) {
    return false;
  }
  session::SbirsVector3M los;
  los.x = tgt_eci.x_m - sat_eci.x_m;
  los.y = tgt_eci.y_m - sat_eci.y_m;
  los.z = tgt_eci.z_m - sat_eci.z_m;
  *azimuth_rad = ToEciAzimuthRad(foundation::ComputeAzimuthDeg(los));
  *elevation_rad = ToEciElevationRad(foundation::ComputeElevationDeg(los));
  return true;
}

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

SbirsDebugTargetState BuildTargetState(const SbirsSceneTarget& target,
                                       const SbirsCycleInput& input,
                                       const SbirsCycleResult& result) {
  SbirsDebugTargetState state;
  state.target_id = target.target_id;
  state.target_name = target.target_name;
  state.present_in_input = true;
  // 输入实体回填（规则 12）：az/el 用卫星→目标视线向量按本周期 GMST 旋转到
  // ECI 后计算（ECI 极坐标，rad，与检测记录同参考系），无论是否检测均可见
  // （检测记录存在时下方以记录观测值覆盖）。JD 缺失时回填失败，保持默认 0。
  (void)TryComputeEciAnglesRad(input, target, &state.azimuth_rad, &state.elevation_rad);
  if (result.status != SbirsCycleStatus::kCompleted) {
    state.status = SbirsDebugTargetStatus::kCycleNotExecuted;
    return state;
  }

  const attribution::SbirsDetectionAttributionRecord* attribution =
      FindAttribution(target.target_id, result.detection_attributions);
  if (attribution == nullptr) {
    state.status = SbirsDebugTargetStatus::kNotInOutput;
    return state;
  }

  state.tracking_source = attribution->tracking_source;
  state.estimated_range_m = attribution->estimated_range_m;
  state.max_detection_range_m = attribution->max_detection_range_m;
  state.has_estimation_nis = attribution->has_estimation_nis;
  state.estimation_nis = attribution->estimation_nis;
  state.estimation_nis_gate_exceeded = attribution->estimation_nis_gate_exceeded;
  state.nfov_channel_id = attribution->nfov_channel_id;
  state.has_nfov_tracking_diagnostics = attribution->has_nfov_tracking_diagnostics;
  state.nfov_pointing_error_deg = attribution->nfov_pointing_error_deg;
  state.nfov_geometry_gate_passed = attribution->nfov_geometry_gate_passed;
  state.nfov_snr_gate_passed = attribution->nfov_snr_gate_passed;
  state.nfov_tracking_gate_failure_count = attribution->nfov_tracking_gate_failure_count;
  state.nfov_tracking_coasting = attribution->nfov_tracking_coasting;
  state.observation_stage = InferObservationStage(*attribution);

  if (attribution->nfov_tracking_coasting) {
    state.status = SbirsDebugTargetStatus::kCoasting;
    return state;
  }

  const output::SbirsDetectionRecord* record =
      FindRecord(attribution->detection_id, result.output_frame);
  if (record == nullptr) {
    state.status = SbirsDebugTargetStatus::kNotInOutput;
    return state;
  }

  state.has_raw_output_record = true;
  state.detected = record->detected;
  state.measurement_time_s = record->measurement_time_s;
  state.azimuth_rad = record->azimuth_rad;
  state.elevation_rad = record->elevation_rad;
  state.infrared_snr_linear = record->infrared_snr_linear;
  state.observation_stage = record->observation_stage;
  state.status = record->detected ? SbirsDebugTargetStatus::kDetected
                                  : SbirsDebugTargetStatus::kObservedBelowThreshold;
  return state;
}

}  // namespace

SbirsOutputDebugView SbirsOutputDebugViewBuilder::Build(const SbirsCycleInput& input,
                                                        const SbirsCycleResult& result) {
  SbirsOutputDebugView view;
  view.input_cycle_index = result.input_cycle_index;
  view.output_cycle_index = result.output_frame.cycle_index;
  view.executed_this_cycle = (result.status == SbirsCycleStatus::kCompleted);
  view.abort_reason = result.abort_reason;
  view.issues = result.issues;
  view.targets.reserve(input.scene.size());
  for (const SbirsSceneTarget& target : input.scene) {
    view.targets.push_back(BuildTargetState(target, input, result));
  }
  return view;
}

}  // namespace session
}  // namespace sbirs_sensor
