/*
 * Supplemental architecture notes for acceptance review.
 * Unit: RirEffectiveRcs.
 * Model: LRDR remote identification radar.
 * Layer: dwell.
 * File role: interface declarations and data contracts.
 * Responsibility: per-dwell emission, receiver, beam, echo, and detection processing.
 * Upstream boundary: dwell plan, RF scene, target geometry, and radar hardware state.
 * Downstream boundary: resolved RF cells, detection decisions, and track measurements.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

/**
 * @file RirEffectiveRcs.h
 * @brief 有效目标 RCS 求解（物理估计与经验值混合）。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_EFFECTIVE_RCS_H_
#define REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_EFFECTIVE_RCS_H_

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "1q/remote_identification_radar/session/RirSceneTypes.h"

namespace remote_identification_radar {
namespace dwell {

/** @brief 目标视线角（雷达局部 ENU，deg）。 */
struct RirTargetLookAngles {
  float look_az_deg{0.0f};
  float look_el_deg{0.0f};
  bool has_look_angles{false};
};

/**
 * @brief 计算有效目标 RCS（m²）。
 *
 * 逻辑对齐 AR `DetectionExecution::ComputeEffectiveTargetRcsM2`；
 * `enable_physical_rcs=false` 或 `physics_mix_ratio=0` 时返回输入 RCS。
 */
float ComputeEffectiveTargetRcsM2(const session::RirSceneTarget& target,
                                  const RirTargetLookAngles& look_angles,
                                  const config::hardware::RirRcsPhysicsConfig& rcs_config,
                                  float carrier_hz);

}  // namespace dwell
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_EFFECTIVE_RCS_H_
