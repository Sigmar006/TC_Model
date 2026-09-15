#include "remote_identification_radar/dwell/RirEffectiveRcs.h"

#include "common/rcs/RcsPhysics.h"

namespace remote_identification_radar {
namespace dwell {

float ComputeEffectiveTargetRcsM2(const session::RirSceneTarget& target,
                                  const RirTargetLookAngles& look_angles,
                                  const config::hardware::RirRcsPhysicsConfig& rcs_config,
                                  float carrier_hz) {
  // carrier_hz <= 0 时 common 直接返回 input RCS（RIR 模块侧策略，无 TX 回退）。
  oneq::common::rcs::RcsPhysicsParams params;
  params.enable_physical_rcs = rcs_config.enable_physical_rcs;
  params.physics_mix_ratio = rcs_config.physics_mix_ratio;
  params.cylinder_weight = rcs_config.cylinder_weight;
  params.min_equivalent_radius_m = rcs_config.min_equivalent_radius_m;
  params.max_equivalent_radius_m = rcs_config.max_equivalent_radius_m;
  params.min_rcs_m2 = rcs_config.min_rcs_m2;
  params.max_rcs_m2 = rcs_config.max_rcs_m2;
  params.bistatic_psi_offset_deg = rcs_config.bistatic_psi_offset_deg;

  return oneq::common::rcs::ComputeMixedPhysicalRcsM2(
      target.rcs, carrier_hz, look_angles.look_az_deg, look_angles.look_el_deg,
      look_angles.has_look_angles, params);
}

}  // namespace dwell
}  // namespace remote_identification_radar
