#include "remote_identification_radar/dwell/RirEffectiveRcs.h"

#include <cmath>

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

  // 中译：**视角 RCS 表不进检测链**（2026-10-26 结论）——场景给的 `视角RCS=[…]` 是
  //   **识别特征表**（第 45 项 RCS 均值/标准差用它），不是检测回波用的 RCS；
  //   若把它接进回波方程，会整体改变能量预算并打破量测误差标定用例
  //   (`RirMeasurementErrorCalibrationTest.ErrorModelInputIncludesIntegrationGain`)。
  //   要得到随视角变化的**检测** RCS，应打开 `hardware.rcs_physics.enable_physical_rcs`。
  // 标识：视角 RCS 表用途界定（2026-10-26）。
  return oneq::common::rcs::ComputeMixedPhysicalRcsM2(
      target.rcs, carrier_hz, look_angles.look_az_deg, look_angles.look_el_deg,
      look_angles.has_look_angles, params);
}

}  // namespace dwell
}  // namespace remote_identification_radar
