#include "remote_identification_radar/dwell/RirEffectiveRcs.h"

#include <cmath>

#include "common/rcs/RcsPhysics.h"

/**
 * ## 过程：有效目标 RCS（LRDR 探测链 · 第 4 步）
 * - 上游：场景目标标称 RCS（target.rcs，m²）、目标视线角（look_az/look_el 及是否有效）、
 *   hardware.rcs_physics（物理模型开关/混合比例/圆柱权重/等效半径与 RCS 钳位/双站角偏移）、
 *   周期有效载频（规划频率优先，回退发射机频率）。
 * - 本步：把场景标称 RCS 与视角相关物理估计按 physics_mix_ratio 混合成回波方程实际使用的
 *   RCS：σ_eff = mix·σ_phys(视角, f) + (1−mix)·σ_scene，再钳到 [min_rcs_m2, max_rcs_m2]；
 *   载频 ≤ 0（或无有效视线角）时公共核直接返回输入 RCS，不虚构物理值。
 * - 下游：检测单元目标量 cell_target.rcs_m2（回波功率 → SINR → 判决）、量测 RCS
 *   （built.rcs，供第 43 项与识别 RCS 特征）、验收 detection_cell 的 RCS 口径字段。
 * - 关键参数：hardware.rcs_physics.{enable_physical_rcs(默认 false),physics_mix_ratio(1.0),
 *   cylinder_weight(0.5),min/max_equivalent_radius_m(0.05/5.0),min/max_rcs_m2(0.01/1000),
 *   bistatic_psi_offset_deg(5.0)}。
 * - 对应验收项：第 43 项根据目标 RCS 实时计算目标探测结果、第 45 项 RCS 统计特征处理
 *   （识别侧消费同源 RCS）、第 71 项信号处理与检测判决交互接口。
 */

namespace remote_identification_radar {
namespace dwell {

float ComputeEffectiveTargetRcsM2(const session::RirSceneTarget& target,
                                  const RirTargetLookAngles& look_angles,
                                  const config::hardware::RirRcsPhysicsConfig& rcs_config,
                                  float carrier_hz) {
  // carrier_hz <= 0 时 common 直接返回 input RCS（RIR 模块侧策略，无 TX 回退）。
  // 过程：hardware.rcs_physics 域 → 公共核参数结构（配置原值透传，不做单位换算与默认值改写）。
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
  // 过程：混合物理 RCS——按视角/载频求 σ_phys，与场景标称 σ 按 physics_mix_ratio 混合后钳位，返回 m²。
  return oneq::common::rcs::ComputeMixedPhysicalRcsM2(
      target.rcs, carrier_hz, look_angles.look_az_deg, look_angles.look_el_deg,
      look_angles.has_look_angles, params);
}

}  // namespace dwell
}  // namespace remote_identification_radar
