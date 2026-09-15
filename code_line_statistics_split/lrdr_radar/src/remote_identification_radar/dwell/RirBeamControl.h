/**
 * @file RirBeamControl.h
 * @brief 定义 RIR 波束宽度解析与给定指向下的方向图增益求解（私有实现头）。
 *
 * 波束宽度解析数值内核为 common 单源；方向图增益求解保留在本文件。
 * 与 AR 版的刻意差异：
 *   - 无 commanded 波束宽度覆盖分支（AR orientation 域机制，RIR 不随迁）；
 *   - 无平台姿态/稳定模式安装系指向解算：RIR 驻留指向由驻留调度显式给定，
 *     本文件只消费给定值（调度器给指向、RIR 信指向），见下方指向契约；
 *   - 保留：有效波束宽度两级回退（nominal → λ/L 物理推导）、
 *     离轴角 → 方向图增益链路、无视线角退化输入回退主瓣峰值。
 *     （2026-08-29 架构还债：enable_directional_pattern 开关已删除，RIR 离轴
 *     方向图恒开——探测与波束指向耦合；common 输入结构保留该字段供 AR 使用，
 *     RIR 侧恒传 true。）
 *
 * 驻留指向契约（冻结）：
 *   - 来源：调用方驻留调度器显式给定波束中心；RIR 不生成指向、不按目标位置
 *     重算或吸附指向。
 *   - 角度：`RirAzimuthElevationDeg`（deg），雷达局部 ENU 右手系；
 *     az ∈ [-180, 180]，el ∈ [-90, 90]，方位角须由调用方预先折算到该区间。
 *   - 含义：az=0/el=0 指向 +x（东向水平），az 正向转向 +y（北），
 *     el 正向指向 +z（天）；单位向量
 *     (cos(el)·cos(az), cos(el)·sin(az), sin(el))。
 *   - 离轴：目标视线角以同一坐标系公式计算；方位差经
 *     `NormalizeAzimuthDeltaDeg` 折算到 [-180, 180]，俯仰差直接相减。
 * @note 本文件仅供 RIR 模块内部使用，不作为公开 API。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_BEAM_CONTROL_H_
#define REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_BEAM_CONTROL_H_

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "common/radar/BeamwidthResolution.h"
#include "common/radar/FrozenBeamResolve.h"
#include "common/radar/ScanScheduleRuntime.h"
#include "remote_identification_radar/dwell/RirAntennaPatternRuntime.h"

namespace remote_identification_radar {
namespace dwell {

/**
 * @brief RirEffectiveBeamwidthDeg 表示解析后的有效波束宽度（单位：度）。
 */
using RirEffectiveBeamwidthDeg = ::oneq::common::radar::EffectiveBeamwidthDeg;

/**
 * @brief 从物理孔径尺寸推导半功率波束宽度（单位：rad）。
 */
inline float RirDeriveBeamwidthFromApertureRad(float aperture_length_m, float wavelength_m) {
  return ::oneq::common::radar::DeriveBeamwidthFromApertureRad(aperture_length_m, wavelength_m);
}

/**
 * @brief 解析有效波束宽度（两级回退，无 commanded 覆盖）。
 */
inline RirEffectiveBeamwidthDeg RirResolveEffectiveBeamwidth(
    const config::hardware::RirAntennaConfig& antenna_config, float wavelength_m = 0.0f) {
  return ::oneq::common::radar::ResolveEffectiveBeamwidth(
      antenna_config.nominal_az_beamwidth_deg, antenna_config.nominal_el_beamwidth_deg,
      antenna_config.antenna_length_m, antenna_config.antenna_width_m, wavelength_m);
}

/**
 * @brief RirResolvedBeamState 表示 RIR 当前驻留使用的波束状态。
 */
struct RirResolvedBeamState {
  RirEffectiveBeamwidthDeg effective_beamwidth_deg{};
  config::RirAzimuthElevationDeg beam_pointing_deg{};
  float one_way_antenna_gain_db{0.0f};
};

/**
 * @brief 解析给定驻留指向下的目标方向波束状态（common 冻结变体 + 方位差预归一化）。
 */
inline RirResolvedBeamState RirResolveBeamStateForPointing(
    const config::hardware::RirAntennaConfig& antenna_config,
    const config::RirAzimuthElevationDeg& beam_pointing_deg, float target_look_az_deg,
    float target_look_el_deg, bool has_look_angles, float wavelength_m = 0.0f) {
  RirResolvedBeamState state;
  state.effective_beamwidth_deg = RirResolveEffectiveBeamwidth(antenna_config, wavelength_m);
  state.beam_pointing_deg = beam_pointing_deg;

  oneq::common::radar::FrozenBeamResolveInputs inputs;
  inputs.main_beam_gain_db = antenna_config.main_beam_gain_db;
  inputs.enable_directional_pattern = true;  // RIR 方向图恒开（开关已删，AR 侧仍保留该入参）。
  inputs.pattern = rir_antenna_pattern_adapter::ToCommonPatternConfig(antenna_config);
  inputs.effective_beamwidth_deg = state.effective_beamwidth_deg;
  inputs.beam_pointing_az_deg = beam_pointing_deg.az_deg;
  inputs.beam_pointing_el_deg = beam_pointing_deg.el_deg;
  inputs.look_az_deg = target_look_az_deg;
  inputs.look_el_deg = target_look_el_deg;
  inputs.has_look_angles = has_look_angles;
  inputs.antenna_length_m = antenna_config.antenna_length_m;
  inputs.antenna_width_m = antenna_config.antenna_width_m;
  inputs.wavelength_m = wavelength_m;
  inputs.normalize_azimuth_delta = true;
  const oneq::common::radar::FrozenBeamResolveResult frozen =
      oneq::common::radar::ResolveFrozenBeamState(inputs);
  state.one_way_antenna_gain_db = frozen.one_way_antenna_gain_db;
  return state;
}

}  // namespace dwell
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_BEAM_CONTROL_H_
