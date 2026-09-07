/**
 * @file scenes/scene_script.cpp
 * @brief 世界模型目标真值脚本实现（见 scenes/scene_script.h）。
 */

#include "scenes/scene_script.h"

#include <algorithm>
#include <cmath>

#include "1q/coordinate/scene_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"
#include "1q/coordinate/velocity_transform.h"

namespace component_attachment {
namespace app {

std::vector<TargetEcefState> MakeTargetStates(
    const std::vector<ScriptedTarget>& script,
    const oneq::coordinate::LlaPositionDegM& platform_origin) {
  std::vector<TargetEcefState> states;
  states.reserve(script.size());
  for (const auto& entry : script) {
    TargetEcefState state;
    state.id = entry.id;
    state.type = entry.type;
    if (entry.is_ballistic) {
      // 弹道条目：加载时闭式解二体椭圆（起止 LLA + 顶高 + 顶高时刻），初始
      // 状态 = 场景时刻 0（助推段：静止于发射点）。场景加载已在解析期校验过
      // 可解性，此处失败属防御分支：退化为静止于发射点（有物理意义的降级，
      // 不留零坐标）。不参与后续 ENU 方位/斜距投影（ENU 字段与弹道互斥）。
      if (!SolveBallisticTrajectory(entry.start_lla, entry.end_lla, entry.max_alt_m,
                                    entry.max_alt_time_s, &state.ballistic)) {
        oneq::coordinate::TryLlaToEcef(entry.start_lla, &state.position);
      } else {
        PropagateBallistic(state.ballistic, 0.0, &state.position, &state.velocity);
      }
    } else {
      oneq::coordinate::EnuPositionM offset;
      // 目标脚本（方位/距离/高度）→ ENU 水平偏移；高度由场景文件显式给出
      // （TryBearingRangeToEnuOffset 清零 up，须在此重设）。投影必然成功，
      // 失败时位置留默认零向量。
      if (oneq::coordinate::TryBearingRangeToEnuOffset(entry.azimuth_deg, entry.range_m,
                                                       &offset)) {
        offset.up_m = entry.altitude_m;
        oneq::coordinate::EcefPositionM target;
        if (oneq::coordinate::TryEnuToEcef(offset, platform_origin, &target)) {
          state.position = target;
        }
      }
      oneq::coordinate::EnuVelocityMps enu_velocity;
      enu_velocity.east_mps = entry.v_east_mps;
      enu_velocity.north_mps = entry.v_north_mps;
      enu_velocity.up_mps = 0.0;
      // 脚本输入合法（有限/非负），投影必然成功；失败时 velocity 留默认零向量。
      oneq::coordinate::TryEnuToEcefVelocity(enu_velocity, platform_origin, &state.velocity);
    }
    state.rcs = static_cast<float>(entry.rcs);
    state.temperature_k = static_cast<float>(entry.temperature_k);
    state.projected_area_m2 = static_cast<float>(entry.projected_area_m2);
    state.radiant_intensity_w_per_sr = entry.radiant_intensity_w_per_sr;
    state.emitter_center_frequency_hz = entry.emitter_center_frequency_hz;
    state.has_rir_features = entry.has_rir_features;
    state.rir_rcs_dbsm = entry.rir_rcs_dbsm;
    state.rir_pol_dictionary = entry.rir_pol_dictionary;
    state.rir_pol_window_deg = entry.rir_pol_window_deg;
    state.rir_truth_model = entry.rir_truth_model;
    state.rir_scatterers = entry.rir_scatterers;
    state.maneuvers = entry.maneuvers;
    states.push_back(state);
  }
  return states;
}

std::vector<oneq::electromagnetics::RfSceneEmission> MakeEmitterTruths(
    const std::vector<TargetEcefState>& states, const EsrEmitterParams& esr,
    double window_start_time_s) {
  std::vector<oneq::electromagnetics::RfSceneEmission> emitters;
  emitters.reserve(states.size());
  for (const auto& state : states) {
    if (state.emitter_center_frequency_hz <= 0.0) {
      continue;  // 该目标不配辐射源（场景级开关）
    }
    oneq::electromagnetics::RfSceneEmission emitter;
    emitter.identity.sensor_entity_id = state.id;
    emitter.identity.emission_id = 1U;
    emitter.position_ecef_m = state.position;
    emitter.velocity_ecef_mps = state.velocity;
    emitter.antenna.peak_gain_dbi = esr.peak_gain_dbi;
    emitter.polarization = oneq::electromagnetics::RfScenePolarization::kHorizontal;
    // 脉冲列 @ 10 GHz 级：单周期积分脉冲数足够，统计检测概率趋近 1。
    if (!oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
            window_start_time_s, state.emitter_center_frequency_hz, esr.bandwidth_hz,
            esr.peak_power_w, esr.pulse_width_s, esr.pri_s, esr.pulse_count, 0.0,
            /*timing_seed=*/esr.timing_seed, /*timing_epoch=*/1U, &emitter.waveform)) {
      continue;  // 波形构造失败：该辐射源本周期不发射
    }
    emitters.push_back(emitter);
  }
  return emitters;
}

std::vector<sbirs_sensor::session::SbirsSceneTarget> MakeSbirsTargetInputs(
    const std::vector<TargetEcefState>& states) {
  std::vector<sbirs_sensor::session::SbirsSceneTarget> targets;
  targets.reserve(states.size());
  for (const auto& state : states) {
    sbirs_sensor::session::SbirsSceneTarget target;
    target.target_id = state.id;
    target.target_name = "ir_target_" + std::to_string(state.id);
    target.position_ecef_m.x = state.position.x_m;
    target.position_ecef_m.y = state.position.y_m;
    target.position_ecef_m.z = state.position.z_m;
    target.radiant_intensity_w_per_sr = state.radiant_intensity_w_per_sr;
    target.velocity_ecef_m_per_s.x = state.velocity.x_mps;
    target.velocity_ecef_m_per_s.y = state.velocity.y_mps;
    target.velocity_ecef_m_per_s.z = state.velocity.z_mps;
    target.has_velocity_ecef_m_per_s = true;
    target.active = true;
    targets.push_back(target);
  }
  return targets;
}

std::vector<sar::session::SarPointTarget> MakeSarPointTargets(
    const std::vector<TargetEcefState>& states) {
  std::vector<sar::session::SarPointTarget> targets;
  targets.reserve(states.size());
  for (const auto& state : states) {
    oneq::coordinate::LlaPositionDegM lla;
    if (!oneq::coordinate::TryEcefToLla(state.position, &lla)) {
      continue;  // 坐标转换失败：该点目标本周期不入 SAR 场景
    }
    sar::session::SarPointTarget target;
    target.target_id = state.id;
    target.target_name = "sar_target_" + std::to_string(state.id);
    target.latitude_deg = lla.latitude_deg;
    target.longitude_deg = lla.longitude_deg;
    target.altitude_m = lla.altitude_m;
    target.radar_cross_section_dbsm =
        10.0 * std::log10(std::max(1.0e-6, static_cast<double>(state.rcs)));
    targets.push_back(target);
  }
  return targets;
}

std::vector<remote_identification_radar::session::RirSceneTarget> MakeRirSceneTargets(
    const std::vector<TargetEcefState>& states,
    const oneq::coordinate::LlaPositionDegM& site_origin) {
  namespace rir = remote_identification_radar::session;
  std::vector<rir::RirSceneTarget> targets;
  targets.reserve(states.size());
  for (const auto& state : states) {
    oneq::coordinate::ExternalKinematics kinematics;
    kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    kinematics.position_ecef_m = state.position;
    kinematics.velocity_mps = state.velocity;
    oneq::coordinate::EnuSceneState enu;
    if (!oneq::coordinate::TryMakeEnuSceneState(kinematics, site_origin, &enu)) {
      continue;  // 坐标转换失败：该目标本周期不入 RIR 场景
    }
    rir::RirSceneTarget target;
    target.external_target_id = state.id;
    target.target_name = state.rir_truth_model;
    target.position_x = static_cast<float>(enu.position_enu_m.east_m);
    target.position_y = static_cast<float>(enu.position_enu_m.north_m);
    target.position_z = static_cast<float>(enu.position_enu_m.up_m);
    target.velocity_x = static_cast<float>(enu.velocity_enu_mps.east_mps);
    target.velocity_y = static_cast<float>(enu.velocity_enu_mps.north_mps);
    target.velocity_z = static_cast<float>(enu.velocity_enu_mps.up_mps);
    target.rcs = state.rcs;
    target.target_swerling_type = rir::RirSwerlingType::kSwerling0;
    if (state.has_rir_features) {
      // 特征真值铺样：RCS 视角网格方位 ±10°/步 5°、俯仰 5°~30°/步 10°
      // （跨度 20° 覆盖特征库视角门槛，2026-08-31 修订 2 定位）；散射器逐条
      // 透传。极化走架构 B 窗口（2026-09-03 冻结）：全量字典开机一次性载入
      // 在此，每周期只按当前视线角裁 ±窗口的行进周期输入，不再整表重交、
      // 也不复制占位值——标准差因此有真实含义。
      for (float az = -10.0f; az <= 10.0f; az += 5.0f) {
        for (float el = 5.0f; el <= 30.0f; el += 10.0f) {
          rir::RirAspectRcsSample aspect;
          aspect.aspect_az_deg = az;
          aspect.aspect_el_deg = el;
          aspect.rcs_dbsm = static_cast<float>(state.rir_rcs_dbsm);
          target.aspect_rcs_samples.push_back(aspect);
        }
      }
      if (!state.rir_pol_dictionary.empty()) {
        // 视线角与库内 ComputeLookAngles 同口径：az=atan2(北,东)、
        // el=atan2(天,水平距离)（站点 ENU，雷达位于原点）。
        const double east = static_cast<double>(enu.position_enu_m.east_m);
        const double north = static_cast<double>(enu.position_enu_m.north_m);
        const double up = static_cast<double>(enu.position_enu_m.up_m);
        const double look_az_deg =
            std::atan2(north, east) * 180.0 / 3.14159265358979323846;
        const double look_el_deg =
            std::atan2(up, std::sqrt(east * east + north * north)) * 180.0 /
            3.14159265358979323846;
        const double window_deg = state.rir_pol_window_deg;
        for (const rir::RirPolSMatrixSample& row : state.rir_pol_dictionary) {
          const double delta_az = std::fabs(static_cast<double>(row.aspect_az_deg) - look_az_deg);
          const double delta_el = std::fabs(static_cast<double>(row.aspect_el_deg) - look_el_deg);
          if (delta_az <= window_deg && delta_el <= window_deg) {
            target.polarization_samples.push_back(row);
          }
        }
      }
      for (const auto& scatterer : state.rir_scatterers) {
        rir::RirRangeRcsScatterer entry;
        entry.range_offset_m = static_cast<float>(scatterer.offset_m);
        entry.rcs_dbsm = static_cast<float>(scatterer.rcs_dbsm);
        target.range_rcs_scatterers.push_back(entry);
      }
    }
    targets.push_back(target);
  }
  return targets;
}

void AdvanceTargetStates(std::vector<TargetEcefState>& states, std::uint32_t cycle,
                         double dt_s,
                         const oneq::coordinate::LlaPositionDegM& platform_origin) {
  for (auto& state : states) {
    if (state.ballistic.valid) {
      // 弹道目标：按绝对场景时刻 t = cycle·dt 解析求值（助推段占位 / Kepler
      // 弧段），不累积不漂移；机动表与 ENU 语义和弹道互斥，不适用。
      PropagateBallistic(state.ballistic, static_cast<double>(cycle) * dt_s,
                         &state.position, &state.velocity);
      continue;
    }
    // 变速机动：start_cycle 严格递增（加载器校验），逐条对比取生效条目。
    for (const auto& maneuver : state.maneuvers) {
      if (maneuver.start_cycle == cycle) {
        // 机动速度为局部 ENU（东/北），投影回 ECEF（与 MakeTargetStates
        // 初始速度投影一致；投影输入合法必然成功，失败时速度留零向量）。
        oneq::coordinate::EnuVelocityMps enu_velocity;
        enu_velocity.east_mps = maneuver.v_east_mps;
        enu_velocity.north_mps = maneuver.v_north_mps;
        enu_velocity.up_mps = 0.0;
        oneq::coordinate::TryEnuToEcefVelocity(enu_velocity, platform_origin,
                                               &state.velocity);
        break;
      }
    }
    state.position.x_m += state.velocity.x_mps * dt_s;
    state.position.y_m += state.velocity.y_mps * dt_s;
    state.position.z_m += state.velocity.z_mps * dt_s;
  }
}

}  // namespace app
}  // namespace component_attachment
