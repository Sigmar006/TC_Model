#include "remote_identification_radar/dwell/RirEmissionFactory.h"

#include <algorithm>
#include <cmath>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "common/numerics/Constants.h"

/**
 * ## 过程：发射信号与波形参数（LRDR 探测链 · 第 2 步）
 * - 上游：hardware.transmitter（峰值功率/频率/带宽/脉宽/PRF/发射损耗）、hardware.antenna
 *   （主瓣增益/波束宽度/旁瓣与后瓣电平）、hardware.receiver（极化/交叉极化隔离）、
 *   RirController 解析的驻留指向（首驻留中心）与驻留时序（PRI、驻留内脉冲数）。
 * - 本步：把硬件与指向描述装配成一条 RfSceneEmission——波束指向单位矢量（az/el → 局部 →
 *   ENU → ECEF）、天线与极化描述、辐射峰值功率 P_rad = P_peak·10^(−L_tx/10)（W），以及
 *   脉冲串波形（载频/带宽/脉宽/PRI/脉冲数/时序种子）。
 * - 下游：RfSceneFrame.emissions（供 RF 前端求入射链与同址耦合）、RirReceiverStateBuilder
 *   （中心频率/天线/极化）、检测单元（占用带宽/脉冲数）、对外发射帧（每驻留一条）。
 * - 关键参数：hardware.transmitter.{peak_power_w,transmit_loss_db,frequency_hz,bandwidth_hz,
 *   pulse_width_s,prf_hz}、hardware.antenna.{main_beam_gain_db,nominal_az_beamwidth_deg,
 *   nominal_el_beamwidth_deg,max_sidelobe_level_db,backlobe_level_db}、hardware.receiver.
 *   {scene_polarization,cross_polarization_isolation_db}；脉冲数 = ceil(驻留时长 / PRI)。
 * - 对应验收项：第 32 项目标信号增益（发射功率/增益口径）、第 48 项波束扫描（指向）、
 *   第 61 项雷达平台与工作参数配置接口、第 70 项雷达回波与信号处理交互接口。
 */

namespace remote_identification_radar {
namespace dwell {
namespace {

bool TryResolveEcefBoresight(const RirRfCycleInput& input,
                             oneq::electromagnetics::RfSceneDirection* boresight_ecef) {
  if (boresight_ecef == nullptr || !oneq::coordinate::IsFinite(input.radar_frame_attitude_deg) ||
      !std::isfinite(input.beam_pointing_deg.az_deg) ||
      !std::isfinite(input.beam_pointing_deg.el_deg) || input.beam_pointing_deg.az_deg < -180.0f ||
      input.beam_pointing_deg.az_deg > 180.0f || input.beam_pointing_deg.el_deg < -90.0f ||
      input.beam_pointing_deg.el_deg > 90.0f) {
    return false;
  }
  // 过程：az/el（deg）→ 局部单位矢量 [cosε·cosα, cosε·sinα, sinε]；再经平台姿态旋到 ENU、经 LLA 锚点旋到 ECEF。
  using oneq::common::numerics::kPi;
  const double azimuth_rad = static_cast<double>(input.beam_pointing_deg.az_deg) * kPi / 180.0;
  const double elevation_rad = static_cast<double>(input.beam_pointing_deg.el_deg) * kPi / 180.0;
  const double cos_elevation = std::cos(elevation_rad);
  const oneq::coordinate::Vector3d local_direction{cos_elevation * std::cos(azimuth_rad),
                                                   cos_elevation * std::sin(azimuth_rad),
                                                   std::sin(elevation_rad)};
  // 过程：局部 → ENU（按平台姿态旋转）→ ECEF（按平台 LLA 锚点）；任一转换失败即返回 false。
  const oneq::coordinate::Vector3d enu_direction = oneq::coordinate::RotateLocalToEnu(
      local_direction.x, local_direction.y, local_direction.z, input.radar_frame_attitude_deg);
  oneq::coordinate::LlaPositionDegM platform_lla;
  oneq::coordinate::Vector3d resolved_ecef;
  if (!oneq::coordinate::TryEcefToLla(input.platform_position_ecef_m, &platform_lla) ||
      !oneq::coordinate::TryEnuToEcefDirection(enu_direction, platform_lla, &resolved_ecef)) {
    return false;
  }
  boresight_ecef->x = resolved_ecef.x;
  boresight_ecef->y = resolved_ecef.y;
  boresight_ecef->z = resolved_ecef.z;
  return true;
}

}  // namespace

// 过程：载频解析——当前取发射机频率（cycle_index 为预留形参，不参与取值）。
double RirEmissionFactory::ResolveCarrierHz(
    const config::hardware::RirTransmitterConfig& transmitter, std::uint32_t /*cycle_index*/) {
  return static_cast<double>(transmitter.frequency_hz);
}

// 过程：发射记录装配——身份/平台运动/波束指向/天线与极化/辐射功率/脉冲串波形，一次调用产出一条 RfSceneEmission。
bool RirEmissionFactory::TryBuildEmission(
    const RirRfCycleInput& input, const config::RirHardwareConfig& hardware,
    std::uint64_t emission_id, double carrier_hz, double pulse_repetition_interval_s,
    std::uint32_t pulse_count, std::uint64_t timing_seed, std::uint64_t successful_cycle_count,
    oneq::electromagnetics::RfSceneEmission* emission) {
  if (emission == nullptr) {
    return false;
  }
  const config::hardware::RirTransmitterConfig& transmitter = hardware.transmitter;
  const config::hardware::RirReceiverConfig& receiver = hardware.receiver;
  const config::hardware::RirAntennaConfig& antenna = hardware.antenna;

  // 过程：身份与平台运动——接收侧据此识别自发射并剔除同址耦合链（二元组 sensor_entity_id + emission_id）。
  emission->identity.sensor_entity_id = input.sensor_entity_id;
  emission->identity.emission_id = emission_id;
  emission->position_ecef_m = input.platform_position_ecef_m;
  emission->velocity_ecef_mps = input.platform_velocity_ecef_mps;
  // 过程：波束指向 → ECEF 单位矢量；角度非法或坐标转换失败即整条发射构建失败（不虚构指向）。
  if (!TryResolveEcefBoresight(input, &emission->antenna.boresight_ecef)) {
    return false;
  }
  // 过程：天线与极化描述——主瓣峰值增益、半功率波束宽度（取两轴较大者）、旁瓣/后瓣电平、交叉极化隔离。
  emission->antenna.peak_gain_dbi = static_cast<double>(antenna.main_beam_gain_db);
  emission->antenna.half_power_beamwidth_deg = static_cast<double>(
      std::max(antenna.nominal_az_beamwidth_deg, antenna.nominal_el_beamwidth_deg));
  emission->antenna.sidelobe_level_db = static_cast<double>(antenna.max_sidelobe_level_db);
  emission->antenna.backlobe_level_db = static_cast<double>(antenna.backlobe_level_db);
  emission->antenna.cross_polarization_isolation_db =
      static_cast<double>(receiver.cross_polarization_isolation_db);
  emission->polarization = receiver.scene_polarization;

  // 过程：辐射峰值功率 P_rad = P_peak·10^(−L_tx/10)（W）——发射链路损耗在进链路方程前先扣除。
  const double radiated_peak_power_w =
      static_cast<double>(transmitter.peak_power_w) *
      std::pow(10.0, -static_cast<double>(transmitter.transmit_loss_db) / 10.0);
  // 过程：脉冲串波形——中心载频/带宽/脉宽/PRI/脉冲数 + 时序种子（可复现），失败则整条发射不可用。
  return oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      input.window_start_time_s, carrier_hz, static_cast<double>(transmitter.bandwidth_hz),
      radiated_peak_power_w, static_cast<double>(transmitter.pulse_width_s),
      pulse_repetition_interval_s, pulse_count, 0.0, timing_seed, successful_cycle_count,
      &emission->waveform);
}

}  // namespace dwell
}  // namespace remote_identification_radar
