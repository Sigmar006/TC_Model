#include "remote_identification_radar/dwell/RirReceiverStateBuilder.h"

/**
 * ## 过程：接收机工作状态与噪声口径（LRDR 探测链 · 第 5 步）
 * - 上游：RirRfCycleInput（设备号/平台位置速度/波束指向/接收窗）、本周期自发射（天线与
 *   极化描述）、hardware.receiver（噪声系数/接收损耗/线性上限/极化/交叉极化隔离）、
 *   hardware.transmitter.bandwidth_hz（匹配滤波带宽）。
 * - 本步：装配 RfSceneReceiverState（几何、天线、极化、接收窗、中心频率、预选带宽、接收
 *   链路损耗、远场下限、同址隔离）与 RirReceiverOperatingState（波束指向、匹配滤波带宽
 *   B = 发射带宽、噪声系数 F、线性输入上限）。热噪声功率本身统一由
 *   RirRadarEquations::ComputeThermalNoisePower_W 按 N = kT₀·B·F 计算（第 8 步），
 *   本步只把 B 与 F 如实透传，保证检测单元、杂波模型与回退检测器三处噪声同源。
 * - 下游：RirRfFrontEndResolver（入射链与饱和判定）、检测单元与回退检测器（噪声底）、
 *   验收 detection_cell 的噪声分项。
 * - 关键参数：hardware.receiver.{noise_figure_db(4 dB),receive_loss_db(2 dB),
 *   maximum_linear_input_power_w(1e-3 W),scene_polarization,cross_polarization_isolation_db}、
 *   hardware.transmitter.bandwidth_hz；库内常量 kDefaultPreselectorBandwidthHz = 20 MHz、
 *   kDefaultMinimumFarFieldRangeM = 1 m、kDefaultCoSiteIsolationDb = 120 dB（不暴露为配置）。
 * - 对应验收项：第 31 项接收机噪声功率计算、第 32 项目标信号增益（接收链路损耗口径）、
 *   第 61 项雷达平台与工作参数配置接口。
 */

namespace remote_identification_radar {
namespace dwell {
namespace {

// 过程：库内缺省常量——预选带宽 20 MHz、远场下限 1 m、同平台隔离 120 dB（刻意不暴露为公开配置）。
constexpr double kDefaultPreselectorBandwidthHz = 20.0e6;
constexpr double kDefaultMinimumFarFieldRangeM = 1.0;
constexpr double kDefaultCoSiteIsolationDb = 120.0;

}  // namespace

// 过程：接收机状态装配——几何/极化/接收窗给射频前端；带宽/噪声系数/线性上限给检测链。
RirReceiverOperatingState RirReceiverStateBuilder::Build(
    const RirRfCycleInput& input, const oneq::electromagnetics::RfSceneEmission& emission,
    const config::RirHardwareConfig& hardware, double carrier_hz) {
  const config::hardware::RirReceiverConfig& receiver = hardware.receiver;
  const config::hardware::RirTransmitterConfig& transmitter = hardware.transmitter;

  // 过程：接收机几何与场景描述（天线/极化与自发射同源，收发共用一套波束描述）。
  RirReceiverOperatingState operating_state;
  oneq::electromagnetics::RfSceneReceiverState& receiver_state = operating_state.rf_receiver;
  receiver_state.sensor_entity_id = input.sensor_entity_id;
  receiver_state.position_ecef_m = input.platform_position_ecef_m;
  receiver_state.velocity_ecef_mps = input.platform_velocity_ecef_mps;
  receiver_state.antenna = emission.antenna;
  receiver_state.polarization = receiver.scene_polarization;
  receiver_state.window_start_time_s = input.window_start_time_s;
  receiver_state.window_duration_s = input.window_duration_s;
  receiver_state.center_frequency_hz = carrier_hz;
  // 预选带宽 / 远场下限 / 同平台隔离：库内默认，不暴露为公开配置。
  receiver_state.bandwidth_hz = kDefaultPreselectorBandwidthHz;
  receiver_state.receiver_system_loss_db = static_cast<double>(receiver.receive_loss_db);
  receiver_state.minimum_far_field_range_m = kDefaultMinimumFarFieldRangeM;
  receiver_state.co_site_paths = {oneq::electromagnetics::RfCoSiteIsolationPath(
      input.sensor_entity_id, kDefaultCoSiteIsolationDb)};
  // 过程：检测链工作状态——波束指向、匹配滤波带宽 B = 发射带宽、噪声系数 F、线性输入上限。
  operating_state.beam_pointing_deg = input.beam_pointing_deg;
  operating_state.matched_filter_bandwidth_hz = static_cast<double>(transmitter.bandwidth_hz);
  operating_state.receiver_noise_figure_db = static_cast<double>(receiver.noise_figure_db);
  operating_state.maximum_linear_input_power_w =
      static_cast<double>(receiver.maximum_linear_input_power_w);
  return operating_state;
}

}  // namespace dwell
}  // namespace remote_identification_radar
