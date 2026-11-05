/**
 * @file RirDetectionCellResolver.cpp
 * @brief RIR detection cell 薄适配：转调 common DetectionCellResolver（anti_rgpo=false）。
 */

/**
 * ## 过程：检测单元 SINR 账本（LRDR 探测链 · 第 6 步）
 * - 上游：RirDetectionCellConfig（本周期波形/接收窗/匹配滤波带宽/单程增益/接收损耗/噪声
 *   系数/四项信号处理增益/MTI-MTD 开关）、RirDetectionCellTarget（斜距/径向速度/RCS/双程
 *   附加损耗/积累脉冲数）、自发射身份与入射链、逐目标主瓣杂波功率。
 * - 本步：转调公共核 DetectionCellResolver（本模块 anti_rgpo 恒关），求解
 *   ① 回波功率 P_e ∝ P_t·G²·λ²·σ /((4π)³·R⁴·L)，含脉压能量增益 G_pc = max(1, Bτ)；
 *   ② 热噪声 N = kT₀·B·F（匹配滤波带宽口径）；③ 干扰/杂波按各自抑制增益进分母；
 *   ④ 单脉冲 SINR = P_e·G_target /(N·G_noise + P_j/G_jam + P_c/G_clutter)（线性与 dB 双口径）；
 *   ⑤ MTI 响应/MTD 通道响应/相干积累/噪声总增益（仅 enable_mti_mtd_effects 开启时可读）。
 * - 下游：RirSignalDetector::DetectResolvedCell（CFAR 判决）、量测误差模型的生效积累脉冲数、
 *   验收 detection_cell 事件的分项功率与增益。
 * - 关键参数：hardware.signal_processing.{target_processing_gain_db(3),
 *   noise_processing_gain_db(1),clutter_suppression_gain_db(10),
 *   jamming_suppression_gain_db(8),enable_mti_mtd_effects(false)}、
 *   hardware.receiver.{receive_loss_db,noise_figure_db}；调用方按
 *   hardware 开关 ∧ policy.detection.pulse_count ≥ 2 决定是否并入 MTI/MTD。
 * - 对应验收项：第 31 项噪声功率、第 32 项目标信号增益、第 33 项噪声增益、第 34 项杂波
 *   处理增益、第 35 项干扰处理增益、第 36 项恒虚警检测（SINR 输入）、第 70/71 项内部接口。
 */

#include "remote_identification_radar/dwell/RirDetectionCellResolver.h"

#include "common/radar/DetectionCellResolver.h"

namespace remote_identification_radar {
namespace dwell {
namespace {

// 过程：配置镜像到公共核（本模块 anti_rgpo 恒关；波形/接收窗/带宽/增益按配置原值透传）。
oneq::common::radar::DetectionCellConfig ToCommon(const RirDetectionCellConfig& config) {
  oneq::common::radar::DetectionCellConfig out;
  out.own_transmit_waveform = config.own_transmit_waveform;
  out.receive_window_start_time_s = config.receive_window_start_time_s;
  out.receive_window_duration_s = config.receive_window_duration_s;
  out.matched_filter_bandwidth_hz = config.matched_filter_bandwidth_hz;
  out.one_way_antenna_gain_dbi = config.one_way_antenna_gain_dbi;
  out.receiver_loss_db = config.receiver_loss_db;
  out.receiver_noise_figure_db = config.receiver_noise_figure_db;
  out.reference_temperature_k = config.reference_temperature_k;
  out.enable_anti_rgpo_leading_edge = false;
  // 过程：四项处理增益 + MTI/MTD 使能（需模块开关与检测单元开关同时为真才并入联算）。
  out.signal_processing.target_processing_gain_db =
      static_cast<double>(config.signal_processing.target_processing_gain_db);
  out.signal_processing.noise_processing_gain_db =
      static_cast<double>(config.signal_processing.noise_processing_gain_db);
  out.signal_processing.clutter_suppression_gain_db =
      static_cast<double>(config.signal_processing.clutter_suppression_gain_db);
  out.signal_processing.jamming_suppression_gain_db =
      static_cast<double>(config.signal_processing.jamming_suppression_gain_db);
  out.enable_mti_mtd_effects =
      config.enable_mti_mtd_effects && config.signal_processing.enable_mti_mtd_effects;
  return out;
}

// 过程：目标量镜像——斜距/径向速度/RCS/双程附加传播损耗/生效积累脉冲数。
oneq::common::radar::DetectionCellTarget ToCommon(const RirDetectionCellTarget& target) {
  oneq::common::radar::DetectionCellTarget out;
  out.range_m = target.range_m;
  out.closing_radial_velocity_mps = target.closing_radial_velocity_mps;
  out.rcs_m2 = target.rcs_m2;
  out.two_way_additional_propagation_loss_db = target.two_way_additional_propagation_loss_db;
  out.effective_pulse_count = target.effective_pulse_count;
  return out;
}

// 过程：结果回写——分项功率（回波/热噪/干扰/杂波）、脉压增益、单脉冲 SINR（线性 + dB）、
//       MTI/MTD 响应与噪声总增益（验收分项与量测误差模型的输入口径）。
void FromCommon(const oneq::common::radar::DetectionCellResult& in, RirDetectionCellResult* out) {
  out->echo_delay_s = in.echo_delay_s;
  out->two_way_doppler_shift_hz = in.two_way_doppler_shift_hz;
  out->echo_power_w = in.echo_power_w;
  out->pulse_compression_gain = in.pulse_compression_gain;
  out->thermal_noise_power_w = in.thermal_noise_power_w;
  out->interference_power_w = in.interference_power_w;
  out->clutter_power_w = in.clutter_power_w;
  out->processed_single_pulse_sinr_linear = in.processed_single_pulse_sinr_linear;
  out->processed_single_pulse_sinr_db = in.processed_single_pulse_sinr_db;
  out->effective_pulse_count = in.effective_pulse_count;
  out->folded_doppler_hz = in.folded_doppler_hz;
  out->mti_response_db = in.mti_response_db;
  out->mtd_bin_response_db = in.mtd_bin_response_db;
  out->coherent_integration_db = in.coherent_integration_db;
  out->noise_total_gain_db = in.noise_total_gain_db;
}

}  // namespace

// 过程：检测单元求解——公共核返回失败（波形/接收窗/数值非法）时返回 false，调用方回退效能级检测路径。
bool TryResolveRirDetectionCell(
    const RirDetectionCellConfig& config, const RirDetectionCellTarget& target,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double clutter_power_w, RirDetectionCellResult* result) {
  if (result == nullptr) {
    return false;
  }
  oneq::common::radar::DetectionCellResult common_result;
  if (!oneq::common::radar::TryResolveDetectionCell(ToCommon(config), ToCommon(target),
                                                    own_emission_identity, incident_links,
                                                    clutter_power_w, &common_result)) {
    return false;
  }
  FromCommon(common_result, result);
  return true;
}

}  // namespace dwell
}  // namespace remote_identification_radar
