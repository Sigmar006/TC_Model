/**
 * @file RirRadarEquations.cpp
 * @brief RIR 雷达方程薄适配层（common 单源）。
 */

/**
 * ## 过程：雷达方程与检测概率（LRDR 探测链 · 第 8 步）
 * - 上游：hardware.transmitter（峰值功率/发射损耗/脉宽/带宽/频率）、hardware.receiver
 *   （噪声系数）、天线单程增益、目标 RCS（第 4 步）与斜距（几何段）、传播损耗（第 9 步）。
 * - 本步：本文件是公共域雷达方程单源之上的薄适配，提供
 *   ① 回波功率 P_r = P_t·G²·λ²·σ /((4π)³·R⁴·L_prop)（dBW，公共核内含脉宽 13 µs 能量基准）；
 *   ② 脉压增益 10·log10(max(1, Bτ))，以及回退路径的基准修正 10·log10(Bτ) − 10·log10(τ/13 µs)；
 *   ③ 热噪声功率 N = k·T₀·B·F（B 取发射/匹配滤波带宽，T₀ = 290 K，F 为噪声系数）；
 *   ④ 测距/测角误差标准差（0.5·δ_R/√SNR 与 0.317·θ_3dB/√SNR，含低 SNR 保护分支）；
 *   ⑤ CFAR 门限、Marcum Q、Swerling 检测概率 P_d 与蒙特卡洛抽签。
 * - 下游：RirSignalDetector（回波功率/热噪声/P_d/抽签）、RirSurfaceClutterModel（回波功率与
 *   脉压增益）、RirMeasurementErrorModel（距离/角度误差标准差）。
 * - 关键参数：hardware.transmitter.{peak_power_w,transmit_loss_db,pulse_width_s,
 *   frequency_hz,bandwidth_hz}、hardware.receiver.noise_figure_db、
 *   hardware.antenna.main_beam_gain_db（主瓣口径调用方）。
 * - 对应验收项：第 31 项接收机噪声功率、第 32 项目标信号增益、第 36 项恒虚警检测
 *   （门限/Pd）、第 38 项测距测角（误差标准差）、第 70 项回波与信号处理交互接口。
 */

#include "remote_identification_radar/internal/RirRadarEquations.h"

#include <cmath>

#include "common/radar/RadarEquations.h"

namespace remote_identification_radar {
namespace internal {

namespace {

// 过程：起伏型枚举镜像（未知取值按 kSwerling0 非起伏处理）。
oneq::common::radar::SwerlingModel ToCommonSwerling(RirSwerlingModel model) {
  switch (model) {
    case RirSwerlingModel::kSwerling1:
      return oneq::common::radar::SwerlingModel::kSwerling1;
    case RirSwerlingModel::kSwerling2:
      return oneq::common::radar::SwerlingModel::kSwerling2;
    case RirSwerlingModel::kSwerling3:
      return oneq::common::radar::SwerlingModel::kSwerling3;
    case RirSwerlingModel::kSwerling4:
      return oneq::common::radar::SwerlingModel::kSwerling4;
    case RirSwerlingModel::kSwerling0:
    default:
      return oneq::common::radar::SwerlingModel::kSwerling0;
  }
}

}  // namespace

// 过程：回波功率（dBW，单程增益口径）——逐目标离轴增益代入雷达方程：P_r ∝ P_t·G²·λ²·σ/((4π)³R⁴L)。
float RirRadarEquations::ComputeEchoPowerWithGain_dBW(
    const config::hardware::RirTransmitterConfig& tx, float one_way_gain_db, float rcs_m2,
    float range_m, float propagation_loss_db) {
  return oneq::common::radar::RadarEquations::ComputeEchoPowerWithGain_dBW(
      tx.peak_power_w, tx.transmit_loss_db, tx.pulse_width_s, tx.frequency_hz, one_way_gain_db,
      rcs_m2, range_m, propagation_loss_db);
}

// 过程：回波功率（dBW，主瓣峰值增益口径）——杂波回波走同一方程，用 G₀ 近似。
float RirRadarEquations::ComputeEchoPower_dBW(
    const config::hardware::RirTransmitterConfig& tx,
    const config::hardware::RirAntennaConfig& ant, float rcs_m2, float range_m,
    float propagation_loss_db) {
  return oneq::common::radar::RadarEquations::ComputeEchoPower_dBW(
      tx.peak_power_w, tx.transmit_loss_db, tx.pulse_width_s, tx.frequency_hz,
      ant.main_beam_gain_db, rcs_m2, range_m, propagation_loss_db);
}

// 过程：脉压增益 G_pc = 10·log10(B·τ)（时间带宽积 ≤ 1 或输入非法时退化为 0 dB）。
float RirRadarEquations::ComputePulseCompressionGainDb(
    const config::hardware::RirTransmitterConfig& tx) {
  // 非法带宽/脉宽（非正或非有限，含 NaN）按无增益退化，不污染消费侧账本。
  if (!std::isfinite(tx.bandwidth_hz) || !std::isfinite(tx.pulse_width_s) ||
      !(tx.bandwidth_hz > 0.0f) || !(tx.pulse_width_s > 0.0f)) {
    return 0.0f;
  }
  const float time_bandwidth_product = tx.bandwidth_hz * tx.pulse_width_s;
  return time_bandwidth_product > 1.0f ? 10.0f * std::log10(time_bandwidth_product) : 0.0f;
}

// 过程：回退路径脉压修正 = G_pc − 10·log10(τ/13 µs)：先抵掉公共核的参考脉宽能量缩放，
//       再按 B·τ 施加一次，使回退路径与检测单元的单脉冲能量口径一致。
float RirRadarEquations::ComputeFallbackPulseCompressionCorrectionDb(
    const config::hardware::RirTransmitterConfig& tx) {
  constexpr float kReferencePulseWidthS = 13.0e-6f;
  if (!std::isfinite(tx.bandwidth_hz) || !std::isfinite(tx.pulse_width_s) ||
      !(tx.bandwidth_hz > 0.0f) || !(tx.pulse_width_s > 0.0f)) {
    return 0.0f;
  }
  const float reference_scale_db =
      10.0f * std::log10(tx.pulse_width_s / kReferencePulseWidthS);
  return ComputePulseCompressionGainDb(tx) - reference_scale_db;
}

// 过程：热噪声功率 N = k·T₀·B·F（W）——B 取发射/匹配滤波带宽，F 取接收机噪声系数。
float RirRadarEquations::ComputeThermalNoisePower_W(
    const config::hardware::RirTransmitterConfig& tx,
    const config::hardware::RirReceiverConfig& rx) {
  return oneq::common::radar::RadarEquations::ComputeThermalNoisePower_W(tx.bandwidth_hz,
                                                                          rx.noise_figure_db);
}

// 过程：测距误差 σ_R = 0.5·δ_R/√SNR，δ_R = c/(2B)；极低 SNR 走公共核保护分支。
float RirRadarEquations::ComputeRangeErrorStdDev(float snr_db, float bandwidth_hz) {
  return oneq::common::radar::RadarEquations::ComputeRangeErrorStdDev(snr_db, bandwidth_hz);
}

// 过程：测角误差 σ_θ = 0.317·θ_3dB/√SNR（工程近似系数，非教科书 1/√(2·SNR)）。
float RirRadarEquations::ComputeAngleErrorStdDev(float snr_db, float beamwidth_rad) {
  return oneq::common::radar::RadarEquations::ComputeAngleErrorStdDev(snr_db, beamwidth_rad);
}

// 过程：CFAR 判决门限（按虚警概率 P_fa 与积累脉冲数 N）。
double RirRadarEquations::ComputeThreshold(double pfa, int num_pulses) {
  return oneq::common::radar::RadarEquations::ComputeThreshold(pfa, num_pulses);
}

// 过程：Marcum Q 函数——Swerling/非起伏目标检测概率的数值核。
double RirRadarEquations::MarcumQ(int order, double a, double b) {
  return oneq::common::radar::RadarEquations::MarcumQ(order, a, b);
}

// 过程：检测概率 P_d(SINR, P_fa, 起伏型, N)。
float RirRadarEquations::ComputeDetectionProbability(float snr_db, float pfa,
                                                     RirSwerlingModel model, int num_pulses) {
  return oneq::common::radar::RadarEquations::ComputeDetectionProbability(
      snr_db, pfa, ToCommonSwerling(model), num_pulses);
}

// 过程：蒙特卡洛抽签——u ~ U(0,1)，u < P_d 判为检测；随机数状态由 policy.detection.random_seed 决定。
bool RirRadarEquations::ThresholdDecision(float detection_prob, std::mt19937& rng) {
  return oneq::common::radar::RadarEquations::ThresholdDecision(detection_prob, rng);
}

}  // namespace internal
}  // namespace remote_identification_radar
