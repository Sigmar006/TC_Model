/**
 * @file RirRadarEquations.h
 * @brief RIR 内部链路预算与检测物理函数薄适配层（common 单源）。
 *
 * 数值实现位于 `oneq::common::radar::RadarEquations`；本层仅负责把
 * `RirHardwareConfig` 与 `RirSwerlingModel` 映射到 common 参数。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_RADAR_EQUATIONS_H_
#define REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_RADAR_EQUATIONS_H_

#include <random>

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"

namespace remote_identification_radar {
namespace internal {

/**
 * @brief 目标起伏模型（值域对齐 `oneq::common::radar::SwerlingModel`）。
 * @note 阶段 2-M 为内部枚举；阶段 2-S 场景目标补 `target_swerling_type` 时
 *       评估升 public（值域保持 0-4 加性扩展）。
 */
enum class RirSwerlingModel {
  kSwerling0 = 0, /**< 非起伏（Marcum 平稳目标）。 */
  kSwerling1 = 1, /**< 慢起伏（扫描间 Rayleigh，Chi-2）。 */
  kSwerling2 = 2, /**< 快起伏（脉冲间 Rayleigh，Chi-2）。 */
  kSwerling3 = 3, /**< 慢起伏（一个主加 Rayleigh，Chi-4）。 */
  kSwerling4 = 4  /**< 快起伏（脉冲间 Chi-4）。 */
};

/**
 * @brief 无状态的雷达物理计算函数集合（识别链路 + 检测子集）。
 */
struct RirRadarEquations {
  /**
   * @brief 单站雷达方程（对数域），按指定单程天线增益计算接收回波功率。
   * 公式: Pr_dBW = Pt_dB + 2*G_one_way_dB + 2*λ_dB + σ_dB
   *                - 30*log10(4π) - 4*R_dB - L_sys
   * @param tx 发射机参数
   * @param one_way_gain_db 当前波束指向下的单程天线增益
   * @param rcs_m2 目标雷达散射截面 (m²)
   * @param range_m 目标斜距 (m)
   * @param propagation_loss_db 大气传播往返损耗 (dB)
   * @return 接收回波功率 (dBW)
   */
  static float ComputeEchoPowerWithGain_dBW(const config::hardware::RirTransmitterConfig& tx,
                                            float one_way_gain_db, float rcs_m2, float range_m,
                                            float propagation_loss_db);

  /**
   * @brief 单站雷达方程（对数域），以天线主瓣峰值增益计算接收回波功率。
   * @param tx 发射机参数
   * @param ant 天线参数
   * @param rcs_m2 目标雷达散射截面 (m²)
   * @param range_m 目标斜距 (m)
   * @param propagation_loss_db 大气传播往返损耗 (dB)
   * @return 接收回波功率 (dBW)
   */
  static float ComputeEchoPower_dBW(const config::hardware::RirTransmitterConfig& tx,
                                    const config::hardware::RirAntennaConfig& ant, float rcs_m2,
                                    float range_m, float propagation_loss_db);

  /**
   * @brief 脉压（匹配滤波）能量增益 10·log10(max(1, B·τ))，单位 dB。
   *
   * 物理：回波与噪声经同一匹配滤波，信号能量按 B·τ 相干积累、噪声底按 B
   * 线性抬升，故匹配滤波后单脉冲 SNR 口径 = 峰值雷达方程回波 × B·τ /
   * (kT·B·NF)。相对 common 雷达方程内置的 τ/13µs 参考能量缩放，本增益的
   * 净修正量 = +10·log10(B·13µs)（与 τ 无关），把消费侧回波能量对齐到与
   * 检测单元路径（common DetectionCellResolver 的 max(1, B_matched·τ)）
   * 相同的 B·τ 基准。
   * @param tx 发射机参数（取带宽 B 与脉宽 τ）
   * @return 脉压能量增益（dB）；B·τ ≤ 1 或 B/τ 非法（非正/非有限）时返回 0。
   * @note 本函数是 RIR 消费层（杂波模型/回退探测回波）对 B·τ 口径的对齐
   *       单源，不改 common 单源行为。
   */
  static float ComputePulseCompressionGainDb(const config::hardware::RirTransmitterConfig& tx);

  /**
   * @brief 接收机热噪声功率底: N₀ = k·T₀·B·F。
   * T₀ = 290K (IEEE 标准参考温度)。
   * @param tx 发射机参数（取带宽）
   * @param rx 接收机参数（取噪声系数）
   * @return 热噪声功率 (W)
   */
  static float ComputeThermalNoisePower_W(const config::hardware::RirTransmitterConfig& tx,
                                          const config::hardware::RirReceiverConfig& rx);

  /**
   * @brief 测距标准差 σ_R (m)。
   * 工程近似: σ_R ≈ 0.5·δ_R / √(SNR_linear)（纯随机项；
   * 2026-08-30 拆分后系统偏差经 kRangeMeasurementBiasM 施加在均值侧），
   * 其中 δ_R = c/(2B) 为距离分辨力。
   * @param snr_db    信噪比 (dB)
   * @param bandwidth_hz 信号带宽 (Hz)
   * @return 距离测量标准差 (m)
   */
  static float ComputeRangeErrorStdDev(float snr_db, float bandwidth_hz);

  /**
   * @brief 测角标准差 σ_θ (rad)。
   * 工程近似: σ_θ ≈ 0.317·θ_bw / √(SNR_linear)（纯随机项；角偏 θ_bw/30
   * 已拆至 ComputeAngleMeasurementBiasRad 均值侧）。
   * @param snr_db         信噪比 (dB)
   * @param beamwidth_rad  波束宽度 (rad)
   * @return 角度测量标准差 (rad)
   */
  static float ComputeAngleErrorStdDev(float snr_db, float beamwidth_rad);

  /**
   * @brief 支持 Swerling 0~4 全模型的多脉冲检测概率计算。
   * @param snr_db      每脉冲信噪比 (dB)
   * @param pfa         虚警概率
   * @param model       Swerling 起伏模型
   * @param num_pulses  检测脉冲数 N (N ≥ 1)
   * @return 检测概率 Pd ∈ [0, 1]
   */
  static float ComputeDetectionProbability(float snr_db, float pfa, RirSwerlingModel model,
                                           int num_pulses);

  /**
   * @brief 计算方波检测器的检测门限 T。
   * @param pfa         虚警概率
   * @param num_pulses  积累脉冲数 (N ≥ 1)
   * @return 检测门限 T
   */
  static double ComputeThreshold(double pfa, int num_pulses);

  /**
   * @brief 广义 Marcum Q 函数 Q_M(a, b)。
   * @param order  阶数 M (= 积累脉冲数 N)
   * @param a      √(2·N·SNR_linear)
   * @param b      √(2·T)
   * @return Q_M(a, b) ∈ [0, 1]
   */
  static double MarcumQ(int order, double a, double b);

  /**
   * @brief 蒙特卡洛探测判决。
   * @param detection_prob 检测概率 Pd
   * @param rng            随机数引擎（外部注入保证确定性）
   * @return 是否探测成功
   */
  static bool ThresholdDecision(float detection_prob, std::mt19937& rng);
};

}  // namespace internal
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_RADAR_EQUATIONS_H_
