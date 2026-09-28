/**
 * @file RadarEquations.h
 * @brief 定义雷达探测通用物理方程纯函数接口（common 单源，标量参数）。
 */

#ifndef COMMON_RADAR_RADAR_EQUATIONS_H_
#define COMMON_RADAR_RADAR_EQUATIONS_H_

#include <random>

namespace oneq {
namespace common {
namespace radar {

/**
 * @brief 距离量测固定系统偏置（未标定残差），单位：米。
 * @note 口径：固定系统偏置应施加在量测均值侧，而非并入随机 std；2026-08-30 从
 *       ComputeRangeErrorStdDev 的标准差中拆出（此前被当作随机抖动加进 std，物理
 *       不正确）。来源口径不变：工程实测数据拟合，含系统偏置与量化误差等固定分量。
 */
constexpr float kRangeMeasurementBiasM = 20.0f;

/**
 * @brief 角度量测固定系统偏置（未标定残差），单位：弧度。
 * @param beamwidth_rad 波束宽度（弧度）。
 * @return 波束宽度的 1/30（随波束宽度变化；单脉冲测角工程经验口径）。
 * @note 口径：固定系统偏置应施加在量测均值侧，而非并入随机 std；2026-08-30 从
 *       ComputeAngleErrorStdDev 的标准差中拆出。
 */
inline float ComputeAngleMeasurementBiasRad(float beamwidth_rad) {
  return beamwidth_rad / 30.0f;
}

/**
 * @brief 目标雷达散射截面起伏统计模型（通用枚举）。
 */
enum class SwerlingModel {
  kSwerling0 = 0, /**< 不起伏——目标 RCS 在各次观测中恒定。 */
  kSwerling1 = 1, /**< 扫描间慢起伏，单次扫描内 RCS 恒定。 */
  kSwerling2 = 2, /**< 脉冲间快起伏，每个脉冲 RCS 独立采样。 */
  kSwerling3 = 3, /**< 扫描间慢起伏，RCS 服从 2 自由度 chi-squared 分布。 */
  kSwerling4 = 4  /**< 脉冲间快起伏，RCS 服从 2 自由度 chi-squared 分布。 */
};

/**
 * @brief 无状态的雷达物理计算函数集合。
 * 所有方法为 static，不持有任何内部状态；参数全部使用标量，不依赖模块 config。
 */
struct RadarEquations {
  /**
   * @brief 单站雷达方程（对数域），按指定单程天线增益计算接收回波功率。
   * 公式: Pr_dBW = Pt_dB + 2*G_one_way_dB + 2*λ_dB + σ_dB
   *                - 30*log10(4π) - 4*R_dB - L_sys
   * 其中 Pt 项按峰值功率与脉宽的单脉冲能量语义进行缩放。
   * @param peak_power_w 发射峰值功率 (W)
   * @param transmit_loss_db 发射系统损耗 (dB)
   * @param pulse_width_s 发射脉宽 (s)
   * @param frequency_hz 发射频率 (Hz)
   * @param one_way_gain_db 当前波束指向下的单程天线增益 (dB)
   * @param rcs_m2 目标雷达散射截面 (m²)
   * @param range_m 目标斜距 (m)
   * @param propagation_loss_db 大气传播往返损耗 (dB)
   * @return 接收回波功率 (dBW)
   */
  static float ComputeEchoPowerWithGain_dBW(float peak_power_w, float transmit_loss_db,
                                            float pulse_width_s, float frequency_hz,
                                            float one_way_gain_db, float rcs_m2, float range_m,
                                            float propagation_loss_db);

  /**
   * @brief 单站雷达方程（对数域），以天线主瓣峰值增益计算接收回波功率。
   * @param peak_power_w 发射峰值功率 (W)
   * @param transmit_loss_db 发射系统损耗 (dB)
   * @param pulse_width_s 发射脉宽 (s)
   * @param frequency_hz 发射频率 (Hz)
   * @param main_beam_gain_db 天线主瓣峰值增益 (dB)
   * @param rcs_m2 目标雷达散射截面 (m²)
   * @param range_m 目标斜距 (m)
   * @param propagation_loss_db 大气传播往返损耗 (dB)
   * @return 接收回波功率 (dBW)
   */
  static float ComputeEchoPower_dBW(float peak_power_w, float transmit_loss_db,
                                    float pulse_width_s, float frequency_hz,
                                    float main_beam_gain_db, float rcs_m2, float range_m,
                                    float propagation_loss_db);

  /**
   * @brief 接收机热噪声功率底: N₀ = k·T₀·B·F。
   * T₀ = 290K (IEEE 标准参考温度)。
   * @param bandwidth_hz 接收带宽 (Hz)
   * @param noise_figure_db 接收机噪声系数 (dB)
   * @return 热噪声功率 (W)
   */
  static float ComputeThermalNoisePower_W(float bandwidth_hz, float noise_figure_db);

  /**
   * @brief 脉冲积累增益因子。
   * 当前统一采用线性脉冲积累语义: G = N。
   * @param pulse_count 积累脉冲数
   * @return 积累增益因子（线性值）
   */
  static float ComputeIntegrationGain(int pulse_count);

  /**
   * @brief 测距随机标准差 σ_R (m)。
   * 工程近似: σ_R ≈ 0.5·δ_R / √(SNR_linear)，
   * 其中 δ_R = c/(2B) 为距离分辨力。
   * 固定系统偏置（kRangeMeasurementBiasM）不并入本值，应由消费方施加在量测均值侧
   * （2026-08-30 从 std 中拆出）；snr_db < -10 dB 下限分支返回 1.5777·δ_R，
   * 同样只含随机项的既有形态。
   * @param snr_db    信噪比 (dB)
   * @param bandwidth_hz 信号带宽 (Hz)
   * @return 距离测量随机标准差（不含固定偏置）(m)
   */
  static float ComputeRangeErrorStdDev(float snr_db, float bandwidth_hz);

  /**
   * @brief 测角随机标准差 σ_θ (rad)。
   * 工程近似: σ_θ ≈ 0.317·θ_bw / √(SNR_linear)。
   * 固定系统偏置（θ_bw/30，见 ComputeAngleMeasurementBiasRad）不并入本值，
   * 应由消费方施加在量测均值侧（2026-08-30 从 std 中拆出）；snr_db < -10 dB
   * 下限分支返回 θ_bw，同样只含随机项的既有形态。
   * @param snr_db         信噪比 (dB)
   * @param beamwidth_rad  波束宽度 (rad)
   * @return 角度测量随机标准差（不含固定偏置）(rad)
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
  static float ComputeDetectionProbability(float snr_db, float pfa, SwerlingModel model,
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

}  // namespace radar
}  // namespace common
}  // namespace oneq

#endif  // COMMON_RADAR_RADAR_EQUATIONS_H_
