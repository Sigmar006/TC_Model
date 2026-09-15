/**
 * @file SbirsNoiseModel.h
 * @brief SBIRS-inspired 红外噪声分解（design 2.8；口径修订 2026-09-02）。
 *
 * 噪声分母统一"积分时间内噪声能量"口径，供 SNR = P_sig·t_int / N_eff 使用：
 *   N_total = sqrt((NEP·t)² + N_photon² + N_thermal² + (N_readout·t)²)
 * 其中：
 *   - 探测器 NEP：noise_equivalent_power_w 直接计入合成（而非仅回退项）。
 *   - 光子噪声：√(P_bg·t·E_ph)，P_bg=背景亮度×孔径×光学透过率×像元视场
 *     （Ω=(像元间距/焦距)²），E_ph=hc/λ_center（波段中心单色近似）。
 *   - 热噪声：显式选配——仅探测器温度 > 0 计入（默认 0 关闭），√(4 k_B T t)。
 *   - 读出噪声：readout_noise_rms_w × t_int。
 * 全部分量退化（NEP=0 且三项全关）时回退 NEP 标量下限 1e-18。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_NOISE_MODEL_H_
#define ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_NOISE_MODEL_H_

#include "1q/sbirs_sensor/config/SbirsHardwareConfig.h"

namespace sbirs_sensor {
namespace foundation {

/**
 * @brief 噪声分量统计（design 2.8 ComputeBackgroundNoiseStatistics）。
 * @note 内部数据结构。各分量与合成总噪声均为"积分时间内噪声能量"口径
 *       （SNR=P_sig·t_int/N_eff 的分母），字段名沿用 *_w。
 */
struct SbirsNoiseStatistics {
  double photon_noise_w{0.0};    /**< 光子（散粒）噪声分量，能量分母口径 */
  double thermal_noise_w{0.0};   /**< 热噪声分量（仅温度>0 时非零），能量分母口径 */
  double readout_noise_w{0.0};   /**< 读出噪声分量（RMS×t_int），能量分母口径 */
  double total_noise_w{0.0};     /**< NEP+光子+热+读出 RSS 合成，能量分母口径 */
};

/**
 * @brief 计算背景噪声统计。
 *
 * 光子噪声由背景亮度、孔径、光学透过率、像元视场与波段中心光子能量按散粒统计
 * 合成；探测器 NEP 直接计入 RSS；热噪声仅温度>0 计入；读出噪声按积分时间折算。
 * @param[in] hardware 硬件配置
 * @return 噪声分量统计
 */
SbirsNoiseStatistics ComputeBackgroundNoiseStatistics(const config::SbirsHardwareConfig& hardware);

/**
 * @brief 得到有效噪声分母（能量分母口径，SNR=P_sig·t_int/N_eff）。
 *
 * total_noise_w > 0 时直接返回；全部分量退化时回退 max(1e-18, NEP) 保证分母恒正。
 * @param[in] hardware 硬件配置（取 NEP 作回退）
 * @param[in] statistics 噪声分量统计
 * @return 有效噪声分母
 */
double ResolveEffectiveNoiseW(const config::SbirsHardwareConfig& hardware,
                              const SbirsNoiseStatistics& statistics);

}  // namespace foundation
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_NOISE_MODEL_H_
