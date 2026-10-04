/**
 * @file SarOmegaKFocusing.h
 * @brief Omega-K stripmap 聚焦编排器:把 front-end + 全部 Omega-K 部件串联成完整聚焦入口。
 *
 * 数据流(契约 §4.2): raw history → front-end(2D FFT + H_bulk) → Stolt 几何 →
 * 共同支持 → 网格收缩 → 相对延迟变换 → 参考映射 → 参考相位补偿 → 方位逆变换 → 图像。
 * 仅 L1 匀速直线 broadside 条带(契约 §7)。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_FOCUSING_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_FOCUSING_H_

#include <string>

#include "sar/imaging/SarOmegaKAzimuthInverseTransform.h"
#include "sar/imaging/SarOmegaKCommonSupport.h"
#include "sar/imaging/SarOmegaKGeometry.h"
#include "sar/imaging/SarOmegaKGridReduction.h"
#include "sar/imaging/SarOmegaKReferenceMapping.h"
#include "sar/imaging/SarOmegaKReferencePhaseCompensation.h"
#include "sar/imaging/SarOmegaKRelativeDelayTransform.h"
#include "sar/imaging/SarOmegaKSpectrumFrontEnd.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief Omega-K 条带聚焦物理参数配置。
 */
struct OmegaKConfig {
  std::size_t range_sample_count{0U};   /**< 距离向采样点数 */
  std::size_t azimuth_pulse_count{0U};  /**< 方位向脉冲数 */
  double sample_rate_hz{0.0};           /**< 距离向采样率（Hz） */
  double prf_hz{0.0};                  /**< 脉冲重复频率 PRF（Hz） */
  double carrier_frequency_hz{0.0};     /**< 载频（Hz） */
  double platform_velocity_mps{0.0};    /**< 平台速度（m/s） */
  double reference_range_m{0.0};        /**< 参考斜距（m） */
};

/**
 * @brief Omega-K 聚焦各阶段诊断结果。
 */
struct OmegaKDiagnostics {
  OmegaKSpectrumFrontEndResult front_end{};        /**< front-end（2D FFT + H_bulk）诊断 */
  OmegaKCommonSupportDiagnostics common_support{}; /**< 共同支持阶段诊断 */
  OmegaKGridReductionResult grid_reduction{};      /**< 网格收缩阶段结果 */
  OmegaKRelativeDelayResult relative_delay{};      /**< 相对延迟变换结果 */
  OmegaKReferenceMappingResult reference_mapping{};/**< 参考映射结果 */
  OmegaKAzimuthInverseResult azimuth_inverse{};    /**< 方位逆变换结果 */
  std::string failure_stage = "none";               /**< 失败阶段标记，"none" 表示无失败 */
};

/**
 * @brief Omega-K 聚焦输出（复图像 + 诊断）。
 */
struct FocusedOmegaKImage {
  signal::ComplexMatrix image{};   /**< 聚焦复图像 */
  OmegaKDiagnostics diagnostics{}; /**< 各阶段诊断 */
};

/**
 * @brief Omega-K 条带 broadside 聚焦编排器。
 *
 * 把 front-end + 全部 Omega-K 部件串联成完整聚焦入口。仅支持 L1 匀速直线 broadside 条带。
 * @param[in] config Omega-K 物理参数配置。
 * @param[in] raw_pulse_history 原始相位历史矩阵（行=方位脉冲，列=距离采样）。
 * @param[out] output 聚焦输出（复图像 + 诊断）。
 * @return 聚焦成功返回 true；任一阶段失败或参数非法返回 false。
 */
bool FocusStripmapOmegaK(const OmegaKConfig& config,
                         const signal::ComplexMatrix& raw_pulse_history,
                         FocusedOmegaKImage* output);

/**
 * @brief 聚束(Spotlight)Omega-K 聚焦配置。
 *
 * 沿用全部 OmegaKConfig 物理参数。聚束的 Stolt 几何与条带相同(squint-invariant),
 * 仅方位坐标原点由 scene_center_azimuth_m 偏移。
 */
struct SpotlightOmegaKConfig : OmegaKConfig {
  double scene_center_azimuth_m{0.0};  /**< 聚束场景中心方位(方位坐标原点偏移) */
};

/**
 * @brief 聚束 Omega-K 聚焦编排器。
 *
 * 与 FocusStripmapOmegaK 共享全部 8 个 Omega-K 部件(front-end/几何/Stolt/参考映射/相位
 * 补偿/方位逆变换),因 Stolt 映射天然 squint-invariant。仅方位坐标原点用场景中心偏移。
 *
 * broadside 退化不变量:scene_center_azimuth_m=0 且 raw history 无天线调制时,
 * 输出与 FocusStripmapOmegaK 完全一致。
 */
bool FocusSpotlightOmegaK(const SpotlightOmegaKConfig& config,
                          const signal::ComplexMatrix& raw_pulse_history,
                          FocusedOmegaKImage* output);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_FOCUSING_H_
