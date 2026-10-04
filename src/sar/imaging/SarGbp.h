/**
 * @file SarGbp.h
 * @brief SAR 内部小场景全局后向投影参考聚焦工具。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_GBP_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_GBP_H_

#include <cstddef>
#include <string>
#include <vector>

#include "sar/geometry/SarGeometry.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief GBP/BP 成像网格配置。
 */
struct GbpGridConfig {
  std::size_t azimuth_pixel_count{0U}; /**< 方位像素数 */
  std::size_t range_pixel_count{0U};   /**< 距离像素数 */
  double azimuth_start_m{0.0};         /**< 方位起始坐标（m） */
  double range_start_m{0.0};           /**< 距离起始坐标（m） */
  double azimuth_spacing_m{0.0};       /**< 方位像素间距（m） */
  double range_spacing_m{0.0};         /**< 距离像素间距（m） */
  double image_plane_z_m{0.0};         /**< 成像平面高度 z（m） */
};

/**
 * @brief GBP/BP 聚焦配置。
 */
struct GbpConfig {
  double sample_rate_hz{0.0};        /**< 采样率（Hz） */
  double carrier_frequency_hz{0.0};  /**< 载频（Hz） */
  GbpGridConfig grid{};              /**< 成像网格配置 */
};

/**
 * @brief GBP/BP 聚焦诊断信息。
 */
struct GbpDiagnostics {
  std::size_t evaluated_pixels{0U};       /**< 已累加像素数 */
  std::size_t accumulated_samples{0U};    /**< 累加样本数 */
  std::size_t out_of_bounds_samples{0U};  /**< 越界样本数 */
  std::size_t max_approved_dimension{128U}; /**< 批准的最大维度上限 */
  std::string range_interpolation = "linear"; /**< 距离插值方式 */
  std::string traversal_order = "pixel_major"; /**< 遍历顺序 */
};

/**
 * @brief GBP/BP 聚焦输出（复图像 + 诊断）。
 */
struct FocusedGbpImage {
  signal::ComplexMatrix image{}; /**< 聚焦复图像 */
  GbpDiagnostics diagnostics{};  /**< 聚焦诊断 */
};

/**
 * @brief 小场景全局后向投影（GBP）参考聚焦。
 * @param[in] config 聚焦配置。
 * @param[in] pulses 平台脉冲状态序列。
 * @param[in] raw_pulse_history 原始相位历史矩阵。
 * @param[in] matched_filter 匹配滤波器系数。
 * @param[out] output 聚焦输出。
 * @return 成功返回 true，失败返回 false。
 */
bool FocusSmallSceneGbp(const GbpConfig& config,
                        const std::vector<geometry::PlatformPulseState>& pulses,
                        const signal::ComplexMatrix& raw_pulse_history,
                        const signal::ComplexVector& matched_filter, FocusedGbpImage* output);

/**
 * @brief 逐脉冲后向投影（BP）聚焦。
 * @param[in] config 聚焦配置。
 * @param[in] pulses 平台脉冲状态序列。
 * @param[in] raw_pulse_history 原始相位历史矩阵。
 * @param[in] matched_filter 匹配滤波器系数。
 * @param[out] output 聚焦输出。
 * @return 成功返回 true，失败返回 false。
 */
bool FocusSmallSceneBp(const GbpConfig& config,
                       const std::vector<geometry::PlatformPulseState>& pulses,
                       const signal::ComplexMatrix& raw_pulse_history,
                       const signal::ComplexVector& matched_filter, FocusedGbpImage* output);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_GBP_H_
