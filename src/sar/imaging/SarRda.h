/**
 * @file SarRda.h
 * @brief SAR 内部 L1 条带 RDA 聚焦工具。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_RDA_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_RDA_H_

#include <cstddef>
#include <string>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief 距离徙动校正（RCMC）插值方法。
 */
enum class RcmcInterpolation {
  kNone = 0,   /**< 不做插值 */
  kLinear = 1, /**< 线性插值 */
  kSinc = 2,    /**< sinc 插值 */
};

/**
 * @brief RDA 聚焦配置。
 */
struct RdaConfig {
  double sample_rate_hz{0.0};        /**< 采样率（Hz） */
  double carrier_frequency_hz{0.0};  /**< 载频（Hz） */
  double prf_hz{0.0};               /**< 脉冲重复频率 PRF（Hz） */
  double platform_velocity_mps{0.0}; /**< 平台速度（m/s） */
  double reference_range_m{0.0};     /**< 参考斜距（m） */
  RcmcInterpolation rcmc_interpolation{RcmcInterpolation::kLinear}; /**< RCMC 插值方法 */
  std::size_t sinc_half_width{4U};   /**< sinc 插值半宽（kSinc 时生效） */
};

/**
 * @brief RDA 聚焦诊断信息。
 */
struct RdaDiagnostics {
  double reference_range_m{0.0};                          /**< 参考斜距（m） */
  double doppler_rate_hz_per_s{0.0};                      /**< 多普勒调频率（Hz/s） */
  double range_bin_spacing_m{0.0};                        /**< 距离 bin 间距（m） */
  double azimuth_sample_spacing_m{0.0};                   /**< 方位采样间距（m） */
  double azimuth_phase_curvature_rad_per_pulse2{0.0};     /**< 方位相位曲率（rad/pulse²） */
  double azimuth_quadratic_phase_span_rad{0.0};           /**< 方位二次相位跨度（rad） */
  double max_geometric_doppler_hz{0.0};                   /**< 最大几何多普勒（Hz） */
  double doppler_nyquist_margin{0.0};                     /**< 多普勒 Nyquist 余量 */
  double range_width_3db_bins{0.0};                       /**< 距离向 3dB 主瓣宽度（bin） */
  double azimuth_width_3db_bins{0.0};                     /**< 方位向 3dB 主瓣宽度（bin） */
  bool resolution_m_valid{false};                         /**< 物理分辨率是否有效 */
  double range_resolution_3db_m{0.0};                     /**< 距离向 3dB 分辨率（m） */
  double azimuth_resolution_3db_m{0.0};                   /**< 方位向 3dB 分辨率（m） */
  double image_entropy_nats{0.0};                         /**< 图像熵（nats） */
  double image_contrast{0.0};                             /**< 图像对比度 */
  std::size_t out_of_bounds_samples{0U};                  /**< 越界样本数 */
  std::string rcmc_interpolation = "none";                 /**< RCMC 插值方式描述 */
  std::string phase_reference_mode = "native";             /**< 相位参考模式 */
  std::string image_quality_mainlobe_method = "3db";       /**< 图像质量主瓣估计方法 */
  bool phase_reference_applied{false};                    /**< 是否施加了相位参考 */
  bool range_compression_applied{false};                  /**< 是否执行了距离压缩 */
  bool azimuth_fft_applied{false};                        /**< 是否执行了方位 FFT */
  bool azimuth_matched_filter_applied{false};             /**< 是否施加了方位匹配滤波 */
  bool azimuth_ifft_applied{false};                       /**< 是否执行了方位 IFFT */
};

/**
 * @brief RDA 聚焦输出（复图像 + 诊断）。
 */
struct FocusedSarImage {
  signal::ComplexMatrix image{}; /**< 聚焦复图像 */
  RdaDiagnostics diagnostics{};  /**< 聚焦诊断 */
};

/**
 * @brief L1 条带 RDA 聚焦（距离压缩 + RCMC + 方位压缩）。
 * @param[in] config RDA 配置。
 * @param[in] raw_pulse_history 原始相位历史矩阵。
 * @param[in] matched_filter 匹配滤波器系数。
 * @param[out] output 聚焦输出。
 * @return 成功返回 true，失败返回 false。
 */
bool FocusStripmapRda(const RdaConfig& config, const signal::ComplexMatrix& raw_pulse_history,
                      const signal::ComplexVector& matched_filter, FocusedSarImage* output);

/**
 * @brief 仅计算 RDA 采样诊断（不执行聚焦）。
 * @param[in] config RDA 配置。
 * @param[in] pulse_count 脉冲数。
 * @param[out] diagnostics 采样诊断。
 * @return 成功返回 true，失败返回 false。
 */
bool ComputeRdaSamplingDiagnostics(const RdaConfig& config, std::size_t pulse_count,
                                   RdaDiagnostics* diagnostics);

/**
 * @brief 对输入矩阵逐行做距离徙动校正（RCMC）。
 * @param[in] input 输入矩阵。
 * @param[in] delta_bins_by_row 各行距离徙动量（bin）。
 * @param[in] interpolation 插值方法。
 * @param[in] sinc_half_width sinc 插值半宽（kSinc 时生效）。
 * @param[out] output 校正后矩阵。
 * @param[out] out_of_bounds_samples 越界样本数。
 * @return 成功返回 true，失败返回 false。
 */
bool ApplyRangeMigrationCorrection(const signal::ComplexMatrix& input,
                                   const std::vector<double>& delta_bins_by_row,
                                   RcmcInterpolation interpolation, std::size_t sinc_half_width,
                                   signal::ComplexMatrix* output,
                                   std::size_t* out_of_bounds_samples);

/**
 * @brief 查找复图像幅度峰值所在的行主序线性索引。
 * @param[in] image 复图像。
 * @return 峰值线性索引。
 */
std::size_t FindPeakIndex(const signal::ComplexMatrix& image);
/**
 * @brief 估计复图像方位向 3dB 主瓣宽度（bin）。
 * @param[in] image 复图像。
 * @return 方位向 3dB 宽度（bin）。
 */
double EstimateAzimuthWidth3dbBins(const signal::ComplexMatrix& image);
/**
 * @brief 估计复图像熵（nats）。
 * @param[in] image 复图像。
 * @return 图像熵（nats）。
 */
double EstimateImageEntropyNats(const signal::ComplexMatrix& image);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_RDA_H_
