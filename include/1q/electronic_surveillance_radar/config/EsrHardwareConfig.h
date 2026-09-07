/**
 * @file EsrHardwareConfig.h
 * @brief 定义 ESR 装备固有参数配置结构。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_HARDWARE_CONFIG_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_HARDWARE_CONFIG_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/electromagnetics/RfScene.h"

namespace electronic_surveillance_radar {
namespace config {

/** @brief EsrTuningWindow 描述可回放的接收调谐驻留窗口。 */
struct ONEQ_API EsrTuningWindow {
  double center_frequency_hz{0.0}; /**< 调谐中心频率（单位：Hz）。 */
  double bandwidth_hz{0.0};        /**< 调谐带宽（单位：Hz）。 */
  std::uint32_t dwell_cycles{1U};  /**< 连续驻留的成功周期数。 */
};

/** @brief ESR 接收设备的有向同平台隔离路径。 */
struct ONEQ_API EsrCoSiteIsolationPath {
  std::uint32_t transmitter_sensor_entity_id{0U};
  double isolation_db{0.0};
};

/**
 * @brief EsrHardwareConfig 描述 ESR 装备固有参数。
 */
struct ONEQ_API EsrHardwareConfig {
  double receiver_band_lower_hz{0.23e9};   /**< 接收频段下限（单位：Hz） */
  double receiver_band_upper_hz{100.0e9};  /**< 接收频段上限（单位：Hz） */
  float receiver_sensitivity_w{1.0e-12f}; /**< 接收机等效噪声基底（单位：W）；设为 0 时由
      receiver_noise_figure_db 和 receiver_reference_temperature_k 物理计算 */
  float receiver_noise_figure_db{5.0f}; /**< 接收机噪声系数（单位：dB）。 */
  float receiver_reference_temperature_k{290.0f}; /**< 噪声参考温度（单位：K）。 */
  float integrated_receive_loss_db{0.0f};  /**< 系统综合接收损耗（单位：dB） */
  float beam_az_width_deg{5.0f};           /**< 方位波束宽度（单位：deg） */
  float beam_el_width_deg{5.0f};           /**< 俯仰波束宽度（单位：deg） */
  float az_scan_range_deg{120.0f};         /**< 方位扫描范围（单位：deg） */
  float el_scan_range_deg{20.0f};          /**< 俯仰扫描范围（单位：deg） */
  float antenna_peak_gain_dbi{0.0f};       /**< 接收天线峰值增益（单位：dBi）。 */
  float antenna_sidelobe_level_db{-30.0f}; /**< 旁瓣相对峰值电平（单位：dB）。 */
  float antenna_backlobe_level_db{-40.0f}; /**< 后瓣相对峰值电平（单位：dB）。 */
  oneq::electromagnetics::RfScenePolarization polarization{
      oneq::electromagnetics::RfScenePolarization::kUnpolarized}; /**< 接收极化。 */
  float cross_polarization_isolation_db{20.0f};              /**< 交叉极化隔离（单位：dB）。 */
  float minimum_far_field_range_m{1.0f};                     /**< 远场公式最小距离（单位：m）。 */
  std::vector<EsrCoSiteIsolationPath> co_site_paths{}; /**< RF v2 有向同平台隔离路径。 */
  float maximum_linear_input_power_w{1.0e-3f};               /**< 最大线性输入功率（单位：W）。 */
  std::vector<EsrTuningWindow> tuning_plan{}; /**< 显式调谐计划；空列表表示全硬件频段驻留。 */
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_HARDWARE_CONFIG_H_
