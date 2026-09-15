/**
 * @file SbirsHardwareConfig.h
 * @brief 定义 SBIRS-inspired 传感器硬件参数。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_HARDWARE_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_HARDWARE_CONFIG_H_

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace config {

/**
 * @brief SBIRS-inspired 传感器硬件参数。
 * @note 纯数据类型 (POD)。波段、孔径等参数进入 foundation 物理链路（Planck 辐射、
 *       接收功率、噪声合成、IR SNR）。噪声合成：NEP 直接计入 RSS；光子噪声由背景
 *       亮度×像元视场驱动（默认开）；热项显式选配（温度>0 才计入）；全部分量退化时
 *       回退 NEP 标量下限（见 2.8 注释与 SbirsNoiseModel.h）。
 */
struct ONEQ_API SbirsHardwareConfig {
  float wavelength_lower_um{3.0f};   /**< 工作波段下限，单位 μm（默认 MWIR 下沿 3.0） */
  float wavelength_upper_um{5.0f};   /**< 工作波段上限，单位 μm（默认 MWIR 上沿 5.0） */
  float optical_aperture_m{0.5f};    /**< 光学孔径，单位 m */
  float optical_transmission{0.8f};  /**< 光学透过率，无量纲 */
  float detector_quantum_efficiency{0.7f}; /**< 探测器量子效率，无量纲 */
  float integration_time_sec{0.02f};       /**< 积分时间，单位 s */
  float noise_equivalent_power_w{1.0e-12f}; /**< 等效噪声功率 NEP，单位 W */
  // 2.8 噪声分解：NEP 直接计入 RSS 合成；光子噪声由背景亮度×像元视场驱动（默认开，
  // 地球 MWIR 典型 2.0，0=关闭）；热项显式选配（温度>0 才计入，默认 0 关闭）。
  float background_radiance_w_sr_m2{2.0f};   /**< 背景辐射亮度，用于光子噪声（W/(sr·m²)），默认地球 MWIR 典型值 */
  float detector_temperature_k{0.0f};        /**< 探测器工作温度，单位 K；>0 才计入热噪声（显式选配） */
  float readout_noise_rms_w{0.0f};            /**< 读出噪声 RMS，单位 W 等效 */
  // 焦平面几何（3.2.1.3.2.3 焦平面脱靶量映射）：仅验收日志消费，不进标量探测链路。
  float focal_length_m{2.0f};       /**< 光学焦距，单位 m（>0；NFOV 脱靶量 x = f·tan(Δaz)） */
  float detector_pixel_pitch_m{30.0e-6f}; /**< 探测器像元间距，单位 m（>0；脱靶量像素数 = 米/间距） */
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_HARDWARE_CONFIG_H_
