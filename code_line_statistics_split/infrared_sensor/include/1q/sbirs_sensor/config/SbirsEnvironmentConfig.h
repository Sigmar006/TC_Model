/**
 * @file SbirsEnvironmentConfig.h
 * @brief 定义 SBIRS-inspired 环境和气象衰减参数。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_ENVIRONMENT_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_ENVIRONMENT_CONFIG_H_

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace config {

/** @brief 气象类型枚举，对应气象衰减模型中的天气类别查表项。 */
enum class ONEQ_API SbirsWeatherType { kClear = 0, kCloudy, kRain, kFog };

/** @brief 海浪等级枚举，对应气象衰减模型中的海况查表项。 */
enum class ONEQ_API SbirsSeaState { kLow = 0, kMedium, kHigh };

/**
 * @brief SBIRS-inspired 环境与气象衰减配置。
 * @note 纯数据类型 (POD)。各字段参与气象衰减模型：独立衰减项加权叠加得到 `A_total`，
 *       作用于路径透过率后进入 IR SNR 链路。
 */
struct ONEQ_API SbirsEnvironmentConfig {
  SbirsWeatherType weather_type{SbirsWeatherType::kClear}; /**< 天气类型，默认晴 */
  SbirsSeaState sea_state{SbirsSeaState::kLow};            /**< 海浪等级，默认低 */
  float temperature_c{15.0f};                              /**< 环境温度，单位 ℃ */
  float relative_humidity_percent{50.0f};                  /**< 相对湿度（%） */
  float visibility_km{20.0f};                              /**< 能见度，单位 km */
  float base_atmospheric_transmittance{0.8f};              /**< 基础大气透过率 τ(λ,d)，无量纲 */
  // 2.9 气象交互项权重：A_total 加权叠加中的交互项系数 k_j（默认 0 = 无交互项，
  // 保持向后兼容）。单位：无量纲。当前仅 humidity×visibility 与 rain×humidity 两项。
  float humidity_visibility_interaction_weight{0.0f}; /**< 湿度×能见度交互项系数 k_j（默认 0 关闭） */
  float rain_humidity_interaction_weight{0.0f};       /**< 雨×湿度交互项系数 k_j（仅雨天生效，默认 0 关闭） */
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_ENVIRONMENT_CONFIG_H_
