/**
 * @file SbirsSessionConfig.h
 * @brief 定义 SBIRS-inspired 会话初始化配置。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/config/SbirsEnvironmentConfig.h"
#include "1q/sbirs_sensor/config/SbirsHardwareConfig.h"
#include "1q/sbirs_sensor/config/SbirsMissionConfig.h"
#include "1q/sbirs_sensor/config/SbirsOrientationConfig.h"
#include "1q/sbirs_sensor/config/SbirsPolicyConfig.h"

namespace sbirs_sensor {
namespace config {

/**
 * @brief SBIRS-inspired 会话初始化配置，聚合硬件、任务、安装指向、策略、环境五域配置。
 * @note 纯数据类型 (POD)，是构造 `SbirsSession` 的主要输入。
 */
struct ONEQ_API SbirsSessionConfig {
  SbirsHardwareConfig hardware{};       /**< 传感器硬件参数 */
  SbirsMissionConfig mission{};         /**< 任务与视场参数 */
  SbirsOrientationConfig orientation{}; /**< 安装指向与稳定参数（静态） */
  SbirsPolicyConfig policy{};           /**< 检测/误差/调度策略 */
  SbirsEnvironmentConfig environment{}; /**< 环境与气象衰减参数 */
  bool sensor_enabled{true};            /**< 传感器初始电源状态（COMMON-OQ-4 字段提升） */
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_H_
