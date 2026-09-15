/**
 * @file SbirsRuntimeConfigPatch.h
 * @brief 定义 SBIRS-inspired 运行期配置补丁。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_RUNTIME_CONFIG_PATCH_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_RUNTIME_CONFIG_PATCH_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/config/SbirsEnvironmentConfig.h"
#include "1q/sbirs_sensor/config/SbirsMissionConfig.h"
#include "1q/sbirs_sensor/config/SbirsPolicyConfig.h"

namespace sbirs_sensor {
namespace config {

/**
 * @brief SBIRS-inspired 运行期配置补丁，描述会话运行期可立即提交的有限参数变更。
 * @note 各 `has_*` 布尔位标记对应字段是否被设置；未设置的字段不覆盖现有配置。
 *       补丁由 `SbirsSession::TryApplyRuntimeConfig` 提交并立即生效，不在 session 层回滚。
 *       电源状态仅由叶子 `has_sensor_enabled` 控制（COMMON-OQ-4 收敛）：
 *       `has_mission` 不改变电源，mission 域在类型层面已无电源字段（字段提升）。
 */
struct ONEQ_API SbirsRuntimeConfigPatch {
  bool has_mission{false};          /**< 是否覆盖整域 mission 配置 */
  SbirsMissionConfig mission{};     /**< mission 配置覆盖值（仅当 has_mission 为真生效） */

  bool has_policy{false};           /**< 是否覆盖整域 policy 配置 */
  SbirsPolicyConfig policy{};       /**< policy 配置覆盖值（仅当 has_policy 为真生效） */

  bool has_environment{false};      /**< 是否覆盖整域 environment 配置 */
  SbirsEnvironmentConfig environment{}; /**< environment 配置覆盖值（仅当 has_environment 为真生效） */

  bool has_work_mode{false};        /**< 是否覆盖工作模式 */
  SbirsWorkMode work_mode{SbirsWorkMode::kSearchAndStare}; /**< 工作模式覆盖值 */

  bool has_scan_rate_deg_per_sec{false}; /**< 是否覆盖扫描速率 */
  float scan_rate_deg_per_sec{10.0f};    /**< 扫描速率覆盖值，单位 deg/s */

  bool has_sensor_enabled{false};   /**< 是否覆盖传感器电源状态 */
  bool sensor_enabled{true};        /**< 传感器电源状态覆盖值；字段名沿用跨模块约定 */
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_RUNTIME_CONFIG_PATCH_H_
