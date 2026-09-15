/**
 * @file SbirsRuntimeConfigResolver.h
 * @brief SBIRS-inspired runtime patch resolver。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_RUNTIME_CONFIG_RESOLVER_H_
#define ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_RUNTIME_CONFIG_RESOLVER_H_

#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "sbirs_sensor/runtime/SbirsRuntimeConfigImpact.h"

namespace sbirs_sensor {
namespace runtime {

/**
 * @brief runtime patch 解析结果。
 * @note `is_valid` 表示 patch 是否合法；`has_requested_update` 表示是否产生实际更新；
 *       仅当两者均为真时调用方才应采用 `resolved_config`。
 */
struct SbirsRuntimeConfigResolution {
  bool is_valid{false};                        /**< patch 是否合法 */
  bool has_requested_update{false};            /**< 是否产生实际更新 */
  config::SbirsSessionConfig resolved_config{}; /**< 解析后的会话配置 */
  SbirsRuntimeConfigImpact impact{};            /**< pipeline 最小状态迁移影响 */
};

/**
 * @brief 校验 runtime patch 并在当前配置上叠加得到解析后的配置。
 * @param[in] current_config 当前会话配置
 * @param[in] patch 运行期配置补丁
 * @return 解析结果（含合法性、是否更新与解析后配置）
 * @note 该函数为纯计算，不修改输入；立即生效由调用方负责。
 */
SbirsRuntimeConfigResolution ResolveSbirsRuntimeConfigPatch(
    const config::SbirsSessionConfig& current_config, const config::SbirsRuntimeConfigPatch& patch);

}  // namespace runtime
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_RUNTIME_CONFIG_RESOLVER_H_
