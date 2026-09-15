/**
 * @file SbirsSessionCompositionRoot.h
 * @brief SBIRS-inspired session 默认依赖装配。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_SESSION_SBIRS_SESSION_COMPOSITION_ROOT_H_
#define ONEQ_SRC_SBIRS_SENSOR_SESSION_SBIRS_SESSION_COMPOSITION_ROOT_H_

#include <memory>

#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "sbirs_sensor/runtime/SbirsController.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief 装配并返回默认依赖图的控制器实例。
 * @param[in] config 会话配置
 * @return 装配好的 `SbirsController`
 */
std::unique_ptr<runtime::SbirsController> CreateSbirsController(
    const config::SbirsSessionConfig& config);

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_SESSION_SBIRS_SESSION_COMPOSITION_ROOT_H_
