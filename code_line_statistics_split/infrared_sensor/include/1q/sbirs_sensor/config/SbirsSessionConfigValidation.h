/**
 * @file SbirsSessionConfigValidation.h
 * @brief 定义 SBIRS-inspired 配置校验入口。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_VALIDATION_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"

namespace sbirs_sensor {
namespace config {

/**
 * @brief 校验 `SbirsSessionConfig` 各域取值合理性（波段正序、孔径/FOV/帧率为正、
 *        距离门控有序非负、扫描速率非负、检测门限非负）。
 *
 * 采用统一问题列表模型（session_contract.md 规则 14）：每条问题为 `session::SbirsIssue`，
 * severity 固定为 `kError`，phase 固定为 `kInputValidation`，code 为
 * `"sbirs.validation.<snake_case>"` 字符串，供机器消费；location/field 保持默认（无定位）。
 *
 * @param[in] config 待校验的会话配置
 * @return 校验问题列表；为空表示配置通过校验
 * @note 该函数为纯校验，不修改配置，不抛异常。
 */
ONEQ_API session::SbirsIssueList ValidateSbirsSessionConfig(const SbirsSessionConfig& config);

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_VALIDATION_H_
