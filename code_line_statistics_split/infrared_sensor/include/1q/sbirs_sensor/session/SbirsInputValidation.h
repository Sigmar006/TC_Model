/**
 * @file SbirsInputValidation.h
 * @brief 定义 SBIRS-inspired 输入校验类型与入口。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_INPUT_VALIDATION_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_INPUT_VALIDATION_H_

#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"

namespace sbirs_sensor {
namespace session {

struct SbirsCycleInput;

/**
 * @brief 校验单周期输入：步长正有限且在帧率合理范围内、非零 ECEF、目标 ID 唯一性、物理取值域、
 *        速度 flag/data 一致性及 environment override 的枚举与连续参数。
 * @param[in] input 待校验的单周期输入
 * @param[in] frame_rate_hz 传感器帧率（Hz），用于 dt_sec 上界校验；必须正有限
 * @return 问题条目列表（统一问题列表模型，规则 14；所有条目 phase=kInputValidation，
 *         code 形如 "sbirs.validation.<snake_case>"）；为空表示输入通过校验
 * @note 该函数为纯校验，不修改输入，不抛异常。
 */
ONEQ_API SbirsIssueList ValidateSbirsCycleInput(const SbirsCycleInput& input,
                                                float frame_rate_hz);
/**
 * @brief 判断问题列表中是否存在输入校验 error 级问题。
 * @param[in] issues 问题条目列表
 * @return 存在 phase==kInputValidation 且 severity==kError 的条目时返回 true，否则返回 false
 */
ONEQ_API bool HasValidationError(const SbirsIssueList& issues);

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_INPUT_VALIDATION_H_
