/**
 * @file RirInputValidation.h
 * @brief 远程识别雷达周期输入校验入口。
 *
 * 周期输入静态校验的主头文件。实现随会话组装落地（阶段 1 步骤 4）。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_INPUT_VALIDATION_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_INPUT_VALIDATION_H_

#include "1q/api.hpp"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirOutputTypes.h"

namespace remote_identification_radar {
namespace session {

/**
 * @brief 校验周期步长字段。
 * @param[in] dt_sec 当前周期步长（单位：秒）。
 * @return 问题条目列表（统一问题列表模型，规则 14；所有条目 phase=kInputValidation，
 *         code 形如 "rir.validation.<snake_case>"）。
 */
ONEQ_API RirIssueList ValidateRirCycleDeltaTime(double dt_sec);

/**
 * @brief 校验完整周期输入。
 * @param[in] input 当前周期输入。
 * @param[in] recognition_dwell_sec 会话识别驻留时长（s）；> 0 时校验 `rf_scene`
 *            窗口与 `sim_time_sec`/驻留对齐。
 * @return 按发现顺序返回的问题条目列表（所有条目 phase=kInputValidation）。
 */
ONEQ_API RirIssueList ValidateRirCycleInput(const RirCycleInput& input,
                                            float recognition_dwell_sec = 0.0f);

/**
 * @brief 校验场景目标列表。
 * @param[in] targets 当前周期场景目标列表。
 * @return 按发现顺序返回的问题条目列表（所有条目 phase=kInputValidation）。
 */
ONEQ_API RirIssueList ValidateRirSceneTargets(const RirSceneTargetList& targets);

/**
 * @brief 判断问题列表中是否存在输入校验 error 级问题。
 * @param[in] issues 问题条目列表。
 * @return 存在 phase==kInputValidation 且 severity==kError 的条目时返回 `true`。
 */
ONEQ_API bool HasValidationError(const RirIssueList& issues);

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_INPUT_VALIDATION_H_
