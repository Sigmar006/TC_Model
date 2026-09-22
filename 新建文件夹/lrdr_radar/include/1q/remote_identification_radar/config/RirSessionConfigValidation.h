/*
 * Supplemental architecture notes for acceptance review.
 * Unit: RirSessionConfigValidation.
 * Model: LRDR remote identification radar.
 * Layer: config.
 * File role: interface declarations and data contracts.
 * Responsibility: validated radar hardware, mission, environment, and policy contracts.
 * Upstream boundary: external initialization values and staged runtime settings.
 * Downstream boundary: session, RF, detection, tracking, and recognition configuration.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

/**
 * @file RirSessionConfigValidation.h
 * @brief 远程识别雷达会话配置校验入口。
 *
 * 配置静态校验的主头文件。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_SESSION_CONFIG_VALIDATION_H_

#include "1q/api.hpp"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirOutputTypes.h"

namespace remote_identification_radar {
namespace config {

/**
 * @brief 校验会话配置合法性。
 * @param[in] config 四域会话配置。
 * @return 按发现顺序返回的问题条目列表（所有条目 phase=kInputValidation，
 *         code 形如 "rir.validation.<snake_case>"）。
 */
ONEQ_API session::RirIssueList ValidateRirSessionConfig(const RirSessionConfig& config);

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_SESSION_CONFIG_VALIDATION_H_
