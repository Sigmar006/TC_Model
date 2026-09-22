/*
 * Supplemental architecture notes for acceptance review.
 * Unit: RirSessionConfig.
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
 * @file RirSessionConfig.h
 * @brief 远程识别雷达会话初始化主配置类型。
 *
 * 会话初始化配置（五域公开模型）的主头文件。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_SESSION_CONFIG_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_SESSION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/remote_identification_radar/config/RirEnvironmentConfig.h"
#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "1q/remote_identification_radar/config/RirMissionConfig.h"
#include "1q/remote_identification_radar/config/RirOrientationConfig.h"
#include "1q/remote_identification_radar/config/RirPolicyConfig.h"

namespace remote_identification_radar {
namespace config {

/**
 * @brief RirSessionConfig 会话初始化配置（五域公开模型）。
 *
 * 电源状态由顶层 `sensor_enabled` 唯一承载（COMMON-OQ-4 字段提升）：
 * `mission` 域不含电源字段，运行时补丁的电源入口为
 * `RirRuntimeConfigPatch::has_sensor_enabled`。
 */
struct ONEQ_API RirSessionConfig {
  RirHardwareConfig hardware{};
  RirOrientationConfig orientation{};
  RirMissionConfig mission{};
  RirPolicyConfig policy{};
  RirEnvironmentConfig environment{};
  bool sensor_enabled{true}; /**< 传感器初始电源状态 */
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_SESSION_CONFIG_H_
