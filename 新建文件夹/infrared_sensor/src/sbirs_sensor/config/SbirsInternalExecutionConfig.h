/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsInternalExecutionConfig.
 * Model: SBIRS infrared sensor.
 * Layer: config.
 * File role: interface declarations and data contracts.
 * Responsibility: validated sensor configuration and policy contracts.
 * Upstream boundary: mission, hardware, orientation, environment, and runtime settings.
 * Downstream boundary: session construction and pipeline configuration.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

/**
 * @file SbirsInternalExecutionConfig.h
 * @brief SBIRS-inspired pipeline 内部执行配置。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_CONFIG_SBIRS_INTERNAL_EXECUTION_CONFIG_H_
#define ONEQ_SRC_SBIRS_SENSOR_CONFIG_SBIRS_INTERNAL_EXECUTION_CONFIG_H_

#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"

namespace sbirs_sensor {
namespace config {

/**
 * @brief pipeline 内部执行配置，由 session 配置映射得到。
 * @note 内部实现细节，不在 public header 中暴露；不构成模块间契约。
 */
struct SbirsInternalExecutionConfig {
  SbirsSessionConfig session{}; /**< 来源会话配置 */
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_CONFIG_SBIRS_INTERNAL_EXECUTION_CONFIG_H_
