/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsPipelineConfigMapper.
 * Model: SBIRS infrared sensor.
 * Layer: runtime.
 * File role: interface declarations and data contracts.
 * Responsibility: runtime configuration resolution and controller orchestration.
 * Upstream boundary: validated session configuration and staged runtime patches.
 * Downstream boundary: the executable SBIRS pipeline and cycle result assembly.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

/**
 * @file SbirsPipelineConfigMapper.h
 * @brief SBIRS-inspired public config 到 internal config 映射。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_PIPELINE_CONFIG_MAPPER_H_
#define ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_PIPELINE_CONFIG_MAPPER_H_

#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "sbirs_sensor/config/SbirsInternalExecutionConfig.h"

namespace sbirs_sensor {
namespace runtime {

/**
 * @brief 将 public 会话配置映射为 pipeline 内部执行配置。
 * @param[in] config public 会话配置
 * @return 内部执行配置
 */
config::SbirsInternalExecutionConfig MapSessionToInternal(const config::SbirsSessionConfig& config);

}  // namespace runtime
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_PIPELINE_CONFIG_MAPPER_H_
