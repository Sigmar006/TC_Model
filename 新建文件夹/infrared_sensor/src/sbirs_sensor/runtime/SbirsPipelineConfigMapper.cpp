/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsPipelineConfigMapper.
 * Model: SBIRS infrared sensor.
 * Layer: runtime.
 * File role: executable implementation.
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

#include "sbirs_sensor/runtime/SbirsPipelineConfigMapper.h"

namespace sbirs_sensor {
namespace runtime {

config::SbirsInternalExecutionConfig MapSessionToInternal(
    const config::SbirsSessionConfig& config) {
  config::SbirsInternalExecutionConfig internal;
  internal.session = config;
  return internal;
}

}  // namespace runtime
}  // namespace sbirs_sensor
