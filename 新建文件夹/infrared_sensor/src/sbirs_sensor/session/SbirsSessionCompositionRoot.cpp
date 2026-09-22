/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsSessionCompositionRoot.
 * Model: SBIRS infrared sensor.
 * Layer: session.
 * File role: executable implementation.
 * Responsibility: public cycle contract, validation, adapters, recording, and replay.
 * Upstream boundary: external cycle data and validated session configuration.
 * Downstream boundary: stable output frames, issues, debug views, and replay records.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

#include "sbirs_sensor/session/SbirsSessionCompositionRoot.h"

#include "sbirs_sensor/runtime/SbirsPipelineConfigMapper.h"

namespace sbirs_sensor {
namespace session {

std::unique_ptr<runtime::SbirsController> CreateSbirsController(
    const config::SbirsSessionConfig& config) {
  return std::unique_ptr<runtime::SbirsController>(
      new runtime::SbirsController(runtime::MapSessionToInternal(config)));
}

}  // namespace session
}  // namespace sbirs_sensor
