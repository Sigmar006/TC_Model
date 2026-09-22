/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsExternalInputAdapter.
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

#include "1q/sbirs_sensor/session/SbirsExternalInputAdapter.h"

namespace sbirs_sensor {
namespace session {

SbirsCycleInput MakeSbirsCycleInput(std::uint32_t cycle_index, float dt_sec,
                                    double utc_julian_day,
                                    const SbirsVector3M& satellite_position_ecef_m,
                                    const SbirsVector3M& satellite_velocity_ecef_m_per_s,
                                    const SbirsEulerAnglesDeg& satellite_attitude_eci_body_deg,
                                    const SbirsSceneTargetList& scene) {
  SbirsCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = dt_sec;
  input.utc_julian_day = utc_julian_day;
  input.satellite_position_ecef_m = satellite_position_ecef_m;
  input.satellite_velocity_ecef_m_per_s = satellite_velocity_ecef_m_per_s;
  input.satellite_attitude_eci_body_deg = satellite_attitude_eci_body_deg;
  input.scene = scene;
  return input;
}

}  // namespace session
}  // namespace sbirs_sensor
