#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"

namespace sbirs_sensor {
namespace session {

SbirsCycleInputBuilder& SbirsCycleInputBuilder::WithCycleIndex(std::uint32_t cycle_index) {
  input_.cycle_index = cycle_index;
  return *this;
}

SbirsCycleInputBuilder& SbirsCycleInputBuilder::WithDeltaTimeSec(float dt_sec) {
  input_.dt_sec = dt_sec;
  return *this;
}

SbirsCycleInputBuilder& SbirsCycleInputBuilder::WithSatellitePosition(
    const SbirsVector3M& position_ecef_m) {
  input_.satellite_position_ecef_m = position_ecef_m;
  return *this;
}

SbirsCycleInputBuilder& SbirsCycleInputBuilder::WithSatelliteVelocity(
    const SbirsVector3M& velocity_ecef_m_per_s) {
  input_.satellite_velocity_ecef_m_per_s = velocity_ecef_m_per_s;
  return *this;
}

SbirsCycleInputBuilder& SbirsCycleInputBuilder::WithSatelliteAttitude(
    const SbirsEulerAnglesDeg& attitude_eci_body_deg) {
  input_.satellite_attitude_eci_body_deg = attitude_eci_body_deg;
  return *this;
}

SbirsCycleInputBuilder& SbirsCycleInputBuilder::WithUtcJulianDay(double utc_julian_day) {
  input_.utc_julian_day = utc_julian_day;
  return *this;
}

SbirsCycleInputBuilder& SbirsCycleInputBuilder::AddTarget(const SbirsSceneTarget& target) {
  input_.scene.push_back(target);
  return *this;
}

SbirsCycleInput SbirsCycleInputBuilder::Build() const { return input_; }

}  // namespace session
}  // namespace sbirs_sensor
