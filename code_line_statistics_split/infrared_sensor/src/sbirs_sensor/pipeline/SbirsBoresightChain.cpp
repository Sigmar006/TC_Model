#include "sbirs_sensor/pipeline/SbirsBoresightChain.h"

namespace sbirs_sensor {
namespace pipeline {

namespace {

oneq::coordinate::EulerAnglesDeg ToCoordinateEuler(const session::SbirsEulerAnglesDeg& angles) {
  return oneq::coordinate::EulerAnglesDeg(angles.yaw_deg, angles.pitch_deg, angles.roll_deg);
}

oneq::coordinate::Vector3d ToVector3d(const session::SbirsVector3M& vector) {
  return oneq::coordinate::Vector3d(vector.x, vector.y, vector.z);
}

session::SbirsVector3M ToSbirsVector(const oneq::coordinate::Vector3d& vector) {
  return session::SbirsVector3M(vector.x, vector.y, vector.z);
}

}  // namespace

SbirsBoresightChain::SbirsBoresightChain(const session::SbirsEulerAnglesDeg& attitude_eci_body_deg,
                                         const oneq::coordinate::EulerAnglesDeg& mount_angles_deg)
    : SbirsBoresightChain(attitude_eci_body_deg, mount_angles_deg,
                          oneq::coordinate::EulerAnglesDeg()) {}

SbirsBoresightChain::SbirsBoresightChain(const session::SbirsEulerAnglesDeg& attitude_eci_body_deg,
                                         const oneq::coordinate::EulerAnglesDeg& mount_angles_deg,
                                         const oneq::coordinate::EulerAnglesDeg& misalignment_deg)
    : chain_(ToCoordinateEuler(attitude_eci_body_deg), mount_angles_deg, misalignment_deg) {}

bool SbirsBoresightChain::IsIdentity() const { return chain_.IsIdentity(); }

session::SbirsVector3M SbirsBoresightChain::RotateSensorToEci(
    const session::SbirsVector3M& sensor_los) const {
  return ToSbirsVector(chain_.RotateSensorToReference(ToVector3d(sensor_los)));
}

session::SbirsVector3M SbirsBoresightChain::RotateEciToSensor(
    const session::SbirsVector3M& eci_vector) const {
  return ToSbirsVector(chain_.RotateReferenceToSensor(ToVector3d(eci_vector)));
}

session::SbirsVector3M SbirsBoresightChain::EciLosOfSensorPointing(
    float sensor_azimuth_deg, float sensor_elevation_deg) const {
  return ToSbirsVector(
      chain_.ReferenceLosOfSensorPointing(sensor_azimuth_deg, sensor_elevation_deg));
}

void SbirsBoresightChain::SensorAzElOfEciVector(const session::SbirsVector3M& eci_los,
                                                float* azimuth_deg,
                                                float* elevation_deg) const {
  chain_.SensorAzElOfReferenceVector(ToVector3d(eci_los), azimuth_deg, elevation_deg);
}

void SbirsBoresightChain::SensorPointingForDesiredEciLos(
    const session::SbirsVector3M& desired_eci_los, float* azimuth_deg,
    float* elevation_deg) const {
  chain_.SensorPointingForDesiredReferenceLos(ToVector3d(desired_eci_los), azimuth_deg,
                                              elevation_deg);
}

void SbirsBoresightChain::ClampToScanLimits(const config::SbirsScanLimitsDeg& limits,
                                            float* azimuth_deg, float* elevation_deg) {
  oneq::common::geometry::AzimuthElevationLimitsDeg common_limits;
  common_limits.az_min_deg = limits.az_min_deg;
  common_limits.az_max_deg = limits.az_max_deg;
  common_limits.el_min_deg = limits.el_min_deg;
  common_limits.el_max_deg = limits.el_max_deg;
  oneq::common::geometry::BoresightChain::ClampToScanLimits(common_limits, azimuth_deg,
                                                            elevation_deg);
}

}  // namespace pipeline
}  // namespace sbirs_sensor
