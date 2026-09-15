#include "sbirs_sensor/pipeline/SbirsEciScene.h"

namespace sbirs_sensor {
namespace pipeline {

SbirsEciSceneTarget RotateSceneTargetToEci(const session::SbirsSceneTarget& source,
                                           double gmst_rad) {
  SbirsEciSceneTarget rotated;
  rotated.target_id = source.target_id;
  rotated.target_name = source.target_name;
  rotated.radiant_intensity_w_per_sr = source.radiant_intensity_w_per_sr;
  rotated.active = source.active;
  rotated.has_velocity_eci_m_per_s = source.has_velocity_ecef_m_per_s;

  const oneq::coordinate::EcefPositionM ecef_position(
      source.position_ecef_m.x, source.position_ecef_m.y, source.position_ecef_m.z);
  oneq::coordinate::EciPositionM eci_position;
  if (oneq::coordinate::TryEcefToEci(ecef_position, gmst_rad, &eci_position)) {
    rotated.position_eci_m = session::SbirsVector3M{eci_position.x_m, eci_position.y_m,
                                                    eci_position.z_m};
  } else {
    // 输入校验已拒绝非有限分量；此防御路径保留未旋转值，不静默丢目标。
    rotated.position_eci_m = source.position_ecef_m;
  }
  if (source.has_velocity_ecef_m_per_s) {
    const oneq::coordinate::EcefVelocityMps ecef_velocity(
        source.velocity_ecef_m_per_s.x, source.velocity_ecef_m_per_s.y,
        source.velocity_ecef_m_per_s.z);
    oneq::coordinate::EciVelocityMps eci_velocity;
    if (oneq::coordinate::TryEcefVelocityToEci(ecef_position, ecef_velocity, gmst_rad,
                                               &eci_velocity)) {
      rotated.velocity_eci_m_per_s =
          session::SbirsVector3M{eci_velocity.x_mps, eci_velocity.y_mps,
                                 eci_velocity.z_mps};
    } else {
      rotated.velocity_eci_m_per_s = source.velocity_ecef_m_per_s;
    }
  }
  return rotated;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
