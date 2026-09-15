#include "sbirs_sensor/foundation/SbirsGeometry.h"

#include <algorithm>
#include <cmath>

#include "1q/coordinate/inertial_transform.h"
#include "1q/coordinate/types.h"
#include "common/geometry/EarthOccultation.h"
#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace foundation {
namespace {

using oneq::common::numerics::kPi;
using oneq::common::numerics::DegToRad;
using oneq::common::numerics::RadToDeg;

double Clamp(double value, double low, double high) { return std::max(low, std::min(high, value)); }

}  // namespace

double Dot(const session::SbirsVector3M& lhs, const session::SbirsVector3M& rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

double Norm(const session::SbirsVector3M& value) { return std::sqrt(Dot(value, value)); }

session::SbirsVector3M Subtract(const session::SbirsVector3M& lhs,
                                const session::SbirsVector3M& rhs) {
  session::SbirsVector3M result;
  result.x = lhs.x - rhs.x;
  result.y = lhs.y - rhs.y;
  result.z = lhs.z - rhs.z;
  return result;
}

session::SbirsVector3M Unit(const session::SbirsVector3M& value) {
  const double norm = Norm(value);
  if (norm <= 0.0) {
    return session::SbirsVector3M{};
  }
  session::SbirsVector3M result;
  result.x = value.x / norm;
  result.y = value.y / norm;
  result.z = value.z / norm;
  return result;
}

float ComputeAzimuthDeg(const session::SbirsVector3M& los) {
  return static_cast<float>(RadToDeg(std::atan2(los.y, los.x)));
}

float ComputeElevationDeg(const session::SbirsVector3M& los) {
  const double range = Norm(los);
  if (range <= 0.0) {
    return 0.0f;
  }
  return static_cast<float>(RadToDeg(std::asin(Clamp(los.z / range, -1.0, 1.0))));
}

float AngularSeparationDeg(float az_a_deg, float el_a_deg, float az_b_deg, float el_b_deg) {
  const double az_a = DegToRad(az_a_deg);
  const double el_a = DegToRad(el_a_deg);
  const double az_b = DegToRad(az_b_deg);
  const double el_b = DegToRad(el_b_deg);
  const double dot =
      std::sin(el_a) * std::sin(el_b) + std::cos(el_a) * std::cos(el_b) * std::cos(az_a - az_b);
  return static_cast<float>(RadToDeg(std::acos(Clamp(dot, -1.0, 1.0))));
}

float ComputeRelativeAngularRateDegPerSec(const session::SbirsVector3M& relative_position_m,
                                          const session::SbirsVector3M& relative_velocity_m_per_s) {
  const double range = Norm(relative_position_m);
  if (range <= 0.0) {
    return 0.0f;
  }
  // 视线单位向量与速度的径向分量，垂直分量 v_perp = v - (v·r̂) r̂
  const session::SbirsVector3M u = Unit(relative_position_m);
  const double radial_speed = Dot(relative_velocity_m_per_s, u);
  session::SbirsVector3M perp;
  perp.x = relative_velocity_m_per_s.x - radial_speed * u.x;
  perp.y = relative_velocity_m_per_s.y - radial_speed * u.y;
  perp.z = relative_velocity_m_per_s.z - radial_speed * u.z;
  const double perp_norm = Norm(perp);
  return static_cast<float>(RadToDeg(perp_norm / range));
}

double ComputeEarthOccultationMarginM(const session::SbirsVector3M& satellite_position_ecef_m,
                                      const session::SbirsVector3M& target_position_ecef_m,
                                      double earth_radius_m) {
  const oneq::coordinate::EcefPositionM observer(satellite_position_ecef_m.x,
                                                 satellite_position_ecef_m.y,
                                                 satellite_position_ecef_m.z);
  const oneq::coordinate::EcefPositionM target(target_position_ecef_m.x, target_position_ecef_m.y,
                                               target_position_ecef_m.z);
  return oneq::common::geometry::ComputeEarthOccultationMarginM(observer, target, earth_radius_m);
}

bool IsEarthOcculted(const session::SbirsVector3M& satellite_position_ecef_m,
                     const session::SbirsVector3M& target_position_ecef_m, double earth_radius_m) {
  const oneq::coordinate::EcefPositionM observer(satellite_position_ecef_m.x,
                                                 satellite_position_ecef_m.y,
                                                 satellite_position_ecef_m.z);
  const oneq::coordinate::EcefPositionM target(target_position_ecef_m.x, target_position_ecef_m.y,
                                               target_position_ecef_m.z);
  return oneq::common::geometry::IsEarthOcculted(observer, target, earth_radius_m);
}

bool TryIntersectRayWithSphere(const session::SbirsVector3M& origin_m,
                               const session::SbirsVector3M& direction_m, double sphere_radius_m,
                               session::SbirsVector3M* intersection_m) {
  if (intersection_m == nullptr || sphere_radius_m <= 0.0) {
    return false;
  }
  // |o + t·d|² = r² → a·t² + b·t + c = 0；取最小的正根（最近交点）。
  const double a = Dot(direction_m, direction_m);
  if (a <= 0.0) {
    return false;
  }
  const double b = 2.0 * Dot(origin_m, direction_m);
  const double c = Dot(origin_m, origin_m) - sphere_radius_m * sphere_radius_m;
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0) {
    return false;
  }
  const double root = std::sqrt(discriminant);
  const double t_near = (-b - root) / (2.0 * a);
  const double t_far = (-b + root) / (2.0 * a);
  // 起点在球外时近根有效；起点在球内时近根为负，取远根（出射点）。
  const double t = t_near > 0.0 ? t_near : t_far;
  if (t <= 0.0) {
    return false;
  }
  intersection_m->x = origin_m.x + t * direction_m.x;
  intersection_m->y = origin_m.y + t * direction_m.y;
  intersection_m->z = origin_m.z + t * direction_m.z;
  return true;
}

void ComputeGeocentricLatLonDeg(const session::SbirsVector3M& position_ecef_m,
                                double* latitude_deg, double* longitude_deg) {
  const double radius = Norm(position_ecef_m);
  if (radius <= 0.0 || latitude_deg == nullptr || longitude_deg == nullptr) {
    if (latitude_deg != nullptr) {
      *latitude_deg = 0.0;
    }
    if (longitude_deg != nullptr) {
      *longitude_deg = 0.0;
    }
    return;
  }
  *latitude_deg = RadToDeg(std::asin(Clamp(position_ecef_m.z / radius, -1.0, 1.0)));
  *longitude_deg = RadToDeg(std::atan2(position_ecef_m.y, position_ecef_m.x));
}

bool TryComputeGroundIntersectionLatLonDeg(const session::SbirsVector3M& satellite_position_eci_m,
                                           const session::SbirsVector3M& direction_eci,
                                           double earth_radius_m, double gmst_rad,
                                           double* latitude_deg, double* longitude_deg) {
  if (latitude_deg == nullptr || longitude_deg == nullptr) {
    return false;
  }
  // 圆球模型对绕 z 旋转不变：直接在 ECI 中求交，交点再旋回 ECEF 取固连地球经纬度。
  session::SbirsVector3M intersection_eci;
  if (!TryIntersectRayWithSphere(satellite_position_eci_m, direction_eci, earth_radius_m,
                                 &intersection_eci)) {
    return false;
  }
  const oneq::coordinate::EciPositionM intersection_eci_position(
      intersection_eci.x, intersection_eci.y, intersection_eci.z);
  oneq::coordinate::EcefPositionM intersection_ecef;
  if (!oneq::coordinate::TryEciToEcef(intersection_eci_position, gmst_rad, &intersection_ecef)) {
    return false;
  }
  const session::SbirsVector3M intersection_ecef_m{intersection_ecef.x_m, intersection_ecef.y_m,
                                                   intersection_ecef.z_m};
  ComputeGeocentricLatLonDeg(intersection_ecef_m, latitude_deg, longitude_deg);
  return true;
}

bool ComputeFocalPlaneOffset(double focal_length_m, double pixel_pitch_m, float delta_az_deg,
                             float delta_el_deg, SbirsFocalPlaneOffset* offset) {
  if (offset == nullptr || focal_length_m <= 0.0 || pixel_pitch_m <= 0.0) {
    return false;
  }
  offset->x_m = focal_length_m * std::tan(DegToRad(static_cast<double>(delta_az_deg)));
  offset->y_m = focal_length_m * std::tan(DegToRad(static_cast<double>(delta_el_deg)));
  offset->x_pixels = offset->x_m / pixel_pitch_m;
  offset->y_pixels = offset->y_m / pixel_pitch_m;
  return true;
}

}  // namespace foundation
}  // namespace sbirs_sensor
