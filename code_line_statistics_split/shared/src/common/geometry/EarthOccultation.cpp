#include "common/geometry/EarthOccultation.h"

#include <algorithm>
#include <cmath>

namespace oneq {
namespace common {
namespace geometry {
namespace {

double Dot(const oneq::coordinate::EcefPositionM& lhs, const oneq::coordinate::EcefPositionM& rhs) {
  return lhs.x_m * rhs.x_m + lhs.y_m * rhs.y_m + lhs.z_m * rhs.z_m;
}

double Norm(const oneq::coordinate::EcefPositionM& value) { return std::sqrt(Dot(value, value)); }

oneq::coordinate::EcefPositionM Subtract(const oneq::coordinate::EcefPositionM& lhs,
                                         const oneq::coordinate::EcefPositionM& rhs) {
  return oneq::coordinate::EcefPositionM(lhs.x_m - rhs.x_m, lhs.y_m - rhs.y_m, lhs.z_m - rhs.z_m);
}

oneq::coordinate::EcefPositionM Unit(const oneq::coordinate::EcefPositionM& value) {
  const double norm = Norm(value);
  if (norm <= 0.0) {
    return oneq::coordinate::EcefPositionM{};
  }
  return oneq::coordinate::EcefPositionM(value.x_m / norm, value.y_m / norm, value.z_m / norm);
}

}  // namespace

double ComputeEarthOccultationMarginM(const oneq::coordinate::EcefPositionM& observer_position_ecef_m,
                                      const oneq::coordinate::EcefPositionM& target_position_ecef_m,
                                      double earth_radius_m) {
  const oneq::coordinate::EcefPositionM los = Subtract(target_position_ecef_m, observer_position_ecef_m);
  const double range = Norm(los);
  if (range <= 0.0 || earth_radius_m <= 0.0) {
    return earth_radius_m;
  }
  const oneq::coordinate::EcefPositionM u = Unit(los);
  const double s_closest = -Dot(observer_position_ecef_m, u);
  if (s_closest <= 0.0 || s_closest >= range) {
    return earth_radius_m;
  }
  const double observer_norm_sq = Dot(observer_position_ecef_m, observer_position_ecef_m);
  const double closest_sq = observer_norm_sq - s_closest * s_closest;
  return std::sqrt(std::max(closest_sq, 0.0)) - earth_radius_m;
}

bool IsEarthOcculted(const oneq::coordinate::EcefPositionM& observer_position_ecef_m,
                     const oneq::coordinate::EcefPositionM& target_position_ecef_m,
                     double earth_radius_m) {
  // 相切（margin == 0）视为遮挡：与历史 SBIRS closest_sq <= r² 判定语义一致。
  return ComputeEarthOccultationMarginM(observer_position_ecef_m, target_position_ecef_m,
                                        earth_radius_m) <= 0.0;
}

}  // namespace geometry
}  // namespace common
}  // namespace oneq
