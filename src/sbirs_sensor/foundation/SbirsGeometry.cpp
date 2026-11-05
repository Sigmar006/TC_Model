/**
 * ## 过程：视线几何量与地平线/临边几何（红外处理链 · 第 3 步）
 * - 上游：ECI 场景目标位置/速度（第 1 步产物）与卫星 ECI 位置/速度；地球取圆球模型。
 * - 本步：把"目标 − 卫星"矢量折算成可观测/可判定的标量：
 *   · 方位 az = atan2(y, x)、俯仰 el = asin(z/|r|)（ECI 主轴定义，单位 deg）；
 *   · 两方向夹角用球面余弦律 cosθ = sin el₁·sin el₂ + cos el₁·cos el₂·cos Δaz；
 *   · 相对视线角速度 ω = |v⊥|/R，其中 v⊥ = v − (v·r̂)r̂ 剔除径向分量（单位 deg/s）；
 *   · 地球遮挡：视线最近接近距离与地球半径之差（余量 < 0 即被遮挡）；
 *   · 视线与地球球面交点 → 地面落点经纬度（ECI 求交后旋回 ECEF 取固连经纬度）；
 *   · 焦平面脱靶量 x = f·tan(Δaz)、y = f·tan(Δel)，再除像元间距换算像素。
 * - 下游：WFOV/NFOV 视场门与排除码、遮挡判据、误差模型（折射/动态滞后用的斜距与角速度）、
 *   归属诊断（指向误差、焦平面脱靶量）、`SbirsTrackingCoordinator` 把后验位置转输出角。
 * - 关键参数：焦距 `hardware.focal_length_m`(2 m)、像元间距 `hardware.detector_pixel_pitch_m`
 *   (30 µm) 来自硬件配置；地球半径 `kMeanEarthRadiusM`(6371 km) 为库内常量。
 * - 对应验收项：第 5 项 探测角度计算功能测试；第 6 项 最大探测距离计算功能测试（几何地平线
 *   同源判据）；第 7 项 目标可探测性与动态参数生成功能测试（视线角速度）；
 *   第 26 项 关键精度指标提取功能测试（焦平面脱靶量数据源）。
 */

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

// 过程：把 asin/acos 的入参钳到 [−1, 1]，防浮点误差越界产生 NaN。
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

// 过程：方位角取视线在 ECI 的 xy 投影，atan2 保证四象限正确，值域 (−180°, 180°]。
float ComputeAzimuthDeg(const session::SbirsVector3M& los) {
  return static_cast<float>(RadToDeg(std::atan2(los.y, los.x)));
}

// 过程：俯仰角 = asin(z/|r|)，z/|r| 即视线单位向量的天向分量；零长视线无定义时回 0
//       （调用方另有距离门，不会真把零向量送到这里算角）。
float ComputeElevationDeg(const session::SbirsVector3M& los) {
  const double range = Norm(los);
  if (range <= 0.0) {
    return 0.0f;
  }
  return static_cast<float>(RadToDeg(std::asin(Clamp(los.z / range, -1.0, 1.0))));
}

// 过程：两方向张角按球面余弦律求解（等价于两单位向量点积的 acos），
//       用于指向误差/角距这类只关心张角、不关心朝向的度量。
float AngularSeparationDeg(float az_a_deg, float el_a_deg, float az_b_deg, float el_b_deg) {
  const double az_a = DegToRad(az_a_deg);
  const double el_a = DegToRad(el_a_deg);
  const double az_b = DegToRad(az_b_deg);
  const double el_b = DegToRad(el_b_deg);
  const double dot =
      std::sin(el_a) * std::sin(el_b) + std::cos(el_a) * std::cos(el_b) * std::cos(az_a - az_b);
  return static_cast<float>(RadToDeg(std::acos(Clamp(dot, -1.0, 1.0))));
}

// 过程：角速度量纲为 rad/s，这里乘 180/π 转 deg/s —— 误差模型的动态滞后按 deg/s 取用。
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

// 过程：适配层——SBIRS 三分量向量转坐标域 EcefPositionM；圆球模型对旋转不变，
//       同原点的惯性系分量可直接代入，遮挡判据本身在 common/geometry/EarthOccultation。
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

// 过程：只做球体相交判定（不含地形/椭球/大气折射），相切视为遮挡，布尔结果进排除码。
bool IsEarthOcculted(const session::SbirsVector3M& satellite_position_ecef_m,
                     const session::SbirsVector3M& target_position_ecef_m, double earth_radius_m) {
  const oneq::coordinate::EcefPositionM observer(satellite_position_ecef_m.x,
                                                 satellite_position_ecef_m.y,
                                                 satellite_position_ecef_m.z);
  const oneq::coordinate::EcefPositionM target(target_position_ecef_m.x, target_position_ecef_m.y,
                                               target_position_ecef_m.z);
  return oneq::common::geometry::IsEarthOcculted(observer, target, earth_radius_m);
}

// 过程：方向向量不要求单位化（二次式系数 a=|d|² 已吸收尺度），半径必须为正；
//       输出为米制交点坐标，无有效交点时不写输出。
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

// 过程：地心纬度（非大地纬度）= asin(z/|r|)，经度 = atan2(y, x)；圆球模型下不做椭球
//       扁率修正，与遮挡/地平线判据同一几何口径；零长向量或空指针时两角都回 0。
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
  // 过程：交点在 ECI 求得后已旋回 ECEF，此处取的是与地球固连的地心经纬度（deg）。
  ComputeGeocentricLatLonDeg(intersection_ecef_m, latitude_deg, longitude_deg);
  return true;
}

// 过程：焦面线位移用正切映射 x = f·tan(Δaz)、y = f·tan(Δel)（Δ 可达度级，故不用小角线性
//       近似），再除以像元间距得像素偏移；焦距或像元间距非正时不可解算。
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
