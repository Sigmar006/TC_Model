/**
 * @file scenes/ballistic_trajectory.cpp
 * @brief 弹道目标二体椭圆轨道闭式求解与解析推进（见 scenes/ballistic_trajectory.h）。
 */

#include "scenes/ballistic_trajectory.h"

#include <algorithm>
#include <cmath>

#include "1q/coordinate/position_transform.h"

namespace component_attachment {
namespace app {
namespace {

// 场景层本地定义（与 src/common/geometry/EarthOccultation.h 的
// kMeanEarthRadiusM 同值同源语义；examples 层不引 src 内部头）。
// 设计文档 rir_ballistic_scene_design_2026-08-28.md §3.1 的 Re = 6371 km。
constexpr double kDesignMeanEarthRadiusM = 6371000.0;
// 地心引力常数（m³/s²；与推演层 TargetInferenceConfig 数值同源，常数复制
// 而非引头文件，避免场景层 → 推理层逆向依赖）。
constexpr double kEarthMuM3PerS2 = 3.986004418e14;
// 重合/对跖点余弦域判定余隙（|cos D| > 1 − 1e-12 视为退化几何；换算角域
// ≈ 1.4e-6 rad，远小于任何真实弹道的转角）。
constexpr double kDegenerateCosineMargin = 1.0e-12;
// 二分迭代次数：区间 (π−D, π) 宽度 ≤ π，64 次后宽度 ~1e-16·π，达双精度极限。
constexpr int kBisectIterations = 64;

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

double Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

double Norm(const Vec3& a) { return std::sqrt(Dot(a, a)); }

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

Vec3 Scaled(const Vec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }

Vec3 Normalized(const Vec3& a) {
  const double n = Norm(a);
  return n > 0.0 ? Scaled(a, 1.0 / n) : Vec3{};
}

Vec3 Add(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }

Vec3 Sub(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

Vec3 ToVec3(const oneq::coordinate::Vector3d& v) { return {v.x, v.y, v.z}; }

oneq::coordinate::Vector3d ToVector3d(const Vec3& v) { return {v.x, v.y, v.z}; }

/// 真近点角 → 偏近点角（E 与 ν 同半平面，ν ∈ (0, 2π) 时 E ∈ (0, 2π)）。
double TrueToEccentricAnomalyRad(double true_anomaly_rad, double e) {
  const double half = 0.5 * true_anomaly_rad;
  return 2.0 * std::atan2(std::sqrt(1.0 - e) * std::sin(half),
                          std::sqrt(1.0 + e) * std::cos(half));
}

/// 平近点角 M = E − e·sinE。
double MeanAnomalyRad(double eccentric_anomaly_rad, double e) {
  return eccentric_anomaly_rad - e * std::sin(eccentric_anomaly_rad);
}

/// 牛顿迭代解 Kepler 方程 E − e·sinE = M（M ∈ [0, 2π)，e < 1）。
double SolveKeplerEquationRad(double mean_anomaly_rad, double e) {
  double m = std::fmod(mean_anomaly_rad, 2.0 * 3.14159265358979323846);
  if (m < 0.0) {
    m += 2.0 * 3.14159265358979323846;
  }
  // 初值 M + e·sinM（e ≤ ~0.7 收敛稳健），收敛门 1e-13 rad。
  double eccentric = m + e * std::sin(m);
  for (int i = 0; i < 50; ++i) {
    const double residual = eccentric - e * std::sin(eccentric) - m;
    if (std::fabs(residual) < 1.0e-13) {
      break;
    }
    eccentric -= residual / (1.0 - e * std::cos(eccentric));
  }
  return eccentric;
}

/// r(ν) = p/(1 + e·cosν)：约束 e₀(ν) = (r_a−r₀)/(r_a + r₀·cosν)。
double EccentricityFromStartRadius(double nu_rad, double r_a_m, double r0_m) {
  return (r_a_m - r0_m) / (r_a_m + r0_m * std::cos(nu_rad));
}

}  // namespace

bool SolveBallisticTrajectory(const oneq::coordinate::LlaPositionDegM& start_lla,
                              const oneq::coordinate::LlaPositionDegM& end_lla,
                              double max_alt_m, double max_alt_time_s,
                              BallisticTrajectory* trajectory) {
  if (trajectory == nullptr) {
    return false;
  }
  *trajectory = BallisticTrajectory{};
  if (!(max_alt_m > 0.0) || !(max_alt_time_s > 0.0) || !std::isfinite(max_alt_m) ||
      !std::isfinite(max_alt_time_s)) {
    return false;
  }

  oneq::coordinate::EcefPositionM start_ecef;
  oneq::coordinate::EcefPositionM end_ecef;
  if (!oneq::coordinate::TryLlaToEcef(start_lla, &start_ecef) ||
      !oneq::coordinate::TryLlaToEcef(end_lla, &end_ecef)) {
    return false;
  }
  const Vec3 r0_vec{start_ecef.x_m, start_ecef.y_m, start_ecef.z_m};
  const Vec3 r1_vec{end_ecef.x_m, end_ecef.y_m, end_ecef.z_m};
  const double r0 = Norm(r0_vec);
  const double r1 = Norm(r1_vec);
  if (!(r0 > 0.0) || !(r1 > 0.0)) {
    return false;
  }
  const Vec3 u0 = Scaled(r0_vec, 1.0 / r0);
  const Vec3 u1 = Scaled(r1_vec, 1.0 / r1);

  // 大圆角 D（发射→落点）；重合/对跖点无唯一轨道面。判定在余弦域做——
  // acos 在 ±1 附近把 1e-16 级点积误差放大为 ~1e-8 rad，角域阈值会漏放
  // 数值噪声下的对跖点（D = π − 1.4e-8 会被误判为有效弧线）。
  const double cos_d = Clamp(Dot(u0, u1), -1.0, 1.0);
  if (cos_d > 1.0 - kDegenerateCosineMargin ||
      cos_d < -(1.0 - kDegenerateCosineMargin)) {
    return false;
  }
  const double d = std::acos(cos_d);

  // 远地点地心距 = 均值半径 + 顶高；须高于两端点（顶点约束才可满足）。
  const double r_a = kDesignMeanEarthRadiusM + max_alt_m;
  if (!(r_a > r0) || !(r_a > r1)) {
    return false;
  }

  // 三约束解发射点真近点角 ν₀ ∈ (π−D, π)：e₀(ν₀) = e₁(ν₀)。
  // e₀ 在区间内随 ν₀ 严格增、e₁ 严格减 → f = e₀ − e₁ 严格增，两端异号，
  // 二分必收敛（对称情形 r₀ = r₁ 时根为 ν₀ = π − D/2，退化回设计表公式）。
  const double pi = 3.14159265358979323846;
  double lo = pi - d;
  double hi = pi;
  const double f_lo =
      EccentricityFromStartRadius(lo, r_a, r0) - (r_a - r1) / (r_a + r1 * std::cos(lo + d));
  const double f_hi =
      EccentricityFromStartRadius(hi, r_a, r0) - (r_a - r1) / (r_a + r1 * std::cos(hi + d));
  if (!(f_lo < 0.0) || !(f_hi > 0.0)) {
    return false;
  }
  double nu0 = 0.5 * (lo + hi);
  for (int i = 0; i < kBisectIterations; ++i) {
    nu0 = 0.5 * (lo + hi);
    const double f_mid = EccentricityFromStartRadius(nu0, r_a, r0) -
                         (r_a - r1) / (r_a + r1 * std::cos(nu0 + d));
    if (f_mid < 0.0) {
      lo = nu0;
    } else {
      hi = nu0;
    }
  }

  const double e = EccentricityFromStartRadius(nu0, r_a, r0);
  if (!(e > 0.0) || !(e < 1.0 - 1.0e-12)) {
    return false;
  }
  const double semi_latus = r_a * (1.0 - e);
  const double semi_major = r_a / (1.0 + e);
  const double mean_motion = std::sqrt(kEarthMuM3PerS2 / (semi_major * semi_major * semi_major));

  // 关机点（发射点）速度：vis-viva。
  const double burnout_velocity =
      std::sqrt(kEarthMuM3PerS2 * (2.0 / r0 - 1.0 / semi_major));

  // 时间闭式：M(ν) = E(ν) − e·sinE，t 自近地点起算。
  const double m0 = MeanAnomalyRad(TrueToEccentricAnomalyRad(nu0, e), e);
  const double m_land = MeanAnomalyRad(TrueToEccentricAnomalyRad(nu0 + d, e), e);
  const double fit_time_to_apogee = (pi - m0) / mean_motion;
  const double fit_flight_time = (m_land - m0) / mean_motion;
  const double burnout_time = max_alt_time_s - fit_time_to_apogee;
  if (!(burnout_time > 0.0)) {
    return false;  // 数据顶高时刻早于拟合弧线顶点：两段式时间基准不成立
  }

  // 轨道面基矢：N = u0×u1（法向）、e2 = N×u0（发射点运动方向）；
  // P（近地点方向）与 Q = N×P 使 r_dir(ν₀) = u0、r_dir(ν₀+D) = u1。
  const Vec3 normal = Normalized(Cross(u0, u1));
  const Vec3 e2 = Normalized(Cross(normal, u0));
  const Vec3 perigee_dir =
      Sub(Scaled(u0, std::cos(nu0)), Scaled(e2, std::sin(nu0)));
  const Vec3 in_plane_dir =
      Add(Scaled(e2, std::cos(nu0)), Scaled(u0, std::sin(nu0)));
  // 关机点速度方向：v ∝ −sinν·P + (e+cosν)·Q（椭圆运动通用式，模长恒正）。
  const Vec3 velocity_dir = Normalized(
      Add(Scaled(perigee_dir, -std::sin(nu0)),
          Scaled(in_plane_dir, e + std::cos(nu0))));
  if (!(Norm(velocity_dir) > 0.0)) {
    return false;
  }

  trajectory->valid = true;
  trajectory->semi_major_axis_m = semi_major;
  trajectory->eccentricity = e;
  trajectory->semi_latus_rectum_m = semi_latus;
  trajectory->mean_motion_rad_per_s = mean_motion;
  trajectory->apogee_radius_m = r_a;
  trajectory->transfer_angle_rad = d;
  trajectory->burnout_true_anomaly_rad = nu0;
  trajectory->burnout_mean_anomaly_rad = m0;
  trajectory->burnout_time_s = burnout_time;
  trajectory->burnout_velocity_mps = burnout_velocity;
  trajectory->fit_time_to_apogee_s = fit_time_to_apogee;
  trajectory->fit_flight_time_s = fit_flight_time;
  trajectory->max_alt_time_s = burnout_time + fit_time_to_apogee;
  trajectory->landing_time_s = burnout_time + fit_flight_time;
  trajectory->launch_position_ecef_m = start_ecef;
  trajectory->launch_velocity_dir = ToVector3d(velocity_dir);
  trajectory->perigee_dir = ToVector3d(perigee_dir);
  trajectory->in_plane_dir = ToVector3d(in_plane_dir);
  return true;
}

void PropagateBallistic(const BallisticTrajectory& trajectory, double t_sec,
                        oneq::coordinate::EcefPositionM* position_ecef_m,
                        oneq::coordinate::EcefVelocityMps* velocity_ecef_mps) {
  if (!trajectory.valid) {
    return;
  }
  if (t_sec < trajectory.burnout_time_s) {
    // 助推段占位：保持发射点，速度幅值 0→v_bo 线性爬升（RIR 不可见段）。
    const double ramp =
        trajectory.burnout_time_s > 0.0
            ? Clamp(t_sec / trajectory.burnout_time_s, 0.0, 1.0)
            : 0.0;
    const double speed = trajectory.burnout_velocity_mps * ramp;
    if (position_ecef_m != nullptr) {
      *position_ecef_m = trajectory.launch_position_ecef_m;
    }
    if (velocity_ecef_mps != nullptr) {
      velocity_ecef_mps->x_mps =
          trajectory.launch_velocity_dir.x * speed;
      velocity_ecef_mps->y_mps =
          trajectory.launch_velocity_dir.y * speed;
      velocity_ecef_mps->z_mps =
          trajectory.launch_velocity_dir.z * speed;
    }
    return;
  }

  // 椭圆弧段：M = M₀ + n·(t − t_bo)，牛顿解 Kepler 方程后换算真近点角。
  const double mean_anomaly =
      trajectory.burnout_mean_anomaly_rad +
      trajectory.mean_motion_rad_per_s * (t_sec - trajectory.burnout_time_s);
  const double eccentric =
      SolveKeplerEquationRad(mean_anomaly, trajectory.eccentricity);
  const double true_anomaly = 2.0 * std::atan2(
      std::sqrt(1.0 + trajectory.eccentricity) * std::sin(0.5 * eccentric),
      std::sqrt(1.0 - trajectory.eccentricity) * std::cos(0.5 * eccentric));
  const double radius = trajectory.semi_latus_rectum_m /
                        (1.0 + trajectory.eccentricity * std::cos(true_anomaly));
  const Vec3 p = ToVec3(trajectory.perigee_dir);
  const Vec3 q = ToVec3(trajectory.in_plane_dir);
  const double cos_nu = std::cos(true_anomaly);
  const double sin_nu = std::sin(true_anomaly);
  const Vec3 position{
      radius * (cos_nu * p.x + sin_nu * q.x),
      radius * (cos_nu * p.y + sin_nu * q.y),
      radius * (cos_nu * p.z + sin_nu * q.z)};
  const double velocity_scale =
      std::sqrt(kEarthMuM3PerS2 / trajectory.semi_latus_rectum_m);
  const Vec3 velocity = Scaled(
      Add(Scaled(p, -sin_nu), Scaled(q, trajectory.eccentricity + cos_nu)),
      velocity_scale);
  if (position_ecef_m != nullptr) {
    position_ecef_m->x_m = position.x;
    position_ecef_m->y_m = position.y;
    position_ecef_m->z_m = position.z;
  }
  if (velocity_ecef_mps != nullptr) {
    velocity_ecef_mps->x_mps = velocity.x;
    velocity_ecef_mps->y_mps = velocity.y;
    velocity_ecef_mps->z_mps = velocity.z;
  }
}

}  // namespace app
}  // namespace component_attachment
