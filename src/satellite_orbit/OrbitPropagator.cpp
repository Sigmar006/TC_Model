#include "1q/satellite_orbit/OrbitPropagator.h"

#include <algorithm>
#include <cmath>

#include "1q/coordinate/inertial_transform.h"
#include "common/numerics/Constants.h"

namespace satellite_orbit {
namespace {

using oneq::common::numerics::DegToRad;
using oneq::common::numerics::kPi;
using oneq::common::numerics::kTwoPi;

/** @brief 地球引力常数 GM（WGS-84），单位 m³/s²。 */
constexpr double kEarthMuM3PerS2 = 3.986004418e14;
/** @brief J2 带谐项系数（无量纲）。 */
constexpr double kJ2Coefficient = 1.08262668e-3;
/** @brief 地球赤道半径（WGS-84），单位 m。 */
constexpr double kEarthEquatorialRadiusM = 6378137.0;
/** @brief 一天的秒数。 */
constexpr double kSecondsPerDay = 86400.0;
/** @brief 积分步长正下限，单位 s。 */
constexpr double kMinStepSizeSec = 1.0e-3;
/** @brief 积分结束判定阈值，单位 s。 */
constexpr double kPropagationEpsilonSec = 1.0e-9;
/** @brief 开普勒方程迭代上限。 */
constexpr int kKeplerMaxIterations = 100;
/** @brief 开普勒方程迭代收敛阈值，单位 rad。 */
constexpr double kKeplerToleranceRad = 1.0e-12;

/** @brief RK4 笛卡尔状态（ECI 位置 + 速度）。 */
struct CartesianState {
  double position[3U];
  double velocity[3U];
};

/** @brief 状态导数：位置导数为速度，速度导数为加速度。 */
struct Derivative {
  double position[3U];
  double velocity[3U];
};

double VectorNorm(const double vector[3U]) {
  return std::sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

bool StateIsFinite(const CartesianState& state) {
  for (int i = 0; i < 3; ++i) {
    const int index = static_cast<int>(i);
    if (!std::isfinite(state.position[index]) || !std::isfinite(state.velocity[index])) {
      return false;
    }
  }
  return true;
}

double NormalizeRadians(double angle_rad) {
  const double normalized = std::fmod(angle_rad, kTwoPi);
  return normalized < 0.0 ? normalized + kTwoPi : normalized;
}

bool ElementsAreValid(const OrbitalElements& elements) {
  return std::isfinite(elements.semi_major_axis_m) && elements.semi_major_axis_m > 0.0 &&
         std::isfinite(elements.eccentricity) && elements.eccentricity >= 0.0 &&
         elements.eccentricity < 1.0 && std::isfinite(elements.inclination_deg) &&
         elements.inclination_deg >= 0.0 && elements.inclination_deg <= 180.0 &&
         std::isfinite(elements.raan_deg) && std::isfinite(elements.argument_of_perigee_deg) &&
         std::isfinite(elements.mean_anomaly_deg);
}

/**
 * @brief 牛顿迭代求解开普勒方程 E − e·sinE = M。
 * @param[in] mean_anomaly_rad 归一化平近点角，范围 [0, 2π)
 * @param[in] eccentricity 偏心率，范围 [0, 1)
 * @return 偏近点角 E，单位 rad
 */
double SolveKeplerEquation(double mean_anomaly_rad, double eccentricity) {
  double eccentric_anomaly = (eccentricity < 0.8) ? mean_anomaly_rad : kPi;
  for (int iteration = 0; iteration < kKeplerMaxIterations; ++iteration) {
    const double residual =
        eccentric_anomaly - eccentricity * std::sin(eccentric_anomaly) - mean_anomaly_rad;
    const double derivative = 1.0 - eccentricity * std::cos(eccentric_anomaly);
    if (std::fabs(derivative) < 1.0e-12) {
      break;
    }
    const double delta = residual / derivative;
    eccentric_anomaly -= delta;
    if (std::fabs(delta) < kKeplerToleranceRad) {
      break;
    }
  }
  return eccentric_anomaly;
}

/** @brief 绕 z 轴旋转向量（就地）。 */
void RotateZ(double angle_rad, double vector[3U]) {
  const double cos_angle = std::cos(angle_rad);
  const double sin_angle = std::sin(angle_rad);
  const double x = cos_angle * vector[0] - sin_angle * vector[1];
  const double y = sin_angle * vector[0] + cos_angle * vector[1];
  vector[0] = x;
  vector[1] = y;
}

/** @brief 绕 x 轴旋转向量（就地）。 */
void RotateX(double angle_rad, double vector[3U]) {
  const double cos_angle = std::cos(angle_rad);
  const double sin_angle = std::sin(angle_rad);
  const double y = cos_angle * vector[1] - sin_angle * vector[2];
  const double z = sin_angle * vector[1] + cos_angle * vector[2];
  vector[1] = y;
  vector[2] = z;
}

/**
 * @brief 由轨道六根数计算历元时刻的 ECI 笛卡尔状态。
 * @param[in] elements 轨道六根数
 * @param[out] state 输出的 ECI 位置与速度
 * @return 结果有限且半径为正时返回 true
 */
bool StateFromElements(const OrbitalElements& elements, CartesianState* state) {
  const double semi_major_axis = elements.semi_major_axis_m;
  const double eccentricity = elements.eccentricity;
  const double mean_anomaly = NormalizeRadians(DegToRad(elements.mean_anomaly_deg));
  const double eccentric_anomaly = SolveKeplerEquation(mean_anomaly, eccentricity);
  const double cos_eccentric = std::cos(eccentric_anomaly);
  const double sin_eccentric = std::sin(eccentric_anomaly);
  const double one_minus_e2 = 1.0 - eccentricity * eccentricity;
  const double radius = semi_major_axis * (1.0 - eccentricity * cos_eccentric);
  if (!(radius > 0.0)) {
    return false;
  }
  const double true_anomaly =
      std::atan2(std::sqrt(one_minus_e2) * sin_eccentric, cos_eccentric - eccentricity);
  const double cos_true = std::cos(true_anomaly);
  const double sin_true = std::sin(true_anomaly);
  // 近焦点坐标系：位置沿矢径，速度按 vis-viva 关系分解为切向与径向分量。
  const double semi_latus_rectum = semi_major_axis * one_minus_e2;
  const double speed_scale = std::sqrt(kEarthMuM3PerS2 / semi_latus_rectum);
  double position[3U] = {radius * cos_true, radius * sin_true, 0.0};
  double velocity[3U] = {-speed_scale * sin_true, speed_scale * (eccentricity + cos_true), 0.0};
  // 近焦点系 → ECI：R3(Ω)·R1(i)·R3(ω)。
  const double argument_of_perigee = DegToRad(elements.argument_of_perigee_deg);
  const double inclination = DegToRad(elements.inclination_deg);
  const double raan = DegToRad(elements.raan_deg);
  RotateZ(argument_of_perigee, position);
  RotateZ(argument_of_perigee, velocity);
  RotateX(inclination, position);
  RotateX(inclination, velocity);
  RotateZ(raan, position);
  RotateZ(raan, velocity);
  state->position[0] = position[0];
  state->position[1] = position[1];
  state->position[2] = position[2];
  state->velocity[0] = velocity[0];
  state->velocity[1] = velocity[1];
  state->velocity[2] = velocity[2];
  return StateIsFinite(*state);
}

/** @brief 计算 ECI 系下的引力加速度（二体 + 可选 J2）。 */
void ComputeAcceleration(const double position[3U], bool enable_j2, double acceleration[3U]) {
  const double x = position[0];
  const double y = position[1];
  const double z = position[2];
  const double radius_squared = x * x + y * y + z * z;
  const double radius = std::sqrt(radius_squared);
  const double inverse_radius_cubed = 1.0 / (radius_squared * radius);
  for (int i = 0; i < 3; ++i) {
    const int index = static_cast<int>(i);
    acceleration[index] = -kEarthMuM3PerS2 * inverse_radius_cubed * position[index];
  }
  if (!enable_j2) {
    return;
  }
  const double z_squared_ratio = (z * z) / radius_squared;
  // a_J2 = −(3/2)·J2·μ·Re²/r⁵·[x(1−5z²/r²), y(1−5z²/r²), z(3−5z²/r²)]。
  const double common = 1.5 * kJ2Coefficient * kEarthMuM3PerS2 * kEarthEquatorialRadiusM *
                        kEarthEquatorialRadiusM * inverse_radius_cubed / radius_squared;
  acceleration[0] -= common * x * (1.0 - 5.0 * z_squared_ratio);
  acceleration[1] -= common * y * (1.0 - 5.0 * z_squared_ratio);
  acceleration[2] -= common * z * (3.0 - 5.0 * z_squared_ratio);
}

/** @brief 计算状态导数。 */
Derivative Evaluate(const CartesianState& state, bool enable_j2) {
  Derivative derivative;
  for (int i = 0; i < 3; ++i) {
    const int index = static_cast<int>(i);
    derivative.position[index] = state.velocity[index];
  }
  ComputeAcceleration(state.position, enable_j2, derivative.velocity);
  return derivative;
}

/** @brief 按导数推进状态：base + scale·derivative。 */
CartesianState Advance(const CartesianState& base, const Derivative& derivative, double scale) {
  CartesianState advanced;
  for (int i = 0; i < 3; ++i) {
    const int index = static_cast<int>(i);
    advanced.position[index] = base.position[index] + scale * derivative.position[index];
    advanced.velocity[index] = base.velocity[index] + scale * derivative.velocity[index];
  }
  return advanced;
}

/** @brief 经典四阶龙格-库塔单步。 */
CartesianState Rk4Step(const CartesianState& state, double dt_sec, bool enable_j2) {
  const Derivative k1 = Evaluate(state, enable_j2);
  const Derivative k2 = Evaluate(Advance(state, k1, 0.5 * dt_sec), enable_j2);
  const Derivative k3 = Evaluate(Advance(state, k2, 0.5 * dt_sec), enable_j2);
  const Derivative k4 = Evaluate(Advance(state, k3, dt_sec), enable_j2);
  CartesianState next = state;
  const double scale = dt_sec / 6.0;
  for (int i = 0; i < 3; ++i) {
    const int index = static_cast<int>(i);
    next.position[index] =
        state.position[index] + scale * (k1.position[index] + 2.0 * k2.position[index] +
                                         2.0 * k3.position[index] + k4.position[index]);
    next.velocity[index] =
        state.velocity[index] + scale * (k1.velocity[index] + 2.0 * k2.velocity[index] +
                                         2.0 * k3.velocity[index] + k4.velocity[index]);
  }
  return next;
}

CartesianState ToCartesian(const EciState& state) {
  CartesianState cartesian;
  cartesian.position[0] = state.position.x_m;
  cartesian.position[1] = state.position.y_m;
  cartesian.position[2] = state.position.z_m;
  cartesian.velocity[0] = state.velocity.x_mps;
  cartesian.velocity[1] = state.velocity.y_mps;
  cartesian.velocity[2] = state.velocity.z_mps;
  return cartesian;
}

EciState ToEciState(const CartesianState& state) {
  EciState eci;
  eci.position =
      oneq::coordinate::EciPositionM(state.position[0], state.position[1], state.position[2]);
  eci.velocity =
      oneq::coordinate::EciVelocityMps(state.velocity[0], state.velocity[1], state.velocity[2]);
  return eci;
}

/**
 * @brief 把 ECI 状态按指定时刻 GMST 转换到 ECEF（速度含输运项补偿）。
 * @return GMST 或坐标转换失败时返回 false，输出不变
 */
bool BuildEcefState(const EciState& eci, double utc_julian_day, EcefState* ecef) {
  double gmst_rad = 0.0;
  if (!oneq::coordinate::TryComputeGmstRad(utc_julian_day, &gmst_rad)) {
    return false;
  }
  oneq::coordinate::EcefPositionM ecef_position;
  if (!oneq::coordinate::TryEciToEcef(eci.position, gmst_rad, &ecef_position)) {
    return false;
  }
  // v_ECEF = R3(−θ)·(v_ECI − ω_e×r_ECI)，ω_e×r = (−ω·y, ω·x, 0)。
  const double omega = oneq::coordinate::kEarthRotationRateRadPerSec;
  const double vx = eci.velocity.x_mps + omega * eci.position.y_m;
  const double vy = eci.velocity.y_mps - omega * eci.position.x_m;
  const double vz = eci.velocity.z_mps;
  const double cos_gmst = std::cos(gmst_rad);
  const double sin_gmst = std::sin(gmst_rad);
  EcefState result;
  result.position = ecef_position;
  result.velocity = oneq::coordinate::EcefVelocityMps(
      cos_gmst * vx + sin_gmst * vy, -sin_gmst * vx + cos_gmst * vy, vz);
  if (!std::isfinite(result.velocity.x_mps) || !std::isfinite(result.velocity.y_mps) ||
      !std::isfinite(result.velocity.z_mps)) {
    return false;
  }
  *ecef = result;
  return true;
}

}  // namespace

OrbitPropagator::OrbitPropagator() = default;

bool OrbitPropagator::Initialize(const OrbitalElements& elements, double epoch_jd) {
  if (!ElementsAreValid(elements) || !std::isfinite(epoch_jd) || epoch_jd <= 0.0) {
    return false;
  }
  CartesianState epoch_state{};
  if (!StateFromElements(elements, &epoch_state)) {
    return false;
  }
  elements_ = elements;
  epoch_jd_ = epoch_jd;
  current_state_eci_ = ToEciState(epoch_state);
  last_propagated_jd_ = epoch_jd;
  initialized_ = true;
  return true;
}

bool OrbitPropagator::PropagateToTime(double target_jd, EcefState* ecef_state) {
  if (ecef_state == nullptr || !initialized_ || !std::isfinite(target_jd) ||
      target_jd < last_propagated_jd_) {
    return false;
  }
  CartesianState state = ToCartesian(current_state_eci_);
  double remaining_sec = (target_jd - last_propagated_jd_) * kSecondsPerDay;
  if (!std::isfinite(remaining_sec)) {
    return false;
  }
  while (remaining_sec > kPropagationEpsilonSec) {
    const double dt_sec = std::min(step_size_s_, remaining_sec);
    state = Rk4Step(state, dt_sec, enable_j2_);
    if (!StateIsFinite(state)) {
      return false;
    }
    remaining_sec -= dt_sec;
  }
  if (VectorNorm(state.position) <= 0.0) {
    return false;
  }
  const EciState propagated = ToEciState(state);
  EcefState ecef;
  if (!BuildEcefState(propagated, target_jd, &ecef)) {
    return false;
  }
  current_state_eci_ = propagated;
  last_propagated_jd_ = target_jd;
  *ecef_state = ecef;
  return true;
}

void OrbitPropagator::SetStepSize(double step_size_s) {
  if (!std::isfinite(step_size_s)) {
    return;
  }
  step_size_s_ = std::max(step_size_s, kMinStepSizeSec);
}

void OrbitPropagator::SetJ2Enabled(bool enable) { enable_j2_ = enable; }

}  // namespace satellite_orbit
