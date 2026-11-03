/**
 * @file orbit_propagator_test.cpp
 * @brief OrbitPropagator 单元测试：根数→历元状态、RK4 周期闭合、J2 摄动、
 *        ECEF/GMST 输出一致性与 fail-closed 输入校验。
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "1q/coordinate/inertial_transform.h"
#include "1q/satellite_orbit/OrbitPropagator.h"

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kEarthMuM3PerS2 = 3.986004418e14;
constexpr double kSecondsPerDay = 86400.0;
// GMST 近似为 0 的 UTC 儒略日（J2000 前 0.7769 天，残差 <1e-6 rad），
// 用于 ECEF 与 ECI 按解析旋转逐分量对照。
constexpr double kGmstZeroJulianDay = 2451544.2230698913;

satellite_orbit::OrbitalElements CircularEquatorialElements() {
  satellite_orbit::OrbitalElements elements;
  elements.semi_major_axis_m = 7000000.0;
  elements.eccentricity = 0.0;
  elements.inclination_deg = 0.0;
  elements.raan_deg = 0.0;
  elements.argument_of_perigee_deg = 0.0;
  elements.mean_anomaly_deg = 0.0;
  return elements;
}

double Norm(const oneq::coordinate::EciPositionM& position) {
  return std::sqrt(position.x_m * position.x_m + position.y_m * position.y_m +
                   position.z_m * position.z_m);
}

void NormalOf(const satellite_orbit::EciState& state, double normal[3U]) {
  const double rx = state.position.x_m;
  const double ry = state.position.y_m;
  const double rz = state.position.z_m;
  const double vx = state.velocity.x_mps;
  const double vy = state.velocity.y_mps;
  const double vz = state.velocity.z_mps;
  normal[0] = ry * vz - rz * vy;
  normal[1] = rz * vx - rx * vz;
  normal[2] = rx * vy - ry * vx;
  const double length = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] +
                                   normal[2] * normal[2]);
  normal[0] /= length;
  normal[1] /= length;
  normal[2] /= length;
}

double AngleBetweenNormals(const satellite_orbit::EciState& lhs,
                           const satellite_orbit::EciState& rhs) {
  double lhs_normal[3U];
  double rhs_normal[3U];
  NormalOf(lhs, lhs_normal);
  NormalOf(rhs, rhs_normal);
  double cosine = lhs_normal[0] * rhs_normal[0] + lhs_normal[1] * rhs_normal[1] +
                  lhs_normal[2] * rhs_normal[2];
  cosine = std::max(-1.0, std::min(1.0, cosine));
  return std::acos(cosine);
}

}  // namespace

TEST(SatelliteOrbitPropagatorTest, EpochStateMatchesCircularEquatorialElements) {
  satellite_orbit::OrbitPropagator propagator;
  ASSERT_TRUE(propagator.Initialize(CircularEquatorialElements(), 2451545.0));
  ASSERT_TRUE(propagator.initialized());

  const satellite_orbit::EciState& epoch = propagator.current_state_eci();
  EXPECT_NEAR(epoch.position.x_m, 7000000.0, 1.0e-6);
  EXPECT_NEAR(epoch.position.y_m, 0.0, 1.0e-6);
  EXPECT_NEAR(epoch.position.z_m, 0.0, 1.0e-6);
  EXPECT_NEAR(epoch.velocity.x_mps, 0.0, 1.0e-6);
  EXPECT_NEAR(epoch.velocity.y_mps, std::sqrt(kEarthMuM3PerS2 / 7000000.0), 1.0e-6);
  EXPECT_NEAR(epoch.velocity.z_mps, 0.0, 1.0e-6);
  EXPECT_DOUBLE_EQ(propagator.epoch_julian_day(), 2451545.0);
  EXPECT_DOUBLE_EQ(propagator.last_propagated_julian_day(), 2451545.0);
}

TEST(SatelliteOrbitPropagatorTest, PerigeeRadiusAndSpeedMatchVisViva) {
  satellite_orbit::OrbitalElements elements = CircularEquatorialElements();
  elements.eccentricity = 0.2;
  elements.inclination_deg = 55.0;
  elements.raan_deg = 40.0;
  elements.argument_of_perigee_deg = 70.0;
  elements.mean_anomaly_deg = 0.0;

  satellite_orbit::OrbitPropagator propagator;
  ASSERT_TRUE(propagator.Initialize(elements, 2451545.0));
  const satellite_orbit::EciState& epoch = propagator.current_state_eci();
  const double perigee_radius = elements.semi_major_axis_m * (1.0 - elements.eccentricity);
  const double perigee_speed =
      std::sqrt(kEarthMuM3PerS2 * (1.0 + elements.eccentricity) /
                (elements.semi_major_axis_m * (1.0 - elements.eccentricity)));
  EXPECT_NEAR(Norm(epoch.position), perigee_radius, 1.0e-6);
  const double speed = std::sqrt(epoch.velocity.x_mps * epoch.velocity.x_mps +
                                 epoch.velocity.y_mps * epoch.velocity.y_mps +
                                 epoch.velocity.z_mps * epoch.velocity.z_mps);
  EXPECT_NEAR(speed, perigee_speed, 1.0e-6);
}

TEST(SatelliteOrbitPropagatorTest, OneOrbitPeriodReturnsToInitialState) {
  satellite_orbit::OrbitalElements elements = CircularEquatorialElements();
  elements.eccentricity = 0.01;
  elements.inclination_deg = 30.0;
  elements.raan_deg = 40.0;
  elements.argument_of_perigee_deg = 20.0;
  elements.mean_anomaly_deg = 10.0;

  satellite_orbit::OrbitPropagator propagator;
  ASSERT_TRUE(propagator.Initialize(elements, 2451545.0));
  propagator.SetJ2Enabled(false);
  const satellite_orbit::EciState initial = propagator.current_state_eci();
  const double period_sec =
      2.0 * kPi * std::sqrt(std::pow(elements.semi_major_axis_m, 3.0) / kEarthMuM3PerS2);

  satellite_orbit::EcefState ecef;
  ASSERT_TRUE(propagator.PropagateToTime(2451545.0 + period_sec / kSecondsPerDay, &ecef));
  const satellite_orbit::EciState& final_state = propagator.current_state_eci();
  EXPECT_NEAR(final_state.position.x_m, initial.position.x_m, 1.0);
  EXPECT_NEAR(final_state.position.y_m, initial.position.y_m, 1.0);
  EXPECT_NEAR(final_state.position.z_m, initial.position.z_m, 1.0);
  EXPECT_NEAR(final_state.velocity.x_mps, initial.velocity.x_mps, 1.0e-3);
  EXPECT_NEAR(final_state.velocity.y_mps, initial.velocity.y_mps, 1.0e-3);
  EXPECT_NEAR(final_state.velocity.z_mps, initial.velocity.z_mps, 1.0e-3);
}

TEST(SatelliteOrbitPropagatorTest, J2PerturbationRotatesOrbitPlane) {
  satellite_orbit::OrbitalElements elements = CircularEquatorialElements();
  elements.eccentricity = 0.001;
  elements.inclination_deg = 30.0;
  elements.raan_deg = 40.0;
  elements.argument_of_perigee_deg = 20.0;
  elements.mean_anomaly_deg = 10.0;

  satellite_orbit::OrbitPropagator without_j2;
  satellite_orbit::OrbitPropagator with_j2;
  ASSERT_TRUE(without_j2.Initialize(elements, 2451545.0));
  ASSERT_TRUE(with_j2.Initialize(elements, 2451545.0));
  without_j2.SetJ2Enabled(false);
  with_j2.SetJ2Enabled(true);
  const satellite_orbit::EciState initial = without_j2.current_state_eci();

  satellite_orbit::EcefState ecef;
  ASSERT_TRUE(without_j2.PropagateToTime(2451545.0 + 1.0, &ecef));
  ASSERT_TRUE(with_j2.PropagateToTime(2451545.0 + 1.0, &ecef));

  const double angle_without_j2 = AngleBetweenNormals(initial, without_j2.current_state_eci());
  const double angle_with_j2 = AngleBetweenNormals(initial, with_j2.current_state_eci());
  // 二体引力不改变轨道面，残差只剩积分舍入；J2 在一天内产生可观测的交点进动。
  EXPECT_LT(angle_without_j2, 1.0e-6);
  EXPECT_GT(angle_with_j2, 1.0e-3);
}

TEST(SatelliteOrbitPropagatorTest, EcefOutputMatchesGmstRotationAndTransport) {
  satellite_orbit::OrbitPropagator propagator;
  ASSERT_TRUE(propagator.Initialize(CircularEquatorialElements(), kGmstZeroJulianDay));
  satellite_orbit::EcefState ecef;
  ASSERT_TRUE(propagator.PropagateToTime(kGmstZeroJulianDay, &ecef));

  const satellite_orbit::EciState& eci = propagator.current_state_eci();
  // 该历元 GMST 仅剩亚微弧度残差，但不足以忽略：按解析 R3(−θ) 逐分量对照，
  // 位置用旋转矩阵，速度额外补偿 ω×r 输运项。
  double gmst_rad = 0.0;
  ASSERT_TRUE(oneq::coordinate::TryComputeGmstRad(kGmstZeroJulianDay, &gmst_rad));
  ASSERT_LT(std::fabs(gmst_rad), 1.0e-6);
  const double cos_gmst = std::cos(gmst_rad);
  const double sin_gmst = std::sin(gmst_rad);
  EXPECT_NEAR(ecef.position.x_m,
              cos_gmst * eci.position.x_m + sin_gmst * eci.position.y_m, 1.0e-6);
  EXPECT_NEAR(ecef.position.y_m,
              -sin_gmst * eci.position.x_m + cos_gmst * eci.position.y_m, 1.0e-6);
  EXPECT_NEAR(ecef.position.z_m, eci.position.z_m, 1.0e-6);
  const double omega = oneq::coordinate::kEarthRotationRateRadPerSec;
  const double velocity_x = eci.velocity.x_mps + omega * eci.position.y_m;
  const double velocity_y = eci.velocity.y_mps - omega * eci.position.x_m;
  EXPECT_NEAR(ecef.velocity.x_mps, cos_gmst * velocity_x + sin_gmst * velocity_y, 1.0e-6);
  EXPECT_NEAR(ecef.velocity.y_mps, -sin_gmst * velocity_x + cos_gmst * velocity_y, 1.0e-6);
  EXPECT_NEAR(ecef.velocity.z_mps, eci.velocity.z_mps, 1.0e-6);
}

TEST(SatelliteOrbitPropagatorTest, EcefVelocityIsInverseOfEcefToEciTransform) {
  satellite_orbit::OrbitPropagator propagator;
  ASSERT_TRUE(propagator.Initialize(CircularEquatorialElements(), 2451545.0));
  const double target_jd = 2451545.0 + 0.5;
  satellite_orbit::EcefState ecef;
  ASSERT_TRUE(propagator.PropagateToTime(target_jd, &ecef));

  double gmst_rad = 0.0;
  ASSERT_TRUE(oneq::coordinate::TryComputeGmstRad(target_jd, &gmst_rad));
  oneq::coordinate::EciPositionM eci_position;
  oneq::coordinate::EciVelocityMps eci_velocity;
  ASSERT_TRUE(oneq::coordinate::TryEcefToEci(ecef.position, gmst_rad, &eci_position));
  ASSERT_TRUE(oneq::coordinate::TryEcefVelocityToEci(ecef.position, ecef.velocity, gmst_rad,
                                                      &eci_velocity));
  const satellite_orbit::EciState& current = propagator.current_state_eci();
  EXPECT_NEAR(eci_position.x_m, current.position.x_m, 1.0e-6);
  EXPECT_NEAR(eci_position.y_m, current.position.y_m, 1.0e-6);
  EXPECT_NEAR(eci_position.z_m, current.position.z_m, 1.0e-6);
  EXPECT_NEAR(eci_velocity.x_mps, current.velocity.x_mps, 1.0e-6);
  EXPECT_NEAR(eci_velocity.y_mps, current.velocity.y_mps, 1.0e-6);
  EXPECT_NEAR(eci_velocity.z_mps, current.velocity.z_mps, 1.0e-6);
}

TEST(SatelliteOrbitPropagatorTest, InitializeRejectsInvalidElementsAndKeepsState) {
  satellite_orbit::OrbitPropagator propagator;
  ASSERT_TRUE(propagator.Initialize(CircularEquatorialElements(), 2451545.0));
  const satellite_orbit::EciState baseline = propagator.current_state_eci();

  satellite_orbit::OrbitalElements invalid = CircularEquatorialElements();
  invalid.semi_major_axis_m = 0.0;
  EXPECT_FALSE(propagator.Initialize(invalid, 2451545.0));

  invalid = CircularEquatorialElements();
  invalid.eccentricity = 1.0;
  EXPECT_FALSE(propagator.Initialize(invalid, 2451545.0));

  invalid = CircularEquatorialElements();
  invalid.eccentricity = -0.1;
  EXPECT_FALSE(propagator.Initialize(invalid, 2451545.0));

  invalid = CircularEquatorialElements();
  invalid.inclination_deg = 181.0;
  EXPECT_FALSE(propagator.Initialize(invalid, 2451545.0));

  invalid = CircularEquatorialElements();
  invalid.semi_major_axis_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(propagator.Initialize(invalid, 2451545.0));

  EXPECT_FALSE(propagator.Initialize(CircularEquatorialElements(), 0.0));
  EXPECT_FALSE(propagator.Initialize(
      CircularEquatorialElements(), std::numeric_limits<double>::quiet_NaN()));

  // fail-closed：失败的重新初始化不得覆盖上一有效状态。
  EXPECT_TRUE(propagator.initialized());
  EXPECT_DOUBLE_EQ(propagator.last_propagated_julian_day(), 2451545.0);
  EXPECT_DOUBLE_EQ(propagator.current_state_eci().position.x_m, baseline.position.x_m);
  EXPECT_DOUBLE_EQ(propagator.current_state_eci().velocity.y_mps, baseline.velocity.y_mps);
}

TEST(SatelliteOrbitPropagatorTest, PropagateRejectsBackwardAndNonFiniteTime) {
  satellite_orbit::OrbitPropagator propagator;
  ASSERT_TRUE(propagator.Initialize(CircularEquatorialElements(), 2451545.0));
  satellite_orbit::EcefState ecef;
  ASSERT_TRUE(propagator.PropagateToTime(2451545.0 + 60.0 / kSecondsPerDay, &ecef));
  const double last_jd = propagator.last_propagated_julian_day();

  EXPECT_FALSE(propagator.PropagateToTime(2451545.0, &ecef));
  EXPECT_DOUBLE_EQ(propagator.last_propagated_julian_day(), last_jd);
  EXPECT_FALSE(propagator.PropagateToTime(
      std::numeric_limits<double>::quiet_NaN(), &ecef));
  EXPECT_FALSE(propagator.PropagateToTime(last_jd + 1.0, nullptr));
  EXPECT_DOUBLE_EQ(propagator.last_propagated_julian_day(), last_jd);
}

TEST(SatelliteOrbitPropagatorTest, StepSizeHasPositiveFloorAndIgnoresNonFinite) {
  satellite_orbit::OrbitPropagator propagator;
  propagator.SetStepSize(30.0);
  EXPECT_DOUBLE_EQ(propagator.step_size_sec(), 30.0);
  propagator.SetStepSize(0.0);
  EXPECT_GT(propagator.step_size_sec(), 0.0);
  propagator.SetStepSize(-5.0);
  EXPECT_GT(propagator.step_size_sec(), 0.0);
  const double floored = propagator.step_size_sec();
  propagator.SetStepSize(std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(propagator.step_size_sec(), floored);
}

TEST(SatelliteOrbitPropagatorTest, SameInputSequenceIsDeterministic) {
  satellite_orbit::OrbitPropagator lhs;
  satellite_orbit::OrbitPropagator rhs;
  ASSERT_TRUE(lhs.Initialize(CircularEquatorialElements(), 2451545.0));
  ASSERT_TRUE(rhs.Initialize(CircularEquatorialElements(), 2451545.0));
  lhs.SetStepSize(7.0);
  rhs.SetStepSize(7.0);
  const double target_jd = 2451545.0 + 0.25;
  satellite_orbit::EcefState lhs_ecef;
  satellite_orbit::EcefState rhs_ecef;
  ASSERT_TRUE(lhs.PropagateToTime(target_jd, &lhs_ecef));
  ASSERT_TRUE(rhs.PropagateToTime(target_jd, &rhs_ecef));
  EXPECT_DOUBLE_EQ(lhs_ecef.position.x_m, rhs_ecef.position.x_m);
  EXPECT_DOUBLE_EQ(lhs_ecef.position.y_m, rhs_ecef.position.y_m);
  EXPECT_DOUBLE_EQ(lhs_ecef.position.z_m, rhs_ecef.position.z_m);
  EXPECT_DOUBLE_EQ(lhs_ecef.velocity.x_mps, rhs_ecef.velocity.x_mps);
  EXPECT_DOUBLE_EQ(lhs_ecef.velocity.y_mps, rhs_ecef.velocity.y_mps);
  EXPECT_DOUBLE_EQ(lhs_ecef.velocity.z_mps, rhs_ecef.velocity.z_mps);
}
