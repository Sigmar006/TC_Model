/**
 * @file sbirs_session_orbit_test.cpp
 * @brief 会话级卫星轨道传播测试：配置驱动建立传播器、ECEF 状态注入周期输入、
 *        跨周期推进与 fail-closed 语义。
 */

#include <gtest/gtest.h>

#include <cmath>

#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsInputValidation.h"
#include "1q/sbirs_sensor/session/SbirsIssueCodes.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"

namespace {

using sbirs_sensor::config::SbirsSessionConfig;
using sbirs_sensor::session::SbirsCycleInput;
using sbirs_sensor::session::SbirsCycleInputBuilder;
using sbirs_sensor::session::SbirsEulerAnglesDeg;
using sbirs_sensor::session::SbirsIssueList;
using sbirs_sensor::session::SbirsSession;
using sbirs_sensor::session::SbirsVector3M;

constexpr double kEpochJd = 2451545.0;   // J2000
constexpr double kSemiMajorAxisM = 7000000.0;

/** @brief 基准会话配置（与既有会话测试同口径），卫星状态由调用方提供。 */
SbirsSessionConfig BaseConfig() {
  SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.hardware.integration_time_sec = 1.0f;
  config.mission.scan_start_az_deg = -1.0f;
  config.mission.scan_span_deg = 11.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  return config;
}

/** @brief 在基准会话配置上启用赤道圆轨道（e=0，i=0，M=0）。 */
SbirsSessionConfig OrbitConfig() {
  SbirsSessionConfig config = BaseConfig();
  config.orbit.propagation_enabled = true;
  config.orbit.orbital_elements.semi_major_axis_m = kSemiMajorAxisM;
  config.orbit.orbital_elements.eccentricity = 0.0;
  config.orbit.orbital_elements.inclination_deg = 0.0;
  config.orbit.orbital_elements.raan_deg = 0.0;
  config.orbit.orbital_elements.argument_of_perigee_deg = 0.0;
  config.orbit.orbital_elements.mean_anomaly_deg = 0.0;
  config.orbit.epoch_utc_julian_day = kEpochJd;
  config.orbit.propagation_step_sec = 10.0;
  return config;
}

/** @brief 输入卫星字段的哨兵值，用于验证失败路径不修改入参。 */
SbirsCycleInput SentinelInput() {
  SbirsCycleInput input;
  input.utc_julian_day = 123.5;
  input.satellite_position_ecef_m = SbirsVector3M(1.0, 2.0, 3.0);
  input.satellite_velocity_ecef_m_per_s = SbirsVector3M(4.0, 5.0, 6.0);
  return input;
}

double Norm(const SbirsVector3M& vector) {
  return std::sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

bool HasCode(const SbirsIssueList& issues, const char* code) {
  for (const auto& issue : issues) {
    if (issue.code == code) {
      return true;
    }
  }
  return false;
}

TEST(SbirsSessionOrbitTest, DisabledPropagationKeepsInputUntouched) {
  SbirsSession session = SbirsSession::Create(BaseConfig());
  SbirsCycleInput input = SentinelInput();
  EXPECT_FALSE(session.PropagateSatelliteState(kEpochJd + 1.0, &input));
  EXPECT_DOUBLE_EQ(input.utc_julian_day, 123.5);
  EXPECT_DOUBLE_EQ(input.satellite_position_ecef_m.x, 1.0);
}

TEST(SbirsSessionOrbitTest, EnabledPropagationWritesSatelliteStateAtEpoch) {
  SbirsSession session = SbirsSession::Create(OrbitConfig());
  SbirsCycleInput input;
  ASSERT_TRUE(session.PropagateSatelliteState(kEpochJd, &input));
  EXPECT_DOUBLE_EQ(input.utc_julian_day, kEpochJd);
  // 历元推进量为零：输出即历元 ECI 状态按 GMST 旋转，半径等于半长轴。
  EXPECT_NEAR(Norm(input.satellite_position_ecef_m), kSemiMajorAxisM, 1.0);
}

TEST(SbirsSessionOrbitTest, EnabledPropagationAdvancesAcrossCycles) {
  SbirsSession session = SbirsSession::Create(OrbitConfig());
  SbirsCycleInput first;
  ASSERT_TRUE(session.PropagateSatelliteState(kEpochJd + 0.25, &first));
  SbirsCycleInput second;
  ASSERT_TRUE(session.PropagateSatelliteState(kEpochJd + 0.5, &second));
  EXPECT_DOUBLE_EQ(second.utc_julian_day, kEpochJd + 0.5);
  // 圆轨道半径在 J2 摄动下仅有界漂移，量级仍为半长轴。
  EXPECT_NEAR(Norm(first.satellite_position_ecef_m), kSemiMajorAxisM, 0.02 * kSemiMajorAxisM);
  EXPECT_NEAR(Norm(second.satellite_position_ecef_m), kSemiMajorAxisM, 0.02 * kSemiMajorAxisM);
  // 四分之一天的传播量足以让位置显著变化（圆轨道周期约 97 min）。
  const double delta = std::sqrt(
      std::pow(second.satellite_position_ecef_m.x - first.satellite_position_ecef_m.x, 2.0) +
      std::pow(second.satellite_position_ecef_m.y - first.satellite_position_ecef_m.y, 2.0) +
      std::pow(second.satellite_position_ecef_m.z - first.satellite_position_ecef_m.z, 2.0));
  EXPECT_GT(delta, 1.0e5);
  // 速度量级接近圆轨道速度 sqrt(mu/a)，ECEF 速度含地球自转输运项。
  EXPECT_NEAR(Norm(second.satellite_velocity_ecef_m_per_s), 7546.0, 800.0);
}

TEST(SbirsSessionOrbitTest, BackwardTargetRejectedWithoutCorruptingState) {
  SbirsSession session = SbirsSession::Create(OrbitConfig());
  SbirsCycleInput forward;
  ASSERT_TRUE(session.PropagateSatelliteState(kEpochJd + 1.0, &forward));
  SbirsCycleInput backward = SentinelInput();
  EXPECT_FALSE(session.PropagateSatelliteState(kEpochJd + 0.5, &backward));
  EXPECT_DOUBLE_EQ(backward.utc_julian_day, 123.5);
  EXPECT_DOUBLE_EQ(backward.satellite_position_ecef_m.y, 2.0);
  // 失败不推进状态：仍可继续向更晚时刻传播。
  SbirsCycleInput resumed;
  EXPECT_TRUE(session.PropagateSatelliteState(kEpochJd + 2.0, &resumed));
  EXPECT_DOUBLE_EQ(resumed.utc_julian_day, kEpochJd + 2.0);
}

TEST(SbirsSessionOrbitTest, NullInputRejected) {
  SbirsSession session = SbirsSession::Create(OrbitConfig());
  EXPECT_FALSE(session.PropagateSatelliteState(kEpochJd, nullptr));
}

TEST(SbirsSessionOrbitTest, InvalidOrbitElementsBuildNoPropagator) {
  SbirsSessionConfig config = OrbitConfig();
  config.orbit.orbital_elements.eccentricity = 1.5;  // 非椭球轨道
  SbirsSession session = SbirsSession::Create(config);
  SbirsCycleInput input;
  EXPECT_FALSE(session.PropagateSatelliteState(kEpochJd, &input));
}

TEST(SbirsSessionOrbitTest, ValidationReportsOrbitIssues) {
  SbirsSessionConfig valid = OrbitConfig();
  SbirsIssueList issues;
  SbirsSession::CreateWithDiagnostics(valid, &issues);
  EXPECT_FALSE(HasCode(issues, sbirs_sensor::session::codes::kInvalidOrbitElements));
  EXPECT_FALSE(HasCode(issues, sbirs_sensor::session::codes::kInvalidOrbitEpoch));
  EXPECT_FALSE(HasCode(issues, sbirs_sensor::session::codes::kInvalidOrbitPropagationStep));

  SbirsSessionConfig bad_elements = OrbitConfig();
  bad_elements.orbit.orbital_elements.semi_major_axis_m = -1.0;
  SbirsSession::CreateWithDiagnostics(bad_elements, &issues);
  EXPECT_TRUE(HasCode(issues, sbirs_sensor::session::codes::kInvalidOrbitElements));

  SbirsSessionConfig bad_epoch = OrbitConfig();
  bad_epoch.orbit.epoch_utc_julian_day = 0.0;
  SbirsSession::CreateWithDiagnostics(bad_epoch, &issues);
  EXPECT_TRUE(HasCode(issues, sbirs_sensor::session::codes::kInvalidOrbitEpoch));

  SbirsSessionConfig bad_step = OrbitConfig();
  bad_step.orbit.propagation_step_sec = 0.0;
  SbirsSession::CreateWithDiagnostics(bad_step, &issues);
  EXPECT_TRUE(HasCode(issues, sbirs_sensor::session::codes::kInvalidOrbitPropagationStep));
}

TEST(SbirsSessionOrbitTest, DisabledPropagationSkipsOrbitValidation) {
  SbirsSessionConfig config = OrbitConfig();
  config.orbit.propagation_enabled = false;
  config.orbit.orbital_elements.eccentricity = 2.0;
  config.orbit.epoch_utc_julian_day = 0.0;
  config.orbit.propagation_step_sec = 0.0;
  SbirsIssueList issues;
  SbirsSession::CreateWithDiagnostics(config, &issues);
  EXPECT_FALSE(HasCode(issues, sbirs_sensor::session::codes::kInvalidOrbitElements));
  EXPECT_FALSE(HasCode(issues, sbirs_sensor::session::codes::kInvalidOrbitEpoch));
  EXPECT_FALSE(HasCode(issues, sbirs_sensor::session::codes::kInvalidOrbitPropagationStep));
}

TEST(SbirsSessionOrbitTest, PropagatedInputExecutesCycle) {
  SbirsSession session = SbirsSession::Create(OrbitConfig());
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.target_name = "hot";
  target.position_ecef_m = SbirsVector3M(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e8;
  SbirsCycleInput input = SbirsCycleInputBuilder()
                              .WithCycleIndex(1U)
                              .WithDeltaTimeSec(1.0f)
                              .WithSatelliteAttitude(SbirsEulerAnglesDeg{})
                              .AddTarget(target)
                              .Build();
  ASSERT_TRUE(session.PropagateSatelliteState(kEpochJd, &input));
  const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(input);
  EXPECT_FALSE(sbirs_sensor::session::HasValidationError(result.issues));
  EXPECT_EQ(result.status, sbirs_sensor::session::SbirsCycleStatus::kCompleted);
}

}  // namespace
