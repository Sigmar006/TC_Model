// Copyright 2026. All Rights Reserved.
//
// @file rir_emission_frame_test.cpp
// @brief 验证 RIR 公开 API 输出本周期实际发射（与 AR emission_frame 同契约；
// 2026-08-30 核查 9.2：多驻留周期逐驻留记录，单驻留周期仍恰一条）。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/remote_identification_radar/config/RirMissionConfig.h"
#include "1q/remote_identification_radar/config/RirPolicyConfig.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirOutputTypes.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "RirCycleInputTestUtil.h"
#include "remote_identification_radar/runtime/RirController.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using config::RirSessionConfig;
using config::RirWorkMode;
using session::RirCycleInput;
using session::RirCycleStatus;
using session::RirSession;

RirCycleInput MakeInput(std::uint32_t cycle) {
  RirCycleInput input;
  input.input_cycle_index = cycle;
  input.dt_sec = 0.5;
  input.sim_time_sec = static_cast<float>(cycle - 1U) * 0.5f;
  SetDefaultTestPlatformEcef(&input);
  session::RirSceneTarget target;
  target.external_target_id = 9U;
  target.position_x = 8000.0f;
  target.rcs = 2.0f;
  input.scene_targets.push_back(target);
  return input;
}

RirSessionConfig MakeIdentifyConfig() {
  RirSessionConfig session_config;
  session_config.mission.work_mode = RirWorkMode::kIdentify;
  session_config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  return session_config;
}

/// 与发射工厂同源口径：雷达局部 az/el 指向（零姿态）→ ECEF 单位方向。
oneq::coordinate::Vector3d ExpectedBoresightEcef(
    const config::RirAzimuthElevationDeg& pointing,
    const oneq::coordinate::LlaPositionDegM& platform_lla) {
  constexpr double kPi = 3.14159265358979323846;
  const double az_rad = static_cast<double>(pointing.az_deg) * kPi / 180.0;
  const double el_rad = static_cast<double>(pointing.el_deg) * kPi / 180.0;
  const double cos_el = std::cos(el_rad);
  const oneq::coordinate::Vector3d local{cos_el * std::cos(az_rad),
                                         cos_el * std::sin(az_rad), std::sin(el_rad)};
  const oneq::coordinate::Vector3d enu = oneq::coordinate::RotateLocalToEnu(
      local.x, local.y, local.z, oneq::coordinate::EulerAnglesDeg{});
  oneq::coordinate::Vector3d ecef;
  oneq::coordinate::TryEnuToEcefDirection(enu, platform_lla, &ecef);
  return ecef;
}

TEST(RirEmissionFrameTest, CompletedIdentifyCyclePublishesEmissionFrame) {
  const RirSessionConfig session_config = MakeIdentifyConfig();
  RirSession session = RirSession::Create(session_config);
  session.SetSensorEntityId(42U);
  const RirCycleInput input = MakeInput(1U);

  const auto result = session.StepWithResult(input);
  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
  EXPECT_EQ(result.emission_frame.world_cycle_index, 1U);
  EXPECT_DOUBLE_EQ(result.emission_frame.window_start_time_s, 0.0);
  EXPECT_DOUBLE_EQ(result.emission_frame.window_duration_s,
                   static_cast<double>(session_config.mission.recognition_dwell_sec));
  // 缺省配置 dt=0.5s / dwell=0.05s → 每周期 10 条驻留（核查 9.2 前只记首波位
  // 一条；现在逐驻留一条，条数 = 驻留预算摘要的计划驻留数）。
  ASSERT_TRUE(result.has_recognition_summary);
  ASSERT_GT(result.recognition_summary.dwell_budget.scheduled_dwell_count, 0U);
  EXPECT_EQ(result.emission_frame.emissions.size(),
            result.recognition_summary.dwell_budget.scheduled_dwell_count);
  EXPECT_EQ(result.emission_frame.emissions.front().identity.sensor_entity_id, 42U);
  EXPECT_GT(result.emission_frame.emissions.front().waveform.transmit_power_w, 0.0);
}

/// @brief 核查 9.2：多驻留周期逐驻留记录发射——emissions 条数 = 驻留计划
///        条数，逐条 boresight 与该驻留指向对应；identity 帧内唯一；首条
///        emission_id 保持 = 周期号（单驻留行为兼容）。
TEST(RirEmissionFrameTest, MultiDwellCycleRecordsEmissionPerDwell) {
  runtime::RirController controller;
  controller.SetSensorEntityId(42U);
  config::RirMissionConfig mission;
  mission.work_mode = RirWorkMode::kIdentify;
  config::RirPolicyConfig policy;
  policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  controller.UpdateRuntime(mission, policy);

  std::vector<runtime::RirDwellPlan> plan(3U);
  plan[0].pointing_deg = config::RirAzimuthElevationDeg{0.0f, 0.0f};
  plan[1].pointing_deg = config::RirAzimuthElevationDeg{30.0f, 5.0f};
  plan[2].pointing_deg = config::RirAzimuthElevationDeg{-40.0f, 10.0f};

  const RirCycleInput input = MakeInput(1U);
  session::RirOutputFrame frame;
  controller.RunCycle(input, &frame, 1U, plan);

  const oneq::electromagnetics::RfEmissionFrame& emission_frame =
      controller.LatestEmissionFrame();
  ASSERT_EQ(emission_frame.emissions.size(), plan.size())
      << "多驻留周期应逐驻留记录发射，否则下游只见首波位";
  // 帧内 identity 唯一（RfSceneFrame 契约：编排层汇集后整帧校验）。
  EXPECT_TRUE(oneq::electromagnetics::TryValidateRfSceneFrame(emission_frame));
  EXPECT_EQ(emission_frame.emissions.front().identity.emission_id, 1U)
      << "首驻留 emission_id 应保持周期号（单驻留兼容）";
  for (std::size_t i = 0U; i < emission_frame.emissions.size(); ++i) {
    for (std::size_t j = i + 1U; j < emission_frame.emissions.size(); ++j) {
      EXPECT_NE(emission_frame.emissions[i].identity.emission_id,
                emission_frame.emissions[j].identity.emission_id)
          << "逐驻留 emission_id 必须唯一";
    }
  }
  // 逐驻留 boresight：与坐标单源同口径换算该驻留指向的 ECEF 单位方向。
  oneq::coordinate::LlaPositionDegM platform_lla;
  ASSERT_TRUE(oneq::coordinate::TryEcefToLla(input.platform_position, &platform_lla));
  for (std::size_t i = 0U; i < plan.size(); ++i) {
    const oneq::coordinate::Vector3d expected =
        ExpectedBoresightEcef(plan[i].pointing_deg, platform_lla);
    const oneq::electromagnetics::RfSceneDirection& actual =
        emission_frame.emissions[i].antenna.boresight_ecef;
    const double dot = expected.x * actual.x + expected.y * actual.y + expected.z * actual.z;
    EXPECT_NEAR(dot, 1.0, 1e-9) << "驻留 " << i << " 的 boresight 应指向该驻留波位";
  }
}

/// @brief 核查 9.2 会话级：驻留预算 2 条的周期，emission_frame 同步携带 2 条。
TEST(RirEmissionFrameTest, SessionMultiDwellCycleEmissionCountMatchesDwellBudget) {
  RirSessionConfig session_config = MakeIdentifyConfig();
  session_config.mission.recognition_dwell_sec = 0.25f;  // dt=0.5s → 每周期 2 驻留
  RirSession session = RirSession::Create(session_config);

  const auto result = session.StepWithResult(MakeInput(1U));
  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
  ASSERT_EQ(result.recognition_summary.dwell_budget.scheduled_dwell_count, 2U);
  EXPECT_EQ(result.emission_frame.emissions.size(), 2U);
}

TEST(RirEmissionFrameTest, StandbyCycleLeavesEmissionFrameEmpty) {
  RirSessionConfig session_config = MakeIdentifyConfig();
  session_config.mission.work_mode = RirWorkMode::kStby;
  RirSession session = RirSession::Create(session_config);

  const auto result = session.StepWithResult(MakeInput(1U));
  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
  EXPECT_TRUE(result.emission_frame.emissions.empty());
}

TEST(RirEmissionFrameTest, ValidationRejectLeavesEmissionFrameEmpty) {
  RirSession session = RirSession::Create(MakeIdentifyConfig());
  RirCycleInput input = MakeInput(0U);
  input.input_cycle_index = 0U;

  const auto result = session.StepWithResult(input);
  EXPECT_EQ(result.status, RirCycleStatus::kRejectedInvalidInput);
  EXPECT_TRUE(result.emission_frame.emissions.empty());
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
