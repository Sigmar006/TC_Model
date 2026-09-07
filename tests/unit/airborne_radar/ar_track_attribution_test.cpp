// Copyright 2026. All Rights Reserved.
//
// @file ar_track_attribution_test.cpp
// @brief 验证 AR 航迹归属对照表（信封通道，库内键 ↔ 场景真值目标）。
//
// 覆盖：完成周期归属表与产品导出航迹逐条对应（association_key/external_target_id/
// target_name 一致）、键↔真值映射每条真值恰一次、非执行周期空列表（校验拒绝与
// 关机，五模块统一规则，不复用上一周期）。产品帧上的 external_target_id/target_name
// 为注册 deprecated 遗留（sim-only），本表是权威关联路径（session_contract.md
// Attribution 挂载表）。

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/coordinate/position_transform.h"

namespace airborne_radar {
namespace tests {
namespace {

using airborne_radar::session::ArCycleInput;
using airborne_radar::session::ArCycleResult;
using airborne_radar::session::ArCycleStatus;
using airborne_radar::session::ArPlatformInput;
using airborne_radar::session::ArSession;
using airborne_radar::session::ArTargetInput;

config::ArSessionConfig MakeDetectionFocusedConfig() {
  config::ArSessionConfig cfg;
  cfg.policy.detection = config::profiles::kDetectionPriorityDetection;
  cfg.policy.tracking = config::profiles::kFastAssociationTracking;
  cfg.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
  return cfg;
}

ArPlatformInput MakePlatformInput() {
  oneq::coordinate::EcefPositionM platform_ecef;
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 30.0;
  platform_lla.longitude_deg = 120.0;
  platform_lla.altitude_m = 1000.0;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(platform_lla, &platform_ecef));

  ArPlatformInput platform;
  platform.platform_position_ecef_m = platform_ecef;
  platform.platform_velocity_mps.x_mps = 0.0;
  platform.platform_velocity_mps.y_mps = 0.0;
  platform.platform_velocity_mps.z_mps = 0.0;
  platform.platform_attitude_deg.yaw_deg = 0.0;
  platform.platform_attitude_deg.pitch_deg = 0.0;
  platform.platform_attitude_deg.roll_deg = 0.0;
  return platform;
}

// 静止目标：平台 ENU 直填（零姿态下 ENU ≈ 雷达局部），东北偏上、RCS 充足，
// 保证检测稳定；不同 east 错开方位使两条目标可分辨。
ArTargetInput MakeTarget(std::uint64_t id, const char* name, float east_m) {
  ArTargetInput target;
  target.target_id = id;
  target.target_name = name;
  target.position_x = east_m;
  target.position_y = 8000.0f;
  target.position_z = 500.0f;
  target.rcs = 5.0f;
  target.swerling_type = 2;
  return target;
}

ArCycleInput MakeCycleInput(std::uint32_t cycle, const ArPlatformInput& platform,
                            const std::vector<ArTargetInput>& targets) {
  ArCycleInput input;
  input.cycle_index = cycle;
  input.cycle_start_time_s = static_cast<double>(cycle - 1U) * 0.5;
  input.dt_sec = 0.5;
  input.platform = platform;
  input.targets = targets;
  return input;
}

/// @brief 完成周期：归属表与产品导出航迹逐条对应，两条真值各出现一次。
TEST(ArTrackAttributionTest, AttributionMapsProductTracksToTruthTargets) {
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());
  const ArPlatformInput platform = MakePlatformInput();
  const std::vector<ArTargetInput> targets = {
      MakeTarget(7U, "truth-a", 5000.0f), MakeTarget(8U, "truth-b", 9000.0f)};

  // 运行至两条真值目标的航迹均进入产品帧（导出含 tentative，快确认生命周期；
  // 上限保护，扫描覆盖两个方位需要数个周期）。
  ArCycleResult result;
  for (std::uint32_t cycle = 1U; cycle <= 20U; ++cycle) {
    result = session.StepWithResult(MakeCycleInput(cycle, platform, targets));
    ASSERT_EQ(result.status, ArCycleStatus::kCompleted);
    if (result.output_frame.tracks.size() >= 2U) {
      break;
    }
  }
  ASSERT_GE(result.output_frame.tracks.size(), 2U);
  ASSERT_EQ(result.status, ArCycleStatus::kCompleted);

  // 归属表与产品帧逐条对应（同序、同键、同真值）——信封对照表是产品航迹的
  // "电话簿"，逐条 join 必须精确成立。
  const std::vector<session::ArTrackAttributionRecord>& attributions =
      result.track_attributions;
  ASSERT_EQ(attributions.size(), result.output_frame.tracks.size());
  std::uint64_t mapped_ids = 0U;
  for (std::size_t i = 0U; i < attributions.size(); ++i) {
    EXPECT_EQ(attributions[i].association_key,
              result.output_frame.tracks[i].association_key);
    EXPECT_EQ(attributions[i].external_target_id,
              result.output_frame.tracks[i].external_target_id);
    EXPECT_EQ(attributions[i].target_name, result.output_frame.tracks[i].target_name);
    if (attributions[i].external_target_id != 0U) {
      mapped_ids |= attributions[i].external_target_id;
    }
  }
  EXPECT_EQ(mapped_ids, 7U | 8U);
}

/// @brief 非执行周期空列表：校验拒绝与关机周期均不复用上一周期归属。
TEST(ArTrackAttributionTest, NonExecutedCyclesReturnEmptyAttribution) {
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());
  const ArPlatformInput platform = MakePlatformInput();
  const std::vector<ArTargetInput> targets = {MakeTarget(7U, "truth-a", 5000.0f)};

  const ArCycleResult completed = session.StepWithResult(MakeCycleInput(1U, platform, targets));
  ASSERT_EQ(completed.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(completed.track_attributions.size(), completed.output_frame.tracks.size());
  ASSERT_FALSE(completed.track_attributions.empty());

  // 校验拒绝（dt 非法）：空列表，不推进状态。
  ArCycleInput rejected = MakeCycleInput(2U, platform, targets);
  rejected.dt_sec = 0.0;
  const ArCycleResult rejected_result = session.StepWithResult(rejected);
  EXPECT_EQ(rejected_result.status, ArCycleStatus::kRejectedInvalidInput);
  EXPECT_TRUE(rejected_result.track_attributions.empty());

  // 关机：补丁提交后的非执行周期 → 空列表。
  config::ArRuntimeConfigPatch power_off;
  power_off.has_sensor_enabled = true;
  power_off.sensor_enabled = false;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(power_off));
  const ArCycleResult powered_off =
      session.StepWithResult(MakeCycleInput(3U, platform, targets));
  EXPECT_EQ(powered_off.status, ArCycleStatus::kPoweredOff);
  EXPECT_TRUE(powered_off.track_attributions.empty());
}

}  // namespace
}  // namespace tests
}  // namespace airborne_radar
