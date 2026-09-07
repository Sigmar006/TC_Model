// Copyright 2026. All Rights Reserved.
//
// @file ar_three_write_guard_test.cpp
// @brief AR 三写约束机制性守卫（session_contract.md 规则 9）。
//
// 任何 abort 路径（abort_reason 非 kNone）必须同时写：
//   1) abort_reason（结构化信号）
//   2) diagnostics（结构化诊断：severity + code + message，code 带模块前缀）
//   3) PROJECT_LOG（人读日志，由统一 RecordAbort 写入；测试不解析日志文本，规则 3）
// 并保证 status 与 abort 类别一致（validation → kRejectedInvalidInput，
// powered-off → kPoweredOff，execution → kRejectedExecution）。
//
// 守卫对所有模块统一形态；新增 abort 路径时必须通过本测试。

#include <gtest/gtest.h>

#include <cstdint>

#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArInputValidation.h"
#include "1q/airborne_radar/session/ArOutputTypes.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"

namespace airborne_radar {
namespace tests {
namespace {

config::ArSessionConfig MakeSessionConfig() {
  config::ArSessionConfig cfg;
  cfg.policy.detection = config::profiles::kDetectionPriorityDetection;
  cfg.policy.tracking = config::profiles::kFastAssociationTracking;
  cfg.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
  return cfg;
}

session::ArCycleInput MakeInput(std::uint32_t cycle_index = 1U, double cycle_start_time_s = 0.0) {
  session::ArCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = cycle_start_time_s;
  input.dt_sec = 0.5;

  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 31.0;
  platform_lla.longitude_deg = 121.0;
  platform_lla.altitude_m = 1000.0;
  EXPECT_TRUE(
      oneq::coordinate::TryLlaToEcef(platform_lla, &input.platform.platform_position_ecef_m));

  // 世界 ECEF → 平台锚点 ENU（公共一站式入口）。
  oneq::coordinate::LlaPositionDegM anchor_lla;
  EXPECT_TRUE(oneq::coordinate::TryEcefToLla(input.platform.platform_position_ecef_m, &anchor_lla));
  oneq::coordinate::ExternalKinematics world_kinematics;
  world_kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  world_kinematics.position_ecef_m = input.platform.platform_position_ecef_m;
  world_kinematics.position_ecef_m.x_m += 5000.0;
  oneq::coordinate::EnuSceneState enu;
  EXPECT_TRUE(oneq::coordinate::TryMakeEnuSceneState(world_kinematics, anchor_lla, &enu));

  session::ArTargetInput target;
  target.target_id = 7U;
  target.target_name = "guard-target";
  target.position_x = static_cast<float>(enu.position_enu_m.east_m);
  target.position_y = static_cast<float>(enu.position_enu_m.north_m);
  target.position_z = static_cast<float>(enu.position_enu_m.up_m);
  target.velocity_x = static_cast<float>(enu.velocity_enu_mps.east_mps);
  target.velocity_y = static_cast<float>(enu.velocity_enu_mps.north_mps);
  target.velocity_z = static_cast<float>(enu.velocity_enu_mps.up_mps);
  target.rcs = 2.0f;
  input.targets.push_back(target);
  return input;
}

// 三写守卫断言：abort 周期必须三写齐备且 status 与 abort 类别一致。
void ExpectThreeWriteAbort(const session::ArCycleResult& result,
                           session::SignalCycleAbortReason expected_reason,
                           session::ArCycleStatus expected_status) {
  EXPECT_NE(result.status, session::ArCycleStatus::kCompleted);
  EXPECT_EQ(result.abort_reason, expected_reason);
  EXPECT_EQ(result.status, expected_status);

  // 写二：结构化诊断中至少一条 error 级、code 带模块前缀
  // （info/warning 级伴随诊断合法，三写要求的是 error 级主诊断）。
  EXPECT_FALSE(result.issues.empty());
  bool saw_error_diagnostic = false;
  for (const auto& issue : result.issues) {
    EXPECT_FALSE(issue.code.empty());
    EXPECT_EQ(issue.code.compare(0, 3, "ar."), 0);
    if (issue.severity == session::ArIssueSeverity::kError) {
      saw_error_diagnostic = true;
    }
  }
  EXPECT_TRUE(saw_error_diagnostic);
  // 写三：人读日志由统一 RecordAbort（PROJECT_LOG_ERROR）保证，测试不解析日志文本（规则 3）。
}

// 规则 14 断言：issues 列表中存在指定 phase 的条目。
bool ContainsPhase(const session::ArIssueList& issues, session::ArIssuePhase phase) {
  for (const auto& issue : issues) {
    if (issue.phase == phase) {
      return true;
    }
  }
  return false;
}

TEST(ArThreeWriteGuardTest, ValidationAbortWritesAllThree) {
  session::ArSession radar = session::ArSession::Create(MakeSessionConfig());

  session::ArCycleInput invalid = MakeInput();
  invalid.dt_sec = 0.0;
  const session::ArCycleResult result = radar.StepWithResult(invalid);
  ExpectThreeWriteAbort(result, session::SignalCycleAbortReason::kValidationRejected,
                        session::ArCycleStatus::kRejectedInvalidInput);
  // 规则 14：校验拒绝的问题条目 phase=kInputValidation（校验问题本身就是 error 级诊断，
  // code 形如 "ar.validation.<snake>" 也以 "ar." 开头）。
  EXPECT_TRUE(ContainsPhase(result.issues, session::ArIssuePhase::kInputValidation));
  EXPECT_TRUE(session::HasValidationError(result.issues));
}

TEST(ArThreeWriteGuardTest, PoweredOffAbortWritesAllThree) {
  session::ArSession radar = session::ArSession::Create(MakeSessionConfig());
  config::ArRuntimeConfigPatch power_off;
  power_off.has_sensor_enabled = true;
  power_off.sensor_enabled = false;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(power_off));

  const session::ArCycleResult result = radar.StepWithResult(MakeInput());
  ExpectThreeWriteAbort(result, session::SignalCycleAbortReason::kSensorPoweredOff,
                        session::ArCycleStatus::kPoweredOff);
  // 规则 14：关机属运行态条件，中止条目 phase=kExecution。
  EXPECT_TRUE(ContainsPhase(result.issues, session::ArIssuePhase::kExecution));
}

}  // namespace
}  // namespace tests
}  // namespace airborne_radar
