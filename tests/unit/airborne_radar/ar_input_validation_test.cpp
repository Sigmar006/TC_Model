// Copyright 2026. All Rights Reserved.
//
// @file ar_input_validation_test.cpp
// @brief 验证 AR 内部目标与单周期用户输入的原子校验合同。

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "1q/airborne_radar/session/ArInputValidation.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"

namespace airborne_radar {
namespace tests {

using session::ArIssue;
using session::ArIssueList;
using session::ArIssueSeverity;
using session::HasValidationError;
using session::ValidateArCycleInput;
using session::ValidateArSceneTargets;

namespace {

session::ArSceneTarget MakeValidLocalTarget(std::uint64_t id = 1U) {
  session::ArSceneTarget target(100.0f, 0.0f, 0.0f, 1.0f);
  target.external_target_id = id;
  target.position_x = 1000.0f;
  target.range_m = 1000.0f;
  return target;
}

session::ArCycleInput MakeValidCycleInput() {
  session::ArCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 0.5;

  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 31.0;
  platform_lla.longitude_deg = 121.0;
  platform_lla.altitude_m = 1000.0;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(
      platform_lla, &input.platform.platform_position_ecef_m));

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

// 统一问题列表（规则 14）：校验 code 形如 "ar.validation.<snake_case>"。
// 这里按 snake 后缀匹配，避免在测试里重复完整前缀。
const session::ArIssue* FindIssue(const session::ArIssueList& issues,
                                  const std::string& snake_code) {
  const std::string expected = std::string("ar.validation.") + snake_code;
  for (const session::ArIssue& issue : issues) {
    if (issue.code == expected) {
      return &issue;
    }
  }
  return nullptr;
}

void AddValidInterferenceEmission(session::ArCycleInput* input) {
  ASSERT_NE(input, nullptr);
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = 99U;
  emission.identity.emission_id = 1U;
  emission.position_ecef_m = input->platform.platform_position_ecef_m;
  emission.position_ecef_m.x_m += 10000.0;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      input->cycle_start_time_s, input->dt_sec, 10.0e9, 5.0e6, 100.0,
      &emission.waveform));
  input->interference.world_cycle_index = input->cycle_index;
  input->interference.window_start_time_s = input->cycle_start_time_s;
  input->interference.window_duration_s = input->dt_sec;
  input->interference.emissions.push_back(emission);
}

}  // namespace

TEST(ArSceneTargetValidationTest, OriginRequiresPositiveRange) {
  session::ArSceneTarget target;
  target.external_target_id = 1U;
  target.rcs = 1.0f;

  const ArIssueList issues = ValidateArSceneTargets({target});
  EXPECT_TRUE(HasValidationError(issues));
  const ArIssue* issue = FindIssue(issues, "missing_range_and_cartesian_position");
  ASSERT_NE(issue, nullptr);
  // 输入校验问题统一 phase=kInputValidation（规则 14b；HasValidationError 依赖该判定）。
  EXPECT_EQ(issue->phase, session::ArIssuePhase::kInputValidation);
}

TEST(ArSceneTargetValidationTest, PositiveRangeAllowsOrigin) {
  session::ArSceneTarget target;
  target.external_target_id = 1U;
  target.range_m = 5000.0f;
  target.rcs = 1.0f;

  const ArIssueList issues = ValidateArSceneTargets({target});
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(ArSceneTargetValidationTest, CartesianPositionAllowsMissingRange) {
  session::ArSceneTarget target;
  target.external_target_id = 1U;
  target.position_x = 3000.0f;
  target.rcs = 1.0f;

  const ArIssueList issues = ValidateArSceneTargets({target});
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(ArSceneTargetValidationTest, NonFiniteFieldsReject) {
  session::ArSceneTarget target = MakeValidLocalTarget();
  target.velocity_y = std::numeric_limits<float>::infinity();

  const ArIssueList issues = ValidateArSceneTargets({target});
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, "non_finite_target_field"), nullptr);
}

TEST(ArSceneTargetValidationTest, DuplicateIdentityRejects) {
  session::ArSceneTarget first = MakeValidLocalTarget(42U);
  session::ArSceneTarget second = MakeValidLocalTarget(42U);
  second.position_x = 2000.0f;
  second.range_m = 2000.0f;

  const ArIssueList issues = ValidateArSceneTargets({first, second});
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, "duplicate_external_target_id"), nullptr);
}

TEST(ArSceneTargetValidationTest, UnknownIdentityIsInformational) {
  const ArIssueList issues = ValidateArSceneTargets({MakeValidLocalTarget(0U)});
  const ArIssue* issue = FindIssue(issues, "unknown_external_target_id");
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, ArIssueSeverity::kInfo);
  EXPECT_EQ(issue->phase, session::ArIssuePhase::kInputValidation);
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(ArSceneTargetValidationTest, NegativeRcsIsWarning) {
  session::ArSceneTarget target = MakeValidLocalTarget();
  target.rcs = -0.5f;

  const ArIssueList issues = ValidateArSceneTargets({target});
  const ArIssue* issue = FindIssue(issues, "negative_rcs");
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, ArIssueSeverity::kWarning);
  EXPECT_EQ(issue->phase, session::ArIssuePhase::kInputValidation);
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(ArCycleInputValidationTest, FullyValidInputProducesNoErrors) {
  EXPECT_FALSE(HasValidationError(ValidateArCycleInput(MakeValidCycleInput())));
}

TEST(ArCycleInputValidationTest, CycleIndexMustBeNonZero) {
  session::ArCycleInput input = MakeValidCycleInput();
  input.cycle_index = 0U;

  const ArIssueList issues = ValidateArCycleInput(input);
  EXPECT_NE(FindIssue(issues, "invalid_cycle_index"), nullptr);
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(ArCycleInputValidationTest, CycleStartMustBeFiniteAndNonNegative) {
  session::ArCycleInput input = MakeValidCycleInput();
  input.cycle_start_time_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), "invalid_cycle_start_time"), nullptr);

  input.cycle_start_time_s = -1.0;
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), "invalid_cycle_start_time"), nullptr);
}

TEST(ArCycleInputValidationTest, CycleDurationMustBeFiniteAndPositive) {
  session::ArCycleInput input = MakeValidCycleInput();
  input.dt_sec = 0.0;
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), "invalid_cycle_delta_time"), nullptr);

  input.dt_sec = -1.0;
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), "invalid_cycle_delta_time"), nullptr);

  input.dt_sec = std::numeric_limits<double>::infinity();
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), "non_finite_cycle_delta_time"), nullptr);
}

TEST(ArCycleInputValidationTest, PlatformWorldKinematicsAreRequired) {
  session::ArCycleInput input = MakeValidCycleInput();
  input.platform.platform_velocity_mps.z_mps =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), "invalid_platform_input"), nullptr);
}

TEST(ArCycleInputValidationTest, InvalidOrDuplicateWorldTargetsReject) {
  session::ArCycleInput input = MakeValidCycleInput();
  input.targets.front().position_x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), "invalid_target_input"), nullptr);

  input = MakeValidCycleInput();
  input.targets.push_back(input.targets.front());
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), "duplicate_external_target_id"), nullptr);
}

TEST(ArCycleInputValidationTest, NonEmptyInterferenceMustMatchCycleWindow) {
  session::ArCycleInput input = MakeValidCycleInput();
  AddValidInterferenceEmission(&input);
  input.interference.world_cycle_index += 1U;

  const ArIssueList issues = ValidateArCycleInput(input);
  EXPECT_NE(FindIssue(issues, "interference_frame_mismatch"), nullptr);
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(ArCycleInputValidationTest, InvalidRfFactsRejectAtomically) {
  session::ArCycleInput input = MakeValidCycleInput();
  AddValidInterferenceEmission(&input);
  input.interference.emissions.front().identity.emission_id = 0U;

  const ArIssueList issues = ValidateArCycleInput(input);
  EXPECT_NE(FindIssue(issues, "invalid_interference_input"), nullptr);
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(ArCycleInputValidationTest, EmptyInterferenceNeedsNoMetadata) {
  session::ArCycleInput input = MakeValidCycleInput();
  input.interference.world_cycle_index = 999U;
  input.interference.window_start_time_s = -10.0;
  input.interference.window_duration_s = -1.0;

  EXPECT_FALSE(HasValidationError(ValidateArCycleInput(input)));
}

}  // namespace tests
}  // namespace airborne_radar
