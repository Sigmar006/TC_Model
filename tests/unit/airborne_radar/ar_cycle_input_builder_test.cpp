/**
 * @file ar_cycle_input_builder_test.cpp
 * @brief 验证 AR 单周期用户输入的时间、坐标和 interference 合同。
 */

#include <gtest/gtest.h>

#include "1q/airborne_radar/session/ArInputValidation.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"

namespace airborne_radar {
namespace session {
namespace {

ArCycleInput ValidInput() {
  ArCycleInput input;
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

  ArTargetInput target;
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

TEST(ArCycleInputTest, WorldInputsConvertWithoutCallerOwnedLocalState) {
  const ArCycleInput input = ValidInput();
  const ArIssueList issues = ValidateArCycleInput(input);
  EXPECT_FALSE(HasValidationError(issues));
  ASSERT_EQ(input.targets.size(), 1U);
  EXPECT_EQ(input.targets.front().target_id, 7U);
  EXPECT_TRUE(input.interference.emissions.empty());
}

TEST(ArCycleInputTest, NonEmptyInterferenceFrameMustMatchCycleWindow) {
  ArCycleInput input = ValidInput();
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = 99U;
  emission.identity.emission_id = 1U;
  emission.position_ecef_m = input.platform.platform_position_ecef_m;
  emission.position_ecef_m.x_m += 10000.0;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      0.0, 0.5, 10.0e9, 5.0e6, 100.0, &emission.waveform));
  input.interference.world_cycle_index = 2U;
  input.interference.window_start_time_s = 0.0;
  input.interference.window_duration_s = 0.5;
  input.interference.emissions.push_back(emission);

  const ArIssueList issues = ValidateArCycleInput(input);
  ASSERT_TRUE(HasValidationError(issues));
  bool found_mismatch = false;
  for (const ArIssue& issue : issues) {
    found_mismatch = found_mismatch ||
                     issue.code == "ar.validation.interference_frame_mismatch";
  }
  EXPECT_TRUE(found_mismatch);
}

TEST(ArCycleInputTest, EmptyInterferenceFrameNeedsNoModeOrDummyIdentity) {
  ArCycleInput input = ValidInput();
  input.interference = oneq::electromagnetics::RfEmissionFrame{};
  EXPECT_FALSE(HasValidationError(ValidateArCycleInput(input)));
}

TEST(ArCycleInputTest, DuplicateWorldTargetIdsRejectAtomically) {
  ArCycleInput input = ValidInput();
  input.targets.push_back(input.targets.front());
  EXPECT_TRUE(HasValidationError(ValidateArCycleInput(input)));
}

}  // namespace
}  // namespace session
}  // namespace airborne_radar
