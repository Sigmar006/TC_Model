// Copyright 2026. All Rights Reserved.
//
// @file rir_self_contained_validation_test.cpp
// @brief 验证阶段 2-S 独立输入面校验：速度/起伏/环境/RF 输入。

#include <gtest/gtest.h>

#include <limits>

#include "1q/electromagnetics/RfScene.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/config/RirSessionConfigValidation.h"
#include "1q/remote_identification_radar/session/RirInputValidation.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "RirCycleInputTestUtil.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleInput;
using session::RirSceneTarget;
using session::ValidateRirCycleInput;

RirCycleInput MakeValidInput() {
  RirCycleInput input;
  input.input_cycle_index = 1U;
  input.dt_sec = 0.5;
  input.sim_time_sec = 0.0f;
  SetDefaultTestPlatformEcef(&input);
  RirSceneTarget target;
  target.external_target_id = 7U;
  target.position_x = 5000.0f;
  target.rcs = 1.0f;
  input.scene_targets.push_back(target);
  return input;
}

bool HasCode(const session::RirIssueList& issues, const char* code) {
  for (const session::RirIssue& issue : issues) {
    if (issue.code == code) {
      return true;
    }
  }
  return false;
}

oneq::electromagnetics::RfSceneEmission MakeNoiseEmission(std::uint64_t emission_id) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = 100U + emission_id;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m.x_m = 1000.0;
  emission.antenna.boresight_ecef.x = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(0.0, 1.0, 3.0e9, 20.0e6, 10.0,
                                                               &emission.waveform));
  return emission;
}

TEST(RirSelfContainedValidationTest, PolarizationHasFlagsRequireFiniteOptionalFields) {
  RirCycleInput input = MakeValidInput();
  session::RirPolSMatrixSample sample;
  sample.hv_amp_db = std::numeric_limits<float>::quiet_NaN();
  input.scene_targets[0].polarization_samples.push_back(sample);
  EXPECT_TRUE(HasCode(ValidateRirCycleInput(input), session::codes::kNonFiniteTargetField));

  input = MakeValidInput();
  sample = session::RirPolSMatrixSample{};
  sample.vv_phase_deg = std::numeric_limits<float>::infinity();
  input.scene_targets[0].polarization_samples.push_back(sample);
  EXPECT_TRUE(HasCode(ValidateRirCycleInput(input), session::codes::kNonFiniteTargetField));
}

TEST(RirSelfContainedValidationTest, SceneTargetMotionAndSwerlingFieldsAreValidated) {
  RirCycleInput input = MakeValidInput();
  input.scene_targets[0].velocity_y = std::numeric_limits<float>::infinity();
  const auto issues = ValidateRirCycleInput(input);
  EXPECT_TRUE(HasCode(issues, session::codes::kNonFiniteTargetField));

  input = MakeValidInput();
  input.scene_targets[0].target_swerling_type = static_cast<session::RirSwerlingType>(9);
  const auto swerling_issues = ValidateRirCycleInput(input);
  EXPECT_TRUE(HasCode(swerling_issues, session::codes::kInvalidTargetMotionField));
}

TEST(RirSelfContainedValidationTest, EnvironmentAndRfSceneInputsAreValidated) {
  config::RirSessionConfig session_config;
  session_config.environment.weather_attenuation_db = -1.0f;
  const auto env_issues = config::ValidateRirSessionConfig(session_config);
  EXPECT_TRUE(HasCode(env_issues, session::codes::kInvalidEnvironmentSnapshot));

  RirCycleInput input = MakeValidInput();
  input.rf_scene.world_cycle_index = 1U;
  input.rf_scene.window_start_time_s = 0.0;
  input.rf_scene.window_duration_s = session_config.mission.recognition_dwell_sec;
  input.rf_scene.emissions.push_back(MakeNoiseEmission(1U));
  input.rf_scene.emissions.push_back(MakeNoiseEmission(1U));
  const auto invalid_scene_issues =
      ValidateRirCycleInput(input, session_config.mission.recognition_dwell_sec);
  EXPECT_TRUE(HasCode(invalid_scene_issues, session::codes::kInvalidRfSceneFrame));

  input = MakeValidInput();
  input.rf_scene.emissions.push_back(MakeNoiseEmission(2U));
  input.rf_scene.window_start_time_s = 1.0;
  const auto window_issues =
      ValidateRirCycleInput(input, session_config.mission.recognition_dwell_sec);
  EXPECT_TRUE(HasCode(window_issues, session::codes::kInvalidRfSceneFrame));
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
