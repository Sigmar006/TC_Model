/**
 * @file eos_session_create_test.cpp
 * @brief 验证 EOS 会话创建的默认装配路径契约。
 * @note 管线与环境服务已完全内部化，仅保留 Create() 路径。
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "electro_optical_sensor/runtime/EosController.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosSession.h"

namespace electro_optical_sensor {
namespace session {
namespace {

config::EosSessionConfig MakeSessionConfig() {
  config::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;
  config.mission.scan_rate_deg_per_sec = 5.0f;
  config.mission.horizontal_fov_deg = 20.0f;
  config.mission.vertical_fov_deg = 4.0f;
  return config;
}

EosCycleInput MakeValidInput(std::uint32_t cycle_index) {
  EosCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 0.1f;
  return input;
}

TEST(EosSessionCreateTest, CreateUsesDefaultPipelineAndProducesResult) {
  EosSession session = EosSession::Create(MakeSessionConfig());

  EosCycleInput input = MakeValidInput(10U);
  input.scene.clear();

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(HasValidationError(result.issues));
  EXPECT_EQ(result.status, EosCycleStatus::kCompleted);
  EXPECT_EQ(result.output_frame.cycle_index, 10U);
  EXPECT_TRUE(result.output_frame.detections.empty());
}

TEST(EosSessionCreateTest, SetSensorEntityIdCanBeSetAndRestored) {
  EosSession session = EosSession::Create(MakeSessionConfig());
  session.SetSensorEntityId(103U);

  EosCycleInput input = MakeValidInput(1U);
  input.scene.clear();
  EosCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, EosCycleStatus::kCompleted);

  // Restore with sentinel 0
  session.SetSensorEntityId(0U);
  input.cycle_index = 2U;
  result = session.StepWithResult(input);
  EXPECT_EQ(result.status, EosCycleStatus::kCompleted);
}

}  // namespace
}  // namespace session
}  // namespace electro_optical_sensor
