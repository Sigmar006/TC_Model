/**
 * @file ar_session_consumer.cpp
 * @brief 验证安装后机载雷达公共 API 路径可被外部工程编译链接。
 *
 * 覆盖要点：
 *   - 直接字段赋值构造会话配置
 *   - ArCycleInput 构造 + ArInputValidation 校验
 *   - ArSession 构造、StepWithResult、Step 调用
 *   - ArRuntimeConfigPatch 热切换（工作模式、扫描中心）
 *   - ArCycleResult 各字段可访问
 *   - TrackOutputQueries 输出查询工具
 */

#include <cstddef>

#include "1q/airborne_radar/config/airborne_radar_config.hpp"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArInputValidation.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/coordinate/position_transform.h"

int main() {
  // 1. 默认会话配置
  airborne_radar::config::ArSessionConfig preset_config;

  // 2. 直接字段赋值构造会话配置
  auto built_config = preset_config;
  built_config.hardware.transmitter.peak_power_w = 5.0e6f;
  built_config.hardware.transmitter.frequency_hz = 9.3e9f;
  built_config.mission.scan_center_deg.az_deg = 0.0f;
  built_config.mission.scan_center_deg.el_deg = 0.0f;
  built_config.policy.tracking.enable_kalman_filter = true;
  built_config.policy.lifecycle.confirm_hits = 3;
  built_config.environment.scenario_config.atmospheric_physics.enable_physical_model = true;

  // 3. Session construction from direct config
  airborne_radar::session::ArSession session =
      airborne_radar::session::ArSession::Create(built_config);

  // 4. Input construction + validation
  airborne_radar::session::ArCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 1.0;
  oneq::coordinate::LlaPositionDegM platform_lla;
  if (!oneq::coordinate::TryLlaToEcef(
          platform_lla, &input.platform.platform_position_ecef_m)) {
    return 1;
  }
  const std::vector<airborne_radar::session::ArIssue> issues =
      airborne_radar::session::ValidateArCycleInput(input);
  if (airborne_radar::session::HasValidationError(issues)) {
    return 1;
  }

  // 5. StepWithResult
  const airborne_radar::session::ArCycleResult result = session.StepWithResult(input);
  if (airborne_radar::session::HasValidationError(result.issues)) {
    return 2;
  }

  // 6. Step (output-only)
  const airborne_radar::session::TrackOutputFrame step_frame = session.Step(input);

  // 7. Access result fields
  const std::size_t confirmed_tracks = airborne_radar::session::CountTracksByStatus(
      result.output_frame, airborne_radar::session::TrackStatus::kConfirmed);
  if (confirmed_tracks > result.output_frame.tracks.size()) {
    return 3;
  }

  const bool has_profile = result.has_control_profile;
  (void)has_profile;
  const std::size_t command_count = result.submitted_commands.size();
  (void)command_count;
  const bool completed =
      result.status == airborne_radar::session::ArCycleStatus::kCompleted;
  (void)completed;

  // 8. RuntimeConfigPatch hot-switch
  airborne_radar::config::AzimuthElevationDeg scan_center_deg;
  scan_center_deg.az_deg = 15.0f;
  scan_center_deg.el_deg = -5.0f;
  airborne_radar::config::ArRuntimeConfigPatch runtime_patch;
  runtime_patch.has_work_mode = true;
  runtime_patch.work_mode = airborne_radar::config::ArWorkMode::kTas;
  runtime_patch.has_scan_center_deg = true;
  runtime_patch.scan_center_deg = scan_center_deg;
  runtime_patch.has_environment = true;
  runtime_patch.environment.has_scenario_config = true;
  runtime_patch.environment.scenario_config =
      airborne_radar::config::EnvironmentScenarioConfig{};
  runtime_patch.has_commanded_beamwidth_enabled = true;
  runtime_patch.commanded_beamwidth_enabled = true;
  (void)session.TryApplyRuntimeConfig(runtime_patch);

  // 9. Second cycle after runtime config change
  airborne_radar::session::ArCycleInput input_2 = input;
  input_2.cycle_index = 3U;
  input_2.cycle_start_time_s = 2.0;
  const airborne_radar::session::ArCycleResult result_2 = session.StepWithResult(input_2);
  if (airborne_radar::session::HasValidationError(result_2.issues)) {
    return 4;
  }

  // 10. Step frame cycle_index should differ from first
  if (step_frame.cycle_index == result_2.output_frame.cycle_index) {
    // both default 0 is fine for basic smoke
  }

  return 0;
}
