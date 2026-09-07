/**
 * @file esr_session_consumer.cpp
 * @brief 验证安装后 ESR 公共 API 路径可被外部工程编译链接。
 *
 * 覆盖要点：
 *   - EsrSessionConfig 语义档位常量构造会话配置
 *   - 直接字段赋值构造详细会话配置
 *   - EsrCycleInput + RfEmissionFrame 构造周期输入
 *   - EsrInputValidation 输入校验
 *   - EsrSession 构造、Step、StepWithResult 调用
 *   - EsrOutputFrame 的观测与假设输出字段可访问
 *   - EsrRuntimeConfigPatch 热切换（传感器开关、扫描率、接收窗、检测门限）
 */

#include <cstddef>
#include <string>

#include "1q/electronic_surveillance_radar/config/EsrProfileConstants.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"

namespace esr = electronic_surveillance_radar;

int main() {
  // 1. 语义档位常量整域赋值
  //    （kStandard 灵敏度为 no-op，已随 Profile 枚举删除）
  esr::config::EsrSessionConfig config;
  config.mission = esr::config::profiles::kElectronicOrderOfBattleMission;
  config.mission.scan.scan_rate_hz = 1.0f;

  // 2. 直接字段赋值构造详细会话配置
  esr::config::EsrSessionConfig detailed_config{};
  detailed_config.mission.work_mode = esr::config::EsrWorkMode::kEsm;
  detailed_config.mission.scan.scan_rate_hz = 2.0f;
  detailed_config.policy.detection.minimum_snr_db = 8.0f;
  detailed_config.policy.detection.pfa = 1.0e-6f;
  detailed_config.policy.detection.pulse_count = 16U;
  detailed_config.policy.detection.threshold_scale = 1.0f;
  detailed_config.policy.detection.enable_statistical_detection = true;
  detailed_config.environment.scenario_config.preset = esr::config::EsrEnvironmentPreset::kStandard;

  // 3. Session construction
  auto session = esr::session::EsrSession::Create(config);

  // 4. CycleInput with an empty, valid RF frame.
  esr::session::EsrCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;

  // 5. Input validation
  const esr::session::EsrIssueList issues = esr::session::ValidateEsrCycleInput(input);
  if (esr::session::HasValidationError(issues)) {
    return 1;
  }

  // 6. StepWithResult
  const esr::session::EsrCycleResult result = session.StepWithResult(input);
  if (esr::session::HasValidationError(result.issues)) {
    return 2;
  }

  // 7. Step (output-only)
  const esr::session::EsrOutputFrame step_frame = session.Step(input);

  // 8. Access sensor-facing output.
  const std::size_t obs_count = result.output_frame.observation_output.observations.size();
  const std::size_t hyp_count = result.output_frame.emitter_output.hypotheses.size();
  const std::uint32_t cycle_index = result.output_frame.cycle_index;
  (void)obs_count;
  (void)hyp_count;
  (void)cycle_index;

  // 9. RuntimeConfigPatch: disable sensor
  esr::config::EsrRuntimeConfigPatch disable_patch;
  disable_patch.has_sensor_enabled = true;
  disable_patch.sensor_enabled = false;
  (void)session.TryApplyRuntimeConfig(disable_patch);

  // 10. Step after sensor disabled — should return empty output
  esr::session::EsrCycleInput input_2 = input;
  input_2.cycle_index = 2U;
  input_2.rf_emissions.world_cycle_index = input_2.cycle_index;
  const esr::session::EsrOutputFrame disabled_frame = session.Step(input_2);
  if (!disabled_frame.observation_output.observations.empty()) {
    return 3;
  }

  // 11. RuntimeConfigPatch: re-enable sensor
  esr::config::EsrRuntimeConfigPatch enable_patch;
  enable_patch.has_sensor_enabled = true;
  enable_patch.sensor_enabled = true;
  (void)session.TryApplyRuntimeConfig(enable_patch);

  // 12. RuntimeConfigPatch: scan rate + work mode
  esr::config::EsrRuntimeConfigPatch tune_patch;
  tune_patch.has_scan_rate_hz = true;
  tune_patch.scan_rate_hz = 2.0f;
  tune_patch.has_work_mode = true;
  tune_patch.work_mode = esr::config::EsrWorkMode::kHgesm;
  (void)session.TryApplyRuntimeConfig(tune_patch);

  // 13. RuntimeConfigPatch: explicit scan bounds
  esr::config::EsrRuntimeConfigPatch window_patch;
  window_patch.has_explicit_scan_bounds = true;
  window_patch.explicit_scan_bounds.enabled = true;
  window_patch.explicit_scan_bounds.scan_start_az_deg = -45.0f;
  window_patch.explicit_scan_bounds.scan_end_az_deg = 45.0f;
  window_patch.explicit_scan_bounds.scan_start_el_deg = -15.0f;
  window_patch.explicit_scan_bounds.scan_end_el_deg = 15.0f;
  (void)session.TryApplyRuntimeConfig(window_patch);

  // 14. RuntimeConfigPatch: reset to center-driven scan
  esr::config::EsrRuntimeConfigPatch clear_window_patch;
  clear_window_patch.has_explicit_scan_bounds = true;
  clear_window_patch.explicit_scan_bounds.enabled = false;
  (void)session.TryApplyRuntimeConfig(clear_window_patch);

  // 15. RuntimeConfigPatch: environment atmospheric config via environment runtime config
  esr::config::EsrAtmosphericPhysicsConfig atmospheric_physics;
  atmospheric_physics.enable_physical_model = true;
  atmospheric_physics.relative_humidity = 0.7f;
  esr::config::EsrRuntimeConfigPatch env_patch;
  env_patch.has_environment = true;
  env_patch.environment.has_atmospheric_physics = true;
  env_patch.environment.atmospheric_physics = atmospheric_physics;
  (void)session.TryApplyRuntimeConfig(env_patch);

  // 16. Final cycle
  esr::session::EsrCycleInput input_3 = input;
  input_3.cycle_index = 3U;
  input_3.rf_emissions.world_cycle_index = input_3.cycle_index;
  const esr::session::EsrCycleResult result_3 = session.StepWithResult(input_3);
  if (esr::session::HasValidationError(result_3.issues)) {
    return 4;
  }

  return 0;
}
