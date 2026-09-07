/**
 * @file esr_extension_consumer.cpp
 * @brief 验证安装后 ESR 扩展接口可被外部工程实现并访问。
 *
 * 覆盖要点：
 *   - EsrPipelineAbortReason 公共结果类型可达
 *   - EsrSession 构建、Step、StepWithResult、TryApplyRuntimeConfig
 *   - EsrCycleResult::issues 问题列表可访问
 *
 * 注：环境服务与控制器已内部化，不再支持外部注入；本 consumer 仅验证安装后公共面可达。
 */

#include "1q/electronic_surveillance_radar/config/EsrProfileConstants.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electronic_surveillance_radar/session/EsrOutputTypes.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"

int main() {
  // 1. Session assembly：零值 EsrSessionConfig 不合法（scan_rate/接收频段/检测策略
  // 均为 0 会触发 ValidateEsrSessionConfig），必须使用语义档位常量或显式合法字段。
  using electronic_surveillance_radar::config::profiles::kElectronicOrderOfBattleMission;
  electronic_surveillance_radar::config::EsrSessionConfig config;
  config.mission = kElectronicOrderOfBattleMission;
  config.mission.scan.scan_rate_hz = 1.0f;
  electronic_surveillance_radar::session::EsrSession session =
      electronic_surveillance_radar::session::EsrSession::Create(config);

  // 2. StepWithResult（周期输入要求有限 ECEF 运动学，见 EsrInputValidation）
  electronic_surveillance_radar::session::EsrCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.platform_position_ecef_m.x_m = 6378137.0;
  // 空 RF 帧也必须与周期窗口匹配（world_cycle_index/时间戳三字段，见 EsrInputValidation）。
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = static_cast<double>(input.dt_sec);

  const electronic_surveillance_radar::session::EsrCycleResult result = session.StepWithResult(input);
  if (result.status !=
      electronic_surveillance_radar::session::EsrCycleExecutionStatus::kCompleted) {
    return 1;
  }
  (void)result.output_frame.observation_output.observations.size();
  (void)result.output_frame.emitter_output.hypotheses.size();

  // 3. Step（同样需要合法平台运动学 + 匹配窗口的 RF 帧）
  electronic_surveillance_radar::session::EsrCycleInput input_2;
  input_2.cycle_index = 2U;
  input_2.dt_sec = 1.0f;
  input_2.platform_position_ecef_m.x_m = 6378137.0;
  input_2.rf_emissions.world_cycle_index = input_2.cycle_index;
  input_2.rf_emissions.window_start_time_s = input_2.cycle_start_time_s;
  input_2.rf_emissions.window_duration_s = static_cast<double>(input_2.dt_sec);
  const electronic_surveillance_radar::session::EsrOutputFrame frame = session.Step(input_2);
  (void)frame.cycle_index;

  // 4. Runtime config patch
  electronic_surveillance_radar::config::EsrRuntimeConfigPatch patch;
  patch.has_sensor_enabled = true;
  patch.sensor_enabled = true;
  (void)session.TryApplyRuntimeConfig(patch);

  // 5. Validation access
  const electronic_surveillance_radar::session::EsrIssueList& issues = result.issues;
  (void)issues.size();

  // 6. EsrPipelineAbortReason accessible
  electronic_surveillance_radar::session::EsrPipelineAbortReason abort_reason{
      electronic_surveillance_radar::session::EsrPipelineAbortReason::kNone};
  (void)abort_reason;

  return 0;
}
