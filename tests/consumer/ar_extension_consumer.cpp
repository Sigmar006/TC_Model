/**
 * @file ar_extension_consumer.cpp
 * @brief 验证安装后机载雷达步间外部 profile 覆盖接口可被外部工程使用。
 *
 * 本 consumer 演示 StepWithResult -> 构建 ExternalDecisionOverride -> SubmitExternalDecision。
 */

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/DecisionControlTypes.h"

int main() {
  airborne_radar::session::ArSession session =
      airborne_radar::session::ArSession::Create(airborne_radar::config::ArSessionConfig{});

  // 第一步：执行周期，确认会话正常运转。
  airborne_radar::session::ArCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 1.0;
  input.platform.platform_position_ecef_m.x_m = 6378137.0;
  const airborne_radar::session::ArCycleResult first = session.StepWithResult(input);
  if (first.status != airborne_radar::session::ArCycleStatus::kCompleted) {
    return 1;
  }

  // 第二步：构建 profile 覆盖值（整包替换）并提交。
  airborne_radar::session::ExternalDecisionOverride override_decision;
  airborne_radar::session::ArControlProfile profile;
  profile.enable_agility_frequency = true;
  override_decision.profile = profile;
  if (session.SubmitExternalDecision(std::move(override_decision)) !=
      airborne_radar::session::ExternalDecisionSubmitStatus::kAccepted) {
    return 2;
  }

  // 第三步：执行下一周期，验证覆盖已生效。
  // 注意：周期窗口时间戳必须随 cycle_index 前进（PreparedCycleLedger 编年史校验
  // 拒绝 start < 上一周期窗口结束；此前未推进导致周期 2 被 kRejected，覆盖从未被应用）。
  ++input.cycle_index;
  input.cycle_start_time_s += input.dt_sec;
  const airborne_radar::session::ArCycleResult second = session.StepWithResult(input);
  return second.status == airborne_radar::session::ArCycleStatus::kCompleted &&
                 second.applied_decision_source ==
                     airborne_radar::session::DecisionControlSource::kExternal
             ? 0
             : 3;
}
