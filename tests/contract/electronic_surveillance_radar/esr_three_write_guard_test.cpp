// Copyright 2026. All Rights Reserved.
//
// @file esr_three_write_guard_test.cpp
// @brief ESR 三写约束机制性守卫（session_contract.md 规则 9 + 统一问题列表模型规则 14）。
//
// 任何 abort 路径（abort_reason 非 kNone）必须同时写：
//   1) abort_reason（结构化信号）
//   2) issues（结构化诊断：severity + phase + code + message，code 带模块前缀）
//   3) PROJECT_LOG（人读日志，由统一 RecordAbort 写入；测试不解析日志文本，规则 3）
// 并保证 status 与 abort 类别一致（validation → kRejected，powered-off → kPoweredOff；
// ESR 的 EsrCycleExecutionStatus 无 InvalidInput/Execution 细分，统一为 kRejected）。
// 校验拒绝时校验问题本身就是 error 级诊断（phase=kInputValidation），不再附加粗粒度条目。
//
// 守卫对所有模块统一形态；新增 abort 路径时必须通过本测试。

#include <gtest/gtest.h>

#include <cstdint>

#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrOutputTypes.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"

namespace electronic_surveillance_radar {
namespace tests {
namespace {

namespace esr_session = ::electronic_surveillance_radar::session;

session::EsrCycleInput MakeValidInput() {
  session::EsrCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 10.0;
  input.dt_sec = 1.0f;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  return input;
}

// 三写守卫断言：abort 周期必须三写齐备且 status 与 abort 类别一致。
void ExpectThreeWriteAbort(const session::EsrCycleResult& result,
                           session::EsrPipelineAbortReason expected_reason,
                           session::EsrCycleExecutionStatus expected_status) {
  EXPECT_NE(result.status, session::EsrCycleExecutionStatus::kCompleted);
  EXPECT_EQ(result.abort_reason, expected_reason);
  EXPECT_EQ(result.status, expected_status);

  // 写二：结构化诊断中至少一条 error 级、code 带模块前缀
  // （info/warning 级伴随诊断合法，三写要求的是 error 级主诊断）。
  EXPECT_FALSE(result.issues.empty());
  bool saw_error_diagnostic = false;
  for (const auto& issue : result.issues) {
    EXPECT_FALSE(issue.code.empty());
    EXPECT_EQ(issue.code.compare(0, 4, "esr."), 0);
    if (issue.severity == session::EsrIssueSeverity::kError) {
      saw_error_diagnostic = true;
    }
  }
  EXPECT_TRUE(saw_error_diagnostic);
  // 写三：人读日志由统一 RecordAbort（PROJECT_LOG_ERROR）保证，测试不解析日志文本（规则 3）。
}

// 规则 14 断言：校验拒绝时错误条目 phase=kInputValidation；执行中止时 phase=kExecution。
bool ContainsPhase(const session::EsrIssueList& issues, session::EsrIssuePhase phase) {
  for (const auto& issue : issues) {
    if (issue.phase == phase) {
      return true;
    }
  }
  return false;
}

TEST(EsrThreeWriteGuardTest, ValidationAbortWritesAllThree) {
  session::EsrSession session = session::EsrSession::Create();

  session::EsrCycleInput invalid_input = MakeValidInput();
  invalid_input.cycle_index = 2U;
  invalid_input.dt_sec = 0.0f;

  const session::EsrCycleResult result = session.StepWithResult(invalid_input);
  ExpectThreeWriteAbort(result, session::EsrPipelineAbortReason::kValidationRejected,
                        session::EsrCycleExecutionStatus::kRejected);
  // 规则 14：校验拒绝的问题条目 phase=kInputValidation（校验问题本身就是 error 级诊断）。
  EXPECT_TRUE(ContainsPhase(result.issues, session::EsrIssuePhase::kInputValidation));
  EXPECT_TRUE(session::HasValidationError(result.issues));
}

TEST(EsrThreeWriteGuardTest, PoweredOffAbortWritesAllThree) {
  session::EsrSession session = session::EsrSession::Create();
  const session::EsrCycleResult active = session.StepWithResult(MakeValidInput());
  ASSERT_EQ(active.status, session::EsrCycleExecutionStatus::kCompleted);

  config::EsrRuntimeConfigPatch power_off;
  power_off.has_sensor_enabled = true;
  power_off.sensor_enabled = false;
  (void)session.TryApplyRuntimeConfig(power_off);

  // rf_emissions.world_cycle_index 必须与 cycle_index 一致（EsrInputValidation），
  // 故 powered-off 周期复用与 active 周期一致的输入构造。
  const session::EsrCycleResult powered_off = session.StepWithResult(MakeValidInput());
  ExpectThreeWriteAbort(powered_off, session::EsrPipelineAbortReason::kSensorPoweredOff,
                        session::EsrCycleExecutionStatus::kPoweredOff);
  // 规则 14：关机属运行态条件，中止条目 phase=kExecution。
  EXPECT_TRUE(ContainsPhase(powered_off.issues, session::EsrIssuePhase::kExecution));
}

}  // namespace
}  // namespace tests
}  // namespace electronic_surveillance_radar
