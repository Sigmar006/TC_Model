/**
 * @file esr_exclusion_cause_recorder_test.cpp
 * @brief ESR 排除原因跨周期差分记录器单元测试（规则 13b 差分观测）。
 *
 * 直接构造 EsrCycleInput + EsrCycleResult 调用 Update()，覆盖 A1/A2/A3/A4 四条转换、
 * 非执行周期早退、多发射源独立差分，以及 ESR 特有语义——identity 三元组内部键
 * （免疫跨周期发射源集合变化的下标移位）与 location.entity_index（排序下标）映射。
 */

#include "1q/electronic_surveillance_radar/session/EsrExclusionCauseRecorder.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrOutputTypes.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/foundation/validation_types.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

using oneq::electromagnetics::RfEmissionFrame;
using oneq::electromagnetics::RfEmissionIdentity;
using oneq::electromagnetics::RfSceneEmission;
using oneq::foundation::ValidationLocationKind;

// 构造仅携带 identity 的发射源（recorder 只读 identity，不执行管线）。
RfSceneEmission MakeEmissionWithId(std::uint64_t emission_id) {
  RfSceneEmission emission;
  emission.identity.sensor_entity_id = static_cast<std::uint32_t>(20U + emission_id);
  emission.identity.emission_id = emission_id;
  return emission;
}

// 构造 kInfo 排除诊断（规则 13b），定位到排序后 emissions 数组的 entity_index。
EsrIssue MakeExclusionAtEntity(std::uint64_t entity_index, const std::string& code,
                               EsrIssueCause cause) {
  EsrIssue issue;
  issue.severity = EsrIssueSeverity::kInfo;
  issue.phase = EsrIssuePhase::kExecution;
  issue.code = code;
  issue.message = "emission_id=" + std::to_string(entity_index);
  issue.cause = cause;
  issue.location.kind = ValidationLocationKind::kSceneEntity;
  issue.location.entity_index = static_cast<std::size_t>(entity_index);
  return issue;
}

// 构造 kCompleted 周期结果，携带给定排除诊断列表。
EsrCycleResult MakeCompletedResult(std::uint32_t cycle_index, EsrIssueList issues) {
  EsrCycleResult result;
  result.input_cycle_index = cycle_index;
  result.status = EsrCycleExecutionStatus::kCompleted;
  result.issues = std::move(issues);
  return result;
}

// 构造单发射源输入。
EsrCycleInput MakeInputWithEmission(std::uint64_t emission_id) {
  EsrCycleInput input;
  input.rf_emissions.emissions.push_back(MakeEmissionWithId(emission_id));
  return input;
}

RfEmissionIdentity IdentityOf(std::uint64_t emission_id) {
  return RfEmissionIdentity{static_cast<std::uint32_t>(20U + emission_id), emission_id};
}

// 在事件列表中按 (identity, kind) 查找事件。
const EsrExclusionCauseEvent* FindEvent(const std::vector<EsrExclusionCauseEvent>& events,
                                        std::uint64_t emission_id,
                                        EsrExclusionCauseEventKind kind) {
  const RfEmissionIdentity wanted = IdentityOf(emission_id);
  for (const EsrExclusionCauseEvent& event : events) {
    if (event.identity.sensor_entity_id == wanted.sensor_entity_id &&
        event.identity.emission_id == wanted.emission_id && event.kind == kind) {
      return &event;
    }
  }
  return nullptr;
}

// A2：未被排除 → 被排除（零功率）。
TEST(EsrExclusionCauseRecorderTest, EntryIntoExclusionProducesEnteredEvent) {
  EsrExclusionCauseRecorder recorder;
  EsrCycleInput input = MakeInputWithEmission(7U);
  recorder.Update(input, MakeCompletedResult(1U, {}));
  // emission_id=7 排序后 entity_index=0（单源）。
  const std::vector<EsrExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "esr.emission_below_threshold",
                                                            EsrIssueCause::kHardGateFailed)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, EsrExclusionCauseEventKind::kEntered);
  EXPECT_EQ(events[0].identity.emission_id, 7U);
  EXPECT_EQ(events[0].cycle_index, 2U);
  EXPECT_TRUE(events[0].previous_code.empty());
  EXPECT_EQ(events[0].current_code, "esr.emission_below_threshold");
  EXPECT_EQ(events[0].current_cause, EsrIssueCause::kHardGateFailed);
}

// A3：排除门变化（co-site kNone → zero-power kTransmitSilent），code 与 cause 都变。
TEST(EsrExclusionCauseRecorderTest, GateChangeProducesChangedEvent) {
  EsrExclusionCauseRecorder recorder;
  EsrCycleInput input = MakeInputWithEmission(7U);
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "esr.emission_co_site",
                                                     EsrIssueCause::kNone)}));
  const std::vector<EsrExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "esr.emission_zero_power",
                                                            EsrIssueCause::kTransmitSilent)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, EsrExclusionCauseEventKind::kChanged);
  EXPECT_EQ(events[0].previous_code, "esr.emission_co_site");
  EXPECT_EQ(events[0].previous_cause, EsrIssueCause::kNone);
  EXPECT_EQ(events[0].current_code, "esr.emission_zero_power");
  EXPECT_EQ(events[0].current_cause, EsrIssueCause::kTransmitSilent);
}

// A1：连续两周期相同 (code,cause) → 第二周期不产事件。
TEST(EsrExclusionCauseRecorderTest, StableCauseProducesNoEvent) {
  EsrExclusionCauseRecorder recorder;
  EsrCycleInput input = MakeInputWithEmission(7U);
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "esr.emission_below_threshold",
                                                     EsrIssueCause::kStatisticalGateFailed)}));
  const std::vector<EsrExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "esr.emission_below_threshold",
                                                            EsrIssueCause::kStatisticalGateFailed)}));
  EXPECT_TRUE(events.empty());
}

// A4：被排除 → 不再被排除（issues 为空）。
TEST(EsrExclusionCauseRecorderTest, ExitFromExclusionProducesExitedEvent) {
  EsrExclusionCauseRecorder recorder;
  EsrCycleInput input = MakeInputWithEmission(7U);
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "esr.emission_below_threshold",
                                                     EsrIssueCause::kHardGateFailed)}));
  const std::vector<EsrExclusionCauseEvent> events =
      recorder.Update(input, MakeCompletedResult(2U, {}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, EsrExclusionCauseEventKind::kExited);
  EXPECT_EQ(events[0].identity.emission_id, 7U);
  EXPECT_EQ(events[0].previous_code, "esr.emission_below_threshold");
  EXPECT_TRUE(events[0].current_code.empty());
}

// A4（发射源消失）+ identity 内部键：上周 emission_id=7 被排除，本周 7 消失换成 8
// 且 8 同样被排除（同 entity_index=0，同 code+cause）。
// 纯下标键会把 8 误判为"7 的 A1 稳定"（同下标同 code+cause，无事件）——漏报 7 的 A4 与 8 的 A2；
// identity 键正确判 7 为 A4（消失）、8 为 A2（首次排除）。
TEST(EsrExclusionCauseRecorderTest, DisappearedEmissionProducesExitedEvent) {
  EsrExclusionCauseRecorder recorder;
  // 周期 1：emission_id=7 被排除（单源，entity_index=0）。
  recorder.Update(MakeInputWithEmission(7U),
                  MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                               0U, "esr.emission_below_threshold",
                                               EsrIssueCause::kHardGateFailed)}));
  // 周期 2：emission_id=7 消失，换成 emission_id=8（单源，entity_index=0），8 同样被排除。
  const std::vector<EsrExclusionCauseEvent> events = recorder.Update(
      MakeInputWithEmission(8U),
      MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "esr.emission_below_threshold",
                                                     EsrIssueCause::kHardGateFailed)}));
  // identity 键正确判：7 消失 → A4；8 首次排除 → A2。
  ASSERT_EQ(events.size(), 2U);
  EXPECT_NE(FindEvent(events, 7U, EsrExclusionCauseEventKind::kExited), nullptr);
  EXPECT_NE(FindEvent(events, 8U, EsrExclusionCauseEventKind::kEntered), nullptr);
}

// 非执行周期不产生事件、不推进内部状态。
TEST(EsrExclusionCauseRecorderTest, NonExecutedCycleDoesNotAdvanceState) {
  EsrExclusionCauseRecorder recorder;
  EsrCycleInput input = MakeInputWithEmission(7U);
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "esr.emission_below_threshold",
                                                     EsrIssueCause::kHardGateFailed)}));
  const std::size_t last_size = recorder.GetLastEvents().size();
  EsrCycleResult rejected;
  rejected.status = EsrCycleExecutionStatus::kRejected;
  const std::vector<EsrExclusionCauseEvent> events = recorder.Update(input, rejected);
  EXPECT_TRUE(events.empty());
  EXPECT_EQ(recorder.GetLastEvents().size(), last_size);
  // 周期 3：再次相同排除 → 应为 A1（状态未被非执行周期污染）。
  const std::vector<EsrExclusionCauseEvent> events3 = recorder.Update(
      input, MakeCompletedResult(3U, {MakeExclusionAtEntity(0U, "esr.emission_below_threshold",
                                                            EsrIssueCause::kHardGateFailed)}));
  EXPECT_TRUE(events3.empty());
}

// 多发射源独立差分 + identity 排序（emission_id=7/9 排序后 7 在前）。
TEST(EsrExclusionCauseRecorderTest, MultipleEmissionsTrackIndependently) {
  EsrExclusionCauseRecorder recorder;
  EsrCycleInput input;
  // 注意：push 顺序为 9、7，排序后应为 7（entity_index=0）、9（entity_index=1）。
  input.rf_emissions.emissions.push_back(MakeEmissionWithId(9U));
  input.rf_emissions.emissions.push_back(MakeEmissionWithId(7U));
  // 周期 1：两源均被排除（按排序后 entity_index）。
  recorder.Update(input, MakeCompletedResult(1U,
                                             {MakeExclusionAtEntity(0U, "esr.emission_co_site",
                                                                    EsrIssueCause::kNone),
                                              MakeExclusionAtEntity(1U, "esr.emission_co_site",
                                                                    EsrIssueCause::kNone)}));
  // 周期 2：emission_id=7（entity_index=0）切到 zero-power（A3），9（entity_index=1）退出（A4）。
  const std::vector<EsrExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "esr.emission_zero_power",
                                                            EsrIssueCause::kTransmitSilent)}));
  EXPECT_NE(FindEvent(events, 7U, EsrExclusionCauseEventKind::kChanged), nullptr);
  EXPECT_NE(FindEvent(events, 9U, EsrExclusionCauseEventKind::kExited), nullptr);
}

// Reset 清空内部状态。
TEST(EsrExclusionCauseRecorderTest, ResetClearsState) {
  EsrExclusionCauseRecorder recorder;
  EsrCycleInput input = MakeInputWithEmission(7U);
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "esr.emission_below_threshold",
                                                     EsrIssueCause::kHardGateFailed)}));
  recorder.Reset();
  const std::vector<EsrExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "esr.emission_below_threshold",
                                                            EsrIssueCause::kHardGateFailed)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, EsrExclusionCauseEventKind::kEntered);
}

}  // namespace
}  // namespace session
}  // namespace electronic_surveillance_radar
