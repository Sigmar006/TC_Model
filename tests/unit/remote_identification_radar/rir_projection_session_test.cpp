// Copyright 2026. All Rights Reserved.
//
// @file rir_projection_session_test.cpp
// @brief 观测投影会话级契约与排除诊断发射单元测试（规则 10/11/13b）。
//
// 真实 RirSession 驱动：Attach 自动驱动/nullptr 解绑/注册零行为改变、
// 非执行周期缓存保持；执行期按目标排除诊断（检测门/模式门/识别距离门）
// 携带 cause + scene-entity location 落入完成周期 result.issues。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/foundation/validation_types.h"
#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirExclusionCauseRecorder.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "1q/remote_identification_radar/session/RirTrackLifecycleRecorder.h"
#include "RirCycleInputTestUtil.h"
#include "RirSqliteTestUtil.h"
#include "recognition_feature_database_schema.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using oneq::foundation::ValidationLocationKind;

using session::RirCycleInput;
using session::RirCycleResult;
using session::RirCycleStatus;
using session::RirExclusionCauseEventKind;
using session::RirExclusionCauseRecorder;
using session::RirIssue;
using session::RirSceneTarget;
using session::RirSession;
using session::RirTrackLifecycleEventKind;
using session::RirTrackLifecycleRecorder;

// 近距可检测目标（与 rir_track_attribution_test 同几何，rcs=5 m²）。
RirSceneTarget MakeDetectableTarget(std::uint64_t id, const char* name) {
  RirSceneTarget target;
  target.external_target_id = id;
  target.target_name = name;
  target.position_x = 5000.0f;
  target.position_z = 2000.0f;
  target.rcs = 5.0f;
  return target;
}

// 远距微 RCS 目标：SNR 必然低于 6 dB 回退门 → 检测门排除。
RirSceneTarget MakeUndetectableTarget(std::uint64_t id) {
  RirSceneTarget target;
  target.external_target_id = id;
  target.position_x = 300000.0f;
  target.position_z = 2000.0f;
  target.rcs = 0.01f;
  return target;
}

RirCycleInput MakeInput(std::uint32_t cycle, const std::vector<RirSceneTarget>& targets) {
  RirCycleInput input;
  input.input_cycle_index = cycle;
  input.dt_sec = 0.5;
  input.sim_time_sec = static_cast<float>(cycle - 1U) * 0.5f;
  SetDefaultTestPlatformEcef(&input);
  input.scene_targets = targets;
  return input;
}

config::RirSessionConfig MakeIdentifyConfig() {
  config::RirSessionConfig config;
  config.mission.work_mode = config::RirWorkMode::kIdentify;
  config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  config.policy.lifecycle.confirm_hits = 1U;
  // 主瓣覆盖门放宽：本文件聚焦排除诊断与记录器接线，不测波束覆盖门（门限=半功率宽）。
  config.hardware.antenna.nominal_az_beamwidth_deg = 160.0f;
  config.hardware.antenna.nominal_el_beamwidth_deg = 160.0f;
  return config;
}

// 仅元数据的合法空库：可加载（database_ != nullptr），无类别/型号模板。
std::string WriteEmptyRecognitionDatabase() {
  const char* kProjectionEmptyMetaSql = R"sql(
INSERT INTO meta VALUES
  ('schema_version','1.1'),
  ('database_id','rir-projection-empty'),
  ('version','1.0.0'),
  ('created_utc','2026-08-22T00:00:00Z'),
  ('polarization_channels','H,V'),
  ('polarization_energy_reference','range_propagation_antenna_compensated');
INSERT INTO units VALUES
  ('rcs','dBsm'),('speed','m/s'),('altitude','m'),('acceleration','m/s2'),
  ('turn_radius','m'),('polarization','dB'),('range','m');
)sql";
  return WriteTempSqlite("rir_projection_empty.db",
                         std::string(kRecognitionSchemaSql) + kProjectionEmptyMetaSql);
}

const RirIssue* FindIssue(const RirCycleResult& result, const char* code,
                          std::uint64_t target_id) {
  const std::string id_text = "target_id=" + std::to_string(target_id);
  for (const RirIssue& issue : result.issues) {
    if (issue.code == code && issue.message.find(id_text) != std::string::npos) {
      return &issue;
    }
  }
  return nullptr;
}

/// @brief 规则 10：Attach 后 Session 在 StepWithResult 内自动驱动记录器。
TEST(RirProjectionSessionTest, AttachRecorderDrivesUpdateAutomatically) {
  RirSession session = RirSession::Create(MakeIdentifyConfig());
  RirTrackLifecycleRecorder lifecycle;
  RirExclusionCauseRecorder exclusion;
  session.AttachTrackLifecycleRecorder(&lifecycle);
  session.AttachExclusionCauseRecorder(&exclusion);

  const std::vector<RirSceneTarget> targets = {MakeDetectableTarget(1001U, "alpha")};
  const RirCycleResult cycle1 = session.StepWithResult(MakeInput(1U, targets));
  ASSERT_EQ(cycle1.status, RirCycleStatus::kCompleted);
  // confirm_hits=1：首周期即确认（无需手动 Update）。
  ASSERT_EQ(lifecycle.GetLastEvents().size(), 1U);
  EXPECT_EQ(lifecycle.GetLastEvents()[0].kind, RirTrackLifecycleEventKind::kFirstConfirmed);
  EXPECT_EQ(lifecycle.GetLastEvents()[0].external_target_id, 1001U);
  // 默认配置无特征库：可检测目标检测通过后落 no_feature_database 排除诊断 →
  // 差分记录器同周期产出 A2 进入事件（驱动源同为会话）。
  ASSERT_EQ(exclusion.GetLastEvents().size(), 1U);
  EXPECT_EQ(exclusion.GetLastEvents()[0].kind, RirExclusionCauseEventKind::kEntered);
  EXPECT_EQ(exclusion.GetLastEvents()[0].current_code, session::codes::kTargetNoFeatureDatabase);

  const RirCycleResult cycle2 = session.StepWithResult(MakeInput(2U, targets));
  ASSERT_EQ(cycle2.status, RirCycleStatus::kCompleted);
  ASSERT_EQ(lifecycle.GetLastEvents().size(), 1U);
  EXPECT_EQ(lifecycle.GetLastEvents()[0].kind, RirTrackLifecycleEventKind::kUpdated);
}

/// @brief 规则 11a：nullptr 解绑后 Session 不再驱动（缓存停留最后执行周期）。
TEST(RirProjectionSessionTest, DetachRecorderStopsAutomaticDriving) {
  RirSession session = RirSession::Create(MakeIdentifyConfig());
  RirTrackLifecycleRecorder lifecycle;
  session.AttachTrackLifecycleRecorder(&lifecycle);

  const std::vector<RirSceneTarget> targets = {MakeDetectableTarget(1001U, "alpha")};
  ASSERT_EQ(session.StepWithResult(MakeInput(1U, targets)).status, RirCycleStatus::kCompleted);
  ASSERT_EQ(lifecycle.GetLastEvents().size(), 1U);

  session.AttachTrackLifecycleRecorder(nullptr);
  ASSERT_EQ(session.StepWithResult(MakeInput(2U, targets)).status, RirCycleStatus::kCompleted);
  // 解绑后不再驱动：缓存保留周期 1 的 kFirstConfirmed（周期 2 未进入）。
  ASSERT_EQ(lifecycle.GetLastEvents().size(), 1U);
  EXPECT_EQ(lifecycle.GetLastEvents()[0].kind, RirTrackLifecycleEventKind::kFirstConfirmed);
}

/// @brief 规则 11c：注册与否不影响周期返回值（状态与归属一致）。
TEST(RirProjectionSessionTest, RegistrationDoesNotAffectCycleResults) {
  const std::vector<RirSceneTarget> targets = {MakeDetectableTarget(1001U, "alpha")};

  RirSession bare = RirSession::Create(MakeIdentifyConfig());
  const RirCycleResult bare_result = bare.StepWithResult(MakeInput(1U, targets));

  RirSession equipped = RirSession::Create(MakeIdentifyConfig());
  RirTrackLifecycleRecorder lifecycle;
  RirExclusionCauseRecorder exclusion;
  equipped.AttachTrackLifecycleRecorder(&lifecycle);
  equipped.AttachExclusionCauseRecorder(&exclusion);
  const RirCycleResult equipped_result = equipped.StepWithResult(MakeInput(1U, targets));

  EXPECT_EQ(bare_result.status, equipped_result.status);
  ASSERT_EQ(bare_result.track_attributions.size(), equipped_result.track_attributions.size());
  EXPECT_EQ(bare_result.track_attributions[0].external_target_id,
            equipped_result.track_attributions[0].external_target_id);
  EXPECT_EQ(bare_result.track_attributions[0].association_key,
            equipped_result.track_attributions[0].association_key);
}

/// @brief 非执行周期：记录器缓存保持（校验拒绝不清空、不推进）。
TEST(RirProjectionSessionTest, NonExecutedCycleKeepsRecorderCaches) {
  RirSession session = RirSession::Create(MakeIdentifyConfig());
  RirTrackLifecycleRecorder lifecycle;
  session.AttachTrackLifecycleRecorder(&lifecycle);

  const std::vector<RirSceneTarget> targets = {MakeDetectableTarget(1001U, "alpha")};
  ASSERT_EQ(session.StepWithResult(MakeInput(1U, targets)).status, RirCycleStatus::kCompleted);
  ASSERT_EQ(lifecycle.GetLastEvents().size(), 1U);

  RirCycleInput rejected = MakeInput(2U, targets);
  rejected.dt_sec = 0.0;
  EXPECT_EQ(session.StepWithResult(rejected).status, RirCycleStatus::kRejectedInvalidInput);
  EXPECT_EQ(lifecycle.GetLastEvents().size(), 1U);
}

/// @brief 规则 13b：检测门排除 → kInfo/执行期 issue，携带主因与 scene-entity 定位。
TEST(RirProjectionSessionTest, DetectionGateFailureEmitsTargetedIssue) {
  RirSession session = RirSession::Create(MakeIdentifyConfig());
  const std::vector<RirSceneTarget> targets = {MakeDetectableTarget(1001U, "alpha"),
                                               MakeUndetectableTarget(1002U)};
  const RirCycleResult result = session.StepWithResult(MakeInput(1U, targets));

  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
  const RirIssue* issue =
      FindIssue(result, session::codes::kTargetDetectionGate, 1002U);
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, session::RirIssueSeverity::kInfo);
  EXPECT_EQ(issue->phase, session::RirIssuePhase::kExecution);
  EXPECT_NE(issue->cause, session::RirIssueCause::kNone);
  EXPECT_EQ(issue->location.kind, ValidationLocationKind::kSceneEntity);
  EXPECT_EQ(issue->location.entity_index, 1U);
  // 可检测目标不落检测门诊断。
  EXPECT_EQ(FindIssue(result, session::codes::kTargetDetectionGate, 1001U), nullptr);
}

/// @brief 规则 13b：非识别工作模式 → 逐目标模式门诊断（检测/跟踪不受影响）。
TEST(RirProjectionSessionTest, StandbyModeEmitsModeGateIssuePerTarget) {
  config::RirSessionConfig config;  // 默认 work_mode = kStby。
  RirSession session = RirSession::Create(config);
  const std::vector<RirSceneTarget> targets = {MakeDetectableTarget(1001U, "alpha"),
                                               MakeDetectableTarget(1002U, "beta")};
  const RirCycleResult result = session.StepWithResult(MakeInput(1U, targets));

  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
  const RirIssue* alpha_issue =
      FindIssue(result, session::codes::kTargetModeNotIdentify, 1001U);
  const RirIssue* beta_issue =
      FindIssue(result, session::codes::kTargetModeNotIdentify, 1002U);
  ASSERT_NE(alpha_issue, nullptr);
  ASSERT_NE(beta_issue, nullptr);
  EXPECT_EQ(alpha_issue->phase, session::RirIssuePhase::kExecution);
  EXPECT_EQ(alpha_issue->cause, session::RirIssueCause::kNone);
  EXPECT_EQ(alpha_issue->location.kind, ValidationLocationKind::kSceneEntity);
  EXPECT_EQ(alpha_issue->location.entity_index, 0U);
  EXPECT_EQ(beta_issue->location.entity_index, 1U);
}

/// @brief 规则 13b：检测通过 + 库已加载 + 斜距超识别距离门 → 距离门诊断。
TEST(RirProjectionSessionTest, BeyondRecognitionRangeEmitsRangeGateIssue) {
  const std::string database_path = WriteEmptyRecognitionDatabase();
  ASSERT_FALSE(database_path.empty());

  config::RirSessionConfig config = MakeIdentifyConfig();
  config.mission.max_range_m = 1000.0f;  // 目标斜距 ~5.4 km，必超门。
  config.policy.recognition.enabled = true;
  config.policy.recognition.database_path = database_path;
  RirSession session = RirSession::Create(config);

  const std::vector<RirSceneTarget> targets = {MakeDetectableTarget(1001U, "alpha")};
  const RirCycleResult result = session.StepWithResult(MakeInput(1U, targets));

  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
  const RirIssue* issue =
      FindIssue(result, session::codes::kTargetBeyondRecognitionRange, 1001U);
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->phase, session::RirIssuePhase::kExecution);
  EXPECT_EQ(issue->cause, session::RirIssueCause::kNone);
  EXPECT_EQ(issue->location.kind, ValidationLocationKind::kSceneEntity);
  EXPECT_EQ(issue->location.entity_index, 0U);
}

/// @brief 会话驱动差分记录器：检测门 A2 进入事件（排除码即 13b code）。
TEST(RirProjectionSessionTest, ExclusionRecorderDrivenBySessionProducesEnteredEvent) {
  RirSession session = RirSession::Create(MakeIdentifyConfig());
  RirExclusionCauseRecorder exclusion;
  session.AttachExclusionCauseRecorder(&exclusion);

  const std::vector<RirSceneTarget> targets = {MakeUndetectableTarget(1002U)};
  const RirCycleResult result = session.StepWithResult(MakeInput(1U, targets));
  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);

  ASSERT_EQ(exclusion.GetLastEvents().size(), 1U);
  const session::RirExclusionCauseEvent& event = exclusion.GetLastEvents()[0];
  EXPECT_EQ(event.kind, RirExclusionCauseEventKind::kEntered);
  EXPECT_EQ(event.external_target_id, 1002U);
  EXPECT_EQ(event.current_code, session::codes::kTargetDetectionGate);
}

/// @brief 会话级传感器实体 ID 注入契约（0 恢复缺省，非零生效）。
TEST(RirProjectionSessionTest, SetSensorEntityIdContract) {
  RirSession session = RirSession::Create(MakeIdentifyConfig());
  session.SetSensorEntityId(101U);
  const std::vector<RirSceneTarget> targets = {MakeDetectableTarget(2001U, "test")};
  const RirCycleResult result = session.StepWithResult(MakeInput(1U, targets));
  EXPECT_EQ(result.status, RirCycleStatus::kCompleted);

  // 恢复 0 哨兵
  session.SetSensorEntityId(0U);
  const RirCycleResult result2 = session.StepWithResult(MakeInput(2U, targets));
  EXPECT_EQ(result2.status, RirCycleStatus::kCompleted);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
