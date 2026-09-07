#include <gtest/gtest.h>

#include <string>

#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArExclusionCauseRecorder.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "airborne_radar/session/ArReplayCycleRecord.h"

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"

namespace airborne_radar {
namespace session {
namespace {

config::ArSessionConfig MakeRfConfig() {
  config::ArSessionConfig cfg;
  cfg.policy.detection = config::profiles::kDetectionPriorityDetection;
  cfg.policy.tracking = config::profiles::kFastAssociationTracking;
  cfg.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
  cfg.hardware.rcs_physics.enable_physical_rcs = false;
  cfg.hardware.rcs_physics.physics_mix_ratio = 0.0f;
  return cfg;
}

ArCycleInput MakeInput(std::uint32_t cycle, double start_time_s) {
  ArCycleInput input;
  input.cycle_index = cycle;
  input.cycle_start_time_s = start_time_s;
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
  target.target_id = 77U;
  target.position_x = static_cast<float>(enu.position_enu_m.east_m);
  target.position_y = static_cast<float>(enu.position_enu_m.north_m);
  target.position_z = static_cast<float>(enu.position_enu_m.up_m);
  target.velocity_x = static_cast<float>(enu.velocity_enu_mps.east_mps);
  target.velocity_y = static_cast<float>(enu.velocity_enu_mps.north_mps);
  target.velocity_z = static_cast<float>(enu.velocity_enu_mps.up_mps);
  target.rcs = 5.0f;
  input.targets.push_back(target);
  return input;
}

void AddNoiseInterference(double transmit_power_w, ArCycleInput* input) {
  ASSERT_NE(input, nullptr);
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity = {99U, 1U};
  emission.position_ecef_m = input->platform.platform_position_ecef_m;
  emission.position_ecef_m.x_m += 10000.0;
  const double radar_frequency_hz =
      static_cast<double>(MakeRfConfig().hardware.transmitter.frequency_hz);
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      input->cycle_start_time_s, input->dt_sec, radar_frequency_hz, 20.0e6,
      transmit_power_w, &emission.waveform));
  input->interference.world_cycle_index = input->cycle_index;
  input->interference.window_start_time_s = input->cycle_start_time_s;
  input->interference.window_duration_s = input->dt_sec;
  input->interference.emissions.push_back(emission);
}

void AddUnconfiguredCoSiteInterference(ArCycleInput* input) {
  ASSERT_NE(input, nullptr);
  AddNoiseInterference(1.0, input);
  oneq::electromagnetics::RfSceneEmission& emission = input->interference.emissions.back();
  emission.identity.sensor_entity_id = 999U;
  emission.position_ecef_m = input->platform.platform_position_ecef_m;
}

TEST(ArRfSessionTest, SaturationCompletesWithoutFalseRfObservation) {
  ArCycleInput input = MakeInput(1U, 0.0);
  AddNoiseInterference(1.0e18, &input);
  ArSession radar = ArSession::Create(MakeRfConfig());

  const ArCycleResult result = radar.StepWithResult(input);

  EXPECT_EQ(result.status, ArCycleStatus::kCompleted);
  EXPECT_EQ(result.receiver_impairment, ArReceiverImpairment::kSaturated);
  EXPECT_TRUE(result.interference_observations.empty());
  EXPECT_TRUE(result.output_frame.tracks.empty());
}

TEST(ArRfSessionTest, ReceiverObservationContainsNoTruthIdentity) {
  ArCycleInput input = MakeInput(1U, 0.0);
  AddNoiseInterference(1.0e8, &input);
  ArSession radar = ArSession::Create(MakeRfConfig());

  const ArCycleResult result = radar.StepWithResult(input);

  ASSERT_EQ(result.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(result.receiver_impairment, ArReceiverImpairment::kNone);
  ASSERT_FALSE(result.interference_observations.empty());
  EXPECT_GT(result.interference_observations.front().jammer_to_noise_db, 0.0);
  EXPECT_NE(result.interference_observations.front().observation_id, 0U);
}

TEST(ArRfSessionTest, ExternalAgilityDecisionChangesNextActualCarrier) {
  config::ArSessionConfig config;
  config.hardware.transmitter.frequency_plan_hz = {3.0e9, 3.1e9};
  ArSession radar = ArSession::Create(config);
  const ArCycleResult first = radar.StepWithResult(MakeInput(1U, 0.0));
  ASSERT_EQ(first.status, ArCycleStatus::kCompleted);

  ExternalDecisionOverride override_decision;
  ArControlProfile profile;
  profile.enable_agility_frequency = true;
  override_decision.profile = profile;
  ASSERT_EQ(radar.SubmitExternalDecision(std::move(override_decision)),
            ExternalDecisionSubmitStatus::kAccepted);

  const ArCycleResult second = radar.StepWithResult(MakeInput(2U, 0.5));
  ASSERT_EQ(second.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(second.emission_frame.emissions.size(), 1U);
  EXPECT_DOUBLE_EQ(
      second.emission_frame.emissions.front().waveform.center_frequency_hz,
      3.1e9);
}

TEST(ArRfSessionTest, ReceiveRejectionCommitsEmissionIdentityChronologyAndAppliedAgility) {
  config::ArSessionConfig config;
  config.hardware.transmitter.frequency_plan_hz = {3.0e9, 3.1e9};
  ArSession radar = ArSession::Create(config);
  const ArCycleResult first = radar.StepWithResult(MakeInput(1U, 0.0));
  ASSERT_EQ(first.status, ArCycleStatus::kCompleted);

  ExternalDecisionOverride override_decision;
  ArControlProfile profile;
  profile.enable_agility_frequency = true;
  override_decision.profile = profile;
  ASSERT_EQ(radar.SubmitExternalDecision(std::move(override_decision)),
            ExternalDecisionSubmitStatus::kAccepted);

  ArCycleInput rejected_input = MakeInput(2U, 0.5);
  AddUnconfiguredCoSiteInterference(&rejected_input);
  const ArCycleResult rejected = radar.StepWithResult(rejected_input);
  ASSERT_EQ(rejected.status, ArCycleStatus::kRejectedExecution);
  ASSERT_EQ(rejected.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(rejected.emission_frame.emissions.front().identity.emission_id, 2U);
  EXPECT_DOUBLE_EQ(rejected.emission_frame.emissions.front().waveform.center_frequency_hz, 3.1e9);
  EXPECT_EQ(rejected.applied_decision_source, DecisionControlSource::kExternal);

  // Override was consumed during PrepareEmissionControl of cycle 2.
  // Recovery cycle has no pending override, so native decisions apply (no agility).
  const ArCycleResult next = radar.StepWithResult(MakeInput(3U, 1.0));
  ASSERT_EQ(next.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(next.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(next.emission_frame.emissions.front().identity.emission_id, 3U);
  EXPECT_DOUBLE_EQ(next.emission_frame.emissions.front().waveform.center_frequency_hz, 3.0e9);
}

TEST(ArRfSessionTest, EccmSidelobeControlsKeepNextReceiverPatternValid) {
  ArSession radar = ArSession::Create(MakeRfConfig());
  const ArCycleResult first = radar.StepWithResult(MakeInput(1U, 0.0));
  ASSERT_EQ(first.status, ArCycleStatus::kCompleted);

  ExternalDecisionOverride override_decision;
  ArControlProfile profile;
  profile.enable_sidelobe_canceller = true;
  profile.enable_adaptive_beamforming = true;
  override_decision.profile = profile;
  ASSERT_EQ(radar.SubmitExternalDecision(std::move(override_decision)),
            ExternalDecisionSubmitStatus::kAccepted);

  const ArCycleResult second = radar.StepWithResult(MakeInput(2U, 0.5));
  EXPECT_EQ(second.status, ArCycleStatus::kCompleted);
}

// 旁瓣对消 directive 与发射天线方向图的关系（C1 架构固化）：
// REQUEST_ENABLE_SIDELOBE_CANCELLER 经 ControlProfileEffects 只作用于 SignalPipeline 的
// 内部 detector runtime config（max_sidelobe_level_db -= sidelobe_level_reduction_db）。
// ArSession 的发射构造读取的是未经 ControlProfileEffects 处理的 base detection 工程配置
// （detection.antenna.pattern.max_sidelobe_level_db），故公开 emission_frame 的旁瓣电平
// 不随该 directive 变化；ArSession 的 receiver_state 则另走一套独立常量（12.0 dB）。
// 这正是 C1「两个消费者代表两个物理面（对内 detector vs 对外 RF 场景）」的可观测证据——
// 固化此现状，防止未来把两套消费者误并为单一权威而改变公开 emission 契约。
TEST(ArRfSessionTest, SidelobeCancellerLeavesPublishedEmissionSidelobeUnchanged) {
  config::ArSessionConfig config;
  config.hardware.transmitter.frequency_plan_hz = {3.0e9};
  ArSession radar = ArSession::Create(config);
  const ArCycleResult first = radar.StepWithResult(MakeInput(1U, 0.0));
  ASSERT_EQ(first.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(first.emission_frame.emissions.size(), 1U);
  const double baseline_sidelobe_db =
      first.emission_frame.emissions.front().antenna.sidelobe_level_db;

  ExternalDecisionOverride override_decision;
  ArControlProfile profile;
  profile.enable_sidelobe_canceller = true;
  override_decision.profile = profile;
  ASSERT_EQ(radar.SubmitExternalDecision(std::move(override_decision)),
            ExternalDecisionSubmitStatus::kAccepted);

  const ArCycleResult second = radar.StepWithResult(MakeInput(2U, 0.5));
  ASSERT_EQ(second.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(second.emission_frame.emissions.size(), 1U);
  const double controlled_sidelobe_db =
      second.emission_frame.emissions.front().antenna.sidelobe_level_db;
  // 发射方向图旁瓣读取自 base detection 工程配置，不经 ControlProfileEffects，
  // 故 directive 生效后公开 emission 旁瓣电平保持不变（C1 两面物理事实）。
  EXPECT_DOUBLE_EQ(controlled_sidelobe_db, baseline_sidelobe_db);
}

TEST(ArRfSessionTest, InertialStabilizationKeepsActualEcefBoresightFixed) {
  config::ArSessionConfig body_config = MakeRfConfig();
  body_config.mission.work_mode = config::ArWorkMode::kStt;
  body_config.orientation.stabilization_mode = config::StabilizationMode::kBodyStabilized;
  config::ArSessionConfig inertial_config = body_config;
  inertial_config.orientation.stabilization_mode =
      config::StabilizationMode::kInertialStabilized;

  ArCycleInput level_input = MakeInput(1U, 0.0);
  ArCycleInput yawed_input = level_input;
  yawed_input.platform.platform_attitude_deg.yaw_deg = 30.0;

  ArSession body_level = ArSession::Create(body_config);
  ArSession body_yawed = ArSession::Create(body_config);
  ArSession inertial_level = ArSession::Create(inertial_config);
  ArSession inertial_yawed = ArSession::Create(inertial_config);
  const ArCycleResult body_level_result = body_level.StepWithResult(level_input);
  const ArCycleResult body_yawed_result = body_yawed.StepWithResult(yawed_input);
  const ArCycleResult inertial_level_result = inertial_level.StepWithResult(level_input);
  const ArCycleResult inertial_yawed_result = inertial_yawed.StepWithResult(yawed_input);

  ASSERT_EQ(body_level_result.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(body_yawed_result.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(inertial_level_result.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(inertial_yawed_result.status, ArCycleStatus::kCompleted);
  const auto& body_level_boresight =
      body_level_result.emission_frame.emissions.front().antenna.boresight_ecef;
  const auto& body_yawed_boresight =
      body_yawed_result.emission_frame.emissions.front().antenna.boresight_ecef;
  const auto& inertial_level_boresight =
      inertial_level_result.emission_frame.emissions.front().antenna.boresight_ecef;
  const auto& inertial_yawed_boresight =
      inertial_yawed_result.emission_frame.emissions.front().antenna.boresight_ecef;
  const double body_delta_squared = (body_level_boresight.x - body_yawed_boresight.x) *
                                        (body_level_boresight.x - body_yawed_boresight.x) +
                                    (body_level_boresight.y - body_yawed_boresight.y) *
                                        (body_level_boresight.y - body_yawed_boresight.y) +
                                    (body_level_boresight.z - body_yawed_boresight.z) *
                                        (body_level_boresight.z - body_yawed_boresight.z);
  const double inertial_delta_squared =
      (inertial_level_boresight.x - inertial_yawed_boresight.x) *
          (inertial_level_boresight.x - inertial_yawed_boresight.x) +
      (inertial_level_boresight.y - inertial_yawed_boresight.y) *
          (inertial_level_boresight.y - inertial_yawed_boresight.y) +
      (inertial_level_boresight.z - inertial_yawed_boresight.z) *
          (inertial_level_boresight.z - inertial_yawed_boresight.z);
  EXPECT_GT(body_delta_squared, 0.1);
  EXPECT_LT(inertial_delta_squared, 1.0e-10);
}

TEST(ArRfSessionTest, RuntimePointingPatchChangesNextActualBoresight) {
  // STT 模式隔离扫描动画（TWS 下波束已逐周期推进，scan_center patch 不再
  // 单独决定指向）：STT 下指向 = scan_center，patch 语义可独立验证。
  config::ArSessionConfig config;
  config.mission.work_mode = config::ArWorkMode::kStt;
  ArSession radar = ArSession::Create(config);
  const ArCycleResult first = radar.StepWithResult(MakeInput(1U, 0.0));
  ASSERT_EQ(first.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(first.emission_frame.emissions.size(), 1U);

  config::AzimuthElevationDeg scan_center;
  scan_center.az_deg = 20.0f;
  config::ArRuntimeConfigPatch patch;
  patch.has_scan_center_deg = true;
  patch.scan_center_deg = scan_center;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(patch));

  const ArCycleResult second = radar.StepWithResult(MakeInput(2U, 0.5));
  ASSERT_EQ(second.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(second.emission_frame.emissions.size(), 1U);
  const oneq::electromagnetics::RfSceneDirection& before =
      first.emission_frame.emissions.front().antenna.boresight_ecef;
  const oneq::electromagnetics::RfSceneDirection& after =
      second.emission_frame.emissions.front().antenna.boresight_ecef;
  EXPECT_TRUE(before.x != after.x || before.y != after.y ||
              before.z != after.z);
}

TEST(ArRfSessionTest, PoweredOffCycleDoesNotConsumeEmissionIdentity) {
  ArSession radar = ArSession::Create();
  config::ArRuntimeConfigPatch power_off;
  power_off.has_sensor_enabled = true;
  power_off.sensor_enabled = false;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(power_off));
  EXPECT_EQ(radar.StepWithResult(MakeInput(1U, 0.0)).status,
            ArCycleStatus::kPoweredOff);

  config::ArRuntimeConfigPatch power_on;
  power_on.has_sensor_enabled = true;
  power_on.sensor_enabled = true;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(power_on));
  const ArCycleResult on = radar.StepWithResult(MakeInput(2U, 0.5));
  ASSERT_EQ(on.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(on.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(on.emission_frame.emissions.front().identity.emission_id, 1U);
}

// RunCycle 事务不变量：当 PrepareRfCycle 在进入事务区后失败时，restore_user_cycle
// 必须把会话逐字段回滚到周期开始前——既不预扣 emission ID / token / prepare 计数，
// 也不推进编年史或频率跳频。
// 触发点：第一个周期成功（start=0.0, dt=0.5 → last_window_end=0.5）后，第二个周期
// 用回退的 window_start_time（0.25 < 0.5），命中 PrepareRfCycle 的编年史守护，
// 经 restore_user_cycle() 返回 kRejectedInvalidConfig。
TEST(ArRfSessionTest, PrepareFailureLeavesSessionStateUnchanged) {
  ArSession radar = ArSession::Create();
  ASSERT_EQ(radar.StepWithResult(MakeInput(1U, 0.0)).status, ArCycleStatus::kCompleted);

  const ArSessionReplayState before = ArSessionReplayAccess::CaptureSessionState(radar);
  ASSERT_TRUE(before.has_world_chronology);
  ASSERT_GT(before.next_emission_id, 1U);
  ASSERT_GT(before.successful_prepare_count, 0U);

  // 回退的起始时间触发 PrepareRfCycle 编年史拒绝（事务快照之后的失败路径）。
  const ArCycleResult rejected = radar.StepWithResult(MakeInput(2U, 0.25));
  EXPECT_EQ(rejected.status, ArCycleStatus::kRejectedInvalidConfig);
  EXPECT_TRUE(rejected.output_frame.tracks.empty());
  EXPECT_TRUE(rejected.emission_frame.emissions.empty());

  // 事务不变量：会话逐字段不变。
  const ArSessionReplayState after = ArSessionReplayAccess::CaptureSessionState(radar);
  EXPECT_EQ(after.has_world_chronology, before.has_world_chronology);
  EXPECT_DOUBLE_EQ(after.last_world_window_end_s, before.last_world_window_end_s);
  EXPECT_EQ(after.next_emission_id, before.next_emission_id)
      << "emission ID 不得在 prepare 失败时被预留";
  EXPECT_EQ(after.successful_prepare_count, before.successful_prepare_count);
  EXPECT_EQ(after.frequency_hop_index, before.frequency_hop_index);
  EXPECT_EQ(after.has_pending_runtime_update, before.has_pending_runtime_update);
  EXPECT_EQ(after.pending_execution_config_changed, before.pending_execution_config_changed);
  EXPECT_EQ(after.pending_environment_scenario_config_changed,
           before.pending_environment_scenario_config_changed);
  EXPECT_EQ(after.decision_state.applied_decision_source,
            before.decision_state.applied_decision_source);
  EXPECT_EQ(after.decision_state.applied_decision_cycle_index,
            before.decision_state.applied_decision_cycle_index);

  // 后续合法周期可正常执行——会话未被失败路径破坏。
  const ArCycleResult recovered = radar.StepWithResult(MakeInput(2U, 0.5));
  EXPECT_EQ(recovered.status, ArCycleStatus::kCompleted);
}

TEST(ArRfSessionTest, BelowSnrTargetWritesInfoExclusionDiagnostic) {
  ArCycleInput input = MakeInput(1U, 0.0);
  // 极小 RCS 目标：SNR 落在检测门（min_snr_db / min_detection_margin_db）以下 → 被排除。
  input.targets.back().rcs = 1.0e-9f;
  ArSession radar = ArSession::Create(MakeRfConfig());

  const ArCycleResult result = radar.StepWithResult(input);

  // 行为中立：排除目标不产出航迹；排除原因只经 issues 承载（规则 13b/13c）。
  EXPECT_EQ(result.status, ArCycleStatus::kCompleted);
  EXPECT_TRUE(result.output_frame.tracks.empty());
  bool found = false;
  for (const ArIssue& issue : result.issues) {
    if (issue.code == "ar.target_snr_below_threshold") {
      found = true;
      EXPECT_EQ(issue.severity, ArIssueSeverity::kInfo);
      // 排除诊断属执行阶段（规则 14b phase=kExecution），不得误标为校验问题。
      EXPECT_EQ(issue.phase, ArIssuePhase::kExecution);
      EXPECT_NE(issue.message.find("target_id=77"), std::string::npos);
      // 门内归因（规则 13b）：极小 RCS 主导门失败 → kRcsLimited；
      // message 补充偏轴量值（机器消费仍只认 code/cause）。
      EXPECT_EQ(issue.cause, ArIssueCause::kRcsLimited);
      EXPECT_NE(issue.message.find("off_axis_deg=("), std::string::npos);
      // 实体机器可读关联（规则 14e）：排除诊断结构化携带场景实体索引，
      // 供跨周期差分记录器按实体关联消费（target_id=77 是 targets[0]）。
      EXPECT_EQ(issue.location.kind, oneq::foundation::ValidationLocationKind::kSceneEntity);
      EXPECT_EQ(issue.location.entity_index, 0U);
    }
  }
  EXPECT_TRUE(found);
}

// 排除原因差分记录器经 Session 自动驱动：极小 RCS 目标首周期被排除 → A2 进入事件。
TEST(ArRfSessionTest, ExclusionCauseRecorderDrivenBySessionProducesEnteredEvent) {
  ArCycleInput input = MakeInput(1U, 0.0);
  input.targets.back().rcs = 1.0e-9f;  // 极小 RCS：SNR 门排除（主因 kRcsLimited）
  ArSession radar = ArSession::Create(MakeRfConfig());
  ArExclusionCauseRecorder recorder;
  radar.AttachExclusionCauseRecorder(&recorder);

  radar.StepWithResult(input);

  // Session 自动驱动：recorder 产出 A2 进入事件（target_id=77 首次被排除）。
  const std::vector<ArExclusionCauseEvent>& events = recorder.GetLastEvents();
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, ArExclusionCauseEventKind::kEntered);
  EXPECT_EQ(events[0].external_target_id, 77U);
  EXPECT_EQ(events[0].current_code, "ar.target_snr_below_threshold");
  EXPECT_EQ(events[0].current_cause, ArIssueCause::kRcsLimited);
  EXPECT_TRUE(events[0].previous_code.empty());

  // 解除注册后 Session 不再驱动：第二个周期 GetLastEvents 保持上一周期缓存。
  radar.AttachExclusionCauseRecorder(nullptr);
  input.cycle_index = 2U;
  radar.StepWithResult(input);
  EXPECT_EQ(recorder.GetLastEvents().size(), 1U);
}

TEST(ArRfSessionTest, SetSensorEntityIdCanBeSetAndRestored) {
  ArSession radar = ArSession::Create(MakeRfConfig());
  radar.SetSensorEntityId(101U);
  ArCycleInput input = MakeInput(1U, 0.0);
  ArCycleResult result = radar.StepWithResult(input);
  EXPECT_EQ(result.status, ArCycleStatus::kCompleted);

  // Restore to default (0 sentinel)
  radar.SetSensorEntityId(0U);
  ArCycleInput input2 = MakeInput(2U, 0.5);
  result = radar.StepWithResult(input2);
  EXPECT_EQ(result.status, ArCycleStatus::kCompleted);
}

}  // namespace
}  // namespace session
}  // namespace airborne_radar
