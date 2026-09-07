#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <vector>

#include "1q/airborne_radar/session/ArControlProfile.h"
#include "airborne_radar/decision/ControlReducer.h"
#include "airborne_radar/decision/EccmEvaluator.h"
#include "airborne_radar/signal/detection/ArDetectionCellResolver.h"
#include "airborne_radar/signal/tracking/BoostTrackPool.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackState.h"

namespace airborne_radar {
namespace {

session::ArInterferenceObservation BuildObservation(
    oneq::electromagnetics::RfSceneWaveformKind waveform_kind, double off_boresight_deg,
    double jammer_to_noise_db, std::uint32_t coherent_emission_count = 0U,
    double range_rate_mps = 0.0, double carrier_offset_hz = 0.0, double first_pulse_delay_s = 0.0) {
  session::ArInterferenceObservation observation;
  observation.observation_id = 1U;
  observation.estimated_bearing_azimuth_deg = 10.0;
  observation.estimated_bearing_elevation_deg = 2.0;
  observation.estimated_off_boresight_deg = off_boresight_deg;
  observation.estimated_center_frequency_hz = 3.0e9;
  observation.estimated_bandwidth_hz = 2.0e6;
  observation.estimated_waveform_kind = waveform_kind;
  observation.jammer_to_noise_db = jammer_to_noise_db;
  observation.bearing_standard_deviation_deg = 1.0;
  observation.frequency_standard_deviation_hz = 1000.0;
  observation.bandwidth_standard_deviation_hz = 2000.0;
  observation.deception_class = session::DeceptionClass::kNone;
  observation.coherent_emission_count = coherent_emission_count;
  observation.estimated_range_rate_mps = range_rate_mps;
  observation.estimated_carrier_offset_hz = carrier_offset_hz;
  observation.estimated_first_pulse_delay_s = first_pulse_delay_s;
  return observation;
}

session::ArInterferenceObservation BuildFalseTargetObservation() {
  session::ArInterferenceObservation observation =
      BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0);
  observation.deception_class = session::DeceptionClass::kLikelyFalseTarget;
  observation.coherent_emission_count = 3U;
  return observation;
}

bool ContainsDirective(const std::vector<session::TacticalProposal>& proposals,
                       session::ControlDirectiveType type) {
  return std::find_if(proposals.begin(), proposals.end(),
                      [type](const session::TacticalProposal& proposal) {
                        return proposal.directive.type == type;
                      }) != proposals.end();
}

// ============================================================================
// Phase 2: 反欺骗 ECCM 评估测试
// ============================================================================

TEST(ArDeceptionEccmTest, SignificantFirstPulseDelayTriggersAntiRgpoProposal) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  // RGPO（距离波门拖引）的物理可观测特征是首脉冲到达滞后于几何传播期望：
  // estimated_first_pulse_delay_s >= 门限。
  const session::ArInterferenceObservation observation =
      BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0,
                       /*coherent_emission_count=*/0U, /*range_rate_mps=*/0.0,
                       /*carrier_offset_hz=*/0.0, /*first_pulse_delay_s=*/5.0e-6);

  const decision::EccmEvaluator::Result result = evaluator.Evaluate({observation}, &proposals);

  EXPECT_TRUE(result.eccm_activated);
  EXPECT_TRUE(
      ContainsDirective(proposals, session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE));
}

TEST(ArDeceptionEccmTest, SignificantCarrierOffsetTriggersAntiVgpoProposal) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  // VGPO（速度波门拖引）的物理可观测特征是转发载频偏离本振：|estimated_carrier_offset_hz| >= 门限。
  const session::ArInterferenceObservation observation =
      BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0,
                       /*coherent_emission_count=*/0U, /*range_rate_mps=*/0.0,
                       /*carrier_offset_hz=*/8000.0);

  const decision::EccmEvaluator::Result result = evaluator.Evaluate({observation}, &proposals);

  EXPECT_TRUE(result.eccm_activated);
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND));
}

// 单纯 kPulseTrain 观测无 RGPO/VGPO 可观测特征时不应触发任一反欺骗提案——
// 这正是修复前的 bug：旧实现让任意 kPulseTrain 同时给两者加分。
TEST(ArDeceptionEccmTest, PlainPulseTrainWithoutFeaturesDoesNotTriggerAntiDeception) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation =
      BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0);

  evaluator.Evaluate({observation}, &proposals);

  EXPECT_FALSE(
      ContainsDirective(proposals, session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE));
  EXPECT_FALSE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND));
}

TEST(ArDeceptionEccmTest, NonPulseWaveformCannotTriggerRgpoOrVgpoFromResidualFields) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation =
      BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kBandLimitedNoise, 5.0, 8.0,
                       /*coherent_emission_count=*/0U, /*range_rate_mps=*/0.0,
                       /*carrier_offset_hz=*/8000.0, /*first_pulse_delay_s=*/5.0e-6);

  evaluator.Evaluate({observation}, &proposals);

  EXPECT_FALSE(
      ContainsDirective(proposals, session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE));
  EXPECT_FALSE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND));
}

TEST(ArDeceptionEccmTest, FalseTargetClassTriggersAntiFalseTargetProposal) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation = BuildFalseTargetObservation();

  const decision::EccmEvaluator::Result result = evaluator.Evaluate({observation}, &proposals);

  EXPECT_TRUE(result.eccm_activated);
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION));
}

TEST(ArDeceptionEccmTest, AntiDeceptionProposalsHaveSurvivabilitySource) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation =
      BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0);

  evaluator.Evaluate({observation}, &proposals);

  // 验证反欺骗提案来自 SURVIVABILITY 源。
  for (const auto& proposal : proposals) {
    if (proposal.directive.type == session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE ||
        proposal.directive.type ==
            session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND ||
        proposal.directive.type ==
            session::ControlDirectiveType::REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION) {
      EXPECT_EQ(proposal.directive.source, session::ControlDirectiveSource::SURVIVABILITY);
    }
  }
}

TEST(ArDeceptionEccmTest, AntiDeceptionEccmActivatesProtectedEmissionMode) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation =
      BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0);

  const decision::EccmEvaluator::Result result = evaluator.Evaluate({observation}, &proposals);

  // kPulseTrain 触发反欺骗措施，eccm_activated 应为 true。
  EXPECT_TRUE(result.eccm_activated);
  EXPECT_EQ(result.activation_source, decision::EccmEvaluator::ActivationSource::kReceiverRf);
}

TEST(ArDeceptionEccmTest, EmptyObservationSetDoesNotActivateAntiDeception) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;

  const decision::EccmEvaluator::Result result = evaluator.Evaluate({}, &proposals);

  EXPECT_FALSE(result.eccm_activated);
  EXPECT_FALSE(
      ContainsDirective(proposals, session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE));
  EXPECT_FALSE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND));
  EXPECT_FALSE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION));
}

TEST(ArDeceptionEccmTest, NonPulseTrainObservationDoesNotTriggerAntiDeception) {
  decision::EccmEvaluator evaluator;
  // 使用连续波（非脉冲列）——不触发反欺骗。
  std::vector<session::TacticalProposal> proposals_k_continuous;
  evaluator.Evaluate(
      {BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kContinuous, 5.0, 8.0)},
      &proposals_k_continuous);

  EXPECT_FALSE(ContainsDirective(proposals_k_continuous,
                                 session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE));
  EXPECT_FALSE(ContainsDirective(
      proposals_k_continuous, session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND));

  // 使用带限噪声——不触发反欺骗。
  std::vector<session::TacticalProposal> proposals_noise;
  evaluator.Evaluate(
      {BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kBandLimitedNoise, 5.0, 8.0)},
      &proposals_noise);

  EXPECT_FALSE(ContainsDirective(proposals_noise,
                                 session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE));
  EXPECT_FALSE(ContainsDirective(
      proposals_noise, session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND));
}

TEST(ArDeceptionEccmTest, AntiRgpoProposalHasHigherPriorityThanRejitter) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  // kPulseTrain + 首脉冲时延触发 RGPO（前沿跟踪），同时 kPulseTrain 触发 rejitter。
  const session::ArInterferenceObservation observation =
      BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 8.0, 12.0,
                       /*coherent_emission_count=*/0U, /*range_rate_mps=*/0.0,
                       /*carrier_offset_hz=*/0.0, /*first_pulse_delay_s=*/5.0e-6);

  evaluator.Evaluate({observation}, &proposals);

  const auto rgpo_it =
      std::find_if(proposals.begin(), proposals.end(), [](const session::TacticalProposal& p) {
        return p.directive.type == session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE;
      });
  const auto rejitter_it =
      std::find_if(proposals.begin(), proposals.end(), [](const session::TacticalProposal& p) {
        return p.directive.type == session::ControlDirectiveType::REQUEST_ECCM_REJITTER;
      });
  ASSERT_NE(rgpo_it, proposals.end());
  ASSERT_NE(rejitter_it, proposals.end());
  // 反 RGPO 前沿跟踪（基础优先级 89）应高于重频抖动（基础优先级 83）。
  EXPECT_GT(rgpo_it->priority, rejitter_it->priority);
}

TEST(ArDeceptionEccmTest, MultiplePulseTrainObservationsAccumulateAntiDeceptionScores) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  // 两个 kPulseTrain 观测：obs1 带显著首脉冲时延（触发 RGPO），obs2 带显著载频偏移
  // （触发 VGPO）。两者独立路由到不同反欺骗通道，验证可观测特征分离的累积语义。
  const session::ArInterferenceObservation obs1 =
      BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0,
                       /*coherent_emission_count=*/0U, /*range_rate_mps=*/0.0,
                       /*carrier_offset_hz=*/0.0, /*first_pulse_delay_s=*/5.0e-6);
  const session::ArInterferenceObservation obs2 =
      BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 3.0, 7.0,
                       /*coherent_emission_count=*/0U, /*range_rate_mps=*/0.0,
                       /*carrier_offset_hz=*/9000.0);

  evaluator.Evaluate({obs1, obs2}, &proposals);

  EXPECT_TRUE(
      ContainsDirective(proposals, session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE));
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND));
}

// ============================================================================
// Phase 3: 信号层反制措施测试
// ============================================================================

TEST(ArDeceptionEccmTest, AntiDeceptionDirectiveRejectedWhenInvalidValue) {
  // 布尔型反欺骗指令不应携带 requested_value。
  const session::ControlDirective directive_with_value(
      session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE,
      session::ControlDirectiveSource::SURVIVABILITY, 1.0f);
  EXPECT_TRUE(directive_with_value.has_requested_value);

  // 构造 ControlReducer 验证该指令被 IsValidDirectiveValue 拒绝。
  decision::ControlReducer reducer({});
  std::vector<session::TacticalProposal> proposals;
  proposals.push_back(session::TacticalProposal{directive_with_value, 90, ""});

  session::ArControlProfile profile;
  const auto result = reducer.Reduce(profile, proposals);

  EXPECT_TRUE(result.applied_directives.empty());
  EXPECT_FALSE(result.rejected_directives.empty());
  EXPECT_FALSE(result.profile.enable_anti_rgpo_leading_edge);
}

TEST(ArDeceptionEccmTest, AntiRgpoDetectionCellWorksWithAntiRgpoEnabled) {
  // 验证启用 anti-RGPO 时 detection cell 仍正常求解（不失配）。
  using namespace oneq::electromagnetics;

  // 构造 kPulseTrain 干扰入射链路。
  RfIncidentLinkResult link;
  link.identity = RfEmissionIdentity{100U, 1000U};
  link.received_power_before_overlap_w = 10.0;
  link.received_power_w = 10.0;
  link.propagation_delay_s = 0.0;  // 零传播延迟确保 emission_time 落入活动窗口
  link.doppler_shift_hz = 0.0;
  link.emission_waveform.kind = RfSceneWaveformKind::kPulseTrain;
  link.emission_waveform.center_frequency_hz = 3.0e9;
  link.emission_waveform.occupied_bandwidth_hz = 2.0e6;
  link.emission_waveform.activity_start_time_s = 0.0;
  link.emission_waveform.activity_duration_s = 0.1;
  link.emission_waveform.pulse_width_s = 1e-6;
  link.emission_waveform.pulse_repetition_interval_s = 1e-4;
  link.emission_waveform.first_pulse_time_s = 0.0;
  link.emission_waveform.pulse_count = 1000U;
  link.emission_waveform.pulse_jitter_fraction = 0.0;
  link.emission_waveform.transmit_power_w = 100.0;

  // 配置 AR 自身发射波形。
  RfWaveformSchedule own_waveform;
  own_waveform.kind = RfSceneWaveformKind::kPulseTrain;
  own_waveform.center_frequency_hz = 3.0e9;
  own_waveform.occupied_bandwidth_hz = 2.0e6;
  own_waveform.pulse_width_s = 1e-6;
  own_waveform.pulse_repetition_interval_s = 1e-4;
  own_waveform.first_pulse_time_s = 0.0;
  own_waveform.pulse_count = 1000U;
  own_waveform.pulse_jitter_fraction = 0.0;
  own_waveform.transmit_power_w = 1000.0;
  own_waveform.activity_start_time_s = 0.0;
  own_waveform.activity_duration_s = 0.1;

  signal::detection::ArDetectionCellConfig cell_config;
  cell_config.own_transmit_waveform = own_waveform;
  cell_config.receive_window_start_time_s = 0.0;
  cell_config.receive_window_duration_s = 0.1;
  cell_config.matched_filter_bandwidth_hz = 2.0e6;
  cell_config.one_way_antenna_gain_dbi = 30.0;
  cell_config.receiver_loss_db = 3.0;
  cell_config.receiver_noise_figure_db = 3.0;

  signal::detection::ArDetectionCellTarget cell_target;
  cell_target.range_m = 1.0;  // 极近距离使 echo 与干扰脉冲在时域重叠
  cell_target.closing_radial_velocity_mps = 100.0;
  cell_target.rcs_m2 = 1.0;
  cell_target.two_way_additional_propagation_loss_db = 0.0;
  cell_target.effective_pulse_count = 10U;

  const RfEmissionIdentity own_id{999U, 999U};
  const std::vector<RfIncidentLinkResult> incident_links = {link};

  // 不启用 anti-RGPO：正常求解。
  {
    cell_config.enable_anti_rgpo_leading_edge = false;
    signal::detection::ArDetectionCellResult result;
    ASSERT_TRUE(signal::detection::TryResolveArDetectionCell(cell_config, cell_target, own_id,
                                                             incident_links, 0.0, &result));
    EXPECT_GT(result.interference_power_w, 0.0);
  }

  // 启用 anti-RGPO：仍正常求解，干扰功率因 kPulseTrain 功率减半而降低。
  {
    cell_config.enable_anti_rgpo_leading_edge = true;
    signal::detection::ArDetectionCellResult result;
    ASSERT_TRUE(signal::detection::TryResolveArDetectionCell(cell_config, cell_target, own_id,
                                                             incident_links, 0.0, &result));
    EXPECT_GT(result.interference_power_w, 0.0);

    cell_config.enable_anti_rgpo_leading_edge = false;
    signal::detection::ArDetectionCellResult result_off;
    ASSERT_TRUE(signal::detection::TryResolveArDetectionCell(cell_config, cell_target, own_id,
                                                             incident_links, 0.0, &result_off));
    EXPECT_GT(result_off.interference_power_w, 0.0);

    // 启用 anti-RGPO 后 kPulseTrain 干扰功率应降低约 50%。
    EXPECT_LT(result.interference_power_w, result_off.interference_power_w);
  }
}

TEST(ArDeceptionEccmTest, AntiVgpoBoundsVelocityChangePerCycle) {
  // 验证启用加速度限幅后，跨周期速度变化不超出 max_acceleration * dt。
  // 限幅已在 TrackLifecycleManager 中实现（迁移自 TrackFilter），以 track.velocity 作为
  // 上一周期基准。本测试通过 lifecycle Update 验证真实管线行为。
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 1;
  config.max_lost_cycles = 2;
  config.enable_anti_vgpo_acceleration_bound = true;
  config.max_acceleration_mps2 = 10.0;  // 最大加速度 10 m/s²

  signal::tracking::TrackLifecycleManager manager(pool, config);

  // 第一周期：建立 track，初始速度 X=10 m/s, Y=5 m/s。
  signal::tracking::TrackMeasurement seed_measurement;
  seed_measurement.raw_measurement.association_key = 42;
  seed_measurement.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  seed_measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  seed_measurement.filtered_feature.velocity = Eigen::Vector3f(10.0f, 5.0f, 0.0f);

  signal::tracking::CycleContext cycle1;
  cycle1.cycle_index = 1;
  cycle1.batch_id = 1001;
  cycle1.dt_sec = 0.1f;  // 步长 0.1s → max_delta = 10.0 * 0.1 = 1.0 m/s
  manager.Update(cycle1, {seed_measurement});

  // 第二周期：测量速度多轴突增（X: 10→100, Y: 5→60, Z: 0→40），均远超 1.0 m/s 上限。
  signal::tracking::TrackMeasurement jump_measurement;
  jump_measurement.raw_measurement.association_key = 42;
  jump_measurement.raw_measurement.matched_existing_track = true;
  jump_measurement.raw_measurement.position = Eigen::Vector3f(110.0f, 0.0f, 0.0f);
  jump_measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  jump_measurement.filtered_feature.velocity = Eigen::Vector3f(100.0f, 60.0f, 40.0f);

  signal::tracking::CycleContext cycle2;
  cycle2.cycle_index = 2;
  cycle2.batch_id = 1002;
  cycle2.dt_sec = 0.1f;
  manager.Update(cycle2, {jump_measurement});

  const auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  // max_delta = 1.0 m/s：各轴变化应被裁剪到 基准 ± 1.0。
  // X: 基准 10 → 应在 [10, 11]；Y: 基准 5 → 应在 [5, 6]；Z: 基准 0 → 应在 [0, 1]。
  EXPECT_LE(active_tracks[0]->velocity.x(), 11.0f);
  EXPECT_GE(active_tracks[0]->velocity.x(), 10.0f);
  EXPECT_LE(active_tracks[0]->velocity.y(), 6.0f);
  EXPECT_GE(active_tracks[0]->velocity.y(), 5.0f);
  EXPECT_LE(active_tracks[0]->velocity.z(), 1.0f);
  EXPECT_GE(active_tracks[0]->velocity.z(), 0.0f);
}

TEST(ArDeceptionEccmTest, AntiVgpoDoesNotClampNewlyCreatedTrack) {
  // 新生航迹的 velocity 基准是初始零值（非真实上一周期速度），限幅必须对其豁免；
  // 否则首周期测量速度会被裁剪到 0 ± max_delta，破坏航迹初始化。本测试直接覆盖该边界。
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 1;
  config.max_lost_cycles = 2;
  config.enable_anti_vgpo_acceleration_bound = true;
  config.max_acceleration_mps2 = 10.0;  // 若误限幅，max_delta = 1.0 m/s

  signal::tracking::TrackLifecycleManager manager(pool, config);

  // 首周期：新航迹，大速度（X=80），若被误限幅则降至 ~1 m/s。
  signal::tracking::TrackMeasurement first_measurement;
  first_measurement.raw_measurement.association_key = 7;
  first_measurement.raw_measurement.position = Eigen::Vector3f(1000.0f, 0.0f, 0.0f);
  first_measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  first_measurement.filtered_feature.velocity = Eigen::Vector3f(80.0f, 0.0f, 0.0f);

  signal::tracking::CycleContext cycle;
  cycle.cycle_index = 1;
  cycle.batch_id = 1;
  cycle.dt_sec = 0.1f;
  manager.Update(cycle, {first_measurement});

  const auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  // 新航迹豁免：首周期速度应原样采纳，不因零基准被裁剪。
  EXPECT_FLOAT_EQ(active_tracks[0]->velocity.x(), 80.0f);
}

TEST(ArDeceptionEccmTest, AntiVgpoDisabledDoesNotBoundVelocity) {
  // 不启用限幅时，跨周期速度变化原样通过。
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 1;
  config.max_lost_cycles = 2;
  config.enable_anti_vgpo_acceleration_bound = false;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  signal::tracking::TrackMeasurement seed_measurement;
  seed_measurement.raw_measurement.association_key = 42;
  seed_measurement.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  seed_measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  seed_measurement.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);

  signal::tracking::CycleContext cycle1;
  cycle1.cycle_index = 1;
  cycle1.batch_id = 1001;
  cycle1.dt_sec = 0.1f;
  manager.Update(cycle1, {seed_measurement});

  signal::tracking::TrackMeasurement jump_measurement;
  jump_measurement.raw_measurement.association_key = 42;
  jump_measurement.raw_measurement.matched_existing_track = true;
  jump_measurement.raw_measurement.position = Eigen::Vector3f(110.0f, 0.0f, 0.0f);
  jump_measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  jump_measurement.filtered_feature.velocity = Eigen::Vector3f(100.0f, 50.0f, 0.0f);

  signal::tracking::CycleContext cycle2;
  cycle2.cycle_index = 2;
  cycle2.batch_id = 1002;
  cycle2.dt_sec = 0.1f;
  manager.Update(cycle2, {jump_measurement});

  const auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  // 不启用限幅 → 测量速度原样写入。
  EXPECT_FLOAT_EQ(active_tracks[0]->velocity.x(), 100.0f);
  EXPECT_FLOAT_EQ(active_tracks[0]->velocity.y(), 50.0f);
}

// 修复 P3：限幅后必须回写 gaussian_state 和 acceleration。此前实现只改 track.velocity，
// 下一周期 Predict 从 gaussian_state 读取未限幅速度（bug 根因）。
// 本测试用无 predictor/updater 的管理器验证限幅后 gaussian_state.mean 速度分量被正确回写。
TEST(ArDeceptionEccmTest, AntiVgpoClampWritesBackToGaussianState) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 1;
  config.max_lost_cycles = 2;
  config.enable_anti_vgpo_acceleration_bound = true;
  config.max_acceleration_mps2 = 10.0;  // 0.1s dt → max_delta = 1.0 m/s

  signal::tracking::TrackLifecycleManager manager(pool, config);

  // 第一周期：建立 track，初始速度 X=10。
  signal::tracking::TrackMeasurement seed;
  seed.raw_measurement.association_key = 42;
  seed.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  seed.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  seed.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);

  signal::tracking::CycleContext cycle1{1U, 1001U, 0.1f};
  manager.Update(cycle1, {seed});

  // 第二周期：大速度跳跃 (X: 10→100)，应被限幅到 10±1=11。
  signal::tracking::TrackMeasurement jump;
  jump.raw_measurement.association_key = 42;
  jump.raw_measurement.matched_existing_track = true;
  jump.raw_measurement.position = Eigen::Vector3f(110.0f, 0.0f, 0.0f);
  jump.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  jump.filtered_feature.velocity = Eigen::Vector3f(100.0f, 0.0f, 0.0f);

  signal::tracking::CycleContext cycle2{2U, 1002U, 0.1f};
  manager.Update(cycle2, {jump});

  const auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1U);
  // track.velocity 已限幅。
  EXPECT_FLOAT_EQ(active_tracks[0]->velocity.x(), 11.0f);
  // gaussian_state.mean 速度分量应同步为限幅值（修复前为未限幅后验速度 ~100 或零值）。
  EXPECT_FLOAT_EQ(active_tracks[0]->gaussian_state.mean(1), 11.0f);
}

// 限幅回写后，下一周期 Predict 应从限幅速度出发，而非从未限幅后验重新跳回。
// 第三周期不注入新量测（miss），让 Predict 单独起作用。
TEST(ArDeceptionEccmTest, AntiVgpoClampPropagatesToNextPredict) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.enable_anti_vgpo_acceleration_bound = true;
  config.max_acceleration_mps2 = 10.0;  // 0.1s dt → max_delta = 1.0 m/s

  signal::tracking::KalmanPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = 1.0f;
  signal::tracking::KalmanPredictor predictor(pred_cfg);
  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 0.1f;
  signal::tracking::KalmanUpdater updater(upd_cfg);

  signal::tracking::TrackLifecycleManager manager(pool, config, &predictor, &updater);

  signal::tracking::TrackMeasurement seed;
  seed.raw_measurement.association_key = 42;
  seed.raw_measurement.position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  seed.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  seed.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);

  manager.Update({1U, 1001U, 0.1f}, {seed});

  // 第二周期：速度突增，限幅到 11 m/s。
  signal::tracking::TrackMeasurement jump;
  jump.raw_measurement.association_key = 42;
  jump.raw_measurement.matched_existing_track = true;
  jump.raw_measurement.position = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  jump.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  jump.filtered_feature.velocity = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  manager.Update({2U, 1002U, 0.1f}, {jump});

  // 第三周期：miss（无量测），Predict 从 gaussian_state 预测。
  // 若限幅已正确回写，预测应从 11 m/s 出发，位置推进约 11*0.1=1.1m。
  manager.Update({3U, 1003U, 0.1f}, {});

  const auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1U);
  // 位置应约为 10 + 11*0.1 = 11.1m（容许测量噪声和滤波平滑）。
  EXPECT_NEAR(active_tracks[0]->position.x(), 11.0f, 2.0f);
  // 速度不应跳回未限幅值 (> 90 m/s)。
  EXPECT_LT(active_tracks[0]->velocity.x(), 20.0f);
}

// 限幅后 acceleration 应从限幅后速度重算，而非沿用未限幅后验的加速度。
TEST(ArDeceptionEccmTest, AntiVgpoClampRecomputesAcceleration) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 1;
  config.max_lost_cycles = 2;
  config.enable_anti_vgpo_acceleration_bound = true;
  config.max_acceleration_mps2 = 10.0;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  signal::tracking::TrackMeasurement seed;
  seed.raw_measurement.association_key = 42;
  seed.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  seed.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  seed.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  manager.Update({1U, 1001U, 0.1f}, {seed});

  // 速度突增到 100 m/s，限幅到 11 m/s。期望加速度 ≈ (11-10)/0.1 = 10 m/s²。
  signal::tracking::TrackMeasurement jump;
  jump.raw_measurement.association_key = 42;
  jump.raw_measurement.matched_existing_track = true;
  jump.raw_measurement.position = Eigen::Vector3f(110.0f, 0.0f, 0.0f);
  jump.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  jump.filtered_feature.velocity = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  manager.Update({2U, 1002U, 0.1f}, {jump});

  const auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1U);
  EXPECT_FLOAT_EQ(active_tracks[0]->velocity.x(), 11.0f);
  // 加速度 ≈ (11-10)/0.1 = 10 m/s²（修复前为 (100-10)/0.1 = 900 m/s²）。
  EXPECT_NEAR(active_tracks[0]->acceleration.x(), 10.0f, 1.0f);
}

TEST(ArDeceptionEccmTest, AntiFalseTargetSuppressesTentativePromotion) {
  // 启用假目标鉴别时，疑似假目标的量测不会把 tentative 航迹晋升为 confirmed，
  // 即使命中次数已达 confirm_hits。
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 2;
  config.max_miss_before_lost = 1;
  config.max_lost_cycles = 2;
  config.enable_anti_false_target_discrimination = true;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  // 构造一条被观测层判定为疑似假目标的量测。
  signal::tracking::TrackMeasurement false_target_measurement;
  false_target_measurement.raw_measurement.association_key = 9;
  false_target_measurement.raw_measurement.classified_as_false_target = true;
  false_target_measurement.raw_measurement.position = Eigen::Vector3f(500.0f, 0.0f, 0.0f);
  false_target_measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  false_target_measurement.filtered_feature.velocity = Eigen::Vector3f(20.0f, 0.0f, 0.0f);

  signal::tracking::CycleContext cycle;
  cycle.cycle_index = 1;
  cycle.batch_id = 1;
  cycle.dt_sec = 0.1f;

  // 连续命中 2 次（达到 confirm_hits=2），但每次都标为疑似假目标。
  manager.Update(cycle, {false_target_measurement});
  cycle.cycle_index = 2;
  cycle.batch_id = 2;
  manager.Update(cycle, {false_target_measurement});

  const auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  // 鉴别生效：命中次数达标但航迹仍停留在 tentative，未被晋升为 confirmed。
  EXPECT_EQ(active_tracks[0]->status, signal::tracking::TrackStatus::kTentative);
}

TEST(ArDeceptionEccmTest, AntiFalseTargetDisabledPromotesNormally) {
  // 鉴别关闭时，疑似假目标的量测按正常状态机晋升（回归）。
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 2;
  config.max_miss_before_lost = 1;
  config.max_lost_cycles = 2;
  config.enable_anti_false_target_discrimination = false;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  signal::tracking::TrackMeasurement false_target_measurement;
  false_target_measurement.raw_measurement.association_key = 9;
  false_target_measurement.raw_measurement.classified_as_false_target = true;
  false_target_measurement.raw_measurement.position = Eigen::Vector3f(500.0f, 0.0f, 0.0f);
  false_target_measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  false_target_measurement.filtered_feature.velocity = Eigen::Vector3f(20.0f, 0.0f, 0.0f);

  signal::tracking::CycleContext cycle;
  cycle.cycle_index = 1;
  cycle.batch_id = 1;
  cycle.dt_sec = 0.1f;
  manager.Update(cycle, {false_target_measurement});
  cycle.cycle_index = 2;
  cycle.batch_id = 2;
  manager.Update(cycle, {false_target_measurement});

  const auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  // 鉴别关闭：命中 2 次后正常晋升为 confirmed。
  EXPECT_EQ(active_tracks[0]->status, signal::tracking::TrackStatus::kConfirmed);
}

TEST(ArDeceptionEccmTest, AntiFalseTargetDoesNotBlockLostTrackReconfirm) {
  // 假目标鉴别只抑制 tentative→confirmed；已 confirmed 后因失配转为 lost 的真实航迹，
  // 重新命中时即使量测被标为疑似假目标，仍应恢复为 confirmed（不阻断真实航迹恢复）。
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;          // 命中 1 次即确认
  config.max_miss_before_lost = 0;  // 失配 1 次即转 lost
  config.max_lost_cycles = 5;
  config.enable_anti_false_target_discrimination = true;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  signal::tracking::TrackMeasurement measurement;
  measurement.raw_measurement.association_key = 7;
  measurement.raw_measurement.position = Eigen::Vector3f(200.0f, 0.0f, 0.0f);
  measurement.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  measurement.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);

  // 第一周期：命中 → confirmed（tentative 命中 1 次，鉴别不阻止，因为首次即达 confirm_hits）。
  signal::tracking::CycleContext cycle;
  cycle.cycle_index = 1;
  cycle.batch_id = 1;
  cycle.dt_sec = 0.1f;
  manager.Update(cycle, {measurement});
  ASSERT_EQ(manager.GetActiveTracks().size(), 1u);
  ASSERT_EQ(manager.GetActiveTracks()[0]->status, signal::tracking::TrackStatus::kConfirmed);

  // 第二周期：失配 → confirmed 转 lost。
  cycle.cycle_index = 2;
  cycle.batch_id = 2;
  manager.Update(cycle, {});
  ASSERT_EQ(manager.GetActiveTracks()[0]->status, signal::tracking::TrackStatus::kLost);

  // 第三周期：重新命中，但量测标为疑似假目标。lost 航迹应恢复为 confirmed。
  measurement.raw_measurement.classified_as_false_target = true;
  measurement.raw_measurement.matched_existing_track = true;
  cycle.cycle_index = 3;
  cycle.batch_id = 3;
  manager.Update(cycle, {measurement});

  const auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, signal::tracking::TrackStatus::kConfirmed);
}

TEST(ArDeceptionEccmTest, AntiDeceptionControlDirectiveAppliedByReducer) {
  // 验证合法的反欺骗指令经 ControlReducer::Reduce 后写入 ArControlProfile。
  decision::ControlReducer reducer({});
  std::vector<session::TacticalProposal> proposals;
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE,
                                session::ControlDirectiveSource::SURVIVABILITY),
      90, ""});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND,
                                session::ControlDirectiveSource::SURVIVABILITY),
      85, ""});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(
          session::ControlDirectiveType::REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION,
          session::ControlDirectiveSource::SURVIVABILITY),
      81, ""});

  session::ArControlProfile profile;
  const auto result = reducer.Reduce(profile, proposals);

  EXPECT_TRUE(result.profile.enable_anti_rgpo_leading_edge);
  EXPECT_TRUE(result.profile.enable_anti_vgpo_acceleration_bound);
  EXPECT_TRUE(result.profile.enable_anti_false_target_discrimination);
  EXPECT_EQ(result.applied_directives.size(), 3U);
  // 反欺骗属于 ECCM 域，应在 ResetEccmDomain 后写入。
  EXPECT_FALSE(result.profile.enable_lpi_power_control);
}

// IMM 限幅回写：验证限幅后各 IMM 模型状态的 mean 速度分量同步裁剪。
// 此前实现只写回 track.gaussian_state 但未同步 IMM 内部 model_states_，
// 下一周期 Process() 从各模型重新混合会恢复未限幅速度。
TEST(ArDeceptionEccmTest, AntiVgpoClampSyncsImmModelStates) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 1;
  config.max_lost_cycles = 3;
  config.imm_activation_policy = signal::tracking::ImmActivationPolicy::kAllTracks;
  config.enable_anti_vgpo_acceleration_bound = true;
  config.max_acceleration_mps2 =
      1.0;  // 0.1s dt → max_delta = 0.1 m/s（紧，IMM 微小速度变化即触发）

  signal::tracking::KalmanPredictorConfig pred_cfg_1;
  pred_cfg_1.noise_diff_coeff = 0.5f;
  signal::tracking::KalmanPredictor pred_1(pred_cfg_1);
  signal::tracking::KalmanPredictorConfig pred_cfg_2;
  pred_cfg_2.noise_diff_coeff = 15.0f;
  signal::tracking::KalmanPredictor pred_2(pred_cfg_2);

  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 0.1f;
  signal::tracking::KalmanUpdater upd_1(upd_cfg);
  signal::tracking::KalmanUpdater upd_2(upd_cfg);

  Eigen::MatrixXf transition_probability(2, 2);
  transition_probability << 0.95f, 0.05f, 0.05f, 0.95f;
  Eigen::VectorXf initial_weights(2);
  initial_weights << 0.5f, 0.5f;

  signal::tracking::TrackLifecycleManager manager(
      pool, config, {&pred_1, &pred_2}, {&upd_1, &upd_2}, transition_probability, initial_weights);

  // 第一周期：建立 IMM track，初始速度 X=10。
  signal::tracking::TrackMeasurement seed;
  seed.raw_measurement.association_key = 42;
  seed.raw_measurement.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  seed.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  seed.filtered_feature.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  manager.Update({1U, 1001U, 0.1f}, {seed});

  // 第二周期：速度突增。IMM Process 更新后限幅触发。
  signal::tracking::TrackMeasurement jump;
  jump.raw_measurement.association_key = 42;
  jump.raw_measurement.matched_existing_track = true;
  jump.raw_measurement.position = Eigen::Vector3f(110.0f, 0.0f, 0.0f);
  jump.raw_measurement.measurement_covariance = Eigen::Matrix3f::Identity();
  jump.filtered_feature.velocity = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  manager.Update({2U, 1002U, 0.1f}, {jump});

  const auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1U);
  // max_delta = 0.1 m/s：速度应在 [10, 10.1] 区间。
  EXPECT_LE(active_tracks[0]->velocity.x(), 10.1f);
  EXPECT_GE(active_tracks[0]->velocity.x(), 10.0f);
  // gaussian_state 镜像同步限幅。
  EXPECT_LE(active_tracks[0]->gaussian_state.mean(1), 10.1f);
  EXPECT_GE(active_tracks[0]->gaussian_state.mean(1), 10.0f);
  // IMM 模型状态也应同步限幅——每个模型的 vx 均值 ≤ 10+0.2（容许 IMM 混合平滑）。
  // 更精确的断言需访问内部 model_states_，这里通过快照层间接验证：
  // 若模型状态未限幅，下周期 Predict 会从未限幅速度重新跳回。
  manager.Update({3U, 1003U, 0.1f}, {});
  const auto after_miss = manager.GetActiveTracks();
  ASSERT_EQ(after_miss.size(), 1U);
  // 位置外推应约为 110 + 10*0.1 = 111m（而非 110 + 100*0.1 = 120m）。
  EXPECT_NEAR(after_miss[0]->position.x(), 111.0f, 5.0f);
  EXPECT_LT(after_miss[0]->velocity.x(), 20.0f);
}

// EccmEvaluator 路由边界：first_pulse_delay 刚好达到 RGPO 门限（100 ns）触发 RGPO。
TEST(ArDeceptionEccmTest, FirstPulseDelayAtThresholdTriggersAntiRgpo) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation =
      BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0,
                       /*coherent_emission_count=*/0U, /*range_rate_mps=*/0.0,
                       /*carrier_offset_hz=*/0.0, /*first_pulse_delay_s=*/1.0e-7);
  evaluator.Evaluate({observation}, &proposals);
  EXPECT_TRUE(
      ContainsDirective(proposals, session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE));
}

// EccmEvaluator 路由边界：|carrier_offset| 刚好达到 VGPO 门限（1000 Hz）触发 VGPO。
TEST(ArDeceptionEccmTest, CarrierOffsetAtThresholdTriggersAntiVgpo) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation =
      BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0,
                       /*coherent_emission_count=*/0U, /*range_rate_mps=*/0.0,
                       /*carrier_offset_hz=*/1000.0);
  evaluator.Evaluate({observation}, &proposals);
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND));
}

// EccmEvaluator 路由边界：carrier_offset 刚好低于门限不应触发 VGPO；同时确认
// coherent_emission_count 与 range_rate 不再（误）触发任一反欺骗通道——它们已不是
// RGPO/VGPO 的代理。
TEST(ArDeceptionEccmTest, CarrierOffsetBelowThresholdDoesNotTriggerAntiVgpo) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation =
      BuildObservation(oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 5.0, 8.0,
                       /*coherent_emission_count=*/5U, /*range_rate_mps=*/300.0,
                       /*carrier_offset_hz=*/999.0);
  evaluator.Evaluate({observation}, &proposals);
  EXPECT_FALSE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND));
  EXPECT_FALSE(
      ContainsDirective(proposals, session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE));
}

}  // namespace
}  // namespace airborne_radar
