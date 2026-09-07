/**
 * @file ar_stt_track_follow_test.cpp
 * @brief 验证 STT 指定航迹跟随指向（方案 A）与自动丢跟踪回退。
 *
 * 覆盖：
 *   1) 雷达局部位置 → az/el 换算（TryTrackPositionToLookAnglesDeg）；
 *   2) 指向来源优先级矩阵（显式 dwell > 指定航迹 > scan_center 回退，
 *      ResolveSttTrackFollowingPointing）；
 *   3) 挂架指向解算链（含限位 clamp）；
 *   4) Session 端到端：指定目标 → 航迹确认 → 发射 boresight 跟随航迹 →
 *      清除指定/显式 dwell 不跟随 → 航迹丢失自动回退 TWS +
 *      L2 结果 / L3 调试视图 / 生命周期事件暴露；
 *   5) TWS 生效模式（含 STT 回退）下 session 级扫描动画：发射 boresight
 *      逐周期按扫描表推进（boundaries.md 已知限制修复）；
 *   6) 限时指定指令（designation_duration_cycles）：窗口内捕获 confirmed 航迹
 *      后持续跟随（不受窗口限制）；窗口耗尽仍未捕获 → 指令作废（回到扫描，
 *      kAcquisitionTimeout 经结果/事件暴露）。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/ArTrackLifecycleRecorder.h"
#include "1q/airborne_radar/session/ArTrackOutputDebugView.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"
#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/pipeline/ScanScheduleResolver.h"

namespace airborne_radar {
namespace tests {
namespace {

using airborne_radar::session::ArCycleInput;
using airborne_radar::session::ArCycleResult;
using airborne_radar::session::ArDesignationRevertReason;
using airborne_radar::session::ArPlatformInput;
using airborne_radar::session::ArSession;
using airborne_radar::session::ArTargetInput;
using airborne_radar::session::ArTrackLifecycleEvent;
using airborne_radar::session::ArTrackLifecycleEventKind;
using airborne_radar::session::ArTrackLifecycleRecorder;
using airborne_radar::session::ArTrackOutputDebugView;
using airborne_radar::session::ArTrackOutputDebugViewBuilder;
using airborne_radar::session::TrackOutputFrame;

config::ArSessionConfig MakeDetectionFocusedConfig() {
  config::ArSessionConfig cfg;
  cfg.policy.detection = config::profiles::kDetectionPriorityDetection;
  cfg.policy.tracking = config::profiles::kFastAssociationTracking;
  cfg.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
  cfg.hardware.rcs_physics.enable_physical_rcs = false;
  cfg.hardware.rcs_physics.physics_mix_ratio = 0.0f;
  return cfg;
}

ArPlatformInput MakePlatformInput() {
  oneq::coordinate::EcefPositionM platform_ecef;
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 30.0;
  platform_lla.longitude_deg = 120.0;
  platform_lla.altitude_m = 1000.0;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(platform_lla, &platform_ecef));

  ArPlatformInput platform;
  platform.platform_position_ecef_m = platform_ecef;
  platform.platform_velocity_mps.x_mps = 0.0;
  platform.platform_velocity_mps.y_mps = 0.0;
  platform.platform_velocity_mps.z_mps = 0.0;
  platform.platform_attitude_deg.yaw_deg = 0.0;
  platform.platform_attitude_deg.pitch_deg = 0.0;
  platform.platform_attitude_deg.roll_deg = 0.0;
  return platform;
}

// 静止目标：位于平台东北偏上方向（雷达局部约 az=54.7°、el=14.4°），
// 保证检测稳定且指向断言不受运动/一周期滞后影响。
// 世界真值（LLA + 零 ECEF 速度）经公共入口转换为平台锚点 ENU 后直填。
ArTargetInput MakeStationaryTargetInput() {
  oneq::coordinate::EcefPositionM target_ecef;
  oneq::coordinate::LlaPositionDegM target_lla;
  target_lla.latitude_deg = 30.0007;
  target_lla.longitude_deg = 120.0012;
  target_lla.altitude_m = 1035.0;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));

  oneq::coordinate::ExternalKinematics kinematics;
  kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  kinematics.position_ecef_m = target_ecef;

  oneq::coordinate::LlaPositionDegM anchor_lla;
  EXPECT_TRUE(oneq::coordinate::TryEcefToLla(MakePlatformInput().platform_position_ecef_m,
                                             &anchor_lla));
  oneq::coordinate::EnuSceneState enu;
  EXPECT_TRUE(oneq::coordinate::TryMakeEnuSceneState(kinematics, anchor_lla, &enu));

  ArTargetInput target;
  target.target_id = 9001U;
  target.position_x = static_cast<float>(enu.position_enu_m.east_m);
  target.position_y = static_cast<float>(enu.position_enu_m.north_m);
  target.position_z = static_cast<float>(enu.position_enu_m.up_m);
  target.rcs = 1.7f;
  target.swerling_type = 2;
  return target;
}

ArCycleInput MakeCycleInput(std::uint32_t cycle_index, const ArPlatformInput& platform,
                            const std::vector<ArTargetInput>& targets) {
  ArCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = static_cast<double>(cycle_index - 1U) * 0.5;
  input.dt_sec = 0.5;
  input.platform = platform;
  input.targets = targets;
  return input;
}

// 运行 [start_cycle, max_cycles] 区间的周期直到指定条件满足（上限保护），
// 返回最后一次结果。周期号必须连续（ArSession 有世界时钟连续性检查）。
template <typename Predicate>
ArCycleResult RunUntil(ArSession* session, const ArPlatformInput& platform,
                       const std::vector<ArTargetInput>& targets, std::uint32_t start_cycle,
                       std::uint32_t max_cycles, Predicate predicate, std::uint32_t* last_cycle) {
  ArCycleResult last;
  for (std::uint32_t cycle = start_cycle; cycle <= max_cycles; ++cycle) {
    last = session->StepWithResult(MakeCycleInput(cycle, platform, targets));
    *last_cycle = cycle;
    if (predicate(last)) {
      break;
    }
  }
  return last;
}

oneq::electromagnetics::RfSceneDirection Normalize(
    const oneq::electromagnetics::RfSceneDirection& v) {
  const double norm = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  if (norm <= 0.0) {
    return v;
  }
  return oneq::electromagnetics::RfSceneDirection{v.x / norm, v.y / norm, v.z / norm};
}

double Dot(const oneq::electromagnetics::RfSceneDirection& lhs,
           const oneq::electromagnetics::RfSceneDirection& rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

// 未配置同址路径的近场干扰：距离小于远场准则且未命中 co_site_paths → 接收侧校验拒绝（kRejectedExecution）。
void AddUnconfiguredCoSiteInterference(ArCycleInput* input) {
  ASSERT_NE(input, nullptr);
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity = {999U, 1U};
  emission.position_ecef_m = input->platform.platform_position_ecef_m;
  const double radar_frequency_hz =
      static_cast<double>(MakeDetectionFocusedConfig().hardware.transmitter.frequency_hz);
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      input->cycle_start_time_s, input->dt_sec, radar_frequency_hz, 20.0e6, 1.0,
      &emission.waveform));
  input->interference.world_cycle_index = input->cycle_index;
  input->interference.window_start_time_s = input->cycle_start_time_s;
  input->interference.window_duration_s = input->dt_sec;
  input->interference.emissions.push_back(emission);
}

// ---------------------------------------------------------------------------
// 1) 雷达局部位置 → az/el 换算
// ---------------------------------------------------------------------------

TEST(ArSttTrackFollowTest, TrackPositionToLookAnglesMath) {
  config::AzimuthElevationDeg pointing;
  // 正东水平 → az=0, el=0。
  ASSERT_TRUE(signal::pipeline::TryTrackPositionToLookAnglesDeg(1000.0f, 0.0f, 0.0f, &pointing));
  EXPECT_NEAR(pointing.az_deg, 0.0f, 1.0e-4f);
  EXPECT_NEAR(pointing.el_deg, 0.0f, 1.0e-4f);
  // 正北水平 → az=90。
  ASSERT_TRUE(signal::pipeline::TryTrackPositionToLookAnglesDeg(0.0f, 1000.0f, 0.0f, &pointing));
  EXPECT_NEAR(pointing.az_deg, 90.0f, 1.0e-4f);
  EXPECT_NEAR(pointing.el_deg, 0.0f, 1.0e-4f);
  // 天顶 → el=90。
  ASSERT_TRUE(signal::pipeline::TryTrackPositionToLookAnglesDeg(0.0f, 0.0f, 1000.0f, &pointing));
  EXPECT_NEAR(pointing.el_deg, 90.0f, 1.0e-4f);
  // 东北偏上 → az≈45, el≈atan2(1000, sqrt(2)*1000)。
  ASSERT_TRUE(
      signal::pipeline::TryTrackPositionToLookAnglesDeg(1000.0f, 1000.0f, 1000.0f, &pointing));
  EXPECT_NEAR(pointing.az_deg, 45.0f, 1.0e-4f);
  EXPECT_NEAR(pointing.el_deg, 35.2644f, 1.0e-3f);

  // 无效输入：NaN / 零范数。
  EXPECT_FALSE(signal::pipeline::TryTrackPositionToLookAnglesDeg(
      std::numeric_limits<float>::quiet_NaN(), 1.0f, 1.0f, &pointing));
  EXPECT_FALSE(signal::pipeline::TryTrackPositionToLookAnglesDeg(0.0f, 0.0f, 0.0f, &pointing));
}

// ---------------------------------------------------------------------------
// 2) 指向来源优先级矩阵（显式 dwell > 指定航迹 > scan_center 回退）
// ---------------------------------------------------------------------------

TEST(ArSttTrackFollowTest, PointingPriorityMatrix) {
  config::ArEffectiveOrientationConfig orientation;
  orientation.work_mode = config::ArWorkMode::kStt;
  orientation.scan_center_deg.az_deg = 10.0f;
  orientation.scan_center_deg.el_deg = 5.0f;
  const config::AzimuthElevationDeg track_pointing{45.0f, 12.0f};

  // 优先级 1：显式 dwell 非零 → 现状语义（scan_center + dwell），不跟随航迹。
  const config::AzimuthElevationDeg explicit_dwell{3.0f, -2.0f};
  const signal::pipeline::SttTrackFollowingResolution dwell_override =
      signal::pipeline::ResolveSttTrackFollowingPointing(orientation, explicit_dwell, true, true,
                                                         track_pointing);
  EXPECT_FALSE(dwell_override.track_following_active);
  EXPECT_EQ(dwell_override.scan_center_deg.az_deg, 10.0f);
  EXPECT_EQ(dwell_override.scan_center_deg.el_deg, 5.0f);
  EXPECT_EQ(dwell_override.dwell_center_deg.az_deg, 3.0f);
  EXPECT_EQ(dwell_override.dwell_center_deg.el_deg, -2.0f);

  // 优先级 2：STT + 指定 + confirmed + 无显式 dwell → 跟随航迹。
  const signal::pipeline::SttTrackFollowingResolution following =
      signal::pipeline::ResolveSttTrackFollowingPointing(orientation, config::AzimuthElevationDeg(),
                                                         true, true, track_pointing);
  EXPECT_TRUE(following.track_following_active);
  EXPECT_EQ(following.scan_center_deg.az_deg, 45.0f);
  EXPECT_EQ(following.scan_center_deg.el_deg, 12.0f);
  EXPECT_EQ(following.dwell_center_deg.az_deg, 0.0f);
  EXPECT_EQ(following.dwell_center_deg.el_deg, 0.0f);

  // 优先级 3a：指定但航迹未确认 → 回退 scan_center。
  const signal::pipeline::SttTrackFollowingResolution not_confirmed =
      signal::pipeline::ResolveSttTrackFollowingPointing(orientation, config::AzimuthElevationDeg(),
                                                         true, false, track_pointing);
  EXPECT_FALSE(not_confirmed.track_following_active);
  EXPECT_EQ(not_confirmed.scan_center_deg.az_deg, 10.0f);
  EXPECT_EQ(not_confirmed.scan_center_deg.el_deg, 5.0f);

  // 优先级 3b：未指定 → 回退 scan_center。
  const signal::pipeline::SttTrackFollowingResolution no_designation =
      signal::pipeline::ResolveSttTrackFollowingPointing(orientation, config::AzimuthElevationDeg(),
                                                         false, true, track_pointing);
  EXPECT_FALSE(no_designation.track_following_active);
  EXPECT_EQ(no_designation.scan_center_deg.az_deg, 10.0f);

  // 优先级 3c：指定航迹指向非有限 → 不跟随。
  const config::AzimuthElevationDeg invalid_pointing{std::numeric_limits<float>::quiet_NaN(), 0.0f};
  const signal::pipeline::SttTrackFollowingResolution invalid =
      signal::pipeline::ResolveSttTrackFollowingPointing(orientation, config::AzimuthElevationDeg(),
                                                         true, true, invalid_pointing);
  EXPECT_FALSE(invalid.track_following_active);

  // 指定在非 STT 模式下被忽略（TWS + 指定 + confirmed → 不跟随）。
  config::ArEffectiveOrientationConfig tws_orientation = orientation;
  tws_orientation.work_mode = config::ArWorkMode::kTws;
  const signal::pipeline::SttTrackFollowingResolution tws_ignored =
      signal::pipeline::ResolveSttTrackFollowingPointing(
          tws_orientation, config::AzimuthElevationDeg(), true, true, track_pointing);
  EXPECT_FALSE(tws_ignored.track_following_active);
  EXPECT_EQ(tws_ignored.scan_center_deg.az_deg, 10.0f);
}

// ---------------------------------------------------------------------------
// 3) 挂架指向解算链（航迹跟随 + 限位 clamp）
// ---------------------------------------------------------------------------

TEST(ArSttTrackFollowTest, MountFramePointingFollowsTrackAndClamps) {
  config::ArEffectiveOrientationConfig orientation;
  orientation.work_mode = config::ArWorkMode::kStt;
  orientation.scan_center_deg.az_deg = 0.0f;
  orientation.scan_center_deg.el_deg = 0.0f;
  const config::PlatformAttitudeDeg attitude{};  // 零姿态，机体稳定。

  // 限位内目标：雷达局部 (10000, 10000, 2000) → az=45, el≈8.05。
  config::AzimuthElevationDeg track_pointing;
  ASSERT_TRUE(signal::pipeline::TryTrackPositionToLookAnglesDeg(10000.0f, 10000.0f, 2000.0f,
                                                                &track_pointing));
  const signal::pipeline::SttTrackFollowingResolution resolution =
      signal::pipeline::ResolveSttTrackFollowingPointing(orientation, config::AzimuthElevationDeg(),
                                                         true, true, track_pointing);
  ASSERT_TRUE(resolution.track_following_active);
  config::ArEffectiveOrientationConfig effective = orientation;
  effective.scan_center_deg = resolution.scan_center_deg;
  const config::AzimuthElevationDeg pointing =
      signal::detection::BeamControlResolver::ResolveMountFrameBeamPointing(
          effective, attitude, resolution.dwell_center_deg);
  EXPECT_NEAR(pointing.az_deg, 45.0f, 1.0e-3f);
  EXPECT_NEAR(pointing.el_deg, 8.0495f, 1.0e-3f);

  // 限位外目标（az=80 > 60）：clamp 到机械/电子限位交集边界。
  config::AzimuthElevationDeg beyond_limits;
  ASSERT_TRUE(
      signal::pipeline::TryTrackPositionToLookAnglesDeg(5000.0f, 30000.0f, 0.0f, &beyond_limits));
  ASSERT_GT(beyond_limits.az_deg, 60.0f);
  const signal::pipeline::SttTrackFollowingResolution beyond_resolution =
      signal::pipeline::ResolveSttTrackFollowingPointing(orientation, config::AzimuthElevationDeg(),
                                                         true, true, beyond_limits);
  ASSERT_TRUE(beyond_resolution.track_following_active);
  effective.scan_center_deg = beyond_resolution.scan_center_deg;
  const config::AzimuthElevationDeg clamped =
      signal::detection::BeamControlResolver::ResolveMountFrameBeamPointing(
          effective, attitude, beyond_resolution.dwell_center_deg);
  EXPECT_NEAR(clamped.az_deg, 60.0f, 1.0e-3f);
  EXPECT_NEAR(clamped.el_deg, 0.0f, 1.0e-3f);
}

// ---------------------------------------------------------------------------
// 4) Session 端到端：指定 → 确认 → 跟随 → 清除/覆盖 → 丢失回退
// ---------------------------------------------------------------------------

oneq::electromagnetics::RfSceneDirection TargetDirectionFromEnu(const ArTargetInput& target) {
  const double norm = std::sqrt(static_cast<double>(target.position_x) * target.position_x +
                                static_cast<double>(target.position_y) * target.position_y +
                                static_cast<double>(target.position_z) * target.position_z);
  oneq::coordinate::Vector3d enu_dir;
  enu_dir.x = target.position_x / norm;
  enu_dir.y = target.position_y / norm;
  enu_dir.z = target.position_z / norm;

  oneq::coordinate::LlaPositionDegM anchor_lla;
  EXPECT_TRUE(oneq::coordinate::TryEcefToLla(MakePlatformInput().platform_position_ecef_m,
                                             &anchor_lla));
  oneq::coordinate::Vector3d ecef_dir;
  EXPECT_TRUE(oneq::coordinate::TryEnuToEcefDirection(enu_dir, anchor_lla, &ecef_dir));
  return oneq::electromagnetics::RfSceneDirection{ecef_dir.x, ecef_dir.y, ecef_dir.z};
}

TEST(ArSttTrackFollowTest, SessionFollowsDesignatedTrackThenRevertsOnLoss) {
  const ArPlatformInput platform = MakePlatformInput();
  const ArTargetInput target = MakeStationaryTargetInput();
  const std::vector<ArTargetInput> targets{target};
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());
  ArTrackLifecycleRecorder recorder;
  session.AttachTrackLifecycleRecorder(&recorder);
  const oneq::electromagnetics::RfSceneDirection target_direction =
      TargetDirectionFromEnu(target);

  // 阶段 0（周期 1-8）：TWS 基线——无指定，生效模式 TWS，无指定字段。
  std::uint32_t last_cycle = 0U;
  ArCycleResult result = RunUntil(
      &session, platform, targets, 1U, 8U,
      [](const ArCycleResult& r) {
        return !r.output_frame.tracks.empty() &&
               r.output_frame.tracks.front().status ==
                   airborne_radar::session::TrackStatus::kConfirmed;
      },
      &last_cycle);
  ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
  EXPECT_EQ(result.effective_work_mode, config::ArWorkMode::kTws);
  EXPECT_FALSE(result.designation_active);
  EXPECT_FALSE(result.designation_reverted_to_tws);
  // TWS 基线：session 级波束按扫描表逐周期推进（boundaries.md 已知限制修复），
  // 发射 boresight 随周期移动而非静止于 scan_center；连续两周期指向不同。
  ASSERT_FALSE(result.emission_frame.emissions.empty());
  const oneq::electromagnetics::RfSceneDirection baseline_boresight =
      Normalize(result.emission_frame.emissions.front().antenna.boresight_ecef);
  const ArCycleResult baseline_next =
      session.StepWithResult(MakeCycleInput(last_cycle + 1U, platform, targets));
  ASSERT_EQ(baseline_next.status, airborne_radar::session::ArCycleStatus::kCompleted);
  ASSERT_FALSE(baseline_next.emission_frame.emissions.empty());
  const oneq::electromagnetics::RfSceneDirection baseline_next_boresight =
      Normalize(baseline_next.emission_frame.emissions.front().antenna.boresight_ecef);
  EXPECT_LT(Dot(baseline_boresight, baseline_next_boresight), 0.999f)
      << "TWS 基线发射 boresight 应随扫描表逐周期推进";
  ++last_cycle;  // 消费掉动画断言周期，保证后续周期号连续。

  // 阶段 1（后续周期）：patch STT + 指定目标 → 确认后航迹跟随，boresight 指向目标。
  config::ArRuntimeConfigPatch stt_patch;
  stt_patch.has_work_mode = true;
  stt_patch.work_mode = config::ArWorkMode::kStt;
  stt_patch.has_designated_target_id = true;
  stt_patch.designated_external_target_id = target.target_id;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(stt_patch));

  result = RunUntil(
      &session, platform, targets, last_cycle + 1U, 16U,
      [](const ArCycleResult& r) { return r.designation_active; }, &last_cycle);
  ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
  EXPECT_TRUE(result.designation_active);
  EXPECT_EQ(result.effective_work_mode, config::ArWorkMode::kStt);
  EXPECT_EQ(result.designated_target_id, target.target_id);
  EXPECT_FALSE(result.designation_reverted_to_tws);
  // 再跑一个周期：指向跟随指定航迹（发射 boresight ≈ 目标方向）。
  result = session.StepWithResult(MakeCycleInput(last_cycle + 1U, platform, targets));
  ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
  ASSERT_FALSE(result.emission_frame.emissions.empty());
  const oneq::electromagnetics::RfSceneDirection follow_boresight =
      Normalize(result.emission_frame.emissions.front().antenna.boresight_ecef);
  EXPECT_GT(Dot(follow_boresight, target_direction), 0.99)
      << "STT 指定航迹跟随下发射指向应指向目标";
  ++last_cycle;  // 消费掉 boresight 断言周期，保证后续周期号连续。

  // 阶段 2：指定航迹丢失 → 自动回退 TWS + 事件暴露。
  // 目标保留在输入但位移 20km 且 RCS 降到 1e-6 m²：SNR 远低于检测门
  // （无新量测/新航迹；大距离也不会被大协方差关联吞掉），旧航迹连续失配
  // → lost。目标必须留在输入表，ArTrackLifecycleRecorder 只遍历输入目标
  // （消失目标不产生事件）。
  ArTargetInput silent_target = target;
  silent_target.position_x += 20000.0f;
  silent_target.rcs = 1.0e-6f;
  const std::vector<ArTargetInput> silent_targets{silent_target};
  std::uint32_t revert_cycle = 0U;
  result = RunUntil(
      &session, platform, silent_targets, last_cycle + 1U, 22U,
      [](const ArCycleResult& r) { return r.designation_reverted_to_tws; }, &revert_cycle);
  ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
  EXPECT_TRUE(result.designation_reverted_to_tws);
  EXPECT_EQ(result.effective_work_mode, config::ArWorkMode::kTws);
  EXPECT_FALSE(result.designation_active);
  EXPECT_EQ(result.designation_revert_reason, ArDesignationRevertReason::kTrackLost);
  EXPECT_EQ(result.designated_target_id, target.target_id);

  // 生命周期记录器：回退转换沿产生 kDesignationDropped（成因 kTrackLost）。
  // 注意：事件窗口只在回退周期（revert_cycle）有效，必须在跑下一周期前断言。
  const std::vector<ArTrackLifecycleEvent>& events = recorder.GetLastEvents();
  const auto dropped =
      std::find_if(events.begin(), events.end(), [](const ArTrackLifecycleEvent& event) {
        return event.kind == ArTrackLifecycleEventKind::kDesignationDropped;
      });
  ASSERT_NE(dropped, events.end());
  EXPECT_EQ(dropped->external_target_id, target.target_id);
  EXPECT_EQ(dropped->designation_revert_reason, ArDesignationRevertReason::kTrackLost);

  // 回退后生效模式 TWS：session 级波束恢复扫描表逐周期推进（"回 TWS"不再
  // 回到静态指向）；回退周期与下一周期发射 boresight 不同。
  ASSERT_FALSE(result.emission_frame.emissions.empty());
  const oneq::electromagnetics::RfSceneDirection revert_boresight =
      Normalize(result.emission_frame.emissions.front().antenna.boresight_ecef);
  const ArCycleResult revert_next =
      session.StepWithResult(MakeCycleInput(revert_cycle + 1U, platform, silent_targets));
  ASSERT_EQ(revert_next.status, airborne_radar::session::ArCycleStatus::kCompleted);
  ASSERT_FALSE(revert_next.emission_frame.emissions.empty());
  const oneq::electromagnetics::RfSceneDirection revert_next_boresight =
      Normalize(revert_next.emission_frame.emissions.front().antenna.boresight_ecef);
  EXPECT_LT(Dot(revert_boresight, revert_next_boresight), 0.999f)
      << "STT 回退 TWS 后发射 boresight 应恢复扫描表逐周期推进";
  const std::uint32_t revert_animation_cycle = revert_cycle + 1U;

  // L3 调试视图：回退状态与成因可见（使用回退周期 result）。
  const ArTrackOutputDebugView debug_view = ArTrackOutputDebugViewBuilder::Build(
      MakeCycleInput(revert_cycle, platform, silent_targets), result);
  EXPECT_EQ(debug_view.effective_work_mode, config::ArWorkMode::kTws);
  EXPECT_TRUE(debug_view.designation_reverted_to_tws);
  EXPECT_EQ(debug_view.designated_target_id, target.target_id);
  EXPECT_EQ(debug_view.designation_revert_reason, ArDesignationRevertReason::kTrackLost);

  // 阶段 3：清除指定后不再有回退状态（无指定 = 现状 STT 驻留语义，不视为回退）。
  config::ArRuntimeConfigPatch clear_patch;
  clear_patch.has_designated_target_id = true;
  clear_patch.designated_external_target_id = 0U;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(clear_patch));
  result = session.StepWithResult(MakeCycleInput(revert_animation_cycle + 1U, platform,
                                                 silent_targets));
  ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
  EXPECT_EQ(result.designated_target_id, 0U);
  EXPECT_FALSE(result.designation_reverted_to_tws);
  EXPECT_FALSE(result.designation_active);
}

TEST(ArSttTrackFollowTest, SessionExplicitDwellOverridesTrackFollowing) {
  const ArPlatformInput platform = MakePlatformInput();
  const ArTargetInput target = MakeStationaryTargetInput();
  const std::vector<ArTargetInput> targets{target};
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());

  // 阶段 1（周期 1-8）：STT + 指定 + 显式 dwell → 指向按现状语义（scan_center + dwell），
  // 不跟随航迹（designation_active == false 且不构成回退）。
  config::ArRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::ArWorkMode::kStt;
  patch.has_designated_target_id = true;
  patch.designated_external_target_id = target.target_id;
  patch.has_dwell_center_deg = true;
  patch.dwell_center_deg.az_deg = 5.0f;
  patch.dwell_center_deg.el_deg = 3.0f;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(patch));

  std::uint32_t last_cycle = 0U;
  ArCycleResult result = RunUntil(
      &session, platform, targets, 1U, 8U,
      [](const ArCycleResult& r) {
        return !r.output_frame.tracks.empty() &&
               r.output_frame.tracks.front().status ==
                   airborne_radar::session::TrackStatus::kConfirmed;
      },
      &last_cycle);
  ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
  EXPECT_EQ(result.effective_work_mode, config::ArWorkMode::kStt);
  EXPECT_FALSE(result.designation_active) << "显式 dwell 覆盖时不得标记航迹跟随生效";
  EXPECT_FALSE(result.designation_reverted_to_tws) << "显式 dwell 覆盖不构成回退";

  // 发射 boresight = scan_center(0,0) + dwell(5,3)（限位内），不指向目标。
  result = session.StepWithResult(MakeCycleInput(last_cycle + 1U, platform, targets));
  ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
  ASSERT_FALSE(result.emission_frame.emissions.empty());
  const oneq::electromagnetics::RfSceneDirection boresight =
      Normalize(result.emission_frame.emissions.front().antenna.boresight_ecef);
  const oneq::electromagnetics::RfSceneDirection target_direction =
      TargetDirectionFromEnu(target);
  EXPECT_LT(Dot(boresight, target_direction), 0.99)
      << "显式 dwell 优先于航迹跟随，发射指向不应指向目标";
  ++last_cycle;  // 消费掉 boresight 断言周期，保证后续周期号连续。

  // 阶段 2：清除 dwell → 恢复航迹跟随。
  config::ArRuntimeConfigPatch clear_dwell;
  clear_dwell.has_dwell_center_deg = true;
  clear_dwell.dwell_center_deg.az_deg = 0.0f;
  clear_dwell.dwell_center_deg.el_deg = 0.0f;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(clear_dwell));
  result = session.StepWithResult(MakeCycleInput(last_cycle + 2U, platform, targets));
  ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
  EXPECT_TRUE(result.designation_active) << "清除显式 dwell 后应恢复航迹跟随";
  ASSERT_FALSE(result.emission_frame.emissions.empty());
  const oneq::electromagnetics::RfSceneDirection follow_boresight =
      Normalize(result.emission_frame.emissions.front().antenna.boresight_ecef);
  EXPECT_GT(Dot(follow_boresight, target_direction), 0.99) << "清除 dwell 后发射指向应跟随指定航迹";
}

// TWS 生效模式（含 STT 回退）下 session 级波束动画：发射 boresight 逐周期
// 按扫描表推进（boundaries.md 已知限制修复），不再静止于 base scan_center + dwell。
TEST(ArSttTrackFollowTest, SessionTwsScanAnimatesBeamAcrossCycles) {
  const ArPlatformInput platform = MakePlatformInput();
  const ArTargetInput target = MakeStationaryTargetInput();
  const std::vector<ArTargetInput> targets{target};
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());

  // 连续 4 个 TWS 周期：每相邻两周期发射 boresight 都应不同（扫描推进）。
  std::vector<oneq::electromagnetics::RfSceneDirection> boresights;
  for (std::uint32_t cycle = 1U; cycle <= 4U; ++cycle) {
    const ArCycleResult result = session.StepWithResult(MakeCycleInput(cycle, platform, targets));
    ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
    ASSERT_FALSE(result.emission_frame.emissions.empty());
    boresights.push_back(
        Normalize(result.emission_frame.emissions.front().antenna.boresight_ecef));
  }
  std::size_t animated_pairs = 0U;
  for (std::size_t i = 0U; i + 1U < boresights.size(); ++i) {
    if (Dot(boresights[i], boresights[i + 1U]) < 0.999f) {
      ++animated_pairs;
    }
  }
  EXPECT_EQ(animated_pairs, boresights.size() - 1U)
      << "TWS 模式下每周期发射 boresight 都应按扫描表逐周期推进";
}

// 限时指定指令（捕获超时作废）：窗口内始终未捕获 confirmed 航迹 → 窗口期
// 每周期回退报告（kTrackNotConfirmed）且波束继续扫描；窗口耗尽周期成因变为
// kAcquisitionTimeout（kDesignationDropped 事件）；作废后指定清零、生效模式
// 按扫描（kTws）处理且波束继续逐周期推进。
TEST(ArSttTrackFollowTest, SessionDesignationExpiresOnAcquisitionTimeout) {
  const ArPlatformInput platform = MakePlatformInput();
  // 静默目标：位移 20km 且 RCS 远低于检测门（与回退测试的静默口径一致）——
  // 任何周期都不会被检测/建航迹 → 窗口内无法捕获（近距离目标即使 RCS 极小
  // 仍可被检测确认，不能用于本测试）。
  ArTargetInput silent = MakeStationaryTargetInput();
  silent.position_x += 20000.0f;
  silent.rcs = 1.0e-9f;
  const std::vector<ArTargetInput> targets{silent};
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());
  ArTrackLifecycleRecorder recorder;
  session.AttachTrackLifecycleRecorder(&recorder);

  // 指令：STT + 指定 9001 + 3 周期捕获窗口（生效于周期 1 → 窗口 1..3，周期 4 作废）。
  config::ArRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::ArWorkMode::kStt;
  patch.has_designated_target_id = true;
  patch.designated_external_target_id = silent.target_id;
  patch.has_designation_duration_cycles = true;
  patch.designation_duration_cycles = 3U;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(patch));

  // 窗口内（周期 1-3）：每周期回退报告（kTrackNotConfirmed），波束继续扫描
  // （相邻周期 boresight 不同），指定目标 ID 保持。
  oneq::electromagnetics::RfSceneDirection prev_boresight{};
  bool has_prev_boresight = false;
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    const ArCycleResult r = session.StepWithResult(MakeCycleInput(cycle, platform, targets));
    ASSERT_EQ(r.status, airborne_radar::session::ArCycleStatus::kCompleted);
    EXPECT_EQ(r.effective_work_mode, config::ArWorkMode::kTws) << "窗口内未捕获 → 扫描";
    EXPECT_TRUE(r.designation_reverted_to_tws);
    EXPECT_EQ(r.designation_revert_reason, ArDesignationRevertReason::kTrackNotConfirmed);
    EXPECT_EQ(r.designated_target_id, silent.target_id);
    EXPECT_FALSE(r.designation_active);
    ASSERT_FALSE(r.emission_frame.emissions.empty());
    const oneq::electromagnetics::RfSceneDirection boresight =
        Normalize(r.emission_frame.emissions.front().antenna.boresight_ecef);
    if (has_prev_boresight) {
      EXPECT_LT(Dot(prev_boresight, boresight), 0.999f) << "窗口内未捕获期间继续扫描";
    }
    prev_boresight = boresight;
    has_prev_boresight = true;
  }

  // 周期 4（作废沿）：成因变为 kAcquisitionTimeout，ID 保留供事件关联。
  const ArCycleResult expiry = session.StepWithResult(MakeCycleInput(4U, platform, targets));
  ASSERT_EQ(expiry.status, airborne_radar::session::ArCycleStatus::kCompleted);
  EXPECT_TRUE(expiry.designation_reverted_to_tws);
  EXPECT_EQ(expiry.designation_revert_reason, ArDesignationRevertReason::kAcquisitionTimeout);
  EXPECT_EQ(expiry.designated_target_id, silent.target_id);
  EXPECT_EQ(expiry.effective_work_mode, config::ArWorkMode::kTws);
  // 记录器：作废沿产生 kDesignationDropped（成因 kAcquisitionTimeout）。
  const std::vector<ArTrackLifecycleEvent>& events = recorder.GetLastEvents();
  const auto dropped = std::find_if(
      events.begin(), events.end(), [](const ArTrackLifecycleEvent& event) {
        return event.kind == ArTrackLifecycleEventKind::kDesignationDropped;
      });
  ASSERT_NE(dropped, events.end());
  EXPECT_EQ(dropped->external_target_id, silent.target_id);
  EXPECT_EQ(dropped->designation_revert_reason, ArDesignationRevertReason::kAcquisitionTimeout);

  // 作废后（周期 5-6）：指定清零、无回退报告、生效模式仍按扫描（kTws）。
  for (std::uint32_t cycle = 5U; cycle <= 6U; ++cycle) {
    const ArCycleResult r = session.StepWithResult(MakeCycleInput(cycle, platform, targets));
    ASSERT_EQ(r.status, airborne_radar::session::ArCycleStatus::kCompleted);
    EXPECT_EQ(r.designated_target_id, 0U) << "作废后指定清零";
    EXPECT_FALSE(r.designation_reverted_to_tws);
    EXPECT_EQ(r.designation_revert_reason, ArDesignationRevertReason::kNone);
    EXPECT_EQ(r.effective_work_mode, config::ArWorkMode::kTws) << "作废后回到扫描";
  }
}

// 限时指定指令（窗口内捕获成功）：窗口内 confirmed 航迹出现 → 跟随航迹；
// 超过窗口截止周期后仍持续跟随（窗口只约束首次捕获，捕获后不受窗口限制、
// 不作废），记录器无 kAcquisitionTimeout 事件。
TEST(ArSttTrackFollowTest, SessionDesignationAcquiresWithinWindowKeepsFollowingPastDeadline) {
  const ArPlatformInput platform = MakePlatformInput();
  const ArTargetInput target = MakeStationaryTargetInput();
  const std::vector<ArTargetInput> targets{target};
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());
  ArTrackLifecycleRecorder recorder;
  session.AttachTrackLifecycleRecorder(&recorder);
  const oneq::electromagnetics::RfSceneDirection target_direction =
      TargetDirectionFromEnu(target);

  // 指令：STT + 指定 + 4 周期窗口（生效于周期 1 → 窗口 1..4，周期 5 截止）。
  config::ArRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::ArWorkMode::kStt;
  patch.has_designated_target_id = true;
  patch.designated_external_target_id = target.target_id;
  patch.has_designation_duration_cycles = true;
  patch.designation_duration_cycles = 4U;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(patch));

  // 窗口内（周期 1-4）捕获并跟随。
  std::uint32_t last_cycle = 0U;
  ArCycleResult result = RunUntil(
      &session, platform, targets, 1U, 4U,
      [](const ArCycleResult& r) { return r.designation_active; }, &last_cycle);
  ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
  ASSERT_TRUE(result.designation_active) << "窗口内应捕获并跟随";
  EXPECT_EQ(result.effective_work_mode, config::ArWorkMode::kStt);
  EXPECT_EQ(result.designation_revert_reason, ArDesignationRevertReason::kNone);
  // 指向使用上一周期航迹帧（滞后一周期）：再跑一个周期后 boresight 才跟随航迹。
  result = session.StepWithResult(MakeCycleInput(last_cycle + 1U, platform, targets));
  ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
  ASSERT_TRUE(result.designation_active);
  ASSERT_FALSE(result.emission_frame.emissions.empty());
  const oneq::electromagnetics::RfSceneDirection follow_boresight =
      Normalize(result.emission_frame.emissions.front().antenna.boresight_ecef);
  EXPECT_GT(Dot(follow_boresight, target_direction), 0.99) << "跟随指定航迹";
  const std::uint32_t follow_cycle = last_cycle + 1U;  // 消费掉的周期号

  // 超过窗口截止周期（duration=4 → deadline=5）后仍持续跟随，无作废报告。
  std::uint32_t past_deadline_start = follow_cycle + 1U;
  if (past_deadline_start < 5U) {
    past_deadline_start = 5U;
  }
  for (std::uint32_t cycle = past_deadline_start; cycle <= past_deadline_start + 1U; ++cycle) {
    result = session.StepWithResult(MakeCycleInput(cycle, platform, targets));
    ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
    EXPECT_TRUE(result.designation_active) << "窗口只约束首次捕获，捕获后不受窗口限制";
    EXPECT_EQ(result.effective_work_mode, config::ArWorkMode::kStt);
    EXPECT_EQ(result.designation_revert_reason, ArDesignationRevertReason::kNone);
    EXPECT_EQ(result.designated_target_id, target.target_id);
  }
  // 记录器：窗口内已捕获，无 kAcquisitionTimeout 事件。
  const std::vector<ArTrackLifecycleEvent>& events = recorder.GetLastEvents();
  EXPECT_TRUE(std::none_of(events.begin(), events.end(), [](const ArTrackLifecycleEvent& event) {
    return event.designation_revert_reason == ArDesignationRevertReason::kAcquisitionTimeout;
  }));
}

// 失败周期不消耗指定窗口（审核修复）：post-emission 拒绝周期后，作废沿
// 在后续首个成功周期才可见（kAcquisitionTimeout 报告不被失败周期吞掉）。
TEST(ArSttTrackFollowTest, RejectedCycleDoesNotConsumeDesignationWindow) {
  const ArPlatformInput platform = MakePlatformInput();
  // 静默目标（移远 + 极小 RCS）：窗口内无法捕获 → 到期作废。
  ArTargetInput silent = MakeStationaryTargetInput();
  silent.position_x += 20000.0f;
  silent.rcs = 1.0e-9f;
  const std::vector<ArTargetInput> targets{silent};
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());

  // 指令：STT + 指定 + 1 周期窗口（生效于周期 1 → deadline = 2）。
  config::ArRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::ArWorkMode::kStt;
  patch.has_designated_target_id = true;
  patch.designated_external_target_id = silent.target_id;
  patch.has_designation_duration_cycles = true;
  patch.designation_duration_cycles = 1U;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(patch));

  // 周期 1：窗口内（pending，未捕获回退报告）。
  ArCycleResult result = session.StepWithResult(MakeCycleInput(1U, platform, targets));
  ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
  EXPECT_TRUE(result.designation_reverted_to_tws);
  EXPECT_EQ(result.designation_revert_reason, ArDesignationRevertReason::kTrackNotConfirmed);

  // 周期 2（截止周期）：post-emission 拒绝——本周期不消耗窗口。
  ArCycleInput rejected_input = MakeCycleInput(2U, platform, targets);
  AddUnconfiguredCoSiteInterference(&rejected_input);
  result = session.StepWithResult(rejected_input);
  ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kRejectedExecution);

  // 周期 3：作废沿在首个成功周期可见（kAcquisitionTimeout + ID 保留）。
  result = session.StepWithResult(MakeCycleInput(3U, platform, targets));
  ASSERT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
  EXPECT_TRUE(result.designation_reverted_to_tws);
  EXPECT_EQ(result.designation_revert_reason, ArDesignationRevertReason::kAcquisitionTimeout);
  EXPECT_EQ(result.designated_target_id, silent.target_id);
}

}  // namespace
}  // namespace tests
}  // namespace airborne_radar
