/**
 * @file ar_cycle_output_builder_test.cpp
 * @brief 验证 ArCycleOutputAdapter 将内部雷达局部输出转换回外部 ECEF 输出。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArCycleOutputAdapter.h"
#include "1q/airborne_radar/session/ArRadarFrameTransform.h"
#include "1q/airborne_radar/session/ArTrackLifecycleRecorder.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"
#include "1q/coordinate/velocity_transform.h"

namespace {

using airborne_radar::session::TrackStateSnapshot;
using airborne_radar::session::ArCycleInput;
using airborne_radar::session::ArCycleOutputAdapter;
using airborne_radar::session::ArCycleResult;
using airborne_radar::session::ArPlatformInput;
using airborne_radar::session::ArExternalTrackOutputFrame;
using airborne_radar::session::ArSession;
using airborne_radar::session::ArTargetInput;
using airborne_radar::session::TrackOutputFrame;
using airborne_radar::session::ArTrackLifecycleRecorder;

airborne_radar::config::ArSessionConfig MakeDetectionFocusedConfig() {
  airborne_radar::config::ArSessionConfig cfg;
  cfg.policy.detection = airborne_radar::config::profiles::kDetectionPriorityDetection;
  cfg.policy.tracking = airborne_radar::config::profiles::kFastAssociationTracking;
  cfg.policy.lifecycle = airborne_radar::config::profiles::kFastConfirmLifecycle;
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
  platform.platform_velocity_mps.x_mps = -20.0;
  platform.platform_velocity_mps.y_mps = 35.0;
  platform.platform_velocity_mps.z_mps = 2.0;
  platform.platform_attitude_deg.yaw_deg = 12.0;
  platform.platform_attitude_deg.pitch_deg = -3.0;
  platform.platform_attitude_deg.roll_deg = 1.5;
  return platform;
}

// 平台锚点 LLA（ENU 契约锚点，见 docs/common/contract.md「场景目标平台锚点 ENU 输入契约」）。
oneq::coordinate::LlaPositionDegM MakeAnchorLla() {
  oneq::coordinate::LlaPositionDegM anchor_lla;
  EXPECT_TRUE(oneq::coordinate::TryEcefToLla(MakePlatformInput().platform_position_ecef_m,
                                             &anchor_lla));
  return anchor_lla;
}

// 世界真值（LLA 位置 + ECEF 速度）→ 平台锚点 ENU 场景目标输入。
ArTargetInput MakeEnuTargetInput(std::uint64_t target_id,
                                 const oneq::coordinate::LlaPositionDegM& target_lla,
                                 const oneq::coordinate::EcefVelocityMps& velocity_ecef,
                                 float rcs, int swerling_type) {
  oneq::coordinate::EcefPositionM target_ecef;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));

  oneq::coordinate::ExternalKinematics kinematics;
  kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  kinematics.position_ecef_m = target_ecef;
  kinematics.velocity_mps = velocity_ecef;

  oneq::coordinate::EnuSceneState enu;
  EXPECT_TRUE(oneq::coordinate::TryMakeEnuSceneState(kinematics, MakeAnchorLla(), &enu));

  ArTargetInput target;
  target.target_id = target_id;
  target.position_x = static_cast<float>(enu.position_enu_m.east_m);
  target.position_y = static_cast<float>(enu.position_enu_m.north_m);
  target.position_z = static_cast<float>(enu.position_enu_m.up_m);
  target.velocity_x = static_cast<float>(enu.velocity_enu_mps.east_mps);
  target.velocity_y = static_cast<float>(enu.velocity_enu_mps.north_mps);
  target.velocity_z = static_cast<float>(enu.velocity_enu_mps.up_mps);
  target.rcs = rcs;
  target.swerling_type = swerling_type;
  return target;
}

ArTargetInput MakeTargetInput() {
  oneq::coordinate::LlaPositionDegM target_lla;
  target_lla.latitude_deg = 30.0007;
  target_lla.longitude_deg = 120.0012;
  target_lla.altitude_m = 1035.0;

  oneq::coordinate::EcefVelocityMps velocity_ecef;
  velocity_ecef.x_mps = -15.0;
  velocity_ecef.y_mps = 42.0;
  velocity_ecef.z_mps = 3.5;
  return MakeEnuTargetInput(9001U, target_lla, velocity_ecef, 1.7f, 2);
}

std::vector<ArTargetInput> MakeMovingTargetInputs(std::size_t target_count) {
  std::vector<ArTargetInput> targets;
  targets.reserve(target_count);

  for (std::size_t i = 0; i < target_count; ++i) {
    oneq::coordinate::LlaPositionDegM target_lla;
    target_lla.latitude_deg = 30.0004 + static_cast<double>(i) * 0.0003;
    target_lla.longitude_deg = 120.0006 + static_cast<double>(i % 4U) * 0.0005;
    target_lla.altitude_m = 1010.0 + static_cast<double>(i % 5U) * 8.0;

    oneq::coordinate::EcefVelocityMps velocity_ecef;
    velocity_ecef.x_mps = -18.0 + static_cast<double>(i % 5U) * 2.5;
    velocity_ecef.y_mps = 24.0 + static_cast<double>(i % 7U) * 1.7;
    velocity_ecef.z_mps = -0.4 + static_cast<double>(i % 3U) * 0.4;
    targets.push_back(MakeEnuTargetInput(static_cast<std::uint64_t>(i) + 1U, target_lla,
                                         velocity_ecef, 0.8f + static_cast<float>(i % 4U) * 0.2f,
                                         static_cast<int>(i % 3U)));
  }
  return targets;
}

void AdvanceExternalTargets(double dt_sec, std::vector<ArTargetInput>* targets) {
  ASSERT_NE(targets, nullptr);
  for (std::size_t i = 0; i < targets->size(); ++i) {
    ArTargetInput& target = (*targets)[i];
    target.position_x += target.velocity_x * static_cast<float>(dt_sec);
    target.position_y += target.velocity_y * static_cast<float>(dt_sec);
    target.position_z += target.velocity_z * static_cast<float>(dt_sec);
  }
}

const airborne_radar::session::ArExternalTrackKinematics* FindExternalTrackByTargetId(
    const ArExternalTrackOutputFrame& frame, std::uint64_t external_target_id) {
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].external_target_id == external_target_id) {
      return &frame.tracks[i];
    }
  }
  return nullptr;
}

TrackOutputFrame MakeFrameFromInternalTarget(const ArPlatformInput& platform,
                                             const ArTargetInput& target) {
  oneq::coordinate::LocalFrameReference reference;
  oneq::coordinate::Vector3d velocity;
  const oneq::coordinate::EulerAnglesDeg zero_mount{};
  EXPECT_TRUE(airborne_radar::session::TryMakeArPoseFromPlatform(
      platform, zero_mount, &reference, &velocity));
  airborne_radar::session::ArSceneTarget local_target;
  EXPECT_TRUE(airborne_radar::session::TryMakeArTargetFromEnu(target, reference, velocity,
                                                              &local_target));
  TrackOutputFrame frame;
  frame.cycle_index = 7U;
  frame.batch_id = 11U;

  TrackStateSnapshot snapshot;
  snapshot.association_key = 1001U;
  snapshot.external_target_id = 9001U;
  snapshot.status = airborne_radar::session::TrackStatus::kConfirmed;
  snapshot.position_x = local_target.position_x;
  snapshot.position_y = local_target.position_y;
  snapshot.position_z = local_target.position_z;
  snapshot.velocity_x = local_target.velocity_x;
  snapshot.velocity_y = local_target.velocity_y;
  snapshot.velocity_z = local_target.velocity_z;
  snapshot.speed = std::sqrt(snapshot.velocity_x * snapshot.velocity_x +
                             snapshot.velocity_y * snapshot.velocity_y +
                             snapshot.velocity_z * snapshot.velocity_z);
  snapshot.rcs = local_target.rcs;
  snapshot.hit_count = 3U;
  frame.tracks.push_back(snapshot);
  return frame;
}

}  // namespace

TEST(RadarCycleOutputBuilderTest, ConvertsInternalLocalFrameBackToExternalEcef) {
  const ArPlatformInput platform = MakePlatformInput();
  const ArTargetInput target = MakeTargetInput();

  const TrackOutputFrame frame = MakeFrameFromInternalTarget(platform, target);
  ArExternalTrackOutputFrame external_frame;
  ASSERT_TRUE(ArCycleOutputAdapter::Build(platform, frame, &external_frame));

  ASSERT_EQ(external_frame.cycle_index, frame.cycle_index);
  ASSERT_EQ(external_frame.batch_id, frame.batch_id);
  ASSERT_EQ(external_frame.tracks.size(), 1U);

  // 真值以 ENU 输入给出；对照输出前先换回 ECEF 世界真值。
  const oneq::coordinate::LlaPositionDegM anchor_lla = MakeAnchorLla();
  oneq::coordinate::EnuPositionM truth_enu;
  truth_enu.east_m = target.position_x;
  truth_enu.north_m = target.position_y;
  truth_enu.up_m = target.position_z;
  oneq::coordinate::EnuVelocityMps truth_vel_enu;
  truth_vel_enu.east_mps = target.velocity_x;
  truth_vel_enu.north_mps = target.velocity_y;
  truth_vel_enu.up_mps = target.velocity_z;
  oneq::coordinate::EcefPositionM truth_ecef;
  oneq::coordinate::EcefVelocityMps truth_vel_ecef;
  ASSERT_TRUE(oneq::coordinate::TryEnuToEcef(truth_enu, anchor_lla, &truth_ecef));
  ASSERT_TRUE(oneq::coordinate::TryEnuToEcefVelocity(truth_vel_enu, anchor_lla, &truth_vel_ecef));

  const airborne_radar::session::ArExternalTrackKinematics& output = external_frame.tracks[0];
  EXPECT_EQ(output.association_key, 1001U);
  EXPECT_EQ(output.external_target_id, 9001U);
  EXPECT_EQ(output.status, airborne_radar::session::TrackStatus::kConfirmed);
  EXPECT_NEAR(output.target_position_ecef_m.x_m, truth_ecef.x_m, 0.1);
  EXPECT_NEAR(output.target_position_ecef_m.y_m, truth_ecef.y_m, 0.1);
  EXPECT_NEAR(output.target_position_ecef_m.z_m, truth_ecef.z_m, 0.1);
  EXPECT_NEAR(output.target_velocity_mps.x_mps, truth_vel_ecef.x_mps, 1.0e-4);
  EXPECT_NEAR(output.target_velocity_mps.y_mps, truth_vel_ecef.y_mps, 1.0e-4);
  EXPECT_NEAR(output.target_velocity_mps.z_mps, truth_vel_ecef.z_mps, 1.0e-4);
  EXPECT_FLOAT_EQ(output.rcs, target.rcs);
  EXPECT_EQ(output.hit_count, 3U);
}

TEST(RadarCycleOutputBuilderTest, NullOutputReturnsFalse) {
  const ArPlatformInput platform = MakePlatformInput();
  const TrackOutputFrame frame;
  EXPECT_FALSE(ArCycleOutputAdapter::Build(platform, frame, nullptr));
}

TEST(RadarCycleOutputBuilderTest, FullSessionEstimateConvertsNearExternalTruth) {
  const ArPlatformInput platform = MakePlatformInput();
  const ArTargetInput target = MakeTargetInput();

  ArCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 1.0;
  input.platform = platform;
  input.targets.push_back(target);

  ArSession session =
      airborne_radar::session::ArSession::Create(MakeDetectionFocusedConfig());
  const ArCycleResult result = session.StepWithResult(input);
  ASSERT_FALSE(airborne_radar::session::HasValidationError(result.issues));
  ASSERT_FALSE(result.output_frame.tracks.empty());

  ArExternalTrackOutputFrame external_frame;
  ASSERT_TRUE(ArCycleOutputAdapter::Build(platform, result.output_frame, &external_frame));
  ASSERT_FALSE(external_frame.tracks.empty());

  // 真值以 ENU 输入给出；对照输出前先换回 ECEF 世界真值。
  const oneq::coordinate::LlaPositionDegM anchor_lla = MakeAnchorLla();
  oneq::coordinate::EnuPositionM truth_enu;
  truth_enu.east_m = target.position_x;
  truth_enu.north_m = target.position_y;
  truth_enu.up_m = target.position_z;
  oneq::coordinate::EnuVelocityMps truth_vel_enu;
  truth_vel_enu.east_mps = target.velocity_x;
  truth_vel_enu.north_mps = target.velocity_y;
  truth_vel_enu.up_mps = target.velocity_z;
  oneq::coordinate::EcefPositionM truth_ecef;
  oneq::coordinate::EcefVelocityMps truth_vel_ecef;
  ASSERT_TRUE(oneq::coordinate::TryEnuToEcef(truth_enu, anchor_lla, &truth_ecef));
  ASSERT_TRUE(oneq::coordinate::TryEnuToEcefVelocity(truth_vel_enu, anchor_lla, &truth_vel_ecef));

  const airborne_radar::session::ArExternalTrackKinematics& estimate = external_frame.tracks[0];
  EXPECT_NEAR(estimate.target_position_ecef_m.x_m, truth_ecef.x_m, 5.0);
  EXPECT_NEAR(estimate.target_position_ecef_m.y_m, truth_ecef.y_m, 5.0);
  EXPECT_NEAR(estimate.target_position_ecef_m.z_m, truth_ecef.z_m, 5.0);
  EXPECT_NEAR(estimate.target_velocity_mps.x_mps, truth_vel_ecef.x_mps, 0.5);
  EXPECT_NEAR(estimate.target_velocity_mps.y_mps, truth_vel_ecef.y_mps, 0.5);
  EXPECT_NEAR(estimate.target_velocity_mps.z_mps, truth_vel_ecef.z_mps, 0.5);
}

TEST(RadarCycleOutputBuilderTest, NaturalEnvironmentIsIndependentFromInterference) {
  const ArPlatformInput platform = MakePlatformInput();
  const ArTargetInput target = MakeTargetInput();

  ArCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 1.0;
  input.platform = platform;
  input.targets.push_back(target);
  EXPECT_TRUE(input.interference.emissions.empty());

  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());
  const ArCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
}

TEST(RadarCycleOutputBuilderTest, MultiCycleMovingTargetsStayNearExternalTruth) {
  ArPlatformInput platform = MakePlatformInput();
  platform.platform_velocity_mps.x_mps = 0.0;
  platform.platform_velocity_mps.y_mps = 0.0;
  platform.platform_velocity_mps.z_mps = 0.0;

  std::vector<ArTargetInput> targets = MakeMovingTargetInputs(12U);
  ArSession session =
      airborne_radar::session::ArSession::Create(MakeDetectionFocusedConfig());

  const std::size_t cycle_count = 40U;
  const float dt_sec = 0.5f;
  for (std::size_t cycle = 0; cycle < cycle_count; ++cycle) {
    ArCycleInput input;
    input.cycle_index = static_cast<std::uint32_t>(cycle + 1U);
    input.cycle_start_time_s = static_cast<double>(cycle) * dt_sec;
    input.dt_sec = dt_sec;
    input.platform = platform;
    input.targets = targets;

    const ArCycleResult result = session.StepWithResult(input);
    ASSERT_FALSE(airborne_radar::session::HasValidationError(result.issues)) << "cycle=" << cycle;
    ASSERT_FALSE(result.output_frame.tracks.empty()) << "cycle=" << cycle;

    ArExternalTrackOutputFrame external_frame;
    ASSERT_TRUE(
        ArCycleOutputAdapter::Build(platform, result.output_frame, &external_frame))
        << "cycle=" << cycle;

    for (std::size_t target_index = 1U; target_index < targets.size(); ++target_index) {
      const airborne_radar::session::ArExternalTrackKinematics* estimate =
          FindExternalTrackByTargetId(external_frame, static_cast<std::uint64_t>(target_index) + 1U);
      ASSERT_NE(estimate, nullptr) << "cycle=" << cycle << " target_index=" << target_index;

      // 真值以 ENU 输入给出；对照输出前先换回 ECEF 世界真值。
      const ArTargetInput& truth = targets[target_index];
      const oneq::coordinate::LlaPositionDegM anchor_lla = MakeAnchorLla();
      oneq::coordinate::EnuPositionM truth_enu;
      truth_enu.east_m = truth.position_x;
      truth_enu.north_m = truth.position_y;
      truth_enu.up_m = truth.position_z;
      oneq::coordinate::EnuVelocityMps truth_vel_enu;
      truth_vel_enu.east_mps = truth.velocity_x;
      truth_vel_enu.north_mps = truth.velocity_y;
      truth_vel_enu.up_mps = truth.velocity_z;
      oneq::coordinate::EcefPositionM truth_ecef;
      oneq::coordinate::EcefVelocityMps truth_vel_ecef;
      ASSERT_TRUE(oneq::coordinate::TryEnuToEcef(truth_enu, anchor_lla, &truth_ecef));
      ASSERT_TRUE(oneq::coordinate::TryEnuToEcefVelocity(truth_vel_enu, anchor_lla,
                                                         &truth_vel_ecef));
      EXPECT_NEAR(estimate->target_position_ecef_m.x_m, truth_ecef.x_m, 10.0)
          << "cycle=" << cycle << " target_index=" << target_index;
      EXPECT_NEAR(estimate->target_position_ecef_m.y_m, truth_ecef.y_m, 10.0)
          << "cycle=" << cycle << " target_index=" << target_index;
      EXPECT_NEAR(estimate->target_position_ecef_m.z_m, truth_ecef.z_m, 10.0)
          << "cycle=" << cycle << " target_index=" << target_index;
      EXPECT_NEAR(estimate->target_velocity_mps.x_mps, truth_vel_ecef.x_mps, 1.0)
          << "cycle=" << cycle << " target_index=" << target_index;
      EXPECT_NEAR(estimate->target_velocity_mps.y_mps, truth_vel_ecef.y_mps, 1.0)
          << "cycle=" << cycle << " target_index=" << target_index;
      EXPECT_NEAR(estimate->target_velocity_mps.z_mps, truth_vel_ecef.z_mps, 1.0)
          << "cycle=" << cycle << " target_index=" << target_index;
    }

    AdvanceExternalTargets(dt_sec, &targets);
  }
}

TEST(RadarCycleOutputBuilderTest, AttachRecorderDrivesUpdateAutomatically) {
  const ArPlatformInput platform = MakePlatformInput();
  const ArTargetInput target = MakeTargetInput();
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());
  ArTrackLifecycleRecorder recorder;
  session.AttachTrackLifecycleRecorder(&recorder);

  // 运行足够多周期让轨迹进入 confirmed 状态（kFastConfirmLifecycle）。
  ArTargetInput moving_target = target;
  const float dt_sec = 0.5f;
  bool saw_event = false;
  for (std::uint32_t cycle = 1U; cycle <= 10U; ++cycle) {
    ArCycleInput input;
    input.cycle_index = cycle;
    input.cycle_start_time_s = static_cast<double>(cycle - 1U) * dt_sec;
    input.dt_sec = dt_sec;
    input.platform = platform;
    input.targets.push_back(moving_target);
    session.StepWithResult(input);
    if (!recorder.GetLastEvents().empty()) {
      saw_event = true;
    }
    moving_target.position_x += moving_target.velocity_x * dt_sec;
    moving_target.position_y += moving_target.velocity_y * dt_sec;
    moving_target.position_z += moving_target.velocity_z * dt_sec;
  }
  EXPECT_TRUE(saw_event);
}

TEST(RadarCycleOutputBuilderTest, DetachRecorderStopsAutomaticDriving) {
  const ArPlatformInput platform = MakePlatformInput();
  ArTargetInput target = MakeTargetInput();
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());
  ArTrackLifecycleRecorder recorder;
  session.AttachTrackLifecycleRecorder(&recorder);

  // 第一个周期驱动 recorder。
  ArCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0;
  input.platform = platform;
  input.targets.push_back(target);
  session.StepWithResult(input);
  const std::size_t first_size = recorder.GetLastEvents().size();

  // 解除注册后再步进——recorder 不应被驱动，GetLastEvents 不变。
  session.AttachTrackLifecycleRecorder(nullptr);
  target.position_x += target.velocity_x * 1.0f;
  target.position_y += target.velocity_y * 1.0f;
  input.cycle_index = 2U;
  input.targets.clear();
  input.targets.push_back(target);
  session.StepWithResult(input);
  EXPECT_EQ(recorder.GetLastEvents().size(), first_size);
}

TEST(RadarCycleOutputBuilderTest, SessionWithoutRecorderIsBackwardCompatible) {
  const ArPlatformInput platform = MakePlatformInput();
  const ArTargetInput target = MakeTargetInput();
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());

  ArCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0;
  input.platform = platform;
  input.targets.push_back(target);
  const ArCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
}

TEST(RadarCycleOutputBuilderTest, GetLastEventsEmptyAfterConstruction) {
  ArTrackLifecycleRecorder recorder;
  EXPECT_TRUE(recorder.GetLastEvents().empty());
}

TEST(RadarCycleOutputBuilderTest, NonExecutedCycleDoesNotUpdateLastEvents) {
  const ArPlatformInput platform = MakePlatformInput();
  ArTargetInput target = MakeTargetInput();
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());
  ArTrackLifecycleRecorder recorder;
  session.AttachTrackLifecycleRecorder(&recorder);

  // 第一个周期执行并驱动 recorder。
  ArCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0;
  input.platform = platform;
  input.targets.push_back(target);
  session.StepWithResult(input);
  const std::size_t first_size = recorder.GetLastEvents().size();

  // 非法输入（dt_sec=0）→ validation rejection → 非执行周期，缓存保持不变。
  ArCycleInput invalid = input;
  invalid.dt_sec = 0.0;
  const ArCycleResult rejected = session.StepWithResult(invalid);
  EXPECT_EQ(rejected.status, airborne_radar::session::ArCycleStatus::kRejectedInvalidInput);
  EXPECT_EQ(recorder.GetLastEvents().size(), first_size);
}
