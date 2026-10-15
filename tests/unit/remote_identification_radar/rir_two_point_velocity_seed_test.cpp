// Copyright 2026. All Rights Reserved.
//
// @file rir_two_point_velocity_seed_test.cpp
// @brief 验证两点起批速度种子（2026-10-04 契约修订驱动）。
//
// 背景（rir-tracking-realism 契约修订 1 第 4 条）：真值速度种子被禁之后，建轨速度
// 均值恒为零，滤波速度只能靠后验收敛。两点起批给出**零时刻即正确**的初速，取代
// 「零速均值 + 速度无知先验」。
//
// 修法：速度种子取**同一目标相邻两次雷达自身量测**之差除以实际时间间隔（标准航迹
// 起始法）。不引入任何场景真值：只读雷达自己的量测序列。
//
// 覆盖：
//   1) 首个检测周期无差分基准 → 速度 ≈ 0（与既有行为逐位一致，无副作用）；
//   2) 第二个检测周期起种子生效 → 航迹速度**当拍**即达真值量级（±10%），
//      航迹保持单一关联键、不再每周期重建。
// 说明：本用例的"真值"只用于断言容差，不参与种子计算（种子只读雷达量测）。
//
// 边界（2026-10-04 实测核对，务必勿据本用例排障别的现象）：
//   在本用例几何下，把种子改回填零**同样**能关联上并在一拍内收敛到真值——因为
//   速度无知先验（3000 m/s，3σ）把关联波门撑到约 36 km 宽，零速初值的预测位置
//   虽然不动，量测仍落在门内。也就是说：本用例锚定的是"两点起批按预期生效"，
//   不是"零种子一定会失配"。若观察到航迹逐周期重建，应先排查指向/量测来源，
//   不要预设是速度种子所致。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "1q/remote_identification_radar/session/RirTrackRequest.h"
#include "RirCycleInputTestUtil.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleInput;
using session::RirCycleResult;
using session::RirCycleStatus;
using session::RirSceneTarget;
using session::RirSession;
using session::RirTrackAttributionRecord;

/** @brief 目标起始位置（雷达局部 ENU，m）：正东 50 km、高 10 km（仰角 ≈ 11.3°）。 */
constexpr float kTargetEastM = 50000.0f;
constexpr float kTargetUpM = 10000.0f;
/** @brief 目标速度（ENU 正北向，m/s）：弹道段量级。 */
constexpr float kTargetNorthSpeedMps = 3000.0f;
/** @brief 周期步长（s）；与交付场景同量级（弹道目标一跳数公里）。 */
constexpr double kStepSec = 4.0;
/** @brief 目标 RCS（m²）：本用例只验跟踪链路，取足够大的值保证稳定过检测门。 */
constexpr float kTargetRcsM2 = 100.0f;
/** @brief 执行周期数（首周期建轨 + 后续协方差收敛）。 */
constexpr std::uint32_t kCycleCount = 6U;
/** @brief 末周期航迹数上限：修复前每周期新建一条（6 周期 → 6 条）。 */
constexpr std::size_t kMaxTrackCountAfterConvergence = 2U;

config::RirSessionConfig MakeConfig() {
  config::RirSessionConfig config;
  config.mission.work_mode = config::RirWorkMode::kIdentify;
  // 6 dB 回退口径：量测位置取真值，把断言从量测噪声里解耦，只验速度种子机制
  // （detector 门控路径的量测噪声不改变两点差分的无偏性）。
  config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  config.policy.association.distance_gate_sigma = 3.0f;
  config.hardware.antenna.nominal_az_beamwidth_deg = 4.0f;
  config.hardware.antenna.nominal_el_beamwidth_deg = 4.0f;
  // 放开可扫描体积，避免体积/子窗门把目标裁掉（本用例只验跟踪链路）。
  config.orientation.az_min_deg = -180.0f;
  config.orientation.az_max_deg = 180.0f;
  config.orientation.el_min_deg = -90.0f;
  config.orientation.el_max_deg = 90.0f;
  return config;
}

/** @brief 构造第 cycle 周期输入：目标从起始位置按速度沿 +y 匀速前进。 */
RirCycleInput MakeInput(std::uint32_t cycle, float north_speed_mps) {
  RirCycleInput input;
  input.input_cycle_index = cycle;
  input.dt_sec = kStepSec;
  input.sim_time_sec = static_cast<float>(static_cast<double>(cycle - 1U) * kStepSec);
  SetDefaultTestPlatformEcef(&input);

  RirSceneTarget target;
  target.external_target_id = 1U;
  target.target_name = "two-point-seed-target";
  target.position_x = kTargetEastM;
  target.position_y = north_speed_mps * input.sim_time_sec;
  target.position_z = kTargetUpM;
  target.velocity_x = 0.0f;
  target.velocity_y = north_speed_mps;
  target.velocity_z = 0.0f;
  target.rcs = kTargetRcsM2;
  target.target_swerling_type = session::RirSwerlingType::kSwerling0;
  input.scene_targets.push_back(target);
  return input;
}

/** @brief 把目标当前 ENU 位置/速度转成外部跟踪请求（模拟外部位置录入）。 */
session::RirExternalTrackRequest MakeExternalTrackRequest(float north_position_m,
                                                         float north_speed_mps,
                                                         double source_time_s) {
  oneq::coordinate::LlaPositionDegM platform_lla{};
  platform_lla.latitude_deg = 30.0;
  platform_lla.longitude_deg = 120.0;
  platform_lla.altitude_m = 1000.0;

  // 小角度 ENU 偏移近似（测试几何，误差远小于断言容差）。
  oneq::coordinate::LlaPositionDegM target_lla = platform_lla;
  const double kPi = 3.14159265358979323846;
  const double m_per_deg_lat = 111320.0;
  const double m_per_deg_lon = 111320.0 * std::cos(platform_lla.latitude_deg * kPi / 180.0);
  target_lla.longitude_deg += static_cast<double>(kTargetEastM) / m_per_deg_lon;
  target_lla.latitude_deg += static_cast<double>(north_position_m) / m_per_deg_lat;
  target_lla.altitude_m += static_cast<double>(kTargetUpM);

  session::RirExternalTrackRequest request;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &request.position_ecef_m));
  // 速度只用于库内外推；本用例 source_time = 本周期时刻（零外推时延），故近似即可。
  request.velocity_ecef_mps.x_mps = 0.0;
  request.velocity_ecef_mps.y_mps = static_cast<double>(north_speed_mps);
  request.velocity_ecef_mps.z_mps = 0.0;
  request.source_time_s = source_time_s;
  request.source_target_id = 77U;
  request.source_entity_id = 4U;
  return request;
}

/** @brief 先按外部位置录入指向中心，再执行一个周期（每周期重发，模拟持续跟踪）。 */
RirCycleResult SubmitAndStep(RirSession* session, std::uint32_t cycle, float north_speed_mps) {
  const RirCycleInput input = MakeInput(cycle, north_speed_mps);
  const float north_position_m = north_speed_mps * input.sim_time_sec;
  EXPECT_TRUE(session->SubmitExternalTrack(
      MakeExternalTrackRequest(north_position_m, north_speed_mps,
                               static_cast<double>(input.sim_time_sec))));
  return session->StepWithResult(input);
}

/** @brief 末周期航迹归属视图（每航迹一条，含滤波速度模长）。 */
std::vector<RirTrackAttributionRecord> RunCycles(RirSession* session, float north_speed_mps,
                                                 float* last_slant_range_m,
                                                 session::RirIssueList* last_issues) {
  std::vector<RirTrackAttributionRecord> attributions;
  for (std::uint32_t cycle = 1U; cycle <= kCycleCount; ++cycle) {
    const RirCycleResult result = SubmitAndStep(session, cycle, north_speed_mps);
    EXPECT_EQ(result.status, RirCycleStatus::kCompleted);
    attributions = result.track_attributions;
    *last_slant_range_m = result.max_detected_slant_range_m;
    *last_issues = result.issues;
  }
  return attributions;
}

/** @brief 把周期问题清单渲染成一行，供断言失败时定位（诊断可读性，非产品逻辑）。 */
std::string DescribeIssues(const session::RirIssueList& issues) {
  std::string text;
  for (const session::RirIssue& issue : issues) {
    text += "[" + issue.code + "] " + issue.message + " | ";
  }
  return text;
}

/// @brief 静止目标：两点差分恒为零，速度与修复前行为逐位一致（无副作用）。
TEST(RirTwoPointVelocitySeedTest, StationaryTargetKeepsZeroSpeed) {
  RirSession session = RirSession::Create(MakeConfig());
  float last_slant_range_m = 0.0f;
  session::RirIssueList issues;
  const std::vector<RirTrackAttributionRecord> tracks =
      RunCycles(&session, 0.0f, &last_slant_range_m, &issues);

  ASSERT_FALSE(tracks.empty()) << DescribeIssues(issues);
  for (const RirTrackAttributionRecord& track : tracks) {
    EXPECT_NEAR(track.speed_m_per_s, 0.0, 0.01)
        << "静止目标的两点差分必须为零（首次检测口径不变）";
  }
}

/// @brief 匀速运动目标：两点起批给出真实量级初速，航迹不再每周期重建。
TEST(RirTwoPointVelocitySeedTest, MovingTargetSeedsSpeedFromTwoMeasurements) {
  RirSession session = RirSession::Create(MakeConfig());

  // 首周期：仅一次量测，无差分基准 → 零种子（与修复前一致）。
  const RirCycleResult first = SubmitAndStep(&session, 1U, kTargetNorthSpeedMps);
  ASSERT_EQ(first.status, RirCycleStatus::kCompleted);
  ASSERT_EQ(first.track_attributions.size(), 1U) << DescribeIssues(first.issues);
  EXPECT_NEAR(first.track_attributions[0].speed_m_per_s, 0.0, 0.01)
      << "单次检测没有速度量测通道：首个周期必须仍是零种子";

  float last_slant_range_m = 0.0f;
  session::RirIssueList issues;
  const std::vector<RirTrackAttributionRecord> tracks =
      RunCycles(&session, kTargetNorthSpeedMps, &last_slant_range_m, &issues);

  // 末周期仍是单航迹：两点起批生效后关联键不逐周期更换。
  EXPECT_LE(tracks.size(), kMaxTrackCountAfterConvergence)
      << "航迹数随周期数增长说明关联每周期失配（两点起批未生效）";

  // 速度收敛到真值量级（真值仅作容差，不参与种子计算）。
  // 实测：加种子后第 2 拍即达真值；不加种子同样能在本几何下一拍收敛（超宽波门所致，
  // 见文件头"边界"说明），故此处只锚定收敛结果，不把"必须靠种子收敛"写成断言。
  double best_speed_error_ratio = 1.0e9;
  for (const RirTrackAttributionRecord& track : tracks) {
    const double error_ratio =
        std::fabs(track.speed_m_per_s - static_cast<double>(kTargetNorthSpeedMps)) /
        static_cast<double>(kTargetNorthSpeedMps);
    best_speed_error_ratio = std::min(best_speed_error_ratio, error_ratio);
  }
  ASSERT_LT(best_speed_error_ratio, 1.0e9) << "末周期必须有航迹输出";
  EXPECT_LT(best_speed_error_ratio, 0.10)
      << "两点起批速度种子应把航迹速度带到真值 ±10% 内（实测 "
      << tracks.front().speed_m_per_s << " m/s）";
  EXPECT_GT(last_slant_range_m, 0.0f);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
