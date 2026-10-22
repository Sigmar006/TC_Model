// Copyright 2026. All Rights Reserved.
//
// @file rir_external_track_test.cpp
// @brief 验证外部目标跟踪（位置 + 速度 -> 外推中心 + 邻域窗口）接口。
//
// 机理（2026-10-04 实测驱动，见
// docs/review/ir_external_position_driven_rir_e2e_2026-10-04.md）：
// 本接口的指向语义是「以该中心为范围的跟踪」，**不是**把中心当整机扫描基准
// （`scan_center`）。实测：仅用扫描基准时零偏移、完美指向下仍 0/25 周期检出，
// 且失败静默（只报 rir.target_outside_beam_coverage）；用外部跟踪指向时
// 25/25 检出。故本测试重点锚定两者可区分、且中心每周期被照到。
//
// 覆盖：
//   1) 提交合法请求 -> 首周期即为 kFresh（不被误判为 coasting）；
//   2) 指向 = 由外部 ECEF 位置反算的雷达局部 ENU 视线角（az/el 与真值一致）；
//   3) 中心优先：本周期首个外部驻留指向中心本身（"每周期必照到"）；
//   4) 速度外推：source_time 落后一周期时，中心指向按速度前推的位置；
//   5) 无新请求 -> kCoasting 且继续外推（窗口跟随）；超有效窗口 -> kExpired 释放；
//   6) 非法请求原子拒绝且不改状态；
//   7) 指向来源与扫描基准可区分（pointing_source 非 kScan）。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/coordinate/scene_transform.h"
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
using session::RirExternalTrackPhase;
using session::RirExternalTrackRequest;
using session::RirPointingSource;
using session::RirSceneTarget;
using session::RirSession;
using session::RirTrackWindowDeg;

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

/** @brief 测试平台：30°N / 120°E / 1000 m（与 RirCycleInputTestUtil 同源）。 */
oneq::coordinate::LlaPositionDegM TestPlatformLla() {
  oneq::coordinate::LlaPositionDegM lla{};
  lla.latitude_deg = 30.0;
  lla.longitude_deg = 120.0;
  lla.altitude_m = 1000.0;
  return lla;
}

/** @brief 把「相对平台的 ENU 位置/速度」转成 ECEF 请求（模拟外部给的目标状态）。 */
RirExternalTrackRequest MakeRequestFromEnu(double east_m, double north_m, double up_m,
                                           double v_east_mps, double v_north_mps,
                                           double v_up_mps, double source_time_s) {
  const oneq::coordinate::LlaPositionDegM anchor = TestPlatformLla();
  oneq::coordinate::EcefPositionM platform_ecef;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(anchor, &platform_ecef));

  // 先由本地 ENU 目标位置反推 ECEF：用平台锚点做「ENU -> ECEF」正向换系。
  oneq::coordinate::EcefPositionM target_ecef;
  {
    oneq::coordinate::ExternalKinematics kin;
    kin.position_frame = oneq::coordinate::PositionFrame::kLla;
    oneq::coordinate::LlaPositionDegM target_lla = anchor;
    // 用 ENU 偏移的近似 LLA 增量构造（小角度、测试几何，误差远小于断言容差）。
    const double m_per_deg_lat = 111320.0;
    const double m_per_deg_lon = 111320.0 * std::cos(anchor.latitude_deg * 3.14159265358979323846 / 180.0);
    target_lla.latitude_deg += north_m / m_per_deg_lat;
    target_lla.longitude_deg += east_m / m_per_deg_lon;
    target_lla.altitude_m += up_m;
    kin.position_lla_deg_m = target_lla;
    kin.position_frame = oneq::coordinate::PositionFrame::kLla;
    EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));
  }

  RirExternalTrackRequest request;
  request.position_ecef_m = target_ecef;
  // 速度按 ECEF 给：测试只关心其被用于外推，方向由 ENU->ECEF 近似换算。
  request.velocity_ecef_mps.x_mps = v_east_mps;
  request.velocity_ecef_mps.y_mps = v_north_mps;
  request.velocity_ecef_mps.z_mps = v_up_mps;
  request.source_time_s = source_time_s;
  request.window.az_half_deg = 8.0f;
  request.window.el_half_deg = 4.0f;
  request.source_target_id = 77U;
  request.source_entity_id = 4U;
  return request;
}

config::RirSessionConfig MakeConfig() {
  config::RirSessionConfig config;
  config.mission.work_mode = config::RirWorkMode::kIdentify;
  config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  // 本测试只验指向与窗口，主瓣覆盖门另有用例；波束放宽避免把指向断言耦合到覆盖门。
  config.hardware.antenna.nominal_az_beamwidth_deg = 4.0f;
  config.hardware.antenna.nominal_el_beamwidth_deg = 4.0f;
  config.orientation.az_min_deg = -180.0f;
  config.orientation.az_max_deg = 180.0f;
  config.orientation.el_min_deg = -90.0f;
  config.orientation.el_max_deg = 90.0f;
  return config;
}

RirCycleInput MakeInput(std::uint32_t cycle, double dt_sec) {
  RirCycleInput input;
  input.input_cycle_index = cycle;
  input.dt_sec = dt_sec;
  input.sim_time_sec = static_cast<float>(static_cast<double>(cycle - 1U) * dt_sec);
  SetDefaultTestPlatformEcef(&input);
  return input;
}

/// @brief 首周期即 kFresh（回归：阶段判定必须先于外推解析，否则首周期被误判 coasting）。
TEST(RirExternalTrackTest, FirstCycleIsFreshNotCoasting) {
  RirSession session = RirSession::Create(MakeConfig());
  ASSERT_TRUE(session.SubmitExternalTrack(MakeRequestFromEnu(100000.0, 0.0, 20000.0, 0.0, 0.0, 0.0, 0.0)));

  const RirCycleResult result = session.StepWithResult(MakeInput(1U, 0.5));
  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
  EXPECT_EQ(result.external_track_phase, RirExternalTrackPhase::kFresh);

  const session::RirExternalTrackState state = session.GetExternalTrackState();
  EXPECT_EQ(state.phase, RirExternalTrackPhase::kFresh);
  EXPECT_EQ(state.coasting_cycles, 0U);
  EXPECT_EQ(state.last_request_cycle_index, 1U);
}

/// @brief 指向 = 外部位置反算的局部 ENU 视线角；且来源标记为 kExternalTrack。
TEST(RirExternalTrackTest, CenterMatchesSubmittedPositionDirection) {
  RirSession session = RirSession::Create(MakeConfig());
  const double east_m = 100000.0;
  const double up_m = 20000.0;
  ASSERT_TRUE(session.SubmitExternalTrack(MakeRequestFromEnu(east_m, 0.0, up_m, 0.0, 0.0, 0.0, 0.0)));

  const RirCycleResult result = session.StepWithResult(MakeInput(1U, 0.5));
  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);

  // 期望：az 约 0°（正东），el 约 atan(up / east)。
  const double expect_el = std::atan2(up_m, east_m) * kRadToDeg;
  EXPECT_NEAR(result.external_track_center_deg.az_deg, 0.0f, 0.5f);
  EXPECT_NEAR(result.external_track_center_deg.el_deg, expect_el, 0.5f);

  // 「以中心为范围的跟踪」而不是「扫描基准」：来源必须可区分。
  EXPECT_EQ(result.pointing_source, RirPointingSource::kExternalTrack);
  EXPECT_GT(result.external_track_sweep_index, 0U);
}

/// @brief 中心优先：本周期首个外部驻留指向中心本身（每周期必照到）。
TEST(RirExternalTrackTest, CenterIsFirstDwell) {
  RirSession session = RirSession::Create(MakeConfig());
  ASSERT_TRUE(session.SubmitExternalTrack(MakeRequestFromEnu(80000.0, 30000.0, 10000.0, 0.0, 0.0, 0.0, 0.0)));

  const RirCycleResult result = session.StepWithResult(MakeInput(1U, 0.5));
  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
  // 首个驻留即中心（dwell_center_deg 取首个非跟踪驻留）。
  EXPECT_NEAR(result.dwell_center_deg.az_deg, result.external_track_center_deg.az_deg, 0.01f);
  EXPECT_NEAR(result.dwell_center_deg.el_deg, result.external_track_center_deg.el_deg, 0.01f);
}

/// @brief 速度外推：source_time 落后一个周期时，中心按速度前推（不外推则时间偏斜变指向误差）。
TEST(RirExternalTrackTest, ExtrapolatesByVelocity) {
  RirSession session = RirSession::Create(MakeConfig());
  // 目标正东 100 km；速度沿 ENU 东向 1000 m/s；位置时刻落后本周期 1 s。
  // 期望中心 = east 100000 + 1000*1 = 101000 m 方向（az 仍约 0，el 略降）。
  const double east_m = 100000.0;
  const double up_m = 20000.0;
  const double speed_mps = 1000.0;
  ASSERT_TRUE(session.SubmitExternalTrack(
      MakeRequestFromEnu(east_m, 0.0, up_m, speed_mps, 0.0, 0.0, -1.0)));

  const RirCycleResult result = session.StepWithResult(MakeInput(1U, 0.5));
  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);

  // 本周期 sim_time_sec = 0，source_time = -1 -> dt = 1 s -> 沿东前推 1000 m。
  const double expect_el_no_extrap = std::atan2(up_m, east_m) * kRadToDeg;
  EXPECT_LT(result.external_track_center_deg.el_deg, expect_el_no_extrap)
      << "沿水平方向前推必然降低俯仰角；未外推则等于未前推值";
}

/// @brief 无新请求 -> kCoasting（继续按最后速度外推），超有效窗口 -> kExpired 释放。
TEST(RirExternalTrackTest, CoastingThenExpired) {
  RirSession session = RirSession::Create(MakeConfig());
  RirExternalTrackRequest request =
      MakeRequestFromEnu(100000.0, 0.0, 20000.0, 500.0, 0.0, 0.0, 0.0);
  request.valid_until_cycle_index = 2U;  // 第 3 周期起失效
  ASSERT_TRUE(session.SubmitExternalTrack(request));

  const RirCycleResult first = session.StepWithResult(MakeInput(1U, 0.5));
  ASSERT_EQ(first.status, RirCycleStatus::kCompleted);
  EXPECT_EQ(first.external_track_phase, RirExternalTrackPhase::kFresh);

  // 第 2 周期不重新提交：进入外推保持，仍排外部驻留。
  const RirCycleResult second = session.StepWithResult(MakeInput(2U, 0.5));
  ASSERT_EQ(second.status, RirCycleStatus::kCompleted);
  EXPECT_EQ(second.external_track_phase, RirExternalTrackPhase::kCoasting);
  EXPECT_EQ(second.pointing_source, RirPointingSource::kExternalTrack)
      << "保持期内仍应在窗口内跟踪（这是相对旧方案的关键改进）";
  EXPECT_EQ(session.GetExternalTrackState().coasting_cycles, 1U);

  // 第 3 周期：超出有效窗口 -> kExpired，释放外部槽位。
  const RirCycleResult third = session.StepWithResult(MakeInput(3U, 0.5));
  ASSERT_EQ(third.status, RirCycleStatus::kCompleted);
  EXPECT_EQ(third.external_track_phase, RirExternalTrackPhase::kExpired);
  EXPECT_EQ(third.pointing_source, RirPointingSource::kScan);
  EXPECT_EQ(session.GetExternalTrackState().phase, RirExternalTrackPhase::kExpired);
}

/// @brief 非法请求原子拒绝且不改变当前状态。
TEST(RirExternalTrackTest, InvalidRequestRejectedAtomically) {
  RirSession session = RirSession::Create(MakeConfig());
  const RirExternalTrackRequest good =
      MakeRequestFromEnu(100000.0, 0.0, 20000.0, 0.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(session.SubmitExternalTrack(good));

  // 窗口非正：拒绝。
  RirExternalTrackRequest bad_window = good;
  bad_window.window.az_half_deg = 0.0f;
  EXPECT_FALSE(session.SubmitExternalTrack(bad_window));

  // 位置非有限：拒绝。
  RirExternalTrackRequest bad_position = good;
  bad_position.position_ecef_m.x_m = std::nan("");
  EXPECT_FALSE(session.SubmitExternalTrack(bad_position));

  // 速度非有限：拒绝。
  RirExternalTrackRequest bad_velocity = good;
  bad_velocity.velocity_ecef_mps.y_mps = std::nan("");
  EXPECT_FALSE(session.SubmitExternalTrack(bad_velocity));

  // 位置退化（原点）：拒绝。
  RirExternalTrackRequest degenerate = good;
  degenerate.position_ecef_m = oneq::coordinate::EcefPositionM(0.0, 0.0, 0.0);
  EXPECT_FALSE(session.SubmitExternalTrack(degenerate));

  // 原子性：非法提交不改动状态——仍保持最后一次**合法**提交的账目
  // （若被拒绝的请求部分生效，phase 会变成别的值或请求周期号会被改写）。
  const session::RirExternalTrackState state = session.GetExternalTrackState();
  EXPECT_EQ(state.phase, RirExternalTrackPhase::kFresh)
      << "非法提交不得改变状态机（仍为上次合法提交后的 kFresh）";
  EXPECT_EQ(state.last_request_cycle_index, 0U)
      << "尚未执行任何周期，请求账目不应被非法提交推进";
}

/// @brief Clear 立即回到既定调度。
TEST(RirExternalTrackTest, ClearReturnsToScan) {
  RirSession session = RirSession::Create(MakeConfig());
  ASSERT_TRUE(session.SubmitExternalTrack(MakeRequestFromEnu(100000.0, 0.0, 20000.0, 0.0, 0.0, 0.0, 0.0)));
  const RirCycleResult with_external = session.StepWithResult(MakeInput(1U, 0.5));
  EXPECT_EQ(with_external.pointing_source, RirPointingSource::kExternalTrack);

  session.ClearExternalTrack();
  const RirCycleResult after_clear = session.StepWithResult(MakeInput(2U, 0.5));
  ASSERT_EQ(after_clear.status, RirCycleStatus::kCompleted);
  EXPECT_EQ(after_clear.pointing_source, RirPointingSource::kScan);
  EXPECT_EQ(after_clear.external_track_phase, RirExternalTrackPhase::kNone);
  EXPECT_EQ(session.GetExternalTrackState().phase, RirExternalTrackPhase::kNone);
}

/// @brief 位置帧语义：外部位置必须真正决定指向（改变位置即改变指向）。
TEST(RirExternalTrackTest, PointingFollowsSubmittedPosition) {
  RirSession session = RirSession::Create(MakeConfig());

  ASSERT_TRUE(session.SubmitExternalTrack(MakeRequestFromEnu(100000.0, 0.0, 20000.0, 0.0, 0.0, 0.0, 0.0)));
  const RirCycleResult east = session.StepWithResult(MakeInput(1U, 0.5));
  ASSERT_EQ(east.status, RirCycleStatus::kCompleted);

  ASSERT_TRUE(session.SubmitExternalTrack(MakeRequestFromEnu(0.0, 100000.0, 20000.0, 0.0, 0.0, 0.0, 0.0)));
  const RirCycleResult north = session.StepWithResult(MakeInput(2U, 0.5));
  ASSERT_EQ(north.status, RirCycleStatus::kCompleted);

  // 正东 -> az≈0；正北 -> az≈90（az 自 +x 东向 +y 北）。
  EXPECT_NEAR(east.external_track_center_deg.az_deg, 0.0f, 0.5f);
  EXPECT_NEAR(north.external_track_center_deg.az_deg, 90.0f, 0.5f);
  EXPECT_GT(std::fabs(north.external_track_center_deg.az_deg -
                      east.external_track_center_deg.az_deg),
            45.0f)
      << "位置变了指向必须跟着变（否则外部位置没被真正消费）";
}

/// @brief 外部优先调度（2026-10-22 用户裁定）：有外部引导时全部槽位给外部窗口。
///
/// 判定：外部窗口吃满预算的周期里，本周期**不排**固定扫描光栅驻留与库内跟踪驻留
/// （`search_dwell_count == 0` 且 `internal_track_dwell_count == 0`），
/// 外部驻留条数 = 周期槽位数（dt / recognition_dwell_sec，本用例 0.5/0.05 = 10）。
TEST(RirExternalTrackTest, ExternalCueTakesAllDwellSlots) {
  RirSession session = RirSession::Create(MakeConfig());
  ASSERT_TRUE(session.SubmitExternalTrack(MakeRequestFromEnu(100000.0, 0.0, 20000.0, 0.0, 0.0, 0.0, 0.0)));

  const RirCycleResult result = session.StepWithResult(MakeInput(1U, 0.5));
  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
  EXPECT_EQ(result.pointing_source, RirPointingSource::kExternalTrack);
  EXPECT_EQ(result.search_dwell_count, 0U)
      << "有外部引导时不再按预设扫描光栅补扫（来袭方向随目标变，固定光栅是浪费）";
  EXPECT_EQ(result.internal_track_dwell_count, 0U)
      << "外部窗口吃满槽位，库内跟踪驻留本周期不排";
  EXPECT_EQ(result.search_dwell_count + result.designate_dwell_count +
                result.internal_track_dwell_count + result.external_track_dwell_count,
            10U)
      << "本周期驻留总数应等于槽位数 floor(dt / recognition_dwell_sec) = 10";
  EXPECT_GE(result.external_track_dwell_count, 1U);
  // 窗口半宽 8°/4°、波束 4° ⇒ 窗口波位数（15）足以覆盖 10 个槽位。
  EXPECT_EQ(result.external_track_dwell_count, 10U)
      << "槽位数 = floor(dt / recognition_dwell_sec) = floor(0.5 / 0.05) = 10";
}

/// @brief 无外部引导时仍按既有调度：固定光栅搜索照常（回归，避免策略改动外溢）。
TEST(RirExternalTrackTest, WithoutExternalCueSearchStillRuns) {
  RirSession session = RirSession::Create(MakeConfig());
  const RirCycleResult result = session.StepWithResult(MakeInput(1U, 0.5));
  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
  EXPECT_GT(result.search_dwell_count, 0U) << "无外部引导时搜索驻留必须照排";
  EXPECT_EQ(result.external_track_dwell_count, 0U);
}

/// @brief cue 合理性门"只标注不改分配"（2026-10-22）：跳变再大也照常吃满窗口。
///
/// 现场实测（19:35 轮）：数千公里外目标的位置噪声 0.5°~1° 折合数十至上百公里，
/// 60 km/周期 的门在 893/1156 个引导周期触发；若降级就退回固定光栅，等于把 8~9 拍
/// 白扔在找不回目标的区域——故降级仅置 `external_cue_degraded`，槽位分配不变。
TEST(RirExternalTrackTest, DegradedCueStillTakesAllSlots) {
  RirSession session = RirSession::Create(MakeConfig());
  ASSERT_TRUE(session.SubmitExternalTrack(MakeRequestFromEnu(100000.0, 0.0, 20000.0, 0.0, 0.0, 0.0, 0.0)));
  const RirCycleResult first = session.StepWithResult(MakeInput(1U, 0.5));
  ASSERT_EQ(first.status, RirCycleStatus::kCompleted);
  ASSERT_FALSE(first.external_cue_degraded) << "首周期没有历史位置，不应判降级";

  // 下一周期把 cue 挪 200 km（> 10 km/s × 1 + 50 km = 60 km）→ 判降级。
  ASSERT_TRUE(session.SubmitExternalTrack(MakeRequestFromEnu(300000.0, 0.0, 20000.0, 0.0, 0.0, 0.0, 0.0)));
  const RirCycleResult second = session.StepWithResult(MakeInput(2U, 0.5));
  ASSERT_EQ(second.status, RirCycleStatus::kCompleted);
  EXPECT_TRUE(second.external_cue_degraded) << "200 km 跳变应触发合理性门";
  EXPECT_EQ(second.search_dwell_count, 0U) << "降级不得把槽位退回固定光栅";
  EXPECT_EQ(second.internal_track_dwell_count, 0U);
  EXPECT_EQ(second.external_track_dwell_count, 10U) << "降级周期仍吃满全部槽位";
  EXPECT_EQ(second.search_dwell_count + second.designate_dwell_count +
                second.internal_track_dwell_count + second.external_track_dwell_count,
            10U);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar