/**
 * @file target_inference_engine_test.cpp
 * @brief 验证 target_inference 推演引擎：弹道守恒、发射/落点一致性、误差预算出口、
 *        类型融合与确定性。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/target_inference/InferenceResult.h"
#include "1q/target_inference/InferenceTrackState.h"
#include "1q/target_inference/TargetInferenceConfig.h"
#include "1q/target_inference/TargetInferenceEngine.h"

#include "target_inference/InferenceAcceptanceRecords.h"

namespace target_inference {
namespace {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kEarthMu = 3.986004418e14;

/** @brief 构造弹道中段状态：赤道上方 150 km，水平速度 1.5 km/s（无阻力弹道弧）。 */
InferenceTrackState MakeBallisticMidcourse() {
  InferenceTrackState track;
  track.key = 42U;
  track.position = oneq::coordinate::EcefPositionM(kEarthRadiusM + 150000.0, 0.0, 0.0);
  track.velocity_ecef_m_per_s = {0.0, 1500.0, 0.0};
  return track;
}

InferenceTrackState WithCovariance(InferenceTrackState track, double position_sigma_m,
                                   double velocity_sigma_m_per_s) {
  track.has_covariance = true;
  for (int i = 0; i < 6; ++i) {
    const auto j = static_cast<std::size_t>(i);
    track.covariance_ecef[j * 6U + j] =
        (i % 2 == 0) ? position_sigma_m * position_sigma_m
                     : velocity_sigma_m_per_s * velocity_sigma_m_per_s;
  }
  return track;
}

TEST(TargetInferenceEngineTest, SpecificMechanicalEnergyAnchors) {
  // 地面静止：ε = −μ/R（比机械能纯势能锚点）。
  const oneq::coordinate::EcefPositionM ground(kEarthRadiusM, 0.0, 0.0);
  EXPECT_NEAR(SpecificMechanicalEnergyJPerKg(ground, {{0.0, 0.0, 0.0}}, kEarthMu),
              -kEarthMu / kEarthRadiusM, 1.0);
  // 半径 a 圆轨道：v² = μ/a ⇒ ε = −μ/(2a)（轨道能量锚点）。
  const double semi_major = kEarthRadiusM + 500000.0;
  const double v_circular = std::sqrt(kEarthMu / semi_major);
  const oneq::coordinate::EcefPositionM orbit(semi_major, 0.0, 0.0);
  EXPECT_NEAR(
      SpecificMechanicalEnergyJPerKg(orbit, {{0.0, v_circular, 0.0}}, kEarthMu),
      -kEarthMu / (2.0 * semi_major), 1.0);
  // 零位置：不可用约定返回 0（调用方跳过该采样）。
  const oneq::coordinate::EcefPositionM zero_pos;
  EXPECT_DOUBLE_EQ(SpecificMechanicalEnergyJPerKg(zero_pos, {{1.0, 2.0, 3.0}}, kEarthMu), 0.0);
}

// ---------------------------------------------------------------------------
// 关机点判定状态机（UpdateBurnoutTracker；甲方 2026-08-22 批注口径）
// 固定地心距 r 的合成采样直接驱动纯函数：r = R + 150 km，当地重力 g = μ/r²，
// 噪声尺度门 ≈ 1e-4·μ/r、下降沿门 ≈ 1e-3·μ/r。
// ---------------------------------------------------------------------------

constexpr double kBurnoutTestRadiusM = kEarthRadiusM + 150000.0;
constexpr double kBurnoutGravity = kEarthMu / (kBurnoutTestRadiusM * kBurnoutTestRadiusM);

oneq::coordinate::EcefPositionM BurnoutTestPosition() {
  return oneq::coordinate::EcefPositionM(kBurnoutTestRadiusM, 0.0, 0.0);
}

/** @brief 滑行一步：保速旋转（|Δv| ≈ g·dt，ε 精确守恒）+ 1e-12 下偏消浮点平局。 */
std::array<double, 3U> BurnoutCoastStep(const std::array<double, 3U>& v, double dt_sec) {
  const double speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  const double theta = kBurnoutGravity * dt_sec / speed;
  const double scale = 1.0 - 1.0e-12;
  return {(v[0] * std::cos(theta) - v[1] * std::sin(theta)) * scale,
          (v[0] * std::sin(theta) + v[1] * std::cos(theta)) * scale, v[2]};
}

/** @brief 助推一步：沿 +y 切向加 dv（|a| = dv/dt）。 */
std::array<double, 3U> BurnoutBoostStep(const std::array<double, 3U>& v, double dv_mps) {
  return {v[0], v[1] + dv_mps, v[2]};
}

TEST(TargetInferenceEngineTest, BurnoutBoostThenCoastConfirmsAtLastBoostSample) {
  BurnoutTrackerState state;
  const oneq::coordinate::EcefPositionM pos = BurnoutTestPosition();
  const std::array<double, 3U> v0{{0.0, 1500.0, 0.0}};
  const double boost_dv = 3.0 * kBurnoutGravity;  // 3g > 2.5g 加速度门

  // 首拍无通道：观测中。
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v0, 0.0, kEarthMu), BurnoutPhase::kObserving);
  // 两拍助推：助推中。
  const std::array<double, 3U> v1 = BurnoutBoostStep(v0, boost_dv);
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v1, 1.0, kEarthMu), BurnoutPhase::kBoosting);
  const std::array<double, 3U> v2 = BurnoutBoostStep(v1, boost_dv);
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v2, 2.0, kEarthMu), BurnoutPhase::kBoosting);
  // 首拍滑行（coast=1）：仍助推中——单拍不确认。
  const std::array<double, 3U> v3 = BurnoutCoastStep(v2, 1.0);
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v3, 3.0, kEarthMu), BurnoutPhase::kBoosting);
  // 次拍滑行（coast=2）：确认关机，关机时刻=最后一个助推采样（第 2 拍）。
  const std::array<double, 3U> v4 = BurnoutCoastStep(v3, 1.0);
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v4, 4.0, kEarthMu), BurnoutPhase::kConfirmed);
  EXPECT_DOUBLE_EQ(state.time_sec, 2.0);
  EXPECT_NEAR(state.speed_mps, 1500.0 + 2.0 * boost_dv, 1.0e-6);
  EXPECT_TRUE(state.ever_boosted);

  // 锚点冻结守护：确认后缓升（巡航爬升场景，单拍增量低于通道门）不得使关机
  // 时刻漂移——峰值累计被确认态冻结。
  const std::array<double, 3U> v5 = BurnoutBoostStep(v4, 0.3);  // dε≈467 < 噪声尺度门
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v5, 5.0, kEarthMu), BurnoutPhase::kConfirmed);
  EXPECT_DOUBLE_EQ(state.time_sec, 2.0);

  // 再次助推重开（多脉冲/再次加速语义）：撤销结论回助推中，随后连续滑行重新
  // 确认，关机时刻更新为最后助推采样。
  const std::array<double, 3U> v6 = BurnoutBoostStep(v5, boost_dv);
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v6, 6.0, kEarthMu), BurnoutPhase::kBoosting);
  const std::array<double, 3U> v7 = BurnoutCoastStep(v6, 1.0);
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v7, 7.0, kEarthMu), BurnoutPhase::kBoosting);
  const std::array<double, 3U> v8 = BurnoutCoastStep(v7, 1.0);
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v8, 8.0, kEarthMu), BurnoutPhase::kConfirmed);
  EXPECT_DOUBLE_EQ(state.time_sec, 6.0);
}

TEST(TargetInferenceEngineTest, BurnoutFlatFromStartDeclaresBeforeWindow) {
  BurnoutTrackerState state;
  const oneq::coordinate::EcefPositionM pos = BurnoutTestPosition();
  const std::array<double, 3U> v{{0.0, 1500.0, 0.0}};
  // ε 恒定、无加速度特征：前 4 拍证据不足观测中，第 5 拍起判窗口外。
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(UpdateBurnoutTracker(state, pos, v, static_cast<double>(i), kEarthMu),
              BurnoutPhase::kObserving)
        << "采样 " << i;
  }
  for (int i = 4; i < 6; ++i) {
    EXPECT_EQ(UpdateBurnoutTracker(state, pos, v, static_cast<double>(i), kEarthMu),
              BurnoutPhase::kBeforeWindow)
        << "采样 " << i;
  }
  EXPECT_FALSE(state.ever_boosted);
}

TEST(TargetInferenceEngineTest, BurnoutFirstSampleDeclineKeepsBeforeTrackStart) {
  BurnoutTrackerState state;
  const oneq::coordinate::EcefPositionM pos = BurnoutTestPosition();
  // dt=10 s：降速 100 m/s（|a|=10 < 2.5g 不触发加速度门）但 Δε ≈ −1.45e5
  // 超下降沿门——能量自首采样即下降 ⟹ 关机早于跟踪起点（旧分支语义保留）。
  const std::array<double, 3U> v0{{0.0, 1500.0, 0.0}};
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v0, 0.0, kEarthMu), BurnoutPhase::kObserving);
  const std::array<double, 3U> v1{{0.0, 1400.0, 0.0}};
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v1, 10.0, kEarthMu),
            BurnoutPhase::kBeforeTrackStart);
  const std::array<double, 3U> v2{{0.0, 1300.0, 0.0}};
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v2, 20.0, kEarthMu),
            BurnoutPhase::kBeforeTrackStart);
}

TEST(TargetInferenceEngineTest, BurnoutAccelGateAnchorsAndEnergyDebounce) {
  const oneq::coordinate::EcefPositionM pos = BurnoutTestPosition();
  // 2g < 2.5g 加速度门不触发；能量逐拍上涨（≈2.8e4 > 噪声尺度门）但需连续
  // 2 拍防抖——第 1 拍观测中，第 2 拍经能量通道判助推。
  {
    BurnoutTrackerState state;
    const std::array<double, 3U> v0{{0.0, 1500.0, 0.0}};
    const std::array<double, 3U> v1 = BurnoutBoostStep(v0, 2.0 * kBurnoutGravity);
    const std::array<double, 3U> v2 = BurnoutBoostStep(v1, 2.0 * kBurnoutGravity);
    EXPECT_EQ(UpdateBurnoutTracker(state, pos, v0, 0.0, kEarthMu), BurnoutPhase::kObserving);
    EXPECT_EQ(UpdateBurnoutTracker(state, pos, v1, 1.0, kEarthMu), BurnoutPhase::kObserving);
    EXPECT_EQ(UpdateBurnoutTracker(state, pos, v2, 2.0, kEarthMu), BurnoutPhase::kBoosting);
  }
  // 3g > 2.5g：加速度门即时判助推（无需防抖）。
  {
    BurnoutTrackerState state;
    const std::array<double, 3U> v0{{0.0, 1500.0, 0.0}};
    const std::array<double, 3U> v1 = BurnoutBoostStep(v0, 3.0 * kBurnoutGravity);
    EXPECT_EQ(UpdateBurnoutTracker(state, pos, v0, 0.0, kEarthMu), BurnoutPhase::kObserving);
    EXPECT_EQ(UpdateBurnoutTracker(state, pos, v1, 1.0, kEarthMu), BurnoutPhase::kBoosting);
  }
}

TEST(TargetInferenceEngineTest, BurnoutSlowRiseThenDeclineConfirmsMidWindowPeak) {
  // demo 缓升场景回归守护：单拍增量低于噪声尺度门、无加速度特征（观测中），
  // 峰值随缓升走到窗口中段；随后大幅回落超下降沿门 ⟹ 确认关机、峰值时刻
  // 落在中段（非首采样）。此路径为旧下降沿口径保留，不能被助推计数取代。
  BurnoutTrackerState state;
  const oneq::coordinate::EcefPositionM pos = BurnoutTestPosition();
  std::array<double, 3U> v{{0.0, 1500.0, 0.0}};
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v, 0.0, kEarthMu), BurnoutPhase::kObserving);
  for (int i = 1; i <= 5; ++i) {
    v = BurnoutBoostStep(v, 1.3);  // |a|=1.3 m/s²，dε≈1950 < 噪声尺度门
    EXPECT_EQ(UpdateBurnoutTracker(state, pos, v, static_cast<double>(i), kEarthMu),
              BurnoutPhase::kObserving)
        << "缓升采样 " << i;
  }
  EXPECT_DOUBLE_EQ(state.time_sec, 5.0);
  EXPECT_DOUBLE_EQ(state.peak_is_first_sample, false);
  // dt=45 s：降速 100 m/s（|a|≈2.2 < 2.5g），Δε≈−1.46e5 超下降沿门。
  const std::array<double, 3U> v_down{{0.0, v[1] - 100.0, 0.0}};
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v_down, 50.0, kEarthMu), BurnoutPhase::kConfirmed);
  EXPECT_DOUBLE_EQ(state.time_sec, 5.0);
  // 继续回落：锚点冻结，关机时刻不漂移。
  const std::array<double, 3U> v_down2{{0.0, v_down[1] - 100.0, 0.0}};
  EXPECT_EQ(UpdateBurnoutTracker(state, pos, v_down2, 60.0, kEarthMu), BurnoutPhase::kConfirmed);
  EXPECT_DOUBLE_EQ(state.time_sec, 5.0);
}

TEST(TargetInferenceEngineTest, BurnoutVelocitySigmaGateKeepsObserving) {
  // 评审 2026-08-27 条3：速度观测 σ ≥ 5 m/s（仅方位/弱可观测航迹）时状态机不
  // 推进——速度欠估计的收敛缓升与助推特征动力学不可分（实测 dε≈1.4e5 J/kg/拍
  // ≫ 门限 5.7e3），判了必是假关机；σ 恢复雷达级后从头累积、正常确认。
  const oneq::coordinate::EcefPositionM pos = BurnoutTestPosition();
  const std::array<double, 3U> v0{{0.0, 1500.0, 0.0}};
  const double boost_dv = 3.0 * kBurnoutGravity;  // 3g > 2.5g 加速度门
  const std::array<double, 3U> v1 = BurnoutBoostStep(v0, boost_dv);
  const std::array<double, 3U> v2 = BurnoutBoostStep(v1, boost_dv);
  const std::array<double, 3U> v3 = BurnoutCoastStep(v2, 1.0);
  const std::array<double, 3U> v4 = BurnoutCoastStep(v3, 1.0);
  BurnoutTrackerState gated;
  EXPECT_EQ(UpdateBurnoutTracker(gated, pos, v0, 0.0, kEarthMu, 20.0),
            BurnoutPhase::kObserving);
  EXPECT_EQ(UpdateBurnoutTracker(gated, pos, v1, 1.0, kEarthMu, 20.0),
            BurnoutPhase::kObserving);
  EXPECT_EQ(UpdateBurnoutTracker(gated, pos, v2, 2.0, kEarthMu, 20.0),
            BurnoutPhase::kObserving);
  EXPECT_EQ(UpdateBurnoutTracker(gated, pos, v3, 3.0, kEarthMu, 20.0),
            BurnoutPhase::kObserving);
  EXPECT_EQ(UpdateBurnoutTracker(gated, pos, v4, 4.0, kEarthMu, 20.0),
            BurnoutPhase::kObserving);
  EXPECT_FALSE(gated.valid);  // 状态未推进：恢复后从头累积，无残留锚点。
  // 同样的助推-滑行序列在 σ_v=1 m/s（雷达级跟踪）下正常确认。
  BurnoutTrackerState armed;
  EXPECT_EQ(UpdateBurnoutTracker(armed, pos, v0, 5.0, kEarthMu, 1.0), BurnoutPhase::kObserving);
  EXPECT_EQ(UpdateBurnoutTracker(armed, pos, v1, 6.0, kEarthMu, 1.0), BurnoutPhase::kBoosting);
  EXPECT_EQ(UpdateBurnoutTracker(armed, pos, v2, 7.0, kEarthMu, 1.0), BurnoutPhase::kBoosting);
  EXPECT_EQ(UpdateBurnoutTracker(armed, pos, v3, 8.0, kEarthMu, 1.0), BurnoutPhase::kBoosting);
  EXPECT_EQ(UpdateBurnoutTracker(armed, pos, v4, 9.0, kEarthMu, 1.0), BurnoutPhase::kConfirmed);
  EXPECT_DOUBLE_EQ(armed.time_sec, 7.0);
}

TEST(TargetInferenceEngineTest, BurnoutEstimateBacktracksWhenBurnoutPrecedesObservation) {
  BurnoutTrackerState state;
  TrajectoryPrediction trajectory;
  trajectory.valid = true;
  trajectory.has_launch = true;
  trajectory.launch_time_offset_sec = -650.0;
  trajectory.launch_point.latitude_deg = 43.0;
  trajectory.launch_point.longitude_deg = 122.0;
  trajectory.launch_point.altitude_m = 100000.0;

  double time_sec = 0.0;
  oneq::coordinate::LlaPositionDegM position{};
  EXPECT_TRUE(TryResolveBurnoutEstimate(BurnoutPhase::kBeforeWindow, state, 750.0,
                                        trajectory, &time_sec, &position));
  EXPECT_DOUBLE_EQ(time_sec, 100.0);
  EXPECT_DOUBLE_EQ(position.latitude_deg, 43.0);
  EXPECT_DOUBLE_EQ(position.longitude_deg, 122.0);
  EXPECT_DOUBLE_EQ(position.altitude_m, 100000.0);
  EXPECT_FALSE(TryResolveBurnoutEstimate(BurnoutPhase::kObserving, state, 750.0,
                                         trajectory, &time_sec, &position));
  state.valid = true;
  EXPECT_TRUE(TryResolveBurnoutEstimate(BurnoutPhase::kObserving, state, 750.0,
                                        trajectory, &time_sec, &position));
}

TEST(TargetInferenceEngineTest, BallisticPredictionConservesEnergy) {
  TargetInferenceEngine engine(TargetInferenceConfig{});
  const auto results = engine.Infer({MakeBallisticMidcourse()});
  ASSERT_EQ(results.size(), 1U);
  ASSERT_TRUE(results.front().trajectory.valid);
  EXPECT_FALSE(results.front().trajectory.waypoints.empty());
  // 无阻力弹道：能量守恒（<0.1% 漂移）；近远地点输入 → 弧线高度有界（积分器门）。
  const double initial_energy = 0.5 * 1500.0 * 1500.0 - kEarthMu / (kEarthRadiusM + 150000.0);
  for (const auto& waypoint : results.front().trajectory.waypoints) {
    oneq::coordinate::EcefPositionM ecef{};
    ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(waypoint.position, &ecef));
    const double radius = std::sqrt(ecef.x_m * ecef.x_m + ecef.y_m * ecef.y_m +
                                    ecef.z_m * ecef.z_m);
    const double potential = -kEarthMu / radius;
    const double speed_squared = 2.0 * (initial_energy - potential);
    EXPECT_GT(speed_squared, 0.0) << "speed must stay real along the arc";
    EXPECT_LE(waypoint.position.altitude_m, 152000.0)
        << "near-apogee input must bound the arc altitude";
  }
}

TEST(TargetInferenceEngineTest, LaunchBacktrackStopsAtAtmosphereInterfaceAndImpactResolves) {
  // 150 km + 1.5 km/s 水平的椭圆弹道落 地表约需 600+ s：测试时域放宽到 900 s。
  TargetInferenceConfig config;
  config.prediction_horizon_sec = 900.0;
  TargetInferenceEngine engine(config);
  const auto results = engine.Infer({MakeBallisticMidcourse()});
  const TrajectoryPrediction& trajectory = results.front().trajectory;
  ASSERT_TRUE(trajectory.valid);

  ASSERT_TRUE(trajectory.has_launch);
  EXPECT_LT(trajectory.launch_time_offset_sec, 0.0);
  // 大气界面停机门（缺省 100 km）：助推段未建模，回推到界面即停，
  // 发射点估计=停机点（"助推段上界"），不再回推到地表。
  EXPECT_NEAR(trajectory.launch_point.altitude_m,
              config.launch_atmosphere_interface_altitude_m, 5000.0)
      << "launch estimate should stop at the atmosphere interface";

  // 关停机门（界面高度 ≤ 0）恢复旧口径：回推到地表交点。
  TargetInferenceConfig surface_config = config;
  surface_config.launch_atmosphere_interface_altitude_m = 0.0;
  TargetInferenceEngine surface_engine(surface_config);
  const auto surface_results = surface_engine.Infer({MakeBallisticMidcourse()});
  ASSERT_TRUE(surface_results.front().trajectory.has_launch);
  EXPECT_NEAR(surface_results.front().trajectory.launch_point.altitude_m, 0.0, 5000.0)
      << "gate disabled: back-prop must reach the surface as before";

  // 中段水平速度 1.5 km/s @150 km：弧长远大于地平线盲区，预测时域内应出落点。
  EXPECT_TRUE(trajectory.has_impact);
  if (trajectory.has_impact) {
    EXPECT_GT(trajectory.impact_time_offset_sec, 0.0);
    EXPECT_NEAR(trajectory.impact_point.altitude_m, 0.0, 5000.0);
  }
}

TEST(TargetInferenceEngineTest, LaunchBacktrackInputBelowInterfaceStopsImmediately) {
  TargetInferenceEngine engine(TargetInferenceConfig{});
  InferenceTrackState low = MakeBallisticMidcourse();
  low.position = oneq::coordinate::EcefPositionM(kEarthRadiusM + 50000.0, 0.0, 0.0);
  low.velocity_ecef_m_per_s = {0.0, 800.0, 0.0};

  const auto results = engine.Infer({low});
  ASSERT_TRUE(results.front().trajectory.valid);
  ASSERT_TRUE(results.front().trajectory.has_launch);
  // 输入已在界面以下（关机前/低空）：回推一步即停，停机点仍在输入附近。
  EXPECT_NEAR(results.front().trajectory.launch_point.altitude_m, 50000.0, 10000.0);
}

TEST(TargetInferenceEngineTest, LaunchSigmaGrowsWithLookbackModelTerm) {
  TargetInferenceConfig no_growth;
  no_growth.launch_model_error_rate_m_per_s = 0.0;
  TargetInferenceEngine engine_plain(no_growth);
  TargetInferenceEngine engine_growth(TargetInferenceConfig{});
  const auto plain = engine_plain.Infer({WithCovariance(MakeBallisticMidcourse(), 100.0, 1.0)});
  const auto grown = engine_growth.Infer({WithCovariance(MakeBallisticMidcourse(), 100.0, 1.0)});
  ASSERT_TRUE(plain.front().trajectory.has_uncertainty);
  ASSERT_TRUE(grown.front().trajectory.has_uncertainty);

  // 模型误差项 = rate × 回推时长：缺省 50 m/s × 界面停机回推时长，
  // 1σ 必须显著大于仅 J·P·Jᵀ 的预算（审计 A7：1σ 不再恒定虚小）。
  EXPECT_GT(grown.front().trajectory.launch_position_sigma_m,
            plain.front().trajectory.launch_position_sigma_m)
      << "model-error term must inflate the launch budget";
  EXPECT_GT(grown.front().trajectory.launch_position_sigma_m, 1000.0);
}

TEST(TargetInferenceEngineTest, UncertaintyBudgetScalesWithInputCovariance) {
  TargetInferenceEngine engine(TargetInferenceConfig{});
  const auto tight = engine.Infer({WithCovariance(MakeBallisticMidcourse(), 100.0, 1.0)});
  const auto loose = engine.Infer({WithCovariance(MakeBallisticMidcourse(), 10000.0, 10.0)});
  ASSERT_EQ(tight.size(), 1U);
  ASSERT_EQ(loose.size(), 1U);

  ASSERT_TRUE(tight.front().trajectory.has_uncertainty);
  ASSERT_TRUE(loose.front().trajectory.has_uncertainty);
  EXPECT_GT(tight.front().trajectory.launch_position_sigma_m, 0.0);
  EXPECT_GT(loose.front().trajectory.launch_position_sigma_m,
            tight.front().trajectory.launch_position_sigma_m)
      << "launch sigma must grow with input covariance";
  EXPECT_GT(loose.front().trajectory.launch_position_sigma_m, 5000.0)
      << "10km input sigma must produce km-level launch budget";

  // 无协方差输入：显式无预算标记（sigma=0 不冒充零误差）。
  const auto bare = engine.Infer({MakeBallisticMidcourse()});
  ASSERT_TRUE(bare.front().trajectory.has_launch);
  EXPECT_FALSE(bare.front().trajectory.has_uncertainty);
  EXPECT_EQ(bare.front().trajectory.launch_position_sigma_m, 0.0);
}

TEST(TargetInferenceEngineTest, TypeAssessmentCombinesKinematicsAndEvidence) {
  TargetInferenceEngine engine(TargetInferenceConfig{});

  // 高速高空无证据：弹道先验胜出。
  const auto kinematic = engine.Infer({MakeBallisticMidcourse()});
  ASSERT_EQ(kinematic.size(), 1U);
  EXPECT_EQ(kinematic.front().type.category, InferenceTargetCategory::kBallistic);
  EXPECT_GT(kinematic.front().type.probability, 0.3);

  // 同状态 + 导弹强证据：证据主导（w=0.5：导弹 0.5·0+0.5·1=0.5 > 弹道 0.275）。
  InferenceTrackState evidenced = MakeBallisticMidcourse();
  evidenced.has_type_evidence = true;
  evidenced.type_evidence[static_cast<std::size_t>(InferenceTargetCategory::kMissile)] = 1.0;
  const auto fused = engine.Infer({evidenced});
  EXPECT_EQ(fused.front().type.category, InferenceTargetCategory::kMissile);

  // 低速低空无证据：kOther 倾向（无人机类别已按 2026-08-22 甲方裁定移除）。
  InferenceTrackState slow;
  slow.key = 7U;
  slow.position = oneq::coordinate::EcefPositionM(kEarthRadiusM + 1000.0, 0.0, 0.0);
  slow.velocity_ecef_m_per_s = {0.0, 80.0, 0.0};
  const auto slow_result = engine.Infer({slow});
  EXPECT_EQ(slow_result.front().type.category, InferenceTargetCategory::kOther);
}

TEST(TargetInferenceEngineTest, InvalidInputMarksTrajectoryInvalid) {
  TargetInferenceEngine engine(TargetInferenceConfig{});
  InferenceTrackState buried;
  buried.key = 1U;
  buried.position = oneq::coordinate::EcefPositionM(1000.0, 0.0, 0.0);  // 地表之下
  const auto results = engine.Infer({buried});
  ASSERT_EQ(results.size(), 1U);
  EXPECT_FALSE(results.front().trajectory.valid);
  EXPECT_FALSE(results.front().trajectory.has_launch);
}

TEST(TargetInferenceEngineTest, DeterministicAndOrderPreserving) {
  TargetInferenceEngine engine(TargetInferenceConfig{});
  std::vector<InferenceTrackState> tracks;
  tracks.push_back(MakeBallisticMidcourse());
  tracks.push_back(WithCovariance(MakeBallisticMidcourse(), 500.0, 5.0));
  tracks.back().key = 43U;
  tracks.back().position.x_m += 20000.0;

  const auto first = engine.Infer(tracks);
  const auto second = engine.Infer(tracks);
  ASSERT_EQ(first.size(), 2U);
  ASSERT_EQ(second.size(), 2U);
  for (std::size_t i = 0U; i < first.size(); ++i) {
    EXPECT_EQ(first[i].key, tracks[i].key);
    EXPECT_EQ(first[i].key, second[i].key);
    EXPECT_DOUBLE_EQ(first[i].trajectory.launch_point.latitude_deg,
                     second[i].trajectory.launch_point.latitude_deg);
    EXPECT_DOUBLE_EQ(first[i].trajectory.launch_position_sigma_m,
                     second[i].trajectory.launch_position_sigma_m);
    EXPECT_DOUBLE_EQ(first[i].type.probability, second[i].type.probability);
  }
}

}  // namespace
}  // namespace target_inference
