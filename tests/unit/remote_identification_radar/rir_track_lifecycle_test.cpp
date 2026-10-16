// Copyright 2026. All Rights Reserved.
//
// @file rir_track_lifecycle_test.cpp
// @brief 验证 RIR 轻量跟踪子集航迹生命周期与 KF 接线（阶段 2-T T3/T4）。

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

#include "remote_identification_radar/tracking/RirTrackLifecycle.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using tracking::RirCycleContext;
using tracking::RirLifecycleConfig;
using tracking::RirMeasurementCovariance;
using tracking::RirTrackFilterConfig;
using tracking::RirTrackLifecycle;
using tracking::RirTrackMeasurement;
using tracking::RirTrackSeed;
using tracking::RirTrackSnapshotList;
using tracking::RirTrackState;
using tracking::RirTrackStatus;

RirTrackMeasurement MakeMeasurement(std::uint64_t key, float px, float vx,
                                    bool matched_existing = false, std::uint64_t external_id = 0U,
                                    const std::string& target_name = "") {
  RirTrackMeasurement measurement;
  measurement.association_key = key;
  measurement.matched_existing_track = matched_existing;
  measurement.position = Eigen::Vector3f(px, 0.0f, 0.0f);
  measurement.velocity = Eigen::Vector3f(vx, 0.0f, 0.0f);
  measurement.measurement_covariance = RirMeasurementCovariance::Identity();
  measurement.external_target_id = external_id;
  measurement.target_name = target_name;
  return measurement;
}

RirCycleContext MakeCycle(std::uint32_t cycle_index, std::uint64_t batch_id, float dt_sec = 1.0f) {
  RirCycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.batch_id = batch_id;
  cycle.dt_sec = dt_sec;
  return cycle;
}

/// @brief tentative 达到 confirm_hits 后转 confirmed；hit/miss 计数语义正确。
TEST(RirTrackLifecycleTest, ConfirmsTrackAfterConfiguredHits) {
  RirLifecycleConfig config;
  config.confirm_hits = 2U;
  config.max_miss_before_lost = 1U;
  config.max_lost_cycles = 2U;
  RirTrackLifecycle lifecycle(config);

  lifecycle.Update(MakeCycle(1U, 1001U),
                   {MakeMeasurement(7U, 0.0f, 3.0f, false, 7001U, "target-a")});

  RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].status, RirTrackStatus::kTentative);
  EXPECT_EQ(snapshots[0].hit_count, 1U);
  EXPECT_EQ(snapshots[0].miss_count, 0U);

  lifecycle.Update(MakeCycle(2U, 1002U), {MakeMeasurement(7U, 0.0f, 4.0f, true)});

  snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].status, RirTrackStatus::kConfirmed);
  EXPECT_EQ(snapshots[0].hit_count, 2U);
  // 去真值化后速度证据只来自位置差分：两拍位置相同 → 后验速度≈0（速度无知先验
  // 9e6 m²/s² 下量测主导；旧断言 1.5025 依赖真值速度种子 3 m/s + 小速度方差）。
  EXPECT_NEAR(snapshots[0].velocity.x(), 0.0f, 0.01f);
  EXPECT_NEAR(snapshots[0].speed, 0.0f, 0.01f);
  // 加速度 = 后验速度的周期间差分：首拍种子 3（测试 helper 显式传入）→ 本拍 ≈0，
  // 差分 ≈ −3 m/s²；量测速度字段从 3 改到 4 不影响（种子不参与第二拍证据）。
  EXPECT_NEAR(snapshots[0].acceleration_mps2, 3.0f, 0.05f);
  EXPECT_EQ(snapshots[0].external_target_id, 7001U);
  EXPECT_EQ(snapshots[0].target_name, "target-a");
  // EstimationUncertaintyTrace 只累加位置分块（初始 3×100=300，速度无知先验
  // 不进该口径）；两拍后位置迹收敛至个位数量级，保持紧界。
  EXPECT_LT(snapshots[0].EstimationUncertaintyTrace(), 300.0f);
}

/// @brief 连续失配：confirmed → lost → 超时回收，回收后键不再出现在快照。
TEST(RirTrackLifecycleTest, RecyclesTrackAfterLostTimeout) {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  config.max_miss_before_lost = 0U;
  config.max_lost_cycles = 1U;
  RirTrackLifecycle lifecycle(config);

  lifecycle.Update(MakeCycle(1U, 2001U), {MakeMeasurement(3U, 0.0f, 1.0f)});
  lifecycle.Update(MakeCycle(2U, 2002U), {});

  RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].status, RirTrackStatus::kLost);
  EXPECT_EQ(snapshots[0].miss_count, 1U);

  lifecycle.Update(MakeCycle(3U, 2003U), {});
  EXPECT_TRUE(lifecycle.BuildTrackSnapshots().empty());
}

/// @brief lost 航迹再次命中立即确认，并从量测速度种子重初始化（无速度尖峰）。
TEST(RirTrackLifecycleTest, LostTrackRehitResetsFilterAndConfirms) {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  config.max_miss_before_lost = 0U;
  config.max_lost_cycles = 3U;
  RirTrackLifecycle lifecycle(config);

  lifecycle.Update(MakeCycle(1U, 2301U), {MakeMeasurement(31U, 0.0f, 0.0f, false, 8001U)});
  lifecycle.Update(MakeCycle(2U, 2302U), {});

  ASSERT_EQ(lifecycle.BuildTrackSnapshots()[0].status, RirTrackStatus::kLost);
  lifecycle.Update(MakeCycle(3U, 2303U), {MakeMeasurement(31U, 300.0f, 0.2f, true, 8001U)});

  const RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].status, RirTrackStatus::kConfirmed);
  EXPECT_FLOAT_EQ(snapshots[0].velocity.x(), 0.2f);
  EXPECT_LT(std::sqrt(snapshots[0].velocity.x() * snapshots[0].velocity.x()), 1.0f);
  // 重捕周期滤波速度以速度种子重初始化、无差分基准：加速度显式置零（无尖峰）。
  EXPECT_FLOAT_EQ(snapshots[0].acceleration.x(), 0.0f);
}

/// @brief 航迹回收后新键获得新 track_id；键重分配语义由单调键保证。
/// N3 池化：回收槽位被复用，generation 单调递增标识旧引用。
TEST(RirTrackLifecycleTest, ReusedKeyAfterRecycleStartsAsNewTrack) {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  config.max_miss_before_lost = 0U;
  config.max_lost_cycles = 1U;
  RirTrackLifecycle lifecycle(config);

  lifecycle.Update(MakeCycle(1U, 2101U), {MakeMeasurement(11U, 100.0f, 5.0f)});
  ASSERT_EQ(lifecycle.BuildTrackSnapshots()[0].track_id, 1U);
  EXPECT_EQ(lifecycle.BuildTrackSnapshots()[0].generation, 0U);
  lifecycle.Update(MakeCycle(2U, 2102U), {});
  lifecycle.Update(MakeCycle(3U, 2103U), {});
  EXPECT_TRUE(lifecycle.BuildTrackSnapshots().empty());

  lifecycle.Update(MakeCycle(4U, 2104U), {MakeMeasurement(12U, 120.0f, 6.0f)});
  const RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].association_key, 12U);
  EXPECT_EQ(snapshots[0].track_id, 2U);
  EXPECT_EQ(snapshots[0].hit_count, 1U);
  // 复用槽位携带代次：新航迹的 generation = 槽位被回收次数（LIFO 复用刚回收的槽位）。
  EXPECT_EQ(snapshots[0].generation, 1U);
}

/// @brief 关联种子导出包含位置与高斯状态，供下一周期门控。
TEST(RirTrackLifecycleTest, BuildAssociationSeedsExportsGaussianState) {
  RirTrackLifecycle lifecycle;
  lifecycle.Update(MakeCycle(1U, 2201U), {MakeMeasurement(21U, 100.0f, 2.0f)});

  const std::vector<RirTrackSeed> seeds = lifecycle.BuildAssociationSeeds();
  ASSERT_EQ(seeds.size(), 1U);
  EXPECT_EQ(seeds[0].association_key, 21U);
  EXPECT_TRUE(seeds[0].has_position);
  EXPECT_TRUE(seeds[0].has_gaussian_state);
  EXPECT_FLOAT_EQ(seeds[0].position.x(), 100.0f);
  EXPECT_FLOAT_EQ(seeds[0].gaussian_state.mean(0), 100.0f);
}

/// @brief 非法 dt：整周期跳过，不建轨也不推进既有状态。
TEST(RirTrackLifecycleTest, InvalidDtSkipsCycleWithoutStateChange) {
  RirTrackLifecycle lifecycle;
  lifecycle.Update(MakeCycle(1U, 2401U), {MakeMeasurement(41U, 10.0f, 1.0f)});
  const RirTrackState before = lifecycle.BuildTrackSnapshots()[0];

  lifecycle.Update(MakeCycle(2U, 2402U, 0.0f), {MakeMeasurement(41U, 20.0f, 3.0f, true)});

  const RirTrackSnapshotList after = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(after.size(), 1U);
  EXPECT_FLOAT_EQ(after[0].position.x(), before.position.x());
  EXPECT_FLOAT_EQ(after[0].velocity.x(), before.velocity.x());
  EXPECT_EQ(after[0].last_update_cycle, before.last_update_cycle);
}

/// @brief 运行态捕获/恢复往返：航迹、track_id 分配器与周期号全部回滚。
TEST(RirTrackLifecycleTest, RuntimeStateRoundTrip) {
  RirTrackLifecycle source;
  source.Update(MakeCycle(1U, 2501U), {MakeMeasurement(51U, 10.0f, 1.0f)});
  source.Update(MakeCycle(2U, 2502U), {MakeMeasurement(52U, 20.0f, 2.0f)});
  const auto state = source.CaptureRuntimeState();

  RirTrackLifecycle restored;
  restored.RestoreRuntimeState(state);
  EXPECT_EQ(restored.ActiveTrackCount(), 2U);
  // 捕获发生在第 2 周期后：key=51 已按 dt=1 外推一个周期。
  EXPECT_FLOAT_EQ(restored.FindTrack(51U)->position.x(), 11.0f);
  EXPECT_FLOAT_EQ(restored.FindTrack(51U)->velocity.x(), 1.0f);
  EXPECT_EQ(restored.FindTrack(52U)->position.x(), 20.0f);

  restored.Update(MakeCycle(3U, 2503U), {MakeMeasurement(53U, 30.0f, 3.0f)});
  EXPECT_EQ(restored.FindTrack(53U)->track_id, 3U);
}

/// @brief 快照按关联键升序，保证 replay/日志输出确定性。
TEST(RirTrackLifecycleTest, SnapshotsAreOrderedByAssociationKey) {
  RirTrackLifecycle lifecycle;
  lifecycle.Update(MakeCycle(1U, 2601U),
                   {MakeMeasurement(20U, 10.0f, 1.0f), MakeMeasurement(5U, 20.0f, 2.0f)});

  const RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 2U);
  EXPECT_EQ(snapshots[0].association_key, 5U);
  EXPECT_EQ(snapshots[1].association_key, 20U);
  // 同周期多新航迹的 track_id 按输入顺序分配（replay 确定性）。
  EXPECT_EQ(snapshots[1].track_id, 1U);
  EXPECT_EQ(snapshots[0].track_id, 2U);
}

/// @brief 退化量测（速度向量为零）：速度唯一来源是滤波后验，无种子回填路径；
///        位置量测仍按匀速真值给出时滤波速度与加速度均保持无扰动。
TEST(RirTrackLifecycleTest, DegradedZeroVectorVelocityKeepsFilterContinuity) {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  config.max_miss_before_lost = 2U;
  config.max_lost_cycles = 5U;
  RirTrackLifecycle lifecycle(config);

  lifecycle.Update(MakeCycle(1U, 2701U), {MakeMeasurement(7U, 0.0f, 10.0f, false)});
  lifecycle.Update(MakeCycle(2U, 2702U), {MakeMeasurement(7U, 10.0f, 10.0f, true)});

  // 退化量测：速度向量为全零（无独立标量速度源可回填）。
  lifecycle.Update(MakeCycle(3U, 2703U), {MakeMeasurement(7U, 20.0f, 0.0f, true)});

  const RirTrackState* track = lifecycle.FindTrack(7U);
  ASSERT_NE(track, nullptr);
  // 零新息下后验速度保持 ≈10 m/s；加速度为后验差分（上周期后验 10 → 本周期 10）→ 0。
  EXPECT_NEAR(track->velocity.x(), 10.0f, 0.01f);
  EXPECT_NEAR(track->acceleration.x(), 0.0f, 1e-4f);
  EXPECT_NEAR(track->acceleration_mps2, 0.0f, 1e-4f);
}

/// @brief 加速度 = 后验速度周期间差分再做一阶指数平滑（识别消费口径，2026-10-16）。
///        匀加速目标（v0=10 m/s、a=4 m/s²、dt=1 s）：建轨周期置零，此后首个差分样本
///        直接起批，后续按 α=1/30 递推；平滑不改变收敛均值，仍逼近真值 4 m/s² 且方向
///        为正（旧种子差分口径为负向虚高）。
///        为什么加平滑：单周期差分的噪声按 1/dt 放大——实测弹道目标真值加速度仅
///        4.4~11 m/s²，而报出 0~40 m/s²（均值 31.9），该维在识别打分里变成噪声放大器。
TEST(RirTrackLifecycleTest, AccelerationIsSmoothedPosteriorVelocityDifference) {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  config.max_miss_before_lost = 3U;
  config.max_lost_cycles = 5U;
  config.enable_imm_lifecycle = false;  // CV 路径，聚焦差分口径本身
  RirTrackLifecycle lifecycle(config);

  // 真值运动学：p(k)=10k+2k²，v(k)=10+4k（匀加速度真值 4 m/s²）。
  lifecycle.Update(MakeCycle(1U, 2801U), {MakeMeasurement(9U, 12.0f, 14.0f)});
  ASSERT_NE(lifecycle.FindTrack(9U), nullptr);
  // 建轨周期：滤波以速度种子重初始化，重置分支清零加速度并复位起批标志。
  EXPECT_FLOAT_EQ(lifecycle.FindTrack(9U)->acceleration.x(), 0.0f);
  EXPECT_FALSE(lifecycle.FindTrack(9U)->acceleration_smoothed);

  // ---- 短窗：起批一次性 + EMA 递推 + 输出有界 ----
  float previous_velocity = lifecycle.FindTrack(9U)->velocity.x();
  float previous_smoothed = 0.0f;
  float peak_instantaneous = 0.0f;
  for (std::uint32_t cycle = 2U; cycle <= 8U; ++cycle) {
    const float position =
        10.0f * static_cast<float>(cycle) +
        2.0f * static_cast<float>(cycle) * static_cast<float>(cycle);
    lifecycle.Update(MakeCycle(cycle, 2800U + cycle),
                     {MakeMeasurement(9U, position, 10.0f + 4.0f * static_cast<float>(cycle),
                                      true)});
    const RirTrackState* track = lifecycle.FindTrack(9U);
    ASSERT_NE(track, nullptr);
    // 平滑前的瞬时差分 = 后验速度差分（速度种子不参与基准）。
    const float instantaneous = (track->velocity.x() - previous_velocity) / 1.0f;
    const float smoothed = track->acceleration.x();
    peak_instantaneous = std::max(peak_instantaneous, std::fabs(instantaneous));
    // α<1 的凸组合不含外推 ⇒ 输出幅度不得超过历史瞬时幅度峰值。
    EXPECT_LE(std::fabs(smoothed), peak_instantaneous + 1.0e-4f);
    if (cycle == 2U) {
      // 重置后首个差分样本直接起批（不被 1/τ 衰减）。
      EXPECT_NEAR(smoothed, instantaneous, 1.0e-4f);
    } else {
      // 此后按 α=1/30 递推：a_k = (1-α)·a_{k-1} + α·瞬时差分。
      EXPECT_NEAR(
          smoothed,
          (1.0f - 1.0f / 30.0f) * previous_smoothed + (1.0f / 30.0f) * instantaneous, 1.0e-3f);
    }
    previous_smoothed = smoothed;
    previous_velocity = track->velocity.x();
  }
  EXPECT_TRUE(lifecycle.FindTrack(9U)->acceleration_smoothed);

  // ---- 长窗：噪声衰减 + 无偏收敛 ----
  // 继续跑到周期 120，记录平滑输出与同拍瞬时差分的峰值，并取末段均值。
  float raw_max = 0.0f;
  float smooth_max = 0.0f;
  float tail_sum = 0.0f;
  std::uint32_t tail_count = 0U;
  for (std::uint32_t cycle = 9U; cycle <= 120U; ++cycle) {
    const float position =
        10.0f * static_cast<float>(cycle) +
        2.0f * static_cast<float>(cycle) * static_cast<float>(cycle);
    lifecycle.Update(MakeCycle(cycle, 2800U + cycle),
                     {MakeMeasurement(9U, position, 10.0f + 4.0f * static_cast<float>(cycle),
                                      true)});
    const RirTrackState* track = lifecycle.FindTrack(9U);
    ASSERT_NE(track, nullptr);
    const float instantaneous = (track->velocity.x() - previous_velocity) / 1.0f;
    raw_max = std::max(raw_max, std::fabs(instantaneous));
    smooth_max = std::max(smooth_max, std::fabs(track->acceleration.x()));
    if (cycle > 110U) {
      tail_sum += track->acceleration.x();
      ++tail_count;
    }
    previous_velocity = track->velocity.x();
  }

  // ① 平滑确实压低了幅度峰值（加它进识别的全部理由）。
  EXPECT_LE(smooth_max, raw_max);
  // ② 末段收敛到真值 4 m/s² 量级：EMA 不改变均值、只降方差。
  EXPECT_NEAR(tail_sum / static_cast<float>(tail_count), 4.0f, 1.0f);
}

/// @brief IMM 模型集锚定滤波配置 q：模型 q = 配置 q 的对数等距张成
///        （q=30 → {30, 300}），配置不再被缺省 {1,10} 覆盖（审计 A5）。
TEST(RirTrackLifecycleTest, ImmModelSetAnchorsConfiguredProcessNoise) {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  RirTrackFilterConfig filter_config;
  filter_config.process_noise_diff_coeff = 30.0f;
  RirTrackLifecycle lifecycle(config, filter_config);

  lifecycle.Update(MakeCycle(1U, 3001U), {MakeMeasurement(7U, 0.0f, 5.0f)});
  lifecycle.Update(MakeCycle(2U, 3002U), {MakeMeasurement(7U, 5.0f, 5.0f, true)});

  const tracking::RirImmFilter* imm = lifecycle.FindImmFilter(7U);
  ASSERT_NE(imm, nullptr);
  ASSERT_EQ(imm->ModelCount(), 2U);
  EXPECT_NEAR(imm->model_noise_diff_coeffs()[0], 30.0f, 1e-3f);
  EXPECT_NEAR(imm->model_noise_diff_coeffs()[1], 300.0f, 1e-3f);
}

/// @brief 加速度差分的时间基准 = **距上次命中的实际经过时间**，不按单周期 dt 膨胀。
///        覆盖两条路径：
///        ① 滑行未失跟（confirmed 保持）→ 走主滤波分支，分母取 (m+1)·dt；
///        ② 失跟重捕 → 走重置分支，加速度清零并复位起批标志（防重捕后长期低报）。
TEST(RirTrackLifecycleTest, RehitAfterCoastDividesByElapsedTime) {
  // ---- ① 滑行 1 拍仍保持 confirmed：分母按 2 s 折算 ----
  {
    RirLifecycleConfig config;
    config.confirm_hits = 1U;
    config.max_miss_before_lost = 3U;  // 失配 1 拍不失跟
    config.max_lost_cycles = 5U;
    config.enable_imm_lifecycle = false;  // CV 路径，聚焦差分口径本身
    RirTrackLifecycle lifecycle(config);

    // 周期1 命中建轨；周期2 失配滑行；周期3 重命中加速目标。
    lifecycle.Update(MakeCycle(1U, 2901U), {MakeMeasurement(9U, 0.0f, 10.0f)});
    lifecycle.Update(MakeCycle(2U, 2902U), {});
    const RirTrackState* coasted = lifecycle.FindTrack(9U);
    ASSERT_NE(coasted, nullptr);
    ASSERT_EQ(coasted->status, RirTrackStatus::kConfirmed);
    // 滑行拍也走平滑（瞬时差分 ≈0），故起批标志在重命中前已是 true。
    ASSERT_TRUE(coasted->acceleration_smoothed);
    const float velocity_before_rehit = coasted->velocity.x();
    const float smoothed_before_rehit = coasted->acceleration.x();

    lifecycle.Update(MakeCycle(3U, 2903U), {MakeMeasurement(9U, 48.0f, 18.0f, true)});
    const RirTrackState* track = lifecycle.FindTrack(9U);
    ASSERT_NE(track, nullptr);
    // 瞬时差分分母 = 实际经过时间 2 s（若误用单周期 dt，该值会膨胀 2 倍）。
    const float instantaneous = (track->velocity.x() - velocity_before_rehit) / 2.0f;
    EXPECT_NEAR(track->acceleration.x(),
                (1.0f - 1.0f / 30.0f) * smoothed_before_rehit + (1.0f / 30.0f) * instantaneous,
                1.0e-3f);
  }

  // ---- ② 失跟重捕：重置分支清零 + 复位起批标志 ----
  {
    RirLifecycleConfig config;
    config.confirm_hits = 1U;
    config.max_miss_before_lost = 0U;  // 失配 1 拍即 lost
    config.max_lost_cycles = 5U;
    config.enable_imm_lifecycle = false;
    RirTrackLifecycle lifecycle(config);

    lifecycle.Update(MakeCycle(1U, 2911U), {MakeMeasurement(11U, 0.0f, 10.0f)});
    lifecycle.Update(MakeCycle(2U, 2912U), {});
    ASSERT_EQ(lifecycle.FindTrack(11U)->status, RirTrackStatus::kLost);
    lifecycle.Update(MakeCycle(3U, 2913U), {MakeMeasurement(11U, 48.0f, 18.0f, true)});

    const RirTrackState* track = lifecycle.FindTrack(11U);
    ASSERT_NE(track, nullptr);
    EXPECT_EQ(track->status, RirTrackStatus::kConfirmed);
    // 重初始化周期：加速度清零、起批标志复位（下次命中重新以瞬时差分起批）。
    EXPECT_FLOAT_EQ(track->acceleration.x(), 0.0f);
    EXPECT_FLOAT_EQ(track->acceleration_mps2, 0.0f);
    EXPECT_FALSE(track->acceleration_smoothed);
  }
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
