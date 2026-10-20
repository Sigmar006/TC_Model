// Copyright 2026. All Rights Reserved.
//
// @file rir_imm_tracking_test.cpp
// @brief 验证 RIR IMM 生命周期双路径（阶段 2-T N4/N5）。

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <vector>

#include "remote_identification_radar/tracking/RirImmFilter.h"
#include "remote_identification_radar/tracking/RirTrackLifecycle.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using tracking::RirCycleContext;
using tracking::RirLifecycleConfig;
using tracking::RirMeasurementCovariance;
using tracking::RirTrackLifecycle;
using tracking::RirTrackMeasurement;
using tracking::RirTrackSnapshotList;
using tracking::RirTrackStatus;

RirTrackMeasurement MakeMeasurement(std::uint64_t key, float px, float vx,
                                    bool matched_existing) {
  RirTrackMeasurement measurement;
  measurement.association_key = key;
  measurement.matched_existing_track = matched_existing;
  measurement.position = Eigen::Vector3f(px, 0.0f, 0.0f);
  measurement.velocity = Eigen::Vector3f(vx, 0.0f, 0.0f);
  measurement.measurement_covariance = RirMeasurementCovariance::Identity();
  return measurement;
}

RirCycleContext MakeCycle(std::uint32_t cycle_index, float dt_sec = 1.0f) {
  RirCycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.batch_id = 1U;
  cycle.dt_sec = dt_sec;
  return cycle;
}

RirLifecycleConfig MakeImmConfig() {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  config.max_miss_before_lost = 2U;
  config.max_lost_cycles = 5U;
  config.enable_imm_lifecycle = true;
  config.model_count_hint = 2U;
  return config;
}

/// @brief IMM 缺省未启用：confirmed 命中不建 IMM 运行态（CV KF 路径），
///        与 2-T 既有行为数值一致（既有生命周期用例锁定）。
TEST(RirImmTrackingTest, DisabledLifecycleKeepsCvPathWithoutImmState) {
  RirLifecycleConfig config = MakeImmConfig();
  config.enable_imm_lifecycle = false;
  RirTrackLifecycle lifecycle(config);

  lifecycle.Update(MakeCycle(1U), {MakeMeasurement(7U, 0.0f, 10.0f, false)});
  lifecycle.Update(MakeCycle(2U), {MakeMeasurement(7U, 10.0f, 10.0f, true)});
  lifecycle.Update(MakeCycle(3U), {MakeMeasurement(7U, 20.0f, 10.0f, true)});

  EXPECT_FALSE(lifecycle.IsImmEnabled());
  EXPECT_EQ(lifecycle.FindImmFilter(7U), nullptr);
  const RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].status, RirTrackStatus::kConfirmed);
}

/// @brief B6 包络门的数据侧契约：航迹可被整体丢弃（出表、快照消失、重复丢弃返回 false），
///        丢弃后同一关联键重建航迹时不带回旧状态。
TEST(RirImmTrackingTest, DropTrackRemovesTrackAndAllowsCleanRebuild) {
  RirTrackLifecycle lifecycle(MakeImmConfig());

  lifecycle.Update(MakeCycle(1U), {MakeMeasurement(11U, 1.0e5f, 100.0f, false)});
  ASSERT_EQ(lifecycle.ActiveTrackCount(), 1U);
  ASSERT_EQ(lifecycle.BuildTrackSnapshots().size(), 1U);

  EXPECT_TRUE(lifecycle.DropTrack(11U));
  EXPECT_EQ(lifecycle.ActiveTrackCount(), 0U);
  EXPECT_TRUE(lifecycle.BuildTrackSnapshots().empty());
  EXPECT_EQ(lifecycle.FindTrack(11U), nullptr);
  EXPECT_FALSE(lifecycle.DropTrack(11U)) << "重复丢弃同一关联键必须返回 false";

  lifecycle.Update(MakeCycle(2U), {MakeMeasurement(11U, 5.0e5f, 200.0f, false)});
  const RirTrackSnapshotList rebuilt = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(rebuilt.size(), 1U);
  EXPECT_NEAR(rebuilt[0].position.x(), 5.0e5f, 1.0e3f);
}

/// @brief confirmed 命中激活 IMM：运行态按关联键建立，权重归一；
///        失配周期对 confirmed 航迹执行 IMM 仅预测（位置按速度推进）。
TEST(RirImmTrackingTest, ConfirmedMatchBuildsImmAndMissPredictsForward) {
  RirTrackLifecycle lifecycle(MakeImmConfig());

  lifecycle.Update(MakeCycle(1U), {MakeMeasurement(7U, 0.0f, 10.0f, false)});
  ASSERT_EQ(lifecycle.FindImmFilter(7U), nullptr);  // 新建航迹（未命中既有）不激活
  lifecycle.Update(MakeCycle(2U), {MakeMeasurement(7U, 10.0f, 10.0f, true)});
  ASSERT_NE(lifecycle.FindImmFilter(7U), nullptr);  // confirmed 命中激活 IMM

  const Eigen::VectorXf weights = lifecycle.FindImmFilter(7U)->GetModelWeights();
  ASSERT_EQ(weights.size(), 2);
  EXPECT_NEAR(weights.sum(), 1.0f, 1.0e-5f);

  lifecycle.Update(MakeCycle(3U), {});  // 失配：IMM 仅预测
  const RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_TRUE(snapshots[0].position.x() > 10.0f);  // 速度 10 m/s 推进一个周期
  EXPECT_LT(snapshots[0].position.x(), 25.0f);     // 组合预测不外推超过速度积分上界
  EXPECT_TRUE(std::isfinite(snapshots[0].speed));
}

/// @brief 高机动切换：持续强加速命中使高过程噪声模型权重占优，
///        且位置/速度连续（无发散/跳变）。
TEST(RirImmTrackingTest, ManeuverShiftsWeightToHighNoiseModelAndStaysContinuous) {
  RirTrackLifecycle lifecycle(MakeImmConfig());

  // 匀速段：建立 confirmed 航迹并激活 IMM。
  lifecycle.Update(MakeCycle(1U), {MakeMeasurement(7U, 0.0f, 10.0f, false)});
  lifecycle.Update(MakeCycle(2U), {MakeMeasurement(7U, 10.0f, 10.0f, true)});
  lifecycle.Update(MakeCycle(3U), {MakeMeasurement(7U, 20.0f, 10.0f, true)});

  // 机动段：每周期 +8 m/s（强加速度），量测位置按积分真值给出。
  float position = 20.0f;
  float velocity = 10.0f;
  for (std::uint32_t cycle = 4U; cycle <= 10U; ++cycle) {
    velocity += 8.0f;
    position += velocity;
    lifecycle.Update(MakeCycle(cycle),
                     {MakeMeasurement(7U, position, velocity, true)});
  }

  const tracking::RirImmFilter* imm = lifecycle.FindImmFilter(7U);
  ASSERT_NE(imm, nullptr);
  const Eigen::VectorXf weights = imm->GetModelWeights();
  // 模型序按噪声由低到高（{1.0, 10.0}）：高噪声模型应占优。
  EXPECT_GT(weights(1), weights(0));
  EXPECT_GT(weights(1), 0.5f);

  const RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_TRUE(snapshots[0].position.allFinite());
  EXPECT_TRUE(snapshots[0].velocity.allFinite());
  // 连续性：机动段后滤波位置与量测真值的偏差在速度量级内（不发散）。
  EXPECT_LT(std::fabs(snapshots[0].position.x() - position), 50.0f);
}

/// @brief RirImmFilter 直接契约：缺省双模型 {1.0, 10.0}；转移矩阵行归一。
TEST(RirImmFilterTest, DefaultDualModelIsValid) {
  tracking::RirImmFilter filter;  // 缺省配置
  EXPECT_TRUE(filter.IsValid());
  EXPECT_EQ(filter.ModelCount(), 2U);

  filter.Process(Eigen::Vector3f(1.0f, 2.0f, 3.0f), 1.0f, RirMeasurementCovariance::Identity());
  const Eigen::VectorXf weights = filter.GetModelWeights();
  ASSERT_EQ(weights.size(), 2);
  EXPECT_NEAR(weights.sum(), 1.0f, 1.0e-5f);
  EXPECT_TRUE(filter.GetCombinedState().mean.allFinite());
}

/// @brief RirImmFilter 运行时热调参契约（AR SyncRuntimeTuning 同口径）：
///        同模型数在线重调每模型 q 与转移矩阵，已演化权重/状态保留；
///        模型数变化拒绝原位重调（调用方丢弃运行态并惰性重建）。
TEST(RirImmFilterTest, UpdateRuntimeTuningRetunesWithinSameModelCount) {
  tracking::RirImmFilter filter;
  ASSERT_TRUE(filter.IsValid());
  filter.Process(Eigen::Vector3f(1.0f, 2.0f, 3.0f), 1.0f, RirMeasurementCovariance::Identity());
  const Eigen::VectorXf weights_before = filter.GetModelWeights();

  tracking::RirImmFilter::Config config;
  config.model_noise_diff_coeffs = {0.5f, 20.0f};
  config.transition_diagonal_probability = 0.9f;
  EXPECT_TRUE(filter.UpdateRuntimeTuning(config));
  EXPECT_TRUE(filter.IsValid());
  EXPECT_EQ(filter.ModelCount(), 2U);

  // 热调参不重置已演化的模型权重。
  const Eigen::VectorXf weights_after = filter.GetModelWeights();
  ASSERT_EQ(weights_after.size(), weights_before.size());
  for (Eigen::Index i = 0; i < weights_after.size(); ++i) {
    EXPECT_NEAR(weights_after(i), weights_before(i), 1.0e-6f);
  }

  // 重调后数值链路仍可用。
  filter.Process(Eigen::Vector3f(2.0f, 2.0f, 3.0f), 1.0f, RirMeasurementCovariance::Identity());
  EXPECT_TRUE(filter.GetCombinedState().mean.allFinite());

  tracking::RirImmFilter::Config mismatched;
  mismatched.model_noise_diff_coeffs = {1.0f, 10.0f, 100.0f};  // 模型数变化
  EXPECT_FALSE(filter.UpdateRuntimeTuning(mismatched));
}

/// @brief 生命周期 UpdateConfig 热同步既有 IMM 运行态（AR SyncRuntimeTuning 同口径）：
///        模型数不变 → 原位重调、运行态实例保留；模型数变化 → 运行态丢弃、
///        下次 confirmed 命中按新模型集惰性重建。
TEST(RirImmTrackingTest, UpdateConfigHotTunesExistingImmRuntimeState) {
  RirTrackLifecycle lifecycle(MakeImmConfig());
  lifecycle.Update(MakeCycle(1U), {MakeMeasurement(7U, 0.0f, 10.0f, false)});
  lifecycle.Update(MakeCycle(2U), {MakeMeasurement(7U, 10.0f, 10.0f, true)});
  const tracking::RirImmFilter* imm_before = lifecycle.FindImmFilter(7U);
  ASSERT_NE(imm_before, nullptr);
  ASSERT_EQ(imm_before->ModelCount(), 2U);

  lifecycle.UpdateConfig(MakeImmConfig(), tracking::RirTrackFilterConfig{});
  EXPECT_EQ(lifecycle.FindImmFilter(7U), imm_before);  // 原位重调，不丢弃运行态
  EXPECT_EQ(lifecycle.FindImmFilter(7U)->ModelCount(), 2U);

  RirLifecycleConfig three_models = MakeImmConfig();
  three_models.model_count_hint = 3U;
  lifecycle.UpdateConfig(three_models, tracking::RirTrackFilterConfig{});
  EXPECT_EQ(lifecycle.FindImmFilter(7U), nullptr);  // 模型数变化 → 运行态丢弃

  lifecycle.Update(MakeCycle(3U), {MakeMeasurement(7U, 20.0f, 10.0f, true)});
  const tracking::RirImmFilter* rebuilt = lifecycle.FindImmFilter(7U);
  ASSERT_NE(rebuilt, nullptr);
  EXPECT_EQ(rebuilt->ModelCount(), 3U);  // 惰性重建按新模型集
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
