// Copyright 2026. All Rights Reserved.
//
// @file rir_recognition_feature_test.cpp
// @brief 验证远程识别观测构造器与四类特征提取器的效能级行为。

#include <gtest/gtest.h>

#include <cmath>
#include <unordered_map>

#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "1q/remote_identification_radar/session/RirSceneTypes.h"
#include "remote_identification_radar/recognition/MotionFeatureExtractor.h"
#include "remote_identification_radar/recognition/PolarizationFeatureExtractor.h"
#include "remote_identification_radar/recognition/RangeProfileFeatureExtractor.h"
#include "remote_identification_radar/recognition/RcsFeatureExtractor.h"
#include "remote_identification_radar/recognition/RecognitionObservationBuilder.h"
#include "remote_identification_radar/recognition/RecognitionTracker.h"
#include "remote_identification_radar/recognition/RecognitionTypes.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirAspectRcsSample;
using session::RirPolSMatrixSample;
using session::RirRangeRcsScatterer;
using session::RirRecognitionFeatureDimension;
using session::RirSceneTarget;
using tracking::RirStateCovariance;
using tracking::RirTrackState;
using tracking::RirTrackStatus;

recognition::RirObservationContext MakeContext(float snr_db = 20.0f, float bandwidth_hz = 3.0e6f,
                                               float range_m = 100000.0f) {
  recognition::RirObservationContext context;
  context.snr_db = snr_db;
  context.bandwidth_hz = bandwidth_hz;
  context.range_m = range_m;
  context.dwell_sec = 0.05f;  // 标称驻留
  context.look_az_deg = -30.0f;
  context.look_el_deg = 5.0f;
  return context;
}

TEST(RirObservationBuilderTest, EmptyFeatureListsProduceZeroMask) {
  RirSceneTarget target;
  RirTrackState snapshot;  // 默认快照（kTentative）
  const recognition::RirFeatureSet set =
      recognition::RirObservationBuilder::Build(target, snapshot, MakeContext());
  EXPECT_EQ(set.valid_feature_mask, 0U);
  EXPECT_FALSE(set.rcs.valid);
  EXPECT_FALSE(set.motion.valid);
  EXPECT_FALSE(set.polarization.valid);
  EXPECT_FALSE(set.range_profile.valid);
}

TEST(RirRcsFeatureExtractorTest, SingleSampleAtLookAngleYieldsMeanWithinSnrTolerance) {
  RirAspectRcsSample sample;
  sample.aspect_az_deg = -30.0f;
  sample.aspect_el_deg = 5.0f;
  sample.rcs_dbsm = -3.0f;
  std::vector<RirAspectRcsSample> samples = {sample};

  const recognition::RirRcsObservation observation =
      recognition::RirRcsFeatureExtractor::Extract(samples, -30.0f, 5.0f, 20.0f, 15.0f);

  ASSERT_TRUE(observation.valid);
  EXPECT_LE(std::fabs(observation.mean_dbsm - (-3.0f)), 3.0f);
}

TEST(RirRcsFeatureExtractorTest, InsufficientAspectCoverageInvalidatesDimension) {
  // 多样本网格方位跨距 5° < 15° 下限。
  std::vector<RirAspectRcsSample> samples = {
      {-30.0f, 5.0f, -3.0f},
      {-28.0f, 5.0f, -2.5f},
      {-25.0f, 5.0f, -4.0f},
  };

  const recognition::RirRcsObservation observation =
      recognition::RirRcsFeatureExtractor::Extract(samples, -30.0f, 5.0f, 20.0f, 15.0f);

  EXPECT_FALSE(observation.valid);
}

TEST(RirMotionFeatureExtractorTest, ExtractsSpeedAndStraightFlightFromConfirmedTrack) {
  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 1800.0f;
  snapshot.acceleration_mps2 = 12.0f;
  snapshot.velocity.x() = 1800.0f;
  snapshot.velocity.y() = 0.0f;
  snapshot.velocity.z() = 0.0f;
  snapshot.acceleration.x() = 12.0f;
  snapshot.acceleration.y() = 0.0f;
  snapshot.acceleration.z() = 0.0f;
  snapshot.position.z() = 20000.0f;
  // 速度已收敛（单轴方差远小于门限）→ 加速度维照常输出。
  snapshot.gaussian_state.covariance = RirStateCovariance::Identity();

  const recognition::RirMotionObservation observation =
      recognition::RirMotionFeatureExtractor::Extract(snapshot, 30000.0f, 2500.0f);

  ASSERT_TRUE(observation.valid);
  EXPECT_FLOAT_EQ(observation.speed_mps, 1800.0f);
  EXPECT_FLOAT_EQ(observation.altitude_m, 50000.0f);
  EXPECT_FLOAT_EQ(observation.acceleration_mps2, 12.0f);
  EXPECT_TRUE(observation.is_straight);
  EXPECT_GT(observation.quality, 0.0f);
}

/// @brief 来源阶段标志（2026-10-22）：速度/加速度取自"滤波前"还是"滤波后"决定验收行
///        标注（`速度来源=滤波前|滤波后`）。标志必须跟随实际取值通路，否则日志会把滤波前
///        统计值标成滤波后验；拟合加速度越界不入特征时，标志必须回落到滤波后。
TEST(RirMotionFeatureExtractorTest, MarksPreFilterSourceFlagsWithTheValues) {
  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 1800.0f;
  snapshot.velocity.x() = 1800.0f;
  snapshot.acceleration_mps2 = 12.0f;
  snapshot.acceleration.x() = 12.0f;
  snapshot.gaussian_state.covariance = RirStateCovariance::Identity();

  // 1) 只给滤波后验：两个标志都是"滤波后"。
  const recognition::RirMotionObservation by_filter =
      recognition::RirMotionFeatureExtractor::Extract(snapshot, 0.0f, 0.0f);
  EXPECT_FALSE(by_filter.speed_from_fit);
  EXPECT_FALSE(by_filter.acceleration_from_fit);

  // 2) 给出滤波前统计值：速度取统计值、加速度取趋势斜率，标志同步为"滤波前"。
  const Eigen::Vector3f velocity_pre_filter(4200.0f, 0.0f, 0.0f);
  const Eigen::Vector3f acceleration_pre_filter(0.0f, 5.0f, 0.0f);
  const recognition::RirMotionObservation by_pre_filter =
      recognition::RirMotionFeatureExtractor::Extract(snapshot, 0.0f, 0.0f, true,
                                                     velocity_pre_filter, true,
                                                     acceleration_pre_filter);
  EXPECT_TRUE(by_pre_filter.speed_from_fit);
  EXPECT_FLOAT_EQ(by_pre_filter.speed_mps, 4200.0f);
  EXPECT_TRUE(by_pre_filter.acceleration_from_fit);
  EXPECT_FLOAT_EQ(by_pre_filter.acceleration_mps2, 5.0f);

  // 3) 滤波前加速度量级越界（>50 m/s²）不入特征：标志回落到"滤波后"。
  const Eigen::Vector3f out_of_range(0.0f, 80.0f, 0.0f);
  const recognition::RirMotionObservation by_range =
      recognition::RirMotionFeatureExtractor::Extract(snapshot, 0.0f, 0.0f, true,
                                                     velocity_pre_filter, true, out_of_range);
  EXPECT_TRUE(by_range.speed_from_fit);
  EXPECT_FALSE(by_range.acceleration_from_fit);
}

/// @brief 速度未收敛时加速度维置零（2026-10-16）：滤波重置后首个后验速度会跳到真值
///        量级，而差分基准是重置前的旧速度（≈0），使 a=Δv/Δt 数值上约等于速度本身
///        （集成实测出现过加速度与速度逐分量相同、模长 5 640 m/s²，真实值仅约 5 m/s²）。
///        该伪值经指数平滑仍会污染整窗中位数、令运动维对正确型号相似度恒为 0，
///        故在识别侧按速度方差门控：置零后该维对所有型号给出同一相似度（不再反向拖累）。
TEST(RirMotionFeatureExtractorTest, GatesAccelerationWhileVelocityUnconverged) {
  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 5200.0f;
  snapshot.acceleration_mps2 = 5140.0f;  // 伪值：与速度同量级
  snapshot.velocity.x() = 5200.0f;
  snapshot.acceleration.x() = 5140.0f;
  snapshot.position.z() = 1200000.0f;
  // 只在速度分块注入重置后的无知先验；位置分块保持收敛（否则运动质量因子也会被压低）。
  snapshot.gaussian_state.covariance = RirStateCovariance::Identity() * 100.0f;
  snapshot.gaussian_state.covariance(1, 1) = 9.0e6f;
  snapshot.gaussian_state.covariance(3, 3) = 9.0e6f;
  snapshot.gaussian_state.covariance(5, 5) = 9.0e6f;

  const recognition::RirMotionObservation observation =
      recognition::RirMotionFeatureExtractor::Extract(snapshot, 300.0f, 300.0f);

  ASSERT_TRUE(observation.valid);
  // 速度与高度来自位置滤波、协方差已收敛，照常保留。
  EXPECT_FLOAT_EQ(observation.speed_mps, 5200.0f);
  EXPECT_FLOAT_EQ(observation.altitude_m, 1200300.0f);
  // 加速度被门控置零。
  EXPECT_FLOAT_EQ(observation.acceleration_mps2, 0.0f);
}

/// @brief 协方差未设置（全零）时加速度维同样不输出：零方差不是"已收敛"的证据，
///        而是"未知"，按不可信处理避免把未经确认的差分当物理量送给识别。
TEST(RirMotionFeatureExtractorTest, GatesAccelerationWhenCovarianceUnknown) {
  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 1000.0f;
  snapshot.acceleration_mps2 = 3000.0f;
  snapshot.acceleration.x() = 3000.0f;
  snapshot.position.z() = 100000.0f;
  snapshot.gaussian_state.covariance = RirStateCovariance::Zero();

  const recognition::RirMotionObservation observation =
      recognition::RirMotionFeatureExtractor::Extract(snapshot, 0.0f, 0.0f);

  ASSERT_TRUE(observation.valid);
  EXPECT_FLOAT_EQ(observation.speed_mps, 1000.0f);
  EXPECT_FLOAT_EQ(observation.acceleration_mps2, 0.0f);
}

TEST(RirMotionFeatureExtractorTest, UnconfirmedTrackIsInvalid) {
  RirTrackState snapshot;  // kTentative
  const recognition::RirMotionObservation observation =
      recognition::RirMotionFeatureExtractor::Extract(snapshot, 0.0f, 0.0f);
  EXPECT_FALSE(observation.valid);
}

/// @brief 质量因子按三轴均值归一化：输入迹为三轴之和，参考方差 (100 m)² 是每轴
///        口径——迹 30000（每轴均值 10000 = (100 m)²）应得质量 0.5。
TEST(RirMotionFeatureExtractorTest, QualityNormalizesTracePerAxis) {
  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 250.0f;
  snapshot.velocity.x() = 250.0f;

  const recognition::RirMotionObservation observation =
      recognition::RirMotionFeatureExtractor::Extract(snapshot, 0.0f, 30000.0f);
  ASSERT_TRUE(observation.valid);
  EXPECT_FLOAT_EQ(observation.quality, 0.5f);
}

TEST(RirPolarizationFeatureExtractorTest, MissingChannelInvalidatesDimension) {
  // 任一通道 RCS 缺失（样本列表为空）→ 维度无效。
  std::vector<RirPolSMatrixSample> samples;
  const recognition::RirPolarizationObservation observation =
      recognition::RirPolarizationFeatureExtractor::Extract(samples, -30.0f, 5.0f, 20.0f,
                                                            100000.0f);
  EXPECT_FALSE(observation.valid);
}

TEST(RirPolarizationFeatureExtractorTest, ChannelEnergyDifferenceMatchesRcsRatio) {
  RirPolSMatrixSample sample;
  sample.aspect_az_deg = -30.0f;
  sample.aspect_el_deg = 5.0f;
  sample.hh_amp_db = -3.0f;
  sample.vv_amp_db = -6.0f;
  std::vector<RirPolSMatrixSample> samples = {sample};

  const recognition::RirPolarizationObservation observation =
      recognition::RirPolarizationFeatureExtractor::Extract(samples, -30.0f, 5.0f, 20.0f,
                                                            100000.0f);

  ASSERT_TRUE(observation.valid);
  EXPECT_NEAR(observation.energy_difference_db, 3.0f, 1.5f);
}

TEST(RirRangeProfileFeatureExtractorTest, ScatterersProduceLengthAndPeakCount) {
  std::vector<RirRangeRcsScatterer> scatterers;
  const float offsets[] = {-4.0f, -1.0f, 0.0f, 2.0f, 5.0f};
  const float rcs[] = {0.0f, -3.0f, -6.0f, -3.0f, 0.0f};
  for (int i = 0; i < 5; ++i) {
    RirRangeRcsScatterer scatterer;
    scatterer.range_offset_m = offsets[i];
    scatterer.rcs_dbsm = rcs[i];
    scatterers.push_back(scatterer);
  }

  const recognition::RirRangeProfileObservation observation =
      recognition::RirRangeProfileFeatureExtractor::Extract(scatterers, 3.0e6f, 20.0f, 0.0f);

  ASSERT_TRUE(observation.valid);
  EXPECT_NEAR(observation.resolution_m, 50.0f, 0.1f);  // c/(2B) = 2.9979e8/(2·3e6) ≈ 49.97
  EXPECT_NEAR(observation.length_m, 9.0f, 50.0f);      // 容差等于距离分辨率
  EXPECT_EQ(observation.peak_count, 5U);
}

TEST(RirRangeProfileFeatureExtractorTest, ResolutionCoarserThanLimitInvalidatesDimension) {
  std::vector<RirRangeRcsScatterer> scatterers;
  RirRangeRcsScatterer scatterer;
  scatterer.range_offset_m = 0.0f;
  scatterer.rcs_dbsm = 0.0f;
  scatterers.push_back(scatterer);

  // 分辨率 50 m > 允许上限 10 m → 维度无效。
  const recognition::RirRangeProfileObservation observation =
      recognition::RirRangeProfileFeatureExtractor::Extract(scatterers, 3.0e6f, 20.0f, 10.0f);

  EXPECT_FALSE(observation.valid);
}

TEST(RecognitionFeatureExtractorTest, InvalidInputsReturnZeroQualityWithoutThrowing) {
  // 各提取器对无效输入返回 quality=0、valid=false，不抛异常。
  const recognition::RirRcsObservation rcs =
      recognition::RirRcsFeatureExtractor::Extract({}, -30.0f, 5.0f, 20.0f, 15.0f);
  EXPECT_EQ(rcs.quality, 0.0f);
  EXPECT_FALSE(rcs.valid);

  RirTrackState snapshot;
  const recognition::RirMotionObservation motion =
      recognition::RirMotionFeatureExtractor::Extract(snapshot, 0.0f, 0.0f);
  EXPECT_EQ(motion.quality, 0.0f);
  EXPECT_FALSE(motion.valid);

  const recognition::RirPolarizationObservation polarization =
      recognition::RirPolarizationFeatureExtractor::Extract({}, 0.0f, 0.0f, 20.0f, 100000.0f);
  EXPECT_EQ(polarization.quality, 0.0f);
  EXPECT_FALSE(polarization.valid);

  const recognition::RirRangeProfileObservation range_profile =
      recognition::RirRangeProfileFeatureExtractor::Extract({}, 3.0e6f, 20.0f, 0.0f);
  EXPECT_EQ(range_profile.quality, 0.0f);
  EXPECT_FALSE(range_profile.valid);
}

TEST(RirObservationBuilderTest, ConfirmedTrackWithAllFeaturesProducesFullMask) {
  RirSceneTarget target;
  target.aspect_rcs_samples.push_back({-30.0f, 5.0f, -3.0f});
  RirPolSMatrixSample polarization;
  polarization.aspect_az_deg = -30.0f;
  polarization.aspect_el_deg = 5.0f;
  polarization.hh_amp_db = -3.0f;
  polarization.vv_amp_db = -6.0f;
  target.polarization_samples.push_back(polarization);
  target.range_rcs_scatterers.push_back({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});

  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 1800.0f;
  snapshot.velocity.x() = 1800.0f;

  const recognition::RirFeatureSet set =
      recognition::RirObservationBuilder::Build(target, snapshot, MakeContext());

  EXPECT_EQ(
      set.valid_feature_mask & static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kRcs),
      static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kRcs));
  EXPECT_EQ(
      set.valid_feature_mask & static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kMotion),
      static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kMotion));
  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kPolarization),
            static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kPolarization));
  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kRangeProfile),
            static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kRangeProfile));
}

// -- 场景 4-7：低 SNR / 低带宽 / 强干扰 / 短驻留门控 -----------------------
// （原 TrackStateSnapshotEmitterTest 属 AR 航迹快照发射器行为，随解耦回归 AR，
//   不由本模块承担；RIR 侧运动特征不确定性由内部航迹协方差供给。）

TEST(RecognitionScenarioGateTest, LowSnrExcludesRcsAndPolarizationButKeepsMotion) {
  RirSceneTarget target;
  target.aspect_rcs_samples.push_back({0.0f, 20.0f, -3.0f});
  RirPolSMatrixSample polarization;
  polarization.aspect_az_deg = 0.0f;
  polarization.aspect_el_deg = 20.0f;
  polarization.hh_amp_db = -3.0f;
  polarization.vv_amp_db = -5.0f;
  target.polarization_samples.push_back(polarization);

  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 100.0f;
  snapshot.velocity.x() = 100.0f;

  recognition::RirObservationContext context = MakeContext();
  context.snr_db = 3.0f;  // 低于 6 dB 门限
  context.look_az_deg = 0.0f;
  context.look_el_deg = 20.0f;

  const recognition::RirFeatureSet set =
      recognition::RirObservationBuilder::Build(target, snapshot, context);

  EXPECT_EQ(
      set.valid_feature_mask & static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kRcs), 0U);
  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kPolarization),
            0U);
  EXPECT_EQ(
      set.valid_feature_mask & static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kMotion),
      static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kMotion));
}

TEST(RecognitionScenarioGateTest, LowBandwidthExcludesRangeProfile) {
  RirSceneTarget target;
  target.range_rcs_scatterers.push_back({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 100.0f;
  snapshot.velocity.x() = 100.0f;

  recognition::RirObservationContext context = MakeContext();
  context.bandwidth_hz = 1.0e6f;  // 分辨率 150 m > 上限 50 m
  context.max_range_resolution_m = 50.0f;
  context.look_az_deg = 0.0f;
  context.look_el_deg = 20.0f;

  const recognition::RirFeatureSet set =
      recognition::RirObservationBuilder::Build(target, snapshot, context);

  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kRangeProfile),
            0U);
}

TEST(RecognitionScenarioGateTest, StrongJammingExcludesPolarization) {
  RirSceneTarget target;
  RirPolSMatrixSample polarization;
  polarization.aspect_az_deg = 0.0f;
  polarization.aspect_el_deg = 20.0f;
  polarization.hh_amp_db = -3.0f;
  polarization.vv_amp_db = -5.0f;
  target.polarization_samples.push_back(polarization);
  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 100.0f;
  snapshot.velocity.x() = 100.0f;

  recognition::RirObservationContext context = MakeContext();
  // 强干扰（jnr > 20 dB）：有效 SNR 压到门限以下 → 极化维度不可用。
  context.snr_db = 20.0f - 25.0f;
  context.look_az_deg = 0.0f;
  context.look_el_deg = 20.0f;

  const recognition::RirFeatureSet set =
      recognition::RirObservationBuilder::Build(target, snapshot, context);

  EXPECT_EQ(set.valid_feature_mask &
                static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kPolarization),
            0U);
}

TEST(RecognitionScenarioGateTest, ShortDwellLowersQualityAndSlowsObservationGrowth) {
  RirSceneTarget target;
  target.aspect_rcs_samples.push_back({0.0f, 20.0f, -3.0f});
  RirTrackState snapshot;
  snapshot.status = RirTrackStatus::kConfirmed;
  snapshot.speed = 100.0f;
  snapshot.velocity.x() = 100.0f;

  recognition::RirObservationContext nominal = MakeContext();
  nominal.snr_db = 6.5f;
  nominal.dwell_sec = 0.05f;
  nominal.look_az_deg = 0.0f;
  nominal.look_el_deg = 20.0f;
  const recognition::RirFeatureSet set_nominal =
      recognition::RirObservationBuilder::Build(target, snapshot, nominal);

  recognition::RirObservationContext short_dwell = MakeContext();
  short_dwell.snr_db = 6.5f;
  short_dwell.dwell_sec = 0.01f;
  short_dwell.look_az_deg = 0.0f;
  short_dwell.look_el_deg = 20.0f;
  const recognition::RirFeatureSet set_short =
      recognition::RirObservationBuilder::Build(target, snapshot, short_dwell);

  // 短驻留：质量因子下降。
  ASSERT_TRUE(set_nominal.rcs.valid);
  ASSERT_TRUE(set_short.rcs.valid);
  EXPECT_LT(set_short.rcs.quality, set_nominal.rcs.quality);
  // 短驻留 + 低 SNR：质量低于观测下限 → tracker 不计为观测（增长速率为 0）。
  recognition::RirTracker tracker;
  recognition::RirTracker::Options options;
  options.min_confirmed_hits = 1U;
  options.min_observation_count = 1U;
  options.accumulation_window_sec = 10.0f;
  options.acceptance_score = 0.6f;
  options.minimum_margin = 0.05f;
  options.result_hold_sec = 10.0f;
  options.max_range_m = 1.0e6f;
  tracker.SetOptions(options);

  tracking::RirTrackSnapshotList track_list;
  tracking::RirTrackState track;
  track.association_key = 1U;
  track.status = RirTrackStatus::kConfirmed;
  track.hit_count = 3U;
  track.speed = 100.0f;
  track.velocity.x() = 100.0f;
  track.gaussian_state.covariance =
      tracking::RirStateCovariance::Identity() * (1.0e6f / 3.0f);  // 大不确定度：运动质量也低于下限
  track_list.push_back(track);
  recognition::RirFeatureDatabase database;  // 空库：仅验证计数行为
  std::unordered_map<std::uint64_t, recognition::RirTracker::TrackObservationInput> observations;
  recognition::RirTracker::TrackObservationInput input;
  input.target = &target;
  input.context = short_dwell;
  observations[1U] = input;
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    tracker.UpdateCycle(track_list, observations, database, {}, static_cast<float>(cycle), cycle,
                        1U);
  }
  const session::RirRecognitionResult* result = tracker.FindResult(1U);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->observation_count, 0U);  // 短驻留 + 低质量：不计为观测

  // 标称驻留对照：同样低 SNR 下质量高于下限 → 观测正常积累。
  recognition::RirTracker nominal_tracker;
  nominal_tracker.SetOptions(options);
  input.context = nominal;
  std::unordered_map<std::uint64_t, recognition::RirTracker::TrackObservationInput>
      nominal_observations;
  nominal_observations[1U] = input;
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    nominal_tracker.UpdateCycle(track_list, nominal_observations, database, {},
                                static_cast<float>(cycle), cycle, 1U);
  }
  const session::RirRecognitionResult* nominal_result = nominal_tracker.FindResult(1U);
  ASSERT_NE(nominal_result, nullptr);
  EXPECT_EQ(nominal_result->observation_count, 3U);  // 标称驻留：增长速率正常
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
