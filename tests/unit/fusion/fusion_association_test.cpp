// Copyright 2026. All Rights Reserved.
//
// @file fusion_association_test.cpp
// @brief 验证关联分层：身份键直挂、位置半径、方位相干、特征门限与失跟删除。
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/fusion/fusion.hpp"

namespace fusion {
namespace {

using oneq::coordinate::EcefPositionM;
using oneq::coordinate::EnuPositionM;
using oneq::coordinate::LlaPositionDegM;

constexpr std::uint64_t kSyntheticKeyBase = (1ULL << 63);

LlaPositionDegM MakeLla(double latitude_deg, double longitude_deg, double altitude_m) {
  LlaPositionDegM lla;
  lla.latitude_deg = latitude_deg;
  lla.longitude_deg = longitude_deg;
  lla.altitude_m = altitude_m;
  return lla;
}

// 以 origin 为参考点，把 ENU 偏移换算为 LLA。
LlaPositionDegM OffsetLla(const LlaPositionDegM& origin, double east_m, double north_m) {
  EnuPositionM enu{east_m, north_m, 0.0};
  EcefPositionM ecef{};
  LlaPositionDegM lla{};
  EXPECT_TRUE(oneq::coordinate::TryEnuToEcef(enu, origin, &ecef));
  EXPECT_TRUE(oneq::coordinate::TryEcefToLla(ecef, &lla));
  return lla;
}

DetectionRecord MakePositionDetection(std::uint64_t key, std::uint32_t source_id,
                                      const LlaPositionDegM& position) {
  DetectionRecord detection;
  detection.key = key;
  detection.source_id = source_id;
  detection.has_position = true;
  detection.position = position;
  detection.verdict = 1.0;
  detection.quality = 1.0;
  return detection;
}

DetectionRecord MakeBearingDetection(std::uint64_t key, std::uint32_t source_id,
                                     double az_deg, double el_deg) {
  DetectionRecord detection;
  detection.key = key;
  detection.source_id = source_id;
  detection.has_bearing = true;
  detection.bearing_az_deg = az_deg;
  detection.bearing_el_deg = el_deg;
  detection.verdict = 1.0;
  detection.quality = 1.0;
  return detection;
}

TEST(FusionAssociationTest, SameKeyMergesAcrossSources) {
  const LlaPositionDegM a = MakeLla(30.0, 120.0, 0.0);
  FusionEngine engine(FusionConfig{});
  const auto targets = engine.Update(
      {MakePositionDetection(7U, 0U, a), MakePositionDetection(7U, 1U, a)}, 1U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_EQ(targets[0].key, 7U);
  EXPECT_EQ(targets[0].channels.size(), 2U);
  EXPECT_DOUBLE_EQ(targets[0].confidence, 2.0);
}

TEST(FusionAssociationTest, IdentifiedDetectionCreatesTrack) {
  const LlaPositionDegM a = MakeLla(30.0, 120.0, 0.0);
  FusionEngine engine(FusionConfig{});
  const auto targets = engine.Update({MakePositionDetection(9U, 0U, a)}, 1U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_EQ(targets[0].key, 9U);
  EXPECT_EQ(targets[0].channels[0].source_id, 0U);
  EXPECT_EQ(targets[0].channels[0].sample_count, 1U);
}

TEST(FusionAssociationTest, SensorNamePropagatesToOutputChannel) {
  DetectionRecord detection = MakePositionDetection(
      10U, 4U, MakeLla(30.0, 120.0, 0.0));
  detection.source_name = "红外预警卫星A";
  FusionEngine engine(FusionConfig{});

  const auto targets = engine.Update({detection}, 1U);

  ASSERT_EQ(targets.size(), 1U);
  ASSERT_EQ(targets[0].channels.size(), 1U);
  EXPECT_EQ(targets[0].channels[0].source_id, 4U);
  EXPECT_EQ(targets[0].channels[0].source_name, "红外预警卫星A");
}

TEST(FusionAssociationTest, UnidentifiedJoinsTrackWithinRadius) {
  const LlaPositionDegM a = MakeLla(30.0, 120.0, 0.0);
  const LlaPositionDegM b = OffsetLla(a, 0.0, 100.0);  // 100 m 北
  FusionConfig config;
  config.position_radius_m = 1000.0;
  FusionEngine engine(config);

  engine.Update({MakePositionDetection(7U, 0U, a)}, 1U);
  const auto targets = engine.Update({MakePositionDetection(0U, 1U, b)}, 2U);

  // 无身份探测并入既有航迹：仍为单目标，置信度跨周期累积。
  ASSERT_EQ(targets.size(), 1U);
  EXPECT_EQ(targets[0].key, 7U);
  EXPECT_DOUBLE_EQ(targets[0].confidence, 2.0);
  EXPECT_EQ(targets[0].channels.size(), 2U);
}

TEST(FusionAssociationTest, UnidentifiedBeyondRadiusCreatesNewTrack) {
  const LlaPositionDegM a = MakeLla(30.0, 120.0, 0.0);
  const LlaPositionDegM c = OffsetLla(a, 0.0, 2000.0);  // 2000 m 北
  FusionConfig config;
  config.position_radius_m = 1000.0;
  FusionEngine engine(config);

  engine.Update({MakePositionDetection(7U, 0U, a)}, 1U);
  const auto targets = engine.Update({MakePositionDetection(0U, 1U, c)}, 2U);

  // 超出空间门限 → 新建无身份航迹（合成键 ≥ 2^63）。
  ASSERT_EQ(targets.size(), 2U);
  EXPECT_EQ(targets[0].key, 7U);
  EXPECT_GE(targets[1].key, kSyntheticKeyBase);
}

TEST(FusionAssociationTest, BearingOnlyCoherentJoins) {
  FusionConfig config;
  config.bearing_beamwidth_deg = 5.0;
  FusionEngine engine(config);

  engine.Update({MakeBearingDetection(7U, 0U, 10.0, 5.0)}, 1U);
  const auto targets =
      engine.Update({MakeBearingDetection(0U, 1U, 11.0, 5.0)}, 2U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_EQ(targets[0].key, 7U);
  EXPECT_DOUBLE_EQ(targets[0].confidence, 2.0);
}

TEST(FusionAssociationTest, BearingOnlyIncoherentSeparates) {
  FusionConfig config;
  config.bearing_beamwidth_deg = 5.0;
  FusionEngine engine(config);

  engine.Update({MakeBearingDetection(7U, 0U, 10.0, 5.0)}, 1U);
  const auto targets =
      engine.Update({MakeBearingDetection(0U, 1U, 30.0, 5.0)}, 2U);

  ASSERT_EQ(targets.size(), 2U);
  EXPECT_EQ(targets[0].key, 7U);
  EXPECT_GE(targets[1].key, kSyntheticKeyBase);
}

TEST(FusionAssociationTest, FeatureGateRejectsSamePosition) {
  const LlaPositionDegM a = MakeLla(30.0, 120.0, 0.0);
  FusionConfig config;
  config.feature_threshold = 10.0;
  FusionEngine engine(config);

  DetectionRecord first = MakePositionDetection(7U, 0U, a);
  first.feature = {0.0, 0.0};
  engine.Update({first}, 1U);
  DetectionRecord second = MakePositionDetection(0U, 1U, a);
  second.feature = {100.0, 100.0};
  const auto targets = engine.Update({second}, 2U);

  // 空间相邻但特征相距远超门限 → 拒配，各自成航迹。
  ASSERT_EQ(targets.size(), 2U);
  EXPECT_EQ(targets[0].key, 7U);
  EXPECT_GE(targets[1].key, kSyntheticKeyBase);
}

TEST(FusionAssociationTest, FeatureGateDisabledJoins) {
  const LlaPositionDegM a = MakeLla(30.0, 120.0, 0.0);
  FusionConfig config;
  config.feature_threshold = 0.0;  // 未启用特征门。
  FusionEngine engine(config);

  DetectionRecord first = MakePositionDetection(7U, 0U, a);
  first.feature = {0.0, 0.0};
  engine.Update({first}, 1U);
  DetectionRecord second = MakePositionDetection(0U, 1U, a);
  second.feature = {100.0, 100.0};
  const auto targets = engine.Update({second}, 2U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_EQ(targets[0].key, 7U);
}

TEST(FusionAssociationTest, FeatureDimensionMismatchDoesNotConstrain) {
  // 异构传感器特征维度不一致：特征门不构成约束（冻结语义）。
  const LlaPositionDegM a = MakeLla(30.0, 120.0, 0.0);
  FusionConfig config;
  config.feature_threshold = 10.0;
  FusionEngine engine(config);

  DetectionRecord first = MakePositionDetection(7U, 0U, a);
  first.feature = {0.0};
  engine.Update({first}, 1U);
  DetectionRecord second = MakePositionDetection(0U, 1U, a);
  second.feature = {100.0, 0.0};  // 维度 2 vs 1。
  const auto targets = engine.Update({second}, 2U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_EQ(targets[0].key, 7U);
}

TEST(FusionAssociationTest, FeatureOnlyRecordAssociatesByFeature) {
  // 无位置无方位的探测：仅按特征门限关联。
  FusionConfig config;
  config.feature_threshold = 10.0;
  FusionEngine engine(config);

  DetectionRecord first = MakePositionDetection(7U, 0U, MakeLla(30.0, 120.0, 0.0));
  first.feature = {5.0, 5.0};
  engine.Update({first}, 1U);
  DetectionRecord second;
  second.key = 0U;
  second.source_id = 1U;
  second.feature = {6.0, 5.0};  // 无位置无方位。
  second.verdict = 1.0;
  second.quality = 1.0;
  const auto targets = engine.Update({second}, 2U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_EQ(targets[0].key, 7U);
  EXPECT_DOUBLE_EQ(targets[0].confidence, 2.0);
}

TEST(FusionAssociationTest, InvalidPositionFallsThroughToBearing) {
  // 位置量测非法（LLA 越界）时降级走方位相干通道，而非必然新建航迹。
  FusionConfig config;
  config.bearing_beamwidth_deg = 5.0;
  FusionEngine engine(config);

  engine.Update({MakeBearingDetection(7U, 0U, 10.0, 5.0)}, 1U);
  DetectionRecord second = MakeBearingDetection(0U, 1U, 11.0, 5.0);
  second.has_position = true;
  second.position = MakeLla(95.0, 120.0, 0.0);  // 纬度越界，位置不可用。
  const auto targets = engine.Update({second}, 2U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_EQ(targets[0].key, 7U);
}

TEST(FusionAssociationTest, SameCycleUnidentifiedBearingsDoNotMerge) {
  // 冻结语义：同周期新建航迹不参与本周期后续关联——同一批次内两个相干
  // 方位探测互为独立航迹（合并依赖身份键或后续周期关联）。
  FusionConfig config;
  config.bearing_beamwidth_deg = 5.0;
  FusionEngine engine(config);

  const auto targets = engine.Update(
      {MakeBearingDetection(0U, 0U, 10.0, 5.0), MakeBearingDetection(0U, 1U, 11.0, 5.0)}, 1U);

  ASSERT_EQ(targets.size(), 2U);
  EXPECT_GE(targets[0].key, kSyntheticKeyBase);
  EXPECT_GE(targets[1].key, kSyntheticKeyBase);
  EXPECT_DOUBLE_EQ(targets[0].confidence, 1.0);
  EXPECT_DOUBLE_EQ(targets[1].confidence, 1.0);
}

TEST(FusionAssociationTest, SameCycleUnidentifiedPositionsDoNotMerge) {
  // 位置通道同批次两探测同样各自成航迹（与方位通道语义一致）。
  const LlaPositionDegM a = MakeLla(30.0, 120.0, 0.0);
  const LlaPositionDegM b = OffsetLla(a, 0.0, 100.0);
  FusionConfig config;
  config.position_radius_m = 1000.0;
  FusionEngine engine(config);

  const auto targets = engine.Update({MakePositionDetection(0U, 0U, a),
                                      MakePositionDetection(0U, 1U, b)},
                                     1U);

  ASSERT_EQ(targets.size(), 2U);
  EXPECT_GE(targets[0].key, kSyntheticKeyBase);
  EXPECT_GE(targets[1].key, kSyntheticKeyBase);
}

TEST(FusionAssociationTest, TrackDropsAfterMaxMissedCycles) {
  const LlaPositionDegM a = MakeLla(30.0, 120.0, 0.0);
  FusionConfig config;
  config.max_missed_cycles = 1U;
  FusionEngine engine(config);

  engine.Update({MakePositionDetection(7U, 0U, a)}, 1U);
  EXPECT_EQ(engine.Update({}, 2U).size(), 1U);  // 失跟 1 周期，仍保留。
  EXPECT_TRUE(engine.Update({}, 3U).empty());   // 失跟超过门限，删除。
}

TEST(FusionAssociationTest, MixedBatchOutputSortedByKey) {
  const LlaPositionDegM a = MakeLla(30.0, 120.0, 0.0);
  FusionEngine engine(FusionConfig{});
  const auto targets = engine.Update({MakePositionDetection(3U, 0U, a),
                                      MakePositionDetection(1U, 0U, a),
                                      MakePositionDetection(2U, 0U, a)},
                                     1U);

  ASSERT_EQ(targets.size(), 3U);
  EXPECT_EQ(targets[0].key, 1U);
  EXPECT_EQ(targets[1].key, 2U);
  EXPECT_EQ(targets[2].key, 3U);
}

TEST(FusionAssociationTest, ResetClearsTrackState) {
  const LlaPositionDegM a = MakeLla(30.0, 120.0, 0.0);
  const LlaPositionDegM b = OffsetLla(a, 0.0, 100.0);
  FusionConfig config;
  config.position_radius_m = 1000.0;
  FusionEngine engine(config);

  engine.Update({MakePositionDetection(7U, 0U, a)}, 1U);
  engine.Reset();
  const auto targets = engine.Update({MakePositionDetection(0U, 1U, b)}, 2U);

  // 重置后无既有航迹：无身份探测自建新航迹（合成键）。
  ASSERT_EQ(targets.size(), 1U);
  EXPECT_GE(targets[0].key, kSyntheticKeyBase);
}

}  // namespace
}  // namespace fusion
