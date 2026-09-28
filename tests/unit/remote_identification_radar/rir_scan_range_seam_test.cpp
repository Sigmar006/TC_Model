// Copyright 2026. All Rights Reserved.
//
// @file rir_scan_range_seam_test.cpp
// @brief 2026-09-28 审计 Q-2 的针对性测试：波位表方位跨 ±180° 缝合线时，
//        第48项「方位扫描范围」必须给出真实连续扇区，而不是规范化最值的
//        `-179.600/179.600`（那读起来像全向扫描）。
//
// 覆盖：
//   1) 本场景形态（中心 −110°、扫描窗 ±92° → 扇区 184°、跨缝）；
//   2) 不跨缝扇区（含 -180° 下界的边界）；
//   3) 单波位退化（半宽 0，不编造跨度）；
//   4) 非有限方位角被忽略、空表被拒绝。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "1q/electromagnetics/RfScene.h"
#include "remote_identification_radar/runtime/RirAcceptanceRecords.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using runtime::TryResolveAzimuthSpanDeg;

struct AzimuthSpan {
  double center_deg{0.0};
  double half_width_deg{0.0};
  double min_deg{0.0};
  double max_deg{0.0};
};

AzimuthSpan ResolveSpan(const std::vector<float>& azimuths_deg) {
  AzimuthSpan span;
  EXPECT_TRUE(TryResolveAzimuthSpanDeg(azimuths_deg, &span.center_deg, &span.half_width_deg,
                                       &span.min_deg, &span.max_deg));
  return span;
}

/// @brief 本场景形态：中心 −110°、相对扇区 ±92° → 归一化 ±179.6°，真实跨度 184°。
TEST(RirScanRangeSeamTest, SeamCrossingSectorReportsContinuousSpan) {
  // 与 Rir_scan_pattern.csv 同构：相对波位 −92…92（步长 0.8°）叠加中心 −110° 后
  // 经 NormalizeAzimuthDeg 折到 [-180, 180]。
  std::vector<float> normalized;
  const double center_deg = -110.0;
  for (int index = -115; index <= 115; ++index) {
    const double relative_deg = 0.8 * static_cast<double>(index);
    double absolute_deg = center_deg + relative_deg;
    while (absolute_deg < -180.0) {
      absolute_deg += 360.0;
    }
    while (absolute_deg > 180.0) {
      absolute_deg -= 360.0;
    }
    normalized.push_back(static_cast<float>(absolute_deg));
  }

  const AzimuthSpan span = ResolveSpan(normalized);
  // 真实扇区 = 配置指向 −110° ± 92°，按规范化中心展开为 −202°/−18°。
  EXPECT_NEAR(span.min_deg, -202.0, 1.0e-3);
  EXPECT_NEAR(span.max_deg, -18.0, 1.0e-3);
  EXPECT_NEAR(span.half_width_deg, 92.0, 1.0e-3);
  EXPECT_NEAR(span.center_deg, -110.0, 1.0e-3);
  // 连续跨度必须等于 2×半宽，且不能退化成"规范化最值之差"（后者恒 ≤ 360° 但无意义）。
  EXPECT_NEAR(span.max_deg - span.min_deg, 2.0 * span.half_width_deg, 1.0e-6);
  EXPECT_GT(span.max_deg - span.min_deg, 180.0) << "跨缝扇区跨度大于 180° 才是真实形态";

  // 规范化最值确实是 ±179.6°（旧实现打印出来的就是这个），证明本用例命中的是跨缝输入。
  const float normalized_min = *std::min_element(normalized.begin(), normalized.end());
  const float normalized_max = *std::max_element(normalized.begin(), normalized.end());
  EXPECT_NEAR(static_cast<double>(normalized_min), -179.6, 1.0e-3);
  EXPECT_NEAR(static_cast<double>(normalized_max), 179.6, 1.0e-3);
}

/// @brief 不跨缝扇区：输出与直接取最值一致，中心即扇区中点。
TEST(RirScanRangeSeamTest, NonCrossingSectorMatchesDirectExtremes) {
  const std::vector<float> azimuths_deg = {26.0f, 30.0f, 34.0f, 38.0f};
  const AzimuthSpan span = ResolveSpan(azimuths_deg);
  EXPECT_NEAR(span.center_deg, 32.0, 1.0e-9);
  EXPECT_NEAR(span.half_width_deg, 6.0, 1.0e-9);
  EXPECT_NEAR(span.min_deg, 26.0, 1.0e-9);
  EXPECT_NEAR(span.max_deg, 38.0, 1.0e-9);
}

/// @brief 靠近 +180° 但不跨缝的扇区：按常规最值/中点给出，不得被误判为跨缝。
TEST(RirScanRangeSeamTest, SectorNearPlus180DoesNotWrap) {
  const std::vector<float> azimuths_deg = {160.0f, 165.0f, 170.0f, 175.0f};
  const AzimuthSpan span = ResolveSpan(azimuths_deg);
  EXPECT_NEAR(span.min_deg, 160.0, 1.0e-9);
  EXPECT_NEAR(span.max_deg, 175.0, 1.0e-9);
  EXPECT_NEAR(span.center_deg, 167.5, 1.0e-9);
  EXPECT_NEAR(span.half_width_deg, 7.5, 1.0e-9);
}

/// @brief 跨缝判定基于"最大角间隙 = 缝"：细密波位（本模型扫描生成器的形态，
///        步长远小于扇区宽度）下缝即 ±180° 接缝，区间以规范化中心展开。
TEST(RirScanRangeSeamTest, DenseWavesNearPlus180UnwrapToRealSector) {
  // 0.4° 步长、覆盖 175°~185.4° 的连续扇区（归一化后分成 175…179.8 与
  // -179.8…-174.6 两段，两段之间的接缝间隙为全场最大）。
  std::vector<float> azimuths_deg;
  for (int index = 0; index <= 26; ++index) {
    double absolute_deg = 175.0 + 0.4 * static_cast<double>(index);
    while (absolute_deg > 180.0) {
      absolute_deg -= 360.0;
    }
    azimuths_deg.push_back(static_cast<float>(absolute_deg));
  }
  const AzimuthSpan span = ResolveSpan(azimuths_deg);
  // 中心规范化为 −179.8°，因此等价连续区间按该中心表示为 −185°/−174.6°。
  EXPECT_NEAR(span.min_deg, -185.0, 1.0e-3);
  EXPECT_NEAR(span.max_deg, -174.6, 1.0e-3);
  EXPECT_NEAR(span.half_width_deg, 5.2, 1.0e-3);
  EXPECT_NEAR(span.max_deg - span.min_deg, 2.0 * span.half_width_deg, 1.0e-6);
  EXPECT_NEAR(span.center_deg, -179.8, 1.0e-3);
}

/// @brief 稀疏跨缝扇区：判据由"最大角间隙"决定，与点密度无关——同样落到
///        等价的 −185°/−174.6° 连续区间，只是端点分辨率更粗。
TEST(RirScanRangeSeamTest, SparseSamplingPicksLargestGapInterpretation) {
  // 同一条 175°~185.4° 扇区，仅 14 个点（0.8° 步长）：归一化后成两个簇，
  // 簇间 354.4° 仍是全场最大间隙，故补集仍是同一条连续扇区。
  std::vector<float> azimuths_deg;
  for (int index = 0; index <= 13; ++index) {
    double absolute_deg = 175.0 + 0.8 * static_cast<double>(index);
    while (absolute_deg > 180.0) {
      absolute_deg -= 360.0;
    }
    azimuths_deg.push_back(static_cast<float>(absolute_deg));
  }
  const AzimuthSpan span = ResolveSpan(azimuths_deg);
  EXPECT_NEAR(span.min_deg, -185.0, 1.0e-3);
  EXPECT_NEAR(span.max_deg, -174.6, 1.0e-3);
  EXPECT_NEAR(span.half_width_deg, 5.2, 1.0e-3);
  EXPECT_NEAR(span.max_deg - span.min_deg, 2.0 * span.half_width_deg, 1.0e-6);
}

/// @brief 单波位退化：半宽 0，min == max == 中心（不编造 360° 跨度）。
TEST(RirScanRangeSeamTest, SingleWaveDegeneratesToPoint) {
  const AzimuthSpan span = ResolveSpan({-42.5f});
  EXPECT_NEAR(span.center_deg, -42.5, 1.0e-9);
  EXPECT_DOUBLE_EQ(span.half_width_deg, 0.0);
  EXPECT_DOUBLE_EQ(span.min_deg, span.max_deg);
}

/// @brief 非有限方位角跳过；全部非有限或空表时拒绝且不改输出。
TEST(RirScanRangeSeamTest, NonFiniteAzimuthsAreIgnoredOrRejected) {
  const double nan_value = std::numeric_limits<double>::quiet_NaN();
  const AzimuthSpan span =
      ResolveSpan({-10.0f, static_cast<float>(nan_value), 10.0f, static_cast<float>(nan_value)});
  EXPECT_NEAR(span.center_deg, 0.0, 1.0e-9);
  EXPECT_NEAR(span.half_width_deg, 10.0, 1.0e-9);

  AzimuthSpan untouched;
  untouched.center_deg = 123.0;
  EXPECT_FALSE(TryResolveAzimuthSpanDeg({}, &untouched.center_deg, &untouched.half_width_deg,
                                        &untouched.min_deg, &untouched.max_deg));
  EXPECT_DOUBLE_EQ(untouched.center_deg, 123.0) << "拒绝时必须不改输出";
  EXPECT_FALSE(TryResolveAzimuthSpanDeg({static_cast<float>(nan_value)}, &untouched.center_deg,
                                        &untouched.half_width_deg, &untouched.min_deg,
                                        &untouched.max_deg));
  // 空指针拒绝。
  std::vector<float> azimuths = {1.0f};
  EXPECT_FALSE(TryResolveAzimuthSpanDeg(azimuths, nullptr, nullptr, nullptr, nullptr));
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
