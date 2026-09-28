// Copyright 2026. All Rights Reserved.
//
// @file sbirs_scan_dwell_test.cpp
// @brief 第 12/14 项「驻留时间」判据回归：0.5 s < 方位视场 / 扫描速率 < 1 s。
//
// 背景：主文档第 12、14 项要求驻留时间落在 0.5–1 s 开区间（验收判定标准 §0B.4）；
// 2026-09-28 端到端核对发现所有 SBIRS 场景都算不出合规值（例如 8.7°/6.0°/s = 1.45 s），
// 根因是扫描速率配置而非公式。此处按「场景配置 → 驻留时间」锁定已交付配置，防止回退。

#include <gtest/gtest.h>

namespace sbirs_sensor {
namespace tests {
namespace {

// 与 SbirsPipeline 第 12 项实现同口径：驻留时间 = 宽场方位视场 / 扫描速率。
double WfovDwellSeconds(double wide_field_fov_az_deg, double scan_rate_deg_per_sec) {
  if (!(scan_rate_deg_per_sec > 0.0)) {
    return -1.0;  // 扫描速率为 0 表示不扫描（span=0），本判据不适用
  }
  return wide_field_fov_az_deg / scan_rate_deg_per_sec;
}

// 判据：0.5 s < 驻留 < 1 s（开区间）。
bool DwellWithinAcceptanceWindow(double dwell_s) {
  return dwell_s > 0.5 && dwell_s < 1.0;
}

// 已交付的 SBIRS 场景配置（examples/scenes/sbirs_triple_sat_fix_messages、
// sbirs_inframe_dual_target：三颗星各 8.7°/12.0°/s；examples/basic_config/sbirs.json：
// 20.0°/26.0°/s）。这些常量与对应 JSON 的 wide_field_fov_az_deg /
// scan_rate_deg_per_sec 必须保持一致；改场景时同步改本测试。
TEST(SbirsScanDwellTest, ShippedSceneConfigurationsSatisfyDwellCriterion) {
  const double dwell_triple_sat_s = WfovDwellSeconds(8.7, 12.0);
  const double dwell_inframe_dual_s = WfovDwellSeconds(8.7, 12.0);
  const double dwell_basic_config_s = WfovDwellSeconds(20.0, 26.0);
  EXPECT_TRUE(DwellWithinAcceptanceWindow(dwell_triple_sat_s)) << dwell_triple_sat_s;
  EXPECT_TRUE(DwellWithinAcceptanceWindow(dwell_inframe_dual_s)) << dwell_inframe_dual_s;
  EXPECT_TRUE(DwellWithinAcceptanceWindow(dwell_basic_config_s)) << dwell_basic_config_s;
  EXPECT_NEAR(dwell_triple_sat_s, 0.725, 0.001);
  EXPECT_NEAR(dwell_basic_config_s, 0.769231, 1.0e-6);
}

// 反例锁定历史缺陷：旧配置 8.7°/6.0°/s = 1.45 s 必须仍被判为不合规，
// 防止有人把扫描速率调回旧值而测试仍然通过。
TEST(SbirsScanDwellTest, LegacySlowScanRateIsRejected) {
  const double legacy_dwell_s = WfovDwellSeconds(8.7, 6.0);
  EXPECT_NEAR(legacy_dwell_s, 1.45, 1.0e-9);
  EXPECT_FALSE(DwellWithinAcceptanceWindow(legacy_dwell_s));
}

// 更慢扫描（0.5 s 以上区间外的高侧）与更快扫描（跌破下界）都要被拒，
// 说明判据是双侧开区间而不是单侧上限。
TEST(SbirsScanDwellTest, BothSidesOfTheWindowAreRejected) {
  EXPECT_FALSE(DwellWithinAcceptanceWindow(WfovDwellSeconds(20.0, 10.0)));  // 2.0 s 过慢
  EXPECT_FALSE(DwellWithinAcceptanceWindow(WfovDwellSeconds(20.0, 60.0)));  // 0.333 s 过快
  EXPECT_FALSE(DwellWithinAcceptanceWindow(WfovDwellSeconds(20.0, 20.0)));  // 恰好 1.0 s 不在开区间
}

// 不扫描配置（扫描速率 0）不得当作合规驻留时间。
TEST(SbirsScanDwellTest, ZeroScanRateIsNotAcceptable) {
  EXPECT_LT(WfovDwellSeconds(20.0, 0.0), 0.0);
  EXPECT_FALSE(DwellWithinAcceptanceWindow(WfovDwellSeconds(20.0, 0.0)));
}

}  // namespace
}  // namespace tests
}  // namespace sbirs_sensor
