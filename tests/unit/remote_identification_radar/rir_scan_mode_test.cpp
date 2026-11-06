/**
 * @file rir_scan_mode_test.cpp
 * @brief 搜索模式（指定空域 / 地平线）扇区解析单测。
 */

#include <gtest/gtest.h>

#include "remote_identification_radar/internal/RirScanVolume.h"

namespace tests {
namespace remote_identification_radar {
namespace {

// 中译：本文件位于 `tests::remote_identification_radar`，而被测符号在全局
//   `::remote_identification_radar` 下——用别名显式指向，避免 `config::` / `internal::`
//   在 tests 命名空间里找不到（WSL/g++ 首次编译该文件时暴露，2026-11-07）。
namespace rircfg = ::remote_identification_radar::config;
namespace riri = ::remote_identification_radar::internal;

using rircfg::RirAzimuthElevationLimitsDeg;
using rircfg::RirHorizonScanConfig;
using rircfg::RirMissionConfig;
using rircfg::RirSearchMode;
using riri::ResolveHorizonBand;
using riri::ResolveSearchSector;

// 硬件体积：方位相对 ±110°、仰角 0~85°（宿主 LRDR 口径）。
RirAzimuthElevationLimitsDeg MakeVolume() {
  RirAzimuthElevationLimitsDeg volume;
  volume.az_min_deg = -110.0f;
  volume.az_max_deg = 130.0f;
  volume.el_min_deg = 0.0f;
  volume.el_max_deg = 85.0f;
  return volume;
}

// 指定空域模式：仰角仍取任务子窗（既有行为逐位不变）。
TEST(RirScanModeTest, SectorModeKeepsWindowElevation) {
  RirMissionConfig mission;
  mission.search_mode = RirSearchMode::kSectorScan;
  mission.scan_window_deg.az_min_deg = -17.5f;
  mission.scan_window_deg.az_max_deg = 17.5f;
  mission.scan_window_deg.el_min_deg = 5.0f;
  mission.scan_window_deg.el_max_deg = 55.0f;

  const auto sector = ResolveSearchSector(mission, MakeVolume(), 0.8f);
  EXPECT_FLOAT_EQ(sector.az_min_deg, -17.5f);
  EXPECT_FLOAT_EQ(sector.az_max_deg, 17.5f);
  EXPECT_FLOAT_EQ(sector.el_min_deg, 5.0f);
  EXPECT_FLOAT_EQ(sector.el_max_deg, 55.0f);
}

// 地平线模式：方位仍取子窗，仰角改用地平线自适应窄带 [−θ/2, +2°]。
TEST(RirScanModeTest, HorizonModeReplacesElevationWithAdaptiveBand) {
  RirMissionConfig mission;
  mission.search_mode = RirSearchMode::kHorizonScan;
  mission.scan_window_deg.az_min_deg = -30.0f;
  mission.scan_window_deg.az_max_deg = 30.0f;
  mission.scan_window_deg.el_min_deg = 5.0f;   // 应被地平线带覆盖
  mission.scan_window_deg.el_max_deg = 55.0f;

  const auto sector = ResolveSearchSector(mission, MakeVolume(), 0.8f);
  EXPECT_FLOAT_EQ(sector.az_min_deg, -30.0f) << "方位范围仍由任务子窗决定";
  EXPECT_FLOAT_EQ(sector.az_max_deg, 30.0f);
  EXPECT_NEAR(sector.el_min_deg, -0.4f, 1.0e-5f) << "下沿 = −θ_el/2";
  EXPECT_NEAR(sector.el_max_deg, 2.0f, 1.0e-6f) << "上沿 = +2°";
}

// 自适应带被硬件体积裁掉负仰角：体积 el_min = 0 时带下沿落到 0。
TEST(RirScanModeTest, HorizonBandIntersectsSteerableVolume) {
  RirMissionConfig mission;
  mission.search_mode = RirSearchMode::kHorizonScan;
  const auto sector = ResolveSearchSector(mission, MakeVolume(), 2.0f);
  EXPECT_FLOAT_EQ(sector.el_min_deg, 0.0f) << "体积不含负仰角，带下沿 −1° 被裁掉";
  EXPECT_FLOAT_EQ(sector.el_max_deg, 2.0f);
}

// 手动带：el_band_auto=false 时直接用配置值。
TEST(RirScanModeTest, HorizonBandManualValuesRespected) {
  RirHorizonScanConfig horizon;
  horizon.el_band_auto = false;
  horizon.el_lower_deg = 0.5f;
  horizon.el_upper_deg = 6.5f;
  const auto band = ResolveHorizonBand(horizon, 0.8f);
  EXPECT_FLOAT_EQ(band.el_min_deg, 0.5f);
  EXPECT_FLOAT_EQ(band.el_max_deg, 6.5f);

  RirMissionConfig mission;
  mission.search_mode = RirSearchMode::kHorizonScan;
  mission.horizon_scan = horizon;
  const auto sector = ResolveSearchSector(mission, MakeVolume(), 0.8f);
  EXPECT_FLOAT_EQ(sector.el_min_deg, 0.5f);
  EXPECT_FLOAT_EQ(sector.el_max_deg, 6.5f);
}

// 波束宽未知（0）时自适应下沿退化为 0，不产生非法负带。
TEST(RirScanModeTest, AdaptiveBandDegeneratesSafelyWithoutBeamwidth) {
  RirHorizonScanConfig horizon;
  const auto band = ResolveHorizonBand(horizon, 0.0f);
  EXPECT_FLOAT_EQ(band.el_min_deg, 0.0f);
  EXPECT_FLOAT_EQ(band.el_max_deg, 2.0f);
}

}  // namespace
}  // namespace remote_identification_radar
}  // namespace tests
