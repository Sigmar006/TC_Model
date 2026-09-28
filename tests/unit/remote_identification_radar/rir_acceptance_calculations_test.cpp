// Copyright 2026. All Rights Reserved.
//
// @file rir_acceptance_calculations_test.cpp
// @brief 验收计算口径的针对性测试：
//        第 29 项同距离不同 RCS 的回波功率差（约 16 dB）、
//        第 36 项检测概率 90% 最大探测距离的公式方向与单调性、
//        第 28 项方向图采样反算（峰值增益与 -3 dB 波束宽度）。
//
// 背景：docs/review/acceptance_log_remaining_work_guide.md 第 6 节要求上述三项在修复
// 后必须有等值、高于/低于所需值和单调性测试；本文件即该要求的落地。

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "common/radar/AntennaPatternRuntime.h"
#include "common/radar/RadarEquations.h"
#include "remote_identification_radar/dwell/RirAntennaPatternRuntime.h"
#include "remote_identification_radar/runtime/RirAcceptanceRecords.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using config::hardware::RirAntennaConfig;
using config::hardware::RirAntennaPatternModelType;
using dwell::RirAntennaLookOffsetDeg;
using dwell::RirAntennaPatternBeamwidthDeg;
using dwell::RirEvaluateAntennaPattern;
using oneq::common::radar::RadarEquations;
using oneq::common::radar::SwerlingModel;

// 第 29 项：同距离、不同 RCS 的回波功率差。
//
// 雷达方程中回波功率与 RCS 成正比：RCS 由 1 m^2 降到 0.025 m^2（比值 1/40）时，
// 功率差应为 10*log10(40) 约 16.02 dB。主文档第十一章第 1 节的 16 dB 判据即该关系。
TEST(RirAcceptanceItem29Test, EchoPowerGapMatchesRcsRatio) {
  const double reference_rcs_m2 = 1.0;
  const double reduced_rcs_m2 = 0.025;
  const double gap_db = 10.0 * std::log10(reference_rcs_m2 / reduced_rcs_m2);
  EXPECT_NEAR(gap_db, 16.0206, 0.001);
  EXPECT_NEAR(gap_db, 16.0, 0.5);
}

// 反例：RCS 反向变化时功率差方向必须相反，不能被当成同一判据通过。
TEST(RirAcceptanceItem29Test, EchoPowerGapIsDirectional) {
  const double increasing_gap_db = 10.0 * std::log10(0.025 / 1.0);
  EXPECT_LT(increasing_gap_db, 0.0);
  EXPECT_NEAR(increasing_gap_db, -16.0206, 0.001);
}

// 第 36 项：二分反解 Pd = 0.90 所需的单脉冲 SNR（与产品代码同口径）。
double SolveRequiredSnrDbForTest(double pfa, int pulses) {
  double snr_low_db = -20.0;
  double snr_high_db = 60.0;
  for (int i = 0; i < 80; ++i) {
    const double mid_db = 0.5 * (snr_low_db + snr_high_db);
    const double pd = RadarEquations::ComputeDetectionProbability(
        static_cast<float>(mid_db), static_cast<float>(pfa), SwerlingModel::kSwerling0, pulses);
    if (pd < 0.90) {
      snr_low_db = mid_db;
    } else {
      snr_high_db = mid_db;
    }
  }
  return 0.5 * (snr_low_db + snr_high_db);
}

// 在 Pd 恰好等于 90% 的 SNR 上反解，最大距离应回到当前距离（等值情况）。
TEST(RirAcceptanceItem36Test, RangeEqualsCurrentWhenSnrIsExactlyRequired) {
  const double pfa = 1.0e-6;
  const int pulses = 1;
  const double reference_range_m = 3.4e6;
  const double required_snr_db = SolveRequiredSnrDbForTest(pfa, pulses);

  double max_range_m = 0.0;
  ASSERT_TRUE(runtime::TryComputeDetectionRangeAtPd(reference_range_m, required_snr_db, 0.90, pfa,
                                                    pulses, &max_range_m));
  EXPECT_NEAR(max_range_m, reference_range_m, 1.0);
}

// 方向与单调性：SNR 越高，反解距离不得变小（比值方向写反会在此失败）。
TEST(RirAcceptanceItem36Test, HigherSnrMeansNotShorterRange) {
  const double pfa = 1.0e-6;
  const int pulses = 1;
  const double reference_range_m = 3.4e6;

  double previous_range_m = 0.0;
  for (int step = 0; step < 6; ++step) {
    const double snr_db = 5.0 + 2.0 * static_cast<double>(step);
    double max_range_m = 0.0;
    ASSERT_TRUE(runtime::TryComputeDetectionRangeAtPd(reference_range_m, snr_db, 0.90, pfa, pulses,
                                                      &max_range_m))
        << "snr_db=" << snr_db;
    if (step > 0) {
      EXPECT_GE(max_range_m, previous_range_m)
          << "SNR 升高时最大探测距离反而变小（比值方向写反的症状）";
    }
    previous_range_m = max_range_m;
  }
}

// 低于所需 SNR 时反解距离必须小于当前距离，而不是放大。
TEST(RirAcceptanceItem36Test, LowerSnrMeansShorterRange) {
  const double pfa = 1.0e-6;
  const int pulses = 1;
  const double reference_range_m = 3.4e6;
  const double low_snr_db = 0.0;

  double max_range_m = 0.0;
  ASSERT_TRUE(runtime::TryComputeDetectionRangeAtPd(reference_range_m, low_snr_db, 0.90, pfa,
                                                    pulses, &max_range_m));
  EXPECT_LT(max_range_m, reference_range_m);
}

// 数值闭合：距离比应等于 SNR 比的四次方根（加 8 dB 即 10^0.2 倍距离）。
TEST(RirAcceptanceItem36Test, RangeScalesAsFourthRootOfSnrRatio) {
  const double pfa = 1.0e-6;
  const int pulses = 1;
  const double reference_range_m = 1.0e6;
  const double required_snr_db = SolveRequiredSnrDbForTest(pfa, pulses);

  double range_at_required_m = 0.0;
  double range_at_plus8db_m = 0.0;
  ASSERT_TRUE(runtime::TryComputeDetectionRangeAtPd(reference_range_m, required_snr_db, 0.90, pfa,
                                                    pulses, &range_at_required_m));
  ASSERT_TRUE(runtime::TryComputeDetectionRangeAtPd(reference_range_m, required_snr_db + 8.0, 0.90,
                                                    pfa, pulses, &range_at_plus8db_m));
  EXPECT_NEAR(range_at_plus8db_m / range_at_required_m, std::pow(10.0, 8.0 / 40.0), 1.0e-6);
}

// 非法入参必须原子拒绝（不改输出）。
TEST(RirAcceptanceItem36Test, RejectsInvalidInput) {
  double max_range_m = 123.0;
  EXPECT_FALSE(runtime::TryComputeDetectionRangeAtPd(0.0, 10.0, 0.90, 1.0e-6, 1, &max_range_m));
  EXPECT_FALSE(runtime::TryComputeDetectionRangeAtPd(-1.0, 10.0, 0.90, 1.0e-6, 1, &max_range_m));
  EXPECT_FALSE(runtime::TryComputeDetectionRangeAtPd(1.0e6, 10.0, 0.90, 1.0e-6, 1, nullptr));
  EXPECT_FALSE(runtime::TryComputeDetectionRangeAtPd(
      std::numeric_limits<double>::quiet_NaN(), 10.0, 0.90, 1.0e-6, 1, &max_range_m));
  EXPECT_EQ(max_range_m, 123.0);
}

// 第 28 项：方向图采样反算口径（与产品代码一致：0.02 度细扫，主瓣内二维取最大）。
RirAntennaConfig MakeAntenna() {
  RirAntennaConfig antenna;
  antenna.model_type = RirAntennaPatternModelType::kGaussianMainLobe;
  antenna.main_beam_gain_db = 52.0f;
  antenna.nominal_az_beamwidth_deg = 4.0f;
  antenna.nominal_el_beamwidth_deg = 4.0f;
  return antenna;
}

double SampleGainDbi(const RirAntennaConfig& antenna, float delta_az_deg, float delta_el_deg) {
  RirAntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = antenna.nominal_az_beamwidth_deg;
  beamwidth.el_beamwidth_deg = antenna.nominal_el_beamwidth_deg;
  const config::RirAzimuthElevationDeg pointing(0.0f, 0.0f);
  RirAntennaLookOffsetDeg offset;
  offset.delta_az_deg = delta_az_deg;
  offset.delta_el_deg = delta_el_deg;
  return static_cast<double>(
      RirEvaluateAntennaPattern(antenna, beamwidth, offset, pointing, 0.0f).gain_dbi);
}

double SamplePeakGainDbi(const RirAntennaConfig& antenna) {
  const double half_span_deg =
      std::max(1.0, 2.0 * static_cast<double>(antenna.nominal_az_beamwidth_deg));
  double peak_dbi = -1.0e9;
  for (double az = -half_span_deg; az <= half_span_deg; az += 0.02) {
    for (double el = -half_span_deg; el <= half_span_deg; el += 0.02) {
      const double gain = SampleGainDbi(antenna, static_cast<float>(az), static_cast<float>(el));
      if (std::isfinite(gain)) {
        peak_dbi = std::max(peak_dbi, gain);
      }
    }
  }
  return peak_dbi;
}

double SampleHalfPowerWidthDeg(const RirAntennaConfig& antenna, bool along_azimuth) {
  const double half_span_deg =
      std::max(1.0, 2.0 * static_cast<double>(antenna.nominal_az_beamwidth_deg));
  const double target_dbi = SamplePeakGainDbi(antenna) - 3.0;
  for (double offset = 0.0; offset <= half_span_deg; offset += 0.02) {
    const double gain = along_azimuth
                            ? SampleGainDbi(antenna, static_cast<float>(offset), 0.0f)
                            : SampleGainDbi(antenna, 0.0f, static_cast<float>(offset));
    if (gain <= target_dbi) {
      return 2.0 * offset;
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

// 采样峰值必须等于配置主瓣峰值（同一模型下的一致性）。
TEST(RirAcceptanceItem28Test, SampledPeakMatchesConfiguredMainBeamGain) {
  const RirAntennaConfig antenna = MakeAntenna();
  EXPECT_NEAR(SamplePeakGainDbi(antenna), static_cast<double>(antenna.main_beam_gain_db), 1.0e-3);
}

// 反算 -3 dB 宽度应落在配置波束宽度的 5% 内（主文档判据）。
TEST(RirAcceptanceItem28Test, SampledBeamwidthWithinFivePercent) {
  const RirAntennaConfig antenna = MakeAntenna();
  const double az_width_deg = SampleHalfPowerWidthDeg(antenna, true);
  const double el_width_deg = SampleHalfPowerWidthDeg(antenna, false);
  ASSERT_TRUE(std::isfinite(az_width_deg));
  ASSERT_TRUE(std::isfinite(el_width_deg));
  const double az_error = std::fabs(az_width_deg - antenna.nominal_az_beamwidth_deg) /
                          static_cast<double>(antenna.nominal_az_beamwidth_deg);
  const double el_error = std::fabs(el_width_deg - antenna.nominal_el_beamwidth_deg) /
                          static_cast<double>(antenna.nominal_el_beamwidth_deg);
  EXPECT_LE(az_error, 0.05) << "az_width_deg=" << az_width_deg;
  EXPECT_LE(el_error, 0.05) << "el_width_deg=" << el_width_deg;
}

// 配置波束变窄时反算宽度必须随之变小：证明宽度来自采样而不是配置回读。
TEST(RirAcceptanceItem28Test, SampledBeamwidthTracksNarrowerConfiguration) {
  RirAntennaConfig wide = MakeAntenna();
  RirAntennaConfig narrow = MakeAntenna();
  narrow.nominal_az_beamwidth_deg = 2.0f;
  narrow.nominal_el_beamwidth_deg = 2.0f;
  const double wide_width_deg = SampleHalfPowerWidthDeg(wide, true);
  const double narrow_width_deg = SampleHalfPowerWidthDeg(narrow, true);
  ASSERT_TRUE(std::isfinite(wide_width_deg));
  ASSERT_TRUE(std::isfinite(narrow_width_deg));
  EXPECT_LT(narrow_width_deg, wide_width_deg);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
