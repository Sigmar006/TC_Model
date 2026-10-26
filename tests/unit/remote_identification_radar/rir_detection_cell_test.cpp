// Copyright 2026. All Rights Reserved.
//
// @file rir_detection_cell_test.cpp
// @brief 验证 RIR detection cell 求解器（副本改写自 ar_detection_cell_resolver_test.cpp；
//        阶段 2-M M3：新增四增益偏置单独激励与配置校验拒绝用例）。

#include <gtest/gtest.h>

#include <cmath>

#include "1q/electromagnetics/RfScene.h"
#include "1q/remote_identification_radar/config/RirSessionConfigValidation.h"
#include "remote_identification_radar/dwell/RirDetectionCellResolver.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using dwell::RirDetectionCellConfig;
using dwell::RirDetectionCellResult;
using dwell::RirDetectionCellTarget;
using dwell::TryResolveRirDetectionCell;

RirDetectionCellConfig MakeConfig() {
  RirDetectionCellConfig config;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      0.0, 10.0e9, 10.0e6, 1.0e6, 10.0e-6, 1.0e-3, 100U, 0.0, 11U, 0U,
      &config.own_transmit_waveform));
  config.receive_window_start_time_s = 0.0;
  config.receive_window_duration_s = 1.0;
  config.matched_filter_bandwidth_hz = 10.0e6;
  config.one_way_antenna_gain_dbi = 35.0;
  config.receiver_loss_db = 2.0;
  config.receiver_noise_figure_db = 4.0;
  // 偏置测试以零增益为基线：signal_processing 缺省 3/1/10/8 dB（RIR 产品缺省）
  // 会吃掉偏置差，须显式清零。
  config.signal_processing.target_processing_gain_db = 0.0f;
  config.signal_processing.noise_processing_gain_db = 0.0f;
  config.signal_processing.clutter_suppression_gain_db = 0.0f;
  config.signal_processing.jamming_suppression_gain_db = 0.0f;
  return config;
}

oneq::electromagnetics::RfIncidentLinkResult MakeIncidentLink(
    const oneq::electromagnetics::RfWaveformSchedule& waveform, double received_power_w,
    std::uint64_t emission_id = 6U) {
  oneq::electromagnetics::RfIncidentLinkResult link;
  link.identity = oneq::electromagnetics::RfEmissionIdentity{4U, emission_id};
  link.emission_waveform = waveform;
  link.receiver_sensor_entity_id = 5U;
  link.received_power_before_overlap_w = received_power_w;
  link.received_power_w = received_power_w;
  return link;
}

RirDetectionCellTarget MakeTarget(double range_m) {
  RirDetectionCellTarget target;
  target.range_m = range_m;
  target.closing_radial_velocity_mps = 300.0;
  target.rcs_m2 = 1.0;
  target.effective_pulse_count = 8U;
  return target;
}

const oneq::electromagnetics::RfEmissionIdentity kOwnIdentity{5U, 3U};

/// @brief 物理量自检：R⁴ 规律、时延线性、多普勒为正、脉压增益 = B·τ。
TEST(RirDetectionCellTest, EchoRangeDelayDopplerAndCompressionArePhysical) {
  RirDetectionCellResult near_cell;
  RirDetectionCellResult far_cell;
  ASSERT_TRUE(TryResolveRirDetectionCell(MakeConfig(), MakeTarget(10000.0), kOwnIdentity, {},
                                         0.0, &near_cell));
  ASSERT_TRUE(TryResolveRirDetectionCell(MakeConfig(), MakeTarget(20000.0), kOwnIdentity, {},
                                         0.0, &far_cell));
  EXPECT_NEAR(10.0 * std::log10(far_cell.echo_power_w / near_cell.echo_power_w), -12.0412, 1.0e-3);
  EXPECT_NEAR(far_cell.echo_delay_s / near_cell.echo_delay_s, 2.0, 1.0e-12);
  EXPECT_GT(near_cell.two_way_doppler_shift_hz, 0.0);
  EXPECT_DOUBLE_EQ(near_cell.pulse_compression_gain, 100.0);
}

/// @brief 干扰抬高噪声分母只影响 SINR；同带噪声压制不改变回波与分项功率。
TEST(RirDetectionCellTest, SuppressionOnlyChangesSinr) {
  const RirDetectionCellConfig config = MakeConfig();
  RirDetectionCellTarget target = MakeTarget(10000.0);
  RirDetectionCellResult baseline;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, kOwnIdentity, {}, 0.0, &baseline));

  oneq::electromagnetics::RfWaveformSchedule jammer_waveform;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(0.0, 1.0, 10.0e9, 10.0e6, 1.0,
                                                               &jammer_waveform));
  const auto jammer = MakeIncidentLink(jammer_waveform, baseline.thermal_noise_power_w * 10.0);
  RirDetectionCellResult jammed;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, kOwnIdentity, {jammer}, 0.0, &jammed));
  EXPECT_LT(jammed.processed_single_pulse_sinr_db, baseline.processed_single_pulse_sinr_db);
  EXPECT_DOUBLE_EQ(jammed.echo_power_w, baseline.echo_power_w);
  EXPECT_GT(jammed.interference_power_w, 0.0);

  // 更多的请求脉冲数不重复积分（单脉冲 SINR 语义不变）。
  target.effective_pulse_count = 64U;
  RirDetectionCellResult more_pulses;
  ASSERT_TRUE(
      TryResolveRirDetectionCell(config, target, kOwnIdentity, {jammer}, 0.0, &more_pulses));
  EXPECT_DOUBLE_EQ(more_pulses.processed_single_pulse_sinr_linear,
                   jammed.processed_single_pulse_sinr_linear);
  EXPECT_EQ(more_pulses.effective_pulse_count, 64U);
}

/// @brief 自身发射身份的入射链路不计入干扰。
TEST(RirDetectionCellTest, OwnEmissionIdentityExcludedFromInterference) {
  const RirDetectionCellConfig config = MakeConfig();
  const RirDetectionCellTarget target = MakeTarget(10000.0);
  RirDetectionCellResult baseline;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, kOwnIdentity, {}, 0.0, &baseline));

  oneq::electromagnetics::RfWaveformSchedule jammer_waveform;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(0.0, 1.0, 10.0e9, 10.0e6, 1.0,
                                                               &jammer_waveform));
  auto own_link = MakeIncidentLink(jammer_waveform, baseline.thermal_noise_power_w * 10.0);
  own_link.identity = kOwnIdentity;
  RirDetectionCellResult result;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, kOwnIdentity, {own_link}, 0.0, &result));
  EXPECT_DOUBLE_EQ(result.interference_power_w, 0.0);
  EXPECT_DOUBLE_EQ(result.processed_single_pulse_sinr_db, baseline.processed_single_pulse_sinr_db);
}

/// @brief 缺省偏置（全 0 dB）逐位等于保守账本：SINR = 回波×脉压/(热噪声+干扰+杂波)。
TEST(RirDetectionCellTest, DefaultGainsMatchConservativeLedger) {
  const RirDetectionCellConfig config = MakeConfig();
  const RirDetectionCellTarget target = MakeTarget(10000.0);
  RirDetectionCellResult result;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, kOwnIdentity, {}, 1.0e-15, &result));
  const double expected_sinr_db =
      10.0 * std::log10(result.echo_power_w * result.pulse_compression_gain /
                        (result.thermal_noise_power_w + result.interference_power_w +
                         result.clutter_power_w));
  EXPECT_NEAR(result.processed_single_pulse_sinr_db, expected_sinr_db, 1.0e-9);
}

/// @brief 目标信号偏置 +3 dB → SINR +3 dB（单独激励，其余分项不变）。
TEST(RirDetectionCellTest, TargetGainOffsetRaisesSinrByOffset) {
  RirDetectionCellConfig config = MakeConfig();
  const RirDetectionCellTarget target = MakeTarget(10000.0);
  RirDetectionCellResult baseline;
  RirDetectionCellResult boosted;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, kOwnIdentity, {}, 0.0, &baseline));
  config.signal_processing.target_processing_gain_db = 3.0f;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, kOwnIdentity, {}, 0.0, &boosted));
  EXPECT_NEAR(boosted.processed_single_pulse_sinr_db - baseline.processed_single_pulse_sinr_db,
              3.0, 1.0e-6);
}

/// @brief 噪声代价 +3 dB → 噪声底 ×10^0.3 → SINR 恰降 3 dB。
TEST(RirDetectionCellTest, NoiseGainOffsetLowersSinrWhenNoiseDominant) {
  RirDetectionCellConfig config = MakeConfig();
  const RirDetectionCellTarget target = MakeTarget(10000.0);
  RirDetectionCellResult baseline;
  RirDetectionCellResult penalized;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, kOwnIdentity, {}, 0.0, &baseline));
  config.signal_processing.noise_processing_gain_db = 3.0f;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, kOwnIdentity, {}, 0.0, &penalized));
  EXPECT_NEAR(baseline.processed_single_pulse_sinr_db - penalized.processed_single_pulse_sinr_db,
              3.0, 1.0e-6);
}

/// @brief 杂波抑制 +10 dB（杂波主导）→ SINR +10 dB；分项物理功率不变。
TEST(RirDetectionCellTest, ClutterSuppressionOffsetRaisesSinrWhenClutterDominant) {
  RirDetectionCellConfig config = MakeConfig();
  const RirDetectionCellTarget target = MakeTarget(10000.0);
  const double dominant_clutter_w = 1.0e-8;  // 远大于热噪声（残留热噪声引起 <1e-3 dB 偏差）
  RirDetectionCellResult baseline;
  RirDetectionCellResult suppressed;
  ASSERT_TRUE(
      TryResolveRirDetectionCell(config, target, kOwnIdentity, {}, dominant_clutter_w, &baseline));
  config.signal_processing.clutter_suppression_gain_db = 10.0f;
  ASSERT_TRUE(
      TryResolveRirDetectionCell(config, target, kOwnIdentity, {}, dominant_clutter_w,
                                 &suppressed));
  EXPECT_NEAR(suppressed.processed_single_pulse_sinr_db - baseline.processed_single_pulse_sinr_db,
              10.0, 1.0e-3);
  EXPECT_DOUBLE_EQ(suppressed.clutter_power_w, baseline.clutter_power_w);
}

/// @brief 干扰抑制 +10 dB（干扰主导）→ SINR +10 dB。
TEST(RirDetectionCellTest, JammingSuppressionOffsetRaisesSinrWhenJammingDominant) {
  RirDetectionCellConfig config = MakeConfig();
  const RirDetectionCellTarget target = MakeTarget(10000.0);
  RirDetectionCellResult probe;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, kOwnIdentity, {}, 0.0, &probe));

  oneq::electromagnetics::RfWaveformSchedule jammer_waveform;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(0.0, 1.0, 10.0e9, 10.0e6, 1.0,
                                                               &jammer_waveform));
  const auto jammer = MakeIncidentLink(jammer_waveform, probe.thermal_noise_power_w * 1000.0);
  RirDetectionCellResult baseline;
  RirDetectionCellResult suppressed;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, kOwnIdentity, {jammer}, 0.0, &baseline));
  config.signal_processing.jamming_suppression_gain_db = 10.0f;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, kOwnIdentity, {jammer}, 0.0,
                                         &suppressed));
  // 干扰聚合含时频重叠分数与热噪声残留，+10 dB 偏置的实测提升略低于理想值。
  EXPECT_NEAR(suppressed.processed_single_pulse_sinr_db - baseline.processed_single_pulse_sinr_db,
              10.0, 0.05);
}

/// @brief 越界/非有限增益偏置被配置校验拒绝（rir.validation.signal_processing_gains_invalid）。
TEST(RirDetectionCellTest, GainOffsetsOutOfRangeRejectedByConfigValidation) {
  config::RirSessionConfig session_config;
  session_config.hardware.signal_processing.clutter_suppression_gain_db = 41.0f;
  session::RirIssueList issues = config::ValidateRirSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.front().code, "rir.validation.signal_processing_gains_invalid");

  session_config.hardware.signal_processing.clutter_suppression_gain_db = -1.0f;
  issues = config::ValidateRirSessionConfig(session_config);
  EXPECT_EQ(issues.front().code, "rir.validation.signal_processing_gains_invalid");

  session_config.hardware.signal_processing.noise_processing_gain_db = 40.0f;
  session_config.hardware.signal_processing.clutter_suppression_gain_db = 0.0f;
  issues = config::ValidateRirSessionConfig(session_config);
  EXPECT_TRUE(issues.empty());
}

/// @brief 四域归位后：任务域识别最大距离/驻留非法被配置校验拒绝。
TEST(RirDetectionCellTest, MissionRangeAndDwellRejectedByConfigValidation) {
  config::RirSessionConfig session_config;
  session_config.mission.max_range_m = 0.0f;
  session::RirIssueList issues = config::ValidateRirSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  bool found = false;
  for (const auto& issue : issues) {
    if (issue.code == "rir.validation.recognition_time_range_invalid" &&
        issue.field.find("mission.max_range_m") != std::string::npos) {
      found = true;
    }
  }
  EXPECT_TRUE(found);

  session_config.mission.max_range_m = 300000.0f;
  session_config.mission.recognition_dwell_sec = -1.0f;
  issues = config::ValidateRirSessionConfig(session_config);
  found = false;
  for (const auto& issue : issues) {
    if (issue.code == "rir.validation.recognition_time_range_invalid" &&
        issue.field.find("recognition_dwell_sec") != std::string::npos) {
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

/// @brief 扫描策略/体积/朝向非法被配置校验拒绝。
TEST(RirDetectionCellTest, ScanStrategyRejectedByConfigValidation) {
  config::RirSessionConfig session_config;

  // 体积方位乱序。
  session_config.orientation.az_min_deg = 30.0f;
  session_config.orientation.az_max_deg = -30.0f;
  session::RirIssueList issues = config::ValidateRirSessionConfig(session_config);
  bool found = false;
  for (const auto& issue : issues) {
    if (issue.code == "rir.validation.steerable_volume_invalid") {
      found = true;
    }
  }
  EXPECT_TRUE(found);

  // 体积方位越相对域（az > 180）。
  session_config.orientation.az_min_deg = -60.0f;
  session_config.orientation.az_max_deg = 200.0f;
  issues = config::ValidateRirSessionConfig(session_config);
  found = false;
  for (const auto& issue : issues) {
    if (issue.code == "rir.validation.steerable_volume_invalid") {
      found = true;
    }
  }
  EXPECT_TRUE(found);

  // 转台朝向越域。
  session_config.orientation.az_max_deg = 60.0f;
  session_config.mission.scan_center_deg.az_deg = 200.0f;
  issues = config::ValidateRirSessionConfig(session_config);
  found = false;
  for (const auto& issue : issues) {
    if (issue.code == "rir.validation.scan_center_invalid") {
      found = true;
    }
  }
  EXPECT_TRUE(found);

  // 步长系数非正。
  session_config.mission.scan_center_deg.az_deg = 0.0f;
  session_config.mission.step_scale = 0.0f;
  issues = config::ValidateRirSessionConfig(session_config);
  found = false;
  for (const auto& issue : issues) {
    if (issue.code == "rir.validation.scan_strategy_invalid") {
      found = true;
    }
  }
  EXPECT_TRUE(found);

  // 默认扫描配置合法。
  session_config.mission.step_scale = 1.0f;
  issues = config::ValidateRirSessionConfig(session_config);
  EXPECT_TRUE(issues.empty());
}

/// @brief 甲方探测距离硬指标（2026-08-30 口径）：峰值功率 582 kW、天线增益 52 dBi、
///        脉宽 16 ms、积累 1 脉冲下——3400 km 探测 0.025 m²、8550 km 探测 1 m²。
///        两指标点为未计外部干扰的底数，干净链路须高出 6 dB kSnrFallback 判决门
///        ≥3 dB（干扰余量）。参数镜像弹道场景发射/接收链（3 GHz、B=4.5 MHz、
///        发射损耗 3.5 dB、接收损耗 2 dB、噪系数 4 dB、产品缺省处理增益 3/1/10/8）。
TEST(RirDetectionCellTest, CustomerDetectionRangeRequirementsHoldWithMargin) {
  // 辐射峰值功率 = 582 kW 扣 3.5 dB 发射损耗（与 RirEmissionFactory 同口径）。
  const double radiated_peak_power_w = 582.0e3 * std::pow(10.0, -3.5 / 10.0);

  RirDetectionCellConfig config;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      0.0, 3.0e9, 4.5e6, radiated_peak_power_w, 0.016, 0.1, 1U, 0.0, 1U, 0U,
      &config.own_transmit_waveform));
  config.receive_window_start_time_s = 0.0;
  config.receive_window_duration_s = 1.0;
  config.matched_filter_bandwidth_hz = 4.5e6;
  config.one_way_antenna_gain_dbi = 52.0;
  config.receiver_loss_db = 2.0;
  config.receiver_noise_figure_db = 4.0;

  struct SpecPoint {
    double range_m;
    double rcs_m2;
  };
  const SpecPoint spec_points[] = {{3.4e6, 0.025}, {8.55e6, 1.0}};
  for (const SpecPoint& point : spec_points) {
    RirDetectionCellTarget target;
    target.range_m = point.range_m;
    target.rcs_m2 = point.rcs_m2;
    target.closing_radial_velocity_mps = 1000.0;
    target.effective_pulse_count = 1U;

    RirDetectionCellResult cell;
    ASSERT_TRUE(TryResolveRirDetectionCell(config, target, kOwnIdentity, {}, 0.0, &cell))
        << "range=" << point.range_m;
    // 判决门 6 dB + 3 dB 外部干扰余量。
    EXPECT_GE(cell.processed_single_pulse_sinr_db, 9.0)
        << "range=" << point.range_m << " rcs=" << point.rcs_m2
        << " sinr_db=" << cell.processed_single_pulse_sinr_db;
  }
}


/// @brief MTI/MTD 联算（2026-10-22）：开启后 SINR 相对"单脉冲账本"的变化必须等于
///        MTI 响应 + 相干积累 10log10(N) + MTD 跨 bin 响应的代数和。
TEST(RirDetectionCellMtiMtdTest, SinrChangeMatchesMtiCoherentAndBinTerms) {
  RirDetectionCellConfig config = MakeConfig();
  config.signal_processing.target_processing_gain_db = 0.0f;
  config.signal_processing.noise_processing_gain_db = 0.0f;
  config.signal_processing.clutter_suppression_gain_db = 0.0f;
  config.signal_processing.jamming_suppression_gain_db = 0.0f;

  RirDetectionCellTarget target;
  target.range_m = 1.0e6;
  target.rcs_m2 = 1.0;
  target.effective_pulse_count = 10U;
  target.closing_radial_velocity_mps = 750.0;

  const oneq::electromagnetics::RfEmissionIdentity own{1U, 1U};
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> none;
  RirDetectionCellResult baseline;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, own, none, 0.0, &baseline));

  config.enable_mti_mtd_effects = true;
  config.signal_processing.enable_mti_mtd_effects = true;
  RirDetectionCellResult integrated;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, own, none, 0.0, &integrated));

  ASSERT_GE(integrated.effective_pulse_count, 2U);
  const double delta_db =
      integrated.processed_single_pulse_sinr_db - baseline.processed_single_pulse_sinr_db;
  // 中译：**对消器噪声增益**（2026-10-26）——2 脉冲对消器对白噪声的功率增益为 2，
  //   已并入 SINR 分母，故 MTI 开启后的 SINR 变化还要减去 10log10(2)。
  // 标识：对消器噪声增益计入期望值（2026-10-26）。
  const double canceller_noise_db = 10.0 * std::log10(2.0);
  const double expected_db = integrated.mti_response_db + integrated.coherent_integration_db +
                             integrated.mtd_bin_response_db - canceller_noise_db;
  EXPECT_NEAR(delta_db, expected_db, 1.0e-6);
  EXPECT_NEAR(integrated.coherent_integration_db, 10.0, 1.0e-6);   // N=10
  EXPECT_LE(integrated.mtd_bin_response_db, 0.0);                  // 恒为损耗
  EXPECT_LT(integrated.mti_response_db, 6.1);                      // 上限 10log10(4)
}

/// @brief 零径向速度 = 盲速：MTI 响应应落到地板（远低于 -100 dB）。
TEST(RirDetectionCellMtiMtdTest, ZeroRadialVelocityHitsMtiBlindSpeed) {
  RirDetectionCellConfig config = MakeConfig();
  config.enable_mti_mtd_effects = true;
  config.signal_processing.enable_mti_mtd_effects = true;
  RirDetectionCellTarget target;
  target.range_m = 1.0e6;
  target.rcs_m2 = 1.0;
  target.closing_radial_velocity_mps = 0.0;
  target.effective_pulse_count = 10U;
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 1U};
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> none;
  RirDetectionCellResult result;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, own, none, 0.0, &result));
  EXPECT_LT(result.mti_response_db, -100.0);
  EXPECT_NEAR(result.coherent_integration_db, 10.0, 1.0e-6);
}

/// @brief 脉冲数 < 2 时不启用联算：行为与关闭开关逐位一致。
TEST(RirDetectionCellMtiMtdTest, SinglePulseKeepsLegacyLedger) {
  RirDetectionCellConfig config = MakeConfig();
  config.enable_mti_mtd_effects = true;
  config.signal_processing.enable_mti_mtd_effects = true;
  RirDetectionCellTarget target;
  target.range_m = 1.0e6;
  target.rcs_m2 = 1.0;
  target.closing_radial_velocity_mps = 100.0;
  target.effective_pulse_count = 1U;
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 1U};
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> none;
  RirDetectionCellResult result;
  ASSERT_TRUE(TryResolveRirDetectionCell(config, target, own, none, 0.0, &result));
  EXPECT_DOUBLE_EQ(result.mti_response_db, 0.0);
  EXPECT_DOUBLE_EQ(result.coherent_integration_db, 0.0);
  EXPECT_DOUBLE_EQ(result.mtd_bin_response_db, 0.0);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
