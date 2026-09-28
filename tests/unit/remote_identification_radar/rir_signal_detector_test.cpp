// Copyright 2026. All Rights Reserved.
//
// @file rir_signal_detector_test.cpp
// @brief 验证 RIR 统计级 CFAR 探测判决器（副本改写自 ar_signal_detection_test.cpp
//        的 SignalDetector 段；阶段 2-M M4：新增 Pfa 闭环与统计验证用例）。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "common/numerics/Constants.h"
#include "remote_identification_radar/dwell/RirSignalDetector.h"
#include "remote_identification_radar/internal/RirRadarEquations.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using dwell::RirDetectionResult;
using dwell::RirDetectorConfig;
using dwell::RirEnvironmentNoise;
using dwell::RirSignalDetector;
using dwell::RirTargetReturn;
using internal::RirRadarEquations;
using internal::RirSwerlingModel;

/// @brief 近距大 RCS 目标 → 高 SNR，必然探测成功。
TEST(RirSignalDetectorTest, CloseTarget_HighSNR_Detected) {
  RirDetectorConfig config;  // 默认参数（1MW, S-band, 35dB gain）
  RirSignalDetector detector(config);
  detector.SetRandomSeed(42u);

  RirTargetReturn target;  // 近距大目标：5km, RCS=100m²
  target.rcs_m2 = 100.0f;
  target.range_m = 5000.0f;

  RirEnvironmentNoise env;
  env.propagation_loss_db = 2.0f;

  const RirDetectionResult result = detector.Detect(target, env);

  EXPECT_TRUE(result.detected);
  EXPECT_GT(result.snr_db, 10.0f);
  EXPECT_GT(result.detection_prob, 0.9f);
}

/// @brief 远距小 RCS 目标 → 低 SNR。
TEST(RirSignalDetectorTest, FarTarget_LowSNR_NotDetected) {
  RirDetectorConfig config;
  config.transmitter.peak_power_w = 1e4f;  // 降低发射功率
  RirSignalDetector detector(config);
  detector.SetRandomSeed(42u);

  RirTargetReturn target;  // 远距小目标：300km, RCS=0.1m²
  target.rcs_m2 = 0.1f;
  target.range_m = 300000.0f;

  RirEnvironmentNoise env;
  env.propagation_loss_db = 5.0f;
  env.clutter_noise_w = 1e-14f;

  const RirDetectionResult result = detector.Detect(target, env);

  EXPECT_LT(result.snr_db, 0.0f);
}

/// @brief 相同随机种子 → 完全确定性输出（replay 语义）。
TEST(RirSignalDetectorTest, DeterministicWithSeed) {
  const RirDetectorConfig config;

  RirSignalDetector d1(config);
  d1.SetRandomSeed(123u);
  RirTargetReturn target;
  target.rcs_m2 = 10.0f;
  target.range_m = 50000.0f;
  RirEnvironmentNoise env;
  env.propagation_loss_db = 2.0f;
  const RirDetectionResult r1 = d1.Detect(target, env);

  RirSignalDetector d2(config);
  d2.SetRandomSeed(123u);
  const RirDetectionResult r2 = d2.Detect(target, env);

  EXPECT_EQ(r1.detected, r2.detected);
  EXPECT_FLOAT_EQ(r1.snr_db, r2.snr_db);
  EXPECT_FLOAT_EQ(r1.detection_prob, r2.detection_prob);
}

/// @brief 注入干扰噪声 → SNR 下降。
TEST(RirSignalDetectorTest, JammingIncreasesNoise) {
  const RirDetectorConfig config;
  RirSignalDetector detector(config);
  detector.SetRandomSeed(42u);

  RirTargetReturn target;
  target.rcs_m2 = 10.0f;
  target.range_m = 50000.0f;

  RirEnvironmentNoise env_clean;
  env_clean.propagation_loss_db = 2.0f;
  const RirDetectionResult clean = detector.Detect(target, env_clean);

  detector.SetRandomSeed(42u);
  RirEnvironmentNoise env_jam = env_clean;
  env_jam.jam_noise_w = 1e-10f;
  const RirDetectionResult jammed = detector.Detect(target, env_jam);

  EXPECT_LT(jammed.snr_db, clean.snr_db);
}

/// @brief 同能量（同峰值功率/同 τ）下带宽 ×4：热噪底 ×B，脉压增益同步 ×B
///        （B·τ ≥ 1 区），匹配滤波后单脉冲 SNR 不变（SNR = E/N₀ 口径）。
/// @note B·τ 脉压口径裁定前的旧口径下该差值为 +6.0206 dB（仅热噪底抬升）。
TEST(RirSignalDetectorTest, WiderBandwidthKeepsMatchedFilterSnrAtSameEnergy) {
  RirDetectorConfig narrow;
  narrow.transmitter.bandwidth_hz = 1.0e6f;
  narrow.transmitter.pulse_width_s = 13.0e-6f;
  RirSignalDetector detector_narrow(narrow);
  detector_narrow.SetRandomSeed(42u);

  RirDetectorConfig wide = narrow;
  wide.transmitter.bandwidth_hz = 4.0e6f;  // 带宽 ×4 → 噪声底 ×4、脉压增益 ×4
  RirSignalDetector detector_wide(wide);
  detector_wide.SetRandomSeed(42u);

  RirTargetReturn target;
  target.rcs_m2 = 10.0f;
  target.range_m = 50000.0f;
  RirEnvironmentNoise env;
  env.propagation_loss_db = 2.0f;

  const RirDetectionResult r_narrow = detector_narrow.Detect(target, env);
  const RirDetectionResult r_wide = detector_wide.Detect(target, env);

  EXPECT_NEAR(r_narrow.snr_db - r_wide.snr_db, 0.0f, 1.0e-3f);
}

/// @brief 回退路径手算恒等式（B·τ 脉压口径裁定）：snr_db =
///        10·log10(峰值方程回波 × B·τ / (kT·B·NF + 杂波 + 干扰))；
///        独立双精度复算全链路，容差 1e-3。
/// @note common 方程保留 `τ/13µs` 参考缩放；RIR 回退路径用局部净修正抵消它，
///       因而最终复算式只含一次 B·τ，与检测单元路径一致。
TEST(RirSignalDetectorTest, DetectSnrMatchesPeakEquationTimesBtOverNoiseBudget) {
  struct HandComputedBudget {
    double echo_power_dbw;
    double snr_db;
  };
  const auto hand_compute = [](const RirDetectorConfig& config,
                               const RirTargetReturn& target,
                               const RirEnvironmentNoise& env) {
    const double pi = oneq::common::numerics::kPi;
    const double wavelength_m =
        oneq::common::numerics::kLightSpeed / config.transmitter.frequency_hz;
    const double antenna_gain_linear =
        std::pow(10.0, config.antenna.main_beam_gain_db / 10.0);
    const double total_loss_db = config.transmitter.transmit_loss_db +
                                 env.propagation_loss_db + config.receiver.receive_loss_db;
    const double total_loss_linear = std::pow(10.0, total_loss_db / 10.0);
    const double time_bandwidth_product =
        static_cast<double>(config.transmitter.bandwidth_hz) * config.transmitter.pulse_width_s;
    const double echo_w =
        config.transmitter.peak_power_w * antenna_gain_linear * antenna_gain_linear *
        wavelength_m * wavelength_m * target.rcs_m2 * std::max(1.0, time_bandwidth_product) /
        (std::pow(4.0 * pi, 3.0) * std::pow(static_cast<double>(target.range_m), 4.0) *
         total_loss_linear);
    const double thermal_noise_w =
        oneq::common::numerics::kBoltzmann * 290.0 * config.transmitter.bandwidth_hz *
        std::pow(10.0, config.receiver.noise_figure_db / 10.0);
    HandComputedBudget budget;
    budget.echo_power_dbw = 10.0 * std::log10(echo_w);
    budget.snr_db = 10.0 * std::log10(echo_w / (thermal_noise_w + env.clutter_noise_w +
                                                env.jam_noise_w));
    return budget;
  };

  RirTargetReturn target;
  target.rcs_m2 = 5.0f;
  target.range_m = 80000.0f;
  RirEnvironmentNoise env;
  env.propagation_loss_db = 4.5f;
  env.clutter_noise_w = 3.0e-13f;
  env.jam_noise_w = 2.0e-13f;

  // 默认参数（B=4.5 MHz, τ=13 µs → B·τ=58.5，脉压增益 +17.67 dB）。
  const RirDetectorConfig config;
  RirSignalDetector detector(config);
  detector.SetRandomSeed(42u);
  const RirDetectionResult result = detector.Detect(target, env);
  const HandComputedBudget expected = hand_compute(config, target, env);
  EXPECT_NEAR(result.snr_db, expected.snr_db, 1.0e-3);
  EXPECT_NEAR(result.echo_power_dbw, expected.echo_power_dbw, 1.0e-3);

  // 非默认带宽/脉宽验证局部抵消生效，最终账本没有额外参考脉宽缩放项。
  RirDetectorConfig wide_pulse = config;
  wide_pulse.transmitter.bandwidth_hz = 2.0e6f;
  wide_pulse.transmitter.pulse_width_s = 26.0e-6f;
  RirSignalDetector wide_pulse_detector(wide_pulse);
  wide_pulse_detector.SetRandomSeed(42u);
  const RirDetectionResult wide_result = wide_pulse_detector.Detect(target, env);
  const HandComputedBudget wide_expected = hand_compute(wide_pulse, target, env);
  EXPECT_NEAR(wide_result.snr_db, wide_expected.snr_db, 1.0e-3);
  EXPECT_NEAR(wide_result.echo_power_dbw, wide_expected.echo_power_dbw, 1.0e-3);
  // 移除 τ/13µs 参考缩放后的净账本：默认 τ=13 µs 时该缩放恰为 1，wide_pulse
  // （τ=26 µs、B 减半）相对默认点只多出 10·log10(26/13)=+3.01 dB 的脉压增益
  // 净额（B·τ 由 58.5 降到 52 的 -0.51 dB 已含在脉压项内）。
  EXPECT_NEAR(wide_result.echo_power_dbw - result.echo_power_dbw,
              10.0 * std::log10(2.0), 1.0e-3)
      << "τ 翻倍后峰值回波差 = 脉压增益净额（+3.01 dB）";
}

/// @brief 同 SNR 下脉冲数 N=8 的 Pd 高于 N=1。
/// @note B·τ 脉压口径裁定后回退 SNR 抬升 +10·log10(B·τ)（默认 +17.67 dB），
///       原 RCS=10 m² 会把 Pd 推到饱和区（N=1 与 N=8 均 ≈1）；按同倍数
///       （÷58.5 ≈ 0.17 m²）调回 Pd 对 SNR 的敏感区，保持用例意图不变。
TEST(RirSignalDetectorTest, HigherPulseCountYieldsHigherPd) {
  const RirDetectorConfig config;
  RirSignalDetector detector(config);
  detector.SetRandomSeed(42u);

  RirTargetReturn target;
  target.rcs_m2 = 0.17f;
  target.range_m = 120000.0f;
  RirEnvironmentNoise env;
  env.propagation_loss_db = 3.0f;
  env.clutter_noise_w = 1.0e-14f;

  const RirDetectionResult r_n1 = detector.Detect(target, env);
  detector.SetRandomSeed(42u);
  const RirDetectionResult r_n8 = detector.Detect(target, env);
  const float pd_n1 = internal::RirRadarEquations::ComputeDetectionProbability(
      r_n1.snr_db, config.detection_policy.cfar_pfa, target.swerling_type, 1);
  const float pd_n8 = internal::RirRadarEquations::ComputeDetectionProbability(
      r_n8.snr_db, config.detection_policy.cfar_pfa, target.swerling_type, 8);
  EXPECT_GT(pd_n8, pd_n1);
}

// ===========================================================================
// 统计级 CFAR 口径验证（《能力边界界定》§3.1 决策；v2 计划 §7.3）
// ===========================================================================

/// @brief Pfa 闭环：SNR → -∞ 时 Pd 收敛到配置的 Pfa（门限由 Pfa 反解）。
TEST(RirCfarTest, PdConvergesToPfaAtZeroSignal) {
  const float pfa = 1.0e-2f;
  const float pd = RirRadarEquations::ComputeDetectionProbability(
      -300.0f, pfa, RirSwerlingModel::kSwerling0, 1);
  EXPECT_NEAR(pd, pfa, 1.0e-3f);
}

/// @brief 蒙特卡洛判决的统计频率收敛到输入概率（100k 抽样，±20% 相对容差）。
TEST(RirCfarTest, ThresholdDecisionRateMatchesProbability) {
  const float prob = 1.0e-2f;
  const int trials = 100000;
  std::mt19937 rng(7u);
  int hits = 0;
  for (int i = 0; i < trials; ++i) {
    if (RirRadarEquations::ThresholdDecision(prob, rng)) {
      ++hits;
    }
  }
  const double rate = static_cast<double>(hits) / trials;
  EXPECT_NEAR(rate, prob, prob * 0.2);
}

/// @brief 虚警率统计：门限判决 P=0.5 时频率接近 0.5（对称抽样快速收敛）。
TEST(RirCfarTest, ThresholdDecisionFairCoin) {
  const int trials = 100000;
  std::mt19937 rng(11u);
  int hits = 0;
  for (int i = 0; i < trials; ++i) {
    if (RirRadarEquations::ThresholdDecision(0.5f, rng)) {
      ++hits;
    }
  }
  const double rate = static_cast<double>(hits) / trials;
  EXPECT_NEAR(rate, 0.5, 5.0e-3);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
