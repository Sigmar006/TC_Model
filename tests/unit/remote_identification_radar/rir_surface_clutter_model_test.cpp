// Copyright 2026. All Rights Reserved.
//
// @file rir_surface_clutter_model_test.cpp
// @brief 逐目标主瓣地杂波最小物理模型特征化测试。
//
// 锁定口径：σ₀ 档位序、擦地角几何响应（主瓣离地归零）、脉冲限制区距离
// 衰减、雷达方程 λ² 频率响应、退化输入免疫与确定性。序关系与量级带锁定
// 物理行为；杂波 CNR 含脉压能量增益 max(1, B·τ)（B·τ 脉压口径裁定），
// 以独立双精度手算恒等式锁定（旧公式 dB 值 + 10·log10(max(1, B·τ))）。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "common/numerics/Constants.h"
#include "remote_identification_radar/internal/RirSurfaceClutterModel.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using config::RirVegetationCoverProfile;
using internal::RirSurfaceClutterInput;
using internal::RirSurfaceClutterModel;

constexpr float kThermalNoiseW = 4.5e-14f;  ///< kTBF 量级参考（NF≈4 dB @ 4.5 MHz）。

RirSurfaceClutterInput MakeInput(RirVegetationCoverProfile profile, float range_m,
                                 float look_el_deg) {
  RirSurfaceClutterInput input;
  input.vegetation_cover_profile = profile;
  input.transmitter.frequency_hz = 3.0e9f;
  input.transmitter.bandwidth_hz = 4.5e6f;
  input.transmitter.peak_power_w = 1.0e6f;
  input.transmitter.transmit_loss_db = 3.5f;
  input.antenna.main_beam_gain_db = 35.0f;
  input.antenna.nominal_az_beamwidth_deg = 4.0f;
  input.antenna.nominal_el_beamwidth_deg = 4.0f;
  input.propagation_loss_db = 6.5f;
  input.range_m = range_m;
  input.look_el_deg = look_el_deg;
  input.thermal_noise_w = kThermalNoiseW;
  return input;
}

float CnrDb(const RirSurfaceClutterModel& model, const RirSurfaceClutterInput& input) {
  const float clutter_w = model.EvaluateClutterNoiseW(input);
  return 10.0f * std::log10(clutter_w / input.thermal_noise_w);
}

/// @brief 独立双精度手算期望（B·τ 脉压口径）：旧公式 dB 值（峰值雷达方程 +
///        common τ/13µs 参考能量缩放）+ 10·log10(max(1, B·τ)) − 10·log10(热噪)。
/// @note  输入须用 MakeInput 构造（nominal 4°/4° 为正直接生效 → 波束宽度 4°；
///        草地档 σ₀ = -28 dB），手算链与被测实现无共享代码路径。
double HandComputedCnrDb(const RirSurfaceClutterInput& input, double sigma0_profile_db) {
  const double pi = oneq::common::numerics::kPi;
  const double light_speed_mps = oneq::common::numerics::kLightSpeed;
  const double wavelength_m = light_speed_mps / input.transmitter.frequency_hz;
  const double az_bw_rad = 4.0 * pi / 180.0;
  const double el_bw_rad = 4.0 * pi / 180.0;
  const double grazing_deg = std::min(std::max(2.0 - input.look_el_deg, 1.0), 89.0);
  const double grazing_rad = grazing_deg * pi / 180.0;
  const double range_cell_m = light_speed_mps / (2.0 * input.transmitter.bandwidth_hz);
  const double pulse_limited_area_m2 =
      input.range_m * az_bw_rad * range_cell_m / std::cos(grazing_rad);
  const double beam_limited_area_m2 =
      input.range_m * input.range_m * az_bw_rad * el_bw_rad / std::sin(grazing_rad);
  const double clutter_area_m2 = std::min(pulse_limited_area_m2, beam_limited_area_m2);
  const double grazing_factor_db =
      10.0 * std::log10(std::sin(grazing_rad) / std::sin(10.0 * pi / 180.0));
  const double clutter_rcs_db =
      sigma0_profile_db + grazing_factor_db + 10.0 * std::log10(clutter_area_m2);
  const double legacy_echo_dbw =
      10.0 * std::log10(input.transmitter.peak_power_w) +
      10.0 * std::log10(input.transmitter.pulse_width_s / 13.0e-6) +
      2.0 * input.antenna.main_beam_gain_db + 20.0 * std::log10(wavelength_m) +
      clutter_rcs_db - 30.0 * std::log10(4.0 * pi) - 40.0 * std::log10(input.range_m) -
      (input.transmitter.transmit_loss_db + input.propagation_loss_db);
  const double time_bandwidth_product =
      static_cast<double>(input.transmitter.bandwidth_hz) * input.transmitter.pulse_width_s;
  const double pulse_compression_gain_db =
      10.0 * std::log10(std::max(1.0, time_bandwidth_product));
  return legacy_echo_dbw + pulse_compression_gain_db - 10.0 * std::log10(input.thermal_noise_w);
}

/// @brief 植被关闭恒零（行为不变式：默认环境零杂波）。
TEST(RirSurfaceClutterModelTest, DisabledVegetationIsZero) {
  const RirSurfaceClutterModel model;
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(
                      MakeInput(RirVegetationCoverProfile::kDisabled, 5000.0f, 0.0f)),
                  0.0f);
}

/// @brief 主瓣离地归零：目标仰角达到半俯仰波束宽（4°/2 = 2°）后无主瓣地杂波。
TEST(RirSurfaceClutterModelTest, BeamFullyAboveHorizonIsZero) {
  const RirSurfaceClutterModel model;
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(
                      MakeInput(RirVegetationCoverProfile::kDeciduousForest, 5000.0f, 2.0f)),
                  0.0f);
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(
                      MakeInput(RirVegetationCoverProfile::kDeciduousForest, 5000.0f, 8.0f)),
                  0.0f);
}

/// @brief 地平线附近杂波存在且随俯角压低单调增强（擦地角增大 → σ₀ 因子增大）。
TEST(RirSurfaceClutterModelTest, NearHorizonClutterGrowsWithDepression) {
  const RirSurfaceClutterModel model;
  const float at_horizon =
      model.EvaluateClutterNoiseW(MakeInput(RirVegetationCoverProfile::kOpenGrassland,
                                            5000.0f, 0.0f));
  const float depressed =
      model.EvaluateClutterNoiseW(MakeInput(RirVegetationCoverProfile::kOpenGrassland,
                                            5000.0f, -5.0f));
  EXPECT_GT(at_horizon, 0.0f);
  EXPECT_GT(depressed, at_horizon);
}

/// @brief 量级带：参考几何（草地 5 km 地平线）CNR 落在主瓣地杂波物理量级内。
TEST(RirSurfaceClutterModelTest, ReferenceGeometryCnrInPhysicalBand) {
  const RirSurfaceClutterModel model;
  const float cnr_db = CnrDb(model, MakeInput(RirVegetationCoverProfile::kOpenGrassland,
                                              5000.0f, 0.0f));
  EXPECT_GT(cnr_db, 20.0f);
  EXPECT_LT(cnr_db, 100.0f);
}

/// @brief 手算恒等式（B·τ 脉压口径裁定）：CNR = 旧公式 dB 值 +
///        10·log10(max(1, B·τ)) − 10·log10(热噪)；独立双精度复算，容差 1e-3 dB。
TEST(RirSurfaceClutterModelTest, CnrMatchesLegacyPlusBtPulseCompressionGain) {
  const RirSurfaceClutterModel model;
  constexpr double kGrasslandSigma0Db = -28.0;

  // 默认参数（B=4.5 MHz, τ=13 µs → B·τ=58.5，修正量 +17.672 dB）。
  const RirSurfaceClutterInput reference =
      MakeInput(RirVegetationCoverProfile::kOpenGrassland, 5000.0f, 0.0f);
  EXPECT_NEAR(CnrDb(model, reference),
              HandComputedCnrDb(reference, kGrasslandSigma0Db), 1.0e-3);

  // 非默认带宽/脉宽（B=2 MHz, τ=20 µs → B·τ=40）：τ/13µs 参考缩放与
  // max(1, B·τ) 增益同时激活，验证修正公式形状而非单一默认点。
  RirSurfaceClutterInput wide_pulse = reference;
  wide_pulse.transmitter.bandwidth_hz = 2.0e6f;
  wide_pulse.transmitter.pulse_width_s = 20.0e-6f;
  EXPECT_NEAR(CnrDb(model, wide_pulse),
              HandComputedCnrDb(wide_pulse, kGrasslandSigma0Db), 1.0e-3);

  // 压缩增益钳位（B=10 kHz, τ=1 µs → B·τ=0.01 → max(1,·)=1，修正量 0 dB）。
  RirSurfaceClutterInput uncompressed = reference;
  uncompressed.transmitter.bandwidth_hz = 1.0e4f;
  uncompressed.transmitter.pulse_width_s = 1.0e-6f;
  EXPECT_NEAR(CnrDb(model, uncompressed),
              HandComputedCnrDb(uncompressed, kGrasslandSigma0Db), 1.0e-3);
}

/// @brief 档位序：σ₀ 表序 → CNR 严格同序（修复旧模型档位差 <0.1 dB 的失效）。
TEST(RirSurfaceClutterModelTest, ProfileOrderingFollowsSigma0Table) {
  const RirSurfaceClutterModel model;
  const RirVegetationCoverProfile profiles[] = {
      RirVegetationCoverProfile::kOpenGrassland, RirVegetationCoverProfile::kSparseWoodland,
      RirVegetationCoverProfile::kDeciduousForest,
      RirVegetationCoverProfile::kConiferousForest, RirVegetationCoverProfile::kTropicalDense};
  float previous_cnr_db = -1.0e9f;
  for (auto profile : profiles) {
    const float cnr_db =
        CnrDb(model, MakeInput(profile, 5000.0f, -10.0f));
    EXPECT_GT(cnr_db, previous_cnr_db) << "profile=" << static_cast<int>(profile);
    previous_cnr_db = cnr_db;
  }
}

/// @brief 距离响应：脉冲限制区杂波功率随斜距三次方衰减（σ_c∝R，雷达方程 R⁻⁴）。
TEST(RirSurfaceClutterModelTest, PulseLimitedClutterDecaysWithRange) {
  const RirSurfaceClutterModel model;
  const float near = model.EvaluateClutterNoiseW(
      MakeInput(RirVegetationCoverProfile::kSparseWoodland, 2000.0f, -5.0f));
  const float mid = model.EvaluateClutterNoiseW(
      MakeInput(RirVegetationCoverProfile::kSparseWoodland, 5000.0f, -5.0f));
  const float far = model.EvaluateClutterNoiseW(
      MakeInput(RirVegetationCoverProfile::kSparseWoodland, 10000.0f, -5.0f));
  EXPECT_GT(near, mid);
  EXPECT_GT(mid, far);
}

/// @brief 频率响应：同几何下载频升高 → λ² 项使杂波下降（σ₀ 频段常数是声明简化）。
TEST(RirSurfaceClutterModelTest, HigherCarrierLowersClutter) {
  const RirSurfaceClutterModel model;
  RirSurfaceClutterInput s_band = MakeInput(RirVegetationCoverProfile::kDeciduousForest,
                                            5000.0f, 0.0f);
  RirSurfaceClutterInput x_band = s_band;
  x_band.transmitter.frequency_hz = 9.0e9f;
  EXPECT_LT(model.EvaluateClutterNoiseW(x_band), model.EvaluateClutterNoiseW(s_band));
}

/// @brief 退化输入免疫：斜距/热噪/载频/带宽非正一律返回 0。
TEST(RirSurfaceClutterModelTest, DegenerateInputsAreZero) {
  const RirSurfaceClutterModel model;
  RirSurfaceClutterInput base = MakeInput(RirVegetationCoverProfile::kTropicalDense,
                                          5000.0f, 0.0f);
  RirSurfaceClutterInput zero_range = base;
  zero_range.range_m = 0.0f;
  RirSurfaceClutterInput zero_thermal = base;
  zero_thermal.thermal_noise_w = 0.0f;
  RirSurfaceClutterInput zero_carrier = base;
  zero_carrier.transmitter.frequency_hz = 0.0f;
  RirSurfaceClutterInput zero_bandwidth = base;
  zero_bandwidth.transmitter.bandwidth_hz = 0.0f;
  RirSurfaceClutterInput zero_beamwidth = base;
  zero_beamwidth.antenna.nominal_az_beamwidth_deg = 0.0f;
  zero_beamwidth.antenna.nominal_el_beamwidth_deg = 0.0f;
  zero_beamwidth.antenna.antenna_length_m = 0.0f;
  zero_beamwidth.antenna.antenna_width_m = 0.0f;
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(zero_range), 0.0f);
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(zero_thermal), 0.0f);
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(zero_carrier), 0.0f);
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(zero_bandwidth), 0.0f);
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(zero_beamwidth), 0.0f);
}

/// @brief 同输入确定性输出。
TEST(RirSurfaceClutterModelTest, DeterministicAcrossCalls) {
  const RirSurfaceClutterModel model;
  const auto input = MakeInput(RirVegetationCoverProfile::kTropicalDense, 5000.0f, 0.0f);
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(input), model.EvaluateClutterNoiseW(input));
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
