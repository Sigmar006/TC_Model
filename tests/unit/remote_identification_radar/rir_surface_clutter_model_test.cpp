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

/// @brief 副瓣杂波几何输入：平台 300 m（无线电地平线约 71.4 km）、4° 波束。
/// @note  既有用例不填 platform_altitude_m，主瓣项走 h=0 退化分支；副瓣项必须显式给高度，
///        否则地平线退化成约 4 km，测试几何不成立。
RirSurfaceClutterInput MakeSidelobeInput(float range_m, float look_el_deg,
                                         float sidelobe_level_db) {
  RirSurfaceClutterInput input = MakeInput(RirVegetationCoverProfile::kOpenGrassland, range_m,
                                           look_el_deg);
  input.platform_altitude_m = 300.0f;
  input.antenna.max_sidelobe_level_db = sidelobe_level_db;
  return input;
}

/// @brief 主瓣离地（目标仰角 > 半波束宽）且在地平线内时，副瓣项**单独**给出杂波。
TEST(RirSurfaceClutterModelTest, SidelobeClutterExistsWhenMainlobeIsOffGround) {
  const RirSurfaceClutterModel model;
  const float clutter_w = model.EvaluateClutterNoiseW(MakeSidelobeInput(50000.0f, 5.0f, -20.0f));
  EXPECT_GT(clutter_w, 0.0f) << "主瓣离地后副瓣照地仍应产生杂波";
  // 主瓣项为 0 的佐证：把副瓣压到 -300 dB（等效关闭）后杂波降到可忽略——
  // 注意写出通道有 CNR 下限钳位（-120 dB），故不是精确 0，而是热噪声的 1e-12 量级。
  EXPECT_LT(model.EvaluateClutterNoiseW(MakeSidelobeInput(50000.0f, 5.0f, -300.0f)), 1.0e-20f);
}

/// @brief 超出无线电地平线（约 71.4 km）后副瓣项归零：地面被地球遮挡。
TEST(RirSurfaceClutterModelTest, SidelobeClutterVanishesBeyondRadioHorizon) {
  const RirSurfaceClutterModel model;
  EXPECT_GT(model.EvaluateClutterNoiseW(MakeSidelobeInput(70000.0f, 5.0f, -20.0f)), 0.0f);
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(MakeSidelobeInput(80000.0f, 5.0f, -20.0f)), 0.0f);
  // 弹道目标（4,300 km）同仰角不应挂上"同斜距全向地面环"。
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(MakeSidelobeInput(4300000.0f, 26.0f, -20.0f)), 0.0f);
}

/// @brief 副瓣电平每降 10 dB（单程），杂波功率降 20 dB（双程）——比值固定 100。
TEST(RirSurfaceClutterModelTest, SidelobeClutterScalesWithTwoWaySidelobeLevel) {
  const RirSurfaceClutterModel model;
  const float at_minus_30 = model.EvaluateClutterNoiseW(MakeSidelobeInput(50000.0f, 5.0f, -30.0f));
  const float at_minus_40 = model.EvaluateClutterNoiseW(MakeSidelobeInput(50000.0f, 5.0f, -40.0f));
  ASSERT_GT(at_minus_40, 0.0f);
  EXPECT_NEAR(at_minus_30 / at_minus_40, 100.0f, 1.0f);
}

/// @brief 主瓣判据（严格几何）：只有波束下沿压到"目标距离处地面点"以下才算主瓣杂波。
///        平台 300 m、0.8° 波束（半宽 0.4°）：
///        φ = −1.0° 时下沿 −1.4°，20~71 km 处地面俯角 0.5°~0.9° 都在其内 ⇒ 有杂波；
///        φ = +0.3° 时下沿 −0.1°，地平线方向都在 −0.48° ⇒ 一片地面也照不到 ⇒ 0。
TEST(RirSurfaceClutterModelTest, MainlobeClutterRequiresBeamEdgeBelowGroundPoint) {
  const RirSurfaceClutterModel model;
  // 波束压到地平线以下：主瓣照地（副瓣关掉以隔离主瓣项）。
  EXPECT_GT(model.EvaluateClutterNoiseW(MakeSidelobeInput(40000.0f, -1.0f, -300.0f)), 0.0f);
  EXPECT_GT(model.EvaluateClutterNoiseW(MakeSidelobeInput(71000.0f, -1.0f, -300.0f)), 0.0f);
  // 波束下沿仍在地平线之上（夹具为 4° 波束 ⇒ 半宽 2°，φ=2.0 时下沿恰在水平线）：
  // 主瓣一片地面也照不到。
  EXPECT_LT(model.EvaluateClutterNoiseW(MakeSidelobeInput(40000.0f, 2.0f, -300.0f)), 1.0e-20f);
  EXPECT_LT(model.EvaluateClutterNoiseW(MakeSidelobeInput(71000.0f, 2.0f, -300.0f)), 1.0e-20f);
  // 超出无线电地平线：该距离处没有地面，主瓣项恒 0（哪怕波束压得很低）。
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(MakeSidelobeInput(100000.0f, -1.0f, -300.0f)), 0.0f);
}

/// @brief 副瓣杂波几何输入：平台 300 m（无线电地平线约 71.4 km）、4° 波束。
/// @note  既有用例不填 platform_altitude_m，主瓣项走 h=0 退化分支；副瓣项必须显式给高度，
///        否则地平线退化成约 4 km，测试几何不成立。
/// @brief 再散射项（A：雷达→目标→地面→雷达）：只有近距低空目标才有，且正比于目标 RCS。
/// @note  远程弹道目标的地面投影在 3,868 km（远超 71.4 km 地平线）⇒ 该项为 0，
///        因为那条回波路径被地球挡住——这正是"远距目标没有这项杂波"的物理原因。
TEST(RirSurfaceClutterModelTest, TargetGroundRescatterNeedsNearLowTarget) {
  const RirSurfaceClutterModel model;

  // φ=2.0°：4° 波束的下沿恰在水平线 ⇒ 主瓣离地，杂波只剩再散射项（便于隔离）。
  RirSurfaceClutterInput near_low = MakeSidelobeInput(20000.0f, 2.0f, -300.0f);
  near_low.target_rcs_m2 = 0.1f;
  near_low.target_height_m = 400.0f;        // 平台 300 m + 目标相对高度 100 m
  near_low.target_ground_range_m = 20000.0f;
  const float rescatter_only = model.EvaluateClutterNoiseW(near_low);
  EXPECT_GT(rescatter_only, 0.0f) << "近距低空目标应有三阶再散射项";

  // RCS 翻 10 倍 ⇒ 该项功率随之 ×10。
  RirSurfaceClutterInput stronger = near_low;
  stronger.target_rcs_m2 = 1.0f;
  EXPECT_NEAR(model.EvaluateClutterNoiseW(stronger) / rescatter_only, 10.0f, 1.0f);

  // 远程弹道目标：投影 3,868 km 远超地平线 ⇒ 再散射为 0。
  RirSurfaceClutterInput far_high = MakeSidelobeInput(4324000.0f, 26.6f, -300.0f);
  far_high.target_rcs_m2 = 0.1f;
  far_high.target_height_m = 2237900.0f;
  far_high.target_ground_range_m = 3868000.0f;
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(far_high), 0.0f);

  // 目标高度低于 30 m 门槛（几乎贴地，三次散射退化为二阶）⇒ 不计（只剩 CNR 下限钳位量级）。
  RirSurfaceClutterInput too_low = near_low;
  too_low.target_height_m = 20.0f;
  EXPECT_LT(model.EvaluateClutterNoiseW(too_low), 1.0e-20f);
}

/// @brief 多重散射 + 起伏：均值由几何定（一次散射 × σ₀/4π），逐周期在 dB 域浮动 ±3 dB。
/// @note  起伏落在**整个杂波项**上（甲方口径：给一个能到达雷达的数值 + 随机范围）；
///        没有地面回波的单元（远距目标）起伏后仍为 0，不会被抬成底噪。
TEST(RirSurfaceClutterModelTest, ClutterFluctuationScalesWholeTerm) {
  const RirSurfaceClutterModel model;
  RirSurfaceClutterInput base = MakeSidelobeInput(20000.0f, 0.3f, -20.0f);
  base.clutter_fluctuation_db = 0.0f;
  const float nominal = model.EvaluateClutterNoiseW(base);
  ASSERT_GT(nominal, 0.0f);

  RirSurfaceClutterInput up = base;
  up.clutter_fluctuation_db = 3.0f;
  RirSurfaceClutterInput down = base;
  down.clutter_fluctuation_db = -3.0f;
  // ±3 dB ⇒ 功率 ×1.995 / ×0.501（只测起伏增益，多重散射的 −39 dB 结构项可忽略）。
  EXPECT_NEAR(model.EvaluateClutterNoiseW(up) / nominal, 1.9953f, 0.01f);
  EXPECT_NEAR(model.EvaluateClutterNoiseW(down) / nominal, 0.5012f, 0.01f);

  // 远距弹道目标（该单元无地面）：起伏后仍为 0。
  RirSurfaceClutterInput far_target = MakeSidelobeInput(4324000.0f, 26.6f, -20.0f);
  far_target.clutter_fluctuation_db = 3.0f;
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(far_target), 0.0f);
}

/// @brief 弥散散射底（工程兜底）：开启后任何单元都有非零杂波，且同样带 ±3 dB 起伏。
/// @note  关掉（`diffuse_floor_w = 0`）即回到"纯物理：无地面即 0"。
TEST(RirSurfaceClutterModelTest, DiffuseFloorGivesEveryCellAClutterValue) {
  const RirSurfaceClutterModel model;
  RirSurfaceClutterInput far_target = MakeSidelobeInput(4324000.0f, 26.6f, -20.0f);

  // 关：无地面 ⇒ 0。
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(far_target), 0.0f);

  // 开：恒等于底值（起伏 0 dB）。
  far_target.diffuse_floor_w = 1.0e-15f;
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(far_target), 1.0e-15f);

  // 起伏 +3 dB ⇒ 底值 ×1.995。
  far_target.clutter_fluctuation_db = 3.0f;
  EXPECT_NEAR(model.EvaluateClutterNoiseW(far_target) / 1.0e-15f, 1.9953f, 0.01f);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
