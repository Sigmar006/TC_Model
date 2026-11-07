/**
 * @file common_vegetation_clutter_model_test.cpp
 * @brief 验证 common 植被散射杂波与最小传播损耗组合模型。
 */

#include <gtest/gtest.h>

#include <cmath>

#include "common/radar/VegetationClutterModel.h"

namespace oneq {
namespace common {
namespace radar {
namespace {

VegetationScatterPhysicsConfig MakeConfig(VegetationCoverProfile profile,
                                          bool enable_physical) {
  VegetationScatterPhysicsConfig config;
  config.cover_profile = profile;
  config.enable_physical_model = enable_physical;
  return config;
}

TEST(CommonVegetationClutterModelTest, DisabledVegetationUsesBaselines) {
  const auto result = EvaluatePropagationClutter(
      MakeConfig(VegetationCoverProfile::kDisabled, false));
  EXPECT_NEAR(result.propagation_loss_db, 2.0f, 1.0e-4f);
  EXPECT_NEAR(result.clutter_power_db, 3.0f, 1.0e-4f);
}

TEST(CommonVegetationClutterModelTest, EnabledVegetationRaisesClutterAboveBaseline) {
  const VegetationCoverProfile profiles[] = {
      VegetationCoverProfile::kOpenGrassland, VegetationCoverProfile::kSparseWoodland,
      VegetationCoverProfile::kDeciduousForest, VegetationCoverProfile::kConiferousForest,
      VegetationCoverProfile::kTropicalDense};
  for (const auto profile : profiles) {
    const auto result = EvaluatePropagationClutter(MakeConfig(profile, true));
    EXPECT_GE(result.clutter_power_db, 3.0f) << "profile=" << static_cast<int>(profile);
    EXPECT_NEAR(result.propagation_loss_db, 2.0f, 1.0e-4f);
  }
}

TEST(CommonVegetationClutterModelTest, DeterministicAcrossCalls) {
  const auto config = MakeConfig(VegetationCoverProfile::kDeciduousForest, true);
  const auto first = EvaluatePropagationClutter(config);
  const auto second = EvaluatePropagationClutter(config);
  EXPECT_FLOAT_EQ(first.propagation_loss_db, second.propagation_loss_db);
  EXPECT_FLOAT_EQ(first.clutter_power_db, second.clutter_power_db);
}

TEST(CommonVegetationClutterModelTest, DenseProfileExceedsGrassland) {
  const auto grass = EvaluatePropagationClutter(
      MakeConfig(VegetationCoverProfile::kOpenGrassland, true));
  const auto dense = EvaluatePropagationClutter(
      MakeConfig(VegetationCoverProfile::kTropicalDense, true));
  EXPECT_GT(dense.clutter_power_db, grass.clutter_power_db);
}

TEST(CommonVegetationClutterModelTest, EquivalentClutterNoiseScalesThermalNoise) {
  // 3 dB 基线 = 2 倍热噪（相对 dB / CNR 口径，不是绝对 dBW）。
  EXPECT_NEAR(ComputeEquivalentClutterNoiseW(1.0e-14f, 3.0f), 2.0e-14f, 1.0e-15f);
  EXPECT_FLOAT_EQ(ComputeEquivalentClutterNoiseW(5.0e-14f, 10.0f), 5.0e-13f);
}

TEST(CommonVegetationClutterModelTest, EquivalentClutterNoiseClampsAndRejectsInvalidInput) {
  // 相对 dB 钳制到 [-120, +120]。
  EXPECT_FLOAT_EQ(ComputeEquivalentClutterNoiseW(1.0e-14f, 1000.0f),
                  1.0e-14f * std::pow(10.0f, 120.0f / 10.0f));
  EXPECT_FLOAT_EQ(ComputeEquivalentClutterNoiseW(1.0e-14f, -1000.0f),
                  1.0e-14f * std::pow(10.0f, -120.0f / 10.0f));
  // 非有限杂波/热噪与非正热噪返回 0。
  EXPECT_FLOAT_EQ(ComputeEquivalentClutterNoiseW(1.0e-14f, std::nanf("")), 0.0f);
  EXPECT_FLOAT_EQ(ComputeEquivalentClutterNoiseW(std::nanf(""), 3.0f), 0.0f);
  EXPECT_FLOAT_EQ(ComputeEquivalentClutterNoiseW(0.0f, 3.0f), 0.0f);
  EXPECT_FLOAT_EQ(ComputeEquivalentClutterNoiseW(-1.0f, 3.0f), 0.0f);
}

}  // namespace
}  // namespace radar
}  // namespace common
}  // namespace oneq
