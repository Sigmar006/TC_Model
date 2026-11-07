/**
 * @file VegetationClutterModel.cpp
 * @brief 实现植被散射杂波与最小传播损耗组合模型（common 单源）。
 */

#include "common/radar/VegetationClutterModel.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "common/numerics/ClampUtils.h"
#include "common/rcs/RcsPhysics.h"
#include "common/numerics/Constants.h"

namespace oneq {
namespace common {
namespace radar {

namespace {
using oneq::common::numerics::RadToDeg;

// 中译：**传播损耗常数回归物理**（2026-11-08）——原三项常数 4.0 + 1.5 + 1.0 = 6.5 dB，
//   其中 4.0（"基础传播损耗"）与 1.0（"地形反射"）既无几何依赖、也无依据，且对 5,000 km
//   掠地路径与 50 km 近距离一视同仁地扣 6.5 dB，把雷达作用距离整体压低约 1.5 倍
//   （现场第 36 项 Pd=90% 距离被压到 3,938 km）。现只保留 1.5 dB 大气项，
//   其余两项置 0：合计 1.5 dB，与"S 波段低仰角单程大气衰减"量级一致。
//   说明：植被档位仍只影响杂波项，不影响这 1.5 dB（与改前一致）。
// 标识：植被传播损耗常数修正（2026-11-08）。
constexpr float kInternalBasePropagationLossDb = 0.5f;
constexpr float kInternalAtmosphericAttenuationDb = 1.5f;
constexpr float kInternalTerrainReflectionDb = 0.0f;
constexpr float kInternalBaselineClutterPowerDb = 3.0f;

float DbToLinear(float db_value) { return std::pow(10.0f, db_value / 10.0f); }

float LinearToDb(float linear_value) {
  const float safe_linear_value = std::max(linear_value, 1.0e-12f);
  return 10.0f * std::log10(safe_linear_value);
}

struct VegetationPhysicsParameters {
  float leaf_size_m{0.0f};
  float dielectric_constant_real{1.0f};
  std::size_t leaf_count{0U};
  float canopy_radius_m{1.0f};
  float canopy_height_m{1.0f};
};

VegetationPhysicsParameters ResolveVegetationPhysicsParameters(VegetationCoverProfile profile) {
  VegetationPhysicsParameters parameters;
  switch (profile) {
    case VegetationCoverProfile::kOpenGrassland:
      parameters.leaf_size_m = 0.02f;
      parameters.dielectric_constant_real = 1.8f;
      parameters.leaf_count = 24U;
      parameters.canopy_radius_m = 1.5f;
      parameters.canopy_height_m = 0.8f;
      break;
    case VegetationCoverProfile::kSparseWoodland:
      parameters.leaf_size_m = 0.04f;
      parameters.dielectric_constant_real = 2.2f;
      parameters.leaf_count = 56U;
      parameters.canopy_radius_m = 2.8f;
      parameters.canopy_height_m = 3.0f;
      break;
    case VegetationCoverProfile::kDeciduousForest:
      parameters.leaf_size_m = 0.05f;
      parameters.dielectric_constant_real = 2.6f;
      parameters.leaf_count = 96U;
      parameters.canopy_radius_m = 4.0f;
      parameters.canopy_height_m = 6.0f;
      break;
    case VegetationCoverProfile::kConiferousForest:
      parameters.leaf_size_m = 0.035f;
      parameters.dielectric_constant_real = 2.4f;
      parameters.leaf_count = 120U;
      parameters.canopy_radius_m = 3.2f;
      parameters.canopy_height_m = 8.0f;
      break;
    case VegetationCoverProfile::kTropicalDense:
      parameters.leaf_size_m = 0.065f;
      parameters.dielectric_constant_real = 3.1f;
      parameters.leaf_count = 180U;
      parameters.canopy_radius_m = 4.8f;
      parameters.canopy_height_m = 9.0f;
      break;
    case VegetationCoverProfile::kDisabled:
    default:
      break;
  }
  return parameters;
}

float ResolveVegetationIncidenceDeg(const VegetationPhysicsParameters& parameters) {
  const float canopy_slope_rad =
      std::atan2(std::max(parameters.canopy_height_m, 0.1f),
                 std::max(parameters.canopy_radius_m, 0.1f));
  const float incidence_deg = 8.0f + RadToDeg(canopy_slope_rad) * 0.5f;
  return oneq::common::numerics::Clamp(incidence_deg, 8.0f, 75.0f);
}

float ResolveVegetationScatterDeg(const VegetationPhysicsParameters& parameters,
                                  float incidence_deg) {
  const float canopy_aspect_ratio = std::max(parameters.canopy_height_m, 0.1f) /
                                    std::max(parameters.canopy_radius_m, 0.1f);
  const float anisotropy = std::fabs(canopy_aspect_ratio - 1.0f);
  const float scatter_deg =
      incidence_deg + 12.0f + 6.0f * oneq::common::numerics::Clamp(anisotropy, 0.0f, 2.0f);
  return oneq::common::numerics::Clamp(scatter_deg, 15.0f, 85.0f);
}

float ComputeVegetationClutterMultiplier(const VegetationScatterPhysicsConfig& config) {
  const VegetationPhysicsParameters parameters =
      ResolveVegetationPhysicsParameters(config.cover_profile);
  oneq::common::rcs::TreeScattererConfig scatterer_config;
  scatterer_config.leaf_count = std::max(parameters.leaf_count, std::size_t(1U));
  scatterer_config.canopy_radius_m = std::max(parameters.canopy_radius_m, 0.1f);
  scatterer_config.canopy_height_m = std::max(parameters.canopy_height_m, 0.1f);
  const oneq::common::rcs::TreeScattererState scatterer_state =
      oneq::common::rcs::InitializeTreeScatterer(scatterer_config);

  std::vector<float> x_param;
  std::vector<float> y_param;
  oneq::common::rcs::ComputeLeavesParametricEquation(
      scatterer_state, 0.8f, 0.5f,
      &x_param, &y_param);

  const float incidence_deg = ResolveVegetationIncidenceDeg(parameters);
  const float scatter_deg = ResolveVegetationScatterDeg(parameters, incidence_deg);
  const oneq::common::rcs::LeafPhaseMatrices phase_matrices =
      oneq::common::rcs::compute_leaf_phase_matrices(
          std::max(parameters.leaf_size_m, 0.0f),
          std::max(parameters.dielectric_constant_real, 1.0f),
          incidence_deg, scatter_deg);

  float average_extent = 0.0f;
  for (std::size_t index = 0U; index < x_param.size(); ++index) {
    average_extent += std::sqrt(x_param[index] * x_param[index] + y_param[index] * y_param[index]);
  }
  if (!x_param.empty()) {
    average_extent /= static_cast<float>(x_param.size());
  }

  const float normalized_extent = average_extent / scatterer_config.canopy_radius_m;
  const float phase_energy =
      std::max(0.0f, phase_matrices.m11 + phase_matrices.m22 +
                         std::fabs(phase_matrices.m12) + std::fabs(phase_matrices.m21));
  const float density_term =
      std::log1p(static_cast<float>(scatterer_config.leaf_count)) * (1.0f + normalized_extent);

  const float unclamped_multiplier = 1.0f + 0.05f * phase_energy * density_term;
  return oneq::common::numerics::Clamp(unclamped_multiplier, 1.0f, 30.0f);
}

}  // namespace

PropagationClutterResult EvaluatePropagationClutter(
    const VegetationScatterPhysicsConfig& config) {
  PropagationClutterResult result;
  result.propagation_loss_db = kInternalBasePropagationLossDb +
                               kInternalAtmosphericAttenuationDb +
                               kInternalTerrainReflectionDb;
  float clutter_power_db = kInternalBaselineClutterPowerDb;
  if (config.enable_physical_model) {
    const float mix_ratio = 0.7f;
    if (mix_ratio > 0.0f) {
      const float baseline_clutter_w = DbToLinear(clutter_power_db);
      const float physical_multiplier = ComputeVegetationClutterMultiplier(config);
      const float physical_clutter_w = baseline_clutter_w * physical_multiplier;
      const float mixed_clutter_w =
          baseline_clutter_w * (1.0f - mix_ratio) + physical_clutter_w * mix_ratio;
      clutter_power_db = LinearToDb(mixed_clutter_w);
    }
  }
  result.clutter_power_db = clutter_power_db;
  return result;
}

}  // namespace radar
}  // namespace common
}  // namespace oneq
