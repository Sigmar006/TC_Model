/**
 * @file RirPropagationModel.cpp
 * @brief RIR 环境层传播损耗薄适配层（common 单源）。
 */

#include "remote_identification_radar/internal/RirPropagationModel.h"

#include "common/radar/VegetationClutterModel.h"

namespace remote_identification_radar {
namespace internal {

namespace {

oneq::common::radar::VegetationCoverProfile ToCommonProfile(
    config::RirVegetationCoverProfile profile) {
  switch (profile) {
    case config::RirVegetationCoverProfile::kOpenGrassland:
      return oneq::common::radar::VegetationCoverProfile::kOpenGrassland;
    case config::RirVegetationCoverProfile::kSparseWoodland:
      return oneq::common::radar::VegetationCoverProfile::kSparseWoodland;
    case config::RirVegetationCoverProfile::kDeciduousForest:
      return oneq::common::radar::VegetationCoverProfile::kDeciduousForest;
    case config::RirVegetationCoverProfile::kConiferousForest:
      return oneq::common::radar::VegetationCoverProfile::kConiferousForest;
    case config::RirVegetationCoverProfile::kTropicalDense:
      return oneq::common::radar::VegetationCoverProfile::kTropicalDense;
    case config::RirVegetationCoverProfile::kDisabled:
    default:
      return oneq::common::radar::VegetationCoverProfile::kDisabled;
  }
}

oneq::common::radar::VegetationScatterPhysicsConfig ToCommonConfig(
    config::RirVegetationCoverProfile profile) {
  oneq::common::radar::VegetationScatterPhysicsConfig common_config;
  common_config.cover_profile = ToCommonProfile(profile);
  // 档位非 kDisabled 即启用植被物理；与公开配置的单一开关语义对齐。
  common_config.enable_physical_model =
      profile != config::RirVegetationCoverProfile::kDisabled;
  return common_config;
}

}  // namespace

RirPropagationResult RirPropagationModel::Evaluate(
    const RirEnvironmentSceneState& scene_state) const {
  const oneq::common::radar::PropagationClutterResult common_result =
      oneq::common::radar::EvaluatePropagationClutter(
          ToCommonConfig(scene_state.vegetation_cover_profile));
  RirPropagationResult result;
  result.propagation_loss_db = common_result.propagation_loss_db;
  return result;
}

}  // namespace internal
}  // namespace remote_identification_radar
