/**
 * @file RirPropagationModel.cpp
 * @brief RIR 环境层传播损耗薄适配层（common 单源）。
 */

/**
 * ## 过程：传播损耗与环境衰减（LRDR 探测链 · 第 9 步）
 * - 上游：environment.vegetation_cover_profile（植被覆盖档位）；environment.weather_
 *   attenuation_db（天气附加损耗）与逐目标大气物理损耗由 RirController 侧合成。
 * - 本步：植被档位 → 公共核传播/杂波单源求解该档位的传播附加损耗（dB）；档位为
 *   kDisabled 时 RirController 直接跳过调用（不产生该项账本）。
 * - 下游：RirController::ResolveEnvironment 把结果累加进周期传播损耗，叠加逐目标大气损耗
 *   后作为雷达方程（回波/杂波）的传播项。
 * - 关键参数：environment.vegetation_cover_profile（默认 kDisabled）、
 *   environment.weather_attenuation_db（默认 0）、environment.atmospheric_physics.*
 *   （逐目标大气损耗输入，交由公共大气单源计算）。
 * - 对应验收项：第 30 项干扰功率与第 34 项杂波增益的传播项、第 43 项 RCS 实时探测结果
 *   （探测距离的传播修正）、第 62 项目标场景接口。
 */

#include "remote_identification_radar/internal/RirPropagationModel.h"

#include "common/radar/VegetationClutterModel.h"

namespace remote_identification_radar {
namespace internal {

namespace {

// 过程：植被覆盖档位枚举镜像（未知取值按 kDisabled 关闭）。
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

// 过程：档位 → 公共核配置（档位非 kDisabled 即启用植被物理，与单一开关语义对齐）。
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

// 过程：传播损耗求解——植被档位 → 公共核 → 只取传播损耗项（dB）。
RirPropagationResult RirPropagationModel::Evaluate(
    const RirEnvironmentSceneState& scene_state) const {
  const oneq::common::radar::PropagationClutterResult common_result =
      oneq::common::radar::EvaluatePropagationClutter(
          ToCommonConfig(scene_state.vegetation_cover_profile));
  RirPropagationResult result;
  // 过程：仅回传传播损耗（杂波/散射项由 RirSurfaceClutterModel 独立求解，避免同一损耗双算）。
  result.propagation_loss_db = common_result.propagation_loss_db;
  return result;
}

}  // namespace internal
}  // namespace remote_identification_radar
