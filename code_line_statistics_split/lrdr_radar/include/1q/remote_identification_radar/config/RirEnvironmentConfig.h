/**
 * @file RirEnvironmentConfig.h
 * @brief 远程识别雷达环境域主配置类型。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ENVIRONMENT_CONFIG_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ENVIRONMENT_CONFIG_H_

#include "1q/api.hpp"
#include "1q/environment/AtmosphericTypes.h"

namespace remote_identification_radar {
namespace config {

/**
 * @brief RirAtmosphericPhysicsConfig 复用统一环境模块基础气象观测类型。
 * @note k 因子为运行期派生量（`ResolveEffectiveKFactor`），不进配置。
 */
using RirAtmosphericPhysicsConfig = oneq::environment::AtmosphericObservation;

/**
 * @brief 地表植被覆盖档位。
 *
 * 选择档位后自动填写叶片尺寸、介电常数、叶片密度、冠层半径和冠层高度等
 * 植被散射物理参数，影响近地传播路径上的多径散射和杂波估计。
 * `kDisabled` 表示不建模植被散射（关闭该路径）。
 */
enum class ONEQ_API RirVegetationCoverProfile {
  kDisabled = 0,     /**< 不建模植被散射。 */
  kOpenGrassland,    /**< 开阔草地——低矮冠层（0.8 m），少量小叶片。 */
  kSparseWoodland,   /**< 稀疏林地——中等冠层（3 m），中等密度散射体。 */
  kDeciduousForest,  /**< 落叶林——高大冠层（6 m），高密度大叶片。 */
  kConiferousForest, /**< 针叶林——高冠层（8 m），密集细小针叶散射体。 */
  kTropicalDense     /**< 热带密林——高冠层（9 m），最高密度与介电常数。 */
};

/**
 * @brief RirEnvironmentConfig 远程识别雷达环境域配置（场景事实输入）。
 *
 * 环境事实在会话初始化（`RirSessionConfig.environment`）注入；运行期变更经
 * `RirRuntimeConfigPatch.has_environment` 整域覆盖。禁止经周期输入携带。
 */
struct ONEQ_API RirEnvironmentConfig {
  float weather_attenuation_db{0.0f}; /**< 天气附加双程传播损耗（dB），须有限且 ≥0；`0` 表示无。 */
  RirVegetationCoverProfile vegetation_cover_profile{
      RirVegetationCoverProfile::kDisabled}; /**< 场景植被覆盖档位；`kDisabled` 表示关闭。 */
  RirAtmosphericPhysicsConfig atmospheric_physics{}; /**< 场景气象观测输入（气压/温度/相对湿度） */
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ENVIRONMENT_CONFIG_H_
