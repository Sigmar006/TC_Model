/**
 * @file RirPropagationModel.h
 * @brief 定义 RIR 环境层传播损耗薄适配模型（私有实现头）。
 *
 * 数值内核为 common 单源（`oneq::common::radar::VegetationClutterModel`），
 * 场景事实类型换 `Rir*`，零 AR 依赖；损耗基线系数与 AR 一致保留在
 * common 实现内部（环境域"不含内部调参项"合约）。
 * 杂波已迁至逐目标物理模型（`RirSurfaceClutterModel`），本层只承载
 * 会话级传播损耗（defer：损耗链物理化留待后续冻结项）。
 * @note 本文件仅供 RIR 模块内部使用，不作为公开 API；周期输入面接线随阶段 2-S。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_PROPAGATION_MODEL_H_
#define REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_PROPAGATION_MODEL_H_

#include "1q/remote_identification_radar/config/RirEnvironmentConfig.h"

namespace remote_identification_radar {
namespace internal {

/**
 * @brief RirPropagationResult 表示传播损耗输出。
 */
struct RirPropagationResult {
  float propagation_loss_db{0.0f}; /**< 传播损耗（单位：dB）。 */
};

/**
 * @brief RirEnvironmentSceneState 当前周期环境场景事实（植被覆盖档位）。
 */
struct RirEnvironmentSceneState {
  config::RirVegetationCoverProfile vegetation_cover_profile{
      config::RirVegetationCoverProfile::kDisabled};
};

/**
 * @brief RirPropagationModel 提供会话级传播损耗建模。
 */
class RirPropagationModel {
 public:
  /**
   * @brief 根据场景状态计算传播损耗。
   * @param[in] scene_state 当前周期场景状态（含植被覆盖档位）。
   * @return 包含传播损耗的 RirPropagationResult。
   * @note 逐目标大气物理损耗由驻留链路预算按真实目标几何计算
   *       （`RirController::ComputeTargetAtmosphericLossDb`，common 大气单源），
   *       环境层不重复计算（避免硬编码几何的死计算）；逐目标地杂波由
   *       `RirSurfaceClutterModel` 按目标几何求解。
   */
  RirPropagationResult Evaluate(const RirEnvironmentSceneState& scene_state) const;
};

}  // namespace internal
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_PROPAGATION_MODEL_H_
