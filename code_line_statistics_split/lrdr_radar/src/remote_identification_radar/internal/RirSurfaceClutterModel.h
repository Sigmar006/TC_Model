/**
 * @file RirSurfaceClutterModel.h
 * @brief RIR 逐目标主瓣地杂波最小物理模型（私有实现头）。
 *
 * 杂波按"σ₀×杂波区面积 + 雷达方程"逐目标求解：响应载频（λ²）、斜距
 * （杂波区雷达方程）与目标俯仰（主瓣下沿擦地角），替代旧的会话级恒定
 * 杂波口径。σ₀ 档位表为 S 波段量级声明值（非实测标定，简化口径）。
 * @note 本文件仅供 RIR 模块内部使用，不作为公开 API。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_SURFACE_CLUTTER_MODEL_H_
#define REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_SURFACE_CLUTTER_MODEL_H_

#include "1q/remote_identification_radar/config/RirEnvironmentConfig.h"
#include "1q/remote_identification_radar/config/RirHardwareConfig.h"

namespace remote_identification_radar {
namespace internal {

/**
 * @brief RirSurfaceClutterInput 表示单目标地杂波求解输入。
 */
struct RirSurfaceClutterInput {
  config::RirVegetationCoverProfile vegetation_cover_profile{
      config::RirVegetationCoverProfile::kDisabled};
  config::hardware::RirTransmitterConfig transmitter{}; /**< 载频/带宽/峰值功率/发射损耗（周期有效值由调用方预填）。 */
  config::hardware::RirAntennaConfig antenna{};         /**< 主瓣增益与波束宽度解析输入。 */

  float propagation_loss_db{0.0f}; /**< 全链路双程传播损耗（天气+植被恒定+大气物理）。 */
  float range_m{0.0f};             /**< 目标斜距（m）。 */
  float look_el_deg{0.0f};         /**< 目标俯仰视线角（deg，向上为正）。 */
  float thermal_noise_w{0.0f};     /**< 接收机热噪功率（W）。 */
};

/**
 * @brief RirSurfaceClutterModel 提供逐目标主瓣地杂波等效噪声求解。
 */
class RirSurfaceClutterModel {
 public:
  /**
   * @brief 计算当前目标几何下的主瓣地杂波等效噪声功率。
   *
   * 物理：擦地角取主瓣俯仰半波束宽减目标俯仰（下限 1°，目标仰角过半波束宽
   * 即主瓣完全离地 → 0）；杂波区面积取脉冲限制与波束限制的较小者，
   * 距离向深度为脉压后距离单元 c/(2B)；σ₀ 随擦地角按 sinψ 一阶折算。
   * 杂波回波在雷达方程之上叠加脉压能量增益 max(1, B·τ)（杂波与目标经同一
   * 匹配滤波，等效噪声按 B·τ 脉压能量基准折算，与检测单元路径同口径）。
   * 非法/退化输入（斜距、热噪、载频、带宽、波束宽度非正）一律返回 0。
   *
   * @param[in] input 单目标求解输入。
   * @return 杂波等效噪声功率（W，相对热噪口径，与 AR 统一换算单源一致）。
   */
  float EvaluateClutterNoiseW(const RirSurfaceClutterInput& input) const;
};

}  // namespace internal
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_SURFACE_CLUTTER_MODEL_H_
