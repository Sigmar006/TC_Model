/**
 * @file RirSurfaceClutterModel.h
 * @brief RIR 逐目标地杂波最小物理模型（私有实现头）：**主瓣项 + 副瓣项**。
 *
 * 杂波按"σ₀×杂波区面积 + 雷达方程"逐目标求解：响应载频（λ²）、斜距
 * （杂波区雷达方程）与目标俯仰（主瓣下沿擦地角），替代旧的会话级恒定
 * 杂波口径。σ₀ 档位表为 S 波段量级声明值（非实测标定，简化口径）。
 *
 * 2026-11-06 补**副瓣地杂波**：主瓣离地时（目标仰角 > 半俯仰波束宽）主瓣项恒 0，
 * 但副瓣仍照得到地面，此时副瓣项是唯一杂波源。副瓣按"全向距离环 × 最大副瓣电平"
 * 建模，**只在目标斜距 ≤ 无线电地平线**时计入——超过地平线的地面被地球遮挡，
 * 不存在同距离地面环，否则远距目标会被非物理的全向地面环压死。
 * 两项在功率域相加后作为单元的 `clutter_power_w`，日志不分项。
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
  /**
   * @brief 目标 RCS（m²）；0 = 不建模"目标→地面→雷达"再散射项。
   *
   * @details 再散射是**三次散射**链路：雷达 → 目标 → 地面补片 → 雷达。它与直接地杂波
   *   （雷达→地面→雷达）不同：强度正比于 σ_t·σ_c/(r²·R_g²)，只有目标贴近地面、
   *   且其地面投影仍在雷达视线内（≤ 无线电地平线）时才不可忽略。
   */
  float target_rcs_m2{0.0f};
  /** @brief 目标离地高度（m，绝对高度 = 平台海拔 + 目标相对平台高度）；≤0 不建模再散射。 */
  float target_height_m{0.0f};
  /** @brief 目标地面投影的水平距离（m，雷达站心）；用于判定投影点是否在视线内与补片面积。 */
  float target_ground_range_m{0.0f};
  float platform_altitude_m{0.0f}; /**< 雷达平台海拔高（m，AGL/椭球高口径由调用方保证一致）；
                                        主瓣下沿的地面照射距离由该高度与下沿俯角共同决定
                                        （2026-10-15 补：此前直接用目标斜距，远距离目标会算出
                                        非物理的巨大杂波区面积）。 */
};

/**
 * @brief RirSurfaceClutterModel 提供逐目标主瓣地杂波等效噪声求解。
 */
class RirSurfaceClutterModel {
 public:
  /**
   * @brief 计算当前目标几何下的地杂波等效噪声功率（主瓣项 + 副瓣项）。
   *
   * 物理：擦地角取主瓣俯仰半波束宽减目标俯仰（下限 1°，目标仰角过半波束宽
   * 即主瓣完全离地 → 0）；杂波区面积取脉冲限制与波束限制的较小者，
   * 距离向深度为脉压后距离单元 c/(2B)；σ₀ 随擦地角按 sinψ 一阶折算。
   * 杂波回波在雷达方程之上叠加脉压能量增益 max(1, B·τ)（杂波与目标经同一
   * 匹配滤波，等效噪声按 B·τ 脉压能量基准折算，与检测单元路径同口径）。
   * 副瓣项：全向距离环（周长 × 距离单元）+ 最大副瓣电平（`max_sidelobe_level_db`），
   * 仅当目标斜距 ≤ 无线电地平线（4/3 地球半径口径）时计入；否则地面被地球遮挡，该项为 0。
   * 再散射项：目标把雷达波再散射到其地面投影处、地面补片再回雷达（三次散射）；仅当
   *   `target_rcs_m2 > 0`、目标离地高度 > 0、且投影点水平距离 ≤ 地平线时计入。
   *   等效 RCS σ_eff = σ_t·σ_c·R²/(4π·h_t²·R_g²)，量级远低于直接地杂波。
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
