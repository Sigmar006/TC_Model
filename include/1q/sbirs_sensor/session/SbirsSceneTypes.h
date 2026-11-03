/**
 * @file SbirsSceneTypes.h
 * @brief 定义 SBIRS-inspired 场景目标输入类型。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SCENE_TYPES_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SCENE_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace session {

/** @brief ECEF 坐标系下的三维位置向量，单位 m。 */
struct ONEQ_API SbirsVector3M {
  double x{0.0}; /**< X 分量，单位 m */
  double y{0.0}; /**< Y 分量，单位 m */
  double z{0.0}; /**< Z 分量，单位 m */

  SbirsVector3M() = default;
  SbirsVector3M(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in) {}
};

/**
 * @brief 输入场景中的单个目标描述（仿真真值）。
 * @note 纯数据类型 (POD)。`target_id` 是目标级状态机的键；真值位置仅用于真值辅助跟踪
 *       与仿真判定，不进入 `SbirsOutputFrame` raw output。速度真值用于 cue 延迟外推与
 *       动态滞后误差，缺省（`has_velocity_ecef_m_per_s=false`）时按 0 处理，保持旧行为。
 * @note 目标红外签名由调用方以辐射强度（radiant intensity，W/sr）直接提供：接收功率
 *       P = I · A_ap · τ_opt · τ_atm · η / d²，模块不再持有温度/发射率/投影面积。
 *       默认值 10388.1146 W/sr 等价于旧默认 (1200 K, ε=0.85, A=1 m², 3–5 μm)。
 */
struct ONEQ_API SbirsSceneTarget {
  std::uint64_t target_id{0U};     /**< 目标唯一标识，状态机以此键管理状态 */
  std::string target_name{};       /**< 目标名称，仅进入归属/调试层，不进 raw output */
  SbirsVector3M position_ecef_m{}; /**< ECEF 位置真值，单位 m */
  /**
   * @brief 目标辐射强度（波段内、朝向传感器方向），单位 W/sr。
   *
   * @details 默认值 **500000 W/sr**（2026-11-04 甲方要求统一口径）。原默认 10388.1146 是
   *   "1200 K / ε=0.85 / 1 m² / 3–5 μm"的等效值；装配层（OPIR 传感器）实际一直在用
   *   `kDefaultRadiantIntensityWPerSr = 500000`，两处口径不一致会让"库默认场景"与
   *   "宿主实际场景"的 SNR 差 16.8 dB。此处与装配层对齐。
   */
  double radiant_intensity_w_per_sr{500000.0};
  SbirsVector3M velocity_ecef_m_per_s{}; /**< ECEF 速度真值，单位 m/s（仅 cue 外推与动态滞后用） */
  bool has_velocity_ecef_m_per_s{false}; /**< 是否提供速度；为 false 时数据必须为有限零向量，避免静默忽略矛盾载荷 */
  bool active{true};               /**< 目标是否在场景中有效 */
};

/** @brief 目标列表。 */
using SbirsSceneTargetList = std::vector<SbirsSceneTarget>;

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SCENE_TYPES_H_
