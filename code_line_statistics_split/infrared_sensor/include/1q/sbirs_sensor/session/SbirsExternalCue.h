/**
 * @file SbirsExternalCue.h
 * @brief 定义星间 cross-cue（交叉提示）运行时引导消息。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXTERNAL_CUE_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXTERNAL_CUE_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief 星间 cross-cue（交叉提示）引导消息：一颗星宽场发现目标后，把"往这儿看"
 *        递给另一颗星（星→地→星转发），受话星窄场直接转头，不等自家宽场扫描。
 * @note 运行时输入（修订 5，2026-09-01）：经 `SbirsSession::SubmitExternalCue` 在周期之间
 *       注入，不进每帧 `SbirsCycleInput`；回放/快照路径不含该消息。内容为来源星宽场
 *       带误差测角（ECI 极坐标）与带误差距离，不携带目标位置类合成量；受话星按
 *       来源星位置 + 距离 × 视线方向三角化出目标 ECI 位置，再换算本星视线角。
 *       来源星位置给 ECEF（GEO 在 ECEF 恒定），受话星用当周期 GMST 旋入 ECI——
 *       转发延迟秒级的地球转角差在 GEO 半径上约 3 km，折视线角 <0.01°，远小于
 *       NFOV 半角，按可忽略处理。
 */
struct ONEQ_API SbirsExternalCue {
  std::uint64_t target_id{0U};             /**< 被引导的目标 ID（须存在于受话星场景） */
  std::uint32_t source_satellite_entity_id{0U}; /**< 来源星实体/融合源 ID（引导来源标记） */
  SbirsVector3M source_position_ecef_m{};  /**< 来源星 ECEF 位置，单位 m */
  float azimuth_deg{0.0f};                 /**< 来源星宽场量测方位角（ECI 极坐标，带误差），单位 deg */
  float elevation_deg{0.0f};               /**< 来源星宽场量测俯仰角（ECI 极坐标，带误差），单位 deg */
  double range_m{0.0};                     /**< 来源星带误差距离（归属层估计距离），单位 m */
  std::uint32_t cycle_index{0U};           /**< 来源星量测周期号（诊断用，不参与门控） */
};

/**
 * @brief 单周期宽场候选量测（cross-cue 外发数据源，修订 7）。
 * @note 仅归属/调试面：管线在自星宽场候选创建处填充；组件据此构造 cross-cue 递话。
 *       不进验收行；基线（无消费者）行为零变化。
 */
struct ONEQ_API SbirsWideCueMeasurement {
  std::uint64_t target_id{0U};       /**< 候选目标 ID */
  float azimuth_deg{0.0f};           /**< 本星宽场带误差量测方位（ECI 极坐标，deg） */
  float elevation_deg{0.0f};         /**< 本星宽场带误差量测俯仰（ECI 极坐标，deg） */
  double measured_range_m{0.0};      /**< 本星带误差距离（误差模型，m） */
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXTERNAL_CUE_H_
