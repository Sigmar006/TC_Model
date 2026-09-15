/**
 * @file RirRuntimeConfigPatch.h
 * @brief 远程识别雷达运行期配置补丁类型。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_RUNTIME_CONFIG_PATCH_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_RUNTIME_CONFIG_PATCH_H_

#include "1q/api.hpp"
#include "1q/remote_identification_radar/config/RirEnvironmentConfig.h"
#include "1q/remote_identification_radar/config/RirMissionConfig.h"
#include "1q/remote_identification_radar/config/RirPolicyConfig.h"

namespace remote_identification_radar {
namespace config {

/**
 * @brief RirRuntimeConfigPatch 运行期可变参数补丁。
 * @note `TryApplyRuntimeConfig` 暂存，下一成功周期生效；整域先于叶子，叶子优先。
 *       指定识别仅 `kIdentify` 消费；目标 id=0 清除，窗口周期数=0 为无限期。
 */
struct ONEQ_API RirRuntimeConfigPatch {
  bool has_mission{false}; /**< 是否覆盖 mission 整域。 */
  RirMissionConfig mission{}; /**< mission 整域覆盖值。 */

  bool has_work_mode{false}; /**< 是否覆盖工作模式。 */
  RirWorkMode work_mode{RirWorkMode::kStby}; /**< 工作模式叶子值。 */

  bool has_scan_center{false}; /**< 是否覆盖转台朝向。 */
  RirAzimuthElevationDeg scan_center_deg{}; /**< 转台朝向（ENU az/el）。 */

  bool has_policy{false}; /**< 是否覆盖 policy 整域。 */
  RirPolicyConfig policy{}; /**< policy 整域覆盖值。 */

  bool has_environment{false}; /**< 是否覆盖 environment 整域。 */
  RirEnvironmentConfig environment{}; /**< environment 整域覆盖值。 */

  bool has_sensor_enabled{false}; /**< 是否覆盖传感器电源。 */
  bool sensor_enabled{true}; /**< 传感器电源叶子值。 */

  bool has_designated_target_id{false}; /**< 是否覆盖指定识别目标。 */
  std::uint64_t designated_external_target_id{0U}; /**< 指定目标句柄；0=清除。 */

  bool has_designation_duration_cycles{false}; /**< 是否覆盖识别窗口时长。 */
  std::uint32_t designation_duration_cycles{0U}; /**< 限时窗口（周期数）；0=无限期。 */
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_RUNTIME_CONFIG_PATCH_H_
