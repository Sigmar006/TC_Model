/**
 * @file ArPlatformInput.h
 * @brief AR 平台 ECEF 位姿（CycleInput.platform 与输出反算共用）。
 *
 * 场景目标 ECEF→ENU 请用公共 `oneq::coordinate::TryMakeEnuSceneState` 直填
 * `ArTargetInput`；本结构不是目标适配入口。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_PLATFORM_INPUT_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_PLATFORM_INPUT_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace airborne_radar {
namespace session {

/**
 * @brief 雷达平台世界运动学输入。
 * @note 速度固定为 ECEF；姿态角采用 Body→ENU。
 * @note 雷达安装角在 ArOrientationConfig::mount_angles_deg，不在本结构中。
 */
struct ONEQ_API ArPlatformInput {
  oneq::coordinate::EcefPositionM platform_position_ecef_m{}; /**< 平台位置（ECEF，m） */
  oneq::coordinate::EcefVelocityMps platform_velocity_mps{};  /**< 平台速度（ECEF，m/s） */
  oneq::coordinate::EulerAnglesDeg platform_attitude_deg{};   /**< 平台姿态角（Body→ENU，deg） */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_PLATFORM_INPUT_H_
