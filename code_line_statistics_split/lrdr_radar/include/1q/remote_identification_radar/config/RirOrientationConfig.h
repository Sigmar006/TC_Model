/**
 * @file RirOrientationConfig.h
 * @brief 远程识别雷达安装指向静态配置（会话第五域）。
 *
 * 阵面相对可扫描体积（az 相对阵面/转台基准，el 绝对，ENU deg）。
 * 运行期转台朝向见 `RirMissionConfig::scan_center_deg`；本域不进
 * `RirRuntimeConfigPatch`。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ORIENTATION_CONFIG_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ORIENTATION_CONFIG_H_

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"

namespace remote_identification_radar {
namespace config {

/** @brief 阵面相对可扫描体积限位（默认 ±60/±30）。 */
using RirOrientationConfig = RirAzimuthElevationLimitsDeg;

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ORIENTATION_CONFIG_H_
