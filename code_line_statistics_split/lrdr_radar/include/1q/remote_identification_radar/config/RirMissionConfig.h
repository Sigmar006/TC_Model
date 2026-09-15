/**
 * @file RirMissionConfig.h
 * @brief 远程识别雷达任务域主配置类型。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_MISSION_CONFIG_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_MISSION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/foundation/scan_schedule_types.h"
#include "1q/remote_identification_radar/config/RirHardwareConfig.h"

namespace remote_identification_radar {
namespace config {

/** @brief RirWorkMode 表示远程识别雷达工作模式。 */
enum class ONEQ_API RirWorkMode {
  kStby = 0,     /**< 待机：不执行识别积累，结论保持至过期。 */
  kIdentify = 1  /**< 识别驻留：对重点航迹执行识别观测与积累。 */
};

/**
 * @brief RirMissionConfig 远程识别雷达任务域配置。
 * @note `scan_window_deg`：任务扫描子窗（作战搜索扇区，deg）；默认无界。
 *       az 相对 scan_center、el 绝对。实际搜索扇区 = 本窗 ∩ orientation；
 *       扫描波位与搜索候选按此裁剪；子窗外、体积内目标仅「指定识别」豁免。
 */
struct ONEQ_API RirMissionConfig {
  RirWorkMode work_mode{RirWorkMode::kStby}; /**< 当前工作模式。 */
  RirAzimuthElevationDeg scan_center_deg{}; /**< 转台当前朝向（ENU az/el）；默认 (0,0)=东水平。 */
  float max_range_m{300000.0f};              /**< 识别任务最大作用距离（m），>0。 */
  float recognition_dwell_sec{0.05f};        /**< 单次识别驻留时间（s），>0。 */
  oneq::foundation::ScanStartPosition scan_start_position{
      oneq::foundation::ScanStartPosition::kLeftTop}; /**< 扫描起始象限。 */
  oneq::foundation::ScanSequence scan_sequence{
      oneq::foundation::ScanSequence::kAzimuthFirst}; /**< 二维扫描推进顺序。 */
  float step_scale{1.0f}; /**< 波位步长系数（波束宽度 × 系数），>0。 */
  RirAzimuthElevationLimitsDeg scan_window_deg{-180.0f, 180.0f, -90.0f, 90.0f}; /**< 任务扫描子窗。 */
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_MISSION_CONFIG_H_
