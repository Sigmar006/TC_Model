/**
 * @file RirCycleInput.h
 * @brief 远程识别雷达单周期输入类型。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_INPUT_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_INPUT_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/remote_identification_radar/session/RirSceneTypes.h"

namespace remote_identification_radar {
namespace session {

/**
 * @brief RirCycleInput 描述单周期输入。
 * @note `dt_sec`：本周期步长，用于驻留配额、航迹预测/滤波与关联。
 *       `sim_time_sec`：仿真绝对时刻，用于 RF 窗、识别积累/结论过期与回放时间戳。
 */
struct ONEQ_API RirCycleInput {
  std::uint32_t input_cycle_index{0U}; /**< 本次调用输入周期号（≠0 合法）。 */
  double dt_sec{0.0};                  /**< 周期步长（s），有限且为正。 */
  float sim_time_sec{0.0f};            /**< 当前仿真时间（s）。 */
  oneq::coordinate::EcefPositionM platform_position{}; /**< 平台 ECEF 位置（m，必填）。 */
  RirSceneTargetList scene_targets{}; /**< 场景目标（含识别特征真值与运动事实）。 */
  oneq::electromagnetics::RfSceneFrame rf_scene{}; /**< 外部 RF 场景（仅非本机发射；可空）。 */
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_INPUT_H_
