/**
 * @file EsrCycleInput.h
 * @brief 定义电子侦察模块单周期输入载荷。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_H_

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/electromagnetics/RfScene.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrCycleInput 描述电子侦察单周期输入。
 */
struct ONEQ_API EsrCycleInput {
  std::uint32_t cycle_index{0U};     /**< 当前周期号 */
  double cycle_start_time_s{0.0};    /**< 当前周期绝对 world-time 起点（单位：s）。 */
  float dt_sec{1.0f};                /**< 当前周期步长（单位：s） */
  oneq::coordinate::EcefPositionM platform_position_ecef_m{}; /**< 接收平台 ECEF 位置（m，必填：有限且可定位）。 */
  oneq::coordinate::EcefVelocityMps platform_velocity_ecef_mps{}; /**< 接收平台 ECEF 速度（m/s，必填：有限；零向量合法）。 */
  oneq::coordinate::EulerAnglesDeg platform_attitude_deg{}; /**< 接收设备姿态（局部 yaw/pitch/roll，单位：deg）。 */
  oneq::electromagnetics::RfEmissionFrame rf_emissions{}; /**< 当前周期全部实际 RF 发射。其 envelope（world_cycle_index/window_start_time_s/window_duration_s）须与本周期权威时间一致，含空帧亦须填齐。 */
};

}  // namespace session

}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_H_
