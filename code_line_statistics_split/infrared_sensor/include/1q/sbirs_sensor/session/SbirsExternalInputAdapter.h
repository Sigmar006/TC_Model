/**
 * @file SbirsExternalInputAdapter.h
 * @brief 定义 SBIRS-inspired 外部输入适配占位入口。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXTERNAL_INPUT_ADAPTER_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXTERNAL_INPUT_ADAPTER_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief 由最小输入集合构造单周期输入的便捷工厂。
 * @param[in] cycle_index 周期序号
 * @param[in] dt_sec 步长，单位 s
 * @param[in] utc_julian_day UTC 儒略日（JD_UTC；ECI 输出参考系必需，缺失即校验拒绝）
 * @param[in] satellite_position_ecef_m 卫星 ECEF 位置，单位 m
 * @param[in] satellite_velocity_ecef_m_per_s 卫星 ECEF 速度，单位 m/s（必填；零向量合法）
 * @param[in] satellite_attitude_eci_body_deg 卫星姿态（Z-Y-X，Body->ECI，单位 deg；
 *            必填；零欧拉合法 = 体轴对齐 ECI）
 * @param[in] scene 目标场景列表
 * @return 已设置平台几何、姿态、UTC 时刻与场景的 `SbirsCycleInput`
 */
ONEQ_API SbirsCycleInput MakeSbirsCycleInput(std::uint32_t cycle_index, float dt_sec,
                                             double utc_julian_day,
                                             const SbirsVector3M& satellite_position_ecef_m,
                                             const SbirsVector3M& satellite_velocity_ecef_m_per_s,
                                             const SbirsEulerAnglesDeg& satellite_attitude_eci_body_deg,
                                             const SbirsSceneTargetList& scene);

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXTERNAL_INPUT_ADAPTER_H_
