/**
 * @file SbirsCycleInput.h
 * @brief 定义 SBIRS-inspired 单周期输入。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief 欧拉角姿态（Z-Y-X 顺序，yaw/pitch/roll，单位 deg，double 精度）。
 * @note 与 oneq::coordinate::EulerAnglesDeg 同约定（正 pitch = 正仰角）；
 *       会话层自有 POD，沿 SbirsVector3M 惯例。
 */
struct ONEQ_API SbirsEulerAnglesDeg {
  double yaw_deg{0.0};   /**< 偏航角（单位：deg） */
  double pitch_deg{0.0}; /**< 俯仰角（单位：deg） */
  double roll_deg{0.0};  /**< 横滚角（单位：deg） */

  SbirsEulerAnglesDeg() = default;
  SbirsEulerAnglesDeg(double yaw_deg_in, double pitch_deg_in, double roll_deg_in)
      : yaw_deg(yaw_deg_in), pitch_deg(pitch_deg_in), roll_deg(roll_deg_in) {}
};

/**
 * @brief 单周期输入，描述一个仿真步的平台几何与目标场景。
 * @note 纯数据类型 (POD)。卫星位置/速度/姿态均为必填（无可选标志——必填输入
 *       不用 has_xxx 在场模式，缺失语义由值校验承担）：位置用于地球遮挡门控与
 *       辐射/噪声几何；速度（ECEF 零向量合法——如 GEO 卫星）旋入 ECI 后与目标
 *       速度合成相对速度，驱动动态滞后误差、cue 延迟外推与 EKF R 阵；姿态
 *       （Body->ECI，零欧拉合法 = 体轴对齐 ECI）与安装角复合为指向合成链，驱动
 *       WFOV/NFOV 内部光轴几何；`dt_sec` 推进扫描相位。输出 az/el 保持 ECI
 *       极坐标参考，不随姿态/安装变化。
 */
struct ONEQ_API SbirsCycleInput {
  std::uint32_t cycle_index{0U};        /**< 周期序号 */
  float dt_sec{1.0f};                   /**< 本周期步长，单位 s */
  double utc_julian_day{0.0};           /**< UTC 儒略日（JD_UTC，必填且须为有限正数；0 表示未提供）。
                                             ECI 输出参考系需要该时刻的 GMST 完成 ECEF→ECI 旋转 */
  SbirsVector3M satellite_position_ecef_m{};       /**< 卫星 ECEF 位置，单位 m（必填：非有限或零向量即校验拒绝） */
  SbirsVector3M satellite_velocity_ecef_m_per_s{}; /**< 卫星 ECEF 速度，单位 m/s（必填：非有限即校验拒绝；零向量合法，如 GEO） */
  SbirsEulerAnglesDeg satellite_attitude_eci_body_deg{}; /**< 卫星姿态欧拉角（Z-Y-X，Body->ECI，
                                                              单位 deg；必填：非有限即校验拒绝；零欧拉合法） */
  SbirsSceneTargetList scene{};         /**< 目标场景列表 */
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_H_
