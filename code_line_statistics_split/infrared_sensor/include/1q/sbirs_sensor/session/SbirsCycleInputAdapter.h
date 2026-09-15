/**
 * @file SbirsCycleInputAdapter.h
 * @brief 定义 SBIRS-inspired 输入构造辅助。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_ADAPTER_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_ADAPTER_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief 单周期输入构造器，按链式调用累积各字段后构造 `SbirsCycleInput`。
 * @note 未显式设置的字段保留默认值；`AddTarget` 可多次调用以累积多个目标。
 */
class ONEQ_API SbirsCycleInputBuilder {
 public:
  /**
   * @brief 设置周期序号。
   * @param[in] cycle_index 周期序号
   * @return 自身引用，支持链式调用
   */
  SbirsCycleInputBuilder& WithCycleIndex(std::uint32_t cycle_index);
  /**
   * @brief 设置本周期步长。
   * @param[in] dt_sec 步长，单位 s
   * @return 自身引用，支持链式调用
   */
  SbirsCycleInputBuilder& WithDeltaTimeSec(float dt_sec);
  /**
   * @brief 设置卫星 ECEF 位置（必填：非有限或零向量即校验拒绝）。
   * @param[in] position_ecef_m 卫星位置，单位 m
   * @return 自身引用，支持链式调用
   */
  SbirsCycleInputBuilder& WithSatellitePosition(const SbirsVector3M& position_ecef_m);
  /**
   * @brief 设置卫星 ECEF 速度（必填：非有限即校验拒绝；零向量合法，如 GEO 卫星）。
   * @param[in] velocity_ecef_m_per_s 卫星速度，单位 m/s
   * @return 自身引用，支持链式调用
   */
  SbirsCycleInputBuilder& WithSatelliteVelocity(const SbirsVector3M& velocity_ecef_m_per_s);
  /**
   * @brief 设置卫星姿态欧拉角（必填：非有限即校验拒绝；零欧拉合法 = 体轴对齐 ECI）。
   * @param[in] attitude_eci_body_deg 卫星姿态（Z-Y-X，Body->ECI，单位 deg）
   * @return 自身引用，支持链式调用
   */
  SbirsCycleInputBuilder& WithSatelliteAttitude(
      const SbirsEulerAnglesDeg& attitude_eci_body_deg);
  /**
   * @brief 设置 UTC 儒略日（ECI 输出参考系必需；未设置时校验拒绝）。
   * @param[in] utc_julian_day UTC 儒略日（JD_UTC）
   * @return 自身引用，支持链式调用
   */
  SbirsCycleInputBuilder& WithUtcJulianDay(double utc_julian_day);
  /**
   * @brief 追加一个目标到场景列表。
   * @param[in] target 场景目标
   * @return 自身引用，支持链式调用
   */
  SbirsCycleInputBuilder& AddTarget(const SbirsSceneTarget& target);
  /**
   * @brief 构造并返回累积的单周期输入。
   * @return 已设置字段的 `SbirsCycleInput`
   */
  SbirsCycleInput Build() const;

 private:
  SbirsCycleInput input_{};
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_ADAPTER_H_
