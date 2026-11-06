/**
 * @file SbirsOrbitConfig.h
 * @brief 定义 SBIRS-inspired 卫星轨道传播配置。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_ORBIT_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_ORBIT_CONFIG_H_

#include "1q/api.hpp"
#include "1q/satellite_orbit/OrbitPropagator.h"

namespace sbirs_sensor {
namespace config {

/**
 * @brief 卫星轨道传播配置（设计说明 5.1.1「卫星轨道传播」）。
 *
 * `orbital_elements` 与 `epoch_utc_julian_day` 即设计说明中的 `orbitalElements`
 * 与 `epochTime`，为 `satellite_orbit::OrbitPropagator` 提供初始基准；启用后会话
 * 持有传播器，调用方逐周期推进并把 ECEF 状态填入 `SbirsCycleInput` 的卫星字段。
 * 默认关闭：既有配置的卫星状态仍由调用方逐周期提供，行为不变。
 */
struct ONEQ_API SbirsOrbitConfig {
  /** 是否由会话驱动轨道传播；false 时传播器不存在，卫星状态由调用方提供。 */
  bool propagation_enabled{false};
  /** 轨道六根数（a, e, i, Ω, ω, M），角度单位 deg。 */
  satellite_orbit::OrbitalElements orbital_elements{};
  /** 历元 UTC 儒略日；启用时须为有限正数。 */
  double epoch_utc_julian_day{0.0};
  /** RK4 积分步长，单位 s；须为正有限值，低于模块正下限时按模块下限钳制。 */
  double propagation_step_sec{10.0};
  /** J2 摄动开关；false 时仅保留二体中心引力。 */
  bool j2_enabled{true};
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_ORBIT_CONFIG_H_
