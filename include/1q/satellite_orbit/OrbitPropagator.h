/**
 * @file OrbitPropagator.h
 * @brief 定义 OPIR 卫星轨道传播器（二体引力 + J2 摄动、RK4 积分、ECI→ECEF 输出）。
 *
 * 设计依据：《探测仿真系统软件设计说明》5.1.1「模拟 OPIR 轨道运动能力」——
 * 轨道传播由 OrbitPropagator 类承担，由轨道六根数与历元推进卫星状态，逐周期
 * 输出 ECEF 位置与速度供周期输入使用。
 *
 * 约定与近似（效能级）：
 * - 动力学仅含二体中心引力与可选 J2 项；不建模高阶重力场、日月引力、太阳光压
 *   与大气阻力。
 * - 积分采用经典四阶龙格-库塔（RK4），步长由调用方给定并设有正下限。
 * - 输出 ECEF 时按 IAU 1982 GMST 近似旋转（UT1 ≈ UTC），不建模章动与极移；
 *   速度输出补偿地球自转输运项，与 1q/coordinate/inertial_transform.h 同约定。
 * - 不抛出 C++ 异常：非法输入返回 false，失败时不修改已持有的传播状态。
 */

#ifndef ONEQ_SATELLITE_ORBIT_ORBIT_PROPAGATOR_H_
#define ONEQ_SATELLITE_ORBIT_ORBIT_PROPAGATOR_H_

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace satellite_orbit {

/**
 * @brief 轨道六根数（经典开普勒根数）。
 * @note 角度均为 deg；平近点角 M 可超出 [0, 360)，初始化时按 2π 归一化。
 */
struct ONEQ_API OrbitalElements {
  double semi_major_axis_m{0.0};          /**< 半长轴 a，单位 m（须为有限正数） */
  double eccentricity{0.0};               /**< 偏心率 e，范围 [0, 1)（仅椭球轨道） */
  double inclination_deg{0.0};            /**< 轨道倾角 i，范围 [0, 180]，单位 deg */
  double raan_deg{0.0};                   /**< 升交点赤经 Ω，单位 deg */
  double argument_of_perigee_deg{0.0};    /**< 近地点幅角 ω，单位 deg */
  double mean_anomaly_deg{0.0};           /**< 历元平近点角 M，单位 deg */
};

/** @brief 惯性系（ECI，J2000 平赤道面）位置与速度状态。 */
struct ONEQ_API EciState {
  oneq::coordinate::EciPositionM position{};   /**< ECI 位置，单位 m */
  oneq::coordinate::EciVelocityMps velocity{}; /**< ECI 速度，单位 m/s */
};

/** @brief 地固系（ECEF）位置与速度状态，用于注入周期输入的卫星状态字段。 */
struct ONEQ_API EcefState {
  oneq::coordinate::EcefPositionM position{};   /**< ECEF 位置，单位 m */
  oneq::coordinate::EcefVelocityMps velocity{}; /**< ECEF 速度，单位 m/s */
};

/**
 * @brief OPIR 卫星轨道传播器：轨道六根数 + J2 摄动，经 RK4 逐周期推进。
 * @details 历元与当前积分状态跨周期保持；`Initialize` 重置历元状态，
 *          `PropagateToTime` 从上次传播时刻推进到目标 UTC 儒略日。同一输入序列
 *          产生确定性的 ECEF 状态。
 * @note 目标时刻早于上次传播时刻视为非法（不执行反向积分），返回 false 且保持
 *       当前状态不变。
 */
class ONEQ_API OrbitPropagator {
 public:
  OrbitPropagator();

  /**
   * @brief 由轨道六根数与历元计算 ECI 初态，并置为当前传播状态。
   * @param[in] elements 轨道六根数
   * @param[in] epoch_jd 历元 UTC 儒略日（须为有限正数）
   * @return 根数与历元合法时为 true；失败时不修改已有状态
   */
  bool Initialize(const OrbitalElements& elements, double epoch_jd);

  /**
   * @brief 从上次传播时刻以配置步长执行 RK4 积分，推进至目标 UTC 儒略日。
   * @param[in] target_jd 目标 UTC 儒略日（须不小于上次传播时刻）
   * @param[out] ecef_state 目标时刻的 ECEF 位置与速度；失败时不修改
   * @return 积分与坐标转换成功时为 true
   * @note 积分步长取 `step_size_sec()` 与剩余时长的较小值；输出速度含地球自转
   *       输运项补偿，与 TryEcefVelocityToEci 互逆。
   */
  bool PropagateToTime(double target_jd, EcefState* ecef_state);

  /**
   * @brief 设置积分步长（秒），非有限值被忽略、低于正下限时钳制。
   * @param[in] step_size_s 积分步长，单位 s
   * @note 已积分的当前状态不受影响，下一周期起按新步长推进。
   */
  void SetStepSize(double step_size_s);

  /**
   * @brief 启用或关闭 J2 摄动项。
   * @param[in] enable true 启用 J2，false 仅保留二体引力
   * @note 下一周期起生效，已积分的当前状态不回溯重算。
   */
  void SetJ2Enabled(bool enable);

  /** @return 是否已成功初始化。 */
  bool initialized() const { return initialized_; }

  /** @return 当前轨道六根数（未初始化时为默认值）。 */
  const OrbitalElements& elements() const { return elements_; }

  /** @return 历元 UTC 儒略日；未初始化时为 0。 */
  double epoch_julian_day() const { return epoch_jd_; }

  /** @return 当前积分步长，单位 s。 */
  double step_size_sec() const { return step_size_s_; }

  /** @return J2 摄动是否启用。 */
  bool j2_enabled() const { return enable_j2_; }

  /** @return 当前 ECI 状态（未初始化时为默认零状态）。 */
  const EciState& current_state_eci() const { return current_state_eci_; }

  /** @return 上次传播到的 UTC 儒略日；未初始化时为 0。 */
  double last_propagated_julian_day() const { return last_propagated_jd_; }

 private:
  OrbitalElements elements_{};  /**< 轨道六根数（a, e, i, Ω, ω, M） */
  double epoch_jd_{0.0};        /**< 历元 UTC 儒略日 */
  double step_size_s_{10.0};    /**< 积分步长，单位 s（正下限见实现） */
  bool enable_j2_{true};        /**< J2 摄动开关 */
  bool initialized_{false};     /**< 是否已由 Initialize 建立历元状态 */
  EciState current_state_eci_{}; /**< 当前 ECI 位置与速度，跨周期保持 */
  double last_propagated_jd_{0.0}; /**< 上次传播到的 UTC 儒略日 */
};

}  // namespace satellite_orbit

#endif  // ONEQ_SATELLITE_ORBIT_ORBIT_PROPAGATOR_H_
