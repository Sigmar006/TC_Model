/**
 * @file Constants.h
 * @brief 定义库内共享的物理常量与角度换算工具。
 */

#ifndef COMMON_NUMERICS_CONSTANTS_H_
#define COMMON_NUMERICS_CONSTANTS_H_

namespace oneq {
namespace common {
namespace numerics {

/** @brief 圆周率 (π) */
constexpr double kPi = 3.14159265358979323846;

/** @brief 二倍圆周率 (2π) */
constexpr double kTwoPi = 6.28318530717958647692;

/** @brief 半圆周率 (π/2) */
constexpr double kHalfPi = 1.57079632679489661923;

/** @brief 真空光速 (m/s)，CODATA 2018 */
constexpr double kLightSpeed = 299792458.0;

/** @brief 玻尔兹曼常数 (J/K)，CODATA 2019 */
constexpr double kBoltzmann = 1.380649e-23;

/** @brief 普朗克常数 (J·s)，CODATA 2019 */
constexpr double kPlanck = 6.62607015e-34;

/**
 * @brief 角度（deg）转弧度（rad）。
 * @tparam T 标量类型。
 * @param[in] angle_deg 输入角度（单位：deg）。
 * @return 等效弧度（单位：rad）。
 */
template <typename T>
inline T DegToRad(T angle_deg) {
  return angle_deg * static_cast<T>(kPi) / static_cast<T>(180);
}

/**
 * @brief 弧度（rad）转角度（deg）。
 * @tparam T 标量类型。
 * @param[in] angle_rad 输入弧度（单位：rad）。
 * @return 等效角度（单位：deg）。
 */
template <typename T>
inline T RadToDeg(T angle_rad) {
  return angle_rad * static_cast<T>(180) / static_cast<T>(kPi);
}

}  // namespace numerics
}  // namespace common
}  // namespace oneq

#endif  // COMMON_NUMERICS_CONSTANTS_H_
