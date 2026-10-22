/**
 * @file SbirsStateSanity.h
 * @brief 红外滤波状态的"可发布"判定：发散/非有限状态不得当作跟踪结果输出。
 *
 * 背景（2026-10-21 红外日志体检）：角度-only EKF 在 4 万公里斜距上距离不可观测，
 * 实测有 62 行状态分量到 1e9 m、速度 7.7e5 m/s、协方差 1e19，日志仍按"跟踪"发布，
 * 并导致第 9 项测角残差在若干时段出现 ±2.5° 的逐拍变号量（2,962 行超 ±360″ 中有
 * 2,820 行落在 NIS=inf 的同期）。
 *
 * 判定口径（方法常数，来自物理量级而非调参）：
 *   - 位置必须是有限量，且地心距落在 [地球半径量级, 1e8 m] —— 目标在地球附近或弹道中段，
 *     1e8 m 已是 GEO 高度的 2.4 倍，超出即不可信；
 *   - 位置/速度全部有限（不得有 nan/inf）；
 *   - 协方差对角必须有限且 ≤ 1e16（1e9 m 位置对应的方差为 1e18，必然越界）。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_STATE_SANITY_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_STATE_SANITY_H_

#include <cmath>

#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace pipeline {

/** @brief 可发布状态的地心距下界（m）：略低于地球半径，覆盖地面/掠地目标。 */
constexpr double kSbirsSanePositionMinM = 6.0e6;
/** @brief 可发布状态的地心距上界（m）：GEO 高度 4.2e7 的 2.4 倍，弹道中段远小于此。 */
constexpr double kSbirsSanePositionMaxM = 1.0e8;
/** @brief 可发布状态协方差对角上限（位置方差 1e16 ⇔ 1σ=1e8 m）。 */
constexpr double kSbirsSaneCovarianceDiagMax = 1.0e16;

/**
 * @brief 判断一次滤波后验是否可发布（模板化以避免引入 Eigen 依赖）。
 * @param[in] mean 状态向量（布局 [x, vx, y, vy, z, vz]，位置在 0/2/4）。
 * @param[in] covariance 状态协方差。
 * @return true 表示有限、量级合理、协方差未爆炸。
 * @note 协方差对角非正（<0）同样判不可发布：负方差不代表"精度极好"。
 */
template <typename MeanT, typename CovT>
bool IsSaneSbirsState(const MeanT& mean, const CovT& covariance) {
  const double x = static_cast<double>(mean(0));
  const double y = static_cast<double>(mean(2));
  const double z = static_cast<double>(mean(4));
  const double velocity[3] = {static_cast<double>(mean(1)), static_cast<double>(mean(3)),
                              static_cast<double>(mean(5))};
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }
  for (int index = 0; index < 3; ++index) {
    if (!std::isfinite(velocity[index])) {
      return false;
    }
  }
  const double radius = std::sqrt(x * x + y * y + z * z);
  if (!(radius >= kSbirsSanePositionMinM) || !(radius <= kSbirsSanePositionMaxM)) {
    return false;
  }
  const int diagonal_size = static_cast<int>(covariance.rows());
  for (int index = 0; index < diagonal_size; ++index) {
    const double variance = static_cast<double>(covariance(index, index));
    if (!(variance >= 0.0) || !(variance <= kSbirsSaneCovarianceDiagMax)) {
      return false;
    }
  }
  return true;
}

/** @brief 可发布角度域状态的最大协方差对角（rad²；1e6 rad² ⇔ 1σ≈1000 rad）。 */
constexpr double kSbirsSaneAngleCovarianceDiagMax = 1.0e6;

/**
 * @brief 角度域 CV 状态（[az, vaz, el, vel]）的可发布判定。
 * @param[in] mean 角度状态向量（rad、rad/s；az 在 0、el 在 2）。
 * @param[in] covariance 状态协方差（rad²、rad²/s²）。
 * @return 有限、角度在物理区间、协方差未爆炸时返回 true。
 * @note 与 `IsSaneSbirsState` 分开：6 维 ECI 状态的 0/2/4 是位置，4 维角度状态的 0/2 是角度，
 *       混用会把正常的角度状态（az/el≈0）误判成"地心距为 0"并丢弃整条航迹。
 */
template <typename MeanT, typename CovT>
bool IsSaneSbirsAngleState(const MeanT& mean, const CovT& covariance) {
  const double azimuth_rad = static_cast<double>(mean(0));
  const double azimuth_rate = static_cast<double>(mean(1));
  const double elevation_rad = static_cast<double>(mean(2));
  const double elevation_rate = static_cast<double>(mean(3));
  if (!std::isfinite(azimuth_rad) || !std::isfinite(azimuth_rate) ||
      !std::isfinite(elevation_rad) || !std::isfinite(elevation_rate)) {
    return false;
  }
  if (std::fabs(azimuth_rad) > 12.6) {  // 2·2π：允许未归一的累计方位
    return false;
  }
  if (std::fabs(elevation_rad) > 1.6) {  // π/2 + 余量
    return false;
  }
  if (std::fabs(azimuth_rate) > 100.0 || std::fabs(elevation_rate) > 100.0) {
    return false;  // rad/s 上限：视线角速率物理上远小于此
  }
  const int diagonal_size = static_cast<int>(covariance.rows());
  for (int index = 0; index < diagonal_size; ++index) {
    const double variance = static_cast<double>(covariance(index, index));
    if (!(variance >= 0.0) || !(variance <= kSbirsSaneAngleCovarianceDiagMax)) {
      return false;
    }
  }
  return true;
}

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_STATE_SANITY_H_
