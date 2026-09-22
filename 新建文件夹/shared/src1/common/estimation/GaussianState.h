/**
 * @file GaussianState.h
 * @brief 定义高斯状态表示与维度化类型别名，作为 Kalman 滤波系列的核心数据载体。
 *
 * 本头从 airborne_radar 的 GaussianTrackState 迁移并模板化：状态维与量测维由非类型模板参数
 * 表达，调用方按场景实例化（如机载雷达 6 维 CV 状态 / 3 维位置量测，天基红外 6 维状态 /
 * 2 维角度量测）。常用 6/3 实例化通过 GaussianTrackState 别名保留，向后兼容。
 */

#ifndef COMMON_ESTIMATION_GAUSSIAN_STATE_H_
#define COMMON_ESTIMATION_GAUSSIAN_STATE_H_

#include <Eigen/Core>

namespace oneq {
namespace common {
namespace estimation {

/**
 * @brief 高斯状态表示，作为 Kalman 滤波器的核心数据载体。
 * @details 封装状态均值向量和协方差矩阵，对应 Stone Soup 中的 GaussianState。
 * @tparam kStateDim 状态空间维度（如 3D 恒速模型为 6：[x, vx, y, vy, z, vz]）。
 * @tparam kMeasurementDim 量测空间维度（如 3D 位置量测为 3：[x, y, z]）。
 */
template <int kStateDim, int kMeasurementDim>
struct GaussianState {
  static constexpr int state_dim = kStateDim;                  /**< 状态空间维度。 */
  static constexpr int measurement_dim = kMeasurementDim;      /**< 量测空间维度。 */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief 状态向量类型（固定维度，列向量）。 */
  using StateVector = Eigen::Matrix<float, kStateDim, 1>;
  /** @brief 状态协方差矩阵类型。 */
  using StateCovariance = Eigen::Matrix<float, kStateDim, kStateDim>;
  /** @brief 量测向量类型。 */
  using MeasurementVector = Eigen::Matrix<float, kMeasurementDim, 1>;
  /** @brief 量测协方差矩阵类型。 */
  using MeasurementCovariance = Eigen::Matrix<float, kMeasurementDim, kMeasurementDim>;
  /** @brief 状态转移矩阵类型。 */
  using TransitionMatrix = Eigen::Matrix<float, kStateDim, kStateDim>;
  /** @brief 过程噪声协方差矩阵类型。 */
  using ProcessNoiseCovariance = Eigen::Matrix<float, kStateDim, kStateDim>;
  /** @brief 量测矩阵类型（H: kMeasurementDim × kStateDim）。 */
  using MeasurementMatrix = Eigen::Matrix<float, kMeasurementDim, kStateDim>;
  /** @brief Kalman 增益矩阵类型（K: kStateDim × kMeasurementDim）。 */
  using KalmanGainMatrix = Eigen::Matrix<float, kStateDim, kMeasurementDim>;

  StateVector mean{StateVector::Zero()};                   /**< 状态均值向量。 */
  StateCovariance covariance{StateCovariance::Identity()}; /**< 状态协方差矩阵。 */

  GaussianState() = default;
  /**
   * @brief 从均值和协方差构造。
   * @param[in] m 状态均值向量。
   * @param[in] p 状态协方差矩阵。
   */
  GaussianState(const StateVector& m, const StateCovariance& p) : mean(m), covariance(p) {}
};

/**
 * @brief 机载雷达 3D 恒速模型高斯状态（6 维状态 / 3 维位置量测）的便捷别名。
 * @details 状态向量布局为 [x, vx, y, vy, z, vz]，位置和速度交替排列。量测为 [x, y, z]。
 */
using GaussianTrackState = GaussianState<6, 3>;

}  // namespace estimation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ESTIMATION_GAUSSIAN_STATE_H_
