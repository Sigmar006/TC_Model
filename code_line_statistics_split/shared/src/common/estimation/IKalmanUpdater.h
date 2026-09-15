/**
 * @file IKalmanUpdater.h
 * @brief 定义基于 Kalman 滤波的量测更新器抽象接口（维度模板化）。
 */

#ifndef COMMON_ESTIMATION_I_KALMAN_UPDATER_H_
#define COMMON_ESTIMATION_I_KALMAN_UPDATER_H_

#include "common/estimation/GaussianState.h"

#include <type_traits>

namespace oneq {
namespace common {
namespace estimation {

/**
 * @brief Kalman 更新器配置。
 */
struct KalmanUpdaterConfig {
  float measurement_noise_std{10.0f}; /**< 位置量测噪声标准差（米），各轴相同。对应 Stone Soup
                                         MeasurementModel 中 noise_covar 的对角元素平方根。 */
};

/**
 * @brief Kalman 更新结果，包含后验状态和增益等诊断信息。
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
struct KalmanUpdateResult {
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;
  using MeasurementVector = typename GaussianStateT::MeasurementVector;
  using MeasurementCovariance = typename GaussianStateT::MeasurementCovariance;

  GaussianStateT posterior;                                              /**< 后验高斯状态。 */
  MeasurementVector innovation{MeasurementVector::Zero()};               /**< 新息向量 y = z - H·x̂。 */
  MeasurementCovariance innovation_covariance{
      MeasurementCovariance::Identity()}; /**< 新息协方差 S = H·P̂·Hᵀ + R。 */
};

/**
 * @brief Kalman 更新器抽象接口。
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
class IKalmanUpdater {
 public:
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;
  using MeasurementVector = typename GaussianStateT::MeasurementVector;
  using MeasurementCovariance = typename GaussianStateT::MeasurementCovariance;
  using MeasurementMatrix = typename GaussianStateT::MeasurementMatrix;

  virtual ~IKalmanUpdater() = default;
  /**
   * @brief 对预测状态执行量测更新。
   * @param[in] predicted 预测后的高斯状态。
   * @param[in] measurement 量测向量。
   * @return 更新结果，包含后验状态和诊断信息。
   */
  virtual KalmanUpdateResult<kStateDim, kMeasurementDim> Update(
      const GaussianStateT& predicted, const MeasurementVector& measurement) const = 0;
  /**
   * @brief 对预测状态执行带有动态误差协方差矩阵的量测更新。
   * @param[in] predicted 预测后的高斯状态。
   * @param[in] measurement 量测向量。
   * @param[in] dynamic_R 动态计算的量测噪声协方差矩阵 R。
   * @return 更新结果，包含后验状态和诊断信息。
   */
  virtual KalmanUpdateResult<kStateDim, kMeasurementDim> Update(
      const GaussianStateT& predicted, const MeasurementVector& measurement,
      const MeasurementCovariance& dynamic_R) const = 0;
  /**
   * @brief 更新更新器配置。
   * @details 默认实现为空操作。使用 KalmanUpdaterConfig 的子类应重写本方法。
   *          使用自定义配置类型的子类（如 EkfUpdater）保留其自身的 UpdateConfig 重载。
   * @param[in] config 更新器配置。
   */
  virtual void UpdateConfig(KalmanUpdaterConfig /*config*/) {}
  /**
   * @brief 构建位置量测矩阵 H，从状态中提取位置分量。
   * @details 当前仅对 6 维 CV 状态布局 [x, vx, y, vy, z, vz] 提供 3 维位置提取（H(0,0)/H(1,2)/H(2,4)）。
   *          其它维度布局返回零矩阵，调用方应通过 EKF 的 IMeasurementModel 提供自定义量测映射。
   * @return 位置提取矩阵，各 Kalman 变体共享。
   */
  static MeasurementMatrix BuildPositionMeasurementMatrix() {
    MeasurementMatrix H = MeasurementMatrix::Zero();
#if __cplusplus >= 201703L
    if constexpr (kStateDim == 6 && kMeasurementDim == 3) {
      H(0, 0) = 1.0f;  // x
      H(1, 2) = 1.0f;  // y
      H(2, 4) = 1.0f;  // z
    }
#else
    ApplyPositionEntries6x3(H, std::integral_constant<bool, kStateDim == 6 && kMeasurementDim == 3>{});
#endif
    return H;
  }
  /**
   * @brief 构建默认量测噪声协方差矩阵 R（对角阵）。
   * @param[in] std_dev 各轴量测噪声标准差（单位：m）。
   * @return 对角协方差矩阵，各 Kalman 变体共享。
   */
  static MeasurementCovariance BuildDefaultMeasurementNoise(float std_dev) {
    const float variance = std_dev * std_dev;
    return MeasurementCovariance::Identity() * variance;
  }

 private:
  // C++17 以下兼容：tag dispatch 消除维度不匹配时越界的矩阵访问分支。
  static void ApplyPositionEntries6x3(MeasurementMatrix& H, std::true_type) {
    H(0, 0) = 1.0f;  // x
    H(1, 2) = 1.0f;  // y
    H(2, 4) = 1.0f;  // z
  }
  static void ApplyPositionEntries6x3(MeasurementMatrix&, std::false_type) {}
};

}  // namespace estimation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ESTIMATION_I_KALMAN_UPDATER_H_
