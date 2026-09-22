/**
 * @file KalmanPredictor.h
 * @brief 定义基于 Kalman 滤波的恒速（CV）状态预测器（维度模板化）。
 */

#ifndef COMMON_ESTIMATION_KALMAN_PREDICTOR_H_
#define COMMON_ESTIMATION_KALMAN_PREDICTOR_H_

#include "common/estimation/GaussianState.h"
#include "common/estimation/IKalmanPredictor.h"

#include <type_traits>

namespace oneq {
namespace common {
namespace estimation {

/**
 * @brief 3D 恒速（Constant Velocity）Kalman 预测器。
 * @details 参考 Stone Soup CombinedLinearGaussianTransitionModel(3×ConstantVelocity)。
 *          状态向量布局 [x, vx, y, vy, z, vz]，每轴独立的 2D CV 模型通过 block_diag 组合。
 *
 *          单轴转移矩阵 F_1d：
 *          | 1  dt |
 *          | 0   1 |
 *
 *          单轴过程噪声 Q_1d（连续白噪声加速度离散化）：
 *          | dt³/3  dt²/2 |   × q
 *          | dt²/2  dt    |
 *
 *          CV 结构仅在 kStateDim == 6 时生效；其它维度下 F 退化为单位矩阵、Q 退化为零矩阵，
 *          调用方应改走 EKF 的 ITransitionModel 注入自定义动力学。
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
class KalmanPredictor final : public IKalmanPredictor<kStateDim, kMeasurementDim> {
 public:
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;
  using TransitionMatrix = typename GaussianStateT::TransitionMatrix;
  using ProcessNoiseCovariance = typename GaussianStateT::ProcessNoiseCovariance;

  /**
   * @brief 构造函数。
   * @param[in] config 预测器配置。
   */
  explicit KalmanPredictor(KalmanPredictorConfig config = {}) : config_(config) {}
  /**
   * @brief 对先验状态执行恒速模型预测。
   * @param[in] prior 先验高斯状态。
   * @param[in] dt 预测时间步长（秒），应由调用方保证 > 0。
   * @note 当前实现不做 dt 的运行时校验。
   * @return 预测后的高斯状态（含传播后的协方差）。
   */
  GaussianStateT Predict(const GaussianStateT& prior, float dt) const override {
    const TransitionMatrix F = BuildTransitionMatrix(dt);
    const ProcessNoiseCovariance Q = BuildProcessNoise(dt, config_.noise_diff_coeff);

    GaussianStateT predicted;
    predicted.mean = F * prior.mean;
    predicted.covariance = F * prior.covariance * F.transpose() + Q;
    return predicted;
  }
  /** @copydoc IKalmanPredictor::UpdateConfig */
  void UpdateConfig(KalmanPredictorConfig config) override { config_ = config; }

  /**
   * @brief 构建状态转移矩阵 F。
   * @details 仅 kStateDim == 6 时构造 3D CV 块对角矩阵；其它维度返回单位矩阵。
   * @param[in] dt 时间步长（秒）。
   * @return 状态转移矩阵。
   */
  static TransitionMatrix BuildTransitionMatrix(float dt) {
    TransitionMatrix F = TransitionMatrix::Identity();
#if __cplusplus >= 201703L
    if constexpr (kStateDim == 6) {
      F(0, 1) = dt;  // X 轴
      F(2, 3) = dt;  // Y 轴
      F(4, 5) = dt;  // Z 轴
    }
#else
    ApplyTransitionEntries6(F, dt, std::integral_constant<bool, kStateDim == 6>{});
#endif
    return F;
  }
  /**
   * @brief 构建过程噪声协方差矩阵 Q。
   * @details 仅 kStateDim == 6 时构造 3D CV 块对角噪声矩阵；其它维度返回零矩阵。
   * @param[in] dt 时间步长（秒）。
   * @param[in] q 噪声扩散系数。
   * @return 过程噪声矩阵。
   */
  static ProcessNoiseCovariance BuildProcessNoise(float dt, float q) {
    ProcessNoiseCovariance Q = ProcessNoiseCovariance::Zero();
#if __cplusplus >= 201703L
    if constexpr (kStateDim == 6) {
      const float dt2 = dt * dt;
      const float dt3 = dt2 * dt;
      for (int axis = 0; axis < 3; ++axis) {
        const int base = axis * 2;
        Q(base, base) = dt3 / 3.0f;
        Q(base, base + 1) = dt2 / 2.0f;
        Q(base + 1, base) = dt2 / 2.0f;
        Q(base + 1, base + 1) = dt;
      }
      Q *= q;
    }
#else
    ApplyProcessNoise6(Q, dt, q, std::integral_constant<bool, kStateDim == 6>{});
#endif
    return Q;
  }

 private:
  // C++17 以下兼容：tag dispatch 消除维度不匹配时越界的矩阵访问分支。
  static void ApplyTransitionEntries6(TransitionMatrix& F, float dt, std::true_type) {
    F(0, 1) = dt;  // X 轴
    F(2, 3) = dt;  // Y 轴
    F(4, 5) = dt;  // Z 轴
  }
  static void ApplyTransitionEntries6(TransitionMatrix&, float, std::false_type) {}
  static void ApplyProcessNoise6(ProcessNoiseCovariance& Q, float dt, float q, std::true_type) {
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    for (int axis = 0; axis < 3; ++axis) {
      const int base = axis * 2;
      Q(base, base) = dt3 / 3.0f;
      Q(base, base + 1) = dt2 / 2.0f;
      Q(base + 1, base) = dt2 / 2.0f;
      Q(base + 1, base + 1) = dt;
    }
    Q *= q;
  }
  static void ApplyProcessNoise6(ProcessNoiseCovariance&, float, float, std::false_type) {}

  KalmanPredictorConfig config_{}; /**< 当前配置。 */
};

extern template class KalmanPredictor<6, 3>;
extern template class KalmanPredictor<6, 2>;

}  // namespace estimation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ESTIMATION_KALMAN_PREDICTOR_H_
