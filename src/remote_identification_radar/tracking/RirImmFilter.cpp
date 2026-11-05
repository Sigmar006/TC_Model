/**
 * @file RirImmFilter.cpp
 * @brief RIR IMM 滤波器包装实现（阶段 2-T N4）。
 */

/**
 * ## 过程：IMM 多模型滤波（LRDR 跟踪/识别链 · 第 3 步）
 * - 上游：生命周期在 confirmed 航迹命中/失配时调用的滤波器包装（IMM 与 CV 二选一）。
 * - 本步：装配 N 个 CV 模型（过程噪声系数按 q·10^(i/(N−1)) 对数等距），每步执行
 *   混合 → 各模型预测 → 动态 R 更新 → 按量测似然更新模型概率 → 概率加权合并输出；
 *   转移矩阵对角固定 0.95。模型数 <2 时保持惰性（`IsValid()==false`），调用方回退 CV KF。
 * - 下游：`RirTrackLifecycle::ApplyGaussianState` 消费混合状态；模型权重可作机动强度诊断。
 * - 关键参数：`model_noise_diff_coeffs`（`RirTrackLifecycle::BuildImmNoiseDiffCoeffs`，
 *   以滤波器过程噪声系数为低端锚点）、`transition_diagonal_probability`（生命周期固定
 *   0.95）、模型数 `RirLifecycleConfig::model_count_hint`（场景配置为 2）。
 * - 对应验收项：第 42 项 跟踪滤波功能测试（机动段的速度/加速度估计由本步提供）。
 */

#include "remote_identification_radar/tracking/RirImmFilter.h"

#include <algorithm>
#include <cmath>

namespace remote_identification_radar {
namespace tracking {

namespace {

/** @brief 过程噪声差异系数下限（与 AR SignalComponentFactory 工厂钳制同口径）。 */
constexpr float kMinimumNoiseDiffCoeff = 0.001f;

/** @brief 对数等距过程噪声差异系数（口径同 AR BuildDefaultImmNoiseDiffCoeffs）。 */
// 过程：q_i = 10^(i/(N−1)) 对数等距——低档贴合匀速、高档容忍机动，模型数下限取 2。
std::vector<float> BuildDefaultNoiseDiffCoeffs(std::uint32_t model_count_hint) {
  const std::size_t model_count =
      static_cast<std::size_t>(model_count_hint < 2U ? 2U : model_count_hint);
  std::vector<float> coeffs;
  coeffs.reserve(model_count);
  for (std::size_t i = 0U; i < model_count; ++i) {
    coeffs.push_back(std::pow(10.0f, static_cast<float>(i) /
                                         static_cast<float>(model_count - 1U)));
  }
  return coeffs;
}

/** @brief 由对角自保持概率构建转移概率矩阵与均匀初始权重。 */
// 过程：对角越界或 ≥1 时回退 0.95；非对角均分剩余概率，初始模型权重取 1/N。
::oneq::common::estimation::ImmConfig BuildImmConfig(float transition_diagonal_probability,
                                                     std::size_t model_count) {
  const Eigen::Index n = static_cast<Eigen::Index>(model_count);
  const float diagonal =
      transition_diagonal_probability >= 0.5f && transition_diagonal_probability < 1.0f
          ? transition_diagonal_probability
          : 0.95f;
  ::oneq::common::estimation::ImmConfig imm_config;
  imm_config.transition_probability =
      Eigen::MatrixXf::Constant(n, n, (1.0f - diagonal) / static_cast<float>(model_count - 1U));
  imm_config.transition_probability.diagonal().setConstant(diagonal);
  imm_config.initial_weights = Eigen::VectorXf::Constant(n, 1.0f / static_cast<float>(model_count));
  return imm_config;
}

}  // namespace

// 过程：缺省构造走默认配置（空模型集 → 惰性态），等价于"未启用 IMM"。
RirImmFilter::RirImmFilter() : RirImmFilter(Config()) {}

// 过程：按模型数装配 N 个预测器（各自 q 档位）+ N 个更新器（R 逐拍动态注入，故更新器
//       缺省量测噪声不参与数值）；模型数 <2 时保持惰性，调用方按 IsValid() 回退 CV KF。
RirImmFilter::RirImmFilter(const Config& config) {
  std::vector<float> coeffs = config.model_noise_diff_coeffs;
  if (coeffs.empty()) {
    coeffs = BuildDefaultNoiseDiffCoeffs(2U);
  }
  if (coeffs.size() < 2U) {
    // 单模型 IMM 无混合意义：保持惰性（IsValid == false），调用方回退 CV KF。
    model_count_ = 0U;
    return;
  }
  model_noise_diff_coeffs_ = coeffs;

  const std::size_t model_count = coeffs.size();
  predictors_.resize(model_count);
  updaters_.resize(model_count);
  std::vector<ImmFilterT::Predictor*> predictor_ptrs;
  std::vector<ImmFilterT::Updater*> updater_ptrs;
  predictor_ptrs.reserve(model_count);
  updater_ptrs.reserve(model_count);
  for (std::size_t i = 0U; i < model_count; ++i) {
    ::oneq::common::estimation::KalmanPredictorConfig predictor_config;
    // 越界 q 钳制口径同 AR 工厂：非法 q 会污染 IMM 协方差传播。
    predictor_config.noise_diff_coeff = std::max(coeffs[i], kMinimumNoiseDiffCoeff);
    predictors_[i].UpdateConfig(predictor_config);
    // 更新器走动态 R 路径，缺省量测噪声不参与 IMM 数值。
    predictor_ptrs.push_back(&predictors_[i]);
    updater_ptrs.push_back(&updaters_[i]);
  }

  // 过程：模型表与转移/权重配置交给公共 IMM 原语，RIR 只负责模型集（q 档位）与接线。
  filter_.reset(new ImmFilterT(BuildImmConfig(config.transition_diagonal_probability, model_count),
                               predictor_ptrs, updater_ptrs));
  model_count_ = model_count;
}

// 过程：同一初始高斯状态复制到每个模型分支，权重取 IMM 初始权重。
void RirImmFilter::Initialize(const RirGaussianState& initial_state) {
  if (filter_ == nullptr) {
    return;
  }
  const Eigen::VectorXf weights = filter_->GetModelWeights();
  std::vector<ImmFilterT::ModelState> states;
  states.reserve(model_count_);
  for (std::size_t i = 0U; i < model_count_; ++i) {
    states.push_back(ImmFilterT::ModelState(
        initial_state, weights(static_cast<Eigen::Index>(i))));
  }
  filter_->SetModelStates(states);
}

// 过程：一步 IMM——按转移概率混合各模型初值 → 各自预测 → 以动态 R 量测更新 →
//       按各模型似然更新模型概率 → 概率加权合并输出。
void RirImmFilter::Process(const Eigen::Vector3f& position, float dt_sec,
                           const RirMeasurementCovariance& dynamic_R) {
  if (filter_ == nullptr) {
    return;
  }
  filter_->Process(position, dt_sec, dynamic_R);
}

// 过程：失配周期只做混合 + 预测，不更新模型概率（无新量测，算不出似然）。
void RirImmFilter::Predict(float dt_sec) {
  if (filter_ == nullptr) {
    return;
  }
  filter_->Predict(dt_sec);
}

// 过程：热调参要求模型数不变（模型集结构无法原位替换）；逐模型刷 q 后重建转移矩阵
//       与初始权重。
bool RirImmFilter::UpdateRuntimeTuning(const Config& config) {
  if (filter_ == nullptr || model_count_ < 2U) {
    return false;
  }
  std::vector<float> coeffs = config.model_noise_diff_coeffs;
  if (coeffs.empty()) {
    coeffs = BuildDefaultNoiseDiffCoeffs(static_cast<std::uint32_t>(model_count_));
  }
  // 模型数变化无法原位重调（模型集结构不同）：调用方按契约丢弃运行态并惰性重建。
  if (coeffs.size() != model_count_) {
    return false;
  }

  std::vector<ImmFilterT::Predictor*> predictor_ptrs;
  std::vector<ImmFilterT::Updater*> updater_ptrs;
  predictor_ptrs.reserve(model_count_);
  updater_ptrs.reserve(model_count_);
  for (std::size_t i = 0U; i < model_count_; ++i) {
    ::oneq::common::estimation::KalmanPredictorConfig predictor_config;
    predictor_config.noise_diff_coeff = std::max(coeffs[i], kMinimumNoiseDiffCoeff);
    predictors_[i].UpdateConfig(predictor_config);
    predictor_ptrs.push_back(&predictors_[i]);
    updater_ptrs.push_back(&updaters_[i]);
  }
  model_noise_diff_coeffs_ = coeffs;
  return filter_->UpdateRuntimeTuning(
      BuildImmConfig(config.transition_diagonal_probability, model_count_), predictor_ptrs,
      updater_ptrs);
}

// 过程：惰性态（filter_ 为空）返回缺省空状态，调用方按 IsValid() 决定是否采用。
RirGaussianState RirImmFilter::GetCombinedState() const {
  if (filter_ == nullptr) {
    return RirGaussianState();
  }
  return filter_->GetCombinedState();
}

// 过程：模型权重即各 CV 模型的后验概率——低档权重大意味着当前机动弱（近似匀速）。
Eigen::VectorXf RirImmFilter::GetModelWeights() const {
  if (filter_ == nullptr) {
    return Eigen::VectorXf();
  }
  return filter_->GetModelWeights();
}

}  // namespace tracking
}  // namespace remote_identification_radar
