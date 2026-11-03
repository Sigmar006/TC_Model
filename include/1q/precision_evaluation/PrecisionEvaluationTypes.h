/**
 * @file PrecisionEvaluationTypes.h
 * @brief 定义精度评估层的输入真值、逐周期误差样本与汇总报告类型（需求 3.2.1.6.3）。
 */

#ifndef ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_TYPES_H_
#define ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_TYPES_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace precision_evaluation {

/**
 * @brief 评估用真值目标状态（每周期由调用方从场景真值提供）。
 * @note 评估层是场景真值的合法消费者（分层契约去真值化规则的唯一出口），
 *       真值只进本层、只用于对照，不回写任何产品层。
 */
struct ONEQ_API EvaluationTruthTarget {
  std::uint64_t key{0U};                              /**< 目标关联键（与各层产品同键体系） */
  oneq::coordinate::EcefPositionM position_ecef_m{};  /**< 真值位置（ECEF，单位 m） */
  oneq::coordinate::EcefVelocityMps velocity_ecef_m_per_s{}; /**< 真值速度（ECEF，单位 m/s） */
  bool has_velocity{false};                           /**< 是否提供真值速度 */
  /**
   * @brief 目标辐射强度（W/sr，构造 SBIRS 场景与 SNR 门用）。
   *
   * @details 改 **500000**（2026-11-04）——与 `SbirsSceneTarget` 库默认、以及 OPIR 装配层
   *   的 `kDefaultRadiantIntensityWPerSr` 统一。原 1e8 使评估内部的场景比实际链路亮
   *   23 dB，评估出的角度/落点误差与真实链路不同源。
   */
  double radiant_intensity_w_per_sr{500000.0};
  bool active{true};                                  /**< 目标是否在场 */
};

/**
 * @brief 双星星历输入（每周期）：两颗卫星的 ECEF 位置/速度与 ECI 姿态。
 * @note 两颗卫星由各自实体驱动独立 SBIRS 会话；本结构只提供交会几何与评估对照用的星历。
 */
struct ONEQ_API DualSatEphemerisInput {
  oneq::coordinate::EcefPositionM satellite_a_position_ecef_m{};   /**< 主星位置（ECEF，m） */
  oneq::coordinate::EcefVelocityMps satellite_a_velocity_ecef_m_per_s{}; /**< 主星速度（ECEF，m/s） */
  oneq::coordinate::EcefPositionM satellite_b_position_ecef_m{};   /**< 辅星位置（ECEF，m） */
  oneq::coordinate::EcefVelocityMps satellite_b_velocity_ecef_m_per_s{}; /**< 辅星速度（ECEF，m/s） */
  double satellite_a_attitude_yaw_deg{0.0};   /**< 主星姿态 yaw（Z-Y-X，Body→ECI，deg） */
  double satellite_a_attitude_pitch_deg{0.0}; /**< 主星姿态 pitch（deg） */
  double satellite_a_attitude_roll_deg{0.0};  /**< 主星姿态 roll（deg） */
  double satellite_b_attitude_yaw_deg{0.0};   /**< 辅星姿态 yaw（deg） */
  double satellite_b_attitude_pitch_deg{0.0}; /**< 辅星姿态 pitch（deg） */
  double satellite_b_attitude_roll_deg{0.0};  /**< 辅星姿态 roll（deg） */
};

/** @brief 精度指标全集（AHP 准则层五项，需求 3.2.1.6.3.1 关键精度指标提取）。 */
enum class ONEQ_API PrecisionMetric {
  kAngular = 0,   /**< 红外定位角度误差（deg） */
  kDualSatFix,    /**< 双星交会定位位置误差（m） */
  kVelocity,      /**< 速度误差（m/s，估计层航迹 vs 真值） */
  kImpactPoint,   /**< 落点预测误差（m） */
  kLaunchPoint,   /**< 发射点预测误差（m） */
};

/** @brief 指标数量（AHP 判断矩阵维度）。 */
constexpr std::size_t kPrecisionMetricCount = 5U;

/** @brief 单指标误差序列汇总统计。 */
struct ONEQ_API ErrorMetricSummary {
  std::size_t count{0U}; /**< 样本数 */
  double mean{0.0};      /**< 误差均值 */
  double rmse{0.0};      /**< 误差均方根（AHP 归一化输入） */
  double p95{0.0};       /**< 误差 95 分位 */
  double max{0.0};       /**< 误差最大值 */
};

/** @brief AHP 判断矩阵（5×5 Saaty 1-9 标度互反矩阵；默认全 1 = 等权）。 */
struct ONEQ_API AhpJudgmentMatrix {
  double values[kPrecisionMetricCount][kPrecisionMetricCount]; /**< 行主序判断矩阵 */

  AhpJudgmentMatrix();  // cpp 中默认填充全 1（等权、完全一致）
};

/** @brief AHP 求解结果（权重与一致性）。
 *  @note 无等级评定（分档映射）、多层级层次树与贡献度排序输出——2026-08-20
 *        验收输出统计裁定不新增（docs/review/acceptance_output_inventory_2026-08-20.md §4.3/§6）。 */
struct ONEQ_API AhpEvaluation {
  double weights[kPrecisionMetricCount]{}; /**< 归一化权重（Σ=1，主特征向量） */
  double lambda_max{0.0};                  /**< 最大特征值 */
  double consistency_index{0.0};           /**< CI = (λmax−n)/(n−1) */
  double consistency_ratio{0.0};           /**< CR = CI/RI（n=5 时 RI=1.12） */
  bool is_consistent{false};               /**< CR ≤ 0.1 判一致 */
};

/** @brief 角度误差样本（单星单目标单周期）。 */
struct ONEQ_API AngularErrorSample {
  std::uint64_t key{0U};        /**< 目标键 */
  std::uint32_t cycle_index{0U}; /**< 周期号 */
  int satellite_index{0};       /**< 卫星序号（0=主星，1=辅星） */
  double azimuth_error_deg{0.0};   /**< 方位角误差（输出 − 真值，wrap-aware，deg） */
  double elevation_error_deg{0.0}; /**< 俯仰角误差（deg） */
};

/** @brief 双星交会定位误差样本（同周期两星同目标均检出时产生）。 */
struct ONEQ_API DualSatFixSample {
  std::uint64_t key{0U};
  std::uint32_t cycle_index{0U};
  double position_error_m{0.0}; /**< 交会位置与真值位置的三维距离 */
  double los_residual_m{0.0};   /**< 两视线异面直线最近距离（几何残差，越小说明交会越"实"） */
  double slant_range_error_m{0.0}; /**< 斜距误差：|主星−交会解| − |主星−真值|（评审 2026-08-26 条13） */
};

/** @brief 速度误差样本（估计层航迹 vs 真值；附位置误差供参考，不进 AHP）。 */
struct ONEQ_API VelocityErrorSample {
  std::uint64_t key{0U};
  std::uint32_t cycle_index{0U};
  double velocity_error_m_per_s{0.0}; /**< 速度矢量误差模长 */
  double position_error_m{0.0};       /**< 航迹位置误差模长（附注字段） */
};

/** @brief 落点/发射点预测误差样本（推演层关键点对照）。 */
struct ONEQ_API KeyPointErrorSample {
  std::uint64_t key{0U};
  std::uint32_t cycle_index{0U};
  bool has_impact{false};       /**< 本周期是否形成落点预测 */
  double impact_error_m{0.0};   /**< 估计状态落点与真值状态落点的三维距离 */
  bool has_launch{false};       /**< 本周期是否形成发射点预测 */
  double launch_error_m{0.0};   /**< 估计状态发射点与真值状态发射点的三维距离 */
};

/** @brief 落点预报外发样本（评审 2026-08-26 条12：装配层据此外发事件）。 */
struct ONEQ_API ImpactForecastSample {
  std::uint64_t key{0U};              /**< 目标键 */
  std::uint32_t cycle_index{0U};      /**< 周期号 */
  oneq::coordinate::LlaPositionDegM impact_point{}; /**< 预测落点（度制 LLA） */
  double impact_position_sigma_m{0.0}; /**< 落点 1-σ（m） */
  bool has_burnout_sigma{false};      /**< 是否携带关机点 1-σ */
  double burnout_position_sigma_m{0.0}; /**< 关机点 1-σ（m） */
};

/** @brief 单周期评估结果（四类误差样本 + 外发样本 + 周期号）。 */
struct ONEQ_API PrecisionEvaluationCycleResult {
  std::uint32_t cycle_index{0U};            /**< 周期号 */
  std::vector<AngularErrorSample> angular{};   /**< 角度误差样本 */
  std::vector<DualSatFixSample> dual_sat{};    /**< 双星交会样本 */
  std::vector<VelocityErrorSample> velocity{}; /**< 速度误差样本 */
  std::vector<KeyPointErrorSample> keypoints{}; /**< 关键点误差样本 */
  std::vector<ImpactForecastSample> forecasts{}; /**< 落点预报外发样本（有落点解的估计航迹） */
};

/** @brief 全程精度评估报告（五指标汇总 + AHP 权重/一致性 + 综合评分）。 */
struct ONEQ_API PrecisionEvaluationReport {
  ErrorMetricSummary metrics[kPrecisionMetricCount]{}; /**< 五指标汇总（角度/双星/速度/落点/发射点） */
  AhpEvaluation ahp{};                    /**< AHP 权重与一致性 */
  double reference_errors[kPrecisionMetricCount]{}; /**< 归一化参考误差（配置透传） */
  double metric_scores[kPrecisionMetricCount]{};    /**< 单指标得分 ref/(ref+rmse) ∈ [0,1) */
  double metric_contributions[kPrecisionMetricCount]{}; /**< 加权贡献 w·score（Σ=综合分） */
  double composite_score{0.0};            /**< 综合定位精度得分 ∈ [0,1]（越大越好） */
  bool all_metrics_sampled{false};        /**< 五指标是否均有样本 */
  bool ahp_valid{false};                  /**< AHP 判断矩阵是否合法求解（false 时综合分保持 0，不得静默退化） */
};

}  // namespace precision_evaluation

#endif  // ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_TYPES_H_
