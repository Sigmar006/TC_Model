/**
 * @file RecognitionMatcher.h
 * @brief 动态加权匹配与候选排序（库内部）。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_MATCHER_H_
#define REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_MATCHER_H_

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "1q/remote_identification_radar/config/RirPolicyConfig.h"
#include "remote_identification_radar/recognition/RecognitionFeatureDatabase.h"
#include "remote_identification_radar/recognition/RecognitionTypes.h"

namespace remote_identification_radar {
namespace recognition {

/** @brief 单候选：型号 + 大类 + 先验加权分数。 */
struct RirCandidate {
  std::string model_id{};
  std::string category_id{};
  float score{0.0f}; /**< 先验加权后的型号得分（未归一化）。 */
};

/**
 * @brief RirMatchResult 匹配结果：候选排序、大类分数与可分性。
 */
struct RirMatchResult {
  bool has_candidates{false};  /**< 是否有任何模型参与匹配。 */
  std::vector<RirCandidate> candidates{}; /**< 按得分降序。 */
  std::vector<std::pair<std::string, float>>
      category_scores{}; /**< category_id → 大类内最佳型号得分（降序）。 */
  std::string best_model_id{};
  std::string best_category_id{};
  float best_score{0.0f};       /**< 第一候选先验加权分数。 */
  float runner_up_score{0.0f};  /**< 第二候选先验加权分数。 */
  float confidence{0.0f};       /**< 第一候选在全部候选中的归一化置信度，[0, 1]。 */
  std::uint8_t used_feature_mask{0U}; /**< 实际参与融合的维度掩码。 */
  /** 第一候选得分的 profile 下标（型号得分取其下所有适用 profile 的最高分，
   * front 未必是得分的那个；分项相似度输出应比对该下标处的模板）。 */
  std::size_t best_profile_index{0U};
};

/**
 * @brief RirMatcher 将单周期特征观测与数据库模板比对。
 *
 * 相似度：连续特征 z = |x - mean| / std，s = exp(-0.5·z²)。
 * 动态加权：score = Σ w(d)·q(d)·s(d) / Σ w(d)·q(d)，质量 0 的维度
 * 不参与分子也不参与分母。型号取适用 profile 的最高分并乘以先验；
 * 大类分数取大类内最佳型号的得分（成员数量不加分）；置信度为第一候选
 * 分数在全部候选分数和中的占比。
 */
class RirMatcher {
 public:
  /**
   * @brief 查询最佳匹配候选。
   * @param[in] features 单周期四维特征观测。
   * @param[in] context 周期效能上下文（applicability 判定用）。
   * @param[in] database 已加载特征数据库。
   * @param[in] weights 特征基础权重（默认等权 0.25×4）。
   * @return 匹配结果；空数据库或零可用维度时 has_candidates=false。
   */
  static RirMatchResult QueryBestMatch(
      const RirFeatureSet& features, const RirObservationContext& context,
      const RirFeatureDatabase& database,
      const config::RirRecognitionFeatureWeights& weights = config::RirRecognitionFeatureWeights());

  /**
   * @brief 计算观测相对指定 profile 的四维分项相似度（rcs/motion/polarization/range_profile）。
   * @param[in] features 聚合观测。
   * @param[in] profile 型号模板。
   * @return 四维相似度数组（不可用维度为 0）。
   */
  static std::array<float, 4> ComputeFeatureSimilarities(
      const RirFeatureSet& features, const RirModelProfile& profile);
};

}  // namespace recognition
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_MATCHER_H_
