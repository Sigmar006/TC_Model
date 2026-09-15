/**
 * @file RecognitionMatcher.cpp
 * @brief 动态加权匹配实现。
 */

#include "remote_identification_radar/recognition/RecognitionMatcher.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace remote_identification_radar {
namespace recognition {

namespace {

/** @brief 连续特征相似度：z = |x - mean| / std，s = exp(-0.5·z²)。 */
float GaussianSimilarity(float value, float mean, float std) {
  if (std <= 0.0f) {
    return 0.0f;
  }
  const float z = std::fabs(value - mean) / std;
  return std::exp(-0.5f * z * z);
}

float MotionSimilarity(const RirMotionObservation& motion, const RirMotionTemplate& tpl) {
  float sum = 0.0f;
  float weight_sum = 0.0f;
  const struct {
    float value;
    float mean;
    float std;
    float weight;
  } features[] = {
      {motion.speed_mps, tpl.speed_mps.mean, tpl.speed_mps.std, 1.0f},
      {motion.altitude_m, tpl.altitude_m.mean, tpl.altitude_m.std, 1.0f},
      {motion.acceleration_mps2, tpl.acceleration_mps2.mean, tpl.acceleration_mps2.std, 1.0f},
  };
  for (const auto& feature : features) {
    sum += feature.weight * GaussianSimilarity(feature.value, feature.mean, feature.std);
    weight_sum += feature.weight;
  }
  if (!motion.is_straight && motion.turn_radius_m > 0.0f && motion.turn_radius_m
      < std::numeric_limits<float>::infinity()) {
    const float log10_radius = std::log10(motion.turn_radius_m);
    sum += GaussianSimilarity(log10_radius, tpl.turn_radius_log10.mean,
                              tpl.turn_radius_log10.std);
    weight_sum += 1.0f;
  }
  return weight_sum > 0.0f ? sum / weight_sum : 0.0f;
}

float PolarizationSimilarity(const RirPolarizationObservation& polarization,
                             const RirPolarizationTemplate& tpl) {
  float sum = 0.0f;
  float weight_sum = 0.0f;
  const struct {
    float value;
    float mean;
    float std;
  } features[] = {
      {polarization.energy_difference_db, tpl.energy_difference_db.mean,
       tpl.energy_difference_db.std},
      {polarization.relative_difference_db, tpl.relative_difference_db.mean,
       tpl.relative_difference_db.std},
      {polarization.energy_sum_db, tpl.energy_sum_db.mean, tpl.energy_sum_db.std},
  };
  for (const auto& feature : features) {
    sum += GaussianSimilarity(feature.value, feature.mean, feature.std);
    weight_sum += 1.0f;
  }
  return weight_sum > 0.0f ? sum / weight_sum : 0.0f;
}

float RangeProfileSimilarity(const RirRangeProfileObservation& range_profile,
                             const RirRangeProfileTemplate& tpl) {
  float sum = 0.0f;
  float weight_sum = 0.0f;
  const struct {
    float value;
    float mean;
    float std;
  } features[] = {
      {range_profile.length_m, tpl.length_m.mean, tpl.length_m.std},
      {static_cast<float>(range_profile.peak_count), tpl.peak_count.mean, tpl.peak_count.std},
      {range_profile.peak_energy_concentration, tpl.peak_energy_concentration.mean,
       tpl.peak_energy_concentration.std},
  };
  for (const auto& feature : features) {
    sum += GaussianSimilarity(feature.value, feature.mean, feature.std);
    weight_sum += 1.0f;
  }
  return weight_sum > 0.0f ? sum / weight_sum : 0.0f;
}

/** @brief profile 是否满足适用条件。 */
bool Applicable(const RirModelProfile& profile, const RirFeatureSet& features,
                const RirObservationContext& context) {
  if (context.snr_db < profile.min_snr_db) {
    return false;
  }
  if (profile.max_range_resolution_m > 0.0f && features.range_profile.valid &&
      features.range_profile.resolution_m > profile.max_range_resolution_m) {
    return false;
  }
  // 数据库 profile 级适用条件：视角覆盖下限与最低带宽。
  if (profile.rcs.minimum_aspect_coverage_deg > 0.0f && features.rcs.valid &&
      features.rcs.aspect_coverage_deg < profile.rcs.minimum_aspect_coverage_deg) {
    return false;
  }
  if (profile.range_profile.minimum_bandwidth_hz > 0.0f &&
      context.bandwidth_hz < profile.range_profile.minimum_bandwidth_hz) {
    return false;
  }
  return true;
}

}  // namespace

std::array<float, 4> RirMatcher::ComputeFeatureSimilarities(
    const RirFeatureSet& features, const RirModelProfile& profile) {
  std::array<float, 4> similarities = {0.0f, 0.0f, 0.0f, 0.0f};
  if (features.rcs.valid) {
    similarities[0] = GaussianSimilarity(features.rcs.mean_dbsm, profile.rcs.mean_dbsm,
                                         profile.rcs.std_db);
  }
  if (features.motion.valid) {
    similarities[1] = MotionSimilarity(features.motion, profile.motion);
  }
  if (features.polarization.valid) {
    similarities[2] = PolarizationSimilarity(features.polarization, profile.polarization);
  }
  if (features.range_profile.valid) {
    similarities[3] = RangeProfileSimilarity(features.range_profile, profile.range_profile);
  }
  return similarities;
}

RirMatchResult RirMatcher::QueryBestMatch(
    const RirFeatureSet& features, const RirObservationContext& context,
    const RirFeatureDatabase& database,
    const config::RirRecognitionFeatureWeights& weights) {
  RirMatchResult result;
  if (!database.IsLoaded() || database.models().empty()) {
    return result;
  }
  result.used_feature_mask = features.valid_feature_mask;

  // 基础权重（来自 RirRecognitionPolicy::feature_weights）。
  const float base_weights[4] = {weights.rcs_weight, weights.motion_weight,
                                 weights.polarization_weight, weights.range_profile_weight};
  const float total_base_weight = base_weights[0] + base_weights[1] + base_weights[2] +
                                  base_weights[3];
  if (total_base_weight <= 0.0f) {
    return result;
  }

  struct ModelScore {
    std::string model_id;
    std::string category_id;
    float score{0.0f};
    std::size_t best_profile_index{0U};
  };
  std::vector<ModelScore> model_scores;
  model_scores.reserve(database.models().size());

  for (std::size_t m = 0U; m < database.models().size(); ++m) {
    const RirModel& model = database.models()[m];
    float best_profile_score = 0.0f;
    std::size_t best_profile_index = 0U;
    for (std::size_t p = 0U; p < model.profiles.size(); ++p) {
      const RirModelProfile& profile = model.profiles[p];
      if (!Applicable(profile, features, context)) {
        continue;
      }
      // 动态加权：质量 0 的维度不参与分子也不参与分母。
      float weighted_sum = 0.0f;
      float quality_sum = 0.0f;
      const float qualities[4] = {features.rcs.quality, features.motion.quality,
                                  features.polarization.quality,
                                  features.range_profile.quality};
      const float similarities[4] = {
          GaussianSimilarity(features.rcs.mean_dbsm, profile.rcs.mean_dbsm, profile.rcs.std_db),
          MotionSimilarity(features.motion, profile.motion),
          PolarizationSimilarity(features.polarization, profile.polarization),
          RangeProfileSimilarity(features.range_profile, profile.range_profile)};
      const bool valid[4] = {features.rcs.valid, features.motion.valid,
                             features.polarization.valid, features.range_profile.valid};
      for (int d = 0; d < 4; ++d) {
        if (!valid[d] || qualities[d] <= 0.0f) {
          continue;
        }
        weighted_sum += base_weights[d] * qualities[d] * similarities[d];
        quality_sum += base_weights[d] * qualities[d];
      }
      if (quality_sum > 0.0f) {
        const float profile_score = weighted_sum / quality_sum;
        // 记录 argmax 下标：front 未必是得分的 profile，分项相似度需按它比对
        //（与既有 std::max 口径一致，平分保留先出现的 profile）。
        if (profile_score > best_profile_score) {
          best_profile_score = profile_score;
          best_profile_index = p;
        }
      }
    }
    if (best_profile_score > 0.0f) {
      ModelScore entry;
      entry.model_id = model.model_id;
      entry.category_id = model.category_id;
      entry.score = best_profile_score * model.prior;
      entry.best_profile_index = best_profile_index;
      model_scores.push_back(std::move(entry));
    }
  }

  if (model_scores.empty()) {
    return result;
  }
  std::sort(model_scores.begin(), model_scores.end(),
            [](const ModelScore& lhs, const ModelScore& rhs) { return lhs.score > rhs.score; });

  // 大类分数：大类内最佳型号的得分（2026-08-30 核查 8.3：由"其下型号分数之和"
  // 改判——求和让型号多的大类天然占优；取 max 后大类竞争与成员数量解耦）。
  // model_scores 已按分数降序，大类首个命中即其最佳型号；std::max 显式表达
  // "取最佳"语义（不依赖排序不变式）。
  struct CategoryScore {
    std::string category_id;
    float score{0.0f};
  };
  std::vector<CategoryScore> category_scores;
  for (std::size_t i = 0U; i < model_scores.size(); ++i) {
    bool found = false;
    for (std::size_t c = 0U; c < category_scores.size(); ++c) {
      if (category_scores[c].category_id == model_scores[i].category_id) {
        category_scores[c].score = std::max(category_scores[c].score, model_scores[i].score);
        found = true;
        break;
      }
    }
    if (!found) {
      CategoryScore entry;
      entry.category_id = model_scores[i].category_id;
      entry.score = model_scores[i].score;
      category_scores.push_back(std::move(entry));
    }
  }
  std::sort(category_scores.begin(), category_scores.end(),
            [](const CategoryScore& lhs, const CategoryScore& rhs) { return lhs.score > rhs.score; });

  float total_score = 0.0f;
  for (std::size_t i = 0U; i < model_scores.size(); ++i) {
    total_score += model_scores[i].score;
  }

  result.has_candidates = true;
  result.candidates.reserve(model_scores.size());
  for (std::size_t i = 0U; i < model_scores.size(); ++i) {
    RirCandidate candidate;
    candidate.model_id = model_scores[i].model_id;
    candidate.category_id = model_scores[i].category_id;
    candidate.score = model_scores[i].score;
    result.candidates.push_back(std::move(candidate));
  }
  result.category_scores.reserve(category_scores.size());
  for (std::size_t i = 0U; i < category_scores.size(); ++i) {
    result.category_scores.push_back(
        std::make_pair(category_scores[i].category_id, category_scores[i].score));
  }
  result.best_model_id = model_scores.front().model_id;
  result.best_category_id = model_scores.front().category_id;
  result.best_score = model_scores.front().score;
  result.best_profile_index = model_scores.front().best_profile_index;
  if (model_scores.size() >= 2U) {
    result.runner_up_score = model_scores[1].score;
  }
  result.confidence = total_score > 0.0f ? model_scores.front().score / total_score : 0.0f;
  return result;
}

}  // namespace recognition
}  // namespace remote_identification_radar
