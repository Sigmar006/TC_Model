/**
 * @file RecognitionTracker.cpp
 * @brief 识别积累与判定状态机实现。
 */

#include "remote_identification_radar/recognition/RecognitionTracker.h"

#include <algorithm>
#include <cmath>

#include "common/logging/ProjectLog.h"
#include "remote_identification_radar/recognition/RecognitionMatcher.h"
#include "remote_identification_radar/recognition/RecognitionObservationBuilder.h"

namespace remote_identification_radar {
namespace recognition {

namespace {

/** @brief 型号确认所需最小有效维度数（运动不能单独确认型号）。 */
constexpr std::uint32_t kMinValidDimensionsForModel = 2U;
/** @brief 有效观测质量下限：全部维度质量低于此值（如短驻留）不计为观测。 */
constexpr float kMinimumObservationQuality = 0.05f;

float Median(std::vector<float> values) {
  if (values.empty()) {
    return 0.0f;
  }
  std::sort(values.begin(), values.end());
  return values[values.size() / 2U];
}

float Mean(const std::vector<float>& values) {
  if (values.empty()) {
    return 0.0f;
  }
  float sum = 0.0f;
  for (std::size_t i = 0U; i < values.size(); ++i) {
    sum += values[i];
  }
  return sum / static_cast<float>(values.size());
}

std::uint8_t CountValidDimensions(const RirFeatureSet& set) {
  std::uint8_t count = 0U;
  if (set.rcs.valid && set.rcs.quality > 0.0f) {
    ++count;
  }
  if (set.motion.valid && set.motion.quality > 0.0f) {
    ++count;
  }
  if (set.polarization.valid && set.polarization.quality > 0.0f) {
    ++count;
  }
  if (set.range_profile.valid && set.range_profile.quality > 0.0f) {
    ++count;
  }
  return count;
}

bool MotionOnly(const RirFeatureSet& set) {
  return CountValidDimensions(set) == 1U && set.motion.valid && set.motion.quality > 0.0f;
}

/** @brief 数据库 category_id 字符串 → 公共大类枚举；未知映射为 kUnknown。 */
session::RirRecognitionCategory CategoryToPublic(const std::string& category_id) {
  if (category_id == "BALLISTIC") {
    return session::RirRecognitionCategory::kBallistic;
  }
  if (category_id == "NEAR_SPACE") {
    return session::RirRecognitionCategory::kNearSpace;
  }
  if (category_id == "FIGHTER") {
    return session::RirRecognitionCategory::kFighter;
  }
  if (category_id == "BOMBER") {
    return session::RirRecognitionCategory::kBomber;
  }
  if (category_id == "MISSILE") {
    return session::RirRecognitionCategory::kMissile;
  }
  // "UAV" 不再映射专属大类（2026-08-22 甲方裁定移除无人机识别），落入 kUnknown。
  if (category_id == "OTHER") {
    return session::RirRecognitionCategory::kOther;
  }
  return session::RirRecognitionCategory::kUnknown;
}

/**
 * @brief 滑动窗口聚合：运动子特征取中位数，其余取均值；质量取均值。
 */
RirFeatureSet AggregateWindow(const std::vector<RirFeatureSet>& window) {
  RirFeatureSet aggregate;
  std::vector<float> rcs_values;
  std::vector<float> speed_values;
  std::vector<float> altitude_values;
  std::vector<float> acceleration_values;
  std::vector<float> turn_radius_values;
  std::vector<float> pol_difference_values;
  std::vector<float> pol_relative_values;
  std::vector<float> pol_sum_values;
  std::vector<float> rp_length_values;
  std::vector<float> rp_peak_count_values;
  std::vector<float> rp_concentration_values;
  std::vector<float> rcs_quality;
  std::vector<float> motion_quality;
  std::vector<float> polarization_quality;
  std::vector<float> range_profile_quality;

  for (std::size_t i = 0U; i < window.size(); ++i) {
    const RirFeatureSet& set = window[i];
    if (set.rcs.valid) {
      rcs_values.push_back(set.rcs.mean_dbsm);
      rcs_quality.push_back(set.rcs.quality);
    }
    if (set.motion.valid) {
      speed_values.push_back(set.motion.speed_mps);
      altitude_values.push_back(set.motion.altitude_m);
      acceleration_values.push_back(set.motion.acceleration_mps2);
      if (!set.motion.is_straight && set.motion.turn_radius_m > 0.0f) {
        turn_radius_values.push_back(set.motion.turn_radius_m);
      }
      motion_quality.push_back(set.motion.quality);
    }
    if (set.polarization.valid) {
      pol_difference_values.push_back(set.polarization.energy_difference_db);
      pol_relative_values.push_back(set.polarization.relative_difference_db);
      pol_sum_values.push_back(set.polarization.energy_sum_db);
      polarization_quality.push_back(set.polarization.quality);
    }
    if (set.range_profile.valid) {
      rp_length_values.push_back(set.range_profile.length_m);
      rp_peak_count_values.push_back(static_cast<float>(set.range_profile.peak_count));
      rp_concentration_values.push_back(set.range_profile.peak_energy_concentration);
      range_profile_quality.push_back(set.range_profile.quality);
      aggregate.range_profile.resolution_m = set.range_profile.resolution_m;
    }
  }

  if (!rcs_values.empty()) {
    aggregate.rcs.valid = true;
    aggregate.rcs.mean_dbsm = Mean(rcs_values);
    aggregate.rcs.quality = Mean(rcs_quality);
  }
  if (!speed_values.empty()) {
    aggregate.motion.valid = true;
    aggregate.motion.speed_mps = Median(speed_values);
    aggregate.motion.altitude_m = Median(altitude_values);
    aggregate.motion.acceleration_mps2 = Median(acceleration_values);
    if (!turn_radius_values.empty()) {
      aggregate.motion.turn_radius_m = Median(turn_radius_values);
      aggregate.motion.is_straight = false;
    } else {
      aggregate.motion.is_straight = true;
    }
    aggregate.motion.quality = Mean(motion_quality);
  }
  if (!pol_difference_values.empty()) {
    aggregate.polarization.valid = true;
    aggregate.polarization.energy_difference_db = Mean(pol_difference_values);
    aggregate.polarization.relative_difference_db = Mean(pol_relative_values);
    aggregate.polarization.energy_sum_db = Mean(pol_sum_values);
    aggregate.polarization.quality = Mean(polarization_quality);
  }
  if (!rp_length_values.empty()) {
    aggregate.range_profile.valid = true;
    aggregate.range_profile.length_m = Mean(rp_length_values);
    aggregate.range_profile.peak_count =
        static_cast<std::uint32_t>(std::lround(Mean(rp_peak_count_values)));
    aggregate.range_profile.peak_energy_concentration = Mean(rp_concentration_values);
    aggregate.range_profile.quality = Mean(range_profile_quality);
  }
  aggregate.valid_feature_mask = 0U;
  if (aggregate.rcs.valid && aggregate.rcs.quality > 0.0f) {
    aggregate.valid_feature_mask |=
        static_cast<std::uint8_t>(session::RirRecognitionFeatureDimension::kRcs);
  }
  if (aggregate.motion.valid && aggregate.motion.quality > 0.0f) {
    aggregate.valid_feature_mask |=
        static_cast<std::uint8_t>(session::RirRecognitionFeatureDimension::kMotion);
  }
  if (aggregate.polarization.valid && aggregate.polarization.quality > 0.0f) {
    aggregate.valid_feature_mask |=
        static_cast<std::uint8_t>(session::RirRecognitionFeatureDimension::kPolarization);
  }
  if (aggregate.range_profile.valid && aggregate.range_profile.quality > 0.0f) {
    aggregate.valid_feature_mask |=
        static_cast<std::uint8_t>(session::RirRecognitionFeatureDimension::kRangeProfile);
  }
  return aggregate;
}

}  // namespace

void RirTracker::ExpireIfHeld(RirTrackState* state, float sim_time_sec) {
  if (state != nullptr && state->has_conclusion && state->conclusion_time_sec >= 0.0f &&
      sim_time_sec - state->conclusion_time_sec > options_.result_hold_sec) {
    state->result.state = session::RirRecognitionState::kStale;
  }
}

void RirTracker::ExitRecognitionMode() {
  for (std::unordered_map<std::uint64_t, RirTrackState>::iterator it = tracks_.begin();
       it != tracks_.end(); ++it) {
    // 保留结论进入保持期；清空积累。
    it->second.window.clear();
    it->second.window_timestamps_sec.clear();
    it->second.confirmed_hit_count = 0U;
    it->second.observation_count = 0U;
    it->second.first_observation_sec = 0.0f;
  }
}

void RirTracker::UpdateCycle(
    const std::vector<tracking::RirTrackState>& tracks,
    const std::unordered_map<std::uint64_t, TrackObservationInput>& observations_by_key,
    const RirFeatureDatabase& database, const config::RirRecognitionFeatureWeights& weights,
    float sim_time_sec, std::uint32_t cycle_index, std::uint64_t batch_id,
    std::vector<CycleFeatureObservation>* cycle_observations) {
  // 捕获 model_id → category_id 映射（真值准确率统计用，与识别结论无关）。
  if (model_categories_.empty() && database.IsLoaded()) {
    for (std::size_t m = 0U; m < database.models().size(); ++m) {
      model_categories_[database.models()[m].model_id] = database.models()[m].category_id;
    }
  }
  for (std::size_t i = 0U; i < tracks.size(); ++i) {
    const tracking::RirTrackState& snapshot = tracks[i];
    const std::uint64_t key = snapshot.association_key;
    RirTrackState& state = tracks_[key];
    state.association_key = key;

    // 键重分配检测：hit_count 回落视为新目标，清空旧状态。
    if (state.last_seen_hit_count > 0U && snapshot.hit_count < state.last_seen_hit_count) {
      state = RirTrackState{};
      state.association_key = key;
    }
    state.last_seen_hit_count = snapshot.hit_count;

    const std::unordered_map<std::uint64_t, TrackObservationInput>::const_iterator found =
        observations_by_key.find(key);
    if (found == observations_by_key.end() || found->second.target == nullptr) {
      // 本周期无观测输入：结论按 result_hold_sec 过期为 kStale。
      ExpireIfHeld(&state, sim_time_sec);
      continue;  // 无场景目标特征输入 → 本周期无观测
    }
    const TrackObservationInput& input = found->second;
    if (input.context.range_m > options_.max_range_m) {
      ExpireIfHeld(&state, sim_time_sec);
      continue;  // 超出最大识别距离
    }

    const RirFeatureSet features =
        RirObservationBuilder::Build(*input.target, snapshot, input.context);
    if (features.valid_feature_mask == 0U) {
      ExpireIfHeld(&state, sim_time_sec);
      continue;  // 无有效特征维度 → 本周期不积累
    }
    // 出口①透出采集：全维无效已排除；后续质量门只挡积累不挡量测出口。
    if (cycle_observations != nullptr) {
      CycleFeatureObservation observation;
      observation.association_key = key;
      observation.features = features;
      cycle_observations->push_back(observation);
    }
    // 观测质量下限：全部维度质量低于下限（如短驻留）的周期不计为有效观测。
    const float max_quality =
        std::max(std::max(features.rcs.quality, features.motion.quality),
                 std::max(features.polarization.quality, features.range_profile.quality));
    if (max_quality < kMinimumObservationQuality) {
      ExpireIfHeld(&state, sim_time_sec);
      continue;
    }

    // 时间窗维护：按 accumulation_window_sec 裁剪。
    if (state.window.empty()) {
      state.first_observation_sec = sim_time_sec;
    }
    state.window.push_back(features);
    state.window_timestamps_sec.push_back(sim_time_sec);
    while (state.window.size() > 1U &&
           sim_time_sec - state.window_timestamps_sec.front() > options_.accumulation_window_sec) {
      state.window.erase(state.window.begin());
      state.window_timestamps_sec.erase(state.window_timestamps_sec.begin());
    }
    ++state.observation_count;
    if (snapshot.status == tracking::RirTrackStatus::kConfirmed) {
      ++state.confirmed_hit_count;
    }

    session::RirRecognitionResult judgement;
    judgement.observation_count = state.observation_count;
    judgement.accumulation_sec = sim_time_sec - state.first_observation_sec;
    judgement.database_version = active_database_version_;
    judgement.source_cycle_index = cycle_index;
    judgement.source_batch_id = batch_id;

    // 判定门槛：确认命中数与有效观测数。
    if (state.confirmed_hit_count < options_.min_confirmed_hits ||
        state.observation_count < options_.min_observation_count) {
      judgement.state = session::RirRecognitionState::kAccumulating;
      state.result = judgement;
      continue;
    }

    const RirFeatureSet aggregate = AggregateWindow(state.window);
    if (aggregate.valid_feature_mask == 0U) {
      judgement.state = session::RirRecognitionState::kUnknown;
      state.result = judgement;
      continue;
    }

    const RirMatchResult match =
        RirMatcher::QueryBestMatch(aggregate, input.context, database, weights);
    const std::uint8_t valid_dimensions = CountValidDimensions(aggregate);
    const bool motion_only = MotionOnly(aggregate);

    judgement.valid_feature_mask = aggregate.valid_feature_mask;
    judgement.feature_scores.rcs_quality = aggregate.rcs.quality;
    judgement.feature_scores.motion_quality = aggregate.motion.quality;
    judgement.feature_scores.polarization_quality = aggregate.polarization.quality;
    judgement.feature_scores.range_profile_quality = aggregate.range_profile.quality;

    if (!match.has_candidates) {
      judgement.state = session::RirRecognitionState::kUnknown;
    } else {
      // 分项相似度：与最佳候选模板比对（best model 的适用 profile）。
      const RirModel* best_model = nullptr;
      for (std::size_t m = 0U; m < database.models().size(); ++m) {
        if (database.models()[m].model_id == match.best_model_id) {
          best_model = &database.models()[m];
          break;
        }
      }
      if (best_model != nullptr && !best_model->profiles.empty()) {
        // 分项相似度比对实际得分的 profile（QueryBestMatch 对各 profile 取 max，
        // front 未必是得分的那个）；下标越界（库热替换后模型变短）防御回退 front。
        const std::size_t best_profile_index =
            match.best_profile_index < best_model->profiles.size() ? match.best_profile_index
                                                                   : 0U;
        const std::array<float, 4> similarities = RirMatcher::ComputeFeatureSimilarities(
            aggregate, best_model->profiles[best_profile_index]);
        judgement.feature_scores.rcs_similarity = similarities[0];
        judgement.feature_scores.motion_similarity = similarities[1];
        judgement.feature_scores.polarization_similarity = similarities[2];
        judgement.feature_scores.range_profile_similarity = similarities[3];
      }
      judgement.best_score = match.best_score;
      judgement.runner_up_score = match.runner_up_score;
      judgement.confidence = match.confidence;
      const float margin = judgement.best_score - judgement.runner_up_score;
      const bool margin_ok = margin >= options_.minimum_margin;
      const bool dimensions_ok = valid_dimensions >= kMinValidDimensionsForModel && !motion_only;
      if (judgement.best_score < options_.acceptance_score) {
        // 中译：识别综合得分跌破接受门、结论翻转为未识别——沿触发记录得分与
        // （注：仅覆盖本判定路径；无候选/聚合全维无效路径进入 unknown 不打点，刻意收窄）
        // 四维相似度/质量快照，供识别链排查（每航迹仅在进入未识别的翻转沿打一条）。
        // 标识：状态语义=kUnknown 进入沿；触发条件=best_score<acceptance_score，
        // 分项数值反映当拍窗口聚合后的特征匹配形态。
        if (state.result.state != session::RirRecognitionState::kUnknown) {
          PROJECT_LOG_INFO(
              "[RecognitionTracker] key={} fell below acceptance: best={:.3f} "
              "runner_up={:.3f} model={} sims=(rcs={:.3f},motion={:.3f},pol={:.3f},rp={:.3f}) "
              "qualities=({:.3f},{:.3f},{:.3f},{:.3f}) valid_dims={} obs={} "
              "motion_obs=(v={:.0f},alt={:.0f},a={:.2f},straight={})",
              key, judgement.best_score, judgement.runner_up_score, match.best_model_id,
              judgement.feature_scores.rcs_similarity,
              judgement.feature_scores.motion_similarity,
              judgement.feature_scores.polarization_similarity,
              judgement.feature_scores.range_profile_similarity, aggregate.rcs.quality,
              aggregate.motion.quality, aggregate.polarization.quality,
              aggregate.range_profile.quality, valid_dimensions,
              state.observation_count, aggregate.motion.speed_mps,
              aggregate.motion.altitude_m, aggregate.motion.acceleration_mps2,
              aggregate.motion.is_straight ? 1 : 0);
        }
        judgement.state = session::RirRecognitionState::kUnknown;
      } else if (margin_ok && dimensions_ok) {
        judgement.state = session::RirRecognitionState::kModelConfirmed;
        judgement.target_model = match.best_model_id;
        judgement.target_category = CategoryToPublic(match.best_category_id);
      } else {
        judgement.state = session::RirRecognitionState::kCategoryConfirmed;
        judgement.target_category = CategoryToPublic(match.best_category_id);
      }
    }

    state.result = judgement;
    state.conclusion_time_sec = sim_time_sec;
    state.has_conclusion = true;
    // 首次确认时刻：仅在大类/型号确认时记录一次（用于平均首次确认时间）。
    if (state.first_conclusion_time_sec < 0.0f &&
        (judgement.state == session::RirRecognitionState::kCategoryConfirmed ||
         judgement.state == session::RirRecognitionState::kModelConfirmed)) {
      state.first_conclusion_time_sec = sim_time_sec;
    }
  }
}

void RirTracker::HoldCycle(const std::vector<tracking::RirTrackState>& tracks, float sim_time_sec) {
  for (std::size_t i = 0U; i < tracks.size(); ++i) {
    const tracking::RirTrackState& snapshot = tracks[i];
    const std::uint64_t key = snapshot.association_key;
    const std::unordered_map<std::uint64_t, RirTrackState>::iterator found = tracks_.find(key);
    if (found == tracks_.end()) {
      continue;
    }
    RirTrackState& state = found->second;
    if (state.last_seen_hit_count > 0U && snapshot.hit_count < state.last_seen_hit_count) {
      tracks_.erase(found);
      continue;
    }
    state.last_seen_hit_count = snapshot.hit_count;
    ExpireIfHeld(&state, sim_time_sec);
  }
}

const session::RirRecognitionResult* RirTracker::FindResult(std::uint64_t association_key) const {
  const std::unordered_map<std::uint64_t, RirTrackState>::const_iterator found =
      tracks_.find(association_key);
  return found == tracks_.end() ? nullptr : &found->second.result;
}

session::RirRecognitionCycleSummary RirTracker::BuildSummary(
    const std::vector<tracking::RirTrackState>& tracks) const {
  session::RirRecognitionCycleSummary summary;
  std::uint32_t confirmed_count = 0U;
  std::uint32_t rcs_available = 0U;
  std::uint32_t motion_available = 0U;
  std::uint32_t polarization_available = 0U;
  std::uint32_t range_profile_available = 0U;
  std::uint32_t truth_count = 0U;
  std::uint32_t category_correct = 0U;
  std::uint32_t model_correct = 0U;
  float first_confirmation_sum = 0.0f;
  float confidence_sum = 0.0f;
  for (std::size_t i = 0U; i < tracks.size(); ++i) {
    const tracking::RirTrackState& snapshot = tracks[i];
    const session::RirRecognitionResult* result = FindResult(snapshot.association_key);
    if (result == nullptr || result->state == session::RirRecognitionState::kDisabled) {
      ++summary.disabled_count;
      continue;
    }
    ++summary.participating_track_count;
    switch (result->state) {
      case session::RirRecognitionState::kCategoryConfirmed:
        ++summary.category_confirmed_count;
        break;
      case session::RirRecognitionState::kModelConfirmed:
        ++summary.model_confirmed_count;
        break;
      case session::RirRecognitionState::kUnknown:
        ++summary.unknown_count;
        break;
      default:
        break;
    }
    if (result->state == session::RirRecognitionState::kCategoryConfirmed ||
        result->state == session::RirRecognitionState::kModelConfirmed) {
      ++confirmed_count;
      confidence_sum += result->confidence;
      const std::unordered_map<std::uint64_t, RirTrackState>::const_iterator found =
          tracks_.find(snapshot.association_key);
      if (found != tracks_.end() && found->second.first_conclusion_time_sec > 0.0f &&
          found->second.first_observation_sec > 0.0f) {
        first_confirmation_sum +=
            found->second.first_conclusion_time_sec - found->second.first_observation_sec;
      }
      // 真值准确率：target_name 命中数据库 model_id 时视为有真值（仅统计，不参与识别）。
      const std::unordered_map<std::string, std::string>::const_iterator truth =
          model_categories_.find(snapshot.target_name);
      if (truth != model_categories_.end()) {
        ++truth_count;
        if (result->target_category == CategoryToPublic(truth->second)) {
          ++category_correct;
        }
        if (result->target_model == snapshot.target_name) {
          ++model_correct;
        }
      }
    }
    if (result->feature_scores.rcs_quality > 0.0f) {
      ++rcs_available;
    }
    if (result->feature_scores.motion_quality > 0.0f) {
      ++motion_available;
    }
    if (result->feature_scores.polarization_quality > 0.0f) {
      ++polarization_available;
    }
    if (result->feature_scores.range_profile_quality > 0.0f) {
      ++range_profile_available;
    }
  }
  if (summary.participating_track_count > 0U) {
    const float participating = static_cast<float>(summary.participating_track_count);
    summary.rcs_availability_rate = static_cast<float>(rcs_available) / participating;
    summary.motion_availability_rate = static_cast<float>(motion_available) / participating;
    summary.polarization_availability_rate =
        static_cast<float>(polarization_available) / participating;
    summary.range_profile_availability_rate =
        static_cast<float>(range_profile_available) / participating;
  }
  if (confirmed_count > 0U) {
    summary.mean_confidence = confidence_sum / static_cast<float>(confirmed_count);
    summary.mean_first_confirmation_sec =
        first_confirmation_sum / static_cast<float>(confirmed_count);
  }
  if (truth_count > 0U) {
    summary.has_ground_truth = true;
    summary.category_accuracy =
        static_cast<float>(category_correct) / static_cast<float>(truth_count);
    summary.model_accuracy = static_cast<float>(model_correct) / static_cast<float>(truth_count);
  }
  return summary;
}

RirTracker::Snapshot RirTracker::Capture() const {
  Snapshot snapshot;
  snapshot.active_database_version = active_database_version_;
  snapshot.tracks.reserve(tracks_.size());
  for (const auto& entry : tracks_) {
    snapshot.tracks.push_back(entry.second);
  }
  return snapshot;
}

void RirTracker::Restore(const Snapshot& snapshot) {
  tracks_.clear();
  for (std::size_t i = 0U; i < snapshot.tracks.size(); ++i) {
    tracks_[snapshot.tracks[i].association_key] = snapshot.tracks[i];
  }
  active_database_version_ = snapshot.active_database_version;
}

void RirTracker::Reset() {
  tracks_.clear();
  active_database_version_.clear();
}

}  // namespace recognition
}  // namespace remote_identification_radar
