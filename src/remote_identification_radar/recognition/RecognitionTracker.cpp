/**
 * @file RecognitionTracker.cpp
 * @brief 识别积累与判定状态机实现。
 */

/**
 * ## 过程：识别积累与判决（LRDR 跟踪/识别链 · 第 12 步）
 * - 上游：每周期航迹快照 + 按关联键组织的观测输入（场景目标与上下文）、第 11 步匹配结果。
 * - 本步：每航迹维护 `accumulation_window_sec` 滑动窗口；达到判定门槛
 *   （`min_confirmed_hits` + `min_observation_count`）后对窗口聚合特征做匹配，按
 *   `acceptance_score` / `minimum_margin` / 有效维度数三条件判决 kUnknown /
 *   kCategoryConfirmed / kModelConfirmed；结论在 `result_hold_sec` 内保持，超期置 kStale。
 *   键复用（hit_count 回落）时清积累但保留已有结论。
 * - 下游：`RirController` 输出与验收第 47A 项（类别/型号/置信度）；真值正确率由
 *   `BuildSummary` 汇总，识别结论亦随航迹写入验收行。
 * - 关键参数：`Options` 默认 min_confirmed_hits=5、accumulation_window_sec=10 s、
 *   min_observation_count=3、acceptance_score=0.70、minimum_margin=0.10、
 *   result_hold_sec=30 s、max_range_m=300 km；场景 rir_ground_site_recognition 覆盖为
 *   1 拍 / 10 s / 0.6 / 0.05 / 30 s。
 * - 对应验收项：第 47A 项 综合特征识别功能测试（正确率 >60%）。
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

// 过程：滑窗聚合用的中位数——按值传参（内部排序不改调用方序列），抗单拍离群。
float Median(std::vector<float> values) {
  if (values.empty()) {
    return 0.0f;
  }
  std::sort(values.begin(), values.end());
  return values[values.size() / 2U];
}

// 过程：算术均值；空序列返回 0，避免 0/0。
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

// 过程：有效维度计数（有效且质量>0）；型号确认要求 ≥2 维且不能只有运动维。
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

// 过程：仅运动一维有效时禁止型号确认——运动不足以单独定型号（数据口径）。
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
// 过程：运动子特征（速度/高度/加速度/转弯半径）取中位数抗单拍伪值；峰数取均值后
//       四舍五入成整数。
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
  // 过程：聚合后重算维度掩码——某维若在窗口内从未有效，则该位不置位。
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

// 过程：结论保持期——距最近结论超过 result_hold_sec 置 kStale：数据不再新鲜，
//       但不清空结论（长滑行段仍可读到最后一次判定）。
void RirTracker::ExpireIfHeld(RirTrackState* state, float sim_time_sec) {
  if (state != nullptr && state->has_conclusion && state->conclusion_time_sec >= 0.0f &&
      sim_time_sec - state->conclusion_time_sec > options_.result_hold_sec) {
    state->result.state = session::RirRecognitionState::kStale;
  }
}

// 过程：退出识别模式只清积累（窗口/计数/首次观测时刻），保留已出结论进保持期。
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

// 过程：每周期对每条航迹：取观测 → 组装特征 → 窗口积累 → 门槛判定 → 聚合匹配 → 三态
//       判决；判定不了时按保持期语义处理既有结论。
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
      // 中译：槽位复用导致 hit_count 回退时，只清空累积窗口与计数，**保留上一次结论**
      //       （类别/型号/结论时刻）——否则漏检后重捕会把已确认类别抹成“未知”，
      //       长滑行段更会把结论整段丢失；若新目标不同，重新积累后会覆盖旧结论。
      // 标识：识别结论保持（2026-10-21，A）。
      const session::RirRecognitionResult previous_result = state.result;
      const float previous_conclusion_time = state.conclusion_time_sec;
      const float previous_first_conclusion = state.first_conclusion_time_sec;
      const bool previous_has_conclusion = state.has_conclusion;
      state = RirTrackState{};
      state.association_key = key;
      if (previous_has_conclusion) {
        state.result = previous_result;
        state.conclusion_time_sec = previous_conclusion_time;
        state.first_conclusion_time_sec = previous_first_conclusion;
        state.has_conclusion = true;
      }
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

    // 过程：组装本拍四维特征（RCS/运动/极化/宽带一维像），各维自带有效性与质量。
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
    // 过程：同时累计有效观测数与"已确认航迹上的观测数"（后者是型号确认门槛之一）。
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

    // 过程：先填通用字段（观测数、积累时长、库版本、周期/批号），判决分支只覆盖状态与
    //       得分，保证每拍输出都带得上溯源信息。
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

    // 过程：窗口聚合特征用于匹配——压低单拍噪声对判决的影响（积累的意义所在）。
    const RirFeatureSet aggregate = AggregateWindow(state.window);
    if (aggregate.valid_feature_mask == 0U) {
      // 中译：本拍没有可用的特征维度（窗口被裁空/全部维度失效）**不等于**"目标类别未知"，
      //       已有结论时保持上一次结论，不覆盖成未知。
      // 标识：识别结论保持（2026-10-21，A-补充）。
      if (!state.has_conclusion) {
        judgement.state = session::RirRecognitionState::kUnknown;
        state.result = judgement;
      }
      continue;
    }

    // 过程：特征库匹配——得到最佳型号/大类、best/runner-up 得分与归一化置信度。
    const RirMatchResult match =
        RirMatcher::QueryBestMatch(aggregate, input.context, database, weights);
    // 过程：统计聚合后的有效维数与"仅运动"标志——型号确认要求 ≥2 维且非仅运动。
    const std::uint8_t valid_dimensions = CountValidDimensions(aggregate);
    const bool motion_only = MotionOnly(aggregate);

    // 过程：把四维质量写进结论（验收第 47A 项与排查日志按此展示特征可用性）。
    judgement.valid_feature_mask = aggregate.valid_feature_mask;
    judgement.feature_scores.rcs_quality = aggregate.rcs.quality;
    judgement.feature_scores.motion_quality = aggregate.motion.quality;
    judgement.feature_scores.polarization_quality = aggregate.polarization.quality;
    judgement.feature_scores.range_profile_quality = aggregate.range_profile.quality;

    if (!match.has_candidates) {
      // 中译：本拍所有模板 profile 都不可适用（视角/量程落在 profile 适用域之外）时，
      //       同样保持上一次结论——单拍"无适用模板"不代表目标类别应回退为未知。
      // 标识：识别结论保持（2026-10-21，A-补充）。
      if (!state.has_conclusion) {
        judgement.state = session::RirRecognitionState::kUnknown;
        state.result = judgement;
        continue;
      }
      continue;
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
      // 过程：判决三态——得分低于 acceptance_score 判未知；裕度 ≥ minimum_margin 且维度
      //       达标判型号确认；否则退一级，只判大类确认。
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

    // 过程：结论落盘并刷新结论时刻；首次大类/型号确认时刻只记一次（用于平均首次确认
    //       时间统计）。
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

// 过程：保持周期（本拍无识别调度/无观测）——只做键重分配清理与结论保持期推进，不积累。
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

// 过程：按关联键查结论；无记录返回 nullptr（不造空结论，避免"看起来有结论"）。
const session::RirRecognitionResult* RirTracker::FindResult(std::uint64_t association_key) const {
  const std::unordered_map<std::uint64_t, RirTrackState>::const_iterator found =
      tracks_.find(association_key);
  return found == tracks_.end() ? nullptr : &found->second.result;
}

// 过程：周期汇总——参与航迹数、各判决态计数、四维可用率、平均置信度与平均首次确认时间。
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

// 过程：运行态快照 = 每键积累窗口 + 结论 + 库版本（replay 用值迁移）。
RirTracker::Snapshot RirTracker::Capture() const {
  Snapshot snapshot;
  snapshot.active_database_version = active_database_version_;
  snapshot.tracks.reserve(tracks_.size());
  for (const auto& entry : tracks_) {
    snapshot.tracks.push_back(entry.second);
  }
  return snapshot;
}

// 过程：以快照整值恢复（先清后建），恢复后的判决流程与回放前一致。
void RirTracker::Restore(const Snapshot& snapshot) {
  tracks_.clear();
  for (std::size_t i = 0U; i < snapshot.tracks.size(); ++i) {
    tracks_[snapshot.tracks[i].association_key] = snapshot.tracks[i];
  }
  active_database_version_ = snapshot.active_database_version;
}

// 过程：清空全部识别运行态（结论与积累一并丢弃，库版本字符串清空）。
void RirTracker::Reset() {
  tracks_.clear();
  active_database_version_.clear();
}

}  // namespace recognition
}  // namespace remote_identification_radar
