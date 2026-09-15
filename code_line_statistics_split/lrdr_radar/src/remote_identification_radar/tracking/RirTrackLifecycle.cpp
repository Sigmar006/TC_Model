/**
 * @file RirTrackLifecycle.cpp
 * @brief RIR 轻量跟踪子集的航迹生命周期管理器实现（阶段 2-T T3，N3 池化）。
 */

#include "remote_identification_radar/tracking/RirTrackLifecycle.h"

#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "common/logging/ProjectLog.h"
#include "common/tracking/TrackLifecyclePromote.h"

namespace remote_identification_radar {
namespace tracking {

namespace {

/** @brief 生命周期保留键：未关联量测。 */
constexpr std::uint64_t kUnassociatedKey = 0U;

/** @brief 加速度估计的最小有效步长（s）。 */
constexpr float kMinimumAccelerationDtSec = 1.0e-6f;

/**
 * @brief 对数等距 IMM 模型噪声差异系数：以配置 q 为低端锚点，按
 *        q·10^(i/(N-1)) 排列（低噪声到高噪声）；配置 q=1 时退化为
 *        与 AR 缺省口径同源的 {1, 10, ...}。
 */
std::vector<float> BuildImmNoiseDiffCoeffs(std::uint32_t model_count_hint, float base_q) {
  const std::size_t model_count =
      static_cast<std::size_t>(model_count_hint < 2U ? 2U : model_count_hint);
  std::vector<float> coeffs;
  coeffs.reserve(model_count);
  for (std::size_t i = 0U; i < model_count; ++i) {
    coeffs.push_back(base_q * std::pow(10.0f, static_cast<float>(i) /
                                                   static_cast<float>(model_count - 1U)));
  }
  return coeffs;
}

}  // namespace

RirTrackLifecycle::RirTrackLifecycle(RirLifecycleConfig config, RirTrackFilterConfig filter_config)
    : lifecycle_config_(config), filter_config_(filter_config), filter_(filter_config),
      imm_enabled_(config.enable_imm_lifecycle) {}

void RirTrackLifecycle::Update(const RirCycleContext& cycle,
                               const std::vector<RirTrackMeasurement>& measurements) {
  if (!std::isfinite(cycle.dt_sec) || cycle.dt_sec <= 0.0f) {
    // 中译：周期时间步长非法，本周期航迹更新被跳过且状态不变。
    // 标识：时间保护——无效 dt 不推进状态机与滤波器，防止协方差传播被污染。
    PROJECT_LOG_ERROR(
        "[RirTrackLifecycle] invalid cycle dt_sec={}, update skipped with no state changes.",
        cycle.dt_sec);
    return;
  }

  // 同键多量测取最后一条（与 AR 子集一致）；按输入顺序建工作单元，
  // 保证同周期多个新航迹的 track_id 分配与输入顺序确定一致。
  std::unordered_map<std::uint64_t, std::size_t> measurement_slot_by_key;
  std::vector<const RirTrackMeasurement*> unique_measurements;
  std::unordered_set<std::uint64_t> hit_keys;
  measurement_slot_by_key.reserve(measurements.size());
  unique_measurements.reserve(measurements.size());
  hit_keys.reserve(measurements.size());
  for (const RirTrackMeasurement& measurement : measurements) {
    if (measurement.association_key == kUnassociatedKey) {
      // 中译：忽略未分配关联键的量测。
      // 标识：输入契约——生命周期只接受关联器已回填键的量测。
      PROJECT_LOG_WARN(
          "[RirTrackLifecycle] ignored measurement with association_key=0 at source_index={}",
          measurement.source_index);
      continue;
    }
    const auto existing = measurement_slot_by_key.find(measurement.association_key);
    if (existing != measurement_slot_by_key.end()) {
      // 中译：同一周期内关联键 {} 出现重复量测，暂存更新使用最后一条。
      // 标识：输入去重——同键多量测取最后一条，防止一次更新写多次。
      PROJECT_LOG_WARN(
          "[RirTrackLifecycle] duplicate measurement for association_key={} in one cycle, use "
          "last measurement for staged update",
          measurement.association_key);
      unique_measurements[existing->second] = &measurement;
    } else {
      measurement_slot_by_key[measurement.association_key] = unique_measurements.size();
      unique_measurements.push_back(&measurement);
    }
    hit_keys.insert(measurement.association_key);
  }

  // 值快照工作集（工作单元按值流转，回写阶段经池化指针落盘）。
  std::map<std::uint64_t, RirTrackState> snapshots;
  for (const auto& entry : tracks_) {
    snapshots[entry.first] = *entry.second;
  }
  std::vector<WorkItem> work_items;
  work_items.reserve(snapshots.size() + unique_measurements.size());

  for (const RirTrackMeasurement* measurement : unique_measurements) {
    const std::uint64_t key = measurement->association_key;
    WorkItem work_item;
    work_item.association_key = key;
    work_item.kind = WorkItemKind::kHit;
    work_item.measurement = measurement;

    const std::map<std::uint64_t, RirTrackState>::const_iterator found = snapshots.find(key);
    if (found == snapshots.end()) {
      RirTrackState track;
      track.association_key = key;
      track.track_id = next_track_id_++;
      track.batch_id = cycle.batch_id;
      track.status = RirTrackStatus::kTentative;
      track.first_cycle = cycle.cycle_index;
      work_item.track_before = track;
      work_item.status_before = RirTrackStatus::kTentative;
      work_item.track_existed_before_cycle = false;
    } else {
      work_item.track_before = found->second;
      work_item.status_before = found->second.status;
      work_item.track_existed_before_cycle = true;
    }
    // IMM 双路径（N5）：confirmed 命中既有航迹时挂 IMM 运行态（以周期前状态播种）。
    if (ShouldUseImmForMeasurement(work_item.track_existed_before_cycle,
                                   work_item.status_before,
                                   measurement->matched_existing_track)) {
      work_item.imm_filter = GetOrCreateImmFilter(key, work_item.track_before.gaussian_state);
    }
    work_items.push_back(work_item);
  }

  for (const auto& entry : snapshots) {
    if (hit_keys.find(entry.first) != hit_keys.end()) {
      continue;
    }
    WorkItem work_item;
    work_item.association_key = entry.first;
    work_item.kind = WorkItemKind::kMiss;
    work_item.track_before = entry.second;
    work_item.status_before = entry.second.status;
    work_item.track_existed_before_cycle = true;
    // 失配周期：confirmed 航迹已有 IMM 态则仅预测推进（AR confirmed-only 策略）。
    if (ShouldUseImmForMiss(work_item.status_before)) {
      work_item.imm_filter = FindImmFilter(entry.first);
    }
    work_items.push_back(work_item);
  }

  std::vector<WorkResult> results;
  results.reserve(work_items.size());
  for (const WorkItem& work_item : work_items) {
    WorkResult result;
    result.association_key = work_item.association_key;
    result.track_after = work_item.track_before;
    RirTrackState& track = result.track_after;

    if (work_item.kind == WorkItemKind::kHit) {
      const RirTrackMeasurement& measurement = *work_item.measurement;
      // 加速度差分时间基准=距上次命中的实际经过时间：滑行（失配）周期 CV 外推
      // 速度不变，重命中差分须按 (周期差)·dt 折算，否则机动目标加速度按单周期
      // dt 膨胀；新航迹/重置周期无差分基准，该值在重置分支被显式置零覆盖。
      float acceleration_dt_sec = cycle.dt_sec;
      if (cycle.cycle_index > track.last_update_cycle) {
        acceleration_dt_sec =
            static_cast<float>(cycle.cycle_index - track.last_update_cycle) * cycle.dt_sec;
      }
      track.last_update_cycle = cycle.cycle_index;
      track.hit_count += 1U;
      track.miss_count = 0U;

      track.position = measurement.position;
      if (measurement.external_target_id != 0U) {
        track.external_target_id = measurement.external_target_id;
      }
      if (!measurement.target_name.empty()) {
        track.target_name = measurement.target_name;
      }
      track.rcs = measurement.rcs;

      PromoteState(&track, cycle.cycle_index, true);
      // track.velocity 在此仍持有上周期后验速度（速度唯一来源是滤波后验，速度种子
      // 不写入状态）：ApplyHitFilter 以其为基准做周期间差分得到物理加速度。
      ApplyHitFilter(&track, measurement, work_item.status_before, cycle.dt_sec,
                     acceleration_dt_sec, work_item.imm_filter);
    } else {
      const bool should_recycle = PromoteState(&track, cycle.cycle_index, false);
      if (should_recycle) {
        result.should_recycle = true;
      } else {
        ApplyMissPredict(&track, track.velocity, cycle.dt_sec, work_item.imm_filter);
      }
    }
    results.push_back(result);
  }

  // 回写（N3）：回收键出表并归还池；既有航迹经原槽位整值回写；
  // 新航迹从池申请槽位（ResetForReuse 清业务字段、保留槽位代次）。
  for (const WorkResult& result : results) {
    const std::map<std::uint64_t, RirTrackState*>::iterator found =
        tracks_.find(result.association_key);
    const bool existed = found != tracks_.end();
    if (result.should_recycle) {
      if (existed) {
        RecycleTrack(found->second);
        tracks_.erase(found);
        // IMM 运行态随航迹回收销毁（键不复用，无残留引用）。
        imm_filters_by_key_.erase(result.association_key);
      }
      continue;
    }
    if (existed) {
      // 既有槽位：整值回写（快照内 generation 未被修改，随值原样落盘）。
      *found->second = result.track_after;
      continue;
    }
    RirTrackState* slot = pool_.Acquire();
    if (slot == nullptr) {
      // 中译：对象池申请失败，本周期放弃新建航迹（关联键）。
      // 标识：内存池耗尽——新航迹被丢弃，既有航迹不受影响；
      //       调用方可经池容量配置或内存排查恢复。
      PROJECT_LOG_ERROR(
          "[RirTrackLifecycle] pool acquire failed, new track dropped for association_key={}",
          result.association_key);
      continue;
    }
    ResetForReuse(*slot);
    const std::uint32_t slot_generation = slot->generation;
    *slot = result.track_after;
    slot->generation = slot_generation;
    tracks_[result.association_key] = slot;
  }
  last_cycle_index_ = cycle.cycle_index;
}

RirTrackSnapshotList RirTrackLifecycle::BuildTrackSnapshots() const {
  RirTrackSnapshotList snapshots;
  snapshots.reserve(tracks_.size());
  for (const auto& entry : tracks_) {
    RirTrackState snapshot = *entry.second;
    snapshot.association_key = entry.first;
    snapshots.push_back(snapshot);
  }
  return snapshots;
}

std::vector<RirTrackSeed> RirTrackLifecycle::BuildAssociationSeeds() const {
  std::vector<RirTrackSeed> seeds;
  seeds.reserve(tracks_.size());
  for (const auto& entry : tracks_) {
    const RirTrackState& track = *entry.second;
    RirTrackSeed seed;
    seed.association_key = entry.first;
    seed.has_position = true;
    seed.position = track.position;
    seed.has_gaussian_state = true;
    seed.gaussian_state = track.gaussian_state;
    seeds.push_back(seed);
  }
  return seeds;
}

const RirTrackState* RirTrackLifecycle::FindTrack(std::uint64_t association_key) const {
  const std::map<std::uint64_t, RirTrackState*>::const_iterator found =
      tracks_.find(association_key);
  if (found == tracks_.end()) {
    return nullptr;
  }
  return found->second;
}

void RirTrackLifecycle::Reset() {
  for (auto& entry : tracks_) {
    ResetForReuse(*entry.second);
    pool_.Release(entry.second);
  }
  tracks_.clear();
  imm_filters_by_key_.clear();
  next_track_id_ = 1U;
  last_cycle_index_ = 0U;
}

void RirTrackLifecycle::UpdateConfig(RirLifecycleConfig lifecycle_config,
                                     RirTrackFilterConfig filter_config) {
  lifecycle_config_ = lifecycle_config;
  filter_config_ = filter_config;
  imm_enabled_ = lifecycle_config.enable_imm_lifecycle;
  filter_.UpdateConfig(filter_config);

  // 与 AR SyncRuntimeTuning 同口径：运行期改配在线同步已建 IMM 运行态的每模型 q 与
  // 转移矩阵（CV KF 经 filter_.UpdateConfig 同步）；模型数变化无法原位重调 → 丢弃
  // 该运行态，下次 confirmed 命中按航迹当前高斯状态惰性重建（与 replay 恢复同语义）。
  const RirImmFilter::Config imm_config = BuildImmFilterConfig();
  for (std::unordered_map<std::uint64_t, std::unique_ptr<RirImmFilter>>::iterator it =
           imm_filters_by_key_.begin();
       it != imm_filters_by_key_.end();) {
    if (it->second != nullptr && it->second->UpdateRuntimeTuning(imm_config)) {
      ++it;
      continue;
    }
    // 中译：模型数变化后关联键 {} 的 IMM 运行态无法原位重调，已丢弃；
    // 下次 confirmed 命中时按新模型集惰性重建。
    // 标识：配置热更新——运行态丢弃语义与 replay 恢复一致，航迹本身不受影响。
    PROJECT_LOG_WARN(
        "[RirTrackLifecycle] imm runtime state dropped for association_key={} after model "
        "count change; lazily rebuilt on next confirmed hit",
        it->first);
    imm_filters_by_key_.erase(it++);
  }
}

RirLifecycleRuntimeState RirTrackLifecycle::CaptureRuntimeState() const {
  RirLifecycleRuntimeState state;
  state.next_track_id = next_track_id_;
  state.last_cycle_index = last_cycle_index_;
  state.tracks = BuildTrackSnapshots();
  return state;
}

void RirTrackLifecycle::RestoreRuntimeState(const RirLifecycleRuntimeState& state) {
  for (auto& entry : tracks_) {
    ResetForReuse(*entry.second);
    pool_.Release(entry.second);
  }
  tracks_.clear();
  // IMM 运行态不在 replay 快照内（AR 同口径）：清空后由下次 confirmed 命中
  // 按航迹当前高斯状态惰性重建。
  imm_filters_by_key_.clear();
  for (const RirTrackState& track : state.tracks) {
    RirTrackState* slot = pool_.Acquire();
    if (slot == nullptr) {
      // 中译：恢复运行态时对象池申请失败，该航迹未被恢复（关联键）。
      // 标识：内存池耗尽——恢复不完整，后续周期可经日志定位缺失航迹。
      PROJECT_LOG_ERROR(
          "[RirTrackLifecycle] pool acquire failed during restore, track skipped for "
          "association_key={}",
          track.association_key);
      continue;
    }
    ResetForReuse(*slot);
    // replay 以快照为权威：整值回写（含 generation）。
    *slot = track;
    tracks_[track.association_key] = slot;
  }
  next_track_id_ = state.next_track_id;
  last_cycle_index_ = state.last_cycle_index;
}

bool RirTrackLifecycle::PromoteState(RirTrackState* track, std::uint32_t cycle_index,
                                     bool hit_this_cycle) {
  oneq::common::tracking::TrackLifecycleCounters counters;
  counters.status = static_cast<oneq::common::tracking::TrackLifecyclePhase>(
      static_cast<std::uint8_t>(track->status));
  counters.hit_count = track->hit_count;
  counters.miss_count = track->miss_count;
  counters.last_update_cycle = track->last_update_cycle;
  counters.generation = track->generation;

  oneq::common::tracking::TrackLifecyclePromotePolicy policy;
  policy.confirm_hits = lifecycle_config_.confirm_hits;
  policy.max_miss_before_lost = lifecycle_config_.max_miss_before_lost;
  policy.max_lost_cycles = lifecycle_config_.max_lost_cycles;
  policy.extra_miss_tolerance = 0U;
  policy.suppress_confirm = false;
  policy.recycle_as_status = false;

  const bool should_recycle = oneq::common::tracking::PromoteTrackLifecycle(
      &counters, cycle_index, hit_this_cycle, policy);

  track->status = static_cast<RirTrackStatus>(static_cast<std::uint8_t>(counters.status));
  track->hit_count = counters.hit_count;
  track->miss_count = counters.miss_count;
  return should_recycle;
}

void RirTrackLifecycle::ApplyGaussianState(RirTrackState* track, const RirGaussianState& state,
                                           const Eigen::Vector3f& previous_velocity,
                                           float dt_sec) const {
  track->gaussian_state = state;
  track->position = Eigen::Vector3f(state.mean(0), state.mean(2), state.mean(4));
  track->velocity = Eigen::Vector3f(state.mean(1), state.mean(3), state.mean(5));
  track->speed = track->velocity.norm();
  if (dt_sec > kMinimumAccelerationDtSec) {
    track->acceleration = (track->velocity - previous_velocity) / dt_sec;
  }
  track->acceleration_mps2 = track->acceleration.norm();
}

void RirTrackLifecycle::ApplyHitFilter(RirTrackState* track, const RirTrackMeasurement& measurement,
                                       RirTrackStatus status_before, float dt_sec,
                                       float acceleration_dt_sec,
                                       RirImmFilter* imm_filter) const {
  // 加速度 = 滤波后验速度的周期间差分（物理加速度口径）：差分基准取进更新前的
  // 上周期后验速度（按值留存，避免与 ApplyGaussianState 内的回写别名），分母取
  // 距上次命中的实际经过时间（滑行周期后重命中不按单周期 dt 膨胀）。(重)置周期
  // （建轨/失跟重捕）滤波以零速均值 + 速度无知先验重初始化（2026-08-31 去真值
  // 种子；速度唯一来源是滤波后验，重捕后速度重新收敛）。
  const Eigen::Vector3f velocity_before_update = track->velocity;
  const bool reset_filter_on_hit =
      !measurement.matched_existing_track || status_before == RirTrackStatus::kLost;
  if (imm_filter != nullptr && imm_filter->IsValid()) {
    if (reset_filter_on_hit) {
      // 防御分支（confirmed-only 激活策略下不可达）：IMM 命中重置退化为 CV 初始化，
      // 与 AR ApplyKalmanHitUpdate 同构。
      ApplyGaussianState(track, filter_.Initialize(measurement), velocity_before_update,
                         acceleration_dt_sec);
      track->acceleration.setZero();
      track->acceleration_mps2 = 0.0f;
      return;
    }
    imm_filter->Process(measurement.position, dt_sec, measurement.measurement_covariance);
    ApplyGaussianState(track, imm_filter->GetCombinedState(), velocity_before_update,
                       acceleration_dt_sec);
    return;
  }
  if (reset_filter_on_hit) {
    ApplyGaussianState(track, filter_.Initialize(measurement), velocity_before_update,
                       acceleration_dt_sec);
    track->acceleration.setZero();
    track->acceleration_mps2 = 0.0f;
    return;
  }

  const RirGaussianState predicted = filter_.Predict(track->gaussian_state, dt_sec);
  const RirKalmanUpdateResult update_result = filter_.Update(predicted, measurement);
  ApplyGaussianState(track, update_result.posterior, velocity_before_update, acceleration_dt_sec);
}

void RirTrackLifecycle::ApplyMissPredict(RirTrackState* track,
                                         const Eigen::Vector3f& previous_velocity, float dt_sec,
                                         RirImmFilter* imm_filter) const {
  if (imm_filter != nullptr && imm_filter->IsValid()) {
    imm_filter->Predict(dt_sec);
    ApplyGaussianState(track, imm_filter->GetCombinedState(), previous_velocity, dt_sec);
    return;
  }
  ApplyGaussianState(track, filter_.Predict(track->gaussian_state, dt_sec), previous_velocity,
                     dt_sec);
}

bool RirTrackLifecycle::ShouldUseImmForMeasurement(bool track_existed_before_cycle,
                                                   RirTrackStatus status_before,
                                                   bool matched_existing_track) const {
  if (!imm_enabled_) {
    return false;
  }
  // RIR 仅实现 confirmed-only 激活（AR kAllTracks 策略不迁）。
  return track_existed_before_cycle && status_before == RirTrackStatus::kConfirmed &&
         matched_existing_track;
}

bool RirTrackLifecycle::ShouldUseImmForMiss(RirTrackStatus status_before) const {
  if (!imm_enabled_) {
    return false;
  }
  return status_before == RirTrackStatus::kConfirmed;
}

RirImmFilter::Config RirTrackLifecycle::BuildImmFilterConfig() const {
  RirImmFilter::Config imm_config;
  const std::uint32_t model_count_hint =
      lifecycle_config_.model_count_hint < 2U ? 2U : lifecycle_config_.model_count_hint;
  imm_config.model_noise_diff_coeffs =
      BuildImmNoiseDiffCoeffs(model_count_hint, filter_config_.process_noise_diff_coeff);
  imm_config.transition_diagonal_probability = 0.95f;
  return imm_config;
}

RirImmFilter* RirTrackLifecycle::GetOrCreateImmFilter(std::uint64_t association_key,
                                                      const RirGaussianState& initial_state) {
  const std::unordered_map<std::uint64_t, std::unique_ptr<RirImmFilter>>::iterator found =
      imm_filters_by_key_.find(association_key);
  if (found != imm_filters_by_key_.end()) {
    return found->second.get();
  }

  std::unique_ptr<RirImmFilter> filter(new RirImmFilter(BuildImmFilterConfig()));
  filter->Initialize(initial_state);
  RirImmFilter* filter_ptr = filter.get();
  imm_filters_by_key_[association_key] = std::move(filter);
  return filter_ptr;
}

RirImmFilter* RirTrackLifecycle::FindImmFilter(std::uint64_t association_key) const {
  const std::unordered_map<std::uint64_t, std::unique_ptr<RirImmFilter>>::const_iterator found =
      imm_filters_by_key_.find(association_key);
  if (found == imm_filters_by_key_.end()) {
    return nullptr;
  }
  return found->second.get();
}

void RirTrackLifecycle::ResetForReuse(RirTrackState& track) {
  track.association_key = 0U;
  track.track_id = 0U;
  track.batch_id = 0U;
  track.external_target_id = 0U;
  track.target_name.clear();
  track.status = RirTrackStatus::kTentative;
  track.first_cycle = 0U;
  track.last_update_cycle = 0U;
  track.miss_count = 0U;
  track.hit_count = 0U;
  track.position.setZero();
  track.velocity.setZero();
  track.acceleration.setZero();
  track.speed = 0.0f;
  track.acceleration_mps2 = 0.0f;
  track.rcs = 0.0f;
  track.gaussian_state = RirGaussianState();
  // generation 保持不清零：对象复用代次单调递增（与 AR ResetForReuse 一致）。
}

void RirTrackLifecycle::RecycleTrack(RirTrackState* track) {
  track->generation += 1U;
  ResetForReuse(*track);
  pool_.Release(track);
}

}  // namespace tracking
}  // namespace remote_identification_radar
