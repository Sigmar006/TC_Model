#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "airborne_radar/signal/tracking/ImmFilter.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "common/logging/ProjectLog.h"
#include "common/tracking/TrackLifecyclePromote.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

namespace {

struct TrackLifecycleRuntimeSnapshot {
  std::uint64_t next_track_id{1U};
  std::uint32_t last_cycle_index{0U};
  std::vector<std::pair<std::uint64_t, TrackState>> tracks_by_key{};
  std::vector<std::pair<std::uint64_t, ImmFilter>> imm_filters_by_key{};
};

/**
 * @brief 将位置量测写入滤波观测向量。
 * @param measurement 当前量测。
 * @return 供滤波器消费的观测向量。
 */
MeasurementVector BuildMeasurementVector(const TrackMeasurement& measurement) {
  MeasurementVector z;
  z(0) = measurement.raw_measurement.position(0);
  z(1) = measurement.raw_measurement.position(1);
  z(2) = measurement.raw_measurement.position(2);
  return z;
}
void UpdatePredictorConfigIfSupported(IKalmanPredictor* predictor, float noise_diff_coeff) {
  if (predictor == nullptr || std::isfinite(noise_diff_coeff) == 0 || noise_diff_coeff <= 0.0f) {
    return;
  }
  KalmanPredictorConfig config;
  config.noise_diff_coeff = noise_diff_coeff;
  predictor->UpdateConfig(config);
}

void UpdateUpdaterConfigIfSupported(IKalmanUpdater* updater, float measurement_noise_std) {
  if (updater == nullptr || std::isfinite(measurement_noise_std) == 0 ||
      measurement_noise_std <= 0.0f) {
    return;
  }
  KalmanUpdaterConfig config;
  config.measurement_noise_std = measurement_noise_std;
  updater->UpdateConfig(config);
}

bool IsValidTransitionProbability(const Eigen::MatrixXf& transition_probability) {
  if (transition_probability.rows() <= 0 ||
      transition_probability.rows() != transition_probability.cols()) {
    return false;
  }
  for (Eigen::Index r = 0; r < transition_probability.rows(); ++r) {
    float row_sum = 0.0f;
    for (Eigen::Index c = 0; c < transition_probability.cols(); ++c) {
      const float value = transition_probability(r, c);
      if (std::isfinite(value) == 0 || value < 0.0f || value > 1.0f) {
        return false;
      }
      row_sum += value;
    }
    if (std::fabs(row_sum - 1.0f) > 1.0e-3f) {
      return false;
    }
  }
  return true;
}

bool IsValidInitialWeights(const Eigen::VectorXf& initial_weights) {
  if (initial_weights.size() <= 0) {
    return false;
  }
  float sum = 0.0f;
  for (Eigen::Index i = 0; i < initial_weights.size(); ++i) {
    const float value = initial_weights(i);
    if (std::isfinite(value) == 0 || value < 0.0f || value > 1.0f) {
      return false;
    }
    sum += value;
  }
  return std::fabs(sum - 1.0f) <= 1.0e-3f;
}

}  // namespace

TrackLifecycleManager::TrackLifecycleManager(ITrackPool& pool, const LifecycleConfig& config,
                                             IKalmanPredictor* predictor, IKalmanUpdater* updater)
    : pool_(&pool),
      config_(config),
      tracks_by_key_(),
      kalman_predictor_(predictor),
      kalman_updater_(updater) {}

TrackLifecycleManager::TrackLifecycleManager(ITrackPool& pool, const LifecycleConfig& config,
                                             const std::vector<IKalmanPredictor*>& imm_predictors,
                                             const std::vector<IKalmanUpdater*>& imm_updaters,
                                             const Eigen::MatrixXf& imm_transition_probability,
                                             const Eigen::VectorXf& imm_initial_weights)
    : pool_(&pool),
      config_(config),
      tracks_by_key_(),
      kalman_predictor_(imm_predictors.empty() ? nullptr : imm_predictors.front()),
      kalman_updater_(imm_updaters.empty() ? nullptr : imm_updaters.front()),
      imm_predictors_(imm_predictors),
      imm_updaters_(imm_updaters),
      imm_transition_probability_(imm_transition_probability),
      imm_initial_weights_(imm_initial_weights) {
  const int num_models = static_cast<int>(imm_predictors_.size());
  if (num_models <= 0 || imm_updaters_.size() != imm_predictors_.size()) {
    kalman_predictor_ = nullptr;
    kalman_updater_ = nullptr;
    imm_predictors_.clear();
    imm_updaters_.clear();
    imm_transition_probability_.resize(0, 0);
    imm_initial_weights_.resize(0);
    return;
  }

  if (imm_transition_probability_.rows() != num_models ||
      imm_transition_probability_.cols() != num_models) {
    imm_transition_probability_ = Eigen::MatrixXf::Constant(
        num_models, num_models, num_models > 1 ? 0.05f / static_cast<float>(num_models - 1) : 1.0f);
    imm_transition_probability_.diagonal().setConstant(num_models > 1 ? 0.95f : 1.0f);
  }

  if (imm_initial_weights_.size() != num_models) {
    imm_initial_weights_ =
        Eigen::VectorXf::Constant(num_models, 1.0f / static_cast<float>(num_models));
  }
}

TrackLifecycleManager::~TrackLifecycleManager() = default;

TrackLifecycleRuntimeState TrackLifecycleManager::CaptureRuntimeState() const {
  TrackLifecycleRuntimeState state;
  state.owner_identity = this;
  state.schema_version = 1U;

  std::shared_ptr<TrackLifecycleRuntimeSnapshot> snapshot(new TrackLifecycleRuntimeSnapshot());
  snapshot->next_track_id = next_track_id_;
  snapshot->last_cycle_index = last_cycle_index_;
  snapshot->tracks_by_key.reserve(tracks_by_key_.size());
  for (std::unordered_map<std::uint64_t, TrackState*>::const_iterator it = tracks_by_key_.begin();
       it != tracks_by_key_.end(); ++it) {
    if (it->second != nullptr) {
      snapshot->tracks_by_key.push_back(std::make_pair(it->first, *it->second));
    }
  }
  snapshot->imm_filters_by_key.reserve(imm_filters_by_key_.size());
  for (std::unordered_map<std::uint64_t, std::unique_ptr<ImmFilter>>::const_iterator it =
           imm_filters_by_key_.begin();
       it != imm_filters_by_key_.end(); ++it) {
    if (it->second != nullptr) {
      snapshot->imm_filters_by_key.push_back(std::make_pair(it->first, *it->second));
    }
  }
  state.opaque = snapshot;
  return state;
}

void TrackLifecycleManager::RestoreRuntimeState(const TrackLifecycleRuntimeState& state) {
  if (state.schema_version != 1U) {
    // 中译：运行状态恢复被拒绝：快照结构与本实例不匹配。
    // 标识：回滚保护——结构版本不符时拒绝恢复，防止错误状态写入。
    PROJECT_LOG_ERROR(
        "[TrackLifecycleManager] runtime state restore rejected because snapshot schema does not "
        "match this instance.");
    return;
  }

  const std::shared_ptr<TrackLifecycleRuntimeSnapshot> snapshot =
      std::static_pointer_cast<TrackLifecycleRuntimeSnapshot>(state.opaque);
  if (snapshot == nullptr) {
    return;
  }

  if (pool_ != nullptr) {
    for (std::unordered_map<std::uint64_t, TrackState*>::iterator it = tracks_by_key_.begin();
         it != tracks_by_key_.end(); ++it) {
      pool_->Release(it->second);
    }
  }
  tracks_by_key_.clear();
  imm_filters_by_key_.clear();

  next_track_id_ = snapshot->next_track_id;
  last_cycle_index_ = snapshot->last_cycle_index;

  for (std::vector<std::pair<std::uint64_t, TrackState>>::const_iterator it =
           snapshot->tracks_by_key.begin();
       it != snapshot->tracks_by_key.end(); ++it) {
    TrackState* track = pool_ != nullptr ? pool_->Acquire() : nullptr;
    if (track == nullptr) {
      // 中译：恢复运行状态时未能为关联键 {} 获取航迹对象。
      // 标识：对象池耗尽——恢复跳过该航迹，快照中的航迹可能丢失。
      PROJECT_LOG_ERROR(
          "[TrackLifecycleManager] runtime state restore failed to acquire track for "
          "association_key={}",
          it->first);
      continue;
    }
    *track = it->second;
    tracks_by_key_[it->first] = track;
  }
  for (std::vector<std::pair<std::uint64_t, ImmFilter>>::const_iterator it =
           snapshot->imm_filters_by_key.begin();
       it != snapshot->imm_filters_by_key.end(); ++it) {
    imm_filters_by_key_[it->first].reset(new ImmFilter(it->second));
  }
  snapshot_emitter_.Refresh(tracks_by_key_);
}

void TrackLifecycleManager::SyncRuntimeTuning(const LifecycleConfig& lifecycle_config,
                                              float kalman_noise_diff_coeff,
                                              float kalman_measurement_noise_std,
                                              const std::vector<float>& imm_model_noise_diff_coeffs,
                                              const Eigen::MatrixXf& imm_transition_probability,
                                              const Eigen::VectorXf& imm_initial_weights) {
  // 整体赋值（收敛 AR-OQ-2）：将 LifecycleConfig 全量覆写至 config_，
  // 未来新增任何可同步字段都会随整体赋值自动覆盖，无需逐字段维护。
  //
  // 整体赋值安全的原因：
  //   1. track_pool_thread_safety_mode 进入 LifecycleConfigSignature，其变化触发
  //      ShouldRebuildLifecycleAssembly 的重建路径（而非本同步路径），故本路径上
  //      lifecycle_config.track_pool_thread_safety_mode 与 config_ 内的值恒等。
  //   2. 本管理器从不读取 config_.track_pool_thread_safety_mode，即便被覆盖也无副作用。
  config_ = lifecycle_config;

  UpdatePredictorConfigIfSupported(kalman_predictor_, kalman_noise_diff_coeff);
  UpdateUpdaterConfigIfSupported(kalman_updater_, kalman_measurement_noise_std);

  for (std::size_t i = 0; i < imm_predictors_.size(); ++i) {
    const float model_noise_diff_coeff = i < imm_model_noise_diff_coeffs.size()
                                             ? imm_model_noise_diff_coeffs[i]
                                             : kalman_noise_diff_coeff;
    UpdatePredictorConfigIfSupported(imm_predictors_[i], model_noise_diff_coeff);
  }
  for (std::size_t i = 0; i < imm_updaters_.size(); ++i) {
    // IMM 更新器 std 仅作量测协方差缺失时的回退 R（正常命中走逐量测动态 R）。
    UpdateUpdaterConfigIfSupported(imm_updaters_[i], kalman_measurement_noise_std);
  }

  const bool transition_shape_match =
      imm_transition_probability.rows() == static_cast<Eigen::Index>(imm_predictors_.size()) &&
      imm_transition_probability.cols() == static_cast<Eigen::Index>(imm_predictors_.size());
  if (transition_shape_match && IsValidTransitionProbability(imm_transition_probability)) {
    imm_transition_probability_ = imm_transition_probability;
  }
  const bool weights_shape_match =
      imm_initial_weights.size() == static_cast<Eigen::Index>(imm_predictors_.size());
  if (weights_shape_match && IsValidInitialWeights(imm_initial_weights)) {
    imm_initial_weights_ = imm_initial_weights;
  }

  ImmConfig imm_config;
  imm_config.transition_probability = imm_transition_probability_;
  imm_config.initial_weights = imm_initial_weights_;
  for (std::unordered_map<std::uint64_t, std::unique_ptr<ImmFilter>>::iterator it =
           imm_filters_by_key_.begin();
       it != imm_filters_by_key_.end(); ++it) {
    if (it->second == nullptr) {
      continue;
    }
    it->second->UpdateRuntimeTuning(imm_config, imm_predictors_, imm_updaters_);
  }
}

bool TrackLifecycleManager::IsImmEnabled() const {
  return !imm_predictors_.empty() && imm_predictors_.size() == imm_updaters_.size() &&
         imm_transition_probability_.rows() == static_cast<Eigen::Index>(imm_predictors_.size()) &&
         imm_initial_weights_.size() == static_cast<Eigen::Index>(imm_predictors_.size());
}

GaussianTrackState TrackLifecycleManager::BuildInitialGaussianState(
    const TrackMeasurement& measurement) const {
  GaussianTrackState init;
  init.mean(0) = measurement.raw_measurement.position(0);
  init.mean(1) = measurement.filtered_feature.velocity(0);
  init.mean(2) = measurement.raw_measurement.position(1);
  init.mean(3) = measurement.filtered_feature.velocity(1);
  init.mean(4) = measurement.raw_measurement.position(2);
  init.mean(5) = measurement.filtered_feature.velocity(2);
  init.covariance = StateCovariance::Identity() * 100.0f;
  return init;
}

bool TrackLifecycleManager::ShouldUseImmForMeasurement(bool track_existed_before_cycle,
                                                       TrackStatus status_before_update,
                                                       bool matched_existing_track) const {
  if (!IsImmEnabled()) {
    return false;
  }

  if (config_.imm_activation_policy == ImmActivationPolicy::kAllTracks) {
    return true;
  }

  return track_existed_before_cycle && status_before_update == TrackStatus::kConfirmed &&
         matched_existing_track;
}

bool TrackLifecycleManager::ShouldUseImmForMiss(TrackStatus status_before_prediction) const {
  if (!IsImmEnabled()) {
    return false;
  }

  if (config_.imm_activation_policy == ImmActivationPolicy::kAllTracks) {
    return true;
  }

  return status_before_prediction == TrackStatus::kConfirmed;
}
ImmFilter* TrackLifecycleManager::GetOrCreateImmFilter(std::uint64_t association_key,
                                                       const GaussianTrackState& initial_state) {
  std::unordered_map<std::uint64_t, std::unique_ptr<ImmFilter>>::iterator found =
      imm_filters_by_key_.find(association_key);
  if (found != imm_filters_by_key_.end()) {
    return found->second.get();
  }

  ImmConfig config;
  config.transition_probability = imm_transition_probability_;
  config.initial_weights = imm_initial_weights_;

  std::unique_ptr<ImmFilter> filter(new ImmFilter(config, imm_predictors_, imm_updaters_));
  std::vector<ImmModelState> initial_states;
  initial_states.reserve(imm_predictors_.size());
  for (std::size_t i = 0; i < imm_predictors_.size(); ++i) {
    initial_states.push_back(
        ImmModelState(initial_state, imm_initial_weights_(static_cast<Eigen::Index>(i))));
  }
  filter->SetModelStates(initial_states);
  ImmFilter* filter_ptr = filter.get();
  imm_filters_by_key_[association_key] = std::move(filter);
  return filter_ptr;
}
ImmFilter* TrackLifecycleManager::FindImmFilter(std::uint64_t association_key) const {
  std::unordered_map<std::uint64_t, std::unique_ptr<ImmFilter>>::const_iterator found =
      imm_filters_by_key_.find(association_key);
  if (found == imm_filters_by_key_.end()) {
    return nullptr;
  }
  return found->second.get();
}
void TrackLifecycleManager::ApplyGaussianState(TrackState& track, const GaussianTrackState& state,
                                               const Eigen::Vector3f& previous_velocity,
                                               float dt) const {
  track.gaussian_state = state;
  track.position(0) = state.mean(0);
  track.position(1) = state.mean(2);
  track.position(2) = state.mean(4);
  track.velocity(0) = state.mean(1);
  track.velocity(1) = state.mean(3);
  track.velocity(2) = state.mean(5);

  if (dt > 1e-6f) {
    track.acceleration = (track.velocity - previous_velocity) / dt;
  }
}

void TrackLifecycleManager::Update(const CycleContext& cycle,
                                   const std::vector<TrackMeasurement>& measurements) {
  float effective_dt_sec = 0.0f;
  if (!TryResolveEffectiveCycleDeltaTimeSec(cycle, &effective_dt_sec)) {
    // 中译：周期时间步长非法（{}），本周期更新被跳过且状态不变。
    // 标识：时间保护——无效 dt 时不推进航迹，防止数值污染。
    PROJECT_LOG_ERROR(
        "[TrackLifecycleManager] invalid cycle dt_sec={}, update skipped with no state changes.",
        cycle.dt_sec);
    return;
  }

  LifecycleUpdateScratch scratch;
  scratch.measurement_by_key.reserve(measurements.size());
  scratch.track_snapshots.reserve(tracks_by_key_.size());
  scratch.hit_keys.reserve(measurements.size());
  scratch.keys_to_recycle.reserve(tracks_by_key_.size());

  PreparePhase(scratch, measurements);

  scratch.work_items.reserve(tracks_by_key_.size() + scratch.measurement_by_key.size());
  scratch.results.reserve(tracks_by_key_.size() + scratch.measurement_by_key.size());

  EnsurePhase(scratch, measurements, cycle);
  ComputePhase(scratch, cycle, effective_dt_sec);
  CommitPhase(scratch, cycle.cycle_index);
  RecyclePhase(scratch);

  snapshot_emitter_.Refresh(tracks_by_key_);
}

void TrackLifecycleManager::PreparePhase(LifecycleUpdateScratch& scratch,
                                         const std::vector<TrackMeasurement>& measurements) const {
  for (std::unordered_map<std::uint64_t, TrackState*>::const_iterator it = tracks_by_key_.begin();
       it != tracks_by_key_.end(); ++it) {
    scratch.track_snapshots[it->first] = *it->second;
  }
  for (std::vector<TrackMeasurement>::const_iterator it = measurements.begin();
       it != measurements.end(); ++it) {
    const std::uint64_t association_key = it->raw_measurement.association_key;
    if (scratch.measurement_by_key.find(association_key) != scratch.measurement_by_key.end()) {
      // 中译：同一周期内关联键 {} 出现重复量测，暂存更新使用最后一条。
      // 标识：输入去重——同键多量测时取最后一条，防止一次更新写多次。
      PROJECT_LOG_WARN(
          "[TrackLifecycleManager] duplicate measurement for association_key={} in one cycle, use "
          "last measurement for staged update",
          association_key);
    }
    scratch.measurement_by_key[association_key] = &(*it);
    scratch.hit_keys.insert(association_key);
  }
}

void TrackLifecycleManager::EnsurePhase(LifecycleUpdateScratch& scratch,
                                        const std::vector<TrackMeasurement>& measurements,
                                        const CycleContext& cycle) {
  for (std::vector<TrackMeasurement>::const_iterator it = measurements.begin();
       it != measurements.end(); ++it) {
    const TrackMeasurement& measurement = *it;
    const std::uint64_t association_key = measurement.raw_measurement.association_key;
    if (scratch.measurement_by_key[association_key] != &measurement) {
      continue;
    }

    std::unordered_map<std::uint64_t, TrackState*>::iterator found =
        tracks_by_key_.find(association_key);
    const bool track_existed_before_cycle = found != tracks_by_key_.end();
    TrackState* track = nullptr;
    TrackState track_before_update;
    if (!track_existed_before_cycle) {
      track = pool_ != nullptr ? pool_->Acquire() : nullptr;
      if (track == nullptr) {
        // 中译：未能从对象池获取航迹对象（关联键 {}），本周期跳过该量测。
        // 标识：对象池耗尽——航迹数超过池容量时新航迹不建，
        //       该量测本周期不参与更新。
        PROJECT_LOG_WARN(
            "[TrackLifecycleManager] failed to acquire track from pool for association_key={}",
            association_key);
        continue;
      }
      ResetForReuse(*track);
      track->track_id = next_track_id_++;
      track->batch_id = cycle.batch_id;
      track->status = TrackStatus::kTentative;
      track->first_cycle = cycle.cycle_index;
      track->last_update_cycle = 0;
      track->hit_count = 0;
      track->miss_count = 0;
      tracks_by_key_[association_key] = track;
      track_before_update = *track;
      ++scratch.new_track_count;
    } else {
      track = found->second;
      track_before_update = scratch.track_snapshots[association_key];
      ++scratch.updated_track_count;
    }

    const TrackStatus status_before_update =
        track_existed_before_cycle ? track_before_update.status : TrackStatus::kTentative;
    const bool use_imm =
        ShouldUseImmForMeasurement(track_existed_before_cycle, status_before_update,
                                   measurement.raw_measurement.matched_existing_track);
    ImmFilter* imm_filter = nullptr;
    if (use_imm) {
      const bool reset_filter_on_hit = !measurement.raw_measurement.matched_existing_track ||
                                       status_before_update == TrackStatus::kLost;
      const GaussianTrackState initial_state = reset_filter_on_hit
                                                   ? BuildInitialGaussianState(measurement)
                                                   : track_before_update.gaussian_state;
      imm_filter = GetOrCreateImmFilter(association_key, initial_state);
    }

    TrackUpdateWorkItem work_item;
    work_item.association_key = association_key;
    work_item.kind = WorkItemKind::kHit;
    work_item.track = track;
    work_item.measurement = &measurement;
    work_item.track_existed_before_cycle = track_existed_before_cycle;
    work_item.use_imm = use_imm;
    work_item.imm_filter = imm_filter;
    work_item.status_before_update = status_before_update;
    work_item.track_before_update = track_before_update;
    scratch.work_items.push_back(work_item);
  }

  for (std::unordered_map<std::uint64_t, TrackState*>::const_iterator it = tracks_by_key_.begin();
       it != tracks_by_key_.end(); ++it) {
    if (scratch.hit_keys.find(it->first) != scratch.hit_keys.end()) {
      continue;
    }

    const TrackState& track_before_update = scratch.track_snapshots[it->first];
    const TrackStatus status_before_prediction = track_before_update.status;
    ImmFilter* imm_filter = nullptr;
    bool use_imm = false;
    if (ShouldUseImmForMiss(status_before_prediction)) {
      imm_filter = FindImmFilter(it->first);
      use_imm = imm_filter != nullptr;
    }

    TrackUpdateWorkItem work_item;
    work_item.association_key = it->first;
    work_item.kind = WorkItemKind::kMiss;
    work_item.track = it->second;
    work_item.track_existed_before_cycle = true;
    work_item.use_imm = use_imm;
    work_item.imm_filter = imm_filter;
    work_item.status_before_update = status_before_prediction;
    work_item.track_before_update = track_before_update;
    scratch.work_items.push_back(work_item);
    ++scratch.predicted_without_hit_count;
  }
}

void TrackLifecycleManager::ComputePhase(LifecycleUpdateScratch& scratch, const CycleContext& cycle,
                                         float effective_dt_sec) const {
  for (std::vector<TrackUpdateWorkItem>::const_iterator it = scratch.work_items.begin();
       it != scratch.work_items.end(); ++it) {
    const TrackUpdateWorkItem& work_item = *it;
    TrackUpdateResult result;
    result.association_key = work_item.association_key;
    result.track = work_item.track;
    result.track_after_update = work_item.track_before_update;

    TrackState& track = result.track_after_update;
    if (work_item.kind == WorkItemKind::kHit) {
      const TrackMeasurement& measurement = *work_item.measurement;
      track.last_update_cycle = cycle.cycle_index;
      track.hit_count += 1;
      track.miss_count = 0;

      track.position = measurement.raw_measurement.position;
      if (measurement.raw_measurement.external_target_id != 0U) {
        track.external_target_id = measurement.raw_measurement.external_target_id;
      }
      if (!measurement.raw_measurement.target_name.empty()) {
        track.target_name = measurement.raw_measurement.target_name;
      }
      // track.velocity 仍持有上一周期速度（track_before_update 来自持久化 tracks_by_key_），
      // 在被本周期测量覆盖前保存，供反 VGPO 加速度限幅裁剪。
      const Eigen::Vector3f velocity_before_update = track.velocity;
      if (measurement.filtered_feature.velocity.squaredNorm() > 0.0f) {
        track.velocity = measurement.filtered_feature.velocity;
      } else {
        track.velocity = Eigen::Vector3f(measurement.filtered_feature.observed_speed, 0.0f, 0.0f);
      }
      track.rcs = measurement.filtered_feature.rcs;

      PromoteState(track, cycle.cycle_index, true, cycle.extra_miss_tolerance,
                   measurement.raw_measurement.classified_as_false_target);

      ApplyKalmanHitUpdate(work_item, measurement, track, effective_dt_sec);

      // 反 VGPO 加速度限幅：裁剪超出物理上限的航迹速度变化，抑制 VGPO 制造的速度拖引。
      // 必须在 Kalman/IMM 更新之后执行，否则后验会覆盖限幅结果。仅对预先存在的航迹限幅——
      // 新生航迹的 velocity_before_update 为初始零值，不是真实上一周期速度，限幅无意义。
      if (config_.enable_anti_vgpo_acceleration_bound && work_item.track_existed_before_cycle &&
          effective_dt_sec > 0.0f) {
        const float max_delta =
            static_cast<float>(config_.max_acceleration_mps2 * effective_dt_sec);
        bool clamped_any = false;
        for (int axis = 0; axis < 3; ++axis) {
          const float delta = track.velocity[axis] - velocity_before_update[axis];
          if (delta > max_delta) {
            track.velocity[axis] = velocity_before_update[axis] + max_delta;
            clamped_any = true;
          } else if (delta < -max_delta) {
            track.velocity[axis] = velocity_before_update[axis] - max_delta;
            clamped_any = true;
          }
        }
        // 限幅回写全套状态：下一周期 Predict 读 track.gaussian_state（非 track.velocity），
        // 关联门控同样读 gaussian_state（DataAssociation::Predict）。把限幅后速度写回
        // gaussian_state.mean 的速度分量（[1,3,5]），并按同口径重算 acceleration。
        if (clamped_any) {
          track.gaussian_state.mean(1) = track.velocity(0);
          track.gaussian_state.mean(3) = track.velocity(1);
          track.gaussian_state.mean(5) = track.velocity(2);
          track.acceleration = (track.velocity - velocity_before_update) / effective_dt_sec;
          // IMM 路径：track.gaussian_state 只是镜像，IMM 下一周期从各 model_states_ 重新混合。
          // 同步把限幅速度写回每个模型的 mean 速度分量（权重不变），保证 re-mix 从限幅状态
          // 出发。combined_state_ 会暂陈旧，但下个 Process 的 CombineEstimates 会重算。
          if (work_item.use_imm && work_item.imm_filter != nullptr) {
            std::vector<ImmModelState> model_states = work_item.imm_filter->GetModelStates();
            for (ImmModelState& model_state : model_states) {
              model_state.state.mean(1) = track.velocity(0);
              model_state.state.mean(3) = track.velocity(1);
              model_state.state.mean(5) = track.velocity(2);
            }
            work_item.imm_filter->SetModelStates(model_states);
          }
        }
      }
    } else {
      PromoteState(track, cycle.cycle_index, false, cycle.extra_miss_tolerance, false);

      if (track.status == TrackStatus::kRecycled) {
        result.should_recycle = true;
      } else {
        ApplyKalmanMissPredict(work_item, track, effective_dt_sec);
      }
    }

    scratch.results.push_back(result);
  }
}

void TrackLifecycleManager::CommitPhase(LifecycleUpdateScratch& scratch,
                                        std::uint32_t cycle_index) {
  for (std::vector<TrackUpdateResult>::const_iterator it = scratch.results.begin();
       it != scratch.results.end(); ++it) {
    if (it->track == nullptr) {
      continue;
    }
    *(it->track) = it->track_after_update;
    if (it->should_recycle) {
      scratch.keys_to_recycle.push_back(it->association_key);
    }
  }
  last_cycle_index_ = cycle_index;
}

void TrackLifecycleManager::RecyclePhase(LifecycleUpdateScratch& scratch) {
  for (std::vector<std::uint64_t>::const_iterator it = scratch.keys_to_recycle.begin();
       it != scratch.keys_to_recycle.end(); ++it) {
    imm_filters_by_key_.erase(*it);

    std::unordered_map<std::uint64_t, TrackState*>::iterator found = tracks_by_key_.find(*it);
    if (found == tracks_by_key_.end()) {
      continue;
    }
    ResetForReuse(*found->second);
    if (pool_ != nullptr) {
      pool_->Release(found->second);
    }
    tracks_by_key_.erase(found);
  }
}

bool TrackLifecycleManager::TryResolveEffectiveCycleDeltaTimeSec(const CycleContext& cycle,
                                                                 float* effective_dt_sec) const {
  if (effective_dt_sec == nullptr) {
    return false;
  }
  if (std::isfinite(cycle.dt_sec) == 0 || cycle.dt_sec <= 0.0f) {
    return false;
  }
  *effective_dt_sec = cycle.dt_sec;
  return true;
}

void TrackLifecycleManager::ApplyKalmanHitUpdate(const TrackUpdateWorkItem& work_item,
                                                 const TrackMeasurement& measurement,
                                                 TrackState& track, float effective_dt_sec) const {
  const Eigen::Vector3f velocity_before_filter = track.velocity;
  const bool reset_filter_on_hit = !measurement.raw_measurement.matched_existing_track ||
                                   work_item.status_before_update == TrackStatus::kLost;
  if (work_item.use_imm && work_item.imm_filter != nullptr) {
    if (reset_filter_on_hit) {
      ApplyGaussianState(track, BuildInitialGaussianState(measurement), velocity_before_filter,
                         effective_dt_sec);
    } else {
      // IMM 命中更新与 CV KF 路径/关联门控同口径：消费逐量测动态 R。
      // 零矩阵/非有限视为"未注入动态 R"，回退更新器配置的标量 R（缺省量测噪声）。
      const Eigen::Matrix3f& dynamic_r = measurement.raw_measurement.measurement_covariance;
      if (dynamic_r.allFinite() && !dynamic_r.isZero(0.0f)) {
        work_item.imm_filter->Process(BuildMeasurementVector(measurement), effective_dt_sec,
                                      dynamic_r);
      } else {
        work_item.imm_filter->Process(BuildMeasurementVector(measurement), effective_dt_sec);
      }
      ApplyGaussianState(track, work_item.imm_filter->GetCombinedState(), velocity_before_filter,
                         effective_dt_sec);
    }
  } else if (kalman_predictor_ != nullptr && kalman_updater_ != nullptr) {
    if (reset_filter_on_hit) {
      ApplyGaussianState(track, BuildInitialGaussianState(measurement), velocity_before_filter,
                         effective_dt_sec);
    } else {
      const GaussianTrackState predicted =
          kalman_predictor_->Predict(track.gaussian_state, effective_dt_sec);
      const KalmanUpdateResult update_result =
          kalman_updater_->Update(predicted, BuildMeasurementVector(measurement),
                                  measurement.raw_measurement.measurement_covariance);
      ApplyGaussianState(track, update_result.posterior, velocity_before_filter, effective_dt_sec);
    }
  }
}

void TrackLifecycleManager::ApplyKalmanMissPredict(const TrackUpdateWorkItem& work_item,
                                                   TrackState& track,
                                                   float effective_dt_sec) const {
  const Eigen::Vector3f velocity_before_filter = track.velocity;
  if (work_item.use_imm && work_item.imm_filter != nullptr) {
    work_item.imm_filter->Predict(effective_dt_sec);
    ApplyGaussianState(track, work_item.imm_filter->GetCombinedState(), velocity_before_filter,
                       effective_dt_sec);
  } else if (kalman_predictor_ != nullptr) {
    ApplyGaussianState(track, kalman_predictor_->Predict(track.gaussian_state, effective_dt_sec),
                       velocity_before_filter, effective_dt_sec);
  }
}

void TrackLifecycleManager::PromoteState(TrackState& track, std::uint32_t cycle_index,
                                         bool hit_this_cycle,
                                         std::uint32_t extra_miss_tolerance,
                                         bool classified_as_false_target) const {
  oneq::common::tracking::TrackLifecycleCounters counters;
  counters.status = static_cast<oneq::common::tracking::TrackLifecyclePhase>(
      static_cast<std::uint8_t>(track.status));
  counters.hit_count = track.hit_count;
  counters.miss_count = track.miss_count;
  counters.last_update_cycle = track.last_update_cycle;
  counters.generation = track.generation;

  oneq::common::tracking::TrackLifecyclePromotePolicy policy;
  policy.confirm_hits = config_.confirm_hits;
  policy.max_miss_before_lost = config_.max_miss_before_lost;
  policy.max_lost_cycles = config_.max_lost_cycles;
  policy.extra_miss_tolerance = extra_miss_tolerance;
  // 假目标鉴别：启用时抑制 tentative→confirmed（模块侧钩子）。
  policy.suppress_confirm = config_.enable_anti_false_target_discrimination &&
                            classified_as_false_target &&
                            track.status == TrackStatus::kTentative;
  policy.recycle_as_status = true;

  oneq::common::tracking::PromoteTrackLifecycle(&counters, cycle_index, hit_this_cycle, policy);

  track.status = static_cast<TrackStatus>(static_cast<std::uint8_t>(counters.status));
  track.hit_count = counters.hit_count;
  track.miss_count = counters.miss_count;
  track.generation = counters.generation;
}

void TrackLifecycleManager::ResetForReuse(TrackState& track) const {
  track.track_id = 0;
  track.batch_id = 0;
  track.external_target_id = 0;
  track.target_name.clear();
  track.status = TrackStatus::kTentative;
  track.first_cycle = 0;
  track.last_update_cycle = 0;
  track.miss_count = 0;
  track.hit_count = 0;
  track.position.setZero();
  track.velocity.setZero();
  track.acceleration.setZero();
  track.rcs = 0.0f;
  track.gaussian_state = GaussianTrackState();
}

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
