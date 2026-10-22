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

/**
 * @brief 速度拟合窗口长度（拍）与最少样本数（B3，2026-10-21）。
 *
 * @details 单拍位置噪声 σ_p≈2.7–5.5 km，两拍差分噪声达 √2·σ_p/Δt≈7 km/s，不可用；
 *   对 N 拍直线拟合后 σ_v≈σ_p·√(12/(N(N²−1)))/Δt。取 N=20（Δt=1 s）时约 200 m/s，
 *   足以分辨弹道族模板（σ=500–800 m/s）；更长的窗口会把目标机动抹平。
 */
constexpr std::size_t kVelocityFitWindow = 20U;
constexpr std::size_t kVelocityFitMinSamples = 5U;
/**
 * @brief 加速度斜率拟合窗口与最少样本（2026-10-22）。
 *
 * @details 对"逐拍拟合速度"做直线拟合取斜率作为加速度。σ_a ≈ √12·σ_v/(N^1.5·Δt)：
 *   σ_v≈200 m/s（B3 拟合误差约 6%）、Δt=1 s 时，N=40 约 2.7 m/s²、N=20 约 7.7 m/s²；
 *   取 40 拍兼顾噪声（目标加速度约 5 m/s²）与机动时间尺度。
 */
constexpr std::size_t kVelocityTrendWindow = 40U;
constexpr std::size_t kVelocityTrendMinSamples = 10U;

/** @brief 加速度估计的最小有效步长（s）。 */
constexpr float kMinimumAccelerationDtSec = 1.0e-6f;

/**
 * @brief 加速度一阶指数平滑的时间常数（周期数，2026-10-16）。
 *
 * 为什么必须平滑：加速度取**单周期**后验速度差分 `a = Δv/dt`，而真值速度只能由量测估计，
 * 其单拍噪声 `σ_v ≈ σ_p/dt` 会被 `1/dt` 直接放大成加速度噪声（dt=1 s、σ_p≈10 m 时
 * σ_a ≈ 10 m/s² 量级）。实测弹道目标真值加速度仅 4.4~11 m/s²，而报出值为 0~40、
 * 均值 31.9 m/s² —— 该维在识别打分里因此变成噪声放大器：正确型号
 * `BALLISTIC_EXAMPLE_B`（加速度模板 7.5±3.0）相似度≈0，而巡航导弹模板（20±8）反而满分。
 *
 * 取 30 个周期：平滑后加速度噪声 σ 降到约 `σ_a/√(τ/2) ≈ 3.6 m/s²`，与模板 7.5±3.0 同量级；
 * 引入的滞后 ≈ τ 个周期（1 Hz 下 30 s），而弹道段加速度变化率约 0.05 m/s²/周期（约
 * 1.5 m/s²/30 周期），滞后代价小于降噪收益。
 *
 * 该常数只影响识别/验收消费的加速度，不改变滤波状态本身（KF 不消费加速度）。
 */
constexpr float kAccelerationSmoothingCycles = 30.0f;

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
      // 中译：记录本拍量测位置，供识别侧"多点最小二乘速度"使用（B3，2026-10-21）。
      //       只保留最近 kVelocityFitWindow 拍：更早的样本对速度拟合贡献极小，且会
      //       把目标机动抹平。时间基准用 周期序号 × dt，同一航迹内自洽。
      // 标识：B3 速度拟合历史。
      {
        std::vector<PositionSample>& history =
            velocity_history_by_key_[work_item.association_key];
        PositionSample sample;
        sample.time_sec = static_cast<float>(cycle.cycle_index) * cycle.dt_sec;
        sample.position = measurement.position;
        history.push_back(sample);
        if (history.size() > kVelocityFitWindow) {
          history.erase(history.begin());
        }
      }

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

bool RirTrackLifecycle::DropTrack(std::uint64_t association_key) {
  const auto track_it = tracks_.find(association_key);
  if (track_it == tracks_.end()) {
    return false;
  }
  // 中译：连同 IMM 运行态一起清除——否则下一拍重建航迹时会继承发散的多模型状态。
  // 标识：B6 包络门丢弃路径（2026-10-20）。
  imm_filters_by_key_.erase(association_key);
  // 中译：航迹被丢弃时同步清理速度拟合历史，避免同键新航迹继承旧目标的位置序列。
  // 标识：B3 速度拟合历史清理。
  velocity_history_by_key_.erase(association_key);
  velocity_trend_by_key_.erase(association_key);
  RecycleTrack(track_it->second);
  tracks_.erase(track_it);
  return true;
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

bool RirTrackLifecycle::TryGetLeastSquaresVelocity(std::uint64_t association_key,
                                                   Eigen::Vector3f* velocity_mps) const {
  if (velocity_mps == nullptr) {
    return false;
  }
  const std::unordered_map<std::uint64_t, std::vector<PositionSample>>::const_iterator found =
      velocity_history_by_key_.find(association_key);
  if (found == velocity_history_by_key_.end()) {
    return false;
  }
  const std::vector<PositionSample>& history = found->second;
  if (history.size() < kVelocityFitMinSamples) {
    return false;
  }
  // 中译：对最近 N 拍位置做"时间—位置"直线拟合，斜率即速度。三轴各自独立拟合；
  //       时间取相对首个样本的偏移，避免大数相减丢精度。
  // 标识：B3 速度拟合（最小二乘）。
  double time_sum = 0.0;
  double time_sq_sum = 0.0;
  const double t0 = static_cast<double>(history.front().time_sec);
  for (std::size_t i = 0U; i < history.size(); ++i) {
    const double dt = static_cast<double>(history[i].time_sec) - t0;
    time_sum += dt;
    time_sq_sum += dt * dt;
  }
  const double count = static_cast<double>(history.size());
  const double time_mean = time_sum / count;
  const double denom = time_sq_sum - count * time_mean * time_mean;
  if (!(denom > 0.0)) {
    return false;
  }
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    double position_sum = 0.0;
    for (std::size_t i = 0U; i < history.size(); ++i) {
      position_sum += static_cast<double>(history[i].position[axis]);
    }
    const double position_mean = position_sum / count;
    double covariance = 0.0;
    for (std::size_t i = 0U; i < history.size(); ++i) {
      const double dt = static_cast<double>(history[i].time_sec) - t0 - time_mean;
      covariance += dt * (static_cast<double>(history[i].position[axis]) - position_mean);
    }
    velocity[axis] = static_cast<float>(covariance / denom);
  }
  if (!velocity.allFinite()) {
    return false;
  }
  *velocity_mps = velocity;
  return true;
}

void RirTrackLifecycle::RecordLeastSquaresVelocity(std::uint64_t association_key,
                                                  float time_sec,
                                                  const Eigen::Vector3f& velocity_mps) {
  std::vector<VelocitySample>& trend = velocity_trend_by_key_[association_key];
  VelocitySample sample;
  sample.time_sec = time_sec;
  sample.velocity = velocity_mps;
  trend.push_back(sample);
  if (trend.size() > kVelocityTrendWindow) {
    trend.erase(trend.begin());
  }
}

bool RirTrackLifecycle::TryGetLeastSquaresAcceleration(std::uint64_t association_key,
                                                       Eigen::Vector3f* acceleration_mps2) const {
  if (acceleration_mps2 == nullptr) {
    return false;
  }
  const std::unordered_map<std::uint64_t, std::vector<VelocitySample>>::const_iterator found =
      velocity_trend_by_key_.find(association_key);
  if (found == velocity_trend_by_key_.end()) {
    return false;
  }
  const std::vector<VelocitySample>& trend = found->second;
  if (trend.size() < kVelocityTrendMinSamples) {
    return false;
  }
  // 中译：对"时间—拟合速度"做直线拟合，斜率即加速度；时间取相对首样本的偏移。
  // 标识：加速度斜率拟合（2026-10-22）。
  const double t0 = static_cast<double>(trend.front().time_sec);
  double time_sum = 0.0;
  double time_sq_sum = 0.0;
  for (std::size_t i = 0U; i < trend.size(); ++i) {
    const double dt = static_cast<double>(trend[i].time_sec) - t0;
    time_sum += dt;
    time_sq_sum += dt * dt;
  }
  const double count = static_cast<double>(trend.size());
  const double time_mean = time_sum / count;
  const double denom = time_sq_sum - count * time_mean * time_mean;
  if (!(denom > 0.0)) {
    return false;
  }
  Eigen::Vector3f acceleration = Eigen::Vector3f::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    double value_sum = 0.0;
    for (std::size_t i = 0U; i < trend.size(); ++i) {
      value_sum += static_cast<double>(trend[i].velocity[axis]);
    }
    const double value_mean = value_sum / count;
    double covariance = 0.0;
    for (std::size_t i = 0U; i < trend.size(); ++i) {
      const double dt = static_cast<double>(trend[i].time_sec) - t0 - time_mean;
      covariance += dt * (static_cast<double>(trend[i].velocity[axis]) - value_mean);
    }
    acceleration[axis] = static_cast<float>(covariance / denom);
  }
  if (!acceleration.allFinite()) {
    return false;
  }
  *acceleration_mps2 = acceleration;
  return true;
}

void RirTrackLifecycle::Reset() {
  for (auto& entry : tracks_) {
    ResetForReuse(*entry.second);
    pool_.Release(entry.second);
  }
  tracks_.clear();
  imm_filters_by_key_.clear();
  velocity_history_by_key_.clear();
  velocity_trend_by_key_.clear();
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
    // 瞬时差分（物理口径）→ 一阶指数平滑后再落盘：单周期差分的噪声按 1/dt 放大，
    // 直接用会把识别"运动"维变成噪声放大器（见 kAccelerationSmoothingCycles 注释）。
    // 首个样本直接起批（无历史可比，平滑无意义）；此后按时间常数递推。
    const Eigen::Vector3f instantaneous = (track->velocity - previous_velocity) / dt_sec;
    if (!track->acceleration_smoothed) {
      track->acceleration = instantaneous;
      track->acceleration_smoothed = true;
    } else {
      const float alpha = 1.0f / kAccelerationSmoothingCycles;
      track->acceleration = (1.0f - alpha) * track->acceleration + alpha * instantaneous;
    }
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
  // 中译：**速度两点初始化**（2026-10-21，B）——量测不携带真值速度（去真值化契约），
  //       但航迹此前已有命中时，可用"(本次量测位置 − 上次位置) / 间隔"给出与测量自洽的
  //       初速，避免零速先验在长斜距、低角度精度下收敛过慢（实测滤波速度约 1.6 km/s，
  //       而真值约 6 km/s，弹道族模型因此分不开）。
  // 标识：B 速度两点初始化（仅用测量，不含真值）。
  Eigen::Vector3f two_point_velocity = Eigen::Vector3f::Zero();
  // 仅用于**新建航迹的第二次命中**：此时两次量测间隔短、位置差可信；失跟重捕
  // （kLost）不适用——重捕间隔长、目标可能已机动，仍按零速无知先验重置（既有契约
  // RirTrackLifecycleTest.LostTrackRehitResetsFilterAndConfirms）。
  const bool has_two_point_velocity =
      reset_filter_on_hit && measurement.matched_existing_track &&
      status_before != RirTrackStatus::kLost && track->hit_count == 2U && dt_sec > 0.0f;
  if (has_two_point_velocity) {
    two_point_velocity = (measurement.position - track->position) / dt_sec;
  }
  const RirGaussianState reset_state =
      has_two_point_velocity ? filter_.Initialize(measurement.position, two_point_velocity)
                             : filter_.Initialize(measurement);
  if (imm_filter != nullptr && imm_filter->IsValid()) {
    if (reset_filter_on_hit) {
      // 防御分支（confirmed-only 激活策略下不可达）：IMM 命中重置退化为 CV 初始化，
      // 与 AR ApplyKalmanHitUpdate 同构。
      ApplyGaussianState(track, reset_state, velocity_before_update,
                         acceleration_dt_sec);
      // 本次是重初始化：加速度清零并**复位起批标志**，使下一次命中重新以瞬时差分起批
      // （否则新基准下的差分会被 EMA 压成 1/τ，重捕后长期低报）。
      track->acceleration.setZero();
      track->acceleration_mps2 = 0.0f;
      track->acceleration_smoothed = false;
      return;
    }
    imm_filter->Process(measurement.position, dt_sec, measurement.measurement_covariance);
    ApplyGaussianState(track, imm_filter->GetCombinedState(), velocity_before_update,
                       acceleration_dt_sec);
    return;
  }
  if (reset_filter_on_hit) {
    ApplyGaussianState(track, reset_state, velocity_before_update,
                       acceleration_dt_sec);
    track->acceleration.setZero();
    track->acceleration_mps2 = 0.0f;
    track->acceleration_smoothed = false;
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
  // 平滑起批标志随槽位复用清零：否则新航迹首个差分会被当成"已平滑"样本，
  // 以零为历史做 EMA，首拍加速度被人为压低（池化复用的隐蔽串味）。
  track.acceleration_smoothed = false;
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
