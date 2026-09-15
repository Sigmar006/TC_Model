/**
 * @file RecognitionTracker.h
 * @brief 单航迹识别积累、判定与结论保持（库内部）。
 *
 * 每个活跃 association_key 对应一个识别积累状态：保存滑动窗口
 * 特征样本、确认命中数、最近结论与结论时间。状态随航迹创建，随航迹
 * 回收时清理。关联键由自持跟踪单调分配，键重分配天然等于新目标。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_TRACKER_H_
#define REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_TRACKER_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "1q/remote_identification_radar/config/RirPolicyConfig.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "1q/remote_identification_radar/session/RirSceneTypes.h"
#include "remote_identification_radar/recognition/RecognitionFeatureDatabase.h"
#include "remote_identification_radar/recognition/RecognitionTypes.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace recognition {

/**
 * @brief RirTrackState 单航迹跨周期识别积累状态。
 */
struct RirTrackState {
  std::uint64_t association_key{0U};
  std::uint32_t last_seen_hit_count{0U}; /**< 上周期快照命中计数；下降视为键重分配。 */

  /** 积累 */
  std::vector<RirFeatureSet> window{}; /**< 时间序滑动窗口（按 accumulation_window_sec）。 */
  std::vector<float> window_timestamps_sec{};
  std::uint32_t confirmed_hit_count{0U}; /**< 已确认航迹上的有效观测数。 */
  std::uint32_t observation_count{0U};   /**< 有效观测总数。 */
  float first_observation_sec{0.0f};     /**< 首次有效观测时刻（sim 秒）。 */

  /** 结论 */
  session::RirRecognitionResult result{};
  float conclusion_time_sec{-1.0f};       /**< 最近一次结论时刻；-1 表示尚无结论。 */
  float first_conclusion_time_sec{-1.0f}; /**< 首次确认（大类/型号）时刻；-1 表示尚未确认。 */
  bool has_conclusion{false};
};

/**
 * @brief RirTracker 识别积累与判定状态机。
 *
 * 模式语义：
 * - kLrr 已执行周期 → UpdateCycle：积累 + 判定；
 * - 非 kLrr 已执行周期 → HoldCycle：不积累，结论按 result_hold_sec 过期为 kStale；
 * - 退出 kLrr（ExitRecognitionMode）→ 清空积累（保留结论，进入保持期）；
 * - 航迹键重分配（内部关联键单调不回收，不回落）→ 新键天然新目标；
 * - 非执行周期不触碰任何状态（校验拒绝/关机由调用方保证不调用）。
 */
class RirTracker {
 public:
  /** @brief 判定与积累选项（源自 RirRecognitionPolicy）。 */
  struct Options {
    std::uint32_t min_confirmed_hits{5U};
    float accumulation_window_sec{10.0f};
    std::uint32_t min_observation_count{3U};
    float acceptance_score{0.70f};
    float minimum_margin{0.10f};
    float result_hold_sec{30.0f};
    float max_range_m{300000.0f};
  };

  /** @brief 单航迹周期观测输入（场景目标特征真值 + 效能上下文）。 */
  struct TrackObservationInput {
    const session::RirSceneTarget* target{nullptr};
    RirObservationContext context{};
  };

  /**
   * @brief CycleFeatureObservation 本周期实际构建的特征观测（出口①透出来源）。
   * @note 填充口径 = 识别链实际构建且至少一维有效（Build 后 mask != 0）的观测；
   *       积累质量门只挡积累不挡透出；超距/无观测输入的键不产生（透出原则，
   *       冻结契约裁定：识别链未构建观测的周期出口①为空，不虚构）。
   */
  struct CycleFeatureObservation {
    std::uint64_t association_key{0U};
    RirFeatureSet features{};
  };

  /** @brief 整快照捕获/恢复（回滚边界）。 */
  struct Snapshot {
    std::vector<RirTrackState> tracks{};
    std::string active_database_version{};
  };

  void SetOptions(const Options& options) { options_ = options; }
  const Options& options() const { return options_; }

  /** @brief 设置当前生效数据库版本（供 replay 溯源）。 */
  void SetActiveDatabaseVersion(std::string version) {
    active_database_version_ = std::move(version);
  }
  const std::string& ActiveDatabaseVersion() const { return active_database_version_; }

  /** @brief 退出识别模式：清空全部积累，保留结论进入保持期。 */
  void ExitRecognitionMode();

  /**
   * @brief kLrr 已执行周期更新。
   * @param[in] tracks 本周期决策帧轨迹快照。
   * @param[in] observations_by_key 按 association_key 的观测输入。
   * @param[in] database 当前生效特征数据库。
   * @param[in] weights 特征基础权重。
   * @param[in] sim_time_sec 当前仿真时刻（s）。
   * @param[in] cycle_index 当前周期号。
   * @param[in] batch_id 当前批号。
   * @param[out] cycle_observations 可选：本周期实际构建的有效特征观测
   *            （出口①透出来源，按航迹键；nullptr = 不采集）。
   */
  void UpdateCycle(
      const std::vector<tracking::RirTrackState>& tracks,
      const std::unordered_map<std::uint64_t, TrackObservationInput>& observations_by_key,
      const RirFeatureDatabase& database, const config::RirRecognitionFeatureWeights& weights,
      float sim_time_sec, std::uint32_t cycle_index, std::uint64_t batch_id,
      std::vector<CycleFeatureObservation>* cycle_observations = nullptr);

  /**
   * @brief 非 kLrr 已执行周期：结论按 result_hold_sec 过期为 kStale，不积累。
   * @param[in] tracks 本周期决策帧轨迹快照（仅用于键重分配检测）。
   * @param[in] sim_time_sec 当前仿真时刻（s）。
   */
  void HoldCycle(const std::vector<tracking::RirTrackState>& tracks, float sim_time_sec);

  /** @brief 查询航迹识别结果；无状态时返回 nullptr。 */
  const session::RirRecognitionResult* FindResult(std::uint64_t association_key) const;

  /** @brief 构建本周期识别效能摘要。 */
  session::RirRecognitionCycleSummary BuildSummary(
      const std::vector<tracking::RirTrackState>& tracks) const;

  Snapshot Capture() const;
  void Restore(const Snapshot& snapshot);
  void Reset();

 private:
  /** @brief 结论过期判定：无新观测周期内超过 result_hold_sec 后置 kStale。 */
  void ExpireIfHeld(RirTrackState* state, float sim_time_sec);

  Options options_{};
  std::string active_database_version_{};
  /** @brief model_id → category_id（DB 加载时捕获，供真值准确率统计）。 */
  std::unordered_map<std::string, std::string> model_categories_{};
  std::unordered_map<std::uint64_t, RirTrackState> tracks_{};
};

}  // namespace recognition
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_TRACKER_H_
