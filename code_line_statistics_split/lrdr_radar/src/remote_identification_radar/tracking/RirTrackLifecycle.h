/**
 * @file RirTrackLifecycle.h
 * @brief RIR 轻量跟踪子集的航迹生命周期管理器（阶段 2-T T3，N3 池化、
 *        N5 IMM 双路径）。
 *
 * 副本来源：`src/airborne_radar/signal/tracking/TrackLifecycleManager.*` 子集
 * （审计基线 96de367c）。
 * 刻意不迁：假目标鉴别、反 VGPO 加速度限幅、快照事件发射器、
 * IMM 全航迹激活策略（kAllTracks；RIR 仅 confirmed 命中激活）。
 * 内部航迹表为有序键→池化对象指针映射：航迹对象经 `RirTrackPool` 申请/归还，
 * 回收即出表并归还池（无 `kRecycled` 中间态），槽位复用经 `generation`
 * 单调递增标识；快照仍按关联键升序导出（确定性，便于 replay）。
 * IMM 路径：confirmed 且命中既有航迹时启用（`enable_imm_lifecycle`），
 * 失配周期对已有 IMM 态的 confirmed 航迹执行仅预测；lost 重捕获仍走
 * CV KF 重置（与 AR confirmed-only 激活策略一致）。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_LIFECYCLE_H_
#define REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_LIFECYCLE_H_

#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include "remote_identification_radar/tracking/RirImmFilter.h"
#include "remote_identification_radar/tracking/RirTrackFilter.h"
#include "remote_identification_radar/tracking/RirTrackPool.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace tracking {

/**
 * @brief RirLifecycleConfig 航迹状态机阈值配置。
 */
struct RirLifecycleConfig {
  std::uint32_t confirm_hits{3U};         /**< tentative 转 confirmed 所需累计命中数。 */
  std::uint32_t max_miss_before_lost{2U}; /**< tentative/confirmed 转 lost 的连续失配阈值。 */
  std::uint32_t max_lost_cycles{5U};      /**< lost 保留周期数，超出即回收。 */
  bool enable_imm_lifecycle{true};        /**< 是否启用 IMM 生命周期路径（confirmed 命中激活）。 */
  std::uint32_t model_count_hint{2U};     /**< IMM 模型数提示值（对数等距 CV 模型集）。 */
};

/**
 * @brief RirCycleContext 生命周期更新周期上下文。
 */
struct RirCycleContext {
  std::uint32_t cycle_index{0U}; /**< 当前周期号。 */
  std::uint64_t batch_id{0U};    /**< 当前批号。 */
  float dt_sec{0.1f};            /**< 周期步长（s）；非正/非有限时整周期跳过。 */
};

/**
 * @brief RirLifecycleRuntimeState 生命周期运行态快照。
 */
struct RirLifecycleRuntimeState {
  std::vector<RirTrackState> tracks;  /**< 当前航迹（按关联键升序）。 */
  std::uint64_t next_track_id{1U};    /**< 下一内部航迹编号。 */
  std::uint32_t last_cycle_index{0U}; /**< 最近成功更新周期号。 */
};

/**
 * @brief RirTrackLifecycle 轻量航迹生命周期管理器。
 *
 * 状态机语义（与 AR 子集一致）：
 * - hit：`hit_count += 1`、`miss_count = 0`；tentative 达到 `confirm_hits`
 *   后转 confirmed；lost 再次命中立即恢复 confirmed；
 * - miss：tentative/confirmed 连续失配超过 `max_miss_before_lost` 转 lost；
 *   lost 保持 `max_lost_cycles` 个周期后回收（出表、`generation += 1`、
 *   归还对象池）；
 * - 航迹回收不回收关联键：`RirTrackAssociator` 继续单调分配新键，
 *   因此键重分配在识别积累侧天然是新目标，无需 `hit_count` 回落检测。
 * - 对象池复用代次：新航迹复用已归还槽位时保留槽位 `generation`
 *   （业务字段由 ResetForReuse 清零），供外部识别已回收对象的旧引用。
 *
 * @note 持有池化裸指针，不可拷贝；运行态经 Capture/RestoreRuntimeState
 *       以值快照迁移（replay 兼容）。
 */
class RirTrackLifecycle {
 public:
  /** @brief 构造生命周期管理器。 */
  explicit RirTrackLifecycle(RirLifecycleConfig config = {},
                             RirTrackFilterConfig filter_config = {});

  RirTrackLifecycle(const RirTrackLifecycle&) = delete;
  RirTrackLifecycle& operator=(const RirTrackLifecycle&) = delete;

  /**
   * @brief 以本周期关联量测推进航迹状态机与 KF。
   * @param[in] cycle 周期上下文；dt 非法时整周期跳过且状态不变。
   * @param[in] measurements 关联后量测（association_key 不得为 0）。
   */
  void Update(const RirCycleContext& cycle, const std::vector<RirTrackMeasurement>& measurements);

  /** @brief 导出内部航迹快照（tentative/confirmed/lost，按关联键升序）。 */
  RirTrackSnapshotList BuildTrackSnapshots() const;

  /** @brief 导出关联阶段消费的航迹种子（所有未回收航迹）。 */
  std::vector<RirTrackSeed> BuildAssociationSeeds() const;

  /** @brief 查询航迹；不存在返回 nullptr。 */
  const RirTrackState* FindTrack(std::uint64_t association_key) const;

  /** @brief 当前未回收航迹数量。 */
  std::size_t ActiveTrackCount() const { return tracks_.size(); }

  /** @brief IMM 是否启用（enable_imm_lifecycle 且模型集可构建）。 */
  bool IsImmEnabled() const { return imm_enabled_; }

  /** @brief 查关联键的 IMM 运行态（诊断/测试消费）；不存在或未启用返回 nullptr。 */
  RirImmFilter* FindImmFilter(std::uint64_t association_key) const;

  /** @brief 清空全部航迹并重置内部航迹编号。 */
  void Reset();

  /** @brief 全量更新生命周期与滤波器配置（保留既有航迹状态）。 */
  void UpdateConfig(RirLifecycleConfig lifecycle_config, RirTrackFilterConfig filter_config);

  /** @brief 捕获运行态（replay/回滚边界）。 */
  RirLifecycleRuntimeState CaptureRuntimeState() const;

  /** @brief 恢复运行态。 */
  void RestoreRuntimeState(const RirLifecycleRuntimeState& state);

 private:
  enum class WorkItemKind { kHit = 0, kMiss };

  struct WorkItem {
    std::uint64_t association_key{0U};
    WorkItemKind kind{WorkItemKind::kMiss};
    RirTrackState track_before;
    const RirTrackMeasurement* measurement{nullptr};
    RirTrackStatus status_before{RirTrackStatus::kTentative};
    bool track_existed_before_cycle{false};
    RirImmFilter* imm_filter{nullptr}; /**< IMM 运行态；空表示本周期走 CV KF 路径。 */
  };

  struct WorkResult {
    std::uint64_t association_key{0U};
    RirTrackState track_after;
    bool should_recycle{false};
  };

  bool PromoteState(RirTrackState* track, std::uint32_t cycle_index, bool hit_this_cycle);
  void ApplyGaussianState(RirTrackState* track, const RirGaussianState& state,
                          const Eigen::Vector3f& previous_velocity, float dt_sec) const;
  void ApplyHitFilter(RirTrackState* track, const RirTrackMeasurement& measurement,
                      RirTrackStatus status_before, float dt_sec, float acceleration_dt_sec,
                      RirImmFilter* imm_filter) const;
  void ApplyMissPredict(RirTrackState* track, const Eigen::Vector3f& previous_velocity,
                        float dt_sec, RirImmFilter* imm_filter) const;

  /** @brief IMM 是否启用且注入契约成立（模型集已构建）。 */
  bool ShouldUseImmForMeasurement(bool track_existed_before_cycle,
                                  RirTrackStatus status_before, bool matched_existing_track) const;
  /** @brief 失配路径是否走 IMM：confirmed 航迹且已有 IMM 运行态。 */
  bool ShouldUseImmForMiss(RirTrackStatus status_before) const;
  /** @brief 取或建关联键对应的 IMM 运行态（以 initial_state 播种各模型分支）。 */
  RirImmFilter* GetOrCreateImmFilter(std::uint64_t association_key,
                                     const RirGaussianState& initial_state);
  /** @brief 由当前 lifecycle 配置构建 IMM 包装配置（建轨与热调参同源）。 */
  RirImmFilter::Config BuildImmFilterConfig() const;

  /** @brief 复用槽位业务字段清零；`generation` 保持单调不清零。 */
  static void ResetForReuse(RirTrackState& track);

  /** @brief 回收航迹：代次 +1、业务字段清零、归还对象池。 */
  void RecycleTrack(RirTrackState* track);

  RirLifecycleConfig lifecycle_config_{};
  RirTrackFilterConfig filter_config_{}; /**< 滤波器配置副本（IMM 模型集锚定其 q）。 */
  RirTrackFilter filter_;
  RirTrackPool pool_{};
  std::map<std::uint64_t, RirTrackState*> tracks_;
  std::unordered_map<std::uint64_t, std::unique_ptr<RirImmFilter>> imm_filters_by_key_{};
  bool imm_enabled_{false};
  std::uint64_t next_track_id_{1U};
  std::uint32_t last_cycle_index_{0U};
};

}  // namespace tracking
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_LIFECYCLE_H_
