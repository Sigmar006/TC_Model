/**
 * @file RirTrackLifecycleRecorder.h
 * @brief 远程识别雷达航迹生命周期记录类型集合。
 *
 * 航迹首次确认/更新/丢失/未跟踪事件记录的主头文件。观测投影契约见
 * docs/review/rir_observability_projections_freeze_2026-08-21.md §3.3：
 * 航迹级语义对齐 AR（kFirstConfirmed/kUpdated/kLost/kNotTracked），另含
 * RIR 特有的指定识别任务作废/回扫终态事件（kDesignationDropped）。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_TRACK_LIFECYCLE_RECORDER_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_TRACK_LIFECYCLE_RECORDER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirSceneTypes.h"

namespace remote_identification_radar {
namespace session {

/**
 * @brief 航迹生命周期事件类型。
 */
enum class ONEQ_API RirTrackLifecycleEventKind {
  kFirstConfirmed = 0, /**< 目标对应航迹首次进入已确认状态。 */
  kUpdated = 1,        /**< 已确认航迹本周期再次更新。 */
  kLost = 2,           /**< 航迹进入丢失状态。 */
  kNotTracked = 3,     /**< 输入目标无对应航迹（需显式开启诊断）。 */
  kDesignationDropped = 4 /**< 指定识别任务作废/回扫终态（镜像信封 designation_revert_reason）。 */
};

/**
 * @brief 未跟踪事件的成因归类（仅在 kNotTracked 诊断时填充）。
 */
enum class ONEQ_API RirTrackLifecycleReason {
  kNone = 0,    /**< 无特殊成因（确认/更新/丢失事件均用此值）。 */
  kNoTrack = 1, /**< 本周期未为该输入目标建立任何航迹。 */
  kUnknown = 2  /**< 其他无法归类的情形。 */
};

/**
 * @brief 单条航迹生命周期事件记录。
 */
struct ONEQ_API RirTrackLifecycleEvent {
  std::uint32_t world_cycle_index{0U}; /**< 触发该事件的输入周期号 */
  std::uint64_t external_target_id{0U}; /**< 输入目标外部标识符 */
  std::string target_name{};            /**< 目标名称（人读标签） */
  RirTrackLifecycleEventKind kind{RirTrackLifecycleEventKind::kUpdated}; /**< 事件类型 */
  RirTrackLifecycleReason reason{RirTrackLifecycleReason::kNone}; /**< 事件成因（诊断用） */
  std::uint64_t association_key{0U};    /**< 关联航迹的关联键（无航迹时为 0） */
  RirTrackLifecycleStatus track_status{RirTrackLifecycleStatus::kTentative}; /**< 关联航迹状态 */
  double speed_m_per_s{0.0};            /**< 关联航迹滤波速度模长（无航迹时为 0） */
  RirDesignationRevertReason designation_revert_reason{
      RirDesignationRevertReason::kNone}; /**< 指定任务作废成因（仅 kDesignationDropped 填充） */
};

/**
 * @brief 航迹生命周期记录器配置。
 */
struct ONEQ_API RirTrackLifecycleRecorderConfig {
  bool emit_not_tracked_events{false}; /**< 是否为无对应航迹的输入目标产生 kNotTracked 事件 */
};

/**
 * @brief 记录航迹首次确认/更新/丢失/未跟踪事件；未跟踪原因需显式开启。
 *
 * 生命周期贴合 RIR 航迹语义（kTentative/kConfirmed/kLost）：
 * 首次进入 kConfirmed 产生 kFirstConfirmed；已确认周期更新产生 kUpdated；
 * 进入 kLost 产生 kLost；输入目标无对应航迹且开启诊断时产生 kNotTracked；
 * 指定识别任务回扫沿（镜像信封 designation_reverted_to_scan）产生
 * kDesignationDropped。非 completed 周期不产生事件，也不推进记录器状态。
 * 私有状态（含 unordered_map）与判定逻辑见 .cpp，避免在 public header 暴露实现细节。
 */
class ONEQ_API RirTrackLifecycleRecorder {
 public:
  explicit RirTrackLifecycleRecorder(
      RirTrackLifecycleRecorderConfig config = RirTrackLifecycleRecorderConfig{});
  ~RirTrackLifecycleRecorder();

  RirTrackLifecycleRecorder(const RirTrackLifecycleRecorder&) = delete;
  RirTrackLifecycleRecorder& operator=(const RirTrackLifecycleRecorder&) = delete;
  // 移动操作声明在 header、定义在 .cpp：unique_ptr<Impl> 析构需要完整类型，
  // 不能内联定义否则破坏 PImpl 不透明性。
  RirTrackLifecycleRecorder(RirTrackLifecycleRecorder&&) noexcept;
  RirTrackLifecycleRecorder& operator=(RirTrackLifecycleRecorder&&) noexcept;

  /**
   * @brief 基于场景目标事实与单周期结果产出生命周期事件。
   *
   * 仅处理 `external_target_id != 0` 的输入目标；按确认/更新/丢失/未跟踪规则
   * 生成事件，未跟踪事件仅在配置开启时产生。
   *
   * @param[in] targets 当前周期场景目标事实（用于遍历输入目标表）。
   * @param[in] result 当前周期结果（用于查询归属航迹状态）。
   * @return 本周期产生的生命周期事件列表（可能为空）。
   */
  std::vector<RirTrackLifecycleEvent> Update(const RirSceneTargetList& targets,
                                             const RirCycleResult& result);

  /**
   * @brief 清空内部航迹状态，回到初始状态。
   */
  void Reset();

  /**
   * @brief 获取最近一次执行周期 `Update()` 返回的生命周期事件列表。
   *
   * 事件在每次执行周期的 `Update()` 调用时缓存；非执行周期不刷新缓存，
   * 保留上一次执行周期的事件。供注册到 Session 后由调用方事后读取。
   * @return 最近一次执行周期 `Update()` 产生的事件列表的 const 引用。
   */
  const std::vector<RirTrackLifecycleEvent>& GetLastEvents() const noexcept;

 private:
  // 不透明私有状态，定义在 .cpp 中，避免在 header 暴露 <unordered_map> 依赖。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_TRACK_LIFECYCLE_RECORDER_H_
