/**
 * @file SbirsNfovScheduler.h
 * @brief NFOV 锁定集合管理：候选排序与锁定/释放。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_NFOV_SCHEDULER_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_NFOV_SCHEDULER_H_

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"
#include "sbirs_sensor/pipeline/SbirsEciScene.h"

namespace sbirs_sensor {
namespace pipeline {

/**
 * @brief 单个 WFOV 候选的调度中间结构。
 * @note 由 pipeline 在 WFOV 发现阶段填充（target 指向周期入口旋转后的 ECI 场景
 *       副本），交由调度器排序与选取（design 2.6）。
 */
struct SbirsCandidate {
  const SbirsEciSceneTarget* target{nullptr};
  float azimuth_deg{0.0f};
  float elevation_deg{0.0f};
  float delayed_truth_azimuth_deg{0.0f};   ///< cue 延迟后的仿真真值方位角，仅用于捕获判定
  float delayed_truth_elevation_deg{0.0f}; ///< cue 延迟后的仿真真值俯仰角，仅用于捕获判定
  float measured_azimuth_deg{0.0f};
  float measured_elevation_deg{0.0f};
  float command_azimuth_deg{0.0f};   ///< 仅由 WFOV 测量历史生成的 NFOV 命令方位角
  float command_elevation_deg{0.0f}; ///< 仅由 WFOV 测量历史生成的 NFOV 命令俯仰角
  double range_m{0.0};        ///< 真值距离（调度优先级用）
  double measured_range_m{0.0};  ///< 带误差距离（NFOV cue 与 attribution 诊断用）
  double max_detection_range_m{0.0}; ///< 当前时刻最大探测距离（WFOV 门限反解，归属层诊断用）
  float relative_angular_rate_deg_per_sec{0.0f}; ///< 相对视线角速度（v_target−v_satellite 推导），Sensor-like 成功观测的动态滞后输入
  double snr{0.0};
  int cue_source_satellite_entity_id{-1}; ///< 引导来源星 ID（-1=自星宽场；cross-cue 候选填来源星，只记录不参与排序）
  /**
   * @brief 候选来源是否为外部引导（cross-cue 旁路）。
   *
   * @details `cue_source_satellite_entity_id == -1` 同时可能是"自星宽场"或"非卫星来源的
   *   外部引导"（例如雷达引导），日志无法区分，会让"SNR 低于宽场门限却已选中"看起来
   *   自相矛盾。本字段把两条路径分开标注（2026-10-21 红外特性日志修复）。
   */
  bool from_external_cue{false};
  double measurement_time_s{0.0};         ///< 绝对测量时刻（秒）
};

/**
 * @brief NFOV 锁定集合快照，用于 pipeline validated checkpoint 与确定性 continuation。
 * @note 仅包含目标→引导来源的集合关系；滤波/IMM 状态由 pipeline 各自管理。
 */
struct SbirsNfovSchedulerSnapshot {
  std::map<std::uint64_t, int> target_to_cue_source{}; ///< 锁定目标 ID 到引导来源星 ID 的映射（-1=自星宽场）
};

/**
 * @brief NFOV 锁定集合管理器（单镜筒，2026-09-02）。
 * @details 窄场只有一个镜筒：锁定集合无配置上限，容量由单镜筒分时轮转的物理涌现——
 *          转场占用窗口、轮转周期超过跟踪门容忍的目标自动丢锁（见 pointing/pipeline）。
 *          新目标按 design 2.6 优先级规则（SNR 降序 → 距离升序 → target_id 升序）
 *          从 WFOV 候选中选取进入首次捕获队列；map 有序保证相同输入下集合遍历
 *          确定（replay 可复现）。
 */
class SbirsNfovScheduler {
 public:
  /** @brief 构造调度器。 */
  SbirsNfovScheduler() = default;

  /** @return 当前锁定集合大小。 */
  std::size_t LockedCount() const { return target_to_cue_source_.size(); }

  /**
   * @brief 按优先级排序候选，选出可进入首次捕获的目标（无并发截断）。
   * @param[in,out] candidates WFOV 候选列表（就地排序，稳定规则保证 replay 可复现）
   * @return 可进入首次捕获的候选指针列表（按优先级排序；已锁定目标不重复入选）
   */
  std::vector<const SbirsCandidate*> SelectForAcquisition(std::vector<SbirsCandidate>& candidates);

  /** @return 目标是否在锁定集合内。 */
  bool IsLocked(std::uint64_t target_id) const;

  /**
   * @brief 将目标加入锁定集合。
   * @param[in] target_id 待锁定的目标 ID
   * @param[in] cue_source_satellite_entity_id 引导来源星 ID（-1=自星宽场引导；cross-cue
   *            锁定后随集合持久记录，Release/Clear 单点清除，供归属层"引导来源"标记）
   * @return 加入成功（已在集合内时幂等返回 true）
   */
  bool Acquire(std::uint64_t target_id, int cue_source_satellite_entity_id = -1);

  /**
   * @brief 将目标移出锁定集合（目标消失、门丢锁或传感器进入 standby）。
   * @param[in] target_id 待释放的目标 ID
   */
  void Release(std::uint64_t target_id);

  /**
   * @return 目标锁定时的引导来源星 ID（cross-cue 归属标记）。
   * @return 引导来源星 ID；-1=自星宽场引导或目标未锁定。
   */
  int CueSourceOf(std::uint64_t target_id) const;

  /** @return 锁定集合内全部目标 ID（map 有序，升序）。 */
  std::vector<std::uint64_t> LockedTargetIds() const;

  /** @brief 清空锁定集合（standby / 重置）。 */
  void Clear();

  /** @return 当前调度状态快照，用于 pipeline checkpoint 与状态恢复测试。 */
  SbirsNfovSchedulerSnapshot Capture() const;

  /**
   * @brief 从快照恢复调度状态。
   * @param[in] snapshot 待恢复的状态快照
   */
  void Restore(const SbirsNfovSchedulerSnapshot& snapshot);

 private:
  std::map<std::uint64_t, int> target_to_cue_source_{};
};

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_NFOV_SCHEDULER_H_
