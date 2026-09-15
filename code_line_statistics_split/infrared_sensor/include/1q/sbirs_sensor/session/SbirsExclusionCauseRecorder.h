/**
 * @file SbirsExclusionCauseRecorder.h
 * @brief SBIRS 排除原因跨周期差分记录器（规则 13b 排除诊断的差分观测）。
 *
 * 对持续被排除的目标，当其排除原因（code + cause 组合对）跨周期变化时产生结构化事件，
 * 与既有探测生命周期事件并列（独立 recorder、独立 `GetLastEvents()` 通道）。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXCLUSION_CAUSE_RECORDER_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXCLUSION_CAUSE_RECORDER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace sbirs_sensor {
namespace session {

struct SbirsCycleInput;

/**
 * @brief 排除原因跨周期变化事件类型（需求文档 §2.3 转换语义）。
 *
 * A1（原因稳定）不产生事件。
 */
enum class ONEQ_API SbirsExclusionCauseEventKind {
  kEntered = 0, /**< A2：目标从未被排除（kNone）进入被排除。 */
  kChanged = 1, /**< A3：被排除目标的 (code,cause) 组合对发生变化（本需求核心）。 */
  kExited = 2   /**< A4：目标从被排除恢复为未被排除（kNone）。 */
};

/**
 * @brief 单条排除原因跨周期变化事件记录。
 *
 * `previous_*` 为空（code）/ kNone（cause）表示上一周期未被排除（A2）；
 * `current_*` 为空（code）/ kNone（cause）表示本周期不再被排除（A4）。
 */
struct ONEQ_API SbirsExclusionCauseEvent {
  std::uint32_t cycle_index{0U};                                    /**< 触发该事件的周期序号 */
  std::uint64_t target_id{0U};                                      /**< 目标 ID */
  std::string target_name{};                                        /**< 目标名称（人读标签） */
  SbirsExclusionCauseEventKind kind{SbirsExclusionCauseEventKind::kChanged}; /**< 事件类型 */
  std::string previous_code{};                                      /**< 上一执行周期排除 code（空 = 上周未被排除） */
  SbirsIssueCause previous_cause{SbirsIssueCause::kNone};           /**< 上一执行周期排除主因 */
  std::string current_code{};                                       /**< 本执行周期排除 code（空 = 本周不再被排除） */
  SbirsIssueCause current_cause{SbirsIssueCause::kNone};            /**< 本执行周期排除主因 */
};

/**
 * @brief 排除原因跨周期差分记录器。
 *
 * 转换检测状态机（非数据存储）：累积状态刻意最小化为每目标上一执行周期的
 * (code,cause) 组合对（无条目 = 上周未被排除）。非 completed 周期不产生事件，
 * 也不推进记录器状态（与既有 `SbirsDetectionLifecycleRecorder` 语义一致）。
 *
 * 差分键为 (code,cause) 组合对：SBIRS 排除诊断涵盖 4 个 code（遮挡/距离带/视场/SNR），
 * 其中遮挡与距离带为具体门（cause 恒 kNone）。用 code+cause 组合键能正确捕获
 * 遮挡↔距离带切换（同为 kNone、code 不同）的 A3 变化，避免纯 cause 键的盲区。
 *
 * 纯观测：只读 `result.issues`（按 `location.kind == kSceneEntity` 关联目标），
 * 不改变 `*CycleStatus`、排除诊断、DebugView 状态语义（规则 13c 边界延续）。
 * 私有状态（含 unordered_map）与判定逻辑见 .cpp，避免在 public header 暴露实现细节。
 *
 * @note 单目标单周期多条排除 issue 的假设：当前 SBIRS 四门互斥（遮挡→距离带→视场→SNR
 *       在管线中为 continue 链，单周期单目标最多命中一门），取按 location 命中的第一条；
 *       若未来门并发需 revisit。
 */
class ONEQ_API SbirsExclusionCauseRecorder {
 public:
  SbirsExclusionCauseRecorder();
  ~SbirsExclusionCauseRecorder();

  SbirsExclusionCauseRecorder(const SbirsExclusionCauseRecorder&) = delete;
  SbirsExclusionCauseRecorder& operator=(const SbirsExclusionCauseRecorder&) = delete;
  SbirsExclusionCauseRecorder(SbirsExclusionCauseRecorder&&) noexcept;
  SbirsExclusionCauseRecorder& operator=(SbirsExclusionCauseRecorder&&) noexcept;

  /**
   * @brief 基于目标事实与单周期结果产出排除原因跨周期变化事件。
   *
   * 仅处理 `target_id != 0` 的输入目标；按 A2/A3/A4 转换规则生成事件，
   * A1（原因稳定）不产生事件。差分原料取自 `result.issues` 中
   * `location.kind == kSceneEntity` 的排除诊断条目（按 `entity_index` 关联目标）。
   *
   * @param[in] input 当前周期输入（用于遍历场景目标表与回查 target_name）。
   * @param[in] result 当前周期结果（用于读取排除诊断与周期号）。
   * @return 本周期产生的排除原因变化事件列表（可能为空）。
   */
  std::vector<SbirsExclusionCauseEvent> Update(const SbirsCycleInput& input,
                                               const SbirsCycleResult& result);

  /**
   * @brief 清空内部目标排除状态，回到初始状态。
   */
  void Reset();

  /**
   * @brief 获取最近一次执行周期 `Update()` 返回的事件列表。
   *
   * 事件在每次执行周期的 `Update()` 调用时缓存；非执行周期不刷新缓存，
   * 保留上一次执行周期的事件。供注册到 Session 后由调用方事后读取。
   * @return 最近一次执行周期 `Update()` 产生的事件列表的 const 引用。
   */
  const std::vector<SbirsExclusionCauseEvent>& GetLastEvents() const noexcept;

 private:
  // 不透明私有状态，定义在 .cpp 中，避免在 header 暴露 <unordered_map> 依赖。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXCLUSION_CAUSE_RECORDER_H_
