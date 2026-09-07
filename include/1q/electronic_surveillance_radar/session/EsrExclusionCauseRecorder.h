/**
 * @file EsrExclusionCauseRecorder.h
 * @brief ESR 排除原因跨周期差分记录器（规则 13b 排除诊断的差分观测）。
 *
 * 对持续被排除的发射源，当其排除原因（code + cause 组合对）跨周期变化时产生结构化事件。
 *
 * ESR 无 target_id 概念，以发射源标识（sensor_entity_id/emission_id 二字段）为
 * 实体关联键。排除诊断的 `location.entity_index` = 发射源在 identity 排序后数组中的下标
 * （与 InterceptDetectionExecutor 排序序一致）；记录器 Update 时按同一序重排输入 emissions，
 * 把 entity_index 解析回 identity 二元组，**内部状态以 identity 二元组为键**（免疫跨周期
 * 发射源集合变化时的下标移位）。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EXCLUSION_CAUSE_RECORDER_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EXCLUSION_CAUSE_RECORDER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electromagnetics/RfScene.h"

namespace electronic_surveillance_radar {
namespace session {

struct EsrCycleInput;

/**
 * @brief 排除原因跨周期变化事件类型（需求文档 §2.3 转换语义）。
 *
 * A1（原因稳定）不产生事件。
 */
enum class ONEQ_API EsrExclusionCauseEventKind {
  kEntered = 0, /**< A2：发射源从未被排除（kNone）进入被排除。 */
  kChanged = 1, /**< A3：被排除发射源的 (code,cause) 组合对发生变化（本需求核心）。 */
  kExited = 2   /**< A4：发射源从被排除恢复为未被排除（kNone）。 */
};

/**
 * @brief 单条排除原因跨周期变化事件记录。
 *
 * `previous_*` 为空（code）/ kNone（cause）表示上一周期未被排除（A2）；
 * `current_*` 为空（code）/ kNone（cause）表示本周期不再被排除（A4）。
 * 发射源标识二字段（sensor_entity_id/emission_id）结构化携带，供集成方按实体关联。
 */
struct ONEQ_API EsrExclusionCauseEvent {
  std::uint32_t cycle_index{0U};                                    /**< 触发该事件的周期序号 */
  oneq::electromagnetics::RfEmissionIdentity identity{};            /**< 发射源标识（实体关联键） */
  EsrExclusionCauseEventKind kind{EsrExclusionCauseEventKind::kChanged}; /**< 事件类型 */
  std::string previous_code{};                                      /**< 上一执行周期排除 code（空 = 上周未被排除） */
  EsrIssueCause previous_cause{EsrIssueCause::kNone};               /**< 上一执行周期排除主因 */
  std::string current_code{};                                       /**< 本执行周期排除 code（空 = 本周不再被排除） */
  EsrIssueCause current_cause{EsrIssueCause::kNone};                /**< 本执行周期排除主因 */
};

/**
 * @brief 排除原因跨周期差分记录器。
 *
 * 转换检测状态机（非数据存储）：累积状态刻意最小化为每发射源上一执行周期的
 * (code,cause) 组合对（无条目 = 上周未被排除）。非 completed 周期不产生事件，
 * 也不推进记录器状态。
 *
 * 差分键为 (code,cause) 组合对（与 AR/SBIRS/EOS 一致）。
 *
 * **实体关联**：ESR 排除诊断的 `location.entity_index` = 发射源在 identity 排序后数组中
 * 的下标（InterceptDetectionExecutor 按 sensor_entity_id/emission_id 排序）。记录器 Update
 * 时按同一序重排 `input.rf_emissions.emissions`，把 entity_index 解析回 identity 二元组，
 * 以 identity 为内部状态键——免疫跨周期发射源集合变化时的下标移位（源 A 消失后源 B 的
 * 下标变化不会误判为 B 的原因变化）。
 *
 * 纯观测：只读 `result.issues`（按 `location.kind == kSceneEntity` 关联），
 * 不改变 `*CycleStatus`、排除诊断、DebugView 状态语义（规则 13c 边界延续）。
 * 私有状态（含 unordered_map）与判定逻辑见 .cpp，避免在 public header 暴露实现细节。
 *
 * @note 单发射源单周期多条排除 issue 的假设：当前 ESR 三排除门（co-site/zero-power/
 *       below-threshold）在循环 A 内 continue 链互斥，below-threshold 仅对 cell candidate
 *       最强源触发，每源每周期最多命中一门。取按 location 命中的第一条；若未来门并发需 revisit。
 */
class ONEQ_API EsrExclusionCauseRecorder {
 public:
  EsrExclusionCauseRecorder();
  ~EsrExclusionCauseRecorder();

  EsrExclusionCauseRecorder(const EsrExclusionCauseRecorder&) = delete;
  EsrExclusionCauseRecorder& operator=(const EsrExclusionCauseRecorder&) = delete;
  EsrExclusionCauseRecorder(EsrExclusionCauseRecorder&&) noexcept;
  EsrExclusionCauseRecorder& operator=(EsrExclusionCauseRecorder&&) noexcept;

  /**
   * @brief 基于单周期输入与结果产出排除原因跨周期变化事件。
   *
   * 按 A2/A3/A4 转换规则生成事件，A1（原因稳定）不产生事件。差分原料取自
   * `result.issues` 中 `location.kind == kSceneEntity` 的排除诊断条目；按
   * identity 排序序重排 emissions 把 entity_index 解析回发射源标识。
   *
   * @param[in] input 当前周期输入（提供 RF 发射源列表）。
   * @param[in] result 当前周期结果（提供排除诊断与周期号）。
   * @return 本周期产生的排除原因变化事件列表（可能为空）。
   */
  std::vector<EsrExclusionCauseEvent> Update(const EsrCycleInput& input,
                                             const EsrCycleResult& result);

  /**
   * @brief 清空内部发射源排除状态，回到初始状态。
   */
  void Reset();

  /**
   * @brief 获取最近一次执行周期 `Update()` 返回的事件列表。
   *
   * 事件在每次执行周期的 `Update()` 调用时缓存；非执行周期不刷新缓存，
   * 保留上一次执行周期的事件。供注册到 Session 后由调用方事后读取。
   * @return 最近一次执行周期 `Update()` 产生的事件列表的 const 引用。
   */
  const std::vector<EsrExclusionCauseEvent>& GetLastEvents() const noexcept;

 private:
  // 不透明私有状态，定义在 .cpp 中，避免在 header 暴露 <unordered_map> 依赖。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EXCLUSION_CAUSE_RECORDER_H_
