/**
 * @file SbirsSession.h
 * @brief 定义 SBIRS-inspired 会话门面。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SESSION_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/foundation/SensorContract.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsExternalCue.h"

namespace sbirs_sensor {
namespace session {

class SbirsDetectionLifecycleRecorder;
class SbirsExclusionCauseRecorder;

/**
 * @brief SBIRS-inspired 会话门面，是外部调用方的主要使用面。
 * @note 该类不可拷贝但可移动，内部持有实现 (PIMPL)。实例本身非线程安全；
 *       运行期配置采用立即提交策略；pipeline snapshot 是经完整校验的 internal checkpoint，
 *       当前 controller 周期路径不声明执行后回滚步骤。
 */
class ONEQ_API SbirsSession {
 public:
  SbirsSession();
  ~SbirsSession() noexcept;

  SbirsSession(const SbirsSession&) = delete;
  SbirsSession& operator=(const SbirsSession&) = delete;
  SbirsSession(SbirsSession&&) noexcept;
  SbirsSession& operator=(SbirsSession&&) noexcept;

  /**
   * @brief 执行一个仿真周期，返回原始系统输出帧（主输出层）。
   * @param[in] input 单周期输入
   * @return 本周期的 `SbirsOutputFrame`
   */
  SbirsOutputFrame Step(const SbirsCycleInput& input);
  /**
   * @brief 执行一个仿真周期，返回结构化执行结果（含输出帧、归属、校验与状态）。
   * @param[in] input 单周期输入
   * @return 本周期的 `SbirsCycleResult`
   */
  SbirsCycleResult StepWithResult(const SbirsCycleInput& input);
  /**
   * @brief 尝试提交运行期配置补丁并立即生效。
   * @param[in] patch 运行期配置补丁
   * @return patch 有效且产生更新返回 true；patch 无效或无可更新项返回 false
   */
  bool TryApplyRuntimeConfig(const config::SbirsRuntimeConfigPatch& patch);

  /**
   * @brief 运行时注入一条星间 cross-cue（交叉提示）引导消息。
   * @param[in] cue 引导消息：来源星宽场带误差测角（ECI）+ 带误差距离 + 来源星 ECEF 位置。
   * @note 运行时输入（修订 5，2026-09-01）：不进每帧 `SbirsCycleInput`；消息入运行期
   *       队列，下一次 `StepWithResult` 开始时消费。非法消息（非有限角度/非正距离/
   *       未知目标键）消费时丢弃并计入该周期 issues（kInfo）。队列不属于快照：回放
   *       路径从不注入，恢复后行为逐位一致。实例非线程安全（与周期步进同线程调用）。
   */
  void SubmitExternalCue(const SbirsExternalCue& cue);

  /**
   * @brief 标注本会话卫星实体/融合源 ID（仅进验收日志行的 卫星ID=/相对卫星ID= 字段）。
   * @param[in] satellite_entity_id 场景实体/融合源 ID；双星场景各星互异（如 4 与 104）。
   * @note 不影响任何计算路径；未标注时验收行如实写 0。
   */
  void SetSatelliteEntityId(std::uint32_t satellite_entity_id) noexcept;

  /**
   * @brief 注册探测生命周期记录器，由 Session 在每个周期自动驱动。
   *
   * 注册后，`StepWithResult()` 和 `Step()` 内部在 CycleResult 构建完成后自动调用
   * `recorder->Update()`，调用方无需手动调用。本周期产生的生命周期事件可通过
   * `recorder->GetLastEvents()` 获取。
   * @param[in] recorder 记录器指针；传入 `nullptr` 解除注册。
   * @note Session 不拥有 recorder，调用方须保证 recorder 生命周期长于 Session 的注册期。
   */
  void AttachDetectionLifecycleRecorder(SbirsDetectionLifecycleRecorder* recorder) noexcept;

  /**
   * @brief 注册排除原因跨周期差分记录器，由 Session 在每个周期自动驱动。
   *
   * 与 `AttachDetectionLifecycleRecorder` 独立并列：一个 Session 可同时注册两者，
   * 各自独立驱动、独立 `GetLastEvents()` 通道，注册与否互不影响、不影响执行语义
   *（规则 11c）。
   * @param[in] recorder 记录器指针；传入 `nullptr` 解除注册。
   * @note Session 不拥有 recorder，调用方须保证 recorder 生命周期长于 Session 的注册期。
   * @note 排除原因变化观测（进入/原因变化/退出排除）建议通过本机制获取：recorder 内建
   *       跨周期 (code,cause) 对差分，规则 13b 排除诊断的差分观测。
   */
  void AttachExclusionCauseRecorder(SbirsExclusionCauseRecorder* recorder) noexcept;

  /** @brief 使用四域配置创建会话（推荐入口，信任路径，不做配置校验）。 */
  static SbirsSession Create(const config::SbirsSessionConfig& config = {});
  /**
   * @brief 创建会话并报告配置校验结果（校验路径）。
   *
   * 与 `Create()` 唯一区别：构造前调用 `config::ValidateSbirsSessionConfig`
   * 校验配置合法性，将发现的问题写入 @p issues。无论 @p issues 是否为空，
   * 都会构造并返回会话（不阻断），调用方据 `issues->empty()` 决定后续。
   *
   * @param[in] config 四域会话配置。
   * @param[out] issues 校验问题输出；传入 nullptr 则不写回但仍构造会话。
   * @return 构造完成的会话。
   * @note `ValidateSbirsSessionConfig` 由此路径被实调用，构成真实契约。
   */
  static SbirsSession CreateWithDiagnostics(const config::SbirsSessionConfig& config,
                                            SbirsIssueList* issues);

 private:
  struct Impl;
  explicit SbirsSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session

ONEQ_SENSOR_SESSION_CONTRACT(session::SbirsSession, session::SbirsCycleInput,
                             session::SbirsOutputFrame, session::SbirsCycleResult);

}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SESSION_H_
