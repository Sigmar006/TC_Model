/**
 * @file ArSession.h
 * @brief 定义面向外部接入的高层机载雷达会话门面。
 *
 * AR 模块会话门面（一步一帧执行、运行期补丁）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_SESSION_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_SESSION_H_

#include <memory>
#include <vector>

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/config/ArSessionConfigValidation.h"
#include "1q/airborne_radar/session/ArCommand.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/api.hpp"
#include "1q/foundation/SensorContract.h"

namespace airborne_radar {
namespace config {
struct ArRuntimeConfigPatch;
}  // namespace config
}  // namespace airborne_radar

namespace airborne_radar {
namespace session {

class ArSessionReplayAccess;
class ArTrackLifecycleRecorder;
class ArExclusionCauseRecorder;

/**
 * @brief ArSession 提供"一步一帧"的外部接入门面。
 */
class ONEQ_API ArSession {
 public:
  ArSession();
  ~ArSession();

  ArSession(const ArSession&) = delete;
  ArSession& operator=(const ArSession&) = delete;
  ArSession(ArSession&&) noexcept;
  ArSession& operator=(ArSession&&) noexcept;

  /**
   * @brief 执行一个不显式切场景的处理周期。
   * @param[in] input 当前周期输入。
   * @return 当前周期生成的轨迹输出帧拷贝。
   * @note 非法或被拒绝的周期返回默认的当前帧，不复用历史输出。
   */
  TrackOutputFrame Step(const ArCycleInput& input);

  /**
   * @brief 执行一个不显式切场景的处理周期，并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   * @note `status` 是结果有效性的唯一真相；只有 `kCompleted` 携带本周期
   *       探测/跟踪输出，`kPoweredOff` 只推进世界时间。
   */
  ArCycleResult StepWithResult(const ArCycleInput& input);

  /** @brief 获取当前周期已提交的控制指令。
   * @return 最近一次成功执行周期提交的控制指令列表引用。
   */
  const std::vector<session::ArCommand>& GetSubmittedCommands() const;

  /**
   * @brief 判断是否已经保存过最新控制真值。
   * @return 若已持有最近一次成功执行周期留下的控制真值则返回 true。
   */
  bool HasLatestControlProfile() const;

  /**
   * @brief 获取最近一次控制真值。
   * @return 最近一次成功执行周期留下的控制真值引用。
   */
  const session::ArControlProfile& GetLatestControlProfile() const;

  /**
   * @brief 获取最近一次关联质量观测指标。
   * @return 最近一次成功执行周期留下的关联质量观测指标。
   */
  session::AssociationQualityMetrics GetLastAssociationQualityMetrics() const;

  /**
   * @brief 尝试应用运行期可变配置补丁。
   * @param[in] patch 运行期可变配置补丁。
   * @note 该接口作为运行期可调参数的统一入口；未设置的字段保持现值不变。
   *       补丁会先进入 session 内部暂存区，并在下一次成功执行主链路或确认设备关机的
   *       `Step()/StepWithResult()` 边界最终提交；若本次调用在下游执行阶段失败，补丁仍
   *       保持 staged 状态。
   * @return 补丁被接受并暂存成功时返回 true；补丁无效或无变更时返回 false。
   */
  bool TryApplyRuntimeConfig(const config::ArRuntimeConfigPatch& patch);

  /**
   * @brief 注册轨迹生命周期记录器，由 Session 在每个周期自动驱动。
   *
   * 注册后，`StepWithResult()` 和 `Step()` 内部在 CycleResult 构建完成后自动调用
   * `recorder->Update()`，调用方无需手动调用。本周期产生的生命周期事件可通过
   * `recorder->GetLastEvents()` 获取。
   * @param[in] recorder 记录器指针；传入 `nullptr` 解除注册。
   * @note Session 不拥有 recorder，调用方须保证 recorder 生命周期长于 Session 的注册期。
   * @note 轨迹事件通知（首确认/更新/丢失）建议通过本机制获取：recorder 内建
   *       跨周期状态迁移差分与掉轨重捕语义。
   */
  void AttachTrackLifecycleRecorder(ArTrackLifecycleRecorder* recorder) noexcept;

  /**
   * @brief 注册排除原因跨周期差分记录器，由 Session 在每个周期自动驱动。
   *
   * 与 `AttachTrackLifecycleRecorder` 独立并列：一个 Session 可同时注册两者，
   * 各自独立驱动、独立 `GetLastEvents()` 通道，注册与否互不影响、不影响执行语义
   *（规则 11c）。
   * @param[in] recorder 记录器指针；传入 `nullptr` 解除注册。
   * @note Session 不拥有 recorder，调用方须保证 recorder 生命周期长于 Session 的注册期。
   * @note 排除原因变化观测（进入/原因变化/退出排除）建议通过本机制获取：recorder 内建
   *       跨周期 (code,cause) 对差分，规则 13b 排除诊断的差分观测。
   */
  void AttachExclusionCauseRecorder(ArExclusionCauseRecorder* recorder) noexcept;

  /**
   * @brief 设置传感器设备实体 ID（贯通 RF 发射/接收身份、自剔除与融合/日志通道标注）。
   * @param[in] sensor_entity_id 传感器设备实体 ID；传 0 恢复缺省常量。
   * @note 构造后注入设备号，任意时刻可调，下一周期起生效。
   */
  void SetSensorEntityId(std::uint32_t sensor_entity_id) noexcept;

  /** @brief 提交外部 profile 覆盖（整包替换值，绕过 TacticalProposal 管线与 hold/cooldown）。 */
  session::ExternalDecisionSubmitStatus SubmitExternalDecision(
      session::ExternalDecisionOverride override_decision);

  /** @brief 使用四域配置创建会话（推荐入口，信任路径，不做配置校验）。 */
  static ArSession Create(const config::ArSessionConfig& config = {});
  /**
   * @brief 创建会话并报告配置校验结果（校验路径）。
   *
   * 与 `Create()` 唯一区别：构造前调用 `config::ValidateArSessionConfig`
   * 校验配置合法性，将发现的问题写入 @p issues。无论 @p issues 是否为空，
   * 都会构造并返回会话（不阻断），调用方据 `issues->empty()` 决定后续。
   *
   * @param[in] config 四域会话配置。
   * @param[out] issues 校验问题输出；传入 nullptr 则不写回但仍构造会话。
   * @return 构造完成的会话。
   * @note `ValidateArSessionConfig` 由此路径被实调用，构成真实契约。
   */
  static ArSession CreateWithDiagnostics(const config::ArSessionConfig& config,
                                         session::ArIssueList* issues);

 private:
  friend class ArSessionReplayAccess;

  struct Impl;
  explicit ArSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session

// 跨域传感器会话形状契约：锚定 Step/StepWithResult 签名，防止伪对称漂移。
// 注意：AR 主输出帧类型为 TrackOutputFrame（领域历史命名，非 ArOutputFrame）。
ONEQ_SENSOR_SESSION_CONTRACT(session::ArSession, session::ArCycleInput, session::TrackOutputFrame,
                             session::ArCycleResult);

}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_SESSION_H_
