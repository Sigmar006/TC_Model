/**
 * @file EosSession.h
 * @brief 定义光学传感器对外会话门面。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigValidation.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/foundation/SensorContract.h"

namespace electro_optical_sensor {
namespace session {

class EosDetectionLifecycleRecorder;
class EosExclusionCauseRecorder;

/**
 * @brief EosSession 提供单周期步进执行入口。
 * @note 通过静态工厂 `EosSession::Create` 创建，避免外部直接拼装不一致依赖图。
 * @note 线程模型：会话内部维护可变运行态，非线程安全；并发调用需外部串行化或加锁。
 */
class ONEQ_API EosSession {
 public:
  EosSession();
  ~EosSession() noexcept;

  EosSession(const EosSession&) = delete;
  EosSession& operator=(const EosSession&) = delete;
  EosSession(EosSession&&) noexcept;
  EosSession& operator=(EosSession&&) noexcept;

  /**
   * @brief 执行单周期并返回输出帧（输出便捷入口）。
   * @param[in] input 当前周期输入。
   * @return 当前周期输出帧。
   * @note 该接口仅返回输出帧，不携带 `status` 等状态语义；
   *       若调用方需要区分"本周期实际执行"与"非执行周期"，请使用 `StepWithResult()`。
   * @note 非线程安全：会读写会话内部状态；并发调用需外部同步。
   */
  EosOutputFrame Step(const EosCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   * @note 结果中的 `status` / `abort_reason` 提供结构化周期状态语义。
   * @note 非线程安全：会读写会话内部状态；并发调用需外部同步。
   */
  EosCycleResult StepWithResult(const EosCycleInput& input);

  /**
   * @brief 尝试应用运行期可变配置补丁。
   * @param[in] patch 运行期可变配置补丁。
   * @return 补丁被接受并应用成功时返回 true；补丁无效或无变更时返回 false。
   */
  bool TryApplyRuntimeConfig(const config::EosRuntimeConfigPatch& patch);

  /**
   * @brief 注册探测生命周期记录器，由 Session 在每个周期自动驱动。
   *
   * 注册后，`StepWithResult()` 和 `Step()` 内部在 CycleResult 构建完成后自动调用
   * `recorder->Update()`，调用方无需手动调用。本周期产生的生命周期事件可通过
   * `recorder->GetLastEvents()` 获取。
   * @param[in] recorder 记录器指针；传入 `nullptr` 解除注册。
   * @note Session 不拥有 recorder，调用方须保证 recorder 生命周期长于 Session 的注册期。
   * @note 探测事件通知（首发现/更新/丢失）建议通过本机制获取：recorder 内建
   *       跨周期状态迁移差分语义。
   */
  void AttachDetectionLifecycleRecorder(EosDetectionLifecycleRecorder* recorder) noexcept;

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
  void AttachExclusionCauseRecorder(EosExclusionCauseRecorder* recorder) noexcept;

  /**
   * @brief 设置传感器设备实体 ID（用于周期摘要日志行标记与融合源通道对齐）。
   * @param[in] sensor_entity_id 传感器设备实体 ID；传 0 恢复缺省常量。
   * @note 构造后注入设备号，任意时刻可调，下一周期起生效。不影响光电物理计算。
   */
  void SetSensorEntityId(std::uint32_t sensor_entity_id) noexcept;

  /** @brief 使用四域配置创建会话（推荐入口，信任路径，不做配置校验）。 */
  static EosSession Create(const config::EosSessionConfig& config = {});

  /**
   * @brief 创建会话并报告配置校验结果（校验路径）。
   *
   * 与 `Create()` 唯一区别：构造前调用 `config::ValidateEosSessionConfig`
   * 校验配置合法性，将发现的问题写入 @p issues。无论 @p issues 是否为空，
   * 都会构造并返回会话（不阻断），调用方据 `issues->empty()` 决定后续。
   *
   * @param[in] config 四域会话配置。
   * @param[out] issues 校验问题输出；传入 nullptr 则不写回但仍构造会话。
   * @return 构造完成的会话。
   * @note `ValidateEosSessionConfig` 由此路径被实调用，构成真实契约。
   */
  static EosSession CreateWithDiagnostics(const config::EosSessionConfig& config,
                                          EosIssueList* issues);

 private:

  struct Impl;
  explicit EosSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session

// 跨域传感器会话形状契约：锚定 Step/StepWithResult 签名，防止伪对称漂移。
ONEQ_SENSOR_SESSION_CONTRACT(session::EosSession, session::EosCycleInput,
                             session::EosOutputFrame, session::EosCycleResult);

}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_H_
