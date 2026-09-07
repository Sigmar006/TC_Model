/**
 * @file EcmSession.h
 * @brief 定义工程级电子对抗会话门面。
 */

#ifndef ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_SESSION_H_
#define ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/electronic_countermeasure/EcmTypes.h"

namespace electronic_countermeasure {
namespace session {

/**
 * @brief EcmSession 提供确定性压制干扰调度与累积资源状态。
 * @note 会话持有可变状态且默认非线程安全；不公开内部规划器扩展点。
 */
class ONEQ_API EcmSession {
 public:
  EcmSession();
  ~EcmSession();
  EcmSession(const EcmSession&) = delete;
  EcmSession& operator=(const EcmSession&) = delete;
  EcmSession(EcmSession&&) noexcept;
  EcmSession& operator=(EcmSession&&) noexcept;

  /** @brief 使用会话配置创建 ECM 会话。 */
  static EcmSession Create(const config::EcmSessionConfig& config = {});

  /**
   * @brief 设置传感器/对抗设备实体 ID（用于 RF 发射身份与场景标识对齐）。
   * @param[in] sensor_entity_id 传感器/对抗设备实体 ID；传 0 恢复缺省常量（101U）。
   * @note 构造后注入设备号，任意时刻可调，下一周期起生效。
   */
  void SetSensorEntityId(std::uint32_t sensor_entity_id) noexcept;

  /** @brief 原子执行一个 ECM 周期并返回实际发射和结构化决策。 */
  EcmCycleResult StepWithResult(const EcmCycleInput& input);

  /** @brief 原子应用运行期配置补丁。 */
  EcmRuntimeConfigApplyResult ApplyRuntimeConfig(
      const config::EcmRuntimeConfigPatch& patch);

  /** @brief 捕获全部累积调度、热状态和确定性随机流。 */
  EcmRuntimeState CaptureRuntimeState() const;

  /** @brief 原子恢复由当前会话捕获的兼容快照。 */
  bool RestoreRuntimeState(const EcmRuntimeState& state);

 private:
  struct Impl;
  explicit EcmSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace electronic_countermeasure

#endif  // ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_SESSION_H_
