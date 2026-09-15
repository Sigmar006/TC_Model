/**
 * @file SbirsRecordingSession.h
 * @brief 为 SBIRS 模块提供 Replay 记录包装器。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_RECORDING_SESSION_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_RECORDING_SESSION_H_

#include <memory>
#include <utility>

#include "1q/sbirs_sensor/session/SbirsSession.h"

namespace oneq {
namespace replay {
class ReplayTraceWriter;
}
}  // namespace oneq

namespace sbirs_sensor {
namespace session {

/**
 * @brief trace 记录包装器的配置选项。
 * @note 提供 `replay_writer` 时写出可被 `ReplaySbirsTrace()` 消费的目录。
 */
struct ONEQ_API SbirsRecordingSessionOptions {
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{}; /**< replay trace writer */
  bool record_config_on_construct{true};                          /**< 构造时是否记录配置快照 */

  SbirsRecordingSessionOptions() = default;
};

/**
 * @brief 在 `SbirsSession` 之上叠加 Replay 记录能力的包装会话。
 * @note 该类不可拷贝但可移动，内部持有实现 (PIMPL)；实例本身非线程安全。
 */
class ONEQ_API SbirsRecordingSession {
 public:
  /**
   * @brief 构造 Replay 记录包装会话。
   * @param[in] config 会话初始化配置
   * @param[in] options Replay 记录选项
   */
  explicit SbirsRecordingSession(config::SbirsSessionConfig config = {},
                             SbirsRecordingSessionOptions options = {});
  ~SbirsRecordingSession();

  SbirsRecordingSession(const SbirsRecordingSession&) = delete;
  SbirsRecordingSession& operator=(const SbirsRecordingSession&) = delete;
  SbirsRecordingSession(SbirsRecordingSession&&) noexcept;
  SbirsRecordingSession& operator=(SbirsRecordingSession&&) noexcept;

  /**
   * @brief 执行一个周期并写入 Replay，返回原始系统输出帧。
   * @param[in] input 单周期输入
   * @return 本周期的 `SbirsOutputFrame`
   */
  SbirsOutputFrame Step(const SbirsCycleInput& input);
  /**
   * @brief 执行一个周期并写入 Replay，返回结构化执行结果。
   * @param[in] input 单周期输入
   * @return 本周期的 `SbirsCycleResult`
   */
  SbirsCycleResult StepWithResult(const SbirsCycleInput& input);
  /**
   * @brief 尝试提交运行期配置补丁并写入 Replay。
   * @param[in] patch 运行期配置补丁
   * @return patch 有效且产生更新返回 true；patch 无效或无可更新项返回 false
   */
  bool TryApplyRuntimeConfig(const config::SbirsRuntimeConfigPatch& patch);

  /** @brief 访问被包装的底层会话（可写）。 */
  SbirsSession& session();
  /** @brief 访问被包装的底层会话（只读）。 */
  const SbirsSession& session() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_RECORDING_SESSION_H_
