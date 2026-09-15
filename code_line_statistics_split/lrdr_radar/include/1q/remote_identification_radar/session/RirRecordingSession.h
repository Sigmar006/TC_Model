/**
 * @file RirRecordingSession.h
 * @brief 远程识别雷达 Replay 记录包装会话。
 *
 * 包装 `RirSession`，拦截 Step / runtime patch 并写入 `ReplayTraceWriter`。
 * 未配置 writer 时只透传底层会话（零开销旁路）。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_RECORDING_SESSION_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_RECORDING_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "1q/replay/ReplayTrace.h"

namespace remote_identification_radar {
namespace config {
struct RirRuntimeConfigPatch;
}  // namespace config
}  // namespace remote_identification_radar

namespace remote_identification_radar {
namespace session {

/**
 * @brief RirRecordingSessionOptions 描述 Replay 记录包装器配置。
 * @note `replay_writer` 产出可被 `ReplayRirTrace()` 消费的 trace 目录。
 */
struct ONEQ_API RirRecordingSessionOptions {
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool record_config_on_construct{true}; /**< 构造时是否记录会话配置 */

  RirRecordingSessionOptions() = default;
};

/**
 * @brief RirRecordingSession 作为 RirSession 的独立中间层记录包装器。
 */
class ONEQ_API RirRecordingSession {
 public:
  /**
   * @brief 用会话配置与记录选项构造包装会话。
   * @param[in] config 五域会话配置，透传给内部 RirSession。
   * @param[in] options 记录器配置（replay_writer / 是否记录配置）。
   */
  explicit RirRecordingSession(const config::RirSessionConfig& config = {},
                               RirRecordingSessionOptions options = {});

  RirRecordingSession(RirRecordingSession&& other) noexcept;
  RirRecordingSession& operator=(RirRecordingSession&& other) noexcept;

  RirRecordingSession(const RirRecordingSession&) = delete;
  RirRecordingSession& operator=(const RirRecordingSession&) = delete;

  ~RirRecordingSession();

  /**
   * @brief 记录并执行一个识别处理周期，返回本周期识别输出帧。
   * @param[in] input 当前周期输入。
   */
  RirOutputFrame Step(const RirCycleInput& input);

  /**
   * @brief 记录并执行一个识别处理周期，返回结构化聚合结果。
   * @param[in] input 当前周期输入。
   * @note 周期输入先写 `cycle_input`，执行后写 `cycle_output`（结果 + 会话状态）。
   */
  RirCycleResult StepWithResult(const RirCycleInput& input);

  /**
   * @brief 尝试应用运行期配置补丁；补丁内容在 apply 之后写入 Replay 记录。
   * @param[in] patch 运行期可变配置补丁。
   * @return 补丁被接受并暂存成功时返回 true；补丁无效或无变更时返回 false。
   */
  bool TryApplyRuntimeConfig(const config::RirRuntimeConfigPatch& patch);

  /** @brief 获取被包装的底层 RirSession（只读）。 */
  const RirSession& session() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_RECORDING_SESSION_H_
