/*
 * Supplemental architecture notes for acceptance review.
 * Unit: RirSession.
 * Model: LRDR remote identification radar.
 * Layer: session.
 * File role: interface declarations and data contracts.
 * Responsibility: public cycle contract, validation, adapters, recording, and replay.
 * Upstream boundary: external targets, platform position, RF scene, and session settings.
 * Downstream boundary: stable output frames, issues, task feedback, and replay records.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

/**
 * @file RirSession.h
 * @brief 定义面向外部接入的远程识别雷达会话门面。
 *
 * 会话门面（一步一帧执行、运行期补丁）的主头文件。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SESSION_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/foundation/SensorContract.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/config/RirSessionConfigValidation.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirOutputTypes.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"

namespace remote_identification_radar {
namespace config {
struct RirRuntimeConfigPatch;
}  // namespace config
}  // namespace remote_identification_radar

namespace remote_identification_radar {
namespace session {

class RirTrackLifecycleRecorder;
class RirExclusionCauseRecorder;

/**
 * @brief RirSession 提供"一步一帧"的外部接入门面。
 */
class ONEQ_API RirSession {
 public:
  RirSession();
  ~RirSession();

  RirSession(const RirSession&) = delete;
  RirSession& operator=(const RirSession&) = delete;
  RirSession(RirSession&&) noexcept;
  RirSession& operator=(RirSession&&) noexcept;

  /**
   * @brief 执行一个识别处理周期。
   * @param[in] input 当前周期输入。
   * @return 当前周期生成的识别输出帧拷贝。
   * @note 非法或被拒绝的周期返回默认的当前帧，不复用历史输出。
   */
  RirOutputFrame Step(const RirCycleInput& input);

  /**
   * @brief 执行一个识别处理周期，并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   * @note `status` 是结果有效性的唯一真相；只有 `kCompleted` 携带本周期识别输出。
   */
  RirCycleResult StepWithResult(const RirCycleInput& input);

  /**
   * @brief 尝试应用运行期可变配置补丁。
   * @param[in] patch 运行期可变配置补丁。
   * @note 未设置的字段保持现值不变；补丁在下一个成功执行周期边界提交。
   * @return 补丁被接受并暂存成功时返回 true；补丁无效或无变更时返回 false。
   */
  bool TryApplyRuntimeConfig(const config::RirRuntimeConfigPatch& patch);

  /**
   * @brief 注册航迹生命周期记录器（规则 10/11）。
   *
   * Session 在 `StepWithResult()`/`Step()` 内部自动调用 `Update()`，调用方无需
   * （也不应）手动调用；注册与否不影响 `Step()`/`StepWithResult()` 的返回值
   * 和执行语义（纯观测工具，零行为改变）。
   *
   * @param[in] recorder 生命周期记录器；传入 nullptr 解除注册。
   * @note Session 持有非拥有裸指针，调用方须保证 recorder 生命周期长于注册期。
   */
  void AttachTrackLifecycleRecorder(RirTrackLifecycleRecorder* recorder) noexcept;

  /**
   * @brief 注册排除原因差分记录器（规则 10/11/13e）。
   *
   * 与 `AttachTrackLifecycleRecorder` 独立并列：各自注册、各自驱动、各自
   * `GetLastEvents()` 通道，互不影响。非执行周期两记录器均不推进内部状态。
   *
   * @param[in] recorder 排除原因差分记录器；传入 nullptr 解除注册。
   * @note Session 持有非拥有裸指针，调用方须保证 recorder 生命周期长于注册期。
   */
  void AttachExclusionCauseRecorder(RirExclusionCauseRecorder* recorder) noexcept;

  /** @brief 最近周期是否发布了识别效能摘要。 */
  bool HasLatestRecognitionSummary() const;

  /** @brief 获取最近周期识别效能摘要。 */
  const RirRecognitionCycleSummary& GetLatestRecognitionSummary() const;

  /** @brief 最近一次识别库加载耗时（毫秒）；未加载过为 0。 */
  double LastRecognitionDatabaseLoadMs() const;

  /**
   * @brief 注入传感器实体标识（设备 ID / 融合通道）。
   *
   * 用于在日志与验收记录中区分同型多实例传感器，并作为融合通道 source_id。
   * 传 0 恢复缺省（fusion::kRirSourceId = 5U）；下一周期起生效。
   * @param[in] sensor_entity_id 传感器实体标识（非零；0 为恢复缺省哨兵）。
   */
  void SetSensorEntityId(std::uint32_t sensor_entity_id);

  /** @brief 使用四域配置创建会话（推荐入口，信任路径，不做配置校验）。 */
  static RirSession Create(const config::RirSessionConfig& config = {});

  /**
   * @brief 创建会话并报告配置校验结果（校验路径）。
   * @param[in] config 四域会话配置。
   * @param[out] issues 校验问题输出；传入 nullptr 则不写回但仍构造会话。
   * @return 构造完成的会话。
   * @note 无论 @p issues 是否为空都会构造并返回会话（不阻断）。
   */
  static RirSession CreateWithDiagnostics(const config::RirSessionConfig& config,
                                          RirIssueList* issues);

 private:
  friend class RirSessionReplayAccess;

  struct Impl;
  explicit RirSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace remote_identification_radar

// 跨域传感器会话形状契约：锚定 Step/StepWithResult 签名。
ONEQ_SENSOR_SESSION_CONTRACT(remote_identification_radar::session::RirSession,
                             remote_identification_radar::session::RirCycleInput,
                             remote_identification_radar::session::RirOutputFrame,
                             remote_identification_radar::session::RirCycleResult);

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SESSION_H_
