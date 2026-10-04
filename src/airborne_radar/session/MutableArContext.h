/**
 * @file MutableArContext.h
 * @brief 定义面向外部接入的可变 AR 上下文默认实现。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTEXT_MUTABLE_AR_CONTEXT_H_
#define AIRBORNE_RADAR_CORE_CONTEXT_MUTABLE_AR_CONTEXT_H_

#include <memory>
#include <utility>
#include <vector>

#include "1q/airborne_radar/session/ArCommand.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"

namespace airborne_radar {
namespace session {

struct ArContextRuntimeSnapshot;
struct ArContextRuntimeIdentity;
class MutableArContext;

/**
 * @brief MutableArContext 运行态快照，用于失败回滚等场景的整快照捕获/恢复。
 * @note envelope 仅能由 MutableArContext 构造，调用方可整体复制/移动，
 *       但无法重组 owner 与 typed snapshot 的绑定关系。
 */
class ArContextRuntimeState final {
 public:
  ArContextRuntimeState(const ArContextRuntimeState&) = default;
  ArContextRuntimeState& operator=(const ArContextRuntimeState&) = default;
  // VS2013 不支持 = default 的移动操作（C2610），故显式实现成员级移动。
  ArContextRuntimeState(ArContextRuntimeState&& other) noexcept
      : owner_identity_(std::move(other.owner_identity_)), snapshot_(std::move(other.snapshot_)) {}
  ArContextRuntimeState& operator=(ArContextRuntimeState&& other) noexcept {
    owner_identity_ = std::move(other.owner_identity_);
    snapshot_ = std::move(other.snapshot_);
    return *this;
  }

 private:
  ArContextRuntimeState(std::shared_ptr<const ArContextRuntimeIdentity> owner_identity,
                        std::shared_ptr<const ArContextRuntimeSnapshot> snapshot)
      : owner_identity_(std::move(owner_identity)), snapshot_(std::move(snapshot)) {}

  std::shared_ptr<const ArContextRuntimeIdentity> owner_identity_{};
  std::shared_ptr<const ArContextRuntimeSnapshot> snapshot_{};

  friend class MutableArContext;
};

/**
 * @brief 提供一个可直接驱动控制器的默认 AR 上下文实现。
 */
class MutableArContext final {
 public:
  /**
   * @brief 默认构造函数。
   */
  MutableArContext();
  /**
   * @brief 使用初始场景目标列表构造上下文。
   * @param[in] scene_targets 初始场景目标列表。
   */
  explicit MutableArContext(ArSceneTargetList scene_targets);
  ~MutableArContext() = default;

  MutableArContext(const MutableArContext&) = delete;
  MutableArContext& operator=(const MutableArContext&) = delete;
  MutableArContext(MutableArContext&&) = delete;
  MutableArContext& operator=(MutableArContext&&) = delete;

  /** @brief 以已验证的内部执行事实刷新上下文，并清空本周期输出缓存。 */
  void BeginCycle(ArSceneTargetList scene_targets,
                  float platform_altitude_m, float dt_sec,
                  std::uint32_t cycle_index);

  /**
   * @brief 更新当前周期场景目标列表。
   * @param[in] scene_targets 新的场景目标列表。
   */
  void SetSceneTargets(ArSceneTargetList scene_targets);

  /**
   * @brief 更新当前平台姿态角。
   * @param[in] platform_attitude_deg 平台姿态角，单位为度。
   */
  void SetPlatformAttitude(const config::PlatformAttitudeDeg& platform_attitude_deg);

  /**
   * @brief 更新当前周期时间步长。
   * @param[in] dt_sec 周期步长，单位为秒。
   */
  void SetCycleDeltaTimeSec(float dt_sec);

  /**
   * @brief 更新当前输入周期号。
   * @param[in] cycle_index 当前周期号。
   */
  void SetCycleIndex(std::uint32_t cycle_index);

  /**
   * @brief 清空本周期输出缓存。
   * @note 当前仅清空已提交指令列表，不重置最近一次控制真值。
   */
  void ResetCycleOutputs();

  /**
   * @brief 获取本周期已提交的控制指令。
   * @return 当前周期命令缓存。
   */
  const std::vector<session::ArCommand>& GetSubmittedCommands() const;

  /**
   * @brief GetSubmittedCommands 的别名，返回本周期命令缓存只读引用。
   * @return 当前周期已提交命令缓存。
   */
  const std::vector<session::ArCommand>& SubmittedCommands() const;

  /**
   * @brief 判断是否已经收到过控制真值更新。
   * @return 若至少收到过一次 `UpdateRadarControlProfile` 则返回 `true`。
   */
  bool HasLatestControlProfile() const;

  /**
   * @brief 获取最近一次保存的控制真值。
   * @return 最近一次控制真值；若尚未更新则返回默认值。
   */
  const session::ArControlProfile& GetLatestControlProfile() const;

  /**
   * @brief GetLatestControlProfile 的别名，返回最近一次控制真值只读引用。
   * @return 最近一次控制真值；若尚未更新则返回默认值。
   */
  const session::ArControlProfile& LatestControlProfile() const;

  /**
   * @brief 捕获当前上下文运行态快照。
   * @return 可用于失败回滚的上下文运行态快照。
   */
  ArContextRuntimeState CaptureRuntimeState() const;

  /**
   * @brief 恢复此前捕获的上下文运行态快照。
   * @param[in] state 待恢复的上下文运行态快照。
   * @return owner 与快照载荷均有效时返回 true，否则返回 false 且不修改当前状态。
   */
  bool RestoreRuntimeState(const ArContextRuntimeState& state);

  /**
   * @brief 获取当前周期场景目标列表。
   * @return 当前周期场景目标列表只读引用。
   */
  const ArSceneTargetList& GetSceneTargets() const;

  /**
   * @brief 获取当前平台姿态角。
   * @return 当前平台姿态角。
   */
  config::PlatformAttitudeDeg GetPlatformAttitude() const;

  /**
   * @brief 获取当前平台海拔高度。
   * @return 当前平台海拔高度（单位：m）。
   */
  float GetPlatformAltitudeM() const;

  /**
   * @brief 获取当前周期时间步长。
   * @return 当前周期步长，单位为秒。
   */
  float GetCycleDeltaTimeSec() const;

  /**
   * @brief 获取当前输入周期号。
   * @return 当前周期号。
   */
  std::uint32_t GetCycleIndex() const;

  /**
   * @brief 记录控制器提交的单条控制指令。
   * @param[in] cmd 控制指令。
   */
  void SubmitControlCommand(session::ArCommand cmd);

  /**
   * @brief 保存最近一次控制真值。
   * @param[in] profile 下一周期控制真值。
   */
  void UpdateRadarControlProfile(const session::ArControlProfile& profile);

 private:
  std::shared_ptr<const ArContextRuntimeIdentity> owner_identity_;
  std::shared_ptr<ArSceneTargetList> scene_targets_{new ArSceneTargetList()};
  config::PlatformAttitudeDeg platform_attitude_deg_{};
  float platform_altitude_m_{0.0f};
  float cycle_dt_sec_{1.0f};
  std::uint32_t cycle_index_{0U};
  std::vector<session::ArCommand> submitted_commands_{};
  session::ArControlProfile latest_control_profile_{};
  bool has_latest_control_profile_{false};
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTEXT_MUTABLE_AR_CONTEXT_H_
