/**
 * @file SbirsPointingCoordinator.h
 * @brief Internal single-telescope NFOV pointing runtime coordinator.
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_POINTING_COORDINATOR_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_POINTING_COORDINATOR_H_

#include <cstdint>
#include <map>

#include "sbirs_sensor/pipeline/SbirsPointingActuator.h"
#include "sbirs_sensor/pipeline/SbirsPointingDisturbance.h"

namespace sbirs_sensor {
namespace pipeline {

/**
 * @brief 单镜筒指向运行态快照（逐目标簿记 + 共享执行器 + 单通道扰动）。
 * @note 捕获等待与跟踪门失败计数按目标记录；镜筒视线只有一份。
 */
struct SbirsPointingCoordinatorSnapshot {
  bool actuator_initialized{false};
  SbirsPointingActuatorSnapshot actuator{};
  std::map<std::uint64_t, double> acquisition_wait_sec{};
  std::map<std::uint64_t, unsigned int> tracking_gate_failure_counts{};
  SbirsPointingDisturbanceSnapshot disturbance{};
};

enum class SbirsPointingAdvanceStatus { kRejected = 0, kSlewing, kSettled, kTimedOut };

struct SbirsPointingAdvanceResult {
  SbirsPointingAdvanceStatus status{SbirsPointingAdvanceStatus::kRejected};
  session::SbirsVector3M current_los{};
  double remaining_angle_deg{0.0};
  double elapsed_wait_sec{0.0};
  double settled_duration_sec{0.0};
};

/**
 * @brief 单镜筒 NFOV 指向运行时协调器（2026-09-02，单镜筒分时轮转）。
 * @details 窄场只有一个镜筒：整星共享一个限速执行器（视线唯一），由 pipeline 的
 *          轮转仲裁决定本周期服务于哪个目标；本类只保管镜筒与逐目标簿记——
 *          捕获等待时长（超时判定）与跟踪门失败计数（丢锁判定）。
 *          扰动按单通道采样（通道 0）。
 */
class SbirsPointingCoordinator {
 public:
  explicit SbirsPointingCoordinator(std::uint32_t disturbance_seed = 1U);

  /** @brief 镜筒未初始化时用首次指向初始化（已初始化时幂等返回 true）。 */
  bool EnsureActuatorInitialized(const session::SbirsVector3M& initial_los);

  /**
   * @brief 捕获语义推进共享镜筒：累计目标捕获等待时长并在 180°/转速 超时。
   * @note 超时返回 kTimedOut 并清除该目标的等待簿记；镜筒视线保持（不回退）。
   */
  SbirsPointingAdvanceResult AdvanceAcquisition(std::uint64_t target_id,
                                                const session::SbirsVector3M& command_los,
                                                double dt_sec,
                                                const SbirsPointingActuatorConfig& config);

  /** @brief 跟踪语义推进共享镜筒（无等待累计；轮转窗口内跟随滤波命令）。 */
  SbirsPointingAdvanceResult AdvanceTracking(const session::SbirsVector3M& command_los,
                                             double dt_sec,
                                             const SbirsPointingActuatorConfig& config);

  /** @brief 捕获成功转跟踪：清除该目标的捕获等待簿记。 */
  bool PromoteToTracking(std::uint64_t target_id);

  unsigned int RecordTrackingGateResult(std::uint64_t target_id, bool gate_passed);
  bool AdvanceDisturbance(double dt_sec, const SbirsPointingDisturbanceParameters& parameters);
  bool DisturbanceSample(int channel_id, const SbirsPointingDisturbanceParameters& parameters,
                         SbirsPointingDisturbanceSample* sample) const;
  /** @brief 指向扰动种子变更（runtime patch）：仅重启扰动流，镜筒与簿记保持。 */
  void RestartDisturbance(std::uint32_t disturbance_seed);
  bool ReleaseTarget(std::uint64_t target_id);
  void ResetTrackingGateFailureCounts();
  void Clear();

  SbirsPointingCoordinatorSnapshot Capture() const;
  bool Restore(const SbirsPointingCoordinatorSnapshot& snapshot);

 private:
  SbirsPointingActuator actuator_{};
  bool actuator_initialized_{false};
  std::map<std::uint64_t, double> acquisition_wait_sec_{};
  std::map<std::uint64_t, unsigned int> tracking_gate_failure_counts_{};
  SbirsPointingDisturbance disturbance_;
};

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_POINTING_COORDINATOR_H_
