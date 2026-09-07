/**
 * @file ArRfCycleState.h
 * @brief 定义 AR 单周期门面内部的 RF 准备与接收状态。
 */

#ifndef AIRBORNE_RADAR_SESSION_AR_RF_CYCLE_STATE_H_
#define AIRBORNE_RADAR_SESSION_AR_RF_CYCLE_STATE_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/airborne_radar/session/ArInterferenceObservation.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/airborne_radar/session/ArTrackOutput.h"
#include "1q/airborne_radar/session/DecisionControlTypes.h"
#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/electromagnetics/RfScene.h"

namespace airborne_radar {
namespace session {

/** @brief Prepare 阶段的不透明周期令牌。 */
struct ArPreparedCycleToken {
  std::uint64_t value{0U};             /**< 会话内单调令牌值；0 表示无效。 */
  std::uint64_t world_cycle_index{0U}; /**< 令牌所属世界周期。 */
};

/** @brief Prepare 请求；world time 由外部 orchestrator 唯一拥有。 */
struct ArPrepareCycleInput {
  std::uint64_t world_cycle_index{0U};                        /**< 世界周期号。 */
  double window_start_time_s{0.0};                            /**< 接收窗口绝对起始时间（s）。 */
  double window_duration_s{0.0};                              /**< 接收窗口持续时间（s）。 */
  std::uint32_t sensor_entity_id{0U};                         /**< RF sensor_entity_id 身份；必须非零。 */
  oneq::coordinate::EcefPositionM platform_position_ecef_m{}; /**< 平台 ECEF 位置。 */
  oneq::coordinate::EcefVelocityMps platform_velocity_ecef_mps{};    /**< 平台 ECEF 速度。 */
  oneq::coordinate::EulerAnglesDeg
      radar_frame_attitude_deg{}; /**< 雷达局部框架相对 ENU 的姿态。 */
  config::AzimuthElevationDeg beam_pointing_deg{}; /**< 雷达局部框架中的实际波束中心。 */
};

/** @brief Prepare 冻结并由同一 token 的 Complete 原样使用的接收工作状态。 */
struct ArReceiverOperatingState {
  oneq::electromagnetics::RfSceneReceiverState rf_receiver{}; /**< 公共单程链路输入状态。 */
  config::AzimuthElevationDeg beam_pointing_deg{};             /**< 雷达局部实际波束中心。 */
  double matched_filter_bandwidth_hz{0.0};                     /**< 匹配滤波带宽。 */
  double receiver_noise_figure_db{0.0};                        /**< 接收机噪声系数。 */
  double maximum_linear_input_power_w{0.0};                    /**< 前端最大线性输入功率。 */
  std::vector<oneq::electromagnetics::RfSceneDirection>
      adaptive_nulls_ecef{}; /**< 当前周期实际自适应零陷方向。 */
};

/** @brief Prepare 阶段状态。 */
enum class ArPrepareCycleStatus : std::uint8_t {
  kPrepared = 0, /**< 已发布实际发射并冻结接收状态。 */
  kPoweredOff,   /**< 世界时间已推进，但未消费发射身份或随机状态。 */
  kBusy,         /**< 已存在一个待完成令牌。 */
  kRejected      /**< 输入或运行期配置非法，未改变周期状态。 */
};

/** @brief Prepare 阶段结果。 */
struct ArPrepareCycleResult {
  ArPrepareCycleStatus status{ArPrepareCycleStatus::kRejected}; /**< Prepare 状态。 */
  ArPreparedCycleToken token{};                                 /**< 成功时的待完成令牌。 */
  bool has_emission{false};                                     /**< 是否发布了实际发射。 */
  oneq::electromagnetics::RfSceneEmission emission{};           /**< 实际 AR 发射事实。 */
  ArReceiverOperatingState operating_state{}; /**< Complete 使用的冻结接收工作状态。 */
};

/** @brief Complete 阶段场景与目标输入。 */
struct ArCompleteCycleInput {
  oneq::electromagnetics::RfSceneFrame rf_scene{}; /**< orchestrator 冻结的 RF v2 场景。 */
  ArSceneTargetList targets{};                     /**< 本周期目标事实。 */
};

/** @brief Complete 阶段状态。 */
enum class ArCompleteCycleStatus : std::uint8_t {
  kCompleted = 0, /**< 本周期已完成物理执行并产生新输出。 */
  kRejected,      /**< 输入或执行被拒绝；令牌保留，可重试。 */
  kTokenMismatch  /**< 令牌不属于当前待完成周期。 */
};

/** @brief Complete 阶段结果；仅 kCompleted 携带本周期输出。 */
struct ArCompleteCycleResult {
  ArCompleteCycleStatus status{ArCompleteCycleStatus::kRejected}; /**< Complete 状态。 */
  std::uint64_t world_cycle_index{0U};                            /**< 结果所属世界周期。 */
  session::SignalCycleAbortReason abort_reason{
      session::SignalCycleAbortReason::kNone}; /**< 执行失败时的信号流水线中止原因（透传）。 */
  TrackOutputFrame output_frame{};                          /**< 本周期新轨迹帧。 */
  ArInterferenceObservationList interference_observations{}; /**< 仅通过 J/N 门的本机 RF 观测。 */
  ArReceiverImpairment receiver_impairment{ArReceiverImpairment::kNone}; /**< 结构化接收机损伤。 */
  session::ArIssueList issues{}; /**< 正常执行周期按目标排除的 kInfo 诊断（规则 13b）；
                                      执行失败时承载执行侧校验/中止明细（COMMON-OQ-9）。 */
};

/** @brief Abandon 阶段状态。 */
enum class ArAbandonCycleStatus : std::uint8_t {
  kAbandoned = 0, /**< 已释放待完成接收阶段；发射状态不回滚。 */
  kTokenMismatch  /**< 无待完成周期或令牌不匹配。 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_AR_RF_CYCLE_STATE_H_
