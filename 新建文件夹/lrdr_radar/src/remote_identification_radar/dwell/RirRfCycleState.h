/*
 * Supplemental architecture notes for acceptance review.
 * Unit: RirRfCycleState.
 * Model: LRDR remote identification radar.
 * Layer: dwell.
 * File role: interface declarations and data contracts.
 * Responsibility: per-dwell emission, receiver, beam, echo, and detection processing.
 * Upstream boundary: dwell plan, RF scene, target geometry, and radar hardware state.
 * Downstream boundary: resolved RF cells, detection decisions, and track measurements.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

/**
 * @file RirRfCycleState.h
 * @brief 定义 RIR 单周期 RF 构建输入与冻结接收工作状态。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_RF_CYCLE_STATE_H_
#define REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_RF_CYCLE_STATE_H_

#include <cstdint>

#include "1q/coordinate/types.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/remote_identification_radar/config/RirHardwareConfig.h"

namespace remote_identification_radar {
namespace dwell {

/** @brief RF 发射/接收构造所需的周期输入（无 ECCM 控制档）。 */
struct RirRfCycleInput {
  std::uint32_t sensor_entity_id{0U};                         /**< 传感器实体身份；非零，缺省 5。 */
  oneq::coordinate::EcefPositionM platform_position_ecef_m{};   /**< 平台 ECEF 位置。 */
  oneq::coordinate::EcefVelocityMps platform_velocity_ecef_mps{}; /**< 平台 ECEF 速度。 */
  oneq::coordinate::EulerAnglesDeg radar_frame_attitude_deg{};    /**< 雷达局部相对 ENU 姿态。 */
  config::RirAzimuthElevationDeg beam_pointing_deg{};             /**< 驻留波束中心（雷达局部）。 */
  double window_start_time_s{0.0};                              /**< 接收窗口起始时间（s）。 */
  double window_duration_s{0.0};                                /**< 接收窗口持续时间（s）。 */
};

/** @brief 冻结的接收工作状态，供 RF 前端与 detection cell 只读消费。 */
struct RirReceiverOperatingState {
  oneq::electromagnetics::RfSceneReceiverState rf_receiver{}; /**< 公共单程链路输入状态。 */
  config::RirAzimuthElevationDeg beam_pointing_deg{};             /**< 雷达局部实际波束中心。 */
  double matched_filter_bandwidth_hz{0.0};                     /**< 匹配滤波带宽。 */
  double receiver_noise_figure_db{0.0};                        /**< 接收机噪声系数。 */
  double maximum_linear_input_power_w{0.0};                    /**< 前端最大线性输入功率。 */
};

}  // namespace dwell
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_RF_CYCLE_STATE_H_
