/**
 * @file EsrRfV2FrontEnd.h
 * @brief 定义 ESR RF v2 接收前端的单次入射链路求解。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_ESR_RF_V2_FRONT_END_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_ESR_RF_V2_FRONT_END_H_

#include <vector>

#include "1q/electromagnetics/RfScene.h"
#include "1q/electronic_surveillance_radar/config/EsrHardwareConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrOrientationConfig.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"

namespace electronic_surveillance_radar {
namespace pipeline {

/** @brief RF v2 ESR 前端在一个冻结接收窗口的求解结果。 */
struct EsrRfV2FrontEndResult {
  // The physical preselector is intentionally wider than the tuned channel.
  // Every incident signal in this band contributes to front-end blocking and
  // saturation; only channel links are eligible for intercept processing.
  oneq::electromagnetics::RfSceneReceiverState front_end_receiver{};
  oneq::electromagnetics::RfSceneReceiverState channel_receiver{};
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> front_end_incident_links{};
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> channel_incident_links{};
  double total_incident_power_w{0.0};
  bool receiver_saturated{false};
};

/**
 * @brief 构建冻结接收机状态并对帧中每条实际发射求解一次入射链路。
 * @note 该层只计算接收设备输入端，不计算热噪声、检测概率或观测值。
 */
bool TryResolveEsrRfV2FrontEnd(const session::EsrCycleInput& input,
                               const config::EsrHardwareConfig& hardware,
                               const config::EsrOrientationConfig& orientation,
                               double beam_az_deg, double beam_el_deg,
                               double receiver_center_frequency_hz,
                               double receiver_bandwidth_hz,
                               double additional_propagation_loss_db,
                               std::uint32_t sensor_entity_id,
                               EsrRfV2FrontEndResult* result);

}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_ESR_RF_V2_FRONT_END_H_
