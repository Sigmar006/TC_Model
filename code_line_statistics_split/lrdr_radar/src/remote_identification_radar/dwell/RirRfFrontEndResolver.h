/**
 * @file RirRfFrontEndResolver.h
 * @brief 定义 RIR RF v2 宽带接收前端的纯求解边界。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_RF_FRONT_END_RESOLVER_H_
#define REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_RF_FRONT_END_RESOLVER_H_

#include <vector>

#include "1q/electromagnetics/RfScene.h"

namespace remote_identification_radar {
namespace dwell {

/** @brief 一个接收窗口内冻结的宽带前端结果。 */
struct RirRfFrontEndResult {
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> incident_links{};
  double total_incident_power_w{0.0};
  bool receiver_saturated{false};
};

/**
 * @brief 求解 scene 中全部 emission 到接收设备输入端的单程链路并聚合功率。
 * @return 成功返回 true；非法 scene、缺失 co-site 路径或非法接收边界返回 false。
 */
bool TryResolveRirRfFrontEnd(const oneq::electromagnetics::RfSceneFrame& scene,
                             const oneq::electromagnetics::RfSceneReceiverState& receiver,
                             double maximum_linear_input_power_w,
                             const oneq::electromagnetics::RfIncidentLinkConfig& link_config,
                             RirRfFrontEndResult* result);

}  // namespace dwell
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_RF_FRONT_END_RESOLVER_H_
