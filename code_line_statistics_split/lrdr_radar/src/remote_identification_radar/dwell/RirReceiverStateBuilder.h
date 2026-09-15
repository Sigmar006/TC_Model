/**
 * @file RirReceiverStateBuilder.h
 * @brief 无状态构造器——从周期输入、发射事实与硬件配置组装接收工作状态。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_RECEIVER_STATE_BUILDER_H_
#define REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_RECEIVER_STATE_BUILDER_H_

#include "1q/electromagnetics/RfScene.h"
#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "remote_identification_radar/dwell/RirRfCycleState.h"

namespace remote_identification_radar {
namespace dwell {

/**
 * @brief 无状态接收状态构造器。
 *
 * 与 AR 版差异：无旁瓣对消/自适应波束形成运行时改方向图。
 */
class RirReceiverStateBuilder {
 public:
  static RirReceiverOperatingState Build(const RirRfCycleInput& input,
                                         const oneq::electromagnetics::RfSceneEmission& emission,
                                         const config::RirHardwareConfig& hardware,
                                         double carrier_hz);
};

}  // namespace dwell
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_RECEIVER_STATE_BUILDER_H_
