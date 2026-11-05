#include "remote_identification_radar/dwell/RirRfFrontEndResolver.h"

#include <algorithm>
#include <cmath>

/**
 * ## 过程：RF 前端入射链与饱和判定（LRDR 探测链 · 第 3 步）
 * - 上游：RfSceneFrame（外部干扰源发射 + 本机自发射）、RfSceneReceiverState（接收机几何/
 *   天线/极化/接收窗/预选带宽/接收损耗）、receiver.maximum_linear_input_power_w。
 * - 本步：对每条发射做一次链路求解（天线方向图取样 → 单程发/收增益 → 收端入射功率），
 *   按 (sensor_entity_id, emission_id) 归一排序后聚合成总入射功率；总功率超过线性输入
 *   上限即判接收前端饱和（本周期致盲）。
 * - 下游：RirController 的 incident_links（检测单元 SINR 分母的干扰项、验收
 *   interference_link 事件）与 receiver_saturated（链上第一道致盲门）。
 * - 关键参数：hardware.receiver.maximum_linear_input_power_w（默认 1e-3 W）、
 *   hardware.receiver.receive_loss_db、hardware.antenna.*（方向图取样输入）、
 *   接收窗时长 = mission.recognition_dwell_sec。
 * - 对应验收项：第 30 项干扰功率计算、第 35 项干扰信号处理增益（分母来源）、
 *   第 63 项外部干扰输入雷达接口。
 */

namespace remote_identification_radar {
namespace dwell {
namespace {

// 过程：排序键——(sensor_entity_id, emission_id) 字典序，使入射链顺序确定（聚合与验收日志可复现）。
bool IdentityLess(const oneq::electromagnetics::RfIncidentLinkResult& left,
                  const oneq::electromagnetics::RfIncidentLinkResult& right) {
  if (left.identity.sensor_entity_id != right.identity.sensor_entity_id) {
    return left.identity.sensor_entity_id < right.identity.sensor_entity_id;
  }
  return left.identity.emission_id < right.identity.emission_id;
}

}  // namespace

// 过程：前端求解入口——场景/接收窗/线性上限的合法性门 → 逐发射求入射链 → 聚合 → 饱和判定。
bool TryResolveRirRfFrontEnd(const oneq::electromagnetics::RfSceneFrame& scene,
                             const oneq::electromagnetics::RfSceneReceiverState& receiver,
                             double maximum_linear_input_power_w,
                             const oneq::electromagnetics::RfIncidentLinkConfig& link_config,
                             RirRfFrontEndResult* result) {
  if (result == nullptr || !std::isfinite(maximum_linear_input_power_w) ||
      maximum_linear_input_power_w <= 0.0 ||
      !oneq::electromagnetics::TryValidateRfSceneFrame(scene) ||
      scene.window_start_time_s != receiver.window_start_time_s ||
      scene.window_duration_s != receiver.window_duration_s) {
    return false;
  }

  // 过程：逐发射求解入射链（含发射与接收方向图取样、单程/双程增益与收端入射功率），任一失败即整体失败。
  RirRfFrontEndResult candidate;
  candidate.incident_links.reserve(scene.emissions.size());
  for (const auto& emission : scene.emissions) {
    oneq::electromagnetics::RfIncidentLinkResult link;
    if (!oneq::electromagnetics::TryEvaluateRfIncidentLink(emission, receiver, link_config,
                                                           &link)) {
      return false;
    }
    candidate.incident_links.push_back(link);
  }
  // 过程：入射链归一排序（与 RF 场景消费方同序，保证聚合顺序与验收事件稳定）。
  std::sort(candidate.incident_links.begin(), candidate.incident_links.end(), IdentityLess);
  // 过程：线性功率求和得总入射功率（同平台自发射耦合链亦计入，故饱和含自干扰）。
  if (!oneq::electromagnetics::TryAggregateRfIncidentPower(candidate.incident_links,
                                                           &candidate.total_incident_power_w)) {
    return false;
  }
  // 过程：饱和判据——总入射功率严格大于线性上限即判前端失真/致盲（等号不判饱和）。
  candidate.receiver_saturated = candidate.total_incident_power_w > maximum_linear_input_power_w;
  *result = candidate;
  return true;
}

}  // namespace dwell
}  // namespace remote_identification_radar
