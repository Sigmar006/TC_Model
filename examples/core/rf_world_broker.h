/**
 * @file core/rf_world_broker.h
 * @brief 演示层 RF-WORLD 编排：跨实体汇集/分发装备发射（无库内全局 RfScene 总线）。
 *
 * 编排语义（见 docs/common/rf_architecture.md）：
 *  - 每周期初：rf_world = 上周期装备发射 + 脚本辐射源真值；
 *  - ESR → ECM → AR 挂载序：ECM 发布干扰进 rf_world，AR 从 rf_world 派生
 *    interference（排除本机发射 equipment，保留同平台 ECM/外部源）；
 *  - AR/RIR Step 成功后：本周期 emission_frame 追加到 rf_world（同周期下游可见）
 *    并写入 pending_equipment_emissions（下周期初注入）；
 *  - RIR 站点实体先于平台步进：RIR 消费的是上周期 AR 发射 + 本周期脚本源。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_RF_WORLD_BROKER_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_RF_WORLD_BROKER_H_

#include <cstdint>

#include "1q/electromagnetics/RfScene.h"
#include "core/scene_types.h"

namespace component_attachment {

/** @brief 周期初重建 RF-WORLD（上周期装备发射 + 脚本辐射源）。 */
inline void BeginRfWorldCycle(AppSceneState* scene, double window_duration_sec) {
  if (scene == nullptr) {
    return;
  }
  scene->rf_world = {};
  scene->rf_world.world_cycle_index = scene->cycle;
  scene->rf_world.window_start_time_s = scene->t_sec;
  scene->rf_world.window_duration_s = window_duration_sec;
  for (const auto& emission : scene->pending_equipment_emissions.emissions) {
    scene->rf_world.emissions.push_back(emission);
  }
  scene->pending_equipment_emissions = {};
  scene->rf_world.emissions.insert(scene->rf_world.emissions.end(), scene->emitters.begin(),
                                   scene->emitters.end());
}

/** @brief 装备 Step 成功后发布本周期发射（同周期可见 + 下周期 carryover）。 */
inline void PublishEquipmentEmissions(AppSceneState* scene,
                                      const oneq::electromagnetics::RfEmissionFrame& frame) {
  if (scene == nullptr || frame.emissions.empty()) {
    return;
  }
  for (const auto& emission : frame.emissions) {
    scene->rf_world.emissions.push_back(emission);
    scene->pending_equipment_emissions.emissions.push_back(emission);
  }
  scene->pending_equipment_emissions.world_cycle_index = frame.world_cycle_index;
  scene->pending_equipment_emissions.window_start_time_s = frame.window_start_time_s;
  scene->pending_equipment_emissions.window_duration_s = frame.window_duration_s;
}

/** @brief 构建外部 rf_scene（排除本装备 sensor_entity_id；窗口由调用方对齐会话校验）。 */
inline oneq::electromagnetics::RfSceneFrame BuildExternalRfScene(
    const oneq::electromagnetics::RfSceneFrame& rf_world, std::uint32_t exclude_sensor_entity_id,
    double window_start_time_s, double window_duration_sec, std::uint64_t world_cycle_index) {
  oneq::electromagnetics::RfSceneFrame external;
  external.world_cycle_index = world_cycle_index;
  external.window_start_time_s = window_start_time_s;
  external.window_duration_s = window_duration_sec;
  for (const auto& emission : rf_world.emissions) {
    if (emission.identity.sensor_entity_id != exclude_sensor_entity_id) {
      external.emissions.push_back(emission);
    }
  }
  return external;
}

/** @brief 从 RF-WORLD 派生 AR 干扰帧（排除本机发射链，保留 ECM/脚本/它平台发射）。 */
inline oneq::electromagnetics::RfEmissionFrame BuildArInterferenceFromRfWorld(
    const oneq::electromagnetics::RfSceneFrame& rf_world, std::uint32_t own_sensor_entity_id,
    double window_start_time_s, double window_duration_sec, std::uint64_t world_cycle_index) {
  oneq::electromagnetics::RfEmissionFrame interference;
  interference.world_cycle_index = world_cycle_index;
  interference.window_start_time_s = window_start_time_s;
  interference.window_duration_s = window_duration_sec;
  for (const auto& emission : rf_world.emissions) {
    if (emission.identity.sensor_entity_id != own_sensor_entity_id) {
      interference.emissions.push_back(emission);
    }
  }
  return interference;
}

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_RF_WORLD_BROKER_H_
