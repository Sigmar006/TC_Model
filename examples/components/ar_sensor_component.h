/**
 * @file ar_sensor_component.h
 * @brief 自定义实体-组件示例：AR（机载雷达）传感器组件。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_AR_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_AR_SENSOR_COMPONENT_H_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/ArCycleOutputAdapter.h"
#include "1q/airborne_radar/session/ArExclusionCauseRecorder.h"
#include "1q/airborne_radar/session/ArOutputTypes.h"
#include "1q/airborne_radar/session/ArTrackLifecycleRecorder.h"
#include "1q/airborne_radar/session/ArTrackOutputDebugView.h"
#include "1q/coordinate/types.h"
#include "1q/fusion/SensorAdapters.h"
#include "core/component.h"
#include "core/detection_delivery.h"
#include "logger/logger_modes.h"

namespace component_attachment {

class FlightComponent;  // components/flight_component.h（头文件仅引用，实现文件含入）
struct AppSceneState;  // core/scene_types.h（同上）

/**
 * @brief AR 传感器组件：雷达会话驱动 + 轨迹生命周期事件转发。
 */
class ArSensorComponent : public Component {
 public:
  ArSensorComponent(airborne_radar::session::ArSession session, std::uint64_t platform_entity_id,
                    std::uint64_t transmitter_equipment_id,
                    std::uint32_t device_id = fusion::kArSourceId,
                    DetectionDeliveryMode detection_delivery = DetectionDeliveryMode::kSharedBlackboard);
  ~ArSensorComponent() override = default;

  ArSensorComponent(const ArSensorComponent&) = delete;
  ArSensorComponent& operator=(const ArSensorComponent&) = delete;

  const char* Name() const override { return "ArSensor"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 当前电源状态（由 sensor_enabled 补丁唯一维护；未步进前默认 true，关机时组件不驱动会话）。 */
  bool powered_on() const { return powered_on_; }

  /** @brief 当前传感器设备实体 ID（融合源通道 ID）。 */
  std::uint32_t device_id() const { return device_id_; }

  /**
   * @brief 最近周期调试视图快照（关机周期清零，拒绝周期为 kCycleNotCompleted）。
   */
  const airborne_radar::session::ArTrackOutputDebugView& LastDebugView() const {
    return last_debug_view_;
  }

  /**
   * @brief 最近成功周期的航迹归属对照表（association_key → external_target_id）。
   *
   * 指令路由器经此把融合键（AR 通道 = 内部 association_key）翻译为外部目标
   * ID，再下发 STT 指定/识别指定补丁；未步进或周期被拒时为上一成功周期值。
   */
  const std::vector<airborne_radar::session::ArTrackAttributionRecord>& last_track_attributions()
      const {
    return last_track_attributions_;
  }

  /**
   * @brief 运行时修改入口：包装 ArSession::TryApplyRuntimeConfig。
   *
   * AR 为事务性提交：补丁先暂存，下次成功周期边界统一生效（提交失败
   * 由库内快照完整回滚）；与现有配置冲突的非法补丁在入口即原子拒绝。
   * @param[in] patch 运行期可变参数补丁（has_* 位标志选择字段）。
   * @return true 已接受并暂存；false 补丁非法（原子拒绝，现有配置不变）。
   */
  bool TryApplyRuntimeConfig(const airborne_radar::config::ArRuntimeConfigPatch& patch);

 private:
  // lifecycle_/exclusion_ 声明在 session_ 之前：析构逆序时 session_ 先析构（其析构不
  // 触达 recorder 指针），满足"recorder 生命周期长于 Session 注册期"约束。
  airborne_radar::session::ArTrackLifecycleRecorder lifecycle_{}; /**< 轨迹生命周期事件源 */
  airborne_radar::session::ArExclusionCauseRecorder exclusion_{}; /**< 排除原因跨周期差分事件源 */
  airborne_radar::session::ArSession session_;
  Entity* host_{nullptr};

  std::uint64_t platform_entity_id_{1U};         /**< RF platform_id（rf_world 派生干扰时排除本机发射链） */
  std::uint64_t transmitter_equipment_id_{1U};   /**< 本机发射 equipment_id（与 hardware.transmitter 一致） */

  airborne_radar::session::ArTrackOutputDebugView last_debug_view_{};  /**< 本周期调试视图快照（视图行数据源） */
  std::vector<airborne_radar::session::ArTrackAttributionRecord> last_track_attributions_{}; /**< 最近成功周期航迹归属表（指令路由器键翻译） */

  std::uint64_t prev_designated_target_id_{0U}; /**< 上一周期指定目标 ID（锁定生效/终态沿事件判定；0 = 无指定） */
#if defined(CA_VIEW_LOG_MODE_DELTA)
  /// 视图模式二：上一周期逐目标调试状态（首次出现视为变化；表只增不减，示例不清理）。
  std::unordered_map<std::uint64_t, airborne_radar::session::ArDebugTrackStatus>
      prev_track_status_{};
#endif

  bool powered_on_{true}; /**< 电源状态（由 sensor_enabled 补丁唯一维护；关机时不驱动会话） */
  std::uint32_t device_id_{fusion::kArSourceId}; /**< 传感器设备实体 ID（融合源通道 ID） */
  DetectionDeliveryMode detection_delivery_{DetectionDeliveryMode::kSharedBlackboard};

  /// 调试视图中文人读行写入（三模式分支见 .cpp；宏选择见 logger/logger_modes.h）。
  void LogDebugView(const airborne_radar::session::ArTrackOutputDebugView& view,
                    const oneq::coordinate::LlaPositionDegM* origin_lla);
  /// 周期输入组装：平台运动学（Flight）+ ENU 目标 + RF 干扰投影（供 StepWithResult）。
  airborne_radar::session::ArCycleInput BuildCycleInput(const FlightComponent& flight,
                                                        const AppSceneState& scene,
                                                        double dt_sec) const;
  /// STT 指定任务沿事件（锁定生效沿/终态归零沿，见 .cpp 注释）。
  void PublishDesignationEvent(World& world,
                               const airborne_radar::session::ArCycleResult& result);
  /// 轨迹生命周期事件转发（首确认/失跟 → World 信号 + 事件日志；kUpdated 不转发）。
  void PublishTrackLifecycleEvents(
      World& world, const AppSceneState& scene,
      const airborne_radar::session::ArExternalTrackOutputFrame& external_frame);
  /// 排除原因跨周期差分事件（纯诊断观测，仅落事件日志）。
  void PublishExclusionEvents(World& world);
  /// AR 航迹逐周期状态事件（速度/RCS/位置展平 → on_ar_track_state；威胁评估订阅）。
  void PublishTrackStateEvents(World& world,
                               const airborne_radar::session::ArTrackOutputDebugView& view);
  /// 已发布轨迹 → 泛型探测记录写共享探测池或 on_detection_batch_submitted（源通道 kArSourceId；失跟轨迹不入融合）。
  void AdaptDetections(World& world, AppSceneState& scene,
                       const airborne_radar::session::ArExternalTrackOutputFrame& external_frame);
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_AR_SENSOR_COMPONENT_H_
