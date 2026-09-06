/**
 * @file eos_sensor_component.h
 * @brief 自定义实体-组件示例：EOS（光电）传感器组件。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_EOS_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_EOS_SENSOR_COMPONENT_H_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"
#include "1q/electro_optical_sensor/session/EosDetectionLifecycleRecorder.h"
#include "1q/electro_optical_sensor/session/EosExclusionCauseRecorder.h"
#include "1q/electro_optical_sensor/session/EosOutputDebugView.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/fusion/SensorAdapters.h"
#include "core/component.h"
#include "core/detection_delivery.h"
#include "logger/logger_modes.h"

namespace component_attachment {

class FlightComponent;  // components/flight_component.h（头文件仅引用，实现文件含入）
struct AppSceneState;  // core/scene_types.h（同上）

/**
 * @brief EOS 传感器组件：光电会话驱动 + 探测生命周期事件转发。
 */
class EosSensorComponent : public Component {
 public:
  explicit EosSensorComponent(
      electro_optical_sensor::session::EosSession session,
      std::uint32_t device_id = fusion::kEosSourceId,
      DetectionDeliveryMode detection_delivery = DetectionDeliveryMode::kSharedBlackboard);
  ~EosSensorComponent() override = default;

  EosSensorComponent(const EosSensorComponent&) = delete;
  EosSensorComponent& operator=(const EosSensorComponent&) = delete;

  const char* Name() const override { return "EosSensor"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 当前电源状态（由 sensor_enabled 补丁唯一维护；未步进前默认 true，关机时组件不驱动会话）。 */
  bool powered_on() const { return powered_on_; }

  /** @brief 当前传感器设备实体 ID（融合源通道 ID）。 */
  std::uint32_t device_id() const { return device_id_; }

  /** @brief 最近周期波束中心方位角（单位：deg，平台系；仅开机且 kCompleted 周期有效，其余为 0）。 */
  float scan_azimuth_deg() const { return scan_azimuth_deg_; }

  /**
   * @brief 最近周期调试视图快照（关机周期清零，拒绝周期为 kCycleNotExecuted）。
   */
  const electro_optical_sensor::session::EosOutputDebugView& LastDebugView() const {
    return last_debug_view_;
  }

  /**
   * @brief 运行时修改入口：包装 EosSession::TryApplyRuntimeConfig。
   *
   * EOS 为立即提交：补丁经 resolver 原子校验后一次生效（调用即生效，
   * session 层无回滚）；扫描相位是否重置由 resolver 显式决定。frame_rate_hz
   * 热更新经 resolver 校验有限且 > 0（非法值整补丁原子拒绝）。
   * @param[in] patch 运行期可变参数补丁（has_* 位标志选择字段）。
   * @return true 已应用；false 补丁无效或无变更（现有配置不变）。
   */
  bool TryApplyRuntimeConfig(const electro_optical_sensor::config::EosRuntimeConfigPatch& patch);

 private:
  // lifecycle_/exclusion_ 声明在 session_ 之前：析构逆序时 session_ 先析构（其析构不
  // 触达 recorder 指针），满足"recorder 生命周期长于 Session 注册期"约束。
  electro_optical_sensor::session::EosDetectionLifecycleRecorder lifecycle_{}; /**< 探测生命周期事件源 */
  electro_optical_sensor::session::EosExclusionCauseRecorder exclusion_{}; /**< 排除原因跨周期差分事件源 */
  electro_optical_sensor::session::EosSession session_;
  Entity* host_{nullptr};

  float scan_azimuth_deg_{0.0f}; /**< 最近周期波束中心方位角（deg，随周期结果刷新） */

  electro_optical_sensor::session::EosOutputDebugView last_debug_view_{};  /**< 本周期调试视图快照（视图行数据源） */

#if defined(CA_VIEW_LOG_MODE_DELTA)
  /// 视图模式二：上一周期逐目标调试状态（首次出现视为变化；表只增不减，示例不清理）。
  std::unordered_map<std::uint64_t, electro_optical_sensor::session::EosDebugTargetStatus>
      prev_target_status_{};
#endif

  bool powered_on_{true}; /**< 电源状态（由 sensor_enabled 补丁唯一维护；关机时不驱动会话） */
  std::uint32_t device_id_{fusion::kEosSourceId};
  DetectionDeliveryMode detection_delivery_{DetectionDeliveryMode::kSharedBlackboard};

  /// 调试视图中文人读行写入（三模式分支见 .cpp；宏选择见 logger/logger_modes.h）。
  void LogDebugView(const electro_optical_sensor::session::EosOutputDebugView& view);
  /// 周期输入组装：平台锚点（ECEF→LLA，失败返回 false）+ ENU 场景目标直填。
  bool BuildCycleInput(const FlightComponent& flight, const AppSceneState& scene,
                       double dt_sec,
                       electro_optical_sensor::session::EosCycleInput* input) const;
  /// 探测生命周期事件发布（首发现/更新/丢失 → World 信号 + 事件日志）。
  void PublishDetectionEvents(
      World& world, const AppSceneState& scene,
      const electro_optical_sensor::session::EosCycleResult& result);
  /// 排除原因跨周期差分事件（纯诊断观测，仅落事件日志）。
  void PublishExclusionEvents(World& world);
  /// 探测记录 → 泛型探测记录写共享探测池或 on_detection_batch_submitted（源通道 kEosSourceId，无身份键 0）。
  void AdaptDetections(World& world, AppSceneState& scene,
                       const electro_optical_sensor::session::EosCycleResult& result);
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_EOS_SENSOR_COMPONENT_H_
