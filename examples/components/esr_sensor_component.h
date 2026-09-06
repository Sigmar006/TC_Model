/**
 * @file esr_sensor_component.h
 * @brief 自定义实体-组件示例：ESR（电子侦察）传感器组件。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_ESR_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_ESR_SENSOR_COMPONENT_H_

#include <vector>

#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/session/EsrExclusionCauseRecorder.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"
#include "1q/fusion/SensorAdapters.h"
#include "core/component.h"
#include "core/detection_delivery.h"

namespace component_attachment {

class FlightComponent;  // components/flight_component.h（头文件仅引用，实现文件含入）
struct AppSceneState;  // core/scene_types.h（同上）

/**
 * @brief ESR 传感器组件：侦察会话驱动 + 假设事件发布。
 */
class EsrSensorComponent : public Component {
 public:
  explicit EsrSensorComponent(
      electronic_surveillance_radar::session::EsrSession session,
      std::uint32_t device_id = fusion::kEsrSourceId,
      DetectionDeliveryMode detection_delivery = DetectionDeliveryMode::kSharedBlackboard);
  ~EsrSensorComponent() override = default;

  EsrSensorComponent(const EsrSensorComponent&) = delete;
  EsrSensorComponent& operator=(const EsrSensorComponent&) = delete;

  const char* Name() const override { return "EsrSensor"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 当前电源状态（由 sensor_enabled 补丁唯一维护；未步进前默认 true，关机时组件不驱动会话）。 */
  bool powered_on() const { return powered_on_; }

  /** @brief 当前传感器设备实体 ID（融合源通道 ID）。 */
  std::uint32_t device_id() const { return device_id_; }

  /** @brief 最近周期波束中心方位角（单位：deg，平台系；仅开机且 kCompleted 周期有效，其余为 0）。 */
  float scan_azimuth_deg() const { return scan_azimuth_deg_; }

  /**
   * @brief 运行时修改入口：包装 EsrSession::TryApplyRuntimeConfig。
   *
   * ESR 为立即提交：调用即生效、单向落定（session 层无回滚），扫描相位
   * 重置语义由库内 resolver 决定。
   * @param[in] patch 运行期可变参数补丁（has_* 位标志选择字段）。
   * @return true 已应用；false 未请求更新或补丁被拒绝（整补丁原子拒绝）。
   */
  bool TryApplyRuntimeConfig(
      const electronic_surveillance_radar::config::EsrRuntimeConfigPatch& patch);

  /**
   * @brief 运行时修改入口（结构化结果）：包装
   *        EsrSession::ApplyRuntimeConfigWithResult。
   *
   * 与 TryApplyRuntimeConfig 同语义，另返回结构化状态码（含拒绝原因枚举），
   * 供外部决策/诊断使用。
   * @param[in] patch 运行期可变参数补丁。
   * @return 结构化应用结果（status 枚举 + applied/has_requested_update 位）。
   */
  electronic_surveillance_radar::session::EsrRuntimeConfigApplyResult ApplyRuntimeConfigWithResult(
      const electronic_surveillance_radar::config::EsrRuntimeConfigPatch& patch);

 private:
  // exclusion_ 声明在 session_ 之前：析构逆序时 session_ 先析构（其析构不触达
  // recorder 指针），满足"recorder 生命周期长于 Session 注册期"约束。
  // ESR 无既有 lifecycle recorder，exclusion_ 为首个 recorder 成员。
  electronic_surveillance_radar::session::EsrExclusionCauseRecorder exclusion_{}; /**< 排除原因跨周期差分事件源 */
  electronic_surveillance_radar::session::EsrSession session_;
  Entity* host_{nullptr};

  float scan_azimuth_deg_{0.0f}; /**< 最近周期波束中心方位角（deg，随周期结果刷新） */

  electronic_surveillance_radar::session::EmitterHypothesisList last_hypotheses_{}; /**< 上一成功周期辐射源假设（假设集事件/探测适配数据源） */
  std::uint64_t last_batch_id_{0U};                 /**< 上一成功周期 batch_id（假设集事件 fresh-frame 判据） */
  std::uint32_t last_completed_cycle_index_{0U};    /**< 上一成功周期世界周期号（0 = 尚无成功输出） */
  bool has_last_completed_output_{false};           /**< 是否至少完成过一个成功周期 */

  bool powered_on_{true}; /**< 电源状态（由 sensor_enabled 补丁唯一维护；关机时不驱动会话） */
  std::uint32_t device_id_{fusion::kEsrSourceId};
  DetectionDeliveryMode detection_delivery_{DetectionDeliveryMode::kSharedBlackboard};

  /// 周期输入组装：平台运动学（Flight）+ RF-WORLD 发射包络（供 StepWithResult）。
  electronic_surveillance_radar::session::EsrCycleInput BuildCycleInput(
      const FlightComponent& flight, const AppSceneState& scene, double dt_sec) const;
  /// 辐射源假设事件逐条发布（World 信号 + 事件日志；库内键 0 不发布）。
  void PublishHypothesisEvents(World& world, const AppSceneState& scene);
  /// 假设集快照事件发布（每成功周期全量展平 → on_esr_scan_updated；ECM 订阅）。
  void PublishScanEvent(World& world, const AppSceneState& scene);
  /// 排除原因跨周期差分事件（纯诊断观测，仅落事件日志）。
  void PublishExclusionEvents(World& world);
  /// 辐射源假设 → 泛型探测记录写共享探测池或 on_detection_batch_submitted（源通道 kEsrSourceId）。
  void AdaptDetections(World& world, AppSceneState& scene);
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_ESR_SENSOR_COMPONENT_H_
