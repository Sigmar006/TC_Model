/**
 * @file ecm_sensor_component.h
 * @brief 自定义实体-组件示例：ECM（电子对抗）组件。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_ECM_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_ECM_SENSOR_COMPONENT_H_

#include <cstdint>

#include "1q/electronic_countermeasure/EcmSession.h"
#include "core/component.h"
#include "core/signals.h"

namespace component_attachment {

/// 演示层 ECM 默认传感器实体 ID。
// C++14：namespace 作用域 constexpr 自带内部链接（inline 变量为 C++17 特性）。
constexpr std::uint32_t kEcmDefaultDeviceId = 101U;

/**
 * @brief ECM 组件：ESR 驱动干扰调度 + RF-WORLD 发布。
 */
class EcmSensorComponent : public Component {
 public:
  explicit EcmSensorComponent(electronic_countermeasure::session::EcmSession session,
                              std::uint32_t device_id = kEcmDefaultDeviceId);
  ~EcmSensorComponent() override = default;

  EcmSensorComponent(const EcmSensorComponent&) = delete;
  EcmSensorComponent& operator=(const EcmSensorComponent&) = delete;

  const char* Name() const override { return "EcmSensor"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 当前电源状态（关机时组件不驱动会话）。 */
  bool powered_on() const { return powered_on_; }

  /** @brief 当前传感器设备实体 ID。 */
  std::uint32_t device_id() const { return device_id_; }

  /** @brief 累计成功发射周期数（冒烟断言）。 */
  std::uint32_t executed_cycle_count() const { return executed_cycle_count_; }

 private:
  electronic_countermeasure::session::EcmSession session_;
  Entity* host_{nullptr};

  bool powered_on_{true};  /**< 电源状态（关机时组件不驱动会话） */
  std::uint32_t device_id_{kEcmDefaultDeviceId};
  std::uint32_t executed_cycle_count_{0U};  /**< 累计成功发射周期数（冒烟断言） */
  std::uint64_t last_submitted_esr_batch_id_{0U};  /**< 已提交给会话的最新 ESR 批次（防重复消费） */
  EsrScanUpdatedEvent esr_scan_{};  /**< 最近一次 ESR 假设集事件缓存（batch_id 为 0 = 尚无成功扫描） */
  boost::signals2::scoped_connection esr_connection_{}; /**< ESR 假设集信号订阅（首次 Step 惰性连接） */

  void OnEsrScanUpdated(const EsrScanUpdatedEvent& event);
  void EnsureSignalConnections(World& world);
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_ECM_SENSOR_COMPONENT_H_
