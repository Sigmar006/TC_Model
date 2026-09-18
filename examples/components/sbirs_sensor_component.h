/**
 * @file sbirs_sensor_component.h
 * @brief 自定义实体-组件示例：SBIRS（天基红外）传感器组件。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SBIRS_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SBIRS_SENSOR_COMPONENT_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "1q/fusion/SensorAdapters.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"
#include "1q/sbirs_sensor/session/SbirsExclusionCauseRecorder.h"
#include "1q/sbirs_sensor/session/SbirsOutputDebugView.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"
#include "core/component.h"
#include "core/detection_delivery.h"
#include "core/signals.h"
#include "logger/logger_modes.h"

namespace component_attachment {

struct AppSceneState;  // core/scene_types.h（头文件仅引用，实现文件含入）

/** @brief 卫星探测帧投递方式（ground_station_source_id 非 0 时生效）。 */
enum class SbirsGroundDeliveryMode {
  kSharedBlackboard = 0, /**< 写 AppSceneState.sbirs_ground_station_inbox */
  kMessage = 1           /**< 发 on_sbirs_frame_submitted */
};

/**
 * @brief SBIRS 传感器组件：天基红外会话驱动 + 探测生命周期事件转发。
 */
class SbirsSensorComponent : public Component {
 public:
  /**
   * @brief 机载/地面单站：会话由调用方构造后移入（示例：main 用默认或场景 JSON 配置
   *            `SbirsSession::Create(...)`）。
   * @param[in] session 调用方构造的 SBIRS 会话实例。
   * @param[in] device_id 融合源通道 / 传感器设备实体 ID（单站默认 kSbirsSourceId）。
   * @param[in] detection_delivery 探测记录投递方式（示例：消息场景取 kMessage）。
   */
  explicit SbirsSensorComponent(
      sbirs_sensor::session::SbirsSession session,
      std::uint32_t device_id = fusion::kSbirsSourceId,
      DetectionDeliveryMode detection_delivery = DetectionDeliveryMode::kSharedBlackboard);
  /**
   * @brief 卫星实体：自持星历，探测帧投递地面站（不写探测池）。
   * @param[in] session 同单参构造（示例：main 用 scene.config.satellite_a/b
   *            `SbirsSession::Create(...)`）。
   * @param[in] ground_station_source_id 融合源通道（示例：scene.config.satellite_*_source_id，
   *            每星互异）。
   * @param[in] position_ecef_m 初始 ECEF 位置（示例：scene JSON ephemeris 段，挂载后不变）。
   * @param[in] velocity_ecef_m_per_s 初始 ECEF 速度（同上 ephemeris 段）。
   * @param[in] attitude_eci_body_deg 初始 ECI 体轴姿态（同上 ephemeris 段）。
   * @param[in] ground_delivery 探测帧投递方式（示例：消息场景取 kMessage）。
   */
  SbirsSensorComponent(sbirs_sensor::session::SbirsSession session,
                       std::uint32_t ground_station_source_id,
                       sbirs_sensor::session::SbirsVector3M position_ecef_m,
                       sbirs_sensor::session::SbirsVector3M velocity_ecef_m_per_s,
                       sbirs_sensor::session::SbirsEulerAnglesDeg attitude_eci_body_deg,
                       SbirsGroundDeliveryMode ground_delivery =
                           SbirsGroundDeliveryMode::kSharedBlackboard);
  ~SbirsSensorComponent() override = default;

  SbirsSensorComponent(const SbirsSensorComponent&) = delete;
  SbirsSensorComponent& operator=(const SbirsSensorComponent&) = delete;

  const char* Name() const override { return "SbirsSensor"; }
  void OnAttach(Entity& host) override;
  void Step(World& world, double dt_sec) override;

  /** @brief 本周期适配后的泛型探测记录（融合聚合读）。 */
  /** @brief 当前电源状态（由 sensor_enabled 补丁唯一维护；未步进前默认 true，关机时组件不驱动会话）。 */
  bool powered_on() const { return powered_on_; }

  /** @brief 当前传感器设备实体 ID（融合源通道 ID；单站为 device_id_，卫星为 ground_station_source_id_）。 */
  std::uint32_t device_id() const {
    return ground_station_source_id_ != 0U ? ground_station_source_id_ : device_id_;
  }

  /** @brief 最近周期波束中心方位角（deg，ECI 极坐标参考——库内为弧度，组件转度显示；仅开机且最近周期 kCompleted 为有效扫描方位，关机时组件清零）。 */
  float scan_azimuth_deg() const { return scan_azimuth_deg_; }

  /**
   * @brief 最近周期调试视图快照（关机周期清零，拒绝周期为 kCycleNotExecuted）。
   */
  const sbirs_sensor::session::SbirsOutputDebugView& LastDebugView() const {
    return last_debug_view_;
  }

  /**
   * @brief 运行时修改入口：包装 SbirsSession::TryApplyRuntimeConfig。
   *
   * SBIRS 为立即提交：补丁经校验一次生效（调用即生效，session 层无回滚）。
   * @param[in] patch 运行期可变参数补丁（has_* 位标志选择字段）。
   * @return true 已应用；false 补丁无效或无变更（现有配置不变）。
   */
  bool TryApplyRuntimeConfig(const sbirs_sensor::config::SbirsRuntimeConfigPatch& patch);

  /**
   * @brief 开关星间 cross-cue（交叉提示）：开 = 本星每周期把自星宽场检出外发递话，
   *        并接收他星递话在下一周期注入会话（星→地→星，修订 1 拓扑）。
   * @note 默认关；场景 JSON cross_cue 键经 main 装配时调用。
   */
  void SetCrossCueEnabled(bool enabled) { cross_cue_enabled_ = enabled; }
  /** @brief 设置验收记录使用的传感器信息源名称。 */
  void SetSensorName(const std::string& name) { sensor_name_ = name; }
  /** @return cross-cue 是否开启。 */
  bool cross_cue_enabled() const { return cross_cue_enabled_; }

 private:
  // lifecycle_/exclusion_ 声明在 session_ 之前：析构逆序时 session_ 先析构（其析构不
  // 触达 recorder 指针），满足"recorder 生命周期长于 Session 注册期"约束。
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder lifecycle_{}; /**< 探测生命周期事件源 */
  sbirs_sensor::session::SbirsExclusionCauseRecorder exclusion_{}; /**< 排除原因跨周期差分事件源 */
  sbirs_sensor::session::SbirsSession session_;
  Entity* host_{nullptr};

  float scan_azimuth_deg_{0.0f}; /**< 最近周期波束中心方位角（deg，ECI 极坐标——库内弧度，组件转度显示） */

  sbirs_sensor::session::SbirsOutputDebugView last_debug_view_{};  /**< 本周期调试视图快照（视图行数据源） */

#if defined(CA_VIEW_LOG_MODE_DELTA)
  /// 视图模式二：上一周期逐目标调试状态（首次出现视为变化；表只增不减，示例不清理）。
  std::unordered_map<std::uint64_t, sbirs_sensor::session::SbirsDebugTargetStatus>
      prev_target_status_{};
#endif

  bool powered_on_{true}; /**< 电源状态（由 sensor_enabled 补丁唯一维护；关机时不驱动会话） */
  std::uint32_t device_id_{fusion::kSbirsSourceId}; /**< 单站传感器设备实体 ID（融合源通道 ID） */
  std::uint32_t ground_station_source_id_{0U}; /**< 非 0：探测帧投递地面站，不写探测池 */
  std::string sensor_name_{"红外预警卫星"}; /**< 传感器信息源名称（消息与融合日志透传） */
  SbirsGroundDeliveryMode ground_delivery_{SbirsGroundDeliveryMode::kSharedBlackboard};
  DetectionDeliveryMode detection_delivery_{DetectionDeliveryMode::kSharedBlackboard};
  bool use_own_satellite_pose_{false}; /**< 真：用本实体星历，不要求同实体 Flight */
  sbirs_sensor::session::SbirsVector3M own_position_ecef_m_{}; /**< 本实体卫星 ECEF 位置（m） */
  sbirs_sensor::session::SbirsVector3M own_velocity_ecef_m_per_s_{}; /**< 本实体卫星 ECEF 速度（m/s） */
  sbirs_sensor::session::SbirsEulerAnglesDeg own_attitude_eci_body_deg_{}; /**< 本实体卫星姿态（Body→ECI，deg） */

  /// 调试视图中文人读行写入（三模式分支见 .cpp；宏选择见 logger/logger_modes.h）。
  void LogDebugView(const sbirs_sensor::session::SbirsOutputDebugView& view);
  /// 周期输入组装：天基平台状态 + ECI 场景目标（消费方每周期注入共享场景状态）。
  sbirs_sensor::session::SbirsCycleInput BuildCycleInput(const AppSceneState& scene,
                                                  double dt_sec) const;
  /// 探测生命周期事件发布（首发现/更新/coasting/丢失 → World 信号 + 事件日志）。
  void PublishDetectionEvents(World& world, const AppSceneState& scene,
                             const sbirs_sensor::session::SbirsCycleResult& result);
  /// 排除原因跨周期差分事件（纯诊断观测，仅落事件日志）。
  void PublishExclusionEvents(World& world);
  /// 探测记录 → 融合探测池或 on_detection_batch_submitted（源通道 kSbirsSourceId）。
  void AdaptDetections(World& world, AppSceneState& scene,
                       const sbirs_sensor::session::SbirsCycleResult& result);
  /// 探测帧 → 地面站（黑板或 on_sbirs_frame_submitted 消息）。
  void PublishToGroundStation(World& world, AppSceneState& scene,
                              const sbirs_sensor::session::SbirsCycleResult& result);
  /// cross-cue：地面站转发收件（懒连接信号；忽略自己发的，按收到周期暂存）。
  void EnsureCrossCueRelayConnection(World& world);
  /// cross-cue：本星宽场检出 → on_sbirs_cross_cue（裁定 2 口径：测角+带误差距离）。
  void PublishCrossCue(World& world, const AppSceneState& scene,
                       const sbirs_sensor::session::SbirsCycleResult& result);

  bool cross_cue_enabled_{false}; /**< cross-cue 开关（默认关；场景 JSON cross_cue） */
  boost::signals2::scoped_connection cross_cue_relay_connection_; /**< 转发信号连接 */
  std::vector<SbirsCrossCueEvent> stashed_cue_events_; /**< 已收未消费的转发消息（含收到周期） */
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SBIRS_SENSOR_COMPONENT_H_
