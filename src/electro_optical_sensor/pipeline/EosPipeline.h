/**
 * @file EosPipeline.h
 * @brief 定义 EOS 核心处理层管线（扫描递推、视场判定、探测评估）。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SIGNAL_PIPELINE_EOS_PIPELINE_H_
#define ELECTRO_OPTICAL_SENSOR_SIGNAL_PIPELINE_EOS_PIPELINE_H_

#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electro_optical_sensor/session/EosSceneTypes.h"
#include "electro_optical_sensor/config/EosInternalExecutionConfig.h"
#include "electro_optical_sensor/pipeline/EosPipelineRuntimeTypes.h"

#include <string>
#include <vector>

namespace electro_optical_sensor {
namespace signal {
namespace pipeline {

using ::electro_optical_sensor::extension::EosPipelineWorkMode;

/** @brief 帧级别环境计算上下文，目标无关字段的聚合（完整定义见 cpp）。 */
struct FrameContext;

/**
 * @brief 管道内部球坐标场景目标（由控制器从平台锚点 ENU 输入派生）。
 * @note 公开输入面 `session::EosSceneTarget` 为平台锚点 ENU；体系球坐标
 *       （斜距/方位/仰角）是库内量测几何，不进入公开输入契约。
 */
struct EosPipelineSceneTarget {
  std::uint64_t target_id{0U};                                  /**< 目标标识 */
  std::string target_name{};                                    /**< 可选目标名称（人读/归属） */
  float range_m{0.0f};                                          /**< 斜距（单位：m） */
  float azimuth_deg{0.0f};                                      /**< 体系方位角（单位：deg） */
  float elevation_deg{0.0f};                                    /**< 体系仰角（单位：deg） */
  ::electro_optical_sensor::session::EosTargetAppearance appearance{}; /**< 外观参数 */
};

/** @brief 管道内部周期输入（球坐标场景 + 环境事实）。 */
struct EosPipelineCycleInput {
  std::uint32_t cycle_index{0U};       /**< 周期号 */
  float dt_sec{1.0f};                  /**< 周期步长（单位：s） */
  float platform_altitude_m{0.0f};     /**< 平台 WGS84 绝对海拔（单位：m） */
  std::vector<EosPipelineSceneTarget> scene{}; /**< 球坐标场景目标 */
};

/**
 * @brief EosPipeline 封装核心处理层执行。
 * @note 线程模型：实例维护可变扫描相位状态，不是线程安全类型；并发访问需外部同步。
 */
class EosPipeline {
 public:
  /**
   * @brief 以内部执行配置构造管线，扫描相位初始化为起始方位角。
   * @param[in] config 内部执行配置真值。
   */
  explicit EosPipeline(const config::execution::EosInternalExecutionConfig& config);

  // ---- 内部接口 (直接操作 EosInternalExecutionConfig, 无转换开销) ----

  /**
   * @brief 应用新的内部执行配置（直接覆盖，无转换开销）。
   * @param[in] config 新的内部执行配置。
   * @param[in] reset_scan_phase 是否将扫描相位重置为新配置的起始方位角，默认为 true。
   */
  void ApplyInternalConfig(const config::execution::EosInternalExecutionConfig& config,
                           bool reset_scan_phase = true);

  /**
   * @brief 捕获当前管线运行态快照（扫描相位等），用于跨周期恢复。
   * @return 管线运行态快照，`owner_identity` 指向本实例。
   */
  extension::EosPipelineRuntimeState CaptureRuntimeState() const;

  /**
   * @brief 从快照恢复管线运行态。
   * @param[in] state 待恢复的运行态快照。
   * @return owner/schema/config 匹配且恢复成功返回 true；否则返回 false 并记录错误日志。
   */
  bool RestoreRuntimeState(const extension::EosPipelineRuntimeState& state);

  /**
   * @brief 执行单周期核心处理（扫描递推、视场判定、探测评估）。
   * @param[in] input 管道内部周期输入（球坐标场景，由控制器从公开 ENU 输入派生）。
   * @return 单周期执行结果，含探测记录、归属映射与扫描方位角。
   * @note `sensor_enabled` 为 false 时直接返回 `kSensorPoweredOff` 未执行结果。
   */
  extension::EosPipelineExecuteResult RunCycle(const EosPipelineCycleInput& input);

  /**
   * @brief 设置传感器设备实体 ID（用于周期摘要日志行标记与融合源通道对齐）。
   * @param[in] sensor_entity_id 传感器设备实体 ID；传 0 恢复缺省常量。
   */
  void SetSensorEntityId(std::uint32_t sensor_entity_id) {
    sensor_entity_id_ = sensor_entity_id;
  }

  /** @return 当前配置的帧率（Hz）。 */
  float GetFrameRateHz() const { return config_.scan.frame_rate_hz; }

 private:
  void AdvanceScan(float dt_sec);
  bool IsTargetInCurrentFov(const EosPipelineSceneTarget& target) const;
  FrameContext BuildFrameContext(const EosPipelineCycleInput& input) const;
  output::EosDetectionRecord BuildDetectionRecord(std::uint64_t detection_id,
                                                  const EosPipelineSceneTarget& target,
                                                  const EosPipelineCycleInput& input,
                                                  const FrameContext& frame_ctx) const;

  config::execution::EosInternalExecutionConfig config_{};
  float current_scan_azimuth_deg_{0.0f};
  std::uint32_t sensor_entity_id_{0U};
};

}  // namespace pipeline
}  // namespace signal
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SIGNAL_PIPELINE_EOS_PIPELINE_H_
