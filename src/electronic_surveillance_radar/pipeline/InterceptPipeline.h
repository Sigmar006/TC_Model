/**
 * @file InterceptPipeline.h
 * @brief 定义电子侦察默认流水线实现。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_PIPELINE_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_PIPELINE_H_

#include <cstdint>

#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "electronic_surveillance_radar/config/EsrInternalExecutionConfig.h"
#include "electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "electronic_surveillance_radar/pipeline/HypothesisAssociator.h"
#include "electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.h"
#include "electronic_surveillance_radar/pipeline/InterceptPostProcessingExecutor.h"
#include "electronic_surveillance_radar/pipeline/KdTreeClusterer.h"
#include "electronic_surveillance_radar/pipeline/ObservationPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/ObservationPreprocessor.h"

namespace electronic_surveillance_radar {
namespace pipeline {

/**
 * @brief InterceptPipeline 是电子侦察流水线默认实现。
 *
 * 流水线分为两个阶段：
 *   1. 截获检测阶段（InterceptDetectionExecutor）——扫描图生成 + 逐辐射源截获判定
 *   2. 后处理阶段（InterceptPostProcessingExecutor）——预处理 + 编码 + 聚类 + 摘要 + 关联 + 真值评估
 */
class InterceptPipeline final {
 public:
  /**
   * @brief 构造默认流水线。
   * @param[in] config 内部执行态配置。
   */
  explicit InterceptPipeline(EsrInternalExecutionConfig config = {});

  /**
   * @brief 更新流水线配置（直接接收内部执行态配置）。
   *
   * 整块赋值 config_ 后重建派生状态（feature_scales、associator）。
   * 写路径直接吃 internal config，避免 internal→extension→internal 的往返；
   * RunCycle 的 per-cycle MutableEsrContext 投影不受影响。
   * @param[in] config 新的内部执行态配置。
   */
  void UpdateConfig(const EsrInternalExecutionConfig& config);

  /**
   * @brief 捕获 pipeline 累积运行期状态快照。
   *
   * 快照仅含累积量（next_observation_id、next_hypothesis_id、tracks），
   * **不含 config_ / feature_scales_**。配置是无累积的派生源（每 RunCycle 从
   * config_ 重新派生 pipeline_config/runtime_config），且 UpdateConfig 换 config
   * 时有意保留 tracks，故 config 与累积状态是独立状态空间。
   *
   * 按 docs/common/contract.md「运行期配置提交策略」，ESR 属立即提交类：配置不在
   * session 层回滚。本 capture/restore 只描述累积状态能力，不代表当前存在
   * pipeline 自报失败后的执行回滚路径。
   */
  extension::InterceptPipelineRuntimeState CaptureRuntimeState() const;

  /**
   * @brief 从快照恢复 pipeline 累积运行期状态。
   * @param[in] state CaptureRuntimeState 产生的快照。
   * @return owner_identity/schema 不匹配或快照损坏时返回 false。
   *
   * 只恢复累积量（scan phase/id/tracks），不恢复配置。参见 CaptureRuntimeState doc。
   */
  bool RestoreRuntimeState(const extension::InterceptPipelineRuntimeState& state);

  /**
   * @brief 设置传感器设备实体 ID（用于周期摘要日志行标记与融合源通道对齐）。
   * @param[in] sensor_entity_id 传感器设备实体 ID；传 0 恢复缺省常量。
   */
  void SetSensorEntityId(std::uint32_t sensor_entity_id) {
    sensor_entity_id_ = sensor_entity_id;
  }

  /**
   * @brief 执行单周期流水线。
   * @param[in] input_state 当前周期输入。
   * @param[in] environment 环境服务。
   * @return 单周期输出。
   */
  extension::InterceptPipelineResult RunCycle(
      const session::EsrCycleInput& input_state,
      const environment::IEsrEnvironmentService& environment);

 private:
  std::uint32_t sensor_entity_id_{0U};
  EsrInternalExecutionConfig config_{};
  ObservationFeatureScales feature_scales_{};
  ObservationPreprocessor preprocessor_{};
  KdTreeClusterer clusterer_{};
  HypothesisAssociator associator_{};
  InterceptDetectionExecutor detection_executor_{};
  InterceptPostProcessingExecutor post_processing_executor_{};
  double scan_phase_cycles_{0.0};
  std::uint64_t completed_receive_cycles_{0U};
  std::uint64_t next_observation_id_{1U};
  std::uint64_t next_hypothesis_id_{1U};
};

}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_PIPELINE_H_
