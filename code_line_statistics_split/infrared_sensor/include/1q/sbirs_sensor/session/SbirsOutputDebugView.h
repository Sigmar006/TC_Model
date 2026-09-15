/**
 * @file SbirsOutputDebugView.h
 * @brief 定义 SBIRS-inspired 输出开发调试视图。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_DEBUG_VIEW_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_DEBUG_VIEW_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"

namespace sbirs_sensor {
namespace session {

struct SbirsCycleInput;

/** @brief 调试视图中的目标状态：已检测、低于门限、coasting、不在输出或未执行。 */
enum class ONEQ_API SbirsDebugTargetStatus {
  kDetected = 0,           /**< 已检测（通过门限） */
  kObservedBelowThreshold, /**< 被观测但低于 SNR 门限 */
  kCoasting,               /**< 暂无有效 NFOV 量测但仍保持跟踪锁定 */
  kNotInOutput,            /**< 不在输出中 */
  kCycleNotExecuted        /**< 本周期未执行 */
};

/**
 * @brief 调试视图中单个目标的状态快照，由输入与结构化结果回填。
 * @note 仅供人读诊断，不进入 `SbirsOutputFrame` raw output。
 */
struct ONEQ_API SbirsDebugTargetState {
  std::uint64_t target_id{0U}; /**< 目标 ID */
  std::string target_name{};   /**< 目标名称 */
  SbirsDebugTargetStatus status{SbirsDebugTargetStatus::kNotInOutput}; /**< 调试状态 */
  bool present_in_input{false};      /**< 是否在输入场景中存在 */
  bool has_raw_output_record{false}; /**< 是否在 raw output 中有记录 */
  bool detected{false};              /**< 是否通过门限检测 */
  attribution::SbirsTrackingSource tracking_source{
      attribution::SbirsTrackingSource::kNotApplicable}; /**< 正式跟踪来源 */
  float estimated_range_m{0.0f};     /**< 估计距离，单位 m */
  float max_detection_range_m{0.0f}; /**< 当前时刻最大探测距离 d_max(t)，单位 m（WFOV 门限反解） */
  bool has_estimation_nis{false};    /**< 是否包含 EKF 估计跟踪 NIS */
  float estimation_nis{0.0f};        /**< EKF 归一化新息平方 */
  bool estimation_nis_gate_exceeded{false}; /**< EKF NIS 是否超过 2 维 95% 门限 */
  int nfov_channel_id{-1};           /**< NFOV 通道编号；-1 表示 WFOV/未占用 NFOV 资源 */
  bool has_nfov_tracking_diagnostics{false}; /**< 是否包含闭环 NFOV 跟踪门诊断 */
  float nfov_pointing_error_deg{0.0f};       /**< 有效光轴与目标 LOS 角距 */
  bool nfov_geometry_gate_passed{false};
  bool nfov_snr_gate_passed{false};
  unsigned int nfov_tracking_gate_failure_count{0U};
  bool nfov_tracking_coasting{false};
  double measurement_time_s{0.0};    /**< 量测产生的绝对仿真时刻（秒） */
  float azimuth_rad{0.0f};           /**< 方位角（ECI 极坐标），单位 rad，[0, 2π) */
  float elevation_rad{0.0f};         /**< 仰角（ECI 极坐标），单位 rad，[-π/2, π/2] */
  float infrared_snr_linear{0.0f};   /**< 红外通道线性 IR SNR */
  output::SbirsObservationStage observation_stage{output::SbirsObservationStage::kWideFieldSearch}; /**< 观测阶段 */
};

/**
 * @brief 单周期输出调试视图，汇总周期执行状态与各目标调试快照。
 * @note 属于三层输出模型的最上层调试视图，仅供消费，不影响 raw output。
 */
struct ONEQ_API SbirsOutputDebugView {
  std::uint32_t input_cycle_index{0U};  /**< 输入周期序号 */
  std::uint32_t output_cycle_index{0U}; /**< 输出周期序号 */
  bool executed_this_cycle{false};      /**< 本周期是否执行 */
  SbirsPipelineAbortReason abort_reason{SbirsPipelineAbortReason::kNone}; /**< 中止原因 */
  std::vector<SbirsDebugTargetState> targets{}; /**< 各目标调试快照 */
  SbirsIssueList issues{};                      /**< 统一问题列表（规则 14，与 SbirsCycleResult.issues 一致） */
};

/**
 * @brief 由周期输入与结构化结果构造调试视图。
 * @param[in] input 单周期输入
 * @param[in] result 单周期结构化结果
 * @return 回填好的 `SbirsOutputDebugView`
 * @note 纯函数，不修改输入；消费 DTO 与归属信息，不混入 raw output。
 */
class ONEQ_API SbirsOutputDebugViewBuilder {
 public:
  static SbirsOutputDebugView Build(const SbirsCycleInput& input, const SbirsCycleResult& result);
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_DEBUG_VIEW_H_
