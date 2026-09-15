/**
 * @file SbirsDetectionLifecycleRecorder.h
 * @brief 定义 SBIRS-inspired 目标探测生命周期记录器。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_DETECTION_LIFECYCLE_RECORDER_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_DETECTION_LIFECYCLE_RECORDER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace sbirs_sensor {
namespace session {

struct SbirsCycleInput;

/** @brief 生命周期事件类型：首次检测、更新、coasting、丢失、未检测。 */
enum class ONEQ_API SbirsDetectionLifecycleEventKind {
  kFirstDetected = 0, /**< 目标首次被检测 */
  kUpdated,           /**< 目标状态更新（持续检测） */
  kCoasting,          /**< 暂无有效 NFOV 量测但仍保持锁定 */
  kLost,              /**< 目标丢失 */
  kNotDetected        /**< 目标本周期未检测 */
};

/** @brief 生命周期事件原因：视场外、低于 SNR 门限、捕获失败、调度跳过等。 */
enum class ONEQ_API SbirsDetectionLifecycleReason {
  kNone = 0,               /**< 无具体原因 */
  kOutOfFieldOfView,       /**< 目标在视场（FOV）外 */
  kBelowSnrThreshold,      /**< IR SNR 低于门限 */
  kTargetMissingFromInput, /**< 目标从输入场景消失 */
  kNfovAcquisitionFailed,  /**< 进入 NFOV 待捕获但捕获失败 */
  kSchedulerSkipped,       /**< WFOV 候选未被调度器选中 */
  kEstimationNisGateLost,  /**< EKF NIS 连续超限导致释放 NFOV 锁定 */
  kNfovPointingTimeout,    /**< ATP 光轴未在派生等待上限内稳定 */
  kNfovTrackingGateLost,   /**< NFOV 跟踪几何/SNR 门连续失败 */
  kUnknown                 /**< 未知原因 */
};

/**
 * @brief 单条目标探测生命周期事件，记录某周期某目标的状态变化与原因。
 * @note 仅供调试层消费，不进入 `SbirsOutputFrame` raw output。
 * @note 事件不携带事件等级与空间位置字段，也不做文件日志持久化（各记录器仅
 *       缓存最近周期）——2026-08-20 验收输出统计裁定不新增
 *       （docs/review/acceptance_output_inventory_2026-08-20.md §4.2/§6）。
 */
struct ONEQ_API SbirsDetectionLifecycleEvent {
  std::uint32_t cycle_index{0U}; /**< 周期序号 */
  std::uint64_t target_id{0U};   /**< 目标 ID */
  std::string target_name{};     /**< 目标名称 */
  SbirsDetectionLifecycleEventKind kind{
      SbirsDetectionLifecycleEventKind::kUpdated};                            /**< 事件类型 */
  SbirsDetectionLifecycleReason reason{SbirsDetectionLifecycleReason::kNone}; /**< 事件原因 */
  output::SbirsObservationStage observation_stage{
      output::SbirsObservationStage::kWideFieldSearch}; /**< 观测阶段 */
  float infrared_snr_linear{0.0f};                      /**< 红外通道线性 IR SNR */
  float estimated_range_m{0.0f};                        /**< 估计距离，单位 m */
  float max_detection_range_m{0.0f};                    /**< 当前时刻最大探测距离 d_max(t)，单位 m */
  attribution::SbirsTrackingSource tracking_source{
      attribution::SbirsTrackingSource::kNotApplicable}; /**< 正式跟踪来源 */
  bool has_estimation_nis{false};                       /**< 是否包含 EKF 估计跟踪 NIS */
  float estimation_nis{0.0f};                           /**< EKF 归一化新息平方 */
  bool estimation_nis_gate_exceeded{false};             /**< EKF NIS 是否超过 2 维 95% 门限 */
  int nfov_channel_id{-1}; /**< NFOV 通道编号；-1 表示 WFOV/未占用 NFOV 资源 */
  bool has_nfov_tracking_diagnostics{false};
  float nfov_pointing_error_deg{0.0f};
  bool nfov_geometry_gate_passed{false};
  bool nfov_snr_gate_passed{false};
  unsigned int nfov_tracking_gate_failure_count{0U};
  bool nfov_tracking_coasting{false};
};

/**
 * @brief 生命周期记录器配置。
 * @note `emit_not_detected_events` 为真时，未检测目标也会产生 `kNotDetected` 事件。
 */
struct ONEQ_API SbirsDetectionLifecycleRecorderConfig {
  bool emit_not_detected_events{false}; /**< 是否输出未检测事件 */
};

/**
 * @brief 目标探测生命周期记录器，跨周期累积目标状态并产生 found/lost 生命周期事件。
 * @note 该类不可拷贝但可移动，内部持有实现 (PIMPL)，记录器实例本身非线程安全。
 */
class ONEQ_API SbirsDetectionLifecycleRecorder {
 public:
  /**
   * @brief 构造记录器。
   * @param[in] config 记录器配置
   */
  explicit SbirsDetectionLifecycleRecorder(
      SbirsDetectionLifecycleRecorderConfig config = SbirsDetectionLifecycleRecorderConfig{});
  ~SbirsDetectionLifecycleRecorder();

  SbirsDetectionLifecycleRecorder(const SbirsDetectionLifecycleRecorder&) = delete;
  SbirsDetectionLifecycleRecorder& operator=(const SbirsDetectionLifecycleRecorder&) = delete;
  SbirsDetectionLifecycleRecorder(SbirsDetectionLifecycleRecorder&&) noexcept;
  SbirsDetectionLifecycleRecorder& operator=(SbirsDetectionLifecycleRecorder&&) noexcept;

  /**
   * @brief 推进一个周期，对照输入与结果产生本周期生命周期事件。
   * @note `result.status != kCompleted` 时不产生事件，也不修改累积状态。
   * @param[in] input 单周期输入
   * @param[in] result 单周期结构化结果
   * @return 本周期产生的生命周期事件列表
   */
  std::vector<SbirsDetectionLifecycleEvent> Update(const SbirsCycleInput& input,
                                                   const SbirsCycleResult& result);
  /** @brief 清空记录器内部累积的目标状态。 */
  void Reset();

  /**
   * @brief 获取最近一次执行周期 `Update()` 返回的生命周期事件列表。
   *
   * 事件在每次执行周期的 `Update()` 调用时缓存；非执行周期不刷新缓存，
   * 保留上一次执行周期的事件。供注册到 Session 后由调用方事后读取。
   * @return 最近一次执行周期 `Update()` 产生的事件列表的 const 引用。
   */
  const std::vector<SbirsDetectionLifecycleEvent>& GetLastEvents() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_DETECTION_LIFECYCLE_RECORDER_H_
