/**
 * @file SbirsCycleResult.h
 * @brief 定义 SBIRS-inspired 单周期执行结果。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_RESULT_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_RESULT_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsInputValidation.h"
#include "1q/sbirs_sensor/session/SbirsExternalCue.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief 单周期原始系统输出帧，属于三层输出模型的最底层主输出。
 * @note 不携带目标真值或归属信息；`detections` 为 WFOV/NFOV 通道本周期检测记录。
 */
struct ONEQ_API SbirsOutputFrame {
  std::uint32_t cycle_index{0U};              /**< 周期序号 */
  float scan_azimuth_rad{0.0f};               /**< 本周期 WFOV 扫描方位角，单位 rad（ECI 极坐标参考，同 SbirsDetectionRecord::azimuth_rad，[0, 2π)） */
  float scan_elevation_rad{0.0f};             /**< 本周期 WFOV 扫描中心俯仰角，单位 rad（ECI 极坐标参考，[-π/2, π/2]；2-D 栅格下为当前行中心） */
  output::SbirsDetectionRecordList detections{}; /**< 检测记录列表 */
};

/**
 * @brief 单周期结构化执行结果，是外部调用方通过 `StepWithResult()` 获取的主要产物。
 * @note 输出帧、归属、校验、执行/中止状态分层携带。非执行周期（校验失败）返回默认空帧，
 *       不复用上一有效输出（见 contract.md §实现安全与失败语义规则 3）。
 */
struct ONEQ_API SbirsCycleResult {
  std::uint32_t input_cycle_index{0U};       /**< 对应输入周期序号 */
  SbirsOutputFrame output_frame{};           /**< 原始系统输出帧 */
  attribution::SbirsDetectionAttributionRecordList detection_attributions{}; /**< 检测归属列表 */
  SbirsIssueList issues{};                   /**< 统一问题列表（规则 14：校验问题 phase=kInputValidation + 执行诊断） */
  SbirsCycleStatus status{
      SbirsCycleStatus::kRejectedInvalidInput}; /**< 当前周期高层执行状态 */
  SbirsPipelineAbortReason abort_reason{SbirsPipelineAbortReason::kNone}; /**< 中止原因 */
  std::vector<SbirsWideCueMeasurement> wide_cue_measurements{}; /**< 宽场候选量测（cross-cue
      外发数据源，修订 7；仅归属/调试面，基线无消费者行为不变） */
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_RESULT_H_
