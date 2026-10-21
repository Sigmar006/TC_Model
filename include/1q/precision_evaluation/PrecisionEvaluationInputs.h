/**
 * @file PrecisionEvaluationInputs.h
 * @brief 检测级输入装配：把宿主侧已拿到的帧数据还原成精度评估会话消费的单星周期结果。
 *
 * 现场宿主里传感器与融合处理器是两个模块，融合侧只拿得到"帧"（目标 ID / 视线方向 /
 * 卫星位置 / 观测阶段 / 量测时刻 / 焦平面脱靶量），拿不到 `SbirsCycleResult`；而精度
 * 评估会话要求两星的检测记录与归属记录。本接口把这层装配固定下来：宿主逐条填
 * `PrecisionDetectionInput`，由库内统一完成"检测记录 + 归属记录"的构造与目标 ID 映射，
 * 避免宿主重复实现阶段枚举、归属索引和空归属规则。
 *
 * @note 只填会话消费的字段：`detection_id` / `detected` / `observation_stage` /
 *       `azimuth_rad` / `elevation_rad` / `measurement_time_s`，归属记录带
 *       `target_id` 与焦平面脱靶量。`target_id == 0` 的检测被视为未归属，不建归属记录，
 *       会话因此不会把它送进双星交会（与库内归属语义一致）。
 * @note 装配是纯数据搬运、无状态，故以 inline 形式随头文件分发：宿主只依赖公开头，
 *       不引入新的链接符号。
 */

#ifndef ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_INPUTS_H_
#define ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_INPUTS_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace precision_evaluation {

/**
 * @brief 宿主侧一条窄场检测（装配会话输入的最小集合）。
 */
struct ONEQ_API PrecisionDetectionInput {
  std::uint64_t detection_id{0U};            /**< 本周期检测标识（同一星内唯一） */
  std::uint64_t target_id{0U};               /**< 归属真值目标 ID；0 = 未归属（不进交会） */
  double measurement_time_s{0.0};            /**< 量测时刻（s，与场景时间同基准） */
  float azimuth_rad{0.0f};                   /**< 方位角（ECI 极坐标，rad） */
  float elevation_rad{0.0f};                 /**< 仰角（ECI 极坐标，rad） */
  sbirs_sensor::output::SbirsObservationStage observation_stage{
      sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack};
  /**< 观测阶段；宽场搜索阶段的样本会被会话剔除（双星定位仅用窄场数据）。 */
  bool detected{true};                       /**< 是否通过探测门限 */
  bool has_focal_plane_offset{false};        /**< 是否带焦平面脱靶量（第 26 项数据源） */
  float focal_plane_offset_x_m{0.0f};        /**< 焦平面脱靶量 x（m） */
  float focal_plane_offset_y_m{0.0f};        /**< 焦平面脱靶量 y（m） */
};

/**
 * @brief 一颗星单周期的检测集合。
 */
struct ONEQ_API PrecisionSatelliteCycleInput {
  std::uint32_t cycle_index{0U};             /**< 对应周期序号 */
  std::vector<PrecisionDetectionInput> detections; /**< 本周期检测 */
};

/**
 * @brief 检测级输入 → 会话可消费的单星周期结果（`status` 置 `kCompleted`）。
 *
 * @param[in] input 一颗星单周期的检测集合
 * @return 装配后的周期结果；`output_frame.detections` 按输入顺序逐条对应，
 *         `detection_attributions` 只为 `target_id != 0` 的检测生成。
 */
inline sbirs_sensor::session::SbirsCycleResult BuildSatelliteCycleResult(
    const PrecisionSatelliteCycleInput& input) {
  sbirs_sensor::session::SbirsCycleResult result;
  result.input_cycle_index = input.cycle_index;
  // 装配结果代表"本周期正常完成"：会话对非 kCompleted 结果一律不取样本。
  result.status = sbirs_sensor::session::SbirsCycleStatus::kCompleted;
  result.output_frame.cycle_index = input.cycle_index;
  result.output_frame.detections.reserve(input.detections.size());
  for (const PrecisionDetectionInput& detection : input.detections) {
    sbirs_sensor::output::SbirsDetectionRecord record;
    record.detection_id = detection.detection_id;
    record.measurement_time_s = detection.measurement_time_s;
    record.azimuth_rad = detection.azimuth_rad;
    record.elevation_rad = detection.elevation_rad;
    record.observation_stage = detection.observation_stage;
    record.detected = detection.detected;
    result.output_frame.detections.push_back(record);

    if (detection.target_id == 0U) {
      continue;  // 未归属：不建归属记录，不参与双星交会与逐目标误差。
    }
    sbirs_sensor::attribution::SbirsDetectionAttributionRecord attribution;
    attribution.detection_id = detection.detection_id;
    attribution.target_id = detection.target_id;
    attribution.has_focal_plane_offset = detection.has_focal_plane_offset;
    attribution.focal_plane_offset_x_m = detection.focal_plane_offset_x_m;
    attribution.focal_plane_offset_y_m = detection.focal_plane_offset_y_m;
    result.detection_attributions.push_back(attribution);
  }
  return result;
}

}  // namespace precision_evaluation

#endif  // ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_INPUTS_H_
