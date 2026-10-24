/**
 * @file AcceptanceSourceStage.h
 * @brief 验收行"来源"标注的**雷达/传感器处理阶段词表**（全库单源）。
 */

#ifndef ONEQ_SRC_COMMON_ACCEPTANCE_ACCEPTANCE_SOURCE_STAGE_H_
#define ONEQ_SRC_COMMON_ACCEPTANCE_ACCEPTANCE_SOURCE_STAGE_H_

namespace oneq {
namespace common {
namespace acceptance {

/**
 * @brief 阶段词表（2026-10-22 统一口径）。
 * @note 验收行只标"这个值由处理链哪一段出来"，**不标具体算法名**（最小二乘/差分/EKF…）：
 *   - 检测前：尚未形成量测（扫描调度、波束指向、门控预测）；
 *   - 检测后：检测判决产出的单拍点迹/量测（含通过门控的 RCS、角度、斜距）；
 *   - 滤波前：由检测后点迹在**进入跟踪滤波器之前**统计出的多拍量；
 *   - 滤波后：跟踪滤波后验（航迹状态：位置/速度/加速度/协方差）；
 *   - 识别后：识别器输出（类别/型号/置信度）。
 *   算法实现的说明归代码注释与 docs/，日志不承担算法披露职责。
 */
constexpr char kStageBeforeDetection[] = "检测前";
constexpr char kStageAfterDetection[] = "检测后";
constexpr char kStageBeforeFilter[] = "滤波前";
constexpr char kStageAfterFilter[] = "滤波后";
constexpr char kStageAfterRecognition[] = "识别后";

/**
 * @brief 速度/加速度等运动量的来源阶段文本。
 * @param[in] from_pre_filter true = 由检测后点迹在滤波前统计得到（"滤波前"）；
 *                            false = 跟踪滤波后验（"滤波后"）。
 * @return 阶段文本（静态字符串，可直接拼接）。
 */
inline const char* FilterStageText(bool from_pre_filter) {
  return from_pre_filter ? kStageBeforeFilter : kStageAfterFilter;
}

}  // namespace acceptance
}  // namespace common
}  // namespace oneq

#endif  // ONEQ_SRC_COMMON_ACCEPTANCE_ACCEPTANCE_SOURCE_STAGE_H_
