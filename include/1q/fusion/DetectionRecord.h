/**
 * @file DetectionRecord.h
 * @brief 定义多源融合的泛型探测记录。
 */

#ifndef ONEQ_FUSION_DETECTION_RECORD_H_
#define ONEQ_FUSION_DETECTION_RECORD_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace fusion {

/**
 * @brief 泛型探测记录（算法不感知 ESR/EOS/AR 具体类型；库提供官方适配器
 *        SensorAdapters.h（可选），业务层也可自行适配）。
 * @note 位置/方位/特征三通道可独立存在：带位置记录走空间门限，
 *       仅方位记录走方位相干门限，二者皆无时仅走身份键与特征门限。
 * @note 量测原点通道（P2）：has_sensor_origin 为真时方位角定义为该原点（度制 LLA）
 *       传感器局部 ENU（az 自北向东 [-180,180]、el 出地平 [-90,90]），该类记录
 *       额外参与融合引擎的三维方位滤波；无原点的方位记录只参与关联。
 *       跨系对齐（如 ECI 角度 → ENU）归调用方，库内不做跨系转换。
 */
struct ONEQ_API DetectionRecord {
  std::uint64_t key{0U};                 /**< 库内身份键（调用方提供并保证跨源一致；0 = 无身份） */
  std::uint32_t source_id{0U};           /**< 源通道标识（由调用方映射） */
  bool has_position{false};              /**< 是否携带位置量测 */
  oneq::coordinate::LlaPositionDegM position{}; /**< 位置量测（度制 LLA） */
  bool has_bearing{false};              /**< 是否携带方位量测 */
  double bearing_az_deg{0.0};            /**< 方位角（单位：deg） */
  double bearing_el_deg{0.0};            /**< 俯仰角（单位：deg） */
  bool has_sensor_origin{false};         /**< 是否携带量测原点（角度-only 三维滤波通道） */
  oneq::coordinate::LlaPositionDegM sensor_origin{}; /**< 量测原点（度制 LLA）；携带时方位定义为该原点传感器局部 ENU */
  bool has_bearing_noise{false};         /**< 是否携带方位量测噪声 1-σ */
  double bearing_noise_sigma_rad{0.0};   /**< 方位轴 1-σ（单位：rad，az/el 同 σ；缺省用 FusionConfig 默认） */
  std::vector<double> feature{};         /**< 特征向量（可选，空 = 无特征） */
  double verdict{0.0};                   /**< 判决值（0/1） */
  double quality{0.0};                   /**< 探测质量归一化值（[0,1]） */
  bool has_timestamp{false};             /**< 是否携带量测绝对时间戳 */
  double timestamp_s{0.0};               /**< 量测绝对时刻（单位：秒） */
};

}  // namespace fusion

#endif  // ONEQ_FUSION_DETECTION_RECORD_H_
