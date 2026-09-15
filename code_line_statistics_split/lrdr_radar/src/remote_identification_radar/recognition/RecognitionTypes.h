/**
 * @file RecognitionTypes.h
 * @brief 远程目标识别内部观测与特征类型。
 *
 * 识别链路内部类型（`src/remote_identification_radar/recognition/`），不进入 public API。
 * 观测为效能级派生：由目标真值特征列表经 SNR、距离、带宽、驻留与视角覆盖
 * 等效能约束转换而来，保证可解释、可配置、可回放。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_TYPES_H_
#define REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_TYPES_H_

#include <cstdint>

#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "remote_identification_radar/recognition/PolarizationStatsExtractor.h"

namespace remote_identification_radar {
namespace recognition {

/**
 * @brief RirObservationContext 单周期识别观测上下文（效能约束）。
 */
struct RirObservationContext {
  float snr_db{0.0f};                 /**< 周期信噪比（dB）。 */
  float range_m{0.0f};                /**< 目标斜距（m）。 */
  float bandwidth_hz{0.0f};           /**< 有效带宽（Hz）；决定距离像分辨率。 */
  float dwell_sec{0.0f};              /**< 识别驻留时间（s）。 */
  float look_az_deg{0.0f};            /**< 视线方位角（目标参考系，deg）。 */
  float look_el_deg{0.0f};            /**< 视线俯仰角（目标参考系，deg）。 */
  float platform_altitude_m{0.0f};    /**< 平台绝对高度（m）；用于运动高度换算。 */
  float minimum_aspect_coverage_deg{15.0f}; /**< 视角覆盖下限（deg），低于则 RCS 维度无效。 */
  float max_range_resolution_m{0.0f}; /**< 距离像允许的最大距离分辨率（m）；0 表示不限。 */
};

/**
 * @brief RirRcsObservation RCS 特征观测（dBsm 域）。
 */
struct RirRcsObservation {
  bool valid{false};          /**< 维度可用性（样本非空、视角覆盖达标）。 */
  float mean_dbsm{0.0f};      /**< 视线角插值平均 RCS（dBsm）。 */
  float std_db{0.0f};         /**< 量测/起伏标准差（dB）。 */
  float azimuth_variation_db{0.0f};  /**< 方位变化幅度（dB）；样本不足时无效。 */
  float elevation_variation_db{0.0f}; /**< 俯仰变化幅度（dB）；样本不足时无效。 */
  float peak_to_valley_db{0.0f}; /**< 样本峰谷比（dB）。 */
  float aspect_coverage_deg{0.0f}; /**< 有效视角覆盖（deg）。 */
  float quality{0.0f};        /**< 维度质量因子，[0, 1]。 */
};

/**
 * @brief RirMotionObservation 运动特征观测（由滤波航迹多周期估计派生）。
 */
struct RirMotionObservation {
  bool valid{false};          /**< 维度可用性（快照为已确认航迹）。 */
  float speed_mps{0.0f};      /**< 速度模长（m/s）。 */
  float altitude_m{0.0f};     /**< 绝对高度（m）= 平台高度 + 雷达局部 position_z。 */
  float acceleration_mps2{0.0f}; /**< 加速度模长（m/s²）。 */
  float turn_radius_m{0.0f};  /**< 转弯半径（m）；直线飞行时无效。 */
  bool is_straight{false};    /**< 直线飞行标记（横向加速度低于阈值）。 */
  float quality{0.0f};        /**< 维度质量因子，[0, 1]（由估计不确定度归一化）。 */
};

/**
 * @brief RirPolarizationObservation 双通道极化特征观测（dB 域）。
 */
struct RirPolarizationObservation {
  bool valid{false};          /**< 维度可用性（双通道样本齐全且在覆盖内）。 */
  float energy_difference_db{0.0f}; /**< 通道功率比，10*log10(E1/E2)。 */
  float relative_difference_db{0.0f}; /**< 通道相对差，10*log10(|E1-E2|/(E1+E2))。 */
  float energy_sum_db{0.0f};  /**< 统一参考条件下的总能量（dB）。 */
  float quality{0.0f};        /**< 维度质量因子，[0, 1]。 */
};

/**
 * @brief RirRangeProfileObservation 宽带一维距离像特征观测。
 */
struct RirRangeProfileObservation {
  bool valid{false};          /**< 维度可用性（散射列表非空且分辨率达标）。 */
  float length_m{0.0f};       /**< 目标长度（首末有效峰距离，m）。 */
  std::uint32_t peak_count{0U}; /**< 有效峰数量。 */
  float peak_energy_concentration{0.0f}; /**< 前 K 峰能量集中率，[0, 1]。 */
  float resolution_m{0.0f};   /**< 实际距离分辨率 c/(2B)（m）。 */
  float quality{0.0f};        /**< 维度质量因子，[0, 1]。 */
};

/**
 * @brief RirFeatureSet 单周期四维特征观测集合。
 */
struct RirFeatureSet {
  RirRcsObservation rcs{};
  RirMotionObservation motion{};
  RirPolarizationObservation polarization{};
  RirPolarizationStatsObservation polarization_stats{}; /**< 极化散射矩阵统计维（判决判据未冻结，不进掩码）。 */
  RirRangeProfileObservation range_profile{};
  /** @brief 本周期有效特征维度掩码（RirRecognitionFeatureDimension 按位或）。 */
  std::uint8_t valid_feature_mask{0U};
};

}  // namespace recognition
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_TYPES_H_
