/**
 * @file RirFeatureMeasurementTypes.h
 * @brief 特征量测帧公开类型（双产品出口①）。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_FEATURE_MEASUREMENT_TYPES_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_FEATURE_MEASUREMENT_TYPES_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace remote_identification_radar {
namespace session {

/**
 * @brief RirRcsFeatureObservation RCS 特征量测（dBsm 域）。
 * @note mean_dbsm 为无偏真值取样；std_db 是由 SNR 推定的不确定度（3 dB/√snr），
 *       不是实测起伏统计。
 */
struct ONEQ_API RirRcsFeatureObservation {
  bool valid{false};                  /**< 维度可用性（样本非空且视角覆盖达标）。 */
  float mean_dbsm{0.0f};              /**< 视线角插值平均 RCS（dBsm）。 */
  float std_db{0.0f};                 /**< SNR 推定量测不确定度（dB）。 */
  float azimuth_variation_db{0.0f};   /**< 方位变化幅度（dB）；样本不足时为 0。 */
  float elevation_variation_db{0.0f}; /**< 俯仰变化幅度（dB）；样本不足时为 0。 */
  float peak_to_valley_db{0.0f};      /**< 样本峰谷比（dB）。 */
  float aspect_coverage_deg{0.0f};    /**< 有效视角覆盖（deg）。 */
  float quality{0.0f};                /**< 维度质量因子，[0, 1]。 */
};

/**
 * @brief RirMotionFeatureObservation 运动特征量测（由滤波航迹估计派生）。
 * @note 无航向（速度方向）字段——2026-08-20 验收输出统计裁定不新增
 *       （docs/review/acceptance_output_inventory_2026-08-20.md §4.5/§6）。
 */
struct ONEQ_API RirMotionFeatureObservation {
  bool valid{false};                 /**< 维度可用性（快照为已确认航迹）。 */
  float speed_m_per_s{0.0f};         /**< 速度模长（m/s）。 */
  float altitude_m{0.0f};            /**< 绝对高度（m）= 平台海拔 + 雷达局部 ENU z。 */
  float acceleration_m_per_s2{0.0f}; /**< 加速度模长（m/s²）。 */
  float turn_radius_m{0.0f};         /**< 转弯半径（m）；直线飞行时为 0。 */
  bool is_straight{false};           /**< 直线飞行标记（横向加速度低于阈值）。 */
  float quality{0.0f};               /**< 维度质量因子，[0, 1]。 */
};

/**
 * @brief RirPolarizationFeatureObservation 双通道极化特征量测（dB 域）。
 */
struct ONEQ_API RirPolarizationFeatureObservation {
  bool valid{false};                    /**< 维度可用性（双通道样本齐全且在覆盖内）。 */
  float energy_difference_db{0.0f};     /**< 通道功率比 10·log10(E1/E2)（dB）。 */
  float relative_difference_db{0.0f};   /**< 通道相对差（dB）。 */
  float energy_sum_db{0.0f};            /**< 参考距离补偿后的总能量（dB）。 */
  float quality{0.0f};                  /**< 维度质量因子，[0, 1]。 */
};

/**
 * @brief RirPolarizationStatsFeatureObservation 极化散射矩阵统计特征量测
 *        （五量×(均值, 标准差)；统计总体＝当前视角附近的窗口行）。
 * @note 方向角 psi 周期 180°，采用圆统计：psi_mean_deg 为均值角（deg）、
 *       psi_std_deg 为角度散布（deg）——缠绕处（+89° 与 −89° 物理同向）不会
 *       产生算术平均式的假 0°。其余四量为算术均值与样本标准差（n=1 时 std=0）。
 *       细分类型（弹头/重诱饵/轻诱饵/碎片）判据未冻结，当前仅量测、不判决。
 */
struct ONEQ_API RirPolarizationStatsFeatureObservation {
  bool valid{false};                    /**< 维度可用性（窗口存在有效行）。 */
  float determinant_mean{0.0f};         /**< |det S| 均值。 */
  float determinant_std{0.0f};          /**< |det S| 样本标准差。 */
  float span_mean{0.0f};                /**< 总功率（Span）均值。 */
  float span_std{0.0f};                 /**< Span 样本标准差。 */
  float depolarization_mean{0.0f};      /**< 去极化系数均值。 */
  float depolarization_std{0.0f};       /**< 去极化系数样本标准差。 */
  float psi_mean_deg{0.0f};             /**< 本征极化方向角均值（圆统计均值角，deg）。 */
  float psi_std_deg{0.0f};              /**< 本征极化方向角角度散布（圆统计口径，deg）。 */
  float tau_mean_deg{0.0f};             /**< 本征极化椭圆率均值（算术，deg）。 */
  float tau_std_deg{0.0f};              /**< 本征极化椭圆率样本标准差（deg）。 */
};

/**
 * @brief RirRangeProfileFeatureObservation 宽带一维距离像特征量测。
 */
struct ONEQ_API RirRangeProfileFeatureObservation {
  bool valid{false};                        /**< 维度可用性（散射列表非空且分辨率达标）。 */
  float length_m{0.0f};                     /**< 目标长度（首末有效峰距离，m）。 */
  std::uint32_t peak_count{0U};             /**< 有效峰数量。 */
  float peak_energy_concentration{0.0f};    /**< 前 K 峰能量集中率，[0, 1]。 */
  float resolution_m{0.0f};                 /**< 实际距离分辨率 c/(2B)（m）。 */
  float quality{0.0f};                      /**< 维度质量因子，[0, 1]。 */
};

/**
 * @brief RirFeatureObservations 四维特征量测聚合（RCS/运动/极化/距离像）。
 */
struct ONEQ_API RirFeatureObservations {
  RirRcsFeatureObservation rcs{};                 /**< RCS 维。 */
  RirMotionFeatureObservation motion{};           /**< 运动维。 */
  RirPolarizationFeatureObservation polarization{}; /**< 极化维（双通道能量特征）。 */
  RirPolarizationStatsFeatureObservation polarization_stats{}; /**< 极化散射矩阵统计维。 */
  RirRangeProfileFeatureObservation range_profile{}; /**< 距离像维。 */
};

/**
 * @brief RirFeatureMeasurementRecord 单周期单航迹特征量测记录
 */
struct ONEQ_API RirFeatureMeasurementRecord {
  std::uint64_t association_key{0U}; /**< RIR 内部航迹关联键（库内键透传）。 */
  RirFeatureObservations features{}; /**< 四维特征量测（无效维按内部现状 valid=false 透出）。 */
  std::uint8_t valid_feature_mask{0U}; /**< 四维有效位掩码（RirRecognitionFeatureDimension 按位或）。 */
  float look_az_deg{0.0f};             /**< 视线方位角（雷达局部 ENU，自 +x 东起量，deg）。 */
  float look_el_deg{0.0f};             /**< 视线俯仰角（deg）。 */
  float range_m{0.0f};                 /**< 目标斜距（m）。 */
  float snr_db{0.0f};                  /**< 周期信噪比（dB）。 */
  float dwell_sec{0.0f};               /**< 识别驻留时间（s）。 */
  float bandwidth_hz{0.0f};            /**< 有效带宽（Hz）。 */
  bool has_platform_position{false};   /**< 成功执行周期恒为 true（replay 兼容字段）。 */
  oneq::coordinate::EcefPositionM platform_position{}; /**< 平台 ECEF 位置（m）。 */
  std::uint32_t cycle_index{0U};       /**< 归属周期号。 */
  std::uint64_t batch_id{0U};          /**< 归属批号。 */
};

/**
 * @brief RirFeatureMeasurementFrame 特征量测帧（出口①的适配器入参形态）。
 * @note 由调用方从 RirOutputFrame（cycle/batch + feature_measurements）组装；
 *       随周期产出，不跨周期持有。
 */
struct ONEQ_API RirFeatureMeasurementFrame {
  std::uint32_t input_cycle_index{0U}; /**< 周期号。 */
  std::uint64_t batch_id{0U};          /**< 批号。 */
  std::vector<RirFeatureMeasurementRecord> records{}; /**< 逐航迹特征量测记录。 */
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_FEATURE_MEASUREMENT_TYPES_H_
