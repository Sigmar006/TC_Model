/**
 * @file FusionConfig.h
 * @brief 定义多源融合配置（权重/门限/窗口）。
 */

#ifndef ONEQ_FUSION_FUSION_CONFIG_H_
#define ONEQ_FUSION_FUSION_CONFIG_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/api.hpp"

namespace fusion {

/**
 * @brief 多源融合配置。
 * @note 融合置信度 = Σ 判决值 × 质量归一化 × 权重（滑窗内精确求和，不归一化）；
 *       权重按 source_id 索引，缺失时按 1.0 计。
 */
struct ONEQ_API FusionConfig {
  double position_radius_m{1000.0};    /**< 空间门限（带位置记录，单位：m） */
  double bearing_beamwidth_deg{5.0};   /**< 方位相干门限（仅方位记录，单位：deg） */
  double feature_threshold{0.0};       /**< 特征相似度门限（≤ 0 = 不启用特征门） */
  std::size_t window_size{10U};        /**< 每源每航迹滑窗量测数（超出驱逐并重算置信度） */
  std::size_t max_missed_cycles{5U};   /**< 失跟删除周期数（连续无量测超过该值删除航迹） */
  std::vector<double> source_weights{}; /**< 按 source_id 索引的源权重（空或缺项 = 1.0） */

  // ---- 航迹滤波（默认开启；置 false 时行为与 P2 前关联/置信度一致） ----
  bool enable_track_filtering{true};         /**< 是否启用逐航迹无迹滤波（关闭时无运动学估计） */
  double track_cycle_period_sec{1.0};        /**< 周期时长（滤波 dt = 周期差 × 该值，单位：s） */
  double track_process_noise{1.0};           /**< CV 过程噪声扩散系数 q（common/estimation 语义） */
  double track_initial_position_std_m{2000.0};      /**< 位置记录起始先验 1-σ（单位：m） */
  double track_initial_velocity_std_m_per_s{300.0}; /**< 速度起始先验 1-σ（单位：m/s） */
  double track_bearing_init_range_m{100000.0};      /**< 方位+原点记录起始距离先验（沿 LOS，单位：m） */
  double track_bearing_init_range_std_m{300000.0};  /**< 方位+原点起始距离先验 1-σ（各向同性，单位：m） */
  double default_position_noise_std_m{50.0};        /**< 位置量测噪声默认 1-σ（单位：m） */
  double default_bearing_noise_sigma_rad{2.0e-4};   /**< 方位量测噪声默认 1-σ（单位：rad，az/el 同 σ） */
  std::size_t confirm_hits{3U};              /**< 航迹确认门：累计命中数达该值转 confirmed */

  // ---- 接力覆盖估算（验收行派生量；库内无接力协议） ----
  /** 接力覆盖估算用视场角宽度（单位：deg；业务层按场景覆写）。
   *  验收行按「视场宽度 ÷ 视线角速率 = 覆盖时长」简易外推剩余覆盖时间；
   *  默认 20° 对应示例 SBIRS 宽场。 */
  /**
   * @brief 融合处理所属地面站名称（验收判定标准 第15项 `测角信息融合卫星地面站名称=`）。
   *
   * @details 判据要求把"各源测角信息在哪个地面站完成融合"写出来，否则外部无法区分
   *   同一目标在不同站的融合结果。空 = 未标注（该字段省略）。
   */
  std::string fusion_station_name = std::string("红外融合地面站");
  double relay_fov_width_deg{20.0};
};

}  // namespace fusion

#endif  // ONEQ_FUSION_FUSION_CONFIG_H_
