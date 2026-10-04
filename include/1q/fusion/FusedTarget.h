/**
 * @file FusedTarget.h
 * @brief 定义融合目标态势输出记录。
 */

#ifndef ONEQ_FUSION_FUSED_TARGET_H_
#define ONEQ_FUSION_FUSED_TARGET_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace fusion {

/**
 * @brief 单源通道探测状态与最近量测。
 */
struct ONEQ_API ChannelMeasurement {
  std::uint32_t source_id{0U};                 /**< 源通道标识 */
  std::string source_name{};                   /**< 传感器信息源名称（空 = 未提供） */
  std::size_t sample_count{0U};                /**< 滑窗内该源量测数 */
  double latest_verdict{0.0};                  /**< 最近一次判决值 */
  double latest_quality{0.0};                  /**< 最近一次质量归一化值 */
  bool has_position{false};                    /**< 该源是否提供位置量测 */
  oneq::coordinate::LlaPositionDegM position{}; /**< 最近一次位置量测 */
  bool has_bearing{false};                     /**< 该源是否提供方位量测 */
  double bearing_az_deg{0.0};                  /**< 最近一次方位角（单位：deg） */
  double bearing_el_deg{0.0};                  /**< 最近一次俯仰角（单位：deg） */
};

/**
 * @brief 航迹生命周期状态（P2 起）。
 * @note 确认门为累计命中数 ≥ FusionConfig::confirm_hits；kCoasting 表示已确认航迹
 *       当前失跟、运动学估计由预测外推维持。
 */
enum class ONEQ_API FusedTrackLifecycle : std::uint8_t {
  kTentative = 0, /**< 已起始未确认（命中数低于 confirm_hits）。 */
  kConfirmed = 1,  /**< 已确认且最近周期收到量测。 */
  kCoasting = 2    /**< 已确认但当前失跟（预测外推维持）。 */
};

/**
 * @brief 运动学估计（P2 起，ECEF 滤波后验）。
 * @note 协方差为 6×6 ECEF 行主序，状态排列 [x, vx, y, vy, z, vz]（与
 *       common/estimation CV 布局一致）；角度-only 弱可观测必须由该协方差如实承载
 *       （contract.md §目标处理分层契约规则 6），消费方不得只读点估计。
 * @note ECEF 三维位置向量不单独导出（公开面为 LLA 位置 + ECEF 速度/协方差，
 *       消费方可自行换算）；融合层状态为 6 维 CV、无加速度字段——两者均为
 *       2026-08-20 验收输出统计裁定不新增（docs/review/acceptance_output_inventory_
 *       2026-08-20.md §4.2/§6）。
 */
struct ONEQ_API FusedKinematicEstimate {
  oneq::coordinate::LlaPositionDegM position{}; /**< 位置估计（度制 LLA）。 */
  std::array<double, 3U> velocity_ecef_m_per_s = std::array<double, 3U>(); /**< ECEF 速度（m/s）。 */
  std::array<double, 36U> covariance_ecef{}; /**< 6×6 协方差（行主序 [x,vx,y,vy,z,vz]）。 */
};

/**
 * @brief 融合目标态势记录。
 */
struct ONEQ_API FusedTarget {
  std::uint64_t key{0U};                  /**< 航迹库内键（身份键，或引擎为无身份航迹合成的键） */
  std::vector<ChannelMeasurement> channels{}; /**< 各源探测状态与量测 */
  double confidence{0.0};                 /**< 融合置信度（滑窗内 Σ 判决值 × 质量 × 权重） */
  std::uint64_t last_update_cycle{0U};    /**< 最近一次收到量测的周期号 */
  FusedTrackLifecycle lifecycle{FusedTrackLifecycle::kTentative}; /**< 航迹生命周期。 */
  bool has_kinematic_estimate{false};     /**< 是否携带运动学估计（滤波未启用/未起始/坐标回写失败时为 false）。 */
  FusedKinematicEstimate kinematic_estimate{}; /**< 运动学估计（ECEF 滤波后验）。 */
};

}  // namespace fusion

#endif  // ONEQ_FUSION_FUSED_TARGET_H_
