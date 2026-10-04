/**
 * @file TrackStateSnapshot.h
 * @brief 定义供决策层消费的稳定轨迹快照。
 */

#ifndef ONEQ_AIRBORNE_RADAR_MODEL_TRACK_STATE_SNAPSHOT_H_
#define ONEQ_AIRBORNE_RADAR_MODEL_TRACK_STATE_SNAPSHOT_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief TrackStatus 表示对外可见的轨迹生命周期状态。
 */
enum class ONEQ_API TrackStatus {
  kTentative = 0, /**< 候选轨迹 */
  kConfirmed,     /**< 已确认轨迹 */
  kLost           /**< 丢失轨迹 */
};

/**
 * @brief TrackStateSnapshot 表示跨周期稳定的轨迹状态快照。
 */
struct ONEQ_API TrackStateSnapshot {
  std::uint64_t association_key{0};    /**< 当前快照对应的关联键 */
  std::uint64_t external_target_id{0}; /**< DEPRECATED（sim-only，注册遗留）：场景真值目标 ID（0 表示未知/未提供）。仿真真值归属的权威路径是信封通道 ArCycleResult.track_attributions（session_contract.md Attribution 挂载表）；新代码不得以本字段为关联依据，回收由后续独立工作处理 */
  std::string target_name{};           /**< DEPRECATED（sim-only，注册遗留）：场景真值目标名，仅用于人读、trace 与调试视图；权威归属见 ArCycleResult.track_attributions */
  TrackStatus status{TrackStatus::kTentative}; /**< 轨迹生命周期状态 */

  float position_x{0.0f}; /**< 雷达局部笛卡尔坐标 x（单位：m；平台 ENU 切平面东向分量，含平台姿态旋转） */
  float position_y{0.0f}; /**< 雷达局部笛卡尔坐标 y（单位：m；平台 ENU 切平面北向分量，含平台姿态旋转） */
  float position_z{0.0f}; /**< 雷达局部笛卡尔坐标 z（单位：m；平台 ENU 切平面上向分量，含平台姿态旋转） */

  float velocity_x{0.0f}; /**< 目标速度向量 x 分量（单位：m/s） */
  float velocity_y{0.0f}; /**< 目标速度向量 y 分量（单位：m/s） */
  float velocity_z{0.0f}; /**< 目标速度向量 z 分量（单位：m/s） */

  float speed{0.0f}; /**< 速度模长（单位：m/s） */

  float acceleration_x{0.0f}; /**< 目标加速度向量 x 分量（单位：m/s^2） */
  float acceleration_y{0.0f}; /**< 目标加速度向量 y 分量（单位：m/s^2） */
  float acceleration_z{0.0f}; /**< 目标加速度向量 z 分量（单位：m/s^2） */

  float acceleration{0.0f}; /**< 加速度模长（单位：m/s^2） */

  float rcs{0.0f}; /**< 目标估计雷达散射截面积（单位：平方米） */

  std::uint32_t hit_count{0};   /**< 命中累计计数 */
  std::uint32_t miss_count{0};  /**< 连续失配计数 */

  /** @brief 决策层填充的目标分类类型（"UNKNOWN"/"LOW_THREAT_TARGET"/"HIGH_THREAT_FIGHTER" 等） */
  std::string target_type = "UNKNOWN";
  /** @brief 决策层填充的目标分类置信度，范围 [0, 1] */
  float target_probability{0.0f};

  /**
   * @brief 航迹估计不确定度（m²），预测协方差 P 的 position 分块迹。
   * @note 由轨迹快照导出器从滤波协方差左上 3×3 分块求迹填充；
   *       作为识别运动质量因子的本源信号（比单周期 innovation covariance 更稳定）。
   */
  float estimation_uncertainty_trace{0.0f};
};

/** @brief TrackStateSnapshotList 表示供外部消费的轨迹状态快照集合 */
using TrackStateSnapshotList = std::vector<TrackStateSnapshot>;

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_MODEL_TRACK_STATE_SNAPSHOT_H_
