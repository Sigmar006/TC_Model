/**
 * @file InferenceTrackState.h
 * @brief 定义推演层泛型航迹输入帧（算法不感知传感器与坐标系来源）。
 */

#ifndef ONEQ_TARGET_INFERENCE_INFERENCE_TRACK_STATE_H_
#define ONEQ_TARGET_INFERENCE_INFERENCE_TRACK_STATE_H_

#include <array>
#include <cstdint>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace target_inference {

/**
 * @brief 推演层目标大类（泛型；与具体传感器识别枚举的映射归调用方）。
 * @note 取值加性扩展（不重排既有值）。
 */
enum class ONEQ_API InferenceTargetCategory : std::uint8_t {
  kBallistic = 0, /**< 弹道目标。 */
  kNearSpace = 1, /**< 临近空间目标。 */
  kFighter = 2,   /**< 战斗机。 */
  kBomber = 3,    /**< 轰炸机。 */
  kMissile = 4,   /**< 导弹。 */
  /* kUav = 5 已移除（2026-08-22 甲方裁定，产品不识别无人机）：槽位 5 保留
     不复用（加性扩展、不重排），先验恒零、永不为 argmax。 */
  kOther = 6,     /**< 其它。 */
  kUnknown = 7    /**< 未知。 */
};

/** @brief 推演层目标大类总数（枚举加性扩展约束下保持稳定语义）。 */
constexpr std::size_t kInferenceCategoryCount = 8U;

/**
 * @brief 单航迹推演输入帧（ECEF 状态 + 可选协方差 + 可选类型证据）。
 * @note 与 fusion::FusedTarget 的对接由调用方组装（值级传递，不引用 fusion 类型——
 *       分层契约规则 1）；协方差排列 [x,vx,y,vy,z,vz] 行主序（与估计层一致）。
 */
struct ONEQ_API InferenceTrackState {
  std::uint64_t key{0U};                  /**< 航迹库内键（透传，用于输出对齐）。 */
  double sim_time_sec{0.0};               /**< 采样仿真时间（单位：s；0 = 调用方未提供）。 */
  std::uint32_t input_cycle_index{0U};    /**< 采样周期号（0 = 调用方未提供）。 */
  oneq::coordinate::EcefPositionM position{}; /**< ECEF 位置（单位：m）。 */
  std::array<double, 3U> velocity_ecef_m_per_s = std::array<double, 3U>(); /**< ECEF 速度（单位：m/s）。 */
  bool has_covariance{false};             /**< 是否携带状态协方差（影响误差预算出口）。 */
  std::array<double, 36U> covariance_ecef{}; /**< 6×6 协方差（行主序 [x,vx,y,vy,z,vz]）。 */
  bool has_type_evidence{false};          /**< 是否携带外部类型证据（如识别结论）。 */
  std::array<double, kInferenceCategoryCount> type_evidence{}; /**< 按大类证据得分 [0,1]。 */
};

}  // namespace target_inference

#endif  // ONEQ_TARGET_INFERENCE_INFERENCE_TRACK_STATE_H_
