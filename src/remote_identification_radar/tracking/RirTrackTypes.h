/**
 * @file RirTrackTypes.h
 * @brief RIR 轻量跟踪子集的内部航迹类型（阶段 2-T T4）。
 *
 * 字段集合对齐识别消费闭包（`association_key`/`status`/`hit_count`/
 * 位置/速度/加速度/`estimation_uncertainty_trace`），并保留
 * `external_target_id`/`target_name` 供场景目标回联与真值准确率统计。
 * 该类型是阶段 1 供给航迹语义的内部化身：阶段 2-T 旁路新增，阶段 2-S
 * 起由 `RirTrackLifecycle`/`RirTrackFilter` 生产并被识别链路直接消费。
 *
 * 副本来源：`src/airborne_radar/signal/tracking/TrackState.h` 与
 * `TrackLifecycleTypes.h` 子集（审计基线 96de367c，阶段 2-T）。
 * 刻意差异：无 `TrackStatus::kRecycled` 中间态（航迹回收即从内部表删除并
 * 归还对象池，池化与 `generation` 复用代次语义自 N3 起与 AR 对齐）；
 * 关联键由 `RirTrackAssociator` 单调分配，键重分配天然等于新目标，
 * 无需 `hit_count` 回落检测。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_TYPES_H_
#define REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_TYPES_H_

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "common/estimation/GaussianState.h"

namespace remote_identification_radar {
namespace tracking {

/** @brief RIR 3D 恒速高斯状态（[x, vx, y, vy, z, vz] / [x, y, z]）。 */
using RirGaussianState = ::oneq::common::estimation::GaussianState<6, 3>;
using RirStateVector = RirGaussianState::StateVector;
using RirStateCovariance = RirGaussianState::StateCovariance;
using RirMeasurementVector = RirGaussianState::MeasurementVector;
using RirMeasurementCovariance = RirGaussianState::MeasurementCovariance;
using RirTransitionMatrix = RirGaussianState::TransitionMatrix;
using RirProcessNoiseCovariance = RirGaussianState::ProcessNoiseCovariance;
using RirMeasurementMatrix = RirGaussianState::MeasurementMatrix;
using RirKalmanGainMatrix = RirGaussianState::KalmanGainMatrix;

/** @brief RIR 内部航迹生命周期状态（镜像识别消费语义）。 */
enum class RirTrackStatus {
  kTentative = 0, /**< 候选航迹，尚未达到确认阈值。 */
  kConfirmed,     /**< 已确认航迹，可参与识别积累。 */
  kLost           /**< 丢失航迹，等待短时重捕获。 */
};

/**
 * @brief RirTrackMeasurement 轻量关联/生命周期消费的单周期量测。
 *
 * `position` 为检测量测位置（已施加量测误差）；`velocity` 为**该雷达自身量测序列的
 * 两点起批差分速度**（本目标上周量测位置到本周量测位置、除以实际时间间隔；
 * 首次检测/间隔非法时为零），只用于新航迹/失配重捕获时的 KF 状态初始化，
 * 不进入量测更新方程。场景真值速度**不得**填入该字段
 * （rir-tracking-realism 契约修订 1 第 4 条：真值速度种子属泄漏）。
 */
struct RirTrackMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t source_index{0U};                      /**< 输入目标索引（调试/回联）。 */
  std::uint64_t external_target_id{0U};              /**< 场景目标原始标识；0 表示未知。 */
  std::string target_name{};                         /**< 可选目标名称（真值统计/人读）。 */
  std::uint64_t association_key{0U};                 /**< 关联键；0 为保留值（未关联）。 */
  bool matched_existing_track{false};                /**< 是否命中既有航迹。 */
  Eigen::Vector3f position{Eigen::Vector3f::Zero()}; /**< 量测位置（m）。 */
  Eigen::Vector3f velocity{Eigen::Vector3f::Zero()}; /**< 两点起批速度种子（m/s；首次检测为零）。 */
  float detection_pd{0.0f};   /**< 本量测的检测概率 Pd（检测链透传；验收旁路用，不进关联方程）。 */
  float detection_snr_db{0.0f}; /**< 本量测对应的检测 SNR（dB，外部验收投影用）。 */
  float radial_velocity_m_per_s{0.0f}; /**< 检测链使用的径向速度（m/s，外部验收投影用）。 */
  float rcs{0.0f};                                   /**< 目标估计 RCS（m²）。 */
  RirMeasurementCovariance measurement_covariance{
      RirMeasurementCovariance::Zero()}; /**< 笛卡尔量测噪声协方差 R（m²）。 */
};

/**
 * @brief RirTrackState 内部航迹状态。
 *
 * 运动学位置/速度与高斯状态同源：从 `gaussian_state.mean` 按
 * [x, vx, y, vy, z, vz] 布局回写。加速度为滤波后验速度的周期间差分
 * （物理加速度口径）：hit 时以上周期后验速度为基准、按距上次命中的实际
 * 经过时间折算（滑行周期后重命中不膨胀），(重)置周期（建轨/失跟重捕）
 * 无差分基准、显式置零，miss 时 CV 外推保持速度、加速度归零。
 */
struct RirTrackState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint64_t association_key{0U};                     /**< 跨周期稳定关联键。 */
  std::uint64_t track_id{0U};                            /**< 内部航迹编号（单调递增）。 */
  std::uint64_t batch_id{0U};                            /**< 首次建轨所在批号。 */
  std::uint64_t external_target_id{0U};                  /**< 最近已知场景目标标识；0 表示未知。 */
  std::string target_name{};                             /**< 最近已知目标名称。 */
  RirTrackStatus status{RirTrackStatus::kTentative};     /**< 生命周期状态。 */
  std::uint32_t generation{0U};                          /**< 对象池复用代次，识别已回收槽位的旧引用。 */
  std::uint32_t first_cycle{0U};                         /**< 首次建轨周期号。 */
  std::uint32_t last_update_cycle{0U};                   /**< 最近命中周期号。 */
  std::uint32_t miss_count{0U};                          /**< 连续失配计数。 */
  std::uint32_t hit_count{0U};                           /**< 累计命中计数。 */
  Eigen::Vector3f position{Eigen::Vector3f::Zero()};     /**< 滤波位置（m）。 */
  Eigen::Vector3f velocity{Eigen::Vector3f::Zero()};     /**< 滤波速度（m/s）。 */
  Eigen::Vector3f acceleration{Eigen::Vector3f::Zero()}; /**< 滤波加速度（m/s²）。 */
  float speed{0.0f};               /**< 速度模长（m/s），识别运动特征直接消费。 */
  float acceleration_mps2{0.0f};   /**< 加速度模长（m/s²），识别运动特征直接消费。 */
  float rcs{0.0f};                 /**< 最近已知 RCS（m²）。 */
  RirGaussianState gaussian_state; /**< KF 状态与协方差。 */

  /**
   * @brief 识别运动质量因子的本源信号：协方差 P 的 position 分块迹（m²）。
   * @details 状态序 [x, vx, y, vy, z, vz]，位置方差位于 (0,0)/(2,2)/(4,4)。
   */
  float EstimationUncertaintyTrace() const {
    const RirStateCovariance& covariance = gaussian_state.covariance;
    return covariance(0, 0) + covariance(2, 2) + covariance(4, 4);
  }
};

/**
 * @brief RirTrackSeed 关联阶段的既有航迹种子。
 */
struct RirTrackSeed {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint64_t association_key{0U};                 /**< 关联键。 */
  bool has_position{false};                          /**< 是否具备有效位置。 */
  Eigen::Vector3f position{Eigen::Vector3f::Zero()}; /**< 上一周期位置。 */
  bool has_gaussian_state{false};                    /**< 是否具备高斯状态。 */
  RirGaussianState gaussian_state;                   /**< 用于预测门控的高斯状态。 */
};

/** @brief RIR 内部航迹快照列表（按关联键升序）。 */
using RirTrackSnapshotList = std::vector<RirTrackState>;

}  // namespace tracking
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_TYPES_H_
