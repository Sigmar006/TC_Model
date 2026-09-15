/**
 * @file RirTrackAssociator.h
 * @brief RIR 轻量跟踪子集的门限 + LAPJV 全局最优检测-航迹关联器（阶段 2-T T2，
 *        N1/N2 升级为全局最优指派）。
 *
 * 副本来源：`src/airborne_radar/signal/association/DataAssociation.*` /
 * `DistanceMetric.*` / `LapjvSolver.*` 子集（审计基线 96de367c）。
 * 刻意不迁：欺骗候选关联、假设生成器可注入度量、逐航迹外部 seed 模式切换。
 * 关联采用方阵代价矩阵 + LAPJV 全局最优指派（D-A4 边界突破）：波门内代价
 * 入矩阵，门外对填拒绝代价，未分配行/列填未分配代价；波门按每对"预测协方差
 * 投影 + 动态量测 R"定标。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_ASSOCIATOR_H_
#define REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_ASSOCIATOR_H_

#include <cstdint>
#include <vector>

#include "common/estimation/KalmanPredictor.h"
#include "remote_identification_radar/tracking/RirLapjvSolver.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace tracking {

/**
 * @brief RirAssociationConfig 全局最优关联配置。
 */
struct RirAssociationConfig {
  /**
   * @brief 波门阈值：归一化新息平方（马氏距离平方）上限；缺省 9 对应 3σ 门。
   * @note 同时作为 LAPJV 未分配代价（policy `distance_gate_sigma` 的 σ→σ² 映射，
   *       与 AR `AssociationConfig` 的 `unassigned_cost = sigma²` 口径一致）。
   */
  float gate_threshold{9.0f};
  /** @brief 门控先验预测的过程噪声扩散系数 q（m/s²）。 */
  float kalman_noise_diff_coeff{1.0f};
  /** @brief 缺省量测噪声标准差（m），仅在量测 R 不可用时构造对角 R。 */
  float default_measurement_noise_std{10.0f};
  /** @brief 关联键起始值；0 为保留值。 */
  std::uint64_t initial_next_key{1U};
};

/**
 * @brief RirAssociationMatch 一次成功命中既有航迹的关联结果。
 */
struct RirAssociationMatch {
  std::uint64_t association_key{0U}; /**< 稳定关联键。 */
  std::size_t source_index{0U};      /**< 量测原始输入索引。 */
  float cost{0.0f};                  /**< 马氏距离平方代价。 */

  RirAssociationMatch() = default;
  RirAssociationMatch(std::uint64_t association_key_in, std::size_t source_index_in, float cost_in)
      : association_key(association_key_in), source_index(source_index_in), cost(cost_in) {}
};

/**
 * @brief RirAssociationResult 一次关联计算的完整输出。
 */
struct RirAssociationResult {
  std::vector<RirTrackMeasurement> measurements; /**< 有效量测；key/flags 已回填。 */
  std::vector<RirAssociationMatch> matches;      /**< 命中既有航迹的关联。 */
  std::vector<std::uint64_t> missed_track_keys;  /**< 本周期未命中量测的航迹键。 */
};

/**
 * @brief RirAssociationRuntimeState 关联器运行态快照。
 */
struct RirAssociationRuntimeState {
  std::uint64_t next_key{1U}; /**< 下一次新航迹要分配的关联键。 */
};

/**
 * @brief RirTrackAssociator 门限 + LAPJV 全局最优关联器。
 *
 * 关联键由本类单调分配且不回收复用：生命周期回收只删除内部航迹，不会把
 * 旧键放回分配池。因此"键重分配 = 新目标"语义在自持链路内天然成立，
 * 识别积累无需再检测 `hit_count` 回落。
 */
class RirTrackAssociator {
 public:
  /** @brief 构造关联器。 */
  explicit RirTrackAssociator(RirAssociationConfig config = {});

  /**
   * @brief 对检测量测与既有航迹种子执行门限 + 全局最优关联。
   * @param[in] measurements 本周期检测量测（association_key 初始为 0）。
   * @param[in] seeds 既有航迹种子（由 RirTrackLifecycle 导出）。
   * @param[in] dt_sec 周期步长（s）；非正或非有限时按 0 处理（只门控不外推）。
   * @return 关联结果；无效量测被剔除，新量测分配单调递增键。
   */
  RirAssociationResult Associate(const std::vector<RirTrackMeasurement>& measurements,
                                 const std::vector<RirTrackSeed>& seeds, float dt_sec);

  /** @brief 全量更新关联配置（不重置 next_key）。 */
  void UpdateConfig(RirAssociationConfig config);

  /** @brief 重置关联键分配器到 `initial_next_key`。 */
  void Reset();

  /** @brief 捕获关联键分配运行态（replay/回滚边界）。 */
  RirAssociationRuntimeState CaptureRuntimeState() const;

  /** @brief 恢复关联键分配运行态。 */
  void RestoreRuntimeState(const RirAssociationRuntimeState& state);

 private:
  bool IsUsableMeasurement(const RirTrackMeasurement& measurement) const;
  RirMeasurementCovariance ResolveMeasurementCovariance(
      const RirMeasurementCovariance& covariance) const;
  RirGaussianState PredictSeed(const RirTrackSeed& seed, float dt_sec) const;

  RirAssociationConfig config_{};
  ::oneq::common::estimation::KalmanPredictor<6, 3> predictor_;
  RirLapjvSolver assignment_solver_{};
  std::uint64_t next_key_{1U};
};

}  // namespace tracking
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_ASSOCIATOR_H_
