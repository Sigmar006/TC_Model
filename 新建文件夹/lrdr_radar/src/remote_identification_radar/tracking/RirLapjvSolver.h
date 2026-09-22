/*
 * Supplemental architecture notes for acceptance review.
 * Unit: RirLapjvSolver.
 * Model: LRDR remote identification radar.
 * Layer: tracking.
 * File role: interface declarations and data contracts.
 * Responsibility: measurement association, filtering, and track lifecycle management.
 * Upstream boundary: accepted detections, measurement covariance, and prior track seeds.
 * Downstream boundary: ordered track snapshots consumed by recognition and output assembly.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

/**
 * @file RirLapjvSolver.h
 * @brief RIR LAPJV 指派求解器薄适配层（common 单源，阶段 2-T N1）。
 *
 * 数值实现位于 `oneq::common::optimization::LapjvSolver`。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_LAPJV_SOLVER_H_
#define REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_LAPJV_SOLVER_H_

#include <Eigen/Core>
#include <vector>

namespace remote_identification_radar {
namespace tracking {

/**
 * @brief RirLapjvSolver 转发 common LAPJV 指派求解器。
 * @details 供关联引擎做全局最优检测-航迹分配（替代最近邻贪心）。
 */
class RirLapjvSolver final {
 public:
  /**
   * @brief 求解方阵代价矩阵上的指派关系。
   * @param cost_matrix 稠密方阵代价矩阵。
   * @return 行索引到列索引的映射；非方阵或空矩阵返回空解。
   */
  std::vector<int> Solve(const Eigen::Ref<const Eigen::MatrixXf>& cost_matrix) const;
};

}  // namespace tracking
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_LAPJV_SOLVER_H_
