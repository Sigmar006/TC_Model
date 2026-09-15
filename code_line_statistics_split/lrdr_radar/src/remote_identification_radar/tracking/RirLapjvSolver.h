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
