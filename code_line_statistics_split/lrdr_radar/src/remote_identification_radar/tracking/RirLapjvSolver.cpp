/**
 * @file RirLapjvSolver.cpp
 * @brief RIR LAPJV 指派求解器薄适配层（common 单源）。
 */

#include "remote_identification_radar/tracking/RirLapjvSolver.h"

#include "common/optimization/LapjvSolver.h"

namespace remote_identification_radar {
namespace tracking {

std::vector<int> RirLapjvSolver::Solve(
    const Eigen::Ref<const Eigen::MatrixXf>& cost_matrix) const {
  return oneq::common::optimization::LapjvSolver().Solve(cost_matrix);
}

}  // namespace tracking
}  // namespace remote_identification_radar
