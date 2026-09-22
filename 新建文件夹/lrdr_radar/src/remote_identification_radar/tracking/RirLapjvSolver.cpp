/*
 * Supplemental architecture notes for acceptance review.
 * Unit: RirLapjvSolver.
 * Model: LRDR remote identification radar.
 * Layer: tracking.
 * File role: executable implementation.
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
