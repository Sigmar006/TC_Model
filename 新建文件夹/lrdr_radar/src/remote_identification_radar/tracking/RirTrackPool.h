/*
 * Supplemental architecture notes for acceptance review.
 * Unit: RirTrackPool.
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
 * @file RirTrackPool.h
 * @brief RIR 航迹对象池（common ObjectPool 薄适配）。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_POOL_H_
#define REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_POOL_H_

#include <cstddef>

#include "common/tracking/ObjectPool.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace tracking {

/**
 * @brief RirTrackPool 轻量航迹对象池（common ObjectPool 薄适配）。
 */
class RirTrackPool final {
 public:
  explicit RirTrackPool(std::size_t prewarm_count = 128, std::size_t max_cached_objects = 4096);

  RirTrackState* Acquire();
  void Release(RirTrackState* track);
  std::size_t Capacity() const;
  std::size_t InUseCount() const;
  std::size_t FreeCount() const;

 private:
  oneq::common::tracking::ObjectPool<RirTrackState> pool_;
};

}  // namespace tracking
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_POOL_H_
