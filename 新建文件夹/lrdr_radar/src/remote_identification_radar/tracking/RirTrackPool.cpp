/*
 * Supplemental architecture notes for acceptance review.
 * Unit: RirTrackPool.
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

#include "remote_identification_radar/tracking/RirTrackPool.h"

namespace remote_identification_radar {
namespace tracking {

RirTrackPool::RirTrackPool(std::size_t prewarm_count, std::size_t max_cached_objects)
    : pool_(prewarm_count, max_cached_objects, "RirTrackPool") {}

RirTrackState* RirTrackPool::Acquire() { return pool_.Acquire(); }

void RirTrackPool::Release(RirTrackState* track) { pool_.Release(track); }

std::size_t RirTrackPool::Capacity() const { return pool_.Capacity(); }

std::size_t RirTrackPool::InUseCount() const { return pool_.InUseCount(); }

std::size_t RirTrackPool::FreeCount() const { return pool_.FreeCount(); }

}  // namespace tracking
}  // namespace remote_identification_radar
