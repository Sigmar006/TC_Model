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
