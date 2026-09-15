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
