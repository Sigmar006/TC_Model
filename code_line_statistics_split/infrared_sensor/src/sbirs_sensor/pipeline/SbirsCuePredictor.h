/**
 * @file SbirsCuePredictor.h
 * @brief Internal measurement-derived angular constant-velocity cue predictor.
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_CUE_PREDICTOR_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_CUE_PREDICTOR_H_

#include <cstdint>
#include <map>

namespace sbirs_sensor {
namespace pipeline {

/** @brief Last valid WFOV bearing retained for one target. */
struct SbirsCuePredictorTargetState {
  float measured_azimuth_deg{0.0f};
  float measured_elevation_deg{0.0f};
};

/** @brief Persisted per-target cue-predictor state. */
struct SbirsCuePredictorSnapshot {
  std::map<std::uint64_t, SbirsCuePredictorTargetState> targets{};
};

/** @brief Result of one measurement-derived cue prediction. */
struct SbirsCuePrediction {
  float command_azimuth_deg{0.0f};
  float command_elevation_deg{0.0f};
  float angular_rate_azimuth_deg_per_sec{0.0f};
  float angular_rate_elevation_deg_per_sec{0.0f};
  bool used_motion_prediction{false};
};

/**
 * @brief Predicts a future NFOV command from consecutive noisy WFOV bearings.
 * @note The first sample, non-positive/non-finite dt, and zero latency fall back to the current
 *       measurement. Target truth position and velocity are never consumed.
 */
class SbirsCuePredictor {
 public:
  SbirsCuePrediction Update(std::uint64_t target_id, float measured_azimuth_deg,
                            float measured_elevation_deg, float dt_sec, float cue_latency_s);
  void Release(std::uint64_t target_id);
  void Clear();
  SbirsCuePredictorSnapshot Capture() const;
  void Restore(const SbirsCuePredictorSnapshot& snapshot);

 private:
  std::map<std::uint64_t, SbirsCuePredictorTargetState> targets_{};
};

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_CUE_PREDICTOR_H_
