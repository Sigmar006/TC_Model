#include "sbirs_sensor/pipeline/SbirsPointingCoordinator.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace sbirs_sensor {
namespace pipeline {

SbirsPointingCoordinator::SbirsPointingCoordinator(std::uint32_t disturbance_seed)
    : disturbance_(1, disturbance_seed) {}

bool SbirsPointingCoordinator::EnsureActuatorInitialized(
    const session::SbirsVector3M& initial_los) {
  if (actuator_initialized_) {
    return true;
  }
  if (!actuator_.Initialize(initial_los)) {
    return false;
  }
  actuator_initialized_ = true;
  return true;
}

SbirsPointingAdvanceResult SbirsPointingCoordinator::AdvanceAcquisition(
    std::uint64_t target_id, const session::SbirsVector3M& command_los, double dt_sec,
    const SbirsPointingActuatorConfig& config) {
  SbirsPointingAdvanceResult result;
  if (!actuator_initialized_) {
    return result;
  }
  SbirsPointingActuatorResult actuator_result;
  if (!actuator_.Step(command_los, dt_sec, config, &actuator_result)) {
    return result;
  }

  const double next_elapsed = acquisition_wait_sec_[target_id] + dt_sec;
  acquisition_wait_sec_[target_id] = next_elapsed;
  result.current_los = actuator_result.current_los;
  result.remaining_angle_deg = actuator_result.remaining_angle_deg;
  result.settled_duration_sec = actuator_result.settled_duration_sec;
  result.elapsed_wait_sec = next_elapsed;
  if (actuator_result.settled) {
    result.status = SbirsPointingAdvanceStatus::kSettled;
    return result;
  }

  const double timeout_sec = 180.0 / config.max_slew_rate_deg_per_sec;
  if (next_elapsed >= timeout_sec) {
    acquisition_wait_sec_.erase(target_id);
    result.status = SbirsPointingAdvanceStatus::kTimedOut;
    return result;
  }
  result.status = SbirsPointingAdvanceStatus::kSlewing;
  return result;
}

SbirsPointingAdvanceResult SbirsPointingCoordinator::AdvanceTracking(
    const session::SbirsVector3M& command_los, double dt_sec,
    const SbirsPointingActuatorConfig& config) {
  SbirsPointingAdvanceResult result;
  if (!actuator_initialized_) {
    return result;
  }
  SbirsPointingActuatorResult actuator_result;
  if (!actuator_.Step(command_los, dt_sec, config, &actuator_result)) {
    return result;
  }
  result.current_los = actuator_result.current_los;
  result.remaining_angle_deg = actuator_result.remaining_angle_deg;
  result.settled_duration_sec = actuator_result.settled_duration_sec;
  result.elapsed_wait_sec = 0.0;
  result.status = actuator_result.settled ? SbirsPointingAdvanceStatus::kSettled
                                          : SbirsPointingAdvanceStatus::kSlewing;
  return result;
}

bool SbirsPointingCoordinator::PromoteToTracking(std::uint64_t target_id) {
  acquisition_wait_sec_.erase(target_id);
  return true;
}

unsigned int SbirsPointingCoordinator::RecordTrackingGateResult(std::uint64_t target_id,
                                                                bool gate_passed) {
  if (gate_passed) {
    tracking_gate_failure_counts_[target_id] = 0U;
  } else {
    ++tracking_gate_failure_counts_[target_id];
  }
  return tracking_gate_failure_counts_[target_id];
}

bool SbirsPointingCoordinator::AdvanceDisturbance(
    double dt_sec, const SbirsPointingDisturbanceParameters& parameters) {
  return disturbance_.Advance(dt_sec, parameters);
}

bool SbirsPointingCoordinator::DisturbanceSample(
    int channel_id, const SbirsPointingDisturbanceParameters& parameters,
    SbirsPointingDisturbanceSample* sample) const {
  return disturbance_.Sample(channel_id, parameters, sample);
}

void SbirsPointingCoordinator::RestartDisturbance(std::uint32_t disturbance_seed) {
  disturbance_ = SbirsPointingDisturbance(1, disturbance_seed);
}

bool SbirsPointingCoordinator::ReleaseTarget(std::uint64_t target_id) {
  const bool had_entries =
      acquisition_wait_sec_.erase(target_id) != 0U ||
      tracking_gate_failure_counts_.erase(target_id) != 0U;
  return had_entries;
}

void SbirsPointingCoordinator::ResetTrackingGateFailureCounts() {
  tracking_gate_failure_counts_.clear();
}

void SbirsPointingCoordinator::Clear() {
  const std::uint32_t seed = disturbance_.Capture().base_seed;
  actuator_ = SbirsPointingActuator{};
  actuator_initialized_ = false;
  acquisition_wait_sec_.clear();
  tracking_gate_failure_counts_.clear();
  disturbance_ = SbirsPointingDisturbance(1, seed);
}

SbirsPointingCoordinatorSnapshot SbirsPointingCoordinator::Capture() const {
  SbirsPointingCoordinatorSnapshot snapshot;
  snapshot.actuator_initialized = actuator_initialized_;
  snapshot.actuator = actuator_.Capture();
  snapshot.acquisition_wait_sec = acquisition_wait_sec_;
  snapshot.tracking_gate_failure_counts = tracking_gate_failure_counts_;
  snapshot.disturbance = disturbance_.Capture();
  return snapshot;
}

bool SbirsPointingCoordinator::Restore(const SbirsPointingCoordinatorSnapshot& snapshot) {
  SbirsPointingDisturbance restored_disturbance(1, 1U);
  if (!restored_disturbance.Restore(snapshot.disturbance)) {
    return false;
  }
  SbirsPointingActuator restored_actuator;
  if (!restored_actuator.Restore(snapshot.actuator)) {
    return false;
  }
  if (snapshot.actuator_initialized && !snapshot.actuator.initialized) {
    return false;
  }
  std::set<std::uint64_t> bookkeeping_targets;
  for (const auto& entry : snapshot.acquisition_wait_sec) {
    if (!std::isfinite(entry.second) || entry.second < 0.0 ||
        !bookkeeping_targets.insert(entry.first).second) {
      return false;
    }
  }
  for (const auto& entry : snapshot.tracking_gate_failure_counts) {
    if (!bookkeeping_targets.insert(entry.first).second) {
      return false;
    }
  }
  actuator_ = restored_actuator;
  actuator_initialized_ = snapshot.actuator_initialized;
  acquisition_wait_sec_ = snapshot.acquisition_wait_sec;
  tracking_gate_failure_counts_ = snapshot.tracking_gate_failure_counts;
  disturbance_ = restored_disturbance;
  return true;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
