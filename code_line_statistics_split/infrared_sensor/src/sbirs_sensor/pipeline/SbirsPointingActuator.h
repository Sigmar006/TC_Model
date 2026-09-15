/**
 * @file SbirsPointingActuator.h
 * @brief Internal rate-limited optical line-of-sight actuator characterization model.
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_POINTING_ACTUATOR_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_POINTING_ACTUATOR_H_

#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace sbirs_sensor {
namespace pipeline {

/** @brief Internal-only ATP characterization parameters. */
struct SbirsPointingActuatorConfig {
  double max_slew_rate_deg_per_sec{0.0};
  double settle_tolerance_deg{0.0};
};

/** @brief Persisted optical pointing state. */
struct SbirsPointingActuatorSnapshot {
  session::SbirsVector3M current_los{};
  session::SbirsVector3M command_los{};
  bool initialized{false};
  bool settled{false};
};

/** @brief Result of one rate-limited pointing step. */
struct SbirsPointingActuatorResult {
  session::SbirsVector3M current_los{};
  double remaining_angle_deg{0.0};
  bool settled{false};
  double settled_duration_sec{0.0};  /**< 本步内到达指令角后剩余的稳定时长（转动期间为 0） */
};

/**
 * @brief Advances a unit optical LOS toward a command on the shortest spherical path.
 * @note This primitive is not wired to SBIRS session or NFOV scheduling.
 */
class SbirsPointingActuator {
 public:
  bool Initialize(const session::SbirsVector3M& initial_los);
  bool Step(const session::SbirsVector3M& command_los, double dt_sec,
            const SbirsPointingActuatorConfig& config, SbirsPointingActuatorResult* result);
  SbirsPointingActuatorSnapshot Capture() const;
  bool Restore(const SbirsPointingActuatorSnapshot& snapshot);

 private:
  SbirsPointingActuatorSnapshot state_{};
};

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_POINTING_ACTUATOR_H_
