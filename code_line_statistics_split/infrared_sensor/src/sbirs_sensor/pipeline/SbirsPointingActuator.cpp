#include "sbirs_sensor/pipeline/SbirsPointingActuator.h"

#include <algorithm>
#include <cmath>
#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

using oneq::common::numerics::kPi;
constexpr double kVectorEpsilon = 1.0e-12;

bool IsFinite(const session::SbirsVector3M& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

double Dot(const session::SbirsVector3M& left, const session::SbirsVector3M& right) {
  return left.x * right.x + left.y * right.y + left.z * right.z;
}

session::SbirsVector3M Cross(const session::SbirsVector3M& left,
                             const session::SbirsVector3M& right) {
  session::SbirsVector3M result;
  result.x = left.y * right.z - left.z * right.y;
  result.y = left.z * right.x - left.x * right.z;
  result.z = left.x * right.y - left.y * right.x;
  return result;
}

double Norm(const session::SbirsVector3M& value) { return std::sqrt(Dot(value, value)); }

bool Normalize(const session::SbirsVector3M& value, session::SbirsVector3M* normalized) {
  if (normalized == nullptr || !IsFinite(value)) {
    return false;
  }
  const double norm = Norm(value);
  if (!std::isfinite(norm) || norm <= kVectorEpsilon) {
    return false;
  }
  normalized->x = value.x / norm;
  normalized->y = value.y / norm;
  normalized->z = value.z / norm;
  return true;
}

double AngularDistanceRad(const session::SbirsVector3M& left, const session::SbirsVector3M& right) {
  return std::acos(std::max(-1.0, std::min(1.0, Dot(left, right))));
}

session::SbirsVector3M ChooseAntipodalAxis(const session::SbirsVector3M& current) {
  session::SbirsVector3M reference;
  const double abs_x = std::abs(current.x);
  const double abs_y = std::abs(current.y);
  const double abs_z = std::abs(current.z);
  if (abs_x <= abs_y && abs_x <= abs_z) {
    reference.x = 1.0;
  } else if (abs_y <= abs_z) {
    reference.y = 1.0;
  } else {
    reference.z = 1.0;
  }
  session::SbirsVector3M axis;
  Normalize(Cross(current, reference), &axis);
  return axis;
}

session::SbirsVector3M RotateAroundAxis(const session::SbirsVector3M& value,
                                        const session::SbirsVector3M& axis, double angle_rad) {
  const double cosine = std::cos(angle_rad);
  const double sine = std::sin(angle_rad);
  const session::SbirsVector3M cross = Cross(axis, value);
  const double projection = Dot(axis, value) * (1.0 - cosine);
  session::SbirsVector3M result;
  result.x = value.x * cosine + cross.x * sine + axis.x * projection;
  result.y = value.y * cosine + cross.y * sine + axis.y * projection;
  result.z = value.z * cosine + cross.z * sine + axis.z * projection;
  return result;
}

}  // namespace

bool SbirsPointingActuator::Initialize(const session::SbirsVector3M& initial_los) {
  session::SbirsVector3M normalized;
  if (!Normalize(initial_los, &normalized)) {
    return false;
  }
  state_.current_los = normalized;
  state_.command_los = normalized;
  state_.initialized = true;
  state_.settled = true;
  return true;
}

bool SbirsPointingActuator::Step(const session::SbirsVector3M& command_los, double dt_sec,
                                 const SbirsPointingActuatorConfig& config,
                                 SbirsPointingActuatorResult* result) {
  session::SbirsVector3M normalized_command;
  if (result == nullptr || !state_.initialized || !Normalize(command_los, &normalized_command) ||
      !std::isfinite(dt_sec) || dt_sec <= 0.0 || !std::isfinite(config.max_slew_rate_deg_per_sec) ||
      config.max_slew_rate_deg_per_sec <= 0.0 || !std::isfinite(config.settle_tolerance_deg) ||
      config.settle_tolerance_deg < 0.0) {
    return false;
  }

  const double angle_rad = AngularDistanceRad(state_.current_los, normalized_command);
  const double angle_deg = angle_rad * 180.0 / kPi;
  const double max_step_rad = config.max_slew_rate_deg_per_sec * dt_sec * kPi / 180.0;
  session::SbirsVector3M next_los = normalized_command;
  if (angle_rad > max_step_rad) {
    session::SbirsVector3M axis;
    if (!Normalize(Cross(state_.current_los, normalized_command), &axis)) {
      axis = ChooseAntipodalAxis(state_.current_los);
    }
    next_los = RotateAroundAxis(state_.current_los, axis, max_step_rad);
    Normalize(next_los, &next_los);
  }
  const double remaining_angle_deg = AngularDistanceRad(next_los, normalized_command) * 180.0 / kPi;

  // 单镜筒分时轮转（2026-09-02）：本步稳定时长 = 周期内"进入稳定容差后"的剩余时间。
  // 转动耗时按 (夹角 − 容差)/转速 计，进入容差后的微调视作已稳定可积分；全程转不完
  // 则稳定时长为 0（该周期帧作废）。
  const double slew_sec =
      std::max(0.0, (angle_deg - static_cast<double>(config.settle_tolerance_deg)) /
                        static_cast<double>(config.max_slew_rate_deg_per_sec));
  const double settled_duration_sec = std::max(0.0, dt_sec - slew_sec);

  SbirsPointingActuatorSnapshot next_state;
  next_state.current_los = next_los;
  next_state.command_los = normalized_command;
  next_state.initialized = true;
  next_state.settled = remaining_angle_deg <= config.settle_tolerance_deg;
  state_ = next_state;
  result->current_los = next_los;
  result->remaining_angle_deg = remaining_angle_deg;
  result->settled = next_state.settled;
  result->settled_duration_sec = settled_duration_sec;
  return true;
}

SbirsPointingActuatorSnapshot SbirsPointingActuator::Capture() const { return state_; }

bool SbirsPointingActuator::Restore(const SbirsPointingActuatorSnapshot& snapshot) {
  if (!snapshot.initialized) {
    state_ = SbirsPointingActuatorSnapshot{};
    return true;
  }
  session::SbirsVector3M current;
  session::SbirsVector3M command;
  if (!Normalize(snapshot.current_los, &current) || !Normalize(snapshot.command_los, &command)) {
    return false;
  }
  SbirsPointingActuatorSnapshot restored = snapshot;
  restored.current_los = current;
  restored.command_los = command;
  state_ = restored;
  return true;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
