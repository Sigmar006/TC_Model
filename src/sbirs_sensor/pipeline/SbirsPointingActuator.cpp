/**
 * ## 过程：指向执行机构（限速转动 + 稳定等待）（红外处理链 · 第 10 步）
 * - 上游：`SbirsPointingCoordinator`（第 12 步）逐拍传入的指令光轴单位向量与
 *   `SbirsPointingActuatorConfig`（由 mission 的 `narrow_pointing_max_slew_rate_deg_per_sec`
 *   与 `narrow_pointing_settle_tolerance_deg` 组装）。
 * - 本步：单镜筒 ATP 的伺服特性建模：
 *   · 本步可达角 = 最大转速 × dt（限制的是角速度，不是位置跳变）；
 *   · 一步转不到位时沿最短球面路径（大圆）前进一个 max_step，转轴取当前×指令；
 *   · 剩余角 ≤ 稳定容差 ⇒ settled；本周期稳定时长 = max(0, dt − (夹角−容差)/转速)，
 *     供帧数折算（转动期间该周期的帧作废）。
 * - 下游：coordinator 的捕获/跟踪推进结果；pipeline 用稳定时长折算可积分帧数、用剩余角生成
 *   "指向未稳定/已捕获跟踪"候选状态与第 76 项接口记录。
 * - 关键参数：`max_slew_rate_deg_per_sec`(30°/s)、`settle_tolerance_deg`；均为 mission 配置，
 *   经 coordinator 逐拍传入，不是本类的成员状态。
 * - 对应验收项：第 13 项 窄视场跟踪探测功能测试（稳定后才允许出帧）；第 14 项 宽窄视场联合
 *   探测功能测试（镜头转向时间）；第 76 项 红外宽场候选与窄场调度交互接口测试。
 */

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

// 过程：两单位向量张角 = acos(点积)，点积先钳到 [−1,1]（浮点误差可使它略越界，acos 会返回 NaN）。
double AngularDistanceRad(const session::SbirsVector3M& left, const session::SbirsVector3M& right) {
  return std::acos(std::max(-1.0, std::min(1.0, Dot(left, right))));
}

// 过程：两方向共线时叉积为零、无法定转轴 —— 取分量绝对值最小的那根坐标轴做参考
//       （它与光轴最不平行），叉乘后必得有效转轴。
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

// 过程：罗德里格斯旋转 v' = v·cosθ + (k×v)·sinθ + k(k·v)(1−cosθ)，k 为单位转轴，模长不变。
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

// 过程：初始光轴归一化后同时作为当前与指令，并置为已稳定 —— 起点无转角，首拍即可出帧。
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

  // 过程：入参在此全部校验（归一化失败、dt/转速/容差非法即拒绝且不改状态）；
  //       通过后按本步可达角 = 转速·dt 推进。
  const double angle_rad = AngularDistanceRad(state_.current_los, normalized_command);
  const double angle_deg = angle_rad * 180.0 / kPi;
  const double max_step_rad = config.max_slew_rate_deg_per_sec * dt_sec * kPi / 180.0;
  session::SbirsVector3M next_los = normalized_command;
  // 过程：一步转不到位才需限速 —— 沿最短球面路径只转 max_step，转轴取当前×指令；
  //       否则直接到位（等价于舵机足够快，不发生位置跳变）。
  if (angle_rad > max_step_rad) {
    session::SbirsVector3M axis;
    if (!Normalize(Cross(state_.current_los, normalized_command), &axis)) {
      axis = ChooseAntipodalAxis(state_.current_los);
    }
    next_los = RotateAroundAxis(state_.current_los, axis, max_step_rad);
    Normalize(next_los, &next_los);
  }
  // 过程：剩余角按下一步位置到指令重算（而不是用原夹角减步长），即落点与指令的真实角距。
  const double remaining_angle_deg = AngularDistanceRad(next_los, normalized_command) * 180.0 / kPi;

  // 单镜筒分时轮转（2026-09-02）：本步稳定时长 = 周期内"进入稳定容差后"的剩余时间。
  // 转动耗时按 (夹角 − 容差)/转速 计，进入容差后的微调视作已稳定可积分；全程转不完
  // 则稳定时长为 0（该周期帧作废）。
  const double slew_sec =
      std::max(0.0, (angle_deg - static_cast<double>(config.settle_tolerance_deg)) /
                        static_cast<double>(config.max_slew_rate_deg_per_sec));
  const double settled_duration_sec = std::max(0.0, dt_sec - slew_sec);

  // 过程：状态提交 —— 即使未稳定也把 current_los 推进到本步位置（舵机不会瞬移），
  //       settled 由剩余角与容差比较得出。
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

// 过程：快照恢复只接受可归一化的向量；未初始化的快照等价于从未指向，回到默认空状态。
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
