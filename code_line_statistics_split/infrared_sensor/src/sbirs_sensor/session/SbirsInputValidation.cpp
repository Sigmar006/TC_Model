#include "1q/sbirs_sensor/session/SbirsInputValidation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsIssueCodes.h"
#include "common/validation/ValidationUtils.h"

namespace sbirs_sensor {
namespace session {
namespace {

bool Finite(double value) { return std::isfinite(value); }

bool FiniteVector(const SbirsVector3M& value) {
  return Finite(value.x) && Finite(value.y) && Finite(value.z);
}

bool NonZeroVector(const SbirsVector3M& value) {
  return value.x != 0.0 || value.y != 0.0 || value.z != 0.0;
}

bool ZeroVector(const SbirsVector3M& value) {
  return value.x == 0.0 && value.y == 0.0 && value.z == 0.0;
}

// 统一问题列表模型（规则 14）：校验问题 code 引用 SbirsIssueCodes.h 常量
// （"sbirs.validation.<snake_case>" 全集单一事实来源）；phase 固定为
// kInputValidation；location/field 为可选定位。
void AddError(const char* code, const char* field, const char* message,
              oneq::foundation::ValidationLocation location, SbirsIssueList* issues) {
  SbirsIssue issue;
  issue.severity = SbirsIssueSeverity::kError;
  issue.phase = SbirsIssuePhase::kInputValidation;
  issue.code = code;
  issue.message = message;
  issue.location = location;
  issue.field = field;
  issues->push_back(issue);
}

}  // namespace

SbirsIssueList ValidateSbirsCycleInput(const SbirsCycleInput& input, float frame_rate_hz) {
  SbirsIssueList issues;
  if (input.dt_sec <= 0.0f || !std::isfinite(input.dt_sec)) {
    oneq::foundation::ValidationLocation location;
    location.kind = oneq::foundation::ValidationLocationKind::kGlobal;
    AddError(codes::kInvalidCycleDeltaTime, "dt_sec", "dt_sec must be positive and finite", location,
             &issues);
  } else {
    constexpr float kMaxDtFactor = 10.0f;
    const float max_dt_sec =
        kMaxDtFactor / std::max(frame_rate_hz, std::numeric_limits<float>::min());
    if (input.dt_sec > max_dt_sec) {
      oneq::foundation::ValidationLocation location;
      location.kind = oneq::foundation::ValidationLocationKind::kGlobal;
      AddError(codes::kCycleDeltaTimeExceedsFramePeriod, "dt_sec",
               "dt_sec exceeds reasonable range based on frame_rate_hz", location, &issues);
    }
  }
  // ECI 输出参考系必需：UTC 儒略日缺失（默认 0）/非有限/非正 → 校验拒绝。
  if (!std::isfinite(input.utc_julian_day) || input.utc_julian_day <= 0.0) {
    oneq::foundation::ValidationLocation location;
    location.kind = oneq::foundation::ValidationLocationKind::kGlobal;
    AddError(codes::kInvalidUtcJulianDay, "utc_julian_day",
             "utc_julian_day must be provided, finite, and positive", location, &issues);
  }
  if (!FiniteVector(input.satellite_position_ecef_m) ||
      !NonZeroVector(input.satellite_position_ecef_m)) {
    oneq::foundation::ValidationLocation location;
    location.kind = oneq::foundation::ValidationLocationKind::kPlatform;
    AddError(codes::kInvalidSatellitePosition, "satellite_position_ecef_m",
             "satellite position must be provided, finite, and non-zero", location, &issues);
  }
  // 卫星速度必填（与位置同等待遇）：非有限即拒绝；零向量合法（如 GEO 卫星）。
  // 速度旋入 ECI 后与目标速度合成相对速度，静止卫星会把动态滞后低估为仅目标运动。
  if (!FiniteVector(input.satellite_velocity_ecef_m_per_s)) {
    oneq::foundation::ValidationLocation location;
    location.kind = oneq::foundation::ValidationLocationKind::kPlatform;
    AddError(codes::kInvalidSatelliteVelocity, "satellite_velocity_ecef_m_per_s",
             "satellite velocity must be provided and finite (zero vector is valid)", location,
             &issues);
  }
  // 卫星姿态必填（阶段 2 指向合成链）：非有限即拒绝；零欧拉合法（体轴对齐 ECI）。
  // 无姿态会把卫星隐含为无姿态质点，安装角/姿态无法驱动内部光轴几何。
  if (!Finite(input.satellite_attitude_eci_body_deg.yaw_deg) ||
      !Finite(input.satellite_attitude_eci_body_deg.pitch_deg) ||
      !Finite(input.satellite_attitude_eci_body_deg.roll_deg)) {
    oneq::foundation::ValidationLocation location;
    location.kind = oneq::foundation::ValidationLocationKind::kPlatform;
    AddError(codes::kInvalidSatelliteAttitude, "satellite_attitude_eci_body_deg",
             "satellite attitude must be provided and finite (zero euler is valid)", location,
             &issues);
  }
  std::set<std::uint64_t> target_ids;
  for (std::size_t i = 0; i < input.scene.size(); ++i) {
    const SbirsSceneTarget& target = input.scene[i];
    const bool valid_velocity =
        FiniteVector(target.velocity_ecef_m_per_s) &&
        (target.has_velocity_ecef_m_per_s || ZeroVector(target.velocity_ecef_m_per_s));
    if (target.target_id == 0U || !target_ids.insert(target.target_id).second ||
        !FiniteVector(target.position_ecef_m) || !NonZeroVector(target.position_ecef_m) ||
        !std::isfinite(target.radiant_intensity_w_per_sr) ||
        target.radiant_intensity_w_per_sr < 0.0 || !valid_velocity) {
      oneq::foundation::ValidationLocation location;
      location.kind = oneq::foundation::ValidationLocationKind::kSceneEntity;
      location.entity_index = i;
      AddError(codes::kInvalidTargetPhysical, "scene",
               "target physical inputs must be finite; radiant intensity must be non-negative",
               location, &issues);
    }
  }
  return issues;
}

bool HasValidationError(const SbirsIssueList& issues) {
  // RED-1 收敛：统一判定逻辑在 common::validation::HasValidationPhaseError
  // （phase == kInputValidation && severity == kError）。
  return oneq::common::validation::HasValidationPhaseError(
      issues, &SbirsIssue::phase, &SbirsIssue::severity);
}

}  // namespace session
}  // namespace sbirs_sensor
