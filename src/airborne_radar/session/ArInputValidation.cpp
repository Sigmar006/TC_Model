#include "1q/airborne_radar/session/ArInputValidation.h"

#include <cmath>
#include <sstream>
#include <unordered_map>

#include "1q/airborne_radar/session/ArIssueCodes.h"
#include "1q/airborne_radar/session/ArRadarFrameTransform.h"
#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "common/validation/ValidationUtils.h"

namespace airborne_radar {
namespace session {
namespace {

using oneq::common::validation::IsFinite;

// 统一问题列表模型（规则 14）：校验问题 code 引用 ArIssueCodes.h 常量
// （"ar.validation.<snake_case>" 全集单一事实来源）；phase 固定为
// kInputValidation；location/field 为可选定位。
ArIssue MakeIssue(ArIssueSeverity severity, const char* code,
                  oneq::foundation::ValidationLocationKind location_kind,
                  std::size_t entity_index, const std::string& field,
                  const std::string& message) {
  ArIssue issue = oneq::common::validation::MakeLocatedIssue<
      ArIssue, oneq::foundation::ValidationLocation>(
      severity, code, location_kind, entity_index, field, message);
  issue.phase = ArIssuePhase::kInputValidation;
  return issue;
}

bool FrameMatchesCycle(const ArCycleInput& input) {
  if (input.interference.emissions.empty()) {
    return true;
  }
  return oneq::electromagnetics::RfFrameMatchesCycleWindow(
      input.interference, input.cycle_index, input.cycle_start_time_s, input.dt_sec);
}

bool HasCartesianPosition(const ArSceneTarget& target) {
  return target.position_x != 0.0f || target.position_y != 0.0f || target.position_z != 0.0f;
}

void ValidateSingleSceneTarget(const ArSceneTarget& target, std::size_t target_index,
                               ArIssueList* issues) {
  if (!IsFinite(target.position_x) || !IsFinite(target.position_y) ||
      !IsFinite(target.position_z) || !IsFinite(target.velocity_x) ||
      !IsFinite(target.velocity_y) || !IsFinite(target.velocity_z) || !IsFinite(target.rcs) ||
      !IsFinite(target.range_m)) {
    issues->push_back(MakeIssue(ArIssueSeverity::kError, codes::kNonFiniteTargetField,
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "target",
                                "target contains non-finite numeric field"));
  }
  if (target.range_m <= 0.0f && !HasCartesianPosition(target)) {
    issues->push_back(MakeIssue(ArIssueSeverity::kError, codes::kMissingRangeAndCartesianPosition,
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "range_m",
                                "target requires positive range or cartesian position"));
  }
  if (target.external_target_id == 0U) {
    issues->push_back(MakeIssue(ArIssueSeverity::kInfo, codes::kUnknownExternalTargetId,
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "external_target_id",
                                "target external id is unknown"));
  }
  if (target.rcs < 0.0f) {
    issues->push_back(MakeIssue(ArIssueSeverity::kWarning, codes::kNegativeRcs,
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "rcs", "target rcs is negative"));
  }
}

}  // namespace

ArIssueList ValidateArCycleDeltaTime(double dt_sec) {
  ArIssueList issues;
  if (!std::isfinite(dt_sec)) {
    issues.push_back(MakeIssue(ArIssueSeverity::kError, codes::kNonFiniteCycleDeltaTime,
                               oneq::foundation::ValidationLocationKind::kGlobal,
                               static_cast<std::size_t>(-1), "dt_sec",
                               "cycle duration must be finite"));
  } else if (dt_sec <= 0.0) {
    issues.push_back(MakeIssue(ArIssueSeverity::kError, codes::kInvalidCycleDeltaTime,
                               oneq::foundation::ValidationLocationKind::kGlobal,
                               static_cast<std::size_t>(-1), "dt_sec",
                               "cycle duration must be positive"));
  }
  return issues;
}

ArIssueList ValidateArCycleInput(const ArCycleInput& input) {
  ArIssueList issues = ValidateArCycleDeltaTime(input.dt_sec);
  if (input.cycle_index == 0U) {
    issues.push_back(MakeIssue(ArIssueSeverity::kError, codes::kInvalidCycleIndex,
                               oneq::foundation::ValidationLocationKind::kGlobal,
                               static_cast<std::size_t>(-1), "cycle_index",
                               "cycle index must be non-zero"));
  }
  if (!std::isfinite(input.cycle_start_time_s) || input.cycle_start_time_s < 0.0) {
    issues.push_back(MakeIssue(ArIssueSeverity::kError, codes::kInvalidCycleStartTime,
                               oneq::foundation::ValidationLocationKind::kGlobal,
                               static_cast<std::size_t>(-1), "cycle_start_time_s",
                               "cycle start time must be finite and non-negative"));
  }

  oneq::coordinate::LocalFrameReference reference;
  oneq::coordinate::Vector3d radar_local_velocity;
  const oneq::coordinate::EulerAnglesDeg zero_mount{};
  if (!oneq::coordinate::IsFinite(input.platform.platform_position_ecef_m) ||
      !oneq::coordinate::IsFinite(input.platform.platform_velocity_mps) ||
      !oneq::coordinate::IsFinite(input.platform.platform_attitude_deg) ||
      !TryMakeArPoseFromPlatform(input.platform, zero_mount, &reference,
                                 &radar_local_velocity)) {
    issues.push_back(MakeIssue(ArIssueSeverity::kError, codes::kInvalidPlatformInput,
                               oneq::foundation::ValidationLocationKind::kPlatform,
                               static_cast<std::size_t>(-1), "platform",
                               "platform requires finite world kinematics"));
  } else {
    std::unordered_map<std::uint64_t, std::size_t> first_seen_target_index;
    for (std::size_t index = 0U; index < input.targets.size(); ++index) {
      const ArTargetInput& target = input.targets[index];
      ArSceneTarget local_target;
      if (!TryMakeArTargetFromEnu(target, reference, radar_local_velocity, &local_target)) {
        issues.push_back(MakeIssue(ArIssueSeverity::kError, codes::kInvalidTargetInput,
                                   oneq::foundation::ValidationLocationKind::kSceneEntity, index,
                                   "targets", "target ENU fields must be finite"));
        continue;
      }
      ValidateSingleSceneTarget(local_target, index, &issues);
      if (target.target_id == 0U) {
        continue;
      }
      const std::unordered_map<std::uint64_t, std::size_t>::const_iterator found =
          first_seen_target_index.find(target.target_id);
      if (found == first_seen_target_index.end()) {
        first_seen_target_index[target.target_id] = index;
      } else {
        std::ostringstream stream;
        stream << "duplicate target id " << target.target_id << " first seen at index "
               << found->second;
        issues.push_back(MakeIssue(ArIssueSeverity::kError, codes::kDuplicateExternalTargetId,
                                   oneq::foundation::ValidationLocationKind::kSceneEntity, index,
                                   "target_id", stream.str()));
      }
    }
  }

  if (!FrameMatchesCycle(input)) {
    issues.push_back(MakeIssue(ArIssueSeverity::kError, codes::kInterferenceFrameMismatch,
                               oneq::foundation::ValidationLocationKind::kEnvironment,
                               static_cast<std::size_t>(-1), "interference",
                               "non-empty interference frame must match the AR cycle window"));
  } else if (!input.interference.emissions.empty() &&
             !oneq::electromagnetics::TryValidateRfSceneFrame(input.interference)) {
    issues.push_back(MakeIssue(ArIssueSeverity::kError, codes::kInvalidInterferenceInput,
                               oneq::foundation::ValidationLocationKind::kEnvironment,
                               static_cast<std::size_t>(-1), "interference",
                               "interference frame contains invalid RF facts"));
  }
  return issues;
}

ArIssueList ValidateArSceneTargets(const ArSceneTargetList& targets) {
  ArIssueList issues;
  std::unordered_map<std::uint64_t, std::size_t> first_seen_target_index;
  for (std::size_t index = 0U; index < targets.size(); ++index) {
    ValidateSingleSceneTarget(targets[index], index, &issues);
    const std::uint64_t id = targets[index].external_target_id;
    if (id == 0U) {
      continue;
    }
    const std::unordered_map<std::uint64_t, std::size_t>::const_iterator found =
        first_seen_target_index.find(id);
    if (found == first_seen_target_index.end()) {
      first_seen_target_index[id] = index;
    } else {
      issues.push_back(MakeIssue(ArIssueSeverity::kError, codes::kDuplicateExternalTargetId,
                                 oneq::foundation::ValidationLocationKind::kSceneEntity, index,
                                 "external_target_id", "duplicate external target id"));
    }
  }
  return issues;
}

bool HasValidationError(const ArIssueList& issues) {
  // RED-1 收敛：统一判定逻辑在 common::validation::HasValidationPhaseError
  // （phase == kInputValidation && severity == kError）。
  return oneq::common::validation::HasValidationPhaseError(
      issues, &ArIssue::phase, &ArIssue::severity);
}

}  // namespace session
}  // namespace airborne_radar
