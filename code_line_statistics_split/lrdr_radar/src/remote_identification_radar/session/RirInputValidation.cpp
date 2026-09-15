/**
 * @file RirInputValidation.cpp
 * @brief 远程识别雷达周期输入校验实现。
 *
 * 校验语义对齐 AR `ArInputValidation` 与 `ValidateArSceneTargets`（审计基线
 * 96de367c）：有限性 + 正值 + 标识唯一性；所有条目 phase=kInputValidation。
 */

#include "1q/coordinate/position_transform.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/remote_identification_radar/session/RirInputValidation.h"

#include <cmath>

#include "1q/remote_identification_radar/session/RirIssueCodes.h"

namespace remote_identification_radar {
namespace session {

namespace {

void PushIssue(RirIssueList* issues, RirIssueSeverity severity, const char* code,
               const std::string& field, const std::string& message) {
  RirIssue issue;
  issue.severity = severity;
  issue.phase = RirIssuePhase::kInputValidation;
  issue.code = code;
  issue.field = field;
  issue.message = message;
  issues->push_back(issue);
}

bool IsFinite(float value) { return std::isfinite(value); }

}  // namespace

RirIssueList ValidateRirCycleDeltaTime(double dt_sec) {
  RirIssueList issues;
  if (!std::isfinite(dt_sec)) {
    PushIssue(&issues, RirIssueSeverity::kError, codes::kNonFiniteCycleDeltaTime, "dt_sec",
              "Cycle delta time must be finite.");
  } else if (dt_sec <= 0.0) {
    PushIssue(&issues, RirIssueSeverity::kError, codes::kInvalidCycleDeltaTime, "dt_sec",
              "Cycle delta time must be positive.");
  }
  return issues;
}

RirIssueList ValidateRirSceneTargets(const RirSceneTargetList& targets) {
  RirIssueList issues;
  for (std::size_t i = 0U; i < targets.size(); ++i) {
    const RirSceneTarget& target = targets[i];
    const std::string location = "scene_targets[" + std::to_string(i) + "]";
    if (!IsFinite(target.position_x) || !IsFinite(target.position_y) ||
        !IsFinite(target.position_z) || !IsFinite(target.velocity_x) ||
        !IsFinite(target.velocity_y) || !IsFinite(target.velocity_z) || !IsFinite(target.rcs)) {
      PushIssue(&issues, RirIssueSeverity::kError, codes::kNonFiniteTargetField, location,
                "Scene target position/velocity/RCS must be finite.");
    }
    if (target.target_swerling_type < RirSwerlingType::kSwerling0 ||
        target.target_swerling_type > RirSwerlingType::kSwerling4) {
      PushIssue(&issues, RirIssueSeverity::kError, codes::kInvalidTargetMotionField,
                location + ".target_swerling_type",
                "Scene target Swerling type must be in [0, 4].");
    }
    const bool has_position =
        target.position_x != 0.0f || target.position_y != 0.0f || target.position_z != 0.0f;
    if (!has_position) {
      PushIssue(&issues, RirIssueSeverity::kError, codes::kMissingRangeAndCartesianPosition,
                location, "Scene target must carry non-zero cartesian position.");
    }
    // 真值样本有限性（识别专用特征输入）。
    for (const RirAspectRcsSample& sample : target.aspect_rcs_samples) {
      if (!IsFinite(sample.aspect_az_deg) || !IsFinite(sample.aspect_el_deg) ||
          !IsFinite(sample.rcs_dbsm)) {
        PushIssue(&issues, RirIssueSeverity::kError, codes::kNonFiniteTargetField,
                  location + ".aspect_rcs_samples", "Aspect RCS sample fields must be finite.");
      }
    }
    for (const RirPolSMatrixSample& sample : target.polarization_samples) {
      if (!IsFinite(sample.aspect_az_deg) || !IsFinite(sample.aspect_el_deg) ||
          !IsFinite(sample.hh_amp_db) || !IsFinite(sample.hh_phase_deg) ||
          !IsFinite(sample.hv_amp_db) || !IsFinite(sample.hv_phase_deg) ||
          !IsFinite(sample.vh_amp_db) || !IsFinite(sample.vh_phase_deg) ||
          !IsFinite(sample.vv_amp_db) || !IsFinite(sample.vv_phase_deg)) {
        PushIssue(&issues, RirIssueSeverity::kError, codes::kNonFiniteTargetField,
                  location + ".polarization_samples",
                  "Polarization S-matrix sample fields must be finite.");
      }
    }
    for (const RirRangeRcsScatterer& scatterer : target.range_rcs_scatterers) {
      if (!IsFinite(scatterer.range_offset_m) || !IsFinite(scatterer.rcs_dbsm) ||
          !IsFinite(scatterer.channel_1_rcs_dbsm) || !IsFinite(scatterer.channel_2_rcs_dbsm) ||
          !IsFinite(scatterer.phase_deg) || !IsFinite(scatterer.fluctuation_std_db)) {
        PushIssue(&issues, RirIssueSeverity::kError, codes::kNonFiniteTargetField,
                  location + ".range_rcs_scatterers", "Range scatterer fields must be finite.");
      }
    }
  }
  // 外部目标 ID 唯一性（0 表示未提供，不参与查重）。
  for (std::size_t i = 0U; i < targets.size(); ++i) {
    if (targets[i].external_target_id == 0U) {
      continue;
    }
    for (std::size_t j = i + 1U; j < targets.size(); ++j) {
      if (targets[j].external_target_id == targets[i].external_target_id) {
        PushIssue(&issues, RirIssueSeverity::kError, codes::kDuplicateExternalTargetId,
                  "scene_targets[" + std::to_string(j) + "].external_target_id",
                  "Duplicate external target id in scene targets.");
      }
    }
  }
  return issues;
}

RirIssueList ValidateRirCycleInput(const RirCycleInput& input, float recognition_dwell_sec) {
  RirIssueList issues = ValidateRirCycleDeltaTime(input.dt_sec);
  if (input.input_cycle_index == 0U) {
    PushIssue(&issues, RirIssueSeverity::kError, codes::kInvalidCycleIndex, "input_cycle_index",
              "Cycle index must be non-zero.");
  }
  if (!IsFinite(input.sim_time_sec)) {
    PushIssue(&issues, RirIssueSeverity::kError, codes::kNonFiniteTargetField, "sim_time_sec",
              "Simulation time must be finite.");
  }
  const oneq::coordinate::EcefPositionM& platform_position = input.platform_position;
  const bool components_finite = oneq::coordinate::IsFinite(platform_position);
  const double norm_m =
      std::sqrt(platform_position.x_m * platform_position.x_m +
                platform_position.y_m * platform_position.y_m +
                platform_position.z_m * platform_position.z_m);
  if (!components_finite || !(norm_m > 0.0)) {
    PushIssue(&issues, RirIssueSeverity::kError, codes::kInvalidPlatformPosition,
              "platform_position",
              "Platform ECEF position must be finite with positive norm.");
  } else {
    oneq::coordinate::LlaPositionDegM lla;
    if (!oneq::coordinate::TryEcefToLla(platform_position, &lla)) {
      PushIssue(&issues, RirIssueSeverity::kError, codes::kInvalidPlatformPosition,
                "platform_position", "Platform ECEF position must convert to valid LLA.");
    }
  }
  const RirIssueList target_issues = ValidateRirSceneTargets(input.scene_targets);
  issues.insert(issues.end(), target_issues.begin(), target_issues.end());

  const bool has_external_rf_scene = !input.rf_scene.emissions.empty();
  if (has_external_rf_scene) {
    if (!oneq::electromagnetics::TryValidateRfSceneFrame(input.rf_scene)) {
      PushIssue(&issues, RirIssueSeverity::kError, codes::kInvalidRfSceneFrame, "rf_scene",
                "RF scene frame must be valid when external emissions are provided.");
    }
    if (std::isfinite(recognition_dwell_sec) && recognition_dwell_sec > 0.0f &&
        (input.rf_scene.window_start_time_s != static_cast<double>(input.sim_time_sec) ||
         input.rf_scene.window_duration_s != static_cast<double>(recognition_dwell_sec))) {
      PushIssue(&issues, RirIssueSeverity::kError, codes::kInvalidRfSceneFrame,
                "rf_scene.window_start_time_s / window_duration_s",
                "RF scene window must match sim_time_sec and recognition dwell.");
    }
  }
  return issues;
}

bool HasValidationError(const RirIssueList& issues) {
  for (std::size_t i = 0U; i < issues.size(); ++i) {
    if (issues[i].phase == RirIssuePhase::kInputValidation &&
        issues[i].severity == RirIssueSeverity::kError) {
      return true;
    }
  }
  return false;
}

}  // namespace session
}  // namespace remote_identification_radar
