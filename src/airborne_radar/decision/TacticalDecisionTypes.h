/**
 * @file TacticalDecisionTypes.h
 * @brief 定义 AR 默认战术决策器的内部状态与结果类型。
 */

#ifndef AIRBORNE_RADAR_DECISION_TACTICAL_DECISION_TYPES_H_
#define AIRBORNE_RADAR_DECISION_TACTICAL_DECISION_TYPES_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "airborne_radar/decision/ControlReducerTypes.h"

namespace airborne_radar {
namespace session {

struct TargetCategory {
  std::string target_type = "UNKNOWN";
  float probability{0.0f};
  std::unordered_map<std::string, double> feature_values;

  TargetCategory() = default;
  explicit TargetCategory(const std::string& type) : target_type(type) {}

  void SetFeature(const std::string& feature_name, double value) {
    feature_values[feature_name] = value;
  }
};

using TargetCategoryList = std::vector<TargetCategory>;

enum class TacticalMode {
  kBaseline = 0,
  kThreatResponse,
  kProtectedEmission
};

struct TacticalStateStore {
  TacticalMode current_mode{TacticalMode::kBaseline};
  std::unordered_map<std::uint64_t, float> threat_memory;
  std::unordered_map<std::uint64_t, float> confidence_memory;
  std::vector<std::string> last_classification_labels;
  std::string last_decision_summary;
};

struct TacticalDecisionResult {
  TargetCategoryList target_classification_result;
  std::vector<TacticalProposal> proposals;
  TacticalMode selected_mode{TacticalMode::kBaseline};
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_TACTICAL_DECISION_TYPES_H_
