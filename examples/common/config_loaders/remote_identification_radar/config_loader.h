#ifndef EXAMPLES_RIR_CONFIG_LOADER_H_
#define EXAMPLES_RIR_CONFIG_LOADER_H_

#include <string>

#include "1q/remote_identification_radar/remote_identification_radar.hpp"
#include "config_loader_common.h"
#include "config_loader_detail.h"
#include "json_reader.h"

namespace examples {

/// Load a config::RirSessionConfig from a parsed JSON object.
inline void LoadRirSessionConfig(const examples::JsonValue& root,
                                 remote_identification_radar::config::RirSessionConfig* config) {
  LoadRirHardware(root["hardware"], &config->hardware);
  LoadRirOrientation(root["orientation"], &config->orientation);
  LoadRirMission(root["mission"], &config->mission);
  LoadRirPolicy(root["policy"], &config->policy);
  LoadRirEnvironment(root["environment"], &config->environment);
  if (root.Has("sensor_enabled")) {
    config->sensor_enabled = root["sensor_enabled"].AsBool();
  }
  if (root.Has("acceptance_log_enabled")) {
    config->acceptance_log_enabled = root["acceptance_log_enabled"].AsBool();
  }
  if (root.Has("acceptance_log_output_directory")) {
    config->acceptance_log_output_directory =
        root["acceptance_log_output_directory"].AsString();
  }
  // acceptance_log_enabled 是外部 JSON 运行时开关：false 禁止本会话的 LRDR
  // 验收日志和验收 CSV；该参数不能绕过编译期总开关。
}

/// Load a config::RirSessionConfig from a JSON file.
inline bool LoadRirSessionConfigFromFile(const char* path,
                                         remote_identification_radar::config::RirSessionConfig* config,
                                         std::string* error_msg) {
  examples::JsonValue root;
  if (!examples::JsonReader::ParseFile(path, &root, error_msg)) return false;
  if (root.type() != examples::JsonValue::kObject) {
    *error_msg = "root value must be a JSON object";
    return false;
  }
  LoadRirSessionConfig(root, config);
  return true;
}

}  // namespace examples

#endif  // EXAMPLES_RIR_CONFIG_LOADER_H_
