#ifndef EXAMPLES_SBIRS_CONFIG_LOADER_H_
#define EXAMPLES_SBIRS_CONFIG_LOADER_H_

#include <string>

#include "1q/sbirs_sensor/sbirs_sensor.hpp"
#include "json_reader.h"
#include "config_loader_common.h"
#include "config_loader_detail.h"

namespace examples {

/// Load a config::SbirsSessionConfig from a parsed JSON object.
inline void LoadSbirsSessionConfig(
    const examples::JsonValue& root,
    sbirs_sensor::config::SbirsSessionConfig* config) {
  LoadSbirsHardware(root["hardware"], &config->hardware);
  LoadSbirsMission(root["mission"], &config->mission);
  LoadSbirsOrientation(root["orientation"], &config->orientation);
  LoadSbirsPolicy(root["policy"], &config->policy);
  LoadSbirsEnvironment(root["environment"], &config->environment);
  if (root.Has("sensor_enabled")) {
    config->sensor_enabled = root["sensor_enabled"].AsBool();
  }
  if (root.Has("acceptance_log_enabled")) {
    config->acceptance_log_enabled = root["acceptance_log_enabled"].AsBool();
  }
  // acceptance_log_enabled 是外部 JSON 运行时开关：false 禁止本会话的红外
  // 验收日志和融合副本；该参数不能绕过对应的编译期总开关。
}

/// Load a config::SbirsSessionConfig from a JSON file.
inline bool LoadSbirsSessionConfigFromFile(
    const char* path, sbirs_sensor::config::SbirsSessionConfig* config,
    std::string* error_msg) {
  examples::JsonValue root;
  if (!examples::JsonReader::ParseFile(path, &root, error_msg)) return false;
  if (root.type() != examples::JsonValue::kObject) {
    *error_msg = "root value must be a JSON object";
    return false;
  }
  LoadSbirsSessionConfig(root, config);
  return true;
}

}  // namespace examples

#endif  // EXAMPLES_SBIRS_CONFIG_LOADER_H_
