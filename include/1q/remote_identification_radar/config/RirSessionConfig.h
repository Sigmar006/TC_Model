/**
 * @file RirSessionConfig.h
 * @brief 远程识别雷达会话初始化主配置类型。
 *
 * 会话初始化配置（五域公开模型）的主头文件。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_SESSION_CONFIG_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_SESSION_CONFIG_H_

#include <string>

#include "1q/api.hpp"
#include "1q/remote_identification_radar/config/RirEnvironmentConfig.h"
#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "1q/remote_identification_radar/config/RirMissionConfig.h"
#include "1q/remote_identification_radar/config/RirOrientationConfig.h"
#include "1q/remote_identification_radar/config/RirPolicyConfig.h"

namespace remote_identification_radar {
namespace config {

/**
 * @brief RirSessionConfig 会话初始化配置（五域公开模型）。
 *
 * 电源状态由顶层 `sensor_enabled` 唯一承载（COMMON-OQ-4 字段提升）：
 * `mission` 域不含电源字段，运行时补丁的电源入口为
 * `RirRuntimeConfigPatch::has_sensor_enabled`。
 */
struct ONEQ_API RirSessionConfig {
  RirHardwareConfig hardware{};
  RirOrientationConfig orientation{};
  RirMissionConfig mission{};
  RirPolicyConfig policy{};
  RirEnvironmentConfig environment{};
  bool sensor_enabled{true}; /**< 传感器初始电源状态 */
  /**
   * 外部运行时日志开关：true 打印 LRDR 功能、接口验收日志及验收 CSV，false 全部禁止。
   * 仅在编译期开关 ONEQ_ENABLE_RIR_ACCEPTANCE_LOG 已启用时生效，默认 true。
   */
  bool acceptance_log_enabled{true};
  /**
   * 验收日志输出目录；非空时 RadarOutput.log、RadarInternalInterface.log 及两份
   * 雷达验收 CSV 均写入该目录。空字符串沿用环境变量或编译期默认路径。
   */
  std::string acceptance_log_output_directory{};
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_SESSION_CONFIG_H_
