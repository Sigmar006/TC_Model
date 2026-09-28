/**
 * @file acceptance_paths.h
 * @brief 验收文件路径钉扎（场景输出目录 → 库验收 sink 环境变量），跨栈共用。
 *
 * 库侧验收文件与天线/波位 CSV 默认写 CWD（AcceptanceFileLog 的 env > 编译宏
 * > 源码默认解析序）；任何场景可执行（ECS 场景栈与精度评估栈）在初始化时
 * 调 BindAcceptanceLogPaths 把全部验收产物钉到本场景输出目录，避免落到
 * 运行目录。
 */

#ifndef EXAMPLES_LOGGER_ACCEPTANCE_PATHS_H_
#define EXAMPLES_LOGGER_ACCEPTANCE_PATHS_H_

#include <string>

namespace component_attachment {
namespace app {

/// 进程环境变量写入（Windows 用 _putenv_s）。
inline void SetProcessEnv(const char* name, const std::string& value) {
#if defined(_WIN32)
  _putenv_s(name, value.c_str());
#else
  setenv(name, value.c_str(), /*overwrite=*/1);
#endif
}

/// 验收文件与天线/波位 CSV 钉到场景输出目录（库侧 env 覆盖优先级最高）。
/// 2026-09-28 归并后目标文件只有六个：SatelliteTarget / RadarOutput /
/// RadarExternalInterface / IRExternalInterface / RadarInternalInterface /
/// IRInternalInterface；融合、推演、精度不再生成独立文件，故不再设置其旧变量。
inline void BindAcceptanceLogPaths(const std::string& output_dir) {
  SetProcessEnv("ONEQ_SATELLITE_TARGET_LOG_PATH", output_dir + "/SatelliteTarget.log");
  SetProcessEnv("ONEQ_RADAR_OUTPUT_LOG_PATH", output_dir + "/RadarOutput.log");
  SetProcessEnv("ONEQ_RADAR_EXTERNAL_INTERFACE_LOG_PATH",
                output_dir + "/RadarExternalInterface.log");
  SetProcessEnv("ONEQ_IR_EXTERNAL_INTERFACE_LOG_PATH",
                output_dir + "/IRExternalInterface.log");
  SetProcessEnv("ONEQ_RADAR_INTERNAL_INTERFACE_LOG_PATH",
                output_dir + "/RadarInternalInterface.log");
  SetProcessEnv("ONEQ_IR_INTERNAL_INTERFACE_LOG_PATH",
                output_dir + "/IRInternalInterface.log");
  SetProcessEnv("ONEQ_RIR_ANTENNA_PATTERN_CSV_PATH",
                output_dir + "/RadarAntennaGainPatterndata.csv");
  SetProcessEnv("ONEQ_RIR_SCAN_PATTERN_CSV_PATH", output_dir + "/Rir_scan_pattern.csv");
}

}  // namespace app
}  // namespace component_attachment

#endif  // EXAMPLES_LOGGER_ACCEPTANCE_PATHS_H_
