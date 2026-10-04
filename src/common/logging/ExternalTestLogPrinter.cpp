/**
 * @file ExternalTestLogPrinter.cpp
 * @brief 外部集成侧简易测试日志打印实现。
 */
#include "1q/logging/ExternalTestLogPrinter.h"
#include "common/platform/OneqPlatformCompat.h"

#include <cstdio>
#include <fstream>

namespace oneq {
namespace logging {
namespace {

std::string MakeSingleLine(const std::string& content) {
  std::string result = content;
  for (std::string::size_type i = 0; i < result.size(); ++i) {
    if (result[i] == '\r' || result[i] == '\n') {
      result[i] = ' ';
    }
  }
  return result;
}

std::string FormatLine(double sim_time_sec, std::uint32_t cycle, const char* item,
                       const std::string& content) {
  char time_buffer[32];
  char cycle_buffer[16];
  ONEQ_SNPRINTF(time_buffer, sizeof(time_buffer), "%.3f", sim_time_sec);
  ONEQ_SNPRINTF(cycle_buffer, sizeof(cycle_buffer), "%u", cycle);

  const std::string single_line_content = MakeSingleLine(content);
  std::string line;
  line.reserve(64U + single_line_content.size());
  line += "仿真时间=";
  line += time_buffer;
  line += "s 仿真周期=";
  line += cycle_buffer;
  line += " [";
  line += item;
  line += "]";
  if (!single_line_content.empty()) {
    line += " ";
    line += single_line_content;
  }
  return line;
}

}  // namespace

ExternalTestLogPrinter::ExternalTestLogPrinter(const std::string& output_path)
    : output_path_(output_path) {}

void ExternalTestLogPrinter::SetOutputPath(const std::string& output_path) {
  std::lock_guard<std::mutex> lock(mutex_);
  output_path_ = output_path;
}

std::string ExternalTestLogPrinter::GetOutputPath() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return output_path_;
}

bool ExternalTestLogPrinter::PrintBallisticData(double sim_time_sec, std::uint32_t cycle,
                                                const std::string& content) {
  return Print("弹道数据", sim_time_sec, cycle, content);
}

bool ExternalTestLogPrinter::PrintSimulationControlClock(double sim_time_sec,
                                                         std::uint32_t cycle,
                                                         const std::string& content) {
  return Print("仿真控制与时钟接口测试", sim_time_sec, cycle, content);
}

bool ExternalTestLogPrinter::PrintRadarSimulationControlClock(
    double sim_time_sec, std::uint32_t cycle, const std::string& content) {
  return Print("仿真控制与时钟-雷达接口测试", sim_time_sec, cycle, content);
}

bool ExternalTestLogPrinter::Print(const char* item, double sim_time_sec,
                                   std::uint32_t cycle, const std::string& content) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (output_path_.empty()) {
    return false;
  }

  std::ofstream stream(output_path_.c_str(), std::ios::out | std::ios::app | std::ios::binary);
  if (!stream.is_open()) {
    return false;
  }
  stream << FormatLine(sim_time_sec, cycle, item, content) << '\n';
  stream.flush();
  return stream.good();
}

}  // namespace logging
}  // namespace oneq
