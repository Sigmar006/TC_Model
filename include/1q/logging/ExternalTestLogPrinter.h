/**
 * @file ExternalTestLogPrinter.h
 * @brief 可复制到外部集成工程使用的简易测试日志打印类。
 *
 * 本类不接入 1q 内部验收日志通道，也不决定日志文件名。调用方提供完整
 * 输出路径和待记录内容，本类仅按现有验收记录格式组织并追加写入。
 */
#ifndef ONEQ_LOGGING_EXTERNAL_TEST_LOG_PRINTER_H_
#define ONEQ_LOGGING_EXTERNAL_TEST_LOG_PRINTER_H_

#include <cstdint>
#include <mutex>
#include <string>

#include "1q/api.hpp"

namespace oneq {
namespace logging {

/**
 * @brief 外部集成侧的简易测试日志打印器。
 *
 * 输出格式：`仿真时间=1.000s 仿真周期=1 [测试项名称] 外部输入内容`。
 * 每次调用以追加模式打开文件。类不创建父目录；失败时返回 false，不抛异常。
 */
class ONEQ_API ExternalTestLogPrinter {
 public:
  explicit ExternalTestLogPrinter(const std::string& output_path);

  ExternalTestLogPrinter(const ExternalTestLogPrinter&) = delete;
  ExternalTestLogPrinter& operator=(const ExternalTestLogPrinter&) = delete;

  void SetOutputPath(const std::string& output_path);
  std::string GetOutputPath() const;

  bool PrintBallisticData(double sim_time_sec, std::uint32_t cycle,
                          const std::string& content);
  bool PrintSimulationControlClock(double sim_time_sec, std::uint32_t cycle,
                                   const std::string& content);
  bool PrintRadarSimulationControlClock(double sim_time_sec, std::uint32_t cycle,
                                        const std::string& content);

 private:
  bool Print(const char* item, double sim_time_sec, std::uint32_t cycle,
             const std::string& content);

  mutable std::mutex mutex_;
  std::string output_path_;
};

}  // namespace logging
}  // namespace oneq

#endif  // ONEQ_LOGGING_EXTERNAL_TEST_LOG_PRINTER_H_
