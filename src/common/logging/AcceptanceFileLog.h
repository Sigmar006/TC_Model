/**
 * @file AcceptanceFileLog.h
 * @brief 分层验收日志落盘：各层独立文件，与 1q_library.log 解耦。
 *
 * 路径优先级：OpenAcceptanceLog 显式路径 > 环境变量 > 编译期默认文件名。
 * 覆写语义：同一进程内首次打开截断旧文件（重跑进程得到干净的验收文件），
 * 进程内再次打开（会话重启）追加，保住本进程已写行。
 * 无异常；打开失败向 stderr 提示一次后静默丢弃。
 */

#ifndef ONEQ_SRC_COMMON_LOGGING_ACCEPTANCE_FILE_LOG_H_
#define ONEQ_SRC_COMMON_LOGGING_ACCEPTANCE_FILE_LOG_H_

#include <string>

namespace oneq {
namespace logging {

enum class AcceptanceChannel {
  kOpir = 0,
  kRir = 1,
  kFusion = 2,
  kInference = 3,
  kPrecision = 4,
  kIrInternalInterface = 5,
  kRadarInternalInterface = 6,
  kRadarOutput = 7,
  kSatelliteTarget = 8,
};

const std::string& CurrentAcceptanceLogOutputDirectory();

/** @brief 在当前线程内覆盖验收日志输出目录，析构时恢复上一层目录。 */
class ScopedAcceptanceLogOutputDirectory final {
 public:
  explicit ScopedAcceptanceLogOutputDirectory(const std::string& output_directory);
  ~ScopedAcceptanceLogOutputDirectory();

  ScopedAcceptanceLogOutputDirectory(const ScopedAcceptanceLogOutputDirectory&) = delete;
  ScopedAcceptanceLogOutputDirectory& operator=(
      const ScopedAcceptanceLogOutputDirectory&) = delete;

 private:
  std::string previous_output_directory_;
};

void OpenAcceptanceLog(AcceptanceChannel channel, const char* path);
void CloseAcceptanceLog(AcceptanceChannel channel);
// 刷出当前目录作用域的通道；无目录作用域时刷出默认通道。
void FlushAcceptanceLog(AcceptanceChannel channel);
bool IsAcceptanceLogOpen(AcceptanceChannel channel);
void WriteAcceptanceLog(AcceptanceChannel channel, const char* text);
void WriteAcceptanceLog(AcceptanceChannel channel, const std::string& text);
/** @brief 将通道日志写入指定目录；目录为空时沿用环境变量/编译期默认路径。 */
void WriteAcceptanceLogToDirectory(AcceptanceChannel channel,
                                   const std::string& output_directory,
                                   const std::string& text);

}  // namespace logging
}  // namespace oneq

#endif
