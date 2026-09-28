/**
 * @file AcceptanceFileLog.h
 * @brief 分层验收日志落盘：各层独立文件，与 1q_library.log 解耦。
 *
 * 目标物理文件（验收判定标准 §0「开发落盘口径」，2026-09-28）只有六个：
 * `log/SatelliteTarget.log`（红外普通功能 + 融合/推演/精度归并）、
 * `log/RadarOutput.log`（雷达普通功能）、`log/RadarExternalInterface.log`、
 * `log/IRExternalInterface.log`、`log/RadarInternalInterface.log`、
 * `log/IRInternalInterface.log`。融合、推演、精度不再生成独立验收文件，
 * 其原编译开关只作功能控制。
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
  kSatelliteTarget = 0,        /**< 红外普通功能 + 融合/推演/精度归并。 */
  kRadarOutput = 1,            /**< 雷达普通功能。 */
  kRadarExternalInterface = 2, /**< 雷达对外/外部接口。 */
  kIrExternalInterface = 3,    /**< 红外对外/外部接口。 */
  kRadarInternalInterface = 4, /**< 雷达内部接口。 */
  kIrInternalInterface = 5,    /**< 红外内部接口。 */
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
