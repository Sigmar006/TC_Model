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

/* =============================================================================
 * 外部集成使用说明（2026-10-27）
 * -----------------------------------------------------------------------------
 * 1. 六个通道各对应一个物理文件，通道与文件的对应关系是**固定契约**：
 *      SatelliteTarget        -> log/SatelliteTarget.log        红外普通功能 + 融合/推演/精度
 *      RadarOutput            -> log/RadarOutput.log            雷达普通功能（29..48 等）
 *      RadarExternalInterface -> log/RadarExternalInterface.log 雷达对外/外部接口
 *      IrExternalInterface    -> log/IRExternalInterface.log    红外对外/外部接口
 *      RadarInternalInterface -> log/RadarInternalInterface.log 雷达内部接口（70..73）
 *      IrInternalInterface    -> log/IRInternalInterface.log    红外内部接口（74..77）
 *    每个文件**只由本通道写入**，不要跨通道写同一文件（跨模块并发会串行化，见下）。
 *
 * 2. 行格式由 AcceptanceRecordFormat 统一：
 *      `仿真时间=<秒>s 仿真周期=<周期> [<验收项名>] <内容…>`
 *    写入方只需给出「项名 + 内容」，不要自己拼时间/周期前缀，也不要带换行符——
 *    WriteAcceptanceLog 会补一个 '\n'，带入换行会产生空行。
 *
 * 3. 启用与落盘目录：
 *    - 编译期：各模块的 `*_ACCEPTANCE_LOG_ENABLED()` 宏决定是否编译进写入点；
 *    - 运行期：`ScopedAcceptanceLogOutputDirectory` 覆盖输出目录（空串 = 不覆盖），
 *      或 `OpenAcceptanceLog(channel, path)` 显式指定文件；未指定时用
 *      `ONEQ_*_LOG_PATH` 编译期默认路径（宿主里通常指向 FOSimStudio\bin\App\log）。
 *
 * 4. 线程安全（**重要，外部集成请勿绕过**）：
 *    - `WriteAcceptanceLog` 内部有通道级互斥体，**且**在 Windows 上叠加了一个进程内
 *      命名互斥体（`Local\1q_acceptance_log_write_v1`）：因为 1q.lib 是静态库，会被
 *      Sensors.dll / DataProcessors.dll 各链一份，两个模块各有一份 statics 与各自的锁，
 *      仅靠模块内的锁挡不住跨模块并发写（实测会出现两行拼成一行）。
 *    - 因此**不要**在集成方另开文件句柄直接写这六个文件；要写就走本接口，
 *      或至少调用 `EncodeAcceptanceTextForFile` 并在自己的进程级锁内整行写出。
 *
 * 5. 排障接口（只读，不改变状态）：
 *    - `CurrentAcceptanceLogPath(channel)`：这一通道实际打开的是哪个文件；
 *    - `CurrentAcceptanceLogStateText(channel)`：open/good/fail/bad + tellp；
 *    - `IsAcceptanceLogOpen(channel)`：是否已打开。
 *    三者用于“日志没出现/写到别处”的定位，可安全地在任意周期调用。
 *
 * 6. 与诊断日志的区别：本接口写的是**验收证据**（要交付、要按项核对）；
 *    `oneq_diag_log.h` 写的是**临时诊断**（可清可弃）。两者不要混用同一文件。
 * ========================================================================== */


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

/**
 * @brief 在当前线程内覆盖验收日志输出目录，析构时恢复上一层目录。
 *
 * @note 传入**空字符串**表示“本会话不指定目录”，既不覆盖也不清空当前作用域，
 *       验收日志继续按环境变量 → 编译期默认路径解析。这样 `--output-dir` 与
 *       `BindAcceptanceLogPaths` 的钉扎在会话配置留空时仍然生效。
 */
class ScopedAcceptanceLogOutputDirectory final {
 public:
  explicit ScopedAcceptanceLogOutputDirectory(const std::string& output_directory);
  ~ScopedAcceptanceLogOutputDirectory();

  ScopedAcceptanceLogOutputDirectory(const ScopedAcceptanceLogOutputDirectory&) = delete;
  ScopedAcceptanceLogOutputDirectory& operator=(
      const ScopedAcceptanceLogOutputDirectory&) = delete;

 private:
  std::string previous_output_directory_;
  bool active_{false};
};

void OpenAcceptanceLog(AcceptanceChannel channel, const char* path);
void CloseAcceptanceLog(AcceptanceChannel channel);
// 刷出当前目录作用域的通道；无目录作用域时刷出默认通道。
void FlushAcceptanceLog(AcceptanceChannel channel);
bool IsAcceptanceLogOpen(AcceptanceChannel channel);
/**
 * @brief 只读诊断：该通道当前实际打开的文件路径（未打开返回空串）。
 * @note `IsAcceptanceLogOpen` 只知道开没开、不知道开的是哪个文件；验收排查"行写到
 *       了哪儿"时需要本访问器（2026-10-22 第 26/27 项空行排查）。
 */
std::string CurrentAcceptanceLogPath(AcceptanceChannel channel);
/**
 * @brief 把一行验收文本转成落盘编码（Windows: ANSI -> UTF-8；其它平台原样）。
 *
 * @note 供"库出字符串、集成方落盘"的用法：集成方在 WriteAcceptanceLog 之外自己写文件时，
 *       必须先用本函数转码，否则文件会是宿主 ANSI 编码、与验收日志的 UTF-8 约定不符。
 *       与 WriteAcceptanceLog 内部用的是同一个转换，不会重复转码。
 */
std::string EncodeAcceptanceTextForFile(const std::string& text);
/** @brief 只读诊断：通道流的 open/good/fail/bad 与 tellp（排查"行写不进去"）。 */
std::string CurrentAcceptanceLogStateText(AcceptanceChannel channel);
void WriteAcceptanceLog(AcceptanceChannel channel, const char* text);
void WriteAcceptanceLog(AcceptanceChannel channel, const std::string& text);
/** @brief 将通道日志写入指定目录；目录为空时沿用环境变量/编译期默认路径。 */
void WriteAcceptanceLogToDirectory(AcceptanceChannel channel,
                                   const std::string& output_directory,
                                   const std::string& text);

}  // namespace logging
}  // namespace oneq

#endif
