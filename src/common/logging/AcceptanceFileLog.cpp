/**
 * @file AcceptanceFileLog.cpp
 * @brief 九通道验收文件 sink（功能/性能验收、内部接口及雷达外部接口）。
 */

#include "common/logging/AcceptanceFileLog.h"
#include "common/logging/LogDirectory.h"
#include "common/platform/OneqPlatformCompat.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <sstream>

#if defined(_MSC_VER)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#ifndef ONEQ_SATELLITE_TARGET_LOG_PATH
#define ONEQ_SATELLITE_TARGET_LOG_PATH "log/SatelliteTarget.log"
#endif
#ifndef ONEQ_RADAR_OUTPUT_LOG_PATH
#define ONEQ_RADAR_OUTPUT_LOG_PATH "log/RadarOutput.log"
#endif
#ifndef ONEQ_RADAR_EXTERNAL_INTERFACE_LOG_PATH
#define ONEQ_RADAR_EXTERNAL_INTERFACE_LOG_PATH "log/RadarExternalInterface.log"
#endif
#ifndef ONEQ_IR_EXTERNAL_INTERFACE_LOG_PATH
#define ONEQ_IR_EXTERNAL_INTERFACE_LOG_PATH "log/IRExternalInterface.log"
#endif
#ifndef ONEQ_RADAR_INTERNAL_INTERFACE_LOG_PATH
#define ONEQ_RADAR_INTERNAL_INTERFACE_LOG_PATH "log/RadarInternalInterface.log"
#endif
#ifndef ONEQ_IR_INTERNAL_INTERFACE_LOG_PATH
#define ONEQ_IR_INTERNAL_INTERFACE_LOG_PATH "log/IRInternalInterface.log"
#endif

namespace oneq {

namespace {

// 本进程内"某验收文件是否已被首次打开"的跨模块共享判定（2026-10-15）。
//
// 背景：1q.lib 是静态库，会被分别链进多个模型 DLL（本工程实测 Sensors.dll 与
// DataProcessors.dll 各含一份本文件的 statics 与 ONEQ_*_LOG_PATH 字面量）。
// 仅靠 sink 实例上的 opened_in_process_ 判定"首次"，两个模块会各自认为自己是
// 首次、各自以 trunc 打开同一个文件，后写的一方把先写的一方已落盘的内容整段
// 抹掉。实证：log/SatelliteTarget.log 的最小周期恒等于融合侧（DataProcessors.dll）
// 首次写出的周期 234，而 OPIR 侧（Sensors.dll）已写下的周期 1..233 与其中
// "首个执行周期写一次"的第 10 项「安装矩阵误差」全部消失。
//
// 判定分两层：本模块内用进程内静态集合（快路径）；跨模块用 Windows 命名内核
// 对象（Local\ 命名空间，CreateMutex 返回 ERROR_ALREADY_EXISTS 即已被其它模块
// 认领）。句柄故意不关闭 —— 由进程退出回收，标记随之消失。非 Windows 平台
// 退化为模块内判定（当前交付平台为 Windows）。
bool ClaimFirstOpenForPath(const std::string& path) {
  static std::mutex claim_mutex;
  static std::map<std::string, bool> claimed;
  std::lock_guard<std::mutex> lock(claim_mutex);
  std::string key;
  key.reserve(path.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    // Windows 路径大小写不敏感：统一小写，避免同一文件两种写法被当成两个键。
    const char c = path[i];
    key += (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
  }
  if (claimed.find(key) != claimed.end()) {
    return false;  // 本模块已认领过
  }
  claimed[key] = true;
#if defined(_WIN32)
  // FNV-1a 32 位散列：只用作内核对象名后缀，碰撞后果仅为"少一次 trunc"，
  // 不影响正确性（宁可追加也不误截断）。
  unsigned int hash = 2166136261U;
  for (std::size_t i = 0; i < key.size(); ++i) {
    hash ^= static_cast<unsigned char>(key[i]);
    hash *= 16777619U;
  }
  char name[64];
  // 用 sprintf 而非 snprintf：VS2013 的 <cstdio> 尚未提供 std::snprintf。
  // 缓冲 64 字节，内容为固定前缀 "Local\oneq_acc_" + 8 位十六进制（约 25 字符），无溢出风险。
  std::sprintf(name, "Local\\oneq_acc_%08x", hash);
  HANDLE handle = ::CreateMutexA(nullptr, FALSE, name);
  if (handle != nullptr && ::GetLastError() == ERROR_ALREADY_EXISTS) {
    return false;  // 本进程内已有其它模块（另一个 DLL）认领过同一路径
  }
#endif
  return true;
}

}  // namespace
namespace logging {
namespace {

#if defined(_MSC_VER)
// MSVC 窄字符串字面量为系统 ANSI（中文 Windows 常为 GBK）；验收日志统一落 UTF-8，
// 便于 rg/编辑器按 UTF-8 检索，且不依赖 /utf-8 编译选项。
std::string NarrowSystemToUtf8(const std::string& text) {
  if (text.empty()) {
    return text;
  }
  const int wide_len =
      MultiByteToWideChar(CP_ACP, MB_ERR_INVALID_CHARS, text.data(),
                          static_cast<int>(text.size()), nullptr, 0);
  if (wide_len <= 0) {
    return text;
  }
  std::wstring wide(static_cast<std::size_t>(wide_len), L'\0');
  MultiByteToWideChar(CP_ACP, MB_ERR_INVALID_CHARS, text.data(),
                      static_cast<int>(text.size()), &wide[0], wide_len);
  const int utf8_len = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, wide.data(), wide_len,
                                           nullptr, 0, nullptr, nullptr);
  if (utf8_len <= 0) {
    return text;
  }
  std::string utf8(static_cast<std::size_t>(utf8_len), '\0');
  WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, wide.data(), wide_len, &utf8[0], utf8_len,
                      nullptr, nullptr);
  return utf8;
}
#endif

std::string EncodeAcceptanceText(const std::string& text) {
#if defined(_MSC_VER)
  return NarrowSystemToUtf8(text);
#else
  return text;
#endif
}

const char* EnvName(AcceptanceChannel channel) {
  switch (channel) {
    case AcceptanceChannel::kSatelliteTarget:
      return "ONEQ_SATELLITE_TARGET_LOG_PATH";
    case AcceptanceChannel::kRadarOutput:
      return "ONEQ_RADAR_OUTPUT_LOG_PATH";
    case AcceptanceChannel::kRadarExternalInterface:
      return "ONEQ_RADAR_EXTERNAL_INTERFACE_LOG_PATH";
    case AcceptanceChannel::kIrExternalInterface:
      return "ONEQ_IR_EXTERNAL_INTERFACE_LOG_PATH";
    case AcceptanceChannel::kRadarInternalInterface:
      return "ONEQ_RADAR_INTERNAL_INTERFACE_LOG_PATH";
    case AcceptanceChannel::kIrInternalInterface:
      return "ONEQ_IR_INTERNAL_INTERFACE_LOG_PATH";
  }
  return "ONEQ_SATELLITE_TARGET_LOG_PATH";
}

/**
 * @brief 兼容旧独立文件的环境变量名（融合/推演/精度/OPIR/RIR 归并前）。
 *
 * 归并后这些域统一落到 SatelliteTarget.log / RadarOutput.log；旧集成脚本若仍只设置
 * 旧变量，应继续生效而不是被静默忽略。返回空串表示本通道无旧变量。
 */
const char* LegacyEnvName(AcceptanceChannel channel) {
  switch (channel) {
    case AcceptanceChannel::kSatelliteTarget:
      return "ONEQ_OPIR_ACCEPTANCE_LOG_PATH";
    case AcceptanceChannel::kRadarOutput:
      return "ONEQ_RIR_ACCEPTANCE_LOG_PATH";
    default:
      return nullptr;
  }
}

const char* CompileDefaultPath(AcceptanceChannel channel) {
  switch (channel) {
    case AcceptanceChannel::kSatelliteTarget:
      return ONEQ_SATELLITE_TARGET_LOG_PATH;
    case AcceptanceChannel::kRadarOutput:
      return ONEQ_RADAR_OUTPUT_LOG_PATH;
    case AcceptanceChannel::kRadarExternalInterface:
      return ONEQ_RADAR_EXTERNAL_INTERFACE_LOG_PATH;
    case AcceptanceChannel::kIrExternalInterface:
      return ONEQ_IR_EXTERNAL_INTERFACE_LOG_PATH;
    case AcceptanceChannel::kRadarInternalInterface:
      return ONEQ_RADAR_INTERNAL_INTERFACE_LOG_PATH;
    case AcceptanceChannel::kIrInternalInterface:
      return ONEQ_IR_INTERNAL_INTERFACE_LOG_PATH;
  }
  return ONEQ_SATELLITE_TARGET_LOG_PATH;
}

const char* ChannelLabel(AcceptanceChannel channel) {
  switch (channel) {
    case AcceptanceChannel::kSatelliteTarget:
      return "satellite-target";
    case AcceptanceChannel::kRadarOutput:
      return "radar-output";
    case AcceptanceChannel::kRadarExternalInterface:
      return "radar-external-interface";
    case AcceptanceChannel::kIrExternalInterface:
      return "ir-external-interface";
    case AcceptanceChannel::kRadarInternalInterface:
      return "radar-internal-interface";
    case AcceptanceChannel::kIrInternalInterface:
      return "ir-internal-interface";
  }
  return "satellite-target";
}

const char* ChannelFileName(AcceptanceChannel channel) {
  switch (channel) {
    case AcceptanceChannel::kSatelliteTarget:
      return "SatelliteTarget.log";
    case AcceptanceChannel::kRadarOutput:
      return "RadarOutput.log";
    case AcceptanceChannel::kRadarExternalInterface:
      return "RadarExternalInterface.log";
    case AcceptanceChannel::kIrExternalInterface:
      return "IRExternalInterface.log";
    case AcceptanceChannel::kRadarInternalInterface:
      return "RadarInternalInterface.log";
    case AcceptanceChannel::kIrInternalInterface:
      return "IRInternalInterface.log";
  }
  return "SatelliteTarget.log";
}

std::string JoinPath(const std::string& directory, const char* file_name) {
  if (directory.empty()) {
    return std::string();
  }
  const char last = directory[directory.size() - 1U];
  return directory + ((last == '/' || last == '\\') ? "" : "/") + file_name;
}

/** @brief 读环境变量；未设置返回空串（MSVC 用 _dupenv_s 避免 getenv 告警）。 */
std::string ReadEnv(const char* name) {
  if (name == nullptr || *name == '\0') {
    return std::string();
  }
#if defined(_MSC_VER)
  char* buffer = nullptr;
  if (_dupenv_s(&buffer, nullptr, name) != 0 || buffer == nullptr) {
    return std::string();
  }
  std::string result(buffer);
  std::free(buffer);
  return result;
#else
  const char* env = std::getenv(name);
  return (env != nullptr && *env != '\0') ? std::string(env) : std::string();
#endif
}

std::string ResolveDefaultPath(AcceptanceChannel channel) {
  const std::string primary = ReadEnv(EnvName(channel));
  if (!primary.empty()) {
    return primary;
  }
  // 归并前的旧独立文件变量（如 ONEQ_FUSION_ACCEPTANCE_LOG_PATH）仍生效。
  const std::string legacy = ReadEnv(LegacyEnvName(channel));
  if (!legacy.empty()) {
    return legacy;
  }
  return std::string(CompileDefaultPath(channel));
}

class ChannelSink {
 public:
  explicit ChannelSink(AcceptanceChannel channel,
                       const std::string& explicit_path = std::string())
      : channel_(channel), explicit_path_(explicit_path) {}

  void Open(const char* path) {
    std::lock_guard<std::mutex> lock(mutex_);
    DoOpen(path);
  }

  void Close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_.is_open()) {
      stream_.flush();
      stream_.close();
    }
    open_failed_ = false;
    warned_ = false;
  }

  void Flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_.is_open()) {
      stream_.flush();
    }
  }

  bool IsOpen() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stream_.is_open();
  }

  /** @brief 只读诊断：当前实际打开的文件路径（未打开时为空串）。 */
  std::string OpenedPath() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return opened_path_;
  }

  /** @brief 只读诊断：流状态与写入位置（"open=1 good=1 fail=0 tellp=123"）。 */
  std::string StateText() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string text;
    text += stream_.is_open() ? "open=1 " : "open=0 ";
    text += stream_.good() ? "good=1 " : "good=0 ";
    text += stream_.fail() ? "fail=1 " : "fail=0 ";
    text += stream_.bad() ? "bad=1 " : "bad=0 ";
    std::ostringstream position;
    position << "tellp=" << static_cast<long long>(stream_.tellp());
    text += position.str();
    return text;
  }

  void Write(const std::string& text) {
    if (text.empty()) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stream_.is_open() && !open_failed_) {
      const std::string path =
          explicit_path_.empty() ? ResolveDefaultPath(channel_) : explicit_path_;
      DoOpen(path.c_str());
    }
    if (!stream_.is_open()) {
      return;
    }
    const std::string encoded = EncodeAcceptanceText(text);
    stream_ << encoded;
    if (encoded[encoded.size() - 1U] != '\n') {
      stream_ << '\n';
    }
    // 逐行落盘（2026-10-15）：宿主运行期读不到验收日志——实测一轮 22,801 周期写出的
    // 2.25 GB 直到进程退出（13:54:51）才在磁盘上可见，此前文件恒为 0 字节，排查与
    // 验收都无法边跑边读。这里显式 flush，代价是每行一次写系统调用；验收日志本就
    // 是旁路证据通道，可接受。若后续成为性能瓶颈，可改为累计 N 行或每周期 flush 一次。
    stream_.flush();
  }

 private:
  void DoOpen(const char* path) {
    if (path == nullptr || *path == '\0') {
      return;
    }
    if (stream_.is_open()) {
      stream_.flush();
      stream_.close();
    }
    oneq::logging::EnsureParentDirectory(path);
    // 同一进程内首次打开截断旧内容（重跑进程重写干净的验收文件，避免向归档
    // 证据目录追加重复行）；进程内再次打开（会话重启等）仍追加，保住本进程
    // 已写下的行。
    //
    // "首次"必须跨模块判定：1q.lib 被多个模型 DLL 各链一份，同一文件会被各模块
    // 分别判定为首次而各做一次 trunc，后写方抹掉先写方（详见 ClaimFirstOpenForPath
    // 注释里的实证）。故此处叠加跨模块判定，两者都认领为首次才截断。
    const bool first_open = (path != nullptr) && ClaimFirstOpenForPath(path);
    const std::ios::openmode mode =
        (opened_in_process_ || !first_open) ? (std::ios::out | std::ios::app | std::ios::binary)
                                            : (std::ios::out | std::ios::trunc | std::ios::binary);
    stream_.open(path, mode);
    if (stream_.is_open()) {
      open_failed_ = false;
      warned_ = false;
      opened_in_process_ = true;
      opened_path_ = path;
    } else {
      open_failed_ = true;
      opened_path_.clear();
      if (!warned_) {
        warned_ = true;
        std::fprintf(stderr, "[1q %s acceptance log] cannot open log file: %s\n",
                     ChannelLabel(channel_), path);
        std::fflush(stderr);
      }
    }
  }

  AcceptanceChannel channel_;
  std::string explicit_path_;
  mutable std::mutex mutex_;
  std::ofstream stream_;
  bool open_failed_ = false;
  bool warned_ = false;
  bool opened_in_process_ = false;
  std::string opened_path_; /**< 诊断：实际打开的文件路径（空 = 未打开）。 */
};

ChannelSink& SinkOf(AcceptanceChannel channel) {
  static ChannelSink satellite_target(AcceptanceChannel::kSatelliteTarget);
  static ChannelSink radar_output(AcceptanceChannel::kRadarOutput);
  static ChannelSink radar_external_interface(AcceptanceChannel::kRadarExternalInterface);
  static ChannelSink ir_external_interface(AcceptanceChannel::kIrExternalInterface);
  static ChannelSink radar_internal_interface(AcceptanceChannel::kRadarInternalInterface);
  static ChannelSink ir_internal_interface(AcceptanceChannel::kIrInternalInterface);
  switch (channel) {
    case AcceptanceChannel::kRadarOutput:
      return radar_output;
    case AcceptanceChannel::kRadarExternalInterface:
      return radar_external_interface;
    case AcceptanceChannel::kIrExternalInterface:
      return ir_external_interface;
    case AcceptanceChannel::kRadarInternalInterface:
      return radar_internal_interface;
    case AcceptanceChannel::kIrInternalInterface:
      return ir_internal_interface;
    case AcceptanceChannel::kSatelliteTarget:
    default:
      return satellite_target;
  }
}

ChannelSink& SinkOfPath(AcceptanceChannel channel, const std::string& path) {
  static std::mutex registry_mutex;
  static std::map<std::string, std::unique_ptr<ChannelSink>> sinks;
  const std::string key = std::to_string(static_cast<int>(channel)) + ":" + path;
  std::lock_guard<std::mutex> lock(registry_mutex);
  std::unique_ptr<ChannelSink>& sink = sinks[key];
  if (!sink) {
    sink.reset(new ChannelSink(channel, path));
  }
  return *sink;
}

}  // namespace

namespace {

std::string& ThreadAcceptanceLogOutputDirectory() {
#if defined(_MSC_VER) && _MSC_VER < 1900
  // VS2013 无 thread_local，且 __declspec(thread) 不支持非平凡类型，改以线程局部指针托管。
  static ONEQ_THREAD_LOCAL std::string* output_directory = nullptr;
  if (output_directory == nullptr) {
    output_directory = new std::string();
  }
  return *output_directory;
#else
  static thread_local std::string output_directory;
  return output_directory;
#endif
}

}  // namespace

const std::string& CurrentAcceptanceLogOutputDirectory() {
  return ThreadAcceptanceLogOutputDirectory();
}

ScopedAcceptanceLogOutputDirectory::ScopedAcceptanceLogOutputDirectory(
    const std::string& output_directory)
    : previous_output_directory_(ThreadAcceptanceLogOutputDirectory()),
      active_(!output_directory.empty()) {
  // 空目录表示“本会话不指定目录”，而不是“清空已有目录”：后者会让验收日志退回
  // 进程工作目录的编译期默认路径，使 `--output-dir` / 环境变量钉扎失效
  //（2026-09-28 端到端核对发现：会话配置留空时验收日志没有落到场景输出目录）。
  if (active_) {
    ThreadAcceptanceLogOutputDirectory() = output_directory;
  }
}

ScopedAcceptanceLogOutputDirectory::~ScopedAcceptanceLogOutputDirectory() {
  if (active_) {
    ThreadAcceptanceLogOutputDirectory() = previous_output_directory_;
  }
}

void OpenAcceptanceLog(AcceptanceChannel channel, const char* path) {
  SinkOf(channel).Open(path);
}

void CloseAcceptanceLog(AcceptanceChannel channel) { SinkOf(channel).Close(); }

void FlushAcceptanceLog(AcceptanceChannel channel) {
  const std::string& directory = CurrentAcceptanceLogOutputDirectory();
  if (directory.empty()) {
    SinkOf(channel).Flush();
  } else {
    SinkOfPath(channel, JoinPath(directory, ChannelFileName(channel))).Flush();
  }
}

bool IsAcceptanceLogOpen(AcceptanceChannel channel) { return SinkOf(channel).IsOpen(); }

std::string CurrentAcceptanceLogPath(AcceptanceChannel channel) {
  return SinkOf(channel).OpenedPath();
}

std::string EncodeAcceptanceTextForFile(const std::string& text) {
  return EncodeAcceptanceText(text);
}


std::string CurrentAcceptanceLogStateText(AcceptanceChannel channel) {
  return SinkOf(channel).StateText();
}

void WriteAcceptanceLog(AcceptanceChannel channel, const char* text) {
  if (text == nullptr || *text == '\0') {
    return;
  }
  WriteAcceptanceLog(channel, std::string(text));
}

void WriteAcceptanceLog(AcceptanceChannel channel, const std::string& text) {
  WriteAcceptanceLogToDirectory(
      channel, CurrentAcceptanceLogOutputDirectory(), text);
}

void WriteAcceptanceLogToDirectory(AcceptanceChannel channel,
                                   const std::string& output_directory,
                                   const std::string& text) {
  if (output_directory.empty()) {
    SinkOf(channel).Write(text);
    return;
  }
  SinkOfPath(channel, JoinPath(output_directory, ChannelFileName(channel))).Write(text);
}

}  // namespace logging
}  // namespace oneq
