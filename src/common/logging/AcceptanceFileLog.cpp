/**
 * @file AcceptanceFileLog.cpp
 * @brief 九通道验收文件 sink（功能/性能验收、内部接口及雷达外部接口）。
 */

#include "common/logging/AcceptanceFileLog.h"
#include "common/logging/LogDirectory.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#if defined(_MSC_VER)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#ifndef ONEQ_OPIR_ACCEPTANCE_LOG_PATH
#define ONEQ_OPIR_ACCEPTANCE_LOG_PATH "log/opir_acceptance.log"
#endif
#ifndef ONEQ_RIR_ACCEPTANCE_LOG_PATH
#define ONEQ_RIR_ACCEPTANCE_LOG_PATH "log/rir_acceptance.log"
#endif
#ifndef ONEQ_FUSION_ACCEPTANCE_LOG_PATH
#define ONEQ_FUSION_ACCEPTANCE_LOG_PATH "log/fusion_acceptance.log"
#endif
#ifndef ONEQ_INFERENCE_ACCEPTANCE_LOG_PATH
#define ONEQ_INFERENCE_ACCEPTANCE_LOG_PATH "log/inference_acceptance.log"
#endif
#ifndef ONEQ_PRECISION_ACCEPTANCE_LOG_PATH
#define ONEQ_PRECISION_ACCEPTANCE_LOG_PATH "log/precision_acceptance.log"
#endif
#ifndef ONEQ_IR_INTERNAL_INTERFACE_LOG_PATH
#define ONEQ_IR_INTERNAL_INTERFACE_LOG_PATH "log/IRInternalInterface.log"
#endif
#ifndef ONEQ_RADAR_INTERNAL_INTERFACE_LOG_PATH
#define ONEQ_RADAR_INTERNAL_INTERFACE_LOG_PATH "log/RadarInternalInterface.log"
#endif
#ifndef ONEQ_RADAR_OUTPUT_LOG_PATH
#define ONEQ_RADAR_OUTPUT_LOG_PATH "log/RadarOutput.log"
#endif
#ifndef ONEQ_SATELLITE_TARGET_LOG_PATH
#define ONEQ_SATELLITE_TARGET_LOG_PATH "log/SatelliteTarget.log"
#endif

namespace oneq {
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
    case AcceptanceChannel::kOpir:
      return "ONEQ_OPIR_ACCEPTANCE_LOG_PATH";
    case AcceptanceChannel::kRir:
      return "ONEQ_RIR_ACCEPTANCE_LOG_PATH";
    case AcceptanceChannel::kFusion:
      return "ONEQ_FUSION_ACCEPTANCE_LOG_PATH";
    case AcceptanceChannel::kInference:
      return "ONEQ_INFERENCE_ACCEPTANCE_LOG_PATH";
    case AcceptanceChannel::kPrecision:
      return "ONEQ_PRECISION_ACCEPTANCE_LOG_PATH";
    case AcceptanceChannel::kIrInternalInterface:
      return "ONEQ_IR_INTERNAL_INTERFACE_LOG_PATH";
    case AcceptanceChannel::kRadarInternalInterface:
      return "ONEQ_RADAR_INTERNAL_INTERFACE_LOG_PATH";
    case AcceptanceChannel::kRadarOutput:
      return "ONEQ_RADAR_OUTPUT_LOG_PATH";
    case AcceptanceChannel::kSatelliteTarget:
      return "ONEQ_SATELLITE_TARGET_LOG_PATH";
  }
  return "ONEQ_OPIR_ACCEPTANCE_LOG_PATH";
}

const char* CompileDefaultPath(AcceptanceChannel channel) {
  switch (channel) {
    case AcceptanceChannel::kOpir:
      return ONEQ_OPIR_ACCEPTANCE_LOG_PATH;
    case AcceptanceChannel::kRir:
      return ONEQ_RIR_ACCEPTANCE_LOG_PATH;
    case AcceptanceChannel::kFusion:
      return ONEQ_FUSION_ACCEPTANCE_LOG_PATH;
    case AcceptanceChannel::kInference:
      return ONEQ_INFERENCE_ACCEPTANCE_LOG_PATH;
    case AcceptanceChannel::kPrecision:
      return ONEQ_PRECISION_ACCEPTANCE_LOG_PATH;
    case AcceptanceChannel::kIrInternalInterface:
      return ONEQ_IR_INTERNAL_INTERFACE_LOG_PATH;
    case AcceptanceChannel::kRadarInternalInterface:
      return ONEQ_RADAR_INTERNAL_INTERFACE_LOG_PATH;
    case AcceptanceChannel::kRadarOutput:
      return ONEQ_RADAR_OUTPUT_LOG_PATH;
    case AcceptanceChannel::kSatelliteTarget:
      return ONEQ_SATELLITE_TARGET_LOG_PATH;
  }
  return ONEQ_OPIR_ACCEPTANCE_LOG_PATH;
}

const char* ChannelLabel(AcceptanceChannel channel) {
  switch (channel) {
    case AcceptanceChannel::kOpir:
      return "opir";
    case AcceptanceChannel::kRir:
      return "rir";
    case AcceptanceChannel::kFusion:
      return "fusion";
    case AcceptanceChannel::kInference:
      return "inference";
    case AcceptanceChannel::kPrecision:
      return "precision";
    case AcceptanceChannel::kIrInternalInterface:
      return "ir-internal-interface";
    case AcceptanceChannel::kRadarInternalInterface:
      return "radar-internal-interface";
    case AcceptanceChannel::kRadarOutput:
      return "radar-output";
    case AcceptanceChannel::kSatelliteTarget:
      return "satellite-target";
  }
  return "opir";
}

const char* ChannelFileName(AcceptanceChannel channel) {
  switch (channel) {
    case AcceptanceChannel::kOpir:
      return "opir_acceptance.log";
    case AcceptanceChannel::kRir:
      return "rir_acceptance.log";
    case AcceptanceChannel::kFusion:
      return "fusion_acceptance.log";
    case AcceptanceChannel::kInference:
      return "inference_acceptance.log";
    case AcceptanceChannel::kPrecision:
      return "precision_acceptance.log";
    case AcceptanceChannel::kIrInternalInterface:
      return "IRInternalInterface.log";
    case AcceptanceChannel::kRadarInternalInterface:
      return "RadarInternalInterface.log";
    case AcceptanceChannel::kRadarOutput:
      return "RadarOutput.log";
    case AcceptanceChannel::kSatelliteTarget:
      return "SatelliteTarget.log";
  }
  return "acceptance.log";
}

std::string JoinPath(const std::string& directory, const char* file_name) {
  if (directory.empty()) {
    return std::string();
  }
  const char last = directory[directory.size() - 1U];
  return directory + ((last == '/' || last == '\\') ? "" : "/") + file_name;
}

std::string ResolveDefaultPath(AcceptanceChannel channel) {
#if defined(_MSC_VER)
  char* buffer = nullptr;
  if (_dupenv_s(&buffer, nullptr, EnvName(channel)) == 0 && buffer != nullptr) {
    std::string result(buffer);
    std::free(buffer);
    if (!result.empty()) {
      return result;
    }
  }
#else
  const char* env = std::getenv(EnvName(channel));
  if (env != nullptr && *env != '\0') {
    return std::string(env);
  }
#endif
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
    const std::ios::openmode mode =
        opened_in_process_ ? (std::ios::out | std::ios::app | std::ios::binary)
                           : (std::ios::out | std::ios::trunc | std::ios::binary);
    stream_.open(path, mode);
    if (stream_.is_open()) {
      open_failed_ = false;
      warned_ = false;
      opened_in_process_ = true;
    } else {
      open_failed_ = true;
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
};

ChannelSink& SinkOf(AcceptanceChannel channel) {
  static ChannelSink opir(AcceptanceChannel::kOpir);
  static ChannelSink rir(AcceptanceChannel::kRir);
  static ChannelSink fusion(AcceptanceChannel::kFusion);
  static ChannelSink inference(AcceptanceChannel::kInference);
  static ChannelSink precision(AcceptanceChannel::kPrecision);
  static ChannelSink ir_internal_interface(AcceptanceChannel::kIrInternalInterface);
  static ChannelSink radar_internal_interface(AcceptanceChannel::kRadarInternalInterface);
  static ChannelSink radar_output(AcceptanceChannel::kRadarOutput);
  static ChannelSink satellite_target(AcceptanceChannel::kSatelliteTarget);
  switch (channel) {
    case AcceptanceChannel::kRir:
      return rir;
    case AcceptanceChannel::kFusion:
      return fusion;
    case AcceptanceChannel::kInference:
      return inference;
    case AcceptanceChannel::kPrecision:
      return precision;
    case AcceptanceChannel::kIrInternalInterface:
      return ir_internal_interface;
    case AcceptanceChannel::kRadarInternalInterface:
      return radar_internal_interface;
    case AcceptanceChannel::kRadarOutput:
      return radar_output;
    case AcceptanceChannel::kSatelliteTarget:
      return satellite_target;
    case AcceptanceChannel::kOpir:
    default:
      return opir;
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
  static thread_local std::string output_directory;
  return output_directory;
}

}  // namespace

const std::string& CurrentAcceptanceLogOutputDirectory() {
  return ThreadAcceptanceLogOutputDirectory();
}

ScopedAcceptanceLogOutputDirectory::ScopedAcceptanceLogOutputDirectory(
    const std::string& output_directory)
    : previous_output_directory_(ThreadAcceptanceLogOutputDirectory()) {
  ThreadAcceptanceLogOutputDirectory() = output_directory;
}

ScopedAcceptanceLogOutputDirectory::~ScopedAcceptanceLogOutputDirectory() {
  ThreadAcceptanceLogOutputDirectory() = previous_output_directory_;
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
