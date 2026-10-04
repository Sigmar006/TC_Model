/**
 * @file ProjectFileLog.cpp
 * @brief 文件日志后端 sink 实现：单例 + mutex + ofstream + 时间戳 + 懒打开。
 *
 * 行为约定：
 *  - 默认最低级别 kInfo（镜像 spdlog 默认 logger），debug 消息默认不落盘；
 *  - 懒打开：首次 Write 时按 路径优先级（显式 OpenFileLog > 环境变量
 *    ONEQ_FILE_LOG_PATH > 编译期宏 ONEQ_FILE_LOG_PATH）解析并打开文件；
 *  - 打开失败：向 stderr 提示一次并置 open_failed_，后续写入静默丢弃；
 *  - 不注册 atexit：流析构时自动 flush，规避静态析构顺序相关的未定义行为；
 *  - 全路径不抛异常（项目无异常纪律），失败只降级不中断调用方。
 */

#include "common/logging/LogDirectory.h"
#include "common/platform/OneqPlatformCompat.h"
#include "common/logging/ProjectFileLog.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <mutex>
#include <string>

// 编译期默认日志路径（CMake 在 ProjectTargets.cmake 注入；此处兜底）。
#ifndef ONEQ_FILE_LOG_PATH
#define ONEQ_FILE_LOG_PATH "log/1q_library.log"
#endif

namespace oneq {
namespace logging {

namespace {

// 级别 → 行内标签文本（与 spdlog 默认 pattern 的级别名一致）。
const char* LevelName(Level level) {
  switch (level) {
    case Level::kDebug:
      return "debug";
    case Level::kInfo:
      return "info";
    case Level::kWarn:
      return "warn";
    case Level::kError:
      return "error";
    case Level::kCritical:
      return "critical";
  }
  return "info";
}

// 线程安全文件 sink 单例。函数级静态（magic static，C++11 起初始化线程安全）。
class Sink {
 public:
  static Sink& Instance() {
    static Sink instance;
    return instance;
  }

  bool ShouldLog(Level level) const {
    return static_cast<int>(level) >= level_.load(std::memory_order_relaxed);
  }

  void SetLevel(Level level) {
    level_.store(static_cast<int>(level), std::memory_order_relaxed);
  }

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

  void Write(Level level, const std::string& text) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ShouldLog(level)) {
      return;
    }
    if (!stream_.is_open() && !open_failed_) {
      DoOpen(DefaultPath().c_str());  // 懒打开：按路径优先级解析
    }
    if (!stream_.is_open()) {
      return;  // 打开失败后静默丢弃
    }
    stream_ << TimestampPrefix() << LevelName(level) << "] " << text << '\n';
  }

 private:
  Sink() : level_(static_cast<int>(Level::kInfo)) {}

  // 打开路径解析：环境变量优先，回退编译期默认。
  // MSVC 将 std::getenv 标记为弃用（C4996），Windows 侧改用 _dupenv_s。
  static std::string DefaultPath() {
#if defined(_MSC_VER)
    char* buffer = nullptr;
    if (_dupenv_s(&buffer, nullptr, "ONEQ_FILE_LOG_PATH") == 0 && buffer != nullptr) {
      std::string result(buffer);
      std::free(buffer);
      if (!result.empty()) {
        return result;
      }
    }
#else
    const char* env = std::getenv("ONEQ_FILE_LOG_PATH");
    if (env != nullptr && *env != '\0') {
      return std::string(env);
    }
#endif
    return std::string(ONEQ_FILE_LOG_PATH);
  }

  // 实际打开（调用方须已持有 mutex_）。已打开时先 flush+close 再切换。
  void DoOpen(const char* path) {
    if (path == nullptr || *path == '\0') {
      return;
    }
    if (stream_.is_open()) {
      stream_.flush();
      stream_.close();
    }
    oneq::logging::EnsureParentDirectory(path);
    stream_.open(path, std::ios::out | std::ios::app | std::ios::binary);
    if (stream_.is_open()) {
      open_failed_ = false;
      warned_ = false;
    } else {
      open_failed_ = true;
      if (!warned_) {
        warned_ = true;
        std::fprintf(stderr, "[1q file log] cannot open log file: %s\n", path);
        std::fflush(stderr);
      }
    }
  }

  // "[YYYY-MM-DD HH:MM:SS.mmm] [" 前缀（本地时区 + 毫秒），与 spdlog 默认 pattern 同构。
  static std::string TimestampPrefix() {
    using namespace std::chrono;
    const system_clock::time_point now = system_clock::now();
    const std::time_t t = system_clock::to_time_t(now);
    const long long ms =
        duration_cast<milliseconds>(now.time_since_epoch()).count() % 1000;
    std::tm tm_buf;
#if defined(_WIN32)
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif
    char date[32];
    std::strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", &tm_buf);
    char prefix[48];
    ONEQ_SNPRINTF(prefix, sizeof(prefix), "[%s.%03lld] [", date, ms);
    return std::string(prefix);
  }

  mutable std::mutex mutex_;
  std::ofstream stream_;
  std::atomic<int> level_;
  bool open_failed_ = false;
  bool warned_ = false;
};

}  // namespace

// ---------------------------------------------------------------------------
// 宿主 API
// ---------------------------------------------------------------------------

void OpenFileLog(const char* path) { Sink::Instance().Open(path); }

void CloseFileLog() { Sink::Instance().Close(); }

void FlushFileLog() { Sink::Instance().Flush(); }

bool IsFileLogOpen() { return Sink::Instance().IsOpen(); }

void SetFileLogLevel(Level level) { Sink::Instance().SetLevel(level); }

bool ShouldLog(Level level) { return Sink::Instance().ShouldLog(level); }

// ---------------------------------------------------------------------------
// 宏转发入口
// ---------------------------------------------------------------------------
namespace internal {

void Write(Level level, const std::string& text) { Sink::Instance().Write(level, text); }

}  // namespace internal

}  // namespace logging
}  // namespace oneq
