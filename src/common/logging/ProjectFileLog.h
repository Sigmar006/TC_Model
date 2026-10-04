/**
 * @file ProjectFileLog.h
 * @brief 库内内置文件日志后端：PROJECT_LOG_* 在无 spdlog 平台（Windows）的落盘实现。
 *
 * 设计要点：
 *  - 纯 C++11（VS2015 可编译）：不使用任何 C++14/17 写法，不引入 Boost；
 *  - 无异常纪律：本后端任何路径都不抛出异常，文件打开失败仅向 stderr 提示一次；
 *  - 线程安全：sink 内部以 std::mutex 串行化 open/write/flush/close；
 *  - 迷你格式化：仅支持调用点实际使用的两种占位符形态 "{}" 与 "{:.Nf}"，
 *    其余形态按字面文本输出，绝不崩溃；与 spdlog/fmt 的语义差异见
 *    docs/practice/output_view_and_logging_guide.md。
 *
 * 路径解析优先级（懒打开时生效）：
 *   1) OpenFileLog(path) 显式指定；
 *   2) 环境变量 ONEQ_FILE_LOG_PATH（仅打开时读取一次）；
 *   3) 编译期宏 ONEQ_FILE_LOG_PATH（CMake 默认注入 <cwd>/log/ 下的文件名）。
 *
 * 本头为内部头（src/ 下，不随安装导出），仅由 ProjectLog.h 的 FILE 分支与
 * 单元测试包含。
 */

#pragma once

#include <cstdio>
#include "common/platform/OneqPlatformCompat.h"
#include <cstring>
#include <string>
#include <type_traits>

namespace oneq {
namespace logging {

// 级别枚举：与 spdlog 级别一一对应，默认最低级别为 kInfo（镜像 spdlog 默认 logger）。
enum class Level {
  kDebug = 0,
  kInfo = 1,
  kWarn = 2,
  kError = 3,
  kCritical = 4,
};

// ---------------------------------------------------------------------------
// 宿主 API（实现在 ProjectFileLog.cpp）
// ---------------------------------------------------------------------------

// 显式打开日志文件；已打开时先关闭旧文件再打开新路径。path 为空时忽略。
void OpenFileLog(const char* path);

// 关闭日志文件（先 flush）；未打开时为空操作。
void CloseFileLog();

// 立即把缓冲内容写入磁盘；未打开时为空操作。
void FlushFileLog();

// 日志文件当前是否已打开（显式打开或懒打开成功之后为 true）。
bool IsFileLogOpen();

// 设置最低输出级别；低于该级别的消息不落盘。默认 kInfo。
void SetFileLogLevel(Level level);

// level 是否达到当前最低输出级别（格式化前的快速过滤）。
bool ShouldLog(Level level);

// ---------------------------------------------------------------------------
// 宏转发入口（实现在 ProjectFileLog.cpp）
// ---------------------------------------------------------------------------
namespace internal {

// 把格式化完成的一行写入 sink（内部实现，宏门面经此落盘）。
void Write(Level level, const std::string& text);

// ---------------------------------------------------------------------------
// 迷你格式化引擎（仅头文件模板，C++11）
// ---------------------------------------------------------------------------

// 单值默认表示。重载集合设计（MSVC 对同形参模板偏序判定严格，需保证任意实参
// 至多命中一个模板）：
//  - 非模板精确重载（字符串/字符/bool）优先于一切模板；
//  - 类别模板（整型有符号/无符号、浮点、枚举）共享 "(std::string&, T)" 签名形状，
//    但 enable_if 互斥，同一实参至多一个可实例化，天然无歧义；
//  - 指针模板使用独立的 "(std::string&, const T*)" 形状，与按值模板不构成竞争；
//  - 兜底模板以 enable_if 显式排除全部已覆盖类别（整型/浮点/枚举/指针/字符串/
//    字符/bool），只在无任何类别命中时可实例化，因此任意未覆盖类型只会输出
//    "<unprintable>" 而不会编译失败。
inline void AppendValue(std::string& out, const std::string& v) { out += v; }

inline void AppendValue(std::string& out, const char* v) {
  out += (v != nullptr) ? v : "(null)";
}

inline void AppendValue(std::string& out, char v) { out.push_back(v); }

inline void AppendValue(std::string& out, bool v) { out += v ? "true" : "false"; }

template <typename T>
typename std::enable_if<std::is_integral<T>::value && std::is_signed<T>::value, void>::type
AppendValue(std::string& out, T v) {
  char buf[32];
  ONEQ_SNPRINTF(buf, sizeof(buf), "%lld", static_cast<long long>(v));
  out += buf;
}

template <typename T>
typename std::enable_if<std::is_integral<T>::value && std::is_unsigned<T>::value, void>::type
AppendValue(std::string& out, T v) {
  char buf[32];
  ONEQ_SNPRINTF(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(v));
  out += buf;
}

template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, void>::type
AppendValue(std::string& out, T v) {
  // 默认形态：%g（6 位有效数字）。与 fmt 的"最短往返表示"存在微小差异，
  // 仅影响浮点打印精度，对调试日志可接受（见文档）。
  char buf[64];
  ONEQ_SNPRINTF(buf, sizeof(buf), "%g", static_cast<double>(v));
  out += buf;
}

template <typename T>
typename std::enable_if<std::is_enum<T>::value, void>::type
AppendValue(std::string& out, T v) {
  AppendValue(out, static_cast<typename std::underlying_type<T>::type>(v));
}

// 指针（const char* 已被非模板重载截获，此处覆盖其余指针类型）。
template <typename T>
void AppendValue(std::string& out, const T* v) {
  char buf[32];
  ONEQ_SNPRINTF(buf, sizeof(buf), "%p", static_cast<const void*>(v));
  out += buf;
}

// 兜底：显式排除所有已覆盖类别，保证只对未覆盖类型实例化。
template <typename T>
typename std::enable_if<
    !std::is_integral<T>::value && !std::is_floating_point<T>::value &&
        !std::is_enum<T>::value && !std::is_pointer<T>::value &&
        !std::is_same<typename std::decay<T>::type, std::string>::value &&
        !std::is_same<typename std::decay<T>::type, char>::value &&
        !std::is_same<typename std::decay<T>::type, bool>::value,
    void>::type
AppendValue(std::string& out, const T&) {
  out += "<unprintable>";
}

// "{:.Nf}" 精度形态：仅对浮点生效，其余类型回退到默认表示。
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, void>::type
AppendFixed(std::string& out, T v, int precision) {
  char buf[64];
  ONEQ_SNPRINTF(buf, sizeof(buf), "%.*f", precision, static_cast<double>(v));
  out += buf;
}

template <typename T>
typename std::enable_if<!std::is_floating_point<T>::value, void>::type
AppendFixed(std::string& out, T v, int precision) {
  (void)precision;
  AppendValue(out, v);
}

// 字面段复制：处理 fmt 风格的 "{{" / "}}" 转义（各输出一个花括号），
// 与 spdlog/fmt 行为一致；孤立花括号原样输出（防御性，不崩溃）。
inline void AppendLiteral(std::string& out, const char* begin, const char* end) {
  while (begin < end) {
    const char c = *begin;
    if (c == '{' && begin + 1 < end && *(begin + 1) == '{') {
      out.push_back('{');
      begin += 2;
    } else if (c == '}' && begin + 1 < end && *(begin + 1) == '}') {
      out.push_back('}');
      begin += 2;
    } else {
      out.push_back(c);
      ++begin;
    }
  }
}

// 无剩余参数：fmt 剩余部分整体按字面追加（占位符缺参时不崩溃，原样输出）。
inline void FormatOne(std::string& out, const char* fmt) {
  AppendLiteral(out, fmt, fmt + std::strlen(fmt));
}

// 递归展开一个占位符后继续处理剩余参数（C++11 递归变参，无 fold expression）。
template <typename T, typename... Rest>
void FormatOne(std::string& out, const char* fmt, const T& value, const Rest&... rest) {
  const char* brace = std::strchr(fmt, '{');
  if (brace == nullptr) {
    AppendLiteral(out, fmt, fmt + std::strlen(fmt));  // 无更多占位符：剩余参数忽略
    return;
  }
  AppendLiteral(out, fmt, brace);  // 字面前缀（含转义处理）
  const char* p = brace + 1;
  if (*p == '{') {
    // "{{" 转义：输出单个 '{'，不消费参数
    out.push_back('{');
    FormatOne(out, p + 1, value, rest...);
    return;
  }
  if (*p == '}') {
    // 裸形态 "{}"
    AppendValue(out, value);
    FormatOne(out, p + 1, rest...);
    return;
  }
  if (*p == ':' && *(p + 1) == '.') {
    // 精度形态 "{:.Nf}"
    const char* q = p + 2;
    if (*q >= '0' && *q <= '9') {
      int precision = 0;
      while (*q >= '0' && *q <= '9') {
        precision = precision * 10 + (*q - '0');
        ++q;
      }
      if (*q == 'f' && *(q + 1) == '}') {
        AppendFixed(out, value, precision);
        FormatOne(out, q + 2, rest...);  // 跳过 'f}' 两个字符
        return;
      }
    }
  }
  // 未知占位符形态：整体按字面输出，并消费一个参数，保证后续占位符与参数对齐。
  const char* close = std::strchr(p, '}');
  if (close == nullptr) {
    out.append(brace);  // 未闭合的 '{'：余下全部按字面
    return;
  }
  out.append(brace, static_cast<std::size_t>(close - brace + 1));
  FormatOne(out, close + 1, rest...);
}

}  // namespace internal

// 统一的级别 + 格式化 + 落盘入口（宏门面经此转发）。
template <typename... Args>
inline void LogMessage(Level level, const char* format, const Args&... args) {
  if (!ShouldLog(level)) {
    return;  // 低于最低级别的消息在格式化前直接丢弃
  }
  std::string text;
  text.reserve(128);
  internal::FormatOne(text, format, args...);
  internal::Write(level, text);
}

// 五个级别的便捷转发（PROJECT_LOG_* 宏的 FILE 分支映射目标）。
template <typename... Args>
inline void LogDebug(const char* format, const Args&... args) {
  LogMessage(Level::kDebug, format, args...);
}

template <typename... Args>
inline void LogInfo(const char* format, const Args&... args) {
  LogMessage(Level::kInfo, format, args...);
}

template <typename... Args>
inline void LogWarn(const char* format, const Args&... args) {
  LogMessage(Level::kWarn, format, args...);
}

template <typename... Args>
inline void LogError(const char* format, const Args&... args) {
  LogMessage(Level::kError, format, args...);
}

template <typename... Args>
inline void LogCritical(const char* format, const Args&... args) {
  LogMessage(Level::kCritical, format, args...);
}

}  // namespace logging
}  // namespace oneq
