/**
 * @file ProjectLog.h
 * @brief 定义项目统一的日志宏门面，按编译期后端开关在三种实现之间切换：
 *
 *  - PROJECT_LOG_BACKEND_SPDLOG=1（macOS 且 ONEQ_ENABLE_FILE_LOG=ON）：转发到 spdlog 默认 logger；
 *  - PROJECT_LOG_BACKEND_FILE=1（无 spdlog 且 ONEQ_ENABLE_FILE_LOG=ON）：转发到
 *    库内内置文件日志后端 ProjectFileLog，默认落盘 <cwd>/log/
 *    （路径可用 OpenFileLog / 环境变量 ONEQ_FILE_LOG_PATH / 宏 ONEQ_FILE_LOG_PATH 覆盖）；
 *  - 两者皆 0（默认，含 Windows）：全部展开为空操作（ONEQ_ENABLE_FILE_LOG=OFF）。
 *
 * 三个分支的宏名与签名完全一致，调用点无感知；spdlog 分支优先于文件分支。
 * @note 本头仅提供宏，不包含可链接符号；文件后端的可链接符号在 ProjectFileLog.cpp。
 */

#pragma once

#if defined(PROJECT_LOG_BACKEND_SPDLOG) && PROJECT_LOG_BACKEND_SPDLOG

#include <spdlog/spdlog.h>

#define PROJECT_LOG_DEBUG(...) spdlog::debug(__VA_ARGS__)
#define PROJECT_LOG_INFO(...) spdlog::info(__VA_ARGS__)
#define PROJECT_LOG_WARN(...) spdlog::warn(__VA_ARGS__)
#define PROJECT_LOG_ERROR(...) spdlog::error(__VA_ARGS__)
#define PROJECT_LOG_CRITICAL(...) spdlog::critical(__VA_ARGS__)
#define PROJECT_LOG_SHOULD_LOG_DEBUG() spdlog::should_log(spdlog::level::debug)
#define PROJECT_LOG_HAS_DEFAULT_LOGGER() (spdlog::default_logger_raw() != nullptr)
#define PROJECT_LOG_FLUSH_DEFAULT()                \
  do {                                             \
    if (spdlog::default_logger_raw() != nullptr) { \
      spdlog::default_logger_raw()->flush();       \
    }                                              \
  } while (false)

#elif defined(PROJECT_LOG_BACKEND_FILE) && PROJECT_LOG_BACKEND_FILE

// 文件后端：迷你格式化 + 单例 sink，线程安全，无异常（见 ProjectFileLog.h）。
#include "ProjectFileLog.h"

#define PROJECT_LOG_DEBUG(...) oneq::logging::LogDebug(__VA_ARGS__)
#define PROJECT_LOG_INFO(...) oneq::logging::LogInfo(__VA_ARGS__)
#define PROJECT_LOG_WARN(...) oneq::logging::LogWarn(__VA_ARGS__)
#define PROJECT_LOG_ERROR(...) oneq::logging::LogError(__VA_ARGS__)
#define PROJECT_LOG_CRITICAL(...) oneq::logging::LogCritical(__VA_ARGS__)
#define PROJECT_LOG_SHOULD_LOG_DEBUG() oneq::logging::ShouldLog(oneq::logging::Level::kDebug)
#define PROJECT_LOG_HAS_DEFAULT_LOGGER() oneq::logging::IsFileLogOpen()
#define PROJECT_LOG_FLUSH_DEFAULT() oneq::logging::FlushFileLog()

#else

#define PROJECT_LOG_DEBUG(...) ((void)0)
#define PROJECT_LOG_INFO(...) ((void)0)
#define PROJECT_LOG_WARN(...) ((void)0)
#define PROJECT_LOG_ERROR(...) ((void)0)
#define PROJECT_LOG_CRITICAL(...) ((void)0)
#define PROJECT_LOG_SHOULD_LOG_DEBUG() (false)
#define PROJECT_LOG_HAS_DEFAULT_LOGGER() (false)
#define PROJECT_LOG_FLUSH_DEFAULT() ((void)0)

#endif
