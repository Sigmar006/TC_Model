/**
 * @file AcceptanceRecordFormat.h
 * @brief 验收日志一行四段格式（仿真时间 / 仿真周期 / 测试项 / 内容）。
 */

#ifndef ONEQ_SRC_COMMON_LOGGING_ACCEPTANCE_RECORD_FORMAT_H_
#define ONEQ_SRC_COMMON_LOGGING_ACCEPTANCE_RECORD_FORMAT_H_

#include <cstdint>
#include "common/platform/OneqPlatformCompat.h"
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <string>

namespace oneq {
namespace logging {

/** 拼出验收日志一行。item 为人读验收项名；content 为验收内容（无数据时调用方省略整行）。 */
namespace detail {

// 中译：**大幅面预警扫描模式下的措辞改写**（2026-11-05 甲方要求）——该模式下所有行里
//   "宽视场/窄视场"字样统一改写为"大幅面预警扫描"，避免外部看到同一模式出现三种波束名。
//   模式由 Sensors 侧每周期写入进程环境变量 `ONEQ_SBIRS_WIDE_AREA`（同进程共享，
//   跨 DLL 可用）；未设置或非 "1" 时原样输出，其它模式行为逐位不变。
// 标识：大幅面模式措辞改写（2026-11-05）。
inline bool WideAreaRewriteActive() {
  const char* flag = std::getenv("ONEQ_SBIRS_WIDE_AREA");
  return flag != nullptr && flag[0] == '1';
}

inline void RewriteBeamWording(std::string* text) {
  if (text == nullptr || text->empty() || !WideAreaRewriteActive()) {
    return;
  }
  const char* const kPairs[][2] = {{"宽视场", "大幅面"}, {"窄视场", "大幅面"}};
  for (std::size_t i = 0U; i < 2U; ++i) {
    const std::string from(kPairs[i][0]);
    std::string::size_type pos = text->find(from);
    while (pos != std::string::npos) {
      text->replace(pos, from.size(), kPairs[i][1]);
      pos = text->find(from, pos + 3);
    }
  }
}

}  // namespace detail

namespace detail {

// 中译：**真实系统时间前缀**（2026-11-06 甲方要求）——每行验收记录在"仿真时间/仿真周期"之前
//   追加写盘当时的本机时间（年-月-日 时:分:秒），便于外部按真实时间归档、定位与外部系统对账；
//   仿真时间仍是模型内部的物理时刻，两者并存不冲突。Windows 用 localtime_s，POSIX 用 localtime_r。
// 标识：验收行系统时间戳（2026-11-06）。
inline std::string FormatSystemTimestamp() {
  const std::time_t now = std::time(nullptr);
  std::tm local_tm{};
#if defined(_WIN32)
  if (localtime_s(&local_tm, &now) != 0) {
    return std::string("1970-01-01 00:00:00");
  }
#else
  if (localtime_r(&now, &local_tm) == nullptr) {
    return std::string("1970-01-01 00:00:00");
  }
#endif
  char buffer[32];
  const std::size_t written = std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &local_tm);
  return std::string(buffer, written);
}

}  // namespace detail

inline std::string FormatAcceptanceLine(float sim_time_sec, std::uint32_t cycle, const char* item,
                                        const std::string& content) {
  char time_buf[32];
  ONEQ_SNPRINTF(time_buf, sizeof(time_buf), "%.3f", static_cast<double>(sim_time_sec));
  std::string line;
  line.reserve(64U + (item != nullptr ? std::char_traits<char>::length(item) : 0U) + content.size());
  // 过程：行首追加真实系统时间戳（本机墙钟），与仿真时间/周期并列。
  line += "[";
  line += detail::FormatSystemTimestamp();
  line += "] 仿真时间=";
  line += time_buf;
  line += "s 仿真周期=";
  char cycle_buf[16];
  ONEQ_SNPRINTF(cycle_buf, sizeof(cycle_buf), "%u", cycle);
  line += cycle_buf;
  // 规范行格式（验收判定标准 §0）：方括号内为规范测试项名称，不写「验收项」三字，
  // 方括号后也不写「验收内容：」。
  // 中译：**按当前模式改写波束措辞**（2026-11-05）——大幅面预警扫描模式下，"宽视场/窄视场"
  //   统一改写为"大幅面"，保证同一模式对外只有一种波束名（含项名与行内字段/统计口径）。
  //   未启用大幅面模式时不做任何替换，其它模式行为逐位不变。
  // 标识：大幅面模式措辞统一（2026-11-05）。
  std::string item_text(item != nullptr ? item : "");
  std::string content_text(content);
  if (detail::WideAreaRewriteActive()) {
    detail::RewriteBeamWording(&item_text);
    detail::RewriteBeamWording(&content_text);
  }
  line += " [";
  line += item_text;
  line += "] ";
  line += content_text;
  return line;
}

}  // namespace logging
}  // namespace oneq

#endif
