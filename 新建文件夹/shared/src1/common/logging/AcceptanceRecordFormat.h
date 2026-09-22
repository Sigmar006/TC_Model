/**
 * @file AcceptanceRecordFormat.h
 * @brief 验收日志一行四段格式（仿真时间 / 仿真周期 / 测试项 / 内容）。
 */

#ifndef ONEQ_SRC_COMMON_LOGGING_ACCEPTANCE_RECORD_FORMAT_H_
#define ONEQ_SRC_COMMON_LOGGING_ACCEPTANCE_RECORD_FORMAT_H_

#include <cstdint>
#include <cstdio>
#include <string>

namespace oneq {
namespace logging {

/** 拼出验收日志一行。item 为人读验收项名；content 为验收内容（无数据时调用方省略整行）。 */
inline std::string FormatAcceptanceLine(float sim_time_sec, std::uint32_t cycle, const char* item,
                                        const std::string& content) {
  char time_buf[32];
  std::snprintf(time_buf, sizeof(time_buf), "%.3f", static_cast<double>(sim_time_sec));
  std::string line;
  line.reserve(64U + (item != nullptr ? std::char_traits<char>::length(item) : 0U) + content.size());
  line += "仿真时间=";
  line += time_buf;
  line += "s 仿真周期=";
  char cycle_buf[16];
  std::snprintf(cycle_buf, sizeof(cycle_buf), "%u", cycle);
  line += cycle_buf;
  // 规范行格式（验收判定标准 §0）：方括号内为规范测试项名称，不写「验收项」三字，
  // 方括号后也不写「验收内容：」。
  line += " [";
  line += (item != nullptr ? item : "");
  line += "] ";
  line += content;
  return line;
}

}  // namespace logging
}  // namespace oneq

#endif
