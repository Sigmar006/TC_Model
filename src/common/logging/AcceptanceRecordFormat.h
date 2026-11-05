/**
 * @file AcceptanceRecordFormat.h
 * @brief 验收日志一行四段格式（仿真时间 / 仿真周期 / 测试项 / 内容）。
 */

#ifndef ONEQ_SRC_COMMON_LOGGING_ACCEPTANCE_RECORD_FORMAT_H_
#define ONEQ_SRC_COMMON_LOGGING_ACCEPTANCE_RECORD_FORMAT_H_

#include <cstdint>
#include "common/platform/OneqPlatformCompat.h"
#include <cstdio>
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

inline std::string FormatAcceptanceLine(float sim_time_sec, std::uint32_t cycle, const char* item,
                                        const std::string& content) {
  char time_buf[32];
  ONEQ_SNPRINTF(time_buf, sizeof(time_buf), "%.3f", static_cast<double>(sim_time_sec));
  std::string line;
  line.reserve(64U + (item != nullptr ? std::char_traits<char>::length(item) : 0U) + content.size());
  line += "仿真时间=";
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
