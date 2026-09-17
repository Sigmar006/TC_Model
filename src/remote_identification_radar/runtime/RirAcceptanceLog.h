/**
 * @file RirAcceptanceLog.h
 * @brief RIR 功能验收日志宏：普通功能与外部接口项写入 RadarOutput.log。
 *
 * 编译期由 `ONEQ_ENABLE_RIR_ACCEPTANCE_LOG` 门控（默认 OFF）。
 * 开启后写入独立验收文件，不进 1q_library.log。关闭时宏与派生计算一并剪除。
 */

#ifndef ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_ACCEPTANCE_LOG_H_
#define ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_ACCEPTANCE_LOG_H_

#include "common/logging/AcceptanceFileLog.h"
#include "common/logging/AcceptanceRecordFormat.h"

#if defined(ONEQ_ENABLE_RIR_ACCEPTANCE_LOG) && ONEQ_ENABLE_RIR_ACCEPTANCE_LOG

#define RIR_ACCEPTANCE_LOG_ENABLED() true
#define RIR_ACCEPTANCE_RECORD(text) \
  ::oneq::logging::WriteAcceptanceLog(::oneq::logging::AcceptanceChannel::kRadarOutput, (text))
#define RIR_ACCEPTANCE_ITEM(sim_time, cycle, item, content) \
  RIR_ACCEPTANCE_RECORD(::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))
#define RIR_PERFORMANCE_ITEM(sim_time, cycle, item, content)                         \
  ::oneq::logging::WriteAcceptanceLog(                                               \
      ::oneq::logging::AcceptanceChannel::kRir,                                     \
      ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))
#define RIR_INTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content)                   \
  ::oneq::logging::WriteAcceptanceLog(                                               \
      ::oneq::logging::AcceptanceChannel::kRadarInternalInterface,                  \
      ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))
#define RIR_EXTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content)                   \
  ::oneq::logging::WriteAcceptanceLog(                                               \
      ::oneq::logging::AcceptanceChannel::kRadarOutput,                             \
      ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))

#else

#define RIR_ACCEPTANCE_LOG_ENABLED() false
#define RIR_ACCEPTANCE_RECORD(text) ((void)0)
#define RIR_ACCEPTANCE_ITEM(sim_time, cycle, item, content) ((void)0)
#define RIR_PERFORMANCE_ITEM(sim_time, cycle, item, content) ((void)0)
#define RIR_INTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content) ((void)0)
#define RIR_EXTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content) ((void)0)

#endif

#endif  // ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_ACCEPTANCE_LOG_H_
