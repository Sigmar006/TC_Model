/**
 * @file SbirsAcceptanceLog.h
 * @brief OPIR 验收日志宏：按项写入 opir_acceptance.log（四段同一行）。
 *
 * 编译期由 `ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG` 门控（默认 OFF）。
 * 开启后写入独立验收文件，不进 1q_library.log。关闭时宏与派生计算一并剪除。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_LOG_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_LOG_H_

#include "common/logging/AcceptanceFileLog.h"
#include "common/logging/AcceptanceRecordFormat.h"

#if defined(ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG) && ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG

#define SBIRS_ACCEPTANCE_LOG_ENABLED() true
#define SBIRS_ACCEPTANCE_RECORD(text) \
  ::oneq::logging::WriteAcceptanceLog(::oneq::logging::AcceptanceChannel::kOpir, (text))
#define SBIRS_ACCEPTANCE_ITEM(sim_time, cycle, item, content) \
  SBIRS_ACCEPTANCE_RECORD(                                    \
      ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))

#else

#define SBIRS_ACCEPTANCE_LOG_ENABLED() false
#define SBIRS_ACCEPTANCE_RECORD(text) ((void)0)
#define SBIRS_ACCEPTANCE_ITEM(sim_time, cycle, item, content) ((void)0)

#endif

#if defined(ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG) && ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG

#define SBIRS_FUSION_ACCEPTANCE_LOG_ENABLED() true
#define SBIRS_FUSION_ACCEPTANCE_RECORD(text) \
  ::oneq::logging::WriteAcceptanceLog(::oneq::logging::AcceptanceChannel::kFusion, (text))
#define SBIRS_FUSION_ACCEPTANCE_ITEM(sim_time, cycle, item, content) \
  SBIRS_FUSION_ACCEPTANCE_RECORD(                                    \
      ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))

#else

#define SBIRS_FUSION_ACCEPTANCE_LOG_ENABLED() false
#define SBIRS_FUSION_ACCEPTANCE_RECORD(text) ((void)0)
#define SBIRS_FUSION_ACCEPTANCE_ITEM(sim_time, cycle, item, content) ((void)0)

#endif

#define SBIRS_DUAL_ACCEPTANCE_LOG_ENABLED() \
  (SBIRS_ACCEPTANCE_LOG_ENABLED() || SBIRS_FUSION_ACCEPTANCE_LOG_ENABLED())
#define SBIRS_DUAL_ACCEPTANCE_ITEM(sim_time, cycle, item, content)        \
  do {                                                                    \
    SBIRS_ACCEPTANCE_ITEM((sim_time), (cycle), (item), (content));        \
    SBIRS_FUSION_ACCEPTANCE_ITEM((sim_time), (cycle), (item), (content)); \
  } while (false)

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_LOG_H_
