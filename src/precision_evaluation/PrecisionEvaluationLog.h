/**
 * @file PrecisionEvaluationLog.h
 * @brief 精度评估验收日志宏：按项写入 log/SatelliteTarget.log（四段同一行）。
 *
 * 2026-09-28 归并：精度评估不再生成独立验收文件，原
 * `ONEQ_ENABLE_PRECISION_EVALUATION_LOG` 开关保留为功能控制，落盘通道统一为
 * SatelliteTarget。
 */

#ifndef ONEQ_SRC_PRECISION_EVALUATION_PRECISION_EVALUATION_LOG_H_
#define ONEQ_SRC_PRECISION_EVALUATION_PRECISION_EVALUATION_LOG_H_

#include "common/logging/AcceptanceFileLog.h"
#include "common/logging/AcceptanceRecordFormat.h"

#if defined(ONEQ_ENABLE_PRECISION_EVALUATION_LOG) && ONEQ_ENABLE_PRECISION_EVALUATION_LOG

#define PRECISION_EVAL_LOG_ENABLED() true
#define PRECISION_EVAL_RECORD(text) \
  ::oneq::logging::WriteAcceptanceLog( \
      ::oneq::logging::AcceptanceChannel::kSatelliteTarget, (text))
#define PRECISION_EVAL_ITEM(sim_time, cycle, item, content) \
  PRECISION_EVAL_RECORD(::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))

#else

#define PRECISION_EVAL_LOG_ENABLED() false
#define PRECISION_EVAL_RECORD(text) ((void)0)
#define PRECISION_EVAL_ITEM(sim_time, cycle, item, content) ((void)0)

#endif

#endif  // ONEQ_SRC_PRECISION_EVALUATION_PRECISION_EVALUATION_LOG_H_
