/*
 * Supplemental architecture notes for acceptance review.
 * Unit: RirAcceptanceLog.
 * Model: LRDR remote identification radar.
 * Layer: runtime.
 * File role: interface declarations and data contracts.
 * Responsibility: top-level RF, detection, tracking, and recognition orchestration.
 * Upstream boundary: validated cycle input, dwell plan, runtime policy, and environment.
 * Downstream boundary: cycle products, emissions, diagnostics, and attribution snapshots.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

/**
 * @file RirAcceptanceLog.h
 * @brief RIR 验收日志宏：按项写入 rir_acceptance.log（四段同一行）。
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
  ::oneq::logging::WriteAcceptanceLog(::oneq::logging::AcceptanceChannel::kRir, (text))
#define RIR_ACCEPTANCE_ITEM(sim_time, cycle, item, content) \
  RIR_ACCEPTANCE_RECORD(::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))
#define RIR_INTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content)                   \
  ::oneq::logging::WriteAcceptanceLog(                                               \
      ::oneq::logging::AcceptanceChannel::kRadarInternalInterface,                  \
      ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))
#define RIR_EXTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content)                   \
  ::oneq::logging::WriteAcceptanceLog(                                               \
      ::oneq::logging::AcceptanceChannel::kRadarInterface,                          \
      ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))

#else

#define RIR_ACCEPTANCE_LOG_ENABLED() false
#define RIR_ACCEPTANCE_RECORD(text) ((void)0)
#define RIR_ACCEPTANCE_ITEM(sim_time, cycle, item, content) ((void)0)
#define RIR_INTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content) ((void)0)
#define RIR_EXTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content) ((void)0)

#endif

#endif  // ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_ACCEPTANCE_LOG_H_
