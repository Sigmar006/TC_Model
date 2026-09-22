/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsAcceptanceLog.
 * Model: SBIRS infrared sensor.
 * Layer: pipeline.
 * File role: interface declarations and data contracts.
 * Responsibility: wide-field search, pointing, acquisition, and tracking processing.
 * Upstream boundary: cycle scene, cue, sensor state, and resolved physical parameters.
 * Downstream boundary: detections, estimates, diagnostics, and lifecycle observations.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

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
#define SBIRS_INTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content)                \
  ::oneq::logging::WriteAcceptanceLog(                                               \
      ::oneq::logging::AcceptanceChannel::kIrInternalInterface,                     \
      ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))

#else

#define SBIRS_ACCEPTANCE_LOG_ENABLED() false
#define SBIRS_ACCEPTANCE_RECORD(text) ((void)0)
#define SBIRS_ACCEPTANCE_ITEM(sim_time, cycle, item, content) ((void)0)
#define SBIRS_INTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content) ((void)0)

#endif

#if defined(ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG) && ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG

#define SBIRS_FUSION_ACCEPTANCE_LOG_ENABLED() true
#define SBIRS_FUSION_ACCEPTANCE_ITEM(sim_time, cycle, item, content) \
  ::oneq::logging::WriteAcceptanceLog(                              \
      ::oneq::logging::AcceptanceChannel::kFusion,                  \
      ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))

#else

#define SBIRS_FUSION_ACCEPTANCE_LOG_ENABLED() false
#define SBIRS_FUSION_ACCEPTANCE_ITEM(sim_time, cycle, item, content) ((void)0)

#endif

#define SBIRS_DUAL_ACCEPTANCE_LOG_ENABLED()                                      \
  (SBIRS_ACCEPTANCE_LOG_ENABLED() || SBIRS_FUSION_ACCEPTANCE_LOG_ENABLED())
#define SBIRS_DUAL_ACCEPTANCE_ITEM(sim_time, cycle, item, content)          \
  do {                                                                      \
    SBIRS_ACCEPTANCE_ITEM((sim_time), (cycle), (item), (content));           \
    SBIRS_FUSION_ACCEPTANCE_ITEM((sim_time), (cycle), (item), (content));     \
  } while (false)

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_LOG_H_
