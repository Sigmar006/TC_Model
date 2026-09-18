/**
 * @file SbirsAcceptanceLog.h
 * @brief OPIR 功能验收日志宏：普通功能项写入 SatelliteTarget.log。
 *
 * 编译期由 `ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG` 门控（默认 OFF），运行期再由
 * `SbirsSessionConfig::acceptance_log_enabled` 按会话门控。
 * 开启后写入独立验收文件，不进 1q_library.log。关闭时宏与派生计算一并剪除。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_LOG_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_LOG_H_

#include "common/logging/AcceptanceFileLog.h"
#include "common/logging/AcceptanceRecordFormat.h"

namespace sbirs_sensor {
namespace pipeline {

inline bool& SbirsAcceptanceLogRuntimeFlag() {
  static thread_local bool enabled = true;
  return enabled;
}

class SbirsScopedAcceptanceLogEnable final {
 public:
  explicit SbirsScopedAcceptanceLogEnable(bool enabled)
      : previous_(SbirsAcceptanceLogRuntimeFlag()) {
    SbirsAcceptanceLogRuntimeFlag() = enabled;
  }

  ~SbirsScopedAcceptanceLogEnable() { SbirsAcceptanceLogRuntimeFlag() = previous_; }

  SbirsScopedAcceptanceLogEnable(const SbirsScopedAcceptanceLogEnable&) = delete;
  SbirsScopedAcceptanceLogEnable& operator=(const SbirsScopedAcceptanceLogEnable&) = delete;

 private:
  bool previous_;
};

}  // namespace pipeline
}  // namespace sbirs_sensor

#if defined(ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG) && ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG

#define SBIRS_ACCEPTANCE_LOG_ENABLED() \
  (::sbirs_sensor::pipeline::SbirsAcceptanceLogRuntimeFlag())
#define SBIRS_ACCEPTANCE_RECORD(text)                                                  \
  do {                                                                                 \
    if (SBIRS_ACCEPTANCE_LOG_ENABLED()) {                                              \
      ::oneq::logging::WriteAcceptanceLog(                                             \
          ::oneq::logging::AcceptanceChannel::kSatelliteTarget, (text));              \
    }                                                                                  \
  } while (false)
#define SBIRS_ACCEPTANCE_ITEM(sim_time, cycle, item, content) \
  SBIRS_ACCEPTANCE_RECORD(                                    \
      ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))
#define SBIRS_INTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content)                  \
  do {                                                                                 \
    if (SBIRS_ACCEPTANCE_LOG_ENABLED()) {                                              \
      ::oneq::logging::WriteAcceptanceLog(                                             \
          ::oneq::logging::AcceptanceChannel::kIrInternalInterface,                   \
          ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item),          \
                                                 (content)));                          \
    }                                                                                  \
  } while (false)

#else

#define SBIRS_ACCEPTANCE_LOG_ENABLED() false
#define SBIRS_ACCEPTANCE_RECORD(text) ((void)0)
#define SBIRS_ACCEPTANCE_ITEM(sim_time, cycle, item, content) ((void)0)
#define SBIRS_INTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content) ((void)0)

#endif

#if defined(ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG) && ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG

#define SBIRS_FUSION_ACCEPTANCE_LOG_ENABLED() \
  (::sbirs_sensor::pipeline::SbirsAcceptanceLogRuntimeFlag())
#define SBIRS_FUSION_ACCEPTANCE_ITEM(sim_time, cycle, item, content)                   \
  do {                                                                                 \
    if (SBIRS_FUSION_ACCEPTANCE_LOG_ENABLED()) {                                       \
      ::oneq::logging::WriteAcceptanceLog(                                             \
          ::oneq::logging::AcceptanceChannel::kFusion,                                \
          ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item),          \
                                                 (content)));                          \
    }                                                                                  \
  } while (false)

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
