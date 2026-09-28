/**
 * @file RirAcceptanceLog.h
 * @brief RIR 功能验收日志宏：普通功能项写入 RadarOutput.log，外部接口项写入
 *        RadarExternalInterface.log，内部接口项写入 RadarInternalInterface.log。
 *
 * 编译期由 `ONEQ_ENABLE_RIR_ACCEPTANCE_LOG` 门控（默认 OFF），运行期再由
 * `RirSessionConfig::acceptance_log_enabled` 按会话门控。
 * 开启后写入独立验收文件，不进 1q_library.log。关闭时宏与派生计算一并剪除。
 */

#ifndef ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_ACCEPTANCE_LOG_H_
#define ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_ACCEPTANCE_LOG_H_

#include "common/logging/AcceptanceFileLog.h"
#include "common/logging/AcceptanceRecordFormat.h"

namespace remote_identification_radar {
namespace runtime {

inline bool& RirAcceptanceLogRuntimeFlag() {
  static thread_local bool enabled = true;
  return enabled;
}

class RirScopedAcceptanceLogEnable final {
 public:
  explicit RirScopedAcceptanceLogEnable(bool enabled)
      : previous_(RirAcceptanceLogRuntimeFlag()) {
    RirAcceptanceLogRuntimeFlag() = enabled;
  }

  ~RirScopedAcceptanceLogEnable() { RirAcceptanceLogRuntimeFlag() = previous_; }

  RirScopedAcceptanceLogEnable(const RirScopedAcceptanceLogEnable&) = delete;
  RirScopedAcceptanceLogEnable& operator=(const RirScopedAcceptanceLogEnable&) = delete;

 private:
  bool previous_;
};

}  // namespace runtime
}  // namespace remote_identification_radar

#if defined(ONEQ_ENABLE_RIR_ACCEPTANCE_LOG) && ONEQ_ENABLE_RIR_ACCEPTANCE_LOG

#define RIR_ACCEPTANCE_LOG_ENABLED() \
  (::remote_identification_radar::runtime::RirAcceptanceLogRuntimeFlag())
#define RIR_ACCEPTANCE_RECORD(text)                                                    \
  do {                                                                                 \
    if (RIR_ACCEPTANCE_LOG_ENABLED()) {                                                \
      ::oneq::logging::WriteAcceptanceLog(                                             \
          ::oneq::logging::AcceptanceChannel::kRadarOutput, (text));                  \
    }                                                                                  \
  } while (false)
#define RIR_ACCEPTANCE_ITEM(sim_time, cycle, item, content) \
  RIR_ACCEPTANCE_RECORD(::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))
#define RIR_INTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content)                    \
  do {                                                                                 \
    if (RIR_ACCEPTANCE_LOG_ENABLED()) {                                                \
      ::oneq::logging::WriteAcceptanceLog(                                             \
          ::oneq::logging::AcceptanceChannel::kRadarInternalInterface,                \
          ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item),          \
                                                 (content)));                          \
    }                                                                                  \
  } while (false)
#define RIR_EXTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content)                    \
  do {                                                                                 \
    if (RIR_ACCEPTANCE_LOG_ENABLED()) {                                                \
      ::oneq::logging::WriteAcceptanceLog(                                             \
          ::oneq::logging::AcceptanceChannel::kRadarExternalInterface,                \
          ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item),          \
                                                 (content)));                          \
    }                                                                                  \
  } while (false)

#else

#define RIR_ACCEPTANCE_LOG_ENABLED() false
#define RIR_ACCEPTANCE_RECORD(text) ((void)0)
#define RIR_ACCEPTANCE_ITEM(sim_time, cycle, item, content) ((void)0)
#define RIR_INTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content) ((void)0)
#define RIR_EXTERNAL_INTERFACE_ITEM(sim_time, cycle, item, content) ((void)0)

#endif

#endif  // ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_ACCEPTANCE_LOG_H_
