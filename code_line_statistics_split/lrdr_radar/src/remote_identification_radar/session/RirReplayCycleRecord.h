/**
 * @file RirReplayCycleRecord.h
 * @brief 远程识别雷达 replay 记录类型与窄访问适配器（内部）。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_REPLAY_CYCLE_RECORD_H_
#define REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_REPLAY_CYCLE_RECORD_H_

#include <cstdint>
#include <string>

#include "1q/remote_identification_radar/session/RirCycleResult.h"

namespace remote_identification_radar {
namespace session {

/** @brief replay 比对所需的会话拥有状态。 */
struct RirSessionReplayState {
  std::string active_database_version{}; /**< 当前生效识别特征库版本；未加载识别库时为空串。 */
  std::uint32_t detection_random_seed{
      42U}; /**< 检测/量测误差随机种子（阶段 2-S 起入 replay 状态）。 */
};

/** @brief 单周期用户门面一次调用的结果与调用后状态。 */
struct RirCycleReplayRecord {
  RirCycleResult result{};
  RirSessionReplayState session_state{};
};

class RirSession;

/** @brief 仅供 RIR replay 包装器读取 RirSession 内部快照的窄访问适配器。 */
class RirSessionReplayAccess {
 public:
  static RirSessionReplayState CaptureSessionState(const RirSession& session);
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_REPLAY_CYCLE_RECORD_H_
