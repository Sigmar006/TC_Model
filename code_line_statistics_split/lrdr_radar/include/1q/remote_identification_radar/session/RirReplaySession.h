/**
 * @file RirReplaySession.h
 * @brief 远程识别雷达 trace 回放入口。
 *
 * 回放会话（按记录的 trace 重放周期输入并校验输出一致性）的主头文件。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_REPLAY_SESSION_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_REPLAY_SESSION_H_

#include <string>

#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"

namespace remote_identification_radar {
namespace session {

/**
 * @brief 单次 trace 回放聚合结果。
 * @note 当 `ok=false` 时，`first_error` 携带首个错误说明；当 trace 命中失败标记时，
 *       `reached_failure_marker=true` 并通过 `failure_marker_*` 字段给出原始标记内容。
 */
struct ONEQ_API RirReplaySessionResult {
  oneq::replay::ReplayTraceReplayReport report{};         /**< trace 兼容性检查报告 */
  oneq::replay::ReplayTracePlaybackResult playback{};     /**< 逐周期回放执行结果 */
  bool ok{false};                                         /**< 回放整体是否成功 */
  bool reached_failure_marker{false};                     /**< 是否在回放过程中命中失败标记 */
  std::string failure_marker_payload{};                   /**< 失败标记原始负载 */
  oneq::replay::ReplayTraceFailure failure_marker_data{}; /**< 解码后的失败标记结构 */
  std::string first_error{};                              /**< 首个错误说明（成功时为空） */
};

/**
 * @brief 回放指定目录下的远程识别雷达 trace 并校验输出一致性。
 *
 * 依次回放 trace 中的会话配置、周期输入、运行期补丁，逐周期执行并与记录输出
 * 按字节比对；首次出现输出不一致即停止。命中失败标记只记录、不中止回放。
 *
 * @param[in] trace_dir trace 目录路径。
 * @return 包含兼容性报告、回放状态与潜在失败标记的聚合结果。
 */
ONEQ_API RirReplaySessionResult ReplayRirTrace(const std::string& trace_dir);

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_REPLAY_SESSION_H_
