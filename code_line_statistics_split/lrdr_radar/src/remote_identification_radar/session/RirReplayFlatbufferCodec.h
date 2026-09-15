/**
 * @file RirReplayFlatbufferCodec.h
 * @brief 提供远程识别雷达会话回放（replay）相关结构的 FlatBuffers 编解码。
 *
 * 覆盖单周期输入（rir_replay.fbs 的 RirCycleInput）、单周期结果记录 + 会话状态
 * （active_database_version / detection_random_seed）、会话配置与运行期补丁
 * （rir_session_replay.fbs）以及失败标记。所有 Decode* 在解析失败时返回 false
 * 并通过输出参数回填错误描述；逐周期 replay 比较采用字节精确口径。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_REPLAY_FLATBUFFER_CODEC_H_
#define REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_REPLAY_FLATBUFFER_CODEC_H_

#include <string>

#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/replay/ReplayTrace.h"
#include "remote_identification_radar/session/RirReplayCycleRecord.h"

namespace remote_identification_radar {
namespace session {

/** @brief 编解码 RIR 单周期结果与状态记录。 */
std::string EncodeCycleReplayRecordFlatbuffer(const RirCycleReplayRecord& record);
bool DecodeCycleReplayRecordFlatbuffer(const std::string& payload_bytes,
                                       RirCycleReplayRecord* record, std::string* error);

/** @brief 编解码 RIR 单周期输入（cycle_input 事件载荷）。 */
std::string EncodeRirCycleInput(const RirCycleInput& input);
bool DecodeRirCycleInput(const std::string& payload_bytes, RirCycleInput* input,
                         std::string* error);

/** @brief 编解码 RIR 五域会话配置（session_config 事件载荷）。 */
std::string EncodeRirSessionConfig(const config::RirSessionConfig& config);
bool DecodeRirSessionConfig(const std::string& payload_bytes, config::RirSessionConfig* config,
                            std::string* error);

/** @brief 编解码 RIR 运行期配置补丁（runtime_config_patch 事件载荷）。 */
std::string EncodeRirRuntimeConfigPatch(const config::RirRuntimeConfigPatch& patch);
bool DecodeRirRuntimeConfigPatch(const std::string& payload_bytes,
                                 config::RirRuntimeConfigPatch* patch, std::string* error);

/** @brief 编解码 RIR 回放失败标记（failure_marker 事件载荷）。 */
std::string EncodeRirFailureMarker(const oneq::replay::ReplayTraceFailure& failure);
bool DecodeRirFailureMarker(const std::string& payload_bytes,
                            oneq::replay::ReplayTraceFailure* failure, std::string* error);

}  // namespace session
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_REPLAY_FLATBUFFER_CODEC_H_
