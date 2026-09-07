/**
 * @file SbirsReplayFlatbufferCodec.h
 * @brief SBIRS-inspired replay payload 的 FlatBuffers encode/decode 接口。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_SESSION_SBIRS_REPLAY_FLATBUFFER_CODEC_H_
#define ONEQ_SRC_SBIRS_SENSOR_SESSION_SBIRS_REPLAY_FLATBUFFER_CODEC_H_

#include <string>

#include "1q/replay/ReplayTrace.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief 回放负载版本标识（v5，2026-09-07：SbirsDetectionRecord 增加 measurement_time_s，
 * 任务配置增加 initial_scan_phase_deg）。
 * @note Encode 写入、Decode 校验：v1 时代（deg/ECEF 语义，无标识符）、v2（"SBI2"，
 * 含 has_* 标志字段）、v3（"SBI3"，多通道调度配置）、v4（"SBI4"，无时间戳）
 * 与任何异源负载按标识符不符显式拒绝，避免跨版本静默误解码。
 */
constexpr char kSbirsReplayFileIdentifier[] = "SBI5";

/** @brief 将单周期输入编码为 FlatBuffers payload 字节串。 */
std::string EncodeSbirsCycleInput(const SbirsCycleInput& value);
/** @brief 将原始输出帧编码为 FlatBuffers payload 字节串。 */
std::string EncodeSbirsOutputFrame(const SbirsOutputFrame& value);
/** @brief 将结构化周期结果编码为 FlatBuffers payload 字节串。 */
std::string EncodeSbirsCycleResult(const SbirsCycleResult& value);
/** @brief 将会话配置编码为 FlatBuffers payload 字节串。 */
std::string EncodeSbirsSessionConfig(const config::SbirsSessionConfig& value);
/** @brief 将 runtime patch 编码为 FlatBuffers payload 字节串。 */
std::string EncodeSbirsRuntimeConfigPatch(const config::SbirsRuntimeConfigPatch& value);
/** @brief 将 failure marker 编码为 FlatBuffers payload 字节串。 */
std::string EncodeSbirsFailureMarker(const oneq::replay::ReplayTraceFailure& failure);

/**
 * @brief 解码单周期输入。
 * @param[in] bytes payload 字节串
 * @param[out] out 接收解码结果
 * @return 解码成功返回 true，否则返回 false
 */
bool DecodeSbirsCycleInput(const std::string& bytes, SbirsCycleInput* out);
/**
 * @brief 解码原始输出帧。
 * @param[in] bytes payload 字节串
 * @param[out] out 接收解码结果
 * @return 解码成功返回 true，否则返回 false
 */
bool DecodeSbirsOutputFrame(const std::string& bytes, SbirsOutputFrame* out);
/**
 * @brief 解码结构化周期结果。
 * @param[in] bytes payload 字节串
 * @param[out] out 接收解码结果
 * @return 解码成功返回 true，否则返回 false
 */
bool DecodeSbirsCycleResult(const std::string& bytes, SbirsCycleResult* out);
/**
 * @brief 解码会话配置。
 * @param[in] bytes payload 字节串
 * @param[out] out 接收解码结果
 * @return 解码成功返回 true，否则返回 false
 */
bool DecodeSbirsSessionConfig(const std::string& bytes, config::SbirsSessionConfig* out);
/**
 * @brief 解码 runtime patch。
 * @param[in] bytes payload 字节串
 * @param[out] out 接收解码结果
 * @return 解码成功返回 true，否则返回 false
 */
bool DecodeSbirsRuntimeConfigPatch(const std::string& bytes, config::SbirsRuntimeConfigPatch* out);
/**
 * @brief 解码 failure marker。
 * @param[in] bytes payload 字节串
 * @param[out] failure 接收解码出的 failure marker
 * @param[out] error 接收解码错误信息
 * @return 解码成功返回 true，否则返回 false 并写入 error
 */
bool DecodeSbirsFailureMarker(const std::string& bytes, oneq::replay::ReplayTraceFailure* failure,
                              std::string* error);

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_SESSION_SBIRS_REPLAY_FLATBUFFER_CODEC_H_
