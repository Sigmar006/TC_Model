/**
 * @file EsrReplayFlatbufferCodec.h
 * @brief ESR replay payload 的 FlatBuffers encode/decode 接口。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_REPLAY_FLATBUFFER_CODEC_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_REPLAY_FLATBUFFER_CODEC_H_

#include <string>

#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/replay/ReplayTrace.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief 回放负载版本标识（2026-08-24 起写入并校验；2026-09-06 RF 标识统一后升级）。
 * @note Encode 写入、Decode 校验：此前编码的负载不带标识符，与任何异源负载
 *       一并按标识符不符显式拒绝，避免跨版本/异源静默误解码（对齐 SBIRS
 *       SBI3 护栏先例）。
 */
constexpr char kEsrReplayFileIdentifier[] = "ESRD";
constexpr char kEsrSessionReplayFileIdentifier[] = "ESRT";

/**
 * @brief 将 EsrCycleInput 序列化为 FlatBuffers payload。
 * @param[in] value 待序列化的单周期输入。
 * @return 编码后的二进制字符串。
 */
std::string EncodeEsrCycleInput(const EsrCycleInput& value);

/**
 * @brief 将 EsrOutputFrame 序列化为 FlatBuffers payload。
 * @param[in] value 待序列化的输出帧。
 * @return 编码后的二进制字符串。
 */
std::string EncodeEsrOutputFrame(const session::EsrOutputFrame& value);

/**
 * @brief 将 EsrCycleResult 序列化为 FlatBuffers payload。
 * @param[in] value 待序列化的单周期结果。
 * @return 编码后的二进制字符串。
 */
std::string EncodeEsrCycleResult(const EsrCycleResult& value);

/**
 * @brief 将 EsrSessionConfig 序列化为 FlatBuffers payload。
 * @param[in] value 待序列化的会话配置。
 * @return 编码后的二进制字符串。
 */
std::string EncodeEsrSessionConfig(const config::EsrSessionConfig& value);

/**
 * @brief 将 EsrRuntimeConfigPatch 序列化为 FlatBuffers payload。
 * @param[in] value 待序列化的运行期补丁。
 * @return 编码后的二进制字符串。
 */
std::string EncodeEsrRuntimeConfigPatch(const config::EsrRuntimeConfigPatch& value);

/** @brief 编码一次运行期补丁及其结构化应用结果。 */
std::string EncodeEsrRuntimeConfigPatchEvent(
    const config::EsrRuntimeConfigPatch& patch,
    const EsrRuntimeConfigApplyResult& result);

/**
 * @brief 将 ReplayTraceFailure 失败标记序列化为 FlatBuffers payload。
 * @param[in] failure 待序列化的失败标记结构。
 * @return 编码后的二进制字符串。
 */
std::string EncodeEsrFailureMarker(const oneq::replay::ReplayTraceFailure& failure);

/**
 * @brief 从 FlatBuffers payload 反序列化 EsrCycleInput。
 * @param[in] bytes 编码后的二进制字符串。
 * @param[out] out 输出反序列化结果。
 * @return 解码成功返回 true；payload 非法返回 false。
 */
bool DecodeEsrCycleInput(const std::string& bytes, EsrCycleInput* out);

/**
 * @brief 从 FlatBuffers payload 反序列化 EsrOutputFrame。
 * @param[in] bytes 编码后的二进制字符串。
 * @param[out] out 输出反序列化结果。
 * @return 解码成功返回 true；payload 非法返回 false。
 */
bool DecodeEsrOutputFrame(const std::string& bytes, session::EsrOutputFrame* out);

/**
 * @brief 从 FlatBuffers payload 反序列化 EsrCycleResult。
 * @param[in] bytes 编码后的二进制字符串。
 * @param[out] out 输出反序列化结果。
 * @return 解码成功返回 true；payload 非法返回 false。
 */
bool DecodeEsrCycleResult(const std::string& bytes, EsrCycleResult* out);

/**
 * @brief 从 FlatBuffers payload 反序列化 EsrSessionConfig。
 * @param[in] bytes 编码后的二进制字符串。
 * @param[out] out 输出反序列化结果。
 * @return 解码成功返回 true；payload 非法返回 false。
 */
bool DecodeEsrSessionConfig(const std::string& bytes, config::EsrSessionConfig* out);

/**
 * @brief 从 FlatBuffers payload 反序列化 EsrRuntimeConfigPatch。
 * @param[in] bytes 编码后的二进制字符串。
 * @param[out] out 输出反序列化结果。
 * @return 解码成功返回 true；payload 非法返回 false。
 */
bool DecodeEsrRuntimeConfigPatch(const std::string& bytes, config::EsrRuntimeConfigPatch* out);

/** @brief 原子解码运行期补丁事件。 */
bool DecodeEsrRuntimeConfigPatchEvent(
    const std::string& bytes, config::EsrRuntimeConfigPatch* patch,
    EsrRuntimeConfigApplyResult* result);

/**
 * @brief 从 FlatBuffers payload 反序列化失败标记。
 * @param[in] bytes 编码后的二进制字符串。
 * @param[out] failure 输出反序列化的失败标记结构。
 * @param[out] error 解码失败时的错误描述。
 * @return 解码成功返回 true；失败返回 false 并写入 @p error。
 */
bool DecodeEsrFailureMarker(const std::string& bytes, oneq::replay::ReplayTraceFailure* failure,
                            std::string* error);

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_REPLAY_FLATBUFFER_CODEC_H_
