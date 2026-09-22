/**
 * @file ReplayFlatbufferCodecSupport.h
 * @brief 模块 replay codec 共用的内部 FlatBuffers 机械基元。
 *
 * 只处理已完成 builder 的字节复制和统一 FailureMarker 的解码保护；各模块继续拥有
 * 自己的 schema、payload type、DTO 映射和 field identifier。
 */

#ifndef ONEQ_COMMON_REPLAY_REPLAY_FLATBUFFER_CODEC_SUPPORT_H_
#define ONEQ_COMMON_REPLAY_REPLAY_FLATBUFFER_CODEC_SUPPORT_H_

#include <cstdint>
#include <string>

#include "1q/replay/ReplayTrace.h"
#include "flatbuffers/flatbuffers.h"

namespace oneq {
namespace common {
namespace replay {

inline std::string CopyFinishedFlatbuffer(const flatbuffers::FlatBufferBuilder& builder) {
  const std::uint8_t* const buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer), builder.GetSize());
}

template <typename FailureMarker>
bool DecodeFailureMarkerPayload(const std::string& payload_bytes,
                                ::oneq::replay::ReplayTraceFailure* failure,
                                std::string* error) {
  if (failure == nullptr) {
    if (error != nullptr) {
      *error = "null FailureMarker output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty FailureMarker flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* const data =
      reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  const FailureMarker* const root = flatbuffers::GetRoot<FailureMarker>(data);
  if (root == nullptr || !root->Verify(verifier)) {
    if (error != nullptr) {
      *error = "invalid FailureMarker flatbuffers payload";
    }
    return false;
  }

  failure->error_code = root->error_code() ? root->error_code()->str() : "";
  failure->message = root->message() ? root->message()->str() : "";
  failure->location = root->location() ? root->location()->str() : "";
  failure->has_cycle_index = root->has_cycle_index();
  failure->cycle_index = root->cycle_index();
  failure->has_sim_time_sec = root->has_sim_time_sec();
  failure->sim_time_sec = root->sim_time_sec();
  failure->diagnostics_payload = root->diagnostics() ? root->diagnostics()->str() : "{}";
  return true;
}

}  // namespace replay
}  // namespace common
}  // namespace oneq

#endif  // ONEQ_COMMON_REPLAY_REPLAY_FLATBUFFER_CODEC_SUPPORT_H_
