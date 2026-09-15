/**
 * @file RirRecordingSession.cpp
 * @brief 远程识别雷达 Replay 记录包装会话实现。
 *
 * 蓝本：`src/airborne_radar/session/ArRecordingSession.cpp`。事件顺序为
 * cycle_input → 执行 → cycle_output，运行期补丁在 apply 之后落盘。
 */

#include "1q/remote_identification_radar/session/RirRecordingSession.h"

#include <string>
#include <utility>

#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "remote_identification_radar/session/RirReplayCycleRecord.h"
#include "remote_identification_radar/session/RirReplayFlatbufferCodec.h"

namespace remote_identification_radar {
namespace session {
namespace {

void WriteReplayEvent(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                      const char* event_type, const char* payload_type,
                      const std::string& payload_bytes, const RirCycleInput& input) {
  if (!writer) {
    return;
  }
  oneq::replay::ReplayTraceEvent event;
  event.module = "remote_identification_radar";
  event.event_type = event_type;
  event.payload_type = payload_type;
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = payload_bytes;
  event.has_cycle_index = true;
  event.cycle_index = input.input_cycle_index;
  event.has_sim_time_sec = true;
  event.sim_time_sec = static_cast<double>(input.sim_time_sec);
  writer->WriteEvent(event);
}

void WriteSessionConfigReplay(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                              const config::RirSessionConfig& config) {
  if (!writer) {
    return;
  }
  oneq::replay::ReplayTraceEvent event;
  event.module = "remote_identification_radar";
  event.event_type = "session_config";
  event.payload_type = "RirSessionConfig";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = EncodeRirSessionConfig(config);
  writer->WriteEvent(event);
}

}  // namespace

struct RirRecordingSession::Impl {
  Impl(RirSession value, std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_trace_writer)
      : session(std::move(value)), replay_writer(std::move(replay_trace_writer)) {}

  RirSession session;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer;
};

RirRecordingSession::RirRecordingSession(const config::RirSessionConfig& config,
                                         RirRecordingSessionOptions options)
    : impl_(new Impl(RirSession::Create(config), std::move(options.replay_writer))) {
  if (options.record_config_on_construct) {
    WriteSessionConfigReplay(impl_->replay_writer, config);
  }
}

RirRecordingSession::RirRecordingSession(RirRecordingSession&& other) noexcept = default;
RirRecordingSession& RirRecordingSession::operator=(RirRecordingSession&& other) noexcept =
    default;
RirRecordingSession::~RirRecordingSession() = default;

RirOutputFrame RirRecordingSession::Step(const RirCycleInput& input) {
  return StepWithResult(input).output_frame;
}

RirCycleResult RirRecordingSession::StepWithResult(const RirCycleInput& input) {
  WriteReplayEvent(impl_->replay_writer, "cycle_input", "RirCycleInput",
                   EncodeRirCycleInput(input), input);

  const RirCycleResult result = impl_->session.StepWithResult(input);
  RirCycleReplayRecord record;
  record.result = result;
  record.session_state = RirSessionReplayAccess::CaptureSessionState(impl_->session);
  WriteReplayEvent(impl_->replay_writer, "cycle_output", "RirCycleReplayRecordV3",
                   EncodeCycleReplayRecordFlatbuffer(record), input);
  return result;
}

bool RirRecordingSession::TryApplyRuntimeConfig(const config::RirRuntimeConfigPatch& patch) {
  const bool accepted = impl_->session.TryApplyRuntimeConfig(patch);
  if (impl_->replay_writer) {
    oneq::replay::ReplayTraceEvent event;
    event.module = "remote_identification_radar";
    event.event_type = "runtime_config_patch";
    event.payload_type = "RirRuntimeConfigPatch";
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = EncodeRirRuntimeConfigPatch(patch);
    impl_->replay_writer->WriteEvent(event);
  }
  return accepted;
}

const RirSession& RirRecordingSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace remote_identification_radar
