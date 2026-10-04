#include "1q/airborne_radar/session/ArRecordingSession.h"

#include <string>
#include <utility>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "airborne_radar/session/ArReplayCycleRecord.h"
#include "airborne_radar/session/ArReplayFlatbufferCodec.h"

namespace airborne_radar {
namespace session {
namespace {

void WriteReplayEvent(
    const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
    const char* event_type, const char* payload_type,
    const std::string& payload_bytes, const ArCycleInput& input) {
  if (!writer) {
    return;
  }
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = event_type;
  event.payload_type = payload_type;
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = payload_bytes;
  event.has_cycle_index = true;
  event.cycle_index = input.cycle_index;
  event.has_sim_time_sec = true;
  event.sim_time_sec = input.cycle_start_time_s;
  writer->WriteEvent(event);
}

void WriteSessionConfigReplay(
    const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
    const config::ArSessionConfig& config) {
  if (!writer) {
    return;
  }
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = "session_config";
  event.payload_type = "ArSessionConfig";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = EncodeSessionConfigFlatbuffer(config);
  writer->WriteEvent(event);
}

}  // namespace

struct ArRecordingSession::Impl {
  Impl(ArSession value,
       std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_trace_writer)
      : session(std::move(value)),
        replay_writer(std::move(replay_trace_writer)) {}

  ArSession session;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer;
};

ArRecordingSession::ArRecordingSession(const config::ArSessionConfig& config,
                                       ArRecordingSessionOptions options)
    : impl_(new Impl(ArSession::Create(config),
                     std::move(options.replay_writer))) {
  if (options.record_config_on_construct) {
    WriteSessionConfigReplay(impl_->replay_writer, config);
  }
}

ArRecordingSession::ArRecordingSession(ArRecordingSession&& other) noexcept : impl_(std::move(other.impl_)) {}
ArRecordingSession& ArRecordingSession::operator=(ArRecordingSession&& other) noexcept { impl_ = std::move(other.impl_); return *this; }
ArRecordingSession::~ArRecordingSession() = default;

TrackOutputFrame ArRecordingSession::Step(const ArCycleInput& input) {
  return StepWithResult(input).output_frame;
}

ArCycleResult ArRecordingSession::StepWithResult(const ArCycleInput& input) {
  WriteReplayEvent(impl_->replay_writer, "cycle_input", "ArCycleInputV3",
                   EncodeCycleInputFlatbuffer(input), input);

  const ArCycleResult result = impl_->session.StepWithResult(input);
  ArCycleReplayRecord record;
  record.result = result;
  record.session_state =
      ArSessionReplayAccess::CaptureSessionState(impl_->session);
  WriteReplayEvent(impl_->replay_writer, "cycle_output",
                   "ArCycleReplayRecordV3",
                   EncodeCycleReplayRecordFlatbuffer(record), input);
  return result;
}

bool ArRecordingSession::TryApplyRuntimeConfig(
    const config::ArRuntimeConfigPatch& patch) {
  const bool accepted = impl_->session.TryApplyRuntimeConfig(patch);
  if (impl_->replay_writer) {
    oneq::replay::ReplayTraceEvent event;
    event.module = "airborne_radar";
    event.event_type = "runtime_config_patch";
    event.payload_type = "ArRuntimeConfigAttemptV3";
    event.payload_encoding = "flatbuffers";
    event.payload_bytes =
        EncodeRuntimeConfigAttemptFlatbuffer(patch, accepted);
    impl_->replay_writer->WriteEvent(event);
  }
  return accepted;
}

session::ExternalDecisionSubmitStatus ArRecordingSession::SubmitExternalDecision(
    session::ExternalDecisionOverride override_decision) {
  // 先序列化 profile 值（移动前），再按返回状态决定是否写回放事件。
  const std::string replay_payload =
      impl_->replay_writer ? EncodeArControlProfileFlatbuffer(override_decision.profile)
                           : std::string{};
  const session::ExternalDecisionSubmitStatus status =
      impl_->session.SubmitExternalDecision(std::move(override_decision));
  if (status == session::ExternalDecisionSubmitStatus::kAccepted && impl_->replay_writer) {
    oneq::replay::ReplayTraceEvent event;
    event.module = "airborne_radar";
    event.event_type = "decision_input";
    event.payload_type = "ArControlProfilePayload";
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = replay_payload;
    impl_->replay_writer->WriteEvent(event);
  }
  return status;
}

const ArSession& ArRecordingSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace airborne_radar
