#include "1q/electro_optical_sensor/session/EosRecordingSession.h"
#include "1q/electro_optical_sensor/session/EosSession.h"

#include <string>
#include <utility>

#include "1q/replay/ReplayTrace.h"
#include "EosReplayFlatbufferCodec.h"

namespace electro_optical_sensor {
namespace session {

struct EosRecordingSession::Impl {
  Impl(config::EosSessionConfig config, EosRecordingSessionOptions options)
      : session(EosSession::Create(config)),
        replay_writer(std::move(options.replay_writer)) {
    if (replay_writer && options.record_config_on_construct) {
      WriteReplayEvent("session_config", "EosSessionConfig", EncodeEosSessionConfig(config));
    }
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes) const {
    oneq::replay::ReplayTraceEvent ev;
    ev.module = "electro_optical_sensor";
    ev.event_type = event_type;
    ev.payload_type = payload_type;
    ev.payload_encoding = "flatbuffers";
    ev.payload_bytes = payload_bytes;
    replay_writer->WriteEvent(ev);
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes, std::uint32_t cycle_index) const {
    oneq::replay::ReplayTraceEvent ev;
    ev.module = "electro_optical_sensor";
    ev.event_type = event_type;
    ev.payload_type = payload_type;
    ev.payload_encoding = "flatbuffers";
    ev.payload_bytes = payload_bytes;
    ev.has_cycle_index = true;
    ev.cycle_index = cycle_index;
    replay_writer->WriteEvent(ev);
  }

  void WriteValidationFailureMarker(const EosCycleResult& result) const {
    oneq::replay::ReplayTraceFailure failure;
    failure.error_code = "EOS_VALIDATION_ERROR";
    failure.message = "EosCycleResult input validation rejected";
    failure.location = "EosRecordingSession::StepWithResult";
    failure.cycle_index = result.input_cycle_index;
    failure.has_cycle_index = true;
    const std::string failure_bytes = EncodeEosFailureMarker(failure);
    replay_writer->WriteFailureMarker(failure, failure_bytes);
  }

  EosSession session;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer;
  bool pending_input_written{false};
};

EosRecordingSession::EosRecordingSession(config::EosSessionConfig config, EosRecordingSessionOptions options)
    : impl_(new Impl(std::move(config), std::move(options))) {}

EosRecordingSession::~EosRecordingSession() = default;
EosRecordingSession::EosRecordingSession(EosRecordingSession&& other) noexcept : impl_(std::move(other.impl_)) {}
EosRecordingSession& EosRecordingSession::operator=(EosRecordingSession&& other) noexcept { impl_ = std::move(other.impl_); return *this; }

session::EosOutputFrame EosRecordingSession::Step(const EosCycleInput& input) {
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_input", "EosCycleInput", EncodeEosCycleInput(input),
                            input.cycle_index);
    impl_->pending_input_written = true;
  }
  const session::EosCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "EosCycleResult", EncodeEosCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    if (HasValidationError(result.issues)) {
      impl_->WriteValidationFailureMarker(result);
    }
  }
  return result.output_frame;
}

::electro_optical_sensor::session::EosCycleResult EosRecordingSession::StepWithResult(
    const EosCycleInput& input) {
  if (impl_->replay_writer) {
    if (impl_->pending_input_written) {
      // P2-B: 连续两次 cycle_input 无中间 output，记录 warning 后继续
      oneq::replay::ReplayTraceEvent warn_ev;
      warn_ev.module = "electro_optical_sensor";
      warn_ev.event_type = "warning";
      warn_ev.payload_type = "ConsecutiveCycleInputWarning";
      warn_ev.payload_inline = "{\"message\":\"consecutive cycle_input without cycle_output\"}";
      impl_->replay_writer->WriteEvent(warn_ev);
    }
    impl_->WriteReplayEvent("cycle_input", "EosCycleInput", EncodeEosCycleInput(input),
                            input.cycle_index);
    impl_->pending_input_written = true;
  }
  const ::electro_optical_sensor::session::EosCycleResult result =
      impl_->session.StepWithResult(input);
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "EosCycleResult", EncodeEosCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    if (HasValidationError(result.issues)) {
      impl_->WriteValidationFailureMarker(result);
    }
  }
  return result;
}

bool EosRecordingSession::TryApplyRuntimeConfig(const config::EosRuntimeConfigPatch& patch) {
  const bool accepted = impl_->session.TryApplyRuntimeConfig(patch);
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("runtime_config_patch", "EosRuntimeConfigPatch",
                            EncodeEosRuntimeConfigPatch(patch));
  }
  return accepted;
}

EosSession& EosRecordingSession::session() { return impl_->session; }
const EosSession& EosRecordingSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace electro_optical_sensor
