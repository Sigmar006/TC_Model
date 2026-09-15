#include "1q/sbirs_sensor/session/SbirsRecordingSession.h"

#include <string>
#include <utility>

#include "1q/replay/ReplayTrace.h"
#include "sbirs_sensor/session/SbirsReplayFlatbufferCodec.h"

namespace sbirs_sensor {
namespace session {

struct SbirsRecordingSession::Impl {
  Impl(config::SbirsSessionConfig session_config, SbirsRecordingSessionOptions session_options)
      : session(SbirsSession::Create(session_config)),
        replay_writer(std::move(session_options.replay_writer)) {
    if (replay_writer && session_options.record_config_on_construct) {
      WriteReplayEvent("session_config", "SbirsSessionConfig",
                       EncodeSbirsSessionConfig(session_config));
    }
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes) const {
    oneq::replay::ReplayTraceEvent event;
    event.module = "sbirs_sensor";
    event.event_type = event_type;
    event.payload_type = payload_type;
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = payload_bytes;
    replay_writer->WriteEvent(event);
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes, std::uint32_t cycle_index) const {
    oneq::replay::ReplayTraceEvent event;
    event.module = "sbirs_sensor";
    event.event_type = event_type;
    event.payload_type = payload_type;
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = payload_bytes;
    event.has_cycle_index = true;
    event.cycle_index = cycle_index;
    replay_writer->WriteEvent(event);
  }

  void WriteValidationFailureMarker(const SbirsCycleResult& result) const {
    oneq::replay::ReplayTraceFailure failure;
    failure.error_code = "SBIRS_VALIDATION_ERROR";
    failure.message = "SbirsCycleResult input validation rejected";
    failure.location = "SbirsRecordingSession::StepWithResult";
    failure.cycle_index = result.input_cycle_index;
    failure.has_cycle_index = true;
    replay_writer->WriteFailureMarker(failure, EncodeSbirsFailureMarker(failure));
  }

  SbirsSession session;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer;
  bool pending_input_written{false};
};

SbirsRecordingSession::SbirsRecordingSession(config::SbirsSessionConfig config,
                                     SbirsRecordingSessionOptions options)
    : impl_(new Impl(std::move(config), std::move(options))) {}

SbirsRecordingSession::~SbirsRecordingSession() = default;
SbirsRecordingSession::SbirsRecordingSession(SbirsRecordingSession&&) noexcept = default;
SbirsRecordingSession& SbirsRecordingSession::operator=(SbirsRecordingSession&&) noexcept = default;

SbirsOutputFrame SbirsRecordingSession::Step(const SbirsCycleInput& input) {
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_input", "SbirsCycleInput", EncodeSbirsCycleInput(input),
                            input.cycle_index);
    impl_->pending_input_written = true;
  }
  const SbirsCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "SbirsCycleResult", EncodeSbirsCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    if (HasValidationError(result.issues)) {
      impl_->WriteValidationFailureMarker(result);
    }
  }
  return result.output_frame;
}

SbirsCycleResult SbirsRecordingSession::StepWithResult(const SbirsCycleInput& input) {
  if (impl_->replay_writer) {
    if (impl_->pending_input_written) {
      oneq::replay::ReplayTraceEvent warning;
      warning.module = "sbirs_sensor";
      warning.event_type = "warning";
      warning.payload_type = "ConsecutiveCycleInputWarning";
      warning.payload_inline = "{\"message\":\"consecutive cycle_input without cycle_output\"}";
      impl_->replay_writer->WriteEvent(warning);
    }
    impl_->WriteReplayEvent("cycle_input", "SbirsCycleInput", EncodeSbirsCycleInput(input),
                            input.cycle_index);
    impl_->pending_input_written = true;
  }
  const SbirsCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "SbirsCycleResult", EncodeSbirsCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    if (HasValidationError(result.issues)) {
      impl_->WriteValidationFailureMarker(result);
    }
  }
  return result;
}

bool SbirsRecordingSession::TryApplyRuntimeConfig(const config::SbirsRuntimeConfigPatch& patch) {
  const bool accepted = impl_->session.TryApplyRuntimeConfig(patch);
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("runtime_config_patch", "SbirsRuntimeConfigPatch",
                            EncodeSbirsRuntimeConfigPatch(patch));
  }
  return accepted;
}

SbirsSession& SbirsRecordingSession::session() { return impl_->session; }
const SbirsSession& SbirsRecordingSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace sbirs_sensor
