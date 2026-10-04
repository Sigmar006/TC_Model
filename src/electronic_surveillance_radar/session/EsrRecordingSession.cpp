#include "1q/electronic_surveillance_radar/session/EsrRecordingSession.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"

#include <string>
#include <utility>

#include "1q/replay/ReplayTrace.h"
#include "EsrReplayFlatbufferCodec.h"

namespace electronic_surveillance_radar {
namespace session {

struct EsrRecordingSession::Impl {
  Impl(config::EsrSessionConfig config, EsrRecordingSessionOptions options)
      : session(EsrSession::Create(config)),
        replay_writer(std::move(options.replay_writer)) {
    if (replay_writer && options.record_config_on_construct) {
      WriteReplayEvent("session_config", "EsrSessionConfig", EncodeEsrSessionConfig(config));
    }
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes) const {
    oneq::replay::ReplayTraceEvent ev;
    ev.module = "electronic_surveillance_radar";
    ev.event_type = event_type;
    ev.payload_type = payload_type;
    ev.payload_encoding = "flatbuffers";
    ev.payload_bytes = payload_bytes;
    replay_writer->WriteEvent(ev);
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes, std::uint32_t cycle_index) const {
    oneq::replay::ReplayTraceEvent ev;
    ev.module = "electronic_surveillance_radar";
    ev.event_type = event_type;
    ev.payload_type = payload_type;
    ev.payload_encoding = "flatbuffers";
    ev.payload_bytes = payload_bytes;
    ev.has_cycle_index = true;
    ev.cycle_index = cycle_index;
    replay_writer->WriteEvent(ev);
  }

  void WriteValidationFailureMarker(const EsrCycleResult& result) const {
    oneq::replay::ReplayTraceFailure failure;
    failure.error_code = "ESR_VALIDATION_ERROR";
    failure.message = "EsrCycleResult input validation rejected";
    failure.location = "EsrRecordingSession::StepWithResult";
    failure.cycle_index = result.input_cycle_index;
    failure.has_cycle_index = true;
    const std::string failure_bytes = EncodeEsrFailureMarker(failure);
    replay_writer->WriteFailureMarker(failure, failure_bytes);
  }

  EsrSession session;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer;
  bool pending_input_written{false};
};

EsrRecordingSession::EsrRecordingSession(config::EsrSessionConfig config, EsrRecordingSessionOptions options)
    : impl_(new Impl(std::move(config), std::move(options))) {}

EsrRecordingSession::~EsrRecordingSession() = default;
EsrRecordingSession::EsrRecordingSession(EsrRecordingSession&& other) noexcept : impl_(std::move(other.impl_)) {}
EsrRecordingSession& EsrRecordingSession::operator=(EsrRecordingSession&& other) noexcept { impl_ = std::move(other.impl_); return *this; }

session::EsrOutputFrame EsrRecordingSession::Step(const EsrCycleInput& input) {
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_input", "EsrCycleInput", EncodeEsrCycleInput(input),
                            input.cycle_index);
    impl_->pending_input_written = true;
  }
  const session::EsrCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "EsrCycleResult", EncodeEsrCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    if (HasValidationError(result.issues)) {
      impl_->WriteValidationFailureMarker(result);
    }
  }
  return result.output_frame;
}

EsrCycleResult EsrRecordingSession::StepWithResult(const EsrCycleInput& input) {
  if (impl_->replay_writer) {
    if (impl_->pending_input_written) {
      oneq::replay::ReplayTraceEvent warn;
      warn.module = "electronic_surveillance_radar";
      warn.event_type = "warning";
      warn.payload_type = "ConsecutiveCycleInputWarning";
      warn.payload_inline = "{\"message\":\"consecutive cycle_input without cycle_output\"}";
      impl_->replay_writer->WriteEvent(warn);
    }
    impl_->WriteReplayEvent("cycle_input", "EsrCycleInput", EncodeEsrCycleInput(input),
                            input.cycle_index);
    impl_->pending_input_written = true;
  }
  const EsrCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "EsrCycleResult", EncodeEsrCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    if (HasValidationError(result.issues)) {
      impl_->WriteValidationFailureMarker(result);
    }
  }
  return result;
}

EsrRuntimeConfigApplyResult EsrRecordingSession::TryApplyRuntimeConfig(
    const config::EsrRuntimeConfigPatch& patch) {
  const EsrRuntimeConfigApplyResult result =
      impl_->session.ApplyRuntimeConfigWithResult(patch);
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent(
        "runtime_config_patch", "EsrRuntimeConfigPatchEvent",
        EncodeEsrRuntimeConfigPatchEvent(patch, result));
  }
  return result;
}

EsrSession& EsrRecordingSession::session() { return impl_->session; }
const EsrSession& EsrRecordingSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace electronic_surveillance_radar
