#include "1q/electronic_countermeasure/EcmRecordingSession.h"

#include <string>
#include <utility>

#include "1q/replay/ReplayTrace.h"
#include "electronic_countermeasure/EcmReplayFlatbufferCodec.h"

namespace electronic_countermeasure {
namespace session {

struct EcmRecordingSession::Impl {
  Impl(config::EcmSessionConfig config, EcmRecordingSessionOptions options)
      : session(EcmSession::Create(config)),
        replay_writer(std::move(options.replay_writer)) {
    if (replay_writer && options.record_config_on_construct) {
      WriteEvent("session_config", "EcmSessionConfig", EncodeEcmSessionConfig(config), false, 0U);
    }
  }

  void WriteEvent(const std::string& event_type, const std::string& payload_type,
                  const std::string& payload, bool has_cycle, std::uint32_t cycle_index) const {
    oneq::replay::ReplayTraceEvent event;
    event.module = "electronic_countermeasure";
    event.event_type = event_type;
    event.payload_type = payload_type;
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = payload;
    event.has_cycle_index = has_cycle;
    event.cycle_index = cycle_index;
    replay_writer->WriteEvent(event);
  }

  EcmSession session;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer;
};

EcmRecordingSession::EcmRecordingSession(config::EcmSessionConfig config,
                                 EcmRecordingSessionOptions options)
    : impl_(new Impl(std::move(config), std::move(options))) {}
EcmRecordingSession::~EcmRecordingSession() = default;
EcmRecordingSession::EcmRecordingSession(EcmRecordingSession&& other) noexcept : impl_(std::move(other.impl_)) {}
EcmRecordingSession& EcmRecordingSession::operator=(EcmRecordingSession&& other) noexcept { impl_ = std::move(other.impl_); return *this; }

EcmCycleResult EcmRecordingSession::StepWithResult(const EcmCycleInput& input) {
  if (impl_->replay_writer) {
    impl_->WriteEvent("cycle_input", "EcmCycleInput", EncodeEcmCycleInput(input), true,
                      input.cycle_index);
  }
  const EcmCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->replay_writer) {
    impl_->WriteEvent("cycle_output", "EcmCycleResult", EncodeEcmCycleResult(result), true,
                      input.cycle_index);
  }
  return result;
}

EcmRuntimeConfigApplyResult EcmRecordingSession::ApplyRuntimeConfig(
    const config::EcmRuntimeConfigPatch& patch) {
  if (impl_->replay_writer) {
    impl_->WriteEvent("runtime_config_patch", "EcmRuntimeConfigPatch",
                      EncodeEcmRuntimeConfigPatch(patch), false, 0U);
  }
  return impl_->session.ApplyRuntimeConfig(patch);
}

EcmSession& EcmRecordingSession::session() { return impl_->session; }
const EcmSession& EcmRecordingSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace electronic_countermeasure
