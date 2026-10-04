#include "1q/sar/session/SarRecordingSession.h"
#include "1q/sar/session/SarSession.h"

#include <string>
#include <utility>

#include "1q/replay/ReplayTrace.h"
#include "SarReplayFlatbufferCodec.h"
#include "1q/sar/session/SarInputValidation.h"
#include "sar/session/SarDiagnosticUtils.h"
#include "sar/session/SarRawHistoryBuilder.h"

namespace sar {
namespace session {

struct SarRecordingSession::Impl {
  Impl(SarSession initial_session, SarRecordingSessionOptions session_options)
      : session(std::move(initial_session)),
        replay_writer(std::move(session_options.replay_writer)) {}

  Impl(config::SarSessionConfig config, SarRecordingSessionOptions session_options)
      : session(SarSession::Create(config)),
        replay_writer(std::move(session_options.replay_writer)) {
    if (replay_writer && session_options.record_config_on_construct) {
      WriteReplayEvent("session_config", "SarSessionConfig", EncodeSarSessionConfig(config));
    }
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes) const {
    oneq::replay::ReplayTraceEvent event;
    event.module = "sar";
    event.event_type = event_type;
    event.payload_type = payload_type;
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = payload_bytes;
    replay_writer->WriteEvent(event);
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes, std::uint32_t cycle_index) const {
    oneq::replay::ReplayTraceEvent event;
    event.module = "sar";
    event.event_type = event_type;
    event.payload_type = payload_type;
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = payload_bytes;
    event.has_cycle_index = true;
    event.cycle_index = cycle_index;
    replay_writer->WriteEvent(event);
  }

  void WriteFailureMarker(const SarCycleResult& result) const {
    oneq::replay::ReplayTraceFailure failure;
    const char* reason_tag = AbortReasonToDiagnosticCode(result.abort_reason);
    failure.error_code = reason_tag[0] != '\0' ? reason_tag : "sar_error";
    failure.message = "SarCycleResult cycle rejected";
    failure.has_cycle_index = true;
    failure.cycle_index = result.input_cycle_index;
    replay_writer->WriteFailureMarker(failure);
  }

  SarSession session;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool pending_input_written{false};
};

SarRecordingSession::SarRecordingSession() : impl_(new Impl(SarSession{}, SarRecordingSessionOptions{})) {}

SarRecordingSession::SarRecordingSession(SarSession session)
    : impl_(new Impl(std::move(session), SarRecordingSessionOptions{})) {}

SarRecordingSession::SarRecordingSession(config::SarSessionConfig config, SarRecordingSessionOptions options)
    : impl_(new Impl(std::move(config), std::move(options))) {}

SarRecordingSession::~SarRecordingSession() = default;
SarRecordingSession::SarRecordingSession(SarRecordingSession&& other) noexcept : impl_(std::move(other.impl_)) {}
SarRecordingSession& SarRecordingSession::operator=(SarRecordingSession&& other) noexcept { impl_ = std::move(other.impl_); return *this; }

SarCycleProduct SarRecordingSession::Step(const SarCycleInput& input) {
  // 规则 15c：产品层从同一份周期记录移动取出，非第二份副本。
  SarCycleResult result = StepWithResult(input);
  return std::move(result.product);
}

SarCycleResult SarRecordingSession::StepWithResult(const SarCycleInput& input) {
  if (impl_->replay_writer) {
    if (impl_->pending_input_written) {
      oneq::replay::ReplayTraceEvent warning;
      warning.module = "sar";
      warning.event_type = "warning";
      warning.payload_type = "ConsecutiveCycleInputWarning";
      warning.payload_inline = "{\"message\":\"consecutive cycle_input without cycle_output\"}";
      impl_->replay_writer->WriteEvent(warning);
    }
    impl_->WriteReplayEvent("cycle_input", "SarCycleInput", EncodeSarCycleInput(input),
                            input.cycle_index);
    impl_->pending_input_written = true;
  }

  const SarCycleResult result = impl_->session.StepWithResult(input);

  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "SarCycleResult", EncodeSarCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    // 失败周期 marker（规则 14 可推导字段 has_error 已删除）：status 为校验/执行拒绝
    // 时写 failure marker；关机（kPoweredOff）是合法非执行状态，不写。
    const bool cycle_rejected = result.status == session::SarCycleStatus::kRejectedInvalidInput ||
                                result.status == session::SarCycleStatus::kRejectedExecution;
    if (cycle_rejected) {
      impl_->WriteFailureMarker(result);
    }
  }
  return result;
}

bool SarRecordingSession::TryApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch) {
  const bool accepted = impl_->session.TryApplyRuntimeConfig(patch);
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("runtime_config_patch", "SarRuntimeConfigPatch",
                            EncodeSarRuntimeConfigPatch(patch));
  }
  return accepted;
}

SarSession& SarRecordingSession::session() { return impl_->session; }
const SarSession& SarRecordingSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace sar
