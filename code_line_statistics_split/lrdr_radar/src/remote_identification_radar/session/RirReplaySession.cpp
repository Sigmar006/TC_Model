/**
 * @file RirReplaySession.cpp
 * @brief 远程识别雷达 trace 回放实现。
 *
 * 蓝本：`src/airborne_radar/session/ArReplaySession.cpp`。回放侧按记录顺序重建
 * 会话、应用运行期补丁、逐周期执行，并以 `RirCycleReplayRecordV3` 字节比对判定分叉。
 */

#include "1q/remote_identification_radar/session/RirReplaySession.h"

#include <memory>
#include <string>

#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "remote_identification_radar/session/RirReplayCycleRecord.h"
#include "remote_identification_radar/session/RirReplayFlatbufferCodec.h"

namespace remote_identification_radar {
namespace session {
namespace {

struct RirReplayState {
  std::unique_ptr<RirSession> session{};
  bool has_pending_cycle{false};
  RirCycleInput pending_input{};
  bool reached_failure_marker{false};
  std::string failure_marker_payload{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
};

bool OnSessionConfig(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  if (event.payload_type != "RirSessionConfig") {
    *error = "RIR replay expected RirSessionConfig session_config";
    return false;
  }
  RirReplayState* state = static_cast<RirReplayState*>(user_data);
  if (state->session) {
    *error = "RIR replay received duplicate session_config";
    return false;
  }
  config::RirSessionConfig config;
  if (!DecodeRirSessionConfig(event.payload_bytes, &config, error)) {
    return false;
  }
  state->session.reset(new RirSession(RirSession::Create(config)));
  return true;
}

bool OnCycleInput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                  std::string* error) {
  RirReplayState* state = static_cast<RirReplayState*>(user_data);
  if (!state->session) {
    *error = "RIR replay received cycle_input before session_config";
    return false;
  }
  if (state->has_pending_cycle) {
    *error = "RIR replay received consecutive cycle_input without cycle_output";
    return false;
  }
  if (event.payload_type != "RirCycleInput") {
    *error = "RIR replay rejects unknown cycle_input payload type: " + event.payload_type;
    return false;
  }
  if (!DecodeRirCycleInput(event.payload_bytes, &state->pending_input, error)) {
    return false;
  }
  state->has_pending_cycle = true;
  return true;
}

bool OnRuntimeConfigPatch(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                          std::string* error) {
  if (event.payload_type != "RirRuntimeConfigPatch") {
    *error =
        "RIR replay rejects unknown runtime_config_patch payload type: " + event.payload_type;
    return false;
  }
  RirReplayState* state = static_cast<RirReplayState*>(user_data);
  if (!state->session) {
    *error = "RIR replay received runtime_config_patch before session_config";
    return false;
  }
  config::RirRuntimeConfigPatch patch;
  if (!DecodeRirRuntimeConfigPatch(event.payload_bytes, &patch, error)) {
    return false;
  }
  // 记录侧只落盘补丁内容（不含 accepted 标记），回放侧按同一补丁重放，
  // 接受与否由会话自身判定，不参与分叉判据。
  state->session->TryApplyRuntimeConfig(patch);
  return true;
}

oneq::replay::ReplayTraceOutputStatus OnCycleOutput(
    const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
    std::string* actual_output, std::string* error) {
  RirReplayState* state = static_cast<RirReplayState*>(user_data);
  if (!state->session) {
    *error = "RIR replay received cycle_output before session_config";
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  if (!state->has_pending_cycle) {
    *error = "RIR replay cycle_output arrived before cycle_input";
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  if (event.payload_type != "RirCycleReplayRecordV3") {
    *error = "RIR replay expected RirCycleReplayRecordV3 cycle_output";
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  RirCycleReplayRecord expected;
  if (!DecodeCycleReplayRecordFlatbuffer(event.payload_bytes, &expected, error)) {
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  RirCycleReplayRecord actual;
  actual.result = state->session->StepWithResult(state->pending_input);
  actual.session_state = RirSessionReplayAccess::CaptureSessionState(*state->session);
  state->has_pending_cycle = false;
  const std::string actual_payload = EncodeCycleReplayRecordFlatbuffer(actual);
  if (event.payload_bytes == actual_payload) {
    actual_output->clear();
    return oneq::replay::ReplayTraceOutputStatus::kHandledByModule;
  }
  // 识别输出条目数分叉优先报告（定位更直接），其余情况回落到字节规模摘要。
  if (expected.result.output_frame.recognition_outputs.size() !=
      actual.result.output_frame.recognition_outputs.size()) {
    *actual_output =
        "{\"payload_type\":\"RirCycleReplayRecordV3\",\"actual_size\":" +
        std::to_string(actual_payload.size()) + ",\"expected_recognition_output_count\":" +
        std::to_string(expected.result.output_frame.recognition_outputs.size()) +
        ",\"actual_recognition_output_count\":" +
        std::to_string(actual.result.output_frame.recognition_outputs.size()) + "}";
    *error = "RIR replay recognition output count divergence";
    return oneq::replay::ReplayTraceOutputStatus::kDivergence;
  }
  *actual_output = "{\"payload_type\":\"RirCycleReplayRecordV3\",\"actual_size\":" +
                   std::to_string(actual_payload.size()) + "}";
  *error = "RIR replay output divergence (RirCycleReplayRecordV3)";
  return oneq::replay::ReplayTraceOutputStatus::kDivergence;
}

bool OnFailureMarker(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  RirReplayState* state = static_cast<RirReplayState*>(user_data);
  state->reached_failure_marker = true;
  state->failure_marker_payload = event.payload_bytes;
  return DecodeRirFailureMarker(event.payload_bytes, &state->failure_marker_data, error);
}

}  // namespace

RirReplaySessionResult ReplayRirTrace(const std::string& trace_dir) {
  RirReplaySessionResult result;
  oneq::replay::ReplayTraceCompatibilityExpectation expectation;
  expectation.module = "remote_identification_radar";
  expectation.require_module_match = true;
  result.report = oneq::replay::BuildReplayTraceReport(trace_dir, expectation);
  if (!result.report.replay_ready) {
    result.first_error = result.report.first_error;
    return result;
  }

  RirReplayState state;
  oneq::replay::ReplayTracePlaybackCallbacks callbacks;
  callbacks.user_data = &state;
  callbacks.on_session_config = OnSessionConfig;
  callbacks.on_cycle_input = OnCycleInput;
  callbacks.on_runtime_config_patch = OnRuntimeConfigPatch;
  callbacks.on_cycle_output = OnCycleOutput;
  callbacks.on_failure_marker = OnFailureMarker;

  oneq::replay::ReplayTracePlaybackOptions options;
  options.require_output_callback = true;
  options.stop_on_first_divergence = true;
  options.stop_on_failure_marker = false;
  result.playback = oneq::replay::PlaybackReplayTrace(trace_dir, callbacks, options);
  result.ok = result.playback.ok;
  if (result.ok && state.has_pending_cycle) {
    result.ok = false;
    result.playback.ok = false;
    result.first_error = "RIR replay ended with pending cycle_input without cycle_output";
    result.playback.first_error = result.first_error;
  }
  result.reached_failure_marker = state.reached_failure_marker;
  result.failure_marker_payload = state.failure_marker_payload;
  result.failure_marker_data = state.failure_marker_data;
  if (!result.ok && result.first_error.empty()) {
    result.first_error = result.playback.first_error;
  }
  return result;
}

}  // namespace session
}  // namespace remote_identification_radar
