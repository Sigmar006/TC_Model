/**
 * @file sbirs_batch_validation.cpp
 * @brief SBIRS 距离与辐射强度的批量物理场景验证。
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <oneq/FsCompat.h>
#include <set>
#include <string>
#include <system_error>
#include <vector>

#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsInputValidation.h"
#include "1q/sbirs_sensor/session/SbirsReplaySession.h"
#include "1q/sbirs_sensor/session/SbirsRecordingSession.h"
#include "batch_assertions.h"
#include "batch_checks.h"
#include "batch_cli.h"
#include "csv_writer.h"
#include "batch_replay.h"

namespace sbirs_config = sbirs_sensor::config;
namespace sbirs_output = sbirs_sensor::output;
namespace sbirs_session = sbirs_sensor::session;
using examples::CsvWriter;
using batch_validation::ContractCheckCollector;
using batch_validation::ModuleName;
using batch_validation::ReplayCheckResult;
using batch_validation::Severity;
using batch_validation::WarningCollector;

namespace {

constexpr const char* kDefaultOutputDir = "/tmp/1q/batch_validation/sbirs_sensor";
constexpr const char* kTraceId = "sbirs-batch-validation";
constexpr double kSatelliteXM = 7000000.0;

struct SbirsCase {
  std::string scenario_id;
  double range_km{0.0};
  double radiant_intensity_w_per_sr{0.0};
  bool sequence{false};
  std::string family{"parameter_sweep"};
};

std::vector<SbirsCase> BuildCases() {
  const double ranges_km[] = {1000.0, 2000.0, 4000.0};
  const double intensities[] = {1.0e5, 1.0e7, 1.0e9};
  std::vector<SbirsCase> cases;
  char id[128];
  for (double range_km : ranges_km) {
    for (double intensity : intensities) {
      std::snprintf(id, sizeof(id), "sbirs_r%.0fkm_i%.0e", range_km, intensity);
      cases.push_back({id, range_km, intensity});
    }
  }
  return cases;
}

std::vector<SbirsCase> BuildSequenceCases() {
  const char* ids[] = {"sbirs_seq_two_target_crossing_two_locks",
                       "sbirs_seq_three_target_one_lock_handoff",
                       "sbirs_seq_boost_maneuver_nis_reacquire",
                       "sbirs_seq_cue_latency_cross_velocity",
                       "sbirs_seq_occultation_reappearance",
                       "sbirs_seq_standby_mission_retask",
                       "sbirs_seq_invalid_input_recovery"};
  std::vector<SbirsCase> cases;
  for (const char* id : ids) {
    SbirsCase c;
    c.scenario_id = id;
    c.range_km = 1000.0;
    c.radiant_intensity_w_per_sr = 2.5e8;
    c.sequence = true;
    c.family = std::strstr(id, "target") != nullptr ? "multi_target_resource" :
               std::strstr(id, "maneuver") != nullptr ? "tracking_reacquisition" :
               std::strstr(id, "occultation") != nullptr ? "visibility_interruption" :
               std::strstr(id, "invalid_input") != nullptr ? "invalid_input_recovery" :
                                                                  "mission_reconfiguration";
    cases.push_back(c);
  }
  return cases;
}

std::uint32_t CycleCount(const SbirsCase& c) { return c.sequence ? 12U : 2U; }

const char* PhaseFor(const SbirsCase& c, std::uint32_t cycle) {
  if (!c.sequence) return "sweep";
  if (cycle <= 4U) return "establish";
  if (cycle <= 8U) return "interruption";
  return "recovery";
}

sbirs_session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

sbirs_config::SbirsSessionConfig MakeConfig(const SbirsCase* scenario = nullptr) {
  sbirs_config::SbirsSessionConfig config;
  config.hardware.integration_time_sec = 1.0f;
  // ECI 方位约定（2026-08）：scan_start_az ∈ [0, 360)；测试几何在 GMST≈0 时刻
  // （utc_julian_day = 2451544.2230698913）下 ECI≡ECEF，az=0 目标仍被扫描覆盖。
  config.mission.scan_start_az_deg = 0.0f;
  config.mission.scan_span_deg = 11.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.mission.wide_field_fov_az_deg = 20.0f;
  config.mission.wide_field_fov_el_deg = 20.0f;
  config.mission.narrow_field_fov_az_deg = 5.0f;
  config.mission.narrow_field_fov_el_deg = 5.0f;
  config.policy.error_model.range_fraction_sigma = 0.0f;
  config.policy.error_model.attitude_sigma_deg = 0.0f;
  if (scenario != nullptr) {
    if (scenario->scenario_id == "sbirs_seq_boost_maneuver_nis_reacquire") {
      config.policy.tracking.nis_gate_loss_cycles = 2U;
      config.policy.tracking.process_noise_diff_coeff = 0.01f;
    }
    if (scenario->scenario_id == "sbirs_seq_cue_latency_cross_velocity") {
      config.mission.narrow_cue_latency_s = 0.5f;
    }
  }
  return config;
}

sbirs_session::SbirsCycleInput MakeInput(const SbirsCase& scenario, std::uint32_t cycle_index) {
  sbirs_session::SbirsSceneTarget target;
  target.target_id = 101U;
  target.target_name = "batch_ir_target";
  target.position_ecef_m = Vector(kSatelliteXM + scenario.range_km * 1000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = scenario.radiant_intensity_w_per_sr;
  if (scenario.sequence && scenario.scenario_id == "sbirs_seq_boost_maneuver_nis_reacquire" &&
      cycle_index >= 5U && cycle_index <= 8U) {
    target.position_ecef_m.y = static_cast<double>(cycle_index - 4U) * 10000.0;
  }
  if (scenario.sequence && scenario.scenario_id == "sbirs_seq_cue_latency_cross_velocity") {
    target.position_ecef_m.y = -10000.0 + static_cast<double>(cycle_index) * 1000.0;
    target.velocity_ecef_m_per_s = Vector(0.0, 1000.0, 0.0);
    target.has_velocity_ecef_m_per_s = true;
  }
  if (scenario.sequence && scenario.scenario_id == "sbirs_seq_occultation_reappearance" &&
      cycle_index >= 5U && cycle_index <= 8U) {
    target.position_ecef_m = Vector(-kSatelliteXM, 0.0, 0.0);
  }
  if (scenario.sequence && scenario.scenario_id == "sbirs_seq_three_target_one_lock_handoff" &&
      cycle_index >= 5U && cycle_index <= 8U) {
    target.active = false;
  }
  sbirs_session::SbirsCycleInputBuilder builder = sbirs_session::SbirsCycleInputBuilder()
      .WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF，测试期望几何不变
      .WithSatellitePosition(Vector(kSatelliteXM, 0.0, 0.0))
      .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
      .AddTarget(target);
  if (scenario.sequence && (scenario.scenario_id == "sbirs_seq_two_target_crossing_two_locks" ||
                            scenario.scenario_id == "sbirs_seq_three_target_one_lock_handoff" ||
                            scenario.scenario_id == "sbirs_seq_invalid_input_recovery")) {
    const int count = scenario.scenario_id == "sbirs_seq_three_target_one_lock_handoff" ? 3 : 2;
    for (int i = 1; i < count; ++i) {
      sbirs_session::SbirsSceneTarget other = target;
      other.target_id = 101U + static_cast<std::uint64_t>(i);
      other.target_name = "batch_ir_target_" + std::to_string(i + 1);
      other.active = true;
      const double crossing = (6.5 - static_cast<double>(cycle_index)) * 1000.0;
      other.position_ecef_m.y = i == 1 ? -crossing : 25000.0;
      if (scenario.scenario_id == "sbirs_seq_two_target_crossing_two_locks") {
        target.position_ecef_m.y = crossing;
      }
      builder.AddTarget(other);
    }
  }
  sbirs_session::SbirsCycleInput input = builder.Build();
  if (scenario.sequence && scenario.scenario_id == "sbirs_seq_two_target_crossing_two_locks") {
    input.scene[0].position_ecef_m.y = (6.5 - static_cast<double>(cycle_index)) * 1000.0;
  }
  if (scenario.sequence && scenario.scenario_id == "sbirs_seq_invalid_input_recovery") {
    if (cycle_index == 5U) input.dt_sec = 0.0f;
    if (cycle_index == 6U && input.scene.size() >= 2U) {
      input.scene[1].target_id = input.scene[0].target_id;
    }
  }
  return input;
}

const char* StageName(sbirs_output::SbirsObservationStage stage) {
  switch (stage) {
    case sbirs_output::SbirsObservationStage::kWideFieldSearch:
      return "wfov_search";
    case sbirs_output::SbirsObservationStage::kNarrowFieldAcquisition:
      return "nfov_acquisition";
    case sbirs_output::SbirsObservationStage::kNarrowFieldTrack:
      return "nfov_track";
  }
  return "none";
}

struct ScenarioSummary {
  SbirsCase scenario;
  std::string suite;
  std::string scenario_family;
  std::size_t executed_cycles{0U};
  std::size_t detection_count{0U};
  float max_snr_linear{0.0f};
  std::string final_stage{"none"};
  bool replay_ok{false};
  std::uint64_t replay_compared{0U};
  bool replay_divergence{false};
  std::size_t expected_failure_count{0U};
  std::size_t contract_check_count{0U};
  std::size_t contract_failure_count{0U};
  std::uint64_t failure_marker_count{0U};
  WarningCollector warnings;
};

constexpr const char* kCycleHeader =
    "scenario_id,suite,scenario_family,phase,cycle_index,executed,validation_error,abort_reason,scan_azimuth_rad,"
    "detection_count,max_snr_linear,observation_stage";
constexpr const char* kScenarioHeader =
    "scenario_id,suite,scenario_family,range_km,radiant_intensity_w_per_sr,executed_cycles,detection_count,"
    "max_snr_linear,final_stage,replay_ok,replay_compared,replay_divergence,warning_count,"
    "error_count,expected_failure_count,contract_check_count,contract_failure_count,"
    "failure_marker_count,warnings";

ScenarioSummary RunScenario(const SbirsCase& scenario, const std::string& output_dir,
                            CsvWriter& cycle_writer, ContractCheckCollector& checks) {
  ScenarioSummary summary;
  summary.scenario = scenario;
  summary.suite = scenario.sequence ? "sequence" : "sweep";
  summary.scenario_family = scenario.family;
  const std::string trace_dir = output_dir + "/traces/" + scenario.scenario_id;
  auto replay_writer = batch_validation::MakeReplayWriter(trace_dir, ModuleName::kSbirsSensor,
                                                          kTraceId, scenario.scenario_id);
  {
    sbirs_session::SbirsRecordingSessionOptions options;
    options.replay_writer = replay_writer;
    sbirs_session::SbirsRecordingSession session(MakeConfig(&scenario), options);
    const std::uint32_t cycle_count = CycleCount(scenario);
    std::size_t nonexecuted_count = 0U;
    std::size_t max_nfov_targets_per_cycle = 0U;
    std::set<std::uint64_t> nfov_target_ids;
    bool saw_nis_exceeded = false;
    bool saw_nis_loss = false;
    std::size_t interruption_detections = 0U;
    std::size_t recovery_detections = 0U;
    const char* previous_phase = nullptr;
    for (std::uint32_t cycle = 1U; cycle <= cycle_count; ++cycle) {
      const char* phase = PhaseFor(scenario, cycle);
      if (previous_phase == nullptr || std::strcmp(previous_phase, phase) != 0) {
        std::fprintf(stderr, "  [phase] scenario=%s phase=%s cycle=%u\n",
                     scenario.scenario_id.c_str(), phase, cycle);
        previous_phase = phase;
      }
      if (scenario.scenario_id == "sbirs_seq_standby_mission_retask" && cycle == 5U) {
        sbirs_config::SbirsRuntimeConfigPatch patch;
        patch.has_work_mode = true;
        patch.work_mode = sbirs_config::SbirsWorkMode::kStandby;
        (void)session.TryApplyRuntimeConfig(patch);
      } else if (scenario.scenario_id == "sbirs_seq_standby_mission_retask" && cycle == 9U) {
        sbirs_config::SbirsRuntimeConfigPatch patch;
        patch.has_work_mode = true;
        patch.work_mode = sbirs_config::SbirsWorkMode::kSearchAndStare;
        patch.has_scan_rate_deg_per_sec = true;
        patch.scan_rate_deg_per_sec = 2.0f;
        patch.has_policy = true;
        patch.policy = MakeConfig(&scenario).policy;
        (void)session.TryApplyRuntimeConfig(patch);
      }
      const sbirs_session::SbirsCycleResult result =
          session.StepWithResult(MakeInput(scenario, cycle));
      if (result.status == sbirs_session::SbirsCycleStatus::kCompleted) ++summary.executed_cycles;
      else ++nonexecuted_count;
      std::set<std::uint64_t> cycle_nfov_targets;
      for (std::size_t i = 0; i < result.detection_attributions.size(); ++i) {
        const auto& attribution = result.detection_attributions[i];
        saw_nis_exceeded = saw_nis_exceeded || attribution.estimation_nis_gate_exceeded;
        saw_nis_loss = saw_nis_loss ||
                       attribution.capture_failure_reason ==
                           sbirs_sensor::attribution::SbirsCaptureFailureReason::kEstimationNisGateLost;
        if (attribution.nfov_channel_id < 0) continue;
        cycle_nfov_targets.insert(attribution.target_id);
        nfov_target_ids.insert(attribution.target_id);
      }
      max_nfov_targets_per_cycle =
          std::max(max_nfov_targets_per_cycle, cycle_nfov_targets.size());
      if (cycle >= 5U && cycle <= 8U) {
        interruption_detections += result.output_frame.detections.size();
      } else if (cycle >= 9U) {
        recovery_detections += result.output_frame.detections.size();
      }
      summary.detection_count += result.output_frame.detections.size();
      float cycle_max_snr = 0.0f;
      const char* stage = "none";
      for (const auto& detection : result.output_frame.detections) {
        cycle_max_snr = std::max(cycle_max_snr, detection.infrared_snr_linear);
        stage = StageName(detection.observation_stage);
      }
      summary.max_snr_linear = std::max(summary.max_snr_linear, cycle_max_snr);
      summary.final_stage = stage;
      std::fprintf(
          cycle_writer.file(), "%s,%s,%s,%s,%u,%d,%d,%d,%.5f,%zu,%.9g,%s\n",
          scenario.scenario_id.c_str(), scenario.sequence ? "sequence" : "sweep",
          scenario.family.c_str(), phase, cycle,
          static_cast<int>(result.status == sbirs_session::SbirsCycleStatus::kCompleted),
          static_cast<int>(sbirs_session::HasValidationError(result.issues)),
          static_cast<int>(result.abort_reason),
          result.output_frame.scan_azimuth_rad, result.output_frame.detections.size(),
          static_cast<double>(cycle_max_snr), stage);
    }
    replay_writer->Flush();

    if (scenario.sequence) {
      const std::size_t expected_nonexecuted =
          scenario.scenario_id == "sbirs_seq_invalid_input_recovery"
              ? 2U
              : scenario.scenario_id == "sbirs_seq_standby_mission_retask" ? 4U : 0U;
      summary.expected_failure_count = expected_nonexecuted;
      checks.Add(scenario.scenario_id, "recovery", cycle_count, "expected_nonexecuted_cycles",
                 std::to_string(expected_nonexecuted), std::to_string(nonexecuted_count),
                 expected_nonexecuted == nonexecuted_count);
      if (scenario.scenario_id == "sbirs_seq_two_target_crossing_two_locks") {
        // 单镜筒（2026-09-02）：同帧双目标同周期都可被 NFOV 服务（窗口+同帧搭车），
        // 单周期最多服务的不同目标数用于确认双目标均已进入窄场。
        checks.Add(scenario.scenario_id, "interruption", 8U, "two_nfov_targets_served", ">=2",
                   std::to_string(max_nfov_targets_per_cycle), max_nfov_targets_per_cycle >= 2U);
        checks.Add(scenario.scenario_id, "recovery", cycle_count, "two_targets_locked", "2",
                   std::to_string(nfov_target_ids.size()), nfov_target_ids.size() == 2U);
      } else if (scenario.scenario_id == "sbirs_seq_three_target_one_lock_handoff") {
        checks.Add(scenario.scenario_id, "recovery", cycle_count, "handoff_reaches_multiple_targets",
                   ">=2", std::to_string(nfov_target_ids.size()), nfov_target_ids.size() >= 2U);
      } else if (scenario.scenario_id == "sbirs_seq_boost_maneuver_nis_reacquire") {
        checks.Add(scenario.scenario_id, "interruption", 8U, "nis_gate_exceeded", "true",
                   saw_nis_exceeded ? "true" : "false", saw_nis_exceeded);
        checks.Add(scenario.scenario_id, "interruption", 8U, "nis_lock_released", "true",
                   saw_nis_loss ? "true" : "false", saw_nis_loss);
        checks.Add(scenario.scenario_id, "recovery", cycle_count, "reacquired_after_maneuver",
                   ">0", std::to_string(recovery_detections), recovery_detections > 0U);
      } else if (scenario.scenario_id == "sbirs_seq_cue_latency_cross_velocity") {
        checks.Add(scenario.scenario_id, "recovery", cycle_count, "latency_geometry_captures_target",
                   ">0", std::to_string(nfov_target_ids.size()), !nfov_target_ids.empty());
      } else if (scenario.scenario_id == "sbirs_seq_occultation_reappearance" ||
                 scenario.scenario_id == "sbirs_seq_standby_mission_retask") {
        checks.Add(scenario.scenario_id, "interruption", 8U, "interruption_suppresses_detections",
                   "0", std::to_string(interruption_detections), interruption_detections == 0U);
        checks.Add(scenario.scenario_id, "recovery", cycle_count, "reappearance_detected", ">0",
                   std::to_string(recovery_detections), recovery_detections > 0U);
      } else if (scenario.scenario_id == "sbirs_seq_invalid_input_recovery") {
        checks.Add(scenario.scenario_id, "recovery", cycle_count, "recovery_detection_continues",
                   ">0", std::to_string(recovery_detections), recovery_detections > 0U);
      }
    }
  }

  const sbirs_session::SbirsReplaySessionResult replay = sbirs_session::ReplaySbirsTrace(trace_dir);
  summary.replay_ok = replay.ok;
  summary.replay_compared = replay.playback.compared_output_count;
  summary.replay_divergence = replay.playback.divergence_found;
  summary.failure_marker_count = replay.playback.failure_marker_count;
  if (!scenario.sequence && summary.executed_cycles != 2U) {
    summary.warnings.Error("not all cycles executed");
  }
  const std::uint32_t cycle_count = CycleCount(scenario);
  if (!replay.ok || replay.playback.divergence_found || summary.replay_compared != cycle_count) {
    summary.warnings.Error("replay divergence: " + replay.first_error);
  }
  if (scenario.sequence) {
    checks.Add(scenario.scenario_id, "replay", cycle_count, "replay_complete",
               std::to_string(cycle_count), std::to_string(summary.replay_compared),
               replay.ok && !summary.replay_divergence && summary.replay_compared == cycle_count);
    const std::uint64_t expected_markers =
        scenario.scenario_id == "sbirs_seq_invalid_input_recovery" ? 2U : 0U;
    checks.Add(scenario.scenario_id, "replay", cycle_count, "failure_marker_count",
               std::to_string(expected_markers), std::to_string(summary.failure_marker_count),
               expected_markers == summary.failure_marker_count);
  }
  batch_validation::LogReplayResult(
      scenario.scenario_id,
      ReplayCheckResult{replay.ok, replay.playback.divergence_found,
                        replay.playback.compared_output_count, replay.playback.applied_input_count,
                        replay.reached_failure_marker, replay.first_error},
      cycle_count);
  summary.warnings.DumpToStderr(scenario.scenario_id + ": ");
  return summary;
}

void CheckIntensityTrend(std::vector<ScenarioSummary>& summaries) {
  for (double range_km : {1000.0, 2000.0, 4000.0}) {
    std::vector<std::pair<double, float>> samples;
    for (const auto& summary : summaries) {
      if (summary.scenario.range_km == range_km) {
        samples.emplace_back(summary.scenario.radiant_intensity_w_per_sr,
                             summary.max_snr_linear);
      }
    }
    std::sort(samples.begin(), samples.end());
    for (std::size_t i = 1U; i < samples.size(); ++i) {
      if (samples[i].second < samples[i - 1U].second) {
        for (auto& summary : summaries) {
          if (summary.scenario.range_km == range_km) {
            summary.warnings.Warn("SNR decreased as radiant intensity increased");
            break;
          }
        }
        break;
      }
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  batch_validation::BatchCliOptions cli;
  std::string cli_error;
  if (!batch_validation::ParseBatchCli(argc, argv, kDefaultOutputDir, &cli, &cli_error)) {
    std::fprintf(stderr, "FATAL: %s\n", cli_error.c_str());
    batch_validation::PrintBatchUsage(argv[0]);
    return 1;
  }
  const std::string output_dir = cli.output_dir;
  std::vector<SbirsCase> cases;
  if (batch_validation::IncludesSweep(cli.suite)) cases = BuildCases();
  if (batch_validation::IncludesSequence(cli.suite)) {
    const std::vector<SbirsCase> sequences = BuildSequenceCases();
    cases.insert(cases.end(), sequences.begin(), sequences.end());
  }
  if (cli.list_scenarios) {
    for (const SbirsCase& c : cases) std::printf("%s\n", c.scenario_id.c_str());
    return 0;
  }
  if (!cli.scenario_id.empty()) {
    cases.erase(std::remove_if(cases.begin(), cases.end(), [&](const SbirsCase& c) {
                  return c.scenario_id != cli.scenario_id;
                }), cases.end());
  }
  if (cases.empty()) {
    std::fprintf(stderr, "FATAL: no scenario matched\n");
    return 1;
  }
  std::error_code ec;
  oneq_fs::create_directories(output_dir, ec);
  if (ec) {
    std::fprintf(stderr, "FATAL: cannot create %s: %s\n", output_dir.c_str(), ec.message().c_str());
    return 1;
  }
  CsvWriter cycle_writer(output_dir + "/cycles.csv", kCycleHeader);
  CsvWriter scenario_writer(output_dir + "/scenarios.csv", kScenarioHeader);
  std::vector<ScenarioSummary> summaries;
  ContractCheckCollector checks;
  summaries.reserve(cases.size());
  std::fprintf(stderr, "=== SBIRS batch validation ===\n  scenarios: %zu\n", cases.size());
  for (std::size_t i = 0U; i < cases.size(); ++i) {
    std::fprintf(stderr, "[%zu/%zu] %s\n", i + 1U, cases.size(), cases[i].scenario_id.c_str());
    std::fprintf(stderr, "  [scenario] id=%s suite=%s family=%s\n", cases[i].scenario_id.c_str(),
                 cases[i].sequence ? "sequence" : "sweep", cases[i].family.c_str());
    const std::size_t check_begin = checks.size();
    summaries.push_back(RunScenario(cases[i], output_dir, cycle_writer, checks));
    summaries.back().contract_check_count = checks.size() - check_begin;
    for (std::size_t j = check_begin; j < checks.checks().size(); ++j) {
      if (!checks.checks()[j].passed && checks.checks()[j].severity == Severity::kError) {
        ++summaries.back().contract_failure_count;
      }
    }
  }
  CheckIntensityTrend(summaries);
  for (const auto& summary : summaries) {
    std::fprintf(stderr,
                 "  [scenario] module=SBIRS id=%s range_km=%.0f intensity_w_per_sr=%.3g "
                 "executed=%zu/%u detections=%zu max_snr_linear=%.9g final_stage=%s replay_ok=%d "
                 "compared=%llu divergence=%d warn=%zu error=%zu\n",
                 summary.scenario.scenario_id.c_str(), summary.scenario.range_km,
                 summary.scenario.radiant_intensity_w_per_sr, summary.executed_cycles,
                 CycleCount(summary.scenario),
                 summary.detection_count, static_cast<double>(summary.max_snr_linear),
                 summary.final_stage.c_str(), static_cast<int>(summary.replay_ok),
                 static_cast<unsigned long long>(summary.replay_compared),
                 static_cast<int>(summary.replay_divergence),
                 summary.warnings.Count(Severity::kWarning),
                 summary.warnings.Count(Severity::kError));
    std::fprintf(
        scenario_writer.file(), "%s,%s,%s,%.0f,%.9g,%zu,%zu,%.9g,%s,%d,%llu,%d,%zu,%zu,%zu,%zu,%zu,%llu,%s\n",
        summary.scenario.scenario_id.c_str(), summary.suite.c_str(),
        summary.scenario_family.c_str(), summary.scenario.range_km,
        summary.scenario.radiant_intensity_w_per_sr, summary.executed_cycles,
        summary.detection_count, static_cast<double>(summary.max_snr_linear),
        summary.final_stage.c_str(), static_cast<int>(summary.replay_ok),
        static_cast<unsigned long long>(summary.replay_compared),
        static_cast<int>(summary.replay_divergence), summary.warnings.Count(Severity::kWarning),
        summary.warnings.Count(Severity::kError), summary.expected_failure_count,
        summary.contract_check_count, summary.contract_failure_count,
        static_cast<unsigned long long>(summary.failure_marker_count),
        batch_validation::EscapeCsvField(summary.warnings.JoinForCsv()).c_str());
  }
  checks.WriteCsv(output_dir + "/checks.csv");
  std::size_t errors = 0U;
  std::size_t warnings = 0U;
  for (const auto& summary : summaries) {
    errors += summary.warnings.Count(Severity::kError);
    warnings += summary.warnings.Count(Severity::kWarning);
  }
  std::fprintf(stderr, "=== SBIRS summary: scenarios=%zu warnings=%zu errors=%zu ===\n",
               summaries.size(), warnings, errors);
  return errors == 0U && checks.FailureCount() == 0U ? 0 : 2;
}
