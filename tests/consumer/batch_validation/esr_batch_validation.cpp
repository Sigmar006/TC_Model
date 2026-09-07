/**
 * @file esr_batch_validation.cpp
 * @brief 电子侦察（ESR）批量场景验证。
 *
 * @par 目标
 * 通过公开 Session 接口（EsrRecordingSession + RF v2 发射帧）对 ESR 模块做多场景
 * 参数扫描，验证其在不同辐射源距离 / 载频 / 频谱占用率下的泛用性：
 *   - 采集周期级 CSV（观测、侦察假设与接收机状态）。
 *   - 软断言：假设置信度 ∈ [0,1]。
 *   - 每场景录制可回放 trace，用 ReplayEsrTrace 做确定性回归。
 *
 * @par 运行方式
 *   ./esr_batch_validation [--suite sweep|sequence|all] [--scenario ID]
 *                          [--output-dir PATH] [--list-scenarios]
 *   默认输出 /tmp/1q/batch_validation/electronic_surveillance_radar/
 */

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrReplaySession.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/electronic_surveillance_radar/session/EsrRecordingSession.h"

#include "batch_assertions.h"
#include "batch_checks.h"
#include "batch_cli.h"
#include "csv_writer.h"
#include "batch_replay.h"
#include "config_loader.h"

namespace esr = electronic_surveillance_radar;
namespace esr_config = electronic_surveillance_radar::config;
namespace esr_session = electronic_surveillance_radar::session;
using examples::CsvWriter;
using batch_validation::ContractCheckCollector;
using batch_validation::ModuleName;
using batch_validation::ReplayCheckResult;
using batch_validation::Severity;
using batch_validation::WarningCollector;

namespace {

#ifndef BATCH_CONFIG_DIR
#define BATCH_CONFIG_DIR "."
#endif

constexpr const char* kDefaultOutputDir = "/tmp/1q/batch_validation/electronic_surveillance_radar";
constexpr std::uint32_t kNumCycles = 40;
constexpr std::uint32_t kWarmupCycles = 5;
constexpr const char* kTraceId = "esr-batch-validation";

// =============================================================================
// 场景参数表
// =============================================================================

struct EsrCase {
  std::string scenario_id;
  double emitter_range_km;    ///< 辐射源相对平台的初始斜距（km）
  double carrier_ghz;          ///< 辐射源载频（GHz）
  float spectrum_occupancy;    ///< 频谱占用率 [0,1]（影响干扰）
  bool sequence{false};
  std::string family{"parameter_sweep"};
};

std::vector<EsrCase> BuildEsrCases() {
  std::vector<EsrCase> cases;
  const double ranges[] = {10.0, 30.0, 60.0, 100.0};
  const double carriers[] = {2.0, 8.0, 18.0};  // S/C/Ku 波段
  const float occupancies[] = {0.1f, 0.4f, 0.7f, 0.95f};
  char buf[128];
  // 完整扫描：距离 × 载频 × 频谱占用率，覆盖传播与干扰的交互。
  for (double r : ranges) {
    for (double fc : carriers) {
      for (float occ : occupancies) {
        EsrCase c;
        std::snprintf(buf, sizeof(buf), "esr_r%03.0fkm_fc%02.0f_occ%.2f", r, fc,
                      static_cast<double>(occ));
        c.scenario_id = buf;
        c.emitter_range_km = r;
        c.carrier_ghz = fc;
        c.spectrum_occupancy = occ;
        cases.push_back(c);
      }
    }
  }
  return cases;
}

std::vector<EsrCase> BuildEsrSequenceCases() {
  const char* ids[] = {"esr_seq_two_emitter_angular_crossing",
                       "esr_seq_dense_emitters_with_silence", "esr_seq_mode_switch",
                       "esr_seq_scan_bounds_retask", "esr_seq_power_cycle",
                       "esr_seq_invalid_input_recovery"};
  std::vector<EsrCase> cases;
  for (const char* id : ids) {
    EsrCase c;
    c.scenario_id = id;
    c.emitter_range_km = 30.0;
    c.carrier_ghz = 8.0;
    c.spectrum_occupancy = 0.1f;
    c.sequence = true;
    c.family = std::strstr(id, "crossing") != nullptr ? "multi_emitter_association" :
               std::strstr(id, "silence") != nullptr ? "emitter_lifecycle" :
               std::strstr(id, "power") != nullptr ? "lifecycle_interruption" :
               std::strstr(id, "invalid_input") != nullptr ? "invalid_input_recovery" :
                                                                  "runtime_reconfiguration";
    cases.push_back(c);
  }
  return cases;
}

std::uint32_t CycleCount(const EsrCase& c) { return c.sequence ? 24U : kNumCycles; }

const char* PhaseFor(const EsrCase& c, std::uint32_t cycle) {
  if (!c.sequence) return "sweep";
  if (cycle <= 8U) return "establish";
  if (cycle <= 16U) return "transition";
  return "recovery";
}

std::string JoinIds(const std::vector<std::uint64_t>& ids) {
  std::string text;
  for (std::uint64_t id : ids) {
    if (!text.empty()) text += ":";
    text += std::to_string(id);
  }
  return text;
}

// =============================================================================
// 输入构造
// =============================================================================

esr_session::EsrCycleInput MakeInput(const EsrCase& c, std::uint32_t cycle_index) {
  esr_session::EsrCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = static_cast<double>(cycle_index - 1U);
  input.dt_sec = 1.0f;
  input.platform_position_ecef_m.x_m = -2289512.0;
  input.platform_position_ecef_m.y_m = 4909946.0;
  input.platform_position_ecef_m.z_m = 3650982.0;
  input.rf_emissions.world_cycle_index = cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  oneq::coordinate::LlaPositionDegM platform_lla;
  if (!oneq::coordinate::TryEcefToLla(input.platform_position_ecef_m, &platform_lla)) {
    return input;
  }
  const std::uint32_t count = c.sequence && c.scenario_id.find("dense_emitters") != std::string::npos
                                  ? 3U
                                  : c.sequence && c.scenario_id.find("crossing") != std::string::npos ? 2U : 1U;
  for (std::uint32_t index = 0U; index < count; ++index) {
    if (c.scenario_id.find("dense_emitters") != std::string::npos && index == 1U &&
        cycle_index >= 9U && cycle_index <= 12U) continue;
    oneq::electromagnetics::RfSceneEmission emission;
    emission.identity.sensor_entity_id = 2001U + index;
    emission.identity.emission_id = index + 1U;
    const double azimuth_deg =
        (static_cast<double>(index) - 0.5 * static_cast<double>(count - 1U)) * 8.0;
    const double azimuth_rad = azimuth_deg * 3.14159265358979323846 / 180.0;
    const oneq::coordinate::Vector3d enu_direction =
        oneq::coordinate::RotateLocalToEnu(std::cos(azimuth_rad), std::sin(azimuth_rad), 0.0,
                                            input.platform_attitude_deg);
    oneq::coordinate::Vector3d ecef_direction;
    if (!oneq::coordinate::TryEnuToEcefDirection(enu_direction, platform_lla, &ecef_direction)) {
      continue;
    }
    const double range_m = c.emitter_range_km * 1000.0;
    emission.position_ecef_m.x_m = input.platform_position_ecef_m.x_m + range_m * ecef_direction.x;
    emission.position_ecef_m.y_m = input.platform_position_ecef_m.y_m + range_m * ecef_direction.y;
    emission.position_ecef_m.z_m = input.platform_position_ecef_m.z_m + range_m * ecef_direction.z;
    emission.antenna.peak_gain_dbi = 30.0;
    // Point the source main lobe at the receiver in ECEF.  A local “south” is
    // not the ECEF Y axis at this latitude; using it was still physically
    // valid but placed the receiver outside the intended receive lobe.
    const double toward_receiver_x =
        input.platform_position_ecef_m.x_m - emission.position_ecef_m.x_m;
    const double toward_receiver_y =
        input.platform_position_ecef_m.y_m - emission.position_ecef_m.y_m;
    const double toward_receiver_z =
        input.platform_position_ecef_m.z_m - emission.position_ecef_m.z_m;
    const double toward_receiver_norm = std::sqrt(
        toward_receiver_x * toward_receiver_x + toward_receiver_y * toward_receiver_y +
        toward_receiver_z * toward_receiver_z);
    emission.antenna.boresight_ecef.x = toward_receiver_x / toward_receiver_norm;
    emission.antenna.boresight_ecef.y = toward_receiver_y / toward_receiver_norm;
    emission.antenna.boresight_ecef.z = toward_receiver_z / toward_receiver_norm;
    emission.polarization = oneq::electromagnetics::RfScenePolarization::kHorizontal;
    if (oneq::electromagnetics::TryCreateRfNoiseWaveform(
            input.cycle_start_time_s, input.dt_sec, c.carrier_ghz * 1.0e9 + index * 5.0e6,
            2.0e6, 5.0e6, &emission.waveform)) input.rf_emissions.emissions.push_back(emission);
  }
  return input;
}

// =============================================================================
// CSV schema
// =============================================================================

constexpr const char* kCycleHeader =
    "scenario_id,suite,scenario_family,phase,cycle_index,executed_this_cycle,has_validation_error,abort_reason,"
    "raw_observation_count,cluster_count,receiver_saturated,obs_snr_db_mean,"
    "hypothesis_count,hypothesis_confidence_mean,hypothesis_confidence_p95";

constexpr const char* kScenarioHeader =
    "scenario_id,suite,scenario_family,emitter_range_km,carrier_ghz,spectrum_occupancy,executed_cycles,"
    "steady_obs_count_mean,steady_hyp_confidence_mean,steady_receiver_saturated_mean,"
    "replay_ok,replay_compared,replay_divergence,warning_count,"
    "error_count,expected_failure_count,contract_check_count,contract_failure_count,"
    "failure_marker_count,warnings";

// =============================================================================
// 单场景执行
// =============================================================================

struct CycleMetrics {
  std::uint32_t cycle_index{0};
  bool executed{false};
  bool has_validation_error{false};
  int abort_reason{0};
  std::size_t raw_obs{0};
  std::size_t cluster{0};
  std::size_t receiver_saturated{0};
  double obs_snr_mean{0.0};
  std::size_t hyp_count{0};
  double hyp_conf_mean{0.0};
  double hyp_conf_p95{0.0};
};

CycleMetrics ExtractCycleMetrics(const esr_session::EsrCycleResult& r) {
  CycleMetrics m;
  m.cycle_index = r.input_cycle_index;
  m.executed = r.status == esr_session::EsrCycleExecutionStatus::kCompleted;
  m.has_validation_error = esr_session::HasValidationError(r.issues);
  m.abort_reason = static_cast<int>(r.abort_reason);

  const auto& of = r.output_frame.observation_output;
  m.raw_obs = of.raw_observation_count;
  m.cluster = of.cluster_count;
  std::vector<double> snrs;
  for (const auto& o : of.observations) {
    snrs.push_back(o.snr_db);
  }
  m.obs_snr_mean = batch_validation::Mean(snrs);

  const auto& ef = r.output_frame.emitter_output;
  m.hyp_count = ef.hypotheses.size();
  std::vector<double> confs;
  for (const auto& h : ef.hypotheses) confs.push_back(static_cast<double>(h.confidence));
  m.hyp_conf_mean = batch_validation::Mean(confs);
  m.hyp_conf_p95 = batch_validation::Percentile(confs, 95.0);

  m.receiver_saturated = of.receiver_saturated ? 1U : 0U;
  return m;
}

struct ScenarioSummary {
  std::string scenario_id;
  std::string suite;
  std::string scenario_family;
  double emitter_range_km{0.0};
  double carrier_ghz{0.0};
  double spectrum_occupancy{0.0};
  std::uint32_t executed_cycles{0};
  double steady_obs_count_mean{0.0};
  double steady_hyp_confidence_mean{0.0};
  double steady_receiver_saturated_mean{0.0};
  bool replay_ok{false};
  std::uint64_t replay_compared{0};
  bool replay_divergence{false};
  std::size_t expected_failure_count{0U};
  std::size_t contract_check_count{0U};
  std::size_t contract_failure_count{0U};
  std::uint64_t failure_marker_count{0U};
  WarningCollector warnings;
};

ScenarioSummary RunEsrScenario(const EsrCase& c, const esr_config::EsrSessionConfig& base_config,
                               const std::string& output_dir, CsvWriter& cycle_writer,
                               ContractCheckCollector& checks) {
  ScenarioSummary s;
  s.scenario_id = c.scenario_id;
  s.suite = c.sequence ? "sequence" : "sweep";
  s.scenario_family = c.family;
  s.emitter_range_km = c.emitter_range_km;
  s.carrier_ghz = c.carrier_ghz;
  s.spectrum_occupancy = static_cast<double>(c.spectrum_occupancy);

  esr_config::EsrSessionConfig config = base_config;
  // Batch scenarios exercise scene geometry and lifecycle semantics.  Keep a
  // deterministic, explicitly documented intercept threshold so profile
  // defaults cannot turn every scenario into a stochastic no-observation run.
  config.policy.detection.enable_statistical_detection = false;
  config.policy.detection.minimum_snr_db = -10.0f;
  config.hardware.tuning_plan.clear();
  esr_config::EsrTuningWindow tuning_window;
  tuning_window.center_frequency_hz = c.carrier_ghz * 1.0e9 + 5.0e6;
  tuning_window.bandwidth_hz = 50.0e6;
  tuning_window.dwell_cycles = 1U;
  config.hardware.tuning_plan.push_back(tuning_window);

  const std::string trace_dir = output_dir + "/traces/" + c.scenario_id;
  auto replay_writer = batch_validation::MakeReplayWriter(
      trace_dir, ModuleName::kElectronicSurveillanceRadar, kTraceId, c.scenario_id);

  std::vector<CycleMetrics> metrics;
  const std::uint32_t cycle_count = CycleCount(c);
  metrics.reserve(cycle_count);
  std::size_t nonexecuted_count = 0U;
  bool invalid_bounds_rejected = false;
  std::vector<std::uint64_t> established_ids;
  std::vector<std::uint64_t> recovered_ids;

  {
    esr_session::EsrRecordingSessionOptions options;
    options.replay_writer = replay_writer;
    options.record_config_on_construct = true;
    esr_session::EsrRecordingSession session(config, options);

    const char* previous_phase = nullptr;
    for (std::uint32_t i = 0; i < cycle_count; ++i) {
      const std::uint32_t cycle_index = i + 1;
      const char* phase = PhaseFor(c, cycle_index);
      if (previous_phase == nullptr || std::strcmp(previous_phase, phase) != 0) {
        std::fprintf(stderr, "  [phase] scenario=%s phase=%s cycle=%u\n",
                     c.scenario_id.c_str(), phase, cycle_index);
        previous_phase = phase;
      }
      if (c.scenario_id == "esr_seq_mode_switch" && (cycle_index == 9U || cycle_index == 17U)) {
        esr_config::EsrRuntimeConfigPatch patch;
        patch.has_work_mode = true;
        patch.work_mode = cycle_index == 9U ? esr_config::EsrWorkMode::kRwr
                                            : esr_config::EsrWorkMode::kHgesm;
        (void)session.TryApplyRuntimeConfig(patch);
      }
      if (c.scenario_id == "esr_seq_scan_bounds_retask" && cycle_index == 9U) {
        esr_config::EsrRuntimeConfigPatch patch;
        patch.has_scan_center_az_deg = true;
        patch.scan_center_az_deg = 20.0f;
        patch.has_explicit_scan_bounds = true;
        patch.explicit_scan_bounds.enabled = true;
        patch.explicit_scan_bounds.scan_start_az_deg = 30.0f;
        patch.explicit_scan_bounds.scan_end_az_deg = -30.0f;
        patch.explicit_scan_bounds.scan_start_el_deg = -5.0f;
        patch.explicit_scan_bounds.scan_end_el_deg = 5.0f;
        invalid_bounds_rejected = !session.session().TryApplyRuntimeConfig(patch);
      } else if (c.scenario_id == "esr_seq_scan_bounds_retask" && cycle_index == 10U) {
        esr_config::EsrRuntimeConfigPatch patch;
        patch.has_explicit_scan_bounds = true;
        patch.explicit_scan_bounds.enabled = true;
        patch.explicit_scan_bounds.scan_start_az_deg = -20.0f;
        patch.explicit_scan_bounds.scan_end_az_deg = 20.0f;
        patch.explicit_scan_bounds.scan_start_el_deg = -5.0f;
        patch.explicit_scan_bounds.scan_end_el_deg = 5.0f;
        (void)session.TryApplyRuntimeConfig(patch);
      } else if (c.scenario_id == "esr_seq_scan_bounds_retask" && cycle_index == 17U) {
        esr_config::EsrRuntimeConfigPatch patch;
        patch.has_scan_center_az_deg = true;
        patch.scan_center_az_deg = 0.0f;
        patch.has_explicit_scan_bounds = true;
        patch.explicit_scan_bounds.enabled = false;
        (void)session.TryApplyRuntimeConfig(patch);
      }
      if (c.scenario_id == "esr_seq_power_cycle" && (cycle_index == 9U || cycle_index == 14U)) {
        esr_config::EsrRuntimeConfigPatch patch;
        patch.has_sensor_enabled = true;
        patch.sensor_enabled = cycle_index == 14U;
        (void)session.TryApplyRuntimeConfig(patch);
      }
      esr_session::EsrCycleInput input = MakeInput(c, cycle_index);
      if (c.scenario_id == "esr_seq_invalid_input_recovery" &&
          (cycle_index == 9U || cycle_index == 10U)) {
        input.dt_sec = 0.0f;
      }

      const esr_session::EsrCycleResult result = session.StepWithResult(input);
      if (result.status != esr_session::EsrCycleExecutionStatus::kCompleted) ++nonexecuted_count;
      const bool capture_established_ids = cycle_index == 8U;
      const bool capture_recovered_ids =
          (c.scenario_id == "esr_seq_power_cycle" && cycle_index == 14U) ||
          (c.scenario_id == "esr_seq_invalid_input_recovery" &&
           cycle_index == 11U) ||
          (c.scenario_id == "esr_seq_two_emitter_angular_crossing" &&
           cycle_index == 24U);
      if (capture_established_ids || capture_recovered_ids) {
        std::vector<std::uint64_t>& ids =
            capture_established_ids ? established_ids : recovered_ids;
        for (const auto& hypothesis : result.output_frame.emitter_output.hypotheses) {
          ids.push_back(hypothesis.hypothesis_id);
        }
        std::sort(ids.begin(), ids.end());
      }
      CycleMetrics m = ExtractCycleMetrics(result);
      metrics.push_back(m);

      std::fprintf(cycle_writer.file(),
                   "%s,%s,%s,%s,%u,%d,%d,%d,%zu,%zu,%zu,%.5f,%zu,%.5f,%.5f\n",
                   c.scenario_id.c_str(), c.sequence ? "sequence" : "sweep", c.family.c_str(),
                   phase, m.cycle_index, static_cast<int>(m.executed),
                   static_cast<int>(m.has_validation_error), m.abort_reason, m.raw_obs, m.cluster,
                   m.receiver_saturated, m.obs_snr_mean, m.hyp_count, m.hyp_conf_mean, m.hyp_conf_p95);
    }
    replay_writer->Flush();
  }

  // 聚合
  std::vector<double> steady_obs, steady_conf, steady_receiver_saturated;
  for (const auto& m : metrics) {
    if (m.executed) ++s.executed_cycles;
    if (!m.executed || m.cycle_index <= kWarmupCycles) continue;
    steady_obs.push_back(static_cast<double>(m.raw_obs));
    steady_conf.push_back(m.hyp_conf_mean);
    steady_receiver_saturated.push_back(static_cast<double>(m.receiver_saturated));
  }
  s.steady_obs_count_mean = batch_validation::Mean(steady_obs);
  s.steady_hyp_confidence_mean = batch_validation::Mean(steady_conf);
  s.steady_receiver_saturated_mean = batch_validation::Mean(steady_receiver_saturated);

  // 回放
  const esr_session::EsrReplaySessionResult replay = esr_session::ReplayEsrTrace(trace_dir);
  s.replay_ok = replay.ok;
  s.replay_compared = replay.playback.compared_output_count;
  s.replay_divergence = replay.playback.divergence_found;
  s.failure_marker_count = replay.playback.failure_marker_count;
  if (!replay.ok || replay.playback.divergence_found) {
    s.warnings.Error("replay divergence: " + replay.first_error);
  }
  if (replay.playback.compared_output_count != metrics.size()) {
    s.warnings.Error("replay output count does not equal Step count");
  }

  if (c.sequence) {
    const std::size_t expected_nonexecuted =
        c.scenario_id == "esr_seq_power_cycle" ? 5U :
        c.scenario_id == "esr_seq_invalid_input_recovery" ? 2U : 0U;
    s.expected_failure_count = c.scenario_id == "esr_seq_invalid_input_recovery" ? 2U :
                               c.scenario_id == "esr_seq_scan_bounds_retask" ? 1U : 0U;
    checks.Add(c.scenario_id, "replay", cycle_count, "replay_complete",
               std::to_string(metrics.size()), std::to_string(s.replay_compared),
               replay.ok && !s.replay_divergence && s.replay_compared == metrics.size());
    checks.Add(c.scenario_id, "recovery", cycle_count, "expected_nonexecuted_cycles",
               std::to_string(expected_nonexecuted), std::to_string(nonexecuted_count),
               expected_nonexecuted == nonexecuted_count);
    const std::uint64_t expected_markers =
        c.scenario_id == "esr_seq_invalid_input_recovery" ? 2U : 0U;
    checks.Add(c.scenario_id, "replay", cycle_count, "failure_marker_count",
               std::to_string(expected_markers), std::to_string(s.failure_marker_count),
               expected_markers == s.failure_marker_count);
    if (c.scenario_id == "esr_seq_scan_bounds_retask") {
      checks.Add(c.scenario_id, "transition", 9U, "invalid_bounds_atomic_rejection", "rejected",
                 invalid_bounds_rejected ? "rejected" : "accepted", invalid_bounds_rejected);
    }
    if (c.scenario_id == "esr_seq_two_emitter_angular_crossing" ||
        c.scenario_id == "esr_seq_power_cycle" ||
        c.scenario_id == "esr_seq_invalid_input_recovery") {
      checks.Add(c.scenario_id, "recovery", cycle_count, "hypothesis_identity_continuity",
                 JoinIds(established_ids), JoinIds(recovered_ids),
                 !established_ids.empty() && established_ids == recovered_ids);
    }
  }

  // 软断言
  if (s.executed_cycles == 0) {
    s.warnings.Error("no cycle executed");
  } else {
    // ① 假设置信度应 ∈ [0,1]
    if (s.steady_hyp_confidence_mean < 0.0 || s.steady_hyp_confidence_mean > 1.0) {
      s.warnings.Warn("hypothesis confidence out of [0,1]: " +
                      std::to_string(s.steady_hyp_confidence_mean));
    }
  }

  batch_validation::LogReplayResult(
      c.scenario_id,
      ReplayCheckResult{replay.ok, replay.playback.divergence_found,
                        replay.playback.compared_output_count, replay.playback.applied_input_count,
                        replay.reached_failure_marker, replay.first_error},
      metrics.size());
  s.warnings.DumpToStderr(c.scenario_id + ": ");
  return s;
}

/// 跨场景趋势检查占位。
///
/// 历史上这里曾对 `steady_truth_match_rate_mean`（恒 0 的死指标）和别名自
/// `receiver_saturated` 的 `steady_jammed_mean` 做距离/占用率单调断言。当前 48 个 sweep 场景
/// 都能在稳态产生真实 observation 和 hypothesis，但现有发射功率下 observation 数与置信度不随
/// range/occupancy 分档，接收机也始终不饱和；直接添加单调断言仍会成为恒通过的"空检查"。
///
/// 待 sweep 引入可形成分档差异的功率、干扰或多源几何后，再在此处补充基于真实观测/估计的趋势
/// 软断言。
void CheckCrossScenarioTrends(std::vector<ScenarioSummary>& /*summaries*/) {}

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
  const std::string config_path = BATCH_CONFIG_DIR "/electronic_warfare.json";

  std::fprintf(stderr, "=== ESR 批量场景验证 ===\n");
  std::fprintf(stderr, "  配置: %s\n  输出: %s\n", config_path.c_str(), output_dir.c_str());

  std::vector<EsrCase> cases;
  if (batch_validation::IncludesSweep(cli.suite)) cases = BuildEsrCases();
  if (batch_validation::IncludesSequence(cli.suite)) {
    const std::vector<EsrCase> sequences = BuildEsrSequenceCases();
    cases.insert(cases.end(), sequences.begin(), sequences.end());
  }
  if (cli.list_scenarios) {
    for (const EsrCase& c : cases) std::printf("%s\n", c.scenario_id.c_str());
    return 0;
  }
  if (!cli.scenario_id.empty()) {
    cases.erase(std::remove_if(cases.begin(), cases.end(), [&](const EsrCase& c) {
                  return c.scenario_id != cli.scenario_id;
                }), cases.end());
  }
  if (cases.empty()) {
    std::fprintf(stderr, "FATAL: no scenario matched\n");
    return 1;
  }

  esr_config::EsrSessionConfig base_config;
  std::string load_error;
  if (!examples::LoadEsrSessionConfigFromFile(config_path.c_str(), &base_config, &load_error)) {
    std::fprintf(stderr, "FATAL: 加载配置失败 %s: %s\n", config_path.c_str(), load_error.c_str());
    return 1;
  }

  std::error_code ec;
  std::filesystem::create_directories(output_dir, ec);
  if (ec) {
    std::fprintf(stderr, "FATAL: 无法创建输出目录 %s: %s\n", output_dir.c_str(), ec.message().c_str());
    return 1;
  }
  const std::string cycles_csv = output_dir + "/cycles.csv";
  const std::string scenarios_csv = output_dir + "/scenarios.csv";
  CsvWriter cycle_writer(cycles_csv, kCycleHeader);
  CsvWriter scenario_writer(scenarios_csv, kScenarioHeader);

  std::vector<ScenarioSummary> summaries;
  ContractCheckCollector checks;
  summaries.reserve(cases.size());
  std::fprintf(stderr, "  场景数: %zu\n", cases.size());

  for (std::size_t i = 0; i < cases.size(); ++i) {
    std::fprintf(stderr, "[%zu/%zu] %s\n", i + 1, cases.size(), cases[i].scenario_id.c_str());
    std::fprintf(stderr, "  [scenario] id=%s suite=%s family=%s\n", cases[i].scenario_id.c_str(),
                 cases[i].sequence ? "sequence" : "sweep", cases[i].family.c_str());
    const std::size_t check_begin = checks.size();
    summaries.push_back(RunEsrScenario(cases[i], base_config, output_dir, cycle_writer, checks));
    summaries.back().contract_check_count = checks.size() - check_begin;
    for (std::size_t j = check_begin; j < checks.checks().size(); ++j) {
      if (!checks.checks()[j].passed && checks.checks()[j].severity == Severity::kError) {
        ++summaries.back().contract_failure_count;
      }
    }
    cycle_writer.Flush();
  }

  CheckCrossScenarioTrends(summaries);

  for (const auto& s : summaries) {
    std::fprintf(stderr,
                 "  [scenario] module=ESR id=%s range_km=%.3f carrier_ghz=%.3f occupancy=%.3f "
                 "executed=%u/%u observations=%.4f hypothesis_confidence=%.4f "
                 "receiver_saturated=%.4f replay_ok=%d compared=%llu divergence=%d "
                 "warn=%zu error=%zu\n",
                 s.scenario_id.c_str(), s.emitter_range_km, s.carrier_ghz,
                 s.spectrum_occupancy, s.executed_cycles,
                 s.suite == "sequence" ? 24U : kNumCycles, s.steady_obs_count_mean,
                 s.steady_hyp_confidence_mean, s.steady_receiver_saturated_mean,
                 static_cast<int>(s.replay_ok),
                 static_cast<unsigned long long>(s.replay_compared),
                 static_cast<int>(s.replay_divergence),
                 s.warnings.Count(Severity::kWarning), s.warnings.Count(Severity::kError));
    std::fprintf(scenario_writer.file(),
                 "%s,%s,%s,%.3f,%.3f,%.3f,%u,%.4f,%.4f,%.4f,%d,%llu,%d,%zu,%zu,%zu,%zu,%zu,%llu,%s\n",
                 s.scenario_id.c_str(), s.suite.c_str(), s.scenario_family.c_str(),
                 s.emitter_range_km, s.carrier_ghz, s.spectrum_occupancy,
                 s.executed_cycles, s.steady_obs_count_mean, s.steady_hyp_confidence_mean,
                 s.steady_receiver_saturated_mean,
                 static_cast<int>(s.replay_ok), static_cast<unsigned long long>(s.replay_compared),
                 static_cast<int>(s.replay_divergence), s.warnings.Count(Severity::kWarning),
                 s.warnings.Count(Severity::kError), s.expected_failure_count,
                 s.contract_check_count, s.contract_failure_count,
                 static_cast<unsigned long long>(s.failure_marker_count),
                 batch_validation::EscapeCsvField(s.warnings.JoinForCsv()).c_str());
  }
  scenario_writer.Flush();
  checks.WriteCsv(output_dir + "/checks.csv");

  std::size_t total_warn = 0, total_err = 0, replay_div = 0;
  for (const auto& s : summaries) {
    total_warn += s.warnings.Count(Severity::kWarning);
    total_err += s.warnings.Count(Severity::kError);
    if (!s.replay_ok || s.replay_divergence) ++replay_div;
  }
  std::fprintf(stderr, "\n=== ESR 批量验证完成 ===\n");
  std::fprintf(stderr, "  场景数: %zu\n  周期 CSV: %s\n  场景 CSV: %s\n", summaries.size(),
               cycles_csv.c_str(), scenarios_csv.c_str());
  std::fprintf(stderr, "  软断言 warning: %zu, error: %zu\n", total_warn, total_err);
  std::fprintf(stderr, "  回放分叉场景: %zu\n", replay_div);
  std::fprintf(stderr, "  trace 目录: %s/traces/<scenario_id>/\n", output_dir.c_str());
  return (total_err > 0 || replay_div > 0 || checks.FailureCount() > 0U) ? 2 : 0;
}
