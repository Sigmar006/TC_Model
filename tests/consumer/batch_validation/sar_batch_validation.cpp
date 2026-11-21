/**
 * @file sar_batch_validation.cpp
 * @brief 合成孔径雷达（SAR）批量场景验证。
 *
 * @par 目标
 * 通过公开 Session 接口（SarRecordingSession）对 SAR 模块做多场景参数扫描，验证其在不同
 * 带宽 / 斜距 / 孔径长度下的成像泛用性：
 *   - 采集周期级 CSV（聚焦阶段、图像质量指标：SNR / 主瓣宽 / 分辨率 / 图像熵 / 对比度）。
 *   - 软断言：带宽↑ → 距离分辨率↑（分辨率数值↓）；聚焦阶段达预期 kL1RdaImage；
 *     图像质量指标非 NaN / 合理。
 *   - 每场景录制可回放 trace，用 ReplaySarTrace 做确定性回归。
 *
 * @par 与 AR/EOS/ESR 的差异
 *   - SAR 平台/点目标 public 输入使用 LLA+NED；外部 IQ 的 pulse_states 由调用方直接填
 *     scene-center-relative ENU，库内无坐标适配器。
 *   - SAR 单周期即完成聚焦成像（非多周期跟踪）。
 *   - 输出是聚焦图像而非航迹/检测；CSV 只记录图像质量摘要，复数像素不入 CSV。
 *
 * @par 运行方式
 *   ./sar_batch_validation [--suite sweep|sequence|all] [--scenario ID]
 *                          [--output-dir PATH] [--list-scenarios]
 *   默认输出 /tmp/1q/batch_validation/sar/
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <oneq/FsCompat.h>
#include <limits>
#include <string>
#include <system_error>
#include <vector>

#include "1q/sar/sar.hpp"
#include "1q/sar/config/SarRuntimeConfigPatch.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "1q/sar/session/SarReplaySession.h"
#include "1q/sar/session/SarSession.h"
#include "1q/sar/session/SarRecordingSession.h"
#include "batch_assertions.h"
#include "batch_checks.h"
#include "batch_cli.h"
#include "csv_writer.h"
#include "batch_replay.h"
#include "config_loader.h"

namespace sar_session = sar::session;
namespace sar_config = sar::config;
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

constexpr const char* kDefaultOutputDir = "/tmp/1q/batch_validation/sar";
constexpr const char* kTraceId = "sar-batch-validation";

// 场景几何常量（与 sar/integration_demo.cpp 同源，确保物理合理）。
constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kPlatformAltitudeM = 10000.0;
constexpr double kPlatformSpeedMps = 180.0;
constexpr double kSceneCenterLatDeg = 40.1;
constexpr double kSceneCenterLonDeg = -105.0;
constexpr double kDefaultDtSec = 0.1;
// 使用强点目标作为成像质量标定源。真实链路预算启用后，原 10 dBsm 目标在
// 100--150 km 扫描范围内低于默认 -10 dB SNR 门限，所有场景都会在 raw echo
// 阶段按设计中止，无法验证后续成像链路。
constexpr double kTargetRcsDbsm = 80.0;

// =============================================================================
// 场景参数表
// =============================================================================

struct SarCase {
  std::string scenario_id;
  double bandwidth_mhz;          ///< 信号带宽（MHz）→ 影响距离分辨率
  double slant_range_km;         ///< 标称斜距（km）
  std::uint32_t azimuth_pulses;  ///< 方位向脉冲数 → 影响方位分辨率 / 孔径长度
  bool sequence{false};
  std::string family{"parameter_sweep"};
};

std::vector<SarCase> BuildSarCases() {
  std::vector<SarCase> cases;
  // 带宽扫描：保持在 sample_rate(1MHz) 以下，避免离散波形分辨率饱和。
  // 距离分辨率 = c/(2*B)，带宽↑ → 分辨率数值↓（更精细）。
  const double bws[] = {0.2, 0.4, 0.6, 0.8};       // MHz
  const double ranges[] = {100.0, 120.0, 150.0};   // km（标称斜距）
  const std::uint32_t pulses[] = {17U, 33U, 65U};  // 方位脉冲数（孔径长度）
  char buf[128];
  // 完整扫描：带宽 × 斜距 × 方位脉冲数，覆盖采样、几何与孔径长度的交互。
  for (double bw : bws) {
    for (double r : ranges) {
      for (std::uint32_t p : pulses) {
        SarCase c;
        std::snprintf(buf, sizeof(buf), "sar_bw%.1f_r%.0fkm_p%u", bw, r, p);
        c.scenario_id = buf;
        c.bandwidth_mhz = bw;
        c.slant_range_km = r;
        c.azimuth_pulses = p;
        cases.push_back(c);
      }
    }
  }
  return cases;
}

std::vector<SarCase> BuildSarSequenceCases() {
  const char* ids[] = {"sar_seq_multi_scatterer_resolution", "sar_seq_squint_gate_recovery",
                       "sar_seq_raw_to_image", "sar_seq_invalid_runtime_atomic",
                       "sar_seq_invalid_input_recovery", "sar_seq_low_snr_recovery"};
  std::vector<SarCase> cases;
  for (const char* id : ids) {
    SarCase c;
    c.scenario_id = id;
    c.bandwidth_mhz = 0.8;
    c.slant_range_km = 100.0;
    c.azimuth_pulses = 17U;
    c.sequence = true;
    c.family = std::strstr(id, "multi_scatterer") != nullptr ? "multi_scatterer_imaging" :
               std::strstr(id, "runtime") != nullptr || std::strstr(id, "raw_to_image") != nullptr ?
                   "runtime_reconfiguration" :
               std::strstr(id, "squint") != nullptr ? "geometry_gate_recovery" :
                                                        "processing_failure_recovery";
    cases.push_back(c);
  }
  return cases;
}

std::uint32_t CycleCount(const SarCase& c) { return c.sequence ? 5U : 1U; }

const char* PhaseFor(const SarCase& c, std::uint32_t cycle) {
  if (!c.sequence) return "sweep";
  if (cycle == 1U) return "establish";
  if (cycle <= 3U) return "interruption";
  return "recovery";
}

// =============================================================================
// config 按场景调整
// =============================================================================

/// 把场景参数写入 SarSessionConfig。
///
/// @note SAR 参数强耦合（PRF / 平台速度 / 孔径时间 / 斜距 / 波长 / 采样窗口），
///   这里以 sar.json / sar/integration_demo.cpp 共享的验证参数集为基准
///  （sample_rate=1MHz, pulse_width=20us, range_sample=1024, PRF=100Hz, slant=100km），
///   仅在保持采样窗口与孔径时间自洽的前提下小幅扫描带宽 / 斜距 / 方位脉冲数：
///   - 保持 sample_rate=1MHz、pulse_width=20us 固定（满足 ceil(20) << 1024）。
///   - 合成孔径时间 = azimuth_pulses / PRF，与孔径长度自洽。
void ApplyCaseToConfig(const SarCase& c, sar_config::SarSessionConfig& config) {
  // 基准参数（demo 验证过）。
  config.hardware.sample_rate_hz = 1.0e6;
  config.hardware.pulse_width_s = 20.0e-6;
  config.hardware.pulse_repetition_frequency_hz = 100.0;  // PRF=100Hz
  config.mission.range_sample_count = 1024U;
  config.mission.platform_speed_mps = 180.0;
  config.mission.scene_center_latitude_deg = kSceneCenterLatDeg;
  config.mission.scene_center_longitude_deg = kSceneCenterLonDeg;
  config.mission.scene_center_altitude_m = 0.0;

  // 场景扫描维度。
  config.hardware.bandwidth_hz = c.bandwidth_mhz * 1.0e6;
  config.mission.nominal_slant_range_m = c.slant_range_km * 1000.0;
  config.mission.azimuth_pulse_count = c.azimuth_pulses;
}

// =============================================================================
// 输入构造（仿 sar/integration_demo.cpp::MakeCycleInput）
// =============================================================================

/// 构造单周期 SAR 输入：平台沿东向匀速、场景中心一个静止点目标。
/// 直接填充 SarCycleInput（SAR 内部存 LLA，无需 Adapter 做坐标转换）。
sar_session::SarCycleInput MakeCycleInput(std::uint32_t cycle_index,
                                          const sar_config::SarMissionConfig& mission,
                                          const SarCase* scenario = nullptr) {
  const double elapsed_s = static_cast<double>(cycle_index - 1) * kDefaultDtSec;
  const double east_disp_m = kPlatformSpeedMps * elapsed_s;
  const double delta_lon_deg =
      east_disp_m / (kEarthRadiusM * std::cos(kSceneCenterLatDeg * kPi / 180.0)) * (180.0 / kPi);

  sar_session::SarCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = static_cast<float>(kDefaultDtSec);

  // 按每个场景的标称斜距构造真实几何，避免只改变配置标签而物理输入不变。
  const double vertical_separation_m = kPlatformAltitudeM - mission.scene_center_altitude_m;
  const double horizontal_range_m =
      std::sqrt(std::max(0.0, mission.nominal_slant_range_m * mission.nominal_slant_range_m -
                                  vertical_separation_m * vertical_separation_m));
  const double platform_lat_offset_deg = horizontal_range_m / kEarthRadiusM * (180.0 / kPi);
  input.platform.time_s = elapsed_s;
  input.platform.latitude_deg = kSceneCenterLatDeg - platform_lat_offset_deg;
  input.platform.longitude_deg = kSceneCenterLonDeg + delta_lon_deg;
  input.platform.altitude_m = kPlatformAltitudeM;
  input.platform.velocity_north_mps = 0.0;
  input.platform.velocity_east_mps = kPlatformSpeedMps;
  input.platform.velocity_down_mps = 0.0;
  input.platform.roll_deg = 0.0;
  input.platform.pitch_deg = 0.0;
  input.platform.yaw_deg = 90.0;

  sar_session::SarPointTarget target;
  target.target_id = 101;
  target.target_name = "scene_center_target";
  target.latitude_deg = mission.scene_center_latitude_deg;
  target.longitude_deg = mission.scene_center_longitude_deg;
  target.altitude_m = mission.scene_center_altitude_m;
  target.radar_cross_section_dbsm = kTargetRcsDbsm;
  input.point_targets = {target};

  if (scenario != nullptr && scenario->scenario_id == "sar_seq_multi_scatterer_resolution") {
    sar_session::SarPointTarget range_offset = target;
    range_offset.target_id = 102U;
    range_offset.target_name = "range_offset_scatterer";
    range_offset.latitude_deg += 0.00025;
    sar_session::SarPointTarget azimuth_offset = target;
    azimuth_offset.target_id = 103U;
    azimuth_offset.target_name = "azimuth_offset_scatterer";
    azimuth_offset.longitude_deg += 0.00035;
    input.point_targets.push_back(range_offset);
    input.point_targets.push_back(azimuth_offset);
  }
  if (scenario != nullptr && scenario->scenario_id == "sar_seq_squint_gate_recovery" &&
      cycle_index >= 2U && cycle_index <= 3U) {
    input.platform.longitude_deg += 0.5;
    input.platform.time_s += 10.0;
  }
  if (scenario != nullptr && scenario->scenario_id == "sar_seq_invalid_input_recovery") {
    if (cycle_index == 2U) input.dt_sec = 0.0f;
    if (cycle_index == 3U) input.point_targets[0].latitude_deg =
        std::numeric_limits<double>::quiet_NaN();
  }
  if (scenario != nullptr && scenario->scenario_id == "sar_seq_low_snr_recovery" &&
      cycle_index >= 2U && cycle_index <= 4U) {
    input.point_targets[0].radar_cross_section_dbsm = -40.0;
  }

  return input;
}

// =============================================================================
// CSV schema
// =============================================================================

constexpr const char* kCycleHeader =
    "scenario_id,suite,scenario_family,phase,cycle_index,executed_this_cycle,has_error,abort_reason,"
    "completed_stage,has_raw_echo,has_range_compressed_echo,has_l1_image,has_l3_bp_image,"
    "estimated_snr_db,center_slant_range_m,range_sample_count,azimuth_pulse_count,"
    "image_entropy_nats,image_contrast,range_resolution_3db_m,azimuth_resolution_3db_m,"
    "range_width_3db_bins,azimuth_width_3db_bins,has_image_quality_metrics,"
    "image_resolution_m_valid,diag_warning_count,diag_error_count,image_rows,image_cols";

constexpr const char* kScenarioHeader =
    "scenario_id,suite,scenario_family,bandwidth_mhz,slant_range_km,azimuth_pulses,executed,replay_ok,"
    "replay_compared,replay_divergence,completed_stage,estimated_snr_db,"
    "range_resolution_3db_m,azimuth_resolution_3db_m,image_entropy_nats,image_contrast,"
    "has_image_quality_metrics,warning_count,error_count,expected_failure_count,"
    "contract_check_count,contract_failure_count,failure_marker_count,warnings";

// =============================================================================
// 单场景执行
// =============================================================================

struct CycleMetrics {
  std::uint32_t cycle_index{0};
  bool executed{false};
  bool has_error{false};  // 推导：status 为校验/执行拒绝（规则 14 可推导字段）
  int abort_reason{0};
  int completed_stage{0};
  bool has_raw_echo{false};
  bool has_rc_echo{false};
  bool has_l1{false};
  bool has_l3{false};
  double snr_db{0.0};
  double center_slant_range_m{0.0};
  std::uint32_t range_samples{0};
  std::uint32_t azimuth_pulses{0};
  double image_entropy{0.0};
  double image_contrast{0.0};
  double range_res_m{0.0};
  double az_res_m{0.0};
  double range_width_bins{0.0};
  double az_width_bins{0.0};
  bool has_iqm{false};
  bool res_valid{false};
  std::size_t diag_warn{0};
  std::size_t diag_err{0};
  std::uint32_t img_rows{0};
  std::uint32_t img_cols{0};
};

CycleMetrics ExtractCycleMetrics(const sar_session::SarCycleResult& r) {
  CycleMetrics m;
  m.cycle_index = r.input_cycle_index;
  m.executed = (r.status == sar_session::SarCycleStatus::kCompleted);
  m.has_error = r.status == sar_session::SarCycleStatus::kRejectedInvalidInput ||
                r.status == sar_session::SarCycleStatus::kRejectedExecution;
  m.abort_reason = static_cast<int>(r.abort_reason);
  const auto& f = r.product.output_frame;
  m.completed_stage = static_cast<int>(f.completed_stage);
  m.has_raw_echo = f.has_raw_echo;
  m.has_rc_echo = f.has_range_compressed_echo;
  m.has_l1 = f.has_l1_image;
  m.has_l3 = f.has_l3_bp_image;
  m.snr_db = f.estimated_snr_db;
  m.center_slant_range_m = f.center_slant_range_m;
  m.range_samples = f.range_sample_count;
  m.azimuth_pulses = f.azimuth_pulse_count;
  m.image_entropy = f.image_entropy_nats;
  m.image_contrast = f.image_contrast;
  m.range_res_m = f.range_resolution_3db_m;
  m.az_res_m = f.azimuth_resolution_3db_m;
  m.range_width_bins = f.range_width_3db_bins;
  m.az_width_bins = f.azimuth_width_3db_bins;
  m.has_iqm = f.has_image_quality_metrics;
  m.res_valid = f.image_resolution_m_valid;
  for (const auto& d : r.issues) {
    if (d.severity == sar_session::SarIssueSeverity::kWarning) ++m.diag_warn;
    if (d.severity == sar_session::SarIssueSeverity::kError) ++m.diag_err;
  }
  m.img_rows = r.product.focused_image.row_count;
  m.img_cols = r.product.focused_image.column_count;
  return m;
}

struct ScenarioSummary {
  std::string scenario_id;
  std::string suite;
  std::string scenario_family;
  double bandwidth_mhz{0.0};
  double slant_range_km{0.0};
  std::uint32_t azimuth_pulses{0};
  bool executed{false};
  bool has_error{false};
  int abort_reason{0};
  std::size_t diagnostic_warning_count{0};
  std::size_t diagnostic_error_count{0};
  bool replay_ok{false};
  std::uint64_t replay_compared{0};
  bool replay_divergence{false};
  int completed_stage{0};
  double snr_db{0.0};
  double range_res_m{0.0};
  double az_res_m{0.0};
  double image_entropy{0.0};
  double image_contrast{0.0};
  bool has_iqm{false};
  std::size_t expected_failure_count{0U};
  std::size_t contract_check_count{0U};
  std::size_t contract_failure_count{0U};
  std::uint64_t failure_marker_count{0U};
  WarningCollector warnings;
};

ScenarioSummary RunSarScenario(const SarCase& c, const sar_config::SarSessionConfig& base_config,
                               const std::string& output_dir, CsvWriter& cycle_writer,
                               ContractCheckCollector& checks) {
  ScenarioSummary s;
  s.scenario_id = c.scenario_id;
  s.suite = c.sequence ? "sequence" : "sweep";
  s.scenario_family = c.family;
  s.bandwidth_mhz = c.bandwidth_mhz;
  s.slant_range_km = c.slant_range_km;
  s.azimuth_pulses = c.azimuth_pulses;

  // 从基础配置读取不变的几何参数（PRF / 处理开关）。
  sar_config::SarSessionConfig config = base_config;
  ApplyCaseToConfig(c, config);

  const std::string trace_dir = output_dir + "/traces/" + c.scenario_id;
  auto replay_writer =
      batch_validation::MakeReplayWriter(trace_dir, ModuleName::kSar, kTraceId, c.scenario_id);

  CycleMetrics m;
  std::vector<CycleMetrics> metrics;
  const std::uint32_t cycle_count = CycleCount(c);
  metrics.reserve(cycle_count);
  bool invalid_patch_rejected = false;
  {
    sar_session::SarRecordingSessionOptions options;
    options.replay_writer = replay_writer;
    options.record_config_on_construct = true;
    sar_session::SarRecordingSession session(config, options);

    const char* previous_phase = nullptr;
    for (std::uint32_t cycle_index = 1U; cycle_index <= cycle_count; ++cycle_index) {
      const char* phase = PhaseFor(c, cycle_index);
      if (previous_phase == nullptr || std::strcmp(previous_phase, phase) != 0) {
        std::fprintf(stderr, "  [phase] scenario=%s phase=%s cycle=%u\n",
                     c.scenario_id.c_str(), phase, cycle_index);
        previous_phase = phase;
      }
      if (c.scenario_id == "sar_seq_raw_to_image" && (cycle_index == 2U || cycle_index == 4U)) {
        sar_config::SarRuntimeConfigPatch patch;
        patch.has_enable_l1_rda_imaging = true;
        patch.enable_l1_rda_imaging = cycle_index == 4U;
        (void)session.TryApplyRuntimeConfig(patch);
      }
      if (c.scenario_id == "sar_seq_invalid_runtime_atomic" && cycle_index == 2U) {
        sar_config::SarRuntimeConfigPatch patch;
        patch.has_enable_raw_echo_generation = true;
        patch.enable_raw_echo_generation = false;
        patch.has_enable_l1_rda_imaging = true;
        patch.enable_l1_rda_imaging = true;
        invalid_patch_rejected = !session.session().TryApplyRuntimeConfig(patch);
      } else if (c.scenario_id == "sar_seq_invalid_runtime_atomic" && cycle_index == 3U) {
        sar_config::SarRuntimeConfigPatch patch;
        patch.has_enable_raw_echo_generation = true;
        patch.enable_raw_echo_generation = true;
        patch.has_enable_l1_rda_imaging = true;
        patch.enable_l1_rda_imaging = true;
        (void)session.TryApplyRuntimeConfig(patch);
      }
      sar_session::SarCycleInput input = MakeCycleInput(cycle_index, config.mission, &c);
      const sar_session::SarCycleResult result = session.StepWithResult(input);
      m = ExtractCycleMetrics(result);
      metrics.push_back(m);

      std::fprintf(cycle_writer.file(),
                 "%s,%s,%s,%s,%u,%d,%d,%d,%d,%d,%d,%d,%d,%.5f,%.3f,%u,%u,%.5f,%.5f,%.5f,"
                 "%.5f,%.5f,%.5f,%d,%d,%zu,%zu,%u,%u\n",
                 c.scenario_id.c_str(), c.sequence ? "sequence" : "sweep", c.family.c_str(),
                 phase, m.cycle_index, static_cast<int>(m.executed),
                 static_cast<int>(m.has_error),
                 m.abort_reason, m.completed_stage,
                 static_cast<int>(m.has_raw_echo), static_cast<int>(m.has_rc_echo),
                 static_cast<int>(m.has_l1), static_cast<int>(m.has_l3), m.snr_db,
                 m.center_slant_range_m, m.range_samples, m.azimuth_pulses, m.image_entropy,
                 m.image_contrast, m.range_res_m, m.az_res_m, m.range_width_bins, m.az_width_bins,
                 static_cast<int>(m.has_iqm), static_cast<int>(m.res_valid), m.diag_warn,
                 m.diag_err, m.img_rows, m.img_cols);
    }
    replay_writer->Flush();
  }

  s.executed = !metrics.empty() && metrics.back().executed;

  s.completed_stage = m.completed_stage;
  s.has_error = m.has_error;
  s.abort_reason = m.abort_reason;
  s.diagnostic_warning_count = m.diag_warn;
  s.diagnostic_error_count = m.diag_err;
  s.snr_db = m.snr_db;
  s.range_res_m = m.range_res_m;
  s.az_res_m = m.az_res_m;
  s.image_entropy = m.image_entropy;
  s.image_contrast = m.image_contrast;
  s.has_iqm = m.has_iqm;

  // 回放（SAR 单周期，compared 应为 1）。
  const sar_session::SarReplaySessionResult replay = sar_session::ReplaySarTrace(trace_dir);
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
    std::size_t nonexecuted_count = 0U;
    for (const CycleMetrics& cycle : metrics) if (!cycle.executed) ++nonexecuted_count;
    const std::size_t expected_nonexecuted =
        c.scenario_id == "sar_seq_squint_gate_recovery" ||
        c.scenario_id == "sar_seq_invalid_input_recovery" ||
        c.scenario_id == "sar_seq_low_snr_recovery" ? 2U : 0U;
    s.expected_failure_count = expected_nonexecuted +
                               (c.scenario_id == "sar_seq_invalid_runtime_atomic" ? 1U : 0U);
    checks.Add(c.scenario_id, "replay", cycle_count, "replay_complete",
               std::to_string(metrics.size()), std::to_string(s.replay_compared),
               replay.ok && !s.replay_divergence && s.replay_compared == metrics.size());
    checks.Add(c.scenario_id, "recovery", cycle_count, "expected_nonexecuted_cycles",
               std::to_string(expected_nonexecuted), std::to_string(nonexecuted_count),
               expected_nonexecuted == nonexecuted_count);
    const std::uint64_t expected_markers = expected_nonexecuted;
    checks.Add(c.scenario_id, "replay", cycle_count, "failure_marker_count",
               std::to_string(expected_markers), std::to_string(s.failure_marker_count),
               expected_markers == s.failure_marker_count);
    checks.Add(c.scenario_id, "recovery", cycle_count, "recovery_produces_image", "executed",
               metrics.back().executed ? "executed" : std::to_string(metrics.back().abort_reason),
               metrics.back().executed);
    if (c.scenario_id == "sar_seq_invalid_runtime_atomic") {
      checks.Add(c.scenario_id, "interruption", 2U, "invalid_runtime_atomic_rejection", "rejected",
                 invalid_patch_rejected ? "rejected" : "accepted", invalid_patch_rejected);
    }
    if (c.scenario_id == "sar_seq_raw_to_image") {
      checks.Add(c.scenario_id, "interruption", 3U, "range_compression_only_stage", "no_l1",
                 metrics[2].has_l1 ? "has_l1" : "no_l1", !metrics[2].has_l1);
    }
  }

  // 软断言
  if (!s.executed) {
    s.warnings.Error("cycle not executed");
  } else {
    // ① 聚焦阶段应达 kL1RdaImage 或更高，不绑定枚举的整数编码。
    if (s.completed_stage < static_cast<int>(sar_session::SarProcessingStage::kL1RdaImage)) {
      s.warnings.Warn("completed_stage below kL1RdaImage: " + std::to_string(s.completed_stage));
    }
    // ② 图像质量指标应有效
    if (!s.has_iqm) {
      s.warnings.Warn("has_image_quality_metrics is false");
    }
    // ③ SNR 应为有限值
    if (!std::isfinite(s.snr_db)) {
      s.warnings.Warn("estimated_snr_db not finite");
    }
    // ④ 图像熵/对比度应合理非零
    if (s.image_entropy <= 0.0) {
      s.warnings.Warn("image_entropy_nats non-positive: " + std::to_string(s.image_entropy));
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

/// 跨场景趋势：带宽↑ → 距离分辨率数值↓（分辨率更高）。
void CheckCrossScenarioTrends(std::vector<ScenarioSummary>& summaries) {
  std::vector<std::pair<double, double>> by_bw;  // (bandwidth, range_resolution)
  for (const auto& s : summaries) {
    if (std::abs(s.slant_range_km - 100.0) < 1e-9 && s.azimuth_pulses == 33U && s.has_iqm) {
      by_bw.emplace_back(s.bandwidth_mhz, s.range_res_m);
    }
  }
  std::sort(by_bw.begin(), by_bw.end());
  std::vector<double> res;
  for (auto& pr : by_bw) res.push_back(pr.second);
  if (res.size() >= 2 && !batch_validation::IsMonotonicNonIncreasing(res)) {
    for (auto& s : summaries) {
      if (std::abs(s.slant_range_km - 100.0) < 1e-9 && s.azimuth_pulses == 33U) {
        s.warnings.Warn(
            "cross-scenario: range_resolution not improving (decreasing) with bandwidth");
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
  const std::string config_path = BATCH_CONFIG_DIR "/sar.json";

  std::fprintf(stderr, "=== SAR 批量场景验证 ===\n");
  std::fprintf(stderr, "  配置: %s\n  输出: %s\n", config_path.c_str(), output_dir.c_str());

  std::vector<SarCase> cases;
  if (batch_validation::IncludesSweep(cli.suite)) cases = BuildSarCases();
  if (batch_validation::IncludesSequence(cli.suite)) {
    const std::vector<SarCase> sequences = BuildSarSequenceCases();
    cases.insert(cases.end(), sequences.begin(), sequences.end());
  }
  if (cli.list_scenarios) {
    for (const SarCase& c : cases) std::printf("%s\n", c.scenario_id.c_str());
    return 0;
  }
  if (!cli.scenario_id.empty()) {
    cases.erase(std::remove_if(cases.begin(), cases.end(), [&](const SarCase& c) {
                  return c.scenario_id != cli.scenario_id;
                }), cases.end());
  }
  if (cases.empty()) {
    std::fprintf(stderr, "FATAL: no scenario matched\n");
    return 1;
  }

  sar_config::SarSessionConfig base_config;
  std::string load_error;
  if (!examples::LoadSarSessionConfigFromFile(config_path.c_str(), &base_config, &load_error)) {
    std::fprintf(stderr, "FATAL: 加载配置失败 %s: %s\n", config_path.c_str(), load_error.c_str());
    return 1;
  }

  std::error_code ec;
  oneq_fs::create_directories(output_dir, ec);
  if (ec) {
    std::fprintf(stderr, "FATAL: 无法创建输出目录 %s: %s\n", output_dir.c_str(),
                 ec.message().c_str());
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
    summaries.push_back(RunSarScenario(cases[i], base_config, output_dir, cycle_writer, checks));
    summaries.back().contract_check_count = checks.size() - check_begin;
    for (std::size_t j = check_begin; j < checks.checks().size(); ++j) {
      if (!checks.checks()[j].passed && checks.checks()[j].severity == Severity::kError) {
        ++summaries.back().contract_failure_count;
      }
    }
  }
  cycle_writer.Flush();

  CheckCrossScenarioTrends(summaries);

  for (const auto& s : summaries) {
    std::fprintf(stderr,
                 "  [scenario] module=SAR id=%s bandwidth_mhz=%.3f slant_range_km=%.3f "
                 "azimuth_pulses=%u executed=%d/%u stage=%d snr_db=%.5f range_res_m=%.5f "
                 "az_res_m=%.5f entropy=%.5f contrast=%.5f iqm=%d has_error=%d abort_reason=%d "
                 "diag_warn=%zu diag_error=%zu replay_ok=%d compared=%llu divergence=%d "
                 "warn=%zu error=%zu\n",
                 s.scenario_id.c_str(), s.bandwidth_mhz, s.slant_range_km, s.azimuth_pulses,
                 static_cast<int>(s.executed), s.suite == "sequence" ? 5U : 1U,
                 s.completed_stage, s.snr_db, s.range_res_m,
                 s.az_res_m, s.image_entropy, s.image_contrast, static_cast<int>(s.has_iqm),
                 static_cast<int>(s.has_error), s.abort_reason,
                 s.diagnostic_warning_count, s.diagnostic_error_count,
                 static_cast<int>(s.replay_ok),
                 static_cast<unsigned long long>(s.replay_compared),
                 static_cast<int>(s.replay_divergence),
                 s.warnings.Count(Severity::kWarning), s.warnings.Count(Severity::kError));
    std::fprintf(scenario_writer.file(),
                 "%s,%s,%s,%.3f,%.3f,%u,%d,%d,%llu,%d,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%zu,%zu,%zu,%zu,%zu,%llu,%s\n",
                 s.scenario_id.c_str(), s.suite.c_str(), s.scenario_family.c_str(),
                 s.bandwidth_mhz, s.slant_range_km, s.azimuth_pulses,
                 static_cast<int>(s.executed), static_cast<int>(s.replay_ok),
                 static_cast<unsigned long long>(s.replay_compared),
                 static_cast<int>(s.replay_divergence), s.completed_stage, s.snr_db, s.range_res_m,
                 s.az_res_m, s.image_entropy, s.image_contrast, static_cast<int>(s.has_iqm),
                 s.warnings.Count(Severity::kWarning), s.warnings.Count(Severity::kError),
                 s.expected_failure_count, s.contract_check_count, s.contract_failure_count,
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
  std::fprintf(stderr, "\n=== SAR 批量验证完成 ===\n");
  std::fprintf(stderr, "  场景数: %zu\n  周期 CSV: %s\n  场景 CSV: %s\n", summaries.size(),
               cycles_csv.c_str(), scenarios_csv.c_str());
  std::fprintf(stderr, "  软断言 warning: %zu, error: %zu\n", total_warn, total_err);
  std::fprintf(stderr, "  回放分叉场景: %zu\n", replay_div);
  std::fprintf(stderr, "  trace 目录: %s/traces/<scenario_id>/\n", output_dir.c_str());
  return (total_err > 0 || replay_div > 0 || checks.FailureCount() > 0U) ? 2 : 0;
}
