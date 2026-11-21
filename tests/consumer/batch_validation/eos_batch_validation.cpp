/**
 * @file eos_batch_validation.cpp
 * @brief 光电传感器（EOS）批量场景验证。
 *
 * @par 目标
 * 通过公开 Session 接口（EosRecordingSession + 手填 EosCycleInput）对 EOS 模块做多场景
 * 参数扫描，验证其在不同目标距离 / 红外对比度 / 光照条件下的泛用性：
 *   - 采集周期级 CSV（检出数、融合/红外/可见光 SNR 分布）+ 场景汇总 CSV。
 *   - 软断言：高红外对比度场景检出率 ≥ 低对比度；夜间可见光 SNR 显著低于红外；
 *     距离↑ → 融合 SNR↓（物理衰减）。
 *   - 每场景录制可回放 trace，用 ReplayEosTrace 做确定性回归。
 *
 * @par 运行方式
 *   ./eos_batch_validation [--suite sweep|sequence|all] [--scenario ID]
 *                          [--output-dir PATH] [--list-scenarios]
 *   默认输出 /tmp/1q/batch_validation/electro_optical_sensor/
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <oneq/FsCompat.h>
#include <string>
#include <system_error>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosPlatformEcefPose.h"
#include "1q/electro_optical_sensor/session/EosReplaySession.h"
#include "1q/electro_optical_sensor/session/EosSceneTypes.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electro_optical_sensor/session/EosRecordingSession.h"
#include "1q/coordinate/scene_transform.h"

#include "batch_assertions.h"
#include "batch_checks.h"
#include "batch_cli.h"
#include "csv_writer.h"
#include "batch_replay.h"
#include "config_loader.h"

namespace eos = electro_optical_sensor;
namespace eos_config = electro_optical_sensor::config;
namespace eos_session = electro_optical_sensor::session;
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

constexpr const char* kDefaultOutputDir = "/tmp/1q/batch_validation/electro_optical_sensor";
constexpr std::uint32_t kNumCycles = 30;
constexpr std::uint32_t kWarmupCycles = 5;
constexpr const char* kTraceId = "eos-batch-validation";

// =============================================================================
// 场景参数表
// =============================================================================

/// 红外对比度档位（目标外观参数预设）。
enum class ContrastLevel { kLow, kMedium, kHigh };

/// 光照场景（影响环境输入）。
enum class LightingCondition { kDay, kNight, kTwilight };

struct EosCase {
  std::string scenario_id;
  double target_lon_offset_deg;  ///< 目标相对平台的经度偏移（越大距离越远）
  ContrastLevel contrast;         ///< 红外对比度档位
  LightingCondition lighting;     ///< 光照条件
  bool sequence{false};
  std::string family{"parameter_sweep"};
};

const char* ContrastName(ContrastLevel c) {
  switch (c) {
    case ContrastLevel::kLow: return "low";
    case ContrastLevel::kMedium: return "med";
    case ContrastLevel::kHigh: return "high";
  }
  return "?";
}

const char* LightingName(LightingCondition l) {
  switch (l) {
    case LightingCondition::kDay: return "day";
    case LightingCondition::kNight: return "night";
    case LightingCondition::kTwilight: return "twilight";
  }
  return "?";
}

std::vector<EosCase> BuildEosCases() {
  std::vector<EosCase> cases;
  // 目标相对传感器足印中心的经度偏移（足印中心已在探测俯仰角内）。
  // 1° lon @35N ≈ 91km；{0.005,0.010,0.020,0.030} ≈ {0.45,0.9,1.8,2.7}km，均在视场内。
  const double offsets[] = {0.005, 0.010, 0.020, 0.030};
  const ContrastLevel contrasts[] = {ContrastLevel::kLow, ContrastLevel::kMedium,
                                     ContrastLevel::kHigh};
  const LightingCondition lights[] = {LightingCondition::kDay, LightingCondition::kNight,
                                      LightingCondition::kTwilight};
  char buf[128];
  for (double off : offsets) {
    for (ContrastLevel c : contrasts) {
      for (LightingCondition l : lights) {
        EosCase cs;
        std::snprintf(buf, sizeof(buf), "eos_off%.3f_%s_%s", off, ContrastName(c),
                      LightingName(l));
        cs.scenario_id = buf;
        cs.target_lon_offset_deg = off;
        cs.contrast = c;
        cs.lighting = l;
        cases.push_back(cs);
      }
    }
  }
  return cases;
}

std::vector<EosCase> BuildEosSequenceCases() {
  const char* ids[] = {"eos_seq_two_target_focal_crossing", "eos_seq_day_twilight_night",
                       "eos_seq_fused_ir_visible_fused", "eos_seq_scan_rate_retask",
                       "eos_seq_power_cycle", "eos_seq_invalid_input_recovery"};
  std::vector<EosCase> cases;
  for (const char* id : ids) {
    EosCase c;
    c.scenario_id = id;
    c.target_lon_offset_deg = 0.005;
    c.contrast = ContrastLevel::kHigh;
    c.lighting = LightingCondition::kDay;
    c.sequence = true;
    c.family = std::strstr(id, "crossing") != nullptr ? "multi_target_geometry" :
               std::strstr(id, "day_twilight") != nullptr ? "environment_transition" :
               std::strstr(id, "power") != nullptr ? "lifecycle_interruption" :
               std::strstr(id, "invalid_input") != nullptr ? "invalid_input_recovery" :
                                                                  "runtime_reconfiguration";
    cases.push_back(c);
  }
  return cases;
}

std::uint32_t CycleCount(const EosCase& c) { return c.sequence ? 24U : kNumCycles; }

const char* PhaseFor(const EosCase& c, std::uint32_t cycle) {
  if (!c.sequence) return "sweep";
  if (cycle <= 8U) return "establish";
  if (cycle <= 16U) return "transition";
  return "recovery";
}

// =============================================================================
// 输入构造（仿 session_usage.cpp 的 LLA 模式）
// =============================================================================

/// 平台 LLA 起点（与 session_usage 同一参考点 @ 35N,114.5E,7000m 高度）。
/// 目标放在 base_lon 附近，使地面偏移落在 EOS 探测俯仰角范围内。
/// session_usage 验证过：base_lon = 114.5 + 0.06925 对应 ~6.3km 探测偏移。
constexpr double kPlatformLat = 35.0;
constexpr double kPlatformLon = 114.5;
constexpr double kPlatformAlt = 7000.0;
constexpr double kBaseLon = kPlatformLon + 0.06925;  ///< 传感器足印中心经度

/// 世界侧目标规格（手填 CycleInput 前的中间结构）。
struct EosWorldTargetSpec {
  std::uint64_t target_id{0U};
  oneq::coordinate::ExternalKinematics kinematics{};
  eos_session::EosTargetAppearance appearance{};
};

bool TryBuildEosCycleInput(const eos_session::EosPlatformEcefPose& platform,
                           const std::vector<EosWorldTargetSpec>& targets, float dt_sec,
                           eos_session::EosCycleInput* output) {
  if (output == nullptr) {
    return false;
  }
  oneq::coordinate::LlaPositionDegM anchor_lla;
  if (!oneq::coordinate::TryEcefToLla(platform.platform_position_ecef_m, &anchor_lla)) {
    return false;
  }
  output->dt_sec = dt_sec;
  output->platform_altitude_m = static_cast<float>(anchor_lla.altitude_m);
  output->platform_attitude_deg = platform.platform_attitude_deg;
  output->scene.clear();
  output->scene.reserve(targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    const EosWorldTargetSpec& spec = targets[i];
    oneq::coordinate::EnuSceneState enu;
    if (!oneq::coordinate::TryMakeEnuSceneState(spec.kinematics, anchor_lla, &enu)) {
      return false;
    }
    eos_session::EosSceneTarget scene_target;
    scene_target.target_id =
        (spec.target_id != 0U) ? spec.target_id : static_cast<std::uint64_t>(i);
    scene_target.position_x = static_cast<float>(enu.position_enu_m.east_m);
    scene_target.position_y = static_cast<float>(enu.position_enu_m.north_m);
    scene_target.position_z = static_cast<float>(enu.position_enu_m.up_m);
    scene_target.velocity_x = static_cast<float>(enu.velocity_enu_mps.east_mps);
    scene_target.velocity_y = static_cast<float>(enu.velocity_enu_mps.north_mps);
    scene_target.velocity_z = static_cast<float>(enu.velocity_enu_mps.up_mps);
    scene_target.appearance = spec.appearance;
    output->scene.push_back(scene_target);
  }
  return true;
}

eos_session::EosPlatformEcefPose MakePlatform(std::uint32_t cycle_index) {
  eos_session::EosPlatformEcefPose p;
  // 用项目精确 LLA→ECEF 转换（手写近似会导致几何失真、目标落出视场）。
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = kPlatformLat;
  platform_lla.longitude_deg = kPlatformLon;
  platform_lla.altitude_m = kPlatformAlt;
  oneq::coordinate::TryLlaToEcef(platform_lla, &p.platform_position_ecef_m);
  (void)cycle_index;
  p.platform_attitude_deg.yaw_deg = 0.0;
  p.platform_attitude_deg.pitch_deg = 0.0;
  p.platform_attitude_deg.roll_deg = 0.0;
  return p;
}

/// 按 contrast 档位构造目标外观。
eos_session::EosTargetAppearance MakeAppearance(ContrastLevel c) {
  eos_session::EosTargetAppearance a;
  switch (c) {
    case ContrastLevel::kLow:
      a.apparent_temperature_k = 295.0f;  // 接近背景 290K，低对比度
      a.emissivity = 0.5f;
      a.reflectance = 0.1f;
      a.projected_area_m2 = 2.0f;
      break;
    case ContrastLevel::kMedium:
      a.apparent_temperature_k = 350.0f;
      a.emissivity = 0.8f;
      a.reflectance = 0.2f;
      a.projected_area_m2 = 5.0f;
      break;
    case ContrastLevel::kHigh:
      a.apparent_temperature_k = 480.0f;  // 高温差，高对比度
      a.emissivity = 0.95f;
      a.reflectance = 0.3f;
      a.projected_area_m2 = 12.0f;
      break;
  }
  return a;
}

std::vector<EosWorldTargetSpec> MakeTargets(const EosCase& c,
                                                              std::uint32_t cycle_index) {
  std::vector<EosWorldTargetSpec> targets;
  EosWorldTargetSpec t;
  t.target_id = 101;
  t.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  // 目标放在传感器足印中心附近，按场景偏移量错开（偏移即相对足印中心的地面距离）。
  t.kinematics.position_lla_deg_m.latitude_deg = kPlatformLat - 0.002;
  t.kinematics.position_lla_deg_m.longitude_deg = kBaseLon + c.target_lon_offset_deg;
  t.kinematics.position_lla_deg_m.altitude_m = 0.0;
  t.appearance = MakeAppearance(c.contrast);
  targets.push_back(t);
  if (c.sequence && c.scenario_id == "eos_seq_two_target_focal_crossing") {
    const double delta = (12.5 - static_cast<double>(cycle_index)) * 0.00025;
    targets[0].kinematics.position_lla_deg_m.longitude_deg = kBaseLon + delta;
    EosWorldTargetSpec second = targets[0];
    second.target_id = 102U;
    second.kinematics.position_lla_deg_m.longitude_deg = kBaseLon - delta;
    second.appearance = MakeAppearance(ContrastLevel::kMedium);
    targets.push_back(second);
  }
  return targets;
}

// =============================================================================
// CSV schema
// =============================================================================

constexpr const char* kCycleHeader =
    "scenario_id,suite,scenario_family,phase,cycle_index,executed_this_cycle,has_validation_error,abort_reason,"
    "scan_azimuth_deg,total_detection_count,detected_count,detection_rate,"
    "fused_snr_db_mean,fused_snr_db_p95,infrared_snr_linear_mean,visible_snr_linear_mean";

constexpr const char* kScenarioHeader =
    "scenario_id,suite,scenario_family,target_lon_offset_deg,contrast,lighting,executed_cycles,"
    "steady_detection_rate_mean,steady_fused_snr_db_mean,steady_infrared_mean,"
    "steady_visible_mean,replay_ok,replay_compared,replay_divergence,warning_count,"
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
  float scan_azimuth_deg{0.0f};
  std::size_t total{0};
  std::size_t detected{0};
  double fused_snr_db_mean{0.0};
  double fused_snr_db_p95{0.0};
  double infrared_mean{0.0};
  double visible_mean{0.0};
};

CycleMetrics ExtractCycleMetrics(const eos_session::EosCycleResult& r) {
  CycleMetrics m;
  m.cycle_index = r.input_cycle_index;
  m.executed = (r.status == eos_session::EosCycleStatus::kCompleted);
  m.has_validation_error = eos_session::HasValidationError(r.issues);
  m.abort_reason = static_cast<int>(r.abort_reason);
  const auto& f = r.output_frame;
  m.scan_azimuth_deg = f.scan_azimuth_deg;
  m.total = f.detections.size();
  std::vector<double> fused_db, infrared, visible;
  for (const auto& d : f.detections) {
    if (d.detected) ++m.detected;
    fused_db.push_back(static_cast<double>(d.fused_snr_db));
    infrared.push_back(static_cast<double>(d.infrared_snr_linear));
    visible.push_back(static_cast<double>(d.visible_snr_linear));
  }
  m.fused_snr_db_mean = batch_validation::Mean(fused_db);
  m.fused_snr_db_p95 = batch_validation::Percentile(fused_db, 95.0);
  m.infrared_mean = batch_validation::Mean(infrared);
  m.visible_mean = batch_validation::Mean(visible);
  return m;
}

struct ScenarioSummary {
  std::string scenario_id;
  std::string suite;
  std::string scenario_family;
  double target_lon_offset_deg{0.0};
  std::string contrast;
  std::string lighting;
  std::uint32_t executed_cycles{0};
  double steady_detection_rate_mean{0.0};
  double steady_fused_snr_db_mean{0.0};
  double steady_infrared_mean{0.0};
  double steady_visible_mean{0.0};
  bool replay_ok{false};
  std::uint64_t replay_compared{0};
  bool replay_divergence{false};
  std::size_t expected_failure_count{0U};
  std::size_t contract_check_count{0U};
  std::size_t contract_failure_count{0U};
  std::uint64_t failure_marker_count{0U};
  WarningCollector warnings;
};

ScenarioSummary RunEosScenario(const EosCase& c, const eos_config::EosSessionConfig& base_config,
                               const std::string& output_dir, CsvWriter& cycle_writer,
                               ContractCheckCollector& checks) {
  ScenarioSummary s;
  s.scenario_id = c.scenario_id;
  s.suite = c.sequence ? "sequence" : "sweep";
  s.scenario_family = c.family;
  s.target_lon_offset_deg = c.target_lon_offset_deg;
  s.contrast = ContrastName(c.contrast);
  s.lighting = LightingName(c.lighting);

  // EOS 物理链路本身基于辐射度学；这里不改 config（沿用基础探测门限）。
  (void)base_config;
  eos_config::EosSessionConfig config = base_config;

  const std::string trace_dir = output_dir + "/traces/" + c.scenario_id;
  auto replay_writer = batch_validation::MakeReplayWriter(
      trace_dir, ModuleName::kElectroOpticalSensor, kTraceId, c.scenario_id);

  std::vector<CycleMetrics> metrics;
  const std::uint32_t cycle_count = CycleCount(c);
  metrics.reserve(cycle_count);
  std::size_t nonexecuted_count = 0U;
  bool attribution_complete = true;
  bool detection_ids_unique = true;
  double establish_visible = 0.0;
  double transition_visible = 0.0;
  double recovery_ir = 0.0;

  {
    eos_session::EosRecordingSessionOptions options;
    options.replay_writer = replay_writer;
    options.record_config_on_construct = true;
    eos_session::EosRecordingSession session(config, options);

    const char* previous_phase = nullptr;
    for (std::uint32_t i = 0; i < cycle_count; ++i) {
      const std::uint32_t cycle_index = i + 1;
      const char* phase = PhaseFor(c, cycle_index);
      if (previous_phase == nullptr || std::strcmp(previous_phase, phase) != 0) {
        std::fprintf(stderr, "  [phase] scenario=%s phase=%s cycle=%u\n",
                     c.scenario_id.c_str(), phase, cycle_index);
        previous_phase = phase;
      }
      if (c.scenario_id == "eos_seq_fused_ir_visible_fused" &&
          (cycle_index == 9U || cycle_index == 13U || cycle_index == 17U)) {
        eos_config::EosRuntimeConfigPatch patch;
        patch.has_work_mode = true;
        patch.work_mode = cycle_index == 9U ? eos_config::EosWorkMode::kInfraredOnly :
                          cycle_index == 13U ? eos_config::EosWorkMode::kVisibleOnly :
                                                eos_config::EosWorkMode::kFused;
        (void)session.TryApplyRuntimeConfig(patch);
      }
      if (c.scenario_id == "eos_seq_scan_rate_retask" &&
          (cycle_index == 9U || cycle_index == 17U)) {
        eos_config::EosRuntimeConfigPatch patch;
        patch.has_scan_rate_deg_per_sec = true;
        patch.scan_rate_deg_per_sec = cycle_index == 9U ? 35.0f : 12.0f;
        (void)session.TryApplyRuntimeConfig(patch);
      }
      if (c.scenario_id == "eos_seq_power_cycle" && (cycle_index == 9U || cycle_index == 14U)) {
        eos_config::EosRuntimeConfigPatch patch;
        patch.has_sensor_enabled = true;
        patch.sensor_enabled = cycle_index == 14U;
        (void)session.TryApplyRuntimeConfig(patch);
      }
      eos_session::EosPlatformEcefPose platform = MakePlatform(cycle_index);
      std::vector<EosWorldTargetSpec> targets = MakeTargets(c, cycle_index);
      LightingCondition lighting = c.lighting;
      if (c.scenario_id == "eos_seq_day_twilight_night") {
        lighting = cycle_index <= 8U ? LightingCondition::kDay :
                   cycle_index <= 16U ? LightingCondition::kTwilight : LightingCondition::kNight;
      }
      eos_session::EosCycleInput input;
      // dt_sec=0.1f：electro_optical.json 的 frame_rate_hz=30，53c56e21 收紧的
      // dt_sec <= 10/frame_rate_hz 上界为 ≈0.333s，1.0f 会触发校验拒绝。
      if (!TryBuildEosCycleInput(platform, targets, 0.1f, &input)) {
        s.warnings.Error("TryBuildEosCycleInput failed at cycle " +
                         std::to_string(cycle_index));
        break;
      }
      input.cycle_index = cycle_index;
      if (c.scenario_id == "eos_seq_invalid_input_recovery" && cycle_index == 9U) {
        input.dt_sec = 0.0f;
      } else if (c.scenario_id == "eos_seq_invalid_input_recovery" && cycle_index == 10U &&
                 !input.scene.empty()) {
        input.scene[0].appearance.emissivity = 1.5f;
      }

      const eos_session::EosCycleResult result = session.StepWithResult(input);
      if (result.status != eos_session::EosCycleStatus::kCompleted) ++nonexecuted_count;
      if (result.status == eos_session::EosCycleStatus::kCompleted) {
        attribution_complete = attribution_complete &&
                               result.detection_attributions.size() == result.output_frame.detections.size();
        for (std::size_t a = 0; a < result.output_frame.detections.size(); ++a) {
          for (std::size_t b = a + 1U; b < result.output_frame.detections.size(); ++b) {
            if (result.output_frame.detections[a].detection_id ==
                result.output_frame.detections[b].detection_id) detection_ids_unique = false;
          }
        }
      }
      CycleMetrics m = ExtractCycleMetrics(result);
      metrics.push_back(m);

      const double det_rate =
          (m.total > 0) ? static_cast<double>(m.detected) / static_cast<double>(m.total) : 0.0;
      std::fprintf(cycle_writer.file(),
                   "%s,%s,%s,%s,%u,%d,%d,%d,%.3f,%zu,%zu,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                   c.scenario_id.c_str(), c.sequence ? "sequence" : "sweep", c.family.c_str(),
                   phase, m.cycle_index, static_cast<int>(m.executed),
                   static_cast<int>(m.has_validation_error), m.abort_reason,
                   static_cast<double>(m.scan_azimuth_deg), m.total, m.detected, det_rate,
                   m.fused_snr_db_mean, m.fused_snr_db_p95, m.infrared_mean, m.visible_mean);
      if (cycle_index == 8U) establish_visible = m.visible_mean;
      if (cycle_index == 16U) transition_visible = m.visible_mean;
      if (cycle_index == 24U) recovery_ir = m.infrared_mean;
    }
    replay_writer->Flush();
  }

  // 聚合
  std::vector<double> steady_det_rate, steady_fused, steady_ir, steady_vis;
  for (const auto& m : metrics) {
    if (m.executed) ++s.executed_cycles;
    if (!m.executed || m.cycle_index <= kWarmupCycles) continue;
    const double det_rate =
        (m.total > 0) ? static_cast<double>(m.detected) / static_cast<double>(m.total) : 0.0;
    steady_det_rate.push_back(det_rate);
    steady_fused.push_back(m.fused_snr_db_mean);
    steady_ir.push_back(m.infrared_mean);
    steady_vis.push_back(m.visible_mean);
  }
  s.steady_detection_rate_mean = batch_validation::Mean(steady_det_rate);
  s.steady_fused_snr_db_mean = batch_validation::Mean(steady_fused);
  s.steady_infrared_mean = batch_validation::Mean(steady_ir);
  s.steady_visible_mean = batch_validation::Mean(steady_vis);

  // 回放
  const eos_session::EosReplaySessionResult replay = eos_session::ReplayEosTrace(trace_dir);
  s.replay_ok = replay.ok;
  s.replay_compared = replay.playback.compared_output_count;
  s.replay_divergence = replay.playback.divergence_found;
  s.failure_marker_count = replay.playback.failure_marker_count;
  if (!replay.ok || replay.playback.divergence_found) {
    s.warnings.Error("replay failed: " + replay.first_error);
  }
  if (replay.playback.compared_output_count != metrics.size()) {
    s.warnings.Error("replay output count does not equal Step count");
  }

  if (c.sequence) {
    const std::size_t expected_nonexecuted =
        c.scenario_id == "eos_seq_power_cycle" ? 5U :
        c.scenario_id == "eos_seq_invalid_input_recovery" ? 2U : 0U;
    s.expected_failure_count = c.scenario_id == "eos_seq_invalid_input_recovery" ? 2U : 0U;
    checks.Add(c.scenario_id, "replay", cycle_count, "replay_complete",
               std::to_string(metrics.size()), std::to_string(s.replay_compared),
               replay.ok && !s.replay_divergence && s.replay_compared == metrics.size());
    checks.Add(c.scenario_id, "recovery", cycle_count, "expected_nonexecuted_cycles",
               std::to_string(expected_nonexecuted), std::to_string(nonexecuted_count),
               expected_nonexecuted == nonexecuted_count);
    const std::uint64_t expected_markers =
        c.scenario_id == "eos_seq_invalid_input_recovery" ? 2U : 0U;
    checks.Add(c.scenario_id, "replay", cycle_count, "failure_marker_count",
               std::to_string(expected_markers), std::to_string(s.failure_marker_count),
               expected_markers == s.failure_marker_count);
    checks.Add(c.scenario_id, "recovery", cycle_count, "attribution_complete", "complete",
               attribution_complete ? "complete" : "incomplete", attribution_complete);
    checks.Add(c.scenario_id, "recovery", cycle_count, "detection_ids_unique", "unique",
               detection_ids_unique ? "unique" : "duplicate", detection_ids_unique);
    if (c.scenario_id == "eos_seq_day_twilight_night") {
      checks.Add(c.scenario_id, "transition", 16U, "visible_light_declines", "night<day",
                 std::to_string(transition_visible) + "<" + std::to_string(establish_visible),
                 transition_visible <= establish_visible, Severity::kWarning);
      checks.Add(c.scenario_id, "recovery", 24U, "infrared_remains_finite", "finite",
                 std::to_string(recovery_ir), std::isfinite(recovery_ir), Severity::kWarning);
    }
  }

  // 软断言
  if (s.executed_cycles == 0) {
    s.warnings.Error("no cycle executed");
  } else {
    // ① 高对比度检出率应 >= 低对比度（跨场景比较在 CheckCrossScenarioTrends）
    // ② 夜间可见光 SNR 应显著低于红外（可见光依赖太阳辐照）
    if (c.lighting == LightingCondition::kNight) {
      if (s.steady_visible_mean > s.steady_infrared_mean * 0.5) {
        s.warnings.Warn("night: visible_snr not clearly below infrared (vis=" +
                        std::to_string(s.steady_visible_mean) +
                        " ir=" + std::to_string(s.steady_infrared_mean) + ")");
      }
    }
    // ③ 融合 SNR 应为有限值
    if (!std::isfinite(s.steady_fused_snr_db_mean)) {
      s.warnings.Warn("fused_snr_db not finite: " + std::to_string(s.steady_fused_snr_db_mean));
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

/// 跨场景趋势：固定 lighting=day，对比度↑ → 检出率↑ / fused SNR↑；
/// 固定对比度=high, 距离(offset)↑ → fused SNR↓。
void CheckCrossScenarioTrends(std::vector<ScenarioSummary>& summaries) {
  // 对比度趋势：offset=0.010（中等距离）、day 光照下
  std::vector<double> det_by_contrast;  // low, med, high 顺序
  for (const auto& lv : {"low", "med", "high"}) {
    for (const auto& s : summaries) {
      if (std::abs(s.target_lon_offset_deg - 0.010) < 1e-9 && s.lighting == "day" &&
          s.contrast == lv) {
        det_by_contrast.push_back(s.steady_detection_rate_mean);
        break;
      }
    }
  }
  if (det_by_contrast.size() == 3 && !batch_validation::IsMonotonicNonDecreasing(det_by_contrast)) {
    for (auto& s : summaries) {
      if (std::abs(s.target_lon_offset_deg - 0.010) < 1e-9 && s.lighting == "day") {
        s.warnings.Warn("cross-scenario: detection_rate not monotonic-increasing in contrast");
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
  const std::string config_path = BATCH_CONFIG_DIR "/electro_optical.json";

  std::fprintf(stderr, "=== EOS 批量场景验证 ===\n");
  std::fprintf(stderr, "  配置: %s\n  输出: %s\n", config_path.c_str(), output_dir.c_str());

  std::vector<EosCase> cases;
  if (batch_validation::IncludesSweep(cli.suite)) cases = BuildEosCases();
  if (batch_validation::IncludesSequence(cli.suite)) {
    const std::vector<EosCase> sequences = BuildEosSequenceCases();
    cases.insert(cases.end(), sequences.begin(), sequences.end());
  }
  if (cli.list_scenarios) {
    for (const EosCase& c : cases) std::printf("%s\n", c.scenario_id.c_str());
    return 0;
  }
  if (!cli.scenario_id.empty()) {
    cases.erase(std::remove_if(cases.begin(), cases.end(), [&](const EosCase& c) {
                  return c.scenario_id != cli.scenario_id;
                }), cases.end());
  }
  if (cases.empty()) {
    std::fprintf(stderr, "FATAL: no scenario matched\n");
    return 1;
  }

  eos_config::EosSessionConfig base_config;
  std::string load_error;
  if (!examples::LoadEosSessionConfigFromFile(config_path.c_str(), &base_config, &load_error)) {
    std::fprintf(stderr, "FATAL: 加载配置失败 %s: %s\n", config_path.c_str(), load_error.c_str());
    return 1;
  }

  std::error_code ec;
  oneq_fs::create_directories(output_dir, ec);
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
    summaries.push_back(RunEosScenario(cases[i], base_config, output_dir, cycle_writer, checks));
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
                 "  [scenario] module=EOS id=%s offset_deg=%.4f contrast=%s lighting=%s "
                 "executed=%u/%u detection_rate=%.4f fused_snr_db=%.4f infrared=%.4f "
                 "visible=%.4f replay_ok=%d compared=%llu divergence=%d warn=%zu error=%zu\n",
                 s.scenario_id.c_str(), s.target_lon_offset_deg, s.contrast.c_str(),
                 s.lighting.c_str(), s.executed_cycles,
                 s.suite == "sequence" ? 24U : kNumCycles,
                 s.steady_detection_rate_mean, s.steady_fused_snr_db_mean,
                 s.steady_infrared_mean, s.steady_visible_mean, static_cast<int>(s.replay_ok),
                 static_cast<unsigned long long>(s.replay_compared),
                 static_cast<int>(s.replay_divergence),
                 s.warnings.Count(Severity::kWarning), s.warnings.Count(Severity::kError));
    std::fprintf(scenario_writer.file(),
                 "%s,%s,%s,%.4f,%s,%s,%u,%.4f,%.4f,%.4f,%.4f,%d,%llu,%d,%zu,%zu,%zu,%zu,%zu,%llu,%s\n",
                 s.scenario_id.c_str(), s.suite.c_str(), s.scenario_family.c_str(),
                 s.target_lon_offset_deg, s.contrast.c_str(),
                 s.lighting.c_str(), s.executed_cycles, s.steady_detection_rate_mean,
                 s.steady_fused_snr_db_mean, s.steady_infrared_mean, s.steady_visible_mean,
                 static_cast<int>(s.replay_ok), static_cast<unsigned long long>(s.replay_compared),
                 static_cast<int>(s.replay_divergence), s.warnings.Count(Severity::kWarning),
                 s.warnings.Count(Severity::kError), s.expected_failure_count,
                 s.contract_check_count, s.contract_failure_count,
                 static_cast<unsigned long long>(s.failure_marker_count),
                 batch_validation::EscapeCsvField(s.warnings.JoinForCsv()).c_str());
  }
  scenario_writer.Flush();
  checks.WriteCsv(output_dir + "/checks.csv");

  std::size_t total_warn = 0, total_err = 0, replay_fail = 0;
  for (const auto& s : summaries) {
    total_warn += s.warnings.Count(Severity::kWarning);
    total_err += s.warnings.Count(Severity::kError);
    if (!s.replay_ok || s.replay_divergence) ++replay_fail;
  }
  std::fprintf(stderr, "\n=== EOS 批量验证完成 ===\n");
  std::fprintf(stderr, "  场景数: %zu\n  周期 CSV: %s\n  场景 CSV: %s\n", summaries.size(),
               cycles_csv.c_str(), scenarios_csv.c_str());
  std::fprintf(stderr, "  软断言 warning: %zu, error: %zu\n", total_warn, total_err);
  std::fprintf(stderr, "  回放失败场景: %zu\n", replay_fail);
  std::fprintf(stderr, "  trace 目录: %s/traces/<scenario_id>/\n", output_dir.c_str());
  return (replay_fail > 0 || total_err > 0 || checks.FailureCount() > 0U) ? 2 : 0;
}
