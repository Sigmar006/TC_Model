/**
 * @file sbirs_triple_sat_fix_messages/main.cpp
 * @brief 三颗地球静止轨道卫星覆盖 + 消息机制地面站融合；末尾导出 3D 可视化 CSV。
 *
 * 卫星经 on_sbirs_frame_submitted 投递，地面站 GroundStationFusionComponent
 * 订阅收件箱后融合。精度评估交会仍只用前两颗星（库 API 是双视线）。
 *
 * 每周期落盘六份 CSV（sbirs_sats / sbirs_scan / sbirs_truth / sbirs_los /
 * sbirs_fused / sbirs_dual_fix，与验收日志同目录），供 examples/common/viz/
 * sbirs_orbit_viewer.py 构建离线三维查看器：卫星几何与视场锥、真值轨迹、
 * 每星逐目标视线状态（检测/被地球遮挡）、融合航迹与双星交会误差。
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "app/command_routing.h"
#include "app/fs_compat.h"

#include "1q/coordinate/inertial_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/fusion/FusionEngine.h"
#include "1q/fusion/FusedTarget.h"
#include "1q/precision_evaluation/PrecisionEvaluationConfig.h"
#include "1q/precision_evaluation/PrecisionEvaluationSession.h"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"
#include "json_reader.h"
#include "config_loaders/sbirs_sensor/config_loader.h"
#include "csv_writer.h"
#include "app/output_dir.h"
#include "components/ground_station_fusion_component.h"
#include "components/rir_sensor_component.h"
#include "components/sbirs_sensor_component.h"
#include "core/rf_world_broker.h"
#include "core/scene_types.h"
#include "core/world.h"
#include "logger/logger.h"
#include "logger/acceptance_paths.h"
#include "scenes/scene_data.h"
#include "scenes/scene_script.h"

namespace pe = precision_evaluation;

namespace {

#ifndef ONEQ_SCENE_JSON
#define ONEQ_SCENE_JSON \
  "examples/scenes/sbirs_triple_sat_fix_messages/sbirs_triple_sat_fix_messages.json"
#endif
#ifndef PE_DEFAULT_OUTPUT_DIR
#define PE_DEFAULT_OUTPUT_DIR "examples/log"
#endif

struct LoadedSatellite {
  std::string id{};
  std::uint32_t source_id{0U};
  sbirs_sensor::config::SbirsSessionConfig config{};
  oneq::coordinate::EcefPositionM position_ecef_m{};
  oneq::coordinate::EcefVelocityMps velocity_ecef_m_per_s{};
  double attitude_yaw_deg{0.0};
  double attitude_pitch_deg{0.0};
  double attitude_roll_deg{0.0};
  bool cross_cue{false};  // 星间递话开关（2026-09-01 契约：默认关；true=递话+受话）
};

struct LoadedScene {
  std::uint32_t cycles{60U};
  std::string log_dir{};
  float dt_sec{1.0f};
  double utc_julian_day{2451544.2230698913};
  pe::PrecisionEvaluationConfig config{};
  pe::DualSatEphemerisInput ephemeris{};
  std::vector<LoadedSatellite> satellites{};
  std::vector<pe::EvaluationTruthTarget> truth{};
  bool lrdr_enabled{false};
};

struct PendingRirCue {
  bool active{false};
  std::string source_name{};
  std::uint64_t target_id{0U};
  double bearing_time_s{0.0};
  double cue_az_deg{0.0};
  double cue_el_deg{0.0};
};

bool ReadVec3(const examples::JsonValue& value, double* x, double* y, double* z,
              const char* name, std::string* error) {
  if (value.IsNull() || value.Size() != 3U) {
    *error = std::string(name) + " must be [x,y,z]";
    return false;
  }
  *x = value[static_cast<std::size_t>(0)].AsDouble();
  *y = value[static_cast<std::size_t>(1)].AsDouble();
  *z = value[static_cast<std::size_t>(2)].AsDouble();
  return true;
}

bool ReadLla(const examples::JsonValue& value, oneq::coordinate::LlaPositionDegM* lla,
             const char* name, std::string* error) {
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt_m = 0.0;
  if (!ReadVec3(value, &lat_deg, &lon_deg, &alt_m, name, error)) {
    return false;
  }
  *lla = oneq::coordinate::LlaPositionDegM(lat_deg, lon_deg, alt_m);
  if (!oneq::coordinate::IsValid(*lla)) {
    *error = std::string(name) + " is not a valid LLA [lat_deg, lon_deg, alt_m]";
    return false;
  }
  return true;
}

sbirs_sensor::config::SbirsSessionConfig LoadSatelliteConfig(const examples::JsonValue& block) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w =
      block.Has("noise_equivalent_power_w")
          ? static_cast<float>(block["noise_equivalent_power_w"].AsDouble())
          : 1.0e-18f;
  config.hardware.integration_time_sec =
      block.Has("integration_time_sec")
          ? static_cast<float>(block["integration_time_sec"].AsDouble())
          : 1.0f;
  config.mission.work_mode = sbirs_sensor::config::SbirsWorkMode::kSearchAndStare;
  config.mission.scan_start_az_deg = static_cast<float>(block["scan_start_az_deg"].AsDouble());
  // 方位基准（2026-08-31）：kNadirRelative 时 scan_start_az_deg 语义 = 相对星下点
  // 偏移（0 = 正对星下点，免推算）；缺省 kEciAbsolute 维持既有绝对方位语义。
  if (block.Has("scan_azimuth_reference") &&
      block["scan_azimuth_reference"].AsString() == "kNadirRelative") {
    config.mission.scan_azimuth_reference =
        sbirs_sensor::config::SbirsScanAzimuthReference::kNadirRelative;
  }
  config.mission.scan_span_deg =
      block.Has("scan_span_deg") ? static_cast<float>(block["scan_span_deg"].AsDouble()) : 11.0f;
  if (block.Has("initial_scan_phase_deg")) {
    config.mission.initial_scan_phase_deg =
        static_cast<float>(block["initial_scan_phase_deg"].AsDouble());
  }
  config.mission.scan_rate_deg_per_sec =
      block.Has("scan_rate_deg_per_sec")
          ? static_cast<float>(block["scan_rate_deg_per_sec"].AsDouble())
          : 1.0f;
  // 2-D 俯仰栅格（库内阶段 4；缺省 span=0 = 单行模式，行为不变）。
  config.mission.scan_el_start_deg =
      block.Has("scan_el_start_deg")
          ? static_cast<float>(block["scan_el_start_deg"].AsDouble())
          : config.mission.scan_el_start_deg;
  config.mission.scan_el_span_deg =
      block.Has("scan_el_span_deg")
          ? static_cast<float>(block["scan_el_span_deg"].AsDouble())
          : config.mission.scan_el_span_deg;
  config.mission.scan_el_step_deg =
      block.Has("scan_el_step_deg")
          ? static_cast<float>(block["scan_el_step_deg"].AsDouble())
          : config.mission.scan_el_step_deg;
  config.mission.frame_rate_hz =
      block.Has("frame_rate_hz")
          ? static_cast<float>(block["frame_rate_hz"].AsDouble())
          : config.mission.frame_rate_hz;
  // 窄场单帧角噪声 1-σ（高刷新率多帧融合建模：N = frame_rate_hz×dt 帧融合，
  // 随机 1-σ 1/√N；宽场共用同一误差模型，见 md 口径说明）。
  config.policy.error_model.fov_sigma_deg =
      block.Has("fov_sigma_deg")
          ? static_cast<float>(block["fov_sigma_deg"].AsDouble())
          : config.policy.error_model.fov_sigma_deg;
  config.mission.wide_field_fov_az_deg =
      block.Has("wide_field_fov_az_deg")
          ? static_cast<float>(block["wide_field_fov_az_deg"].AsDouble())
          : 20.0f;
  config.mission.wide_field_fov_el_deg =
      block.Has("wide_field_fov_el_deg")
          ? static_cast<float>(block["wide_field_fov_el_deg"].AsDouble())
          : 20.0f;
  config.mission.narrow_field_fov_az_deg =
      block.Has("narrow_field_fov_az_deg")
          ? static_cast<float>(block["narrow_field_fov_az_deg"].AsDouble())
          : 5.0f;
  config.mission.narrow_field_fov_el_deg =
      block.Has("narrow_field_fov_el_deg")
          ? static_cast<float>(block["narrow_field_fov_el_deg"].AsDouble())
          : 5.0f;
  if (block.Has("scan_center_el_deg")) {
    config.mission.scan_center_el_deg = static_cast<float>(block["scan_center_el_deg"].AsDouble());
  }
  config.policy.detection.wide_min_snr_linear =
      block.Has("wide_min_snr_linear")
          ? static_cast<float>(block["wide_min_snr_linear"].AsDouble())
          : 0.001f;
  config.policy.detection.narrow_min_snr_linear =
      block.Has("narrow_min_snr_linear")
          ? static_cast<float>(block["narrow_min_snr_linear"].AsDouble())
          : 0.001f;
  // 安装指向域（第10项验收行数据源）：安装角/失准/限位/稳定方式。缺段/缺键继承
  // 加载器默认集 SbirsOrientationDefaults()（验收演示误差参数，非全零）；写了的
  // 键逐一覆盖（如 B/C 星互异安装角）。库结构体默认仍为全零，单测不受影响。
  LoadSbirsOrientation(block, &config.orientation);
  return config;
}

bool LoadEphemerisSat(const examples::JsonValue& block, oneq::coordinate::EcefPositionM* position,
                      oneq::coordinate::EcefVelocityMps* velocity, std::string* error,
                      const char* tag) {
  if (block.Has("position_lla_deg_m")) {
    oneq::coordinate::LlaPositionDegM lla;
    if (!ReadLla(block["position_lla_deg_m"], &lla,
                 (std::string(tag) + ".position_lla_deg_m").c_str(), error)) {
      return false;
    }
    if (!oneq::coordinate::TryLlaToEcef(lla, position)) {
      *error = std::string(tag) + ".position_lla_deg_m could not be converted to ECEF";
      return false;
    }
  } else {
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    if (!ReadVec3(block["position_ecef_m"], &px, &py, &pz,
                  (std::string(tag) + ".position_ecef_m").c_str(), error)) {
      return false;
    }
    *position = oneq::coordinate::EcefPositionM(px, py, pz);
  }
  if (block.Has("velocity_ecef_mps")) {
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    if (!ReadVec3(block["velocity_ecef_mps"], &vx, &vy, &vz,
                  (std::string(tag) + ".velocity_ecef_mps").c_str(), error)) {
      return false;
    }
    *velocity = oneq::coordinate::EcefVelocityMps(vx, vy, vz);
  }
  return true;
}

bool LoadScene(const char* path, LoadedScene* scene, std::string* error) {
  examples::JsonValue root;
  if (!examples::JsonReader::ParseFile(path, &root, error)) {
    return false;
  }
  if (root.Has("cycles")) {
    const std::int64_t cycles = root["cycles"].AsInt();
    if (cycles <= 0) {
      *error = "cycles must be > 0";
      return false;
    }
    scene->cycles = static_cast<std::uint32_t>(cycles);
  }
  if (root.Has("dt_sec")) {
    scene->dt_sec = static_cast<float>(root["dt_sec"].AsDouble());
  }
  // 场景自带日志输出目录（强制约束）：相对路径（基点 examples/log/），无 ..。
  if (!root["log_dir"].IsString() || root["log_dir"].AsString().empty()) {
    *error = "missing required \"log_dir\"（相对 examples/log/ 的路径）";
    return false;
  }
  scene->log_dir = root["log_dir"].AsString();
  if (scene->log_dir[0] == '/' ||
      scene->log_dir.find("..") != std::string::npos) {
    *error = "invalid \"log_dir\"：须为相对路径且不含 ..";
    return false;
  }
  if (root.Has("utc_julian_day")) {
    scene->utc_julian_day = root["utc_julian_day"].AsDouble();
  }
  if (root.Has("inference_horizon_sec")) {
    scene->config.inference.prediction_horizon_sec = root["inference_horizon_sec"].AsDouble();
  } else {
    scene->config.inference.prediction_horizon_sec = 1200.0;
  }
  if (root.Has("rir") && root["rir"].type() == examples::JsonValue::kObject &&
      root["rir"].Has("enabled")) {
    scene->lrdr_enabled = root["rir"]["enabled"].AsBool();
  }
  if (root.Has("fusion")) {
    const examples::JsonValue& fusion = root["fusion"];
    if (fusion.Has("track_bearing_init_range_m")) {
      scene->config.fusion.track_bearing_init_range_m =
          fusion["track_bearing_init_range_m"].AsDouble();
    }
    if (fusion.Has("track_bearing_init_range_std_m")) {
      scene->config.fusion.track_bearing_init_range_std_m =
          fusion["track_bearing_init_range_std_m"].AsDouble();
    }
    if (fusion.Has("relay_fov_width_deg")) {
      scene->config.fusion.relay_fov_width_deg = fusion["relay_fov_width_deg"].AsDouble();
    }
  }
  // 双星配对窗（扫描模式时间语义）：窗宽 0 = 严格同周期交会（历史行为）；
  // 场景开扫描后按"地面按时刻融合"口径放宽为窗内配对（时间基准=周期号抽帧时刻）。
  if (root.Has("precision")) {
    const examples::JsonValue& precision = root["precision"];
    if (precision.Has("dual_sat_pair_window_cycles")) {
      const std::int64_t window = precision["dual_sat_pair_window_cycles"].AsInt();
      if (window < 0) {
        *error = "precision.dual_sat_pair_window_cycles must be >= 0";
        return false;
      }
      scene->config.dual_sat_pair_window_cycles = static_cast<std::uint32_t>(window);
    }
  }

  const examples::JsonValue& satellites = root["satellites"];
  if (satellites.IsNull() || satellites.Size() < 2U) {
    *error = "satellites must be an array of at least two objects (first two used for PE)";
    return false;
  }
  scene->satellites.clear();
  scene->satellites.reserve(satellites.Size());
  // 顶层外部输入开关统一作用于本场景创建的全部红外卫星会话。
  const bool acceptance_log_enabled =
      !root.Has("acceptance_log_enabled") || root["acceptance_log_enabled"].AsBool();
  for (std::size_t i = 0U; i < satellites.Size(); ++i) {
    const examples::JsonValue& block = satellites[i];
    LoadedSatellite sat;
    sat.id = block.Has("id") ? block["id"].AsString() : ("sat" + std::to_string(i));
    sat.source_id = block.Has("source_id")
                        ? static_cast<std::uint32_t>(block["source_id"].AsInt())
                        : (4U + static_cast<std::uint32_t>(i) * 100U);
    sat.config = LoadSatelliteConfig(block);
    sat.config.acceptance_log_enabled = acceptance_log_enabled;
    if (block.Has("max_concurrent_nfov_locks")) {
      // 已删除的"窄场镜筒数"配置（2026-09-02 单镜筒化）：显式拒绝而非静默忽略——
      // 该键在本场景家族曾以 2 写入，静默跳过会让读者误以为多镜筒行为仍受控。
      *error = "satellites[" + std::to_string(i) +
               "] contains retired key 'max_concurrent_nfov_locks' (single-telescope NFOV "
               "since 2026-09-02); remove it from the scene json";
      return false;
    }
    sat.cross_cue = block.Has("cross_cue") && block["cross_cue"].AsBool();
    if (!LoadEphemerisSat(block, &sat.position_ecef_m, &sat.velocity_ecef_m_per_s, error,
                          ("satellites[" + std::to_string(i) + "]").c_str())) {
      return false;
    }
    if (block.Has("attitude_yaw_deg")) {
      sat.attitude_yaw_deg = block["attitude_yaw_deg"].AsDouble();
    }
    if (block.Has("attitude_pitch_deg")) {
      sat.attitude_pitch_deg = block["attitude_pitch_deg"].AsDouble();
    }
    if (block.Has("attitude_roll_deg")) {
      sat.attitude_roll_deg = block["attitude_roll_deg"].AsDouble();
    }
    scene->satellites.push_back(sat);
  }
  for (std::size_t i = 0U; i < scene->satellites.size(); ++i) {
    for (std::size_t j = i + 1U; j < scene->satellites.size(); ++j) {
      if (scene->satellites[i].source_id == scene->satellites[j].source_id) {
        *error = "satellites[].source_id must be unique";
        return false;
      }
    }
  }
  scene->config.satellite_a = scene->satellites[0].config;
  scene->config.satellite_b = scene->satellites[1].config;
  scene->config.satellite_a_source_id = scene->satellites[0].source_id;
  scene->config.satellite_b_source_id = scene->satellites[1].source_id;
  scene->ephemeris.satellite_a_position_ecef_m = scene->satellites[0].position_ecef_m;
  scene->ephemeris.satellite_a_velocity_ecef_m_per_s = scene->satellites[0].velocity_ecef_m_per_s;
  scene->ephemeris.satellite_a_attitude_yaw_deg = scene->satellites[0].attitude_yaw_deg;
  scene->ephemeris.satellite_a_attitude_pitch_deg = scene->satellites[0].attitude_pitch_deg;
  scene->ephemeris.satellite_a_attitude_roll_deg = scene->satellites[0].attitude_roll_deg;
  scene->ephemeris.satellite_b_position_ecef_m = scene->satellites[1].position_ecef_m;
  scene->ephemeris.satellite_b_velocity_ecef_m_per_s = scene->satellites[1].velocity_ecef_m_per_s;
  scene->ephemeris.satellite_b_attitude_yaw_deg = scene->satellites[1].attitude_yaw_deg;
  scene->ephemeris.satellite_b_attitude_pitch_deg = scene->satellites[1].attitude_pitch_deg;
  scene->ephemeris.satellite_b_attitude_roll_deg = scene->satellites[1].attitude_roll_deg;

  const examples::JsonValue& targets = root["targets"];
  if (targets.IsNull() || targets.Size() == 0U) {
    *error = "targets must be a non-empty array";
    return false;
  }
  scene->truth.clear();
  for (std::size_t i = 0U; i < targets.Size(); ++i) {
    const examples::JsonValue& block = targets[i];
    pe::EvaluationTruthTarget truth;
    truth.key = static_cast<std::uint64_t>(block["key"].AsInt());
    oneq::coordinate::LlaPositionDegM target_lla;
    bool have_lla = false;
    if (block.Has("position_lla_deg_m")) {
      if (!ReadLla(block["position_lla_deg_m"], &target_lla, "targets[].position_lla_deg_m",
                   error)) {
        return false;
      }
      if (!oneq::coordinate::TryLlaToEcef(target_lla, &truth.position_ecef_m)) {
        *error = "targets[].position_lla_deg_m could not be converted to ECEF";
        return false;
      }
      have_lla = true;
    } else {
      double px = 0.0;
      double py = 0.0;
      double pz = 0.0;
      if (!ReadVec3(block["position_ecef_m"], &px, &py, &pz, "targets[].position_ecef_m", error)) {
        return false;
      }
      truth.position_ecef_m = oneq::coordinate::EcefPositionM(px, py, pz);
    }
    if (block.Has("velocity_enu_mps")) {
      double ve = 0.0;
      double vn = 0.0;
      double vu = 0.0;
      if (!ReadVec3(block["velocity_enu_mps"], &ve, &vn, &vu, "targets[].velocity_enu_mps",
                    error)) {
        return false;
      }
      if (!have_lla &&
          !oneq::coordinate::TryEcefToLla(truth.position_ecef_m, &target_lla)) {
        *error = "targets[].velocity_enu_mps needs a convertible position for ENU axes";
        return false;
      }
      const oneq::coordinate::EnuVelocityMps enu_velocity(ve, vn, vu);
      if (!oneq::coordinate::TryEnuToEcefVelocity(enu_velocity, target_lla,
                                                  &truth.velocity_ecef_m_per_s)) {
        *error = "targets[].velocity_enu_mps could not be converted to ECEF";
        return false;
      }
      truth.has_velocity = true;
    } else if (block.Has("velocity_ecef_mps")) {
      double vx = 0.0;
      double vy = 0.0;
      double vz = 0.0;
      if (!ReadVec3(block["velocity_ecef_mps"], &vx, &vy, &vz, "targets[].velocity_ecef_mps",
                    error)) {
        return false;
      }
      truth.has_velocity = true;
      truth.velocity_ecef_m_per_s = oneq::coordinate::EcefVelocityMps(vx, vy, vz);
    }
    if (block.Has("radiant_intensity_w_per_sr")) {
      truth.radiant_intensity_w_per_sr = block["radiant_intensity_w_per_sr"].AsDouble();
    }
    scene->truth.push_back(truth);
  }
  return true;
}


const char* MetricName(pe::PrecisionMetric metric) {
  switch (metric) {
    case pe::PrecisionMetric::kAngular:
      return "angular (deg)";
    case pe::PrecisionMetric::kDualSatFix:
      return "dual_sat_fix (m)";
    case pe::PrecisionMetric::kVelocity:
      return "velocity (m/s)";
    case pe::PrecisionMetric::kImpactPoint:
      return "impact_point (m)";
    case pe::PrecisionMetric::kLaunchPoint:
      return "launch_point (m)";
  }
  return "unknown";
}

// 调试状态枚举 → CSV 字符串（sbirs_los.csv 的 status 列；查看器按词着色：
// detected = 画视线，not_in_output = 目标在场但该星看不到，多数是被地球挡住）。
const char* DebugStatusName(sbirs_sensor::session::SbirsDebugTargetStatus status) {
  switch (status) {
    case sbirs_sensor::session::SbirsDebugTargetStatus::kDetected:
      return "detected";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kObservedBelowThreshold:
      return "below_threshold";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kCoasting:
      return "coasting";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kNotInOutput:
      return "not_in_output";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kCycleNotExecuted:
      return "not_executed";
  }
  return "unknown";
}

void PrintReport(const pe::PrecisionEvaluationReport& report) {
  std::cout << "\n=== Precision Evaluation Report ===\n";
  std::cout << "metric               count          mean          rmse           p95           max\n";
  for (std::size_t i = 0U; i < pe::kPrecisionMetricCount; ++i) {
    const pe::ErrorMetricSummary& m = report.metrics[i];
    std::cout << MetricName(static_cast<pe::PrecisionMetric>(i)) << std::string(
                     22U - std::string(MetricName(static_cast<pe::PrecisionMetric>(i))).size(), ' ')
              << m.count << std::string(9U - std::to_string(m.count).size(), ' ')
              << m.mean << "  " << m.rmse << "  " << m.p95 << "  " << m.max << "\n";
  }
  std::cout << "\nahp: valid=" << (report.ahp_valid ? 1 : 0)
            << " consistent=" << (report.ahp.is_consistent ? 1 : 0)
            << " lambda_max=" << report.ahp.lambda_max
            << " ci=" << report.ahp.consistency_index
            << " cr=" << report.ahp.consistency_ratio << "\n";
  std::cout << "weights:";
  for (std::size_t i = 0U; i < pe::kPrecisionMetricCount; ++i) {
    std::cout << " " << report.ahp.weights[i];
  }
  std::cout << "\nscores (score = ref/(ref+rmse), contribution = w*score):\n";
  for (std::size_t i = 0U; i < pe::kPrecisionMetricCount; ++i) {
    std::cout << "  " << MetricName(static_cast<pe::PrecisionMetric>(i))
              << " ref=" << report.reference_errors[i]
              << " score=" << report.metric_scores[i]
              << " contribution=" << report.metric_contributions[i] << "\n";
  }
  std::cout << "composite score = " << report.composite_score << "\n";
}

std::vector<sbirs_sensor::session::SbirsSceneTarget> MakeSbirsTargets(
    const std::vector<pe::EvaluationTruthTarget>& truth) {
  std::vector<sbirs_sensor::session::SbirsSceneTarget> targets;
  targets.reserve(truth.size());
  for (std::size_t i = 0U; i < truth.size(); ++i) {
    const pe::EvaluationTruthTarget& item = truth[i];
    sbirs_sensor::session::SbirsSceneTarget scene_target;
    scene_target.target_id = item.key;
    scene_target.target_name = "eval_truth";
    scene_target.position_ecef_m = sbirs_sensor::session::SbirsVector3M{
        item.position_ecef_m.x_m, item.position_ecef_m.y_m, item.position_ecef_m.z_m};
    scene_target.radiant_intensity_w_per_sr = item.radiant_intensity_w_per_sr;
    scene_target.active = item.active;
    scene_target.has_velocity_ecef_m_per_s = item.has_velocity;
    scene_target.velocity_ecef_m_per_s = sbirs_sensor::session::SbirsVector3M{
        item.velocity_ecef_m_per_s.x_mps, item.velocity_ecef_m_per_s.y_mps,
        item.velocity_ecef_m_per_s.z_mps};
    targets.push_back(scene_target);
  }
  return targets;
}

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " [--scene <path>] [--cycles <n>] [--output-dir <dir>]\n"
            << "  --scene       JSON 场景（默认 " << ONEQ_SCENE_JSON << "）\n"
            << "  --cycles      覆盖场景 cycles\n"
            << "  --output-dir  验收文件目录（默认 " << PE_DEFAULT_OUTPUT_DIR
            << "/<场景名>/）\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  std::string scene_path = ONEQ_SCENE_JSON;
  std::string output_dir = PE_DEFAULT_OUTPUT_DIR;
  bool output_dir_overridden = false;
  bool cycles_overridden = false;
  std::uint32_t cycles_override = 0U;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--cycles" && i + 1 < argc) {
      cycles_overridden = true;
      cycles_override = static_cast<std::uint32_t>(std::atoi(argv[++i]));
    } else if (arg == "--scene" && i + 1 < argc) {
      scene_path = argv[++i];
    } else if (arg == "--output-dir" && i + 1 < argc) {
      output_dir = argv[++i];
      output_dir_overridden = true;
    } else {
      PrintUsage(argv[0]);
      return 1;
    }
  }

  LoadedScene scene;
  std::string error;
  if (!LoadScene(scene_path.c_str(), &scene, &error)) {
    std::cerr << "Failed to load scene \"" << scene_path << "\": " << error << "\n";
    return 1;
  }
  if (cycles_overridden) {
    scene.cycles = cycles_override;
  }
  if (scene.cycles == 0U) {
    std::cerr << "Invalid cycle count: must be > 0\n";
    return 1;
  }

  component_attachment::app::SceneData lrdr_scene;
  component_attachment::app::SceneSessionConfigs lrdr_configs;
  std::vector<component_attachment::app::TargetEcefState> lrdr_target_states;
  if (scene.lrdr_enabled) {
    std::string lrdr_error;
    if (!component_attachment::app::LoadSceneData(
            scene_path.c_str(), &lrdr_scene, &lrdr_configs, &lrdr_error)) {
      std::cerr << "Failed to load LRDR portion of scene: " << scene_path
                << ": " << lrdr_error << "\n";
      return 1;
    }
    lrdr_target_states = component_attachment::app::MakeTargetStates(
        lrdr_scene.targets, lrdr_scene.platform_origin);
    for (component_attachment::app::TargetEcefState& state : lrdr_target_states) {
      for (const pe::EvaluationTruthTarget& truth : scene.truth) {
        if (truth.key == state.id) {
          state.position = truth.position_ecef_m;
          state.velocity = truth.velocity_ecef_m_per_s;
          break;
        }
      }
    }
  }

  if (!output_dir_overridden) {
    output_dir = std::string(PE_DEFAULT_OUTPUT_DIR) + "/" + scene.log_dir;
  }
  if (component_attachment::app::IsInsideTempArea(output_dir)) {
    std::cerr << "refusing output dir \"" << output_dir
              << "\": 场景产物禁止落到系统临时目录（强制约束；输出位置由场景 "
                 "JSON 的 log_dir 声明，--output-dir 仅允许非临时目录覆盖）\n";
    return 1;
  }
  std::error_code fs_error;
  app_fs::create_directories(output_dir, fs_error);
  if (fs_error) {
    std::cerr << "Failed to create output dir \"" << output_dir << "\": " << fs_error.message()
              << "\n";
    return 1;
  }
  component_attachment::app::InitIntegrationLog(output_dir);

#if !defined(PE_ACCEPTANCE_LOG_ENABLED) || !PE_ACCEPTANCE_LOG_ENABLED
  std::cerr << "warning: ONEQ_ENABLE_PRECISION_EVALUATION_LOG=OFF; "
            << output_dir << "/precision_acceptance.log will not be written\n";
#endif

  scene.config.fusion.enable_track_filtering = true;
  // 评审 2026-08-26 条12：装配层接好落点预报外发事件（on_impact_forecast_published），
  // 推演验收行据此写「分发状态=已发布(事件…)」。
  scene.config.inference.impact_distribution_channel = "on_impact_forecast_published";

  component_attachment::AppSceneState app_scene;
  app_scene.sbirs_utc_julian_day = scene.utc_julian_day;
  component_attachment::World world(app_scene);

  component_attachment::RirSensorComponent* rir_component = nullptr;
  if (scene.lrdr_enabled) {
    component_attachment::Entity& lrdr =
        world.CreateEntity(component_attachment::kRirSiteEntityName);
    lrdr.Attach(std::unique_ptr<component_attachment::RirSensorComponent>(
        new component_attachment::RirSensorComponent(
            remote_identification_radar::session::RirSession::Create(lrdr_configs.rir),
            lrdr_scene.rir_site_origin, lrdr_configs.rir.mission.recognition_dwell_sec,
            lrdr_scene.device_ids.rir,
            component_attachment::DetectionDeliveryMode::kSharedBlackboard)));
    rir_component = lrdr.Find<component_attachment::RirSensorComponent>();
  }

  // 3D 可视化 CSV 落盘：与验收日志同目录，查看器 sbirs_orbit_viewer.py 消费。
  // gmst_rad 为 ECI↔ECEF 旋转角（场景儒略日固定 → 全程不变，随 sats.csv 导出
  // 一次，查看器用它把测角视线从 ECI 极坐标转回 ECEF 画线）。
  double gmst_rad = 0.0;
  if (!oneq::coordinate::TryComputeGmstRad(scene.utc_julian_day, &gmst_rad)) {
    std::cerr << "Failed to compute GMST from utc_julian_day=" << scene.utc_julian_day << "\n";
    return 1;
  }
  examples::CsvWriter sats_csv(
      output_dir + "/sbirs_sats.csv",
      "sat_id,source_id,ecef_x_m,ecef_y_m,ecef_z_m,gmst_rad,scan_center_az_deg,"
      "scan_center_el_deg,wfov_az_deg,wfov_el_deg,nfov_az_deg,nfov_el_deg,"
      "scan_span_deg,scan_rate_deg_per_sec,nadir_az_deg");
  examples::CsvWriter scan_csv(
      output_dir + "/sbirs_scan.csv",
      "cycle,t_sec,source_id,sat_id,scan_azimuth_deg,scan_rel_deg,nadir_az_deg,"
      "scan_span_deg,wfov_az_deg");
  examples::CsvWriter truth_csv(output_dir + "/sbirs_truth.csv",
                                "cycle,t_sec,target_id,ecef_x_m,ecef_y_m,ecef_z_m");
  examples::CsvWriter los_csv(output_dir + "/sbirs_los.csv",
                              "cycle,t_sec,source_id,target_id,present_in_input,status,"
                              "az_rad,el_rad,snr_linear,estimated_range_m");
  examples::CsvWriter fused_csv(output_dir + "/sbirs_fused.csv",
                                "cycle,t_sec,key,lifecycle,confidence,has_position,"
                                "ecef_x_m,ecef_y_m,ecef_z_m,channels");
  examples::CsvWriter dual_fix_csv(output_dir + "/sbirs_dual_fix.csv",
                                   "cycle,t_sec,key,position_error_m,los_residual_m,"
                                   "slant_range_error_m");
  for (std::size_t i = 0U; i < scene.satellites.size(); ++i) {
    const LoadedSatellite& sat = scene.satellites[i];
    // scan_center_az_deg 导出"有效绝对方位"（查看器按它画视场锥）：nadir 基准下 =
    // 星下点 ECI 方位 + 偏移（星下点方位 = atan2(-y,-x)，卫星 ECEF 旋 ECI 同 gmst）；
    // 绝对基准下 = scan_start_az_deg 原值。
    double effective_start_az_deg = sat.config.mission.scan_start_az_deg;
    if (sat.config.mission.scan_azimuth_reference ==
        sbirs_sensor::config::SbirsScanAzimuthReference::kNadirRelative) {
      const double cg = std::cos(gmst_rad), sg = std::sin(gmst_rad);
      const double eci_x = cg * sat.position_ecef_m.x_m - sg * sat.position_ecef_m.y_m;
      const double eci_y = sg * sat.position_ecef_m.x_m + cg * sat.position_ecef_m.y_m;
      const double nadir_az_deg =
          std::atan2(-eci_y, -eci_x) * 180.0 / 3.14159265358979323846;
      effective_start_az_deg = nadir_az_deg +
          static_cast<double>(sat.config.mission.scan_start_az_deg);
    }
    while (effective_start_az_deg < 0.0) effective_start_az_deg += 360.0;
    while (effective_start_az_deg >= 360.0) effective_start_az_deg -= 360.0;
    double nadir_az_deg = effective_start_az_deg -
        static_cast<double>(sat.config.mission.scan_start_az_deg);
    if (sat.config.mission.scan_azimuth_reference !=
        sbirs_sensor::config::SbirsScanAzimuthReference::kNadirRelative) {
      const double cg = std::cos(gmst_rad), sg = std::sin(gmst_rad);
      const double eci_x = cg * sat.position_ecef_m.x_m - sg * sat.position_ecef_m.y_m;
      const double eci_y = sg * sat.position_ecef_m.x_m + cg * sat.position_ecef_m.y_m;
      nadir_az_deg = std::atan2(-eci_y, -eci_x) * 180.0 / 3.14159265358979323846;
    }
    while (nadir_az_deg < 0.0) nadir_az_deg += 360.0;
    while (nadir_az_deg >= 360.0) nadir_az_deg -= 360.0;
    char row[640];
    std::snprintf(row, sizeof(row), "%s,%u,%.3f,%.3f,%.3f,%.9f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f",
                  sat.id.c_str(), sat.source_id, sat.position_ecef_m.x_m,
                  sat.position_ecef_m.y_m, sat.position_ecef_m.z_m, gmst_rad,
                  effective_start_az_deg,
                  static_cast<double>(sat.config.mission.scan_center_el_deg),
                  static_cast<double>(sat.config.mission.wide_field_fov_az_deg),
                  static_cast<double>(sat.config.mission.wide_field_fov_el_deg),
                  static_cast<double>(sat.config.mission.narrow_field_fov_az_deg),
                  static_cast<double>(sat.config.mission.narrow_field_fov_el_deg),
                  static_cast<double>(sat.config.mission.scan_span_deg),
                  static_cast<double>(sat.config.mission.scan_rate_deg_per_sec),
                  nadir_az_deg);
    sats_csv.WriteRow(row);
  }

  std::vector<component_attachment::SbirsSensorComponent*> sbirs_components;
  sbirs_components.reserve(scene.satellites.size());
  for (std::size_t i = 0U; i < scene.satellites.size(); ++i) {
    const LoadedSatellite& sat = scene.satellites[i];
    const sbirs_sensor::session::SbirsVector3M pose(sat.position_ecef_m.x_m, sat.position_ecef_m.y_m,
                                                    sat.position_ecef_m.z_m);
    const sbirs_sensor::session::SbirsVector3M vel(sat.velocity_ecef_m_per_s.x_mps,
                                                   sat.velocity_ecef_m_per_s.y_mps,
                                                   sat.velocity_ecef_m_per_s.z_mps);
    const sbirs_sensor::session::SbirsEulerAnglesDeg att(
        sat.attitude_yaw_deg, sat.attitude_pitch_deg, sat.attitude_roll_deg);
    component_attachment::Entity& entity = world.CreateEntity("satellite_" + sat.id);
    entity.Attach(std::unique_ptr<component_attachment::SbirsSensorComponent>(
        new component_attachment::SbirsSensorComponent(
            sbirs_sensor::session::SbirsSession::Create(sat.config), sat.source_id, pose, vel, att,
            component_attachment::SbirsGroundDeliveryMode::kMessage)));
    sbirs_components.push_back(entity.Find<component_attachment::SbirsSensorComponent>());
    sbirs_components.back()->SetSensorName("红外预警卫星" + sat.id);
    if (sat.cross_cue) {
      sbirs_components.back()->SetCrossCueEnabled(true);  // cross-cue 常开（JSON cross_cue）
    }
  }
  std::vector<std::uint32_t> evaluation_source_ids;
  evaluation_source_ids.push_back(scene.satellites[0].source_id);
  evaluation_source_ids.push_back(scene.satellites[1].source_id);

  // 实体先建：分发来源/对象实体 ID 在引擎构造前注入推演配置（验收判定标准
  // 第25项·修改：已发布行须写实体 ID 对，不用通道名/角色名代替）。
  component_attachment::Entity& ground_station = world.CreateEntity("ground_station");
  component_attachment::Entity& impact_receiver = world.CreateEntity("impact_receiver");
  scene.config.inference.impact_distribution_source_id = ground_station.id();
  scene.config.inference.impact_distribution_target_id = impact_receiver.id();

  std::unique_ptr<fusion::FusionEngine> fusion_engine(
      new fusion::FusionEngine(scene.config.fusion));
  std::unique_ptr<pe::PrecisionEvaluationSession> precision_session(
      new pe::PrecisionEvaluationSession(scene.config));

  ground_station.Attach(
      std::unique_ptr<component_attachment::GroundStationFusionComponent>(
          new component_attachment::GroundStationFusionComponent(std::move(fusion_engine),
                                                                 std::move(precision_session),
                                                                 evaluation_source_ids)));
  component_attachment::GroundStationFusionComponent* fusion =
      ground_station.Find<component_attachment::GroundStationFusionComponent>();
  fusion->SetPairWindowCycles(scene.config.dual_sat_pair_window_cycles);

  // 评审 2026-08-26 条12：订阅落点预报外发事件，收件回执写 integration_events.log
  // （验证分发链路真实可达，非仅日志口径；接收方为 impact_receiver 实体）。
  component_attachment::app::ImpactForecastReceiptListener impact_receipt(
      world, static_cast<std::uint64_t>(impact_receiver.id()));
  (void)impact_receipt;

  std::cout << "sbirs_triple_sat_fix_messages: " << scene_path << ", " << scene.cycles
            << " cycles x " << scene.dt_sec << " s, satellites=" << scene.satellites.size()
            << " GEO (ground-station message delivery)\n"
            << "acceptance logs -> " << output_dir
            << " (SatelliteTarget.log + precision/fusion/inference_acceptance.log)\n";
  for (std::size_t i = 0U; i < scene.satellites.size(); ++i) {
    const LoadedSatellite& sat = scene.satellites[i];
    std::cout << "  sat " << sat.id << " source_id=" << sat.source_id << " ecef=("
              << sat.position_ecef_m.x_m << "," << sat.position_ecef_m.y_m << ","
              << sat.position_ecef_m.z_m << ")\n";
  }
  std::uint32_t dual_sat_cycles = 0U;
  PendingRirCue pending_rir_cue;
  std::vector<std::uint64_t> cued_target_ids;
  for (std::uint32_t cycle = 1U; cycle <= scene.cycles; ++cycle) {
    // 真值弹道由调用方推进（评估会话不拥有真值；同单测约定 p += v·dt）。
    for (std::size_t i = 0U; i < scene.truth.size(); ++i) {
      pe::EvaluationTruthTarget& target = scene.truth[i];
      target.position_ecef_m.x_m +=
          static_cast<double>(scene.dt_sec) * target.velocity_ecef_m_per_s.x_mps;
      target.position_ecef_m.y_m +=
          static_cast<double>(scene.dt_sec) * target.velocity_ecef_m_per_s.y_mps;
      target.position_ecef_m.z_m +=
          static_cast<double>(scene.dt_sec) * target.velocity_ecef_m_per_s.z_mps;
    }
    app_scene.cycle = cycle;
    app_scene.t_sec = static_cast<double>(cycle) * static_cast<double>(scene.dt_sec);
    app_scene.sbirs_targets = MakeSbirsTargets(scene.truth);
    if (scene.lrdr_enabled) {
      for (component_attachment::app::TargetEcefState& state : lrdr_target_states) {
        for (const pe::EvaluationTruthTarget& truth : scene.truth) {
          if (truth.key == state.id) {
            state.position = truth.position_ecef_m;
            state.velocity = truth.velocity_ecef_m_per_s;
            break;
          }
        }
      }
      app_scene.rir_targets = component_attachment::app::MakeRirSceneTargets(
          lrdr_target_states, lrdr_scene.rir_site_origin);
      app_scene.emitters = component_attachment::app::MakeEmitterTruths(
          lrdr_target_states, lrdr_scene.esr, app_scene.t_sec);
      component_attachment::BeginRfWorldCycle(&app_scene, scene.dt_sec);
    }
    fusion->BeginCycle(world, cycle);
    fusion->SetEvaluationInputs(scene.ephemeris, scene.truth);
    component_attachment::app::BeginViewLogCycle(cycle);
    world.Step(static_cast<double>(scene.dt_sec));
    if (rir_component != nullptr && pending_rir_cue.active) {
      const remote_identification_radar::session::RirOutputDebugView& rir_view =
          rir_component->LastDebugView();
      for (const remote_identification_radar::session::RirDebugTargetState& radar_target :
           rir_view.targets) {
        if (radar_target.external_target_id != pending_rir_cue.target_id ||
            !radar_target.has_track || !radar_target.has_cycle_detection_snr ||
            app_scene.t_sec <= pending_rir_cue.bearing_time_s) {
          continue;
        }
        rir_component->WriteSbirsCueDetectionAcceptance(
            cycle, app_scene.t_sec, pending_rir_cue.source_name,
            pending_rir_cue.target_id, pending_rir_cue.bearing_time_s,
            pending_rir_cue.cue_az_deg, pending_rir_cue.cue_el_deg,
            radar_target.slant_range_m, radar_target.look_az_deg,
            radar_target.position_enu_z_m);
        pending_rir_cue.active = false;
        break;
      }
    }
    if (rir_component != nullptr && !pending_rir_cue.active) {
      const std::vector<component_attachment::SbirsFrameSubmittedEvent>& frames =
          fusion->last_sbirs_frames();
      bool cue_applied = false;
      for (const component_attachment::SbirsFrameSubmittedEvent& frame : frames) {
        for (const component_attachment::SbirsBearingSample& detection : frame.detections) {
          if (detection.target_id == 0U ||
              std::find(cued_target_ids.begin(), cued_target_ids.end(),
                        detection.target_id) != cued_target_ids.end()) {
            continue;
          }
          remote_identification_radar::config::RirRuntimeConfigPatch patch;
          patch.has_designated_target_id = true;
          patch.designated_external_target_id = detection.target_id;
          patch.has_designation_duration_cycles = true;
          patch.designation_duration_cycles = 20U;
          if (!rir_component->TryApplyRuntimeConfig(patch)) {
            continue;
          }
          pending_rir_cue.active = true;
          pending_rir_cue.source_name = frame.source_name;
          pending_rir_cue.target_id = detection.target_id;
          pending_rir_cue.bearing_time_s = detection.measurement_time_s;
          pending_rir_cue.cue_az_deg =
              static_cast<double>(detection.azimuth_rad) * 180.0 /
              3.14159265358979323846;
          pending_rir_cue.cue_el_deg =
              static_cast<double>(detection.elevation_rad) * 180.0 /
              3.14159265358979323846;
          cued_target_ids.push_back(detection.target_id);
          cue_applied = true;
          break;
        }
        if (cue_applied) {
          break;
        }
      }
    }
    const pe::PrecisionEvaluationCycleResult& result = fusion->last_evaluation();
    if (!result.dual_sat.empty()) {
      ++dual_sat_cycles;
    }
    std::cout << "cycle=" << cycle << " inbox_sats=" << fusion->last_sbirs_frame_count()
              << " fused=" << fusion->targets().size() << " angular=" << result.angular.size()
              << " dual_sat=" << result.dual_sat.size()
              << " velocity=" << result.velocity.size()
              << " keypoints=" << result.keypoints.size();
    if (!fusion->targets().empty()) {
      std::cout << " channels=";
      for (std::size_t t = 0U; t < fusion->targets().size(); ++t) {
        const fusion::FusedTarget& target = fusion->targets()[t];
        if (t > 0U) {
          std::cout << " | ";
        }
        std::cout << "k" << target.key << "[";
        for (std::size_t c = 0U; c < target.channels.size(); ++c) {
          if (c > 0U) {
            std::cout << ",";
          }
          std::cout << target.channels[c].source_id << ":" << target.channels[c].sample_count;
        }
        std::cout << "]";
      }
    }
    std::cout << "\n";

    // 3D 可视化落盘（本周期四表）：真值轨迹（推进后的当前位置）、每星逐目标
    // 视线状态（取各卫星组件的调试视图快照）、融合航迹（LLA 后验转 ECEF 统一
    // 口径）、双星交会误差样本。
    for (std::size_t i = 0U; i < scene.truth.size(); ++i) {
      const pe::EvaluationTruthTarget& target = scene.truth[i];
      char row[256];
      std::snprintf(row, sizeof(row), "%u,%.3f,%llu,%.3f,%.3f,%.3f", cycle, app_scene.t_sec,
                    static_cast<unsigned long long>(target.key), target.position_ecef_m.x_m,
                    target.position_ecef_m.y_m, target.position_ecef_m.z_m);
      truth_csv.WriteRow(row);
    }
    for (std::size_t s = 0U; s < sbirs_components.size(); ++s) {
      const LoadedSatellite& sat = scene.satellites[s];
      const double scan_az_deg =
          static_cast<double>(sbirs_components[s]->scan_azimuth_deg());
      const double cg = std::cos(gmst_rad), sg = std::sin(gmst_rad);
      const double eci_x = cg * sat.position_ecef_m.x_m - sg * sat.position_ecef_m.y_m;
      const double eci_y = sg * sat.position_ecef_m.x_m + cg * sat.position_ecef_m.y_m;
      double nadir_az_deg =
          std::atan2(-eci_y, -eci_x) * 180.0 / 3.14159265358979323846;
      while (nadir_az_deg < 0.0) nadir_az_deg += 360.0;
      while (nadir_az_deg >= 360.0) nadir_az_deg -= 360.0;
      double scan_rel_deg = scan_az_deg - nadir_az_deg;
      while (scan_rel_deg < 0.0) scan_rel_deg += 360.0;
      while (scan_rel_deg >= 360.0) scan_rel_deg -= 360.0;
      char scan_row[320];
      std::snprintf(scan_row, sizeof(scan_row),
                    "%u,%.3f,%u,%s,%.4f,%.4f,%.2f,%.1f,%.1f", cycle, app_scene.t_sec,
                    sat.source_id, sat.id.c_str(), scan_az_deg, scan_rel_deg, nadir_az_deg,
                    static_cast<double>(sat.config.mission.scan_span_deg),
                    static_cast<double>(sat.config.mission.wide_field_fov_az_deg));
      scan_csv.WriteRow(scan_row);
      const sbirs_sensor::session::SbirsOutputDebugView& view =
          sbirs_components[s]->LastDebugView();
      for (std::size_t t = 0U; t < view.targets.size(); ++t) {
        const sbirs_sensor::session::SbirsDebugTargetState& target = view.targets[t];
        char row[320];
        std::snprintf(row, sizeof(row), "%u,%.3f,%u,%llu,%d,%s,%.9f,%.9f,%.6g,%.3f", cycle,
                      app_scene.t_sec, scene.satellites[s].source_id,
                      static_cast<unsigned long long>(target.target_id),
                      target.present_in_input ? 1 : 0, DebugStatusName(target.status),
                      static_cast<double>(target.azimuth_rad),
                      static_cast<double>(target.elevation_rad),
                      static_cast<double>(target.infrared_snr_linear),
                      static_cast<double>(target.estimated_range_m));
        los_csv.WriteRow(row);
      }
    }
    for (std::size_t t = 0U; t < fusion->targets().size(); ++t) {
      const fusion::FusedTarget& target = fusion->targets()[t];
      oneq::coordinate::EcefPositionM fused_ecef;
      const bool has_position =
          target.has_kinematic_estimate &&
          oneq::coordinate::TryLlaToEcef(target.kinematic_estimate.position, &fused_ecef);
      std::string channels;
      for (std::size_t c = 0U; c < target.channels.size(); ++c) {
        if (c > 0U) {
          channels += "|";
        }
        channels += std::to_string(target.channels[c].source_id) + ":" +
                    std::to_string(target.channels[c].sample_count);
      }
      char row[384];
      std::snprintf(row, sizeof(row), "%u,%.3f,%llu,%s,%.6f,%d,%.3f,%.3f,%.3f,%s", cycle,
                    app_scene.t_sec, static_cast<unsigned long long>(target.key),
                    target.lifecycle == fusion::FusedTrackLifecycle::kConfirmed
                        ? "confirmed"
                        : (target.lifecycle == fusion::FusedTrackLifecycle::kCoasting
                               ? "coasting"
                               : "tentative"),
                    target.confidence, has_position ? 1 : 0, fused_ecef.x_m, fused_ecef.y_m,
                    fused_ecef.z_m, channels.c_str());
      fused_csv.WriteRow(row);
    }
    for (std::size_t i = 0U; i < result.dual_sat.size(); ++i) {
      const pe::DualSatFixSample& sample = result.dual_sat[i];
      char row[256];
      std::snprintf(row, sizeof(row), "%u,%.3f,%llu,%.3f,%.3f,%.3f", cycle, app_scene.t_sec,
                    static_cast<unsigned long long>(sample.key), sample.position_error_m,
                    sample.los_residual_m, sample.slant_range_error_m);
      dual_fix_csv.WriteRow(row);
    }
  }

  const pe::PrecisionEvaluationReport report = fusion->SummarizeEvaluation();
  PrintReport(report);
  std::cout << "dual_sat_cycles=" << dual_sat_cycles << "/" << scene.cycles << "\n";
  component_attachment::app::FlushIntegrationLog();
  // 3D 可视化 CSV 刷盘在析构前显式完成（CsvWriter 析构亦会关闭，双保险）。
  sats_csv.Flush();
  scan_csv.Flush();
  truth_csv.Flush();
  los_csv.Flush();
  fused_csv.Flush();
  dual_fix_csv.Flush();
  std::cout << "3D viewer CSV -> " << output_dir
            << " (sbirs_sats/scan/truth/los/fused/dual_fix.csv; viewer: "
               "examples/common/viz/sbirs_orbit_viewer.py)\n";

  // 自检：五指标均有样本、AHP 矩阵合法求解、综合分 ∈ (0,1]，且周期内有双星交会。
  if (!report.all_metrics_sampled || !report.ahp_valid || !(report.composite_score > 0.0) ||
      report.composite_score > 1.0 || dual_sat_cycles == 0U) {
    std::cerr << "SMOKE FAILED: all_metrics_sampled=" << (report.all_metrics_sampled ? 1 : 0)
              << " ahp_valid=" << (report.ahp_valid ? 1 : 0)
              << " composite=" << report.composite_score
              << " dual_sat_cycles=" << dual_sat_cycles << "\n";
    return 1;
  }
  return 0;
}
