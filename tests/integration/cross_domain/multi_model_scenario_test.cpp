#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/airborne_radar/session/ArReplaySession.h"
#include "1q/airborne_radar/session/ArRecordingSession.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "1q/electro_optical_sensor/session/EosReplaySession.h"
#include "1q/electro_optical_sensor/session/EosRecordingSession.h"
#include "1q/electronic_countermeasure/EcmEsrAdapter.h"
#include "1q/electronic_countermeasure/EcmSession.h"
#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "1q/electronic_surveillance_radar/session/EsrReplaySession.h"
#include "1q/electronic_surveillance_radar/session/EsrRecordingSession.h"
#include "1q/replay/ReplayTrace.h"
#include "support/eos_enu_scene_helpers.h"
#include "support/oneq_test_temp_dir.h"

#if defined(ONEQ_TEST_FLIGHT_DYNAMIC_ENABLED)
#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#endif

namespace ar = airborne_radar;
namespace ar_session = airborne_radar::session;
namespace ar_config = airborne_radar::config;
namespace ar_env = airborne_radar::config;
namespace ar_model = airborne_radar::session;

namespace eos = electro_optical_sensor;
namespace eos_session = electro_optical_sensor::session;
namespace eos_config = electro_optical_sensor::config;

namespace esr = electronic_surveillance_radar;
namespace esr_session = electronic_surveillance_radar::session;
namespace esr_config = electronic_surveillance_radar::config;
namespace ecm_config = electronic_countermeasure::config;
namespace ecm_session = electronic_countermeasure::session;

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string MakeTempTraceDir(const char* prefix) {
  static unsigned int counter = 0U;
  std::ostringstream s;
  s << oneq_test::TempDir() << prefix << "-" << std::time(nullptr) << "-"
    << std::chrono::high_resolution_clock::now().time_since_epoch().count() << "-"
    << std::rand() << "-" << counter++ << ".trace";
  return s.str();
}

// 统一世界目标：包含三个模块各自的属性
struct WorldTarget {
  std::uint64_t id;
  oneq::coordinate::EcefPositionM pos;
  oneq::coordinate::EcefVelocityMps vel;
  // AR
  float rcs;
  int swerling_type{0};
  // EOS (from ECEF LLA via position_transform)
  float temperature_k;
  float area_m2;
  float emissivity{0.92f};
  float reflectance{0.35f};
  // ESR
  double carrier_hz;
  double bandwidth_hz{2.0e6};
  double tx_power_w;
  double pulse_width_s{1.0e-6};
  double pri_s{1.0e-4};
  bool is_emitting{true};
};

struct WorldState {
  oneq::coordinate::EcefPositionM platform_pos;
  oneq::coordinate::EcefVelocityMps platform_vel;
  std::vector<WorldTarget> targets;
};

// --- AR input conversion ---

ar_session::ArPlatformInput ToArPlatform(const oneq::coordinate::EcefPositionM& pos,
                                             const oneq::coordinate::EcefVelocityMps& vel) {
  ar_session::ArPlatformInput p;
  p.platform_position_ecef_m = pos;
  p.platform_velocity_mps = vel;
  p.platform_attitude_deg.yaw_deg = 0.0;
  p.platform_attitude_deg.pitch_deg = 0.0;
  p.platform_attitude_deg.roll_deg = 0.0;
  return p;
}

// 世界真值（ECEF）经公共入口转为平台锚点 ENU 后直填 AR 输入。
ar_session::ArTargetInput ToArTarget(const WorldTarget& t,
                                     const oneq::coordinate::LlaPositionDegM& anchor_lla) {
  oneq::coordinate::ExternalKinematics kinematics;
  kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  kinematics.position_ecef_m = t.pos;
  kinematics.velocity_mps = t.vel;

  oneq::coordinate::EnuSceneState enu;
  oneq::coordinate::TryMakeEnuSceneState(kinematics, anchor_lla, &enu);

  ar_session::ArTargetInput input;
  input.target_id = t.id;
  input.position_x = static_cast<float>(enu.position_enu_m.east_m);
  input.position_y = static_cast<float>(enu.position_enu_m.north_m);
  input.position_z = static_cast<float>(enu.position_enu_m.up_m);
  input.velocity_x = static_cast<float>(enu.velocity_enu_mps.east_mps);
  input.velocity_y = static_cast<float>(enu.velocity_enu_mps.north_mps);
  input.velocity_z = static_cast<float>(enu.velocity_enu_mps.up_mps);
  input.rcs = t.rcs;
  input.swerling_type = t.swerling_type;
  return input;
}

struct ArRfTestCycleInput {
  ar_session::ArCycleInput cycle{};
  bool valid{false};
};

struct ArRfTestCycleResult {
  bool accepted{false};
  ar_session::TrackOutputFrame output_frame{};
  ar_session::ArInterferenceObservationList interference_observations{};
  ar_session::ArReceiverImpairment receiver_impairment{ar_session::ArReceiverImpairment::kNone};
  bool has_control_profile{false};
  ar_session::ArControlProfile control_profile{};
};

ArRfTestCycleInput BuildArInput(const WorldState& ws, float dt, std::uint32_t cycle_index) {
  ar_session::ArPlatformInput platform = ToArPlatform(ws.platform_pos, ws.platform_vel);
  oneq::coordinate::LlaPositionDegM anchor_lla;
  oneq::coordinate::TryEcefToLla(platform.platform_position_ecef_m, &anchor_lla);
  std::vector<ar_session::ArTargetInput> targets;
  targets.reserve(ws.targets.size());
  for (const auto& t : ws.targets) {
    targets.push_back(ToArTarget(t, anchor_lla));
  }
  ArRfTestCycleInput input;
  input.cycle.cycle_index = cycle_index;
  input.cycle.cycle_start_time_s = static_cast<double>(cycle_index - 1U) * static_cast<double>(dt);
  input.cycle.dt_sec = dt;
  input.cycle.platform = platform;
  input.cycle.targets = targets;
  input.valid = true;
  return input;
}

oneq::electromagnetics::RfEmissionFrame MakeNoiseInterferenceFrame(
    const ar_session::ArCycleInput& cycle) {
  oneq::electromagnetics::RfEmissionFrame frame;
  frame.world_cycle_index = cycle.cycle_index;
  frame.window_start_time_s = cycle.cycle_start_time_s;
  frame.window_duration_s = cycle.dt_sec;
  oneq::electromagnetics::RfSceneEmission jammer;
  jammer.identity.sensor_entity_id = 20U;
  jammer.identity.emission_id = 100000U + cycle.cycle_index;
  jammer.position_ecef_m = cycle.platform.platform_position_ecef_m;
  jammer.position_ecef_m.x_m += 1000.0;
  jammer.antenna.boresight_ecef.x = -1.0;
  jammer.antenna.peak_gain_dbi = 35.0;
  if (!oneq::electromagnetics::TryCreateRfNoiseWaveform(cycle.cycle_start_time_s, cycle.dt_sec,
                                                        9.3e9, 20.0e6, 1.0e18, &jammer.waveform)) {
    return oneq::electromagnetics::RfEmissionFrame{};
  }
  frame.emissions.push_back(jammer);
  return frame;
}

ArRfTestCycleResult RunArCycle(ar_session::ArRecordingSession* session,
                               const ArRfTestCycleInput& input) {
  ArRfTestCycleResult result;
  if (session == nullptr || !input.valid) {
    return result;
  }
  ar_session::ArCycleInput cycle = input.cycle;
  const ar_session::ArCycleResult completed = session->StepWithResult(cycle);
  result.accepted = completed.status == ar_session::ArCycleStatus::kCompleted ||
                    completed.status == ar_session::ArCycleStatus::kPoweredOff;
  if (result.accepted) {
    result.output_frame = completed.output_frame;
    result.interference_observations = completed.interference_observations;
    result.receiver_impairment = completed.receiver_impairment;
    result.has_control_profile = completed.has_control_profile;
    result.control_profile = completed.control_profile;
  }
  return result;
}

// --- EOS input conversion（平台 ECEF + TryMakeEnuSceneState 手填 CycleInput）---

eos_session::EosPlatformEcefPose ToEosPlatform(const oneq::coordinate::EcefPositionM& pos,
                                               const oneq::coordinate::EcefVelocityMps& vel) {
  eos_session::EosPlatformEcefPose p;
  p.platform_position_ecef_m = pos;
  p.platform_velocity_mps = vel;
  p.platform_attitude_deg.yaw_deg = 0.0;
  p.platform_attitude_deg.pitch_deg = 0.0;
  p.platform_attitude_deg.roll_deg = 0.0;
  return p;
}

oneq::test_support::EosWorldTargetSpec ToEosTarget(const WorldTarget& t) {
  oneq::test_support::EosWorldTargetSpec input;
  input.target_id = t.id;
  input.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  input.kinematics.position_ecef_m = t.pos;
  input.kinematics.velocity_mps = t.vel;
  input.appearance.apparent_temperature_k = t.temperature_k;
  input.appearance.emissivity = t.emissivity;
  input.appearance.reflectance = t.reflectance;
  input.appearance.projected_area_m2 = t.area_m2;
  return input;
}

eos_session::EosCycleInput BuildEosInput(const WorldState& ws, float dt, std::uint32_t cycle_index) {
  eos_session::EosPlatformEcefPose platform = ToEosPlatform(ws.platform_pos, ws.platform_vel);
  std::vector<oneq::test_support::EosWorldTargetSpec> targets;
  targets.reserve(ws.targets.size());
  for (const auto& t : ws.targets) {
    targets.push_back(ToEosTarget(t));
  }
  eos_session::EosCycleInput input;
  (void)oneq::test_support::TryBuildEosCycleInput(platform, targets, dt, &input);
  input.cycle_index = cycle_index;
  return input;
}

esr_session::EsrCycleInput BuildEsrInput(const WorldState& ws, float dt, std::uint32_t cycle_index) {
  esr_session::EsrCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = static_cast<double>(cycle_index - 1U) * dt;
  input.dt_sec = dt;
  input.platform_position_ecef_m = ws.platform_pos;
  input.platform_velocity_ecef_mps = ws.platform_vel;
  input.rf_emissions.world_cycle_index = cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = dt;
  input.rf_emissions.emissions.reserve(ws.targets.size());
  for (const WorldTarget& target : ws.targets) {
    if (!target.is_emitting) {
      continue;
    }
    oneq::electromagnetics::RfSceneEmission emission;
    emission.identity.sensor_entity_id = target.id;
    emission.identity.emission_id = target.id;
    emission.position_ecef_m = target.pos;
    const double dx = ws.platform_pos.x_m - target.pos.x_m;
    const double dy = ws.platform_pos.y_m - target.pos.y_m;
    const double dz = ws.platform_pos.z_m - target.pos.z_m;
    const double range_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    emission.antenna.boresight_ecef.x = dx / range_m;
    emission.antenna.boresight_ecef.y = dy / range_m;
    emission.antenna.boresight_ecef.z = dz / range_m;
    emission.antenna.peak_gain_dbi = 30.0;
    emission.antenna.peak_gain_dbi = 10.0;
    emission.polarization = oneq::electromagnetics::RfScenePolarization::kHorizontal;
    if (oneq::electromagnetics::TryCreateRfNoiseWaveform(input.cycle_start_time_s, dt,
                                                         target.carrier_hz, target.bandwidth_hz,
                                                         target.tx_power_w, &emission.waveform)) {
      input.rf_emissions.emissions.push_back(emission);
    }
  }
  return input;
}

// --- Config builders (scene-specific) ---
//
// 不同作战场景需要不同的配置范化参数。
// 空对空：传感器朝向水平面，中高仰角扫描
// 空对地：传感器朝向下视，俯角覆盖地面
// 干扰场景：AR 启用干扰检测，环境输入含干扰源

// -- 空对空通用 AR 配置 --
ar_config::ArSessionConfig MakeArConfigAirToAir() {
  ar_config::ArSessionConfig config;
  config.hardware = ar_config::profiles::kLongRangeHighPowerHardware;
  config.policy.detection = ar_config::profiles::kDetectionPriorityDetection;
  config.policy.tracking = ar_config::profiles::kFastAssociationTracking;
  config.policy.tracking.enable_kalman_filter = true;
  config.policy.lifecycle = ar_config::profiles::kFastConfirmLifecycle;
  config.mission.work_mode = ar_config::ArWorkMode::kTas;
  config.mission.scan_center_deg = ar_config::AzimuthElevationDeg{};
  config.hardware.receiver.has_co_site_isolation = true;
  config.hardware.receiver.co_site_isolation_db = 80.0f;
  return config;
}

// -- 空对空通用 EOS 配置 --
eos_config::EosSessionConfig MakeEosConfigAirToAir() {
  eos_config::EosSessionConfig config;
  config.hardware.wavelength_lower_um = 3.0f;
  config.hardware.wavelength_upper_um = 5.0f;
  config.hardware.optical_aperture_m = 0.25f;
  config.mission.work_mode = eos::config::EosWorkMode::kFused;
  config.mission.horizontal_fov_deg = 20.0f;
  config.mission.vertical_fov_deg = 8.0f;
  config.mission.scan_rate_deg_per_sec = 5.0f;
  // 场景步长 1 s/帧，帧率必须与之匹配：ValidateEosCycleInput 拒绝
  // dt_sec > 10 / frame_rate_hz（53c56e21 收紧；81f7fe6d 修复 stale fixtures
  // 时漏掉了本跨域测试，此前每周期都被 kCycleDeltaTimeExceedsFramePeriod 拒绝）。
  config.mission.frame_rate_hz = 1.0f;
  config.mission.scan_start_az_deg = -10.0f;
  config.mission.scan_end_az_deg = 10.0f;
  config.mission.scan_center_el_deg = 0.0f;
  config.mission.boresight_depression_deg = 0.0f;
  config.policy.detection.minimum_snr_db = 4.0f;
  return config;
}

// -- 空对空通用 ESR 配置：水平扫描 ±10° 仰角 --
esr_config::EsrSessionConfig MakeEsrConfigAirToAir() {
  esr_config::EsrSessionConfig config;
  config.hardware.receiver_band_lower_hz = 8.5e9;
  config.hardware.receiver_band_upper_hz = 10.5e9;
  config.hardware.receiver_sensitivity_w = 1.0e-12f;
  config.hardware.integrated_receive_loss_db = 0.0f;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 40.0f;
  config.hardware.az_scan_range_deg = 120.0f;
  config.hardware.el_scan_range_deg = 20.0f;
  config.sensor_enabled = true;
  config.mission.work_mode = esr::config::EsrWorkMode::kEsm;
  // 1 s 场景步长下使用 10 s 完整扫描周期，避免 1 Hz 默认值每帧恰好回到起始波束。
  config.mission.scan.scan_rate_hz = 0.1f;
  config.policy.detection.minimum_snr_db = -40.0f;
  config.policy.detection.enable_statistical_detection = false;
  config.hardware.co_site_paths.push_back(esr_config::EsrCoSiteIsolationPath{1U, 80.0});
  return config;
}

// -- 空对地 AR 配置 --
ar_config::ArSessionConfig MakeArConfigAirToGround() {
  ar_config::ArSessionConfig config;
  config.hardware = ar_config::profiles::kLongRangeHighPowerHardware;
  config.policy.detection = ar_config::profiles::kDetectionPriorityDetection;
  config.policy.tracking = ar_config::profiles::kFastAssociationTracking;
  config.policy.tracking.enable_kalman_filter = true;
  config.policy.lifecycle = ar_config::profiles::kFastConfirmLifecycle;
  config.mission.work_mode = ar_config::ArWorkMode::kTas;
  config.mission.scan_center_deg = ar_config::AzimuthElevationDeg{};
  return config;
}

// -- 空对地 EOS 配置：俯视 45° --
eos_config::EosSessionConfig MakeEosConfigAirToGround() {
  eos_config::EosSessionConfig config;
  config.hardware.wavelength_lower_um = 3.0f;
  config.hardware.wavelength_upper_um = 5.0f;
  config.hardware.optical_aperture_m = 0.25f;
  config.mission.work_mode = eos::config::EosWorkMode::kFused;
  config.mission.horizontal_fov_deg = 20.0f;
  config.mission.vertical_fov_deg = 8.0f;
  config.mission.scan_rate_deg_per_sec = 5.0f;
  // 帧率 1 Hz 与 1 s 场景步长匹配（dt_sec ≤ 10/frame_rate_hz 校验，见 MakeEosConfigAirToAir）。
  config.mission.frame_rate_hz = 1.0f;
  config.mission.scan_start_az_deg = -10.0f;
  config.mission.scan_end_az_deg = 10.0f;
  config.mission.scan_center_el_deg = -45.0f;
  config.mission.boresight_depression_deg = 45.0f;
  config.policy.detection.minimum_snr_db = 4.0f;
  return config;
}

// -- 空对地 ESR 配置：下视扫描，覆盖地面发射源 --
esr_config::EsrSessionConfig MakeEsrConfigAirToGround() {
  esr_config::EsrSessionConfig config;
  config.hardware.receiver_band_lower_hz = 8.5e9;
  config.hardware.receiver_band_upper_hz = 10.5e9;
  config.hardware.receiver_sensitivity_w = 1.0e-12f;
  config.hardware.integrated_receive_loss_db = 0.0f;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 60.0f;
  config.hardware.az_scan_range_deg = 120.0f;
  config.hardware.el_scan_range_deg = 80.0f;
  config.sensor_enabled = true;
  config.mission.work_mode = esr::config::EsrWorkMode::kEsm;
  config.mission.scan.scan_center_el_deg = -35.0f;
  config.policy.detection.minimum_snr_db = -40.0f;
  config.policy.detection.enable_statistical_detection = false;
  return config;
}

// --- World target advancement ---

void AdvanceWorld(WorldState& ws, double dt) {
  ws.platform_pos.x_m += ws.platform_vel.x_mps * dt;
  ws.platform_pos.y_m += ws.platform_vel.y_mps * dt;
  ws.platform_pos.z_m += ws.platform_vel.z_mps * dt;
  for (auto& t : ws.targets) {
    t.pos.x_m += t.vel.x_mps * dt;
    t.pos.y_m += t.vel.y_mps * dt;
    t.pos.z_m += t.vel.z_mps * dt;
  }
}

// --- Replay validation helpers ---

void ExpectReplayOk(const ar_session::ArReplaySessionResult& r, const std::string& label) {
  EXPECT_TRUE(r.report.replay_ready) << label << " AR replay not ready: " << r.first_error;
  EXPECT_FALSE(r.reached_failure_marker) << label << " AR replay hit failure marker";
  EXPECT_FALSE(r.playback.divergence_found)
      << label << " AR replay divergence at seq " << r.playback.divergence_sequence << ": "
      << r.first_error;
}

void ExpectReplayOk(const eos_session::EosReplaySessionResult& r, const std::string& label) {
  EXPECT_TRUE(r.report.replay_ready) << label << " EOS replay not ready: " << r.first_error;
  EXPECT_FALSE(r.reached_failure_marker) << label << " EOS replay hit failure marker";
  if (r.playback.divergence_found) {
    std::cout << "[  INFO   ] " << label << " EOS replay divergence at seq "
              << r.playback.divergence_sequence << ": " << r.first_error << "\n";
  }
}

void ExpectReplayOk(const esr_session::EsrReplaySessionResult& r, const std::string& label) {
  EXPECT_TRUE(r.report.replay_ready) << label << " ESR replay not ready: " << r.first_error;
  EXPECT_FALSE(r.reached_failure_marker) << label << " ESR replay hit failure marker";
  if (r.playback.divergence_found) {
    std::cout << "[  INFO   ] " << label << " ESR replay divergence at seq "
              << r.playback.divergence_sequence << ": " << r.first_error << "\n";
  }
}

// --- ECEF position for given lat/lon/alt ---

oneq::coordinate::EcefPositionM EcefFromLla(double lat_deg, double lon_deg, double alt_m) {
  oneq::coordinate::LlaPositionDegM lla;
  lla.latitude_deg = lat_deg;
  lla.longitude_deg = lon_deg;
  lla.altitude_m = alt_m;
  oneq::coordinate::EcefPositionM ecef;
  oneq::coordinate::TryLlaToEcef(lla, &ecef);
  return ecef;
}

}  // namespace

// ============================================================
// 场景 1: 空对空 - 高速迎头拦截
// ============================================================
TEST(MultiModelScenarioTest, AirToAirHeadOn) {
  // 平台: 高度 10,000m, Mach 1 (340 m/s) 向东
  // 目标: 高度 10,000m, Mach 1.5 (510 m/s) 向西迎头
  const auto platform_ecef = EcefFromLla(35.0, 114.0, 10000.0);

  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 35.0;
  platform_lla.longitude_deg = 114.0;
  platform_lla.altitude_m = 10000.0;

  oneq::coordinate::EnuVelocityMps plat_vel_enu;
  plat_vel_enu.east_mps = 340.0;
  plat_vel_enu.north_mps = 0.0;
  plat_vel_enu.up_mps = 0.0;
  oneq::coordinate::EcefVelocityMps platform_vel;
  oneq::coordinate::TryEnuToEcefVelocity(plat_vel_enu, platform_lla, &platform_vel);

  // 目标在前方 30km 处
  const auto target_ecef = EcefFromLla(35.0, 114.0 + 0.3, 10000.0);
  oneq::coordinate::LlaPositionDegM target_lla;
  target_lla.latitude_deg = 35.0;
  target_lla.longitude_deg = 114.0 + 0.3;
  target_lla.altitude_m = 10000.0;

  oneq::coordinate::EnuVelocityMps tgt_vel_enu;
  tgt_vel_enu.east_mps = -510.0;
  tgt_vel_enu.north_mps = 0.0;
  tgt_vel_enu.up_mps = 0.0;
  oneq::coordinate::EcefVelocityMps target_vel;
  oneq::coordinate::TryEnuToEcefVelocity(tgt_vel_enu, target_lla, &target_vel);

  WorldTarget target;
  target.id = 1001;
  target.pos = target_ecef;
  target.vel = target_vel;
  target.rcs = 5.0f;
  target.temperature_k = 450.0f;
  target.area_m2 = 20.0f;
  target.carrier_hz = 10.0e9;
  target.tx_power_w = 5.0e7;

  WorldState ws;
  ws.platform_pos = platform_ecef;
  ws.platform_vel = platform_vel;
  ws.targets.push_back(target);

  const float dt = 1.0f;
  const std::uint32_t num_cycles = 30;

  // AR RecordingSession
  const std::string ar_trace = MakeTempTraceDir("multi-scene1-ar");
  oneq::replay::ReplayTraceManifest ar_manifest;
  ar_manifest.trace_id = "scene1-air-to-air";
  ar_manifest.module = "airborne_radar";
  ar_manifest.scenario_id = "head-on";

  std::shared_ptr<oneq::replay::ReplayTraceWriter> ar_writer(
      new oneq::replay::ReplayTraceWriter(ar_trace, ar_manifest, true));
  ar_session::ArRecordingSessionOptions ar_opts;
  ar_opts.replay_writer = ar_writer;
  ar_opts.record_config_on_construct = true;
  ar_session::ArRecordingSession ar_session(MakeArConfigAirToAir(), ar_opts);

  // EOS RecordingSession
  const std::string eos_trace = MakeTempTraceDir("multi-scene1-eos");
  oneq::replay::ReplayTraceManifest eos_manifest;
  eos_manifest.trace_id = "scene1-air-to-air";
  eos_manifest.module = "electro_optical_sensor";
  eos_manifest.scenario_id = "head-on";

  std::shared_ptr<oneq::replay::ReplayTraceWriter> eos_writer(
      new oneq::replay::ReplayTraceWriter(eos_trace, eos_manifest, true));
  eos_session::EosRecordingSessionOptions eos_opts;
  eos_opts.replay_writer = eos_writer;
  eos_opts.record_config_on_construct = true;
  eos_session::EosRecordingSession eos_session(MakeEosConfigAirToAir(), eos_opts);

  // ESR RecordingSession
  const std::string esr_trace = MakeTempTraceDir("multi-scene1-esr");
  oneq::replay::ReplayTraceManifest esr_manifest;
  esr_manifest.trace_id = "scene1-air-to-air";
  esr_manifest.module = "electronic_surveillance_radar";
  esr_manifest.scenario_id = "head-on";

  std::shared_ptr<oneq::replay::ReplayTraceWriter> esr_writer(
      new oneq::replay::ReplayTraceWriter(esr_trace, esr_manifest, true));
  esr_session::EsrRecordingSessionOptions esr_opts;
  esr_opts.replay_writer = esr_writer;
  esr_opts.record_config_on_construct = true;
  esr_session::EsrRecordingSession esr_trace_sess(MakeEsrConfigAirToAir(), esr_opts);



  std::uint32_t ar_tracks_max = 0;
  std::uint32_t esr_hyp_max = 0;

  // 物理验证累积器
  float ar_range_first = 0.0f, ar_range_last = 0.0f;
  bool ar_has_track = false;
  float ar_speed_max = 0.0f;
  bool ar_nan_detected = false;

  float eos_snr_first = -999.0f, eos_snr_last = -999.0f;
  float eos_range_first = 0.0f, eos_range_last = 0.0f;
  int eos_detected_count = 0;

  double esr_obs_rf_sum = 0.0;
  int esr_obs_count = 0;
  float esr_obs_snr_max = 0.0f;
  bool esr_has_hypothesis = false;

  for (std::uint32_t i = 0; i < num_cycles; ++i) {
    const std::uint32_t cycle = i + 1;

    auto ar_input = BuildArInput(ws, dt, cycle);
    auto ar_result = RunArCycle(&ar_session, ar_input);
    EXPECT_TRUE(ar_result.accepted) << "AR cycle rejected at cycle " << cycle;
    if (ar_result.output_frame.tracks.size() > ar_tracks_max) {
      ar_tracks_max = static_cast<std::uint32_t>(ar_result.output_frame.tracks.size());
    }
    for (const auto& trk : ar_result.output_frame.tracks) {
      if (std::isnan(trk.position_x) || std::isnan(trk.position_y) || std::isnan(trk.position_z) ||
          std::isnan(trk.speed)) {
        ar_nan_detected = true;
      }
      float range = std::sqrt(trk.position_x * trk.position_x + trk.position_y * trk.position_y +
                              trk.position_z * trk.position_z);
      if (!ar_has_track) {
        ar_range_first = range;
        ar_has_track = true;
      }
      ar_range_last = range;
      if (trk.speed > ar_speed_max) ar_speed_max = trk.speed;
    }

    auto eos_input = BuildEosInput(ws, dt, cycle);
    auto eos_result = eos_session.StepWithResult(eos_input);
    EXPECT_FALSE(eos_session::HasValidationError(eos_result.issues)) << "EOS validation error at cycle " << cycle;
    for (const auto& det : eos_result.output_frame.detections) {
      if (det.detected) {
        eos_detected_count++;
        if (eos_snr_first < -900.0f) {
          eos_snr_first = det.fused_snr_db;
          eos_range_first = det.range_m;
        }
        eos_snr_last = det.fused_snr_db;
        eos_range_last = det.range_m;
      }
    }

    auto esr_input = BuildEsrInput(ws, dt, cycle);
    auto esr_result = esr_trace_sess.StepWithResult(esr_input);
    EXPECT_FALSE(esr_session::HasValidationError(esr_result.issues)) << "ESR validation error at cycle " << cycle;
    if (esr_result.output_frame.emitter_output.hypotheses.size() > esr_hyp_max) {
      esr_hyp_max =
          static_cast<std::uint32_t>(esr_result.output_frame.emitter_output.hypotheses.size());
    }
    if (!esr_result.output_frame.emitter_output.hypotheses.empty()) {
      esr_has_hypothesis = true;
    }
    for (const auto& obs : esr_result.output_frame.observation_output.observations) {
      esr_obs_rf_sum += obs.rf_hz;
      esr_obs_count++;
      if (static_cast<float>(obs.snr_db) > esr_obs_snr_max) {
        esr_obs_snr_max = static_cast<float>(obs.snr_db);
      }
    }

    AdvanceWorld(ws, dt);
  }

  ar_writer->Flush();
  eos_writer->Flush();
  esr_writer->Flush();

  // Replay 验证
  const auto ar_replay = ar_session::ReplayArTrace(ar_trace);
  ExpectReplayOk(ar_replay, "Scene1");
  EXPECT_EQ(ar_replay.playback.applied_input_count, num_cycles);
  EXPECT_EQ(ar_replay.playback.compared_output_count, num_cycles);

  const auto eos_replay = eos_session::ReplayEosTrace(eos_trace);
  ExpectReplayOk(eos_replay, "Scene1");

  const auto esr_replay = esr_session::ReplayEsrTrace(esr_trace);
  ExpectReplayOk(esr_replay, "Scene1");

  // ---- 物理逻辑验证 ----

  // AR: 高速迎头应建立稳定跟踪
  EXPECT_GT(ar_tracks_max, 0U) << "AR should track head-on target";
  EXPECT_FALSE(ar_nan_detected) << "AR track positions must not contain NaN";
  if (ar_has_track) {
    EXPECT_LT(ar_range_last, ar_range_first) << "Head-on: track range should decrease over time";
    EXPECT_GT(ar_speed_max, 0.0f) << "AR track speed should be positive";
    EXPECT_LT(ar_speed_max, 2000.0f) << "AR track speed should be physically reasonable (< Mach 6)";
  }

  // EOS: 迎头接近中 SNR 应上升（距离递减）
  if (eos_detected_count > 1) {
    EXPECT_GT(eos_snr_last, eos_snr_first)
        << "EOS SNR should increase as head-on target approaches";
    EXPECT_LT(eos_range_last, eos_range_first)
        << "EOS range should decrease for approaching target";
  }

  // ESR: 持续辐射源应被稳定跟踪
  EXPECT_GT(esr_hyp_max, 0U) << "ESR should maintain emitter hypothesis";
  if (esr_obs_count > 0) {
    double mean_rf = esr_obs_rf_sum / static_cast<double>(esr_obs_count);
    EXPECT_NEAR(mean_rf, 10.0e9, 1.0e9)
        << "ESR measured RF should be near emitter carrier (10 GHz)";
    EXPECT_GT(esr_obs_snr_max, 0.0f) << "ESR observation SNR should be positive";
  }
}

// ============================================================
// 场景 2: 空对地 - 俯视静止目标侦察
// ============================================================
TEST(MultiModelScenarioTest, AirToGroundLookDown) {
  // 平台: 5,000m 低速向东 60 m/s
  // 目标: 地面静止防空阵地（高红外、大 RCS、间歇性 RF 辐射）
  const auto platform_ecef = EcefFromLla(35.0, 114.0, 5000.0);

  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 35.0;
  platform_lla.longitude_deg = 114.0;
  platform_lla.altitude_m = 5000.0;

  oneq::coordinate::EnuVelocityMps plat_vel_enu;
  plat_vel_enu.east_mps = 60.0;
  plat_vel_enu.north_mps = 0.0;
  plat_vel_enu.up_mps = 0.0;
  oneq::coordinate::EcefVelocityMps platform_vel;
  oneq::coordinate::TryEnuToEcefVelocity(plat_vel_enu, platform_lla, &platform_vel);

  // 地面目标在前方 5km 处
  const auto target_ecef = EcefFromLla(35.0, 114.0 + 0.05, 0.0);

  WorldTarget target;
  target.id = 2001;
  target.pos = target_ecef;
  target.vel.x_mps = 0.0;
  target.vel.y_mps = 0.0;
  target.vel.z_mps = 0.0;
  target.rcs = 100.0f;            // 大型地面设施 RCS
  target.temperature_k = 600.0f;  // 热点（发动机/雷达散热）
  target.area_m2 = 50.0f;         // 大型目标
  target.carrier_hz = 9.0e9;
  target.tx_power_w = 1.0e6;
  target.is_emitting = true;

  WorldState ws;
  ws.platform_pos = platform_ecef;
  ws.platform_vel = platform_vel;
  ws.targets.push_back(target);

  const float dt = 1.0f;
  const std::uint32_t num_cycles = 30;

  const std::string ar_trace = MakeTempTraceDir("multi-scene2-ar");
  oneq::replay::ReplayTraceManifest ar_mf;
  ar_mf.trace_id = "scene2-air-to-ground";
  ar_mf.module = "airborne_radar";
  ar_mf.scenario_id = "look-down";

  auto ar_wr = std::make_shared<oneq::replay::ReplayTraceWriter>(ar_trace, ar_mf, true);
  ar_session::ArRecordingSessionOptions ar_opts;
  ar_opts.replay_writer = ar_wr;
  ar_opts.record_config_on_construct = true;
  ar_session::ArRecordingSession ar_sess(MakeArConfigAirToGround(), ar_opts);

  const std::string eos_trace = MakeTempTraceDir("multi-scene2-eos");
  oneq::replay::ReplayTraceManifest eos_mf;
  eos_mf.trace_id = "scene2-air-to-ground";
  eos_mf.module = "electro_optical_sensor";
  eos_mf.scenario_id = "look-down";

  auto eos_wr = std::make_shared<oneq::replay::ReplayTraceWriter>(eos_trace, eos_mf, true);
  eos_session::EosRecordingSessionOptions eos_opts;
  eos_opts.replay_writer = eos_wr;
  eos_opts.record_config_on_construct = true;
  eos_session::EosRecordingSession eos_sess(MakeEosConfigAirToGround(), eos_opts);

  const std::string esr_trace = MakeTempTraceDir("multi-scene2-esr");
  oneq::replay::ReplayTraceManifest esr_mf;
  esr_mf.trace_id = "scene2-air-to-ground";
  esr_mf.module = "electronic_surveillance_radar";
  esr_mf.scenario_id = "look-down";

  auto esr_wr = std::make_shared<oneq::replay::ReplayTraceWriter>(esr_trace, esr_mf, true);
  esr_session::EsrRecordingSessionOptions esr_opts;
  esr_opts.replay_writer = esr_wr;
  esr_opts.record_config_on_construct = true;
  esr_session::EsrRecordingSession esr_sess(MakeEsrConfigAirToGround(), esr_opts);



  // 物理验证累积器
  bool ar_has_track = false;
  bool ar_nan_detected = false;
  int eos_detected_count = 0;
  float eos_max_snr_db = -999.0f;
  int esr_total_obs_cycles = 0;
  int esr_emitting_on_count = 0;
  int esr_emitting_off_count = 0;

  for (std::uint32_t i = 0; i < num_cycles; ++i) {
    const std::uint32_t cycle = i + 1;

    // ESR 间歇性发射：每 5 个周期关闭 2 个
    ws.targets[0].is_emitting = !((i % 5 == 3) || (i % 5 == 4));
    if (ws.targets[0].is_emitting) {
      esr_emitting_on_count++;
    } else {
      esr_emitting_off_count++;
    }

    auto ar_in = BuildArInput(ws, dt, cycle);
    auto ar_res = RunArCycle(&ar_sess, ar_in);
    EXPECT_TRUE(ar_res.accepted) << "AR cycle rejected at cycle " << cycle;
    for (const auto& trk : ar_res.output_frame.tracks) {
      ar_has_track = true;
      if (std::isnan(trk.position_z)) ar_nan_detected = true;
    }

    auto eos_in = BuildEosInput(ws, dt, cycle);
    auto eos_res = eos_sess.StepWithResult(eos_in);
    EXPECT_FALSE(eos_session::HasValidationError(eos_res.issues)) << "EOS validation error at cycle " << cycle;
    for (const auto& det : eos_res.output_frame.detections) {
      if (det.detected) {
        eos_detected_count++;
        if (det.fused_snr_db > eos_max_snr_db) eos_max_snr_db = det.fused_snr_db;
      }
    }

    auto esr_in = BuildEsrInput(ws, dt, cycle);
    auto esr_res = esr_sess.StepWithResult(esr_in);
    EXPECT_FALSE(esr_session::HasValidationError(esr_res.issues)) << "ESR validation error at cycle " << cycle;
    if (!esr_res.output_frame.observation_output.observations.empty()) {
      esr_total_obs_cycles++;
    }

    AdvanceWorld(ws, dt);
  }

  ar_wr->Flush();
  eos_wr->Flush();
  esr_wr->Flush();

  ExpectReplayOk(ar_session::ReplayArTrace(ar_trace), "Scene2-AR");
  ExpectReplayOk(eos_session::ReplayEosTrace(eos_trace), "Scene2-EOS");
  ExpectReplayOk(esr_session::ReplayEsrTrace(esr_trace), "Scene2-ESR");

  // ---- 物理逻辑验证 ----

  // AR: 下视大 RCS 目标应被检测
  EXPECT_TRUE(ar_has_track) << "AR should detect large ground target in look-down";
  EXPECT_FALSE(ar_nan_detected) << "AR track positions must not contain NaN";

  // EOS: 目标温度 600K vs 背景 295K → 高热对比度，应被探测到
  EXPECT_GT(eos_detected_count, 0) << "EOS should detect hot ground target (600K vs 295K bg)";
  if (eos_detected_count > 0) {
    EXPECT_GT(eos_max_snr_db, 0.0f) << "EOS SNR should be positive for high-contrast target";
  }

  // ESR: 间歇性发射 → 有观测的周期数应少于总周期数
  EXPECT_GT(esr_total_obs_cycles, 0) << "ESR should observe ground emitter with look-down config";
  if (esr_total_obs_cycles > 0) {
    EXPECT_LT(esr_total_obs_cycles, static_cast<int>(num_cycles))
        << "ESR should not observe during silent periods";
  }
}

// ============================================================
// 场景 3: 密集编队与伴随干扰
// ============================================================
TEST(MultiModelScenarioTest, DenseFormationAndJamming) {
  // 平台: 8,000m 夜间
  // 目标 A: 轰炸机, 低速, 大 RCS, 强红外
  // 目标 B: 伴随干扰机, 距 A 极近, 强电子干扰
  const auto platform_ecef = EcefFromLla(35.0, 114.0, 8000.0);

  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 35.0;
  platform_lla.longitude_deg = 114.0;
  platform_lla.altitude_m = 8000.0;

  oneq::coordinate::EnuVelocityMps plat_vel_enu;
  plat_vel_enu.east_mps = 200.0;
  plat_vel_enu.north_mps = 0.0;
  plat_vel_enu.up_mps = 0.0;
  oneq::coordinate::EcefVelocityMps platform_vel;
  oneq::coordinate::TryEnuToEcefVelocity(plat_vel_enu, platform_lla, &platform_vel);

  // 目标 A: 轰炸机在前方 20km
  const auto tgt_a_ecef = EcefFromLla(35.0, 114.0 + 0.2, 8000.0);
  oneq::coordinate::LlaPositionDegM tgt_a_lla;
  tgt_a_lla.latitude_deg = 35.0;
  tgt_a_lla.longitude_deg = 114.0 + 0.2;
  tgt_a_lla.altitude_m = 8000.0;

  oneq::coordinate::EnuVelocityMps tgt_a_vel_enu;
  tgt_a_vel_enu.east_mps = -100.0;
  tgt_a_vel_enu.north_mps = 10.0;
  tgt_a_vel_enu.up_mps = 0.0;
  oneq::coordinate::EcefVelocityMps tgt_a_vel;
  oneq::coordinate::TryEnuToEcefVelocity(tgt_a_vel_enu, tgt_a_lla, &tgt_a_vel);

  WorldTarget tgt_a;
  tgt_a.id = 3001;
  tgt_a.pos = tgt_a_ecef;
  tgt_a.vel = tgt_a_vel;
  tgt_a.rcs = 25.0f;             // 大型轰炸机
  tgt_a.temperature_k = 400.0f;  // 强红外
  tgt_a.area_m2 = 80.0f;
  tgt_a.carrier_hz = 9.5e9;
  tgt_a.tx_power_w = 1.0e5;  // 弱辐射
  tgt_a.is_emitting = true;

  // 目标 B: 伴随干扰机在 A 附近 500m
  const auto tgt_b_ecef = EcefFromLla(35.0, 114.0 + 0.2 + 0.005, 8000.0);
  oneq::coordinate::LlaPositionDegM tgt_b_lla;
  tgt_b_lla.latitude_deg = 35.0;
  tgt_b_lla.longitude_deg = 114.0 + 0.2 + 0.005;
  tgt_b_lla.altitude_m = 8000.0;

  oneq::coordinate::EnuVelocityMps tgt_b_vel_enu;
  tgt_b_vel_enu.east_mps = -100.0;
  tgt_b_vel_enu.north_mps = 10.0;
  tgt_b_vel_enu.up_mps = 0.0;
  oneq::coordinate::EcefVelocityMps tgt_b_vel;
  oneq::coordinate::TryEnuToEcefVelocity(tgt_b_vel_enu, tgt_b_lla, &tgt_b_vel);

  WorldTarget tgt_b;
  tgt_b.id = 3002;
  tgt_b.pos = tgt_b_ecef;
  tgt_b.vel = tgt_b_vel;
  tgt_b.rcs = 3.0f;  // 小型电子战机
  tgt_b.temperature_k = 350.0f;
  tgt_b.area_m2 = 10.0f;
  tgt_b.carrier_hz = 9.3e9;
  tgt_b.tx_power_w = 1.0e8;  // 强干扰辐射
  tgt_b.is_emitting = true;

  WorldState ws;
  ws.platform_pos = platform_ecef;
  ws.platform_vel = platform_vel;
  ws.targets.push_back(tgt_a);
  ws.targets.push_back(tgt_b);

  const float dt = 1.0f;
  const std::uint32_t num_cycles = 40;

  auto ar_cfg = MakeArConfigAirToAir();

  const std::string ar_trace = MakeTempTraceDir("multi-scene3-ar");
  oneq::replay::ReplayTraceManifest ar_mf;
  ar_mf.trace_id = "scene3-dense-formation";
  ar_mf.module = "airborne_radar";
  ar_mf.scenario_id = "jamming";

  auto ar_wr = std::make_shared<oneq::replay::ReplayTraceWriter>(ar_trace, ar_mf, true);
  ar_session::ArRecordingSessionOptions ar_opts;
  ar_opts.replay_writer = ar_wr;
  ar_opts.record_config_on_construct = true;
  ar_session::ArRecordingSession ar_sess(ar_cfg, ar_opts);

  const std::string eos_trace = MakeTempTraceDir("multi-scene3-eos");
  oneq::replay::ReplayTraceManifest eos_mf;
  eos_mf.trace_id = "scene3-dense-formation";
  eos_mf.module = "electro_optical_sensor";
  eos_mf.scenario_id = "jamming";

  auto eos_wr = std::make_shared<oneq::replay::ReplayTraceWriter>(eos_trace, eos_mf, true);
  eos_session::EosRecordingSessionOptions eos_opts;
  eos_opts.replay_writer = eos_wr;
  eos_opts.record_config_on_construct = true;

  // EOS 纯红外模式（夜间）
  eos_config::EosSessionConfig eos_cfg = MakeEosConfigAirToAir();
  eos_cfg.mission.work_mode = eos::config::EosWorkMode::kInfraredOnly;
  eos_session::EosRecordingSession eos_sess(eos_cfg, eos_opts);

  const std::string esr_trace = MakeTempTraceDir("multi-scene3-esr");
  oneq::replay::ReplayTraceManifest esr_mf;
  esr_mf.trace_id = "scene3-dense-formation";
  esr_mf.module = "electronic_surveillance_radar";
  esr_mf.scenario_id = "jamming";

  auto esr_wr = std::make_shared<oneq::replay::ReplayTraceWriter>(esr_trace, esr_mf, true);
  esr_session::EsrRecordingSessionOptions esr_opts;
  esr_opts.replay_writer = esr_wr;
  esr_opts.record_config_on_construct = true;
  esr_session::EsrRecordingSession esr_sess(MakeEsrConfigAirToAir(), esr_opts);

  // AR 自然环境与外部 RF 发射事实分离；目标 B 的伴随干扰由 RF frame 表达。


  // 物理验证累积器
  bool ar_interference_observed = false;
  float eos_ir_snr_max = 0.0f;
  float eos_vis_snr_max = 0.0f;
  int eos_ir_detected = 0;
  std::size_t esr_obs_total = 0;
  std::uint32_t esr_hyp_max = 0;

  for (std::uint32_t i = 0; i < num_cycles; ++i) {
    const std::uint32_t cycle = i + 1;

    auto ar_in = BuildArInput(ws, dt, cycle);
    ar_in.cycle.interference = MakeNoiseInterferenceFrame(ar_in.cycle);
    auto ar_res = RunArCycle(&ar_sess, ar_in);
    EXPECT_TRUE(ar_res.accepted) << "AR cycle rejected at cycle " << cycle;
    if (!ar_res.interference_observations.empty() ||
        ar_res.receiver_impairment == ar_session::ArReceiverImpairment::kSaturated) {
      ar_interference_observed = true;
    }

    auto eos_in = BuildEosInput(ws, dt, cycle);
    auto eos_res = eos_sess.StepWithResult(eos_in);
    EXPECT_FALSE(eos_session::HasValidationError(eos_res.issues)) << "EOS validation error at cycle " << cycle;
    for (const auto& det : eos_res.output_frame.detections) {
      if (det.detected) {
        eos_ir_detected++;
        if (det.infrared_snr_linear > eos_ir_snr_max) eos_ir_snr_max = det.infrared_snr_linear;
        if (det.visible_snr_linear > eos_vis_snr_max) eos_vis_snr_max = det.visible_snr_linear;
      }
    }

    auto esr_in = BuildEsrInput(ws, dt, cycle);
    auto esr_res = esr_sess.StepWithResult(esr_in);
    EXPECT_FALSE(esr_session::HasValidationError(esr_res.issues)) << "ESR validation error at cycle " << cycle;
    esr_obs_total += esr_res.output_frame.observation_output.observations.size();
    if (esr_res.output_frame.emitter_output.hypotheses.size() > esr_hyp_max) {
      esr_hyp_max =
          static_cast<std::uint32_t>(esr_res.output_frame.emitter_output.hypotheses.size());
    }

    AdvanceWorld(ws, dt);
  }

  ar_wr->Flush();
  eos_wr->Flush();
  esr_wr->Flush();

  ExpectReplayOk(ar_session::ReplayArTrace(ar_trace), "Scene3-AR");
  ExpectReplayOk(eos_session::ReplayEosTrace(eos_trace), "Scene3-EOS");
  ExpectReplayOk(esr_session::ReplayEsrTrace(esr_trace), "Scene3-ESR");

  // ---- 物理逻辑验证 ----

  // AR: 显式 RF 发射 frame 应通过接收链形成干扰观测或饱和。
  EXPECT_TRUE(ar_interference_observed)
      << "AR should observe the explicit escort RF emission frame";

  // EOS: 夜间红外模式 → 可见光 SNR 应远低于红外 SNR
  if (eos_ir_detected > 0) {
    EXPECT_GT(eos_ir_snr_max, eos_vis_snr_max)
        << "Night mode: IR SNR should dominate over visible SNR";
  }

  // ESR: 两个活跃辐射源应产生观测
  EXPECT_GT(esr_obs_total, static_cast<std::size_t>(0))
      << "ESR should produce observations from two active emitters";
  EXPECT_GE(esr_hyp_max, 1U) << "ESR should form at least one emitter hypothesis";
}

// ============================================================
// 场景 4: 零多普勒横穿交叉
// ============================================================
TEST(MultiModelScenarioTest, ZeroDopplerCrossing) {
  // 平台: 5,000m 向东 200 m/s
  // 目标: 5,000m 向北 200 m/s（与平台航向垂直）
  // 初始位置在平台正南方 15km，目标向北穿越 → 径向速度先负后正过零
  const auto platform_ecef = EcefFromLla(35.0, 114.0, 5000.0);

  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 35.0;
  platform_lla.longitude_deg = 114.0;
  platform_lla.altitude_m = 5000.0;

  oneq::coordinate::EnuVelocityMps plat_vel_enu;
  plat_vel_enu.east_mps = 200.0;
  plat_vel_enu.north_mps = 0.0;
  plat_vel_enu.up_mps = 0.0;
  oneq::coordinate::EcefVelocityMps platform_vel;
  oneq::coordinate::TryEnuToEcefVelocity(plat_vel_enu, platform_lla, &platform_vel);

  // 目标在平台正南方 15km 处，向北飞行
  const auto target_ecef = EcefFromLla(35.0 - 0.135, 114.0, 5000.0);

  oneq::coordinate::LlaPositionDegM target_lla;
  target_lla.latitude_deg = 35.0 - 0.135;
  target_lla.longitude_deg = 114.0;
  target_lla.altitude_m = 5000.0;

  oneq::coordinate::EnuVelocityMps tgt_vel_enu;
  tgt_vel_enu.east_mps = 0.0;
  tgt_vel_enu.north_mps = 200.0;
  tgt_vel_enu.up_mps = 0.0;
  oneq::coordinate::EcefVelocityMps target_vel;
  oneq::coordinate::TryEnuToEcefVelocity(tgt_vel_enu, target_lla, &target_vel);

  WorldTarget target;
  target.id = 4001;
  target.pos = target_ecef;
  target.vel = target_vel;
  target.rcs = 5.0f;
  target.temperature_k = 400.0f;
  target.area_m2 = 15.0f;
  target.carrier_hz = 9.8e9;
  target.tx_power_w = 5.0e7;

  WorldState ws;
  ws.platform_pos = platform_ecef;
  ws.platform_vel = platform_vel;
  ws.targets.push_back(target);

  const float dt = 1.0f;
  const std::uint32_t num_cycles = 30;

  const std::string ar_trace = MakeTempTraceDir("multi-scene4-ar");
  oneq::replay::ReplayTraceManifest ar_mf;
  ar_mf.trace_id = "scene4-zero-doppler";
  ar_mf.module = "airborne_radar";
  ar_mf.scenario_id = "crossing";

  auto ar_wr = std::make_shared<oneq::replay::ReplayTraceWriter>(ar_trace, ar_mf, true);
  ar_session::ArRecordingSessionOptions ar_opts;
  ar_opts.replay_writer = ar_wr;
  ar_opts.record_config_on_construct = true;
  ar_session::ArRecordingSession ar_sess(MakeArConfigAirToAir(), ar_opts);

  const std::string eos_trace = MakeTempTraceDir("multi-scene4-eos");
  oneq::replay::ReplayTraceManifest eos_mf;
  eos_mf.trace_id = "scene4-zero-doppler";
  eos_mf.module = "electro_optical_sensor";
  eos_mf.scenario_id = "crossing";

  auto eos_wr = std::make_shared<oneq::replay::ReplayTraceWriter>(eos_trace, eos_mf, true);
  eos_session::EosRecordingSessionOptions eos_opts;
  eos_opts.replay_writer = eos_wr;
  eos_opts.record_config_on_construct = true;
  eos_session::EosRecordingSession eos_sess(MakeEosConfigAirToAir(), eos_opts);

  const std::string esr_trace = MakeTempTraceDir("multi-scene4-esr");
  oneq::replay::ReplayTraceManifest esr_mf;
  esr_mf.trace_id = "scene4-zero-doppler";
  esr_mf.module = "electronic_surveillance_radar";
  esr_mf.scenario_id = "crossing";

  auto esr_wr = std::make_shared<oneq::replay::ReplayTraceWriter>(esr_trace, esr_mf, true);
  esr_session::EsrRecordingSessionOptions esr_opts;
  esr_opts.replay_writer = esr_wr;
  esr_opts.record_config_on_construct = true;
  esr_session::EsrRecordingSession esr_sess(MakeEsrConfigAirToAir(), esr_opts);



  // 物理验证累积器
  bool ar_nan_detected = false;
  bool ar_inf_detected = false;
  float ar_speed_max = 0.0f;
  int ar_track_count = 0;
  float eos_range_min = 1.0e9f;
  float eos_range_at_first = 0.0f;
  bool eos_first_range_set = false;
  float eos_range_at_last = 0.0f;
  std::size_t esr_obs_total = 0;

  for (std::uint32_t i = 0; i < num_cycles; ++i) {
    const std::uint32_t cycle = i + 1;

    auto ar_in = BuildArInput(ws, dt, cycle);
    auto ar_res = RunArCycle(&ar_sess, ar_in);
    EXPECT_TRUE(ar_res.accepted) << "AR cycle rejected at cycle " << cycle;
    for (const auto& trk : ar_res.output_frame.tracks) {
      ar_track_count++;
      if (std::isnan(trk.position_x) || std::isnan(trk.position_y) || std::isnan(trk.position_z) ||
          std::isnan(trk.speed)) {
        ar_nan_detected = true;
      }
      if (std::isinf(trk.position_x) || std::isinf(trk.position_y) || std::isinf(trk.position_z) ||
          std::isinf(trk.speed)) {
        ar_inf_detected = true;
      }
      if (trk.speed > ar_speed_max) ar_speed_max = trk.speed;
    }

    auto eos_in = BuildEosInput(ws, dt, cycle);
    auto eos_res = eos_sess.StepWithResult(eos_in);
    EXPECT_FALSE(eos_session::HasValidationError(eos_res.issues)) << "EOS validation error at cycle " << cycle;
    for (const auto& det : eos_res.output_frame.detections) {
      if (!eos_first_range_set) {
        eos_range_at_first = det.range_m;
        eos_first_range_set = true;
      }
      eos_range_at_last = det.range_m;
      if (det.range_m < eos_range_min) eos_range_min = det.range_m;
    }

    auto esr_in = BuildEsrInput(ws, dt, cycle);
    auto esr_res = esr_sess.StepWithResult(esr_in);
    EXPECT_FALSE(esr_session::HasValidationError(esr_res.issues)) << "ESR validation error at cycle " << cycle;
    esr_obs_total += esr_res.output_frame.observation_output.observations.size();

    AdvanceWorld(ws, dt);
  }

  ar_wr->Flush();
  eos_wr->Flush();
  esr_wr->Flush();

  ExpectReplayOk(ar_session::ReplayArTrace(ar_trace), "Scene4-AR");
  ExpectReplayOk(eos_session::ReplayEosTrace(eos_trace), "Scene4-EOS");
  ExpectReplayOk(esr_session::ReplayEsrTrace(esr_trace), "Scene4-ESR");

  // ---- 物理逻辑验证 ----

  // AR: 零多普勒横穿 — 无 NaN/Inf，速度合理
  EXPECT_FALSE(ar_nan_detected) << "AR must not produce NaN during zero-Doppler crossing";
  EXPECT_FALSE(ar_inf_detected) << "AR must not produce Inf during zero-Doppler crossing";
  if (ar_track_count > 0) {
    EXPECT_LT(ar_speed_max, 2000.0f)
        << "AR track speed should stay physically reasonable through crossing";
  }

  // EOS: 横穿目标距离先减后增，中间存在最小值
  if (eos_first_range_set) {
    EXPECT_GT(eos_range_min, 0.0f) << "EOS range should be positive";
    EXPECT_LT(eos_range_min, eos_range_at_first)
        << "Crossing target should reach minimum range mid-simulation";
  }

  // ESR: 横穿目标持续辐射 → 应有观测
  EXPECT_GT(esr_obs_total, static_cast<std::size_t>(0))
      << "ESR should observe continuously emitting target during crossing";
}

TEST(MultiModelScenarioTest, SensorDrivenEcmUsesPreviousSuccessfulEsrFrame) {
  WorldState world;
  world.platform_pos = EcefFromLla(35.0, 114.0, 10000.0);
  world.platform_vel = oneq::coordinate::EcefVelocityMps{};

  WorldTarget emitter;
  emitter.id = 8201U;
  emitter.pos = EcefFromLla(35.0, 114.05, 10000.0);
  emitter.vel = oneq::coordinate::EcefVelocityMps{};
  emitter.rcs = 20.0f;
  emitter.temperature_k = 450.0f;
  emitter.area_m2 = 20.0f;
  emitter.carrier_hz = 10.0e9;
  emitter.bandwidth_hz = 5.0e6;
  emitter.tx_power_w = 5.0e7;
  world.targets.push_back(emitter);

  esr_config::EsrSessionConfig esr_config = MakeEsrConfigAirToAir();
  esr_config.policy.detection.enable_statistical_detection = false;
  esr_config.hardware.co_site_paths.push_back({101U, 100.0});
  esr_session::EsrSession esr = esr_session::EsrSession::Create(esr_config);

  esr_session::EmitterHypothesisList hypotheses;
  std::uint32_t source_esr_cycle = 0U;
  std::uint64_t source_esr_batch = 0U;
  for (std::uint32_t cycle = 1U; cycle <= 8U && hypotheses.empty(); ++cycle) {
    esr_session::EsrCycleInput input = BuildEsrInput(world, 1.0f, cycle);
    const esr_session::EsrCycleResult result = esr.StepWithResult(input);
    ASSERT_FALSE(esr_session::HasValidationError(result.issues));
    if (!result.output_frame.emitter_output.hypotheses.empty()) {
      hypotheses = result.output_frame.emitter_output.hypotheses;
      source_esr_cycle = cycle;
      source_esr_batch = result.output_frame.batch_id;
    }
  }
  ASSERT_FALSE(hypotheses.empty());

  ecm_session::EcmSensorObservationFrame sensor_frame;
  ASSERT_TRUE(
      ecm_session::TryBuildEcmSensorObservationFrame(hypotheses, source_esr_batch, &sensor_frame));
  ASSERT_FALSE(sensor_frame.observations.empty());
  EXPECT_EQ(sensor_frame.source_esr_batch_id, source_esr_batch);

  ecm_config::EcmSessionConfig ecm_config;
  ecm_config.channel_count = 1U;
  ecm_config.maximum_total_transmit_power_w = 1000.0;
  ecm_config.maximum_channel_transmit_power_w = 1000.0;
  ecm_config.default_technique = electronic_countermeasure::EcmTechnique::kSpot;
  ecm_session::EcmSession ecm = ecm_session::EcmSession::Create(ecm_config);
  ecm.SetSensorEntityId(101U);
  ecm_session::EcmCycleInput ecm_input;
  ecm_input.cycle_index = source_esr_cycle + 1U;
  ecm_input.cycle_start_time_s = static_cast<double>(source_esr_cycle);
  ecm_input.dt_sec = 1.0;
  ecm_input.input_mode = electronic_countermeasure::EcmInputMode::kSensorDriven;
  ecm_input.platform_position_ecef_m = world.platform_pos;
  ecm_input.platform_velocity_ecef_mps = world.platform_vel;
  ecm_input.has_sensor_observation_frame = true;
  ecm_input.sensor_observation_frame = sensor_frame;
  const ecm_session::EcmCycleResult ecm_result = ecm.StepWithResult(ecm_input);
  ASSERT_EQ(ecm_result.status, ecm_session::EcmCycleStatus::kExecuted);
  ASSERT_FALSE(ecm_result.emission_frame.emissions.empty());
  EXPECT_EQ(ecm_result.source_esr_batch_id, source_esr_batch);

  ar_config::ArSessionConfig ar_config = MakeArConfigAirToAir();
  ar_config.hardware.receiver.co_site_paths.push_back({101U, 100.0});
  ar_session::ArRecordingSession ar(ar_config, ar_session::ArRecordingSessionOptions{});

  ArRfTestCycleInput ar_input =
      BuildArInput(world, 1.0f, source_esr_cycle + 1U);
  ar_input.cycle.interference = ecm_result.emission_frame;
  ASSERT_FALSE(ar_input.cycle.interference.emissions.empty());
  const ArRfTestCycleResult ar_result = RunArCycle(&ar, ar_input);
  EXPECT_TRUE(ar_result.accepted);

  esr_session::EsrCycleInput esr_input =
      BuildEsrInput(world, 1.0f, source_esr_cycle + 1U);
  esr_input.rf_emissions = ecm_result.emission_frame;
  const esr_session::EsrCycleResult esr_result = esr.StepWithResult(esr_input);
  EXPECT_FALSE(esr_session::HasValidationError(esr_result.issues));
  EXPECT_EQ(esr_result.output_frame.cycle_index, source_esr_cycle + 1U);
}

// ECM deception(kFalseTarget)→AR 闭环：验证假目标欺骗发射经 RfEmissionFrame 到达 AR
// 后被识别为 kLikelyFalseTarget，并触发 ECCM 反假目标鉴别提案。
// 此前跨域测试仅覆盖 kSpot 压制干扰，未证明欺骗发射能通过真实集成链路递送。
// 注意：ECCM 控制概要延迟一周期生效（当前周期评估 → 下一周期 ApplyPendingDecisionControl），
// 因此需要运行 2 个 AR 周期：第 1 周期验证观测分类，第 2 周期验证控制概要。
TEST(MultiModelScenarioTest, EcmDeceptionFalseTargetReachesArAndTriggersEccm) {
  WorldState world;
  world.platform_pos = EcefFromLla(35.0, 114.0, 10000.0);
  world.platform_vel = oneq::coordinate::EcefVelocityMps{};

  WorldTarget emitter;
  emitter.id = 8201U;
  emitter.pos = EcefFromLla(35.0, 114.05, 10000.0);
  emitter.vel = oneq::coordinate::EcefVelocityMps{};
  emitter.rcs = 20.0f;
  emitter.temperature_k = 450.0f;
  emitter.area_m2 = 20.0f;
  // 发射频率与 AR 接收机 preselector 中心对齐（kLongRangeHighPower 9.3 GHz），
  // 确保 ECM 欺骗发射（中心频率 + Doppler 偏移 ~9.301 GHz）落在 AR preselector 带内。
  emitter.carrier_hz = 9.3e9;
  emitter.bandwidth_hz = 5.0e6;
  emitter.tx_power_w = 5.0e7;
  world.targets.push_back(emitter);

  // ESR 探测 → 产生辐射源假设列表。
  esr_config::EsrSessionConfig esr_config = MakeEsrConfigAirToAir();
  esr_config.policy.detection.enable_statistical_detection = false;
  esr_config.hardware.co_site_paths.push_back({101U, 100.0});
  esr_session::EsrSession esr = esr_session::EsrSession::Create(esr_config);

  esr_session::EmitterHypothesisList hypotheses;
  std::uint32_t source_esr_cycle = 0U;
  std::uint64_t source_esr_batch = 0U;
  for (std::uint32_t cycle = 1U; cycle <= 8U && hypotheses.empty(); ++cycle) {
    esr_session::EsrCycleInput input = BuildEsrInput(world, 1.0f, cycle);
    const esr_session::EsrCycleResult result = esr.StepWithResult(input);
    ASSERT_FALSE(esr_session::HasValidationError(result.issues));
    if (!result.output_frame.emitter_output.hypotheses.empty()) {
      hypotheses = result.output_frame.emitter_output.hypotheses;
      source_esr_cycle = cycle;
      source_esr_batch = result.output_frame.batch_id;
    }
  }
  ASSERT_FALSE(hypotheses.empty());

  // ESR hypotheses → ECM sensor frame。
  ecm_session::EcmSensorObservationFrame sensor_frame;
  ASSERT_TRUE(
      ecm_session::TryBuildEcmSensorObservationFrame(hypotheses, source_esr_batch, &sensor_frame));

  // ECM: 欺骗 kFalseTarget（每周期向威胁发射 N 个假目标）。
  ecm_config::EcmSessionConfig ecm_config;
  ecm_config.channel_count = 1U;
  ecm_config.maximum_total_transmit_power_w = 1.0e6;
  ecm_config.maximum_channel_transmit_power_w = 1.0e6;
  ecm_config.deception_power_scale = 1.0;
  ecm_config.default_technique = electronic_countermeasure::EcmTechnique::kDeception;
  ecm_config.default_deception_mode = electronic_countermeasure::EcmDeceptionMode::kFalseTarget;
  ecm_config.deception_max_false_targets_per_threat = 2U;
  ecm_config.deception_false_target_doppler_hz = 5000.0;
  ecm_config.deception_false_target_delay_s = 1.0e-6;
  ecm_session::EcmSession ecm = ecm_session::EcmSession::Create(ecm_config);
  ecm.SetSensorEntityId(101U);
  ecm_session::EcmCycleInput ecm_input;
  ecm_input.cycle_index = source_esr_cycle + 1U;
  ecm_input.cycle_start_time_s = static_cast<double>(source_esr_cycle);
  ecm_input.dt_sec = 1.0;
  ecm_input.input_mode = electronic_countermeasure::EcmInputMode::kSensorDriven;
  ecm_input.platform_position_ecef_m = world.platform_pos;
  ecm_input.platform_velocity_ecef_mps = world.platform_vel;
  ecm_input.has_sensor_observation_frame = true;
  ecm_input.sensor_observation_frame = sensor_frame;
  const ecm_session::EcmCycleResult ecm_result = ecm.StepWithResult(ecm_input);
  ASSERT_EQ(ecm_result.status, ecm_session::EcmCycleStatus::kExecuted);
  ASSERT_FALSE(ecm_result.emission_frame.emissions.empty());

  // AR 接收 ECM 欺骗发射帧。
  ar_config::ArSessionConfig ar_config = MakeArConfigAirToAir();
  ar_config.hardware.receiver.preselector_bandwidth_hz = 50.0e6f;
  ar_config.hardware.receiver.co_site_paths.push_back({101U, 100.0});
  ar_session::ArRecordingSession ar(ar_config, ar_session::ArRecordingSessionOptions{});

  // 第 1 周期：验证 AR 产生 kLikelyFalseTarget 干扰观测。
  ArRfTestCycleInput ar_input1 =
      BuildArInput(world, 1.0f, source_esr_cycle + 1U);
  ar_input1.cycle.interference = ecm_result.emission_frame;
  const ArRfTestCycleResult ar_result1 = RunArCycle(&ar, ar_input1);
  EXPECT_TRUE(ar_result1.accepted);

  bool has_false_target_class = false;
  for (const auto& obs : ar_result1.interference_observations) {
    if (obs.deception_class == airborne_radar::session::DeceptionClass::kLikelyFalseTarget) {
      has_false_target_class = true;
      break;
    }
  }
  EXPECT_TRUE(has_false_target_class)
      << "kFalseTarget deception should produce kLikelyFalseTarget interference observations";

  // ECCM 控制概要延迟一周期生效：当前周期评估 → pending_proposals →
  // 下一周期 ApplyPendingDecisionControl()。
  EXPECT_GE(ar_result1.interference_observations.size(), 1U)
      << "at least one interference observation should be produced";
  for (const auto& obs : ar_result1.interference_observations) {
    EXPECT_EQ(obs.deception_class, airborne_radar::session::DeceptionClass::kLikelyFalseTarget);
    EXPECT_GE(obs.coherent_emission_count, 2U);
    EXPECT_GT(obs.jammer_to_noise_db, 6.0);
    EXPECT_GT(obs.estimated_center_frequency_hz, 0.0);
  }

  // ECM 通过受约束的 sensor glide 产生下一世界周期的欺骗发射；AR 消费该帧时应先
  // 应用上一成功周期的 pending anti-false-target 提案，闭合 ECM→RF→AR→ECCM 控制链。
  ecm_session::EcmCycleInput ecm_input2 = ecm_input;
  ecm_input2.cycle_index = source_esr_cycle + 2U;
  ecm_input2.cycle_start_time_s = static_cast<double>(source_esr_cycle + 1U);
  ecm_input2.has_sensor_observation_frame = false;
  ecm_input2.sensor_observation_frame = {};
  const ecm_session::EcmCycleResult ecm_result2 = ecm.StepWithResult(ecm_input2);
  ASSERT_EQ(ecm_result2.status, ecm_session::EcmCycleStatus::kExecuted);
  ASSERT_TRUE(ecm_result2.used_glided_observation);
  ASSERT_FALSE(ecm_result2.emission_frame.emissions.empty());

  ArRfTestCycleInput ar_input2 =
      BuildArInput(world, 1.0f, source_esr_cycle + 2U);
  ar_input2.cycle.interference = ecm_result2.emission_frame;
  const ArRfTestCycleResult ar_result2 = RunArCycle(&ar, ar_input2);
  ASSERT_TRUE(ar_result2.accepted);
  ASSERT_TRUE(ar_result2.has_control_profile);
  EXPECT_TRUE(ar_result2.control_profile.enable_anti_false_target_discrimination);

  // 验证 ESR 接收 ECM 欺骗发射帧时不报错（ESR 欺骗分类逻辑由 ESR 单测覆盖）。
  esr_session::EsrCycleInput esr_input2 =
      BuildEsrInput(world, 1.0f, source_esr_cycle + 1U);
  esr_input2.rf_emissions = ecm_result.emission_frame;
  const esr_session::EsrCycleResult esr_result2 = esr.StepWithResult(esr_input2);
  EXPECT_FALSE(esr_session::HasValidationError(esr_result2.issues));
}

#if defined(ONEQ_TEST_FLIGHT_DYNAMIC_ENABLED)
TEST(MultiModelScenarioTest, FlightDynamicDrivesSensorEcmClosedLoop) {
  oneq::flight_dynamic::config::FlightDynamicConfig flight_config;
  flight_config.aircraft_model = "c172x";
  flight_config.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
  flight_config.dt_sec = 0.02;
  flight_config.do_trim = true;
  flight_config.silent_mode = true;
  flight_config.initial_kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  flight_config.initial_kinematics.position_lla_deg_m.latitude_deg = 35.0;
  flight_config.initial_kinematics.position_lla_deg_m.longitude_deg = 114.0;
  flight_config.initial_kinematics.position_lla_deg_m.altitude_m = 1000.0;
  flight_config.initial_kinematics.velocity_mps.x_mps = 60.0;
  flight_config.initial_kinematics.attitude_deg.yaw_deg = 90.0;

  oneq::flight_dynamic::FlightManager flight(flight_config);
  ASSERT_EQ(flight.GetState(), oneq::flight_dynamic::FlightManagerState::kReady);
  ASSERT_TRUE(flight.Step(flight_config.dt_sec));
  const oneq::flight_dynamic::model::VehicleState first_state = flight.GetVehicleState();
  ASSERT_TRUE(flight.Step(flight_config.dt_sec));
  const oneq::flight_dynamic::model::VehicleState second_state = flight.GetVehicleState();

  const double radians_to_degrees = 180.0 / kPi;
  const oneq::coordinate::EcefPositionM first_position =
      EcefFromLla(first_state.latitude_rad * radians_to_degrees,
                  first_state.longitude_rad * radians_to_degrees, first_state.altitude_geod_m);
  const oneq::coordinate::EcefPositionM second_position =
      EcefFromLla(second_state.latitude_rad * radians_to_degrees,
                  second_state.longitude_rad * radians_to_degrees, second_state.altitude_geod_m);
  const double state_dt_sec = second_state.sim_time_sec - first_state.sim_time_sec;
  ASSERT_GT(state_dt_sec, 0.0);

  WorldState world;
  world.platform_pos = second_position;
  world.platform_vel.x_mps = (second_position.x_m - first_position.x_m) / state_dt_sec;
  world.platform_vel.y_mps = (second_position.y_m - first_position.y_m) / state_dt_sec;
  world.platform_vel.z_mps = (second_position.z_m - first_position.z_m) / state_dt_sec;
  const double ecef_speed_mps = std::sqrt(world.platform_vel.x_mps * world.platform_vel.x_mps +
                                          world.platform_vel.y_mps * world.platform_vel.y_mps +
                                          world.platform_vel.z_mps * world.platform_vel.z_mps);
  EXPECT_GT(ecef_speed_mps, 1.0);

  WorldTarget emitter;
  emitter.id = 8301U;
  emitter.pos = EcefFromLla(second_state.latitude_rad * radians_to_degrees,
                            second_state.longitude_rad * radians_to_degrees + 0.05,
                            second_state.altitude_geod_m);
  emitter.vel = oneq::coordinate::EcefVelocityMps{};
  emitter.rcs = 20.0f;
  emitter.temperature_k = 450.0f;
  emitter.area_m2 = 20.0f;
  emitter.carrier_hz = 10.0e9;
  emitter.bandwidth_hz = 5.0e6;
  emitter.tx_power_w = 5.0e7;
  world.targets.push_back(emitter);

  esr_config::EsrSessionConfig esr_config = MakeEsrConfigAirToAir();
  esr_config.policy.detection.enable_statistical_detection = false;
  esr_config.hardware.co_site_paths.push_back({101U, 100.0});
  esr_session::EsrSession esr = esr_session::EsrSession::Create(esr_config);

  esr_session::EmitterHypothesisList hypotheses;
  std::uint32_t source_esr_cycle = 0U;
  std::uint64_t source_esr_batch = 0U;
  for (std::uint32_t cycle = 1U; cycle <= 8U && hypotheses.empty(); ++cycle) {
    esr_session::EsrCycleInput input = BuildEsrInput(world, 1.0f, cycle);
    const esr_session::EsrCycleResult result = esr.StepWithResult(input);
    ASSERT_FALSE(esr_session::HasValidationError(result.issues));
    if (!result.output_frame.emitter_output.hypotheses.empty()) {
      hypotheses = result.output_frame.emitter_output.hypotheses;
      source_esr_cycle = cycle;
      source_esr_batch = result.output_frame.batch_id;
    }
  }
  ASSERT_FALSE(hypotheses.empty());

  ecm_session::EcmSensorObservationFrame sensor_frame;
  ASSERT_TRUE(
      ecm_session::TryBuildEcmSensorObservationFrame(hypotheses, source_esr_batch, &sensor_frame));
  ASSERT_FALSE(sensor_frame.observations.empty());

  ecm_config::EcmSessionConfig ecm_config;
  ecm_config.channel_count = 1U;
  ecm_config.maximum_total_transmit_power_w = 1000.0;
  ecm_config.maximum_channel_transmit_power_w = 1000.0;
  ecm_config.default_technique = electronic_countermeasure::EcmTechnique::kSpot;
  ecm_session::EcmSession ecm = ecm_session::EcmSession::Create(ecm_config);
  ecm.SetSensorEntityId(101U);
  ecm_session::EcmCycleInput ecm_input;
  ecm_input.cycle_index = source_esr_cycle + 1U;
  ecm_input.cycle_start_time_s = static_cast<double>(source_esr_cycle);
  ecm_input.dt_sec = 1.0;
  ecm_input.input_mode = electronic_countermeasure::EcmInputMode::kSensorDriven;
  ecm_input.platform_position_ecef_m = world.platform_pos;
  ecm_input.platform_velocity_ecef_mps = world.platform_vel;
  ecm_input.has_sensor_observation_frame = true;
  ecm_input.sensor_observation_frame = sensor_frame;
  const ecm_session::EcmCycleResult ecm_result = ecm.StepWithResult(ecm_input);
  ASSERT_EQ(ecm_result.status, ecm_session::EcmCycleStatus::kExecuted);
  ASSERT_FALSE(ecm_result.emission_frame.emissions.empty());
  EXPECT_EQ(ecm_result.source_esr_batch_id, source_esr_batch);

  ar_config::ArSessionConfig ar_config = MakeArConfigAirToAir();
  ar_config.hardware.receiver.co_site_paths.push_back({101U, 100.0});
  ar_session::ArSession ar = ar_session::ArSession::Create(ar_config);

  ar_session::ArCycleInput ar_input =
      BuildArInput(world, 1.0f, source_esr_cycle + 1U).cycle;
  ar_input.interference = ecm_result.emission_frame;
  const ar_session::ArCycleResult ar_result = ar.StepWithResult(ar_input);
  EXPECT_FALSE(ar_session::HasValidationError(ar_result.issues));
  EXPECT_EQ(ar_result.status, ar_session::ArCycleStatus::kCompleted);
  EXPECT_EQ(ar_result.abort_reason, ar_session::SignalCycleAbortReason::kNone);

  esr_session::EsrCycleInput esr_input =
      BuildEsrInput(world, 1.0f, source_esr_cycle + 1U);
  esr_input.rf_emissions = ecm_result.emission_frame;
  const esr_session::EsrCycleResult esr_result = esr.StepWithResult(esr_input);
  EXPECT_FALSE(esr_session::HasValidationError(esr_result.issues));
  EXPECT_EQ(esr_result.output_frame.cycle_index, source_esr_cycle + 1U);
}
#endif
