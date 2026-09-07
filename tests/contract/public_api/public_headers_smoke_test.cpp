/**
 * @file public_headers_smoke_test.cpp
 * @brief 验证稳定公共头集合可被统一包含并完成最小用法编译。
 */

#include <gtest/gtest.h>

#include <memory>
#include <type_traits>
#include <utility>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/airborne_radar/config/ArEnvironmentConfig.h"
#include "1q/airborne_radar/config/ArHardwareConfig.h"
#include "1q/airborne_radar/config/ArMissionConfig.h"
#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/airborne_radar/config/ArPolicyConfig.h"
#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/config/airborne_radar_config.hpp"
#include "1q/airborne_radar/session/ArCommand.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArInputValidation.h"
#include "1q/airborne_radar/session/ArOutputTypes.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/ArRecordingSession.h"
#include "1q/airborne_radar/session/ArTrackLifecycleRecorder.h"
#include "1q/airborne_radar/session/ArTrackOutputDebugView.h"
#include "1q/airborne_radar/session/DecisionControlTypes.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "1q/api.hpp"
#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/electronic_countermeasure/EcmEsrAdapter.h"
#include "1q/electronic_countermeasure/EcmReplaySession.h"
#include "1q/electronic_countermeasure/EcmSession.h"
#include "1q/electronic_countermeasure/EcmRecordingSession.h"
#include "1q/electronic_countermeasure/EcmTypes.h"
#include "1q/electro_optical_sensor/config/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/config/EosProfileConstants.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"
#include "1q/electro_optical_sensor/config/electro_optical_sensor_config.hpp"
#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosDetectionLifecycleRecorder.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosPlatformEcefPose.h"
#include "1q/electro_optical_sensor/session/EosOutputDebugView.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electro_optical_sensor/session/EosRecordingSession.h"
#include "1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrProfileConstants.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/config/electronic_surveillance_radar_config.hpp"
#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/electronic_surveillance_radar/session/EsrRecordingSession.h"
#include "1q/environment/AtmosphericTypes.h"
#include "1q/foundation/RuntimeCycleExecutor.h"
#include "1q/foundation/scan_schedule_types.h"
#include "1q/sar/config/SarEnvironmentConfig.h"
#include "1q/sar/config/SarHardwareConfig.h"
#include "1q/sar/config/SarMissionConfig.h"
#include "1q/sar/config/SarPolicyConfig.h"
#include "1q/sar/config/SarProfileConstants.h"
#include "1q/sar/config/SarRuntimeConfigPatch.h"
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/config/sar_config.hpp"
#include "1q/sar/sar.hpp"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "1q/sar/session/SarProductDebugView.h"
#include "1q/sar/session/SarProductLifecycleRecorder.h"
#include "1q/sar/session/SarReplaySession.h"
#include "1q/sar/session/SarSession.h"
#include "1q/sar/session/SarRecordingSession.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/config/sbirs_sensor_config.hpp"
#include "1q/sbirs_sensor/sbirs_sensor.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"
#include "1q/sbirs_sensor/session/SbirsExternalInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsInputValidation.h"
#include "1q/sbirs_sensor/session/SbirsOutputDebugView.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"
#include "1q/sbirs_sensor/session/SbirsReplaySession.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"
#include "1q/sbirs_sensor/session/SbirsRecordingSession.h"
#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/config/remote_identification_radar_config.hpp"
#include "1q/remote_identification_radar/remote_identification_radar.hpp"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirInputValidation.h"
#include "1q/remote_identification_radar/session/RirRecordingSession.h"
#include "1q/remote_identification_radar/session/RirReplaySession.h"
#include "1q/remote_identification_radar/session/RirSession.h"

using ArSession = airborne_radar::session::ArSession;
using ArConfig = airborne_radar::config::ArSessionConfig;
static_assert(!std::is_constructible<ArSession, const ArConfig&>::value,
              "ArSession direct construction must be disabled");
static_assert(std::is_same<ArSession, decltype(airborne_radar::session::ArSession::Create(
                                          std::declval<const ArConfig&>()))>::value,
              "ArSession::Create must return ArSession");
static_assert(
    !std::is_constructible<electronic_surveillance_radar::session::EsrSession,
                           electronic_surveillance_radar::config::EsrSessionConfig>::value,
    "EsrSession direct construction must be disabled");

static_assert(!std::is_constructible<electro_optical_sensor::session::EosSession,
                                     electro_optical_sensor::config::EosSessionConfig>::value,
              "EosSession direct construction must be disabled");
static_assert(
    std::is_same<
        electro_optical_sensor::session::EosSession,
        decltype(electro_optical_sensor::session::EosSession::Create(
            std::declval<const electro_optical_sensor::config::EosSessionConfig&>()))>::value,
    "EosSession::Create must return EosSession");

static_assert(
    !std::is_constructible<sar::session::SarSession, sar::config::SarSessionConfig>::value,
    "SarSession direct construction must be disabled");
static_assert(std::is_same<sar::session::SarSession,
                           decltype(sar::session::SarSession::Create(
                               std::declval<const sar::config::SarSessionConfig&>()))>::value,
              "SarSession::Create must return SarSession");

static_assert(!std::is_constructible<sbirs_sensor::session::SbirsSession,
                                     sbirs_sensor::config::SbirsSessionConfig>::value,
              "SbirsSession direct construction must be disabled");
static_assert(
    std::is_same<sbirs_sensor::session::SbirsSession,
                 decltype(sbirs_sensor::session::SbirsSession::Create(
                     std::declval<const sbirs_sensor::config::SbirsSessionConfig&>()))>::value,
    "SbirsSession::Create must return SbirsSession");

static_assert(
    !std::is_constructible<remote_identification_radar::session::RirSession,
                           remote_identification_radar::config::RirSessionConfig>::value,
    "RirSession direct construction must be disabled");
static_assert(
    std::is_same<
        remote_identification_radar::session::RirSession,
        decltype(remote_identification_radar::session::RirSession::Create(
            std::declval<const remote_identification_radar::config::RirSessionConfig&>()))>::value,
    "RirSession::Create must return RirSession");

namespace airborne_radar {
namespace {

TEST(PublicHeadersSmokeTest, StablePublicSurfaceSupportsMinimalUsage) {
  oneq::coordinate::LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 0.0;
  origin_lla.longitude_deg = 0.0;
  origin_lla.altitude_m = 0.0;
  oneq::coordinate::EcefPositionM origin_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(origin_lla, &origin_ecef));

  config::ArSessionConfig session_config;
  session_config.environment.scenario_config.atmospheric_physics.enable_physical_model = true;

  session::ArCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 1.0;
  input.platform.platform_position_ecef_m = origin_ecef;
  const std::vector<session::ArIssue> issues = session::ValidateArCycleInput(input);

  EXPECT_FALSE(session::HasValidationError(issues));

  session::ArSession session = session::ArSession::Create(session_config);
  session::ArRecordingSession trace_session(session_config,
                                        session::ArRecordingSessionOptions{});
  config::ArRuntimeConfigPatch runtime_patch;
  runtime_patch.has_work_mode = true;
  runtime_patch.work_mode = config::ArWorkMode::kTas;
  runtime_patch.has_commanded_beamwidth_enabled = true;
  runtime_patch.commanded_beamwidth_enabled = true;
  (void)session.TryApplyRuntimeConfig(runtime_patch);
  const session::ArCycleResult result = session.StepWithResult(input);
  const session::ArCycleResult trace_result =
      trace_session.StepWithResult(input);
  const std::size_t confirmed_tracks =
      session::CountTracksByStatus(result.output_frame, session::TrackStatus::kConfirmed);

  EXPECT_GE(confirmed_tracks, 0U);
  EXPECT_GE(result.association_quality_metrics.detection_count, 0U);
  EXPECT_EQ(trace_result.status, session::ArCycleStatus::kCompleted);
}

TEST(PublicHeadersSmokeTest, SbirsPublicSurfaceSupportsMinimalUsage) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.mission.scan_start_az_deg = 170.0f;
  config.mission.scan_span_deg = 40.0f;
  config.mission.scan_direction = sbirs_sensor::config::SbirsScanDirection::kDecreasingAzimuth;
  sbirs_sensor::session::SbirsSession session = sbirs_sensor::session::SbirsSession::Create(config);

  sbirs_sensor::session::SbirsVector3M satellite;
  satellite.x = 7000000.0;
  satellite.y = 0.0;
  satellite.z = 0.0;
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m.x = 8000000.0;
  target.position_ecef_m.y = 0.0;
  target.position_ecef_m.z = 0.0;
  target.radiant_intensity_w_per_sr = 1.0e8;

  sbirs_sensor::session::SbirsCycleInput input = sbirs_sensor::session::SbirsCycleInputBuilder()
                                                     .WithCycleIndex(1U)
                                                     .WithDeltaTimeSec(1.0f)
                                                     .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                                                     .WithSatellitePosition(satellite)
                                                     .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                                                     .AddTarget(target)
                                                     .Build();
  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_FALSE(sbirs_sensor::session::HasValidationError(issues));

  sbirs_sensor::config::SbirsRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = sbirs_sensor::config::SbirsWorkMode::kSearchAndStare;
  EXPECT_TRUE(session.TryApplyRuntimeConfig(patch));
  const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, sbirs_sensor::session::SbirsCycleStatus::kCompleted);
  EXPECT_TRUE(sbirs_sensor::session::SbirsOutputFrameContainsOnlyNativeFields(result.output_frame));
}

TEST(PublicHeadersSmokeTest, FiveDomainHeadersDefineIndependentConfigTypes) {
  config::ArHardwareConfig hardware{};
  hardware.transmitter.peak_power_w = 2.0e6f;
  EXPECT_FLOAT_EQ(hardware.transmitter.peak_power_w, 2.0e6f);

  config::AzimuthElevationDeg scan_center;
  scan_center.az_deg = 45.0f;
  scan_center.el_deg = -5.0f;
  config::ArMissionConfig mission{};
  mission.scan_center_deg = scan_center;
  EXPECT_FLOAT_EQ(mission.scan_center_deg.az_deg, 45.0f);

  config::ArOrientationConfig orientation{};
  orientation.stabilization_mode = config::StabilizationMode::kBodyStabilized;
  EXPECT_EQ(orientation.stabilization_mode, config::StabilizationMode::kBodyStabilized);

  config::ArPolicyConfig policy{};
  policy.detection.pulse_count = 32;
  policy.lifecycle.confirm_hits = 2U;
  EXPECT_EQ(policy.detection.pulse_count, 32);
  EXPECT_EQ(policy.lifecycle.confirm_hits, 2U);

  config::ArEnvironmentConfig env{};
  env.scenario_config.atmospheric_physics.enable_physical_model = true;
  EXPECT_TRUE(env.scenario_config.atmospheric_physics.enable_physical_model);

  config::ArSessionConfig session_cfg;
  session_cfg.hardware = hardware;
  session_cfg.mission = mission;
  session_cfg.orientation = orientation;
  session_cfg.policy = policy;
  session_cfg.environment = env;
  EXPECT_EQ(session_cfg.policy.detection.pulse_count, 32);
  EXPECT_FLOAT_EQ(session_cfg.mission.scan_center_deg.az_deg, 45.0f);
  EXPECT_EQ(session_cfg.orientation.stabilization_mode,
            config::StabilizationMode::kBodyStabilized);
  EXPECT_EQ(session_cfg.policy.lifecycle.confirm_hits, 2U);
}

TEST(PublicHeadersSmokeTest, RadarSessionBuilderCanConfigureLeafAndDomainFields) {
  config::AzimuthElevationDeg scan_center;
  scan_center.az_deg = -12.0f;
  scan_center.el_deg = 6.0f;

  config::ArSessionConfig session_config;
  session_config.policy.detection = config::profiles::kDetectionPriorityDetection;
  session_config.hardware = config::profiles::kLongRangeHighPowerHardware;
  session_config.hardware.antenna.pattern = config::profiles::kLowSidelobeAntennaPattern;
  session_config.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
  session_config.mission.scan_center_deg = scan_center;
  EXPECT_EQ(session_config.policy.detection.pulse_count, 16);
  EXPECT_FLOAT_EQ(session_config.hardware.transmitter.peak_power_w, 5.0e6f);
  EXPECT_FLOAT_EQ(session_config.hardware.antenna.pattern.max_sidelobe_level_db, -30.0f);
  EXPECT_FLOAT_EQ(session_config.mission.scan_center_deg.az_deg, -12.0f);
  EXPECT_EQ(session_config.policy.lifecycle.confirm_hits, 1U);
  config::EnvironmentScenarioConfig scenario;
  scenario.atmospheric_physics.enable_physical_model = true;
  config::ArEnvironmentConfig env;
  env.scenario_config = scenario;
  EXPECT_TRUE(env.scenario_config.atmospheric_physics.enable_physical_model);
}

}  // namespace
}  // namespace airborne_radar

namespace electronic_surveillance_radar {
namespace {

TEST(PublicHeadersSmokeTest, EsrPublicSurfaceSupportsMinimalUsage) {
  const oneq::foundation::ScanStartPosition shared_start =
      oneq::foundation::ScanStartPosition::kLeftTop;
  EXPECT_EQ(static_cast<int>(shared_start), 0);

  config::EsrSessionConfig session_config;
  session_config.mission = config::profiles::kElectronicOrderOfBattleMission;
  // 演示非 no-op 灵敏度常量（kStandard 与 struct 默认一致，无需赋值）。
  session_config.policy.detection = config::profiles::kHighSensitivityDetection;
  session_config.mission.scan.scan_rate_hz = 1.0f;
  session_config.mission.scan.use_explicit_scan_bounds = true;
  session_config.mission.scan.scan_start_az_deg = -60.0f;
  session_config.mission.scan.scan_end_az_deg = 60.0f;
  session_config.mission.scan.scan_start_el_deg = -20.0f;
  session_config.mission.scan.scan_end_el_deg = 20.0f;
  session_config.hardware.beam_az_width_deg = 120.0f;
  session_config.hardware.beam_el_width_deg = 40.0f;

  session::EsrCycleInput input;
  input.cycle_index = 4U;
  input.dt_sec = 1.0f;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;

  const session::EsrIssueList issues = session::ValidateEsrCycleInput(input);
  EXPECT_FALSE(session::HasValidationError(issues));

  auto session = session::EsrSession::Create(session_config);
  config::EsrRuntimeConfigPatch runtime_patch;
  runtime_patch.has_work_mode = true;
  runtime_patch.work_mode = config::EsrWorkMode::kRwr;
  (void)session.TryApplyRuntimeConfig(runtime_patch);
  const session::EsrCycleResult result = session.StepWithResult(input);
  session::EsrRecordingSession trace_session(session_config, session::EsrRecordingSessionOptions{});
  const session::EsrCycleResult trace_result = trace_session.StepWithResult(input);

  EXPECT_GE(result.output_frame.observation_output.observations.size(), 0U);
  EXPECT_GE(result.output_frame.emitter_output.hypotheses.size(), 0U);
  EXPECT_GE(trace_result.output_frame.emitter_output.hypotheses.size(), 0U);
}

}  // namespace
}  // namespace electronic_surveillance_radar

namespace electro_optical_sensor {
namespace {

TEST(PublicHeadersSmokeTest, EosPublicSurfaceSupportsMinimalUsage) {
  config::EosSessionConfig session_config;
  session_config.mission = config::profiles::kWideAreaSearchMission;
  session_config.policy.detection.minimum_snr_db = 4.5f;
  session_config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  session_config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;
  session_config.mission.scan_start_az_deg = -20.0f;
  session_config.mission.scan_end_az_deg = 20.0f;

  ::electro_optical_sensor::session::EosCycleInput input;
  input.cycle_index = 2U;
  input.dt_sec = 0.1f;
  session::EosSceneTarget target;
  target.target_id = 7U;
  target.position_x = 1500.0f;  // 平台锚点 ENU（东向），零姿态下斜距 = 1500 m
  target.appearance.apparent_temperature_k = 320.0f;
  target.appearance.emissivity = 0.9f;
  target.appearance.reflectance = 0.4f;
  target.appearance.projected_area_m2 = 2.0f;
  input.scene.push_back(target);

  // 平台 ECEF 位姿类型可构造（场景目标 ENU 由公共 TryMakeEnuSceneState 直填）。
  oneq::coordinate::LlaPositionDegM eos_origin_lla;
  eos_origin_lla.latitude_deg = 0.0;
  eos_origin_lla.longitude_deg = 0.0;
  eos_origin_lla.altitude_m = 0.0;
  oneq::coordinate::EcefPositionM eos_origin_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(eos_origin_lla, &eos_origin_ecef));
  session::EosPlatformEcefPose eos_platform_pose;
  eos_platform_pose.platform_position_ecef_m = eos_origin_ecef;
  EXPECT_NEAR(eos_platform_pose.platform_position_ecef_m.x_m, eos_origin_ecef.x_m, 1.0e-6);

  const session::EosIssueList issues = session::ValidateEosCycleInput(input, 30.0f);
  EXPECT_FALSE(session::HasValidationError(issues));

  // 环境 preset 作为唯一公开模型选择入口。
  EXPECT_NE(static_cast<int>(config::EosEnvironmentPreset::kStandard), -1);

  session::EosSession session = session::EosSession::Create(session_config);
  config::EosRuntimeConfigPatch runtime_patch;
  runtime_patch.has_frame_rate_hz = true;
  runtime_patch.frame_rate_hz = 15.0f;
  (void)session.TryApplyRuntimeConfig(runtime_patch);
  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);
  session::EosRecordingSession trace_session(session_config, session::EosRecordingSessionOptions{});
  const ::electro_optical_sensor::session::EosCycleResult trace_result =
      trace_session.StepWithResult(input);
  EXPECT_EQ(result.status, ::electro_optical_sensor::session::EosCycleStatus::kCompleted);
  EXPECT_EQ(trace_result.status, ::electro_optical_sensor::session::EosCycleStatus::kCompleted);
  EXPECT_GE(result.output_frame.detections.size(), 0U);
  EXPECT_GE(trace_result.output_frame.detections.size(), 0U);
}

}  // namespace
}  // namespace electro_optical_sensor

namespace sar {
namespace {

TEST(PublicHeadersSmokeTest, SarPublicSurfaceSupportsMinimalUsage) {
  config::SarSessionConfig session_config;
  session_config.hardware.carrier_frequency_hz = 1.0e9;
  session_config.hardware.bandwidth_hz = 25.0e6;
  session_config.hardware.pulse_width_s = 0.16e-6;
  session_config.hardware.pulse_repetition_frequency_hz = 20.0;
  session_config.hardware.sample_rate_hz = 100.0e6;
  session_config.mission.nominal_slant_range_m = 29.9792458;
  session_config.mission.scene_center_latitude_deg =
      29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  session_config.mission.platform_speed_mps = 2.0;
  session_config.mission.range_sample_count = 64U;
  session_config.mission.azimuth_pulse_count = 9U;
  session_config.policy.enable_l1_rda_imaging = true;
  EXPECT_FALSE(session_config.policy.enable_l2_motion_compensation);
  EXPECT_FALSE(session_config.policy.enable_l3_bp_imaging);
  EXPECT_DOUBLE_EQ(session_config.mission.l2_velocity_error_stddev_y_mps, 0.0);
  config::SarWaypointConfig waypoint;
  waypoint.time_from_session_start_s = 0.0;
  session_config.mission.l3_waypoints.push_back(waypoint);
  EXPECT_EQ(session_config.mission.l3_waypoints.size(), 1U);
  EXPECT_EQ(session_config.mission.range_sample_count, 64U);

  session::SarCycleInput input;
  input.cycle_index = 8U;
  input.dt_sec = 0.5f;
  input.platform.altitude_m = 0.0;
  session::SarPointTarget target;
  target.latitude_deg = 29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  target.longitude_deg = 0.0;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);
  session::SarRawIqFrame raw_iq;
  raw_iq.samples_per_pulse = 1U;
  raw_iq.i_values.push_back(1.0);
  raw_iq.q_values.push_back(0.0);
  session::SarRawIqFrame::PulseState pulse_state;
  pulse_state.pulse_id = 0U;
  raw_iq.pulse_states.push_back(pulse_state);
  raw_iq.ideal_pulse_states.push_back(pulse_state);
  EXPECT_EQ(raw_iq.i_values.size(), 1U);
  EXPECT_EQ(raw_iq.pulse_states.size(), 1U);
  EXPECT_EQ(raw_iq.ideal_pulse_states.size(), 1U);

  session::SarSession session = session::SarSession::Create(session_config);
  config::SarRuntimeConfigPatch patch;
  patch.has_retain_focused_image = true;
  patch.retain_focused_image = true;
  EXPECT_TRUE(session.TryApplyRuntimeConfig(patch));

  const session::SarCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted)
      << static_cast<int>(result.abort_reason);
  EXPECT_EQ(result.product.output_frame.range_sample_count, 64U);
  EXPECT_TRUE(result.product.output_frame.has_raw_echo);
  EXPECT_TRUE(result.product.output_frame.has_range_compressed_echo);
  EXPECT_TRUE(result.product.output_frame.has_l1_image);
  EXPECT_FALSE(result.product.output_frame.has_l3_bp_image);
  EXPECT_EQ(result.product.focused_image.source, session::SarFocusedImageSource::kL1Rda);
  EXPECT_EQ(result.product.focused_image.row_count, 9U);
  EXPECT_EQ(result.product.focused_image.column_count, 64U);
  EXPECT_EQ(result.product.focused_image.real_values.size(), 9U * 64U);
  EXPECT_EQ(result.product.focused_image.imaginary_values.size(), 9U * 64U);
  EXPECT_FALSE(result.product.focused_image.is_placeholder);

  session::SarRecordingSession trace_session(session::SarSession::Create(session_config));
  const session::SarCycleResult trace_result = trace_session.StepWithResult(input);
  EXPECT_EQ(trace_result.status, session::SarCycleStatus::kCompleted);

  session::SarReplaySessionResult replay_result;
  EXPECT_FALSE(replay_result.ok);
}

}  // namespace
}  // namespace sar

namespace remote_identification_radar {
namespace {

TEST(PublicHeadersSmokeTest, RirPublicSurfaceSupportsMinimalUsage) {
  config::RirSessionConfig session_config;
  session_config.mission.work_mode = config::RirWorkMode::kIdentify;
  session_config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  session_config.policy.lifecycle.confirm_hits = 1U;

  session::RirCycleInput input;
  input.input_cycle_index = 1U;
  input.dt_sec = 0.5;
  input.sim_time_sec = 0.0f;
  oneq::coordinate::LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 30.0;
  origin_lla.longitude_deg = 120.0;
  origin_lla.altitude_m = 1000.0;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(origin_lla, &input.platform_position));

  const session::RirIssueList issues =
      session::ValidateRirCycleInput(input, session_config.mission.recognition_dwell_sec);
  EXPECT_FALSE(session::HasValidationError(issues));

  session::RirSession session = session::RirSession::Create(session_config);
  session::RirRecordingSession recording_session(session_config,
                                                session::RirRecordingSessionOptions{});
  const session::RirCycleResult result = session.StepWithResult(input);
  const session::RirCycleResult recorded = recording_session.StepWithResult(input);
  EXPECT_EQ(result.status, session::RirCycleStatus::kCompleted);
  EXPECT_EQ(recorded.status, session::RirCycleStatus::kCompleted);

  session::RirReplaySessionResult replay_result;
  EXPECT_FALSE(replay_result.ok);
}

}  // namespace
}  // namespace remote_identification_radar
