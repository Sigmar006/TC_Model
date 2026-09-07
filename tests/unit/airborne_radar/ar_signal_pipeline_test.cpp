// Copyright 2026. All Rights Reserved.
//
// @file ar_signal_pipeline_test.cpp
// @brief 验证信号处理流水线与内部配置的基础行为。

#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <memory>
#include <vector>

#include "1q/airborne_radar/config/ArHardwareConfig.h"
#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/config/InternalExecutionConfig.h"
#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/ControlProfileEffects.h"
#include "airborne_radar/signal/pipeline/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"
#include "airborne_radar/signal/pipeline/SignalCycleInput.h"

namespace airborne_radar {
namespace tests {

namespace {

using ExecutionConfig = config::execution::InternalExecutionConfig;

config::ArSessionConfig MakeDetectionFocusedConfig() {
  config::ArSessionConfig session_config;
  session_config.policy.detection = config::profiles::kDetectionPriorityDetection;
  session_config.policy.tracking = config::profiles::kFastAssociationTracking;
  session_config.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
  return session_config;
}

config::ArSessionConfig MakeTrackingFocusedConfig() {
  config::ArSessionConfig session_config;
  session_config.policy.detection = config::profiles::kTrackStabilityPriorityDetection;
  return session_config;
}

config::ArSessionConfig MakeRobustTrackingConfig() {
  config::ArSessionConfig session_config;
  session_config.policy.detection = config::profiles::kTrackStabilityPriorityDetection;
  session_config.policy.tracking = config::profiles::kRobustAntiJammingTracking;
  session_config.policy.association = config::profiles::kRobustAntiJammingAssociation;
  session_config.policy.lifecycle = config::profiles::kHighPersistenceLifecycle;
  return session_config;
}

float SpeedOf(const session::ArSceneTarget& target) {
  return std::sqrt(target.velocity_x * target.velocity_x + target.velocity_y * target.velocity_y +
                   target.velocity_z * target.velocity_z);
}

session::ArSceneTarget BuildPhysicsTarget(float range_m, float rcs) {
  session::ArSceneTarget target(220.0f, 0.0f, 0.0f, rcs, 0.0f, 0.0f, 0.0f);

  target.position_x = range_m;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = range_m;
  return target;
}

signal::pipeline::RfV2DetectionContext MakeRfV2DetectionContext(double jammer_power_w) {
  signal::pipeline::RfV2DetectionContext context;
  context.own_emission_identity = oneq::electromagnetics::RfEmissionIdentity{1U, 1U};
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      0.0, 3.0e9, 4.5e6, 1.0e6, 13.0e-6, 1.0 / 300.0, 300U, 0.0, 7U, 0U,
      &context.own_transmit_waveform));
  context.receive_window_start_time_s = 0.0;
  context.receive_window_duration_s = 1.0;
  oneq::electromagnetics::RfIncidentLinkResult jammer;
  jammer.identity = oneq::electromagnetics::RfEmissionIdentity{2U, 2U};
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      0.0, 1.0, 3.0e9, 4.5e6, 1.0, &jammer.emission_waveform));
  jammer.receiver_sensor_entity_id = 1U;
  jammer.received_power_before_overlap_w = jammer_power_w;
  jammer.received_power_w = jammer_power_w;
  context.incident_links.push_back(jammer);
  return context;
}

session::ArSceneTarget ToSceneTarget(const session::ArSceneTarget& target) {
  session::ArSceneTarget out;
  out.external_target_id = target.external_target_id;
  out.velocity_x = target.velocity_x;
  out.velocity_y = target.velocity_y;
  out.velocity_z = target.velocity_z;
  out.rcs = target.rcs;
  out.range_m = target.range_m;
  out.position_x = target.position_x;
  out.position_y = target.position_y;
  out.position_z = target.position_z;
  out.target_swerling_type = target.target_swerling_type;
  return out;
}

session::ArSceneTargetList ToSceneTargets(const session::ArSceneTargetList& targets) {
  session::ArSceneTargetList out;
  out.reserve(targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    out.push_back(ToSceneTarget(targets[i]));
  }
  return out;
}

session::ArSceneTarget CloneSceneTarget(const session::ArSceneTarget& target) {
  session::ArSceneTarget out;
  out.external_target_id = target.external_target_id;
  out.velocity_x = target.velocity_x;
  out.velocity_y = target.velocity_y;
  out.velocity_z = target.velocity_z;
  out.rcs = target.rcs;
  out.range_m = target.range_m;

  out.position_x = target.position_x;
  out.position_y = target.position_y;
  out.position_z = target.position_z;
  out.target_swerling_type = target.target_swerling_type;
  return out;
}

session::ArSceneTargetList CloneSceneTargets(const session::ArSceneTargetList& targets) {
  session::ArSceneTargetList out;
  out.reserve(targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    out.push_back(CloneSceneTarget(targets[i]));
  }
  return out;
}

signal::tracking::CycleContext MakeLifecycleCycle(std::uint32_t cycle_index,
                                                  std::uint64_t batch_id) {
  signal::tracking::CycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.batch_id = batch_id;
  cycle.dt_sec = 1.0f;
  cycle.extra_miss_tolerance = 0u;
  return cycle;
}

session::EnvironmentCycleContext MakeEnvironmentCycle(std::uint32_t cycle_index) {
  session::EnvironmentCycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.dt_sec = 1.0f;
  return cycle;
}

template <typename PipelineType>
session::SignalCycleResult RunPipelineCycle(PipelineType* pipeline,
                                            const session::ArSceneTargetList& input_state,
                                            environment::EnvironmentService* environment_service,
                                            std::uint32_t cycle_index = 1u) {
  environment_service->BeginCycle(MakeEnvironmentCycle(cycle_index));
  return pipeline->RunCycle(
      signal::pipeline::SignalCycleInput{ToSceneTargets(input_state)}, *environment_service);
}

}  // namespace

TEST(SignalPipelineTest, KeepsTrackStableWhenDetectionMarginIsEnough) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  session::ArSceneTarget target(800.0f, 0.0f, 0.0f, 2.5f, 0.0f, 0.0f, 0.0f);

  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const session::ArSceneTargetList input_state{target};

  const auto output_state = CloneSceneTargets(
      RunPipelineCycle(&signal_pipeline, input_state, &environment_service).updated_scene_targets);

  ASSERT_EQ(output_state.size(), 1u);
  EXPECT_FLOAT_EQ(SpeedOf(output_state[0]), SpeedOf(input_state[0]));
  EXPECT_FLOAT_EQ(output_state[0].rcs, input_state[0].rcs);
}

TEST(SignalPipelineTest, PhysicalRcsUsesTransmitterFrequency) {
  config::ArSessionConfig low_frequency_config;
  low_frequency_config.hardware = config::profiles::kLongRangeHighPowerHardware;
  low_frequency_config.hardware.rcs_physics = config::profiles::kEnhancedRcsPhysics;
  low_frequency_config.hardware.transmitter.frequency_hz = 1.0e9f;
  config::ArSessionConfig high_frequency_config = low_frequency_config;
  high_frequency_config.hardware.transmitter.frequency_hz = 3.0e9f;

  const session::ArSceneTargetList input_state{BuildPhysicsTarget(4500.0f, 0.2f)};
  config::EnvironmentScenarioConfig environment_config;
  environment::EnvironmentService low_frequency_environment(environment_config);
  environment::EnvironmentService high_frequency_environment(environment_config);
  signal::pipeline::SignalPipeline low_frequency_pipeline(low_frequency_config);
  signal::pipeline::SignalPipeline high_frequency_pipeline(high_frequency_config);

  RunPipelineCycle(&low_frequency_pipeline, input_state, &low_frequency_environment);
  RunPipelineCycle(&high_frequency_pipeline, input_state, &high_frequency_environment);
  const auto low_frequency_measurements = low_frequency_pipeline.GetLastTrackMeasurements();
  const auto high_frequency_measurements = high_frequency_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(low_frequency_measurements.size(), 1U);
  ASSERT_EQ(high_frequency_measurements.size(), 1U);
  EXPECT_NE(low_frequency_measurements[0].raw_measurement.detection_margin_db,
            high_frequency_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest, ExposesPublicPlatformAttitudeUpdateApi) {
  signal::pipeline::SignalPipeline signal_pipeline;
  config::PlatformAttitudeDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 12.0f;
  platform_attitude_deg.pitch_deg = -3.0f;
  platform_attitude_deg.roll_deg = 1.5f;

  signal_pipeline.UpdatePlatformAttitude(platform_attitude_deg);

  const config::PlatformAttitudeDeg cached_platform_attitude =
      signal_pipeline.GetPlatformAttitude();
  EXPECT_FLOAT_EQ(cached_platform_attitude.yaw_deg, 12.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.pitch_deg, -3.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.roll_deg, 1.5f);
}

TEST(SignalPipelineTest, ExposesPublicPlatformAltitudeUpdateApi) {
  signal::pipeline::SignalPipeline signal_pipeline;

  signal_pipeline.UpdatePlatformAltitudeM(1200.0f);

  EXPECT_FLOAT_EQ(signal_pipeline.GetPlatformAltitudeM(), 1200.0f);
}

TEST(SignalPipelineTest, UsesEnvironmentCycleIndexForExecutionContract) {
  signal::pipeline::SignalPipeline signal_pipeline;
  environment::EnvironmentService environment_service;
  session::ArSceneTarget target = BuildPhysicsTarget(1500.0f, 4.0f);
  target.external_target_id = 42U;

  const session::SignalCycleResult result = RunPipelineCycle(
      &signal_pipeline, session::ArSceneTargetList{target}, &environment_service, 77U);

  ASSERT_TRUE(result.executed_this_cycle);
  EXPECT_EQ(result.decision_frame.cycle_index, 77U);
}

TEST(SignalPipelineTest, RestoreRuntimeStatePreservesLifecycleTracks) {
  signal::pipeline::SignalPipeline signal_pipeline;
  environment::EnvironmentService environment_service;
  session::ArSceneTarget target = BuildPhysicsTarget(1500.0f, 4.0f);
  target.external_target_id = 43U;

  const session::SignalCycleResult baseline = RunPipelineCycle(
      &signal_pipeline, session::ArSceneTargetList{target}, &environment_service, 1U);
  ASSERT_TRUE(baseline.executed_this_cycle);
  ASSERT_FALSE(baseline.decision_frame.tracks.empty());

  const signal::SignalPipelineRuntimeState snapshot = signal_pipeline.CaptureRuntimeState();

  const session::SignalCycleResult missed_once =
      RunPipelineCycle(&signal_pipeline, session::ArSceneTargetList(), &environment_service, 2U);
  ASSERT_TRUE(missed_once.executed_this_cycle);
  ASSERT_FALSE(missed_once.decision_frame.tracks.empty());
  EXPECT_EQ(missed_once.decision_frame.tracks[0].miss_count, 1U);

  signal_pipeline.RestoreRuntimeState(snapshot);

  const session::SignalCycleResult missed_after_restore =
      RunPipelineCycle(&signal_pipeline, session::ArSceneTargetList(), &environment_service, 2U);
  ASSERT_TRUE(missed_after_restore.executed_this_cycle);
  ASSERT_FALSE(missed_after_restore.decision_frame.tracks.empty());
  EXPECT_EQ(missed_after_restore.decision_frame.tracks[0].miss_count, 1U);
}

TEST(SignalPipelineTest, AutoLifecycleManagerBuildsWithDefaultInternalImmConfig) {
  config::ArSessionConfig session_runtime_config;
  session_runtime_config.policy.lifecycle.enable_imm_lifecycle = true;
  const ExecutionConfig exec_config =
      config::mapping::MapSessionToExecution(session_runtime_config);

  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::CreateAutoLifecycleManagerForRuntimeConfig(exec_config);
  ASSERT_TRUE(lifecycle_manager != nullptr);

  const signal::tracking::CycleContext cycle = MakeLifecycleCycle(1u, 7u);
  const std::vector<signal::tracking::TrackMeasurement> measurements;
  lifecycle_manager->Update(cycle, measurements);

  session::DecisionInputFrame decision_frame(lifecycle_manager->BuildTrackStateSnapshots());
  decision_frame.cycle_index = 1u;
  decision_frame.batch_id = 7u;
  EXPECT_EQ(decision_frame.cycle_index, 1u);
  EXPECT_EQ(decision_frame.batch_id, 7u);
}

TEST(SignalPipelineTest, AutoLifecycleManagerCreationFailsWhenImmAssemblyIsInvalid) {
  config::ArSessionConfig session_runtime_config;
  session_runtime_config.policy.tracking.enable_kalman_filter = true;
  session_runtime_config.policy.lifecycle.enable_imm_lifecycle = true;
  ExecutionConfig exec_config = config::mapping::MapSessionToExecution(session_runtime_config);
  exec_config.lifecycle.imm_model_noise_diff_coeffs = {0.5f, 2.0f};
  exec_config.lifecycle.imm_initial_weights = {1.0f};

  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::CreateAutoLifecycleManagerForRuntimeConfig(exec_config);

  EXPECT_EQ(lifecycle_manager, nullptr);
}

TEST(SignalPipelineTest, ControlProfilePowerReductionLowersPhysicalDetectionMargin) {
  config::ArSessionConfig session_config;
  session_config.policy.detection = config::profiles::kDetectionPriorityDetection;

  environment::EnvironmentService environment_service;

  const session::ArSceneTargetList input_state{BuildPhysicsTarget(200.0f, 10000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(session_config);
  RunPipelineCycle(&baseline_pipeline, input_state, &environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  session::ArControlProfile reduced_power_profile;
  reduced_power_profile.enable_lpi_power_control = true;
  reduced_power_profile.lpi_power_scale = 0.20f;
  signal::pipeline::SignalPipeline reduced_pipeline(session_config);
  reduced_pipeline.SetControlProfile(reduced_power_profile);
  RunPipelineCycle(&reduced_pipeline, input_state, &environment_service);
  const auto reduced_measurements = reduced_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(reduced_measurements.size(), 1u);
  EXPECT_LT(reduced_measurements[0].raw_measurement.detection_margin_db,
            baseline_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest, AdaptiveBeamformingProfileTightensMeasurementCovariance) {
  config::ArSessionConfig session_config;
  session_config.policy.detection = config::profiles::kDetectionPriorityDetection;
  session_config.hardware.antenna.pattern.model_type =
      config::AntennaPatternModelType::kParabolicMainLobe;
  session_config.hardware.antenna.pattern.max_sidelobe_level_db = -18.0f;
  session_config.hardware.antenna.pattern.max_scan_loss_db = 8.0f;

  environment::EnvironmentService environment_service;

  const session::ArSceneTargetList input_state{BuildPhysicsTarget(200.0f, 10000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(session_config);
  RunPipelineCycle(&baseline_pipeline, input_state, &environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  session::ArControlProfile adaptive_profile;
  adaptive_profile.enable_adaptive_beamforming = true;
  signal::pipeline::SignalPipeline adaptive_pipeline(session_config);
  adaptive_pipeline.SetControlProfile(adaptive_profile);
  RunPipelineCycle(&adaptive_pipeline, input_state, &environment_service);
  const auto adaptive_measurements = adaptive_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(adaptive_measurements.size(), 1u);
  EXPECT_LT(adaptive_measurements[0].raw_measurement.measurement_covariance(1, 1),
            baseline_measurements[0].raw_measurement.measurement_covariance(1, 1));
  EXPECT_LT(adaptive_measurements[0].raw_measurement.measurement_covariance(2, 2),
            baseline_measurements[0].raw_measurement.measurement_covariance(2, 2));
}

TEST(SignalPipelineTest, AgilityFrequencyHopPhaseControlsFrequencyDirection) {
  config::ArSessionConfig phase_zero_config;
  config::ArSessionConfig phase_one_config = phase_zero_config;
  ExecutionConfig phase_zero_exec = config::mapping::MapSessionToExecution(phase_zero_config);
  ExecutionConfig phase_one_exec = config::mapping::MapSessionToExecution(phase_one_config);
  phase_zero_exec.detection.engineering.transmitter.frequency_hz = 1.0e9f;
  phase_one_exec.detection.engineering.transmitter.frequency_hz = 1.0e9f;

  session::ArControlProfile profile;
  profile.enable_agility_frequency = true;
  profile.agility_frequency_hop_phase = 0U;
  signal::pipeline::ApplyControlProfileToConfig(profile, &phase_zero_exec);
  profile.agility_frequency_hop_phase = 1U;
  signal::pipeline::ApplyControlProfileToConfig(profile, &phase_one_exec);

  EXPECT_FLOAT_EQ(phase_zero_exec.detection.engineering.transmitter.frequency_hz, 1.015e9f);
  EXPECT_FLOAT_EQ(phase_one_exec.detection.engineering.transmitter.frequency_hz, 0.985e9f);
}

TEST(SignalPipelineTest, AutoLifecycleAssemblyUsesControlProfileAdjustedKalmanUpdater) {
  config::ArSessionConfig session_config;
  session_config.policy.detection = config::profiles::kDetectionPriorityDetection;
  session_config.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
  session_config.policy.tracking.enable_kalman_filter = true;

  environment::EnvironmentService environment_service;

  session::ArSceneTarget target(1.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f);

  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const session::ArSceneTargetList cycle_1_input{target};

  signal::pipeline::SignalPipeline baseline_pipeline(session_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> baseline_manager =
      baseline_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(baseline_manager != nullptr);
  RunPipelineCycle(&baseline_pipeline, cycle_1_input, &environment_service, 1u);
  baseline_manager->Update(MakeLifecycleCycle(1u, 1u),
                           baseline_pipeline.GetLastTrackMeasurements());

  target.position_x = 101.0f;
  target.range_m = 101.0f;
  const session::ArSceneTargetList cycle_2_input{target};
  baseline_pipeline.SetAssociationSeeds(baseline_manager->BuildAssociationSeeds());
  RunPipelineCycle(&baseline_pipeline, cycle_2_input, &environment_service, 2u);
  baseline_manager->Update(MakeLifecycleCycle(2u, 2u),
                           baseline_pipeline.GetLastTrackMeasurements());
  const std::vector<signal::tracking::AssociationTrackSeed> baseline_seeds =
      baseline_manager->BuildAssociationSeeds();

  session::ArControlProfile adaptive_profile;
  adaptive_profile.enable_adaptive_beamforming = true;
  signal::pipeline::SignalPipeline adaptive_pipeline(session_config);
  adaptive_pipeline.SetControlProfile(adaptive_profile);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> adaptive_manager =
      adaptive_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(adaptive_manager != nullptr);
  RunPipelineCycle(&adaptive_pipeline, cycle_1_input, &environment_service, 1u);
  adaptive_manager->Update(MakeLifecycleCycle(1u, 1u),
                           adaptive_pipeline.GetLastTrackMeasurements());
  adaptive_pipeline.SetAssociationSeeds(adaptive_manager->BuildAssociationSeeds());
  RunPipelineCycle(&adaptive_pipeline, cycle_2_input, &environment_service, 2u);
  adaptive_manager->Update(MakeLifecycleCycle(2u, 2u),
                           adaptive_pipeline.GetLastTrackMeasurements());
  const std::vector<signal::tracking::AssociationTrackSeed> adaptive_seeds =
      adaptive_manager->BuildAssociationSeeds();

  ASSERT_EQ(baseline_seeds.size(), 1u);
  ASSERT_EQ(adaptive_seeds.size(), 1u);
  EXPECT_LT(adaptive_seeds[0].gaussian_state.covariance(0, 0),
            baseline_seeds[0].gaussian_state.covariance(0, 0));
}

TEST(SignalPipelineTest, FrequencyAgilityDoesNotRetuneLifecycleTrackingParameters) {
  config::ArSessionConfig session_config;
  session_config.policy.detection = config::profiles::kDetectionPriorityDetection;
  session_config.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
  session_config.policy.tracking.enable_kalman_filter = true;

  const ExecutionConfig base_exec_config = config::mapping::MapSessionToExecution(session_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> unsynced_manager =
      signal::pipeline::CreateAutoLifecycleManagerForRuntimeConfig(base_exec_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> synced_manager =
      signal::pipeline::CreateAutoLifecycleManagerForRuntimeConfig(base_exec_config);
  ASSERT_TRUE(unsynced_manager != nullptr);
  ASSERT_TRUE(synced_manager != nullptr);

  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline measurement_source_pipeline(session_config);
  const session::ArSceneTargetList cycle_1_input{BuildPhysicsTarget(100.0f, 4.0f)};
  RunPipelineCycle(&measurement_source_pipeline, cycle_1_input, &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> cycle_1_measurements =
      measurement_source_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(cycle_1_measurements.size(), 1u);

  unsynced_manager->Update(MakeLifecycleCycle(1u, 1u), cycle_1_measurements);
  synced_manager->Update(MakeLifecycleCycle(1u, 1u), cycle_1_measurements);

  session::ArControlProfile agile_profile;
  agile_profile.enable_agility_frequency = true;
  const signal::pipeline::ResolvedRuntimePipelineConfig agile_runtime_config =
      signal::pipeline::ResolveRuntimePipelineConfig(base_exec_config, agile_profile);
  signal::pipeline::SyncAutoLifecycleManagerForResolvedRuntimeConfig(agile_runtime_config,
                                                                     synced_manager.get());

  unsynced_manager->Update(MakeLifecycleCycle(2u, 2u), {});
  synced_manager->Update(MakeLifecycleCycle(2u, 2u), {});

  const std::vector<signal::tracking::AssociationTrackSeed> unsynced_seeds =
      unsynced_manager->BuildAssociationSeeds();
  const std::vector<signal::tracking::AssociationTrackSeed> synced_seeds =
      synced_manager->BuildAssociationSeeds();
  ASSERT_EQ(unsynced_seeds.size(), 1u);
  ASSERT_EQ(synced_seeds.size(), 1u);
  EXPECT_FLOAT_EQ(synced_seeds[0].gaussian_state.covariance(0, 0),
                  unsynced_seeds[0].gaussian_state.covariance(0, 0));
}

TEST(SignalPipelineTest, InvalidTopologyRebuildKeepsPreviousLifecycleAssemblyOperational) {
  config::ArSessionConfig session_config;
  session_config.policy.detection = config::profiles::kDetectionPriorityDetection;
  session_config.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
  session_config.policy.tracking.enable_kalman_filter = true;

  const ExecutionConfig exec_config = config::mapping::MapSessionToExecution(session_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::CreateAutoLifecycleManagerForRuntimeConfig(exec_config);
  ASSERT_TRUE(lifecycle_manager != nullptr);

  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline measurement_source_pipeline(session_config);
  const session::ArSceneTargetList cycle_1_input{BuildPhysicsTarget(100.0f, 4.0f)};
  RunPipelineCycle(&measurement_source_pipeline, cycle_1_input, &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> cycle_1_measurements =
      measurement_source_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(cycle_1_measurements.size(), 1u);

  lifecycle_manager->Update(MakeLifecycleCycle(1u, 1u), cycle_1_measurements);
  const std::vector<signal::tracking::AssociationTrackSeed> previous_seeds =
      lifecycle_manager->BuildAssociationSeeds();
  ASSERT_EQ(previous_seeds.size(), 1u);

  ExecutionConfig invalid_exec = exec_config;
  invalid_exec.lifecycle.policy.enable_imm_lifecycle = true;
  invalid_exec.lifecycle.engineering.enable_imm_lifecycle = true;
  invalid_exec.lifecycle.imm_model_noise_diff_coeffs = {0.5f, 2.0f};
  invalid_exec.lifecycle.imm_initial_weights = {1.0f};

  signal::pipeline::ResolvedRuntimePipelineConfig invalid_runtime_config;
  invalid_runtime_config.config = invalid_exec;

  const bool sync_succeeded = signal::pipeline::SyncAutoLifecycleManagerForResolvedRuntimeConfig(
      invalid_runtime_config, lifecycle_manager.get());
  EXPECT_FALSE(sync_succeeded);

  lifecycle_manager->Update(MakeLifecycleCycle(2u, 2u), {});
  const std::vector<signal::tracking::AssociationTrackSeed> retained_seeds =
      lifecycle_manager->BuildAssociationSeeds();
  ASSERT_EQ(retained_seeds.size(), 1u);
  EXPECT_EQ(retained_seeds[0].association_key, previous_seeds[0].association_key);
}

TEST(SignalPipelineTest, RfV2InterferenceOnlySuppressesDetectionAndIsOneShot) {
  config::ArSessionConfig config = MakeDetectionFocusedConfig();
  environment::EnvironmentService baseline_environment;
  environment::EnvironmentService jammed_environment;
  signal::pipeline::SignalPipeline baseline_pipeline(config);
  signal::pipeline::SignalPipeline jammed_pipeline(config);
  const session::ArSceneTarget target = BuildPhysicsTarget(1000.0f, 10.0f);

  ASSERT_TRUE(RunPipelineCycle(&baseline_pipeline, {target}, &baseline_environment, 1U)
                  .executed_this_cycle);
  ASSERT_FALSE(baseline_pipeline.GetLastTrackMeasurements().empty());

  auto ctx = MakeRfV2DetectionContext(1.0e6);
  // be501c5d（VS2015 兼容）为 SignalCycleInput 引入显式构造函数，聚合初始化
  // 不可用——改用单参构造 + 成员赋值。
  signal::pipeline::SignalCycleInput jammed_input(
      ToSceneTargets(session::ArSceneTargetList{target}));
  jammed_input.rf_v2_detection_context = &ctx;
  jammed_environment.BeginCycle(MakeEnvironmentCycle(1U));
  ASSERT_TRUE(
      jammed_pipeline.RunCycle(jammed_input, jammed_environment).executed_this_cycle);
  EXPECT_TRUE(jammed_pipeline.GetLastTrackMeasurements().empty());

  ASSERT_TRUE(
      RunPipelineCycle(&jammed_pipeline, {target}, &jammed_environment, 2U).executed_this_cycle);
  EXPECT_FALSE(jammed_pipeline.GetLastTrackMeasurements().empty());
}

TEST(SignalPipelineTest, EccmDoesNotRetuneImmLifecycleParameters) {
  config::ArSessionConfig session_config;
  session_config.policy.detection = config::profiles::kDetectionPriorityDetection;
  session_config.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
  session_config.policy.lifecycle.enable_imm_lifecycle = true;

  environment::EnvironmentService environment_service;

  session::ArSceneTarget target = BuildPhysicsTarget(120.0f, 4.0f);
  const session::ArSceneTargetList cycle_1_input{target};

  signal::pipeline::SignalPipeline baseline_pipeline(session_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> baseline_manager =
      baseline_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(baseline_manager != nullptr);
  RunPipelineCycle(&baseline_pipeline, cycle_1_input, &environment_service, 1u);
  baseline_manager->Update(MakeLifecycleCycle(1u, 1u),
                           baseline_pipeline.GetLastTrackMeasurements());
  baseline_manager->Update(MakeLifecycleCycle(2u, 2u), {});
  const std::vector<signal::tracking::AssociationTrackSeed> baseline_seeds =
      baseline_manager->BuildAssociationSeeds();

  session::ArControlProfile protected_profile;
  protected_profile.enable_eccm_rejitter = true;
  protected_profile.eccm_burnthrough_gain = 1.5f;
  signal::pipeline::SignalPipeline protected_pipeline(session_config);
  protected_pipeline.SetControlProfile(protected_profile);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> protected_manager =
      protected_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(protected_manager != nullptr);
  RunPipelineCycle(&protected_pipeline, cycle_1_input, &environment_service, 1u);
  protected_manager->Update(MakeLifecycleCycle(1u, 1u),
                            protected_pipeline.GetLastTrackMeasurements());
  protected_manager->Update(MakeLifecycleCycle(2u, 2u), {});
  const std::vector<signal::tracking::AssociationTrackSeed> protected_seeds =
      protected_manager->BuildAssociationSeeds();

  ASSERT_EQ(baseline_seeds.size(), 1u);
  ASSERT_EQ(protected_seeds.size(), 1u);
  EXPECT_FLOAT_EQ(protected_seeds[0].gaussian_state.covariance(0, 0),
                  baseline_seeds[0].gaussian_state.covariance(0, 0));
}

TEST(SignalPipelineTest, ExposesStructuredTrackMeasurements) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  session::ArSceneTarget first(100.0f, 0.0f, 0.0f, 2.0f);

  first.position_x = 10.0f;
  first.range_m = 10.0f;
  session::ArSceneTarget second(220.0f, 0.0f, 0.0f, 5.0f);

  second.position_x = 100.0f;
  second.range_m = 100.0f;
  const session::ArSceneTargetList cycle_1{first, second};

  RunPipelineCycle(&signal_pipeline, cycle_1, &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(first_measurements.size(), 2u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_FALSE(first_measurements[1].raw_measurement.matched_existing_track);
  EXPECT_EQ(first_measurements[0].raw_measurement.source_index, 0u);
  EXPECT_EQ(first_measurements[1].raw_measurement.source_index, 1u);
  EXPECT_GT(first_measurements[0].filtered_feature.observed_speed, 0.0f);
  EXPECT_GT(first_measurements[0].raw_measurement.detection_margin_db, -2.0f);
  EXPECT_TRUE(first_measurements[0].raw_measurement.used_position_association);

  first.velocity_x = 101.0f;
  first.velocity_y = 0.0f;
  first.velocity_z = 0.0f;
  first.rcs = 2.1f;
  first.position_x = 11.0f;
  first.range_m = 11.0f;
  second.velocity_x = 219.5f;
  second.velocity_y = 0.0f;
  second.velocity_z = 0.0f;
  second.rcs = 4.9f;
  second.position_x = 101.0f;
  second.range_m = 101.0f;
  signal::tracking::AssociationTrackSeed first_seed;
  first_seed.association_key = first_measurements[0].raw_measurement.association_key;
  first_seed.has_position = true;
  first_seed.position = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  first_seed.has_gaussian_state = true;
  first_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  first_seed.gaussian_state.mean(0) = 10.0f;
  first_seed.gaussian_state.covariance = signal::tracking::StateCovariance::Identity() * 25.0f;

  signal::tracking::AssociationTrackSeed second_seed;
  second_seed.association_key = first_measurements[1].raw_measurement.association_key;
  second_seed.has_position = true;
  second_seed.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  second_seed.has_gaussian_state = true;
  second_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  second_seed.gaussian_state.mean(0) = 100.0f;
  second_seed.gaussian_state.covariance = signal::tracking::StateCovariance::Identity() * 25.0f;

  signal_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>{first_seed, second_seed});
  const session::ArSceneTargetList cycle_2{first, second};
  RunPipelineCycle(&signal_pipeline, cycle_2, &environment_service, 2u);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(second_measurements.size(), 2u);
  EXPECT_TRUE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_TRUE(second_measurements[1].raw_measurement.matched_existing_track);
  EXPECT_GT(second_measurements[0].raw_measurement.association_cost, 0.0f);
  EXPECT_GT(second_measurements[1].raw_measurement.association_cost, 0.0f);
  EXPECT_TRUE(second_measurements[0].raw_measurement.used_position_association);
}

TEST(SignalPipelineTest, CompletesWithoutCrashWhenDetectedTargetUsesDefaultPosition) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  const session::ArSceneTargetList input_state{
      session::ArSceneTarget(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f)};

  RunPipelineCycle(&signal_pipeline, input_state, &environment_service);
}

TEST(SignalPipelineTest, UsesPositionAssociationByDefaultWhenCartesianPositionExists) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  session::ArSceneTarget first(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);

  first.position_x = 10.0f;
  first.position_y = 0.0f;
  first.position_z = 0.0f;
  first.range_m = 10.0f;

  session::ArSceneTarget second(220.0f, 0.0f, 0.0f, 5.0f, 3.0f, 0.0f, 0.0f);

  second.position_x = 100.0f;
  second.position_y = 0.0f;
  second.position_z = 0.0f;
  second.range_m = 100.0f;

  RunPipelineCycle(&signal_pipeline, session::ArSceneTargetList{first, second},
                   &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(first_measurements.size(), 2u);
  EXPECT_TRUE(first_measurements[0].raw_measurement.used_position_association);
  EXPECT_TRUE(first_measurements[1].raw_measurement.used_position_association);

  signal::tracking::AssociationTrackSeed first_seed;
  first_seed.association_key = first_measurements[0].raw_measurement.association_key;
  first_seed.has_position = true;
  first_seed.position = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  first_seed.has_gaussian_state = true;
  first_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  first_seed.gaussian_state.mean(0) = 10.0f;
  first_seed.gaussian_state.covariance = signal::tracking::StateCovariance::Identity() * 25.0f;

  signal::tracking::AssociationTrackSeed second_seed;
  second_seed.association_key = first_measurements[1].raw_measurement.association_key;
  second_seed.has_position = true;
  second_seed.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  second_seed.has_gaussian_state = true;
  second_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  second_seed.gaussian_state.mean(0) = 100.0f;
  second_seed.gaussian_state.covariance = signal::tracking::StateCovariance::Identity() * 25.0f;

  signal_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>{first_seed, second_seed});

  first.position_x = 11.0f;
  second.position_x = 101.0f;
  RunPipelineCycle(&signal_pipeline, session::ArSceneTargetList{second, first},
                   &environment_service, 2u);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(second_measurements.size(), 2u);
  EXPECT_TRUE(second_measurements[0].raw_measurement.used_position_association);
  EXPECT_TRUE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_TRUE(second_measurements[1].raw_measurement.matched_existing_track);
}

TEST(SignalPipelineTest, UsesLifecycleAssociationSeedsByDefault) {
  environment::EnvironmentService environment_service;
  environment_service.BeginCycle(MakeEnvironmentCycle(1u));

  signal::pipeline::SignalPipeline signal_pipeline;
  session::ArSceneTarget target(0.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);

  target.position_x = 10.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 10.0f;

  RunPipelineCycle(&signal_pipeline, session::ArSceneTargetList{target}, &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(first_measurements.size(), 1u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);
  const std::uint64_t first_key = first_measurements[0].raw_measurement.association_key;
  EXPECT_NE(first_key, 0u);

  target.position_x = 10.2f;
  target.range_m = 10.2f;
  RunPipelineCycle(&signal_pipeline, session::ArSceneTargetList{target}, &environment_service, 2u);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(second_measurements.size(), 1u);
  EXPECT_TRUE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_EQ(second_measurements[0].raw_measurement.association_key, first_key);
}

TEST(SignalPipelineTest, ClearManualAssociationSeedsKeepsLifecycleSeedsActive) {
  environment::EnvironmentService environment_service;
  environment_service.BeginCycle(MakeEnvironmentCycle(1u));

  signal::pipeline::SignalPipeline signal_pipeline;

  signal::tracking::AssociationTrackSeed side_channel_seed;
  side_channel_seed.association_key = 777u;
  side_channel_seed.has_position = true;
  side_channel_seed.position = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  side_channel_seed.has_gaussian_state = true;
  side_channel_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  side_channel_seed.gaussian_state.mean(0) = 10.0f;
  side_channel_seed.gaussian_state.covariance =
      signal::tracking::StateCovariance::Identity() * 25.0f;
  signal_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, side_channel_seed));
  signal_pipeline.ClearManualAssociationSeeds();

  session::ArSceneTarget target(0.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);

  target.position_x = 10.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 10.0f;

  RunPipelineCycle(&signal_pipeline, session::ArSceneTargetList{target}, &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(first_measurements.size(), 1u);
  const std::uint64_t first_key = first_measurements[0].raw_measurement.association_key;
  EXPECT_NE(first_key, 0u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);

  target.position_x = 10.1f;
  target.range_m = 10.1f;
  RunPipelineCycle(&signal_pipeline, session::ArSceneTargetList{target}, &environment_service, 2u);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(second_measurements.size(), 1u);
  EXPECT_TRUE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_EQ(second_measurements[0].raw_measurement.association_key, first_key);
}

TEST(SignalPipelineTest, InvalidManualAssociationSeedsDoNotDisableLifecycleSeeds) {
  environment::EnvironmentService environment_service;
  environment_service.BeginCycle(MakeEnvironmentCycle(1u));

  signal::pipeline::SignalPipeline signal_pipeline;

  signal::tracking::AssociationTrackSeed invalid_seed;
  invalid_seed.association_key = 0u;
  invalid_seed.has_position = true;
  invalid_seed.position = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  invalid_seed.has_gaussian_state = true;
  invalid_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  invalid_seed.gaussian_state.mean(0) = 10.0f;
  invalid_seed.gaussian_state.covariance = signal::tracking::StateCovariance::Identity() * 25.0f;
  signal_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, invalid_seed));

  session::ArSceneTarget target(0.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);

  target.position_x = 10.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 10.0f;

  const session::SignalCycleResult first_result = RunPipelineCycle(
      &signal_pipeline, session::ArSceneTargetList{target}, &environment_service, 1u);
  EXPECT_TRUE(first_result.executed_this_cycle);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(first_measurements.size(), 1u);
  const std::uint64_t first_key = first_measurements[0].raw_measurement.association_key;
  EXPECT_NE(first_key, 0u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);

  target.position_x = 10.1f;
  target.range_m = 10.1f;
  const session::SignalCycleResult second_result = RunPipelineCycle(
      &signal_pipeline, session::ArSceneTargetList{target}, &environment_service, 2u);
  EXPECT_TRUE(second_result.executed_this_cycle);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(second_measurements.size(), 1u);
  EXPECT_TRUE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_EQ(second_measurements[0].raw_measurement.association_key, first_key);
}

TEST(SignalPipelineTest, InvalidEnvironmentCycleAbortsAndClearsLastCycleCache) {
  signal::pipeline::SignalPipeline signal_pipeline;
  environment::EnvironmentService valid_environment;
  environment::EnvironmentService invalid_environment;

  session::ArSceneTarget target(0.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);

  target.position_x = 10.0f;
  target.range_m = 10.0f;

  const session::SignalCycleResult valid_result = RunPipelineCycle(
      &signal_pipeline, session::ArSceneTargetList{target}, &valid_environment, 1u);
  EXPECT_TRUE(valid_result.executed_this_cycle);
  EXPECT_EQ(valid_result.abort_reason, session::SignalCycleAbortReason::kNone);
  ASSERT_EQ(valid_result.updated_scene_targets.size(), 1u);
  ASSERT_EQ(signal_pipeline.GetLastTrackMeasurements().size(), 1u);

  const session::SignalCycleResult invalid_result = signal_pipeline.RunCycle(
      signal::pipeline::SignalCycleInput{ToSceneTargets(session::ArSceneTargetList{target})},
      invalid_environment);
  EXPECT_FALSE(invalid_result.executed_this_cycle);
  EXPECT_EQ(invalid_result.abort_reason, session::SignalCycleAbortReason::kInvalidEnvironmentCycle);
  EXPECT_TRUE(invalid_result.updated_scene_targets.empty());
  EXPECT_TRUE(invalid_result.decision_frame.tracks.empty());
  EXPECT_TRUE(signal_pipeline.GetLastTrackMeasurements().empty());
  EXPECT_EQ(signal_pipeline.GetLastAssociationQualityMetrics().detection_count, 0u);
}

// ============================================================================
// PropagationModel — 边界条件补充
// ============================================================================

/// @brief 杂波功率由内部模型统一给出，不再由外部场景直填。

TEST(SignalPipelineTest, SameInstanceControlProfileSwitchAcrossCyclesSyncsLifecycleCovariance) {
  config::ArSessionConfig session_config;
  session_config.policy.detection = config::profiles::kDetectionPriorityDetection;
  session_config.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
  session_config.policy.tracking.enable_kalman_filter = true;

  environment::EnvironmentService environment_service;

  session::ArSceneTarget target(1.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f);

  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const session::ArSceneTargetList input{target};

  signal::pipeline::SignalPipeline pipeline(session_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> manager =
      pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(manager != nullptr);

  RunPipelineCycle(&pipeline, input, &environment_service, 1u);
  manager->Update(MakeLifecycleCycle(1u, 1u), pipeline.GetLastTrackMeasurements());

  session::ArControlProfile baseline_profile;
  pipeline.SetControlProfile(baseline_profile);

  target.position_x = 101.0f;
  target.range_m = 101.0f;
  const session::ArSceneTargetList input2{target};
  pipeline.SetAssociationSeeds(manager->BuildAssociationSeeds());
  RunPipelineCycle(&pipeline, input2, &environment_service, 2u);
  manager->Update(MakeLifecycleCycle(2u, 2u), pipeline.GetLastTrackMeasurements());
  const std::vector<signal::tracking::AssociationTrackSeed> baseline_seeds =
      manager->BuildAssociationSeeds();
  ASSERT_EQ(baseline_seeds.size(), 1u);
  const float baseline_cov = baseline_seeds[0].gaussian_state.covariance(0, 0);

  session::ArControlProfile agile_profile;
  agile_profile.enable_agility_frequency = true;
  pipeline.SetControlProfile(agile_profile);

  target.position_x = 102.0f;
  target.range_m = 102.0f;
  const session::ArSceneTargetList input3{target};
  pipeline.SetAssociationSeeds(manager->BuildAssociationSeeds());
  RunPipelineCycle(&pipeline, input3, &environment_service, 3u);
  manager->Update(MakeLifecycleCycle(3u, 3u), pipeline.GetLastTrackMeasurements());
  const std::vector<signal::tracking::AssociationTrackSeed> agile_seeds =
      manager->BuildAssociationSeeds();
  ASSERT_EQ(agile_seeds.size(), 1u);

  EXPECT_GT(std::fabs(agile_seeds[0].gaussian_state.covariance(0, 0) - baseline_cov), 1.0e-4f);
}

TEST(SignalPipelineInternalConfigTest, DetectionBuilderProfileMapsToBaselineProfile) {
  const ExecutionConfig exec_config =
      config::mapping::MapSessionToExecution(MakeDetectionFocusedConfig());

  EXPECT_FLOAT_EQ(exec_config.association.policy.unassigned_cost, 9.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.speed_decay_ratio_on_loss, 0.95f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.rcs_decay_ratio_on_loss, 0.92f);
}

TEST(SignalPipelineInternalConfigTest, TrackingBuilderProfileMapsToTrackingProfile) {
  const ExecutionConfig exec_config =
      config::mapping::MapSessionToExecution(MakeTrackingFocusedConfig());

  EXPECT_FLOAT_EQ(exec_config.association.policy.unassigned_cost, 9.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.speed_decay_ratio_on_loss, 1.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.rcs_decay_ratio_on_loss, 1.0f);
}

TEST(SignalPipelineInternalConfigTest, RobustBuilderProfileMapsToRobustProfile) {
  const ExecutionConfig exec_config =
      config::mapping::MapSessionToExecution(MakeRobustTrackingConfig());

  EXPECT_FLOAT_EQ(exec_config.association.policy.unassigned_cost, 12.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.speed_decay_ratio_on_loss, 0.95f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.rcs_decay_ratio_on_loss, 0.92f);
}

TEST(SignalPipelineInternalConfigTest,
     NonDefaultRcsPhysicsBreaksTrackingProfileSignatureAndFallsBackToBaseline) {
  config::ArSessionConfig session_config = MakeTrackingFocusedConfig();
  session_config.hardware.rcs_physics = config::profiles::kEnhancedRcsPhysics;
  const ExecutionConfig exec_config = config::mapping::MapSessionToExecution(session_config);

  EXPECT_FLOAT_EQ(exec_config.association.policy.unassigned_cost, 9.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.speed_decay_ratio_on_loss, 1.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.rcs_decay_ratio_on_loss, 1.0f);
}

TEST(SignalPipelineInternalConfigTest, CustomConfigStaysBaselineEvenWithHighLifecycleThresholds) {
  config::ArSessionConfig session_config = MakeDetectionFocusedConfig();
  const ExecutionConfig exec_config = config::mapping::MapSessionToExecution(session_config);

  EXPECT_FLOAT_EQ(exec_config.association.policy.unassigned_cost, 9.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.speed_decay_ratio_on_loss, 0.95f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.rcs_decay_ratio_on_loss, 0.92f);
}

TEST(SignalPipelineInternalConfigTest, ImmToggleOnlyControlsImmInternalDefaults) {
  config::ArSessionConfig session_config = MakeTrackingFocusedConfig();
  const ExecutionConfig imm_disabled_exec = config::mapping::MapSessionToExecution(session_config);
  EXPECT_TRUE(imm_disabled_exec.lifecycle.imm_model_noise_diff_coeffs.empty());
  EXPECT_FLOAT_EQ(imm_disabled_exec.tracking.engineering.speed_decay_ratio_on_loss, 1.0f);
  EXPECT_FLOAT_EQ(imm_disabled_exec.tracking.engineering.rcs_decay_ratio_on_loss, 1.0f);

  session_config.policy.lifecycle.enable_imm_lifecycle = true;
  const ExecutionConfig imm_enabled_exec = config::mapping::MapSessionToExecution(session_config);
  ASSERT_EQ(imm_enabled_exec.lifecycle.imm_model_noise_diff_coeffs.size(), 2U);
  EXPECT_FLOAT_EQ(imm_enabled_exec.lifecycle.imm_model_noise_diff_coeffs[0], 1.0f);
  EXPECT_FLOAT_EQ(imm_enabled_exec.lifecycle.imm_model_noise_diff_coeffs[1], 10.0f);
  EXPECT_FLOAT_EQ(imm_enabled_exec.tracking.engineering.speed_decay_ratio_on_loss, 1.0f);
  EXPECT_FLOAT_EQ(imm_enabled_exec.tracking.engineering.rcs_decay_ratio_on_loss, 1.0f);
}

}  // namespace tests
}  // namespace airborne_radar
