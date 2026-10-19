#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <map>
#include <string>

#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsIssueCodes.h"
#include "sbirs_sensor/environment/SbirsEnvironmentModel.h"
#include "sbirs_sensor/foundation/SbirsNoiseModel.h"
#include "sbirs_sensor/foundation/SbirsRadiometry.h"
#include "sbirs_sensor/pipeline/SbirsPipeline.h"
#include "sbirs_sensor/runtime/SbirsPipelineConfigMapper.h"

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

sbirs_sensor::session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_sensor::session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

sbirs_sensor::session::SbirsSceneTarget HotTarget(std::uint64_t id, double y) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = id;
  target.target_name = "target";
  target.position_ecef_m = Vector(8000000.0, y, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e8;
  return target;
}

sbirs_sensor::session::SbirsSceneTarget HotTargetAtAngles(std::uint64_t id, double azimuth_deg,
                                                          double elevation_deg) {
  const double azimuth_rad = azimuth_deg * kPi / 180.0;
  const double elevation_rad = elevation_deg * kPi / 180.0;
  const double horizontal = std::cos(elevation_rad);
  sbirs_sensor::session::SbirsSceneTarget target = HotTarget(id, 0.0);
  target.position_ecef_m =
      Vector(7000000.0 + 1000000.0 * horizontal * std::cos(azimuth_rad),
             1000000.0 * horizontal * std::sin(azimuth_rad), 1000000.0 * std::sin(elevation_rad));
  return target;
}

sbirs_sensor::config::SbirsSessionConfig PipelineConfig() {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.hardware.integration_time_sec = 1.0f;
  config.mission.scan_start_az_deg = -1.0f;
  config.mission.scan_span_deg = 11.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.mission.wide_field_fov_az_deg = 20.0f;
  config.mission.wide_field_fov_el_deg = 20.0f;
  config.mission.narrow_field_fov_az_deg = 5.0f;
  config.mission.narrow_field_fov_el_deg = 5.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  config.policy.error_model.attitude_sigma_deg = 0.0f;
  config.policy.error_model.orbit_sigma_deg = 0.0f;
  config.policy.error_model.fov_sigma_deg = 0.0f;
  config.policy.error_model.range_fraction_sigma = 0.0f;
  return config;
}

sbirs_sensor::config::SbirsSessionConfig ImmMultiTargetConfig() {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.policy.tracking.estimated_backend =
      sbirs_sensor::config::SbirsEstimatedTrackingBackend::kImm;
  config.policy.tracking.imm_model_noise_diff_coeffs = {0.5f, 80.0f};
  return config;
}

sbirs_sensor::session::SbirsCycleInput TwoTargetInput(std::uint32_t cycle_index,
                                                      bool reverse_order = false) {
  sbirs_sensor::session::SbirsCycleInputBuilder builder;
  builder.WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
      .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
      .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{});
  if (reverse_order) {
    builder.AddTarget(HotTarget(2U, 5000.0)).AddTarget(HotTarget(1U, 0.0));
  } else {
    builder.AddTarget(HotTarget(1U, 0.0)).AddTarget(HotTarget(2U, 5000.0));
  }
  return builder.Build();
}

void ExpectImmTargetStateEqual(const sbirs_sensor::pipeline::SbirsPipelineSnapshot& left,
                               const sbirs_sensor::pipeline::SbirsPipelineSnapshot& right,
                               std::uint64_t target_id) {
  ASSERT_EQ(left.filter_states.count(target_id), 1U);
  ASSERT_EQ(right.filter_states.count(target_id), 1U);
  EXPECT_TRUE(
      left.filter_states.at(target_id).mean.isApprox(right.filter_states.at(target_id).mean));
  EXPECT_TRUE(left.filter_states.at(target_id).covariance.isApprox(
      right.filter_states.at(target_id).covariance));
  ASSERT_EQ(left.imm_snapshots.count(target_id), 1U);
  ASSERT_EQ(right.imm_snapshots.count(target_id), 1U);
  const auto& left_models = left.imm_snapshots.at(target_id).model_states;
  const auto& right_models = right.imm_snapshots.at(target_id).model_states;
  ASSERT_EQ(left_models.size(), right_models.size());
  for (std::size_t index = 0U; index < left_models.size(); ++index) {
    EXPECT_TRUE(left_models[index].state.mean.isApprox(right_models[index].state.mean));
    EXPECT_TRUE(left_models[index].state.covariance.isApprox(right_models[index].state.covariance));
    EXPECT_FLOAT_EQ(left_models[index].weight, right_models[index].weight);
  }
}

/// 按 code 查找诊断条目（规则 13b 排除诊断断言用）。
const sbirs_sensor::session::SbirsIssue* FindIssue(
    const sbirs_sensor::pipeline::SbirsPipelineResult& result, const char* code) {
  for (const auto& issue : result.issues) {
    if (issue.code == code) {
      return &issue;
    }
  }
  return nullptr;
}

TEST(SbirsPipelineTest, WideCandidateCapturesIntoNfov) {
  const sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(7U, 0.0))
          .Build();

  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  ASSERT_FALSE(result.detections.empty());
  EXPECT_EQ(result.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
  EXPECT_EQ(result.detections.front().attribution.tracking_source,
            sbirs_sensor::attribution::SbirsTrackingSource::kEstimated);
  EXPECT_FALSE(result.detections.front().attribution.has_estimation_nis);
}

TEST(SbirsPipelineTest, TruthModesShareGatesButExposeDistinctSuccessfulMeasurements) {
  sbirs_sensor::config::SbirsSessionConfig strict_config = PipelineConfig();
  strict_config.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kStrictTruthAssisted;
  strict_config.policy.error_model.attitude_sigma_deg = 0.2f;
  strict_config.policy.error_model.range_fraction_sigma = 0.01f;
  sbirs_sensor::config::SbirsSessionConfig sensor_config = strict_config;
  sensor_config.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kSensorLikeTruthAssisted;

  sbirs_sensor::pipeline::SbirsPipeline strict(
      sbirs_sensor::runtime::MapSessionToInternal(strict_config));
  sbirs_sensor::pipeline::SbirsPipeline sensor(
      sbirs_sensor::runtime::MapSessionToInternal(sensor_config));
  const auto input = TwoTargetInput(1U);
  const auto strict_acquisition = strict.RunCycle(input);
  const auto sensor_acquisition = sensor.RunCycle(input);
  ASSERT_FALSE(strict_acquisition.detections.empty());
  ASSERT_FALSE(sensor_acquisition.detections.empty());
  const auto& strict_detection = strict_acquisition.detections.front();
  const auto& sensor_detection = sensor_acquisition.detections.front();
  EXPECT_EQ(strict_detection.attribution.tracking_source,
            sbirs_sensor::attribution::SbirsTrackingSource::kStrictTruthAssisted);
  EXPECT_EQ(sensor_detection.attribution.tracking_source,
            sbirs_sensor::attribution::SbirsTrackingSource::kSensorLikeTruthAssisted);
  // GMST≈0 测试时刻下 ECI az 残余 2.35e-9 rad（≈1.3e-7°），用近等断言。
  EXPECT_NEAR(strict_detection.record.azimuth_rad, 0.0f, 1.0e-6f);
  EXPECT_NEAR(strict_detection.record.elevation_rad, 0.0f, 1.0e-6f);
  EXPECT_FLOAT_EQ(strict_detection.attribution.estimated_range_m, 1000000.0f);
  EXPECT_NE(sensor_detection.record.azimuth_rad, strict_detection.record.azimuth_rad);
  EXPECT_NE(sensor_detection.attribution.estimated_range_m,
            strict_detection.attribution.estimated_range_m);
  EXPECT_FLOAT_EQ(sensor_detection.record.infrared_snr_linear,
                  strict_detection.record.infrared_snr_linear);

  const auto strict_snapshot = strict.CaptureRuntimeState();
  const auto sensor_snapshot = sensor.CaptureRuntimeState();
  EXPECT_EQ(strict_snapshot.wfov_measurement_random_state,
            sensor_snapshot.wfov_measurement_random_state);
  EXPECT_EQ(strict_snapshot.estimated_measurement_random_state,
            sensor_snapshot.estimated_measurement_random_state);
  EXPECT_NE(strict_snapshot.sensor_like_output_random_state,
            sensor_snapshot.sensor_like_output_random_state);

  sensor_config.policy.detection.narrow_min_snr_linear =
      std::numeric_limits<float>::max();
  sbirs_sensor::runtime::SbirsRuntimeConfigImpact gate_impact;
  gate_impact.reset_nfov_gate_failure_counts = true;
  sensor.ApplyConfig(sbirs_sensor::runtime::MapSessionToInternal(sensor_config), gate_impact);
  const auto before_coast = sensor.CaptureRuntimeState();
  const auto coast = sensor.RunCycle(TwoTargetInput(2U));
  ASSERT_FALSE(coast.detections.empty());
  EXPECT_FALSE(coast.detections.front().record.detected);
  const auto after_coast = sensor.CaptureRuntimeState();
  EXPECT_EQ(after_coast.sensor_like_output_random_state,
            before_coast.sensor_like_output_random_state);
}

TEST(SbirsPipelineTest, TruthModesHaveIdenticalCoastingLossAndChannelRelease) {
  sbirs_sensor::config::SbirsSessionConfig strict_config = PipelineConfig();
  strict_config.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kStrictTruthAssisted;
  sbirs_sensor::config::SbirsSessionConfig sensor_config = strict_config;
  sensor_config.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kSensorLikeTruthAssisted;

  sbirs_sensor::pipeline::SbirsPipeline strict(
      sbirs_sensor::runtime::MapSessionToInternal(strict_config));
  sbirs_sensor::pipeline::SbirsPipeline sensor(
      sbirs_sensor::runtime::MapSessionToInternal(sensor_config));
  auto input = sbirs_sensor::session::SbirsCycleInputBuilder()
                   .WithCycleIndex(1U)
                   .WithDeltaTimeSec(1.0f)
                   .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                   .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                   .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                   .AddTarget(HotTarget(91U, 0.0))
                   .Build();
  strict.RunCycle(input);
  sensor.RunCycle(input);
  const std::uint32_t sensor_random_state =
      sensor.CaptureRuntimeState().sensor_like_output_random_state;

  input.scene.front().radiant_intensity_w_per_sr = 0.0;
  for (std::uint32_t cycle_index = 2U; cycle_index <= 3U; ++cycle_index) {
    input.cycle_index = cycle_index;
    const auto strict_result = strict.RunCycle(input);
    const auto sensor_result = sensor.RunCycle(input);
    ASSERT_EQ(strict_result.detections.size(), 1U);
    ASSERT_EQ(sensor_result.detections.size(), 1U);
    const auto& strict_detection = strict_result.detections.front();
    const auto& sensor_detection = sensor_result.detections.front();
    EXPECT_EQ(strict_detection.record.detected, sensor_detection.record.detected);
    EXPECT_EQ(strict_detection.attribution.nfov_geometry_gate_passed,
              sensor_detection.attribution.nfov_geometry_gate_passed);
    EXPECT_EQ(strict_detection.attribution.nfov_snr_gate_passed,
              sensor_detection.attribution.nfov_snr_gate_passed);
    EXPECT_EQ(strict_detection.attribution.nfov_tracking_gate_failure_count,
              sensor_detection.attribution.nfov_tracking_gate_failure_count);
    EXPECT_EQ(strict_detection.attribution.nfov_tracking_coasting,
              sensor_detection.attribution.nfov_tracking_coasting);
    EXPECT_EQ(strict_detection.attribution.capture_failure_reason,
              sensor_detection.attribution.capture_failure_reason);
  }

  const auto strict_snapshot = strict.CaptureRuntimeState();
  const auto sensor_snapshot = sensor.CaptureRuntimeState();
  EXPECT_EQ(strict_snapshot.target_states, sensor_snapshot.target_states);
  EXPECT_TRUE(strict_snapshot.nfov_scheduler.target_to_cue_source.empty());
  EXPECT_EQ(strict_snapshot.nfov_scheduler.target_to_cue_source,
            sensor_snapshot.nfov_scheduler.target_to_cue_source);
  EXPECT_EQ(strict_snapshot.target_states.at(91U),
            sbirs_sensor::pipeline::SbirsTargetState::kWideCandidate);
  EXPECT_EQ(sensor_snapshot.sensor_like_output_random_state, sensor_random_state);
}

TEST(SbirsPipelineTest, SensorLikeSnapshotRestoreContinuesOutputRandomStream) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kSensorLikeTruthAssisted;
  config.policy.error_model.attitude_sigma_deg = 0.2f;
  config.policy.error_model.range_fraction_sigma = 0.01f;
  sbirs_sensor::pipeline::SbirsPipeline uninterrupted(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  auto input = sbirs_sensor::session::SbirsCycleInputBuilder()
                   .WithCycleIndex(1U)
                   .WithDeltaTimeSec(1.0f)
                   .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                   .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                   .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                   .AddTarget(HotTarget(92U, 0.0))
                   .Build();
  uninterrupted.RunCycle(input);
  const auto snapshot = uninterrupted.CaptureRuntimeState();

  input.cycle_index = 2U;
  const auto expected = uninterrupted.RunCycle(input);
  sbirs_sensor::pipeline::SbirsPipeline restored(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  ASSERT_TRUE(restored.RestoreRuntimeState(snapshot));
  const auto actual = restored.RunCycle(input);
  ASSERT_EQ(expected.detections.size(), 1U);
  ASSERT_EQ(actual.detections.size(), 1U);
  EXPECT_FLOAT_EQ(actual.detections.front().record.azimuth_rad,
                  expected.detections.front().record.azimuth_rad);
  EXPECT_FLOAT_EQ(actual.detections.front().record.elevation_rad,
                  expected.detections.front().record.elevation_rad);
  EXPECT_FLOAT_EQ(actual.detections.front().record.infrared_snr_linear,
                  expected.detections.front().record.infrared_snr_linear);
  EXPECT_FLOAT_EQ(actual.detections.front().attribution.estimated_range_m,
                  expected.detections.front().attribution.estimated_range_m);
  EXPECT_EQ(actual.detections.front().attribution.tracking_source,
            sbirs_sensor::attribution::SbirsTrackingSource::kSensorLikeTruthAssisted);
  EXPECT_EQ(restored.CaptureRuntimeState().sensor_like_output_random_state,
            uninterrupted.CaptureRuntimeState().sensor_like_output_random_state);
}

TEST(SbirsPipelineTest, TruthRetagKeepsLockWhileEstimatedTransitionReleasesIt) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kStrictTruthAssisted;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  pipeline.RunCycle(TwoTargetInput(1U));
  const auto before = pipeline.CaptureRuntimeState();

  config.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kSensorLikeTruthAssisted;
  sbirs_sensor::runtime::SbirsRuntimeConfigImpact retag;
  retag.retag_truth_tracks = true;
  retag.previous_tracking_mode = sbirs_sensor::config::SbirsTrackingMode::kStrictTruthAssisted;
  retag.next_tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kSensorLikeTruthAssisted;
  pipeline.ApplyConfig(sbirs_sensor::runtime::MapSessionToInternal(config), retag);
  const auto retagged = pipeline.CaptureRuntimeState();
  EXPECT_EQ(retagged.nfov_scheduler.target_to_cue_source, before.nfov_scheduler.target_to_cue_source);
  EXPECT_EQ(retagged.target_states.at(1U),
            sbirs_sensor::pipeline::SbirsTargetState::kSensorLikeTruthAssistedTracking);
  EXPECT_EQ(retagged.sensor_like_output_random_state,
            before.sensor_like_output_random_state);

  config.policy.tracking.tracking_mode = sbirs_sensor::config::SbirsTrackingMode::kEstimated;
  sbirs_sensor::runtime::SbirsRuntimeConfigImpact release;
  release.release_incompatible_tracks = true;
  pipeline.ApplyConfig(sbirs_sensor::runtime::MapSessionToInternal(config), release);
  const auto released = pipeline.CaptureRuntimeState();
  EXPECT_TRUE(released.nfov_scheduler.target_to_cue_source.empty());
  EXPECT_EQ(released.target_states.at(1U),
            sbirs_sensor::pipeline::SbirsTargetState::kWideCandidate);
}

TEST(SbirsPipelineTest, CommonAttitudeDisturbanceMovesWfovAndNfovTogether) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.scan_start_az_deg = 0.0f;
  config.mission.scan_rate_deg_per_sec = 0.0f;
  config.mission.wide_field_fov_az_deg = 0.02f;
  config.mission.wide_field_fov_el_deg = 0.02f;
  config.mission.narrow_field_fov_az_deg = 0.1f;
  config.mission.narrow_field_fov_el_deg = 0.1f;
  config.policy.pointing_disturbance.common_attitude_sigma_deg = 5.0f;
  config.policy.pointing_disturbance.common_attitude_correlation_time_s = 1.0f;
  config.policy.pointing_disturbance.random_seed = 53U;

  sbirs_sensor::pipeline::SbirsPointingDisturbance probe(1, 53U);
  sbirs_sensor::pipeline::SbirsPointingDisturbanceParameters parameters;
  parameters.common_attitude_sigma_deg = 5.0;
  parameters.common_attitude_correlation_time_s = 1.0;
  ASSERT_TRUE(probe.Advance(1.0, parameters));
  sbirs_sensor::pipeline::SbirsPointingDisturbanceSample expected;
  ASSERT_TRUE(probe.Sample(0, parameters, &expected));
  ASSERT_GT(std::hypot(expected.common.azimuth_deg, expected.common.elevation_deg), 0.02);

  const auto input = sbirs_sensor::session::SbirsCycleInputBuilder()
                         .WithCycleIndex(1U)
                         .WithDeltaTimeSec(1.0f)
                         .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                         .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                         .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                         .AddTarget(HotTargetAtAngles(71U, expected.common.azimuth_deg,
                                                      expected.common.elevation_deg))
                         .Build();
  sbirs_sensor::pipeline::SbirsPipeline disturbed(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const auto disturbed_result = disturbed.RunCycle(input);
  ASSERT_EQ(disturbed_result.detections.size(), 1U);
  EXPECT_EQ(disturbed_result.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);

  config.policy.pointing_disturbance.common_attitude_sigma_deg = 0.0f;
  sbirs_sensor::pipeline::SbirsPipeline nominal(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  EXPECT_TRUE(nominal.RunCycle(input).detections.empty());
}

TEST(SbirsPipelineTest, ChannelDisturbanceContributesToTrackingPointingError) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_field_fov_az_deg = 10.0f;
  config.mission.narrow_field_fov_el_deg = 10.0f;
  config.policy.pointing_disturbance.channel_vibration_amplitude_deg = 0.5f;
  config.policy.pointing_disturbance.channel_vibration_frequency_hz = 0.25f;
  config.policy.pointing_disturbance.random_seed = 59U;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  auto input = sbirs_sensor::session::SbirsCycleInputBuilder()
                   .WithCycleIndex(1U)
                   .WithDeltaTimeSec(1.0f)
                   .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                   .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                   .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                   .AddTarget(HotTarget(72U, 0.0))
                   .Build();
  ASSERT_EQ(pipeline.RunCycle(input).detections.size(), 1U);
  input.cycle_index = 2U;
  const auto tracking = pipeline.RunCycle(input);
  ASSERT_EQ(tracking.detections.size(), 1U);
  EXPECT_TRUE(tracking.detections.front().attribution.has_nfov_tracking_diagnostics);
  EXPECT_GT(tracking.detections.front().attribution.nfov_pointing_error_deg, 1.0e-4f);
}

TEST(SbirsPipelineTest, LockedTargetProducesEstimatedTrack) {
  const sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(7U, 0.0))
          .Build();
  pipeline.RunCycle(input);
  input.cycle_index = 2U;
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  ASSERT_FALSE(result.detections.empty());
  EXPECT_EQ(result.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack);
  // 持续跟踪走 EKF 估计，输出角度应有限（滤波未发散）
  EXPECT_TRUE(std::isfinite(result.detections.front().record.azimuth_rad));
  EXPECT_TRUE(std::isfinite(result.detections.front().record.elevation_rad));
  EXPECT_EQ(result.detections.front().attribution.tracking_source,
            sbirs_sensor::attribution::SbirsTrackingSource::kEstimated);
  EXPECT_TRUE(result.detections.front().attribution.has_estimation_nis);
  EXPECT_TRUE(std::isfinite(result.detections.front().attribution.estimation_nis));
  EXPECT_FALSE(result.detections.front().attribution.estimation_nis_gate_exceeded);
  EXPECT_TRUE(result.detections.front().attribution.has_nfov_tracking_diagnostics);
  EXPECT_TRUE(result.detections.front().attribution.nfov_geometry_gate_passed);
  EXPECT_TRUE(result.detections.front().attribution.nfov_snr_gate_passed);
}

TEST(SbirsPipelineTest, TrackingGateCoastsOnceThenRecoversWithoutRawMeasurement) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_field_fov_az_deg = 1.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 0.1f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(77U, 0.0))
          .Build();
  pipeline.RunCycle(input);

  input.cycle_index = 2U;
  input.scene[0] = HotTarget(77U, 35000.0);
  const auto coast = pipeline.RunCycle(input);
  ASSERT_EQ(coast.detections.size(), 1U);
  EXPECT_FALSE(coast.detections[0].record.detected);
  EXPECT_TRUE(coast.detections[0].attribution.nfov_tracking_coasting);
  EXPECT_EQ(coast.detections[0].attribution.nfov_tracking_gate_failure_count, 1U);
  EXPECT_EQ(pipeline.CaptureRuntimeState().nfov_scheduler.target_to_cue_source.count(77U), 1U);

  input.cycle_index = 3U;
  input.scene[0] = HotTarget(77U, 0.0);
  const auto recovered = pipeline.RunCycle(input);
  ASSERT_EQ(recovered.detections.size(), 1U);
  EXPECT_TRUE(recovered.detections[0].record.detected);
  EXPECT_EQ(recovered.detections[0].record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack);
  EXPECT_EQ(recovered.detections[0].attribution.nfov_tracking_gate_failure_count, 0U);
}

TEST(SbirsPipelineTest, ConsecutiveTrackingGateFailuresReleaseLock) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_field_fov_az_deg = 1.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 0.1f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(78U, 0.0))
          .Build();
  pipeline.RunCycle(input);
  input.scene[0] = HotTarget(78U, 35000.0);
  input.cycle_index = 2U;
  pipeline.RunCycle(input);
  input.cycle_index = 3U;
  const auto lost = pipeline.RunCycle(input);

  ASSERT_EQ(lost.detections.size(), 1U);
  EXPECT_FALSE(lost.detections[0].record.detected);
  EXPECT_FALSE(lost.detections[0].attribution.nfov_tracking_coasting);
  EXPECT_EQ(lost.detections[0].attribution.capture_failure_reason,
            sbirs_sensor::attribution::SbirsCaptureFailureReason::kNfovTrackingGateLost);
  EXPECT_EQ(pipeline.CaptureRuntimeState().nfov_scheduler.target_to_cue_source.count(78U), 0U);
}

// --- 宽窄切换连续命中计数器（3.2.1.3.2.1 前置条件） ---

TEST(SbirsPipelineTest, ConsecutiveHitGateDelaysHandoverUntilThresholdReached) {
  // required=2：首周期仅输出宽场检测（命中 1 次，被切换门挡下），第二周期命中达标
  // 才进入 NFOV 首捕。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.policy.scheduler.wide_to_narrow_required_consecutive_hits = 2;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(81U, 0.0))
          .Build();

  const auto first = pipeline.RunCycle(input);
  ASSERT_EQ(first.detections.size(), 1U);
  EXPECT_EQ(first.detections[0].record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kWideFieldSearch);
  EXPECT_TRUE(first.detections[0].record.detected);
  EXPECT_EQ(pipeline.CaptureRuntimeState().wfov_consecutive_hits.count(81U), 1U);
  EXPECT_EQ(pipeline.CaptureRuntimeState().wfov_consecutive_hits.at(81U), 1U);
  EXPECT_EQ(pipeline.CaptureRuntimeState().nfov_scheduler.target_to_cue_source.count(81U), 0U);

  input.cycle_index = 2U;
  const auto second = pipeline.RunCycle(input);
  ASSERT_EQ(second.detections.size(), 1U);
  EXPECT_EQ(second.detections[0].record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
  // 捕获成功后计数清零（丢锁需重新积累）。
  EXPECT_EQ(pipeline.CaptureRuntimeState().wfov_consecutive_hits.count(81U), 0U);
}

TEST(SbirsPipelineTest, ConsecutiveHitCounterResetsOnWfovGateFailure) {
  // required=2：首周期命中 1 次；第二周期目标移出宽场视场 → 计数清零；第三周期回到
  // 视场仅命中 1 次，仍不调度；第四周期命中 2 次才完成 NFOV 首捕。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.policy.scheduler.wide_to_narrow_required_consecutive_hits = 2;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(82U, 0.0, 0.0))
          .Build();

  pipeline.RunCycle(input);
  EXPECT_EQ(pipeline.CaptureRuntimeState().wfov_consecutive_hits.at(82U), 1U);

  input.cycle_index = 2U;
  input.scene[0] = HotTargetAtAngles(82U, 40.0, 0.0);  // 宽场视场外 → wfov 门失败
  const auto excluded = pipeline.RunCycle(input);
  ASSERT_TRUE(excluded.detections.empty());
  EXPECT_EQ(pipeline.CaptureRuntimeState().wfov_consecutive_hits.count(82U), 0U);

  input.cycle_index = 3U;
  input.scene[0] = HotTargetAtAngles(82U, 0.0, 0.0);
  const auto third = pipeline.RunCycle(input);
  EXPECT_EQ(third.detections[0].record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kWideFieldSearch);
  EXPECT_EQ(pipeline.CaptureRuntimeState().wfov_consecutive_hits.at(82U), 1U);

  input.cycle_index = 4U;
  const auto fourth = pipeline.RunCycle(input);
  EXPECT_EQ(fourth.detections[0].record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
}

TEST(SbirsPipelineTest, DefaultRequiredHitsKeepsSingleCycleCapture) {
  // 默认 required=1：单周期命中即调度捕获，与既有行为一致（计数进快照）。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(83U, 0.0))
          .Build();

  const auto result = pipeline.RunCycle(input);
  ASSERT_EQ(result.detections.size(), 1U);
  EXPECT_EQ(result.detections[0].record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
  // 捕获即清零：快照无残留计数。
  EXPECT_EQ(pipeline.CaptureRuntimeState().wfov_consecutive_hits.count(83U), 0U);
}

TEST(SbirsPipelineTest, TrackingCoastSnapshotRestoreMatchesUninterrupted) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_field_fov_az_deg = 1.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 0.1f;
  sbirs_sensor::pipeline::SbirsPipeline uninterrupted(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  auto input = sbirs_sensor::session::SbirsCycleInputBuilder()
                   .WithCycleIndex(1U)
                   .WithDeltaTimeSec(1.0f)
                   .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                   .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                   .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                   .AddTarget(HotTarget(79U, 0.0))
                   .Build();
  uninterrupted.RunCycle(input);
  input.cycle_index = 2U;
  input.scene[0] = HotTarget(79U, 35000.0);
  uninterrupted.RunCycle(input);
  const auto coast_snapshot = uninterrupted.CaptureRuntimeState();
  ASSERT_EQ(coast_snapshot.pointing_coordinator.tracking_gate_failure_counts.at(79U), 1U);

  sbirs_sensor::pipeline::SbirsPipeline restored(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  ASSERT_TRUE(restored.RestoreRuntimeState(coast_snapshot));
  input.cycle_index = 3U;
  input.scene[0] = HotTarget(79U, 0.0);
  const auto expected = uninterrupted.RunCycle(input);
  const auto actual = restored.RunCycle(input);
  ASSERT_EQ(expected.detections.size(), 1U);
  ASSERT_EQ(actual.detections.size(), 1U);
  EXPECT_EQ(actual.detections[0].record.detected, expected.detections[0].record.detected);
  EXPECT_FLOAT_EQ(actual.detections[0].record.azimuth_rad,
                  expected.detections[0].record.azimuth_rad);
  EXPECT_EQ(actual.detections[0].attribution.nfov_tracking_gate_failure_count, 0U);
}

TEST(SbirsPipelineTest, StrictTruthAssistedStillUsesActualPointingGate) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kStrictTruthAssisted;
  config.mission.narrow_field_fov_az_deg = 1.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 0.1f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  auto input = sbirs_sensor::session::SbirsCycleInputBuilder()
                   .WithCycleIndex(1U)
                   .WithDeltaTimeSec(1.0f)
                   .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                   .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                   .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                   .AddTarget(HotTarget(80U, 0.0))
                   .Build();
  pipeline.RunCycle(input);
  input.cycle_index = 2U;
  input.scene[0] = HotTarget(80U, 35000.0);
  const auto coast = pipeline.RunCycle(input);
  ASSERT_EQ(coast.detections.size(), 1U);
  EXPECT_FALSE(coast.detections[0].record.detected);
  EXPECT_EQ(coast.detections[0].attribution.tracking_source,
            sbirs_sensor::attribution::SbirsTrackingSource::kStrictTruthAssisted);
  EXPECT_TRUE(coast.detections[0].attribution.nfov_tracking_coasting);
  EXPECT_FALSE(coast.detections[0].attribution.has_estimation_nis);
}

TEST(SbirsPipelineTest, TrackingSnrGateCanCoastWhileGeometryPasses) {
  const sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  auto input = sbirs_sensor::session::SbirsCycleInputBuilder()
                   .WithCycleIndex(1U)
                   .WithDeltaTimeSec(1.0f)
                   .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                   .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                   .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                   .AddTarget(HotTarget(81U, 0.0))
                   .Build();
  pipeline.RunCycle(input);
  input.cycle_index = 2U;
  input.scene[0].radiant_intensity_w_per_sr = 0.0;
  const auto coast = pipeline.RunCycle(input);
  ASSERT_EQ(coast.detections.size(), 1U);
  EXPECT_TRUE(coast.detections[0].attribution.nfov_geometry_gate_passed);
  EXPECT_FALSE(coast.detections[0].attribution.nfov_snr_gate_passed);
  EXPECT_TRUE(coast.detections[0].attribution.nfov_tracking_coasting);
}

TEST(SbirsPipelineTest, StaticSettleErrorOffsetsEffectiveTrackingCenter) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_field_fov_az_deg = 3.0f;
  config.mission.narrow_pointing_settle_error_deg = 1.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  auto input = sbirs_sensor::session::SbirsCycleInputBuilder()
                   .WithCycleIndex(1U)
                   .WithDeltaTimeSec(1.0f)
                   .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                   .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                   .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                   .AddTarget(HotTarget(82U, 0.0))
                   .Build();
  pipeline.RunCycle(input);
  input.cycle_index = 2U;
  input.scene[0] = HotTarget(82U, -10472.0);
  const auto coast = pipeline.RunCycle(input);
  ASSERT_EQ(coast.detections.size(), 1U);
  EXPECT_FALSE(coast.detections[0].attribution.nfov_geometry_gate_passed);
  EXPECT_GT(coast.detections[0].attribution.nfov_pointing_error_deg, 1.5f);
}

TEST(SbirsPipelineTest, ConsecutiveNisGateExceededReleasesEstimatedTrackLock) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.policy.tracking.nis_gate_loss_cycles = 1U;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(7U, 0.0))
          .Build();
  pipeline.RunCycle(input);

  input.cycle_index = 2U;
  // 保持目标仍位于 ±2.5° NFOV 内，使本测试只触发量测后的 NIS 门，而非先被几何门拦截。
  input.scene[0] = HotTarget(7U, 30000.0);
  const sbirs_sensor::pipeline::SbirsPipelineResult lost = pipeline.RunCycle(input);
  ASSERT_FALSE(lost.detections.empty());
  EXPECT_EQ(lost.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack);
  EXPECT_FALSE(lost.detections.front().record.detected);
  EXPECT_TRUE(lost.detections.front().attribution.has_estimation_nis);
  EXPECT_TRUE(lost.detections.front().attribution.estimation_nis_gate_exceeded);
  EXPECT_EQ(lost.detections.front().attribution.capture_failure_reason,
            sbirs_sensor::attribution::SbirsCaptureFailureReason::kEstimationNisGateLost);

  input.cycle_index = 3U;
  input.scene[0] = HotTarget(7U, 0.0);
  const sbirs_sensor::pipeline::SbirsPipelineResult reacquired = pipeline.RunCycle(input);
  ASSERT_FALSE(reacquired.detections.empty());
  EXPECT_EQ(reacquired.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
  EXPECT_TRUE(reacquired.detections.front().record.detected);
}

// 单镜筒轮转（2026-09-02）：优先级（SNR↓→距离↑→ID↑）决定入列顺序，轮转首服务
// 取 ID 升序；同帧对两目标同周期完成捕获（窗口 + 免转动），不再有"强者独占资源"。
TEST(SbirsSchedulerTest, PriorityOrdersEntryWhileRotationServesLowestIdFirst) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsSceneTarget weak = HotTarget(1U, 0.0);
  weak.radiant_intensity_w_per_sr = 1.0e7;
  sbirs_sensor::session::SbirsSceneTarget strong = HotTarget(2U, 1000.0);
  strong.radiant_intensity_w_per_sr = 5.0e7;
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(weak)
          .AddTarget(strong)
          .Build();
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  std::size_t acquisitions = 0U;
  for (const auto& detection : result.detections) {
    if (detection.record.observation_stage ==
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition &&
        detection.record.detected) {
      ++acquisitions;
    }
  }
  EXPECT_EQ(acquisitions, 2U);
  const auto snapshot = pipeline.CaptureRuntimeState();
  EXPECT_EQ(snapshot.target_states.at(1U),
            sbirs_sensor::pipeline::SbirsTargetState::kEstimatedTracking);
  EXPECT_EQ(snapshot.target_states.at(2U),
            sbirs_sensor::pipeline::SbirsTargetState::kEstimatedTracking);
}

// NFOV 首次捕获 raw 必须沿用带噪测量，而不是把通过 eligibility 的目标真值直写为观测；
// 相同 random_seed 的两次独立 pipeline 仍应产生相同测量（replay 可复现）。
TEST(SbirsPipelineTest, NfovAcquisitionRawUsesNoisyMeasurementAndIsReproducible) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.policy.error_model.attitude_sigma_deg = 0.5f;  // 启用姿态误差
  config.policy.error_model.orbit_sigma_deg = 0.0f;
  config.policy.error_model.fov_sigma_deg = 0.0f;
  config.policy.error_model.range_fraction_sigma = 0.0f;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(7U, 0.0))
          .Build();

  sbirs_sensor::pipeline::SbirsPipeline first(sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::pipeline::SbirsPipeline second(sbirs_sensor::runtime::MapSessionToInternal(config));
  const sbirs_sensor::pipeline::SbirsPipelineResult r1 = first.RunCycle(input);
  const sbirs_sensor::pipeline::SbirsPipelineResult r2 = second.RunCycle(input);

  ASSERT_EQ(r1.detections.size(), 1U);
  ASSERT_EQ(r1.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
  EXPECT_NE(r1.detections.front().record.azimuth_rad, 0.0f);
  EXPECT_EQ(r1.detections.front().attribution.target_id, 7U);
  EXPECT_EQ(r1.detections.front().attribution.tracking_source,
            sbirs_sensor::attribution::SbirsTrackingSource::kEstimated);
  // 相同 seed → 相同输出。
  EXPECT_FLOAT_EQ(r1.detections.front().record.azimuth_rad,
                  r2.detections.front().record.azimuth_rad);
}

// design cue 延迟外推：横向高速目标在 narrow_cue_latency_s 期间移出 NFOV，
// 首次捕获应失败，并产出 kNfovAcquisitionFailed 诊断 attribution（detected=false）。
TEST(SbirsPipelineTest, CueLatencyWithCrossVelocityCausesAcquisitionFailure) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_cue_latency_s = 1.0f;     // 1s 延迟
  config.mission.narrow_field_fov_az_deg = 2.0f;  // 收窄 NFOV 使外推后易出界
  config.policy.error_model.attitude_sigma_deg = 0.0f;

  sbirs_sensor::session::SbirsSceneTarget target = HotTarget(7U, 0.0);
  // 横向速度：1s 后在 y 方向移动约 1500 km，视线方位角显著偏移。
  target.velocity_ecef_m_per_s = Vector(0.0, 1500000.0, 0.0);
  target.has_velocity_ecef_m_per_s = true;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();

  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);

  // 捕获失败的目标回退为 WFOV 候选，故同时产出：
  //   - 一条 detected=false 的 NFOV 捕获失败诊断 (kNfovAcquisitionFailed)
  //   - 一条 detected=true 的 WFOV 搜索记录 (kSchedulerSkipped，因无其它目标，无跳过标记)
  bool found_failure = false;
  for (const sbirs_sensor::pipeline::SbirsPipelineDetection& detection : result.detections) {
    if (!detection.record.detected) {
      found_failure = true;
      EXPECT_EQ(detection.attribution.capture_failure_reason,
                sbirs_sensor::attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed);
      EXPECT_EQ(detection.record.observation_stage,
                sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
    }
  }
  EXPECT_TRUE(found_failure);
}

// 无速度（has_velocity_ecef_m_per_s=false）时，即便 narrow_cue_latency_s>0，
// 行为应与无延迟一致：成功捕获（零回归）。
TEST(SbirsPipelineTest, CueLatencyWithoutVelocityKeepsBaselineCapture) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_cue_latency_s = 1.0f;  // 延迟非 0 但目标无速度

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(7U, 0.0))  // 默认 has_velocity=false
          .Build();

  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);

  ASSERT_FALSE(result.detections.empty());
  EXPECT_TRUE(result.detections.front().record.detected);
  EXPECT_EQ(result.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
}

TEST(SbirsPipelineTest, MeasurementCvCueCapturesOnSecondObservationAndRestores) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_cue_latency_s = 1.0f;
  config.mission.narrow_field_fov_az_deg = 1.0f;
  sbirs_sensor::pipeline::SbirsPipeline uninterrupted(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  sbirs_sensor::session::SbirsSceneTarget target = HotTarget(17U, 0.0);
  target.velocity_ecef_m_per_s = Vector(0.0, 20000.0, 0.0);
  target.has_velocity_ecef_m_per_s = true;
  sbirs_sensor::session::SbirsCycleInput first =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();
  const auto first_result = uninterrupted.RunCycle(first);
  ASSERT_FALSE(first_result.detections.empty());
  EXPECT_FALSE(first_result.detections.front().record.detected);
  const auto first_snapshot = uninterrupted.CaptureRuntimeState();
  ASSERT_EQ(first_snapshot.cue_predictor.targets.count(17U), 1U);

  target.position_ecef_m.y = 20000.0;
  sbirs_sensor::session::SbirsCycleInput second =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(2U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();
  const auto uninterrupted_result = uninterrupted.RunCycle(second);
  ASSERT_FALSE(uninterrupted_result.detections.empty());
  EXPECT_TRUE(uninterrupted_result.detections.front().record.detected);
  EXPECT_EQ(uninterrupted_result.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);

  sbirs_sensor::pipeline::SbirsPipeline restored(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  ASSERT_TRUE(restored.RestoreRuntimeState(first_snapshot));
  const auto restored_result = restored.RunCycle(second);
  ASSERT_EQ(restored_result.detections.size(), uninterrupted_result.detections.size());
  EXPECT_EQ(restored_result.detections.front().record.detected,
            uninterrupted_result.detections.front().record.detected);
  EXPECT_EQ(restored.CaptureRuntimeState().cue_predictor.targets.count(17U), 0U);
}

TEST(SbirsPipelineTest, SchedulerSkippedCandidateAccumulatesCueHistoryUntilChannelFrees) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_cue_latency_s = 1.0f;
  config.mission.narrow_field_fov_az_deg = 1.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  sbirs_sensor::session::SbirsSceneTarget locked = HotTarget(1U, 0.0);
  sbirs_sensor::session::SbirsSceneTarget moving = HotTarget(2U, 0.0);
  moving.velocity_ecef_m_per_s = Vector(0.0, 20000.0, 0.0);
  moving.has_velocity_ecef_m_per_s = true;
  pipeline.RunCycle(sbirs_sensor::session::SbirsCycleInputBuilder()
                        .WithCycleIndex(1U)
                        .WithDeltaTimeSec(1.0f)
                        .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                        .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                        .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                        .AddTarget(locked)
                        .Build());
  pipeline.RunCycle(sbirs_sensor::session::SbirsCycleInputBuilder()
                        .WithCycleIndex(2U)
                        .WithDeltaTimeSec(1.0f)
                        .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                        .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                        .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                        .AddTarget(locked)
                        .AddTarget(moving)
                        .Build());
  moving.position_ecef_m.y = 20000.0;
  pipeline.RunCycle(sbirs_sensor::session::SbirsCycleInputBuilder()
                        .WithCycleIndex(3U)
                        .WithDeltaTimeSec(1.0f)
                        .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                        .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                        .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                        .AddTarget(locked)
                        .AddTarget(moving)
                        .Build());
  ASSERT_EQ(pipeline.CaptureRuntimeState().cue_predictor.targets.count(2U), 1U);

  locked.active = false;
  moving.position_ecef_m.y = 40000.0;
  const auto acquired = pipeline.RunCycle(sbirs_sensor::session::SbirsCycleInputBuilder()
                                              .WithCycleIndex(4U)
                                              .WithDeltaTimeSec(1.0f)
                                              .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                                              .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                                              .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                                              .AddTarget(locked)
                                              .AddTarget(moving)
                                              .Build());
  bool found_acquisition = false;
  for (const auto& detection : acquired.detections) {
    if (detection.attribution.target_id == 2U && detection.record.detected &&
        detection.record.observation_stage ==
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition) {
      found_acquisition = true;
    }
  }
  EXPECT_TRUE(found_acquisition);
}

TEST(SbirsPipelineTest, MissingUnboundCandidateClearsCueHistory) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_cue_latency_s = 1.0f;
  // 窄场压到 0.1° 且候选带横向速度（cue 延迟外推出界 → 捕获几何门失败）：候选保持
  // 未锁定身份（cue 历史保留），缺席后清除。
  config.mission.narrow_field_fov_az_deg = 0.1f;
  config.mission.narrow_field_fov_el_deg = 0.1f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const sbirs_sensor::session::SbirsSceneTarget locked = HotTarget(1U, 0.0);
  sbirs_sensor::session::SbirsSceneTarget waiting = HotTarget(2U, 5000.0);
  waiting.velocity_ecef_m_per_s = Vector(0.0, 1500000.0, 0.0);
  waiting.has_velocity_ecef_m_per_s = true;
  pipeline.RunCycle(sbirs_sensor::session::SbirsCycleInputBuilder()
                        .WithCycleIndex(1U)
                        .WithDeltaTimeSec(1.0f)
                        .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                        .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                        .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                        .AddTarget(locked)
                        .Build());
  pipeline.RunCycle(sbirs_sensor::session::SbirsCycleInputBuilder()
                        .WithCycleIndex(2U)
                        .WithDeltaTimeSec(1.0f)
                        .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                        .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                        .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                        .AddTarget(locked)
                        .AddTarget(waiting)
                        .Build());
  ASSERT_EQ(pipeline.CaptureRuntimeState().cue_predictor.targets.count(2U), 1U);

  pipeline.RunCycle(sbirs_sensor::session::SbirsCycleInputBuilder()
                        .WithCycleIndex(3U)
                        .WithDeltaTimeSec(1.0f)
                        .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                        .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                        .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                        .AddTarget(locked)
                        .Build());
  const auto snapshot = pipeline.CaptureRuntimeState();
  EXPECT_EQ(snapshot.cue_predictor.targets.count(2U), 0U);
  EXPECT_EQ(snapshot.target_states.at(2U), sbirs_sensor::pipeline::SbirsTargetState::kLost);
}

TEST(SbirsPipelineTest, ApplyingSameConfigPreservesAccumulatedState) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_cue_latency_s = 1.0f;
  config.mission.narrow_field_fov_az_deg = 1.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsSceneTarget target = HotTarget(23U, 0.0);
  target.velocity_ecef_m_per_s = Vector(0.0, 20000.0, 0.0);
  target.has_velocity_ecef_m_per_s = true;
  pipeline.RunCycle(sbirs_sensor::session::SbirsCycleInputBuilder()
                        .WithCycleIndex(1U)
                        .WithDeltaTimeSec(1.0f)
                        .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                        .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                        .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                        .AddTarget(target)
                        .Build());
  ASSERT_EQ(pipeline.CaptureRuntimeState().cue_predictor.targets.count(23U), 1U);

  pipeline.ApplyConfig(sbirs_sensor::runtime::MapSessionToInternal(config), {});
  const auto snapshot = pipeline.CaptureRuntimeState();
  EXPECT_EQ(snapshot.cue_predictor.targets.count(23U), 1U);
  EXPECT_EQ(snapshot.target_states.count(23U), 1U);
  EXPECT_TRUE(snapshot.pointing_coordinator.actuator_initialized);
  EXPECT_TRUE(snapshot.pointing_coordinator.actuator.initialized);
}

TEST(SbirsPipelineTest, CircularScanSupportsDirectionBoundaryAndMultipleWraps) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.scan_start_az_deg = 170.0f;
  config.mission.scan_span_deg = 40.0f;
  config.mission.scan_rate_deg_per_sec = 15.0f;
  sbirs_sensor::pipeline::SbirsPipeline increasing(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  // 输出为 ECI 弧度 [0, 2π)：185°（对称 -175° 折入 [0,360)）→ 3.2289 rad。
  EXPECT_FLOAT_EQ(increasing.RunCycle(TwoTargetInput(1U)).scan_azimuth_rad, 3.2288592f);

  config.mission.scan_start_az_deg = -170.0f;
  config.mission.scan_direction = sbirs_sensor::config::SbirsScanDirection::kDecreasingAzimuth;
  sbirs_sensor::pipeline::SbirsPipeline decreasing(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  // 175° → 3.0543 rad。
  EXPECT_FLOAT_EQ(decreasing.RunCycle(TwoTargetInput(1U)).scan_azimuth_rad, 3.0543262f);

  config.mission.scan_start_az_deg = -180.0f;
  config.mission.scan_span_deg = 360.0f;
  config.mission.scan_direction = sbirs_sensor::config::SbirsScanDirection::kIncreasingAzimuth;
  config.mission.scan_rate_deg_per_sec = 100.0f;
  sbirs_sensor::pipeline::SbirsPipeline full_circle(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  auto input = TwoTargetInput(1U);
  input.dt_sec = 10.0f;
  // 100° → 1.7453 rad。
  EXPECT_FLOAT_EQ(full_circle.RunCycle(input).scan_azimuth_rad, 1.7453293f);
  EXPECT_FLOAT_EQ(full_circle.CaptureRuntimeState().scan_phase_deg, 280.0f);
}

// 单镜筒分时轮转（2026-09-02）：分离双目标（±3°，6° 间隔 > 4° 窄场）交替伺服——
// 被服务目标量测（转动 6°@30°/s 后稳定 ≈0.80s → 8 帧），轮空目标同帧外滑行
// （帧数 0、门失败 ≤1 < 2 不丢锁）。容量由轮转物理涌现，无配置上限。
TEST(SbirsPipelineTest, RotationTimeSharesSeparatedTargets) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_field_fov_az_deg = 4.0f;
  config.mission.narrow_field_fov_el_deg = 4.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 30.0f;
  config.mission.frame_rate_hz = 10.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, -3.0, 0.0))
          .AddTarget(HotTargetAtAngles(2U, 3.0, 0.0))
          .Build();

  // 前两周期：轮转先后完成两目标首捕（单镜筒一次只服务一个方向）。
  pipeline.RunCycle(input);
  input.cycle_index = 2U;
  pipeline.RunCycle(input);
  const auto snapshot = pipeline.CaptureRuntimeState();
  ASSERT_EQ(snapshot.nfov_scheduler.target_to_cue_source.size(), 2U);
  EXPECT_EQ(snapshot.target_states.at(1U),
            sbirs_sensor::pipeline::SbirsTargetState::kEstimatedTracking);
  EXPECT_EQ(snapshot.target_states.at(2U),
            sbirs_sensor::pipeline::SbirsTargetState::kEstimatedTracking);

  // 稳态交替：任一周期恰有一个目标量测（NFOV 跟踪、帧数=稳定时长×帧率）、
  // 另一个滑行（帧数 0）；连续两周期两目标的量测角色互换。
  input.cycle_index = 3U;
  const auto first = pipeline.RunCycle(input);
  input.cycle_index = 4U;
  const auto second = pipeline.RunCycle(input);
  const auto& first_by_target = first.detections;
  const auto& second_by_target = second.detections;
  ASSERT_EQ(first_by_target.size(), 2U);
  ASSERT_EQ(second_by_target.size(), 2U);
  int first_t1_frames = -1;
  int first_t2_frames = -1;
  int second_t1_frames = -1;
  int second_t2_frames = -1;
  for (const auto& detection : first.detections) {
    if (detection.attribution.target_id == 1U) first_t1_frames = detection.attribution.nfov_frame_count;
    if (detection.attribution.target_id == 2U) first_t2_frames = detection.attribution.nfov_frame_count;
  }
  for (const auto& detection : second.detections) {
    if (detection.attribution.target_id == 1U) second_t1_frames = detection.attribution.nfov_frame_count;
    if (detection.attribution.target_id == 2U) second_t2_frames = detection.attribution.nfov_frame_count;
  }
  // 6° 转动 @30°/s：稳定 ≈(6−0.01)/30=0.20s，帧数 = round(10×0.80)=8。
  EXPECT_EQ(first_t1_frames + first_t2_frames, 8);
  EXPECT_EQ(second_t1_frames + second_t2_frames, 8);
  EXPECT_EQ(std::min(first_t1_frames, first_t2_frames), 0);
  EXPECT_EQ(std::min(second_t1_frames, second_t2_frames), 0);
  // 角色互换：上一周期量测者下一周期滑行。
  EXPECT_EQ(first_t1_frames, second_t2_frames);
  EXPECT_NE(first_t1_frames, second_t1_frames);
}

// 同帧免费多跟（2026-09-02）：两目标同处一个窄视场（间隔 1° < 8°）——无论轮转服务
// 谁，另一目标同帧搭车量测，两目标每周期都满帧（10 帧，与单目标口径一致）。
TEST(SbirsPipelineTest, InFrameTargetsRideAlongWithFullFrames) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_field_fov_az_deg = 8.0f;
  config.mission.narrow_field_fov_el_deg = 8.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 30.0f;
  config.mission.frame_rate_hz = 10.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, -0.5, 0.0))
          .AddTarget(HotTargetAtAngles(2U, 0.5, 0.0))
          .Build();

  // 周期 1：先服务目标 1；目标 2 同帧 → 免转动捕获。
  pipeline.RunCycle(input);
  const auto snapshot = pipeline.CaptureRuntimeState();
  ASSERT_EQ(snapshot.nfov_scheduler.target_to_cue_source.size(), 2U);

  input.cycle_index = 2U;
  const auto result = pipeline.RunCycle(input);
  ASSERT_EQ(result.detections.size(), 2U);
  for (const auto& detection : result.detections) {
    EXPECT_EQ(detection.record.observation_stage,
              sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack);
    EXPECT_TRUE(detection.record.detected);
    EXPECT_EQ(detection.attribution.nfov_frame_count, 10);
    EXPECT_FALSE(detection.attribution.nfov_tracking_coasting);
  }
}

TEST(SbirsPipelineTest, StatisticalPatchesResetOnlyTheirConsecutiveCounters) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  pipeline.RunCycle(TwoTargetInput(1U));
  auto seeded = pipeline.CaptureRuntimeState();
  const std::uint64_t target_id = seeded.nfov_scheduler.target_to_cue_source.begin()->first;
  seeded.nis_gate_exceeded_counts[target_id] = 3U;
  seeded.pointing_coordinator.tracking_gate_failure_counts[target_id] = 1U;
  ASSERT_TRUE(pipeline.RestoreRuntimeState(seeded));
  const auto before = pipeline.CaptureRuntimeState();

  config.policy.tracking.process_noise_diff_coeff = 2.0f;
  config.policy.detection.narrow_min_snr_linear = 0.002f;
  sbirs_sensor::runtime::SbirsRuntimeConfigImpact impact;
  impact.reset_nis_gate_counts = true;
  impact.reset_nfov_gate_failure_counts = true;
  pipeline.ApplyConfig(sbirs_sensor::runtime::MapSessionToInternal(config), impact);

  const auto after = pipeline.CaptureRuntimeState();
  EXPECT_TRUE(
      after.filter_states.at(target_id).mean.isApprox(before.filter_states.at(target_id).mean));
  EXPECT_TRUE(after.filter_states.at(target_id).covariance.isApprox(
      before.filter_states.at(target_id).covariance));
  EXPECT_EQ(after.nis_gate_exceeded_counts.at(target_id), 0U);
  EXPECT_EQ(after.pointing_coordinator.tracking_gate_failure_counts.count(target_id), 0U);
  EXPECT_EQ(after.nfov_scheduler.target_to_cue_source, before.nfov_scheduler.target_to_cue_source);
  EXPECT_EQ(after.pointing_coordinator.actuator.initialized,
            before.pointing_coordinator.actuator.initialized);
}

TEST(SbirsPipelineTest, RuntimeSeedsRestartOnlyTheirOwnedRandomStream) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  pipeline.RunCycle(TwoTargetInput(1U));
  const auto initial = pipeline.CaptureRuntimeState();

  config.policy.error_model.random_seed = 99U;
  sbirs_sensor::runtime::SbirsRuntimeConfigImpact measurement_impact;
  measurement_impact.reset_measurement_random_stream = true;
  pipeline.ApplyConfig(sbirs_sensor::runtime::MapSessionToInternal(config), measurement_impact);
  const auto measurement_reset = pipeline.CaptureRuntimeState();
  EXPECT_NE(measurement_reset.wfov_measurement_random_state, 0U);
  EXPECT_NE(measurement_reset.estimated_measurement_random_state, 0U);
  EXPECT_NE(measurement_reset.sensor_like_output_random_state, 0U);
  EXPECT_EQ(measurement_reset.pointing_coordinator.disturbance.base_seed,
            initial.pointing_coordinator.disturbance.base_seed);
  EXPECT_EQ(measurement_reset.nfov_scheduler.target_to_cue_source,
            initial.nfov_scheduler.target_to_cue_source);

  config.policy.pointing_disturbance.random_seed = 101U;
  sbirs_sensor::runtime::SbirsRuntimeConfigImpact pointing_impact;
  pointing_impact.restart_pointing_disturbance = true;
  pipeline.ApplyConfig(sbirs_sensor::runtime::MapSessionToInternal(config), pointing_impact);
  const auto pointing_reset = pipeline.CaptureRuntimeState();
  EXPECT_EQ(pointing_reset.wfov_measurement_random_state,
            measurement_reset.wfov_measurement_random_state);
  EXPECT_EQ(pointing_reset.estimated_measurement_random_state,
            measurement_reset.estimated_measurement_random_state);
  EXPECT_EQ(pointing_reset.sensor_like_output_random_state,
            measurement_reset.sensor_like_output_random_state);
  EXPECT_EQ(pointing_reset.pointing_coordinator.disturbance.base_seed, 101U);
  EXPECT_EQ(pointing_reset.nfov_scheduler.target_to_cue_source,
            measurement_reset.nfov_scheduler.target_to_cue_source);
  EXPECT_EQ(pointing_reset.pointing_coordinator.actuator.initialized,
            measurement_reset.pointing_coordinator.actuator.initialized);
}

TEST(SbirsPipelineTest, RateLimitedPointingSpansCyclesAndRestores) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.scan_start_az_deg = -10.0f;
  config.mission.scan_span_deg = 20.0f;
  config.mission.scan_rate_deg_per_sec = 0.0f;
  config.mission.wide_field_fov_az_deg = 30.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 2.0f;
  sbirs_sensor::pipeline::SbirsPipeline uninterrupted(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const auto make_input = [](std::uint32_t cycle) {
    return sbirs_sensor::session::SbirsCycleInputBuilder()
        .WithCycleIndex(cycle)
        .WithDeltaTimeSec(1.0f)
        .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
        .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
        .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
        .AddTarget(HotTarget(41U, 0.0))
        .Build();
  };
  const auto first = uninterrupted.RunCycle(make_input(1U));
  ASSERT_EQ(first.detections.size(), 1U);
  EXPECT_EQ(first.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kWideFieldSearch);
  EXPECT_EQ(first.detections.front().attribution.nfov_channel_id, 0);
  const auto first_snapshot = uninterrupted.CaptureRuntimeState();
  EXPECT_EQ(first_snapshot.target_states.at(41U),
            sbirs_sensor::pipeline::SbirsTargetState::kAwaitingNfovAcquisition);
  EXPECT_TRUE(first_snapshot.pointing_coordinator.actuator_initialized);

  sbirs_sensor::pipeline::SbirsPipeline restored(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  ASSERT_TRUE(restored.RestoreRuntimeState(first_snapshot));
  const auto uninterrupted_second = uninterrupted.RunCycle(make_input(2U));
  const auto restored_second = restored.RunCycle(make_input(2U));
  ASSERT_EQ(uninterrupted_second.detections.size(), restored_second.detections.size());
  EXPECT_EQ(uninterrupted_second.detections.front().record.observation_stage,
            restored_second.detections.front().record.observation_stage);
  EXPECT_DOUBLE_EQ(
      uninterrupted.CaptureRuntimeState().pointing_coordinator.acquisition_wait_sec.at(41U),
      restored.CaptureRuntimeState().pointing_coordinator.acquisition_wait_sec.at(41U));

  uninterrupted.RunCycle(make_input(3U));
  uninterrupted.RunCycle(make_input(4U));
  const auto acquisition = uninterrupted.RunCycle(make_input(5U));
  ASSERT_EQ(acquisition.detections.size(), 1U);
  EXPECT_EQ(acquisition.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
  const auto tracking = uninterrupted.RunCycle(make_input(6U));
  ASSERT_EQ(tracking.detections.size(), 1U);
  EXPECT_EQ(tracking.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack);
}

TEST(SbirsPipelineTest, PointingTimeoutReleasesWithoutSameCycleReschedule) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.scan_start_az_deg = -10.0f;
  config.mission.scan_span_deg = 20.0f;
  config.mission.scan_rate_deg_per_sec = 0.0f;
  config.mission.wide_field_fov_az_deg = 30.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 90.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const auto make_input = [](std::uint32_t cycle) {
    return sbirs_sensor::session::SbirsCycleInputBuilder()
        .WithCycleIndex(cycle)
        .WithDeltaTimeSec(0.01f)
        .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
        .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
        .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
        .AddTarget(HotTarget(42U, 0.0))
        .Build();
  };
  pipeline.RunCycle(make_input(1U));
  auto snapshot = pipeline.CaptureRuntimeState();
  snapshot.pointing_coordinator.acquisition_wait_sec[42U] = 1.9999;
  ASSERT_TRUE(pipeline.RestoreRuntimeState(snapshot));

  const auto result = pipeline.RunCycle(make_input(2U));
  ASSERT_EQ(result.detections.size(), 1U);
  EXPECT_FALSE(result.detections.front().record.detected);
  EXPECT_EQ(result.detections.front().attribution.capture_failure_reason,
            sbirs_sensor::attribution::SbirsCaptureFailureReason::kNfovPointingTimeout);
  const auto after = pipeline.CaptureRuntimeState();
  EXPECT_EQ(after.nfov_scheduler.target_to_cue_source.count(42U), 0U);
  EXPECT_EQ(after.pointing_coordinator.acquisition_wait_sec.count(42U), 0U);
}

TEST(SbirsPipelineTest, InvalidPointingSnapshotRestoreIsAtomic) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.scan_start_az_deg = -10.0f;
  config.mission.scan_span_deg = 20.0f;
  config.mission.scan_rate_deg_per_sec = 0.0f;
  config.mission.wide_field_fov_az_deg = 30.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 2.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  pipeline.RunCycle(sbirs_sensor::session::SbirsCycleInputBuilder()
                        .WithCycleIndex(1U)
                        .WithDeltaTimeSec(1.0f)
                        .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                        .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                        .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                        .AddTarget(HotTarget(43U, 0.0))
                        .Build());
  const auto before = pipeline.CaptureRuntimeState();
  auto invalid = before;
  // 非法：捕获等待挂在未被调度的目标上（集合与簿记不一致）。
  invalid.pointing_coordinator.acquisition_wait_sec[99U] = 1.0;
  invalid.pointing_coordinator.disturbance.common.random_state = 0U;
  EXPECT_FALSE(pipeline.RestoreRuntimeState(invalid));
  const auto after = pipeline.CaptureRuntimeState();
  EXPECT_EQ(after.nfov_scheduler.target_to_cue_source, before.nfov_scheduler.target_to_cue_source);
  EXPECT_EQ(after.pointing_coordinator.acquisition_wait_sec, before.pointing_coordinator.acquisition_wait_sec);
  EXPECT_EQ(after.pointing_coordinator.actuator.initialized,
            before.pointing_coordinator.actuator.initialized);
  EXPECT_EQ(after.pointing_coordinator.disturbance.common.random_state,
            before.pointing_coordinator.disturbance.common.random_state);
}

TEST(SbirsPipelineTest, InvalidTrackingSnapshotRestoreIsAtomic) {
  const sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  pipeline.RunCycle(sbirs_sensor::session::SbirsCycleInputBuilder()
                        .WithCycleIndex(1U)
                        .WithDeltaTimeSec(1.0f)
                        .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                        .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                        .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                        .AddTarget(HotTarget(44U, 0.0))
                        .Build());
  const auto before = pipeline.CaptureRuntimeState();
  ASSERT_EQ(before.filter_states.count(44U), 1U);

  auto missing_filter = before;
  missing_filter.filter_states.erase(44U);
  EXPECT_FALSE(pipeline.RestoreRuntimeState(missing_filter));

  auto non_finite_filter = before;
  non_finite_filter.filter_states.at(44U).mean(0) = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(pipeline.RestoreRuntimeState(non_finite_filter));

  auto invalid_random_state = before;
  invalid_random_state.sensor_like_output_random_state = 0U;
  EXPECT_FALSE(pipeline.RestoreRuntimeState(invalid_random_state));

  const auto after = pipeline.CaptureRuntimeState();
  EXPECT_EQ(after.target_states, before.target_states);
  EXPECT_EQ(after.nfov_scheduler.target_to_cue_source, before.nfov_scheduler.target_to_cue_source);
  EXPECT_EQ(after.wfov_measurement_random_state, before.wfov_measurement_random_state);
  EXPECT_EQ(after.estimated_measurement_random_state,
            before.estimated_measurement_random_state);
  EXPECT_EQ(after.sensor_like_output_random_state, before.sensor_like_output_random_state);
  ASSERT_EQ(after.filter_states.count(44U), 1U);
  EXPECT_TRUE(after.filter_states.at(44U).mean.isApprox(before.filter_states.at(44U).mean));
  EXPECT_TRUE(
      after.filter_states.at(44U).covariance.isApprox(before.filter_states.at(44U).covariance));
}

TEST(SbirsPipelineTest, InvalidImmSnapshotRestoreIsAtomic) {
  const sbirs_sensor::config::SbirsSessionConfig config = ImmMultiTargetConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  pipeline.RunCycle(TwoTargetInput(1U));
  const auto before = pipeline.CaptureRuntimeState();
  ASSERT_EQ(before.imm_snapshots.count(1U), 1U);

  auto missing_imm = before;
  missing_imm.imm_snapshots.erase(1U);
  EXPECT_FALSE(pipeline.RestoreRuntimeState(missing_imm));

  auto non_finite_weight = before;
  non_finite_weight.imm_snapshots.at(1U).model_weights(0) = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(pipeline.RestoreRuntimeState(non_finite_weight));

  const auto after = pipeline.CaptureRuntimeState();
  EXPECT_EQ(after.target_states, before.target_states);
  ExpectImmTargetStateEqual(after, before, 1U);
  ExpectImmTargetStateEqual(after, before, 2U);
}

// 单镜筒轮转（2026-09-02）：目标 A 已锁定 NFOV，候选 B 被 WFOV 发现后进入捕获
// 等待（Awaiting）而非"资源跳过"——锁定集合无上限，B 保持宽场候选行等待轮转窗口，
// attribution 不再标 kSchedulerSkipped。
TEST(SbirsPipelineTest, LockedTargetLetsOtherCandidateAwaitRotationWindow) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  // B 放到 45° 偏角（>30°/s×1s 转不完）：首窗口"转向中"，保持 Awaiting 等待下窗。
  config.mission.wide_field_fov_az_deg = 90.0f;
  config.mission.wide_field_fov_el_deg = 90.0f;
  config.mission.narrow_field_fov_az_deg = 4.0f;
  config.mission.narrow_field_fov_el_deg = 4.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  // 第一周期：A 进入捕获锁定 NFOV。
  sbirs_sensor::session::SbirsCycleInput first =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(1U, 0.0))
          .Build();
  pipeline.RunCycle(first);

  // 第二周期：A 继续锁定，新增候选 B（45° 偏角，本周期转不完）。
  sbirs_sensor::session::SbirsCycleInput second =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(2U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
          .AddTarget(HotTargetAtAngles(2U, 45.0, 0.0))
          .Build();
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(second);

  // A 产出 NFOV track；B 保持宽场候选行（Awaiting，窄场归属 0），不再标调度跳过。
  bool found_awaiting = false;
  bool found_track = false;
  for (const sbirs_sensor::pipeline::SbirsPipelineDetection& detection : result.detections) {
    if (detection.attribution.target_id == 1U) {
      found_track = true;
      EXPECT_EQ(detection.attribution.capture_failure_reason,
                sbirs_sensor::attribution::SbirsCaptureFailureReason::kNone);
    }
    if (detection.attribution.target_id == 2U) {
      found_awaiting = true;
      EXPECT_EQ(detection.attribution.capture_failure_reason,
                sbirs_sensor::attribution::SbirsCaptureFailureReason::kNone);
      EXPECT_EQ(detection.attribution.nfov_channel_id, 0);  // 已入列等待轮转窗口
    }
  }
  EXPECT_TRUE(found_track);
  EXPECT_TRUE(found_awaiting);
  EXPECT_EQ(pipeline.CaptureRuntimeState().target_states.at(2U),
            sbirs_sensor::pipeline::SbirsTargetState::kAwaitingNfovAcquisition);
}

// 同帧对（2026-09-02 单镜筒）：两目标同处一个窄视场——轮转窗口捕获第一个，
// 第二个同帧免转动捕获；随后两目标逐周期都产出 NFOV track（同帧免费多跟）。
TEST(SbirsPipelineTest, InFramePairAcquiresInOneCycleAndTracksTogether) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  // 第一周期：两个热目标同时进入 WFOV 且同帧，均应被捕获（窗口 + 免转动）。
  sbirs_sensor::session::SbirsCycleInput first =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(1U, 0.0))
          .AddTarget(HotTarget(2U, 5.0))
          .Build();
  const sbirs_sensor::pipeline::SbirsPipelineResult acq = pipeline.RunCycle(first);

  std::size_t acquisitions = 0U;
  for (const sbirs_sensor::pipeline::SbirsPipelineDetection& detection : acq.detections) {
    if (detection.record.observation_stage ==
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition &&
        detection.attribution.capture_failure_reason ==
            sbirs_sensor::attribution::SbirsCaptureFailureReason::kNone) {
      ++acquisitions;
      EXPECT_GE(detection.attribution.nfov_channel_id, 0);
    }
  }
  EXPECT_EQ(acquisitions, 2U);

  // 快照确认两目标都已进入锁定集合（单镜筒无通道区分）。
  const sbirs_sensor::pipeline::SbirsPipelineSnapshot snapshot = pipeline.CaptureRuntimeState();
  ASSERT_EQ(snapshot.nfov_scheduler.target_to_cue_source.count(1U), 1U);
  ASSERT_EQ(snapshot.nfov_scheduler.target_to_cue_source.count(2U), 1U);

  // 第二周期：两目标同帧免费多跟，均产出 NFOV track（kNarrowFieldTrack）。
  sbirs_sensor::session::SbirsCycleInput second =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(2U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(1U, 0.0))
          .AddTarget(HotTarget(2U, 5.0))
          .Build();
  const sbirs_sensor::pipeline::SbirsPipelineResult tracks = pipeline.RunCycle(second);

  std::size_t track_count = 0U;
  for (const sbirs_sensor::pipeline::SbirsPipelineDetection& detection : tracks.detections) {
    if (detection.record.observation_stage ==
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack &&
        detection.attribution.capture_failure_reason ==
            sbirs_sensor::attribution::SbirsCaptureFailureReason::kNone) {
      ++track_count;
      EXPECT_GE(detection.attribution.nfov_channel_id, 0);
    }
  }
  EXPECT_EQ(track_count, 2U);
}

TEST(SbirsPipelineTest, ImmTrackingProducesFiniteState) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.policy.tracking.estimated_backend =
      sbirs_sensor::config::SbirsEstimatedTrackingBackend::kImm;
  config.policy.tracking.imm_model_noise_diff_coeffs = {0.5f, 80.0f};
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(7U, 0.0))
          .Build();
  pipeline.RunCycle(input);

  input.cycle_index = 2U;
  input.scene[0] = HotTarget(7U, 5000.0);
  const sbirs_sensor::pipeline::SbirsPipelineResult tracked = pipeline.RunCycle(input);
  ASSERT_FALSE(tracked.detections.empty());
  EXPECT_EQ(tracked.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack);
  EXPECT_TRUE(tracked.detections.front().record.detected);
  EXPECT_EQ(tracked.detections.front().attribution.tracking_source,
            sbirs_sensor::attribution::SbirsTrackingSource::kEstimated);
  EXPECT_TRUE(tracked.detections.front().attribution.has_estimation_nis);
  EXPECT_TRUE(std::isfinite(tracked.detections.front().attribution.estimation_nis));
}

TEST(SbirsPipelineTest, ImmKeepsIndependentStateForEachCapturedTarget) {
  const sbirs_sensor::config::SbirsSessionConfig config = ImmMultiTargetConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  pipeline.RunCycle(TwoTargetInput(1U));

  const sbirs_sensor::pipeline::SbirsPipelineSnapshot first_snapshot =
      pipeline.CaptureRuntimeState();
  ASSERT_TRUE(first_snapshot.imm_active);
  ASSERT_EQ(first_snapshot.imm_snapshots.size(), 2U);
  ASSERT_EQ(first_snapshot.imm_snapshots.count(1U), 1U);
  ASSERT_EQ(first_snapshot.imm_snapshots.count(2U), 1U);
  for (const auto& entry : first_snapshot.imm_snapshots) {
    ASSERT_EQ(entry.second.model_states.size(), 2U);
  }
  // ECI 旋转（GMST≈0 残余 2.35e-9 rad）把 ECEF y=0/5000 折入 ECI y≈0.019/5000.019，
  // 用近等断言保持"独立状态"语义。
  EXPECT_NEAR(first_snapshot.imm_snapshots.at(1U).model_states[0].state.mean(2), 0.0f, 0.05f);
  EXPECT_NEAR(first_snapshot.imm_snapshots.at(2U).model_states[0].state.mean(2), 5000.0f, 0.05f);
}

TEST(SbirsPipelineTest, ImmMultiTargetUpdatesMatchIndependentRunsAndInputOrder) {
  const sbirs_sensor::config::SbirsSessionConfig config = ImmMultiTargetConfig();
  const sbirs_sensor::session::SbirsVector3M satellite = Vector(7000000.0, 0.0, 0.0);
  // 中译：卫星速度按 GEO 量级给 3 km/s。本例只比较各目标的后验角是否互相独立，速度本身
  //       不参与断言，但必须显式给出——角速率派生会减掉它（见 A1）。
  // 标识：A1 角速率派生引入的入参（2026-10-19）。
  const sbirs_sensor::session::SbirsVector3M satellite_velocity = Vector(0.0, 3000.0, 0.0);
  // 滤波初始化吃管线内部 ECI 场景目标；GMST=0 时 ECI≡ECEF，几何期望不变。
  const sbirs_sensor::pipeline::SbirsEciSceneTarget first =
      sbirs_sensor::pipeline::RotateSceneTargetToEci(HotTarget(1U, 0.0), 0.0);
  const sbirs_sensor::pipeline::SbirsEciSceneTarget second =
      sbirs_sensor::pipeline::RotateSceneTargetToEci(HotTarget(2U, 5000.0), 0.0);
  sbirs_sensor::pipeline::SbirsTrackingCoordinator joint;
  sbirs_sensor::pipeline::SbirsTrackingCoordinator only_first;
  sbirs_sensor::pipeline::SbirsTrackingCoordinator only_second;
  sbirs_sensor::pipeline::SbirsTrackingCoordinator reversed;
  joint.InitializeTarget(1U, first, config.policy.tracking, 0.0f, 0.0f);
  joint.InitializeTarget(2U, second, config.policy.tracking, 0.0f, 0.0f);
  only_first.InitializeTarget(1U, first, config.policy.tracking, 0.0f, 0.0f);
  only_second.InitializeTarget(2U, second, config.policy.tracking, 0.0f, 0.0f);
  reversed.InitializeTarget(2U, second, config.policy.tracking, 0.0f, 0.0f);
  reversed.InitializeTarget(1U, first, config.policy.tracking, 0.0f, 0.0f);

  sbirs_sensor::foundation::SbirsRandomSource joint_random(1U);
  sbirs_sensor::foundation::SbirsRandomSource first_random(1U);
  sbirs_sensor::foundation::SbirsRandomSource second_random(1U);
  sbirs_sensor::foundation::SbirsRandomSource reversed_random(1U);
  const auto joint_first = joint.Update(1U, config.policy, &joint_random, 0.0f, 0.0f, 1.0e6, 0.0f,
                                        1.0f, satellite, satellite_velocity);
  const auto joint_second = joint.Update(2U, config.policy, &joint_random, 0.3f, 0.1f, 1.0e6, 0.0f,
                                         1.0f, satellite, satellite_velocity);
  const auto alone_first =
      only_first.Update(1U, config.policy, &first_random, 0.0f, 0.0f, 1.0e6, 0.0f, 1.0f, satellite,
                        satellite_velocity);
  const auto alone_second =
      only_second.Update(2U, config.policy, &second_random, 0.3f, 0.1f, 1.0e6, 0.0f, 1.0f,
                         satellite, satellite_velocity);
  const auto reversed_second =
      reversed.Update(2U, config.policy, &reversed_random, 0.3f, 0.1f, 1.0e6, 0.0f, 1.0f, satellite,
                      satellite_velocity);
  const auto reversed_first =
      reversed.Update(1U, config.policy, &reversed_random, 0.0f, 0.0f, 1.0e6, 0.0f, 1.0f, satellite,
                      satellite_velocity);
  EXPECT_FLOAT_EQ(joint_first.output_azimuth_deg, alone_first.output_azimuth_deg);
  EXPECT_FLOAT_EQ(joint_first.output_elevation_deg, alone_first.output_elevation_deg);
  EXPECT_FLOAT_EQ(joint_second.output_azimuth_deg, alone_second.output_azimuth_deg);
  EXPECT_FLOAT_EQ(joint_second.output_elevation_deg, alone_second.output_elevation_deg);
  EXPECT_FLOAT_EQ(joint_first.output_azimuth_deg, reversed_first.output_azimuth_deg);
  EXPECT_FLOAT_EQ(joint_second.output_azimuth_deg, reversed_second.output_azimuth_deg);
}

TEST(SbirsPipelineTest, ImmMultiTargetRestorePreservesPerTargetState) {
  const sbirs_sensor::config::SbirsSessionConfig config = ImmMultiTargetConfig();
  sbirs_sensor::pipeline::SbirsPipeline uninterrupted(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  uninterrupted.RunCycle(TwoTargetInput(1U));
  const auto first_snapshot = uninterrupted.CaptureRuntimeState();
  uninterrupted.RunCycle(TwoTargetInput(2U));
  const auto uninterrupted_snapshot = uninterrupted.CaptureRuntimeState();

  sbirs_sensor::pipeline::SbirsPipeline restored(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  ASSERT_TRUE(restored.RestoreRuntimeState(first_snapshot));
  restored.RunCycle(TwoTargetInput(2U));
  const auto restored_snapshot = restored.CaptureRuntimeState();
  EXPECT_FLOAT_EQ(restored_snapshot.scan_phase_deg, uninterrupted_snapshot.scan_phase_deg);
  ExpectImmTargetStateEqual(uninterrupted_snapshot, restored_snapshot, 1U);
  ExpectImmTargetStateEqual(uninterrupted_snapshot, restored_snapshot, 2U);
}

TEST(SbirsPipelineTest, ImmDisabledFallsBackToSingleEkf) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.policy.tracking.estimated_backend =
      sbirs_sensor::config::SbirsEstimatedTrackingBackend::kEkf;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(7U, 0.0))
          .Build();
  pipeline.RunCycle(input);

  input.cycle_index = 2U;
  input.scene[0] = HotTarget(7U, 5000.0);
  const auto result = pipeline.RunCycle(input);
  ASSERT_FALSE(result.detections.empty());
  EXPECT_TRUE(result.detections.front().record.detected);
  EXPECT_TRUE(result.detections.front().attribution.has_estimation_nis);
}

TEST(SbirsPipelineTest, ImmSupportsCaptureRestoreRoundtrip) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.policy.tracking.estimated_backend =
      sbirs_sensor::config::SbirsEstimatedTrackingBackend::kImm;
  config.policy.tracking.imm_model_noise_diff_coeffs = {0.5f, 80.0f};
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(7U, 0.0))
          .Build();
  pipeline.RunCycle(input);

  const auto snapshot = pipeline.CaptureRuntimeState();

  sbirs_sensor::pipeline::SbirsPipeline pipeline2(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  EXPECT_TRUE(pipeline2.RestoreRuntimeState(snapshot));

  input.cycle_index = 2U;
  input.scene[0] = HotTarget(7U, 5000.0);
  const auto result = pipeline2.RunCycle(input);
  ASSERT_FALSE(result.detections.empty());
  EXPECT_TRUE(result.detections.front().record.detected);
}

// 规则 13b：正常执行周期的按目标门控排除必须产出 kInfo 诊断（不属于三写），
// 且不改变探测行为（detections 仍为空、周期正常完成）。

TEST(SbirsPipelineTest, OccultedTargetWritesInfoExclusionDiagnostic) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  // 对侧地表上方（半径 6.5e6 m > 地球半径 6.371e6 m）：LOS 穿过地球 → 遮挡排除。
  sbirs_sensor::session::SbirsSceneTarget target = HotTarget(1U, 0.0);
  target.position_ecef_m = Vector(-6500000.0, 0.0, 0.0);
  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();

  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  EXPECT_TRUE(result.detections.empty());
  const auto* issue = FindIssue(result, "sbirs.target_occulted");
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, sbirs_sensor::session::SbirsIssueSeverity::kInfo);
  EXPECT_NE(issue->message.find("target_id=1"), std::string::npos);
  // 具体门：cause 保持 kNone；message 携带遮挡余量（负值 = 遮挡深度）。
  EXPECT_EQ(issue->cause, sbirs_sensor::session::SbirsIssueCause::kNone);
  EXPECT_NE(issue->message.find("occultation_margin_m="), std::string::npos);
  // 实体机器可读关联（规则 14e）：单目标 scene[0] → entity_index=0。
  EXPECT_EQ(issue->location.kind, oneq::foundation::ValidationLocationKind::kSceneEntity);
  EXPECT_EQ(issue->location.entity_index, 0U);
}

TEST(SbirsPipelineTest, OutOfRangeTargetWritesInfoExclusionDiagnostic) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.max_range_m = 500000.0f;  // 收紧距离门：HotTarget（range 1e6 m）超限
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(1U, 0.0))
          .Build();

  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  EXPECT_TRUE(result.detections.empty());
  const auto* issue = FindIssue(result, "sbirs.target_out_of_range");
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, sbirs_sensor::session::SbirsIssueSeverity::kInfo);
  EXPECT_NE(issue->message.find("target_id=1"), std::string::npos);
  // 具体门：cause 保持 kNone；message 携带距带边余量。
  EXPECT_EQ(issue->cause, sbirs_sensor::session::SbirsIssueCause::kNone);
  EXPECT_NE(issue->message.find("range_margin_m="), std::string::npos);
  // 实体机器可读关联（规则 14e）：单目标 scene[0] → entity_index=0。
  EXPECT_EQ(issue->location.kind, oneq::foundation::ValidationLocationKind::kSceneEntity);
  EXPECT_EQ(issue->location.entity_index, 0U);
}

TEST(SbirsPipelineTest, TargetOutsideWfovWritesInfoExclusionDiagnostic) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  // 扫描中心约 0°，WFOV 20°×20°：az=100° 目标在视场外（|100-0|=100° > 10°）。
  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, 100.0, 0.0))
          .Build();

  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  EXPECT_TRUE(result.detections.empty());
  const auto* issue = FindIssue(result, "sbirs.target_out_of_wfov");
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, sbirs_sensor::session::SbirsIssueSeverity::kInfo);
  EXPECT_NE(issue->message.find("target_id=1"), std::string::npos);
  // 门内归因（规则 13b）：az=100° 相对扫描中心仅方位越界 → kAzOutside。
  EXPECT_EQ(issue->cause, sbirs_sensor::session::SbirsIssueCause::kAzOutside);
  EXPECT_NE(issue->message.find("az_delta_deg="), std::string::npos);
  // 实体机器可读关联（规则 14e）：单目标 scene[0] → entity_index=0。
  EXPECT_EQ(issue->location.kind, oneq::foundation::ValidationLocationKind::kSceneEntity);
  EXPECT_EQ(issue->location.entity_index, 0U);
}

TEST(SbirsPipelineTest, TargetOutsideWfovElevationWritesElOutsideCause) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  // WFOV 20°×20°：az=0°（视场内）、el=15°（|15-0|=15° > 10° 半视场）→ 俯仰越界。
  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, 0.0, 15.0))
          .Build();

  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  EXPECT_TRUE(result.detections.empty());
  const auto* issue = FindIssue(result, "sbirs.target_out_of_wfov");
  ASSERT_NE(issue, nullptr);
  // 门内归因（规则 13b）：仅俯仰越界 → kElOutside。
  EXPECT_EQ(issue->cause, sbirs_sensor::session::SbirsIssueCause::kElOutside);
  // 实体机器可读关联（规则 14e）：单目标 scene[0] → entity_index=0。
  EXPECT_EQ(issue->location.kind, oneq::foundation::ValidationLocationKind::kSceneEntity);
  EXPECT_EQ(issue->location.entity_index, 0U);
}

TEST(SbirsPipelineTest, TargetOutsideWfovBothAxesWritesBothAxesOutsideCause) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  // WFOV 20°×20°：az=15° 与 el=15° 均越出半视场 10° → 双轴越界。
  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, 15.0, 15.0))
          .Build();

  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  EXPECT_TRUE(result.detections.empty());
  const auto* issue = FindIssue(result, "sbirs.target_out_of_wfov");
  ASSERT_NE(issue, nullptr);
  // 门内归因（规则 13b）：方位与俯仰均越界 → kBothAxesOutside。
  EXPECT_EQ(issue->cause, sbirs_sensor::session::SbirsIssueCause::kBothAxesOutside);
  // 实体机器可读关联（规则 14e）：单目标 scene[0] → entity_index=0。
  EXPECT_EQ(issue->location.kind, oneq::foundation::ValidationLocationKind::kSceneEntity);
  EXPECT_EQ(issue->location.entity_index, 0U);
}

TEST(SbirsPipelineTest, TargetBelowWideSnrWritesInfoExclusionDiagnostic) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  // WFOV SNR 门限抬到极大：视场内热目标必低于门限 → SNR 门排除。
  config.policy.detection.wide_min_snr_linear = 1.0e30f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(1U, 0.0))  // az=0°/el=0°：视场内
          .Build();

  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  EXPECT_TRUE(result.detections.empty());
  const auto* issue = FindIssue(result, "sbirs.target_snr_below_threshold");
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, sbirs_sensor::session::SbirsIssueSeverity::kInfo);
  EXPECT_NE(issue->message.find("target_id=1"), std::string::npos);
  // 门内归因（规则 13b）：门限抬到 1e30 后达标所需签名缺口主导 → kSignatureLimited。
  EXPECT_EQ(issue->cause, sbirs_sensor::session::SbirsIssueCause::kSignatureLimited);
  EXPECT_NE(issue->message.find("range_m="), std::string::npos);
  // 实体机器可读关联（规则 14e）：单目标 scene[0] → entity_index=0。
  EXPECT_EQ(issue->location.kind, oneq::foundation::ValidationLocationKind::kSceneEntity);
  EXPECT_EQ(issue->location.entity_index, 0U);
}

// ===== 合同指标 2：卫星速度进入相对视线角速度（动态滞后/词外推/R 阵） =====

// 卫星切向速度 → 相对视线角速度 ω = |v_perp|/range → WFOV 带误差量测方位额外滞后
// ω/(2π·f_det)。两次运行仅卫星速度不同，方位差应精确等于该滞后量。
TEST(SbirsPipelineTest, SatelliteVelocityAddsRelativeLagToWfovMeasurement) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.work_mode = sbirs_sensor::config::SbirsWorkMode::kWideSearch;

  const auto run_wfov_azimuth_deg = [&config](double satellite_vy) {
    sbirs_sensor::pipeline::SbirsPipeline pipeline(
        sbirs_sensor::runtime::MapSessionToInternal(config));
    const sbirs_sensor::session::SbirsCycleInput input =
        sbirs_sensor::session::SbirsCycleInputBuilder()
            .WithCycleIndex(1U)
            .WithDeltaTimeSec(1.0f)
            .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
            .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
            .WithSatelliteVelocity(Vector(0.0, satellite_vy, 0.0)).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
            .AddTarget(HotTarget(7U, 0.0))
            .Build();
    const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
    EXPECT_EQ(result.detections.size(), 1U);
    return result.detections.front().record.azimuth_rad * 180.0 / 3.14159265358979323846;
  };

  const double az_static_deg = run_wfov_azimuth_deg(0.0);
  const double az_moving_deg = run_wfov_azimuth_deg(7500.0);
  // ω = 7500/1e6 rad/s；滞后 = ω/(2π·f_det)，f_det 默认 100 Hz。
  const double expected_lag_deg =
      (7500.0 / 1000000.0) * (180.0 / 3.14159265358979323846) / (2.0 * 3.14159265358979323846 * 100.0);
  EXPECT_NEAR(az_moving_deg - az_static_deg, expected_lag_deg, 1.0e-7);
}

// ===== 合同指标 4：归属层携带当前时刻最大探测距离 d_max(t) =====

TEST(SbirsPipelineTest, AttributionCarriesCurrentInstantMaxDetectionRange) {
  const sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(Vector(0.0, 0.0, 0.0)).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(7U, 0.0))
          .Build();
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  ASSERT_FALSE(result.detections.empty());

  // 期望值由模块各单测覆盖的组成函数独立拼出（透过率/噪声/反解）。
  // 星目双端均在壳外（7000/8000 km > 壳顶 6471 km，近地点=卫星 629 km 高）：
  // 壳段气团 X=0 → τ_geo=1（纯空间路径不扣大气，2026-09-02 冻结契约）。
  const float transmittance_geo = sbirs_sensor::environment::ResolveGeometricTransmittance(
      config.environment, Vector(7000000.0, 0.0, 0.0), Vector(8000000.0, 0.0, 0.0));
  const double effective_noise_w = sbirs_sensor::foundation::ResolveEffectiveNoiseW(
      config.hardware, sbirs_sensor::foundation::ComputeBackgroundNoiseStatistics(config.hardware));
  const double radiometric_d_max = sbirs_sensor::foundation::ComputeMaxDetectionRangeM(
      1.0e8, config.hardware.optical_aperture_m, config.hardware.optical_transmission,
      transmittance_geo, config.hardware.detector_quantum_efficiency,
      config.hardware.integration_time_sec, effective_noise_w,
      config.policy.detection.wide_min_snr_linear);
  EXPECT_GT(radiometric_d_max, 0.0);
  for (const auto& detection : result.detections) {
    const double effective_d_max = detection.attribution.max_detection_range_m;
    EXPECT_GT(effective_d_max, 0.0);
    EXPECT_LE(effective_d_max, static_cast<double>(config.mission.max_range_m));
    EXPECT_LE(effective_d_max, radiometric_d_max);
    // The current one-megametre target must remain inside the reported capability.
    EXPECT_GE(effective_d_max, 1000000.0);
  }
}

// 气象恶化（雾）→ τ_eff 下降 → 同一目标 d_max 变小（d_max 随周期环境快照变化）。
// 目标置于大气壳内（79 km 高，r=6450 km < 壳顶 6471 km）：视线穿壳，τ_geo=τ_eff^X
// 随天气变化；扫描窗挪到方位 175° 盖住天底方向的视线（X=0.21，穿壳弦 21 km）。
TEST(SbirsPipelineTest, MaxDetectionRangeNeverIncreasesUnderFog) {
  const auto run_first_attribution_d_max = [](sbirs_sensor::config::SbirsWeatherType weather) {
    sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
    config.environment.weather_type = weather;
    config.mission.scan_start_az_deg = 175.0f;  // 视线方位 180°（天底）需入扫描窗
    sbirs_sensor::pipeline::SbirsPipeline pipeline(
        sbirs_sensor::runtime::MapSessionToInternal(config));
    sbirs_sensor::session::SbirsSceneTarget shell_target = HotTarget(7U, 0.0);
    shell_target.position_ecef_m = Vector(6450000.0, 0.0, 0.0);
    const sbirs_sensor::session::SbirsCycleInput input =
        sbirs_sensor::session::SbirsCycleInputBuilder()
            .WithCycleIndex(1U)
            .WithDeltaTimeSec(1.0f)
            .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
            .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
            .WithSatelliteVelocity(Vector(0.0, 0.0, 0.0))
            .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
            .AddTarget(shell_target)
            .Build();
    const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
    EXPECT_FALSE(result.detections.empty());
    return result.detections.front().attribution.max_detection_range_m;
  };
  const double d_max_clear =
      run_first_attribution_d_max(sbirs_sensor::config::SbirsWeatherType::kClear);
  const double d_max_fog =
      run_first_attribution_d_max(sbirs_sensor::config::SbirsWeatherType::kFog);
  EXPECT_GT(d_max_clear, 0.0);
  EXPECT_LE(d_max_fog, d_max_clear);
}

// SNR 门失败目标的 issue 消息携带 d_max 数值（人读诊断，机器可读量仅在归属层）。
TEST(SbirsPipelineTest, SnrExclusionIssueMessageCarriesMaxDetectionRange) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsSceneTarget faint_target = HotTarget(7U, 0.0);
  faint_target.radiant_intensity_w_per_sr = 1.0e-12;  // SNR 远低于 wide_min
  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(Vector(0.0, 0.0, 0.0)).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(faint_target)
          .Build();
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  const sbirs_sensor::session::SbirsIssue* issue = FindIssue(
      result, sbirs_sensor::session::codes::kTargetSnrBelowThreshold);
  ASSERT_NE(issue, nullptr);
  EXPECT_NE(issue->message.find("d_max_m="), std::string::npos);
  EXPECT_TRUE(result.detections.empty());
}

TEST(SbirsPipelineTest, BodyYawSteersWfovFootprintInertialKeepsEciReference) {
  // 阶段 2 稳定方式：体稳定下卫星 yaw=30° 把传感器系视场足迹旋转 30°，固定目标
  //（ECI 方位 0°）落出扫描窗口；惯性稳定下扫描参数保持 ECI 参考方向，目标仍被探测，
  // 且输出 azimuth_rad 保持 ECI 参考（不随姿态/安装变化）。
  const sbirs_sensor::session::SbirsEulerAnglesDeg yaw30;
  sbirs_sensor::session::SbirsEulerAnglesDeg attitude = yaw30;
  attitude.yaw_deg = 30.0;

  // 体稳定（默认）：目标 az=0 相对扫描中心（0°）被 yaw30 平移 30° 出 10° 半宽窗口。
  {
    const sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
    sbirs_sensor::pipeline::SbirsPipeline pipeline(
        sbirs_sensor::runtime::MapSessionToInternal(config));
    sbirs_sensor::session::SbirsCycleInput input =
        sbirs_sensor::session::SbirsCycleInputBuilder()
            .WithCycleIndex(1U)
            .WithDeltaTimeSec(1.0f)
            .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
            .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
            .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
            .WithSatelliteAttitude(attitude)
            .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
            .Build();
    const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
    EXPECT_TRUE(result.detections.empty());
    EXPECT_NE(FindIssue(result, sbirs_sensor::session::codes::kTargetOutOfWfov), nullptr);
  }

  // 惯性稳定：yaw30 下扫描参数反解到传感器系（目标与中心同步平移）→ 仍探测，
  // 输出 az 保持 ECI 参考（≈0 rad）。
  {
    sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
    config.orientation.stabilization_mode =
        sbirs_sensor::config::SbirsStabilizationMode::kInertialStabilized;
    sbirs_sensor::pipeline::SbirsPipeline pipeline(
        sbirs_sensor::runtime::MapSessionToInternal(config));
    sbirs_sensor::session::SbirsCycleInput input =
        sbirs_sensor::session::SbirsCycleInputBuilder()
            .WithCycleIndex(1U)
            .WithDeltaTimeSec(1.0f)
            .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
            .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
            .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
            .WithSatelliteAttitude(attitude)
            .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
            .Build();
    const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
    EXPECT_FALSE(result.detections.empty());
    EXPECT_NEAR(result.detections.front().record.azimuth_rad, 0.0f, 1.0e-4f);
    EXPECT_NEAR(result.detections.front().record.elevation_rad, 0.0f, 1.0e-4f);
  }
}

TEST(SbirsPipelineTest, MountAnglesSteerWfovFootprint) {
  // 安装角（Body->Sensor）独立于姿态生效：零姿态 + mount yaw=30° 等效于 yaw 姿态
  // 的体稳定足迹平移，固定目标落出扫描窗口。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.orientation.mount_angles_deg.yaw_deg = 30.0;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
          .Build();
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  EXPECT_TRUE(result.detections.empty());
  EXPECT_NE(FindIssue(result, sbirs_sensor::session::codes::kTargetOutOfWfov), nullptr);
}

TEST(SbirsPipelineTest, NarrowSensorAzimuthLimitClampsTrackCommandCausingGeometryGateFailure) {
  // 限位钳制 NFOV 命令：真值辅助跟踪模式下目标跨周期跳变到限位外，命令被钳制到
  // 限位边缘，实际指向偏离目标 → 几何门失败（对照无限制配置几何门通过）。
  sbirs_sensor::config::SbirsSessionConfig limited = PipelineConfig();
  limited.orientation.sensor_scan_limits_deg.az_min_deg = -2.0f;
  limited.orientation.sensor_scan_limits_deg.az_max_deg = 2.0f;
  limited.mission.scan_start_az_deg = -1.0f;
  limited.mission.scan_span_deg = 2.0f;
  limited.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kStrictTruthAssisted;

  sbirs_sensor::pipeline::SbirsPipeline limited_pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(limited));
  sbirs_sensor::session::SbirsCycleInput first_cycle =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
          .Build();
  const sbirs_sensor::pipeline::SbirsPipelineResult capture =
      limited_pipeline.RunCycle(first_cycle);
  ASSERT_FALSE(capture.detections.empty());
  EXPECT_TRUE(capture.detections.front().record.detected);

  sbirs_sensor::session::SbirsCycleInput second_cycle = first_cycle;
  second_cycle.cycle_index = 2U;
  second_cycle.scene[0] = HotTargetAtAngles(1U, 10.0, 0.0);
  const sbirs_sensor::pipeline::SbirsPipelineResult limited_result =
      limited_pipeline.RunCycle(second_cycle);
  ASSERT_EQ(limited_result.detections.size(), 1U);
  EXPECT_FALSE(limited_result.detections.front().attribution.nfov_geometry_gate_passed);
  EXPECT_EQ(limited_result.detections.front().attribution.nfov_tracking_gate_failure_count, 1U);

  // 对照：无限位（全开）下命令不钳制，几何门通过（与 limited 同 tracking 模式，
  // 仅差限位；真值辅助模式命令=真值，隔离限位效应）。
  sbirs_sensor::config::SbirsSessionConfig unlimited = PipelineConfig();
  unlimited.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kStrictTruthAssisted;
  sbirs_sensor::pipeline::SbirsPipeline unlimited_pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(unlimited));
  ASSERT_TRUE(unlimited_pipeline.RunCycle(first_cycle).detections.size() >= 1U);
  const sbirs_sensor::pipeline::SbirsPipelineResult unlimited_result =
      unlimited_pipeline.RunCycle(second_cycle);
  ASSERT_EQ(unlimited_result.detections.size(), 1U);
  EXPECT_TRUE(unlimited_result.detections.front().attribution.nfov_geometry_gate_passed);
}

TEST(SbirsPipelineTest, BiasMisalignmentSteersWfovFootprint) {
  // 阶段 3：静态失准偏置独立于周期姿态生效（与 mount 同语义、方向相反）：
  // bias yaw=30° 把传感器系视场足迹相对 ECI 平移 -30°，固定目标（ECI az=0）在
  // 传感器系落入 +30°，出 10° 半宽扫描窗口。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.orientation.misalignment.bias_deg.yaw_deg = 30.0;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
          .Build();
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  EXPECT_TRUE(result.detections.empty());
  EXPECT_NE(FindIssue(result, sbirs_sensor::session::codes::kTargetOutOfWfov), nullptr);
}

TEST(SbirsPipelineTest, RandomMisalignmentIsDrawnOncePerRun) {
  // 阶段 3 常值契约：随机微扰每次运行抽取一次——同种子确定性（两 pipeline 逐位一致）、
  // 种子驱动（不同种子输出不同）、运行内不重抽（dt=0 同输入再跑一周期逐位一致）。
  // 扫描中心 ECI 方位 = 失准驱动的合成光轴（非 identity 链），作为确定性可观测信号。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.orientation.misalignment.bias_deg.yaw_deg = 5.0;
  config.orientation.misalignment.random_sigma_deg = 2.0f;
  config.orientation.misalignment.random_seed = 7U;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
          .Build();

  sbirs_sensor::pipeline::SbirsPipeline pipeline_a(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::pipeline::SbirsPipeline pipeline_b(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const sbirs_sensor::pipeline::SbirsPipelineResult result_a = pipeline_a.RunCycle(input);
  const sbirs_sensor::pipeline::SbirsPipelineResult result_b = pipeline_b.RunCycle(input);
  EXPECT_FALSE(result_a.detections.empty());
  EXPECT_FLOAT_EQ(result_a.scan_azimuth_rad, result_b.scan_azimuth_rad);

  // 与 bias-only（sigma=0）配置输出不同 → 随机偏置确实生效（连续分布相等概率 0）。
  sbirs_sensor::config::SbirsSessionConfig bias_only = PipelineConfig();
  bias_only.orientation.misalignment.bias_deg.yaw_deg = 5.0;
  sbirs_sensor::pipeline::SbirsPipeline bias_only_pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(bias_only));
  const sbirs_sensor::pipeline::SbirsPipelineResult bias_only_result =
      bias_only_pipeline.RunCycle(input);
  EXPECT_NE(result_a.scan_azimuth_rad, bias_only_result.scan_azimuth_rad);

  // 运行内不重抽：dt=0 同输入再跑一周期（扫描相位不推进），输出逐位一致；若实现
  // 为每周期重抽则失准改变导致扫描中心方位变化。
  sbirs_sensor::session::SbirsCycleInput input_zero_dt =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(0.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
          .Build();
  const sbirs_sensor::pipeline::SbirsPipelineResult result_a_repeat =
      pipeline_a.RunCycle(input_zero_dt);
  EXPECT_FLOAT_EQ(result_a.scan_azimuth_rad, result_a_repeat.scan_azimuth_rad);
}

TEST(SbirsPipelineTest, MisalignmentSnapshotRoundTripPreservesDeterminism) {
  // 快照契约：运行期失准进 Capture/Restore 往返——Restore 后回读快照字段确认回填，
  // 修改快照失准值改变后续行为（ECI 输出参考 scan_azimuth_rad 由链驱动，非 identity
  // 链下随失准变化），未修改则确定性 continuation 逐位一致。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.orientation.misalignment.bias_deg.yaw_deg = 5.0;
  config.orientation.misalignment.random_sigma_deg = 2.0f;
  config.orientation.misalignment.random_seed = 7U;

  const sbirs_sensor::session::SbirsCycleInput first =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
          .Build();
  const sbirs_sensor::session::SbirsCycleInput second =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(2U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
          .Build();

  sbirs_sensor::pipeline::SbirsPipeline untouched(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::pipeline::SbirsPipeline tampered(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  untouched.RunCycle(first);
  tampered.RunCycle(first);

  sbirs_sensor::pipeline::SbirsPipelineSnapshot untouched_snapshot =
      untouched.CaptureRuntimeState();
  EXPECT_TRUE(std::isfinite(untouched_snapshot.misalignment_yaw_deg));
  EXPECT_TRUE(std::isfinite(untouched_snapshot.misalignment_pitch_deg));
  EXPECT_TRUE(std::isfinite(untouched_snapshot.misalignment_roll_deg));
  ASSERT_TRUE(untouched.RestoreRuntimeState(untouched_snapshot));

  sbirs_sensor::pipeline::SbirsPipelineSnapshot tampered_snapshot =
      tampered.CaptureRuntimeState();
  tampered_snapshot.misalignment_pitch_deg += 20.0f;
  ASSERT_TRUE(tampered.RestoreRuntimeState(tampered_snapshot));
  // 回填验证：restore 后回读快照字段等于改后的值（失准确实进成员并随快照往返）。
  const sbirs_sensor::pipeline::SbirsPipelineSnapshot tampered_verified =
      tampered.CaptureRuntimeState();
  EXPECT_FLOAT_EQ(tampered_verified.misalignment_pitch_deg,
                  tampered_snapshot.misalignment_pitch_deg);
  EXPECT_FLOAT_EQ(tampered_verified.misalignment_yaw_deg,
                  untouched_snapshot.misalignment_yaw_deg);
  EXPECT_FLOAT_EQ(tampered_verified.misalignment_roll_deg,
                  untouched_snapshot.misalignment_roll_deg);

  const sbirs_sensor::pipeline::SbirsPipelineResult untouched_result =
      untouched.RunCycle(second);
  const sbirs_sensor::pipeline::SbirsPipelineResult tampered_result = tampered.RunCycle(second);
  // 行为差异：非 identity 链下 ECI 扫描方位由失准驱动，快照回填生效（pitch 不同 → 方位不同）。
  EXPECT_NE(untouched_result.scan_azimuth_rad, tampered_result.scan_azimuth_rad);
}

TEST(SbirsPipelineTest, RasterDefaultSpanZeroKeepsLegacySingleRow) {
  // 阶段 4 不变量：span_el=0（默认）→ 行数=1、行索引恒 0、scan_elevation_rad 恒等于
  // scan_center_el_deg；方位输出与既有 CircularScan 金值逐位一致。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.scan_start_az_deg = 170.0f;
  config.mission.scan_span_deg = 40.0f;
  config.mission.scan_rate_deg_per_sec = 15.0f;
  config.mission.scan_center_el_deg = 8.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(TwoTargetInput(1U));
  EXPECT_FLOAT_EQ(result.scan_azimuth_rad, 3.2288592f);  // 185°（既有金值）
  EXPECT_FLOAT_EQ(result.scan_elevation_rad,
                  kPi * 8.0f / 180.0f);  // 8°
  EXPECT_EQ(pipeline.CaptureRuntimeState().scan_row_index, 0);
  EXPECT_FLOAT_EQ(pipeline.CaptureRuntimeState().scan_phase_deg, 15.0f);
}

TEST(SbirsPipelineTest, TwoDimensionalRasterAdvancesRowsAndWraps) {
  // 2-D 栅格：span_el=20、step=10 → 3 行（el 中心 0/10/20）。锯齿单向：每行从 az 起点
  // 同向扫描，行内相位跨过 span 时行步进、az 相位归零；行末回绕。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.scan_start_az_deg = 0.0f;
  config.mission.scan_span_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 5.0f;
  config.mission.scan_el_start_deg = 0.0f;
  config.mission.scan_el_span_deg = 20.0f;
  config.mission.scan_el_step_deg = 10.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const auto run_cycle = [&](std::uint32_t index, float dt_sec) {
    sbirs_sensor::session::SbirsCycleInput input =
        sbirs_sensor::session::SbirsCycleInputBuilder()
            .WithCycleIndex(index)
            .WithDeltaTimeSec(dt_sec)
            .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
            .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
            .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
            .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
            .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
            .Build();
    return pipeline.RunCycle(input);
  };
  // 周期 1：dt=1 → phase 5°，row 0，行中心 el 0°。
  {
    const sbirs_sensor::pipeline::SbirsPipelineResult result = run_cycle(1U, 1.0f);
    const auto snapshot = pipeline.CaptureRuntimeState();
    EXPECT_EQ(snapshot.scan_row_index, 0);
    EXPECT_FLOAT_EQ(snapshot.scan_phase_deg, 5.0f);
    EXPECT_FLOAT_EQ(result.scan_elevation_rad, 0.0f);
  }
  // 周期 2：dt=2 → phase 15° 跨过 span → 归零为 5°，row 1，行中心 el 10°。
  {
    const sbirs_sensor::pipeline::SbirsPipelineResult result = run_cycle(2U, 2.0f);
    const auto snapshot = pipeline.CaptureRuntimeState();
    EXPECT_EQ(snapshot.scan_row_index, 1);
    EXPECT_FLOAT_EQ(snapshot.scan_phase_deg, 5.0f);
    EXPECT_FLOAT_EQ(result.scan_elevation_rad,
                    kPi * 10.0f / 180.0f);
  }
  // 周期 3：dt=2 → 再跨一行 → row 2，行中心 el 20°。
  {
    const sbirs_sensor::pipeline::SbirsPipelineResult result = run_cycle(3U, 2.0f);
    const auto snapshot = pipeline.CaptureRuntimeState();
    EXPECT_EQ(snapshot.scan_row_index, 2);
    EXPECT_FLOAT_EQ(result.scan_elevation_rad,
                    kPi * 20.0f / 180.0f);
  }
  // 周期 4：dt=2 → 跨两行：row2→row0（回绕），行中心 el 0°。
  {
    const sbirs_sensor::pipeline::SbirsPipelineResult result = run_cycle(4U, 2.0f);
    const auto snapshot = pipeline.CaptureRuntimeState();
    EXPECT_EQ(snapshot.scan_row_index, 0);
    EXPECT_FLOAT_EQ(result.scan_elevation_rad, 0.0f);
  }
}

TEST(SbirsPipelineTest, ElevationRasterGatesTargetByRow) {
  // 逐行矩形 FOV 门：kWideSearch 模式（只输出 WFOV 观测、不分配 NFOV）隔离状态累积。
  // 目标 el 5°，FOV_el=20（半门 10）：row 0（中心 0°）与 row 1（中心 10°）在门内探测，
  // row 2（中心 20°）目标落出 WFOV 俯仰门。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.work_mode = sbirs_sensor::config::SbirsWorkMode::kWideSearch;
  config.mission.scan_start_az_deg = 0.0f;
  config.mission.scan_span_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.mission.wide_field_fov_az_deg = 30.0f;
  config.mission.wide_field_fov_el_deg = 20.0f;
  config.mission.scan_el_start_deg = 0.0f;
  config.mission.scan_el_span_deg = 20.0f;
  config.mission.scan_el_step_deg = 10.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const auto run_cycle = [&](std::uint32_t index, float dt_sec) {
    sbirs_sensor::session::SbirsCycleInput input =
        sbirs_sensor::session::SbirsCycleInputBuilder()
            .WithCycleIndex(index)
            .WithDeltaTimeSec(dt_sec)
            .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
            .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
            .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
            .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
            .AddTarget(HotTargetAtAngles(1U, 0.0, 5.0))
            .Build();
    return pipeline.RunCycle(input);
  };
  EXPECT_EQ(run_cycle(1U, 1.0f).detections.size(), 1U);   // row 0（中心 0°），目标 5° 在门内
  EXPECT_EQ(run_cycle(2U, 11.0f).detections.size(), 1U);  // row 1（中心 10°），目标 5° 仍在内
  // row 2（中心 20°）：目标 5° 距行中心 15° > 半门 10° → 门失败。
  const sbirs_sensor::pipeline::SbirsPipelineResult third = run_cycle(3U, 11.0f);
  EXPECT_TRUE(third.detections.empty());
  EXPECT_NE(FindIssue(third, sbirs_sensor::session::codes::kTargetOutOfWfov), nullptr);
  EXPECT_EQ(pipeline.CaptureRuntimeState().scan_row_index, 2);
}

TEST(SbirsPipelineTest, CrossRowTargetRevisitPeriod) {
  // 跨行重访：目标 el 0°（位于行 0 中心），行 0 探测后行推进到行 1（中心 20°，
  // FOV_el=16 半门 8 → 目标落出栅格），一个完整行内扫描周期（span/rate = 10s）
  // 后回到行 0 重访。kWideSearch 隔离 NFOV 状态累积。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.work_mode = sbirs_sensor::config::SbirsWorkMode::kWideSearch;
  config.mission.scan_start_az_deg = 0.0f;
  config.mission.scan_span_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.mission.wide_field_fov_az_deg = 30.0f;
  config.mission.wide_field_fov_el_deg = 16.0f;
  config.mission.scan_el_start_deg = 0.0f;
  config.mission.scan_el_span_deg = 20.0f;
  config.mission.scan_el_step_deg = 20.0f;  // 2 行：中心 0°/20°
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const auto run_cycle = [&](std::uint32_t index, float dt_sec) {
    sbirs_sensor::session::SbirsCycleInput input =
        sbirs_sensor::session::SbirsCycleInputBuilder()
            .WithCycleIndex(index)
            .WithDeltaTimeSec(dt_sec)
            .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
            .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
            .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
            .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
            .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
            .Build();
    return pipeline.RunCycle(input);
  };
  EXPECT_EQ(run_cycle(1U, 1.0f).detections.size(), 1U);   // row 0：探测
  EXPECT_EQ(run_cycle(2U, 11.0f).detections.size(), 0U);  // row 1（el 20°）：出栅格
  // 每行行内周期 10s（span/rate），两行后回到 row 0 → 重访。
  EXPECT_EQ(run_cycle(3U, 11.0f).detections.size(), 1U);  // row 0：重访
  EXPECT_EQ(pipeline.CaptureRuntimeState().scan_row_index, 0);
}

TEST(SbirsPipelineTest, ElevationRasterSnapshotRestoreRoundTrip) {
  // 快照契约：row+phase 往返逐位一致；Restore 拒绝越界 row（快照损坏路径）。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.scan_el_start_deg = 0.0f;
  config.mission.scan_el_span_deg = 20.0f;
  config.mission.scan_el_step_deg = 10.0f;
  const sbirs_sensor::session::SbirsCycleInput input = TwoTargetInput(1U);
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  pipeline.RunCycle(input);
  const sbirs_sensor::pipeline::SbirsPipelineSnapshot snapshot = pipeline.CaptureRuntimeState();
  EXPECT_EQ(snapshot.scan_row_index, 0);
  ASSERT_TRUE(pipeline.RestoreRuntimeState(snapshot));
  EXPECT_EQ(pipeline.CaptureRuntimeState().scan_row_index, 0);
  EXPECT_FLOAT_EQ(pipeline.CaptureRuntimeState().scan_phase_deg, snapshot.scan_phase_deg);

  // 越界 row（3 ≥ row_count=3）→ 拒绝；负 row → 拒绝。
  sbirs_sensor::pipeline::SbirsPipelineSnapshot out_of_range = snapshot;
  out_of_range.scan_row_index = 3;
  EXPECT_FALSE(pipeline.RestoreRuntimeState(out_of_range));
  out_of_range.scan_row_index = -1;
  EXPECT_FALSE(pipeline.RestoreRuntimeState(out_of_range));
}

TEST(SbirsPipelineTest, ElevationRasterSectorPatchReanchorsRow) {
  // 扇区 patch：改 el 栅格且 impact.scan_sector_changed 置位 → 旧行中心 el 映射到
  // 新栅格最近行；行内 az 相位按旧绝对方位保持（同 apply 语义）。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.scan_start_az_deg = 0.0f;
  config.mission.scan_span_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.mission.scan_el_start_deg = 0.0f;
  config.mission.scan_el_span_deg = 20.0f;
  config.mission.scan_el_step_deg = 10.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  pipeline.RunCycle(TwoTargetInput(1U));  // row 0（el 0°），phase 1°
  EXPECT_EQ(pipeline.CaptureRuntimeState().scan_row_index, 0);
  const float previous_phase = pipeline.CaptureRuntimeState().scan_phase_deg;

  // 新栅格 [-10, 10] step 10（行中心 -10/0/10）：旧行中心 el 0° → 最近行索引 1（el 0°）。
  sbirs_sensor::config::SbirsSessionConfig next = config;
  next.mission.scan_el_start_deg = -10.0f;
  next.mission.scan_el_span_deg = 20.0f;
  next.mission.scan_el_step_deg = 10.0f;
  sbirs_sensor::runtime::SbirsRuntimeConfigImpact impact;
  impact.scan_sector_changed = true;
  pipeline.ApplyConfig(sbirs_sensor::runtime::MapSessionToInternal(next), impact);
  EXPECT_EQ(pipeline.CaptureRuntimeState().scan_row_index, 1);
  EXPECT_FLOAT_EQ(pipeline.CaptureRuntimeState().scan_phase_deg, previous_phase);

  // 新栅格不含旧 el：[-40, -20] step 10（行中心 -40/-30/-20），旧 el 0° 不在内 → 行归零。
  sbirs_sensor::config::SbirsSessionConfig outside = next;
  outside.mission.scan_el_start_deg = -40.0f;
  outside.mission.scan_el_span_deg = 20.0f;
  outside.mission.scan_el_step_deg = 10.0f;
  pipeline.ApplyConfig(sbirs_sensor::runtime::MapSessionToInternal(outside), impact);
  EXPECT_EQ(pipeline.CaptureRuntimeState().scan_row_index, 0);
}

// --- nadir 方位基准（2026-08-31）---
// 几何约定：卫星在 (7e6, 0, 0)，GMST≈0（ECI≡ECEF）→ 星下点方位 = 180°；
// 星下点方向目标取 (6.5e6, 0, 0)（地表上方约 129 km，视距 5e5 m，无遮挡）。
sbirs_sensor::config::SbirsSessionConfig NadirStareConfig(float start_offset_deg) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.scan_azimuth_reference =
      sbirs_sensor::config::SbirsScanAzimuthReference::kNadirRelative;
  config.mission.scan_start_az_deg = start_offset_deg;
  config.mission.scan_span_deg = 20.0f;
  config.mission.scan_rate_deg_per_sec = 0.0f;  // 凝视：相位恒 0
  config.mission.wide_field_fov_az_deg = 20.0f;
  config.mission.wide_field_fov_el_deg = 20.0f;
  return config;
}

sbirs_sensor::session::SbirsCycleInput NadirInput() {
  sbirs_sensor::session::SbirsSceneTarget target = HotTarget(11U, 0.0);
  target.position_ecef_m = Vector(6500000.0, 0.0, 0.0);
  return sbirs_sensor::session::SbirsCycleInputBuilder()
      .WithCycleIndex(1U)
      .WithDeltaTimeSec(1.0f)
      .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
      .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
      .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
      .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
      .AddTarget(target)
      .Build();
}

TEST(SbirsPipelineTest, NadirStareOffsetZeroDetectsSubSatelliteTarget) {
  // 偏移 0 + 速率 0 = 视场钉在星下点方位（180°），正下方目标应被检出。
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(NadirStareConfig(0.0f)));
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(NadirInput());
  ASSERT_FALSE(result.detections.empty());
  EXPECT_TRUE(result.detections.front().record.detected);
}

TEST(SbirsPipelineTest, NadirStareOffsetNinetyDegreesMissesTarget) {
  // 偏移 90° → 有效指向 270°，目标在 180°，Δaz=90° 超半视场 10° → 门控排除。
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(NadirStareConfig(90.0f)));
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(NadirInput());
  EXPECT_TRUE(result.detections.empty());
}

TEST(SbirsPipelineTest, NadirOffsetZeroEqualsAbsoluteStartAtNadirAzimuth) {
  // 等价性：nadir 偏移 0 ≡ 绝对起点 180°（该几何下的星下点方位），扫描方位一致。
  sbirs_sensor::config::SbirsSessionConfig absolute = NadirStareConfig(0.0f);
  absolute.mission.scan_azimuth_reference =
      sbirs_sensor::config::SbirsScanAzimuthReference::kEciAbsolute;
  absolute.mission.scan_start_az_deg = 180.0f;
  sbirs_sensor::pipeline::SbirsPipeline nadir_pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(NadirStareConfig(0.0f)));
  sbirs_sensor::pipeline::SbirsPipeline absolute_pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(absolute));
  const sbirs_sensor::pipeline::SbirsPipelineResult nadir_result =
      nadir_pipeline.RunCycle(NadirInput());
  const sbirs_sensor::pipeline::SbirsPipelineResult absolute_result =
      absolute_pipeline.RunCycle(NadirInput());
  EXPECT_NEAR(nadir_result.scan_azimuth_rad, absolute_result.scan_azimuth_rad, 1.0e-5);
  ASSERT_FALSE(nadir_result.detections.empty());
  ASSERT_FALSE(absolute_result.detections.empty());
}

// --- 往复式扫描 + 跨度收敛（2026-08-31，sbirs-scan-realism）---

TEST(SbirsPipelineTest, PingPongSweepReversesAtSpanBoundary) {
  // 往复式：rate=1°/s、span=10°、dt=6 → 行程 6/12/18/24(折 4)。方位序列应为
  // 6°→8°→2°→4°（去程到边后反向），无锯齿式瞬时跳回；相位 ∈ [0, span)。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.scan_start_az_deg = 0.0f;
  config.mission.scan_span_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const auto run_cycle = [&](std::uint32_t index, float dt_sec) {
    sbirs_sensor::session::SbirsCycleInput input =
        sbirs_sensor::session::SbirsCycleInputBuilder()
            .WithCycleIndex(index)
            .WithDeltaTimeSec(dt_sec)
            .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
            .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
            .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
            .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
            .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
            .Build();
    return pipeline.RunCycle(input);
  };
  const auto az_deg = [](const sbirs_sensor::pipeline::SbirsPipelineResult& r) {
    return r.scan_azimuth_rad * 180.0 / kPi;
  };
  // 周期 1：行程 6（去程）→ az 6。
  EXPECT_NEAR(az_deg(run_cycle(1U, 6.0f)), 6.0, 1.0e-4);
  // 周期 2：行程 12 过界反射 → az 8（回程），不是锯齿的 2。
  EXPECT_NEAR(az_deg(run_cycle(2U, 6.0f)), 8.0, 1.0e-4);
  // 周期 3：行程 18 → az 2（回程续走）。
  EXPECT_NEAR(az_deg(run_cycle(3U, 6.0f)), 2.0, 1.0e-4);
  // 周期 4：行程 24 折 4（再过界，回到去程）→ az 4。
  EXPECT_NEAR(az_deg(run_cycle(4U, 6.0f)), 4.0, 1.0e-4);
  const auto snapshot = pipeline.CaptureRuntimeState();
  EXPECT_GE(snapshot.scan_phase_deg, 0.0f);
  EXPECT_LT(snapshot.scan_phase_deg, config.mission.scan_span_deg);
}

TEST(SbirsPipelineTest, PingPongSnapshotRestoresLegDirection) {
  // 快照契约：腿方向随快照往返——回程态恢复后继续反向走，而非复位去程。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.scan_start_az_deg = 0.0f;
  config.mission.scan_span_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const auto run_cycle = [&](std::uint32_t index, float dt_sec) {
    sbirs_sensor::session::SbirsCycleInput input =
        sbirs_sensor::session::SbirsCycleInputBuilder()
            .WithCycleIndex(index)
            .WithDeltaTimeSec(dt_sec)
            .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
            .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
            .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
            .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
            .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
            .Build();
    return pipeline.RunCycle(input);
  };
  run_cycle(1U, 1.0f);  // 行程 1（去程）
  ASSERT_TRUE(pipeline.CaptureRuntimeState().scan_leg_forward);
  run_cycle(2U, 11.0f);  // 行程 12 过界 → 回程，相位 8
  const sbirs_sensor::pipeline::SbirsPipelineSnapshot snapshot =
      pipeline.CaptureRuntimeState();
  EXPECT_FALSE(snapshot.scan_leg_forward);
  sbirs_sensor::pipeline::SbirsPipeline restored(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  ASSERT_TRUE(restored.RestoreRuntimeState(snapshot));
  // 恢复后 dt=2：行程 12+2=14 → 相位 6（回程续走）；若腿错误复位去程则行程 8+2=10 → 相位 10。
  const sbirs_sensor::pipeline::SbirsPipelineResult next = restored.RunCycle(
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(3U)
          .WithDeltaTimeSec(2.0f)
          .WithUtcJulianDay(2451544.2230698913)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, 0.0, 0.0))
          .Build());
  EXPECT_NEAR(next.scan_azimuth_rad * 180.0 / kPi, 6.0, 1.0e-4);
}

TEST(SbirsPipelineTest, NadirSpanConvergesToEarthDisk) {
  // 收敛（按"配置扇区 ∩ 地球可见窗"裁剪）：GEO 位形（|r|=42164km，+x 轴，GMST≈0 →
  // 星下点方位 180°），nadir 偏移 0、span=360、rate=10、wfov=24、递增方向 → 扇区
  // [0°,360°] ∩ 可见窗 [−20.69°,+20.69°] = [0°,20.69°]，有效跨度 20.69。全程方位
  // 限制在星下点 +[0,20.69]，相位以有效跨度回绕（到边反向），配置 360 不整圈扫。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.work_mode = sbirs_sensor::config::SbirsWorkMode::kWideSearch;
  config.mission.scan_azimuth_reference =
      sbirs_sensor::config::SbirsScanAzimuthReference::kNadirRelative;
  config.mission.scan_start_az_deg = 0.0f;
  config.mission.scan_span_deg = 360.0f;
  config.mission.scan_rate_deg_per_sec = 10.0f;
  config.mission.wide_field_fov_az_deg = 24.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const auto run_cycle = [&](std::uint32_t index) {
    sbirs_sensor::session::SbirsCycleInput input =
        sbirs_sensor::session::SbirsCycleInputBuilder()
            .WithCycleIndex(index)
            .WithDeltaTimeSec(1.0f)
            .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
            .WithSatellitePosition(Vector(42164000.0, 0.0, 0.0))
            .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
            .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
            .Build();
    return pipeline.RunCycle(input);
  };
  const double rho_deg = std::asin(6371.0 / 42164.0) * 180.0 / kPi;   // ≈ 8.69°
  const double useful_half_deg = rho_deg + 12.0;                      // ≈ 20.69°
  double last_offset_deg = 0.0;
  bool saw_return_leg = false;
  for (std::uint32_t cycle = 1U; cycle <= 12U; ++cycle) {
    const sbirs_sensor::pipeline::SbirsPipelineResult result = run_cycle(cycle);
    const double az_deg = result.scan_azimuth_rad * 180.0 / kPi;
    double offset = az_deg - 180.0;
    if (offset > 180.0) offset -= 360.0;
    if (offset < -180.0) offset += 360.0;
    EXPECT_LE(offset, useful_half_deg + 0.01) << "cycle " << cycle;
    EXPECT_GE(offset, -useful_half_deg - 0.01) << "cycle " << cycle;
    if (offset < last_offset_deg - 0.5) saw_return_leg = true;  // 方位回头 = 回程
    last_offset_deg = offset;
  }
  EXPECT_TRUE(saw_return_leg);  // 12s×10°/s=120° 行程 ≫ 2×20.69 必见回程
  EXPECT_LT(pipeline.CaptureRuntimeState().scan_phase_deg,
            static_cast<float>(useful_half_deg) + 0.1f);
}

TEST(SbirsPipelineTest, WfovWideElevationCoverageExecutesCleanly) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.wide_field_fov_az_deg = 24.0f;
  config.mission.wide_field_fov_el_deg = 24.0f;
  config.mission.scan_rate_deg_per_sec = 6.0f;
  config.mission.scan_span_deg = 30.0f;

  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  const auto input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)
          .WithSatellitePosition(Vector(42164000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .Build();

  const auto result = pipeline.RunCycle(input);
  EXPECT_TRUE(result.issues.empty() || result.issues.size() <= 2U);
}

TEST(SbirsPipelineTest, ExternalCueGuidesOutOfWfovTargetIntoNfov) {
  // cross-cue（2026-09-01）：目标在本星宽场扫描带外 → 无 cue 时 out_of_wfov 排除；
  // 注入外部引导（来源星带误差测角+距离，修订 5 运行时注入）→ 外部候选进同池调度
  // → 窄场捕获 → 归属标记引导来源星 ID（修订 4 同池同规则，来源只记录）。
  const sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  // 本星 ECEF (7000000,0,0)，GMST≈0 → ECI≡ECEF；扫描带中心近方位 0°，目标在 77°。
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(1U, 4500000.0))
          .Build();

  // 1) 无 cue：目标在宽场外，走排除路径（不产生候选）。
  {
    sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
    EXPECT_NE(FindIssue(result, "sbirs.target_out_of_wfov"), nullptr);
    EXPECT_TRUE(result.detections.empty());
  }

  // 2) 注入外部引导：来源星 ECEF (6000000,1000000,0) 看到同一目标（真值 ECEF
  //    (8000000,4500000,0)），LOS 方位≈60.25°、俯仰 0°、距离≈4.03e6 m。
  sbirs_sensor::session::SbirsExternalCue cue;
  cue.target_id = 1U;
  cue.source_satellite_entity_id = 104U;
  cue.source_position_ecef_m = Vector(6000000.0, 1000000.0, 0.0);
  cue.azimuth_deg = 60.2551f;
  cue.elevation_deg = 0.0f;
  cue.range_m = 4031129.0;
  pipeline.SubmitExternalCue(cue);
  {
    sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
    // 外部候选进调度并窄场捕获：输出帧至少一条检测，归属带来源星 ID。
    EXPECT_FALSE(result.detections.empty());
    bool saw_cue_sourced = false;
    for (const sbirs_sensor::pipeline::SbirsPipelineDetection& detection : result.detections) {
      if (detection.attribution.target_id == 1U &&
          detection.attribution.cue_source_satellite_entity_id == 104) {
        saw_cue_sourced = true;
      }
    }
    EXPECT_TRUE(saw_cue_sourced);
  }
}

TEST(SbirsPipelineTest, ExternalCueInvalidIsDroppedWithIssue) {
  // 非法 cue（非正距离）：注入点消费时整条丢弃并计 kInfo issue，不影响其余目标。
  const sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(1U, 4500000.0))
          .Build();
  sbirs_sensor::session::SbirsExternalCue bad_cue;
  bad_cue.target_id = 1U;
  bad_cue.source_satellite_entity_id = 104U;
  bad_cue.source_position_ecef_m = Vector(6000000.0, 1000000.0, 0.0);
  bad_cue.azimuth_deg = 60.2551f;
  bad_cue.elevation_deg = 0.0f;
  bad_cue.range_m = -1.0;  // 非正距离
  pipeline.SubmitExternalCue(bad_cue);
  sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  EXPECT_NE(FindIssue(result, "sbirs.external_cue_invalid"), nullptr);
  EXPECT_TRUE(result.detections.empty());
}

TEST(SbirsPipelineTest, InitialScanPhaseOffsetsSweep) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.scan_start_az_deg = 0.0f;
  config.mission.scan_span_deg = 20.0f;
  config.mission.scan_rate_deg_per_sec = 2.0f;
  config.mission.initial_scan_phase_deg = 5.0f;

  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  const auto init_snapshot = pipeline.CaptureRuntimeState();
  EXPECT_FLOAT_EQ(init_snapshot.scan_phase_deg, 5.0f);

  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(3.0f)
          .WithUtcJulianDay(2451544.2230698913)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .Build();
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  const double az_deg = result.scan_azimuth_rad * 180.0 / kPi;
  EXPECT_NEAR(az_deg, 11.0, 1.0e-3);
  EXPECT_FLOAT_EQ(pipeline.CaptureRuntimeState().scan_phase_deg, 11.0f);
}

TEST(SbirsPipelineTest, WfovCrossingTimeCalculatesSubCycleMeasurementTimestamp) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.work_mode = sbirs_sensor::config::SbirsWorkMode::kWideSearch;
  config.mission.scan_start_az_deg = 0.0f;
  config.mission.scan_span_deg = 30.0f;
  config.mission.scan_rate_deg_per_sec = 2.0f;
  config.mission.wide_field_fov_az_deg = 15.0f;
  config.mission.wide_field_fov_el_deg = 15.0f;

  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(5.0f)
          .WithUtcJulianDay(2451544.2230698913)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTargetAtAngles(1U, 6.0, 0.0))
          .Build();

  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  ASSERT_FALSE(result.detections.empty());
  const double meas_time_s = result.detections[0].record.measurement_time_s;
  EXPECT_GE(meas_time_s, 5.0);
  EXPECT_LE(meas_time_s, 10.0);
  EXPECT_NEAR(meas_time_s, 8.0, 0.1);
}

}  // namespace
