/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsTrackingCoordinator.
 * Model: SBIRS infrared sensor.
 * Layer: pipeline.
 * File role: executable implementation.
 * Responsibility: wide-field search, pointing, acquisition, and tracking processing.
 * Upstream boundary: cycle scene, cue, sensor state, and resolved physical parameters.
 * Downstream boundary: detections, estimates, diagnostics, and lifecycle observations.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

#include "sbirs_sensor/pipeline/SbirsTrackingCoordinator.h"

#include <Eigen/Cholesky>
#include <limits>

#include "sbirs_sensor/foundation/SbirsGeometry.h"
#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

constexpr float kSbirsNisChiSquare2Dof95 = 5.99f;

float ComputeNormalizedInnovationSquared(const tracking::SbirsKalmanUpdateResult& update_result) {
  const Eigen::LLT<tracking::SbirsMeasurementCovariance> llt(update_result.innovation_covariance);
  if (llt.info() != Eigen::Success) {
    return std::numeric_limits<float>::infinity();
  }
  const tracking::SbirsMeasurementVector solved = llt.solve(update_result.innovation);
  return update_result.innovation.dot(solved);
}

float ComputeNormalizedInnovationSquared(const tracking::SbirsAngleCvUpdateResult& update_result) {
  const Eigen::LLT<tracking::SbirsMeasurementCovariance> llt(update_result.innovation_covariance);
  if (llt.info() != Eigen::Success) {
    return std::numeric_limits<float>::infinity();
  }
  const tracking::SbirsMeasurementVector solved = llt.solve(update_result.innovation);
  return update_result.innovation.dot(solved);
}

}  // namespace

void SbirsTrackingCoordinator::InitializeTarget(std::uint64_t target_id,
                                                const SbirsEciSceneTarget& target,
                                                const config::SbirsTrackingConfig& tracking,
                                                float measured_azimuth_deg,
                                                float measured_elevation_deg) {
  nis_gate_exceeded_counts_[target_id] = 0U;
  if (tracking.estimated_backend == config::SbirsEstimatedTrackingBackend::kAngleCvKf) {
    const float azimuth_rad = oneq::common::numerics::DegToRad(measured_azimuth_deg);
    const float elevation_rad = oneq::common::numerics::DegToRad(measured_elevation_deg);
    angle_kf_states_[target_id] = tracking::MakeInitialAngleCvState(azimuth_rad, elevation_rad);
    filter_states_.erase(target_id);
    imm_snapshots_.erase(target_id);
    imm_filters_by_target_.erase(target_id);
    (void)target;
    return;
  }

  tracking::SbirsGaussianState initial_state;
  initial_state.mean(0) = static_cast<float>(target.position_eci_m.x);
  initial_state.mean(2) = static_cast<float>(target.position_eci_m.y);
  initial_state.mean(4) = static_cast<float>(target.position_eci_m.z);
  if (target.has_velocity_eci_m_per_s) {
    initial_state.mean(1) = static_cast<float>(target.velocity_eci_m_per_s.x);
    initial_state.mean(3) = static_cast<float>(target.velocity_eci_m_per_s.y);
    initial_state.mean(5) = static_cast<float>(target.velocity_eci_m_per_s.z);
  }
  const float pos_var = tracking.initial_position_std_m * tracking.initial_position_std_m;
  const float vel_var =
      tracking.initial_velocity_std_m_per_s * tracking.initial_velocity_std_m_per_s;
  initial_state.covariance = tracking::SbirsStateCovariance::Zero();
  initial_state.covariance(0, 0) = pos_var;
  initial_state.covariance(1, 1) = vel_var;
  initial_state.covariance(2, 2) = pos_var;
  initial_state.covariance(3, 3) = vel_var;
  initial_state.covariance(4, 4) = pos_var;
  initial_state.covariance(5, 5) = vel_var;
  filter_states_[target_id] = initial_state;
  angle_kf_states_.erase(target_id);
  if (tracking.estimated_backend != config::SbirsEstimatedTrackingBackend::kImm) {
    return;
  }
  if (!imm_initialized_) {
    InitializeImmComponents(tracking);
  }
  imm_snapshots_.erase(target_id);
  CreateImmFilter(target_id, initial_state);
}

SbirsTrackingUpdateResult SbirsTrackingCoordinator::Update(
    std::uint64_t target_id, const config::SbirsPolicyConfig& policy,
    foundation::SbirsRandomSource* random_source, float azimuth_deg, float elevation_deg,
    double range_m, float relative_angular_rate_deg_per_sec, float dt_sec,
    const session::SbirsVector3M& satellite_position_eci_m) {
  PredictTarget(target_id, policy, dt_sec, satellite_position_eci_m);
  return CorrectTarget(target_id, policy, random_source, azimuth_deg, elevation_deg, range_m,
                       relative_angular_rate_deg_per_sec, satellite_position_eci_m);
}

SbirsTrackingPredictionResult SbirsTrackingCoordinator::PredictTarget(
    std::uint64_t target_id, const config::SbirsPolicyConfig& policy, float dt_sec,
    const session::SbirsVector3M& satellite_position_eci_m) {
  if (policy.tracking.estimated_backend == config::SbirsEstimatedTrackingBackend::kAngleCvKf) {
    auto state_it = angle_kf_states_.find(target_id);
    if (state_it == angle_kf_states_.end()) {
      return SbirsTrackingPredictionResult{};
    }
    ::oneq::common::estimation::KalmanPredictorConfig predictor_config;
    predictor_config.noise_diff_coeff = policy.tracking.process_noise_diff_coeff;
    const tracking::SbirsAngleCvPredictor predictor(predictor_config);
    state_it->second = predictor.Predict(state_it->second, dt_sec);
    return BuildAnglePredictionResult(state_it->second);
  }

  tracking::SbirsGaussianState predicted;
  if (policy.tracking.estimated_backend == config::SbirsEstimatedTrackingBackend::kImm) {
    if (!imm_initialized_) {
      InitializeImmComponents(policy.tracking);
    }
    auto filter_it = imm_filters_by_target_.find(target_id);
    if (filter_it == imm_filters_by_target_.end()) {
      const auto snapshot_it = imm_snapshots_.find(target_id);
      const tracking::SbirsGaussianState initial_state = filter_states_[target_id];
      CreateImmFilter(target_id, initial_state);
      filter_it = imm_filters_by_target_.find(target_id);
      if (snapshot_it != imm_snapshots_.end() && !snapshot_it->second.model_states.empty()) {
        filter_it->second->SetModelStates(snapshot_it->second.model_states);
        imm_snapshots_.erase(snapshot_it);
      }
    }
    filter_it->second->Predict(dt_sec);
    predicted = filter_it->second->GetCombinedState();
  } else {
    tracking::SbirsEkfPredictorConfig predictor_config;
    predictor_config.noise_diff_coeff = policy.tracking.process_noise_diff_coeff;
    const tracking::SbirsEkfPredictor predictor(
        &cv_transition_model_, predictor_config);
    predicted = predictor.Predict(filter_states_[target_id], dt_sec);
  }
  filter_states_[target_id] = predicted;
  return BuildPredictionResult(predicted, satellite_position_eci_m);
}

SbirsTrackingUpdateResult SbirsTrackingCoordinator::CorrectTarget(
    std::uint64_t target_id, const config::SbirsPolicyConfig& policy,
    foundation::SbirsRandomSource* random_source, float azimuth_deg, float elevation_deg,
    double range_m, float relative_angular_rate_deg_per_sec,
    const session::SbirsVector3M& satellite_position_eci_m, int frame_count) {
  SbirsTrackingUpdateResult result;
  // 高刷新率多帧融合（2026-08-31 窄场建模）：N 帧独立抽样融合为单周期量测，
  // 随机 1-σ 与量测协方差按 1/N 缩放；frame_count<=1 与既有单帧路径逐位一致。
  const foundation::SbirsFusedBearingResult fused = foundation::ApplyAngularErrorModelFused(
      policy.error_model, random_source, azimuth_deg, elevation_deg, range_m,
      relative_angular_rate_deg_per_sec, frame_count);
  const foundation::SbirsErrorBearing& bearing = fused.bearing;
  tracking::SbirsMeasurementVector measurement_rad;
  measurement_rad << oneq::common::numerics::DegToRad(bearing.azimuth_deg),
      oneq::common::numerics::DegToRad(bearing.elevation_deg);
  tracking::SbirsMeasurementCovariance measurement_covariance =
      tracking::BuildMeasurementCovariance(policy.error_model, range_m, elevation_deg,
                                           relative_angular_rate_deg_per_sec);
  if (frame_count > 1) {
    measurement_covariance /= static_cast<float>(frame_count);
  }
  result.has_measurement = true;
  result.measurement_azimuth_deg = bearing.azimuth_deg;
  result.measurement_elevation_deg = bearing.elevation_deg;
  result.measurement_covariance = measurement_covariance;

  tracking::SbirsGaussianState combined;
  if (policy.tracking.estimated_backend == config::SbirsEstimatedTrackingBackend::kAngleCvKf) {
    auto state_it = angle_kf_states_.find(target_id);
    if (state_it == angle_kf_states_.end()) {
      return result;
    }
    const tracking::SbirsAngleCvUpdater updater;
    const tracking::SbirsAngleCvUpdateResult update_result =
        updater.Update(state_it->second, measurement_rad, measurement_covariance);
    state_it->second = update_result.posterior;
    result.has_estimation_nis = true;
    result.estimation_nis = ComputeNormalizedInnovationSquared(update_result);
    result.estimation_nis_gate_exceeded = result.estimation_nis > kSbirsNisChiSquare2Dof95;
    if (policy.tracking.nis_gate_loss_cycles > 0U && result.estimation_nis_gate_exceeded) {
      const unsigned int exceeded_count = ++nis_gate_exceeded_counts_[target_id];
      result.lost_due_to_estimation_nis = exceeded_count >= policy.tracking.nis_gate_loss_cycles;
    } else {
      nis_gate_exceeded_counts_[target_id] = 0U;
    }
    const SbirsTrackingPredictionResult angles = BuildAnglePredictionResult(state_it->second);
    result.output_azimuth_deg = angles.output_azimuth_deg;
    result.output_elevation_deg = angles.output_elevation_deg;
    result.has_angle_rate = true;
    result.azimuth_rate_rad_per_s = state_it->second.mean(1);
    result.elevation_rate_rad_per_s = state_it->second.mean(3);
    return result;
  }
  if (policy.tracking.estimated_backend == config::SbirsEstimatedTrackingBackend::kImm) {
    auto filter_it = imm_filters_by_target_.find(target_id);
    for (auto& measurement_model : imm_measurement_models_) {
      measurement_model->SetSatellitePosition(satellite_position_eci_m);
    }
    tracking::SbirsImmFilter* const imm_filter = filter_it->second.get();
    imm_filter->Correct(measurement_rad, measurement_covariance);
    combined = imm_filter->GetCombinedState();
    filter_states_[target_id] = combined;

    result.has_estimation_nis = true;
    const auto& imm_results = imm_filter->GetModelUpdateResults();
    for (std::size_t index = 0U; index < imm_results.size(); ++index) {
      const float model_nis = ComputeNormalizedInnovationSquared(imm_results[index]);
      if (model_nis > result.estimation_nis) result.estimation_nis = model_nis;
      if (model_nis > kSbirsNisChiSquare2Dof95) result.estimation_nis_gate_exceeded = true;
    }
  } else {
    angle_measurement_model_.SetSatellitePosition(satellite_position_eci_m);
    const tracking::SbirsEkfUpdater updater(&angle_measurement_model_);
    const tracking::SbirsKalmanUpdateResult update_result =
        updater.Update(filter_states_[target_id], measurement_rad, measurement_covariance);
    combined = update_result.posterior;
    filter_states_[target_id] = combined;
    result.has_estimation_nis = true;
    result.estimation_nis = ComputeNormalizedInnovationSquared(update_result);
    result.estimation_nis_gate_exceeded = result.estimation_nis > kSbirsNisChiSquare2Dof95;
  }

  if (policy.tracking.nis_gate_loss_cycles > 0U && result.estimation_nis_gate_exceeded) {
    const unsigned int exceeded_count = ++nis_gate_exceeded_counts_[target_id];
    result.lost_due_to_estimation_nis = exceeded_count >= policy.tracking.nis_gate_loss_cycles;
  } else {
    nis_gate_exceeded_counts_[target_id] = 0U;
  }
  session::SbirsVector3M estimated_position;
  estimated_position.x = combined.mean(0);
  estimated_position.y = combined.mean(2);
  estimated_position.z = combined.mean(4);
  const session::SbirsVector3M estimated_los =
      foundation::Subtract(estimated_position, satellite_position_eci_m);
  result.output_azimuth_deg = foundation::ComputeAzimuthDeg(estimated_los);
  result.output_elevation_deg = foundation::ComputeElevationDeg(estimated_los);
  return result;
}

void SbirsTrackingCoordinator::MarkMeasurementUnavailable(std::uint64_t target_id) {
  nis_gate_exceeded_counts_[target_id] = 0U;
}

SbirsTrackingPredictionResult SbirsTrackingCoordinator::BuildPredictionResult(
    const tracking::SbirsGaussianState& state,
    const session::SbirsVector3M& satellite_position_eci_m) {
  session::SbirsVector3M estimated_position;
  estimated_position.x = state.mean(0);
  estimated_position.y = state.mean(2);
  estimated_position.z = state.mean(4);
  const session::SbirsVector3M estimated_los =
      foundation::Subtract(estimated_position, satellite_position_eci_m);
  SbirsTrackingPredictionResult result;
  result.output_azimuth_deg = foundation::ComputeAzimuthDeg(estimated_los);
  result.output_elevation_deg = foundation::ComputeElevationDeg(estimated_los);
  return result;
}

SbirsTrackingPredictionResult SbirsTrackingCoordinator::BuildAnglePredictionResult(
    const tracking::SbirsAngleCvGaussianState& state) {
  SbirsTrackingPredictionResult result;
  result.output_azimuth_deg = oneq::common::numerics::RadToDeg(state.mean(0));
  result.output_elevation_deg = oneq::common::numerics::RadToDeg(state.mean(2));
  return result;
}

void SbirsTrackingCoordinator::ReleaseTarget(std::uint64_t target_id) {
  filter_states_.erase(target_id);
  angle_kf_states_.erase(target_id);
  nis_gate_exceeded_counts_.erase(target_id);
  imm_filters_by_target_.erase(target_id);
  imm_snapshots_.erase(target_id);
}

void SbirsTrackingCoordinator::ResetNisGateCounts() {
  for (auto& entry : nis_gate_exceeded_counts_) {
    entry.second = 0U;
  }
}

void SbirsTrackingCoordinator::ClearForStandby() {
  filter_states_.clear();
  angle_kf_states_.clear();
  nis_gate_exceeded_counts_.clear();
  imm_filters_by_target_.clear();
  imm_snapshots_.clear();
  imm_initialized_ = false;
}

SbirsTrackingRuntimeState SbirsTrackingCoordinator::CaptureRuntimeState() const {
  SbirsTrackingRuntimeState state;
  state.filter_states = filter_states_;
  state.angle_kf_states = angle_kf_states_;
  state.nis_gate_exceeded_counts = nis_gate_exceeded_counts_;
  state.imm_active = imm_initialized_;
  state.imm_snapshots = imm_snapshots_;
  for (const auto& entry : imm_filters_by_target_) {
    tracking::SbirsImmSnapshot snapshot;
    snapshot.model_states = entry.second->GetModelStates();
    snapshot.model_weights = entry.second->GetModelWeights();
    state.imm_snapshots[entry.first] = snapshot;
  }
  state.imm_active = !state.imm_snapshots.empty();
  return state;
}

void SbirsTrackingCoordinator::RestoreRuntimeState(const SbirsTrackingRuntimeState& state) {
  filter_states_ = state.filter_states;
  angle_kf_states_ = state.angle_kf_states;
  nis_gate_exceeded_counts_ = state.nis_gate_exceeded_counts;
  imm_filters_by_target_.clear();
  imm_snapshots_ = state.imm_snapshots;
  imm_initialized_ = false;
}

void SbirsTrackingCoordinator::InitializeImmComponents(
    const config::SbirsTrackingConfig& tracking) {
  imm_predictors_owned_.clear();
  imm_predictors_.clear();
  imm_measurement_models_.clear();
  imm_updaters_owned_.clear();
  imm_updaters_.clear();
  imm_filters_by_target_.clear();

  const std::vector<float> q_values = tracking.imm_model_noise_diff_coeffs.empty()
                                          ? std::vector<float>{1.0f, 100.0f}
                                          : tracking.imm_model_noise_diff_coeffs;
  const int num_models = static_cast<int>(q_values.size());
  for (int i = 0; i < num_models; ++i) {
    auto measurement_model = std::make_unique<tracking::SbirsAngleMeasurementModel>();
    imm_measurement_models_.push_back(std::move(measurement_model));
    tracking::SbirsEkfPredictorConfig predictor_config;
    predictor_config.noise_diff_coeff = q_values[static_cast<std::size_t>(i)];
    auto predictor =
        std::make_unique<tracking::SbirsEkfPredictor>(&cv_transition_model_, predictor_config);
    imm_predictors_.push_back(predictor.get());
    imm_predictors_owned_.push_back(std::move(predictor));
    auto updater =
        std::make_unique<tracking::SbirsEkfUpdater>(imm_measurement_models_.back().get());
    imm_updaters_.push_back(updater.get());
    imm_updaters_owned_.push_back(std::move(updater));
  }

  imm_initialized_ = true;
}

tracking::SbirsImmFilter* SbirsTrackingCoordinator::CreateImmFilter(
    std::uint64_t target_id, const tracking::SbirsGaussianState& initial_state) {
  const int num_models = static_cast<int>(imm_predictors_.size());
  const float initial_weight = 1.0f / static_cast<float>(num_models);
  std::vector<tracking::SbirsImmModelState> initial_model_states;
  initial_model_states.reserve(static_cast<std::size_t>(num_models));
  for (int index = 0; index < num_models; ++index) {
    initial_model_states.push_back({initial_state, initial_weight});
  }

  tracking::SbirsImmConfig imm_config;
  imm_config.transition_probability.resize(num_models, num_models);
  for (int row = 0; row < num_models; ++row) {
    for (int column = 0; column < num_models; ++column) {
      imm_config.transition_probability(row, column) =
          (row == column) ? 0.95f : 0.05f / static_cast<float>(num_models - 1);
    }
  }
  imm_config.initial_weights.setConstant(num_models, initial_weight);
  std::unique_ptr<tracking::SbirsImmFilter> filter =
      std::make_unique<tracking::SbirsImmFilter>(imm_config, imm_predictors_, imm_updaters_);
  filter->SetModelStates(initial_model_states);
  tracking::SbirsImmFilter* const result = filter.get();
  imm_filters_by_target_[target_id] = std::move(filter);
  return result;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
