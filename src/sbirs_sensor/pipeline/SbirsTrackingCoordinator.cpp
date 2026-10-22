#include "sbirs_sensor/pipeline/SbirsTrackingCoordinator.h"

#include <Eigen/Cholesky>

#include <cmath>
#include <limits>

#include "sbirs_sensor/foundation/SbirsGeometry.h"
#include "sbirs_sensor/pipeline/SbirsStateSanity.h"
#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

constexpr float kSbirsNisChiSquare2Dof95 = 5.99f;
/**
 * @brief 离群量测门限（2 维 NIS，≈31σ）：超过即丢弃量测，后验保持预测。
 * @note 与统计门（95%，5.99）分开：95% 门按定义就有 5% 的正常量测越界，只能用于计数/丢锁
 *       策略；现场真正的毒数据是 2.5° 级离群角（NIS~1e7）与协方差奇异（NIS=inf），用 1000
 *       作阈值既挡得住毒数据，也不会把正常量测当离群丢掉（2026-10-21）。
 */
constexpr float kSbirsNisOutlierChiSquare = 1000.0f;
/** @brief 离群量测连续被拒的最大拍数（配置 nis_gate_loss_cycles=0 时的兜底丢锁判据）。 */
constexpr unsigned int kSbirsMaxConsecutiveRejections = 3U;

/**
 * @brief 丢锁判据：配置 >0 按配置；=0（"禁用 NIS 丢锁"）时仍保留 3 拍保护。
 * @note 2026-10-21：现场 nis_gate_loss_cycles=0 且离群量测照旧进入状态，导致状态发散。
 *       现在离群量测被丢弃、状态保持预测，但若一直丢弃就会带着预测态无限滑行——
 *       故 0 不再等于"永不丢锁"，而是回到 3 拍的量测-状态失配保护。
 */
unsigned int ConsecutiveRejectionThreshold(const config::SbirsPolicyConfig& policy) {
  return policy.tracking.nis_gate_loss_cycles > 0U ? policy.tracking.nis_gate_loss_cycles
                                                   : kSbirsMaxConsecutiveRejections;
}

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

bool TryDeriveAngleRateRadPerSec(const session::SbirsVector3M& target_position_eci_m,
                                 const session::SbirsVector3M& target_velocity_eci_m,
                                 const session::SbirsVector3M& satellite_position_eci_m,
                                 const session::SbirsVector3M& satellite_velocity_eci_m,
                                 float* azimuth_rate_rad_per_s,
                                 float* elevation_rate_rad_per_s) {
  if (azimuth_rate_rad_per_s == nullptr || elevation_rate_rad_per_s == nullptr) {
    return false;
  }
  // 中译：相对几何 —— 目标相对卫星的位置/速度；卫星自身速度必须减掉，否则 GEO 卫星
  //       在 ECI 的约 3 km/s 会把派生角速率完全主导。
  // 标识：第 77 项角速率派生口径（2026-10-19，A1）。
  const double x = target_position_eci_m.x - satellite_position_eci_m.x;
  const double y = target_position_eci_m.y - satellite_position_eci_m.y;
  const double z = target_position_eci_m.z - satellite_position_eci_m.z;
  const double vx = target_velocity_eci_m.x - satellite_velocity_eci_m.x;
  const double vy = target_velocity_eci_m.y - satellite_velocity_eci_m.y;
  const double vz = target_velocity_eci_m.z - satellite_velocity_eci_m.z;

  const double rho_squared = x * x + y * y;
  const double range_squared = rho_squared + z * z;
  // 中译：几何退化门 —— 视轴与天顶重合（ρ=0）时方位角速率无定义；距离为零时整式无意义。
  // 标识：A1 派生的唯一边界条件，退化时由调用方输出"不适用"而不编造数值。
  if (!(rho_squared > 1.0) || !(range_squared > 1.0)) {
    return false;
  }
  const double rho = std::sqrt(rho_squared);
  const double azimuth_rate = (x * vy - y * vx) / rho_squared;
  const double elevation_rate =
      (rho * vz - z * (x * vx + y * vy) / rho) / range_squared;
  if (!std::isfinite(azimuth_rate) || !std::isfinite(elevation_rate)) {
    return false;
  }
  *azimuth_rate_rad_per_s = static_cast<float>(azimuth_rate);
  *elevation_rate_rad_per_s = static_cast<float>(elevation_rate);
  return true;
}

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
    const session::SbirsVector3M& satellite_position_eci_m,
    const session::SbirsVector3M& satellite_velocity_eci_m) {
  PredictTarget(target_id, policy, dt_sec, satellite_position_eci_m);
  return CorrectTarget(target_id, policy, random_source, azimuth_deg, elevation_deg, range_m,
                       relative_angular_rate_deg_per_sec, satellite_position_eci_m,
                       satellite_velocity_eci_m);
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
    const session::SbirsVector3M& satellite_position_eci_m,
    const session::SbirsVector3M& satellite_velocity_eci_m, int frame_count) {
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
    result.has_estimation_nis = true;
    result.estimation_nis = ComputeNormalizedInnovationSquared(update_result);
    result.innovation_covariance_singular = !std::isfinite(result.estimation_nis);
    result.estimation_nis_gate_exceeded =
        result.innovation_covariance_singular ||
        result.estimation_nis > kSbirsNisChiSquare2Dof95;
    // 离群量测不得进入状态（2026-10-21）：此前"先更新、后计数"，超门的量测照样改写后验，
    // 把 ±2.5° 的离群角写进状态并逐拍放大（现场第 9 项残差超 ±360″ 的行有 95% 落在
    // NIS=inf 的同期）。这里按离群门（≠ 统计门，见 kSbirsNisOutlierChiSquare）丢弃量测。
    const bool outlier = result.innovation_covariance_singular ||
                         result.estimation_nis > kSbirsNisOutlierChiSquare;
    if (outlier) {
      result.measurement_discarded = true;
    } else {
      state_it->second = update_result.posterior;
    }
    if (result.estimation_nis_gate_exceeded) {
      ++nis_gate_exceeded_counts_[target_id];
      result.lost_due_to_estimation_nis =
          nis_gate_exceeded_counts_[target_id] >= ConsecutiveRejectionThreshold(policy);
    } else {
      nis_gate_exceeded_counts_[target_id] = 0U;
    }
    if (!IsSaneSbirsAngleState(state_it->second.mean, state_it->second.covariance)) {
      result.state_diverged = true;
      result.lost_due_to_estimation_nis = true;
      angle_kf_states_.erase(state_it);
      return result;
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

    result.has_estimation_nis = true;
    const auto& imm_results = imm_filter->GetModelUpdateResults();
    for (std::size_t index = 0U; index < imm_results.size(); ++index) {
      const float model_nis = ComputeNormalizedInnovationSquared(imm_results[index]);
      if (model_nis > result.estimation_nis) result.estimation_nis = model_nis;
      if (model_nis > kSbirsNisChiSquare2Dof95) result.estimation_nis_gate_exceeded = true;
    }
    result.innovation_covariance_singular = !std::isfinite(result.estimation_nis);
    result.estimation_nis_gate_exceeded =
        result.estimation_nis_gate_exceeded || result.innovation_covariance_singular;
    // IMM 的更新发生在滤波器内部，无法"只丢弃后验"——离群或发散时整条状态作废，
    // 由宽场重新捕获（与 NIS 连续超门同一释放路径，避免把发散状态发到日志）。
    const bool outlier = result.innovation_covariance_singular ||
                         result.estimation_nis > kSbirsNisOutlierChiSquare;
    const bool sane = IsSaneSbirsState(combined.mean, combined.covariance);
    if (outlier || !sane) {
      result.measurement_discarded = true;
      result.state_diverged = !sane;
      result.lost_due_to_estimation_nis = true;
      imm_filters_by_target_.erase(filter_it);
      filter_states_.erase(target_id);
      return result;
    }
    filter_states_[target_id] = combined;
  } else {
    angle_measurement_model_.SetSatellitePosition(satellite_position_eci_m);
    const tracking::SbirsEkfUpdater updater(&angle_measurement_model_);
    const tracking::SbirsKalmanUpdateResult update_result =
        updater.Update(filter_states_[target_id], measurement_rad, measurement_covariance);
    result.has_estimation_nis = true;
    result.estimation_nis = ComputeNormalizedInnovationSquared(update_result);
    result.innovation_covariance_singular = !std::isfinite(result.estimation_nis);
    result.estimation_nis_gate_exceeded =
        result.innovation_covariance_singular ||
        result.estimation_nis > kSbirsNisChiSquare2Dof95;
    const bool outlier = result.innovation_covariance_singular ||
                         result.estimation_nis > kSbirsNisOutlierChiSquare;
    if (outlier) {
      // 离群量测丢弃：后验保持预测（2026-10-21，见 kAngleCvKf 分支注释）。
      result.measurement_discarded = true;
      combined = filter_states_[target_id];
    } else {
      combined = update_result.posterior;
    }
    if (!IsSaneSbirsState(combined.mean, combined.covariance)) {
      result.state_diverged = true;
      result.lost_due_to_estimation_nis = true;
      filter_states_.erase(target_id);
      return result;
    }
    filter_states_[target_id] = combined;
  }

  if (result.estimation_nis_gate_exceeded) {
    ++nis_gate_exceeded_counts_[target_id];
    result.lost_due_to_estimation_nis =
        nis_gate_exceeded_counts_[target_id] >= ConsecutiveRejectionThreshold(policy);
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
  // 中译：EKF/IMM 的状态里没有角度速率分量，但后验同时带位置与速度，故直接解析派生，
  //       不做跨拍差分（差分需要逐目标历史状态，且重捕后首拍无定义）。
  // 标识：第 77 项角速度字段的数据源（2026-10-19，A1）；退化时保持 has_angle_rate=false，
  //       由日志侧输出"不适用"，不得编造 0。
  session::SbirsVector3M estimated_velocity;
  estimated_velocity.x = combined.mean(1);
  estimated_velocity.y = combined.mean(3);
  estimated_velocity.z = combined.mean(5);
  float azimuth_rate_rad_per_s = 0.0f;
  float elevation_rate_rad_per_s = 0.0f;
  if (TryDeriveAngleRateRadPerSec(estimated_position, estimated_velocity,
                                  satellite_position_eci_m, satellite_velocity_eci_m,
                                  &azimuth_rate_rad_per_s, &elevation_rate_rad_per_s)) {
    result.has_angle_rate = true;
    result.azimuth_rate_rad_per_s = azimuth_rate_rad_per_s;
    result.elevation_rate_rad_per_s = elevation_rate_rad_per_s;
  }
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
