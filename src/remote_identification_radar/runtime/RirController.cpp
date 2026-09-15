/**
 * @file RirController.cpp
 * @brief 远程识别雷达自持链路控制器实现（阶段 2-S S2）。
 */

#include "remote_identification_radar/runtime/RirController.h"

#include <Eigen/Cholesky>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/atmosphere/AtmospherePhysics.h"
#include "common/geometry/EarthOccultation.h"
#include "common/logging/AcceptanceText.h"
#include "common/logging/ProjectLog.h"
#include "common/numerics/Constants.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/environment/AtmosphericTypes.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "remote_identification_radar/dwell/RirBeamControl.h"
#include "remote_identification_radar/dwell/RirEffectiveRcs.h"
#include "remote_identification_radar/dwell/RirEmissionFactory.h"
#include "remote_identification_radar/dwell/RirMeasurementErrorModel.h"
#include "remote_identification_radar/dwell/RirReceiverStateBuilder.h"
#include "remote_identification_radar/dwell/RirRfFrontEndResolver.h"
#include "remote_identification_radar/internal/RirScanVolume.h"
#include "remote_identification_radar/recognition/RecognitionObservationBuilder.h"
#include "remote_identification_radar/runtime/RirAcceptanceLog.h"
#include "remote_identification_radar/runtime/RirAcceptanceRecords.h"

namespace remote_identification_radar {
namespace runtime {

namespace {

constexpr float kSnrFallbackGateDb = 6.0f;
constexpr float kCovarianceFloorM2 = 1.0e-6f;
// 未注入设备号时的缺省值，镜像融合常量 fusion::kRirSourceId（=5U）。分层契约
// 禁止传感器层引 fusion 头，故在本文件本地对齐取值；两侧若需改号须同步。
constexpr std::uint32_t kDefaultSensorEntityId = 5U;
// 主瓣覆盖门系数：门限=有效波束宽度（半功率宽）×该系数。恒为 1.0 且不设配置项
// （2026-08-29 还债原则：方向图/覆盖门语义唯一，不再新增同类开关）。
constexpr float kMainLobeGateBeamwidthScale = 1.0f;

// 多驻留发射记录的 emission_id 唯一化（核查 9.2）：RfSceneFrame 契约要求帧内
// identity 唯一（TryValidateRfSceneFrame 拒绝重复），编排层把本帧汇入全球 RF
// scene 供 ESR 等消费，同帧多条同 id 会让下游整帧校验失败。首驻留保持既有
// emission_id=cycle_index（单驻留周期与历史记录逐字节一致）；第 i 条驻留
// (i≥1) 以高位标志位 + (cycle_index << 32) | i 编码——与 uint32 周期号空间
// 不相交、帧内唯一，且是周期号的确定性函数（replay 无需额外会话状态，与 AR
// 的 next_emission_id 台账路线刻意区分）。(cycle, i) 的全局单射以
// cycle_index < 2^31、dwell_index < 2^32 为前提（现实计数远不可达）。
std::uint64_t SecondaryDwellEmissionId(std::uint32_t cycle_index, std::size_t dwell_index) {
  constexpr std::uint64_t kDwellEmissionIdFlag = 1ULL << 63;
  return kDwellEmissionIdFlag | (static_cast<std::uint64_t>(cycle_index) << 32) |
         static_cast<std::uint64_t>(dwell_index);
}

std::string g_acceptance_found_targets;

internal::RirSwerlingModel ToInternalSwerling(session::RirSwerlingType type) {
  switch (type) {
    case session::RirSwerlingType::kSwerling1:
      return internal::RirSwerlingModel::kSwerling1;
    case session::RirSwerlingType::kSwerling2:
      return internal::RirSwerlingModel::kSwerling2;
    case session::RirSwerlingType::kSwerling3:
      return internal::RirSwerlingModel::kSwerling3;
    case session::RirSwerlingType::kSwerling4:
      return internal::RirSwerlingModel::kSwerling4;
    case session::RirSwerlingType::kSwerling0:
    default:
      return internal::RirSwerlingModel::kSwerling0;
  }
}

Eigen::Vector3f PositionOf(const session::RirSceneTarget& target) {
  return Eigen::Vector3f(target.position_x, target.position_y, target.position_z);
}

/** @brief 内部航迹生命周期状态 → 公开枚举镜像（归属记录透出，观测投影消费）。 */
session::RirTrackLifecycleStatus ToPublicTrackStatus(tracking::RirTrackStatus status) {
  switch (status) {
    case tracking::RirTrackStatus::kConfirmed:
      return session::RirTrackLifecycleStatus::kConfirmed;
    case tracking::RirTrackStatus::kLost:
      return session::RirTrackLifecycleStatus::kLost;
    case tracking::RirTrackStatus::kTentative:
    default:
      return session::RirTrackLifecycleStatus::kTentative;
  }
}

/** @brief 内部四维观测 → 公开特征量测镜像（字段同值透出，仅命名对齐公开契约）。 */
session::RirFeatureObservations ToPublicFeatureObservations(const recognition::RirFeatureSet& set) {
  session::RirFeatureObservations features;
  features.rcs.valid = set.rcs.valid;
  features.rcs.mean_dbsm = set.rcs.mean_dbsm;
  features.rcs.std_db = set.rcs.std_db;
  features.rcs.azimuth_variation_db = set.rcs.azimuth_variation_db;
  features.rcs.elevation_variation_db = set.rcs.elevation_variation_db;
  features.rcs.peak_to_valley_db = set.rcs.peak_to_valley_db;
  features.rcs.aspect_coverage_deg = set.rcs.aspect_coverage_deg;
  features.rcs.quality = set.rcs.quality;
  features.motion.valid = set.motion.valid;
  features.motion.speed_m_per_s = set.motion.speed_mps;
  features.motion.altitude_m = set.motion.altitude_m;
  features.motion.acceleration_m_per_s2 = set.motion.acceleration_mps2;
  features.motion.turn_radius_m = set.motion.turn_radius_m;
  features.motion.is_straight = set.motion.is_straight;
  features.motion.quality = set.motion.quality;
  features.polarization.valid = set.polarization.valid;
  features.polarization.energy_difference_db = set.polarization.energy_difference_db;
  features.polarization.relative_difference_db = set.polarization.relative_difference_db;
  features.polarization.energy_sum_db = set.polarization.energy_sum_db;
  features.polarization.quality = set.polarization.quality;
  features.polarization_stats.valid = set.polarization_stats.valid;
  features.polarization_stats.determinant_mean =
      static_cast<float>(set.polarization_stats.determinant.mean);
  features.polarization_stats.determinant_std =
      static_cast<float>(set.polarization_stats.determinant.std);
  features.polarization_stats.span_mean = static_cast<float>(set.polarization_stats.span.mean);
  features.polarization_stats.span_std = static_cast<float>(set.polarization_stats.span.std);
  features.polarization_stats.depolarization_mean =
      static_cast<float>(set.polarization_stats.depolarization.mean);
  features.polarization_stats.depolarization_std =
      static_cast<float>(set.polarization_stats.depolarization.std);
  features.polarization_stats.psi_mean_deg =
      static_cast<float>(set.polarization_stats.psi_deg.mean);
  features.polarization_stats.psi_std_deg =
      static_cast<float>(set.polarization_stats.psi_deg.std);
  features.polarization_stats.tau_mean_deg =
      static_cast<float>(set.polarization_stats.tau_deg.mean);
  features.polarization_stats.tau_std_deg =
      static_cast<float>(set.polarization_stats.tau_deg.std);
  features.range_profile.valid = set.range_profile.valid;
  features.range_profile.length_m = set.range_profile.length_m;
  features.range_profile.peak_count = set.range_profile.peak_count;
  features.range_profile.peak_energy_concentration = set.range_profile.peak_energy_concentration;
  features.range_profile.resolution_m = set.range_profile.resolution_m;
  features.range_profile.quality = set.range_profile.quality;
  return features;
}

/// 格式化量值为两位小数（消息文本稳定，不承诺解析稳定性；规则 13b message 约定）。
std::string RirFormatFloat(float value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.2f", static_cast<double>(value));
  return buffer;
}

/// 构造 kInfo 级按目标排除诊断（不属于三写，仅承载排查信息；规则 13b）。
/// @param cause 门内归因；聚合门排除须给出主因，具体门可 kNone。
/// @param target_index 场景目标下标；写入 `location = {kSceneEntity, index}` 供
///                     跨周期差分记录器按实体关联消费。
session::RirIssue RirMakeExclusionIssue(const char* code, const std::string& message,
                                        session::RirIssueCause cause,
                                        std::size_t target_index) {
  session::RirIssue issue;
  issue.severity = session::RirIssueSeverity::kInfo;
  issue.phase = session::RirIssuePhase::kExecution;
  issue.code = code;
  issue.message = message;
  issue.cause = cause;
  issue.location.kind = oneq::foundation::ValidationLocationKind::kSceneEntity;
  issue.location.entity_index = target_index;
  return issue;
}

// 规则 13b 门内归因：检测准入门（detector 判决/SNR 回退门）折入距离/波束/噪声底/
// RCS 多种物理因素，按各因素相对参考状态的损失 dB 判定主因（损失最大者）。
// 参考态与 AR ClassifySnrExclusionCause 同口径：1 km 距离、主瓣中心增益、
// 1 m² RCS、热噪声底、零传播损耗；传播损耗并入距离项。
session::RirIssueCause ClassifyRirDetectionExclusionCause(
    float range_m, float effective_rcs_m2, float one_way_antenna_gain_db,
    float main_beam_gain_db, float propagation_loss_db, float total_noise_w,
    float thermal_noise_w) {
  constexpr float kReferenceRangeM = 1000.0f;
  constexpr float kMinRcsM2 = 1.0e-12f;
  constexpr float kMinNoiseW = 1.0e-30f;

  const float distance_loss_db =
      4.0f * 10.0f * std::log10(std::max(range_m, 1.0e-3f) / kReferenceRangeM) +
      std::max(propagation_loss_db, 0.0f);
  const float beam_loss_db =
      2.0f * std::max(main_beam_gain_db - one_way_antenna_gain_db, 0.0f);
  const float rcs_loss_db = -10.0f * std::log10(std::max(effective_rcs_m2, kMinRcsM2));
  const float noise_loss_db =
      thermal_noise_w > 0.0f
          ? 10.0f * std::log10(std::max(total_noise_w, kMinNoiseW) /
                               std::max(thermal_noise_w, kMinNoiseW))
          : 0.0f;

  float max_loss_db = 0.0f;
  session::RirIssueCause cause = session::RirIssueCause::kUnknown;
  const struct {
    float loss_db;
    session::RirIssueCause cause;
  } kCandidates[] = {
      {distance_loss_db, session::RirIssueCause::kDistanceLimited},
      {beam_loss_db, session::RirIssueCause::kBeamLimited},
      {noise_loss_db, session::RirIssueCause::kNoiseLimited},
      {rcs_loss_db, session::RirIssueCause::kRcsLimited},
  };
  for (const auto& candidate : kCandidates) {
    if (candidate.loss_db > max_loss_db) {
      max_loss_db = candidate.loss_db;
      cause = candidate.cause;
    }
  }
  return max_loss_db > 0.0f ? cause : session::RirIssueCause::kUnknown;
}

}  // namespace

void RirController::SetHardware(const config::RirHardwareConfig& hardware) {
  hardware_ = hardware;
  if (detector_ == nullptr) {
    detector_.reset(new dwell::RirSignalDetector(MakeDetectorConfig()));
  } else {
    detector_->UpdateConfig(MakeDetectorConfig());
  }
  detector_->SetRandomSeed(detection_random_seed_);
  measurement_rng_.seed(detection_random_seed_);
}

void RirController::SetSensorEntityId(std::uint32_t sensor_entity_id) {
  sensor_entity_id_ = sensor_entity_id;
}

std::uint32_t RirController::EffectiveSensorEntityId() const {
  return sensor_entity_id_ != 0U ? sensor_entity_id_ : kDefaultSensorEntityId;
}

dwell::RirDetectorConfig RirController::MakeDetectorConfig() const {
  dwell::RirDetectorConfig config;
  config.transmitter = hardware_.transmitter;
  config.antenna = hardware_.antenna;
  config.receiver = hardware_.receiver;
  config.detection_policy.cfar_pfa = policy_.detection.cfar_pfa;
  config.detection_policy.min_snr_db = policy_.detection.min_snr_db;
  config.min_detection_margin_db = policy_.detection.min_detection_margin_db;
  config.pulse_count = policy_.detection.pulse_count;
  return config;
}

void RirController::UpdateRuntime(const config::RirMissionConfig& mission,
                                  const config::RirPolicyConfig& policy) {
  mission_ = mission;
  work_mode_ = mission.work_mode;
  policy_ = policy;
  detection_random_seed_ = policy_.detection.random_seed;

  recognition::RirTracker::Options options;
  options.min_confirmed_hits = policy_.recognition.min_confirmed_hits;
  options.accumulation_window_sec = policy_.recognition.accumulation_window_sec;
  options.min_observation_count = policy_.recognition.min_observation_count;
  options.acceptance_score = policy_.recognition.acceptance_score;
  options.minimum_margin = policy_.recognition.minimum_margin;
  options.result_hold_sec = policy_.recognition.result_hold_sec;
  options.max_range_m = mission_.max_range_m;
  tracker_.SetOptions(options);

  tracking::RirLifecycleConfig lifecycle_config;
  lifecycle_config.confirm_hits = policy_.lifecycle.confirm_hits;
  lifecycle_config.max_miss_before_lost = policy_.lifecycle.max_miss_before_lost;
  lifecycle_config.max_lost_cycles = policy_.lifecycle.max_lost_cycles;
  lifecycle_config.enable_imm_lifecycle = policy_.lifecycle.enable_imm_lifecycle;
  lifecycle_config.model_count_hint = policy_.lifecycle.model_count_hint;
  tracking::RirTrackFilterConfig filter_config;
  filter_config.process_noise_diff_coeff = policy_.tracking.kalman_noise_diff_coeff;
  lifecycle_->UpdateConfig(lifecycle_config, filter_config);

  tracking::RirAssociationConfig association_config;
  const float sigma = std::max(0.0f, policy_.association.distance_gate_sigma);
  association_config.gate_threshold = sigma * sigma;
  association_config.kalman_noise_diff_coeff = policy_.tracking.kalman_noise_diff_coeff;
  associator_.UpdateConfig(association_config);

  if (detector_ == nullptr) {
    detector_.reset(new dwell::RirSignalDetector(MakeDetectorConfig()));
  } else {
    detector_->UpdateConfig(MakeDetectorConfig());
  }
  detector_->SetRandomSeed(detection_random_seed_);
  measurement_rng_.seed(detection_random_seed_);

  // 数据库按需加载：路径变化且已启用时重新加载；失败保持原库（识别降级 kDisabled）。
  if (policy_.recognition.enabled && !policy_.recognition.database_path.empty() &&
      policy_.recognition.database_path != database_path_) {
    std::unique_ptr<recognition::RirFeatureDatabase> candidate(
        new recognition::RirFeatureDatabase());
    std::string error;
    const std::chrono::steady_clock::time_point load_begin = std::chrono::steady_clock::now();
    const bool loaded = recognition::RirFeatureDatabase::Load(
        policy_.recognition.database_path, candidate.get(), &error);
    last_database_load_ms_ = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - load_begin)
                                 .count();
    if (loaded) {
      database_ = std::move(candidate);
      database_path_ = policy_.recognition.database_path;
      tracker_.SetActiveDatabaseVersion(database_->version());
    } else {
      // 中译：识别特征数据库加载失败，本次更新不生效，识别链路保持当前库或 kDisabled 降级。
      // 标识：RirController::UpdateRuntime，数据库路径/格式错误时触发；不影响检测跟踪链路。
      PROJECT_LOG_ERROR("[RirController] recognition database load failed: {}", error);
    }
  }
}

void RirController::UpdateEnvironment(const config::RirEnvironmentConfig& environment) {
  environment_ = environment;
  // k 因子随气象观测派生（对标 AR EnvironmentService 冻结快照口径；非法输入内部兜底 4/3）。
  effective_k_factor_ = oneq::environment::ResolveEffectiveKFactor(
      environment.atmospheric_physics);
}

void RirController::ResolveEnvironment(float* propagation_loss_db) const {
  // weather_attenuation_db == 0 表示无天气附加损耗。
  *propagation_loss_db = environment_.weather_attenuation_db;
  if (environment_.vegetation_cover_profile ==
      config::RirVegetationCoverProfile::kDisabled) {
    return;
  }
  internal::RirEnvironmentSceneState scene_state;
  scene_state.vegetation_cover_profile = environment_.vegetation_cover_profile;
  const internal::RirPropagationResult propagation = propagation_model_.Evaluate(scene_state);
  *propagation_loss_db += propagation.propagation_loss_db;
}

float RirController::ResolveTargetClutterPowerW(float look_el_deg, float slant_range_m,
                                                float total_propagation_loss_db,
                                                float carrier_hz,
                                                float bandwidth_hz) const {
  if (environment_.vegetation_cover_profile ==
      config::RirVegetationCoverProfile::kDisabled) {
    return 0.0f;
  }
  internal::RirSurfaceClutterInput clutter_input;
  clutter_input.vegetation_cover_profile = environment_.vegetation_cover_profile;
  clutter_input.transmitter = hardware_.transmitter;
  // 与回波链同口径：规划载频优先，非正回退发射机频率；带宽取匹配滤波带宽
  //（与检测单元一致），决定杂波距离单元 c/(2B)。
  clutter_input.transmitter.frequency_hz =
      carrier_hz > 0.0f ? carrier_hz : hardware_.transmitter.frequency_hz;
  clutter_input.transmitter.bandwidth_hz =
      bandwidth_hz > 0.0f ? bandwidth_hz : hardware_.transmitter.bandwidth_hz;
  clutter_input.antenna = hardware_.antenna;
  clutter_input.propagation_loss_db = total_propagation_loss_db;
  clutter_input.range_m = slant_range_m;
  clutter_input.look_el_deg = look_el_deg;
  clutter_input.thermal_noise_w = internal::RirRadarEquations::ComputeThermalNoisePower_W(
      hardware_.transmitter, hardware_.receiver);
  return surface_clutter_model_.EvaluateClutterNoiseW(clutter_input);
}

float RirController::ComputeTargetAtmosphericLossDb(float carrier_hz, float platform_altitude_m,
                                                    float look_el_deg, bool has_look_angles,
                                                    float slant_range_m,
                                                    float target_position_z_m) const {
  if (!environment_.atmospheric_physics.enable_physical_model) {
    return 0.0f;
  }
  // 与 AR DetectionExecution 同口径：common 标量胶水 + 模块侧 enable/k_factor。
  oneq::common::atmosphere::AtmosphericObservationRef obs;
  obs.pressure_hpa = environment_.atmospheric_physics.pressure_hpa;
  obs.temperature_k = environment_.atmospheric_physics.temperature_k;
  obs.relative_humidity = environment_.atmospheric_physics.relative_humidity;
  obs.k_factor = effective_k_factor_;
  return oneq::common::atmosphere::ComputeTargetAtmosphericPhysicsLossDb(
      carrier_hz, slant_range_m, platform_altitude_m,
      std::max(platform_altitude_m + target_position_z_m, 0.0f), look_el_deg, has_look_angles, obs);
}

void RirController::ComputeLookAngles(const session::RirSceneTarget& target, float* look_az_deg,
                                      float* look_el_deg, float* slant_range_m) {
  const Eigen::Vector3f position = PositionOf(target);
  const float range_hypot = std::sqrt(position.x() * position.x() + position.y() * position.y());
  *look_az_deg = oneq::common::numerics::RadToDeg(std::atan2(position.y(), position.x()));
  *look_el_deg = oneq::common::numerics::RadToDeg(std::atan2(position.z(), range_hypot));
  *slant_range_m = position.norm();
}

void RirController::LookAnglesFromPosition(const Eigen::Vector3f& position, float* look_az_deg,
                                           float* look_el_deg) {
  const float range_hypot = std::sqrt(position.x() * position.x() + position.y() * position.y());
  *look_az_deg = oneq::common::numerics::RadToDeg(std::atan2(position.y(), position.x()));
  *look_el_deg = oneq::common::numerics::RadToDeg(std::atan2(position.z(), range_hypot));
}

std::vector<RirDwellPlan> RirController::CollectTrackDwellRequests(std::size_t max_count,
                                                                   float predict_dt_sec) const {
  std::vector<RirDwellPlan> requests;
  if (max_count == 0U) {
    return requests;
  }
  for (const tracking::RirTrackState& track : last_track_snapshots_) {
    if (requests.size() >= max_count) {
      break;
    }
    if (track.status != tracking::RirTrackStatus::kConfirmed ||
        track.external_target_id == 0U) {
      continue;
    }
    // 库内航迹预测指向：末量测位置 + 速度 × 预测时移（不消费场景真值，
    // 与 designate 的外部导引语义刻意区分——跟踪是雷达自己的闭环）。
    const Eigen::Vector3f predicted = track.position + track.velocity * predict_dt_sec;
    RirDwellPlan dwell;
    LookAnglesFromPosition(predicted, &dwell.pointing_deg.az_deg, &dwell.pointing_deg.el_deg);
    dwell.kind = RirDwellKind::kTrack;
    dwell.external_target_id = track.external_target_id;
    requests.push_back(dwell);
  }
  return requests;
}

tracking::RirMeasurementCovariance RirController::MakeCartesianMeasurementCovariance(
    const session::RirSceneTarget& target, float range_std_m, float angle_std_rad) {
  const Eigen::Vector3f position = PositionOf(target);
  const float range_m = std::max(position.norm(), 1.0f);
  const float range_hypot = std::sqrt(position.x() * position.x() + position.y() * position.y());
  const float angle_variance = angle_std_rad * angle_std_rad;

  Eigen::Matrix3f jacobian = Eigen::Matrix3f::Zero();
  if (range_hypot > 1.0f) {
    // 列含义：∂ENU/∂(r, az, el)。仰角列=(−xz/rh, −yz/rh, rh)，模长=斜距 r
    // （m/rad）；历史实现整列多除 r²，仰角贡献按 1/r⁴ 坍缩（审计 A4）。
    jacobian(0, 0) = position.x() / range_m;
    jacobian(0, 1) = -position.y();
    jacobian(0, 2) = -position.x() * position.z() / range_hypot;
    jacobian(1, 0) = position.y() / range_m;
    jacobian(1, 1) = position.x();
    jacobian(1, 2) = -position.y() * position.z() / range_hypot;
    jacobian(2, 0) = position.z() / range_m;
    jacobian(2, 1) = 0.0f;
    jacobian(2, 2) = range_hypot;
  } else {
    jacobian(0, 0) = position.x() / range_m;
    jacobian(0, 2) = position.x() / range_m;
    jacobian(1, 0) = position.y() / range_m;
    jacobian(1, 2) = position.y() / range_m;
    jacobian(2, 0) = position.z() / range_m;
    jacobian(2, 2) = position.z() / range_m;
  }

  Eigen::Vector3f variances(range_std_m * range_std_m, angle_variance, angle_variance);
  tracking::RirMeasurementCovariance covariance =
      jacobian * variances.asDiagonal() * jacobian.transpose();
  covariance.diagonal().array() += kCovarianceFloorM2;
  return covariance;
}

tracking::RirTrackMeasurement RirController::SampleMeasurementPosition(
    const tracking::RirTrackMeasurement& measurement) {
  tracking::RirTrackMeasurement sampled = measurement;
  Eigen::LLT<tracking::RirMeasurementCovariance> llt(measurement.measurement_covariance);
  if (llt.info() != Eigen::Success) {
    return sampled;
  }
  std::normal_distribution<float> normal(0.0f, 1.0f);
  Eigen::Vector3f noise(normal(measurement_rng_), normal(measurement_rng_),
                        normal(measurement_rng_));
  sampled.position += llt.matrixL() * noise;
  return sampled;
}

RirController::RirResolvedRfCycle RirController::ResolveRfCycle(
    const session::RirCycleInput& input, const std::vector<RirDwellPlan>& plan) const {
  RirResolvedRfCycle resolved;
  if (plan.empty() || hardware_.transmitter.prf_hz <= 0.0f) {
    return resolved;
  }

  dwell::RirRfCycleInput rf_input;
  rf_input.sensor_entity_id = EffectiveSensorEntityId();
  rf_input.platform_position_ecef_m = input.platform_position;
  rf_input.window_start_time_s = static_cast<double>(input.sim_time_sec);
  rf_input.window_duration_s = static_cast<double>(mission_.recognition_dwell_sec);
  // 接收链（饱和/入射链/波形/身份）按计划首驻留指向周期级解析一次——
  // 接收侧聚合语义不变（核查 9.2 只改发射记录的逐驻留透出）。
  rf_input.beam_pointing_deg = plan.front().pointing_deg;

  const double carrier_hz =
      dwell::RirEmissionFactory::ResolveCarrierHz(hardware_.transmitter, input.input_cycle_index);
  const double pri_s = 1.0 / static_cast<double>(hardware_.transmitter.prf_hz);
  // 与 AR PrepareRfCycle 同口径：驻留窗内脉冲数按 ceil 计（首沿落入窗口的脉冲
  // 全部计入积累增益），下限 1 保持驻留窗短于一个 PRI 时仍有单脉冲。
  const double pulse_count_value =
      std::ceil(static_cast<double>(mission_.recognition_dwell_sec) / pri_s);
  const std::uint32_t pulse_count =
      static_cast<std::uint32_t>(std::max(1.0, pulse_count_value));

  oneq::electromagnetics::RfSceneEmission own_emission;
  if (!dwell::RirEmissionFactory::TryBuildEmission(
          rf_input, hardware_, static_cast<std::uint64_t>(input.input_cycle_index), carrier_hz,
          pri_s, pulse_count, static_cast<std::uint64_t>(detection_random_seed_),
          static_cast<std::uint64_t>(input.input_cycle_index), &own_emission)) {
    return resolved;
  }

  resolved.own_emission_identity = own_emission.identity;
  resolved.own_emission = own_emission;
  resolved.own_transmit_waveform = own_emission.waveform;
  resolved.carrier_hz = static_cast<float>(carrier_hz);

  // 发射记录逐驻留（核查 9.2）：一周期多驻留必须对外完整输出（否则下游只见
  // 首波位）。首条复用周期级 own_emission；其余驻留经同一工厂构建路径生成
  // （波形/功率/极化/时序种子与首条同源同值，仅指向换该驻留、emission_id
  // 唯一化）。单条构建失败只少记该驻留，不影响接收链与检测判决。
  resolved.per_dwell_emissions.push_back(own_emission);
  for (std::size_t i = 1U; i < plan.size(); ++i) {
    dwell::RirRfCycleInput dwell_input = rf_input;
    dwell_input.beam_pointing_deg = plan[i].pointing_deg;
    oneq::electromagnetics::RfSceneEmission dwell_emission;
    if (dwell::RirEmissionFactory::TryBuildEmission(
            dwell_input, hardware_, SecondaryDwellEmissionId(input.input_cycle_index, i),
            carrier_hz, pri_s, pulse_count, static_cast<std::uint64_t>(detection_random_seed_),
            static_cast<std::uint64_t>(input.input_cycle_index), &dwell_emission)) {
      resolved.per_dwell_emissions.push_back(dwell_emission);
    }
  }

  const dwell::RirReceiverOperatingState receiver_state =
      dwell::RirReceiverStateBuilder::Build(rf_input, own_emission, hardware_, carrier_hz);

  oneq::electromagnetics::RfSceneFrame resolved_scene = input.rf_scene;
  resolved_scene.world_cycle_index = input.input_cycle_index;
  resolved_scene.window_start_time_s = rf_input.window_start_time_s;
  resolved_scene.window_duration_s = rf_input.window_duration_s;
  resolved_scene.emissions.push_back(own_emission);

  dwell::RirRfFrontEndResult front_end;
  if (!dwell::TryResolveRirRfFrontEnd(
          resolved_scene, receiver_state.rf_receiver,
          receiver_state.maximum_linear_input_power_w,
          oneq::electromagnetics::RfIncidentLinkConfig{}, &front_end)) {
    return resolved;
  }

  resolved.incident_links = std::move(front_end.incident_links);
  resolved.receiver_saturated = front_end.receiver_saturated;
  resolved.resolved = true;
  return resolved;
}

bool RirController::TryBuildMeasurement(
    const session::RirSceneTarget& target, std::size_t source_index, float platform_altitude_m,
    float propagation_loss_db, const session::RirCycleInput& input,
    const config::RirAzimuthElevationDeg& dwell_center_deg, const RirResolvedRfCycle& rf_cycle,
    tracking::RirTrackMeasurement* measurement, float* snr_db) {
  if (measurement == nullptr || snr_db == nullptr) {
    return false;
  }
  // 接收前端饱和周期致盲（核查 5.2 裁定）：饱和由 RirRfFrontEndResolver 周期级
  // 判定（本接收窗全部入射链聚合功率 > receiver.maximum_linear_input_power_w，
  // 含自发射同平台耦合）——越限后前端失真/致盲，本周期所有目标不做检测判决。
  // 规则 13b：链上第一门优先，饱和门排在 SNR 检测门之前；每目标每周期至多这一条。
  if (rf_cycle.receiver_saturated) {
    // 中译：接收前端饱和周期致盲——线性区越限，本周期全部目标直接按门控排除，不再检测。
    // 标识：RirController::TryBuildMeasurement 饱和门；替代旧口径"仅告警仍照常判决"。
    PROJECT_LOG_WARN("[RirController] receiver front-end saturated at cycle={}; detection blinded",
                     input.input_cycle_index);
    last_execution_issues_.push_back(RirMakeExclusionIssue(
        session::codes::kTargetReceiverFrontEndSaturated,
        "target_id=" + std::to_string(target.external_target_id) +
            "; receiver front-end saturated, all detections blinded this cycle",
        session::RirIssueCause::kNone, source_index));
    return false;
  }
  float look_az_deg = 0.0f;
  float look_el_deg = 0.0f;
  float slant_range_m = 0.0f;
  ComputeLookAngles(target, &look_az_deg, &look_el_deg, &slant_range_m);

  // 与 AR TargetLookResolver 同口径：位置范数 ≤0.1 m 时视线角无效
  // （零位附近 az/el 不稳定）→ 方向图增益回退主瓣峰值。
  const Eigen::Vector3f raw_position(target.position_x, target.position_y, target.position_z);
  const bool has_look_angles = raw_position.norm() > 0.1f;
  const float carrier_hz =
      rf_cycle.carrier_hz > 0.0f ? rf_cycle.carrier_hz : hardware_.transmitter.frequency_hz;
  // 逐目标大气物理附加损耗（与 AR 同口径：全局植被/天气损耗之外的按几何分量）。
  const float atmospheric_loss_db = ComputeTargetAtmosphericLossDb(
      carrier_hz, platform_altitude_m, look_el_deg, has_look_angles, slant_range_m,
      PositionOf(target).z());
  const float total_propagation_loss_db = propagation_loss_db + atmospheric_loss_db;
  // 逐目标主瓣地杂波（植被开启时）：按目标几何（斜距/俯仰擦地）与雷达参数
  // 物理求解，替代旧会话级常数；与检测单元杂波分项同注入点。
  const float clutter_power_w = ResolveTargetClutterPowerW(
      look_el_deg, slant_range_m, total_propagation_loss_db, carrier_hz,
      rf_cycle.resolved
          ? static_cast<float>(rf_cycle.own_transmit_waveform.occupied_bandwidth_hz)
          : hardware_.transmitter.bandwidth_hz);
  // 与 AR DetectionExecution 同口径：主路径与回退分支波长同源——回退分支的有效
  // 波束宽度同样经 λ/L 物理推导（结果喂量测误差模型）；载频非正时不启用物理推导。
  const float wavelength_m = carrier_hz > 0.0f
                                 ? static_cast<float>(oneq::common::numerics::kLightSpeed) /
                                       carrier_hz
                                 : 0.0f;
  // 方向图恒开（2026-08-29 架构还债：删除 enable_directional_pattern 死选项）：
  // 库内驻留调度器给定波束中心，目标离轴增益按（目标视线角 - 驻留中心）衰减
  // （与 AR 冻结指向链路同口径）；无视线角（位置范数 ≤0.1 m 的退化输入）由
  // common 回退主瓣峰值。
  const dwell::RirResolvedBeamState beam_state = dwell::RirResolveBeamStateForPointing(
      hardware_.antenna, dwell_center_deg, look_az_deg, look_el_deg, has_look_angles,
      wavelength_m);

  dwell::RirTargetReturn target_return;
  target_return.rcs_m2 = target.rcs;
  target_return.range_m = slant_range_m;
  target_return.swerling_type = ToInternalSwerling(target.target_swerling_type);
  const Eigen::Vector3f radial_position = PositionOf(target);
  const float radial_range_norm = std::max(radial_position.norm(), 1.0f);
  const float radial_velocity =
      (radial_position.x() * target.velocity_x + radial_position.y() * target.velocity_y +
       radial_position.z() * target.velocity_z) /
      radial_range_norm;

  dwell::RirDetectionResult detection;
  dwell::RirDetectionCellResult cell;
  bool resolved_cell = false;
  // 量测误差模型的积累脉冲数 N：detection cell 路径 = 接收窗截断后的
  // cell.effective_pulse_count；回退路径 = 传给 Detect 的同一 pulse_count。
  // detection.snr_db 两条路径均为单脉冲口径（common StatisticalCfarDetector），
  // N 只折算进误差模型输入，不改写 SNR 透出值，也不影响 Pd（CFAR 内部已按
  // pulse_count 积累）。
  std::uint32_t integration_pulse_count = 1U;
  if (rf_cycle.resolved) {
    dwell::RirDetectionCellConfig cell_config;
    cell_config.own_transmit_waveform = rf_cycle.own_transmit_waveform;
    cell_config.receive_window_start_time_s = static_cast<double>(input.sim_time_sec);
    cell_config.receive_window_duration_s = static_cast<double>(mission_.recognition_dwell_sec);
    // 与 AR DetectionExecution 同口径：匹配滤波带宽取本周期波形实际占用带宽
    // （当前发射工厂与 hardware bandwidth_hz 恒等；波形级带宽解耦后两侧同源）。
    cell_config.matched_filter_bandwidth_hz =
        rf_cycle.own_transmit_waveform.occupied_bandwidth_hz;
    cell_config.one_way_antenna_gain_dbi = static_cast<double>(beam_state.one_way_antenna_gain_db);
    cell_config.receiver_loss_db = static_cast<double>(hardware_.receiver.receive_loss_db);
    cell_config.receiver_noise_figure_db = static_cast<double>(hardware_.receiver.noise_figure_db);
    cell_config.signal_processing = hardware_.signal_processing;

    dwell::RirDetectionCellTarget cell_target;
    cell_target.range_m = static_cast<double>(slant_range_m);
    cell_target.closing_radial_velocity_mps = static_cast<double>(-radial_velocity);
    dwell::RirTargetLookAngles look_angles;
    look_angles.look_az_deg = look_az_deg;
    look_angles.look_el_deg = look_el_deg;
    look_angles.has_look_angles = has_look_angles;
    const float carrier_hz =
        rf_cycle.carrier_hz > 0.0f ? rf_cycle.carrier_hz : hardware_.transmitter.frequency_hz;
    cell_target.rcs_m2 = static_cast<double>(dwell::ComputeEffectiveTargetRcsM2(
        target, look_angles, hardware_.rcs_physics, carrier_hz));
    cell_target.two_way_additional_propagation_loss_db =
        static_cast<double>(total_propagation_loss_db);
    cell_target.effective_pulse_count =
        static_cast<std::uint32_t>(std::max(1, policy_.detection.pulse_count));

    if (dwell::TryResolveRirDetectionCell(cell_config, cell_target, rf_cycle.own_emission_identity,
                                        rf_cycle.incident_links, static_cast<double>(clutter_power_w),
                                        &cell)) {
      target_return.rcs_m2 = static_cast<float>(cell_target.rcs_m2);
      detection = detector_->DetectResolvedCell(target_return, cell);
      resolved_cell = true;
      // 生效积累脉冲数 = 接收窗截断后的可用回波脉冲数（TryCountAvailableEchoPulses
      // 输出）；下限 1 防御零脉冲退化输入（后续 10·log10(N) 折算无意义）。
      integration_pulse_count = std::max(1U, cell.effective_pulse_count);
    } else {
      // 中译：detection cell 求解失败，回退效能级检测路径。
      // 标识：数值/输入保护——RF 分解失败时不丢目标，按旧口径继续判决。
      PROJECT_LOG_WARN("[RirController] detection cell resolve failed for target id={}",
                       target.external_target_id);
    }
  }

  if (!resolved_cell) {
    const int fallback_pulse_count = std::max(1, policy_.detection.pulse_count);
    dwell::RirEnvironmentNoise env;
    env.propagation_loss_db = total_propagation_loss_db;
    env.clutter_noise_w = clutter_power_w;
    env.jam_noise_w = 0.0f;
    detection = detector_->Detect(target_return, env, beam_state.one_way_antenna_gain_db,
                                  fallback_pulse_count);
    integration_pulse_count = static_cast<std::uint32_t>(fallback_pulse_count);
  }

  *snr_db = detection.snr_db;
  const bool admitted = policy_.detection.gate_mode == config::RirDetectionGateMode::kSnrFallback
                            ? detection.snr_db >= kSnrFallbackGateDb
                            : detection.detected;
  // 验收事件 detection_cell（3.2.2.1.1.x/3.2.2.1.2）：逐目标检测链物理量——方向图
  // 离轴增益/有效波束宽度/离轴角、回波/热噪/干扰/杂波功率、脉压增益、SINR/SNR、
  // Pd 与判决。has_cell=0 为 v1 效能级回退路径（cell 分项功率不可得，只有 SNR/Pd）。
  if (RIR_ACCEPTANCE_LOG_ENABLED()) {
    RirDetectionAcceptInput snap;
    snap.sim_time_sec = input.sim_time_sec;
    snap.cycle = input.input_cycle_index;
    snap.radar_id = EffectiveSensorEntityId();
    snap.target_id = target.external_target_id;
    snap.range_m = slant_range_m;
    snap.look_az_deg = look_az_deg;
    snap.look_el_deg = look_el_deg;
    snap.rcs_m2 = target_return.rcs_m2;
    snap.snr_db = detection.snr_db;
    snap.pd = detection.detection_prob;
    snap.detected = detection.detected;
    snap.has_cell = resolved_cell;
    snap.peak_gain_dbi = hardware_.antenna.main_beam_gain_db;
    snap.bw_az_deg = beam_state.effective_beamwidth_deg.az_beamwidth_deg;
    snap.bw_el_deg = beam_state.effective_beamwidth_deg.el_beamwidth_deg;
    snap.one_way_antenna_gain_dbi = beam_state.one_way_antenna_gain_db;
    snap.bandwidth_hz = resolved_cell
                            ? rf_cycle.own_transmit_waveform.occupied_bandwidth_hz
                            : static_cast<double>(hardware_.transmitter.bandwidth_hz);
    snap.echo_power_dbw = detection.echo_power_dbw;
    snap.cell = cell;
    snap.gains = hardware_.signal_processing;
    snap.cfar_pfa = static_cast<double>(policy_.detection.cfar_pfa);
    snap.prf_hz = static_cast<double>(hardware_.transmitter.prf_hz);
    snap.center_frequency_hz =
        rf_cycle.own_transmit_waveform.center_frequency_hz > 0.0
            ? rf_cycle.own_transmit_waveform.center_frequency_hz
            : static_cast<double>(hardware_.transmitter.frequency_hz);
    snap.incident_links.reserve(rf_cycle.incident_links.size());
    for (const auto& link : rf_cycle.incident_links) {
      if (link.identity.sensor_entity_id == rf_cycle.own_emission_identity.sensor_entity_id &&
          link.identity.emission_id == rf_cycle.own_emission_identity.emission_id) {
        continue;
      }
      snap.incident_links.push_back(link);
    }
    WriteRirDetectionChain(snap);
  }

  if (!admitted) {
    // 规则 13b：检测准入门排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
    // 检测门为聚合门（距离/波束/噪声底/RCS 折入单一判决），按各因素相对参考态
    // 损失 dB 判定主因（与 AR ClassifySnrExclusionCause 同口径）。
    const float thermal_noise_w = internal::RirRadarEquations::ComputeThermalNoisePower_W(
        hardware_.transmitter, hardware_.receiver);
    float total_noise_w = thermal_noise_w + clutter_power_w;
    if (resolved_cell) {
      total_noise_w = static_cast<float>(cell.thermal_noise_power_w + cell.clutter_power_w +
                                         cell.interference_power_w);
    }
    const session::RirIssueCause cause = ClassifyRirDetectionExclusionCause(
        slant_range_m, target_return.rcs_m2, beam_state.one_way_antenna_gain_db,
        hardware_.antenna.main_beam_gain_db, total_propagation_loss_db, total_noise_w,
        thermal_noise_w);
    const char* gate_mode_text =
        policy_.detection.gate_mode == config::RirDetectionGateMode::kSnrFallback
            ? "snr_fallback"
            : "detector";
    last_execution_issues_.push_back(RirMakeExclusionIssue(
        session::codes::kTargetDetectionGate,
        "target_id=" + std::to_string(target.external_target_id) + "; snr_db=" +
            RirFormatFloat(detection.snr_db) + " range_m=" + RirFormatFloat(slant_range_m) +
            " rcs_m2=" + RirFormatFloat(target_return.rcs_m2) +
            " rejected by detection gate (mode=" + gate_mode_text + ")",
        cause, source_index));
    return false;
  }
  // 误差模型输入口径（核查 6.1 裁定）：模型期望"等效积累后"信噪比，而
  // detection.snr_db 是单脉冲 SINR/SNR → 输入 = 单脉冲值 + 10·log10(N) 积累折算
  // （N=生效积累脉冲数，两分支填充见 integration_pulse_count）。旧口径直传单脉冲
  // 值，角度/距离误差偏大约 √N（N=10 时 3.2 倍）。
  const float integrated_snr_db =
      detection.snr_db + 10.0f * std::log10(static_cast<float>(integration_pulse_count));
  const dwell::RirMeasurementErrorState measurement_error =
      dwell::RirMeasurementErrorModel::Compute(integrated_snr_db,
                                               beam_state.effective_beamwidth_deg,
                                               hardware_.transmitter.bandwidth_hz);
  tracking::RirTrackMeasurement built;
  built.source_index = source_index;
  built.external_target_id = target.external_target_id;
  built.target_name = target.target_name;
  built.position = PositionOf(target);
  // 去真值化（rir-tracking-realism 契约）：雷达单次检测无速度量测通道，旧口径的
  // 真值速度种子属泄漏；滤波速度唯一来源为后验收敛。字段保留填零（公开面与回放
  // 兼容，契约修订 1 第 4 条）。
  built.velocity = Eigen::Vector3f::Zero();
  // 评审 2026-08-26 条15：检测概率透传进关联量测（验收旁路字段，不进关联方程）。
  built.detection_pd = detection.detection_prob;
  built.detection_snr_db = detection.snr_db;
  built.radial_velocity_m_per_s = radial_velocity;
  built.rcs = target.rcs;
  built.measurement_covariance = MakeCartesianMeasurementCovariance(
      target, measurement_error.range_error_std_m, measurement_error.angle_error_std_rad);
  // 6 dB 回退模式保持旧识别门控口径：量测位置取真值，只携带误差协方差；
  // 检测器门控模式按量测误差模型采样位置（自持链路的量测扰动语义）。
  if (policy_.detection.gate_mode == config::RirDetectionGateMode::kSnrFallback) {
    *measurement = built;
  } else {
    // 系统偏差施加（核查 6.2 拆分裁定）：偏置属均值侧而非协方差——采样前把量测
    // 位置沿视线 +20 m（kRangeMeasurementBiasM）、角偏 bw/30 逐轴正偏的横向位移
    // 平移固定量（未标定系统残差，方向取正）；SnrFallback 真值口径不施加（与不
    // 采样噪声同层级简化）。退化输入（无视线角）跳过，与峰值回退口径一致。
    if (has_look_angles) {
      const float bias_az_rad =
          oneq::common::radar::ComputeAngleMeasurementBiasRad(oneq::common::numerics::DegToRad(
              beam_state.effective_beamwidth_deg.az_beamwidth_deg));
      const float bias_el_rad =
          oneq::common::radar::ComputeAngleMeasurementBiasRad(oneq::common::numerics::DegToRad(
              beam_state.effective_beamwidth_deg.el_beamwidth_deg));
      const float az_rad = oneq::common::numerics::DegToRad(look_az_deg);
      const float el_rad = oneq::common::numerics::DegToRad(look_el_deg);
      const Eigen::Vector3f u(std::cos(el_rad) * std::cos(az_rad),
                              std::cos(el_rad) * std::sin(az_rad), std::sin(el_rad));
      const Eigen::Vector3f e_az(-std::sin(az_rad), std::cos(az_rad), 0.0f);
      const Eigen::Vector3f e_el(-std::sin(el_rad) * std::cos(az_rad),
                                 -std::sin(el_rad) * std::sin(az_rad), std::cos(el_rad));
      built.position +=
          oneq::common::radar::kRangeMeasurementBiasM * u +
          (slant_range_m * std::cos(el_rad) * bias_az_rad) * e_az +
          (slant_range_m * bias_el_rad) * e_el;
    }
    *measurement = SampleMeasurementPosition(built);
  }
  // 验收判定标准 第37项（检测与量测信息）：逐检测目标串接——量测斜距与量测方位/
  // 俯仰取采样后的量测位置（6 dB 回退模式量测位置即真值）；Pd 归第 36 项，不写。
  if (RIR_ACCEPTANCE_LOG_ENABLED()) {
    float measured_range_m = 0.0f;
    float measured_az_deg = 0.0f;
    float measured_el_deg = 0.0f;
    if (runtime::TryLookPolarFromEnuM(measurement->position.x(), measurement->position.y(),
                                      measurement->position.z(), &measured_range_m,
                                      &measured_az_deg, &measured_el_deg)) {
      if (!g_acceptance_found_targets.empty()) {
        g_acceptance_found_targets += "; ";
      }
      g_acceptance_found_targets +=
          "ID=" + std::to_string(target.external_target_id) + " 斜距=" +
          oneq::logging::FormatF(measured_range_m, 1) + "m 量测方位/俯仰=(" +
          oneq::logging::FormatF(measured_az_deg, 3) + "," +
          oneq::logging::FormatF(measured_el_deg, 3) + ")°";
    }
  }
  return true;
}

recognition::RirObservationContext RirController::MakeObservationContext(
    const session::RirSceneTarget& target, float platform_altitude_m, float snr_db) const {
  recognition::RirObservationContext context;
  context.snr_db = snr_db;
  context.bandwidth_hz = hardware_.transmitter.bandwidth_hz;
  context.dwell_sec = mission_.recognition_dwell_sec;
  float look_az_deg = 0.0f;
  float look_el_deg = 0.0f;
  float slant_range_m = 0.0f;
  ComputeLookAngles(target, &look_az_deg, &look_el_deg, &slant_range_m);
  context.range_m = slant_range_m;
  context.look_az_deg = look_az_deg;
  context.look_el_deg = look_el_deg;
  context.platform_altitude_m = platform_altitude_m;
  context.minimum_aspect_coverage_deg = 0.0f;
  return context;
}

void RirController::RunCycle(const session::RirCycleInput& input,
                            session::RirOutputFrame* output_frame, std::uint64_t batch_id,
                            const config::RirAzimuthElevationDeg& dwell_center_deg,
                            const config::RirAzimuthElevationLimitsDeg& steerable_volume_deg,
                            const config::RirAzimuthElevationDeg& scan_center_deg,
                            const config::RirAzimuthElevationLimitsDeg& scan_window_deg,
                            std::uint64_t designated_external_target_id) {
  // 单驻留重载 = 单条目搜索驻留计划（直连调用方与既有行为兼容）。
  std::vector<RirDwellPlan> single_dwell_plan(1U);
  single_dwell_plan.front().pointing_deg = dwell_center_deg;
  RunCycle(input, output_frame, batch_id, single_dwell_plan, steerable_volume_deg,
           scan_center_deg, scan_window_deg, designated_external_target_id);
}

void RirController::RunCycle(const session::RirCycleInput& input,
                            session::RirOutputFrame* output_frame, std::uint64_t batch_id,
                            const std::vector<RirDwellPlan>& dwell_plan,
                            const config::RirAzimuthElevationLimitsDeg& steerable_volume_deg,
                            const config::RirAzimuthElevationDeg& scan_center_deg,
                            const config::RirAzimuthElevationLimitsDeg& scan_window_deg,
                            std::uint64_t designated_external_target_id) {
  // TAS 防御：空计划退化为单条目缺省指向搜索驻留（既有兜底语义不变）。
  static const std::vector<RirDwellPlan> kFallbackPlan(1U);
  const std::vector<RirDwellPlan>& plan = dwell_plan.empty() ? kFallbackPlan : dwell_plan;
  // 验收"指向"行与接收链回退口径按计划首条目；发射记录逐驻留
  //（见 ResolveRfCycle / last_emission_frame_，核查 9.2）。
  const config::RirAzimuthElevationDeg& dwell_center_deg = plan.front().pointing_deg;
  const bool in_identify = work_mode_ == config::RirWorkMode::kIdentify;
  if (recognition_mode_active_ && !in_identify) {
    tracker_.ExitRecognitionMode();
  }
  recognition_mode_active_ = in_identify;
  sim_time_sec_ = input.sim_time_sec;

  float platform_altitude_m = 0.0f;
  oneq::coordinate::LlaPositionDegM platform_lla;
  const bool has_platform_lla =
      oneq::coordinate::TryEcefToLla(input.platform_position, &platform_lla);
  if (has_platform_lla) {
    platform_altitude_m = static_cast<float>(platform_lla.altitude_m);
  }

  latest_summary_ = session::RirRecognitionCycleSummary{};
  has_latest_summary_ = false;
  // 出口①与归属视图按周期重置（透出原则：识别链未构建观测的周期为空）。
  output_frame->feature_measurements.clear();
  last_track_attributions_.clear();
  last_execution_issues_.clear();
  last_emission_frame_ = {};
  // 实际有效目标最大斜距按周期重置：非 kIdentify / 无航迹周期回填 0。
  last_max_detected_slant_range_m_ = 0.0f;
  last_cycle_snr_by_target_id_.clear();

  std::vector<tracking::RirTrackState> track_snapshots;
  if (in_identify) {
    float propagation_loss_db = 0.0f;
    ResolveEnvironment(&propagation_loss_db);

    const RirResolvedRfCycle rf_cycle = ResolveRfCycle(input, plan);
    if (rf_cycle.resolved) {
      last_emission_frame_.world_cycle_index = input.input_cycle_index;
      last_emission_frame_.window_start_time_s = static_cast<double>(input.sim_time_sec);
      last_emission_frame_.window_duration_s =
          static_cast<double>(mission_.recognition_dwell_sec);
      // 逐驻留发射记录（核查 9.2）：窗口字段语义不变，emissions 携带本周期
      // 全部驻留的发射（单驻留周期仍恰一条，与历史行为一致）。
      last_emission_frame_.emissions = rf_cycle.per_dwell_emissions;
    }

    // 验收事件 interference_link（3.2.2.1.1.3）：逐干扰源到达接收端的入射功率
    // （RF 前端口径，未做 detection cell 内时频重叠聚合；聚合后进入 SINR 分母的
    // 总量见 detection_cell 事件 interference_w 字段）。
    if (RIR_ACCEPTANCE_LOG_ENABLED()) {
      g_acceptance_found_targets.clear();
      WriteRirInterferenceLinks(input.sim_time_sec, input.input_cycle_index, rf_cycle.incident_links);
    }

    // 上一周期航迹快照先行（子窗豁免判定与候选排序同源消费）。
    const std::vector<tracking::RirTrackState> prior_tracks = lifecycle_->BuildTrackSnapshots();
    std::unordered_map<std::uint64_t, std::uint32_t> recognition_rank_by_target_id;
    std::unordered_map<std::uint64_t, std::uint32_t> confirmed_track_by_target;
    for (const tracking::RirTrackState& track : prior_tracks) {
      const session::RirRecognitionResult* result = tracker_.FindResult(track.association_key);
      const bool recognized =
          result != nullptr && (result->state == session::RirRecognitionState::kCategoryConfirmed ||
                                result->state == session::RirRecognitionState::kModelConfirmed);
      recognition_rank_by_target_id[track.external_target_id] = recognized ? 1U : 0U;
      if (track.status == tracking::RirTrackStatus::kConfirmed) {
        confirmed_track_by_target[track.external_target_id] = 1U;
      }
    }

    std::unordered_map<std::uint64_t, const session::RirSceneTarget*> scene_by_id;
    std::unordered_map<std::uint64_t, float> slant_by_target_id;
    // 候选条目：场景索引 + 缓存的视线角/退化标志（主瓣覆盖门在逐驻留循环内判定）。
    struct RirCandidate {
      std::size_t source_index;
      config::RirAzimuthElevationDeg look;
      char degenerate;
    };
    std::vector<RirCandidate> candidates;
    // 实际搜索扇区 = 任务扫描子窗 ∩ 硬件可扫描体积（子窗缺省无界时等于体积，
    // 与既有行为兼容）。搜索态候选按此扇区裁剪；指定识别目标与确认航迹目标
    // 在硬件体积内豁免子窗（跟踪连续性高于搜索子窗，2026-08-29 TAS）。
    const config::RirAzimuthElevationLimitsDeg search_sector =
        internal::IntersectScanSector(scan_window_deg, steerable_volume_deg);
    // 主瓣覆盖门半宽（半功率宽×系数，驻留指向每轴各开这么宽）：与方向图同一
    // 波束宽度源（有效波束宽度，nominal → λ/L 物理推导），且同一载频口径
    // （与 TryBuildMeasurement 一致：规划频率优先，兜底发射频率）。
    const float gate_carrier_hz = rf_cycle.carrier_hz > 0.0f
                                      ? rf_cycle.carrier_hz
                                      : hardware_.transmitter.frequency_hz;
    const float gate_wavelength_m =
        gate_carrier_hz > 0.0f
            ? static_cast<float>(oneq::common::numerics::kLightSpeed) / gate_carrier_hz
            : 0.0f;
    const dwell::RirEffectiveBeamwidthDeg gate_beamwidth =
        dwell::RirResolveEffectiveBeamwidth(hardware_.antenna, gate_wavelength_m);
    const float main_lobe_half_az_deg =
        0.5f * kMainLobeGateBeamwidthScale * gate_beamwidth.az_beamwidth_deg;
    const float main_lobe_half_el_deg =
        0.5f * kMainLobeGateBeamwidthScale * gate_beamwidth.el_beamwidth_deg;
    for (std::size_t i = 0U; i < input.scene_targets.size(); ++i) {
      const session::RirSceneTarget& target = input.scene_targets[i];
      if (target.external_target_id == 0U) {
        continue;
      }
      scene_by_id[target.external_target_id] = &target;
      // 地球遮挡（有限弦-圆球，与 SBIRS 同口径）：穿地视线不入检测候选，
      // 排在可扫描体积与 SNR 之前。ENU→ECEF 失败则跳过本门，不伪装成遮挡。
      if (has_platform_lla) {
        const Eigen::Vector3f position = PositionOf(target);
        const oneq::coordinate::EnuPositionM enu(static_cast<double>(position.x()),
                                                 static_cast<double>(position.y()),
                                                 static_cast<double>(position.z()));
        oneq::coordinate::EcefPositionM target_ecef;
        if (oneq::coordinate::TryEnuToEcef(enu, platform_lla, &target_ecef) &&
            oneq::common::geometry::IsEarthOcculted(input.platform_position, target_ecef,
                                                   oneq::common::geometry::kMeanEarthRadiusM)) {
          const double occultation_margin_m =
              oneq::common::geometry::ComputeEarthOccultationMarginM(
                  input.platform_position, target_ecef, oneq::common::geometry::kMeanEarthRadiusM);
          last_execution_issues_.push_back(RirMakeExclusionIssue(
              session::codes::kTargetEarthOcculted,
              "target_id=" + std::to_string(target.external_target_id) +
                  "; LOS from platform occulted by Earth; occultation_margin_m=" +
                  std::to_string(static_cast<long long>(occultation_margin_m)),
              session::RirIssueCause::kNone, i));
          continue;
        }
      }
      // 2026-08-22 甲方批注「设定方位俯仰进行扫描」+「任务范围由用户指定」：
      // 搜索候选集按实际搜索扇区裁剪（az 相对 scan_center、el 绝对）——视线角出扇区
      // 的目标不入检测候选，不再依赖方向图衰减软门控。
      float look_az_deg = 0.0f;
      float look_el_deg = 0.0f;
      float slant_range_m = 0.0f;
      ComputeLookAngles(target, &look_az_deg, &look_el_deg, &slant_range_m);
      const config::RirAzimuthElevationDeg look{look_az_deg, look_el_deg};
      const bool in_hardware =
          internal::TargetWithinSteerableVolume(look, steerable_volume_deg, scan_center_deg);
      const bool in_search =
          internal::TargetWithinSteerableVolume(look, search_sector, scan_center_deg);
      const bool is_designated = designated_external_target_id != 0U &&
                                 target.external_target_id == designated_external_target_id;
      const bool has_confirmed_track =
          confirmed_track_by_target.count(target.external_target_id) != 0U;
      if (!(in_search || ((is_designated || has_confirmed_track) && in_hardware))) {
        // 规则 13b：角域裁剪排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
        // 出界目标不入检测候选、航迹按失跟语义自然消退，发码使"目标消失"可归因。
        // 区分两类门：出硬件体积（steerable volume）/ 在体积内但出任务子窗（scan window）。
        const std::string exclusion_detail =
            !in_hardware
                ? ("target_id=" + std::to_string(target.external_target_id) +
                   "; look_az_deg=" + RirFormatFloat(look.az_deg) +
                   " look_el_deg=" + RirFormatFloat(look.el_deg) +
                   " outside steerable volume az_rel_deg=[" +
                   RirFormatFloat(steerable_volume_deg.az_min_deg) + "," +
                   RirFormatFloat(steerable_volume_deg.az_max_deg) + "] el_deg=[" +
                   RirFormatFloat(steerable_volume_deg.el_min_deg) + "," +
                   RirFormatFloat(steerable_volume_deg.el_max_deg) + "]")
                : ("target_id=" + std::to_string(target.external_target_id) +
                   "; look_az_deg=" + RirFormatFloat(look.az_deg) +
                   " look_el_deg=" + RirFormatFloat(look.el_deg) +
                   " outside mission scan window az_rel_deg=[" +
                   RirFormatFloat(search_sector.az_min_deg) + "," +
                   RirFormatFloat(search_sector.az_max_deg) + "] el_deg=[" +
                   RirFormatFloat(search_sector.el_min_deg) + "," +
                   RirFormatFloat(search_sector.el_max_deg) + "]");
        last_execution_issues_.push_back(RirMakeExclusionIssue(
            session::codes::kTargetOutsideSearchVolume, exclusion_detail,
            session::RirIssueCause::kNone, i));
        continue;
      }
      // 退化判定与 TryBuildMeasurement 同口径（按原始位置范数）；
      // 主瓣覆盖门在逐驻留循环内判定。
      const Eigen::Vector3f gate_position(target.position_x, target.position_y,
                                          target.position_z);
      slant_by_target_id[target.external_target_id] = slant_range_m;
      RirCandidate candidate;
      candidate.source_index = i;
      candidate.look = look;
      candidate.degenerate = gate_position.norm() <= 0.1f ? 1 : 0;
      candidates.push_back(candidate);
    }

    std::sort(
        candidates.begin(), candidates.end(),
        [&](const RirCandidate& lhs, const RirCandidate& rhs) {
          const session::RirSceneTarget& lhs_target = input.scene_targets[lhs.source_index];
          const session::RirSceneTarget& rhs_target = input.scene_targets[rhs.source_index];
          const std::uint32_t lhs_rank =
              recognition_rank_by_target_id[lhs_target.external_target_id];
          const std::uint32_t rhs_rank =
              recognition_rank_by_target_id[rhs_target.external_target_id];
          if (lhs_rank != rhs_rank) {
            return lhs_rank < rhs_rank;
          }
          const float lhs_range = PositionOf(lhs_target).norm();
          const float rhs_range = PositionOf(rhs_target).norm();
          return lhs_range < rhs_range;
        });

    std::vector<tracking::RirTrackMeasurement> measurements;
    const float dwell_sec = mission_.recognition_dwell_sec;
    // 驻留预算按计划计（2026-08-29 TAS：口径从"逐候选"改为"逐驻留"）：
    // scheduled/executed = 本周期计划/实际执行的驻留条目数。
    const std::uint32_t scheduled_count = static_cast<std::uint32_t>(plan.size());
    std::uint32_t executed_count = 0U;
    // 逐目标覆盖记账：候选被任一驻留照到即尝试量测（首驻留胜出，不重复尝试）；
    // 从未被照到的候选在循环后发主瓣覆盖排除诊断。
    std::vector<char> candidate_covered(candidates.size(), 0);
    for (const RirDwellPlan& dwell : plan) {
      // 每条计划驻留都实际驻留（波束照射与候选是否过检测门无关）。
      ++executed_count;
      for (std::size_t k = 0U; k < candidates.size(); ++k) {
        if (candidate_covered[k] != 0U) {
          continue;  // 首驻留胜出（同目标同周期不重复量测）。
        }
        const RirCandidate& candidate = candidates[k];
        const session::RirSceneTarget& target = input.scene_targets[candidate.source_index];
        // 主瓣覆盖门（探测⟺波束照到，2026-08-29 还债）：方位差经环绕归一；
        // 退化输入（无真实视线角）跳过本门，与峰值回退口径一致。
        if (candidate.degenerate == 0) {
          const float coverage_az_delta_deg =
              std::fabs(oneq::common::radar::NormalizeAzimuthDeltaDeg(
                  candidate.look.az_deg - dwell.pointing_deg.az_deg));
          const float coverage_el_delta_deg =
              std::fabs(candidate.look.el_deg - dwell.pointing_deg.el_deg);
          if (coverage_az_delta_deg > main_lobe_half_az_deg ||
              coverage_el_delta_deg > main_lobe_half_el_deg) {
            continue;
          }
        }
        candidate_covered[k] = 1;
        tracking::RirTrackMeasurement measurement;
        float snr_db = 0.0f;
        if (!TryBuildMeasurement(target, candidate.source_index, platform_altitude_m,
                                 propagation_loss_db, input, dwell.pointing_deg,
                                 rf_cycle, &measurement, &snr_db)) {
          continue;
        }
        // 规则 13b：检测通过后的识别链门控诊断（每目标至多一条，链上第一门优先；
        // 检测门失败已在 TryBuildMeasurement 内落诊断；斜距用裁剪期缓存）。
        if (database_ == nullptr) {
          last_execution_issues_.push_back(RirMakeExclusionIssue(
              session::codes::kTargetNoFeatureDatabase,
              "target_id=" + std::to_string(target.external_target_id) +
                  "; feature database unavailable, recognition held",
              session::RirIssueCause::kNone, candidate.source_index));
        } else if (slant_by_target_id.count(target.external_target_id) != 0U &&
                   slant_by_target_id[target.external_target_id] > mission_.max_range_m) {
          last_execution_issues_.push_back(RirMakeExclusionIssue(
              session::codes::kTargetBeyondRecognitionRange,
              "target_id=" + std::to_string(target.external_target_id) +
                  "; slant_range_m=" + RirFormatFloat(slant_by_target_id[target.external_target_id]) +
                  " beyond max_range_m=" + RirFormatFloat(mission_.max_range_m),
              session::RirIssueCause::kNone, candidate.source_index));
        }
        last_cycle_snr_by_target_id_[measurement.external_target_id] = snr_db;
        measurements.push_back(measurement);
      }
    }
    // 从未被任何驻留照到的候选：主瓣覆盖排除诊断（每目标每周期至多一条，规则 13b）。
    for (std::size_t k = 0U; k < candidates.size(); ++k) {
      if (candidate_covered[k] != 0U || candidates[k].degenerate != 0) {
        continue;
      }
      const session::RirSceneTarget& target = input.scene_targets[candidates[k].source_index];
      last_execution_issues_.push_back(RirMakeExclusionIssue(
          session::codes::kTargetOutsideBeamCoverage,
          "target_id=" + std::to_string(target.external_target_id) +
              "; look_az_deg=" + RirFormatFloat(candidates[k].look.az_deg) +
              " look_el_deg=" + RirFormatFloat(candidates[k].look.el_deg) +
              " outside main lobe of all dwells this cycle; half-power gate az_deg=" +
              RirFormatFloat(main_lobe_half_az_deg) +
              " el_deg=" + RirFormatFloat(main_lobe_half_el_deg),
          session::RirIssueCause::kNone, candidates[k].source_index));
    }

    const tracking::RirAssociationResult association = associator_.Associate(
        measurements, lifecycle_->BuildAssociationSeeds(), static_cast<float>(input.dt_sec));
    tracking::RirCycleContext cycle_context;
    cycle_context.cycle_index = input.input_cycle_index;
    cycle_context.batch_id = batch_id;
    cycle_context.dt_sec = static_cast<float>(input.dt_sec);
    lifecycle_->Update(cycle_context, association.measurements);

    // 验收事件 association（3.2.2.3.1.1）：全局最优关联结果——命中对（键/量测
    // 索引/马氏代价）与漏检航迹键（量测清单经 detection_cell 事件逐目标留痕）。
    if (RIR_ACCEPTANCE_LOG_ENABLED()) {
      WriteRirSearchDetections(EffectiveSensorEntityId(), input.sim_time_sec, input.input_cycle_index,
                               dwell_center_deg.az_deg, dwell_center_deg.el_deg,
                               g_acceptance_found_targets);
      const float gate_sigma = std::max(0.0f, policy_.association.distance_gate_sigma);
      WriteRirAssociation(input.sim_time_sec, input.input_cycle_index, association, platform_lla,
                          static_cast<double>(gate_sigma * gate_sigma));
      WriteRirExternalMeasurements(input.sim_time_sec, input.input_cycle_index, association);
    }

    track_snapshots = lifecycle_->BuildTrackSnapshots();

    // 实际有效目标最大斜距：本周期持航迹目标的最大输入几何斜距（给外部判断实际
    // 探测距离；仅统计本周期入检测候选且成航迹的目标，出扇区目标不计入）。
    for (const tracking::RirTrackState& track : track_snapshots) {
      const auto slant_found = slant_by_target_id.find(track.external_target_id);
      if (slant_found != slant_by_target_id.end() &&
          slant_found->second > last_max_detected_slant_range_m_) {
        last_max_detected_slant_range_m_ = slant_found->second;
      }
    }

    // 验收事件 cluster（2026-08-22 甲方批注）：集群目标数量 = 确认航迹数
    // （计数口径；非检测条数）。
    if (RIR_ACCEPTANCE_LOG_ENABLED()) {
      WriteRirClusterCount(input.sim_time_sec, input.input_cycle_index, track_snapshots);
    }

    std::unordered_map<std::uint64_t, recognition::RirTracker::TrackObservationInput>
        observations_by_key;
    for (const tracking::RirTrackState& track : track_snapshots) {
      const auto scene_found = scene_by_id.find(track.external_target_id);
      if (scene_found == scene_by_id.end() || scene_found->second == nullptr) {
        continue;
      }
      float snr_db = 0.0f;
      const auto snr_found = last_cycle_snr_by_target_id_.find(track.external_target_id);
      if (snr_found != last_cycle_snr_by_target_id_.end()) {
        snr_db = snr_found->second;
      }
      recognition::RirTracker::TrackObservationInput observation;
      observation.target = scene_found->second;
      observation.context =
          MakeObservationContext(*scene_found->second, platform_altitude_m, snr_db);
      observations_by_key[track.association_key] = observation;
    }

    if (database_ != nullptr) {
      // 出口①：采集本周期实际构建的有效特征观测（透出来源），组装公开量测记录。
      std::vector<recognition::RirTracker::CycleFeatureObservation> cycle_observations;
      tracker_.UpdateCycle(track_snapshots, observations_by_key, *database_,
                           policy_.recognition.feature_weights, sim_time_sec_,
                           input.input_cycle_index, batch_id, &cycle_observations);
      output_frame->feature_measurements.reserve(cycle_observations.size());
      for (const recognition::RirTracker::CycleFeatureObservation& observation :
           cycle_observations) {
        const auto context_found = observations_by_key.find(observation.association_key);
        if (context_found == observations_by_key.end()) {
          continue;  // UpdateCycle 只透出来自本表的键，此分支不可达（防御）。
        }
        const recognition::RirObservationContext& context = context_found->second.context;
        session::RirFeatureMeasurementRecord record;
        record.association_key = observation.association_key;
        record.features = ToPublicFeatureObservations(observation.features);
        record.valid_feature_mask = observation.features.valid_feature_mask;
        record.look_az_deg = context.look_az_deg;
        record.look_el_deg = context.look_el_deg;
        record.range_m = context.range_m;
        record.snr_db = context.snr_db;
        record.dwell_sec = context.dwell_sec;
        record.bandwidth_hz = context.bandwidth_hz;
        record.has_platform_position = true;
        record.platform_position = input.platform_position;
        record.cycle_index = input.input_cycle_index;
        record.batch_id = batch_id;
        output_frame->feature_measurements.push_back(record);
      }
    } else {
      tracker_.HoldCycle(track_snapshots, sim_time_sec_);
    }

    latest_summary_ = tracker_.BuildSummary(track_snapshots);
    latest_summary_.dwell_budget.scheduled_dwell_count = scheduled_count;
    latest_summary_.dwell_budget.executed_dwell_count = executed_count;
    latest_summary_.dwell_budget.dwell_budget_sec = static_cast<float>(scheduled_count) * dwell_sec;
    latest_summary_.dwell_budget.dwell_consumed_sec =
        static_cast<float>(executed_count) * dwell_sec;
    has_latest_summary_ = true;

    // 验收事件 schedule（3.2.2.4.2.2）：驻留调度摘要（按搜索/指定/跟踪分项计数 +
    // 时间预算；2026-08-30 核查 9.1：计划/实际驻留数恒等，验收行不再输出该组数字，
    // 计数仍写入 dwell_budget 摘要与 replay 记录）。
    if (RIR_ACCEPTANCE_LOG_ENABLED()) {
      std::uint32_t confirmed = 0U;
      for (const tracking::RirTrackState& track : track_snapshots) {
        if (track.status == tracking::RirTrackStatus::kConfirmed) {
          ++confirmed;
        }
      }
      std::uint32_t planned_search = 0U;
      std::uint32_t planned_designate = 0U;
      std::uint32_t planned_track = 0U;
      for (const RirDwellPlan& dwell : plan) {
        if (dwell.kind == RirDwellKind::kSearch) {
          ++planned_search;
        } else if (dwell.kind == RirDwellKind::kDesignate) {
          ++planned_designate;
        } else {
          ++planned_track;
        }
      }
      WriteRirSchedule(EffectiveSensorEntityId(), input.sim_time_sec, input.input_cycle_index,
                       latest_summary_.dwell_budget.dwell_budget_sec,
                       latest_summary_.dwell_budget.dwell_consumed_sec, planned_search,
                       planned_designate, planned_track, confirmed);
    }
  } else {
    track_snapshots = lifecycle_->BuildTrackSnapshots();
    tracker_.HoldCycle(track_snapshots, sim_time_sec_);
    latest_summary_ = tracker_.BuildSummary(track_snapshots);
    has_latest_summary_ = true;
    // 规则 13b：非识别工作模式 → 全局模式门，逐目标落 kInfo 诊断（不建识别观测，
    // 检测/跟踪不受影响的 STBY 保持周期）。
    for (std::size_t i = 0U; i < input.scene_targets.size(); ++i) {
      if (input.scene_targets[i].external_target_id == 0U) {
        continue;
      }
      last_execution_issues_.push_back(RirMakeExclusionIssue(
          session::codes::kTargetModeNotIdentify,
          "target_id=" + std::to_string(input.scene_targets[i].external_target_id) +
              "; work_mode=stby, no recognition observation this cycle",
          session::RirIssueCause::kNone, i));
    }
  }

  last_track_snapshots_ = track_snapshots;
  output_frame->recognition_outputs.clear();
  output_frame->recognition_outputs.reserve(track_snapshots.size());
  last_track_attributions_.reserve(track_snapshots.size());
  for (const tracking::RirTrackState& track : track_snapshots) {
    session::RirTrackRecognitionOutput output;
    output.association_key = track.association_key;
    const session::RirRecognitionResult* result = tracker_.FindResult(track.association_key);
    if (result != nullptr) {
      output.result = *result;
    }
    output_frame->recognition_outputs.push_back(output);
    if (RIR_ACCEPTANCE_LOG_ENABLED()) {
      const session::RirFeatureMeasurementRecord* features = nullptr;
      for (const session::RirFeatureMeasurementRecord& record : output_frame->feature_measurements) {
        if (record.association_key == track.association_key) {
          features = &record;
          break;
        }
      }
      std::vector<float> imm_weights;
      if (lifecycle_ != nullptr) {
        const tracking::RirImmFilter* imm = lifecycle_->FindImmFilter(track.association_key);
        if (imm != nullptr && imm->IsValid()) {
          const Eigen::VectorXf weights = imm->GetModelWeights();
          imm_weights.reserve(static_cast<std::size_t>(weights.size()));
          for (Eigen::Index i = 0; i < weights.size(); ++i) {
            imm_weights.push_back(weights(i));
          }
        }
      }
      // 真值上下文（验收判定标准 第38/42/47项）：视线极坐标 + ECEF 位置/速度 +
      // 距离像散射中心（场景输入）。
      runtime::RirTrackTruthContext truth_context;
      const session::RirSceneTarget* truth_target = nullptr;
      for (const session::RirSceneTarget& scene_target : input.scene_targets) {
        if (scene_target.external_target_id == track.external_target_id) {
          truth_target = &scene_target;
          break;
        }
      }
      if (truth_target != nullptr) {
        float truth_range_m = 0.0f;
        float truth_az_deg = 0.0f;
        float truth_el_deg = 0.0f;
        ComputeLookAngles(*truth_target, &truth_az_deg, &truth_el_deg, &truth_range_m);
        truth_context.has_look = true;
        truth_context.truth_range_m = truth_range_m;
        truth_context.truth_az_deg = truth_az_deg;
        truth_context.truth_el_deg = truth_el_deg;
        if (has_platform_lla) {
          const Eigen::Vector3f truth_pos_enu = PositionOf(*truth_target);
          const oneq::coordinate::EnuPositionM truth_enu(truth_pos_enu.x(), truth_pos_enu.y(),
                                                         truth_pos_enu.z());
          const oneq::coordinate::EnuVelocityMps truth_vel_enu(
              truth_target->velocity_x, truth_target->velocity_y, truth_target->velocity_z);
          if (oneq::coordinate::TryEnuToEcef(truth_enu, platform_lla,
                                             &truth_context.position_ecef) &&
              oneq::coordinate::TryEnuToEcefVelocity(truth_vel_enu, platform_lla,
                                                     &truth_context.velocity_ecef)) {
            truth_context.has_ecef = true;
          }
        }
        truth_context.scatterers = &truth_target->range_rcs_scatterers;
      }
      WriteRirTrackAndId(input.sim_time_sec, input.input_cycle_index, track, result, features,
                         latest_summary_.has_ground_truth,
                         static_cast<double>(latest_summary_.category_accuracy), &imm_weights,
                         input.platform_position,
                         truth_target != nullptr ? &truth_context : nullptr);
    }
    // 归属视图与出口②同循环产出（全部航迹快照：tentative/confirmed/lost）。
    session::RirTrackAttributionRecord attribution;
    attribution.association_key = track.association_key;
    attribution.external_target_id = track.external_target_id;
    attribution.target_name = track.target_name;
    attribution.track_status = ToPublicTrackStatus(track.status);
    attribution.hit_count = track.hit_count;
    attribution.position_enu_x_m = static_cast<double>(track.position.x());
    attribution.position_enu_y_m = static_cast<double>(track.position.y());
    attribution.position_enu_z_m = static_cast<double>(track.position.z());
    attribution.speed_m_per_s = static_cast<double>(track.speed);
    const auto cycle_snr_found = last_cycle_snr_by_target_id_.find(track.external_target_id);
    if (cycle_snr_found != last_cycle_snr_by_target_id_.end()) {
      attribution.has_cycle_detection_snr = true;
      attribution.cycle_detection_snr_db = cycle_snr_found->second;
    }
    last_track_attributions_.push_back(attribution);
  }
  if (RIR_ACCEPTANCE_LOG_ENABLED()) {
    WriteRirExternalTracks(input.sim_time_sec, input.input_cycle_index, track_snapshots,
                           *output_frame);
    // 验收判定标准 第40项：本周期航迹数汇总行（逐航迹定位/运动行在
    // WriteRirTrackAndId 内按项名写出）。
    RIR_ACCEPTANCE_ITEM(input.sim_time_sec, input.input_cycle_index, "多目标跟踪功能测试",
                        "本周期航迹数=" + std::to_string(track_snapshots.size()));
  }

  // 中译：RIR 周期处理完成（周期号、设备 ID、航迹数）。
  // 标识：三写之三（人读日志）——识别周期执行完毕摘要。
  PROJECT_LOG_INFO(
      "[RirPipeline] cycle_index={} sensor_entity_id={} tracks={}",
      input.input_cycle_index, EffectiveSensorEntityId(),
      track_snapshots.size());
}

bool RirController::IsTargetRecognized(std::uint64_t external_target_id) const {
  if (external_target_id == 0U) {
    return false;
  }
  for (const tracking::RirTrackState& track : last_track_snapshots_) {
    if (track.external_target_id != external_target_id) {
      continue;
    }
    const session::RirRecognitionResult* result = tracker_.FindResult(track.association_key);
    if (result != nullptr &&
        (result->state == session::RirRecognitionState::kCategoryConfirmed ||
         result->state == session::RirRecognitionState::kModelConfirmed)) {
      return true;
    }
  }
  return false;
}

}  // namespace runtime
}  // namespace remote_identification_radar
