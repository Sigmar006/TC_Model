#include "airborne_radar/signal/pipeline/DetectionExecution.h"
#include "common/platform/OneqPlatformCompat.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "1q/airborne_radar/session/ArIssueCodes.h"
#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/signal/detection/ArDetectionCellResolver.h"
#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/detection/MeasurementErrorModel.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/pipeline/PipelineTargetUtils.h"
#include "common/atmosphere/AtmospherePhysics.h"
#include "common/logging/ProjectLog.h"
#include "common/radar/VegetationClutterModel.h"
#include "common/rcs/RcsPhysics.h"
#include "common/numerics/Constants.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

float ComputeEquivalentClutterNoiseW(const config::engineering::DetectionConfig& detection_config,
                                     float clutter_power_db) {
  const float thermal_noise_w = detection::RadarEquations::ComputeThermalNoisePower_W(
      detection_config.transmitter, detection_config.receiver);
  return oneq::common::radar::ComputeEquivalentClutterNoiseW(thermal_noise_w, clutter_power_db);
}

float ComputeTargetSpecificAtmosphericLossDb(
    const ExecutionConfig& exec_config, const session::EnvironmentSnapshot& environment_snapshot,
    float platform_altitude_m, const detection::ResolvedTargetGeometry& geometry) {
  if (!environment_snapshot.atmospheric_physics.enable_physical_model) {
    return 0.0f;
  }

  oneq::common::atmosphere::AtmosphericObservationRef obs;
  obs.pressure_hpa = environment_snapshot.atmospheric_physics.pressure_hpa;
  obs.temperature_k = environment_snapshot.atmospheric_physics.temperature_k;
  obs.relative_humidity = environment_snapshot.atmospheric_physics.relative_humidity;
  obs.k_factor = environment_snapshot.effective_k_factor;
  return oneq::common::atmosphere::ComputeTargetAtmosphericPhysicsLossDb(
      exec_config.detection.engineering.transmitter.frequency_hz, geometry.range_m,
      platform_altitude_m, std::max(platform_altitude_m + geometry.position_m.z(), 0.0f),
      geometry.look_angles_deg.look_el_deg, geometry.look_angles_deg.has_look_angles, obs);
}

float ComputeEffectiveTargetRcsM2(const session::ArSceneTarget& target,
                                  const detection::ResolvedTargetGeometry& geometry,
                                  const ExecutionConfig& exec_config, float carrier_hz) {
  const float input_rcs_m2 = std::max(target.rcs, 0.0f);
  const config::engineering::RcsPhysicsConfig& rcs_config =
      exec_config.detection.engineering.rcs_physics;
  // carrier_hz 非正（v1 无冻结波形事实）时回退静态 transmitter.frequency_hz（模块侧策略）。
  const float frequency_hz = carrier_hz > 0.0f
                                 ? carrier_hz
                                 : exec_config.detection.engineering.transmitter.frequency_hz;

  oneq::common::rcs::RcsPhysicsParams params;
  params.enable_physical_rcs = rcs_config.enable_physical_rcs;
  params.physics_mix_ratio = rcs_config.physics_mix_ratio;
  params.cylinder_weight = rcs_config.cylinder_weight;
  params.min_equivalent_radius_m = rcs_config.min_equivalent_radius_m;
  params.max_equivalent_radius_m = rcs_config.max_equivalent_radius_m;
  params.min_rcs_m2 = rcs_config.min_rcs_m2;
  params.max_rcs_m2 = rcs_config.max_rcs_m2;
  params.bistatic_psi_offset_deg = rcs_config.bistatic_psi_offset_deg;

  return oneq::common::rcs::ComputeMixedPhysicalRcsM2(
      input_rcs_m2, frequency_hz, geometry.look_angles_deg.look_az_deg,
      geometry.look_angles_deg.look_el_deg, geometry.look_angles_deg.has_look_angles, params);
}

tracking::MeasurementCovariance BuildMeasurementCovariance(
    const detection::ResolvedTargetGeometry& geometry, float range_error_std, float angle_error_std,
    float default_measurement_noise_std) {
  if (range_error_std <= 0.0f || angle_error_std <= 0.0f) {
    return tracking::MeasurementCovariance::Identity() * default_measurement_noise_std *
           default_measurement_noise_std;
  }

  const float range_m = std::max(geometry.range_m, 0.1f);
  const float var_r = range_error_std * range_error_std;
  const float var_theta = angle_error_std * angle_error_std;
  Eigen::Vector3f pos = geometry.position_m;
  const float pos_norm = pos.norm();
  if (pos_norm > 0.1f) {
    const Eigen::Vector3f u = pos / pos_norm;
    const Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
    const Eigen::Matrix3f uu_t = u * u.transpose();
    return var_r * uu_t + (range_m * range_m * var_theta) * (identity - uu_t);
  }

  return tracking::MeasurementCovariance::Identity() * var_r;
}

bool HasValidBuffers(const DetectionExecutionBuffers& buffers) {
  return buffers.target_geometry != nullptr && buffers.signal_term_db != nullptr &&
         buffers.speed_penalty_db != nullptr && buffers.detection_margin_db != nullptr &&
         buffers.detection_succeeded != nullptr && buffers.measurement_covariances != nullptr &&
         buffers.issues != nullptr && buffers.excluded_snr_below != nullptr;
}

// 规则 13b：正常执行周期按目标门控排除的 kInfo 诊断码（不属于三写，仅承载排查信息）。
// code 引用 ArIssueCodes.h 注册表常量。

/// 构造 kInfo 级按目标排除诊断（不属于三写，仅承载排查信息；规则 13b）。
/// @param cause 门内归因（规则 13b 门内归因条款）；聚合门排除须给出主因，具体门可 kNone。
/// @param target_index 场景目标索引；非负时写入 `location = {kSceneEntity, target_index}`，
///                     供跨周期差分记录器按实体关联消费（默认 -1 保持 kGlobal，向后兼容）。
session::ArIssue MakeExclusionIssue(const char* code, const std::string& message,
                                    session::ArIssueCause cause = session::ArIssueCause::kNone,
                                    std::ptrdiff_t target_index = -1) {
  session::ArIssue issue;
  issue.severity = session::ArIssueSeverity::kInfo;
  issue.phase = session::ArIssuePhase::kExecution;
  issue.code = code;
  issue.message = message;
  issue.cause = cause;
  if (target_index >= 0) {
    issue.location.kind = oneq::foundation::ValidationLocationKind::kSceneEntity;
    issue.location.entity_index = static_cast<std::size_t>(target_index);
  }
  return issue;
}

/// 格式化量值为两位小数（消息文本稳定，不承诺解析稳定性；规则 13b message 约定）。
std::string FormatFloat(float value) {
  char buffer[32];
  ONEQ_SNPRINTF(buffer, sizeof(buffer), "%.2f", static_cast<double>(value));
  return buffer;
}

}  // namespace

// 规则 13b 门内归因：SNR 检测门（min_snr_db/min_detection_margin_db）折入距离/波束/
// 噪声底/RCS 多种物理因素。按各因素相对参考状态的损失 dB 判定主因（损失最大者）：
// 参考状态 = 1 km 距离、主瓣中心增益、1 m² RCS、热噪声底、零传播损耗。
// 传播损耗并入距离项（大气损耗随距离/仰角耦合，同属链路衰减）。
// 声明见 DetectionExecution.h（供单元测试直接覆盖五类输出）。
session::ArIssueCause ClassifySnrExclusionCause(float range_m, float effective_rcs_m2,
                                                float one_way_antenna_gain_db,
                                                float main_beam_gain_db,
                                                float propagation_loss_db, float total_noise_w,
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
  session::ArIssueCause cause = session::ArIssueCause::kUnknown;
  const struct {
    float loss_db;
    session::ArIssueCause cause;
  } kCandidates[] = {
      {distance_loss_db, session::ArIssueCause::kDistanceLimited},
      {beam_loss_db, session::ArIssueCause::kBeamLimited},
      {noise_loss_db, session::ArIssueCause::kNoiseLimited},
      {rcs_loss_db, session::ArIssueCause::kRcsLimited},
  };
  for (const auto& candidate : kCandidates) {
    if (candidate.loss_db > max_loss_db) {
      max_loss_db = candidate.loss_db;
      cause = candidate.cause;
    }
  }
  return max_loss_db > 0.0f ? cause : session::ArIssueCause::kUnknown;
}

bool RunPhysicalDetectionPass(const session::ArSceneTargetList& input,
                              const ExecutionConfig& config,
                              const session::EnvironmentSnapshot& environment_snapshot,
                              float platform_altitude_m,
                              const RfV2DetectionContext* rf_v2_detection_context,
                              detection::SignalDetector* signal_detector,
                              DetectionExecutionBuffers* buffers) {
  if (buffers == nullptr || !HasValidBuffers(*buffers)) {
    return false;
  }
  if (signal_detector == nullptr) {
    return true;
  }

  const std::size_t count = input.size();

  float clutter_w = ComputeEquivalentClutterNoiseW(config.detection.engineering,
                                                   environment_snapshot.clutter_power_db);
  // 热噪声底（门内归因判定基准，规则 13b：噪声项损失 = 噪声底相对热噪声的抬高量）。
  const float thermal_noise_w = detection::RadarEquations::ComputeThermalNoisePower_W(
      config.detection.engineering.transmitter, config.detection.engineering.receiver);
  detection::EnvironmentState env;
  env.propagation_loss_db = environment_snapshot.propagation_loss_db;
  env.clutter_noise_w = clutter_w;
  env.jam_noise_w = 0.0f;  // RF v2 interference is resolved in the detection-cell path below.

  signal_detector->UpdateConfig(config.detection.engineering);

  constexpr float kSpeedOfLightMps = static_cast<float>(oneq::common::numerics::kLightSpeed);
  const float wavelength_m =
      config.detection.engineering.transmitter.frequency_hz > 0.0f
          ? kSpeedOfLightMps / config.detection.engineering.transmitter.frequency_hz
          : 0.0f;

  for (std::size_t i = 0; i < count; ++i) {
    (*buffers->target_geometry)[i] = detection::TargetGeometryResolver::Resolve(input[i]);
    env.propagation_loss_db =
        environment_snapshot.propagation_loss_db +
        ComputeTargetSpecificAtmosphericLossDb(config, environment_snapshot, platform_altitude_m,
                                               (*buffers->target_geometry)[i]);
    // 综合噪声底（门内归因判定输入）：RF v1 = 热噪声 + 杂波；RF v2 由检测单元给出
    // 热噪声/杂波/干扰分解。
    float noise_floor_w = thermal_noise_w + clutter_w;
    // 与 RIR 同口径：物理 RCS 的 k0 取本周期实际发射载频（冻结波形的中心频率，
    // 频率捷变时逐周期跳变）；v1 无冻结波形事实时回退静态 transmitter.frequency_hz。
    const float rcs_carrier_hz =
        rf_v2_detection_context != nullptr
            ? static_cast<float>(
                  rf_v2_detection_context->own_transmit_waveform.center_frequency_hz)
            : 0.0f;
    const float effective_rcs_m2 =
        ComputeEffectiveTargetRcsM2(input[i], (*buffers->target_geometry)[i], config,
                                    rcs_carrier_hz);
    detection::TargetReturn target;
    target.rcs_m2 = effective_rcs_m2;
    target.range_m = (*buffers->target_geometry)[i].range_m;
    if (input[i].target_swerling_type >=
            static_cast<int>(config::profiles::SwerlingModel::kSwerling0) &&
        input[i].target_swerling_type <=
            static_cast<int>(config::profiles::SwerlingModel::kSwerling4)) {
      target.swerling_type =
          static_cast<config::profiles::SwerlingModel>(input[i].target_swerling_type);
    } else {
      target.swerling_type = config::profiles::SwerlingModel::kSwerling0;
    }

    const detection::ResolvedBeamState beam_state =
        rf_v2_detection_context == nullptr
            ? detection::BeamControlResolver::Resolve(
                  config.detection.engineering.antenna, config.detection.orientation,
                  config.detection.platform_attitude_deg,
                  (*buffers->target_geometry)[i].look_angles_deg,
                  config::AzimuthElevationDeg{}, wavelength_m)
            : detection::BeamControlResolver::ResolveFrozen(
                  config.detection.engineering.antenna, config.detection.orientation,
                  (*buffers->target_geometry)[i].look_angles_deg,
                  rf_v2_detection_context->beam_pointing_deg, wavelength_m);
    detection::DetectionResult detection_result;
    if (rf_v2_detection_context == nullptr) {
      detection_result = signal_detector->Detect(target, env, beam_state.one_way_antenna_gain_db,
                                                 config.detection.engineering.pulse_count);
    } else {
      const Eigen::Vector3f& position = (*buffers->target_geometry)[i].position_m;
      const float position_norm = position.norm();
      if (!std::isfinite(position_norm) || position_norm <= 0.0f) {
        // 中译：目标 {} 的本地位置范数非法（{}）。
        // 标识：数值保护——位置非有限或非正时该目标检测失败，周期中止。
        PROJECT_LOG_ERROR("[DetectionExecution] target {} has invalid local position norm {}.", i,
                          position_norm);
        return false;
      }
      const Eigen::Vector3f relative_velocity(input[i].velocity_x, input[i].velocity_y,
                                              input[i].velocity_z);
      const double closing_radial_velocity_mps =
          -static_cast<double>(relative_velocity.dot(position / position_norm));
      const double two_way_propagation_loss_db = static_cast<double>(env.propagation_loss_db);
      if (!std::isfinite(two_way_propagation_loss_db) || two_way_propagation_loss_db < 0.0) {
        // 中译：目标 {} 的双向传播损耗非法（{} dB）。
        // 标识：数值保护——损耗非有限或为负时该目标检测失败，周期中止。
        PROJECT_LOG_ERROR(
            "[DetectionExecution] target {} has invalid two-way propagation loss {} dB.", i,
            two_way_propagation_loss_db);
        return false;
      }
      detection::ArDetectionCellConfig cell_config;
      cell_config.own_transmit_waveform = rf_v2_detection_context->own_transmit_waveform;
      cell_config.receive_window_start_time_s =
          rf_v2_detection_context->receive_window_start_time_s;
      cell_config.receive_window_duration_s =
          rf_v2_detection_context->receive_window_duration_s;
      cell_config.matched_filter_bandwidth_hz =
          rf_v2_detection_context->own_transmit_waveform.occupied_bandwidth_hz;
      cell_config.one_way_antenna_gain_dbi =
          static_cast<double>(beam_state.one_way_antenna_gain_db);
      cell_config.receiver_loss_db =
          static_cast<double>(config.detection.engineering.receiver.receive_loss_db);
      cell_config.receiver_noise_figure_db =
          static_cast<double>(config.detection.engineering.receiver.noise_figure_db);
      cell_config.signal_processing = config.detection.engineering.signal_processing;
      cell_config.enable_anti_rgpo_leading_edge =
          rf_v2_detection_context->enable_anti_rgpo_leading_edge;
      detection::ArDetectionCellTarget cell_target;
      cell_target.range_m = static_cast<double>((*buffers->target_geometry)[i].range_m);
      cell_target.closing_radial_velocity_mps = closing_radial_velocity_mps;
      cell_target.rcs_m2 = static_cast<double>(effective_rcs_m2);
      cell_target.two_way_additional_propagation_loss_db = two_way_propagation_loss_db;
      cell_target.effective_pulse_count = static_cast<std::uint32_t>(
          std::max(1, config.detection.engineering.pulse_count));
      detection::ArDetectionCellResult cell_result;
      if (!detection::TryResolveArDetectionCell(
              cell_config, cell_target, rf_v2_detection_context->own_emission_identity,
              rf_v2_detection_context->incident_links, static_cast<double>(clutter_w),
              &cell_result)) {
        // 中译：目标 {} 的 RF 检测单元解析失败（距离/脉冲数/入射链路数）。
        // 标识：检测链路失败——信噪比/杂波竞争解析异常时该目标检测失败。
        PROJECT_LOG_ERROR(
            "[DetectionExecution] target {} RF detection-cell resolution failed "
            "(range_m={}, pulse_count={}, incident_links={}).",
            i, cell_target.range_m, cell_target.effective_pulse_count,
            rf_v2_detection_context->incident_links.size());
        return false;
      }
      // 综合噪声底 = 热噪声 + 杂波 + 干扰（RF v2 检测单元分解）。
      noise_floor_w = static_cast<float>(cell_result.thermal_noise_power_w +
                                         cell_result.clutter_power_w +
                                         cell_result.interference_power_w);
      detection_result = signal_detector->DetectResolvedCell(target, cell_result);
    }
    const detection::MeasurementErrorState measurement_error =
        detection::MeasurementErrorModel::Compute(
            detection_result.snr_db, beam_state.effective_beamwidth_deg,
            config.detection.engineering.transmitter.bandwidth_hz);

    const bool detected = detection_result.detected;
    (*buffers->signal_term_db)[i] = detection_result.snr_db;
    (*buffers->speed_penalty_db)[i] = 0.0f;
    (*buffers->detection_margin_db)[i] =
        detection_result.snr_db - config.detection.engineering.min_detection_margin_db;
    (*buffers->detection_succeeded)[i] = static_cast<std::uint8_t>(detected ? 1U : 0U);
    if (!detected) {
      // 规则 13b：SNR/检测门排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
      // 门内归因（规则 13b 门内归因条款）：SNR 门为聚合门（距离/波束/噪声底/RCS
      // 折入单一门限），按各因素相对参考状态的损失 dB 判定主因。
      const session::ArIssueCause cause = ClassifySnrExclusionCause(
          (*buffers->target_geometry)[i].range_m, effective_rcs_m2,
          beam_state.one_way_antenna_gain_db,
          config.detection.engineering.antenna.main_beam_gain_db, env.propagation_loss_db,
          noise_floor_w, thermal_noise_w);
      // 中译：目标 {} 因 SNR 低于检测门限被排除（主因 {}）。
      // 标识：规则 13b 排除诊断——周期正常完成，目标未过 SNR 门；cause 为机器可读
      //       主因（距离/波束/噪声底/RCS），仅承载排查信息，不用于状态判断。
      std::string off_axis_text = " off_axis_deg=(n/a)";
      if ((*buffers->target_geometry)[i].look_angles_deg.has_look_angles) {
        off_axis_text = " off_axis_deg=(" +
                        FormatFloat((*buffers->target_geometry)[i].look_angles_deg.look_az_deg -
                                    beam_state.beam_pointing_deg.az_deg) +
                        "," +
                        FormatFloat((*buffers->target_geometry)[i].look_angles_deg.look_el_deg -
                                    beam_state.beam_pointing_deg.el_deg) +
                        ")";
      }
      buffers->issues->push_back(MakeExclusionIssue(
          session::codes::kTargetSnrBelowThreshold,
          "target_id=" + std::to_string(input[i].external_target_id) + "; snr_db=" +
              FormatFloat(detection_result.snr_db) + " range_m=" +
              FormatFloat((*buffers->target_geometry)[i].range_m) + " below min_snr_db=" +
              FormatFloat(config.detection.engineering.detection_policy.min_snr_db) +
              " min_detection_margin_db=" +
              FormatFloat(config.detection.engineering.min_detection_margin_db) +
              off_axis_text,
          cause,
          static_cast<std::ptrdiff_t>(i)));
      ++(*buffers->excluded_snr_below);
    }
    (*buffers->measurement_covariances)[i] = BuildMeasurementCovariance(
        (*buffers->target_geometry)[i], measurement_error.range_error_std_m,
        measurement_error.angle_error_std_rad,
        config.tracking.engineering.kalman_measurement_noise_std);
  }
  return true;
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
