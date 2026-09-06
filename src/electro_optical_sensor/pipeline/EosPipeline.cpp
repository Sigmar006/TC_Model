#include "electro_optical_sensor/pipeline/EosPipeline.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "1q/electro_optical_sensor/session/EosIssueCodes.h"
#include "common/logging/ProjectLog.h"
#include "common/numerics/ClampUtils.h"
#include "electro_optical_sensor/environment/EosEnvironmentModel.h"
#include "electro_optical_sensor/foundation/EosNoiseModel.h"
#include "electro_optical_sensor/foundation/EosOpticalCharacteristics.h"
#include "electro_optical_sensor/foundation/EosPhysicalConstants.h"
#include "electro_optical_sensor/foundation/EosPropagation.h"
#include "electro_optical_sensor/foundation/EosRadiativeTransfer.h"
#include "electro_optical_sensor/foundation/EosRadiometry.h"
#include "electro_optical_sensor/foundation/EosSpatialSpectrum.h"
#include "electro_optical_sensor/foundation/EosStrayLight.h"

namespace electro_optical_sensor {
namespace signal {
namespace pipeline {

struct FrameContext {
  bool infrared_enabled{false};
  bool visible_enabled{false};
  float aperture_area_m2{0.0f};
  float fov_solid_angle_sr{0.0f};
  float wavelength_center_um{4.0f};
  float wavelength_bandwidth_um{1.0f};
  float background_spectral_radiance_w_sr_m3{0.0f};
  float visible_photon_noise_enhancement{1.0f};
  float optical_transmittance{0.85f};
  float diffraction_resolution_rad{0.0f};
  float dmin_m{0.0f};
  float dmax_m{0.0f};
  foundation::propagation::NepNoiseModelInputs nep_inputs{};
  foundation::noise::BackgroundNoiseModelInputs noise_inputs_base{};
  environment::EnvironmentModelResult environment_result{};
};

namespace {

bool WorkModeIncludesInfrared(EosPipelineWorkMode mode) {
  return mode == EosPipelineWorkMode::kInfraredOnly || mode == EosPipelineWorkMode::kFused;
}

bool WorkModeIncludesVisible(EosPipelineWorkMode mode) {
  return mode == EosPipelineWorkMode::kVisibleOnly || mode == EosPipelineWorkMode::kFused;
}

foundation::radiometry::IlluminationCondition ToIlluminationCondition(
    ::electro_optical_sensor::config::DayNightType day_night_type) {
  if (day_night_type == ::electro_optical_sensor::config::DayNightType::kNight) {
    return foundation::radiometry::IlluminationCondition::kNight;
  }
  if (day_night_type == ::electro_optical_sensor::config::DayNightType::kTwilight) {
    return foundation::radiometry::IlluminationCondition::kTwilight;
  }
  return foundation::radiometry::IlluminationCondition::kDay;
}

float ComputeApertureAreaM2(float optical_aperture_m) {
  const float safe_diameter_m = oneq::common::numerics::SafePositive(optical_aperture_m, 0.2f);
  const float radius_m = 0.5f * safe_diameter_m;
  return foundation::constants::kPi * radius_m * radius_m;
}

float ResolvePlatformAltitudeM(const EosPipelineCycleInput& input) {
  return std::max(0.0f, input.platform_altitude_m);
}

// 规则 13b：正常执行周期按目标门控排除的 kInfo 诊断码（不属于三写，仅承载排查信息）。
// code 引用 EosIssueCodes.h 注册表常量。

/// 构造 kInfo 级按目标排除诊断（不属于三写，仅承载排查信息；规则 13b）。
/// @param cause 门内归因（规则 13b 门内归因条款）；聚合门排除须给出主因，具体门可 kNone。
/// @param target_index 场景目标索引；非负时写入 `location = {kSceneEntity, target_index}`，
///                     供跨周期差分记录器按实体关联消费（默认 -1 保持 kGlobal，向后兼容）。
session::EosIssue MakeExclusionIssue(const char* code, const std::string& message,
                                     session::EosIssueCause cause = session::EosIssueCause::kNone,
                                     std::ptrdiff_t target_index = -1) {
  session::EosIssue issue;
  issue.severity = session::EosIssueSeverity::kInfo;
  issue.code = code;
  issue.message = message;
  issue.cause = cause;
  if (target_index >= 0) {
    issue.location.kind = oneq::foundation::ValidationLocationKind::kSceneEntity;
    issue.location.entity_index = static_cast<std::size_t>(target_index);
  }
  return issue;
}

/// 格式化量值为一位小数（消息文本稳定，不承诺解析稳定性；规则 13b message 约定）。
std::string FormatFloat(float value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(value));
  return buffer;
}

foundation::radiative_transfer::RadiativeTransferResult ComputePathRadiativeTransfer(
    const config::execution::EosInternalExecutionConfig& config,
    const EosPipelineCycleInput& input, float range_m,
    const environment::EnvironmentModelResult& environment_result) {
  const float cloud_ratio =
      oneq::common::numerics::Clamp01(config.environment.cloud_coverage_ratio);
  const float aerosol_excess = std::max(0.0f, environment_result.aerosol_density_factor - 1.0f);
  const float turbulence_excess = std::max(0.0f, environment_result.turbulence_factor - 1.0f);
  const float path_km = std::max(0.0f, range_m) * 1.0e-3f;
  const float altitude_km = ResolvePlatformAltitudeM(input) * 1.0e-3f;
  const float attenuation_per_km =
      0.03f * environment_result.molecular_density_factor + 0.02f * cloud_ratio +
      0.035f * aerosol_excess + 0.015f * turbulence_excess;
  const float altitude_relief_scale =
      oneq::common::numerics::Clamp(1.0f + 0.04f * altitude_km, 1.0f, 1.12f);
  const float derived_base_transmittance = oneq::common::numerics::Clamp(
      std::exp(-attenuation_per_km * path_km) * altitude_relief_scale, 0.05f, 0.98f);

  foundation::radiative_transfer::RadiativeTransferInputs transfer_inputs;
  transfer_inputs.model = config.environment.radiative_transfer_model;
  transfer_inputs.base_transmittance = derived_base_transmittance;
  transfer_inputs.cloud_coverage_ratio =
      oneq::common::numerics::Clamp01(config.environment.cloud_coverage_ratio);
  transfer_inputs.path_length_m = std::max(0.0f, range_m);
  transfer_inputs.aerosol_density_factor =
      std::max(1.0f, environment_result.aerosol_density_factor);
  transfer_inputs.turbulence_factor = std::max(1.0f, environment_result.turbulence_factor);
  return foundation::radiative_transfer::EvaluateRadiativeTransfer(transfer_inputs);
}

float ComputeFovSolidAngleSr(float horizontal_fov_deg, float vertical_fov_deg) {
  const float horizontal_fov_rad =
      std::max(0.0f, horizontal_fov_deg) * foundation::constants::kPi / 180.0f;
  const float vertical_fov_rad =
      std::max(0.0f, vertical_fov_deg) * foundation::constants::kPi / 180.0f;
  return std::max(0.0f, horizontal_fov_rad * vertical_fov_rad);
}

float ComputeSensorIntegrationTimeSec(
    const config::execution::EosInternalExecutionConfig& config,
    const EosPipelineCycleInput& input) {
  const float frame_period_sec =
      1.0f /
      std::max(oneq::common::numerics::SafePositive(config.scan.frame_rate_hz, 30.0f), 1.0f);
  const float cycle_dt_sec = oneq::common::numerics::SafePositive(input.dt_sec, frame_period_sec);
  const float base_integration_sec = std::min(cycle_dt_sec, frame_period_sec);
  const float scan_blur_penalty =
      1.0f +
      oneq::common::numerics::SafePositive(config.scan.scan_rate_deg_per_sec, 20.0f) / 120.0f;
  return std::max(1.0e-4f, base_integration_sec / scan_blur_penalty);
}

float ComputeVisiblePhotonNoiseEnhancement(
    const config::execution::EosInternalExecutionConfig& config,
    const EosPipelineCycleInput& input) {
  const float reference_irradiance = oneq::common::numerics::SafePositive(
      config.detection.visible_reference_irradiance_w_m2, 800.0f);
  const float observed_irradiance = std::max(0.0f, config.environment.solar_irradiance_w_m2);
  const float irradiance_ratio = std::max(observed_irradiance, 1.0e-3f) / reference_irradiance;
  const float irradiance_mismatch = std::fabs(std::log2(std::max(irradiance_ratio, 1.0e-6f)));
  const float cloud_factor =
      1.0f + 0.5f * oneq::common::numerics::Clamp01(config.environment.cloud_coverage_ratio);
  return oneq::common::numerics::Clamp(1.0f + 0.12f * irradiance_mismatch * cloud_factor, 1.0f,
                                         2.5f);
}

float ComputeSystemNoiseFactorFromSensitivity(float detection_sensitivity_w) {
  const float kReferenceSensitivityW = 1.0e-12f;
  const float safe_sensitivity_w =
      oneq::common::numerics::SafePositive(detection_sensitivity_w, kReferenceSensitivityW);
  return std::max(1.0f, safe_sensitivity_w / kReferenceSensitivityW);
}

struct DetectionComputationContext {
  bool infrared_enabled{false};
  bool visible_enabled{false};
  bool is_within_detection_range{false};
  float aperture_area_m2{0.0f};
  float path_transmittance{0.0f};
  float path_radiance_penalty_scale{1.0f};
  float optical_transmittance{0.85f};
  float fov_solid_angle_sr{0.0f};
  float wavelength_center_um{4.0f};
  float wavelength_bandwidth_um{1.0f};
  float imaging_quality_gain{1.0f};
  float background_spectral_radiance_w_sr_m3{0.0f};
  float visible_photon_noise_enhancement{1.0f};
  float solar_irradiance_w_m2{800.0f};
  float solar_altitude_deg{45.0f};
  float cloud_coverage_ratio{0.2f};
  config::DayNightType day_night_type{config::DayNightType::kDay};
  foundation::propagation::NepNoiseModelInputs nep_inputs{};
  foundation::noise::BackgroundNoiseModelInputs noise_inputs{};
  foundation::stray_light::StrayLightFilterResult stray_light_result{};
};

DetectionComputationContext BuildDetectionComputationContext(
    const config::execution::EosInternalExecutionConfig& config,
    const EosPipelineSceneTarget& target,
    const EosPipelineCycleInput& input, const FrameContext& frame_ctx) {
  DetectionComputationContext context_values;
  // 从帧级上下文复制目标无关字段
  context_values.infrared_enabled = frame_ctx.infrared_enabled;
  context_values.visible_enabled = frame_ctx.visible_enabled;
  context_values.aperture_area_m2 = frame_ctx.aperture_area_m2;
  context_values.fov_solid_angle_sr = frame_ctx.fov_solid_angle_sr;
  context_values.wavelength_center_um = frame_ctx.wavelength_center_um;
  context_values.wavelength_bandwidth_um = frame_ctx.wavelength_bandwidth_um;
  context_values.background_spectral_radiance_w_sr_m3 =
      frame_ctx.background_spectral_radiance_w_sr_m3;
  context_values.visible_photon_noise_enhancement = frame_ctx.visible_photon_noise_enhancement;
  context_values.optical_transmittance = frame_ctx.optical_transmittance;
  context_values.nep_inputs = frame_ctx.nep_inputs;
  context_values.noise_inputs = frame_ctx.noise_inputs_base;
  context_values.solar_irradiance_w_m2 = config.environment.solar_irradiance_w_m2;
  context_values.solar_altitude_deg = config.environment.solar_altitude_deg;
  context_values.cloud_coverage_ratio = config.environment.cloud_coverage_ratio;
  context_values.day_night_type = config.environment.day_night_type;

  // 目标相关字段。
  // 注意：range_m 的权威校验在 EosInputValidation（<= 0 为 error），
  // 控制器在校验失败时不执行本管线，故正常 Session 路径不会传入非法 range_m。
  // 此处 SafePositive 仅作深度防御，防止直接调用管线（绕过校验）时非法值进入
  // 辐射传输计算引发 NaN 传播；兜底值 1000m 不应被视为合法输入约定。
  const foundation::radiative_transfer::RadiativeTransferResult transfer_result =
      ComputePathRadiativeTransfer(config, input,
                                   oneq::common::numerics::SafePositive(target.range_m, 1000.0f),
                                   frame_ctx.environment_result);
  context_values.path_transmittance = transfer_result.transmittance;
  context_values.path_radiance_penalty_scale =
      transfer_result.path_radiance_penalty_scale *
      frame_ctx.environment_result.path_radiance_scale_bias;

  context_values.is_within_detection_range =
      target.range_m >= frame_ctx.dmin_m && target.range_m <= frame_ctx.dmax_m;

  const float gsd_m = foundation::optics::ComputeGroundSampleDistanceM(
      target.range_m, frame_ctx.diffraction_resolution_rad);
  const float target_linear_size_m =
      std::sqrt(oneq::common::numerics::SafePositive(target.appearance.projected_area_m2, 1.0f));
  const float geometric_quality_gain = target_linear_size_m / (target_linear_size_m + gsd_m);
  foundation::spatial_spectrum::SpatialSpectrumInputs spectrum_inputs;
  spectrum_inputs.target_characteristic_size_m = target_linear_size_m;
  spectrum_inputs.ground_sample_distance_m = gsd_m;
  spectrum_inputs.optical_mtf_reference = 0.65f;
  spectrum_inputs.sampling_efficiency = 0.9f;
  spectrum_inputs.scene_contrast_ratio =
      oneq::common::numerics::Clamp01(0.5f * (std::max(0.0f, target.appearance.reflectance) +
                                                std::max(0.0f, target.appearance.emissivity)));
  const foundation::spatial_spectrum::SpatialSpectrumResult spectrum_result =
      foundation::spatial_spectrum::EvaluateSpatialResolvability(spectrum_inputs);
  context_values.imaging_quality_gain = oneq::common::numerics::Clamp(
      0.5f * geometric_quality_gain + 0.5f * spectrum_result.spectrum_quality_gain, 0.05f, 1.0f);

  context_values.noise_inputs.scene_complexity_factor =
      std::max(1.0f, 1.0f + 0.3f * (1.0f - context_values.imaging_quality_gain));

  foundation::stray_light::StrayLightFilterInputs stray_light_inputs;
  stray_light_inputs.enabled = config.stray_light.enable_straylight_filter;
  stray_light_inputs.target_azimuth_deg = target.azimuth_deg;
  stray_light_inputs.target_elevation_deg = target.elevation_deg;
  stray_light_inputs.sun_azimuth_deg = config.environment.solar_azimuth_deg;
  stray_light_inputs.sun_altitude_deg = config.environment.solar_altitude_deg;
  stray_light_inputs.cloud_coverage_ratio = config.environment.cloud_coverage_ratio;
  stray_light_inputs.hood_inner_half_angle_deg = config.stray_light.hood_inner_half_angle_deg;
  stray_light_inputs.hood_outer_half_angle_deg = config.stray_light.hood_outer_half_angle_deg;
  stray_light_inputs.min_suppression_ratio = config.stray_light.hood_min_suppression_ratio;
  stray_light_inputs.max_suppression_ratio = config.stray_light.hood_max_suppression_ratio;
  context_values.stray_light_result =
      foundation::stray_light::EvaluateStrayLightFilter(stray_light_inputs);
  return context_values;
}

float ComputeInfraredSnrLinear(const EosPipelineSceneTarget& target,
                               const EosPipelineCycleInput& input,
                               const DetectionComputationContext& context_values) {
  const float background_spectral_radiance = context_values.background_spectral_radiance_w_sr_m3;
  const float target_spectral_radiance = foundation::radiometry::ComputePlanckRadiance(
      context_values.wavelength_center_um, target.appearance.apparent_temperature_k);
  const float emissivity = oneq::common::numerics::Clamp01(target.appearance.emissivity);
  const float infrared_delta_spectral_radiance =
      emissivity * target_spectral_radiance - background_spectral_radiance;
  const float infrared_delta_radiance = foundation::radiometry::IntegrateSpectralRadianceOverBand(
      infrared_delta_spectral_radiance, context_values.wavelength_bandwidth_um);
  const float background_radiance = foundation::radiometry::IntegrateSpectralRadianceOverBand(
      background_spectral_radiance, context_values.wavelength_bandwidth_um);
  const float infrared_contrast = foundation::radiometry::ComputeRelativeContrast(
      background_radiance + infrared_delta_radiance, background_radiance);
  const float infrared_source_radiance =
      std::max(0.0f, infrared_delta_radiance) * (1.0f + std::max(0.0f, infrared_contrast));
  const float infrared_received_power_w =
      foundation::propagation::ComputeReceivedPowerW(
          infrared_source_radiance, target.appearance.projected_area_m2, target.range_m,
          context_values.aperture_area_m2, context_values.path_transmittance,
          context_values.optical_transmittance) *
      context_values.stray_light_result.signal_transmission_scale;
  const float infrared_background_flux_w =
      foundation::propagation::ComputeBackgroundFluxW(
          std::max(0.0f, background_radiance), context_values.aperture_area_m2,
          context_values.fov_solid_angle_sr, context_values.optical_transmittance) *
      context_values.stray_light_result.background_penalty_scale *
      context_values.path_radiance_penalty_scale;
  foundation::noise::BackgroundNoiseModelInputs noise_inputs = context_values.noise_inputs;
  noise_inputs.background_flux_w = infrared_background_flux_w;
  const foundation::noise::BackgroundNoiseStatistics infrared_noise_stats =
      foundation::noise::ComputeBackgroundNoiseStatistics(noise_inputs);
  const float infrared_effective_signal_w = foundation::noise::ComputeEffectiveSignalPowerW(
      infrared_received_power_w, infrared_background_flux_w, infrared_noise_stats);
  const foundation::propagation::SnrEvaluationResult snr_result =
      foundation::propagation::EvaluateSnrWithNep(infrared_effective_signal_w,
                                                  context_values.nep_inputs);
  return snr_result.snr_linear * context_values.imaging_quality_gain;
}

float ComputeVisibleSnrLinear(const EosPipelineSceneTarget& target,
                              const EosPipelineCycleInput& input,
                              const DetectionComputationContext& context_values) {
  foundation::radiometry::VisibleChannelInputs visible_inputs;
  visible_inputs.target.solar_irradiance_w_m2 = context_values.solar_irradiance_w_m2;
  visible_inputs.target.solar_altitude_deg = context_values.solar_altitude_deg;
  visible_inputs.target.cloud_coverage_ratio = context_values.cloud_coverage_ratio;
  visible_inputs.target.reflectance = target.appearance.reflectance;
  visible_inputs.target.illumination = ToIlluminationCondition(context_values.day_night_type);
  visible_inputs.background_reflectance = 0.12f;
  visible_inputs.background_patch_area_m2 = 20.0f;
  const foundation::radiometry::VisibleChannelResult visible_result =
      foundation::radiometry::ComputeVisibleChannelResult(visible_inputs);
  const float visible_received_power_w =
      foundation::propagation::ComputeReceivedPowerW(
          std::max(0.0f, visible_result.target_radiance) *
              (1.0f + std::max(0.0f, visible_result.normalized_contrast)),
          target.appearance.projected_area_m2, target.range_m, context_values.aperture_area_m2,
          context_values.path_transmittance, context_values.optical_transmittance) *
      context_values.stray_light_result.signal_transmission_scale;
  const float visible_background_flux_w =
      foundation::propagation::ComputeBackgroundFluxW(
          visible_result.background_radiance, context_values.aperture_area_m2,
          context_values.fov_solid_angle_sr, context_values.optical_transmittance) *
      context_values.stray_light_result.background_penalty_scale *
      context_values.path_radiance_penalty_scale;
  const float visible_noise_enhancement = context_values.visible_photon_noise_enhancement;
  foundation::noise::BackgroundNoiseModelInputs noise_inputs = context_values.noise_inputs;
  noise_inputs.background_flux_w = visible_background_flux_w;
  noise_inputs.photon_noise_enhancement_factor = visible_noise_enhancement;
  const foundation::noise::BackgroundNoiseStatistics visible_noise_stats =
      foundation::noise::ComputeBackgroundNoiseStatistics(noise_inputs);
  const float visible_effective_signal_w = foundation::noise::ComputeEffectiveSignalPowerW(
      visible_received_power_w, visible_background_flux_w, visible_noise_stats);
  foundation::propagation::NepNoiseModelInputs visible_nep_inputs = context_values.nep_inputs;
  visible_nep_inputs.system_noise_factor *= visible_noise_enhancement;
  const foundation::propagation::SnrEvaluationResult snr_result =
      foundation::propagation::EvaluateSnrWithNep(visible_effective_signal_w, visible_nep_inputs);
  return snr_result.snr_linear * context_values.imaging_quality_gain;
}

float ComputeFusedSnrLinear(EosPipelineWorkMode work_mode,
                            ::electro_optical_sensor::config::DayNightType day_night_type,
                            float infrared_snr_linear, float visible_snr_linear) {
  if (work_mode == EosPipelineWorkMode::kInfraredOnly) {
    return infrared_snr_linear;
  }
  if (work_mode == EosPipelineWorkMode::kVisibleOnly) {
    return visible_snr_linear;
  }

  const float safe_infrared_snr_linear = std::max(0.0f, infrared_snr_linear);
  const float safe_visible_snr_linear = std::max(0.0f, visible_snr_linear);
  if (safe_infrared_snr_linear <= 1.0e-9f) {
    return safe_visible_snr_linear;
  }
  if (safe_visible_snr_linear <= 1.0e-9f) {
    return safe_infrared_snr_linear;
  }

  float infrared_weight = 0.5f;
  float visible_weight = 0.5f;
  if (day_night_type == ::electro_optical_sensor::config::DayNightType::kDay) {
    infrared_weight = 0.35f;
    visible_weight = 0.65f;
  } else if (day_night_type == ::electro_optical_sensor::config::DayNightType::kNight) {
    infrared_weight = 0.80f;
    visible_weight = 0.20f;
  }
  return infrared_weight * safe_infrared_snr_linear + visible_weight * safe_visible_snr_linear;
}

bool IsCompatiblePipelineRuntimeState(const extension::EosPipelineRuntimeState& state,
                                      const EosPipeline* pipeline,
                                      const config::execution::EosInternalExecutionConfig& config) {
  return state.owner_identity == pipeline && state.schema_version == 1U &&
         state.scan_start_az_deg == config.scan.scan_start_az_deg &&
         state.scan_end_az_deg == config.scan.scan_end_az_deg &&
         state.scan_rate_deg_per_sec == config.scan.scan_rate_deg_per_sec;
}

}  // namespace

EosPipeline::EosPipeline(const config::execution::EosInternalExecutionConfig& config)
    : config_(config), current_scan_azimuth_deg_(config.scan.scan_start_az_deg) {}

void EosPipeline::ApplyInternalConfig(const config::execution::EosInternalExecutionConfig& config,
                                      bool reset_scan_phase) {
  config_ = config;
  if (reset_scan_phase) {
    current_scan_azimuth_deg_ = config_.scan.scan_start_az_deg;
  }
}

extension::EosPipelineRuntimeState EosPipeline::CaptureRuntimeState() const {
  extension::EosPipelineRuntimeState state;
  state.owner_identity = this;
  state.schema_version = 1U;
  state.current_scan_azimuth_deg = current_scan_azimuth_deg_;
  state.scan_start_az_deg = config_.scan.scan_start_az_deg;
  state.scan_end_az_deg = config_.scan.scan_end_az_deg;
  state.scan_rate_deg_per_sec = config_.scan.scan_rate_deg_per_sec;
  return state;
}

bool EosPipeline::RestoreRuntimeState(const extension::EosPipelineRuntimeState& state) {
  if (!IsCompatiblePipelineRuntimeState(state, this, config_)) {
    // 中译：流水线运行时状态恢复被拒绝：所有者/结构/配置不匹配。
    // 标识：执行回滚保护——跨周期状态校验失败时本周期不执行，
    //       避免在错误状态上继续成像；排查配置变更与状态不一致。
    PROJECT_LOG_ERROR(
        "[EosPipeline] runtime state restore rejected: owner/schema/config mismatch.");
    return false;
  }
  current_scan_azimuth_deg_ = state.current_scan_azimuth_deg;
  return true;
}

extension::EosPipelineExecuteResult EosPipeline::RunCycle(
    const EosPipelineCycleInput& input) {
  if (!config_.sensor_enabled) {
    extension::EosPipelineExecuteResult result;
    result.executed_this_cycle = false;
    result.abort_reason = session::EosPipelineAbortReason::kSensorPoweredOff;
    return result;
  }
  extension::EosPipelineExecuteResult result;
  AdvanceScan(input.dt_sec);
  result.scan_azimuth_deg = current_scan_azimuth_deg_;

  const FrameContext frame_ctx = BuildFrameContext(input);

  // 规则 13a：周期执行摘要所需视场外排除计数（按目标循环内累加）。
  std::size_t excluded_out_of_fov = 0U;

  for (std::size_t i = 0; i < input.scene.size(); ++i) {
    const EosPipelineSceneTarget& target = input.scene[i];
    if (!IsTargetInCurrentFov(target)) {
      // 规则 13b：视场外目标排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
      // 门内归因：视场门按越界轴细分（az/el/both），并补越界量（与
      // IsTargetInCurrentFov 同基准：半视场为门限）。
      const float azimuth_delta_deg = std::fabs(
          oneq::common::numerics::NormalizeAngle180(target.azimuth_deg - current_scan_azimuth_deg_));
      const float elevation_delta_deg =
          std::fabs(target.elevation_deg - config_.scan.scan_center_el_deg);
      const float half_hfov_deg = 0.5f * config_.scan.horizontal_fov_deg;
      const float half_vfov_deg = 0.5f * config_.scan.vertical_fov_deg;
      const bool az_out = azimuth_delta_deg > half_hfov_deg;
      const bool el_out = elevation_delta_deg > half_vfov_deg;
      const session::EosIssueCause fov_cause =
          az_out && el_out
              ? session::EosIssueCause::kBothAxesOutside
              : (az_out ? session::EosIssueCause::kAzOutside
                        : session::EosIssueCause::kElOutside);
      result.issues.push_back(MakeExclusionIssue(
          session::codes::kTargetOutOfFov,
          "target_id=" + std::to_string(target.target_id) + "; az/el (" +
              FormatFloat(target.azimuth_deg) + "," + FormatFloat(target.elevation_deg) +
              ") outside scan center (" + FormatFloat(current_scan_azimuth_deg_) + "," +
              FormatFloat(config_.scan.scan_center_el_deg) + ") fov " +
              FormatFloat(config_.scan.horizontal_fov_deg) + "x" +
              FormatFloat(config_.scan.vertical_fov_deg) + " az_delta_deg=" +
              FormatFloat(azimuth_delta_deg) + " el_delta_deg=" +
              FormatFloat(elevation_delta_deg),
          fov_cause,
          static_cast<std::ptrdiff_t>(i)));
      ++excluded_out_of_fov;
      continue;
    }
    const std::uint64_t detection_id = static_cast<std::uint64_t>(result.detections.size() + 1U);
    result.detections.push_back(BuildDetectionRecord(detection_id, target, input, frame_ctx));
    attribution::EosDetectionAttributionRecord attribution_record;
    attribution_record.detection_id = detection_id;
    attribution_record.target_id = target.target_id;
    attribution_record.target_name = target.target_name;
    result.detection_attributions.push_back(attribution_record);
    // 中译：目标检测结果（目标 ID、是否过门限、融合信噪比、斜距）。
    // 标识：逐目标探测摘要——标识每个目标是否被检测到及其信噪比，
    //       供排查门限判决与链路预算。
    PROJECT_LOG_DEBUG("[EosPipeline] target_id={} detected={} fused_snr_db={:.1f} range_m={:.0f}",
                      target.target_id, result.detections.back().detected,
                      result.detections.back().fused_snr_db, target.range_m);
  }

  const std::uint32_t effective_sensor_id =
      (sensor_entity_id_ != 0U) ? sensor_entity_id_ : 3U;

  // 中译：周期执行摘要（周期号、设备号、扫描方位角、检测数/目标总数、视场外排除计数）。
  // 标识：每周期探测概况——检测命中率、扫描位置与门控排除分布，供宏观核对与
  //       "零探测"排查；仅人读，不用于状态判断（规则 3）。
  PROJECT_LOG_INFO("[EosPipeline] cycle_index={} sensor_entity_id={} scan_az={:.2f} detections={}/{} "
                   "excluded={{fov={}}}",
                   input.cycle_index, effective_sensor_id, current_scan_azimuth_deg_,
                   result.detections.size(), input.scene.size(), excluded_out_of_fov);

  result.executed_this_cycle = true;
  result.abort_reason = session::EosPipelineAbortReason::kNone;
  return result;
}

void EosPipeline::AdvanceScan(float dt_sec) {
  const float scan_width_deg = config_.scan.scan_end_az_deg - config_.scan.scan_start_az_deg;
  if (scan_width_deg < 0.001f) {
    current_scan_azimuth_deg_ = config_.scan.scan_start_az_deg;
    return;
  }
  const float total_offset_deg = (current_scan_azimuth_deg_ - config_.scan.scan_start_az_deg) +
                                 config_.scan.scan_rate_deg_per_sec * dt_sec;
  float wrapped_offset_deg = std::fmod(total_offset_deg, scan_width_deg);
  if (wrapped_offset_deg < 0.0f) {
    wrapped_offset_deg += scan_width_deg;
  }
  current_scan_azimuth_deg_ = config_.scan.scan_start_az_deg + wrapped_offset_deg;
}

bool EosPipeline::IsTargetInCurrentFov(
    const EosPipelineSceneTarget& target) const {
  const float azimuth_delta_deg = std::fabs(
      oneq::common::numerics::NormalizeAngle180(target.azimuth_deg - current_scan_azimuth_deg_));
  const float elevation_delta_deg =
      std::fabs(target.elevation_deg - config_.scan.scan_center_el_deg);
  return azimuth_delta_deg <= 0.5f * config_.scan.horizontal_fov_deg &&
         elevation_delta_deg <= 0.5f * config_.scan.vertical_fov_deg;
}

FrameContext EosPipeline::BuildFrameContext(
    const EosPipelineCycleInput& input) const {
  FrameContext frame;
  frame.infrared_enabled = WorkModeIncludesInfrared(config_.scan.work_mode);
  frame.visible_enabled = WorkModeIncludesVisible(config_.scan.work_mode);
  frame.aperture_area_m2 = ComputeApertureAreaM2(config_.optics.optical_aperture_m);
  frame.fov_solid_angle_sr =
      ComputeFovSolidAngleSr(config_.scan.horizontal_fov_deg, config_.scan.vertical_fov_deg);

  const float wl_lower =
      oneq::common::numerics::SafePositive(config_.optics.wavelength_lower_um, 3.0f);
  const float wl_upper =
      oneq::common::numerics::SafePositive(config_.optics.wavelength_upper_um, 5.0f);
  frame.wavelength_center_um = 0.5f * (wl_lower + wl_upper);
  frame.wavelength_bandwidth_um = std::max(std::fabs(wl_upper - wl_lower), 0.1f);

  if (frame.infrared_enabled) {
    frame.background_spectral_radiance_w_sr_m3 = foundation::radiometry::ComputePlanckRadiance(
        frame.wavelength_center_um, config_.environment.background_temperature_k);
  }
  frame.visible_photon_noise_enhancement = ComputeVisiblePhotonNoiseEnhancement(config_, input);
  frame.diffraction_resolution_rad =
      foundation::optics::ComputeDiffractionLimitedAngularResolutionRad(
          frame.wavelength_center_um, config_.optics.optical_aperture_m);

  foundation::optics::DetectionRangeInputs range_inputs;
  range_inputs.platform_altitude_m = std::max(ResolvePlatformAltitudeM(input), 1.0f);
  range_inputs.boresight_depression_deg = config_.scan.boresight_depression_deg;
  range_inputs.vertical_fov_deg = config_.scan.vertical_fov_deg;
  range_inputs.min_depression_deg = config_.detector.min_detection_depression_deg;
  range_inputs.max_depression_deg = config_.detector.max_detection_depression_deg;
  frame.dmin_m = foundation::optics::ComputeMinimumDetectionRangeM(range_inputs);
  frame.dmax_m = foundation::optics::ComputeMaximumDetectionRangeM(range_inputs);

  frame.nep_inputs.detector_detectivity_cm_sqrt_hz_per_w = oneq::common::numerics::SafePositive(
      config_.detector.detector_detectivity_cm_sqrt_hz_per_w, 1.0e10f);
  frame.nep_inputs.detector_area_cm2 =
      oneq::common::numerics::SafePositive(config_.detector.detector_area_cm2, 0.25f);
  frame.nep_inputs.optical_transmittance = frame.optical_transmittance;
  frame.nep_inputs.integration_time_sec = ComputeSensorIntegrationTimeSec(config_, input);
  const float scan_bw =
      oneq::common::numerics::SafePositive(config_.scan.scan_rate_deg_per_sec, 20.0f) * 100.0f;
  const float frame_bw = 0.5f / frame.nep_inputs.integration_time_sec;
  frame.nep_inputs.electrical_bandwidth_hz = std::max(100.0f, std::max(scan_bw, frame_bw));
  frame.nep_inputs.system_noise_factor =
      ComputeSystemNoiseFactorFromSensitivity(config_.detection.detection_sensitivity_w);

  frame.noise_inputs_base.electrical_bandwidth_hz = frame.nep_inputs.electrical_bandwidth_hz;
  frame.noise_inputs_base.integration_time_sec = frame.nep_inputs.integration_time_sec;
  frame.noise_inputs_base.cloud_coverage_ratio = config_.environment.cloud_coverage_ratio;
  frame.noise_inputs_base.detector_area_cm2 = frame.nep_inputs.detector_area_cm2;

  environment::EnvironmentModelInputs env_inputs;
  env_inputs.base_aerosol_density_factor = config_.environment.aerosol_density_factor;
  env_inputs.base_turbulence_factor = config_.environment.turbulence_factor;
  env_inputs.platform_altitude_m = ResolvePlatformAltitudeM(input);
  env_inputs.cloud_coverage_ratio =
      oneq::common::numerics::Clamp01(config_.environment.cloud_coverage_ratio);
  env_inputs.wind_speed_mps = std::max(0.0f, config_.environment.ambient_wind_speed_mps);
  env_inputs.atmospheric_physics = config_.environment.atmospheric_physics;
  frame.environment_result = environment::ResolveEnvironmentFactors(env_inputs);

  return frame;
}

output::EosDetectionRecord EosPipeline::BuildDetectionRecord(
    std::uint64_t detection_id, const EosPipelineSceneTarget& target,
    const EosPipelineCycleInput& input,
    const FrameContext& frame_ctx) const {
  output::EosDetectionRecord record;
  record.detection_id = detection_id;
  record.range_m = target.range_m;
  record.azimuth_deg = target.azimuth_deg;
  record.elevation_deg = target.elevation_deg;
  const DetectionComputationContext context_values =
      BuildDetectionComputationContext(config_, target, input, frame_ctx);
  const float infrared_snr_linear = context_values.infrared_enabled
                                        ? ComputeInfraredSnrLinear(target, input, context_values)
                                        : 0.0f;
  const float visible_snr_linear = context_values.visible_enabled
                                       ? ComputeVisibleSnrLinear(target, input, context_values)
                                       : 0.0f;

  record.infrared_snr_linear = infrared_snr_linear;
  record.visible_snr_linear = visible_snr_linear;
  record.fused_snr_linear =
      ComputeFusedSnrLinear(config_.scan.work_mode, config_.environment.day_night_type,
                            infrared_snr_linear, visible_snr_linear);
  record.fused_snr_db = foundation::propagation::ComputeSnrDb(record.fused_snr_linear);
  record.detected = context_values.is_within_detection_range &&
                    record.fused_snr_db >= config_.detection.minimum_snr_db;
  return record;
}

}  // namespace pipeline
}  // namespace signal
}  // namespace electro_optical_sensor
