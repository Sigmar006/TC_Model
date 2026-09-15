#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"

#include <cmath>
#include <string>

#include "1q/sbirs_sensor/session/SbirsIssueCodes.h"

namespace sbirs_sensor {
namespace config {
namespace {

// 统一问题列表模型（规则 14）：配置校验问题 code 引用 SbirsIssueCodes.h 常量
// （"sbirs.validation.<snake_case>" 全集单一事实来源），severity 固定为 kError，
// phase 固定为 kInputValidation；location/field 保持默认（无定位）。
void AddError(const char* code, const char* message, session::SbirsIssueList* issues) {
  session::SbirsIssue issue;
  issue.severity = session::SbirsIssueSeverity::kError;
  issue.phase = session::SbirsIssuePhase::kInputValidation;
  issue.code = code;
  issue.message = message;
  issues->push_back(issue);
}

}  // namespace

session::SbirsIssueList ValidateSbirsSessionConfig(const SbirsSessionConfig& config) {
  session::SbirsIssueList issues;
  if (config.hardware.wavelength_lower_um <= 0.0f ||
      config.hardware.wavelength_upper_um <= config.hardware.wavelength_lower_um) {
    AddError(session::codes::kWavelengthBandInvalid,
             "hardware wavelength band must be positive and ordered", &issues);
  }
  if (config.hardware.optical_aperture_m <= 0.0f) {
    AddError(session::codes::kOpticalApertureNotPositive,
             "hardware optical aperture must be positive", &issues);
  }
  // 焦平面几何（3.2.1.3.2.3）：焦距/像元间距须为正有限值；仅验收日志消费，默认值恒通过。
  if (!std::isfinite(config.hardware.focal_length_m) ||
      config.hardware.focal_length_m <= 0.0f ||
      !std::isfinite(config.hardware.detector_pixel_pitch_m) ||
      config.hardware.detector_pixel_pitch_m <= 0.0f) {
    AddError(session::codes::kFocalPlaneConfigNotPositive,
             "hardware focal length and detector pixel pitch must be positive and finite", &issues);
  }
  if (config.mission.wide_field_fov_az_deg <= 0.0f ||
      config.mission.wide_field_fov_el_deg <= 0.0f ||
      config.mission.narrow_field_fov_az_deg <= 0.0f ||
      config.mission.narrow_field_fov_el_deg <= 0.0f) {
    AddError(session::codes::kMissionFovNotPositive, "mission FOV values must be positive",
             &issues);
  }
  // ECI 方位角约定（2026-08 正式变更）：扫描起始方位为 ECI 极坐标 az，
  // 取值范围 [0, 360)。nadir 基准（2026-08-31）：scan_start_az_deg 语义切换为
  // 相对星下点方位的带符号偏移，合法域 (-360, 360)；两域互斥，按基准分别校验。
  const bool nadir_referenced =
      config.mission.scan_azimuth_reference == SbirsScanAzimuthReference::kNadirRelative;
  if (!std::isfinite(config.mission.scan_start_az_deg) ||
      (nadir_referenced
           ? (config.mission.scan_start_az_deg <= -360.0f ||
              config.mission.scan_start_az_deg >= 360.0f)
           : (config.mission.scan_start_az_deg < 0.0f ||
              config.mission.scan_start_az_deg >= 360.0f))) {
    AddError(session::codes::kInvalidScanStartAzimuth,
             nadir_referenced
                 ? "mission scan start offset must be finite and in (-360, 360) when "
                   "nadir-referenced"
                 : "mission scan start azimuth must be finite and in [0, 360)",
             &issues);
  }
  if (!std::isfinite(config.mission.scan_span_deg) || config.mission.scan_span_deg <= 0.0f ||
      config.mission.scan_span_deg > 360.0f) {
    AddError(session::codes::kInvalidScanSpan,
             "mission scan span must be finite and in (0, 360]", &issues);
  }
  if (!std::isfinite(config.mission.initial_scan_phase_deg) ||
      config.mission.initial_scan_phase_deg < 0.0f ||
      (std::isfinite(config.mission.scan_span_deg) && config.mission.scan_span_deg > 0.0f &&
       config.mission.initial_scan_phase_deg >= config.mission.scan_span_deg)) {
    AddError(session::codes::kInvalidInitialScanPhase,
             "mission initial scan phase must be finite and in [0, scan_span)", &issues);
  }
  if (config.mission.scan_direction != SbirsScanDirection::kIncreasingAzimuth &&
      config.mission.scan_direction != SbirsScanDirection::kDecreasingAzimuth) {
    AddError(session::codes::kInvalidScanDirection, "mission scan direction is invalid", &issues);
  }
  // 方位基准枚举合法性（2026-08-31，review 修订）：直接构造 struct 塞越界枚举会被拒，
  // 不能静默退化为绝对模式（与 scan_direction 同款检查）。
  if (config.mission.scan_azimuth_reference != SbirsScanAzimuthReference::kEciAbsolute &&
      config.mission.scan_azimuth_reference != SbirsScanAzimuthReference::kNadirRelative) {
    AddError(session::codes::kInvalidScanAzimuthReference,
             "mission scan azimuth reference is invalid", &issues);
  }
  if (config.mission.max_range_m <= config.mission.min_range_m ||
      config.mission.min_range_m < 0.0f) {
    AddError(session::codes::kInvalidRangeGate,
             "mission range gate must be ordered and non-negative", &issues);
  }
  if (config.mission.frame_rate_hz <= 0.0f) {
    AddError(session::codes::kFrameRateNotPositive, "mission frame rate must be positive", &issues);
  }
  if (!std::isfinite(config.mission.scan_rate_deg_per_sec) ||
      config.mission.scan_rate_deg_per_sec < 0.0f) {
    AddError(session::codes::kInvalidScanRate,
             "mission scan rate must be non-negative and finite", &issues);
  }
  // 俯仰栅格（阶段 4）：span 非负有限、step 正有限；span>0 时行间距不得超 WFOV 俯仰
  // 视场（无隙覆盖预算）。默认 span=0（单行模式）下本规则恒通过。
  if (!std::isfinite(config.mission.scan_el_start_deg) ||
      !std::isfinite(config.mission.scan_el_span_deg) || config.mission.scan_el_span_deg < 0.0f ||
      !std::isfinite(config.mission.scan_el_step_deg) || config.mission.scan_el_step_deg <= 0.0f) {
    AddError(session::codes::kInvalidScanElevationRaster,
             "mission elevation raster requires finite el start, non-negative finite span and "
             "positive finite step",
             &issues);
  } else if (config.mission.scan_el_span_deg > 0.0f &&
             config.mission.scan_el_step_deg > config.mission.wide_field_fov_el_deg) {
    AddError(session::codes::kScanElevationStepExceedsFov,
             "mission elevation raster step must not exceed WFOV elevation field of view "
             "(gap-free coverage budget)",
             &issues);
  }
  if (!std::isfinite(config.mission.narrow_pointing_max_slew_rate_deg_per_sec) ||
      config.mission.narrow_pointing_max_slew_rate_deg_per_sec <= 0.0f) {
    AddError(session::codes::kInvalidNarrowPointingSlewRate,
             "mission narrow pointing max slew rate must be positive and finite", &issues);
  }
  if (!std::isfinite(config.mission.narrow_pointing_settle_tolerance_deg) ||
      config.mission.narrow_pointing_settle_tolerance_deg < 0.0f) {
    AddError(session::codes::kInvalidNarrowPointingSettleTolerance,
             "mission narrow pointing settle tolerance must be non-negative and finite", &issues);
  }
  // 安装指向域（对齐 ArOrientationConfig）：mount 欧拉有限、限位有限且有序并在合法域内、
  // 稳定方式合法；WFOV 方位扫掠弧与中心俯仰须落在传感器系限位窗口内（弧段提升到
  // [az_min, az_min+360) 后判界，跨 ±180° 缝的合法情形由全开窗口特例覆盖；
  // 默认全开限位下本规则恒通过，保证既有配置零行为变化）。
  const SbirsOrientationConfig& orientation = config.orientation;
  if (!std::isfinite(orientation.mount_angles_deg.yaw_deg) ||
      !std::isfinite(orientation.mount_angles_deg.pitch_deg) ||
      !std::isfinite(orientation.mount_angles_deg.roll_deg)) {
    AddError(session::codes::kInvalidMountAngles, "orientation mount angles must be finite",
             &issues);
  }
  // 安装失准（阶段 3）：bias 三分量有限、random sigma 非负且有限；random_seed 不校验
  // （0 归一化到 1，沿既有种子惯例）。默认全零下本规则恒通过。
  const SbirsMisalignmentModel& misalignment = orientation.misalignment;
  if (!std::isfinite(misalignment.bias_deg.yaw_deg) ||
      !std::isfinite(misalignment.bias_deg.pitch_deg) ||
      !std::isfinite(misalignment.bias_deg.roll_deg) ||
      !std::isfinite(misalignment.random_sigma_deg) || misalignment.random_sigma_deg < 0.0f) {
    AddError(session::codes::kInvalidMisalignment,
             "misalignment bias must be finite and random sigma must be non-negative and finite",
             &issues);
  }
  const SbirsScanLimitsDeg& sensor_limits = orientation.sensor_scan_limits_deg;
  if (!std::isfinite(sensor_limits.az_min_deg) || !std::isfinite(sensor_limits.az_max_deg) ||
      sensor_limits.az_min_deg > sensor_limits.az_max_deg) {
    AddError(session::codes::kSensorScanLimitsSwappedAzimuth,
             "orientation sensor scan azimuth limits must be finite and ordered", &issues);
  }
  if (!std::isfinite(sensor_limits.el_min_deg) || !std::isfinite(sensor_limits.el_max_deg) ||
      sensor_limits.el_min_deg > sensor_limits.el_max_deg) {
    AddError(session::codes::kSensorScanLimitsSwappedElevation,
             "orientation sensor scan elevation limits must be finite and ordered", &issues);
  }
  if (sensor_limits.az_min_deg < -180.0f || sensor_limits.az_max_deg > 180.0f ||
      sensor_limits.el_min_deg < -90.0f || sensor_limits.el_max_deg > 90.0f) {
    AddError(session::codes::kSensorScanLimitsOutOfRange,
             "orientation sensor scan limits must stay within az [-180, 180] and el [-90, 90]",
             &issues);
  }
  if (orientation.stabilization_mode != SbirsStabilizationMode::kBodyStabilized &&
      orientation.stabilization_mode != SbirsStabilizationMode::kInertialStabilized) {
    AddError(session::codes::kInvalidStabilizationMode,
             "orientation stabilization mode is invalid", &issues);
  }
  const bool sensor_limits_valid =
      std::isfinite(sensor_limits.az_min_deg) && std::isfinite(sensor_limits.az_max_deg) &&
      std::isfinite(sensor_limits.el_min_deg) && std::isfinite(sensor_limits.el_max_deg) &&
      sensor_limits.az_min_deg <= sensor_limits.az_max_deg &&
      sensor_limits.el_min_deg <= sensor_limits.el_max_deg &&
      sensor_limits.az_min_deg >= -180.0f && sensor_limits.az_max_deg <= 180.0f &&
      sensor_limits.el_min_deg >= -90.0f && sensor_limits.el_max_deg <= 90.0f;
  const bool start_domain_ok =
      std::isfinite(config.mission.scan_start_az_deg) &&
      (nadir_referenced
           ? (config.mission.scan_start_az_deg > -360.0f &&
              config.mission.scan_start_az_deg < 360.0f)
           : (config.mission.scan_start_az_deg >= 0.0f &&
              config.mission.scan_start_az_deg < 360.0f));
  const bool scan_params_valid = start_domain_ok &&
                                 std::isfinite(config.mission.scan_span_deg) &&
                                 config.mission.scan_span_deg > 0.0f &&
                                 config.mission.scan_span_deg <= 360.0f &&
                                 std::isfinite(config.mission.scan_center_el_deg) &&
                                 std::isfinite(config.mission.scan_el_start_deg) &&
                                 std::isfinite(config.mission.scan_el_span_deg) &&
                                 config.mission.scan_el_span_deg >= 0.0f &&
                                 std::isfinite(config.mission.scan_el_step_deg) &&
                                 config.mission.scan_el_step_deg > 0.0f;
  if (sensor_limits_valid && scan_params_valid) {
    const float kEps = 1.0e-4f;
    const float az_window_deg = sensor_limits.az_max_deg - sensor_limits.az_min_deg;
    // nadir 基准的有效起点 = 星下点方位 + 偏移，依赖运行期卫星位置，config 期无法
    // 静态判弧段越限（阶段裁定：跳过方位弧段检查，俯仰检查保留；全开窗口本就放行）。
    bool az_path_ok = az_window_deg >= 360.0f - kEps || nadir_referenced;
    if (!az_path_ok) {
      float start_symmetric_deg =
          std::fmod(config.mission.scan_start_az_deg + 180.0f, 360.0f);
      if (start_symmetric_deg < 0.0f) {
        start_symmetric_deg += 360.0f;
      }
      start_symmetric_deg -= 180.0f;
      float lifted_start_deg = start_symmetric_deg;
      if (lifted_start_deg < sensor_limits.az_min_deg) {
        lifted_start_deg += 360.0f;
      }
      const bool increasing =
          config.mission.scan_direction == SbirsScanDirection::kIncreasingAzimuth;
      az_path_ok = lifted_start_deg <= sensor_limits.az_max_deg + kEps &&
                   (increasing
                        ? lifted_start_deg + config.mission.scan_span_deg <=
                              sensor_limits.az_max_deg + kEps
                        : lifted_start_deg - config.mission.scan_span_deg >=
                              sensor_limits.az_min_deg - kEps);
    }
    // 俯仰限位适配（阶段 4）：span=0 单行模式沿用既有 scan_center_el_deg 检查；
    // span>0 栅格模式首末行中心 el（el_start 与 el_start+span）须落在限位窗口内。
    float scan_el_first_deg = config.mission.scan_center_el_deg;
    float scan_el_last_deg = config.mission.scan_center_el_deg;
    if (config.mission.scan_el_span_deg > 0.0f) {
      scan_el_first_deg = config.mission.scan_el_start_deg;
      scan_el_last_deg = config.mission.scan_el_start_deg + config.mission.scan_el_span_deg;
    }
    if (!az_path_ok || scan_el_first_deg < sensor_limits.el_min_deg - kEps ||
        scan_el_last_deg > sensor_limits.el_max_deg + kEps) {
      AddError(session::codes::kScanPathOutsideSensorLimits,
               "mission scan sweep or elevation raster exceeds sensor scan limits", &issues);
    }
  }
  if (config.policy.detection.wide_min_snr_linear < 0.0f ||
      config.policy.detection.narrow_min_snr_linear < 0.0f) {
    AddError(session::codes::kInvalidDetectionThresholds,
             "detection thresholds must be non-negative", &issues);
  }
  const SbirsErrorModelConfig& error_model = config.policy.error_model;
  if (!std::isfinite(error_model.orbit_sigma_deg) || error_model.orbit_sigma_deg < 0.0f ||
      !std::isfinite(error_model.attitude_sigma_deg) || error_model.attitude_sigma_deg < 0.0f ||
      !std::isfinite(error_model.fov_sigma_deg) || error_model.fov_sigma_deg < 0.0f ||
      !std::isfinite(error_model.range_fraction_sigma) ||
      error_model.range_fraction_sigma < 0.0f) {
    AddError(session::codes::kInvalidErrorModelSigmas,
             "error model sigma values must be non-negative and finite", &issues);
  }
  if (!std::isfinite(error_model.detector_bandwidth_hz) ||
      error_model.detector_bandwidth_hz <= 0.0f) {
    AddError(session::codes::kInvalidDetectorBandwidth,
             "error model detector bandwidth must be positive and finite", &issues);
  }
  const SbirsPointingDisturbanceConfig& disturbance = config.policy.pointing_disturbance;
  if (!std::isfinite(disturbance.common_attitude_sigma_deg) ||
      disturbance.common_attitude_sigma_deg < 0.0f ||
      !std::isfinite(disturbance.channel_pointing_sigma_deg) ||
      disturbance.channel_pointing_sigma_deg < 0.0f ||
      !std::isfinite(disturbance.channel_vibration_amplitude_deg) ||
      disturbance.channel_vibration_amplitude_deg < 0.0f ||
      !std::isfinite(disturbance.channel_vibration_frequency_hz) ||
      disturbance.channel_vibration_frequency_hz < 0.0f) {
    AddError(session::codes::kInvalidPointingDisturbanceValues,
             "pointing disturbance amplitudes and frequency must be non-negative and finite",
             &issues);
  }
  if (!std::isfinite(disturbance.common_attitude_correlation_time_s) ||
      disturbance.common_attitude_correlation_time_s <= 0.0f ||
      !std::isfinite(disturbance.channel_pointing_correlation_time_s) ||
      disturbance.channel_pointing_correlation_time_s <= 0.0f) {
    AddError(session::codes::kInvalidPointingDisturbanceCorrelation,
             "pointing disturbance correlation times must be positive and finite", &issues);
  }
  if (disturbance.channel_vibration_amplitude_deg > 0.0f &&
      disturbance.channel_vibration_frequency_hz <= 0.0f) {
    AddError(session::codes::kInvalidPointingDisturbanceVibrationFrequency,
             "pointing disturbance vibration frequency must be positive when amplitude is non-zero",
             &issues);
  }
  if (config.policy.scheduler.wide_to_narrow_required_consecutive_hits < 1) {
    AddError(session::codes::kInvalidWideToNarrowRequiredHits,
             "scheduler wide_to_narrow_required_consecutive_hits must be at least 1", &issues);
  }
  const SbirsTrackingConfig& tracking = config.policy.tracking;
  if (tracking.tracking_mode != SbirsTrackingMode::kEstimated &&
      tracking.tracking_mode != SbirsTrackingMode::kStrictTruthAssisted &&
      tracking.tracking_mode != SbirsTrackingMode::kSensorLikeTruthAssisted) {
    AddError(session::codes::kInvalidTrackingMode, "tracking mode is invalid", &issues);
  }
  if (tracking.estimated_backend != SbirsEstimatedTrackingBackend::kEkf &&
      tracking.estimated_backend != SbirsEstimatedTrackingBackend::kImm &&
      tracking.estimated_backend != SbirsEstimatedTrackingBackend::kAngleCvKf) {
    AddError(session::codes::kInvalidEstimatedTrackingBackend,
             "estimated tracking backend is invalid", &issues);
  }
  if (config.policy.tracking.nfov_tracking_gate_loss_cycles < 1U) {
    AddError(session::codes::kInvalidTrackingGateLossCycles,
             "tracking nfov_tracking_gate_loss_cycles must be at least 1", &issues);
  }
  return issues;
}

}  // namespace config
}  // namespace sbirs_sensor
