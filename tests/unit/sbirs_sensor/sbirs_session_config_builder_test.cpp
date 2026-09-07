#include <gtest/gtest.h>

#include <limits>

#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"

namespace {

// 统一问题列表模型（规则 14）：检查校验问题列表中是否包含指定 code（机器消费字段）。
bool ContainsCode(const sbirs_sensor::session::SbirsIssueList& issues, const std::string& code) {
  for (const sbirs_sensor::session::SbirsIssue& issue : issues) {
    if (issue.code == code) {
      return true;
    }
  }
  return false;
}

// 按 code 取条目（Q-2 审查修复：phase/severity 断言辅助）。
const sbirs_sensor::session::SbirsIssue* FindIssue(
    const sbirs_sensor::session::SbirsIssueList& issues, const std::string& code) {
  for (const sbirs_sensor::session::SbirsIssue& issue : issues) {
    if (issue.code == code) {
      return &issue;
    }
  }
  return nullptr;
}

TEST(SbirsSessionConfigValidationTest, AssignsFourDomainConfiguration) {
  sbirs_sensor::config::SbirsHardwareConfig hardware;
  hardware.optical_aperture_m = 0.7f;
  sbirs_sensor::config::SbirsMissionConfig mission;
  mission.work_mode = sbirs_sensor::config::SbirsWorkMode::kSearchAndStare;
  mission.scan_rate_deg_per_sec = 2.0f;
  sbirs_sensor::config::SbirsPolicyConfig policy;
  policy.detection.wide_min_snr_linear = 3.0f;
  sbirs_sensor::config::SbirsEnvironmentConfig environment;
  environment.weather_type = sbirs_sensor::config::SbirsWeatherType::kCloudy;

  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware = hardware;
  config.mission = mission;
  config.policy = policy;
  config.environment = environment;

  EXPECT_FLOAT_EQ(config.hardware.optical_aperture_m, 0.7f);
  EXPECT_EQ(config.mission.work_mode, sbirs_sensor::config::SbirsWorkMode::kSearchAndStare);
  EXPECT_FLOAT_EQ(config.mission.scan_rate_deg_per_sec, 2.0f);
  EXPECT_FLOAT_EQ(config.policy.detection.wide_min_snr_linear, 3.0f);
  EXPECT_EQ(config.environment.weather_type, sbirs_sensor::config::SbirsWeatherType::kCloudy);
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());
}

TEST(SbirsSessionConfigValidationTest, RejectsInvalidScanRate) {
  sbirs_sensor::config::SbirsMissionConfig mission;
  mission.scan_rate_deg_per_sec = -1.0f;
  sbirs_sensor::config::SbirsSessionConfig config;
  config.mission = mission;

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::config::ValidateSbirsSessionConfig(config);
  // config 域校验问题统一 phase=kInputValidation + severity=kError（规则 14 config 域；
  // HasValidationError 依赖 phase 判定，防误改时拒绝语义静默翻转）。
  const sbirs_sensor::session::SbirsIssue* issue =
      FindIssue(issues, "sbirs.validation.invalid_scan_rate");
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->phase, sbirs_sensor::session::SbirsIssuePhase::kInputValidation);
  EXPECT_EQ(issue->severity, sbirs_sensor::session::SbirsIssueSeverity::kError);
}

TEST(SbirsSessionConfigValidationTest, ValidatesCircularScanContract) {
  // ECI 方位约定（2026-08 正式变更）：scan_start ∈ [0, 360)；180 合法、
  // −180 与 360 非法（旧对称约定 [-180,180) 已废除）。
  sbirs_sensor::config::SbirsSessionConfig config;
  config.mission.scan_start_az_deg = 180.0f;
  config.mission.scan_span_deg = 360.0f;
  config.mission.scan_direction = sbirs_sensor::config::SbirsScanDirection::kDecreasingAzimuth;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());

  config.mission.scan_start_az_deg = -180.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_start_azimuth"));
  config.mission.scan_start_az_deg = 360.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_start_azimuth"));
  config.mission.scan_start_az_deg = 180.0f;
  config.mission.scan_span_deg = 0.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_span"));
  config.mission.scan_span_deg = 360.1f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_span"));
  config.mission.scan_span_deg = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_span"));
  config.mission.scan_span_deg = 120.0f;
  config.mission.scan_direction = static_cast<sbirs_sensor::config::SbirsScanDirection>(99);
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_direction"));
}

TEST(SbirsSessionConfigValidationTest, ValidatesNadirAzimuthReferenceContract) {
  // nadir 基准（2026-08-31）：scan_start_az_deg 语义切换为相对星下点方位的带符号
  // 偏移，合法域 (-360, 360)；eci_absolute（默认）维持 [0, 360) 既有约定。
  sbirs_sensor::config::SbirsSessionConfig config;
  config.mission.scan_azimuth_reference =
      sbirs_sensor::config::SbirsScanAzimuthReference::kNadirRelative;
  config.mission.scan_start_az_deg = 0.0f;   // 0 = 正对星下点
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());

  config.mission.scan_start_az_deg = -15.0f;  // 带符号偏移合法
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());

  config.mission.scan_start_az_deg = 360.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_start_azimuth"));
  config.mission.scan_start_az_deg = -360.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_start_azimuth"));

  // 绝对基准不放宽：负值与 ≥360 仍非法（防基准切换悄悄改变既有域）。
  config.mission.scan_azimuth_reference =
      sbirs_sensor::config::SbirsScanAzimuthReference::kEciAbsolute;
  config.mission.scan_start_az_deg = -15.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_start_azimuth"));
  config.mission.scan_start_az_deg = 0.0f;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());

  // 基准枚举本身须合法（review 修订：越界枚举不得静默退化绝对模式）。
  config.mission.scan_azimuth_reference =
      static_cast<sbirs_sensor::config::SbirsScanAzimuthReference>(99);
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_azimuth_reference"));
}

TEST(SbirsSessionConfigValidationTest, PointingDefaultsAreProductionValues) {
  const sbirs_sensor::config::SbirsMissionConfig mission;
  const sbirs_sensor::config::SbirsTrackingConfig tracking;

  EXPECT_FLOAT_EQ(mission.narrow_pointing_max_slew_rate_deg_per_sec, 30.0f);
  EXPECT_FLOAT_EQ(mission.narrow_pointing_settle_tolerance_deg, 0.01f);
  EXPECT_EQ(tracking.nfov_tracking_gate_loss_cycles, 2U);
  EXPECT_EQ(tracking.tracking_mode, sbirs_sensor::config::SbirsTrackingMode::kEstimated);
  EXPECT_EQ(tracking.estimated_backend,
            sbirs_sensor::config::SbirsEstimatedTrackingBackend::kEkf);
  const sbirs_sensor::config::SbirsPointingDisturbanceConfig disturbance;
  EXPECT_FLOAT_EQ(disturbance.common_attitude_sigma_deg, 0.0f);
  EXPECT_FLOAT_EQ(disturbance.channel_pointing_sigma_deg, 0.0f);
  EXPECT_FLOAT_EQ(disturbance.channel_vibration_amplitude_deg, 0.0f);
}

TEST(SbirsSessionConfigValidationTest, AcceptsAngleCvKfBackend) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.policy.tracking.estimated_backend =
      sbirs_sensor::config::SbirsEstimatedTrackingBackend::kAngleCvKf;
  EXPECT_FALSE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                            "sbirs.validation.invalid_estimated_tracking_backend"));
}

TEST(SbirsSessionConfigValidationTest, RejectsUnknownTrackingEnums) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.policy.tracking.tracking_mode =
      static_cast<sbirs_sensor::config::SbirsTrackingMode>(99);
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_tracking_mode"));

  config.policy.tracking.tracking_mode = sbirs_sensor::config::SbirsTrackingMode::kEstimated;
  config.policy.tracking.estimated_backend =
      static_cast<sbirs_sensor::config::SbirsEstimatedTrackingBackend>(99);
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_estimated_tracking_backend"));
}

TEST(SbirsSessionConfigValidationTest, RejectsInvalidPointingParameters) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 0.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_narrow_pointing_slew_rate"));

  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = std::numeric_limits<float>::infinity();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_narrow_pointing_slew_rate"));

  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 30.0f;
  config.mission.narrow_pointing_settle_tolerance_deg = -0.01f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_narrow_pointing_settle_tolerance"));

  config.mission.narrow_pointing_settle_tolerance_deg = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_narrow_pointing_settle_tolerance"));
}

TEST(SbirsSessionConfigValidationTest, RejectsZeroTrackingGateLossCycles) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.policy.tracking.nfov_tracking_gate_loss_cycles = 0U;

  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_tracking_gate_loss_cycles"));
}

TEST(SbirsSessionConfigValidationTest, RejectsNonPositiveFocalPlaneConfig) {
  // 焦平面几何（3.2.1.3.2.3）：焦距/像元间距非正须拒绝；默认值恒通过。
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.focal_length_m = 0.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.focal_plane_config_not_positive"));

  config.hardware.focal_length_m = 2.0f;
  config.hardware.detector_pixel_pitch_m = -1.0e-6f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.focal_plane_config_not_positive"));

  config.hardware.detector_pixel_pitch_m = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.focal_plane_config_not_positive"));
}

TEST(SbirsSessionConfigValidationTest, RejectsInvalidWideToNarrowRequiredHits) {
  // 宽窄切换前置条件（3.2.1.3.2.1）：连续命中阈值须 >=1；默认 1 恒通过。
  sbirs_sensor::config::SbirsSessionConfig config;
  config.policy.scheduler.wide_to_narrow_required_consecutive_hits = 0;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_wide_to_narrow_required_hits"));

  config.policy.scheduler.wide_to_narrow_required_consecutive_hits = 1;
  EXPECT_FALSE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                            "sbirs.validation.invalid_wide_to_narrow_required_hits"));
}

TEST(SbirsSessionConfigValidationTest, ValidatesPointingDisturbanceParameters) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.policy.pointing_disturbance.common_attitude_sigma_deg = -0.1f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_pointing_disturbance_values"));

  config.policy.pointing_disturbance.common_attitude_sigma_deg = 0.0f;
  config.policy.pointing_disturbance.common_attitude_correlation_time_s = 0.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_pointing_disturbance_correlation"));

  config.policy.pointing_disturbance.common_attitude_correlation_time_s = 1.0f;
  config.policy.pointing_disturbance.channel_pointing_sigma_deg =
      std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_pointing_disturbance_values"));

  config.policy.pointing_disturbance.channel_pointing_sigma_deg = 0.0f;
  config.policy.pointing_disturbance.channel_pointing_correlation_time_s = -1.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_pointing_disturbance_correlation"));

  config.policy.pointing_disturbance.channel_pointing_correlation_time_s = 1.0f;
  config.policy.pointing_disturbance.channel_vibration_amplitude_deg = 0.1f;
  config.policy.pointing_disturbance.channel_vibration_frequency_hz = 0.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_pointing_disturbance_vibration_frequency"));

  config.policy.pointing_disturbance.channel_vibration_frequency_hz = 2.0f;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());
}

TEST(SbirsSessionConfigValidationTest, OrientationDomainValidation) {
  // 阶段 2 安装指向域：默认配置（零安装角、全开限位、体稳定）必须通过校验。
  sbirs_sensor::config::SbirsSessionConfig config;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());

  // 安装欧拉角非有限 → 拒绝。
  config.orientation.mount_angles_deg.yaw_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_mount_angles"));
  config.orientation.mount_angles_deg.yaw_deg = 0.0;

  // 限位倒置/超域 → 拒绝。
  config.orientation.sensor_scan_limits_deg.az_min_deg = 10.0f;
  config.orientation.sensor_scan_limits_deg.az_max_deg = -10.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.sensor_scan_limits_swapped_azimuth"));
  config.orientation.sensor_scan_limits_deg.az_min_deg = -180.0f;
  config.orientation.sensor_scan_limits_deg.az_max_deg = 180.0f;

  config.orientation.sensor_scan_limits_deg.el_min_deg = 20.0f;
  config.orientation.sensor_scan_limits_deg.el_max_deg = -20.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.sensor_scan_limits_swapped_elevation"));
  config.orientation.sensor_scan_limits_deg.el_min_deg = -90.0f;
  config.orientation.sensor_scan_limits_deg.el_max_deg = 90.0f;

  config.orientation.sensor_scan_limits_deg.az_min_deg = -200.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.sensor_scan_limits_out_of_range"));
  config.orientation.sensor_scan_limits_deg.az_min_deg = -180.0f;

  // 稳定方式枚举非法 → 拒绝。
  config.orientation.stabilization_mode =
      static_cast<sbirs_sensor::config::SbirsStabilizationMode>(99);
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_stabilization_mode"));
  config.orientation.stabilization_mode =
      sbirs_sensor::config::SbirsStabilizationMode::kBodyStabilized;

  // 安装失准（阶段 3）：bias 非有限 / random sigma 负或非有限 → 拒绝；默认全零合法。
  config.orientation.misalignment.bias_deg.yaw_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_misalignment"));
  config.orientation.misalignment.bias_deg.yaw_deg = 0.0;

  config.orientation.misalignment.bias_deg.pitch_deg =
      std::numeric_limits<double>::infinity();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_misalignment"));
  config.orientation.misalignment.bias_deg.pitch_deg = 0.0;

  config.orientation.misalignment.random_sigma_deg = -1.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_misalignment"));
  config.orientation.misalignment.random_sigma_deg = 0.0f;

  config.orientation.misalignment.random_sigma_deg = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_misalignment"));
  config.orientation.misalignment.random_sigma_deg = 0.0f;

  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());
}

TEST(SbirsSessionConfigValidationTest, ScanPathMustFitSensorScanLimits) {
  // 扫描路径适配限位：方位扫掠弧（起点 + 有向跨度）与中心俯仰必须落在限位窗口内。
  sbirs_sensor::config::SbirsSessionConfig config;
  config.mission.scan_start_az_deg = 300.0f;
  config.mission.scan_span_deg = 120.0f;
  config.mission.scan_center_el_deg = 0.0f;
  config.orientation.sensor_scan_limits_deg.az_min_deg = 0.0f;
  config.orientation.sensor_scan_limits_deg.az_max_deg = 90.0f;
  // 300°→420° 弧段（对称域 [-60,60]）超 [0,90] 限位 → 拒绝。
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.scan_path_outside_sensor_limits"));

  // 弧段适配限位（对称域内 [0,60] ⊂ [0,90]）→ 通过。
  config.mission.scan_start_az_deg = 0.0f;
  config.mission.scan_span_deg = 60.0f;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());

  // 中心俯仰超限位 → 拒绝。
  config.orientation.sensor_scan_limits_deg.el_min_deg = -10.0f;
  config.orientation.sensor_scan_limits_deg.el_max_deg = 10.0f;
  config.mission.scan_center_el_deg = 20.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.scan_path_outside_sensor_limits"));

  // 惯性稳定同样受限位约束（扫描参数仍须在传感器系窗口内）。
  config.mission.scan_center_el_deg = 5.0f;
  config.orientation.stabilization_mode =
      sbirs_sensor::config::SbirsStabilizationMode::kInertialStabilized;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());
}

TEST(SbirsSessionConfigValidationTest, RejectsInvalidElevationRaster) {
  // 阶段 4 俯仰栅格：span 非负有限、step 正有限；span>0 时 step 不得超 WFOV 俯仰视场。
  sbirs_sensor::config::SbirsSessionConfig config;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());

  config.mission.scan_el_span_deg = -1.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_elevation_raster"));
  config.mission.scan_el_span_deg = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_elevation_raster"));
  config.mission.scan_el_span_deg = 0.0f;

  config.mission.scan_el_step_deg = 0.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_elevation_raster"));
  config.mission.scan_el_step_deg = -2.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_elevation_raster"));
  config.mission.scan_el_step_deg = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_elevation_raster"));
  config.mission.scan_el_step_deg = 1.0f;

  // span>0 且 step 超 WFOV 俯仰视场（默认 20°）→ 无隙覆盖预算违反。
  config.mission.scan_el_span_deg = 40.0f;
  config.mission.scan_el_step_deg = 25.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.scan_elevation_step_exceeds_fov"));
  config.mission.scan_el_step_deg = 20.0f;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());
  config.mission.scan_el_span_deg = 0.0f;
}

TEST(SbirsSessionConfigValidationTest, ElevationRasterMustFitSensorScanLimits) {
  // 栅格模式首末行中心 el 须落在传感器系限位窗口内（span>0 走 el_start 与 el_start+span）。
  sbirs_sensor::config::SbirsSessionConfig config;
  config.mission.scan_el_span_deg = 20.0f;
  config.mission.scan_el_start_deg = 0.0f;
  config.mission.scan_el_step_deg = 10.0f;
  config.orientation.sensor_scan_limits_deg.el_min_deg = -5.0f;
  config.orientation.sensor_scan_limits_deg.el_max_deg = 15.0f;
  // 末行中心 20° 超 [−5, 15] 限位 → 拒绝。
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.scan_path_outside_sensor_limits"));

  config.orientation.sensor_scan_limits_deg.el_max_deg = 20.0f;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());

  // span=0 单行模式沿用既有 scan_center_el_deg 检查（栅格字段不参与）。
  config.mission.scan_el_span_deg = 0.0f;
  config.mission.scan_center_el_deg = 30.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.scan_path_outside_sensor_limits"));
  config.mission.scan_center_el_deg = 0.0f;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());
}

TEST(SbirsSessionConfigValidationTest, ValidatesInitialScanPhase) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.mission.scan_span_deg = 60.0f;
  config.mission.initial_scan_phase_deg = 0.0f;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());

  config.mission.initial_scan_phase_deg = 30.0f;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());

  config.mission.initial_scan_phase_deg = -0.1f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_initial_scan_phase"));

  config.mission.initial_scan_phase_deg = 60.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_initial_scan_phase"));

  config.mission.initial_scan_phase_deg = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_initial_scan_phase"));
}

}  // namespace

