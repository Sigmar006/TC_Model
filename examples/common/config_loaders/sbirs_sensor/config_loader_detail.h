#ifndef EXAMPLES_SBIRS_CONFIG_LOADER_DETAIL_H_
#define EXAMPLES_SBIRS_CONFIG_LOADER_DETAIL_H_

#include "1q/sbirs_sensor/sbirs_sensor.hpp"
#include "json_reader.h"
#include "config_loader_common.h"

namespace examples {

// -- euler / vec helpers -----------------------------------------------------

// [yaw_deg, pitch_deg, roll_deg] 三元数组 → 欧拉角；非三元数组返回 false（缺字段
// 时调用方直接跳过，保持库默认，与其它可选字段 Has 门控同策略）。
inline bool LoadEulerAnglesDegArray(const examples::JsonValue& j,
                                    oneq::coordinate::EulerAnglesDeg* v) {
  if (j.IsNull() || j.type() != examples::JsonValue::kArray || j.Size() < 3) return false;
  v->yaw_deg = static_cast<float>(j[static_cast<std::size_t>(0)].AsDouble());
  v->pitch_deg = static_cast<float>(j[static_cast<std::size_t>(1)].AsDouble());
  v->roll_deg = static_cast<float>(j[static_cast<std::size_t>(2)].AsDouble());
  return true;
}

// -- struct loaders ----------------------------------------------------------

// 安装指向域加载器默认集（2026-09-01，源自 sbirs_triple_sat_fix_messages 验收演示
// 参数提升为默认）：名义安装角 (1.0, 0.5, -0.5)°、常值失准偏置 (0.10, -0.05, 0.02)°、
// 失准随机微扰 1-σ 0.02°（种子 7）；稳定方式/限位沿库默认（体稳定、全开）。
// 字面量带 f 后缀：与 JSON 解析路径 static_cast<float> 逐位一致，保证缺键继承与
// 显式写键产生相同日志。库结构体 SbirsOrientationConfig 默认仍为全零——单测/库内
// 几何不受影响，默认集只作用于 JSON 加载层。
inline sbirs_sensor::config::SbirsOrientationConfig SbirsOrientationDefaults() {
  sbirs_sensor::config::SbirsOrientationConfig v;
  v.mount_angles_deg = oneq::coordinate::EulerAnglesDeg(1.0f, 0.5f, -0.5f);
  v.misalignment.bias_deg = oneq::coordinate::EulerAnglesDeg(0.10f, -0.05f, 0.02f);
  v.misalignment.random_sigma_deg = 0.02f;
  v.misalignment.random_seed = 7U;
  return v;
}

// 安装指向域（SbirsOrientationConfig）：缺段/缺键继承 SbirsOrientationDefaults()
// 默认集，写了的键逐一覆盖。键清单：
// mount_angles_deg=[yaw,pitch,roll] 安装角；misalignment_bias_deg=[yaw,pitch,roll]
// 常值失准偏置；misalignment_random_sigma_deg 运行期一次抽取的微扰 1-σ；
// misalignment_random_seed 微扰种子；stabilization_mode 枚举串；
// sensor_scan_limits_deg=[az_min,az_max,el_min,el_max] 传感器系扫描限位。
// 既支持 config_loader.h 的嵌套 "orientation" 段，也支持场景逐星扁平块直接调用。
inline void LoadSbirsOrientation(const examples::JsonValue& j,
                                 sbirs_sensor::config::SbirsOrientationConfig* v) {
  *v = SbirsOrientationDefaults();
  if (j.IsNull()) return;
  if (j.Has("mount_angles_deg")) {
    LoadEulerAnglesDegArray(j["mount_angles_deg"], &v->mount_angles_deg);
  }
  if (j.Has("misalignment_bias_deg")) {
    LoadEulerAnglesDegArray(j["misalignment_bias_deg"], &v->misalignment.bias_deg);
  }
  if (j.Has("misalignment_random_sigma_deg")) {
    v->misalignment.random_sigma_deg =
        static_cast<float>(j["misalignment_random_sigma_deg"].AsDouble());
  }
  if (j.Has("misalignment_random_seed")) {
    v->misalignment.random_seed =
        static_cast<std::uint32_t>(j["misalignment_random_seed"].AsInt());
  }
  if (j.Has("stabilization_mode")) {
    v->stabilization_mode =
        SbirsStabilizationModeFromString(j["stabilization_mode"].AsString());
  }
  const examples::JsonValue& limits = j["sensor_scan_limits_deg"];
  if (!limits.IsNull() && limits.type() == examples::JsonValue::kArray && limits.Size() >= 4) {
    v->sensor_scan_limits_deg.az_min_deg =
        static_cast<float>(limits[static_cast<std::size_t>(0)].AsDouble());
    v->sensor_scan_limits_deg.az_max_deg =
        static_cast<float>(limits[static_cast<std::size_t>(1)].AsDouble());
    v->sensor_scan_limits_deg.el_min_deg =
        static_cast<float>(limits[static_cast<std::size_t>(2)].AsDouble());
    v->sensor_scan_limits_deg.el_max_deg =
        static_cast<float>(limits[static_cast<std::size_t>(3)].AsDouble());
  }
}

inline void LoadSbirsHardware(const examples::JsonValue& j,
                              sbirs_sensor::config::SbirsHardwareConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("wavelength_lower_um")) {
    v->wavelength_lower_um = static_cast<float>(j["wavelength_lower_um"].AsDouble());
  }
  if (j.Has("wavelength_upper_um")) {
    v->wavelength_upper_um = static_cast<float>(j["wavelength_upper_um"].AsDouble());
  }
  if (j.Has("optical_aperture_m")) {
    v->optical_aperture_m = static_cast<float>(j["optical_aperture_m"].AsDouble());
  }
  if (j.Has("optical_transmission")) {
    v->optical_transmission = static_cast<float>(j["optical_transmission"].AsDouble());
  }
  if (j.Has("detector_quantum_efficiency")) {
    v->detector_quantum_efficiency =
        static_cast<float>(j["detector_quantum_efficiency"].AsDouble());
  }
  if (j.Has("integration_time_sec")) {
    v->integration_time_sec = static_cast<float>(j["integration_time_sec"].AsDouble());
  }
  if (j.Has("noise_equivalent_power_w")) {
    v->noise_equivalent_power_w =
        static_cast<float>(j["noise_equivalent_power_w"].AsDouble());
  }
  if (j.Has("background_radiance_w_sr_m2")) {
    v->background_radiance_w_sr_m2 =
        static_cast<float>(j["background_radiance_w_sr_m2"].AsDouble());
  }
  if (j.Has("detector_temperature_k")) {
    v->detector_temperature_k =
        static_cast<float>(j["detector_temperature_k"].AsDouble());
  }
  if (j.Has("readout_noise_rms_w")) {
    v->readout_noise_rms_w = static_cast<float>(j["readout_noise_rms_w"].AsDouble());
  }
  // 焦平面几何（可选；仅 [SbirsAccept] 验收日志的脱靶量映射消费，缺省保持
  // 库默认 2.0 m / 30 μm——非必填字段用 Has 门控，避免旧 JSON 缺字段被置 0
  // 触发库校验 kFocalPlaneConfigNotPositive）。
  if (j.Has("focal_length_m")) {
    v->focal_length_m = static_cast<float>(j["focal_length_m"].AsDouble());
  }
  if (j.Has("detector_pixel_pitch_m")) {
    v->detector_pixel_pitch_m =
        static_cast<float>(j["detector_pixel_pitch_m"].AsDouble());
  }
}

inline void LoadSbirsMission(const examples::JsonValue& j,
                             sbirs_sensor::config::SbirsMissionConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("work_mode")) {
    v->work_mode = SbirsWorkModeFromString(j["work_mode"].AsString());
  }
  if (j.Has("wide_field_fov_az_deg")) {
    v->wide_field_fov_az_deg = static_cast<float>(j["wide_field_fov_az_deg"].AsDouble());
  }
  if (j.Has("wide_field_fov_el_deg")) {
    v->wide_field_fov_el_deg = static_cast<float>(j["wide_field_fov_el_deg"].AsDouble());
  }
  if (j.Has("narrow_field_fov_az_deg")) {
    v->narrow_field_fov_az_deg =
        static_cast<float>(j["narrow_field_fov_az_deg"].AsDouble());
  }
  if (j.Has("narrow_field_fov_el_deg")) {
    v->narrow_field_fov_el_deg =
        static_cast<float>(j["narrow_field_fov_el_deg"].AsDouble());
  }
  if (j.Has("scan_start_az_deg")) {
    v->scan_start_az_deg = static_cast<float>(j["scan_start_az_deg"].AsDouble());
  }
  if (j.Has("scan_span_deg")) {
    v->scan_span_deg = static_cast<float>(j["scan_span_deg"].AsDouble());
  }
  if (j.Has("initial_scan_phase_deg")) {
    v->initial_scan_phase_deg = static_cast<float>(j["initial_scan_phase_deg"].AsDouble());
  }
  if (j.Has("scan_direction")) {
    v->scan_direction = SbirsScanDirectionFromString(j["scan_direction"].AsString());
  }
  if (j.Has("scan_azimuth_reference")) {
    v->scan_azimuth_reference =
        SbirsScanAzimuthReferenceFromString(j["scan_azimuth_reference"].AsString());
  }
  if (j.Has("scan_center_el_deg")) {
    v->scan_center_el_deg = static_cast<float>(j["scan_center_el_deg"].AsDouble());
  }
  if (j.Has("scan_el_start_deg")) {
    v->scan_el_start_deg = static_cast<float>(j["scan_el_start_deg"].AsDouble());
  }
  if (j.Has("scan_el_span_deg")) {
    v->scan_el_span_deg = static_cast<float>(j["scan_el_span_deg"].AsDouble());
  }
  if (j.Has("scan_el_step_deg")) {
    v->scan_el_step_deg = static_cast<float>(j["scan_el_step_deg"].AsDouble());
  }
  if (j.Has("scan_rate_deg_per_sec")) {
    v->scan_rate_deg_per_sec = static_cast<float>(j["scan_rate_deg_per_sec"].AsDouble());
  }
  if (j.Has("min_range_m")) {
    v->min_range_m = static_cast<float>(j["min_range_m"].AsDouble());
  }
  if (j.Has("max_range_m")) {
    v->max_range_m = static_cast<float>(j["max_range_m"].AsDouble());
  }
  if (j.Has("frame_rate_hz")) {
    v->frame_rate_hz = static_cast<float>(j["frame_rate_hz"].AsDouble());
  }
  if (j.Has("narrow_cue_latency_s")) {
    v->narrow_cue_latency_s = static_cast<float>(j["narrow_cue_latency_s"].AsDouble());
  }
  if (j.Has("narrow_pointing_settle_error_deg")) {
    v->narrow_pointing_settle_error_deg =
        static_cast<float>(j["narrow_pointing_settle_error_deg"].AsDouble());
  }
  if (j.Has("narrow_pointing_max_slew_rate_deg_per_sec")) {
    v->narrow_pointing_max_slew_rate_deg_per_sec =
        static_cast<float>(j["narrow_pointing_max_slew_rate_deg_per_sec"].AsDouble());
  }
  if (j.Has("narrow_pointing_settle_tolerance_deg")) {
    v->narrow_pointing_settle_tolerance_deg =
        static_cast<float>(j["narrow_pointing_settle_tolerance_deg"].AsDouble());
  }
}

inline void LoadSbirsDetection(const examples::JsonValue& j,
                               sbirs_sensor::config::SbirsDetectionPolicyConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("wide_min_snr_linear")) {
    v->wide_min_snr_linear = static_cast<float>(j["wide_min_snr_linear"].AsDouble());
  }
  if (j.Has("narrow_min_snr_linear")) {
    v->narrow_min_snr_linear = static_cast<float>(j["narrow_min_snr_linear"].AsDouble());
  }
}

inline void LoadSbirsErrorModel(const examples::JsonValue& j,
                                sbirs_sensor::config::SbirsErrorModelConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("range_fraction_sigma")) {
    v->range_fraction_sigma =
        static_cast<float>(j["range_fraction_sigma"].AsDouble());
  }
  if (j.Has("random_seed")) {
    v->random_seed = static_cast<std::uint32_t>(j["random_seed"].AsInt());
  }
  if (j.Has("orbit_sigma_deg")) {
    v->orbit_sigma_deg = static_cast<float>(j["orbit_sigma_deg"].AsDouble());
  }
  if (j.Has("attitude_sigma_deg")) {
    v->attitude_sigma_deg = static_cast<float>(j["attitude_sigma_deg"].AsDouble());
  }
  if (j.Has("fov_sigma_deg")) {
    v->fov_sigma_deg = static_cast<float>(j["fov_sigma_deg"].AsDouble());
  }
  if (j.Has("detector_bandwidth_hz")) {
    v->detector_bandwidth_hz = static_cast<float>(j["detector_bandwidth_hz"].AsDouble());
  }
  if (j.Has("nav_position_sigma_m")) {
    v->nav_position_sigma_m = static_cast<float>(j["nav_position_sigma_m"].AsDouble());
  }
}

inline void LoadSbirsPointingDisturbance(
    const examples::JsonValue& j,
    sbirs_sensor::config::SbirsPointingDisturbanceConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("common_attitude_sigma_deg")) {
    v->common_attitude_sigma_deg =
        static_cast<float>(j["common_attitude_sigma_deg"].AsDouble());
  }
  if (j.Has("common_attitude_correlation_time_s")) {
    v->common_attitude_correlation_time_s =
        static_cast<float>(j["common_attitude_correlation_time_s"].AsDouble());
  }
  if (j.Has("channel_pointing_sigma_deg")) {
    v->channel_pointing_sigma_deg =
        static_cast<float>(j["channel_pointing_sigma_deg"].AsDouble());
  }
  if (j.Has("channel_pointing_correlation_time_s")) {
    v->channel_pointing_correlation_time_s =
        static_cast<float>(j["channel_pointing_correlation_time_s"].AsDouble());
  }
  if (j.Has("channel_vibration_amplitude_deg")) {
    v->channel_vibration_amplitude_deg =
        static_cast<float>(j["channel_vibration_amplitude_deg"].AsDouble());
  }
  if (j.Has("channel_vibration_frequency_hz")) {
    v->channel_vibration_frequency_hz =
        static_cast<float>(j["channel_vibration_frequency_hz"].AsDouble());
  }
  if (j.Has("random_seed")) {
    v->random_seed = static_cast<std::uint32_t>(j["random_seed"].AsInt());
  }
}

inline void LoadSbirsScheduler(const examples::JsonValue& j,
                               sbirs_sensor::config::SbirsSchedulerConfig* v) {
  if (j.IsNull()) return;
  // max_concurrent_nfov_locks 已删除（2026-09-02 单镜筒化）：一颗星只有一个窄场镜筒，
  // 可同时保持的精跟条数由分时轮转物理涌现，不再可配置；残留键按本装载器忽略
  // 未知键的既有策略静默跳过（多星场景装载器对残留键显式拒绝并给出错误）。
  // 宽→窄切换连续命中门（可选；缺省保持库默认 1 = 单次命中即调度，与既有
  // 行为逐位一致）。
  if (j.Has("wide_to_narrow_required_consecutive_hits")) {
    v->wide_to_narrow_required_consecutive_hits =
        static_cast<int>(j["wide_to_narrow_required_consecutive_hits"].AsInt());
  }
}

inline void LoadSbirsTracking(const examples::JsonValue& j,
                              sbirs_sensor::config::SbirsTrackingConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("tracking_mode")) {
    v->tracking_mode = SbirsTrackingModeFromString(j["tracking_mode"].AsString());
  }
  if (j.Has("estimated_backend")) {
    v->estimated_backend =
        SbirsEstimatedTrackingBackendFromString(j["estimated_backend"].AsString());
  }
  if (j.Has("process_noise_diff_coeff")) {
    v->process_noise_diff_coeff =
        static_cast<float>(j["process_noise_diff_coeff"].AsDouble());
  }
  if (j.Has("initial_position_std_m")) {
    v->initial_position_std_m =
        static_cast<float>(j["initial_position_std_m"].AsDouble());
  }
  if (j.Has("initial_velocity_std_m_per_s")) {
    v->initial_velocity_std_m_per_s =
        static_cast<float>(j["initial_velocity_std_m_per_s"].AsDouble());
  }
  if (j.Has("nis_gate_loss_cycles")) {
    v->nis_gate_loss_cycles =
        static_cast<unsigned int>(j["nis_gate_loss_cycles"].AsInt());
  }
  if (j.Has("nfov_tracking_gate_loss_cycles")) {
    v->nfov_tracking_gate_loss_cycles =
        static_cast<unsigned int>(j["nfov_tracking_gate_loss_cycles"].AsInt());
  }
  const examples::JsonValue& coeffs = j["imm_model_noise_diff_coeffs"];
  if (!coeffs.IsNull() && coeffs.type() == examples::JsonValue::kArray) {
    v->imm_model_noise_diff_coeffs.clear();
    for (std::size_t i = 0; i < coeffs.Size(); ++i) {
      v->imm_model_noise_diff_coeffs.push_back(
          static_cast<float>(coeffs[i].AsDouble()));
    }
  }
}

inline void LoadSbirsPolicy(const examples::JsonValue& j,
                            sbirs_sensor::config::SbirsPolicyConfig* v) {
  if (j.IsNull()) return;
  LoadSbirsDetection(j["detection"], &v->detection);
  LoadSbirsErrorModel(j["error_model"], &v->error_model);
  LoadSbirsPointingDisturbance(j["pointing_disturbance"], &v->pointing_disturbance);
  LoadSbirsScheduler(j["scheduler"], &v->scheduler);
  LoadSbirsTracking(j["tracking"], &v->tracking);
}

inline void LoadSbirsEnvironment(const examples::JsonValue& j,
                                 sbirs_sensor::config::SbirsEnvironmentConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("weather_type")) {
    v->weather_type = SbirsWeatherTypeFromString(j["weather_type"].AsString());
  }
  if (j.Has("sea_state")) {
    v->sea_state = SbirsSeaStateFromString(j["sea_state"].AsString());
  }
  if (j.Has("temperature_c")) {
    v->temperature_c = static_cast<float>(j["temperature_c"].AsDouble());
  }
  if (j.Has("relative_humidity_percent")) {
    v->relative_humidity_percent =
        static_cast<float>(j["relative_humidity_percent"].AsDouble());
  }
  if (j.Has("visibility_km")) {
    v->visibility_km = static_cast<float>(j["visibility_km"].AsDouble());
  }
  if (j.Has("base_atmospheric_transmittance")) {
    v->base_atmospheric_transmittance =
        static_cast<float>(j["base_atmospheric_transmittance"].AsDouble());
  }
  if (j.Has("humidity_visibility_interaction_weight")) {
    v->humidity_visibility_interaction_weight =
        static_cast<float>(j["humidity_visibility_interaction_weight"].AsDouble());
  }
  if (j.Has("rain_humidity_interaction_weight")) {
    v->rain_humidity_interaction_weight =
        static_cast<float>(j["rain_humidity_interaction_weight"].AsDouble());
  }
}

}  // namespace examples

#endif  // EXAMPLES_SBIRS_CONFIG_LOADER_DETAIL_H_
