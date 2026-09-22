/*
 * Supplemental architecture notes for acceptance review.
 * Unit: RirSessionConfigValidation.
 * Model: LRDR remote identification radar.
 * Layer: session.
 * File role: executable implementation.
 * Responsibility: public cycle contract, validation, adapters, recording, and replay.
 * Upstream boundary: external targets, platform position, RF scene, and session settings.
 * Downstream boundary: stable output frames, issues, task feedback, and replay records.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

/**
 * @file RirSessionConfigValidation.cpp
 * @brief 远程识别雷达会话配置校验实现。
 *
 * 校验逻辑为历史 AR 会话配置校验中的识别校验段（审计基线 96de367c）
 * 的平移改写：识别策略字段路径改为 `policy.recognition.*` 域内、识别任务
 * 作用距离/驻留字段四域归位至 `mission.*` 域内；issue code 改为
 * `rir.validation.recognition_*`（RirIssueCodes.h），语义与门限值不变。
 */

#include "1q/remote_identification_radar/config/RirSessionConfigValidation.h"

#include <cmath>

#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "common/numerics/Constants.h"
#include "common/radar/BeamwidthResolution.h"
#include "common/validation/ValidationUtils.h"
#include "remote_identification_radar/dwell/RirBeamControl.h"
#include "remote_identification_radar/internal/RirScanVolume.h"

namespace remote_identification_radar {
namespace config {

namespace {

bool IsFinite(float value) { return std::isfinite(value); }

void PushIssue(session::RirIssueList* issues, const char* code, const char* field,
               const std::string& message) {
  session::RirIssue issue;
  issue.severity = session::RirIssueSeverity::kError;
  issue.phase = session::RirIssuePhase::kInputValidation;
  issue.code = code;
  issue.field = field;
  issue.message = message;
  issues->push_back(issue);
}

/**
 * @brief 标称波束宽 ↔ 孔径交叉一致性（核查 1.1 可修部分）。
 * @note 波束宽与孔径互为因果（θ ≈ λ/L，与 DeriveBeamwidthFromApertureRad 同
 *       公式）；两数各自合法但互相矛盾时，扫描步长（用 nominal）与检测门/
 *       驻留门（λ/L 推导）口径分叉。仅当双边齐全（nominal>0、孔径>0、波长>0）
 *       才校验；阈值 50% 的依据：默认配置 4.0° vs 推导 4.77°（差 19%）与交付
 *       场景 0.4° vs 0.327°（差 18%）都必须放行，50% 只拦截真矛盾。
 */
void ValidateBeamwidthApertureConsistency(float nominal_beamwidth_deg, float aperture_m,
                                          float wavelength_m, const char* axis_field,
                                          session::RirIssueList* issues) {
  if (!(nominal_beamwidth_deg > 0.0f) || !(aperture_m > 0.0f) || !(wavelength_m > 0.0f)) {
    return;  // 双边不齐全不校验（单边配置由几何校验覆盖；非有限值比较自然为假）。
  }
  const float expected_deg =
      oneq::common::numerics::RadToDeg(oneq::common::radar::DeriveBeamwidthFromApertureRad(
          aperture_m, wavelength_m));
  if (expected_deg > 0.0f &&
      std::fabs(nominal_beamwidth_deg - expected_deg) / expected_deg > 0.5f) {
    PushIssue(issues, session::codes::kAntennaBeamwidthApertureInconsistent, axis_field,
              "Antenna nominal beamwidth must be consistent with the physical aperture "
              "(theta_deg ~= RadToDeg(wavelength / aperture), relative tolerance 50%).");
  }
}

void ValidateRirEnvironmentConfig(const RirEnvironmentConfig& environment,
                                  const char* field_prefix, session::RirIssueList* issues) {
  const std::string prefix = field_prefix == nullptr ? "" : std::string(field_prefix) + ".";
  if (!IsFinite(environment.weather_attenuation_db) || environment.weather_attenuation_db < 0.0f) {
    PushIssue(issues, session::codes::kInvalidEnvironmentSnapshot,
              (prefix + "weather_attenuation_db").c_str(),
              "Weather attenuation must be finite and non-negative.");
  }
  const RirVegetationCoverProfile cover = environment.vegetation_cover_profile;
  if (cover < RirVegetationCoverProfile::kDisabled ||
      cover > RirVegetationCoverProfile::kTropicalDense) {
    PushIssue(issues, session::codes::kInvalidEnvironmentSnapshot,
              (prefix + "vegetation_cover_profile").c_str(),
              "Vegetation cover profile must be a valid enum value.");
  }
  const RirAtmosphericPhysicsConfig& atmospheric = environment.atmospheric_physics;
  if (!IsFinite(atmospheric.pressure_hpa) || atmospheric.pressure_hpa <= 0.0f ||
      !IsFinite(atmospheric.temperature_k) || atmospheric.temperature_k <= 0.0f ||
      !IsFinite(atmospheric.relative_humidity) || atmospheric.relative_humidity < 0.0f ||
      atmospheric.relative_humidity > 1.0f) {
    PushIssue(issues, session::codes::kInvalidEnvironmentSnapshot,
              (prefix + "atmospheric_physics").c_str(),
              "Atmospheric observation must be finite with pressure/temperature > 0 and "
              "relative humidity in [0, 1].");
  }
}

void ValidateRirHardwareConfig(const RirHardwareConfig& hardware, session::RirIssueList* issues) {
  const hardware::RirTransmitterConfig& transmitter = hardware.transmitter;
  const hardware::RirReceiverConfig& receiver = hardware.receiver;
  const hardware::RirAntennaConfig& antenna = hardware.antenna;
  const hardware::RirRcsPhysicsConfig& rcs_physics = hardware.rcs_physics;
  const float transmitter_frequency_hz = transmitter.frequency_hz;

  if (!oneq::common::validation::IsFinite(transmitter_frequency_hz) ||
      transmitter_frequency_hz <= 0.0f) {
    PushIssue(issues, session::codes::kTransmitterFrequencyInvalid,
              "hardware.transmitter.frequency_hz",
              "Transmitter frequency must be finite and positive.");
  }
  const double duty_cycle =
      static_cast<double>(transmitter.pulse_width_s) * static_cast<double>(transmitter.prf_hz);
  if (!oneq::common::validation::IsFinite(transmitter.peak_power_w) ||
      transmitter.peak_power_w <= 0.0f ||
      !oneq::common::validation::IsFinite(transmitter.pulse_width_s) ||
      transmitter.pulse_width_s <= 0.0f || !oneq::common::validation::IsFinite(transmitter.prf_hz) ||
      transmitter.prf_hz <= 0.0f || !IsFinite(static_cast<float>(duty_cycle)) || duty_cycle <= 0.0 ||
      duty_cycle > 1.0 || !IsFinite(transmitter.transmit_loss_db) ||
      transmitter.transmit_loss_db < 0.0f) {
    PushIssue(issues, session::codes::kTransmitterOperatingEnvelopeInvalid, "hardware.transmitter",
              "Transmitter peak power, pulse width, PRF and duty cycle must be valid.");
  }
  if (!oneq::common::validation::IsFinite(receiver.cross_polarization_isolation_db) ||
      receiver.cross_polarization_isolation_db < 0.0f ||
      !oneq::common::validation::IsFinite(receiver.maximum_linear_input_power_w) ||
      receiver.maximum_linear_input_power_w <= 0.0f) {
    PushIssue(issues, session::codes::kReceiverRfHardwareInvalid, "hardware.receiver",
              "Receiver RF isolation and linear input limit must be valid.");
  }

  const auto axis_geometry_valid = [transmitter_frequency_hz](float nominal_beamwidth_deg,
                                                              float aperture_m) {
    if (!oneq::common::validation::IsFinite(nominal_beamwidth_deg) ||
        !oneq::common::validation::IsFinite(aperture_m) || nominal_beamwidth_deg < 0.0f ||
        aperture_m < 0.0f) {
      return false;
    }
    if (nominal_beamwidth_deg > 0.0f) {
      return true;
    }
    return aperture_m > 0.0f && oneq::common::validation::IsFinite(transmitter_frequency_hz) &&
           transmitter_frequency_hz > 0.0f;
  };

  if (!axis_geometry_valid(antenna.nominal_az_beamwidth_deg, antenna.antenna_length_m)) {
    PushIssue(issues, session::codes::kAntennaAzGeometryInvalid,
              "hardware.antenna.nominal_az_beamwidth_deg / antenna_length_m",
              "Azimuth beamwidth requires a positive nominal value or a valid physical aperture.");
  }
  if (!axis_geometry_valid(antenna.nominal_el_beamwidth_deg, antenna.antenna_width_m)) {
    PushIssue(issues, session::codes::kAntennaElGeometryInvalid,
              "hardware.antenna.nominal_el_beamwidth_deg / antenna_width_m",
              "Elevation beamwidth requires a positive nominal value or a valid physical aperture.");
  }

  // 增益-波束宽度物理一致性（2026-08-29 还债 B5）：增益与波束宽度是一枚硬币的
  // 两面（经验式 G_dBi ≈ 10·log10(26000/(az°·el°))，η≈0.5~0.7），各自合法但互相
  // 矛盾的天线会让回波预算虚高（52 dBi 配 4° 波束曾虚高 ~40 dB）。名义波束宽度
  // 均为正时校验，容差 3 dB；走孔径推导的配置不在此校验（几何校验已覆盖孔径）。
  if (antenna.nominal_az_beamwidth_deg > 0.0f && antenna.nominal_el_beamwidth_deg > 0.0f) {
    const float beamwidth_product_deg2 =
        antenna.nominal_az_beamwidth_deg * antenna.nominal_el_beamwidth_deg;
    if (IsFinite(beamwidth_product_deg2) && beamwidth_product_deg2 > 0.0f &&
        IsFinite(antenna.main_beam_gain_db)) {
      const float consistent_gain_dbi = 10.0f * std::log10(26000.0f / beamwidth_product_deg2);
      if (std::fabs(antenna.main_beam_gain_db - consistent_gain_dbi) > 3.0f) {
        PushIssue(issues, session::codes::kAntennaGainBeamwidthInconsistent,
                  "hardware.antenna.main_beam_gain_db / nominal_{az,el}_beamwidth_deg",
                  "Antenna peak gain must be physically consistent with the nominal beamwidths "
                  "(G_dBi ~= 10*log10(26000/(az_deg*el_deg)), tolerance 3 dB).");
      }
    }
  }

  // 标称波束宽 ↔ 孔径交叉一致性：波长 λ = c/f（与 RirSession 扫描波位、
  // RirController 检测门的孔径推导同源同口径）；az/el 两轴共用同一 issue code。
  if (oneq::common::validation::IsFinite(transmitter_frequency_hz) &&
      transmitter_frequency_hz > 0.0f) {
    const float wavelength_m =
        static_cast<float>(oneq::common::numerics::kLightSpeed) / transmitter_frequency_hz;
    ValidateBeamwidthApertureConsistency(
        antenna.nominal_az_beamwidth_deg, antenna.antenna_length_m, wavelength_m,
        "hardware.antenna.nominal_az_beamwidth_deg / antenna_length_m", issues);
    ValidateBeamwidthApertureConsistency(
        antenna.nominal_el_beamwidth_deg, antenna.antenna_width_m, wavelength_m,
        "hardware.antenna.nominal_el_beamwidth_deg / antenna_width_m", issues);
  }

  if (!oneq::common::validation::IsFinite(rcs_physics.physics_mix_ratio) ||
      rcs_physics.physics_mix_ratio < 0.0f || rcs_physics.physics_mix_ratio > 1.0f ||
      !oneq::common::validation::IsFinite(rcs_physics.cylinder_weight) ||
      rcs_physics.cylinder_weight < 0.0f || rcs_physics.cylinder_weight > 1.0f ||
      !oneq::common::validation::IsFinite(rcs_physics.min_equivalent_radius_m) ||
      !oneq::common::validation::IsFinite(rcs_physics.max_equivalent_radius_m) ||
      rcs_physics.min_equivalent_radius_m <= 0.0f ||
      rcs_physics.max_equivalent_radius_m < rcs_physics.min_equivalent_radius_m ||
      !oneq::common::validation::IsFinite(rcs_physics.min_rcs_m2) ||
      !oneq::common::validation::IsFinite(rcs_physics.max_rcs_m2) ||
      rcs_physics.min_rcs_m2 < 0.0f || rcs_physics.max_rcs_m2 < rcs_physics.min_rcs_m2 ||
      !oneq::common::validation::IsFinite(rcs_physics.bistatic_psi_offset_deg)) {
    PushIssue(issues, session::codes::kRcsPhysicsInvalid, "hardware.rcs_physics",
              "RCS physics parameters must be finite with ordered radius and RCS bounds.");
  }
}

}  // namespace

session::RirIssueList ValidateRirSessionConfig(const RirSessionConfig& config) {
  session::RirIssueList issues;
  const RirMissionConfig& mission = config.mission;
  const RirRecognitionPolicy& recognition = config.policy.recognition;
  const RirRecognitionFeatureWeights& weights = recognition.feature_weights;

  // 权重：有限、[0, 1]、四者之和为 1.0。
  const float weight_sum = weights.rcs_weight + weights.motion_weight +
                           weights.polarization_weight + weights.range_profile_weight;
  if (!IsFinite(weights.rcs_weight) || !IsFinite(weights.motion_weight) ||
      !IsFinite(weights.polarization_weight) || !IsFinite(weights.range_profile_weight) ||
      weights.rcs_weight < 0.0f || weights.rcs_weight > 1.0f || weights.motion_weight < 0.0f ||
      weights.motion_weight > 1.0f || weights.polarization_weight < 0.0f ||
      weights.polarization_weight > 1.0f || weights.range_profile_weight < 0.0f ||
      weights.range_profile_weight > 1.0f || std::fabs(weight_sum - 1.0f) > 1.0e-5f) {
    PushIssue(&issues, session::codes::kRecognitionWeightsInvalid,
              "policy.recognition.feature_weights",
              "Recognition feature weights must be finite values in [0, 1] summing to 1.0.");
  }

  // 数据库路径：启用识别时必须非空。
  if (recognition.enabled && recognition.database_path.empty()) {
    PushIssue(&issues, session::codes::kRecognitionDatabasePathMissing,
              "policy.recognition.database_path",
              "Recognition database path must be non-empty when recognition is enabled.");
  }

  // 判定门限：[0, 1]。
  if (!IsFinite(recognition.acceptance_score) || recognition.acceptance_score < 0.0f ||
      recognition.acceptance_score > 1.0f || !IsFinite(recognition.minimum_margin) ||
      recognition.minimum_margin < 0.0f || recognition.minimum_margin > 1.0f) {
    PushIssue(&issues, session::codes::kRecognitionThresholdInvalid,
              "policy.recognition.acceptance_score / minimum_margin",
              "Recognition acceptance score and minimum margin must be finite values in [0, 1].");
  }

  // 积累计数：至少为 1。
  if (recognition.min_confirmed_hits == 0U || recognition.min_observation_count == 0U) {
    PushIssue(&issues, session::codes::kRecognitionAccumulationInvalid,
              "policy.recognition.min_confirmed_hits / min_observation_count",
              "Recognition accumulation counts must be at least 1.");
  }

  // 识别策略时间范围：保持时间非负；累积窗口有限且为正。
  if (!IsFinite(recognition.result_hold_sec) || recognition.result_hold_sec < 0.0f ||
      !IsFinite(recognition.accumulation_window_sec) ||
      recognition.accumulation_window_sec <= 0.0f) {
    PushIssue(&issues, session::codes::kRecognitionTimeRangeInvalid,
              "policy.recognition.result_hold_sec / accumulation_window_sec",
              "Recognition hold time must be non-negative and accumulation window must be "
              "finite and positive.");
  }

  // 任务域识别作用范围/驻留：最大距离与驻留时间有限且为正（四域归位后）。
  if (!IsFinite(mission.max_range_m) || mission.max_range_m <= 0.0f ||
      !IsFinite(mission.recognition_dwell_sec) || mission.recognition_dwell_sec <= 0.0f) {
    PushIssue(&issues, session::codes::kRecognitionTimeRangeInvalid,
              "mission.max_range_m / recognition_dwell_sec",
              "Recognition max range and dwell must be finite and positive.");
  }

  // 扫描策略（库内驻留调度器）：步长系数有限且为正。
  if (!IsFinite(mission.step_scale) || mission.step_scale <= 0.0f) {
    PushIssue(&issues, session::codes::kScanStrategyInvalid, "mission.step_scale",
              "Scan step scale must be finite and positive.");
  }

  // 任务扫描子窗（用户指定作战搜索扇区）：az 相对域 [-180,180]、el 绝对域 [-90,90]，
  // 且逐轴有序（min ≤ max）。缺省无界满足约束；实际搜索扇区为其与体积的交集。
  {
    const RirAzimuthElevationLimitsDeg& window = mission.scan_window_deg;
    if (!IsFinite(window.az_min_deg) || !IsFinite(window.az_max_deg) ||
        !IsFinite(window.el_min_deg) || !IsFinite(window.el_max_deg) ||
        window.az_min_deg > window.az_max_deg || window.el_min_deg > window.el_max_deg ||
        window.az_min_deg < -180.0f || window.az_max_deg > 180.0f ||
        window.el_min_deg < -90.0f || window.el_max_deg > 90.0f) {
      PushIssue(&issues, session::codes::kScanStrategyInvalid, "mission.scan_window_deg",
                "Mission scan window limits must be finite, ordered, az in [-180,180] relative "
                "domain and el in [-90,90].");
    }
  }

  // 可扫描体积（orientation 第五域）：az 相对域 [-180,180]、el 绝对域 [-90,90]。
  {
    const RirAzimuthElevationLimitsDeg& volume = config.orientation;
    if (!IsFinite(volume.az_min_deg) || !IsFinite(volume.az_max_deg) ||
        !IsFinite(volume.el_min_deg) || !IsFinite(volume.el_max_deg) ||
        volume.az_min_deg > volume.az_max_deg || volume.el_min_deg > volume.el_max_deg ||
        volume.az_min_deg < -180.0f || volume.az_max_deg > 180.0f ||
        volume.el_min_deg < -90.0f || volume.el_max_deg > 90.0f) {
      PushIssue(&issues, session::codes::kSteerableVolumeInvalid,
                "orientation",
                "Steerable volume limits must be finite, ordered, az in [-180,180] relative "
                "domain and el in [-90,90].");
    }
  }

  // 转台朝向（mission.scan_center_deg）：有限且在合法域。
  {
    const RirAzimuthElevationDeg& center = mission.scan_center_deg;
    if (!IsFinite(center.az_deg) || !IsFinite(center.el_deg) || center.az_deg < -180.0f ||
        center.az_deg > 180.0f || center.el_deg < -90.0f || center.el_deg > 90.0f) {
      PushIssue(&issues, session::codes::kScanCenterInvalid, "mission.scan_center_deg",
                "Scan center must be finite with az in [-180,180] and el in [-90,90].");
    }
  }

  // 扫描波位单轴采样数上限（核查 2.1）：与 RirSession BuildAbsoluteScanWaves
  // 同口径估算每轴波位数——扇区 = 任务扫描子窗 ∩ 硬件可扫描体积，步长 =
  // 有效波束宽（nominal → λ/L 孔径推导，波长 λ = c/f）× step_scale。任一轴
  // 期望采样数超过扫描内核上限（BuildScanPattern 每轴 4096，超出静默截断）
  // 即在配置期拦截，提示加大步长或缩小扇区；空扇区/零步长由其他校验负责。
  {
    // 与 common/radar/ScanScheduleRuntime.h BuildScanPattern 的 kMaxAxisSamples 同值。
    constexpr double kMaxScanAxisSamples = 4096.0;
    const float frequency_hz = config.hardware.transmitter.frequency_hz;
    const float wavelength_m =
        IsFinite(frequency_hz) && frequency_hz > 0.0f
            ? static_cast<float>(oneq::common::numerics::kLightSpeed) / frequency_hz
            : 0.0f;
    const dwell::RirEffectiveBeamwidthDeg beamwidth =
        dwell::RirResolveEffectiveBeamwidth(config.hardware.antenna, wavelength_m);
    const float az_step_deg = beamwidth.az_beamwidth_deg * mission.step_scale;
    const float el_step_deg = beamwidth.el_beamwidth_deg * mission.step_scale;
    const RirAzimuthElevationLimitsDeg sector =
        internal::IntersectScanSector(mission.scan_window_deg, config.orientation);
    const auto axis_samples_exceeds = [kMaxScanAxisSamples](float min_deg, float max_deg,
                                                            float step_deg) {
      if (!(min_deg <= max_deg) || !(step_deg > 0.0f)) {
        return false;
      }
      // 与内核 BuildScanPattern 的步进边界同口径（循环条件含 0.5·step 半步容差），
      // 避免在半步边界上比内核早拦一格。
      const double samples = std::floor((max_deg - min_deg) / step_deg + 0.5) + 1.0;
      return samples > kMaxScanAxisSamples;
    };
    if (axis_samples_exceeds(sector.az_min_deg, sector.az_max_deg, az_step_deg) ||
        axis_samples_exceeds(sector.el_min_deg, sector.el_max_deg, el_step_deg)) {
      PushIssue(&issues, session::codes::kScanWaveAxisSamplesTruncated,
                "mission.scan_window_deg / orientation / step_scale",
                "Scan wave samples per axis would exceed the kernel limit of 4096 and the "
                "pattern gets silently truncated; enlarge the step or narrow the sector.");
    }
  }

  // 自持检测策略（阶段 2-S）：Pfa/SNR 门限有限且范围合理，脉冲数/种子为正。
  {
    const RirDetectionPolicyConfig& detection = config.policy.detection;
    if (!IsFinite(detection.cfar_pfa) || detection.cfar_pfa <= 0.0f || detection.cfar_pfa > 1.0f ||
        !IsFinite(detection.min_snr_db) || !IsFinite(detection.min_detection_margin_db) ||
        detection.pulse_count <= 0 || detection.random_seed == 0U) {
      PushIssue(&issues, session::codes::kDetectionPolicyInvalid, "policy.detection",
                "Detection policy Pfa/SNR gates must be finite, pulse count and seed positive.");
    }
  }

  // 关联策略：波门 sigma 有限且 > 0。
  {
    const RirAssociationPolicyConfig& association = config.policy.association;
    if (!IsFinite(association.distance_gate_sigma) || association.distance_gate_sigma <= 0.0f) {
      PushIssue(&issues, session::codes::kAssociationPolicyInvalid, "policy.association",
                "Association distance gate sigma must be finite and positive.");
    }
  }

  // 跟踪策略：KF 过程噪声参数有限且 > 0。
  {
    const RirTrackingPolicyConfig& tracking = config.policy.tracking;
    if (!IsFinite(tracking.kalman_noise_diff_coeff) || tracking.kalman_noise_diff_coeff <= 0.0f) {
      PushIssue(&issues, session::codes::kTrackingPolicyInvalid, "policy.tracking",
                "Kalman process noise parameter must be finite and positive.");
    }
  }

  // 生命周期策略：confirm_hits/max_lost_cycles 至少为 1。
  {
    const RirLifecyclePolicyConfig& lifecycle = config.policy.lifecycle;
    if (lifecycle.confirm_hits == 0U || lifecycle.max_lost_cycles == 0U) {
      PushIssue(&issues, session::codes::kLifecyclePolicyInvalid, "policy.lifecycle",
                "Lifecycle confirm hits and max lost cycles must be at least 1.");
    }
  }

  // 信号处理增益偏置（阶段 2-M M3）：有限且 [0, 40] dB。
  {
    const RirHardwareConfig& hardware = config.hardware;
    const hardware::RirSignalProcessingConfig& gains = hardware.signal_processing;
    if (!IsFinite(gains.target_processing_gain_db) || !IsFinite(gains.noise_processing_gain_db) ||
        !IsFinite(gains.clutter_suppression_gain_db) ||
        !IsFinite(gains.jamming_suppression_gain_db) || gains.target_processing_gain_db < 0.0f ||
        gains.target_processing_gain_db > 40.0f || gains.noise_processing_gain_db < 0.0f ||
        gains.noise_processing_gain_db > 40.0f || gains.clutter_suppression_gain_db < 0.0f ||
        gains.clutter_suppression_gain_db > 40.0f || gains.jamming_suppression_gain_db < 0.0f ||
        gains.jamming_suppression_gain_db > 40.0f) {
      PushIssue(&issues, session::codes::kSignalProcessingGainsInvalid,
                "hardware.signal_processing",
                "Signal processing gain offsets must be finite values in [0, 40] dB.");
    }
  }

  ValidateRirHardwareConfig(config.hardware, &issues);

  ValidateRirEnvironmentConfig(config.environment, "environment", &issues);

  return issues;
}

}  // namespace config
}  // namespace remote_identification_radar
