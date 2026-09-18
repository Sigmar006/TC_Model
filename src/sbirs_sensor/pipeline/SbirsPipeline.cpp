#include "sbirs_sensor/pipeline/SbirsPipeline.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <string>

#include "common/geometry/EarthOccultation.h"
#include "common/logging/AcceptanceText.h"
#include "common/logging/ProjectLog.h"
#include "common/numerics/Constants.h"
#include "common/radar/RadarEquations.h"
#include "1q/coordinate/inertial_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/sbirs_sensor/session/SbirsIssueCodes.h"
#include "sbirs_sensor/environment/SbirsEnvironmentModel.h"
#include "sbirs_sensor/foundation/SbirsErrorModel.h"
#include "sbirs_sensor/foundation/SbirsGeometry.h"
#include "sbirs_sensor/foundation/SbirsNoiseModel.h"
#include "sbirs_sensor/foundation/SbirsRadiometry.h"
#include "sbirs_sensor/pipeline/SbirsAcceptanceLog.h"
#include "sbirs_sensor/pipeline/SbirsAcceptanceRecords.h"
#include "sbirs_sensor/pipeline/SbirsBoresightChain.h"
#include "sbirs_sensor/pipeline/SbirsEciScene.h"
#include "sbirs_sensor/pipeline/SbirsNfovAcquisition.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

const double kEarthRadiusM = oneq::common::geometry::kMeanEarthRadiusM;

// 规则 13b：正常执行周期按目标门控排除的 kInfo 诊断码（不属于三写，仅承载排查信息）。
// code 引用 SbirsIssueCodes.h 注册表常量。

/// 构造 kInfo 级按目标排除诊断（不属于三写，仅承载排查信息；规则 13b）。
/// @param cause 门内归因（规则 13b 门内归因条款）；聚合门排除须给出主因，具体门可 kNone。
/// @param target_index 场景目标索引；非负时写入 `location = {kSceneEntity, target_index}`，
///                     供跨周期差分记录器按实体关联消费（默认 -1 保持 kGlobal，向后兼容）。
session::SbirsIssue MakeExclusionIssue(const char* code, const std::string& message,
                                       session::SbirsIssueCause cause =
                                           session::SbirsIssueCause::kNone,
                                       std::ptrdiff_t target_index = -1) {
  session::SbirsIssue issue;
  issue.severity = session::SbirsIssueSeverity::kInfo;
  issue.code = code;
  issue.message = message;
  issue.cause = cause;
  if (target_index >= 0) {
    issue.location.kind = oneq::foundation::ValidationLocationKind::kSceneEntity;
    issue.location.entity_index = static_cast<std::size_t>(target_index);
  }
  return issue;
}

// 规则 13b 门内归因：WFOV SNR 门失败主因分类。SNR 门为聚合门（距离²/大气透过率/
// 目标签名折入单一门限），反事实判定主因——各因子取"达标参考值"后 SNR 提升（dB）
// 最大者：距离参考 1000 km、大气全透过、目标签名取"使 SNR 恰达门限的签名"
//（signature_required 仅依赖硬件/门限配置，调用方在循环外计算）；目标签名即
// 输入辐射强度 I_t（W/sr）。噪声为硬件常数，不参与单目标归因。全部损失 <= 0 时
// 返回 kUnknown。
session::SbirsIssueCause ClassifyWfovSnrExclusionCause(double range_m, float transmittance,
                                                       double signature_actual,
                                                       double signature_required) {
  constexpr double kReferenceRangeM = 1.0e6;
  const double distance_loss_db =
      20.0 * std::log10(std::max(range_m, 1.0) / kReferenceRangeM);
  const double attenuation_loss_db = -20.0 * std::log10(std::max(transmittance, 1.0e-6f));
  const double signature_loss_db =
      signature_actual > 0.0 && signature_required > 0.0
          ? 10.0 * std::log10(signature_required / signature_actual)
          : 0.0;
  session::SbirsIssueCause cause = session::SbirsIssueCause::kUnknown;
  double max_loss_db = 0.0;
  const struct {
    double loss_db;
    session::SbirsIssueCause cause;
  } kCandidates[] = {
      {distance_loss_db, session::SbirsIssueCause::kDistanceLimited},
      {attenuation_loss_db, session::SbirsIssueCause::kAttenuationLimited},
      {signature_loss_db, session::SbirsIssueCause::kSignatureLimited},
  };
  for (const auto& candidate : kCandidates) {
    if (candidate.loss_db > max_loss_db) {
      max_loss_db = candidate.loss_db;
      cause = candidate.cause;
    }
  }
  return max_loss_db > 0.0 ? cause : session::SbirsIssueCause::kUnknown;
}

std::string FormatFloat(float value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(value));
  return buffer;
}

std::string FormatSnr(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3e", value);
  return buffer;
}

std::uint32_t DeriveMeasurementSeed(std::uint32_t base_seed, std::uint32_t domain_tag) {
  std::uint32_t value = base_seed ^ domain_tag;
  value ^= value >> 16U;
  value *= 0x7feb352dU;
  value ^= value >> 15U;
  value *= 0x846ca68bU;
  value ^= value >> 16U;
  return value == 0U ? 1U : value;
}

const std::uint32_t kWfovMeasurementDomain = UINT32_C(0x57464f56);
const std::uint32_t kEstimatedMeasurementDomain = UINT32_C(0x4553544d);
const std::uint32_t kSensorLikeOutputDomain = UINT32_C(0x534c4f55);

bool IsTruthTrackingState(SbirsTargetState state) {
  return state == SbirsTargetState::kStrictTruthAssistedTracking ||
         state == SbirsTargetState::kSensorLikeTruthAssistedTracking;
}

attribution::SbirsTrackingSource TrackingSourceForState(SbirsTargetState state) {
  switch (state) {
    case SbirsTargetState::kEstimatedTracking:
      return attribution::SbirsTrackingSource::kEstimated;
    case SbirsTargetState::kStrictTruthAssistedTracking:
      return attribution::SbirsTrackingSource::kStrictTruthAssisted;
    case SbirsTargetState::kSensorLikeTruthAssistedTracking:
      return attribution::SbirsTrackingSource::kSensorLikeTruthAssisted;
    default:
      return attribution::SbirsTrackingSource::kNotApplicable;
  }
}

attribution::SbirsTrackingSource TrackingSourceForMode(config::SbirsTrackingMode mode) {
  switch (mode) {
    case config::SbirsTrackingMode::kEstimated:
      return attribution::SbirsTrackingSource::kEstimated;
    case config::SbirsTrackingMode::kStrictTruthAssisted:
      return attribution::SbirsTrackingSource::kStrictTruthAssisted;
    case config::SbirsTrackingMode::kSensorLikeTruthAssisted:
      return attribution::SbirsTrackingSource::kSensorLikeTruthAssisted;
    default:
      return attribution::SbirsTrackingSource::kNotApplicable;
  }
}

float NormalizeAzimuth(float azimuth_deg) {
  float result = std::fmod(azimuth_deg + 180.0f, 360.0f);
  if (result < 0.0f) {
    result += 360.0f;
  }
  return result - 180.0f;
}

float AzimuthDelta(float lhs_deg, float rhs_deg) { return NormalizeAzimuth(lhs_deg - rhs_deg); }

float PositiveModulo(float value, float period) {
  float result = std::fmod(value, period);
  if (result < 0.0f) {
    result += period;
  }
  return result;
}

// 2026-08 正式变更（ECI 输出）：内部角度量纲保持 deg、方位约定为对称
// (-180, 180]（AzimuthDelta 最短角差语义）；输出边界统一转换为 ECI 极坐标
// 弧度——az ∈ [0, 2π)、el ∈ [-π/2, π/2]（客户契约，见 session_contract.md
// §传感器方位坐标系约定）。el 越界由误差注入引起时钳制到有效域。
float WrapAzimuthPositive(float azimuth_deg) { return PositiveModulo(azimuth_deg, 360.0f); }

float ToEciAzimuthRad(float azimuth_deg) {
  return oneq::common::numerics::DegToRad(WrapAzimuthPositive(azimuth_deg));
}

float ToEciElevationRad(float elevation_deg) {
  const float clamped = std::max(-90.0f, std::min(90.0f, elevation_deg));
  return oneq::common::numerics::DegToRad(clamped);
}

// 方位基准说明（2026-08-31）：azimuth_base_deg 在 kEciAbsolute 下恒 0（历史行为
// 逐位不变）；kNadirRelative 下为星下点 ECI 方位（deg），由 Execute 按当周期卫星
// 位置现算并经成员传递（ApplyConfig 相位重锚复用最近一次的基准）。
// start_deg 为有效扫描起点（绝对模式 = scan_start_az_deg；nadir 模式 = 收敛裁剪后
// 的有效偏移）；direction_sign 为配置 scan_direction 的带符号方向。
// 往复式（2026-08-31）：回程由相位动态（行程折叠反射）体现，相位→方位映射在两腿
// 上同为 start + dir·phase——到边反向走，而不是符号翻转。
float ScanAzimuth(float start_deg, float phase_deg, float azimuth_base_deg,
                  float direction_sign) {
  return NormalizeAzimuth(azimuth_base_deg + start_deg + direction_sign * phase_deg);
}

float ScanPhaseForAzimuth(float start_deg, float azimuth_deg, float azimuth_base_deg,
                          bool increasing) {
  const float base_start_deg =
      azimuth_base_deg == 0.0f ? start_deg : NormalizeAzimuth(azimuth_base_deg + start_deg);
  if (increasing) {
    return PositiveModulo(azimuth_deg - base_start_deg, 360.0f);
  }
  return PositiveModulo(base_start_deg - azimuth_deg, 360.0f);
}

// 配置扫描方向 → 带符号方向号（+1 递增 / −1 递减）。
float ConfigDirectionSign(const config::SbirsMissionConfig& mission) {
  return mission.scan_direction == config::SbirsScanDirection::kIncreasingAzimuth ? 1.0f : -1.0f;
}

// 2-D 俯仰栅格（阶段 4）：span=0 默认单行模式（行数=1，行中心恒为 scan_center_el_deg，
// 既有行为逐位不变）；span>0 时行数 = 1 + floor(span/step)，行中心 = el_start + row·step。
// 配置校验已保证 span>=0、step>0。
int ScanRowCount(const config::SbirsMissionConfig& mission) {
  if (mission.scan_el_span_deg <= 0.0f) {
    return 1;
  }
  return 1 + static_cast<int>(
                 std::floor(mission.scan_el_span_deg / std::max(1.0e-6f, mission.scan_el_step_deg)));
}

float RowCenterEl(const config::SbirsMissionConfig& mission, int row_index) {
  if (mission.scan_el_span_deg <= 0.0f) {
    return mission.scan_center_el_deg;
  }
  return mission.scan_el_start_deg +
         static_cast<float>(row_index) * std::max(1.0e-6f, mission.scan_el_step_deg);
}

session::SbirsVector3M LosFromAzimuthElevation(float azimuth_deg, float elevation_deg) {
  const double azimuth_rad =
      oneq::common::numerics::DegToRad(static_cast<double>(azimuth_deg));
  const double elevation_rad =
      oneq::common::numerics::DegToRad(static_cast<double>(elevation_deg));
  const double horizontal = std::cos(elevation_rad);
  session::SbirsVector3M los;
  los.x = horizontal * std::cos(azimuth_rad);
  los.y = horizontal * std::sin(azimuth_rad);
  los.z = std::sin(elevation_rad);
  return los;
}

bool InRectangularFov(float target_az_deg, float target_el_deg, float center_az_deg,
                      float center_el_deg, float fov_az_deg, float fov_el_deg) {
  return std::fabs(AzimuthDelta(target_az_deg, center_az_deg)) <= 0.5f * fov_az_deg &&
         std::fabs(target_el_deg - center_el_deg) <= 0.5f * fov_el_deg;
}

struct GroundCoverageEnvelope {
  bool has_intersection = false;
  double min_lat_deg = 0.0;
  double max_lat_deg = 0.0;
  double min_lon_deg = 0.0;
  double max_lon_deg = 0.0;
};

GroundCoverageEnvelope ComputeGroundCoverageEnvelope(
    const session::SbirsVector3M& satellite_position_eci_m,
    const SbirsBoresightChain& boresight_chain,
    float actual_scan_azimuth_sensor_deg,
    float actual_scan_elevation_sensor_deg,
    float fov_az_deg,
    float fov_el_deg,
    double earth_radius_m,
    double gmst_rad) {
  GroundCoverageEnvelope envelope;
  std::vector<std::pair<double, double>> intersection_points;

  auto try_add_sensor_ray = [&](float az_sensor_deg, float el_sensor_deg) {
    const session::SbirsVector3M eci_los =
        boresight_chain.EciLosOfSensorPointing(az_sensor_deg, el_sensor_deg);
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    if (foundation::TryComputeGroundIntersectionLatLonDeg(
            satellite_position_eci_m, eci_los, earth_radius_m, gmst_rad, &lat_deg, &lon_deg)) {
      intersection_points.emplace_back(lat_deg, lon_deg);
    }
  };

  auto try_add_eci_ray = [&](const session::SbirsVector3M& eci_ray) {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    if (foundation::TryComputeGroundIntersectionLatLonDeg(
            satellite_position_eci_m, eci_ray, earth_radius_m, gmst_rad, &lat_deg, &lon_deg)) {
      intersection_points.emplace_back(lat_deg, lon_deg);
    }
  };

  // 1. 视场中心点
  try_add_sensor_ray(actual_scan_azimuth_sensor_deg, actual_scan_elevation_sensor_deg);

  // 2. 采样视场矩形 4 边缘
  const float half_az = 0.5f * fov_az_deg;
  const float half_el = 0.5f * fov_el_deg;
  constexpr int kEdgeSteps = 16;
  for (int i = 0; i <= kEdgeSteps; ++i) {
    const float frac = static_cast<float>(i) / static_cast<float>(kEdgeSteps);
    const float az = (actual_scan_azimuth_sensor_deg - half_az) + frac * (2.0f * half_az);
    try_add_sensor_ray(az, actual_scan_elevation_sensor_deg - half_el);
    try_add_sensor_ray(az, actual_scan_elevation_sensor_deg + half_el);
  }
  for (int i = 0; i <= kEdgeSteps; ++i) {
    const float frac = static_cast<float>(i) / static_cast<float>(kEdgeSteps);
    const float el = (actual_scan_elevation_sensor_deg - half_el) + frac * (2.0f * half_el);
    try_add_sensor_ray(actual_scan_azimuth_sensor_deg - half_az, el);
    try_add_sensor_ray(actual_scan_azimuth_sensor_deg + half_az, el);
  }

  // 3. 地平圈（Limb）与地心指向（解决全盘高下视场矩形外围脱靶但盘内全覆盖问题）
  const double sat_distance_m = foundation::Norm(satellite_position_eci_m);
  if (sat_distance_m > earth_radius_m) {
    const session::SbirsVector3M u_earth{
        -satellite_position_eci_m.x / sat_distance_m,
        -satellite_position_eci_m.y / sat_distance_m,
        -satellite_position_eci_m.z / sat_distance_m};

    // 检查星下点/地心是否在视场内
    float nadir_az_deg = 0.0f;
    float nadir_el_deg = 0.0f;
    boresight_chain.SensorAzElOfEciVector(u_earth, &nadir_az_deg, &nadir_el_deg);
    if (InRectangularFov(nadir_az_deg, nadir_el_deg, actual_scan_azimuth_sensor_deg,
                         actual_scan_elevation_sensor_deg, fov_az_deg, fov_el_deg)) {
      try_add_eci_ray(u_earth);
    }

    auto cross_product = [](const session::SbirsVector3M& a,
                            const session::SbirsVector3M& b) {
      session::SbirsVector3M r;
      r.x = a.y * b.z - a.z * b.y;
      r.y = a.z * b.x - a.x * b.z;
      r.z = a.x * b.y - a.y * b.x;
      return r;
    };

    session::SbirsVector3M temp{0.0, 0.0, 1.0};
    if (std::fabs(u_earth.z) > 0.9) {
      temp = session::SbirsVector3M{1.0, 0.0, 0.0};
    }
    session::SbirsVector3M p = cross_product(u_earth, temp);
    const double p_norm = foundation::Norm(p);
    if (p_norm > 0.0) {
      p.x /= p_norm;
      p.y /= p_norm;
      p.z /= p_norm;
      session::SbirsVector3M q = cross_product(u_earth, p);
      const double q_norm = foundation::Norm(q);
      if (q_norm > 0.0) {
        q.x /= q_norm;
        q.y /= q_norm;
        q.z /= q_norm;

        const double sin_rho = earth_radius_m / sat_distance_m;
        const double rho_rad = std::asin(std::min(1.0, sin_rho));
        // 取 0.999 * rho 避免切线掠射引起的数值求交失败
        const double sample_rho_rad = 0.999 * rho_rad;
        const double cos_rho = std::cos(sample_rho_rad);
        const double sin_sample_rho = std::sin(sample_rho_rad);

        constexpr int kLimbSteps = 72;
        for (int i = 0; i < kLimbSteps; ++i) {
          const double theta = (2.0 * oneq::common::numerics::kPi * i) / kLimbSteps;
          const double cos_th = std::cos(theta);
          const double sin_th = std::sin(theta);
          const session::SbirsVector3M limb_ray{
              cos_rho * u_earth.x + sin_sample_rho * (cos_th * p.x + sin_th * q.x),
              cos_rho * u_earth.y + sin_sample_rho * (cos_th * p.y + sin_th * q.y),
              cos_rho * u_earth.z + sin_sample_rho * (cos_th * p.z + sin_th * q.z)};

          float limb_az_sensor_deg = 0.0f;
          float limb_el_sensor_deg = 0.0f;
          boresight_chain.SensorAzElOfEciVector(limb_ray, &limb_az_sensor_deg,
                                                &limb_el_sensor_deg);
          if (InRectangularFov(limb_az_sensor_deg, limb_el_sensor_deg,
                               actual_scan_azimuth_sensor_deg, actual_scan_elevation_sensor_deg,
                               fov_az_deg, fov_el_deg)) {
            try_add_eci_ray(limb_ray);
          }
        }
      }
    }
  }

  if (intersection_points.empty()) {
    envelope.has_intersection = false;
    return envelope;
  }

  envelope.has_intersection = true;
  envelope.min_lat_deg = intersection_points.front().first;
  envelope.max_lat_deg = intersection_points.front().first;
  envelope.min_lon_deg = intersection_points.front().second;
  envelope.max_lon_deg = intersection_points.front().second;

  for (const auto& pt : intersection_points) {
    envelope.min_lat_deg = std::min(envelope.min_lat_deg, pt.first);
    envelope.max_lat_deg = std::max(envelope.max_lat_deg, pt.first);
    envelope.min_lon_deg = std::min(envelope.min_lon_deg, pt.second);
    envelope.max_lon_deg = std::max(envelope.max_lon_deg, pt.second);
  }
  return envelope;
}

/**
 * @brief 锁定目标的周期上下文暂存（单镜筒轮转用，2026-09-02）。
 * @note 周期前段（几何/SNR 公共段）为锁定目标填充，周期末尾 NFOV 轮转阶段消费：
 *       跟踪目标不再逐目标内联处理，上下文跨段传递。
 */
struct LockedTargetContext {
  const SbirsEciSceneTarget* target{nullptr};
  float azimuth_deg{0.0f};          ///< ECI 真值方位角
  float elevation_deg{0.0f};        ///< ECI 真值俯仰角
  float sensor_azimuth_deg{0.0f};   ///< 传感器系方位角（窄场几何门用）
  float sensor_elevation_deg{0.0f}; ///< 传感器系俯仰角
  double range_m{0.0};
  double snr{0.0};
  double received_power_w{0.0};
  double signal_energy_j{0.0};
  float omega_deg_per_sec{0.0f};
  double max_detection_range_m{0.0};
};

SbirsPointingDisturbanceParameters DisturbanceParameters(
    const config::SbirsPointingDisturbanceConfig& config) {
  SbirsPointingDisturbanceParameters parameters;
  parameters.common_attitude_sigma_deg = static_cast<double>(config.common_attitude_sigma_deg);
  parameters.common_attitude_correlation_time_s =
      static_cast<double>(config.common_attitude_correlation_time_s);
  parameters.channel_pointing_sigma_deg = static_cast<double>(config.channel_pointing_sigma_deg);
  parameters.channel_pointing_correlation_time_s =
      static_cast<double>(config.channel_pointing_correlation_time_s);
  parameters.channel_vibration_amplitude_deg =
      static_cast<double>(config.channel_vibration_amplitude_deg);
  parameters.channel_vibration_frequency_hz =
      static_cast<double>(config.channel_vibration_frequency_hz);
  return parameters;
}

bool EffectiveNfovPointing(const SbirsPointingCoordinator& coordinator,
                           const SbirsPointingDisturbanceParameters& parameters,
                           const session::SbirsVector3M& nominal_los,
                           const SbirsBoresightChain& boresight_chain, float static_error_deg,
                           float* azimuth_deg, float* elevation_deg) {
  if (azimuth_deg == nullptr || elevation_deg == nullptr) {
    return false;
  }
  SbirsPointingDisturbanceSample disturbance;
  // 单镜筒（2026-09-02）：通道扰动只有 0 号一份，宽场帧扰动与窄场指向共用其采样接口。
  if (!coordinator.DisturbanceSample(0, parameters, &disturbance)) {
    return false;
  }
  // 阶段 2：实际指向 = 名义 LOS（传感器系 az/el）+ 共模/通道扰动 + 静态 settle 误差，
  // 全部在传感器系叠加（identity 链下与历史逐位一致）；输出为传感器系 az/el。
  float sensor_azimuth_deg = 0.0f;
  float sensor_elevation_deg = 0.0f;
  boresight_chain.SensorAzElOfEciVector(nominal_los, &sensor_azimuth_deg, &sensor_elevation_deg);
  *azimuth_deg = sensor_azimuth_deg +
                 static_cast<float>(disturbance.common.azimuth_deg + disturbance.channel.azimuth_deg) +
                 static_error_deg;
  *elevation_deg = sensor_elevation_deg +
                   static_cast<float>(disturbance.common.elevation_deg + disturbance.channel.elevation_deg);
  return true;
}

bool IsFiniteGaussianState(const tracking::SbirsGaussianState& state) {
  return state.mean.allFinite() && state.covariance.allFinite();
}

bool IsFiniteGaussianState(const tracking::SbirsAngleCvGaussianState& state) {
  return state.mean.allFinite() && state.covariance.allFinite();
}

bool IsValidTrackingSnapshot(const SbirsPipelineSnapshot& snapshot,
                             const config::SbirsTrackingConfig& tracking_config) {
  if (!std::isfinite(snapshot.scan_phase_deg) || snapshot.scan_row_index < 0 ||
      snapshot.next_detection_id == 0U ||
      snapshot.wfov_measurement_random_state == 0U ||
      snapshot.estimated_measurement_random_state == 0U ||
      snapshot.sensor_like_output_random_state == 0U ||
      !std::isfinite(snapshot.misalignment_yaw_deg) ||
      !std::isfinite(snapshot.misalignment_pitch_deg) ||
      !std::isfinite(snapshot.misalignment_roll_deg)) {
    return false;
  }
  for (const auto& entry : snapshot.cue_predictor.targets) {
    if (!std::isfinite(entry.second.measured_azimuth_deg) ||
        !std::isfinite(entry.second.measured_elevation_deg)) {
      return false;
    }
  }

  std::set<std::uint64_t> estimated_target_ids;
  for (const auto& entry : snapshot.target_states) {
    switch (entry.second) {
      case SbirsTargetState::kUndetected:
      case SbirsTargetState::kWideCandidate:
      case SbirsTargetState::kAwaitingNfovAcquisition:
      case SbirsTargetState::kStrictTruthAssistedTracking:
      case SbirsTargetState::kSensorLikeTruthAssistedTracking:
      case SbirsTargetState::kLost:
        break;
      case SbirsTargetState::kEstimatedTracking:
        estimated_target_ids.insert(entry.first);
        break;
      default:
        return false;
    }
  }
  const bool angle_cv_backend =
      tracking_config.estimated_backend == config::SbirsEstimatedTrackingBackend::kAngleCvKf;
  if (angle_cv_backend) {
    if (snapshot.filter_states.size() != 0U ||
        snapshot.angle_kf_states.size() != estimated_target_ids.size() ||
        snapshot.nis_gate_exceeded_counts.size() != estimated_target_ids.size()) {
      return false;
    }
  } else if (snapshot.filter_states.size() != estimated_target_ids.size() ||
             snapshot.angle_kf_states.size() != 0U ||
             snapshot.nis_gate_exceeded_counts.size() != estimated_target_ids.size()) {
    return false;
  }
  const std::size_t expected_model_count = tracking_config.imm_model_noise_diff_coeffs.empty()
                                               ? 2U
                                               : tracking_config.imm_model_noise_diff_coeffs.size();
  for (const std::uint64_t target_id : estimated_target_ids) {
    if (snapshot.nis_gate_exceeded_counts.count(target_id) == 0U) {
      return false;
    }
    if (angle_cv_backend) {
      const auto angle_filter = snapshot.angle_kf_states.find(target_id);
      if (angle_filter == snapshot.angle_kf_states.end() ||
          !IsFiniteGaussianState(angle_filter->second)) {
        return false;
      }
      continue;
    }
    const auto filter = snapshot.filter_states.find(target_id);
    if (filter == snapshot.filter_states.end() || !IsFiniteGaussianState(filter->second)) {
      return false;
    }
    if (tracking_config.estimated_backend != config::SbirsEstimatedTrackingBackend::kImm) {
      continue;
    }
    const auto imm = snapshot.imm_snapshots.find(target_id);
    if (imm == snapshot.imm_snapshots.end() ||
        imm->second.model_states.size() != expected_model_count ||
        static_cast<std::size_t>(imm->second.model_weights.size()) != expected_model_count ||
        !imm->second.model_weights.allFinite()) {
      return false;
    }
    for (std::size_t index = 0U; index < expected_model_count; ++index) {
      const tracking::SbirsImmModelState& model = imm->second.model_states[index];
      if (!IsFiniteGaussianState(model.state) || !std::isfinite(model.weight) ||
          model.weight != imm->second.model_weights(static_cast<Eigen::Index>(index))) {
        return false;
      }
    }
  }
  const std::size_t expected_imm_count =
      tracking_config.estimated_backend == config::SbirsEstimatedTrackingBackend::kImm
          ? estimated_target_ids.size()
          : 0U;
  return snapshot.imm_snapshots.size() == expected_imm_count &&
         snapshot.imm_active == (expected_imm_count != 0U);
}

double ComputeSnr(const config::SbirsInternalExecutionConfig& config,
                  const SbirsEciSceneTarget& target, double range_m, float transmittance,
                  double* received_power_w = nullptr, double* signal_energy_j = nullptr) {
  const config::SbirsHardwareConfig& hardware = config.session.hardware;
  const double received_power = foundation::ComputeReceivedPowerW(
      target.radiant_intensity_w_per_sr, range_m, hardware.optical_aperture_m,
      hardware.optical_transmission, transmittance, hardware.detector_quantum_efficiency);
  // 2.8 噪声分解：背景/热/读出三项 RMS 合成；默认全 0 时回退到 NEP 标量。
  const foundation::SbirsNoiseStatistics noise =
      foundation::ComputeBackgroundNoiseStatistics(hardware);
  const double effective_noise = foundation::ResolveEffectiveNoiseW(hardware, noise);
  const double signal_energy =
      std::max(0.0, received_power) * std::max(0.0f, hardware.integration_time_sec);
  // 验收日志（3.2.1.3.2/3.2.1.3.2.3）保留中间变量：接收功率与信号能量经出参透出，
  // 不改变 SNR 计算；出参为空时零额外成本。
  if (received_power_w != nullptr) {
    *received_power_w = received_power;
  }
  if (signal_energy_j != nullptr) {
    *signal_energy_j = signal_energy;
  }
  return signal_energy / effective_noise;
}

// 阶段 3：由安装失准配置抽取运行期一次常值失准角总量（常值偏置 + 随机微扰）。
// sigma==0 时直接返回 bias（不消耗随机流）；否则用配置种子构造独立流、每轴一次
// N(0,σ) 抽取（3 次 Box-Muller 采样，弃样语义与量测域 SbirsRandomSource 一致）。
// 每次调用用同种子确定性重抽产生同值——pipeline 构造与 ApplyConfig 均调用，
// 配置不变时运行内不变化，保证 replay 可复现与确定性 continuation。
session::SbirsEulerAnglesDeg DrawMisalignmentTotal(
    const config::SbirsMisalignmentModel& misalignment) {
  session::SbirsEulerAnglesDeg total;
  total.yaw_deg = misalignment.bias_deg.yaw_deg;
  total.pitch_deg = misalignment.bias_deg.pitch_deg;
  total.roll_deg = misalignment.bias_deg.roll_deg;
  if (misalignment.random_sigma_deg <= 0.0f) {
    return total;
  }
  foundation::SbirsRandomSource random(misalignment.random_seed);
  const double sigma = static_cast<double>(misalignment.random_sigma_deg);
  total.yaw_deg += sigma * random.NextStandardNormal();
  total.pitch_deg += sigma * random.NextStandardNormal();
  total.roll_deg += sigma * random.NextStandardNormal();
  return total;
}

}  // namespace

SbirsPipeline::SbirsPipeline(const config::SbirsInternalExecutionConfig& config)
    : config_(config),
      scan_wrap_span_deg_(config.session.mission.scan_span_deg),
      misalignment_total_deg_(DrawMisalignmentTotal(config.session.orientation.misalignment)),
      nfov_scheduler_(),
      pointing_coordinator_(config.session.policy.pointing_disturbance.random_seed),
      wfov_measurement_random_source_(DeriveMeasurementSeed(
          config.session.policy.error_model.random_seed, kWfovMeasurementDomain)),
      estimated_measurement_random_source_(DeriveMeasurementSeed(
          config.session.policy.error_model.random_seed, kEstimatedMeasurementDomain)),
      sensor_like_output_random_source_(DeriveMeasurementSeed(
          config.session.policy.error_model.random_seed, kSensorLikeOutputDomain)) {
  scan_phase_deg_ = config.session.mission.initial_scan_phase_deg;
}

void SbirsPipeline::ApplyConfig(const config::SbirsInternalExecutionConfig& config,
                                const runtime::SbirsRuntimeConfigImpact& impact) {
  const float previous_scan_azimuth_deg =
      ScanAzimuth(config_.session.mission.scan_start_az_deg, scan_phase_deg_,
                  scan_azimuth_base_deg_, ConfigDirectionSign(config_.session.mission));
  // 阶段 4：记录旧栅格当前行中心 el，供新栅格重锚行索引。
  const float previous_row_center_el_deg = RowCenterEl(config_.session.mission, scan_row_index_);
  config_ = config;
  // 阶段 3：安装失准为静态配置（不进运行期 patch），每次应用配置时确定性重抽
  // 运行期失准角总量——配置不变时同种子重抽产生同值（行为不变），配置变时即刻生效。
  misalignment_total_deg_ = DrawMisalignmentTotal(config.session.orientation.misalignment);
  // 验收项「安装矩阵误差功能测试」：配置重抽后矩阵变化，下一执行周期重写一行
  //（写入时卫星实体 ID 可能已由调用方标注，故不在此处直接写）。
  install_matrices_acceptance_pending_ = true;
  if (impact.scan_sector_changed) {
    const float candidate_phase = ScanPhaseForAzimuth(
        config_.session.mission.scan_start_az_deg, previous_scan_azimuth_deg,
        scan_azimuth_base_deg_,
        config_.session.mission.scan_direction ==
            config::SbirsScanDirection::kIncreasingAzimuth);
    scan_phase_deg_ = config_.session.mission.scan_span_deg == 360.0f ||
                              candidate_phase < config_.session.mission.scan_span_deg
                          ? candidate_phase
                          : config_.session.mission.initial_scan_phase_deg;
    // 往复腿（2026-08-31）：重锚进新扇区保留腿方向；相位归零（旧方位不在新扇区）
    // 时腿复位为初始方向——scan_direction 语义 = 初始行进方向。
    if (scan_phase_deg_ == config_.session.mission.initial_scan_phase_deg) {
      scan_leg_forward_ = true;
    }
    // 行重锚（阶段 4）：旧行中心 el 映射到新栅格最近行；新栅格为单行模式
    // （span=0）或旧 el 不在新栅格 [el_start, el_start+span] 内时归零行。
    const int new_row_count = ScanRowCount(config_.session.mission);
    scan_row_index_ = 0;
    if (new_row_count > 1 && config_.session.mission.scan_el_span_deg > 0.0f &&
        previous_row_center_el_deg >= config_.session.mission.scan_el_start_deg &&
        previous_row_center_el_deg <= config_.session.mission.scan_el_start_deg +
                                          config_.session.mission.scan_el_span_deg) {
      const double relative_row =
          (static_cast<double>(previous_row_center_el_deg) -
           static_cast<double>(config_.session.mission.scan_el_start_deg)) /
          static_cast<double>(std::max(1.0e-6f, config_.session.mission.scan_el_step_deg));
      const int nearest_row = static_cast<int>(std::floor(relative_row + 0.5));
      scan_row_index_ =
          std::max(0, std::min(new_row_count - 1, nearest_row));
    }
  }
  if (impact.reset_measurement_random_stream) {
    const std::uint32_t seed = config.session.policy.error_model.random_seed;
    wfov_measurement_random_source_ = foundation::SbirsRandomSource(
        DeriveMeasurementSeed(seed, kWfovMeasurementDomain));
    estimated_measurement_random_source_ = foundation::SbirsRandomSource(
        DeriveMeasurementSeed(seed, kEstimatedMeasurementDomain));
    sensor_like_output_random_source_ = foundation::SbirsRandomSource(
        DeriveMeasurementSeed(seed, kSensorLikeOutputDomain));
  }
  if (impact.restart_pointing_disturbance) {
    // 指向扰动种子变更（runtime patch）：仅重启扰动流；单镜筒视线与逐目标簿记保持
    //（历史多通道时代的按通道数重配已随 max_concurrent_nfov_locks 删除而消失）。
    pointing_coordinator_.RestartDisturbance(
        config.session.policy.pointing_disturbance.random_seed);
  }
  if (impact.reset_nis_gate_counts) {
    tracking_coordinator_.ResetNisGateCounts();
  }
  if (impact.reset_nfov_gate_failure_counts) {
    pointing_coordinator_.ResetTrackingGateFailureCounts();
  }
  std::vector<std::uint64_t> released_tracking_targets;
  for (const auto& entry : target_states_) {
    const bool release_for_backend =
        impact.release_estimated_tracks && entry.second == SbirsTargetState::kEstimatedTracking;
    const bool release_for_mode =
        impact.release_incompatible_tracks &&
        (entry.second == SbirsTargetState::kEstimatedTracking ||
         IsTruthTrackingState(entry.second));
    if (release_for_backend || release_for_mode) {
      released_tracking_targets.push_back(entry.first);
    }
  }
  for (const std::uint64_t target_id : released_tracking_targets) {
    target_states_[target_id] = SbirsTargetState::kWideCandidate;
    nfov_scheduler_.Release(target_id);
    pointing_coordinator_.ReleaseTarget(target_id);
    tracking_coordinator_.ReleaseTarget(target_id);
  }
  if (impact.retag_truth_tracks) {
    const SbirsTargetState next_state =
        impact.next_tracking_mode == config::SbirsTrackingMode::kStrictTruthAssisted
            ? SbirsTargetState::kStrictTruthAssistedTracking
            : SbirsTargetState::kSensorLikeTruthAssistedTracking;
    for (auto& entry : target_states_) {
      if (IsTruthTrackingState(entry.second)) {
        entry.second = next_state;
      }
    }
  }
  if (impact.clear_for_inactive || impact.clear_for_wide_search) {
    target_states_.clear();
    // 模式切换清空目标状态时同步清连续命中计数：WideSearch 等模式不消耗计数，
    // 保留会让切回复合模式后的调度带陈旧命中数（required>1 时失真）。
    wfov_consecutive_hits_.clear();
    tracking_coordinator_.ClearForStandby();
    nfov_scheduler_.Clear();
    pointing_coordinator_.Clear();
    if (impact.clear_for_inactive) {
      cue_predictor_.Clear();
    }
  }
}

// 星间 cross-cue 消息诊断码（2026-09-01）：非法引导消息丢弃时计入 issues 的 kInfo 码。
// 管线局部常量：不进公开码表（include/1q 冻结面，见 sbirs-cross-cue 契约 out-of-scope）。
constexpr char kExternalCueInvalid[] = "sbirs.external_cue_invalid";

void SbirsPipeline::SubmitExternalCue(const session::SbirsExternalCue& cue) {
  pending_external_cues_.push_back(cue);
}

SbirsPipelineResult SbirsPipeline::RunCycle(const session::SbirsCycleInput& input) {
  SbirsPipelineResult result;
  const config::SbirsMissionConfig& mission = config_.session.mission;
  const config::SbirsPolicyConfig& policy = config_.session.policy;
  const config::SbirsEnvironmentConfig& environment_config = config_.session.environment;
  const float sim_time_sec = static_cast<float>(input.cycle_index) * input.dt_sec;
  const double cycle_start_time_s =
      static_cast<double>(input.cycle_index) * static_cast<double>(input.dt_sec);
  const double cycle_end_time_s =
      cycle_start_time_s + static_cast<double>(std::max(0.0f, input.dt_sec));

  // ECI 输出参考系（2026-08 正式变更）：本周期时刻的 GMST 把卫星与目标位置/速度
  // 从 ECEF 旋转到 ECI（J2000 平赤道面），下游 LOS/az/el/遮挡/SNR/EKF 全部在 ECI
  // 中执行；速度含 ω×r 输运项（见 1q/coordinate/inertial_transform.h）。目标侧
  // 旋转结果存入诚实命名的 SbirsEciSceneTarget（position_eci_m 等）。
  // 输入校验已保证 utc_julian_day 正有限；防御性回退 GMST=0 仅防不可达路径。
  // （2026-08-31 提升到指向合成之前：待机路径的名义扫描中心也要用方位基准。）
  double gmst_rad = 0.0;
  if (!oneq::coordinate::TryComputeGmstRad(input.utc_julian_day, &gmst_rad)) {
    gmst_rad = 0.0;
  }
  const oneq::coordinate::EcefPositionM satellite_ecef(
      input.satellite_position_ecef_m.x, input.satellite_position_ecef_m.y,
      input.satellite_position_ecef_m.z);
  oneq::coordinate::EciPositionM satellite_eci;
  if (!oneq::coordinate::TryEcefToEci(satellite_ecef, gmst_rad, &satellite_eci)) {
    satellite_eci = oneq::coordinate::EciPositionM(satellite_ecef.x_m, satellite_ecef.y_m,
                                                   satellite_ecef.z_m);
  }
  const session::SbirsVector3M satellite_position_eci_m{
      satellite_eci.x_m, satellite_eci.y_m, satellite_eci.z_m};
  // 方位基准（2026-08-31）：kEciAbsolute 基准恒 0（行为逐位不变）；kNadirRelative
  // 星下点方位 = atan2(-y, -x)（-卫星位置向量），由当周期位置现算并落成员——
  // GEO 恒定；动轨道随位置漂移，配置免推算（契约记录已删档，见 git 历史）。
  float scan_azimuth_base_deg = 0.0f;
  scan_wrap_span_deg_ = mission.scan_span_deg;
  if (mission.scan_azimuth_reference == config::SbirsScanAzimuthReference::kNadirRelative) {
    const session::SbirsVector3M nadir_direction{-satellite_position_eci_m.x,
                                                 -satellite_position_eci_m.y,
                                                 -satellite_position_eci_m.z};
    scan_azimuth_base_deg = foundation::ComputeAzimuthDeg(nadir_direction);
    // 跨度收敛（2026-08-31，修订 1——按"配置扇区 ∩ 地球可见窗"裁剪）：地球盘角半径
    // ρ=asin(R/|r|)（kEarthRadiusM 与遮挡判定同口径），可见窗 = 星下点方位 ±(ρ +
    // wfov_az/2)（扇区边缘视场外缘仍压着地球盘）。相位原点恒为配置起点（扇区与可见窗
    // 必含起点才可裁剪），只裁远端：扫描超窗部分对空天扫，无探测意义。绝对模式无锚，
    // 不收敛。扇区整段在窗外 = 指向性错配，属登记的俯仰/告警开放问题，不在此收敛。
    const double satellite_radius_m = foundation::Norm(satellite_position_eci_m);
    const double earth_disk_half_angle_deg =
        satellite_radius_m > kEarthRadiusM
            ? oneq::common::numerics::RadToDeg(
                  std::asin(std::min(1.0, kEarthRadiusM / satellite_radius_m)))
            : 90.0;
    const float useful_half_deg = static_cast<float>(
        earth_disk_half_angle_deg + 0.5 * mission.wide_field_fov_az_deg);
    const float sector_lo_deg =
        ConfigDirectionSign(mission) > 0.0f
            ? mission.scan_start_az_deg
            : mission.scan_start_az_deg - mission.scan_span_deg;
    const float sector_hi_deg =
        ConfigDirectionSign(mission) > 0.0f
            ? mission.scan_start_az_deg + mission.scan_span_deg
            : mission.scan_start_az_deg;
    const float clipped_lo_deg = std::max(sector_lo_deg, -useful_half_deg);
    const float clipped_hi_deg = std::min(sector_hi_deg, useful_half_deg);
    if (clipped_hi_deg > clipped_lo_deg &&
        (clipped_hi_deg - clipped_lo_deg) <
            mission.scan_span_deg - 1.0e-4f) {
      scan_wrap_span_deg_ = clipped_hi_deg - clipped_lo_deg;
      if (std::fabs(scan_wrap_span_deg_ - last_logged_wrap_span_deg_) > 0.05f) {
        PROJECT_LOG_INFO(
            "[SbirsPipeline] scan span converged to earth disk: configured={:.2f} -> "
            "effective={:.2f} (rho={:.2f} wfov_az={:.2f})",
            mission.scan_span_deg, scan_wrap_span_deg_, earth_disk_half_angle_deg,
            mission.wide_field_fov_az_deg);
        last_logged_wrap_span_deg_ = scan_wrap_span_deg_;
      }
    }
    // 有效跨度收缩（位置漂移）使相位越界 → 归零重锚，腿复位初始方向。
    if (scan_phase_deg_ >= scan_wrap_span_deg_) {
      scan_phase_deg_ = 0.0f;
      scan_leg_forward_ = true;
    }
  }
  scan_azimuth_base_deg_ = scan_azimuth_base_deg;

  // 指向合成链（阶段 2/3）：每周期由卫星姿态（Body->ECI）与安装角（Body->Sensor）
  // 及运行期安装失准角总量构建。默认零姿态 + 零安装角 + 零失准下为恒等变换
  // （IsIdentity），全部门控与输出与历史逐位一致。
  const oneq::coordinate::EulerAnglesDeg misalignment_angles_deg{
      misalignment_total_deg_.yaw_deg, misalignment_total_deg_.pitch_deg,
      misalignment_total_deg_.roll_deg};
  const SbirsBoresightChain boresight_chain(input.satellite_attitude_eci_body_deg,
                                            config_.session.orientation.mount_angles_deg,
                                            misalignment_angles_deg);
  // 名义扫描中心（扰动前）合成光轴的 ECI 方位/俯仰：输出参考保持 ECI 极坐标，
  // 不随姿态/安装变化；2-D 栅格（阶段 4）下俯仰为当前行中心 el。
  const auto nominal_scan_eci_angles_rad = [&]() -> std::pair<float, float> {
    const float scan_elevation_deg = RowCenterEl(mission, scan_row_index_);
    const float direction_sign = ConfigDirectionSign(mission);
    if (boresight_chain.IsIdentity()) {
      return std::make_pair(ToEciAzimuthRad(ScanAzimuth(mission.scan_start_az_deg,
                                                        scan_phase_deg_,
                                                        scan_azimuth_base_deg, direction_sign)),
                            ToEciElevationRad(scan_elevation_deg));
    }
    const session::SbirsVector3M eci_los = boresight_chain.EciLosOfSensorPointing(
        ScanAzimuth(mission.scan_start_az_deg, scan_phase_deg_, scan_azimuth_base_deg,
                    direction_sign),
        scan_elevation_deg);
    return std::make_pair(ToEciAzimuthRad(foundation::ComputeAzimuthDeg(eci_los)),
                          ToEciElevationRad(foundation::ComputeElevationDeg(eci_los)));
  };

  if (mission.work_mode == config::SbirsWorkMode::kStandby || !config_.session.sensor_enabled) {
    target_states_.clear();
    wfov_consecutive_hits_.clear();
    tracking_coordinator_.ClearForStandby();
    nfov_scheduler_.Clear();
    pointing_coordinator_.Clear();
    cue_predictor_.Clear();
    const std::pair<float, float> nominal_angles = nominal_scan_eci_angles_rad();
    result.scan_azimuth_rad = nominal_angles.first;
    result.scan_elevation_rad = nominal_angles.second;
    return result;
  }

  result.executed = true;

  std::vector<SbirsEciSceneTarget> eci_scene;
  eci_scene.reserve(input.scene.size());
  for (const session::SbirsSceneTarget& target : input.scene) {
    eci_scene.push_back(RotateSceneTargetToEci(target, gmst_rad));
  }
  // 卫星速度旋入 ECI（与目标侧 RotateSceneTargetToEci 同法，含 ω×r 输运项）；
  // 旋换失败走与位置相同的防御性回退（保留原分量）。
  const oneq::coordinate::EcefVelocityMps satellite_velocity_ecef(
      input.satellite_velocity_ecef_m_per_s.x, input.satellite_velocity_ecef_m_per_s.y,
      input.satellite_velocity_ecef_m_per_s.z);
  oneq::coordinate::EciVelocityMps satellite_velocity_eci;
  if (!oneq::coordinate::TryEcefVelocityToEci(satellite_ecef, satellite_velocity_ecef, gmst_rad,
                                              &satellite_velocity_eci)) {
    satellite_velocity_eci = oneq::coordinate::EciVelocityMps(
        satellite_velocity_ecef.x_mps, satellite_velocity_ecef.y_mps,
        satellite_velocity_ecef.z_mps);
  }
  const session::SbirsVector3M satellite_velocity_eci_m{
      satellite_velocity_eci.x_mps, satellite_velocity_eci.y_mps,
      satellite_velocity_eci.z_mps};

  // 规则 13a：周期执行摘要所需四类门控排除计数（按目标循环内累加）。
  std::size_t excluded_occulted = 0U;
  std::size_t excluded_out_of_range = 0U;
  std::size_t excluded_out_of_wfov = 0U;
  std::size_t excluded_snr_below = 0U;
  const auto log_cycle_summary = [&]() {
    // 中译：周期执行摘要（周期号、扫描方位角、探测成功数/目标总数、记录总数、
    //       四类门控排除计数）。
    // 标识：规则 13a 周期级执行摘要日志——每周期探测概况与几何/SNR 排除分布，
    //       供宏观核对与"零探测"排查；仅人读，不用于状态判断（规则 3）。
    //       detected 只统计 record.detected == true 的成功探测；records 为本周期
    //       全部检测记录数（含捕获失败/门失败等未过门限记录），两数分离避免把
    //       失败记录误读为探测（2026-08-08 修正）。
    const std::size_t detected_count = static_cast<std::size_t>(std::count_if(
        result.detections.begin(), result.detections.end(),
        [](const SbirsPipelineDetection& detection) { return detection.record.detected; }));
    PROJECT_LOG_INFO("[SbirsPipeline] cycle_index={} scan_az_rad={:.4f} detected={}/{} records={} "
                     "excluded={{occulted={} range={} wfov={} snr={}}}",
                     input.cycle_index, result.scan_azimuth_rad, detected_count,
                     input.scene.size(), result.detections.size(), excluded_occulted,
                     excluded_out_of_range, excluded_out_of_wfov, excluded_snr_below);
  };

  // 验收日志 E7：NFOV 通道释放事件（3.2.1.3.2.4 传感器侧协同）。只在目标当前持有
  // NFOV 锁定时输出（未锁定目标的门排除不是通道事件）；须在各 Release 调用前打印。
  const auto log_acceptance_release = [&](std::uint64_t target_id, const char* reason,
                                          unsigned int gate_failures) {
    if (!SBIRS_ACCEPTANCE_LOG_ENABLED() || !nfov_scheduler_.IsLocked(target_id)) {
      return;
    }
    // 验收日志 E7 内容由「特殊事件监测与提示功能测试」与「窄视场跟踪探测功能测试」
    // 覆盖（释放原因经事件行/滑行标志承载）；不再单独输出「协同工作机制」诊断行。
    (void)reason;
    (void)gate_failures;
  };

  const SbirsPointingDisturbanceParameters disturbance_parameters =
      DisturbanceParameters(policy.pointing_disturbance);
  if (!pointing_coordinator_.AdvanceDisturbance(static_cast<double>(input.dt_sec),
                                                disturbance_parameters)) {
    const std::pair<float, float> nominal_angles = nominal_scan_eci_angles_rad();
    result.scan_azimuth_rad = nominal_angles.first;
    result.scan_elevation_rad = nominal_angles.second;
    log_cycle_summary();
    return result;
  }
  SbirsPointingDisturbanceSample frame_disturbance;
  if (!pointing_coordinator_.DisturbanceSample(0, disturbance_parameters, &frame_disturbance)) {
    const std::pair<float, float> nominal_angles = nominal_scan_eci_angles_rad();
    result.scan_azimuth_rad = nominal_angles.first;
    result.scan_elevation_rad = nominal_angles.second;
    log_cycle_summary();
    return result;
  }

  // 往复式扫描推进（2026-08-31，牛耕式）：相位沿"行程坐标" travel ∈ [0, 2·wrap_span)
  // 推进 rate·dt 后折叠——去程 [0, span) 正走、回程 [span, 2span) 反走，到边反射
  // 而非跳回起点（真实扫描机构行为；锯齿→往复的行为替换，契约记录已删档、
  // 见 git 历史）。栅格行进=每次过界步进一行
  // （crossings 计数与既有节奏同值：dt·rate=k·span 时相位/行序列与锯齿逐位重合）。
  // rate=0 时 travel 恒 0、腿恒初始方向——凝视行为逐位不变。
  const int row_count = ScanRowCount(mission);
  const float wrap_span = scan_wrap_span_deg_;
  const float prev_travel =
      scan_leg_forward_ ? scan_phase_deg_ : 2.0f * wrap_span - scan_phase_deg_;
  const float advanced_travel =
      prev_travel + mission.scan_rate_deg_per_sec * std::max(0.0f, input.dt_sec);
  const int row_advance =
      static_cast<int>(std::floor(static_cast<double>(advanced_travel) /
                                  static_cast<double>(wrap_span))) -
      static_cast<int>(std::floor(static_cast<double>(prev_travel) /
                                  static_cast<double>(wrap_span)));
  const float folded_travel = std::fmod(advanced_travel, 2.0f * wrap_span);
  scan_leg_forward_ = folded_travel < wrap_span;
  scan_phase_deg_ = scan_leg_forward_ ? folded_travel : 2.0f * wrap_span - folded_travel;
  if (row_advance > 0 && row_count > 1) {
    scan_row_index_ = (scan_row_index_ + row_advance) % row_count;
  }
  const float scan_azimuth_deg =
      ScanAzimuth(mission.scan_start_az_deg, scan_phase_deg_, scan_azimuth_base_deg,
                  ConfigDirectionSign(mission));
  const std::pair<float, float> nominal_angles = nominal_scan_eci_angles_rad();
  result.scan_azimuth_rad = nominal_angles.first;
  result.scan_elevation_rad = nominal_angles.second;
  // 实际扫描中心（传感器系，扰动前相位扫描角 + 共模扰动，随后按扫描限位钳制）：
  // 体稳定 = 扫描参数直接为传感器系角度；惯性稳定 = 扫描参数为 ECI 参考方向，经链
  // 反解到传感器系（物理上保持惯性方向稳定）。实际光轴足迹 = 链旋转该传感器系指向。
  float scan_azimuth_sensor_deg = scan_azimuth_deg;
  float scan_elevation_sensor_deg = RowCenterEl(mission, scan_row_index_);
  if (config_.session.orientation.stabilization_mode ==
      config::SbirsStabilizationMode::kInertialStabilized) {
    const session::SbirsVector3M desired_eci_los =
        LosFromAzimuthElevation(scan_azimuth_deg, scan_elevation_sensor_deg);
    const double desired_norm = foundation::Norm(desired_eci_los);
    session::SbirsVector3M desired_unit;
    desired_unit.x = desired_norm > 0.0 ? desired_eci_los.x / desired_norm : desired_eci_los.x;
    desired_unit.y = desired_norm > 0.0 ? desired_eci_los.y / desired_norm : desired_eci_los.y;
    desired_unit.z = desired_norm > 0.0 ? desired_eci_los.z / desired_norm : desired_eci_los.z;
    boresight_chain.SensorPointingForDesiredEciLos(desired_unit, &scan_azimuth_sensor_deg,
                                                   &scan_elevation_sensor_deg);
  }
  // 名义（扰动前）扫描中心传感器系角度：NFOV 通道初始 LOS 用（历史语义：Reserve 初值
  // 取扰动前扫描指向，不携带本周期共模扰动）。
  const float nominal_scan_azimuth_sensor_deg = scan_azimuth_sensor_deg;
  const float nominal_scan_elevation_sensor_deg = scan_elevation_sensor_deg;
  scan_azimuth_sensor_deg += static_cast<float>(frame_disturbance.common.azimuth_deg);
  scan_elevation_sensor_deg += static_cast<float>(frame_disturbance.common.elevation_deg);
  SbirsBoresightChain::ClampToScanLimits(config_.session.orientation.sensor_scan_limits_deg,
                                         &scan_azimuth_sensor_deg, &scan_elevation_sensor_deg);
  const float actual_scan_azimuth_sensor_deg = scan_azimuth_sensor_deg;
  const float actual_scan_elevation_sensor_deg = scan_elevation_sensor_deg;

  const auto compute_crossing_time_s = [&](float candidate_sensor_az_deg) -> double {
    if (mission.scan_rate_deg_per_sec <= 1e-6f || input.dt_sec <= 0.0f) {
      return cycle_end_time_s;
    }
    const float leg_direction = scan_leg_forward_ ? 1.0f : -1.0f;
    const float effective_direction = ConfigDirectionSign(mission) * leg_direction;
    const float az_velocity = effective_direction * mission.scan_rate_deg_per_sec;
    if (std::fabs(az_velocity) <= 1e-6f) {
      return cycle_end_time_s;
    }
    const float az_delta = AzimuthDelta(candidate_sensor_az_deg, actual_scan_azimuth_sensor_deg);
    const float time_offset_from_end = az_delta / az_velocity;
    const float t_crossing_in_cycle =
        std::max(0.0f, std::min(input.dt_sec, input.dt_sec + time_offset_from_end));
    return cycle_start_time_s + static_cast<double>(t_crossing_in_cycle);
  };

  if (SBIRS_DUAL_ACCEPTANCE_LOG_ENABLED()) {
    // 覆盖四角 = 视场矩形与地球视盘交集区域的经纬度范围框角点（采样视场边界与地平圈圆弧
    // 取经纬度包络），映射为规范顺序 4 角（西北、东北、东南、西南）；视场完全脱靶记 miss。
    // 中心为实际扫描中心与地球交点（指向太空记 miss）。驻留时间 = WFOV 方位视场 / 扫描速率
    // （目标被方位向扫描穿越视场所需时间；scan_rate=0 的退化配置记 0）。
    // 中译：宽视场地面覆盖区事件（周期号、卫星ID、覆盖四角与中心的经纬度、驻留时间）。
    // 标识：验收日志 E1——3.2.1.3.1/3.2.1.3.2.2"各扫描周期地面覆盖区域坐标与驻留
    //       时间"证据；圆球地球模型（椭球为已登记非目标，见 boundaries.md）。
    const float dwell_s = mission.scan_rate_deg_per_sec > 0.0f
                              ? mission.wide_field_fov_az_deg / mission.scan_rate_deg_per_sec
                              : 0.0f;
    const GroundCoverageEnvelope envelope = ComputeGroundCoverageEnvelope(
        satellite_position_eci_m, boresight_chain, actual_scan_azimuth_sensor_deg,
        actual_scan_elevation_sensor_deg, mission.wide_field_fov_az_deg,
        mission.wide_field_fov_el_deg, kEarthRadiusM, gmst_rad);

    std::string corners_csv;
    if (envelope.has_intersection) {
      // 规范 4 角顺序：西北 (lat_max, lon_min)、东北 (lat_max, lon_max)、
      //                东南 (lat_min, lon_max)、西南 (lat_min, lon_min)
      const std::string lat_max_str = FormatFloat(static_cast<float>(envelope.max_lat_deg));
      const std::string lat_min_str = FormatFloat(static_cast<float>(envelope.min_lat_deg));
      const std::string lon_min_str = FormatFloat(static_cast<float>(envelope.min_lon_deg));
      const std::string lon_max_str = FormatFloat(static_cast<float>(envelope.max_lon_deg));

      corners_csv = lat_max_str + "," + lon_min_str + ";" +
                    lat_max_str + "," + lon_max_str + ";" +
                    lat_min_str + "," + lon_max_str + ";" +
                    lat_min_str + "," + lon_min_str;
    } else {
      corners_csv = "miss;miss;miss;miss";
    }

    double center_lat_deg = 0.0;
    double center_lon_deg = 0.0;
    const bool center_valid = foundation::TryComputeGroundIntersectionLatLonDeg(
        satellite_position_eci_m,
        boresight_chain.EciLosOfSensorPointing(actual_scan_azimuth_sensor_deg,
                                               actual_scan_elevation_sensor_deg),
        kEarthRadiusM, gmst_rad, &center_lat_deg, &center_lon_deg);
    const std::string center_text =
        center_valid ? FormatFloat(static_cast<float>(center_lat_deg)) + "," +
                           FormatFloat(static_cast<float>(center_lon_deg))
                     : std::string("miss");

    std::string footprint = "覆盖四角经纬=[" + corners_csv + "] 中心=(" + center_text +
                            ") 驻留=" + oneq::logging::FormatF(dwell_s, 3) + "s";
    // 规范口径（验收判定标准 第12项）：地面覆盖区域坐标与驻留时间 + 卫星ID。
    std::string wfov_row = "卫星ID=" + std::to_string(satellite_entity_id_) + " " + footprint;
    SBIRS_DUAL_ACCEPTANCE_ITEM(sim_time_sec, input.cycle_index, "宽视场扫描探测功能测试",
                               wfov_row);
    // 规范口径（验收判定标准 第14项·其一）：宽视场扫描覆盖区域与第 12 项同数据、
    // 按本项分行重写。
    std::string joint_cover = "卫星ID=" + std::to_string(satellite_entity_id_) +
                              " 宽视场扫描覆盖区域=" + footprint;
    SBIRS_DUAL_ACCEPTANCE_ITEM(sim_time_sec, input.cycle_index, "宽窄视场联合探测功能测试",
                               joint_cover);
    if (install_matrices_acceptance_pending_) {
      // 验收项「安装矩阵误差功能测试」（第10项）：首个执行周期写一次（周期 0 语义；
      // ApplyConfig 重抽失准后置位重写）。卫星 ID 此时已可被调用方标注。
      WriteSbirsInstallMatrices(
          satellite_entity_id_, config_.session.orientation.mount_angles_deg,
          oneq::coordinate::EulerAnglesDeg(misalignment_total_deg_.yaw_deg,
                                           misalignment_total_deg_.pitch_deg,
                                           misalignment_total_deg_.roll_deg));
      install_matrices_acceptance_pending_ = false;
    }
    const double sat_range = std::sqrt(input.satellite_position_ecef_m.x * input.satellite_position_ecef_m.x +
                                       input.satellite_position_ecef_m.y * input.satellite_position_ecef_m.y +
                                       input.satellite_position_ecef_m.z * input.satellite_position_ecef_m.z);
    WriteSbirsOrbitSample(satellite_entity_id_, sim_time_sec, input.cycle_index,
                          policy.error_model.orbit_sigma_deg, sat_range,
                          policy.error_model.nav_position_sigma_m,
                          input.satellite_position_ecef_m);
  }

  // 门内归因目标无关量（仅依赖硬件/门限配置，循环外计算一次）：有效噪声与达标所需签名 =
  // wide_min·噪声/积分时间（见 ClassifyWfovSnrExclusionCause）。
  const config::SbirsHardwareConfig& hw = config_.session.hardware;
  const foundation::SbirsNoiseStatistics noise_statistics =
      foundation::ComputeBackgroundNoiseStatistics(hw);
  const double effective_noise_w = foundation::ResolveEffectiveNoiseW(hw, noise_statistics);
  const double snr_signature_required =
      policy.detection.wide_min_snr_linear * effective_noise_w /
      std::max(0.0f, hw.integration_time_sec);
  std::vector<SbirsCandidate> candidates;
  // 锁定目标上下文暂存（单镜筒轮转）：周期前段填充、NFOV 阶段消费。
  std::map<std::uint64_t, LockedTargetContext> locked_contexts;
  std::set<std::uint64_t> present_target_ids;
  for (const session::SbirsSceneTarget& target : input.scene) {
    present_target_ids.insert(target.target_id);
  }
  // 星间 cross-cue（2026-09-01）：消费运行期注入的引导消息（修订 5：不进每帧输入）。
  // 合法性：目标须在本周期场景内、角度有限、距离为正；非法整条丢弃并计 kInfo issue。
  // 同目标多条取最后一条（同源逐周期刷新）；来源星 ECEF 按当周期 GMST 旋入 ECI。
  std::map<std::uint64_t, session::SbirsExternalCue> external_cue_by_target;
  std::map<std::uint64_t, session::SbirsVector3M> external_source_eci_by_target;
  for (const session::SbirsExternalCue& cue : pending_external_cues_) {
    const bool finite_bearing =
        std::isfinite(cue.azimuth_deg) && std::isfinite(cue.elevation_deg);
    if (!finite_bearing || !(cue.range_m > 0.0) ||
        present_target_ids.count(cue.target_id) == 0U) {
      result.issues.push_back(MakeExclusionIssue(
          kExternalCueInvalid,
          "dropped external cue: target_id=" + std::to_string(cue.target_id) +
              " source_satellite_entity_id=" +
              std::to_string(cue.source_satellite_entity_id) +
              " range_m=" + std::to_string(static_cast<std::int64_t>(cue.range_m)),
          session::SbirsIssueCause::kNone, -1));
      continue;
    }
    const oneq::coordinate::EcefPositionM source_ecef{
        cue.source_position_ecef_m.x, cue.source_position_ecef_m.y,
        cue.source_position_ecef_m.z};
    oneq::coordinate::EciPositionM source_eci;
    if (!oneq::coordinate::TryEcefToEci(source_ecef, gmst_rad, &source_eci)) {
      result.issues.push_back(MakeExclusionIssue(
          kExternalCueInvalid,
          "dropped external cue: source ECEF->ECI rotation failed, target_id=" +
              std::to_string(cue.target_id),
          session::SbirsIssueCause::kNone, -1));
      continue;
    }
    external_cue_by_target[cue.target_id] = cue;
    external_source_eci_by_target[cue.target_id] =
        session::SbirsVector3M{source_eci.x_m, source_eci.y_m, source_eci.z_m};
  }
  pending_external_cues_.clear();
  // 外部候选构造（cross-cue）：自星宽场门失败但有外部引导的目标由此进同一候选池。
  // 三角化只用来源星带误差量测链（测角+距离+来源星位置），不触目标真值；连续命中
  // 计数、调度排序（SNR↓→距离↑→ID↑，修订 4 同池同规则）与 NFOV 捕获/跟踪门全部
  // 复用既有路径；引导来源只随候选/调度器记录，不进验收日志行。
  const auto make_external_cue_candidate = [&](const SbirsEciSceneTarget& target,
                                               float azimuth_deg, float elevation_deg,
                                               double range_m, double snr,
                                               double max_detection_range_m,
                                               float omega_deg_per_sec) {
    const session::SbirsExternalCue& cue = external_cue_by_target.at(target.target_id);
    const session::SbirsVector3M& source_eci =
        external_source_eci_by_target.at(target.target_id);
    SbirsCandidate candidate;
    candidate.target = &target;
    candidate.azimuth_deg = azimuth_deg;
    candidate.elevation_deg = elevation_deg;
    candidate.range_m = range_m;
    candidate.max_detection_range_m = max_detection_range_m;
    const session::SbirsVector3M source_los = LosFromAzimuthElevation(cue.azimuth_deg,
                                                                      cue.elevation_deg);
    const session::SbirsVector3M target_est{
        source_eci.x + source_los.x * cue.range_m, source_eci.y + source_los.y * cue.range_m,
        source_eci.z + source_los.z * cue.range_m};
    const session::SbirsVector3M own_los_est =
        foundation::Subtract(target_est, satellite_position_eci_m);
    candidate.measured_azimuth_deg = foundation::ComputeAzimuthDeg(own_los_est);
    candidate.measured_elevation_deg = foundation::ComputeElevationDeg(own_los_est);
    candidate.measured_range_m = foundation::Norm(own_los_est);
    candidate.relative_angular_rate_deg_per_sec = omega_deg_per_sec;
    candidate.cue_source_satellite_entity_id =
        static_cast<int>(cue.source_satellite_entity_id);
    // 与自星路径同款：本星视角三角化测角喂 cue 预测器（外推 narrow_cue_latency_s）。
    const SbirsCuePrediction cue_prediction = cue_predictor_.Update(
        target.target_id, candidate.measured_azimuth_deg, candidate.measured_elevation_deg,
        input.dt_sec, mission.narrow_cue_latency_s);
    candidate.command_azimuth_deg = cue_prediction.command_azimuth_deg;
    candidate.command_elevation_deg = cue_prediction.command_elevation_deg;
    const float cue_latency_s = mission.narrow_cue_latency_s;
    if (cue_latency_s > 0.0f) {
      session::SbirsVector3M predicted_position;
      predicted_position.x = target.position_eci_m.x;
      predicted_position.y = target.position_eci_m.y;
      predicted_position.z = target.position_eci_m.z;
      if (target.has_velocity_eci_m_per_s) {
        predicted_position.x += target.velocity_eci_m_per_s.x * cue_latency_s;
        predicted_position.y += target.velocity_eci_m_per_s.y * cue_latency_s;
        predicted_position.z += target.velocity_eci_m_per_s.z * cue_latency_s;
      }
      session::SbirsVector3M predicted_satellite_position;
      predicted_satellite_position.x =
          satellite_position_eci_m.x + satellite_velocity_eci_m.x * cue_latency_s;
      predicted_satellite_position.y =
          satellite_position_eci_m.y + satellite_velocity_eci_m.y * cue_latency_s;
      predicted_satellite_position.z =
          satellite_position_eci_m.z + satellite_velocity_eci_m.z * cue_latency_s;
      const session::SbirsVector3M predicted_los =
          foundation::Subtract(predicted_position, predicted_satellite_position);
      candidate.delayed_truth_azimuth_deg = foundation::ComputeAzimuthDeg(predicted_los);
      candidate.delayed_truth_elevation_deg = foundation::ComputeElevationDeg(predicted_los);
    } else {
      candidate.delayed_truth_azimuth_deg = azimuth_deg;
      candidate.delayed_truth_elevation_deg = elevation_deg;
    }
    candidate.snr = snr;
    candidate.measurement_time_s = cycle_start_time_s;
    candidates.push_back(candidate);
    ++wfov_consecutive_hits_[target.target_id];
    if (target_states_[target.target_id] != SbirsTargetState::kAwaitingNfovAcquisition) {
      target_states_[target.target_id] = SbirsTargetState::kWideCandidate;
    }
  };
  for (std::map<std::uint64_t, SbirsTargetState>::value_type& target_state : target_states_) {
    if (present_target_ids.count(target_state.first) == 0U) {
      log_acceptance_release(target_state.first, "scene_absent", 0U);
      target_state.second = SbirsTargetState::kLost;
      wfov_consecutive_hits_.erase(target_state.first);
      nfov_scheduler_.Release(target_state.first);
      pointing_coordinator_.ReleaseTarget(target_state.first);
      cue_predictor_.Release(target_state.first);
      tracking_coordinator_.ReleaseTarget(target_state.first);
    }
  }

  for (std::size_t target_idx = 0U; target_idx < eci_scene.size(); ++target_idx) {
    // ECI 场景副本（真值已在周期入口旋转到 ECI，字段名即 ECI 语义）。
    const SbirsEciSceneTarget& target = eci_scene[target_idx];
    if (!target.active) {
      log_acceptance_release(target.target_id, "inactive", 0U);
      target_states_[target.target_id] = SbirsTargetState::kLost;
      wfov_consecutive_hits_.erase(target.target_id);
      nfov_scheduler_.Release(target.target_id);
      pointing_coordinator_.ReleaseTarget(target.target_id);
      cue_predictor_.Release(target.target_id);
      tracking_coordinator_.ReleaseTarget(target.target_id);
      continue;
    }

    if (foundation::IsEarthOcculted(satellite_position_eci_m, target.position_eci_m,
                                    kEarthRadiusM)) {
      // 规则 13b：遮挡排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
      // 具体门（遮挡判定本身可定位）：cause 保持 kNone，遮挡余量（负值 = 深度）进 message。
      const double occultation_margin_m = foundation::ComputeEarthOccultationMarginM(
          satellite_position_eci_m, target.position_eci_m, kEarthRadiusM);
      result.issues.push_back(MakeExclusionIssue(
          session::codes::kTargetOcculted,
          "target_id=" + std::to_string(target.target_id) +
              "; LOS from satellite occulted by Earth; occultation_margin_m=" +
              std::to_string(static_cast<std::int64_t>(occultation_margin_m)),
          session::SbirsIssueCause::kNone,
          static_cast<std::ptrdiff_t>(target_idx)));
      if (SBIRS_ACCEPTANCE_LOG_ENABLED()) {
        SBIRS_INTERNAL_INTERFACE_ITEM(
            sim_time_sec, input.cycle_index, "红外场景几何与视场门控交互接口测试",
            "卫星ID=" + std::to_string(satellite_entity_id_) + " 目标ID=" +
                std::to_string(target.target_id) + " 门控阶段=地球遮挡 门控结果=排除 排除码=" +
                session::codes::kTargetOcculted + " 遮挡余量=" +
                oneq::logging::FormatF(occultation_margin_m, 3) + "m");
      }
      ++excluded_occulted;
      log_acceptance_release(target.target_id, "earth_occulted", 0U);
      target_states_[target.target_id] = SbirsTargetState::kUndetected;
      wfov_consecutive_hits_.erase(target.target_id);
      nfov_scheduler_.Release(target.target_id);
      pointing_coordinator_.ReleaseTarget(target.target_id);
      cue_predictor_.Release(target.target_id);
      tracking_coordinator_.ReleaseTarget(target.target_id);
      continue;
    }

    const session::SbirsVector3M los =
        foundation::Subtract(target.position_eci_m, satellite_position_eci_m);
    const double range_m = foundation::Norm(los);
    if (range_m < mission.min_range_m || range_m > mission.max_range_m) {
      // 规则 13b：距离门排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
      // 具体门（距离带本身可定位）：cause 保持 kNone，距带边余量进 message。
      const double range_margin_m =
          range_m < mission.min_range_m ? mission.min_range_m - range_m : range_m - mission.max_range_m;
      result.issues.push_back(MakeExclusionIssue(
          session::codes::kTargetOutOfRange,
          "target_id=" + std::to_string(target.target_id) + "; range_m=" +
              std::to_string(static_cast<std::int64_t>(range_m)) + " outside [" +
              std::to_string(static_cast<std::int64_t>(mission.min_range_m)) + "," +
              std::to_string(static_cast<std::int64_t>(mission.max_range_m)) + "] range_margin_m=" +
              std::to_string(static_cast<std::int64_t>(range_margin_m)),
          session::SbirsIssueCause::kNone,
          static_cast<std::ptrdiff_t>(target_idx)));
      if (SBIRS_ACCEPTANCE_LOG_ENABLED()) {
        SBIRS_INTERNAL_INTERFACE_ITEM(
            sim_time_sec, input.cycle_index, "红外场景几何与视场门控交互接口测试",
            "卫星ID=" + std::to_string(satellite_entity_id_) + " 目标ID=" +
                std::to_string(target.target_id) + " 门控阶段=距离门 斜距=" +
                oneq::logging::FormatF(range_m, 3) + "m 距离范围=[" +
                oneq::logging::FormatF(mission.min_range_m, 3) + "," +
                oneq::logging::FormatF(mission.max_range_m, 3) + "]m 门控结果=排除 排除码=" +
                session::codes::kTargetOutOfRange);
      }
      ++excluded_out_of_range;
      log_acceptance_release(target.target_id, "out_of_range", 0U);
      target_states_[target.target_id] = SbirsTargetState::kUndetected;
      wfov_consecutive_hits_.erase(target.target_id);
      nfov_scheduler_.Release(target.target_id);
      pointing_coordinator_.ReleaseTarget(target.target_id);
      cue_predictor_.Release(target.target_id);
      tracking_coordinator_.ReleaseTarget(target.target_id);
      continue;
    }

    const float azimuth_deg = foundation::ComputeAzimuthDeg(los);
    const float elevation_deg = foundation::ComputeElevationDeg(los);
    // 目标 ECI 视线旋入传感器系（指向合成链）：WFOV 门与越界诊断改在传感器系执行
    //（输出 az/el 仍保持 ECI 参考）。identity 链下与传感器系差值逐位一致。
    float sensor_azimuth_deg = 0.0f;
    float sensor_elevation_deg = 0.0f;
    boresight_chain.SensorAzElOfEciVector(los, &sensor_azimuth_deg, &sensor_elevation_deg);
    // 壳段气团（冻结契约 2026-09-02）：逐目标 τ_geo = τ_eff^X，X = 视线在大气壳内的
    // 穿壳弦长÷垂直壳厚（100 km）；纯空间路径 X=0 → τ=1（不穿大气不再扣衰减）。
    const float path_transmittance = environment::ResolveGeometricTransmittance(
        environment_config, satellite_position_eci_m, target.position_eci_m);
    // 接收功率/信号能量中间变量透出（验收日志 E2/E5 消费；不影响 SNR 数值）。
    double received_power_w = 0.0;
    double signal_energy_j = 0.0;
    const double snr = ComputeSnr(config_, target, range_m, path_transmittance,
                                  &received_power_w, &signal_energy_j);
    // 当前时刻最大探测距离（WFOV 门限反解）：随本周期噪声快照、目标辐射强度与逐目标
    // 路径透过率 τ_geo（壳段气团）变化，进归属层诊断（不进 raw output）；SNR 门失败
    // 目标写入 issue 消息。
    const double max_detection_range_m = foundation::ComputeMaxDetectionRangeM(
        target.radiant_intensity_w_per_sr, hw.optical_aperture_m, hw.optical_transmission,
        path_transmittance, hw.detector_quantum_efficiency, hw.integration_time_sec,
        effective_noise_w, policy.detection.wide_min_snr_linear);
    if (SBIRS_ACCEPTANCE_LOG_ENABLED()) {
      // 中译：本星本周期对该目标的最大探测距离 d_max（米）。目标已进窄场锁定、由他星
      //       递话而来、或暂在本星宽场视场门外时同样成行。
      // 标识：验收判定标准 第6项。成行点在逐目标公共段，与目标状态机和候选来源无关，
      //       故跟踪期不再缺行；d_max 为宽场门限反解值，不含窄场门限口径。
      std::string dmax = "相对卫星ID=" + std::to_string(satellite_entity_id_);
      dmax += " 目标ID=" + std::to_string(target.target_id);
      dmax += " 相对卫星最大探测距离=" + oneq::logging::FormatF(max_detection_range_m, 1) + "m";
      SBIRS_ACCEPTANCE_ITEM(sim_time_sec, input.cycle_index, "最大探测距离计算功能测试", dmax);
    }

    // 相对视线角速度：用于动态滞后误差与 R 矩阵（design 2.10）。相对速度 =
    // v_target（未提供时取 0）− v_satellite；卫星速度必填，因此目标静止时 ω 不为 0
    // （卫星运动本身扫过视场）。
    session::SbirsVector3M relative_velocity_eci_m_per_s{
        -satellite_velocity_eci_m.x, -satellite_velocity_eci_m.y, -satellite_velocity_eci_m.z};
    if (target.has_velocity_eci_m_per_s) {
      relative_velocity_eci_m_per_s.x += target.velocity_eci_m_per_s.x;
      relative_velocity_eci_m_per_s.y += target.velocity_eci_m_per_s.y;
      relative_velocity_eci_m_per_s.z += target.velocity_eci_m_per_s.z;
    }
    const float omega_deg_per_sec_cached = foundation::ComputeRelativeAngularRateDegPerSec(
        los, relative_velocity_eci_m_per_s);

    const SbirsTargetState state = target_states_[target.target_id];
    const bool is_locked = nfov_scheduler_.IsLocked(target.target_id) &&
                           (state == SbirsTargetState::kStrictTruthAssistedTracking ||
                            state == SbirsTargetState::kSensorLikeTruthAssistedTracking ||
                            state == SbirsTargetState::kEstimatedTracking);
    if (is_locked) {
      // 单镜筒轮转（2026-09-02）：锁定目标不在此内联处理——几何/SNR 上下文暂存，
      // 交由周期末尾的 NFOV 分时轮转阶段统一量测（同帧免费多跟在彼处判定）。
      LockedTargetContext& context = locked_contexts[target.target_id];
      context.target = &target;
      context.azimuth_deg = azimuth_deg;
      context.elevation_deg = elevation_deg;
      context.sensor_azimuth_deg = sensor_azimuth_deg;
      context.sensor_elevation_deg = sensor_elevation_deg;
      context.range_m = range_m;
      context.snr = snr;
      context.received_power_w = received_power_w;
      context.signal_energy_j = signal_energy_j;
      context.omega_deg_per_sec = omega_deg_per_sec_cached;
      context.max_detection_range_m = max_detection_range_m;
      continue;
    }

    const bool in_wfov = InRectangularFov(sensor_azimuth_deg, sensor_elevation_deg,
                                           actual_scan_azimuth_sensor_deg,
                                           actual_scan_elevation_sensor_deg, mission.wide_field_fov_az_deg,
                                           mission.wide_field_fov_el_deg);
    const bool wfov_snr_gate_passed = snr >= policy.detection.wide_min_snr_linear;
    if (SBIRS_ACCEPTANCE_LOG_ENABLED()) {
      std::string geometry = "卫星ID=" + std::to_string(satellite_entity_id_);
      geometry += " 目标ID=" + std::to_string(target.target_id);
      geometry += " 阶段=WFOV 斜距=" + oneq::logging::FormatF(range_m, 1) + "m";
      geometry += " LOS(ECI)=(" + oneq::logging::FormatF(los.x, 3) + "," +
                  oneq::logging::FormatF(los.y, 3) + "," +
                  oneq::logging::FormatF(los.z, 3) + ")m";
      geometry += " 几何输出角(ECI)=(" + oneq::logging::FormatF(azimuth_deg, 6) + "," +
                  oneq::logging::FormatF(elevation_deg, 6) + ")deg";
      geometry += " 视场门输入角(传感器系)=(" +
                  oneq::logging::FormatF(sensor_azimuth_deg, 6) + "," +
                  oneq::logging::FormatF(sensor_elevation_deg, 6) + ")deg";
      geometry += " 扫描中心=(" + oneq::logging::FormatF(actual_scan_azimuth_sensor_deg, 6) +
                  "," + oneq::logging::FormatF(actual_scan_elevation_sensor_deg, 6) + ")deg";
      geometry += std::string(" 视场门结果=") + (in_wfov ? "通过" : "排除");
      SBIRS_INTERNAL_INTERFACE_ITEM(sim_time_sec, input.cycle_index,
                                    "红外场景几何与视场门控交互接口测试", geometry);

      std::string detection_interface = "卫星ID=" + std::to_string(satellite_entity_id_);
      detection_interface += " 目标ID=" + std::to_string(target.target_id);
      detection_interface += " 阶段=WFOV 目标辐射强度=" +
                             oneq::logging::FormatSci(target.radiant_intensity_w_per_sr) + "W/sr";
      detection_interface += " 大气透过率=" + oneq::logging::FormatF(path_transmittance, 6);
      detection_interface += " 接收功率=" + oneq::logging::FormatSci(received_power_w) + "W";
      detection_interface += " 信号能量=" + oneq::logging::FormatSci(signal_energy_j) + "J";
      detection_interface += " 光子噪声=" +
                             oneq::logging::FormatSci(noise_statistics.photon_noise_w) + "J";
      detection_interface += " 热噪声=" +
                             oneq::logging::FormatSci(noise_statistics.thermal_noise_w) + "J";
      detection_interface += " 读出噪声=" +
                             oneq::logging::FormatSci(noise_statistics.readout_noise_w) + "J";
      detection_interface +=
          " 有效噪声能量=" + oneq::logging::FormatSci(effective_noise_w) + "J";
      detection_interface += " 辐射噪声模块输出SNR=" + oneq::logging::FormatF(snr, 6);
      detection_interface += " 判决模块输入SNR=" + oneq::logging::FormatF(snr, 6);
      detection_interface += " 门限=" +
                             oneq::logging::FormatF(policy.detection.wide_min_snr_linear, 6);
      detection_interface +=
          std::string(" SNR门结果=") + (wfov_snr_gate_passed ? "通过" : "拒绝");
      detection_interface += std::string(" 视场门结果=") + (in_wfov ? "通过" : "拒绝");
      detection_interface += std::string(" 综合准入结果=") +
                             (in_wfov && wfov_snr_gate_passed ? "通过" : "拒绝");
      SBIRS_INTERNAL_INTERFACE_ITEM(
          sim_time_sec, input.cycle_index, "红外辐射噪声与检测判决交互接口测试",
          detection_interface);
    }

    if (!in_wfov) {
      // cross-cue 旁路（2026-09-01）：自星宽场几何门外但有外部引导 → 构造外部候选，
      // 不走排除/释放（修订 3 条 1：发现星自家链路不变，此处只补受话星路径）。
      if (external_cue_by_target.count(target.target_id) != 0U) {
        make_external_cue_candidate(target, azimuth_deg, elevation_deg, range_m, snr,
                                    max_detection_range_m, omega_deg_per_sec_cached);
        continue;
      }
      // 规则 13b：视场排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
      // 门内归因：视场门按越界轴细分（az/el/both），并补相对扫描中心的差值
      //（与 InRectangularFov 同基准、同参考系：传感器系；半视场为门限）。
      const float az_delta_deg =
          std::fabs(AzimuthDelta(sensor_azimuth_deg, actual_scan_azimuth_sensor_deg));
      const float el_delta_deg = std::fabs(sensor_elevation_deg - actual_scan_elevation_sensor_deg);
      const float half_wfov_az_deg = 0.5f * mission.wide_field_fov_az_deg;
      const float half_wfov_el_deg = 0.5f * mission.wide_field_fov_el_deg;
      const bool az_out = az_delta_deg > half_wfov_az_deg;
      const bool el_out = el_delta_deg > half_wfov_el_deg;
      const session::SbirsIssueCause wfov_cause =
          az_out && el_out
              ? session::SbirsIssueCause::kBothAxesOutside
              : (az_out ? session::SbirsIssueCause::kAzOutside
                        : session::SbirsIssueCause::kElOutside);
      result.issues.push_back(MakeExclusionIssue(
          session::codes::kTargetOutOfWfov,
          "target_id=" + std::to_string(target.target_id) + "; sensor_az/el (" +
              FormatFloat(sensor_azimuth_deg) + "," + FormatFloat(sensor_elevation_deg) +
              ") outside scan center (" + FormatFloat(actual_scan_azimuth_sensor_deg) + "," +
              FormatFloat(actual_scan_elevation_sensor_deg) + ") fov " +
              FormatFloat(mission.wide_field_fov_az_deg) + "x" +
              FormatFloat(mission.wide_field_fov_el_deg) + " az_delta_deg=" +
              FormatFloat(az_delta_deg) + " el_delta_deg=" + FormatFloat(el_delta_deg),
          wfov_cause,
          static_cast<std::ptrdiff_t>(target_idx)));
      ++excluded_out_of_wfov;
      target_states_[target.target_id] = SbirsTargetState::kUndetected;
      wfov_consecutive_hits_.erase(target.target_id);
      nfov_scheduler_.Release(target.target_id);
      pointing_coordinator_.ReleaseTarget(target.target_id);
      cue_predictor_.Release(target.target_id);
      tracking_coordinator_.ReleaseTarget(target.target_id);
      continue;
    }
    if (!wfov_snr_gate_passed) {
      // cross-cue 旁路（2026-09-01）：自星宽场 SNR 门外但有外部引导 → 构造外部候选。
      // （外部路径的 SNR 语义：候选优先级仍用物理真值链 SNR；捕获门在 NFOV 段另判。）
      if (external_cue_by_target.count(target.target_id) != 0U) {
        make_external_cue_candidate(target, azimuth_deg, elevation_deg, range_m, snr,
                                    max_detection_range_m, omega_deg_per_sec_cached);
        continue;
      }
      // 规则 13b：WFOV SNR 门排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
      // 门内归因：SNR 门为聚合门，反事实判定主因（见 ClassifyWfovSnrExclusionCause；
      // signature_required 已在循环外计算）。
      const double signature_actual = std::max(0.0, target.radiant_intensity_w_per_sr);
      const session::SbirsIssueCause snr_cause = ClassifyWfovSnrExclusionCause(
          range_m, path_transmittance, signature_actual, snr_signature_required);
      result.issues.push_back(MakeExclusionIssue(
          session::codes::kTargetSnrBelowThreshold,
          "target_id=" + std::to_string(target.target_id) +
              "; snr_linear=" + FormatSnr(snr) + " below wide_min=" +
              FormatSnr(policy.detection.wide_min_snr_linear) + " range_m=" +
              std::to_string(static_cast<std::int64_t>(range_m)) + " transmittance=" +
              FormatSnr(path_transmittance) + " d_max_m=" +
              std::to_string(static_cast<std::int64_t>(max_detection_range_m)),
          snr_cause,
          static_cast<std::ptrdiff_t>(target_idx)));
      ++excluded_snr_below;
      target_states_[target.target_id] = SbirsTargetState::kUndetected;
      wfov_consecutive_hits_.erase(target.target_id);
      nfov_scheduler_.Release(target.target_id);
      pointing_coordinator_.ReleaseTarget(target.target_id);
      cue_predictor_.Release(target.target_id);
      tracking_coordinator_.ReleaseTarget(target.target_id);
      continue;
    }

    SbirsCandidate candidate;
    candidate.target = &target;
    candidate.azimuth_deg = azimuth_deg;
    candidate.elevation_deg = elevation_deg;
    candidate.range_m = range_m;  // 真值距离：调度优先级用（design 2.6）
    candidate.max_detection_range_m = max_detection_range_m;
    // 2.10 WFOV 带误差位置：施加 5 类物理误差（高斯随机 + 折射 + 滞后）。
    // 复用上方已算的目标角速度 omega_deg_per_sec_cached。
    const foundation::SbirsErrorBearing bearing = foundation::ApplyAngularErrorModel(
        policy.error_model, &wfov_measurement_random_source_, azimuth_deg, elevation_deg, range_m,
        /*relative_angular_rate_deg_per_sec=*/omega_deg_per_sec_cached);
    candidate.measured_azimuth_deg = bearing.azimuth_deg;
    candidate.measured_elevation_deg = bearing.elevation_deg;
    candidate.measured_range_m = bearing.range_m;
    candidate.relative_angular_rate_deg_per_sec = omega_deg_per_sec_cached;
    candidate.measurement_time_s = compute_crossing_time_s(sensor_azimuth_deg);
    const SbirsCuePrediction cue_prediction =
        cue_predictor_.Update(target.target_id, bearing.azimuth_deg, bearing.elevation_deg,
                              input.dt_sec, mission.narrow_cue_latency_s);
    candidate.command_azimuth_deg = cue_prediction.command_azimuth_deg;
    candidate.command_elevation_deg = cue_prediction.command_elevation_deg;
    // cue 延迟外推：narrow_cue_latency_s 期间目标与卫星都继续运动，真值 az/el 需按
    // 延迟后相对几何重算：(p_target + v_target·τ) − (p_satellite + v_satellite·τ)。
    const float cue_latency_s = mission.narrow_cue_latency_s;
    if (cue_latency_s > 0.0f) {
      session::SbirsVector3M predicted_position;
      predicted_position.x = target.position_eci_m.x;
      predicted_position.y = target.position_eci_m.y;
      predicted_position.z = target.position_eci_m.z;
      if (target.has_velocity_eci_m_per_s) {
        predicted_position.x += target.velocity_eci_m_per_s.x * cue_latency_s;
        predicted_position.y += target.velocity_eci_m_per_s.y * cue_latency_s;
        predicted_position.z += target.velocity_eci_m_per_s.z * cue_latency_s;
      }
      session::SbirsVector3M predicted_satellite_position;
      predicted_satellite_position.x =
          satellite_position_eci_m.x + satellite_velocity_eci_m.x * cue_latency_s;
      predicted_satellite_position.y =
          satellite_position_eci_m.y + satellite_velocity_eci_m.y * cue_latency_s;
      predicted_satellite_position.z =
          satellite_position_eci_m.z + satellite_velocity_eci_m.z * cue_latency_s;
      const session::SbirsVector3M predicted_los =
          foundation::Subtract(predicted_position, predicted_satellite_position);
      candidate.delayed_truth_azimuth_deg = foundation::ComputeAzimuthDeg(predicted_los);
      candidate.delayed_truth_elevation_deg = foundation::ComputeElevationDeg(predicted_los);
    } else {
      candidate.delayed_truth_azimuth_deg = azimuth_deg;
      candidate.delayed_truth_elevation_deg = elevation_deg;
    }
    candidate.snr = snr;
    candidates.push_back(candidate);
    // cross-cue 外发数据源（修订 7）：本星自星宽场候选的带误差量测（与 E2 宽场疑似
    // 事件同源同量测）。外部引导候选不重复进此列表（受话不递话）。
    // VS2015/C++11：带 NSDMI 的结构体非聚合，逐成员赋值（同组件层 SbirsExternalCue）。
    session::SbirsWideCueMeasurement wide_cue_measurement;
    wide_cue_measurement.target_id = target.target_id;
    wide_cue_measurement.azimuth_deg = candidate.measured_azimuth_deg;
    wide_cue_measurement.elevation_deg = candidate.measured_elevation_deg;
    wide_cue_measurement.measured_range_m = candidate.measured_range_m;
    result.wide_cue_measurements.push_back(wide_cue_measurement);
    // 宽窄切换前置条件（3.2.1.3.2.1）：WFOV 四门全部通过计一次连续命中；
    // 门失败/目标消失分支已清零，进入跟踪时在捕获处清零。
    ++wfov_consecutive_hits_[target.target_id];
    if (SBIRS_DUAL_ACCEPTANCE_LOG_ENABLED()) {
      // 中译：宽视场疑似目标事件（量测角/定位误差/距离/SNR/能量/LLA）。
      // 标识：验收判定标准 第5/11项 逐目标行（第6项已上移至逐目标公共段、第7项改由
      //       输出帧统一登记，见 register_detectability）；
      //       az/el_error = 带误差量测角 − 真值角（方位最短角差）。
      const double az_err = AzimuthDelta(candidate.measured_azimuth_deg, azimuth_deg);
      const double el_err = candidate.measured_elevation_deg - elevation_deg;
      const double kDegToRad = 0.017453292519943295;
      // 第5项 探测角度计算：实体相对卫星的方位/俯仰（ECI、弧度制；身份字段
      // 相对卫星ID= 随行——§0 身份规则，双星场景可辨哪颗星）。
      std::string angle = "相对卫星ID=" + std::to_string(satellite_entity_id_);
      angle += " 目标ID=" + std::to_string(target.target_id);
      angle += " 量测方位/俯仰(ECI)=(" +
               oneq::logging::FormatF(candidate.measured_azimuth_deg * kDegToRad, 8) + "," +
               oneq::logging::FormatF(candidate.measured_elevation_deg * kDegToRad, 8) + ")rad";
      SBIRS_ACCEPTANCE_ITEM(sim_time_sec, input.cycle_index, "探测角度计算功能测试", angle);
      std::string wfov_angle = "卫星ID=" + std::to_string(satellite_entity_id_);
      wfov_angle += " 目标ID=" + std::to_string(target.target_id);
      wfov_angle += " 宽视场目标视线方位/俯仰(ECI)=(" +
                    oneq::logging::FormatF(candidate.measured_azimuth_deg * kDegToRad, 8) + "," +
                    oneq::logging::FormatF(candidate.measured_elevation_deg * kDegToRad, 8) +
                    ")rad";
      SBIRS_DUAL_ACCEPTANCE_ITEM(sim_time_sec, input.cycle_index, "宽视场扫描探测功能测试",
                                 wfov_angle);
      // 第11项 红外预警卫星大幅面扫描与探测：扫描幅宽（ECI rad）/目标辐射能量链/
      // SNR/探测概率/检测标志（评审 2026-08-26 条2：Pd 复用雷达公共层 Marcum Q 模型
      // （Swerling0 单脉冲），Pfa 由宽场门限系数 k 按高斯尾部 Pfa=0.5·erfc(k/√2) 反推）。
      std::string energy = "卫星ID=" + std::to_string(satellite_entity_id_);
      energy += " 扫描幅宽az/el(ECI)=(" +
                oneq::logging::FormatF(mission.wide_field_fov_az_deg * kDegToRad, 6) + "," +
                oneq::logging::FormatF(mission.wide_field_fov_el_deg * kDegToRad, 6) + ")rad";
      energy += " 目标ID=" + std::to_string(target.target_id);
      const double snr_db_wide = 10.0 * std::log10(std::max(snr, 1.0e-12));
      energy += " 接收功率=" + oneq::logging::FormatSci(received_power_w) + "W";
      energy += " 信号能量=" + oneq::logging::FormatSci(signal_energy_j) + "J";
      energy += " SNR(dB)=" + oneq::logging::FormatF(snr_db_wide, 3) + "dB";
      const double pfa_wide =
          0.5 * std::erfc(static_cast<double>(policy.detection.wide_min_snr_linear) /
                          std::sqrt(2.0));
      const double pd_wide = oneq::common::radar::RadarEquations::ComputeDetectionProbability(
          static_cast<float>(snr_db_wide), static_cast<float>(pfa_wide),
          oneq::common::radar::SwerlingModel::kSwerling0, 1);
      energy += " 探测概率=" + oneq::logging::FormatF(pd_wide, 4);
      energy += " 检测标志=是";
      SBIRS_DUAL_ACCEPTANCE_ITEM(
          sim_time_sec, input.cycle_index, "红外预警卫星大幅面扫描与探测功能测试", energy);
      WriteSbirsAngleError(satellite_entity_id_, sim_time_sec, input.cycle_index,
                           target.target_id, az_err, el_err);
    }
    if (target_states_[target.target_id] != SbirsTargetState::kAwaitingNfovAcquisition) {
      target_states_[target.target_id] = SbirsTargetState::kWideCandidate;
    }
  }

  // 评审 2026-08-26 条3 + 验收判定标准 第14项·其二：宽场疑似目标完整列表（规范口径：
  // 只写 卫星ID 与列表，不写 连续命中/序列确认；仅本星宽场检出成行——cross-cue 外部
  // 候选不是本星宽场检出，检出为空时整行不写，不写空列表行）。
  if (SBIRS_DUAL_ACCEPTANCE_LOG_ENABLED()) {
    std::string suspect_ids;
    for (const SbirsCandidate& candidate : candidates) {
      if (candidate.cue_source_satellite_entity_id >= 0) {
        continue;  // cross-cue 外部候选不是本星宽场检出，不进宽场疑似列表（验收行口径不变）
      }
      if (!suspect_ids.empty()) {
        suspect_ids += ",";
      }
      suspect_ids += std::to_string(candidate.target->target_id);
    }
    if (!suspect_ids.empty()) {
      std::string joint_list = "卫星ID=" + std::to_string(satellite_entity_id_) +
                               " 宽场疑似列表=[" + suspect_ids + "]";
      SBIRS_DUAL_ACCEPTANCE_ITEM(sim_time_sec, input.cycle_index,
                                 "宽窄视场联合探测功能测试", joint_list);
    }
  }

  // 第7项（修改）目标可探测性与动态参数：登记点在本星输出帧——宽场候选、窄场锁定
  // 跟踪、cross-cue 引导的检出统一覆盖（2026-09-02 裁定方案A：跟踪期与递话路径同样
  // 登记，否则双星行只在两星都未锁定的首拍才可能凑齐）。斜距取归属层 range，量测角
  // 取输出角（ECI rad，与窄场状态行同口径）。双星定位行/单星顺延行的拼装见
  // SbirsAcceptanceRecords（WriteSbirsTargetDetectability）。
  const auto register_detectability = [&]() {
    if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
      return;
    }
    for (const SbirsPipelineDetection& detection : result.detections) {
      if (!detection.record.detected) {
        continue;  // 捕获失败/调度跳过等非检出记录不成行
      }
      WriteSbirsTargetDetectability(
          satellite_entity_id_, sim_time_sec, input.cycle_index,
          detection.attribution.target_id, detection.attribution.estimated_range_m,
          detection.record.azimuth_rad, detection.record.elevation_rad,
          detection.record.observation_stage != output::SbirsObservationStage::kWideFieldSearch,
          satellite_ecef.x_m,
          satellite_ecef.y_m, satellite_ecef.z_m, gmst_rad);
    }
  };

  if (mission.work_mode == config::SbirsWorkMode::kWideSearch) {
    for (const SbirsCandidate& candidate : candidates) {
      SbirsPipelineDetection detection;
      detection.record.detection_id = next_detection_id_++;
      detection.record.measurement_time_s = candidate.measurement_time_s;
      detection.record.azimuth_rad = ToEciAzimuthRad(candidate.measured_azimuth_deg);
      detection.record.elevation_rad = ToEciElevationRad(candidate.measured_elevation_deg);
      detection.record.infrared_snr_linear = static_cast<float>(candidate.snr);
      detection.record.observation_stage = output::SbirsObservationStage::kWideFieldSearch;
      detection.record.detected = true;
      detection.attribution.detection_id = detection.record.detection_id;
      detection.attribution.target_id = candidate.target->target_id;
      detection.attribution.target_name = candidate.target->target_name;
      detection.attribution.estimated_range_m = static_cast<float>(candidate.range_m);
      detection.attribution.measured_range_m = static_cast<float>(candidate.measured_range_m);
      detection.attribution.max_detection_range_m =
          static_cast<float>(candidate.max_detection_range_m);
      detection.attribution.cue_source_satellite_entity_id =
          candidate.cue_source_satellite_entity_id;
      detection.attribution.nfov_channel_id = -1;
      result.detections.push_back(detection);
    }
    register_detectability();
    log_cycle_summary();
    return result;
  }

  // ===== NFOV 单镜筒分时轮转阶段（2026-09-02，冻结契约 sbirs-nfov-shared-pointing） =====
  // 窄场只有一个镜筒：锁定集合无配置上限，每周期按固定顺序轮转服务一个目标（转动
  // 期间帧作废，稳定后按剩余时长计帧）；当前视场内的其余锁定目标同帧免费多跟（共享
  // 同一批稳定帧）；分离目标轮空周期记门失败，连续超过 nfov_tracking_gate_loss_cycles
  // 即丢锁释放——可同时保持的精跟条数由轮转物理涌现，不再由配置通道数决定。
  SbirsPointingActuatorConfig pointing_config;
  pointing_config.max_slew_rate_deg_per_sec = mission.narrow_pointing_max_slew_rate_deg_per_sec;
  pointing_config.settle_tolerance_deg = mission.narrow_pointing_settle_tolerance_deg;
  std::set<std::uint64_t> processed_target_ids;
  std::set<std::uint64_t> blocked_target_ids;

  const auto append_wfov_detection = [&](const SbirsCandidate& candidate) {
    SbirsPipelineDetection detection;
    detection.record.detection_id = next_detection_id_++;
    detection.record.measurement_time_s = candidate.measurement_time_s;
    detection.record.azimuth_rad = ToEciAzimuthRad(candidate.measured_azimuth_deg);
    detection.record.elevation_rad = ToEciElevationRad(candidate.measured_elevation_deg);
    detection.record.infrared_snr_linear = static_cast<float>(candidate.snr);
    detection.record.observation_stage = output::SbirsObservationStage::kWideFieldSearch;
    detection.record.detected = true;
    detection.attribution.detection_id = detection.record.detection_id;
    detection.attribution.target_id = candidate.target->target_id;
    detection.attribution.target_name = candidate.target->target_name;
    detection.attribution.estimated_range_m = static_cast<float>(candidate.range_m);
    detection.attribution.measured_range_m = static_cast<float>(candidate.measured_range_m);
    detection.attribution.cue_source_satellite_entity_id =
        candidate.cue_source_satellite_entity_id;
    detection.attribution.nfov_channel_id = 0;
    result.detections.push_back(detection);
  };
  const auto append_acquisition_failure = [&](const SbirsCandidate& candidate,
                                               attribution::SbirsCaptureFailureReason reason) {
    SbirsPipelineDetection detection;
    detection.record.detection_id = next_detection_id_++;
    detection.record.measurement_time_s = cycle_end_time_s;
    detection.record.azimuth_rad = ToEciAzimuthRad(candidate.measured_azimuth_deg);
    detection.record.elevation_rad = ToEciElevationRad(candidate.measured_elevation_deg);
    detection.record.infrared_snr_linear = static_cast<float>(candidate.snr);
    detection.record.observation_stage = output::SbirsObservationStage::kNarrowFieldAcquisition;
    detection.record.detected = false;
    detection.attribution.detection_id = detection.record.detection_id;
    detection.attribution.target_id = candidate.target->target_id;
    detection.attribution.target_name = candidate.target->target_name;
    detection.attribution.estimated_range_m = static_cast<float>(candidate.range_m);
    detection.attribution.cue_source_satellite_entity_id =
        candidate.cue_source_satellite_entity_id;
    detection.attribution.nfov_channel_id = 0;
    detection.attribution.capture_failure_reason = reason;
    result.detections.push_back(detection);
    if (SBIRS_ACCEPTANCE_LOG_ENABLED()) {
      SBIRS_INTERNAL_INTERFACE_ITEM(
          sim_time_sec, input.cycle_index, "红外宽场候选与窄场调度交互接口测试",
          "卫星ID=" + std::to_string(satellite_entity_id_) + " 目标ID=" +
              std::to_string(candidate.target->target_id) + " 捕获结果=失败 失败原因枚举=" +
              std::to_string(static_cast<int>(reason)));
    }
  };

  std::map<std::uint64_t, const SbirsCandidate*> candidate_by_target;
  for (const SbirsCandidate& candidate : candidates) {
    candidate_by_target[candidate.target->target_id] = &candidate;
  }

  // 宽窄切换前置条件（3.2.1.3.2.1）：连续命中达到阈值才允许进入 NFOV 调度。
  // 默认阈值 1 下候选创建当周期即计数 >=1，过滤恒通过，与既有单次命中调度逐位一致。
  const int required_consecutive_hits =
      std::max(1, policy.scheduler.wide_to_narrow_required_consecutive_hits);
  std::vector<SbirsCandidate> new_candidates;
  for (const SbirsCandidate& candidate : candidates) {
    const std::uint64_t target_id = candidate.target->target_id;
    if (nfov_scheduler_.IsLocked(target_id)) {
      if (SBIRS_ACCEPTANCE_LOG_ENABLED()) {
        SBIRS_INTERNAL_INTERFACE_ITEM(
            sim_time_sec, input.cycle_index, "红外宽场候选与窄场调度交互接口测试",
            "卫星ID=" + std::to_string(satellite_entity_id_) + " 目标ID=" +
                std::to_string(target_id) + " 候选状态=已锁定 入列=否");
      }
      continue;
    }
    const auto hits_entry = wfov_consecutive_hits_.find(target_id);
    const int consecutive_hits =
        hits_entry == wfov_consecutive_hits_.end() ? 0 : static_cast<int>(hits_entry->second);
    if (consecutive_hits < required_consecutive_hits) {
      if (SBIRS_ACCEPTANCE_LOG_ENABLED()) {
        SBIRS_INTERNAL_INTERFACE_ITEM(
            sim_time_sec, input.cycle_index, "红外宽场候选与窄场调度交互接口测试",
            "卫星ID=" + std::to_string(satellite_entity_id_) + " 目标ID=" +
                std::to_string(target_id) + " 连续命中=" + std::to_string(consecutive_hits) +
                " 要求连续命中=" + std::to_string(required_consecutive_hits) +
                " 候选状态=等待积累 入列=否");
      }
      continue;
    }
    new_candidates.push_back(candidate);
  }
  // 新候选全部入列（design 2.6 优先级排序只决定入列顺序；单镜筒无并发截断）。
  const std::vector<const SbirsCandidate*> selected_candidates =
      nfov_scheduler_.SelectForAcquisition(new_candidates);
  for (const SbirsCandidate* selected : selected_candidates) {
    const std::uint64_t target_id = selected->target->target_id;
    nfov_scheduler_.Acquire(target_id, selected->cue_source_satellite_entity_id);
    target_states_[target_id] = SbirsTargetState::kAwaitingNfovAcquisition;
    if (SBIRS_ACCEPTANCE_LOG_ENABLED()) {
      std::string scheduling = "卫星ID=" + std::to_string(satellite_entity_id_);
      scheduling += " 目标ID=" + std::to_string(target_id);
      scheduling += " 候选量测时刻=" + oneq::logging::FormatF(selected->measurement_time_s, 6) +
                    "s";
      scheduling += " 候选测角=(" + oneq::logging::FormatF(selected->measured_azimuth_deg, 6) +
                    "," + oneq::logging::FormatF(selected->measured_elevation_deg, 6) + ")deg";
      scheduling += " SNR=" + oneq::logging::FormatF(selected->snr, 6);
      scheduling += " 连续命中=" + std::to_string(wfov_consecutive_hits_[target_id]);
      scheduling += " 引导来源卫星ID=" +
                    std::to_string(selected->cue_source_satellite_entity_id);
      scheduling += " 调度结果=已选中 指令方位/俯仰=(" +
                    oneq::logging::FormatF(selected->command_azimuth_deg, 6) + "," +
                    oneq::logging::FormatF(selected->command_elevation_deg, 6) + ")deg";
      SBIRS_INTERNAL_INTERFACE_ITEM(sim_time_sec, input.cycle_index,
                                    "红外宽场候选与窄场调度交互接口测试", scheduling);
    }
  }

  // 轮转服务目标与共享镜筒推进（一次只服务一个方向；帧数按稳定时长计）。
  const std::vector<std::uint64_t> engaged_ids = nfov_scheduler_.LockedTargetIds();
  bool nfov_pointing_valid = false;
  bool has_served_target = false;
  std::uint64_t served_target_id = 0U;
  SbirsPointingAdvanceResult pointing_result;
  float actual_pointing_azimuth_deg = 0.0f;
  float actual_pointing_elevation_deg = 0.0f;
  int settled_frames = 0;
  // 被服务 Estimated 目标的滤波预测缓存：PredictTarget 有状态（原地推进滤波），
  // 命令合成与跟踪段必须共用同一次预测，否则该目标滤波每周期被推进两次。
  bool served_prediction_valid = false;
  SbirsTrackingPredictionResult served_prediction;
  if (!engaged_ids.empty() &&
      pointing_coordinator_.EnsureActuatorInitialized(boresight_chain.EciLosOfSensorPointing(
          nominal_scan_azimuth_sensor_deg, nominal_scan_elevation_sensor_deg))) {
    served_target_id = engaged_ids[nfov_rotation_step_ % engaged_ids.size()];
    has_served_target = true;
    ++nfov_rotation_step_;
    const SbirsTargetState served_state = target_states_[served_target_id];
    float command_azimuth_deg = 0.0f;
    float command_elevation_deg = 0.0f;
    bool command_valid = false;
    if (served_state == SbirsTargetState::kAwaitingNfovAcquisition) {
      const auto candidate_it = candidate_by_target.find(served_target_id);
      if (candidate_it != candidate_by_target.end()) {
        command_azimuth_deg = candidate_it->second->command_azimuth_deg;
        command_elevation_deg = candidate_it->second->command_elevation_deg;
        command_valid = true;
      }
    } else {
      const auto context_it = locked_contexts.find(served_target_id);
      if (context_it != locked_contexts.end()) {
        command_azimuth_deg = context_it->second.azimuth_deg;
        command_elevation_deg = context_it->second.elevation_deg;
        if (served_state == SbirsTargetState::kEstimatedTracking) {
          served_prediction = tracking_coordinator_.PredictTarget(
              served_target_id, policy, input.dt_sec, satellite_position_eci_m);
          served_prediction_valid = true;
          command_azimuth_deg = served_prediction.output_azimuth_deg;
          command_elevation_deg = served_prediction.output_elevation_deg;
        }
        command_valid = true;
      }
    }
    if (command_valid) {
      // NFOV 命令（ECI az/el）旋入传感器系并限位钳制，再经链合成 ECI 单位向量驱动
      // 共享 actuator（限速转向在 ECI 单位向量域，参考系无关；限位够不到时停在边缘）。
      float sensor_command_azimuth_deg = 0.0f;
      float sensor_command_elevation_deg = 0.0f;
      boresight_chain.SensorAzElOfEciVector(
          LosFromAzimuthElevation(command_azimuth_deg, command_elevation_deg),
          &sensor_command_azimuth_deg, &sensor_command_elevation_deg);
      SbirsBoresightChain::ClampToScanLimits(config_.session.orientation.sensor_scan_limits_deg,
                                             &sensor_command_azimuth_deg,
                                             &sensor_command_elevation_deg);
      const session::SbirsVector3M command_los = boresight_chain.EciLosOfSensorPointing(
          sensor_command_azimuth_deg, sensor_command_elevation_deg);
      if (served_state == SbirsTargetState::kAwaitingNfovAcquisition) {
        pointing_result = pointing_coordinator_.AdvanceAcquisition(
            served_target_id, command_los, input.dt_sec, pointing_config);
      } else {
        pointing_result =
            pointing_coordinator_.AdvanceTracking(command_los, input.dt_sec, pointing_config);
      }
      nfov_pointing_valid = EffectiveNfovPointing(
          pointing_coordinator_, disturbance_parameters, pointing_result.current_los,
          boresight_chain, mission.narrow_pointing_settle_error_deg,
          &actual_pointing_azimuth_deg, &actual_pointing_elevation_deg);
      settled_frames =
          std::max(0, static_cast<int>(std::lround(
                          static_cast<double>(mission.frame_rate_hz) *
                           pointing_result.settled_duration_sec)));
      if (SBIRS_ACCEPTANCE_LOG_ENABLED()) {
        const char* pointing_status =
            pointing_result.status == SbirsPointingAdvanceStatus::kSettled
                ? "已稳定"
                : (pointing_result.status == SbirsPointingAdvanceStatus::kSlewing
                       ? "转动中"
                       : (pointing_result.status == SbirsPointingAdvanceStatus::kTimedOut
                              ? "超时"
                              : "已拒绝"));
        SBIRS_INTERNAL_INTERFACE_ITEM(
            sim_time_sec, input.cycle_index, "红外宽场候选与窄场调度交互接口测试",
            "卫星ID=" + std::to_string(satellite_entity_id_) + " 当前服务目标=" +
                std::to_string(served_target_id) + " 指令方位/俯仰=(" +
                oneq::logging::FormatF(command_azimuth_deg, 6) + "," +
                oneq::logging::FormatF(command_elevation_deg, 6) + ")deg 指向状态=" +
                pointing_status + " 剩余角=" +
                oneq::logging::FormatF(pointing_result.remaining_angle_deg, 6) + "deg");
      }
    }
  }

  // 首次捕获尝试（轮转窗口内：转向→稳定→捕获判定；捕获命令 = 实际指向，settle 误差
  // 经请求字段入捕获门，与历史语义一致）。
  const auto attempt_capture = [&](const SbirsCandidate& selected) {
    const std::uint64_t target_id = selected.target->target_id;
    float capture_command_azimuth_deg = 0.0f;
    float capture_command_elevation_deg = 0.0f;
    if (!EffectiveNfovPointing(pointing_coordinator_, disturbance_parameters,
                               pointing_result.current_los, boresight_chain, 0.0f,
                               &capture_command_azimuth_deg, &capture_command_elevation_deg)) {
      log_acceptance_release(target_id, "pointing_unavailable", 0U);
      nfov_scheduler_.Release(target_id);
      pointing_coordinator_.ReleaseTarget(target_id);
      target_states_[target_id] = SbirsTargetState::kWideCandidate;
      blocked_target_ids.insert(target_id);
      append_acquisition_failure(selected,
                                 attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed);
      return;
    }
    SbirsNfovAcquisitionRequest acquisition_request;
    // 首捕窗口判定在传感器系：delayed truth（延迟真值 ECI los）经链转换。
    boresight_chain.SensorAzElOfEciVector(
        LosFromAzimuthElevation(selected.delayed_truth_azimuth_deg,
                                selected.delayed_truth_elevation_deg),
        &acquisition_request.delayed_truth_azimuth_deg,
        &acquisition_request.delayed_truth_elevation_deg);
    acquisition_request.command_azimuth_deg = capture_command_azimuth_deg;
    acquisition_request.command_elevation_deg = capture_command_elevation_deg;
    acquisition_request.pointing_settle_error_deg = mission.narrow_pointing_settle_error_deg;
    acquisition_request.field_of_view_azimuth_deg = mission.narrow_field_fov_az_deg;
    acquisition_request.field_of_view_elevation_deg = mission.narrow_field_fov_el_deg;
    acquisition_request.snr = selected.snr;
    acquisition_request.minimum_snr_linear = policy.detection.narrow_min_snr_linear;
    const bool captured = IsNfovAcquisitionEligible(acquisition_request);
    if (SBIRS_ACCEPTANCE_LOG_ENABLED()) {
      SBIRS_INTERNAL_INTERFACE_ITEM(
          sim_time_sec, input.cycle_index, "红外宽场候选与窄场调度交互接口测试",
          "卫星ID=" + std::to_string(satellite_entity_id_) + " 目标ID=" +
              std::to_string(target_id) + " 捕获判决输入SNR=" +
              oneq::logging::FormatF(selected.snr, 6) + " 捕获门限=" +
              oneq::logging::FormatF(policy.detection.narrow_min_snr_linear, 6) +
              " 捕获结果=" + (captured ? "成功" : "失败"));
    }
    if (SBIRS_DUAL_ACCEPTANCE_LOG_ENABLED()) {
      // 中译：窄视场首次捕获事件（卫星、目标、捕获结论、cue 预测命令角与 SNR）。
      // 标识：验收日志 E4——3.2.1.3.2.1 NFOV 捕获结论证据；窄场角按规范口径写 ECI 弧度。
      constexpr double kDegToRad = 0.017453292519943295;
      const double selected_snr_db = 10.0 * std::log10(std::max(selected.snr, 1.0e-12));
      SBIRS_DUAL_ACCEPTANCE_ITEM(
          sim_time_sec, input.cycle_index, "宽窄视场联合探测功能测试",
          "卫星ID=" + std::to_string(satellite_entity_id_) + " 目标ID=" +
              std::to_string(target_id) + " 捕获=" + std::string(captured ? "成功" : "失败") +
              " 窄场方位/俯仰(ECI)=(" +
              oneq::logging::FormatF(selected.command_azimuth_deg * kDegToRad, 8) + "," +
              oneq::logging::FormatF(selected.command_elevation_deg * kDegToRad, 8) +
              ")rad SNR(dB)=" + oneq::logging::FormatF(selected_snr_db, 3) + "dB");
    }
    if (!captured) {
      log_acceptance_release(target_id, "acquisition_ineligible", 0U);
      nfov_scheduler_.Release(target_id);
      pointing_coordinator_.ReleaseTarget(target_id);
      target_states_[target_id] = SbirsTargetState::kWideCandidate;
      blocked_target_ids.insert(target_id);
      append_acquisition_failure(selected,
                                 attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed);
      return;
    }
    pointing_coordinator_.PromoteToTracking(target_id);
    cue_predictor_.Release(target_id);
    // 进入跟踪即清零连续命中计数：丢锁回宽场后需重新积累 required 次命中才能再调度。
    wfov_consecutive_hits_.erase(target_id);
    const bool use_estimated =
        policy.tracking.tracking_mode == config::SbirsTrackingMode::kEstimated;
    if (use_estimated) {
      target_states_[target_id] = SbirsTargetState::kEstimatedTracking;
    } else if (policy.tracking.tracking_mode == config::SbirsTrackingMode::kStrictTruthAssisted) {
      target_states_[target_id] = SbirsTargetState::kStrictTruthAssistedTracking;
    } else {
      target_states_[target_id] = SbirsTargetState::kSensorLikeTruthAssistedTracking;
    }
    if (use_estimated) {
      tracking_coordinator_.InitializeTarget(target_id, *selected.target, policy.tracking,
                                             selected.measured_azimuth_deg,
                                             selected.measured_elevation_deg);
    }
    SbirsPipelineDetection detection;
    detection.record.detection_id = next_detection_id_++;
    detection.record.measurement_time_s = cycle_end_time_s;
    detection.record.azimuth_rad = ToEciAzimuthRad(selected.measured_azimuth_deg);
    detection.record.elevation_rad = ToEciElevationRad(selected.measured_elevation_deg);
    detection.record.infrared_snr_linear = static_cast<float>(selected.snr);
    detection.record.observation_stage = output::SbirsObservationStage::kNarrowFieldAcquisition;
    detection.record.detected = true;
    detection.attribution.detection_id = detection.record.detection_id;
    detection.attribution.target_id = selected.target->target_id;
    detection.attribution.target_name = selected.target->target_name;
    detection.attribution.tracking_source = TrackingSourceForMode(policy.tracking.tracking_mode);
    detection.attribution.max_detection_range_m =
        static_cast<float>(selected.max_detection_range_m);
    if (policy.tracking.tracking_mode == config::SbirsTrackingMode::kStrictTruthAssisted) {
      detection.record.azimuth_rad = ToEciAzimuthRad(selected.azimuth_deg);
      detection.record.elevation_rad = ToEciElevationRad(selected.elevation_deg);
      detection.attribution.estimated_range_m = static_cast<float>(selected.range_m);
    } else if (policy.tracking.tracking_mode ==
               config::SbirsTrackingMode::kSensorLikeTruthAssisted) {
      const foundation::SbirsErrorBearing bearing = foundation::ApplyAngularErrorModel(
          policy.error_model, &sensor_like_output_random_source_, selected.azimuth_deg,
          selected.elevation_deg, selected.range_m, selected.relative_angular_rate_deg_per_sec);
      detection.record.azimuth_rad = ToEciAzimuthRad(bearing.azimuth_deg);
      detection.record.elevation_rad = ToEciElevationRad(bearing.elevation_deg);
      detection.attribution.estimated_range_m = static_cast<float>(bearing.range_m);
    } else {
      detection.attribution.estimated_range_m = static_cast<float>(selected.range_m);
    }
    detection.attribution.nfov_channel_id = 0;
    result.detections.push_back(detection);
  };

  // 跟踪段逐目标量测：同帧免费多跟——当前视场内全部锁定目标共享 settled_frames。
  for (const std::uint64_t engaged_id : engaged_ids) {
    const SbirsTargetState state = target_states_[engaged_id];
    if (state != SbirsTargetState::kStrictTruthAssistedTracking &&
        state != SbirsTargetState::kSensorLikeTruthAssistedTracking &&
        state != SbirsTargetState::kEstimatedTracking) {
      continue;
    }
    const auto context_it = locked_contexts.find(engaged_id);
    if (context_it == locked_contexts.end()) {
      continue;
    }
    const LockedTargetContext& context = context_it->second;
    const SbirsEciSceneTarget& target = *context.target;
    const bool estimated_tracking = state == SbirsTargetState::kEstimatedTracking;
    float command_azimuth_deg = context.azimuth_deg;
    float command_elevation_deg = context.elevation_deg;
    if (estimated_tracking) {
      if (engaged_id == served_target_id && served_prediction_valid) {
        // 轮转段已为被服务目标做过本周期唯一一次滤波预测（PredictTarget 有状态），
        // 此处复用，保证与单通道时代同款"每周期一次 predict→correct"节奏。
        command_azimuth_deg = served_prediction.output_azimuth_deg;
        command_elevation_deg = served_prediction.output_elevation_deg;
      } else {
        const SbirsTrackingPredictionResult prediction = tracking_coordinator_.PredictTarget(
            engaged_id, policy, input.dt_sec, satellite_position_eci_m);
        command_azimuth_deg = prediction.output_azimuth_deg;
        command_elevation_deg = prediction.output_elevation_deg;
      }
    }
    const bool geometry_gate_passed =
        nfov_pointing_valid &&
        InRectangularFov(context.sensor_azimuth_deg, context.sensor_elevation_deg,
                         actual_pointing_azimuth_deg, actual_pointing_elevation_deg,
                         mission.narrow_field_fov_az_deg, mission.narrow_field_fov_el_deg);
    const bool snr_gate_passed = context.snr >= policy.detection.narrow_min_snr_linear;
    // 高刷新率帧数（单镜筒轮转口径）：仅稳定窗口内的时长可积分；轮空/转动周期帧数为
    // 0，该周期无融合量测（滑行）。
    const int nfov_frame_count = geometry_gate_passed && snr_gate_passed ? settled_frames : 0;
    const bool tracking_gate_passed = nfov_frame_count >= 1;
    const unsigned int gate_failure_count =
        pointing_coordinator_.RecordTrackingGateResult(engaged_id, tracking_gate_passed);
    const bool lost_due_to_tracking_gate =
        !tracking_gate_passed &&
        gate_failure_count >= policy.tracking.nfov_tracking_gate_loss_cycles;
    processed_target_ids.insert(engaged_id);

    SbirsPipelineDetection detection;
    detection.record.detection_id = next_detection_id_++;
    detection.record.measurement_time_s = cycle_end_time_s;
    detection.record.azimuth_rad = ToEciAzimuthRad(command_azimuth_deg);
    detection.record.elevation_rad = ToEciElevationRad(command_elevation_deg);
    detection.record.infrared_snr_linear = static_cast<float>(context.snr);
    detection.record.observation_stage = output::SbirsObservationStage::kNarrowFieldTrack;
    detection.record.detected = tracking_gate_passed;
    detection.attribution.detection_id = detection.record.detection_id;
    detection.attribution.target_id = engaged_id;
    detection.attribution.target_name = target.target_name;
    detection.attribution.estimated_range_m = static_cast<float>(context.range_m);
    detection.attribution.max_detection_range_m =
        static_cast<float>(context.max_detection_range_m);
    detection.attribution.tracking_source = TrackingSourceForState(state);
    detection.attribution.cue_source_satellite_entity_id = nfov_scheduler_.CueSourceOf(engaged_id);
    detection.attribution.nfov_channel_id = 0;
    detection.attribution.has_nfov_tracking_diagnostics = true;
    detection.attribution.nfov_pointing_error_deg = foundation::AngularSeparationDeg(
        context.azimuth_deg, context.elevation_deg, actual_pointing_azimuth_deg,
        actual_pointing_elevation_deg);
    detection.attribution.nfov_geometry_gate_passed = geometry_gate_passed;
    detection.attribution.nfov_snr_gate_passed = snr_gate_passed;
    detection.attribution.nfov_tracking_gate_failure_count = gate_failure_count;
    detection.attribution.nfov_tracking_coasting =
        !tracking_gate_passed && !lost_due_to_tracking_gate;

    bool lost_due_to_estimation_nis = false;
    SbirsTrackingUpdateResult tracking_result;
    // 单帧 σ 为硬件性质（与帧数无关）；融合 σ 仅在有帧时按 1/√N 衰减，滑行周期记 0。
    const double nfov_frame_sigma_deg =
        foundation::ResolveEffectiveAngularSigmaDeg(policy.error_model);
    const double nfov_fused_sigma_deg =
        nfov_frame_count >= 1
            ? nfov_frame_sigma_deg / std::sqrt(static_cast<double>(nfov_frame_count))
            : 0.0;
    detection.attribution.nfov_frame_count = nfov_frame_count;
    detection.attribution.nfov_frame_sigma_deg = static_cast<float>(nfov_frame_sigma_deg);
    detection.attribution.nfov_fused_sigma_deg = static_cast<float>(nfov_fused_sigma_deg);
    if (tracking_gate_passed && estimated_tracking) {
      tracking_result = tracking_coordinator_.CorrectTarget(
          engaged_id, policy, &estimated_measurement_random_source_, context.azimuth_deg,
          context.elevation_deg, context.range_m, context.omega_deg_per_sec,
          satellite_position_eci_m, nfov_frame_count);
      detection.record.azimuth_rad = ToEciAzimuthRad(tracking_result.output_azimuth_deg);
      detection.record.elevation_rad = ToEciElevationRad(tracking_result.output_elevation_deg);
      detection.attribution.has_estimation_nis = tracking_result.has_estimation_nis;
      detection.attribution.estimation_nis = tracking_result.estimation_nis;
      detection.attribution.estimation_nis_gate_exceeded =
          tracking_result.estimation_nis_gate_exceeded;
      lost_due_to_estimation_nis = tracking_result.lost_due_to_estimation_nis;
      detection.record.detected = !lost_due_to_estimation_nis;
    } else if (tracking_gate_passed &&
               state == SbirsTargetState::kSensorLikeTruthAssistedTracking) {
      const foundation::SbirsFusedBearingResult fused_bearing =
          foundation::ApplyAngularErrorModelFused(
              policy.error_model, &sensor_like_output_random_source_, context.azimuth_deg,
              context.elevation_deg, context.range_m, context.omega_deg_per_sec,
              nfov_frame_count);
      detection.record.azimuth_rad = ToEciAzimuthRad(fused_bearing.bearing.azimuth_deg);
      detection.record.elevation_rad = ToEciElevationRad(fused_bearing.bearing.elevation_deg);
      detection.attribution.estimated_range_m =
          static_cast<float>(fused_bearing.bearing.range_m);
    } else if (!tracking_gate_passed && estimated_tracking) {
      tracking_coordinator_.MarkMeasurementUnavailable(engaged_id);
    }
    if (SBIRS_ACCEPTANCE_LOG_ENABLED()) {
      std::string geometry = "卫星ID=" + std::to_string(satellite_entity_id_);
      geometry += " 目标ID=" + std::to_string(engaged_id);
      geometry += " 阶段=NFOV 斜距=" + oneq::logging::FormatF(context.range_m, 1) + "m";
      geometry += " 几何输出角(ECI)=(" + oneq::logging::FormatF(context.azimuth_deg, 6) + "," +
                  oneq::logging::FormatF(context.elevation_deg, 6) + ")deg";
      geometry += " 视场门输入角(传感器系)=(" +
                  oneq::logging::FormatF(context.sensor_azimuth_deg, 6) + "," +
                  oneq::logging::FormatF(context.sensor_elevation_deg, 6) + ")deg";
      geometry += std::string(" 视场门结果=") + (geometry_gate_passed ? "通过" : "排除");
      SBIRS_INTERNAL_INTERFACE_ITEM(sim_time_sec, input.cycle_index,
                                    "红外场景几何与视场门控交互接口测试", geometry);

      std::string detection_interface = "卫星ID=" + std::to_string(satellite_entity_id_);
      detection_interface += " 目标ID=" + std::to_string(engaged_id);
      detection_interface += " 阶段=NFOV 接收功率=" +
                             oneq::logging::FormatSci(context.received_power_w) + "W";
      detection_interface += " 信号能量=" +
                             oneq::logging::FormatSci(context.signal_energy_j) + "J";
      detection_interface += " 光子噪声=" +
                             oneq::logging::FormatSci(noise_statistics.photon_noise_w) + "J";
      detection_interface += " 热噪声=" +
                             oneq::logging::FormatSci(noise_statistics.thermal_noise_w) + "J";
      detection_interface += " 读出噪声=" +
                             oneq::logging::FormatSci(noise_statistics.readout_noise_w) + "J";
      detection_interface +=
          " 有效噪声能量=" + oneq::logging::FormatSci(effective_noise_w) + "J";
      detection_interface += " 辐射噪声模块输出SNR=" +
                             oneq::logging::FormatF(context.snr, 6);
      detection_interface += " 判决模块输入SNR=" +
                             oneq::logging::FormatF(context.snr, 6);
      detection_interface += " 门限=" +
                             oneq::logging::FormatF(policy.detection.narrow_min_snr_linear, 6);
      detection_interface +=
          std::string(" SNR门结果=") + (snr_gate_passed ? "通过" : "拒绝");
      detection_interface +=
          std::string(" 视场门结果=") + (geometry_gate_passed ? "通过" : "拒绝");
      detection_interface += std::string(" 稳定帧门结果=") +
                             (settled_frames >= 1 ? "通过" : "拒绝");
      detection_interface +=
          std::string(" 综合准入结果=") + (tracking_gate_passed ? "通过" : "拒绝");
      SBIRS_INTERNAL_INTERFACE_ITEM(
          sim_time_sec, input.cycle_index, "红外辐射噪声与检测判决交互接口测试",
          detection_interface);

      std::string tracking_interface = "卫星ID=" + std::to_string(satellite_entity_id_);
      tracking_interface += " 目标ID=" + std::to_string(engaged_id);
      tracking_interface += " 帧数=" + std::to_string(nfov_frame_count);
      tracking_interface += " 单帧/融合角噪声=" +
                            oneq::logging::FormatSci(nfov_frame_sigma_deg) + "/" +
                            oneq::logging::FormatSci(nfov_fused_sigma_deg) + "deg";
      if (tracking_result.has_measurement) {
        tracking_interface += " 滤波器量测输入=(" +
                              oneq::logging::FormatF(tracking_result.measurement_azimuth_deg, 6) +
                              "," +
                              oneq::logging::FormatF(tracking_result.measurement_elevation_deg, 6) +
                              ")deg";
        tracking_interface += " 量测协方差=[" +
                              oneq::logging::FormatSci(tracking_result.measurement_covariance(0, 0)) +
                              "," +
                              oneq::logging::FormatSci(tracking_result.measurement_covariance(0, 1)) +
                              ";" +
                              oneq::logging::FormatSci(tracking_result.measurement_covariance(1, 0)) +
                              "," +
                              oneq::logging::FormatSci(tracking_result.measurement_covariance(1, 1)) +
                              "]rad^2";
      }
      tracking_interface += " 滤波输出角=(" +
                            oneq::logging::FormatF(detection.record.azimuth_rad, 8) + "," +
                            oneq::logging::FormatF(detection.record.elevation_rad, 8) + ")rad";
      if (tracking_result.has_estimation_nis) {
        tracking_interface += " NIS=" + oneq::logging::FormatF(tracking_result.estimation_nis, 6);
        tracking_interface += std::string(" NIS门结果=") +
                              (tracking_result.estimation_nis_gate_exceeded ? "超门" : "通过");
      }
      tracking_interface += std::string(" 跟踪状态=") +
                            (detection.attribution.nfov_tracking_coasting ? "滑行" : "跟踪");
      SBIRS_INTERNAL_INTERFACE_ITEM(sim_time_sec, input.cycle_index,
                                    "红外量测与跟踪滤波交互接口测试", tracking_interface);
    }
    if (lost_due_to_estimation_nis) {
      detection.attribution.capture_failure_reason =
          attribution::SbirsCaptureFailureReason::kEstimationNisGateLost;
      log_acceptance_release(engaged_id, "nis_gate_lost", gate_failure_count);
      target_states_[engaged_id] = SbirsTargetState::kWideCandidate;
      nfov_scheduler_.Release(engaged_id);
      pointing_coordinator_.ReleaseTarget(engaged_id);
      tracking_coordinator_.ReleaseTarget(engaged_id);
    } else if (lost_due_to_tracking_gate) {
      detection.attribution.capture_failure_reason =
          attribution::SbirsCaptureFailureReason::kNfovTrackingGateLost;
      log_acceptance_release(engaged_id, "tracking_gate_lost", gate_failure_count);
      target_states_[engaged_id] = SbirsTargetState::kWideCandidate;
      nfov_scheduler_.Release(engaged_id);
      pointing_coordinator_.ReleaseTarget(engaged_id);
      tracking_coordinator_.ReleaseTarget(engaged_id);
    }
    // 焦平面脱靶量：目标传感器系角与实际指向角的逐轴差经 f·tan 映射（米+像素）；
    // 焦距/像元间距非正时（配置校验已拦截）跳过。随归属记录透出（验收判定标准
    // 第26项 脱靶量数据源，精度评估层消费），验收行在此复用同一份计算。
    foundation::SbirsFocalPlaneOffset focal_offset;
    const bool focal_valid = foundation::ComputeFocalPlaneOffset(
        hw.focal_length_m, hw.detector_pixel_pitch_m,
        AzimuthDelta(context.sensor_azimuth_deg, actual_pointing_azimuth_deg),
        context.sensor_elevation_deg - actual_pointing_elevation_deg, &focal_offset);
    if (focal_valid) {
      detection.attribution.has_focal_plane_offset = true;
      detection.attribution.focal_plane_offset_x_m = focal_offset.x_m;
      detection.attribution.focal_plane_offset_y_m = focal_offset.y_m;
    }
    if (SBIRS_DUAL_ACCEPTANCE_LOG_ENABLED()) {
      // 中译：窄视场跟踪事件（目标编号、跟踪模式、实际指向与目标角、指向误差、焦平面
      //       脱靶量、输出角定位角度误差、SNR 与信号能量、几何/SNR 门结果、失败计数、
      //       滑行标志、NIS、本周期帧数与单帧/融合 σ）。
      // 标识：验收日志 E5——3.2.1.3.2.3"焦平面脱靶量、跟踪状态、目标信号能量与 SNR"、
      //       3.2.1.6.3 红外定位角度误差（跟踪段）证据；输出角误差在滤波/误差注入
      //       全部完成后取最终输出角 − 真值角（方位最短角差）。滑行周期 帧数=0。
      const double az_err = AzimuthDelta(
          static_cast<float>(detection.record.azimuth_rad * 57.29577951308232), context.azimuth_deg);
      const double el_err =
          static_cast<float>(detection.record.elevation_rad * 57.29577951308232) - context.elevation_deg;
      std::string nfov = "卫星ID=" + std::to_string(satellite_entity_id_);
      nfov += " 目标ID=" + std::to_string(engaged_id);
      if (focal_valid) {
        nfov += " 脱靶量m=(" + oneq::logging::FormatF(focal_offset.x_m, 6) + "," +
                oneq::logging::FormatF(focal_offset.y_m, 6) + ")";
        nfov += " 脱靶量像素=(" + oneq::logging::FormatF(focal_offset.x_pixels, 2) + "," +
                oneq::logging::FormatF(focal_offset.y_pixels, 2) + ")";
      }
      nfov += " 窄视场目标视线方位/俯仰(ECI)=(" +
              oneq::logging::FormatF(detection.record.azimuth_rad, 8) + "," +
              oneq::logging::FormatF(detection.record.elevation_rad, 8) + ")rad";
      const double snr_db = 10.0 * std::log10(std::max(context.snr, 1.0e-12));
      nfov += " 跟踪状态=";
      nfov += detection.attribution.nfov_tracking_coasting ? "滑行" : "跟踪";
      nfov += " 信号能量=" + oneq::logging::FormatSci(context.signal_energy_j) + "J";
      nfov += " SNR(dB)=" + oneq::logging::FormatF(snr_db, 3) + "dB";
      // 高刷新率高精度数据输出（单镜筒轮转口径）：本周期实际帧数 + 单帧/融合 1-σ。
      nfov += " 帧数=" + std::to_string(nfov_frame_count);
      nfov += " 单帧σ=" + oneq::logging::FormatSci(nfov_frame_sigma_deg) + "deg";
      nfov += " 融合σ=" + oneq::logging::FormatSci(nfov_fused_sigma_deg) + "deg";
      SBIRS_DUAL_ACCEPTANCE_ITEM(sim_time_sec, input.cycle_index, "窄视场跟踪探测功能测试",
                                 nfov);
      // 规范口径（验收判定标准 第14项·其三）：窄视场精确状态序列——同一目标逐周期
      // 的窄场角（ECI rad）、跟踪状态与 SNR。
      std::string joint_track = "卫星ID=" + std::to_string(satellite_entity_id_);
      joint_track += " 目标ID=" + std::to_string(engaged_id);
      joint_track += " 窄场方位/俯仰(ECI)=(" +
                     oneq::logging::FormatF(detection.record.azimuth_rad, 8) + "," +
                     oneq::logging::FormatF(detection.record.elevation_rad, 8) + ")rad";
      joint_track += " 跟踪状态=";
      joint_track += detection.attribution.nfov_tracking_coasting ? "滑行" : "跟踪";
      joint_track += " SNR(dB)=" + oneq::logging::FormatF(snr_db, 3) + "dB";
      SBIRS_DUAL_ACCEPTANCE_ITEM(sim_time_sec, input.cycle_index,
                                 "宽窄视场联合探测功能测试", joint_track);
      WriteSbirsAngleError(satellite_entity_id_, sim_time_sec, input.cycle_index,
                           engaged_id, az_err, el_err);
      if (tracking_result.has_angle_rate) {
        // 中译：角度域 KF 后验（滤波方位/俯仰及其变化率）；不进公开检测记录。
        // 标识：实验后端 kAngleCvKf 的用例 16 状态估计验收行。
        WriteSbirsAngleStateEstimate(
            sim_time_sec, input.cycle_index, engaged_id,
            tracking_result.output_azimuth_deg, tracking_result.output_elevation_deg,
            tracking_result.azimuth_rate_rad_per_s * 57.29577951308232,
            tracking_result.elevation_rate_rad_per_s * 57.29577951308232);
      }
    }
    result.detections.push_back(detection);
  }

  // 被服务目标的首次捕获（轮转窗口语义：转向中宽场行 / 超时或被拒回退 / 稳定后捕获）。
  if (has_served_target &&
      target_states_[served_target_id] == SbirsTargetState::kAwaitingNfovAcquisition) {
    const auto candidate_it = candidate_by_target.find(served_target_id);
    if (candidate_it != candidate_by_target.end()) {
      const SbirsCandidate& selected = *candidate_it->second;
      processed_target_ids.insert(served_target_id);
      if (pointing_result.status == SbirsPointingAdvanceStatus::kSlewing) {
        if (SBIRS_DUAL_ACCEPTANCE_LOG_ENABLED()) {
          // 中译：窄视场首次捕获事件——转向中（卫星、目标、窄场指向命令角、SNR）。
          // 标识：验收日志 E4——3.2.1.3.2.1 NFOV 指向/捕获过程的"转向"中间态证据；
          //       窄场角按规范口径写 ECI 弧度（cue 预测命令角）。
          constexpr double kDegToRad = 0.017453292519943295;
          const double selected_snr_db = 10.0 * std::log10(std::max(selected.snr, 1.0e-12));
          SBIRS_DUAL_ACCEPTANCE_ITEM(
              sim_time_sec, input.cycle_index, "宽窄视场联合探测功能测试",
              "卫星ID=" + std::to_string(satellite_entity_id_) + " 目标ID=" +
                  std::to_string(served_target_id) + " 捕获=转向中 窄场方位/俯仰(ECI)=(" +
                  oneq::logging::FormatF(selected.command_azimuth_deg * kDegToRad, 8) + "," +
                  oneq::logging::FormatF(selected.command_elevation_deg * kDegToRad, 8) +
                  ")rad SNR(dB)=" + oneq::logging::FormatF(selected_snr_db, 3) + "dB");
        }
        append_wfov_detection(selected);
      } else if (pointing_result.status == SbirsPointingAdvanceStatus::kTimedOut) {
        log_acceptance_release(served_target_id, "pointing_timeout", 0U);
        nfov_scheduler_.Release(served_target_id);
        pointing_coordinator_.ReleaseTarget(served_target_id);
        target_states_[served_target_id] = SbirsTargetState::kWideCandidate;
        blocked_target_ids.insert(served_target_id);
        if (SBIRS_DUAL_ACCEPTANCE_LOG_ENABLED()) {
          // 中译：窄视场首次捕获事件——指向超时失败（卫星、目标）。
          // 标识：验收日志 E4——3.2.1.3.2.1 指向超时回退证据（与 E7 release 成对）。
          SBIRS_DUAL_ACCEPTANCE_ITEM(sim_time_sec, input.cycle_index, "宽窄视场联合探测功能测试",
                                "卫星ID=" + std::to_string(satellite_entity_id_) +
                                    " 目标ID=" + std::to_string(served_target_id) +
                                    " 捕获=超时失败");
        }
        append_acquisition_failure(
            selected, attribution::SbirsCaptureFailureReason::kNfovPointingTimeout);
      } else if (pointing_result.status == SbirsPointingAdvanceStatus::kRejected ||
                 !nfov_pointing_valid) {
        log_acceptance_release(served_target_id, "pointing_rejected", 0U);
        nfov_scheduler_.Release(served_target_id);
        pointing_coordinator_.ReleaseTarget(served_target_id);
        target_states_[served_target_id] = SbirsTargetState::kWideCandidate;
        blocked_target_ids.insert(served_target_id);
        if (SBIRS_DUAL_ACCEPTANCE_LOG_ENABLED()) {
          // 中译：窄视场首次捕获事件——指向被拒失败（卫星、目标）。
          // 标识：验收日志 E4——3.2.1.3.2.1 指向拒绝回退证据（与 E7 release 成对）。
          SBIRS_DUAL_ACCEPTANCE_ITEM(sim_time_sec, input.cycle_index, "宽窄视场联合探测功能测试",
                                "卫星ID=" + std::to_string(satellite_entity_id_) +
                                    " 目标ID=" + std::to_string(served_target_id) +
                                    " 捕获=指向被拒");
        }
        append_acquisition_failure(selected,
                                   attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed);
      } else {
        attempt_capture(selected);
      }
    }
  }

  // 同帧免转动捕获：非服务但已在当前窄视场内的 Awaiting 候选立即尝试捕获（镜筒不动，
  // 捕获命令 = 当前实际指向）。
  if (nfov_pointing_valid) {
    for (const std::uint64_t engaged_id : engaged_ids) {
      if (engaged_id == served_target_id ||
          processed_target_ids.count(engaged_id) != 0U ||
          blocked_target_ids.count(engaged_id) != 0U ||
          target_states_[engaged_id] != SbirsTargetState::kAwaitingNfovAcquisition) {
        continue;
      }
      const auto candidate_it = candidate_by_target.find(engaged_id);
      if (candidate_it == candidate_by_target.end()) {
        continue;
      }
      const SbirsCandidate& selected = *candidate_it->second;
      float candidate_sensor_azimuth_deg = 0.0f;
      float candidate_sensor_elevation_deg = 0.0f;
      boresight_chain.SensorAzElOfEciVector(
          LosFromAzimuthElevation(selected.azimuth_deg, selected.elevation_deg),
          &candidate_sensor_azimuth_deg, &candidate_sensor_elevation_deg);
      if (!InRectangularFov(candidate_sensor_azimuth_deg, candidate_sensor_elevation_deg,
                            actual_pointing_azimuth_deg, actual_pointing_elevation_deg,
                            mission.narrow_field_fov_az_deg, mission.narrow_field_fov_el_deg)) {
        continue;
      }
      processed_target_ids.insert(engaged_id);
      attempt_capture(selected);
    }
  }

  // 未处理目标（等待轮转窗口的 Awaiting 与命中门未达标的候选）保持宽场候选行；
  // 单镜筒无"资源满跳过"标记（容量由轮转物理涌现，不再有 kSchedulerSkipped）。
  for (const SbirsCandidate& candidate : candidates) {
    const std::uint64_t target_id = candidate.target->target_id;
    if (processed_target_ids.count(target_id) != 0U ||
        blocked_target_ids.count(target_id) != 0U) {
      continue;
    }
    SbirsPipelineDetection detection;
    detection.record.detection_id = next_detection_id_++;
    detection.record.measurement_time_s = candidate.measurement_time_s;
    detection.record.azimuth_rad = ToEciAzimuthRad(candidate.measured_azimuth_deg);
    detection.record.elevation_rad = ToEciElevationRad(candidate.measured_elevation_deg);
    detection.record.infrared_snr_linear = static_cast<float>(candidate.snr);
    detection.record.observation_stage = output::SbirsObservationStage::kWideFieldSearch;
    detection.record.detected = true;
    detection.attribution.detection_id = detection.record.detection_id;
    detection.attribution.target_id = candidate.target->target_id;
    detection.attribution.target_name = candidate.target->target_name;
    detection.attribution.estimated_range_m = static_cast<float>(candidate.range_m);
    detection.attribution.nfov_channel_id = nfov_scheduler_.IsLocked(target_id) ? 0 : -1;
    result.detections.push_back(detection);
  }

  register_detectability();
  log_cycle_summary();
  return result;
}

SbirsPipelineSnapshot SbirsPipeline::CaptureRuntimeState() const {
  SbirsPipelineSnapshot snapshot;
  snapshot.scan_phase_deg = scan_phase_deg_;
  snapshot.scan_leg_forward = scan_leg_forward_;
  snapshot.scan_azimuth_base_deg = scan_azimuth_base_deg_;
  snapshot.scan_row_index = scan_row_index_;
  snapshot.misalignment_yaw_deg = static_cast<float>(misalignment_total_deg_.yaw_deg);
  snapshot.misalignment_pitch_deg = static_cast<float>(misalignment_total_deg_.pitch_deg);
  snapshot.misalignment_roll_deg = static_cast<float>(misalignment_total_deg_.roll_deg);
  snapshot.next_detection_id = next_detection_id_;
  snapshot.target_states = target_states_;
  snapshot.wfov_consecutive_hits = wfov_consecutive_hits_;
  snapshot.nfov_scheduler = nfov_scheduler_.Capture();
  snapshot.nfov_rotation_step = nfov_rotation_step_;
  snapshot.pointing_coordinator = pointing_coordinator_.Capture();
  snapshot.wfov_measurement_random_state = wfov_measurement_random_source_.Capture();
  snapshot.estimated_measurement_random_state = estimated_measurement_random_source_.Capture();
  snapshot.sensor_like_output_random_state = sensor_like_output_random_source_.Capture();
  snapshot.cue_predictor = cue_predictor_.Capture();
  const SbirsTrackingRuntimeState tracking_state = tracking_coordinator_.CaptureRuntimeState();
  snapshot.filter_states = tracking_state.filter_states;
  snapshot.angle_kf_states = tracking_state.angle_kf_states;
  snapshot.nis_gate_exceeded_counts = tracking_state.nis_gate_exceeded_counts;
  snapshot.imm_active = tracking_state.imm_active;
  snapshot.imm_snapshots = tracking_state.imm_snapshots;
  return snapshot;
}

bool SbirsPipeline::RestoreRuntimeState(const SbirsPipelineSnapshot& snapshot) {
  if (!IsValidTrackingSnapshot(snapshot, config_.session.policy.tracking) ||
      snapshot.scan_phase_deg < 0.0f ||
      snapshot.scan_phase_deg >= config_.session.mission.scan_span_deg ||
      snapshot.scan_row_index < 0 ||
      snapshot.scan_row_index >= ScanRowCount(config_.session.mission)) {
    return false;
  }
  SbirsPointingCoordinator restored_pointing(
      config_.session.policy.pointing_disturbance.random_seed);
  if (!restored_pointing.Restore(snapshot.pointing_coordinator)) {
    return false;
  }
  // 锁定集合与目标状态机一致性：集合成员须处于 Awaiting/Tracking 态，反之亦然。
  for (const std::map<std::uint64_t, int>::value_type& assignment :
       snapshot.nfov_scheduler.target_to_cue_source) {
    const std::map<std::uint64_t, SbirsTargetState>::const_iterator state =
        snapshot.target_states.find(assignment.first);
    if (state == snapshot.target_states.end() ||
        (state->second != SbirsTargetState::kAwaitingNfovAcquisition &&
         state->second != SbirsTargetState::kEstimatedTracking &&
         state->second != SbirsTargetState::kStrictTruthAssistedTracking &&
         state->second != SbirsTargetState::kSensorLikeTruthAssistedTracking)) {
      return false;
    }
  }
  for (const std::map<std::uint64_t, SbirsTargetState>::value_type& state :
       snapshot.target_states) {
    if ((state.second == SbirsTargetState::kAwaitingNfovAcquisition ||
         state.second == SbirsTargetState::kEstimatedTracking ||
         state.second == SbirsTargetState::kStrictTruthAssistedTracking ||
         state.second == SbirsTargetState::kSensorLikeTruthAssistedTracking) &&
        snapshot.nfov_scheduler.target_to_cue_source.find(state.first) ==
            snapshot.nfov_scheduler.target_to_cue_source.end()) {
      return false;
    }
  }
  // 单镜筒簿记一致性：捕获等待只属于 Awaiting 目标；跟踪门失败计数只属于 Tracking
  // 目标且未达丢锁门限（达到即应已在该周期释放）。
  for (const auto& wait_entry : snapshot.pointing_coordinator.acquisition_wait_sec) {
    const std::map<std::uint64_t, SbirsTargetState>::const_iterator state =
        snapshot.target_states.find(wait_entry.first);
    if (state == snapshot.target_states.end() ||
        state->second != SbirsTargetState::kAwaitingNfovAcquisition) {
      return false;
    }
  }
  for (const auto& count_entry : snapshot.pointing_coordinator.tracking_gate_failure_counts) {
    const std::map<std::uint64_t, SbirsTargetState>::const_iterator state =
        snapshot.target_states.find(count_entry.first);
    if (state == snapshot.target_states.end() ||
        (state->second != SbirsTargetState::kEstimatedTracking &&
         state->second != SbirsTargetState::kStrictTruthAssistedTracking &&
         state->second != SbirsTargetState::kSensorLikeTruthAssistedTracking) ||
        count_entry.second >= config_.session.policy.tracking.nfov_tracking_gate_loss_cycles) {
      return false;
    }
  }
  SbirsTrackingRuntimeState tracking_state;
  tracking_state.filter_states = snapshot.filter_states;
  tracking_state.angle_kf_states = snapshot.angle_kf_states;
  tracking_state.nis_gate_exceeded_counts = snapshot.nis_gate_exceeded_counts;
  tracking_state.imm_active = snapshot.imm_active;
  tracking_state.imm_snapshots = snapshot.imm_snapshots;
  scan_phase_deg_ = snapshot.scan_phase_deg;
  scan_leg_forward_ = snapshot.scan_leg_forward;
  scan_azimuth_base_deg_ = snapshot.scan_azimuth_base_deg;
  scan_row_index_ = snapshot.scan_row_index;
  misalignment_total_deg_.yaw_deg = snapshot.misalignment_yaw_deg;
  misalignment_total_deg_.pitch_deg = snapshot.misalignment_pitch_deg;
  misalignment_total_deg_.roll_deg = snapshot.misalignment_roll_deg;
  next_detection_id_ = snapshot.next_detection_id;
  target_states_ = snapshot.target_states;
  wfov_consecutive_hits_ = snapshot.wfov_consecutive_hits;
  nfov_rotation_step_ = snapshot.nfov_rotation_step;
  nfov_scheduler_.Restore(snapshot.nfov_scheduler);
  pointing_coordinator_ = restored_pointing;
  wfov_measurement_random_source_.Restore(snapshot.wfov_measurement_random_state);
  estimated_measurement_random_source_.Restore(snapshot.estimated_measurement_random_state);
  sensor_like_output_random_source_.Restore(snapshot.sensor_like_output_random_state);
  cue_predictor_.Restore(snapshot.cue_predictor);
  tracking_coordinator_.RestoreRuntimeState(tracking_state);
  return true;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
