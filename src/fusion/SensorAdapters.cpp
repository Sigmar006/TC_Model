/**
 * @file SensorAdapters.cpp
 * @brief 传感器输出 → 融合探测记录的官方适配器实现。
 *
 * 语义逐行镜像原 examples/common/sensor_adapt.h（示例层共享边界适配）：
 * 行为不变，仅命名空间从 examples::sensor_adapt 上移为 fusion，函数名加
 * 模块前缀 + ToDetectionRecords 后缀。质量归一化基准为库默认（见头文件）。
 */

#include "1q/fusion/SensorAdapters.h"

#include <cmath>

#include "1q/coordinate/position_transform.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "common/numerics/ClampUtils.h"
#include "common/numerics/Constants.h"

namespace fusion {

namespace {

/// 探测质量基准：无识别置信度（target_probability == 0）时按轨迹状态取基准值。
double ArBaseQualityForStatus(airborne_radar::session::TrackStatus status) {
  return status == airborne_radar::session::TrackStatus::kConfirmed ? 1.0 : 0.5;
}

/// east 起量方位 → north 起量方位（90° − az，wrap 到 [-180, 180]；
/// 非有限输入由 NormalizeAngle180 归 0）。
double EastToNorthAzimuthDeg(float look_az_deg) {
  return static_cast<double>(
      oneq::common::numerics::NormalizeAngle180(90.0f - look_az_deg));
}

/// 11 维固定布局（冻结契约 §3.2）；无效维显式填 0（NaN 禁止——毒化欧氏门），
/// valid_feature_mask 为权威有效性。
std::vector<double> BuildRirFeatureVector(
    const remote_identification_radar::session::RirFeatureMeasurementRecord& measurement) {
  namespace rir = remote_identification_radar::session;
  const std::uint8_t mask = measurement.valid_feature_mask;
  std::vector<double> feature(11U, 0.0);
  if ((mask & static_cast<std::uint8_t>(rir::RirRecognitionFeatureDimension::kRcs)) != 0U) {
    feature[0] = measurement.features.rcs.mean_dbsm;
  }
  if ((mask & static_cast<std::uint8_t>(rir::RirRecognitionFeatureDimension::kMotion)) != 0U) {
    feature[1] = measurement.features.motion.speed_m_per_s / 1.0e3;
    feature[2] = measurement.features.motion.altitude_m / 1.0e3;
    feature[3] = measurement.features.motion.acceleration_m_per_s2;
    // log10 仅对正有限半径定义；非正/非有限半径回退 1.0（log10=0，与无效维 0 填
    // 同值，防 ±inf 毒化欧氏门）。
    feature[4] = std::log10(oneq::common::numerics::SafePositive(
        static_cast<double>(measurement.features.motion.turn_radius_m), 1.0));
  }
  if ((mask & static_cast<std::uint8_t>(rir::RirRecognitionFeatureDimension::kPolarization)) !=
      0U) {
    feature[5] = measurement.features.polarization.energy_difference_db;
    feature[6] = measurement.features.polarization.relative_difference_db;
    feature[7] = measurement.features.polarization.energy_sum_db;
  }
  if ((mask & static_cast<std::uint8_t>(rir::RirRecognitionFeatureDimension::kRangeProfile)) !=
      0U) {
    feature[8] = measurement.features.range_profile.length_m;
    feature[9] = static_cast<double>(measurement.features.range_profile.peak_count);
    feature[10] = measurement.features.range_profile.peak_energy_concentration;
  }
  return feature;
}

/// 有效维质量等权均值（feature_weights 配置口径、缺省等权——适配器无配置参数）。
double MeanValidDimensionQuality(
    const remote_identification_radar::session::RirFeatureMeasurementRecord& measurement) {
  namespace rir = remote_identification_radar::session;
  const std::uint8_t mask = measurement.valid_feature_mask;
  double quality_sum = 0.0;
  int valid_count = 0;
  if ((mask & static_cast<std::uint8_t>(rir::RirRecognitionFeatureDimension::kRcs)) != 0U) {
    quality_sum += measurement.features.rcs.quality;
    ++valid_count;
  }
  if ((mask & static_cast<std::uint8_t>(rir::RirRecognitionFeatureDimension::kMotion)) != 0U) {
    quality_sum += measurement.features.motion.quality;
    ++valid_count;
  }
  if ((mask & static_cast<std::uint8_t>(rir::RirRecognitionFeatureDimension::kPolarization)) !=
      0U) {
    quality_sum += measurement.features.polarization.quality;
    ++valid_count;
  }
  if ((mask & static_cast<std::uint8_t>(rir::RirRecognitionFeatureDimension::kRangeProfile)) !=
      0U) {
    quality_sum += measurement.features.range_profile.quality;
    ++valid_count;
  }
  return valid_count > 0 ? quality_sum / static_cast<double>(valid_count) : 0.0;
}

/// 斜距 + 雷达局部 ENU 视线角 → 东-北-天（ComputeLookAngles 的逆：
/// az 自东 atan2(north, east)，el 出地平 atan2(up, hypot)）。
bool TryLookRangeToEnu(float look_az_deg, float look_el_deg, float range_m,
                       oneq::coordinate::EnuPositionM* enu) {
  const double range = static_cast<double>(range_m);
  if (enu == nullptr || !std::isfinite(range) || range <= 0.0) {
    return false;
  }
  const double az_rad =
      oneq::common::numerics::DegToRad(static_cast<double>(look_az_deg));
  const double el_rad =
      oneq::common::numerics::DegToRad(static_cast<double>(look_el_deg));
  if (!std::isfinite(az_rad) || !std::isfinite(el_rad)) {
    return false;
  }
  const double cos_el = std::cos(el_rad);
  enu->east_m = range * cos_el * std::cos(az_rad);
  enu->north_m = range * cos_el * std::sin(az_rad);
  enu->up_m = range * std::sin(el_rad);
  return std::isfinite(enu->east_m) && std::isfinite(enu->north_m) &&
         std::isfinite(enu->up_m);
}

}  // namespace

std::vector<DetectionRecord> AdaptArTracksToDetectionRecords(
    std::uint32_t source_id,
    const airborne_radar::session::ArExternalTrackOutputFrame& frame) {
  std::vector<DetectionRecord> detections;
  detections.reserve(frame.tracks.size());
  for (const auto& track : frame.tracks) {
    if (track.status == airborne_radar::session::TrackStatus::kLost) {
      continue;  // 失跟轨迹不入融合（避免以旧位置续命航迹）
    }
    DetectionRecord record;
    record.key = track.association_key;
    record.source_id = source_id;
    record.has_position = true;
    oneq::coordinate::LlaPositionDegM lla;
    if (oneq::coordinate::TryEcefToLla(track.target_position_ecef_m, &lla)) {
      record.position = lla;
    } else {
      record.has_position = false;  // 转换失败则退化为仅身份键记录
    }
    record.verdict = 1.0;  // 已发布的轨迹快照视为有效探测
    record.quality = track.target_probability > 0.0f
                         ? static_cast<double>(track.target_probability)
                         : ArBaseQualityForStatus(track.status);
    detections.push_back(record);
  }
  return detections;
}

std::vector<DetectionRecord> AdaptEsrHypothesesToDetectionRecords(
    std::uint32_t source_id,
    const electronic_surveillance_radar::session::EmitterHypothesisList& hypotheses) {
  std::vector<DetectionRecord> detections;
  detections.reserve(hypotheses.size());
  for (const auto& hypothesis : hypotheses) {
    if (hypothesis.hypothesis_id == 0U) {
      continue;  // 库内键 0 = 无身份，不适用身份直挂
    }
    DetectionRecord record;
    record.key = hypothesis.hypothesis_id;
    record.source_id = source_id;
    record.has_bearing = true;
    record.bearing_az_deg = hypothesis.bearing_az_deg;
    record.bearing_el_deg = hypothesis.bearing_el_deg;
    // 射频特征归一化到可比尺度（GHz/MHz/ms/µs），供特征门限未来启用。
    record.feature = {hypothesis.estimated_center_frequency_hz / 1.0e9,
                      hypothesis.estimated_bandwidth_hz / 1.0e6,
                      hypothesis.estimated_pri_s * 1.0e3,
                      hypothesis.estimated_pulse_width_s * 1.0e6};
    record.verdict = 1.0;  // 已发布的辐射源假设视为有效探测
    record.quality = hypothesis.confidence;
    detections.push_back(record);
  }
  return detections;
}

std::vector<DetectionRecord> AdaptEosDetectionsToDetectionRecords(
    std::uint32_t source_id,
    const electro_optical_sensor::output::EosDetectionRecordList& records) {
  std::vector<DetectionRecord> detections;
  detections.reserve(records.size());
  for (const auto& record : records) {
    if (!record.detected) {
      continue;  // 未过探测门限不产生探测
    }
    DetectionRecord detection;
    detection.key = 0U;  // 无外部身份通道：走方位相干关联（去真值化纪律）
    detection.source_id = source_id;
    detection.has_bearing = true;
    detection.bearing_az_deg = record.azimuth_deg;
    detection.bearing_el_deg = record.elevation_deg;
    // range 通道首期不使用（保留方位相干关联路径）；
    // 质量 = 融合 SNR 归一化（10 dB → 1.0，库默认基准）。
    detection.verdict = 1.0;
    detection.quality =
        oneq::common::numerics::Clamp01(static_cast<double>(record.fused_snr_db) / 10.0);
    detections.push_back(detection);
  }
  return detections;
}

std::vector<DetectionRecord> AdaptSbirsDetectionsToDetectionRecords(
    std::uint32_t source_id,
    const sbirs_sensor::output::SbirsDetectionRecordList& records) {
  std::vector<DetectionRecord> detections;
  detections.reserve(records.size());
  for (const auto& record : records) {
    if (!record.detected) {
      continue;  // 未过探测门限不产生探测
    }
    DetectionRecord detection;
    detection.key = 0U;  // 无外部身份通道：走方位相干关联（去真值化纪律）
    detection.source_id = source_id;
    detection.has_bearing = true;
    // SBIRS 输出为 ECI 极坐标弧度（2026-08 正式变更），融合方位通道为 deg（北偏东
    // 平台系语义由调用方对齐）；此处仅做 rad→deg 单位换算，不改变参考系语义。
    detection.bearing_az_deg = oneq::common::numerics::RadToDeg(record.azimuth_rad);
    detection.bearing_el_deg = oneq::common::numerics::RadToDeg(record.elevation_rad);
    // 质量 = 线性 IR SNR 相对 WFOV 检测门限（4.0 → 1.0）的归一化（库默认基准）。
    detection.verdict = 1.0;
    detection.quality = oneq::common::numerics::Clamp01(
        static_cast<double>(record.infrared_snr_linear) / 4.0);
    detection.has_timestamp = true;
    detection.timestamp_s = record.measurement_time_s;
    detections.push_back(detection);
  }
  return detections;
}

std::vector<DetectionRecord> AdaptRirFeatureMeasurementsToDetectionRecords(
    std::uint32_t source_id,
    const remote_identification_radar::session::RirFeatureMeasurementFrame& frame) {
  std::vector<DetectionRecord> detections;
  detections.reserve(frame.records.size());
  for (const auto& measurement : frame.records) {
    if (measurement.association_key == 0U || measurement.valid_feature_mask == 0U) {
      continue;  // 库内键 0 = 无身份；全维无效记录不产生（与出口①同口径）
    }
    DetectionRecord detection;
    detection.key = measurement.association_key;
    detection.source_id = source_id;
    detection.has_bearing = true;
    // RIR 出口① az 自 +x（东）起量，融合方位通道自北：east→north 参考换算。
    detection.bearing_az_deg = EastToNorthAzimuthDeg(measurement.look_az_deg);
    detection.bearing_el_deg = measurement.look_el_deg;
    detection.feature = BuildRirFeatureVector(measurement);
    detection.verdict = 1.0;  // 已发布的特征量测视为有效探测
    detection.quality = MeanValidDimensionQuality(measurement);
    if (measurement.has_platform_position) {
      const oneq::coordinate::EcefPositionM origin(
          measurement.platform_position.x_m, measurement.platform_position.y_m,
          measurement.platform_position.z_m);
      oneq::coordinate::LlaPositionDegM origin_lla;
      if (oneq::coordinate::TryEcefToLla(origin, &origin_lla)) {
        detection.has_sensor_origin = true;
        detection.sensor_origin = origin_lla;  // 参与三维方位滤波通道
        oneq::coordinate::EnuPositionM enu;
        oneq::coordinate::EcefPositionM target_ecef;
        oneq::coordinate::LlaPositionDegM target_lla;
        if (TryLookRangeToEnu(measurement.look_az_deg, measurement.look_el_deg,
                              measurement.range_m, &enu) &&
            oneq::coordinate::TryEnuToEcef(enu, origin_lla, &target_ecef) &&
            oneq::coordinate::TryEcefToLla(target_ecef, &target_lla)) {
          detection.has_position = true;
          detection.position = target_lla;
        }  // 斜距非法或换算失败：维持仅方位+原点
      }  // 转换失败退化为无原点记录（AR 先例）
    }
    detections.push_back(detection);
  }
  return detections;
}

}  // namespace fusion
