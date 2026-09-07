#include "1q/precision_evaluation/SbirsBearingAdapter.h"

#include <algorithm>
#include <cmath>
#include <map>

#include "1q/coordinate/inertial_transform.h"
#include "1q/coordinate/position_transform.h"

namespace precision_evaluation {

oneq::coordinate::Vector3d EciDirectionFromAzimuthElevationRad(double azimuth_rad,
                                                               double elevation_rad) {
  const double horizontal = std::cos(elevation_rad);
  return oneq::coordinate::Vector3d(horizontal * std::cos(azimuth_rad),
                                    horizontal * std::sin(azimuth_rad),
                                    std::sin(elevation_rad));
}

std::vector<fusion::DetectionRecord> AdaptSbirsResultToDetectionRecords(
    const sbirs_sensor::session::SbirsCycleResult& result,
    const oneq::coordinate::EcefPositionM& satellite_position_ecef, double gmst_rad,
    std::uint32_t source_id) {
  std::vector<fusion::DetectionRecord> records;
  if (result.status != sbirs_sensor::session::SbirsCycleStatus::kCompleted) {
    return records;
  }
  // 归属层按 detection_id 索引，恢复每条检测的目标身份（评估层消费归属是分层契约
  // 允许的真值/归属使用；raw output 本身仍无身份）。
  std::map<std::uint64_t, const sbirs_sensor::attribution::SbirsDetectionAttributionRecord*>
      attributions;
  for (const sbirs_sensor::attribution::SbirsDetectionAttributionRecord& attribution :
       result.detection_attributions) {
    attributions[attribution.detection_id] = &attribution;
  }
  // 卫星 LLA（量测原点）；ECEF→ENU 仿射变换的旋转部分用"两点差"提取（R·d =
  // ENU(sat+d) − ENU(sat)），对方向向量与位置量纲无关。
  oneq::coordinate::LlaPositionDegM satellite_lla;
  if (!oneq::coordinate::TryEcefToLla(satellite_position_ecef, &satellite_lla)) {
    return records;
  }
  oneq::coordinate::EnuPositionM satellite_enu;
  if (!oneq::coordinate::TryEcefToEnu(satellite_position_ecef, satellite_lla, &satellite_enu)) {
    return records;
  }
  records.reserve(result.output_frame.detections.size());
  for (const sbirs_sensor::output::SbirsDetectionRecord& detection :
       result.output_frame.detections) {
    if (!detection.detected) {
      continue;
    }
    const auto attribution_entry = attributions.find(detection.detection_id);
    if (attribution_entry == attributions.end()) {
      continue;
    }
    // ECI 输出角 → ECI 单位向量 → 旋回 ECEF（旋转线性，单位向量旋转后仍单位）。
    const oneq::coordinate::Vector3d direction_eci = EciDirectionFromAzimuthElevationRad(
        static_cast<double>(detection.azimuth_rad), static_cast<double>(detection.elevation_rad));
    const oneq::coordinate::EciPositionM direction_eci_position(direction_eci.x, direction_eci.y,
                                                                direction_eci.z);
    oneq::coordinate::EcefPositionM direction_ecef;
    if (!oneq::coordinate::TryEciToEcef(direction_eci_position, gmst_rad, &direction_ecef)) {
      continue;
    }
    oneq::coordinate::EcefPositionM offset_position(
        satellite_position_ecef.x_m + direction_ecef.x_m,
        satellite_position_ecef.y_m + direction_ecef.y_m,
        satellite_position_ecef.z_m + direction_ecef.z_m);
    oneq::coordinate::EnuPositionM offset_enu;
    if (!oneq::coordinate::TryEcefToEnu(offset_position, satellite_lla, &offset_enu)) {
      continue;
    }
    const double east = offset_enu.east_m - satellite_enu.east_m;
    const double north = offset_enu.north_m - satellite_enu.north_m;
    const double up = offset_enu.up_m - satellite_enu.up_m;
    const double horizontal_norm = std::sqrt(east * east + north * north + up * up);
    if (horizontal_norm <= 0.0) {
      continue;
    }
    fusion::DetectionRecord record;
    record.key = attribution_entry->second->target_id;
    record.source_id = source_id;
    record.verdict = 1.0;  // 已发布的检测视为有效探测（与官方适配器口径一致）
    record.has_bearing = true;
    record.bearing_az_deg =
        std::atan2(east / horizontal_norm, north / horizontal_norm) * 180.0 / 3.14159265358979323846;
    record.bearing_el_deg = std::asin(std::max(-1.0, std::min(1.0, up / horizontal_norm))) *
                            180.0 / 3.14159265358979323846;
    record.has_sensor_origin = true;
    record.sensor_origin = satellite_lla;
    record.has_timestamp = true;
    record.timestamp_s = detection.measurement_time_s;
    // 质量：IR SNR/1000 截断 [0,1]（线性 SNR 典型 4~1e3+，与官方适配器 SNR/4 相比
    // 压低强信号饱和；评估侧只影响融合置信度权重，不影响量测本身）。
    const double snr = static_cast<double>(detection.infrared_snr_linear);
    record.quality = std::max(0.0, std::min(1.0, snr / 1000.0));
    records.push_back(record);
  }
  return records;
}

}  // namespace precision_evaluation
