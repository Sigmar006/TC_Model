/**
 * @file fusion_detection_bridge.h
 * @brief 示例层 fusion::DetectionRecord ↔ FusionDetectionSample 互转。
 */

#ifndef EXAMPLES_CORE_FUSION_DETECTION_BRIDGE_H_
#define EXAMPLES_CORE_FUSION_DETECTION_BRIDGE_H_

#include "1q/fusion/DetectionRecord.h"
#include "core/events.h"

namespace component_attachment {

inline FusionDetectionSample ToFusionDetectionSample(const fusion::DetectionRecord& record) {
  FusionDetectionSample sample;
  sample.key = record.key;
  sample.source_id = record.source_id;
  sample.source_name = record.source_name;
  sample.has_position = record.has_position;
  if (record.has_position) {
    sample.latitude_deg = record.position.latitude_deg;
    sample.longitude_deg = record.position.longitude_deg;
    sample.altitude_m = record.position.altitude_m;
  }
  sample.has_bearing = record.has_bearing;
  sample.bearing_az_deg = record.bearing_az_deg;
  sample.bearing_el_deg = record.bearing_el_deg;
  sample.has_sensor_origin = record.has_sensor_origin;
  if (record.has_sensor_origin) {
    sample.origin_latitude_deg = record.sensor_origin.latitude_deg;
    sample.origin_longitude_deg = record.sensor_origin.longitude_deg;
    sample.origin_altitude_m = record.sensor_origin.altitude_m;
  }
  sample.has_bearing_noise = record.has_bearing_noise;
  sample.bearing_noise_sigma_rad = record.bearing_noise_sigma_rad;
  sample.verdict = record.verdict;
  sample.quality = record.quality;
  return sample;
}

inline fusion::DetectionRecord ToDetectionRecord(const FusionDetectionSample& sample) {
  fusion::DetectionRecord record;
  record.key = sample.key;
  record.source_id = sample.source_id;
  record.source_name = sample.source_name;
  record.has_position = sample.has_position;
  if (sample.has_position) {
    record.position = oneq::coordinate::LlaPositionDegM(
        sample.latitude_deg, sample.longitude_deg, sample.altitude_m);
  }
  record.has_bearing = sample.has_bearing;
  record.bearing_az_deg = sample.bearing_az_deg;
  record.bearing_el_deg = sample.bearing_el_deg;
  record.has_sensor_origin = sample.has_sensor_origin;
  if (sample.has_sensor_origin) {
    record.sensor_origin = oneq::coordinate::LlaPositionDegM(
        sample.origin_latitude_deg, sample.origin_longitude_deg, sample.origin_altitude_m);
  }
  record.has_bearing_noise = sample.has_bearing_noise;
  record.bearing_noise_sigma_rad = sample.bearing_noise_sigma_rad;
  record.verdict = sample.verdict;
  record.quality = sample.quality;
  return record;
}

}  // namespace component_attachment

#endif  // EXAMPLES_CORE_FUSION_DETECTION_BRIDGE_H_
