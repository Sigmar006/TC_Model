// Copyright 2026. All Rights Reserved.
//
// @file sensor_adapters_test.cpp
// @brief 传感器输出 → 融合探测记录官方适配器（SensorAdapters）单元测试。
//
// 覆盖五传感器适配函数：正常映射全字段、跳过规则（AR kLost / ESR
// hypothesis_id==0 / EOS-SBIRS detected==false / RIR 全维无效或键 0）、
// 质量归一化夹取、AR ECEF 转换失败退化分支、RIR 11 维布局与 east→north
// 换算、RIR 斜距+视线角+原点还原位置、source_id 透传、空输入 → 空输出。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "1q/airborne_radar/session/ArCycleOutputAdapter.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "1q/coordinate/position_transform.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"
#include "1q/fusion/FusionEngine.h"
#include "1q/fusion/SensorAdapters.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"

namespace fusion {
namespace {

// ============================ AR 轨迹帧构造 ============================

airborne_radar::session::ArExternalTrackKinematics MakeArTrack(
    std::uint64_t key, airborne_radar::session::TrackStatus status,
    const oneq::coordinate::EcefPositionM& ecef, float probability) {
  airborne_radar::session::ArExternalTrackKinematics track;
  track.association_key = key;
  track.status = status;
  track.target_position_ecef_m = ecef;
  track.target_probability = probability;
  return track;
}

/// 由已知 LLA 反向生成 ECEF（测试数据用真实坐标换算；换算失败即测试前提不成立）。
oneq::coordinate::EcefPositionM MakeEcefFromLla(double lat_deg, double lon_deg,
                                                double alt_m) {
  oneq::coordinate::LlaPositionDegM lla;
  lla.latitude_deg = lat_deg;
  lla.longitude_deg = lon_deg;
  lla.altitude_m = alt_m;
  oneq::coordinate::EcefPositionM ecef;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(lla, &ecef));
  return ecef;
}

/// 测试基准位置（机场 30°N, 120°E，400 m 高）：多个 AR 用例共用。
const oneq::coordinate::EcefPositionM kTestEcef = MakeEcefFromLla(30.0, 120.0, 400.0);

// ============================ ESR 假设构造 ============================

electronic_surveillance_radar::session::EmitterHypothesis MakeHypothesis(
    std::uint64_t id, double freq_hz, double bandwidth_hz, double pri_s,
    double pulse_width_s, float confidence) {
  electronic_surveillance_radar::session::EmitterHypothesis hypothesis;
  hypothesis.hypothesis_id = id;
  hypothesis.bearing_az_deg = 45.0f;
  hypothesis.bearing_el_deg = 10.0f;
  hypothesis.estimated_center_frequency_hz = freq_hz;
  hypothesis.estimated_bandwidth_hz = bandwidth_hz;
  hypothesis.estimated_pri_s = pri_s;
  hypothesis.estimated_pulse_width_s = pulse_width_s;
  hypothesis.confidence = confidence;
  return hypothesis;
}

// ============================ EOS/SBIRS 探测构造 ============================

electro_optical_sensor::output::EosDetectionRecord MakeEosRecord(bool detected,
                                                                 float snr_db) {
  electro_optical_sensor::output::EosDetectionRecord record;
  record.azimuth_deg = 90.0f;
  record.elevation_deg = 5.0f;
  record.fused_snr_db = snr_db;
  record.detected = detected;
  return record;
}

sbirs_sensor::output::SbirsDetectionRecord MakeSbirsRecord(
    bool detected, float ir_snr_linear, double measurement_time_s = 12.345) {
  sbirs_sensor::output::SbirsDetectionRecord record;
  // SBIRS 输出为 ECI 弧度（2026-08）：120° ≈ 2.0944 rad、−30° ≈ −0.5236 rad。
  record.azimuth_rad = 2.0943951023931953f;
  record.elevation_rad = -0.5235987755982988f;
  record.infrared_snr_linear = ir_snr_linear;
  record.detected = detected;
  record.measurement_time_s = measurement_time_s;
  return record;
}

// ============================ AR 适配 ============================

TEST(SensorAdaptersTest, ArTracksAdaptToDetectionRecords) {
  const oneq::coordinate::EcefPositionM& ecef = kTestEcef;
  airborne_radar::session::ArExternalTrackOutputFrame frame;
  frame.tracks = {MakeArTrack(1001U, airborne_radar::session::TrackStatus::kConfirmed,
                              ecef, 0.8f)};

  const std::vector<DetectionRecord> detections =
      AdaptArTracksToDetectionRecords(kArSourceId, frame);

  ASSERT_EQ(detections.size(), 1U);
  const DetectionRecord& record = detections[0];
  EXPECT_EQ(record.key, 1001U);
  EXPECT_EQ(record.source_id, kArSourceId);
  EXPECT_TRUE(record.has_position);
  EXPECT_NEAR(record.position.latitude_deg, 30.0, 1e-6);
  EXPECT_NEAR(record.position.longitude_deg, 120.0, 1e-6);
  EXPECT_NEAR(record.position.altitude_m, 400.0, 1.0);
  EXPECT_DOUBLE_EQ(record.verdict, 1.0);
  EXPECT_NEAR(record.quality, 0.8, 1e-6);  // target_probability 透传（float 源）
}

TEST(SensorAdaptersTest, ArLostTrackIsSkipped) {
  const oneq::coordinate::EcefPositionM& ecef = kTestEcef;
  airborne_radar::session::ArExternalTrackOutputFrame frame;
  frame.tracks = {
      MakeArTrack(1001U, airborne_radar::session::TrackStatus::kConfirmed, ecef, 0.8f),
      MakeArTrack(1002U, airborne_radar::session::TrackStatus::kLost, ecef, 0.0f)};

  const std::vector<DetectionRecord> detections =
      AdaptArTracksToDetectionRecords(kArSourceId, frame);

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections[0].key, 1001U);
}

TEST(SensorAdaptersTest, ArZeroProbabilityUsesStatusBaseQuality) {
  const oneq::coordinate::EcefPositionM& ecef = kTestEcef;
  airborne_radar::session::ArExternalTrackOutputFrame frame;
  frame.tracks = {
      MakeArTrack(1001U, airborne_radar::session::TrackStatus::kConfirmed, ecef, 0.0f),
      MakeArTrack(1002U, airborne_radar::session::TrackStatus::kTentative, ecef, 0.0f)};

  const std::vector<DetectionRecord> detections =
      AdaptArTracksToDetectionRecords(kArSourceId, frame);

  ASSERT_EQ(detections.size(), 2U);
  EXPECT_DOUBLE_EQ(detections[0].quality, 1.0);  // kConfirmed → 1.0
  EXPECT_DOUBLE_EQ(detections[1].quality, 0.5);  // kTentative → 0.5
}

TEST(SensorAdaptersTest, ArEcefConversionFailureDegradesToIdentityKeyOnly) {
  // 全零 ECEF：TryEcefToLla 对 norm ≤ floor 返回 false → has_position=false。
  const oneq::coordinate::EcefPositionM zero_ecef{};
  airborne_radar::session::ArExternalTrackOutputFrame frame;
  frame.tracks = {MakeArTrack(1001U, airborne_radar::session::TrackStatus::kConfirmed,
                              zero_ecef, 0.8f)};

  const std::vector<DetectionRecord> detections =
      AdaptArTracksToDetectionRecords(kArSourceId, frame);

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections[0].key, 1001U);
  EXPECT_FALSE(detections[0].has_position);
  EXPECT_NEAR(detections[0].quality, 0.8, 1e-6);  // 退化不影响质量（float 源）
}

// ============================ ESR 适配 ============================

TEST(SensorAdaptersTest, EsrHypothesesAdaptToDetectionRecords) {
  electronic_surveillance_radar::session::EmitterHypothesisList hypotheses;
  // 9.5 GHz / 2 MHz / 1 ms PRI / 1 µs 脉宽。
  hypotheses.push_back(MakeHypothesis(7U, 9.5e9, 2.0e6, 1.0e-3, 1.0e-6, 0.9f));

  const std::vector<DetectionRecord> detections =
      AdaptEsrHypothesesToDetectionRecords(kEsrSourceId, hypotheses);

  ASSERT_EQ(detections.size(), 1U);
  const DetectionRecord& record = detections[0];
  EXPECT_EQ(record.key, 7U);
  EXPECT_EQ(record.source_id, kEsrSourceId);
  EXPECT_TRUE(record.has_bearing);
  EXPECT_DOUBLE_EQ(record.bearing_az_deg, 45.0);
  EXPECT_DOUBLE_EQ(record.bearing_el_deg, 10.0);
  // 特征归一化标尺：GHz / MHz / ms / µs。
  ASSERT_EQ(record.feature.size(), 4U);
  EXPECT_DOUBLE_EQ(record.feature[0], 9.5);
  EXPECT_DOUBLE_EQ(record.feature[1], 2.0);
  EXPECT_DOUBLE_EQ(record.feature[2], 1.0);
  EXPECT_DOUBLE_EQ(record.feature[3], 1.0);
  EXPECT_DOUBLE_EQ(record.verdict, 1.0);
  EXPECT_NEAR(record.quality, 0.9, 1e-6);  // confidence 透传（float 源）
}

TEST(SensorAdaptersTest, EsrZeroHypothesisIdIsSkipped) {
  electronic_surveillance_radar::session::EmitterHypothesisList hypotheses;
  hypotheses.push_back(MakeHypothesis(0U, 9.5e9, 2.0e6, 1.0e-3, 1.0e-6, 0.9f));
  hypotheses.push_back(MakeHypothesis(8U, 10.0e9, 1.0e6, 5.0e-4, 2.0e-6, 0.7f));

  const std::vector<DetectionRecord> detections =
      AdaptEsrHypothesesToDetectionRecords(kEsrSourceId, hypotheses);

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections[0].key, 8U);
}

// ============================ EOS 适配 ============================

TEST(SensorAdaptersTest, EosDetectionsAdaptToDetectionRecords) {
  electro_optical_sensor::output::EosDetectionRecordList records;
  records.push_back(MakeEosRecord(/*detected=*/true, /*snr_db=*/20.0f));

  const std::vector<DetectionRecord> detections =
      AdaptEosDetectionsToDetectionRecords(kEosSourceId, records);

  ASSERT_EQ(detections.size(), 1U);
  const DetectionRecord& record = detections[0];
  EXPECT_EQ(record.key, 0U);  // 无身份 → 走方位相干关联
  EXPECT_EQ(record.source_id, kEosSourceId);
  EXPECT_TRUE(record.has_bearing);
  EXPECT_DOUBLE_EQ(record.bearing_az_deg, 90.0);
  EXPECT_DOUBLE_EQ(record.bearing_el_deg, 5.0);
  EXPECT_DOUBLE_EQ(record.verdict, 1.0);
  EXPECT_DOUBLE_EQ(record.quality, 1.0);  // 20 dB / 10 → 2.0 夹取到 1.0
}

TEST(SensorAdaptersTest, EosQualityClampedBySnrNormalization) {
  electro_optical_sensor::output::EosDetectionRecordList records;
  records.push_back(MakeEosRecord(/*detected=*/true, /*snr_db=*/5.0f));
  records.push_back(MakeEosRecord(/*detected=*/true, /*snr_db=*/-5.0f));

  const std::vector<DetectionRecord> detections =
      AdaptEosDetectionsToDetectionRecords(kEosSourceId, records);

  ASSERT_EQ(detections.size(), 2U);
  EXPECT_DOUBLE_EQ(detections[0].quality, 0.5);  // 5 dB / 10
  EXPECT_DOUBLE_EQ(detections[1].quality, 0.0);  // -5 dB / 10 夹取到 0
}

TEST(SensorAdaptersTest, EosNotDetectedIsSkipped) {
  electro_optical_sensor::output::EosDetectionRecordList records;
  records.push_back(MakeEosRecord(/*detected=*/false, /*snr_db=*/10.0f));

  const std::vector<DetectionRecord> detections =
      AdaptEosDetectionsToDetectionRecords(kEosSourceId, records);

  EXPECT_TRUE(detections.empty());
}

// ============================ SBIRS 适配 ============================

TEST(SensorAdaptersTest, SbirsDetectionsAdaptToDetectionRecords) {
  sbirs_sensor::output::SbirsDetectionRecordList records;
  records.push_back(MakeSbirsRecord(/*detected=*/true, /*ir_snr_linear=*/8.0f));

  const std::vector<DetectionRecord> detections =
      AdaptSbirsDetectionsToDetectionRecords(kSbirsSourceId, records);

  ASSERT_EQ(detections.size(), 1U);
  const DetectionRecord& record = detections[0];
  EXPECT_EQ(record.key, 0U);  // 无身份 → 走方位相干关联
  EXPECT_EQ(record.source_id, kSbirsSourceId);
  EXPECT_TRUE(record.has_bearing);
  // 2.0943951f rad × rad2deg（float 运算）≈ 120.000008°。
  EXPECT_NEAR(record.bearing_az_deg, 120.0, 1.0e-4);
  EXPECT_NEAR(record.bearing_el_deg, -30.0, 1.0e-4);
  EXPECT_DOUBLE_EQ(record.verdict, 1.0);
  EXPECT_DOUBLE_EQ(record.quality, 1.0);  // 8.0 / 4 → 2.0 夹取到 1.0
  EXPECT_TRUE(record.has_timestamp);
  EXPECT_DOUBLE_EQ(record.timestamp_s, 12.345);
}

TEST(SensorAdaptersTest, SbirsQualityNormalizedToWfovThreshold) {
  sbirs_sensor::output::SbirsDetectionRecordList records;
  records.push_back(MakeSbirsRecord(/*detected=*/true, /*ir_snr_linear=*/2.0f));
  records.push_back(MakeSbirsRecord(/*detected=*/true, /*ir_snr_linear=*/-1.0f));

  const std::vector<DetectionRecord> detections =
      AdaptSbirsDetectionsToDetectionRecords(kSbirsSourceId, records);

  ASSERT_EQ(detections.size(), 2U);
  EXPECT_DOUBLE_EQ(detections[0].quality, 0.5);  // 2.0 / 4
  EXPECT_DOUBLE_EQ(detections[1].quality, 0.0);  // 负值夹取到 0
}

TEST(SensorAdaptersTest, SbirsNotDetectedIsSkipped) {
  sbirs_sensor::output::SbirsDetectionRecordList records;
  records.push_back(MakeSbirsRecord(/*detected=*/false, /*ir_snr_linear=*/4.0f));

  const std::vector<DetectionRecord> detections =
      AdaptSbirsDetectionsToDetectionRecords(kSbirsSourceId, records);

  EXPECT_TRUE(detections.empty());
}

// ============================ 通用 ============================

TEST(SensorAdaptersTest, SourceIdPassedThrough) {
  // 验证调用方自定义 source_id 透传（不强制使用常量）。
  const oneq::coordinate::EcefPositionM& ecef = kTestEcef;
  airborne_radar::session::ArExternalTrackOutputFrame frame;
  frame.tracks = {MakeArTrack(1U, airborne_radar::session::TrackStatus::kConfirmed,
                              ecef, 0.5f)};

  const std::vector<DetectionRecord> detections =
      AdaptArTracksToDetectionRecords(42U, frame);

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections[0].source_id, 42U);
}

TEST(SensorAdaptersTest, EmptyInputYieldsEmptyOutput) {
  airborne_radar::session::ArExternalTrackOutputFrame ar_frame;
  EXPECT_TRUE(AdaptArTracksToDetectionRecords(kArSourceId, ar_frame).empty());

  electronic_surveillance_radar::session::EmitterHypothesisList esr_hypotheses;
  EXPECT_TRUE(AdaptEsrHypothesesToDetectionRecords(kEsrSourceId, esr_hypotheses).empty());

  electro_optical_sensor::output::EosDetectionRecordList eos_records;
  EXPECT_TRUE(AdaptEosDetectionsToDetectionRecords(kEosSourceId, eos_records).empty());

  sbirs_sensor::output::SbirsDetectionRecordList sbirs_records;
  EXPECT_TRUE(AdaptSbirsDetectionsToDetectionRecords(kSbirsSourceId, sbirs_records).empty());
}

// ============================ RIR 特征量测适配 ============================

namespace rir = remote_identification_radar::session;

/// 全四维有效特征量测记录（11 维布局期望值见各断言）。
rir::RirFeatureMeasurementRecord MakeRirMeasurement(std::uint64_t key) {
  rir::RirFeatureMeasurementRecord record;
  record.association_key = key;
  record.features.rcs.valid = true;
  record.features.rcs.mean_dbsm = -3.0f;
  record.features.rcs.quality = 0.8f;
  record.features.motion.valid = true;
  record.features.motion.speed_m_per_s = 250.0f;
  record.features.motion.altitude_m = 3000.0f;
  record.features.motion.acceleration_m_per_s2 = 12.0f;
  record.features.motion.turn_radius_m = 10000.0f;
  record.features.motion.quality = 0.4f;
  record.features.polarization.valid = true;
  record.features.polarization.energy_difference_db = 2.5f;
  record.features.polarization.relative_difference_db = 1.5f;
  record.features.polarization.energy_sum_db = -6.0f;
  record.features.polarization.quality = 0.6f;
  record.features.range_profile.valid = true;
  record.features.range_profile.length_m = 8.0f;
  record.features.range_profile.peak_count = 3U;
  record.features.range_profile.peak_energy_concentration = 0.75f;
  record.features.range_profile.quality = 0.2f;
  record.valid_feature_mask = 0x0FU;
  record.look_az_deg = 30.0f;
  record.look_el_deg = 10.0f;
  return record;
}

TEST(SensorAdaptersTest, RirMeasurementsAdaptWithElevenDimensionLayout) {
  rir::RirFeatureMeasurementFrame frame;
  frame.records.push_back(MakeRirMeasurement(7U));

  const std::vector<DetectionRecord> detections =
      AdaptRirFeatureMeasurementsToDetectionRecords(kRirSourceId, frame);

  ASSERT_EQ(detections.size(), 1U);
  const DetectionRecord& detection = detections[0];
  EXPECT_EQ(detection.key, 7U);  // 库内键透传（ESR hypothesis_id 先例）
  EXPECT_EQ(detection.source_id, kRirSourceId);
  EXPECT_TRUE(detection.has_bearing);
  // east→north 换算：look_az=30°（自东 30°）→ 自北 60°；el 原值。
  EXPECT_DOUBLE_EQ(detection.bearing_az_deg, 60.0);
  EXPECT_DOUBLE_EQ(detection.bearing_el_deg, 10.0);
  EXPECT_DOUBLE_EQ(detection.verdict, 1.0);
  // 质量 = 四有效维等权均值 (0.8+0.4+0.6+0.2)/4。
  EXPECT_NEAR(detection.quality, 0.5, 1e-6);  // 质量源自 float 求和
  EXPECT_FALSE(detection.has_position);
  EXPECT_FALSE(detection.has_sensor_origin);

  // 11 维固定布局逐维断言（冻结契约 §3.2）。
  ASSERT_EQ(detection.feature.size(), 11U);
  EXPECT_DOUBLE_EQ(detection.feature[0], -3.0);                       // RCS 均值 dBsm
  EXPECT_DOUBLE_EQ(detection.feature[1], 0.25);                        // 速度 km/s
  EXPECT_DOUBLE_EQ(detection.feature[2], 3.0);                         // 高度 km
  EXPECT_DOUBLE_EQ(detection.feature[3], 12.0);                        // 加速度 m/s²
  EXPECT_DOUBLE_EQ(detection.feature[4], 4.0);                         // log10(1e4 m)
  EXPECT_DOUBLE_EQ(detection.feature[5], 2.5);                         // 极化能量差 dB
  EXPECT_DOUBLE_EQ(detection.feature[6], 1.5);                         // 极化相对差 dB
  EXPECT_DOUBLE_EQ(detection.feature[7], -6.0);                        // 极化能量和 dB
  EXPECT_DOUBLE_EQ(detection.feature[8], 8.0);                         // 距离像长度 m
  EXPECT_DOUBLE_EQ(detection.feature[9], 3.0);                         // 峰数浮点化
  EXPECT_DOUBLE_EQ(detection.feature[10], 0.75);                       // 峰能集中度
}

TEST(SensorAdaptersTest, RirInvalidDimensionsZeroFilled) {
  rir::RirFeatureMeasurementRecord record = MakeRirMeasurement(7U);
  // 仅 RCS + 极化有效；无效维即使携带非零数据也按掩码清零（掩码为权威有效性）。
  record.valid_feature_mask = 0x05U;
  record.features.motion.turn_radius_m = 50000.0f;
  record.features.range_profile.length_m = 20.0f;

  rir::RirFeatureMeasurementFrame frame;
  frame.records.push_back(record);

  const std::vector<DetectionRecord> detections =
      AdaptRirFeatureMeasurementsToDetectionRecords(kRirSourceId, frame);

  ASSERT_EQ(detections.size(), 1U);
  ASSERT_EQ(detections[0].feature.size(), 11U);
  EXPECT_DOUBLE_EQ(detections[0].feature[0], -3.0);
  EXPECT_DOUBLE_EQ(detections[0].feature[1], 0.0);
  EXPECT_DOUBLE_EQ(detections[0].feature[2], 0.0);
  EXPECT_DOUBLE_EQ(detections[0].feature[3], 0.0);
  EXPECT_DOUBLE_EQ(detections[0].feature[4], 0.0);
  EXPECT_DOUBLE_EQ(detections[0].feature[5], 2.5);
  EXPECT_DOUBLE_EQ(detections[0].feature[6], 1.5);
  EXPECT_DOUBLE_EQ(detections[0].feature[7], -6.0);
  EXPECT_DOUBLE_EQ(detections[0].feature[8], 0.0);
  EXPECT_DOUBLE_EQ(detections[0].feature[9], 0.0);
  EXPECT_DOUBLE_EQ(detections[0].feature[10], 0.0);
  // 质量只在有效维上取均值 (0.8+0.6)/2。
  EXPECT_NEAR(detections[0].quality, 0.7, 1e-6);  // 质量源自 float 求和
}

TEST(SensorAdaptersTest, RirBearingWrapsEastToNorthAcrossQuadrants) {
  rir::RirFeatureMeasurementFrame frame;
  for (const float look_az : {0.0f, 90.0f, 180.0f, -170.0f, 45.0f}) {
    rir::RirFeatureMeasurementRecord record = MakeRirMeasurement(7U);
    record.look_az_deg = look_az;
    frame.records.push_back(record);
  }

  const std::vector<DetectionRecord> detections =
      AdaptRirFeatureMeasurementsToDetectionRecords(kRirSourceId, frame);

  ASSERT_EQ(detections.size(), 5U);
  EXPECT_DOUBLE_EQ(detections[0].bearing_az_deg, 90.0);   // 东 → 北
  EXPECT_DOUBLE_EQ(detections[1].bearing_az_deg, 0.0);    // 北（东 90°）→ 0
  EXPECT_DOUBLE_EQ(detections[2].bearing_az_deg, -90.0);  // 西 → -90（wrap）
  EXPECT_DOUBLE_EQ(detections[3].bearing_az_deg, -100.0); // 260° → -100（wrap）
  EXPECT_DOUBLE_EQ(detections[4].bearing_az_deg, 45.0);   // 东北 → 45
}

TEST(SensorAdaptersTest, RirSkipsAllInvalidAndZeroKeyRecords) {
  rir::RirFeatureMeasurementFrame frame;
  rir::RirFeatureMeasurementRecord all_invalid = MakeRirMeasurement(7U);
  all_invalid.valid_feature_mask = 0U;
  frame.records.push_back(all_invalid);
  frame.records.push_back(MakeRirMeasurement(0U));  // 键 0 = 无身份（ESR 先例）
  frame.records.push_back(MakeRirMeasurement(9U));  // 正常记录

  const std::vector<DetectionRecord> detections =
      AdaptRirFeatureMeasurementsToDetectionRecords(kRirSourceId, frame);

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections[0].key, 9U);
}

TEST(SensorAdaptersTest, RirSensorOriginFromPlatformPositionAndDegradeOnFailure) {
  // 携带平台位置（由已知 LLA 反向生成 ECEF）→ 换算回 LLA 填 sensor_origin。
  rir::RirFeatureMeasurementRecord with_origin = MakeRirMeasurement(7U);
  with_origin.has_platform_position = true;
  const oneq::coordinate::EcefPositionM origin_ecef = MakeEcefFromLla(30.0, 120.0, 400.0);
  with_origin.platform_position.x_m = origin_ecef.x_m;
  with_origin.platform_position.y_m = origin_ecef.y_m;
  with_origin.platform_position.z_m = origin_ecef.z_m;

  // 地心 (0,0,0)：TryEcefToLla 失败 → 退化为无原点记录（AR 先例）。
  rir::RirFeatureMeasurementRecord geocenter = MakeRirMeasurement(8U);
  geocenter.has_platform_position = true;

  rir::RirFeatureMeasurementFrame frame;
  frame.records.push_back(with_origin);
  frame.records.push_back(geocenter);
  frame.records.push_back(MakeRirMeasurement(9U));  // 缺省：无原点

  const std::vector<DetectionRecord> detections =
      AdaptRirFeatureMeasurementsToDetectionRecords(kRirSourceId, frame);

  ASSERT_EQ(detections.size(), 3U);
  EXPECT_TRUE(detections[0].has_sensor_origin);
  EXPECT_NEAR(detections[0].sensor_origin.latitude_deg, 30.0, 1e-4);
  EXPECT_NEAR(detections[0].sensor_origin.longitude_deg, 120.0, 1e-4);
  EXPECT_NEAR(detections[0].sensor_origin.altitude_m, 400.0, 1e-1);
  EXPECT_FALSE(detections[0].has_position);  // 缺省斜距 0：有原点无位置
  EXPECT_FALSE(detections[1].has_sensor_origin);
  EXPECT_FALSE(detections[1].has_position);
  EXPECT_FALSE(detections[2].has_sensor_origin);
  EXPECT_FALSE(detections[2].has_position);
}

TEST(SensorAdaptersTest, RirPositionFromLookRangeAndOrigin) {
  // ComputeLookAngles 逆运算：east=3000、north=4000、up=2000 → az/el/range。
  constexpr double kDegPerRad = 180.0 / 3.14159265358979323846;
  const oneq::coordinate::LlaPositionDegM origin_lla(30.0, 120.0, 400.0);
  const oneq::coordinate::EnuPositionM enu(3000.0, 4000.0, 2000.0);
  oneq::coordinate::EcefPositionM expected_ecef;
  ASSERT_TRUE(oneq::coordinate::TryEnuToEcef(enu, origin_lla, &expected_ecef));

  const double range_m =
      std::sqrt(enu.east_m * enu.east_m + enu.north_m * enu.north_m + enu.up_m * enu.up_m);
  const double horiz = std::sqrt(enu.east_m * enu.east_m + enu.north_m * enu.north_m);
  const float look_az_deg = static_cast<float>(std::atan2(enu.north_m, enu.east_m) * kDegPerRad);
  const float look_el_deg = static_cast<float>(std::atan2(enu.up_m, horiz) * kDegPerRad);

  rir::RirFeatureMeasurementRecord record = MakeRirMeasurement(7U);
  record.has_platform_position = true;
  const oneq::coordinate::EcefPositionM origin_ecef = MakeEcefFromLla(30.0, 120.0, 400.0);
  record.platform_position.x_m = origin_ecef.x_m;
  record.platform_position.y_m = origin_ecef.y_m;
  record.platform_position.z_m = origin_ecef.z_m;
  record.look_az_deg = look_az_deg;
  record.look_el_deg = look_el_deg;
  record.range_m = static_cast<float>(range_m);

  rir::RirFeatureMeasurementFrame frame;
  frame.records.push_back(record);
  const std::vector<DetectionRecord> detections =
      AdaptRirFeatureMeasurementsToDetectionRecords(kRirSourceId, frame);

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_TRUE(detections[0].has_position);
  EXPECT_TRUE(detections[0].has_bearing);
  EXPECT_TRUE(detections[0].has_sensor_origin);
  EXPECT_NEAR(detections[0].bearing_az_deg, 90.0 - static_cast<double>(look_az_deg), 1e-3);
  EXPECT_NEAR(detections[0].bearing_el_deg, static_cast<double>(look_el_deg), 1e-4);

  oneq::coordinate::EcefPositionM got_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(detections[0].position, &got_ecef));
  EXPECT_NEAR(got_ecef.x_m, expected_ecef.x_m, 1.0);
  EXPECT_NEAR(got_ecef.y_m, expected_ecef.y_m, 1.0);
  EXPECT_NEAR(got_ecef.z_m, expected_ecef.z_m, 1.0);
}

TEST(SensorAdaptersTest, RirPositionDegradesWhenRangeOrOriginUnusable) {
  const oneq::coordinate::EcefPositionM origin_ecef = MakeEcefFromLla(30.0, 120.0, 400.0);

  rir::RirFeatureMeasurementRecord zero_range = MakeRirMeasurement(7U);
  zero_range.has_platform_position = true;
  zero_range.platform_position.x_m = origin_ecef.x_m;
  zero_range.platform_position.y_m = origin_ecef.y_m;
  zero_range.platform_position.z_m = origin_ecef.z_m;
  zero_range.range_m = 0.0f;

  rir::RirFeatureMeasurementRecord negative_range = zero_range;
  negative_range.association_key = 8U;
  negative_range.range_m = -100.0f;

  rir::RirFeatureMeasurementRecord nan_range = zero_range;
  nan_range.association_key = 9U;
  nan_range.range_m = std::numeric_limits<float>::quiet_NaN();

  rir::RirFeatureMeasurementRecord no_origin = MakeRirMeasurement(10U);
  no_origin.range_m = 5000.0f;

  rir::RirFeatureMeasurementRecord geocenter = MakeRirMeasurement(11U);
  geocenter.has_platform_position = true;
  geocenter.range_m = 5000.0f;

  rir::RirFeatureMeasurementFrame frame;
  frame.records.push_back(zero_range);
  frame.records.push_back(negative_range);
  frame.records.push_back(nan_range);
  frame.records.push_back(no_origin);
  frame.records.push_back(geocenter);

  const std::vector<DetectionRecord> detections =
      AdaptRirFeatureMeasurementsToDetectionRecords(kRirSourceId, frame);

  ASSERT_EQ(detections.size(), 5U);
  EXPECT_TRUE(detections[0].has_sensor_origin);
  EXPECT_FALSE(detections[0].has_position);
  EXPECT_TRUE(detections[0].has_bearing);
  EXPECT_FALSE(detections[1].has_position);
  EXPECT_FALSE(detections[2].has_position);
  EXPECT_FALSE(detections[3].has_sensor_origin);
  EXPECT_FALSE(detections[3].has_position);
  EXPECT_FALSE(detections[4].has_sensor_origin);
  EXPECT_FALSE(detections[4].has_position);
}

TEST(SensorAdaptersTest, RirAdaptedPositionDropsVelocitySigmaBelowBurnoutGate) {
  FusionConfig config;
  config.enable_track_filtering = true;
  config.track_cycle_period_sec = 1.0;
  FusionEngine engine(config);

  const oneq::coordinate::EcefPositionM origin_ecef = MakeEcefFromLla(30.0, 120.0, 400.0);
  constexpr double kEast0 = 10000.0;
  constexpr double kNorth0 = 0.0;
  constexpr double kUp0 = 5000.0;
  constexpr double kNorthSpeed = 200.0;
  constexpr double kDegPerRad = 180.0 / 3.14159265358979323846;
  double velocity_sigma_m = 1.0e9;
  for (std::uint64_t cycle = 1U; cycle <= 40U; ++cycle) {
    const double t = static_cast<double>(cycle - 1U);
    const oneq::coordinate::EnuPositionM enu(kEast0, kNorth0 + kNorthSpeed * t, kUp0);
    const double range_m =
        std::sqrt(enu.east_m * enu.east_m + enu.north_m * enu.north_m + enu.up_m * enu.up_m);
    const double horiz = std::sqrt(enu.east_m * enu.east_m + enu.north_m * enu.north_m);
    rir::RirFeatureMeasurementRecord record = MakeRirMeasurement(7U);
    record.has_platform_position = true;
    record.platform_position.x_m = origin_ecef.x_m;
    record.platform_position.y_m = origin_ecef.y_m;
    record.platform_position.z_m = origin_ecef.z_m;
    record.look_az_deg = static_cast<float>(std::atan2(enu.north_m, enu.east_m) * kDegPerRad);
    record.look_el_deg = static_cast<float>(std::atan2(enu.up_m, horiz) * kDegPerRad);
    record.range_m = static_cast<float>(range_m);
    rir::RirFeatureMeasurementFrame frame;
    frame.records.push_back(record);
    const auto detections =
        AdaptRirFeatureMeasurementsToDetectionRecords(kRirSourceId, frame);
    ASSERT_EQ(detections.size(), 1U);
    ASSERT_TRUE(detections[0].has_position);
    const auto tracks = engine.Update(detections, cycle);
    ASSERT_EQ(tracks.size(), 1U);
    ASSERT_TRUE(tracks.front().has_kinematic_estimate);
    const auto& cov = tracks.front().kinematic_estimate.covariance_ecef;
    const double var_v = (cov[7U] + cov[21U] + cov[35U]) / 3.0;
    velocity_sigma_m = std::sqrt(std::max(var_v, 0.0));
  }
  EXPECT_LT(velocity_sigma_m, 5.0) << "RIR 位置通道应把 σ_v 降到关机护栏以下";
}

TEST(SensorAdaptersTest, RirSourceIdAndEmptyInput) {
  rir::RirFeatureMeasurementFrame frame;
  frame.records.push_back(MakeRirMeasurement(7U));

  const std::vector<DetectionRecord> detections =
      AdaptRirFeatureMeasurementsToDetectionRecords(42U, frame);
  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections[0].source_id, 42U);

  rir::RirFeatureMeasurementFrame empty_frame;
  EXPECT_TRUE(AdaptRirFeatureMeasurementsToDetectionRecords(kRirSourceId, empty_frame).empty());
}

}  // namespace
}  // namespace fusion
