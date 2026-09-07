// Copyright 2026. All Rights Reserved.
//
// @file rir_rf_physical_parity_test.cpp
// @brief 验证 RIR RF 物理链（emission → receiver → FE → cell → CFAR）与 AR 基线数值一致。

#include <gtest/gtest.h>

#include <cmath>

#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/coordinate/position_transform.h"
#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/session/ArEmissionFactory.h"
#include "airborne_radar/session/ArReceiverStateBuilder.h"
#include "airborne_radar/signal/detection/ArDetectionCellResolver.h"
#include "airborne_radar/signal/detection/ArRfFrontEndResolver.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "common/radar/RadarEquations.h"
#include "common/radar/VegetationClutterModel.h"
#include "remote_identification_radar/dwell/RirEmissionFactory.h"
#include "remote_identification_radar/dwell/RirReceiverStateBuilder.h"
#include "remote_identification_radar/dwell/RirDetectionCellResolver.h"
#include "remote_identification_radar/dwell/RirRfFrontEndResolver.h"
#include "remote_identification_radar/dwell/RirSignalDetector.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using config::RirHardwareConfig;
using dwell::RirDetectionCellConfig;
using dwell::RirDetectionCellResult;
using dwell::RirDetectionCellTarget;
using dwell::RirDetectionResult;
using dwell::RirDetectorConfig;
using dwell::RirEmissionFactory;
using dwell::RirReceiverStateBuilder;
using dwell::RirRfCycleInput;
using dwell::RirSignalDetector;
using dwell::RirTargetReturn;
using dwell::TryResolveRirDetectionCell;
using dwell::TryResolveRirRfFrontEnd;

namespace ar = airborne_radar;
namespace ar_det = airborne_radar::signal::detection;
namespace ar_sess = airborne_radar::session;

constexpr std::uint32_t kCycleIndex = 1U;
constexpr std::uint64_t kEmissionId = 3U;
constexpr std::uint64_t kTimingSeed = 42U;
constexpr std::uint64_t kSuccessfulCycleCount = 1U;
constexpr double kDwellSec = 1.0;
constexpr unsigned int kDetectorSeed = 99U;
constexpr double kSinrToleranceDb = 1.0e-6;

ar::config::engineering::DetectionConfig ToArDetectionConfig(
    const RirHardwareConfig& hardware) {
  ar::config::engineering::DetectionConfig detection;
  detection.transmitter.peak_power_w = hardware.transmitter.peak_power_w;
  detection.transmitter.frequency_hz = hardware.transmitter.frequency_hz;
  detection.transmitter.bandwidth_hz = hardware.transmitter.bandwidth_hz;
  detection.transmitter.pulse_width_s = hardware.transmitter.pulse_width_s;
  detection.transmitter.prf_hz = hardware.transmitter.prf_hz;
  detection.transmitter.transmit_loss_db = hardware.transmitter.transmit_loss_db;
  // RIR 已内收包络/载频表；AR 对比侧用工作点推导等价上限，保证钳位不改功率。
  detection.transmitter.maximum_peak_power_w = hardware.transmitter.peak_power_w;
  detection.transmitter.maximum_duty_cycle = 1.0f;
  detection.transmitter.maximum_pulse_energy_j =
      hardware.transmitter.peak_power_w * hardware.transmitter.pulse_width_s;
  detection.transmitter.frequency_plan_hz = {
      static_cast<double>(hardware.transmitter.frequency_hz)};

  detection.antenna.main_beam_gain_db = hardware.antenna.main_beam_gain_db;
  detection.antenna.nominal_az_beamwidth_deg = hardware.antenna.nominal_az_beamwidth_deg;
  detection.antenna.nominal_el_beamwidth_deg = hardware.antenna.nominal_el_beamwidth_deg;
  detection.antenna.pattern.model_type =
      static_cast<ar::config::engineering::AntennaPatternModelType>(
          hardware.antenna.model_type);
  detection.antenna.pattern.max_sidelobe_level_db = hardware.antenna.max_sidelobe_level_db;
  detection.antenna.pattern.backlobe_level_db = hardware.antenna.backlobe_level_db;
  // RIR 侧方向图恒开（开关已删），AR 对比口径显式置 true。
  detection.antenna.enable_directional_pattern = true;

  detection.receiver.noise_figure_db = hardware.receiver.noise_figure_db;
  detection.receiver.receive_loss_db = hardware.receiver.receive_loss_db;
  detection.receiver.cross_polarization_isolation_db =
      hardware.receiver.cross_polarization_isolation_db;
  detection.receiver.maximum_linear_input_power_w = hardware.receiver.maximum_linear_input_power_w;
  detection.receiver.preselector_bandwidth_hz = 20.0e6f;
  detection.receiver.scene_polarization = hardware.receiver.scene_polarization;
  detection.receiver.co_site_paths = {{42U, 120.0}};

  detection.signal_processing.target_processing_gain_db =
      hardware.signal_processing.target_processing_gain_db;
  detection.signal_processing.noise_processing_gain_db =
      hardware.signal_processing.noise_processing_gain_db;
  detection.signal_processing.clutter_suppression_gain_db =
      hardware.signal_processing.clutter_suppression_gain_db;
  detection.signal_processing.jamming_suppression_gain_db =
      hardware.signal_processing.jamming_suppression_gain_db;
  return detection;
}

RirDetectorConfig ToRirDetectorConfig(const RirHardwareConfig& hardware) {
  RirDetectorConfig config;
  config.transmitter = hardware.transmitter;
  config.antenna = hardware.antenna;
  config.receiver = hardware.receiver;
  return config;
}

void FillSharedCycleInputs(RirRfCycleInput* rir_input, ar_sess::ArPrepareCycleInput* ar_input) {
  oneq::coordinate::LlaPositionDegM lla{};
  lla.latitude_deg = 30.0;
  lla.longitude_deg = 120.0;
  lla.altitude_m = 1000.0;
  oneq::coordinate::TryLlaToEcef(lla, &rir_input->platform_position_ecef_m);
  rir_input->sensor_entity_id = 42U;
  rir_input->window_start_time_s = 0.0;
  rir_input->window_duration_s = kDwellSec;
  rir_input->beam_pointing_deg.az_deg = 0.0f;
  rir_input->beam_pointing_deg.el_deg = 0.0f;

  ar_input->world_cycle_index = kCycleIndex;
  ar_input->sensor_entity_id = rir_input->sensor_entity_id;
  ar_input->platform_position_ecef_m = rir_input->platform_position_ecef_m;
  ar_input->window_start_time_s = rir_input->window_start_time_s;
  ar_input->window_duration_s = rir_input->window_duration_s;
  ar_input->beam_pointing_deg.az_deg = rir_input->beam_pointing_deg.az_deg;
  ar_input->beam_pointing_deg.el_deg = rir_input->beam_pointing_deg.el_deg;
}

oneq::electromagnetics::RfSceneEmission MakeExternalJammer(std::uint64_t emission_id,
                                                             double jammer_power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = 100U + emission_id;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m.x_m = 1000.0;
  emission.antenna.boresight_ecef.x = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      0.0, kDwellSec, 3.0e9, 4.5e6, jammer_power_w, &emission.waveform));
  return emission;
}

RirDetectionCellTarget MakeCellTarget(double range_m, double rcs_m2) {
  RirDetectionCellTarget target;
  target.range_m = range_m;
  target.closing_radial_velocity_mps = 300.0;
  target.rcs_m2 = rcs_m2;
  target.effective_pulse_count = 8U;
  return target;
}

ar_det::ArDetectionCellTarget ToArCellTarget(const RirDetectionCellTarget& target) {
  ar_det::ArDetectionCellTarget ar_target;
  ar_target.range_m = target.range_m;
  ar_target.closing_radial_velocity_mps = target.closing_radial_velocity_mps;
  ar_target.rcs_m2 = target.rcs_m2;
  ar_target.effective_pulse_count = target.effective_pulse_count;
  return ar_target;
}

struct ParityOutcome {
  bool chain_resolved{false};
  bool receiver_saturated{false};
  double sinr_db{0.0};
  bool detected{false};
};

ParityOutcome RunRirChain(const RirHardwareConfig& hardware, const RirRfCycleInput& rf_input,
                          const oneq::electromagnetics::RfSceneFrame& external_scene,
                          const RirDetectionCellTarget& cell_target, double clutter_power_w,
                          float one_way_gain_db) {
  ParityOutcome outcome;
  const double carrier_hz =
      RirEmissionFactory::ResolveCarrierHz(hardware.transmitter, kCycleIndex);
  const double pri_s = 1.0 / static_cast<double>(hardware.transmitter.prf_hz);
  const std::uint32_t pulse_count = static_cast<std::uint32_t>(std::max(
      1.0, kDwellSec / pri_s));

  oneq::electromagnetics::RfSceneEmission own_emission;
  if (!RirEmissionFactory::TryBuildEmission(rf_input, hardware, kEmissionId, carrier_hz, pri_s,
                                            pulse_count, kTimingSeed, kSuccessfulCycleCount,
                                            &own_emission)) {
    return outcome;
  }

  const auto receiver_state =
      RirReceiverStateBuilder::Build(rf_input, own_emission, hardware, carrier_hz);

  oneq::electromagnetics::RfSceneFrame scene = external_scene;
  scene.world_cycle_index = kCycleIndex;
  scene.window_start_time_s = rf_input.window_start_time_s;
  scene.window_duration_s = rf_input.window_duration_s;
  scene.emissions.push_back(own_emission);

  dwell::RirRfFrontEndResult front_end;
  if (!TryResolveRirRfFrontEnd(scene, receiver_state.rf_receiver,
                               receiver_state.maximum_linear_input_power_w,
                               oneq::electromagnetics::RfIncidentLinkConfig{}, &front_end)) {
    return outcome;
  }

  RirDetectionCellConfig cell_config;
  cell_config.own_transmit_waveform = own_emission.waveform;
  cell_config.receive_window_start_time_s = rf_input.window_start_time_s;
  cell_config.receive_window_duration_s = rf_input.window_duration_s;
  cell_config.matched_filter_bandwidth_hz = static_cast<double>(hardware.transmitter.bandwidth_hz);
  cell_config.one_way_antenna_gain_dbi = static_cast<double>(one_way_gain_db);
  cell_config.receiver_loss_db = static_cast<double>(hardware.receiver.receive_loss_db);
  cell_config.receiver_noise_figure_db = static_cast<double>(hardware.receiver.noise_figure_db);
  cell_config.signal_processing = hardware.signal_processing;

  RirDetectionCellResult cell;
  if (!TryResolveRirDetectionCell(cell_config, cell_target, own_emission.identity,
                                  front_end.incident_links, clutter_power_w, &cell)) {
    return outcome;
  }

  RirSignalDetector detector(ToRirDetectorConfig(hardware));
  detector.SetRandomSeed(kDetectorSeed);
  RirTargetReturn target_return;
  target_return.rcs_m2 = static_cast<float>(cell_target.rcs_m2);
  target_return.range_m = static_cast<float>(cell_target.range_m);
  const RirDetectionResult detection = detector.DetectResolvedCell(target_return, cell);

  outcome.chain_resolved = true;
  outcome.receiver_saturated = front_end.receiver_saturated;
  outcome.sinr_db = cell.processed_single_pulse_sinr_db;
  outcome.detected = detection.detected;
  return outcome;
}

ParityOutcome RunArChain(const ar::config::engineering::DetectionConfig& detection,
                         const ar_sess::ArPrepareCycleInput& ar_input,
                         const oneq::electromagnetics::RfSceneFrame& external_scene,
                         const RirDetectionCellTarget& cell_target, double clutter_power_w,
                         float one_way_gain_db) {
  ParityOutcome outcome;
  const ar_sess::ArControlProfile baseline_control;
  const double carrier_hz = static_cast<double>(detection.transmitter.frequency_hz);
  const double pri_s = 1.0 / static_cast<double>(detection.transmitter.prf_hz);
  const std::uint32_t pulse_count = static_cast<std::uint32_t>(std::max(
      1.0, kDwellSec / pri_s));

  oneq::electromagnetics::RfSceneEmission own_emission;
  if (!ar_sess::ArEmissionFactory::TryBuildEmission(
          ar_input, detection, baseline_control, kEmissionId, carrier_hz, pri_s, pulse_count,
          kTimingSeed, kSuccessfulCycleCount, &own_emission)) {
    return outcome;
  }

  const auto receiver_state = ar_sess::ArReceiverStateBuilder::Build(
      ar_input, own_emission, detection, baseline_control, carrier_hz);

  oneq::electromagnetics::RfSceneFrame scene = external_scene;
  scene.world_cycle_index = kCycleIndex;
  scene.window_start_time_s = ar_input.window_start_time_s;
  scene.window_duration_s = ar_input.window_duration_s;
  scene.emissions.push_back(own_emission);

  ar_det::ArRfFrontEndResult front_end;
  if (!ar_det::TryResolveArRfFrontEnd(scene, receiver_state.rf_receiver,
                                      receiver_state.maximum_linear_input_power_w,
                                      oneq::electromagnetics::RfIncidentLinkConfig{}, &front_end)) {
    return outcome;
  }

  ar_det::ArDetectionCellConfig cell_config;
  cell_config.own_transmit_waveform = own_emission.waveform;
  cell_config.receive_window_start_time_s = ar_input.window_start_time_s;
  cell_config.receive_window_duration_s = ar_input.window_duration_s;
  cell_config.matched_filter_bandwidth_hz = static_cast<double>(detection.transmitter.bandwidth_hz);
  cell_config.one_way_antenna_gain_dbi = static_cast<double>(one_way_gain_db);
  cell_config.receiver_loss_db = static_cast<double>(detection.receiver.receive_loss_db);
  cell_config.receiver_noise_figure_db = static_cast<double>(detection.receiver.noise_figure_db);
  cell_config.signal_processing = detection.signal_processing;

  ar_det::ArDetectionCellResult cell;
  if (!ar_det::TryResolveArDetectionCell(cell_config, ToArCellTarget(cell_target),
                                         own_emission.identity, front_end.incident_links,
                                         clutter_power_w, &cell)) {
    return outcome;
  }

  ar_det::SignalDetector detector(detection);
  detector.SetRandomSeed(kDetectorSeed);
  ar_det::TargetReturn target_return;
  target_return.rcs_m2 = static_cast<float>(cell_target.rcs_m2);
  target_return.range_m = static_cast<float>(cell_target.range_m);
  const ar_det::DetectionResult detection_result = detector.DetectResolvedCell(target_return, cell);

  outcome.chain_resolved = true;
  outcome.receiver_saturated = front_end.receiver_saturated;
  outcome.sinr_db = cell.processed_single_pulse_sinr_db;
  outcome.detected = detection_result.detected;
  return outcome;
}

class RirRfPhysicalParityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    hardware_.transmitter.prf_hz = 300.0f;
    hardware_.transmitter.bandwidth_hz = 4.5e6f;
    hardware_.transmitter.frequency_hz = 3.0e9f;
    hardware_.antenna.main_beam_gain_db = 35.0f;
    FillSharedCycleInputs(&rir_input_, &ar_input_);
    ar_detection_ = ToArDetectionConfig(hardware_);
  }

  RirHardwareConfig hardware_;
  RirRfCycleInput rir_input_;
  ar_sess::ArPrepareCycleInput ar_input_;
  ar::config::engineering::DetectionConfig ar_detection_;
};

TEST_F(RirRfPhysicalParityTest, BaselineChainMatchesArSinrAndDetection) {
  oneq::electromagnetics::RfSceneFrame external_scene;
  const RirDetectionCellTarget cell_target = MakeCellTarget(10000.0, 1.0);
  const ParityOutcome rir =
      RunRirChain(hardware_, rir_input_, external_scene, cell_target, 0.0, 35.0f);
  const ParityOutcome ar =
      RunArChain(ar_detection_, ar_input_, external_scene, cell_target, 0.0, 35.0f);
  ASSERT_TRUE(rir.chain_resolved);
  ASSERT_TRUE(ar.chain_resolved);
  EXPECT_NEAR(rir.sinr_db, ar.sinr_db, kSinrToleranceDb);
  EXPECT_EQ(rir.detected, ar.detected);
  EXPECT_FALSE(rir.receiver_saturated);
  EXPECT_FALSE(ar.receiver_saturated);
}

TEST_F(RirRfPhysicalParityTest, SingleJammerChainMatchesAr) {
  oneq::electromagnetics::RfSceneFrame external_scene;
  external_scene.emissions.push_back(MakeExternalJammer(6U, 1.0e-9));
  const RirDetectionCellTarget cell_target = MakeCellTarget(10000.0, 1.0);
  const ParityOutcome rir =
      RunRirChain(hardware_, rir_input_, external_scene, cell_target, 0.0, 35.0f);
  const ParityOutcome ar =
      RunArChain(ar_detection_, ar_input_, external_scene, cell_target, 0.0, 35.0f);
  ASSERT_TRUE(rir.chain_resolved);
  ASSERT_TRUE(ar.chain_resolved);
  EXPECT_NEAR(rir.sinr_db, ar.sinr_db, kSinrToleranceDb);
  EXPECT_EQ(rir.detected, ar.detected);
}

TEST_F(RirRfPhysicalParityTest, DominantClutterChainMatchesAr) {
  oneq::electromagnetics::RfSceneFrame external_scene;
  const RirDetectionCellTarget cell_target = MakeCellTarget(10000.0, 1.0);
  const double clutter_power_w = 1.0e-8;
  const ParityOutcome rir =
      RunRirChain(hardware_, rir_input_, external_scene, cell_target, clutter_power_w, 35.0f);
  const ParityOutcome ar =
      RunArChain(ar_detection_, ar_input_, external_scene, cell_target, clutter_power_w, 35.0f);
  ASSERT_TRUE(rir.chain_resolved);
  ASSERT_TRUE(ar.chain_resolved);
  EXPECT_NEAR(rir.sinr_db, ar.sinr_db, kSinrToleranceDb);
  EXPECT_EQ(rir.detected, ar.detected);
}

/// @brief 环境配置杂波对账：植被模型输出的相对热噪 dB 经 common 单源换算为瓦
///        后进入两侧检测链——锁死"dB → W"口径（AR/RIR 同源换算，非绝对 dBW）。
TEST_F(RirRfPhysicalParityTest, EnvironmentDerivedClutterChainMatchesAr) {
  oneq::electromagnetics::RfSceneFrame external_scene;
  const RirDetectionCellTarget cell_target = MakeCellTarget(10000.0, 1.0);

  oneq::common::radar::VegetationScatterPhysicsConfig vegetation;
  vegetation.cover_profile = oneq::common::radar::VegetationCoverProfile::kTropicalDense;
  vegetation.enable_physical_model = true;
  const auto env = oneq::common::radar::EvaluatePropagationClutter(vegetation);
  const float thermal_noise_w = oneq::common::radar::RadarEquations::ComputeThermalNoisePower_W(
      hardware_.transmitter.bandwidth_hz, hardware_.receiver.noise_figure_db);
  const double clutter_power_w = static_cast<double>(
      oneq::common::radar::ComputeEquivalentClutterNoiseW(thermal_noise_w, env.clutter_power_db));

  // 防御断言：换算结果必须落在热噪的倍数量级（修复前 RIR 按绝对 dBW 读出 ~2 W）。
  ASSERT_GT(clutter_power_w, thermal_noise_w);
  ASSERT_LT(clutter_power_w, thermal_noise_w * 1.0e3);

  const ParityOutcome rir =
      RunRirChain(hardware_, rir_input_, external_scene, cell_target, clutter_power_w, 35.0f);
  const ParityOutcome ar =
      RunArChain(ar_detection_, ar_input_, external_scene, cell_target, clutter_power_w, 35.0f);
  ASSERT_TRUE(rir.chain_resolved);
  ASSERT_TRUE(ar.chain_resolved);
  EXPECT_NEAR(rir.sinr_db, ar.sinr_db, kSinrToleranceDb);
  EXPECT_EQ(rir.detected, ar.detected);
  EXPECT_TRUE(rir.detected);
}

TEST_F(RirRfPhysicalParityTest, ReceiverSaturationFlagMatchesAr) {
  RirHardwareConfig saturated_hw = hardware_;
  saturated_hw.receiver.maximum_linear_input_power_w = 1.0e-12f;
  const ar::config::engineering::DetectionConfig saturated_ar = ToArDetectionConfig(saturated_hw);

  oneq::electromagnetics::RfSceneFrame external_scene;
  external_scene.emissions.push_back(MakeExternalJammer(7U, 1.0e-3));
  const RirDetectionCellTarget cell_target = MakeCellTarget(10000.0, 1.0);
  const ParityOutcome rir =
      RunRirChain(saturated_hw, rir_input_, external_scene, cell_target, 0.0, 35.0f);
  const ParityOutcome ar =
      RunArChain(saturated_ar, ar_input_, external_scene, cell_target, 0.0, 35.0f);
  ASSERT_TRUE(rir.chain_resolved);
  ASSERT_TRUE(ar.chain_resolved);
  EXPECT_TRUE(rir.receiver_saturated);
  EXPECT_TRUE(ar.receiver_saturated);
  EXPECT_NEAR(rir.sinr_db, ar.sinr_db, kSinrToleranceDb);
  EXPECT_EQ(rir.detected, ar.detected);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
