#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "1q/electromagnetics/RfScene.h"

namespace oneq {
namespace electromagnetics {
namespace {

RfSceneEmission MakeNoiseEmission(std::uint64_t emission_id, double x_m, double power_w) {
  RfSceneEmission emission;
  emission.identity.sensor_entity_id = static_cast<std::uint32_t>(emission_id + 100U);
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m.x_m = x_m;
  emission.antenna.boresight_ecef.x = -1.0;
  EXPECT_TRUE(TryCreateRfNoiseWaveform(10.0, 1.0, 10.0e9, 20.0e6, power_w, &emission.waveform));
  return emission;
}

RfSceneReceiverState MakeReceiver() {
  RfSceneReceiverState receiver;
  receiver.sensor_entity_id = 1U;
  receiver.window_start_time_s = 10.0;
  receiver.window_duration_s = 1.0;
  receiver.center_frequency_hz = 10.0e9;
  receiver.bandwidth_hz = 20.0e6;
  receiver.antenna.boresight_ecef.x = 1.0;
  return receiver;
}

TEST(RfSceneTest, DistanceAndPowerCharacterizationMatchOneWayBudget) {
  const RfSceneReceiverState receiver = MakeReceiver();
  RfIncidentLinkConfig config;
  RfIncidentLinkResult near_link;
  RfIncidentLinkResult far_link;
  RfIncidentLinkResult double_power_link;
  ASSERT_TRUE(
      TryEvaluateRfIncidentLink(MakeNoiseEmission(1U, 1000.0, 10.0), receiver, config, &near_link));
  ASSERT_TRUE(
      TryEvaluateRfIncidentLink(MakeNoiseEmission(2U, 2000.0, 10.0), receiver, config, &far_link));
  ASSERT_TRUE(TryEvaluateRfIncidentLink(MakeNoiseEmission(3U, 1000.0, 20.0), receiver, config,
                                        &double_power_link));
  EXPECT_NEAR(10.0 * std::log10(far_link.received_power_w / near_link.received_power_w), -6.0206,
              1.0e-3);
  EXPECT_NEAR(10.0 * std::log10(double_power_link.received_power_w / near_link.received_power_w),
              3.0103, 1.0e-3);
}

TEST(RfSceneTest, FullPolarizationPairingAppliesMergeAndHalfRules) {
  RfSceneReceiverState receiver = MakeReceiver();
  receiver.polarization = RfScenePolarization::kHorizontal;
  RfSceneEmission full_tx = MakeNoiseEmission(1U, 1000.0, 10.0);
  full_tx.polarization = RfScenePolarization::kFullPolarization;
  RfSceneReceiverState full_rx = MakeReceiver();
  full_rx.polarization = RfScenePolarization::kFullPolarization;
  RfIncidentLinkConfig config;
  RfIncidentLinkResult link;

  // 全极化发射 → 单极化接收：正交两分量只收其一（一半功率）。
  ASSERT_TRUE(TryEvaluateRfIncidentLink(full_tx, receiver, config, &link));
  EXPECT_NEAR(link.polarization_mismatch_loss_db, 3.0103, 1.0e-4);

  // 固定极化发射 → 全极化接收：两正交通道合并收满。
  RfSceneEmission fixed_tx = MakeNoiseEmission(2U, 1000.0, 10.0);
  fixed_tx.polarization = RfScenePolarization::kHorizontal;
  ASSERT_TRUE(TryEvaluateRfIncidentLink(fixed_tx, full_rx, config, &link));
  EXPECT_DOUBLE_EQ(link.polarization_mismatch_loss_db, 0.0);

  // 全极化对全极化：0 dB。
  ASSERT_TRUE(TryEvaluateRfIncidentLink(full_tx, full_rx, config, &link));
  EXPECT_DOUBLE_EQ(link.polarization_mismatch_loss_db, 0.0);

  // 任一侧非极化公理不变：固定 3.01 dB。
  RfSceneReceiverState unpolarized_rx = MakeReceiver();
  unpolarized_rx.polarization = RfScenePolarization::kUnpolarized;
  ASSERT_TRUE(TryEvaluateRfIncidentLink(full_tx, unpolarized_rx, config, &link));
  EXPECT_NEAR(link.polarization_mismatch_loss_db, 3.0103, 1.0e-4);
  RfSceneEmission unpolarized_tx = MakeNoiseEmission(3U, 1000.0, 10.0);
  unpolarized_tx.polarization = RfScenePolarization::kUnpolarized;
  ASSERT_TRUE(TryEvaluateRfIncidentLink(unpolarized_tx, full_rx, config, &link));
  EXPECT_NEAR(link.polarization_mismatch_loss_db, 3.0103, 1.0e-4);
}

TEST(RfSceneTest, PropagationDelayDopplerAndPriorCycleArrivalAreExplicit) {
  RfSceneReceiverState receiver = MakeReceiver();
  receiver.window_start_time_s = 10.0;
  receiver.window_duration_s = 0.001;
  receiver.velocity_ecef_mps.x_mps = 300.0;
  RfSceneEmission emission = MakeNoiseEmission(1U, 299792.458, 4.0);
  ASSERT_TRUE(TryCreateRfNoiseWaveform(9.999, 0.001, 10.0e9, 20.0e6, 4.0, &emission.waveform));
  RfIncidentLinkResult link;
  ASSERT_TRUE(TryEvaluateRfIncidentLink(emission, receiver, RfIncidentLinkConfig{}, &link));
  EXPECT_NEAR(link.propagation_delay_s, 0.001, 1.0e-12);
  EXPECT_GT(link.doppler_shift_hz, 0.0);
  EXPECT_NEAR(link.arrival_start_time_s, 10.0, 1.0e-12);
  EXPECT_GT(link.received_power_w, 0.0);
}

TEST(RfSceneTest, PulseTrainAndSweepUseParameterizedTimeFrequencyOccupancy) {
  RfSceneReceiverState receiver = MakeReceiver();
  receiver.window_duration_s = 0.01;
  RfSceneEmission pulse = MakeNoiseEmission(1U, 1000.0, 100.0);
  ASSERT_TRUE(TryCreateRfPulseTrainWaveform(10.0, 10.0e9, 10.0e6, 100.0, 1.0e-4, 1.0e-3, 10U, 0.1,
                                            7U, 3U, &pulse.waveform));
  RfIncidentLinkResult pulse_link;
  ASSERT_TRUE(TryEvaluateRfIncidentLink(pulse, receiver, RfIncidentLinkConfig{}, &pulse_link));
  EXPECT_GT(pulse_link.time_overlap_fraction, 0.05);
  EXPECT_LT(pulse_link.time_overlap_fraction, 0.15);

  RfSceneEmission sweep = MakeNoiseEmission(2U, 1000.0, 100.0);
  ASSERT_TRUE(TryCreateRfLinearSweepWaveform(10.0, 0.01, 9.9e9, 10.1e9, 2.0e6, 100.0, 0.01,
                                             &sweep.waveform));
  RfIncidentLinkResult sweep_link;
  ASSERT_TRUE(TryEvaluateRfIncidentLink(sweep, receiver, RfIncidentLinkConfig{}, &sweep_link));
  EXPECT_GT(sweep_link.frequency_overlap_fraction, 0.05);
  EXPECT_LT(sweep_link.frequency_overlap_fraction, 0.20);
}

TEST(RfSceneTest, ArrivalActivityReplaysPulseJitterAndSweepFrequency) {
  RfWaveformSchedule pulse;
  ASSERT_TRUE(TryCreateRfPulseTrainWaveform(10.0, 3.0e9, 1.0e6, 100.0, 1.0e-3,
                                            10.0e-3, 4U, 0.15, 123U, 9U, &pulse));
  bool active = false;
  double center_hz = 0.0;
  ASSERT_TRUE(TryEvaluateRfArrivalActivity(pulse, 0.25, 1200.0, 10.2505, &active,
                                           &center_hz));
  EXPECT_TRUE(active);
  EXPECT_DOUBLE_EQ(center_hz, 3.0e9 + 1200.0);
  ASSERT_TRUE(TryEvaluateRfArrivalActivity(pulse, 0.25, 1200.0, 10.255, &active,
                                           &center_hz));
  EXPECT_FALSE(active);
  EXPECT_DOUBLE_EQ(center_hz, 0.0);
  double pulse_start_s = 0.0;
  ASSERT_TRUE(TryResolveRfPulseStartTime(pulse, 1U, &pulse_start_s));
  EXPECT_GT(pulse_start_s, 10.0);
  EXPECT_FALSE(TryResolveRfPulseStartTime(pulse, pulse.pulse_count, &pulse_start_s));

  RfWaveformSchedule sweep;
  ASSERT_TRUE(TryCreateRfLinearSweepWaveform(20.0, 2.0, 2.9e9, 3.1e9, 1.0e6, 50.0,
                                             1.0, &sweep));
  ASSERT_TRUE(TryEvaluateRfArrivalActivity(sweep, 0.5, -500.0, 21.0, &active, &center_hz));
  EXPECT_TRUE(active);
  EXPECT_NEAR(center_hz, 3.0e9 - 500.0, 1.0e-6);
}

TEST(RfSceneTest, PulseJitterKeepsFirstPulseAtDeclaredWorldTime) {
  RfWaveformSchedule pulse;
  ASSERT_TRUE(TryCreateRfPulseTrainWaveform(10.0, 3.0e9, 1.0e6, 100.0, 1.0e-3, 10.0e-3, 32U, 0.15,
                                            123U, 9U, &pulse));
  EXPECT_DOUBLE_EQ(pulse.activity_start_time_s, 10.0);

  double first_pulse_start_s = 0.0;
  ASSERT_TRUE(TryResolveRfPulseStartTime(pulse, 0U, &first_pulse_start_s));
  EXPECT_DOUBLE_EQ(first_pulse_start_s, 10.0);

  const double activity_end_s = pulse.activity_start_time_s + pulse.activity_duration_s;
  double previous_pulse_start_s = first_pulse_start_s;
  for (std::uint32_t index = 1U; index < pulse.pulse_count; ++index) {
    double pulse_start_s = 0.0;
    ASSERT_TRUE(TryResolveRfPulseStartTime(pulse, index, &pulse_start_s));
    EXPECT_GT(pulse_start_s, previous_pulse_start_s);
    EXPECT_GE(pulse_start_s, pulse.activity_start_time_s);
    EXPECT_LE(pulse_start_s + pulse.pulse_width_s, activity_end_s);
    previous_pulse_start_s = pulse_start_s;
  }
}

TEST(RfSceneTest, CoSiteRequiresIsolationPathAndNearFieldGateRejectsUnmatched) {
  RfSceneReceiverState receiver = MakeReceiver();
  receiver.sensor_entity_id = 30U;
  RfSceneEmission emission = MakeNoiseEmission(1U, 0.0, 100.0);
  emission.identity.sensor_entity_id = 20U;
  emission.position_ecef_m = receiver.position_ecef_m;
  RfIncidentLinkResult untouched;
  untouched.received_power_w = 99.0;
  // 未配对隔离表且在近场门内（0m < 1m），拒解返回 false
  EXPECT_FALSE(TryEvaluateRfIncidentLink(emission, receiver, RfIncidentLinkConfig{}, &untouched));
  EXPECT_DOUBLE_EQ(untouched.received_power_w, 99.0);

  // 命中隔离表，判定为共址，时延置零，计算隔离
  receiver.co_site_paths.push_back(RfCoSiteIsolationPath{20U, 80.0});
  RfIncidentLinkResult isolated;
  ASSERT_TRUE(TryEvaluateRfIncidentLink(emission, receiver, RfIncidentLinkConfig{}, &isolated));
  EXPECT_TRUE(isolated.is_co_site);
  EXPECT_DOUBLE_EQ(isolated.propagation_delay_s, 0.0);
  EXPECT_NEAR(isolated.received_power_before_overlap_w, 1.0e-6, 1.0e-15);
}

TEST(RfSceneTest, ValidationAndAggregationFailClosedAndIgnoreInputOrder) {
  EXPECT_FALSE(TryCreateRfNoiseWaveform(10.0, 1.0, 10.0e9, 20.0e6, 10.0, nullptr));

  RfSceneFrame scene;
  scene.world_cycle_index = 1U;
  scene.window_start_time_s = 10.0;
  scene.window_duration_s = 1.0;
  scene.emissions.push_back(MakeNoiseEmission(1U, 1000.0, 10.0));
  scene.emissions.push_back(MakeNoiseEmission(2U, 2000.0, 10.0));
  EXPECT_TRUE(TryValidateRfSceneFrame(scene));
  scene.emissions[1].identity.emission_id = 1U;
  EXPECT_TRUE(TryValidateRfSceneFrame(scene));
  scene.emissions[1].identity.sensor_entity_id =
      scene.emissions[0].identity.sensor_entity_id;
  EXPECT_FALSE(TryValidateRfSceneFrame(scene));
  scene.emissions[1].identity.sensor_entity_id = 102U;
  scene.emissions[1].identity.emission_id = 2U;
  scene.emissions[1].waveform.transmit_power_w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(TryValidateRfSceneFrame(scene));

  RfIncidentLinkResult first;
  first.identity = RfEmissionIdentity{10U, 1U};
  first.receiver_sensor_entity_id = 1U;
  first.received_power_w = 1.0;
  RfIncidentLinkResult second = first;
  second.identity.emission_id = 2U;
  second.received_power_w = 2.0;
  double forward = 0.0;
  double reverse = 0.0;
  ASSERT_TRUE(TryAggregateRfIncidentPower({first, second}, &forward));
  ASSERT_TRUE(TryAggregateRfIncidentPower({second, first}, &reverse));
  EXPECT_DOUBLE_EQ(forward, reverse);
  EXPECT_DOUBLE_EQ(forward, 3.0);
  double untouched_sum = 77.0;
  EXPECT_FALSE(TryAggregateRfIncidentPower({first, first}, &untouched_sum));
  EXPECT_DOUBLE_EQ(untouched_sum, 77.0);
}

TEST(RfSceneTest, FrameMatchesCycleWindowChecksEnvelopeEquality) {
  // 共享谓词只比较三字段精确相等，不复用 TryValidateRfSceneFrame，也不解释空帧语义。
  RfSceneFrame frame;
  frame.world_cycle_index = 7U;
  frame.window_start_time_s = 100.0;
  frame.window_duration_s = 1.0;

  EXPECT_TRUE(RfFrameMatchesCycleWindow(frame, 7U, 100.0, 1.0));

  // 任一 envelope 字段偏移即失配。
  EXPECT_FALSE(RfFrameMatchesCycleWindow(frame, 8U, 100.0, 1.0));
  EXPECT_FALSE(RfFrameMatchesCycleWindow(frame, 7U, 100.1, 1.0));
  EXPECT_FALSE(RfFrameMatchesCycleWindow(frame, 7U, 100.0, 2.0));

  // 空帧也照常比较 envelope：本谓词不豁免空帧，空帧策略由调用方负责。
  RfSceneFrame empty;
  empty.world_cycle_index = 0U;
  empty.window_start_time_s = 0.0;
  empty.window_duration_s = 0.0;
  EXPECT_TRUE(RfFrameMatchesCycleWindow(empty, 0U, 0.0, 0.0));
}

}  // namespace
}  // namespace electromagnetics
}  // namespace oneq
