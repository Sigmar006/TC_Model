#include <gtest/gtest.h>

#include <cmath>

#include "1q/electromagnetics/RfScene.h"
#include "electronic_surveillance_radar/pipeline/EsrResolutionCellLedger.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace {

oneq::electromagnetics::RfSceneReceiverState MakeReceiver() {
  oneq::electromagnetics::RfSceneReceiverState receiver;
  receiver.sensor_entity_id = 2U;
  receiver.window_start_time_s = 0.0;
  receiver.window_duration_s = 1.0;
  receiver.center_frequency_hz = 10.0e9;
  receiver.bandwidth_hz = 200.0e6;
  return receiver;
}

oneq::electromagnetics::RfIncidentLinkResult MakePulseLink(std::uint64_t emission_id,
                                                           double first_pulse_time_s,
                                                           double power_w,
                                                           double pulse_width_s = 1.0e-3,
                                                           std::uint32_t pulse_count = 1U) {
  oneq::electromagnetics::RfIncidentLinkResult link;
  link.identity.sensor_entity_id = 20U + emission_id;
  link.identity.emission_id = emission_id;
  link.receiver_sensor_entity_id = 2U;
  link.received_power_before_overlap_w = power_w;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      first_pulse_time_s, 10.0e9, 1.0e6, 1.0, pulse_width_s, 0.1, pulse_count, 0.0, emission_id, 0U,
      &link.emission_waveform));
  return link;
}

EsrArrivalBearing MakeBearing(double azimuth_deg) {
  EsrArrivalBearing bearing;
  bearing.defined = true;
  bearing.azimuth_deg = azimuth_deg;
  return bearing;
}

EsrArrivalBearing MakeBearingWithoutObservableAzimuth(double elevation_deg) {
  EsrArrivalBearing bearing;
  bearing.defined = true;
  bearing.azimuth_observable = false;
  bearing.elevation_deg = elevation_deg;
  return bearing;
}

TEST(EsrResolutionCellLedgerTest, TimeSeparatedPulsesDoNotInterfere) {
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> links;
  links.push_back(MakePulseLink(1U, 0.1, 2.0));
  links.push_back(MakePulseLink(2U, 0.6, 1.0));
  const std::vector<EsrArrivalBearing> bearings{MakeBearing(0.0),
                                                MakeBearing(0.0)};

  EsrResolutionCellLedgerResult result;
  ASSERT_TRUE(TryBuildEsrResolutionCellLedger(links, bearings, MakeReceiver(),
                                               5.0, &result));
  ASSERT_EQ(result.candidates.size(), 2U);
  EXPECT_DOUBLE_EQ(result.candidates[0].interference_power_w, 0.0);
  EXPECT_DOUBLE_EQ(result.candidates[1].interference_power_w, 0.0);
}

TEST(EsrResolutionCellLedgerTest, SameCellPublishesStrongestAndBooksOtherAsInterference) {
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> links;
  links.push_back(MakePulseLink(1U, 0.1, 2.0));
  links.push_back(MakePulseLink(2U, 0.1, 1.0));
  const std::vector<EsrArrivalBearing> bearings{MakeBearing(0.0),
                                                MakeBearing(0.0)};

  EsrResolutionCellLedgerResult result;
  ASSERT_TRUE(TryBuildEsrResolutionCellLedger(links, bearings, MakeReceiver(),
                                               5.0, &result));
  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_EQ(result.candidates[0].source_index, 0U);
  EXPECT_GT(result.candidates[0].interference_power_w, 0.0);
}

TEST(EsrResolutionCellLedgerTest,
     PolarBearingWithoutObservableAzimuthCannotBecomeCandidate) {
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> links;
  links.push_back(MakePulseLink(1U, 0.1, 2.0));
  links.push_back(MakePulseLink(2U, 0.1, 1.0));
  EsrArrivalBearing observable_polar = MakeBearing(0.0);
  observable_polar.elevation_deg = 90.0;
  const std::vector<EsrArrivalBearing> bearings{
      MakeBearingWithoutObservableAzimuth(90.0), observable_polar};

  EsrResolutionCellLedgerResult result;
  ASSERT_TRUE(TryBuildEsrResolutionCellLedger(links, bearings, MakeReceiver(),
                                               180.0, &result));
  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_EQ(result.candidates[0].source_index, 1U);
  EXPECT_GT(result.candidates[0].interference_power_w, 0.0);
}

TEST(EsrResolutionCellLedgerTest, LinearSweepUsesPartialInstantaneousChannelDwell) {
  oneq::electromagnetics::RfIncidentLinkResult link;
  link.identity.sensor_entity_id = 20U;
  link.identity.emission_id = 1U;
  link.receiver_sensor_entity_id = 2U;
  link.received_power_before_overlap_w = 1.0;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfLinearSweepWaveform(
      0.0, 1.0, 9.0e9, 11.0e9, 1.0e6, 1.0, 1.0,
      &link.emission_waveform));
  EsrResolutionCellLedgerResult result;
  ASSERT_TRUE(TryBuildEsrResolutionCellLedger(
      std::vector<oneq::electromagnetics::RfIncidentLinkResult>{link},
      std::vector<EsrArrivalBearing>{MakeBearing(0.0)}, MakeReceiver(), 5.0,
      &result));

  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_NEAR(result.candidates[0].signal_power_w, 1.0, 1.0e-12);
  EXPECT_GT(result.candidates[0].active_time_s, 0.08);
  EXPECT_LT(result.candidates[0].active_time_s, 0.12);
  EXPECT_NEAR(result.candidates[0].estimated_center_frequency_hz, 10.0e9,
              5.0e6);
}

TEST(EsrResolutionCellLedgerTest, PulsePowerAndCountAreInvariantToEmptyWindowPadding) {
  const auto link = MakePulseLink(1U, 0.1, 2.0);
  const std::vector<oneq::electromagnetics::RfIncidentLinkResult> links{link};
  const std::vector<EsrArrivalBearing> bearings{MakeBearing(0.0)};
  oneq::electromagnetics::RfSceneReceiverState short_receiver = MakeReceiver();
  short_receiver.window_duration_s = 0.5;
  const oneq::electromagnetics::RfSceneReceiverState long_receiver = MakeReceiver();

  EsrResolutionCellLedgerResult short_result;
  EsrResolutionCellLedgerResult long_result;
  ASSERT_TRUE(TryBuildEsrResolutionCellLedger(links, bearings, short_receiver, 5.0, &short_result));
  ASSERT_TRUE(TryBuildEsrResolutionCellLedger(links, bearings, long_receiver, 5.0, &long_result));
  ASSERT_EQ(short_result.candidates.size(), 1U);
  ASSERT_EQ(long_result.candidates.size(), 1U);
  EXPECT_NEAR(short_result.candidates[0].signal_power_w, 2.0, 1.0e-12);
  EXPECT_NEAR(long_result.candidates[0].signal_power_w, 2.0, 1.0e-12);
  EXPECT_NEAR(short_result.candidates[0].active_time_s, 1.0e-3, 1.0e-12);
  EXPECT_NEAR(long_result.candidates[0].active_time_s, 1.0e-3, 1.0e-12);
  EXPECT_EQ(short_result.candidates[0].effective_pulse_count, 1U);
  EXPECT_EQ(long_result.candidates[0].effective_pulse_count, 1U);
}

TEST(EsrResolutionCellLedgerTest, PulseCrossingTimeBinBoundaryCountsOnce) {
  constexpr double kTimeBinDurationS = 1.0 / 256.0;
  const auto link = MakePulseLink(1U, kTimeBinDurationS - 0.001, 3.0, 0.002);
  EsrResolutionCellLedgerResult result;
  ASSERT_TRUE(TryBuildEsrResolutionCellLedger(
      std::vector<oneq::electromagnetics::RfIncidentLinkResult>{link},
      std::vector<EsrArrivalBearing>{MakeBearing(0.0)}, MakeReceiver(), 5.0, &result));

  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_NEAR(result.candidates[0].signal_power_w, 3.0, 1.0e-12);
  EXPECT_NEAR(result.candidates[0].active_time_s, 0.002, 1.0e-12);
  EXPECT_EQ(result.candidates[0].effective_pulse_count, 1U);
}

}  // namespace
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
