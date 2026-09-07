#include <gtest/gtest.h>

#include <algorithm>

#include "airborne_radar/signal/detection/ArRfFrontEndResolver.h"

namespace airborne_radar {
namespace signal {
namespace detection {
namespace {

oneq::electromagnetics::RfSceneEmission MakeEmission(std::uint64_t emission_id, double position_x_m,
                                                     double power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = 100U + static_cast<std::uint32_t>(emission_id);
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m.x_m = position_x_m;
  emission.antenna.boresight_ecef.x = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(10.0, 1.0, 10.0e9, 20.0e6, power_w,
                                                               &emission.waveform));
  return emission;
}

oneq::electromagnetics::RfSceneReceiverState MakeReceiver() {
  oneq::electromagnetics::RfSceneReceiverState receiver;
  receiver.sensor_entity_id = 1U;
  receiver.window_start_time_s = 10.0;
  receiver.window_duration_s = 1.0;
  receiver.center_frequency_hz = 10.0e9;
  receiver.bandwidth_hz = 20.0e6;
  receiver.antenna.boresight_ecef.x = 1.0;
  return receiver;
}

TEST(ArRfFrontEndResolverTest, AggregatesInStableIdentityOrderAndDetectsSaturation) {
  oneq::electromagnetics::RfSceneFrame scene;
  scene.world_cycle_index = 1U;
  scene.window_start_time_s = 10.0;
  scene.window_duration_s = 1.0;
  scene.emissions = {MakeEmission(2U, 1000.0, 20.0), MakeEmission(1U, 1000.0, 10.0)};

  ArRfFrontEndResult forward;
  ASSERT_TRUE(TryResolveArRfFrontEnd(scene, MakeReceiver(), 1.0,
                                     oneq::electromagnetics::RfIncidentLinkConfig{}, &forward));
  ASSERT_EQ(forward.incident_links.size(), 2U);
  EXPECT_EQ(forward.incident_links[0].identity.emission_id, 1U);
  EXPECT_EQ(forward.incident_links[1].identity.emission_id, 2U);

  std::reverse(scene.emissions.begin(), scene.emissions.end());
  ArRfFrontEndResult reverse;
  ASSERT_TRUE(TryResolveArRfFrontEnd(scene, MakeReceiver(), 1.0,
                                     oneq::electromagnetics::RfIncidentLinkConfig{}, &reverse));
  EXPECT_DOUBLE_EQ(forward.total_incident_power_w, reverse.total_incident_power_w);

  ArRfFrontEndResult saturated;
  ASSERT_TRUE(TryResolveArRfFrontEnd(scene, MakeReceiver(), forward.total_incident_power_w * 0.5,
                                     oneq::electromagnetics::RfIncidentLinkConfig{}, &saturated));
  EXPECT_TRUE(saturated.receiver_saturated);
}

TEST(ArRfFrontEndResolverTest, MissingDirectedCoSitePathRejectsAtomically) {
  const auto receiver = MakeReceiver();
  oneq::electromagnetics::RfSceneEmission emission = MakeEmission(1U, 0.0, 10.0);
  emission.identity.sensor_entity_id = 2U;
  emission.position_ecef_m = receiver.position_ecef_m;
  oneq::electromagnetics::RfSceneFrame scene;
  scene.world_cycle_index = 1U;
  scene.window_start_time_s = 10.0;
  scene.window_duration_s = 1.0;
  scene.emissions.push_back(emission);

  ArRfFrontEndResult untouched;
  untouched.total_incident_power_w = 77.0;
  EXPECT_FALSE(TryResolveArRfFrontEnd(scene, receiver, 1.0,
                                      oneq::electromagnetics::RfIncidentLinkConfig{}, &untouched));
  EXPECT_DOUBLE_EQ(untouched.total_incident_power_w, 77.0);
}

}  // namespace
}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
