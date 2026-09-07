// Copyright 2026. All Rights Reserved.
//
// @file rir_receiver_state_builder_test.cpp
// @brief 验证 RIR 接收状态构造字段映射。

#include <gtest/gtest.h>

#include "1q/coordinate/position_transform.h"
#include "remote_identification_radar/dwell/RirEmissionFactory.h"
#include "remote_identification_radar/dwell/RirReceiverStateBuilder.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using config::RirHardwareConfig;
using dwell::RirEmissionFactory;
using dwell::RirReceiverStateBuilder;
using dwell::RirRfCycleInput;

TEST(RirReceiverStateBuilderTest, MapsDefaultsAndMatchedFilterBandwidth) {
  RirHardwareConfig hardware;
  hardware.receiver.maximum_linear_input_power_w = 2.0e-3f;
  hardware.receiver.noise_figure_db = 3.5f;
  hardware.transmitter.bandwidth_hz = 4.5e6f;

  RirRfCycleInput input;
  input.sensor_entity_id = 9U;
  input.window_start_time_s = 2.0;
  input.window_duration_s = 0.1;
  oneq::coordinate::LlaPositionDegM lla{};
  lla.latitude_deg = 30.0;
  lla.longitude_deg = 120.0;
  lla.altitude_m = 500.0;
  oneq::coordinate::TryLlaToEcef(lla, &input.platform_position_ecef_m);

  oneq::electromagnetics::RfSceneEmission emission;
  ASSERT_TRUE(RirEmissionFactory::TryBuildEmission(input, hardware, 1U, 3.0e9, 1.0 / 300.0, 8U, 1U,
                                                   1U, &emission));

  const auto operating_state =
      RirReceiverStateBuilder::Build(input, emission, hardware, 3.0e9);
  EXPECT_EQ(operating_state.rf_receiver.sensor_entity_id, 9U);
  EXPECT_DOUBLE_EQ(operating_state.rf_receiver.bandwidth_hz, 20.0e6);
  EXPECT_DOUBLE_EQ(operating_state.rf_receiver.minimum_far_field_range_m, 1.0);
  EXPECT_DOUBLE_EQ(operating_state.matched_filter_bandwidth_hz, 4.5e6);
  EXPECT_DOUBLE_EQ(operating_state.receiver_noise_figure_db, 3.5);
  EXPECT_NEAR(operating_state.maximum_linear_input_power_w, 2.0e-3, 1.0e-9);
  ASSERT_EQ(operating_state.rf_receiver.co_site_paths.size(), 1U);
  EXPECT_EQ(operating_state.rf_receiver.co_site_paths.front().transmitter_sensor_entity_id, 9U);
  EXPECT_DOUBLE_EQ(operating_state.rf_receiver.co_site_paths.front().isolation_db, 120.0);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
