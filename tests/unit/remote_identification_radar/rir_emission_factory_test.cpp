// Copyright 2026. All Rights Reserved.
//
// @file rir_emission_factory_test.cpp
// @brief 验证 RIR 自发射构造（载频、ECEF boresight、无效指向拒绝）。

#include <gtest/gtest.h>

#include <cmath>

#include "1q/coordinate/position_transform.h"
#include "remote_identification_radar/dwell/RirEmissionFactory.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using config::RirHardwareConfig;
using dwell::RirEmissionFactory;
using dwell::RirRfCycleInput;

RirRfCycleInput MakeDefaultRfInput() {
  RirRfCycleInput input;
  input.sensor_entity_id = 42U;
  input.beam_pointing_deg.az_deg = 0.0f;
  input.beam_pointing_deg.el_deg = 0.0f;
  input.window_start_time_s = 1.0;
  input.window_duration_s = 0.05;
  oneq::coordinate::LlaPositionDegM lla{};
  lla.latitude_deg = 30.0;
  lla.longitude_deg = 120.0;
  lla.altitude_m = 1000.0;
  oneq::coordinate::TryLlaToEcef(lla, &input.platform_position_ecef_m);
  return input;
}

TEST(RirEmissionFactoryTest, ResolveCarrierHzUsesWorkingFrequency) {
  RirHardwareConfig hardware;
  hardware.transmitter.frequency_hz = 3.1e9f;
  EXPECT_DOUBLE_EQ(RirEmissionFactory::ResolveCarrierHz(hardware.transmitter, 0U), 3.1e9);
  EXPECT_DOUBLE_EQ(RirEmissionFactory::ResolveCarrierHz(hardware.transmitter, 7U), 3.1e9);
}

TEST(RirEmissionFactoryTest, BuildsEmissionWithBoresightAndIdentity) {
  RirHardwareConfig hardware;
  hardware.transmitter.peak_power_w = 5.0e5f;
  hardware.transmitter.pulse_width_s = 13e-6f;

  const RirRfCycleInput input = MakeDefaultRfInput();
  oneq::electromagnetics::RfSceneEmission emission;
  ASSERT_TRUE(RirEmissionFactory::TryBuildEmission(
      input, hardware, 7U, 3.0e9, 1.0 / 300.0, 10U, 123U, 1U, &emission));
  EXPECT_EQ(emission.identity.sensor_entity_id, 42U);
  EXPECT_EQ(emission.identity.emission_id, 7U);
  const double boresight_norm = std::sqrt(emission.antenna.boresight_ecef.x *
                                              emission.antenna.boresight_ecef.x +
                                          emission.antenna.boresight_ecef.y *
                                              emission.antenna.boresight_ecef.y +
                                          emission.antenna.boresight_ecef.z *
                                              emission.antenna.boresight_ecef.z);
  EXPECT_NEAR(boresight_norm, 1.0, 1.0e-6);
}

TEST(RirEmissionFactoryTest, RejectsInvalidBeamPointing) {
  RirHardwareConfig hardware;
  RirRfCycleInput input = MakeDefaultRfInput();
  input.beam_pointing_deg.el_deg = 95.0f;
  oneq::electromagnetics::RfSceneEmission emission;
  EXPECT_FALSE(RirEmissionFactory::TryBuildEmission(
      input, hardware, 1U, 3.0e9, 1.0 / 300.0, 10U, 1U, 1U, &emission));
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
