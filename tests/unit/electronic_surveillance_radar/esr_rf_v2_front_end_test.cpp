#include <gtest/gtest.h>

#include "1q/coordinate/position_transform.h"
#include "1q/electromagnetics/RfScene.h"
#include "electronic_surveillance_radar/pipeline/EsrRfV2FrontEnd.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace {

session::EsrCycleInput MakeInput() {
  session::EsrCycleInput input;
  input.cycle_index = 9U;
  input.cycle_start_time_s = 10.0;
  input.dt_sec = 1.0f;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  return input;
}

oneq::electromagnetics::RfSceneEmission MakeEmission(std::uint64_t emission_id,
                                                      double x_offset_m, double power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = 20U + emission_id;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m.x_m = 6378137.0 + x_offset_m;
  emission.antenna.boresight_ecef.x = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      10.0, 1.0, 10.0e9, 2.0e6, power_w, &emission.waveform));
  return emission;
}

config::EsrHardwareConfig MakeHardware() {
  config::EsrHardwareConfig hardware;
  hardware.receiver_band_lower_hz = 1.0e9;
  hardware.receiver_band_upper_hz = 18.0e9;
  hardware.maximum_linear_input_power_w = 1.0e-30f;
  return hardware;
}

config::EsrOrientationConfig MakeOrientation() {
  return config::EsrOrientationConfig{};
}

TEST(EsrRfV2FrontEndTest, ResolvesEveryEmissionOnceAndAggregatesInStableOrder) {
  session::EsrCycleInput input = MakeInput();
  input.rf_emissions.emissions.push_back(MakeEmission(2U, 2000.0, 10.0));
  input.rf_emissions.emissions.push_back(MakeEmission(1U, 1000.0, 10.0));

  EsrRfV2FrontEndResult result;
  ASSERT_TRUE(TryResolveEsrRfV2FrontEnd(input, MakeHardware(), MakeOrientation(), 0.0, 0.0, 10.0e9, 2.0e6, 0.0,
                                        2U, &result));
  ASSERT_EQ(result.front_end_incident_links.size(), 2U);
  ASSERT_EQ(result.channel_incident_links.size(), 2U);
  EXPECT_EQ(result.front_end_incident_links[0].identity.emission_id, 1U);
  EXPECT_EQ(result.channel_incident_links[1].identity.emission_id, 2U);
  EXPECT_GT(result.total_incident_power_w, 0.0);
  EXPECT_TRUE(result.receiver_saturated);
}

TEST(EsrRfV2FrontEndTest, StrongHardwareBandSignalOutsideTunedChannelStillSaturates) {
  session::EsrCycleInput input = MakeInput();
  input.rf_emissions.emissions.push_back(MakeEmission(1U, 1000.0, 10.0));
  input.rf_emissions.emissions.back().waveform.center_frequency_hz = 10.1e9;

  config::EsrHardwareConfig hardware = MakeHardware();
  hardware.receiver_band_lower_hz = 9.0e9;
  hardware.receiver_band_upper_hz = 11.0e9;
  EsrRfV2FrontEndResult result;
  ASSERT_TRUE(TryResolveEsrRfV2FrontEnd(input, hardware, MakeOrientation(), 0.0, 0.0, 10.0e9, 2.0e6, 0.0,
                                        2U, &result));
  ASSERT_EQ(result.front_end_incident_links.size(), 1U);
  ASSERT_EQ(result.channel_incident_links.size(), 1U);
  EXPECT_GT(result.front_end_incident_links[0].received_power_w, 0.0);
  EXPECT_EQ(result.channel_incident_links[0].received_power_w, 0.0);
  EXPECT_TRUE(result.receiver_saturated);
}

TEST(EsrRfV2FrontEndTest, RejectsUnmatchedFrameBeforeProducingResult) {
  session::EsrCycleInput input = MakeInput();
  input.rf_emissions.window_start_time_s += 0.1;
  EsrRfV2FrontEndResult result;
  EXPECT_FALSE(TryResolveEsrRfV2FrontEnd(input, MakeHardware(), MakeOrientation(), 0.0, 0.0, 10.0e9, 2.0e6, 0.0,
                                         2U, &result));
}

TEST(EsrRfV2FrontEndTest, RequiresEquipmentSpecificCoSiteIsolation) {
  session::EsrCycleInput input = MakeInput();
  oneq::electromagnetics::RfSceneEmission emission = MakeEmission(1U, 0.0, 10.0);
  emission.identity.sensor_entity_id = 99U;
  input.rf_emissions.emissions.push_back(emission);
  EsrRfV2FrontEndResult result;
  EXPECT_FALSE(TryResolveEsrRfV2FrontEnd(input, MakeHardware(), MakeOrientation(), 0.0, 0.0, 10.0e9, 2.0e6, 0.0,
                                         2U, &result));

  config::EsrHardwareConfig hardware = MakeHardware();
  hardware.co_site_paths.push_back(config::EsrCoSiteIsolationPath{99U, 80.0});
  EXPECT_TRUE(TryResolveEsrRfV2FrontEnd(input, hardware, MakeOrientation(), 0.0, 0.0, 10.0e9, 2.0e6, 0.0,
                                        2U, &result));
}

TEST(EsrRfV2FrontEndTest, RealizesBoresightThroughInstallationChain) {
  // 姿态 yaw=90° + 安装方位偏置 10°：天线系 az=0 的波束应指向 ENU 方位 100°
  //（安装链旋转合成；历史角度加法在非零姿态下为近似，此处锁定链路合成语义）。
  session::EsrCycleInput input = MakeInput();
  input.platform_attitude_deg.yaw_deg = 90.0;
  input.rf_emissions.emissions.push_back(MakeEmission(1U, 1000.0, 10.0));

  config::EsrHardwareConfig hardware = MakeHardware();
  config::EsrOrientationConfig orientation = MakeOrientation();
  orientation.antenna_mount_az_deg = 10.0f;
  EsrRfV2FrontEndResult result;
  ASSERT_TRUE(TryResolveEsrRfV2FrontEnd(input, hardware, orientation, 0.0, 0.0, 10.0e9, 2.0e6, 0.0,
                                        2U, &result));

  oneq::coordinate::LlaPositionDegM platform_lla;
  oneq::coordinate::Vector3d expected_ecef;
  ASSERT_TRUE(oneq::coordinate::TryEcefToLla(input.platform_position_ecef_m,
                                             &platform_lla));
  constexpr double kPi = 3.14159265358979323846;
  const double azimuth_rad = 100.0 * kPi / 180.0;
  const oneq::coordinate::Vector3d expected_enu(std::cos(azimuth_rad),
                                                std::sin(azimuth_rad), 0.0);
  ASSERT_TRUE(oneq::coordinate::TryEnuToEcefDirection(expected_enu, platform_lla,
                                                     &expected_ecef));
  EXPECT_NEAR(result.channel_receiver.antenna.boresight_ecef.x, expected_ecef.x,
              1.0e-12);
  EXPECT_NEAR(result.channel_receiver.antenna.boresight_ecef.y, expected_ecef.y,
              1.0e-12);
  EXPECT_NEAR(result.channel_receiver.antenna.boresight_ecef.z, expected_ecef.z,
              1.0e-12);
}

}  // namespace
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
