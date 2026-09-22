// config_loader 缺键门控：缺省字段保持库结构体默认值（Has 门控，非 0.0 覆写）。

#include <cstdio>
#include <fstream>
#include <string>

#include "gtest/gtest.h"

#include "config_loaders/electro_optical/config_loader.h"
#include "config_loaders/remote_identification_radar/config_loader.h"
#include "config_loaders/sar/config_loader.h"
#include "config_loaders/sbirs_sensor/config_loader.h"

namespace examples {
namespace {

const char* kTestJsonPath = "oneq_config_loader_defaults_test.json";

class ConfigLoaderDefaultsTest : public ::testing::Test {
 protected:
  void TearDown() override { std::remove(kTestJsonPath); }

  bool LoadJson(const std::string& text) {
    std::ofstream out(kTestJsonPath, std::ios::binary);
    out << text;
    return out.good();
  }
};

TEST_F(ConfigLoaderDefaultsTest, EosEmptyRootKeepsAllDefaults) {
  ASSERT_TRUE(LoadJson("{}"));
  electro_optical_sensor::config::EosSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadEosSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  EXPECT_FLOAT_EQ(config.hardware.wavelength_lower_um, 3.0f);
  EXPECT_FLOAT_EQ(config.hardware.wavelength_upper_um, 5.0f);
  EXPECT_FLOAT_EQ(config.hardware.optical_aperture_m, 0.2f);
  EXPECT_FLOAT_EQ(config.mission.frame_rate_hz, 30.0f);
  EXPECT_FLOAT_EQ(config.mission.scan_start_az_deg, -60.0f);
  EXPECT_FLOAT_EQ(config.mission.boresight_depression_deg, 45.0f);
  // 环境子块缺键同样保持库默认（1013.25 hPa / 288.15 K / 0.5）。
  EXPECT_FLOAT_EQ(
      config.environment.scenario_config.atmospheric_physics.pressure_hpa, 1013.25f);
  EXPECT_FLOAT_EQ(
      config.environment.scenario_config.atmospheric_physics.temperature_k, 288.15f);
}

TEST_F(ConfigLoaderDefaultsTest, EosPartialBlockKeepsSiblingDefaults) {
  ASSERT_TRUE(LoadJson(
      "{\"hardware\": {\"wavelength_lower_um\": 8.0},"
      " \"mission\": {\"frame_rate_hz\": 10.0}}"));
  electro_optical_sensor::config::EosSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadEosSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  EXPECT_FLOAT_EQ(config.hardware.wavelength_lower_um, 8.0f);
  EXPECT_FLOAT_EQ(config.hardware.wavelength_upper_um, 5.0f);
  EXPECT_FLOAT_EQ(config.mission.frame_rate_hz, 10.0f);
  EXPECT_FLOAT_EQ(config.mission.scan_start_az_deg, -60.0f);
}

TEST_F(ConfigLoaderDefaultsTest, EosAtmospherePartialKeyKeepsSiblings) {
  ASSERT_TRUE(LoadJson(
      "{\"environment\": {\"scenario_config\":"
      " {\"atmospheric_physics\": {\"pressure_hpa\": 900.0}}}}"));
  electro_optical_sensor::config::EosSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadEosSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  EXPECT_FLOAT_EQ(
      config.environment.scenario_config.atmospheric_physics.pressure_hpa, 900.0f);
  EXPECT_FLOAT_EQ(
      config.environment.scenario_config.atmospheric_physics.temperature_k, 288.15f);
  EXPECT_FLOAT_EQ(
      config.environment.scenario_config.atmospheric_physics.relative_humidity, 0.5f);
}

TEST_F(ConfigLoaderDefaultsTest, SarMissionMissingKeysKeepDefaults) {
  ASSERT_TRUE(LoadJson("{\"mission\": {\"platform_speed_mps\": 200.0}}"));
  sar::config::SarSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadSarSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  EXPECT_FLOAT_EQ(config.mission.platform_speed_mps, 200.0);
  EXPECT_DOUBLE_EQ(config.mission.nominal_slant_range_m, 15000.0);
  EXPECT_EQ(config.mission.range_sample_count, 4096U);
}

TEST_F(ConfigLoaderDefaultsTest, SbirsEmptyRootKeepsPoweredOnAndFocalDefaults) {
  ASSERT_TRUE(LoadJson("{}"));
  sbirs_sensor::config::SbirsSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadSbirsSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  // sensor_enabled 缺键曾静默置 false（传感器掉电）；门控后保持库默认 true。
  EXPECT_TRUE(config.sensor_enabled);
  EXPECT_FLOAT_EQ(config.hardware.focal_length_m, 2.0f);
  EXPECT_FLOAT_EQ(config.hardware.detector_pixel_pitch_m, 3.0e-5f);
  EXPECT_FLOAT_EQ(config.mission.scan_el_start_deg, 0.0f);
  EXPECT_FLOAT_EQ(config.mission.scan_el_span_deg, 0.0f);
  EXPECT_FLOAT_EQ(config.mission.scan_el_step_deg, 1.0f);
  EXPECT_FLOAT_EQ(config.policy.error_model.nav_position_sigma_m, 50.0f);
}

TEST_F(ConfigLoaderDefaultsTest, SbirsMissionElevationGridAndNavSigmaLoadFromJson) {
  ASSERT_TRUE(LoadJson(
      "{\"mission\": {\"scan_el_start_deg\": -2.0, \"scan_el_span_deg\": 4.0,"
      " \"scan_el_step_deg\": 0.5},"
      " \"policy\": {\"error_model\": {\"nav_position_sigma_m\": 25.0}}}"));
  sbirs_sensor::config::SbirsSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadSbirsSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  EXPECT_FLOAT_EQ(config.mission.scan_el_start_deg, -2.0f);
  EXPECT_FLOAT_EQ(config.mission.scan_el_span_deg, 4.0f);
  EXPECT_FLOAT_EQ(config.mission.scan_el_step_deg, 0.5f);
  EXPECT_FLOAT_EQ(config.policy.error_model.nav_position_sigma_m, 25.0f);
}

TEST_F(ConfigLoaderDefaultsTest, SbirsEmptyRootInheritsOrientationLoaderDefaults) {
  ASSERT_TRUE(LoadJson("{}"));
  sbirs_sensor::config::SbirsSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadSbirsSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  // orientation 缺段继承示例层默认集 SbirsOrientationDefaults（2026-09-01 起，
  // 库结构体默认仍全零——两者分工见 config_loader_detail.h）。
  EXPECT_FLOAT_EQ(config.orientation.mount_angles_deg.yaw_deg, 1.0f);
  EXPECT_FLOAT_EQ(config.orientation.mount_angles_deg.pitch_deg, 0.5f);
  EXPECT_FLOAT_EQ(config.orientation.mount_angles_deg.roll_deg, -0.5f);
  EXPECT_FLOAT_EQ(config.orientation.misalignment.bias_deg.yaw_deg, 0.10f);
  EXPECT_FLOAT_EQ(config.orientation.misalignment.bias_deg.pitch_deg, -0.05f);
  EXPECT_FLOAT_EQ(config.orientation.misalignment.bias_deg.roll_deg, 0.02f);
  EXPECT_FLOAT_EQ(config.orientation.misalignment.random_sigma_deg, 0.02f);
  EXPECT_EQ(config.orientation.misalignment.random_seed, 7U);
  EXPECT_EQ(config.orientation.stabilization_mode,
            sbirs_sensor::config::SbirsStabilizationMode::kBodyStabilized);
  EXPECT_FLOAT_EQ(config.orientation.sensor_scan_limits_deg.az_min_deg, -180.0f);
  EXPECT_FLOAT_EQ(config.orientation.sensor_scan_limits_deg.el_max_deg, 90.0f);
}

TEST_F(ConfigLoaderDefaultsTest, SbirsOrientationPartialOverrideKeepsSiblingDefaults) {
  ASSERT_TRUE(LoadJson(
      "{\"orientation\": {\"mount_angles_deg\": [2.0, 1.0, 0.0],"
      " \"stabilization_mode\": \"kInertialStabilized\","
      " \"sensor_scan_limits_deg\": [-90.0, 90.0, -40.0, 40.0]}}"));
  sbirs_sensor::config::SbirsSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadSbirsSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  // 写了的键覆盖默认集；没写的失准模型保持默认（缺段整体继承、有段逐键覆盖）。
  EXPECT_FLOAT_EQ(config.orientation.mount_angles_deg.yaw_deg, 2.0f);
  EXPECT_FLOAT_EQ(config.orientation.mount_angles_deg.pitch_deg, 1.0f);
  EXPECT_FLOAT_EQ(config.orientation.mount_angles_deg.roll_deg, 0.0f);
  EXPECT_EQ(config.orientation.stabilization_mode,
            sbirs_sensor::config::SbirsStabilizationMode::kInertialStabilized);
  EXPECT_FLOAT_EQ(config.orientation.sensor_scan_limits_deg.az_min_deg, -90.0f);
  EXPECT_FLOAT_EQ(config.orientation.sensor_scan_limits_deg.el_max_deg, 40.0f);
  EXPECT_FLOAT_EQ(config.orientation.misalignment.bias_deg.yaw_deg, 0.10f);
  EXPECT_FLOAT_EQ(config.orientation.misalignment.random_sigma_deg, 0.02f);
  EXPECT_EQ(config.orientation.misalignment.random_seed, 7U);
}

TEST_F(ConfigLoaderDefaultsTest, SbirsScanAzimuthReferenceDefaultAndNadirLoad) {
  sbirs_sensor::config::SbirsSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadJson("{}"));
  ASSERT_TRUE(LoadSbirsSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  EXPECT_EQ(config.mission.scan_azimuth_reference,
            sbirs_sensor::config::SbirsScanAzimuthReference::kEciAbsolute);
  ASSERT_TRUE(
      LoadJson("{\"mission\": {\"scan_azimuth_reference\": \"kNadirRelative\"}}"));
  ASSERT_TRUE(LoadSbirsSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  EXPECT_EQ(config.mission.scan_azimuth_reference,
            sbirs_sensor::config::SbirsScanAzimuthReference::kNadirRelative);
}

TEST_F(ConfigLoaderDefaultsTest, RirEmptyRootKeepsPoweredOn) {
  ASSERT_TRUE(LoadJson("{}"));
  remote_identification_radar::config::RirSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadRirSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  EXPECT_TRUE(config.sensor_enabled);
}

}  // namespace
}  // namespace examples

TEST(SbirsAcceptanceConfigLoaderTest, ReadsDirectoryAndPreservesEmptyDefault) {
  examples::JsonValue root;
  std::string error;
  const char* path = "sbirs_acceptance_directory_config_test.json";
  {
    std::ofstream file(path);
    file << R"({"acceptance_log_enabled":true,"acceptance_log_output_directory":"log/sbirs-custom"})";
  }
  sbirs_sensor::config::SbirsSessionConfig config;
  EXPECT_TRUE(config.acceptance_log_output_directory.empty());
  ASSERT_TRUE(examples::LoadSbirsSessionConfigFromFile(path, &config, &error)) << error;
  EXPECT_TRUE(config.acceptance_log_enabled);
  EXPECT_EQ(config.acceptance_log_output_directory, "log/sbirs-custom");
  std::remove(path);
}
