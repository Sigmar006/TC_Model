// Copyright 2026. All Rights Reserved.
//
// @file rir_session_config_validation_test.cpp
// @brief 验证 RIR 会话配置 hardware 域校验（对标 AR ValidateArSessionConfig hardware 段）。

#include <gtest/gtest.h>

#include <limits>

#include "1q/remote_identification_radar/config/RirSessionConfigValidation.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using config::RirSessionConfig;
using config::ValidateRirSessionConfig;

bool HasCode(const session::RirIssueList& issues, const char* code) {
  for (const session::RirIssue& issue : issues) {
    if (issue.code == code) {
      return true;
    }
  }
  return false;
}


TEST(RirSessionConfigValidationTest, AcceptsPhysicalApertureWhenNominalBeamwidthIsZero) {
  RirSessionConfig session_config;
  session_config.hardware.antenna.nominal_az_beamwidth_deg = 0.0f;
  session_config.hardware.antenna.antenna_length_m = 1.5f;
  session_config.hardware.antenna.nominal_el_beamwidth_deg = 0.0f;
  session_config.hardware.antenna.antenna_width_m = 0.8f;
  EXPECT_FALSE(HasCode(ValidateRirSessionConfig(session_config),
                        session::codes::kAntennaAzGeometryInvalid));
  EXPECT_FALSE(HasCode(ValidateRirSessionConfig(session_config),
                        session::codes::kAntennaElGeometryInvalid));
}

TEST(RirSessionConfigValidationTest, RejectsMissingOrNonFiniteAntennaGeometry) {
  RirSessionConfig session_config;
  session_config.hardware.antenna.nominal_az_beamwidth_deg = 0.0f;
  session_config.hardware.antenna.antenna_length_m = 0.0f;
  session_config.hardware.antenna.antenna_width_m = std::numeric_limits<float>::quiet_NaN();
  const auto issues = ValidateRirSessionConfig(session_config);
  EXPECT_TRUE(HasCode(issues, session::codes::kAntennaAzGeometryInvalid));
  EXPECT_TRUE(HasCode(issues, session::codes::kAntennaElGeometryInvalid));
}

TEST(RirSessionConfigValidationTest, RejectsInvalidTransmitterFrequency) {
  RirSessionConfig session_config;
  session_config.hardware.transmitter.frequency_hz = 0.0f;
  const auto issues = ValidateRirSessionConfig(session_config);
  EXPECT_TRUE(HasCode(issues, session::codes::kTransmitterFrequencyInvalid));
}

/// @brief 增益-波束宽度物理一致（B5 还债校验）：46 dBi 配 0.8°×0.8°（期望 46.1）过门。
TEST(RirSessionConfigValidationTest, AcceptsPhysicallyConsistentGainAndBeamwidth) {
  RirSessionConfig session_config;
  session_config.hardware.antenna.main_beam_gain_db = 46.0f;
  session_config.hardware.antenna.nominal_az_beamwidth_deg = 0.8f;
  session_config.hardware.antenna.nominal_el_beamwidth_deg = 0.8f;
  EXPECT_FALSE(HasCode(ValidateRirSessionConfig(session_config),
                        session::codes::kAntennaGainBeamwidthInconsistent));
}

/// @brief 增益-波束宽度物理矛盾：52 dBi 配 0.8°×0.8°（期望 46.1，差 5.9 > 3 dB）被拦。
TEST(RirSessionConfigValidationTest, RejectsInconsistentGainAndBeamwidth) {
  RirSessionConfig session_config;
  session_config.hardware.antenna.main_beam_gain_db = 52.0f;
  session_config.hardware.antenna.nominal_az_beamwidth_deg = 0.8f;
  session_config.hardware.antenna.nominal_el_beamwidth_deg = 0.8f;
  EXPECT_TRUE(HasCode(ValidateRirSessionConfig(session_config),
                      session::codes::kAntennaGainBeamwidthInconsistent));
}

/// @brief 默认档案（35 dBi 配 4°×4°，期望 32.1，差 2.9 dB）在 3 dB 容差内过门。
TEST(RirSessionConfigValidationTest, DefaultProfileGainWithinTolerance) {
  RirSessionConfig session_config;
  EXPECT_FALSE(HasCode(ValidateRirSessionConfig(session_config),
                        session::codes::kAntennaGainBeamwidthInconsistent));
}

/// @brief 标称波束宽 ↔ 孔径交叉一致：默认配置（4.0° 名义 + 1.2 m 孔径 + 3 GHz，
///        λ/L 推导 ≈4.77°，差 19%）在 50% 容差内放行。
TEST(RirSessionConfigValidationTest, DefaultBeamwidthAperturePairPasses) {
  RirSessionConfig session_config;
  EXPECT_FALSE(HasCode(ValidateRirSessionConfig(session_config),
                        session::codes::kAntennaBeamwidthApertureInconsistent));
}

/// @brief 交付场景配对（0.4° 名义 + 17.5 m 孔径 → 推导 ≈0.327°，差 18%）放行：
///        阈值 50% 的制定依据之一（另一依据为默认配置 19%）。
TEST(RirSessionConfigValidationTest, DeliveryBeamwidthAperturePairPasses) {
  RirSessionConfig session_config;
  // 增益与 0.4°×4°（默认俯仰）物理相容：10·log10(26000/1.6) ≈ 42.1 dBi。
  session_config.hardware.antenna.main_beam_gain_db = 42.0f;
  session_config.hardware.antenna.nominal_az_beamwidth_deg = 0.4f;
  session_config.hardware.antenna.antenna_length_m = 17.5f;
  const auto issues = ValidateRirSessionConfig(session_config);
  EXPECT_FALSE(HasCode(issues, session::codes::kAntennaBeamwidthApertureInconsistent));
  EXPECT_FALSE(HasCode(issues, session::codes::kAntennaGainBeamwidthInconsistent));
}

/// @brief 标称波束宽与孔径矛盾：4.0° 名义配 0.2 m 孔径（3 GHz，λ≈0.1 m →
///        推导 ≈28.6°，相对偏差 86% > 50%）被拦。
TEST(RirSessionConfigValidationTest, RejectsBeamwidthApertureContradiction) {
  RirSessionConfig session_config;
  session_config.hardware.antenna.antenna_length_m = 0.2f;
  const auto issues = ValidateRirSessionConfig(session_config);
  EXPECT_TRUE(HasCode(issues, session::codes::kAntennaBeamwidthApertureInconsistent));
}

/// @brief 扫描波位单轴采样数不超内核上限：默认配置（方位 ±60°/4° 步长 ≈31 点，
///        俯仰 60°/4° ≈16 点）远低于 4096，放行。
TEST(RirSessionConfigValidationTest, DefaultScanVolumeDoesNotExceedSampleLimit) {
  RirSessionConfig session_config;
  EXPECT_FALSE(HasCode(ValidateRirSessionConfig(session_config),
                        session::codes::kScanWaveAxisSamplesTruncated));
}

/// @brief 大扇区 + 细步长：名义波束宽委托 10 m 孔径 + 300 GHz（λ=1 mm →
///        波束宽 ≈0.0057°），方位扫满 ±180° → 单轴采样 ≈6.3 万 > 4096，
///        波位表会被内核静默截断 → 配置期拦截。
TEST(RirSessionConfigValidationTest, RejectsScanVolumeExceedingAxisSampleLimit) {
  RirSessionConfig session_config;
  session_config.hardware.transmitter.frequency_hz = 3.0e11f;  // λ = 1 mm
  // 两轴均委托孔径推导（俯仰宽度默认 1.2 m → ≈0.048°，60° 扇区 ≈1258 点不越限）。
  session_config.hardware.antenna.nominal_az_beamwidth_deg = 0.0f;
  session_config.hardware.antenna.antenna_length_m = 10.0f;
  session_config.hardware.antenna.nominal_el_beamwidth_deg = 0.0f;
  session_config.orientation.az_min_deg = -180.0f;
  session_config.orientation.az_max_deg = 180.0f;
  const auto issues = ValidateRirSessionConfig(session_config);
  EXPECT_TRUE(HasCode(issues, session::codes::kScanWaveAxisSamplesTruncated));
}

TEST(RirSessionConfigValidationTest, ValidatesTransmitterEnvelopeAndIdentity) {
  RirSessionConfig session_config;
  session_config.hardware.transmitter.peak_power_w = -1.0f;
  EXPECT_TRUE(HasCode(ValidateRirSessionConfig(session_config),
                       session::codes::kTransmitterOperatingEnvelopeInvalid));

  session_config.hardware.transmitter.peak_power_w = 1.0e6f;
  session_config.hardware.transmitter.pulse_width_s = 1.0f;
  session_config.hardware.transmitter.prf_hz = 2.0f;  // duty = 2 > 1
  EXPECT_TRUE(HasCode(ValidateRirSessionConfig(session_config),
                       session::codes::kTransmitterOperatingEnvelopeInvalid));

}

TEST(RirSessionConfigValidationTest, ValidatesReceiverRfHardwareBoundary) {
  RirSessionConfig session_config;
  session_config.hardware.receiver.maximum_linear_input_power_w = 0.0f;
  EXPECT_TRUE(HasCode(ValidateRirSessionConfig(session_config),
                       session::codes::kReceiverRfHardwareInvalid));

  session_config.hardware.receiver.maximum_linear_input_power_w = 1.0e-3f;
  session_config.hardware.receiver.cross_polarization_isolation_db =
      std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(HasCode(ValidateRirSessionConfig(session_config),
                       session::codes::kReceiverRfHardwareInvalid));
}

TEST(RirSessionConfigValidationTest, RejectsInvalidRcsPhysics) {
  RirSessionConfig session_config;
  session_config.hardware.rcs_physics.physics_mix_ratio = 1.5f;
  EXPECT_TRUE(HasCode(ValidateRirSessionConfig(session_config),
                       session::codes::kRcsPhysicsInvalid));
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
