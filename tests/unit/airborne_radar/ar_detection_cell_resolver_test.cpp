#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "airborne_radar/signal/detection/ArDetectionCellResolver.h"

namespace airborne_radar {
namespace signal {
namespace detection {
namespace {

ArDetectionCellConfig MakeConfig() {
  ArDetectionCellConfig config;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      0.0, 10.0e9, 10.0e6, 1.0e6, 10.0e-6, 1.0e-3, 100U, 0.0, 11U, 0U,
      &config.own_transmit_waveform));
  config.receive_window_start_time_s = 0.0;
  config.receive_window_duration_s = 1.0;
  config.matched_filter_bandwidth_hz = 10.0e6;
  config.one_way_antenna_gain_dbi = 35.0;
  config.receiver_loss_db = 2.0;
  config.receiver_noise_figure_db = 4.0;
  return config;
}

oneq::electromagnetics::RfIncidentLinkResult MakeIncidentLink(
    const oneq::electromagnetics::RfWaveformSchedule& waveform, double received_power_w,
    std::uint64_t emission_id = 6U) {
  oneq::electromagnetics::RfIncidentLinkResult link;
  link.identity = oneq::electromagnetics::RfEmissionIdentity{4U, emission_id};
  link.emission_waveform = waveform;
  link.receiver_sensor_entity_id = 1U;
  link.received_power_before_overlap_w = received_power_w;
  link.received_power_w = received_power_w;
  return link;
}

ArDetectionCellTarget MakeTarget(double range_m) {
  ArDetectionCellTarget target;
  target.range_m = range_m;
  target.closing_radial_velocity_mps = 300.0;
  target.rcs_m2 = 1.0;
  target.effective_pulse_count = 8U;
  return target;
}

TEST(ArDetectionCellResolverTest, EchoRangeDelayDopplerAndCompressionArePhysical) {
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 2U};
  ArDetectionCellResult near_cell;
  ArDetectionCellResult far_cell;
  ASSERT_TRUE(
      TryResolveArDetectionCell(MakeConfig(), MakeTarget(10000.0), own, {}, 0.0, &near_cell));
  ASSERT_TRUE(
      TryResolveArDetectionCell(MakeConfig(), MakeTarget(20000.0), own, {}, 0.0, &far_cell));
  EXPECT_NEAR(10.0 * std::log10(far_cell.echo_power_w / near_cell.echo_power_w), -12.0412, 1.0e-3);
  EXPECT_NEAR(far_cell.echo_delay_s / near_cell.echo_delay_s, 2.0, 1.0e-12);
  EXPECT_GT(near_cell.two_way_doppler_shift_hz, 0.0);
  EXPECT_DOUBLE_EQ(near_cell.pulse_compression_gain, 100.0);
}

TEST(ArDetectionCellResolverTest, SuppressionOnlyChangesSinrAndPulseCountDoesNotDoubleIntegrate) {
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 2U};
  ArDetectionCellTarget target = MakeTarget(10000.0);
  ArDetectionCellResult baseline;
  ASSERT_TRUE(TryResolveArDetectionCell(MakeConfig(), target, own, {}, 0.0, &baseline));

  oneq::electromagnetics::RfWaveformSchedule jammer_waveform;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      0.0, 1.0, 10.0e9, 10.0e6, 1.0, &jammer_waveform));
  const auto jammer = MakeIncidentLink(jammer_waveform, baseline.thermal_noise_power_w * 10.0);
  ArDetectionCellResult jammed;
  ASSERT_TRUE(TryResolveArDetectionCell(MakeConfig(), target, own, {jammer}, 0.0, &jammed));
  EXPECT_LT(jammed.processed_single_pulse_sinr_db, baseline.processed_single_pulse_sinr_db);
  EXPECT_DOUBLE_EQ(jammed.echo_power_w, baseline.echo_power_w);

  target.effective_pulse_count = 64U;
  ArDetectionCellResult more_pulses;
  ASSERT_TRUE(TryResolveArDetectionCell(MakeConfig(), target, own, {jammer}, 0.0, &more_pulses));
  EXPECT_DOUBLE_EQ(more_pulses.processed_single_pulse_sinr_linear,
                   jammed.processed_single_pulse_sinr_linear);
  EXPECT_EQ(more_pulses.effective_pulse_count, 64U);
}

TEST(ArDetectionCellResolverTest, DefaultGainsMatchConservativeLedger) {
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 2U};
  const ArDetectionCellConfig config = MakeConfig();
  const ArDetectionCellTarget target = MakeTarget(10000.0);
  ArDetectionCellResult result;
  ASSERT_TRUE(TryResolveArDetectionCell(config, target, own, {}, 1.0e-15, &result));
  const double expected_sinr_db =
      10.0 * std::log10(result.echo_power_w * result.pulse_compression_gain /
                        (result.thermal_noise_power_w + result.interference_power_w +
                         result.clutter_power_w));
  EXPECT_NEAR(result.processed_single_pulse_sinr_db, expected_sinr_db, 1.0e-9);
}

TEST(ArDetectionCellResolverTest, TargetGainOffsetRaisesSinrByOffset) {
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 2U};
  const ArDetectionCellTarget target = MakeTarget(10000.0);
  ArDetectionCellConfig config = MakeConfig();
  ArDetectionCellResult baseline;
  ArDetectionCellResult boosted;
  ASSERT_TRUE(TryResolveArDetectionCell(config, target, own, {}, 0.0, &baseline));
  config.signal_processing.target_processing_gain_db = 3.0f;
  ASSERT_TRUE(TryResolveArDetectionCell(config, target, own, {}, 0.0, &boosted));
  EXPECT_NEAR(boosted.processed_single_pulse_sinr_db - baseline.processed_single_pulse_sinr_db,
              3.0, 1.0e-6);
}

TEST(ArDetectionCellResolverTest, GainOffsetIsClampedToValidRangeInSolver) {
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 2U};
  const ArDetectionCellTarget target = MakeTarget(10000.0);
  ArDetectionCellConfig config = MakeConfig();
  ArDetectionCellResult baseline;
  ArDetectionCellResult clamped_high;
  ArDetectionCellResult clamped_low;
  ASSERT_TRUE(TryResolveArDetectionCell(config, target, own, {}, 0.0, &baseline));
  config.signal_processing.target_processing_gain_db = 50.0f;
  ASSERT_TRUE(TryResolveArDetectionCell(config, target, own, {}, 0.0, &clamped_high));
  EXPECT_NEAR(clamped_high.processed_single_pulse_sinr_db - baseline.processed_single_pulse_sinr_db,
              40.0, 1.0e-6);
  config.signal_processing.target_processing_gain_db = -5.0f;
  ASSERT_TRUE(TryResolveArDetectionCell(config, target, own, {}, 0.0, &clamped_low));
  EXPECT_DOUBLE_EQ(clamped_low.processed_single_pulse_sinr_db,
                   baseline.processed_single_pulse_sinr_db);
}

TEST(ArDetectionCellResolverTest, NoiseGainOffsetLowersSinrWhenNoiseDominant) {
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 2U};
  const ArDetectionCellTarget target = MakeTarget(10000.0);
  ArDetectionCellConfig config = MakeConfig();
  ArDetectionCellResult baseline;
  ArDetectionCellResult penalized;
  ASSERT_TRUE(TryResolveArDetectionCell(config, target, own, {}, 0.0, &baseline));
  config.signal_processing.noise_processing_gain_db = 3.0f;
  ASSERT_TRUE(TryResolveArDetectionCell(config, target, own, {}, 0.0, &penalized));
  EXPECT_NEAR(baseline.processed_single_pulse_sinr_db - penalized.processed_single_pulse_sinr_db,
              3.0, 1.0e-6);
}

TEST(ArDetectionCellResolverTest, ClutterSuppressionOffsetRaisesSinrWhenClutterDominant) {
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 2U};
  const ArDetectionCellTarget target = MakeTarget(10000.0);
  ArDetectionCellConfig config = MakeConfig();
  const double dominant_clutter_w = 1.0e-8;
  ArDetectionCellResult baseline;
  ArDetectionCellResult suppressed;
  ASSERT_TRUE(
      TryResolveArDetectionCell(config, target, own, {}, dominant_clutter_w, &baseline));
  config.signal_processing.clutter_suppression_gain_db = 10.0f;
  ASSERT_TRUE(
      TryResolveArDetectionCell(config, target, own, {}, dominant_clutter_w, &suppressed));
  EXPECT_NEAR(suppressed.processed_single_pulse_sinr_db - baseline.processed_single_pulse_sinr_db,
              10.0, 1.0e-3);
  EXPECT_DOUBLE_EQ(suppressed.clutter_power_w, baseline.clutter_power_w);
}

TEST(ArDetectionCellResolverTest, JammingSuppressionOffsetRaisesSinrWhenJammingDominant) {
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 2U};
  const ArDetectionCellTarget target = MakeTarget(10000.0);
  ArDetectionCellConfig config = MakeConfig();
  ArDetectionCellResult probe;
  ASSERT_TRUE(TryResolveArDetectionCell(config, target, own, {}, 0.0, &probe));

  oneq::electromagnetics::RfWaveformSchedule jammer_waveform;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      0.0, 1.0, 10.0e9, 10.0e6, 1.0, &jammer_waveform));
  const auto jammer = MakeIncidentLink(jammer_waveform, probe.thermal_noise_power_w * 1000.0);
  ArDetectionCellResult baseline;
  ArDetectionCellResult suppressed;
  ASSERT_TRUE(TryResolveArDetectionCell(config, target, own, {jammer}, 0.0, &baseline));
  config.signal_processing.jamming_suppression_gain_db = 10.0f;
  ASSERT_TRUE(
      TryResolveArDetectionCell(config, target, own, {jammer}, 0.0, &suppressed));
  EXPECT_NEAR(suppressed.processed_single_pulse_sinr_db - baseline.processed_single_pulse_sinr_db,
              10.0, 0.05);
}

TEST(ArDetectionCellResolverTest, NonFiniteGainOffsetIsRejectedAtomically) {
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 2U};
  ArDetectionCellConfig config = MakeConfig();
  config.signal_processing.noise_processing_gain_db = std::numeric_limits<float>::quiet_NaN();
  ArDetectionCellResult untouched;
  untouched.echo_power_w = 77.0;
  EXPECT_FALSE(TryResolveArDetectionCell(config, MakeTarget(10000.0), own, {}, 0.0, &untouched));
  EXPECT_DOUBLE_EQ(untouched.echo_power_w, 77.0);
}

TEST(ArDetectionCellResolverTest, FrequencyAndPulseCellsRejectNonOverlappingEnergy) {
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 2U};
  const ArDetectionCellConfig config = MakeConfig();
  const ArDetectionCellTarget target = MakeTarget(10000.0);

  oneq::electromagnetics::RfWaveformSchedule out_of_band_waveform;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      0.0, 1.0, 10.2e9, 1.0e6, 1.0, &out_of_band_waveform));
  ArDetectionCellResult out_of_band;
  ASSERT_TRUE(TryResolveArDetectionCell(
      config, target, own, {MakeIncidentLink(out_of_band_waveform, 100.0)}, 0.0, &out_of_band));
  EXPECT_DOUBLE_EQ(out_of_band.interference_power_w, 0.0);

  const double echo_delay_s = 2.0 * target.range_m / 299792458.0;
  oneq::electromagnetics::RfWaveformSchedule aligned_pulses;
  oneq::electromagnetics::RfWaveformSchedule offset_pulses;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      echo_delay_s, 10.0e9, 10.0e6, 1.0, 10.0e-6, 1.0e-3, 100U, 0.0, 21U, 0U,
      &aligned_pulses));
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      echo_delay_s + 20.0e-6, 10.0e9, 10.0e6, 1.0, 10.0e-6, 1.0e-3, 100U, 0.0,
      22U, 0U, &offset_pulses));
  ArDetectionCellResult aligned;
  ArDetectionCellResult offset;
  ASSERT_TRUE(TryResolveArDetectionCell(
      config, target, own, {MakeIncidentLink(aligned_pulses, 2.0)}, 0.0, &aligned));
  ASSERT_TRUE(TryResolveArDetectionCell(
      config, target, own, {MakeIncidentLink(offset_pulses, 2.0)}, 0.0, &offset));
  EXPECT_NEAR(aligned.interference_power_w, 2.0, 0.01);
  EXPECT_DOUBLE_EQ(offset.interference_power_w, 0.0);
}

TEST(ArDetectionCellResolverTest, SweepDwellAndInputOrderAreResolvedDeterministically) {
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 2U};
  const ArDetectionCellConfig config = MakeConfig();
  const ArDetectionCellTarget target = MakeTarget(10000.0);
  oneq::electromagnetics::RfWaveformSchedule sweep;
  oneq::electromagnetics::RfWaveformSchedule noise;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfLinearSweepWaveform(
      0.0, 1.0, 9.95e9, 10.05e9, 1.0e6, 1.0, 0.73e-3, &sweep));
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      0.0, 1.0, 10.0e9, 10.0e6, 1.0, &noise));
  const auto first = MakeIncidentLink(sweep, 4.0, 7U);
  const auto second = MakeIncidentLink(noise, 2.0, 8U);
  ArDetectionCellResult forward;
  ArDetectionCellResult reverse;
  ASSERT_TRUE(TryResolveArDetectionCell(config, target, own, {first, second}, 0.0, &forward));
  ASSERT_TRUE(TryResolveArDetectionCell(config, target, own, {second, first}, 0.0, &reverse));
  EXPECT_GT(forward.interference_power_w, 2.0);
  EXPECT_LT(forward.interference_power_w, 6.0);
  EXPECT_DOUBLE_EQ(forward.interference_power_w, reverse.interference_power_w);
}

TEST(ArDetectionCellResolverTest, InvalidInputIsRejectedAtomically) {
  ArDetectionCellResult untouched;
  untouched.echo_power_w = 77.0;
  ArDetectionCellTarget target = MakeTarget(0.0);
  EXPECT_FALSE(TryResolveArDetectionCell(MakeConfig(), target,
                                         oneq::electromagnetics::RfEmissionIdentity{1U, 2U}, {},
                                         0.0, &untouched));
  EXPECT_DOUBLE_EQ(untouched.echo_power_w, 77.0);
}

}  // namespace
}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
