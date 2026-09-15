/**
 * @file RirRadarEquations.cpp
 * @brief RIR 雷达方程薄适配层（common 单源）。
 */

#include "remote_identification_radar/internal/RirRadarEquations.h"

#include <cmath>

#include "common/radar/RadarEquations.h"

namespace remote_identification_radar {
namespace internal {

namespace {

oneq::common::radar::SwerlingModel ToCommonSwerling(RirSwerlingModel model) {
  switch (model) {
    case RirSwerlingModel::kSwerling1:
      return oneq::common::radar::SwerlingModel::kSwerling1;
    case RirSwerlingModel::kSwerling2:
      return oneq::common::radar::SwerlingModel::kSwerling2;
    case RirSwerlingModel::kSwerling3:
      return oneq::common::radar::SwerlingModel::kSwerling3;
    case RirSwerlingModel::kSwerling4:
      return oneq::common::radar::SwerlingModel::kSwerling4;
    case RirSwerlingModel::kSwerling0:
    default:
      return oneq::common::radar::SwerlingModel::kSwerling0;
  }
}

}  // namespace

float RirRadarEquations::ComputeEchoPowerWithGain_dBW(
    const config::hardware::RirTransmitterConfig& tx, float one_way_gain_db, float rcs_m2,
    float range_m, float propagation_loss_db) {
  return oneq::common::radar::RadarEquations::ComputeEchoPowerWithGain_dBW(
      tx.peak_power_w, tx.transmit_loss_db, tx.pulse_width_s, tx.frequency_hz, one_way_gain_db,
      rcs_m2, range_m, propagation_loss_db);
}

float RirRadarEquations::ComputeEchoPower_dBW(
    const config::hardware::RirTransmitterConfig& tx,
    const config::hardware::RirAntennaConfig& ant, float rcs_m2, float range_m,
    float propagation_loss_db) {
  return oneq::common::radar::RadarEquations::ComputeEchoPower_dBW(
      tx.peak_power_w, tx.transmit_loss_db, tx.pulse_width_s, tx.frequency_hz,
      ant.main_beam_gain_db, rcs_m2, range_m, propagation_loss_db);
}

float RirRadarEquations::ComputePulseCompressionGainDb(
    const config::hardware::RirTransmitterConfig& tx) {
  // 非法带宽/脉宽（非正或非有限，含 NaN）按无增益退化，不污染消费侧账本。
  if (!std::isfinite(tx.bandwidth_hz) || !std::isfinite(tx.pulse_width_s) ||
      !(tx.bandwidth_hz > 0.0f) || !(tx.pulse_width_s > 0.0f)) {
    return 0.0f;
  }
  const float time_bandwidth_product = tx.bandwidth_hz * tx.pulse_width_s;
  return time_bandwidth_product > 1.0f ? 10.0f * std::log10(time_bandwidth_product) : 0.0f;
}

float RirRadarEquations::ComputeThermalNoisePower_W(
    const config::hardware::RirTransmitterConfig& tx,
    const config::hardware::RirReceiverConfig& rx) {
  return oneq::common::radar::RadarEquations::ComputeThermalNoisePower_W(tx.bandwidth_hz,
                                                                          rx.noise_figure_db);
}

float RirRadarEquations::ComputeRangeErrorStdDev(float snr_db, float bandwidth_hz) {
  return oneq::common::radar::RadarEquations::ComputeRangeErrorStdDev(snr_db, bandwidth_hz);
}

float RirRadarEquations::ComputeAngleErrorStdDev(float snr_db, float beamwidth_rad) {
  return oneq::common::radar::RadarEquations::ComputeAngleErrorStdDev(snr_db, beamwidth_rad);
}

double RirRadarEquations::ComputeThreshold(double pfa, int num_pulses) {
  return oneq::common::radar::RadarEquations::ComputeThreshold(pfa, num_pulses);
}

double RirRadarEquations::MarcumQ(int order, double a, double b) {
  return oneq::common::radar::RadarEquations::MarcumQ(order, a, b);
}

float RirRadarEquations::ComputeDetectionProbability(float snr_db, float pfa,
                                                     RirSwerlingModel model, int num_pulses) {
  return oneq::common::radar::RadarEquations::ComputeDetectionProbability(
      snr_db, pfa, ToCommonSwerling(model), num_pulses);
}

bool RirRadarEquations::ThresholdDecision(float detection_prob, std::mt19937& rng) {
  return oneq::common::radar::RadarEquations::ThresholdDecision(detection_prob, rng);
}

}  // namespace internal
}  // namespace remote_identification_radar
