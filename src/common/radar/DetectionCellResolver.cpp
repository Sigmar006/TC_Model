#include "common/radar/DetectionCellResolver.h"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <utility>
#include "common/numerics/Constants.h"
#include "common/radar/MtiMtdAcceptanceBank.h"

namespace oneq {
namespace common {
namespace radar {
namespace {

using oneq::common::numerics::kLightSpeed;
using oneq::common::numerics::kBoltzmann;
using oneq::common::numerics::kPi;
constexpr double kLinearFloor = 1.0e-300;
constexpr std::uint32_t kRepresentativePulseLimit = 8U;
constexpr std::uint32_t kSamplesPerEchoGate = 5U;
constexpr double kMaxGainOffsetDb = 40.0;

bool IsFinitePositive(double value) { return std::isfinite(value) && value > 0.0; }

double GainFactorFromDb(double gain_db) {
  const double clamped = std::min(std::max(gain_db, 0.0), kMaxGainOffsetDb);
  return std::pow(10.0, clamped / 10.0);
}

bool SameIdentity(const oneq::electromagnetics::RfEmissionIdentity& left,
                  const oneq::electromagnetics::RfEmissionIdentity& right) {
  return left.sensor_entity_id == right.sensor_entity_id &&
         left.emission_id == right.emission_id;
}

bool IsValidIdentity(const oneq::electromagnetics::RfEmissionIdentity& identity) {
  return identity.sensor_entity_id != 0U && identity.emission_id != 0U;
}

double IntervalOverlap(double left_center_hz, double left_bandwidth_hz, double right_center_hz,
                       double right_bandwidth_hz) {
  const double left_low = left_center_hz - 0.5 * left_bandwidth_hz;
  const double left_high = left_center_hz + 0.5 * left_bandwidth_hz;
  const double right_low = right_center_hz - 0.5 * right_bandwidth_hz;
  const double right_high = right_center_hz + 0.5 * right_bandwidth_hz;
  return std::max(0.0, std::min(left_high, right_high) - std::max(left_low, right_low));
}

/**
 * @brief 把多普勒折进 ±PRF/2（2026-10-22，MTI/MTD 联算）。
 */
double FoldIntoPrf(double frequency_hz, double prf_hz) {
  const double half = 0.5 * prf_hz;
  double folded = std::fmod(frequency_hz + half, prf_hz);
  if (folded < 0.0) {
    folded += prf_hz;
  }
  return folded - half;
}

/**
 * @brief 2 脉冲对消器功率响应 4·sin²(πf/PRF)（∈[0,4]，盲速处为 0）。
 */
double MtiPowerResponse(double frequency_hz, double prf_hz) {
  const double folded = FoldIntoPrf(frequency_hz, prf_hz);
  const double sine = std::sin(kPi * folded / prf_hz);
  return 4.0 * sine * sine;
}

/**
 * @brief 归一化 DFT 通道响应 |sin(Nθ/2)/(N·sin(θ/2))|²（∈[0,1]，即跨 bin 损耗）。
 */
double DftBinPower(double frequency_hz, std::uint32_t channel, double prf_hz,
                   std::uint32_t channel_count) {
  if (channel_count == 0U) {
    return 1.0;
  }
  const double center_hz =
      static_cast<double>(channel) * prf_hz / static_cast<double>(channel_count);
  const double delta_hz = FoldIntoPrf(frequency_hz - center_hz, prf_hz);
  const double theta = 2.0 * kPi * delta_hz / prf_hz;
  if (std::abs(theta) < 1.0e-14) {
    return 1.0;
  }
  const double numerator = std::sin(0.5 * static_cast<double>(channel_count) * theta);
  const double denominator = std::sin(0.5 * theta);
  if (std::abs(denominator) < 1.0e-18) {
    return 1.0;
  }
  const double magnitude = numerator / (static_cast<double>(channel_count) * denominator);
  return magnitude * magnitude;
}

bool TryCountAvailableEchoPulses(const oneq::electromagnetics::RfWaveformSchedule& waveform,
                                 double echo_delay_s, double receive_window_end_s,
                                 std::uint32_t requested_count, std::uint32_t* available_count) {
  if (available_count == nullptr) {
    return false;
  }
  std::uint32_t low = 0U;
  std::uint32_t high = std::min(requested_count, waveform.pulse_count);
  while (low < high) {
    const std::uint32_t middle = low + (high - low) / 2U;
    double pulse_start_s = 0.0;
    if (!oneq::electromagnetics::TryResolveRfPulseStartTime(waveform, middle, &pulse_start_s)) {
      return false;
    }
    if (pulse_start_s + echo_delay_s < receive_window_end_s) {
      low = middle + 1U;
    } else {
      high = middle;
    }
  }
  *available_count = low;
  return true;
}

bool TryResolveCellInterference(
    const DetectionCellConfig& config, double echo_delay_s, double target_cell_center_hz,
    std::uint32_t effective_pulse_count,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double* interference_power_w) {
  if (interference_power_w == nullptr) {
    return false;
  }
  typedef std::pair<std::uint32_t, std::uint64_t> IdentityKey;
  std::vector<std::pair<IdentityKey, const oneq::electromagnetics::RfIncidentLinkResult*> > ordered;
  ordered.reserve(incident_links.size());
  for (const auto& link : incident_links) {
    if (!IsValidIdentity(link.identity) || !std::isfinite(link.received_power_before_overlap_w) ||
        link.received_power_before_overlap_w < 0.0 || !std::isfinite(link.propagation_delay_s) ||
        link.propagation_delay_s < 0.0 || !std::isfinite(link.doppler_shift_hz) ||
        !IsFinitePositive(link.emission_waveform.occupied_bandwidth_hz)) {
      return false;
    }
    ordered.push_back(std::make_pair(
        std::make_pair(link.identity.sensor_entity_id, link.identity.emission_id),
        &link));
  }
  std::sort(ordered.begin(), ordered.end(),
            [](const std::pair<IdentityKey,
                               const oneq::electromagnetics::RfIncidentLinkResult*>& left,
               const std::pair<IdentityKey,
                               const oneq::electromagnetics::RfIncidentLinkResult*>& right) {
              return left.first < right.first;
            });
  for (std::size_t index = 1U; index < ordered.size(); ++index) {
    if (ordered[index - 1U].first == ordered[index].first) {
      return false;
    }
  }

  if (effective_pulse_count == 0U) {
    *interference_power_w = 0.0;
    return true;
  }
  const std::uint32_t representative_count =
      std::min(effective_pulse_count, kRepresentativePulseLimit);
  long double total_interference_w = 0.0L;
  for (const auto& ordered_link : ordered) {
    const auto& link = *ordered_link.second;
    if (SameIdentity(link.identity, own_emission_identity)) {
      continue;
    }
    long double link_overlap_sum = 0.0L;
    for (std::uint32_t representative = 0U; representative < representative_count;
         ++representative) {
      const std::uint32_t pulse_index =
          representative_count == 1U
              ? 0U
              : static_cast<std::uint32_t>(
                    (static_cast<std::uint64_t>(representative) *
                     static_cast<std::uint64_t>(effective_pulse_count - 1U)) /
                    static_cast<std::uint64_t>(representative_count - 1U));
      double pulse_start_s = 0.0;
      if (!oneq::electromagnetics::TryResolveRfPulseStartTime(
              config.own_transmit_waveform, pulse_index, &pulse_start_s)) {
        return false;
      }
      for (std::uint32_t sample = 0U; sample < kSamplesPerEchoGate; ++sample) {
        const double sample_fraction =
            (static_cast<double>(sample) + 0.5) / static_cast<double>(kSamplesPerEchoGate);
        const double arrival_time_s = pulse_start_s + echo_delay_s +
                                      sample_fraction * config.own_transmit_waveform.pulse_width_s;
        bool active = false;
        double arrival_center_frequency_hz = 0.0;
        if (!oneq::electromagnetics::TryEvaluateRfArrivalActivity(
                link.emission_waveform, link.propagation_delay_s, link.doppler_shift_hz,
                arrival_time_s, &active, &arrival_center_frequency_hz)) {
          return false;
        }
        if (!active) {
          continue;
        }
        const double overlap_hz =
            IntervalOverlap(target_cell_center_hz, config.matched_filter_bandwidth_hz,
                            arrival_center_frequency_hz,
                            link.emission_waveform.occupied_bandwidth_hz);
        link_overlap_sum += overlap_hz / link.emission_waveform.occupied_bandwidth_hz;
      }
    }
    const long double sample_count = static_cast<long double>(representative_count) *
                                     static_cast<long double>(kSamplesPerEchoGate);
    long double effective_power = static_cast<long double>(link.received_power_before_overlap_w);
    if (config.enable_anti_rgpo_leading_edge &&
        link.emission_waveform.kind ==
            oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain) {
      effective_power *= 0.5L;
    }
    total_interference_w += effective_power * link_overlap_sum / sample_count;
  }
  const double candidate = static_cast<double>(total_interference_w);
  if (!std::isfinite(candidate) || candidate < 0.0) {
    return false;
  }
  *interference_power_w = candidate;
  return true;
}

bool AreGainsUsable(const DetectionCellGainOffsetsDb& gains) {
  return std::isfinite(gains.target_processing_gain_db) &&
         std::isfinite(gains.noise_processing_gain_db) &&
         std::isfinite(gains.clutter_suppression_gain_db) &&
         std::isfinite(gains.jamming_suppression_gain_db);
}

}  // namespace

bool TryResolveDetectionCell(
    const DetectionCellConfig& config, const DetectionCellTarget& target,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double clutter_power_w, DetectionCellResult* result) {
  const auto& own_waveform = config.own_transmit_waveform;
  if (result == nullptr || !IsValidIdentity(own_emission_identity) ||
      own_waveform.kind != oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain ||
      !IsFinitePositive(config.receive_window_duration_s) ||
      !std::isfinite(config.receive_window_start_time_s) ||
      !IsFinitePositive(config.matched_filter_bandwidth_hz) ||
      !std::isfinite(config.one_way_antenna_gain_dbi) || !std::isfinite(config.receiver_loss_db) ||
      config.receiver_loss_db < 0.0 || !std::isfinite(config.receiver_noise_figure_db) ||
      config.receiver_noise_figure_db < 0.0 || !IsFinitePositive(config.reference_temperature_k) ||
      !AreGainsUsable(config.signal_processing) || !IsFinitePositive(target.range_m) ||
      !std::isfinite(target.closing_radial_velocity_mps) || !IsFinitePositive(target.rcs_m2) ||
      !std::isfinite(target.two_way_additional_propagation_loss_db) ||
      target.two_way_additional_propagation_loss_db < 0.0 || target.effective_pulse_count == 0U ||
      !std::isfinite(clutter_power_w) || clutter_power_w < 0.0) {
    return false;
  }
  double first_pulse_start_s = 0.0;
  if (!oneq::electromagnetics::TryResolveRfPulseStartTime(own_waveform, 0U, &first_pulse_start_s) ||
      first_pulse_start_s < config.receive_window_start_time_s ||
      !IsFinitePositive(own_waveform.center_frequency_hz) ||
      !IsFinitePositive(own_waveform.pulse_width_s) ||
      !IsFinitePositive(own_waveform.transmit_power_w)) {
    return false;
  }

  DetectionCellResult candidate;
  const double wavelength_m = kLightSpeed / own_waveform.center_frequency_hz;
  const double antenna_gain_linear = std::pow(10.0, config.one_way_antenna_gain_dbi / 10.0);
  const double total_loss_linear = std::pow(
      10.0, (config.receiver_loss_db + target.two_way_additional_propagation_loss_db) / 10.0);
  const double range_squared_m2 = target.range_m * target.range_m;
  const double range_fourth_m4 = range_squared_m2 * range_squared_m2;
  const double geometric_denominator = std::pow(4.0 * kPi, 3.0) * range_fourth_m4;
  candidate.echo_power_w = own_waveform.transmit_power_w * antenna_gain_linear *
                           antenna_gain_linear * wavelength_m * wavelength_m * target.rcs_m2 /
                           (geometric_denominator * total_loss_linear);
  candidate.echo_delay_s = 2.0 * target.range_m / kLightSpeed;
  candidate.two_way_doppler_shift_hz =
      2.0 * own_waveform.center_frequency_hz * target.closing_radial_velocity_mps /
      kLightSpeed;
  candidate.pulse_compression_gain =
      std::max(1.0, config.matched_filter_bandwidth_hz * own_waveform.pulse_width_s);
  const double noise_figure_linear = std::pow(10.0, config.receiver_noise_figure_db / 10.0);
  candidate.thermal_noise_power_w = kBoltzmann * config.reference_temperature_k *
                                    config.matched_filter_bandwidth_hz * noise_figure_linear;
  if (!TryCountAvailableEchoPulses(
          own_waveform, candidate.echo_delay_s,
          config.receive_window_start_time_s + config.receive_window_duration_s,
          target.effective_pulse_count, &candidate.effective_pulse_count) ||
      !TryResolveCellInterference(
          config, candidate.echo_delay_s,
          own_waveform.center_frequency_hz + candidate.two_way_doppler_shift_hz,
          candidate.effective_pulse_count, own_emission_identity, incident_links,
          &candidate.interference_power_w)) {
    return false;
  }
  candidate.clutter_power_w = clutter_power_w;
  const double target_gain_linear =
      GainFactorFromDb(config.signal_processing.target_processing_gain_db);
  const double noise_gain_linear =
      GainFactorFromDb(config.signal_processing.noise_processing_gain_db);
  const double jamming_suppression_linear =
      GainFactorFromDb(config.signal_processing.jamming_suppression_gain_db);
  const double clutter_suppression_linear =
      GainFactorFromDb(config.signal_processing.clutter_suppression_gain_db);
  // 中译：**MTI/MTD 联算**（2026-10-22）。开启且有效脉冲数 N≥2 时，把三件事并入 SINR：
  //   ① 2 脉冲对消器对目标的功率响应 H_mti = 4sin²(πf_fold/PRF)（盲速处→0）；
  //   ② MTD 选中通道的跨 bin 损耗 H_bin（≤1，恒为"损耗"）；
  //   ③ **相干积累增益 N**（N 个脉冲相干积累，信号功率 ×N）。
  //   对消器同时作用于干扰与杂波（同一响应），故分母两项乘以 H_mti。
  //   未开启或 N<2 时三者取 1，行为与改动前逐位一致。
  // 标识：MTI/MTD 联算（2026-10-22）。
  double mti_linear = 1.0;
  double mtd_bin_linear = 1.0;
  double coherent_linear = 1.0;
  double mti_noise_linear = 1.0;
  double folded_hz = 0.0;
  if (config.enable_mti_mtd_effects && candidate.effective_pulse_count >= 2U &&
      own_waveform.pulse_repetition_interval_s > 0.0) {
    const double prf_hz = 1.0 / own_waveform.pulse_repetition_interval_s;
    const double doppler_hz = candidate.two_way_doppler_shift_hz;
    folded_hz = FoldIntoPrf(doppler_hz, prf_hz);
    mti_linear = std::max(MtiPowerResponse(doppler_hz, prf_hz), 1.0e-30);
    double best_bin = 0.0;
    for (std::uint32_t k = 0U; k < config.mtd_channel_count; ++k) {
      best_bin = std::max(best_bin, DftBinPower(doppler_hz, k, prf_hz, config.mtd_channel_count));
    }
    mtd_bin_linear = std::max(best_bin, 1.0e-30);
    coherent_linear = static_cast<double>(candidate.effective_pulse_count);
    // 中译：**对消器的噪声增益**（2026-10-23）——2 脉冲对消器对白噪声的功率增益为 2
    //   （+3.01 dB），必须与对目标的响应 H_mti 配对进入分母；此前只在验收旁路里算了 2，
    //   SINR 通道漏掉，导致信噪比高估约 3 dB。
    // 标识：对消器噪声增益（2026-10-23）。
    mti_noise_linear = kMtiNoiseFactor;
  }
  const double denominator = std::max(
      candidate.thermal_noise_power_w * noise_gain_linear * mti_noise_linear +
          (candidate.interference_power_w * mti_linear) / jamming_suppression_linear +
          (candidate.clutter_power_w * mti_linear) / clutter_suppression_linear,
      kLinearFloor);
  candidate.processed_single_pulse_sinr_linear = candidate.echo_power_w *
                                                 candidate.pulse_compression_gain *
                                                 target_gain_linear * mti_linear *
                                                 mtd_bin_linear * coherent_linear / denominator;
  candidate.folded_doppler_hz = folded_hz;
  candidate.mti_response_db = 10.0 * std::log10(mti_linear);
  candidate.mtd_bin_response_db = 10.0 * std::log10(mtd_bin_linear);
  candidate.coherent_integration_db = 10.0 * std::log10(coherent_linear);
  candidate.noise_total_gain_db =
      10.0 * std::log10(std::max(noise_gain_linear * mti_noise_linear, 1.0e-30));
  candidate.processed_single_pulse_sinr_db =
      10.0 * std::log10(std::max(candidate.processed_single_pulse_sinr_linear, kLinearFloor));
  if (!std::isfinite(candidate.echo_power_w) || candidate.echo_power_w < 0.0 ||
      !std::isfinite(candidate.processed_single_pulse_sinr_db)) {
    return false;
  }
  *result = candidate;
  return true;
}

}  // namespace radar
}  // namespace common
}  // namespace oneq
