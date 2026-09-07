#include "1q/electromagnetics/RfScene.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>
#include <utility>
#include "common/numerics/Constants.h"

namespace oneq {
namespace electromagnetics {
namespace {

using oneq::common::numerics::kPi;
using oneq::common::numerics::kLightSpeed;
constexpr double kLinearCircularLossDb = 3.010299956639812;
constexpr std::uint32_t kMaximumPulseCount = 1000000U;
constexpr std::size_t kSweepIntegrationSlices = 1024U;

bool IsFinite(double value) { return std::isfinite(value); }

bool IsFinitePosition(const coordinate::EcefPositionM& value) {
  return IsFinite(value.x_m) && IsFinite(value.y_m) && IsFinite(value.z_m);
}

bool IsFiniteVelocity(const coordinate::EcefVelocityMps& value) {
  return IsFinite(value.x_mps) && IsFinite(value.y_mps) && IsFinite(value.z_mps);
}

bool IsKnownPolarization(RfScenePolarization value) {
  switch (value) {
    case RfScenePolarization::kHorizontal:
    case RfScenePolarization::kVertical:
    case RfScenePolarization::kRightHandCircular:
    case RfScenePolarization::kLeftHandCircular:
    case RfScenePolarization::kUnpolarized:
    case RfScenePolarization::kFullPolarization:
      return true;
  }
  return false;
}

bool IsKnownWaveform(RfSceneWaveformKind value) {
  switch (value) {
    case RfSceneWaveformKind::kContinuous:
    case RfSceneWaveformKind::kPulseTrain:
    case RfSceneWaveformKind::kLinearSweep:
    case RfSceneWaveformKind::kBandLimitedNoise:
      return true;
  }
  return false;
}

bool TryNormalize(const RfSceneDirection& value, RfSceneDirection* result) {
  if (result == nullptr || !IsFinite(value.x) || !IsFinite(value.y) || !IsFinite(value.z)) {
    return false;
  }
  const double norm_squared = value.x * value.x + value.y * value.y + value.z * value.z;
  if (!IsFinite(norm_squared) || norm_squared <= 0.0) {
    return false;
  }
  const double scale = 1.0 / std::sqrt(norm_squared);
  RfSceneDirection candidate;
  candidate.x = value.x * scale;
  candidate.y = value.y * scale;
  candidate.z = value.z * scale;
  *result = candidate;
  return true;
}

bool IsValidPattern(const RfSceneAntennaPattern& pattern) {
  RfSceneDirection normalized;
  return TryNormalize(pattern.boresight_ecef, &normalized) && IsFinite(pattern.peak_gain_dbi) &&
         IsFinite(pattern.half_power_beamwidth_deg) && pattern.half_power_beamwidth_deg > 0.0 &&
         pattern.half_power_beamwidth_deg <= 180.0 && IsFinite(pattern.sidelobe_level_db) &&
         pattern.sidelobe_level_db <= -3.0 && IsFinite(pattern.backlobe_level_db) &&
         pattern.backlobe_level_db <= pattern.sidelobe_level_db &&
         IsFinite(pattern.cross_polarization_isolation_db) &&
         pattern.cross_polarization_isolation_db >= 0.0;
}

bool TryEvaluateGain(const RfSceneAntennaPattern& pattern, const RfSceneDirection& look,
                     double* gain_dbi) {
  if (gain_dbi == nullptr || !IsValidPattern(pattern)) {
    return false;
  }
  RfSceneDirection boresight;
  RfSceneDirection normalized_look;
  if (!TryNormalize(pattern.boresight_ecef, &boresight) || !TryNormalize(look, &normalized_look)) {
    return false;
  }
  const double dot = std::max(
      -1.0, std::min(1.0, boresight.x * normalized_look.x + boresight.y * normalized_look.y +
                              boresight.z * normalized_look.z));
  const double off_axis_deg = std::acos(dot) * 180.0 / kPi;
  double candidate = pattern.peak_gain_dbi;
  if (off_axis_deg <= 0.5 * pattern.half_power_beamwidth_deg) {
    const double normalized_offset = off_axis_deg / pattern.half_power_beamwidth_deg;
    candidate -= 12.0 * normalized_offset * normalized_offset;
  } else if (off_axis_deg < 90.0) {
    candidate += pattern.sidelobe_level_db;
  } else {
    candidate += pattern.backlobe_level_db;
  }
  if (!IsFinite(candidate)) {
    return false;
  }
  *gain_dbi = candidate;
  return true;
}

bool IsLinear(RfScenePolarization value) {
  return value == RfScenePolarization::kHorizontal || value == RfScenePolarization::kVertical;
}

bool IsCircular(RfScenePolarization value) {
  return value == RfScenePolarization::kRightHandCircular ||
         value == RfScenePolarization::kLeftHandCircular;
}

bool TryPolarizationLoss(const RfSceneEmission& emission, const RfSceneReceiverState& receiver,
                         double* loss_db) {
  if (loss_db == nullptr || !IsKnownPolarization(emission.polarization) ||
      !IsKnownPolarization(receiver.polarization)) {
    return false;
  }
  double candidate = 0.0;
  if (emission.polarization == RfScenePolarization::kUnpolarized ||
      receiver.polarization == RfScenePolarization::kUnpolarized ||
      (IsLinear(emission.polarization) && IsCircular(receiver.polarization)) ||
      (IsCircular(emission.polarization) && IsLinear(receiver.polarization))) {
    candidate = kLinearCircularLossDb;
  } else if (emission.polarization == RfScenePolarization::kFullPolarization ||
             receiver.polarization == RfScenePolarization::kFullPolarization) {
    // 全极化接收合并两条正交通道，对任意固定极化收满（0 dB）；
    // 单极化接收全极化发射只能收到正交两分量之一（一半功率）。
    candidate = (emission.polarization == RfScenePolarization::kFullPolarization &&
                 receiver.polarization != RfScenePolarization::kFullPolarization)
                    ? kLinearCircularLossDb
                    : 0.0;
  } else if (emission.polarization != receiver.polarization) {
    candidate = std::min(emission.antenna.cross_polarization_isolation_db,
                         receiver.antenna.cross_polarization_isolation_db);
  }
  *loss_db = candidate;
  return true;
}

bool TryFrequencyOverlap(double emission_center_hz, double emission_bandwidth_hz,
                         double receiver_center_hz, double receiver_bandwidth_hz,
                         double* fraction) {
  if (fraction == nullptr || !IsFinite(emission_center_hz) || emission_center_hz <= 0.0 ||
      !IsFinite(emission_bandwidth_hz) || emission_bandwidth_hz <= 0.0 ||
      !IsFinite(receiver_center_hz) || receiver_center_hz <= 0.0 ||
      !IsFinite(receiver_bandwidth_hz) || receiver_bandwidth_hz <= 0.0) {
    return false;
  }
  const double emission_low = emission_center_hz - 0.5 * emission_bandwidth_hz;
  const double emission_high = emission_center_hz + 0.5 * emission_bandwidth_hz;
  const double receiver_low = receiver_center_hz - 0.5 * receiver_bandwidth_hz;
  const double receiver_high = receiver_center_hz + 0.5 * receiver_bandwidth_hz;
  if (emission_low < 0.0 || receiver_low < 0.0) {
    return false;
  }
  const double overlap =
      std::max(0.0, std::min(emission_high, receiver_high) - std::max(emission_low, receiver_low));
  *fraction = std::max(0.0, std::min(1.0, overlap / emission_bandwidth_hz));
  return true;
}

double IntervalOverlap(double left_start, double left_end, double right_start, double right_end) {
  return std::max(0.0, std::min(left_end, right_end) - std::max(left_start, right_start));
}

std::uint64_t Mix64(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

double PulseJitterUnit(const RfWaveformSchedule& waveform, std::uint32_t pulse_index) {
  const std::uint64_t mixed = Mix64(waveform.timing_seed ^ Mix64(waveform.timing_epoch) ^
                                    Mix64(static_cast<std::uint64_t>(pulse_index)));
  const double unit = static_cast<double>(mixed >> 11U) * (1.0 / 9007199254740992.0);
  return 2.0 * unit - 1.0;
}

double PulseStartTime(const RfWaveformSchedule& waveform, std::uint32_t pulse_index) {
  const double jitter = pulse_index == 0U ? 0.0
                                          : PulseJitterUnit(waveform, pulse_index) *
                                                waveform.pulse_jitter_fraction *
                                                waveform.pulse_repetition_interval_s;
  return waveform.first_pulse_time_s +
         static_cast<double>(pulse_index) * waveform.pulse_repetition_interval_s + jitter;
}

bool IsValidWaveform(const RfWaveformSchedule& waveform) {
  if (!IsKnownWaveform(waveform.kind) || !IsFinite(waveform.activity_start_time_s) ||
      !IsFinite(waveform.activity_duration_s) || waveform.activity_duration_s <= 0.0 ||
      !IsFinite(waveform.transmit_power_w) || waveform.transmit_power_w < 0.0) {
    return false;
  }
  if (waveform.kind == RfSceneWaveformKind::kLinearSweep) {
    return IsFinite(waveform.sweep_start_frequency_hz) && waveform.sweep_start_frequency_hz > 0.0 &&
           IsFinite(waveform.sweep_stop_frequency_hz) && waveform.sweep_stop_frequency_hz > 0.0 &&
           waveform.sweep_start_frequency_hz != waveform.sweep_stop_frequency_hz &&
           IsFinite(waveform.occupied_bandwidth_hz) && waveform.occupied_bandwidth_hz > 0.0 &&
           IsFinite(waveform.sweep_period_s) && waveform.sweep_period_s > 0.0;
  }
  if (!IsFinite(waveform.center_frequency_hz) || waveform.center_frequency_hz <= 0.0 ||
      !IsFinite(waveform.occupied_bandwidth_hz) || waveform.occupied_bandwidth_hz <= 0.0) {
    return false;
  }
  if (waveform.kind != RfSceneWaveformKind::kPulseTrain) {
    return true;
  }
  if (!IsFinite(waveform.pulse_width_s) || waveform.pulse_width_s <= 0.0 ||
      !IsFinite(waveform.pulse_repetition_interval_s) ||
      waveform.pulse_repetition_interval_s <= 0.0 ||
      waveform.pulse_width_s >= 0.5 * waveform.pulse_repetition_interval_s ||
      waveform.pulse_count == 0U || waveform.pulse_count > kMaximumPulseCount ||
      !IsFinite(waveform.pulse_jitter_fraction) || waveform.pulse_jitter_fraction < 0.0 ||
      waveform.pulse_jitter_fraction > 0.25 || !IsFinite(waveform.first_pulse_time_s)) {
    return false;
  }
  const double max_jitter = waveform.pulse_jitter_fraction * waveform.pulse_repetition_interval_s;
  return waveform.pulse_width_s + 2.0 * max_jitter < waveform.pulse_repetition_interval_s;
}

double NominalCarrierHz(const RfWaveformSchedule& waveform) {
  return waveform.kind == RfSceneWaveformKind::kLinearSweep
             ? 0.5 * (waveform.sweep_start_frequency_hz + waveform.sweep_stop_frequency_hz)
             : waveform.center_frequency_hz;
}

bool IsValidEmission(const RfSceneEmission& emission) {
  return emission.identity.sensor_entity_id != 0U &&
         emission.identity.emission_id != 0U && IsFinitePosition(emission.position_ecef_m) &&
         IsFiniteVelocity(emission.velocity_ecef_mps) && IsValidPattern(emission.antenna) &&
         IsKnownPolarization(emission.polarization) && IsValidWaveform(emission.waveform);
}

bool IsValidReceiver(const RfSceneReceiverState& receiver) {
  if (receiver.sensor_entity_id == 0U ||
      !IsFinitePosition(receiver.position_ecef_m) ||
      !IsFiniteVelocity(receiver.velocity_ecef_mps) || !IsValidPattern(receiver.antenna) ||
      !IsKnownPolarization(receiver.polarization) || !IsFinite(receiver.window_start_time_s) ||
      !IsFinite(receiver.window_duration_s) || receiver.window_duration_s <= 0.0 ||
      !IsFinite(receiver.center_frequency_hz) || receiver.center_frequency_hz <= 0.0 ||
      !IsFinite(receiver.bandwidth_hz) || receiver.bandwidth_hz <= 0.0 ||
      !IsFinite(receiver.receiver_system_loss_db) || receiver.receiver_system_loss_db < 0.0 ||
      !IsFinite(receiver.minimum_far_field_range_m) || receiver.minimum_far_field_range_m <= 0.0) {
    return false;
  }
  std::set<std::uint32_t> paths;
  for (const RfCoSiteIsolationPath& path : receiver.co_site_paths) {
    if (path.transmitter_sensor_entity_id == 0U ||
        !IsFinite(path.isolation_db) || path.isolation_db < 0.0 ||
        !paths.insert(path.transmitter_sensor_entity_id).second) {
      return false;
    }
  }
  return true;
}

bool TryFindCoSiteIsolation(const RfSceneEmission& emission, const RfSceneReceiverState& receiver,
                            double* isolation_db) {
  if (isolation_db == nullptr) {
    return false;
  }
  for (const RfCoSiteIsolationPath& path : receiver.co_site_paths) {
    if (path.transmitter_sensor_entity_id == emission.identity.sensor_entity_id) {
      *isolation_db = path.isolation_db;
      return true;
    }
  }
  return false;
}

bool TryResolveOverlap(const RfWaveformSchedule& waveform, double propagation_delay_s,
                       double doppler_shift_hz, const RfSceneReceiverState& receiver,
                       double* time_fraction, double* frequency_fraction,
                       double* combined_fraction) {
  if (time_fraction == nullptr || frequency_fraction == nullptr || combined_fraction == nullptr) {
    return false;
  }
  const double receiver_start = receiver.window_start_time_s;
  const double receiver_end = receiver_start + receiver.window_duration_s;
  const double arrival_start = waveform.activity_start_time_s + propagation_delay_s;
  const double arrival_end = arrival_start + waveform.activity_duration_s;
  const double active_overlap =
      IntervalOverlap(arrival_start, arrival_end, receiver_start, receiver_end);
  *time_fraction = active_overlap / receiver.window_duration_s;
  *frequency_fraction = 0.0;
  *combined_fraction = 0.0;
  if (active_overlap <= 0.0) {
    return true;
  }

  if (waveform.kind == RfSceneWaveformKind::kPulseTrain) {
    double pulse_overlap_s = 0.0;
    for (std::uint32_t index = 0U; index < waveform.pulse_count; ++index) {
      const double pulse_start = PulseStartTime(waveform, index) + propagation_delay_s;
      pulse_overlap_s += IntervalOverlap(pulse_start, pulse_start + waveform.pulse_width_s,
                                         receiver_start, receiver_end);
    }
    double frequency = 0.0;
    if (!TryFrequencyOverlap(waveform.center_frequency_hz + doppler_shift_hz,
                             waveform.occupied_bandwidth_hz, receiver.center_frequency_hz,
                             receiver.bandwidth_hz, &frequency)) {
      return false;
    }
    *time_fraction = pulse_overlap_s / receiver.window_duration_s;
    *frequency_fraction = frequency;
    *combined_fraction = *time_fraction * frequency;
    return true;
  }

  if (waveform.kind != RfSceneWaveformKind::kLinearSweep) {
    double frequency = 0.0;
    if (!TryFrequencyOverlap(waveform.center_frequency_hz + doppler_shift_hz,
                             waveform.occupied_bandwidth_hz, receiver.center_frequency_hz,
                             receiver.bandwidth_hz, &frequency)) {
      return false;
    }
    *frequency_fraction = frequency;
    *combined_fraction = *time_fraction * frequency;
    return true;
  }

  const double overlap_start = std::max(arrival_start, receiver_start);
  const double overlap_end = std::min(arrival_end, receiver_end);
  const double slice_duration =
      (overlap_end - overlap_start) / static_cast<double>(kSweepIntegrationSlices);
  double weighted_frequency_time = 0.0;
  for (std::size_t index = 0U; index < kSweepIntegrationSlices; ++index) {
    const double arrival_time = overlap_start + (static_cast<double>(index) + 0.5) * slice_duration;
    const double emitted_elapsed =
        arrival_time - propagation_delay_s - waveform.activity_start_time_s;
    double phase = std::fmod(emitted_elapsed, waveform.sweep_period_s) / waveform.sweep_period_s;
    if (phase < 0.0) {
      phase += 1.0;
    }
    const double center =
        waveform.sweep_start_frequency_hz +
        phase * (waveform.sweep_stop_frequency_hz - waveform.sweep_start_frequency_hz) +
        doppler_shift_hz;
    double frequency = 0.0;
    if (!TryFrequencyOverlap(center, waveform.occupied_bandwidth_hz, receiver.center_frequency_hz,
                             receiver.bandwidth_hz, &frequency)) {
      return false;
    }
    weighted_frequency_time += frequency * slice_duration;
  }
  *frequency_fraction = weighted_frequency_time / active_overlap;
  *combined_fraction = weighted_frequency_time / receiver.window_duration_s;
  return true;
}

}  // namespace

bool TryCreateRfContinuousWaveform(double start_time_s, double duration_s,
                                   double center_frequency_hz, double bandwidth_hz,
                                   double transmit_power_w, RfWaveformSchedule* waveform) {
  if (waveform == nullptr) {
    return false;
  }
  RfWaveformSchedule candidate;
  candidate.kind = RfSceneWaveformKind::kContinuous;
  candidate.activity_start_time_s = start_time_s;
  candidate.activity_duration_s = duration_s;
  candidate.center_frequency_hz = center_frequency_hz;
  candidate.occupied_bandwidth_hz = bandwidth_hz;
  candidate.transmit_power_w = transmit_power_w;
  if (!IsValidWaveform(candidate)) {
    return false;
  }
  *waveform = candidate;
  return true;
}

bool TryCreateRfNoiseWaveform(double start_time_s, double duration_s, double center_frequency_hz,
                              double bandwidth_hz, double transmit_power_w,
                              RfWaveformSchedule* waveform) {
  if (waveform == nullptr) {
    return false;
  }
  RfWaveformSchedule candidate;
  if (!TryCreateRfContinuousWaveform(start_time_s, duration_s, center_frequency_hz, bandwidth_hz,
                                     transmit_power_w, &candidate)) {
    return false;
  }
  candidate.kind = RfSceneWaveformKind::kBandLimitedNoise;
  *waveform = candidate;
  return true;
}

bool TryCreateRfPulseTrainWaveform(double first_pulse_time_s, double center_frequency_hz,
                                   double bandwidth_hz, double peak_power_w, double pulse_width_s,
                                   double pulse_repetition_interval_s, std::uint32_t pulse_count,
                                   double jitter_fraction, std::uint64_t timing_seed,
                                   std::uint64_t timing_epoch, RfWaveformSchedule* waveform) {
  if (waveform == nullptr || !IsFinite(jitter_fraction) || !IsFinite(pulse_width_s) ||
      !IsFinite(pulse_repetition_interval_s)) {
    return false;
  }
  const double jitter_margin = std::max(0.0, jitter_fraction) * pulse_repetition_interval_s;
  RfWaveformSchedule candidate;
  candidate.kind = RfSceneWaveformKind::kPulseTrain;
  candidate.activity_start_time_s = first_pulse_time_s;
  candidate.activity_duration_s =
      pulse_count == 0U ? 0.0
                        : static_cast<double>(pulse_count - 1U) * pulse_repetition_interval_s +
                              pulse_width_s + jitter_margin;
  candidate.center_frequency_hz = center_frequency_hz;
  candidate.occupied_bandwidth_hz = bandwidth_hz;
  candidate.transmit_power_w = peak_power_w;
  candidate.pulse_width_s = pulse_width_s;
  candidate.pulse_repetition_interval_s = pulse_repetition_interval_s;
  candidate.first_pulse_time_s = first_pulse_time_s;
  candidate.pulse_count = pulse_count;
  candidate.pulse_jitter_fraction = jitter_fraction;
  candidate.timing_seed = timing_seed;
  candidate.timing_epoch = timing_epoch;
  if (!IsValidWaveform(candidate)) {
    return false;
  }
  *waveform = candidate;
  return true;
}

bool TryCreateRfLinearSweepWaveform(double start_time_s, double duration_s,
                                    double start_frequency_hz, double stop_frequency_hz,
                                    double instantaneous_bandwidth_hz, double transmit_power_w,
                                    double sweep_period_s, RfWaveformSchedule* waveform) {
  if (waveform == nullptr) {
    return false;
  }
  RfWaveformSchedule candidate;
  candidate.kind = RfSceneWaveformKind::kLinearSweep;
  candidate.activity_start_time_s = start_time_s;
  candidate.activity_duration_s = duration_s;
  candidate.occupied_bandwidth_hz = instantaneous_bandwidth_hz;
  candidate.transmit_power_w = transmit_power_w;
  candidate.sweep_start_frequency_hz = start_frequency_hz;
  candidate.sweep_stop_frequency_hz = stop_frequency_hz;
  candidate.sweep_period_s = sweep_period_s;
  if (!IsValidWaveform(candidate)) {
    return false;
  }
  *waveform = candidate;
  return true;
}

bool TryEvaluateRfArrivalActivity(const RfWaveformSchedule& waveform,
                                  double propagation_delay_s, double doppler_shift_hz,
                                  double arrival_time_s, bool* active,
                                  double* arrival_center_frequency_hz) {
  if (active == nullptr || arrival_center_frequency_hz == nullptr ||
      !IsValidWaveform(waveform) || !IsFinite(propagation_delay_s) ||
      propagation_delay_s < 0.0 || !IsFinite(doppler_shift_hz) ||
      !IsFinite(arrival_time_s)) {
    return false;
  }

  bool candidate_active = false;
  double candidate_frequency_hz = 0.0;
  const double emission_time_s = arrival_time_s - propagation_delay_s;
  const double activity_end_s =
      waveform.activity_start_time_s + waveform.activity_duration_s;
  if (emission_time_s >= waveform.activity_start_time_s && emission_time_s < activity_end_s) {
    if (waveform.kind == RfSceneWaveformKind::kPulseTrain) {
      std::uint32_t low = 0U;
      std::uint32_t high = waveform.pulse_count;
      while (low < high) {
        const std::uint32_t middle = low + (high - low) / 2U;
        if (PulseStartTime(waveform, middle) <= emission_time_s) {
          low = middle + 1U;
        } else {
          high = middle;
        }
      }
      if (low > 0U) {
        const double pulse_start_s = PulseStartTime(waveform, low - 1U);
        candidate_active = emission_time_s >= pulse_start_s &&
                           emission_time_s < pulse_start_s + waveform.pulse_width_s;
      }
      candidate_frequency_hz = waveform.center_frequency_hz + doppler_shift_hz;
    } else if (waveform.kind == RfSceneWaveformKind::kLinearSweep) {
      double phase =
          std::fmod(emission_time_s - waveform.activity_start_time_s, waveform.sweep_period_s) /
          waveform.sweep_period_s;
      if (phase < 0.0) {
        phase += 1.0;
      }
      candidate_active = true;
      candidate_frequency_hz =
          waveform.sweep_start_frequency_hz +
          phase * (waveform.sweep_stop_frequency_hz - waveform.sweep_start_frequency_hz) +
          doppler_shift_hz;
    } else {
      candidate_active = true;
      candidate_frequency_hz = waveform.center_frequency_hz + doppler_shift_hz;
    }
  }
  if (candidate_active && (!IsFinite(candidate_frequency_hz) || candidate_frequency_hz <= 0.0)) {
    return false;
  }
  *active = candidate_active;
  *arrival_center_frequency_hz = candidate_active ? candidate_frequency_hz : 0.0;
  return true;
}

bool TryResolveRfPulseStartTime(const RfWaveformSchedule& waveform,
                                std::uint32_t pulse_index, double* pulse_start_time_s) {
  if (pulse_start_time_s == nullptr || !IsValidWaveform(waveform) ||
      waveform.kind != RfSceneWaveformKind::kPulseTrain ||
      pulse_index >= waveform.pulse_count) {
    return false;
  }
  const double candidate = PulseStartTime(waveform, pulse_index);
  if (!IsFinite(candidate)) {
    return false;
  }
  *pulse_start_time_s = candidate;
  return true;
}

bool TryValidateRfSceneFrame(const RfSceneFrame& scene) {
  if (scene.world_cycle_index == 0U || !IsFinite(scene.window_start_time_s) ||
      !IsFinite(scene.window_duration_s) || scene.window_duration_s <= 0.0) {
    return false;
  }
  typedef std::pair<std::uint32_t, std::uint64_t> IdentityKey;
  std::set<IdentityKey> emission_identities;
  for (const RfSceneEmission& emission : scene.emissions) {
    const IdentityKey identity =
        std::make_pair(emission.identity.sensor_entity_id,
                       emission.identity.emission_id);
    if (!IsValidEmission(emission) ||
        !emission_identities.insert(identity).second) {
      return false;
    }
  }
  return true;
}

bool RfFrameMatchesCycleWindow(const RfSceneFrame& frame, std::uint64_t cycle_index,
                               double cycle_start_time_s, double window_duration_s) {
  return frame.world_cycle_index == cycle_index &&
         frame.window_start_time_s == cycle_start_time_s &&
         frame.window_duration_s == window_duration_s;
}

bool TryEvaluateRfIncidentLink(const RfSceneEmission& emission,
                               const RfSceneReceiverState& receiver,
                               const RfIncidentLinkConfig& config, RfIncidentLinkResult* result) {
  if (result == nullptr || !IsValidEmission(emission) || !IsValidReceiver(receiver) ||
      !IsFinite(config.additional_propagation_loss_db) ||
      config.additional_propagation_loss_db < 0.0) {
    return false;
  }

  double co_site_isolation_db = 0.0;
  const bool is_co_site = TryFindCoSiteIsolation(emission, receiver, &co_site_isolation_db);
  RfSceneDirection emitter_to_receiver;
  emitter_to_receiver.x = receiver.position_ecef_m.x_m - emission.position_ecef_m.x_m;
  emitter_to_receiver.y = receiver.position_ecef_m.y_m - emission.position_ecef_m.y_m;
  emitter_to_receiver.z = receiver.position_ecef_m.z_m - emission.position_ecef_m.z_m;
  const double distance_squared = emitter_to_receiver.x * emitter_to_receiver.x +
                                  emitter_to_receiver.y * emitter_to_receiver.y +
                                  emitter_to_receiver.z * emitter_to_receiver.z;
  if (!IsFinite(distance_squared) || distance_squared < 0.0) {
    return false;
  }
  const double path_length_m = std::sqrt(distance_squared);
  if (!is_co_site && path_length_m < receiver.minimum_far_field_range_m) {
    return false;
  }

  RfIncidentLinkResult candidate;
  candidate.identity = emission.identity;
  candidate.emission_waveform = emission.waveform;
  candidate.receiver_sensor_entity_id = receiver.sensor_entity_id;
  candidate.is_co_site = is_co_site;
  candidate.path_length_m = path_length_m;
  candidate.propagation_delay_s = is_co_site ? 0.0 : path_length_m / kLightSpeed;
  candidate.additional_propagation_loss_db = config.additional_propagation_loss_db;
  candidate.receiver_system_loss_db = receiver.receiver_system_loss_db;

  double total_loss_db = receiver.receiver_system_loss_db;
  const double nominal_carrier_hz = NominalCarrierHz(emission.waveform);
  if (is_co_site) {
    candidate.co_site_isolation_db = co_site_isolation_db;
    total_loss_db += candidate.co_site_isolation_db;
  } else {
    RfSceneDirection direction;
    if (!TryNormalize(emitter_to_receiver, &direction)) {
      return false;
    }
    RfSceneDirection receiver_to_emitter;
    receiver_to_emitter.x = -direction.x;
    receiver_to_emitter.y = -direction.y;
    receiver_to_emitter.z = -direction.z;
    if (!TryEvaluateGain(emission.antenna, direction, &candidate.transmit_antenna_gain_dbi) ||
        !TryEvaluateGain(receiver.antenna, receiver_to_emitter,
                         &candidate.receive_antenna_gain_dbi) ||
        !TryPolarizationLoss(emission, receiver, &candidate.polarization_mismatch_loss_db)) {
      return false;
    }
    const double relative_velocity_x =
        receiver.velocity_ecef_mps.x_mps - emission.velocity_ecef_mps.x_mps;
    const double relative_velocity_y =
        receiver.velocity_ecef_mps.y_mps - emission.velocity_ecef_mps.y_mps;
    const double relative_velocity_z =
        receiver.velocity_ecef_mps.z_mps - emission.velocity_ecef_mps.z_mps;
    const double separation_rate = relative_velocity_x * direction.x +
                                   relative_velocity_y * direction.y +
                                   relative_velocity_z * direction.z;
    candidate.doppler_shift_hz = -nominal_carrier_hz * separation_rate / kLightSpeed;
    const double wavelength_m = kLightSpeed / nominal_carrier_hz;
    const double path_ratio = 4.0 * kPi * path_length_m / wavelength_m;
    if (!IsFinite(path_ratio) || path_ratio <= 0.0) {
      return false;
    }
    candidate.free_space_loss_db = 20.0 * std::log10(path_ratio);
    total_loss_db += candidate.free_space_loss_db + candidate.additional_propagation_loss_db +
                     candidate.polarization_mismatch_loss_db - candidate.transmit_antenna_gain_dbi -
                     candidate.receive_antenna_gain_dbi;
  }

  const double linear_path_gain = std::pow(10.0, -total_loss_db / 10.0);
  candidate.received_power_before_overlap_w = emission.waveform.transmit_power_w * linear_path_gain;
  candidate.arrival_start_time_s =
      emission.waveform.activity_start_time_s + candidate.propagation_delay_s;
  candidate.arrival_end_time_s =
      candidate.arrival_start_time_s + emission.waveform.activity_duration_s;
  double combined_fraction = 0.0;
  if (!TryResolveOverlap(emission.waveform, candidate.propagation_delay_s,
                         candidate.doppler_shift_hz, receiver, &candidate.time_overlap_fraction,
                         &candidate.frequency_overlap_fraction, &combined_fraction)) {
    return false;
  }
  candidate.received_power_w = candidate.received_power_before_overlap_w * combined_fraction;
  candidate.received_power_spectral_density_w_per_hz =
      candidate.received_power_before_overlap_w / emission.waveform.occupied_bandwidth_hz;
  if (!IsFinite(candidate.received_power_before_overlap_w) ||
      !IsFinite(candidate.received_power_w) || candidate.received_power_w < 0.0 ||
      !IsFinite(candidate.received_power_spectral_density_w_per_hz)) {
    return false;
  }
  *result = std::move(candidate);
  return true;
}

bool TryAggregateRfIncidentPower(const std::vector<RfIncidentLinkResult>& links,
                                 double* total_received_power_w) {
  if (total_received_power_w == nullptr) {
    return false;
  }
  typedef std::pair<std::uint32_t, std::uint64_t> IdentityKey;
  std::vector<std::pair<IdentityKey, double>> ordered;
  ordered.reserve(links.size());
  std::uint32_t receiver_sensor_entity_id = 0U;
  bool has_receiver = false;
  for (const RfIncidentLinkResult& link : links) {
    if (!IsFinite(link.received_power_w) || link.received_power_w < 0.0 ||
        (has_receiver && (receiver_sensor_entity_id != link.receiver_sensor_entity_id))) {
      return false;
    }
    receiver_sensor_entity_id = link.receiver_sensor_entity_id;
    has_receiver = true;
    ordered.push_back(
        std::make_pair(std::make_pair(link.identity.sensor_entity_id,
                                      link.identity.emission_id),
                       link.received_power_w));
  }
  std::sort(ordered.begin(), ordered.end(),
            [](const std::pair<IdentityKey, double>& left,
               const std::pair<IdentityKey, double>& right) { return left.first < right.first; });
  long double sum = 0.0L;
  for (std::size_t index = 0U; index < ordered.size(); ++index) {
    if (index > 0U && ordered[index - 1U].first == ordered[index].first) {
      return false;
    }
    sum += static_cast<long double>(ordered[index].second);
  }
  const double candidate = static_cast<double>(sum);
  if (!IsFinite(candidate)) {
    return false;
  }
  *total_received_power_w = candidate;
  return true;
}

}  // namespace electromagnetics
}  // namespace oneq
