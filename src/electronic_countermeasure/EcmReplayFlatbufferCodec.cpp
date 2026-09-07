#include "electronic_countermeasure/EcmReplayFlatbufferCodec.h"

#include <cstdint>
#include <vector>

#include "common/replay/ReplayFlatbufferCodecSupport.h"
#include "electronic_countermeasure/session/generated/ecm_replay_generated.h"
#include "flatbuffers/flatbuffers.h"

namespace electronic_countermeasure {
namespace session {
namespace {

// session_contract 规则 7：以整数存储的 enum 必须逐值校验，未知值原子拒绝。
bool IsKnownEcmInputMode(int32_t raw) { return raw >= 0 && raw <= 1; }

bool IsKnownEcmTechnique(int32_t raw) { return raw >= 0 && raw <= 3; }

bool IsKnownEcmCycleStatus(int32_t raw) { return raw >= 0 && raw <= 4; }

bool IsKnownRfScenePolarization(int32_t raw) { return raw >= 0 && raw <= 4; }

ecm::replay::Vec3 ToVec(const oneq::coordinate::EcefPositionM& value) {
  return {value.x_m, value.y_m, value.z_m};
}

ecm::replay::Vec3 ToVec(const oneq::coordinate::EcefVelocityMps& value) {
  return {value.x_mps, value.y_mps, value.z_mps};
}

flatbuffers::Offset<ecm::replay::AntennaPattern> BuildAntenna(
    flatbuffers::FlatBufferBuilder& builder,
    const oneq::electromagnetics::RfSceneAntennaPattern& antenna) {
  ecm::replay::UnitVector boresight(antenna.boresight_ecef.x,
                                    antenna.boresight_ecef.y,
                                    antenna.boresight_ecef.z);
  return ecm::replay::CreateAntennaPattern(
      builder, &boresight, antenna.peak_gain_dbi, antenna.half_power_beamwidth_deg,
      antenna.sidelobe_level_db, antenna.backlobe_level_db,
      antenna.cross_polarization_isolation_db);
}

void DecodeAntenna(const ecm::replay::AntennaPattern* encoded,
                   oneq::electromagnetics::RfSceneAntennaPattern* output) {
  if (encoded == nullptr || output == nullptr) {
    return;
  }
  if (encoded->boresight()) {
    output->boresight_ecef.x = encoded->boresight()->x();
    output->boresight_ecef.y = encoded->boresight()->y();
    output->boresight_ecef.z = encoded->boresight()->z();
  }
  output->peak_gain_dbi = encoded->peak_gain_dbi();
  output->half_power_beamwidth_deg = encoded->half_power_beamwidth_deg();
  output->sidelobe_level_db = encoded->sidelobe_level_db();
  output->backlobe_level_db = encoded->backlobe_level_db();
  output->cross_polarization_isolation_db = encoded->cross_polarization_isolation_db();
}

flatbuffers::Offset<ecm::replay::SensorObservationFrame> BuildSensorFrame(
    flatbuffers::FlatBufferBuilder& builder, const EcmSensorObservationFrame& frame) {
  std::vector<flatbuffers::Offset<ecm::replay::SensorObservation>> observations;
  for (const EcmSensorObservation& observation : frame.observations) {
    observations.push_back(ecm::replay::CreateSensorObservation(
        builder, observation.source_hypothesis_id,
        observation.estimated_center_frequency_hz, observation.estimated_bandwidth_hz,
        observation.estimated_pri_s, observation.estimated_pulse_width_s,
        observation.center_frequency_std_hz, observation.bandwidth_std_hz,
        observation.bearing_az_deg, observation.bearing_el_deg, observation.bearing_std_deg,
        observation.threat_score, observation.confidence));
  }
  return ecm::replay::CreateSensorObservationFrame(
      builder, frame.source_esr_batch_id, builder.CreateVector(observations));
}

EcmSensorObservationFrame DecodeSensorFrame(
    const ecm::replay::SensorObservationFrame* encoded) {
  EcmSensorObservationFrame frame;
  if (encoded == nullptr) {
    return frame;
  }
  frame.source_esr_batch_id = encoded->source_esr_batch_id();
  if (encoded->observations()) {
    for (const ecm::replay::SensorObservation* value : *encoded->observations()) {
      EcmSensorObservation observation;
      observation.source_hypothesis_id = value->source_hypothesis_id();
      observation.estimated_center_frequency_hz = value->estimated_center_frequency_hz();
      observation.estimated_bandwidth_hz = value->estimated_bandwidth_hz();
      observation.estimated_pri_s = value->estimated_pri_s();
      observation.estimated_pulse_width_s = value->estimated_pulse_width_s();
      observation.center_frequency_std_hz = value->center_frequency_std_hz();
      observation.bandwidth_std_hz = value->bandwidth_std_hz();
      observation.bearing_az_deg = value->bearing_az_deg();
      observation.bearing_el_deg = value->bearing_el_deg();
      observation.bearing_std_deg = value->bearing_std_deg();
      observation.threat_score = value->threat_score();
      observation.confidence = value->confidence();
      frame.observations.push_back(observation);
    }
  }
  return frame;
}

flatbuffers::Offset<ecm::replay::RfEmission> BuildEmission(
    flatbuffers::FlatBufferBuilder& builder,
    const oneq::electromagnetics::RfSceneEmission& emission) {
  ecm::replay::Vec3 position = ToVec(emission.position_ecef_m);
  ecm::replay::Vec3 velocity = ToVec(emission.velocity_ecef_mps);
  auto identity = ecm::replay::CreateRfEmissionIdentity(
      builder, emission.identity.sensor_entity_id,
      emission.identity.emission_id);
  const oneq::electromagnetics::RfWaveformSchedule& waveform = emission.waveform;
  auto encoded_waveform = ecm::replay::CreateRfWaveformSchedule(
      builder, static_cast<int32_t>(waveform.kind), waveform.activity_start_time_s,
      waveform.activity_duration_s, waveform.center_frequency_hz,
      waveform.occupied_bandwidth_hz, waveform.transmit_power_w, waveform.pulse_width_s,
      waveform.pulse_repetition_interval_s, waveform.first_pulse_time_s,
      waveform.pulse_count, waveform.pulse_jitter_fraction, waveform.timing_seed,
      waveform.timing_epoch, waveform.sweep_start_frequency_hz,
      waveform.sweep_stop_frequency_hz, waveform.sweep_period_s);
  return ecm::replay::CreateRfEmission(
      builder, identity, &position, &velocity,
      BuildAntenna(builder, emission.antenna), static_cast<int32_t>(emission.polarization),
      encoded_waveform);
}

oneq::electromagnetics::RfSceneEmission DecodeEmission(const ecm::replay::RfEmission* encoded) {
  oneq::electromagnetics::RfSceneEmission emission;
  if (encoded == nullptr) {
    return emission;
  }
  if (encoded->identity()) {
    emission.identity.sensor_entity_id = encoded->identity()->sensor_entity_id();
    emission.identity.emission_id = encoded->identity()->emission_id();
  }
  if (encoded->position_ecef_m()) {
    emission.position_ecef_m.x_m = encoded->position_ecef_m()->x();
    emission.position_ecef_m.y_m = encoded->position_ecef_m()->y();
    emission.position_ecef_m.z_m = encoded->position_ecef_m()->z();
  }
  if (encoded->velocity_ecef_mps()) {
    emission.velocity_ecef_mps.x_mps = encoded->velocity_ecef_mps()->x();
    emission.velocity_ecef_mps.y_mps = encoded->velocity_ecef_mps()->y();
    emission.velocity_ecef_mps.z_mps = encoded->velocity_ecef_mps()->z();
  }
  DecodeAntenna(encoded->antenna(), &emission.antenna);
  emission.polarization =
      static_cast<oneq::electromagnetics::RfScenePolarization>(encoded->polarization());
  if (encoded->waveform()) {
    const ecm::replay::RfWaveformSchedule* value = encoded->waveform();
    emission.waveform.kind =
        static_cast<oneq::electromagnetics::RfSceneWaveformKind>(value->kind());
    emission.waveform.activity_start_time_s = value->activity_start_time_s();
    emission.waveform.activity_duration_s = value->activity_duration_s();
    emission.waveform.center_frequency_hz = value->center_frequency_hz();
    emission.waveform.occupied_bandwidth_hz = value->occupied_bandwidth_hz();
    emission.waveform.transmit_power_w = value->transmit_power_w();
    emission.waveform.pulse_width_s = value->pulse_width_s();
    emission.waveform.pulse_repetition_interval_s = value->pulse_repetition_interval_s();
    emission.waveform.first_pulse_time_s = value->first_pulse_time_s();
    emission.waveform.pulse_count = value->pulse_count();
    emission.waveform.pulse_jitter_fraction = value->pulse_jitter_fraction();
    emission.waveform.timing_seed = value->timing_seed();
    emission.waveform.timing_epoch = value->timing_epoch();
    emission.waveform.sweep_start_frequency_hz = value->sweep_start_frequency_hz();
    emission.waveform.sweep_stop_frequency_hz = value->sweep_stop_frequency_hz();
    emission.waveform.sweep_period_s = value->sweep_period_s();
  }
  return emission;
}

}  // namespace

std::string EncodeEcmSessionConfig(const config::EcmSessionConfig& value) {
  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(ecm::replay::CreateEcmSessionConfig(
      builder, value.power_on, value.random_seed,
      value.channel_count,
      value.minimum_frequency_hz, value.maximum_frequency_hz,
      value.maximum_total_transmit_power_w, value.maximum_channel_transmit_power_w,
      value.thermal_capacity_j, value.cooling_power_w, value.spot_bandwidth_hz,
      value.barrage_bandwidth_hz, value.sweep_bandwidth_hz, value.sweep_segment_count,
      static_cast<int32_t>(value.default_technique),
      static_cast<ecm::replay::EcmDeceptionMode>(value.default_deception_mode),
      value.deception_rgpo_rate_m_per_s, value.deception_rgpo_max_range_m,
      value.deception_vgpo_rate_hz_per_s, value.deception_vgpo_max_doppler_hz,
      value.deception_hold_time_s, value.deception_power_scale,
      value.deception_max_active, value.deception_false_target_delay_s,
      value.deception_false_target_doppler_hz,
      value.deception_max_false_targets_per_threat));
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeEcmSessionConfig(const std::string& bytes, config::EcmSessionConfig* output) {
  if (output == nullptr) {
    return false;
  }
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (!verifier.VerifyBuffer<ecm::replay::EcmSessionConfig>()) {
    return false;
  }
  const ecm::replay::EcmSessionConfig* value =
      flatbuffers::GetRoot<ecm::replay::EcmSessionConfig>(bytes.data());
  config::EcmSessionConfig decoded;
  decoded.power_on = value->power_on();
  decoded.random_seed = value->random_seed();
  decoded.channel_count = value->channel_count();
  decoded.minimum_frequency_hz = value->minimum_frequency_hz();
  decoded.maximum_frequency_hz = value->maximum_frequency_hz();
  decoded.maximum_total_transmit_power_w = value->maximum_total_transmit_power_w();
  decoded.maximum_channel_transmit_power_w = value->maximum_channel_transmit_power_w();
  decoded.thermal_capacity_j = value->thermal_capacity_j();
  decoded.cooling_power_w = value->cooling_power_w();
  decoded.spot_bandwidth_hz = value->spot_bandwidth_hz();
  decoded.barrage_bandwidth_hz = value->barrage_bandwidth_hz();
  decoded.sweep_bandwidth_hz = value->sweep_bandwidth_hz();
  decoded.sweep_segment_count = value->sweep_segment_count();
  {
    const int32_t raw_technique = value->default_technique();
    if (!IsKnownEcmTechnique(raw_technique)) {
      return false;
    }
    decoded.default_technique = static_cast<EcmTechnique>(raw_technique);
  }
  {
    const int32_t raw_mode = value->default_deception_mode();
    if (raw_mode < 0 || raw_mode > 3) {
      return false;
    }
    decoded.default_deception_mode = static_cast<EcmDeceptionMode>(raw_mode);
  }
  decoded.deception_rgpo_rate_m_per_s = value->deception_rgpo_rate_m_per_s();
  decoded.deception_rgpo_max_range_m = value->deception_rgpo_max_range_m();
  decoded.deception_vgpo_rate_hz_per_s = value->deception_vgpo_rate_hz_per_s();
  decoded.deception_vgpo_max_doppler_hz = value->deception_vgpo_max_doppler_hz();
  decoded.deception_hold_time_s = value->deception_hold_time_s();
  decoded.deception_power_scale = value->deception_power_scale();
  decoded.deception_max_active = value->deception_max_active();
  decoded.deception_false_target_delay_s = value->deception_false_target_delay_s();
  decoded.deception_false_target_doppler_hz = value->deception_false_target_doppler_hz();
  decoded.deception_max_false_targets_per_threat =
      value->deception_max_false_targets_per_threat();
  *output = decoded;
  return true;
}

std::string EncodeEcmRuntimeConfigPatch(const config::EcmRuntimeConfigPatch& value) {
  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(ecm::replay::CreateEcmRuntimeConfigPatch(
      builder, value.has_power_on, value.power_on,
      value.has_maximum_total_transmit_power_w, value.maximum_total_transmit_power_w,
      value.has_default_technique, static_cast<int32_t>(value.default_technique),
      value.has_default_deception_mode,
      static_cast<ecm::replay::EcmDeceptionMode>(value.default_deception_mode)));
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeEcmRuntimeConfigPatch(const std::string& bytes,
                                 config::EcmRuntimeConfigPatch* output) {
  if (output == nullptr) {
    return false;
  }
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (!verifier.VerifyBuffer<ecm::replay::EcmRuntimeConfigPatch>()) {
    return false;
  }
  const ecm::replay::EcmRuntimeConfigPatch* value =
      flatbuffers::GetRoot<ecm::replay::EcmRuntimeConfigPatch>(bytes.data());
  config::EcmRuntimeConfigPatch decoded;
  decoded.has_power_on = value->has_power_on();
  decoded.power_on = value->power_on();
  decoded.has_maximum_total_transmit_power_w = value->has_maximum_total_transmit_power_w();
  decoded.maximum_total_transmit_power_w = value->maximum_total_transmit_power_w();
  decoded.has_default_technique = value->has_default_technique();
  {
    const int32_t raw_technique = value->default_technique();
    if (!IsKnownEcmTechnique(raw_technique)) {
      return false;
    }
    decoded.default_technique = static_cast<EcmTechnique>(raw_technique);
  }
  decoded.has_default_deception_mode = value->has_default_deception_mode();
  {
    const int32_t raw_mode = value->default_deception_mode();
    if (raw_mode < 0 || raw_mode > 3) {
      return false;
    }
    decoded.default_deception_mode = static_cast<EcmDeceptionMode>(raw_mode);
  }
  *output = decoded;
  return true;
}

std::string EncodeEcmCycleInput(const EcmCycleInput& value) {
  flatbuffers::FlatBufferBuilder builder;
  std::vector<flatbuffers::Offset<ecm::replay::TruthThreat>> truths;
  for (const EcmTruthThreat& truth : value.truth_threats) {
    truths.push_back(ecm::replay::CreateTruthThreat(
        builder, truth.truth_entity_id, truth.center_frequency_hz, truth.bandwidth_hz,
        truth.threat_score, truth.estimated_pri_s, truth.estimated_pulse_width_s));
  }
  ecm::replay::Vec3 position = ToVec(value.platform_position_ecef_m);
  ecm::replay::Vec3 velocity = ToVec(value.platform_velocity_ecef_mps);
  builder.Finish(
      ecm::replay::CreateEcmCycleInput(
          builder, value.cycle_index, value.cycle_start_time_s, value.dt_sec,
          static_cast<int32_t>(value.input_mode),
          &position, &velocity,
          BuildAntenna(builder, value.transmit_antenna),
          static_cast<int32_t>(value.transmit_polarization), value.has_sensor_observation_frame,
          BuildSensorFrame(builder, value.sensor_observation_frame), builder.CreateVector(truths)),
      ecm::replay::EcmCycleInputIdentifier());
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeEcmCycleInput(const std::string& bytes, EcmCycleInput* output) {
  if (output == nullptr) {
    return false;
  }
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (!verifier.VerifyBuffer<ecm::replay::EcmCycleInput>(ecm::replay::EcmCycleInputIdentifier())) {
    return false;
  }
  const ecm::replay::EcmCycleInput* value =
      flatbuffers::GetRoot<ecm::replay::EcmCycleInput>(bytes.data());
  EcmCycleInput decoded;
  decoded.cycle_index = value->cycle_index();
  decoded.cycle_start_time_s = value->cycle_start_time_s();
  decoded.dt_sec = value->dt_sec();
  {
    const int32_t raw_input_mode = value->input_mode();
    if (!IsKnownEcmInputMode(raw_input_mode)) {
      return false;
    }
    decoded.input_mode = static_cast<EcmInputMode>(raw_input_mode);
  }
  if (value->platform_position_ecef_m()) {
    decoded.platform_position_ecef_m.x_m = value->platform_position_ecef_m()->x();
    decoded.platform_position_ecef_m.y_m = value->platform_position_ecef_m()->y();
    decoded.platform_position_ecef_m.z_m = value->platform_position_ecef_m()->z();
  }
  if (value->platform_velocity_ecef_mps()) {
    decoded.platform_velocity_ecef_mps.x_mps = value->platform_velocity_ecef_mps()->x();
    decoded.platform_velocity_ecef_mps.y_mps = value->platform_velocity_ecef_mps()->y();
    decoded.platform_velocity_ecef_mps.z_mps = value->platform_velocity_ecef_mps()->z();
  }
  DecodeAntenna(value->transmit_antenna(), &decoded.transmit_antenna);
  {
    const int32_t raw_polarization = value->transmit_polarization();
    if (!IsKnownRfScenePolarization(raw_polarization)) {
      return false;
    }
    decoded.transmit_polarization =
        static_cast<oneq::electromagnetics::RfScenePolarization>(raw_polarization);
  }
  decoded.has_sensor_observation_frame = value->has_sensor_observation_frame();
  decoded.sensor_observation_frame = DecodeSensorFrame(value->sensor_observation_frame());
  if (value->truth_threats()) {
    for (const ecm::replay::TruthThreat* truth : *value->truth_threats()) {
      EcmTruthThreat decoded_truth;
      decoded_truth.truth_entity_id = truth->truth_entity_id();
      decoded_truth.center_frequency_hz = truth->center_frequency_hz();
      decoded_truth.bandwidth_hz = truth->bandwidth_hz();
      decoded_truth.threat_score = truth->threat_score();
      decoded_truth.estimated_pri_s = truth->estimated_pri_s();
      decoded_truth.estimated_pulse_width_s = truth->estimated_pulse_width_s();
      decoded.truth_threats.push_back(decoded_truth);
    }
  }
  *output = decoded;
  return true;
}

std::string EncodeEcmCycleResult(const EcmCycleResult& value) {
  flatbuffers::FlatBufferBuilder builder;
  std::vector<flatbuffers::Offset<ecm::replay::RfEmission>> emissions;
  for (const oneq::electromagnetics::RfSceneEmission& emission :
       value.emission_frame.emissions) {
    emissions.push_back(BuildEmission(builder, emission));
  }
  auto emission_frame = ecm::replay::CreateEcmEmissionFrame(
      builder, value.emission_frame.world_cycle_index,
      value.emission_frame.window_start_time_s, value.emission_frame.window_duration_s,
      builder.CreateVector(emissions));
  std::vector<flatbuffers::Offset<ecm::replay::ResourceDecision>> decisions;
  for (const EcmResourceDecision& decision : value.decisions) {
    decisions.push_back(ecm::replay::CreateResourceDecision(
        builder, decision.source_observation_id, decision.truth_entity_id,
        static_cast<int32_t>(decision.technique),
        static_cast<ecm::replay::EcmDeceptionMode>(decision.deception_mode),
        static_cast<ecm::replay::EcmDeceptionPhase>(decision.deception_phase),
        decision.channel_index, decision.allocated_power_w,
        builder.CreateString(decision.reason)));
  }
  builder.Finish(
      ecm::replay::CreateEcmCycleResult(
          builder, value.input_cycle_index, static_cast<int32_t>(value.status),
          static_cast<int32_t>(value.input_mode), value.truth_assisted,
          value.executed_this_cycle, value.used_glided_observation,
          value.observation_age_successful_ecm_cycles,
          value.source_esr_batch_id, value.thermal_energy_j, emission_frame,
          builder.CreateVector(decisions)),
      ecm::replay::EcmCycleInputIdentifier());
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeEcmCycleResult(const std::string& bytes, EcmCycleResult* output) {
  if (output == nullptr) {
    return false;
  }
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (!verifier.VerifyBuffer<ecm::replay::EcmCycleResult>(ecm::replay::EcmCycleInputIdentifier())) {
    return false;
  }
  const ecm::replay::EcmCycleResult* value =
      flatbuffers::GetRoot<ecm::replay::EcmCycleResult>(bytes.data());
  EcmCycleResult decoded;
  decoded.input_cycle_index = value->input_cycle_index();
  {
    const int32_t raw_status = value->status();
    if (!IsKnownEcmCycleStatus(raw_status)) {
      return false;
    }
    decoded.status = static_cast<EcmCycleStatus>(raw_status);
  }
  {
    const int32_t raw_input_mode = value->input_mode();
    if (!IsKnownEcmInputMode(raw_input_mode)) {
      return false;
    }
    decoded.input_mode = static_cast<EcmInputMode>(raw_input_mode);
  }
  decoded.truth_assisted = value->truth_assisted();
  decoded.executed_this_cycle = value->executed_this_cycle();
  decoded.used_glided_observation = value->used_glided_observation();
  decoded.observation_age_successful_ecm_cycles =
      value->observation_age_successful_ecm_cycles();
  decoded.source_esr_batch_id =
      value->source_esr_batch_id();
  decoded.thermal_energy_j = value->thermal_energy_j();
  if (value->emission_frame()) {
    decoded.emission_frame.world_cycle_index =
        value->emission_frame()->world_cycle_index();
    decoded.emission_frame.window_start_time_s =
        value->emission_frame()->window_start_time_s();
    decoded.emission_frame.window_duration_s =
        value->emission_frame()->window_duration_s();
    if (value->emission_frame()->emissions()) {
      for (const ecm::replay::RfEmission* emission :
           *value->emission_frame()->emissions()) {
        decoded.emission_frame.emissions.push_back(DecodeEmission(emission));
      }
    }
  }
  if (value->decisions()) {
    for (const ecm::replay::ResourceDecision* decision : *value->decisions()) {
      EcmResourceDecision decoded_decision;
      decoded_decision.source_observation_id = decision->source_observation_id();
      decoded_decision.truth_entity_id = decision->truth_entity_id();
      {
        const int32_t raw_technique = decision->technique();
        if (!IsKnownEcmTechnique(raw_technique)) {
          return false;
        }
        decoded_decision.technique = static_cast<EcmTechnique>(raw_technique);
      }
      {
        const int32_t raw_mode = decision->deception_mode();
        if (raw_mode < 0 || raw_mode > 3) {
          return false;
        }
        decoded_decision.deception_mode = static_cast<EcmDeceptionMode>(raw_mode);
      }
      {
        const int32_t raw_phase = decision->deception_phase();
        if (raw_phase < 0 || raw_phase > 3) {
          return false;
        }
        decoded_decision.deception_phase = static_cast<EcmDeceptionPhase>(raw_phase);
      }
      decoded_decision.channel_index = decision->channel_index();
      decoded_decision.allocated_power_w = decision->allocated_power_w();
      decoded_decision.reason = decision->reason() ? decision->reason()->str() : std::string();
      decoded.decisions.push_back(decoded_decision);
    }
  }
  *output = decoded;
  return true;
}

}  // namespace session
}  // namespace electronic_countermeasure
