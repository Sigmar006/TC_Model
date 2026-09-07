#include "airborne_radar/session/ArReplayFlatbufferCodec.h"

#include <cstdint>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "1q/foundation/validation_types.h"
#include "airborne_radar/session/generated/airborne_radar_replay_generated.h"
#include "airborne_radar/session/generated/airborne_radar_session_replay_generated.h"
#include "common/replay/ReplayFlatbufferCodecSupport.h"

namespace airborne_radar {
namespace session {
namespace {

namespace fb = oneq::replay::airborne_radar::fb;
namespace session_fb = oneq::replay::airborne_radar::session::fb;

std::size_t CountTracksByStatus(const session::TrackStateSnapshotList& tracks,
                                session::TrackStatus status) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    if (tracks[i].status == status) {
      ++count;
    }
  }
  return count;
}

flatbuffers::Offset<fb::DecisionTrackStateSnapshot> EncodeTrackStateSnapshot(
    flatbuffers::FlatBufferBuilder* builder, const session::TrackStateSnapshot& value) {
  return fb::CreateDecisionTrackStateSnapshot(
      *builder, value.association_key, value.external_target_id, static_cast<int>(value.status),
      value.position_x, value.position_y, value.position_z, value.velocity_x, value.velocity_y,
      value.velocity_z, value.speed, value.acceleration_x, value.acceleration_y,
      value.acceleration_z, value.acceleration, value.rcs, value.hit_count, value.miss_count,
      builder->CreateString(value.target_type), value.target_probability,
      builder->CreateString(value.target_name), value.estimation_uncertainty_trace);
}

flatbuffers::Offset<fb::TrackStateSnapshot> EncodeTrackSnapshot(
    flatbuffers::FlatBufferBuilder* builder, const session::TrackStateSnapshot& value) {
  return fb::CreateTrackStateSnapshot(*builder, EncodeTrackStateSnapshot(builder, value));
}

session::TrackStateSnapshot DecodeTrackStateSnapshot(const fb::DecisionTrackStateSnapshot* value) {
  session::TrackStateSnapshot result;
  if (value != nullptr) {
    result.association_key = value->association_key();
    result.external_target_id = value->external_target_id();
    result.status = static_cast<session::TrackStatus>(value->status());
    result.position_x = value->position_x();
    result.position_y = value->position_y();
    result.position_z = value->position_z();
    result.velocity_x = value->velocity_x();
    result.velocity_y = value->velocity_y();
    result.velocity_z = value->velocity_z();
    result.speed = value->speed();
    result.acceleration_x = value->acceleration_x();
    result.acceleration_y = value->acceleration_y();
    result.acceleration_z = value->acceleration_z();
    result.acceleration = value->acceleration();
    result.rcs = value->rcs();
    result.hit_count = value->hit_count();
    result.miss_count = value->miss_count();
    if (value->target_type() != nullptr) {
      result.target_type = value->target_type()->str();
    }
    result.target_probability = value->target_probability();
    if (value->target_name() != nullptr) {
      result.target_name = value->target_name()->str();
    }
    result.estimation_uncertainty_trace = value->estimation_uncertainty_trace();
  }
  return result;
}

session::TrackStateSnapshot DecodeTrackSnapshot(const fb::TrackStateSnapshot* value) {
  session::TrackStateSnapshot result;
  if (value != nullptr) {
    result = DecodeTrackStateSnapshot(value->state());
  }
  return result;
}

flatbuffers::Offset<fb::TrackOutputFrame> EncodeTrackOutputFrame(
    flatbuffers::FlatBufferBuilder* builder, const session::TrackOutputFrame& value) {
  std::vector<flatbuffers::Offset<fb::TrackStateSnapshot>> track_offsets;
  track_offsets.reserve(value.tracks.size());
  for (std::size_t i = 0; i < value.tracks.size(); ++i) {
    track_offsets.push_back(EncodeTrackSnapshot(builder, value.tracks[i]));
  }
  const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<fb::TrackStateSnapshot>>>
      tracks_vec = builder->CreateVector(track_offsets);
  return fb::CreateTrackOutputFrame(
      *builder, value.cycle_index, static_cast<std::uint64_t>(value.tracks.size()), value.batch_id,
      static_cast<std::uint64_t>(
          CountTracksByStatus(value.tracks, session::TrackStatus::kConfirmed)),
      CountTracksByStatus(value.tracks, session::TrackStatus::kLost) > 0U, tracks_vec);
}

session::TrackOutputFrame DecodeTrackOutputFrame(const fb::TrackOutputFrame* value) {
  session::TrackOutputFrame result;
  if (value != nullptr) {
    result.cycle_index = value->cycle_index();
    result.batch_id = value->batch_id();
    const flatbuffers::Vector<flatbuffers::Offset<fb::TrackStateSnapshot>>* tracks =
        value->tracks();
    if (tracks != nullptr) {
      result.tracks.reserve(tracks->size());
      for (flatbuffers::uoffset_t i = 0; i < tracks->size(); ++i) {
        result.tracks.push_back(DecodeTrackSnapshot(tracks->Get(i)));
      }
    }
  }
  return result;
}

flatbuffers::Offset<fb::ArControlProfile> EncodeArControlProfile(
    flatbuffers::FlatBufferBuilder* builder, const session::ArControlProfile& value) {
  return fb::CreateArControlProfile(
      *builder, value.version, value.enable_lpi_power_control, value.lpi_power_scale,
      value.enable_lpi_beamforming, value.lpi_dwell_scale, value.enable_agility_frequency,
      value.agility_frequency_hop_phase, value.enable_sidelobe_canceller,
      value.enable_adaptive_beamforming, value.enable_eccm_rejitter, value.eccm_burnthrough_gain,
      value.enable_anti_rgpo_leading_edge, value.enable_anti_vgpo_acceleration_bound,
      value.enable_anti_false_target_discrimination);
}

session::ArControlProfile DecodeArControlProfile(const fb::ArControlProfile* value) {
  session::ArControlProfile result;
  if (value != nullptr) {
    result.version = value->version();
    result.enable_lpi_power_control = value->enable_lpi_power_control();
    result.lpi_power_scale = value->lpi_power_scale();
    result.enable_lpi_beamforming = value->enable_lpi_beamforming();
    result.lpi_dwell_scale = value->lpi_dwell_scale();
    result.enable_agility_frequency = value->enable_agility_frequency();
    result.agility_frequency_hop_phase = value->agility_frequency_hop_phase();
    result.enable_sidelobe_canceller = value->enable_sidelobe_canceller();
    result.enable_adaptive_beamforming = value->enable_adaptive_beamforming();
    result.enable_eccm_rejitter = value->enable_eccm_rejitter();
    result.eccm_burnthrough_gain = value->eccm_burnthrough_gain();
    result.enable_anti_rgpo_leading_edge = value->enable_anti_rgpo_leading_edge();
    result.enable_anti_vgpo_acceleration_bound = value->enable_anti_vgpo_acceleration_bound();
    result.enable_anti_false_target_discrimination =
        value->enable_anti_false_target_discrimination();
  }
  return result;
}

flatbuffers::Offset<fb::ArInterferenceObservation> EncodeArInterferenceObservation(
    flatbuffers::FlatBufferBuilder* builder, const session::ArInterferenceObservation& value) {
  return fb::CreateArInterferenceObservation(
      *builder, value.observation_id, value.estimated_bearing_azimuth_deg,
      value.estimated_bearing_elevation_deg, value.estimated_off_boresight_deg,
      value.estimated_center_frequency_hz, value.estimated_bandwidth_hz,
      static_cast<int>(value.estimated_waveform_kind), value.jammer_to_noise_db,
      value.bearing_standard_deviation_deg, value.frequency_standard_deviation_hz,
      value.bandwidth_standard_deviation_hz, static_cast<int>(value.deception_class),
      value.coherent_emission_count, value.estimated_slant_range_m, value.has_local_bearings,
      value.estimated_bearing_azimuth_local_deg, value.estimated_bearing_elevation_local_deg,
      value.estimated_range_rate_mps, value.estimated_carrier_offset_hz,
      value.estimated_first_pulse_delay_s);
}

flatbuffers::Offset<fb::TacticalProposal> EncodeTacticalProposal(
    flatbuffers::FlatBufferBuilder* builder, const session::TacticalProposal& value) {
  const session::ControlDirective& directive = value.directive;
  return fb::CreateTacticalProposal(
      *builder,
      fb::CreateControlDirective(*builder, static_cast<int>(directive.type),
                                 static_cast<int>(directive.source), directive.has_requested_value,
                                 directive.requested_value),
      value.priority, builder->CreateString(value.rationale));
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<fb::TacticalProposal>>>
EncodeTacticalProposals(flatbuffers::FlatBufferBuilder* builder,
                        const std::vector<session::TacticalProposal>& values) {
  std::vector<flatbuffers::Offset<fb::TacticalProposal>> offsets;
  offsets.reserve(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    offsets.push_back(EncodeTacticalProposal(builder, values[i]));
  }
  return builder->CreateVector(offsets);
}

bool IsKnownRfSceneWaveformKind(int raw_value) {
  using oneq::electromagnetics::RfSceneWaveformKind;
  return raw_value == static_cast<int>(RfSceneWaveformKind::kContinuous) ||
         raw_value == static_cast<int>(RfSceneWaveformKind::kPulseTrain) ||
         raw_value == static_cast<int>(RfSceneWaveformKind::kLinearSweep) ||
         raw_value == static_cast<int>(RfSceneWaveformKind::kBandLimitedNoise);
}

// 控制类枚举的 decode 期 range 校验，与 IsKnownRfSceneWaveformKind / DeceptionClass
// 同属 fail-closed 模式：越界或哨兵值（kCount）必须在 decode 时原子拒绝，
// 不改写输出、不静默穿透到消费端。kCount 是编译期哨兵，不可作为真实意图。
bool IsKnownControlDirectiveType(int raw_value) {
  return raw_value > static_cast<int>(session::ControlDirectiveType::NONE) &&
         raw_value < static_cast<int>(session::ControlDirectiveType::kCount);
}

bool IsKnownControlDirectiveSource(int raw_value) {
  using S = session::ControlDirectiveSource;
  return raw_value == static_cast<int>(S::UNKNOWN) ||
         raw_value == static_cast<int>(S::THREAT_ASSESSMENT) ||
         raw_value == static_cast<int>(S::EMISSION_CONTROL) ||
         raw_value == static_cast<int>(S::SURVIVABILITY);
}

bool IsKnownDecisionControlSource(int raw_value) {
  using S = session::DecisionControlSource;
  return raw_value == static_cast<int>(S::kNone) ||
         raw_value == static_cast<int>(S::kInternal) ||
         raw_value == static_cast<int>(S::kExternal);
}

// 统一问题列表（规则 14）：decode 期对 severity/phase 做范围校验（fail-closed）。
bool IsValidIssueSeverity(std::int32_t value) {
  return value >= static_cast<std::int32_t>(session::ArIssueSeverity::kInfo) &&
         value <= static_cast<std::int32_t>(session::ArIssueSeverity::kError);
}

bool IsValidIssuePhase(std::int32_t value) {
  return value >= static_cast<std::int32_t>(session::ArIssuePhase::kInputValidation) &&
         value <= static_cast<std::int32_t>(session::ArIssuePhase::kOutputContract);
}

bool IsValidIssueCause(std::int32_t value) {
  return value >= static_cast<std::int32_t>(session::ArIssueCause::kNone) &&
         value <= static_cast<std::int32_t>(session::ArIssueCause::kUnknown);
}

// session_contract 规则 7：配置域枚举同样必须逐值校验、未知值原子拒绝。
// 以下辅助覆盖 session config / runtime patch 解码涉及的枚举范围。
bool IsKnownScanStartPosition(int raw_value) {
  using oneq::foundation::ScanStartPosition;
  return raw_value >= static_cast<int>(ScanStartPosition::kLeftTop) &&
         raw_value <= static_cast<int>(ScanStartPosition::kLeftBottom);
}

bool IsKnownScanSequence(int raw_value) {
  using oneq::foundation::ScanSequence;
  return raw_value >= static_cast<int>(ScanSequence::kAzimuthFirst) &&
         raw_value <= static_cast<int>(ScanSequence::kElevationFirst);
}

bool IsKnownArWorkMode(int raw_value) {
  return raw_value >= static_cast<int>(config::ArWorkMode::kStby) &&
         raw_value <= static_cast<int>(config::ArWorkMode::kStt);
}

bool IsKnownStabilizationMode(int raw_value) {
  return raw_value >= static_cast<int>(config::StabilizationMode::kBodyStabilized) &&
         raw_value <= static_cast<int>(config::StabilizationMode::kGroundStabilized);
}

bool IsKnownAntennaPatternModelType(int raw_value) {
  using config::detection::AntennaPatternModelType;
  return raw_value >= static_cast<int>(AntennaPatternModelType::kGaussianMainLobe) &&
         raw_value <= static_cast<int>(AntennaPatternModelType::kSincPattern);
}

bool IsKnownRfScenePolarization(int raw_value) {
  using oneq::electromagnetics::RfScenePolarization;
  return raw_value >= static_cast<int>(RfScenePolarization::kHorizontal) &&
         raw_value <= static_cast<int>(RfScenePolarization::kUnpolarized);
}

bool IsKnownVegetationCoverProfile(int raw_value) {
  return raw_value >= static_cast<int>(config::VegetationCoverProfile::kDisabled) &&
         raw_value <= static_cast<int>(config::VegetationCoverProfile::kTropicalDense);
}

// 校验解码后的 session config / runtime patch 枚举取值；调用方在提交
// （写入输出对象）之前执行，失败时输出对象保持原状（原子拒绝）。
bool IsValidDecodedOrientationEnums(const config::ArOrientationConfig& orientation) {
  return IsKnownScanStartPosition(static_cast<int>(orientation.scan_start_position)) &&
         IsKnownScanSequence(static_cast<int>(orientation.scan_sequence)) &&
         IsKnownStabilizationMode(static_cast<int>(orientation.stabilization_mode));
}

bool IsValidDecodedMissionEnums(const config::ArMissionConfig& mission) {
  return IsKnownArWorkMode(static_cast<int>(mission.work_mode));
}

bool IsValidDecodedDetectionEnums(const config::DetectionConfig& detection) {
  return IsKnownAntennaPatternModelType(
             static_cast<int>(detection.antenna.pattern.model_type)) &&
         IsKnownRfScenePolarization(
             static_cast<int>(detection.receiver.scene_polarization));
}

bool IsValidDecodedEnvironmentEnums(const config::EnvironmentScenarioConfig& scenario) {
  return IsKnownVegetationCoverProfile(
      static_cast<int>(scenario.vegetation_scatter_physics.cover_profile));
}

bool TryDecodeArInterferenceObservation(const fb::ArInterferenceObservation* value,
                                        session::ArInterferenceObservation* observation) {
  if (value == nullptr || observation == nullptr ||
      !IsKnownRfSceneWaveformKind(value->estimated_waveform_kind()) ||
      (value->deception_class() != static_cast<int>(session::DeceptionClass::kNone) &&
       value->deception_class() != static_cast<int>(session::DeceptionClass::kLikelyFalseTarget))) {
    return false;
  }
  session::ArInterferenceObservation candidate;
  candidate.observation_id = value->observation_id();
  candidate.estimated_bearing_azimuth_deg = value->estimated_bearing_azimuth_deg();
  candidate.estimated_bearing_elevation_deg = value->estimated_bearing_elevation_deg();
  candidate.estimated_off_boresight_deg = value->estimated_off_boresight_deg();
  candidate.estimated_center_frequency_hz = value->estimated_center_frequency_hz();
  candidate.estimated_bandwidth_hz = value->estimated_bandwidth_hz();
  candidate.estimated_waveform_kind =
      static_cast<oneq::electromagnetics::RfSceneWaveformKind>(value->estimated_waveform_kind());
  candidate.jammer_to_noise_db = value->jammer_to_noise_db();
  candidate.bearing_standard_deviation_deg = value->bearing_standard_deviation_deg();
  candidate.frequency_standard_deviation_hz = value->frequency_standard_deviation_hz();
  candidate.bandwidth_standard_deviation_hz = value->bandwidth_standard_deviation_hz();
  candidate.deception_class = static_cast<session::DeceptionClass>(value->deception_class());
  candidate.coherent_emission_count = value->coherent_emission_count();
  candidate.estimated_slant_range_m = value->estimated_slant_range_m();
  candidate.has_local_bearings = value->has_local_bearings();
  candidate.estimated_bearing_azimuth_local_deg = value->estimated_bearing_azimuth_local_deg();
  candidate.estimated_bearing_elevation_local_deg = value->estimated_bearing_elevation_local_deg();
  candidate.estimated_range_rate_mps = value->estimated_range_rate_mps();
  candidate.estimated_carrier_offset_hz = value->estimated_carrier_offset_hz();
  candidate.estimated_first_pulse_delay_s = value->estimated_first_pulse_delay_s();
  *observation = candidate;
  return true;
}

bool TryDecodeTacticalProposal(const fb::TacticalProposal* value,
                               session::TacticalProposal* result) {
  if (value == nullptr || result == nullptr) {
    return false;
  }
  if (value->directive() != nullptr) {
    const int type_raw = value->directive()->type();
    const int source_raw = value->directive()->source();
    // fail-closed：directive type/source 越界或为哨兵值时原子拒绝，不改写 *result。
    if (!IsKnownControlDirectiveType(type_raw) || !IsKnownControlDirectiveSource(source_raw)) {
      return false;
    }
    result->directive.type = static_cast<session::ControlDirectiveType>(type_raw);
    result->directive.source = static_cast<session::ControlDirectiveSource>(source_raw);
    result->directive.has_requested_value = value->directive()->has_requested_value();
    result->directive.requested_value = value->directive()->requested_value();
  }
  result->priority = value->priority();
  if (value->rationale() != nullptr) {
    result->rationale = value->rationale()->str();
  }
  return true;
}

bool TryDecodeTacticalProposals(
    const flatbuffers::Vector<flatbuffers::Offset<fb::TacticalProposal>>* values,
    std::vector<session::TacticalProposal>* result) {
  if (values == nullptr || result == nullptr) {
    return false;
  }
  // 先解码到 candidate，全部成功后再提交，保证拒绝时不改写 *result（fail-closed）。
  std::vector<session::TacticalProposal> candidate;
  candidate.reserve(values->size());
  for (flatbuffers::uoffset_t i = 0; i < values->size(); ++i) {
    session::TacticalProposal proposal;
    if (!TryDecodeTacticalProposal(values->Get(i), &proposal)) {
      return false;
    }
    candidate.push_back(proposal);
  }
  *result = std::move(candidate);
  return true;
}

flatbuffers::Offset<fb::ArDecisionReplayState> EncodeDecisionReplayState(
    flatbuffers::FlatBufferBuilder* builder, const ArDecisionReplayState& value) {
  return fb::CreateArDecisionReplayState(
      *builder, value.has_pending_internal_decision, value.pending_internal_cycle_index,
      value.pending_internal_batch_id,
      EncodeTacticalProposals(builder, value.pending_internal_proposals),
      static_cast<int>(value.applied_decision_source), value.applied_decision_cycle_index,
      value.applied_decision_batch_id,
      EncodeTacticalProposals(builder, value.applied_decision_proposals),
      value.reducer_state.lpi_hold_cycles_remaining, value.reducer_state.eccm_hold_cycles_remaining,
      value.reducer_state.lpi_cooldown_cycles_remaining,
      value.reducer_state.eccm_cooldown_cycles_remaining);
}

bool TryDecodeDecisionReplayState(const fb::ArDecisionReplayState* value,
                                  ArDecisionReplayState* result) {
  if (value == nullptr || result == nullptr) {
    return false;
  }
  // 先解码到 candidate，全部成功后再提交，保证拒绝时不改写 *result（fail-closed）。
  ArDecisionReplayState candidate;
  candidate.has_pending_internal_decision = value->has_pending_internal_decision();
  candidate.pending_internal_cycle_index = value->pending_internal_cycle_index();
  candidate.pending_internal_batch_id = value->pending_internal_batch_id();
  if (!TryDecodeTacticalProposals(value->pending_internal_proposals(),
                                  &candidate.pending_internal_proposals)) {
    return false;
  }
  const int applied_source_raw = value->applied_decision_source();
  if (!IsKnownDecisionControlSource(applied_source_raw)) {
    return false;
  }
  candidate.applied_decision_source = static_cast<session::DecisionControlSource>(applied_source_raw);
  candidate.applied_decision_cycle_index = value->applied_decision_cycle_index();
  candidate.applied_decision_batch_id = value->applied_decision_batch_id();
  if (!TryDecodeTacticalProposals(value->applied_decision_proposals(),
                                  &candidate.applied_decision_proposals)) {
    return false;
  }
  candidate.reducer_state.lpi_hold_cycles_remaining = value->lpi_hold_cycles_remaining();
  candidate.reducer_state.eccm_hold_cycles_remaining = value->eccm_hold_cycles_remaining();
  candidate.reducer_state.lpi_cooldown_cycles_remaining = value->lpi_cooldown_cycles_remaining();
  candidate.reducer_state.eccm_cooldown_cycles_remaining = value->eccm_cooldown_cycles_remaining();
  *result = candidate;
  return true;
}

flatbuffers::Offset<session_fb::EulerAnglesDeg> EncodeSessionEulerAngles(
    flatbuffers::FlatBufferBuilder* builder, const config::EulerAnglesDeg& value) {
  return session_fb::CreateEulerAnglesDeg(*builder, value.yaw_deg, value.pitch_deg, value.roll_deg);
}

flatbuffers::Offset<session_fb::AzimuthElevationDeg> EncodeSessionAzEl(
    flatbuffers::FlatBufferBuilder* builder, const config::AzimuthElevationDeg& value) {
  return session_fb::CreateAzimuthElevationDeg(*builder, value.az_deg, value.el_deg);
}

flatbuffers::Offset<session_fb::AzimuthElevationLimitsDeg> EncodeSessionAzElLimits(
    flatbuffers::FlatBufferBuilder* builder, const config::AzimuthElevationLimitsDeg& value) {
  return session_fb::CreateAzimuthElevationLimitsDeg(*builder, value.az_min_deg, value.az_max_deg,
                                                     value.el_min_deg, value.el_max_deg);
}

flatbuffers::Offset<session_fb::CommandedBeamwidthDeg> EncodeSessionCommandedBeamwidth(
    flatbuffers::FlatBufferBuilder* builder, const config::CommandedBeamwidthDeg& value) {
  return session_fb::CreateCommandedBeamwidthDeg(*builder, value.commanded_az_beamwidth_deg,
                                                 value.commanded_el_beamwidth_deg);
}

flatbuffers::Offset<session_fb::ArOrientationConfig> EncodeSessionOrientation(
    flatbuffers::FlatBufferBuilder* builder, const config::ArOrientationConfig& value) {
  return session_fb::CreateArOrientationConfig(
      *builder, EncodeSessionEulerAngles(builder, value.mount_angles_deg),
      EncodeSessionAzElLimits(builder, value.mechanical_scan_limits_deg),
      EncodeSessionAzElLimits(builder, value.electronic_scan_limits_deg),
      static_cast<int>(value.scan_start_position), static_cast<int>(value.scan_sequence),
      static_cast<int>(value.stabilization_mode));
}

flatbuffers::Offset<session_fb::ArMissionConfig> EncodeSessionMission(
    flatbuffers::FlatBufferBuilder* builder, const config::ArMissionConfig& value) {
  return session_fb::CreateArMissionConfig(
      *builder, static_cast<int>(value.work_mode), EncodeSessionAzEl(builder, value.scan_center_deg),
      value.commanded_beamwidth_enabled,
      EncodeSessionCommandedBeamwidth(builder, value.commanded_beamwidth_deg));
}

flatbuffers::Offset<session_fb::DetectionConfig> EncodeSessionDetectionConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::DetectionConfig& value) {
  const auto frequency_plan = builder->CreateVector(value.transmitter.frequency_plan_hz);
  session_fb::TransmitterConfigBuilder transmitter_builder(*builder);
  transmitter_builder.add_peak_power_w(value.transmitter.peak_power_w);
  transmitter_builder.add_frequency_hz(value.transmitter.frequency_hz);
  transmitter_builder.add_bandwidth_hz(value.transmitter.bandwidth_hz);
  transmitter_builder.add_pulse_width_s(value.transmitter.pulse_width_s);
  transmitter_builder.add_prf_hz(value.transmitter.prf_hz);
  transmitter_builder.add_transmit_loss_db(value.transmitter.transmit_loss_db);
  transmitter_builder.add_maximum_peak_power_w(value.transmitter.maximum_peak_power_w);
  transmitter_builder.add_maximum_duty_cycle(value.transmitter.maximum_duty_cycle);
  transmitter_builder.add_maximum_pulse_energy_j(value.transmitter.maximum_pulse_energy_j);
  transmitter_builder.add_frequency_plan_hz(frequency_plan);
  const flatbuffers::Offset<session_fb::TransmitterConfig> transmitter =
      transmitter_builder.Finish();
  const flatbuffers::Offset<session_fb::AntennaPatternConfig> pattern =
      session_fb::CreateAntennaPatternConfig(
          *builder, static_cast<int>(value.antenna.pattern.model_type),
          value.antenna.pattern.max_sidelobe_level_db, value.antenna.pattern.backlobe_level_db,
          value.antenna.pattern.scan_loss_coeff_db_per_deg2, value.antenna.pattern.max_scan_loss_db,
          EncodeSessionAzEl(builder, value.antenna.pattern.boresight_offset_deg));
  const flatbuffers::Offset<session_fb::AntennaConfig> antenna = session_fb::CreateAntennaConfig(
      *builder, value.antenna.main_beam_gain_db, value.antenna.nominal_az_beamwidth_deg,
      value.antenna.nominal_el_beamwidth_deg, value.antenna.enable_directional_pattern, pattern,
      value.antenna.antenna_length_m, value.antenna.antenna_width_m);
  std::vector<flatbuffers::Offset<session_fb::RfCoSiteIsolationPath>> co_site_paths;
  co_site_paths.reserve(value.receiver.co_site_paths.size());
  for (const oneq::electromagnetics::RfCoSiteIsolationPath& path : value.receiver.co_site_paths) {
    co_site_paths.push_back(session_fb::CreateRfCoSiteIsolationPath(
        *builder, path.transmitter_sensor_entity_id, path.isolation_db));
  }
  // 向量必须在打开 table builder 之前创建（flatbuffers NotNested 约束）。
  const auto co_site_paths_vec = builder->CreateVector(co_site_paths);
  session_fb::ReceiverConfigBuilder receiver_builder(*builder);
  receiver_builder.add_noise_figure_db(value.receiver.noise_figure_db);
  receiver_builder.add_receive_loss_db(value.receiver.receive_loss_db);
  receiver_builder.add_cross_polarization_isolation_db(
      value.receiver.cross_polarization_isolation_db);
  receiver_builder.add_minimum_far_field_range_m(value.receiver.minimum_far_field_range_m);
  receiver_builder.add_has_co_site_isolation(value.receiver.has_co_site_isolation);
  receiver_builder.add_co_site_isolation_db(value.receiver.co_site_isolation_db);
  receiver_builder.add_maximum_linear_input_power_w(value.receiver.maximum_linear_input_power_w);
  receiver_builder.add_preselector_bandwidth_hz(value.receiver.preselector_bandwidth_hz);
  receiver_builder.add_interference_observation_jn_gate_db(
      value.receiver.interference_observation_jn_gate_db);
  receiver_builder.add_scene_polarization(static_cast<int>(value.receiver.scene_polarization));
  receiver_builder.add_co_site_paths(co_site_paths_vec);
  const flatbuffers::Offset<session_fb::ReceiverConfig> receiver = receiver_builder.Finish();
  const flatbuffers::Offset<session_fb::RcsPhysicsConfig> rcs_physics =
      session_fb::CreateRcsPhysicsConfig(
          *builder, value.rcs_physics.enable_physical_rcs, value.rcs_physics.physics_mix_ratio,
          value.rcs_physics.cylinder_weight, value.rcs_physics.min_equivalent_radius_m,
          value.rcs_physics.max_equivalent_radius_m, value.rcs_physics.min_rcs_m2,
          value.rcs_physics.max_rcs_m2, value.rcs_physics.bistatic_psi_offset_deg);
  const flatbuffers::Offset<session_fb::SignalProcessingConfig> signal_processing =
      session_fb::CreateSignalProcessingConfig(
          *builder, value.signal_processing.target_processing_gain_db,
          value.signal_processing.noise_processing_gain_db,
          value.signal_processing.clutter_suppression_gain_db,
          value.signal_processing.jamming_suppression_gain_db);
  return session_fb::CreateDetectionConfig(
      *builder, static_cast<int>(config::profiles::SwerlingModel::kSwerling0), transmitter, antenna,
      receiver, rcs_physics, signal_processing);
}

flatbuffers::Offset<session_fb::ArPolicyConfig> EncodeSessionPolicyConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::ArPolicyConfig& value) {
  const flatbuffers::Offset<session_fb::ArDetectionPolicyConfig> detection =
      session_fb::CreateArDetectionPolicyConfig(*builder, value.detection.minimum_snr_db,
                                                value.detection.pfa, value.detection.pulse_count,
                                                value.detection.minimum_detection_margin_db);
  const flatbuffers::Offset<session_fb::BeamPointingConfig> pointing =
      session_fb::CreateBeamPointingConfig(
          *builder, EncodeSessionCommandedBeamwidth(
                        builder, value.beam_control.pointing.nominal_beamwidth_deg));
  const flatbuffers::Offset<session_fb::BeamSchedulerConfig> scheduler =
      session_fb::CreateBeamSchedulerConfig(*builder,
                                            value.beam_control.scheduler.azimuth_step_count_hint,
                                            value.beam_control.scheduler.elevation_step_count_hint,
                                            value.beam_control.scheduler.prefer_dense_tas_sampling);
  const flatbuffers::Offset<session_fb::BeamControlConfig> beam_control =
      session_fb::CreateBeamControlConfig(*builder, pointing, scheduler);
  const flatbuffers::Offset<session_fb::AssociationConfig> association =
      session_fb::CreateAssociationConfig(*builder, value.association.distance_gate_sigma);
  const flatbuffers::Offset<session_fb::TrackingConfig> tracking = session_fb::CreateTrackingConfig(
      *builder, value.tracking.enable_kalman_filter, value.tracking.kalman_measurement_noise_std,
      value.tracking.speed_decay_ratio_on_loss, value.tracking.rcs_decay_ratio_on_loss);
  const flatbuffers::Offset<session_fb::LifecycleConfig> lifecycle =
      session_fb::CreateLifecycleConfig(
          *builder, value.lifecycle.confirm_hits, value.lifecycle.max_miss_before_lost,
          value.lifecycle.max_lost_cycles, value.lifecycle.enable_imm_lifecycle);
  const flatbuffers::Offset<session_fb::ImmConfig> imm = session_fb::CreateImmConfig(
      *builder, value.lifecycle.enable_imm_lifecycle, value.lifecycle.model_count_hint);
  const flatbuffers::Offset<session_fb::DecisionControlConfig> decision_control =
      session_fb::CreateDecisionControlConfig(
          *builder, value.decision_control.lpi_hold_cycles_after_request,
          value.decision_control.eccm_hold_cycles_after_request,
          value.decision_control.lpi_cooldown_cycles_after_release,
          value.decision_control.eccm_cooldown_cycles_after_release);
  return session_fb::CreateArPolicyConfig(*builder, detection, beam_control, association, tracking,
                                          lifecycle, imm, decision_control);
}

flatbuffers::Offset<session_fb::AtmosphericPhysicsConfig> EncodeSessionAtmosphericPhysicsConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::AtmosphericPhysicsConfig& value) {
  return session_fb::CreateAtmosphericPhysicsConfig(*builder, value.enable_physical_model,
                                                    value.pressure_hpa, value.temperature_k,
                                                    value.relative_humidity);
}

flatbuffers::Offset<session_fb::VegetationScatterPhysicsConfig>
EncodeSessionVegetationScatterPhysicsConfig(flatbuffers::FlatBufferBuilder* builder,
                                            const config::VegetationScatterPhysicsConfig& value) {
  return session_fb::CreateVegetationScatterPhysicsConfig(
      *builder, static_cast<int>(value.cover_profile), value.enable_physical_model);
}

flatbuffers::Offset<session_fb::EnvironmentScenarioConfig> EncodeSessionEnvironmentScenarioConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::EnvironmentScenarioConfig& value) {
  return session_fb::CreateEnvironmentScenarioConfig(
      *builder, EncodeSessionAtmosphericPhysicsConfig(builder, value.atmospheric_physics),
      EncodeSessionVegetationScatterPhysicsConfig(builder, value.vegetation_scatter_physics));
}

flatbuffers::Offset<session_fb::EnvironmentRuntimeConfigPatch>
EncodeSessionEnvironmentRuntimeConfigPatch(flatbuffers::FlatBufferBuilder* builder,
                                           const config::EnvironmentRuntimeConfigPatch& value) {
  return session_fb::CreateEnvironmentRuntimeConfigPatch(
      *builder, value.has_scenario_config,
      EncodeSessionEnvironmentScenarioConfig(builder, value.scenario_config));
}

config::EulerAnglesDeg DecodeSessionEulerAngles(const session_fb::EulerAnglesDeg* value) {
  config::EulerAnglesDeg result;
  if (value != nullptr) {
    result.yaw_deg = value->yaw_deg();
    result.pitch_deg = value->pitch_deg();
    result.roll_deg = value->roll_deg();
  }
  return result;
}

config::AzimuthElevationDeg DecodeSessionAzEl(const session_fb::AzimuthElevationDeg* value) {
  config::AzimuthElevationDeg result;
  if (value != nullptr) {
    result.az_deg = value->az_deg();
    result.el_deg = value->el_deg();
  }
  return result;
}

config::AzimuthElevationLimitsDeg DecodeSessionAzElLimits(
    const session_fb::AzimuthElevationLimitsDeg* value) {
  config::AzimuthElevationLimitsDeg result;
  if (value != nullptr) {
    result.az_min_deg = value->az_min_deg();
    result.az_max_deg = value->az_max_deg();
    result.el_min_deg = value->el_min_deg();
    result.el_max_deg = value->el_max_deg();
  }
  return result;
}

config::CommandedBeamwidthDeg DecodeSessionCommandedBeamwidth(
    const session_fb::CommandedBeamwidthDeg* value) {
  config::CommandedBeamwidthDeg result;
  if (value != nullptr) {
    result.commanded_az_beamwidth_deg = value->commanded_az_beamwidth_deg();
    result.commanded_el_beamwidth_deg = value->commanded_el_beamwidth_deg();
  }
  return result;
}

config::ArOrientationConfig DecodeSessionOrientation(const session_fb::ArOrientationConfig* value) {
  config::ArOrientationConfig result;
  if (value != nullptr) {
    result.mount_angles_deg = DecodeSessionEulerAngles(value->mount_angles_deg());
    result.mechanical_scan_limits_deg =
        DecodeSessionAzElLimits(value->mechanical_scan_limits_deg());
    result.electronic_scan_limits_deg =
        DecodeSessionAzElLimits(value->electronic_scan_limits_deg());
    result.scan_start_position =
        static_cast<oneq::foundation::ScanStartPosition>(value->scan_start_position());
    result.scan_sequence = static_cast<oneq::foundation::ScanSequence>(value->scan_sequence());
    result.stabilization_mode = static_cast<config::StabilizationMode>(value->stabilization_mode());
  }
  return result;
}

config::ArMissionConfig DecodeSessionMission(const session_fb::ArMissionConfig* value) {
  config::ArMissionConfig result;
  if (value != nullptr) {
    result.work_mode = static_cast<config::ArWorkMode>(value->work_mode());
    result.scan_center_deg = DecodeSessionAzEl(value->scan_center_deg());
    result.commanded_beamwidth_enabled = value->commanded_beamwidth_enabled();
    result.commanded_beamwidth_deg =
        DecodeSessionCommandedBeamwidth(value->commanded_beamwidth_deg());
  }
  return result;
}

config::DetectionConfig DecodeSessionDetectionConfig(const session_fb::DetectionConfig* value) {
  config::DetectionConfig result;
  if (value != nullptr) {
    const session_fb::TransmitterConfig* transmitter = value->transmitter();
    if (transmitter != nullptr) {
      result.transmitter.peak_power_w = transmitter->peak_power_w();
      result.transmitter.frequency_hz = transmitter->frequency_hz();
      result.transmitter.bandwidth_hz = transmitter->bandwidth_hz();
      result.transmitter.pulse_width_s = transmitter->pulse_width_s();
      result.transmitter.prf_hz = transmitter->prf_hz();
      result.transmitter.transmit_loss_db = transmitter->transmit_loss_db();
      result.transmitter.maximum_peak_power_w = transmitter->maximum_peak_power_w();
      result.transmitter.maximum_duty_cycle = transmitter->maximum_duty_cycle();
      result.transmitter.maximum_pulse_energy_j = transmitter->maximum_pulse_energy_j();
      if (transmitter->frequency_plan_hz() != nullptr) {
        result.transmitter.frequency_plan_hz.assign(transmitter->frequency_plan_hz()->begin(),
                                                    transmitter->frequency_plan_hz()->end());
      }
    }
    const session_fb::AntennaConfig* antenna = value->antenna();
    if (antenna != nullptr) {
      result.antenna.main_beam_gain_db = antenna->main_beam_gain_db();
      result.antenna.nominal_az_beamwidth_deg = antenna->nominal_az_beamwidth_deg();
      result.antenna.nominal_el_beamwidth_deg = antenna->nominal_el_beamwidth_deg();
      result.antenna.antenna_length_m = antenna->antenna_length_m();
      result.antenna.antenna_width_m = antenna->antenna_width_m();
      result.antenna.enable_directional_pattern = antenna->enable_directional_pattern();
      const session_fb::AntennaPatternConfig* pattern = antenna->pattern();
      if (pattern != nullptr) {
        result.antenna.pattern.model_type =
            static_cast<config::detection::AntennaPatternModelType>(pattern->model_type());
        result.antenna.pattern.max_sidelobe_level_db = pattern->max_sidelobe_level_db();
        result.antenna.pattern.backlobe_level_db = pattern->backlobe_level_db();
        result.antenna.pattern.scan_loss_coeff_db_per_deg2 = pattern->scan_loss_coeff_db_per_deg2();
        result.antenna.pattern.max_scan_loss_db = pattern->max_scan_loss_db();
        result.antenna.pattern.boresight_offset_deg =
            DecodeSessionAzEl(pattern->boresight_offset_deg());
      }
    }
    const session_fb::ReceiverConfig* receiver = value->receiver();
    if (receiver != nullptr) {
      result.receiver.noise_figure_db = receiver->noise_figure_db();
      result.receiver.receive_loss_db = receiver->receive_loss_db();
      result.receiver.cross_polarization_isolation_db = receiver->cross_polarization_isolation_db();
      result.receiver.minimum_far_field_range_m = receiver->minimum_far_field_range_m();
      result.receiver.has_co_site_isolation = receiver->has_co_site_isolation();
      result.receiver.co_site_isolation_db = receiver->co_site_isolation_db();
      result.receiver.maximum_linear_input_power_w = receiver->maximum_linear_input_power_w();
      result.receiver.preselector_bandwidth_hz = receiver->preselector_bandwidth_hz();
      result.receiver.interference_observation_jn_gate_db =
          receiver->interference_observation_jn_gate_db();
      result.receiver.scene_polarization =
          static_cast<oneq::electromagnetics::RfScenePolarization>(receiver->scene_polarization());
      result.receiver.co_site_paths.clear();
      if (receiver->co_site_paths() != nullptr) {
        result.receiver.co_site_paths.reserve(receiver->co_site_paths()->size());
        for (const session_fb::RfCoSiteIsolationPath* encoded : *receiver->co_site_paths()) {
          oneq::electromagnetics::RfCoSiteIsolationPath path;
          path.transmitter_sensor_entity_id = encoded->transmitter_sensor_entity_id();
          path.isolation_db = encoded->isolation_db();
          result.receiver.co_site_paths.push_back(path);
        }
      }
    }
    const session_fb::RcsPhysicsConfig* rcs_physics = value->rcs_physics();
    if (rcs_physics != nullptr) {
      result.rcs_physics.enable_physical_rcs = rcs_physics->enable_physical_rcs();
      result.rcs_physics.physics_mix_ratio = rcs_physics->physics_mix_ratio();
      result.rcs_physics.cylinder_weight = rcs_physics->cylinder_weight();
      result.rcs_physics.min_equivalent_radius_m = rcs_physics->min_equivalent_radius_m();
      result.rcs_physics.max_equivalent_radius_m = rcs_physics->max_equivalent_radius_m();
      result.rcs_physics.min_rcs_m2 = rcs_physics->min_rcs_m2();
      result.rcs_physics.max_rcs_m2 = rcs_physics->max_rcs_m2();
      result.rcs_physics.bistatic_psi_offset_deg = rcs_physics->bistatic_psi_offset_deg();
    }
    const session_fb::SignalProcessingConfig* signal_processing = value->signal_processing();
    if (signal_processing != nullptr) {
      result.signal_processing.target_processing_gain_db =
          signal_processing->target_processing_gain_db();
      result.signal_processing.noise_processing_gain_db =
          signal_processing->noise_processing_gain_db();
      result.signal_processing.clutter_suppression_gain_db =
          signal_processing->clutter_suppression_gain_db();
      result.signal_processing.jamming_suppression_gain_db =
          signal_processing->jamming_suppression_gain_db();
    }
  }
  return result;
}

config::ArPolicyConfig DecodeSessionPolicyConfig(const session_fb::ArPolicyConfig* value) {
  config::ArPolicyConfig result;
  if (value != nullptr) {
    const session_fb::ArDetectionPolicyConfig* detection = value->detection();
    if (detection != nullptr) {
      result.detection.minimum_snr_db = detection->minimum_snr_db();
      result.detection.pfa = detection->pfa();
      result.detection.pulse_count = detection->pulse_count();
      result.detection.minimum_detection_margin_db = detection->minimum_detection_margin_db();
    }
    const session_fb::BeamControlConfig* beam_control = value->beam_control();
    if (beam_control != nullptr) {
      const session_fb::BeamPointingConfig* pointing = beam_control->pointing();
      if (pointing != nullptr) {
        result.beam_control.pointing.nominal_beamwidth_deg =
            DecodeSessionCommandedBeamwidth(pointing->nominal_beamwidth_deg());
      }
      const session_fb::BeamSchedulerConfig* scheduler = beam_control->scheduler();
      if (scheduler != nullptr) {
        result.beam_control.scheduler.azimuth_step_count_hint =
            scheduler->azimuth_step_count_hint();
        result.beam_control.scheduler.elevation_step_count_hint =
            scheduler->elevation_step_count_hint();
        result.beam_control.scheduler.prefer_dense_tas_sampling =
            scheduler->prefer_dense_tas_sampling();
      }
    }
    const session_fb::AssociationConfig* association = value->association();
    if (association != nullptr) {
      result.association.distance_gate_sigma = association->distance_gate_sigma();
    }
    const session_fb::TrackingConfig* tracking = value->tracking();
    if (tracking != nullptr) {
      result.tracking.enable_kalman_filter = tracking->enable_kalman_filter();
      result.tracking.kalman_measurement_noise_std = tracking->kalman_measurement_noise_std();
      result.tracking.speed_decay_ratio_on_loss = tracking->speed_decay_ratio_on_loss();
      result.tracking.rcs_decay_ratio_on_loss = tracking->rcs_decay_ratio_on_loss();
    }
    const session_fb::LifecycleConfig* lifecycle = value->lifecycle();
    if (lifecycle != nullptr) {
      result.lifecycle.confirm_hits = lifecycle->confirm_hits();
      result.lifecycle.max_miss_before_lost = lifecycle->max_miss_before_lost();
      result.lifecycle.max_lost_cycles = lifecycle->max_lost_cycles();
      result.lifecycle.enable_imm_lifecycle = lifecycle->enable_imm_lifecycle();
    }
    const session_fb::ImmConfig* imm = value->imm();
    if (imm != nullptr) {
      result.lifecycle.enable_imm_lifecycle = imm->enable_imm_lifecycle();
      result.lifecycle.model_count_hint = imm->model_count_hint();
    }
    const session_fb::DecisionControlConfig* decision_control = value->decision_control();
    if (decision_control != nullptr) {
      result.decision_control.lpi_hold_cycles_after_request =
          decision_control->lpi_hold_cycles_after_request();
      result.decision_control.eccm_hold_cycles_after_request =
          decision_control->eccm_hold_cycles_after_request();
      result.decision_control.lpi_cooldown_cycles_after_release =
          decision_control->lpi_cooldown_cycles_after_release();
      result.decision_control.eccm_cooldown_cycles_after_release =
          decision_control->eccm_cooldown_cycles_after_release();
    }
  }
  return result;
}

config::AtmosphericPhysicsConfig DecodeSessionAtmosphericPhysicsConfig(
    const session_fb::AtmosphericPhysicsConfig* value) {
  config::AtmosphericPhysicsConfig result;
  if (value != nullptr) {
    result.enable_physical_model = value->enable_physical_model();
    result.pressure_hpa = value->pressure_hpa();
    result.temperature_k = value->temperature_k();
    result.relative_humidity = value->relative_humidity();
  }
  return result;
}

config::VegetationScatterPhysicsConfig DecodeSessionVegetationScatterPhysicsConfig(
    const session_fb::VegetationScatterPhysicsConfig* value) {
  config::VegetationScatterPhysicsConfig result;
  if (value != nullptr) {
    result.cover_profile = static_cast<config::VegetationCoverProfile>(value->cover_profile());
    result.enable_physical_model = value->enable_physical_model();
  }
  return result;
}

config::EnvironmentScenarioConfig DecodeSessionEnvironmentScenarioConfig(
    const session_fb::EnvironmentScenarioConfig* value) {
  config::EnvironmentScenarioConfig result;
  if (value != nullptr) {
    result.atmospheric_physics =
        DecodeSessionAtmosphericPhysicsConfig(value->atmospheric_physics());
    result.vegetation_scatter_physics =
        DecodeSessionVegetationScatterPhysicsConfig(value->vegetation_scatter_physics());
  }
  return result;
}

config::EnvironmentRuntimeConfigPatch DecodeSessionEnvironmentRuntimeConfigPatch(
    const session_fb::EnvironmentRuntimeConfigPatch* value) {
  config::EnvironmentRuntimeConfigPatch result;
  if (value != nullptr) {
    result.has_scenario_config = value->has_scenario_config();
    result.scenario_config = DecodeSessionEnvironmentScenarioConfig(value->scenario_config());
  }
  return result;
}

flatbuffers::Offset<session_fb::EnvironmentDefaultConfig> EncodeEnvironmentDefaultConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::ArEnvironmentConfig& value) {
  return session_fb::CreateEnvironmentDefaultConfig(
      *builder, EncodeSessionEnvironmentScenarioConfig(builder, value.scenario_config));
}

config::ArEnvironmentConfig DecodeEnvironmentDefaultConfig(
    const session_fb::EnvironmentDefaultConfig* value) {
  config::ArEnvironmentConfig result;
  if (value != nullptr) {
    result.scenario_config = DecodeSessionEnvironmentScenarioConfig(value->scenario_config());
  }
  return result;
}

flatbuffers::Offset<fb::Vector3d> EncodeRfV2Position(flatbuffers::FlatBufferBuilder* builder,
                                                     const oneq::coordinate::EcefPositionM& value) {
  return fb::CreateVector3d(*builder, value.x_m, value.y_m, value.z_m);
}

flatbuffers::Offset<fb::Vector3d> EncodeRfV2Velocity(
    flatbuffers::FlatBufferBuilder* builder, const oneq::coordinate::EcefVelocityMps& value) {
  return fb::CreateVector3d(*builder, value.x_mps, value.y_mps, value.z_mps);
}

flatbuffers::Offset<fb::RfSceneDirectionV2> EncodeRfV2Direction(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::electromagnetics::RfSceneDirection& value) {
  return fb::CreateRfSceneDirectionV2(*builder, value.x, value.y, value.z);
}

flatbuffers::Offset<fb::RfSceneAntennaPatternV2> EncodeRfV2Antenna(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::electromagnetics::RfSceneAntennaPattern& value) {
  return fb::CreateRfSceneAntennaPatternV2(
      *builder, EncodeRfV2Direction(builder, value.boresight_ecef), value.peak_gain_dbi,
      value.half_power_beamwidth_deg, value.sidelobe_level_db, value.backlobe_level_db,
      value.cross_polarization_isolation_db);
}

flatbuffers::Offset<fb::RfEmissionIdentityV2> EncodeRfV2Identity(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::electromagnetics::RfEmissionIdentity& value) {
  return fb::CreateRfEmissionIdentityV2(*builder, value.sensor_entity_id,
                                        value.emission_id);
}

flatbuffers::Offset<fb::RfWaveformScheduleV2> EncodeRfV2Waveform(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::electromagnetics::RfWaveformSchedule& value) {
  return fb::CreateRfWaveformScheduleV2(
      *builder, static_cast<int>(value.kind), value.activity_start_time_s,
      value.activity_duration_s, value.center_frequency_hz, value.occupied_bandwidth_hz,
      value.transmit_power_w, value.pulse_width_s, value.pulse_repetition_interval_s,
      value.first_pulse_time_s, value.pulse_count, value.pulse_jitter_fraction, value.timing_seed,
      value.timing_epoch, value.sweep_start_frequency_hz, value.sweep_stop_frequency_hz,
      value.sweep_period_s);
}

flatbuffers::Offset<fb::RfSceneEmissionV2> EncodeRfV2Emission(
    flatbuffers::FlatBufferBuilder* builder, const oneq::electromagnetics::RfSceneEmission& value) {
  return fb::CreateRfSceneEmissionV2(*builder, EncodeRfV2Identity(builder, value.identity),
                                     EncodeRfV2Position(builder, value.position_ecef_m),
                                     EncodeRfV2Velocity(builder, value.velocity_ecef_mps),
                                     EncodeRfV2Antenna(builder, value.antenna),
                                     static_cast<int>(value.polarization),
                                     EncodeRfV2Waveform(builder, value.waveform));
}

flatbuffers::Offset<fb::RfSceneFrameV2> EncodeRfV2Scene(
    flatbuffers::FlatBufferBuilder* builder, const oneq::electromagnetics::RfSceneFrame& value) {
  std::vector<flatbuffers::Offset<fb::RfSceneEmissionV2>> emissions;
  emissions.reserve(value.emissions.size());
  for (const auto& emission : value.emissions) {
    emissions.push_back(EncodeRfV2Emission(builder, emission));
  }
  return fb::CreateRfSceneFrameV2(*builder, value.world_cycle_index, value.window_start_time_s,
                                  value.window_duration_s, builder->CreateVector(emissions));
}

oneq::coordinate::EcefPositionM DecodeRfV2Position(const fb::Vector3d* value) {
  oneq::coordinate::EcefPositionM result;
  if (value != nullptr) {
    result.x_m = value->x();
    result.y_m = value->y();
    result.z_m = value->z();
  }
  return result;
}

oneq::coordinate::EcefVelocityMps DecodeRfV2Velocity(const fb::Vector3d* value) {
  oneq::coordinate::EcefVelocityMps result;
  if (value != nullptr) {
    result.x_mps = value->x();
    result.y_mps = value->y();
    result.z_mps = value->z();
  }
  return result;
}

oneq::electromagnetics::RfSceneDirection DecodeRfV2Direction(const fb::RfSceneDirectionV2* value) {
  oneq::electromagnetics::RfSceneDirection result;
  if (value != nullptr) {
    result.x = value->x();
    result.y = value->y();
    result.z = value->z();
  }
  return result;
}

oneq::electromagnetics::RfSceneAntennaPattern DecodeRfV2Antenna(
    const fb::RfSceneAntennaPatternV2* value) {
  oneq::electromagnetics::RfSceneAntennaPattern result;
  if (value != nullptr) {
    result.boresight_ecef = DecodeRfV2Direction(value->boresight_ecef());
    result.peak_gain_dbi = value->peak_gain_dbi();
    result.half_power_beamwidth_deg = value->half_power_beamwidth_deg();
    result.sidelobe_level_db = value->sidelobe_level_db();
    result.backlobe_level_db = value->backlobe_level_db();
    result.cross_polarization_isolation_db = value->cross_polarization_isolation_db();
  }
  return result;
}

oneq::electromagnetics::RfWaveformSchedule DecodeRfV2Waveform(
    const fb::RfWaveformScheduleV2* value) {
  oneq::electromagnetics::RfWaveformSchedule result;
  if (value != nullptr) {
    result.kind = static_cast<oneq::electromagnetics::RfSceneWaveformKind>(value->kind());
    result.activity_start_time_s = value->activity_start_time_s();
    result.activity_duration_s = value->activity_duration_s();
    result.center_frequency_hz = value->center_frequency_hz();
    result.occupied_bandwidth_hz = value->occupied_bandwidth_hz();
    result.transmit_power_w = value->transmit_power_w();
    result.pulse_width_s = value->pulse_width_s();
    result.pulse_repetition_interval_s = value->pulse_repetition_interval_s();
    result.first_pulse_time_s = value->first_pulse_time_s();
    result.pulse_count = value->pulse_count();
    result.pulse_jitter_fraction = value->pulse_jitter_fraction();
    result.timing_seed = value->timing_seed();
    result.timing_epoch = value->timing_epoch();
    result.sweep_start_frequency_hz = value->sweep_start_frequency_hz();
    result.sweep_stop_frequency_hz = value->sweep_stop_frequency_hz();
    result.sweep_period_s = value->sweep_period_s();
  }
  return result;
}

oneq::electromagnetics::RfSceneEmission DecodeRfV2Emission(const fb::RfSceneEmissionV2* value) {
  oneq::electromagnetics::RfSceneEmission result;
  if (value != nullptr) {
    if (value->identity() != nullptr) {
      result.identity.sensor_entity_id = value->identity()->sensor_entity_id();
      result.identity.emission_id = value->identity()->emission_id();
    }
    result.position_ecef_m = DecodeRfV2Position(value->position_ecef_m());
    result.velocity_ecef_mps = DecodeRfV2Velocity(value->velocity_ecef_mps());
    result.antenna = DecodeRfV2Antenna(value->antenna());
    result.polarization =
        static_cast<oneq::electromagnetics::RfScenePolarization>(value->polarization());
    result.waveform = DecodeRfV2Waveform(value->waveform());
  }
  return result;
}

oneq::electromagnetics::RfSceneFrame DecodeRfV2Scene(const fb::RfSceneFrameV2* value) {
  oneq::electromagnetics::RfSceneFrame result;
  if (value != nullptr) {
    result.world_cycle_index = value->world_cycle_index();
    result.window_start_time_s = value->window_start_time_s();
    result.window_duration_s = value->window_duration_s();
    if (value->emissions() != nullptr) {
      result.emissions.reserve(value->emissions()->size());
      for (const fb::RfSceneEmissionV2* emission : *value->emissions()) {
        result.emissions.push_back(DecodeRfV2Emission(emission));
      }
    }
  }
  return result;
}

flatbuffers::Offset<fb::EulerAnglesDeg64> EncodeEulerAnglesV3(
    flatbuffers::FlatBufferBuilder* builder, const oneq::coordinate::EulerAnglesDeg& value) {
  return fb::CreateEulerAnglesDeg64(*builder, value.yaw_deg, value.pitch_deg, value.roll_deg);
}

oneq::coordinate::EulerAnglesDeg DecodeEulerAnglesV3(const fb::EulerAnglesDeg64* value) {
  oneq::coordinate::EulerAnglesDeg result;
  if (value != nullptr) {
    result.yaw_deg = value->yaw_deg();
    result.pitch_deg = value->pitch_deg();
    result.roll_deg = value->roll_deg();
  }
  return result;
}

flatbuffers::Offset<fb::EcefVelocityV3> EncodeEcefVelocityV3(
    flatbuffers::FlatBufferBuilder* builder, const oneq::coordinate::EcefVelocityMps& value) {
  return fb::CreateEcefVelocityV3(*builder, value.x_mps, value.y_mps, value.z_mps);
}

oneq::coordinate::EcefVelocityMps DecodeEcefVelocityV3(const fb::EcefVelocityV3* value) {
  oneq::coordinate::EcefVelocityMps result;
  if (value != nullptr) {
    result.x_mps = value->x_mps();
    result.y_mps = value->y_mps();
    result.z_mps = value->z_mps();
  }
  return result;
}

flatbuffers::Offset<fb::ArPlatformInputV3> EncodePlatformInputV3(
    flatbuffers::FlatBufferBuilder* builder, const ArPlatformInput& value) {
  return fb::CreateArPlatformInputV3(*builder,
                                     EncodeRfV2Position(builder, value.platform_position_ecef_m),
                                     EncodeEcefVelocityV3(builder, value.platform_velocity_mps),
                                     EncodeEulerAnglesV3(builder, value.platform_attitude_deg));
}

ArPlatformInput DecodePlatformInputV3(const fb::ArPlatformInputV3* value) {
  ArPlatformInput result;
  if (value != nullptr) {
    result.platform_position_ecef_m = DecodeRfV2Position(value->platform_position_ecef_m());
    result.platform_velocity_mps = DecodeEcefVelocityV3(value->platform_velocity_mps());
    result.platform_attitude_deg = DecodeEulerAnglesV3(value->platform_attitude_deg());
  }
  return result;
}

flatbuffers::Offset<fb::ArTargetInputV3> EncodeTargetInputV3(
    flatbuffers::FlatBufferBuilder* builder, const ArTargetInput& value) {
  return fb::CreateArTargetInputV3(
      *builder, value.target_id, builder->CreateString(value.target_name), value.position_x,
      value.position_y, value.position_z, value.velocity_x, value.velocity_y, value.velocity_z,
      value.rcs, value.swerling_type);
}

ArTargetInput DecodeTargetInputV3(const fb::ArTargetInputV3* value) {
  ArTargetInput result;
  if (value != nullptr) {
    result.target_id = value->target_id();
    if (value->target_name() != nullptr) {
      result.target_name = value->target_name()->str();
    }
    result.position_x = value->position_x();
    result.position_y = value->position_y();
    result.position_z = value->position_z();
    result.velocity_x = value->velocity_x();
    result.velocity_y = value->velocity_y();
    result.velocity_z = value->velocity_z();
    result.rcs = value->rcs();
    result.swerling_type = value->swerling_type();
  }
  return result;
}

flatbuffers::Offset<fb::ArCycleInputV3> EncodeCycleInputV3(flatbuffers::FlatBufferBuilder* builder,
                                                           const ArCycleInput& value) {
  std::vector<flatbuffers::Offset<fb::ArTargetInputV3>> targets;
  targets.reserve(value.targets.size());
  for (const ArTargetInput& target : value.targets) {
    targets.push_back(EncodeTargetInputV3(builder, target));
  }
  return fb::CreateArCycleInputV3(
      *builder, value.cycle_index, value.cycle_start_time_s, value.dt_sec,
      EncodePlatformInputV3(builder, value.platform), builder->CreateVector(targets),
      EncodeRfV2Scene(builder, value.interference));
}

ArCycleInput DecodeCycleInputV3(const fb::ArCycleInputV3* value) {
  ArCycleInput result;
  if (value != nullptr) {
    result.cycle_index = value->cycle_index();
    result.cycle_start_time_s = value->cycle_start_time_s();
    result.dt_sec = value->dt_sec();
    result.platform = DecodePlatformInputV3(value->platform());
    if (value->targets() != nullptr) {
      result.targets.reserve(value->targets()->size());
      for (const fb::ArTargetInputV3* target : *value->targets()) {
        result.targets.push_back(DecodeTargetInputV3(target));
      }
    }
    result.interference = DecodeRfV2Scene(value->interference());
  }
  return result;
}

flatbuffers::Offset<fb::AssociationQualityMetricsV3> EncodeAssociationQualityMetricsV3(
    flatbuffers::FlatBufferBuilder* builder, const AssociationQualityMetrics& value) {
  return fb::CreateAssociationQualityMetricsV3(
      *builder, static_cast<std::uint64_t>(value.prior_track_count),
      static_cast<std::uint64_t>(value.detection_count),
      static_cast<std::uint64_t>(value.matched_count),
      static_cast<std::uint64_t>(value.new_track_count),
      static_cast<std::uint64_t>(value.missed_track_count), value.match_rate, value.new_track_rate,
      value.missed_track_rate, value.mean_match_cost, value.p95_match_cost,
      value.association_stress);
}

AssociationQualityMetrics DecodeAssociationQualityMetricsV3(
    const fb::AssociationQualityMetricsV3* value) {
  AssociationQualityMetrics result;
  if (value != nullptr) {
    result.prior_track_count = static_cast<std::size_t>(value->prior_track_count());
    result.detection_count = static_cast<std::size_t>(value->detection_count());
    result.matched_count = static_cast<std::size_t>(value->matched_count());
    result.new_track_count = static_cast<std::size_t>(value->new_track_count());
    result.missed_track_count = static_cast<std::size_t>(value->missed_track_count());
    result.match_rate = value->match_rate();
    result.new_track_rate = value->new_track_rate();
    result.missed_track_rate = value->missed_track_rate();
    result.mean_match_cost = value->mean_match_cost();
    result.p95_match_cost = value->p95_match_cost();
    result.association_stress = value->association_stress();
  }
  return result;
}

flatbuffers::Offset<fb::ArCycleResultV3> EncodeCycleResultV3(
    flatbuffers::FlatBufferBuilder* builder, const ArCycleResult& value) {
  std::vector<flatbuffers::Offset<fb::ArInterferenceObservation>> observations;
  observations.reserve(value.interference_observations.size());
  for (const ArInterferenceObservation& observation : value.interference_observations) {
    observations.push_back(EncodeArInterferenceObservation(builder, observation));
  }
  std::vector<flatbuffers::Offset<fb::ArCommandV3>> commands;
  commands.reserve(value.submitted_commands.size());
  for (const ArCommand& command : value.submitted_commands) {
    commands.push_back(fb::CreateArCommandV3(*builder, static_cast<int>(command.type),
                                             static_cast<int>(command.source)));
  }
  // 统一问题列表（规则 14）：单列表编码；entity_index 仅 kSceneEntity 定位有效，
  // 其余定位写 -1 哨兵，decode 期据此还原为 kGlobal（无定位）。
  std::vector<flatbuffers::Offset<fb::ArIssue>> issues;
  issues.reserve(value.issues.size());
  for (const ArIssue& issue : value.issues) {
    const std::size_t encoded_entity_index =
        issue.location.kind == oneq::foundation::ValidationLocationKind::kSceneEntity
            ? issue.location.entity_index
            : static_cast<std::size_t>(-1);
    issues.push_back(fb::CreateArIssue(
        *builder, static_cast<std::int32_t>(issue.severity),
        static_cast<std::int32_t>(issue.phase), builder->CreateString(issue.code),
        builder->CreateString(issue.message), static_cast<std::int32_t>(issue.location.kind),
        static_cast<std::int64_t>(encoded_entity_index), builder->CreateString(issue.field),
        static_cast<std::int32_t>(issue.cause)));
  }
  // 向量创建前置：CreateVector 必须在 CreateArCycleResultV3 打开之前。
  const auto observations_fb = builder->CreateVector(observations);
  const auto commands_fb = builder->CreateVector(commands);
  const auto issues_fb = builder->CreateVector(issues);
  std::vector<flatbuffers::Offset<fb::ArTrackAttributionRecord>> attributions;
  attributions.reserve(value.track_attributions.size());
  for (const ArTrackAttributionRecord& attribution : value.track_attributions) {
    attributions.push_back(fb::CreateArTrackAttributionRecord(
        *builder, attribution.association_key, attribution.external_target_id,
        builder->CreateString(attribution.target_name)));
  }
  const auto attributions_fb = builder->CreateVector(attributions);
  return fb::CreateArCycleResultV3(
      *builder, value.input_cycle_index, static_cast<int>(value.status),
      EncodeTrackOutputFrame(builder, value.output_frame),
      EncodeRfV2Scene(builder, value.emission_frame), static_cast<int>(value.receiver_impairment),
      observations_fb, commands_fb, static_cast<int>(value.abort_reason),
      value.has_control_profile, EncodeArControlProfile(builder, value.control_profile),
      EncodeAssociationQualityMetricsV3(builder, value.association_quality_metrics),
      static_cast<int>(value.applied_decision_source), value.applied_decision_cycle_index,
      value.applied_decision_batch_id, issues_fb,
      static_cast<int>(value.effective_work_mode), value.designation_active,
      value.designated_target_id, value.designation_reverted_to_tws,
      static_cast<int>(value.designation_revert_reason), attributions_fb);
}

bool TryDecodeCycleResultV3(const fb::ArCycleResultV3* value, ArCycleResult* result) {
  if (value == nullptr || result == nullptr) {
    return false;
  }
  ArCycleResult candidate;
  candidate.input_cycle_index = value->input_cycle_index();
  candidate.status = static_cast<ArCycleStatus>(value->status());
  candidate.output_frame = DecodeTrackOutputFrame(value->output_frame());
  candidate.emission_frame = DecodeRfV2Scene(value->emission_frame());
  candidate.receiver_impairment = static_cast<ArReceiverImpairment>(value->receiver_impairment());
  if (value->interference_observations() != nullptr) {
    candidate.interference_observations.reserve(value->interference_observations()->size());
    for (const fb::ArInterferenceObservation* observation : *value->interference_observations()) {
      session::ArInterferenceObservation decoded;
      if (!TryDecodeArInterferenceObservation(observation, &decoded)) {
        return false;
      }
      candidate.interference_observations.push_back(decoded);
    }
  }
  if (value->submitted_commands() != nullptr) {
    candidate.submitted_commands.reserve(value->submitted_commands()->size());
    for (const fb::ArCommandV3* command : *value->submitted_commands()) {
      candidate.submitted_commands.push_back(
          ArCommand(static_cast<ArCommandType>(command->type()),
                    static_cast<ArCommandSource>(command->source())));
    }
  }
  candidate.abort_reason = static_cast<SignalCycleAbortReason>(value->abort_reason());
  candidate.has_control_profile = value->has_control_profile();
  candidate.control_profile = DecodeArControlProfile(value->control_profile());
  candidate.association_quality_metrics =
      DecodeAssociationQualityMetricsV3(value->association_quality_metrics());
  const int applied_source_raw = value->applied_decision_source();
  if (!IsKnownDecisionControlSource(applied_source_raw)) {
    return false;
  }
  candidate.applied_decision_source = static_cast<DecisionControlSource>(applied_source_raw);
  candidate.applied_decision_cycle_index = value->applied_decision_cycle_index();
  candidate.applied_decision_batch_id = value->applied_decision_batch_id();
  const int effective_mode_raw = value->effective_work_mode();
  if (!IsKnownArWorkMode(effective_mode_raw)) {
    return false;
  }
  candidate.effective_work_mode = static_cast<config::ArWorkMode>(effective_mode_raw);
  candidate.designation_active = value->designation_active();
  candidate.designated_target_id = value->designated_target_id();
  candidate.designation_reverted_to_tws = value->designation_reverted_to_tws();
  const int revert_reason_raw = value->revert_reason();
  if (revert_reason_raw < 0 ||
      revert_reason_raw >
          static_cast<int>(session::ArDesignationRevertReason::kAcquisitionTimeout)) {
    return false;
  }
  candidate.designation_revert_reason =
      static_cast<session::ArDesignationRevertReason>(revert_reason_raw);
  // 航迹归属对照表（加性字段）：旧记录缺省时解码为空列表。
  if (value->track_attributions() != nullptr) {
    candidate.track_attributions.reserve(value->track_attributions()->size());
    for (const fb::ArTrackAttributionRecord* attribution : *value->track_attributions()) {
      if (attribution == nullptr) {
        return false;
      }
      ArTrackAttributionRecord decoded;
      decoded.association_key = attribution->association_key();
      decoded.external_target_id = attribution->external_target_id();
      if (attribution->target_name() != nullptr) {
        decoded.target_name = attribution->target_name()->str();
      }
      candidate.track_attributions.push_back(decoded);
    }
  }
  // 统一问题列表（规则 14）：decode 期校验 severity/phase（fail-closed），
  // entity_index 仅在 location_kind==kSceneEntity 且 >=0 时有效，否则还原为 kGlobal。
  if (value->issues() != nullptr) {
    candidate.issues.reserve(value->issues()->size());
    for (const fb::ArIssue* encoded : *value->issues()) {
      if (encoded == nullptr || !IsValidIssueSeverity(encoded->severity()) ||
          !IsValidIssuePhase(encoded->phase())) {
        return false;
      }
      ArIssue issue;
      issue.severity = static_cast<ArIssueSeverity>(encoded->severity());
      issue.phase = static_cast<ArIssuePhase>(encoded->phase());
      if (encoded->code() != nullptr) {
        issue.code = encoded->code()->str();
      }
      if (encoded->message() != nullptr) {
        issue.message = encoded->message()->str();
      }
      if (encoded->entity_index() >= 0 &&
          encoded->location_kind() ==
              static_cast<std::int32_t>(oneq::foundation::ValidationLocationKind::kSceneEntity)) {
        issue.location.kind =
            oneq::foundation::ValidationLocationKind::kSceneEntity;
        issue.location.entity_index = static_cast<std::size_t>(encoded->entity_index());
      } else {
        issue.location.kind = oneq::foundation::ValidationLocationKind::kGlobal;
        issue.location.entity_index = static_cast<std::size_t>(-1);
      }
      if (encoded->field() != nullptr) {
        issue.field = encoded->field()->str();
      }
      if (!IsValidIssueCause(encoded->cause())) {
        return false;
      }
      issue.cause = static_cast<ArIssueCause>(encoded->cause());
      candidate.issues.push_back(std::move(issue));
    }
  }
  *result = candidate;
  return true;
}

flatbuffers::Offset<fb::ArSessionReplayStateV3> EncodeSessionReplayStateV3(
    flatbuffers::FlatBufferBuilder* builder, const ArSessionReplayState& value) {
  return fb::CreateArSessionReplayStateV3(
      *builder, value.has_world_chronology, value.last_world_window_end_s, value.next_emission_id,
      value.successful_prepare_count, value.timing_seed, value.frequency_hop_index,
      value.has_pending_runtime_update, value.pending_execution_config_changed,
      value.pending_environment_scenario_config_changed,
      EncodeDecisionReplayState(builder, value.decision_state));
}

bool TryDecodeSessionReplayStateV3(const fb::ArSessionReplayStateV3* value,
                                   ArSessionReplayState* result) {
  if (value == nullptr || result == nullptr) {
    return false;
  }
  result->has_world_chronology = value->has_world_chronology();
  result->last_world_window_end_s = value->last_world_window_end_s();
  result->next_emission_id = value->next_emission_id();
  result->successful_prepare_count = value->successful_prepare_count();
  result->timing_seed = value->timing_seed();
  result->frequency_hop_index = value->frequency_hop_index();
  result->has_pending_runtime_update = value->has_pending_runtime_update();
  result->pending_execution_config_changed = value->pending_execution_config_changed();
  result->pending_environment_scenario_config_changed =
      value->pending_environment_scenario_config_changed();
  return TryDecodeDecisionReplayState(value->decision_state(), &result->decision_state);
}

template <typename FlatbufferType>
const FlatbufferType* TryGetReplayRoot(const std::string& payload_bytes, const char* payload_name,
                                       std::string* error) {
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = std::string("empty ") + payload_name + " flatbuffers payload";
    }
    return nullptr;
  }
  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  const FlatbufferType* root = flatbuffers::GetRoot<FlatbufferType>(data);
  if (root == nullptr || !root->Verify(verifier)) {
    if (error != nullptr) {
      *error = std::string("invalid ") + payload_name + " flatbuffers payload";
    }
    return nullptr;
  }
  return root;
}

}  // namespace

std::string EncodeCycleInputFlatbuffer(const ArCycleInput& input) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<fb::ArCycleInputV3> root = EncodeCycleInputV3(&builder, input);
  builder.Finish(root, fb::ArCycleInputV3Identifier());
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeCycleInputFlatbuffer(const std::string& payload_bytes, ArCycleInput* input,
                                std::string* error) {
  if (input == nullptr) {
    if (error != nullptr) {
      *error = "null ArCycleInput output";
    }
    return false;
  }
  const fb::ArCycleInputV3* root =
      TryGetReplayRoot<fb::ArCycleInputV3>(payload_bytes, "ArCycleInputV3", error);
  if (root == nullptr) {
    return false;
  }
  if (!flatbuffers::BufferHasIdentifier(payload_bytes.data(), fb::ArCycleInputV3Identifier())) {
    if (error != nullptr) {
      *error = "ArCycleInputV3 payload missing ARC4 file identifier";
    }
    return false;
  }
  *input = DecodeCycleInputV3(root);
  return true;
}

std::string EncodeCycleReplayRecordFlatbuffer(const ArCycleReplayRecord& record) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<fb::ArCycleReplayRecordV3> root =
      fb::CreateArCycleReplayRecordV3(builder, EncodeCycleResultV3(&builder, record.result),
                                      EncodeSessionReplayStateV3(&builder, record.session_state));
  // 规则 15f 收回了 ArCycleResultV3 中段字段（field ID 前移）：record 自此写入 file_identifier，
  // decode 侧校验，无标识符的旧缓冲显式拒绝（参照 RIR replay 的失败闭合范式）。
  builder.Finish(root, fb::ArCycleInputV3Identifier());
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeCycleReplayRecordFlatbuffer(const std::string& payload_bytes,
                                       ArCycleReplayRecord* record, std::string* error) {
  if (record == nullptr) {
    if (error != nullptr) {
      *error = "null ArCycleReplayRecord output";
    }
    return false;
  }
  const fb::ArCycleReplayRecordV3* root =
      TryGetReplayRoot<fb::ArCycleReplayRecordV3>(payload_bytes, "ArCycleReplayRecordV3", error);
  if (root == nullptr || root->result() == nullptr || root->session_state() == nullptr) {
    return false;
  }
  // 规则 15f 落地前的旧 record 未写入 file_identifier（且 result 字段 ID 已前移）：
  // 显式拒绝，不按新布局静默解码。
  if (!flatbuffers::BufferHasIdentifier(payload_bytes.data(), fb::ArCycleInputV3Identifier())) {
    if (error != nullptr) {
      *error = "ArCycleReplayRecordV3 payload missing ARC4 file identifier";
    }
    return false;
  }
  ArCycleReplayRecord candidate;
  if (!TryDecodeCycleResultV3(root->result(), &candidate.result)) {
    if (error != nullptr) {
      *error = "ArCycleReplayRecordV3 contains unknown interference observation enum";
    }
    return false;
  }
  if (!TryDecodeSessionReplayStateV3(root->session_state(), &candidate.session_state)) {
    if (error != nullptr) {
      *error = "ArCycleReplayRecordV3 contains unknown control directive enum";
    }
    return false;
  }
  *record = candidate;
  return true;
}

std::string EncodeArControlProfileFlatbuffer(const session::ArControlProfile& profile) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<fb::ArControlProfilePayload> root =
      fb::CreateArControlProfilePayload(builder, EncodeArControlProfile(&builder, profile));
  builder.Finish(root);
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeArControlProfileFlatbuffer(const std::string& payload_bytes,
                                      session::ArControlProfile* profile, std::string* error) {
  if (profile == nullptr) {
    if (error != nullptr) {
      *error = "null ArControlProfile output";
    }
    return false;
  }
  const fb::ArControlProfilePayload* root =
      TryGetReplayRoot<fb::ArControlProfilePayload>(payload_bytes, "ArControlProfilePayload", error);
  if (root == nullptr) {
    return false;
  }
  *profile = DecodeArControlProfile(root->profile());
  return true;
}

std::string EncodeRuntimeConfigAttemptFlatbuffer(const config::ArRuntimeConfigPatch& patch,
                                                 bool accepted) {
  const std::string patch_payload = EncodeRuntimeConfigPatchFlatbuffer(patch);
  flatbuffers::FlatBufferBuilder builder;
  const auto root = fb::CreateArRuntimeConfigAttemptV2(
      builder,
      builder.CreateVector(reinterpret_cast<const std::uint8_t*>(patch_payload.data()),
                           patch_payload.size()),
      accepted);
  builder.Finish(root);
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeRuntimeConfigAttemptFlatbuffer(const std::string& payload_bytes,
                                          config::ArRuntimeConfigPatch* patch, bool* accepted,
                                          std::string* error) {
  if (patch == nullptr || accepted == nullptr) {
    if (error != nullptr) {
      *error = "null ArRuntimeConfigAttempt output";
    }
    return false;
  }
  const auto* root = TryGetReplayRoot<fb::ArRuntimeConfigAttemptV2>(
      payload_bytes, "ArRuntimeConfigAttemptV2", error);
  if (root == nullptr || root->patch_payload() == nullptr) {
    return false;
  }
  const std::string patch_payload(reinterpret_cast<const char*>(root->patch_payload()->Data()),
                                  root->patch_payload()->size());
  if (!DecodeRuntimeConfigPatchFlatbuffer(patch_payload, patch, error)) {
    return false;
  }
  *accepted = root->accepted();
  return true;
}

std::string EncodeSessionConfigFlatbuffer(const config::ArSessionConfig& config) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<session_fb::ArSessionConfig> root = session_fb::CreateArSessionConfig(
      builder, EncodeSessionDetectionConfig(&builder, config.hardware),
      EncodeSessionOrientation(&builder, config.orientation),
      EncodeSessionMission(&builder, config.mission),
      EncodeSessionPolicyConfig(&builder, config.policy),
      EncodeEnvironmentDefaultConfig(&builder, config.environment), config.sensor_enabled);
  builder.Finish(root, session_fb::ArSessionConfigIdentifier());

  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeSessionConfigFlatbuffer(const std::string& payload_bytes,
                                   config::ArSessionConfig* config, std::string* error) {
  if (config == nullptr) {
    if (error != nullptr) {
      *error = "null config::ArSessionConfig output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty config::ArSessionConfig flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  if (!session_fb::VerifyArSessionConfigBuffer(verifier)) {
    if (error != nullptr) {
      *error = "invalid config::ArSessionConfig flatbuffers payload";
    }
    return false;
  }

  const session_fb::ArSessionConfig* root = session_fb::GetArSessionConfig(data);
  // 先解码到局部对象，枚举逐值校验通过后一次性提交（session_contract 规则 7：
  // 未知值原子拒绝、不得部分修改解码目标）。
  config::ArSessionConfig decoded = *config;
  decoded.hardware = DecodeSessionDetectionConfig(root->hardware_detection());
  decoded.orientation = DecodeSessionOrientation(root->orientation());
  decoded.mission = DecodeSessionMission(root->mission());
  decoded.sensor_enabled = root->sensor_enabled();
  decoded.policy = DecodeSessionPolicyConfig(root->policy());
  decoded.environment = DecodeEnvironmentDefaultConfig(root->environment_default_config());
  if (!IsValidDecodedOrientationEnums(decoded.orientation) ||
      !IsValidDecodedMissionEnums(decoded.mission) ||
      !IsValidDecodedDetectionEnums(decoded.hardware) ||
      !IsValidDecodedEnvironmentEnums(decoded.environment.scenario_config)) {
    if (error != nullptr) {
      *error = "config::ArSessionConfig payload contains unknown enum value";
    }
    return false;
  }
  *config = decoded;
  return true;
}

std::string EncodeRuntimeConfigPatchFlatbuffer(const config::ArRuntimeConfigPatch& patch) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<session_fb::ArRuntimeConfigPatch> root =
      session_fb::CreateArRuntimeConfigPatch(
          builder, patch.has_mission, EncodeSessionMission(&builder, patch.mission),
          patch.has_policy, EncodeSessionPolicyConfig(&builder, patch.policy),
          patch.has_environment,
          EncodeSessionEnvironmentRuntimeConfigPatch(&builder, patch.environment),
          patch.has_work_mode, static_cast<int>(patch.work_mode), patch.has_scan_center_deg,
          EncodeSessionAzEl(&builder, patch.scan_center_deg), patch.has_dwell_center_deg,
          EncodeSessionAzEl(&builder, patch.dwell_center_deg), patch.has_designated_target_id,
          patch.designated_external_target_id, patch.has_designation_duration_cycles,
          patch.designation_duration_cycles, patch.has_commanded_beamwidth_deg,
          EncodeSessionCommandedBeamwidth(&builder, patch.commanded_beamwidth_deg),
          patch.has_commanded_beamwidth_enabled, patch.commanded_beamwidth_enabled,
          patch.has_sensor_enabled, patch.sensor_enabled);
  builder.Finish(root);

  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeRuntimeConfigPatchFlatbuffer(const std::string& payload_bytes,
                                        config::ArRuntimeConfigPatch* patch, std::string* error) {
  if (patch == nullptr) {
    if (error != nullptr) {
      *error = "null ArRuntimeConfigPatch output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty ArRuntimeConfigPatch flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  const session_fb::ArRuntimeConfigPatch* root =
      flatbuffers::GetRoot<session_fb::ArRuntimeConfigPatch>(data);
  if (root == nullptr || !root->Verify(verifier)) {
    if (error != nullptr) {
      *error = "invalid ArRuntimeConfigPatch flatbuffers payload";
    }
    return false;
  }

  // 先解码到局部对象，枚举逐值校验通过后一次性提交（session_contract 规则 7：
  // 未知值原子拒绝、不得部分修改解码目标）。
  config::ArRuntimeConfigPatch decoded = *patch;
  decoded.has_mission = root->has_mission();
  decoded.mission = DecodeSessionMission(root->mission());
  decoded.has_policy = root->has_policy();
  decoded.policy = DecodeSessionPolicyConfig(root->policy());
  decoded.has_environment = root->has_environment();
  decoded.environment = DecodeSessionEnvironmentRuntimeConfigPatch(root->environment());
  decoded.has_work_mode = root->has_work_mode();
  decoded.work_mode = static_cast<config::ArWorkMode>(root->work_mode());
  decoded.has_scan_center_deg = root->has_scan_center_deg();
  decoded.scan_center_deg = DecodeSessionAzEl(root->scan_center_deg());
  decoded.has_dwell_center_deg = root->has_dwell_center_deg();
  decoded.dwell_center_deg = DecodeSessionAzEl(root->dwell_center_deg());
  decoded.has_designated_target_id = root->has_designated_target_id();
  decoded.designated_external_target_id = root->designated_target_id();
  decoded.has_designation_duration_cycles = root->has_designation_duration_cycles();
  decoded.designation_duration_cycles = root->designation_duration_cycles();
  decoded.has_commanded_beamwidth_deg = root->has_commanded_beamwidth_deg();
  decoded.commanded_beamwidth_deg = DecodeSessionCommandedBeamwidth(root->commanded_beamwidth_deg());
  decoded.has_commanded_beamwidth_enabled = root->has_commanded_beamwidth_enabled();
  decoded.commanded_beamwidth_enabled = root->commanded_beamwidth_enabled();
  decoded.has_sensor_enabled = root->has_sensor_enabled();
  decoded.sensor_enabled = root->sensor_enabled();
  if (!IsValidDecodedMissionEnums(decoded.mission) ||
      !IsKnownArWorkMode(static_cast<int>(decoded.work_mode)) ||
      !IsValidDecodedEnvironmentEnums(decoded.environment.scenario_config)) {
    if (error != nullptr) {
      *error = "ArRuntimeConfigPatch payload contains unknown enum value";
    }
    return false;
  }
  *patch = decoded;
  return true;
}

std::string EncodeFailureMarkerFlatbuffer(const oneq::replay::ReplayTraceFailure& failure,
                                          bool has_last_event_sequence,
                                          std::uint64_t last_event_sequence) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<fb::FailureMarker> root = fb::CreateFailureMarkerDirect(
      builder, failure.error_code.c_str(), failure.message.c_str(), failure.location.c_str(),
      failure.has_cycle_index, failure.cycle_index, failure.has_sim_time_sec, failure.sim_time_sec,
      failure.diagnostics_payload.c_str(), has_last_event_sequence, last_event_sequence);
  builder.Finish(root);

  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeFailureMarkerFlatbuffer(const std::string& payload_bytes,
                                   oneq::replay::ReplayTraceFailure* failure, std::string* error) {
  return oneq::common::replay::DecodeFailureMarkerPayload<fb::FailureMarker>(payload_bytes, failure,
                                                                             error);
}

}  // namespace session
}  // namespace airborne_radar
