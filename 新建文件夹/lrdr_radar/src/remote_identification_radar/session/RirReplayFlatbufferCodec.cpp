/*
 * Supplemental architecture notes for acceptance review.
 * Unit: RirReplayFlatbufferCodec.
 * Model: LRDR remote identification radar.
 * Layer: session.
 * File role: executable implementation.
 * Responsibility: public cycle contract, validation, adapters, recording, and replay.
 * Upstream boundary: external targets, platform position, RF scene, and session settings.
 * Downstream boundary: stable output frames, issues, task feedback, and replay records.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

/**
 * @file RirReplayFlatbufferCodec.cpp
 * @brief 远程识别雷达 replay FlatBuffers 编解码实现。
 *
 * 阶段 2-S 起编码 V2 表；旧 V1 记录显式拒绝（破坏性版本，不静默误读）。
 * 编码为字节精确往返（无浮点近似），Decode* 失败返回 false 并回填 error。
 */

#include "remote_identification_radar/session/RirReplayFlatbufferCodec.h"

#include <cstdint>
#include <string>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/electromagnetics/RfScene.h"
#include "common/replay/ReplayFlatbufferCodecSupport.h"
#include "remote_identification_radar/session/generated/rir_replay_generated.h"
#include "remote_identification_radar/session/generated/rir_session_replay_generated.h"

namespace remote_identification_radar {
namespace session {

namespace fb = oneq::replay::remote_identification_radar::fb;
namespace cfb = oneq::replay::remote_identification_radar::session::fb;

namespace {

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

flatbuffers::Offset<fb::RirRecognitionResultV1> EncodeRecognitionResult(
    flatbuffers::FlatBufferBuilder* builder, const RirRecognitionResult& value) {
  return fb::CreateRirRecognitionResultV1(
      *builder, static_cast<int>(value.state), static_cast<int>(value.target_category),
      builder->CreateString(value.target_model), value.confidence, value.best_score,
      value.runner_up_score, value.feature_scores.rcs_similarity, value.feature_scores.rcs_quality,
      value.feature_scores.motion_similarity, value.feature_scores.motion_quality,
      value.feature_scores.polarization_similarity, value.feature_scores.polarization_quality,
      value.feature_scores.range_profile_similarity, value.feature_scores.range_profile_quality,
      value.valid_feature_mask, value.observation_count, value.accumulation_sec,
      builder->CreateString(value.database_version), value.source_cycle_index,
      value.source_batch_id, static_cast<int>(value.ballistic_subclass));
}

bool DecodeRecognitionResult(const fb::RirRecognitionResultV1* value, RirRecognitionResult* out) {
  if (value == nullptr) {
    return false;
  }
  const int state_raw = value->state();
  const int category_raw = value->target_category();
  if (state_raw < static_cast<int>(RirRecognitionState::kDisabled) ||
      state_raw > static_cast<int>(RirRecognitionState::kStale) ||
      category_raw < static_cast<int>(RirRecognitionCategory::kBallistic) ||
      category_raw > static_cast<int>(RirRecognitionCategory::kMissile)) {
    return false;
  }
  const int subclass_raw = value->ballistic_subclass();
  if (subclass_raw < static_cast<int>(RirBallisticSubclass::kUnknown) ||
      subclass_raw > static_cast<int>(RirBallisticSubclass::kDebris)) {
    return false;
  }
  out->state = static_cast<RirRecognitionState>(state_raw);
  out->target_category = static_cast<RirRecognitionCategory>(category_raw);
  out->ballistic_subclass = static_cast<RirBallisticSubclass>(subclass_raw);
  if (value->target_model() != nullptr) {
    out->target_model = value->target_model()->str();
  }
  out->confidence = value->confidence();
  out->best_score = value->best_score();
  out->runner_up_score = value->runner_up_score();
  out->feature_scores.rcs_similarity = value->rcs_similarity();
  out->feature_scores.rcs_quality = value->rcs_quality();
  out->feature_scores.motion_similarity = value->motion_similarity();
  out->feature_scores.motion_quality = value->motion_quality();
  out->feature_scores.polarization_similarity = value->polarization_similarity();
  out->feature_scores.polarization_quality = value->polarization_quality();
  out->feature_scores.range_profile_similarity = value->range_profile_similarity();
  out->feature_scores.range_profile_quality = value->range_profile_quality();
  out->valid_feature_mask = value->valid_feature_mask();
  out->observation_count = value->observation_count();
  out->accumulation_sec = value->accumulation_sec();
  if (value->database_version() != nullptr) {
    out->database_version = value->database_version()->str();
  }
  out->source_cycle_index = value->source_cycle_index();
  out->source_batch_id = value->source_batch_id();
  return true;
}

flatbuffers::Offset<fb::RirFeatureMeasurementRecordV1> EncodeFeatureMeasurement(
    flatbuffers::FlatBufferBuilder* builder, const RirFeatureMeasurementRecord& value) {
  const flatbuffers::Offset<fb::RirRcsFeatureObservationV1> rcs =
      fb::CreateRirRcsFeatureObservationV1(
          *builder, value.features.rcs.valid, value.features.rcs.mean_dbsm,
          value.features.rcs.std_db, value.features.rcs.azimuth_variation_db,
          value.features.rcs.elevation_variation_db, value.features.rcs.peak_to_valley_db,
          value.features.rcs.aspect_coverage_deg, value.features.rcs.quality);
  const flatbuffers::Offset<fb::RirMotionFeatureObservationV1> motion =
      fb::CreateRirMotionFeatureObservationV1(
          *builder, value.features.motion.valid, value.features.motion.speed_m_per_s,
          value.features.motion.altitude_m, value.features.motion.acceleration_m_per_s2,
          value.features.motion.turn_radius_m, value.features.motion.is_straight,
          value.features.motion.quality);
  const flatbuffers::Offset<fb::RirPolarizationFeatureObservationV1> polarization =
      fb::CreateRirPolarizationFeatureObservationV1(
          *builder, value.features.polarization.valid,
          value.features.polarization.energy_difference_db,
          value.features.polarization.relative_difference_db,
          value.features.polarization.energy_sum_db, value.features.polarization.quality);
  const flatbuffers::Offset<fb::RirRangeProfileFeatureObservationV1> range_profile =
      fb::CreateRirRangeProfileFeatureObservationV1(
          *builder, value.features.range_profile.valid, value.features.range_profile.length_m,
          value.features.range_profile.peak_count,
          value.features.range_profile.peak_energy_concentration,
          value.features.range_profile.resolution_m, value.features.range_profile.quality);
  const flatbuffers::Offset<fb::RirPolarizationStatsObservationV1> polarization_stats =
      fb::CreateRirPolarizationStatsObservationV1(
          *builder, value.features.polarization_stats.valid,
          value.features.polarization_stats.determinant_mean,
          value.features.polarization_stats.determinant_std,
          value.features.polarization_stats.span_mean, value.features.polarization_stats.span_std,
          value.features.polarization_stats.depolarization_mean,
          value.features.polarization_stats.depolarization_std,
          value.features.polarization_stats.psi_mean_deg,
          value.features.polarization_stats.psi_std_deg,
          value.features.polarization_stats.tau_mean_deg,
          value.features.polarization_stats.tau_std_deg);
  return fb::CreateRirFeatureMeasurementRecordV1(
      *builder, value.association_key, rcs, motion, polarization, range_profile,
      polarization_stats, value.valid_feature_mask, value.look_az_deg, value.look_el_deg,
      value.range_m,
      value.snr_db, value.dwell_sec, value.bandwidth_hz, value.has_platform_position,
      value.platform_position.x_m, value.platform_position.y_m, value.platform_position.z_m,
      value.cycle_index, value.batch_id);
}

void DecodeFeatureMeasurement(const fb::RirFeatureMeasurementRecordV1* value,
                               RirFeatureMeasurementRecord* out) {
  if (value == nullptr) {
    return;
  }
  out->association_key = value->association_key();
  if (value->rcs() != nullptr) {
    out->features.rcs.valid = value->rcs()->valid();
    out->features.rcs.mean_dbsm = value->rcs()->mean_dbsm();
    out->features.rcs.std_db = value->rcs()->std_db();
    out->features.rcs.azimuth_variation_db = value->rcs()->azimuth_variation_db();
    out->features.rcs.elevation_variation_db = value->rcs()->elevation_variation_db();
    out->features.rcs.peak_to_valley_db = value->rcs()->peak_to_valley_db();
    out->features.rcs.aspect_coverage_deg = value->rcs()->aspect_coverage_deg();
    out->features.rcs.quality = value->rcs()->quality();
  }
  if (value->motion() != nullptr) {
    out->features.motion.valid = value->motion()->valid();
    out->features.motion.speed_m_per_s = value->motion()->speed_m_per_s();
    out->features.motion.altitude_m = value->motion()->altitude_m();
    out->features.motion.acceleration_m_per_s2 = value->motion()->acceleration_m_per_s2();
    out->features.motion.turn_radius_m = value->motion()->turn_radius_m();
    out->features.motion.is_straight = value->motion()->is_straight();
    out->features.motion.quality = value->motion()->quality();
  }
  if (value->polarization() != nullptr) {
    out->features.polarization.valid = value->polarization()->valid();
    out->features.polarization.energy_difference_db =
        value->polarization()->energy_difference_db();
    out->features.polarization.relative_difference_db =
        value->polarization()->relative_difference_db();
    out->features.polarization.energy_sum_db = value->polarization()->energy_sum_db();
    out->features.polarization.quality = value->polarization()->quality();
  }
  if (value->polarization_stats() != nullptr) {
    out->features.polarization_stats.valid = value->polarization_stats()->valid();
    out->features.polarization_stats.determinant_mean =
        value->polarization_stats()->determinant_mean();
    out->features.polarization_stats.determinant_std =
        value->polarization_stats()->determinant_std();
    out->features.polarization_stats.span_mean = value->polarization_stats()->span_mean();
    out->features.polarization_stats.span_std = value->polarization_stats()->span_std();
    out->features.polarization_stats.depolarization_mean =
        value->polarization_stats()->depolarization_mean();
    out->features.polarization_stats.depolarization_std =
        value->polarization_stats()->depolarization_std();
    out->features.polarization_stats.psi_mean_deg = value->polarization_stats()->psi_mean_deg();
    out->features.polarization_stats.psi_std_deg = value->polarization_stats()->psi_std_deg();
    out->features.polarization_stats.tau_mean_deg = value->polarization_stats()->tau_mean_deg();
    out->features.polarization_stats.tau_std_deg = value->polarization_stats()->tau_std_deg();
  }
  if (value->range_profile() != nullptr) {
    out->features.range_profile.valid = value->range_profile()->valid();
    out->features.range_profile.length_m = value->range_profile()->length_m();
    out->features.range_profile.peak_count = value->range_profile()->peak_count();
    out->features.range_profile.peak_energy_concentration =
        value->range_profile()->peak_energy_concentration();
    out->features.range_profile.resolution_m = value->range_profile()->resolution_m();
    out->features.range_profile.quality = value->range_profile()->quality();
  }
  out->valid_feature_mask = value->valid_feature_mask();
  out->look_az_deg = value->look_az_deg();
  out->look_el_deg = value->look_el_deg();
  out->range_m = value->range_m();
  out->snr_db = value->snr_db();
  out->dwell_sec = value->dwell_sec();
  out->bandwidth_hz = value->bandwidth_hz();
  out->has_platform_position = value->has_platform_position();
  out->platform_position.x_m = value->platform_position_x_m();
  out->platform_position.y_m = value->platform_position_y_m();
  out->platform_position.z_m = value->platform_position_z_m();
  out->cycle_index = value->cycle_index();
  out->batch_id = value->batch_id();
}

flatbuffers::Offset<fb::RirTrackAttributionRecordV2> EncodeTrackAttribution(
    flatbuffers::FlatBufferBuilder* builder, const RirTrackAttributionRecord& value) {
  return fb::CreateRirTrackAttributionRecordV2(
      *builder, value.association_key, value.external_target_id,
      builder->CreateString(value.target_name), value.hit_count, value.position_enu_x_m,
      value.position_enu_y_m, value.position_enu_z_m, value.speed_m_per_s,
      static_cast<std::uint8_t>(value.track_status));
}

void DecodeTrackAttribution(const fb::RirTrackAttributionRecordV2* value,
                            RirTrackAttributionRecord* out) {
  if (value == nullptr) {
    return;
  }
  out->association_key = value->association_key();
  out->external_target_id = value->external_target_id();
  if (value->target_name() != nullptr) {
    out->target_name = value->target_name()->str();
  }
  out->track_status = static_cast<RirTrackLifecycleStatus>(value->track_status());
  out->hit_count = value->hit_count();
  out->position_enu_x_m = value->position_enu_x_m();
  out->position_enu_y_m = value->position_enu_y_m();
  out->position_enu_z_m = value->position_enu_z_m();
  out->speed_m_per_s = value->speed_m_per_s();
}

flatbuffers::Offset<fb::RirRecognitionCycleSummaryV2> EncodeSummary(
    flatbuffers::FlatBufferBuilder* builder, const RirRecognitionCycleSummary& value) {
  const flatbuffers::Offset<fb::RirDwellBudgetSummaryV2> dwell_budget =
      fb::CreateRirDwellBudgetSummaryV2(*builder, value.dwell_budget.scheduled_dwell_count,
                                        value.dwell_budget.executed_dwell_count,
                                        value.dwell_budget.dwell_budget_sec,
                                        value.dwell_budget.dwell_consumed_sec);
  return fb::CreateRirRecognitionCycleSummaryV2(
      *builder, value.participating_track_count, value.category_confirmed_count,
      value.model_confirmed_count, value.unknown_count, value.disabled_count,
      value.rcs_availability_rate, value.motion_availability_rate,
      value.polarization_availability_rate, value.range_profile_availability_rate,
      value.mean_confidence, value.mean_first_confirmation_sec, value.has_ground_truth,
      value.category_accuracy, value.model_accuracy, dwell_budget);
}

void DecodeSummary(const fb::RirRecognitionCycleSummaryV2* value, RirRecognitionCycleSummary* out) {
  if (value == nullptr) {
    return;
  }
  out->participating_track_count = value->participating_track_count();
  out->category_confirmed_count = value->category_confirmed_count();
  out->model_confirmed_count = value->model_confirmed_count();
  out->unknown_count = value->unknown_count();
  out->disabled_count = value->disabled_count();
  out->rcs_availability_rate = value->rcs_availability_rate();
  out->motion_availability_rate = value->motion_availability_rate();
  out->polarization_availability_rate = value->polarization_availability_rate();
  out->range_profile_availability_rate = value->range_profile_availability_rate();
  out->mean_confidence = value->mean_confidence();
  out->mean_first_confirmation_sec = value->mean_first_confirmation_sec();
  out->has_ground_truth = value->has_ground_truth();
  out->category_accuracy = value->category_accuracy();
  out->model_accuracy = value->model_accuracy();
  if (value->dwell_budget() != nullptr) {
    out->dwell_budget.scheduled_dwell_count = value->dwell_budget()->scheduled_dwell_count();
    out->dwell_budget.executed_dwell_count = value->dwell_budget()->executed_dwell_count();
    out->dwell_budget.dwell_budget_sec = value->dwell_budget()->dwell_budget_sec();
    out->dwell_budget.dwell_consumed_sec = value->dwell_budget()->dwell_consumed_sec();
  }
}

// ===== 周期输入（rir_replay.fbs 的 RirCycleInput 及其场景目标表）=====

flatbuffers::Offset<fb::RirSceneTarget> EncodeSceneTarget(
    flatbuffers::FlatBufferBuilder* builder, const RirSceneTarget& value) {
  const flatbuffers::Offset<flatbuffers::String> target_name =
      builder->CreateString(value.target_name);

  std::vector<flatbuffers::Offset<fb::RirAspectRcsSample>> aspect_samples;
  aspect_samples.reserve(value.aspect_rcs_samples.size());
  for (std::size_t i = 0U; i < value.aspect_rcs_samples.size(); ++i) {
    const RirAspectRcsSample& sample = value.aspect_rcs_samples[i];
    aspect_samples.push_back(fb::CreateRirAspectRcsSample(
        *builder, sample.aspect_az_deg, sample.aspect_el_deg, sample.rcs_dbsm));
  }
  std::vector<flatbuffers::Offset<fb::RirPolSMatrixSample>> polarization_samples;
  polarization_samples.reserve(value.polarization_samples.size());
  for (std::size_t i = 0U; i < value.polarization_samples.size(); ++i) {
    const RirPolSMatrixSample& sample = value.polarization_samples[i];
    polarization_samples.push_back(fb::CreateRirPolSMatrixSample(
        *builder, sample.aspect_az_deg, sample.aspect_el_deg, sample.hh_amp_db,
        sample.hh_phase_deg, sample.hv_amp_db, sample.hv_phase_deg, sample.vh_amp_db,
        sample.vh_phase_deg, sample.vv_amp_db, sample.vv_phase_deg));
  }
  std::vector<flatbuffers::Offset<fb::RirRangeRcsScatterer>> scatterers;
  scatterers.reserve(value.range_rcs_scatterers.size());
  for (std::size_t i = 0U; i < value.range_rcs_scatterers.size(); ++i) {
    const RirRangeRcsScatterer& scatterer = value.range_rcs_scatterers[i];
    scatterers.push_back(fb::CreateRirRangeRcsScatterer(
        *builder, scatterer.range_offset_m, scatterer.rcs_dbsm, scatterer.channel_1_rcs_dbsm,
        scatterer.channel_2_rcs_dbsm, scatterer.phase_deg, scatterer.fluctuation_std_db));
  }

  return fb::CreateRirSceneTarget(
      *builder, value.external_target_id, target_name, value.position_x, value.position_y,
      value.position_z, value.velocity_x, value.velocity_y, value.velocity_z, value.rcs,
      static_cast<std::uint8_t>(value.target_swerling_type), builder->CreateVector(aspect_samples),
      builder->CreateVector(polarization_samples), builder->CreateVector(scatterers));
}

bool DecodeSceneTarget(const fb::RirSceneTarget* value, RirSceneTarget* out) {
  if (value == nullptr) {
    return false;
  }
  const std::uint8_t swerling_raw = value->target_swerling_type();
  if (swerling_raw > static_cast<std::uint8_t>(RirSwerlingType::kSwerling4)) {
    return false;
  }
  out->external_target_id = value->external_target_id();
  if (value->target_name() != nullptr) {
    out->target_name = value->target_name()->str();
  }
  out->position_x = value->position_x();
  out->position_y = value->position_y();
  out->position_z = value->position_z();
  out->velocity_x = value->velocity_x();
  out->velocity_y = value->velocity_y();
  out->velocity_z = value->velocity_z();
  out->rcs = value->rcs();
  out->target_swerling_type = static_cast<RirSwerlingType>(swerling_raw);
  if (value->aspect_rcs_samples() != nullptr) {
    out->aspect_rcs_samples.reserve(value->aspect_rcs_samples()->size());
    for (const fb::RirAspectRcsSample* sample : *value->aspect_rcs_samples()) {
      if (sample == nullptr) {
        continue;
      }
      RirAspectRcsSample decoded;
      decoded.aspect_az_deg = sample->aspect_az_deg();
      decoded.aspect_el_deg = sample->aspect_el_deg();
      decoded.rcs_dbsm = sample->rcs_dbsm();
      out->aspect_rcs_samples.push_back(decoded);
    }
  }
  if (value->polarization_samples() != nullptr) {
    out->polarization_samples.reserve(value->polarization_samples()->size());
    for (const fb::RirPolSMatrixSample* sample : *value->polarization_samples()) {
      if (sample == nullptr) {
        continue;
      }
      RirPolSMatrixSample decoded;
      decoded.aspect_az_deg = sample->aspect_az_deg();
      decoded.aspect_el_deg = sample->aspect_el_deg();
      decoded.hh_amp_db = sample->hh_amp_db();
      decoded.hh_phase_deg = sample->hh_phase_deg();
      decoded.hv_amp_db = sample->hv_amp_db();
      decoded.hv_phase_deg = sample->hv_phase_deg();
      decoded.vh_amp_db = sample->vh_amp_db();
      decoded.vh_phase_deg = sample->vh_phase_deg();
      decoded.vv_amp_db = sample->vv_amp_db();
      decoded.vv_phase_deg = sample->vv_phase_deg();
      out->polarization_samples.push_back(decoded);
    }
  }
  if (value->range_rcs_scatterers() != nullptr) {
    out->range_rcs_scatterers.reserve(value->range_rcs_scatterers()->size());
    for (const fb::RirRangeRcsScatterer* scatterer : *value->range_rcs_scatterers()) {
      if (scatterer == nullptr) {
        continue;
      }
      RirRangeRcsScatterer decoded;
      decoded.range_offset_m = scatterer->range_offset_m();
      decoded.rcs_dbsm = scatterer->rcs_dbsm();
      decoded.channel_1_rcs_dbsm = scatterer->channel_1_rcs_dbsm();
      decoded.channel_2_rcs_dbsm = scatterer->channel_2_rcs_dbsm();
      decoded.phase_deg = scatterer->phase_deg();
      decoded.fluctuation_std_db = scatterer->fluctuation_std_db();
      out->range_rcs_scatterers.push_back(decoded);
    }
  }
  return true;
}

// ===== 会话配置与运行期补丁（rir_session_replay.fbs）=====

flatbuffers::Offset<cfb::RirAzimuthElevationLimitsDeg> EncodeAzElLimits(
    flatbuffers::FlatBufferBuilder* builder, const config::RirAzimuthElevationLimitsDeg& value) {
  return cfb::CreateRirAzimuthElevationLimitsDeg(*builder, value.az_min_deg, value.az_max_deg,
                                                 value.el_min_deg, value.el_max_deg);
}

flatbuffers::Offset<cfb::RirAzimuthElevationDeg> EncodeAzEl(
    flatbuffers::FlatBufferBuilder* builder, const config::RirAzimuthElevationDeg& value) {
  return cfb::CreateRirAzimuthElevationDeg(*builder, value.az_deg, value.el_deg);
}

void DecodeAzEl(const cfb::RirAzimuthElevationDeg* value, config::RirAzimuthElevationDeg* out) {
  if (value == nullptr) {
    return;
  }
  out->az_deg = value->az_deg();
  out->el_deg = value->el_deg();
}

void DecodeAzElLimits(const cfb::RirAzimuthElevationLimitsDeg* value,
                      config::RirAzimuthElevationLimitsDeg* out) {
  if (value == nullptr) {
    return;
  }
  out->az_min_deg = value->az_min_deg();
  out->az_max_deg = value->az_max_deg();
  out->el_min_deg = value->el_min_deg();
  out->el_max_deg = value->el_max_deg();
}

flatbuffers::Offset<cfb::RirHardwareConfig> EncodeHardwareConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::RirHardwareConfig& value) {
  const flatbuffers::Offset<cfb::RirTransmitterConfig> transmitter =
      cfb::CreateRirTransmitterConfig(
          *builder, value.transmitter.peak_power_w,
          value.transmitter.frequency_hz, value.transmitter.bandwidth_hz,
          value.transmitter.pulse_width_s, value.transmitter.prf_hz,
          value.transmitter.transmit_loss_db);

  const flatbuffers::Offset<cfb::RirAntennaConfig> antenna = cfb::CreateRirAntennaConfig(
      *builder, value.antenna.main_beam_gain_db, value.antenna.nominal_az_beamwidth_deg,
      value.antenna.nominal_el_beamwidth_deg, value.antenna.antenna_length_m,
      value.antenna.antenna_width_m, static_cast<int>(value.antenna.model_type),
      value.antenna.max_sidelobe_level_db, value.antenna.backlobe_level_db,
      value.antenna.scan_loss_coeff_db_per_deg2, value.antenna.max_scan_loss_db,
      EncodeAzEl(builder, value.antenna.boresight_offset_deg));

  const flatbuffers::Offset<cfb::RirReceiverConfig> receiver = cfb::CreateRirReceiverConfig(
      *builder, value.receiver.noise_figure_db,
      value.receiver.receive_loss_db, value.receiver.cross_polarization_isolation_db,
      value.receiver.maximum_linear_input_power_w,
      static_cast<int>(value.receiver.scene_polarization));

  const flatbuffers::Offset<cfb::RirRcsPhysicsConfig> rcs_physics =
      cfb::CreateRirRcsPhysicsConfig(
          *builder, value.rcs_physics.enable_physical_rcs, value.rcs_physics.physics_mix_ratio,
          value.rcs_physics.cylinder_weight, value.rcs_physics.min_equivalent_radius_m,
          value.rcs_physics.max_equivalent_radius_m, value.rcs_physics.min_rcs_m2,
          value.rcs_physics.max_rcs_m2, value.rcs_physics.bistatic_psi_offset_deg);
  const flatbuffers::Offset<cfb::RirSignalProcessingConfig> signal_processing =
      cfb::CreateRirSignalProcessingConfig(
          *builder, value.signal_processing.target_processing_gain_db,
          value.signal_processing.noise_processing_gain_db,
          value.signal_processing.clutter_suppression_gain_db,
          value.signal_processing.jamming_suppression_gain_db);

  return cfb::CreateRirHardwareConfig(*builder, transmitter, antenna, receiver, rcs_physics,
                                      signal_processing);
}

void DecodeHardwareConfig(const cfb::RirHardwareConfig* value, config::RirHardwareConfig* out) {
  if (value == nullptr) {
    return;
  }
  if (value->transmitter() != nullptr) {
    const cfb::RirTransmitterConfig* transmitter = value->transmitter();
    out->transmitter.peak_power_w = transmitter->peak_power_w();
    out->transmitter.frequency_hz = transmitter->frequency_hz();
    out->transmitter.bandwidth_hz = transmitter->bandwidth_hz();
    out->transmitter.pulse_width_s = transmitter->pulse_width_s();
    out->transmitter.prf_hz = transmitter->prf_hz();
    out->transmitter.transmit_loss_db = transmitter->transmit_loss_db();
  }
  if (value->antenna() != nullptr) {
    const cfb::RirAntennaConfig* antenna = value->antenna();
    out->antenna.main_beam_gain_db = antenna->main_beam_gain_db();
    out->antenna.nominal_az_beamwidth_deg = antenna->nominal_az_beamwidth_deg();
    out->antenna.nominal_el_beamwidth_deg = antenna->nominal_el_beamwidth_deg();
    out->antenna.antenna_length_m = antenna->antenna_length_m();
    out->antenna.antenna_width_m = antenna->antenna_width_m();
    out->antenna.model_type =
        static_cast<config::hardware::RirAntennaPatternModelType>(antenna->model_type());
    out->antenna.max_sidelobe_level_db = antenna->max_sidelobe_level_db();
    out->antenna.backlobe_level_db = antenna->backlobe_level_db();
    out->antenna.scan_loss_coeff_db_per_deg2 = antenna->scan_loss_coeff_db_per_deg2();
    out->antenna.max_scan_loss_db = antenna->max_scan_loss_db();
    DecodeAzEl(antenna->boresight_offset_deg(), &out->antenna.boresight_offset_deg);
  }
  if (value->receiver() != nullptr) {
    const cfb::RirReceiverConfig* receiver = value->receiver();
    out->receiver.noise_figure_db = receiver->noise_figure_db();
    out->receiver.receive_loss_db = receiver->receive_loss_db();
    out->receiver.cross_polarization_isolation_db =
        receiver->cross_polarization_isolation_db();
    out->receiver.maximum_linear_input_power_w = receiver->maximum_linear_input_power_w();
    out->receiver.scene_polarization =
        static_cast<oneq::electromagnetics::RfScenePolarization>(receiver->scene_polarization());
  }
  if (value->rcs_physics() != nullptr) {
    const cfb::RirRcsPhysicsConfig* rcs_physics = value->rcs_physics();
    out->rcs_physics.enable_physical_rcs = rcs_physics->enable_physical_rcs();
    out->rcs_physics.physics_mix_ratio = rcs_physics->physics_mix_ratio();
    out->rcs_physics.cylinder_weight = rcs_physics->cylinder_weight();
    out->rcs_physics.min_equivalent_radius_m = rcs_physics->min_equivalent_radius_m();
    out->rcs_physics.max_equivalent_radius_m = rcs_physics->max_equivalent_radius_m();
    out->rcs_physics.min_rcs_m2 = rcs_physics->min_rcs_m2();
    out->rcs_physics.max_rcs_m2 = rcs_physics->max_rcs_m2();
    out->rcs_physics.bistatic_psi_offset_deg = rcs_physics->bistatic_psi_offset_deg();
  }
  if (value->signal_processing() != nullptr) {
    const cfb::RirSignalProcessingConfig* signal_processing = value->signal_processing();
    out->signal_processing.target_processing_gain_db =
        signal_processing->target_processing_gain_db();
    out->signal_processing.noise_processing_gain_db =
        signal_processing->noise_processing_gain_db();
    out->signal_processing.clutter_suppression_gain_db =
        signal_processing->clutter_suppression_gain_db();
    out->signal_processing.jamming_suppression_gain_db =
        signal_processing->jamming_suppression_gain_db();
  }
}

flatbuffers::Offset<cfb::RirMissionConfig> EncodeMissionConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::RirMissionConfig& value) {
  return cfb::CreateRirMissionConfig(
      *builder, static_cast<int>(value.work_mode), EncodeAzEl(builder, value.scan_center_deg),
      value.max_range_m, value.recognition_dwell_sec,
      static_cast<int>(value.scan_start_position), static_cast<int>(value.scan_sequence),
      value.step_scale, EncodeAzElLimits(builder, value.scan_window_deg));
}

void DecodeMissionConfig(const cfb::RirMissionConfig* value, config::RirMissionConfig* out) {
  if (value == nullptr) {
    return;
  }
  out->work_mode = static_cast<config::RirWorkMode>(value->work_mode());
  DecodeAzEl(value->scan_center_deg(), &out->scan_center_deg);
  out->max_range_m = value->max_range_m();
  out->recognition_dwell_sec = value->recognition_dwell_sec();
  out->scan_start_position =
      static_cast<oneq::foundation::ScanStartPosition>(value->scan_start_position());
  out->scan_sequence = static_cast<oneq::foundation::ScanSequence>(value->scan_sequence());
  out->step_scale = value->step_scale();
  // 任务扫描子窗（加性字段）：旧回放无此表时保留默认无界，语义不回归。
  if (value->scan_window_deg() != nullptr) {
    const cfb::RirAzimuthElevationLimitsDeg* window = value->scan_window_deg();
    out->scan_window_deg.az_min_deg = window->az_min_deg();
    out->scan_window_deg.az_max_deg = window->az_max_deg();
    out->scan_window_deg.el_min_deg = window->el_min_deg();
    out->scan_window_deg.el_max_deg = window->el_max_deg();
  }
}

flatbuffers::Offset<cfb::RirPolicyConfig> EncodePolicyConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::RirPolicyConfig& value) {
  const flatbuffers::Offset<cfb::RirDetectionPolicyConfig> detection =
      cfb::CreateRirDetectionPolicyConfig(
          *builder, value.detection.cfar_pfa, value.detection.min_snr_db,
          value.detection.min_detection_margin_db, value.detection.pulse_count,
          value.detection.random_seed, static_cast<int>(value.detection.gate_mode));
  const flatbuffers::Offset<cfb::RirAssociationPolicyConfig> association =
      cfb::CreateRirAssociationPolicyConfig(*builder, value.association.distance_gate_sigma);
  const flatbuffers::Offset<cfb::RirTrackingPolicyConfig> tracking =
      cfb::CreateRirTrackingPolicyConfig(*builder, value.tracking.kalman_noise_diff_coeff);
  const flatbuffers::Offset<cfb::RirLifecyclePolicyConfig> lifecycle =
      cfb::CreateRirLifecyclePolicyConfig(
          *builder, value.lifecycle.confirm_hits, value.lifecycle.max_miss_before_lost,
          value.lifecycle.max_lost_cycles, value.lifecycle.enable_imm_lifecycle,
          value.lifecycle.model_count_hint);
  const flatbuffers::Offset<cfb::RirRecognitionFeatureWeights> feature_weights =
      cfb::CreateRirRecognitionFeatureWeights(
          *builder, value.recognition.feature_weights.rcs_weight,
          value.recognition.feature_weights.motion_weight,
          value.recognition.feature_weights.polarization_weight,
          value.recognition.feature_weights.range_profile_weight);
  const flatbuffers::Offset<cfb::RirRecognitionPolicy> recognition =
      cfb::CreateRirRecognitionPolicy(
          *builder, value.recognition.enabled, value.recognition.min_confirmed_hits,
          value.recognition.accumulation_window_sec, value.recognition.min_observation_count,
          value.recognition.acceptance_score, value.recognition.minimum_margin,
          value.recognition.result_hold_sec, feature_weights,
          builder->CreateString(value.recognition.database_path));
  return cfb::CreateRirPolicyConfig(*builder, detection, association, tracking, lifecycle,
                                    recognition);
}

void DecodePolicyConfig(const cfb::RirPolicyConfig* value, config::RirPolicyConfig* out) {
  if (value == nullptr) {
    return;
  }
  if (value->detection() != nullptr) {
    const cfb::RirDetectionPolicyConfig* detection = value->detection();
    out->detection.cfar_pfa = detection->cfar_pfa();
    out->detection.min_snr_db = detection->min_snr_db();
    out->detection.min_detection_margin_db = detection->min_detection_margin_db();
    out->detection.pulse_count = detection->pulse_count();
    out->detection.random_seed = detection->random_seed();
    out->detection.gate_mode =
        static_cast<config::detection::RirDetectionGateMode>(detection->gate_mode());
  }
  if (value->association() != nullptr) {
    out->association.distance_gate_sigma = value->association()->distance_gate_sigma();
  }
  if (value->tracking() != nullptr) {
    out->tracking.kalman_noise_diff_coeff = value->tracking()->kalman_noise_diff_coeff();
  }
  if (value->lifecycle() != nullptr) {
    const cfb::RirLifecyclePolicyConfig* lifecycle = value->lifecycle();
    out->lifecycle.confirm_hits = lifecycle->confirm_hits();
    out->lifecycle.max_miss_before_lost = lifecycle->max_miss_before_lost();
    out->lifecycle.max_lost_cycles = lifecycle->max_lost_cycles();
    out->lifecycle.enable_imm_lifecycle = lifecycle->enable_imm_lifecycle();
    out->lifecycle.model_count_hint = lifecycle->model_count_hint();
  }
  if (value->recognition() != nullptr) {
    const cfb::RirRecognitionPolicy* recognition = value->recognition();
    out->recognition.enabled = recognition->enabled();
    out->recognition.min_confirmed_hits = recognition->min_confirmed_hits();
    out->recognition.accumulation_window_sec = recognition->accumulation_window_sec();
    out->recognition.min_observation_count = recognition->min_observation_count();
    out->recognition.acceptance_score = recognition->acceptance_score();
    out->recognition.minimum_margin = recognition->minimum_margin();
    out->recognition.result_hold_sec = recognition->result_hold_sec();
    if (recognition->feature_weights() != nullptr) {
      out->recognition.feature_weights.rcs_weight =
          recognition->feature_weights()->rcs_weight();
      out->recognition.feature_weights.motion_weight =
          recognition->feature_weights()->motion_weight();
      out->recognition.feature_weights.polarization_weight =
          recognition->feature_weights()->polarization_weight();
      out->recognition.feature_weights.range_profile_weight =
          recognition->feature_weights()->range_profile_weight();
    }
    out->recognition.database_path =
        recognition->database_path() != nullptr ? recognition->database_path()->str() : "";
  }
}

flatbuffers::Offset<cfb::RirEnvironmentConfig> EncodeEnvironmentConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::RirEnvironmentConfig& value) {
  const flatbuffers::Offset<cfb::RirAtmosphericObservation> atmospheric =
      cfb::CreateRirAtmosphericObservation(
          *builder, value.atmospheric_physics.enable_physical_model,
          value.atmospheric_physics.pressure_hpa, value.atmospheric_physics.temperature_k,
          value.atmospheric_physics.relative_humidity);
  return cfb::CreateRirEnvironmentConfig(
      *builder, value.weather_attenuation_db,
      static_cast<int>(value.vegetation_cover_profile), atmospheric);
}

void DecodeEnvironmentConfig(const cfb::RirEnvironmentConfig* value,
                             config::RirEnvironmentConfig* out) {
  if (value == nullptr) {
    return;
  }
  out->weather_attenuation_db = value->weather_attenuation_db();
  out->vegetation_cover_profile =
      static_cast<config::RirVegetationCoverProfile>(value->vegetation_cover_profile());
  if (value->atmospheric_physics() != nullptr) {
    out->atmospheric_physics.enable_physical_model =
        value->atmospheric_physics()->enable_physical_model();
    out->atmospheric_physics.pressure_hpa = value->atmospheric_physics()->pressure_hpa();
    out->atmospheric_physics.temperature_k = value->atmospheric_physics()->temperature_k();
    out->atmospheric_physics.relative_humidity =
        value->atmospheric_physics()->relative_humidity();
  }
}

}  // namespace

std::string EncodeCycleReplayRecordFlatbuffer(const RirCycleReplayRecord& record) {
  flatbuffers::FlatBufferBuilder builder(1024);

  std::vector<flatbuffers::Offset<fb::RirTrackRecognitionOutputV1>> outputs;
  outputs.reserve(record.result.output_frame.recognition_outputs.size());
  for (std::size_t i = 0U; i < record.result.output_frame.recognition_outputs.size(); ++i) {
    const RirTrackRecognitionOutput& output = record.result.output_frame.recognition_outputs[i];
    outputs.push_back(fb::CreateRirTrackRecognitionOutputV1(
        builder, output.association_key, EncodeRecognitionResult(&builder, output.result)));
  }
  // 出口①特征量测（加性向量，总是写入——空列表编码为存在但为空）。
  std::vector<flatbuffers::Offset<fb::RirFeatureMeasurementRecordV1>> feature_measurements;
  feature_measurements.reserve(record.result.output_frame.feature_measurements.size());
  for (std::size_t i = 0U; i < record.result.output_frame.feature_measurements.size(); ++i) {
    feature_measurements.push_back(
        EncodeFeatureMeasurement(&builder, record.result.output_frame.feature_measurements[i]));
  }
  const flatbuffers::Offset<fb::RirOutputFrameV1> output_frame = fb::CreateRirOutputFrameV1(
      builder, record.result.output_frame.input_cycle_index, record.result.output_frame.batch_id,
      builder.CreateVector(outputs), builder.CreateVector(feature_measurements));

  flatbuffers::Offset<fb::RirRecognitionCycleSummaryV2> summary_offset;
  if (record.result.has_recognition_summary) {
    summary_offset = EncodeSummary(&builder, record.result.recognition_summary);
  }

  // 航迹归属视图（结果层加性向量，总是写入）。
  std::vector<flatbuffers::Offset<fb::RirTrackAttributionRecordV2>> attributions;
  attributions.reserve(record.result.track_attributions.size());
  for (std::size_t i = 0U; i < record.result.track_attributions.size(); ++i) {
    attributions.push_back(
        EncodeTrackAttribution(&builder, record.result.track_attributions[i]));
  }

  const flatbuffers::Offset<fb::RirCycleResultV2> result = fb::CreateRirCycleResultV2(
      builder, record.result.input_cycle_index, static_cast<int>(record.result.status),
      static_cast<int>(record.result.abort_reason), output_frame,
      record.result.has_recognition_summary, summary_offset, record.result.designated_target_id,
      record.result.designation_active, record.result.designation_reverted_to_scan,
      static_cast<int>(record.result.designation_revert_reason),
      record.result.dwell_center_deg.az_deg, record.result.dwell_center_deg.el_deg,
      builder.CreateVector(attributions), EncodeRfV2Scene(&builder, record.result.emission_frame));

  const flatbuffers::Offset<fb::RirSessionReplayStateV2> session_state =
      fb::CreateRirSessionReplayStateV2(
          builder, builder.CreateString(record.session_state.active_database_version),
          record.session_state.detection_random_seed);

  const flatbuffers::Offset<fb::RirCycleReplayRecordV3> root =
      fb::CreateRirCycleReplayRecordV3(builder, result, session_state);
  builder.Finish(root, fb::RirCycleReplayRecordV3Identifier());
  return std::string(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
}

bool DecodeCycleReplayRecordFlatbuffer(const std::string& payload_bytes,
                                       RirCycleReplayRecord* record, std::string* error) {
  if (record == nullptr) {
    if (error != nullptr) {
      *error = "record output must be non-null";
    }
    return false;
  }
  const std::uint8_t* payload = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  if (!flatbuffers::BufferHasIdentifier(payload, fb::RirCycleReplayRecordV3Identifier())) {
    if (error != nullptr) {
      *error =
          "rir_replay record is not supported: unified rf identity uses the "
          "destructive v4 schema (RIR4)";
    }
    return false;
  }
  flatbuffers::Verifier verifier(payload, payload_bytes.size());
  if (!fb::VerifyRirCycleReplayRecordV3Buffer(verifier)) {
    if (error != nullptr) {
      *error = "flatbuffer verification failed";
    }
    return false;
  }
  const fb::RirCycleReplayRecordV3* root = fb::GetRirCycleReplayRecordV3(payload_bytes.data());
  if (root == nullptr) {
    if (error != nullptr) {
      *error = "root table missing";
    }
    return false;
  }
  RirCycleReplayRecord candidate;
  const fb::RirCycleResultV2* result = root->result();
  if (result == nullptr) {
    if (error != nullptr) {
      *error = "result table missing";
    }
    return false;
  }
  candidate.result.input_cycle_index = result->input_cycle_index();
  const int status_raw = result->status();
  if (status_raw < static_cast<int>(RirCycleStatus::kCompleted) ||
      status_raw > static_cast<int>(RirCycleStatus::kRejectedExecution)) {
    if (error != nullptr) {
      *error = "invalid cycle status";
    }
    return false;
  }
  candidate.result.status = static_cast<RirCycleStatus>(status_raw);
  candidate.result.abort_reason = static_cast<RirCycleAbortReason>(result->abort_reason());
  candidate.result.has_recognition_summary = result->has_recognition_summary();
  DecodeSummary(result->recognition_summary(), &candidate.result.recognition_summary);
  candidate.result.designated_target_id = result->designated_target_id();
  candidate.result.designation_active = result->designation_active();
  candidate.result.designation_reverted_to_scan = result->designation_reverted_to_scan();
  const int revert_reason_raw = result->designation_revert_reason();
  if (revert_reason_raw <
          static_cast<int>(RirDesignationRevertReason::kNone) ||
      revert_reason_raw > static_cast<int>(RirDesignationRevertReason::kOutsideSteerableVolume)) {
    if (error != nullptr) {
      *error = "invalid designation revert reason";
    }
    return false;
  }
  candidate.result.designation_revert_reason =
      static_cast<RirDesignationRevertReason>(revert_reason_raw);
  candidate.result.dwell_center_deg.az_deg = result->dwell_center_az_deg();
  candidate.result.dwell_center_deg.el_deg = result->dwell_center_el_deg();
  candidate.result.emission_frame = DecodeRfV2Scene(result->emission_frame());

  const fb::RirOutputFrameV1* output_frame = result->output_frame();
  if (output_frame != nullptr) {
    candidate.result.output_frame.input_cycle_index = output_frame->input_cycle_index();
    candidate.result.output_frame.batch_id = output_frame->batch_id();
    const auto* outputs = output_frame->recognition_outputs();
    if (outputs != nullptr) {
      candidate.result.output_frame.recognition_outputs.reserve(outputs->size());
      for (const fb::RirTrackRecognitionOutputV1* output : *outputs) {
        if (output == nullptr) {
          continue;
        }
        RirTrackRecognitionOutput decoded;
        decoded.association_key = output->association_key();
        if (!DecodeRecognitionResult(output->result(), &decoded.result)) {
          if (error != nullptr) {
            *error = "invalid recognition result";
          }
          return false;
        }
        candidate.result.output_frame.recognition_outputs.push_back(decoded);
      }
    }
    // 出口①特征量测：旧记录字段缺席（null）→ 保持空（加性兼容）。
    const auto* feature_measurements = output_frame->feature_measurements();
    if (feature_measurements != nullptr) {
      candidate.result.output_frame.feature_measurements.reserve(feature_measurements->size());
      for (const fb::RirFeatureMeasurementRecordV1* measurement : *feature_measurements) {
        if (measurement == nullptr) {
          continue;
        }
        RirFeatureMeasurementRecord decoded;
        DecodeFeatureMeasurement(measurement, &decoded);
        candidate.result.output_frame.feature_measurements.push_back(decoded);
      }
    }
  }

  // 航迹归属视图：旧记录字段缺席（null）→ 保持空。
  const auto* attributions = result->track_attributions();
  if (attributions != nullptr) {
    candidate.result.track_attributions.reserve(attributions->size());
    for (const fb::RirTrackAttributionRecordV2* attribution : *attributions) {
      if (attribution == nullptr) {
        continue;
      }
      RirTrackAttributionRecord decoded;
      DecodeTrackAttribution(attribution, &decoded);
      candidate.result.track_attributions.push_back(decoded);
    }
  }

  const fb::RirSessionReplayStateV2* session_state = root->session_state();
  if (session_state != nullptr) {
    if (session_state->active_database_version() != nullptr) {
      candidate.session_state.active_database_version =
          session_state->active_database_version()->str();
    }
    candidate.session_state.detection_random_seed = session_state->detection_random_seed();
  }

  *record = std::move(candidate);
  return true;
}

std::string EncodeRirCycleInput(const RirCycleInput& input) {
  flatbuffers::FlatBufferBuilder builder(1024);
  std::vector<flatbuffers::Offset<fb::RirSceneTarget>> scene_targets;
  scene_targets.reserve(input.scene_targets.size());
  for (std::size_t i = 0U; i < input.scene_targets.size(); ++i) {
    scene_targets.push_back(EncodeSceneTarget(&builder, input.scene_targets[i]));
  }
  const flatbuffers::Offset<fb::RirCycleInput> root = fb::CreateRirCycleInput(
      builder, input.input_cycle_index, input.dt_sec, input.sim_time_sec,
      EncodeRfV2Position(&builder, input.platform_position),
      builder.CreateVector(scene_targets), EncodeRfV2Scene(&builder, input.rf_scene));
  // RirCycleInput 不是 rir_replay.fbs 的 root_type：不写 file_identifier，
  // 解码侧以 GetRoot<RirCycleInput> + Verifier 校验（与 EOS/SAR 同口径）。
  builder.Finish(root);
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeRirCycleInput(const std::string& payload_bytes, RirCycleInput* input,
                         std::string* error) {
  if (input == nullptr) {
    if (error != nullptr) {
      *error = "input output must be non-null";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty RirCycleInput flatbuffers payload";
    }
    return false;
  }
  const std::uint8_t* payload = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(payload, payload_bytes.size());
  if (!verifier.VerifyBuffer<fb::RirCycleInput>()) {
    if (error != nullptr) {
      *error = "flatbuffer verification failed for RirCycleInput";
    }
    return false;
  }
  const fb::RirCycleInput* root = flatbuffers::GetRoot<fb::RirCycleInput>(payload);
  if (root == nullptr) {
    if (error != nullptr) {
      *error = "RirCycleInput root table missing";
    }
    return false;
  }
  RirCycleInput candidate;
  candidate.input_cycle_index = root->input_cycle_index();
  candidate.dt_sec = root->dt_sec();
  candidate.sim_time_sec = root->sim_time_sec();
  candidate.platform_position = DecodeRfV2Position(root->platform_position());
  if (root->scene_targets() != nullptr) {
    candidate.scene_targets.reserve(root->scene_targets()->size());
    for (const fb::RirSceneTarget* target : *root->scene_targets()) {
      RirSceneTarget decoded;
      if (!DecodeSceneTarget(target, &decoded)) {
        if (error != nullptr) {
          *error = "invalid RirCycleInput scene target";
        }
        return false;
      }
      candidate.scene_targets.push_back(decoded);
    }
  }
  candidate.rf_scene = DecodeRfV2Scene(root->rf_scene());
  *input = candidate;
  return true;
}

std::string EncodeRirSessionConfig(const config::RirSessionConfig& config) {
  flatbuffers::FlatBufferBuilder builder(1024);
  const flatbuffers::Offset<cfb::RirHardwareConfig> hardware =
      EncodeHardwareConfig(&builder, config.hardware);
  const flatbuffers::Offset<cfb::RirAzimuthElevationLimitsDeg> orientation =
      EncodeAzElLimits(&builder, config.orientation);
  const flatbuffers::Offset<cfb::RirMissionConfig> mission =
      EncodeMissionConfig(&builder, config.mission);
  const flatbuffers::Offset<cfb::RirPolicyConfig> policy =
      EncodePolicyConfig(&builder, config.policy);
  const flatbuffers::Offset<cfb::RirEnvironmentConfig> environment =
      EncodeEnvironmentConfig(&builder, config.environment);
  const flatbuffers::Offset<cfb::RirSessionConfig> root = cfb::CreateRirSessionConfig(
      builder, hardware, orientation, mission, policy, environment,
      config.sensor_enabled);
  builder.Finish(root, cfb::RirSessionConfigIdentifier());
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeRirSessionConfig(const std::string& payload_bytes, config::RirSessionConfig* config,
                            std::string* error) {
  if (config == nullptr) {
    if (error != nullptr) {
      *error = "config output must be non-null";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty RirSessionConfig flatbuffers payload";
    }
    return false;
  }
  const std::uint8_t* payload = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(payload, payload_bytes.size());
  if (!cfb::VerifyRirSessionConfigBuffer(verifier)) {
    if (error != nullptr) {
      *error = "flatbuffer verification failed for RirSessionConfig";
    }
    return false;
  }
  const cfb::RirSessionConfig* root = cfb::GetRirSessionConfig(payload);
  if (root == nullptr) {
    if (error != nullptr) {
      *error = "RirSessionConfig root table missing";
    }
    return false;
  }
  config::RirSessionConfig candidate;
  DecodeHardwareConfig(root->hardware(), &candidate.hardware);
  if (root->orientation() != nullptr) {
    DecodeAzElLimits(root->orientation(), &candidate.orientation);
  }
  DecodeMissionConfig(root->mission(), &candidate.mission);
  DecodePolicyConfig(root->policy(), &candidate.policy);
  DecodeEnvironmentConfig(root->environment(), &candidate.environment);
  candidate.sensor_enabled = root->sensor_enabled();
  *config = candidate;
  return true;
}

std::string EncodeRirRuntimeConfigPatch(const config::RirRuntimeConfigPatch& patch) {
  flatbuffers::FlatBufferBuilder builder(1024);
  const flatbuffers::Offset<cfb::RirMissionConfig> mission =
      EncodeMissionConfig(&builder, patch.mission);
  const flatbuffers::Offset<cfb::RirAzimuthElevationDeg> scan_center =
      EncodeAzEl(&builder, patch.scan_center_deg);
  const flatbuffers::Offset<cfb::RirPolicyConfig> policy =
      EncodePolicyConfig(&builder, patch.policy);
  const flatbuffers::Offset<cfb::RirEnvironmentConfig> environment =
      EncodeEnvironmentConfig(&builder, patch.environment);
  const flatbuffers::Offset<cfb::RirRuntimeConfigPatch> root =
      cfb::CreateRirRuntimeConfigPatch(
          builder, patch.has_mission, mission, patch.has_work_mode,
          static_cast<int>(patch.work_mode), patch.has_scan_center, scan_center,
          patch.has_policy, policy, patch.has_environment, environment,
          patch.has_sensor_enabled, patch.sensor_enabled, patch.has_designated_target_id,
          patch.designated_external_target_id, patch.has_designation_duration_cycles,
          patch.designation_duration_cycles);
  builder.Finish(root);
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeRirRuntimeConfigPatch(const std::string& payload_bytes,
                                 config::RirRuntimeConfigPatch* patch, std::string* error) {
  if (patch == nullptr) {
    if (error != nullptr) {
      *error = "patch output must be non-null";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty RirRuntimeConfigPatch flatbuffers payload";
    }
    return false;
  }
  const std::uint8_t* payload = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(payload, payload_bytes.size());
  if (!verifier.VerifyBuffer<cfb::RirRuntimeConfigPatch>()) {
    if (error != nullptr) {
      *error = "flatbuffer verification failed for RirRuntimeConfigPatch";
    }
    return false;
  }
  const cfb::RirRuntimeConfigPatch* root =
      flatbuffers::GetRoot<cfb::RirRuntimeConfigPatch>(payload);
  if (root == nullptr) {
    if (error != nullptr) {
      *error = "RirRuntimeConfigPatch root table missing";
    }
    return false;
  }
  config::RirRuntimeConfigPatch candidate;
  candidate.has_mission = root->has_mission();
  DecodeMissionConfig(root->mission(), &candidate.mission);
  candidate.has_work_mode = root->has_work_mode();
  candidate.work_mode = static_cast<config::RirWorkMode>(root->work_mode());
  candidate.has_scan_center = root->has_scan_center();
  DecodeAzEl(root->scan_center_deg(), &candidate.scan_center_deg);
  candidate.has_policy = root->has_policy();
  DecodePolicyConfig(root->policy(), &candidate.policy);
  candidate.has_environment = root->has_environment();
  DecodeEnvironmentConfig(root->environment(), &candidate.environment);
  candidate.has_sensor_enabled = root->has_sensor_enabled();
  candidate.sensor_enabled = root->sensor_enabled();
  candidate.has_designated_target_id = root->has_designated_target_id();
  candidate.designated_external_target_id = root->designated_external_target_id();
  candidate.has_designation_duration_cycles = root->has_designation_duration_cycles();
  candidate.designation_duration_cycles = root->designation_duration_cycles();
  *patch = candidate;
  return true;
}

std::string EncodeRirFailureMarker(const oneq::replay::ReplayTraceFailure& failure) {
  flatbuffers::FlatBufferBuilder builder(512);
  const flatbuffers::Offset<cfb::RirFailureMarker> root = cfb::CreateRirFailureMarkerDirect(
      builder, failure.error_code.c_str(), failure.message.c_str(), failure.location.c_str(),
      failure.has_cycle_index, failure.cycle_index, failure.has_sim_time_sec,
      failure.sim_time_sec, failure.diagnostics_payload.c_str(), false, 0U);
  builder.Finish(root);
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeRirFailureMarker(const std::string& payload_bytes,
                            oneq::replay::ReplayTraceFailure* failure, std::string* error) {
  return oneq::common::replay::DecodeFailureMarkerPayload<cfb::RirFailureMarker>(
      payload_bytes, failure, error);
}

}  // namespace session
}  // namespace remote_identification_radar
