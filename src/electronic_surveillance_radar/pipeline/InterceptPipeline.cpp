#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"

#include <cmath>
#include <utility>

#include "common/logging/ProjectLog.h"
#include "electronic_surveillance_radar/pipeline/MutableEsrContext.h"
#include "electronic_surveillance_radar/pipeline/ObservationFeatureEncoder.h"
#include "electronic_surveillance_radar/pipeline/PipelineRuntimeSnapshot.h"

namespace electronic_surveillance_radar {
namespace pipeline {

namespace {

/**
 * @brief 从 EsrInternalExecutionConfig 的 intercept.cluster 构建特征尺度。
 */
ObservationFeatureScales BuildFeatureScales(const InterceptClusterConfig& cluster) {
  ObservationFeatureScales scales;
  scales.rf_scale_hz = cluster.rf_scale_hz;
  scales.pw_scale_sec = cluster.pw_scale_sec;
  scales.az_scale_deg = cluster.az_scale_deg;
  scales.el_scale_deg = cluster.el_scale_deg;
  scales.snr_scale_db = cluster.snr_scale_db;
  return scales;
}

/**
 * @brief 从 EsrInternalExecutionConfig 的 runtime.track 构建关联配置。
 */
extension::InterceptAssociationConfig BuildAssociationConfig(const RuntimeTrackConfig& track) {
  extension::InterceptAssociationConfig assoc;
  assoc.gate_distance = track.gate_distance;
  assoc.confirm_hits = track.confirm_hits;
  assoc.max_missed_cycles = track.max_missed_cycles;
  assoc.confidence_alpha = track.confidence_alpha;
  assoc.output_tentative = track.output_tentative;
  return assoc;
}

bool HasSameScanGeometry(const extension::InterceptScanConfig& lhs,
                         const extension::InterceptScanConfig& rhs) {
  return lhs.scan_start_az_deg == rhs.scan_start_az_deg &&
         lhs.scan_end_az_deg == rhs.scan_end_az_deg &&
         lhs.scan_start_el_deg == rhs.scan_start_el_deg &&
         lhs.scan_end_el_deg == rhs.scan_end_el_deg && lhs.az_step_deg == rhs.az_step_deg &&
         lhs.el_step_deg == rhs.el_step_deg && lhs.scan_start_pos == rhs.scan_start_pos &&
         lhs.scan_sequence == rhs.scan_sequence;
}

}  // namespace

InterceptPipeline::InterceptPipeline(EsrInternalExecutionConfig config)
    : config_(std::move(config)), associator_(BuildAssociationConfig(config_.runtime.track)) {
  feature_scales_ = BuildFeatureScales(config_.intercept.cluster);
}

void InterceptPipeline::UpdateConfig(const EsrInternalExecutionConfig& config) {
  if (!HasSameScanGeometry(config_.resolved_scan, config.resolved_scan)) {
    scan_phase_cycles_ = 0.0;
  }
  config_ = config;
  // 重建依赖 config_ 的派生状态（feature_scales、associator）。
  feature_scales_ = BuildFeatureScales(config_.intercept.cluster);
  associator_.UpdateConfig(BuildAssociationConfig(config_.runtime.track));
}

extension::InterceptPipelineResult InterceptPipeline::RunCycle(
    const session::EsrCycleInput& input_state,
    const environment::IEsrEnvironmentService& environment_service) {
  extension::InterceptPipelineResult result;
  if (!config_.sensor_enabled) {
    // 中译：传感器已关闭，本周期跳过（周期号）。
    // 标识：电源关闭状态——周期不执行探测，属预期行为而非错误。
    PROJECT_LOG_DEBUG("[InterceptPipeline] sensor disabled, cycle_index={} skipped.",
                      input_state.cycle_index);
    result.sensor_powered_off = true;
    return result;
  }

  const session::EsrEnvironmentSnapshot environment_snapshot =
      environment_service.SampleEnvironment();

  MutableEsrContext ctx;
  const extension::InterceptPipelineConfig pipeline_config = BuildPipelineConfig(config_);
  const extension::InterceptRuntimeConfig runtime_config = BuildRuntimeConfig(config_);
  ctx.BeginCycle(input_state, environment_snapshot, pipeline_config, runtime_config);
  ctx.SetSensorEntityId((sensor_entity_id_ != 0U) ? sensor_entity_id_ : 2U);

  const double scan_phase_before_detection = scan_phase_cycles_;
  const std::uint64_t next_observation_id_before_detection = next_observation_id_;
  const InterceptDetectionOutput detection_output = detection_executor_.Execute(
      ctx, next_observation_id_, &scan_phase_cycles_, completed_receive_cycles_);
  if (detection_output.rf_v2_rejected) {
    scan_phase_cycles_ = scan_phase_before_detection;
    next_observation_id_ = next_observation_id_before_detection;
    result.rf_v2_rejected = true;
    return result;
  }

  result = post_processing_executor_.Execute(detection_output.raw_records, ctx, preprocessor_,
                                             clusterer_, associator_, feature_scales_,
                                             next_hypothesis_id_);
  // 规则 13b：正常周期按发射源排除的 kInfo 诊断并入周期结果（abort 路径不变）。
  result.issues = detection_output.issues;
  result.observation_output.receiver_center_frequency_hz =
      detection_output.receiver_center_frequency_hz;
  result.observation_output.receiver_bandwidth_hz = detection_output.receiver_bandwidth_hz;
  result.observation_output.receiver_saturated = detection_output.receiver_saturated;
  result.scan_azimuth_deg = detection_output.scan_azimuth_deg;
  ++completed_receive_cycles_;

  const std::uint32_t effective_sensor_id =
      (sensor_entity_id_ != 0U) ? sensor_entity_id_ : 2U;

  // 中译：周期执行摘要（周期号、设备号、原始记录数、聚类数、假设数、三类发射源排除计数）。
  // 标识：截获链路每周期概况——检测→聚类→关联各级数量与门控排除分布，供宏观核对与
  //       "零观测"排查；仅人读，不用于状态判断（规则 3）。
  PROJECT_LOG_INFO("[InterceptPipeline] cycle_index={} sensor_entity_id={} raw_records={} clusters={} hypotheses={} "
                   "excluded={{co_site={} zero_power={} below_threshold={}}}",
                   input_state.cycle_index, effective_sensor_id, detection_output.raw_records.size(),
                   result.observation_output.cluster_count,
                   result.emitter_output.hypotheses.size(), detection_output.excluded_co_site,
                   detection_output.excluded_zero_power, detection_output.excluded_below_threshold);

  return result;
}

extension::InterceptPipelineRuntimeState InterceptPipeline::CaptureRuntimeState() const {
  auto snapshot = std::make_shared<PipelineRuntimeSnapshot>();
  snapshot->scan_phase_cycles = scan_phase_cycles_;
  snapshot->completed_receive_cycles = completed_receive_cycles_;
  snapshot->next_observation_id = next_observation_id_;
  snapshot->next_hypothesis_id = next_hypothesis_id_;
  snapshot->tracks = associator_.CaptureTracks();

  extension::InterceptPipelineRuntimeState state;
  state.owner_identity = this;
  state.schema_version = 3U;
  CapturePipelineSnapshot(state, snapshot);
  return state;
}

bool InterceptPipeline::RestoreRuntimeState(const extension::InterceptPipelineRuntimeState& state) {
  if (state.owner_identity != this || state.schema_version != 3U) {
    return false;
  }
  const auto* snapshot = RestorePipelineSnapshot(state);
  if (snapshot == nullptr) {
    return false;
  }
  if (std::isfinite(snapshot->scan_phase_cycles) == 0 || snapshot->scan_phase_cycles < 0.0 ||
      snapshot->scan_phase_cycles >= 1.0) {
    return false;
  }
  scan_phase_cycles_ = snapshot->scan_phase_cycles;
  completed_receive_cycles_ = snapshot->completed_receive_cycles;
  next_observation_id_ = snapshot->next_observation_id;
  next_hypothesis_id_ = snapshot->next_hypothesis_id;
  associator_.RestoreTracks(snapshot->tracks);
  return true;
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar
