#include "electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.h"
#include "common/platform/OneqPlatformCompat.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <random>
#include <string>
#include <utility>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/electronic_surveillance_radar/session/EsrIssueCodes.h"
#include "common/geometry/BearingCluster.h"
#include "common/numerics/ClampUtils.h"
#include "common/numerics/NumericGuard.h"
#include "common/timing/TimingRegimeModel.h"
#include "common/validation/ValidationUtils.h"
#include "electronic_surveillance_radar/pipeline/EsrRfV2FrontEnd.h"
#include "electronic_surveillance_radar/pipeline/EsrResolutionCellLedger.h"
#include "electronic_surveillance_radar/pipeline/InterceptComponentFactory.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace {

using oneq::common::numerics::kNumericFloor;

constexpr std::uint64_t kDetectionRandomDomain = 0x4553524454454354ULL;
constexpr std::uint64_t kAngleRandomDomain = 0x455352414e474c45ULL;
constexpr std::uint64_t kRfRandomDomain = 0x4553525246000000ULL;
constexpr std::uint64_t kBandwidthRandomDomain = 0x4553524257000000ULL;
constexpr std::uint64_t kPriRandomDomain = 0x4553525052000000ULL;
constexpr std::uint64_t kPulseWidthRandomDomain = 0x4553525057000000ULL;

// 规则 13b：正常执行周期按发射源门控排除的 kInfo 诊断码（不属于三写，仅承载
// 排查信息）。code 引用 EsrIssueCodes.h 注册表常量。

/// 构造 kInfo 级按发射源排除诊断（不属于三写，仅承载排查信息；规则 13b）。
/// @param cause 门内归因（规则 13b 门内归因条款）；聚合门排除须给出主因，具体门可 kNone。
/// @param emission_index 发射源在 identity 排序后数组中的下标；非负时写入
///                     `location = {kSceneEntity, emission_index}`，供跨周期差分记录器
///                     按实体关联消费（默认 -1 保持 kGlobal，向后兼容）。
///                     ESR 无 target_id 概念，entity_index = 按 (platform/equipment/emission)
///                     排序后的发射源下标（记录器按同一序重排 emissions 解析回 identity）。
session::EsrIssue MakeExclusionIssue(const char* code, const std::string& message,
                                     session::EsrIssueCause cause = session::EsrIssueCause::kNone,
                                     std::ptrdiff_t emission_index = -1) {
  session::EsrIssue issue;
  issue.severity = session::EsrIssueSeverity::kInfo;
  issue.code = code;
  issue.message = message;
  issue.cause = cause;
  if (emission_index >= 0) {
    issue.location.kind = oneq::foundation::ValidationLocationKind::kSceneEntity;
    issue.location.entity_index = static_cast<std::size_t>(emission_index);
  }
  return issue;
}

/// 格式化发射源标识为结构化 key=value 前缀（规则 13b message 约定，不承诺解析稳定性）。
std::string FormatEmissionIdentity(const oneq::electromagnetics::RfEmissionIdentity& id) {
  return "sensor_entity_id=" + std::to_string(id.sensor_entity_id) +
         "; emission_id=" + std::to_string(id.emission_id);
}

/// 格式化量值为两位小数（消息文本稳定，不承诺解析稳定性；规则 13b message 约定）。
std::string FormatNumber(double value) {
  char buffer[32];
  ONEQ_SNPRINTF(buffer, sizeof(buffer), "%.2f", value);
  return buffer;
}

std::uint64_t Mix64(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

std::mt19937 MakeEmissionRandomStream(unsigned int session_seed, std::uint32_t cycle_index,
                                      const oneq::electromagnetics::RfEmissionIdentity& identity,
                                      std::uint64_t domain) {
  std::uint64_t seed = Mix64(static_cast<std::uint64_t>(session_seed));
  seed ^= Mix64(static_cast<std::uint64_t>(cycle_index));
  seed ^= Mix64(identity.sensor_entity_id);
  seed ^= Mix64(identity.emission_id);
  seed ^= Mix64(domain);
  return std::mt19937(static_cast<std::mt19937::result_type>(seed ^ (seed >> 32U)));
}

float ToDb(double ratio) {
  return static_cast<float>(10.0 * std::log10(std::max(ratio, kNumericFloor)));
}

oneq::common::timing::IntegrationMode ToTimingIntegrationMode(
    extension::InterceptIntegrationMode mode) {
  return mode == extension::InterceptIntegrationMode::kCoherent
             ? oneq::common::timing::IntegrationMode::kCoherent
             : oneq::common::timing::IntegrationMode::kNonCoherent;
}

oneq::common::timing::StatisticalDetectionParams ToTimingDetectionParams(
    const extension::InterceptStatisticalDetectionConfig& config) {
  oneq::common::timing::StatisticalDetectionParams params;
  params.pfa = config.pfa;
  params.min_snr_db = config.minimum_snr_db;
  params.pulse_count = config.pulse_count;
  params.integration_mode = ToTimingIntegrationMode(config.integration_mode);
  params.threshold_scale = config.threshold_scale;
  params.enable_statistical_detection = config.enable_statistical_detection;
  return params;
}

std::pair<double, double> BuildReceiverWindow(
    std::uint64_t completed_receive_cycles,
    const extension::InterceptRuntimeConfig& runtime_config) {
  const std::vector<config::EsrTuningWindow>& plan = runtime_config.receiver_hardware.tuning_plan;
  std::uint64_t total_dwell = 0U;
  for (const config::EsrTuningWindow& window : plan) {
    total_dwell += window.dwell_cycles;
  }
  if (total_dwell > 0U) {
    std::uint64_t phase = completed_receive_cycles % total_dwell;
    for (const config::EsrTuningWindow& window : plan) {
      if (phase < window.dwell_cycles) {
        return std::make_pair(window.center_frequency_hz - 0.5 * window.bandwidth_hz,
                              window.center_frequency_hz + 0.5 * window.bandwidth_hz);
      }
      phase -= window.dwell_cycles;
    }
  }
  if (runtime_config.use_fixed_receiver_window &&
      oneq::common::validation::IsFinite(runtime_config.receiver_lower_hz) &&
      oneq::common::validation::IsFinite(runtime_config.receiver_upper_hz) &&
      runtime_config.receiver_upper_hz > runtime_config.receiver_lower_hz) {
    return std::make_pair(runtime_config.receiver_lower_hz, runtime_config.receiver_upper_hz);
  }
  return std::make_pair(runtime_config.receiver_hardware.receiver_band_lower_hz,
                        runtime_config.receiver_hardware.receiver_band_upper_hz);
}

std::size_t ResolveActiveBeamIndex(double* scan_phase_cycles, float dt_sec,
                                   std::size_t scan_pattern_size,
                                   const extension::InterceptRuntimeConfig& runtime_config) {
  if (scan_pattern_size == 0U || scan_phase_cycles == nullptr) {
    return 0U;
  }
  const double safe_dt = std::isfinite(dt_sec) && dt_sec > 0.0f ? dt_sec : 0.0;
  const double scan_rate = std::isfinite(runtime_config.scan_rate_hz) &&
                                   runtime_config.scan_rate_hz > 0.0f
                               ? runtime_config.scan_rate_hz
                               : 1.0;
  const double phase = std::min(std::nextafter(1.0, 0.0), std::max(0.0, *scan_phase_cycles));
  const std::size_t index = static_cast<std::size_t>(
      std::floor(phase * static_cast<double>(scan_pattern_size)));
  *scan_phase_cycles = std::fmod(phase + scan_rate * safe_dt, 1.0);
  return index;
}

session::EsrObservationQuality ClassifyObservationQuality(float snr_db) {
  return snr_db >= 18.0f ? session::EsrObservationQuality::kHigh
         : snr_db >= 10.0f ? session::EsrObservationQuality::kMedium
                           : session::EsrObservationQuality::kLow;
}

session::EsrWaveformClass MapWaveformKindToClass(
    oneq::electromagnetics::RfSceneWaveformKind kind) {
  switch (kind) {
    case oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain:
      return session::EsrWaveformClass::kPulse;
    case oneq::electromagnetics::RfSceneWaveformKind::kContinuous:
      return session::EsrWaveformClass::kContinuous;
    case oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep:
      return session::EsrWaveformClass::kSweep;
    case oneq::electromagnetics::RfSceneWaveformKind::kBandLimitedNoise:
      return session::EsrWaveformClass::kNoise;
    default:
      return session::EsrWaveformClass::kPulse;
  }
}

bool TryResolveLookAngles(const oneq::electromagnetics::RfSceneReceiverState& receiver,
                          const oneq::electromagnetics::RfIncidentLinkResult& link,
                          const oneq::electromagnetics::RfSceneEmission& emission,
                          const oneq::coordinate::EulerAnglesDeg& platform_attitude_deg,
                          double* azimuth_deg,
                          double* elevation_deg, bool* azimuth_observable) {
  if (azimuth_deg == nullptr || elevation_deg == nullptr ||
      azimuth_observable == nullptr || !std::isfinite(link.path_length_m) ||
      link.path_length_m <= 0.0 || link.is_co_site) {
    return false;
  }
  oneq::coordinate::LlaPositionDegM receiver_lla;
  oneq::coordinate::EnuPositionM enu;
  if (!oneq::coordinate::TryEcefToLla(receiver.position_ecef_m, &receiver_lla) ||
      !oneq::coordinate::TryEcefToEnu(emission.position_ecef_m, receiver_lla, &enu)) {
    return false;
  }
  const oneq::coordinate::Vector3d local = oneq::coordinate::RotateEnuToLocal(
      enu.east_m, enu.north_m, enu.up_m, platform_attitude_deg);
  const double horizontal = std::hypot(local.x, local.y);
  if (!std::isfinite(horizontal) || !std::isfinite(local.z)) {
    return false;
  }
  constexpr double kPolarSingularityRelativeTolerance = 1.0e-12;
  if (horizontal <=
      kPolarSingularityRelativeTolerance * std::max(1.0, link.path_length_m)) {
    if (local.z == 0.0) {
      return false;
    }
    *azimuth_deg = 0.0;
    *elevation_deg = std::copysign(90.0, local.z);
    *azimuth_observable = false;
    return true;
  }
  *azimuth_deg = std::atan2(local.y, local.x) * 180.0 / 3.14159265358979323846;
  *elevation_deg = std::atan2(local.z, horizontal) * 180.0 / 3.14159265358979323846;
  *azimuth_observable = true;
  return std::isfinite(*azimuth_deg) && std::isfinite(*elevation_deg);
}

}  // namespace

void ClassifyDeception(float beam_az_width_deg, float beam_el_width_deg,
                       InterceptDetectionOutput* output) {
  if (output == nullptr || output->raw_records.empty()) {
    return;
  }
  // 方位/俯仰使用独立波束宽度（非对称波束正确分轴比较）。
  const double az_width = static_cast<double>(beam_az_width_deg);
  const double el_width = static_cast<double>(beam_el_width_deg);
  const auto is_pulse = [&output](std::size_t i) {
    return output->raw_records[i].observation.waveform_class == session::EsrWaveformClass::kPulse;
  };
  const auto azimuth_of = [&output](std::size_t i) {
    return output->raw_records[i].observation.aoa_az_deg;
  };
  const auto elevation_of = [&output](std::size_t i) {
    return output->raw_records[i].observation.aoa_el_deg;
  };
  for (std::size_t i = 0U; i < output->raw_records.size(); ++i) {
    if (!is_pulse(i)) {
      continue;
    }
    const std::size_t coherent_count = oneq::common::geometry::CountCoherentNeighbors(
        output->raw_records.size(), is_pulse, azimuth_of, elevation_of,
        az_width, el_width, i);
    if (coherent_count >= 2U) {
      output->raw_records[i].observation.deception_class =
          session::EsrDeceptionClass::kLikelyFalseTarget;
    }
  }
}

InterceptDetectionOutput InterceptDetectionExecutor::Execute(
    const MutableEsrContext& ctx, std::uint64_t& next_observation_id, double* scan_phase_cycles,
    std::uint64_t completed_receive_cycles) {
  InterceptDetectionOutput output;
  output.scan_pattern = intercept::ScanPatternGenerator::Generate(
      InterceptComponentFactory::BuildScanPatternConfig(ctx.GetPipelineConfig()));
  if (output.scan_pattern.empty()) {
    output.scan_pattern.push_back(intercept::BeamPointingDeg());
  }
  const intercept::BeamPointingDeg active_beam =
      output
          .scan_pattern[ResolveActiveBeamIndex(scan_phase_cycles, ctx.GetCycleDeltaTimeSec(),
                                               output.scan_pattern.size(), ctx.GetRuntimeConfig())];
  // 当前扫描方位 = 波束中心方位 + 天线安装角（平台系实际指向；与 RF 前端 LOS 求解同算式）。
  output.scan_azimuth_deg = oneq::common::numerics::NormalizeAngle180(
      active_beam.az_deg + ctx.GetRuntimeConfig().orientation.antenna_mount_az_deg);
  const std::pair<double, double> receiver_window =
      BuildReceiverWindow(completed_receive_cycles, ctx.GetRuntimeConfig());
  output.receiver_center_frequency_hz = 0.5 * (receiver_window.first + receiver_window.second);
  output.receiver_bandwidth_hz = receiver_window.second - receiver_window.first;
  if (!ProcessRfV2Frame(
          ctx, active_beam, receiver_window,
          InterceptComponentFactory::BuildAngleErrorModelConfig(ctx.GetPipelineConfig()),
          ToTimingDetectionParams(ctx.GetPipelineConfig().statistical_detection),
          next_observation_id, &output)) {
    output.rf_v2_rejected = true;
  }
  return output;
}

bool InterceptDetectionExecutor::ProcessRfV2Frame(
    const MutableEsrContext& ctx, const intercept::BeamPointingDeg& active_beam,
    const std::pair<double, double>& receiver_window,
    const intercept::AngleErrorModelConfig& angle_error_config,
    const oneq::common::timing::StatisticalDetectionParams& base_detection,
    std::uint64_t& next_observation_id, InterceptDetectionOutput* output) const {
  if (output == nullptr) {
    return false;
  }
  EsrRfV2FrontEndResult front_end;
  if (!TryResolveEsrRfV2FrontEnd(
          ctx.GetCycleInput(), ctx.GetRuntimeConfig().receiver_hardware,
          ctx.GetRuntimeConfig().orientation, active_beam.az_deg, active_beam.el_deg,
          0.5 * (receiver_window.first + receiver_window.second),
          receiver_window.second - receiver_window.first,
          std::max(0.0f, ctx.GetEnvironmentSnapshot().propagation_loss_db),
          ctx.GetSensorEntityId(), &front_end)) {
    return false;
  }
  output->receiver_center_frequency_hz = front_end.channel_receiver.center_frequency_hz;
  output->receiver_bandwidth_hz = front_end.channel_receiver.bandwidth_hz;
  output->receiver_saturated = front_end.receiver_saturated;
  if (front_end.receiver_saturated) {
    return true;
  }
  std::vector<const oneq::electromagnetics::RfSceneEmission*> emissions;
  for (const oneq::electromagnetics::RfSceneEmission& emission : ctx.GetRfEmissions().emissions) {
    emissions.push_back(&emission);
  }
  std::sort(emissions.begin(), emissions.end(),
            [](const oneq::electromagnetics::RfSceneEmission* a,
               const oneq::electromagnetics::RfSceneEmission* b) {
              return a->identity.sensor_entity_id != b->identity.sensor_entity_id
                         ? a->identity.sensor_entity_id < b->identity.sensor_entity_id
                         : a->identity.emission_id < b->identity.emission_id;
            });
  if (emissions.size() != front_end.channel_incident_links.size()) {
    return false;
  }
  const config::EsrHardwareConfig& hardware = ctx.GetRuntimeConfig().receiver_hardware;
  const double thermal_noise = hardware.receiver_sensitivity_w > 0.0f
      ? static_cast<double>(hardware.receiver_sensitivity_w)
      : 1.380649e-23 * hardware.receiver_reference_temperature_k *
        front_end.channel_receiver.bandwidth_hz *
        std::pow(10.0, hardware.receiver_noise_figure_db / 10.0);
  const double ambient_noise =
      std::max(thermal_noise + static_cast<double>(ctx.GetEnvironmentSnapshot().clutter_noise_w),
               kNumericFloor);
  const double beamwidth = std::max(1.0, std::max(static_cast<double>(hardware.beam_az_width_deg),
                                                  static_cast<double>(hardware.beam_el_width_deg)));
  std::vector<EsrArrivalBearing> bearings(front_end.channel_incident_links.size());
  for (std::size_t index = 0U; index < front_end.channel_incident_links.size(); ++index) {
    const oneq::electromagnetics::RfSceneEmission& emission = *emissions[index];
    if (front_end.channel_incident_links[index].is_co_site) {
      // 规则 13b：同址干扰发射源跳过 → kInfo 诊断（不属于三写，仅承载排查信息）。
      // 具体门（同址判定本身可定位）：cause 保持 kNone，隔离度/路径量值进 message。
      const auto& co_site_link = front_end.channel_incident_links[index];
      output->issues.push_back(MakeExclusionIssue(
          session::codes::kEmissionCoSite,
          FormatEmissionIdentity(emission.identity) + "; co_site=true; isolation_db=" +
              FormatNumber(co_site_link.co_site_isolation_db) + " path_length_m=" +
              FormatNumber(co_site_link.path_length_m),
          session::EsrIssueCause::kNone,
          static_cast<std::ptrdiff_t>(index)));
      ++output->excluded_co_site;
      continue;
    }
    if (front_end.channel_incident_links[index].received_power_w <= 0.0) {
      // 规则 13b：零功率发射源跳过 → kInfo 诊断（不属于三写，仅承载排查信息）。
      // 门内归因：接收功率归零的成因——时频重叠窗口为零（kOverlapWindow）/
      // 链路预算归零（发射静默 kTransmitSilent 或传播 kPropagationLoss）。
      const auto& zero_link = front_end.channel_incident_links[index];
      session::EsrIssueCause zero_cause = session::EsrIssueCause::kPropagationLoss;
      if (zero_link.time_overlap_fraction <= 0.0 || zero_link.frequency_overlap_fraction <= 0.0) {
        zero_cause = session::EsrIssueCause::kOverlapWindow;
      } else if (zero_link.received_power_before_overlap_w <= 0.0) {
        zero_cause = emission.waveform.transmit_power_w <= 0.0
                         ? session::EsrIssueCause::kTransmitSilent
                         : session::EsrIssueCause::kPropagationLoss;
      }
      output->issues.push_back(MakeExclusionIssue(
          session::codes::kEmissionZeroPower,
          FormatEmissionIdentity(emission.identity) + "; received_power_w=" +
              FormatNumber(zero_link.received_power_w) + " time_overlap_fraction=" +
              FormatNumber(zero_link.time_overlap_fraction) + " frequency_overlap_fraction=" +
              FormatNumber(zero_link.frequency_overlap_fraction) + " path_length_m=" +
              FormatNumber(zero_link.path_length_m),
          zero_cause,
          static_cast<std::ptrdiff_t>(index)));
      ++output->excluded_zero_power;
      continue;
    }
    EsrArrivalBearing& bearing = bearings[index];
    if (!TryResolveLookAngles(front_end.channel_receiver, front_end.channel_incident_links[index],
                              *emissions[index], ctx.GetPlatformAttitude(), &bearing.azimuth_deg,
                              &bearing.elevation_deg, &bearing.azimuth_observable)) {
      return false;
    }
    bearing.defined = true;
  }

  EsrResolutionCellLedgerResult cell_ledger;
  if (!TryBuildEsrResolutionCellLedger(
          front_end.channel_incident_links, bearings, front_end.channel_receiver,
          beamwidth, &cell_ledger)) {
    return false;
  }
  for (const EsrResolutionCellCandidate& cell_candidate : cell_ledger.candidates) {
    const std::size_t signal_index = cell_candidate.source_index;
    const oneq::electromagnetics::RfIncidentLinkResult& signal =
        front_end.channel_incident_links[signal_index];
    const double center_hz = cell_candidate.estimated_center_frequency_hz;
    const double bandwidth_hz = signal.emission_waveform.occupied_bandwidth_hz;
    const double snr_db =
        ToDb(cell_candidate.signal_power_w /
             std::max(ambient_noise + cell_candidate.interference_power_w,
                      kNumericFloor));
    oneq::common::timing::StatisticalDetectionParams detection = base_detection;
    detection.pulse_count = cell_candidate.effective_pulse_count;
    const float threshold =
        ctx.GetPipelineConfig().statistical_detection.enable_statistical_detection
            ? std::max(ctx.GetPipelineConfig().detection.minimum_snr_db,
                       oneq::common::timing::ComputeDynamicThresholdSnrDb(
                           ambient_noise + cell_candidate.interference_power_w,
                           detection))
            : ctx.GetPipelineConfig().detection.minimum_snr_db;
    std::mt19937 detection_rng = MakeEmissionRandomStream(
        ctx.GetPipelineConfig().algorithm.random_seed, ctx.GetCycleInput().cycle_index,
        signal.identity, kDetectionRandomDomain);
    std::uniform_real_distribution<float> uniform_01(0.0f, 1.0f);
    if (snr_db < threshold ||
        (ctx.GetPipelineConfig().statistical_detection.enable_statistical_detection &&
         uniform_01(detection_rng) >= oneq::common::timing::ComputeStatisticalDetectionProbability(
                                          static_cast<float>(snr_db), threshold, detection))) {
      // 规则 13b：SNR/统计检测门排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
      // 门内归因：硬门（snr 低于静态 minimum_snr_db）vs 统计检测门（动态门/概率抽签）。
      const double min_snr_db = ctx.GetPipelineConfig().detection.minimum_snr_db;
      const session::EsrIssueCause below_cause =
          snr_db < min_snr_db ? session::EsrIssueCause::kHardGateFailed
                              : session::EsrIssueCause::kStatisticalGateFailed;
      output->issues.push_back(MakeExclusionIssue(
          session::codes::kEmissionBelowThreshold,
          FormatEmissionIdentity(signal.identity) + "; snr_db=" + FormatNumber(snr_db) +
              " below threshold=" + FormatNumber(threshold) +
              " margin_db=" + FormatNumber(snr_db - threshold),
          below_cause,
          static_cast<std::ptrdiff_t>(signal_index)));
      ++output->excluded_below_threshold;
      continue;
    }
    RawObservationRecord record;
    const std::uint32_t session_seed = ctx.GetPipelineConfig().algorithm.random_seed;
    const std::uint32_t cycle_index = ctx.GetCycleInput().cycle_index;
    const intercept::RfErrorModelConfig rf_error_config =
        InterceptComponentFactory::BuildRfErrorModelConfig(ctx.GetPipelineConfig());
    const intercept::BandwidthErrorModelConfig bandwidth_error_config =
        InterceptComponentFactory::BuildBandwidthErrorModelConfig(ctx.GetPipelineConfig());
    const intercept::PriErrorModelConfig pri_error_config =
        InterceptComponentFactory::BuildPriErrorModelConfig(ctx.GetPipelineConfig());
    const intercept::PulseWidthErrorModelConfig pulse_width_error_config =
        InterceptComponentFactory::BuildPulseWidthErrorModelConfig(ctx.GetPipelineConfig());
    std::mt19937 angle_rng = MakeEmissionRandomStream(session_seed, cycle_index,
                                                      signal.identity, kAngleRandomDomain);
    record.observation.observation_id = next_observation_id++;
    record.observation.timestamp_s = signal.arrival_start_time_s;
    record.observation.aoa_az_deg = bearings[signal_index].azimuth_deg +
                                    intercept::AngleErrorModel::SampleErrorDeg(
                                        static_cast<float>(snr_db), static_cast<float>(beamwidth),
                                        &angle_rng, angle_error_config);
    record.observation.aoa_el_deg = bearings[signal_index].elevation_deg +
                                    intercept::AngleErrorModel::SampleErrorDeg(
                                        static_cast<float>(snr_db), static_cast<float>(beamwidth),
                                        &angle_rng, angle_error_config);
    // RF/带宽/PRI/PW 的发布值是接收机测量估计，而非真值副本。均值在真值上叠加 SNR 驱动的
    // 零均值高斯误差，std 用同一模型计算，使发布不确定度与采样误差一致。各测量使用独立的
    // identity-keyed 随机子流（纯派生，无跨周期可变 RNG 状态），保证输入顺序无关与 replay
    // 字节级确定。
    const float channel_bandwidth_hz =
        static_cast<float>(front_end.channel_receiver.bandwidth_hz);
    const float occupied_bandwidth_hz_f = static_cast<float>(bandwidth_hz);
    std::mt19937 rf_rng =
        MakeEmissionRandomStream(session_seed, cycle_index, signal.identity, kRfRandomDomain);
    record.observation.rf_hz =
        center_hz + static_cast<double>(intercept::RfErrorModel::SampleErrorHz(
                        static_cast<float>(snr_db), channel_bandwidth_hz, &rf_rng, rf_error_config));
    record.observation.rf_std_hz = intercept::RfErrorModel::ComputeStdDevHz(
        static_cast<float>(snr_db), channel_bandwidth_hz, rf_error_config);
    std::mt19937 bandwidth_rng = MakeEmissionRandomStream(session_seed, cycle_index,
                                                          signal.identity, kBandwidthRandomDomain);
    record.observation.bandwidth_hz = std::max(
        1.0, bandwidth_hz + static_cast<double>(intercept::BandwidthErrorModel::SampleErrorHz(
                                static_cast<float>(snr_db), occupied_bandwidth_hz_f, &bandwidth_rng,
                                bandwidth_error_config)));
    record.observation.bandwidth_std_hz = intercept::BandwidthErrorModel::ComputeStdDevHz(
        static_cast<float>(snr_db), occupied_bandwidth_hz_f, bandwidth_error_config);
    if (signal.emission_waveform.kind == oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain) {
      const float pri_s_f = static_cast<float>(signal.emission_waveform.pulse_repetition_interval_s);
      const float pulse_width_s_f = static_cast<float>(signal.emission_waveform.pulse_width_s);
      std::mt19937 pri_rng =
          MakeEmissionRandomStream(session_seed, cycle_index, signal.identity, kPriRandomDomain);
      record.observation.pri_s = std::max(
          1.0e-12, static_cast<double>(pri_s_f) +
                       intercept::PriErrorModel::SampleErrorS(static_cast<float>(snr_db), pri_s_f,
                                                               &pri_rng, pri_error_config));
      record.observation.pri_std_s = intercept::PriErrorModel::ComputeStdDevS(
          static_cast<float>(snr_db), pri_s_f, pri_error_config);
      std::mt19937 pulse_width_rng = MakeEmissionRandomStream(
          session_seed, cycle_index, signal.identity, kPulseWidthRandomDomain);
      record.observation.pulse_width_s =
          std::max(0.0, static_cast<double>(pulse_width_s_f) +
                            intercept::PulseWidthErrorModel::SampleErrorS(
                                static_cast<float>(snr_db), pulse_width_s_f, &pulse_width_rng,
                                pulse_width_error_config));
      record.observation.pulse_width_std_s = intercept::PulseWidthErrorModel::ComputeStdDevS(
          static_cast<float>(snr_db), pulse_width_s_f, pulse_width_error_config);
    } else {
      record.observation.pri_s = 0.0;
      record.observation.pulse_width_s = 0.0;
      record.observation.pri_std_s = 0.0;
      record.observation.pulse_width_std_s = 0.0;
    }
    record.observation.amplitude_db = ToDb(cell_candidate.signal_power_w);
    record.observation.snr_db = snr_db;
    record.observation.quality = ClassifyObservationQuality(static_cast<float>(snr_db));
    record.observation.waveform_class = MapWaveformKindToClass(signal.emission_waveform.kind);
    output->raw_records.push_back(record);
  }
  ClassifyDeception(hardware.beam_az_width_deg, hardware.beam_el_width_deg, output);
  return true;
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar
