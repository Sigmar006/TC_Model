#include "airborne_radar/signal/detection/ArInterferenceObservationResolver.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <tuple>
#include <utility>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "airborne_radar/signal/detection/RadarEquations.h"
#include "common/geometry/BearingCluster.h"
#include "common/numerics/Constants.h"

namespace airborne_radar {
namespace signal {
namespace detection {
namespace {

using oneq::common::numerics::RadToDeg;

bool LocalFrameIsUsable(const oneq::coordinate::LocalFrameReference& frame) {
  // LlaPositionDegM 无 IsFinite 重载，显式检查有限性。
  const bool lla_finite = std::isfinite(frame.origin_lla.latitude_deg) &&
                          std::isfinite(frame.origin_lla.longitude_deg) &&
                          std::isfinite(frame.origin_lla.altitude_m);
  return lla_finite && oneq::coordinate::IsFinite(frame.frame_attitude_deg);
}

// 把发射 ECEF 位置转换到雷达局部笛卡尔系下的方位/俯仰（与 ArSceneTarget look angle 同系）。
// 失败时返回 false，由调用方决定是否回退到 ECEF 切平面方位。
bool TryEcefPositionToRadarLocalAngles(const oneq::coordinate::EcefPositionM& emission_ecef,
                                       const oneq::coordinate::EcefPositionM& receiver_ecef,
                                       const oneq::coordinate::LocalFrameReference& frame,
                                       double* azimuth_deg, double* elevation_deg) {
  if (azimuth_deg == nullptr || elevation_deg == nullptr) {
    return false;
  }
  // 以接收机 ECEF 位置为相对原点，构造视线的 ENU 分量：先转 ECEF→ENU（绝对位置）再取差，
  // 等价于差向量在 origin_lla 处的 ECEF→ENU（线性近似下一致，且与目标 look angle 同口径）。
  oneq::coordinate::EcefPositionM emission_relative;
  emission_relative.x_m = emission_ecef.x_m;  // TryEcefToEnu 以 origin_lla 为原点，需绝对 ECEF
  emission_relative.y_m = emission_ecef.y_m;
  emission_relative.z_m = emission_ecef.z_m;
  oneq::coordinate::EcefPositionM receiver_absolute;
  receiver_absolute.x_m = receiver_ecef.x_m;
  receiver_absolute.y_m = receiver_ecef.y_m;
  receiver_absolute.z_m = receiver_ecef.z_m;
  oneq::coordinate::EnuPositionM emission_enu;
  oneq::coordinate::EnuPositionM receiver_enu;
  if (!oneq::coordinate::TryEcefToEnu(emission_relative, frame.origin_lla, &emission_enu) ||
      !oneq::coordinate::TryEcefToEnu(receiver_absolute, frame.origin_lla, &receiver_enu)) {
    return false;
  }
  const double east = emission_enu.east_m - receiver_enu.east_m;
  const double north = emission_enu.north_m - receiver_enu.north_m;
  const double up = emission_enu.up_m - receiver_enu.up_m;
  // ENU→雷达局部（扣除平台姿态+挂架角）。
  const oneq::coordinate::Vector3d local =
      oneq::coordinate::RotateEnuToLocal(east, north, up, frame.frame_attitude_deg);
  // 局部系方位/俯仰口径与 TargetLookResolver::Resolve 一致：
  //   az = atan2(local.y, local.x)，el = atan2(local.z, hypot(local.x, local.y))。
  const double horizontal = std::hypot(local.x, local.y);
  *azimuth_deg = RadToDeg(std::atan2(local.y, local.x));
  *elevation_deg = RadToDeg(std::atan2(local.z, horizontal));
  return true;
}

bool SameIdentity(const oneq::electromagnetics::RfEmissionIdentity& left,
                  const oneq::electromagnetics::RfEmissionIdentity& right) {
  return left.sensor_entity_id == right.sensor_entity_id &&
         left.emission_id == right.emission_id;
}

const oneq::electromagnetics::RfSceneEmission* FindEmission(
    const oneq::electromagnetics::RfSceneFrame& scene,
    const oneq::electromagnetics::RfEmissionIdentity& identity) {
  for (const auto& emission : scene.emissions) {
    if (SameIdentity(emission.identity, identity)) {
      return &emission;
    }
  }
  return nullptr;
}

double CenterFrequencyHz(const oneq::electromagnetics::RfWaveformSchedule& waveform) {
  if (waveform.kind == oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep) {
    return 0.5 * (waveform.sweep_start_frequency_hz + waveform.sweep_stop_frequency_hz);
  }
  return waveform.center_frequency_hz;
}

double ObservableBandwidthHz(const oneq::electromagnetics::RfWaveformSchedule& waveform) {
  if (waveform.kind == oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep) {
    return std::fabs(waveform.sweep_stop_frequency_hz - waveform.sweep_start_frequency_hz) +
           waveform.occupied_bandwidth_hz;
  }
  return waveform.occupied_bandwidth_hz;
}

using ObservationSortKey = std::tuple<double, double, double, double, double, std::uint8_t, double>;

struct ObservationCandidate {
  session::ArInterferenceObservation observation{};
  oneq::electromagnetics::RfEmissionIdentity identity{};
};

ObservationSortKey MakeSortKey(const session::ArInterferenceObservation& observation) {
  return std::make_tuple(
      observation.estimated_bearing_azimuth_deg, observation.estimated_bearing_elevation_deg,
      observation.estimated_off_boresight_deg, observation.estimated_center_frequency_hz,
      observation.estimated_bandwidth_hz,
      static_cast<std::uint8_t>(observation.estimated_waveform_kind),
      observation.jammer_to_noise_db);
}

std::uint64_t Mix64(std::uint64_t value) {
  value += UINT64_C(0x9e3779b97f4a7c15);
  value = (value ^ (value >> 30U)) * UINT64_C(0xbf58476d1ce4e5b9);
  value = (value ^ (value >> 27U)) * UINT64_C(0x94d049bb133111eb);
  value ^= value >> 31U;
  return value;
}

std::uint64_t HashEmissionIdentity(const oneq::electromagnetics::RfEmissionIdentity& identity) {
  std::uint64_t hash = Mix64(identity.sensor_entity_id);
  return Mix64(hash ^ identity.emission_id);
}

bool WaveformsShareResolutionCell(const session::ArInterferenceObservation& left,
                                  const session::ArInterferenceObservation& right) {
  const double half_sum_bandwidth_hz =
      0.5 * (left.estimated_bandwidth_hz + right.estimated_bandwidth_hz);
  return std::fabs(left.estimated_center_frequency_hz - right.estimated_center_frequency_hz) <=
         half_sum_bandwidth_hz;
}

// splitmix32 finalizer keyed by a per-observable domain tag, mirroring the
// ECM/SBIRS DeriveStreamSeed convention. Two observables with the same base
// seed but distinct tags draw from uncorrelated sub-streams, so the range and
// range-rate perturbations are independent.
const std::uint32_t kRangeDomain = UINT32_C(0x524e4745);      // "RNGE"
const std::uint32_t kRangeRateDomain = UINT32_C(0x52415445);  // "RATE"

std::uint32_t DeriveStreamSeed(std::uint32_t base_seed, std::uint32_t domain_tag) {
  std::uint32_t value = base_seed ^ domain_tag;
  value ^= value >> 16U;
  value *= 0x7feb352dU;
  value ^= value >> 15U;
  value *= 0x846ca68bU;
  value ^= value >> 16U;
  return value == 0U ? 1U : value;
}

// Add a deterministic zero-mean Gaussian perturbation of the given standard
// deviation to a truth-derived scalar, so the observable is no longer an exact
// simulation truth value (contract.md:348). The perturbation is keyed by the
// base seed + domain tag + an emission-stable position tag, making it
// reproducible across replays (same inputs ⇒ same output) while decorrelating
// distinct emissions. Returns the perturbed value, or the original on any
// non-finite result (caller still validates downstream).
double Perturb(double truth_value, double std_dev, std::uint32_t base_seed,
               std::uint32_t domain_tag, std::uint64_t emission_tag) {
  if (!std::isfinite(truth_value) || !std::isfinite(std_dev) || std_dev <= 0.0) {
    return truth_value;
  }
  // Mix the emission-stable tag into the seed so two emissions in the same
  // cycle draw independent noise; base seed already encodes cycle + receiver.
  const std::uint32_t seed =
      DeriveStreamSeed(base_seed, domain_tag) ^
      static_cast<std::uint32_t>(emission_tag >> 32U) ^
      static_cast<std::uint32_t>(emission_tag & 0xFFFFFFFFU);
  std::mt19937 engine(seed == 0U ? 1U : seed);
  std::normal_distribution<double> distribution(0.0, std_dev);
  return truth_value + distribution(engine);
}

}  // namespace

bool TryResolveArInterferenceObservations(
    const oneq::electromagnetics::RfSceneFrame& scene,
    const oneq::electromagnetics::RfSceneReceiverState& receiver,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double thermal_noise_power_w, double jammer_to_noise_gate_db,
    const oneq::coordinate::LocalFrameReference& platform_frame, std::uint32_t perturbation_seed,
    std::vector<session::ArInterferenceObservation>* observations,
    ArDeceptionMeasurementCandidateList* deception_candidates) {
  if (observations == nullptr ||
      !oneq::electromagnetics::TryValidateRfSceneFrame(scene) ||
      !std::isfinite(thermal_noise_power_w) || thermal_noise_power_w <= 0.0 ||
      !std::isfinite(jammer_to_noise_gate_db)) {
    return false;
  }
  const bool frame_usable = LocalFrameIsUsable(platform_frame);

  std::vector<ObservationCandidate> candidate;
  candidate.reserve(incident_links.size());
  for (const auto& link : incident_links) {
    if (SameIdentity(link.identity, own_emission_identity)) {
      continue;
    }
    if (!std::isfinite(link.received_power_w) || link.received_power_w < 0.0) {
      return false;
    }
    const double jammer_to_noise_linear = link.received_power_w / thermal_noise_power_w;
    if (jammer_to_noise_linear <= 0.0) {
      continue;
    }
    const double jammer_to_noise_db = 10.0 * std::log10(jammer_to_noise_linear);
    if (jammer_to_noise_db < jammer_to_noise_gate_db) {
      continue;
    }
    const oneq::electromagnetics::RfSceneEmission* emission = FindEmission(scene, link.identity);
    if (emission == nullptr) {
      return false;
    }
    const double x = emission->position_ecef_m.x_m - receiver.position_ecef_m.x_m;
    const double y = emission->position_ecef_m.y_m - receiver.position_ecef_m.y_m;
    const double z = emission->position_ecef_m.z_m - receiver.position_ecef_m.z_m;
    const double raw_range_m = std::sqrt(x * x + y * y + z * z);
    if (!std::isfinite(raw_range_m) || raw_range_m < 0.0) {
      return false;
    }
    // 同平台（co-site）干扰源物理距离为 0 时，取最小可观测距离作为去真值化基线，
    // 避免零均值扰动产生非物理负值触发 fail-closed（co-site 隔离已处理功率预算）。
    constexpr double kMinObservableRangeM = 1.0;
    const double range_m = std::max(raw_range_m, kMinObservableRangeM);
    // 发射稳定的 64 位标签：用于把斜距/径向速度扰动与具体发射绑定，使同种子同输入下
    // 可复现、不同发射之间互不相关。组合 platform/equipment/emission 身份（truth 身份
    // 仅用于种子混入，不写入 observation，仍遵守去真值化约定）。
    const std::uint64_t emission_tag = HashEmissionIdentity(link.identity);
    // 距离/径向速度测量噪声标准差：以接收端 J/N 作为有效信噪比，发射占用带宽作为距离
    // 分辨带宽。径向速度噪声近似为 σ_v ≈ σ_R / dwell（dwell 取场景窗口时长，下界保护）。
    const double range_std_m = static_cast<double>(RadarEquations::ComputeRangeErrorStdDev(
        static_cast<float>(jammer_to_noise_db),
        static_cast<float>(emission->waveform.occupied_bandwidth_hz)));
    const double dwell_s = std::isfinite(scene.window_duration_s) && scene.window_duration_s > 0.0
                               ? scene.window_duration_s
                               : 1.0e-3;
    const double range_rate_std_mps = range_std_m / dwell_s;

    session::ArInterferenceObservation observation;
    // 去真值化：斜距叠加由 MeasurementErrorModel 标准差驱动的确定性零均值噪声。
    double perturbed_range_m =
        Perturb(range_m, range_std_m, perturbation_seed, kRangeDomain, emission_tag);
    // 同平台干扰源 range_m 被钳制到 kMinObservableRangeM（~1 m），而误差模型含 20 m
    // 系统偏置项，扰动后大概率为负值。取绝对值保持去真值化噪声幅度，同时避免触发
    // fail-closed（非 co-site 场景 range_m >> std，不会产生负值）。
    if (raw_range_m < kMinObservableRangeM && perturbed_range_m < 0.0 &&
        std::isfinite(perturbed_range_m)) {
      perturbed_range_m = -perturbed_range_m;
    }
    if (!std::isfinite(perturbed_range_m) || perturbed_range_m <= 0.0) {
      return false;
    }
    observation.estimated_slant_range_m = perturbed_range_m;
    observation.estimated_bearing_azimuth_deg = RadToDeg(std::atan2(y, x));
    observation.estimated_bearing_elevation_deg = RadToDeg(std::asin(z / range_m));
    // 雷达局部系方位（与目标 look angle 同系）。无可用 pose 时留零，由下游回退并告警。
    if (frame_usable) {
      double local_az_deg = 0.0;
      double local_el_deg = 0.0;
      if (TryEcefPositionToRadarLocalAngles(emission->position_ecef_m, receiver.position_ecef_m,
                                            platform_frame, &local_az_deg, &local_el_deg)) {
        observation.has_local_bearings = true;
        observation.estimated_bearing_azimuth_local_deg = local_az_deg;
        observation.estimated_bearing_elevation_local_deg = local_el_deg;
      }
    }
    const double direction_x = x / range_m;
    const double direction_y = y / range_m;
    const double direction_z = z / range_m;
    // 径向速度：相对速度（发射体-接收机）与视线单位向量的点乘（正值表示远离）。
    // 以相对速度计算保证对快速移动平台的距离变化率估计准确，满足反 VGPO 评分门限要求。
    double range_rate_mps = 0.0;
    if (oneq::coordinate::IsFinite(emission->velocity_ecef_mps) &&
        oneq::coordinate::IsFinite(receiver.velocity_ecef_mps)) {
      const double rel_vx = emission->velocity_ecef_mps.x_mps - receiver.velocity_ecef_mps.x_mps;
      const double rel_vy = emission->velocity_ecef_mps.y_mps - receiver.velocity_ecef_mps.y_mps;
      const double rel_vz = emission->velocity_ecef_mps.z_mps - receiver.velocity_ecef_mps.z_mps;
      range_rate_mps = rel_vx * direction_x + rel_vy * direction_y + rel_vz * direction_z;
    } else if (oneq::coordinate::IsFinite(emission->velocity_ecef_mps)) {
      // 回退：接收机 ECEF 速度非有限时仅用发射体速度。
      range_rate_mps = emission->velocity_ecef_mps.x_mps * direction_x +
                       emission->velocity_ecef_mps.y_mps * direction_y +
                       emission->velocity_ecef_mps.z_mps * direction_z;
    }
    // 去真值化：径向速度同样叠加确定性噪声（contract.md:348）。
    observation.estimated_range_rate_mps = Perturb(
        range_rate_mps, range_rate_std_mps, perturbation_seed, kRangeRateDomain, emission_tag);
    const double boresight_dot =
        std::max(-1.0, std::min(1.0, direction_x * receiver.antenna.boresight_ecef.x +
                                         direction_y * receiver.antenna.boresight_ecef.y +
                                         direction_z * receiver.antenna.boresight_ecef.z));
    observation.estimated_off_boresight_deg = RadToDeg(std::acos(boresight_dot));
    const double transmit_center_frequency_hz = CenterFrequencyHz(emission->waveform);
    const double arrival_center_frequency_hz = transmit_center_frequency_hz + link.doppler_shift_hz;
    if (!std::isfinite(arrival_center_frequency_hz) || arrival_center_frequency_hz <= 0.0 ||
        !std::isfinite(link.propagation_delay_s) || link.propagation_delay_s < 0.0) {
      return false;
    }
    observation.estimated_center_frequency_hz = arrival_center_frequency_hz;
    observation.estimated_bandwidth_hz = ObservableBandwidthHz(emission->waveform);
    observation.estimated_waveform_kind = emission->waveform.kind;
    observation.jammer_to_noise_db = jammer_to_noise_db;
    // VGPO 可观测残差：先使用 incident link 的到达载频，再扣除本振与链路物理
    // Doppler 组成的无欺骗期望。这样公开中心频率代表真实接收端事实，而评分残差只保留
    // 转发波形相对受害雷达载频的额外偏移。
    const double reference_carrier_hz =
        std::isfinite(receiver.center_frequency_hz) && receiver.center_frequency_hz > 0.0
            ? receiver.center_frequency_hz
            : transmit_center_frequency_hz;
    observation.estimated_carrier_offset_hz =
        observation.estimated_center_frequency_hz - (reference_carrier_hz + link.doppler_shift_hz);
    // RGPO 可观测残差：首脉冲接收时刻减去“窗口起点 + 同一 incident link 的单程
    // propagation”。first_pulse_time_s 是发射端绝对时刻，因此传播项在到达时加一次、
    // 在期望基线减一次，最终恰好留下 ECM 编入发射计划的额外双程假距离时延。
    // 仅对 kPulseTrain（具有首脉冲时间）有意义；其他波形保持 0。
    if (emission->waveform.kind == oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain) {
      const double first_pulse_arrival_s =
          emission->waveform.first_pulse_time_s + link.propagation_delay_s;
      const double expected_arrival_s = scene.window_start_time_s + link.propagation_delay_s;
      observation.estimated_first_pulse_delay_s = first_pulse_arrival_s - expected_arrival_s;
    }
    const double quality_scale = std::sqrt(std::max(1.0, jammer_to_noise_linear));
    observation.bearing_standard_deviation_deg =
        receiver.antenna.half_power_beamwidth_deg / quality_scale;
    observation.frequency_standard_deviation_hz =
        emission->waveform.occupied_bandwidth_hz / (2.0 * quality_scale);
    observation.bandwidth_standard_deviation_hz =
        observation.estimated_bandwidth_hz / quality_scale;
    ObservationCandidate resolved;
    resolved.observation = observation;
    resolved.identity = link.identity;
    candidate.push_back(resolved);
  }
  std::sort(candidate.begin(), candidate.end(),
            [](const ObservationCandidate& left, const ObservationCandidate& right) {
    const ObservationSortKey left_key = MakeSortKey(left.observation);
    const ObservationSortKey right_key = MakeSortKey(right.observation);
    return left_key != right_key
               ? left_key < right_key
               : std::tie(left.identity.sensor_entity_id, left.identity.emission_id) <
                     std::tie(right.identity.sensor_entity_id, right.identity.emission_id);
  });

  for (std::size_t index = 0U; index < candidate.size(); ++index) {
    candidate[index].observation.observation_id = static_cast<std::uint64_t>(index + 1U);
  }

  // 欺骗特征提取由 resolver 单独拥有：在相同波束与接收频率分辨单元内，对 kPulseTrain
  // 建立连通分量。每个分量只生成一条内部 cluster 元数据，generator 不再按另一套固定网格
  // 重新猜测簇，从结构上消除 N 个成员各自扩展 N 次的 N² 路径。
  const double beamwidth_deg = receiver.antenna.half_power_beamwidth_deg;
  const auto is_pulse_train = [&candidate](std::size_t i) {
    return candidate[i].observation.estimated_waveform_kind ==
           oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain;
  };
  const auto azimuth_of = [frame_usable, &candidate](std::size_t i) {
    return frame_usable ? candidate[i].observation.estimated_bearing_azimuth_local_deg
                        : candidate[i].observation.estimated_bearing_azimuth_deg;
  };
  const auto elevation_of = [frame_usable, &candidate](std::size_t i) {
    return frame_usable ? candidate[i].observation.estimated_bearing_elevation_local_deg
                        : candidate[i].observation.estimated_bearing_elevation_deg;
  };

  std::vector<std::size_t> parent(candidate.size());
  for (std::size_t i = 0U; i < parent.size(); ++i) {
    parent[i] = i;
  }
  for (std::size_t i = 0U; i < candidate.size(); ++i) {
    if (!is_pulse_train(i)) {
      continue;
    }
    for (std::size_t j = i + 1U; j < candidate.size(); ++j) {
      if (!is_pulse_train(j) ||
          !oneq::common::geometry::AreBearingsCoherent(
              azimuth_of(i), elevation_of(i), azimuth_of(j), elevation_of(j), beamwidth_deg) ||
          !WaveformsShareResolutionCell(candidate[i].observation, candidate[j].observation)) {
        continue;
      }
      std::size_t root_i = i;
      while (parent[root_i] != root_i) {
        root_i = parent[root_i];
      }
      std::size_t root_j = j;
      while (parent[root_j] != root_j) {
        root_j = parent[root_j];
      }
      const std::size_t root = std::min(root_i, root_j);
      parent[root_i] = root;
      parent[root_j] = root;
    }
  }

  std::map<std::size_t, std::vector<std::size_t>> components;
  for (std::size_t i = 0U; i < candidate.size(); ++i) {
    if (!is_pulse_train(i)) {
      continue;
    }
    std::size_t root = i;
    while (parent[root] != root) {
      root = parent[root];
    }
    components[root].push_back(i);
  }

  ArDeceptionMeasurementCandidateList resolved_candidates;
  for (const auto& component : components) {
    const std::vector<std::size_t>& members = component.second;
    if (members.size() < 2U) {
      candidate[members.front()].observation.coherent_emission_count = 1U;
      continue;
    }
    for (std::size_t index : members) {
      candidate[index].observation.deception_class = session::DeceptionClass::kLikelyFalseTarget;
      candidate[index].observation.coherent_emission_count =
          static_cast<std::uint32_t>(members.size());
    }
    // 逐 member 生成带 physical provenance 的候选量测。
    for (std::size_t idx : members) {
      const auto& obs = candidate[idx].observation;
      ArDeceptionMeasurementCandidate dc;
      dc.source_observation_id = obs.observation_id;
      dc.source_emission_identity = candidate[idx].identity;
      dc.estimated_first_pulse_delay_s = obs.estimated_first_pulse_delay_s;
      dc.estimated_carrier_offset_hz = obs.estimated_carrier_offset_hz;
      dc.jammer_to_noise_db = obs.jammer_to_noise_db;
      dc.used_local_bearings = obs.has_local_bearings;
      // 表观距离/径向速度：仅当接收端残差超过可观测门限时，把 ECM 编入波形的额外假距离/假多普勒
      // 叠加到几何量上，使候选量测落在欺骗后的 apparent 位置/速率而非干扰机几何位置。门限内保持
      // 几何值，避免无欺骗场景漂移。残差已由 obs 携带（仅 kPulseTrain 有意义），参考载频沿用
      // estimated_carrier_offset_hz 的同口径（receiver 本振 >0 则用本振，否则发射中心）。
      const double geometric_range_m = obs.estimated_slant_range_m > 0.0 ? obs.estimated_slant_range_m : 50000.0;
      double apparent_range_m = geometric_range_m;
      if (std::isfinite(obs.estimated_first_pulse_delay_s) &&
          obs.estimated_first_pulse_delay_s >= kRgpoFirstPulseDelayGateS) {
        // RGPO 双程假距离时延 → 单程等效距离：ΔR = 0.5·c·delay。delay>0 表示距离门被拖远。
        apparent_range_m =
            geometric_range_m + 0.5 * static_cast<double>(oneq::common::numerics::kLightSpeed) *
                                    obs.estimated_first_pulse_delay_s;
      }
      double apparent_range_rate_mps = obs.estimated_range_rate_mps;
      if (std::isfinite(obs.estimated_carrier_offset_hz) &&
          std::fabs(obs.estimated_carrier_offset_hz) >= kVgpoCarrierOffsetGateHz) {
        const oneq::electromagnetics::RfSceneEmission* source_emission =
            FindEmission(scene, candidate[idx].identity);
        const double transmit_center_frequency_hz =
            source_emission != nullptr ? CenterFrequencyHz(source_emission->waveform) : 0.0;
        const double reference_carrier_hz =
            std::isfinite(receiver.center_frequency_hz) && receiver.center_frequency_hz > 0.0
                ? receiver.center_frequency_hz
                : transmit_center_frequency_hz;
        if (std::isfinite(reference_carrier_hz) && reference_carrier_hz > 0.0) {
          // VGPO 假多普勒：单基地双程，Δv = -0.5·λ_ref·Δf（冻结口径）。
          const double lambda_ref_m =
              static_cast<double>(oneq::common::numerics::kLightSpeed) / reference_carrier_hz;
          apparent_range_rate_mps = obs.estimated_range_rate_mps - 0.5 * lambda_ref_m * obs.estimated_carrier_offset_hz;
        }
      }
      dc.apparent_slant_range_m = apparent_range_m;
      dc.apparent_range_rate_mps = apparent_range_rate_mps;
      // 方位/俯仰：优先局部系。
      const double azimuth_deg = obs.has_local_bearings
          ? obs.estimated_bearing_azimuth_local_deg
          : obs.estimated_bearing_azimuth_deg;
      const double elevation_deg = obs.has_local_bearings
          ? obs.estimated_bearing_elevation_local_deg
          : obs.estimated_bearing_elevation_deg;
      const double az_rad = oneq::common::numerics::DegToRad(azimuth_deg);
      const double el_rad = oneq::common::numerics::DegToRad(elevation_deg);
      const double cos_el = std::cos(el_rad);
      const double range_m = apparent_range_m;
      dc.position = Eigen::Vector3f(
          static_cast<float>(range_m * cos_el * std::cos(az_rad)),
          static_cast<float>(range_m * cos_el * std::sin(az_rad)),
          static_cast<float>(range_m * std::sin(el_rad)));
      dc.velocity = Eigen::Vector3f(
          static_cast<float>(apparent_range_rate_mps * cos_el * std::cos(az_rad)),
          static_cast<float>(apparent_range_rate_mps * cos_el * std::sin(az_rad)),
          static_cast<float>(apparent_range_rate_mps * std::sin(el_rad)));
      // 量测噪声协方差（与 DeceptionMeasurementGenerator 同口径）。
      const double bearing_sigma_deg = std::max(obs.bearing_standard_deviation_deg, 0.1);
      const double sigma_cross_range = range_m * oneq::common::numerics::DegToRad(bearing_sigma_deg);
      const double sigma_range = std::max(sigma_cross_range, 50.0);
      dc.measurement_covariance = Eigen::Matrix3f(
          (Eigen::DiagonalMatrix<float, 3>(
              static_cast<float>(sigma_range * sigma_range),
              static_cast<float>(sigma_cross_range * sigma_cross_range),
              static_cast<float>(sigma_cross_range * sigma_cross_range))));
      resolved_candidates.push_back(dc);
    }
  }

  std::vector<session::ArInterferenceObservation> resolved_observations;
  resolved_observations.reserve(candidate.size());
  for (const ObservationCandidate& item : candidate) {
    resolved_observations.push_back(item.observation);
  }
  *observations = std::move(resolved_observations);
  if (deception_candidates != nullptr) {
    *deception_candidates = std::move(resolved_candidates);
  }
  return true;
}

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
