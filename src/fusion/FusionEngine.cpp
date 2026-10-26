/**
 * @file FusionEngine.cpp
 * @brief FusionEngine 实现：身份键直挂 + 空间/方位/特征关联 + 置信度滑窗融合。
 */

#include "1q/fusion/FusionEngine.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nanoflann.hpp>

#include "1q/coordinate/position_transform.h"
#include "common/estimation/EkfFilter.h"
#include "common/estimation/GaussianState.h"
#include "common/estimation/UnscentedPredictor.h"
#include "common/estimation/UnscentedUpdater.h"
#include "common/geometry/BearingCluster.h"
#include "fusion/FusionAcceptanceLog.h"
#include "fusion/FusionAcceptanceRecords.h"
#include "common/numerics/Constants.h"

namespace fusion {

namespace {

namespace estimation = ::oneq::common::estimation;

using oneq::coordinate::EcefPositionM;
using oneq::coordinate::LlaPositionDegM;

constexpr std::uint64_t kUnidentifiedKey = 0U;
// 无身份航迹的合成键起点：与调用方身份键（约定 < 2^63）不冲突。
constexpr std::uint64_t kSyntheticKeyBase = (1ULL << 63);
using oneq::common::numerics::DegToRad;
using oneq::common::numerics::RadToDeg;

// 传感器局部 ENU 基（由原点大地法向导出；极点退化时 invalid）。
struct EnuBasis {
  double east[3];
  double north[3];
  double up[3];
  bool valid;
  // VS2013 不支持成员数组的 NSDMI 花括号初始化（C2536），改由构造函数置零。
  EnuBasis() : valid(false) {
    east[0] = east[1] = east[2] = 0.0;
    north[0] = north[1] = north[2] = 0.0;
    up[0] = up[1] = up[2] = 0.0;
  }
};

EnuBasis MakeEnuBasis(const EcefPositionM& origin) {
  EnuBasis basis;
  const double norm = std::sqrt(origin.x_m * origin.x_m + origin.y_m * origin.y_m +
                                origin.z_m * origin.z_m);
  if (norm < 1.0e-6) {
    return basis;
  }
  basis.up[0] = origin.x_m / norm;
  basis.up[1] = origin.y_m / norm;
  basis.up[2] = origin.z_m / norm;
  // east = normalize(z_world × up) = normalize(-up_y, up_x, 0)；极点处退化。
  const double east_norm = std::sqrt(basis.up[0] * basis.up[0] + basis.up[1] * basis.up[1]);
  if (east_norm < 1.0e-9) {
    return basis;
  }
  basis.east[0] = -basis.up[1] / east_norm;
  basis.east[1] = basis.up[0] / east_norm;
  basis.east[2] = 0.0;
  // north = up × east。
  basis.north[0] = basis.up[1] * basis.east[2] - basis.up[2] * basis.east[1];
  basis.north[1] = basis.up[2] * basis.east[0] - basis.up[0] * basis.east[2];
  basis.north[2] = basis.up[0] * basis.east[1] - basis.up[1] * basis.east[0];
  basis.valid = true;
  return basis;
}

double Dot3(const double* a, const double* b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * @brief ENU 方位量测模型（P2）：h(x) = 目标 ECEF 相对原点的传感器局部 ENU az/el（deg）。
 * @note az 自北向东 [-180,180]、el 出地平 [-90,90]（DetectionRecord 量测原点契约）。
 *       仅被无迹更新器消费（Function）；Jacobian 为解析实现供接口完备。
 */
class EnuBearingMeasurementModel final : public estimation::IMeasurementModel<6, 2> {
 public:
  void SetGeometry(const EcefPositionM& origin) {
    origin_ = origin;
    basis_ = MakeEnuBasis(origin);
  }
  bool HasValidGeometry() const { return basis_.valid; }

  MeasurementVector Function(const StateVector& state) const override {
    MeasurementVector z;
    const double d[3] = {static_cast<double>(state(0)) - origin_.x_m,
                         static_cast<double>(state(2)) - origin_.y_m,
                         static_cast<double>(state(4)) - origin_.z_m};
    const double e = Dot3(d, basis_.east);
    const double n = Dot3(d, basis_.north);
    const double u = Dot3(d, basis_.up);
    const double range = std::sqrt(Dot3(d, d));
    const double sin_el = (range > 1.0e-9) ? std::max(-1.0, std::min(1.0, u / range)) : 0.0;
    z(0) = static_cast<float>(RadToDeg(std::atan2(e, n)));
    z(1) = static_cast<float>(RadToDeg(std::asin(sin_el)));
    return z;
  }

  MeasurementMatrix Jacobian(const StateVector& state) const override {
    MeasurementMatrix h = MeasurementMatrix::Zero();
    if (!basis_.valid) {
      return h;
    }
    const double d[3] = {static_cast<double>(state(0)) - origin_.x_m,
                         static_cast<double>(state(2)) - origin_.y_m,
                         static_cast<double>(state(4)) - origin_.z_m};
    const double e = Dot3(d, basis_.east);
    const double n = Dot3(d, basis_.north);
    const double u = Dot3(d, basis_.up);
    const double range = std::sqrt(Dot3(d, d));
    const double horizontal_sq = e * e + n * n;
    if (range < 1.0e-9 || horizontal_sq < 1.0e-9) {
      return h;
    }
    const double sin_el = std::max(-1.0, std::min(1.0, u / range));
    const double cos_el = std::sqrt(std::max(0.0, 1.0 - sin_el * sin_el));
    if (cos_el < 1.0e-9) {
      return h;
    }
    // ∂az/∂d = (n·east − e·north)/(e²+n²)（rad/m）；×RadToDeg 转 deg/m。
    for (int i = 0; i < 3; ++i) {
      h(0, 2 * i) =
          static_cast<float>(RadToDeg(
              (n * basis_.east[i] - e * basis_.north[i]) / horizontal_sq));
    }
    // ∂el/∂d = (r²·up − u·d)/(r³·cos_el)（rad/m）。
    const double scale = 1.0 / (range * range * range * cos_el);
    for (int i = 0; i < 3; ++i) {
      h(1, 2 * i) = static_cast<float>(
          RadToDeg((range * range * basis_.up[i] - u * d[i]) * scale));
    }
    return h;
  }

 private:
  EcefPositionM origin_{};
  EnuBasis basis_{};
};

// 单次量测样本（滑窗元素）。
struct MeasurementSample {
  std::uint32_t source_id{0U};
  std::string source_name{};
  bool has_position{false};
  LlaPositionDegM position{};
  bool has_bearing{false};
  double bearing_az_deg{0.0};
  double bearing_el_deg{0.0};
  std::vector<double> feature{};
  double verdict{0.0};
  double quality{0.0};
  std::uint64_t cycle{0U};
  bool has_timestamp{false};
  double timestamp_s{0.0};
};

// 航迹（库内状态，业务不可见）。
struct Track {
  std::uint64_t key{0U};
  std::map<std::uint32_t, std::deque<MeasurementSample>> channel_samples{};
  MeasurementSample anchor{};      /**< 最近一次量测（关联锚点） */
  bool has_anchor{false};
  std::size_t missed_cycles{0U};
  std::uint64_t last_update_cycle{0U};
  std::size_t hits{0U};            /**< 累计命中数（确认门输入） */
  bool filter_initialized{false};  /**< 航迹滤波器是否已起始（P2） */
  // 中译：**真实量测记账**（2026-10-26）——合成量测（单星保活）不进滤波，
  //       这里记录航迹是否吃到过真实量测，供精度评估剔除纯合成航迹。
  // 标识：真实量测记账。
  bool has_real_update{false};
  std::uint64_t last_real_update_cycle{0U};
  std::uint64_t filter_cycle{0U};  /**< 滤波器最近推进周期 */
  bool filter_has_timestamp{false}; /**< 滤波器时间基准是否已含物理时刻（FUSION-OQ-1） */
  double filter_timestamp_s{0.0};  /**< 滤波器当前状态对应的物理时刻(s) */
  bool velocity_initialized{false}; /**< 初速是否已完成两点差分初始化 */
  bool has_first_position{false};  /**< 是否已记录首个位置点（差分初速输入） */
  EcefPositionM first_position{};  /**< 首个量测位置 */
  double first_timestamp_s{0.0};   /**< 首个量测时刻(s) */
  Eigen::Matrix<float, 6, 1> filter_mean{Eigen::Matrix<float, 6, 1>::Zero()};
  Eigen::Matrix<float, 6, 6> filter_cov{Eigen::Matrix<float, 6, 6>::Identity()};
  Track() = default;
  explicit Track(std::uint64_t k) : key(k) {}
};

double EuclideanDistance(const EcefPositionM& a, const EcefPositionM& b) {
  const double dx = a.x_m - b.x_m;
  const double dy = a.y_m - b.y_m;
  const double dz = a.z_m - b.z_m;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double EuclideanDistance(const std::vector<double>& a, const std::vector<double>& b) {
  double sum = 0.0;
  const std::size_t n = std::min(a.size(), b.size());
  for (std::size_t k = 0; k < n; ++k) {
    const double d = a[k] - b[k];
    sum += d * d;
  }
  return std::sqrt(sum);
}

// nanoflann 数据适配器：暴露航迹位置锚点（ECEF 三维，double）。
class TrackAnchorAdaptor {
 public:
  explicit TrackAnchorAdaptor(const std::vector<EcefPositionM>& anchors)
      : anchors_(anchors) {}

  inline std::size_t kdtree_get_point_count() const { return anchors_.size(); }

  inline double kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
    if (idx >= anchors_.size() || dim >= 3U) {
      return 0.0;
    }
    if (dim == 0U) {
      return anchors_[idx].x_m;
    }
    if (dim == 1U) {
      return anchors_[idx].y_m;
    }
    return anchors_[idx].z_m;
  }

  // nanoflann 1.2.3（VS2013 降级版本）的 L2_Simple_Adaptor 经由 kdtree_distance 求
  // 查询点到数据点的平方距离；1.3.x 改用 kdtree_get_pt，此接口被忽略，故两版本皆安全。
  inline double kdtree_distance(const double* query, const std::size_t idx, std::size_t size) const {
    double dist_sq = 0.0;
    const EcefPositionM& anchor = anchors_[idx];
    const double coords[3] = {anchor.x_m, anchor.y_m, anchor.z_m};
    for (std::size_t i = 0; i < size && i < 3U; ++i) {
      const double diff = query[i] - coords[i];
      dist_sq += diff * diff;
    }
    return dist_sq;
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }

 private:
  const std::vector<EcefPositionM>& anchors_;  // 非拥有
};

using TrackKdTree =
    nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, TrackAnchorAdaptor>,
                                        TrackAnchorAdaptor, 3U, std::size_t>;

// 半径搜索（兼容 nanoflann 1.3.x 与 1.5+ 的返回类型差异，同 ESR KdTreeClusterer 先例）。
std::vector<std::size_t> RadiusSearch(const TrackKdTree& index,
                                      const EcefPositionM& query, double radius_m) {
  using KdDistanceType = TrackKdTree::DistanceType;
#if defined(NANOFLANN_VERSION) && (NANOFLANN_VERSION >= 0x150)
  using KdIndexType = typename TrackKdTree::IndexType;
  std::vector<nanoflann::ResultItem<KdIndexType, KdDistanceType>> matches;
  nanoflann::SearchParameters search_params;
#else
  using KdIndexType = std::size_t;
  std::vector<std::pair<KdIndexType, KdDistanceType>> matches;
  nanoflann::SearchParams search_params;
#endif
  search_params.sorted = true;
  const KdDistanceType radius_sq = static_cast<KdDistanceType>(radius_m * radius_m);
  const double query_xyz[3] = {query.x_m, query.y_m, query.z_m};
  index.radiusSearch(query_xyz, radius_sq, matches, search_params);
  std::vector<std::size_t> indices;
  indices.reserve(matches.size());
  for (const auto& match : matches) {
    indices.push_back(match.first);
  }
  return indices;
}

}  // namespace

class FusionEngine::Impl {
 public:
  explicit Impl(const FusionConfig& config) : config_(config) {}

  std::vector<FusedTarget> Update(const std::vector<DetectionRecord>& detections,
                                  std::uint64_t cycle) {
    // 本周期先统一递增失跟计数；收到量测的航迹在 AppendSample 中清零。
    for (auto& entry : tracks_) {
      ++entry.second.missed_cycles;
    }

    // 冻结语义：同周期新建航迹不参与本周期后续关联（自下周期起成为候选）。
    // 快照周期起始航迹键集合，关联阶段（含 KD-tree 建树）只考虑快照内航迹。
    std::set<std::uint64_t> cycle_start_keys;
    for (const auto& entry : tracks_) {
      cycle_start_keys.insert(entry.first);
    }

    // ① 身份键直挂：同键探测归并到同航迹（跨源一致性由调用方保证）。
    std::unordered_map<std::uint64_t, std::vector<std::size_t>> keyed;
    std::vector<std::size_t> unidentified;
    for (std::size_t i = 0; i < detections.size(); ++i) {
      if (detections[i].key == kUnidentifiedKey) {
        unidentified.push_back(i);
      } else {
        keyed[detections[i].key].push_back(i);
      }
    }
    const double nominal_cycle_time =
        static_cast<double>(cycle) * config_.track_cycle_period_sec;
    const auto timestamp_order = [&detections, nominal_cycle_time](std::size_t a, std::size_t b) {
      const double ta =
          detections[a].has_timestamp ? detections[a].timestamp_s : nominal_cycle_time;
      const double tb =
          detections[b].has_timestamp ? detections[b].timestamp_s : nominal_cycle_time;
      return ta < tb;
    };
    for (auto& group : keyed) {
      std::stable_sort(group.second.begin(), group.second.end(), timestamp_order);
      auto it = tracks_.find(group.first);
      if (it == tracks_.end()) {
        it = tracks_.emplace(group.first, Track{group.first}).first;
      }
      for (const std::size_t i : group.second) {
        AppendSample(&it->second, detections[i], cycle);
      }
    }

    // ② 无身份探测：空间/方位/特征分层关联。
    AssociateUnidentified(detections, unidentified, cycle, cycle_start_keys);

    // ③ 滑窗裁剪 + 失跟删除。
    PruneWindowsAndAging();

    // ④ 失跟航迹的滤波外推（coasting；本周期已更新的航迹 dt=0 自然跳过）。
    AdvanceFilters(cycle);

    // 验收行写入在 Impl 内收口：直接持有 config_ 单源（周期时长/接力视场宽度）。
    const std::vector<FusedTarget> output = BuildOutput();
    if (FUSION_ACCEPTANCE_LOG_ENABLED()) {
      bool filtering = false;
      for (const FusedTarget& track : output) {
        if (track.has_kinematic_estimate) {
          filtering = true;
          break;
        }
      }
      WriteFusionAcceptance(static_cast<std::uint32_t>(cycle), output, filtering, config_);
    }
    return output;
  }

  void Reset() {
    tracks_.clear();
    synthetic_key_counter_ = 0U;
  }

 private:
  void AppendSample(Track* track, const DetectionRecord& detection, std::uint64_t cycle) {
    MeasurementSample sample;
    sample.source_id = detection.source_id;
    sample.source_name = detection.source_name;
    sample.has_position = detection.has_position;
    sample.position = detection.position;
    sample.has_bearing = detection.has_bearing;
    sample.bearing_az_deg = detection.bearing_az_deg;
    sample.bearing_el_deg = detection.bearing_el_deg;
    sample.feature = detection.feature;
    sample.verdict = detection.verdict;
    sample.quality = detection.quality;
    sample.cycle = cycle;
    sample.has_timestamp = detection.has_timestamp;
    sample.timestamp_s = detection.timestamp_s;
    track->channel_samples[detection.source_id].push_back(std::move(sample));
    track->anchor = track->channel_samples[detection.source_id].back();
    track->has_anchor = true;
    track->missed_cycles = 0U;
    track->last_update_cycle = cycle;
    ++track->hits;
    // 中译：**合成量测不进滤波**（2026-10-26）——单星保活记录的距离是冻结值，
    //       只用于保活/连续性（命中数、最近周期、锚点），否则会把滤波状态拖偏，
    //       并让精度评估把非独立量测当真实观测。
    // 标识：合成量测隔离。
    if (!detection.synthetic) {
      track->has_real_update = true;
      track->last_real_update_cycle = cycle;
    }
    if (config_.enable_track_filtering && !detection.synthetic) {
      ApplyFilterSample(track, detection, cycle);
    }
  }

  // 特征门限：未启用（threshold <= 0）或任一侧无特征/维度不一致时不构成约束。
  bool FeatureGatePasses(const DetectionRecord& detection, const Track& track) const {
    if (!(config_.feature_threshold > 0.0) || !track.has_anchor) {
      return true;
    }
    if (detection.feature.empty() || track.anchor.feature.empty()) {
      return true;
    }
    if (detection.feature.size() != track.anchor.feature.size()) {
      return true;  // 异构传感器特征维度不一致：特征门不约束。
    }
    return EuclideanDistance(detection.feature, track.anchor.feature) <=
           config_.feature_threshold;
  }

  // 无身份探测关联：带位置 → KD-tree 半径搜索；仅方位 → 方位相干；皆无 → 仅特征门。
  // 候选航迹限于周期起始快照（同周期新建航迹不参与本周期关联）。
  // 未关联者创建新航迹（合成键）。
  void AssociateUnidentified(const std::vector<DetectionRecord>& detections,
                             const std::vector<std::size_t>& indices,
                             std::uint64_t cycle,
                             const std::set<std::uint64_t>& cycle_start_keys) {
    if (indices.empty()) {
      return;
    }
    std::vector<std::size_t> sorted_indices = indices;
    const double nominal_cycle_time =
        static_cast<double>(cycle) * config_.track_cycle_period_sec;
    std::stable_sort(
        sorted_indices.begin(), sorted_indices.end(),
        [&detections, nominal_cycle_time](std::size_t a, std::size_t b) {
          const double ta =
              detections[a].has_timestamp ? detections[a].timestamp_s : nominal_cycle_time;
          const double tb =
              detections[b].has_timestamp ? detections[b].timestamp_s : nominal_cycle_time;
          return ta < tb;
        });
    std::vector<std::uint64_t> anchor_track_keys;
    std::vector<EcefPositionM> anchors;
    for (const auto& entry : tracks_) {
      if (cycle_start_keys.count(entry.first) == 0U) {
        continue;
      }
      EcefPositionM ecef{};
      if (entry.second.has_anchor && entry.second.anchor.has_position &&
          oneq::coordinate::TryLlaToEcef(entry.second.anchor.position, &ecef)) {
        anchor_track_keys.push_back(entry.first);
        anchors.push_back(ecef);
      }
    }
    TrackAnchorAdaptor adaptor(anchors);
    TrackKdTree tree(3U, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    if (!anchors.empty()) {
      tree.buildIndex();
    }

    for (const std::size_t i : sorted_indices) {
      const DetectionRecord& detection = detections[i];
      double best_cost = std::numeric_limits<double>::infinity();
      std::uint64_t best_key = 0U;
      bool position_unusable = false;
      if (detection.has_position) {
        EcefPositionM query{};
        if (oneq::coordinate::TryLlaToEcef(detection.position, &query)) {
          for (const std::size_t m : RadiusSearch(tree, query, config_.position_radius_m)) {
            const std::uint64_t track_key = anchor_track_keys[m];
            if (!FeatureGatePasses(detection, tracks_.at(track_key))) {
              continue;
            }
            const double cost = EuclideanDistance(anchors[m], query);
            if (cost < best_cost) {
              best_cost = cost;
              best_key = track_key;
            }
          }
        } else {
          // 位置量测非法（如 LLA 越界）：降级走方位/特征通道。
          position_unusable = true;
        }
      }
      if (position_unusable || !detection.has_position) {
        if (detection.has_bearing) {
          for (const auto& entry : tracks_) {
            if (cycle_start_keys.count(entry.first) == 0U) {
              continue;
            }
            const Track& track = entry.second;
            if (!track.has_anchor || !track.anchor.has_bearing ||
                !FeatureGatePasses(detection, track)) {
              continue;
            }
            if (oneq::common::geometry::AreBearingsCoherent(
                    detection.bearing_az_deg, detection.bearing_el_deg,
                    track.anchor.bearing_az_deg, track.anchor.bearing_el_deg,
                    config_.bearing_beamwidth_deg)) {
              const double cost = oneq::common::geometry::AzimuthShortestDifferenceDeg(
                  detection.bearing_az_deg, track.anchor.bearing_az_deg);
              if (cost < best_cost) {
                best_cost = cost;
                best_key = entry.first;
              }
            }
          }
        } else {
          // 无位置无方位：取首个通过特征门的航迹。
          for (const auto& entry : tracks_) {
            if (cycle_start_keys.count(entry.first) == 0U) {
              continue;
            }
            if (FeatureGatePasses(detection, entry.second)) {
              best_key = entry.first;
              break;
            }
          }
        }
      }

      if (best_key != 0U) {
        AppendSample(&tracks_.at(best_key), detection, cycle);
      } else {
        const std::uint64_t new_key = kSyntheticKeyBase + synthetic_key_counter_++;
        auto it = tracks_.emplace(new_key, Track{new_key}).first;
        AppendSample(&it->second, detection, cycle);
      }
    }
  }

  void PruneWindowsAndAging() {
    auto it = tracks_.begin();
    while (it != tracks_.end()) {
      for (auto& channel : it->second.channel_samples) {
        auto& samples = channel.second;
        while (samples.size() > config_.window_size) {
          samples.pop_front();
        }
      }
      if (it->second.missed_cycles > config_.max_missed_cycles) {
        it = tracks_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // 置信度 = Σ 判决值 × 质量归一化 × 权重（滑窗内精确求和，不归一化，随证据单调累积）。
  double ComputeConfidence(const Track& track) const {
    double confidence = 0.0;
    for (const auto& channel : track.channel_samples) {
      const double weight = SourceWeight(channel.first);
      for (const auto& sample : channel.second) {
        confidence += sample.verdict * sample.quality * weight;
      }
    }
    return confidence;
  }

  double SourceWeight(std::uint32_t source_id) const {
    if (source_id < config_.source_weights.size()) {
      return config_.source_weights[source_id];
    }
    return 1.0;
  }

  // ---- 逐航迹无迹滤波（P2，实现边界见 docs/fusion/algorithms.md §4） ----

  void SetIsotropicCovariance(Track* track, double position_std, double velocity_std) const {
    track->filter_cov = Eigen::Matrix<float, 6, 6>::Zero();
    for (int i = 0; i < 6; i += 2) {
      track->filter_cov(i, i) = static_cast<float>(position_std * position_std);
      track->filter_cov(i + 1, i + 1) = static_cast<float>(velocity_std * velocity_std);
    }
  }

  bool InitFilter(Track* track, const DetectionRecord& detection) const {
    if (detection.has_position) {
      EcefPositionM ecef{};
      if (!oneq::coordinate::TryLlaToEcef(detection.position, &ecef)) {
        return false;
      }
      track->filter_mean(0) = static_cast<float>(ecef.x_m);
      track->filter_mean(1) = 0.0f;
      track->filter_mean(2) = static_cast<float>(ecef.y_m);
      track->filter_mean(3) = 0.0f;
      track->filter_mean(4) = static_cast<float>(ecef.z_m);
      track->filter_mean(5) = 0.0f;
      SetIsotropicCovariance(track, config_.track_initial_position_std_m,
                             config_.track_initial_velocity_std_m_per_s);
      track->filter_initialized = true;
      track->has_first_position = true;
      track->first_position = ecef;
      track->velocity_initialized = false;
      return true;
    }
    if (detection.has_bearing && detection.has_sensor_origin) {
      EcefPositionM origin{};
      if (!oneq::coordinate::TryLlaToEcef(detection.sensor_origin, &origin)) {
        return false;
      }
      const EnuBasis basis = MakeEnuBasis(origin);
      if (!basis.valid) {
        return false;
      }
      // LOS 单位向量（ENU az 自北向东、el 出地平）→ ECEF。
      const double az_rad = DegToRad(detection.bearing_az_deg);
      const double el_rad = DegToRad(detection.bearing_el_deg);
      const double e = std::sin(az_rad) * std::cos(el_rad);
      const double n = std::cos(az_rad) * std::cos(el_rad);
      const double u = std::sin(el_rad);
      track->filter_mean(0) =
          static_cast<float>(origin.x_m + config_.track_bearing_init_range_m *
                                                      (e * basis.east[0] + n * basis.north[0] +
                                                       u * basis.up[0]));
      track->filter_mean(1) = 0.0f;
      track->filter_mean(2) =
          static_cast<float>(origin.y_m + config_.track_bearing_init_range_m *
                                                      (e * basis.east[1] + n * basis.north[1] +
                                                       u * basis.up[1]));
      track->filter_mean(3) = 0.0f;
      track->filter_mean(4) =
          static_cast<float>(origin.z_m + config_.track_bearing_init_range_m *
                                                      (e * basis.east[2] + n * basis.north[2] +
                                                       u * basis.up[2]));
      track->filter_mean(5) = 0.0f;
      SetIsotropicCovariance(track, config_.track_bearing_init_range_std_m,
                             config_.track_initial_velocity_std_m_per_s);
      track->filter_initialized = true;
      return true;
    }
    return false;  // 无位置且无（方位+原点）：不起始滤波器。
  }

  void PredictTrack(Track* track, float dt) const {
    estimation::UnscentedPredictorConfig predictor_config;
    predictor_config.noise_diff_coeff = static_cast<float>(config_.track_process_noise);
    const estimation::LinearCvTransitionModel<6> cv_model;
    const estimation::UnscentedPredictor<6, 2> predictor(&cv_model, predictor_config);
    estimation::GaussianState<6, 2> state;
    state.mean = track->filter_mean;
    state.covariance = track->filter_cov;
    const estimation::GaussianState<6, 2> predicted = predictor.Predict(state, dt);
    track->filter_mean = predicted.mean;
    track->filter_cov = predicted.covariance;
  }

  void ApplyFilterSample(Track* track, const DetectionRecord& detection, std::uint64_t cycle) {
    if (!track->filter_initialized) {
      if (!InitFilter(track, detection)) {
        return;
      }
      track->filter_cycle = cycle;
      track->filter_has_timestamp = detection.has_timestamp;
      track->filter_timestamp_s = detection.has_timestamp ? detection.timestamp_s : 0.0;
      track->first_timestamp_s = track->filter_timestamp_s;
      if (track->has_first_position) {
        // 位置起始：滤波均值即本量测位置，跳过同帧重复量测更新；
        // 初速由下一个位置量测两点差分建立（见下）。
        return;
      }
      // 方位起始：首拍量测照常执行量测更新（角度-only 航迹的距离收敛依赖此更新）。
    }

    // 两点差分初速：位置起始航迹的第二个位置量测以 (p2 − p1)/Δt 直接建立初速，
    // 替代零速先验起步，消除建轨期的速度新息超调。
    if (detection.has_position && !track->velocity_initialized && track->has_first_position) {
      EcefPositionM z_ecef{};
      if (oneq::coordinate::TryLlaToEcef(detection.position, &z_ecef)) {
        double dt_diff = 0.0;
        if (detection.has_timestamp && track->filter_has_timestamp) {
          dt_diff = detection.timestamp_s - track->first_timestamp_s;
        } else {
          dt_diff = static_cast<double>(cycle - track->filter_cycle) * config_.track_cycle_period_sec;
        }
        // 物理间隔过近的两点差分会把位置噪声放大 1/Δt 倍，至少间隔 50 ms 才差分。
        if (dt_diff > 0.05) {
          const double inv_dt = 1.0 / dt_diff;
          const double vx = (z_ecef.x_m - track->first_position.x_m) * inv_dt;
          const double vy = (z_ecef.y_m - track->first_position.y_m) * inv_dt;
          const double vz = (z_ecef.z_m - track->first_position.z_m) * inv_dt;

          track->filter_mean(0) = static_cast<float>(z_ecef.x_m);
          track->filter_mean(1) = static_cast<float>(vx);
          track->filter_mean(2) = static_cast<float>(z_ecef.y_m);
          track->filter_mean(3) = static_cast<float>(vy);
          track->filter_mean(4) = static_cast<float>(z_ecef.z_m);
          track->filter_mean(5) = static_cast<float>(vz);

          const double pos_var =
              config_.default_position_noise_std_m * config_.default_position_noise_std_m;
          // 速度方差 = 2·位置方差/Δt²（两点差分标准传递）+ (100 m/s)² 地板，
          // 地板挡住长间隔差分方差过小、滤波过信差分速度的情形。
          const double vel_var = 2.0 * pos_var * inv_dt * inv_dt + 100.0 * 100.0;
          track->filter_cov = Eigen::Matrix<float, 6, 6>::Zero();
          track->filter_cov(0, 0) = static_cast<float>(pos_var);
          track->filter_cov(1, 1) = static_cast<float>(vel_var);
          track->filter_cov(2, 2) = static_cast<float>(pos_var);
          track->filter_cov(3, 3) = static_cast<float>(vel_var);
          track->filter_cov(4, 4) = static_cast<float>(pos_var);
          track->filter_cov(5, 5) = static_cast<float>(vel_var);

          track->filter_cycle = cycle;
          // 时间基准与常规更新同约定：无时间戳量测不把航迹降级回纯周期模式，
          // 物理时钟按差分间隔从首拍时刻前推。
          if (detection.has_timestamp) {
            track->filter_has_timestamp = true;
            track->filter_timestamp_s = detection.timestamp_s;
          } else if (track->filter_has_timestamp) {
            track->filter_timestamp_s = track->first_timestamp_s + dt_diff;
          }
          track->velocity_initialized = true;
          return;
        }
      }
    } else {
      double dt = 0.0;
      if (detection.has_timestamp && track->filter_has_timestamp) {
        dt = detection.timestamp_s - track->filter_timestamp_s;
      } else {
        dt = static_cast<double>(cycle - track->filter_cycle) * config_.track_cycle_period_sec;
      }
      dt = std::max(0.0, dt);
      if (dt > 0.0) {
        PredictTrack(track, static_cast<float>(dt));
        track->filter_cycle = cycle;
        if (detection.has_timestamp) {
          track->filter_has_timestamp = true;
          track->filter_timestamp_s = detection.timestamp_s;
        } else if (track->filter_has_timestamp) {
          track->filter_timestamp_s += dt;
        }
      } else {
        track->filter_cycle = cycle;
        if (detection.has_timestamp) {
          track->filter_has_timestamp = true;
          track->filter_timestamp_s = std::max(track->filter_timestamp_s, detection.timestamp_s);
        }
      }
    }

    if (detection.has_position) {
      EcefPositionM z_ecef{};
      if (!oneq::coordinate::TryLlaToEcef(detection.position, &z_ecef)) {
        return;
      }
      const estimation::LinearPositionMeasurementModel<6, 3> position_model;
      estimation::UnscentedUpdaterConfig updater_config;
      updater_config.measurement_noise_std =
          static_cast<float>(config_.default_position_noise_std_m);
      const estimation::UnscentedUpdater<6, 3> updater(&position_model, updater_config);
      estimation::GaussianState<6, 3> state;
      state.mean = track->filter_mean;
      state.covariance = track->filter_cov;
      estimation::GaussianState<6, 3>::MeasurementVector z;
      z(0) = static_cast<float>(z_ecef.x_m);
      z(1) = static_cast<float>(z_ecef.y_m);
      z(2) = static_cast<float>(z_ecef.z_m);
      const auto posterior = updater.Update(state, z).posterior;
      track->filter_mean = posterior.mean;
      track->filter_cov = posterior.covariance;
      return;
    }

    if (detection.has_bearing && detection.has_sensor_origin) {
      const double sigma_rad = detection.has_bearing_noise
                                   ? detection.bearing_noise_sigma_rad
                                   : config_.default_bearing_noise_sigma_rad;
      if (!(sigma_rad > 0.0)) {
        return;
      }
      EnuBearingMeasurementModel bearing_model;
      EcefPositionM origin{};
      if (!oneq::coordinate::TryLlaToEcef(detection.sensor_origin, &origin)) {
        return;
      }
      bearing_model.SetGeometry(origin);
      if (!bearing_model.HasValidGeometry()) {
        return;
      }
      estimation::UnscentedUpdaterConfig updater_config;
      const double sigma_deg = RadToDeg(sigma_rad);
      updater_config.measurement_noise_std = static_cast<float>(sigma_deg);
      const estimation::UnscentedUpdater<6, 2> updater(&bearing_model, updater_config);
      estimation::GaussianState<6, 2> state;
      state.mean = track->filter_mean;
      state.covariance = track->filter_cov;
      estimation::GaussianState<6, 2>::MeasurementVector z;
      z(0) = static_cast<float>(detection.bearing_az_deg);
      z(1) = static_cast<float>(detection.bearing_el_deg);
      const auto posterior = updater.Update(state, z).posterior;
      track->filter_mean = posterior.mean;
      track->filter_cov = posterior.covariance;
    }
  }

  void AdvanceFilters(std::uint64_t cycle) {
    if (!config_.enable_track_filtering) {
      return;
    }
    for (auto& entry : tracks_) {
      Track& track = entry.second;
      if (!track.filter_initialized) {
        continue;
      }
      const double dt =
          static_cast<double>(cycle - track.filter_cycle) * config_.track_cycle_period_sec;
      if (dt > 0.0) {
        PredictTrack(&track, static_cast<float>(dt));
        track.filter_cycle = cycle;
        if (track.filter_has_timestamp) {
          track.filter_timestamp_s += dt;
        }
      }
    }
  }

  std::vector<FusedTarget> BuildOutput() const {
    std::vector<FusedTarget> output;
    output.reserve(tracks_.size());
    for (const auto& entry : tracks_) {
      FusedTarget target;
      target.key = entry.first;
      for (const auto& channel : entry.second.channel_samples) {
        ChannelMeasurement measurement;
        measurement.source_id = channel.first;
        const auto& samples = channel.second;
        measurement.sample_count = samples.size();
        if (!samples.empty()) {
          const MeasurementSample& latest = samples.back();
          measurement.source_name = latest.source_name;
          measurement.latest_verdict = latest.verdict;
          measurement.latest_quality = latest.quality;
          measurement.has_position = latest.has_position;
          measurement.position = latest.position;
          measurement.has_bearing = latest.has_bearing;
          measurement.bearing_az_deg = latest.bearing_az_deg;
          measurement.bearing_el_deg = latest.bearing_el_deg;
        }
        target.channels.push_back(std::move(measurement));
      }
      target.confidence = ComputeConfidence(entry.second);
      target.last_update_cycle = entry.second.last_update_cycle;
      const bool confirmed = entry.second.hits >= config_.confirm_hits;
      target.lifecycle =
          confirmed ? (entry.second.missed_cycles > 0U ? FusedTrackLifecycle::kCoasting
                                                        : FusedTrackLifecycle::kConfirmed)
                    : FusedTrackLifecycle::kTentative;
      // 中译：把"是否吃过真实量测"透出给评估侧（合成量测不参与滤波，故仅合成航迹恒 false）。
      // 标识：真实量测透出（2026-10-26）。
      target.updated_by_real_measurement = entry.second.has_real_update;
      target.last_real_measurement_cycle = entry.second.last_real_update_cycle;
      if (config_.enable_track_filtering && entry.second.filter_initialized) {
        const Track& track = entry.second;
        EcefPositionM ecef{};
        ecef.x_m = track.filter_mean(0);
        ecef.y_m = track.filter_mean(2);
        ecef.z_m = track.filter_mean(4);
        if (oneq::coordinate::TryEcefToLla(ecef, &target.kinematic_estimate.position)) {
          target.has_kinematic_estimate = true;
          target.kinematic_estimate.velocity_ecef_m_per_s = {
              static_cast<double>(track.filter_mean(1)),
              static_cast<double>(track.filter_mean(3)),
              static_cast<double>(track.filter_mean(5))};
          for (int row = 0; row < 6; ++row) {
            for (int col = 0; col < 6; ++col) {
              target.kinematic_estimate.covariance_ecef[static_cast<std::size_t>(row) * 6U +
                                                        static_cast<std::size_t>(col)] =
                  static_cast<double>(track.filter_cov(row, col));
            }
          }
        }
      }
      output.push_back(std::move(target));
    }
    return output;
  }

  FusionConfig config_;
  std::map<std::uint64_t, Track> tracks_{};
  std::uint64_t synthetic_key_counter_{0U};
};

FusionEngine::FusionEngine(const FusionConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

FusionEngine::~FusionEngine() = default;

std::vector<FusedTarget> FusionEngine::Update(
    const std::vector<DetectionRecord>& detections, std::uint64_t cycle) {
  return impl_->Update(detections, cycle);
}

void FusionEngine::Reset() { impl_->Reset(); }

}  // namespace fusion
