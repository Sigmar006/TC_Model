#include "electronic_surveillance_radar/pipeline/KdTreeClusterer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <nanoflann.hpp>
#include <unordered_set>
#include <utility>
#include <vector>

namespace electronic_surveillance_radar {
namespace pipeline {

namespace {

/**
 * @brief KdTreeDatasetAdaptor 把特征数组适配为 nanoflann 数据集接口。
 */
class KdTreeDatasetAdaptor {
 public:
  explicit KdTreeDatasetAdaptor(const std::vector<ObservationFeatureVector>& features)
      : features_(features) {}

  inline std::size_t kdtree_get_point_count() const { return features_.size(); }

  inline float kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
    if (idx >= features_.size() || dim >= kObservationFeatureDimension) {
      return 0.0f;
    }
    return features_[idx].values[dim];
  }

  // nanoflann 1.2.3（VS2013 降级版本）的 L2_Simple_Adaptor 经由 kdtree_distance 求
  // 查询点到数据点的平方距离；1.3.x 改用 kdtree_get_pt，此接口被忽略，故两版本皆安全。
  inline float kdtree_distance(const float* query, const std::size_t idx, std::size_t size) const {
    float dist_sq = 0.0f;
    for (std::size_t i = 0; i < size; ++i) {
      const float diff = query[i] - features_[idx].values[i];
      dist_sq += diff * diff;
    }
    return dist_sq;
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }

 private:
  const std::vector<ObservationFeatureVector>& features_;
};

/**
 * @brief 执行 KD-tree 半径搜索。
 * @param[in] index KD-tree 索引。
 * @param[in] feature 查询特征。
 * @param[in] radius 半径阈值。
 * @return 邻域索引列表（升序）。
 */
template <typename IndexType>
std::vector<std::size_t> RadiusSearch(const IndexType& index,
                                      const ObservationFeatureVector& feature, float radius) {
  using KdDistanceType = typename IndexType::DistanceType;
// nanoflann 1.3.x 与 1.5+ 在搜索参数与结果容器类型上存在 API 差异。
#if defined(NANOFLANN_VERSION) && (NANOFLANN_VERSION >= 0x150)
  using KdIndexType = typename IndexType::IndexType;
  std::vector<nanoflann::ResultItem<KdIndexType, KdDistanceType>> matches;
  nanoflann::SearchParameters search_params;
#else
  using KdIndexType = std::size_t;
  std::vector<std::pair<KdIndexType, KdDistanceType>> matches;
  nanoflann::SearchParams search_params;
#endif
  search_params.sorted = true;
  const KdDistanceType radius_sq = static_cast<KdDistanceType>(radius * radius);
  index.radiusSearch(feature.values.data(), radius_sq, matches, search_params);

  std::vector<std::size_t> neighbor_indices;
  neighbor_indices.reserve(matches.size());
  for (std::size_t i = 0; i < matches.size(); ++i) {
    neighbor_indices.push_back(static_cast<std::size_t>(matches[i].first));
  }
  return neighbor_indices;
}

}  // namespace

KdTreeClusterResult KdTreeClusterer::Cluster(const std::vector<ObservationFeatureVector>& features,
                                             const extension::InterceptClusterConfig& config) const {
  KdTreeClusterResult result;
  if (features.empty()) {
    return result;
  }

  const float cluster_radius = config.radius > 0.0f ? config.radius : 1.0f;
  const std::size_t min_points = std::max<std::size_t>(
      1U, std::min<std::size_t>(static_cast<std::size_t>(config.min_points), features.size()));

  KdTreeDatasetAdaptor adaptor(features);
  using KdTreeIndex =
      nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, KdTreeDatasetAdaptor>,
                                          KdTreeDatasetAdaptor, kObservationFeatureDimension,
                                          std::size_t>;
  KdTreeIndex index(kObservationFeatureDimension, adaptor,
                    nanoflann::KDTreeSingleIndexAdaptorParams(10));
  index.buildIndex();

  std::vector<std::int32_t> labels(features.size(), -1);
  std::vector<std::uint8_t> visited(features.size(), 0U);

  for (std::size_t i = 0; i < features.size(); ++i) {
    if (visited[i] != 0U) {
      continue;
    }
    visited[i] = 1U;

    std::vector<std::size_t> neighbors = RadiusSearch(index, features[i], cluster_radius);
    if (neighbors.size() < min_points) {
      continue;
    }

    const std::size_t cluster_id = result.clusters.size();
    result.clusters.push_back(std::vector<std::size_t>());

    std::deque<std::size_t> queue(neighbors.begin(), neighbors.end());
    std::unordered_set<std::size_t> in_queue(neighbors.begin(), neighbors.end());
    while (!queue.empty()) {
      const std::size_t candidate = queue.front();
      queue.pop_front();
      in_queue.erase(candidate);

      if (visited[candidate] == 0U) {
        visited[candidate] = 1U;
        const std::vector<std::size_t> candidate_neighbors =
            RadiusSearch(index, features[candidate], cluster_radius);
        if (candidate_neighbors.size() >= min_points) {
          for (std::size_t j = 0; j < candidate_neighbors.size(); ++j) {
            if (in_queue.find(candidate_neighbors[j]) == in_queue.end()) {
              queue.push_back(candidate_neighbors[j]);
              in_queue.insert(candidate_neighbors[j]);
            }
          }
        }
      }

      if (labels[candidate] == -1) {
        labels[candidate] = static_cast<std::int32_t>(cluster_id);
        result.clusters[cluster_id].push_back(candidate);
      }
    }
  }

  for (std::size_t i = 0; i < result.clusters.size(); ++i) {
    std::sort(result.clusters[i].begin(), result.clusters[i].end());
  }
  for (std::size_t i = 0; i < labels.size(); ++i) {
    if (labels[i] == -1) {
      result.noise_indices.push_back(i);
    }
  }
  std::sort(result.noise_indices.begin(), result.noise_indices.end());
  return result;
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar
