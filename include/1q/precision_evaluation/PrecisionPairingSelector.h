/**
 * @file PrecisionPairingSelector.h
 * @brief 精度评估的双星配对选择：把"本周期用哪两颗星做双星交会"独立成可替换策略。
 *
 * 地面站按时刻对任意两星配对，所以配对不应写死在调用点。本文件只负责**选星**，
 * 不碰评估算式：宿主每周期把"本周期推了帧的星 + 其检测"交给 `SelectSatellitePair`，
 * 拿回主/辅两星，再装配两个 `PrecisionSatelliteCycleInput` 调
 * `PrecisionEvaluationSession::Step`。
 *
 * 两种策略：
 * - `kFixedSources`：固定 A/B 源（口径稳定、可复现；某颗星掉线则该周期无样本）；
 * - `kLongestBaseline`：在本周期**都有同一目标窄场检出**的星里，取基线（两星间距）
 *   最长的一对；并列时按源编号升序取字典序最小的一对（结果确定、可复现）。
 *
 * @note 会话的 A/B 只是调用内角色；锚点缓存自带卫星位置与 GMST，故跨周期换物理星
 *       不会破坏交会几何（见 `PrecisionEvaluationSession.cpp` 的锚点实现）。
 * @note 选星只看"是否有该目标的窄场检出"（宽场搜索样本会被会话剔除，故不参与）；
 *       真正进交会/误差统计的目标仍由 `Step` 内部决定。
 */

#ifndef ONEQ_PRECISION_EVALUATION_PRECISION_PAIRING_SELECTOR_H_
#define ONEQ_PRECISION_EVALUATION_PRECISION_PAIRING_SELECTOR_H_

#include <cmath>
#include <cstdint>
#include <set>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/precision_evaluation/PrecisionEvaluationInputs.h"

namespace precision_evaluation {

/** @brief 双星配对策略。 */
enum class PrecisionPairingStrategy {
  kFixedSources = 0, /**< 固定 A/B 源编号 */
  kLongestBaseline,  /**< 每周期取基线最长的可配星对 */
};

/** @brief 参与配对的单星本周期输入（选星只看源编号、位置、检测）。 */
struct ONEQ_API PairingSatelliteInput {
  std::uint32_t source_id{0U};                                   /**< 源通道标识 */
  oneq::coordinate::EcefPositionM position_ecef_m{};              /**< 卫星 ECEF 位置（m） */
  std::vector<PrecisionDetectionInput> detections{};              /**< 本周期检测 */
};

/** @brief 配对配置（两种策略共用一个结构，便于运行期切换）。 */
struct ONEQ_API PrecisionPairingConfig {
  PrecisionPairingStrategy strategy{PrecisionPairingStrategy::kLongestBaseline};
  std::uint32_t fixed_source_a{4U};   /**< `kFixedSources` 的主星源编号 */
  std::uint32_t fixed_source_b{104U}; /**< `kFixedSources` 的辅星源编号 */
};

/** @brief 配对结果。 */
struct ONEQ_API PrecisionSatellitePair {
  bool valid{false};          /**< 是否选出可用的一对 */
  std::size_t index_a{0U};    /**< 主星在入参向量中的下标 */
  std::size_t index_b{0U};    /**< 辅星在入参向量中的下标 */
  std::uint32_t source_a{0U}; /**< 主星源编号（总为两者较小者） */
  std::uint32_t source_b{0U}; /**< 辅星源编号 */
  std::uint64_t common_target_id{0U}; /**< 两星共同可见的最小目标 ID（无共同目标时为 0） */
  double baseline_m{0.0};     /**< 两星间距（m）；固定策略下同样填，便于对比 */
};

namespace detail {

/** @brief 该星本周期有窄场检出的目标集合（宽场搜索样本不进交会，故不参与配对）。 */
inline std::set<std::uint64_t> PairingTargets(const PairingSatelliteInput& satellite) {
  std::set<std::uint64_t> targets;
  for (const PrecisionDetectionInput& detection : satellite.detections) {
    if (!detection.detected || detection.target_id == 0U) {
      continue;
    }
    if (detection.observation_stage ==
        sbirs_sensor::output::SbirsObservationStage::kWideFieldSearch) {
      continue;
    }
    targets.insert(detection.target_id);
  }
  return targets;
}

inline double PairingBaselineM(const PairingSatelliteInput& left,
                               const PairingSatelliteInput& right) {
  const double dx = left.position_ecef_m.x_m - right.position_ecef_m.x_m;
  const double dy = left.position_ecef_m.y_m - right.position_ecef_m.y_m;
  const double dz = left.position_ecef_m.z_m - right.position_ecef_m.z_m;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/** @brief 按源编号升序把两星排成主/辅，保证 A 始终是编号较小者。 */
inline void OrderBySource(std::size_t left, std::size_t right,
                          const std::vector<PairingSatelliteInput>& satellites,
                          PrecisionSatellitePair* pair) {
  const bool left_first = satellites[left].source_id < satellites[right].source_id;
  pair->index_a = left_first ? left : right;
  pair->index_b = left_first ? right : left;
  pair->source_a = satellites[pair->index_a].source_id;
  pair->source_b = satellites[pair->index_b].source_id;
  pair->baseline_m = PairingBaselineM(satellites[pair->index_a], satellites[pair->index_b]);
}

}  // namespace detail

/**
 * @brief 选出一对本周期用于双星评估的星。
 *
 * @param[in] config 配对配置（策略与固定源编号）
 * @param[in] satellites 本周期推了帧的星（顺序任意；重复源编号只取第一条）
 * @return 配对结果；`valid=false` 时宿主应跳过本周期的评估采样（不填零样本）
 */
inline PrecisionSatellitePair SelectSatellitePair(
    const PrecisionPairingConfig& config,
    const std::vector<PairingSatelliteInput>& satellites) {
  PrecisionSatellitePair pair;

  // 去重：同一源只保留第一条（数组顺序即宿主填充顺序，确定可复现）。
  std::vector<std::size_t> unique;
  std::set<std::uint32_t> seen;
  for (std::size_t index = 0U; index < satellites.size(); ++index) {
    if (seen.insert(satellites[index].source_id).second) {
      unique.push_back(index);
    }
  }
  if (unique.size() < 2U) {
    return pair;
  }

  if (config.strategy == PrecisionPairingStrategy::kFixedSources) {
    std::size_t index_a = unique.size();
    std::size_t index_b = unique.size();
    for (std::size_t index : unique) {
      if (satellites[index].source_id == config.fixed_source_a) {
        index_a = index;
      } else if (satellites[index].source_id == config.fixed_source_b) {
        index_b = index;
      }
    }
    if (index_a == unique.size() || index_b == unique.size()) {
      return pair;  // 固定源缺一：本周期无样本。
    }
    detail::OrderBySource(index_a, index_b, satellites, &pair);
    const std::set<std::uint64_t> targets_a = detail::PairingTargets(satellites[index_a]);
    const std::set<std::uint64_t> targets_b = detail::PairingTargets(satellites[index_b]);
    for (std::uint64_t target : targets_a) {
      if (targets_b.count(target) != 0U) {
        pair.common_target_id = target;
        break;
      }
    }
    pair.valid = true;
    return pair;
  }

  // kLongestBaseline：只在"有共同窄场目标"的星对里挑基线最长者。
  bool found = false;
  double best_baseline_m = 0.0;
  for (std::size_t i = 0U; i < unique.size(); ++i) {
    const std::set<std::uint64_t> targets_i = detail::PairingTargets(satellites[unique[i]]);
    if (targets_i.empty()) {
      continue;
    }
    for (std::size_t j = i + 1U; j < unique.size(); ++j) {
      const std::set<std::uint64_t> targets_j = detail::PairingTargets(satellites[unique[j]]);
      if (targets_j.empty()) {
        continue;
      }
      std::uint64_t common_target = 0U;
      for (std::uint64_t target : targets_i) {
        if (targets_j.count(target) != 0U) {
          common_target = target;
          break;
        }
      }
      if (common_target == 0U) {
        continue;
      }
      const double baseline_m =
          detail::PairingBaselineM(satellites[unique[i]], satellites[unique[j]]);
      // 严格大于才替换 ⇒ 并列时保留先遇到的一对；unique 按源编号升序去重，
      // 先遇到的即字典序（source_a, source_b）最小的一对，结果确定。
      if (!found || baseline_m > best_baseline_m) {
        found = true;
        best_baseline_m = baseline_m;
        detail::OrderBySource(unique[i], unique[j], satellites, &pair);
        pair.common_target_id = common_target;
      }
    }
  }
  pair.valid = found;
  return pair;
}

}  // namespace precision_evaluation

#endif  // ONEQ_PRECISION_EVALUATION_PRECISION_PAIRING_SELECTOR_H_
