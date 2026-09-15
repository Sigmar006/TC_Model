/**
 * @file PolarizationStatsExtractor.h
 * @brief 极化散射矩阵统计特征提取器（识别链，2026-09-03 验收旁路转正）。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_RECOGNITION_POLARIZATION_STATS_EXTRACTOR_H_
#define REMOTE_IDENTIFICATION_RADAR_RECOGNITION_POLARIZATION_STATS_EXTRACTOR_H_

#include <vector>

#include "1q/remote_identification_radar/session/RirSceneTypes.h"

namespace remote_identification_radar {
namespace recognition {

/** @brief RirPolarizationQuantityStats 单个统计量（均值+标准差）。 */
struct RirPolarizationQuantityStats {
  double mean{0.0};
  double std{0.0};
};

/**
 * @brief RirPolarizationStatsObservation 极化散射矩阵五量统计观测。
 * @note 统计总体＝窗口行（当前视角附近的姿态扇区）。psi_deg 为圆统计口径：
 *       mean 是均值角（deg）、std 是角度散布（deg）；其余四量为算术均值与
 *       样本标准差（n=1 时 std=0）。
 */
struct RirPolarizationStatsObservation {
  bool valid{false};
  RirPolarizationQuantityStats determinant;    /**< |det S|，det=HH·VV−HV·VH。 */
  RirPolarizationQuantityStats span;           /**< 总功率 σHH+σHV+σVH+σVV。 */
  RirPolarizationQuantityStats depolarization; /**< 交叉占比 (σHV+σVH)/span。 */
  RirPolarizationQuantityStats psi_deg;        /**< Graves 本征极化方向角（圆统计）。 */
  RirPolarizationQuantityStats tau_deg;        /**< Graves 本征极化椭圆率（算术）。 */
};

/**
 * @brief 方向角圆统计（周期 180°，ψ 与 ψ±180° 同向）。
 * @param[in] angles_deg 方向角样本（deg）。
 * @return mean=均值角（(−90°, 90°]）；std=角度散布 ½·√(−2·ln R̄)（deg，
 *         R̄ 为倍角单位向量平均长度；样本完全同向时 0）。
 */
RirPolarizationQuantityStats CircularMeanStdDeg(const std::vector<double>& angles_deg);

/**
 * @brief RirPolarizationStatsExtractor 从窗口行提取五量统计特征。
 * @note 逐行由四路幅度（dBsm→线性 σ）与相位构造复 S 矩阵，派生
 *       |det|/Span/去极化/Graves ψτ 后统计；任一字段非有限或 Span≤0 的行
 *       整行跳过；无有效行时 valid=false（fail-closed，不冒充）。
 */
class RirPolarizationStatsExtractor {
 public:
  static RirPolarizationStatsObservation Extract(
      const std::vector<session::RirPolSMatrixSample>& window);
};

}  // namespace recognition
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_RECOGNITION_POLARIZATION_STATS_EXTRACTOR_H_
