/**
 * @file EarthOccultation.h
 * @brief 有限视线弦与地球圆球的遮挡判定（common 单源）。
 */

#ifndef COMMON_GEOMETRY_EARTH_OCCULTATION_H_
#define COMMON_GEOMETRY_EARTH_OCCULTATION_H_

#include "1q/coordinate/types.h"

namespace oneq {
namespace common {
namespace geometry {

/** @brief 圆球地球平均半径（单位：m），与历史 SBIRS `kEarthRadiusM` 同值。 */
constexpr double kMeanEarthRadiusM = 6371000.0;

/**
 * @brief 计算有限 LOS 线段相对地球球体的遮挡余量。
 * @param[in] observer_position_ecef_m 观测点 ECEF（或同原点惯性系分量），单位 m
 * @param[in] target_position_ecef_m 目标 ECEF（或同系分量），单位 m
 * @param[in] earth_radius_m 地球半径，单位 m
 * @return 视线最近接近距离 − 地球半径，单位 m；负值表示遮挡深度（视线穿过球体），
 *         正值表示余量；无有效相交几何时返回 earth_radius_m（视为无遮挡）。
 * @note 相切（余量 == 0）由 IsEarthOcculted 视为遮挡。段外最近点视为无遮挡，
 *       因此观测点看向球面上的对地目标本身不会被误判为穿地。
 */
double ComputeEarthOccultationMarginM(const oneq::coordinate::EcefPositionM& observer_position_ecef_m,
                                      const oneq::coordinate::EcefPositionM& target_position_ecef_m,
                                      double earth_radius_m);

/**
 * @brief 用有限 LOS 线段判定目标视线是否被地球球体遮挡。
 * @return 视线在观测点到目标的有限线段内穿过或相切地球球体返回 true。
 * @note 仅做球体相交判定，不负责地形、椭球、云图或电波折射（k 因子）。
 */
bool IsEarthOcculted(const oneq::coordinate::EcefPositionM& observer_position_ecef_m,
                     const oneq::coordinate::EcefPositionM& target_position_ecef_m,
                     double earth_radius_m);

}  // namespace geometry
}  // namespace common
}  // namespace oneq

#endif  // COMMON_GEOMETRY_EARTH_OCCULTATION_H_
