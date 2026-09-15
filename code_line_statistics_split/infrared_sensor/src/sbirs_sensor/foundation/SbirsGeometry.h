/**
 * @file SbirsGeometry.h
 * @brief SBIRS-inspired 基础几何工具。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_GEOMETRY_H_
#define ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_GEOMETRY_H_

#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace sbirs_sensor {
namespace foundation {

/** @brief 计算三维向量点积。 */
double Dot(const session::SbirsVector3M& lhs, const session::SbirsVector3M& rhs);
/** @brief 计算三维向量模长（L2 范数）。 */
double Norm(const session::SbirsVector3M& value);
/** @brief 计算向量逐分量差 `lhs − rhs`。 */
session::SbirsVector3M Subtract(const session::SbirsVector3M& lhs,
                                const session::SbirsVector3M& rhs);
/** @brief 返回单位向量；零向量行为由实现约定。 */
session::SbirsVector3M Unit(const session::SbirsVector3M& value);
/**
 * @brief 由视线向量计算方位角 Az = atan2(y, x)，单位 deg。
 * @param[in] los 视线向量（卫星→目标，ECI 或 ECEF 分量均可）。
 * @return 输入分量所在参考系的极坐标方位角：相对该系 x 轴（y/x 分量），
 *         取值范围 (-180°, 180°]。SBIRS 输出链路传入 ECI 分量（2026-08 正式
 *         变更），得到 ECI 方位角；内部对称约定由输出边界统一转换为 [0, 2π) rad。
 * @note 该参考系是极坐标（ECI/ECEF 由调用方决定），**非卫星局部地平系**；
 *       场景几何编排（扫描中心/覆盖）须按此参考系配置，约定见
 *       docs/common/session_contract.md。
 */
float ComputeAzimuthDeg(const session::SbirsVector3M& los);
/**
 * @brief 由视线向量计算俯仰角 El = asin(z / |los|)，单位 deg；零向量返回 0。
 * @param[in] los 视线向量（卫星→目标，ECI 或 ECEF 分量均可）。
 * @return 输入分量所在参考系的极坐标俯仰角：相对赤道面（z 为该系 z 轴），
 *         星下点方向 ≈ −90°。SBIRS 输出链路传入 ECI 分量（2026-08 正式变更）；
 *         绕 z 旋转（ECEF↔ECI）不改变该值。
 * @note 该参考系是极坐标（ECI/ECEF 由调用方决定），**非卫星局部地平系**；
 *       场景几何编排（扫描中心/覆盖）须按此参考系配置，约定见
 *       docs/common/session_contract.md。
 */
float ComputeElevationDeg(const session::SbirsVector3M& los);
/** @brief 计算两个 (az, el) 角度对之间的角距离，单位 deg。 */
float AngularSeparationDeg(float az_a_deg, float el_a_deg, float az_b_deg, float el_b_deg);
/**
 * @brief 由相对位置与相对速度推导视线角速度（标量，单位 deg/s）。
 * @param[in] relative_position_m 相对位置向量（卫星→目标），单位 m
 * @param[in] relative_velocity_m_per_s 相对速度向量，单位 m/s
 * @return 视线角速度模长 ω = |v_perp| / range，单位 deg/s；range ≤ 0 时返回 0
 * @note v_perp = v − (v·r̂) r̂ 为速度在视线垂直方向的分量；返回值用于动态滞后误差与 cue 外推。
 */
float ComputeRelativeAngularRateDegPerSec(const session::SbirsVector3M& relative_position_m,
                                          const session::SbirsVector3M& relative_velocity_m_per_s);
/**
 * @brief 用有限 LOS 线段判定目标视线是否被地球球体遮挡。
 * @param[in] satellite_position_ecef_m 卫星 ECEF 位置，单位 m
 * @param[in] target_position_ecef_m 目标 ECEF 位置，单位 m
 * @param[in] earth_radius_m 地球半径，单位 m
 * @return 视线在卫星到目标的有限线段内穿过地球球体返回 true，否则返回 false
 * @note 仅做球体相交判定，不负责地形、云图或三维大气廓线。
 *       判定核在 `oneq::common::geometry::IsEarthOcculted`（相切视为遮挡）。
 */
bool IsEarthOcculted(const session::SbirsVector3M& satellite_position_ecef_m,
                     const session::SbirsVector3M& target_position_ecef_m, double earth_radius_m);

/**
 * @brief 计算有限 LOS 线段相对地球球体的遮挡余量（规则 13b 门内归因量值）。
 * @param[in] satellite_position_ecef_m 卫星 ECEF 位置，单位 m
 * @param[in] target_position_ecef_m 目标 ECEF 位置，单位 m
 * @param[in] earth_radius_m 地球半径，单位 m
 * @return 视线最近接近距离 − 地球半径，单位 m；负值表示遮挡深度（视线穿过球体），
 *         正值表示余量；无有效相交几何时返回 earth_radius_m（视为无遮挡）。
 * @note 与 IsEarthOcculted 同几何判定，仅额外返回数值余量供排除诊断携带。
 *       判定核在 `oneq::common::geometry::ComputeEarthOccultationMarginM`。
 */
double ComputeEarthOccultationMarginM(const session::SbirsVector3M& satellite_position_ecef_m,
                                      const session::SbirsVector3M& target_position_ecef_m,
                                      double earth_radius_m);

/**
 * @brief 半无限射线与球面求交（WFOV 地面覆盖区投影原语）。
 * @param[in] origin_m 射线起点（参考系不限，与球心同系），单位 m
 * @param[in] direction_m 射线方向（无需单位化），同参考系
 * @param[in] sphere_radius_m 球体半径，单位 m；球心即该参考系原点
 * @param[out] intersection_m 最近正根交点，同参考系，单位 m
 * @return 存在 t > 0 的交点返回 true，否则 false（射线背离球面或相切于负根）。
 * @note 与 IsEarthOcculted 同一圆球地球模型口径（相切视为相交）；起点在球内时返回
 *       出射点。供验收日志的覆盖区四角投影使用，不改变门控行为。
 */
bool TryIntersectRayWithSphere(const session::SbirsVector3M& origin_m,
                               const session::SbirsVector3M& direction_m, double sphere_radius_m,
                               session::SbirsVector3M* intersection_m);

/**
 * @brief 由 ECEF 位置计算地心纬度/经度（WFOV 地面覆盖区投影坐标）。
 * @param[in] position_ecef_m ECEF 位置，单位 m
 * @param[out] latitude_deg 地心纬度，单位 deg，[-90, 90]
 * @param[out] longitude_deg 经度，单位 deg，(-180, 180]
 * @note 圆球模型下地心纬度即大地纬度；零向量时两输出置 0（与 ComputeElevationDeg
 *       的零向量约定一致）。椭球（WGS84）投影为已登记非目标（见 boundaries.md）。
 */
void ComputeGeocentricLatLonDeg(const session::SbirsVector3M& position_ecef_m,
                                double* latitude_deg, double* longitude_deg);

/**
 * @brief 卫星视线方向与地球圆球交会的地面经纬度（WFOV 覆盖区投影组合原语）。
 * @param[in] satellite_position_eci_m 卫星 ECI 位置，单位 m
 * @param[in] direction_eci 视线方向（ECI 单位向量，无需单位化）
 * @param[in] earth_radius_m 地球半径，单位 m（与遮挡判定同值传入）
 * @param[in] gmst_rad 本周期 GMST，单位 rad（ECI→ECEF 旋转用）
 * @param[out] latitude_deg 交会点地心纬度，单位 deg
 * @param[out] longitude_deg 交会点经度，单位 deg
 * @return 射线与地球相交且坐标转换成功返回 true，否则 false（指向太空的角标记 miss）。
 */
bool TryComputeGroundIntersectionLatLonDeg(const session::SbirsVector3M& satellite_position_eci_m,
                                           const session::SbirsVector3M& direction_eci,
                                           double earth_radius_m, double gmst_rad,
                                           double* latitude_deg, double* longitude_deg);

/** @brief 焦平面脱靶量：目标像点相对焦平面中心的偏移（NFOV 跟踪验收量）。 */
struct SbirsFocalPlaneOffset {
  double x_m{0.0};        /**< 方位轴偏移 f·tan(Δaz)，单位 m */
  double y_m{0.0};        /**< 俯仰轴偏移 f·tan(Δel)，单位 m */
  double x_pixels{0.0};   /**< 方位轴偏移像素数 = x_m / 像元间距 */
  double y_pixels{0.0};   /**< 俯仰轴偏移像素数 = y_m / 像元间距 */
};

/**
 * @brief 由目标相对 NFOV 指向中心的角偏差映射焦平面脱靶量（x = f·tan(Δaz)）。
 * @param[in] focal_length_m 光学焦距，单位 m（>0）
 * @param[in] pixel_pitch_m 探测器像元间距，单位 m（>0）
 * @param[in] delta_az_deg 方位角偏差（目标 − 指向中心），单位 deg
 * @param[in] delta_el_deg 俯仰角偏差（目标 − 指向中心），单位 deg
 * @param[out] offset 脱靶量输出
 * @return 参数合法且 offset 非空返回 true；焦距/像元间距非正返回 false（调用方跳过该字段）。
 * @note 逐轴独立小角投影（f·tan），非畸变光学模型；仅验收日志消费，不进公开输出。
 */
bool ComputeFocalPlaneOffset(double focal_length_m, double pixel_pitch_m, float delta_az_deg,
                             float delta_el_deg, SbirsFocalPlaneOffset* offset);

}  // namespace foundation
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_GEOMETRY_H_
