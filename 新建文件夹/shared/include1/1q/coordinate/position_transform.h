/**
 * @file position_transform.h
 * @brief 定义 WGS-84 坐标系下位置类型的帧间转换。
 *
 * 本文件提供 LLA ↔ ECEF、ECEF/LLA → ENU/NED/NUE 以及三种局部坐标系间的
 * 纯轴重排转换。所有转换遵循 `Try` 模式，输入非法或输出为空时返回 false。
 */

#ifndef ONEQ_COORDINATE_POSITION_TRANSFORM_H_
#define ONEQ_COORDINATE_POSITION_TRANSFORM_H_

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace oneq {
namespace coordinate {

// =============================================================================
// 输入校验
// =============================================================================

/**
 * @brief 校验 LLA 纬度/经度范围及有限性。
 * @param[in] lla 待校验的大地坐标。
 * @return 纬度在 [-90, 90]、经度在 [-180, 180] 且高度有限时返回 true，否则返回 false。
 */
ONEQ_API bool IsValid(const LlaPositionDegM& lla);

/**
 * @brief 校验 ECEF 各分量均为有限值。
 * @param[in] ecef 待校验的 ECEF 坐标。
 * @return 三个分量均为有限值返回 true，否则返回 false。
 */
ONEQ_API bool IsFinite(const EcefPositionM& ecef);

/**
 * @brief 校验 ENU 各分量均为有限值。
 * @param[in] enu 待校验的 ENU 坐标。
 * @return 三个分量均为有限值返回 true，否则返回 false。
 */
ONEQ_API bool IsFinite(const EnuPositionM& enu);

/**
 * @brief 校验 NED 各分量均为有限值。
 * @param[in] ned 待校验的 NED 坐标。
 * @return 三个分量均为有限值返回 true，否则返回 false。
 */
ONEQ_API bool IsFinite(const NedPositionM& ned);

/**
 * @brief 校验 NUE 各分量均为有限值。
 * @param[in] nue 待校验的 NUE 坐标。
 * @return 三个分量均为有限值返回 true，否则返回 false。
 */
ONEQ_API bool IsFinite(const NuePositionM& nue);

// =============================================================================
// LLA ↔ ECEF
// =============================================================================

/**
 * @brief 将 WGS84 大地坐标 LLA 转换为地心地固 ECEF 直角坐标。
 * @param[in] lla 输入大地坐标（单位：deg / m）。
 * @param[out] ecef 输出 ECEF；可为 nullptr。
 * @return 成功返回 true。
 */
ONEQ_API bool TryLlaToEcef(const LlaPositionDegM& lla, EcefPositionM* ecef);

/**
 * @brief 将地心地固 ECEF 直角坐标转换为 WGS84 大地坐标 LLA。
 * @param[in] ecef 输入 ECEF（单位：m）。
 * @param[out] lla 输出 LLA；可为 nullptr。
 * @return 成功返回 true。
 */
ONEQ_API bool TryEcefToLla(const EcefPositionM& ecef, LlaPositionDegM* lla);

// =============================================================================
// ECEF → 局部坐标系
// =============================================================================

/**
 * @brief 将 ECEF 位置转换到以 origin_lla 为参考点的 ENU 坐标。
 * @param[in] ecef ECEF 输入（单位：m）。
 * @param[in] origin_lla 局部 ENU 原点（WGS84 LLA）。
 * @param[out] enu 输出 ENU；可为 nullptr。
 * @return 成功返回 true。
 */
ONEQ_API bool TryEcefToEnu(const EcefPositionM& ecef,
                           const LlaPositionDegM& origin_lla,
                           EnuPositionM* enu);

/**
 * @brief 将 ECEF 位置转换到以 origin_lla 为参考点的 NED 坐标。
 * @param[in] ecef ECEF 输入（单位：m）。
 * @param[in] origin_lla 局部 NED 原点（WGS84 LLA）。
 * @param[out] ned 输出 NED；可为 nullptr。
 * @return 成功返回 true。
 * @note 内部经由 TryEcefToEnu + ToNed 实现。
 */
ONEQ_API bool TryEcefToNed(const EcefPositionM& ecef,
                           const LlaPositionDegM& origin_lla,
                           NedPositionM* ned);

/**
 * @brief 将 ECEF 位置转换到以 origin_lla 为参考点的 NUE 坐标。
 * @param[in] ecef ECEF 输入（单位：m）。
 * @param[in] origin_lla 局部 NUE 原点（WGS84 LLA）。
 * @param[out] nue 输出 NUE；可为 nullptr。
 * @return 成功返回 true。
 * @note 内部经由 TryEcefToEnu + ToNue 实现。
 */
ONEQ_API bool TryEcefToNue(const EcefPositionM& ecef,
                           const LlaPositionDegM& origin_lla,
                           NuePositionM* nue);

// =============================================================================
// LLA → 局部坐标系
// =============================================================================

/**
 * @brief 将 WGS84 LLA 位置转换到以 origin_lla 为参考点的 ENU 坐标。
 * @param[in] lla 输入 LLA（单位：deg / m）。
 * @param[in] origin_lla 局部 ENU 原点（WGS84 LLA）。
 * @param[out] enu 输出 ENU；可为 nullptr。
 * @return 成功返回 true。
 * @note 内部 LLA → ECEF → ENU，两次调用无额外精度损失。
 */
ONEQ_API bool TryLlaToEnu(const LlaPositionDegM& lla,
                          const LlaPositionDegM& origin_lla,
                          EnuPositionM* enu);

/**
 * @brief 将 WGS84 LLA 位置转换到以 origin_lla 为参考点的 NED 坐标。
 * @param[in] lla 输入 LLA。
 * @param[in] origin_lla 局部 NED 原点。
 * @param[out] ned 输出 NED；可为 nullptr。
 * @return 成功返回 true。
 */
ONEQ_API bool TryLlaToNed(const LlaPositionDegM& lla,
                          const LlaPositionDegM& origin_lla,
                          NedPositionM* ned);

/**
 * @brief 将 WGS84 LLA 位置转换到以 origin_lla 为参考点的 NUE 坐标。
 * @param[in] lla 输入 LLA。
 * @param[in] origin_lla 局部 NUE 原点。
 * @param[out] nue 输出 NUE；可为 nullptr。
 * @return 成功返回 true。
 */
ONEQ_API bool TryLlaToNue(const LlaPositionDegM& lla,
                          const LlaPositionDegM& origin_lla,
                          NuePositionM* nue);

// =============================================================================
// 局部坐标系 → ECEF（反向转换）
// =============================================================================

/**
 * @brief 将 ENU 位置转换回 ECEF 坐标。
 * @param[in] enu ENU 输入（单位：m）。
 * @param[in] origin_lla 局部 ENU 原点（WGS84 LLA）。
 * @param[out] ecef 输出 ECEF；可为 nullptr。
 * @return 成功返回 true。
 * @note 为 TryEcefToEnu 的逆运算。
 */
ONEQ_API bool TryEnuToEcef(const EnuPositionM& enu,
                           const LlaPositionDegM& origin_lla,
                           EcefPositionM* ecef);

/**
 * @brief 方位（北偏东）与水平距离 → ENU 水平偏移（up=0）。
 * @param[in] azimuth_deg 方位角（deg，北偏东：0=北、90=东）。
 * @param[in] range_m 水平距离（m，非负）。
 * @param[out] enu_offset 输出 ENU 偏移；可为 nullptr。
 * @return 成功返回 true。
 * @note 纯几何单函数（east=range*sin、north=range*cos），与 TryEnuToEcef
 *       组合即得目标 ECEF 位置；垂向偏移/高度由调用方业务层决定。
 */
ONEQ_API bool TryBearingRangeToEnuOffset(double azimuth_deg, double range_m,
                                         EnuPositionM* enu_offset);

/**
 * @brief 将 ENU 方向向量转换到 ECEF 方向（不含原点平移）。
 * @param[in] enu_dir ENU 方向向量（单位：m/m）。
 * @param[in] origin_lla 参考点 LLA，决定局部坐标轴方向。
 * @param[out] ecef_dir 输出 ECEF 方向向量；可为 nullptr。
 * @return 成功返回 true。
 * @note 与 TryEnuToEcef 的区别在于不叠加原点 ECEF 偏移，
 *       适用于方位/指向等方向量的坐标系变换。
 */
ONEQ_API bool TryEnuToEcefDirection(const Vector3d& enu_dir,
                                    const LlaPositionDegM& origin_lla,
                                    Vector3d* ecef_dir);

// =============================================================================
// 局部坐标系互转（轴重排，无精度损失）
// =============================================================================

/**
 * @brief ENU 位置 → NED 位置。
 * @details 映射关系：NED.north = ENU.north，NED.east = ENU.east，NED.down = -ENU.up。
 */
ONEQ_API NedPositionM ToNed(const EnuPositionM& enu);

/**
 * @brief NED 位置 → ENU 位置。
 * @details 映射关系：ENU.east = NED.east，ENU.north = NED.north，ENU.up = -NED.down。
 */
ONEQ_API EnuPositionM ToEnu(const NedPositionM& ned);

/**
 * @brief ENU 位置 → NUE 位置。
 * @details 映射关系：NUE.north = ENU.north，NUE.up = ENU.up，NUE.east = ENU.east。
 */
ONEQ_API NuePositionM ToNue(const EnuPositionM& enu);

/**
 * @brief NUE 位置 → ENU 位置。
 * @details 映射关系：ENU.east = NUE.east，ENU.north = NUE.north，ENU.up = NUE.up。
 */
ONEQ_API EnuPositionM ToEnu(const NuePositionM& nue);

/**
 * @brief NED 位置 → NUE 位置。
 * @details 映射关系：NUE.north = NED.north，NUE.up = -NED.down，NUE.east = NED.east。
 */
ONEQ_API NuePositionM ToNue(const NedPositionM& ned);

/**
 * @brief NUE 位置 → NED 位置。
 * @details 映射关系：NED.north = NUE.north，NED.east = NUE.east，NED.down = -NUE.up。
 */
ONEQ_API NedPositionM ToNed(const NuePositionM& nue);

}  // namespace coordinate
}  // namespace oneq

#endif  // ONEQ_COORDINATE_POSITION_TRANSFORM_H_
