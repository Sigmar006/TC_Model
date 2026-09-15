/**
 * @file attitude_transform.h
 * @brief 定义欧拉角与旋转矩阵之间的转换工具。
 *
 * 本模块采用 Z-Y-X 旋转顺序，正 pitch 表示正仰角。
 * 使用显式 3x3 矩阵结构体避免公共头暴露 Eigen。
 * 提供 ComposeAttitudeDeg 将双重姿态复合为单一欧拉角。
 */

#ifndef ONEQ_COORDINATE_ATTITUDE_TRANSFORM_H_
#define ONEQ_COORDINATE_ATTITUDE_TRANSFORM_H_

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace oneq {
namespace coordinate {

/**
 * @brief 三维旋转矩阵（double 精度）。
 * @note 矩阵按行主序字段命名 m[row][col]；公共头不暴露 Eigen 避免传播编译依赖。
 */
struct ONEQ_API RotationMatrix3d {
  double m00{1.0};  /**< 第 0 行第 0 列 */
  double m01{0.0};  /**< 第 0 行第 1 列 */
  double m02{0.0};  /**< 第 0 行第 2 列 */
  double m10{0.0};  /**< 第 1 行第 0 列 */
  double m11{1.0};  /**< 第 1 行第 1 列 */
  double m12{0.0};  /**< 第 1 行第 2 列 */
  double m20{0.0};  /**< 第 2 行第 0 列 */
  double m21{0.0};  /**< 第 2 行第 1 列 */
  double m22{1.0};  /**< 第 2 行第 2 列 */
};

// =============================================================================
// 输入校验
// =============================================================================

/**
 * @brief 校验欧拉角各分量均为有限值。
 * @param[in] attitude 待校验的欧拉角。
 * @return 所有三轴分量均为有限值返回 true，否则返回 false。
 */
ONEQ_API bool IsFinite(const EulerAnglesDeg& attitude);

/**
 * @brief 校验旋转矩阵各元素均为有限值。
 * @param[in] rotation 待校验的旋转矩阵。
 * @return 全部九个元素均为有限值返回 true，否则返回 false。
 */
ONEQ_API bool IsFinite(const RotationMatrix3d& rotation);

// =============================================================================
// 欧拉角 ↔ 旋转矩阵
// =============================================================================

/**
 * @brief 将 Z-Y-X 欧拉角构造为旋转矩阵。
 * @param[in] attitude_deg 欧拉角（yaw/pitch/roll，单位：deg）。
 * @return 旋转矩阵。
 * @note 旋转顺序 Z-Y-X：R = Rz(yaw) * Ry(pitch) * Rx(roll)。
 *       约定正 pitch 表示正仰角，因此 Ry 内部取 -pitch。
 */
ONEQ_API RotationMatrix3d BuildRotationMatrix(const EulerAnglesDeg& attitude_deg);

/**
 * @brief 从旋转矩阵反解 Z-Y-X 欧拉角。
 * @param[in] rotation 旋转矩阵（需近似满足正交性）。
 * @return Z-Y-X 欧拉角（单位：deg）。
 * @note 当 cos(pitch) ≈ 0（万向节锁）时 yaw 和 roll 的分解存在多解，
 *       此时返回 roll=0 的解以保证输出确定。
 */
ONEQ_API EulerAnglesDeg ToEulerAnglesDeg(const RotationMatrix3d& rotation);

// =============================================================================
// 矩阵运算
// =============================================================================

/**
 * @brief 计算旋转矩阵的逆（等价于转置，因旋转矩阵正交）。
 * @param[in] rotation 旋转矩阵。
 * @return 逆矩阵。
 */
ONEQ_API RotationMatrix3d Inverse(const RotationMatrix3d& rotation);

/**
 * @brief 复合双层旋转矩阵。
 * @param[in] parent_to_child 父坐标系 → 子坐标系 的旋转。
 * @param[in] child_to_grandchild 子坐标系 → 孙坐标系 的旋转。
 * @return 父坐标系 → 孙坐标系 的复合旋转。
 * @details R_parent_to_grandchild = R_parent_to_child * R_child_to_grandchild。
 */
ONEQ_API RotationMatrix3d Compose(const RotationMatrix3d& parent_to_child,
                                  const RotationMatrix3d& child_to_grandchild);

/**
 * @brief 复合双层姿态为单一欧拉角，避免直接欧拉角代数相加。
 * @param[in] parent_to_child 父坐标系 → 子坐标系 的欧拉角（单位：deg）。
 * @param[in] child_to_grandchild 子坐标系 → 孙坐标系 的欧拉角（单位：deg）。
 * @return 父坐标系 → 孙坐标系 的复合欧拉角（单位：deg）。
 * @details 内部链路：欧拉角 → 旋转矩阵 → 矩阵乘法 → 欧拉角。
 *          用于机载雷达场景中 platform_attitude + mount_angles 的正确复合。
 */
ONEQ_API EulerAnglesDeg ComposeAttitudeDeg(const EulerAnglesDeg& parent_to_child,
                                           const EulerAnglesDeg& child_to_grandchild);

// =============================================================================
// ENU ↔ NED 姿态参考系转换
//
// 适用场景：传感器/云台安装角度等任意朝向的旋转矩阵参考系变换。
//
// ⚠ 不适用于飞机姿态转换：飞机机体 z 轴朝下，而 ENU z 轴朝上，
//   标准 Z-Y-X Euler 角分解会对水平飞行产生 roll=180°（"倒飞"）。
//   飞机姿态的正确处理是 heading 直传、pitch 取反、roll 取反。
// =============================================================================

/**
 * @brief 将 Body→NED 姿态转换为 Body→ENU 姿态（旋转矩阵参考系变换）。
 * @param[in] ned_attitude Body→NED 欧拉角（单位：deg）。
 * @return Body→ENU 欧拉角（单位：deg）。
 * @details 通过 R_body→enu = R_ned→enu × R_body→ned 复合实现，
 *          其中 R_ned→enu = [[0,1,0],[1,0,0],[0,0,-1]] 同时交换 x↔y 并取反 z。
 * @warning 不适用于飞机姿态——会产生 roll=180° 和 yaw 90° 偏移。
 */
ONEQ_API EulerAnglesDeg ToEnuAttitude(const EulerAnglesDeg& ned_attitude);

/**
 * @brief 将 Body→ENU 姿态转换为 Body→NED 姿态（旋转矩阵参考系变换）。
 * @param[in] enu_attitude Body→ENU 欧拉角（单位：deg）。
 * @return Body→NED 欧拉角（单位：deg）。
 * @warning 不适用于飞机姿态——会产生 roll=180° 和 yaw 90° 偏移。
 */
ONEQ_API EulerAnglesDeg ToNedAttitude(const EulerAnglesDeg& enu_attitude);

// =============================================================================
// 向量旋转
// =============================================================================

/**
 * @brief 将 ENU 向量旋转到局部坐标系。
 * @param[in] enu_east ENU 东向分量。
 * @param[in] enu_north ENU 北向分量。
 * @param[in] enu_up ENU 天向分量。
 * @param[in] local_attitude_deg 局部坐标系相对 ENU 的姿态角。
 * @return 局部坐标系下的三维向量。
 * @details 数学上等价于 Inverse(BuildRotationMatrix(attitude)) * (e,n,u)^T。
 *          这是 ENU→局部 坐标变换中的姿态旋转步骤。
 */
ONEQ_API Vector3d RotateEnuToLocal(double enu_east, double enu_north, double enu_up,
                                   const EulerAnglesDeg& local_attitude_deg);

/**
 * @brief 将局部坐标系向量旋转到 ENU 坐标系。
 * @param[in] local_x 局部 X 分量。
 * @param[in] local_y 局部 Y 分量。
 * @param[in] local_z 局部 Z 分量。
 * @param[in] local_attitude_deg 局部坐标系相对 ENU 的姿态角。
 * @return ENU 坐标系下的三维向量。
 * @details 为 RotateEnuToLocal 的逆运算，
 *          数学上等价于 BuildRotationMatrix(attitude) * (x,y,z)^T。
 */
ONEQ_API Vector3d RotateLocalToEnu(double local_x, double local_y, double local_z,
                                   const EulerAnglesDeg& local_attitude_deg);

}  // namespace coordinate
}  // namespace oneq

#endif  // ONEQ_COORDINATE_ATTITUDE_TRANSFORM_H_
