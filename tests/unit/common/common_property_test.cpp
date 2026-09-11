/**
 * @file common_property_test.cpp
 * @brief oneq::coordinate 姿态/旋转工具的属性测试（RapidCheck）。
 *
 * 本文件同时是 tests/support/rapidcheck_gtest.h 的接入参考实现：
 * 常量用例（TEST）负责边界点，属性用例（RC_GTEST_PROP）负责在整个输入区间上
 * 成立的不变量，由 RapidCheck 生成输入并在失败时收缩出最小反例。
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "rapidcheck.h"

#include "1q/coordinate/attitude_transform.h"
#include "support/rapidcheck_gtest.h"

namespace oneq {
namespace coordinate {
namespace {

RotationMatrix3d IdentityMatrix() { return RotationMatrix3d{}; }

double MaxAbsDifference(const RotationMatrix3d& lhs, const RotationMatrix3d& rhs) {
  const double difference[9] = {
      lhs.m00 - rhs.m00, lhs.m01 - rhs.m01, lhs.m02 - rhs.m02, lhs.m10 - rhs.m10,
      lhs.m11 - rhs.m11, lhs.m12 - rhs.m12, lhs.m20 - rhs.m20, lhs.m21 - rhs.m21,
      lhs.m22 - rhs.m22};
  double maximum = 0.0;
  for (const double value : difference) {
    maximum = std::max(maximum, std::abs(value));
  }
  return maximum;
}

// 按 360° 周期把角折回 (-360, 360)。属性测试关心的是角度的周期语义，
// 区间外的输入与区间内某个角等价，折回可避免无意义的极端量级。
double WrapDeg(double deg) { return std::fmod(deg, 360.0); }

}  // namespace

// 属性：任意姿态角构造的矩阵与其逆复合，必为单位阵。
// 依据：attitude_transform.h 声明 RotationMatrix3d 为"三维旋转矩阵"，
// 旋转矩阵满足 R·R⁻¹ = I；本属性同时覆盖 BuildRotationMatrix 与 Inverse。
RC_GTEST_PROP(CoordinateAttitudePropertyTest, MatrixComposedWithInverseIsIdentity,
              (double raw_yaw, double raw_pitch, double raw_roll)) {
  const RotationMatrix3d rotation =
      BuildRotationMatrix(EulerAnglesDeg(WrapDeg(raw_yaw), WrapDeg(raw_pitch), WrapDeg(raw_roll)));
  // 容差依据：矩阵元素量级 ≤1，9 次乘加在 double 下累计误差约 1e-15，
  // 取 1e-12 留 3 个数量级余量。
  RC_ASSERT(MaxAbsDifference(Compose(rotation, Inverse(rotation)), IdentityMatrix()) < 1.0e-12);
}

// 属性：仰角在良态区间内时，经矩阵反解回欧拉角后重建的矩阵应与原矩阵一致。
// 依据：ToEulerAnglesDeg 是 BuildRotationMatrix 的反解（attitude_transform.h）。
// 比较矩阵而非角度：yaw 的角表示不唯一（例如 200° 与 -160° 等价），
// 矩阵相等才是该接口真正承诺的内容。
RC_GTEST_PROP(CoordinateAttitudePropertyTest, EulerAnglesRoundTripPreservesMatrix,
              (double raw_yaw, double raw_pitch, double raw_roll)) {
  const double pitch_deg = WrapDeg(raw_pitch);
  // |pitch| → 90° 时 yaw 与 roll 耦合（万向节锁），反解不唯一，属边界点，
  // 由下面的常量用例单独覆盖；本属性只覆盖良态区间。
  RC_PRE(std::abs(pitch_deg) < 89.0);

  const EulerAnglesDeg attitude(WrapDeg(raw_yaw), pitch_deg, WrapDeg(raw_roll));
  const RotationMatrix3d rebuilt = BuildRotationMatrix(ToEulerAnglesDeg(BuildRotationMatrix(attitude)));
  // 容差依据：|pitch| < 89° ⇒ cos(pitch) > 1.7e-2，反解中 atan2 的误差
  // 约 1e-16 / cos²(pitch) ≈ 3e-13 rad ≈ 2e-11 deg，取 1e-9 留约 50 倍余量。
  RC_ASSERT(MaxAbsDifference(rebuilt, BuildRotationMatrix(attitude)) < 1.0e-9);
}

// 属性：矩阵复合满足结合律。
// 依据：Compose 为矩阵乘法（attitude_transform.h），矩阵乘法天然结合。
RC_GTEST_PROP(CoordinateAttitudePropertyTest, ComposeIsAssociative,
              (double yaw_a, double pitch_a, double roll_a, double yaw_b, double pitch_b,
               double roll_b, double yaw_c, double pitch_c, double roll_c)) {
  const RotationMatrix3d a =
      BuildRotationMatrix(EulerAnglesDeg(WrapDeg(yaw_a), WrapDeg(pitch_a), WrapDeg(roll_a)));
  const RotationMatrix3d b =
      BuildRotationMatrix(EulerAnglesDeg(WrapDeg(yaw_b), WrapDeg(pitch_b), WrapDeg(roll_b)));
  const RotationMatrix3d c =
      BuildRotationMatrix(EulerAnglesDeg(WrapDeg(yaw_c), WrapDeg(pitch_c), WrapDeg(roll_c)));

  // 容差依据：两次复合共 36 次乘加，量级 ≤1，累计误差约 1e-14，取 1e-12。
  RC_ASSERT(MaxAbsDifference(Compose(Compose(a, b), c), Compose(a, Compose(b, c))) < 1.0e-12);
}

// 属性：局部 → ENU → 局部 的旋转往返应还原原向量。
// 依据：RotateEnuToLocal 声明"数学上等价于 Inverse(BuildRotationMatrix(attitude))·v"，
// 与 RotateLocalToEnu 互为逆变换（attitude_transform.h）。
RC_GTEST_PROP(CoordinateAttitudePropertyTest, LocalEnuRotationRoundTrip,
              (double raw_x, double raw_y, double raw_z, double raw_yaw, double raw_pitch,
               double raw_roll)) {
  const double scale = 1000.0;
  const double x = std::fmod(raw_x, scale);
  const double y = std::fmod(raw_y, scale);
  const double z = std::fmod(raw_z, scale);
  const EulerAnglesDeg attitude(WrapDeg(raw_yaw), WrapDeg(raw_pitch), WrapDeg(raw_roll));

  const Vector3d enu = RotateLocalToEnu(x, y, z, attitude);
  const Vector3d local = RotateEnuToLocal(enu.x, enu.y, enu.z, attitude);

  // 容差依据：向量分量 |v| < 1e3，两次 3×3 旋转共 12 次乘加，
  // 相对误差 ~1e-16 ⇒ 绝对误差 ~1e-13，取 1e-9 留 4 个数量级余量。
  EXPECT_NEAR(local.x, x, 1.0e-9);
  EXPECT_NEAR(local.y, y, 1.0e-9);
  EXPECT_NEAR(local.z, z, 1.0e-9);
}

// 边界用例：万向节锁（|pitch| = 90°）处正交性仍必须成立。
// 依据：无论反解分支如何取，BuildRotationMatrix 的产物都是旋转矩阵。
TEST(CoordinateAttitudeBoundaryTest, MatrixStaysOrthonormalAtGimbalLock) {
  const double kPitches[] = {90.0, -90.0};
  for (const double pitch_deg : kPitches) {
    SCOPED_TRACE(::testing::Message() << "pitch_deg=" << pitch_deg);
    const RotationMatrix3d rotation =
        BuildRotationMatrix(EulerAnglesDeg(37.0, pitch_deg, -14.0));
    // 容差依据同 MatrixComposedWithInverseIsIdentity。
    EXPECT_LT(MaxAbsDifference(Compose(rotation, Inverse(rotation)), IdentityMatrix()), 1.0e-12);
  }
}

// 边界用例：非有限输入必须被 IsFinite 显式识别。
// 依据：attitude_transform.h 提供 IsFinite 作为有效性判定出口；
// 调用方据此拒绝输入，而不是让 NaN 静默流入几何链路。
TEST(CoordinateAttitudeBoundaryTest, NonFiniteInputIsReportedByIsFinite) {
  const double kNonFinite[] = {std::nan(""), std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()};
  for (const double value : kNonFinite) {
    SCOPED_TRACE(::testing::Message() << "value=" << value);
    const EulerAnglesDeg attitude(value, 0.0, 0.0);
    EXPECT_FALSE(IsFinite(attitude));
    EXPECT_FALSE(IsFinite(BuildRotationMatrix(attitude)));
  }
}

}  // namespace coordinate
}  // namespace oneq
