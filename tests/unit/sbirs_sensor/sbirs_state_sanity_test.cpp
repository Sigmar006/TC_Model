// 红外滤波状态"可发布"判据的守门用例（2026-10-21 红外日志体检）。
//
// 现场：角度-only EKF 在 4 万公里斜距上距离不可观测，62 行状态到 1e9 m、速度 7.7e5 m/s、
// 协方差 1e19，日志仍按"跟踪"发布，并导致第 9 项测角残差在若干时段出现 ±2.5° 逐拍变号
// （2,962 行超 ±360″ 中 2,820 行落在 NIS=inf 同期）。本用例锁住"不可发布"的判定边界。

#include <limits>

#include <gtest/gtest.h>

#include "sbirs_sensor/pipeline/SbirsStateSanity.h"
#include "sbirs_sensor/tracking/SbirsTrackingTypes.h"

namespace {

using sbirs_sensor::pipeline::IsSaneSbirsState;
using sbirs_sensor::tracking::SbirsStateCovariance;
using sbirs_sensor::tracking::SbirsStateVector;

SbirsStateVector MakeMean(double x, double y, double z, double vx, double vy, double vz) {
  SbirsStateVector mean = SbirsStateVector::Zero();
  mean(0) = static_cast<float>(x);
  mean(1) = static_cast<float>(vx);
  mean(2) = static_cast<float>(y);
  mean(3) = static_cast<float>(vy);
  mean(4) = static_cast<float>(z);
  mean(5) = static_cast<float>(vz);
  return mean;
}

SbirsStateCovariance MakeCovariance(double position_var, double velocity_var) {
  SbirsStateCovariance covariance = SbirsStateCovariance::Zero();
  covariance(0, 0) = static_cast<float>(position_var);
  covariance(2, 2) = static_cast<float>(position_var);
  covariance(4, 4) = static_cast<float>(position_var);
  covariance(1, 1) = static_cast<float>(velocity_var);
  covariance(3, 3) = static_cast<float>(velocity_var);
  covariance(5, 5) = static_cast<float>(velocity_var);
  return covariance;
}

// 地面目标（地心距约 6.4e6 m）与 GEO 目标（约 4.2e7 m）都必须被判为可发布。
TEST(SbirsStateSanityTest, AcceptsGroundAndGeoRangeStates) {
  EXPECT_TRUE(IsSaneSbirsState(MakeMean(-2.36e6, 5.19e6, 2.84e6, 340.0, -100.0, 37.0),
                              MakeCovariance(1.0e6, 1.0e4)));
  EXPECT_TRUE(IsSaneSbirsState(MakeMean(3.36e7, 2.55e7, 1.47e6, -880.0, 2980.0, 0.0),
                              MakeCovariance(1.0e6, 1.0e4)));
}

// 现场发散样例：位置 1.18e9 m（地心距 1.2e9 m）必须被拒。
TEST(SbirsStateSanityTest, RejectsDivergedPositionMagnitude) {
  EXPECT_FALSE(IsSaneSbirsState(MakeMean(1.182e9, 9.8e7, 5.97e7, 7.67e5, -2.3e4, -4.6e5),
                               MakeCovariance(1.0e18, 1.0e12)));
}

// 地心距低于地球半径（目标落进地心）同样不可发布。
TEST(SbirsStateSanityTest, RejectsPositionInsideEarth) {
  EXPECT_FALSE(IsSaneSbirsState(MakeMean(1.0e4, 2.0e4, 3.0e4, 0.0, 0.0, 0.0),
                               MakeCovariance(1.0e6, 1.0e4)));
}

// 协方差爆炸或负方差都不能当成"精度极好"发出去。
TEST(SbirsStateSanityTest, RejectsExplodedOrNegativeCovariance) {
  const SbirsStateVector mean = MakeMean(-2.36e6, 5.19e6, 2.84e6, 340.0, -100.0, 37.0);
  EXPECT_FALSE(IsSaneSbirsState(mean, MakeCovariance(1.0e18, 1.0e4)));
  SbirsStateCovariance negative = MakeCovariance(1.0e6, 1.0e4);
  negative(0, 0) = -1.0f;
  EXPECT_FALSE(IsSaneSbirsState(mean, negative));
}

// 非有限状态（nan/inf）一律不可发布。
TEST(SbirsStateSanityTest, RejectsNonFiniteComponents) {
  const SbirsStateCovariance covariance = MakeCovariance(1.0e6, 1.0e4);
  SbirsStateVector nan_x = MakeMean(-2.36e6, 5.19e6, 2.84e6, 340.0, -100.0, 37.0);
  nan_x(0) = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(IsSaneSbirsState(nan_x, covariance));
  SbirsStateVector inf_velocity = MakeMean(-2.36e6, 5.19e6, 2.84e6, 340.0, -100.0, 37.0);
  inf_velocity(1) = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(IsSaneSbirsState(inf_velocity, covariance));
}

}  // namespace
