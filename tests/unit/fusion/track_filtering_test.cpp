/**
 * @file track_filtering_test.cpp
 * @brief 验证 fusion 逐航迹无迹滤波与航迹管理（P2，docs/fusion/algorithms.md §4）。
 *
 * 覆盖：默认关闭零回退、位置通道滤波收敛与确认门、单原点角度-only 弱可观测
 * （横向收敛/径向滞后的协方差语义）、双原点（身份键直挂）三角定位收敛、
 * coasting/删除生命周期、Reset 后确定性。
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/fusion/DetectionRecord.h"
#include "1q/fusion/FusedTarget.h"
#include "1q/fusion/FusionConfig.h"
#include "1q/fusion/FusionEngine.h"

namespace fusion {
namespace {

using oneq::coordinate::EcefPositionM;
using oneq::coordinate::LlaPositionDegM;

constexpr double kEarthRadiusM = 6378137.0;
constexpr std::uint32_t kTestSourceId = 9U;

FusionConfig MakeFilteringConfig() {
  FusionConfig config;
  config.enable_track_filtering = true;
  config.track_process_noise = 1.0;
  config.track_initial_position_std_m = 2000.0;
  config.track_initial_velocity_std_m_per_s = 300.0;
  config.track_bearing_init_range_m = 250000.0;
  config.track_bearing_init_range_std_m = 300000.0;
  return config;
}

DetectionRecord MakePositionRecord(std::uint64_t key, const LlaPositionDegM& position) {
  DetectionRecord record;
  record.key = key;
  record.source_id = kTestSourceId;
  record.has_position = true;
  record.position = position;
  record.verdict = 1.0;
  record.quality = 1.0;
  return record;
}

EcefPositionM NoisyEcef(const EcefPositionM& truth, double sigma_m, std::mt19937& engine) {
  std::normal_distribution<double> noise(0.0, 1.0);
  EcefPositionM measured(truth.x_m + noise(engine) * sigma_m, truth.y_m + noise(engine) * sigma_m,
                         truth.z_m + noise(engine) * sigma_m);
  return measured;
}

LlaPositionDegM EcefToLla(const EcefPositionM& ecef) {
  LlaPositionDegM lla{};
  if (!oneq::coordinate::TryEcefToLla(ecef, &lla)) {
    lla = LlaPositionDegM(0.0, 0.0, 0.0);
  }
  return lla;
}

// ENU 基（与引擎内部契约一致：az 自北向东、el 出地平）。
struct TestEnuBasis {
  double east[3];
  double north[3];
  double up[3];
};

TestEnuBasis MakeBasis(const EcefPositionM& origin) {
  const double norm = std::sqrt(origin.x_m * origin.x_m + origin.y_m * origin.y_m +
                                origin.z_m * origin.z_m);
  TestEnuBasis basis;
  basis.up[0] = origin.x_m / norm;
  basis.up[1] = origin.y_m / norm;
  basis.up[2] = origin.z_m / norm;
  const double e0 = -basis.up[1];
  const double e1 = basis.up[0];
  const double en = std::sqrt(e0 * e0 + e1 * e1);
  basis.east[0] = e0 / en;
  basis.east[1] = e1 / en;
  basis.east[2] = 0.0;
  basis.north[0] = basis.up[1] * basis.east[2] - basis.up[2] * basis.east[1];
  basis.north[1] = basis.up[2] * basis.east[0] - basis.up[0] * basis.east[2];
  basis.north[2] = basis.up[0] * basis.east[1] - basis.up[1] * basis.east[0];
  return basis;
}

struct EcefPositionDegM {
  double az_deg;
  double el_deg;
};

DetectionRecord MakeBearingRecord(std::uint64_t key, const EcefPositionM& origin,
                                  const EcefPositionDegM& bearing, double sigma_rad,
                                  const LlaPositionDegM& origin_lla) {
  DetectionRecord record;
  record.key = key;
  record.source_id = kTestSourceId;
  record.has_bearing = true;
  record.bearing_az_deg = bearing.az_deg;
  record.bearing_el_deg = bearing.el_deg;
  record.has_sensor_origin = true;
  record.sensor_origin = origin_lla;
  record.has_bearing_noise = true;
  record.bearing_noise_sigma_rad = sigma_rad;
  record.verdict = 1.0;
  record.quality = 1.0;
  return record;
}

EcefPositionDegM BearingTo(const EcefPositionM& origin, const EcefPositionM& target,
                           const TestEnuBasis& basis) {
  const double d[3] = {target.x_m - origin.x_m, target.y_m - origin.y_m,
                       target.z_m - origin.z_m};
  const auto dot = [](const double* a, const double* b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  };
  const double e = dot(d, basis.east);
  const double n = dot(d, basis.north);
  const double u = dot(d, basis.up);
  const double r = std::sqrt(dot(d, d));
  EcefPositionDegM bearing;
  bearing.az_deg = std::atan2(e, n) * 57.29577951308232;
  bearing.el_deg = std::asin(std::max(-1.0, std::min(1.0, u / r))) * 57.29577951308232;
  return bearing;
}

EcefPositionDegM NoisyBearing(const EcefPositionDegM& truth, double sigma_rad,
                              std::mt19937& engine) {
  std::normal_distribution<double> noise(0.0, 1.0);
  EcefPositionDegM measured;
  measured.az_deg = truth.az_deg + noise(engine) * sigma_rad * 57.29577951308232;
  measured.el_deg = truth.el_deg + noise(engine) * sigma_rad * 57.29577951308232;
  return measured;
}

double DistanceM(const EcefPositionM& a, const EcefPositionM& b) {
  const double dx = a.x_m - b.x_m;
  const double dy = a.y_m - b.y_m;
  const double dz = a.z_m - b.z_m;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

EcefPositionM EstimatePositionEcef(const FusedTarget& target) {
  EcefPositionM ecef{};
  oneq::coordinate::TryLlaToEcef(target.kinematic_estimate.position, &ecef);
  return ecef;
}

const FusedTarget* FindByKey(const std::vector<FusedTarget>& targets, std::uint64_t key) {
  for (const auto& target : targets) {
    if (target.key == key) {
      return &target;
    }
  }
  return nullptr;
}

TEST(FusionTrackFilteringTest, FilteringEnabledByDefaultOutputsKinematics) {
  FusionEngine engine(FusionConfig{});
  std::mt19937 engine_random(7U);
  EcefPositionM truth(kEarthRadiusM, 0.0, 0.0);
  for (std::uint64_t cycle = 1U; cycle <= 5U; ++cycle) {
    const auto targets =
        engine.Update({MakePositionRecord(7U, EcefToLla(NoisyEcef(truth, 50.0, engine_random)))},
                      cycle);
    ASSERT_EQ(targets.size(), 1U);
    EXPECT_TRUE(targets.front().has_kinematic_estimate);
  }
}

TEST(FusionTrackFilteringTest, FilteringExplicitlyDisabledOutputsNoKinematics) {
  FusionConfig config;
  config.enable_track_filtering = false;
  FusionEngine engine(config);
  std::mt19937 engine_random(7U);
  EcefPositionM truth(kEarthRadiusM, 0.0, 0.0);
  for (std::uint64_t cycle = 1U; cycle <= 5U; ++cycle) {
    const auto targets =
        engine.Update({MakePositionRecord(7U, EcefToLla(NoisyEcef(truth, 50.0, engine_random)))},
                      cycle);
    ASSERT_EQ(targets.size(), 1U);
    EXPECT_FALSE(targets.front().has_kinematic_estimate);
  }
}

TEST(FusionTrackFilteringTest, PositionChannelTrackFiltersAndConfirms) {
  FusionEngine engine(MakeFilteringConfig());
  std::mt19937 engine_random(11U);
  const EcefPositionM p0(kEarthRadiusM, 0.0, 0.0);
  const EcefPositionM v(0.0, 200.0, 0.0);
  std::vector<double> errors_m;
  double final_velocity_error_m_per_s = -1.0;
  for (std::uint64_t cycle = 1U; cycle <= 40U; ++cycle) {
    const double t = static_cast<double>(cycle - 1U);
    const EcefPositionM truth(p0.x_m + v.x_m * t, p0.y_m + v.y_m * t, p0.z_m + v.z_m * t);
    const auto targets = engine.Update(
        {MakePositionRecord(7U, EcefToLla(NoisyEcef(truth, 50.0, engine_random)))}, cycle);
    ASSERT_EQ(targets.size(), 1U);
    const FusedTarget& target = targets.front();
    ASSERT_TRUE(target.has_kinematic_estimate);
    errors_m.push_back(DistanceM(EstimatePositionEcef(target), truth));
    if (cycle == 1U) {
      EXPECT_EQ(target.lifecycle, FusedTrackLifecycle::kTentative);
    }
    if (cycle == 3U) {
      EXPECT_EQ(target.lifecycle, FusedTrackLifecycle::kConfirmed);
    }
    if (cycle == 40U) {
      const auto& vel = target.kinematic_estimate.velocity_ecef_m_per_s;
      final_velocity_error_m_per_s = std::sqrt((vel[0] - v.x_m) * (vel[0] - v.x_m) +
                                               (vel[1] - v.y_m) * (vel[1] - v.y_m) +
                                               (vel[2] - v.z_m) * (vel[2] - v.z_m));
      for (double cov : target.kinematic_estimate.covariance_ecef) {
        EXPECT_TRUE(std::isfinite(cov));
      }
    }
  }
  // 收敛门：末 5 周期平均误差低于单次量测 1-σ（滤波平滑收益）。
  double tail_mean_m = 0.0;
  for (std::size_t i = errors_m.size() - 5U; i < errors_m.size(); ++i) {
    tail_mean_m += errors_m[i];
  }
  tail_mean_m /= 5.0;
  EXPECT_LT(tail_mean_m, 50.0) << "filtered tail error should beat raw measurement sigma";
  EXPECT_LT(errors_m.back(), 100.0);
  EXPECT_LT(final_velocity_error_m_per_s, 50.0);
}

// 合成量测隔离（2026-10-26）：单星保活推导的位置只用于保活，不得进滤波，
// 也不得让航迹被精度评估当成"有真实量测"。
TEST(FusionTrackFilteringTest, SyntheticMeasurementsKeepTrackAliveWithoutFilterUpdate) {
  FusionEngine engine(MakeFilteringConfig());
  const EcefPositionM p0(kEarthRadiusM, 0.0, 0.0);
  const EcefPositionM v(0.0, 200.0, 0.0);
  const EcefPositionM truth_third(p0.x_m + v.x_m * 2.0, p0.y_m + v.y_m * 2.0, 0.0);

  // 1) 两拍真实位置量测：滤波起始 + 两点差分初速。
  for (std::uint64_t cycle = 1U; cycle <= 2U; ++cycle) {
    const double t = static_cast<double>(cycle - 1U);
    const EcefPositionM truth(p0.x_m + v.x_m * t, p0.y_m + v.y_m * t, p0.z_m);
    const auto targets = engine.Update({MakePositionRecord(7U, EcefToLla(truth))}, cycle);
    ASSERT_EQ(targets.size(), 1U);
    ASSERT_TRUE(targets.front().has_kinematic_estimate);
    EXPECT_TRUE(targets.front().updated_by_real_measurement);
  }
  const std::vector<FusedTarget> before = engine.Update({}, 3U);
  const EcefPositionM mean_before = EstimatePositionEcef(before.front());

  // 2) 合成量测：位置明显偏 30 km。若它进了滤波，均值会被拉走。
  DetectionRecord synthetic = MakePositionRecord(
      7U, EcefToLla(EcefPositionM(truth_third.x_m + 30000.0, truth_third.y_m, truth_third.z_m)));
  synthetic.synthetic = true;
  const std::vector<FusedTarget> after_synthetic = engine.Update({synthetic}, 4U);
  ASSERT_EQ(after_synthetic.size(), 1U);
  const FusedTarget& target = after_synthetic.front();
  // 命中数照常累计（保活语义），但运动学均值不被合成位置拖动。
  EXPECT_GE(target.last_update_cycle, 4U);
  const EcefPositionM mean_after = EstimatePositionEcef(target);
  EXPECT_LT(DistanceM(mean_after, mean_before), 20000.0)
      << "synthetic measurement must not drag the filter mean";
  // 真实量测记账：仍标记为有过真实更新（前两拍），但周期号停在 2。
  EXPECT_TRUE(target.updated_by_real_measurement);
  EXPECT_EQ(target.last_real_measurement_cycle, 2U);
}

// 纯合成航迹：从未吃过真实量测的航迹必须显式标记，供评估侧剔除。
TEST(FusionTrackFilteringTest, SyntheticOnlyTrackIsNotMarkedAsRealMeasurement) {
  FusionEngine engine(MakeFilteringConfig());
  const EcefPositionM p0(kEarthRadiusM, 0.0, 0.0);
  DetectionRecord synthetic = MakePositionRecord(11U, EcefToLla(p0));
  synthetic.synthetic = true;
  const std::vector<FusedTarget> targets = engine.Update({synthetic}, 1U);
  ASSERT_EQ(targets.size(), 1U);
  EXPECT_FALSE(targets.front().updated_by_real_measurement);
  EXPECT_EQ(targets.front().last_real_measurement_cycle, 0U);
  // 合成量测不驱动滤波起始：无运动学估计。
  EXPECT_FALSE(targets.front().has_kinematic_estimate);
}

// 单原点角度-only：横向（切向）快速收敛、径向（距离）滞后——弱可观测产品语义。
TEST(FusionTrackFilteringTest, BearingOnlyTrackKeepsWeakRangeObservability) {
  FusionEngine engine(MakeFilteringConfig());
  std::mt19937 engine_random(23U);
  const EcefPositionM origin(kEarthRadiusM + 20000.0, 0.0, 0.0);
  const LlaPositionDegM origin_lla = EcefToLla(origin);
  const TestEnuBasis basis = MakeBasis(origin);
  const EcefPositionM p0(kEarthRadiusM, 300000.0, 100000.0);
  const EcefPositionM v(-500.0, 1000.0, 0.0);
  double radial_m = -1.0;
  double lateral_m = -1.0;
  for (std::uint64_t cycle = 1U; cycle <= 15U; ++cycle) {
    const double t = static_cast<double>(cycle - 1U);
    const EcefPositionM truth(p0.x_m + v.x_m * t, p0.y_m + v.y_m * t, p0.z_m + v.z_m * t);
    const EcefPositionDegM bearing = NoisyBearing(BearingTo(origin, truth, basis), 2.0e-4,
                                                   engine_random);
    const auto targets = engine.Update({MakeBearingRecord(9U, origin, bearing, 2.0e-4, origin_lla)},
                                       cycle);
    ASSERT_EQ(targets.size(), 1U);
    ASSERT_TRUE(targets.front().has_kinematic_estimate);
    if (cycle == 15U) {
      const EcefPositionM estimate = EstimatePositionEcef(targets.front());
      double los[3] = {truth.x_m - origin.x_m, truth.y_m - origin.y_m, truth.z_m - origin.z_m};
      const double los_norm = std::sqrt(los[0] * los[0] + los[1] * los[1] + los[2] * los[2]);
      for (int i = 0; i < 3; ++i) {
        los[i] /= los_norm;
      }
      double err[3] = {estimate.x_m - truth.x_m, estimate.y_m - truth.y_m,
                       estimate.z_m - truth.z_m};
      radial_m = std::abs(err[0] * los[0] + err[1] * los[1] + err[2] * los[2]);
      const double radial_vec[3] = {radial_m * los[0], radial_m * los[1], radial_m * los[2]};
      lateral_m = std::sqrt((err[0] - radial_vec[0]) * (err[0] - radial_vec[0]) +
                            (err[1] - radial_vec[1]) * (err[1] - radial_vec[1]) +
                            (err[2] - radial_vec[2]) * (err[2] - radial_vec[2]));
    }
  }
  EXPECT_LT(lateral_m, 2000.0) << "cross-range should converge fast";
  EXPECT_LT(lateral_m, radial_m) << "range must stay weaker than cross-range (weak observability)";
  EXPECT_GT(radial_m, 3000.0) << "range should not be resolved by single origin";
}

// 双原点（身份键直挂，OQ-4 方案 a 形态）：距离可观测。
TEST(FusionTrackFilteringTest, TwoOriginsFixRange) {
  auto run = [](bool two_origins, double* radial_out) {
    FusionEngine engine(MakeFilteringConfig());
    std::mt19937 engine_random(31U);
    const EcefPositionM origin_a(kEarthRadiusM + 20000.0, 0.0, 0.0);
    EcefPositionM origin_b{};
    if (!oneq::coordinate::TryLlaToEcef(LlaPositionDegM(10.0, 10.0, 20000.0), &origin_b)) {
      ADD_FAILURE() << "origin B conversion failed";
      return;
    }
    const LlaPositionDegM origin_a_lla = EcefToLla(origin_a);
    const LlaPositionDegM origin_b_lla = EcefToLla(origin_b);
    const TestEnuBasis basis_a = MakeBasis(origin_a);
    const TestEnuBasis basis_b = MakeBasis(origin_b);
    const EcefPositionM p0(kEarthRadiusM, 300000.0, 100000.0);
    const EcefPositionM v(-500.0, 1000.0, 0.0);
    for (std::uint64_t cycle = 1U; cycle <= 15U; ++cycle) {
      const double t = static_cast<double>(cycle - 1U);
      const EcefPositionM truth(p0.x_m + v.x_m * t, p0.y_m + v.y_m * t, p0.z_m + v.z_m * t);
      std::vector<DetectionRecord> records;
      records.push_back(MakeBearingRecord(
          9U, origin_a, NoisyBearing(BearingTo(origin_a, truth, basis_a), 2.0e-4, engine_random),
          2.0e-4, origin_a_lla));
      if (two_origins) {
        records.push_back(MakeBearingRecord(
            9U, origin_b,
            NoisyBearing(BearingTo(origin_b, truth, basis_b), 2.0e-4, engine_random), 2.0e-4,
            origin_b_lla));
      }
      const auto targets = engine.Update(records, cycle);
      ASSERT_EQ(targets.size(), 1U);
      ASSERT_TRUE(targets.front().has_kinematic_estimate);
      if (cycle == 15U) {
        const EcefPositionM estimate = EstimatePositionEcef(targets.front());
        double los[3] = {truth.x_m - origin_a.x_m, truth.y_m - origin_a.y_m,
                         truth.z_m - origin_a.z_m};
        const double los_norm = std::sqrt(los[0] * los[0] + los[1] * los[1] + los[2] * los[2]);
        for (int i = 0; i < 3; ++i) {
          los[i] /= los_norm;
        }
        *radial_out = std::abs((estimate.x_m - truth.x_m) * los[0] +
                               (estimate.y_m - truth.y_m) * los[1] +
                               (estimate.z_m - truth.z_m) * los[2]);
      }
    }
  };
  double radial_single = -1.0;
  double radial_two = -1.0;
  run(false, &radial_single);
  run(true, &radial_two);
  EXPECT_GT(radial_single, 3000.0);
  EXPECT_LT(radial_two, 3000.0) << "two origins must resolve range";
  EXPECT_LT(radial_two, radial_single) << "second origin must improve range";
}

TEST(FusionTrackFilteringTest, LifecycleCoastingAndDeletion) {
  FusionEngine engine(MakeFilteringConfig());
  std::mt19937 engine_random(5U);
  const EcefPositionM truth(kEarthRadiusM, 0.0, 0.0);
  std::uint64_t cycle = 1U;
  for (; cycle <= 5U; ++cycle) {
    const auto targets =
        engine.Update({MakePositionRecord(3U, EcefToLla(NoisyEcef(truth, 50.0, engine_random)))},
                      cycle);
    ASSERT_EQ(targets.size(), 1U);
    EXPECT_EQ(targets.front().lifecycle, cycle >= 3U ? FusedTrackLifecycle::kConfirmed
                                                     : FusedTrackLifecycle::kTentative);
  }
  // 失跟 2 周期：仍在输出，coasting，last_update_cycle 冻结。
  for (std::uint64_t missed = 1U; missed <= 2U; ++missed, ++cycle) {
    const auto targets = engine.Update({}, cycle);
    ASSERT_EQ(targets.size(), 1U);
    EXPECT_EQ(targets.front().lifecycle, FusedTrackLifecycle::kCoasting);
    EXPECT_EQ(targets.front().last_update_cycle, 5U);
    EXPECT_TRUE(targets.front().has_kinematic_estimate);
  }
  // 超过 max_missed_cycles（默认 5）：删除。
  for (std::uint64_t missed = 3U; missed <= 6U; ++missed, ++cycle) {
    const auto targets = engine.Update({}, cycle);
    if (missed > 5U) {
      EXPECT_TRUE(targets.empty());
    }
  }
}

TEST(FusionTrackFilteringTest, DeterministicAfterReset) {
  auto run = [](std::vector<FusedTarget>* final_targets) {
    FusionEngine engine(MakeFilteringConfig());
    std::mt19937 engine_random(99U);
    const EcefPositionM p0(kEarthRadiusM, 0.0, 0.0);
    const EcefPositionM v(0.0, 200.0, 0.0);
    for (std::uint64_t cycle = 1U; cycle <= 10U; ++cycle) {
      const double t = static_cast<double>(cycle - 1U);
      const EcefPositionM truth(p0.x_m, v.y_m * t, 0.0);
      *final_targets = engine.Update(
          {MakePositionRecord(7U, EcefToLla(NoisyEcef(truth, 50.0, engine_random)))}, cycle);
    }
  };
  std::vector<FusedTarget> first;
  std::vector<FusedTarget> second;
  run(&first);
  run(&second);
  ASSERT_EQ(first.size(), second.size());
  ASSERT_EQ(first.size(), 1U);
  EXPECT_EQ(first.front().key, second.front().key);
  EXPECT_DOUBLE_EQ(first.front().kinematic_estimate.position.latitude_deg,
                   second.front().kinematic_estimate.position.latitude_deg);
  EXPECT_DOUBLE_EQ(first.front().kinematic_estimate.position.longitude_deg,
                   second.front().kinematic_estimate.position.longitude_deg);
  EXPECT_DOUBLE_EQ(first.front().kinematic_estimate.position.altitude_m,
                   second.front().kinematic_estimate.position.altitude_m);
  for (std::size_t i = 0U; i < 3U; ++i) {
    EXPECT_DOUBLE_EQ(first.front().kinematic_estimate.velocity_ecef_m_per_s[i],
                     second.front().kinematic_estimate.velocity_ecef_m_per_s[i]);
  }
  for (std::size_t i = 0U; i < 36U; ++i) {
    EXPECT_DOUBLE_EQ(first.front().kinematic_estimate.covariance_ecef[i],
                     second.front().kinematic_estimate.covariance_ecef[i]);
  }
}

TEST(FusionTrackFilteringTest, SubCycleTimestampPropagatesMovingTrackBetweenAsyncDetections) {
  FusionConfig config = MakeFilteringConfig();
  config.track_process_noise = 0.1;
  config.default_position_noise_std_m = 1.0;

  const EcefPositionM p0(kEarthRadiusM, 0.0, 0.0);
  const EcefPositionM v(0.0, 500.0, 0.0);

  auto make_record = [&](std::uint64_t key, double t_sec, std::uint32_t src) {
    const EcefPositionM truth(p0.x_m, p0.y_m + v.y_m * t_sec, p0.z_m);
    DetectionRecord record = MakePositionRecord(key, EcefToLla(truth));
    record.source_id = src;
    record.has_timestamp = true;
    record.timestamp_s = t_sec;
    return record;
  };

  // 1) 正常次序送入：同一周期包含两个不同时刻量测 (t=1.2s, t=1.7s)
  FusionEngine engine_in_order(config);
  auto res1 = engine_in_order.Update({make_record(1U, 1.2, 10U), make_record(1U, 1.7, 20U)}, 1U);
  ASSERT_EQ(res1.size(), 1U);
  ASSERT_TRUE(res1.front().has_kinematic_estimate);

  // 周期 2：送入 t=2.3s, t=2.8s
  auto res2 = engine_in_order.Update({make_record(1U, 2.3, 10U), make_record(1U, 2.8, 20U)}, 2U);
  ASSERT_EQ(res2.size(), 1U);
  ASSERT_TRUE(res2.front().has_kinematic_estimate);

  const EcefPositionM truth_final(p0.x_m, p0.y_m + v.y_m * 2.8, p0.z_m);
  const EcefPositionM est_pos = EstimatePositionEcef(res2.front());
  const double pos_err = DistanceM(est_pos, truth_final);
  EXPECT_LT(pos_err, 5.0) << "Position estimate should track sub-cycle trajectory accurately";

  const auto& vel = res2.front().kinematic_estimate.velocity_ecef_m_per_s;
  EXPECT_NEAR(vel[0], 0.0, 20.0);
  EXPECT_NEAR(vel[1], 500.0, 50.0) << "Velocity should converge toward 500 m/s with sub-cycle dt";
  EXPECT_NEAR(vel[2], 0.0, 20.0);

  // 2) 逆序送入验证 stable_sort：在同周期内若乱序输入 (t=1.7s 在前, t=1.2s 在后)，
  //    引擎内部升序排序后输出应与正常次序完全一致。
  FusionEngine engine_shuffled(config);
  auto res1_shuf = engine_shuffled.Update({make_record(1U, 1.7, 20U), make_record(1U, 1.2, 10U)}, 1U);
  auto res2_shuf = engine_shuffled.Update({make_record(1U, 2.8, 20U), make_record(1U, 2.3, 10U)}, 2U);
  ASSERT_EQ(res2_shuf.size(), 1U);
  const EcefPositionM est_pos_shuf = EstimatePositionEcef(res2_shuf.front());
  EXPECT_DOUBLE_EQ(est_pos.x_m, est_pos_shuf.x_m);
  EXPECT_DOUBLE_EQ(est_pos.y_m, est_pos_shuf.y_m);
  EXPECT_DOUBLE_EQ(est_pos.z_m, est_pos_shuf.z_m);
}

TEST(FusionTrackFilteringTest, SubCycleBearingDualSatResolvesRange) {
  FusionEngine engine(MakeFilteringConfig());
  std::mt19937 engine_random(42U);
  const EcefPositionM origin_a(kEarthRadiusM + 20000.0, 0.0, 0.0);
  EcefPositionM origin_b{};
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(LlaPositionDegM(10.0, 10.0, 20000.0), &origin_b));

  const LlaPositionDegM origin_a_lla = EcefToLla(origin_a);
  const LlaPositionDegM origin_b_lla = EcefToLla(origin_b);
  const TestEnuBasis basis_a = MakeBasis(origin_a);
  const TestEnuBasis basis_b = MakeBasis(origin_b);
  const EcefPositionM p0(kEarthRadiusM, 300000.0, 100000.0);
  const EcefPositionM v(-500.0, 1000.0, 0.0);

  double radial_err_m = -1.0;
  for (std::uint64_t cycle = 1U; cycle <= 15U; ++cycle) {
    const double base_t = static_cast<double>(cycle - 1U);
    const double t_a = base_t + 0.2;
    const double t_b = base_t + 0.7;
    const EcefPositionM truth_a(p0.x_m + v.x_m * t_a, p0.y_m + v.y_m * t_a, p0.z_m + v.z_m * t_a);
    const EcefPositionM truth_b(p0.x_m + v.x_m * t_b, p0.y_m + v.y_m * t_b, p0.z_m + v.z_m * t_b);

    DetectionRecord rec_a = MakeBearingRecord(
        9U, origin_a, NoisyBearing(BearingTo(origin_a, truth_a, basis_a), 2.0e-4, engine_random),
        2.0e-4, origin_a_lla);
    rec_a.has_timestamp = true;
    rec_a.timestamp_s = t_a;

    DetectionRecord rec_b = MakeBearingRecord(
        9U, origin_b, NoisyBearing(BearingTo(origin_b, truth_b, basis_b), 2.0e-4, engine_random),
        2.0e-4, origin_b_lla);
    rec_b.has_timestamp = true;
    rec_b.timestamp_s = t_b;

    const auto targets = engine.Update({rec_a, rec_b}, cycle);
    ASSERT_EQ(targets.size(), 1U);
    ASSERT_TRUE(targets.front().has_kinematic_estimate);

    if (cycle == 15U) {
      const EcefPositionM estimate = EstimatePositionEcef(targets.front());
      double los[3] = {truth_b.x_m - origin_a.x_m, truth_b.y_m - origin_a.y_m,
                       truth_b.z_m - origin_a.z_m};
      const double los_norm = std::sqrt(los[0] * los[0] + los[1] * los[1] + los[2] * los[2]);
      for (int i = 0; i < 3; ++i) {
        los[i] /= los_norm;
      }
      radial_err_m = std::abs((estimate.x_m - truth_b.x_m) * los[0] +
                              (estimate.y_m - truth_b.y_m) * los[1] +
                              (estimate.z_m - truth_b.z_m) * los[2]);
    }
  }
  EXPECT_LT(radial_err_m, 3000.0)
      << "Asynchronous bearing observations with sub-cycle propagation must resolve range";
}

TEST(FusionTrackFilteringTest, TwoPointDifferencingInitializesVelocityAccurately) {
  FusionConfig config = MakeFilteringConfig();
  config.track_process_noise = 0.5;
  config.default_position_noise_std_m = 10.0;

  const EcefPositionM p0(kEarthRadiusM, 0.0, 0.0);
  const EcefPositionM v(300.0, -1800.0, 80.0);

  auto make_record = [&](std::uint64_t key, double t_sec, std::uint32_t src) {
    const EcefPositionM truth(p0.x_m + v.x_m * t_sec, p0.y_m + v.y_m * t_sec,
                              p0.z_m + v.z_m * t_sec);
    DetectionRecord record = MakePositionRecord(key, EcefToLla(truth));
    record.source_id = src;
    record.has_timestamp = true;
    record.timestamp_s = t_sec;
    return record;
  };

  FusionEngine engine(config);

  // 周期 1：首个位置量测到达（t=1.0s），航迹建立，初速为 0
  auto res1 = engine.Update({make_record(10U, 1.0, 1U)}, 1U);
  ASSERT_EQ(res1.size(), 1U);
  ASSERT_TRUE(res1.front().has_kinematic_estimate);
  EXPECT_DOUBLE_EQ(res1.front().kinematic_estimate.velocity_ecef_m_per_s[0], 0.0);
  EXPECT_DOUBLE_EQ(res1.front().kinematic_estimate.velocity_ecef_m_per_s[1], 0.0);
  EXPECT_DOUBLE_EQ(res1.front().kinematic_estimate.velocity_ecef_m_per_s[2], 0.0);

  // 周期 2：第二个位置量测到达（t=2.0s），触发两点差分初速初始化
  auto res2 = engine.Update({make_record(10U, 2.0, 2U)}, 2U);
  ASSERT_EQ(res2.size(), 1U);
  ASSERT_TRUE(res2.front().has_kinematic_estimate);
  const auto& vel2 = res2.front().kinematic_estimate.velocity_ecef_m_per_s;
  EXPECT_NEAR(vel2[0], v.x_m, 1.0);
  EXPECT_NEAR(vel2[1], v.y_m, 1.0);
  EXPECT_NEAR(vel2[2], v.z_m, 1.0);

  // 周期 3：第三个量测（t=3.0s），进入常规 UKF 滤波更新，无超调震荡
  auto res3 = engine.Update({make_record(10U, 3.0, 1U)}, 3U);
  ASSERT_EQ(res3.size(), 1U);
  ASSERT_TRUE(res3.front().has_kinematic_estimate);
  const auto& vel3 = res3.front().kinematic_estimate.velocity_ecef_m_per_s;
  EXPECT_NEAR(vel3[0], v.x_m, 2.0);
  EXPECT_NEAR(vel3[1], v.y_m, 2.0);
  EXPECT_NEAR(vel3[2], v.z_m, 2.0);
}

// ---------------------------------------------------------------------------
// 航迹初始化路径（algorithms.md §4「起始」与「两点差分初速（位置起始航迹，
// 2026-09-07）」；design.md「追认：滤波引擎内部亚周期对齐与两点差分初速」）。
// 锁定首拍量测在两种起始形态下的行为：位置起始只建航迹、不做同帧重复更新；
// 方位起始保留首拍量测更新（角度-only 距离收敛依赖它）。
// ---------------------------------------------------------------------------

// 位置状态三轴在 6×6 ECEF 行主序协方差 [x,vx,y,vy,z,vz] 中的对角下标：0/14/28。
std::array<double, 3U> PositionVarianceDiag(const FusedTarget& target) {
  const auto& cov = target.kinematic_estimate.covariance_ecef;
  return {cov[0], cov[14], cov[28]};
}

// 位置起始航迹：首拍量测只建航迹、不做同帧重复量测更新。
// 依据：algorithms.md §4 起始「位置记录 → ECEF 位置 + 零速 + 配置先验」；
//       两点差分初速条「首个位置量测只建航迹、不做同帧重复量测更新」。
TEST(FusionTrackFilteringTest, PositionInitFirstMeasurementSkipsSameFrameUpdate) {
  FusionConfig config = MakeFilteringConfig();  // 起始位置 1-σ 2000 m
  config.default_position_noise_std_m = 50.0;
  FusionEngine engine(config);
  const EcefPositionM truth(kEarthRadiusM, 0.0, 0.0);
  const LlaPositionDegM measured = EcefToLla(truth);

  const auto targets = engine.Update({MakePositionRecord(21U, measured)}, 1U);
  ASSERT_EQ(targets.size(), 1U);
  const FusedTarget& target = targets.front();
  ASSERT_TRUE(target.has_kinematic_estimate);

  // 零速起始（配置先验）。
  EXPECT_DOUBLE_EQ(target.kinematic_estimate.velocity_ecef_m_per_s[0], 0.0);
  EXPECT_DOUBLE_EQ(target.kinematic_estimate.velocity_ecef_m_per_s[1], 0.0);
  EXPECT_DOUBLE_EQ(target.kinematic_estimate.velocity_ecef_m_per_s[2], 0.0);

  // 起始均值直接落在量测位置。
  EXPECT_LT(DistanceM(EstimatePositionEcef(target), truth), 1.0);

  // 位置协方差保持配置先验（track_initial_position_std_m² = 2000² = 4.0e6 m²）。
  // 若首拍又跑了一次量测更新，位置方差会塌缩到量测噪声 50² = 2.5e3 m²——
  // 据此断言首拍未执行同帧更新。
  for (double var : PositionVarianceDiag(target)) {
    EXPECT_GT(var, 1.0e6) << "first position measurement must not collapse covariance to noise";
    EXPECT_LT(var, 1.0e7) << "position covariance must equal the configured init prior";
  }
}

// 位置起始航迹：第二个位置量测（物理间隔 > 50 ms）以 (p₂ − p₁)/Δt 直接建立初速。
// 依据：algorithms.md §4 两点差分初速条。
TEST(FusionTrackFilteringTest, PositionInitSecondMeasurementDifferencingFiresAboveGate) {
  FusionConfig config = MakeFilteringConfig();
  config.default_position_noise_std_m = 1.0;  // 差分精度只取决于位置差
  const EcefPositionM p0(kEarthRadiusM, 0.0, 0.0);
  const EcefPositionM v(300.0, -1800.0, 80.0);
  auto make_record = [&](double t_sec) {
    const EcefPositionM truth(p0.x_m + v.x_m * t_sec, p0.y_m + v.y_m * t_sec,
                              p0.z_m + v.z_m * t_sec);
    DetectionRecord record = MakePositionRecord(41U, EcefToLla(truth));
    record.has_timestamp = true;
    record.timestamp_s = t_sec;
    return record;
  };

  FusionEngine engine(config);
  engine.Update({make_record(1.0)}, 1U);
  const auto result = engine.Update({make_record(1.1)}, 2U);  // Δt = 0.1 s > 50 ms
  ASSERT_EQ(result.size(), 1U);
  ASSERT_TRUE(result.front().has_kinematic_estimate);

  // 直接差分：(p₂ − p₁)/Δt。位置为真值，故应精确复现 v。
  const auto& vel = result.front().kinematic_estimate.velocity_ecef_m_per_s;
  EXPECT_NEAR(vel[0], v.x_m, 1.0);
  EXPECT_NEAR(vel[1], v.y_m, 1.0);
  EXPECT_NEAR(vel[2], v.z_m, 1.0);

  // 速度方差 = 2·位置方差/Δt² + (100 m/s)² 地板。此处 2·(1 m)²/(0.1 s)² = 200 m²/s²，
  // 远低于地板 1.0e4 m²/s²——地板必须在场（挡长间隔差分方差过小导致滤波过信）。
  const auto& cov = result.front().kinematic_estimate.covariance_ecef;
  EXPECT_GE(cov[7], 1.0e4) << "vx variance must carry the (100 m/s)^2 floor";
  EXPECT_GE(cov[21], 1.0e4) << "vy variance must carry the (100 m/s)^2 floor";
  EXPECT_GE(cov[35], 1.0e4) << "vz variance must carry the (100 m/s)^2 floor";
}

// 方位起始航迹：首拍量测照常执行量测更新（角度-only 距离收敛依赖此更新）。
// 依据：algorithms.md §4 两点差分初速条「方位起始航迹不差分，且首拍量测照常执行
//       量测更新」；design.md 追认条附带修复记录（首版跳过首拍更新致评估会话
//       60 周期落点收敛回退 448 km，超 300 km 门限）。
TEST(FusionTrackFilteringTest, BearingInitFirstMeasurementPerformsUpdate) {
  FusionConfig config = MakeFilteringConfig();
  // 收窄起始距离先验（默认 300000 m）：横向前验过大会让 UKF 在首拍量测上进入强
  // 非线性区，导致“更新是否发生”的协方差判据失效。30000 m 于 250000 m 距离处
  // 对应约 7°，量测近线性，横向后验 ≈ (250000·2e-4)² ≈ 2.5e3 m² 可判。
  config.track_bearing_init_range_std_m = 30000.0;
  const double prior_var = 30000.0 * 30000.0;  // 各向同性起始位置方差 9.0e8 m²

  const EcefPositionM origin(kEarthRadiusM + 20000.0, 0.0, 0.0);
  const LlaPositionDegM origin_lla = EcefToLla(origin);
  const TestEnuBasis basis = MakeBasis(origin);
  // LOS 精确沿 ECEF +y (= 原点处 ENU east)，真值恰在起始距离先验 250000 m 上：
  // 方位量测优良条件（az=90°、el=0°，非简并），横向 = ECEF x/z 两轴。
  const EcefPositionM truth(origin.x_m, origin.y_m + config.track_bearing_init_range_m,
                            origin.z_m);
  const EcefPositionDegM bearing = BearingTo(origin, truth, basis);

  FusionEngine engine(config);
  const auto targets =
      engine.Update({MakeBearingRecord(31U, origin, bearing, 2.0e-4, origin_lla)}, 1U);
  ASSERT_EQ(targets.size(), 1U);
  const FusedTarget& target = targets.front();
  ASSERT_TRUE(target.has_kinematic_estimate);

  const std::array<double, 3U> pos_var = PositionVarianceDiag(target);
  // 首拍更新若被跳过，三轴方差均停留在各向同性先验 9.0e8 m²。更新发生后横向两轴
  // 应塌缩到 (range·σ)² ≈ (250000·2e-4)² ≈ 2.5e3 m²，远低于先验；LOS 方向（y）
  // 保持弱可观测、不被方位量测压低。
  EXPECT_LT(pos_var[0], 0.1 * prior_var)
      << "x (cross-range) variance must reflect the first-cycle bearing update";
  EXPECT_LT(pos_var[2], 0.1 * prior_var)
      << "z (cross-range) variance must reflect the first-cycle bearing update";
  EXPECT_GT(pos_var[1], 0.5 * prior_var)
      << "range (LOS) direction stays weakly observable and must not collapse";
}

// 位置起始航迹的双点差分也走纯周期 dt（algorithms.md §4 状态与时间对齐：「纯周期
// 驱动回退为 dt = (cycle − 上次滤波周期) × track_cycle_period_sec」）。无时间戳时
// 首发零速、次发按周期 dt 差分——验证初始化时间基准不依赖时间戳通道。
TEST(FusionTrackFilteringTest, PositionInitDifferencingUsesPeriodicDtWithoutTimestamps) {
  FusionConfig config = MakeFilteringConfig();
  config.default_position_noise_std_m = 1.0;
  config.track_cycle_period_sec = 1.0;
  const EcefPositionM p0(kEarthRadiusM, 0.0, 0.0);
  const EcefPositionM v(0.0, 200.0, 0.0);
  auto record = [&](std::uint64_t cycle) {
    const double t = static_cast<double>(cycle - 1U);
    const EcefPositionM truth(p0.x_m + v.x_m * t, p0.y_m + v.y_m * t, p0.z_m + v.z_m * t);
    return MakePositionRecord(61U, EcefToLla(truth));  // 无 has_timestamp
  };

  FusionEngine engine(config);
  const auto first = engine.Update({record(1U)}, 1U);
  ASSERT_EQ(first.size(), 1U);
  ASSERT_TRUE(first.front().has_kinematic_estimate);
  EXPECT_DOUBLE_EQ(first.front().kinematic_estimate.velocity_ecef_m_per_s[1], 0.0);

  const auto second = engine.Update({record(2U)}, 2U);  // 周期间隔 1.0 s > 50 ms
  ASSERT_EQ(second.size(), 1U);
  ASSERT_TRUE(second.front().has_kinematic_estimate);
  const auto& vel = second.front().kinematic_estimate.velocity_ecef_m_per_s;
  EXPECT_NEAR(vel[0], v.x_m, 1.0);
  EXPECT_NEAR(vel[1], v.y_m, 1.0);
  EXPECT_NEAR(vel[2], v.z_m, 1.0);
}

// IMPL-DERIVED：规格只声明「物理间隔 > 50 ms 才两点差分」这一门槛，未规定 ≤ 50 ms
// 时第二个位置量测如何处置。实测当前行为：差分分支不触发；该分支既不预测也不返回，
// 记录随后落到常规位置量测更新（同状态、无时间推进），而位置量测对速度增益为零，
// 故初速保持在起始零速、不被爆速污染。本条只记录当前行为，不构成正确性证据。
TEST(FusionTrackFilteringTest, PositionInitSecondMeasurementBelowGateKeepsZeroVelocity) {
  FusionConfig config = MakeFilteringConfig();
  config.default_position_noise_std_m = 1.0;
  const EcefPositionM p1(kEarthRadiusM, 0.0, 0.0);
  // 第二点相对第一点偏移 1000 m；若差分误触发将得到 1000/0.04 = 25000 m/s 爆速。
  const EcefPositionM p2(p1.x_m + 1000.0, p1.y_m, p1.z_m);
  auto record = [](double t_sec, const EcefPositionM& pos) {
    DetectionRecord r = MakePositionRecord(51U, EcefToLla(pos));
    r.has_timestamp = true;
    r.timestamp_s = t_sec;
    return r;
  };

  FusionEngine engine(config);
  engine.Update({record(1.0, p1)}, 1U);
  const auto result = engine.Update({record(1.04, p2)}, 2U);  // Δt = 0.04 s ≤ 50 ms
  ASSERT_EQ(result.size(), 1U);
  ASSERT_TRUE(result.front().has_kinematic_estimate);
  const auto& vel = result.front().kinematic_estimate.velocity_ecef_m_per_s;
  EXPECT_DOUBLE_EQ(vel[0], 0.0) << "differencing must not fire below the 50 ms gate";
  EXPECT_DOUBLE_EQ(vel[1], 0.0);
  EXPECT_DOUBLE_EQ(vel[2], 0.0);
}

}  // namespace
}  // namespace fusion
