// Copyright 2026. All Rights Reserved.
//
// @file rir_dual_satellite_lock_test.cpp
// @brief 双星红外视线交会位置与目标真值速度驱动 LRDR 锁定的端到端测试。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/precision_evaluation/DualLosFix.h"
#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "RirCycleInputTestUtil.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using oneq::coordinate::EcefPositionM;
using oneq::coordinate::LlaPositionDegM;
using oneq::coordinate::Vector3d;
using session::RirCycleInput;
using session::RirSceneTarget;

TEST(RirDualSatelliteLockTest, DualLosFixPositionAndTruthVelocityLockTarget) {
  constexpr std::uint64_t kExternalTargetId = 70421U;
  RirCycleInput input;
  input.input_cycle_index = 1U;
  input.dt_sec = 0.5;
  SetDefaultTestPlatformEcef(&input);

  // 中译：**几何必须落在雷达可观测空域内**（2026-11-04 修正）——原用例把真值点取在
  //   赤道 10,000 km 处，从 30°N/120°E 的平台看它在**地平线以下**（俯仰 −28°），而雷达
  //   扫描范围只有 [−30°, +30°]/[−60°, +60°]，于是驻留中心被钳到边界 +30°、探测不到、
  //   航迹归属为空，三条断言全失败。测试要验的是"交会解 → 锁定"这条链，不是"越界钳位"，
  //   所以真值点改由平台**按方位/俯仰/斜距正向生成**（方位 20°、俯仰 15°、斜距 120 km），
  //   保证它确实在雷达可见空域内，且期望值可解析算出。
  // 标识：测试几何落在可观测空域（2026-11-04）。
  // 中译：方位取 40°（2026-11-04）——雷达机内方位以**正东为 0**、逆时针为正，机内
  //   look_az = 90° − ENU 方位；原取 20° 时机内 look_az=70°，落在 [−60°, +60°] 扫描
  //   体之外（`rir.target_outside_search_volume`），根本进不了探测。40° ⇒ look_az=50°，
  //   在体内且离边界有 10° 余量。
  // 标识：测试方位取机内可扫描区（2026-11-04）。
  constexpr double kTargetAzDeg = 40.0;
  constexpr double kTargetElDeg = 15.0;
  constexpr double kTargetRangeM = 120000.0;
  LlaPositionDegM platform_lla;
  ASSERT_TRUE(oneq::coordinate::TryEcefToLla(input.platform_position, &platform_lla));

  oneq::coordinate::EnuPositionM truth_enu;
  truth_enu.east_m = kTargetRangeM * std::cos(kTargetElDeg * 3.141592653589793 / 180.0) *
                     std::sin(kTargetAzDeg * 3.141592653589793 / 180.0);
  truth_enu.north_m = kTargetRangeM * std::cos(kTargetElDeg * 3.141592653589793 / 180.0) *
                      std::cos(kTargetAzDeg * 3.141592653589793 / 180.0);
  truth_enu.up_m = kTargetRangeM * std::sin(kTargetElDeg * 3.141592653589793 / 180.0);
  EcefPositionM truth_position;
  ASSERT_TRUE(oneq::coordinate::TryEnuToEcef(truth_enu, platform_lla, &truth_position));

  // 两颗星按经纬高摆位（远在 35,000 km，保证交会良态），视线由真值反算。
  const auto satellite_ecef = [](double _lat_deg, double _lon_deg) {
    oneq::coordinate::LlaPositionDegM lla{};
    lla.latitude_deg = _lat_deg;
    lla.longitude_deg = _lon_deg;
    lla.altitude_m = 35000000.0;
    EcefPositionM ecef;
    (void)oneq::coordinate::TryLlaToEcef(lla, &ecef);
    return ecef;
  };
  const EcefPositionM satellite_a = satellite_ecef(30.0, 120.0);
  const EcefPositionM satellite_b = satellite_ecef(10.0, 140.0);
  const Vector3d los_a(truth_position.x_m - satellite_a.x_m,
                       truth_position.y_m - satellite_a.y_m,
                       truth_position.z_m - satellite_a.z_m);
  const Vector3d los_b(truth_position.x_m - satellite_b.x_m,
                       truth_position.y_m - satellite_b.y_m,
                       truth_position.z_m - satellite_b.z_m);
  EcefPositionM dual_sat_fix;
  double residual_m = 0.0;
  ASSERT_TRUE(precision_evaluation::TryComputeDualLosFixM(
      satellite_a, los_a, satellite_b, los_b, &dual_sat_fix, &residual_m));
  EXPECT_NEAR(residual_m, 0.0, 1.0e-6);
  EXPECT_NEAR(dual_sat_fix.x_m, truth_position.x_m, 1.0e-6);
  EXPECT_NEAR(dual_sat_fix.y_m, truth_position.y_m, 1.0e-6);
  EXPECT_NEAR(dual_sat_fix.z_m, truth_position.z_m, 1.0e-6);

  // LRDR 接收相对平台的 ENU 位置；双星解输出 ECEF，故先转到平台锚点 ENU。
  oneq::coordinate::EnuPositionM target_enu;
  ASSERT_TRUE(oneq::coordinate::TryEcefToEnu(dual_sat_fix, platform_lla, &target_enu));

  RirSceneTarget target;
  target.external_target_id = kExternalTargetId;
  target.target_name = "dual-satellite-fix-target";
  target.position_x = static_cast<float>(target_enu.east_m);
  target.position_y = static_cast<float>(target_enu.north_m);
  target.position_z = static_cast<float>(target_enu.up_m);
  // 按测试输入要求：位置取双星定位解，速度填场景真值。
  target.velocity_x = 100.0f;
  target.velocity_y = -20.0f;
  target.velocity_z = 5.0f;
  target.rcs = 1.0f;
  input.scene_targets.push_back(target);

  config::RirSessionConfig radar_config;
  radar_config.mission.work_mode = config::RirWorkMode::kIdentify;
  radar_config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  radar_config.policy.lifecycle.confirm_hits = 1U;
  session::RirSession radar = session::RirSession::Create(radar_config);
  config::RirRuntimeConfigPatch designation;
  designation.has_designated_target_id = true;
  designation.designated_external_target_id = kExternalTargetId;
  designation.has_designation_duration_cycles = true;
  designation.designation_duration_cycles = 10U;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(designation));

  const session::RirCycleResult result = radar.StepWithResult(input);
  ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
  EXPECT_TRUE(result.designation_active);
  EXPECT_EQ(result.designated_target_id, kExternalTargetId);
  EXPECT_EQ(result.pointing_source, session::RirPointingSource::kDesignation);

  const float expected_az_deg =
      static_cast<float>(std::atan2(target.position_y, target.position_x) * 180.0 / 3.141592653589793);
  const float horizontal_range_m =
      std::sqrt(target.position_x * target.position_x + target.position_y * target.position_y);
  const float expected_el_deg =
      static_cast<float>(std::atan2(target.position_z, horizontal_range_m) * 180.0 / 3.141592653589793);
  EXPECT_NEAR(result.dwell_center_deg.az_deg, expected_az_deg, 1.0e-3f);
  EXPECT_NEAR(result.dwell_center_deg.el_deg, expected_el_deg, 1.0e-3f);
  // 中译：输出帧的公开面是"识别结论 + 特征量测"两路（`RirOutputFrame` 无 `detections` 成员），
  //   而"雷达确实锁上了这颗目标"的权威证据是**航迹归属视图**：它把库内关联键映射回场景
  //   目标 ID。这里断言归属视图里出现指定目标，证明双星定位解 → 指定 → 探测 → 航迹归属
  //   这条链整条打通（仅断言"有输出"会把探测失败也判过）。
  ASSERT_FALSE(result.track_attributions.empty());
  bool designated_seen = false;
  for (std::size_t i = 0; i < result.track_attributions.size(); ++i) {
    if (result.track_attributions[i].external_target_id == kExternalTargetId) {
      designated_seen = true;
      break;
    }
  }
  EXPECT_TRUE(designated_seen);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
