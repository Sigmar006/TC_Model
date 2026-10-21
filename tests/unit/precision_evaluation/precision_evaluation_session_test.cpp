#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/coordinate/inertial_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/fusion/DetectionRecord.h"
#include "1q/fusion/FusionEngine.h"
#include "1q/precision_evaluation/PrecisionEvaluationInputs.h"
#include "1q/precision_evaluation/PrecisionEvaluationSession.h"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"
#include "1q/precision_evaluation/SbirsBearingAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"

namespace {

namespace pe = precision_evaluation;

constexpr double kPi = 3.14159265358979323846;
// GMST≈0 的儒略日（ECI≡ECEF，同 SBIRS 单测约定）。
constexpr double kUtcJulianDay = 2451544.2230698913;

sbirs_sensor::config::SbirsSessionConfig SatelliteConfig(float scan_start_az_deg) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.hardware.integration_time_sec = 1.0f;
  config.mission.work_mode = sbirs_sensor::config::SbirsWorkMode::kSearchAndStare;
  config.mission.scan_start_az_deg = scan_start_az_deg;
  config.mission.scan_span_deg = 11.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.mission.wide_field_fov_az_deg = 20.0f;
  config.mission.wide_field_fov_el_deg = 20.0f;
  config.mission.narrow_field_fov_az_deg = 5.0f;
  config.mission.narrow_field_fov_el_deg = 5.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  // 零误差注入：量测角=真值角（角度/双星交会误差应≈0）。
  config.policy.error_model.attitude_sigma_deg = 0.0f;
  config.policy.error_model.orbit_sigma_deg = 0.0f;
  config.policy.error_model.fov_sigma_deg = 0.0f;
  config.policy.error_model.range_fraction_sigma = 0.0f;
  return config;
}

// 双星场景：A 星 (7e6,0,0)（目标方位 ≈80.5°）、B 星 (0,7e6,0)（目标方位 ≈352.9°），
// 目标 (8e6,6e6,0)（|r|=1e7）+ 指向地心的径向下降速度 −4000·r̂（径向入射保证推演
// 时域内落地；纯切向速度会先绕半圈椭圆、远超 1200 s 时域）。
pe::EvaluationTruthTarget DescendingTarget() {
  pe::EvaluationTruthTarget truth;
  truth.key = 1U;
  truth.position_ecef_m = oneq::coordinate::EcefPositionM(8.0e6, 6.0e6, 0.0);
  truth.has_velocity = true;
  truth.velocity_ecef_m_per_s = oneq::coordinate::EcefVelocityMps(-3200.0, -2400.0, 0.0);
  truth.radiant_intensity_w_per_sr = 1.0e8;
  return truth;
}

pe::PrecisionEvaluationConfig EvaluationConfig() {
  pe::PrecisionEvaluationConfig config;
  config.satellite_a = SatelliteConfig(79.5f);   // 周期 1 扫描中心 ≈80.5°（正对目标）
  config.satellite_b = SatelliteConfig(351.9f);  // 周期 1 扫描中心 ≈352.9°
  config.inference.prediction_horizon_sec = 1200.0;  // 径向入射 ~480 s 落地
  return config;
}

pe::DualSatEphemerisInput DualSatEphemeris() {
  pe::DualSatEphemerisInput ephemeris;
  ephemeris.satellite_a_position_ecef_m = oneq::coordinate::EcefPositionM(7.0e6, 0.0, 0.0);
  ephemeris.satellite_b_position_ecef_m = oneq::coordinate::EcefPositionM(0.0, 7.0e6, 0.0);
  return ephemeris;
}

sbirs_sensor::session::SbirsCycleInput BuildSatelliteCycleInput(
    std::uint32_t cycle_index, const oneq::coordinate::EcefPositionM& satellite_position,
    const oneq::coordinate::EcefVelocityMps& satellite_velocity, double yaw_deg, double pitch_deg,
    double roll_deg, const std::vector<pe::EvaluationTruthTarget>& truth_targets) {
  sbirs_sensor::session::SbirsCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 1.0f;
  input.utc_julian_day = kUtcJulianDay;
  input.satellite_position_ecef_m = sbirs_sensor::session::SbirsVector3M{
      satellite_position.x_m, satellite_position.y_m, satellite_position.z_m};
  input.satellite_velocity_ecef_m_per_s = sbirs_sensor::session::SbirsVector3M{
      satellite_velocity.x_mps, satellite_velocity.y_mps, satellite_velocity.z_mps};
  input.satellite_attitude_eci_body_deg =
      sbirs_sensor::session::SbirsEulerAnglesDeg{yaw_deg, pitch_deg, roll_deg};
  input.scene.reserve(truth_targets.size());
  for (std::size_t i = 0U; i < truth_targets.size(); ++i) {
    const pe::EvaluationTruthTarget& truth = truth_targets[i];
    sbirs_sensor::session::SbirsSceneTarget scene_target;
    scene_target.target_id = truth.key;
    scene_target.target_name = "eval_truth";
    scene_target.position_ecef_m = sbirs_sensor::session::SbirsVector3M{
        truth.position_ecef_m.x_m, truth.position_ecef_m.y_m, truth.position_ecef_m.z_m};
    scene_target.radiant_intensity_w_per_sr = truth.radiant_intensity_w_per_sr;
    scene_target.active = truth.active;
    scene_target.has_velocity_ecef_m_per_s = truth.has_velocity;
    scene_target.velocity_ecef_m_per_s = sbirs_sensor::session::SbirsVector3M{
        truth.velocity_ecef_m_per_s.x_mps, truth.velocity_ecef_m_per_s.y_mps,
        truth.velocity_ecef_m_per_s.z_mps};
    input.scene.push_back(scene_target);
  }
  return input;
}

class PrecisionEvaluationSessionRunner {
 public:
  explicit PrecisionEvaluationSessionRunner(const pe::PrecisionEvaluationConfig& config)
      : session_(config),
        satellite_a_(sbirs_sensor::session::SbirsSession::Create(config.satellite_a)),
        satellite_b_(sbirs_sensor::session::SbirsSession::Create(config.satellite_b)),
        fusion_engine_(MakeFusionConfig(config)),
        source_a_(config.satellite_a_source_id),
        source_b_(config.satellite_b_source_id) {
    if (source_b_ == source_a_) {
      source_b_ = source_a_ + 100U;
    }
  }

  pe::PrecisionEvaluationReport Run(std::uint32_t cycle_count,
                                    pe::EvaluationTruthTarget truth = DescendingTarget()) {
    const pe::DualSatEphemerisInput ephemeris = DualSatEphemeris();
    for (std::uint32_t cycle = 1U; cycle <= cycle_count; ++cycle) {
      // 真值弹道逐周期推进（位置 += 速度·dt）：真值状态必须自洽，否则融合会正确地
      // 收敛到静止目标、落点推演超时域。
      truth.position_ecef_m.x_m += truth.velocity_ecef_m_per_s.x_mps;
      truth.position_ecef_m.y_m += truth.velocity_ecef_m_per_s.y_mps;
      truth.position_ecef_m.z_m += truth.velocity_ecef_m_per_s.z_mps;
      const std::vector<pe::EvaluationTruthTarget> truths{truth};
      const sbirs_sensor::session::SbirsCycleResult result_a = satellite_a_.StepWithResult(
          BuildSatelliteCycleInput(cycle, ephemeris.satellite_a_position_ecef_m,
                                   ephemeris.satellite_a_velocity_ecef_m_per_s,
                                   ephemeris.satellite_a_attitude_yaw_deg,
                                   ephemeris.satellite_a_attitude_pitch_deg,
                                   ephemeris.satellite_a_attitude_roll_deg, truths));
      const sbirs_sensor::session::SbirsCycleResult result_b = satellite_b_.StepWithResult(
          BuildSatelliteCycleInput(cycle, ephemeris.satellite_b_position_ecef_m,
                                   ephemeris.satellite_b_velocity_ecef_m_per_s,
                                   ephemeris.satellite_b_attitude_yaw_deg,
                                   ephemeris.satellite_b_attitude_pitch_deg,
                                   ephemeris.satellite_b_attitude_roll_deg, truths));
      double gmst_rad = 0.0;
      if (!oneq::coordinate::TryComputeGmstRad(kUtcJulianDay, &gmst_rad)) {
        last_cycle_result_ = pe::PrecisionEvaluationCycleResult{};
        continue;
      }
      std::vector<fusion::DetectionRecord> records =
          pe::AdaptSbirsResultToDetectionRecords(result_a, ephemeris.satellite_a_position_ecef_m,
                                                 gmst_rad, source_a_);
      const std::vector<fusion::DetectionRecord> records_b =
          pe::AdaptSbirsResultToDetectionRecords(result_b, ephemeris.satellite_b_position_ecef_m,
                                                 gmst_rad, source_b_);
      records.insert(records.end(), records_b.begin(), records_b.end());
      const std::vector<fusion::FusedTarget> tracks = fusion_engine_.Update(records, cycle);
      last_cycle_result_ =
          session_.Step(cycle, 1.0f, kUtcJulianDay, ephemeris, truths, result_a, result_b, tracks);
    }
    return session_.Summarize();
  }

  const pe::PrecisionEvaluationCycleResult& last_cycle_result() const { return last_cycle_result_; }

 private:
  static fusion::FusionConfig MakeFusionConfig(const pe::PrecisionEvaluationConfig& config) {
    fusion::FusionConfig fusion = config.fusion;
    fusion.enable_track_filtering = true;
    return fusion;
  }

  pe::PrecisionEvaluationSession session_;
  sbirs_sensor::session::SbirsSession satellite_a_;
  sbirs_sensor::session::SbirsSession satellite_b_;
  fusion::FusionEngine fusion_engine_;
  std::uint32_t source_a_{4U};
  std::uint32_t source_b_{104U};
  pe::PrecisionEvaluationCycleResult last_cycle_result_;
};

TEST(PrecisionEvaluationSessionTest, ZeroNoiseScenarioYieldsSmallAngularAndDualSatErrors) {
  PrecisionEvaluationSessionRunner runner(EvaluationConfig());
  const pe::PrecisionEvaluationReport report = runner.Run(25U);

  // 角度误差：零误差注入下量测角=真值角（含估计跟踪模式，真值初始化后验≈真值）。
  ASSERT_GT(report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kAngular)].count, 0U);
  EXPECT_LT(report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kAngular)].max, 0.01);

  // 双星交会：两条精确视线交于真值位置，误差为浮点噪声量级。
  ASSERT_GT(report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kDualSatFix)].count, 0U);
  EXPECT_LT(report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kDualSatFix)].max, 50.0);
}

TEST(PrecisionEvaluationSessionTest, VelocityAndImpactSamplesAreProducedAndFinite) {
  PrecisionEvaluationSessionRunner runner(EvaluationConfig());
  const pe::PrecisionEvaluationReport report = runner.Run(60U);

  // 速度误差：融合逐航迹滤波（评估会话强制开启）产出运动学估计后逐周期采样。
  const pe::ErrorMetricSummary& velocity =
      report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kVelocity)];
  ASSERT_GT(velocity.count, 0U);
  EXPECT_TRUE(std::isfinite(velocity.rmse));
  EXPECT_GE(velocity.rmse, 0.0);

  // 落点误差：真值状态（下降目标，时域内落地）与估计状态经同一推演引擎的落点之差。
  const pe::ErrorMetricSummary& impact =
      report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kImpactPoint)];
  ASSERT_GT(impact.count, 0U);
  EXPECT_TRUE(std::isfinite(impact.max));
  // 序列 max 含滤波早期未收敛状态（状态误差传播 ~480 s 放大），不做紧上界断言；
  // 收敛性由末周期样本验证（双源零噪声方位 60 周期收敛后应远小于 300 km）。
  ASSERT_FALSE(runner.last_cycle_result().keypoints.empty());
  const pe::KeyPointErrorSample& last_keypoint = runner.last_cycle_result().keypoints.back();
  ASSERT_TRUE(last_keypoint.has_impact);
  EXPECT_LT(last_keypoint.impact_error_m, 3.0e5);

  // AHP：默认等权矩阵合法，综合分在 (0,1]。
  EXPECT_TRUE(report.ahp_valid);
  EXPECT_TRUE(report.ahp.is_consistent);
  EXPECT_GT(report.composite_score, 0.0);
  EXPECT_LE(report.composite_score, 1.0);
}

TEST(PrecisionEvaluationSessionTest, SessionIsDeterministicAcrossRuns) {
  const pe::PrecisionEvaluationReport first =
      PrecisionEvaluationSessionRunner(EvaluationConfig()).Run(12U);
  const pe::PrecisionEvaluationReport second =
      PrecisionEvaluationSessionRunner(EvaluationConfig()).Run(12U);
  EXPECT_DOUBLE_EQ(first.composite_score, second.composite_score);
  for (std::size_t index = 0U; index < pe::kPrecisionMetricCount; ++index) {
    EXPECT_EQ(first.metrics[index].count, second.metrics[index].count);
    EXPECT_DOUBLE_EQ(first.metrics[index].rmse, second.metrics[index].rmse);
  }
}

TEST(PrecisionEvaluationSessionTest, InvalidJudgmentMatrixYieldsInvalidAhpAndZeroComposite) {
  pe::PrecisionEvaluationConfig config = EvaluationConfig();
  config.ahp.values[2][2] = 2.0;  // 对角非 1 → 非法判断矩阵
  PrecisionEvaluationSessionRunner runner(config);
  const pe::PrecisionEvaluationReport report = runner.Run(3U);
  EXPECT_FALSE(report.ahp_valid);
  EXPECT_DOUBLE_EQ(report.composite_score, 0.0);  // 不静默退化为等权
}

TEST(PrecisionEvaluationSessionTest, EmptyTruthYieldsZeroEvidenceAndZeroComposite) {
  // 空真值：无样本、不崩溃；零证据 = 零分（无样本指标按 0 分计入综合，不因
  // count=0 → rmse=0 而得满分）。
  pe::PrecisionEvaluationSession session(EvaluationConfig());
  const pe::PrecisionEvaluationCycleResult empty = session.Step(
      1U, 1.0f, kUtcJulianDay, DualSatEphemeris(), {}, sbirs_sensor::session::SbirsCycleResult{},
      sbirs_sensor::session::SbirsCycleResult{}, {});
  EXPECT_TRUE(empty.angular.empty());
  EXPECT_TRUE(empty.dual_sat.empty());
  EXPECT_TRUE(empty.velocity.empty());
  EXPECT_TRUE(empty.keypoints.empty());

  const pe::PrecisionEvaluationReport report = session.Summarize();
  EXPECT_TRUE(report.ahp_valid);  // 矩阵本身合法
  EXPECT_FALSE(report.all_metrics_sampled);
  EXPECT_DOUBLE_EQ(report.composite_score, 0.0);
  for (std::size_t index = 0U; index < pe::kPrecisionMetricCount; ++index) {
    EXPECT_EQ(report.metrics[index].count, 0U);
    EXPECT_DOUBLE_EQ(report.metric_scores[index], 0.0);
  }
}

TEST(PrecisionEvaluationSessionTest, SingleSatelliteDetectionSkipsDualSatFix) {
  // B 星扫描背向目标 → 仅 A 星检出：角度样本只来自卫星 0，无双星交会样本。
  pe::PrecisionEvaluationConfig config = EvaluationConfig();
  config.satellite_b.mission.scan_start_az_deg = 200.0f;
  PrecisionEvaluationSessionRunner runner(config);
  const pe::PrecisionEvaluationReport report = runner.Run(5U);
  const pe::PrecisionEvaluationCycleResult& last = runner.last_cycle_result();
  ASSERT_FALSE(last.angular.empty());
  for (const pe::AngularErrorSample& sample : last.angular) {
    EXPECT_EQ(sample.satellite_index, 0);
  }
  EXPECT_TRUE(last.dual_sat.empty());
  EXPECT_EQ(report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kDualSatFix)].count, 0U);
  // 零证据=零分：无样本指标 score=0 拖低综合；有样本指标照常计分（综合分因此
  // 介于 0 与"3/5 满分"之间——五个等权指标中双星/发射点两项零分）。
  EXPECT_DOUBLE_EQ(
      report.metric_scores[static_cast<std::size_t>(pe::PrecisionMetric::kDualSatFix)], 0.0);
  EXPECT_DOUBLE_EQ(
      report.metric_scores[static_cast<std::size_t>(pe::PrecisionMetric::kLaunchPoint)], 0.0);
  EXPECT_GT(report.metric_scores[static_cast<std::size_t>(pe::PrecisionMetric::kAngular)], 0.0);
  EXPECT_GT(report.composite_score, 0.0);
  EXPECT_LT(report.composite_score, 0.61);
}

TEST(PrecisionEvaluationSessionTest, AscendingTargetYieldsLaunchPointSamples) {
  // 径向上升目标：前推无落点（时域外）、回推解出发射点（反向下降至地表）。
  pe::EvaluationTruthTarget truth = DescendingTarget();
  truth.velocity_ecef_m_per_s = oneq::coordinate::EcefVelocityMps(3200.0, 2400.0, 0.0);
  PrecisionEvaluationSessionRunner runner(EvaluationConfig());
  const pe::PrecisionEvaluationReport report = runner.Run(60U, truth);

  const pe::ErrorMetricSummary& launch =
      report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kLaunchPoint)];
  ASSERT_GT(launch.count, 0U);
  EXPECT_TRUE(std::isfinite(launch.max));
  ASSERT_FALSE(runner.last_cycle_result().keypoints.empty());
  EXPECT_TRUE(runner.last_cycle_result().keypoints.back().has_launch);
}

// --- 双星窗口配对（dual_sat_pair_window_cycles）聚焦测试 ---
// 合成检出直打 Step：几何为 A 星 (7e6,0,0)、B 星 (0,7e6,0)、静止真值目标 (8e6,6e6,0)；
// 测角零噪声（视线精确指向真值），交会解误差应为浮点噪声量级。

sbirs_sensor::session::SbirsCycleResult SingleDetectionResult(double az_rad, double el_rad) {
  sbirs_sensor::session::SbirsCycleResult result;
  result.status = sbirs_sensor::session::SbirsCycleStatus::kCompleted;
  sbirs_sensor::output::SbirsDetectionRecord detection;
  detection.detection_id = 1U;
  detection.azimuth_rad = static_cast<float>(az_rad);
  detection.elevation_rad = static_cast<float>(el_rad);
  // 2026-08-31 裁定口径：双星定位仅收窄场数据——合成检出按窄场跟踪阶段喂入
  //（宽场阶段记录在会话聚合处被剔除，见 WideStageDetectionExcludedFromDualSat）。
  detection.observation_stage =
      sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack;
  detection.detected = true;
  result.output_frame.detections.push_back(detection);
  sbirs_sensor::attribution::SbirsDetectionAttributionRecord attribution;
  attribution.detection_id = 1U;
  attribution.target_id = 7U;
  result.detection_attributions.push_back(attribution);
  return result;
}

// 卫星→目标 ECI 极坐标角（与 SBIRS 输出角同参考系；GMST 旋入后取 atan2/asin）。
bool TryLosAzElRad(const oneq::coordinate::EcefPositionM& satellite,
                   const oneq::coordinate::EcefPositionM& target, double gmst_rad, double* az_rad,
                   double* el_rad) {
  oneq::coordinate::EciPositionM satellite_eci;
  oneq::coordinate::EciPositionM target_eci;
  if (!oneq::coordinate::TryEcefToEci(satellite, gmst_rad, &satellite_eci) ||
      !oneq::coordinate::TryEcefToEci(target, gmst_rad, &target_eci)) {
    return false;
  }
  const double los_x = target_eci.x_m - satellite_eci.x_m;
  const double los_y = target_eci.y_m - satellite_eci.y_m;
  const double los_z = target_eci.z_m - satellite_eci.z_m;
  const double range = std::sqrt(los_x * los_x + los_y * los_y + los_z * los_z);
  if (range <= 0.0) {
    return false;
  }
  double az = std::atan2(los_y, los_x);
  if (az < 0.0) {
    az += 2.0 * kPi;
  }
  *az_rad = az;
  *el_rad = std::asin(los_z / range);
  return true;
}

pe::EvaluationTruthTarget StaticWindowTarget() {
  pe::EvaluationTruthTarget truth;
  truth.key = 7U;
  truth.position_ecef_m = oneq::coordinate::EcefPositionM(8.0e6, 6.0e6, 0.0);
  return truth;
}

TEST(PrecisionEvaluationSessionTest, DualSatPairWindowZeroKeepsStrictSameCycle) {
  // 窗 0（默认）：A 星周期 1 检出、B 星周期 2 检出 → 不配对（历史行为逐位保留）。
  pe::PrecisionEvaluationSession session({});
  double gmst_rad = 0.0;
  ASSERT_TRUE(oneq::coordinate::TryComputeGmstRad(kUtcJulianDay, &gmst_rad));
  const oneq::coordinate::EcefPositionM sat_a(7.0e6, 0.0, 0.0);
  const oneq::coordinate::EcefPositionM sat_b(0.0, 7.0e6, 0.0);
  const pe::EvaluationTruthTarget truth = StaticWindowTarget();
  double az_a = 0.0;
  double el_a = 0.0;
  double az_b = 0.0;
  double el_b = 0.0;
  ASSERT_TRUE(TryLosAzElRad(sat_a, truth.position_ecef_m, gmst_rad, &az_a, &el_a));
  ASSERT_TRUE(TryLosAzElRad(sat_b, truth.position_ecef_m, gmst_rad, &az_b, &el_b));
  pe::DualSatEphemerisInput ephemeris;
  ephemeris.satellite_a_position_ecef_m = sat_a;
  ephemeris.satellite_b_position_ecef_m = sat_b;
  const std::vector<pe::EvaluationTruthTarget> truths{truth};
  const sbirs_sensor::session::SbirsCycleResult empty_result{};

  const pe::PrecisionEvaluationCycleResult cycle1 =
      session.Step(1U, 1.0f, kUtcJulianDay, ephemeris, truths,
                   SingleDetectionResult(az_a, el_a), empty_result, {});
  EXPECT_TRUE(cycle1.dual_sat.empty());
  const pe::PrecisionEvaluationCycleResult cycle2 =
      session.Step(2U, 1.0f, kUtcJulianDay, ephemeris, truths, empty_result,
                   SingleDetectionResult(az_b, el_b), {});
  EXPECT_TRUE(cycle2.dual_sat.empty());
}

TEST(PrecisionEvaluationSessionTest, DualSatPairWindowPairsCrossCycleAnchoredAtMeasurementCycle) {
  // 窗 2：A 星周期 1 检出、B 星周期 2 检出 → 窗内配对；A 星视线锚定周期 1 的卫星
  // 位置——周期 2 星历把 A 星移走 500 km，若误用当前周期位置则交会解偏离真值
  // ~百公里量级；锚定正确时零噪声视线精确交于真值。
  pe::PrecisionEvaluationConfig config;
  config.dual_sat_pair_window_cycles = 2U;
  pe::PrecisionEvaluationSession session(config);
  double gmst_rad = 0.0;
  ASSERT_TRUE(oneq::coordinate::TryComputeGmstRad(kUtcJulianDay, &gmst_rad));
  const oneq::coordinate::EcefPositionM sat_a_cycle1(7.0e6, 0.0, 0.0);
  const oneq::coordinate::EcefPositionM sat_a_cycle2(7.0e6, 5.0e5, 0.0);  // A 星移动 500 km
  const oneq::coordinate::EcefPositionM sat_b(0.0, 7.0e6, 0.0);
  const pe::EvaluationTruthTarget truth = StaticWindowTarget();
  double az_a = 0.0;
  double el_a = 0.0;
  double az_b = 0.0;
  double el_b = 0.0;
  ASSERT_TRUE(TryLosAzElRad(sat_a_cycle1, truth.position_ecef_m, gmst_rad, &az_a, &el_a));
  ASSERT_TRUE(TryLosAzElRad(sat_b, truth.position_ecef_m, gmst_rad, &az_b, &el_b));
  const std::vector<pe::EvaluationTruthTarget> truths{truth};
  const sbirs_sensor::session::SbirsCycleResult empty_result{};

  pe::DualSatEphemerisInput ephemeris_cycle1;
  ephemeris_cycle1.satellite_a_position_ecef_m = sat_a_cycle1;
  ephemeris_cycle1.satellite_b_position_ecef_m = sat_b;
  const pe::PrecisionEvaluationCycleResult cycle1 =
      session.Step(1U, 1.0f, kUtcJulianDay, ephemeris_cycle1, truths,
                   SingleDetectionResult(az_a, el_a), empty_result, {});
  EXPECT_TRUE(cycle1.dual_sat.empty());  // B 星尚无检出

  pe::DualSatEphemerisInput ephemeris_cycle2;
  ephemeris_cycle2.satellite_a_position_ecef_m = sat_a_cycle2;
  ephemeris_cycle2.satellite_b_position_ecef_m = sat_b;
  const pe::PrecisionEvaluationCycleResult cycle2 =
      session.Step(2U, 1.0f, kUtcJulianDay, ephemeris_cycle2, truths, empty_result,
                   SingleDetectionResult(az_b, el_b), {});
  ASSERT_EQ(cycle2.dual_sat.size(), 1U);
  EXPECT_EQ(cycle2.dual_sat.front().cycle_index, 2U);
  EXPECT_LT(cycle2.dual_sat.front().position_error_m, 50.0);  // 锚定测量周期位置

  // 周期 3 两侧均无新检出：旧锚点不重复配对。
  const pe::PrecisionEvaluationCycleResult cycle3 =
      session.Step(3U, 1.0f, kUtcJulianDay, ephemeris_cycle2, truths, empty_result,
                   empty_result, {});
  EXPECT_TRUE(cycle3.dual_sat.empty());
}

TEST(PrecisionEvaluationSessionTest, DualSatPairWindowRejectsBeyondWindow) {
  // 窗 1：A 星周期 1、B 星周期 4 检出（相差 3 > 1）→ 不配对。
  pe::PrecisionEvaluationConfig config;
  config.dual_sat_pair_window_cycles = 1U;
  pe::PrecisionEvaluationSession session(config);
  double gmst_rad = 0.0;
  ASSERT_TRUE(oneq::coordinate::TryComputeGmstRad(kUtcJulianDay, &gmst_rad));
  const oneq::coordinate::EcefPositionM sat_a(7.0e6, 0.0, 0.0);
  const oneq::coordinate::EcefPositionM sat_b(0.0, 7.0e6, 0.0);
  const pe::EvaluationTruthTarget truth = StaticWindowTarget();
  double az_a = 0.0;
  double el_a = 0.0;
  double az_b = 0.0;
  double el_b = 0.0;
  ASSERT_TRUE(TryLosAzElRad(sat_a, truth.position_ecef_m, gmst_rad, &az_a, &el_a));
  ASSERT_TRUE(TryLosAzElRad(sat_b, truth.position_ecef_m, gmst_rad, &az_b, &el_b));
  pe::DualSatEphemerisInput ephemeris;
  ephemeris.satellite_a_position_ecef_m = sat_a;
  ephemeris.satellite_b_position_ecef_m = sat_b;
  const std::vector<pe::EvaluationTruthTarget> truths{truth};
  const sbirs_sensor::session::SbirsCycleResult empty_result{};

  const pe::PrecisionEvaluationCycleResult cycle1 =
      session.Step(1U, 1.0f, kUtcJulianDay, ephemeris, truths,
                   SingleDetectionResult(az_a, el_a), empty_result, {});
  EXPECT_TRUE(cycle1.dual_sat.empty());
  for (std::uint32_t cycle = 2U; cycle <= 3U; ++cycle) {
    const pe::PrecisionEvaluationCycleResult mid = session.Step(
        cycle, 1.0f, kUtcJulianDay, ephemeris, truths, empty_result, empty_result, {});
    EXPECT_TRUE(mid.dual_sat.empty());
  }
  const pe::PrecisionEvaluationCycleResult cycle4 =
      session.Step(4U, 1.0f, kUtcJulianDay, ephemeris, truths, empty_result,
                   SingleDetectionResult(az_b, el_b), {});
  EXPECT_TRUE(cycle4.dual_sat.empty());  // Δ=3 超窗
}

TEST(PrecisionEvaluationSessionTest, DualSatSameCyclePairingPersistsWithWindow) {
  // 窗 2 下同周期双检出：照常输出且零噪声误差为浮点噪声量级（窗内非回归）。
  pe::PrecisionEvaluationConfig config;
  config.dual_sat_pair_window_cycles = 2U;
  pe::PrecisionEvaluationSession session(config);
  double gmst_rad = 0.0;
  ASSERT_TRUE(oneq::coordinate::TryComputeGmstRad(kUtcJulianDay, &gmst_rad));
  const oneq::coordinate::EcefPositionM sat_a(7.0e6, 0.0, 0.0);
  const oneq::coordinate::EcefPositionM sat_b(0.0, 7.0e6, 0.0);
  const pe::EvaluationTruthTarget truth = StaticWindowTarget();
  double az_a = 0.0;
  double el_a = 0.0;
  double az_b = 0.0;
  double el_b = 0.0;
  ASSERT_TRUE(TryLosAzElRad(sat_a, truth.position_ecef_m, gmst_rad, &az_a, &el_a));
  ASSERT_TRUE(TryLosAzElRad(sat_b, truth.position_ecef_m, gmst_rad, &az_b, &el_b));
  pe::DualSatEphemerisInput ephemeris;
  ephemeris.satellite_a_position_ecef_m = sat_a;
  ephemeris.satellite_b_position_ecef_m = sat_b;
  const std::vector<pe::EvaluationTruthTarget> truths{truth};

  const pe::PrecisionEvaluationCycleResult cycle1 =
      session.Step(1U, 1.0f, kUtcJulianDay, ephemeris, truths, SingleDetectionResult(az_a, el_a),
                   SingleDetectionResult(az_b, el_b), {});
  ASSERT_EQ(cycle1.dual_sat.size(), 1U);
  EXPECT_LT(cycle1.dual_sat.front().position_error_m, 50.0);
}

// --- 2026-08-31 裁定：双星定位仅收窄场数据（宽场粗测角只承担引导） ---

TEST(PrecisionEvaluationSessionTest, WideStageDetectionExcludedFromDualSat) {
  // 同周期双星几何完备，但两侧均为宽场搜索阶段记录：交会不得产出。
  pe::PrecisionEvaluationConfig config;
  config.dual_sat_pair_window_cycles = 1U;
  pe::PrecisionEvaluationSession session(config);
  double gmst_rad = 0.0;
  ASSERT_TRUE(oneq::coordinate::TryComputeGmstRad(kUtcJulianDay, &gmst_rad));
  const oneq::coordinate::EcefPositionM sat_a(7.0e6, 0.0, 0.0);
  const oneq::coordinate::EcefPositionM sat_b(0.0, 7.0e6, 0.0);
  const pe::EvaluationTruthTarget truth = StaticWindowTarget();
  double az_a = 0.0;
  double el_a = 0.0;
  double az_b = 0.0;
  double el_b = 0.0;
  ASSERT_TRUE(TryLosAzElRad(sat_a, truth.position_ecef_m, gmst_rad, &az_a, &el_a));
  ASSERT_TRUE(TryLosAzElRad(sat_b, truth.position_ecef_m, gmst_rad, &az_b, &el_b));
  sbirs_sensor::session::SbirsCycleResult result_a = SingleDetectionResult(az_a, el_a);
  sbirs_sensor::session::SbirsCycleResult result_b = SingleDetectionResult(az_b, el_b);
  // 降级回宽场搜索阶段（其余逐位不变）。
  result_a.output_frame.detections.front().observation_stage =
      sbirs_sensor::output::SbirsObservationStage::kWideFieldSearch;
  result_b.output_frame.detections.front().observation_stage =
      sbirs_sensor::output::SbirsObservationStage::kWideFieldSearch;
  pe::DualSatEphemerisInput ephemeris;
  ephemeris.satellite_a_position_ecef_m = sat_a;
  ephemeris.satellite_b_position_ecef_m = sat_b;
  const std::vector<pe::EvaluationTruthTarget> truths{truth};
  const pe::PrecisionEvaluationCycleResult cycle1 =
      session.Step(1U, 1.0f, kUtcJulianDay, ephemeris, truths, result_a, result_b, {});
  EXPECT_TRUE(cycle1.dual_sat.empty());
}

// --- SBIRS→fusion 适配器几何往返（评估侧跨系对齐的专项验证） ---

TEST(SbirsBearingAdapterTest, EciBearingMapsToSatelliteLocalEnu) {
  // 赤道 x 轴卫星 (7e6,0,0)（大地=地心坐标重合），目标 (8e6,2e6,0)：ECI 视线
  // (1,2,0)/√5 → ENU（east=(0,1,0), north=(0,0,1), up=(1,0,0)）：
  // az=atan2(east,north)=90°、el=asin(up 分量)=asin(1/√5)。
  sbirs_sensor::session::SbirsCycleResult result;
  result.status = sbirs_sensor::session::SbirsCycleStatus::kCompleted;
  sbirs_sensor::output::SbirsDetectionRecord detection;
  detection.detection_id = 1U;
  const double los_x = 1.0e6;
  const double los_y = 2.0e6;
  const double los_norm = std::sqrt(los_x * los_x + los_y * los_y);
  detection.azimuth_rad = static_cast<float>(std::atan2(los_y, los_x));
  detection.elevation_rad = 0.0f;
  detection.infrared_snr_linear = 500.0f;
  detection.detected = true;
  detection.measurement_time_s = 45.678;
  result.output_frame.detections.push_back(detection);
  sbirs_sensor::attribution::SbirsDetectionAttributionRecord attribution;
  attribution.detection_id = 1U;
  attribution.target_id = 42U;
  result.detection_attributions.push_back(attribution);

  const oneq::coordinate::EcefPositionM satellite(7.0e6, 0.0, 0.0);
  const std::vector<fusion::DetectionRecord> records =
      precision_evaluation::AdaptSbirsResultToDetectionRecords(result, satellite, 0.0,
                                                                         7U);
  ASSERT_EQ(records.size(), 1U);
  const fusion::DetectionRecord& record = records.front();
  EXPECT_EQ(record.key, 42U);
  EXPECT_EQ(record.source_id, 7U);
  EXPECT_TRUE(record.has_bearing);
  EXPECT_NEAR(record.bearing_az_deg, 90.0, 1.0e-4);
  EXPECT_NEAR(record.bearing_el_deg, std::asin(los_x / los_norm) * 180.0 / kPi, 1.0e-4);
  EXPECT_TRUE(record.has_sensor_origin);
  oneq::coordinate::LlaPositionDegM expected_origin;
  ASSERT_TRUE(oneq::coordinate::TryEcefToLla(satellite, &expected_origin));
  EXPECT_DOUBLE_EQ(record.sensor_origin.latitude_deg, expected_origin.latitude_deg);
  EXPECT_DOUBLE_EQ(record.sensor_origin.longitude_deg, expected_origin.longitude_deg);
  EXPECT_DOUBLE_EQ(record.sensor_origin.altitude_m, expected_origin.altitude_m);
  EXPECT_NEAR(record.quality, 0.5, 1.0e-9);  // SNR 500 / 1000
  EXPECT_TRUE(record.has_timestamp);
  EXPECT_DOUBLE_EQ(record.timestamp_s, 45.678);

  // 非执行周期（关机/待机）显式空返回，不产出量测。
  result.status = sbirs_sensor::session::SbirsCycleStatus::kPoweredOff;
  EXPECT_TRUE(precision_evaluation::AdaptSbirsResultToDetectionRecords(result, satellite,
                                                                                0.0, 7U)
                  .empty());
}

// --- 宿主检测级输入装配（第 26/27 项现场接入路径） ---

pe::PrecisionDetectionInput MakePrecisionDetection(std::uint64_t detection_id,
                                                  std::uint64_t target_id, float azimuth_rad) {
  pe::PrecisionDetectionInput detection;
  detection.detection_id = detection_id;
  detection.target_id = target_id;
  detection.measurement_time_s = 1.0;
  detection.azimuth_rad = azimuth_rad;
  detection.elevation_rad = 0.0f;
  detection.observation_stage = sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack;
  detection.detected = true;
  return detection;
}

TEST(PrecisionEvaluationInputsTest, BuildsDetectionAndAttributionRecords) {
  pe::PrecisionSatelliteCycleInput input;
  input.cycle_index = 7U;
  pe::PrecisionDetectionInput attributed = MakePrecisionDetection(11U, 101U, 1.4056f);
  attributed.has_focal_plane_offset = true;
  attributed.focal_plane_offset_x_m = 1.5e-3f;
  attributed.focal_plane_offset_y_m = -2.5e-3f;
  input.detections.push_back(attributed);
  // 未归属检测：不建归属记录，会话因此不会把它送进双星交会。
  input.detections.push_back(MakePrecisionDetection(12U, 0U, 1.5f));

  const sbirs_sensor::session::SbirsCycleResult result =
      pe::BuildSatelliteCycleResult(input);

  EXPECT_EQ(result.status, sbirs_sensor::session::SbirsCycleStatus::kCompleted);
  EXPECT_EQ(result.input_cycle_index, 7U);
  EXPECT_EQ(result.output_frame.cycle_index, 7U);
  ASSERT_EQ(result.output_frame.detections.size(), 2U);
  EXPECT_EQ(result.output_frame.detections[0].detection_id, 11U);
  EXPECT_FLOAT_EQ(result.output_frame.detections[0].azimuth_rad, 1.4056f);
  EXPECT_EQ(result.output_frame.detections[0].observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack);
  EXPECT_TRUE(result.output_frame.detections[0].detected);
  ASSERT_EQ(result.detection_attributions.size(), 1U);
  const sbirs_sensor::attribution::SbirsDetectionAttributionRecord& attribution =
      result.detection_attributions[0];
  EXPECT_EQ(attribution.detection_id, 11U);
  EXPECT_EQ(attribution.target_id, 101U);
  EXPECT_TRUE(attribution.has_focal_plane_offset);
  EXPECT_FLOAT_EQ(attribution.focal_plane_offset_x_m, 1.5e-3f);
  EXPECT_FLOAT_EQ(attribution.focal_plane_offset_y_m, -2.5e-3f);
}

TEST(PrecisionEvaluationInputsTest, SessionConsumesBuiltResultAndSamplesAngularError) {
  pe::PrecisionEvaluationSession session{};
  pe::DualSatEphemerisInput ephemeris;
  ephemeris.satellite_a_position_ecef_m = oneq::coordinate::EcefPositionM(7.0e6, 0.0, 0.0);
  ephemeris.satellite_b_position_ecef_m = oneq::coordinate::EcefPositionM(0.0, 7.0e6, 0.0);

  pe::EvaluationTruthTarget truth;
  truth.key = 101U;
  truth.position_ecef_m = oneq::coordinate::EcefPositionM(8.0e6, 6.0e6, 0.0);

  pe::PrecisionSatelliteCycleInput input;
  input.cycle_index = 1U;
  // 视线精确指向真值：方位 = atan2(6e6, 8e6 − 7e6)，俯仰 0 ⇒ 角度误差应为 0 量级。
  const float exact_azimuth_rad = static_cast<float>(std::atan2(6.0e6, 1.0e6));
  input.detections.push_back(MakePrecisionDetection(1U, 101U, exact_azimuth_rad));

  const pe::PrecisionEvaluationCycleResult cycle = session.Step(
      1U, 1.0f, kUtcJulianDay, ephemeris, std::vector<pe::EvaluationTruthTarget>{truth},
      pe::BuildSatelliteCycleResult(input), sbirs_sensor::session::SbirsCycleResult{}, {});

  ASSERT_EQ(cycle.angular.size(), 1U);
  EXPECT_EQ(cycle.angular[0].key, 101U);
  EXPECT_LT(std::fabs(cycle.angular[0].azimuth_error_deg), 1.0e-3);
  EXPECT_LT(std::fabs(cycle.angular[0].elevation_error_deg), 1.0e-3);
}

}  // namespace
