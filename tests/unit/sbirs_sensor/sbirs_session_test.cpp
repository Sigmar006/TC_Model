#include <gtest/gtest.h>

#include <limits>
#include <fstream>
#include <iterator>
#include "common/logging/AcceptanceFileLog.h"
#include <string>

#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"

namespace {

sbirs_sensor::session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_sensor::session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

sbirs_sensor::session::SbirsCycleInput ValidInput(std::uint32_t cycle_index) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.target_name = "hot";
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e8;
  return sbirs_sensor::session::SbirsCycleInputBuilder()
      .WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
      .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
      .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
      .AddTarget(target)
      .Build();
}

sbirs_sensor::config::SbirsSessionConfig Config() {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.hardware.integration_time_sec = 1.0f;
  config.mission.scan_start_az_deg = -1.0f;
  config.mission.scan_span_deg = 11.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  return config;
}

TEST(SbirsSessionTest, ValidCycleProducesDeterministicOutput) {
  sbirs_sensor::session::SbirsSession session =
      sbirs_sensor::session::SbirsSession::Create(Config());
  const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(ValidInput(1U));
  EXPECT_EQ(result.status, sbirs_sensor::session::SbirsCycleStatus::kCompleted);
  EXPECT_FALSE(HasValidationError(result.issues));
  EXPECT_FALSE(result.output_frame.detections.empty());
}

TEST(SbirsSessionTest, InvalidFirstCycleReturnsEmptyOutput) {
  sbirs_sensor::session::SbirsSession session =
      sbirs_sensor::session::SbirsSession::Create(Config());
  sbirs_sensor::session::SbirsCycleInput input = ValidInput(1U);
  input.dt_sec = -1.0f;
  const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(input);
  EXPECT_NE(result.status, sbirs_sensor::session::SbirsCycleStatus::kCompleted);
  EXPECT_TRUE(HasValidationError(result.issues));
  EXPECT_TRUE(result.output_frame.detections.empty());
}

TEST(SbirsSessionTest, InvalidLaterCycleReturnsEmptyOutputNotReused) {
  sbirs_sensor::session::SbirsSession session =
      sbirs_sensor::session::SbirsSession::Create(Config());
  const sbirs_sensor::session::SbirsCycleResult valid = session.StepWithResult(ValidInput(1U));
  ASSERT_FALSE(valid.output_frame.detections.empty());
  sbirs_sensor::session::SbirsCycleInput invalid = ValidInput(2U);
  invalid.dt_sec = 0.0f;
  const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(invalid);
  EXPECT_NE(result.status, sbirs_sensor::session::SbirsCycleStatus::kCompleted);
  // 非执行周期返回默认空帧，不复用上一有效输出（统一不复用语义）。
  EXPECT_TRUE(result.output_frame.detections.empty());
  EXPECT_EQ(result.output_frame.cycle_index, 0U);
}

TEST(SbirsSessionTest, ValidationRejectDoesNotAdvancePipelineState) {
  sbirs_sensor::session::SbirsSession rejected_then_valid =
      sbirs_sensor::session::SbirsSession::Create(Config());
  sbirs_sensor::session::SbirsCycleInput invalid = ValidInput(1U);
  invalid.scene.front().radiant_intensity_w_per_sr =
      std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(sbirs_sensor::session::HasValidationError(
      rejected_then_valid.StepWithResult(invalid).issues));

  const sbirs_sensor::session::SbirsCycleResult after_reject =
      rejected_then_valid.StepWithResult(ValidInput(2U));
  sbirs_sensor::session::SbirsSession clean =
      sbirs_sensor::session::SbirsSession::Create(Config());
  const sbirs_sensor::session::SbirsCycleResult clean_result =
      clean.StepWithResult(ValidInput(2U));

  ASSERT_EQ(after_reject.status, sbirs_sensor::session::SbirsCycleStatus::kCompleted);
  ASSERT_EQ(after_reject.output_frame.detections.size(),
            clean_result.output_frame.detections.size());
  ASSERT_FALSE(after_reject.output_frame.detections.empty());
  EXPECT_FLOAT_EQ(after_reject.output_frame.scan_azimuth_rad,
                  clean_result.output_frame.scan_azimuth_rad);
  EXPECT_EQ(after_reject.output_frame.detections.front().detection_id,
            clean_result.output_frame.detections.front().detection_id);
  EXPECT_FLOAT_EQ(after_reject.output_frame.detections.front().azimuth_rad,
                  clean_result.output_frame.detections.front().azimuth_rad);
  EXPECT_FLOAT_EQ(after_reject.output_frame.detections.front().elevation_rad,
                  clean_result.output_frame.detections.front().elevation_rad);
  EXPECT_FLOAT_EQ(after_reject.output_frame.detections.front().infrared_snr_linear,
                  clean_result.output_frame.detections.front().infrared_snr_linear);
}

TEST(SbirsRuntimeConfigResolverTest, InvalidPatchDoesNotPolluteConfig) {
  sbirs_sensor::session::SbirsSession session =
      sbirs_sensor::session::SbirsSession::Create(Config());
  sbirs_sensor::config::SbirsRuntimeConfigPatch invalid;
  invalid.has_scan_rate_deg_per_sec = true;
  invalid.scan_rate_deg_per_sec = -5.0f;
  EXPECT_FALSE(session.TryApplyRuntimeConfig(invalid));
  const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(ValidInput(1U));
  EXPECT_EQ(result.status, sbirs_sensor::session::SbirsCycleStatus::kCompleted);
}

TEST(SbirsSessionTest, AttachRecorderDrivesUpdateAutomatically) {
  sbirs_sensor::session::SbirsSession session =
      sbirs_sensor::session::SbirsSession::Create(Config());
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder recorder;

  session.AttachDetectionLifecycleRecorder(&recorder);

  const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(ValidInput(1U));
  ASSERT_EQ(result.status, sbirs_sensor::session::SbirsCycleStatus::kCompleted);
  ASSERT_FALSE(result.output_frame.detections.empty());

  const std::vector<sbirs_sensor::session::SbirsDetectionLifecycleEvent>& events =
      recorder.GetLastEvents();
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events.front().kind,
            sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kFirstDetected);
}

TEST(SbirsSessionTest, DetachRecorderStopsAutomaticDriving) {
  sbirs_sensor::session::SbirsSession session =
      sbirs_sensor::session::SbirsSession::Create(Config());
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder recorder;

  session.AttachDetectionLifecycleRecorder(&recorder);
  session.StepWithResult(ValidInput(1U));
  const std::size_t first_count = recorder.GetLastEvents().size();
  ASSERT_GT(first_count, 0U);

  session.AttachDetectionLifecycleRecorder(nullptr);
  session.StepWithResult(ValidInput(2U));
  EXPECT_EQ(recorder.GetLastEvents().size(), first_count);
}

TEST(SbirsSessionTest, SessionWithoutRecorderIsBackwardCompatible) {
  sbirs_sensor::session::SbirsSession session =
      sbirs_sensor::session::SbirsSession::Create(Config());
  const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(ValidInput(1U));
  EXPECT_EQ(result.status, sbirs_sensor::session::SbirsCycleStatus::kCompleted);
}

TEST(SbirsSessionTest, GetLastEventsEmptyAfterConstruction) {
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder recorder;
  EXPECT_TRUE(recorder.GetLastEvents().empty());
}

TEST(SbirsSessionTest, NonExecutedCycleDoesNotUpdateLastEvents) {
  sbirs_sensor::session::SbirsSession session =
      sbirs_sensor::session::SbirsSession::Create(Config());
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder recorder;
  session.AttachDetectionLifecycleRecorder(&recorder);

  // 第一个周期执行并驱动 recorder。
  session.StepWithResult(ValidInput(1U));
  const std::size_t first_size = recorder.GetLastEvents().size();

  // 非法输入（dt_sec=0）→ validation rejection → 非执行周期，缓存保持不变。
  sbirs_sensor::session::SbirsCycleInput invalid = ValidInput(2U);
  invalid.dt_sec = 0.0f;
  const sbirs_sensor::session::SbirsCycleResult rejected = session.StepWithResult(invalid);
  EXPECT_TRUE(HasValidationError(rejected.issues));
  EXPECT_EQ(recorder.GetLastEvents().size(), first_size);
}

// 规则 13b：正常执行周期的按目标排除 kInfo 诊断须经 controller 转写进 SbirsCycleResult
// （DebugView 序列化按规则 12 自然携带）。

TEST(SbirsSessionTest, ExcludedTargetDiagnosticsPropagateToCycleResult) {
  sbirs_sensor::config::SbirsSessionConfig config = Config();
  config.mission.max_range_m = 500000.0f;  // 收紧距离门：ValidInput 目标（range 1e6 m）被排除
  sbirs_sensor::session::SbirsSession session =
      sbirs_sensor::session::SbirsSession::Create(config);
  const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(ValidInput(1U));
  EXPECT_EQ(result.status, sbirs_sensor::session::SbirsCycleStatus::kCompleted);
  EXPECT_TRUE(result.output_frame.detections.empty());
  bool found = false;
  for (const auto& issue : result.issues) {
    if (issue.code == "sbirs.target_out_of_range") {
      found = true;
      EXPECT_EQ(issue.severity, sbirs_sensor::session::SbirsIssueSeverity::kInfo);
      EXPECT_NE(issue.message.find("target_id=1"), std::string::npos);
    }
  }
  EXPECT_TRUE(found);
}

}  // namespace

#if defined(ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG) && ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG
TEST(SbirsAcceptanceDirectoryTest, RoutesCycleAndDeferredRecordsToSessionDirectory) {
  using namespace oneq::logging;
  auto config_a = Config();
  auto config_b = Config();
  config_a.acceptance_log_output_directory = "sbirs_directory_test/a";
  config_b.acceptance_log_output_directory = "sbirs_directory_test/b";
  {
    auto session_a = sbirs_sensor::session::SbirsSession::Create(config_a);
    auto session_b = sbirs_sensor::session::SbirsSession::Create(config_b);
    session_a.SetSatelliteEntityId(701U);
    session_b.SetSatelliteEntityId(702U);
    auto input_a = ValidInput(1U);
    auto input_b = ValidInput(1U);
    input_a.scene[0].target_id = 801U;
    input_b.scene[0].target_id = 802U;
    EXPECT_EQ(session_a.StepWithResult(input_a).status,
              sbirs_sensor::session::SbirsCycleStatus::kCompleted);
    EXPECT_TRUE(CurrentAcceptanceLogOutputDirectory().empty());
    EXPECT_EQ(session_b.StepWithResult(input_b).status,
              sbirs_sensor::session::SbirsCycleStatus::kCompleted);
  }
  for (const auto& config : {config_a, config_b}) {
    ScopedAcceptanceLogOutputDirectory scope(config.acceptance_log_output_directory);
    FlushAcceptanceLog(AcceptanceChannel::kSatelliteTarget);
    std::ifstream file(config.acceptance_log_output_directory + "/SatelliteTarget.log");
    ASSERT_TRUE(file.good());
    const std::string text((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
    const bool is_a = config.acceptance_log_output_directory == config_a.acceptance_log_output_directory;
    EXPECT_NE(text.find(u8"目标可探测性与动态参数生成功能测试"), std::string::npos);
    EXPECT_NE(text.find(is_a ? u8"目标ID=801 " : u8"目标ID=802 "), std::string::npos);
    EXPECT_EQ(text.find(is_a ? u8"目标ID=802 " : u8"目标ID=801 "), std::string::npos);
  }
}
#endif
