/**
 * @file esr_controller_runtime_state_test.cpp
 * @brief 验证 ESR 控制器运行态快照与失败回退契约。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>

#include "electronic_surveillance_radar/runtime/EsrController.h"
#include "electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "electronic_surveillance_radar/config/EsrInternalExecutionConfig.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"
#include "electronic_surveillance_radar/pipeline/PipelineRuntimeSnapshot.h"

namespace electronic_surveillance_radar {
namespace extension {
namespace {

EsrInternalExecutionConfig MakeDefaultConfig() {
  EsrInternalExecutionConfig config;
  config.sensor_enabled = true;
  config.mission.scan.scan_rate_hz = 1.0f;
  config.resolved_scan.scan_start_az_deg = -60.0f;
  config.resolved_scan.scan_end_az_deg = 60.0f;
  config.resolved_scan.scan_start_el_deg = -10.0f;
  config.resolved_scan.scan_end_el_deg = 10.0f;
  config.resolved_scan.az_step_deg = 5.0f;
  config.resolved_scan.el_step_deg = 5.0f;
  config.detection.minimum_snr_db = 6.0f;
  config.detection.pfa = 1.0e-6f;
  config.detection.pulse_count = 8U;
  config.hardware.receiver_sensitivity_w = 1.0e-12f;
  config.hardware.integrated_receive_loss_db = 0.0f;
  config.orientation.antenna_mount_az_deg = 0.0f;
  config.orientation.antenna_mount_el_deg = 0.0f;
  return config;
}

class StubEnvironmentService final : public environment::IEsrEnvironmentService {
 public:
  void BeginCycle(std::uint32_t, float, float) override {}
  session::EsrEnvironmentSnapshot SampleEnvironment() const override {
    return session::EsrEnvironmentSnapshot{};
  }
  void UpdateModelConfig(config::EsrEnvironmentScenarioConfig) override {}
};

session::EsrCycleInput MakeValidInput(std::uint32_t cycle_index) {
  session::EsrCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = static_cast<double>(cycle_index);
  input.dt_sec = 1.0f;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  return input;
}

double ReadScanPhase(const pipeline::InterceptPipeline& pipeline) {
  const extension::InterceptPipelineRuntimeState state = pipeline.CaptureRuntimeState();
  const pipeline::PipelineRuntimeSnapshot* snapshot =
      pipeline::RestorePipelineSnapshot(state);
  EXPECT_NE(snapshot, nullptr);
  return snapshot == nullptr ? -1.0 : snapshot->scan_phase_cycles;
}

std::uint64_t ReadCompletedReceiveCycles(
    const pipeline::InterceptPipeline& pipeline) {
  const extension::InterceptPipelineRuntimeState state =
      pipeline.CaptureRuntimeState();
  const pipeline::PipelineRuntimeSnapshot* snapshot =
      pipeline::RestorePipelineSnapshot(state);
  EXPECT_NE(snapshot, nullptr);
  return snapshot == nullptr ? 0U : snapshot->completed_receive_cycles;
}

}  // namespace

TEST(EsrControllerRuntimeStateTest, CaptureAndRestoreRoundTripState) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  const session::EsrCycleInput first_input = MakeValidInput(1U);
  controller.RunOnce(first_input);
  ASSERT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kCompleted);
  const EsrControllerRuntimeState snapshot = controller.CaptureRuntimeState();

  const session::EsrCycleInput second_input = MakeValidInput(2U);
  controller.RunOnce(second_input);
  ASSERT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 2U);

  controller.RestoreRuntimeState(snapshot);
  EXPECT_TRUE(controller.HasLatestInterceptOutputFrame());
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 1U);
  EXPECT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kCompleted);
}

TEST(EsrControllerRuntimeStateTest, SuccessfulCyclesAdvanceBatchAndRejectedCycleDoesNot) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller(pipeline, env);
  const std::uint64_t initial_batch_id = controller.CaptureRuntimeState().next_batch_id;

  const session::EsrCycleInput valid_input = MakeValidInput(10U);
  controller.RunOnce(valid_input);
  ASSERT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kCompleted);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 10U);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().batch_id, initial_batch_id);
  EXPECT_EQ(controller.CaptureRuntimeState().next_batch_id, initial_batch_id + 1U);

  session::EsrCycleInput invalid_input = MakeValidInput(11U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);
  EXPECT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kRejected);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().batch_id, initial_batch_id);
  EXPECT_EQ(controller.CaptureRuntimeState().next_batch_id, initial_batch_id + 1U);

  const session::EsrCycleInput next_valid_input = MakeValidInput(12U);
  controller.RunOnce(next_valid_input);
  ASSERT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kCompleted);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 12U);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().batch_id, initial_batch_id + 1U);
  EXPECT_EQ(controller.CaptureRuntimeState().next_batch_id, initial_batch_id + 2U);
}

TEST(EsrControllerRuntimeStateTest, RestoreDoesNotMutatePipelineOwnedState) {
  EsrInternalExecutionConfig config = MakeDefaultConfig();
  config.mission.scan.scan_rate_hz = 0.25f;
  pipeline::InterceptPipeline pipeline(config);
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  session::EsrCycleInput first = MakeValidInput(20U);
  first.dt_sec = 0.2f;
  first.rf_emissions.window_duration_s = first.dt_sec;
  controller.RunOnce(first);
  const EsrControllerRuntimeState controller_state =
      controller.CaptureRuntimeState();
  const double first_phase = ReadScanPhase(pipeline);

  session::EsrCycleInput second = MakeValidInput(21U);
  second.dt_sec = 0.2f;
  second.rf_emissions.window_duration_s = second.dt_sec;
  controller.RunOnce(second);
  const double second_phase = ReadScanPhase(pipeline);
  ASSERT_NE(first_phase, second_phase);

  ASSERT_TRUE(controller.RestoreRuntimeState(controller_state));
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 20U);
  EXPECT_DOUBLE_EQ(ReadScanPhase(pipeline), second_phase);
}

TEST(EsrControllerRuntimeStateTest, RestoreRejectsSnapshotFromOtherControllerInstance) {
  pipeline::InterceptPipeline shared_pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller_a(shared_pipeline, env);
  EsrController controller_b(shared_pipeline, env);

  controller_a.RunOnce(MakeValidInput(21U));
  controller_b.RunOnce(MakeValidInput(31U));

  const EsrControllerRuntimeState foreign_state = controller_b.CaptureRuntimeState();
  controller_a.RestoreRuntimeState(foreign_state);

  EXPECT_EQ(controller_a.GetLatestInterceptOutputFrame().cycle_index, 21U);
}

// ESR controller 层在校验拒绝时保留上一有效输出帧（供内部 RF 状态机跨周期簿记），
// 但 session 层 BuildCycleResult 对非执行周期返回默认空帧（见
// esr_public_api_convenience_test::StepReturnsEmptyFrameOnValidationFailure）。
// 本测试验证 controller 内部保留行为——不等于公开输出被复用。
TEST(EsrControllerRuntimeStateTest, ValidationRejectSetsAbortReasonAndControllerRetainsPreviousOutput) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  const session::EsrCycleInput first_input = MakeValidInput(40U);
  controller.RunOnce(first_input);
  ASSERT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kCompleted);

  session::EsrCycleInput invalid_input = MakeValidInput(41U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);

  EXPECT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kRejected);
  EXPECT_EQ(controller.BuildCycleResult().abort_reason,
            session::EsrPipelineAbortReason::kValidationRejected);
  // controller 内部缓存保留上一帧（非公开复用语义）。
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 40U);
}

TEST(EsrControllerRuntimeStateTest, FirstValidationRejectDoesNotCreateOutputFrame) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  session::EsrCycleInput invalid_input = MakeValidInput(45U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);

  EXPECT_FALSE(controller.HasLatestInterceptOutputFrame());
  EXPECT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kRejected);
  EXPECT_EQ(controller.BuildCycleResult().abort_reason,
            session::EsrPipelineAbortReason::kValidationRejected);
}

TEST(EsrControllerRuntimeStateTest,
     SessionCaptureRestoreSkipsRollbackOnValidationRejection) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  const session::EsrCycleInput first_input = MakeValidInput(100U);
  controller.RunOnce(first_input);
  ASSERT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kCompleted);

  session::EsrCycleInput invalid_input = MakeValidInput(101U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);

  EXPECT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kRejected);
  EXPECT_EQ(controller.BuildCycleResult().abort_reason,
            session::EsrPipelineAbortReason::kValidationRejected);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 100U);
}

TEST(EsrControllerRuntimeStateTest, ScanPhaseUsesFullPatternCycleRateAndVariableStep) {
  EsrInternalExecutionConfig config = MakeDefaultConfig();
  config.mission.scan.scan_rate_hz = 0.5f;
  pipeline::InterceptPipeline pipeline(config);
  StubEnvironmentService env;

  session::EsrCycleInput input = MakeValidInput(1U);
  input.dt_sec = 0.2f;
  input.rf_emissions.window_duration_s = input.dt_sec;
  pipeline.RunCycle(input, env);
  EXPECT_NEAR(ReadScanPhase(pipeline), 0.1, 5.0e-8);

  config.mission.scan.scan_rate_hz = 2.0f;
  pipeline.UpdateConfig(config);
  EXPECT_NEAR(ReadScanPhase(pipeline), 0.1, 5.0e-8);
  input.dt_sec = 0.15f;
  input.rf_emissions.window_duration_s = input.dt_sec;
  pipeline.RunCycle(input, env);
  EXPECT_NEAR(ReadScanPhase(pipeline), 0.4, 5.0e-8);

  config.mission.scan.scan_rate_hz = 5.0f;
  pipeline.UpdateConfig(config);
  input.dt_sec = 0.25f;
  input.rf_emissions.window_duration_s = input.dt_sec;
  pipeline.RunCycle(input, env);
  EXPECT_NEAR(ReadScanPhase(pipeline), 0.65, 5.0e-8);
}

TEST(EsrControllerRuntimeStateTest, ScanGeometryResetPowerFreezeAndSnapshotRestore) {
  EsrInternalExecutionConfig config = MakeDefaultConfig();
  config.mission.scan.scan_rate_hz = 0.5f;
  pipeline::InterceptPipeline pipeline(config);
  StubEnvironmentService env;
  session::EsrCycleInput input = MakeValidInput(1U);
  input.dt_sec = 0.2f;
  input.rf_emissions.window_duration_s = input.dt_sec;
  pipeline.RunCycle(input, env);
  const extension::InterceptPipelineRuntimeState saved = pipeline.CaptureRuntimeState();
  EXPECT_NEAR(ReadScanPhase(pipeline), 0.1, 5.0e-8);

  config.resolved_scan.scan_sequence = 1;
  pipeline.UpdateConfig(config);
  EXPECT_DOUBLE_EQ(ReadScanPhase(pipeline), 0.0);

  pipeline.RunCycle(input, env);
  config.sensor_enabled = false;  // 断电冻结扫描相位（COMMON-OQ-4 字段提升）
  pipeline.UpdateConfig(config);
  const double frozen_phase = ReadScanPhase(pipeline);
  pipeline.RunCycle(input, env);
  EXPECT_DOUBLE_EQ(ReadScanPhase(pipeline), frozen_phase);

  EXPECT_TRUE(pipeline.RestoreRuntimeState(saved));
  EXPECT_NEAR(ReadScanPhase(pipeline), 0.1, 5.0e-8);
}

TEST(EsrControllerRuntimeStateTest, OutputFrameCarriesScanAzimuthAdvancingWithPhase) {
  // 扫描方位角 = 当前波束中心方位 + 天线安装角（平台系），随扫描相位逐周期推进。
  // 默认扫描图：az -60°→+60° 步进 5°（25 列）× el 10°→-10° 步进 5°（5 行），
  // serpentine 折返；rate 0.1 Hz、dt 1 s → 每周期相位推进 0.1。
  EsrInternalExecutionConfig config = MakeDefaultConfig();
  config.mission.scan.scan_rate_hz = 0.1f;
  pipeline::InterceptPipeline pipeline(config);
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  // 相位 0 → 索引 0 → 首波束（az -60°；安装角 0°）。
  const session::EsrCycleInput first_input = MakeValidInput(1U);
  controller.RunOnce(first_input);
  ASSERT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kCompleted);
  EXPECT_FLOAT_EQ(controller.GetLatestInterceptOutputFrame().scan_azimuth_deg, -60.0f);

  // 相位 0.1 → 索引 12 → az 0°（首行内推进）。
  const session::EsrCycleInput second_input = MakeValidInput(2U);
  controller.RunOnce(second_input);
  ASSERT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kCompleted);
  EXPECT_FLOAT_EQ(controller.GetLatestInterceptOutputFrame().scan_azimuth_deg, 0.0f);

  // 相位 0.2 → 索引 25 → 折返行（serpentine）行首 → az +60°。
  const session::EsrCycleInput third_input = MakeValidInput(3U);
  controller.RunOnce(third_input);
  ASSERT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kCompleted);
  EXPECT_FLOAT_EQ(controller.GetLatestInterceptOutputFrame().scan_azimuth_deg, 60.0f);
}

TEST(EsrControllerRuntimeStateTest, OutputFrameScanAzimuthAddsMountOffset) {
  // 扫描方位合成 = 波束中心方位 + 天线安装角。
  // 注：RF 前端 boresight 域校验要求 az+mount ∈ [-180, 180]（TryResolveBoresight），
  // 超界周期被拒绝，故已完成周期的合成值恒在域内，NormalizeAngle180 为恒等。
  // 扫描图：az 130°→150° 步进 20°（2 列）× el 5 行，serpentine；安装角 20°。
  // 首波束 az 130° + 20° = 150°；折返行行首 az 150° + 20° = 170°。
  EsrInternalExecutionConfig config = MakeDefaultConfig();
  config.resolved_scan.scan_start_az_deg = 130.0f;
  config.resolved_scan.scan_end_az_deg = 150.0f;
  config.resolved_scan.az_step_deg = 20.0f;
  config.orientation.antenna_mount_az_deg = 20.0f;
  config.mission.scan.scan_rate_hz = 0.5f;
  pipeline::InterceptPipeline pipeline(config);
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  // 相位 0 → 索引 0 → 首波束 az 130° + 安装角 20° = 150°。
  const session::EsrCycleInput first_input = MakeValidInput(1U);
  controller.RunOnce(first_input);
  ASSERT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kCompleted);
  EXPECT_FLOAT_EQ(controller.GetLatestInterceptOutputFrame().scan_azimuth_deg, 150.0f);

  // 相位 0.5 → 索引 5 → 折返行行首 az 150° + 安装角 20° = 170°。
  const session::EsrCycleInput second_input = MakeValidInput(2U);
  controller.RunOnce(second_input);
  ASSERT_EQ(controller.BuildCycleResult().status,
            session::EsrCycleExecutionStatus::kCompleted);
  EXPECT_FLOAT_EQ(controller.GetLatestInterceptOutputFrame().scan_azimuth_deg, 170.0f);
}

TEST(EsrControllerRuntimeStateTest, RestoreRejectsNonFiniteScanPhase) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  extension::InterceptPipelineRuntimeState state = pipeline.CaptureRuntimeState();
  const pipeline::PipelineRuntimeSnapshot* original =
      pipeline::RestorePipelineSnapshot(state);
  ASSERT_NE(original, nullptr);
  std::shared_ptr<pipeline::PipelineRuntimeSnapshot> corrupted(
      new pipeline::PipelineRuntimeSnapshot(*original));
  corrupted->scan_phase_cycles = std::numeric_limits<double>::quiet_NaN();
  pipeline::CapturePipelineSnapshot(state, corrupted);
  EXPECT_FALSE(pipeline.RestoreRuntimeState(state));
}

TEST(EsrControllerRuntimeStateTest,
     PipelineSnapshotRestoresCompletedTuningPhase) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  session::EsrCycleInput input = MakeValidInput(1U);

  ASSERT_FALSE(pipeline.RunCycle(input, env).rf_v2_rejected);
  ASSERT_EQ(ReadCompletedReceiveCycles(pipeline), 1U);
  const extension::InterceptPipelineRuntimeState saved =
      pipeline.CaptureRuntimeState();

  input.cycle_index = 2U;
  input.cycle_start_time_s = 2.0;
  input.rf_emissions.world_cycle_index = 2U;
  input.rf_emissions.window_start_time_s = 2.0;
  ASSERT_FALSE(pipeline.RunCycle(input, env).rf_v2_rejected);
  ASSERT_EQ(ReadCompletedReceiveCycles(pipeline), 2U);

  ASSERT_TRUE(pipeline.RestoreRuntimeState(saved));
  EXPECT_EQ(ReadCompletedReceiveCycles(pipeline), 1U);
}

}  // namespace extension
}  // namespace electronic_surveillance_radar
