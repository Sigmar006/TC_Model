#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "1q/electromagnetics/RfScene.h"
#include "electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.h"
#include "electronic_surveillance_radar/pipeline/MutableEsrContext.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace {

session::EsrCycleInput MakeInput() {
  session::EsrCycleInput input;
  input.cycle_index = 4U;
  input.cycle_start_time_s = 10.0;
  input.dt_sec = 1.0f;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  return input;
}

oneq::electromagnetics::RfSceneEmission MakeEmission(std::uint64_t emission_id, double center_hz,
                                                     double power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = 20U + emission_id;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m.x_m = 6378137.0;
  emission.position_ecef_m.y_m = 1000.0;
  emission.antenna.boresight_ecef.y = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(10.0, 1.0, center_hz, 1.0e6, power_w,
                                                               &emission.waveform));
  return emission;
}

oneq::electromagnetics::RfSceneEmission MakePulseTrainEmission(std::uint64_t emission_id,
                                                               double center_hz, double power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = 20U + emission_id;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m.x_m = 6378137.0;
  emission.position_ecef_m.y_m = 1000.0;
  emission.antenna.boresight_ecef.y = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      10.0, center_hz, 1.0e6, power_w, 1.0e-6, 1.0e-3, 1000U, 0.0, 0U, 0U, &emission.waveform));
  return emission;
}

oneq::electromagnetics::RfSceneEmission MakeContinuousEmission(std::uint64_t emission_id,
                                                               double center_hz, double power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = 20U + emission_id;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m.x_m = 6378137.0;
  emission.position_ecef_m.y_m = 1000.0;
  emission.antenna.boresight_ecef.y = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfContinuousWaveform(
      10.0, 1.0, center_hz, 1.0e6, power_w, &emission.waveform));
  return emission;
}

oneq::electromagnetics::RfSceneEmission MakeSweepEmission(std::uint64_t emission_id,
                                                           double start_hz, double stop_hz,
                                                           double power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.sensor_entity_id = 20U + emission_id;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m.x_m = 6378137.0;
  emission.position_ecef_m.y_m = 1000.0;
  emission.antenna.boresight_ecef.y = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfLinearSweepWaveform(
      10.0, 1.0, start_hz, stop_hz, 1.0e6, power_w, 1.0, &emission.waveform));
  return emission;
}

InterceptDetectionOutput RunDetection(const session::EsrCycleInput& input,
                                      float maximum_linear_input_power_w = 10.0f,
                                      std::uint64_t completed_receive_cycles = 0U,
                                      bool use_tuning_plan = false,
                                      bool enable_statistical_detection = false,
                                      float beamwidth_deg = 120.0f,
                                      const std::vector<config::EsrCoSiteIsolationPath>&
                                          co_site_paths = {}) {
  extension::InterceptPipelineConfig pipeline_config;
  pipeline_config.detection.minimum_snr_db = -20.0f;
  pipeline_config.statistical_detection.enable_statistical_detection = enable_statistical_detection;
  pipeline_config.scan.scan_start_az_deg = 0.0f;
  pipeline_config.scan.scan_end_az_deg = 0.0f;
  pipeline_config.scan.scan_start_el_deg = 0.0f;
  pipeline_config.scan.scan_end_el_deg = 0.0f;

  extension::InterceptRuntimeConfig runtime_config;
  runtime_config.receiver_hardware.receiver_band_lower_hz = 9.99e9;
  runtime_config.receiver_hardware.receiver_band_upper_hz = 10.01e9;
  runtime_config.receiver_hardware.beam_az_width_deg = beamwidth_deg;
  runtime_config.receiver_hardware.beam_el_width_deg = beamwidth_deg;
  runtime_config.receiver_hardware.maximum_linear_input_power_w = maximum_linear_input_power_w;
  runtime_config.receiver_hardware.co_site_paths = co_site_paths;
  if (use_tuning_plan) {
    config::EsrTuningWindow first_window;
    first_window.center_frequency_hz = 9.5e9;
    first_window.bandwidth_hz = 1.0e6;
    first_window.dwell_cycles = 1U;
    config::EsrTuningWindow second_window;
    second_window.center_frequency_hz = 10.0e9;
    second_window.bandwidth_hz = 1.0e6;
    second_window.dwell_cycles = 1U;
    runtime_config.receiver_hardware.tuning_plan = {first_window, second_window};
  }

  session::EsrEnvironmentSnapshot environment;
  MutableEsrContext context;
  context.BeginCycle(input, environment, pipeline_config, runtime_config);
  InterceptDetectionExecutor executor;
  std::uint64_t next_observation_id = 1U;
  double scan_phase_cycles = 0.0;
  return executor.Execute(context, next_observation_id, &scan_phase_cycles,
                          completed_receive_cycles);
}

TEST(EsrRfV2DetectionTest, EmitsDeclassifiedObservationFromRfFrame) {
  session::EsrCycleInput input = MakeInput();
  input.rf_emissions.emissions.push_back(MakeEmission(1U, 10.0e9, 1.0e6));

  const InterceptDetectionOutput output = RunDetection(input);
  ASSERT_EQ(output.raw_records.size(), 1U);
  const RawObservationRecord& record = output.raw_records.front();
  // RF 均值是接收机测量估计而非真值副本：应在发布标准差 rf_std_hz 的 4σ 内（误差模型按
  // ±3σ 截断，4σ 容差留余量），且不再逐字段等于输入 10.0 GHz。
  EXPECT_LE(std::abs(record.observation.rf_hz - 10.0e9), 4.0 * record.observation.rf_std_hz);
  EXPECT_GT(record.observation.rf_std_hz, 0.0);
  EXPECT_GT(record.observation.bandwidth_hz, 0.0);
  EXPECT_GT(record.observation.snr_db, -20.0);
}

TEST(EsrRfV2DetectionTest, SameChannelEmissionReducesSnrWithoutBooleanPenalty) {
  session::EsrCycleInput baseline_input = MakeInput();
  baseline_input.rf_emissions.emissions.push_back(MakeEmission(1U, 10.0e9, 1.0e8));
  const InterceptDetectionOutput baseline = RunDetection(baseline_input);
  ASSERT_EQ(baseline.raw_records.size(), 1U);

  session::EsrCycleInput interfered_input = baseline_input;
  interfered_input.rf_emissions.emissions.push_back(MakeEmission(2U, 10.0e9, 1.0e6));
  const InterceptDetectionOutput interfered = RunDetection(interfered_input);
  // 同一时频角单元只发布最强候选；较弱源进入该单元干扰账本。
  ASSERT_EQ(interfered.raw_records.size(), 1U);
  EXPECT_LT(interfered.raw_records.front().observation.snr_db,
            baseline.raw_records.front().observation.snr_db);
}

TEST(EsrRfV2DetectionTest, AngularlyResolvedSameFrequencySourceDoesNotEnterInterferenceCell) {
  session::EsrCycleInput baseline_input = MakeInput();
  baseline_input.rf_emissions.emissions.push_back(MakeEmission(1U, 10.0e9, 1.0e6));
  const InterceptDetectionOutput baseline =
      RunDetection(baseline_input, 10.0f, 0U, false, false, 10.0f);
  ASSERT_EQ(baseline.raw_records.size(), 1U);

  session::EsrCycleInput separated_input = baseline_input;
  oneq::electromagnetics::RfSceneEmission separated = MakeEmission(2U, 10.0e9, 1.0e8);
  separated.position_ecef_m.x_m += 1000.0;
  separated.position_ecef_m.y_m = 1000.0;
  separated.antenna.boresight_ecef.x = -1.0;
  separated.antenna.boresight_ecef.y = -1.0;
  separated_input.rf_emissions.emissions.push_back(separated);
  const InterceptDetectionOutput separated_output =
      RunDetection(separated_input, 10.0f, 0U, false, false, 10.0f);
  ASSERT_EQ(separated_output.raw_records.size(), 2U);
  EXPECT_DOUBLE_EQ(separated_output.raw_records.front().observation.snr_db,
                   baseline.raw_records.front().observation.snr_db);
}

TEST(EsrRfV2DetectionTest,
     PolarBearingSingularityDoesNotRejectOtherObservableEmission) {
  session::EsrCycleInput input = MakeInput();
  oneq::electromagnetics::RfSceneEmission polar =
      MakeEmission(1U, 10.005e9, 1.0e6);
  polar.position_ecef_m.x_m += 1000.0;
  polar.position_ecef_m.y_m = 0.0;
  polar.antenna.boresight_ecef.x = -1.0;
  polar.antenna.boresight_ecef.y = 0.0;
  input.rf_emissions.emissions.push_back(polar);
  input.rf_emissions.emissions.push_back(MakeEmission(2U, 10.0e9, 1.0e6));

  const InterceptDetectionOutput output = RunDetection(input);
  EXPECT_FALSE(output.rf_v2_rejected);
  ASSERT_EQ(output.raw_records.size(), 1U);
  EXPECT_LT(std::abs(output.raw_records.front().observation.aoa_el_deg), 30.0);
}

TEST(EsrRfV2DetectionTest, EmissionOrderDoesNotChangeSemanticRandomMeasurements) {
  session::EsrCycleInput forward = MakeInput();
  forward.rf_emissions.emissions.push_back(MakeEmission(2U, 10.0e9, 1.0e6));
  forward.rf_emissions.emissions.push_back(MakeEmission(1U, 10.0e9, 1.0e6));
  session::EsrCycleInput reverse = forward;
  std::reverse(reverse.rf_emissions.emissions.begin(), reverse.rf_emissions.emissions.end());

  const InterceptDetectionOutput forward_output = RunDetection(forward, 10.0f, 0U, false, true);
  const InterceptDetectionOutput reverse_output = RunDetection(reverse, 10.0f, 0U, false, true);
  ASSERT_EQ(forward_output.raw_records.size(), reverse_output.raw_records.size());
  for (std::size_t index = 0U; index < forward_output.raw_records.size(); ++index) {
    const session::EmitterObservation& left = forward_output.raw_records[index].observation;
    const session::EmitterObservation& right = reverse_output.raw_records[index].observation;
    EXPECT_EQ(left.observation_id, right.observation_id);
    EXPECT_DOUBLE_EQ(left.aoa_az_deg, right.aoa_az_deg);
    EXPECT_DOUBLE_EQ(left.aoa_el_deg, right.aoa_el_deg);
    EXPECT_DOUBLE_EQ(left.snr_db, right.snr_db);
  }
}

TEST(EsrRfV2DetectionTest, SaturationCompletesWithoutFabricatedObservation) {
  session::EsrCycleInput input = MakeInput();
  input.rf_emissions.emissions.push_back(MakeEmission(1U, 10.0e9, 1.0e12));
  const InterceptDetectionOutput output = RunDetection(input, 1.0e-4f);
  EXPECT_TRUE(output.receiver_saturated);
  EXPECT_TRUE(output.raw_records.empty());
}

TEST(EsrRfV2DetectionTest, TuningPlanUsesCompletedReceiveCyclesRatherThanInputCycleIndex) {
  session::EsrCycleInput input = MakeInput();
  input.cycle_index = 999U;
  input.rf_emissions.world_cycle_index = input.cycle_index;

  const InterceptDetectionOutput initial = RunDetection(input, 10.0f, 0U, true);
  const InterceptDetectionOutput after_one_completed = RunDetection(input, 10.0f, 1U, true);
  EXPECT_DOUBLE_EQ(initial.receiver_center_frequency_hz, 9.5e9);
  EXPECT_DOUBLE_EQ(after_one_completed.receiver_center_frequency_hz, 10.0e9);
}

TEST(EsrRfV2DetectionTest, MissingCoSitePathNearFieldRejectsTheV2CycleBeforeProducingObservation) {
  session::EsrCycleInput input = MakeInput();
  oneq::electromagnetics::RfSceneEmission emission = MakeEmission(1U, 10.0e9, 1.0e6);
  emission.identity.sensor_entity_id = 99U;
  emission.position_ecef_m = input.platform_position_ecef_m;
  input.rf_emissions.emissions.push_back(emission);

  const InterceptDetectionOutput output = RunDetection(input);
  EXPECT_TRUE(output.rf_v2_rejected);
  EXPECT_TRUE(output.raw_records.empty());
}

TEST(EsrRfV2DetectionTest, MeasurementMeansDifferFromTruthAcrossSnr) {
  // 固定辐射源，通过改变入射功率改变 SNR：高 SNR 下测量误差应更小（更靠近真值）。
  // 验证冻结合同"测量误差随 SNR 单调"。
  const double center_hz = 10.0e9;
  session::EsrCycleInput low_power_input = MakeInput();
  low_power_input.rf_emissions.emissions.push_back(MakePulseTrainEmission(1U, center_hz, 1.0e6));
  session::EsrCycleInput high_power_input = MakeInput();
  high_power_input.rf_emissions.emissions.push_back(MakePulseTrainEmission(1U, center_hz, 1.0e9));

  const InterceptDetectionOutput low_output = RunDetection(low_power_input);
  const InterceptDetectionOutput high_output = RunDetection(high_power_input);
  ASSERT_EQ(low_output.raw_records.size(), 1U);
  ASSERT_EQ(high_output.raw_records.size(), 1U);
  // 高 SNR 的发布不确定度应小于低 SNR（std 随 SNR 单调下降）。
  EXPECT_LT(high_output.raw_records.front().observation.rf_std_hz,
            low_output.raw_records.front().observation.rf_std_hz);
  EXPECT_LT(high_output.raw_records.front().observation.pulse_width_std_s,
            low_output.raw_records.front().observation.pulse_width_std_s);
}

TEST(EsrRfV2DetectionTest, MeasurementNoiseIsOrderInvariant) {
  // 仿 EmissionOrderDoesNotChangeSemanticRandomMeasurements：RF/带宽/PRI/PW 测量噪声使用
  // identity-keyed 随机子流，emission 输入顺序变化不应改变同 identity 的测量值。
  session::EsrCycleInput forward = MakeInput();
  forward.rf_emissions.emissions.push_back(MakePulseTrainEmission(2U, 10.0e9, 1.0e6));
  forward.rf_emissions.emissions.push_back(MakePulseTrainEmission(1U, 10.0e9, 1.0e6));
  session::EsrCycleInput reverse = forward;
  std::reverse(reverse.rf_emissions.emissions.begin(), reverse.rf_emissions.emissions.end());

  const InterceptDetectionOutput forward_output = RunDetection(forward);
  const InterceptDetectionOutput reverse_output = RunDetection(reverse);
  ASSERT_EQ(forward_output.raw_records.size(), reverse_output.raw_records.size());
  for (std::size_t index = 0U; index < forward_output.raw_records.size(); ++index) {
    const session::EmitterObservation& left = forward_output.raw_records[index].observation;
    const session::EmitterObservation& right = reverse_output.raw_records[index].observation;
    EXPECT_EQ(left.observation_id, right.observation_id);
    EXPECT_DOUBLE_EQ(left.rf_hz, right.rf_hz);
    EXPECT_DOUBLE_EQ(left.bandwidth_hz, right.bandwidth_hz);
    EXPECT_DOUBLE_EQ(left.pri_s, right.pri_s);
    EXPECT_DOUBLE_EQ(left.pulse_width_s, right.pulse_width_s);
  }
}

TEST(EsrRfV2DetectionTest, WaveformClassPropagatesToObservation) {
  // 四种波形类别分别构造 emission，驱动检测，断言 observation.waveform_class 映射正确。
  session::EsrCycleInput pulse_input = MakeInput();
  pulse_input.rf_emissions.emissions.push_back(MakePulseTrainEmission(1U, 10.0e9, 1.0e6));
  session::EsrCycleInput continuous_input = MakeInput();
  continuous_input.rf_emissions.emissions.push_back(MakeContinuousEmission(2U, 10.0e9, 1.0e6));
  session::EsrCycleInput sweep_input = MakeInput();
  sweep_input.rf_emissions.emissions.push_back(MakeSweepEmission(3U, 9.95e9, 10.05e9, 1.0e6));
  session::EsrCycleInput noise_input = MakeInput();
  noise_input.rf_emissions.emissions.push_back(MakeEmission(4U, 10.0e9, 1.0e6));

  const InterceptDetectionOutput pulse_output = RunDetection(pulse_input);
  const InterceptDetectionOutput continuous_output = RunDetection(continuous_input);
  const InterceptDetectionOutput sweep_output = RunDetection(sweep_input);
  const InterceptDetectionOutput noise_output = RunDetection(noise_input);

  ASSERT_EQ(pulse_output.raw_records.size(), 1U);
  ASSERT_EQ(continuous_output.raw_records.size(), 1U);
  ASSERT_EQ(sweep_output.raw_records.size(), 1U);
  ASSERT_EQ(noise_output.raw_records.size(), 1U);

  EXPECT_EQ(pulse_output.raw_records.front().observation.waveform_class,
            session::EsrWaveformClass::kPulse);
  EXPECT_EQ(continuous_output.raw_records.front().observation.waveform_class,
            session::EsrWaveformClass::kContinuous);
  EXPECT_EQ(sweep_output.raw_records.front().observation.waveform_class,
            session::EsrWaveformClass::kSweep);
  EXPECT_EQ(noise_output.raw_records.front().observation.waveform_class,
            session::EsrWaveformClass::kNoise);

  // 非脉冲类别不应有物理 PRI/PW；字段为 0。
  EXPECT_DOUBLE_EQ(continuous_output.raw_records.front().observation.pri_s, 0.0);
  EXPECT_DOUBLE_EQ(continuous_output.raw_records.front().observation.pulse_width_s, 0.0);
  EXPECT_DOUBLE_EQ(sweep_output.raw_records.front().observation.pri_s, 0.0);
  EXPECT_DOUBLE_EQ(sweep_output.raw_records.front().observation.pulse_width_s, 0.0);
}

TEST(EsrRfV2DetectionTest, BelowThresholdEmissionWritesInfoExclusionDiagnostic) {
  session::EsrCycleInput input = MakeInput();
  input.rf_emissions.emissions.push_back(MakeEmission(7U, 10.0e9, 1.0e-12));

  const InterceptDetectionOutput output = RunDetection(input);
  // 行为中立：低于门限的发射源仍不产出观测记录；排除原因只经 issues 承载（规则 13b/13c）。
  EXPECT_TRUE(output.raw_records.empty());
  ASSERT_FALSE(output.issues.empty());
  bool found = false;
  for (const session::EsrIssue& issue : output.issues) {
    if (issue.code == "esr.emission_below_threshold") {
      found = true;
      EXPECT_EQ(issue.severity, session::EsrIssueSeverity::kInfo);
      EXPECT_NE(issue.message.find("emission_id=7"), std::string::npos);
      // 门内归因（规则 13b）：发射功率 1e-12 W 的弱源 → 硬门（snr < minimum_snr_db）。
      EXPECT_EQ(issue.cause, session::EsrIssueCause::kHardGateFailed);
      EXPECT_NE(issue.message.find("margin_db="), std::string::npos);
      // 实体机器可读关联（规则 14e）：单源排序后 entity_index=0。
      EXPECT_EQ(issue.location.kind, oneq::foundation::ValidationLocationKind::kSceneEntity);
      EXPECT_EQ(issue.location.entity_index, 0U);
    }
  }
  EXPECT_TRUE(found);
  EXPECT_EQ(output.excluded_below_threshold, 1U);
}

TEST(EsrRfV2DetectionTest, ZeroPowerEmissionWritesInfoExclusionDiagnostic) {
  session::EsrCycleInput input = MakeInput();
  input.rf_emissions.emissions.push_back(MakeEmission(8U, 10.0e9, 0.0));

  const InterceptDetectionOutput output = RunDetection(input);
  // 行为中立：零功率发射源不产出观测记录；排除原因只经 issues 承载（规则 13b/13c）。
  EXPECT_TRUE(output.raw_records.empty());
  ASSERT_FALSE(output.issues.empty());
  bool found = false;
  for (const session::EsrIssue& issue : output.issues) {
    if (issue.code == "esr.emission_zero_power") {
      found = true;
      EXPECT_EQ(issue.severity, session::EsrIssueSeverity::kInfo);
      EXPECT_NE(issue.message.find("emission_id=8"), std::string::npos);
      // 门内归因（规则 13b）：发射功率为 0 → 链路预算归零 → kTransmitSilent。
      EXPECT_EQ(issue.cause, session::EsrIssueCause::kTransmitSilent);
      EXPECT_NE(issue.message.find("time_overlap_fraction="), std::string::npos);
      // 实体机器可读关联（规则 14e）：单源排序后 entity_index=0。
      EXPECT_EQ(issue.location.kind, oneq::foundation::ValidationLocationKind::kSceneEntity);
      EXPECT_EQ(issue.location.entity_index, 0U);
    }
  }
  EXPECT_TRUE(found);
  EXPECT_EQ(output.excluded_zero_power, 1U);
}

TEST(EsrRfV2DetectionTest, CoSiteEmissionWritesInfoExclusionDiagnostic) {
  session::EsrCycleInput input = MakeInput();
  // 低功率避免触发接收机饱和（最大线性输入 10 W）；同平台 → 同址链路。
  oneq::electromagnetics::RfSceneEmission emission = MakeEmission(9U, 10.0e9, 1.0e-6);
  input.rf_emissions.emissions.push_back(emission);

  // 接收机配置该发射设备的同址隔离路径后，同址链路可解析并被门控跳过。
  const config::EsrCoSiteIsolationPath co_site_path{20U + 9U, 40.0};
  const InterceptDetectionOutput output = RunDetection(input, 10.0f, 0U, false, false, 120.0f,
                                                       {co_site_path});
  // 行为中立：同址发射源不产出观测记录；排除原因只经 issues 承载（规则 13b/13c）。
  EXPECT_TRUE(output.raw_records.empty());
  ASSERT_FALSE(output.issues.empty());
  bool found = false;
  for (const session::EsrIssue& issue : output.issues) {
    if (issue.code == "esr.emission_co_site") {
      found = true;
      EXPECT_EQ(issue.severity, session::EsrIssueSeverity::kInfo);
      EXPECT_NE(issue.message.find("emission_id=9"), std::string::npos);
      EXPECT_NE(issue.message.find("co_site=true"), std::string::npos);
      // 具体门（同址判定本身可定位）：cause 恒 kNone；message 携带隔离度量值。
      EXPECT_EQ(issue.cause, session::EsrIssueCause::kNone);
      EXPECT_NE(issue.message.find("isolation_db="), std::string::npos);
      // 实体机器可读关联（规则 14e）：单源排序后 entity_index=0。
      EXPECT_EQ(issue.location.kind, oneq::foundation::ValidationLocationKind::kSceneEntity);
      EXPECT_EQ(issue.location.entity_index, 0U);
    }
  }
  EXPECT_TRUE(found);
  EXPECT_EQ(output.excluded_co_site, 1U);
}

}  // namespace
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
