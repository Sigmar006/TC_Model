// Copyright 2026. All Rights Reserved.
//
// @file rir_recognition_ballistic_scenario_test.cpp
// @brief 弹道目标四类识别效能验证（RIR 独立模块版，交付库直载）。
//
// 交付库只包含 BALLISTIC 一大类四型号（弹头 WARHEAD / 诱饵 DECOY / 残骸 DEBRIS /
// 碎片 FRAGMENT）——LRDR 面向弹道中段目标，具体弹型不可分辨。本用例以交付库
// 端到端验证：型号正确率、大类恒为弹道目标、未知目标拒绝。
//
// 场景几何按弹道中段真实量级设置：目标横距 3.0e6 m、高度 1.15e6~1.2e6 m
// （仰角约 21°、斜距约 3.2e6 m），远场窄波束（4°）+ 长脉冲（16 ms）保证
// SNR 足够成航迹；雷达作用距离放宽到 9.0e6 m（同 rir_ballistic_discrimination 场景）。

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "1q/remote_identification_radar/session/RirSession.h"
#include "RirCycleInputTestUtil.h"
#include "RirSqliteTestUtil.h"
#include "remote_identification_radar/recognition/RecognitionFeatureDatabase.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using recognition::RirFeatureDatabase;
using session::RirCycleInput;
using session::RirCycleResult;
using session::RirCycleStatus;
using session::RirRecognitionCategory;
using session::RirRecognitionState;
using session::RirSceneTarget;
using session::RirSession;

#ifndef ONEQ_RIR_EXAMPLE_DATABASE_PATH
#error "ONEQ_RIR_EXAMPLE_DATABASE_PATH 未定义（Integration.cmake 注入）"
#endif

// 目标横距（m）：与高度合成约 21° 仰角，落在扫描体积内。
constexpr float kTargetGroundRangeM = 3.0e6f;

struct ScenarioOutcome {
  std::uint32_t observed_cycles = 0U;
  std::uint32_t confirmed_cycles = 0U;
  std::uint32_t correct_cycles = 0U;
  std::uint32_t category_correct_cycles = 0U;
  float last_best_score = 0.0f;
};

class RirBallisticRecognitionScenarioTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 交付库可加载是全部场景的前提；失败即快速暴露（错误含表/字段上下文）。
    RirFeatureDatabase database;
    std::string error;
    ASSERT_TRUE(RirFeatureDatabase::Load(ONEQ_RIR_EXAMPLE_DATABASE_PATH, &database, &error))
        << error;
  }

  config::RirSessionConfig MakeScenarioConfig() const {
    config::RirSessionConfig cfg;
    cfg.mission.work_mode = config::RirWorkMode::kIdentify;
    cfg.policy.lifecycle.confirm_hits = 1U;
    // 弹道中段作用距离（同 rir_ballistic_discrimination 场景）。
    cfg.mission.max_range_m = 9.0e6f;
    cfg.mission.recognition_dwell_sec = 0.05f;
    // 远场窄波束：0.4° 波束表 + 长脉冲积累（16 ms × 4.5 MHz ≈ 48.6 dB 脉压增益），
    // 保证 3.2e6 m 斜距下弹道目标 SNR 足够越过 6 dB 准入回退门。
    cfg.hardware.antenna.nominal_az_beamwidth_deg = 4.0f;
    cfg.hardware.antenna.nominal_el_beamwidth_deg = 4.0f;
    cfg.hardware.antenna.main_beam_gain_db = 52.0f;
    cfg.hardware.transmitter.peak_power_w = 1.0e6f;
    cfg.hardware.transmitter.pulse_width_s = 16.0e-3f;
    cfg.hardware.transmitter.prf_hz = 10.0f;
    // 扫描扇区收窄到目标仰角带（21°±10°）与方位 ±20°，减小波位表规模，
    // 让目标在少数周期内被照亮。
    cfg.orientation.az_min_deg = -20.0f;
    cfg.orientation.az_max_deg = 20.0f;
    cfg.orientation.el_min_deg = 10.0f;
    cfg.orientation.el_max_deg = 30.0f;
    cfg.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
    cfg.policy.recognition.enabled = true;
    cfg.policy.recognition.database_path = ONEQ_RIR_EXAMPLE_DATABASE_PATH;
    cfg.policy.recognition.min_confirmed_hits = 1U;
    cfg.policy.recognition.min_observation_count = 1U;
    cfg.policy.recognition.acceptance_score = 0.6f;
    cfg.policy.recognition.minimum_margin = 0.05f;
    cfg.policy.recognition.accumulation_window_sec = 10.0f;
    cfg.policy.recognition.result_hold_sec = 1.0f;
    return cfg;
  }

  /** @brief 构造标注目标：速度/高度/视角 RCS 真值 + 真值名 + 已确认航迹供给。 */
  void AppendTarget(RirCycleInput* input, std::uint64_t target_id, float speed_mps,
                    float altitude_offset_m, float rcs_dbsm, const char* truth_name) {
    RirSceneTarget target;
    target.external_target_id = target_id;
    target.position_x =
        kTargetGroundRangeM +
        speed_mps * static_cast<float>(input->input_cycle_index - 1U) *
            static_cast<float>(input->dt_sec);
    target.position_z = altitude_offset_m;
    target.rcs = 5.0f;
    // 视角网格（方位 ±10°、俯仰 60°~90°）：覆盖近天顶视线，跨距 ≥ 库的 15° 覆盖下限。
    for (float az = -10.0f; az <= 10.0f; az += 5.0f) {
      for (float el = 60.0f; el <= 90.0f; el += 5.0f) {
        session::RirAspectRcsSample aspect;
        aspect.aspect_az_deg = az;
        aspect.aspect_el_deg = el;
        aspect.rcs_dbsm = rcs_dbsm;
        target.aspect_rcs_samples.push_back(aspect);
      }
    }
    target.target_name = truth_name;
    target.velocity_x = speed_mps;
    target.target_swerling_type = session::RirSwerlingType::kSwerling0;
    input->scene_targets.push_back(target);
  }

  RirCycleInput MakeInput(std::uint32_t cycle) {
    RirCycleInput input;
    input.input_cycle_index = cycle;
    input.dt_sec = 1.0;
    input.sim_time_sec = static_cast<float>(cycle - 1U);
    SetDefaultTestPlatformEcef(&input);
    return input;
  }

  /** @brief 模板绝对高度 → 雷达局部 ENU 上向分量（平台海拔 1000 m）。 */
  float AltitudeOffsetFor(float template_altitude_m) const {
    return template_altitude_m - 1000.0f;
  }

  /** @brief 跑单目标场景并汇总确认率/正确率/末周期最佳分数。 */
  ScenarioOutcome RunSingleTargetScenario(float speed_mps, float altitude_offset_m, float rcs_dbsm,
                                          const char* truth_name) {
    RirSession radar = RirSession::Create(MakeScenarioConfig());
    ScenarioOutcome outcome;
    const std::uint32_t kCycles = 10U;
    for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
      RirCycleInput input = MakeInput(cycle);
      AppendTarget(&input, 77U, speed_mps, altitude_offset_m, rcs_dbsm, truth_name);
      const RirCycleResult result = radar.StepWithResult(input);
      EXPECT_EQ(result.status, RirCycleStatus::kCompleted);
      if (result.output_frame.recognition_outputs.size() != 1U) {
        continue;
      }
      const auto& output = result.output_frame.recognition_outputs.front();
      ++outcome.observed_cycles;
      if (output.result.state == RirRecognitionState::kCategoryConfirmed ||
          output.result.state == RirRecognitionState::kModelConfirmed) {
        ++outcome.confirmed_cycles;
        if (output.result.target_model == truth_name) {
          ++outcome.correct_cycles;
        }
        if (output.result.target_category == RirRecognitionCategory::kBallistic) {
          ++outcome.category_correct_cycles;
        }
      }
      outcome.last_best_score = output.result.best_score;
    }
    return outcome;
  }
};

// S1 弹头：RCS -12 dBsm、速度 4400 m/s、高度 1.15e6 m。
TEST_F(RirBallisticRecognitionScenarioTest, WarheadScenarioRecognizesWarhead) {
  const ScenarioOutcome outcome = RunSingleTargetScenario(
      4400.0f, AltitudeOffsetFor(1150000.0f), -12.0f, "WARHEAD");
  EXPECT_GE(outcome.observed_cycles, 8U);
  EXPECT_GE(outcome.confirmed_cycles, outcome.observed_cycles * 6U / 10U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
}

// S2 诱饵：RCS 0 dBsm、速度 4350 m/s、高度 1.15e6 m。
TEST_F(RirBallisticRecognitionScenarioTest, DecoyScenarioRecognizesDecoy) {
  const ScenarioOutcome outcome =
      RunSingleTargetScenario(4350.0f, AltitudeOffsetFor(1150000.0f), 0.0f, "DECOY");
  EXPECT_GE(outcome.observed_cycles, 8U);
  EXPECT_GE(outcome.confirmed_cycles, outcome.observed_cycles * 6U / 10U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
}

// S3 残骸：RCS 6 dBsm、速度 4200 m/s、高度 1.2e6 m。
TEST_F(RirBallisticRecognitionScenarioTest, DebrisScenarioRecognizesDebris) {
  const ScenarioOutcome outcome =
      RunSingleTargetScenario(4200.0f, AltitudeOffsetFor(1200000.0f), 6.0f, "DEBRIS");
  EXPECT_GE(outcome.observed_cycles, 8U);
  EXPECT_GE(outcome.confirmed_cycles, outcome.observed_cycles * 6U / 10U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
}

// S4 碎片：RCS -22 dBsm、速度 4100 m/s、高度 1.2e6 m。
TEST_F(RirBallisticRecognitionScenarioTest, FragmentScenarioRecognizesFragment) {
  const ScenarioOutcome outcome =
      RunSingleTargetScenario(4100.0f, AltitudeOffsetFor(1200000.0f), -22.0f, "FRAGMENT");
  EXPECT_GE(outcome.observed_cycles, 8U);
  EXPECT_GE(outcome.confirmed_cycles, outcome.observed_cycles * 6U / 10U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
}

// S5 未知目标拒绝：几何同在搜索扇区内（高度 1.15e6 m、横距 3.0e6 m），但运动/尺度
// 特征偏离全部弹道模板（RCS +20 dBsm、速度 20 m/s），不得确认任何型号。
TEST_F(RirBallisticRecognitionScenarioTest, UnknownTargetRejectsRecognition) {
  RirSession radar = RirSession::Create(MakeScenarioConfig());
  std::uint32_t observed_cycles = 0U;
  std::uint32_t confirmed_cycles = 0U;
  const std::uint32_t kCycles = 10U;
  for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
    RirCycleInput input = MakeInput(cycle);
    AppendTarget(&input, 99U, 20.0f, AltitudeOffsetFor(1150000.0f), 20.0f, "UNKNOWN_TARGET");
    const RirCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
    if (result.output_frame.recognition_outputs.size() != 1U) {
      continue;
    }
    ++observed_cycles;
    const auto& output = result.output_frame.recognition_outputs.front();
    if (output.result.state == RirRecognitionState::kCategoryConfirmed ||
        output.result.state == RirRecognitionState::kModelConfirmed) {
      ++confirmed_cycles;
    }
  }
  EXPECT_GE(observed_cycles, 8U);
  EXPECT_LE(confirmed_cycles, observed_cycles / 10U)
      << "未知目标不应被确认到弹道目标库中的任何型号";
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
