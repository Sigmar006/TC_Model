#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#if defined(__APPLE__)
#include <mach/mach.h>
#include <malloc/malloc.h>
#elif defined(__linux__)
#include <malloc.h>
#include <unistd.h>
#elif defined(_WIN32)
// NOMINMAX：windows.h 的 min/max 宏会污染 std::min/std::max 及库头中的使用。
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#pragma comment(lib, "psapi.lib")
#endif

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"

namespace {

namespace ar_config = airborne_radar::config;
namespace ar_session = airborne_radar::session;
namespace esr_config = electronic_surveillance_radar::config;
namespace esr_session = electronic_surveillance_radar::session;
namespace rf = oneq::electromagnetics;

constexpr std::size_t kRfEmissionCount = 64U;
constexpr std::size_t kArTargetCount = 1000U;
constexpr std::size_t kEsrEmitterCount = 1000U;
constexpr std::size_t kDenseMinimumProcessedCount =
    9U * kEsrEmitterCount / 10U;
constexpr std::size_t kDenseMaximumHypothesisCount = 5U * kEsrEmitterCount;
// Dense ESR association needs several max-missed windows before track creation
// and retirement reach steady state. Memory growth is measured only after that
// physical lifecycle has stabilized, while the measured interval remains 100
// full cycles.
constexpr std::uint32_t kWarmupCycles = 20U;
constexpr std::uint32_t kMeasuredCycles = 100U;
constexpr std::size_t kMemoryWindowCycles = 20U;
constexpr double kP95LimitMilliseconds = 100.0;
constexpr std::size_t kMaximumPostWarmupAllocatedHeapGrowthBytes = 4U * 1024U * 1024U;

std::size_t ReadResidentBytes() {
#if defined(__APPLE__)
  mach_task_basic_info_data_t task_info_data{};
  mach_msg_type_number_t task_info_count = MACH_TASK_BASIC_INFO_COUNT;
  const kern_return_t status =
      task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
                reinterpret_cast<task_info_t>(&task_info_data),
                &task_info_count);
  if (status != KERN_SUCCESS) {
    return 0U;
  }
  return static_cast<std::size_t>(task_info_data.resident_size);
#elif defined(__linux__)
  std::ifstream statm("/proc/self/statm");
  std::size_t total_pages = 0U;
  std::size_t resident_pages = 0U;
  if (!(statm >> total_pages >> resident_pages)) {
    return 0U;
  }
  const long page_size = sysconf(_SC_PAGESIZE);
  if (page_size <= 0) {
    return 0U;
  }
  return resident_pages * static_cast<std::size_t>(page_size);
#elif defined(_WIN32)
  // WorkingSetSize 对两处读取一致使用：PrivateUsage 需 _WIN32_WINNT>=0x0501 版本宏，
  // 且本测试关注的是工作集增长趋势而非精确堆统计。
  PROCESS_MEMORY_COUNTERS counters{};
  if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)) != 0) {
    return static_cast<std::size_t>(counters.WorkingSetSize);
  }
  return 0U;
#else
  return 0U;
#endif
}

std::size_t ReadAllocatedHeapBytes() {
#if defined(__APPLE__)
  malloc_statistics_t statistics{};
  malloc_zone_statistics(malloc_default_zone(), &statistics);
  return statistics.size_in_use;
#elif defined(__linux__)
  const struct mallinfo2 statistics = mallinfo2();
  return static_cast<std::size_t>(statistics.uordblks);
#elif defined(_WIN32)
  PROCESS_MEMORY_COUNTERS counters{};
  if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)) != 0) {
    return static_cast<std::size_t>(counters.WorkingSetSize);
  }
  return 0U;
#else
  return 0U;
#endif
}

std::size_t ComputeMedianBytes(std::vector<std::size_t> samples) {
  std::sort(samples.begin(), samples.end());
  return samples[samples.size() / 2U];
}

rf::RfEmissionFrame MakeRfEmissions(std::uint32_t cycle_index,
                                    double cycle_start_time_s,
                                    std::size_t emission_count,
                                    std::uint64_t identity_base) {
  rf::RfEmissionFrame frame;
  frame.world_cycle_index = cycle_index;
  frame.window_start_time_s = cycle_start_time_s;
  frame.window_duration_s = 1.0;
  std::vector<rf::RfSceneEmission>& emissions = frame.emissions;
  emissions.reserve(emission_count);
  for (std::size_t i = 0U; i < emission_count; ++i) {
    rf::RfSceneEmission emission;
    emission.identity.sensor_entity_id = static_cast<std::uint32_t>(identity_base + i);
    emission.identity.emission_id = identity_base + 100000U + i;
    emission.position_ecef_m.x_m = 6378137.0;
    emission.position_ecef_m.y_m = 20000.0 + 100.0 * static_cast<double>(i);
    emission.position_ecef_m.z_m = 500.0 * static_cast<double>(i % 8U);
    emission.antenna.boresight_ecef.x = 0.0;
    emission.antenna.boresight_ecef.y = -1.0;
    emission.antenna.peak_gain_dbi = 10.0;
    emission.polarization = rf::RfScenePolarization::kHorizontal;
    EXPECT_TRUE(rf::TryCreateRfNoiseWaveform(
        cycle_start_time_s, frame.window_duration_s,
        9.3e9 + 1.0e6 * static_cast<double>(i % 16U), 5.0e6, 100.0,
        &emission.waveform));
    emissions.push_back(emission);
  }
  return frame;
}

rf::RfEmissionFrame MakeDetectableEsrEmissions(std::uint32_t cycle_index,
                                                double cycle_start_time_s) {
  rf::RfEmissionFrame frame;
  frame.world_cycle_index = cycle_index;
  frame.window_start_time_s = cycle_start_time_s;
  frame.window_duration_s = 1.0;
  frame.emissions.reserve(kEsrEmitterCount);
  for (std::size_t i = 0U; i < kEsrEmitterCount; ++i) {
    rf::RfSceneEmission emission;
    emission.identity.sensor_entity_id = static_cast<std::uint32_t>(50000U + i);
    emission.identity.emission_id = 150000U + i;
    emission.position_ecef_m.x_m = 6378137.0;
    emission.position_ecef_m.y_m = 20000.0 + 10.0 * static_cast<double>(i);
    emission.position_ecef_m.z_m = 10.0 * static_cast<double>(i % 8U);
    emission.antenna.boresight_ecef.x = 0.0;
    emission.antenna.boresight_ecef.y = -1.0;
    emission.antenna.peak_gain_dbi = 10.0;
    EXPECT_TRUE(rf::TryCreateRfNoiseWaveform(
        cycle_start_time_s, frame.window_duration_s,
        9.0e9 + 1.0e6 * static_cast<double>(i), 0.25e6, 1.0e6,
        &emission.waveform));
    frame.emissions.push_back(emission);
  }
  return frame;
}

double ComputeP95Milliseconds(std::vector<double>* elapsed_ms) {
  std::sort(elapsed_ms->begin(), elapsed_ms->end());
  const std::size_t p95_index =
      (95U * elapsed_ms->size() + 99U) / 100U - 1U;
  return (*elapsed_ms)[p95_index];
}

ar_session::ArCycleInput MakeArInput() {
  ar_session::ArCycleInput input;
  input.dt_sec = 1.0;
  input.platform.platform_position_ecef_m.x_m = 6378137.0;
  input.targets.reserve(kArTargetCount);
  oneq::coordinate::LlaPositionDegM anchor_lla;
  oneq::coordinate::TryEcefToLla(input.platform.platform_position_ecef_m, &anchor_lla);
  for (std::size_t i = 0U; i < kArTargetCount; ++i) {
    oneq::coordinate::ExternalKinematics kinematics;
    kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    kinematics.position_ecef_m.x_m =
        6378137.0 + 10000.0 + 50.0 * static_cast<double>(i);
    kinematics.position_ecef_m.y_m =
        10.0 * static_cast<double>(static_cast<int>(i % 21U) - 10);
    kinematics.position_ecef_m.z_m =
        5.0 * static_cast<double>(static_cast<int>(i % 11U) - 5);

    oneq::coordinate::EnuSceneState enu;
    oneq::coordinate::TryMakeEnuSceneState(kinematics, anchor_lla, &enu);

    ar_session::ArTargetInput target;
    target.target_id = i + 1U;
    target.position_x = static_cast<float>(enu.position_enu_m.east_m);
    target.position_y = static_cast<float>(enu.position_enu_m.north_m);
    target.position_z = static_cast<float>(enu.position_enu_m.up_m);
    target.velocity_x = static_cast<float>(enu.velocity_enu_mps.east_mps);
    target.velocity_y = static_cast<float>(enu.velocity_enu_mps.north_mps);
    target.velocity_z = static_cast<float>(enu.velocity_enu_mps.up_mps);
    target.rcs = 10.0f;
    input.targets.push_back(target);
  }
  return input;
}

esr_session::EsrCycleInput MakeEsrInput() {
  esr_session::EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_position_ecef_m.x_m = 6378137.0;
  return input;
}

TEST(RfInterferencePerformanceTest,
     SparseDetectionFullScaleRfFrontEndMeetsReleaseP95Budget) {
  const rf::RfEmissionFrame initial_ar_frame =
      MakeRfEmissions(1U, 0.0, kRfEmissionCount, 20000U);
  const rf::RfEmissionFrame initial_esr_frame =
      MakeRfEmissions(1U, 0.0, kEsrEmitterCount, 40000U);
  ASSERT_EQ(initial_ar_frame.emissions.size(), kRfEmissionCount);
  ASSERT_EQ(initial_esr_frame.emissions.size(), kEsrEmitterCount);

  ar_config::ArSessionConfig ar_config;
  ar_config.hardware = ar_config::profiles::kLongRangeHighPowerHardware;
  ar_config.hardware.receiver.maximum_linear_input_power_w = 1.0e9f;
  ar_session::ArSession ar = ar_session::ArSession::Create(ar_config);

  esr_config::EsrSessionConfig esr_config;
  esr_config.hardware.beam_az_width_deg = 120.0f;
  esr_config.hardware.beam_el_width_deg = 120.0f;
  esr_config.hardware.maximum_linear_input_power_w = 1.0e9f;
  esr_config.policy.detection.enable_statistical_detection = false;
  esr_session::EsrSession esr = esr_session::EsrSession::Create(esr_config);

  ar_session::ArCycleInput ar_input = MakeArInput();
  esr_session::EsrCycleInput esr_input = MakeEsrInput();
  std::vector<double> elapsed_ms;
  elapsed_ms.reserve(kMeasuredCycles);

  for (std::uint32_t cycle = 1U; cycle <= kWarmupCycles + kMeasuredCycles; ++cycle) {
    ar_input.cycle_index = cycle;
    ar_input.cycle_start_time_s = static_cast<double>(cycle - 1U);
    ar_input.interference =
        MakeRfEmissions(cycle, ar_input.cycle_start_time_s, kRfEmissionCount,
                        20000U);
    esr_input.cycle_index = cycle;
    esr_input.cycle_start_time_s = ar_input.cycle_start_time_s;
    esr_input.rf_emissions =
        MakeRfEmissions(cycle, esr_input.cycle_start_time_s, kEsrEmitterCount,
                        40000U);
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    const ar_session::ArCycleResult ar_result = ar.StepWithResult(ar_input);
    const esr_session::EsrCycleResult esr_result = esr.StepWithResult(esr_input);
    const double duration_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    ASSERT_FALSE(ar_session::HasValidationError(ar_result.issues));
    ASSERT_EQ(ar_result.status, ar_session::ArCycleStatus::kCompleted);
    ASSERT_FALSE(esr_session::HasValidationError(esr_result.issues));
    ASSERT_EQ(esr_result.status, esr_session::EsrCycleExecutionStatus::kCompleted);
    if (cycle > kWarmupCycles) {
      elapsed_ms.push_back(duration_ms);
    }
  }

  const double p95_ms = ComputeP95Milliseconds(&elapsed_ms);
  RecordProperty("rf_emission_count", static_cast<int>(kRfEmissionCount));
  RecordProperty("ar_target_count", static_cast<int>(kArTargetCount));
  RecordProperty("esr_emitter_count", static_cast<int>(kEsrEmitterCount));
  RecordProperty("measured_cycle_count", static_cast<int>(kMeasuredCycles));
  RecordProperty("p95_milliseconds", std::to_string(p95_ms));
  EXPECT_LT(p95_ms, kP95LimitMilliseconds);
}

TEST(RfInterferencePerformanceTest,
     DenseDetectionFullScaleEsrPipelineMeetsReleaseP95Budget) {
  esr_config::EsrSessionConfig config;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 120.0f;
  config.hardware.maximum_linear_input_power_w = 1.0e9f;
  config.policy.detection.minimum_snr_db = -20.0f;
  config.policy.detection.enable_statistical_detection = false;
  config.hardware.tuning_plan.push_back(
      esr_config::EsrTuningWindow{9.5e9, 1.1e9, 1U});
  esr_session::EsrSession esr = esr_session::EsrSession::Create(config);
  esr_session::EsrCycleInput input = MakeEsrInput();
  std::vector<double> elapsed_ms;
  elapsed_ms.reserve(kMeasuredCycles);
  std::size_t minimum_observation_count = kEsrEmitterCount;
  std::size_t minimum_cluster_count = kEsrEmitterCount;
  std::size_t minimum_hypothesis_count = kEsrEmitterCount;
  std::size_t maximum_hypothesis_count = 0U;
  std::vector<std::size_t> resident_samples;
  resident_samples.reserve(kMeasuredCycles);
  std::vector<std::size_t> allocated_heap_samples;
  allocated_heap_samples.reserve(kMeasuredCycles);

  for (std::uint32_t cycle = 1U;
       cycle <= kWarmupCycles + kMeasuredCycles; ++cycle) {
    input.cycle_index = cycle;
    input.cycle_start_time_s = static_cast<double>(cycle - 1U);
    input.rf_emissions =
        MakeDetectableEsrEmissions(cycle, input.cycle_start_time_s);
    const std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();
    const esr_session::EsrCycleResult result = esr.StepWithResult(input);
    const double duration_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start)
            .count();
    ASSERT_EQ(result.status, esr_session::EsrCycleExecutionStatus::kCompleted);
    ASSERT_EQ(result.output_frame.observation_output.raw_observation_count,
              kEsrEmitterCount);
    ASSERT_GE(result.output_frame.observation_output.observations.size(),
              kDenseMinimumProcessedCount);
    ASSERT_GE(result.output_frame.observation_output.cluster_count,
              kDenseMinimumProcessedCount);
    ASSERT_GE(result.output_frame.emitter_output.hypotheses.size(),
              kDenseMinimumProcessedCount);
    ASSERT_LE(result.output_frame.emitter_output.hypotheses.size(),
              kDenseMaximumHypothesisCount);
    minimum_observation_count =
        std::min(minimum_observation_count,
                 result.output_frame.observation_output.observations.size());
    minimum_cluster_count =
        std::min(minimum_cluster_count,
                 result.output_frame.observation_output.cluster_count);
    minimum_hypothesis_count =
        std::min(minimum_hypothesis_count,
                 result.output_frame.emitter_output.hypotheses.size());
    maximum_hypothesis_count =
        std::max(maximum_hypothesis_count,
                 result.output_frame.emitter_output.hypotheses.size());
    if (cycle > kWarmupCycles) {
      elapsed_ms.push_back(duration_ms);
      const std::size_t resident_bytes = ReadResidentBytes();
      ASSERT_GT(resident_bytes, 0U);
      resident_samples.push_back(resident_bytes);
      const std::size_t allocated_heap_bytes = ReadAllocatedHeapBytes();
      ASSERT_GT(allocated_heap_bytes, 0U);
      allocated_heap_samples.push_back(allocated_heap_bytes);
    }
  }

  const double p95_ms = ComputeP95Milliseconds(&elapsed_ms);
  ASSERT_EQ(resident_samples.size(), kMeasuredCycles);
  const std::vector<std::size_t> first_resident_window(
      resident_samples.begin(), resident_samples.begin() + kMemoryWindowCycles);
  const std::vector<std::size_t> last_resident_window(resident_samples.end() - kMemoryWindowCycles,
                                                      resident_samples.end());
  const std::size_t first_window_median_bytes =
      ComputeMedianBytes(first_resident_window);
  const std::size_t last_window_median_bytes =
      ComputeMedianBytes(last_resident_window);
  const std::size_t resident_growth_bytes =
      last_window_median_bytes > first_window_median_bytes
          ? last_window_median_bytes - first_window_median_bytes
          : 0U;
  ASSERT_EQ(allocated_heap_samples.size(), kMeasuredCycles);
  const std::vector<std::size_t> first_allocated_heap_window(
      allocated_heap_samples.begin(), allocated_heap_samples.begin() + kMemoryWindowCycles);
  const std::vector<std::size_t> last_allocated_heap_window(
      allocated_heap_samples.end() - kMemoryWindowCycles, allocated_heap_samples.end());
  const std::size_t first_allocated_heap_window_median_bytes =
      ComputeMedianBytes(first_allocated_heap_window);
  const std::size_t last_allocated_heap_window_median_bytes =
      ComputeMedianBytes(last_allocated_heap_window);
  const std::size_t allocated_heap_growth_bytes =
      last_allocated_heap_window_median_bytes > first_allocated_heap_window_median_bytes
          ? last_allocated_heap_window_median_bytes - first_allocated_heap_window_median_bytes
          : 0U;
  RecordProperty("esr_emitter_count", static_cast<int>(kEsrEmitterCount));
  RecordProperty("raw_observation_count_per_cycle",
                 static_cast<int>(kEsrEmitterCount));
  RecordProperty("minimum_preprocessed_observation_count",
                 static_cast<int>(minimum_observation_count));
  RecordProperty("minimum_cluster_count",
                 static_cast<int>(minimum_cluster_count));
  RecordProperty("minimum_hypothesis_count",
                 static_cast<int>(minimum_hypothesis_count));
  RecordProperty("maximum_hypothesis_count",
                 static_cast<int>(maximum_hypothesis_count));
  RecordProperty("measured_cycle_count", static_cast<int>(kMeasuredCycles));
  RecordProperty("p95_milliseconds", std::to_string(p95_ms));
  RecordProperty("first_resident_window_median_bytes",
                 static_cast<double>(first_window_median_bytes));
  RecordProperty("last_resident_window_median_bytes",
                 static_cast<double>(last_window_median_bytes));
  RecordProperty("post_warmup_resident_growth_bytes",
                 static_cast<double>(resident_growth_bytes));
  RecordProperty("first_allocated_heap_window_median_bytes",
                 static_cast<double>(first_allocated_heap_window_median_bytes));
  RecordProperty("last_allocated_heap_window_median_bytes",
                 static_cast<double>(last_allocated_heap_window_median_bytes));
  RecordProperty("post_warmup_allocated_heap_growth_bytes",
                 static_cast<double>(allocated_heap_growth_bytes));
  EXPECT_LT(p95_ms, kP95LimitMilliseconds);
  // Resident pages are retained as a diagnostic because demand paging and
  // shared-library code faults can move RSS without retaining model state.
  // Active heap allocation directly checks whether the steady-state pipeline
  // keeps an unbounded amount of cycle-owned data.
  EXPECT_LE(allocated_heap_growth_bytes, kMaximumPostWarmupAllocatedHeapGrowthBytes);
}

}  // namespace
