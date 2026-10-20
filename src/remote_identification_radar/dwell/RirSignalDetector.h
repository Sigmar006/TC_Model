/**
 * @file RirSignalDetector.h
 * @brief RIR 统计级 CFAR 探测判决器（common StatisticalCfarDetector 薄适配）。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_SIGNAL_DETECTOR_H_
#define REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_SIGNAL_DETECTOR_H_

#include <limits>

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "common/radar/StatisticalCfarDetector.h"
#include "remote_identification_radar/dwell/RirDetectionCellResolver.h"
#include "remote_identification_radar/internal/RirRadarEquations.h"

namespace remote_identification_radar {
namespace dwell {

/** @brief 探测策略参数（副本：AR DetectionPolicy）。 */
struct RirDetectionPolicyConfig {
  float cfar_pfa{1e-6f};
  float min_snr_db{-10.0f};
};

/** @brief 检测器工程配置。 */
struct RirDetectorConfig {
  config::hardware::RirTransmitterConfig transmitter{};
  config::hardware::RirAntennaConfig antenna{};
  config::hardware::RirReceiverConfig receiver{};
  RirDetectionPolicyConfig detection_policy{};
  float min_detection_margin_db{-2.0f};
  int pulse_count{10};
};

/** @brief 单目标探测结果。 */
struct RirDetectionResult {
  float echo_power_dbw{-300.0f};
  float snr_db{-100.0f};
  float detection_prob{0.0f};
  bool detected{false};
};

/** @brief 目标回波特征上下文。 */
struct RirTargetReturn {
  float rcs_m2{0.0f};
  float range_m{0.0f};
  internal::RirSwerlingModel swerling_type{internal::RirSwerlingModel::kSwerling0};
};

/** @brief 环境噪声上下文。 */
struct RirEnvironmentNoise {
  float propagation_loss_db{0.0f};
  float clutter_noise_w{0.0f};
  float jam_noise_w{0.0f};
};

/**
 * @brief RirSignalDetector：回波功率走 RirRadarEquations，判决走 common CFAR。
 * @note 6 dB 真值回退门不在本类；留在 RirController。
 */
class RirSignalDetector {
 public:
  explicit RirSignalDetector(const RirDetectorConfig& config);
  void UpdateConfig(const RirDetectorConfig& config);
  RirDetectionResult Detect(const RirTargetReturn& target, const RirEnvironmentNoise& env,
                            float one_way_antenna_gain_db = std::numeric_limits<float>::quiet_NaN(),
                            int pulse_count = 1);
  RirDetectionResult DetectResolvedCell(const RirTargetReturn& target,
                                        const RirDetectionCellResult& cell);
  void SetRandomSeed(unsigned int seed);

  /**
   * @brief 累计进入判决流程的次数（每次调用最多消耗一次 CFAR 抽签）。
   *
   * @details 2026-10-20 诊断用：与"随机种子被重设次数"一起，可在宿主诊断日志里判定
   *   CFAR 抽签是否真的在推进（抽签冻结时该计数照常增长，但判决结果恒定）。
   */
  std::uint64_t DecisionCount() const { return decision_count_; }

  /**
   * @brief 累计判为"未检测"（`detected == false`）的次数（2026-10-20 诊断用）。
   *
   * @details 与 `DecisionCount()` 配对：若判决次数在增长而本计数恒 0，说明抽签值始终
   *   小于检测概率（发生器未推进），而不是"日志只记了通过项"。
   */
  std::uint64_t FalseDecisionCount() const { return false_decision_count_; }

 private:
  RirDetectorConfig config_;
  oneq::common::radar::StatisticalCfarDetector cfar_;
  std::uint64_t decision_count_{0U};
  std::uint64_t false_decision_count_{0U};
};

}  // namespace dwell
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_SIGNAL_DETECTOR_H_
