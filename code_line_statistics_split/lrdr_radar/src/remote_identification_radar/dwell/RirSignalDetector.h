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

 private:
  RirDetectorConfig config_;
  oneq::common::radar::StatisticalCfarDetector cfar_;
};

}  // namespace dwell
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_SIGNAL_DETECTOR_H_
