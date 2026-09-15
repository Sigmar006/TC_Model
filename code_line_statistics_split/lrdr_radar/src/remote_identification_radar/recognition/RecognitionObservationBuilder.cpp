/**
 * @file RecognitionObservationBuilder.cpp
 * @brief 识别观测构造器实现。
 */

#include "remote_identification_radar/recognition/RecognitionObservationBuilder.h"

#include <algorithm>
#include <cmath>

#include "remote_identification_radar/recognition/MotionFeatureExtractor.h"
#include "remote_identification_radar/recognition/PolarizationFeatureExtractor.h"
#include "remote_identification_radar/recognition/PolarizationStatsExtractor.h"
#include "remote_identification_radar/recognition/RangeProfileFeatureExtractor.h"
#include "remote_identification_radar/recognition/RcsFeatureExtractor.h"

namespace remote_identification_radar {
namespace recognition {

namespace {

/** @brief 驻留时间质量标称值（s）：识别配置默认 recognition_dwell_sec。 */
constexpr float kNominalDwellSec = 0.05f;

std::uint8_t BuildFeatureMask(const RirFeatureSet& set) {
  std::uint8_t mask = 0U;
  if (set.rcs.valid && set.rcs.quality > 0.0f) {
    mask |= static_cast<std::uint8_t>(session::RirRecognitionFeatureDimension::kRcs);
  }
  if (set.motion.valid && set.motion.quality > 0.0f) {
    mask |= static_cast<std::uint8_t>(session::RirRecognitionFeatureDimension::kMotion);
  }
  if (set.polarization.valid && set.polarization.quality > 0.0f) {
    mask |= static_cast<std::uint8_t>(session::RirRecognitionFeatureDimension::kPolarization);
  }
  if (set.range_profile.valid && set.range_profile.quality > 0.0f) {
    mask |= static_cast<std::uint8_t>(session::RirRecognitionFeatureDimension::kRangeProfile);
  }
  return mask;
}

/** @brief 驻留质量因子：驻留越短，观测质量越低（效能级模型）。 */
float DwellQualityFactor(float dwell_sec) {
  if (!std::isfinite(dwell_sec) || dwell_sec <= 0.0f) {
    return 0.0f;
  }
  return std::max(0.0f, std::min(1.0f, dwell_sec / kNominalDwellSec));
}

}  // namespace

RirFeatureSet RirObservationBuilder::Build(const session::RirSceneTarget& target,
                                           const tracking::RirTrackState& snapshot,
                                           const RirObservationContext& context) {
  RirFeatureSet set;
  set.rcs = RirRcsFeatureExtractor::Extract(target.aspect_rcs_samples, context.look_az_deg,
                                            context.look_el_deg, context.snr_db,
                                            context.minimum_aspect_coverage_deg);
  set.motion = RirMotionFeatureExtractor::Extract(snapshot, context.platform_altitude_m,
                                                  snapshot.EstimationUncertaintyTrace());
  set.polarization = RirPolarizationFeatureExtractor::Extract(
      target.polarization_samples, context.look_az_deg, context.look_el_deg, context.snr_db,
      context.range_m);
  set.polarization_stats = RirPolarizationStatsExtractor::Extract(target.polarization_samples);
  set.range_profile =
      RirRangeProfileFeatureExtractor::Extract(target.range_rcs_scatterers, context.bandwidth_hz,
                                               context.snr_db, context.max_range_resolution_m);
  // 驻留时间质量因子：作用于除运动外的观测维度（运动来自滤波航迹，与驻留无关）。
  const float dwell_factor = DwellQualityFactor(context.dwell_sec);
  set.rcs.quality *= dwell_factor;
  set.polarization.quality *= dwell_factor;
  set.range_profile.quality *= dwell_factor;
  set.valid_feature_mask = BuildFeatureMask(set);
  return set;
}

}  // namespace recognition
}  // namespace remote_identification_radar
