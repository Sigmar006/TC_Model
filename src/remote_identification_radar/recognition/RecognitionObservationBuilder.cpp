/**
 * @file RecognitionObservationBuilder.cpp
 * @brief 识别观测构造器实现。
 */

/**
 * ## 过程：识别观测组装（LRDR 跟踪/识别链 · 第 5 步）
 * - 上游：生命周期航迹快照（状态/位置/速度/不确定度）、场景目标的各向 RCS 表/极化散
 *   射矩阵/散射中心表，以及观测上下文（视线角、SNR、带宽、斜距、驻留、拟合速度）。
 * - 本步：并行调用四个特征提取器组装 `RirFeatureSet`（RCS / 运动 / 极化 / 宽带一维像），
 *   再乘驻留质量因子 clamp(驻留/0.05 s, 0, 1)（只作用于量测类维度），最后生成 4 bit
 *   有效维度掩码（"有效且质量>0"才置位）。
 * - 下游：`RirTracker::UpdateCycle` 的滑动窗口积累与 `RirMatcher` 打分；逐维结果分别落
 *   验收第 44/45/46/47 项。
 * - 关键参数：`recognition_dwell_sec`（场景 0.05 s，对齐 `kNominalDwellSec`）、
 *   `minimum_aspect_coverage_deg`（库 profile 视角覆盖下限）、`max_range_resolution_m`
 *   （库 profile 分辨率上限）、特征权重见 `RirRecognitionFeatureWeights`。
 * - 对应验收项：第 47A 项 综合特征识别功能测试的观测入口（分项见 44/45/46/47 项）。
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

// 过程：有效维度掩码 = 4 bit（RCS/运动/极化/宽带）；"有效且质量>0"才置位，
//       质量 0 表示该维不参与打分（分母也不计）。
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
// 过程：因子 = clamp(驻留/0.05 s, 0, 1)（1 表示驻留达到标称值），非有限/非正回 0。
float DwellQualityFactor(float dwell_sec) {
  if (!std::isfinite(dwell_sec) || dwell_sec <= 0.0f) {
    return 0.0f;
  }
  return std::max(0.0f, std::min(1.0f, dwell_sec / kNominalDwellSec));
}

}  // namespace

// 过程：四维特征并行提取——RCS（视线角插值）、运动（滤波后验/多点拟合）、极化（双通
//       道能量）、宽带一维像（散射中心表），各提取器自带有效性门控。
RirFeatureSet RirObservationBuilder::Build(const session::RirSceneTarget& target,
                                           const tracking::RirTrackState& snapshot,
                                           const RirObservationContext& context) {
  RirFeatureSet set;
  set.rcs = RirRcsFeatureExtractor::Extract(target.aspect_rcs_samples, context.look_az_deg,
                                            context.look_el_deg, context.snr_db,
                                            context.minimum_aspect_coverage_deg,
                                            context.minimum_observation_snr_db);
  // 中译：把"量测位置多点拟合速度"透传给运动特征提取器（B3，2026-10-21）——拟合不可用时
  //       该标记为 false，提取器自动回退到滤波后验速度，行为与改动前一致。
  // 标识：B3 运动特征速度来源。
  const Eigen::Vector3f velocity_lsq(context.velocity_lsq_x_mps, context.velocity_lsq_y_mps,
                                    context.velocity_lsq_z_mps);
  const Eigen::Vector3f acceleration_lsq(context.acceleration_lsq_x_mps2,
                                        context.acceleration_lsq_y_mps2,
                                        context.acceleration_lsq_z_mps2);
  set.motion = RirMotionFeatureExtractor::Extract(snapshot, context.platform_altitude_m,
                                                  snapshot.EstimationUncertaintyTrace(),
                                                  context.has_velocity_lsq, velocity_lsq,
                                                  context.has_acceleration_lsq, acceleration_lsq);
  set.polarization = RirPolarizationFeatureExtractor::Extract(
      target.polarization_samples, context.look_az_deg, context.look_el_deg, context.snr_db,
      context.range_m, context.minimum_observation_snr_db);
  set.polarization_stats = RirPolarizationStatsExtractor::Extract(target.polarization_samples);
  set.range_profile =
      RirRangeProfileFeatureExtractor::Extract(target.range_rcs_scatterers, context.bandwidth_hz,
                                               context.snr_db, context.max_range_resolution_m);
  // 驻留时间质量因子：作用于除运动外的观测维度（运动来自滤波航迹，与驻留无关）。
  const float dwell_factor = DwellQualityFactor(context.dwell_sec);
  set.rcs.quality *= dwell_factor;
  set.polarization.quality *= dwell_factor;
  set.range_profile.quality *= dwell_factor;
  // 过程：维度掩码在质量缩放之后生成，保证"掩码置位 <=> 该维质量>0"的口径一致。
  set.valid_feature_mask = BuildFeatureMask(set);
  return set;
}

}  // namespace recognition
}  // namespace remote_identification_radar
