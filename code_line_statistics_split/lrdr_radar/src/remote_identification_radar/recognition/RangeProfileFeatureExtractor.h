/**
 * @file RangeProfileFeatureExtractor.h
 * @brief 宽带一维距离像特征提取器（识别专用）。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RANGE_PROFILE_FEATURE_EXTRACTOR_H_
#define REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RANGE_PROFILE_FEATURE_EXTRACTOR_H_

#include <vector>

#include "1q/remote_identification_radar/session/RirSceneTypes.h"
#include "remote_identification_radar/recognition/RecognitionTypes.h"

namespace remote_identification_radar {
namespace recognition {

/**
 * @brief RirRangeProfileFeatureExtractor 从距离向散射列表构造效能化距离像观测。
 *
 * 距离分辨率 c/(2B) 决定距离单元宽度；散射中心按相对距离投影到单元并按
 * 线性散射功率叠加（提供相位时相干叠加）。峰值以散射中心级判定（超过由
 * SNR 决定的噪声门限），目标长度为首末有效峰跨距。分辨率不满足数据库
 * 要求时维度整体无效。
 */
class RirRangeProfileFeatureExtractor {
 public:
  /**
   * @brief 提取距离像特征观测。
   * @param[in] scatterers 距离向散射中心列表。
   * @param[in] bandwidth_hz 有效带宽（Hz），>0。
   * @param[in] snr_db 周期信噪比（dB）。
   * @param[in] max_range_resolution_m 允许的最大距离分辨率（m）；0 表示不限。
   * @return 距离像观测；散射列表为空或分辨率不达标时 valid=false。
   */
  static RirRangeProfileObservation Extract(
      const std::vector<session::RirRangeRcsScatterer>& scatterers, float bandwidth_hz, float snr_db,
      float max_range_resolution_m);
};

}  // namespace recognition
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RANGE_PROFILE_FEATURE_EXTRACTOR_H_
