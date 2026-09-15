/**
 * @file RcsFeatureExtractor.h
 * @brief 各向 RCS 特征提取器（识别专用，dBsm 域）。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RCS_FEATURE_EXTRACTOR_H_
#define REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RCS_FEATURE_EXTRACTOR_H_

#include <vector>

#include "1q/remote_identification_radar/session/RirSceneTypes.h"
#include "remote_identification_radar/recognition/RecognitionTypes.h"

namespace remote_identification_radar {
namespace recognition {

/**
 * @brief RirRcsFeatureExtractor 从视角离散 RCS 真值表构造效能化 RCS 观测。
 *
 * 输入为目标特征真值列表（dBsm），输出经视线角插值、视角覆盖判定与
 * SNR 决定的量测不确定度扰动后的观测。观测均值无偏（不引入随机偏置），
 * 不确定度以 std_db 表达并进入质量因子。
 */
class RirRcsFeatureExtractor {
 public:
  /**
   * @brief 提取 RCS 特征观测。
   * @param[in] samples 视角离散 RCS 样本列表（真值）。
   * @param[in] look_az_deg 视线方位角（目标参考系，deg）。
   * @param[in] look_el_deg 视线俯仰角（目标参考系，deg）。
   * @param[in] snr_db 周期信噪比（dB）。
   * @param[in] minimum_aspect_coverage_deg 视角覆盖下限（deg）。
   * @return RCS 观测；样本为空或视线角超出覆盖时 valid=false。
   */
  static RirRcsObservation Extract(const std::vector<session::RirAspectRcsSample>& samples,
                                float look_az_deg, float look_el_deg, float snr_db,
                                float minimum_aspect_coverage_deg);
};

}  // namespace recognition
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RCS_FEATURE_EXTRACTOR_H_
