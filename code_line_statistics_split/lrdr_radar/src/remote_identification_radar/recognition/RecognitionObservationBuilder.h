/**
 * @file RecognitionObservationBuilder.h
 * @brief 识别观测构造器（组合四类特征提取器）。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_OBSERVATION_BUILDER_H_
#define REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_OBSERVATION_BUILDER_H_

#include "1q/remote_identification_radar/session/RirSceneTypes.h"
#include "remote_identification_radar/recognition/RecognitionTypes.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace recognition {

/**
 * @brief RirObservationBuilder 从场景目标特征真值与航迹快照构造
 *        单周期四维特征观测集合。
 *
 * 目标特征列表仅由本构造器在效能约束（SNR、距离、带宽、驻留、视角覆盖）
 * 下转换为可识别观测；常规搜索/跟踪目标不提供特征列表时对应维度标记不可用，
 * 不以单值 rcs 或默认零值替代。
 */
class RirObservationBuilder {
 public:
  /**
   * @brief 构造单周期识别观测。
   * @param[in] target 场景目标（含可选识别特征真值列表）。
   * @param[in] snapshot 内部航迹状态（运动特征来源）。
   * @param[in] context 周期效能约束。
   * @return 四维特征观测集合与有效维度掩码。
   */
  static RirFeatureSet Build(const session::RirSceneTarget& target,
                             const tracking::RirTrackState& snapshot,
                             const RirObservationContext& context);
};

}  // namespace recognition
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_OBSERVATION_BUILDER_H_
