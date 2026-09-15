/**
 * @file MotionFeatureExtractor.cpp
 * @brief 运动特征提取器实现。
 */

#include "remote_identification_radar/recognition/MotionFeatureExtractor.h"

#include <algorithm>
#include <cmath>

namespace remote_identification_radar {
namespace recognition {

namespace {

float Clamp01(float value) { return std::max(0.0f, std::min(1.0f, value)); }

}  // namespace

RirMotionObservation RirMotionFeatureExtractor::Extract(const tracking::RirTrackState& snapshot,
                                                        float platform_altitude_m,
                                                        float estimation_uncertainty_trace) {
  RirMotionObservation observation;
  if (snapshot.status != tracking::RirTrackStatus::kConfirmed) {
    return observation;  // 识别仅以已确认航迹为对象；未确认仅积累不输出
  }

  observation.valid = true;
  observation.speed_mps = snapshot.speed;
  observation.acceleration_mps2 = snapshot.acceleration_mps2;
  observation.altitude_m = platform_altitude_m + snapshot.position.z();

  // 横向加速度分解：a_lat = |a - (a·v/|v|²)v|；低速或横向加速度低于阈值记为直线飞行。
  const float speed_sq = snapshot.velocity.squaredNorm();
  if (speed_sq > 1.0e-4f) {
    const float v_dot_a = snapshot.velocity.dot(snapshot.acceleration);
    const Eigen::Vector3f projection = v_dot_a / speed_sq * snapshot.velocity;
    const Eigen::Vector3f lateral = snapshot.acceleration - projection;
    const float lateral_x = lateral.x();
    const float lateral_y = lateral.y();
    const float lateral_z = lateral.z();
    const float lateral_acceleration =
        std::sqrt(lateral_x * lateral_x + lateral_y * lateral_y + lateral_z * lateral_z);
    if (lateral_acceleration < 0.01f) {
      observation.is_straight = true;
    } else {
      observation.is_straight = false;
      observation.turn_radius_m = snapshot.speed * snapshot.speed / lateral_acceleration;
    }
  } else {
    observation.is_straight = true;
  }

  // 质量因子：估计不确定度越小质量越高（D2：P 的 position 分块迹作本源信号）。
  // 迹按三轴取均值归一化到每轴方差，与参考方差 (100 m)² 量纲口径一致。
  const float per_axis_variance = std::max(0.0f, estimation_uncertainty_trace) / 3.0f;
  observation.quality = Clamp01(10000.0f / (10000.0f + per_axis_variance));
  return observation;
}

}  // namespace recognition
}  // namespace remote_identification_radar
