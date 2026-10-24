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

/**
 * @brief 速度估计收敛门限（单轴速度方差，m²/s²，2026-10-16）。
 *
 * 为什么需要：加速度取后验速度差分，而滤波重置（建轨/失跟重捕）以零速均值 + 速度无知
 * 先验（方差 9e6）重初始化，首个后验速度直接跳到真值量级，该拍差分基准却是重置前的旧
 * 速度（≈0），使 `a = Δv/Δt` 数值上约等于速度本身——BJ 集成实测加速度与速度逐分量
 * 完全相同（模长 5 640 m/s²），而真实加速度只有约 5 m/s²。
 *
 * 该伪值经一阶指数平滑（τ=30 周期）只是缓慢衰减，仍会把累积窗的中位数整体抬高，
 * 令识别"运动"维对正确型号的相似度恒为 0（模板 7.5±3.0，z 达数百）。
 *
 * 取 1e5 m²/s²（σ≈316 m/s）：远低于重置后的无知先验 9e6（σ=3000 m/s），
 * 又高于稳定跟踪后的数值，足以把"刚重置"的伪加速度整段挡掉。
 */
constexpr float kMaxVelocityVarianceForAcceleration = 1.0e5f;

}  // namespace

RirMotionObservation RirMotionFeatureExtractor::Extract(const tracking::RirTrackState& snapshot,
                                                        float platform_altitude_m,
                                                        float estimation_uncertainty_trace,
                                                        bool has_velocity_lsq,
                                                        const Eigen::Vector3f& velocity_lsq_mps,
                                                        bool has_acceleration_lsq,
                                                        const Eigen::Vector3f& acceleration_lsq_mps2) {
  RirMotionObservation observation;
  if (snapshot.status != tracking::RirTrackStatus::kConfirmed) {
    return observation;  // 识别仅以已确认航迹为对象；未确认仅积累不输出
  }

  observation.valid = true;
  // 标量模长兜底（2026-10-15 实测）：快照的 speed / acceleration_mps2 标量字段在实跑中
  // 恒为 0（验收行长期打印 速度=0.000m/s 加速度=0.000m/s²，而同一快照的位置与**速度向量**
  // 均正常——下面横向加速度分解用的就是 vector 字段且工作正常）。标量为 0 时改用向量模长，
  // 使"运动"分项恢复判别力；若向量同样为 0（未估计出速度），行为与本改动前完全一致。
  // 中译：速度优先取**量测多点拟合**结果（B3，2026-10-21）。滤波后验速度在该几何下
  //       只有约 1.9 km/s（真值约 6 km/s），把弹道族模板全部推到 4σ 之外；拟合速度
  //       （N=20 拍，误差约 200 m/s）才具备型号分辨力。拟合不可用时行为与改动前一致。
  // 标识：B3 运动特征速度来源。
  observation.speed_from_fit = has_velocity_lsq;
  observation.speed_mps = has_velocity_lsq
                              ? velocity_lsq_mps.norm()
                              : ((snapshot.speed > 0.0f) ? snapshot.speed
                                                         : snapshot.velocity.norm());
  observation.acceleration_mps2 = (snapshot.acceleration_mps2 > 0.0f)
                                      ? snapshot.acceleration_mps2
                                      : snapshot.acceleration.norm();
  observation.altitude_m = platform_altitude_m + snapshot.position.z();

  // 加速度维门控（2026-10-16）：速度尚未收敛时，差分加速度是"重置伪值"而非物理量
  // （见 kMaxVelocityVarianceForAcceleration 注释）。此时**将加速度置零**而不是压低质量：
  // 置零后该维对所有型号模板给出同一相似度（对 7.5±3.0 约为 0.04），既不再反向拖累
  // 正确型号，也不给错误型号额外好处；运动维退化为"速度 + 高度 + 常量"，仍具判别力。
  // 速度/高度两维来自位置滤波，协方差已收敛，不受该伪值影响，故照常保留。
  const bool acceleration_trustworthy =
      snapshot.MaxVelocityVariance() > 0.0f &&
      snapshot.MaxVelocityVariance() <= kMaxVelocityVarianceForAcceleration;
  // 中译：**加速度来源优先级**（2026-10-22）——先用"逐拍拟合速度的斜率"（物理量正确，
  //   能还原约 5 m/s² 的重力分量）；不可用时退回滤波后验速度差分（幅值仅真值 1/3）。
  //   两条通路都要求量级合理（1–50 m/s²），越界视为不可信并按既有规则置零。
  // 标识：加速度来源优先级（2026-10-22）。
  if (has_acceleration_lsq) {
    const float magnitude = acceleration_lsq_mps2.norm();
    if (std::isfinite(magnitude) && magnitude >= 1.0f && magnitude <= 50.0f) {
      observation.acceleration_mps2 = magnitude;
      observation.acceleration_from_fit = true;
      observation.turn_radius_m = 0.0f;
    }
  }
  if (!acceleration_trustworthy) {
    observation.acceleration_mps2 = 0.0f;
    observation.acceleration_from_fit = false;
  }
  // 中译：最终钳制——物理上 4–10 m/s² 量级，任何超过 50 m/s² 的来源（重捕瞬态、退化几何）
  //   都不写入特征，避免一票否决运动维。
  // 标识：加速度量级钳制（2026-10-22）。
  if (!(observation.acceleration_mps2 <= 50.0f)) {
    observation.acceleration_mps2 = 0.0f;
    observation.acceleration_from_fit = false;
  }

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
