/**
 * ## 过程：场景坐标系转换 ECEF → ECI（红外处理链 · 第 1 步）
 * - 上游：`SbirsPipeline` 周期入口的 `input.scene`（session 契约层 ECEF 真值目标）与本周期
 *   GMST（由上送 UTC 儒略日经 `coordinate::TryComputeGmstRad` 得出）。
 * - 本步：只绕地轴做一次 GMST 旋转：位置 p_eci = R_z(gmst)·p_ecef；
 *   速度额外叠加地球自转输运项 v_eci = R_z(gmst)·(v_ecef + ω×r_ecef)；
 *   非几何字段（目标 ID/名称/辐射强度/有效性）与"是否有速度"标志原样透传。
 * - 下游：pipeline 内全部逐目标计算都在 ECI 中进行——相对视线、方位/俯仰、地球遮挡、
 *   路径透过率、SNR、EKF/IMM 状态与协方差、安装矩阵旋转链。
 * - 关键参数：本步无配置项；GMST 来自仿真时刻（会话输入 `utc_julian_day`）。
 * - 对应验收项：第 2 项 坐标系转换功能测试；第 5 项 探测角度计算功能测试（角度参考系）；
 *   第 74 项 红外场景几何与视场门控交互接口测试（卫星/目标 ECEF 与 ECI 对照）。
 */

#include "sbirs_sensor/pipeline/SbirsEciScene.h"

namespace sbirs_sensor {
namespace pipeline {

SbirsEciSceneTarget RotateSceneTargetToEci(const session::SbirsSceneTarget& source,
                                           double gmst_rad) {
  // 过程：先复制与参考系无关的元数据（ID/名称/辐射强度/有效性），本步只换参考系不改语义。
  SbirsEciSceneTarget rotated;
  rotated.target_id = source.target_id;
  rotated.target_name = source.target_name;
  rotated.radiant_intensity_w_per_sr = source.radiant_intensity_w_per_sr;
  rotated.active = source.active;
  rotated.has_velocity_eci_m_per_s = source.has_velocity_ecef_m_per_s;

  // 过程：位置是纯几何旋转，|p| 与地心距不变（正交阵不改变模长）；失败走下方防御回退。
  const oneq::coordinate::EcefPositionM ecef_position(
      source.position_ecef_m.x, source.position_ecef_m.y, source.position_ecef_m.z);
  oneq::coordinate::EciPositionM eci_position;
  if (oneq::coordinate::TryEcefToEci(ecef_position, gmst_rad, &eci_position)) {
    rotated.position_eci_m = session::SbirsVector3M{eci_position.x_m, eci_position.y_m,
                                                    eci_position.z_m};
  } else {
    // 输入校验已拒绝非有限分量；此防御路径保留未旋转值，不静默丢目标。
    rotated.position_eci_m = source.position_ecef_m;
  }
  // 过程：速度必须叠加 ω×r 输运项：ECEF 中静止的地面点（v=0）在 ECI 中仍随地球东向运动。
  if (source.has_velocity_ecef_m_per_s) {
    const oneq::coordinate::EcefVelocityMps ecef_velocity(
        source.velocity_ecef_m_per_s.x, source.velocity_ecef_m_per_s.y,
        source.velocity_ecef_m_per_s.z);
    oneq::coordinate::EciVelocityMps eci_velocity;
    if (oneq::coordinate::TryEcefVelocityToEci(ecef_position, ecef_velocity, gmst_rad,
                                               &eci_velocity)) {
      rotated.velocity_eci_m_per_s =
          session::SbirsVector3M{eci_velocity.x_mps, eci_velocity.y_mps,
                                 eci_velocity.z_mps};
    } else {
      rotated.velocity_eci_m_per_s = source.velocity_ecef_m_per_s;
    }
  }
  return rotated;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
