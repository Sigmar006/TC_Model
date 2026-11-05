/**
 * ## 过程：安装矩阵合成链（ECI ↔ 传感器系）（红外处理链 · 第 2 步）
 * - 上游：卫星姿态（`input.satellite_attitude_eci_body_deg`，Body→ECI 的 Z-Y-X 欧拉角）、
 *   传感器安装角（`orientation.mount_body_sensor_deg`）、安装失准角
 *   （`orientation.misalignment`：常值偏置 + 运行期一次抽取的随机微扰，由 pipeline 合成）。
 * - 本步：把三个旋转合成一条固定矩阵，只做正解/反解：
 *   R_sensor→ECI = R_ECI_body(姿态) · R_body_sensor(安装)⁻¹ · R_sensor_misalign(失准)⁻¹；
 *   正解由传感器系 az/el 合成 ECI 单位光轴，反解把 ECI 视线折成传感器系 az/el；
 *   另有"期望 ECI 光轴 → 应施加的传感器系指向"的逆解与机械限位钳制。
 * - 下游：视场门（WFOV 矩形门、NFOV 捕获门）与指向指令都在传感器系判定；扫描/窄场指向经正解
 *   变回 ECI 驱动舵机与残差诊断。
 * - 关键参数：`orientation.sensor_scan_limits_deg`（机械限位）、`mount_body_sensor_deg`、
 *   `misalignment.bias_deg`/`random_sigma_deg`/`random_seed`（默认零失准 ⇒ 链退化为姿态旋转）。
 * - 对应验收项：第 10 项 安装矩阵误差功能测试；第 9 项 红外载荷角误差（指向与量测的参考系
 *   一致性）；第 74 项 红外场景几何与视场门控交互接口测试（传感器系门输入角）。
 */

#include "sbirs_sensor/pipeline/SbirsBoresightChain.h"

namespace sbirs_sensor {
namespace pipeline {

namespace {

// 过程：类型桥接 —— session 的 (yaw, pitch, roll) 按同序搬到坐标域类型，数值与语义均不变。
oneq::coordinate::EulerAnglesDeg ToCoordinateEuler(const session::SbirsEulerAnglesDeg& angles) {
  return oneq::coordinate::EulerAnglesDeg(angles.yaw_deg, angles.pitch_deg, angles.roll_deg);
}

// 过程：三分量容器互转（session::SbirsVector3M ↔ coordinate::Vector3d），只换类型不换参考系。
oneq::coordinate::Vector3d ToVector3d(const session::SbirsVector3M& vector) {
  return oneq::coordinate::Vector3d(vector.x, vector.y, vector.z);
}

session::SbirsVector3M ToSbirsVector(const oneq::coordinate::Vector3d& vector) {
  return session::SbirsVector3M(vector.x, vector.y, vector.z);
}

}  // namespace

// 过程：两参构造把安装失准置为零 —— 失准是可选叠加项，缺省时链退化为姿态 × 安装角⁻¹。
SbirsBoresightChain::SbirsBoresightChain(const session::SbirsEulerAnglesDeg& attitude_eci_body_deg,
                                         const oneq::coordinate::EulerAnglesDeg& mount_angles_deg)
    : SbirsBoresightChain(attitude_eci_body_deg, mount_angles_deg,
                          oneq::coordinate::EulerAnglesDeg()) {}

SbirsBoresightChain::SbirsBoresightChain(const session::SbirsEulerAnglesDeg& attitude_eci_body_deg,
                                         const oneq::coordinate::EulerAnglesDeg& mount_angles_deg,
                                         const oneq::coordinate::EulerAnglesDeg& misalignment_deg)
    : chain_(ToCoordinateEuler(attitude_eci_body_deg), mount_angles_deg, misalignment_deg) {}

// 过程：零姿态 + 零安装角 + 零失准 ⇒ 传感器系与 ECI 重合，此时传感器系角与 ECI 角逐位相同。
bool SbirsBoresightChain::IsIdentity() const { return chain_.IsIdentity(); }

// 过程：正变换 —— 传感器系分量按 R = R_ECI_body·R_body_sensor⁻¹·R_misalign⁻¹ 旋到 ECI；
//       安装角与失准取逆，是因为它们定义的方向分别是本体→传感器、实际→理想。
session::SbirsVector3M SbirsBoresightChain::RotateSensorToEci(
    const session::SbirsVector3M& sensor_los) const {
  return ToSbirsVector(chain_.RotateSensorToReference(ToVector3d(sensor_los)));
}

// 过程：逆变换 —— 用同一条链的转置把 ECI 分量旋回传感器系（正交阵的逆即转置）。
session::SbirsVector3M SbirsBoresightChain::RotateEciToSensor(
    const session::SbirsVector3M& eci_vector) const {
  return ToSbirsVector(chain_.RotateReferenceToSensor(ToVector3d(eci_vector)));
}

// 过程：正解 —— 传感器系 az/el 先合成单位向量 (cos el·cos az, cos el·sin az, sin el)，
//       再经链旋成 ECI 光轴，作为指向指令下发给舵机。
session::SbirsVector3M SbirsBoresightChain::EciLosOfSensorPointing(
    float sensor_azimuth_deg, float sensor_elevation_deg) const {
  return ToSbirsVector(
      chain_.ReferenceLosOfSensorPointing(sensor_azimuth_deg, sensor_elevation_deg));
}

// 过程：反解 —— ECI 方向先旋到传感器系，再取 az=atan2(y,x)、el=asin(z/|v|)；视场门与
//       机械限位都在这个参考系里判定（输出给融合层的角仍是 ECI 角）。
void SbirsBoresightChain::SensorAzElOfEciVector(const session::SbirsVector3M& eci_los,
                                                float* azimuth_deg,
                                                float* elevation_deg) const {
  chain_.SensorAzElOfReferenceVector(ToVector3d(eci_los), azimuth_deg, elevation_deg);
}

// 过程：逆解 —— 给定期望 ECI 光轴，反算出应施加的传感器系 az/el（惯性稳定指向的实现方式）。
void SbirsBoresightChain::SensorPointingForDesiredEciLos(
    const session::SbirsVector3M& desired_eci_los, float* azimuth_deg,
    float* elevation_deg) const {
  chain_.SensorPointingForDesiredReferenceLos(ToVector3d(desired_eci_los), azimuth_deg,
                                              elevation_deg);
}

// 过程：机械限位钳制 —— 命令角超出 [az_min, az_max]×[el_min, el_max] 时停在边界；
//       够不到目标的差额由剩余角/指向残差诊断体现，不在此处报错。
void SbirsBoresightChain::ClampToScanLimits(const config::SbirsScanLimitsDeg& limits,
                                            float* azimuth_deg, float* elevation_deg) {
  oneq::common::geometry::AzimuthElevationLimitsDeg common_limits;
  common_limits.az_min_deg = limits.az_min_deg;
  common_limits.az_max_deg = limits.az_max_deg;
  common_limits.el_min_deg = limits.el_min_deg;
  common_limits.el_max_deg = limits.el_max_deg;
  oneq::common::geometry::BoresightChain::ClampToScanLimits(common_limits, azimuth_deg,
                                                            elevation_deg);
}

}  // namespace pipeline
}  // namespace sbirs_sensor
