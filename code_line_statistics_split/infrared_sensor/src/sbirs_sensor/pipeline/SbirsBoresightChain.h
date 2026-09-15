/**
 * @file SbirsBoresightChain.h
 * @brief 传感器指向合成链：卫星姿态（Body->ECI）+ 安装角（Body->Sensor）+ 扫描指向。
 * @note 纯几何引擎已提取到公共域（common/geometry/BoresightChain，参考系无关）；
 *       本类是 SBIRS 会话类型（SbirsVector3M/SbirsEulerAnglesDeg）与公共引擎
 *       （coordinate::Vector3d）之间的薄适配层，语义与提取前逐位一致。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_BORESIGHT_CHAIN_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_BORESIGHT_CHAIN_H_

#include "1q/sbirs_sensor/config/SbirsOrientationConfig.h"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"
#include "common/geometry/BoresightChain.h"

namespace sbirs_sensor {
namespace pipeline {

/**
 * @brief 传感器指向合成链（对齐 AR 的 platform_attitude + mount_angles + scan_center 链路）。
 * @details 基准组合关系：actual_boresight = attitude(Body->ECI) ∘ mount(Body->Sensor)
 *          ∘ misalignment⁻¹ ∘ scan(传感器系)。传感器系方位/俯仰定义：单位指向向量 v
 *          （传感器系）由 az/el 按 v = (cos(el)·cos(az), cos(el)·sin(az), sin(el)) 构造
 *          （与 SbirsGeometry 的 ECI 约定相同，仅参考系不同）。零姿态 + 零安装角 +
 *          零失准下链路为恒等变换（历史行为逐位不变）。
 * @note 稳定方式语义（SbirsStabilizationMode）：
 *       - kBodyStabilized：扫描参数（scan_start_az/span/el）为传感器系角度，直接合成；
 *       - kInertialStabilized：扫描参数为 ECI 参考方向（期望光轴），先得期望 ECI 单位
 *         向量再经链路反解到传感器系（R^T 旋转），物理上保持惯性方向稳定。
 *       安装失准（阶段 3）：misalignment 为安装失准角误差（常值偏置 + 运行期一次抽取的
 *       常值随机微扰），作用于传感器系内（等效安装偏置微扰），合成进链路影响实际光轴
 *       足迹与门控；不污染量测输出、不进 BuildMeasurementCovariance。
 *       随机抽取在 SbirsPipeline 承载（DrawMisalignmentTotal）；本类与其委托的公共
 *       引擎只承载纯几何（ECI<->传感器系旋转、传感器系 az/el 提取、限位钳制与
 *       扫掠区间判定），不含任何随机源/时间演化状态。
 */
class SbirsBoresightChain {
 public:
  /** @brief 空链：单位矩阵（零姿态、零安装角、零失准），供默认构造。 */
  SbirsBoresightChain() = default;

  /**
   * @brief 由卫星姿态（Body->ECI）与安装角（Body->Sensor）构造合成链（失准为零）。
   * @param[in] attitude_eci_body_deg 卫星姿态欧拉角（Z-Y-X，deg）
   * @param[in] mount_angles_deg 传感器安装偏置角（Body->Sensor，deg）
   */
  SbirsBoresightChain(const session::SbirsEulerAnglesDeg& attitude_eci_body_deg,
                      const oneq::coordinate::EulerAnglesDeg& mount_angles_deg);

  /**
   * @brief 由卫星姿态（Body->ECI）、安装角（Body->Sensor）与安装失准角构造合成链。
   * @details 合成关系：R_sensor_to_eci = R_body_to_eci · R_sensor_to_body ·
   *          R_sensor_misalign⁻¹（失准作用于传感器系内，等效安装偏置微扰）。
   * @param[in] attitude_eci_body_deg 卫星姿态欧拉角（Z-Y-X，deg）
   * @param[in] mount_angles_deg 传感器安装偏置角（Body->Sensor，deg）
   * @param[in] misalignment_deg 安装失准角误差（常值偏置 + 运行期一次随机抽取，deg）
   */
  SbirsBoresightChain(const session::SbirsEulerAnglesDeg& attitude_eci_body_deg,
                      const oneq::coordinate::EulerAnglesDeg& mount_angles_deg,
                      const oneq::coordinate::EulerAnglesDeg& misalignment_deg);

  /** @brief 是否为恒等链（零姿态 + 零安装角）。 */
  bool IsIdentity() const;

  /**
   * @brief 传感器系单位向量旋转到 ECI 分量。
   * @param[in] sensor_los 传感器系单位视线向量
   * @return ECI 分量
   */
  session::SbirsVector3M RotateSensorToEci(const session::SbirsVector3M& sensor_los) const;

  /**
   * @brief ECI 向量旋转到传感器系分量。
   * @param[in] eci_vector ECI 向量（不要求单位化）
   * @return 传感器系分量（模长不变）
   */
  session::SbirsVector3M RotateEciToSensor(const session::SbirsVector3M& eci_vector) const;

  /**
   * @brief 传感器系单位指向向量旋转到 ECI 单位向量。
   * @param[in] sensor_azimuth_deg 传感器系方位角（单位：deg）
   * @param[in] sensor_elevation_deg 传感器系俯仰角（单位：deg）
   * @return ECI 单位向量
   */
  session::SbirsVector3M EciLosOfSensorPointing(float sensor_azimuth_deg,
                                                float sensor_elevation_deg) const;

  /**
   * @brief ECI 单位向量反解为传感器系 az/el（单位：deg，方位对称域 (-180, 180]）。
   * @param[in] eci_los ECI 单位向量
   * @param[out] azimuth_deg 传感器系方位角（单位：deg）
   * @param[out] elevation_deg 传感器系俯仰角（单位：deg）
   */
  void SensorAzElOfEciVector(const session::SbirsVector3M& eci_los, float* azimuth_deg,
                             float* elevation_deg) const;

  /**
   * @brief 期望 ECI 光轴（单位向量）经链路反解为传感器系指向（惯性稳定反解）。
   * @param[in] desired_eci_los 期望光轴 ECI 单位向量
   * @param[out] azimuth_deg 传感器系方位角（单位：deg）
   * @param[out] elevation_deg 传感器系俯仰角（单位：deg）
   */
  void SensorPointingForDesiredEciLos(const session::SbirsVector3M& desired_eci_los,
                                      float* azimuth_deg, float* elevation_deg) const;

  /**
   * @brief 按传感器系扫描限位钳制 az/el（钳制到窗口边界，不折叠方位）。
   * @param[in] limits 传感器系扫描限位
   * @param[in,out] azimuth_deg 方位角（单位：deg，钳制后）
   * @param[in,out] elevation_deg 俯仰角（单位：deg，钳制后）
   */
  static void ClampToScanLimits(const config::SbirsScanLimitsDeg& limits, float* azimuth_deg,
                                float* elevation_deg);

 private:
  oneq::common::geometry::BoresightChain chain_;
};

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_BORESIGHT_CHAIN_H_
