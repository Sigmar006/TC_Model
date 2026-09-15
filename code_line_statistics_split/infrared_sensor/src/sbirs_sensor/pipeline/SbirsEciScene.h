/**
 * @file SbirsEciScene.h
 * @brief 管线内部 ECI 场景目标：周期入口按 GMST 旋转后的诚实命名副本。
 *
 * 输入契约（session::SbirsSceneTarget）保持 ECEF；pipeline 在周期入口把目标位置与
 * 速度（含 ω×r 输运项）按 GMST 旋转到 ECI 后，下游 LOS/az/el/遮挡/SNR/EKF 全链使用
 * 本类型（2026-08 正式变更，见 docs/common/session_contract.md §传感器方位坐标系约定）。
 * 位置/速度字段名显式标注 eci，避免复用输入结构造成"字段名说 ECEF、值是 ECI"的
 * 语义超载。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ECI_SCENE_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ECI_SCENE_H_

#include <cstdint>
#include <string>

#include "1q/coordinate/inertial_transform.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace sbirs_sensor {
namespace pipeline {

/**
 * @brief ECI 场景目标（管线内部）：输入 ECEF 真值经周期入口 GMST 旋转后的副本。
 * @note 非几何字段（target_id/名称/辐射强度/active）原样透传，语义与输入一致。
 */
struct SbirsEciSceneTarget {
  std::uint64_t target_id{0U};            /**< 目标唯一标识（透传输入） */
  std::string target_name{};              /**< 目标名称（透传输入，仅归属层使用） */
  session::SbirsVector3M position_eci_m{};         /**< ECI 位置（J2000 平赤道面，单位 m） */
  session::SbirsVector3M velocity_eci_m_per_s{};   /**< ECI 速度（含 ω×r 输运项，单位 m/s） */
  bool has_velocity_eci_m_per_s{false};   /**< 是否提供速度（透传输入；false 时速度为零向量） */
  double radiant_intensity_w_per_sr{0.0}; /**< 目标辐射强度（透传输入，单位 W/sr） */
  bool active{true};                      /**< 目标是否在场景中有效（透传输入） */
};

/**
 * @brief 把单个 ECEF 输入目标按 GMST 旋转为 ECI 场景目标。
 * @param[in] source ECEF 输入目标（session 契约层）
 * @param[in] gmst_rad 本周期 GMST（rad，由 coordinate::TryComputeGmstRad 计算）
 * @return ECI 场景目标：位置/速度（含输运项）旋转，非几何字段透传
 * @note 输入校验已保证位置/速度有限；变换失败（不可达防御路径）时保留未旋转值。
 */
SbirsEciSceneTarget RotateSceneTargetToEci(const session::SbirsSceneTarget& source,
                                           double gmst_rad);

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ECI_SCENE_H_
