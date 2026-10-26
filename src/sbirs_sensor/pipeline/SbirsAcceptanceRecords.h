/**
 * @file SbirsAcceptanceRecords.h
 * @brief SBIRS 验收行拼装（安装矩阵、轨道角抽样累计、生命周期、性能暂无项）。
 */

/* =============================================================================
 * 外部集成使用说明（红外 / SBIRS，2026-10-27）
 * -----------------------------------------------------------------------------
 * 本头文件各函数把**已算好**的观测量按验收项组织成一行，落 `SatelliteTarget.log`
 * （经 `SBIRS_ACCEPTANCE_ITEM` 宏 → `WriteAcceptanceLog(kSatelliteTarget, …)`）。
 * 调用方只需按签名传值，不要自己拼 `仿真时间=/仿真周期=/[项名]` 前缀。
 *
 * 函数与验收项对应：
 *   WriteSbirsInstallMatrices          安装矩阵误差（第 3 项）——仅在矩阵变化时写
 *   WriteSbirsOrbitSample              卫星自身定位误差（第 8 项）——每周期一行
 *   WriteSbirsAngleError               红外载荷角误差
 *   WriteSbirsTargetDetectability      目标可探测性与动态参数（第 7 项）——双星定位时一行，
 *                                      单星候选行顺延到下一周期首次写出；**会话结束必须
 *                                      调 Flush 补齐尾拍**，否则最后一拍的候选会丢
 *   WriteSbirsWideFieldScan / 窄场跟踪 宽视场扫描探测 / 窄视场跟踪探测（每驻留一行）
 *   WriteSbirsMaxDetectionRange        最大探测距离（第 4 项）
 *   WriteSbirsEvents                   特殊事件监测与提示
 *   Flush…                             把缓冲中的候选行落盘（会话退出前调用）
 *
 * 约定：
 *   - 行内角度用 deg、距离用 m、功率用 W；与其他验收文件保持一致；
 *   - 事件类项（关机点预测、接力、协同融合）只在事件发生周期写一行，不是每周期；
 *   - 本文件的写入同样受 `SbirsAcceptanceLog.h` 的两级开关控制（编译期 + 运行期）。
 * ========================================================================== */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_RECORDS_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_RECORDS_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/sbirs_sensor/config/SbirsEnvironmentConfig.h"
#include "1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"
#include "sbirs_sensor/pipeline/SbirsPipeline.h"

namespace sbirs_sensor {
namespace pipeline {

// satellite_id：卫星实体/融合源 ID（验收行 卫星ID= 标注；0 表示调用方未标注）。
void WriteSbirsInstallMatrices(std::uint32_t satellite_id,
                               const oneq::coordinate::EulerAnglesDeg& mount_deg,
                               const oneq::coordinate::EulerAnglesDeg& misalignment_deg);

// 规范口径（验收判定标准 第8项）：每周期一行 卫星ID/真值位置ECEF/实际位置ECEF/
// 定位误差ECEF/模长/真值速度ECEF/实际速度ECEF/速度误差ECEF。
void WriteSbirsOrbitSample(std::uint32_t satellite_id, float sim_time_sec, std::uint32_t cycle,
                           float orbit_sigma_deg, double reference_range_m,
                           float nav_position_sigma_m,
                           const session::SbirsVector3M& satellite_ecef,
                           const session::SbirsVector3M& satellite_velocity_ecef);

void WriteSbirsAngleError(std::uint32_t satellite_id, float sim_time_sec, std::uint32_t cycle,
                          std::uint64_t target_id, double az_error_deg, double el_error_deg);

// 验收判定标准 第7项（修改）：目标可探测性与动态参数——双星定位时同一目标单行
//（位置 LLA 只在该情况下写出，两颗相对卫星ID/斜距/量测角同行）；单星候选行待
// 同周期无第二星时顺延到下一周期首次写出时落盘。会话结束须调 Flush补齐尾拍。
void WriteSbirsTargetDetectability(std::uint32_t satellite_entity_id, float sim_time_sec,
                                   std::uint32_t cycle, std::uint64_t target_id,
                                   double slant_range_m, double measured_az_rad,
                                   double measured_el_rad, bool narrow_field,
                                   double satellite_ecef_x_m,
                                   double satellite_ecef_y_m, double satellite_ecef_z_m,
                                   double gmst_rad);

// 落盘当前输出目录内所有待定可探测性行（会话结束时调用，补最后一拍的顺延行）。
void FlushSbirsDetectabilityPending();

// kAngleCvKf 后验：滤波方位/俯仰及其变化率。不进公开检测记录（F4）。
void WriteSbirsAngleStateEstimate(float sim_time_sec, std::uint32_t cycle, std::uint64_t target_id,
                                  double azimuth_deg, double elevation_deg,
                                  double azimuth_rate_deg_per_s, double elevation_rate_deg_per_s);

void WriteSbirsLifecycleEvents(std::uint32_t satellite_id, float sim_time_sec,
                               std::uint32_t cycle,
                               const std::vector<session::SbirsDetectionLifecycleEvent>& events,
                               const session::SbirsCycleInput& input);

/**
 * @brief 验收判定标准 第58项：目标与红外环境场景输入投影 → log/IRExternalInterface.log。
 *
 * 记录目标数量、逐目标状态数组（ECEF 位置/速度、有效标志、红外辐射强度）、背景辐射
 * 亮度与基础大气透过率。背景噪声分量由载荷/环境在模型内部计算，本项只投影输入事实。
 */
void WriteSbirsExternalSceneInput(const config::SbirsEnvironmentConfig& environment,
                                  double background_radiance_w_sr_m2,
                                  double detector_temperature_k, float sim_time_sec,
                                  std::uint32_t cycle, std::uint32_t satellite_id,
                                  const session::SbirsSceneTargetList& scene);

/**
 * @brief 验收判定标准 第67项：红外观测与航迹输出接口测试 → log/IRExternalInterface.log。
 *
 * 分三层输出：原始观测（检测 ID/ECI 方位俯仰/IR SNR/阶段/检测标志）、归属诊断
 *（目标 ID 与估计距离，显式标注非被动红外直接测距）、跟踪状态（状态估计与协方差）。
 */
void WriteSbirsExternalObservationOutput(
    std::uint32_t satellite_id, float sim_time_sec, std::uint32_t cycle,
    const std::vector<SbirsPipelineDetection>& detections, const SbirsPipelineSnapshot& snapshot);

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif
