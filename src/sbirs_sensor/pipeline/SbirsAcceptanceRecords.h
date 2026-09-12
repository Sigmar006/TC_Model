/**
 * @file SbirsAcceptanceRecords.h
 * @brief SBIRS 验收行拼装（安装矩阵、轨道角抽样累计、生命周期、性能暂无项）。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_RECORDS_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_RECORDS_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace sbirs_sensor {
namespace pipeline {

// satellite_id：卫星实体/融合源 ID（验收行 卫星ID= 标注；0 表示调用方未标注）。
void WriteSbirsInstallMatrices(std::uint32_t satellite_id,
                               const oneq::coordinate::EulerAnglesDeg& mount_deg,
                               const oneq::coordinate::EulerAnglesDeg& misalignment_deg);

// 规范口径（验收判定标准 第8项）：每周期一行 卫星ID/真值ECEF/实际ECEF/定位误差ECEF/模长。
void WriteSbirsOrbitSample(std::uint32_t satellite_id, float sim_time_sec, std::uint32_t cycle,
                           float orbit_sigma_deg, double reference_range_m,
                           float nav_position_sigma_m,
                           const session::SbirsVector3M& satellite_ecef);

void WriteSbirsAngleError(std::uint32_t satellite_id, float sim_time_sec, std::uint32_t cycle,
                          std::uint64_t target_id, double az_error_deg, double el_error_deg,
                          double measured_az_deg, double measured_el_deg);

// 验收判定标准 第7项（修改）：目标可探测性与动态参数——双星定位时同一目标单行
// （位置 LLA 只在该情况下写出，两颗相对卫星ID/斜距/量测角同行）；单星候选行待
// 同周期无第二星时顺延到下一周期首次写出时落盘。会话结束须调 Flush补齐尾拍。
void WriteSbirsTargetDetectability(std::uint32_t satellite_entity_id, float sim_time_sec,
                                   std::uint32_t cycle, std::uint64_t target_id,
                                   double slant_range_m, double measured_az_rad,
                                   double measured_el_rad, bool narrow_field,
                                   double satellite_ecef_x_m, double satellite_ecef_y_m,
                                   double satellite_ecef_z_m, double gmst_rad);

// 落盘所有待定可探测性行（会话结束时调用，补最后一拍的顺延行）。
void FlushSbirsDetectabilityPending();

// kAngleCvKf 后验：滤波方位/俯仰及其变化率。不进公开检测记录（F4）。
void WriteSbirsAngleStateEstimate(float sim_time_sec, std::uint32_t cycle, std::uint64_t target_id,
                                  double azimuth_deg, double elevation_deg,
                                  double azimuth_rate_deg_per_s, double elevation_rate_deg_per_s);

void WriteSbirsLifecycleEvents(std::uint32_t satellite_id, float sim_time_sec, std::uint32_t cycle,
                               const std::vector<session::SbirsDetectionLifecycleEvent>& events,
                               const session::SbirsCycleInput& input);

void WriteSbirsOncePerSession(float sim_time_sec, std::uint32_t cycle);
void WriteSbirsCycleRunCount(float sim_time_sec, std::uint32_t cycle);

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif
