/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsMissionConfig.
 * Model: SBIRS infrared sensor.
 * Layer: config.
 * File role: interface declarations and data contracts.
 * Responsibility: validated sensor configuration and policy contracts.
 * Upstream boundary: mission, hardware, orientation, environment, and runtime settings.
 * Downstream boundary: session construction and pipeline configuration.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

/**
 * @file SbirsMissionConfig.h
 * @brief 定义 SBIRS-inspired 任务与视场参数。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_MISSION_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_MISSION_CONFIG_H_

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace config {

/** @brief 传感器工作模式：待机、仅 WFOV 宽视场搜索、搜索+凝视协同。 */
enum class ONEQ_API SbirsWorkMode { kStandby = 0, kWideSearch, kSearchAndStare };

/** @brief WFOV 环形扫描方向；扫描速率始终为非负标量。 */
enum class ONEQ_API SbirsScanDirection { kIncreasingAzimuth = 0, kDecreasingAzimuth };

/**
 * @brief 扫描方位基准：ECI 绝对方位（默认，行为不变）或相对星下点方位的偏移。
 * @note nadir 模式下 scan_start_az_deg 是相对星下点方位的带符号偏移（deg），
 *       星下点方位 = atan2(-eci_y, -eci_x) 逐周期由卫星当前位置现算；
 *       偏移 0 + 速率 0 即视场钉在星下点（GEO 星下点凝视的免推算配置）。
 */
enum class ONEQ_API SbirsScanAzimuthReference { kEciAbsolute = 0, kNadirRelative };

/**
 * @brief SBIRS-inspired 任务与视场参数。
 * @note 纯数据类型 (POD)。WFOV/NFOV 视场、扫描范围与速率、距离门控和帧率共同定义
 *       pipeline 几何门控与扫描相位推进；凝视相关参数仅在 NFOV 通道生效。
 *       电源状态由 `SbirsSessionConfig::sensor_enabled` 顶层字段唯一承载
 *       （COMMON-OQ-4 收敛）。
 */
struct ONEQ_API SbirsMissionConfig {
  SbirsWorkMode work_mode{SbirsWorkMode::kSearchAndStare}; /**< 传感器工作模式 */
  float wide_field_fov_az_deg{20.0f};                      /**< WFOV 方位视场（FOV），单位 deg */
  float wide_field_fov_el_deg{20.0f};                      /**< WFOV 俯仰视场（FOV），单位 deg */
  float narrow_field_fov_az_deg{2.0f};                     /**< NFOV 方位视场（IFOV），单位 deg */
  float narrow_field_fov_el_deg{2.0f};                     /**< NFOV 俯仰视场（IFOV），单位 deg */
  // ECI 方位约定（2026-08 正式变更）：合法域 [0, 360)；-60° 等价折入 300°。
  float scan_start_az_deg{300.0f};                        /**< WFOV 扫描方位起点（ECI 方位，单位 deg，[0, 360)） */
  float scan_span_deg{120.0f}; /**< WFOV 有向扫描跨度，范围 (0, 360] deg */
  float initial_scan_phase_deg{0.0f}; /**< WFOV 初始扫描相位（相对扫描起点的初始行程偏移），范围 [0, scan_span) deg，默认 0.0f */
  SbirsScanDirection scan_direction{SbirsScanDirection::kIncreasingAzimuth}; /**< WFOV 扫描方向 */
  // 方位基准（2026-08-31）：kEciAbsolute 时 scan_start_az_deg 语义同上（ECI 绝对方位，
  // 既有行为逐位不变）；kNadirRelative 时为相对星下点方位的带符号偏移（合法域
  // (-360, 360)，0 = 正对星下点），有效起点 = normalize(星下点方位 + 偏移) 逐周期现算。
  SbirsScanAzimuthReference scan_azimuth_reference{
      SbirsScanAzimuthReference::kEciAbsolute}; /**< WFOV 扫描方位基准（绝对 ECI / 相对星下点） */
  float scan_center_el_deg{0.0f};                          /**< WFOV 扫描中心俯仰角，单位 deg */
  // 2-D 俯仰栅格（阶段 4）：与既有方位环扫正交组合；span=0 默认 = 既有单行模式
  // （行中心恒为 scan_center_el_deg，历史行为逐位不变）。行中心 el 与 scan_center_el
  // 同参考系语义（体稳定=传感器系、惯性稳定=ECI）。
  float scan_el_start_deg{0.0f}; /**< WFOV 俯仰栅格起始行中心俯仰角，单位 deg */
  float scan_el_span_deg{0.0f};  /**< WFOV 俯仰栅格跨度，单位 deg；0 = 单行模式（行数=1） */
  float scan_el_step_deg{1.0f};  /**< WFOV 俯仰栅格行间距，单位 deg；span>0 时须 (0, wide_field_fov_el_deg]（无隙覆盖预算） */
  float scan_rate_deg_per_sec{10.0f};                      /**< WFOV 扫描速率，单位 deg/s */
  float min_range_m{1.0e3f};                               /**< 距离门控下限 Dmin，单位 m */
  float max_range_m{5.0e7f};                               /**< 距离门控上限 Dmax，单位 m */
  float frame_rate_hz{10.0f};                              /**< 帧率，单位 Hz */
  float narrow_cue_latency_s{0.0f};                        /**< WFOV→NFOV cue 延迟，单位 s */
  float narrow_pointing_settle_error_deg{0.0f};            /**< NFOV 指向稳定误差，单位 deg */
  float narrow_pointing_max_slew_rate_deg_per_sec{30.0f};  /**< NFOV 光轴最大转速，单位 deg/s */
  float narrow_pointing_settle_tolerance_deg{0.01f};       /**< NFOV 光轴稳定容差，单位 deg */
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_MISSION_CONFIG_H_
