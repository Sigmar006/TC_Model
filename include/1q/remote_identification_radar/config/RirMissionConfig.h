/**
 * @file RirMissionConfig.h
 * @brief 远程识别雷达任务域主配置类型。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_MISSION_CONFIG_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_MISSION_CONFIG_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/foundation/scan_schedule_types.h"
#include "1q/remote_identification_radar/config/RirHardwareConfig.h"

namespace remote_identification_radar {
namespace config {

/** @brief RirWorkMode 表示远程识别雷达工作模式。 */
enum class ONEQ_API RirWorkMode {
  kStby = 0,     /**< 待机：不执行识别积累，结论保持至过期。 */
  kIdentify = 1  /**< 识别驻留：对重点航迹执行识别观测与积累。 */
};

/**
 * @brief RirSearchMode 搜索模式：决定搜索驻留的仰角口径（2026-11-06）。
 *
 * @details 两种搜索在**方位**上共用 `scan_window_deg` 的 az 子窗，区别只在仰角：
 *   - `kSectorScan`：指定空域搜索，仰角直接用 `scan_window_deg` 的 el 区间（既有行为）；
 *   - `kHorizonScan`：地平线扫描，仰角改用地平线附近的窄带（见
 *     `RirHorizonScanConfig`），用于让主瓣下沿擦地、覆盖掠海/掠地目标。
 *
 *   早期实现没有独立模式，靠"驻留俯仰是否接近 0°"反推地平线周期（见
 *   `IsHorizonSearchCycle`）；模式化之后由本字段直接决定，反推只作兼容兜底。
 */
enum class ONEQ_API RirSearchMode : std::uint8_t {
  kSectorScan = 0, /**< 指定空域搜索（默认）：按 scan_window_deg 的 az/el 子窗铺波位。 */
  kHorizonScan = 1 /**< 地平线扫描：方位仍按 az 子窗，仰角改用地平线窄带。 */
};

/**
 * @brief RirHorizonScanConfig 地平线扫描参数（仅 `search_mode == kHorizonScan` 时生效）。
 *
 * @note 仰角带默认**随俯仰波束自适应**：`[−θ_el/2, +2°]`——下沿让波束下沿恰好掠过地平线
 *   （留出半个波束宽，才有擦地杂波），上沿 2° 覆盖低空突防目标。手动给值时
 *   `el_band_auto=false`，直接用 `el_lower_deg`/`el_upper_deg`。
 */
struct ONEQ_API RirHorizonScanConfig {
  /** @brief 仰角带是否随波束自适应（true = 忽略下面两个值，用 [−θ_el/2, +2°]）。 */
  bool el_band_auto{true};
  /** @brief 手动模式下仰角带下沿（deg，相对地平线，可为负）。 */
  float el_lower_deg{-1.0f};
  /** @brief 手动模式下仰角带上沿（deg，相对地平线）。 */
  float el_upper_deg{2.0f};
};

/**
 * @brief RirMissionConfig 远程识别雷达任务域配置。
 * @note `scan_window_deg`：任务扫描子窗（作战搜索扇区，deg）；默认无界。
 *       az 相对 scan_center、el 绝对。实际搜索扇区 = 本窗 ∩ orientation；
 *       扫描波位与搜索候选按此裁剪；子窗外、体积内目标仅「指定识别」豁免。
 * @note `search_mode == kHorizonScan` 时，仰角区间被 `horizon_scan` 解析出的地平线窄带
 *       替换（方位子窗不变），因此 `scan_window_deg` 仍是**方位**搜索范围的单源。
 */
struct ONEQ_API RirMissionConfig {
  RirWorkMode work_mode{RirWorkMode::kStby}; /**< 当前工作模式。 */
  RirSearchMode search_mode{RirSearchMode::kSectorScan}; /**< 搜索模式（扇区/地平线）。 */
  RirHorizonScanConfig horizon_scan{}; /**< 地平线扫描参数（kHorizonScan 时生效）。 */
  RirAzimuthElevationDeg scan_center_deg{}; /**< 转台当前朝向（ENU az/el）；默认 (0,0)=东水平。 */
  float max_range_m{300000.0f};              /**< 识别任务最大作用距离（m），>0。 */
  float recognition_dwell_sec{0.05f};        /**< 单次识别驻留时间（s），>0。 */
  oneq::foundation::ScanStartPosition scan_start_position{
      oneq::foundation::ScanStartPosition::kLeftTop}; /**< 扫描起始象限。 */
  oneq::foundation::ScanSequence scan_sequence{
      oneq::foundation::ScanSequence::kAzimuthFirst}; /**< 二维扫描推进顺序。 */
  float step_scale{1.0f}; /**< 波位步长系数（波束宽度 × 系数），>0。 */
  RirAzimuthElevationLimitsDeg scan_window_deg{-180.0f, 180.0f, -90.0f, 90.0f}; /**< 任务扫描子窗。 */
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_MISSION_CONFIG_H_
