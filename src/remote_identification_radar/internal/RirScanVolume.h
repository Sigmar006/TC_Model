/**
 * @file RirScanVolume.h
 * @brief 可扫描体积判定（会话驻留门与检测候选裁剪共用单源）。
 */

#ifndef ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_SCAN_VOLUME_H_
#define ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_SCAN_VOLUME_H_

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "1q/remote_identification_radar/config/RirMissionConfig.h"
#include "common/radar/ScanScheduleRuntime.h"

namespace remote_identification_radar {
namespace internal {

/**
 * @brief 目标视线角是否在 scan_center + 可扫描体积内（az 相对、el 绝对）。
 * @param[in] look        目标视线角（雷达局部 ENU，deg）。
 * @param[in] volume      阵面相对可扫描体积（az 相对 scan_center，el 绝对，deg）。
 * @param[in] scan_center 转台当前朝向（ENU az/el，deg）。
 * @return 体积内为 true。
 * @note 同一口径两处消费：会话层指定识别目标驻留门（RirSession）与检测候选
 *       角域裁剪（RirController，2026-08-22 甲方批注「设定方位俯仰进行扫描」）。
 */
inline bool TargetWithinSteerableVolume(
    const config::RirAzimuthElevationDeg& look,
    const config::RirAzimuthElevationLimitsDeg& volume,
    const config::RirAzimuthElevationDeg& scan_center) {
  const float delta_az_deg =
      oneq::common::radar::NormalizeAzimuthDeltaDeg(look.az_deg - scan_center.az_deg);
  return delta_az_deg >= volume.az_min_deg && delta_az_deg <= volume.az_max_deg &&
         look.el_deg >= volume.el_min_deg && look.el_deg <= volume.el_max_deg;
}

/**
 * @brief 实际搜索扇区 = 任务扫描子窗 ∩ 硬件可扫描体积（逐轴取交，az 相对、el 绝对）。
 * @param[in] window 任务扫描子窗（用户指定作战搜索扇区；缺省无界即不收窄）。
 * @param[in] volume 硬件最大可扫描体积。
 * @return 逐轴交集限位；子窗缺省无界时结果等于 volume（既有行为兼容）。
 * @note 交集可能为空（min > max）；空扇区在扫描内核（BuildScanPattern）与候选
 *       裁剪处按「无波位/无候选」自然退化，不额外抛错（校验层已拒非法子窗）。
 */
inline config::RirAzimuthElevationLimitsDeg IntersectScanSector(
    const config::RirAzimuthElevationLimitsDeg& window,
    const config::RirAzimuthElevationLimitsDeg& volume) {
  config::RirAzimuthElevationLimitsDeg sector;
  sector.az_min_deg = window.az_min_deg > volume.az_min_deg ? window.az_min_deg : volume.az_min_deg;
  sector.az_max_deg = window.az_max_deg < volume.az_max_deg ? window.az_max_deg : volume.az_max_deg;
  sector.el_min_deg = window.el_min_deg > volume.el_min_deg ? window.el_min_deg : volume.el_min_deg;
  sector.el_max_deg = window.el_max_deg < volume.el_max_deg ? window.el_max_deg : volume.el_max_deg;
  return sector;
}

/**
 * @brief 解析地平线扫描的仰角带（`search_mode == kHorizonScan` 时用）。
 *
 * @details 自适应口径（`el_band_auto = true`）：`[−θ_el/2, +2°]`——下沿取半个俯仰波束宽，
 *   让主瓣下沿正好掠过地平线（波束中心在地平线上时下沿已入地，才会产生擦地杂波），
 *   上沿 2° 覆盖低空目标。手动口径直接用配置值，只保证有限与有序。
 *
 * @param[in] horizon    地平线扫描参数。
 * @param[in] el_beam_deg 有效俯仰波束宽（deg；≤0 时自适应下沿退化为 0）。
 * @return 仰角带限位（deg，绝对仰角口径）。
 */
inline config::RirAzimuthElevationLimitsDeg ResolveHorizonBand(
    const config::RirHorizonScanConfig& horizon, float el_beam_deg) {
  config::RirAzimuthElevationLimitsDeg band;
  if (horizon.el_band_auto) {
    band.el_min_deg = el_beam_deg > 0.0f ? -0.5f * el_beam_deg : 0.0f;
    band.el_max_deg = 2.0f;
  } else {
    band.el_min_deg = horizon.el_lower_deg;
    band.el_max_deg = horizon.el_upper_deg;
  }
  return band;
}

/**
 * @brief 实际搜索扇区（搜索模式单源）：方位仍取 `scan_window_deg`，仰角按模式取。
 *
 * @details 两种模式共用方位子窗，只切换仰角口径：
 *   - `kSectorScan`：仰角 = `scan_window_deg` 的 el 区间（既有行为，逐位不变）；
 *   - `kHorizonScan`：仰角 = `ResolveHorizonBand()` 解析出的地平线窄带。
 *   之后再与硬件可扫描体积逐轴求交，得到扫描内核与候选裁剪共用的同一扇区。
 *
 * @param[in] mission      任务域配置（读 search_mode / scan_window_deg / horizon_scan）。
 * @param[in] volume       硬件最大可扫描体积（az 相对 scan_center、el 绝对）。
 * @param[in] el_beam_deg  有效俯仰波束宽（deg，仅地平线自适应带使用）。
 * @return 逐轴交集限位；交集可能为空（内核按无波位自然退化）。
 */
inline config::RirAzimuthElevationLimitsDeg ResolveSearchSector(
    const config::RirMissionConfig& mission,
    const config::RirAzimuthElevationLimitsDeg& volume, float el_beam_deg) {
  if (mission.search_mode == config::RirSearchMode::kHorizonScan) {
    config::RirAzimuthElevationLimitsDeg window = mission.scan_window_deg;
    const config::RirAzimuthElevationLimitsDeg band = ResolveHorizonBand(mission.horizon_scan, el_beam_deg);
    window.el_min_deg = band.el_min_deg;
    window.el_max_deg = band.el_max_deg;
    return IntersectScanSector(window, volume);
  }
  return IntersectScanSector(mission.scan_window_deg, volume);
}

}  // namespace internal
}  // namespace remote_identification_radar

#endif  // ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_SCAN_VOLUME_H_
