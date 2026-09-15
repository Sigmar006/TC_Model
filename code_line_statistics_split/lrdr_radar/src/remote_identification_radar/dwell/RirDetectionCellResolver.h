/**
 * @file RirDetectionCellResolver.h
 * @brief 定义 RIR 统计级 range-Doppler-beam-time-frequency detection cell 求解器。
 *
 * 副本来源：`src/airborne_radar/signal/detection/ArDetectionCellResolver.*`
 * （审计基线 96de367c，阶段 2-M M3）。与 AR 版的刻意差异（副本头注纪律）：
 *   - 无 `enable_anti_rgpo_leading_edge` 前沿跟踪减半分支（ECCM 语义属 AR）；
 *   - 分项 SINR 施加 `RirSignalProcessingConfig` 四增益偏置（分子 target、
 *     分母 noise/clutter/jamming，《能力边界界定》§3.2；缺省 0 dB 逐位等于
 *     AR 保守账本）。
 * @note 本文件仅供 RIR 模块内部使用，不作为公开 API。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_DETECTION_CELL_RESOLVER_H_
#define REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_DETECTION_CELL_RESOLVER_H_

#include <vector>

#include "1q/electromagnetics/RfScene.h"
#include "1q/remote_identification_radar/config/RirHardwareConfig.h"

namespace remote_identification_radar {
namespace dwell {

/** @brief detection cell 所需的冻结雷达工作参数。 */
struct RirDetectionCellConfig {
  oneq::electromagnetics::RfWaveformSchedule own_transmit_waveform{};
  double receive_window_start_time_s{0.0};
  double receive_window_duration_s{0.0};
  double matched_filter_bandwidth_hz{0.0};
  double one_way_antenna_gain_dbi{0.0};
  double receiver_loss_db{0.0};
  double receiver_noise_figure_db{0.0};
  double reference_temperature_k{290.0};
  config::hardware::RirSignalProcessingConfig signal_processing{}; /**< 四增益偏置。 */
};

/** @brief 一个目标在本周期 detection cell 中的物理事实。 */
struct RirDetectionCellTarget {
  double range_m{0.0};
  double closing_radial_velocity_mps{0.0};
  double rcs_m2{0.0};
  double two_way_additional_propagation_loss_db{0.0};
  std::uint32_t effective_pulse_count{1U};
};

/** @brief 单目标 detection cell 的统计级求解结果（分项功率为施加偏置前的物理量）。 */
struct RirDetectionCellResult {
  double echo_delay_s{0.0};
  double two_way_doppler_shift_hz{0.0};
  double echo_power_w{0.0};
  double pulse_compression_gain{1.0};
  double thermal_noise_power_w{0.0};
  double interference_power_w{0.0};
  double clutter_power_w{0.0};
  double processed_single_pulse_sinr_linear{0.0};
  double processed_single_pulse_sinr_db{0.0};
  std::uint32_t effective_pulse_count{1U};
};

/**
 * @brief 求解一个目标 detection cell；外部 incident link 仅按目标单元内的时频重叠聚合。
 * @param[in] own_emission_identity 当前 RIR 发射身份；该链路不会计入干扰。
 * @param[in] clutter_power_w 当前 cell 的杂波功率（W，施加杂波抑制偏置前）。
 * @return 成功返回 true；非法输入原子拒绝且不修改 @p result。
 */
bool TryResolveRirDetectionCell(
    const RirDetectionCellConfig& config, const RirDetectionCellTarget& target,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double clutter_power_w, RirDetectionCellResult* result);

}  // namespace dwell
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_DETECTION_CELL_RESOLVER_H_
