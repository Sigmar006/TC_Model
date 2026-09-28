/**
 * @file DetectionCellResolver.h
 * @brief 统计级 range-Doppler-beam-time-frequency detection cell 单源求解器。
 *
 * AR/RIR 共用；ECCM 仅以 enable_anti_rgpo_leading_edge 标量钩子存在（RIR 恒 false）。
 */

#ifndef COMMON_RADAR_DETECTION_CELL_RESOLVER_H_
#define COMMON_RADAR_DETECTION_CELL_RESOLVER_H_

#include <cstdint>
#include <vector>

#include "1q/electromagnetics/RfScene.h"

namespace oneq {
namespace common {
namespace radar {

/** @brief 四增益偏置（dB；缺省 0 = 保守账本）。 */
struct DetectionCellGainOffsetsDb {
  double target_processing_gain_db{0.0};
  double noise_processing_gain_db{0.0};
  double clutter_suppression_gain_db{0.0};
  double jamming_suppression_gain_db{0.0};
};

/** @brief detection cell 所需的冻结雷达工作参数（标量 + 共享 RF 类型）。 */
struct DetectionCellConfig {
  oneq::electromagnetics::RfWaveformSchedule own_transmit_waveform{};
  double receive_window_start_time_s{0.0};
  double receive_window_duration_s{0.0};
  double matched_filter_bandwidth_hz{0.0};
  double one_way_antenna_gain_dbi{0.0};
  double receiver_loss_db{0.0};
  double receiver_noise_figure_db{0.0};
  double reference_temperature_k{290.0};
  bool enable_anti_rgpo_leading_edge{false};
  DetectionCellGainOffsetsDb signal_processing{};
};

/** @brief 一个目标在本周期 detection cell 中的物理事实。 */
struct DetectionCellTarget {
  double range_m{0.0};
  double closing_radial_velocity_mps{0.0};
  double rcs_m2{0.0};
  double two_way_additional_propagation_loss_db{0.0};
  std::uint32_t effective_pulse_count{1U};
};

/** @brief 单目标 detection cell 的统计级求解结果。 */
struct DetectionCellResult {
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
 * @return 成功返回 true；非法输入原子拒绝且不修改 @p result。
 */
bool TryResolveDetectionCell(
    const DetectionCellConfig& config, const DetectionCellTarget& target,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double clutter_power_w, DetectionCellResult* result);

}  // namespace radar
}  // namespace common
}  // namespace oneq

#endif  // COMMON_RADAR_DETECTION_CELL_RESOLVER_H_
