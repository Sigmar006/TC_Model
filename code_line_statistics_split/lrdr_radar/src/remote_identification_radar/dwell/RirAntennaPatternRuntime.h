/**
 * @file RirAntennaPatternRuntime.h
 * @brief 定义 RIR 方向图运行期中间量与评估函数（RIR 薄适配层，common 单源）。
 * @note 本文件仅供 RIR 模块内部使用，不作为公开 API。
 * @note 三维天线增益方向图数据表不输出（2026-08-20 验收输出统计裁定，
 *       docs/review/acceptance_output_inventory_2026-08-20.md §4.4/§6）；逐目标
 *       方向图评估结果（离轴增益，含主瓣衰减/扫描损耗的总效果）经 [RirAccept]
 *       detection_cell 事件的 gain_dbi 字段输出，衰减分量不单列。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_ANTENNA_PATTERN_RUNTIME_H_
#define REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_ANTENNA_PATTERN_RUNTIME_H_

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "common/radar/AntennaPatternRuntime.h"

namespace remote_identification_radar {
namespace dwell {

/**
 * @brief RirAntennaPatternBeamwidthDeg 表示方向图评估使用的有效波束宽度。
 */
struct RirAntennaPatternBeamwidthDeg {
  float az_beamwidth_deg{3.0f};
  float el_beamwidth_deg{3.0f};
};

/**
 * @brief RirAntennaLookOffsetDeg 表示目标相对当前波束中心的离轴角。
 */
struct RirAntennaLookOffsetDeg {
  float delta_az_deg{0.0f};
  float delta_el_deg{0.0f};
};

/**
 * @brief RirAntennaPatternSample 表示方向图采样结果。
 */
struct RirAntennaPatternSample {
  float gain_dbi{0.0f};
  float main_lobe_attenuation_db{0.0f};
  float scan_loss_db{0.0f};
  bool inside_main_lobe{false};
  bool inside_back_lobe{false};
};

namespace rir_antenna_pattern_adapter {

inline oneq::common::radar::AntennaPatternBeamwidthDeg ToCommonBeamwidth(
    const RirAntennaPatternBeamwidthDeg& beamwidth_deg) {
  return oneq::common::radar::AntennaPatternBeamwidthDeg{beamwidth_deg.az_beamwidth_deg,
                                                         beamwidth_deg.el_beamwidth_deg};
}

inline oneq::common::radar::AntennaLookOffsetDeg ToCommonLookOffset(
    const RirAntennaLookOffsetDeg& offset_deg) {
  return oneq::common::radar::AntennaLookOffsetDeg{offset_deg.delta_az_deg,
                                                   offset_deg.delta_el_deg};
}

inline oneq::common::radar::AntennaPatternConfig ToCommonPatternConfig(
    const config::hardware::RirAntennaConfig& antenna) {
  oneq::common::radar::AntennaPatternConfig common_config;
  common_config.model_type = static_cast<oneq::common::radar::AntennaPatternModelType>(
      static_cast<int>(antenna.model_type));
  common_config.max_sidelobe_level_db = antenna.max_sidelobe_level_db;
  common_config.backlobe_level_db = antenna.backlobe_level_db;
  common_config.scan_loss_coeff_db_per_deg2 = antenna.scan_loss_coeff_db_per_deg2;
  common_config.max_scan_loss_db = antenna.max_scan_loss_db;
  common_config.boresight_offset_deg.az_deg = antenna.boresight_offset_deg.az_deg;
  common_config.boresight_offset_deg.el_deg = antenna.boresight_offset_deg.el_deg;
  return common_config;
}

inline oneq::common::radar::AzimuthElevationDeg ToCommonPointing(
    const config::RirAzimuthElevationDeg& beam_pointing_deg) {
  return oneq::common::radar::AzimuthElevationDeg{beam_pointing_deg.az_deg,
                                                  beam_pointing_deg.el_deg};
}

inline RirAntennaPatternSample FromCommonSample(
    const oneq::common::radar::AntennaPatternSample& sample) {
  RirAntennaPatternSample result;
  result.gain_dbi = sample.gain_dbi;
  result.main_lobe_attenuation_db = sample.main_lobe_attenuation_db;
  result.scan_loss_db = sample.scan_loss_db;
  result.inside_main_lobe = sample.inside_main_lobe;
  result.inside_back_lobe = sample.inside_back_lobe;
  return result;
}

}  // namespace rir_antenna_pattern_adapter

inline bool RirIsInsideMainLobe(const RirAntennaPatternBeamwidthDeg& beamwidth_deg,
                                const RirAntennaLookOffsetDeg& offset_deg) {
  return oneq::common::radar::IsInsideMainLobe(
      rir_antenna_pattern_adapter::ToCommonBeamwidth(beamwidth_deg),
      rir_antenna_pattern_adapter::ToCommonLookOffset(offset_deg));
}

inline float RirComputeMainLobeAttenuationDb(
    const config::hardware::RirAntennaConfig& antenna,
    const RirAntennaPatternBeamwidthDeg& beamwidth_deg, const RirAntennaLookOffsetDeg& offset_deg,
    float wavelength_m = 0.0f) {
  return oneq::common::radar::ComputeMainLobeAttenuationDb(
      rir_antenna_pattern_adapter::ToCommonPatternConfig(antenna),
      rir_antenna_pattern_adapter::ToCommonBeamwidth(beamwidth_deg),
      rir_antenna_pattern_adapter::ToCommonLookOffset(offset_deg), antenna.antenna_length_m,
      antenna.antenna_width_m, wavelength_m);
}

inline float RirComputeScanLossDb(const config::hardware::RirAntennaConfig& antenna,
                                  const config::RirAzimuthElevationDeg& beam_pointing_deg) {
  return oneq::common::radar::ComputeScanLossDb(
      rir_antenna_pattern_adapter::ToCommonPatternConfig(antenna),
      rir_antenna_pattern_adapter::ToCommonPointing(beam_pointing_deg));
}

inline RirAntennaPatternSample RirEvaluateAntennaPattern(
    const config::hardware::RirAntennaConfig& antenna,
    const RirAntennaPatternBeamwidthDeg& beamwidth_deg, const RirAntennaLookOffsetDeg& offset_deg,
    const config::RirAzimuthElevationDeg& beam_pointing_deg, float wavelength_m = 0.0f) {
  return rir_antenna_pattern_adapter::FromCommonSample(
      oneq::common::radar::EvaluateAntennaPattern(
          antenna.main_beam_gain_db, rir_antenna_pattern_adapter::ToCommonPatternConfig(antenna),
          rir_antenna_pattern_adapter::ToCommonBeamwidth(beamwidth_deg),
          rir_antenna_pattern_adapter::ToCommonLookOffset(offset_deg),
          rir_antenna_pattern_adapter::ToCommonPointing(beam_pointing_deg),
          antenna.antenna_length_m, antenna.antenna_width_m, wavelength_m));
}

}  // namespace dwell
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_ANTENNA_PATTERN_RUNTIME_H_
