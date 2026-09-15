/**
 * @file RirProfileConstants.h
 * @brief 远程识别雷达语义档位常量表。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_PROFILE_CONSTANTS_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_PROFILE_CONSTANTS_H_

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "1q/remote_identification_radar/config/RirPolicyConfig.h"

namespace remote_identification_radar {
namespace config {
namespace profiles {

/**
 * @brief 远程识别档位：2 MW、9.5 GHz、100 MHz 宽带（距离像分辨率 c/(2B)≈1.5 m）、
 * 高增益天线，适用于远距目标识别场景。
 * @note 语义档位示例值，非真实装备参数；等价性测试以同一常量构造 AR 与 Rir
 *       硬件数值。
 */
const RirHardwareConfig kLongRangeIdentificationHardware = [] {
  RirHardwareConfig c{};
  c.transmitter.peak_power_w = 2.0e6f;
  c.transmitter.frequency_hz = 9.5e9f;
  c.transmitter.bandwidth_hz = 100.0e6f;
  c.transmitter.pulse_width_s = 20e-6f;
  c.transmitter.prf_hz = 400.0f;
  c.antenna.main_beam_gain_db = 38.0f;
  c.receiver.noise_figure_db = 3.0f;
  return c;
}();

/** @brief 重点目标识别档位：低积累门槛、宽松分差，适用于对重点航迹快速出结论。 */
const RirRecognitionPolicy kPriorityTrackIdentificationPolicy = [] {
  RirRecognitionPolicy p{};
  p.enabled = true;
  p.min_confirmed_hits = 3U;
  p.min_observation_count = 2U;
  p.acceptance_score = 0.6f;
  p.minimum_margin = 0.05f;
  p.result_hold_sec = 10.0f;
  return p;
}();

}  // namespace profiles
}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_PROFILE_CONSTANTS_H_
