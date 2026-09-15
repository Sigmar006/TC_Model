/**
 * @file RirEmissionFactory.h
 * @brief 无状态构造器——从周期输入与硬件配置编写一次 RIR 脉冲列发射。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_EMISSION_FACTORY_H_
#define REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_EMISSION_FACTORY_H_

#include <cstdint>

#include "1q/electromagnetics/RfScene.h"
#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "remote_identification_radar/dwell/RirRfCycleState.h"

namespace remote_identification_radar {
namespace dwell {

/**
 * @brief 无状态发射构造器：组装 identity/位置/天线/极化/功率预算/脉冲列波形。
 *
 * 与 AR 版差异：无 ECCM/LPI 控制档，功率缩放与 rejitter 固定为基线（1.0 / 0）。
 */
class RirEmissionFactory {
 public:
  /**
   * @brief 从频率计划按周期序号确定性选择载频。
   * @return 载频（Hz）；计划为空时回退 `transmitter.frequency_hz`。
   */
  static double ResolveCarrierHz(const config::hardware::RirTransmitterConfig& transmitter,
                                 std::uint32_t cycle_index);

  /**
   * @brief 编写一次 RIR 脉冲列发射。
   *
   * @param input 周期 RF 输入（平台/波束/窗口）。
   * @param hardware 硬件配置（发射机/接收机/天线）。
   * @param emission_id 预留的 emission 身份。
   * @param carrier_hz 已解析载频。
   * @param pulse_repetition_interval_s 脉冲重复间隔。
   * @param pulse_count 脉冲数。
   * @param timing_seed 去真值化波形时序种子。
   * @param successful_cycle_count 已成功周期次数（时序种子入参）。
   * @param emission 输出发射事实。
   * @return boresight 解析或波形构造成功返回 true。
   */
  static bool TryBuildEmission(const RirRfCycleInput& input, const config::RirHardwareConfig& hardware,
                               std::uint64_t emission_id, double carrier_hz,
                               double pulse_repetition_interval_s, std::uint32_t pulse_count,
                               std::uint64_t timing_seed, std::uint64_t successful_cycle_count,
                               oneq::electromagnetics::RfSceneEmission* emission);
};

}  // namespace dwell
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_EMISSION_FACTORY_H_
