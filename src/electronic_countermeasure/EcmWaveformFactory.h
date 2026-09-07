/**
 * @file EcmWaveformFactory.h
 * @brief ECM 波形工厂——根据已冻结交战状态构造 RF 波形，不提交功率或 ID。
 *
 * 所有函数为纯静态方法，不拥有可变状态。RNG 参数为非 const 指针，
 * 必须指向候选副本而非真实会话状态——commit 语义由 EcmSession 保证。
 */

#ifndef ELECTRONIC_COUNTERMEASURE_ECM_WAVEFORM_FACTORY_H_
#define ELECTRONIC_COUNTERMEASURE_ECM_WAVEFORM_FACTORY_H_

#include <cstdint>
#include <random>

#include "1q/electromagnetics/RfScene.h"
#include "1q/electronic_countermeasure/EcmTypes.h"
#include "electronic_countermeasure/EcmInternalTypes.h"

namespace electronic_countermeasure {
namespace session {

/**
 * @brief 无状态波形构造函数集合。
 *
 * 所有参数显式传入，不隐式引用全局或成员状态。RNG 指针非 const 是有意设计——
 * 调用方应传入候选 RNG 副本的地址，所有波形成功后再将候选 RNG 提交到真实状态。
 */
class EcmWaveformFactory {
 public:
  /**
   * @brief 构造压制干扰发射（点频/阻塞/扫频）。
   *
   * @param scheduling_rng 调度 RNG（sweep 方向采样）；必须指向候选副本。
   */
  static bool TryBuildEmission(const EcmCycleInput& input, const config::EcmSessionConfig& config,
                               const SchedulingThreat& threat, double allocated_power_w,
                               std::uint32_t channel_index, std::uint32_t sensor_entity_id,
                               std::uint64_t emission_id, std::mt19937* scheduling_rng,
                               oneq::electromagnetics::RfSceneEmission* output);

  /**
   * @brief 构造欺骗干扰发射（RGPO/VGPO/RGPO+VGPO）。
   *
   * @param deception_state 当前欺骗交战状态的只读快照。
   * @param pulse_rng 欺骗 RNG（脉冲 timing 随机性）；必须指向候选副本。
   */
  static bool TryBuildDeceptionEmission(const EcmCycleInput& input,
                                        const config::EcmSessionConfig& config,
                                        const SchedulingThreat& threat,
                                        const EcmDeceptionState& deception_state,
                                        double allocated_power_w, std::uint32_t sensor_entity_id,
                                        std::uint64_t emission_id, std::mt19937* pulse_rng,
                                        oneq::electromagnetics::RfSceneEmission* output);

  /**
   * @brief 构造假目标发射。
   *
   * @param pulse_rng 欺骗 RNG（脉冲 timing 随机性）；必须指向候选副本。
   * @param ft_index 假目标序号（用于延迟交错，0-indexed）。
   */
  static bool TryBuildFalseTargetEmission(const EcmCycleInput& input,
                                          const config::EcmSessionConfig& config,
                                          const SchedulingThreat& threat,
                                          double allocated_power_w, std::uint32_t sensor_entity_id,
                                          std::uint64_t emission_id, std::mt19937* pulse_rng,
                                          std::uint32_t ft_index,
                                          oneq::electromagnetics::RfSceneEmission* output);
};

}  // namespace session
}  // namespace electronic_countermeasure

#endif  // ELECTRONIC_COUNTERMEASURE_ECM_WAVEFORM_FACTORY_H_
