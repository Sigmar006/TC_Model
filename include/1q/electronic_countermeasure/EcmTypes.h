/**
 * @file EcmTypes.h
 * @brief 定义电子对抗模块的配置、周期输入、发射输出和快照值类型。
 */

#ifndef ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_TYPES_H_
#define ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/electromagnetics/RfScene.h"

namespace electronic_countermeasure {

/** @brief ECM 威胁输入来源；两种模式的载荷必须严格互斥。 */
enum class EcmInputMode : std::uint8_t {
  kSensorDriven = 0,
  kTruthAssisted = 1,
};

/** @brief 支持的压制干扰技术。 */
enum class EcmTechnique : std::uint8_t {
  kSpot = 0,
  kBarrage = 1,
  kSweep = 2,
  kDeception = 3,
};

/** @brief 欺骗干扰子技术。 */
enum class EcmDeceptionMode : std::uint8_t {
  kRgpo = 0,
  kVgpo = 1,
  kRgpoVgpo = 2,
  kFalseTarget = 3,
};

/** @brief 欺骗拖引状态机相位。 */
enum class EcmDeceptionPhase : std::uint8_t {
  kIdle = 0,
  kTowing = 1,
  kHolding = 2,
  kStopped = 3,
};

namespace config {

/** @brief ECM 会话硬件资源和调度策略配置。 */
struct ONEQ_API EcmSessionConfig {
  bool power_on{true};
  std::uint32_t random_seed{20260722U};
  std::uint32_t channel_count{4U};
  double minimum_frequency_hz{0.23e9};
  double maximum_frequency_hz{40.0e9};
  double maximum_total_transmit_power_w{4000.0};
  double maximum_channel_transmit_power_w{1500.0};
  double thermal_capacity_j{2.0e6};
  double cooling_power_w{500.0};
  double spot_bandwidth_hz{5.0e6};
  double barrage_bandwidth_hz{100.0e6};
  double sweep_bandwidth_hz{200.0e6};
  std::uint32_t sweep_segment_count{8U};
  EcmTechnique default_technique{EcmTechnique::kSpot};
  // --- 欺骗干扰配置 ---
  EcmDeceptionMode default_deception_mode{EcmDeceptionMode::kRgpo};
  double deception_rgpo_rate_m_per_s{100.0};
  double deception_rgpo_max_range_m{5000.0};
  double deception_vgpo_rate_hz_per_s{1000.0};
  double deception_vgpo_max_doppler_hz{10000.0};
  double deception_hold_time_s{2.0};
  double deception_power_scale{0.5};
  std::uint32_t deception_max_active{4U};
  double deception_false_target_delay_s{10.0e-6};
  double deception_false_target_doppler_hz{5000.0};
  std::uint32_t deception_max_false_targets_per_threat{5U};
};

/** @brief ECM 运行期可变配置补丁。 */
struct ONEQ_API EcmRuntimeConfigPatch {
  bool has_power_on{false};
  bool power_on{true};
  bool has_maximum_total_transmit_power_w{false};
  double maximum_total_transmit_power_w{0.0};
  bool has_default_technique{false};
  EcmTechnique default_technique{EcmTechnique::kSpot};
  bool has_default_deception_mode{false};
  EcmDeceptionMode default_deception_mode{EcmDeceptionMode::kRgpo};
};

}  // namespace config

namespace session {

/** @brief 去真值化的 ESR 威胁观测，不含场景 emitter ID。 */
struct ONEQ_API EcmSensorObservation {
  std::uint64_t source_hypothesis_id{0U};
  double estimated_center_frequency_hz{0.0};
  double estimated_bandwidth_hz{0.0};
  double estimated_pri_s{0.0};
  double estimated_pulse_width_s{0.0};
  double center_frequency_std_hz{0.0};
  double bandwidth_std_hz{0.0};
  double bearing_az_deg{0.0};
  double bearing_el_deg{0.0};
  double bearing_std_deg{0.0};
  float threat_score{0.0f};
  float confidence{0.0f};
};

/** @brief 一个成功 ESR 周期发布的去真值化观测帧。 */
struct ONEQ_API EcmSensorObservationFrame {
  /**
   * @brief 发布该帧的 ESR 成功批次的 batch_id（即 ESR 只在成功执行周期自增的批次序号）。
   * @note 作为 fresh-frame provenance：必须非 0、且相对上一帧单调递增。这是 ESR 实际发布的批次
   *       出处，不是调用方随手填的本地 cycle 号；伪造单调递增的 batch_id 仍属违反合同。
   *       注意：source world cycle（绑定世界周期）尚未实现，见 design.md §2 的 prototype 限制。
   */
  std::uint64_t source_esr_batch_id{0U};
  std::vector<EcmSensorObservation> observations{};
};

/** @brief TruthAssisted 验证模式的独立威胁载荷。 */
struct ONEQ_API EcmTruthThreat {
  std::uint64_t truth_entity_id{0U};
  double center_frequency_hz{0.0};
  double bandwidth_hz{0.0};
  float threat_score{0.0f};
  double estimated_pri_s{1.0e-3};
  double estimated_pulse_width_s{1.0e-6};
};

/** @brief ECM 单周期输入。 */
struct ONEQ_API EcmCycleInput {
  std::uint32_t cycle_index{0U};
  double cycle_start_time_s{0.0};
  double dt_sec{1.0};
  EcmInputMode input_mode{EcmInputMode::kSensorDriven};
  oneq::coordinate::EcefPositionM platform_position_ecef_m{};
  oneq::coordinate::EcefVelocityMps platform_velocity_ecef_mps{};
  oneq::electromagnetics::RfSceneAntennaPattern transmit_antenna{};
  oneq::electromagnetics::RfScenePolarization transmit_polarization{
      oneq::electromagnetics::RfScenePolarization::kUnpolarized};
  bool has_sensor_observation_frame{false};
  EcmSensorObservationFrame sensor_observation_frame{};
  std::vector<EcmTruthThreat> truth_threats{};
};

/** @brief 单个资源分配决策，仅用于 result/debug。 */
struct ONEQ_API EcmResourceDecision {
  std::uint64_t source_observation_id{0U};
  std::uint64_t truth_entity_id{0U};
  EcmTechnique technique{EcmTechnique::kSpot};
  EcmDeceptionMode deception_mode{EcmDeceptionMode::kRgpo};
  EcmDeceptionPhase deception_phase{EcmDeceptionPhase::kIdle};
  std::uint32_t channel_index{0U};
  double allocated_power_w{0.0};
  std::string reason{};
};

/** @brief ECM 周期执行状态。 */
enum class EcmCycleStatus : std::uint8_t {
  kExecuted = 0,
  kSafeStopNoFreshObservation,
  kPoweredOff,
  kRejectedInvalidInput,
  kRejectedInvalidConfig,
};

/** @brief ECM 周期聚合结果。 */
struct ONEQ_API EcmCycleResult {
  std::uint32_t input_cycle_index{0U};
  EcmCycleStatus status{EcmCycleStatus::kRejectedInvalidInput};
  EcmInputMode input_mode{EcmInputMode::kSensorDriven};
  bool truth_assisted{false};
  bool executed_this_cycle{false};
  bool used_glided_observation{false};
  std::uint32_t observation_age_successful_ecm_cycles{0U};
  std::uint64_t source_esr_batch_id{0U};
  double thermal_energy_j{0.0};
  oneq::electromagnetics::RfEmissionFrame emission_frame{};
  std::vector<EcmResourceDecision> decisions{};
};

/** @brief 单次欺骗干扰交战累积状态。 */
struct ONEQ_API EcmDeceptionState {
  std::uint64_t threat_id{0U};
  EcmDeceptionMode mode{EcmDeceptionMode::kRgpo};
  EcmDeceptionPhase phase{EcmDeceptionPhase::kIdle};
  double current_delay_s{0.0};
  double current_doppler_offset_hz{0.0};
  double phase_elapsed_s{0.0};
  std::uint32_t cycle_count{0U};
  bool engaged{false};
};

/** @brief 无异常运行期补丁应用结果。 */
struct ONEQ_API EcmRuntimeConfigApplyResult {
  bool has_requested_update{false};
  bool applied{false};
};

/** @brief ECM 累积调度状态快照；仅可恢复到创建它的会话。 */
struct ONEQ_API EcmRuntimeState {
  const void* owner_identity{nullptr};
  std::uint32_t schema_version{0U};
  config::EcmSessionConfig active_config{};
  bool has_successful_cycle{false};
  bool has_last_sensor_frame{false};
  EcmSensorObservationFrame last_sensor_frame{};
  std::uint32_t observation_age_successful_ecm_cycles{0U};
  std::uint32_t last_successful_cycle_index{0U};
  bool has_world_chronology{false};
  std::uint32_t last_world_cycle_index{0U};
  double last_world_window_end_time_s{0.0};
  std::uint64_t next_emission_id{1U};
  double thermal_energy_j{0.0};
  std::string scheduling_rng_state{};
  std::string tie_break_rng_state{};
  std::string deception_rng_state{};
  std::vector<EcmDeceptionState> deception_states{};
};

}  // namespace session
}  // namespace electronic_countermeasure

#endif  // ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_TYPES_H_
