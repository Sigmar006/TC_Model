/**
 * @file EcmSession.cpp
 * @brief EcmSession 实现：编排配置校验、欺骗交战、资源分配和波形构造。
 *
 * 内部委托给 EcmConfigValidator、DeceptionEngagementManager、EcmResourceLedger
 * 和 EcmWaveformFactory。StepWithResult 遵循严格的候选/提交事务模式。
 */

#include "1q/electronic_countermeasure/EcmSession.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "electronic_countermeasure/DeceptionEngagementManager.h"
#include "electronic_countermeasure/EcmConfigValidator.h"
#include "electronic_countermeasure/EcmInternalTypes.h"
#include "electronic_countermeasure/EcmResourceLedger.h"
#include "electronic_countermeasure/EcmWaveformFactory.h"

namespace electronic_countermeasure {
namespace session {

struct EcmSession::Impl {
  explicit Impl(config::EcmSessionConfig value)
      : active_config(std::move(value)),
        deception_mgr(active_config.random_seed),
        resource_ledger(active_config.random_seed) {}

  config::EcmSessionConfig active_config{};

  // 传感器缓存（不属于子组件）
  bool has_successful_cycle{false};
  bool has_last_sensor_frame{false};
  EcmSensorObservationFrame last_sensor_frame{};
  std::uint32_t observation_age_successful_ecm_cycles{0U};
  std::uint32_t last_successful_cycle_index{0U};
  bool has_world_chronology{false};
  std::uint32_t last_world_cycle_index{0U};
  double last_world_window_end_time_s{0.0};

  // 子组件（各自拥有独立的状态域）
  DeceptionEngagementManager deception_mgr;
  EcmResourceLedger resource_ledger;
  std::uint32_t sensor_entity_id{101U};
};

EcmSession::EcmSession() : impl_(new Impl(config::EcmSessionConfig())) {}
EcmSession::~EcmSession() = default;
EcmSession::EcmSession(EcmSession&&) noexcept = default;
EcmSession& EcmSession::operator=(EcmSession&&) noexcept = default;
EcmSession::EcmSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

void EcmSession::SetSensorEntityId(std::uint32_t sensor_entity_id) noexcept {
  impl_->sensor_entity_id = (sensor_entity_id != 0U) ? sensor_entity_id : 101U;
}

EcmSession EcmSession::Create(const config::EcmSessionConfig& config) {
  return EcmSession(std::unique_ptr<Impl>(new Impl(config)));
}

EcmCycleResult EcmSession::StepWithResult(const EcmCycleInput& input) {
  EcmCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.input_mode = input.input_mode;
  result.truth_assisted = input.input_mode == EcmInputMode::kTruthAssisted;
  result.emission_frame.world_cycle_index = input.cycle_index;
  result.emission_frame.window_start_time_s = input.cycle_start_time_s;
  result.emission_frame.window_duration_s = input.dt_sec;

  // 0. 错误处理 lambda——确保所有失败路径一致返回不可变会话状态
  auto RejectInvalidConfig = [&]() -> EcmCycleResult {
    result.status = EcmCycleStatus::kRejectedInvalidConfig;
    result.decisions.clear();
    result.emission_frame.emissions.clear();
    result.thermal_energy_j = impl_->resource_ledger.thermal_energy_j();
    return result;
  };
  auto RejectInvalidInput = [&]() -> EcmCycleResult {
    result.status = EcmCycleStatus::kRejectedInvalidInput;
    result.thermal_energy_j = impl_->resource_ledger.thermal_energy_j();
    return result;
  };

  // 1. 配置校验（纯函数，无副作用）
  if (!EcmConfigValidator::IsValidConfig(impl_->active_config)) {
    return RejectInvalidConfig();
  }

  // 2. 输入校验与时序检查
  if (!EcmConfigValidator::IsValidInput(input) ||
      (impl_->has_successful_cycle && input.cycle_index <= impl_->last_successful_cycle_index) ||
      (impl_->has_world_chronology &&
       (input.cycle_index <= impl_->last_world_cycle_index ||
        input.cycle_start_time_s < impl_->last_world_window_end_time_s))) {
    return RejectInvalidInput();
  }

  // 3. 关机快捷路径
  if (!impl_->active_config.power_on) {
    result.status = EcmCycleStatus::kPoweredOff;
    result.thermal_energy_j = impl_->resource_ledger.thermal_energy_j();
    impl_->has_world_chronology = true;
    impl_->last_world_cycle_index = input.cycle_index;
    impl_->last_world_window_end_time_s = input.cycle_start_time_s + input.dt_sec;
    return result;
  }

  // 4. 创建候选副本（深度复制子组件——此后的所有修改都在候选上进行）
  auto candidate_deception = impl_->deception_mgr;
  auto candidate_ledger = impl_->resource_ledger;
  bool candidate_has_last_frame = impl_->has_last_sensor_frame;
  EcmSensorObservationFrame candidate_last_frame = impl_->last_sensor_frame;
  std::uint32_t candidate_age = impl_->observation_age_successful_ecm_cycles;

  // 5. 应用冷却到候选热状态
  candidate_ledger.ApplyCooling(impl_->active_config.cooling_power_w, input.dt_sec);

  // 6. 构建威胁列表
  std::vector<SchedulingThreat> threats;

  if (input.input_mode == EcmInputMode::kSensorDriven) {
    if (input.has_sensor_observation_frame) {
      if (candidate_has_last_frame && input.sensor_observation_frame.source_esr_batch_id <=
                                          candidate_last_frame.source_esr_batch_id) {
        return RejectInvalidInput();
      }
      candidate_last_frame = input.sensor_observation_frame;
      candidate_has_last_frame = true;
      candidate_age = 0U;
    } else if (candidate_has_last_frame) {
      ++candidate_age;
      result.used_glided_observation = candidate_age <= kMaximumGlideSuccessfulCycles;
    }
    if (candidate_has_last_frame && candidate_age <= kMaximumGlideSuccessfulCycles) {
      result.source_esr_batch_id = candidate_last_frame.source_esr_batch_id;
      for (const EcmSensorObservation& observation : candidate_last_frame.observations) {
        SchedulingThreat threat;
        threat.observation_id = observation.source_hypothesis_id;
        threat.center_frequency_hz = observation.estimated_center_frequency_hz;
        threat.bandwidth_hz = observation.estimated_bandwidth_hz;
        threat.score = observation.threat_score;
        threat.estimated_pri_s = observation.estimated_pri_s;
        threat.estimated_pulse_width_s = observation.estimated_pulse_width_s;
        threats.push_back(threat);
      }
    }
  } else {
    for (const EcmTruthThreat& truth : input.truth_threats) {
      SchedulingThreat threat;
      threat.truth_entity_id = truth.truth_entity_id;
      threat.center_frequency_hz = truth.center_frequency_hz;
      threat.bandwidth_hz = truth.bandwidth_hz;
      threat.score = truth.threat_score;
      threat.estimated_pri_s = truth.estimated_pri_s;
      threat.estimated_pulse_width_s = truth.estimated_pulse_width_s;
      threats.push_back(threat);
    }
  }

  // 7. 推进欺骗状态机（在候选上进行）
  candidate_deception.AdvanceStates(impl_->active_config, input.dt_sec);

  // 8. 可行性过滤
  threats.erase(std::remove_if(threats.begin(), threats.end(),
                               [&](const SchedulingThreat& threat) {
                                 return !EcmResourceLedger::IsFeasibleThreat(threat,
                                                                             impl_->active_config);
                               }),
                threats.end());

  // 9. 派发平局裁决键并排序
  candidate_ledger.AssignTieBreakKeys(&threats);
  std::stable_sort(threats.begin(), threats.end(),
                   [](const SchedulingThreat& lhs, const SchedulingThreat& rhs) {
                     if (lhs.score != rhs.score) {
                       return lhs.score > rhs.score;
                     }
                     return lhs.tie_break_key < rhs.tie_break_key;
                   });

  // 10. 信道分配与波形构造
  const std::size_t selected_count =
      std::min<std::size_t>(threats.size(), impl_->active_config.channel_count);
  const double thermal_power_limit_w =
      candidate_ledger.ComputeThermalPowerLimit(impl_->active_config.thermal_capacity_j,
                                                input.dt_sec);
  double remaining_power_w =
      std::min(impl_->active_config.maximum_total_transmit_power_w, thermal_power_limit_w);

  for (std::size_t index = 0U; index < selected_count && remaining_power_w > 0.0; ++index) {
    const double allocated_power_w =
        std::min(impl_->active_config.maximum_channel_transmit_power_w,
                 remaining_power_w / static_cast<double>(selected_count - index));
    if (allocated_power_w <= 0.0) {
      break;
    }
    const EcmTechnique technique = impl_->active_config.default_technique;
    EcmResourceDecision decision;
    decision.source_observation_id = threats[index].observation_id;
    decision.truth_entity_id = threats[index].truth_entity_id;
    decision.technique = technique;
    decision.channel_index = static_cast<std::uint32_t>(index);
    decision.reason = "highest-threat feasible channel allocation";

    if (technique == EcmTechnique::kDeception) {
      EcmDeceptionState* state = candidate_deception.FindOrCreate(
          ThreatStableId(threats[index]), impl_->active_config.default_deception_mode,
          impl_->active_config.deception_max_active);
      if (state == nullptr) {
        decision.deception_mode = impl_->active_config.default_deception_mode;
        decision.deception_phase = EcmDeceptionPhase::kIdle;
        decision.reason = "deception concurrency cap reached";
        result.decisions.push_back(decision);
        continue;
      }
      decision.deception_mode = state->mode;
      decision.deception_phase = state->phase;
      if (state->phase == EcmDeceptionPhase::kStopped ||
          state->phase == EcmDeceptionPhase::kIdle) {
        decision.reason = "deception engagement released or idle";
        result.decisions.push_back(decision);
        continue;
      }

      const double deception_power_w =
          allocated_power_w * impl_->active_config.deception_power_scale;

      if (state->mode == EcmDeceptionMode::kFalseTarget) {
        const std::uint32_t ft_count = impl_->active_config.deception_max_false_targets_per_threat;
        const double ft_power_w = deception_power_w / static_cast<double>(ft_count);
        for (std::uint32_t ft_idx = 0U; ft_idx < ft_count; ++ft_idx) {
          oneq::electromagnetics::RfSceneEmission emission;
          if (!EcmWaveformFactory::TryBuildFalseTargetEmission(
                  input, impl_->active_config, threats[index], ft_power_w,
                  impl_->sensor_entity_id, candidate_ledger.ReserveEmissionId(),
                  &candidate_deception.deception_rng(), ft_idx, &emission)) {
            // 波形构造失败 → 丢弃候选，返回错误（会话状态不变）
            return RejectInvalidConfig();
          }
          result.emission_frame.emissions.push_back(emission);
        }
        decision.allocated_power_w = deception_power_w;
      } else {
        decision.allocated_power_w = deception_power_w;
        oneq::electromagnetics::RfSceneEmission emission;
        if (!EcmWaveformFactory::TryBuildDeceptionEmission(
                input, impl_->active_config, threats[index], *state, deception_power_w,
                impl_->sensor_entity_id, candidate_ledger.ReserveEmissionId(),
                &candidate_deception.deception_rng(), &emission)) {
          result.status = EcmCycleStatus::kRejectedInvalidConfig;
          result.decisions.clear();
          result.emission_frame.emissions.clear();
          result.thermal_energy_j = impl_->resource_ledger.thermal_energy_j();
          return result;
        }
        result.emission_frame.emissions.push_back(emission);
      }
    } else {
      decision.allocated_power_w = allocated_power_w;
      oneq::electromagnetics::RfSceneEmission emission;
      if (!EcmWaveformFactory::TryBuildEmission(
              input, impl_->active_config, threats[index], allocated_power_w,
              static_cast<std::uint32_t>(index), impl_->sensor_entity_id,
              candidate_ledger.ReserveEmissionId(), &candidate_ledger.scheduling_rng(), &emission)) {
        result.status = EcmCycleStatus::kRejectedInvalidConfig;
        result.decisions.clear();
        result.emission_frame.emissions.clear();
        result.thermal_energy_j = impl_->resource_ledger.thermal_energy_j();
        return result;
      }
      result.emission_frame.emissions.push_back(emission);
    }
    result.decisions.push_back(decision);
    remaining_power_w -= decision.allocated_power_w;
    candidate_ledger.AddThermalEnergy(decision.allocated_power_w * input.dt_sec);
  }

  // 11. 帧校验
  if (!oneq::electromagnetics::TryValidateRfSceneFrame(result.emission_frame)) {
    result.status = EcmCycleStatus::kRejectedInvalidConfig;
    result.decisions.clear();
    result.emission_frame.emissions.clear();
    result.thermal_energy_j = impl_->resource_ledger.thermal_energy_j();
    return result;
  }

  // 12. 原子提交：所有波形成功，将候选状态写回真实状态
  result.executed_this_cycle = true;
  result.status = result.emission_frame.emissions.empty()
                      ? EcmCycleStatus::kSafeStopNoFreshObservation
                      : EcmCycleStatus::kExecuted;
  result.observation_age_successful_ecm_cycles = candidate_age;
  result.thermal_energy_j = candidate_ledger.thermal_energy_j();

  impl_->has_successful_cycle = true;
  impl_->has_last_sensor_frame = candidate_has_last_frame;
  impl_->last_sensor_frame = candidate_last_frame;
  impl_->observation_age_successful_ecm_cycles = candidate_age;
  impl_->last_successful_cycle_index = input.cycle_index;
  impl_->has_world_chronology = true;
  impl_->last_world_cycle_index = input.cycle_index;
  impl_->last_world_window_end_time_s = input.cycle_start_time_s + input.dt_sec;
  impl_->resource_ledger = candidate_ledger;
  impl_->deception_mgr = candidate_deception;
  return result;
}

EcmRuntimeConfigApplyResult EcmSession::ApplyRuntimeConfig(
    const config::EcmRuntimeConfigPatch& patch) {
  EcmRuntimeConfigApplyResult result;
  result.has_requested_update = patch.has_power_on || patch.has_maximum_total_transmit_power_w ||
                                patch.has_default_technique || patch.has_default_deception_mode;
  if (!result.has_requested_update) {
    return result;
  }

  // 合并补丁并验证
  config::EcmSessionConfig candidate;
  if (!EcmConfigValidator::TryMergePatch(impl_->active_config, patch, &candidate)) {
    return result;
  }

  // 检查是否需要清空欺骗交战状态
  if (DeceptionEngagementManager::ModeChangeInvalidates(impl_->active_config, candidate)) {
    impl_->deception_mgr.Clear();
  }
  impl_->active_config = candidate;
  result.applied = true;
  return result;
}

EcmRuntimeState EcmSession::CaptureRuntimeState() const {
  EcmRuntimeState state;
  state.owner_identity = impl_.get();
  state.schema_version = kRuntimeStateSchemaVersion;
  state.active_config = impl_->active_config;
  state.has_successful_cycle = impl_->has_successful_cycle;
  state.has_last_sensor_frame = impl_->has_last_sensor_frame;
  state.last_sensor_frame = impl_->last_sensor_frame;
  state.observation_age_successful_ecm_cycles = impl_->observation_age_successful_ecm_cycles;
  state.last_successful_cycle_index = impl_->last_successful_cycle_index;
  state.has_world_chronology = impl_->has_world_chronology;
  state.last_world_cycle_index = impl_->last_world_cycle_index;
  state.last_world_window_end_time_s = impl_->last_world_window_end_time_s;
  state.next_emission_id = impl_->resource_ledger.next_emission_id();
  state.thermal_energy_j = impl_->resource_ledger.thermal_energy_j();
  state.scheduling_rng_state = impl_->resource_ledger.SerializeSchedulingRng();
  state.tie_break_rng_state = impl_->resource_ledger.SerializeTieBreakRng();
  state.deception_rng_state = impl_->deception_mgr.SerializeDeceptionRng();
  state.deception_states = impl_->deception_mgr.CaptureStates();
  return state;
}

bool EcmSession::RestoreRuntimeState(const EcmRuntimeState& state) {
  // 1. 归属权与 schema 检查
  if (state.owner_identity != impl_.get() ||
      state.schema_version != kRuntimeStateSchemaVersion ||
      !EcmConfigValidator::IsValidConfig(state.active_config) ||
      !std::isfinite(state.thermal_energy_j) ||
      !std::isfinite(state.last_world_window_end_time_s) ||
      (state.has_world_chronology && state.last_world_window_end_time_s <= 0.0) ||
      (!state.has_world_chronology && state.last_world_cycle_index != 0U) ||
      state.thermal_energy_j < 0.0 ||
      state.thermal_energy_j > state.active_config.thermal_capacity_j ||
      state.next_emission_id == 0U ||
      !EcmConfigValidator::IsSnapshotInternallyConsistent(state)) {
    return false;
  }

  // 2. 解析所有 RNG 流到本地候选对象（失败前不修改 impl_）
  EcmResourceLedger candidate_ledger(0U);  // 临时 seed，将被反序列化覆盖
  if (!candidate_ledger.DeserializeSchedulingRng(state.scheduling_rng_state)) {
    return false;
  }
  if (!candidate_ledger.DeserializeTieBreakRng(state.tie_break_rng_state)) {
    return false;
  }
  candidate_ledger.SetNextEmissionId(state.next_emission_id);
  candidate_ledger.SetThermalEnergy(state.thermal_energy_j);

  DeceptionEngagementManager candidate_dem(0U);  // 临时 seed，将被反序列化覆盖
  if (!candidate_dem.DeserializeDeceptionRng(state.deception_rng_state)) {
    return false;
  }
  candidate_dem.RestoreStates(state.deception_states);

  // 3. 原子提交到 impl_
  impl_->active_config = state.active_config;
  impl_->has_successful_cycle = state.has_successful_cycle;
  impl_->has_last_sensor_frame = state.has_last_sensor_frame;
  impl_->last_sensor_frame = state.last_sensor_frame;
  impl_->observation_age_successful_ecm_cycles = state.observation_age_successful_ecm_cycles;
  impl_->last_successful_cycle_index = state.last_successful_cycle_index;
  impl_->has_world_chronology = state.has_world_chronology;
  impl_->last_world_cycle_index = state.last_world_cycle_index;
  impl_->last_world_window_end_time_s = state.last_world_window_end_time_s;
  impl_->resource_ledger = candidate_ledger;
  impl_->deception_mgr = candidate_dem;
  return true;
}

}  // namespace session
}  // namespace electronic_countermeasure
