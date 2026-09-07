/**
 * @file ecm_session_consumer.cpp
 * @brief 验证安装后的 ECM、ESR adapter 与公共 RF 发射事实可被 C++11 消费者使用。
 */

#include "1q/electronic_countermeasure/EcmEsrAdapter.h"
#include "1q/electronic_countermeasure/EcmSession.h"

namespace ecm = electronic_countermeasure;
namespace esr_session = electronic_surveillance_radar::session;

int main() {
  esr_session::EmitterHypothesis hypothesis;
  hypothesis.hypothesis_id = 17U;
  hypothesis.estimated_center_frequency_hz = 9.3e9;
  hypothesis.estimated_bandwidth_hz = 5.0e6;
  hypothesis.estimated_pri_s = 1.0e-3;
  hypothesis.estimated_pulse_width_s = 1.0e-6;
  hypothesis.center_frequency_std_hz = 1.0e5;
  hypothesis.bandwidth_std_hz = 1.0e5;
  hypothesis.bearing_std_deg = 2.0f;
  hypothesis.confidence = 0.9f;
  hypothesis.threat_level = esr_session::EsrThreatLevel::kHigh;

  esr_session::EmitterHypothesisList hypotheses;
  hypotheses.push_back(hypothesis);
  ecm::session::EcmSensorObservationFrame frame;
  if (!ecm::session::TryBuildEcmSensorObservationFrame(hypotheses, 400U, &frame)) {
    return 1;
  }

  ecm::config::EcmSessionConfig config;
  config.channel_count = 1U;
  config.maximum_total_transmit_power_w = 500.0;
  config.maximum_channel_transmit_power_w = 500.0;
  ecm::session::EcmSession session = ecm::session::EcmSession::Create(config);

  ecm::session::EcmCycleInput input;
  input.cycle_index = 5U;
  input.cycle_start_time_s = 4.0;
  input.input_mode = ecm::EcmInputMode::kSensorDriven;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.has_sensor_observation_frame = true;
  input.sensor_observation_frame = frame;
  const ecm::session::EcmCycleResult result = session.StepWithResult(input);
  if (result.status != ecm::session::EcmCycleStatus::kExecuted ||
      result.source_esr_batch_id != 400U ||
      result.emission_frame.emissions.empty()) {
    return 2;
  }
  return 0;
}
