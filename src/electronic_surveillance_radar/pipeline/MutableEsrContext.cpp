#include "electronic_surveillance_radar/pipeline/MutableEsrContext.h"

namespace electronic_surveillance_radar {
namespace pipeline {

void MutableEsrContext::BeginCycle(const session::EsrCycleInput& input,
                                   const session::EsrEnvironmentSnapshot& environment_snapshot,
                                   const extension::InterceptPipelineConfig& pipeline_config,
                                   const extension::InterceptRuntimeConfig& runtime_config) {
  cycle_input_ = &input;
  cycle_index_ = input.cycle_index;
  cycle_start_time_s_ = input.cycle_start_time_s;
  dt_sec_ = input.dt_sec;
  platform_position_ecef_m_ = input.platform_position_ecef_m;
  platform_velocity_ecef_mps_ = input.platform_velocity_ecef_mps;
  platform_attitude_deg_ = input.platform_attitude_deg;
  rf_emissions_ = input.rf_emissions;
  environment_snapshot_ = environment_snapshot;
  pipeline_config_ = pipeline_config;
  runtime_config_ = runtime_config;
}

std::uint32_t MutableEsrContext::GetCycleIndex() const { return cycle_index_; }

const session::EsrCycleInput& MutableEsrContext::GetCycleInput() const { return *cycle_input_; }

double MutableEsrContext::GetCycleStartTimeSec() const { return cycle_start_time_s_; }

float MutableEsrContext::GetCycleDeltaTimeSec() const { return dt_sec_; }

const oneq::coordinate::EulerAnglesDeg& MutableEsrContext::GetPlatformAttitude() const {
  return platform_attitude_deg_;
}

const oneq::coordinate::EcefPositionM& MutableEsrContext::GetPlatformPositionEcefM() const {
  return platform_position_ecef_m_;
}

const oneq::coordinate::EcefVelocityMps& MutableEsrContext::GetPlatformVelocityEcefMps() const {
  return platform_velocity_ecef_mps_;
}

const oneq::electromagnetics::RfEmissionFrame& MutableEsrContext::GetRfEmissions() const {
  return rf_emissions_;
}

const session::EsrEnvironmentSnapshot& MutableEsrContext::GetEnvironmentSnapshot() const {
  return environment_snapshot_;
}

const extension::InterceptPipelineConfig& MutableEsrContext::GetPipelineConfig() const {
  return pipeline_config_;
}

const extension::InterceptRuntimeConfig& MutableEsrContext::GetRuntimeConfig() const {
  return runtime_config_;
}

}  // namespace pipeline

}  // namespace electronic_surveillance_radar
