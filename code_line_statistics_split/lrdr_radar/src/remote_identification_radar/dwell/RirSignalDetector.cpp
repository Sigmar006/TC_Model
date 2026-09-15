#include "remote_identification_radar/dwell/RirSignalDetector.h"

#include <cmath>

namespace remote_identification_radar {
namespace dwell {
namespace {

oneq::common::radar::SwerlingModel ToCommonSwerling(internal::RirSwerlingModel model) {
  switch (model) {
    case internal::RirSwerlingModel::kSwerling1:
      return oneq::common::radar::SwerlingModel::kSwerling1;
    case internal::RirSwerlingModel::kSwerling2:
      return oneq::common::radar::SwerlingModel::kSwerling2;
    case internal::RirSwerlingModel::kSwerling3:
      return oneq::common::radar::SwerlingModel::kSwerling3;
    case internal::RirSwerlingModel::kSwerling4:
      return oneq::common::radar::SwerlingModel::kSwerling4;
    case internal::RirSwerlingModel::kSwerling0:
    default:
      return oneq::common::radar::SwerlingModel::kSwerling0;
  }
}

oneq::common::radar::StatisticalCfarPolicy ToPolicy(const RirDetectorConfig& config) {
  oneq::common::radar::StatisticalCfarPolicy policy;
  policy.cfar_pfa = config.detection_policy.cfar_pfa;
  policy.min_snr_db = config.detection_policy.min_snr_db;
  policy.min_detection_margin_db = config.min_detection_margin_db;
  return policy;
}

RirDetectionResult FromCommon(const oneq::common::radar::StatisticalCfarResult& in) {
  RirDetectionResult out;
  out.echo_power_dbw = in.echo_power_dbw;
  out.snr_db = in.snr_db;
  out.detection_prob = in.detection_prob;
  out.detected = in.detected;
  return out;
}

}  // namespace

RirSignalDetector::RirSignalDetector(const RirDetectorConfig& config)
    : config_(config),
      cfar_(internal::RirRadarEquations::ComputeThermalNoisePower_W(config.transmitter,
                                                                    config.receiver),
            ToPolicy(config)) {}

void RirSignalDetector::UpdateConfig(const RirDetectorConfig& config) {
  config_ = config;
  cfar_.Update(internal::RirRadarEquations::ComputeThermalNoisePower_W(config_.transmitter,
                                                                       config_.receiver),
               ToPolicy(config_));
}

RirDetectionResult RirSignalDetector::Detect(const RirTargetReturn& target,
                                             const RirEnvironmentNoise& env,
                                             float one_way_antenna_gain_db, int pulse_count) {
  if (std::isnan(one_way_antenna_gain_db)) {
    one_way_antenna_gain_db = config_.antenna.main_beam_gain_db;
  }

  float echo_power_dbw = internal::RirRadarEquations::ComputeEchoPowerWithGain_dBW(
      config_.transmitter, one_way_antenna_gain_db, target.rcs_m2, target.range_m,
      env.propagation_loss_db);
  // 脉压能量增益 max(1, B·τ)：匹配滤波后单脉冲 SNR = 峰值方程回波 × B·τ /
  // (kT·B·NF + 杂波 + 干扰)，与检测单元路径（common DetectionCellResolver）
  // 同口径；相对 common 参考脉宽基准的修正量为 +10·log10(B·13µs)（与 τ 无关）。
  echo_power_dbw += internal::RirRadarEquations::ComputePulseCompressionGainDb(
      config_.transmitter);
  echo_power_dbw -= config_.receiver.receive_loss_db;

  return FromCommon(cfar_.DetectFromEchoBudget(echo_power_dbw, env.clutter_noise_w, env.jam_noise_w,
                                               ToCommonSwerling(target.swerling_type),
                                               pulse_count));
}

RirDetectionResult RirSignalDetector::DetectResolvedCell(const RirTargetReturn& target,
                                                         const RirDetectionCellResult& cell) {
  return FromCommon(cfar_.DetectResolvedCell(
      static_cast<float>(cell.echo_power_w),
      static_cast<float>(cell.processed_single_pulse_sinr_db),
      ToCommonSwerling(target.swerling_type), static_cast<int>(cell.effective_pulse_count)));
}

void RirSignalDetector::SetRandomSeed(unsigned int seed) { cfar_.SetRandomSeed(seed); }

}  // namespace dwell
}  // namespace remote_identification_radar
