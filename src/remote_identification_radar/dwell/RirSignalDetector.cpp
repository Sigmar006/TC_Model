#include "remote_identification_radar/dwell/RirSignalDetector.h"

#include <cmath>

/**
 * ## 过程：CFAR 检测判决（LRDR 探测链 · 第 7 步）
 * - 上游：检测单元单脉冲 SINR 与分项功率（第 6 步），或回退路径的回波功率 + 杂波/干扰
 *   等效噪声；目标 Swerling 起伏型；积累脉冲数 N。
 * - 本步：按检测策略执行判决——P_fa（cfar_pfa，默认 1e-6）→ CFAR 门限 → Swerling
 *   检测概率 P_d(SINR, P_fa, N)（Marcum Q，见第 8 步）→ 蒙特卡洛随机抽签（u < P_d 判
 *   检测）→ 最小 SNR 余量门（min_detection_margin_db，默认 −2 dB）与硬下限（min_snr_db，
 *   默认 −10 dB）。热噪声功率在构造/更新配置时按 kT₀·B·F 预置，硬件热更新后自动重算。
 * - 下游：RirController（detected 决定目标是否成量测，snr_db 进量测误差模型与准入门）、
 *   验收第 36 项检出结果与 detection_cell 事件的 Pd 字段。
 * - 关键参数：RirDetectorConfig.detection_policy.{cfar_pfa,min_snr_db}、
 *   min_detection_margin_db(−2 dB)、pulse_count(10 ← policy.detection.pulse_count)、
 *   hardware.transmitter.bandwidth_hz、hardware.receiver.noise_figure_db；随机种子取
 *   policy.detection.random_seed（仅在真正变化时重设）。
 * - 对应验收项：第 36 项恒虚警检测功能测试、第 71 项信号处理与检测判决交互接口。
 */

namespace remote_identification_radar {
namespace dwell {
namespace {

// 过程：起伏型枚举镜像（模块私有 → 公共核），未知取值按非起伏 kSwerling0 处理。
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

// 过程：判决策略装配——P_fa（虚警概率）、SNR 硬下限、判决后最小 SNR 余量门。
oneq::common::radar::StatisticalCfarPolicy ToPolicy(const RirDetectorConfig& config) {
  oneq::common::radar::StatisticalCfarPolicy policy;
  policy.cfar_pfa = config.detection_policy.cfar_pfa;
  policy.min_snr_db = config.detection_policy.min_snr_db;
  policy.min_detection_margin_db = config.min_detection_margin_db;
  return policy;
}

// 过程：结果映射——回波功率 dBW、SNR dB、检测概率 P_d、布尔检测标志。
RirDetectionResult FromCommon(const oneq::common::radar::StatisticalCfarResult& in) {
  RirDetectionResult out;
  out.echo_power_dbw = in.echo_power_dbw;
  out.snr_db = in.snr_db;
  out.detection_prob = in.detection_prob;
  out.detected = in.detected;
  return out;
}

}  // namespace

// 过程：构造即按硬件算热噪声功率 N = kT₀·B·F（W），供回退路径的 SNR 账本使用。
RirSignalDetector::RirSignalDetector(const RirDetectorConfig& config)
    : config_(config),
      cfar_(internal::RirRadarEquations::ComputeThermalNoisePower_W(config.transmitter,
                                                                    config.receiver),
            ToPolicy(config)) {}

// 过程：硬件/策略热更新——重算热噪声功率并刷新判决策略（随机数发生器状态保持不变）。
void RirSignalDetector::UpdateConfig(const RirDetectorConfig& config) {
  config_ = config;
  cfar_.Update(internal::RirRadarEquations::ComputeThermalNoisePower_W(config_.transmitter,
                                                                       config_.receiver),
               ToPolicy(config_));
}

// 过程：回退路径判决（无检测单元）——雷达方程回波功率 + 脉压修正 → CFAR 判决；增益缺省回退主瓣峰值。
RirDetectionResult RirSignalDetector::Detect(const RirTargetReturn& target,
                                             const RirEnvironmentNoise& env,
                                             float one_way_antenna_gain_db, int pulse_count) {
  // 中译：诊断计数（2026-10-20）——与"种子重设次数"配对，用于在宿主诊断日志里判定
  //       CFAR 抽签是否真的在推进；不参与任何数值计算。
  ++decision_count_;
  if (std::isnan(one_way_antenna_gain_db)) {
    one_way_antenna_gain_db = config_.antenna.main_beam_gain_db;
  }

  // 过程：回波功率（dBW）= 雷达方程（单程离轴增益）+ 脉压能量修正 − 接收链路损耗。
  float echo_power_dbw = internal::RirRadarEquations::ComputeEchoPowerWithGain_dBW(
      config_.transmitter, one_way_antenna_gain_db, target.rcs_m2, target.range_m,
      env.propagation_loss_db);
  // common 方程已含 τ/13µs 参考缩放；回退路径先抵消该项再施加一次 B·τ，
  // 与 detection cell 的匹配滤波后单脉冲能量口径一致。
  echo_power_dbw += internal::RirRadarEquations::ComputeFallbackPulseCompressionCorrectionDb(
      config_.transmitter);
  echo_power_dbw -= config_.receiver.receive_loss_db;

  // 过程：CFAR 判决——SINR = P_e/(N_热 + C_杂波 + J_干扰) → P_d(Swerling, N) → 随机抽签 → 最小 SNR 余量门。
  RirDetectionResult result = FromCommon(cfar_.DetectFromEchoBudget(
      echo_power_dbw, env.clutter_noise_w, env.jam_noise_w, ToCommonSwerling(target.swerling_type),
      pulse_count));
  if (!result.detected) {
    ++false_decision_count_;
  }
  return result;
}

// 过程：检测单元路径判决——直接取单脉冲 SINR 与生效积累数 N，不再重算功率账本（口径由第 6 步给定）。
RirDetectionResult RirSignalDetector::DetectResolvedCell(const RirTargetReturn& target,
                                                         const RirDetectionCellResult& cell) {
  ++decision_count_;
  RirDetectionResult result = FromCommon(cfar_.DetectResolvedCell(
      static_cast<float>(cell.echo_power_w),
      static_cast<float>(cell.processed_single_pulse_sinr_db),
      ToCommonSwerling(target.swerling_type), static_cast<int>(cell.effective_pulse_count)));
  if (!result.detected) {
    ++false_decision_count_;
  }
  return result;
}

void RirSignalDetector::SetRandomSeed(unsigned int seed) { cfar_.SetRandomSeed(seed); }

}  // namespace dwell
}  // namespace remote_identification_radar
