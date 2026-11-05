/**
 * ## 过程：指向扰动（共模姿态 + 逐通道指向 + 确定性振动）（红外处理链 · 第 11 步）
 * - 上游：`policy.pointing_disturbance`（各 σ、相关时间、振动振幅/频率、基础种子），
 *   经 pipeline 的 `DisturbanceParameters` 逐周期传入；dt 为本周期步长。
 * - 本步：三条通道各自演化：
 *   · 共模姿态：影响全部视场的 ORN/AR(1) 过程，α = e^(−dt/τ)、新息尺度 σ·√(1−α²)，
 *     使平稳方差恒为 σ²（不是每步叠加等量白噪声）；
 *   · 逐通道指向：另一条独立 AR(1)，只作用于对应 NFOV 通道；
 *   · 确定性振动：A·sin(2πft + φ)，相位由派生种子给出，与随机项相加。
 * - 下游：pipeline 合成为实际指向角（指令角 + 共模 + 通道扰动），进视场门、指向残差与第 76 项
 *   接口记录；扰动只改传感器看到的指向，不改目标真值角。
 * - 关键参数：`common_attitude_sigma_deg`/`common_attitude_correlation_time_s`、
 *   `channel_pointing_sigma_deg`/`channel_pointing_correlation_time_s`、
 *   `channel_vibration_amplitude_deg`/`channel_vibration_frequency_hz`、`random_seed`；
 *   默认全 0（等价于理想指向）。
 * - 对应验收项：第 9 项 红外载荷角误差功能测试（指向扰动的物理来源之一）；第 12 项 宽视场
 *   扫描探测、第 13 项 窄视场跟踪探测（指向残差）；第 76 项 红外宽场候选与窄场调度交互接口测试。
 */

#include "sbirs_sensor/pipeline/SbirsPointingDisturbance.h"

#include <cmath>
#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

using oneq::common::numerics::kPi;

// 过程：快照合法性 = 两轴角有限且随机状态非零（xorshift 状态为 0 会永远输出 0）。
bool IsFinite(const SbirsGaussMarkovSnapshot& snapshot) {
  return std::isfinite(snapshot.azimuth_deg) && std::isfinite(snapshot.elevation_deg) &&
         snapshot.random_state != 0U;
}

}  // namespace

// 过程：种子派生 —— 基础种子 + 黄金比例常数 0x9e3779b9×流编号，再经两轮整数散列，
//       使共模与各通道随机流互不相关；结果为 0 时兜底为 1（xorshift 不允许零状态）。
std::uint32_t SbirsPointingDisturbance::DeriveSeed(std::uint32_t base_seed,
                                                   std::uint32_t stream_id) {
  std::uint32_t value =
      (base_seed == 0U ? 1U : base_seed) + UINT32_C(0x9e3779b9) * (stream_id + 1U);
  value ^= value >> 16;
  value *= 0x7feb352dU;
  value ^= value >> 15;
  value *= 0x846ca68bU;
  value ^= value >> 16;
  return value == 0U ? 1U : value;
}

// 过程：振动相位 = 派生种子/2³² × 2π，落在 [0, 2π)；同种子重放时相位逐位可复现。
double SbirsPointingDisturbance::DerivePhaseRad(std::uint32_t base_seed,
                                                 std::uint32_t stream_id) {
  return 2.0 * kPi * static_cast<double>(DeriveSeed(base_seed, stream_id)) / 4294967296.0;
}

// 过程：一条共模流（流 0）+ 每通道一条独立流（流 1, 2, …），通道数至少为 1。
SbirsPointingDisturbance::SbirsPointingDisturbance(int channel_count, std::uint32_t seed)
    : base_seed_(seed == 0U ? 1U : seed), common_(DeriveSeed(base_seed_, 0U)) {
  const int normalized_count = channel_count < 1 ? 1 : channel_count;
  channels_.reserve(static_cast<std::size_t>(normalized_count));
  for (int channel_id = 0; channel_id < normalized_count; ++channel_id) {
    channels_.push_back(ChannelRuntime(
        DeriveSeed(base_seed_, 1U + static_cast<std::uint32_t>(channel_id))));
  }
}

// 过程：参数校验 —— σ ≥ 0、相关时间 > 0（否则 α 无定义）、振幅为 0 时频率可省略，
//       振幅非 0 时频率必须为正。
bool SbirsPointingDisturbance::ValidateParameters(
    const SbirsPointingDisturbanceParameters& parameters) {
  return std::isfinite(parameters.common_attitude_sigma_deg) &&
         parameters.common_attitude_sigma_deg >= 0.0 &&
         std::isfinite(parameters.common_attitude_correlation_time_s) &&
         parameters.common_attitude_correlation_time_s > 0.0 &&
         std::isfinite(parameters.channel_pointing_sigma_deg) &&
         parameters.channel_pointing_sigma_deg >= 0.0 &&
         std::isfinite(parameters.channel_pointing_correlation_time_s) &&
         parameters.channel_pointing_correlation_time_s > 0.0 &&
         std::isfinite(parameters.channel_vibration_amplitude_deg) &&
         parameters.channel_vibration_amplitude_deg >= 0.0 &&
         std::isfinite(parameters.channel_vibration_frequency_hz) &&
         parameters.channel_vibration_frequency_hz >= 0.0 &&
         (parameters.channel_vibration_amplitude_deg == 0.0 ||
          parameters.channel_vibration_frequency_hz > 0.0);
}

// 过程：一阶自回归 AR(1)/Ornstein-Uhlenbeck 精确离散：x ← α·x + σ√(1−α²)·N(0,1)，
//       α = e^(−dt/τ)，使平稳方差恒为 σ²、相关时间恰为 τ。
void SbirsPointingDisturbance::AdvanceGaussMarkov(double dt_sec, double sigma_deg,
                                                   double correlation_time_s,
                                                   GaussMarkovRuntime* runtime) {
  if (sigma_deg == 0.0) {
    runtime->azimuth_deg = 0.0;
    runtime->elevation_deg = 0.0;
    return;
  }
  // 过程：α = e^(−dt/τ) 是衰减系数（τ 为相关时间）；上方 σ=0 的分支已直接归零，
  //       且不消耗随机数 —— 关闭扰动不改变其余随机流的推进。
  const double alpha = std::exp(-dt_sec / correlation_time_s);
  const double innovation_scale = sigma_deg * std::sqrt(1.0 - alpha * alpha);
  runtime->azimuth_deg =
      alpha * runtime->azimuth_deg + innovation_scale * runtime->random.NextStandardNormal();
  runtime->elevation_deg =
      alpha * runtime->elevation_deg + innovation_scale * runtime->random.NextStandardNormal();
}

// 过程：先在副本上推进再整体提交 —— 任一参数非法时直接返回 false，已有状态不受影响。
bool SbirsPointingDisturbance::Advance(
    double dt_sec, const SbirsPointingDisturbanceParameters& parameters) {
  if (!std::isfinite(dt_sec) || dt_sec <= 0.0 || !ValidateParameters(parameters)) {
    return false;
  }
  GaussMarkovRuntime next_common = common_;
  std::vector<ChannelRuntime> next_channels = channels_;
  AdvanceGaussMarkov(dt_sec, parameters.common_attitude_sigma_deg,
                     parameters.common_attitude_correlation_time_s, &next_common);
  for (ChannelRuntime& channel : next_channels) {
    AdvanceGaussMarkov(dt_sec, parameters.channel_pointing_sigma_deg,
                       parameters.channel_pointing_correlation_time_s, &channel.gauss_markov);
    // 过程：通道累计运行时间只用于振动相位（GM 项是马尔可夫过程，与时间原点无关）。
    channel.elapsed_time_s += dt_sec;
  }
  common_ = next_common;
  channels_ = next_channels;
  return true;
}

// 过程：采样 = 共模 GM 角 +（通道 GM 角 + 确定性正弦振动）；振动两轴各用一条派生相位，
//       避免方位/俯仰同相（那样振动会退化成沿一条直线的往复摆动）。
bool SbirsPointingDisturbance::Sample(
    int channel_id, const SbirsPointingDisturbanceParameters& parameters,
    SbirsPointingDisturbanceSample* sample) const {
  if (sample == nullptr || channel_id < 0 ||
      static_cast<std::size_t>(channel_id) >= channels_.size() || !ValidateParameters(parameters)) {
    return false;
  }
  const ChannelRuntime& channel = channels_[static_cast<std::size_t>(channel_id)];
  const double angular_frequency = 2.0 * kPi * parameters.channel_vibration_frequency_hz;
  const std::uint32_t stream_base = UINT32_C(0x100) +
                                    2U * static_cast<std::uint32_t>(channel_id);
  SbirsPointingDisturbanceSample next;
  next.common.azimuth_deg = common_.azimuth_deg;
  next.common.elevation_deg = common_.elevation_deg;
  next.channel.azimuth_deg = channel.gauss_markov.azimuth_deg +
                             parameters.channel_vibration_amplitude_deg *
                                 std::sin(angular_frequency * channel.elapsed_time_s +
                                          DerivePhaseRad(base_seed_, stream_base));
  next.channel.elevation_deg = channel.gauss_markov.elevation_deg +
                               parameters.channel_vibration_amplitude_deg *
                                   std::sin(angular_frequency * channel.elapsed_time_s +
                                            DerivePhaseRad(base_seed_, stream_base + 1U));
  *sample = next;
  return true;
}

// 过程：快照含基础种子与每条随机流的内部状态，恢复后能接着产生同一序列。
SbirsPointingDisturbanceSnapshot SbirsPointingDisturbance::Capture() const {
  SbirsPointingDisturbanceSnapshot snapshot;
  snapshot.base_seed = base_seed_;
  snapshot.common.azimuth_deg = common_.azimuth_deg;
  snapshot.common.elevation_deg = common_.elevation_deg;
  snapshot.common.random_state = common_.random.Capture();
  snapshot.channels.reserve(channels_.size());
  for (const ChannelRuntime& channel : channels_) {
    SbirsChannelDisturbanceSnapshot channel_snapshot;
    channel_snapshot.gauss_markov.azimuth_deg = channel.gauss_markov.azimuth_deg;
    channel_snapshot.gauss_markov.elevation_deg = channel.gauss_markov.elevation_deg;
    channel_snapshot.gauss_markov.random_state = channel.gauss_markov.random.Capture();
    channel_snapshot.elapsed_time_s = channel.elapsed_time_s;
    snapshot.channels.push_back(channel_snapshot);
  }
  return snapshot;
}

// 过程：恢复校验 —— 种子非零、通道数一致、共模/通道角有限、累计时间非负；通道数变化
//       说明配置已不同，不可续跑，整体拒绝。
bool SbirsPointingDisturbance::Restore(const SbirsPointingDisturbanceSnapshot& snapshot) {
  if (snapshot.base_seed == 0U || snapshot.channels.size() != channels_.size() ||
      !IsFinite(snapshot.common)) {
    return false;
  }
  GaussMarkovRuntime next_common(DeriveSeed(snapshot.base_seed, 0U));
  next_common.azimuth_deg = snapshot.common.azimuth_deg;
  next_common.elevation_deg = snapshot.common.elevation_deg;
  next_common.random.Restore(snapshot.common.random_state);
  std::vector<ChannelRuntime> next_channels;
  next_channels.reserve(snapshot.channels.size());
  for (std::size_t i = 0; i < snapshot.channels.size(); ++i) {
    const SbirsChannelDisturbanceSnapshot& source = snapshot.channels[i];
    if (!IsFinite(source.gauss_markov) || !std::isfinite(source.elapsed_time_s) ||
        source.elapsed_time_s < 0.0) {
      return false;
    }
    ChannelRuntime destination(
        DeriveSeed(snapshot.base_seed, 1U + static_cast<std::uint32_t>(i)));
    destination.gauss_markov.azimuth_deg = source.gauss_markov.azimuth_deg;
    destination.gauss_markov.elevation_deg = source.gauss_markov.elevation_deg;
    destination.gauss_markov.random.Restore(source.gauss_markov.random_state);
    destination.elapsed_time_s = source.elapsed_time_s;
    next_channels.push_back(destination);
  }
  base_seed_ = snapshot.base_seed;
  common_ = next_common;
  channels_ = next_channels;
  return true;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
