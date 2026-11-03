#include "sbirs_sensor/foundation/SbirsErrorModel.h"

#include <cmath>
#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace foundation {
namespace {

using oneq::common::numerics::kPi;
using oneq::common::numerics::DegToRad;

}  // namespace

double SbirsRandomSource::NextUniform() {
  // xorshift32（Marsaglia）。state 永远非零（构造/Restore 时夹紧）。
  std::uint32_t x = state_;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  state_ = x;
  // 映射到 [0,1)，避免返回正好 1.0 破坏 Box-Muller 的 log。
  return (static_cast<double>(state_) / 4294967296.0);
}

double SbirsRandomSource::NextStandardNormal() {
  // Box-Muller 变换。u2 保证 > 0，避免 log(0)。
  const double u1 = NextUniform();
  double u2 = NextUniform();
  if (u2 <= 0.0) {
    u2 = 1.0e-12;
  }
  const double radius = std::sqrt(-2.0 * std::log(u2));
  const double angle = 2.0 * kPi * u1;
  // 仅消费一个正态样本，丢弃另一个以保持确定性推进。
  return radius * std::cos(angle);
}

double RefractionErrorDeg(double range_m, float elevation_deg) {
  if (range_m <= 0.0) {
    return 0.0;
  }
  const double cos_beta = std::cos(DegToRad(static_cast<double>(elevation_deg)));
  // 仰角接近 0° 时 cosβ→1；俯仰角在天基 LOS 下通常较大，cos 不会发散。
  // 1.5e-6 为红外波段折射角系数（design 2.10）。
  const double denominator = range_m * std::fabs(cos_beta);
  if (denominator <= 0.0) {
    return 0.0;
  }
  return 1.5e-6 / denominator;
}

double DynamicLagErrorDeg(float relative_angular_rate_deg_per_sec, float detector_bandwidth_hz) {
  if (detector_bandwidth_hz <= 0.0f) {
    return 0.0;
  }
  return static_cast<double>(relative_angular_rate_deg_per_sec) / (2.0 * kPi * detector_bandwidth_hz);
}

double ResolveEffectiveAngularSigmaDeg(const config::SbirsErrorModelConfig& model) {
  const double orbit = static_cast<double>(model.orbit_sigma_deg);
  const double attitude = static_cast<double>(model.attitude_sigma_deg);
  const double fov = static_cast<double>(model.fov_sigma_deg);
  return std::sqrt(orbit * orbit + attitude * attitude + fov * fov);
}

double ResolveLinearSnrSigmaDeg(const config::SbirsErrorModelConfig& model, double snr_db) {
  const double hi_db = static_cast<double>(model.sigma_linear_snr_high_db);
  const double lo_db = static_cast<double>(model.sigma_linear_snr_low_db);
  const double hi_urad = static_cast<double>(model.sigma_linear_high_urad);
  const double lo_urad = static_cast<double>(model.sigma_linear_low_urad);
  // 中译：**SNR↔测角误差的直接线性标定**（2026-11-04 甲方口径）——不走 IFOV/√SNR 的内部
  //   推导，只按 dB 线性插值：SNR 从高锚点降到低锚点，σ 从 σ_high 线性升到 σ_low；
  //   区间外按端点钳位，避免极端值把误差推到无意义量级。外部验证程序可直接用
  //   "该行 SNR ⇒ 应有 σ" 对账，无需了解内部实现。
  // 标识：SNR-误差线性标定（2026-11-04）。
  const double kUradToDeg = 180.0 / kPi * 1.0e-6;
  if (!(hi_db > lo_db) || !(hi_urad > 0.0) || !(lo_urad > 0.0)) {
    return hi_urad > 0.0 ? hi_urad * kUradToDeg : ResolveEffectiveAngularSigmaDeg(model);
  }
  if (!(snr_db > 0.0)) {
    return hi_urad * kUradToDeg;
  }
  double t = (hi_db - snr_db) / (hi_db - lo_db);   // 0 在高锚点，1 在低锚点
  if (t < 0.0) {
    t = 0.0;
  }
  if (t > 1.0) {
    t = 1.0;
  }
  const double urad = hi_urad + (lo_urad - hi_urad) * t;
  return urad * kUradToDeg;
}

double ResolveFrameAngularSigmaDeg(const config::SbirsErrorModelConfig& model, double snr_linear,
                                   double ifov_rad) {
  // 中译：线性标定开关打开时**只**用 SNR 线性表（覆盖内部 RSS 推导），这正是甲方要的
  //   "直接挂钩"口径；关闭时回落到"仪器地板 ⊕ 焦面光子噪声"的物理模型。
  // 标识：线性标定优先（2026-11-04）。
  if (model.use_linear_snr_sigma) {
    const double snr_db = (snr_linear > 0.0) ? 10.0 * std::log10(snr_linear) : 0.0;
    return ResolveLinearSnrSigmaDeg(model, snr_db);
  }

  const double floor_deg = ResolveEffectiveAngularSigmaDeg(model);
  // 中译：**焦面光子噪声项**（2026-11-04）——质心定位的标准极限是 IFOV/(k·√SNR)：
  //   同一像元内光子数越多，质心越准；弱目标自然变差。此前该项完全缺失，测角误差是
  //   常数，与 SNR 无关（实测相关系数 −0.007），"能否到 3 µrad"成了配置写死的巧合。
  // 标识：焦面光子噪声项（2026-11-04）。
  const double gain = static_cast<double>(model.centroid_gain);
  if (!(snr_linear > 0.0) || !(ifov_rad > 0.0) || !(gain > 0.0)) {
    return floor_deg;
  }
  const double photon_rad = ifov_rad / (gain * std::sqrt(snr_linear));
  const double photon_deg = photon_rad * (180.0 / kPi);
  return std::sqrt(floor_deg * floor_deg + photon_deg * photon_deg);
}

SbirsErrorBearing ApplyAngularErrorModel(const config::SbirsErrorModelConfig& model,
                                         SbirsRandomSource* random, float true_azimuth_deg,
                                         float true_elevation_deg, double true_range_m,
                                         float relative_angular_rate_deg_per_sec, double snr_linear,
                                         double ifov_rad) {
  SbirsErrorBearing bearing;
  // 物理分项按 RSS 合成为单一 1-σ；az/el 独立采样，与对角量测协方差一致。
  const double angular_sigma_deg = ResolveFrameAngularSigmaDeg(model, snr_linear, ifov_rad);
  const double azimuth_random_error =
      angular_sigma_deg > 0.0 && random != nullptr
          ? angular_sigma_deg * random->NextStandardNormal()
          : 0.0;
  const double elevation_random_error =
      angular_sigma_deg > 0.0 && random != nullptr
          ? angular_sigma_deg * random->NextStandardNormal()
          : 0.0;

  const double refraction = RefractionErrorDeg(true_range_m, true_elevation_deg);
  const double lag =
      DynamicLagErrorDeg(relative_angular_rate_deg_per_sec, model.detector_bandwidth_hz);

  // 加法合成（design 2.10 式 1111-1113）。
  bearing.azimuth_deg =
      static_cast<float>(static_cast<double>(true_azimuth_deg) + azimuth_random_error +
                         refraction + lag);
  bearing.elevation_deg =
      static_cast<float>(static_cast<double>(true_elevation_deg) + elevation_random_error +
                         refraction + lag);

  // 乘法合成（距离误差）：d_meas = d_true · (1 + Δd_rand)。
  double range_factor = 0.0;
  if (model.range_fraction_sigma > 0.0f && random != nullptr) {
    range_factor = model.range_fraction_sigma * random->NextStandardNormal();
  }
  bearing.range_m = true_range_m * (1.0 + range_factor);
  return bearing;
}

namespace {

/// 角度差折叠到 (−180°, 180°]（方位均值走最短角差，防 0°/360° 环绕错均值）。
double Wrap180Deg(double delta_deg) {
  double wrapped = std::fmod(delta_deg + 180.0, 360.0);
  if (wrapped < 0.0) {
    wrapped += 360.0;
  }
  return wrapped - 180.0;
}

}  // namespace

SbirsFusedBearingResult ApplyAngularErrorModelFused(
    const config::SbirsErrorModelConfig& model, SbirsRandomSource* random, float true_azimuth_deg,
    float true_elevation_deg, double true_range_m, float relative_angular_rate_deg_per_sec,
    int frame_count) {
  SbirsFusedBearingResult result;
  const double sigma_deg = ResolveEffectiveAngularSigmaDeg(model);
  result.frame_sigma_deg = sigma_deg;
  if (frame_count <= 1 || sigma_deg <= 0.0 || random == nullptr) {
    // 单帧/无随机分量：与既有单帧路径逐位一致（确定性偏差照常施加）。
    result.bearing = ApplyAngularErrorModel(model, random, true_azimuth_deg, true_elevation_deg,
                                            true_range_m, relative_angular_rate_deg_per_sec);
    result.fused_sigma_deg = sigma_deg;
    return result;
  }
  // N 帧独立抽样：方位以首帧为基准走最短角差累加，俯仰算术均值；
  // 折射/滞后为确定性公共偏差，叠加一次；距离误差保持单帧口径。
  double azimuth_delta_sum = 0.0;
  double elevation_sum = 0.0;
  for (int frame = 0; frame < frame_count; ++frame) {
    const double az_error = sigma_deg * random->NextStandardNormal();
    const double el_error = sigma_deg * random->NextStandardNormal();
    azimuth_delta_sum += Wrap180Deg(az_error);
    elevation_sum += el_error;
  }
  const double refraction = RefractionErrorDeg(true_range_m, true_elevation_deg);
  const double lag = DynamicLagErrorDeg(relative_angular_rate_deg_per_sec, model.detector_bandwidth_hz);
  result.bearing.azimuth_deg = static_cast<float>(
      static_cast<double>(true_azimuth_deg) + azimuth_delta_sum / frame_count + refraction + lag);
  result.bearing.elevation_deg = static_cast<float>(
      static_cast<double>(true_elevation_deg) + elevation_sum / frame_count + refraction + lag);
  double range_factor = 0.0;
  if (model.range_fraction_sigma > 0.0f && random != nullptr) {
    range_factor = model.range_fraction_sigma * random->NextStandardNormal();
  }
  result.bearing.range_m = true_range_m * (1.0 + range_factor);
  result.fused_sigma_deg = sigma_deg / std::sqrt(static_cast<double>(frame_count));
  return result;
}

}  // namespace foundation
}  // namespace sbirs_sensor
