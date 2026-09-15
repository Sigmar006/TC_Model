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

SbirsErrorBearing ApplyAngularErrorModel(const config::SbirsErrorModelConfig& model,
                                         SbirsRandomSource* random, float true_azimuth_deg,
                                         float true_elevation_deg, double true_range_m,
                                         float relative_angular_rate_deg_per_sec) {
  SbirsErrorBearing bearing;
  // 物理分项按 RSS 合成为单一 1-σ；az/el 独立采样，与对角量测协方差一致。
  const double angular_sigma_deg = ResolveEffectiveAngularSigmaDeg(model);
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
