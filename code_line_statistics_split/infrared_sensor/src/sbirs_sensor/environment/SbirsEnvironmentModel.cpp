#include "sbirs_sensor/environment/SbirsEnvironmentModel.h"

#include <algorithm>
#include <cmath>
#include "common/geometry/EarthOccultation.h"

namespace sbirs_sensor {
namespace environment {

namespace {

/** @brief 壳顶球半径（地球平均半径 + 大气有效壳厚），单位 m。 */
double ShellTopRadiusM() {
  return oneq::common::geometry::kMeanEarthRadiusM + kAtmosphereShellThicknessM;
}

float WeatherAttenuation(config::SbirsWeatherType weather_type) {
  switch (weather_type) {
    case config::SbirsWeatherType::kCloudy:
      return 0.05f;
    case config::SbirsWeatherType::kRain:
      return 0.15f;
    case config::SbirsWeatherType::kFog:
      return 0.20f;
    case config::SbirsWeatherType::kClear:
    default:
      return 0.0f;
  }
}

float SeaAttenuation(config::SbirsSeaState sea_state) {
  switch (sea_state) {
    case config::SbirsSeaState::kMedium:
      return 0.10f;
    case config::SbirsSeaState::kHigh:
      return 0.15f;
    case config::SbirsSeaState::kLow:
    default:
      return 0.05f;
  }
}

}  // namespace

double ComputeShellAirmassFactor(const session::SbirsVector3M& satellite_position_eci_m,
                                 const session::SbirsVector3M& target_position_eci_m) {
  // 段参数化 p(s) = target + s·d（s∈[0,1]），解 |p(s)| = 壳顶半径得穿壳区间。
  const double dx = satellite_position_eci_m.x - target_position_eci_m.x;
  const double dy = satellite_position_eci_m.y - target_position_eci_m.y;
  const double dz = satellite_position_eci_m.z - target_position_eci_m.z;
  const double segment_length = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (!(segment_length > 0.0) || !std::isfinite(segment_length)) {
    return 0.0;
  }
  const double shell_radius = ShellTopRadiusM();
  // |p(s)|² = a·s² + b·s + c；开口向上，壳内 ⇔ 该二次式 < 0（根区间内部）。
  const double a = segment_length * segment_length;
  const double b = 2.0 * (target_position_eci_m.x * dx + target_position_eci_m.y * dy +
                          target_position_eci_m.z * dz);
  const double c = target_position_eci_m.x * target_position_eci_m.x +
                   target_position_eci_m.y * target_position_eci_m.y +
                   target_position_eci_m.z * target_position_eci_m.z - shell_radius * shell_radius;
  double in_shell_fraction = 0.0;  // 段长落在壳内的比例
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant > 0.0) {
    // 目标在壳内（c<0）时两根异号，s_lo 夹到 0、s_hi 取出壳根 → 部分弦；
    // 双端在壳外时两根同号，仅当都落在 [0,1] 内才有穿壳段（横穿大气壳的掠射路径）。
    const double sqrt_disc = std::sqrt(discriminant);
    const double s_lo = std::max(0.0, (-b - sqrt_disc) / (2.0 * a));
    const double s_hi = std::min(1.0, (-b + sqrt_disc) / (2.0 * a));
    if (s_hi > s_lo) {
      in_shell_fraction = s_hi - s_lo;
    }
  }
  // 判别式 ≤ 0：段与壳顶球无实交点（相切或整段在壳外侧）→ 穿壳弦为 0。
  const double chord_m = in_shell_fraction * segment_length;
  const double factor = chord_m / kAtmosphereShellThicknessM;
  return std::max(0.0, std::min(kMaxShellAirmassFactor, factor));
}

float ResolveGeometricTransmittance(const config::SbirsEnvironmentConfig& environment,
                                    const session::SbirsVector3M& satellite_position_eci_m,
                                    const session::SbirsVector3M& target_position_eci_m) {
  const float base = ResolveEffectiveTransmittance(environment);
  const double airmass = ComputeShellAirmassFactor(satellite_position_eci_m, target_position_eci_m);
  return static_cast<float>(std::pow(static_cast<double>(base), airmass));
}

float ResolveWeatherAttenuation(const config::SbirsEnvironmentConfig& environment) {
  const float weather = WeatherAttenuation(environment.weather_type);
  const float sea = SeaAttenuation(environment.sea_state);
  const float humidity =
      std::max(0.0f, environment.relative_humidity_percent - 50.0f) / 20.0f * 0.05f;
  const float visibility =
      environment.visibility_km < 1.0f
          ? 0.20f
          : (environment.visibility_km < 5.0f ? 0.10f
                                              : (environment.visibility_km < 10.0f ? 0.05f : 0.0f));
  const float temperature_relief =
      std::max(0.0f, environment.temperature_c - 15.0f) / 10.0f * 0.02f;
  // 2.9 气象交互项 k_j·A_p·A_q：湿度×能见度、雨×湿度两项联合影响。
  // 仅当对应权重 > 0 时启用，默认权重 0 保持向后兼容。
  float interaction = 0.0f;
  if (environment.humidity_visibility_interaction_weight > 0.0f) {
    // 湿度与能见度均为衰减贡献时（即都 > 0），交互项放大联合衰减。
    const float humidity_contrib = std::max(0.0f, humidity);
    const float visibility_contrib = std::max(0.0f, visibility);
    interaction += environment.humidity_visibility_interaction_weight * humidity_contrib *
                   visibility_contrib;
  }
  if (environment.rain_humidity_interaction_weight > 0.0f &&
      environment.weather_type == config::SbirsWeatherType::kRain) {
    const float rain_contrib = weather;
    const float humidity_contrib = std::max(0.0f, humidity);
    interaction += environment.rain_humidity_interaction_weight * rain_contrib * humidity_contrib;
  }
  const float attenuation = 0.25f * weather + 0.15f * sea + 0.25f * humidity + 0.25f * visibility -
                            0.10f * temperature_relief + interaction;
  return std::max(0.0f, std::min(1.0f, attenuation));
}

float ResolveEffectiveTransmittance(const config::SbirsEnvironmentConfig& environment) {
  const float base = std::max(0.0f, std::min(1.0f, environment.base_atmospheric_transmittance));
  return base * (1.0f - ResolveWeatherAttenuation(environment));
}

}  // namespace environment
}  // namespace sbirs_sensor
