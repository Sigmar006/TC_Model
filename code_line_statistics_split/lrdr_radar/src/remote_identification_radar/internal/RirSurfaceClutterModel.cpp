/**
 * @file RirSurfaceClutterModel.cpp
 * @brief 实现逐目标主瓣地杂波最小物理模型。
 */

#include "remote_identification_radar/internal/RirSurfaceClutterModel.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/ClampUtils.h"
#include "common/numerics/Constants.h"
#include "common/radar/BeamwidthResolution.h"
#include "common/radar/VegetationClutterModel.h"
#include "remote_identification_radar/internal/RirRadarEquations.h"

namespace remote_identification_radar {
namespace internal {

namespace {

using oneq::common::numerics::DegToRad;

// 擦地角下限：低于该角进入多径干涉区，一阶 σ₀ 模型不再可信，钳到 1°。
constexpr float kMinGrazingDeg = 1.0f;
// 擦地角上限：超过 90° 已越过垂直照射几何，cosψ 变号会使脉冲限制面积失效。
constexpr float kMaxGrazingDeg = 89.0f;
// σ₀ 档位表参考擦地角：表值按该角声明，其他角度按 sinψ 一阶折算。
constexpr float kReferenceGrazingDeg = 10.0f;

// σ₀ 档位表（dB，S 波段 @ 参考擦地角量级声明值，非实测标定）：
// 按覆盖密度分层，林分体积散射强于草地当面散射。
float ResolveSigma0Db(config::RirVegetationCoverProfile profile) {
  switch (profile) {
    case config::RirVegetationCoverProfile::kOpenGrassland:
      return -28.0f;
    case config::RirVegetationCoverProfile::kSparseWoodland:
      return -22.0f;
    case config::RirVegetationCoverProfile::kDeciduousForest:
      return -18.0f;
    case config::RirVegetationCoverProfile::kConiferousForest:
      return -16.0f;
    case config::RirVegetationCoverProfile::kTropicalDense:
      return -14.0f;
    case config::RirVegetationCoverProfile::kDisabled:
    default:
      return 0.0f;
  }
}

}  // namespace

float RirSurfaceClutterModel::EvaluateClutterNoiseW(const RirSurfaceClutterInput& input) const {
  const config::RirVegetationCoverProfile profile = input.vegetation_cover_profile;
  if (profile == config::RirVegetationCoverProfile::kDisabled || !(input.range_m > 0.0f) ||
      !(input.thermal_noise_w > 0.0f) || !(input.transmitter.frequency_hz > 0.0f) ||
      !(input.transmitter.bandwidth_hz > 0.0f)) {
    return 0.0f;
  }

  const float wavelength_m = static_cast<float>(oneq::common::numerics::kLightSpeed) /
                             input.transmitter.frequency_hz;
  // 有效波束宽度与方向图/量测误差同源：nominal 两级回退（0 → λ/L 物理推导）。
  const oneq::common::radar::EffectiveBeamwidthDeg beamwidth =
      oneq::common::radar::ResolveEffectiveBeamwidth(
          input.antenna.nominal_az_beamwidth_deg, input.antenna.nominal_el_beamwidth_deg,
          input.antenna.antenna_length_m, input.antenna.antenna_width_m, wavelength_m);
  const float az_beamwidth_rad = DegToRad(beamwidth.az_beamwidth_deg);
  const float el_beamwidth_rad = DegToRad(beamwidth.el_beamwidth_deg);
  if (!(az_beamwidth_rad > 0.0f) || !(el_beamwidth_rad > 0.0f)) {
    return 0.0f;
  }

  // 主瓣下沿擦地角：目标仰角达到半俯仰波束宽后主瓣完全离地，杂波归零；
  // 上限钳 89° 防超宽波束配置（如 160° 全向档）下 cosψ 变号产生负面积。
  const float raw_grazing_deg = beamwidth.el_beamwidth_deg * 0.5f - input.look_el_deg;
  if (raw_grazing_deg <= 0.0f) {
    return 0.0f;
  }
  const float grazing_deg =
      oneq::common::numerics::Clamp(raw_grazing_deg, kMinGrazingDeg, kMaxGrazingDeg);
  const float grazing_rad = DegToRad(grazing_deg);

  // 杂波区面积（脉压后距离单元 δR = c/(2B)，与测距误差模型同源）：
  // 低擦地走脉冲限制（距离向 × 方位向），高擦地走波束限制，取较小者。
  const float range_cell_m = static_cast<float>(oneq::common::numerics::kLightSpeed) /
                             (2.0f * input.transmitter.bandwidth_hz);
  const float pulse_limited_area_m2 =
      input.range_m * az_beamwidth_rad * range_cell_m / std::cos(grazing_rad);
  const float beam_limited_area_m2 =
      input.range_m * input.range_m * az_beamwidth_rad * el_beamwidth_rad /
      std::sin(grazing_rad);
  const float clutter_area_m2 = std::min(pulse_limited_area_m2, beam_limited_area_m2);

  const float grazing_factor_db =
      10.0f * std::log10(std::sin(grazing_rad) /
                         std::sin(DegToRad(kReferenceGrazingDeg)));
  const float clutter_rcs_db = ResolveSigma0Db(profile) + grazing_factor_db +
                               10.0f * std::log10(clutter_area_m2);
  const float clutter_rcs_m2 = std::pow(10.0f, clutter_rcs_db / 10.0f);

  // 主瓣杂波回波走与目标同一雷达方程（主瓣峰值增益近似），再叠加脉压能量
  // 增益 max(1, B·τ)：杂波与目标经同一匹配滤波，等效杂波噪声按脉压能量
  // 基准折算。相对 common 参考脉宽基准的修正量为 +10·log10(B·13µs)（与 τ
  // 无关），把杂波对齐到与检测单元路径目标分子一致的 B·τ 口径，避免同一
  // SINR 账本内分子/分母双口径。CNR 经统一单源换算回瓦（保留 ±120 dB
  // 相对钳制口径）。
  const float clutter_echo_dbw =
      RirRadarEquations::ComputeEchoPower_dBW(input.transmitter, input.antenna, clutter_rcs_m2,
                                              input.range_m, input.propagation_loss_db) +
      RirRadarEquations::ComputePulseCompressionGainDb(input.transmitter);
  const float cnr_db =
      clutter_echo_dbw - 10.0f * std::log10(input.thermal_noise_w);
  return oneq::common::radar::ComputeEquivalentClutterNoiseW(input.thermal_noise_w, cnr_db);
}

}  // namespace internal
}  // namespace remote_identification_radar
