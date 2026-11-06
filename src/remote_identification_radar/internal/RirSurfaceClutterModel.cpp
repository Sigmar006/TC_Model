/**
 * @file RirSurfaceClutterModel.cpp
 * @brief 实现逐目标主瓣地杂波最小物理模型。
 */

/**
 * ## 过程：主瓣面杂波模型（LRDR 探测链 · 第 10 步）
 * - 上游：environment.vegetation_cover_profile、hardware.{transmitter,antenna}、目标斜距与
 *   俯仰视线角、平台海拔、周期总传播损耗、接收机热噪声功率。
 * - 本步：逐目标求解主瓣地杂波等效噪声（W）：
 *   ① 擦地角 ψ = clamp(θ_3dB,el/2 − φ, 1°, 89°)（ψ ≤ 0 即主瓣完全离地，杂波为 0）；
 *   ② 波束下沿与地面的交点距离 s = Re_eff·(√(sin²δ + 2h/Re_eff) − sinδ)（Re_eff = 4/3·R_e，
 *      超过目标斜距时按目标斜距取）；
 *   ③ 杂波区面积 A_c = min(R·θ_az·δ_R/cosψ, R²·θ_az·φ_el/sinψ)，距离单元 δ_R = c/(2B)；
 *   ④ σ₀ = 植被档位表值·sinψ/sin10°（一阶折算）→ 杂波 RCS σ_c = σ₀·A_c；
 *   ⑤ 杂波回波代入与目标同一雷达方程（主瓣峰值增益 G₀）+ 脉压增益 max(1, Bτ) → CNR →
 *      经公共单源换算回等效杂波噪声功率。
 * - 下游：RirController::ResolveTargetClutterPowerW → 检测单元 SINR 分母的杂波项 / 回退
 *   检测器的 clutter_noise_w、验收第 34 项杂波处理增益。
 * - 关键参数：σ₀ 档位表（开阔草地 −28 dB … 热带密林 −14 dB，S 波段量级声明值）、
 *   kMinGrazingDeg = 1°、kMaxGrazingDeg = 89°、kReferenceGrazingDeg = 10°、
 *   hardware.transmitter.{frequency_hz,bandwidth_hz,pulse_width_s}、hardware.antenna.*。
 * - 对应验收项：第 34 项杂波信号处理增益功能测试、第 30 项干扰/杂波功率口径、
 *   第 43 项 RCS 实时探测结果（杂噪比对探测距离的影响）。
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
// 圆周率：副瓣全向距离环周长用（避免依赖公共常量的可用性）。
constexpr float kPi = 3.14159265358979323846f;

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

// 过程：杂波求解入口——档位关闭或斜距/热噪/载频/带宽非正时直接返回 0（本周期不产生杂波项）。
float RirSurfaceClutterModel::EvaluateClutterNoiseW(const RirSurfaceClutterInput& input) const {
  const config::RirVegetationCoverProfile profile = input.vegetation_cover_profile;
  if (profile == config::RirVegetationCoverProfile::kDisabled || !(input.range_m > 0.0f) ||
      !(input.thermal_noise_w > 0.0f) || !(input.transmitter.frequency_hz > 0.0f) ||
      !(input.transmitter.bandwidth_hz > 0.0f)) {
    return 0.0f;
  }

  // 过程：波长与有效波束宽度（与回波链/方向图同源：nominal 两级回退 0 → λ/L 物理推导）。
  const float wavelength_m = static_cast<float>(oneq::common::numerics::kLightSpeed) /
                             input.transmitter.frequency_hz;
  const oneq::common::radar::EffectiveBeamwidthDeg beamwidth =
      oneq::common::radar::ResolveEffectiveBeamwidth(
          input.antenna.nominal_az_beamwidth_deg, input.antenna.nominal_el_beamwidth_deg,
          input.antenna.antenna_length_m, input.antenna.antenna_width_m, wavelength_m);
  const float az_beamwidth_rad = DegToRad(beamwidth.az_beamwidth_deg);
  const float el_beamwidth_rad = DegToRad(beamwidth.el_beamwidth_deg);
  if (!(az_beamwidth_rad > 0.0f) || !(el_beamwidth_rad > 0.0f)) {
    return 0.0f;
  }

  // 过程：σ₀ 一阶擦地折算与"杂波回波 → 等效噪声（W）"的公共换算，主瓣/副瓣两路共用，
  //   保证同一几何下的两种照射只在"面积 × 双程增益"上有差别。
  const float re_eff_m = 6371000.0f * (4.0f / 3.0f);
  const float range_cell_m = static_cast<float>(oneq::common::numerics::kLightSpeed) /
                             (2.0f * input.transmitter.bandwidth_hz);
  // 过程：σ₀ 一阶擦地角折算（表值按 10° 声明）——**逐路用各自的擦地角**：
  //   主瓣项用实际擦地角 ψ，副瓣项用地面环俯角 ψ_g（钳到 [1°,89°]）。两路共用同一个
  //   σ₀ 档位表与同一个"面积 → RCS → 回声 → 等效噪声"换算，只是面积/增益/擦地角不同。
  auto to_noise_w = [&](float area_m2, float clutter_range_m, float one_way_gain_db,
                        float grazing_deg) -> float {
    if (!(area_m2 > 0.0f) || !(clutter_range_m > 0.0f)) {
      return 0.0f;
    }
    const float sigma0_db = ResolveSigma0Db(profile) +
        10.0f * std::log10(std::sin(DegToRad(grazing_deg)) /
                           std::sin(DegToRad(kReferenceGrazingDeg)));
    const float clutter_rcs_m2 = std::pow(10.0f, (sigma0_db + 10.0f * std::log10(area_m2)) / 10.0f);
    // 过程：杂波回波走与目标同一雷达方程（逐路给定单程增益），再叠加脉压能量增益
    //   max(1, B·τ)：与检测单元路径同一 B·τ 口径，避免分子/分母双口径。
    const float clutter_echo_dbw =
        RirRadarEquations::ComputeEchoPowerWithGain_dBW(input.transmitter, one_way_gain_db,
                                                       clutter_rcs_m2, clutter_range_m,
                                                       input.propagation_loss_db) +
        RirRadarEquations::ComputePulseCompressionGainDb(input.transmitter);
    const float cnr_db = clutter_echo_dbw - 10.0f * std::log10(input.thermal_noise_w);
    return oneq::common::radar::ComputeEquivalentClutterNoiseW(input.thermal_noise_w, cnr_db);
  };

  // ---- 主瓣项：主瓣下沿擦地才成立（ψ_raw ≤ 0 即主瓣完全离地，该项 0）----
  float main_lobe_w = 0.0f;
  const float raw_grazing_deg = beamwidth.el_beamwidth_deg * 0.5f - input.look_el_deg;
  if (raw_grazing_deg > 0.0f) {
    const float grazing_deg =
        oneq::common::numerics::Clamp(raw_grazing_deg, kMinGrazingDeg, kMaxGrazingDeg);
    const float grazing_rad = DegToRad(grazing_deg);
    // 过程：主瓣下沿的地面照射距离（4/3 地球半径射线-球面交点；超过目标斜距时按目标斜距取）。
    const float sin_delta = std::sin(DegToRad(raw_grazing_deg));
    float clutter_range_m =
        re_eff_m * (std::sqrt(sin_delta * sin_delta +
                              2.0f * std::max(input.platform_altitude_m, 0.0f) / re_eff_m) -
                    sin_delta);
    // 退化几何（例如 h=0 时擦地射线与球面无交点）：只跳过主瓣项，副瓣项仍按自己的
    // 可见性门独立判定，不因主瓣几何坏掉而连带丢失副瓣杂波。
    if (clutter_range_m > 0.0f) {
    if (input.range_m > 0.0f && input.range_m < clutter_range_m) {
      clutter_range_m = input.range_m;
    }
    // 过程：擦地面积取脉冲限制与波束限制的较小者（距离单元 c/(2B)，与测距误差同源）。
    const float pulse_limited_area_m2 =
        clutter_range_m * az_beamwidth_rad * range_cell_m / std::cos(grazing_rad);
    const float beam_limited_area_m2 = clutter_range_m * clutter_range_m * az_beamwidth_rad *
                                       el_beamwidth_rad / std::sin(grazing_rad);
      const float clutter_area_m2 = std::min(pulse_limited_area_m2, beam_limited_area_m2);
      // 过程：**距离一致性门**（2026-11-06 补）——主瓣地面回波落在"主瓣落地足迹"所在的
      //   距离单元里，足迹径向长度约 s·θ_el（500 m 量级），单元深度 δR 只有 5 m。
      //   目标若远离该足迹（例如 200 km 外 0.2° 的低空目标），它所在单元里**没有**地面，
      //   不能把近处地面的回波记到它头上。容差取 max(3·δR, 足迹径向长度)。
      //   之前缺这道门：100 km / 200 km / 500 km 的低仰角目标都会被记上 ~47.6 km 处的
      //   主瓣地面杂波（探针实测 CNR 从 0 直接跳到顶钳位 120 dB）。
      // 标识：主瓣距离一致性门（2026-11-06）。
      const float footprint_radial_m = clutter_range_m * el_beamwidth_rad;
      const float range_tolerance_m = std::max(3.0f * range_cell_m, footprint_radial_m);
      if (std::fabs(input.range_m - clutter_range_m) <= range_tolerance_m) {
        main_lobe_w = to_noise_w(clutter_area_m2, clutter_range_m,
                                 input.antenna.main_beam_gain_db, grazing_deg);
      }
    }
  }

  // ---- 副瓣项（2026-11-06 新增）：全向距离环 × 最大副瓣电平 ----
  // 物理：主瓣离地后副瓣仍照地，环上各点多处于天线方向图的副瓣区；取配置的
  //   `max_sidelobe_level_db`（相对主瓣峰值）作为代表值，双程都走副瓣。
  // 可见性门（关键）：只有目标斜距 ≤ 无线电地平线 d_h = sqrt(2·Re_eff·h) 时，
  //   同距离的地面环才在视线内；超出地平线的地面被地球遮挡，不能算进来——否则
  //   4,000 km 外的弹道目标会挂上一个"同斜距全向地面环"，把 SINR 压死（非物理）。
  float side_lobe_w = 0.0f;
  const float horizon_m = std::sqrt(2.0f * re_eff_m * std::max(input.platform_altitude_m, 1.0f));
  if (input.range_m <= horizon_m) {
    // 过程：环半径（地面距离）= sqrt(R² − h²)，环深 = 距离单元 / cos(俯角)；
    //   俯角 ψ_g = asin(h / R)（目标在该斜距处对应的地面点视角），σ₀ 仍按 1° 擦地折算。
    const float height_m = std::max(input.platform_altitude_m, 0.0f);
    const float ground_radius_m =
        std::sqrt(std::max(input.range_m * input.range_m - height_m * height_m, 0.0f));
    const float depression_rad =
        std::asin(oneq::common::numerics::Clamp(height_m / std::max(input.range_m, 1.0f), 0.0f, 1.0f));
    const float ring_depth_m = range_cell_m / std::max(std::cos(depression_rad), 1.0e-3f);
    const float ring_area_m2 = 2.0f * kPi * ground_radius_m * ring_depth_m;
    const float sidelobe_gain_db =
        input.antenna.main_beam_gain_db + input.antenna.max_sidelobe_level_db;
    // 过程：副瓣环的擦地角 = 环点俯角 ψ_g，与主瓣项同样钳到 [1°, 89°] 后折算 σ₀。
    const float ring_grazing_deg = oneq::common::numerics::Clamp(
        depression_rad * 57.29577951308232f, kMinGrazingDeg, kMaxGrazingDeg);
    side_lobe_w = to_noise_w(ring_area_m2, input.range_m, sidelobe_gain_db, ring_grazing_deg);
  }

  // 过程：两项同属"进入该距离单元的杂波等效噪声"，功率域相加后返回（日志不分项）。
  return main_lobe_w + side_lobe_w;
}

}  // namespace internal
}  // namespace remote_identification_radar
