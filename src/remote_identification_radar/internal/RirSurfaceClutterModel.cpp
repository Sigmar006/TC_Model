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
// 再散射项的最小目标离地高度（m）：低于它目标几乎贴地，"目标→地面"距离趋于 0、
// 三次散射退化为二阶，公式不再适用（也避免除零）。
constexpr float kMinRescatterHeightM = 30.0f;

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
  auto rcs_to_noise_w = [&](float clutter_rcs_m2, float clutter_range_m,
                            float one_way_gain_db) -> float {
    if (!(clutter_rcs_m2 > 0.0f) || !(clutter_range_m > 0.0f)) {
      return 0.0f;
    }
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
  // 过程：面积版换算 = σ₀(擦地角) × 面积 → RCS → 同一路径。
  auto to_noise_w = [&](float area_m2, float clutter_range_m, float one_way_gain_db,
                        float grazing_deg) -> float {
    if (!(area_m2 > 0.0f) || !(clutter_range_m > 0.0f)) {
      return 0.0f;
    }
    const float sigma0_db = ResolveSigma0Db(profile) +
        10.0f * std::log10(std::sin(DegToRad(grazing_deg)) /
                           std::sin(DegToRad(kReferenceGrazingDeg)));
    const float clutter_rcs_m2 =
        std::pow(10.0f, (sigma0_db + 10.0f * std::log10(area_m2)) / 10.0f);
    return rcs_to_noise_w(clutter_rcs_m2, clutter_range_m, one_way_gain_db);
  };

  // ---- 主瓣项（2026-11-06 改为严格几何）：波束是否真的照到"目标那个距离处"的地面 ----
  // 旧写法用"下沿擦地角"反算一个**落地距离**，再把它当成目标处的杂波——近轴近似在
  // 接近切线时会给出假交点（δ=0.32° 算出 38 km，而解析几何说这条射线根本不落地），
  // 于是"目标在 5,000 km 外、仰角 0.1°"也会被记上一份近距地面杂波。
  // 现在按精确三角形几何逐点判定：目标斜距 R 处的地面点俯角 ψ_g 只有满足
  //   R ≤ 地平线斜距 且 ψ_g ≤ θ_el/2 − φ（波束下沿压到该点以下）
  // 时，才认为该单元有主瓣地杂波；面积/σ₀ 都在该 R 与 ψ_g 上求。
  // 标识：主瓣严格几何（2026-11-06）。
  float main_lobe_w = 0.0f;
  {
    const float height_m = std::max(input.platform_altitude_m, 0.0f);
    const float radius_platform_m = re_eff_m + height_m;
    // 地平线斜距：视线与球面相切处，d_h = √((Re+h)² − Re²)（≈ √(2·Re·h)）。
    const float horizon_m = std::sqrt(std::max(
        radius_platform_m * radius_platform_m - re_eff_m * re_eff_m, 0.0f));
    // 波束能覆盖的最大俯角 = 下沿相对水平线的下压量 = θ_el/2 − φ。
    const float max_covered_depression_deg =
        0.5f * beamwidth.el_beamwidth_deg - input.look_el_deg;
    if (max_covered_depression_deg > 0.0f && input.range_m > height_m &&
        input.range_m <= horizon_m) {
      // 三角形 O-A-B：A=平台、B=地面点、|AB|=R、|OA|=Re+h、|OB|=Re。
      //   cos∠OAB = (a² + R² − Re²) / (2·a·R)，地面点俯角 ψ_g = 90° − ∠OAB。
      const float r = input.range_m;
      const float cos_angle =
          (radius_platform_m * radius_platform_m + r * r - re_eff_m * re_eff_m) /
          (2.0f * radius_platform_m * r);
      const float angle_rad = std::acos(oneq::common::numerics::Clamp(cos_angle, -1.0f, 1.0f));
      const float depression_deg = 90.0f - angle_rad * 57.29577951308232f;
      if (depression_deg > 0.0f && depression_deg <= max_covered_depression_deg) {
        const float grazing_deg =
            oneq::common::numerics::Clamp(depression_deg, kMinGrazingDeg, kMaxGrazingDeg);
        const float grazing_rad = DegToRad(grazing_deg);
        // 过程：擦地面积仍取脉冲限制与波束限制的较小者（距离单元 c/(2B)），只是
        //   距离与擦地角都取目标处的值（不再用下沿落地距离替代）。
        const float pulse_limited_area_m2 =
            r * az_beamwidth_rad * range_cell_m / std::cos(grazing_rad);
        const float beam_limited_area_m2 =
            r * r * az_beamwidth_rad * el_beamwidth_rad / std::sin(grazing_rad);
        const float clutter_area_m2 = std::min(pulse_limited_area_m2, beam_limited_area_m2);
        main_lobe_w = to_noise_w(clutter_area_m2, r, input.antenna.main_beam_gain_db, grazing_deg);
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

  // ---- 再散射项（2026-11-07 甲方要求）：雷达 → 目标 → 地面 → 雷达（三次散射）----
  // 物理：目标把入射波再散射到正下方的地面补片，补片按面杂波再回雷达。
  //   等效 RCS：σ_eff = σ_t · σ_c · R² / (4π · h_t² · R_g²)
  //     σ_c = σ₀(ψ_g,proj)·A_patch（补片=雷达分辨率单元），h_t=目标离地高度，
  //     R_g=目标地面投影的水平距离（补片到雷达的距离），R=目标斜距。
  //   可见性（关键）：投影点必须仍在雷达视线内（R_g ≤ 地平线），否则那条回波被地球挡住
  //   ——远程弹道目标（R≈5,000 km）因此天然为 0，只有近距低空目标才有这一项。
  // 量级：20 km 远、100 m 高的 0.1 m² 目标约 −35 dB 相对其自身回波（CNR ~46 dB），
  //   远低于同一单元的副瓣杂波；属"低但合理"的补充项。
  // 标识：目标-地面再散射（2026-11-07）。
  float rescatter_w = 0.0f;
  if (input.target_rcs_m2 > 0.0f && input.target_height_m > kMinRescatterHeightM &&
      input.target_ground_range_m > 0.0f) {
    const float ground_range_m = input.target_ground_range_m;
    const float height_m = std::max(input.platform_altitude_m, 0.0f);
    const float radius_platform_m = re_eff_m + height_m;
    const float horizon_m = std::sqrt(std::max(
        radius_platform_m * radius_platform_m - re_eff_m * re_eff_m, 0.0f));
    if (ground_range_m <= horizon_m) {
      // 过程：投影点擦地角——用"平台→投影点"的斜距做精确三角（斜距 ≈ √(R_g² + h²)）。
      const float proj_slant_m = std::sqrt(ground_range_m * ground_range_m + height_m * height_m);
      const float cos_angle =
          (radius_platform_m * radius_platform_m + proj_slant_m * proj_slant_m -
           re_eff_m * re_eff_m) /
          (2.0f * radius_platform_m * proj_slant_m);
      const float angle_rad = std::acos(oneq::common::numerics::Clamp(cos_angle, -1.0f, 1.0f));
      const float proj_depression_deg = 90.0f - angle_rad * 57.29577951308232f;
      if (proj_depression_deg > 0.0f) {
        const float grazing_deg =
            oneq::common::numerics::Clamp(proj_depression_deg, kMinGrazingDeg, kMaxGrazingDeg);
        const float grazing_rad = DegToRad(grazing_deg);
        // 过程：补片面积 = 距离单元 × 方位波束（投影点的雷达分辨率单元）。
        const float patch_area_m2 = proj_slant_m * az_beamwidth_rad * range_cell_m /
                                    std::cos(grazing_rad);
        const float sigma0_db = ResolveSigma0Db(profile) +
            10.0f * std::log10(std::sin(grazing_rad) /
                               std::sin(DegToRad(kReferenceGrazingDeg)));
        const float patch_rcs_m2 =
            std::pow(10.0f, (sigma0_db + 10.0f * std::log10(patch_area_m2)) / 10.0f);
        const float height_to_ground_m = input.target_height_m;
        const float effective_rcs_m2 =
            input.target_rcs_m2 * patch_rcs_m2 *
            (input.range_m * input.range_m) /
            (4.0f * kPi * height_to_ground_m * height_to_ground_m *
             ground_range_m * ground_range_m);
        rescatter_w = rcs_to_noise_w(effective_rcs_m2, input.range_m,
                                     input.antenna.main_beam_gain_db);
      }
    }
  }

  // ---- 多重散射项（2026-11-07 甲方要求）：地面补片之间的二次及更高次散射 ----
  // 物理：雷达→补片A（一次散射，已由上面三项给出）→ 补片B → … → 雷达。
  //   相邻补片间距 d ≈ √A_patch（补片自身尺度），单次 bounce 的耦合
  //     η = σ₀·A_patch/(4π·d²) = σ₀/(4π)
  //   多次求和为几何级数 Σ_{k≥1} ηᵏ = η/(1−η) ≈ η（η ≪ 1，σ₀ 取参考擦地角）。
  //   所以多重散射的**均值**只是一次散射项的 η 倍（σ₀=−28 dB 时约 −39 dB），
  //   再乘调用方给的逐周期随机起伏（dB）——理论上界与随机范围都由这两个量界定，
  //   不会凭空放大到主导量级。
  // 标识：多重散射项（2026-11-07）。
  const float single_bounce_w = main_lobe_w + side_lobe_w + rescatter_w;
  float multi_scatter_w = 0.0f;
  if (single_bounce_w > 0.0f) {
    const float sigma0_ref_lin =
        std::pow(10.0f, ResolveSigma0Db(profile) / 10.0f);  // σ₀@参考擦地角（线性）
    const float eta = oneq::common::numerics::Clamp(sigma0_ref_lin / (4.0f * kPi), 0.0f, 1.0e-2f);
    multi_scatter_w = single_bounce_w * eta;
  }
  // 过程：**起伏落在整个杂波项上**（散斑 + 多重散射同乘一份起伏）——甲方要"一个能到达
  //   雷达的数值 + 随机范围"：均值仍由几何决定，逐周期在 dB 域上下浮动（默认 ±3 dB）。
  //   没有地面回波的单元（single_bounce_w = 0）起伏后仍为 0，不会被凭空抬成底噪。
  // 标识：杂波起伏（2026-11-07）。
  const float fluctuation_gain = std::pow(10.0f, input.clutter_fluctuation_db / 10.0f);
  return (single_bounce_w + multi_scatter_w) * fluctuation_gain;
}

}  // namespace internal
}  // namespace remote_identification_radar
