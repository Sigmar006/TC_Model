// 验证 algorithms.md Foundation 物理链路条目：sbirs_noise_model_test
// 覆盖 NEP+光子+读出 RSS 合成（能量分母口径，2026-09-02 修订；2026-10-21 移除温度反解
// 的热噪声项）与全退化回退。
#include <cmath>

#include <gtest/gtest.h>

#include "common/numerics/Constants.h"
#include "sbirs_sensor/foundation/SbirsNoiseModel.h"

namespace {

using sbirs_sensor::config::SbirsHardwareConfig;
using sbirs_sensor::foundation::ComputeBackgroundNoiseStatistics;
using sbirs_sensor::foundation::ResolveEffectiveNoiseW;
using sbirs_sensor::foundation::SbirsNoiseStatistics;

// 全部分量关闭（NEP=0 且背景/热/读出全关）→ total 为 0，回退 1e-18 下限。
TEST(SbirsNoiseModelTest, AllComponentsOffFallsBackToNepFloor) {
  SbirsHardwareConfig hardware;
  hardware.noise_equivalent_power_w = 0.0f;
  hardware.background_radiance_w_sr_m2 = 0.0f;
  hardware.detector_temperature_k = 0.0f;
  hardware.readout_noise_rms_w = 0.0f;
  const SbirsNoiseStatistics stats = ComputeBackgroundNoiseStatistics(hardware);
  EXPECT_DOUBLE_EQ(stats.total_noise_w, 0.0);
  // 1e-18f 回退值经 float 提升有尾差，用相对容差。
  EXPECT_NEAR(ResolveEffectiveNoiseW(hardware, stats), 1.0e-18, 1.0e-24);
}

// 探测器 NEP 直接计入 RSS（能量口径 NEP·t），不再只是回退项。
TEST(SbirsNoiseModelTest, NepAlwaysContributesToRss) {
  SbirsHardwareConfig hardware;
  hardware.noise_equivalent_power_w = 1.0e-12f;
  hardware.background_radiance_w_sr_m2 = 0.0f;
  hardware.detector_temperature_k = 0.0f;
  hardware.readout_noise_rms_w = 0.0f;
  hardware.integration_time_sec = 0.02f;
  const SbirsNoiseStatistics stats = ComputeBackgroundNoiseStatistics(hardware);
  EXPECT_NEAR(stats.total_noise_w, 1.0e-12 * 0.02, 1.0e-18);
  EXPECT_DOUBLE_EQ(ResolveEffectiveNoiseW(hardware, stats), stats.total_noise_w);
}

// 光子噪声闭式核对：N_photon=√(P_bg·t·E_ph)，P_bg=L·A·τ_opt·(pitch/f)²，E_ph=hc/λ_center。
// 默认硬件（L=2.0、A=π/4·0.5²、τ=0.8、f=2.0、pitch=30μm、波段 3~5μm、t=1s）
// → N_photon≈1.87e-15（背景限制真实量级的锚点值）。
TEST(SbirsNoiseModelTest, PhotonNoiseMatchesClosedForm) {
  const SbirsHardwareConfig hardware;  // 全默认（背景亮度 2.0、温度 0、积分 0.02→改 1.0）
  SbirsHardwareConfig one_sec = hardware;
  one_sec.integration_time_sec = 1.0f;
  one_sec.noise_equivalent_power_w = 0.0f;  // 隔离光子项
  const SbirsNoiseStatistics stats = ComputeBackgroundNoiseStatistics(one_sec);
  const double aperture_area = oneq::common::numerics::kPi * one_sec.optical_aperture_m *
                               one_sec.optical_aperture_m * 0.25;
  const double omega = (one_sec.detector_pixel_pitch_m / one_sec.focal_length_m) *
                       (one_sec.detector_pixel_pitch_m / one_sec.focal_length_m);
  const double lambda_center_m =
      0.5e-6 * (one_sec.wavelength_lower_um + one_sec.wavelength_upper_um);
  const double e_ph = oneq::common::numerics::kPlanck * oneq::common::numerics::kLightSpeed /
                      lambda_center_m;
  const double expected = std::sqrt(one_sec.background_radiance_w_sr_m2 * aperture_area *
                                    one_sec.optical_transmission * omega * 1.0f * e_ph);
  // 期望值与实现同式但浮点求值顺序略有差异，取相对容差 1e-6（float 字段提升）。
  EXPECT_NEAR(stats.photon_noise_w, expected, expected * 1.0e-6);
  EXPECT_NEAR(expected, 1.87e-15, 1.0e-17);
}

// 像元间距 ×2 → 像元视场 ×4 → 光子噪声 ×2（√ 缩放）。
TEST(SbirsNoiseModelTest, PixelPitchScalesPhotonNoiseBySquareRoot) {
  SbirsHardwareConfig base;
  base.integration_time_sec = 1.0f;
  base.noise_equivalent_power_w = 0.0f;
  SbirsHardwareConfig doubled = base;
  doubled.detector_pixel_pitch_m *= 2.0f;
  const double base_photon = ComputeBackgroundNoiseStatistics(base).photon_noise_w;
  const double doubled_photon = ComputeBackgroundNoiseStatistics(doubled).photon_noise_w;
  ASSERT_GT(base_photon, 0.0);  // 守门：光子分支须生效，否则 0≈0 空转
  EXPECT_NEAR(doubled_photon, base_photon * 2.0, base_photon * 1.0e-9);
}

// 热噪声不再由探测器温度反解（2026-10-21 修订）：温度不进噪声合成。
// 原实现 √(4·k_B·T·t) 的量纲是 √(J·s)，T=80 K/t=1 s 给出 6.6e-11 J，比 NEP 折算能量大
// 4 个数量级、比物理热噪声能量（~k_B·T≈1e-21 J）大 10 个数量级，把 SNR 从 +18 dB 压到 −20 dB。
TEST(SbirsNoiseModelTest, ThermalNoiseIsNotDerivedFromTemperature) {
  SbirsHardwareConfig cold;
  cold.detector_temperature_k = 40.0f;
  cold.integration_time_sec = 1.0f;
  SbirsHardwareConfig hot = cold;
  hot.detector_temperature_k = 300.0f;
  const SbirsNoiseStatistics cold_stats = ComputeBackgroundNoiseStatistics(cold);
  const SbirsNoiseStatistics hot_stats = ComputeBackgroundNoiseStatistics(hot);
  EXPECT_DOUBLE_EQ(cold_stats.total_noise_w, hot_stats.total_noise_w);
}

// 现场工作点回归（sbirs.json：NEP=1e-14、T=80 K、t=1 s、背景 1.2）：
// 噪声底必须由 NEP 主导（≈1e-14 J），不得被温度项抬到 6.6e-11 J 那种量级。
TEST(SbirsNoiseModelTest, FieldWorkingPointNoiseFloorIsNepDominated) {
  SbirsHardwareConfig hardware;
  hardware.noise_equivalent_power_w = 1.0e-14f;
  hardware.detector_temperature_k = 80.0f;
  hardware.integration_time_sec = 1.0f;
  hardware.readout_noise_rms_w = 0.0f;
  hardware.background_radiance_w_sr_m2 = 1.2f;
  const SbirsNoiseStatistics stats = ComputeBackgroundNoiseStatistics(hardware);
  EXPECT_GT(stats.total_noise_w, 1.0e-14);   // NEP 项在 RSS 里
  EXPECT_LT(stats.total_noise_w, 1.1e-14);   // 只剩光子项 1.45e-15 的贡献
}

// 读出噪声远大于其他项时，总噪声趋近读出项（RMS×t）；关闭 NEP/热/背景以隔离。
TEST(SbirsNoiseModelTest, RmsCompositionDominatedByLargestComponent) {
  SbirsHardwareConfig hardware;
  hardware.readout_noise_rms_w = 1.0e-9f;
  hardware.noise_equivalent_power_w = 0.0f;
  hardware.background_radiance_w_sr_m2 = 0.0f;
  hardware.detector_temperature_k = 0.0f;
  hardware.integration_time_sec = 0.02f;
  const SbirsNoiseStatistics stats = ComputeBackgroundNoiseStatistics(hardware);
  EXPECT_NEAR(stats.readout_noise_w, 1.0e-9 * 0.02, 2.0e-11 * 1.0e-6);
  EXPECT_NEAR(stats.total_noise_w, stats.readout_noise_w, 1.0e-18);
}

}  // namespace
