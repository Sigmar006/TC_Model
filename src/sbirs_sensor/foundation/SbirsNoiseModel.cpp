#include "sbirs_sensor/foundation/SbirsNoiseModel.h"

#include <algorithm>
#include <cmath>
#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace foundation {
namespace {

using oneq::common::numerics::kLightSpeed;
using oneq::common::numerics::kPi;
using oneq::common::numerics::kPlanck;

}  // namespace

SbirsNoiseStatistics ComputeBackgroundNoiseStatistics(const config::SbirsHardwareConfig& hardware) {
  SbirsNoiseStatistics stats;
  // 噪声分母统一"积分时间内噪声能量"口径（SNR = P_sig·t_int / N_eff，冻结契约
  // sbirs-noise-realistic-snr 修订 1）：各分量按贡献能量折算后 RSS 合成。
  const double t_int = std::max(0.0f, hardware.integration_time_sec);

  // 光子（散粒）噪声：背景辐射亮度 × 孔径面积 × 光学透过率 × 像元视场立体角得背景
  // 功率，经单光子能量换算成光子数涨落 √(P_bg·t·E_ph)。像元视场 Ω=(像元间距/焦距)²；
  // E_ph=hc/λ_center（波段中心单色近似，修订 4）。任一几何量非法时该项为 0。
  if (hardware.background_radiance_w_sr_m2 > 0.0f && hardware.optical_aperture_m > 0.0f &&
      hardware.focal_length_m > 0.0f && hardware.detector_pixel_pitch_m > 0.0f &&
      hardware.wavelength_lower_um > 0.0f &&
      hardware.wavelength_upper_um > hardware.wavelength_lower_um && t_int > 0.0) {
    const double aperture_area = kPi * hardware.optical_aperture_m * hardware.optical_aperture_m * 0.25;
    const double pixel_solid_angle_sr =
        (static_cast<double>(hardware.detector_pixel_pitch_m) / hardware.focal_length_m) *
        (static_cast<double>(hardware.detector_pixel_pitch_m) / hardware.focal_length_m);
    const double lambda_center_m =
        0.5e-6 * (static_cast<double>(hardware.wavelength_lower_um) +
                  static_cast<double>(hardware.wavelength_upper_um));
    const double photon_energy_j = kPlanck * kLightSpeed / lambda_center_m;
    const double background_power_w =
        static_cast<double>(hardware.background_radiance_w_sr_m2) * aperture_area *
        std::max(0.0f, hardware.optical_transmission) * pixel_solid_angle_sr;
    stats.photon_noise_w = std::sqrt(std::max(0.0, background_power_w * t_int * photon_energy_j));
  }

  // 热噪声：不再由探测器温度反解（2026-10-21 修订）。原实现 sqrt(4·k_B·T·t_int) 的量纲是
  // √(J·s) 而非能量，T=80 K/t=1 s 时得 6.6e-11 J，压过 NEP 折算能量 4 个数量级，把 SNR
  // 从 +18 dB 拉到 −20 dB（现场“有效噪声能量”与“热噪声”逐位相等即此项所致）。
  // 探测器噪声由 NEP 承担；需要显式热噪声时应以 NEP 类配置给出，不能由温度推能量。

  // 读出噪声：配置 RMS 折算到能量口径（×t_int）。
  stats.readout_noise_w = std::max(0.0f, hardware.readout_noise_rms_w) * t_int;

  // 探测器 NEP 计入合成（修订 1）：同样折算能量口径（NEP·t_int）；三项 RSS。
  const double nep_energy = std::max(0.0f, hardware.noise_equivalent_power_w) * t_int;
  const double sum_sq = nep_energy * nep_energy +
                        stats.photon_noise_w * stats.photon_noise_w +
                        stats.readout_noise_w * stats.readout_noise_w;
  stats.total_noise_w = std::sqrt(std::max(0.0, sum_sq));
  return stats;
}

double ResolveEffectiveNoiseW(const config::SbirsHardwareConfig& hardware,
                              const SbirsNoiseStatistics& statistics) {
  // 全部分量退化（NEP=0 且背景/热/读出全关，或积分时间 0）时回退 NEP 标量下限，
  // 保证分母恒正。NEP>0 时 total 恒大于 0，不走该分支。
  if (statistics.total_noise_w <= 0.0) {
    return std::max(1.0e-18f, hardware.noise_equivalent_power_w);
  }
  return statistics.total_noise_w;
}

}  // namespace foundation
}  // namespace sbirs_sensor
