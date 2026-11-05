/**
 * ## 过程：目标辐射 → 入瞳功率 → 线性 SNR / 最大探测距离（红外处理链 · 第 5 步）
 * - 上游：目标辐射强度 `SbirsEciSceneTarget::radiant_intensity_w_per_sr`（场景输入）、
 *   斜距（第 3 步）、大气路径透过率（第 4 步）、硬件光学参数（`SbirsHardwareConfig`）。
 * - 本步：
 *   · 入瞳接收功率 P = I·A·τ_opt·τ_path·η / R²，圆孔径 A = π·D²/4；
 *   · 线性 SNR = P·t_int / N_eff（分子为积分时间内的信号能量，分母由第 6 步噪声模型给出）；
 *   · 由 SNR 门反解最大探测距离 R_max = √(I·A·τ_opt·τ_path·η·t_int / (N_eff·SNR_min))。
 * - 下游：pipeline `ComputeSnr` 的 WFOV/NFOV 检测门与候选排序；第 6 项最大探测距离（再与
 *   几何地平线、任务上限取小）；接收功率/信号能量/SNR 进第 75 项内部接口记录。
 * - 关键参数：孔径 `hardware.optical_aperture_m`(0.5 m)、光学透过率 0.8、量子效率 0.7、
 *   积分时间 0.02 s、NEP 1e-12 W（见 SbirsHardwareConfig.h）；门限
 *   `policy.detection.wide_min_snr_linear`(4) / `narrow_min_snr_linear`(6)。
 * - 对应验收项：第 6 项 最大探测距离计算功能测试；第 7 项 目标可探测性与动态参数生成功能测试；
 *   第 75 项 红外辐射噪声与检测判决交互接口测试（链路预算）。
 */

#include "sbirs_sensor/foundation/SbirsRadiometry.h"

#include <algorithm>
#include <cmath>
#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace foundation {
namespace {

using oneq::common::numerics::kPi;

}  // namespace

// 过程：接收功率 = 目标辐射强度 I × 孔径面积 A × 光学透过率 × 路径透过率 × 量子效率 / R²；
//       A = π·D²/4 为圆口径面积，1/R² 体现辐照度随距离平方衰减。
double ComputeReceivedPowerW(double radiant_intensity_w_per_sr, double range_m,
                             double aperture_m, double optical_transmission,
                             double path_transmittance, double detector_quantum_efficiency) {
  if (radiant_intensity_w_per_sr < 0.0 || range_m <= 0.0 || aperture_m <= 0.0) {
    return 0.0;
  }
  const double aperture_area = kPi * aperture_m * aperture_m * 0.25;
  const double signal = radiant_intensity_w_per_sr * aperture_area;
  return signal * std::max(0.0, optical_transmission) * std::max(0.0, path_transmittance) *
         std::max(0.0, detector_quantum_efficiency) / (range_m * range_m);
}

// 过程：SNR = 信号能量 / 噪声能量 = (P·t_int) / N_eff，量纲抵消后为无量纲比值；
//       分母取下限 1e-18 W 保证恒正（全零配置时不产生 inf/NaN）。
double ComputeInfraredSnrLinear(double received_power_w,
                                const config::SbirsHardwareConfig& hardware) {
  const double signal_energy =
      std::max(0.0, received_power_w) * std::max(0.0f, hardware.integration_time_sec);
  const double noise = std::max(1.0e-18f, hardware.noise_equivalent_power_w);
  return signal_energy / noise;
}

// 过程：把 SNR(R) = I·A·τ_opt·τ_path·η·t_int / (N_eff·R²) 对 R 反解，得
//       R_max = √(分子 / (N_eff·SNR_min))；任一参数为 0（该项未建模）即回 0（不可解算）。
double ComputeMaxDetectionRangeM(double radiant_intensity_w_per_sr, double aperture_m,
                                 double optical_transmission, double path_transmittance,
                                 double detector_quantum_efficiency,
                                 double integration_time_sec, double effective_noise_w,
                                 double min_snr_linear) {
  if (radiant_intensity_w_per_sr <= 0.0 || aperture_m <= 0.0 || optical_transmission <= 0.0 ||
      path_transmittance <= 0.0 || detector_quantum_efficiency <= 0.0 ||
      integration_time_sec <= 0.0 || effective_noise_w <= 0.0 || min_snr_linear <= 0.0) {
    return 0.0;
  }
  const double aperture_area = kPi * aperture_m * aperture_m * 0.25;
  const double numerator =
      radiant_intensity_w_per_sr * aperture_area * optical_transmission * path_transmittance *
      detector_quantum_efficiency * integration_time_sec;
  return std::sqrt(numerator / (effective_noise_w * min_snr_linear));
}

}  // namespace foundation
}  // namespace sbirs_sensor
