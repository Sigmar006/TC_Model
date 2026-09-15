/**
 * @file PolarizationStatsExtractor.cpp
 * @brief 极化散射矩阵统计特征提取器实现（五量×(均值, 标准差)）。
 */

#include "remote_identification_radar/recognition/PolarizationStatsExtractor.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>

#include "common/numerics/Constants.h"

namespace remote_identification_radar {
namespace recognition {
namespace {

using oneq::common::numerics::kPi;
/** @brief Graves 非对角下限：|c|² 低于该值视为对角占优（ψ 取 0/90、τ 取 0）。 */
constexpr double kOffDiagonalFloor = 1.0e-18;
/** @brief 圆统计平均向量长度下限：低于该值按完全弥散截断，防止 ln(0)。 */
constexpr double kMeanResultantFloor = 1.0e-12;

double DbsmToLinear(float dbsm) { return std::pow(10.0, static_cast<double>(dbsm) / 10.0); }

/** @brief 幅度（dBsm）+相位（deg）→ 复散射场幅度 √σ·e^{jφ}。 */
std::complex<double> MakeField(float amp_db, float phase_deg) {
  const double sigma = DbsmToLinear(amp_db);
  const double phase_rad = static_cast<double>(phase_deg) * kPi / 180.0;
  return std::sqrt(sigma) * std::complex<double>(std::cos(phase_rad), std::sin(phase_rad));
}

bool IsFiniteRow(const session::RirPolSMatrixSample& row) {
  return std::isfinite(row.aspect_az_deg) && std::isfinite(row.aspect_el_deg) &&
         std::isfinite(row.hh_amp_db) && std::isfinite(row.hh_phase_deg) &&
         std::isfinite(row.hv_amp_db) && std::isfinite(row.hv_phase_deg) &&
         std::isfinite(row.vh_amp_db) && std::isfinite(row.vh_phase_deg) &&
         std::isfinite(row.vv_amp_db) && std::isfinite(row.vv_phase_deg);
}

/** @brief 单行五量派生；行非法（含 Span≤0）返回 false。 */
bool DeriveRowQuantities(const session::RirPolSMatrixSample& row, double* abs_det, double* span,
                         double* depolarization, double* psi_deg, double* tau_deg) {
  using Complex = std::complex<double>;
  const Complex shh = MakeField(row.hh_amp_db, row.hh_phase_deg);
  const Complex shv = MakeField(row.hv_amp_db, row.hv_phase_deg);
  const Complex svh = MakeField(row.vh_amp_db, row.vh_phase_deg);
  const Complex svv = MakeField(row.vv_amp_db, row.vv_phase_deg);

  const Complex det = shh * svv - shv * svh;
  const double local_abs_det = std::abs(det);
  const double sigma_hh = std::norm(shh);
  const double sigma_hv = std::norm(shv);
  const double sigma_vh = std::norm(svh);
  const double sigma_vv = std::norm(svv);
  const double local_span = sigma_hh + sigma_hv + sigma_vh + sigma_vv;
  if (!std::isfinite(local_abs_det) || !std::isfinite(local_span) || !(local_span > 0.0)) {
    return false;
  }
  *abs_det = local_abs_det;
  *span = local_span;
  *depolarization = (sigma_hv + sigma_vh) / local_span;

  // Graves G = SᴴS（2×2 Hermitian），主特征向量极化即 ψ/τ。
  const double a = sigma_hh + sigma_vh;
  const double b = sigma_hv + sigma_vv;
  const Complex c = std::conj(shh) * shv + std::conj(svh) * svv;
  const double disc = std::sqrt((a - b) * (a - b) + 4.0 * std::norm(c));
  const double lambda = 0.5 * (a + b + disc);

  double local_psi_deg = 0.0;
  double local_tau_deg = 0.0;
  if (std::norm(c) <= kOffDiagonalFloor * kOffDiagonalFloor) {
    if (b > a) {
      local_psi_deg = 90.0;
    }
  } else {
    const Complex rho = (lambda - a) / c;
    const double rho_abs2 = std::norm(rho);
    const double psi_rad = 0.5 * std::atan2(2.0 * rho.real(), 1.0 - rho_abs2);
    double tau_arg = 2.0 * rho.imag() / (1.0 + rho_abs2);
    if (tau_arg > 1.0) {
      tau_arg = 1.0;
    } else if (tau_arg < -1.0) {
      tau_arg = -1.0;
    }
    const double tau_rad = 0.5 * std::asin(tau_arg);
    local_psi_deg = oneq::common::numerics::RadToDeg(psi_rad);
    local_tau_deg = oneq::common::numerics::RadToDeg(tau_rad);
  }
  if (!std::isfinite(local_psi_deg) || !std::isfinite(local_tau_deg)) {
    return false;
  }
  *psi_deg = local_psi_deg;
  *tau_deg = local_tau_deg;
  return true;
}

/** @brief 算术均值+样本标准差（n=1 时 std=0）。 */
RirPolarizationQuantityStats ArithmeticStats(const std::vector<double>& values) {
  RirPolarizationQuantityStats stats;
  const std::size_t n = values.size();
  if (n == 0U) {
    return stats;
  }
  double sum = 0.0;
  for (double value : values) {
    sum += value;
  }
  stats.mean = sum / static_cast<double>(n);
  if (n >= 2U) {
    double squared_difference = 0.0;
    for (double value : values) {
      const double difference = value - stats.mean;
      squared_difference += difference * difference;
    }
    stats.std = std::sqrt(squared_difference / static_cast<double>(n - 1U));
  }
  return stats;
}

}  // namespace

RirPolarizationQuantityStats CircularMeanStdDeg(const std::vector<double>& angles_deg) {
  RirPolarizationQuantityStats stats;
  const std::size_t n = angles_deg.size();
  if (n == 0U) {
    return stats;
  }
  // ψ 周期 180°：对倍角 2ψ 做单位向量平均，均值角取回 (−90°, 90°]。
  double cos_sum = 0.0;
  double sin_sum = 0.0;
  for (double angle_deg : angles_deg) {
    const double doubled_rad = 2.0 * angle_deg * kPi / 180.0;
    cos_sum += std::cos(doubled_rad);
    sin_sum += std::sin(doubled_rad);
  }
  cos_sum /= static_cast<double>(n);
  sin_sum /= static_cast<double>(n);
  double mean_rad = 0.5 * std::atan2(sin_sum, cos_sum);
  double resultant = std::sqrt(cos_sum * cos_sum + sin_sum * sin_sum);
  if (resultant > 1.0) {
    resultant = 1.0;
  }
  double std_rad = 0.0;
  if (resultant < 1.0 - 1.0e-15) {
    const double clamped = std::max(kMeanResultantFloor, resultant);
    std_rad = 0.5 * std::sqrt(-2.0 * std::log(clamped));
  }
  if (!std::isfinite(mean_rad) || !std::isfinite(std_rad)) {
    return stats;
  }
  stats.mean = oneq::common::numerics::RadToDeg(mean_rad);
  stats.std = oneq::common::numerics::RadToDeg(std_rad);
  return stats;
}

RirPolarizationStatsObservation RirPolarizationStatsExtractor::Extract(
    const std::vector<session::RirPolSMatrixSample>& window) {
  RirPolarizationStatsObservation observation;
  std::vector<double> abs_det_values;
  std::vector<double> span_values;
  std::vector<double> depolarization_values;
  std::vector<double> psi_values;
  std::vector<double> tau_values;
  abs_det_values.reserve(window.size());
  span_values.reserve(window.size());
  depolarization_values.reserve(window.size());
  psi_values.reserve(window.size());
  tau_values.reserve(window.size());
  for (const session::RirPolSMatrixSample& row : window) {
    if (!IsFiniteRow(row)) {
      continue;
    }
    double abs_det = 0.0;
    double span = 0.0;
    double depolarization = 0.0;
    double psi_deg = 0.0;
    double tau_deg = 0.0;
    if (!DeriveRowQuantities(row, &abs_det, &span, &depolarization, &psi_deg, &tau_deg)) {
      continue;
    }
    abs_det_values.push_back(abs_det);
    span_values.push_back(span);
    depolarization_values.push_back(depolarization);
    psi_values.push_back(psi_deg);
    tau_values.push_back(tau_deg);
  }
  if (abs_det_values.empty()) {
    return observation;
  }
  observation.determinant = ArithmeticStats(abs_det_values);
  observation.span = ArithmeticStats(span_values);
  observation.depolarization = ArithmeticStats(depolarization_values);
  observation.psi_deg = CircularMeanStdDeg(psi_values);
  observation.tau_deg = ArithmeticStats(tau_values);
  observation.valid = std::isfinite(observation.determinant.mean) &&
                      std::isfinite(observation.span.mean) &&
                      std::isfinite(observation.depolarization.mean) &&
                      std::isfinite(observation.psi_deg.mean) &&
                      std::isfinite(observation.tau_deg.mean);
  return observation;
}

}  // namespace recognition
}  // namespace remote_identification_radar
