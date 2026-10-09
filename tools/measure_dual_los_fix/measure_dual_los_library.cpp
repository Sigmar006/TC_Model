/**
 * @file measure_dual_los_library.cpp
 * @brief 开发期度量：双星交会定位精度表征（使用库内真实 SBIRS 误差模型）。
 *
 * 与 tools/dual_los_only 的区别：本工具链接已安装的 1q 库，量测误差直接调用
 * `sbirs_sensor::foundation::ApplyAngularErrorModel`，因此包含库内全部 5 类误差
 * （orbit/attitude/fov 高斯 + 大气折射 + 动态滞后确定性项），而非仅角度随机项。
 *
 * 链路（全部使用库内实现）：
 *   真值目标 ECEF
 *     -> 逐星视线方向（ECEF）-> TryEciToEcef 旋到 ECI -> ECI 极坐标 (az, el)
 *     -> ApplyAngularErrorModel 注入 5 类误差
 *     -> (az, el) -> ECI 方向 -> TryEciToEcef 回 ECEF
 *     -> precision_evaluation::TryComputeDualLosFixM 交会
 *     -> 与真值比较：三维位置误差、对 A 星斜距误差、几何残差
 *
 * 用法：
 *   measure_dual_los_library [--samples N] [--csv <path>]
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "1q/coordinate/inertial_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/precision_evaluation/DualLosFix.h"
#include "1q/sbirs_sensor/config/SbirsPolicyConfig.h"
#include "sbirs_sensor/foundation/SbirsErrorModel.h"

namespace {

using oneq::coordinate::EcefPositionM;
using oneq::coordinate::EciPositionM;
using oneq::coordinate::Vector3d;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kEarthEquatorialRadiusM = 6378137.0;
constexpr double kGeostationaryRadiusM = 42164000.0;

Vector3d DirectionFromAzElRad(double az_rad, double el_rad) {
  const double cos_el = std::cos(el_rad);
  return Vector3d(cos_el * std::cos(az_rad), cos_el * std::sin(az_rad), std::sin(el_rad));
}

double Norm3(const Vector3d& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

double Distance(const EcefPositionM& a, const EcefPositionM& b) {
  const double dx = a.x_m - b.x_m;
  const double dy = a.y_m - b.y_m;
  const double dz = a.z_m - b.z_m;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool Normalize(const Vector3d& v, Vector3d* out) {
  const double n = Norm3(v);
  if (!(n > 0.0) || !std::isfinite(n)) {
    return false;
  }
  out->x = v.x / n;
  out->y = v.y / n;
  out->z = v.z / n;
  return true;
}

// ECEF 方向 -> ECI -> 极坐标 (az, el)（与 SbirsAngleMeasurementModel 同口径）。
bool EcefDirectionToAzElRad(const Vector3d& dir_ecef, double gmst_rad, double* az_rad,
                            double* el_rad) {
  const EciPositionM as_eci(dir_ecef.x, dir_ecef.y, dir_ecef.z);
  EcefPositionM dir_eci;
  if (!oneq::coordinate::TryEciToEcef(as_eci, gmst_rad, &dir_eci)) {
    return false;
  }
  const double n = std::sqrt(dir_eci.x_m * dir_eci.x_m + dir_eci.y_m * dir_eci.y_m +
                             dir_eci.z_m * dir_eci.z_m);
  if (!(n > 0.0)) {
    return false;
  }
  *az_rad = std::atan2(dir_eci.y_m, dir_eci.x_m);
  if (*az_rad < 0.0) {
    *az_rad += 2.0 * kPi;
  }
  double sin_el = dir_eci.z_m / n;
  sin_el = sin_el < -1.0 ? -1.0 : (sin_el > 1.0 ? 1.0 : sin_el);
  *el_rad = std::asin(sin_el);
  return true;
}

// (az, el) [rad, ECI] -> ECEF 方向（库内交会自行归一，无需单位化）。
bool AzElRadToEcefDirection(double az_rad, double el_rad, double gmst_rad, Vector3d* out) {
  const Vector3d dir_eci = DirectionFromAzElRad(az_rad, el_rad);
  const EciPositionM eci_pos(dir_eci.x, dir_eci.y, dir_eci.z);
  EcefPositionM ecef_dir;
  if (!oneq::coordinate::TryEciToEcef(eci_pos, gmst_rad, &ecef_dir)) {
    return false;
  }
  *out = Vector3d(ecef_dir.x_m, ecef_dir.y_m, ecef_dir.z_m);
  return true;
}

struct Stats {
  double mean{0.0};
  double p50{0.0};
  double p95{0.0};
  double max{0.0};
};

Stats Summarize(std::vector<double> v) {
  Stats s;
  if (v.empty()) {
    return s;
  }
  double sum = 0.0;
  for (double x : v) {
    sum += x;
  }
  s.mean = sum / static_cast<double>(v.size());
  std::sort(v.begin(), v.end());
  s.p50 = v[v.size() / 2U];
  std::size_t i95 = static_cast<std::size_t>(0.95 * static_cast<double>(v.size()));
  if (i95 >= v.size()) {
    i95 = v.size() - 1U;
  }
  s.p95 = v[i95];
  s.max = v.back();
  return s;
}

struct SweepRow {
  double sigma_urad{0.0};
  double setting_deg{0.0};
  double actual_angle_deg{0.0};
  double truth_slant_a_m{0.0};
  Stats position_error_m{};
  Stats slant_error_m{};
  Stats residual_m{};
  std::size_t failed{0U};
};

const SweepRow* FindRow(const std::vector<SweepRow>& rows, double sigma, double setting) {
  for (const SweepRow& r : rows) {
    if (r.sigma_urad == sigma && r.setting_deg == setting) {
      return &r;
    }
  }
  return nullptr;
}

struct Pair {
  EcefPositionM a{};
  EcefPositionM b{};
};

Pair MakePair(double angle_deg) {
  Pair p;
  p.a = EcefPositionM(kGeostationaryRadiusM, 0.0, 0.0);
  const double th = angle_deg * kDegToRad;
  p.b = EcefPositionM(kGeostationaryRadiusM * std::cos(th), kGeostationaryRadiusM * std::sin(th),
                      0.0);
  return p;
}

EcefPositionM MakeTarget(double offset_deg) {
  const double ang = offset_deg * kDegToRad;
  return EcefPositionM(kEarthEquatorialRadiusM * std::cos(ang),
                       kEarthEquatorialRadiusM * std::sin(ang), 0.0);
}

double IntersectionAngleDeg(const Pair& p, const EcefPositionM& t) {
  Vector3d la(t.x_m - p.a.x_m, t.y_m - p.a.y_m, t.z_m - p.a.z_m);
  Vector3d lb(t.x_m - p.b.x_m, t.y_m - p.b.y_m, t.z_m - p.b.z_m);
  Normalize(la, &la);
  Normalize(lb, &lb);
  double c = la.x * lb.x + la.y * lb.y + la.z * lb.z;
  c = c < -1.0 ? -1.0 : (c > 1.0 ? 1.0 : c);
  return std::acos(c) * kRadToDeg;
}

}  // namespace

int main(int argc, char** argv) {
  std::size_t samples = 2000U;
  std::string csv_path;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--samples") == 0 && i + 1 < argc) {
      samples = static_cast<std::size_t>(std::atol(argv[++i]));
    } else if (std::strcmp(argv[i], "--csv") == 0 && i + 1 < argc) {
      csv_path = argv[++i];
    }
  }
  if (samples == 0U) {
    samples = 1U;
  }

  double gmst_rad = 0.0;
  if (!oneq::coordinate::TryComputeGmstRad(2451544.2230698913, &gmst_rad)) {
    std::fprintf(stderr, "TryComputeGmstRad failed\n");
    return 1;
  }

  const std::vector<double> sigmas_urad = {1.95, 3.0, 10.0, 30.0, 100.0, 300.0, 500.0};
  const std::vector<double> settings = {5.0, 10.0, 20.0, 45.0};

  std::printf("=== dual-LOS fix accuracy (library SBIRS error model, incl. refraction/lag) ===\n");
  std::printf("samples/grid = %zu ; GMST = %.6f rad\n", samples, gmst_rad);

  std::FILE* csv = nullptr;
  if (!csv_path.empty()) {
    csv = std::fopen(csv_path.c_str(), "w");
    if (csv != nullptr) {
      std::fprintf(csv, "sigma_urad,actual_angle_deg,position_error_m,slant_error_m,residual_m\n");
    }
  }

  std::vector<SweepRow> rows;

  for (double setting : settings) {
    Pair pair = MakePair(setting);

    double best_offset = 0.0;
    double best_err = 1.0e9;
    for (double off = 0.0; off <= 60.0; off += 0.1) {
      const double actual = IntersectionAngleDeg(pair, MakeTarget(off));
      const double err = std::fabs(actual - setting);
      if (err < best_err) {
        best_err = err;
        best_offset = off;
      }
    }
    const EcefPositionM target = MakeTarget(best_offset);
    const double actual_angle = IntersectionAngleDeg(pair, target);

    Vector3d ua;
    Vector3d ub;
    if (!Normalize(
            Vector3d(target.x_m - pair.a.x_m, target.y_m - pair.a.y_m, target.z_m - pair.a.z_m),
            &ua) ||
        !Normalize(
            Vector3d(target.x_m - pair.b.x_m, target.y_m - pair.b.y_m, target.z_m - pair.b.z_m),
            &ub)) {
      std::fprintf(stderr, "LOS normalize failed\n");
      continue;
    }
    const double slant_a = Distance(pair.a, target);
    const double slant_b = Distance(pair.b, target);

    double az_a = 0.0;
    double el_a = 0.0;
    double az_b = 0.0;
    double el_b = 0.0;
    if (!EcefDirectionToAzElRad(ua, gmst_rad, &az_a, &el_a) ||
        !EcefDirectionToAzElRad(ub, gmst_rad, &az_b, &el_b)) {
      std::fprintf(stderr, "LOS angle solve failed\n");
      continue;
    }

    for (double sigma_urad : sigmas_urad) {
      const double sigma_deg = sigma_urad * 1.0e-6 * kRadToDeg;
      const double each = sigma_deg / std::sqrt(3.0);

      sbirs_sensor::config::SbirsErrorModelConfig model;
      model.orbit_sigma_deg = static_cast<float>(each);
      model.attitude_sigma_deg = static_cast<float>(each);
      model.fov_sigma_deg = static_cast<float>(each);
      model.random_seed = 20261004U;

      sbirs_sensor::foundation::SbirsRandomSource rng_a(model.random_seed);
      sbirs_sensor::foundation::SbirsRandomSource rng_b(model.random_seed + 977U);

      std::vector<double> pos_err;
      std::vector<double> slant_err;
      std::vector<double> residual;
      pos_err.reserve(samples);
      slant_err.reserve(samples);
      residual.reserve(samples);
      std::size_t failed = 0U;

      for (std::size_t k = 0; k < samples; ++k) {
        const sbirs_sensor::foundation::SbirsErrorBearing m_a =
            sbirs_sensor::foundation::ApplyAngularErrorModel(
                model, &rng_a, static_cast<float>(az_a * kRadToDeg),
                static_cast<float>(el_a * kRadToDeg), slant_a, 0.0f);
        const sbirs_sensor::foundation::SbirsErrorBearing m_b =
            sbirs_sensor::foundation::ApplyAngularErrorModel(
                model, &rng_b, static_cast<float>(az_b * kRadToDeg),
                static_cast<float>(el_b * kRadToDeg), slant_b, 0.0f);

        Vector3d dir_a;
        Vector3d dir_b;
        if (!AzElRadToEcefDirection(static_cast<double>(m_a.azimuth_deg) * kDegToRad,
                                    static_cast<double>(m_a.elevation_deg) * kDegToRad, gmst_rad,
                                    &dir_a) ||
            !AzElRadToEcefDirection(static_cast<double>(m_b.azimuth_deg) * kDegToRad,
                                    static_cast<double>(m_b.elevation_deg) * kDegToRad, gmst_rad,
                                    &dir_b)) {
          ++failed;
          continue;
        }

        EcefPositionM fix;
        double res_m = 0.0;
        if (!precision_evaluation::TryComputeDualLosFixM(pair.a, dir_a, pair.b, dir_b, &fix,
                                                        &res_m)) {
          ++failed;
          continue;
        }
        const double pe = Distance(fix, target);
        const double se = Distance(pair.a, fix) - slant_a;
        pos_err.push_back(pe);
        slant_err.push_back(se);
        residual.push_back(res_m);
        if (csv != nullptr) {
          std::fprintf(csv, "%.4f,%.3f,%.6f,%.6f,%.6f\n", sigma_urad, actual_angle, pe, se, res_m);
        }
      }

      SweepRow row;
      row.sigma_urad = sigma_urad;
      row.setting_deg = setting;
      row.actual_angle_deg = actual_angle;
      row.truth_slant_a_m = slant_a;
      row.position_error_m = Summarize(pos_err);
      row.slant_error_m = Summarize(slant_err);
      row.residual_m = Summarize(residual);
      row.failed = failed;
      rows.push_back(row);
    }
  }

  if (csv != nullptr) {
    std::fclose(csv);
    std::printf("CSV: %s\n", csv_path.c_str());
  }

  std::printf("\nGeometry:\n");
  for (double setting : settings) {
    const SweepRow* r = FindRow(rows, sigmas_urad.front(), setting);
    if (r != nullptr) {
      std::printf("  set %5.1f deg -> actual intersection %5.2f deg, slant(A) %.0f km\n", setting,
                  r->actual_angle_deg, r->truth_slant_a_m / 1000.0);
    }
  }

  std::printf("\nTABLE 1  position error (3D, m) = mean | P95\n");
  std::printf("%-12s", "sigma(urad)");
  for (double s : settings) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "th=%ddeg", static_cast<int>(s));
    std::printf(" %16s", buf);
  }
  std::printf("\n");
  for (double sigma : sigmas_urad) {
    std::printf("%-12.2f", sigma);
    for (double s : settings) {
      const SweepRow* r = FindRow(rows, sigma, s);
      if (r == nullptr) {
        std::printf(" %16s", "-");
        continue;
      }
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.1f|%.0f", r->position_error_m.mean,
                    r->position_error_m.p95);
      std::printf(" %16s", buf);
    }
    std::printf("\n");
  }

  std::printf("\nTABLE 2  slant-range error to A (mean, m) | geometric residual (mean, m)\n");
  std::printf("%-12s", "sigma(urad)");
  for (double s : settings) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "th=%ddeg", static_cast<int>(s));
    std::printf(" %18s", buf);
  }
  std::printf("\n");
  for (double sigma : sigmas_urad) {
    std::printf("%-12.2f", sigma);
    for (double s : settings) {
      const SweepRow* r = FindRow(rows, sigma, s);
      if (r == nullptr) {
        std::printf(" %18s", "-");
        continue;
      }
      char buf[40];
      std::snprintf(buf, sizeof(buf), "%.1f|%.2f", r->slant_error_m.mean, r->residual_m.mean);
      std::printf(" %18s", buf);
    }
    std::printf("\n");
  }

  std::size_t failed_total = 0U;
  for (const SweepRow& r : rows) {
    failed_total += r.failed;
  }
  std::printf("\nfailed samples total = %zu\n", failed_total);
  return 0;
}
