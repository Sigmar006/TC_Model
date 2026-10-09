/**
 * @file measure_fusion_velocity.cpp
 * @brief 开发期度量：双星红外观测下融合 UKF 的速度估计精度与收敛性。
 *
 * 目的：回答「SBIRS 能否给出较准确的测速信息」——用库内真实 FusionEngine 的
 * 6 维（位置+速度）UKF，喂入两星的方位量测（含量测原点），实测：
 *   1、速度估计误差随周期的收敛过程；
 *   2、收敛后的稳态精度；
 *   3、UKF 协方差的速度块（1σ）是否如实反映精度；
 *   4、CV 模型对机动目标的结构性滞后。
 *
 * 不改库代码：本工具自行完成「ECI 极坐标角度 → 该星局部 ENU（自北向东）」
 * 的换系，然后直接调用 fusion::FusionEngine。换系结果用一条独立的几何校验
 * （ECEF 视线 → 经 TryEcefToEnu 得 az/el）对账，避免绕错轴向静默出错。
 *
 * 数据源：场景 sbirs_triple_sat_fix_messages 的落盘 CSV
 *   sbirs_sats.csv      —— 星星 ECEF 位置 + GMST
 *   sbirs_los.csv       —— 逐周期逐星逐目标 ECI az/el（*_rad 列）
 *   sbirs_truth.csv     —— 目标真值 ECEF 轨迹（仅用于误差统计）
 *
 * 用法：
 *   measure_fusion_velocity --scene-dir <out_croscue_on 目录> [--target 1] [--csv <path>]
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "1q/coordinate/inertial_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/fusion/FusionConfig.h"
#include "1q/fusion/FusionEngine.h"

namespace {

using oneq::coordinate::EcefPositionM;
using oneq::coordinate::EcefVelocityMps;
using oneq::coordinate::LlaPositionDegM;
using oneq::coordinate::Vector3d;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

// ---------------------------------------------------------------------------
// CSV 轻量读取（场景产物为 ASCII 数字列）
// ---------------------------------------------------------------------------
struct CsvTable {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;
  int Index(const std::string& name) const {
    for (std::size_t i = 0; i < header.size(); ++i) {
      if (header[i] == name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }
};

bool ReadCsv(const std::string& path, CsvTable* out) {
  std::ifstream in(path.c_str());
  if (!in) {
    std::fprintf(stderr, "无法打开: %s\n", path.c_str());
    return false;
  }
  std::string line;
  bool first = true;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      cells.push_back(cell);
    }
    if (first) {
      out->header = cells;
      first = false;
    } else {
      out->rows.push_back(cells);
    }
  }
  return true;
}

double Cell(const std::vector<std::string>& row, int idx) {
  if (idx < 0 || idx >= static_cast<int>(row.size())) {
    return 0.0;
  }
  return std::atof(row[static_cast<std::size_t>(idx)].c_str());
}

std::uint64_t CellU64(const std::vector<std::string>& row, int idx) {
  if (idx < 0 || idx >= static_cast<int>(row.size())) {
    return 0U;
  }
  return static_cast<std::uint64_t>(std::strtoull(row[static_cast<std::size_t>(idx)].c_str(),
                                                  nullptr, 10));
}

// ---------------------------------------------------------------------------
// 几何：ECI 极坐标方向 → 该星局部 ENU（az 自北向东、el 出地平）
// ---------------------------------------------------------------------------
Vector3d EciDirectionFromAzElRad(double az_rad, double el_rad) {
  const double cos_el = std::cos(el_rad);
  return Vector3d(cos_el * std::cos(az_rad), cos_el * std::sin(az_rad), std::sin(el_rad));
}

double Norm3(const Vector3d& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

void Normalize(Vector3d* v) {
  const double n = Norm3(*v);
  if (n > 0.0) {
    v->x /= n;
    v->y /= n;
    v->z /= n;
  }
}

// ECEF 单位方向 → 给定 LLA 原点处的 ENU 分量
void EcefDirectionToEnu(const Vector3d& dir_ecef, const LlaPositionDegM& origin,
                        double* east, double* north, double* up) {
  const double lat = origin.latitude_deg * kDegToRad;
  const double lon = origin.longitude_deg * kDegToRad;
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);
  *east = -sin_lon * dir_ecef.x + cos_lon * dir_ecef.y;
  *north = -sin_lat * cos_lon * dir_ecef.x - sin_lat * sin_lon * dir_ecef.y + cos_lat * dir_ecef.z;
  *up = cos_lat * cos_lon * dir_ecef.x + cos_lat * sin_lon * dir_ecef.y + sin_lat * dir_ecef.z;
}

// 角度包到 (-180, 180]
double Wrap180Deg(double deg) {
  double v = std::fmod(deg + 540.0, 360.0) - 180.0;
  return v;
}

struct Stats {
  double mean{0.0};
  double p50{0.0};
  double p95{0.0};
  double max{0.0};
  std::size_t n{0U};
};

Stats Summarize(std::vector<double> v) {
  Stats s;
  s.n = v.size();
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

}  // namespace

int main(int argc, char** argv) {
  std::string scene_dir;
  std::uint64_t target_id = 1U;
  std::string csv_out;
  std::uint32_t only_source = 0U;  // 0 = 两源；非 0 = 只用该源（诊断）
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--scene-dir") == 0 && i + 1 < argc) {
      scene_dir = argv[++i];
    } else if (std::strcmp(argv[i], "--target") == 0 && i + 1 < argc) {
      target_id = static_cast<std::uint64_t>(std::atol(argv[++i]));
    } else if (std::strcmp(argv[i], "--csv") == 0 && i + 1 < argc) {
      csv_out = argv[++i];
    } else if (std::strcmp(argv[i], "--only-source") == 0 && i + 1 < argc) {
      only_source = static_cast<std::uint32_t>(std::atol(argv[++i]));  // 诊断：只用单源
    }
  }
  if (scene_dir.empty()) {
    std::fprintf(stderr, "用法: measure_fusion_velocity --scene-dir <场景输出目录> "
                         "[--target N] [--csv path]\n");
    return 2;
  }

  // ---- 读入三份 CSV ----
  CsvTable sats;
  CsvTable los;
  CsvTable truth;
  if (!ReadCsv(scene_dir + "/sbirs_sats.csv", &sats) ||
      !ReadCsv(scene_dir + "/sbirs_los.csv", &los) ||
      !ReadCsv(scene_dir + "/sbirs_truth.csv", &truth)) {
    return 1;
  }

  const int s_src = sats.Index("source_id");
  const int s_x = sats.Index("ecef_x_m");
  const int s_y = sats.Index("ecef_y_m");
  const int s_z = sats.Index("ecef_z_m");
  const int s_gmst = sats.Index("gmst_rad");
  const int l_cycle = los.Index("cycle");
  const int l_src = los.Index("source_id");
  const int l_tgt = los.Index("target_id");
  const int l_status = los.Index("status");
  const int l_az = los.Index("az_rad");
  const int l_el = los.Index("el_rad");
  const int t_cycle = truth.Index("cycle");
  const int t_tgt = truth.Index("target_id");
  const int t_x = truth.Index("ecef_x_m");
  const int t_y = truth.Index("ecef_y_m");
  const int t_z = truth.Index("ecef_z_m");

  if (s_src < 0 || l_az < 0 || l_status < 0 || t_x < 0) {
    std::fprintf(stderr, "CSV 列缺失（sats/los/truth 表头不符预期）\n");
    return 1;
  }

  // 星星 ECEF 与 GMST
  std::map<std::uint32_t, EcefPositionM> sat_ecef;
  std::map<std::uint32_t, LlaPositionDegM> sat_lla;
  double gmst_rad = 0.0;
  for (const auto& row : sats.rows) {
    const std::uint32_t src = static_cast<std::uint32_t>(CellU64(row, s_src));
    EcefPositionM p{Cell(row, s_x), Cell(row, s_y), Cell(row, s_z)};
    sat_ecef[src] = p;
    LlaPositionDegM lla;
    if (oneq::coordinate::TryEcefToLla(p, &lla)) {
      sat_lla[src] = lla;
    }
    gmst_rad = Cell(row, s_gmst);
  }

  // 目标真值轨迹
  std::map<std::uint32_t, EcefPositionM> truth_pos;
  for (const auto& row : truth.rows) {
    if (CellU64(row, t_tgt) != target_id) {
      continue;
    }
    truth_pos[static_cast<std::uint32_t>(CellU64(row, t_cycle))] =
        EcefPositionM{Cell(row, t_x), Cell(row, t_y), Cell(row, t_z)};
  }
  if (truth_pos.size() < 3U) {
    std::fprintf(stderr, "目标 %llu 真值轨迹不足\n",
                 static_cast<unsigned long long>(target_id));
    return 1;
  }

  // 真值速度（中心差分，用于误差统计）
  std::map<std::uint32_t, EcefVelocityMps> truth_vel;
  for (const auto& kv : truth_pos) {
    const std::uint32_t c = kv.first;
    auto prev = truth_pos.find(c - 1U);
    auto next = truth_pos.find(c + 1U);
    EcefVelocityMps v{};
    if (prev != truth_pos.end() && next != truth_pos.end()) {
      const double dt = 2.0;
      v.x_mps = (next->second.x_m - prev->second.x_m) / dt;
      v.y_mps = (next->second.y_m - prev->second.y_m) / dt;
      v.z_mps = (next->second.z_m - prev->second.z_m) / dt;
      truth_vel[c] = v;
    }
  }

  // ---- 逐周期构造方位量测（带量测原点）----
  const int max_cycle = 80;
  std::map<std::uint32_t, std::vector<std::pair<std::uint32_t, std::pair<double, double>>>>
      per_cycle;  // cycle -> [(source, (az_deg, el_deg in ENU))]
  std::size_t frame_check_fail = 0U;
  std::size_t frame_check_ok = 0U;
  double frame_check_max_err_deg = 0.0;
  for (const auto& row : los.rows) {
    if (CellU64(row, l_tgt) != target_id) {
      continue;
    }
    if (row[static_cast<std::size_t>(l_status)] != "detected") {
      continue;
    }
    const std::uint32_t src = static_cast<std::uint32_t>(CellU64(row, l_src));
    const std::uint32_t cyc = static_cast<std::uint32_t>(CellU64(row, l_cycle));
    const double az_rad = Cell(row, l_az);
    const double el_rad = Cell(row, l_el);
    if (sat_ecef.count(src) == 0U || sat_lla.count(src) == 0U) {
      continue;
    }
    // 1) ECI 方向 → ECEF 方向（旋转为绕 z 轴，做与库内相反方向的旋转）
    const Vector3d dir_eci = EciDirectionFromAzElRad(az_rad, el_rad);
    oneq::coordinate::EciPositionM eci_as_pos(dir_eci.x, dir_eci.y, dir_eci.z);
    EcefPositionM ecef_dir;
    if (!oneq::coordinate::TryEciToEcef(eci_as_pos, gmst_rad, &ecef_dir)) {
      continue;
    }
    Vector3d dir_ecef{ecef_dir.x_m, ecef_dir.y_m, ecef_dir.z_m};
    Normalize(&dir_ecef);
    // 2) ECEF 方向 → 该星局部 ENU
    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
    EcefDirectionToEnu(dir_ecef, sat_lla[src], &east, &north, &up);
    const double az_enu_deg = std::atan2(east, north) * kRadToDeg;  // 自北向东
    const double el_enu_deg = std::asin(std::max(-1.0, std::min(1.0, up))) * kRadToDeg;

    // 3) 独立几何校验：真值 ECEF 视线 → 经库内 TryEcefToEnu 得 az/el（仅作轴向自检）
    auto tp = truth_pos.find(cyc);
    if (tp != truth_pos.end()) {
      oneq::coordinate::EnuPositionM enu;
      if (oneq::coordinate::TryEcefToEnu(tp->second, sat_lla[src], &enu)) {
        const double az2 = std::atan2(enu.east_m, enu.north_m) * kRadToDeg;
        const double el2 = std::asin(std::max(-1.0, std::min(1.0, enu.up_m /
            std::sqrt(enu.east_m * enu.east_m + enu.north_m * enu.north_m + enu.up_m * enu.up_m)))) *
                           kRadToDeg;
        const double d_az = std::fabs(Wrap180Deg(az_enu_deg - az2));
        const double d_el = std::fabs(el_enu_deg - el2);
        const double worst = std::max(d_az, d_el);
        frame_check_max_err_deg = std::max(frame_check_max_err_deg, worst);
        // 自检容差放宽（量测含噪）：>2° 视为换系轴向错误
        if (worst > 2.0) {
          ++frame_check_fail;
        } else {
          ++frame_check_ok;
        }
      }
    }
    per_cycle[cyc].push_back({src, {az_enu_deg, el_enu_deg}});
  }

  std::printf("=== 双星红外观测下融合 UKF 速度估计实测 ===\n");
  std::printf("场景目录: %s\n目标: %llu\n", scene_dir.c_str(),
              static_cast<unsigned long long>(target_id));
  if (only_source != 0U) { std::printf("[诊断] 仅用 source_id = %u\n", only_source); }
  std::printf("参与星: ");
  for (const auto& kv : sat_ecef) {
    std::printf("%u ", kv.first);
  }
  std::printf("\nGMST = %.6f rad\n", gmst_rad);
  std::printf("换系自检: 通过 %zu 帧 / 失败 %zu 帧，最大偏差 %.3f deg（失败阈值 2 deg）\n",
              frame_check_ok, frame_check_fail, frame_check_max_err_deg);
  if (frame_check_fail > frame_check_ok) {
    std::fprintf(stderr, "\n[中止] 换系自检大面积失败：轴向或 GMST 方向可能搞反。\n");
    return 1;
  }

  // ---- 驱动 FusionEngine ----
  fusion::FusionConfig cfg;
  cfg.track_cycle_period_sec = 1.0;
  cfg.track_bearing_init_range_m = 36000000.0;      // 场景值：GEO 真距量级
  cfg.track_bearing_init_range_std_m = 8000000.0;   // 场景值
  cfg.enable_track_filtering = true;
  fusion::FusionEngine engine(cfg);

  std::FILE* csv = nullptr;
  if (!csv_out.empty()) {
    csv = std::fopen(csv_out.c_str(), "w");
    if (csv != nullptr) {
      std::fprintf(csv, "cycle,has_estimate,pos_err_m,vel_err_m_per_s,"
                        "pos_sigma_m,vel_sigma_m_per_s,n_detections\n");
    }
  }

  std::vector<std::uint64_t> conv_cycles;
  std::vector<double> conv_pos_err;
  std::vector<double> conv_vel_err;
  std::vector<double> conv_pos_sigma;
  std::vector<double> conv_vel_sigma;
  std::size_t no_estimate_cycles = 0U;

  for (int cyc = 1; cyc <= max_cycle; ++cyc) {
    std::vector<fusion::DetectionRecord> detections;
    auto it = per_cycle.find(static_cast<std::uint32_t>(cyc));
    if (it != per_cycle.end()) {
      for (const auto& entry : it->second) {
        if (only_source != 0U && entry.first != only_source) {
          continue;
        }
        fusion::DetectionRecord d;
        d.key = target_id;  // 身份键直挂：两星量测并到同一航迹（对应场景"归属键直挂"）
        d.source_id = entry.first;
        d.has_bearing = true;
        d.bearing_az_deg = entry.second.first;
        d.bearing_el_deg = entry.second.second;
        d.has_sensor_origin = true;
        d.sensor_origin = sat_lla[entry.first];
        d.has_timestamp = true;
        d.timestamp_s = static_cast<double>(cyc) * 1.0;
        d.verdict = 1.0;
        d.quality = 1.0;
        detections.push_back(d);
      }
    }
    const std::vector<fusion::FusedTarget> targets =
        engine.Update(detections, static_cast<std::uint64_t>(cyc));

    const fusion::FusedTarget* me = nullptr;
    for (const auto& t : targets) {
      if (t.key == target_id) {
        me = &t;
        break;
      }
    }
    const bool has_est = (me != nullptr) && me->has_kinematic_estimate;
    double pos_err = 0.0;
    double vel_err = 0.0;
    double pos_sigma = 0.0;
    double vel_sigma = 0.0;
    if (has_est && truth_pos.count(static_cast<std::uint32_t>(cyc)) != 0U) {
      EcefPositionM est_ecef;
      if (oneq::coordinate::TryLlaToEcef(me->kinematic_estimate.position, &est_ecef)) {
        const EcefPositionM& tp = truth_pos[static_cast<std::uint32_t>(cyc)];
        pos_err = std::sqrt((est_ecef.x_m - tp.x_m) * (est_ecef.x_m - tp.x_m) +
                            (est_ecef.y_m - tp.y_m) * (est_ecef.y_m - tp.y_m) +
                            (est_ecef.z_m - tp.z_m) * (est_ecef.z_m - tp.z_m));
        const auto& cov = me->kinematic_estimate.covariance_ecef;
        pos_sigma = std::sqrt(std::max(0.0, 0.5 * (cov[0] + cov[14] + cov[28])));
        vel_sigma = std::sqrt(std::max(0.0, 0.5 * (cov[7] + cov[21] + cov[35])));
        auto tv = truth_vel.find(static_cast<std::uint32_t>(cyc));
        if (tv != truth_vel.end()) {
          const auto& v = me->kinematic_estimate.velocity_ecef_m_per_s;
          vel_err = std::sqrt((v[0] - tv->second.x_mps) * (v[0] - tv->second.x_mps) +
                              (v[1] - tv->second.y_mps) * (v[1] - tv->second.y_mps) +
                              (v[2] - tv->second.z_mps) * (v[2] - tv->second.z_mps));
          conv_vel_err.push_back(vel_err);
          conv_vel_sigma.push_back(vel_sigma);
        }
        conv_cycles.push_back(static_cast<std::uint64_t>(cyc));
        conv_pos_err.push_back(pos_err);
        conv_pos_sigma.push_back(pos_sigma);
      }
    } else if (!has_est) {
      ++no_estimate_cycles;
    }
    if (csv != nullptr) {
      std::fprintf(csv, "%d,%d,%.3f,%.3f,%.3f,%.3f,%zu\n", cyc, has_est ? 1 : 0, pos_err, vel_err,
                   pos_sigma, vel_sigma,
                   it == per_cycle.end() ? static_cast<std::size_t>(0U) : it->second.size());
    }
  }
  if (csv != nullptr) {
    std::fclose(csv);
    std::printf("\nCSV: %s\n", csv_out.c_str());
  }

  // ---- 逐周期表（前 25 拍）----
  std::printf("\n表 1  逐周期误差与协方差（1σ）\n");
  std::printf("%-7s %-14s %-16s %-14s %-16s\n", "周期", "位置误差(m)", "速度误差(m/s)",
              "位置1σ(m)", "速度1σ(m/s)");
  for (std::size_t i = 0; i < conv_cycles.size() && i < 25U; ++i) {
    std::printf("%-7llu %-14.1f %-16.1f %-14.1f %-16.1f\n",
                static_cast<unsigned long long>(conv_cycles[i]), conv_pos_err[i],
                (i < conv_vel_err.size() ? conv_vel_err[i] : 0.0), conv_pos_sigma[i],
                (i < conv_vel_sigma.size() ? conv_vel_sigma[i] : 0.0));
  }

  // ---- 阶段统计 ----
  auto slice = [&](std::size_t from, std::size_t to) {
    std::vector<double> pe(conv_pos_err.begin() + static_cast<long>(from),
                           conv_pos_err.begin() + static_cast<long>(std::min(to, conv_pos_err.size())));
    std::vector<double> ve(conv_vel_err.begin() + static_cast<long>(from),
                           conv_vel_err.begin() + static_cast<long>(std::min(to, conv_vel_err.size())));
    return std::make_pair(Summarize(pe), Summarize(ve));
  };

  std::printf("\n表 2  分阶段统计（速度真值来自真值轨迹中心差分）\n");
  std::printf("%-22s %-24s %-24s\n", "阶段", "位置误差 均值/P95 (m)", "速度误差 均值/P95 (m/s)");
  const struct {
    const char* name;
    std::size_t from;
    std::size_t to;
  } stages[] = {
      {"第 1-5 拍（起始）", 0U, 5U},
      {"第 6-15 拍", 5U, 15U},
      {"第 16-40 拍", 15U, 40U},
      {"第 41 拍以后（稳态）", 40U, 10000U},
  };
  for (const auto& st : stages) {
    if (st.from >= conv_pos_err.size()) {
      continue;
    }
    const auto pr = slice(st.from, st.to);
    std::printf("%-22s %-24s %-24s\n", st.name,
                (std::to_string(static_cast<long>(pr.first.mean)) + " / " +
                 std::to_string(static_cast<long>(pr.first.p95))).c_str(),
                (std::to_string(static_cast<long>(pr.second.mean)) + " / " +
                 std::to_string(static_cast<long>(pr.second.p95))).c_str());
  }

  const Stats vs = Summarize(conv_vel_err);
  const Stats vss = Summarize(conv_vel_sigma);
  std::printf("\n表 3  速度估计总览\n");
  std::printf("  有估计周期数 = %zu（无估计 %zu 拍；总 %d 拍）\n", conv_vel_err.size(),
              no_estimate_cycles, max_cycle);
  std::printf("  速度误差   均值 %.1f  中位 %.1f  P95 %.1f  (m/s)\n", vs.mean, vs.p50, vs.p95);
  std::printf("  速度协方差 均值 %.1f  中位 %.1f  P95 %.1f  (m/s, 1σ)\n", vss.mean, vss.p50,
              vss.p95);
  double tgt_speed = 0.0;
  {
    auto it = truth_vel.begin();
    if (it != truth_vel.end()) {
      tgt_speed = std::sqrt(it->second.x_mps * it->second.x_mps +
                            it->second.y_mps * it->second.y_mps +
                            it->second.z_mps * it->second.z_mps);
    }
  }
  std::printf("  目标真值速度量级 ~ %.0f m/s -> 速度相对误差 ~ %.1f%%\n", tgt_speed,
              tgt_speed > 0.0 ? 100.0 * vs.mean / tgt_speed : 0.0);
  std::printf("  协方差一致性：报告 1σ %.1f vs 实测误差均值 %.1f（比值 %.2f，接近 1 表示如实）\n",
              vss.mean, vs.mean, vs.mean > 0.0 ? vss.mean / vs.mean : 0.0);
  return 0;
}
