/**
 * @file measure_external_track.cpp
 * @brief 开发期实测：场景真值驱动外部跟踪接口（位置 + 速度 + 中心邻域窗口）。
 *
 * 本轮实测的目的（承接
 * docs/review/ir_external_position_driven_rir_e2e_2026-10-04.md）：
 * 上一轮已证「仅用 scan_center（扫描基准语义）零偏移也 0/25 检出，且失败静默」。
 * 本轮改用**新的外部跟踪接口**（照射中心 + 邻域窗口，按速度外推），用场景真值
 * 作为输入以隔离机理验证——先确认机制能通，再叠加位置误差。
 *
 * 三种指向模式的对照（同一几何、同一目标、同一特征真值）：
 *   scan      仅设 scan_center（旧形态，预期 0 检出）
 *   designate 下发指定目标 ID（人工指定语义，上一轮的 100% 基准）
 *   external  新接口 SubmitExternalTrack（本轮被测对象）
 *
 * 判定量取 RirOutputDebugView 与 RirCycleResult 的加性可观测量：
 *   探测 = cycle_detection_snr_db 有效；跟踪 = has_track + hit_count；
 *   识别 = has_recognition_output + 置信度 + 观测数；
 *   指向 = pointing_source / external_track_phase / external_track_center_deg。
 *
 * 用法：
 *   measure_external_track --db <特征库> [--range-km 200] [--cycles 25]
 *                          [--mode scan|designate|external|all]
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirOutputDebugView.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "1q/remote_identification_radar/session/RirTrackRequest.h"

namespace {

using remote_identification_radar::config::RirAzimuthElevationDeg;
using remote_identification_radar::config::RirRuntimeConfigPatch;
using remote_identification_radar::config::RirSessionConfig;
using remote_identification_radar::config::RirWorkMode;
using remote_identification_radar::session::RirCycleInput;
using remote_identification_radar::session::RirCycleResult;
using remote_identification_radar::session::RirCycleStatus;
using remote_identification_radar::session::RirExternalTrackPhase;
using remote_identification_radar::session::RirExternalTrackRequest;
using remote_identification_radar::session::RirOutputDebugView;
using remote_identification_radar::session::RirPointingSource;
using remote_identification_radar::session::RirSceneTarget;
using remote_identification_radar::session::RirSession;
using remote_identification_radar::session::RirSwerlingType;

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr std::uint64_t kTargetId = 1U;
// WGS-84
constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);

// 目标真值按库内模板 BALLISTIC_EXAMPLE_A 反推，使识别链有据可依。
constexpr float kRcsM2 = 0.5f;        // 10log10(0.5) = -3.01 dBsm
constexpr float kSpeedMps = 1800.0f;  // 模板 speed_mean
constexpr double kAltitudeM = 50000.0;
constexpr double kDtSec = 0.5;

enum class AimMode { kScan, kDesignate, kExternal };

struct RunOutcome {
  std::size_t executed{0U};
  std::size_t excluded{0U};
  std::size_t detected{0U};
  std::size_t tracked{0U};
  std::size_t recognized{0U};
  std::uint32_t max_hits{0U};
  std::uint32_t max_obs{0U};
  float best_confidence{0.0f};
  std::size_t external_cycles{0U};
  std::size_t fresh_cycles{0U};
  std::size_t coasting_cycles{0U};
  std::size_t expired_cycles{0U};
};

/** @brief 平台锚点 LLA + 局部 ENU 偏移 -> ECEF（与库内 ENU 口径一致的独立实现）。 */
oneq::coordinate::EcefPositionM EnuToEcef(const oneq::coordinate::LlaPositionDegM& anchor,
                                          double east_m, double north_m, double up_m) {
  const double lat = anchor.latitude_deg * kDegToRad;
  const double lon = anchor.longitude_deg * kDegToRad;
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);
  const double n = kA / std::sqrt(1.0 - kE2 * sin_lat * sin_lat);
  const double h = anchor.altitude_m;
  // 锚点 ECEF
  const double ax = (n + h) * cos_lat * cos_lon;
  const double ay = (n + h) * cos_lat * sin_lon;
  const double az = (n * (1.0 - kE2) + h) * sin_lat;
  // ENU 基（东/北/天）
  const double e[3] = {-sin_lon, cos_lon, 0.0};
  const double nn[3] = {-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat};
  const double u[3] = {cos_lat * cos_lon, cos_lat * sin_lon, sin_lat};
  oneq::coordinate::EcefPositionM out;
  out.x_m = ax + east_m * e[0] + north_m * nn[0] + up_m * u[0];
  out.y_m = ay + east_m * e[1] + north_m * nn[1] + up_m * u[1];
  out.z_m = az + east_m * e[2] + north_m * nn[2] + up_m * u[2];
  return out;
}

/** @brief 目标在周期 k 的局部 ENU 真值位置（匀速沿 +北穿越）。 */
void TruthEnu(std::size_t k, double range_m, double* east, double* north, double* up) {
  *east = range_m;
  *north = static_cast<double>(k) * kDtSec * kSpeedMps;
  *up = kAltitudeM;
}

RirSessionConfig MakeConfig(const std::string& db_path, double range_m) {
  RirSessionConfig cfg;
  cfg.mission.work_mode = RirWorkMode::kIdentify;
  cfg.mission.max_range_m = static_cast<float>(range_m * 2.5);
  cfg.mission.recognition_dwell_sec = 0.05f;
  cfg.policy.recognition.enabled = true;
  cfg.policy.recognition.database_path = db_path;
  // 默认可扫描体积 az ±60 / el ±30 装不下仰角约 14 度的目标之外的场景；
  // 本度量测「指向语义」，故放开到全空域。
  cfg.orientation.az_min_deg = -180.0f;
  cfg.orientation.az_max_deg = 180.0f;
  cfg.orientation.el_min_deg = -90.0f;
  cfg.orientation.el_max_deg = 90.0f;
  return cfg;
}

/** @brief 铺齐识别特征真值（照 BALLISTIC_EXAMPLE_A 模板）。 */
void PopulateFeatures(RirSceneTarget* target) {
  for (float az = -10.0f; az <= 10.0f; az += 5.0f) {
    for (float el = 5.0f; el <= 30.0f; el += 10.0f) {
      remote_identification_radar::session::RirAspectRcsSample aspect;
      aspect.aspect_az_deg = az;
      aspect.aspect_el_deg = el;
      aspect.rcs_dbsm = -3.0f;  // 模板 rcs mean
      target->aspect_rcs_samples.push_back(aspect);
    }
  }
  remote_identification_radar::session::RirPolSMatrixSample pol;
  pol.aspect_az_deg = 0.0f;
  pol.aspect_el_deg = 14.0f;
  pol.hh_amp_db = 2.0f;  // 模板 energy_difference = 2.0 dB
  pol.hh_phase_deg = 0.0f;
  pol.hv_amp_db = -60.0f;
  pol.hv_phase_deg = 0.0f;
  pol.vh_amp_db = -60.0f;
  pol.vh_phase_deg = 0.0f;
  pol.vv_amp_db = 0.0f;
  pol.vv_phase_deg = 0.0f;
  target->polarization_samples.push_back(pol);
}

RunOutcome RunOnce(const std::string& db_path, double range_m, std::size_t cycles, AimMode mode) {
  RunOutcome out;
  RirSession session = RirSession::Create(MakeConfig(db_path, range_m));

  oneq::coordinate::LlaPositionDegM anchor{};
  anchor.latitude_deg = 30.0;
  anchor.longitude_deg = 120.0;
  anchor.altitude_m = 1000.0;

  for (std::size_t k = 0; k < cycles; ++k) {
    const double sim_time = static_cast<double>(k) * kDtSec;
    double e = 0.0;
    double n = 0.0;
    double u = 0.0;
    TruthEnu(k, range_m, &e, &n, &u);
    const oneq::coordinate::EcefPositionM target_ecef = EnuToEcef(anchor, e, n, u);

    // ---- 指向下发（三种模式）----
    if (mode == AimMode::kScan || mode == AimMode::kDesignate) {
      // 旧形态：仅改扫描基准（方位原点）；designate 另加指定目标 ID。
      const double hypot = std::sqrt(e * e + n * n);
      RirRuntimeConfigPatch patch;
      patch.has_scan_center = true;
      patch.scan_center_deg.az_deg = static_cast<float>(std::atan2(n, e) * kRadToDeg);
      patch.scan_center_deg.el_deg = static_cast<float>(std::atan2(u, hypot) * kRadToDeg);
      if (mode == AimMode::kDesignate) {
        patch.has_designated_target_id = true;
        patch.designated_external_target_id = kTargetId;
      }
      session.TryApplyRuntimeConfig(patch);
    } else {
      // 新接口：位置 + 速度 + 邻域窗口；库内按速度外推到本周期。
      RirExternalTrackRequest request;
      request.position_ecef_m = target_ecef;
      // 速度沿 ENU 北向；用相邻周期真值差分给出 ECEF 速度（避免手写换系）。
      double e2 = 0.0;
      double n2 = 0.0;
      double u2 = 0.0;
      TruthEnu(k + 1U, range_m, &e2, &n2, &u2);
      const oneq::coordinate::EcefPositionM next_ecef = EnuToEcef(anchor, e2, n2, u2);
      request.velocity_ecef_mps.x_mps = (next_ecef.x_m - target_ecef.x_m) / kDtSec;
      request.velocity_ecef_mps.y_mps = (next_ecef.y_m - target_ecef.y_m) / kDtSec;
      request.velocity_ecef_mps.z_mps = (next_ecef.z_m - target_ecef.z_m) / kDtSec;
      request.source_time_s = sim_time;
      request.window.az_half_deg = 8.0f;
      request.window.el_half_deg = 4.0f;
      request.source_target_id = kTargetId;
      request.source_entity_id = 4U;
      session.SubmitExternalTrack(request);
    }

    RirCycleInput input;
    input.input_cycle_index = static_cast<std::uint32_t>(k + 1U);
    input.dt_sec = kDtSec;
    input.sim_time_sec = static_cast<float>(sim_time);
    oneq::coordinate::LlaPositionDegM pa = anchor;
    oneq::coordinate::TryLlaToEcef(pa, &input.platform_position);

    RirSceneTarget target;
    target.external_target_id = kTargetId;
    target.target_name = "ext_truth_target";
    target.position_x = static_cast<float>(e);
    target.position_y = static_cast<float>(n);
    target.position_z = static_cast<float>(u);
    target.velocity_x = 0.0f;
    target.velocity_y = kSpeedMps;
    target.velocity_z = 0.0f;
    target.rcs = kRcsM2;
    target.target_swerling_type = RirSwerlingType::kSwerling0;
    PopulateFeatures(&target);
    input.scene_targets.push_back(target);

    const RirCycleResult result = session.StepWithResult(input);
    if (result.status != RirCycleStatus::kCompleted) {
      continue;
    }
    ++out.executed;
    switch (result.external_track_phase) {
      case RirExternalTrackPhase::kFresh:
        ++out.fresh_cycles;
        break;
      case RirExternalTrackPhase::kCoasting:
        ++out.coasting_cycles;
        break;
      case RirExternalTrackPhase::kExpired:
        ++out.expired_cycles;
        break;
      case RirExternalTrackPhase::kNone:
      default:
        break;
    }
    if (result.pointing_source == RirPointingSource::kExternalTrack) {
      ++out.external_cycles;
    }
    for (const auto& iss : result.issues) {
      if (iss.code == "rir.target_outside_beam_coverage") {
        ++out.excluded;
        break;
      }
    }
    const RirOutputDebugView view =
        remote_identification_radar::session::RirOutputDebugViewBuilder::Build(input, result);
    for (const auto& t : view.targets) {
      if (t.external_target_id != kTargetId) {
        continue;
      }
      if (t.has_cycle_detection_snr) {
        ++out.detected;
      }
      if (t.has_track) {
        ++out.tracked;
        if (t.hit_count > out.max_hits) {
          out.max_hits = t.hit_count;
        }
      }
      if (t.has_recognition_output) {
        ++out.recognized;
        if (t.confidence > out.best_confidence) {
          out.best_confidence = t.confidence;
        }
        if (t.observation_count > out.max_obs) {
          out.max_obs = t.observation_count;
        }
      }
    }
  }
  return out;
}

const char* ModeName(AimMode mode) {
  switch (mode) {
    case AimMode::kScan:
      return "scan";
    case AimMode::kDesignate:
      return "designate";
    case AimMode::kExternal:
    default:
      return "external";
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string db_path;
  double range_km = 200.0;
  std::size_t cycles = 25U;
  std::string mode_arg = "all";
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--db") == 0 && i + 1 < argc) {
      db_path = argv[++i];
    } else if (std::strcmp(argv[i], "--range-km") == 0 && i + 1 < argc) {
      range_km = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--cycles") == 0 && i + 1 < argc) {
      cycles = static_cast<std::size_t>(std::atol(argv[++i]));
    } else if (std::strcmp(argv[i], "--mode") == 0 && i + 1 < argc) {
      mode_arg = argv[++i];
    }
  }
  if (db_path.empty()) {
    std::fprintf(stderr,
                 "usage: measure_external_track --db <db> [--range-km 200] [--cycles 25] "
                 "[--mode scan|designate|external|all]\n");
    return 2;
  }

  const double range_m = range_km * 1000.0;
  std::vector<AimMode> modes;
  if (mode_arg == "all") {
    modes = {AimMode::kScan, AimMode::kDesignate, AimMode::kExternal};
  } else if (mode_arg == "scan") {
    modes = {AimMode::kScan};
  } else if (mode_arg == "designate") {
    modes = {AimMode::kDesignate};
  } else {
    modes = {AimMode::kExternal};
  }

  std::printf("=== external-track interface driven by scenario truth ===\n");
  std::printf("range=%.0fkm alt=%.0fkm speed=%.0fkm/s cycles=%zu dt=%.1fs\n", range_km,
              kAltitudeM / 1000.0, kSpeedMps / 1000.0, cycles, kDtSec);
  std::printf("target truth follows template BALLISTIC_EXAMPLE_A "
              "(rcs -3dBsm / 1800m/s / 50km / 3 feature dims populated)\n\n");

  for (AimMode mode : modes) {
    const RunOutcome o = RunOnce(db_path, range_m, cycles, mode);
    std::printf("--- mode=%s ---\n", ModeName(mode));
    std::printf("  exec=%zu excl=%zu detect=%zu track=%zu hits=%u recogn=%zu obs=%u conf=%.3f\n",
                o.executed, o.excluded, o.detected, o.tracked, o.max_hits, o.recognized,
                o.max_obs, static_cast<double>(o.best_confidence));
    if (mode == AimMode::kExternal) {
      std::printf("  pointing=external %zu/%zu cycles; phase fresh=%zu coasting=%zu expired=%zu\n",
                  o.external_cycles, o.executed, o.fresh_cycles, o.coasting_cycles,
                  o.expired_cycles);
    }
    std::printf("\n");
  }

  std::printf("legend: excl=cycles reporting outside_beam_coverage; detect=cycles with SNR;\n");
  std::printf("        hits=max hit_count (confirm gate=3); obs=max observation_count (min=3);\n");
  std::printf("        conf=best recognition score (gate=0.70)\n");
  return 0;
}
