/**
 * @file measure_rir_from_external_fix.cpp
 * @brief 开发期度量：把「外部双星交会位置」喂给 RIR，实测雷达能否指向/探测/跟踪/识别。
 *
 * 被测链路（与外部位置录入接口的目标形态一致）：
 *   外部位置（含误差）-> 转雷达局部 ENU -> 反算指向 -> 运行期补丁设 scan_center
 *     -> RIR 逐周期探测 -> 建航确认 -> 识别积累 -> 识别结论
 *
 * 场景：雷达位于原点，目标置于正东 range、高度 alt。位置误差按「横向偏移 delta」
 * 注入——delta 是双星交会误差的主导分量，且直接等价于指向误差 atan(delta/斜距)。
 * 目标真值始终在真实位置；带误差的位置只用于**指向**，因此重演真实链路：
 * 用有误差的位置去指，看能不能照到真目标。
 *
 * 两种指向模式（--mode）：
 *   search    仅设 scan_center（等价"设定方位俯仰进行扫描"）。波束按波位表推进，
 *             目标只在少数驻留被照到，其余周期报 rir.target_outside_beam_coverage。
 *   designate 同时下发指定目标 ID（人工指定优先，每周期首个驻留对准目标）。
 *             用于测量"持续照射"下的探测/跟踪/识别能力上限。
 *
 * 判定量取 RirOutputDebugView（帧作用域快照，不改变执行语义）：
 *   探测 = cycle_detection_snr_db 有效；跟踪 = has_track + hit_count；
 *   识别 = has_recognition_output + 置信度 + 观测数。
 *
 * 用法：
 *   measure_rir_from_external_fix --db <特征库> [--range-km 200] [--cycles 25]
 *                                 [--mode search|designate]
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirOutputDebugView.h"
#include "1q/remote_identification_radar/session/RirSession.h"

namespace {

using remote_identification_radar::config::RirAzimuthElevationDeg;
using remote_identification_radar::config::RirRuntimeConfigPatch;
using remote_identification_radar::config::RirSessionConfig;
using remote_identification_radar::config::RirWorkMode;
using remote_identification_radar::session::RirCycleInput;
using remote_identification_radar::session::RirCycleResult;
using remote_identification_radar::session::RirCycleStatus;
using remote_identification_radar::session::RirOutputDebugView;
using remote_identification_radar::session::RirSceneTarget;
using remote_identification_radar::session::RirSession;
using remote_identification_radar::session::RirSwerlingType;

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kEarthRadiusM = 6378137.0;
constexpr std::uint64_t kTargetId = 1U;

enum class AimMode { kSearch, kDesignate };

struct RunOutcome {
  std::size_t executed_cycles{0U};
  std::size_t excluded_cycles{0U};   // 报 outside_beam_coverage 的周期
  std::size_t detect_cycles{0U};     // 检测链产出 SNR 的周期
  std::size_t track_cycles{0U};      // 归属表存在航迹的周期
  std::size_t recognition_cycles{0U};
  std::uint32_t max_hit_count{0U};
  std::size_t first_detect{0U};
  std::size_t first_track{0U};
  float best_confidence{0.0f};
  std::uint32_t max_observation_count{0U};
  double mean_snr_db{0.0};
  int patch_ok{-1};
  double dwell_az{0.0};
  double dwell_el{0.0};
  double last_dwell_az{0.0};
  double last_dwell_el{0.0};
  std::string first_issues{};
  std::string pattern;  // 'D' 检出 / 'X' 排除 / '.' 其它
};

RunOutcome RunOnce(const std::string& db_path, double range_m, double altitude_m,
                   double lateral_offset_m, std::size_t cycles, AimMode mode) {
  RunOutcome out;

  RirSessionConfig cfg;
  cfg.mission.work_mode = RirWorkMode::kIdentify;
  cfg.mission.max_range_m = static_cast<float>(range_m * 2.5);
  cfg.mission.recognition_dwell_sec = 0.05f;
  cfg.policy.recognition.enabled = true;
  cfg.policy.recognition.database_path = db_path;
  // 默认可扫描体积 az +-60 / el +-30 装不下仰角约 45 度的目标，会被体积门排除；
  // 本度量测的是「位置精度能否支撑指向」，故放开到全空域。
  cfg.orientation.az_min_deg = -180.0f;
  cfg.orientation.az_max_deg = 180.0f;
  cfg.orientation.el_min_deg = -90.0f;
  cfg.orientation.el_max_deg = 90.0f;

  RirSession session = RirSession::Create(cfg);

  const float truth_x = static_cast<float>(range_m);
  const float truth_y = 0.0f;
  const float truth_z = static_cast<float>(altitude_m);
  // 目标真值按库内模板 BALLISTIC_EXAMPLE_A 反推，使识别链**有据可依**：
  //   rcs mean -3 dBsm (0.5 m2) / speed 1800 m/s / altitude 50000 m /
  //   polarization energy_difference 2.0 dB (hh/vv = +2/0 dBsm) /
  //   aspect 覆盖 >= 15 deg（模板 minimum_aspect_coverage_deg）。
  constexpr float kRcsM2 = 0.5f;        // 10log10(0.5) = -3.01 dBsm
  constexpr float kSpeedMps = 1800.0f;  // 模板 speed_mean

  // 指向 = 外部给的位置（含横向误差 delta）的方向。
  const double aim_x = range_m;
  const double aim_y = lateral_offset_m;
  const double aim_z = altitude_m;
  const double aim_norm = std::sqrt(aim_x * aim_x + aim_y * aim_y + aim_z * aim_z);
  RirAzimuthElevationDeg scan_center;
  scan_center.az_deg = static_cast<float>(std::atan2(aim_y, aim_x) * kRadToDeg);
  scan_center.el_deg = static_cast<float>(std::asin(aim_z / aim_norm) * kRadToDeg);

  for (std::size_t k = 0; k < cycles; ++k) {
    RirRuntimeConfigPatch patch;
    patch.has_scan_center = true;
    patch.scan_center_deg = scan_center;
    if (mode == AimMode::kDesignate) {
      patch.has_designated_target_id = true;
      patch.designated_external_target_id = kTargetId;
    }
    const bool ok = session.TryApplyRuntimeConfig(patch);

    RirCycleInput input;
    input.input_cycle_index = static_cast<std::uint32_t>(k + 1U);
    input.dt_sec = 0.5;
    input.sim_time_sec = static_cast<float>(k) * 0.5f;
    input.platform_position = oneq::coordinate::EcefPositionM(kEarthRadiusM, 0.0, 0.0);

    RirSceneTarget target;
    target.external_target_id = kTargetId;
    target.target_name = "ext_fix_target";
    target.position_x = truth_x;
    target.position_y = truth_y;
    target.position_z = truth_z;
    target.velocity_x = 0.0f;
    target.velocity_y = kSpeedMps;
    target.velocity_z = 0.0f;
    target.rcs = kRcsM2;
    target.target_swerling_type = RirSwerlingType::kSwerling0;
    // 视角 RCS 样本：网格覆盖 20deg x 25deg（>= 模板要求的 15deg 覆盖）
    for (float az = -10.0f; az <= 10.0f; az += 5.0f) {
      for (float el = 5.0f; el <= 30.0f; el += 10.0f) {
        remote_identification_radar::session::RirAspectRcsSample aspect;
        aspect.aspect_az_deg = az;
        aspect.aspect_el_deg = el;
        aspect.rcs_dbsm = -3.0f;  // 模板 rcs mean -3 dBsm
        target.aspect_rcs_samples.push_back(aspect);
      }
    }
    // 极化散射样本：交叉通道置零，共极化差 2.0 dB（模板 energy_difference 2.0 dB）
    {
      remote_identification_radar::session::RirPolSMatrixSample pol;
      pol.aspect_az_deg = 0.0f;
      pol.aspect_el_deg = static_cast<float>(std::atan2(altitude_m, range_m) * kRadToDeg);
      pol.hh_amp_db = 2.0f;
      pol.hh_phase_deg = 0.0f;
      pol.hv_amp_db = -30.0f;
      pol.hv_phase_deg = 0.0f;
      pol.vh_amp_db = -30.0f;
      pol.vh_phase_deg = 0.0f;
      pol.vv_amp_db = 0.0f;
      pol.vv_phase_deg = 0.0f;
      target.polarization_samples.push_back(pol);
    }
    input.scene_targets.push_back(target);

    const RirCycleResult result = session.StepWithResult(input);
    if (result.status != RirCycleStatus::kCompleted) {
      continue;
    }
    ++out.executed_cycles;

    const RirOutputDebugView view =
        remote_identification_radar::session::RirOutputDebugViewBuilder::Build(input, result);
    if (out.patch_ok < 0) {
      out.patch_ok = ok ? 1 : 0;
      out.dwell_az = static_cast<double>(result.dwell_center_deg.az_deg);
      out.dwell_el = static_cast<double>(result.dwell_center_deg.el_deg);
      for (const auto& iss : result.issues) {
        out.first_issues += "[" + iss.code + "]";
      }
    }
    bool excluded = false;
    for (const auto& iss : result.issues) {
      if (iss.code == "rir.target_outside_beam_coverage") {
        excluded = true;
        break;
      }
    }
    if (excluded) {
      ++out.excluded_cycles;
    }
    out.last_dwell_az = static_cast<double>(result.dwell_center_deg.az_deg);
    out.last_dwell_el = static_cast<double>(result.dwell_center_deg.el_deg);
    {
      bool det = false;
      for (const auto& t1 : view.targets) {
        if (t1.external_target_id == kTargetId && t1.has_cycle_detection_snr) {
          det = true;
        }
      }
      out.pattern += det ? 'D' : (excluded ? 'X' : '.');
    }

    for (const auto& t : view.targets) {
      if (t.external_target_id != kTargetId) {
        continue;
      }
      if (t.has_cycle_detection_snr) {
        ++out.detect_cycles;
        out.mean_snr_db += t.cycle_detection_snr_db;
        if (out.first_detect == 0U) {
          out.first_detect = k + 1U;
        }
      }
      if (t.has_track) {
        ++out.track_cycles;
        if (out.first_track == 0U) {
          out.first_track = k + 1U;
        }
        if (t.hit_count > out.max_hit_count) {
          out.max_hit_count = t.hit_count;
        }
      }
      if (t.has_recognition_output) {
        ++out.recognition_cycles;
        if (t.confidence > out.best_confidence) {
          out.best_confidence = t.confidence;
        }
        if (t.observation_count > out.max_observation_count) {
          out.max_observation_count = t.observation_count;
        }
      }
    }
  }
  if (out.detect_cycles > 0U) {
    out.mean_snr_db /= static_cast<double>(out.detect_cycles);
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  std::string db_path;
  double range_km = 200.0;
  std::size_t cycles = 25U;
  AimMode mode = AimMode::kDesignate;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--db") == 0 && i + 1 < argc) {
      db_path = argv[++i];
    } else if (std::strcmp(argv[i], "--range-km") == 0 && i + 1 < argc) {
      range_km = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--cycles") == 0 && i + 1 < argc) {
      cycles = static_cast<std::size_t>(std::atol(argv[++i]));
    } else if (std::strcmp(argv[i], "--mode") == 0 && i + 1 < argc) {
      mode = (std::strcmp(argv[++i], "search") == 0) ? AimMode::kSearch : AimMode::kDesignate;
    }
  }
  if (db_path.empty()) {
    std::fprintf(stderr,
                 "usage: measure_rir_from_external_fix --db <db> [--range-km 200] "
                 "[--cycles 25] [--mode search|designate]\n");
    return 2;
  }

  const double range_m = range_km * 1000.0;
  const double altitude_m = 50000.0;  // 模板 altitude_mean = 50000 m
  const char* mode_name = (mode == AimMode::kSearch) ? "search" : "designate";

  std::printf("=== RIR end-to-end driven by external (dual-sat) position ===\n");
  std::printf("mode=%s  range=%.0fkm  alt=%.0fkm  cycles=%zu  dt=0.5s\n", mode_name, range_km,
              altitude_m / 1000.0, cycles);
  std::printf("beam: default HPBW 4.0deg -> 3dB(round-trip) tolerance ~1.4deg\n");
  std::printf("pointing error = atan(delta / range)\n\n");

  {
    const RunOutcome p = RunOnce(db_path, range_m, altitude_m, 0.0, 3U, mode);
    std::printf("[DIAG delta=0] patch_ok=%d executed=%zu dwell=(%.3f,%.3f) excluded=%zu issues=%s\n",
                p.patch_ok, p.executed_cycles, p.dwell_az, p.dwell_el, p.excluded_cycles,
                p.first_issues.c_str());
    std::printf("%s\n", std::string(112, '-').c_str());
  }

  const std::vector<double> offsets_m = {0.0,    375.0,   1000.0,  2000.0,  3000.0, 5000.0,
                                         7000.0, 10000.0, 15000.0, 30000.0};

  std::printf("%-10s %-9s %-7s %-7s %-8s %-7s %-7s %-8s %-7s %-8s\n", "delta(m)", "pt_err",
              "exec", "excl", "detect", "track", "hits", "recogn", "obs", "conf");
  std::printf("%s\n", std::string(112, '-').c_str());
  for (double off : offsets_m) {
    const RunOutcome o = RunOnce(db_path, range_m, altitude_m, off, cycles, mode);
    const double pt = std::atan(off / range_m) * kRadToDeg;
    const char* verdict = pt < 1.4 ? "OK" : (pt < 2.0 ? "edge" : "FAIL");
    std::printf("%-10.0f %-9.3f %-7zu %-7zu %-8zu %-7zu %-7u %-8zu %-7u %-8.3f %s\n", off, pt,
                o.executed_cycles, o.excluded_cycles, o.detect_cycles, o.track_cycles,
                o.max_hit_count, o.recognition_cycles, o.max_observation_count,
                static_cast<double>(o.best_confidence), verdict);
    std::printf("           pattern=%s  last_dwell=(%.3f,%.3f)\n", o.pattern.c_str(),
                o.last_dwell_az, o.last_dwell_el);
  }

  std::printf("\nexec=executed cycles, excl=cycles reporting outside_beam_coverage,\n");
  std::printf("detect=cycles with detection SNR, track=cycles with a track,\n");
  std::printf("hits=max hit_count (confirm gate=3), recogn=cycles with recognition output,\n");
  std::printf("obs=max observation_count (min=3), conf=best recognition score (gate=0.70)\n");
  return 0;
}
