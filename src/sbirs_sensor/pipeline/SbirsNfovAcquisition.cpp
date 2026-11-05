/**
 * ## 过程：窄场首次捕获判定（红外处理链 · 第 13 步）
 * - 上游：pipeline 组装的 `SbirsNfovAcquisitionRequest` —— 延迟真值 ECI 角（已经安装矩阵链折到
 *   传感器系）、指令指向角（第 12 步稳定后的实际指向）、`mission.narrow_pointing_settle_error_deg`
 *   与 `narrow_field_fov_az_deg`/`narrow_field_fov_el_deg`、候选 SNR（第 5 步）与
 *   `policy.detection.narrow_min_snr_linear`。
 * - 本步：两条判据同时成立才算捕获：
 *   · 视场判据：|Δaz| ≤ ½·FOV_az 且 |Δel| ≤ ½·FOV_el，Δaz 按 ±180° 回绕，比较基准是
 *     "指令角 + 静态 settle 误差"；
 *   · SNR 判据：SNR ≥ 窄场门限（默认 6，要求比宽场更低的虚警率）。
 * - 下游：pipeline 依此决定进入窄场驻留/跟踪（并清掉捕获等待计时），失败则写捕获失败原因与
 *   子判据（outside-fov / snr-below）并释放本次调度。
 * - 关键参数：`mission.narrow_field_fov_az_deg`/`narrow_field_fov_el_deg`、
 *   `mission.narrow_pointing_settle_error_deg`(0)、`policy.detection.narrow_min_snr_linear`(6)。
 * - 对应验收项：第 13 项 窄视场跟踪探测功能测试（捕获门）；第 14 项 宽窄视场联合探测功能测试
 *   （捕获结论与转向时间）；第 76 项 红外宽场候选与窄场调度交互接口测试（捕获判决输入）。
 */

#include "sbirs_sensor/pipeline/SbirsNfovAcquisition.h"

#include <cmath>

namespace sbirs_sensor {
namespace pipeline {
namespace {

// 过程：方位差折到 (−180°, 180°] —— 否则 359°→1° 这类跨零残差会被算成 358° 而误判出视场。
float NormalizeAzimuth(float azimuth_deg) {
  float result = std::fmod(azimuth_deg + 180.0f, 360.0f);
  if (result < 0.0f) {
    result += 360.0f;
  }
  return result - 180.0f;
}

}  // namespace

// 过程：比较基准 = 指令角 + 静态 settle 误差（视为已知常值指向偏差），真值角与之作差。
bool IsNfovAcquisitionEligible(const SbirsNfovAcquisitionRequest& request) {
  const float cue_azimuth_deg = request.command_azimuth_deg + request.pointing_settle_error_deg;
  const bool in_nfov =
      std::fabs(NormalizeAzimuth(request.delayed_truth_azimuth_deg - cue_azimuth_deg)) <=
          0.5f * request.field_of_view_azimuth_deg &&
      std::fabs(request.delayed_truth_elevation_deg - request.command_elevation_deg) <=
          0.5f * request.field_of_view_elevation_deg;
  // 过程：两条判据取与 —— 视场内且 SNR 过门；任一不满足都不算捕获（子判据由调用方重算诊断）。
  return in_nfov && request.snr >= request.minimum_snr_linear;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
