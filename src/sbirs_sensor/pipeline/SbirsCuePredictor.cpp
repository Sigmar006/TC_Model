/**
 * ## 过程：引导预测（WFOV 量测 → NFOV 指向命令）（红外处理链 · 第 8 步）
 * - 上游：本拍 WFOV 带误差量测角（第 7 步输出）、周期步长 dt、`mission.narrow_cue_latency_s`；
 *   目标是判断"延迟补偿之后目标会在哪"，不使用真值位置/速度。
 * - 本步：相邻两拍量测差分得角速度 ω = Δθ/dt（方位走最短角差），再按 cue 延迟外推
 *   θ_cmd = θ_meas + ω·τ；首拍、dt/延迟非法或量测非法时退化为直接指向本拍量测。
 * - 下游：pipeline 的候选命令角（`SbirsCandidate::command_azimuth_deg`/`command_elevation_deg`）
 *   → 窄场指向指令（第 12 步）；外推结果也是第 14 项"cue 延迟/镜头转向"链路的一环。
 * - 关键参数：`mission.narrow_cue_latency_s`（默认 0，即不补偿）；dt 来自仿真步长。
 * - 对应验收项：第 13 项 窄视场跟踪探测功能测试（引导精度）；第 14 项 宽窄视场联合探测功能测试
 *   （cue 延迟补偿）；第 76 项 红外宽场候选与窄场调度交互接口测试（指令角来源）。
 */

#include "sbirs_sensor/pipeline/SbirsCuePredictor.h"

#include <algorithm>
#include <cmath>

namespace sbirs_sensor {
namespace pipeline {
namespace {

// 过程：方位归一化到 (−180°, 180°] —— 外推跨 0°/360° 时不能出现 358° 这种跳变。
float NormalizeAzimuth(float azimuth_deg) {
  float result = std::fmod(azimuth_deg + 180.0f, 360.0f);
  if (result < 0.0f) {
    result += 360.0f;
  }
  return result - 180.0f;
}

// 过程：俯仰钳到 [−90°, 90°]（外推可能越过极点，越界值没有物理意义）。
float ClampElevation(float elevation_deg) {
  return std::max(-90.0f, std::min(90.0f, elevation_deg));
}

}  // namespace

// 过程：默认命令角就是本拍量测 —— 首拍/时序非法/无历史时的退化路径，不使用运动预测。
SbirsCuePrediction SbirsCuePredictor::Update(std::uint64_t target_id, float measured_azimuth_deg,
                                             float measured_elevation_deg, float dt_sec,
                                             float cue_latency_s) {
  SbirsCuePrediction result;
  result.command_azimuth_deg = NormalizeAzimuth(measured_azimuth_deg);
  result.command_elevation_deg = ClampElevation(measured_elevation_deg);

  const auto previous = targets_.find(target_id);
  const bool valid_timing = std::isfinite(dt_sec) && dt_sec > 0.0f &&
                            std::isfinite(cue_latency_s) && cue_latency_s > 0.0f;
  const bool valid_measurement =
      std::isfinite(measured_azimuth_deg) && std::isfinite(measured_elevation_deg);
  if (previous != targets_.end() && valid_timing && valid_measurement) {
    // 过程：角速度由相邻两拍量测差分求（方位取最短角差，避免跨零把 ω 放大成 360°/dt）。
    result.angular_rate_azimuth_deg_per_sec =
        NormalizeAzimuth(measured_azimuth_deg - previous->second.measured_azimuth_deg) / dt_sec;
    result.angular_rate_elevation_deg_per_sec =
        (measured_elevation_deg - previous->second.measured_elevation_deg) / dt_sec;
    // 过程：按 cue 延迟外推 θ_cmd = θ_meas + ω·τ —— 补偿量测已过时 τ 秒造成的指向落后。
    result.command_azimuth_deg = NormalizeAzimuth(
        measured_azimuth_deg + result.angular_rate_azimuth_deg_per_sec * cue_latency_s);
    result.command_elevation_deg = ClampElevation(
        measured_elevation_deg + result.angular_rate_elevation_deg_per_sec * cue_latency_s);
    result.used_motion_prediction = true;
  }

  // 过程：无论是否预测都缓存本拍有效量测供下一拍差分；非法量测不写缓存（避免污染角速度）。
  if (valid_measurement) {
    SbirsCuePredictorTargetState state;
    state.measured_azimuth_deg = NormalizeAzimuth(measured_azimuth_deg);
    state.measured_elevation_deg = ClampElevation(measured_elevation_deg);
    targets_[target_id] = state;
  }
  return result;
}

// 过程：目标释放即删缓存 —— 下次重捕重新起算，不会用陈旧角速度外推。
void SbirsCuePredictor::Release(std::uint64_t target_id) { targets_.erase(target_id); }

void SbirsCuePredictor::Clear() { targets_.clear(); }

// 过程：快照只需最近一次有效量测（角速度由实时差分得出，不单独持久化）。
SbirsCuePredictorSnapshot SbirsCuePredictor::Capture() const {
  SbirsCuePredictorSnapshot snapshot;
  snapshot.targets = targets_;
  return snapshot;
}

void SbirsCuePredictor::Restore(const SbirsCuePredictorSnapshot& snapshot) {
  targets_ = snapshot.targets;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
