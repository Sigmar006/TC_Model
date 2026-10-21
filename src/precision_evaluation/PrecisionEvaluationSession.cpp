#include "1q/precision_evaluation/PrecisionEvaluationSession.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "1q/coordinate/inertial_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/fusion/FusedTarget.h"
#include "1q/precision_evaluation/AhpEvaluator.h"
#include "1q/precision_evaluation/DualLosFix.h"
#include "1q/precision_evaluation/PrecisionEvaluationMetrics.h"
#include "1q/precision_evaluation/SbirsBearingAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/target_inference/InferenceResult.h"
#include "1q/target_inference/InferenceTrackState.h"
#include "1q/target_inference/TargetInferenceEngine.h"
#include "precision_evaluation/PrecisionEvaluationLog.h"
#include "precision_evaluation/PrecisionAcceptanceRecords.h"
#include "common/numerics/Constants.h"

namespace precision_evaluation {
namespace {

using oneq::common::numerics::kPi;
using oneq::common::numerics::RadToDeg;

// 方位角最短角差（deg）：结果落在 (-180, 180]。
float WrapAzimuthDeltaDeg(double a_deg, double b_deg) {
  double delta = std::fmod(a_deg - b_deg + 540.0, 360.0) - 180.0;
  if (delta <= -180.0) {
    delta += 360.0;
  }
  return static_cast<float>(delta);
}

// ECEF 向量模长。
double NormM(double x, double y, double z) { return std::sqrt(x * x + y * y + z * z); }

// ECI 极坐标方位 [0,2π)。
double PositiveAzimuthRad(double azimuth_rad) {
  double value = std::fmod(azimuth_rad, 2.0 * kPi);
  if (value < 0.0) {
    value += 2.0 * kPi;
  }
  return value;
}

double DistanceM(const oneq::coordinate::EcefPositionM& lhs,
                 const oneq::coordinate::EcefPositionM& rhs) {
  return NormM(lhs.x_m - rhs.x_m, lhs.y_m - rhs.y_m, lhs.z_m - rhs.z_m);
}

// 由卫星与目标的 ECEF 位置（经 GMST 旋入 ECI）计算真值 LOS 的 ECI 极坐标角，
// 与 SBIRS 输出角同参考系可直接相减。
bool TryComputeTruthAzElRad(const oneq::coordinate::EcefPositionM& satellite_ecef,
                            const oneq::coordinate::EcefPositionM& target_ecef, double gmst_rad,
                            double* azimuth_rad, double* elevation_rad) {
  if (azimuth_rad == nullptr || elevation_rad == nullptr) {
    return false;
  }
  oneq::coordinate::EciPositionM satellite_eci;
  oneq::coordinate::EciPositionM target_eci;
  if (!oneq::coordinate::TryEcefToEci(satellite_ecef, gmst_rad, &satellite_eci) ||
      !oneq::coordinate::TryEcefToEci(target_ecef, gmst_rad, &target_eci)) {
    return false;
  }
  const double los_x = target_eci.x_m - satellite_eci.x_m;
  const double los_y = target_eci.y_m - satellite_eci.y_m;
  const double los_z = target_eci.z_m - satellite_eci.z_m;
  const double range = NormM(los_x, los_y, los_z);
  if (range <= 0.0) {
    return false;
  }
  *azimuth_rad = PositiveAzimuthRad(std::atan2(los_y, los_x));
  *elevation_rad = std::asin(std::max(-1.0, std::min(1.0, los_z / range)));
  return true;
}

// ECI 单位向量旋回 ECEF 方向（旋转线性，单位向量旋转后模长不变）。
bool TryRotateEciDirectionToEcef(const oneq::coordinate::Vector3d& direction_eci, double gmst_rad,
                                 oneq::coordinate::Vector3d* direction_ecef) {
  if (direction_ecef == nullptr) {
    return false;
  }
  const oneq::coordinate::EciPositionM as_position(direction_eci.x, direction_eci.y,
                                                    direction_eci.z);
  oneq::coordinate::EcefPositionM rotated;
  if (!oneq::coordinate::TryEciToEcef(as_position, gmst_rad, &rotated)) {
    return false;
  }
  *direction_ecef = oneq::coordinate::Vector3d(rotated.x_m, rotated.y_m, rotated.z_m);
  return true;
}

// LLA → ECEF（推演关键点与融合位置估计的统一换算）。
bool TryLlaToEcefPosition(const oneq::coordinate::LlaPositionDegM& lla,
                          oneq::coordinate::EcefPositionM* ecef) {
  return oneq::coordinate::TryLlaToEcef(lla, ecef);
}

/** @brief 真值关键点缓存条目（每目标一次，经同一推演引擎解算）。 */
struct TruthKeyPoints {
  bool computed{false};
  bool has_impact{false};
  oneq::coordinate::EcefPositionM impact_ecef{};
  bool has_launch{false};
  oneq::coordinate::EcefPositionM launch_ecef{};
};

/** @brief 双星检出锚点：最近一次检出的测角与其测量周期锚点（卫星位置 + GMST）。
 *  窗口配对时两视线各锚定各自测量周期，对应"按时刻融合"的时间对齐口径。 */
struct DualSatDetectionAnchor {
  bool valid{false};
  std::uint32_t cycle_index{0U};
  float azimuth_rad{0.0f};
  float elevation_rad{0.0f};
  oneq::coordinate::EcefPositionM satellite_position_ecef_m{};
  double gmst_rad{0.0};
};

}  // namespace

struct PrecisionEvaluationSession::Impl {
  // 推演间隔 ≥1。双星会话与融合引擎由地面站组件持有，本会话只对照产品。
  static PrecisionEvaluationConfig MakeEffectiveConfig(PrecisionEvaluationConfig config) {
    if (config.inference_interval_cycles == 0U) {
      config.inference_interval_cycles = 1U;
    }
    return config;
  }

  explicit Impl(const PrecisionEvaluationConfig& evaluation_config)
      : config(MakeEffectiveConfig(evaluation_config)), inference_engine(config.inference) {}

  PrecisionEvaluationConfig config;
  target_inference::TargetInferenceEngine inference_engine;

  std::vector<double> angular_series;   // hypot(az_err, el_err)，deg
  std::vector<double> dual_sat_series;  // 交会位置误差，m
  std::vector<double> velocity_series;  // 速度矢量误差模长，m/s
  std::vector<double> impact_series;    // 落点误差，m
  std::vector<double> launch_series;    // 发射点误差，m
  // 验收判定标准 第26项：逐目标最新一拍误差快照（误差本身，非统计）。
  std::map<std::uint64_t, TargetKeyErrorSnapshot> latest_errors;
  std::map<std::uint64_t, TruthKeyPoints> truth_keypoints;
  // 双星检出锚点缓存（窗口配对用）：每星逐目标最近一次检出及其测量周期锚点。
  std::map<std::uint64_t, DualSatDetectionAnchor> last_anchor_a;
  std::map<std::uint64_t, DualSatDetectionAnchor> last_anchor_b;
  std::uint32_t cycles_since_inference{0U};
};

PrecisionEvaluationSession::PrecisionEvaluationSession(const PrecisionEvaluationConfig& config)
    : impl_(new Impl(config)) {}

PrecisionEvaluationSession::~PrecisionEvaluationSession() = default;

PrecisionEvaluationSession::PrecisionEvaluationSession(PrecisionEvaluationSession&& other) noexcept : impl_(std::move(other.impl_)) {}
PrecisionEvaluationSession& PrecisionEvaluationSession::operator=(PrecisionEvaluationSession&& other) noexcept { impl_ = std::move(other.impl_); return *this; }

PrecisionEvaluationCycleResult PrecisionEvaluationSession::Step(
    std::uint32_t cycle_index, float dt_sec, double utc_julian_day,
    const DualSatEphemerisInput& ephemeris,
    const std::vector<EvaluationTruthTarget>& truth_targets,
    const sbirs_sensor::session::SbirsCycleResult& result_a,
    const sbirs_sensor::session::SbirsCycleResult& result_b,
    const std::vector<fusion::FusedTarget>& tracks) {
  (void)dt_sec;
  PrecisionEvaluationCycleResult cycle_result;
  cycle_result.cycle_index = cycle_index;
  double gmst_rad = 0.0;
  if (!oneq::coordinate::TryComputeGmstRad(utc_julian_day, &gmst_rad)) {
    // GMST 不可解即显式返回空样本（同一坏时刻下传感器链也会判 kRejected），
    // 不以 GMST=0 静默产出方向错误的样本。
    return cycle_result;
  }

  // 真值索引。
  std::map<std::uint64_t, const EvaluationTruthTarget*> truth_by_key;
  for (const EvaluationTruthTarget& truth : truth_targets) {
    truth_by_key[truth.key] = &truth;
  }

  // 归属索引：detection_id → target_id（每星独立建立）。
  const auto build_attribution_index =
      [](const sbirs_sensor::session::SbirsCycleResult& result) {
        std::map<std::uint64_t, std::uint64_t> index;
        for (const sbirs_sensor::attribution::SbirsDetectionAttributionRecord& attribution :
             result.detection_attributions) {
          index[attribution.detection_id] = attribution.target_id;
        }
        return index;
      };
  const std::map<std::uint64_t, std::uint64_t> attribution_a = build_attribution_index(result_a);
  const std::map<std::uint64_t, std::uint64_t> attribution_b = build_attribution_index(result_b);

  // 每星按目标聚合检出记录（角度误差 + 双星交会共用）。
  // 2026-08-31 用户裁定：双星定位仅基于窄视场数据（宽场粗测角只承担引导，不进
  // 交会）——宽场搜索阶段记录（kWideFieldSearch）在聚合处即剔除；窄场捕获/跟踪
  // 两个阶段均属窄视场量测，保留。
  const auto collect_detections_by_target =
      [](const sbirs_sensor::session::SbirsCycleResult& result,
         const std::map<std::uint64_t, std::uint64_t>& attribution_index) {
        std::map<std::uint64_t, const sbirs_sensor::output::SbirsDetectionRecord*> by_target;
        if (result.status != sbirs_sensor::session::SbirsCycleStatus::kCompleted) {
          return by_target;
        }
        for (const sbirs_sensor::output::SbirsDetectionRecord& detection :
             result.output_frame.detections) {
          if (!detection.detected) {
            continue;
          }
          if (detection.observation_stage ==
              sbirs_sensor::output::SbirsObservationStage::kWideFieldSearch) {
            continue;
          }
          const auto entry = attribution_index.find(detection.detection_id);
          if (entry == attribution_index.end()) {
            continue;
          }
          by_target[entry->second] = &detection;
        }
        return by_target;
      };
  const std::map<std::uint64_t, const sbirs_sensor::output::SbirsDetectionRecord*> detected_a =
      collect_detections_by_target(result_a, attribution_a);
  const std::map<std::uint64_t, const sbirs_sensor::output::SbirsDetectionRecord*> detected_b =
      collect_detections_by_target(result_b, attribution_b);

  // 第26项 脱靶量：NFOV 焦平面脱靶量经归属记录透出（每星独立，后写覆盖先写——
  // 双星同周期均有跟踪段时取后处理星的一拍）。
  const auto collect_focal_by_target =
      [](const sbirs_sensor::session::SbirsCycleResult& result,
         std::map<std::uint64_t, std::pair<float, float>>* out) {
        if (result.status != sbirs_sensor::session::SbirsCycleStatus::kCompleted) {
          return;
        }
        for (const sbirs_sensor::attribution::SbirsDetectionAttributionRecord& attribution :
             result.detection_attributions) {
          if (!attribution.has_focal_plane_offset) {
            continue;
          }
          (*out)[attribution.target_id] = std::make_pair(attribution.focal_plane_offset_x_m,
                                                         attribution.focal_plane_offset_y_m);
        }
      };
  std::map<std::uint64_t, std::pair<float, float>> focal_by_target;
  collect_focal_by_target(result_a, &focal_by_target);
  collect_focal_by_target(result_b, &focal_by_target);
  for (const auto& focal_entry : focal_by_target) {
    TargetKeyErrorSnapshot& snapshot = impl_->latest_errors[focal_entry.first];
    snapshot.last_cycle = cycle_index;
    snapshot.has_focal = true;
    snapshot.focal_x_m = focal_entry.second.first;
    snapshot.focal_y_m = focal_entry.second.second;
  }

  // ② 红外定位角度误差（各星输出角 vs 真值角）。
  const auto accumulate_angular = [&](const std::map<std::uint64_t,
                                                  const sbirs_sensor::output::SbirsDetectionRecord*>&
                                          detected,
                                      const oneq::coordinate::EcefPositionM& satellite_ecef,
                                      int satellite_index) {
    for (const auto& entry : detected) {
      const auto truth_entry = truth_by_key.find(entry.first);
      if (truth_entry == truth_by_key.end()) {
        continue;
      }
      double truth_az_rad = 0.0;
      double truth_el_rad = 0.0;
      if (!TryComputeTruthAzElRad(satellite_ecef, truth_entry->second->position_ecef_m, gmst_rad,
                                  &truth_az_rad, &truth_el_rad)) {
        continue;
      }
      AngularErrorSample sample;
      sample.key = entry.first;
      sample.cycle_index = cycle_index;
      sample.satellite_index = satellite_index;
      sample.azimuth_error_deg =
          WrapAzimuthDeltaDeg(RadToDeg(static_cast<double>(entry.second->azimuth_rad)),
                              RadToDeg(truth_az_rad));
      sample.elevation_error_deg =
          static_cast<float>(RadToDeg(static_cast<double>(entry.second->elevation_rad)) -
                             RadToDeg(truth_el_rad));
      cycle_result.angular.push_back(sample);
      impl_->angular_series.push_back(
          std::hypot(static_cast<double>(sample.azimuth_error_deg),
                     static_cast<double>(sample.elevation_error_deg)));
      // 第26项：最新一拍测角误差（写 Summarize 的逐目标行，不再逐样本落盘）。
      TargetKeyErrorSnapshot& snapshot = impl_->latest_errors[entry.first];
      snapshot.last_cycle = cycle_index;
      snapshot.has_angular = true;
      snapshot.az_error_deg = sample.azimuth_error_deg;
      snapshot.el_error_deg = sample.elevation_error_deg;
    }
  };
  accumulate_angular(detected_a, ephemeris.satellite_a_position_ecef_m, 0);
  accumulate_angular(detected_b, ephemeris.satellite_b_position_ecef_m, 1);

  // ③ 双星双视线交会（同目标双检出；配对窗 = dual_sat_pair_window_cycles）。
  // 锚点缓存：每星逐目标最近检出连同其测量周期锚点（卫星位置 + GMST）。窗内
  // 配对时两视线各锚定各自测量周期——对应现实"地面按时刻融合"的时间对齐环节
  // （时间基准为周期号抽帧时刻）；窗宽 0 退化为严格同周期（历史行为）。
  const auto refresh_anchors =
      [&](const std::map<std::uint64_t,
                         const sbirs_sensor::output::SbirsDetectionRecord*>& detected,
          const oneq::coordinate::EcefPositionM& satellite_position,
          std::map<std::uint64_t, DualSatDetectionAnchor>* anchors) {
        for (const auto& detected_entry : detected) {
          DualSatDetectionAnchor& anchor = (*anchors)[detected_entry.first];
          anchor.valid = true;
          anchor.cycle_index = cycle_index;
          anchor.azimuth_rad = detected_entry.second->azimuth_rad;
          anchor.elevation_rad = detected_entry.second->elevation_rad;
          anchor.satellite_position_ecef_m = satellite_position;
          anchor.gmst_rad = gmst_rad;
        }
      };
  refresh_anchors(detected_a, ephemeris.satellite_a_position_ecef_m, &impl_->last_anchor_a);
  refresh_anchors(detected_b, ephemeris.satellite_b_position_ecef_m, &impl_->last_anchor_b);
  for (const auto& anchor_entry : impl_->last_anchor_a) {
    const DualSatDetectionAnchor& anchor_a = anchor_entry.second;
    if (!anchor_a.valid) {
      continue;
    }
    const auto anchor_b_entry = impl_->last_anchor_b.find(anchor_entry.first);
    if (anchor_b_entry == impl_->last_anchor_b.end() || !anchor_b_entry->second.valid) {
      continue;
    }
    const DualSatDetectionAnchor& anchor_b = anchor_b_entry->second;
    // 每目标每周期至多一条样本：仅当至少一侧为本周期新检出时输出（避免旧锚点
    // 重复配对；窗宽 0 时该条件蕴含两侧均为本周期，与历史行为一致）。
    if (anchor_a.cycle_index != cycle_index && anchor_b.cycle_index != cycle_index) {
      continue;
    }
    const std::uint32_t newer_cycle = std::max(anchor_a.cycle_index, anchor_b.cycle_index);
    const std::uint32_t older_cycle = std::min(anchor_a.cycle_index, anchor_b.cycle_index);
    if (newer_cycle - older_cycle > impl_->config.dual_sat_pair_window_cycles) {
      continue;
    }
    const auto truth_entry = truth_by_key.find(anchor_entry.first);
    if (truth_entry == truth_by_key.end()) {
      continue;
    }
    oneq::coordinate::Vector3d direction_a_ecef;
    oneq::coordinate::Vector3d direction_b_ecef;
    if (!TryRotateEciDirectionToEcef(
            EciDirectionFromAzimuthElevationRad(static_cast<double>(anchor_a.azimuth_rad),
                                                static_cast<double>(anchor_a.elevation_rad)),
            anchor_a.gmst_rad, &direction_a_ecef) ||
        !TryRotateEciDirectionToEcef(
            EciDirectionFromAzimuthElevationRad(static_cast<double>(anchor_b.azimuth_rad),
                                                static_cast<double>(anchor_b.elevation_rad)),
            anchor_b.gmst_rad, &direction_b_ecef)) {
      continue;
    }
    oneq::coordinate::EcefPositionM fix_position;
    double residual_m = 0.0;
    if (!TryComputeDualLosFixM(anchor_a.satellite_position_ecef_m, direction_a_ecef,
                               anchor_b.satellite_position_ecef_m, direction_b_ecef,
                               &fix_position, &residual_m)) {
      continue;
    }
    DualSatFixSample sample;
    sample.key = anchor_entry.first;
    sample.cycle_index = cycle_index;
    sample.position_error_m = DistanceM(fix_position, truth_entry->second->position_ecef_m);
    sample.los_residual_m = residual_m;
    // 评审 2026-08-26 条13：与目标的距离误差（斜距）——交会解到主星距离 vs 真值到
    // 主星距离之差（SBIRS 无源测角无真实测距，此为交会解可得的真实口径）。
    sample.slant_range_error_m =
        DistanceM(anchor_a.satellite_position_ecef_m, fix_position) -
        DistanceM(anchor_a.satellite_position_ecef_m, truth_entry->second->position_ecef_m);
    cycle_result.dual_sat.push_back(sample);
    impl_->dual_sat_series.push_back(sample.position_error_m);
    // 第26项：最新一拍交会位置误差 ECEF 向量与距离误差（Summarize 逐目标行）。
    TargetKeyErrorSnapshot& snapshot = impl_->latest_errors[anchor_entry.first];
    snapshot.last_cycle = cycle_index;
    snapshot.has_ecef = true;
    snapshot.ecef_error_m[0] = fix_position.x_m - truth_entry->second->position_ecef_m.x_m;
    snapshot.ecef_error_m[1] = fix_position.y_m - truth_entry->second->position_ecef_m.y_m;
    snapshot.ecef_error_m[2] = fix_position.z_m - truth_entry->second->position_ecef_m.z_m;
    snapshot.has_slant_range = true;
    snapshot.slant_range_error_m = sample.slant_range_error_m;
  }

  // ④ 融合航迹（调用方已 Update）→ 速度误差（附位置误差）。
  std::map<std::uint64_t, const fusion::FusedTarget*> track_by_key;
  for (const fusion::FusedTarget& track : tracks) {
    track_by_key[track.key] = &track;
  }
  for (const auto& truth_entry : truth_by_key) {
    const EvaluationTruthTarget* truth = truth_entry.second;
    const auto track_entry = track_by_key.find(truth->key);
    if (track_entry == track_by_key.end() || !track_entry->second->has_kinematic_estimate ||
        !truth->has_velocity) {
      continue;
    }
    oneq::coordinate::EcefPositionM estimated_position;
    if (!TryLlaToEcefPosition(track_entry->second->kinematic_estimate.position, &estimated_position)) {
      continue;
    }
    const std::array<double, 3U>& velocity = track_entry->second->kinematic_estimate.velocity_ecef_m_per_s;
    const double velocity_error = NormM(velocity[0] - truth->velocity_ecef_m_per_s.x_mps,
                                        velocity[1] - truth->velocity_ecef_m_per_s.y_mps,
                                        velocity[2] - truth->velocity_ecef_m_per_s.z_mps);
    VelocityErrorSample sample;
    sample.key = truth->key;
    sample.cycle_index = cycle_index;
    sample.velocity_error_m_per_s = velocity_error;
    sample.position_error_m = DistanceM(estimated_position, truth->position_ecef_m);
    cycle_result.velocity.push_back(sample);
    impl_->velocity_series.push_back(sample.velocity_error_m_per_s);
  }

  // ⑤ 按间隔以估计状态与真值状态分别推演 → 落点/发射点预测误差。
  ++impl_->cycles_since_inference;
  if (impl_->cycles_since_inference >= impl_->config.inference_interval_cycles) {
    impl_->cycles_since_inference = 0U;
    // 真值关键点缓存（每目标一次；经同一推演引擎解算，衡量状态误差传播）。
    for (const auto& truth_entry : truth_by_key) {
      TruthKeyPoints& cached = impl_->truth_keypoints[truth_entry.first];
      if (cached.computed || !truth_entry.second->has_velocity) {
        continue;
      }
      target_inference::InferenceTrackState truth_state;
      truth_state.key = truth_entry.first;
      truth_state.sim_time_sec =
          static_cast<double>(cycle_index) * static_cast<double>(dt_sec);
      truth_state.input_cycle_index = cycle_index;
      truth_state.position = truth_entry.second->position_ecef_m;
      truth_state.velocity_ecef_m_per_s = {{truth_entry.second->velocity_ecef_m_per_s.x_mps,
                                            truth_entry.second->velocity_ecef_m_per_s.y_mps,
                                            truth_entry.second->velocity_ecef_m_per_s.z_mps}};
      const std::vector<target_inference::TargetInferenceResult> truth_results =
          impl_->inference_engine.Infer({truth_state}, false);
      if (!truth_results.empty()) {
        const target_inference::TrajectoryPrediction& trajectory = truth_results.front().trajectory;
        // 关键点 LLA→ECEF 失败视为无关键点（与其余分支的守卫口径一致）。
        cached.has_impact =
            trajectory.has_impact &&
            TryLlaToEcefPosition(trajectory.impact_point, &cached.impact_ecef);
        cached.has_launch =
            trajectory.has_launch &&
            TryLlaToEcefPosition(trajectory.launch_point, &cached.launch_ecef);
      }
      cached.computed = true;
    }
    // 估计状态推演（有运动学估计的航迹）。
    std::vector<target_inference::InferenceTrackState> estimate_states;
    for (const auto& track_entry : track_by_key) {
      if (!track_entry.second->has_kinematic_estimate) {
        continue;
      }
      target_inference::InferenceTrackState state;
      state.key = track_entry.first;
      // 行前缀与关机状态机的时间基准：不填则验收行恒 周期=0/时间=0，且关机
      // 状态机 dt=0（加速度通道失效）——评审验证时发现的缺陷，随条9/10 一并修。
      state.sim_time_sec = static_cast<double>(cycle_index) * static_cast<double>(dt_sec);
      state.input_cycle_index = cycle_index;
      oneq::coordinate::EcefPositionM estimated_position;
      if (!TryLlaToEcefPosition(track_entry.second->kinematic_estimate.position, &estimated_position)) {
        continue;
      }
      state.position = estimated_position;
      state.velocity_ecef_m_per_s = track_entry.second->kinematic_estimate.velocity_ecef_m_per_s;
      state.has_covariance = true;
      state.covariance_ecef = track_entry.second->kinematic_estimate.covariance_ecef;
      estimate_states.push_back(state);
    }
    if (!estimate_states.empty()) {
      const std::vector<target_inference::TargetInferenceResult> estimate_results =
          impl_->inference_engine.Infer(estimate_states);
      for (const target_inference::TargetInferenceResult& estimate_result : estimate_results) {
        const auto truth_cache = impl_->truth_keypoints.find(estimate_result.key);
        if (truth_cache == impl_->truth_keypoints.end() || !truth_cache->second.computed) {
          continue;
        }
        KeyPointErrorSample sample;
        sample.key = estimate_result.key;
        sample.cycle_index = cycle_index;
        const target_inference::TrajectoryPrediction& trajectory = estimate_result.trajectory;
        if (trajectory.has_impact && truth_cache->second.has_impact) {
          oneq::coordinate::EcefPositionM estimated_impact;
          if (TryLlaToEcefPosition(trajectory.impact_point, &estimated_impact)) {
            sample.has_impact = true;
            sample.impact_error_m = DistanceM(estimated_impact, truth_cache->second.impact_ecef);
          }
        }
        if (trajectory.has_launch && truth_cache->second.has_launch) {
          oneq::coordinate::EcefPositionM estimated_launch;
          if (TryLlaToEcefPosition(trajectory.launch_point, &estimated_launch)) {
            sample.has_launch = true;
            sample.launch_error_m = DistanceM(estimated_launch, truth_cache->second.launch_ecef);
          }
        }
        if (sample.has_impact || sample.has_launch) {
          cycle_result.keypoints.push_back(sample);
          if (sample.has_impact) {
            impl_->impact_series.push_back(sample.impact_error_m);
          }
          if (sample.has_launch) {
            impl_->launch_series.push_back(sample.launch_error_m);
          }
        }
        // 评审 2026-08-26 条12：落点预报外发样本（有落点解的估计航迹，装配层据此
        // 发布事件；真值对照推演已抑制验收写出，不产生外发样本）。
        if (trajectory.has_impact) {
          ImpactForecastSample forecast;
          forecast.key = estimate_result.key;
          forecast.cycle_index = cycle_index;
          forecast.impact_point = trajectory.impact_point;
          forecast.impact_position_sigma_m = trajectory.impact_position_sigma_m;
          forecast.has_burnout_sigma = trajectory.has_burnout_sigma;
          forecast.burnout_position_sigma_m = trajectory.burnout_position_sigma_m;
          cycle_result.forecasts.push_back(forecast);
        }
      }
    }
  }
  return cycle_result;
}

std::size_t PrecisionEvaluationSession::DebugLatestErrorCount() const {
  return impl_->latest_errors.size();
}

PrecisionEvaluationReport PrecisionEvaluationSession::Summarize() const {  PrecisionEvaluationReport report;
  const std::vector<double>* series[kPrecisionMetricCount] = {
      &impl_->angular_series, &impl_->dual_sat_series, &impl_->velocity_series,
      &impl_->impact_series, &impl_->launch_series};
  double rmse[kPrecisionMetricCount] = {0.0, 0.0, 0.0, 0.0, 0.0};
  bool all_sampled = true;
  for (std::size_t index = 0U; index < kPrecisionMetricCount; ++index) {
    report.metrics[index] = SummarizeErrorSeries(*series[index]);
    rmse[index] = report.metrics[index].rmse;
    all_sampled = all_sampled && report.metrics[index].count > 0U;
    // 零样本指标按"无证据=0 分"进综合：rmse 置 +∞ → score 0（贡献 0）。避免
    // "全程无检出也得满分"的读数污染（count=0 → rmse=0 → score 1 的天然缺陷）。
    if (report.metrics[index].count == 0U) {
      rmse[index] = std::numeric_limits<double>::infinity();
    }
  }
  report.all_metrics_sampled = all_sampled;

  const double references[kPrecisionMetricCount] = {
      impl_->config.reference_error_angular_deg, impl_->config.reference_error_dual_sat_fix_m,
      impl_->config.reference_error_velocity_m_per_s, impl_->config.reference_error_impact_m,
      impl_->config.reference_error_launch_m};
  AhpEvaluation ahp;
  if (!TryEvaluateAhp(impl_->config.ahp, &ahp)) {
    report.ahp_valid = false;
    if (PRECISION_EVAL_LOG_ENABLED()) {
      WritePrecisionAhp(report);
    }
    return report;
  }
  report.ahp = ahp;
  report.ahp_valid = true;
  ComposePrecisionScore(ahp.weights, rmse, references, &report);
  if (PRECISION_EVAL_LOG_ENABLED()) {
    WritePrecisionKeyMetrics(impl_->latest_errors);
    WritePrecisionAhp(report);
  }
  return report;
}

}  // namespace precision_evaluation
