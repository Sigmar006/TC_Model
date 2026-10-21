/**
 * @file InferenceAcceptanceRecords.cpp
 * @brief 推演验收行：轨迹预报 / 落点 / 发射点 / 发布；关机点按助推-滑行状态机判定。
 */

#include "target_inference/InferenceAcceptanceRecords.h"

#include <cmath>
#include <map>
#include <string>

#include "1q/coordinate/position_transform.h"
#include "1q/target_inference/TargetInferenceEngine.h"
#include "common/logging/AcceptanceText.h"
#include "target_inference/InferenceAcceptanceLog.h"

namespace target_inference {
namespace {

using oneq::logging::FormatF;
using oneq::logging::FormatVec3;

struct LastVel {
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  bool valid{false};
};

std::map<std::uint64_t, LastVel>& LastVelocities() {
  static std::map<std::uint64_t, LastVel> values;
  return values;
}

/**
 * @brief 单航迹关机点判定状态表（会话内累计，仿 LastVelocities 先例）。
 * @note 甲方 2026-08-22 批注「判断其势能+动能最大的地方」：助推-滑行状态机，
 *       双通道助推判别（加速度 > 2.5·μ/r² 或 ε 上涨超 0.1% 势能尺度），
 *       连续 2 拍无助推确认关机；从未助推且能量平稳 3 拍判窗口外。
 *       方法口径与阈值见验收目录条目与 algorithms.md。
 */
std::map<std::uint64_t, BurnoutTrackerState>& BurnoutTrackers() {
  static std::map<std::uint64_t, BurnoutTrackerState> values;
  return values;
}

void HorizontalEllipseFromCov(const std::array<double, 36U>& cov, double* semi_major,
                              double* semi_minor, double* azimuth_deg) {
  // 位置方差在 [x,vx,y,vy,z,vz] 的 (0,0)/(2,2)/(4,4)；水平取 ECEF x-y 近似。
  const double a = cov[0];
  const double b = cov[2];
  const double c = cov[14];
  const double disc = std::sqrt(std::max(0.0, (a - c) * (a - c) + 4.0 * b * b));
  const double l1 = 0.5 * (a + c + disc);
  const double l2 = 0.5 * (a + c - disc);
  *semi_major = std::sqrt(std::max(0.0, l1));
  *semi_minor = std::sqrt(std::max(0.0, l2));
  *azimuth_deg = 0.5 * std::atan2(2.0 * b, a - c) * 57.29577951308232;
}

}  // namespace

double SpecificMechanicalEnergyJPerKg(const oneq::coordinate::EcefPositionM& position,
                                      const std::array<double, 3U>& velocity_ecef_m_per_s,
                                      double earth_mu_m3_per_s2) {
  const double radius = std::sqrt(position.x_m * position.x_m + position.y_m * position.y_m +
                                  position.z_m * position.z_m);
  if (radius <= 0.0) {
    return 0.0;
  }
  const double speed_sq = velocity_ecef_m_per_s[0] * velocity_ecef_m_per_s[0] +
                          velocity_ecef_m_per_s[1] * velocity_ecef_m_per_s[1] +
                          velocity_ecef_m_per_s[2] * velocity_ecef_m_per_s[2];
  return 0.5 * speed_sq - earth_mu_m3_per_s2 / radius;
}

BurnoutPhase UpdateBurnoutTracker(BurnoutTrackerState& state,
                                  const oneq::coordinate::EcefPositionM& position,
                                  const std::array<double, 3U>& velocity_ecef_m_per_s,
                                  double t_sec, double earth_mu_m3_per_s2,
                                  double velocity_sigma_m) {
  // 方法常数（尺度均由 μ/r 推导，不新造物理常数；依据见 algorithms.md「关机点判定」）：
  // 推力典型几倍重力、滤波速度噪声仅几 m/s，2.5 倍重力门两不相扰；逐拍能量门取
  // 噪声尺度 1e-4·μ/r（高于 v·δv 噪声、低于缓升段单拍增量，防把慢推力误判平稳），
  // 累计下降门沿用旧口径 1e-3·μ/r；防抖/确认/窗口拍数为噪声与果断性折中。
  // 速度 σ 护栏（评审 2026-08-27 条3）：5 m/s——加速度门 2.5·μ/r² 在 1s 步长下
  // 折合 Δv≈20 m/s，速度噪声需低于其 1/4 才不至于把噪声当助推。
  constexpr double kBoostAccelGravityScale = 2.5;
  constexpr double kEnergyStepScale = 1.0e-4;
  constexpr double kEnergyDeclineScale = 1.0e-3;
  constexpr std::uint32_t kRiseDebounceSamples = 2U;
  constexpr std::uint32_t kCoastConfirmSamples = 2U;
  constexpr std::uint32_t kFlatWindowSamples = 5U;
  constexpr double kBurnoutVelocitySigmaM = 5.0;

  const double radius = std::sqrt(position.x_m * position.x_m + position.y_m * position.y_m +
                                  position.z_m * position.z_m);
  if (radius <= 0.0) {
    return BurnoutPhase::kObserving;  // 不可用采样不推进状态（调用方以半径守卫另行写行）。
  }
  if (velocity_sigma_m >= kBurnoutVelocitySigmaM) {
    // 弱可观测（如仅方位融合）航迹：不推进助推/峰值/相态状态（状态原样保留，
    // 精度恢复后从头累积），相态如实停观测中。
    return BurnoutPhase::kObserving;
  }
  const double speed = std::sqrt(
      velocity_ecef_m_per_s[0] * velocity_ecef_m_per_s[0] +
      velocity_ecef_m_per_s[1] * velocity_ecef_m_per_s[1] +
      velocity_ecef_m_per_s[2] * velocity_ecef_m_per_s[2]);
  const double energy =
      SpecificMechanicalEnergyJPerKg(position, velocity_ecef_m_per_s, earth_mu_m3_per_s2);
  const double step_margin = kEnergyStepScale * earth_mu_m3_per_s2 / radius;
  const double decline_margin = kEnergyDeclineScale * earth_mu_m3_per_s2 / radius;
  const double gravity_m_per_s2 = earth_mu_m3_per_s2 / (radius * radius);

  // 助推双通道（任一命中即助推采样）：加速度 |Δv|/Δt > 2.5·当地重力；低推力时
  // 推力做正功 ⇒ ε 逐拍上涨超噪声尺度，连续 ≥2 拍防单拍噪声。物理依据：
  // 无阻力滑行 dε/dt = 0 恒平稳，ε 上涨 ⟺ 推力做正功。
  bool boost_sample = false;
  if (state.valid) {
    const double dt_sec = t_sec - state.last_time_sec;
    if (dt_sec > 0.0) {
      const double dv = std::sqrt(
          (velocity_ecef_m_per_s[0] - state.last_velocity_m_per_s[0]) *
              (velocity_ecef_m_per_s[0] - state.last_velocity_m_per_s[0]) +
          (velocity_ecef_m_per_s[1] - state.last_velocity_m_per_s[1]) *
              (velocity_ecef_m_per_s[1] - state.last_velocity_m_per_s[1]) +
          (velocity_ecef_m_per_s[2] - state.last_velocity_m_per_s[2]) *
              (velocity_ecef_m_per_s[2] - state.last_velocity_m_per_s[2]));
      boost_sample = dv / dt_sec > kBoostAccelGravityScale * gravity_m_per_s2;
    }
    const double d_energy = energy - state.last_energy_j_per_kg;
    state.rise_streak = d_energy > step_margin ? state.rise_streak + 1U : 0U;
    if (!boost_sample && state.rise_streak >= kRiseDebounceSamples) {
      boost_sample = true;
    }
  }

  // 再次助推重开（多脉冲/再次加速语义）：已确认遇助推特征则撤销结论、峰值锚点
  // 解冻续推——否则持续缓升目标（如巡航爬升飞机）的关机时刻会随峰值漂移。
  if (boost_sample && state.confirmed) {
    state.confirmed = false;
  }

  // 峰值累计（确认期间冻结，锚点稳定在确认时刻）：助推段 ε 单调升 ⇒
  // 峰值=最后一个助推采样=关机时刻。
  const bool first_sample = !state.valid;
  if (!state.confirmed && (first_sample || energy > state.energy_j_per_kg)) {
    state.energy_j_per_kg = energy;
    state.time_sec = t_sec;
    state.position = position;
    state.radius_m = radius;
    state.speed_mps = speed;
    state.peak_is_first_sample = first_sample;
  }
  if (first_sample) {
    state.first_energy_j_per_kg = energy;  // 平稳带锚点：从未助推时以首采样为基准。
  }

  // 相态推进：助推清零滑行/平稳计数；已见助推按连续滑行计数；从未助推按平稳带
  // （|ε − 首采样 ε| ≤ 噪声尺度）计数——缓升段累计破带即脱离平稳，不误判窗口外。
  if (boost_sample) {
    state.ever_boosted = true;
    state.coast_samples = 0U;
    state.flat_samples = 0U;
  } else if (state.ever_boosted) {
    ++state.coast_samples;
    state.flat_samples = 0U;
  } else {
    state.coast_samples = 0U;
    const bool within_band = std::fabs(energy - state.first_energy_j_per_kg) <= step_margin;
    state.flat_samples = within_band ? state.flat_samples + 1U : 0U;
  }
  // 关机确认双路径（粘性，遇助推重开）：下降沿——ε 自峰值累计降幅超 0.1% 势能
  // 尺度（旧口径，覆盖慢推力缓升后回落）；滑行——见助推后连续 2 拍无助推。
  if (!state.confirmed) {
    if (energy < state.energy_j_per_kg - decline_margin) {
      state.confirmed = true;
    } else if (state.ever_boosted && state.coast_samples >= kCoastConfirmSamples) {
      state.confirmed = true;
    }
  }

  state.last_velocity_m_per_s = velocity_ecef_m_per_s;
  state.last_time_sec = t_sec;
  state.last_energy_j_per_kg = energy;
  state.valid = true;

  // 相态裁决：确认（粘性）> 助推中 > 窗口外 > 观测中。
  if (state.confirmed) {
    return state.peak_is_first_sample ? BurnoutPhase::kBeforeTrackStart : BurnoutPhase::kConfirmed;
  }
  if (state.ever_boosted) {
    return BurnoutPhase::kBoosting;
  }
  if (state.flat_samples >= kFlatWindowSamples) {
    return BurnoutPhase::kBeforeWindow;
  }
  return BurnoutPhase::kObserving;
}

bool TryResolveBurnoutEstimate(BurnoutPhase phase, const BurnoutTrackerState& state,
                               double current_time_sec,
                               const TrajectoryPrediction& trajectory,
                               double* burnout_time_sec,
                               oneq::coordinate::LlaPositionDegM* burnout_position) {
  if (burnout_time_sec == nullptr || burnout_position == nullptr) {
    return false;
  }
  if (phase == BurnoutPhase::kConfirmed) {
    if (!oneq::coordinate::TryEcefToLla(state.position, burnout_position)) {
      return false;
    }
    *burnout_time_sec = state.time_sec;
    return true;
  }
  const bool can_backtrack = phase == BurnoutPhase::kBeforeWindow ||
                             phase == BurnoutPhase::kBeforeTrackStart ||
                             (phase == BurnoutPhase::kObserving && state.valid);
  if (!can_backtrack || !trajectory.valid || !trajectory.has_launch) {
    return false;
  }
  *burnout_time_sec = current_time_sec + trajectory.launch_time_offset_sec;
  *burnout_position = trajectory.launch_point;
  return true;
}

void WriteInferenceAcceptance(const std::vector<InferenceTrackState>& tracks,
                              std::vector<TargetInferenceResult>& results,
                              const TargetInferenceConfig& config) {
  if (!INFERENCE_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }

  // 关机点预测（2026-08-22 甲方批注「判断其势能+动能最大的地方」）：助推-滑行
  // 状态机逐航迹判定（双通道助推判别，见 UpdateBurnoutTracker）。
  // 评审 2026-08-26 条9：行内只保留关机点时刻与关机点坐标；精度门未通过或
  // 正在助推时写「正在模拟计算」，其余未确认状态可写带依据标识的回推估计。
  if (tracks.empty()) {
    return;
  }
  for (std::size_t i = 0U; i < tracks.size(); ++i) {
    const InferenceTrackState& track = tracks[i];
    const float row_sim_time = static_cast<float>(track.sim_time_sec);
    const std::uint32_t row_cycle = track.input_cycle_index;
    const double radius = std::sqrt(track.position.x_m * track.position.x_m +
                                    track.position.y_m * track.position.y_m +
                                    track.position.z_m * track.position.z_m);
    if (radius <= 0.0) {
      INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "关机点预测功能测试",
                                "目标键=" + std::to_string(track.key) + " 位置不可用");
      continue;
    }
    BurnoutTrackerState& state = BurnoutTrackers()[track.key];
    // 评审 2026-08-27 条3：速度观测 σ 护栏——仅方位/弱可观测航迹的速度欠估计在
    // 滤波收敛期呈持续缓升（实测 dε≈1.4e5 J/kg/拍 ≫ 门限 5.7e3），与助推特征
    // 动力学不可分；σ_v 未达雷达级跟踪精度时状态机不推进（先前表现为关机时刻
    // 恒冻结在收敛暂态的早期假峰值，如 42.0s/18.0s/8.0s），如实写正在模拟计算。
    double velocity_sigma_m = 0.0;
    if (track.has_covariance) {
      const double var_v = (track.covariance_ecef[7U] + track.covariance_ecef[21U] +
                            track.covariance_ecef[35U]) / 3.0;
      velocity_sigma_m = std::sqrt(std::max(var_v, 0.0));
    }
    const BurnoutPhase phase =
        UpdateBurnoutTracker(state, track.position, track.velocity_ecef_m_per_s,
                             track.sim_time_sec, config.earth_mu_m3_per_s2,
                             velocity_sigma_m);
    std::string burnout = "目标键=" + std::to_string(track.key);
    const TrajectoryPrediction empty_trajectory{};
    const TrajectoryPrediction& trajectory =
        i < results.size() ? results[i].trajectory : empty_trajectory;
    double burnout_time_sec = 0.0;
    oneq::coordinate::LlaPositionDegM burnout_lla{};
    if (TryResolveBurnoutEstimate(phase, state, track.sim_time_sec, trajectory,
                                  &burnout_time_sec, &burnout_lla)) {
      burnout += " 关机点时刻=" + FormatF(burnout_time_sec, 1) + "s";
      burnout += " 关机点经纬高=" + FormatVec3(
          burnout_lla.latitude_deg, burnout_lla.longitude_deg, burnout_lla.altitude_m, 3);
      if (phase != BurnoutPhase::kConfirmed) {
        burnout += " 判定依据=大气界面回推估计";
      }
      // 评审 2026-08-26 条10 + 2026-08-27 条3：关机点 1-σ = 当前协方差敏度传播
      // 到关机时刻（引擎静态敏度面），随行输出并回填 API 字段供发布行使用。
      if (i < results.size() && track.has_covariance) {
        const double burnout_offset_sec = burnout_time_sec - track.sim_time_sec;
        const double burnout_sigma =
            TargetInferenceEngine::PositionSigmaAt(track, burnout_offset_sec, config);
        if (burnout_sigma > 0.0) {
          results[i].trajectory.has_burnout_sigma = true;
          results[i].trajectory.burnout_position_sigma_m = burnout_sigma;
          burnout += " 关机点误差1σ=" + FormatF(burnout_sigma, 1) + "m";
        }
      }
    } else {
      // 速度精度门未通过、助推中或无可用回推结果：关机点仍需继续计算。
      burnout += " 关机点时刻=正在模拟计算";
      burnout += " 关机点经纬高=正在模拟计算";
    }
    INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "关机点预测功能测试", burnout);
  }

  for (std::size_t i = 0U; i < results.size(); ++i) {
    const TargetInferenceResult& result = results[i];
    const TrajectoryPrediction& traj = result.trajectory;
    // 行前缀时间：与航迹采样同源（调用方未提供时保持 0，与既有行为一致）。
    const float row_sim_time =
        i < tracks.size() ? static_cast<float>(tracks[i].sim_time_sec) : 0.0f;
    const std::uint32_t row_cycle = i < tracks.size() ? tracks[i].input_cycle_index : 0U;
    if (!traj.valid) {
      continue;
    }
    // 评审 2026-08-26 条7：预报时段固定为当前周期时刻起 10s（记录层裁剪，引擎
    // 时域不动——落点解算依赖完整时域）；航路点点数只统计 10s 内的点。
    constexpr double kForecastWindowSec = 10.0;
    std::size_t waypoints_in_window = 0U;
    for (const InferenceWaypoint& waypoint : traj.waypoints) {
      if (waypoint.time_offset_sec <= kForecastWindowSec) {
        ++waypoints_in_window;
      }
    }
    const double sigma = traj.impact_position_sigma_m;
    // 验收判定标准 第20项：目标键随行（10s 预报轨迹与预测落点同条）。
    std::string forecast = "目标键=" + std::to_string(i < tracks.size() ? tracks[i].key : 0U);
    forecast += " 航路点点数=" + std::to_string(waypoints_in_window);
    forecast += " 预报时段=[0.0," + FormatF(kForecastWindowSec, 1) + "]s";
    // 评审 2026-08-27 条2：预测列表给出 10 个 LLA 预测点（每 1s 一点，泛式代表
    // 未来 10s 轨迹）；点值由引擎 RK4 标称传播取（与航路点/落点同动力学），某点
    // 传播失败则列表截断到已得点位。
    if (i < tracks.size()) {
      std::string points;
      for (int k = 1; k <= 10; ++k) {
        oneq::coordinate::LlaPositionDegM point_lla{};
        if (!TargetInferenceEngine::PositionAt(tracks[i], static_cast<double>(k), config,
                                               &point_lla)) {
          break;
        }
        if (!points.empty()) {
          points += ";";
        }
        points += "+" + std::to_string(k) + "s:" + FormatVec3(point_lla.latitude_deg,
                                                              point_lla.longitude_deg,
                                                              point_lla.altitude_m, 3);
      }
      if (!points.empty()) {
        forecast += " 预测点LLA=[" + points + "]";
      }
    }
    if (i < tracks.size()) {
      oneq::coordinate::LlaPositionDegM current_lla{};
      if (oneq::coordinate::TryEcefToLla(tracks[i].position, &current_lla)) {
        forecast += " 当前位置经纬高=" + FormatVec3(current_lla.latitude_deg,
                                                     current_lla.longitude_deg,
                                                     current_lla.altitude_m, 3);
      }
    }
    if (traj.has_impact) {
      forecast += " 落点经纬高=" + FormatVec3(traj.impact_point.latitude_deg,
                                              traj.impact_point.longitude_deg,
                                              traj.impact_point.altitude_m, 3);
      forecast += " 误差椭圆半长/半短/方位=(" + FormatF(sigma, 1) + "m," + FormatF(sigma, 1) +
                  "m,0°)";
    }
    INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "目标轨迹预报功能测试", forecast);

    if (traj.has_impact) {
      std::string impact = "目标键=" + std::to_string(i < tracks.size() ? tracks[i].key : 0U);
      impact += " 初步落点经纬高=" + FormatVec3(traj.impact_point.latitude_deg,
                                                traj.impact_point.longitude_deg,
                                                traj.impact_point.altitude_m, 3);
      INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "落点预测功能测试", impact);

      // 评审 2026-08-26 条10：误差口径改为关机点误差（关机点 1-σ，敏度传播；
      // 关机点未确认/无协方差时如实写无）。条12：分发状态不暴露落盘细节——配置了
      // 分发通道名写已发布，否则已封装待分发。
      // 验收判定标准 第25项（修改）：不写 弹道模型/标准化封装/硬编码置信度占位；
      // 分发状态=已发布 须随 分发来源ID/分发对象ID（装配层标注的实体 ID）。
      const bool has_burnout_sigma = traj.has_burnout_sigma;
      const double publish_sigma = has_burnout_sigma ? traj.burnout_position_sigma_m : 0.0;
      const std::string sigma_text =
          has_burnout_sigma ? FormatF(publish_sigma, 1) + "m" : std::string();
      std::string publish = "目标键=" + std::to_string(i < tracks.size() ? tracks[i].key : 0U);
      publish += " 预测落点=" + FormatVec3(traj.impact_point.latitude_deg,
                                           traj.impact_point.longitude_deg,
                                           traj.impact_point.altitude_m, 3);
      if (!sigma_text.empty()) {
        publish += " 关机点误差1σ=" + sigma_text;
      }
      if (config.impact_distribution_channel.empty()) {
        publish += " 分发状态=已封装待分发";
      } else {
        publish += " 分发状态=已发布";
        if (config.impact_distribution_source_id != 0U) {
          publish += " 分发来源ID=" + std::to_string(config.impact_distribution_source_id);
        }
        if (config.impact_distribution_target_id != 0U) {
          publish += " 分发对象ID=" + std::to_string(config.impact_distribution_target_id);
        }
      }
      INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "落点预报与信息分发功能测试", publish);
    }

    if (traj.has_launch) {
      double a = 0.0;
      double b = 0.0;
      double az = 0.0;
      HorizontalEllipseFromCov(traj.launch_covariance_ecef, &a, &b, &az);
      // 评审 2026-08-26 条8：发射时刻字段删除，行首直接输出地理位置。
      // 验收判定标准 第22项（修改）：键名对齐规范「预测发射点地理位置」；位置误差1σ
      // + 误差椭圆（不写硬编码占位置信度）。
      std::string launch = "目标键=" + std::to_string(i < tracks.size() ? tracks[i].key : 0U);
      launch += " 预测发射点地理位置=" + FormatVec3(traj.launch_point.latitude_deg,
                                                   traj.launch_point.longitude_deg,
                                                   traj.launch_point.altitude_m, 3);
      launch += " 位置误差1σ=" + FormatF(traj.launch_position_sigma_m, 1) + "m";
      launch += " 误差椭圆半长/半短/方位=(" + FormatF(a, 1) + "m," + FormatF(b, 1) + "m," +
                FormatF(az, 1) + "°)";
      INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "发射点预测功能测试", launch);
    }

    if (i < tracks.size()) {
      const InferenceTrackState& track = tracks[i];
      LastVel& prev = LastVelocities()[track.key];
      const double speed = std::sqrt(track.velocity_ecef_m_per_s[0] * track.velocity_ecef_m_per_s[0] +
                                     track.velocity_ecef_m_per_s[1] * track.velocity_ecef_m_per_s[1] +
                                     track.velocity_ecef_m_per_s[2] * track.velocity_ecef_m_per_s[2]);
      if (prev.valid) {
        const double dv = std::sqrt(
            (track.velocity_ecef_m_per_s[0] - prev.vx) * (track.velocity_ecef_m_per_s[0] - prev.vx) +
            (track.velocity_ecef_m_per_s[1] - prev.vy) * (track.velocity_ecef_m_per_s[1] - prev.vy) +
            (track.velocity_ecef_m_per_s[2] - prev.vz) * (track.velocity_ecef_m_per_s[2] - prev.vz));
        const double rel = dv / std::max(speed, 1.0);
        if (rel > 0.2) {
          // 规范口径（验收判定标准 第24项·轨迹突变）：「验收内容：」后首字段为 事件=，
          // 位置写 LLA（推演侧事件与 SBIRS 侧同项名分行）。
          std::string event = "事件=轨迹突变";
          event += " 目标键=" + std::to_string(track.key);
          event += " 等级=3 相对Δv=" + FormatF(rel, 3);
          oneq::coordinate::LlaPositionDegM event_lla{};
          if (oneq::coordinate::TryEcefToLla(track.position, &event_lla)) {
            event += " 位置LLA=" + FormatVec3(event_lla.latitude_deg, event_lla.longitude_deg,
                                             event_lla.altitude_m, 3);
          }
          INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "特殊事件监测与提示功能测试",
                                    event);
        }
      }
      prev.vx = track.velocity_ecef_m_per_s[0];
      prev.vy = track.velocity_ecef_m_per_s[1];
      prev.vz = track.velocity_ecef_m_per_s[2];
      prev.valid = true;
    }
  }
}

}  // namespace target_inference

void WriteSatelliteTargetAcceptanceLine(const std::string& item, const std::string& content) {
  INFERENCE_ACCEPTANCE_ITEM(0.0f, 0U, item.c_str(), content);
}
