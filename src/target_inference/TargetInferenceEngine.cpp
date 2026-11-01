/**
 * @file TargetInferenceEngine.cpp
 * @brief TargetInferenceEngine 实现：弹道前向/回推 + 敏度误差预算 + 类型融合。
 */

#include "1q/target_inference/TargetInferenceEngine.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "target_inference/InferenceAcceptanceLog.h"
#include "target_inference/InferenceAcceptanceRecords.h"

namespace target_inference {

namespace {

using oneq::coordinate::EcefPositionM;
using oneq::coordinate::LlaPositionDegM;

/** @brief 动力学状态（ECEF 位置 + 速度）。 */
struct RvState {
  std::array<double, 3U> r = std::array<double, 3U>();
  std::array<double, 3U> v = std::array<double, 3U>();
};

double Norm3(const std::array<double, 3U>& a) {
  return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

/** @brief 加速度：中心引力 + 可选指数大气阻力（ballistic coefficient ≤ 0 关闭）。 */
std::array<double, 3U> Acceleration(const std::array<double, 3U>& r,
                                    const std::array<double, 3U>& v,
                                    const TargetInferenceConfig& config) {
  const double radius = Norm3(r);
  const double gravity_scale =
      (radius > 1.0e-6) ? -config.earth_mu_m3_per_s2 / (radius * radius * radius) : 0.0;
  std::array<double, 3U> a{{gravity_scale * r[0], gravity_scale * r[1], gravity_scale * r[2]}};
  if (config.drag_ballistic_coefficient_m2_per_kg > 0.0) {
    const double altitude = radius - config.earth_radius_m;
    const double density = 1.225 * std::exp(-std::max(altitude, 0.0) / 8500.0);
    const double speed = Norm3(v);
    const double drag_scale =
        -0.5 * density * speed / config.drag_ballistic_coefficient_m2_per_kg;
    for (int i = 0; i < 3; ++i) {
      a[static_cast<std::size_t>(i)] += drag_scale * v[static_cast<std::size_t>(i)];
    }
  }
  return a;
}

/** @brief RK4 单步（dt 可为负：负值即回推）。k.r 成员承载位置导数（=v），k.v 承载速度导数（=a）。 */
RvState Rk4Step(const RvState& state, double dt, const TargetInferenceConfig& config) {
  const auto accel = [&config](const RvState& s) { return Acceleration(s.r, s.v, config); };

  RvState k1;
  k1.r = state.v;
  k1.v = accel(state);
  RvState s2;
  for (int i = 0; i < 3; ++i) {
    const auto j = static_cast<std::size_t>(i);
    s2.r[j] = state.r[j] + 0.5 * dt * k1.r[j];
    s2.v[j] = state.v[j] + 0.5 * dt * k1.v[j];
  }
  RvState k2;
  k2.r = s2.v;
  k2.v = accel(s2);
  RvState s3;
  for (int i = 0; i < 3; ++i) {
    const auto j = static_cast<std::size_t>(i);
    s3.r[j] = state.r[j] + 0.5 * dt * k2.r[j];
    s3.v[j] = state.v[j] + 0.5 * dt * k2.v[j];
  }
  RvState k3;
  k3.r = s3.v;
  k3.v = accel(s3);
  RvState s4;
  for (int i = 0; i < 3; ++i) {
    const auto j = static_cast<std::size_t>(i);
    s4.r[j] = state.r[j] + dt * k3.r[j];
    s4.v[j] = state.v[j] + dt * k3.v[j];
  }
  RvState k4;
  k4.r = s4.v;
  k4.v = accel(s4);

  RvState next;
  for (int i = 0; i < 3; ++i) {
    const auto j = static_cast<std::size_t>(i);
    next.r[j] = state.r[j] + dt / 6.0 * (k1.r[j] + 2.0 * k2.r[j] + 2.0 * k3.r[j] + k4.r[j]);
    next.v[j] = state.v[j] + dt / 6.0 * (k1.v[j] + 2.0 * k2.v[j] + 2.0 * k3.v[j] + k4.v[j]);
  }
  return next;
}

bool StateIsFinite(const RvState& state) {
  for (int i = 0; i < 3; ++i) {
    const auto j = static_cast<std::size_t>(i);
    if (!std::isfinite(state.r[j]) || !std::isfinite(state.v[j])) {
      return false;
    }
  }
  return true;
}

RvState MakeRvState(const InferenceTrackState& track) {
  RvState state;
  state.r = {track.position.x_m, track.position.y_m, track.position.z_m};
  state.v = track.velocity_ecef_m_per_s;
  return state;
}

EcefPositionM MakeEcef(const std::array<double, 3U>& r) {
  return EcefPositionM(r[0U], r[1U], r[2U]);
}

bool ToLla(const RvState& state, LlaPositionDegM* lla) {
  return oneq::coordinate::TryEcefToLla(MakeEcef(state.r), lla);
}

/** @brief 6×6 矩阵乘（行主序数组）。 */
std::array<double, 36U> Multiply6x6(const std::array<double, 36U>& a,
                                    const std::array<double, 36U>& b) {
  std::array<double, 36U> out{};
  for (std::size_t i = 0U; i < 6U; ++i) {
    for (std::size_t j = 0U; j < 6U; ++j) {
      double sum = 0.0;
      for (std::size_t k = 0U; k < 6U; ++k) {
        sum += a[i * 6U + k] * b[k * 6U + j];
      }
      out[i * 6U + j] = sum;
    }
  }
  return out;
}

/** @brief 运动学类型先验（速度/高度门判，向量按 InferenceTargetCategory 排列）。 */
std::array<double, kInferenceCategoryCount> KinematicPrior(double speed_m_per_s, double altitude_m,
                                                           const TargetInferenceConfig& config) {
  std::array<double, kInferenceCategoryCount> prior{};
  if (speed_m_per_s >= config.high_energy_speed_threshold_m_per_s) {
    if (altitude_m >= config.high_altitude_threshold_m) {
      prior[static_cast<std::size_t>(InferenceTargetCategory::kBallistic)] = 0.55;
      prior[static_cast<std::size_t>(InferenceTargetCategory::kMissile)] = 0.30;
      prior[static_cast<std::size_t>(InferenceTargetCategory::kNearSpace)] = 0.10;
    } else {
      prior[static_cast<std::size_t>(InferenceTargetCategory::kMissile)] = 0.45;
      prior[static_cast<std::size_t>(InferenceTargetCategory::kFighter)] = 0.20;
      prior[static_cast<std::size_t>(InferenceTargetCategory::kOther)] = 0.15;
    }
  } else if (speed_m_per_s >= 500.0) {
    prior[static_cast<std::size_t>(InferenceTargetCategory::kFighter)] = 0.35;
    prior[static_cast<std::size_t>(InferenceTargetCategory::kBomber)] = 0.25;
    prior[static_cast<std::size_t>(InferenceTargetCategory::kOther)] = 0.20;
  } else {
    // 低速低空：原无人机倾向已移除（2026-08-22 甲方裁定），归 kOther。
    prior[static_cast<std::size_t>(InferenceTargetCategory::kOther)] = 0.35;
    prior[static_cast<std::size_t>(InferenceTargetCategory::kFighter)] = 0.10;
  }
  return prior;
}

}  // namespace

TargetInferenceEngine::TargetInferenceEngine(const TargetInferenceConfig& config)
    : config_(config) {}

TargetInferenceEngine::~TargetInferenceEngine() = default;

std::vector<TargetInferenceResult> TargetInferenceEngine::Infer(
    const std::vector<InferenceTrackState>& tracks) const {
  return Infer(tracks, true);
}

std::vector<TargetInferenceResult> TargetInferenceEngine::Infer(
    const std::vector<InferenceTrackState>& tracks, bool write_acceptance) const {
  std::vector<TargetInferenceResult> results;
  results.reserve(tracks.size());

  const double dt = std::max(config_.integration_step_sec, 0.01);

  for (const InferenceTrackState& track : tracks) {
    TargetInferenceResult result;
    result.key = track.key;

    const RvState input_state = MakeRvState(track);
    const bool input_valid =
        StateIsFinite(input_state) && Norm3(input_state.r) > config_.earth_radius_m;
    result.trajectory.valid = input_valid;

    if (input_valid) {
      /* ---- 前向预测 + 落点 ---- */
      {
        RvState state = input_state;
        double t = 0.0;
        double next_waypoint = 0.0;
        while (t < config_.prediction_horizon_sec) {
          const RvState next = Rk4Step(state, dt, config_);
          if (!StateIsFinite(next)) {
            break;
          }
          const double t_next = t + dt;
          if (Norm3(next.r) <= config_.earth_radius_m) {
            /* 地表穿越线性插值 → 落点。 */
            const double r_old = Norm3(state.r);
            const double r_new = Norm3(next.r);
            const double frac =
                (r_old - config_.earth_radius_m) / std::max(r_old - r_new, 1.0e-9);
            RvState impact = state;
            for (int i = 0; i < 3; ++i) {
              const auto j = static_cast<std::size_t>(i);
              impact.r[j] = state.r[j] + frac * (next.r[j] - state.r[j]);
              impact.v[j] = state.v[j] + frac * (next.v[j] - state.v[j]);
            }
            LlaPositionDegM impact_lla{};
            if (ToLla(impact, &impact_lla)) {
              result.trajectory.has_impact = true;
              /* 地表穿越点按本引擎球面地表模型即高度 0；ToLla 的 WGS84 椭球高与
                 球面（赤道半径）混用会产生中纬度约 +5km 的伪高差，交付口径归零。 */
              impact_lla.altitude_m = 0.0;
              result.trajectory.impact_point = impact_lla;
              result.trajectory.impact_time_offset_sec = t + frac * dt;
            }
            break;
          }
          if (t_next >= next_waypoint) {
            LlaPositionDegM waypoint_lla{};
            if (ToLla(next, &waypoint_lla)) {
              InferenceWaypoint waypoint;
              waypoint.time_offset_sec = t_next;
              waypoint.position = waypoint_lla;
              result.trajectory.waypoints.push_back(waypoint);
            }
            next_waypoint += std::max(config_.waypoint_interval_sec, dt);
          }
          state = next;
          t = t_next;
        }
      }

      /* ---- 发射点回推（大气界面停机门/地表交点/速度停机门；助推段未建模，
              停机点即"助推段上界"估计） ---- */
      auto backtrack = [this, dt](const RvState& start, double* elapsed,
                                  bool* hit_surface) -> RvState {
        RvState state = start;
        double t = 0.0;
        *hit_surface = false;
        while (t < config_.launch_max_backtrack_sec) {
          const RvState next = Rk4Step(state, -dt, config_);
          if (!StateIsFinite(next)) {
            break;
          }
          const double r_next = Norm3(next.r);
          if (r_next <= config_.earth_radius_m) {
            const double r_old = Norm3(state.r);
            const double frac =
                (r_old - config_.earth_radius_m) / std::max(r_old - r_next, 1.0e-9);
            RvState launch = state;
            for (int i = 0; i < 3; ++i) {
              const auto j = static_cast<std::size_t>(i);
              launch.r[j] = state.r[j] + frac * (next.r[j] - state.r[j]);
              launch.v[j] = state.v[j] + frac * (next.v[j] - state.v[j]);
            }
            *hit_surface = true;
            *elapsed = -(t + frac * dt);
            return launch;
          }
          // 大气界面停机门：回推状态地心高降至界面高度即停——继续回推将进入
          // 助推段（推力未建模），弹道外推发散（审计 A7）。
          if (r_next - config_.earth_radius_m <=
              config_.launch_atmosphere_interface_altitude_m) {
            *elapsed = -(t + dt);
            return next;
          }
          // 中译：**速度停机门只在低空生效**（2026-11-02）——原实现对任意高度都停机，
          //   于是"高空、速度接近零"的异常状态（保活/合成位置航迹常见）会在**卫星轨道
          //   高度**直接停机，把那个点当成"发射点/关机点"（现场：[predict-out] launch=930，
          //   第 22 项零行、第 23 项全占位，因为输出的点被高度门判为假值）。
          //   速度门本意是"回推进了助推段（推力未建模）就收手"，这只有在低空才有意义；
          //   高空应继续回推，让弹道解真正降到大气界面，解不出就按上限超时丢弃（诚实）。
          // 标识：回推速度门只在低空生效（2026-11-02）。
          if (r_next - config_.earth_radius_m <=
                  config_.launch_atmosphere_interface_altitude_m &&
              Norm3(next.v) <= config_.launch_speed_threshold_m_per_s) {
            *elapsed = -(t + dt);
            return next;
          }
          state = next;
          t += dt;
        }
        *elapsed = 0.0;
        return start;  // 未解算出交点：调用方以 has_launch=false 丢弃。
      };

      double launch_elapsed = 0.0;
      bool launch_hit_surface = false;
      const RvState launch_state = backtrack(input_state, &launch_elapsed, &launch_hit_surface);
      if (launch_elapsed < 0.0) {
        LlaPositionDegM launch_lla{};
        if (ToLla(launch_state, &launch_lla)) {
          if (launch_hit_surface) {
            /* 同落点口径：地表交点高度归零；速度门停机分支保留真实空中高度。 */
            launch_lla.altitude_m = 0.0;
          }
          result.trajectory.has_launch = true;
          result.trajectory.launch_point = launch_lla;
          result.trajectory.launch_time_offset_sec = launch_elapsed;
        }
      }

      /* ---- 误差预算：线性化敏度 J·P·Jᵀ（6 个扰动态重推） ---- */
      if (track.has_covariance) {
        result.trajectory.has_uncertainty = true;
        /* 状态向量 [x,vx,y,vy,z,vz]；扰动步长位置 100 m / 速度 1 m/s（动力学平滑，敏度
           对步长不敏感；固定步长保证确定性）。 */
        std::array<double, 6U> eps{100.0, 1.0, 100.0, 1.0, 100.0, 1.0};
        std::array<double, 36U> sensitivity{};  // sensitivity(i*6+j) = d(out_i)/d(in_j)
        for (std::size_t j = 0U; j < 6U; ++j) {
          RvState perturbed = input_state;
          if (j % 2U == 0U) {
            perturbed.r[j / 2U] += eps[j];
          } else {
            perturbed.v[j / 2U] += eps[j];
          }
          double perturbed_elapsed = 0.0;
          bool perturbed_hit_surface = false;
          const RvState perturbed_launch =
              backtrack(perturbed, &perturbed_elapsed, &perturbed_hit_surface);
          const std::array<double, 6U> nominal_in{input_state.r[0U], input_state.v[0U],
                                                  input_state.r[1U], input_state.v[1U],
                                                  input_state.r[2U], input_state.v[2U]};
          const std::array<double, 6U> perturbed_in{
              perturbed.r[0U], perturbed.v[0U], perturbed.r[1U], perturbed.v[1U],
              perturbed.r[2U], perturbed.v[2U]};
          const std::array<double, 6U> nominal_out{
              launch_state.r[0U], launch_state.v[0U], launch_state.r[1U], launch_state.v[1U],
              launch_state.r[2U], launch_state.v[2U]};
          const std::array<double, 6U> perturbed_out{
              perturbed_launch.r[0U], perturbed_launch.v[0U], perturbed_launch.r[1U],
              perturbed_launch.v[1U], perturbed_launch.r[2U], perturbed_launch.v[2U]};
          for (std::size_t i = 0U; i < 6U; ++i) {
            sensitivity[i * 6U + j] =
                (perturbed_out[i] - nominal_out[i]) / (perturbed_in[j] - nominal_in[j]);
          }
        }
        /* launch_cov = J·P·Jᵀ；位置对角再叠加随回推时长线性增长的模型误差主项
           （助推段/模型未建模误差：σ_model = rate × 回推时长，审计 A7）。 */
        std::array<double, 36U> jt{};
        for (std::size_t i = 0U; i < 6U; ++i) {
          for (std::size_t j = 0U; j < 6U; ++j) {
            jt[j * 6U + i] = sensitivity[i * 6U + j];
          }
        }
        std::array<double, 36U> launch_cov =
            Multiply6x6(Multiply6x6(sensitivity, track.covariance_ecef), jt);
        const double model_variance =
            config_.launch_model_error_rate_m_per_s * config_.launch_model_error_rate_m_per_s *
            launch_elapsed * launch_elapsed;
        for (std::size_t i = 0U; i < 6U; i += 2U) {
          launch_cov[i * 6U + i] += model_variance;
        }
        result.trajectory.launch_covariance_ecef = launch_cov;
        double max_position_variance = 0.0;
        for (std::size_t i = 0U; i < 6U; i += 2U) {
          max_position_variance = std::max(max_position_variance, launch_cov[i * 6U + i]);
        }
        result.trajectory.launch_position_sigma_m = std::sqrt(std::max(max_position_variance, 0.0));

        /* 落点 1-σ：同一敏度框架的前向版本（无落点则跳过）。 */
        if (result.trajectory.has_impact) {
          const double impact_time = result.trajectory.impact_time_offset_sec;
          auto forward_at = [this, dt](const RvState& start, double duration, RvState* out) {
            RvState state = start;
            double t = 0.0;
            while (t < duration) {
              const double step = std::min(dt, duration - t);
              state = Rk4Step(state, step, config_);
              t += step;
              if (!StateIsFinite(state)) {
                return false;
              }
            }
            *out = state;
            return true;
          };
          RvState nominal_impact{};
          if (forward_at(input_state, impact_time, &nominal_impact)) {
            double max_impact_variance = 0.0;
            for (std::size_t j = 0U; j < 6U; ++j) {
              RvState perturbed = input_state;
              if (j % 2U == 0U) {
                perturbed.r[j / 2U] += eps[j];
              } else {
                perturbed.v[j / 2U] += eps[j];
              }
              RvState perturbed_impact{};
              if (!forward_at(perturbed, impact_time, &perturbed_impact)) {
                continue;
              }
              /* 落点位置敏度行（3×6）：协方差贡献 Σ_j H_ij² P_jj 近似（对角占优简化，
                 完整 J·P·Jᵀ 的非对角项贡献在高维耦合下二阶小；证据级口径）。 */
              double sensitivity_sq_sum = 0.0;
              for (std::size_t i = 0U; i < 3U; ++i) {
                const double h =
                    (perturbed_impact.r[i] - nominal_impact.r[i]) / eps[j];
                sensitivity_sq_sum += h * h;
              }
              max_impact_variance += sensitivity_sq_sum * track.covariance_ecef[j * 6U + j];
            }
            result.trajectory.impact_position_sigma_m =
                std::sqrt(std::max(max_impact_variance, 0.0));
          }
        }
      }
    }

    /* ---- 类型评估：运动学先验 + 外部证据加权融合 ---- */
    {
      const double speed = Norm3(input_state.v);
      const double altitude = Norm3(input_state.r) - config_.earth_radius_m;
      const auto prior = KinematicPrior(speed, altitude, config_);
      std::array<double, kInferenceCategoryCount> scores{};
      double sum = 0.0;
      for (std::size_t i = 0U; i < kInferenceCategoryCount; ++i) {
        scores[i] = track.has_type_evidence
                        ? config_.kinematic_type_weight * prior[i] +
                              (1.0 - config_.kinematic_type_weight) * track.type_evidence[i]
                        : prior[i];
        scores[i] = std::max(scores[i], 0.0);
        sum += scores[i];
      }
      if (sum > 1.0e-12) {
        std::size_t best = 0U;
        for (std::size_t i = 1U; i < kInferenceCategoryCount; ++i) {
          if (scores[i] > scores[best]) {
            best = i;
          }
        }
        result.type.category = static_cast<InferenceTargetCategory>(best);
        result.type.probability = scores[best] / sum;
      }
    }

    results.push_back(std::move(result));
  }
  if (write_acceptance && INFERENCE_ACCEPTANCE_LOG_ENABLED()) {
    WriteInferenceAcceptance(tracks, results, config_);
  }
  return results;
}

double TargetInferenceEngine::PositionSigmaAt(const InferenceTrackState& track,
                                              double time_offset_sec,
                                              const TargetInferenceConfig& config) {
  if (!track.has_covariance || !std::isfinite(time_offset_sec)) {
    return 0.0;
  }
  const double dt = std::max(config.integration_step_sec, 0.01);
  const RvState input_state = MakeRvState(track);
  if (!StateIsFinite(input_state)) {
    return 0.0;
  }
  /* 与落点 1-σ 同框架的 6 扰动对角简化：把标称与 6 个扰动态积分到同一时刻偏移
     （Rk4Step 支持负 dt，可回推到关机时刻早于当前拍的场景），位置敏度行
     h_ij = (perturbed.r_i − nominal.r_i)/eps_j，方差 ≈ Σ_j Σ_i h_ij²·P_jj。 */
  auto propagate_to = [&config, dt](const RvState& start, double duration, RvState* out) {
    RvState state = start;
    double t = 0.0;
    const double sign = duration >= 0.0 ? 1.0 : -1.0;
    const double magnitude = std::fabs(duration);
    while (t < magnitude) {
      const double step = std::min(dt, magnitude - t);
      state = Rk4Step(state, sign * step, config);
      t += step;
      if (!StateIsFinite(state)) {
        return false;
      }
    }
    *out = state;
    return true;
  };
  RvState nominal{};
  if (!propagate_to(input_state, time_offset_sec, &nominal)) {
    return 0.0;
  }
  const std::array<double, 6U> eps{100.0, 1.0, 100.0, 1.0, 100.0, 1.0};
  double variance = 0.0;
  for (std::size_t j = 0U; j < 6U; ++j) {
    RvState perturbed = input_state;
    if (j % 2U == 0U) {
      perturbed.r[j / 2U] += eps[j];
    } else {
      perturbed.v[j / 2U] += eps[j];
    }
    RvState perturbed_out{};
    if (!propagate_to(perturbed, time_offset_sec, &perturbed_out)) {
      continue;
    }
    for (std::size_t i = 0U; i < 3U; ++i) {
      const double h = (perturbed_out.r[i] - nominal.r[i]) / eps[j];
      variance += h * h * track.covariance_ecef[j * 6U + j];
    }
  }
  return std::sqrt(std::max(variance, 0.0));
}

bool TargetInferenceEngine::PositionAt(const InferenceTrackState& track, double time_offset_sec,
                                       const TargetInferenceConfig& config,
                                       oneq::coordinate::LlaPositionDegM* out_lla) {
  if (out_lla == nullptr || !std::isfinite(time_offset_sec)) {
    return false;
  }
  const RvState input_state = MakeRvState(track);
  if (!StateIsFinite(input_state)) {
    return false;
  }
  const double dt = std::max(config.integration_step_sec, 0.01);
  RvState state = input_state;
  double t = 0.0;
  const double sign = time_offset_sec >= 0.0 ? 1.0 : -1.0;
  const double magnitude = std::fabs(time_offset_sec);
  while (t < magnitude) {
    const double step = std::min(dt, magnitude - t);
    state = Rk4Step(state, sign * step, config);
    t += step;
    if (!StateIsFinite(state)) {
      return false;
    }
  }
  return ToLla(state, out_lla);
}

}  // namespace target_inference
