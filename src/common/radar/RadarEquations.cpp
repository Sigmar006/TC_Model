#include "common/radar/RadarEquations.h"

#include <algorithm>
#include <cmath>
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-W#warnings"
#endif
#include <boost/math/special_functions/gamma.hpp>
#include "common/numerics/Constants.h"
#include "common/numerics/NumericGuard.h"
#if defined(__clang__)
#pragma clang diagnostic pop
#endif

namespace oneq {
namespace common {
namespace radar {

namespace {
/**
 * @brief 光速 (m/s)。
 * @note 代码行为依据：当前实现用该值完成波长与距离分辨力换算。
 */
/**
 * @brief 玻尔兹曼常数 (J/K)。
 * @note 代码行为依据：当前实现用该值计算热噪声功率 `k*T*B*F`。
 */
/**
 * @brief IEEE 标准参考温度 (K)。
 * @note 代码行为依据：当前实现把热噪声参考温度固定为该值，与
 *       `ComputeThermalNoisePower_W()` 的噪声底计算保持一致。
 */
const float kRefTemperature = 290.0f;
/**
 * @brief π 常数。
 * @note 代码行为依据：当前实现用该值展开雷达方程中的 `4π` 项。
 */
/**
 * @brief 参考脉宽（单位：s），用于把峰值功率映射到单脉冲能量尺度。
 * @note 取配置默认值，确保默认参数下行为连续。
 */
const float kReferencePulseWidthS = 13.0e-6f;
using oneq::common::numerics::kLog10Floor;
/**
 * @brief 将线性值转为 dB。
 * @param linear 线性值。
 * @return 对应的 dB 值。
 */
float LinearToDb(float linear) {
  if (linear <= kLog10Floor) {
    return 10.0f * std::log10(kLog10Floor);
  }
  return 10.0f * std::log10(linear);
}

/**
 * @brief 计算相对参考脉宽的单脉冲能量缩放。
 * @param tx 发射机参数。
 * @return 线性能量缩放因子，最小钳位到机器精度量级。
 */
float ComputePulseEnergyScale(float pulse_width_s) {
  constexpr float kMinEnergyScale = 1e-12f;
  if (!std::isfinite(pulse_width_s) || pulse_width_s <= 0.0f) {
    return kMinEnergyScale;
  }
  return std::max(pulse_width_s / kReferencePulseWidthS, kMinEnergyScale);
}
/**
 * @brief 将 dB 转为线性值。
 * @param db dB 值。
 * @return 对应的线性值。
 */
float DbToLinear(float db) { return std::pow(10.0f, db / 10.0f); }
/**
 * @brief 钳位到 [0, 1]。
 * @param pd 输入检测概率。
 * @return 钳位后的检测概率。
 */
float ClampPd(float pd) {
  if (pd < 0.0f) {
    return 0.0f;
  }
  if (pd > 1.0f) {
    return 1.0f;
  }
  return pd;
}

}  // namespace

float RadarEquations::ComputeEchoPowerWithGain_dBW(float peak_power_w,
                                                   float transmit_loss_db, float pulse_width_s,
                                                   float frequency_hz, float one_way_gain_db,
                                                   float rcs_m2, float range_m,
                                                   float propagation_loss_db) {
  if (range_m <= 0.0f || rcs_m2 <= 0.0f || frequency_hz <= 0.0f) {
    return -300.0f;
  }

  const float wavelength_m =
      static_cast<float>(oneq::common::numerics::kLightSpeed) / frequency_hz;
  const float pt_db = LinearToDb(peak_power_w);
  const float pulse_energy_scale_db = LinearToDb(ComputePulseEnergyScale(pulse_width_s));
  /* lambda_db = 10·log10(λ)，公式中乘以 2 等价于 10·log10(λ²) */
  const float lambda_db = LinearToDb(wavelength_m);
  /* r_db = 10·log10(R)，公式中乘以 4 等价于 10·log10(R⁴) */
  const float r_db = LinearToDb(range_m);
  /* rcs_db = 10·log10(σ)，标准雷达方程中 RCS 为线性值 */
  const float rcs_db = LinearToDb(rcs_m2);
  /* 总损耗 L_sys 包含发射系统损耗和传播损耗 */
  const float total_loss_db = transmit_loss_db + propagation_loss_db;

  const float pr_dbw = pt_db + pulse_energy_scale_db + one_way_gain_db + one_way_gain_db +
                       2.0f * lambda_db + rcs_db - 30.0f * std::log10(4.0f * static_cast<float>(oneq::common::numerics::kPi)) - 4.0f * r_db -
                       total_loss_db;

  return pr_dbw;
}

float RadarEquations::ComputeEchoPower_dBW(float peak_power_w,
                                           float transmit_loss_db, float pulse_width_s,
                                           float frequency_hz, float main_beam_gain_db,
                                           float rcs_m2, float range_m,
                                           float propagation_loss_db) {
  return ComputeEchoPowerWithGain_dBW(peak_power_w, transmit_loss_db, pulse_width_s,
                                      frequency_hz, main_beam_gain_db, rcs_m2, range_m,
                                      propagation_loss_db);
}

float RadarEquations::ComputeThermalNoisePower_W(float bandwidth_hz,
                                                 float noise_figure_db) {
  const float noise_figure_linear = DbToLinear(noise_figure_db);
  return static_cast<float>(oneq::common::numerics::kBoltzmann) * kRefTemperature * bandwidth_hz *
         noise_figure_linear;
}

float RadarEquations::ComputeIntegrationGain(int pulse_count) {
  if (pulse_count <= 0) {
    return 1.0f;
  }
  return static_cast<float>(pulse_count);
}

float RadarEquations::ComputeRangeErrorStdDev(float snr_db, float bandwidth_hz) {
  const float range_resolution = 0.5f * static_cast<float>(oneq::common::numerics::kLightSpeed) / bandwidth_hz;
  const float kMinSnrDb = -10.0f;
  if (snr_db < kMinSnrDb) {
    return range_resolution * 1.5777f;
  }
  const float snr_linear = DbToLinear(snr_db);
  // 纯随机项：固定系统偏置 kRangeMeasurementBiasM（20 m，工程实测数据拟合的
  // 系统偏置/量化误差等固定分量）已于 2026-08-30 拆出，由消费方施加在量测均值侧。
  return 0.5f * range_resolution / std::sqrt(snr_linear);
}

float RadarEquations::ComputeAngleErrorStdDev(float snr_db, float beamwidth_rad) {
  const float kMinSnrDb = -10.0f;
  if (snr_db < kMinSnrDb) {
    return beamwidth_rad;
  }
  const float snr_linear = DbToLinear(snr_db);
  // 纯随机项：固定系统偏置 θ_bw/30（见 ComputeAngleMeasurementBiasRad）已于
  // 2026-08-30 拆出，由消费方施加在量测均值侧。
  return 0.317f * beamwidth_rad / std::sqrt(snr_linear);
}

double RadarEquations::ComputeThreshold(double pfa, int num_pulses) {
  if (pfa <= 0.0 || pfa >= 1.0) {
    pfa = 1e-6;
  }
  if (num_pulses <= 1) {
    // N=1: P_fa = exp(-T) → T = -ln(P_fa)
    return -std::log(pfa);
  }
  /**
   *  N>1: 求解 Q(N, T) = P_fa
   * boost::math::gamma_q_inv(a, q) 返回 x 使得 Q(a,x) = q
   */
  return boost::math::gamma_q_inv(static_cast<double>(num_pulses), pfa);
}

double RadarEquations::MarcumQ(int order, double a, double b) {
  /**
   *  Q_M(a, b) = Σ_{k=0}^∞ [e^{-λ} · λ^k / k!] · Q(M+k, b²/2)
   *  其中 λ = a²/2
   *
   *  数值稳定性：从 Poisson 分布峰值 k₀ ≈ ⌊λ⌋ 处开始，
   *  向两侧累加。避免从 k=0 开始导致 exp(-λ) 下溢。
   */

  if (b <= 0.0) {
    return 1.0;
  }
  const double a_clamped = (a < 0.0) ? 0.0 : a;

  /**
   *  极大信噪比情况（极远大于检测门限），由于 Poisson 分布的方差导致两翼截断误差会显现出数值不稳定
   *  根据渐进性，当 a >> b 时，检测概率必然趋于 1.0
   */
  if (a_clamped > b + 20.0) {
    return 1.0;
  }

  const double lambda = a_clamped * a_clamped / 2.0;
  const double x = b * b / 2.0;

  const int kMaxIter = 500;
  const double kConvergence = 1e-12;

  /* 从 Poisson 峰值处开始 */
  const int k0 = static_cast<int>(lambda);

  /**
   *  计算 k0 处的 log(Poisson(k0, λ))，然后以此为基准
   *  log P(k, λ) = -λ + k·ln(λ) - ln(k!)
   *  使用 lgamma 计算 ln(k!) = lgamma(k+1)
   */
  auto log_poisson = [lambda](int k) -> double {
    if (k == 0) {
      return -lambda;
    }
    return -lambda + k * std::log(lambda) - std::lgamma(k + 1);
  };

  double sum = 0.0;

  /* 向右累加：k = k0, k0+1, k0+2, ... */
  {
    double log_pk = log_poisson(k0);
    for (int k = k0; k < k0 + kMaxIter; ++k) {
      if (k > k0) {
        log_pk += std::log(lambda / static_cast<double>(k));
      }
      const double pk = std::exp(log_pk);
      const double gq = boost::math::gamma_q(static_cast<double>(order + k), x);
      const double term = pk * gq;
      sum += term;
      if (term < kConvergence && k > k0 + 2) {
        break;
      }
    }
  }

  /* 向左累加：k = k0-1, k0-2, ..., 0 */
  {
    double log_pk = log_poisson(k0 > 0 ? k0 - 1 : 0);
    for (int k = (k0 > 0 ? k0 - 1 : -1); k >= 0; --k) {
      if (k < k0 - 1) {
        log_pk += std::log(static_cast<double>(k + 1) / lambda);
      }
      const double pk = std::exp(log_pk);
      const double gq = boost::math::gamma_q(static_cast<double>(order + k), x);
      const double term = pk * gq;
      sum += term;
      if (term < kConvergence && k < k0 - 2) {
        break;
      }
    }
  }

  if (sum < 0.0) {
    return 0.0;
  }
  if (sum > 1.0) {
    return 1.0;
  }
  return sum;
}

float RadarEquations::ComputeDetectionProbability(float snr_db, float pfa,
                                                  SwerlingModel model,
                                                  int num_pulses) {
  if (pfa <= 0.0f || pfa >= 1.0f) {
    pfa = 1e-6f;
  }
  if (num_pulses < 1) {
    num_pulses = 1;
  }

  const double chi = static_cast<double>(DbToLinear(snr_db));
  const int N = num_pulses;
  const double T = ComputeThreshold(static_cast<double>(pfa), N);
  double pd = 0.0;

  switch (model) {
    case SwerlingModel::kSwerling0: {
      const double a = std::sqrt(2.0 * N * chi);
      const double b = std::sqrt(2.0 * T);
      pd = MarcumQ(N, a, b);
      break;
    }
    case SwerlingModel::kSwerling1: {
      const double total_snr = N * chi;  // N 个脉冲的总 SNR
      if (N == 1) {
        pd = std::exp(-T / (1.0 + chi));
      } else {
        const double V = T / (1.0 + total_snr);
        const double C = total_snr / (1.0 + total_snr);
        const double a_shape = static_cast<double>(N - 1);

        const double q1 = boost::math::gamma_q(a_shape, V);

        if (C < 1e-15) {
          pd = q1;
        } else {
          const double q2 = boost::math::gamma_q(a_shape, V / C);
          pd = q1 + std::pow(C, a_shape) * (q2 - q1);
        }
      }
      break;
    }

    case SwerlingModel::kSwerling2: {
      const double cT = T / (1.0 + chi);
      pd = boost::math::gamma_q(static_cast<double>(N), cT);
      break;
    }

    case SwerlingModel::kSwerling3: {
      const double total_snr = N * chi;  // N 个脉冲的总 SNR
      if (N == 1) {
        const double u = 2.0 * T / (2.0 + chi);
        pd = (1.0 + u) * std::exp(-u);
      } else {
        const double V = 2.0 * T / (2.0 + total_snr);
        const double C = total_snr / (2.0 + total_snr);
        const double M = 2.0 * (N - 1);

        if (C < 1e-15) {
          pd = boost::math::gamma_q(M, V);
        } else {
          const double q_M_V = boost::math::gamma_q(M, V);
          const double q_M_VC = boost::math::gamma_q(M, V / C);
          const double c_pow_M = std::pow(C, M);
          pd = q_M_V + c_pow_M * (q_M_VC - q_M_V);

          if (M >= 2.0) {
            const double q_M1_V = boost::math::gamma_q(M - 1.0, V);
            const double q_M1_VC = boost::math::gamma_q(M - 1.0, V / C);
            pd += M * std::pow(C, M - 1.0) * (1.0 - C) * (q_M1_VC - q_M1_V);
          }
        }
      }
      break;
    }
    case SwerlingModel::kSwerling4: {
      const double u = 2.0 * T / (2.0 + chi);
      pd = boost::math::gamma_q(2.0 * N, u);
      break;
    }

    default: {
      const double a = std::sqrt(2.0 * N * chi);
      const double b = std::sqrt(2.0 * T);
      pd = MarcumQ(N, a, b);
      break;
    }
  }

  return ClampPd(static_cast<float>(pd));
}

bool RadarEquations::ThresholdDecision(float detection_prob, std::mt19937& rng) {
  std::uniform_real_distribution<float> dist(0.0f, 1.0f);
  const float r = dist(rng);
  return r <= detection_prob;
}

}  // namespace radar
}  // namespace common
}  // namespace oneq
