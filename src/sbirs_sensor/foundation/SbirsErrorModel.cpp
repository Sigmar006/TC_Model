/**
 * ## 过程：测角误差模型（物理 RSS 与 SNR 线性标定两条口径）（红外处理链 · 第 7 步）
 * - 上游：真值方位/俯仰/斜距（第 3 步）、相对视线角速度、本拍线性 SNR（第 5 步）、
 *   IFOV = 像元间距/焦距（硬件配置）、`SbirsErrorModelConfig` 与随机源。
 * - 本步：产出一帧（或 N 帧融合）的带误差量测：
 *   · 随机项 σ：默认走"线性 SNR 标定"——35 dB→1 µrad、10 dB→5 µrad，dB 域线性插值、
 *     区间外按端点钳位（`ResolveLinearSnrSigmaDeg`）；关闭标定时回落到物理模型
 *     √(σ_floor² + (IFOV/(k·√SNR))²)，σ_floor = √(σ_轨道²+σ_姿态²+σ_视场²)；
 *   · 确定性偏差：折射 δ = 1.5e-6/(R·|cos β|)（deg）与动态滞后 δ = ω/(2π·f_c)（deg），
 *     两项对方位/俯仰同加；
 *   · 距离乘性误差 R_meas = R_true·(1 + Δ)，Δ ~ N(0, range_fraction_sigma)；
 *   · 融合路径按 N 帧独立抽样后取均值，随机项 1-σ 降为 σ/√N（确定性偏差不衰减）。
 * - 下游：`SbirsTrackingCoordinator::CorrectTarget` 的量测及其 2×2 量测协方差 R 的 σ 口径
 *   （`tracking::BuildMeasurementCovariance` 复用同一批 σ），以及 WFOV/Sensor-like 输出角。
 * - 关键参数：`error_model.orbit_sigma_deg`(5e-5)、`attitude_sigma_deg`(1e-4)、
 *   `fov_sigma_deg`(5e-5)、`detector_bandwidth_hz`(100)、`range_fraction_sigma`(0.001)、
 *   `random_seed`(1)；线性标定锚点为库内常量（默认启用，可用
 *   `SetLinearSnrAngleSigmaEnabled(false)` 关闭）。
 * - 对应验收项：第 9 项 红外载荷角误差功能测试；第 55 项 红外系统测角误差性能测试；
 *   第 77 项 红外量测与跟踪滤波交互接口测试（量测 σ 与 NIS）。
 */

#include "sbirs_sensor/foundation/SbirsErrorModel.h"

#include <cmath>
#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace foundation {
namespace {

using oneq::common::numerics::kPi;
using oneq::common::numerics::DegToRad;

}  // namespace

double SbirsRandomSource::NextUniform() {
  // xorshift32（Marsaglia）。state 永远非零（构造/Restore 时夹紧）。
  std::uint32_t x = state_;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  state_ = x;
  // 映射到 [0,1)，避免返回正好 1.0 破坏 Box-Muller 的 log。
  return (static_cast<double>(state_) / 4294967296.0);
}

double SbirsRandomSource::NextStandardNormal() {
  // Box-Muller 变换。u2 保证 > 0，避免 log(0)。
  const double u1 = NextUniform();
  double u2 = NextUniform();
  if (u2 <= 0.0) {
    u2 = 1.0e-12;
  }
  const double radius = std::sqrt(-2.0 * std::log(u2));
  const double angle = 2.0 * kPi * u1;
  // 仅消费一个正态样本，丢弃另一个以保持确定性推进。
  return radius * std::cos(angle);
}

// 过程：大气折射造成的角度偏差按 1/(R·|cos β|) 衰减——斜距越远、视线越接近天顶，偏差越小。
double RefractionErrorDeg(double range_m, float elevation_deg) {
  if (range_m <= 0.0) {
    return 0.0;
  }
  const double cos_beta = std::cos(DegToRad(static_cast<double>(elevation_deg)));
  // 仰角接近 0° 时 cosβ→1；俯仰角在天基 LOS 下通常较大，cos 不会发散。
  // 1.5e-6 为红外波段折射角系数（design 2.10）。
  const double denominator = range_m * std::fabs(cos_beta);
  if (denominator <= 0.0) {
    return 0.0;
  }
  return 1.5e-6 / denominator;
}

// 过程：一阶滞后环节的稳态误差——视线以 ω 匀速扫过时，滞后 τ=1/(2π·f_c) 秒，即 δ=ω·τ（deg）。
double DynamicLagErrorDeg(float relative_angular_rate_deg_per_sec, float detector_bandwidth_hz) {
  if (detector_bandwidth_hz <= 0.0f) {
    return 0.0;
  }
  return static_cast<double>(relative_angular_rate_deg_per_sec) / (2.0 * kPi * detector_bandwidth_hz);
}

// 过程：仪器地板 σ_floor = √(σ_轨道² + σ_姿态² + σ_视场²)（deg）——平台与载荷的静态误差
//       按独立高斯 RSS，不随 SNR 变化。
double ResolveEffectiveAngularSigmaDeg(const config::SbirsErrorModelConfig& model) {
  const double orbit = static_cast<double>(model.orbit_sigma_deg);
  const double attitude = static_cast<double>(model.attitude_sigma_deg);
  const double fov = static_cast<double>(model.fov_sigma_deg);
  return std::sqrt(orbit * orbit + attitude * attitude + fov * fov);
}

namespace {

// 中译：**SNR↔测角误差线性标定锚点**（2026-11-04 甲方口径）——固定为库内常量，不另开配置：
//   SNR 35 dB ⇒ σ 1.0 µrad；SNR 10 dB ⇒ σ 5.0 µrad；dB 域线性插值，区间外按端点钳位。
//   默认启用；需要回到物理模型时调用 oneq::sbirs_sensor::SetLinearSnrAngleSigmaEnabled(false)。
// 标识：线性标定锚点常量（2026-11-04）。
constexpr double kSigmaLinearSnrHighDb = 35.0;
constexpr double kSigmaLinearHighUrad = 1.0;
constexpr double kSigmaLinearSnrLowDb = 10.0;
constexpr double kSigmaLinearLowUrad = 5.0;

// 全局开关（默认启用线性标定）。库内单线程仿真使用，不引入额外同步开销。
bool g_linear_snr_sigma_enabled = true;

// 中译：**按观测模式覆盖线性锚点**（2026-11-05 大幅面预警扫描）——宽场/窄场共用上面那组
//   锚点，第三种模式（大幅面）需要自己的精度档位（落在宽窄之间），又不能改前两种的既有
//   数值，故用作用域覆盖：进入分支前设置、离开自动恢复（RAII），单线程仿真无需加锁。
// 标识：大幅面模式精度锚点覆盖（2026-11-05）。
struct LinearAnchorOverride {
  bool active{false};
  double high_db{35.0};
  double high_urad{1.0};
  double low_db{10.0};
  double low_urad{5.0};
};
LinearAnchorOverride g_linear_anchor_override;

}  // namespace

bool SetLinearSnrAngleSigmaEnabled(bool enabled) {
  const bool previous = g_linear_snr_sigma_enabled;
  g_linear_snr_sigma_enabled = enabled;
  return previous;
}

bool IsLinearSnrAngleSigmaEnabled() { return g_linear_snr_sigma_enabled; }

// 过程：dB 域线性插值 t=(35−SNR_dB)/(35−10)，σ = 1 + (5−1)·t（µrad），t 钳到 [0,1] 即区间外
//       按端点取；最后乘 180/π·1e-6 换成 deg。SNR 非正时直接取弱端 5 µrad。
double ResolveLinearSnrSigmaDeg(double snr_db) {
  constexpr double kUradToDeg = 180.0 / kPi * 1.0e-6;
  // 过程：命中某观测模式的作用域覆盖时用该模式自己的锚点（否则用库内默认锚点）。
  const double high_db = g_linear_anchor_override.active ? g_linear_anchor_override.high_db
                                                         : kSigmaLinearSnrHighDb;
  const double high_urad = g_linear_anchor_override.active
                               ? g_linear_anchor_override.high_urad
                               : kSigmaLinearHighUrad;
  const double low_db = g_linear_anchor_override.active ? g_linear_anchor_override.low_db
                                                        : kSigmaLinearSnrLowDb;
  const double low_urad = g_linear_anchor_override.active ? g_linear_anchor_override.low_urad
                                                          : kSigmaLinearLowUrad;
  if (!(snr_db > 0.0)) {
    return high_urad * kUradToDeg;
  }
  double t = (high_db - snr_db) / (high_db - low_db);
  if (t < 0.0) {
    t = 0.0;
  }
  if (t > 1.0) {
    t = 1.0;
  }
  const double urad = high_urad + (low_urad - high_urad) * t;
  return urad * kUradToDeg;
}

// 过程：作用域锚点覆盖的构造/析构（进入设置、离开恢复），供 pipeline 在"大幅面预警扫描"
//       分支内使用；异常/提前 return 都会恢复，不污染宽场/窄场的默认口径。
LinearSigmaScope::LinearSigmaScope(bool enabled, double high_db, double high_urad, double low_db,
                                   double low_urad) {
  prev_active_ = g_linear_anchor_override.active;
  prev_high_db_ = g_linear_anchor_override.high_db;
  prev_high_urad_ = g_linear_anchor_override.high_urad;
  prev_low_db_ = g_linear_anchor_override.low_db;
  prev_low_urad_ = g_linear_anchor_override.low_urad;
  if (enabled) {
    g_linear_anchor_override.active = true;
    g_linear_anchor_override.high_db = high_db;
    g_linear_anchor_override.high_urad = high_urad;
    g_linear_anchor_override.low_db = low_db;
    g_linear_anchor_override.low_urad = low_urad;
  }
}

LinearSigmaScope::~LinearSigmaScope() {
  g_linear_anchor_override.active = prev_active_;
  g_linear_anchor_override.high_db = prev_high_db_;
  g_linear_anchor_override.high_urad = prev_high_urad_;
  g_linear_anchor_override.low_db = prev_low_db_;
  g_linear_anchor_override.low_urad = prev_low_urad_;
}

double ResolveFrameAngularSigmaDeg(const config::SbirsErrorModelConfig& model, double snr_linear,
                                   double ifov_rad) {
  // 中译：线性标定开关打开时**只**用 SNR 线性表（覆盖内部 RSS 推导），这正是甲方要的
  //   "直接挂钩"口径；关闭时回落到"仪器地板 ⊕ 焦面光子噪声"的物理模型。
  // 标识：线性标定优先（2026-11-04）。
  if (g_linear_snr_sigma_enabled) {
    const double snr_db = (snr_linear > 0.0) ? 10.0 * std::log10(snr_linear) : 0.0;
    return ResolveLinearSnrSigmaDeg(snr_db);
  }

  // 过程：物理模型口径 —— 仪器地板与焦面光子噪声按平方和合成，光子项随 SNR 增大收敛到 0。
  const double floor_deg = ResolveEffectiveAngularSigmaDeg(model);
  // 中译：**焦面光子噪声项**（2026-11-04）——质心定位的标准极限是 IFOV/(k·√SNR)：
  //   同一像元内光子数越多，质心越准；弱目标自然变差。此前该项完全缺失，测角误差是
  //   常数，与 SNR 无关（实测相关系数 −0.007），"能否到 3 µrad"成了配置写死的巧合。
  // 标识：焦面光子噪声项（2026-11-04）。
  constexpr double gain = 1.0;  // 质心增益固定（不开放配置，见线性标定常量说明）
  if (!(snr_linear > 0.0) || !(ifov_rad > 0.0) || !(gain > 0.0)) {
    return floor_deg;
  }
  const double photon_rad = ifov_rad / (gain * std::sqrt(snr_linear));
  const double photon_deg = photon_rad * (180.0 / kPi);
  return std::sqrt(floor_deg * floor_deg + photon_deg * photon_deg);
}

// 过程：单帧路径 —— 随机项 N(0,σ²) 每轴独立抽样，折射/滞后作为确定性公共偏差叠加；
//       距离误差走乘性模型（与角度误差的加性模型分开）。
SbirsErrorBearing ApplyAngularErrorModel(const config::SbirsErrorModelConfig& model,
                                         SbirsRandomSource* random, float true_azimuth_deg,
                                         float true_elevation_deg, double true_range_m,
                                         float relative_angular_rate_deg_per_sec, double snr_linear,
                                         double ifov_rad) {
  SbirsErrorBearing bearing;
  // 物理分项按 RSS 合成为单一 1-σ；az/el 独立采样，与对角量测协方差一致。
  const double angular_sigma_deg = ResolveFrameAngularSigmaDeg(model, snr_linear, ifov_rad);
  const double azimuth_random_error =
      angular_sigma_deg > 0.0 && random != nullptr
          ? angular_sigma_deg * random->NextStandardNormal()
          : 0.0;
  const double elevation_random_error =
      angular_sigma_deg > 0.0 && random != nullptr
          ? angular_sigma_deg * random->NextStandardNormal()
          : 0.0;

  const double refraction = RefractionErrorDeg(true_range_m, true_elevation_deg);
  const double lag =
      DynamicLagErrorDeg(relative_angular_rate_deg_per_sec, model.detector_bandwidth_hz);

  // 加法合成（design 2.10 式 1111-1113）。
  bearing.azimuth_deg =
      static_cast<float>(static_cast<double>(true_azimuth_deg) + azimuth_random_error +
                         refraction + lag);
  bearing.elevation_deg =
      static_cast<float>(static_cast<double>(true_elevation_deg) + elevation_random_error +
                         refraction + lag);

  // 乘法合成（距离误差）：d_meas = d_true · (1 + Δd_rand)。
  double range_factor = 0.0;
  if (model.range_fraction_sigma > 0.0f && random != nullptr) {
    range_factor = model.range_fraction_sigma * random->NextStandardNormal();
  }
  bearing.range_m = true_range_m * (1.0 + range_factor);
  return bearing;
}

namespace {

/// 角度差折叠到 (−180°, 180°]（方位均值走最短角差，防 0°/360° 环绕错均值）。
double Wrap180Deg(double delta_deg) {
  double wrapped = std::fmod(delta_deg + 180.0, 360.0);
  if (wrapped < 0.0) {
    wrapped += 360.0;
  }
  return wrapped - 180.0;
}

}  // namespace

// 过程：融合路径 —— N 帧各抽一次随机角误差后取均值：随机项按 1/√N 收敛，折射/滞后只加一次
//       （它们是同一几何下的系统偏差，不随帧数抵消）。
SbirsFusedBearingResult ApplyAngularErrorModelFused(
    const config::SbirsErrorModelConfig& model, SbirsRandomSource* random, float true_azimuth_deg,
    float true_elevation_deg, double true_range_m, float relative_angular_rate_deg_per_sec,
    int frame_count) {
  SbirsFusedBearingResult result;
  const double sigma_deg = ResolveEffectiveAngularSigmaDeg(model);
  result.frame_sigma_deg = sigma_deg;
  if (frame_count <= 1 || sigma_deg <= 0.0 || random == nullptr) {
    // 单帧/无随机分量：与既有单帧路径逐位一致（确定性偏差照常施加）。
    result.bearing = ApplyAngularErrorModel(model, random, true_azimuth_deg, true_elevation_deg,
                                            true_range_m, relative_angular_rate_deg_per_sec);
    result.fused_sigma_deg = sigma_deg;
    return result;
  }
  // N 帧独立抽样：方位以首帧为基准走最短角差累加，俯仰算术均值；
  // 折射/滞后为确定性公共偏差，叠加一次；距离误差保持单帧口径。
  double azimuth_delta_sum = 0.0;
  double elevation_sum = 0.0;
  for (int frame = 0; frame < frame_count; ++frame) {
    const double az_error = sigma_deg * random->NextStandardNormal();
    const double el_error = sigma_deg * random->NextStandardNormal();
    azimuth_delta_sum += Wrap180Deg(az_error);
    elevation_sum += el_error;
  }
  const double refraction = RefractionErrorDeg(true_range_m, true_elevation_deg);
  const double lag = DynamicLagErrorDeg(relative_angular_rate_deg_per_sec, model.detector_bandwidth_hz);
  result.bearing.azimuth_deg = static_cast<float>(
      static_cast<double>(true_azimuth_deg) + azimuth_delta_sum / frame_count + refraction + lag);
  result.bearing.elevation_deg = static_cast<float>(
      static_cast<double>(true_elevation_deg) + elevation_sum / frame_count + refraction + lag);
  double range_factor = 0.0;
  if (model.range_fraction_sigma > 0.0f && random != nullptr) {
    range_factor = model.range_fraction_sigma * random->NextStandardNormal();
  }
  result.bearing.range_m = true_range_m * (1.0 + range_factor);
  // 过程：融合后 1-σ = 单帧 σ/√N —— 供第 77 项日志把单帧 σ 与融合 σ 并列比较。
  result.fused_sigma_deg = sigma_deg / std::sqrt(static_cast<double>(frame_count));
  return result;
}

}  // namespace foundation
}  // namespace sbirs_sensor
