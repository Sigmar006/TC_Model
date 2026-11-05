/**
 * @file FusionAcceptanceRecords.cpp
 * @brief 融合验收行：多传感器跟踪 / 接力可写子集 / 协同融合 / UKF。
 */

#include "fusion/FusionAcceptanceRecords.h"

#include <cstdlib>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <utility>

#include "1q/coordinate/position_transform.h"
#include "common/logging/AcceptanceText.h"
#include "common/numerics/Constants.h"
#include "fusion/FusionAcceptanceLog.h"
#include "sbirs_sensor/tracking/SbirsAngleCvKalman.h"

namespace fusion {
namespace {

using oneq::logging::AppendField;
using oneq::logging::FormatF;
using oneq::logging::FormatCov6x6;
using oneq::logging::FormatCovDiag6;
using oneq::logging::FormatVec3;
using oneq::logging::CovarianceTrace6;

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
 * @brief （航迹键, 源）视线角采样状态（会话内累计，仿 LastVelocities 先例）。
 * @note 2026-08-22 甲方批注：剩余覆盖时间以「(视场宽度 − 已扫过角) ÷ 滑窗
 *       最小二乘角速率」的简易外推估计（自该源首见该航迹起累计扫过角）；
 *       库内无接力协议。
 */
struct RelaySight {
  double az_deg{0.0};        /**< 最近一次方位角（deg，原始缠绕值）。 */
  double el_deg{0.0};        /**< 最近一次俯仰角（deg）。 */
  std::uint64_t cycle{0U};   /**< 最近采样周期号。 */
  double unwrapped_az_deg{0.0}; /**< 自首见累计解缠方位（deg，扫过角分子用）。 */
  double first_el_deg{0.0};  /**< 首见俯仰角（扫过角俯仰分量锚点）。 */
  std::vector<RelayAngularSample> window; /**< 最近采样滑窗（最小二乘角速率用）。 */
  bool valid{false};         /**< 已收到首个方位样本。 */
};

/** @brief 接力角速率滑窗长度（逐拍差分噪声均摊；方法常数见目录条目）。 */
constexpr std::size_t kRelayWindowSamples = 8U;

std::map<std::pair<std::uint64_t, std::uint32_t>, RelaySight>& RelaySights() {
  static std::map<std::pair<std::uint64_t, std::uint32_t>, RelaySight> values;
  return values;
}

/** @brief (航迹键, 源) 角度域标准 KF 滤波状态（会话内累计，第18项验收行）。 */
struct AngleKfState {
  sbirs_sensor::tracking::SbirsAngleCvGaussianState state{};
  bool valid{false};
};

std::map<std::pair<std::uint64_t, std::uint32_t>, AngleKfState>& AngleKfStates() {
  static std::map<std::pair<std::uint64_t, std::uint32_t>, AngleKfState> values;
  return values;
}

/** @brief 会话已知源集合（跨航迹累计）：单星接力星候选全集（库内无轨道/视场模型）。 */
std::set<std::uint32_t>& KnownSources() {
  static std::set<std::uint32_t> values;
  return values;
}

// 优先输出输入携带的业务名称；空名称兼容输出「实体<source_id>」。
std::string SourceLabel(std::uint32_t source_id, const std::string& source_name) {
  if (!source_name.empty()) {
    return source_name;
  }
  return "实体" + std::to_string(source_id);
}

std::string ChannelNames(const FusedTarget& track) {
  std::set<std::pair<std::uint32_t, std::string>> sources;
  for (const ChannelMeasurement& channel : track.channels) {
    sources.insert(std::make_pair(channel.source_id, channel.source_name));
  }
  std::string text;
  for (const auto& source : sources) {
    if (!text.empty()) {
      text += "+";
    }
    text += SourceLabel(source.first, source.second);
  }
  return text;
}

std::string ChannelBearings(const FusedTarget& track) {
  std::string text;
  for (const ChannelMeasurement& channel : track.channels) {
    if (!channel.has_bearing) {
      continue;
    }
    if (!text.empty()) {
      text += ";";
    }
    text += SourceLabel(channel.source_id, channel.source_name) + ":(" +
            FormatF(channel.bearing_az_deg, 6) + "," +
            FormatF(channel.bearing_el_deg, 6) + ")deg";
  }
  return text;
}

}  // namespace

double WrappedAzimuthDeltaDeg(double az_from_deg, double az_to_deg) {
  double delta = std::fmod(az_to_deg - az_from_deg, 360.0);
  if (delta <= -180.0) {
    delta += 360.0;
  } else if (delta > 180.0) {
    delta -= 360.0;
  }
  return delta;
}

double LeastSquaresAngularSpeedDegPerSec(const std::vector<RelayAngularSample>& samples) {
  // 需 ≥2 个不同时刻采样；时间跨度为零（同拍重复）不可估。
  if (samples.size() < 2U) {
    return -1.0;
  }
  const double t0 = samples.front().time_sec;
  double sum_t = 0.0;
  double sum_tt = 0.0;
  double sum_az = 0.0;
  double sum_el = 0.0;
  double sum_t_az = 0.0;
  double sum_t_el = 0.0;
  double unwrapped_az = 0.0;
  double prev_az = samples.front().az_deg;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    const double t = samples[i].time_sec - t0;
    // 逐差分解缠：跨 ±180 缠绕不产生 360° 假差分。
    unwrapped_az += WrappedAzimuthDeltaDeg(prev_az, samples[i].az_deg);
    prev_az = samples[i].az_deg;
    sum_t += t;
    sum_tt += t * t;
    sum_az += unwrapped_az;
    sum_el += samples[i].el_deg;
    sum_t_az += t * unwrapped_az;
    sum_t_el += t * samples[i].el_deg;
  }
  const double n = static_cast<double>(samples.size());
  const double det = n * sum_tt - sum_t * sum_t;
  if (det <= 0.0) {
    return -1.0;  // 全部同拍（时间跨度为零）。
  }
  const double slope_az = (n * sum_t_az - sum_t * sum_az) / det;
  const double slope_el = (n * sum_t_el - sum_t * sum_el) / det;
  return std::sqrt(slope_az * slope_az + slope_el * slope_el);
}

double RelayRemainingCoverageSec(double fov_width_deg, double swept_deg,
                                 double omega_deg_per_s) {
  if (fov_width_deg <= 0.0 || omega_deg_per_s <= 0.0 || swept_deg < 0.0) {
    return -1.0;
  }
  const double remaining = (fov_width_deg - swept_deg) / omega_deg_per_s;
  return remaining > 0.0 ? remaining : 0.0;
}

void WriteFusionAcceptance(std::uint32_t cycle, const std::vector<FusedTarget>& tracks,
                           bool filtering_enabled, const FusionConfig& config) {
  if (!FUSION_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 行前缀仿真时间 = 周期 × 周期时长（此前隐含 dt=1s，按配置单源修正）。
  const float sim_time =
      static_cast<float>(static_cast<double>(cycle) * config.track_cycle_period_sec);
  const std::uint64_t cycle_u64 = static_cast<std::uint64_t>(cycle);
  if (tracks.empty()) {
    return;
  }
  // 会话已知源注册（验收判定标准 第16项·修改：单星跟踪也写接力计划/交接指令，
  // 接力星自会话源全集选取——本星离场后星座内其他源即候选）。
  for (const FusedTarget& track : tracks) {
    for (const ChannelMeasurement& channel : track.channels) {
      KnownSources().insert(channel.source_id);
    }
  }

  for (const FusedTarget& track : tracks) {
    oneq::coordinate::EcefPositionM ecef{};
    const bool have_ecef = track.has_kinematic_estimate &&
                           oneq::coordinate::TryLlaToEcef(track.kinematic_estimate.position, &ecef);
    // 中译：**补 `目标ID=`**（2026-11-02 甲方要求）——判据示例是
    //   `航迹键=1 目标ID=1001 …`，而此前只有 `航迹键`；外部要按目标聚合验收数据时，
    //   没有目标 ID 就无法与场景/其它项对齐。身份保留时 `key` 即场景目标 ID；
    //   上游身份丢失的航迹由引擎合成键，此时两字段同值（如实反映"只有库内键"）。
    // 标识：第15项目标ID（2026-11-02）。
    std::string content = "航迹键=" + std::to_string(track.key);
    content += " 目标ID=" + std::to_string(track.key);
    content += " 传感器信息源名称=[" + ChannelNames(track) + "]";
    content += " 信息源测角数据=[" + ChannelBearings(track) + "]";
    // 中译：**判据第15项外部导入合并判据要求写"测角信息融合卫星地面站名称"**
    //   （2026-11-02）——否则外部无法区分同一目标在不同地面站得到的融合结果。
    // 标识：第15项融合地面站名称（2026-11-02）。
    if (!config.fusion_station_name.empty()) {
      content += " 测角信息融合卫星地面站名称=" + config.fusion_station_name;
    }
    // 中译：**单通道（单星）航迹不输出位置**（2026-11-04，甲方"不输出"）——单星只测向，
    //   位置来自保活/单星视线外推，不是交会解；双星及以上才写 ECEF 位置。
    // 标识：第15项单星不发位置（2026-11-04）。
    std::size_t active_channels_15 = 0U;
    for (const ChannelMeasurement& channel : track.channels) {
      if (channel.sample_count > 0U) {
        ++active_channels_15;
      }
    }
    const bool position_available = active_channels_15 >= 2U;
    if (have_ecef && position_available) {
      content += " ECEF位置m=" + FormatVec3(ecef.x_m, ecef.y_m, ecef.z_m, 1);
    }
    if (track.has_kinematic_estimate) {
      content += " 协方差对角=" + FormatCovDiag6(track.kinematic_estimate.covariance_ecef);
      // 中译：**融合位置精度**（2026-11-02）——协方差对角虽然完整，但读不出一句
      //   "这条航迹现在准到几米"。这里由位置块/速度块对角开方给出 3D 1σ（RSS）：
      //   协方差排布 [x,vx,y,vy,z,vz]，位置轴 0/2/4、速度轴 1/3/5；方差取 max(0)
      //   再开方，避免滤波数值噪声产生的极小负值变成 NaN。
      // 标识：融合位置精度输出（2026-11-02）。
      const std::array<double, 36U>& cov = track.kinematic_estimate.covariance_ecef;
      const double var_x = std::max(0.0, cov[0]);
      const double var_y = std::max(0.0, cov[14]);
      const double var_z = std::max(0.0, cov[28]);
      const double var_vx = std::max(0.0, cov[7]);
      const double var_vy = std::max(0.0, cov[21]);
      const double var_vz = std::max(0.0, cov[35]);
      if (position_available) {
        content += " 位置精度1σ=" + FormatF(std::sqrt(var_x + var_y + var_z), 1) + "m";
        content += " 位置精度轴向1σ=(" + FormatF(std::sqrt(var_x), 1) + "," +
                   FormatF(std::sqrt(var_y), 1) + "," + FormatF(std::sqrt(var_z), 1) + ")m";
      }
      content += " 速度精度1σ=" + FormatF(std::sqrt(var_vx + var_vy + var_vz), 3) + "m/s";
    }
    // 过程：**大幅面预警扫描模式下静默本行**（2026-11-05 甲方口径：大幅面只出大幅面）。
    //   融合模块与红外传感器分属不同 DLL（DataProcessors / Sensors），静态标志不共享，
    //   故用进程级环境变量做一次性模式通告：Sensors 侧在大幅面启用时置
    //   `ONEQ_SBIRS_WIDE_AREA=1`，本处按其值跳过"宽窄联合"行。
    //   未设置时行为不变（联合模式/宽扫模式照常输出）。
    // 标识：融合侧联合行随红外模式静默（2026-11-05）。
    const char* wide_area_env = std::getenv("ONEQ_SBIRS_WIDE_AREA");
    const bool wide_area_mode = (wide_area_env != nullptr && wide_area_env[0] == '1');
    if (!wide_area_mode) {
      FUSION_ACCEPTANCE_ITEM(sim_time, cycle, "宽窄视场联合探测功能测试", content);
    }
    FUSION_ACCEPTANCE_ITEM(sim_time, cycle, "多传感器目标跟踪功能测试", content);
    // 中译：**判据示例第二行：逐源测角 + 红外可探测**（2026-11-02）——判据示例是
    //   `相对卫星ID=4 目标ID=1001 目标方位/俯仰(ECI)=(…)rad 红外可探测=是`；
    //   本行按"每个源一行"写，`相对卫星ID` 取该源的 source_id，方位/俯仰由度转弧度
    //   （融合层收到的就是 ECI 测角）。`红外可探测` 以"该源本周期是否有测角"为判据：
    //   有测角即有红外探测，无测角（仅位置量测/失跟）记"否"。
    // 标识：第15项逐源测角与红外可探测（2026-11-02）。
    for (const ChannelMeasurement& channel : track.channels) {
      std::string per_source = "相对卫星ID=" + std::to_string(channel.source_id);
      per_source += " 目标ID=" + std::to_string(track.key);
      if (channel.has_bearing) {
        per_source += " 目标方位/俯仰(ECI)=(" +
                      FormatF(oneq::common::numerics::DegToRad(channel.bearing_az_deg), 6) +
                      "," + FormatF(oneq::common::numerics::DegToRad(channel.bearing_el_deg), 6) +
                      ")rad";
      }
      per_source += channel.has_bearing ? " 红外可探测=是" : " 红外可探测=否";
      FUSION_ACCEPTANCE_ITEM(sim_time, cycle, "多传感器目标跟踪功能测试", per_source);
    }

    const auto& vel = track.kinematic_estimate.velocity_ecef_m_per_s;
    LastVel& prev = LastVelocities()[track.key];
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    if (prev.valid) {
      ax = vel[0] - prev.vx;
      ay = vel[1] - prev.vy;
      az = vel[2] - prev.vz;
    }
    const double speed = std::sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]);
    const double acc = std::sqrt(ax * ax + ay * ay + az * az);
    prev.vx = vel[0];
    prev.vy = vel[1];
    prev.vz = vel[2];
    prev.valid = track.has_kinematic_estimate;

    std::string relay = "目标键=" + std::to_string(track.key);
    // 验收判定标准 第16项（修改）：位置/速度/加速度即融合跟踪结果——仅当本航迹
    // 确有多源融合（滑窗内 ≥2 个通道有量测）时写「多源数据融合后的目标跟踪结果=」，
    // 单星跟踪不写该字段（红外单星只测向，运动学估计不构成多源融合结果）。
    std::size_t active_channels = 0U;
    for (const ChannelMeasurement& channel : track.channels) {
      if (channel.sample_count > 0U) {
        ++active_channels;
      }
    }
    if (active_channels >= 2U && track.has_kinematic_estimate) {
      relay += " 多源数据融合后的目标跟踪结果=位置LLA=(";
      relay += FormatF(track.kinematic_estimate.position.latitude_deg, 6) + ",";
      relay += FormatF(track.kinematic_estimate.position.longitude_deg, 6) + ",";
      relay += FormatF(track.kinematic_estimate.position.altitude_m, 1) + ")";
      relay += " 速度模=" + FormatF(speed, 3) + "m/s";
      relay += " 加速度模=" + FormatF(acc, 3) + "m/s^2";
    }
    // 中译：**单星跟踪不写"位置/速度/加速度"整组**（2026-11-04，判据第16项 L712）——
    //   判据正文明确："**仅当本航迹有多源融合（≥2 个通道）时**写
    //   `多源数据融合后的目标跟踪结果=`（位置 LLA、速度模、加速度模）；单星跟踪省略该字段"。
    //   位置/速度/加速度三件套整体属于该字段，故单星（单通道）周期一个都不写；
    //   接力计划、交接指令、预测目标运动轨迹仍照写（判据同句要求"无论单星还是多源都写"）。
    // 标识：第16项单星省略融合跟踪结果（2026-11-04）。
    // 中译：**预测目标运动轨迹**（2026-11-02）——判据第16项正文要求"预测目标的运动轨迹"，
    //   示例为 `预测目标运动轨迹=[+1s:(lat,lon,h);+10s:(…)]`。融合层是 CV 模型（无加速度
    //   状态），按**匀速外推**给 +1/+5/+10 s 三点：`p(t) = p0 + v·t`（ECEF），再转 LLA。
    //   该轨迹是"当前状态的可外推段"，不含重力/阻力，故只给 10 s 内三点，不外推到落点。
    // 标识：第16项预测目标运动轨迹（2026-11-02）。
    if (track.has_kinematic_estimate) {
      oneq::coordinate::EcefPositionM base_ecef{};
      if (oneq::coordinate::TryLlaToEcef(track.kinematic_estimate.position, &base_ecef)) {
        std::string trajectory = " 预测目标运动轨迹=[";
        const double offsets[3] = {1.0, 5.0, 10.0};
        for (std::size_t k = 0U; k < 3U; ++k) {
          oneq::coordinate::EcefPositionM point = base_ecef;
          point.x_m += vel[0] * offsets[k];
          point.y_m += vel[1] * offsets[k];
          point.z_m += vel[2] * offsets[k];
          oneq::coordinate::LlaPositionDegM point_lla{};
          if (!oneq::coordinate::TryEcefToLla(point, &point_lla)) {
            continue;
          }
          if (k > 0U && trajectory.size() > std::string(" 预测目标运动轨迹=[").size()) {
            trajectory += ";";
          }
          trajectory += "+" + std::to_string(static_cast<int>(offsets[k])) + "s:(" +
                        FormatF(point_lla.latitude_deg, 6) + "," +
                        FormatF(point_lla.longitude_deg, 6) + "," +
                        FormatF(point_lla.altitude_m, 1) + ")";
        }
        trajectory += "]";
        relay += trajectory;
      }
    }

    // 接力三项（2026-08-22 甲方批注）：逐方位源以「(视场宽度 − 自首见累计扫过角)
    // ÷ 滑窗最小二乘角速率」外推剩余覆盖；取最早离开的源为接力对象。
    const double t_sec =
        static_cast<double>(cycle_u64) * config.track_cycle_period_sec;
    std::uint32_t exit_source = 0U;
    double min_remaining_sec = -1.0;
    for (const ChannelMeasurement& channel : track.channels) {
      if (!channel.has_bearing) {
        continue;
      }
      RelaySight& sight = RelaySights()[{track.key, channel.source_id}];
      double remaining = -1.0;
      if (sight.valid && cycle_u64 > sight.cycle) {
        // 自首见累计解缠扫过角：方位逐差分归一（跨 ±180 不假跳 360°），俯仰直线差。
        sight.unwrapped_az_deg +=
            WrappedAzimuthDeltaDeg(sight.az_deg, channel.bearing_az_deg);
        const double swept_el_deg = channel.bearing_el_deg - sight.first_el_deg;
        const double swept_deg = std::sqrt(sight.unwrapped_az_deg * sight.unwrapped_az_deg +
                                           swept_el_deg * swept_el_deg);
        sight.window.push_back({t_sec, channel.bearing_az_deg, channel.bearing_el_deg});
        if (sight.window.size() > kRelayWindowSamples) {
          sight.window.erase(sight.window.begin());
        }
        const double omega_deg_per_s = LeastSquaresAngularSpeedDegPerSec(sight.window);
        remaining =
            RelayRemainingCoverageSec(config.relay_fov_width_deg, swept_deg, omega_deg_per_s);
      } else if (!sight.valid) {
        sight.first_el_deg = channel.bearing_el_deg;
        sight.unwrapped_az_deg = 0.0;
        sight.window.push_back({t_sec, channel.bearing_az_deg, channel.bearing_el_deg});
      }
      if (remaining >= 0.0 && (min_remaining_sec < 0.0 || remaining < min_remaining_sec)) {
        min_remaining_sec = remaining;
        exit_source = channel.source_id;
      }
      sight.az_deg = channel.bearing_az_deg;
      sight.el_deg = channel.bearing_el_deg;
      sight.cycle = cycle_u64;
      sight.valid = true;
    }

    // 交接对象（验收判定标准 第16项·修改）：优先同航迹当前仍有量测的其他源；
    // 单星跟踪时自会话已知源全集选取——先取曾测过本航迹的源（接力链成员），
    // 否则取编号最小的其他源。库内无轨道/视场模型，不做视场归属几何判定。
    std::uint32_t takeover_source = 0U;
    std::size_t takeover_samples = 0U;
    if (exit_source != 0U) {
      for (const ChannelMeasurement& channel : track.channels) {
        if (channel.source_id == exit_source || channel.sample_count == 0U) {
          continue;
        }
        if (channel.sample_count > takeover_samples) {
          takeover_samples = channel.sample_count;
          takeover_source = channel.source_id;
        }
      }
      if (takeover_source == 0U) {
        for (const std::uint32_t source : KnownSources()) {
          if (source == exit_source) {
            continue;
          }
          if (RelaySights().count({track.key, source}) != 0U) {
            takeover_source = source;
            break;
          }
          if (takeover_source == 0U) {
            takeover_source = source;
          }
        }
      }
    }

    if (exit_source != 0U) {
      relay += " 剩余覆盖时间=" + FormatF(min_remaining_sec, 1) + "s(卫星ID=" +
               std::to_string(exit_source) + ")";
      relay += " 接力计划=卫星ID=" + std::to_string(exit_source) + "预计" +
               FormatF(min_remaining_sec, 1) + "s离开视场";
      if (takeover_source != 0U) {
        relay += " 接力卫星ID=" + std::to_string(takeover_source);
        relay += " 交接指令=卫星ID=" + std::to_string(exit_source) + "(剩余" +
                 FormatF(min_remaining_sec, 1) + "s离场)→卫星ID=" +
                 std::to_string(takeover_source) + "接管";
      }
    }
    FUSION_ACCEPTANCE_ITEM(sim_time, cycle, "多传感器接力跟踪功能测试", relay);

    if (filtering_enabled && track.has_kinematic_estimate) {
      std::string ukf = "位置LLA=(";
      ukf += FormatF(track.kinematic_estimate.position.latitude_deg, 6) + ",";
      ukf += FormatF(track.kinematic_estimate.position.longitude_deg, 6) + ",";
      ukf += FormatF(track.kinematic_estimate.position.altitude_m, 1) + ")";
      if (have_ecef) {
        ukf += " ECEF位置m=" + FormatVec3(ecef.x_m, ecef.y_m, ecef.z_m, 1);
      }
      ukf += " 速度m/s=" + FormatVec3(vel[0], vel[1], vel[2], 3);
      ukf += " 加速度m/s^2=" + FormatVec3(ax, ay, az, 3);
      ukf += " 协方差迹=" + FormatF(CovarianceTrace6(track.kinematic_estimate.covariance_ecef), 2);
      ukf += " 完整协方差=" + FormatCov6x6(track.kinematic_estimate.covariance_ecef);
      FUSION_ACCEPTANCE_ITEM(sim_time, cycle, "UKF滤波", ukf);
    }
  }

  // 评审 2026-08-26 条5 + 2026-08-27 条1：协同融合信息逐目标一行（原单行只取首
  // 航迹，句柄整场恒定、无法区分目标）；删除滑窗字样，行内列出融合定位 LLA 与
  // 各源测向信息（方位/俯仰为该源最近一拍视线角）。
  for (const FusedTarget& track : tracks) {
    std::string collab = "目标键=" + std::to_string(track.key);
    AppendField(collab, "参与通道", ChannelNames(track));
    if (track.has_kinematic_estimate) {
      collab += " 定位LLA=(" + FormatF(track.kinematic_estimate.position.latitude_deg, 6) +
                "," + FormatF(track.kinematic_estimate.position.longitude_deg, 6) + "," +
                FormatF(track.kinematic_estimate.position.altitude_m, 1) + ")";
    }
    std::string bearings;
    for (const ChannelMeasurement& channel : track.channels) {
      if (!channel.has_bearing) {
        continue;
      }
      if (!bearings.empty()) {
        bearings += ";";
      }
      bearings += SourceLabel(channel.source_id, channel.source_name) + "(方位" +
                  FormatF(channel.bearing_az_deg, 3) + "°/俯仰" +
                  FormatF(channel.bearing_el_deg, 3) + "°)";
    }
    if (!bearings.empty()) {
      collab += " 测向信息=[" + bearings + "]";
    }
    collab += " 融合目标数=" + std::to_string(tracks.size());
    FUSION_ACCEPTANCE_ITEM(sim_time, cycle, "协同探测信息融合功能测试", collab);
  }

  // 标准卡尔曼滤波（验收判定标准 第18项）：对逐源视线角（方位/俯仰，源本地北东天
  // 参考的量测）做 4 维 [az, ω_az, el, ω_el] 线性标准 KF——估计器与 SBIRS 实验后端
  // kAngleCvKf 同口径（F=CV、Q=连续白噪声加速度离散化、R 取融合配置
  // default_bearing_noise_sigma_rad）；输出滤波后方位/俯仰及其变化率，逐周期一行
  // 构成连续时刻的平滑视线估计。
  for (const FusedTarget& track : tracks) {
    for (const ChannelMeasurement& channel : track.channels) {
      if (!channel.has_bearing) {
        continue;
      }
      constexpr double kDegToRad = 0.017453292519943295;
      const auto channel_key = std::make_pair(track.key, channel.source_id);
      AngleKfState& kf = AngleKfStates()[channel_key];
      const double dt_sec = config.track_cycle_period_sec;
      if (!kf.valid) {
        kf.state = sbirs_sensor::tracking::MakeInitialAngleCvState(
            static_cast<float>(channel.bearing_az_deg * kDegToRad),
            static_cast<float>(channel.bearing_el_deg * kDegToRad));
        kf.valid = true;
      } else {
        const sbirs_sensor::tracking::SbirsAngleCvPredictor predictor;
        const sbirs_sensor::tracking::SbirsAngleCvGaussianState predicted =
            predictor.Predict(kf.state, static_cast<float>(dt_sec));
        sbirs_sensor::tracking::SbirsAngleCvGaussianState::MeasurementVector measurement;
        measurement(0) = static_cast<float>(channel.bearing_az_deg * kDegToRad);
        measurement(1) = static_cast<float>(channel.bearing_el_deg * kDegToRad);
        const sbirs_sensor::tracking::SbirsAngleCvGaussianState::MeasurementCovariance noise =
            sbirs_sensor::tracking::SbirsAngleCvGaussianState::MeasurementCovariance::Identity() *
            static_cast<float>(config.default_bearing_noise_sigma_rad *
                               config.default_bearing_noise_sigma_rad);
        const sbirs_sensor::tracking::SbirsAngleCvUpdater updater;
        kf.state = updater.Update(predicted, measurement, noise).posterior;
      }
      // 中译：**补 `目标ID=`**（2026-11-02）——判据示例带目标 ID，此前只有航迹键，
      //   外部无法把卡尔曼滤波行与场景目标对齐。
      // 标识：第18项目标ID（2026-11-02）。
      std::string std_kf = "目标键=" + std::to_string(track.key);
      std_kf += " 目标ID=" + std::to_string(track.key);
      std_kf += " 源=" + SourceLabel(channel.source_id, channel.source_name);
      std_kf += " 滤波方位/俯仰=(" + FormatF(kf.state.mean(0), 6) + "," +
                FormatF(kf.state.mean(2), 6) + ")rad";
      std_kf += " 方位变化率=" + FormatF(kf.state.mean(1), 8) + "rad/s";
      std_kf += " 俯仰变化率=" + FormatF(kf.state.mean(3), 8) + "rad/s";
      FUSION_ACCEPTANCE_ITEM(sim_time, cycle, "标准卡尔曼滤波功能测试", std_kf);
    }
  }
}

}  // namespace fusion
