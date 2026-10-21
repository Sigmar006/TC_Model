/**
 * @file SbirsAcceptanceRecords.cpp
 * @brief SBIRS 验收行：安装矩阵派生、轨道角抽样累计、生命周期与计时暂无项。
 */

#include "sbirs_sensor/pipeline/SbirsAcceptanceRecords.h"

#include <cmath>
#include <array>
#include <cstdint>
#include <iterator>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/inertial_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "common/logging/AcceptanceText.h"
#include "sbirs_sensor/pipeline/SbirsAcceptanceLog.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

using oneq::logging::FormatF;
using oneq::logging::FormatPairDeg;
using oneq::logging::FormatVec3;
using oneq::logging::YesNo;

std::string FormatRotation(const oneq::coordinate::RotationMatrix3d& r) {
  return "[[" + FormatF(r.m00, 6) + "," + FormatF(r.m01, 6) + "," + FormatF(r.m02, 6) + "];[" +
         FormatF(r.m10, 6) + "," + FormatF(r.m11, 6) + "," + FormatF(r.m12, 6) + "];[" +
         FormatF(r.m20, 6) + "," + FormatF(r.m21, 6) + "," + FormatF(r.m22, 6) + "]]";
}

oneq::coordinate::RotationMatrix3d ComposeMountAndMisalignment(
    const oneq::coordinate::EulerAnglesDeg& mount, const oneq::coordinate::EulerAnglesDeg& misalign) {
  return oneq::coordinate::Compose(oneq::coordinate::BuildRotationMatrix(mount),
                                   oneq::coordinate::Inverse(oneq::coordinate::BuildRotationMatrix(misalign)));
}

/** @brief 第7项可探测性行待定条目（单星候选顺延至确定无第二星再落盘）。 */
struct DetectabilityEntry {
  std::uint32_t satellite_id{0U};
  float sim_time_sec{0.0f};
  std::uint32_t cycle{0U};
  double slant_range_m{0.0};
  double measured_az_rad{0.0};
  double measured_el_rad{0.0};
  bool narrow_field{false};
  std::array<double, 3> satellite_ecef_m = std::array<double, 3>();
  double gmst_rad{0.0};
};

// 按输出目录隔离待定表；同目录内按周期 → 目标ID → 卫星ID 会合。
std::map<std::uint32_t, std::map<std::uint64_t, std::map<std::uint32_t, DetectabilityEntry>>>&
DetectabilityPending() {
  static std::map<std::string, std::map<std::uint32_t,
                  std::map<std::uint64_t, std::map<std::uint32_t, DetectabilityEntry>>>>
      pending;
  return pending[oneq::logging::CurrentAcceptanceLogOutputDirectory()];
}

// 待定表为进程级共享（双星定位依赖两星写同一张表会合），组件层当前串行步进；
// 此锁防集成方同进程多线程跑多个会话时并发增删坏表。锁内落盘会经
// WriteAcceptanceLog 取日志通道自身的锁（该路径无回调、不重入本文件），
// 锁序恒为 待定表锁→日志锁，无反向路径。
std::mutex& DetectabilityPendingMutex() {
  static std::mutex mutex;
  return mutex;
}

double HashUnit(std::uint32_t cycle, std::uint32_t salt) {
  std::uint32_t x = cycle * 747796405U + salt * 2891336453U;
  x = ((x >> ((x >> 28U) + 4U)) ^ x) * 277803737U;
  x = (x >> 22U) ^ x;
  return static_cast<double>(x) / 4294967295.0;
}

double SampleNormal(std::uint32_t cycle, std::uint32_t salt) {
  const double u1 = std::max(1.0e-12, HashUnit(cycle, salt));
  const double u2 = HashUnit(cycle, salt + 1U);
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(6.283185307179586 * u2);
}

const char* EventName(session::SbirsDetectionLifecycleEventKind kind) {
  switch (kind) {
    case session::SbirsDetectionLifecycleEventKind::kFirstDetected:
      return "首次探测";
    case session::SbirsDetectionLifecycleEventKind::kCoasting:
      return "跟踪中断(滑行)";
    case session::SbirsDetectionLifecycleEventKind::kLost:
      return "跟踪中断";
    default:
      return nullptr;
  }
}

int EventLevel(session::SbirsDetectionLifecycleEventKind kind) {
  switch (kind) {
    case session::SbirsDetectionLifecycleEventKind::kFirstDetected:
      return 1;
    case session::SbirsDetectionLifecycleEventKind::kCoasting:
    case session::SbirsDetectionLifecycleEventKind::kLost:
      return 2;
    default:
      return 3;
  }
}

const char* StageName(const session::SbirsDetectionLifecycleEvent& event) {
  switch (event.observation_stage) {
    case output::SbirsObservationStage::kWideFieldSearch:
      return "宽视场搜索";
    case output::SbirsObservationStage::kNarrowFieldAcquisition:
      return "窄视场捕获";
    case output::SbirsObservationStage::kNarrowFieldTrack:
      return "窄视场跟踪";
    default:
      return "未知阶段";
  }
}

}  // namespace

void WriteSbirsInstallMatrices(std::uint32_t satellite_id,
                               const oneq::coordinate::EulerAnglesDeg& mount_deg,
                               const oneq::coordinate::EulerAnglesDeg& misalignment_deg) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const oneq::coordinate::RotationMatrix3d r_mount = oneq::coordinate::BuildRotationMatrix(mount_deg);
  const oneq::coordinate::RotationMatrix3d r_mis =
      oneq::coordinate::BuildRotationMatrix(misalignment_deg);
  const oneq::coordinate::RotationMatrix3d r_actual =
      ComposeMountAndMisalignment(mount_deg, misalignment_deg);
  // 规范口径（验收判定标准 第10项）：只写理想指向R/实际指向R/安装误差矩阵R + 卫星ID，
  // 不输出失准角欧拉角字段。
  std::string content = "卫星ID=" + std::to_string(satellite_id);
  content += " 理想指向R=" + FormatRotation(r_mount);
  content += " 实际指向R=" + FormatRotation(r_actual);
  content += " 安装误差矩阵R=" + FormatRotation(r_mis);
  SBIRS_ACCEPTANCE_ITEM(0.0f, 0U, "安装矩阵误差功能测试", content);
}

void WriteSbirsOrbitSample(std::uint32_t satellite_id, float sim_time_sec, std::uint32_t cycle,
                           float orbit_sigma_deg, double reference_range_m,
                           float nav_position_sigma_m,
                           const session::SbirsVector3M& satellite_ecef,
                           const session::SbirsVector3M& satellite_velocity_ecef) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const double sigma = static_cast<double>(orbit_sigma_deg);
  const double nav_sigma = static_cast<double>(nav_position_sigma_m);
  // 卫星位置混入盐值：双星同进程同周期时两星抽样相互独立（否则逐拍同值）。
  const std::uint32_t sat_salt =
      static_cast<std::uint32_t>(std::fabs(satellite_ecef.x) * 1.0e-3) * 31U +
      static_cast<std::uint32_t>(std::fabs(satellite_ecef.y) * 1.0e-3) * 17U +
      static_cast<std::uint32_t>(std::fabs(satellite_ecef.z) * 1.0e-3);
  const double az = sigma * SampleNormal(cycle, 11U + sat_salt);
  const double el = sigma * SampleNormal(cycle, 29U + sat_salt);
  const double range = reference_range_m > 0.0 ? reference_range_m : 40000000.0;

  // 定位误差抽样口径（评审 2026-08-26 条1 + 2026-08-27 条4）：东/北 = 轨道角抽样
  // 弧长映射（轨道角 σ 为 0 的场景仍有径向分量），天 = 径向按导航 σ 独立抽样；
  // 真值（传入 ECEF）不可用时退化为按导航 σ 三维随机抽样（分量记 X/Y/Z）。
  const double r_norm = std::sqrt(satellite_ecef.x * satellite_ecef.x +
                                  satellite_ecef.y * satellite_ecef.y +
                                  satellite_ecef.z * satellite_ecef.z);
  const double radial = nav_sigma * SampleNormal(cycle, 47U + sat_salt);
  const bool have_truth_ecef = r_norm > 0.0;
  double dx;
  double dy;
  double dz;
  if (have_truth_ecef) {
    const double kDegToRad = 0.017453292519943295;
    dx = az * kDegToRad * range;
    dy = el * kDegToRad * range;
    dz = radial;
  } else {
    dx = nav_sigma * SampleNormal(cycle, 53U + sat_salt);
    dy = nav_sigma * SampleNormal(cycle, 59U + sat_salt);
    dz = radial;
  }
  const double err_norm = std::sqrt(dx * dx + dy * dy + dz * dz);

  // 规范口径（验收判定标准 第8项）：真值/实际位置ECEF、定位误差ECEF、模长，以及
  // 真值/实际速度ECEF 与速度误差ECEF + 卫星ID；不输出东/北/天统计、会话 n、均值/RMS、
  // 分量 σ 与口径说明。
  // 速度误差：与位置误差同源抽样（同一 σ 口径），仅作模型动态参数输出，不参与位置阈值判定。
  const double vel_sigma = nav_sigma * 0.01;  // 速度误差量级：导航 σ 的 1%（m/s）
  const double dvx = vel_sigma * SampleNormal(cycle, 67U + sat_salt);
  const double dvy = vel_sigma * SampleNormal(cycle, 71U + sat_salt);
  const double dvz = vel_sigma * SampleNormal(cycle, 73U + sat_salt);
  std::string nav = "卫星ID=" + std::to_string(satellite_id);
  if (have_truth_ecef) {
    // ECEF 误差向量 = ENU 分量经本地东北天基旋转合成；实际 ECEF = 真值 + 误差向量。
    const double up_x = satellite_ecef.x / r_norm;
    const double up_y = satellite_ecef.y / r_norm;
    const double up_z = satellite_ecef.z / r_norm;
    // 东向单位矢 = ẑ×r̂ = (-ry, rx, 0)/|·|（极轨卫星不在极点，叉积非退化；退化时
    // 回退到经度 0 的本地东北天基）；北向单位矢 = r̂×east。
    const double east_norm = std::sqrt(satellite_ecef.x * satellite_ecef.x +
                                       satellite_ecef.y * satellite_ecef.y);
    double east_x;
    double east_y;
    if (east_norm > 1.0e-6) {
      east_x = -satellite_ecef.y / east_norm;
      east_y = satellite_ecef.x / east_norm;
    } else {
      east_x = 0.0;
      east_y = 1.0;
    }
    const double north_x = -up_z * east_y;
    const double north_y = up_z * east_x;
    const double north_z = up_x * east_y - up_y * east_x;
    const double vec_x = dx * east_x + dy * north_x + dz * up_x;
    const double vec_y = dx * east_y + dy * north_y + dz * up_y;
    const double vec_z = dy * north_z + dz * up_z;
    nav += " 真值位置ECEF=";
    nav += FormatVec3(satellite_ecef.x, satellite_ecef.y, satellite_ecef.z, 1);
    nav += "m 实际位置ECEF=";
    nav += FormatVec3(satellite_ecef.x + vec_x, satellite_ecef.y + vec_y,
                      satellite_ecef.z + vec_z, 1);
    nav += "m 定位误差ECEF=(" + FormatF(vec_x, 1) + "," + FormatF(vec_y, 1) + "," +
           FormatF(vec_z, 1) + ")m";
    nav += " 真值速度ECEF=";
    nav += FormatVec3(satellite_velocity_ecef.x, satellite_velocity_ecef.y,
                      satellite_velocity_ecef.z, 1);
    nav += "m/s 实际速度ECEF=";
    nav += FormatVec3(satellite_velocity_ecef.x + dvx, satellite_velocity_ecef.y + dvy,
                      satellite_velocity_ecef.z + dvz, 1);
    nav += "m/s 速度误差ECEF=(" + FormatF(dvx, 3) + "," + FormatF(dvy, 3) + "," +
           FormatF(dvz, 3) + ")m/s";
  } else {
    // 传入 ECEF 不可用：位置字段如实记不可用，误差向量按导航 σ 三维抽样。
    nav += " 真值位置ECEF=不可用 实际位置ECEF=不可用";
    nav += " 定位误差ECEF=(" + FormatF(dx, 1) + "," + FormatF(dy, 1) + "," + FormatF(dz, 1) +
           ")m";
    nav += " 真值速度ECEF=不可用 实际速度ECEF=不可用";
    nav += " 速度误差ECEF=(" + FormatF(dvx, 3) + "," + FormatF(dvy, 3) + "," +
           FormatF(dvz, 3) + ")m/s";
  }
  nav += " |定位误差|=" + FormatF(err_norm, 1) + "m";
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "卫星自身定位误差功能测试", nav);
}

void WriteSbirsAngleError(std::uint32_t satellite_id, float sim_time_sec, std::uint32_t cycle,
                          std::uint64_t target_id, double az_error_deg, double el_error_deg) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  constexpr double kDegToRad = 0.017453292519943295;
  constexpr double kDegToArcsec = 3600.0;
  // 规范口径（验收判定标准 第9项）：写 ECI 测角残差（量测-真值，rad；须补 arcsec 以便
  // 与 5 arcsec 判定句直接比对）+ 相对卫星ID/目标ID；测量/真值角本身、会话 RMSE、
  // σ 来源不写进本项。
  std::string content = "相对卫星ID=" + std::to_string(satellite_id);
  content += " 目标ID=" + std::to_string(target_id);
  content += " 测角残差方位/俯仰(ECI)=(" + FormatF(az_error_deg * kDegToRad, 8) + "," +
             FormatF(el_error_deg * kDegToRad, 8) + ")rad";
  content += " 测角残差方位/俯仰=(" + FormatF(az_error_deg * kDegToArcsec, 2) + "," +
             FormatF(el_error_deg * kDegToArcsec, 2) + ")arcsec";
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "红外载荷角误差功能测试", content);
}

namespace {

// ECI 方位/俯仰（rad）→ ECEF 单位向量（方向随位置同旋转，旋转线性保模长）。
bool TryEciAzElToEcefDirection(double az_rad, double el_rad, double gmst_rad, double* dx,
                               double* dy, double* dz) {
  const double cos_el = std::cos(el_rad);
  oneq::coordinate::EcefPositionM rotated;
  const oneq::coordinate::EciPositionM direction(cos_el * std::cos(az_rad),
                                                 cos_el * std::sin(az_rad), std::sin(el_rad));
  if (!oneq::coordinate::TryEciToEcef(direction, gmst_rad, &rotated)) {
    return false;
  }
  *dx = rotated.x_m;
  *dy = rotated.y_m;
  *dz = rotated.z_m;
  return true;
}

// 双视线最近交会（ECEF）：两射线最近点取中点；平行/退化返回 false。
bool TryDualLosMidpointEcef(const DetectabilityEntry& a, const DetectabilityEntry& b,
                            double* x_m, double* y_m, double* z_m) {
  double dax = 0.0;
  double day = 0.0;
  double daz = 0.0;
  double dbx = 0.0;
  double dby = 0.0;
  double dbz = 0.0;
  if (!TryEciAzElToEcefDirection(a.measured_az_rad, a.measured_el_rad, a.gmst_rad, &dax, &day,
                                  &daz) ||
      !TryEciAzElToEcefDirection(b.measured_az_rad, b.measured_el_rad, b.gmst_rad, &dbx, &dby,
                                 &dbz)) {
    return false;
  }
  const double wx = a.satellite_ecef_m[0] - b.satellite_ecef_m[0];
  const double wy = a.satellite_ecef_m[1] - b.satellite_ecef_m[1];
  const double wz = a.satellite_ecef_m[2] - b.satellite_ecef_m[2];
  const double aa = dax * dax + day * day + daz * daz;
  const double bb = dax * dbx + day * dby + daz * dbz;
  const double cc = dbx * dbx + dby * dby + dbz * dbz;
  const double dd = dax * wx + day * wy + daz * wz;
  const double ee = dbx * wx + dby * wy + dbz * wz;
  const double denom = aa * cc - bb * bb;
  if (std::fabs(denom) < 1.0e-12) {
    return false;
  }
  const double t = (bb * ee - cc * dd) / denom;
  const double s = (aa * ee - bb * dd) / denom;
  *x_m = 0.5 * (a.satellite_ecef_m[0] + t * dax + b.satellite_ecef_m[0] + s * dbx);
  *y_m = 0.5 * (a.satellite_ecef_m[1] + t * day + b.satellite_ecef_m[1] + s * dby);
  *z_m = 0.5 * (a.satellite_ecef_m[2] + t * daz + b.satellite_ecef_m[2] + s * dbz);
  return true;
}

// 成行：目标ID [+位置LLA（仅双星定位）] + 逐星（相对卫星ID/斜距/量测角）+ 红外可探测。
// 仅限持有 DetectabilityPendingMutex() 的调用点使用：entries 引用自待定表，
// 落盘期间表不得被增删。
void EmitDetectabilityLine(std::uint64_t target_id,
                           const std::map<std::uint32_t, DetectabilityEntry>& entries) {
  if (entries.empty()) {
    return;
  }
  const DetectabilityEntry& first = entries.begin()->second;
  std::string content = "目标ID=" + std::to_string(target_id);
  if (entries.size() >= 2U) {
    // 位置 LLA 只在双星定位时写出：前两颗星（按卫星ID序）量测视线最近交会中点。
    const auto it_b = std::next(entries.begin());
    double x_m = 0.0;
    double y_m = 0.0;
    double z_m = 0.0;
    oneq::coordinate::LlaPositionDegM fix_lla{};
    if (TryDualLosMidpointEcef(first, it_b->second, &x_m, &y_m, &z_m) &&
        oneq::coordinate::TryEcefToLla(oneq::coordinate::EcefPositionM(x_m, y_m, z_m),
                                       &fix_lla)) {
      content += " 位置LLA=(" + FormatF(fix_lla.latitude_deg, 6) + "," +
                 FormatF(fix_lla.longitude_deg, 6) + "," + FormatF(fix_lla.altitude_m, 1) + ")";
    }
  }
  for (const auto& sat_entry : entries) {
    const DetectabilityEntry& entry = sat_entry.second;
    content += " 相对卫星ID=" + std::to_string(entry.satellite_id);
    content += " 相对卫星斜距=" + FormatF(entry.slant_range_m, 1) + "m";
    content += " 量测方位/俯仰(ECI)=(" + FormatF(entry.measured_az_rad, 8) + "," +
               FormatF(entry.measured_el_rad, 8) + ")rad";
    content += entry.narrow_field ? " 阶段=NFOV" : " 阶段=WFOV";
  }
  content += " 红外可探测=是";
  SBIRS_ACCEPTANCE_ITEM(first.sim_time_sec, first.cycle, "目标可探测性与动态参数生成功能测试",
                        content);
}

}  // namespace

void WriteSbirsTargetDetectability(std::uint32_t satellite_entity_id, float sim_time_sec,
                                   std::uint32_t cycle, std::uint64_t target_id,
                                   double slant_range_m, double measured_az_rad,
                                   double measured_el_rad, bool narrow_field,
                                   double satellite_ecef_x_m,
                                   double satellite_ecef_y_m, double satellite_ecef_z_m,
                                   double gmst_rad) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const std::lock_guard<std::mutex> pending_lock(DetectabilityPendingMutex());
  auto& pending = DetectabilityPending();
  // 旧周期条目已不可能再等来第二颗星：先顺延落盘。
  for (auto it = pending.begin(); it != pending.end() && it->first < cycle;) {
    for (auto& target_entry : it->second) {
      EmitDetectabilityLine(target_entry.first, target_entry.second);
    }
    it = pending.erase(it);
  }
  DetectabilityEntry entry;
  entry.satellite_id = satellite_entity_id;
  entry.sim_time_sec = sim_time_sec;
  entry.cycle = cycle;
  entry.slant_range_m = slant_range_m;
  entry.measured_az_rad = measured_az_rad;
  entry.measured_el_rad = measured_el_rad;
  entry.narrow_field = narrow_field;
  entry.satellite_ecef_m[0] = satellite_ecef_x_m;
  entry.satellite_ecef_m[1] = satellite_ecef_y_m;
  entry.satellite_ecef_m[2] = satellite_ecef_z_m;
  entry.gmst_rad = gmst_rad;
  auto& target_entries = pending[cycle][target_id];
  target_entries[satellite_entity_id] = entry;
  if (target_entries.size() >= 2U) {
    EmitDetectabilityLine(target_id, target_entries);
    pending[cycle].erase(target_id);
    if (pending[cycle].empty()) {
      pending.erase(cycle);
    }
  }
}

void FlushSbirsDetectabilityPending() {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const std::lock_guard<std::mutex> pending_lock(DetectabilityPendingMutex());
  auto& pending = DetectabilityPending();
  for (auto& cycle_entry : pending) {
    for (auto& target_entry : cycle_entry.second) {
      EmitDetectabilityLine(target_entry.first, target_entry.second);
    }
  }
  pending.clear();
}

void WriteSbirsAngleStateEstimate(float sim_time_sec, std::uint32_t cycle, std::uint64_t target_id,
                                  double azimuth_deg, double elevation_deg,
                                  double azimuth_rate_deg_per_s, double elevation_rate_deg_per_s) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED() && !SBIRS_FUSION_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  constexpr double kDegToRad = 0.017453292519943295;
  // 2026-09-28 合并（验收判定标准 第18项·标准卡尔曼滤波）：「滤波方位/俯仰及其变化率」
  // 属第18项判定字段，方括号须用规范测试项名、只写融合文件，角度与变化率统一 rad / rad/s。
  std::string content = "目标ID=" + std::to_string(target_id);
  content += " 滤波方位/俯仰(ECI)=(" + FormatF(azimuth_deg * kDegToRad, 9) + "," +
             FormatF(elevation_deg * kDegToRad, 9) + ")rad";
  content += " 方位变化率=" + FormatF(azimuth_rate_deg_per_s * kDegToRad, 12) + "rad/s";
  content += " 俯仰变化率=" + FormatF(elevation_rate_deg_per_s * kDegToRad, 12) + "rad/s";
  SBIRS_FUSION_ACCEPTANCE_ITEM(sim_time_sec, cycle, "标准卡尔曼滤波功能测试", content);
}

void WriteSbirsLifecycleEvents(std::uint32_t satellite_id, float sim_time_sec,
                               std::uint32_t cycle,
                               const std::vector<session::SbirsDetectionLifecycleEvent>& events,
                               const session::SbirsCycleInput& input) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  std::map<std::uint64_t, const session::SbirsSceneTarget*> by_id;
  for (const session::SbirsSceneTarget& target : input.scene) {
    by_id[target.target_id] = &target;
  }
  for (const session::SbirsDetectionLifecycleEvent& event : events) {
    const char* name = EventName(event.kind);
    if (name == nullptr) {
      continue;
    }
    // 规范口径（验收判定标准 第24项）：「验收内容：」后首字段为 事件=，随后卫星ID/
    // 等级/目标ID/阶段/时间/位置（评审 2026-08-26 条11：事件行标注观测阶段，区分
    // 宽场首次探测与窄场（捕获/跟踪）首次探测）。
    std::string content = "事件=";
    content += name;
    content += " 卫星ID=" + std::to_string(satellite_id);
    content += " 等级=" + std::to_string(EventLevel(event.kind));
    content += " 目标ID=" + std::to_string(event.target_id);
    content += " 阶段=";
    content += StageName(event);
    content += " 时间=" + FormatF(static_cast<double>(sim_time_sec), 3) + "s";
    const auto found = by_id.find(event.target_id);
    if (found != by_id.end() && found->second != nullptr) {
      content += " 位置ECEF=" + FormatVec3(found->second->position_ecef_m.x,
                                           found->second->position_ecef_m.y,
                                           found->second->position_ecef_m.z, 1);
    }
    SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "特殊事件监测与提示功能测试", content);
  }
}

void WriteSbirsExternalSceneInput(const config::SbirsEnvironmentConfig& environment,
                                  double background_radiance_w_sr_m2,
                                  double detector_temperature_k, float sim_time_sec,
                                  std::uint32_t cycle, std::uint32_t satellite_id,
                                  const session::SbirsSceneTargetList& scene) {
  // 验收判定标准 第58项：目标与红外环境场景输入接口测试 → log/IRExternalInterface.log。
  // 记录目标数量、目标状态数组、红外辐射特征、背景辐射与大气透过率；场景有效期字段已删除。
  // 无场景目标时本项没有可投影的输入对象（目标数量/状态数组/辐射特征全为空），
  // 按「没有相关信息输入则不打印日志」整条省略（2026-09-28）；不再写“目标数量=0”空投影行。
  if (scene.empty()) {
    return;
  }
  std::string head = "卫星ID=" + std::to_string(satellite_id);
  head += " 目标数量=" + std::to_string(scene.size());
  head += " 背景辐射亮度=" + FormatF(background_radiance_w_sr_m2, 4) + "W/(sr·m^2)";
  head += " 基础大气透过率=" +
          FormatF(static_cast<double>(environment.base_atmospheric_transmittance), 6);
  head += " 环境温度=" + FormatF(static_cast<double>(environment.temperature_c), 2) + "C";
  head += " 相对湿度=" +
          FormatF(static_cast<double>(environment.relative_humidity_percent), 2) + "%";
  if (detector_temperature_k > 0.0) {
    head += " 探测器温度=" + FormatF(detector_temperature_k, 2) + "K";
  }
  SBIRS_IR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, "目标与红外环境场景输入接口测试", head);

  // 目标状态数组：逐目标 ECEF 位置/速度、有效标志与红外辐射特征（辐射强度）。
  for (const session::SbirsSceneTarget& target : scene) {
    std::string row = "卫星ID=" + std::to_string(satellite_id);
    row += " 目标ID=" + std::to_string(target.target_id);
    row += " 目标名称=" +
           (target.target_name.empty() ? std::string("未提供") : target.target_name);
    row += " 目标状态=" + std::string(target.active ? "有效" : "无效");
    row += " 位置ECEF=" + FormatVec3(target.position_ecef_m.x, target.position_ecef_m.y,
                                     target.position_ecef_m.z, 3) + "m";
    if (target.has_velocity_ecef_m_per_s) {
      row += " 速度ECEF=" + FormatVec3(target.velocity_ecef_m_per_s.x,
                                       target.velocity_ecef_m_per_s.y,
                                       target.velocity_ecef_m_per_s.z, 3) + "m/s";
    }
    // 中译：红外辐射强度是本项的核心字段，2026-10-21 一次回退时被误删，此处恢复。
    // 标识：第 58 项字段恢复（2026-10-21）。
    row += " 红外辐射强度=" + FormatF(target.radiant_intensity_w_per_sr, 6) + "W/sr";
    // 中译：目标行**变化沿 + 心跳**——静止目标（阵地/指挥所）的位置与强度整轮不变，但此前
    //   每卫星每周期都重写一行，实测单目标重复 4,673 次、本文件因此多出十几万行完全相同
    //   的文本。规则：首次必输出、内容变化必输出、未变化时每 kSceneHeartbeatCycles 周期
    //   补一条心跳，保留"该目标仍在场景中"的周期性证据。
    // 标识：第 58 项目标行变化沿（2026-10-21）。
    constexpr std::uint32_t kSceneHeartbeatCycles = 100U;
    static std::map<std::string, std::pair<std::string, std::uint32_t>> last_scene_rows;
    const std::string scene_key =
        std::to_string(satellite_id) + ":" + std::to_string(target.target_id);
    std::map<std::string, std::pair<std::string, std::uint32_t>>::iterator cached =
        last_scene_rows.find(scene_key);
    const bool changed = cached == last_scene_rows.end() || cached->second.first != row;
    const bool heartbeat_due =
        cached == last_scene_rows.end() || cycle < cached->second.second ||
        (cycle - cached->second.second >= kSceneHeartbeatCycles);
    if (changed || heartbeat_due) {
      SBIRS_IR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, "目标与红外环境场景输入接口测试",
                                       row);
      last_scene_rows[scene_key] = std::make_pair(row, cycle);
    }
  }
}

namespace {

/** @brief Eigen 方阵 → "[[a,b];[c,d]]" 文本（验收行内联协方差用）。 */
template <typename MatrixType>
std::string FormatEigenMatrix(const MatrixType& matrix) {
  std::string text = "[";
  for (int row = 0; row < matrix.rows(); ++row) {
    if (row != 0) {
      text += ";";
    }
    text += "[";
    for (int col = 0; col < matrix.cols(); ++col) {
      if (col != 0) {
        text += ",";
      }
      text += FormatF(static_cast<double>(matrix(row, col)), 6);
    }
    text += "]";
  }
  text += "]";
  return text;
}

}  // namespace

void WriteSbirsExternalObservationOutput(
    std::uint32_t satellite_id, float sim_time_sec, std::uint32_t cycle,
    const std::vector<SbirsPipelineDetection>& detections, const SbirsPipelineSnapshot& snapshot) {
  // 验收判定标准 第67项：红外观测与航迹输出接口测试 → log/IRExternalInterface.log。
  // 分层输出，不混淆语义：
  //   ① 原始观测层：检测 ID、ECI 方位/俯仰、IR SNR、观测阶段、检测标志；
  //   ② 归属/诊断层：目标 ID 与估计距离（明确标注「非被动红外直接测距」）；
  //   ③ 跟踪状态层：内部滤波状态估计与状态协方差（按目标 ID 关联）。
  // SCR 与目标类别字段已删除，不写。
  std::size_t observed_count = 0U;
  for (const SbirsPipelineDetection& detection : detections) {
    if (detection.record.detected) {
      ++observed_count;
    }
  }
  // 本周期没有任何检出记录时，对外观测输出接口没有可发布的观测事实：
  // 按「没有相关信息输出则不打印日志」整条省略（2026-09-28），
  // 不再写“观测数量=0 目标数量=N”这类无观测内容的空投影行。
  if (observed_count == 0U) {
    return;
  }
  std::string head = "卫星ID=" + std::to_string(satellite_id);
  head += " 观测数量=" + std::to_string(observed_count);
  head += " 目标数量=" + std::to_string(detections.size());
  SBIRS_IR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, "红外观测与航迹输出接口测试", head);

  for (const SbirsPipelineDetection& detection : detections) {
    const output::SbirsDetectionRecord& record = detection.record;
    const attribution::SbirsDetectionAttributionRecord& attribution = detection.attribution;
    if (!record.detected) {
      continue;  // 未过门记录不属于对外观测输出
    }
    std::string row = "卫星ID=" + std::to_string(satellite_id);
    row += " 观测ID=" + std::to_string(record.detection_id);
    row += " 目标ID=" + std::to_string(attribution.target_id);
    row += " 检测标志=" + std::string(YesNo(record.detected));
    row += " 方位(ECI)=" + FormatF(static_cast<double>(record.azimuth_rad), 8) + "rad";
    row += " 俯仰(ECI)=" + FormatF(static_cast<double>(record.elevation_rad), 8) + "rad";
    row += " IR_SNR=" + FormatF(static_cast<double>(record.infrared_snr_linear), 6);
    row += " 观测阶段=";
    switch (record.observation_stage) {
      case output::SbirsObservationStage::kWideFieldSearch:
        row += "WFOV";
        break;
      case output::SbirsObservationStage::kNarrowFieldAcquisition:
        row += "NFOV捕获";
        break;
      case output::SbirsObservationStage::kNarrowFieldTrack:
        row += "NFOV跟踪";
        break;
    }
    // 归属/诊断层：显式标注不是被动红外载荷的直接测距结果。
    row += " 引导估计距离=" + FormatF(static_cast<double>(attribution.estimated_range_m), 1) +
           "m(归属诊断,非被动红外直接测距)";
    SBIRS_IR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, "红外观测与航迹输出接口测试", row);

    // 跟踪状态层：优先 IMM 合成状态，其次 EKF/IMM 单模型状态，最后角度 CV-KF 状态。
    std::string state_row = "卫星ID=" + std::to_string(satellite_id);
    state_row += " 目标ID=" + std::to_string(attribution.target_id);
    bool wrote_state = false;
    const auto imm_found = snapshot.imm_snapshots.find(attribution.target_id);
    if (snapshot.imm_active && imm_found != snapshot.imm_snapshots.end() &&
        !imm_found->second.model_states.empty()) {
      const tracking::SbirsImmModelState& model = imm_found->second.model_states.front();
      state_row += " 状态估计来源=IMM";
      state_row += " 状态向量=" + FormatEigenMatrix(model.state.mean);
      state_row += " 状态协方差=" + FormatEigenMatrix(model.state.covariance);
      state_row += " 模型权重=" + FormatF(static_cast<double>(model.weight), 6);
      wrote_state = true;
    } else {
      const auto filter_found = snapshot.filter_states.find(attribution.target_id);
      if (filter_found != snapshot.filter_states.end()) {
        state_row += " 状态估计来源=EKF";
        state_row += " 状态向量=" + FormatEigenMatrix(filter_found->second.mean);
        state_row += " 状态协方差=" + FormatEigenMatrix(filter_found->second.covariance);
        wrote_state = true;
      } else {
        const auto angle_found = snapshot.angle_kf_states.find(attribution.target_id);
        if (angle_found != snapshot.angle_kf_states.end()) {
          state_row += " 状态估计来源=角度CV-KF";
          state_row += " 状态向量=" + FormatEigenMatrix(angle_found->second.mean);
          state_row += " 状态协方差=" + FormatEigenMatrix(angle_found->second.covariance);
          wrote_state = true;
        }
      }
    }
    // 过滤/角度后端均未持有该目标状态时不伪造估计值（§0 规则3）。
    // 但第 67 项交换数据要求"状态估计 + 状态协方差"恒在：捕获阶段确实没有滤波器状态，
    // 故显式声明"未建立/不适用"，既不编造数值也让字段恒在（2026-10-19 回归：421 行缺）。
    if (!wrote_state) {
      state_row += " 状态估计来源=未建立(捕获阶段) 状态向量=不适用 状态协方差=不适用";
    }
    SBIRS_IR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, "红外观测与航迹输出接口测试",
                                     state_row);
  }
}

}  // namespace pipeline
}  // namespace sbirs_sensor
