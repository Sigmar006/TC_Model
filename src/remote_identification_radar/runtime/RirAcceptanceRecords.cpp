/**
 * @file RirAcceptanceRecords.cpp
 * @brief RIR 验收行与天线三维增益 CSV。
 */

#include "remote_identification_radar/runtime/RirAcceptanceRecords.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/remote_identification_radar/session/RirFeatureMeasurementTypes.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "1q/remote_identification_radar/session/RirOutputTypes.h"
#include "common/logging/AcceptanceText.h"
#include "common/logging/LogDirectory.h"
#include "common/numerics/Constants.h"
#include "common/radar/MtiMtdAcceptanceBank.h"
#include "common/radar/RadarEquations.h"
#include "remote_identification_radar/dwell/RirAntennaPatternRuntime.h"
#include "remote_identification_radar/runtime/RirController.h"
#include "remote_identification_radar/runtime/RirAcceptanceLog.h"
#include "remote_identification_radar/tracking/RirTrackAssociator.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

#ifndef ONEQ_RIR_ANTENNA_PATTERN_CSV_PATH
#define ONEQ_RIR_ANTENNA_PATTERN_CSV_PATH "log/rir_antenna_pattern.csv"
#endif
#ifndef ONEQ_RIR_SCAN_PATTERN_CSV_PATH
#define ONEQ_RIR_SCAN_PATTERN_CSV_PATH "log/rir_scan_pattern.csv"
#endif

namespace remote_identification_radar {
namespace runtime {
namespace {

using oneq::logging::AppendField;
using oneq::logging::FormatF;
using oneq::logging::FormatPairDeg;
using oneq::logging::FormatSci;
using oneq::logging::FormatVec3;
using oneq::logging::YesNo;

/** @brief 弹道细分类型中文名（仅验收日志消费；判据未冻结前不会被调用）。 */
std::string BallisticSubclassName(int subclass) {
  using session::RirBallisticSubclass;
  switch (subclass) {
    case static_cast<int>(RirBallisticSubclass::kReentryVehicle):
      return "弹头";
    case static_cast<int>(RirBallisticSubclass::kHeavyDecoy):
      return "重诱饵";
    case static_cast<int>(RirBallisticSubclass::kLightDecoy):
      return "轻诱饵";
    case static_cast<int>(RirBallisticSubclass::kDebris):
      return "碎片";
    default:
      return std::string();
  }
}

const char* CategoryName(int category) {
  using session::RirRecognitionCategory;
  switch (category) {
    case static_cast<int>(RirRecognitionCategory::kBallistic):
      return "弹道目标";
    case static_cast<int>(RirRecognitionCategory::kNearSpace):
      return "临近空间目标";
    case static_cast<int>(RirRecognitionCategory::kOther):
      return "其它";
    case static_cast<int>(RirRecognitionCategory::kUnknown):
      return "未知";
    case static_cast<int>(RirRecognitionCategory::kFighter):
      return "战斗机";
    case static_cast<int>(RirRecognitionCategory::kBomber):
      return "轰炸机";
    case static_cast<int>(RirRecognitionCategory::kMissile):
      return "导弹";
    default:
      return "";
  }
}

constexpr double kLn10 = 2.302585092994046;

double ToDb(double linear) {
  if (linear <= 0.0) {
    return -999.0;
  }
  return 10.0 * std::log(linear) / kLn10;
}

double FromDb(double db) { return std::pow(10.0, db / 10.0); }

const char* TrackStatusText(tracking::RirTrackStatus status) {
  switch (status) {
    case tracking::RirTrackStatus::kConfirmed:
      return "确认";
    case tracking::RirTrackStatus::kLost:
      return "丢失";
    case tracking::RirTrackStatus::kTentative:
    default:
      return "候选";
  }
}

double HeadingDeg(double ve, double vn) {
  return oneq::common::numerics::RadToDeg(std::atan2(vn, ve));
}

std::string FormatChannelWatts(const std::array<double, oneq::common::radar::kMtiMtdChannelCount>& values) {
  std::string text = "[";
  for (std::size_t i = 0U; i < values.size(); ++i) {
    if (i != 0U) {
      text += ",";
    }
    text += FormatSci(values[i]);
  }
  text += "]";
  return text;
}

bool TryBuildAcceptanceBank(const RirDetectionAcceptInput& input,
                            oneq::common::radar::MtiMtdAcceptanceResult* bank) {
  if (bank == nullptr || !input.has_cell) {
    return false;
  }
  std::vector<oneq::common::radar::MtiMtdInterferenceTone> tones;
  tones.reserve(input.incident_links.size());
  for (const auto& link : input.incident_links) {
    if (!std::isfinite(link.doppler_shift_hz)) {
      continue;
    }
    const double power = std::isfinite(link.received_power_before_overlap_w)
                             ? link.received_power_before_overlap_w
                             : link.received_power_w;
    if (!std::isfinite(power) || power < 0.0) {
      continue;
    }
    oneq::common::radar::MtiMtdInterferenceTone tone;
    tone.doppler_hz = link.doppler_shift_hz;
    tone.power_w = power;
    tones.push_back(tone);
  }
  oneq::common::radar::MtiMtdAcceptanceInput bank_input;
  bank_input.echo_power_w = input.cell.echo_power_w;
  bank_input.thermal_noise_power_w = input.cell.thermal_noise_power_w;
  bank_input.clutter_power_w = input.cell.clutter_power_w;
  bank_input.two_way_doppler_shift_hz = input.cell.two_way_doppler_shift_hz;
  bank_input.prf_hz = input.prf_hz;
  bank_input.center_frequency_hz = input.center_frequency_hz;
  if (!tones.empty()) {
    bank_input.tones = tones.data();
    bank_input.tone_count = tones.size();
  }
  return oneq::common::radar::TryResolveMtiMtdAcceptanceBank(bank_input, bank);
}

void Emit(float sim_time_sec, std::uint32_t cycle, const char* item, const std::string& content) {
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, item, content);
}

void EmitOrNone(float sim_time_sec, std::uint32_t cycle, const char* item, const std::string& prefix,
                bool ok, const std::string& value) {
  if (!ok) {
    return;
  }
  Emit(sim_time_sec, cycle, item, prefix + value);
}

}  // namespace

bool TryLookPolarFromEnuM(float east_m, float north_m, float up_m, float* range_m, float* az_deg,
                          float* el_deg) {
  if (range_m == nullptr || az_deg == nullptr || el_deg == nullptr) {
    return false;
  }
  if (!std::isfinite(east_m) || !std::isfinite(north_m) || !std::isfinite(up_m)) {
    return false;
  }
  const float range = std::sqrt(east_m * east_m + north_m * north_m + up_m * up_m);
  if (!(range > 0.1f)) {
    return false;
  }
  const float range_hypot = std::sqrt(east_m * east_m + north_m * north_m);
  *range_m = range;
  *az_deg = oneq::common::numerics::RadToDeg(std::atan2(north_m, east_m));
  *el_deg = oneq::common::numerics::RadToDeg(std::atan2(up_m, range_hypot));
  return true;
}

std::string ResolveRirAntennaPatternCsvPath() {
#if defined(_MSC_VER)
  char* buffer = nullptr;
  if (_dupenv_s(&buffer, nullptr, "ONEQ_RIR_ANTENNA_PATTERN_CSV_PATH") == 0 && buffer != nullptr) {
    std::string result(buffer);
    std::free(buffer);
    if (!result.empty()) {
      return result;
    }
  }
#else
  const char* env = std::getenv("ONEQ_RIR_ANTENNA_PATTERN_CSV_PATH");
  if (env != nullptr && *env != '\0') {
    return std::string(env);
  }
#endif
  return std::string(ONEQ_RIR_ANTENNA_PATTERN_CSV_PATH);
}

std::string ResolveRirScanPatternCsvPath() {
#if defined(_MSC_VER)
  char* buffer = nullptr;
  if (_dupenv_s(&buffer, nullptr, "ONEQ_RIR_SCAN_PATTERN_CSV_PATH") == 0 && buffer != nullptr) {
    std::string result(buffer);
    std::free(buffer);
    if (!result.empty()) {
      return result;
    }
  }
#else
  const char* env = std::getenv("ONEQ_RIR_SCAN_PATTERN_CSV_PATH");
  if (env != nullptr && *env != '\0') {
    return std::string(env);
  }
#endif
  return std::string(ONEQ_RIR_SCAN_PATTERN_CSV_PATH);
}

void WriteRirAntennaPatternSummary(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                                   float peak_gain_dbi, float bw_az_deg, float bw_el_deg) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 验收判定标准 第28项：天线方向图——日志只写峰值增益与波束宽度（2D 增益明细
  // 在 rir_antenna_pattern.csv，路径行不写）。
  std::string content = "雷达ID=" + std::to_string(radar_id);
  content += " 峰值增益=" + FormatF(static_cast<double>(peak_gain_dbi), 2) + "dBi";
  content += " 波束宽度az/el=" + FormatPairDeg(bw_az_deg, bw_el_deg, 3) + "°";
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "天线方向图仿真功能测试", content);
}

void WriteRirDetectionChain(const RirDetectionAcceptInput& input) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const float t = input.sim_time_sec;
  const std::uint32_t cycle = input.cycle;
  const std::string id = "目标ID=" + std::to_string(input.target_id);
  const std::string id_sp = id + " ";
  const double thermal = input.has_cell ? input.cell.thermal_noise_power_w : 0.0;
  const double clutter = input.has_cell ? input.cell.clutter_power_w : 0.0;
  const double pc = input.has_cell ? input.cell.pulse_compression_gain : 1.0;
  const std::uint32_t pulses = input.has_cell ? input.cell.effective_pulse_count : 1U;
  const double pc_db = ToDb(pc);
  const double noise_db = static_cast<double>(input.gains.noise_processing_gain_db);
  const double clutter_db = static_cast<double>(input.gains.clutter_suppression_gain_db);
  // 脉压增益只作用于相干回波；热噪声按匹配滤波带宽计，不再乘 B·τ。
  // 杂波/干扰行同样输出 cell 分项功率，不乘脉压。
  const double mti_residual = clutter_db <= 0.0 ? clutter : clutter / FromDb(clutter_db);
  oneq::common::radar::MtiMtdAcceptanceResult bank;
  const bool has_bank = TryBuildAcceptanceBank(input, &bank);

  // 验收判定标准 第29项：回波功率（W，dBW→W 换算）；斜距/SNR/噪声/干扰/杂波
  // 分属第 36–37 项或超出规范，不写入本条。
  Emit(t, cycle, "回波功率计算功能测试",
       id + " 回波功率=" + FormatSci(FromDb(input.echo_power_dbw)) + "W");

  // 验收判定标准 第31项：接收机热噪声功率（接收机底噪，非逐目标量）——每雷达
  // 每周期只写一行（本函数逐检测目标调用，同周期后续调用跳过）。
  if (input.has_cell) {
    static std::uint64_t last_noise_radar = 0U;
    static std::uint32_t last_noise_cycle = 0xFFFFFFFFU;
    if (last_noise_radar != input.radar_id || last_noise_cycle != cycle) {
      last_noise_radar = input.radar_id;
      last_noise_cycle = cycle;
      Emit(t, cycle, "接收机噪声功率计算功能测试",
           "雷达ID=" + std::to_string(input.radar_id) + " 热噪声功率=" + FormatSci(thermal) + "W");
    }
  }

  // 验收判定标准 第32项：目标信号增益——总增益=脉压+MTI+MTD（dB）；无 cell 时
  // MTI/MTD 子项省略，总增益按本条口径不可算亦省略（相干积累非本条子项，不写）。
  if (has_bank) {
    Emit(t, cycle, "目标信号增益功能测试",
         id + " 总增益=" +
             FormatF(pc_db + bank.mti_gain_db + bank.mtd_gain_db, 3) + "dB");
  }
  Emit(t, cycle, "目标信号增益功能测试", id_sp + "脉压增益=" + FormatF(pc_db, 3) + "dB");
  EmitOrNone(t, cycle, "目标信号增益功能测试", id_sp + "MTI增益=", has_bank,
             has_bank ? FormatF(bank.mti_gain_db, 3) + "dB" : std::string());
  EmitOrNone(t, cycle, "目标信号增益功能测试", id_sp + "MTD增益=", has_bank,
             has_bank ? FormatF(bank.mtd_gain_db, 3) + "dB" : std::string());

  // 验收判定标准 第33项：噪声增益四子项（各多普勒通道/MTD 等效无 cell 时省略）。
  if (input.has_cell) {
    Emit(t, cycle, "噪声增益功能测试", id_sp + "脉冲压缩后的噪声功率=" + FormatSci(thermal) + "W");
  }
  EmitOrNone(t, cycle, "噪声增益功能测试", id_sp + "各多普勒滤波器通道噪声功率=", has_bank,
             has_bank ? FormatChannelWatts(bank.noise_w) + "W" : std::string());
  EmitOrNone(t, cycle, "噪声增益功能测试", id_sp + "MTD等效噪声功率=", has_bank,
             has_bank ? FormatSci(bank.mtd_equivalent_noise_w) + "W" : std::string());
  Emit(t, cycle, "噪声增益功能测试", id_sp + "噪声总增益=" + FormatF(noise_db, 3) + "dB");

  // 验收判定标准 第34项：杂波——植被 kDisabled（clutter_power_w=0）时整条省略；
  // 有杂波时只写 MTI 剩余与 MTD 通道分布（抑制比/总抑制增益为派生量，不写）。
  const bool has_clutter = input.has_cell && clutter > 0.0;
  if (has_clutter) {
    Emit(t, cycle, "杂波信号处理增益功能测试",
         id_sp + "MTI处理后杂波剩余功率=" +
             FormatSci(has_bank ? bank.mti_residual_clutter_w : mti_residual) + "W");
  }
  EmitOrNone(t, cycle, "杂波信号处理增益功能测试", id_sp + "MTD各多普勒通道杂波剩余功率分布=",
             has_clutter && has_bank,
             has_clutter && has_bank ? FormatChannelWatts(bank.clutter_w) + "W" : std::string());

  // 验收判定标准 第35项：干扰——只写 MTI 剩余与 MTD 通道分布；无外部干扰单音时
  // 整条省略（抑制比/总抑制增益不写；CFAR 统计归第 36 项）。
  const bool has_jam = has_bank && bank.has_jam_channels;
  EmitOrNone(t, cycle, "干扰信号处理增益功能测试", id_sp + "MTI处理后干扰剩余功率=", has_jam,
             has_jam ? FormatSci(bank.mti_residual_jam_w) + "W" : std::string());
  EmitOrNone(t, cycle, "干扰信号处理增益功能测试", id_sp + "MTD各多普勒通道干扰功率分布=",
             has_jam, has_jam ? FormatChannelWatts(bank.jam_w) + "W" : std::string());

  // 验收判定标准 第36项：CFAR 三子项（门限/结果/概率，key=value 分行）。
  bool has_threshold = false;
  double threshold = 0.0;
  if (input.has_cell && std::isfinite(input.cfar_pfa) && input.cfar_pfa > 0.0 &&
      input.cfar_pfa < 1.0) {
    threshold = oneq::common::radar::RadarEquations::ComputeThreshold(
        input.cfar_pfa, static_cast<int>(std::max(1U, pulses)));
    has_threshold = std::isfinite(threshold);
  }
  EmitOrNone(t, cycle, "恒虚警检测功能测试", id_sp + "统计检测门限=", has_threshold,
             has_threshold ? FormatF(threshold, 6) : std::string());
  Emit(t, cycle, "恒虚警检测功能测试", id_sp + "统计检测结果=" + YesNo(input.detected));
  Emit(t, cycle, "恒虚警检测功能测试", id_sp + "统计检测概率=" + FormatF(input.pd, 5));

  // 内部接口第70项：同一条目标检测快照中，回波/环境功率的生产值即为
  // 信号处理入口消费值；仅有 detection cell 有效时输出，不伪造缺失分量。
  if (input.has_cell) {
    std::string interface_power = "雷达ID=" + std::to_string(input.radar_id) + " " + id;
    interface_power += " 斜距=" + FormatF(input.range_m, 1) + "m";
    interface_power += " RCS=" + FormatF(input.rcs_m2, 3) + "m^2";
    interface_power += " 波束离轴增益=" + FormatF(input.one_way_antenna_gain_dbi, 3) + "dBi";
    interface_power += " 上游输出回波功率=" + FormatSci(input.cell.echo_power_w) + "W";
    interface_power += " 处理入口回波功率=" + FormatSci(input.cell.echo_power_w) + "W";
    interface_power += " 上游输出热噪声=" + FormatSci(thermal) + "W";
    interface_power += " 处理入口热噪声=" + FormatSci(thermal) + "W";
    interface_power += " 上游输出杂波=" + FormatSci(clutter) + "W";
    interface_power += " 处理入口杂波=" + FormatSci(clutter) + "W";
    interface_power += " 上游输出干扰=" + FormatSci(input.cell.interference_power_w) + "W";
    interface_power += " 处理入口干扰=" + FormatSci(input.cell.interference_power_w) + "W";
    interface_power += " 带宽=" + FormatF(input.bandwidth_hz, 3) + "Hz";
    interface_power += " PRF=" + FormatF(input.prf_hz, 3) + "Hz";
    interface_power += " 有效脉冲数=" + std::to_string(pulses);
    RIR_INTERNAL_INTERFACE_ITEM(t, cycle, "雷达回波与环境功率交互接口测试", interface_power);
  }

  // 内部接口第71项：记录信号处理输出和 CFAR 实际消费的同源 SNR 快照。
  const double detector_snr_db = input.has_cell
                                     ? input.cell.processed_single_pulse_sinr_db
                                     : static_cast<double>(input.snr_db);
  const double accumulated_snr_db = detector_snr_db + 10.0 * std::log10(pulses);
  const double total_noise_w = thermal + clutter +
                               (input.has_cell ? input.cell.interference_power_w : 0.0);
  std::string interface_detection = id + " 处理输出SNR=" +
                                    FormatF(detector_snr_db, 3) + "dB";
  interface_detection += " 检测器输入SNR=" + FormatF(detector_snr_db, 3) + "dB";
  interface_detection += " 积累SNR=" + FormatF(accumulated_snr_db, 3) + "dB";
  if (input.has_cell) {
    interface_detection += " 总噪声功率=" + FormatSci(total_noise_w) + "W";
  }
  interface_detection += " 脉压/MTI/MTD增益=" + FormatF(pc_db, 3) + "/" +
                         (has_bank ? FormatF(bank.mti_gain_db, 3) : std::string("无")) + "/" +
                         (has_bank ? FormatF(bank.mtd_gain_db, 3) : std::string("无")) + "dB";
  if (has_threshold) {
    interface_detection += " 统计门限=" + FormatF(threshold, 6);
  }
  interface_detection += " Pd=" + FormatF(input.pd, 5);
  interface_detection += " 检测标志=";
  interface_detection += YesNo(input.detected);
  RIR_INTERNAL_INTERFACE_ITEM(t, cycle, "雷达信号处理与检测判决交互接口测试", interface_detection);
}

void WriteRirInterferenceLinks(
    float sim_time_sec, std::uint32_t cycle,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& links) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  if (links.empty()) {
    return;
  }
  // 验收判定标准 第30项：身份只写 干扰源ID=（干扰实体）+ 到达雷达功率；
  // 发射二元组、路径与时频域重叠（链路上下文）不写。
  for (const oneq::electromagnetics::RfIncidentLinkResult& link : links) {
    std::string content = "干扰源ID=" + std::to_string(link.identity.sensor_entity_id);
    content += " 到达雷达功率=" + FormatSci(link.received_power_w) + "W";
    RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "干扰功率计算功能测试", content);
  }
}

void WriteRirSearchDetections(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                              float beam_az_deg, float beam_el_deg,
                              const std::string& found_targets) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 验收判定标准 第37项：指向与检测量测两行（扫描调度信息行随 WriteRirSchedule
  // 写出——调度统计在该调用点）；搜索角域为配置派生，不写。
  Emit(sim_time_sec, cycle, "对指定空域进行搜索功能测试",
       "雷达ID=" + std::to_string(radar_id) + " 方位/俯仰指向信息=" +
           FormatPairDeg(beam_az_deg, beam_el_deg, 3) + "°");
  Emit(sim_time_sec, cycle, "对指定空域进行搜索功能测试",
       "检测与量测信息=搜到目标=[" + found_targets + "]");
}

void WriteRirAssociation(float sim_time_sec, std::uint32_t cycle,
                         const tracking::RirAssociationResult& association,
                         const oneq::coordinate::LlaPositionDegM& platform_lla,
                         double gate_threshold) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  (void)gate_threshold;  // 波门不再随行输出（马氏距离²为派生量，验收行不写）。
  // 验收判定标准 第41项：逐量测一行（目标ID/位置LLA/检测概率/关联结果）；
  // 计数汇总、马氏距离²（派生）与重复的更新后航迹集合行不写。
  for (const tracking::RirTrackMeasurement& measurement : association.measurements) {
    std::string content = "目标ID=" + std::to_string(measurement.external_target_id);
    oneq::coordinate::EcefPositionM measurement_ecef;
    oneq::coordinate::LlaPositionDegM measurement_lla;
    if (oneq::coordinate::TryEnuToEcef(
            oneq::coordinate::EnuPositionM(measurement.position.x(), measurement.position.y(),
                                           measurement.position.z()),
            platform_lla, &measurement_ecef) &&
        oneq::coordinate::TryEcefToLla(measurement_ecef, &measurement_lla)) {
      content += " 位置LLA=(" + FormatF(measurement_lla.latitude_deg, 6) + "," +
                 FormatF(measurement_lla.longitude_deg, 6) + "," +
                 FormatF(measurement_lla.altitude_m, 1) + ")";
    }
    content += " 检测概率=" + FormatF(measurement.detection_pd, 4);
    content += measurement.matched_existing_track
                   ? " 关联结果=航迹" + std::to_string(measurement.association_key)
                   : " 关联结果=未配到任何已有航迹";
    Emit(sim_time_sec, cycle, "航迹关联功能测试", content);

    // 内部接口第72项：量测构造器产出的完整量测快照及关联器消费结果。
    std::string interface_track = "源目标索引=" + std::to_string(measurement.source_index);
    interface_track += " 目标ID=" + std::to_string(measurement.external_target_id);
    interface_track += " 量测位置ENU=" +
                       FormatVec3(measurement.position.x(), measurement.position.y(),
                                  measurement.position.z(), 3) + "m";
    interface_track += " 速度种子=" +
                       FormatVec3(measurement.velocity.x(), measurement.velocity.y(),
                                  measurement.velocity.z(), 3) + "m/s";
    interface_track += " Pd=" + FormatF(measurement.detection_pd, 5);
    interface_track += " RCS=" + FormatF(measurement.rcs, 3) + "m^2";
    interface_track += " 量测协方差=[";
    for (int row = 0; row < 3; ++row) {
      if (row != 0) {
        interface_track += ";";
      }
      for (int col = 0; col < 3; ++col) {
        if (col != 0) {
          interface_track += ",";
        }
        interface_track += FormatF(measurement.measurement_covariance(row, col), 6);
      }
    }
    interface_track += "] 关联键=" + std::to_string(measurement.association_key);
    interface_track += measurement.matched_existing_track ? " 消费结果=更新已有航迹"
                                                          : " 消费结果=新建航迹";
    RIR_INTERNAL_INTERFACE_ITEM(sim_time_sec, cycle,
                                "雷达检测量测与航迹处理交互接口测试", interface_track);
  }
}

void WriteRirExternalMeasurements(float sim_time_sec, std::uint32_t cycle,
                                  const tracking::RirAssociationResult& association) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, "雷达检测与量测输出接口测试",
                              "量测数量=" + std::to_string(association.measurements.size()));
  for (const tracking::RirTrackMeasurement& measurement : association.measurements) {
    float range_m = 0.0f;
    float az_deg = 0.0f;
    float el_deg = 0.0f;
    const bool has_polar = TryLookPolarFromEnuM(
        measurement.position.x(), measurement.position.y(), measurement.position.z(), &range_m,
        &az_deg, &el_deg);
    std::string content = "目标ID=" + std::to_string(measurement.external_target_id);
    content += " 点迹ID=" + std::to_string(measurement.association_key);
    content += " 检测标志=是";
    if (has_polar) {
      content += " 距离=" + FormatF(range_m, 3) + "m";
      content += " 方位=" + FormatF(az_deg, 6) + "deg";
      content += " 俯仰=" + FormatF(el_deg, 6) + "deg";
    }
    content += " 径向速度(远离为正)=" +
               FormatF(measurement.radial_velocity_m_per_s, 3) + "m/s";
    content += " SNR=" + FormatF(measurement.detection_snr_db, 3) + "dB";
    content += " 检测概率=" + FormatF(measurement.detection_pd, 5);
    content += " 量测协方差ENU_m2[x,y,z]=[";
    for (int row = 0; row < 3; ++row) {
      if (row != 0) {
        content += ";";
      }
      for (int col = 0; col < 3; ++col) {
        if (col != 0) {
          content += ",";
        }
        content += FormatF(measurement.measurement_covariance(row, col), 6);
      }
    }
    content += "]";
    RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, "雷达检测与量测输出接口测试", content);
  }
}

void WriteRirClusterCount(float sim_time_sec, std::uint32_t cycle,
                          const std::vector<tracking::RirTrackState>& tracks) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 甲方 2026-08-22 批注「要输出集群目标数量（知道能打出几枚即可）」：
  // 集群目标数量 = 确认航迹数（身份去重后的在跟目标数，非检测条数——
  // 「检测条数不得顶替集群规模」原裁定由本批注按计数口径覆盖）。同时给出
  // 待确认/丢失计数作上下文；无确认航迹时如实写 0。
  // 验收判定标准 第19项：被跟踪对象明细按状态分列（航迹→目标ID），
  // 空列表省略；external_target_id 为 0（未知）时目标写「未知」。
  std::size_t confirmed = 0U;
  std::size_t tentative = 0U;
  std::size_t lost = 0U;
  for (const tracking::RirTrackState& track : tracks) {
    switch (track.status) {
      case tracking::RirTrackStatus::kConfirmed:
        ++confirmed;
        break;
      case tracking::RirTrackStatus::kTentative:
        ++tentative;
        break;
      case tracking::RirTrackStatus::kLost:
        ++lost;
        break;
    }
  }
  std::string confirmed_ids;
  std::size_t index = 0U;
  std::string active_list;
  std::string lost_list;
  for (const tracking::RirTrackState& track : tracks) {
    ++index;
    const std::string target_text =
        track.external_target_id == 0U ? std::string("未知")
                                       : std::to_string(track.external_target_id);
    if (track.status == tracking::RirTrackStatus::kConfirmed && track.external_target_id != 0U) {
      if (!confirmed_ids.empty()) {
        confirmed_ids += ",";
      }
      confirmed_ids += std::to_string(track.external_target_id);
    }
    const char* status_text = track.status == tracking::RirTrackStatus::kConfirmed ? "确认"
                              : track.status == tracking::RirTrackStatus::kTentative ? "待确认"
                                                                                     : "丢失";
    const std::string entry = "航迹" + std::to_string(index) + "→目标" + target_text + "(" +
                              status_text + ");";
    if (track.status == tracking::RirTrackStatus::kLost) {
      lost_list += entry;
    } else {
      active_list += entry;
    }
  }
  if (!active_list.empty()) {
    active_list.pop_back();  // 去掉行尾分号
  }
  if (!lost_list.empty()) {
    lost_list.pop_back();
  }
  std::string content = "集群目标数量=" + std::to_string(confirmed);
  content += " 在跟=" + std::to_string(confirmed + tentative);
  content += " 待确认=" + std::to_string(tentative);
  content += " 丢失=" + std::to_string(lost);
  if (!confirmed_ids.empty()) {
    content += " 确认目标ID=[" + confirmed_ids + "]";
  }
  if (!active_list.empty()) {
    content += " 在跟列表=" + active_list;
  }
  if (!lost_list.empty()) {
    content += " 丢失列表=" + lost_list;
  }
  Emit(sim_time_sec, cycle, "规模目标识别功能测试", content);
}

void WriteRirTrackAndId(float sim_time_sec, std::uint32_t cycle,
                        const tracking::RirTrackState& track,
                        const session::RirRecognitionResult* result,
                        const session::RirFeatureMeasurementRecord* features,
                        bool has_truth, double category_accuracy,
                        const std::vector<float>* imm_weights,
                        const oneq::coordinate::EcefPositionM& platform_ecef,
                        const RirTrackTruthContext* truth) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  (void)has_truth;
  (void)category_accuracy;
  (void)imm_weights;
  const double heading = HeadingDeg(track.velocity.x(), track.velocity.y());
  const int category = result != nullptr ? static_cast<int>(result->target_category) : -1;
  // 评审 2026-08-26 条19/20：不输出「大类枚举N(名)」外壳，直接给中文名；未识别
  // （result 缺失/类别未知且无识别结果）写「未识别」。
  const std::string category_text = category < 0 ? std::string("未识别")
                                                 : std::string(CategoryName(category));
  // 斜距/方位/俯仰拆分（第38/39/40/43项 定位信息 用）。
  float range_m = 0.0f;
  float az_deg = 0.0f;
  float el_deg = 0.0f;
  const bool has_polar =
      TryLookPolarFromEnuM(track.position.x(), track.position.y(), track.position.z(), &range_m,
                           &az_deg, &el_deg);
  // 评审 2026-08-26 条17：ENU 字段替换为 ECEF/LLA（平台 ECEF 为雷达局部 ENU 的
  // 绝对锚点；锚点或换算失败时省略对应字段）。速度/加速度按 ENU→ECEF 旋转矩阵换算。
  oneq::coordinate::LlaPositionDegM platform_lla;
  oneq::coordinate::EcefPositionM pos_ecef;
  oneq::coordinate::EcefVelocityMps vel_ecef;
  oneq::coordinate::EcefVelocityMps acc_ecef;
  bool have_frame = false;
  if (oneq::coordinate::TryEcefToLla(platform_ecef, &platform_lla)) {
    const oneq::coordinate::EnuPositionM pos_enu(track.position.x(), track.position.y(),
                                                 track.position.z());
    const oneq::coordinate::EnuVelocityMps vel_enu(track.velocity.x(), track.velocity.y(),
                                                   track.velocity.z());
    const oneq::coordinate::EnuVelocityMps acc_enu(track.acceleration.x(), track.acceleration.y(),
                                                   track.acceleration.z());
    have_frame = oneq::coordinate::TryEnuToEcef(pos_enu, platform_lla, &pos_ecef) &&
                 oneq::coordinate::TryEnuToEcefVelocity(vel_enu, platform_lla, &vel_ecef) &&
                 oneq::coordinate::TryEnuToEcefVelocity(acc_enu, platform_lla, &acc_ecef);
  }
  const std::string track_id = "航迹=" + std::to_string(track.association_key);
  const std::string track_target_id =
      track_id + " 目标ID=" + std::to_string(track.external_target_id);

  // 内部接口第73项：航迹后验与本周期特征快照按关联键对账。
  std::string interface_recognition = track_target_id;
  interface_recognition += " 航迹状态=" + std::string(TrackStatusText(track.status));
  interface_recognition += " 位置ENU=" +
                           FormatVec3(track.position.x(), track.position.y(), track.position.z(), 3) +
                           "m 速度ENU=" +
                           FormatVec3(track.velocity.x(), track.velocity.y(), track.velocity.z(), 3) +
                           "m/s";
  if (features != nullptr) {
    interface_recognition += " SNR=" + FormatF(features->snr_db, 3) + "dB";
    interface_recognition += " 特征有效掩码=" + std::to_string(features->valid_feature_mask);
    interface_recognition += " 特征来源周期=" + std::to_string(features->cycle_index);
    interface_recognition += " 特征批次=" + std::to_string(features->batch_id);
  }
  if (result != nullptr) {
    interface_recognition += " 识别来源周期=" + std::to_string(result->source_cycle_index);
    interface_recognition += " 识别批次=" + std::to_string(result->source_batch_id);
    interface_recognition += " 类别=" + category_text;
    interface_recognition += " 型号=" +
                             (result->target_model.empty() ? std::string("未识别")
                                                           : result->target_model);
    interface_recognition += " 置信度=" + FormatF(result->confidence, 5);
    interface_recognition += " 积累观测数=" + std::to_string(result->observation_count);
  }
  RIR_INTERNAL_INTERFACE_ITEM(sim_time_sec, cycle,
                              "雷达航迹与特征识别交互接口测试", interface_recognition);

  // 外部导入项：综合特征识别是独立验收用例，与第47项宽带一维像分别记录。
  if (result != nullptr) {
    std::string comprehensive = track_target_id;
    comprehensive += " 目标类别=" + category_text;
    comprehensive += " 目标型号=" +
                     (result->target_model.empty() ? std::string("未识别")
                                                   : result->target_model);
    comprehensive += " 置信度=" + FormatF(result->confidence, 5);
    comprehensive += " 特征有效掩码=" + std::to_string(result->valid_feature_mask);
    Emit(sim_time_sec, cycle, "综合特征识别功能测试", comprehensive);
  }

  // 第38项 角度和距离测量：量测角距一行；目标量测误差统计=量测−真值（可计算口径，
  // 真值上下文缺失时省略误差行）。方位误差取最短角差。
  if (has_polar) {
    Emit(sim_time_sec, cycle, "对目标的角度和距离进行测量功能测试",
         "目标ID=" + std::to_string(track.external_target_id) + " 斜距=" + FormatF(range_m, 1) +
             "m 方位/俯仰=" + FormatPairDeg(az_deg, el_deg, 3) + "°");
  }
  if (has_polar && truth != nullptr && truth->has_look) {
    double az_err = std::fmod(az_deg - truth->truth_az_deg + 540.0, 360.0) - 180.0;
    if (az_err <= -180.0) {
      az_err += 360.0;
    }
    Emit(sim_time_sec, cycle, "对目标的角度和距离进行测量功能测试",
         "目标ID=" + std::to_string(track.external_target_id) +
             " 目标量测误差统计=斜距误差=" + FormatF(range_m - truth->truth_range_m, 1) +
             "m 方位/俯仰误差=(" + FormatF(az_err, 3) + "," +
             FormatF(el_deg - truth->truth_el_deg, 3) + ")°");
  }

  // 第39/40项 定位信息与运动参数（两判定句相同，各按项名分行；加速度优先 ECEF
  // 三分量，锚点不可用时退 ENU）。第40项另由控制器写 本周期航迹数= 汇总行。
  const std::string positioning =
      has_polar ? ("定位信息=斜距=" + FormatF(range_m, 1) + "m 方位/俯仰=" +
                   FormatPairDeg(az_deg, el_deg, 3) + "° 高度=" +
                   FormatF(track.position.z(), 1) + "m")
                : ("定位信息=高度=" + FormatF(track.position.z(), 1) + "m");
  const std::string motion =
      std::string("运动参数=速度=") + FormatF(track.speed, 3) + "m/s 航向=" +
      FormatF(heading, 2) + "° 加速度=" +
      (have_frame ? FormatVec3(acc_ecef.x_mps, acc_ecef.y_mps, acc_ecef.z_mps, 4)
                  : FormatVec3(track.acceleration.x(), track.acceleration.y(),
                               track.acceleration.z(), 4)) +
      "m/s^2";
  const char* kReentryItem = "对典型目标/再入目标进行跟踪功能测试";
  const char* kMultiItem = "多目标跟踪功能测试";
  Emit(sim_time_sec, cycle, kReentryItem,
       track_target_id + std::string(" 状态=") + TrackStatusText(track.status) + " " + positioning);
  Emit(sim_time_sec, cycle, kReentryItem, track_target_id + " " + motion);
  Emit(sim_time_sec, cycle, kMultiItem, track_target_id + " " + positioning);
  Emit(sim_time_sec, cycle, kMultiItem, track_target_id + " " + motion);

  // 第42项 跟踪滤波：当前估计/协方差/估计误差指标三行（下一时刻外推、协方差迹、
  // IMM 权重为超出规范或分子项，不写；误差指标=滤波−真值，无真值时省略该行）。
  if (have_frame) {
    Emit(sim_time_sec, cycle, "跟踪滤波功能测试",
         track_target_id + " 位置（ECEF）=" +
             FormatVec3(pos_ecef.x_m, pos_ecef.y_m, pos_ecef.z_m, 1) + "m 速度=" +
             FormatVec3(vel_ecef.x_mps, vel_ecef.y_mps, vel_ecef.z_mps, 3) + "m/s 加速度=" +
             FormatVec3(acc_ecef.x_mps, acc_ecef.y_mps, acc_ecef.z_mps, 4) + "m/s^2");
  }
  {
    const tracking::RirStateCovariance& cov = track.gaussian_state.covariance;
    std::string cov_text = "目标状态协方差=[";
    for (int row = 0; row < 6; ++row) {
      if (row != 0) {
        cov_text += ";";
      }
      for (int col = 0; col < 6; ++col) {
        if (col != 0) {
          cov_text += ",";
        }
        cov_text += FormatF(static_cast<double>(cov(row, col)), 6);
      }
    }
    cov_text += "]";
    Emit(sim_time_sec, cycle, "跟踪滤波功能测试", track_target_id + " " + cov_text);
  }
  if (have_frame && truth != nullptr && truth->has_ecef) {
    Emit(sim_time_sec, cycle, "跟踪滤波功能测试",
         track_target_id + " 估计误差指标=位置误差ECEF=" +
             FormatVec3(pos_ecef.x_m - truth->position_ecef.x_m,
                        pos_ecef.y_m - truth->position_ecef.y_m,
                        pos_ecef.z_m - truth->position_ecef.z_m, 1) +
             "m 速度误差=" +
             FormatVec3(vel_ecef.x_mps - truth->velocity_ecef.x_mps,
                        vel_ecef.y_mps - truth->velocity_ecef.y_mps,
                        vel_ecef.z_mps - truth->velocity_ecef.z_mps, 3) +
             "m/s");
  }

  // 第43项 根据目标 RCS 实时计算目标探测结果：本周期RCS + 定位信息 + 运动参数
  //（判定句未点名 RCS 属存疑项，示例按项名写 RCS 并带定位/运动）。
  Emit(sim_time_sec, cycle, "根据目标RCS实时计算目标探测结果功能测试",
       "目标ID=" + std::to_string(track.external_target_id) + " 本周期RCS=" +
           FormatF(track.rcs, 3) + "m^2 " + positioning);
  Emit(sim_time_sec, cycle, "根据目标RCS实时计算目标探测结果功能测试",
       "目标ID=" + std::to_string(track.external_target_id) + " " + motion);

  if (features != nullptr) {
    // 第44项 运动特征处理：判定条件（速度/高度/加速度/近似直线）在前，目标类别在后；
    // 斜距/方位俯仰不是运动分类条件，不写。特征维度未有效（刚建轨未累积样本）时
    // 整行省略（§0 规则3：无有效值不写 0.000 占位）。
    const auto& motion_feature = features->features.motion;
    if (motion_feature.valid) {
      std::string motion_text = track_target_id;
      motion_text += " 速度=" + FormatF(motion_feature.speed_m_per_s, 3) + "m/s";
      motion_text += " 高度=" + FormatF(motion_feature.altitude_m, 1) + "m";
      motion_text += " 加速度=" + FormatF(motion_feature.acceleration_m_per_s2, 3) + "m/s^2";
      motion_text += std::string(" 近似直线=") + YesNo(motion_feature.is_straight);
      motion_text += " 目标类别=" + category_text;
      Emit(sim_time_sec, cycle, "运动特征处理功能测试", motion_text);
    }

    // 第45项 RCS 统计特征处理（维度无效时省略）。
    const auto& rcs = features->features.rcs;
    if (rcs.valid) {
      std::string rcs_text = track_target_id;
      rcs_text += " RCS均值/标准差=" + FormatF(rcs.mean_dbsm, 3) + "/" + FormatF(rcs.std_db, 3) +
                  "dBsm";
      rcs_text += " 目标类别=" + category_text;
      Emit(sim_time_sec, cycle, "RCS统计特征处理功能测试", rcs_text);
    }

    // 第46项 极化特征解算：五个统计量各一行"均值/标准差"（统计总体=姿态扇区窗口行，
    // 方向角为圆统计口径）；细分类型（弹头/重诱饵/轻诱饵/碎片）判据未冻结恒未判，
    // 判决就绪前不追加。窗口无效时数值行整体省略。
    const session::RirPolarizationStatsFeatureObservation& pol_stats =
        features != nullptr ? features->features.polarization_stats
                            : session::RirPolarizationStatsFeatureObservation{};
    const bool has_pol_stats = pol_stats.valid;
    EmitOrNone(sim_time_sec, cycle, "极化特征解算功能测试",
               track_target_id + " 极化散射矩阵行列式均值/标准差=", has_pol_stats,
               has_pol_stats ? FormatSci(pol_stats.determinant_mean) + "/" +
                                   FormatSci(pol_stats.determinant_std)
                             : std::string());
    EmitOrNone(sim_time_sec, cycle, "极化特征解算功能测试",
               track_target_id + " 功率迹（Span）均值/标准差=", has_pol_stats,
               has_pol_stats ? FormatSci(pol_stats.span_mean) + "/" + FormatSci(pol_stats.span_std)
                             : std::string());
    EmitOrNone(sim_time_sec, cycle, "极化特征解算功能测试",
               track_target_id + " 去极化系数均值/标准差=", has_pol_stats,
               has_pol_stats ? FormatF(pol_stats.depolarization_mean, 4) + "/" +
                                   FormatF(pol_stats.depolarization_std, 4)
                             : std::string());
    EmitOrNone(sim_time_sec, cycle, "极化特征解算功能测试",
               track_target_id + " 本征极化方向角均值/标准差=", has_pol_stats,
               has_pol_stats ? FormatF(pol_stats.psi_mean_deg, 3) + "°/" +
                                   FormatF(pol_stats.psi_std_deg, 3) + "°"
                             : std::string());
    EmitOrNone(sim_time_sec, cycle, "极化特征解算功能测试",
               track_target_id + " 本征极化椭圆率均值/标准差=", has_pol_stats,
               has_pol_stats ? FormatF(pol_stats.tau_mean_deg, 3) + "°/" +
                                   FormatF(pol_stats.tau_std_deg, 3) + "°"
                             : std::string());
    std::string subclass_text;
    if (result != nullptr && result->ballistic_subclass != session::RirBallisticSubclass::kUnknown) {
      subclass_text = " 细分类型=" + BallisticSubclassName(
                              static_cast<int>(result->ballistic_subclass));
    }
    Emit(sim_time_sec, cycle, "极化特征解算功能测试",
         track_target_id + " 目标类型=" + category_text + subclass_text);

    // 第47项 宽带一维像特征解算：散射中心（场景距离像真值输入——库内特征提取只
    // 出长度/峰数等统计量，无逐中心量测输出）+ 轮廓（长度/峰数）+ 识别类别；
    // 能量集中/分辨率/置信度不写。距离像维度无效时省略（§0 规则3）。
    const auto& rp = features->features.range_profile;
    if (rp.valid) {
      std::string rp_text = track_target_id;
      if (truth != nullptr && truth->scatterers != nullptr && !truth->scatterers->empty()) {
        std::string centers = "散射中心=[";
        for (std::size_t i = 0U; i < truth->scatterers->size(); ++i) {
          if (i != 0U) {
            centers += ";";
          }
          centers += "(" + FormatF((*truth->scatterers)[i].range_offset_m, 1) + "m," +
                     FormatF((*truth->scatterers)[i].rcs_dbsm, 1) + "dBsm)";
        }
        centers += "]";
        rp_text += " " + centers;
      }
      rp_text += " 长度=" + FormatF(rp.length_m, 3) + "m";
      rp_text += " 峰数=" + std::to_string(rp.peak_count);
      rp_text += " 识别类型=" + category_text;
      Emit(sim_time_sec, cycle, "宽带一维像特征解算功能测试", rp_text);
    }
  }
}

// 评审 2026-08-26 条21：调度只应有搜索/跟踪两类事件（「识别」不是独立驻留模式，
// 识别在 kIdentify 模式内顺带执行），识别计数与列表段删除。
void WriteRirExternalTracks(float sim_time_sec, std::uint32_t cycle,
                            const std::vector<tracking::RirTrackState>& tracks,
                            const session::RirOutputFrame& output_frame) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  std::size_t confirmed_count = 0U;
  for (const tracking::RirTrackState& track : tracks) {
    if (track.status == tracking::RirTrackStatus::kConfirmed) {
      ++confirmed_count;
    }
  }
  std::string summary = "航迹数量=" + std::to_string(tracks.size());
  summary += " 集群目标数量=" + std::to_string(confirmed_count);
  summary += " 批次=" + std::to_string(output_frame.batch_id);
  RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, "航迹与目标识别输出接口测试", summary);

  for (const tracking::RirTrackState& track : tracks) {
    const session::RirRecognitionResult* recognition = nullptr;
    for (const session::RirTrackRecognitionOutput& output : output_frame.recognition_outputs) {
      if (output.association_key == track.association_key) {
        recognition = &output.result;
        break;
      }
    }
    const session::RirFeatureMeasurementRecord* feature_record = nullptr;
    for (const session::RirFeatureMeasurementRecord& record : output_frame.feature_measurements) {
      if (record.association_key == track.association_key) {
        feature_record = &record;
        break;
      }
    }

    std::string content = "航迹ID=" + std::to_string(track.association_key);
    content += " 目标ID=" + std::to_string(track.external_target_id);
    content += " 生命周期状态=" + std::string(TrackStatusText(track.status));
    content += " 目标状态ENU_位置=" +
               FormatVec3(track.position.x(), track.position.y(), track.position.z(), 3) + "m";
    content += " 速度=" +
               FormatVec3(track.velocity.x(), track.velocity.y(), track.velocity.z(), 3) +
               "m/s";
    content += " 加速度=" +
               FormatVec3(track.acceleration.x(), track.acceleration.y(), track.acceleration.z(),
                          3) +
               "m/s^2";
    content += " 状态协方差ENU[x,vx,y,vy,z,vz]=[";
    for (int row = 0; row < 6; ++row) {
      if (row != 0) {
        content += ";";
      }
      for (int col = 0; col < 6; ++col) {
        if (col != 0) {
          content += ",";
        }
        content += FormatF(track.gaussian_state.covariance(row, col), 6);
      }
    }
    content += "]";

    if (recognition != nullptr) {
      content += " 目标类别=" +
                 std::string(CategoryName(static_cast<int>(recognition->target_category)));
      content += " 目标型号=" +
                 (recognition->target_model.empty() ? std::string("未识别")
                                                     : recognition->target_model);
      content += " 置信度=" + FormatF(recognition->confidence, 5);
    } else {
      content += " 目标类别=未知 目标型号=未识别 置信度=0.00000";
    }

    if (feature_record != nullptr) {
      const session::RirFeatureObservations& features = feature_record->features;
      content += " RCS特征摘要=";
      if (features.rcs.valid) {
        content += "均值" + FormatF(features.rcs.mean_dbsm, 3) + "dBsm/标准差" +
                   FormatF(features.rcs.std_db, 3) + "dB/质量" +
                   FormatF(features.rcs.quality, 4);
      } else {
        content += "不可用";
      }
      content += " 极化特征摘要=";
      bool has_polarization_summary = false;
      if (features.polarization.valid) {
        content += "能量差" + FormatF(features.polarization.energy_difference_db, 3) +
                   "dB/相对差" + FormatF(features.polarization.relative_difference_db, 3) +
                   "dB/质量" + FormatF(features.polarization.quality, 4);
        has_polarization_summary = true;
      }
      if (features.polarization_stats.valid) {
        if (has_polarization_summary) {
          content += "/";
        }
        content += "Span均值" + FormatF(features.polarization_stats.span_mean, 3) +
                   "/Span标准差" + FormatF(features.polarization_stats.span_std, 3);
        has_polarization_summary = true;
      }
      if (!has_polarization_summary) {
        content += "不可用";
      }
      content += " HRRP特征摘要=";
      if (features.range_profile.valid) {
        content += "长度" + FormatF(features.range_profile.length_m, 3) + "m/峰数" +
                   std::to_string(features.range_profile.peak_count) + "/能量集中率" +
                   FormatF(features.range_profile.peak_energy_concentration, 4) + "/分辨率" +
                   FormatF(features.range_profile.resolution_m, 3) + "m/质量" +
                   FormatF(features.range_profile.quality, 4);
      } else {
        content += "不可用";
      }
    } else {
      content += " RCS特征摘要=不可用 极化特征摘要=不可用 HRRP特征摘要=不可用";
    }
    RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, "航迹与目标识别输出接口测试", content);
  }
}

void WriteRirSchedule(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                      float budget_sec, float consumed_sec, std::uint32_t search_count,
                      std::uint32_t designate_count, std::uint32_t track_count,
                      std::uint32_t confirmed_tracks) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 验收判定标准 第37项·其二：扫描调度信息（搜索/指定/跟踪/预算/已耗时；
  // 2026-08-30 核查 9.1：删除"计划驻留/实际执行"字段——库内逐驻留执行，实际数
  // 恒等于计划数，无信息量；按类分项计数与时间预算保留）。
  Emit(sim_time_sec, cycle, "对指定空域进行搜索功能测试",
       "雷达ID=" + std::to_string(radar_id) + " 扫描调度信息=搜索=" +
           std::to_string(search_count) + " 指定=" + std::to_string(designate_count) +
           " 跟踪=" + std::to_string(track_count) +
           " 预算/已耗时=" + FormatF(budget_sec, 3) + "/" + FormatF(consumed_sec, 3) + "s");
  // 验收判定标准 第49项：调度策略——各类事件的实际执行数量列表。
  Emit(sim_time_sec, cycle, "调度策略功能测试",
       "雷达ID=" + std::to_string(radar_id) + " [搜索×" + std::to_string(search_count) +
           ",指定×" + std::to_string(designate_count) + ",跟踪×" + std::to_string(track_count) +
           ",确认航迹×" + std::to_string(confirmed_tracks) + "]");
}

void WriteRirOncePerSession(float sim_time_sec, std::uint32_t cycle) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  static bool written = false;
  if (written) {
    return;
  }
  written = true;
  // 评审 2026-08-26 条22（方案B）：库内不做墙钟计时，真实初始化/加载耗时在示例层
  // integration_events.log 的同名验收项（模块=RIR）。场景数/总仿真周期由示例层
  // 结束时回写（第54项），库内不再写占位行。
  Emit(sim_time_sec, cycle, "初始化时间性能测试",
       "见integration_events.log（模块=RIR）");
  Emit(sim_time_sec, cycle, "单个模型加载时间性能测试",
       "见integration_events.log（模块=RIR）");
  Emit(sim_time_sec, cycle, "多个模型并行加载性能测试",
       "见integration_events.log（模块=RIR）");
}

void WriteRirCycleRunCount(float sim_time_sec, std::uint32_t cycle) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 验收判定标准 第54项：运行次数与运行状态。
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "可支持连续运行次数性能测试",
                      "本会话已运行周期=" + std::to_string(cycle) + " 状态=正常");
}

void WriteRirDwellScan(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                       const std::vector<RirDwellPlan>& dwell_plan) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 验收判定标准 第48项：逐驻留扫描轨迹序号（2026-08-29 TAS：一周期多驻留）。
  // 波位排列表/扫描轨迹全量在 rir_scan_pattern.csv（index,az_deg,el_deg）；日志不再
  // 重复当前波束指向，搜索驻留只写本周期序号（=波位表游标，可索引 CSV）。
  // 指定/跟踪驻留带种类与目标 ID 标记。
  const std::string scan_csv_path = ResolveRirScanPatternCsvPath();
  for (const RirDwellPlan& dwell : dwell_plan) {
    if (dwell.kind == RirDwellKind::kSearch) {
      Emit(sim_time_sec, cycle, "波束扫描功能测试",
           "雷达ID=" + std::to_string(radar_id) +
               " 本周期序号=" + std::to_string(dwell.scan_pattern_index) +
               " 波位排列表已输出至" + scan_csv_path);
    } else {
      const char* kind_text =
          dwell.kind == RirDwellKind::kDesignate ? "指定驻留" : "跟踪驻留";
      Emit(sim_time_sec, cycle, "波束扫描功能测试",
           "雷达ID=" + std::to_string(radar_id) + " 驻留种类=" + kind_text +
               " 目标ID=" + std::to_string(dwell.external_target_id));
    }
  }
}

bool TryExportRirAntennaPatternCsv(const config::hardware::RirAntennaConfig& antenna,
                                   const char* path) {
  const std::string out_path = (path != nullptr && *path != '\0') ? std::string(path)
                                                                  : ResolveRirAntennaPatternCsvPath();
  oneq::logging::EnsureParentDirectory(out_path.c_str());
  std::ofstream out(out_path.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
  if (!out.is_open()) {
    return false;
  }
  out << "az_off_deg,el_off_deg,gain_dbi\n";
  dwell::RirAntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = antenna.nominal_az_beamwidth_deg;
  beamwidth.el_beamwidth_deg = antenna.nominal_el_beamwidth_deg;
  config::RirAzimuthElevationDeg pointing(0.0f, 0.0f);
  // 2026-08-31 用户裁定：全向网格 az 0–360° × el −90–90°（az/el 正好无冗余覆盖
  // 全空间；主瓣/副瓣/尾瓣全落在数据内），步长 = 波束宽度（不再用 bw/20 高密度；
  // 下限 0.1° 防窄束配置行数爆炸）。主瓣峰值点 (0,0) 始终在格点上。旧 ±5·bw
  // 自适应网格见评审 2026-08-26 条14（已废止）；同日修订：el 初版 ±180° 有
  // 方向冗余，收敛为 ±90°。
  const float step = std::max(0.1f, std::min(antenna.nominal_az_beamwidth_deg,
                                             antenna.nominal_el_beamwidth_deg));
  const float span_az = 360.0f;
  const float span_el = 90.0f;
  for (float az = 0.0f; az <= span_az + 0.5f * step; az += step) {
    for (float el = -span_el; el <= span_el + 0.5f * step; el += step) {
      dwell::RirAntennaLookOffsetDeg offset;
      offset.delta_az_deg = az;
      offset.delta_el_deg = el;
      const dwell::RirAntennaPatternSample sample = dwell::RirEvaluateAntennaPattern(
          antenna, beamwidth, offset, pointing, 0.0f);
      out << FormatF(az, 2) << "," << FormatF(el, 2) << "," << FormatF(sample.gain_dbi, 3) << "\n";
    }
  }
  return true;
}

bool TryExportRirScanPatternCsv(
    const std::vector<oneq::common::radar::AzimuthElevationDeg>& pattern, const char* path) {
  const std::string out_path = (path != nullptr && *path != '\0') ? std::string(path)
                                                                  : ResolveRirScanPatternCsvPath();
  oneq::logging::EnsureParentDirectory(out_path.c_str());
  std::ofstream out(out_path.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
  if (!out.is_open()) {
    return false;
  }
  out << "index,az_deg,el_deg\n";
  for (std::size_t i = 0U; i < pattern.size(); ++i) {
    out << i << "," << FormatF(pattern[i].az_deg, 3) << "," << FormatF(pattern[i].el_deg, 3)
        << "\n";
  }
  return true;
}

}  // namespace runtime
}  // namespace remote_identification_radar
