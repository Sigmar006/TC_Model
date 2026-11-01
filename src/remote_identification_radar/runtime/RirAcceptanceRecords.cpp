/**
 * @file RirAcceptanceRecords.cpp
 * @brief RIR 验收行与天线三维增益 CSV。
 */

#include "remote_identification_radar/runtime/RirAcceptanceRecords.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <numeric>
#include <set>
#include <unordered_map>
#include <string>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/remote_identification_radar/session/RirFeatureMeasurementTypes.h"
#include "common/acceptance/AcceptanceSourceStage.h"
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
#define ONEQ_RIR_ANTENNA_PATTERN_CSV_PATH "log/RadarAntennaGainPatterndata.csv"
#endif
#ifndef ONEQ_RIR_SCAN_PATTERN_CSV_PATH
#define ONEQ_RIR_SCAN_PATTERN_CSV_PATH "log/Rir_scan_pattern.csv"
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

bool SameIdentity(const oneq::electromagnetics::RfEmissionIdentity& left,
                  const oneq::electromagnetics::RfEmissionIdentity& right) {
  return left.sensor_entity_id == right.sensor_entity_id &&
         left.emission_id == right.emission_id;
}

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

/** @brief 方向图模型中文名（第28项日志的「模型=」字段；未识别返回「未知模型」）。 */
const char* AntennaPatternModelText(config::hardware::RirAntennaPatternModelType model) {
  using config::hardware::RirAntennaPatternModelType;
  switch (model) {
    case RirAntennaPatternModelType::kGaussianMainLobe:
      return "高斯主瓣";
    case RirAntennaPatternModelType::kCosinePower:
      return "余弦幂";
    case RirAntennaPatternModelType::kSincPattern:
      return "辛克";
    default:
      return "未知模型";
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

/** @brief dB 域高斯标准差对应的线性域变异系数。 */
double RcsFluctuationCoefficient(double std_db) {
  const double log_std = std::max(0.0, std_db) * kLn10 / 10.0;
  return std::sqrt(std::expm1(log_std * log_std));
}

/** @brief 最佳候选综合得分中单个特征维度的归一化加权贡献。 */
bool TryRecognitionContribution(
    const session::RirRecognitionResult* result,
    const config::RirRecognitionFeatureWeights& weights,
    session::RirRecognitionFeatureDimension dimension, double* contribution) {
  if (result == nullptr || contribution == nullptr || result->best_score <= 0.0f) {
    return false;
  }
  const session::RirRecognitionFeatureScores& scores = result->feature_scores;
  const double base_weights[] = {weights.rcs_weight, weights.motion_weight,
                                 weights.polarization_weight, weights.range_profile_weight};
  const double qualities[] = {scores.rcs_quality, scores.motion_quality,
                              scores.polarization_quality, scores.range_profile_quality};
  const double similarities[] = {scores.rcs_similarity, scores.motion_similarity,
                                  scores.polarization_similarity,
                                  scores.range_profile_similarity};
  const std::uint8_t bits[] = {
      static_cast<std::uint8_t>(session::RirRecognitionFeatureDimension::kRcs),
      static_cast<std::uint8_t>(session::RirRecognitionFeatureDimension::kMotion),
      static_cast<std::uint8_t>(session::RirRecognitionFeatureDimension::kPolarization),
      static_cast<std::uint8_t>(session::RirRecognitionFeatureDimension::kRangeProfile)};
  const std::uint8_t requested_bit = static_cast<std::uint8_t>(dimension);
  double denominator = 0.0;
  int requested_index = -1;
  for (int i = 0; i < 4; ++i) {
    if ((result->valid_feature_mask & bits[i]) != 0U && qualities[i] > 0.0) {
      denominator += base_weights[i] * qualities[i];
    }
    if (bits[i] == requested_bit) {
      requested_index = i;
    }
  }
  if (requested_index < 0 || denominator <= 0.0 ||
      (result->valid_feature_mask & requested_bit) == 0U ||
      qualities[requested_index] <= 0.0) {
    return false;
  }
  *contribution = base_weights[requested_index] * qualities[requested_index] *
                  similarities[requested_index] / denominator;
  return std::isfinite(*contribution);
}

/** @brief 统计特征有效位图里置位的维度数（识别判据诊断用）。 */
std::uint32_t CountBits(std::uint32_t mask) {
  std::uint32_t count = 0U;
  while (mask != 0U) {
    count += (mask & 1U);
    mask >>= 1U;
  }
  return count;
}

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

const char* RfWaveformKindText(oneq::electromagnetics::RfSceneWaveformKind kind) {
  using oneq::electromagnetics::RfSceneWaveformKind;
  switch (kind) {
    case RfSceneWaveformKind::kContinuous:
      return "连续波";
    case RfSceneWaveformKind::kPulseTrain:
      return "脉冲列";
    case RfSceneWaveformKind::kLinearSweep:
      return "线性扫频";
    case RfSceneWaveformKind::kBandLimitedNoise:
      return "带限噪声";
    default:
      return "未知";
  }
}

const char* WorkModeText(config::RirWorkMode mode) {
  return mode == config::RirWorkMode::kIdentify ? "识别" : "待机";
}

const char* DwellKindText(RirDwellKind kind) {
  switch (kind) {
    case RirDwellKind::kDesignate:
      return "指定";
    case RirDwellKind::kTrack:
    case RirDwellKind::kExternalTrack:
      // 中译：库内跟踪驻留与外部跟踪驻留合并为"驻留"（2026-10-22 用户要求统一命名）——
      //       两者都是对已知目标的照射，只是指向来源不同（库内航迹预测 / 外部引导窗口）。
      // 标识：第 66 项波束任务队列措辞合并。
      return "驻留";
    case RirDwellKind::kSearch:
    default:
      return "搜索";
  }
}

/**
 * @brief 扫描起始象限的日志文本：`枚举名(数值)`。
 *
 * @details 判定标准第 48 项要求记录"扫描起始象限 KLeftTop"这类**配置输入**；用
 *   `名称(数值)` 同时满足人读与机器解析（外部验证程序可按数值比对）。
 *   枚举定义见 `include/1q/foundation/scan_schedule_types.h:17-22`。
 * @param[in] position 起始象限。
 * @return 形如 `kLeftTop(0)` 的静态字符串；未知值返回 `未知(-1)`。
 */
std::string ScanStartPositionText(oneq::foundation::ScanStartPosition position) {
  switch (position) {
    case oneq::foundation::ScanStartPosition::kLeftTop:
      return "kLeftTop(0)";
    case oneq::foundation::ScanStartPosition::kRightTop:
      return "kRightTop(1)";
    case oneq::foundation::ScanStartPosition::kRightBottom:
      return "kRightBottom(2)";
    case oneq::foundation::ScanStartPosition::kLeftBottom:
      return "kLeftBottom(3)";
    default:
      return "未知(-1)";
  }
}

/**
 * @brief 二维扫描推进顺序的日志文本：`枚举名(数值)`（见 `scan_schedule_types.h:27-29`）。
 * @param[in] sequence 推进顺序。
 * @return 形如 `kAzimuthFirst(0)` 的静态字符串；未知值返回 `未知(-1)`。
 */
std::string ScanSequenceText(oneq::foundation::ScanSequence sequence) {
  switch (sequence) {
    case oneq::foundation::ScanSequence::kAzimuthFirst:
      return "kAzimuthFirst(0)";
    case oneq::foundation::ScanSequence::kElevationFirst:
      return "kElevationFirst(1)";
    default:
      return "未知(-1)";
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
  // 中译：把本拍检测单元的相干积累脉冲数传给旁路，使 `MTD增益` 输出**真实相干增益**
  //   （10log10(N) + 跨 bin 损耗），而不是仅归一化 bin 响应（恒 ≤0）。
  // 标识：MTD 真实增益（2026-10-22）。
  bank_input.coherent_pulse_count = input.cell.effective_pulse_count;
  if (!tones.empty()) {
    bank_input.tones = tones.data();
    bank_input.tone_count = tones.size();
  }
  return oneq::common::radar::TryResolveMtiMtdAcceptanceBank(bank_input, bank);
}

/**
 * @brief 反解达到目标检测概率所需的单脉冲 SNR (dB)。
 *
 * 验收判定标准 第36项：由同一 Marcum Q 检测模型对 Pd 二分搜索（SNR 在 [-20, 60] dB
 * 上单调递增），不引入第二套检测模型。参数非法或区间内无解返回非有限值，调用方省略该行。
 */
double SolveRequiredSnrDb(double pfa, int num_pulses, double target_pd) {
  if (!std::isfinite(pfa) || pfa <= 0.0 || pfa >= 1.0 || num_pulses < 1 ||
      !std::isfinite(target_pd) || target_pd <= 0.0 || target_pd >= 1.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double low_probe_db = -20.0;
  const double high_probe_db = 60.0;
  const double pd_low = oneq::common::radar::RadarEquations::ComputeDetectionProbability(
      static_cast<float>(low_probe_db), static_cast<float>(pfa),
      oneq::common::radar::SwerlingModel::kSwerling0, num_pulses);
  const double pd_high = oneq::common::radar::RadarEquations::ComputeDetectionProbability(
      static_cast<float>(high_probe_db), static_cast<float>(pfa),
      oneq::common::radar::SwerlingModel::kSwerling0, num_pulses);
  if (!(pd_low <= target_pd && target_pd <= pd_high)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double low_db = low_probe_db;
  double high_db = high_probe_db;
  for (int i = 0; i < 60; ++i) {
    const double mid_db = 0.5 * (low_db + high_db);
    const double pd = oneq::common::radar::RadarEquations::ComputeDetectionProbability(
        static_cast<float>(mid_db), static_cast<float>(pfa),
        oneq::common::radar::SwerlingModel::kSwerling0, num_pulses);
    if (pd < target_pd) {
      low_db = mid_db;
    } else {
      high_db = mid_db;
    }
  }
  return 0.5 * (low_db + high_db);
}

void Emit(float sim_time_sec, std::uint32_t cycle, const char* item, const std::string& content) {
  std::string missing_marker;
  missing_marker.push_back(static_cast<char>(-25));
  missing_marker.push_back(static_cast<char>(-68));
  missing_marker.push_back(static_cast<char>(-70));
  missing_marker.push_back(static_cast<char>(-27));
  missing_marker.push_back(static_cast<char>(-92));
  missing_marker.push_back(static_cast<char>(-79));
  if (content.empty() || content.find(missing_marker) != std::string::npos) {
    return;
  }
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

bool TryComputeDetectionRangeAtPd(double current_range_m, double current_snr_db,
                                  double pd_target, double pfa, int num_pulses,
                                  double* max_range_m) {
  if (max_range_m == nullptr || !std::isfinite(current_range_m) || current_range_m <= 0.0 ||
      !std::isfinite(current_snr_db) || !std::isfinite(pd_target)) {
    return false;
  }
  // 四次方根距离定律：RCS/增益/带宽等条件不变时 SNR ∝ 1/R⁴，故达到目标 Pd 所需 SNR
  // 对应的距离为 R = R_now · (SNR_now / SNR_req)^(1/4)。
  // 2026-09-28 复核修正：早期实现把比值写反（(SNR_req/SNR_now)^(1/4)），会出现
  // SNR 越高、最大距离反而越小的反向单调性。
  const double snr_required_db = SolveRequiredSnrDb(pfa, num_pulses, pd_target);
  if (!std::isfinite(snr_required_db)) {
    return false;
  }
  const double snr_ratio = std::pow(10.0, (current_snr_db - snr_required_db) / 10.0);
  const double computed_range_m = current_range_m * std::pow(snr_ratio, 0.25);
  if (!std::isfinite(computed_range_m) || computed_range_m <= 0.0) {
    return false;
  }
  *max_range_m = computed_range_m;
  return true;
}

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
  if (!oneq::logging::CurrentAcceptanceLogOutputDirectory().empty()) {
    const std::string& directory =
        oneq::logging::CurrentAcceptanceLogOutputDirectory();
    const char last = directory[directory.size() - 1U];
    return directory + ((last == '/' || last == '\\') ? "" : "/") +
           "RadarAntennaGainPatterndata.csv";
  }
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
  if (!oneq::logging::CurrentAcceptanceLogOutputDirectory().empty()) {
    const std::string& directory =
        oneq::logging::CurrentAcceptanceLogOutputDirectory();
    const char last = directory[directory.size() - 1U];
    return directory + ((last == '/' || last == '\\') ? "" : "/") +
           "Rir_scan_pattern.csv";
  }
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

void WriteRirAntennaPatternDerived(std::uint64_t radar_id, float sim_time_sec,
                                   std::uint32_t cycle,
                                   const config::hardware::RirAntennaConfig& antenna) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 验收判定标准 第28项（2026-09-28 按主文档补全）：从**实际生成的方向图采样**反算
  // 可配置特征，再与外部参数逐项比较，按 5% 判据给出各项与整体结果。
  // 2026-09-28 复核修正：峰值增益也必须从采样反算（早期实现直接取
  // antenna.main_beam_gain_db，误差恒为 0，属自证），此处按网格求采样最大值。
  // 反算量：峰值增益、方位/俯仰波束宽度（主瓣 -3 dB 宽度）、最大旁瓣电平、后瓣电平。
  dwell::RirAntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = antenna.nominal_az_beamwidth_deg;
  beamwidth.el_beamwidth_deg = antenna.nominal_el_beamwidth_deg;
  const config::RirAzimuthElevationDeg pointing(0.0f, 0.0f);
  const config::RirAzimuthElevationDeg boresight(antenna.boresight_offset_deg.az_deg,
                                                 antenna.boresight_offset_deg.el_deg);

  // 中译：反算与数据文件同用 0.1° 网格，保证"日志里的反算值"可用随行给出的
  //   RadarAntennaGainPatterndata.csv 逐点复核（2026-11-02 要求）。
  // 标识：方向图反算采样步长（2026-11-02）。
  const double step_deg = 0.1;
  const double half_span_deg =
      std::max(1.0, 2.0 * std::max(static_cast<double>(antenna.nominal_az_beamwidth_deg),
                                   static_cast<double>(antenna.nominal_el_beamwidth_deg)));

  auto evaluate_gain = [&](double delta_az_deg, double delta_el_deg) -> double {
    dwell::RirAntennaLookOffsetDeg offset;
    offset.delta_az_deg = static_cast<float>(delta_az_deg);
    offset.delta_el_deg = static_cast<float>(delta_el_deg);
    return static_cast<double>(
        dwell::RirEvaluateAntennaPattern(antenna, beamwidth, offset, pointing, 0.0f).gain_dbi);
  };

  // 采样峰值增益：在主瓣范围内二维细扫求最大值（不读配置峰值）。
  double derived_peak_gain_dbi = -1.0e9;
  for (double az = -half_span_deg; az <= half_span_deg; az += step_deg) {
    for (double el = -half_span_deg; el <= half_span_deg; el += step_deg) {
      const double gain = evaluate_gain(az, el);
      if (std::isfinite(gain)) {
        derived_peak_gain_dbi = std::max(derived_peak_gain_dbi, gain);
      }
    }
  }
  if (!std::isfinite(derived_peak_gain_dbi)) {
    return;  // 采样全部非有限：本项无有效反算值，整行省略（§0 规则3）
  }

  // -3 dB 宽度以采样峰值为基准（不再以配置峰值为基准，避免配置误差掩盖采样特征）。
  auto half_power_width_deg = [&](bool along_azimuth) -> double {
    const double target_dbi = derived_peak_gain_dbi - 3.0;
    for (double offset = 0.0; offset <= half_span_deg; offset += step_deg) {
      const double gain =
          along_azimuth ? evaluate_gain(offset, 0.0) : evaluate_gain(0.0, offset);
      if (gain <= target_dbi) {
        return 2.0 * offset;  // 主瓣对称：单侧 -3 dB 点 × 2
      }
    }
    return std::numeric_limits<double>::quiet_NaN();
  };

  const double derived_az_bw_deg = half_power_width_deg(true);
  const double derived_el_bw_deg = half_power_width_deg(false);

  // 后瓣电平：与波束指向相反方向（离轴 180°）的采样增益，相对采样峰值。
  const double derived_backlobe_db =
      evaluate_gain(180.0 - boresight.az_deg, 0.0) - derived_peak_gain_dbi;

  // 最大旁瓣电平：主瓣外（离轴 > 半波束宽）方位扫描的最大采样增益，相对采样峰值。
  double derived_sidelobe_db = -60.0;
  for (double offset = 1.5 * antenna.nominal_az_beamwidth_deg; offset <= 180.0;
       offset += step_deg) {
    const double gain = evaluate_gain(offset, 0.0);
    if (std::isfinite(gain)) {
      derived_sidelobe_db = std::max(derived_sidelobe_db, gain - derived_peak_gain_dbi);
    }
  }

  const double conf_peak_gain_dbi = static_cast<double>(antenna.main_beam_gain_db);
  const double conf_az_bw_deg = static_cast<double>(antenna.nominal_az_beamwidth_deg);
  const double conf_el_bw_deg = static_cast<double>(antenna.nominal_el_beamwidth_deg);
  const double conf_sidelobe_db = static_cast<double>(antenna.max_sidelobe_level_db);
  const double conf_backlobe_db = static_cast<double>(antenna.backlobe_level_db);

  // 误差口径：|反算 - 配置| / |配置|；配置为 0 时退化为绝对差。dB 量按同式给出比值误差，
  // 反算值与配置值一并落盘，便于验证程序按各自口径复核。
  auto relative_error = [](double derived, double configured) -> double {
    if (!std::isfinite(derived) || !std::isfinite(configured)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    if (std::fabs(configured) < 1.0e-9) {
      return std::fabs(derived - configured);
    }
    return std::fabs(derived - configured) / std::fabs(configured);
  };

  const double err_peak_gain = relative_error(derived_peak_gain_dbi, conf_peak_gain_dbi);
  const double err_az_bw = relative_error(derived_az_bw_deg, conf_az_bw_deg);
  const double err_el_bw = relative_error(derived_el_bw_deg, conf_el_bw_deg);
  const double err_sidelobe = relative_error(derived_sidelobe_db, conf_sidelobe_db);
  const double err_backlobe = relative_error(derived_backlobe_db, conf_backlobe_db);

  const double kTolerance = 0.05;
  auto verdict = [](double error, double tolerance) -> const char* {
    if (!std::isfinite(error)) {
      return "缺失";
    }
    return error <= tolerance ? "通过" : "不通过";
  };
  const char* verdict_peak = verdict(err_peak_gain, kTolerance);
  const char* verdict_az_bw = verdict(err_az_bw, kTolerance);
  const char* verdict_el_bw = verdict(err_el_bw, kTolerance);
  const char* verdict_sidelobe = verdict(err_sidelobe, kTolerance);
  const char* verdict_backlobe = verdict(err_backlobe, kTolerance);
  const bool all_within = std::string(verdict_peak) == "通过" &&
                          std::string(verdict_az_bw) == "通过" &&
                          std::string(verdict_el_bw) == "通过" &&
                          std::string(verdict_sidelobe) == "通过" &&
                          std::string(verdict_backlobe) == "通过";

  // 中译：**逐项输出反算值、配置值与误差**（2026-11-02 要求）——原实现按第 28 项
  //   「日志边界」只写校验结论、不写数值，现场无法核对"反算到底是多少、和配置差多少"。
  //   现按甲方要求改为：每个参数给出「反算 / 配置 / 误差 / 校验」四要素，并写出采样步长，
  //   使日志可用同目录 RadarAntennaGainPatterndata.csv 逐点复核。
  // 标识：第28项反算-配置对照输出（2026-11-02），取代 2026-09-28 的"只写结论"口径。
  const std::string pattern_csv_path = ResolveRirAntennaPatternCsvPath();
  std::string content = "雷达ID=" + std::to_string(radar_id);
  content += " 模型=" + std::string(AntennaPatternModelText(antenna.model_type));
  content += " 方向图文件=" + pattern_csv_path;
  content += " 采样步长=" + FormatF(step_deg, 3) + "°";
  content += " 峰值增益(dBi)[反算=" + FormatF(derived_peak_gain_dbi, 4) +
             " 配置=" + FormatF(conf_peak_gain_dbi, 4) +
             " 误差=" + FormatF(err_peak_gain * 100.0, 2) + "%" + "] 峰值增益校验=" + std::string(verdict_peak);
  content += " 方位波束宽度(deg)[反算=" + FormatF(derived_az_bw_deg, 4) +
             " 配置=" + FormatF(conf_az_bw_deg, 4) +
             " 误差=" + FormatF(err_az_bw * 100.0, 2) + "%" + "] 方位波束宽度校验=" + std::string(verdict_az_bw);
  content += " 俯仰波束宽度(deg)[反算=" + FormatF(derived_el_bw_deg, 4) +
             " 配置=" + FormatF(conf_el_bw_deg, 4) +
             " 误差=" + FormatF(err_el_bw * 100.0, 2) + "%" + "] 俯仰波束宽度校验=" + std::string(verdict_el_bw);
  content += " 最大旁瓣电平(dB)[反算=" + FormatF(derived_sidelobe_db, 3) +
             " 配置=" + FormatF(conf_sidelobe_db, 3) +
             " 误差=" + FormatF(err_sidelobe * 100.0, 2) + "%" + "] 最大旁瓣电平校验=" + std::string(verdict_sidelobe);
  content += " 后瓣电平(dB)[反算=" + FormatF(derived_backlobe_db, 3) +
             " 配置=" + FormatF(conf_backlobe_db, 3) +
             " 误差=" + FormatF(err_backlobe * 100.0, 2) + "%" + "] 后瓣电平校验=" + std::string(verdict_backlobe);
  content += " 外部配置一致性=" + std::string(all_within ? "通过" : "不通过");
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "天线方向图仿真功能测试", content);
}

void WriteRirDetectionChain(const RirDetectionAcceptInput& input) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 无检测单元（未走到检测链）时本函数没有任何真实过程数据：不写 29/32/33/34/35/36 任何行。
  // 2026-09-28 按“没有相关信息输出或输入则不打印日志；未经过具体流程步骤则不输出”收紧：
  // 早期实现在无 cell 时会写出仅含配置派生值或「缺失」占位的行，属于无输入也要出日志的伪输出。
  const bool has_detection_cell = input.has_cell && input.detected;
  if (!has_detection_cell) {
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

  // 验收判定标准 第29项：逐目标一行记录 目标ID/斜距/目标RCS/回波功率/噪声功率/干扰功率。
  // 2026-09-28 按主文档补全：本项不能只写回波功率——同距离不同 RCS（1 m^2 与 0.025 m^2）
  // 的 16 dB 差值判据需要同一行同时给出六项，外部验证程序据斜距配对样本计算功率差。
  // 干扰功率取该检测单元实际收到的干扰功率（无干扰时为 0）。
  {
    std::string echo = id;
    echo += " 斜距=" + FormatF(input.range_m, 1) + "m";
    // 中译：RCS 口径与物理开关挂钩（2026-10-26）——开=视角 RCS；关=外部输入标称 RCS。
    // 标识：第29项 RCS 口径（2026-10-26）。
    echo += std::string(" 目标RCS=") + FormatF(input.rcs_m2, 3) + "m^2";
    // 中译：本行三项**全部是处理前（进入信号处理链之前）的单元级功率**，口径必须写明，
    //   否则会与第 33 项"脉冲压缩后的噪声功率"（处理后）只差一个脉压增益而被误读为矛盾。
    //   回波：目标回波在单元内的接收功率；噪声：匹配滤波带宽内的接收机热噪声；干扰：进入
    //   本单元的外部干扰功率。脉压/MTI/MTD 等处理增益均不计入本行。
    // 标识：第 29 项口径标注（2026-10-22）。
    echo += " 回波功率=" + FormatSci(FromDb(input.echo_power_dbw)) + "W";
    echo += " 噪声功率=" + FormatSci(thermal) + "W";
    echo += " 干扰功率=" +
            FormatSci(input.has_cell ? input.cell.interference_power_w : 0.0) + "W";
    Emit(t, cycle, "回波功率计算功能测试", echo);
  }

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
  // 中译：**处理后雷达回波信号**（判据第 32 项原文要求记录"经处理后的雷达回波信号"）——
  //   等于处理前回波功率乘上处理链全部功率增益（脉压 + MTI + MTD，其中 MTD 含相干积累）。
  //   与第 29 项的 `回波功率=`（处理前）成对出现，便于外部核对"增益乘完是否等于处理后电平"。
  // 标识：第 32 项处理后回波（2026-10-23）。
  {
    const double chain_gain_db = pc_db + (has_bank ? bank.mti_gain_db + bank.mtd_gain_db : 0.0);
    const double echo_before_w = FromDb(input.echo_power_dbw);
    std::string processed = id + " 处理后回波功率=" + FormatSci(echo_before_w * FromDb(chain_gain_db)) + "W";
    processed += std::string(" 处理链路=") +
                 (has_bank ? "脉压+MTI+MTD" : "脉压");
    processed += " 处理前回波功率=" + FormatSci(echo_before_w) + "W";
    Emit(t, cycle, "目标信号增益功能测试", processed);
  }

  // 验收判定标准 第33项：噪声增益四子项（各多普勒通道/MTD 等效无 cell 时省略）。
  if (input.has_cell) {
    // 中译：**脉压后的噪声功率**（2026-10-22 修正数值口径）——匹配滤波器按"单位峰值增益"
    //   归一（信号峰值不变、噪声按 ∫h²=1/τ 收缩），故压缩后噪声 = 输入噪声 /(B·τ)。
    //   与模型的 SINR 记账等价：SNR = 回波 /(N/(Bτ)) = 回波·Bτ/N，两条口径给出同一 SNR。
    //   此前该字段误写为压缩前的 N，与第 29 项同值，属口径错误。
    // 标识：第 33 项脉压后噪声（2026-10-22）。
    const double noise_after_pc = pc > 0.0 ? thermal / pc : thermal;
    Emit(t, cycle, "噪声增益功能测试",
         id_sp + "脉冲压缩后的噪声功率=" + FormatSci(noise_after_pc) + "W");
  }
  EmitOrNone(t, cycle, "噪声增益功能测试", id_sp + "各多普勒滤波器通道噪声功率=", has_bank,
             has_bank ? FormatChannelWatts(bank.noise_w) + "W" : std::string());
  EmitOrNone(t, cycle, "噪声增益功能测试", id_sp + "MTD等效噪声功率=", has_bank,
             has_bank ? FormatSci(bank.mtd_equivalent_noise_w) + "W" : std::string());
  // 中译：**噪声总增益**（2026-10-23）——输出真正施加到 SINR 分母上的总噪声增益，
  //   即"配置的噪声处理增益 + MTI 对消器对白噪声的 ×2（+3.01 dB）"。此前只打印配置项
  //   （1 dB），与实际进入分母的值（含对消器增益）不一致，属口径错误。
  // 标识：噪声总增益（2026-10-23）。
  {
    const double applied_noise_db =
        input.has_cell ? input.cell.noise_total_gain_db : noise_db;
    std::string row = id_sp + "噪声总增益=" + FormatF(applied_noise_db, 3) + "dB";
    Emit(t, cycle, "噪声增益功能测试", row);
  }

  // 验收判定标准 第34项：杂波——植被 kDisabled（clutter_power_w=0）时整条省略；
  // 有杂波时只写 MTI 剩余与 MTD 通道分布（抑制比/总抑制增益为派生量，不写）。
  const bool has_clutter = clutter > 0.0;
  if (has_clutter) {
    Emit(t, cycle, "杂波信号处理增益功能测试",
         id_sp + "MTI处理后杂波剩余功率=" +
             FormatSci(has_bank ? bank.mti_residual_clutter_w : mti_residual) + "W");
    EmitOrNone(t, cycle, "杂波信号处理增益功能测试", id_sp + "MTD各多普勒通道杂波剩余功率分布=",
               has_bank, has_bank ? FormatChannelWatts(bank.clutter_w) + "W" : std::string());
  }
  // 2026-09-28 收紧（无输入不出日志）：无杂波时整条省略，不再写「缺失」占位行——
  // 本项要求“记录杂波处理增益”，没有杂波输入就没有可记录的量。
  // 2026-09-28 合并（§0C.2 裁定 4 复核后放开）：截图要求「MTD 处理后的杂波抑制比」与
  // 「总杂波抑制增益」，由本条已落盘的原始/剩余功率直接求比值（10log10 输入/剩余）。
  // 抑制比取目标所在多普勒通道，总增益取全通道合计，两者均为功率比，不引入新观测量。
  if (has_clutter && has_bank) {
    const double channel_count = static_cast<double>(oneq::common::radar::kMtiMtdChannelCount);
    const double original_channel_clutter =
        clutter_db <= 0.0 ? clutter / channel_count : clutter / FromDb(clutter_db);
    const double residual_total = std::accumulate(bank.clutter_w.begin(), bank.clutter_w.end(), 0.0);
    double ratio_db = 0.0;
    if (oneq::common::radar::TryAcceptancePowerRatioDb(
            original_channel_clutter, bank.clutter_w[bank.selected_channel], &ratio_db)) {
      Emit(t, cycle, "杂波信号处理增益功能测试",
           id_sp + "MTD处理后的杂波抑制比=" + FormatF(ratio_db, 3) + "dB");
    }
    if (oneq::common::radar::TryAcceptancePowerRatioDb(input.has_cell ? clutter : mti_residual,
                                                       residual_total, &ratio_db)) {
      Emit(t, cycle, "杂波信号处理增益功能测试",
           id_sp + "总杂波抑制增益=" + FormatF(ratio_db, 3) + "dB");
    }
  }

  // 验收判定标准 第35项：干扰——无外部干扰单音时整条省略（无输入不出日志）；
  // CFAR 统计归第 36 项。
  const bool has_jam = has_bank && bank.has_jam_channels;
  if (has_jam) {
    Emit(t, cycle, "干扰信号处理增益功能测试",
         id_sp + "MTI处理后干扰剩余功率=" + FormatSci(bank.mti_residual_jam_w) + "W");
    Emit(t, cycle, "干扰信号处理增益功能测试",
         id_sp + "MTD各多普勒通道干扰功率分布=" + FormatChannelWatts(bank.jam_w) + "W");
    // 2026-09-28 合并（§0C.2 裁定 4 复核后放开）：截图要求「MTD 处理后的干扰总抑制比」与
    // 「总干扰抑制增益」，口径同第 34 项（干扰原始总功率取各路到达功率之和）。
    double original_jam = 0.0;
    for (const auto& link : input.incident_links) {
      const double power = std::isfinite(link.received_power_before_overlap_w)
                               ? link.received_power_before_overlap_w
                               : link.received_power_w;
      if (std::isfinite(power) && power > 0.0) {
        original_jam += power;
      }
    }
    const double residual_total = std::accumulate(bank.jam_w.begin(), bank.jam_w.end(), 0.0);
    double ratio_db = 0.0;
    if (std::isfinite(bank.mti_residual_jam_w) &&
        oneq::common::radar::TryAcceptancePowerRatioDb(
            bank.mti_residual_jam_w, bank.jam_w[bank.selected_channel], &ratio_db)) {
      Emit(t, cycle, "干扰信号处理增益功能测试",
           id_sp + "MTD处理后的干扰总抑制比=" + FormatF(ratio_db, 3) + "dB");
    }
    if (oneq::common::radar::TryAcceptancePowerRatioDb(original_jam, residual_total, &ratio_db)) {
      Emit(t, cycle, "干扰信号处理增益功能测试",
           id_sp + "总干扰抑制增益=" + FormatF(ratio_db, 3) + "dB");
    }
  }

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
  // 中译：补"实际计算得出的信噪比"——门限与结果之间的判决依据此前缺失，读日志只能看到
  //       "门限 13.8 / 结果 否"，无法判断差多少。该 SNR 就是 CFAR **实际消费**的那个值：
  //       cell 路径为处理后单脉冲 SINR，回退路径为雷达方程算出的 SNR。
  // 标识：2026-10-20 用户要求第 36 项补信噪比。
  // 中译：标注 SNR 口径——CFAR 实际消费的是"处理后"信噪比（含脉压 48.57 dB 与 MTD 增益），
  //       而第 29 项的六列功率是**处理前**的；不标注会被外部按原始功率复算判为矛盾。
  // 标识：第 36 项 SNR 口径（2026-10-21）。
  Emit(t, cycle, "恒虚警检测功能测试",
       id_sp + "信噪比=" + FormatF(input.snr_db, 3) + "dB");
  Emit(t, cycle, "恒虚警检测功能测试", id_sp + "统计检测结果=" + YesNo(input.detected));
  Emit(t, cycle, "恒虚警检测功能测试", id_sp + "统计检测概率=" + FormatF(input.pd, 5));
  // 2026-09-28 新增要求：本项还须写「检测概率=90%的最大探测距离」及所依据的 RCS。
  // 四次方根距离定律：RCS/增益等条件不变时 SNR ∝ 1/R⁴，故
  //   R_90 = R_now · (SNR_now / SNR_req)^(1/4)
  // 当前 SNR 高于所需值时比值 > 1（目标还能更远），低于时 < 1（已不足）。方向在
  // 2026-09-28 复核中修正：早期实现把比值写反，会出现 SNR 越高、最大距离反而越小。
  // SNR_req 由同一 Marcum Q 检测模型对 Pd=0.90 二分反解，不使用任务粗筛距离充当该值。
  if (input.has_cell && input.range_m > 0.0f) {
    const double snr_now_db = static_cast<double>(input.cell.processed_single_pulse_sinr_db);
    double max_range_m = 0.0;
    if (TryComputeDetectionRangeAtPd(static_cast<double>(input.range_m), snr_now_db, 0.90,
                                     input.cfar_pfa, static_cast<int>(std::max(1U, pulses)),
                                     &max_range_m)) {
      Emit(t, cycle, "恒虚警检测功能测试",
           id_sp + "检测概率=90%的最大探测距离=" + FormatF(max_range_m, 1) +
               "m 依据RCS=" + FormatF(input.rcs_m2, 3) + "m^2");
    }
  }

  // 内部接口第70项：同一条目标检测快照中，回波计算结果即为信号处理入口消费值。
  // 仅有 detection cell 有效时输出，不伪造缺失分量。
  if (input.has_cell) {
    std::string interface_power = "雷达ID=" + std::to_string(input.radar_id) + " " + id;
    interface_power += " 斜距=" + FormatF(input.range_m, 1) + "m";
    interface_power += std::string(" RCS=") + FormatF(input.rcs_m2, 3) + "m^2";
    interface_power += " 波束离轴增益=" + FormatF(input.one_way_antenna_gain_dbi, 3) + "dBi";
    // 中译：判据第 70 项原文要求「并分别标识回波计算输出值与信号处理入口消费值」，
    //   故两个字段都保留；本实现为同一 detection cell 直传，二者必然相等——标注口径
    //   以免被误读为两次独立计算。
    // 标识：第 70 项双值口径（2026-10-26）。
    interface_power += " 回波计算输出功率=" + FormatSci(input.cell.echo_power_w) +
                       "W";
    interface_power += " 处理入口回波功率=" + FormatSci(input.cell.echo_power_w) + "W";
    interface_power += " 回波计算输出时延=" + FormatSci(input.cell.echo_delay_s) + "s";
    interface_power += " 处理入口回波时延=" + FormatSci(input.cell.echo_delay_s) + "s";
    interface_power += " 回波计算输出双程多普勒=" +
                       FormatF(input.cell.two_way_doppler_shift_hz, 3) + "Hz";
    interface_power += " 处理入口双程多普勒=" +
                       FormatF(input.cell.two_way_doppler_shift_hz, 3) + "Hz";
    interface_power += " 脉压增益=" + FormatF(pc_db, 3) + "dB";
    interface_power += " 带宽=" + FormatF(input.bandwidth_hz, 3) + "Hz";
    interface_power += " PRF=" + FormatF(input.prf_hz, 3) + "Hz";
    interface_power += " 有效脉冲数=" + std::to_string(pulses);
    RIR_INTERNAL_INTERFACE_ITEM(t, cycle, "雷达回波与信号处理交互接口测试", interface_power);
  }

  // 内部接口第71项：记录信号处理输出和 CFAR 实际消费的同源 SNR 快照。
  const double detector_snr_db = input.has_cell
                                     ? input.cell.processed_single_pulse_sinr_db
                                     : static_cast<double>(input.snr_db);
  const double accumulated_snr_db = detector_snr_db + 10.0 * std::log10(pulses);
  const double total_noise_w = thermal + clutter +
                               (input.has_cell ? input.cell.interference_power_w : 0.0);
  // 中译：**SNR 三口径对齐判据**（2026-10-26）——判据要求同快照输出「单脉冲 SNR /
  //   积累 SNR / CFAR 消费 SNR」。检测单元的 processed SINR 已含相干积累 N，故：
  //     单脉冲 = processed − 10log10(N)   （脉压+MTI+MTD 之后、积累之前）
  //     CFAR 消费 = 积累 = processed      （检测器实际用的就是积累后的值）
  //   原实现把 processed 当单脉冲再 +10log10(N) 当积累，N 被计了两次（N=11 时 +10.4 dB）。
  // 标识：第 71 项 SNR 三口径（2026-10-26）。
  const double coherent_db = input.has_cell ? input.cell.coherent_integration_db : 0.0;
  const double single_pulse_snr_db = detector_snr_db - coherent_db;
  (void)accumulated_snr_db;
  std::string interface_detection = id + " 处理输出SNR=" +
                                    FormatF(single_pulse_snr_db, 3) +
                                    "dB";
  interface_detection += " 检测器输入SNR=" + FormatF(detector_snr_db, 3) +
                         "dB";
  interface_detection += " 积累SNR=" + FormatF(detector_snr_db, 3) + "dB";
  if (input.has_cell) {
    // 中译：这里的"总噪声功率"是**热噪声 + 杂波 + 干扰**的处理前合计（SINR 分母），
    //   不是单一"噪声"；与第 29 项的单列噪声功率、第 33 项的脉压后噪声都不同口径。
    // 标识：第 71 项噪声合计口径（2026-10-22）。
    interface_detection += " 总噪声功率=" + FormatSci(total_noise_w) +
                           "W";
  }
  interface_detection += " 脉压/MTI/MTD增益=" +
                         FormatF(pc_db, 3) + "/" +
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

std::vector<oneq::electromagnetics::RfIncidentLinkResult> SelectRirExternalInterferenceLinks(
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& links) {
  // 身份无效（任一字段为 0）时无法判定"何者为自身"，保守保留全部链路，避免把
  // 真实外部干扰静默丢掉。
  const bool can_exclude_own = own_emission_identity.sensor_entity_id != 0U &&
                               own_emission_identity.emission_id != 0U;
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> external;
  external.reserve(links.size());
  for (const oneq::electromagnetics::RfIncidentLinkResult& link : links) {
    if (can_exclude_own && SameIdentity(link.identity, own_emission_identity)) {
      continue;
    }
    external.push_back(link);
  }
  return external;
}

void WriteRirInterferenceLinks(
    float sim_time_sec, std::uint32_t cycle,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& links) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 验收判定标准 第30项：身份只写 干扰源ID=（干扰实体）+ 到达雷达功率；
  // 发射二元组、路径与时频域重叠（链路上下文）不写。
  // 2026-09-28 审计 Q-1：只写**外部**干扰源。控制器为接收前端饱和门把自身发射
  // 追加进解析场景（RirController::ResolveRfCycle），该同址耦合链不是干扰源；
  // 与第 29 项 cell.interference_power_w、第 35 项 MTI/MTD 干扰分布同一排除规则。
  const std::vector<oneq::electromagnetics::RfIncidentLinkResult> external_links =
      SelectRirExternalInterferenceLinks(own_emission_identity, links);
  if (external_links.empty()) {
    return;  // 本周期无外部干扰源：第30项整条省略（无输入不出日志）。
  }
  for (const oneq::electromagnetics::RfIncidentLinkResult& link : external_links) {
    std::string content = "干扰源ID=" + std::to_string(link.identity.sensor_entity_id);
    // 中译：判定标准第 30 项要的是"到达雷达接收端"的干扰功率，即**未乘**时频重叠的前端
    //       到达功率；此前打印的是 `received_power_w`（已乘场景级重叠），与第 29 项
    //       "单元内干扰功率"不可复算。字段名按用户 2026-10-20 要求改为
    //       "到达雷达的干扰功率"。
    // 标识：T-02（2026-10-20）。
    content += " 到达雷达的干扰功率=" + FormatSci(link.received_power_before_overlap_w) + "W";
    RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "干扰功率计算功能测试", content);
  }
}

void WriteRirExternalInterferenceInput(float sim_time_sec, std::uint32_t cycle,
                                       const oneq::electromagnetics::RfSceneFrame& rf_scene) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED() || rf_scene.emissions.empty()) {
    return;
  }
  const char* item = "外部干扰输入雷达接口测试";
  std::string summary = "干扰数量=" + std::to_string(rf_scene.emissions.size());
  summary += " 世界周期=" + std::to_string(rf_scene.world_cycle_index);
  summary += " 接收时间窗=" + FormatF(rf_scene.window_start_time_s, 6) + "/" +
             FormatF(rf_scene.window_duration_s, 6) + "s";
  RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, item, summary);

  for (const oneq::electromagnetics::RfSceneEmission& emission : rf_scene.emissions) {
    const oneq::electromagnetics::RfWaveformSchedule& waveform = emission.waveform;
    std::string content = "干扰源ID=" + std::to_string(emission.identity.sensor_entity_id);
    content += " 发射ID=" + std::to_string(emission.identity.emission_id);
    content += " 位置ECEF=" +
               FormatVec3(emission.position_ecef_m.x_m, emission.position_ecef_m.y_m,
                          emission.position_ecef_m.z_m, 3) +
               "m";
    content += " 速度ECEF=" +
               FormatVec3(emission.velocity_ecef_mps.x_mps, emission.velocity_ecef_mps.y_mps,
                          emission.velocity_ecef_mps.z_mps, 3) +
               "m/s";
    content += " 波形=" + std::string(RfWaveformKindText(waveform.kind));
    content += " 活动起点/持续时间=" + FormatF(waveform.activity_start_time_s, 6) + "/" +
               FormatF(waveform.activity_duration_s, 6) + "s";
    content += " 中心频率=" + FormatF(waveform.center_frequency_hz, 3) + "Hz";
    content += " 带宽=" + FormatF(waveform.occupied_bandwidth_hz, 3) + "Hz";
    content += " 发射功率=" + FormatSci(waveform.transmit_power_w) + "W";
    content += " 峰值增益=" + FormatF(emission.antenna.peak_gain_dbi, 3) + "dBi";
    content += " 极化=" + std::to_string(static_cast<int>(emission.polarization));
    RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, item, content);
  }
}

void WriteRirExternalSceneTargets(float sim_time_sec, std::uint32_t cycle,
                                  const session::RirSceneTargetList& targets) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED() || targets.empty()) {
    return;
  }
  const char* item = "目标场景接口测试";
  RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, item,
                              "目标数量=" + std::to_string(targets.size()));
  for (std::size_t target_index = 0U; target_index < targets.size(); ++target_index) {
    const session::RirSceneTarget& target = targets[target_index];
    std::string content = "目标序号=" + std::to_string(target_index);
    content += " 目标ID=" + std::to_string(target.external_target_id);
    content += " 目标名称=" + (target.target_name.empty() ? std::string("未提供")
                                                           : target.target_name);
    // 中译：场景接口的 ENU 位置/速度是**场景真值**（来自想定），与第 73 项的航迹估计同名不同源，
    //   必须标注，否则会被当成同一量的两个互相矛盾的估计。
    // 标识：场景接口真值口径（2026-10-22）。
    content += " 位置ENU=" +
               FormatVec3(target.position_x, target.position_y, target.position_z, 3) +
               "m";
    content += " 速度ENU=" + FormatVec3(target.velocity_x, target.velocity_y, target.velocity_z, 3) +
               "m/s";
    content += " RCS=" + FormatF(target.rcs, 6) + "m^2";
    content += " Swerling=" +
               std::to_string(static_cast<int>(target.target_swerling_type));

    content += " 视角RCS=[";
    for (std::size_t i = 0U; i < target.aspect_rcs_samples.size(); ++i) {
      if (i != 0U) {
        content += ";";
      }
      const session::RirAspectRcsSample& sample = target.aspect_rcs_samples[i];
      content += "(" + FormatF(sample.aspect_az_deg, 3) + "," +
                 FormatF(sample.aspect_el_deg, 3) + "," + FormatF(sample.rcs_dbsm, 3) + ")";
    }
    content += "]deg/deg/dBsm";

    content += " 极化数据=[";
    for (std::size_t i = 0U; i < target.polarization_samples.size(); ++i) {
      if (i != 0U) {
        content += ";";
      }
      const session::RirPolSMatrixSample& sample = target.polarization_samples[i];
      content += "(" + FormatF(sample.aspect_az_deg, 3) + "," +
                 FormatF(sample.aspect_el_deg, 3) + ",HH=" + FormatF(sample.hh_amp_db, 3) + "/" +
                 FormatF(sample.hh_phase_deg, 3) + ",HV=" + FormatF(sample.hv_amp_db, 3) + "/" +
                 FormatF(sample.hv_phase_deg, 3) + ",VH=" + FormatF(sample.vh_amp_db, 3) + "/" +
                 FormatF(sample.vh_phase_deg, 3) + ",VV=" + FormatF(sample.vv_amp_db, 3) + "/" +
                 FormatF(sample.vv_phase_deg, 3) + ")";
    }
    content += "]deg/deg/dB/deg";

    content += " HRRP散射中心=[";
    for (std::size_t i = 0U; i < target.range_rcs_scatterers.size(); ++i) {
      if (i != 0U) {
        content += ";";
      }
      const session::RirRangeRcsScatterer& scatterer = target.range_rcs_scatterers[i];
      content += "(" + FormatF(scatterer.range_offset_m, 3) + "m," +
                 FormatF(scatterer.rcs_dbsm, 3) + "dBsm,ch1=" +
                 FormatF(scatterer.channel_1_rcs_dbsm, 3) + ",ch2=" +
                 FormatF(scatterer.channel_2_rcs_dbsm, 3) + ",phase=" +
                 FormatF(scatterer.phase_deg, 3) + "deg,fluct=" +
                 FormatF(scatterer.fluctuation_std_db, 3) + "dB)";
    }
    content += "]";
    RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, item, content);
  }
}

void WriteRirSearchDetections(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                              float beam_az_deg, float beam_el_deg,
                              const std::vector<RirSearchDetectionLite>& detections) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED() || detections.empty()) {
    return;
  }
  // 验收判定标准 第37项「空域自主搜索功能测试」：指向信息与检测量测两行（扫描调度信息
  // 行随 WriteRirSchedule 写出——调度统计在该调用点）。项名固定为
  // [空域自主搜索功能测试]，不与 37A 互相替代。检测量测只写目标ID/斜距/量测方位俯仰
  //（Pd 归第36项，不写）。
  Emit(sim_time_sec, cycle, "空域自主搜索功能测试",
       "雷达ID=" + std::to_string(radar_id) + " 方位/俯仰指向信息=" +
           FormatPairDeg(beam_az_deg, beam_el_deg, 3) + "°");
  std::string found;
  for (std::size_t i = 0U; i < detections.size(); ++i) {
    if (i != 0U) {
      found += "; ";
    }
    found += "ID=" + std::to_string(detections[i].target_id);
    found += " 斜距=" + FormatF(detections[i].range_m, 1) + "m";
    found += " 量测方位/俯仰=" + FormatPairDeg(detections[i].az_deg, detections[i].el_deg, 3) + "°";
  }
  Emit(sim_time_sec, cycle, "空域自主搜索功能测试",
       "检测与量测信息=搜到目标=[" + found + "]");
}

void WriteRirHorizonSearchDetections(std::uint64_t radar_id, float sim_time_sec,
                                     std::uint32_t cycle, float beam_az_deg, float beam_el_deg,
                                     const std::vector<RirSearchDetectionLite>& detections) {
  // 中译：**地平线搜索按“驻留事实”记账**（2026-10-26）——此前 `detections.empty()` 直接
  //   return，只有“地平线波位恰好检到目标”才留下日志，搜索窗抬高后整项几乎不可见。
  //   现改为：只要有地平线波位驻留就写指向行；检到目标才追加量测行。
  // 标识：地平线搜索驻留即记（2026-10-26）。
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 验收判定标准 第37A项「地平线搜索功能测试」：与第37项同批字段、独立项名，按 §0.9
  // 条目独立判定，不得互相替代。指向角取该波位实际指向（地平线波位 el≈0）。
  Emit(sim_time_sec, cycle, "地平线搜索功能测试",
       "雷达ID=" + std::to_string(radar_id) + " 方位/俯仰指向信息=" +
           FormatPairDeg(beam_az_deg, beam_el_deg, 3) + "°");
  if (detections.empty()) {
    return;  // 无检测：只留指向行，不伪造量测。
  }
  std::string found;
  for (std::size_t i = 0U; i < detections.size(); ++i) {
    if (i != 0U) {
      found += "; ";
    }
    found += "ID=" + std::to_string(detections[i].target_id);
    found += " 斜距=" + FormatF(detections[i].range_m, 1) + "m";
    found += " 量测方位/俯仰=" + FormatPairDeg(detections[i].az_deg, detections[i].el_deg, 3) + "°";
  }
  Emit(sim_time_sec, cycle, "地平线搜索功能测试",
       "检测与量测信息=搜到目标=[" + found + "]");
}

void WriteRirAssociation(float sim_time_sec, std::uint32_t cycle,
                         const tracking::RirAssociationResult& association,
                         const oneq::coordinate::LlaPositionDegM& platform_lla,
                         double gate_threshold) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  (void)gate_threshold;  // 波门不再随行输出（马氏距离^2为派生量，验收行不写）。
  // 验收判定标准 第41项：逐量测一行（目标ID/位置LLA/检测概率/关联结果）；
  // 计数汇总、马氏距离^2（派生）与重复的更新后航迹集合行不写。
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
    // 中译：本项 Pd 与第 36 项的「统计检测概率」同源不同口径（此处按本周期该次量测），
    //       加限定词避免两处数值被当作互相印证或互相矛盾。
    // 标识：第 43 项检测概率口径（2026-10-21）。
    content += " 检测概率=" + FormatF(measurement.detection_pd, 4) ;
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
  if (!RIR_ACCEPTANCE_LOG_ENABLED() || association.measurements.empty()) {
    return;
  }
  const char* item = "雷达对外目标信息输出接口测试-点迹量测";
  RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, item,
                              "数据层级=点迹量测 量测数量=" +
                                  std::to_string(association.measurements.size()));
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
    RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, item, "数据层级=点迹量测 " + content);
  }
}

void WriteRirClusterCount(float sim_time_sec, std::uint32_t cycle,
                          const std::vector<tracking::RirTrackState>& tracks) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED() || tracks.empty()) {
    return;
  }
  // 甲方 2026-08-22 批注「要输出集群目标数量（知道能打出几枚即可）」：
  // 集群目标数量 = 确认航迹数（身份去重后的在跟目标数，非检测条数——
  // 「检测条数不得顶替集群规模」原裁定由本批注按计数口径覆盖）。同时给出
  // 待确认/丢失计数作上下文；无确认航迹时整项省略，不写规模 0 占位。
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
  if (confirmed == 0U) {
    return;
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
                        const config::RirRecognitionFeatureWeights& feature_weights,
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
  // 中译：加速度优先取"拟合速度斜率"（2026-10-26）——滤波后验差分在重置周期会
  //   跳到上百 m/s²，验收第 40/42 项会显示非物理值；拟合值与识别运动维同源。
  // 标识：验收加速度取拟合斜率（2026-10-26）。
  const Eigen::Vector3f acc_source = track.has_acceleration_lsq
                                         ? track.acceleration_lsq_mps2
                                         : track.acceleration;
  bool have_frame = false;
  if (oneq::coordinate::TryEcefToLla(platform_ecef, &platform_lla)) {
    const oneq::coordinate::EnuPositionM pos_enu(track.position.x(), track.position.y(),
                                                 track.position.z());
    const oneq::coordinate::EnuVelocityMps vel_enu(track.velocity.x(), track.velocity.y(),
                                                   track.velocity.z());
    const oneq::coordinate::EnuVelocityMps acc_enu(acc_source.x(), acc_source.y(), acc_source.z());
    have_frame = oneq::coordinate::TryEnuToEcef(pos_enu, platform_lla, &pos_ecef) &&
                 oneq::coordinate::TryEnuToEcefVelocity(vel_enu, platform_lla, &vel_ecef) &&
                 oneq::coordinate::TryEnuToEcefVelocity(acc_enu, platform_lla, &acc_ecef);
  }
  const std::string track_id = "航迹=" + std::to_string(track.association_key);
  const std::string track_target_id =
      track_id + " 目标ID=" + std::to_string(track.external_target_id);

  // 中译：航迹已判丢失时只报状态，不再输出定位/运动/协方差/识别——这些数值是发散外推
  //       或纯惯性保持，当成产品输出会误导判读（本轮实测丢失航迹 ENU 上分量可达 +8,390 km，
  //       而真值只有约 2,100 km）。
  // 标识：B7 丢失降级（2026-10-20），依据 docs/review/round_analysis_2026-10-20_1006.md §4-S1#3。
  if (track.status == tracking::RirTrackStatus::kLost) {
    const std::string lost_state = track_target_id + " 状态=丢失";
    Emit(sim_time_sec, cycle, "对典型目标/再入目标进行跟踪功能测试", lost_state);
    Emit(sim_time_sec, cycle, "多目标跟踪功能测试", lost_state);
    // 内部接口第 73 项保持一行对账（只带状态，不重复发散状态量）。
    RIR_INTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, "雷达航迹与特征识别交互接口测试",
                                track_target_id + " 航迹状态=丢失");
    return;
  }

  // 内部接口第73项：航迹后验与本周期特征快照按关联键对账。
  std::string interface_recognition = track_target_id;
  interface_recognition += " 航迹状态=" + std::string(TrackStatusText(track.status));
  // 中译：**来源标注**（2026-10-22）——速度/位置与第 42、65 项同源（同一航迹快照），
  //   此处标注坐标系与口径，避免"同一拍多处速度数值几乎相同"被误读为重复测量。
  // 标识：第 73 项速度来源（2026-10-22）。
  interface_recognition += " 位置ENU=" +
                           FormatVec3(track.position.x(), track.position.y(), track.position.z(), 3) +
                           "m 速度ENU=" +
                           FormatVec3(track.velocity.x(), track.velocity.y(), track.velocity.z(), 3) +
                           std::string("m/s 坐标系=ENU");
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
    // 中译：不再输出"置信度"（用户 2026-10-21 裁定）——该值实为
    //       best_score/total_score 的"得分份额"（≈1/模型数），不是类别置信概率，
    //       与"已给出类别"并列会被误读；型号/类别本身仍在同一行输出。
    // 标识：2026-10-21 删除置信度字段。
    // 中译：**识别判据诊断**（2026-10-27）——把决定"类别能不能确认"的三个数直接写出来：
    //   综合分（最佳候选分）、次优分、二者边际。类别确认要求 综合分 ≥ acceptance_score(0.6)；
    //   型号确认还要求 边际 ≥ minimum_margin(0.05) 且有效维度足够。
    //   之前日志只有"类别/型号"，判不出来时无法区分是"所有模板都低分"还是"候选太接近"。
    // 标识：识别判据诊断（2026-10-27）。
    interface_recognition += " 综合分=" + FormatF(result->best_score, 4);
    interface_recognition += " 次优分=" + FormatF(result->runner_up_score, 4);
    interface_recognition += " 边际=" + FormatF(result->best_score - result->runner_up_score, 4);
    interface_recognition += " 有效维度数=" + std::to_string(CountBits(result->valid_feature_mask));
    interface_recognition += " 积累观测数=" + std::to_string(result->observation_count);
  }
  RIR_INTERNAL_INTERFACE_ITEM(sim_time_sec, cycle,
                              "雷达航迹与特征识别交互接口测试", interface_recognition);

  // 外部导入项：综合特征识别是独立验收用例，与第47项宽带一维像分别记录。
  // 中译：本项只输出**识别目标类别结果**（用户 2026-10-20 要求）——型号/置信度/
  //       特征掩码不再写；航迹与目标 ID 保留，否则该结果无法归因到目标。
  // 标识：47A 输出精简（2026-10-20）。
  if (result != nullptr) {
    std::string comprehensive = track_target_id + " 目标类别=" + category_text;
    Emit(sim_time_sec, cycle, "综合特征识别功能测试", comprehensive);
  }

  // 第38项 角度和距离测量：量测角距一行；目标量测误差统计=量测-真值（可计算口径，
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
  // 2026-09-28 按更新后的规范：定位信息固定为 斜距 + 方位/俯仰（角定位），不再追加
  // 高度= 代替俯仰；无有效角距时整行以「定位信息=缺失」如实标记，不写高度占位。
  // 中译：**来源标注**（2026-10-22 改口径）——同一定位/运动量在多项中重复出现是判据要求，
  //   为避免把"同一值多处出现"误读成"多处独立测量"，在复合字段尾部标注该值由雷达**哪一段**
  //   处理出来：检测前/检测后/滤波前/滤波后/识别后（词表见 common/acceptance/AcceptanceSourceStage.h）。
  //   **不标具体算法**（最小二乘/差分等）：算法属实现细节，随实现可换；判据只关心阶段。
  //   判据要求的分项保持原样，不删任何字段。
  // 标识：复合字段来源标注（2026-10-22 阶段口径）。
  const std::string positioning =
      has_polar ? ("定位信息=斜距=" + FormatF(range_m, 1) + "m 方位/俯仰=" +
                   FormatPairDeg(az_deg, el_deg, 3) + "° 来源=" + oneq::common::acceptance::kStageAfterFilter)
                : std::string("定位信息=缺失");
  const std::string motion =
      std::string("运动参数=速度=") + FormatF(track.speed, 3) + "m/s 航向=" +
      FormatF(heading, 2) + "° 加速度=" +
      (have_frame ? FormatVec3(acc_ecef.x_mps, acc_ecef.y_mps, acc_ecef.z_mps, 4)
                  : FormatVec3(acc_source.x(), acc_source.y(), acc_source.z(), 4)) +
      std::string("m/s^2 来源=") + oneq::common::acceptance::kStageAfterFilter;
  const char* kReentryItem = "对典型目标/再入目标进行跟踪功能测试";
  const char* kMultiItem = "多目标跟踪功能测试";
  Emit(sim_time_sec, cycle, kReentryItem,
       track_target_id + std::string(" 状态=") + TrackStatusText(track.status) + " " + positioning);
  Emit(sim_time_sec, cycle, kReentryItem, track_target_id + " " + motion);
  Emit(sim_time_sec, cycle, kMultiItem, track_target_id + " " + positioning);
  Emit(sim_time_sec, cycle, kMultiItem, track_target_id + " " + motion);

  // 第42项 跟踪滤波：当前估计/协方差/估计误差指标三行（下一时刻外推、协方差迹、
  // IMM 权重为超出规范或分子项，不写；误差指标=滤波-真值，无真值时省略该行）。
  if (have_frame) {
    Emit(sim_time_sec, cycle, "跟踪滤波功能测试",
         track_target_id + " 位置（ECEF）=" +
             FormatVec3(pos_ecef.x_m, pos_ecef.y_m, pos_ecef.z_m, 1) + "m 速度=" +
             FormatVec3(vel_ecef.x_mps, vel_ecef.y_mps, vel_ecef.z_mps, 3) +
             std::string("m/s 加速度=") +
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
       "目标ID=" + std::to_string(track.external_target_id) +
       std::string(" 本周期RCS=") +
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
      // 中译：标注速度/加速度的来源**阶段**（2026-10-22 改口径）：识别侧这两个量可能取自
      //       "滤波前"（由检测后点迹序列统计）或"滤波后"（跟踪滤波后验），两者数值量级
      //       不同；日志只写阶段，不写具体算法。
      // 标识：运动特征速度/加速度来源阶段（2026-10-22）。
      motion_text += std::string(" 速度来源=") +
                     oneq::common::acceptance::FilterStageText(motion_feature.speed_from_fit);
      motion_text += std::string(" 加速度来源=") +
                     oneq::common::acceptance::FilterStageText(motion_feature.acceleration_from_fit);
      motion_text += " 识别类别=" + category_text;
      Emit(sim_time_sec, cycle, "运动特征处理功能测试", motion_text);
    }

    // 第45项 RCS 统计特征处理（维度无效时省略）。
    const auto& rcs = features->features.rcs;
    if (rcs.valid) {
      std::string rcs_text = track_target_id;
      rcs_text += " RCS均值/标准差=" + FormatF(rcs.mean_dbsm, 3) + "/" + FormatF(rcs.std_db, 3) +
                  "dBsm";
      rcs_text += " 平均RCS归一化值=" + FormatF(FromDb(rcs.mean_dbsm), 6);
      rcs_text += " RCS波动系数=" + FormatF(RcsFluctuationCoefficient(rcs.std_db), 6);
      double contribution = 0.0;
      if (TryRecognitionContribution(result, feature_weights,
                                     session::RirRecognitionFeatureDimension::kRcs,
                                     &contribution)) {
        rcs_text += " 综合识别加权贡献=" + FormatF(contribution, 6);
      }
      rcs_text += " 识别类别=" + category_text;
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
    std::string polarization_result =
        track_target_id + " 目标类型=" + category_text + subclass_text;
    double polarization_contribution = 0.0;
    if (TryRecognitionContribution(result, feature_weights,
                                   session::RirRecognitionFeatureDimension::kPolarization,
                                   &polarization_contribution)) {
      polarization_result +=
          " 综合识别加权贡献=" + FormatF(polarization_contribution, 6);
    }
    Emit(sim_time_sec, cycle, "极化特征解算功能测试", polarization_result);

    // 第47项 宽带一维像特征解算：散射中心（场景距离像真值输入——库内特征提取只
    // 出长度/峰数等统计量，无逐中心量测输出）+ 轮廓（长度/峰数）+ 识别类别 +
    // 综合识别加权贡献；能量集中/分辨率/置信度不写。距离像维度无效时省略（§0 规则3）。
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
      double contribution = 0.0;
      if (TryRecognitionContribution(result, feature_weights,
                                     session::RirRecognitionFeatureDimension::kRangeProfile,
                                     &contribution)) {
        rp_text += " 综合识别加权贡献=" + FormatF(contribution, 6);
      }
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
  if (!RIR_ACCEPTANCE_LOG_ENABLED() || tracks.empty()) {
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
  const char* item = "雷达对外目标信息输出接口测试-航迹识别";
  RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, item, "数据层级=航迹识别 " + summary);

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
    // 中译：**来源标注**（2026-10-22）——位置/速度/加速度均取自同一航迹快照，与第 42 项
    //   （ECEF）和第 73 项（ENU）同源；速度与加速度是滤波后验（不是量测差分）。
    // 标识：第 65 项速度来源（2026-10-22）。
    content += " 速度=" +
               FormatVec3(track.velocity.x(), track.velocity.y(), track.velocity.z(), 3) +
               std::string("m/s 坐标系=ENU");
    content += " 加速度=" +
               FormatVec3(track.acceleration.x(), track.acceleration.y(), track.acceleration.z(),
                          3) +
               std::string("m/s^2");
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
    } else {
      content += " 目标类别=未知 目标型号=未识别";
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
    RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, item, "数据层级=航迹识别 " + content);
  }
}

void WriteRirExternalResourceStatus(
    std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
    config::RirWorkMode work_mode, const std::vector<RirDwellPlan>& dwell_plan,
    const std::vector<tracking::RirTrackState>& tracks, std::size_t track_pool_capacity,
    std::size_t track_pool_free_count, const session::RirIssueList& issues) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const char* item = "雷达资源状态与日志接口测试";
  std::size_t tentative_count = 0U;
  std::size_t confirmed_count = 0U;
  std::size_t lost_count = 0U;
  std::string target_list = "[";
  for (std::size_t i = 0U; i < tracks.size(); ++i) {
    const tracking::RirTrackState& track = tracks[i];
    if (track.status == tracking::RirTrackStatus::kConfirmed) {
      ++confirmed_count;
    } else if (track.status == tracking::RirTrackStatus::kLost) {
      ++lost_count;
    } else {
      ++tentative_count;
    }
    if (i != 0U) {
      target_list += ";";
    }
    target_list += "航迹" + std::to_string(track.association_key) + "->目标" +
                   std::to_string(track.external_target_id) + "(" + TrackStatusText(track.status) +
                   ")";
  }
  target_list += "]";

  std::size_t search_count = 0U;
  std::size_t designate_count = 0U;
  std::size_t external_track_count = 0U;
  std::size_t internal_track_count = 0U;
  std::string dwell_queue = "[";
  for (std::size_t i = 0U; i < dwell_plan.size(); ++i) {
    const RirDwellPlan& dwell = dwell_plan[i];
    if (dwell.kind == RirDwellKind::kSearch) {
      ++search_count;
    } else if (dwell.kind == RirDwellKind::kDesignate) {
      ++designate_count;
    } else if (dwell.kind == RirDwellKind::kExternalTrack) {
      ++external_track_count;
    } else {
      ++internal_track_count;
    }
    if (i != 0U) {
      dwell_queue += ";";
    }
    dwell_queue += std::to_string(i) + ":" + DwellKindText(dwell.kind) + "@(" +
                   FormatF(dwell.pointing_deg.az_deg, 3) + "," +
                   FormatF(dwell.pointing_deg.el_deg, 3) + ")deg";
    if (dwell.external_target_id != 0U) {
      dwell_queue += "/目标" + std::to_string(dwell.external_target_id);
    }
  }
  dwell_queue += "]";

  const std::size_t in_use = track_pool_capacity >= track_pool_free_count
                                 ? track_pool_capacity - track_pool_free_count
                                 : tracks.size();
  std::string summary = "雷达ID=" + std::to_string(radar_id);
  summary += " 生命周期状态=运行";
  summary += " 工作模式=" + std::string(WorkModeText(work_mode));
  summary += " 已用跟踪容量=" + std::to_string(in_use);
  summary += " 可用跟踪容量=" + std::to_string(track_pool_free_count);
  summary += " 航迹状态候选/确认/丢失=" + std::to_string(tentative_count) + "/" +
             std::to_string(confirmed_count) + "/" + std::to_string(lost_count);
  summary += " 波束任务数=" + std::to_string(dwell_plan.size());
  summary += " 搜索/指定/外部跟踪/库内跟踪任务=" + std::to_string(search_count) + "/" +
             std::to_string(designate_count) + "/" + std::to_string(external_track_count) + "/" +
             std::to_string(internal_track_count);
  summary += " 波束任务队列=" + dwell_queue;
  summary += " 当前目标数量=" + std::to_string(tracks.size());
  summary += " 当前目标列表=" + target_list;
  summary += " 告警数量=" + std::to_string(issues.size());
  RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, item, summary);

  // 中译：告警**沿触发**——同一告警（码 + 内容）在状态未变化时不再每周期重复输出。
  //   实测地球遮挡告警每周期刷 4,000+ 行完全相同的文本（共 1.8 万行），既淹没其他
  //   证据，也让日志体积翻倍。规则：首次出现必输出、内容变化必输出、消失时输出一条
  //   "恢复"；此外每 kIssueHeartbeatCycles 周期补一条心跳，保留"持续存在"的证据。
  // 标识：告警沿触发（2026-10-21）。
  constexpr std::uint32_t kIssueHeartbeatCycles = 100U;
  static std::unordered_map<std::uint64_t, std::set<std::string>> last_issue_keys;
  static std::unordered_map<std::uint64_t, std::uint32_t> last_issue_cycle;
  std::set<std::string>& previous_keys = last_issue_keys[radar_id];
  std::set<std::string> current_keys;
  const std::uint32_t last_cycle = last_issue_cycle[radar_id];
  const bool heartbeat_due =
      (last_cycle == 0U) || (cycle < last_cycle) ||
      (cycle - last_cycle >= kIssueHeartbeatCycles);
  for (const session::RirIssue& issue : issues) {
    std::string content = "雷达ID=" + std::to_string(radar_id);
    content += " 告警码=" + issue.code;
    content += " 严重度=" + std::to_string(static_cast<int>(issue.severity));
    content += " 阶段=" + std::to_string(static_cast<int>(issue.phase));
    content += " 日志内容=" + issue.message;
    const std::string key = issue.code + "|" + issue.message;
    current_keys.insert(key);
    if (previous_keys.find(key) == previous_keys.end() || heartbeat_due) {
      RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, item, content);
    }
  }
  for (const std::string& stale : previous_keys) {
    if (current_keys.find(stale) != current_keys.end()) {
      continue;
    }
    RIR_EXTERNAL_INTERFACE_ITEM(sim_time_sec, cycle, item,
                                "雷达ID=" + std::to_string(radar_id) + " 告警恢复=" + stale);
  }
  previous_keys = current_keys;
  if (heartbeat_due) {
    last_issue_cycle[radar_id] = cycle;
  }
}

void WriteRirSchedule(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                      float budget_sec, float consumed_sec, std::uint32_t search_count,
                      std::uint32_t designate_count, std::uint32_t external_track_count,
                      std::uint32_t internal_track_count, std::uint32_t confirmed_tracks) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 验收判定标准 第37项·其二：扫描调度信息（搜索/指定/驻留/预算/已耗时；
  // 2026-08-30 核查 9.1：删除"计划驻留/实际执行"字段——库内逐驻留执行，实际数
  // 恒等于计划数，无信息量；按类分项计数与时间预算保留。
  // 2026-10-22：库内跟踪与外部跟踪合并为"驻留"一项输出（用户要求统一命名）。
  Emit(sim_time_sec, cycle, "空域自主搜索功能测试",
       "雷达ID=" + std::to_string(radar_id) + " 扫描调度信息=搜索=" +
           std::to_string(search_count) + " 指定=" + std::to_string(designate_count) +
           " 驻留=" + std::to_string(external_track_count + internal_track_count) +
           " 预算/已耗时=" + FormatF(budget_sec, 3) + "/" + FormatF(consumed_sec, 3) + "s");
  // 验收判定标准 第49项：调度策略——各类事件的实际执行数量列表。
  // 中译：第 49 项同批拆分——"跟踪数"此前把外部引导与库内航迹混并，改为分别列出。
  // 标识：T-04a 计数拆分（2026-10-21）。
  Emit(sim_time_sec, cycle, "调度策略功能测试",
       "雷达ID=" + std::to_string(radar_id) + " [搜索数" + std::to_string(search_count) +
           ",指定数" + std::to_string(designate_count) + ",外部跟踪数" +
           std::to_string(external_track_count) + ",库内跟踪数" +
           std::to_string(internal_track_count) + ",确认航迹数" +
           std::to_string(confirmed_tracks) + "]");
}

void WriteRirDwellScan(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                       const std::vector<RirDwellPlan>& dwell_plan) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 验收判定标准 第48项：逐驻留扫描轨迹序号（2026-08-29 TAS：一周期多驻留）。
  // 波位排列表/扫描轨迹全量在 Rir_scan_pattern.csv（index,az_deg,el_deg）；日志不再
  // 重复当前波束指向，搜索驻留只写波位序号（=波位表游标，可索引 CSV）。
  // 指定/跟踪驻留带种类与目标 ID 标记。
  // 2026-09-28 追加：本模型无独立地平线搜索模式，故以波位表内俯仰接近 0°（|el| ≤
  // 半波束宽）的搜索驻留作为地平线波位，按其实际指向独立写出第 37A 项「方位/俯仰
  // 指向信息」行；该项的扫描调度信息行随 WriteRirSchedule 一并写出。
  const std::string scan_csv_path = ResolveRirScanPatternCsvPath();
  constexpr double kHorizonElevationToleranceDeg = 0.5;
  std::size_t search_dwell_count = 0U;
  for (const RirDwellPlan& dwell : dwell_plan) {
    if (dwell.kind == RirDwellKind::kSearch) {
      ++search_dwell_count;
    }
  }
  // 中译：每周期先写一行"本周期搜索驻留数"，让"驻留点数量"在第 48 项自身可读，
  //       不必数逐驻留行、也不必再对照第 37 项的搜索计数。
  // 标识：第 48 项驻留点计数（2026-10-21，核对第 37/48 项驻留数一致性后新增；此前
  //       `本周期序号` 实为波位表游标，容易被误读成周期内序号且总数无从自证）。
  Emit(sim_time_sec, cycle, "波束扫描功能测试",
       "雷达ID=" + std::to_string(radar_id) +
           " 本周期搜索驻留数=" + std::to_string(search_dwell_count));
  for (const RirDwellPlan& dwell : dwell_plan) {
    if (dwell.kind == RirDwellKind::kSearch) {
      // 中译：**搜索驻留也写指向**（2026-10-26）——原来搜索行只有波位序号、没有指向，
      //   于是"本周期搜索驻留数=N"与带指向的行数对不上（只有跟踪/指定行带指向）。
      //   现在每类驻留都写指向，(数量,指向) 逐行可对数。
      // 标识：搜索驻留指向（2026-10-26）。
      Emit(sim_time_sec, cycle, "波束扫描功能测试",
           "雷达ID=" + std::to_string(radar_id) + " 驻留种类=搜索 波位序号=" +
               std::to_string(dwell.scan_pattern_index) + " 指向=" +
               FormatPairDeg(dwell.pointing_deg.az_deg, dwell.pointing_deg.el_deg, 3) +
               "° 波位排列表已输出至" + scan_csv_path);
      if (std::fabs(static_cast<double>(dwell.pointing_deg.el_deg)) <=
          kHorizonElevationToleranceDeg) {
        Emit(sim_time_sec, cycle, "地平线搜索功能测试",
             "雷达ID=" + std::to_string(radar_id) + " 方位/俯仰指向信息=" +
                 FormatPairDeg(dwell.pointing_deg.az_deg, dwell.pointing_deg.el_deg, 3) + "°");
      }
    } else {
      // 中译：驻留种类只分两类——"指定驻留"（算子/任务点名）与"驻留"（一切跟踪类照射：
      //       库内航迹预测指向与外部引导窗口波位都归此，2026-10-22 用户要求合并命名）。
      //       内部仍按 kTrack / kExternalTrack 两种调度语义分别排波位，仅日志措辞统一。
      // 标识：第 48 项字段改造（2026-10-22 合并命名）。
      const char* kind_text =
          dwell.kind == RirDwellKind::kDesignate ? "指定驻留" : "驻留";
      Emit(sim_time_sec, cycle, "波束扫描功能测试",
           "雷达ID=" + std::to_string(radar_id) + " 驻留种类=" + kind_text + " 指向=" +
               FormatPairDeg(dwell.pointing_deg.az_deg, dwell.pointing_deg.el_deg, 3) + "°");
    }
  }
}

bool TryResolveAzimuthSpanDeg(const std::vector<float>& azimuths_deg, double* center_deg,
                              double* half_width_deg, double* min_deg, double* max_deg) {
  if (center_deg == nullptr || half_width_deg == nullptr || min_deg == nullptr ||
      max_deg == nullptr) {
    return false;
  }
  std::vector<double> sorted;
  sorted.reserve(azimuths_deg.size());
  for (float azimuth_deg : azimuths_deg) {
    if (std::isfinite(azimuth_deg)) {
      sorted.push_back(static_cast<double>(azimuth_deg));
    }
  }
  if (sorted.empty()) {
    return false;
  }
  std::sort(sorted.begin(), sorted.end());
  // 去重：波位表逐 (az, el) 成行，同一方位会出现多次（每行一个俯仰）；不去重则
  // 相邻零间隙会同时污染"最大角间隙"判定与展开累加。
  std::vector<double> distinct;
  distinct.reserve(sorted.size());
  for (double azimuth_deg : sorted) {
    if (distinct.empty() || (azimuth_deg - distinct.back()) > 1.0e-9) {
      distinct.push_back(azimuth_deg);
    }
  }
  const double full_circle_deg = 360.0;
  // 单方位（或全部重合）：扇区退化为一个点，半宽 0，不编造跨度。
  if (distinct.size() == 1U) {
    *center_deg = distinct.front();
    *half_width_deg = 0.0;
    *min_deg = distinct.front();
    *max_deg = distinct.front();
    return true;
  }
  // 缝 = 相邻方位（含首尾环绕）之间的最大角间隙；连续扇区即该缝的**补集**，
  // 从缝的结束端一路到缝的起始端。补集起点取"缝起始端 + 缝宽"再规范化回
  // [-180, 180)——它就是缝另一侧的波位（本场景 158°），终点为起点加补集宽。
  std::size_t seam_start_index = distinct.size() - 1U;  // 环绕缝：起始端 = 最后一个方位
  double largest_gap_deg = (distinct.front() + full_circle_deg) - distinct.back();
  for (std::size_t index = 0U; index + 1U < distinct.size(); ++index) {
    const double gap_deg = distinct[index + 1U] - distinct[index];
    if (gap_deg > largest_gap_deg) {
      largest_gap_deg = gap_deg;
      seam_start_index = index;
    }
  }
  const double span_deg = full_circle_deg - largest_gap_deg;
  double complement_start_deg = distinct[seam_start_index] + largest_gap_deg;
  complement_start_deg = std::fmod(complement_start_deg + 180.0, full_circle_deg);
  if (complement_start_deg < 0.0) {
    complement_start_deg += full_circle_deg;
  }
  complement_start_deg -= 180.0;
  const double min_unwrapped_deg = complement_start_deg;
  const double max_unwrapped_deg = complement_start_deg + span_deg;
  // 中心取连续区间中点后规范化回 [-180, 180)：跨缝扇区的中心即扫描中心
  // （本场景 −202°~−18° → 中心 −110°）。
  double center_unwrapped_deg = 0.5 * (min_unwrapped_deg + max_unwrapped_deg);
  center_unwrapped_deg = std::fmod(center_unwrapped_deg + 180.0, full_circle_deg);
  if (center_unwrapped_deg < 0.0) {
    center_unwrapped_deg += full_circle_deg;
  }
  center_unwrapped_deg -= 180.0;
  *center_deg = center_unwrapped_deg;
  *half_width_deg = 0.5 * span_deg;
  // 以规范化后的中心为锚点输出连续区间，便于验收日志直接读出配置语义。
  // 例如中心 -110°、半宽 92° 输出 -202°/-18°，不会因补集起点的等价表示
  // 落成 158°/342°。
  *min_deg = center_unwrapped_deg - *half_width_deg;
  *max_deg = center_unwrapped_deg + *half_width_deg;
  return true;
}

void WriteRirScanRangeSummary(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                              const std::vector<oneq::common::radar::AzimuthElevationDeg>& pattern,
                              const config::RirMissionConfig& mission,
                              const config::RirOrientationConfig& orientation) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  if (pattern.empty()) {
    return;  // 无波位表：判定字段无有效值，整行省略（§0 规则3）。
  }
  // 2026-09-28 合并：截图第48项要求记录方位/俯仰扫描范围。按实际波位表求上下限，
  // 不另引入配置回显（阵面方向/扫描起始为配置输入，按 §0B 不作为模型输出复写）。
  std::vector<float> azimuths_deg;
  azimuths_deg.reserve(pattern.size());
  float el_min = pattern[0].el_deg;
  float el_max = pattern[0].el_deg;
  for (const oneq::common::radar::AzimuthElevationDeg& point : pattern) {
    azimuths_deg.push_back(point.az_deg);
    if (point.el_deg < el_min) {
      el_min = point.el_deg;
    }
    if (point.el_deg > el_max) {
      el_max = point.el_deg;
    }
  }
  double az_center_deg = 0.0;
  double az_half_width_deg = 0.0;
  double az_continuous_min_deg = 0.0;
  double az_continuous_max_deg = 0.0;
  if (!TryResolveAzimuthSpanDeg(azimuths_deg, &az_center_deg, &az_half_width_deg,
                                &az_continuous_min_deg, &az_continuous_max_deg)) {
    return;  // 无有效方位角：判定字段无有效值，整行省略。
  }
  // 2026-09-28 审计 Q-2：跨缝方位按连续区间输出，避免直接 min/max 得到
  // `-179.600/179.600` 并被误读为全向扫描；字段名称与验收标准第48项保持一致。
  std::string content = "雷达ID=" + std::to_string(radar_id);
  content += " 方位扫描范围=" + FormatF(az_continuous_min_deg, 3) + "/" +
             FormatF(az_continuous_max_deg, 3) + "°";
  content += " 俯仰扫描范围=" + FormatF(el_min, 3) + "/" + FormatF(el_max, 3) + "°";
  // 中译：追加扫描配置回显（用户 2026-10-20 要求）——阵面朝向、可扫描体积、任务子窗、
  //       起始象限、推进顺序、波位步长系数。静态配置只在此行回显一次，不逐驻留重复；
  //       波位总数/间距/单帧时间可由 Rir_scan_pattern.csv 反算，故不落日志。
  // 标识：第 48 项配置回显。
  content += " 阵面方位朝向=" + FormatF(mission.scan_center_deg.az_deg, 3) + "°";
  content += " 阵面俯仰朝向=" + FormatF(mission.scan_center_deg.el_deg, 3) + "°";
  content += " 可扫描方位范围=" + FormatF(orientation.az_min_deg, 3) + "/" +
             FormatF(orientation.az_max_deg, 3) + "°(相对朝向)";
  content += " 可扫描俯仰范围=" + FormatF(orientation.el_min_deg, 3) + "/" +
             FormatF(orientation.el_max_deg, 3) + "°(绝对)";
  content += " 任务扫描子窗=az[" + FormatF(mission.scan_window_deg.az_min_deg, 3) + "," +
             FormatF(mission.scan_window_deg.az_max_deg, 3) + "]el[" +
             FormatF(mission.scan_window_deg.el_min_deg, 3) + "," +
             FormatF(mission.scan_window_deg.el_max_deg, 3) + "]°";
  content += " 扫描起始象限=" + ScanStartPositionText(mission.scan_start_position);
  content += " 二维扫描推进顺序=" + ScanSequenceText(mission.scan_sequence);
  content += " 波位步长系数=" + FormatF(mission.step_scale, 3);
  Emit(sim_time_sec, cycle, "波束扫描功能测试", content);
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
  // 2026-08-31 用户裁定：全向网格 az 0–360° × el -90–90°（az/el 正好无冗余覆盖
  // 全空间；主瓣/副瓣/尾瓣全落在数据内），步长 = 波束宽度（不再用 bw/20 高密度；
  // 下限 0.1° 防窄束配置行数爆炸）。主瓣峰值点 (0,0) 始终在格点上。旧 ±5·bw
  // 自适应网格见评审 2026-08-26 条14（已废止）；同日修订：el 初版 ±180° 有
  // 方向冗余，收敛为 ±90°。
  // 以整数序号生成坐标，避免浮点逐次累加造成网格漂移；输出域、顺序和步长不变。
  // 中译：采样点固定 0.1°（2026-11-02 要求）——原实现步长 = 波束宽度（本机 0.4°），
  //   主瓣只有 1~2 个采样点，外部按数据文件复算波束宽度/旁瓣时分辨率不足，
  //   验收第 28 项要求的参数无法从文件算准。0.1° 下 0.4° 波束有 4 个点、-3 dB 点落在格点上。
  // 标识：方向图采样步长 0.1°（2026-11-02）。
  const double step_deg = 0.1;
  const int az_count = static_cast<int>(std::floor(360.0 / step_deg + 0.5));
  const int el_count = static_cast<int>(std::floor(180.0 / step_deg + 0.5));
  for (int az_index = 0; az_index <= az_count; ++az_index) {
    const float az_deg = static_cast<float>(static_cast<double>(az_index) * step_deg);
    for (int el_index = 0; el_index <= el_count; ++el_index) {
      const float el_deg =
          static_cast<float>(-90.0 + static_cast<double>(el_index) * step_deg);
      dwell::RirAntennaLookOffsetDeg offset;
      offset.delta_az_deg = az_deg;
      offset.delta_el_deg = el_deg;
      const dwell::RirAntennaPatternSample sample =
          dwell::RirEvaluateAntennaPattern(antenna, beamwidth, offset, pointing, 0.0f);
      out << FormatF(az_deg, 2) << "," << FormatF(el_deg, 2) << ","
          << FormatF(sample.gain_dbi, 3) << "\n";
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
