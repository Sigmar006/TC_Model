/*
 * Supplemental architecture notes for acceptance review.
 * Unit: RirAcceptanceRecords.
 * Model: LRDR remote identification radar.
 * Layer: runtime.
 * File role: interface declarations and data contracts.
 * Responsibility: top-level RF, detection, tracking, and recognition orchestration.
 * Upstream boundary: validated cycle input, dwell plan, runtime policy, and environment.
 * Downstream boundary: cycle products, emissions, diagnostics, and attribution snapshots.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

/**
 * @file RirAcceptanceRecords.h
 * @brief RIR 验收行拼装（检测链拆项、航迹/识别、调度、天线 CSV）。
 */

#ifndef ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_ACCEPTANCE_RECORDS_H_
#define ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_ACCEPTANCE_RECORDS_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/electromagnetics/RfScene.h"
#include "1q/coordinate/types.h"
#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "1q/remote_identification_radar/session/RirSceneTypes.h"
#include "common/radar/AntennaPatternRuntime.h"
#include "remote_identification_radar/dwell/RirDetectionCellResolver.h"

namespace remote_identification_radar {

namespace tracking {
struct RirTrackState;
struct RirAssociationResult;
}  // namespace tracking

namespace session {
struct RirRecognitionResult;
struct RirRecognitionCycleSummary;
struct RirFeatureMeasurementRecord;
struct RirOutputFrame;
}  // namespace session

namespace runtime {

struct RirDwellPlan;  /**< 前置声明：逐驻留验收行消费（完整定义见 RirController.h）。 */

/** @brief 航迹对应真值上下文（验收判定标准 第38/42/47项：量测/估计误差与散射中心）。 */
struct RirTrackTruthContext {
  bool has_look{false};        /**< 真值视线极坐标可用（ENU 自 +x 东起量）。 */
  float truth_range_m{0.0f};   /**< 真值斜距（m）。 */
  float truth_az_deg{0.0f};    /**< 真值方位角（deg）。 */
  float truth_el_deg{0.0f};    /**< 真值俯仰角（deg）。 */
  bool has_ecef{false};        /**< 真值 ECEF 位置/速度可用（滤波误差指标用）。 */
  oneq::coordinate::EcefPositionM position_ecef{};
  oneq::coordinate::EcefVelocityMps velocity_ecef{};
  /** 第47项散射中心：场景距离像真值输入（库内特征提取只出统计量，无逐中心量测）。 */
  const std::vector<session::RirRangeRcsScatterer>* scatterers{nullptr};
};

struct RirDetectionAcceptInput {
  float sim_time_sec{0.0f};
  std::uint32_t cycle{0U};
  std::uint64_t radar_id{1U};  /**< 雷达实体 ID（验收行 雷达ID= 标注）。 */
  std::uint64_t target_id{0U};
  float range_m{0.0f};
  float look_az_deg{0.0f};
  float look_el_deg{0.0f};
  float rcs_m2{0.0f};
  float snr_db{0.0f};
  float pd{0.0f};
  bool detected{false};
  bool has_cell{false};
  float peak_gain_dbi{0.0f};
  float bw_az_deg{0.0f};
  float bw_el_deg{0.0f};
  double one_way_antenna_gain_dbi{0.0};
  double bandwidth_hz{0.0};
  double echo_power_dbw{0.0};
  dwell::RirDetectionCellResult cell{};
  config::hardware::RirSignalProcessingConfig gains{};
  double prf_hz{0.0};
  double center_frequency_hz{0.0};
  double cfar_pfa{1.0e-6};
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> incident_links{};
};

std::string ResolveRirAntennaPatternCsvPath();
std::string ResolveRirScanPatternCsvPath();

void WriteRirAntennaPatternSummary(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                                   float peak_gain_dbi, float bw_az_deg, float bw_el_deg);

void WriteRirDetectionChain(const RirDetectionAcceptInput& input);

void WriteRirInterferenceLinks(
    float sim_time_sec, std::uint32_t cycle,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& links);

void WriteRirSearchDetections(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                              float beam_az_deg, float beam_el_deg,
                              const std::string& found_targets);

/**
 * @param[in] platform_lla 平台 LLA（雷达局部 ENU 的绝对锚点；量测位置 LLA 换算用）。
 * @param[in] gate_threshold 关联波门（马氏距离² 门限，无量纲；行内量纲说明用）。
 */
void WriteRirAssociation(float sim_time_sec, std::uint32_t cycle,
                         const tracking::RirAssociationResult& association,
                         const oneq::coordinate::LlaPositionDegM& platform_lla,
                         double gate_threshold);

/** @brief 第64项：将同周期检测量测聚合写入 RadarInterface.log。 */
void WriteRirExternalMeasurements(float sim_time_sec, std::uint32_t cycle,
                                  const tracking::RirAssociationResult& association);

void WriteRirClusterCount(float sim_time_sec, std::uint32_t cycle,
                          const std::vector<tracking::RirTrackState>& tracks);

/**
 * @brief 雷达局部 ENU 位置换斜距与视线角（与 RirController::ComputeLookAngles 同口径）。
 * @param[in] east_m 东向坐标（m）。
 * @param[in] north_m 北向坐标（m）。
 * @param[in] up_m 天向坐标（m）。
 * @param[out] range_m 斜距（m）。
 * @param[out] az_deg 方位角（deg，自 +x 东起量）。
 * @param[out] el_deg 俯仰角（deg）。
 * @return 输出指针有效、坐标有限且位置范数 > 0.1 m 时为 true。
 */
bool TryLookPolarFromEnuM(float east_m, float north_m, float up_m, float* range_m, float* az_deg,
                          float* el_deg);

/**
 * @param[in] platform_ecef 平台 ECEF 位置（雷达局部 ENU 的绝对锚点；航迹位置
 *            ECEF/LLA 换算用——评审 2026-08-26 条17：ENU 字段替换为 ECEF/LLA）。
 * @param[in] truth 对应场景真值上下文（可空指针 = 无真值，误差统计行省略）。
 */
void WriteRirTrackAndId(float sim_time_sec, std::uint32_t cycle,
                        const tracking::RirTrackState& track,
                        const session::RirRecognitionResult* result,
                        const session::RirFeatureMeasurementRecord* features,
                        bool has_truth, double category_accuracy,
                        const std::vector<float>* imm_weights,
                        const oneq::coordinate::EcefPositionM& platform_ecef,
                        const RirTrackTruthContext* truth);

/** @brief 第65项：将同周期航迹、协方差、识别和特征聚合写入 RadarInterface.log。 */
void WriteRirExternalTracks(float sim_time_sec, std::uint32_t cycle,
                            const std::vector<tracking::RirTrackState>& tracks,
                            const session::RirOutputFrame& output_frame);

/**
 * @brief 验收判定标准 第37项·其二：扫描调度信息行（搜索/指定/跟踪分项计数 +
 *        预算/已耗时；2026-08-30 核查 9.1 删除恒等的计划/实际驻留数字段）。
 */
void WriteRirSchedule(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                      float budget_sec, float consumed_sec, std::uint32_t search_count,
                      std::uint32_t designate_count, std::uint32_t track_count,
                      std::uint32_t confirmed_tracks);

void WriteRirOncePerSession(float sim_time_sec, std::uint32_t cycle);
void WriteRirCycleRunCount(float sim_time_sec, std::uint32_t cycle);

/**
 * @brief 验收判定标准 第48项：逐驻留波束指向行（2026-08-29 TAS：一周期多驻留）。
 * @note 搜索驻留每条一行（保"本周期序号/波位"既有字段语义）；指定/跟踪驻留
 *       各一行，带驻留种类与目标 ID 标记。
 */
void WriteRirDwellScan(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                       const std::vector<RirDwellPlan>& dwell_plan);

bool TryExportRirAntennaPatternCsv(const config::hardware::RirAntennaConfig& antenna,
                                   const char* path);

bool TryExportRirScanPatternCsv(
    const std::vector<oneq::common::radar::AzimuthElevationDeg>& pattern, const char* path);

}  // namespace runtime
}  // namespace remote_identification_radar

#endif
