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
#include "1q/remote_identification_radar/config/RirMissionConfig.h"
#include "1q/remote_identification_radar/config/RirOrientationConfig.h"
#include "1q/remote_identification_radar/config/RirPolicyConfig.h"
#include "1q/remote_identification_radar/session/RirOutputTypes.h"
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
  /**
   * @brief RCS 口径（2026-10-26）：物理模型开启时 `rcs_m2` 是**视角 RCS**（随视线角变化），
   *        关闭时它是**外部输入的标称 RCS**（场景给的值）。验收行据此标注口径。
   */
  bool rcs_physics_enabled{false};
  float nominal_rcs_m2{0.0f};
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

/**
 * @brief 第28项：从实际方向图采样反算峰值增益/波束宽度/最大旁瓣/后瓣电平，与配置逐项
 *        比较后按主文档 `5%` 判据输出各项判据与整体结果。
 * @note 2026-09-28 复核后本项**只此一个**输出入口：早期的
 *       `WriteRirAntennaPatternSummary`（直接打印配置峰值/宽度）已删除，避免与
 *       “反算校验”并存造成自证。
 */
void WriteRirAntennaPatternDerived(std::uint64_t radar_id, float sim_time_sec,
                                   std::uint32_t cycle,
                                   const config::hardware::RirAntennaConfig& antenna);

/**
 * @brief 波位表方位角跨度（跨 ±180° 缝合线时给出真实连续扇区）。
 * @param[in] azimuths_deg 波位方位角（deg，已规范化到 [-180, 180]；同一方位可重复，
 *            函数内部按 1e-9 容差去重，以适配"逐 (az, el) 成行"的波位表）。
 * @param[out] center_deg 扇区中心（deg，连续区间中点规范化回 [-180, 180)）。
 * @param[out] half_width_deg 半宽（deg，非负；单方位时为 0）。
 * @param[out] min_deg 连续展开区间下界（deg，可超出 ±180）。
 * @param[out] max_deg 连续展开区间上界（deg，可超出 ±180）。
 * @return 输出指针有效且输入至少含一个有限方位角时为 true；否则不修改输出。
 * @note 2026-09-28 审计 Q-2：波位表由"扫描中心 + 相对扇区"生成后经
 *       `NormalizeAzimuthDeg` 折到 ±180°，跨缝表（如中心 −110°、宽 184°）直接求
 *       min/max 会得 `-179.6/179.6`，读起来像全向扫描。本函数以"最大角间隙 = 缝"
 *       回推真实扇区，输出连续区间（本场景 −202°~−18°）。
 * @note 判据前提：波位在扇区内足够密集，使 `360° − 扇区宽` 的接缝间隙大于任何
 *       带内间隙。本模型扫描生成器的步长 = 波束宽 × `step_scale`，远小于扇区宽度，
 *       前提成立；若输入是极稀疏的两个点簇，函数会按同一判据给出较小的那一侧区间
 *       （结果仍自洽，只是分辨率不足），不属计算错误。
 */
bool TryResolveAzimuthSpanDeg(const std::vector<float>& azimuths_deg, double* center_deg,
                              double* half_width_deg, double* min_deg, double* max_deg);

void WriteRirDetectionChain(const RirDetectionAcceptInput& input);

/**
 * @brief 验收判定标准 第30项「干扰功率计算功能测试」：逐**外部**干扰源写出到达功率。
 * @param[in] own_emission_identity 本雷达本周期的发射身份；身份相同的链路（自身同址
 *            耦合）不写出——本项记录的是外部干扰源，同址耦合不是干扰源。
 *            剔除谓词与检测链共用 `RfEmissionIdentity` 的 `sensor_entity_id +
 *            emission_id` 二元组同一性。
 * @note 2026-09-28 审计 Q-1：早期实现直接遍历 `incident_links`，把控制器为饱和门
 *       有意追加的自身发射写成"干扰源"，与同周期第 29 项 `干扰功率=0`、第 35 项
 *       整条省略互相矛盾（本场景会读出 +60.6 dB 的假干扰）。
 */
void WriteRirInterferenceLinks(
    float sim_time_sec, std::uint32_t cycle,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& links);

/**
 * @brief 从入射链中挑出**外部**干扰源（剔除自身同址耦合链），并保持输入顺序。
 *
 * 契约：本机发射身份由 `sensor_entity_id + emission_id` 二元组判定；被追加进场景的
 * 自身链一律剔除，其余链路原序保留（含功率为 0 的合法外部链）。
 * @param[in] links 周期级入射链（含自身链）。
 * @param[in] own_emission_identity 本雷达本周期发射身份；无效身份（任一字段为 0）时
 *            不做剔除（无法判定何者为自身）。
 * @return 外部干扰链（值拷贝，便于测试与日志写出共用）。
 */
std::vector<oneq::electromagnetics::RfIncidentLinkResult> SelectRirExternalInterferenceLinks(
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& links);

/** @brief 第63项：将外部 RF 场景中的干扰发射输入摘要写入 RadarOutput.log。 */
void WriteRirExternalInterferenceInput(float sim_time_sec, std::uint32_t cycle,
                                       const oneq::electromagnetics::RfSceneFrame& rf_scene);

/** @brief 第62项：将目标运动状态与电磁特征输入写入 RadarOutput.log。 */
void WriteRirExternalSceneTargets(float sim_time_sec, std::uint32_t cycle,
                                  const session::RirSceneTargetList& targets);

/** @brief 第37项「检测与量测信息」逐检测目标量测（Pd 归第36项，本条不写）。 */
struct RirSearchDetectionLite {
  std::uint64_t target_id{0U};
  float range_m{0.0f};
  float az_deg{0.0f};
  float el_deg{0.0f};
};

/**
 * @brief 验收判定标准 第37项「空域自主搜索功能测试」：指向信息行 + 检测与量测信息行
 *        （扫描调度信息另见 `WriteRirSchedule`）。
 */
void WriteRirSearchDetections(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                              float beam_az_deg, float beam_el_deg,
                              const std::vector<RirSearchDetectionLite>& detections);

/**
 * @brief 验收判定标准 第37A项「地平线搜索功能测试」：与第37项同批字段、独立项名。
 */
void WriteRirHorizonSearchDetections(std::uint64_t radar_id, float sim_time_sec,
                                     std::uint32_t cycle, float beam_az_deg, float beam_el_deg,
                                     const std::vector<RirSearchDetectionLite>& detections);

/**
 * @param[in] platform_lla 平台 LLA（雷达局部 ENU 的绝对锚点；量测位置 LLA 换算用）。
 * @param[in] gate_threshold 关联波门（马氏距离^2 门限，无量纲；行内量纲说明用）。
 */
void WriteRirAssociation(float sim_time_sec, std::uint32_t cycle,
                         const tracking::RirAssociationResult& association,
                         const oneq::coordinate::LlaPositionDegM& platform_lla,
                         double gate_threshold);

/** @brief 第64项：将对外目标信息的点迹量测层写入 RadarOutput.log。 */
void WriteRirExternalMeasurements(float sim_time_sec, std::uint32_t cycle,
                                  const tracking::RirAssociationResult& association);

void WriteRirClusterCount(float sim_time_sec, std::uint32_t cycle,
                          const std::vector<tracking::RirTrackState>& tracks);

/**
 * @brief 第36项：按目标检测概率反解最大探测距离（四次方根距离定律）。
 *
 * `R = R_now · (SNR_now / SNR_req)^(1/4)`，其中 `SNR_req` 由同一 Marcum Q 检测模型对
 * `pd_target` 二分反解。方向约定：当前 SNR 高于所需值时 R > R_now（目标可以更远），
 * 低于时 R < R_now；当前 SNR 越高，反解距离单调不减。
 *
 * @param[in]  current_range_m 当前斜距（m，须 > 0）。
 * @param[in]  current_snr_db  当前单脉冲 SNR（dB）。
 * @param[in]  pd_target       目标检测概率（0,1）。
 * @param[in]  pfa             虚警概率（0,1）。
 * @param[in]  num_pulses      积累脉冲数（≥1）。
 * @param[out] max_range_m     反解出的最大探测距离（m）。
 * @return 入参合法且反解有解时为 true；否则不改输出。
 */
bool TryComputeDetectionRangeAtPd(double current_range_m, double current_snr_db,
                                  double pd_target, double pfa, int num_pulses,
                                  double* max_range_m);

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
                        const config::RirRecognitionFeatureWeights& feature_weights,
                        bool has_truth, double category_accuracy,
                        const std::vector<float>* imm_weights,
                        const oneq::coordinate::EcefPositionM& platform_ecef,
                        const RirTrackTruthContext* truth);

/** @brief 第65项：将对外目标信息的航迹识别层写入 RadarOutput.log。 */
void WriteRirExternalTracks(float sim_time_sec, std::uint32_t cycle,
                            const std::vector<tracking::RirTrackState>& tracks,
                            const session::RirOutputFrame& output_frame);

/** @brief 第66项：将工作模式、航迹容量、驻留队列和告警写入 RadarOutput.log。 */
void WriteRirExternalResourceStatus(
    std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
    config::RirWorkMode work_mode, const std::vector<RirDwellPlan>& dwell_plan,
    const std::vector<tracking::RirTrackState>& tracks, std::size_t track_pool_capacity,
    std::size_t track_pool_free_count, const session::RirIssueList& issues);

/**
 * @brief 验收判定标准 第37项·其二：扫描调度信息行（搜索/指定/驻留分项计数 +
 *        预算/已耗时；2026-08-30 核查 9.1 删除恒等的计划/实际驻留数字段）。
 * @note 2026-10-22 用户要求：库内跟踪与外部跟踪合并为一项 `驻留=` 输出（内部仍按
 *       kTrack / kExternalTrack 两种调度语义分别排波位，仅日志措辞统一）。
 */
void WriteRirSchedule(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                      float budget_sec, float consumed_sec, std::uint32_t search_count,
                      std::uint32_t designate_count, std::uint32_t external_track_count,
                      std::uint32_t internal_track_count,
                      std::uint32_t confirmed_tracks);

/**
 * @brief 验收判定标准 第48项：逐驻留波束指向行（2026-08-29 TAS：一周期多驻留）。
 * @note 每周期先写一行 `本周期搜索驻留数=N`，再逐驻留写指向行：搜索驻留每条一行，
 *       字段 `波位序号=`（波位表游标，可索引 `Rir_scan_pattern.csv`）；非搜索驻留各
 *       一行，`驻留种类=` 只取 `指定驻留` / `驻留` 两种取值（2026-10-22 合并命名）。
 *       2026-10-21 起原名 `本周期序号` 更名 `波位序号`，
 *       消除"周期内序号"的误读（该值跨周期连续递增并在波位表长度处回绕）。
 */
void WriteRirDwellScan(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                       const std::vector<RirDwellPlan>& dwell_plan);

/**
 * @brief 验收判定标准 第48项：波束扫描范围摘要（扫描上下限 + 扫描配置回显）。
 *
 * 与 `TryExportRirScanPatternCsv` 同源调用（波位表生成处），按实际波位表求方位/俯仰
 * 最小/最大值；空表不写该行（判定字段无有效值）。
 *
 * 2026-10-20 追加配置回显（用户要求）：阵面方位/俯仰朝向、可扫描体积、任务扫描子窗、
 * 扫描起始象限、二维扫描推进顺序、波位步长系数。**静态配置只在本行回显一次**，
 * 不逐驻留重复；波位总数/间距/单帧扫描时间可由 `Rir_scan_pattern.csv` 反算，故不落日志。
 */
void WriteRirScanRangeSummary(std::uint64_t radar_id, float sim_time_sec, std::uint32_t cycle,
                              const std::vector<oneq::common::radar::AzimuthElevationDeg>& pattern,
                              const config::RirMissionConfig& mission,
                              const config::RirOrientationConfig& orientation);

/**
 * @brief 导出方向图增益 CSV（列：`az_off_deg,el_off_deg,gain_dbi`）。
 *
 * 网格覆盖 az 0–360°、el -90–90°，步长取波束宽度且不小于 0.1°。
 * 格点由整数序号与步长相乘生成，避免浮点逐次累加造成坐标漂移。
 * @note 极区/钳位处的增益是方向图模型的数值下限，不是物理电平。
 * @return 文件成功打开并写完返回 true；打不开返回 false。
 */
bool TryExportRirAntennaPatternCsv(const config::hardware::RirAntennaConfig& antenna,
                                   const char* path);

bool TryExportRirScanPatternCsv(
    const std::vector<oneq::common::radar::AzimuthElevationDeg>& pattern, const char* path);

}  // namespace runtime
}  // namespace remote_identification_radar

#endif
