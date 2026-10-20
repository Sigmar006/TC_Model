/**
 * @file RirController.h
 * @brief 远程识别雷达自持链路控制器（内部）。
 *
 * 阶段 2-S 起为完整自持链路：
 *   检测（CFAR/6 dB 回退）→ 量测误差 → 门限最近邻关联 → KF/生命周期
 *   → 内部航迹 → 识别积累。
 * 不消费任何 AR 输出；`RirTrackFeed` 公开输入已退役。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_CONTROLLER_H_
#define REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_CONTROLLER_H_

#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "1q/remote_identification_radar/config/RirMissionConfig.h"
#include "1q/remote_identification_radar/config/RirPolicyConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirOutputTypes.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "remote_identification_radar/dwell/RirSignalDetector.h"
#include "remote_identification_radar/internal/RirPropagationModel.h"
#include "remote_identification_radar/internal/RirRadarEquations.h"
#include "remote_identification_radar/internal/RirSurfaceClutterModel.h"
#include "remote_identification_radar/recognition/RecognitionFeatureDatabase.h"
#include "remote_identification_radar/recognition/RecognitionTracker.h"
#include "remote_identification_radar/tracking/RirTrackAssociator.h"
#include "remote_identification_radar/tracking/RirTrackLifecycle.h"

namespace remote_identification_radar {
namespace runtime {

/**
 * @brief 驻留种类（2026-08-29 TAS 边搜边跟调度语义）。
 */
enum class RirDwellKind {
  kSearch = 0,    /**< 搜索波位（扫描光栅推进）。 */
  kDesignate = 1, /**< 指定识别目标驻留（算子导引，真值指向语义）。 */
  kTrack = 2,     /**< 确认航迹跟踪驻留（库内航迹预测指向）。 */
  /** 外部跟踪驻留（中心 + 邻域窗口；每周期由会话按外推中心重建波位）。
   *  与 kDesignate 的区别：kDesignate 由人工指定目标 ID 驱动、指向该目标位置；
   *  kExternalTrack 由外部位置/速度驱动、指向按速度外推后的中心及其邻域。 */
  kExternalTrack = 3
};

/**
 * @brief 单驻留计划条目：一个周期内波束的一次指向。
 * @note 周期驻留计划由 RirSession 组装（指定 → 确认航迹 → 搜索填充），
 *       控制器逐驻留执行"主瓣覆盖门 + 量测构造"，量测汇入同一周期批次。
 */
struct RirDwellPlan {
  config::RirAzimuthElevationDeg pointing_deg{}; /**< 驻留波束中心（ENU，deg）。 */
  RirDwellKind kind{RirDwellKind::kSearch};      /**< 驻留种类。 */
  std::uint64_t external_target_id{0U};          /**< kDesignate/kTrack 的目标 ID；kSearch 为 0。 */
  std::size_t scan_pattern_index{0U};            /**< kSearch 的波位表序号（验收日志用；其他种类为 0）。 */
};

/**
 * @brief RirController 调度自持检测-跟踪-识别链路。
 */
class RirController {
 public:
  RirController() = default;

  /** @brief 更新运行期上下文（任务域 + 完整策略域）；数据库路径变化时按需加载。 */
  void UpdateRuntime(const config::RirMissionConfig& mission,
                     const config::RirPolicyConfig& policy);

  /** @brief 更新环境域配置（会话初始化或运行期补丁提交后调用）。 */
  void UpdateEnvironment(const config::RirEnvironmentConfig& environment);

  /** @brief 设置静态硬件上下文并重建检测器。 */
  void SetHardware(const config::RirHardwareConfig& hardware);

  /** @brief 设置传感器实体标识（设备 ID / 融合通道；0 为恢复缺省哨兵）。 */
  void SetSensorEntityId(std::uint32_t sensor_entity_id);

  /** @brief 获取当前有效的传感器实体标识（未注入时回退 fusion::kRirSourceId = 5U）。 */
  std::uint32_t EffectiveSensorEntityId() const;

  /**
   * @brief 执行一个自持识别周期。
   * @param[in] input 周期输入（场景目标 + RF 链路）。
   * @param[out] output_frame 识别输出帧（逐内部航迹结论回填）。
   * @param[in] batch_id 本周期会话内部批号（由 RirSession 分配）。
   * @param[in] dwell_center_deg 本周期驻留波束中心（库内驻留调度器给定：
   *            扫描波位或指定识别目标指向；雷达局部 ENU 系，deg）。
   * @param[in] steerable_volume_deg 硬件最大可扫描体积（az 相对 scan_center、
   *            el 绝对，deg）；缺省无界 = 不裁剪（直连调用方与既有行为兼容）。
   *            指定识别目标的驻留/探测门始终以此为界。
   * @param[in] scan_center_deg 转台当前朝向（ENU az/el，deg）。
   * @param[in] scan_window_deg 任务扫描子窗（用户指定的作战搜索扇区，az 相对
   *            scan_center、el 绝对，deg）；缺省无界 = 不额外收窄。实际搜索扇区
   *            = scan_window ∩ steerable_volume：搜索态检测候选按此裁剪。
   * @param[in] designated_external_target_id 当前指定识别目标外部 ID（0 = 无）。
   *            该目标即便落在 scan_window 外，只要仍在 steerable_volume 内即豁免
   *            子窗裁剪（算子显式指派高于常规搜索扇区）。
   * @note 2026-08-22 甲方批注「设定方位俯仰进行扫描」：视线角出扇区的场景目标
   *       不入检测候选集（与指定识别目标驻留门同口径）。
   */
  void RunCycle(const session::RirCycleInput& input, session::RirOutputFrame* output_frame,
                std::uint64_t batch_id,
                const config::RirAzimuthElevationDeg& dwell_center_deg =
                    config::RirAzimuthElevationDeg(),
                const config::RirAzimuthElevationLimitsDeg& steerable_volume_deg =
                    config::RirAzimuthElevationLimitsDeg{-180.0f, 180.0f, -90.0f, 90.0f},
                const config::RirAzimuthElevationDeg& scan_center_deg =
                    config::RirAzimuthElevationDeg(),
                const config::RirAzimuthElevationLimitsDeg& scan_window_deg =
                    config::RirAzimuthElevationLimitsDeg{-180.0f, 180.0f, -90.0f, 90.0f},
                std::uint64_t designated_external_target_id = 0U);

  /**
   * @brief 执行一个自持识别周期（多驻留计划版，TAS 边搜边跟）。
   * @param[in] dwell_plan 本周期驻留计划（逐条目独立执行主瓣覆盖门+量测构造；
   *            同目标同周期被多个驻留照到时首条目胜出）。上一周期全部确认航迹
   *            目标（与其是否入本周期计划无关——预算不足未获跟踪驻留者也然）
   *            连同指定目标，在硬件体积内豁免扫描子窗裁剪（跟踪连续性高于
   *            搜索子窗）。
   * @note 其余参数语义同单驻留重载；RF 接收链按计划首条目指向周期级解析一次
   *       （接收侧聚合语义不变），发射记录逐驻留透出（`emission_frame`
   *       一条驻留一条，多驻留周期不再只记首波位——核查 9.2）。
   */
  void RunCycle(const session::RirCycleInput& input, session::RirOutputFrame* output_frame,
                std::uint64_t batch_id, const std::vector<RirDwellPlan>& dwell_plan,
                const config::RirAzimuthElevationLimitsDeg& steerable_volume_deg =
                    config::RirAzimuthElevationLimitsDeg{-180.0f, 180.0f, -90.0f, 90.0f},
                const config::RirAzimuthElevationDeg& scan_center_deg =
                    config::RirAzimuthElevationDeg(),
                const config::RirAzimuthElevationLimitsDeg& scan_window_deg =
                    config::RirAzimuthElevationLimitsDeg{-180.0f, 180.0f, -90.0f, 90.0f},
                std::uint64_t designated_external_target_id = 0U);

  /**
   * @brief 收集确认航迹的跟踪驻留请求（库内航迹预测指向，不消费场景真值）。
   * @param[in] max_count 上限（会话按周期驻留预算裁剪）。
   * @param[in] predict_dt_sec 预测时移（s）：末量测位置 + 速度 × 时移。
   * @return 跟踪驻留计划（kTrack 条目；至多 max_count 条）。
   */
  std::vector<RirDwellPlan> CollectTrackDwellRequests(std::size_t max_count,
                                                      float predict_dt_sec) const;

  /** @brief 最近周期是否发布了识别效能摘要。 */
  bool HasLatestSummary() const { return has_latest_summary_; }

  /** @brief 最近周期识别效能摘要。 */
  const session::RirRecognitionCycleSummary& GetLatestSummary() const { return latest_summary_; }

  /**
   * @brief 最近周期航迹归属视图（库内键 ↔ 场景真值目标对照）。
   * @note 结果层产品（结果层携带，不进输出帧）；仅已执行周期刷新，
   *       覆盖本周期全部航迹快照（tentative/confirmed/lost）。
   */
  const std::vector<session::RirTrackAttributionRecord>& LatestTrackAttributions() const {
    return last_track_attributions_;
  }

  /**
   * @brief 最近周期按目标门控排除诊断（规则 13b kInfo 执行期条目）。
   * @note 纯观测（不改变周期语义）；每目标每周期至多一条，链上第一门优先；
   *       供 RirSession 并入完成周期 result.issues，供排除差分记录器消费。
   */
  const session::RirIssueList& LatestExecutionIssues() const {
    return last_execution_issues_;
  }

  /** @brief 最近周期实际 RIR 发射帧（`kIdentify` 且 RF 链成功时非空；
   *         逐驻留一条记录，多驻留周期携带全部驻留发射）。 */
  const oneq::electromagnetics::RfEmissionFrame& LatestEmissionFrame() const {
    return last_emission_frame_;
  }

  /**
   * @brief 最近周期「实际有效目标最大斜距」（m）：本周期持有航迹的目标里最大输入几何斜距。
   * @note 供 RirSession 回填 result.max_detected_slant_range_m；本周期无航迹为 0，
   *       非 kIdentify / 早退周期为 0（RunCycle 入口重置）。
   */
  float LastMaxDetectedSlantRangeM() const { return last_max_detected_slant_range_m_; }

  /**
   * @brief 指定外部目标是否已达识别结论（上一周期口径，供任务生命周期推进）。
   * @param[in] external_target_id 外部目标 ID。
   * @return 该目标任一航迹的识别状态达 kCategoryConfirmed/kModelConfirmed 时为 true。
   * @note 读取上一周期航迹快照（与指向同源滞后一周期，镜像 AR designation 口径）。
   */
  bool IsTargetRecognized(std::uint64_t external_target_id) const;

  /** @brief 当前生效识别特征数据库版本（供 replay 溯源）。 */
  const std::string& ActiveDatabaseVersion() const { return tracker_.ActiveDatabaseVersion(); }

  /** @brief 当前检测随机种子（供 replay 状态溯源）。 */
  std::uint32_t DetectionRandomSeed() const { return detection_random_seed_; }

  /** @brief 最近一次识别库加载耗时（毫秒）；未加载过为 0。 */
  double LastDatabaseLoadMs() const { return last_database_load_ms_; }

  /** @brief 由距离/角度量测误差构造笛卡尔量测协方差（纯函数；测试/诊断消费）。 */
  static tracking::RirMeasurementCovariance MakeCartesianMeasurementCovariance(
      const session::RirSceneTarget& target, float range_std_m, float angle_std_rad);

 private:
  struct RirResolvedRfCycle {
    bool resolved{false};
    bool receiver_saturated{false};
    oneq::electromagnetics::RfEmissionIdentity own_emission_identity{};
    oneq::electromagnetics::RfSceneEmission own_emission{};
    /** 逐驻留发射记录（首条 = own_emission；接收链仍按首驻留周期级一次求解）。 */
    std::vector<oneq::electromagnetics::RfSceneEmission> per_dwell_emissions{};
    std::vector<oneq::electromagnetics::RfIncidentLinkResult> incident_links{};
    oneq::electromagnetics::RfWaveformSchedule own_transmit_waveform{};
    float carrier_hz{0.0f};
  };

  /** @brief 检测器配置装配。 */
  dwell::RirDetectorConfig MakeDetectorConfig() const;

  /**
   * @brief 航迹输出门：物理包络过滤 + 同目标去重（B6，2026-10-20）。
   *
   * 两步处理只作用于**输出用**的航迹快照（不改变滤波器内部状态，越界航迹由
   * `RirTrackLifecycle::DropTrack` 真正丢弃）：
   * 1. 包络门：斜距超过任务最大作用距离、高度超过弹道顶点余量、高度低于地面或
   *    非有限量 ⇒ 判为滤波器发散，丢弃该航迹并从快照中移除；
   * 2. 去重：同一 `external_target_id` 只保留最优一条（确认优先 → 位置不确定度小
   *    → 命中次数多），其余只留在滤波器内部，不再重复输出。
   *
   * @param[in] platform_lla 雷达站 LLA 锚点（ENU→ECEF 必需）。
   * @param[in] has_platform_lla 锚点是否可用；不可用则跳过高度判定、仅按斜距门。
   * @param[in,out] snapshots 本周期输出用航迹快照（会被就地裁剪）。
   */
  void GateTrackSnapshots(const oneq::coordinate::LlaPositionDegM& platform_lla,
                          bool has_platform_lla,
                          std::vector<tracking::RirTrackState>* snapshots);

  /**
   * @brief 解析本周期 RF 前端（自发射 + 可选外部 scene）与逐驻留发射记录。
   * @note 接收侧（饱和/入射链/波形/身份）按计划首驻留指向周期级求解一次
   *       （聚合语义）；发射记录逐驻留构建（首条复用周期级自发射，其余驻留
   *       同工厂路径、指向换该驻留、emission_id 唯一化——核查 9.2）。
   */
  RirResolvedRfCycle ResolveRfCycle(const session::RirCycleInput& input,
                                    const std::vector<RirDwellPlan>& plan) const;

  /** @brief 环境事实解析：未启用环境效应时退化到阶段 1 旧口径。 */
  void ResolveEnvironment(float* propagation_loss_db) const;

  /**
   * @brief 仅在种子**真正变化**时重设检测/量测随机数发生器（2026-10-20）。
   *
   * @details 无条件重设会让每周期第一次抽签恒为同一值（CFAR 概率判决退化为固定门限）。
   *   本函数记住"已施加的种子"，同一 seed 连续下发时保持发生器状态自然推进。
   */
  void ApplyDetectionSeedIfChanged();

  /**
   * @brief 逐目标主瓣地杂波等效噪声功率（植被开启时物理求解，关闭恒 0）。
   * @param[in] carrier_hz 周期有效载频（规划频率优先，回退发射机频率）。
   * @param[in] bandwidth_hz 匹配滤波带宽（决定杂波距离单元 c/(2B)）。
   */
  float ResolveTargetClutterPowerW(float look_el_deg, float slant_range_m,
                                   float total_propagation_loss_db, float carrier_hz,
                                   float bandwidth_hz, float platform_altitude_m) const;

  /**
   * @brief 逐目标大气物理附加损耗（common 大气单源，与 AR 同口径）。
   * @note 大气物理关闭时返回 0；k 因子经 UpdateEnvironment 预派生（非 4/3 硬值）。
   */
  float ComputeTargetAtmosphericLossDb(float carrier_hz, float platform_altitude_m,
                                       float look_el_deg, bool has_look_angles,
                                       float slant_range_m, float target_position_z_m) const;

  /** @brief 单个目标检测与量测构造；未通过门控返回空 optional 语义（out=false）。 */
  bool TryBuildMeasurement(const session::RirSceneTarget& target, std::size_t source_index,
                           float platform_altitude_m, float propagation_loss_db,
                           const session::RirCycleInput& input,
                           const config::RirAzimuthElevationDeg& dwell_center_deg,
                           const RirResolvedRfCycle& rf_cycle,
                           tracking::RirTrackMeasurement* measurement, float* snr_db);

  /** @brief 计算目标视线角。 */
  static void ComputeLookAngles(const session::RirSceneTarget& target, float* look_az_deg,
                                float* look_el_deg, float* slant_range_m);

  /** @brief 计算平台相对位置（ENU 直角坐标，m）的视线角。 */
  static void LookAnglesFromPosition(const Eigen::Vector3f& position, float* look_az_deg,
                                     float* look_el_deg);

  /** @brief 在笛卡尔量测协方差下采样量测位置。 */
  tracking::RirTrackMeasurement SampleMeasurementPosition(
      const tracking::RirTrackMeasurement& measurement);

  /** @brief 单目标识别观测上下文（视线角、SNR、带宽、驻留）。 */
  recognition::RirObservationContext MakeObservationContext(const session::RirSceneTarget& target,
                                                            float platform_altitude_m,
                                                            float snr_db) const;

  config::RirHardwareConfig hardware_{};
  config::RirMissionConfig mission_{};
  config::RirPolicyConfig policy_{};
  config::RirEnvironmentConfig environment_{};
  // 大气 k 因子（UpdateEnvironment 时按气象观测派生，缺省 4/3，与 AR 冻结口径一致）。
  float effective_k_factor_{4.0f / 3.0f};
  std::uint32_t sensor_entity_id_{0U};
  config::RirWorkMode work_mode_{config::RirWorkMode::kStby};
  bool recognition_mode_active_{false};
  float sim_time_sec_{0.0f};
  std::uint32_t detection_random_seed_{42U};
  /**
   * @brief 已实际施加到检测/量测随机数发生器的种子（2026-10-20）。
   *
   * @details 此前 `UpdateRuntime`/`SetHardware` 无条件重设种子，而宿主每周期下发
   *   运行时补丁 ⇒ 检测链的 CFAR 抽签**每周期被重置到同一个值**，概率判决退化为
   *   `Pd >= 首抽值`（实测首抽 ≈ 0.0105，于是 1,334 条"统计检测结果"全是"是"）。
   *   改为只有种子真正变化时才重设，发生器状态随之自然推进。
   */
  std::uint32_t applied_detection_seed_{42U};
  bool detection_seed_applied_{false};

  std::unique_ptr<dwell::RirSignalDetector> detector_{};
  tracking::RirTrackAssociator associator_{};
  // 池化生命周期管理器不可拷贝，经 unique_ptr 持有以保持控制器可移动。
  std::unique_ptr<tracking::RirTrackLifecycle> lifecycle_{new tracking::RirTrackLifecycle()};
  internal::RirPropagationModel propagation_model_{};
  internal::RirSurfaceClutterModel surface_clutter_model_{};
  std::mt19937 measurement_rng_{42U};

  std::unique_ptr<recognition::RirFeatureDatabase> database_{};
  std::string database_path_{};
  double last_database_load_ms_{0.0};
  recognition::RirTracker tracker_{};

  session::RirRecognitionCycleSummary latest_summary_{};
  bool has_latest_summary_{false};
  std::vector<tracking::RirTrackState> last_track_snapshots_{}; /**< 上一周期航迹快照（任务判定用）。 */
  std::vector<session::RirTrackAttributionRecord> last_track_attributions_{}; /**< 最近周期归属视图。 */
  session::RirIssueList last_execution_issues_{}; /**< 最近周期按目标排除诊断（规则 13b）。 */
  oneq::electromagnetics::RfEmissionFrame last_emission_frame_{}; /**< 最近周期实际发射。 */
  float last_max_detected_slant_range_m_{0.0f}; /**< 最近周期持航迹目标最大输入斜距（m）。 */
  /** 本周期逐目标检测 SNR（单脉冲口径）；归属视图与调试视图透出，非验收统计项。 */
  std::unordered_map<std::uint64_t, float> last_cycle_snr_by_target_id_{};
};

}  // namespace runtime
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_CONTROLLER_H_
