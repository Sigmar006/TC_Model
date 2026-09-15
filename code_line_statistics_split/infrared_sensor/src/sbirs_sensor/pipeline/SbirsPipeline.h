/**
 * @file SbirsPipeline.h
 * @brief SBIRS-inspired WFOV/NFOV pipeline。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_PIPELINE_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_PIPELINE_H_

#include <cstdint>
#include <map>
#include <vector>

#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsExternalCue.h"
#include "sbirs_sensor/config/SbirsInternalExecutionConfig.h"
#include "sbirs_sensor/foundation/SbirsErrorModel.h"
#include "sbirs_sensor/pipeline/SbirsCuePredictor.h"
#include "sbirs_sensor/pipeline/SbirsNfovScheduler.h"
#include "sbirs_sensor/pipeline/SbirsPointingCoordinator.h"
#include "sbirs_sensor/pipeline/SbirsTrackingCoordinator.h"
#include "sbirs_sensor/runtime/SbirsRuntimeConfigImpact.h"

namespace sbirs_sensor {
namespace pipeline {

/** @brief 目标级状态机 7 状态枚举（见 algorithms.md 目标状态机），驱动 WFOV 发现、NFOV 首次捕获与持续跟踪。 */
enum class SbirsTargetState {
  kUndetected = 0,          /**< 初始或目标未被任何视场发现 */
  kWideCandidate,           /**< WFOV 已发现，等待 NFOV 资源调度 */
  kAwaitingNfovAcquisition, /**< 已入 NFOV 锁定集合，等待轮转窗口执行首次捕获 */
  kStrictTruthAssistedTracking, /**< 真值 LOS 驱动并输出精确真值 */
  kSensorLikeTruthAssistedTracking, /**< 真值 LOS 驱动并输出带误差观测 */
  kEstimatedTracking,       /**< EKF/IMM/角度标准 KF 滤波测量跟踪 */
  kLost                     /**< 目标从输入场景消失或传感器关闭 */
};

/**
 * @brief pipeline 运行期状态快照，用于 validated checkpoint 与确定性 continuation。
 * @note 包含扫描相位、目标状态表、NFOV 锁定集合与轮转步进、随机源状态与 EKF 滤波状态表。
 */
struct SbirsPipelineSnapshot {
  float scan_phase_deg{0.0f};          /**< 当前 WFOV 有向扫描相位（行内方位相位，往复腿内距起点的距离），范围 [0, span) deg */
  bool scan_leg_forward{true};         /**< 往复腿方向：true=起点→终点（scan_direction 初始方向），false=回程 */
  float scan_azimuth_base_deg{0.0f};   /**< 最近执行周期的方位基准（nadir=星下点方位；absolute=0），ApplyConfig 重锚用 */
  int scan_row_index{0};               /**< 当前 WFOV 俯仰栅格行索引，范围 [0, row_count) */
  float misalignment_yaw_deg{0.0f};    /**< 运行期安装失准角总量（常值偏置 + 一次随机抽取），deg */
  float misalignment_pitch_deg{0.0f};  /**< 运行期安装失准角总量 pitch，deg */
  float misalignment_roll_deg{0.0f};   /**< 运行期安装失准角总量 roll，deg */
  std::uint64_t next_detection_id{1U}; /**< 下一个检测记录 ID */
  std::map<std::uint64_t, SbirsTargetState>
      target_states{};                         /**< 各目标状态表（按 target_id 索引） */
  std::map<std::uint64_t, unsigned int> wfov_consecutive_hits{}; /**< WFOV 连续命中计数表（宽窄切换前置条件） */
  SbirsNfovSchedulerSnapshot nfov_scheduler{}; /**< NFOV 锁定集合状态（目标→引导来源） */
  std::uint64_t nfov_rotation_step{0U}; /**< 单镜筒轮转步进（每执行周期 +1；决定服务目标选择） */
  std::uint32_t wfov_measurement_random_state{1U}; /**< WFOV/cue 量测随机状态 */
  std::uint32_t estimated_measurement_random_state{1U}; /**< Estimated 校正量测随机状态 */
  std::uint32_t sensor_like_output_random_state{1U}; /**< Sensor-like 输出随机状态 */
  std::map<std::uint64_t, tracking::SbirsGaussianState>
      filter_states{}; /**< EKF 滤波状态表（kEstimatedTracking 且非 kAngleCvKf 的均值+协方差） */
  std::map<std::uint64_t, tracking::SbirsAngleCvGaussianState>
      angle_kf_states{}; /**< 实验角度 CV 线性 KF 状态表（kAngleCvKf） */
  std::map<std::uint64_t, unsigned int> nis_gate_exceeded_counts{}; /**< EKF NIS 连续超限计数 */
  bool imm_active{false}; /**< 当前 snapshot 是否使用 IMM 模式 */
  std::map<std::uint64_t, tracking::SbirsImmSnapshot> imm_snapshots{}; /**< IMM 滤波状态表 */
  SbirsCuePredictorSnapshot cue_predictor{};               /**< 测量驱动 cue predictor 逐目标历史 */
  SbirsPointingCoordinatorSnapshot pointing_coordinator{}; /**< 单镜筒 NFOV ATP 状态（共享执行器+逐目标簿记） */
};

/** @brief 单条 pipeline 内部检测结果，组合原始记录与归属。 */
struct SbirsPipelineDetection {
  output::SbirsDetectionRecord record{};                      /**< 原始观测记录 */
  attribution::SbirsDetectionAttributionRecord attribution{}; /**< 仿真归属记录 */
};

/** @brief 单周期 pipeline 执行结果，含扫描相位与本周期检测列表。 */
struct SbirsPipelineResult {
  float scan_azimuth_rad{0.0f};        /**< 本周期扫描方位角（ECI 极坐标，单位 rad，[0, 2π)） */
  float scan_elevation_rad{0.0f};      /**< 本周期扫描中心俯仰角（ECI 极坐标，单位 rad，[-π/2, π/2]；2-D 栅格下为当前行中心） */
  std::vector<SbirsPipelineDetection> detections{}; /**< 检测列表 */
  bool executed{false};                             /**< 核心 pipeline 是否实际执行（非关机/待机） */
  std::vector<session::SbirsWideCueMeasurement> wide_cue_measurements{}; /**< 宽场候选量测
      （cross-cue 外发数据源，修订 7；controller 组装 SbirsCycleResult 时透传） */
  session::SbirsIssueList issues{};  /**< 正常执行周期按目标排除的 kInfo 诊断（规则 13b），经 controller 转写进 SbirsCycleResult */
};

/**
 * @brief WFOV/NFOV 双视场探测流水线，执行帧级几何门控、WFOV 发现、状态机决策与 NFOV 捕获/跟踪。
 * @note pipeline 跨周期累积状态（扫描相位、目标状态机、NFOV 锁定、随机源）；
 *       Capture/RestoreRuntimeState 是经完整校验的 internal checkpoint，用于确定性 continuation
 *       与状态恢复测试；当前 controller 周期路径不声明执行后回滚步骤。
 */
class SbirsPipeline {
 public:
  /**
   * @brief 构造 pipeline 并应用初始内部执行配置。
   * @param[in] config 内部执行配置
   */
  explicit SbirsPipeline(const config::SbirsInternalExecutionConfig& config);

  /**
   * @brief 应用新的内部执行配置（runtime patch 立即生效后调用）。
   * @param[in] config 新的内部执行配置
   */
  void ApplyConfig(const config::SbirsInternalExecutionConfig& config,
                   const runtime::SbirsRuntimeConfigImpact& impact);
  /**
   * @brief 标注本管线实例的卫星实体/融合源 ID（仅进验收日志行的 卫星ID=/相对卫星ID= 字段）。
   * @note 双星同文件写 opir_acceptance.log 时靠该 ID 区分行归属哪颗卫星；默认 0 表示
   *       调用方未标注，行内如实写 0。不影响任何计算路径。
   */
  void SetSatelliteEntityId(std::uint32_t satellite_entity_id) {
    satellite_entity_id_ = satellite_entity_id;
  }
  /** @brief 本管线实例的卫星实体/融合源 ID（验收标注用；默认 0）。 */
  std::uint32_t satellite_entity_id() const { return satellite_entity_id_; }
  /**
   * @brief 执行一个仿真周期的探测流水线。
   * @param[in] input 单周期输入
   * @return 本周期 pipeline 结果
   */
  SbirsPipelineResult RunCycle(const session::SbirsCycleInput& input);

  /**
   * @brief 运行时注入一条星间 cross-cue 引导消息（修订 5：不进每帧输入）。
   * @param[in] cue 引导消息（来源星宽场带误差测角+距离+来源星 ECEF 位置）
   * @note 消息入运行期队列，下一次 `RunCycle` 开始时消费（消费后清空）；非法消息
   *       （非有限角度/非正距离/未知目标键）消费时丢弃并计 kInfo issue。队列不属于
   *       快照：回放路径从不注入，恢复后逐位一致。
   */
  void SubmitExternalCue(const session::SbirsExternalCue& cue);

  /** @return 当前运行期状态快照，用于 validated checkpoint 与状态恢复测试。 */
  SbirsPipelineSnapshot CaptureRuntimeState() const;
  /**
   * @brief 从快照恢复运行期状态。
   * @param[in] snapshot 待恢复的状态快照
   * @return 恢复成功返回 true，否则返回 false
   */
  bool RestoreRuntimeState(const SbirsPipelineSnapshot& snapshot);

 private:
  config::SbirsInternalExecutionConfig config_{};
  float scan_phase_deg_{0.0f};
  // 扫描方位基准（2026-08-31）：kEciAbsolute 恒 0；kNadirRelative 为最近一次执行
  // 周期算出的星下点 ECI 方位（deg），ApplyConfig 相位重锚与 Execute 指向共用。
  float scan_azimuth_base_deg_{0.0f};
  // 往复腿方向（2026-08-31）：真实扫描机构到边反向（牛耕式）；true = 沿 scan_direction
  // 初始方向，false = 回程。rate=0 时恒为初始方向（凝视行为逐位不变）。
  bool scan_leg_forward_{true};
  // 有效扫描跨度（2026-08-31）：nadir 模式收敛 min(span, 2(asin(R/|r|)+wfov_az/2))
  // （超出地球盘可见范围的扫描无效）；绝对模式 = 配置跨度。每执行周期更新。
  float scan_wrap_span_deg_{0.0f};
  float last_logged_wrap_span_deg_{-1.0f};  /**< 收敛日志去重（值变化越过阈值才打） */
  int scan_row_index_{0}; /**< 当前 WFOV 俯仰栅格行索引，范围 [0, row_count) */
  std::uint32_t satellite_entity_id_{0U}; /**< 卫星实体/融合源 ID（验收行标注用，不影响计算） */
  bool install_matrices_acceptance_pending_{true}; /**< 安装矩阵验收行待写（构造/ApplyConfig 置位，首个执行周期写出） */
  session::SbirsEulerAnglesDeg misalignment_total_deg_{}; /**< 运行期安装失准角总量（构造/ApplyConfig 一次抽取） */
  std::uint64_t next_detection_id_{1U};
  std::map<std::uint64_t, SbirsTargetState> target_states_{};
  // WFOV 连续命中计数（3.2.1.3.2.1 宽窄切换前置条件）：目标连续通过 WFOV 门
  //（遮挡+距离+视场+SNR）的周期数；任一门失败或目标消失清零，进入跟踪时清零。
  // 阈值 policy.scheduler.wide_to_narrow_required_consecutive_hits（默认 1 = 单次命中
  // 即可调度，与既有行为逐位一致）。
  std::map<std::uint64_t, unsigned int> wfov_consecutive_hits_{};
  // 星间 cross-cue 运行期队列（2026-09-01）：周期之间注入、RunCycle 开始时消费并清空；
  // 不进快照（回放从不注入），非法消息消费时丢弃并计 issue。
  std::vector<session::SbirsExternalCue> pending_external_cues_{};
  SbirsNfovScheduler nfov_scheduler_;              // NFOV 锁定集合管理器（无配置上限，容量物理涌现）
  SbirsPointingCoordinator pointing_coordinator_;  // NFOV 单镜筒 ATP 执行状态（共享执行器）
  // 单镜筒轮转步进（2026-09-02）：每执行周期在非空锁定集合上 +1，服务目标 =
  // 有序锁定集合[step % size]，确定性轮转（replay 可复现）。
  std::uint64_t nfov_rotation_step_{0U};
  foundation::SbirsRandomSource wfov_measurement_random_source_;
  foundation::SbirsRandomSource estimated_measurement_random_source_;
  foundation::SbirsRandomSource sensor_like_output_random_source_;
  SbirsCuePredictor cue_predictor_{};
  SbirsTrackingCoordinator tracking_coordinator_{};
};

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_PIPELINE_H_
