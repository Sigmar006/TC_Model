/**
 * @file SbirsOutputTypes.h
 * @brief 定义 SBIRS-inspired 原生观测输出与归属类型。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_TYPES_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/foundation/validation_types.h"

namespace sbirs_sensor {
namespace output {

/**
 * @brief 表示原生观测所处的 WFOV 搜索、NFOV 首次捕获或 NFOV 持续跟踪阶段。
 * @note NFOV 持续跟踪同时承载估计跟踪和真值辅助跟踪，不区分滤波后端。
 */
enum class ONEQ_API SbirsObservationStage {
  kWideFieldSearch = 0,    /**< WFOV 宽视场搜索 */
  kNarrowFieldAcquisition, /**< NFOV 首次捕获 */
  kNarrowFieldTrack        /**< NFOV 持续跟踪（估计或真值辅助） */
};

/**
 * @brief 单条 SBIRS-inspired 原生观测检测记录，属于 1q 仿真传感器主输出层。
 * @note 该结构不含目标真值或仿真归属；归属信息进入 `SbirsDetectionAttributionRecord`。
 */
struct ONEQ_API SbirsDetectionRecord {
  std::uint64_t detection_id{0U};  /**< 本输出帧内的检测记录标识 */
  double measurement_time_s{0.0};  /**< 量测产生的绝对仿真时刻（秒） */
  float azimuth_rad{0.0f};         /**< 方位角（ECI 极坐标 az = atan2(y,x)，相对 ECI x 轴），单位 rad，范围 [0, 2π)；非卫星局部地平系 */
  float elevation_rad{0.0f};       /**< 仰角（ECI 极坐标 el = asin(z/r)，相对赤道面），单位 rad，范围 [-π/2, π/2]；非卫星局部地平系 */
  float infrared_snr_linear{0.0f}; /**< 红外通道线性 IR SNR */
  SbirsObservationStage observation_stage{SbirsObservationStage::kWideFieldSearch}; /**< 观测阶段 */
  bool detected{false}; /**< 是否通过探测门限判决 */
};

/** @brief 检测记录列表。 */
using SbirsDetectionRecordList = std::vector<SbirsDetectionRecord>;

}  // namespace output

namespace attribution {

/**
 * @brief NFOV 交接与跟踪失败原因，仅供归属/调试层消费，不进 `SbirsOutputFrame` raw output。
 * @note 标识首次捕获、调度、估计门、pointing timeout 或闭环跟踪门导致的失败。
 */
enum class ONEQ_API SbirsCaptureFailureReason {
  kNone = 0,              /**< 无失败（成功捕获或仅 WFOV 观测） */
  kNfovAcquisitionFailed, /**< 进入 NFOV 待捕获但捕获失败（视场外或 SNR 不足） */
  kSchedulerSkipped,      /**< WFOV 候选未被调度器选中（资源被占用或排序靠后） */
  kEstimationNisGateLost, /**< EKF NIS 连续超限导致释放 NFOV 锁定 */
  kNfovPointingTimeout,   /**< ATP 光轴在派生等待上限内未稳定 */
  kNfovTrackingGateLost   /**< NFOV 跟踪几何/SNR 门连续失败导致丢锁 */
};

/** @brief 归属记录所对应的正式跟踪来源；WFOV 与失败捕获不适用。 */
enum class ONEQ_API SbirsTrackingSource {
  kNotApplicable = 0,          /**< 尚未建立 NFOV 跟踪 */
  kEstimated,                  /**< Estimated 跟踪 */
  kStrictTruthAssisted,        /**< StrictTruthAssisted 跟踪 */
  kSensorLikeTruthAssisted     /**< SensorLikeTruthAssisted 跟踪 */
};

/**
 * @brief 检测记录到输入目标的仿真归属记录，仅供结构化结果/调试层消费。
 * @note 归属信息不得混入 `SbirsOutputFrame` raw output；`estimated_range_m` 为内部
 *       cue/诊断层估计距离，不代表真实被动红外测距能力。
 */
struct ONEQ_API SbirsDetectionAttributionRecord {
  std::uint64_t detection_id{0U}; /**< 对应的检测记录标识 */
  std::uint64_t target_id{0U};    /**< 输入场景目标 ID */
  std::string target_name{};      /**< 输入场景目标名称 */
  float estimated_range_m{0.0f};  /**< 估计距离，单位 m（仅归属/诊断层） */
  float max_detection_range_m{0.0f}; /**< 当前时刻最大探测距离 d_max(t)，单位 m（仅归属/诊断层；
                                          由 WFOV 检测门限反解，依赖该目标辐射强度与本周期
                                          环境透过率/噪声快照，逐周期变化；NFOV 门限版 =
                                          该值 × sqrt(wide_min_snr/narrow_min_snr)） */
  SbirsTrackingSource tracking_source{
      SbirsTrackingSource::kNotApplicable}; /**< 本记录的正式跟踪来源 */
  SbirsCaptureFailureReason capture_failure_reason{
      SbirsCaptureFailureReason::kNone};    /**< NFOV 交接/跟踪失败原因（仅归属/诊断层） */
  bool has_estimation_nis{false};           /**< 是否包含 EKF 估计跟踪 NIS 诊断 */
  float estimation_nis{0.0f};               /**< EKF 归一化新息平方，仅归属/诊断层 */
  bool estimation_nis_gate_exceeded{false}; /**< EKF NIS 是否超过 2 维 95% 门限 */
  int nfov_channel_id{-1}; /**< NFOV 通道编号；-1 表示 WFOV/未占用 NFOV 资源（仅归属/诊断层） */
  int nfov_frame_count{0};          /**< 本周期 NFOV 多帧采样帧数（高刷新率建模；0 = 单帧/非 NFOV，仅归属/诊断层） */
  float nfov_frame_sigma_deg{0.0f}; /**< NFOV 单帧角噪声 1-σ（deg；误差模型 RSS 合成值，仅归属/诊断层） */
  float nfov_fused_sigma_deg{0.0f}; /**< NFOV 多帧融合后角噪声 1-σ（deg = 单帧/√帧数，仅归属/诊断层） */
  bool has_nfov_tracking_diagnostics{false}; /**< 是否包含闭环 NFOV 跟踪门诊断 */
  float nfov_pointing_error_deg{0.0f};       /**< 有效光轴中心与目标真值 LOS 角距 */
  bool nfov_geometry_gate_passed{false};     /**< 目标是否位于实际 NFOV 矩形窗口内 */
  bool nfov_snr_gate_passed{false};          /**< 目标是否通过既有 NFOV SNR 门 */
  unsigned int nfov_tracking_gate_failure_count{0U}; /**< 连续 NFOV 跟踪门失败计数 */
  bool nfov_tracking_coasting{false}; /**< 本周期无有效 NFOV 量测但尚未正式丢锁 */
  bool has_focal_plane_offset{false}; /**< 是否包含焦平面脱靶量（NFOV 跟踪段；验收判定标准 第26项 脱靶量数据源） */
  float focal_plane_offset_x_m{0.0f}; /**< 焦平面脱靶量 x（m，仅归属/诊断层） */
  float focal_plane_offset_y_m{0.0f}; /**< 焦平面脱靶量 y（m，仅归属/诊断层） */
  int cue_source_satellite_entity_id{-1}; /**< 引导来源星 ID（cross-cue，2026-09-01；-1=自星宽场
                                               引导；锁定后随调度器通道持久、释放清除；只进
                                               归属/调试层，不进验收日志行，不参与调度排序） */
  float measured_range_m{0.0f}; /**< 误差模型距离（带误差；修订 6：宽场段=candidate.measured_range_m，
                                     消息路径 cross-cue 递话数据源；与 estimated_range_m（宽场=真值/
                                     窄场=融合估计）语义并存，仅归属/诊断层） */
};

/** @brief 归属记录列表。 */
using SbirsDetectionAttributionRecordList = std::vector<SbirsDetectionAttributionRecord>;

}  // namespace attribution

namespace session {

/**
 * @brief SBIRS 问题条目严重等级。
 */
enum class ONEQ_API SbirsIssueSeverity : std::uint8_t {
  kInfo = 0,    /**< 信息 */
  kWarning = 1, /**< 警告 */
  kError = 2    /**< 错误 */
};

/**
 * @brief SBIRS 问题条目来源阶段（统一问题列表模型，session_contract.md 规则 14）。
 * @note 结构化来源判别字段；状态判断仍以 `status`/`abort_reason` 为准，phase 不改变状态语义。
 */
enum class ONEQ_API SbirsIssuePhase : std::uint8_t {
  kInputValidation = 0, /**< 输入校验阶段（调用方输入问题） */
  kExecution = 1,       /**< 管线执行阶段（含关机等运行态条件） */
  kOutputContract = 2   /**< 输出违反内部契约 */
};

/**
 * @brief SBIRS 问题条目门内归因（规则 13b 门内归因条款，session_contract.md）。
 * @note 仅排除诊断使用：视场门排除标识越界轴；SNR 门排除标识物理链路主因（距离/
 *       大气透过率/目标签名/噪声底）；遮挡与距离带为具体门，cause 保持 kNone 仅量值。
 *       不替代 code（机器键仍只认 code），不用于状态判断。
 */
enum class ONEQ_API SbirsIssueCause : std::uint8_t {
  kNone = 0,            /**< 无归因（具体门排除或非排除诊断） */
  kAzOutside,           /**< 仅方位越出视场 */
  kElOutside,           /**< 仅俯仰越出视场 */
  kBothAxesOutside,     /**< 方位与俯仰均越出视场 */
  kDistanceLimited,     /**< 目标距离主导 SNR 门失败 */
  kAttenuationLimited,  /**< 大气透过率/路径衰减主导 SNR 门失败 */
  kSignatureLimited,    /**< 目标签名（辐射强度）主导 SNR 门失败 */
  kNoiseLimited,        /**< 噪声底主导 SNR 门失败（当前不产生：噪声为硬件常数，保留供未来硬件噪声建模） */
  kUnknown              /**< 无法判定主因 */
};

/**
 * @brief SbirsIssue 描述单周期问题条目（统一问题列表模型，session_contract.md 规则 14）。
 * @note 承载输入校验问题（phase=kInputValidation）与执行诊断（phase=kExecution）；
 *       code 带模块前缀（如 "sbirs.target_out_of_wfov"、
 *       "sbirs.validation.invalid_satellite_position"），机器消费只认 code；
 *       不用于调用方状态判断。
 */
struct ONEQ_API SbirsIssue {
  SbirsIssueSeverity severity{SbirsIssueSeverity::kInfo};
  SbirsIssuePhase phase{SbirsIssuePhase::kExecution};
  std::string code{};
  std::string message{};
  oneq::foundation::ValidationLocation location{}; /**< 可选定位；kind==kGlobal 表示无定位 */
  std::string field{}; /**< 可选定位；为空表示无关联字段（跨字段或域级问题） */
  SbirsIssueCause cause{SbirsIssueCause::kNone}; /**< 可选归因；仅排除诊断使用（规则 13b） */
};

/** @brief SBIRS 问题条目列表。 */
using SbirsIssueList = std::vector<SbirsIssue>;

/** @brief 单周期执行的中止原因。 */
enum class ONEQ_API SbirsPipelineAbortReason {
  kNone = 0,             /**< 正常执行，无中止 */
  kValidationRejected,   /**< 输入校验拒绝 */
  kSensorPoweredOff      /**< 设备关机或待机，核心 pipeline 未执行 */
};

/**
 * @brief SbirsCycleStatus 描述单周期高层执行状态。
 * @note 与 ArCycleStatus / EsrCycleExecutionStatus / EosCycleStatus 对齐的强类型枚举。
 */
enum class ONEQ_API SbirsCycleStatus : std::uint8_t {
  kCompleted = 0,           /**< 周期正常完成 */
  kPoweredOff,              /**< 设备关机或待机，核心 pipeline 未执行 */
  kRejectedInvalidInput,    /**< 输入校验失败 */
  kRejectedExecution        /**< 执行失败 */
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_TYPES_H_
