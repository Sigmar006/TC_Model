/**
 * @file events.h
 * @brief 自定义实体-组件示例：组件间通信的事件类型集合（集成契约）。
 *
 * 事件是集成契约：本头文件零 1q 库依赖（仅 C++ 标准库），集成方模块不含
 * 库头即可订阅/发布。事件为纯数据结构（无逻辑、无虚函数）；库内枚举/结构
 * 由发布方展平为镜像枚举或原始数值字段，需要喂库 API 的消费方在本地把
 * 字段重建为库类型（防腐层归消费方）。
 *
 * 组件交互规则：host_->Find<T>() 仅允许取本平台机动组件（FlightComponent，
 * 传感器读平台位姿）；其余跨模块交互一律走本文件事件（Boost.Signals2 多播，
 * 见 signals.h）或共享场景状态黑板（探测池/RF 世界，见 core/scene_types.h；
 * 集成方对应消息推送/共享总线）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_CORE_EVENTS_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_CORE_EVENTS_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace component_attachment {

/**
 * @brief 威胁等级镜像枚举（值同 threat_assessment::ThreatLevel 与 esr
 *        EsrThreatLevel：低 < 中 < 高，供升级判定比较）。
 */
enum class EventThreatLevel : std::uint8_t {
  kLow = 0,    /**< 低威胁 */
  kMedium = 1, /**< 中威胁 */
  kHigh = 2    /**< 高威胁 */
};

/** @brief 辐射源工作模式镜像枚举（值同 esr EsrEmitterMode）。 */
enum class EventEmitterMode : std::uint8_t {
  kUnknown = 0,              /**< 未知模式 */
  kSearch = 1,               /**< 搜索模式 */
  kTracking = 2,             /**< 跟踪模式 */
  kGuidance = 3,             /**< 制导模式 */
  kContinuousIllumination = 4 /**< 连续波照射 */
};

/** @brief SAR 处理阶段镜像枚举（值同 sar SarProcessingStage）。 */
enum class EventSarStage : std::uint8_t {
  kNone = 0,      /**< 无 */
  kRawEcho = 1,   /**< 原始回波 */
  kL1RdaImage = 2, /**< L1 距离多普勒图像 */
  kL3BpImage = 3  /**< L3 后处理图像 */
};

/** @brief 平台状态事件：FlightComponent 每周期推进后发布。 */
struct PlatformStateEvent {
  std::uint64_t cycle{0U};              /**< 世界周期号 */
  double t_sec{0.0};                    /**< 周期绝对时间（s） */
  double position_ecef_x_m{0.0};        /**< 平台位置（ECEF X，m） */
  double position_ecef_y_m{0.0};        /**< 平台位置（ECEF Y，m） */
  double position_ecef_z_m{0.0};        /**< 平台位置（ECEF Z，m） */
  double altitude_m{0.0};               /**< 海拔（m） */
  double heading_deg{0.0};              /**< 航向（deg，北偏东） */
  double speed_mps{0.0};                /**< 速度（m/s） */
  std::size_t waypoint_index{0U};       /**< 下一航点索引 */
  std::size_t waypoint_count{0U};       /**< 航点总数 */
};

/** @brief 航点到达事件：FlightComponent 在航点完成判定时发布。 */
struct WaypointReachedEvent {
  double t_sec{0.0};              /**< 到达时刻（s） */
  std::size_t waypoint_index{0U}; /**< 完成航点索引 */
  double distance_m{0.0};         /**< 到达时刻距离（m） */
};

/** @brief AR 目标首确认事件（源：库内 ArTrackLifecycleRecorder 的 kFirstConfirmed）。 */
struct TargetConfirmedEvent {
  std::uint64_t cycle{0U};      /**< 世界周期号 */
  std::uint64_t target_id{0U};  /**< 外部目标 ID（关联键） */
  double latitude_deg{0.0};     /**< 目标位置纬度（deg） */
  double longitude_deg{0.0};    /**< 目标位置经度（deg） */
  double altitude_m{0.0};       /**< 目标位置海拔（m） */
};

/** @brief AR 目标失跟事件（源：库内 ArTrackLifecycleRecorder 的 kLost）。 */
struct TargetLostEvent {
  std::uint64_t cycle{0U};     /**< 世界周期号 */
  std::uint64_t target_id{0U}; /**< 外部目标 ID（关联键） */
  std::string reason{};        /**< 失跟原因（可读文本） */
};

/**
 * @brief AR 航迹逐周期状态事件：属性侧输入（威胁评估等消费方订阅）。
 *
 * 每成功周期逐在跟航迹发布（association_key 与融合键对齐——FusedTarget.key
 * 源自 AR association_key 适配）；速度/RCS/位置为 AR 调试视图展平值。
 */
struct ArTrackStateEvent {
  std::uint64_t cycle{0U};            /**< 世界周期号 */
  std::uint64_t association_key{0U};  /**< AR 库内航迹键（与融合键同键空间） */
  double speed_m_per_s{0.0};          /**< 航迹速度（m/s） */
  double rcs_m2{0.0};                 /**< 航迹 RCS（m²） */
  double position_x_m{0.0};           /**< 航迹位置（平台局部系 X，m；距离取模长） */
  double position_y_m{0.0};           /**< 航迹位置（平台局部系 Y，m） */
  double position_z_m{0.0};           /**< 航迹位置（平台局部系 Z，m） */
};

/**
 * @brief ESR 辐射源假设事件：每条新假设发布一次（人读/集成通知粒度）。
 */
struct EmitterHypothesisEvent {
  std::uint64_t cycle{0U};                           /**< 世界周期号 */
  std::uint64_t hypothesis_id{0U};                   /**< 假设标识 */
  double bearing_az_deg{0.0};                        /**< 方位线方位角（deg） */
  double confidence{0.0};                            /**< 假设置信度 [0,1] */
  EventEmitterMode mode{EventEmitterMode::kUnknown}; /**< 工作模式假设（镜像枚举） */
  EventThreatLevel threat_level{EventThreatLevel::kLow}; /**< 威胁等级（镜像枚举） */
};

/**
 * @brief ESR 辐射源假设数据（纯数据镜像，字段同库内 EmitterHypothesis）。
 *
 * ESR 每成功周期经 EsrScanUpdatedEvent 全量发布；需要库类型的消费方（如
 * ECM 适配器）在本地重建 EmitterHypothesis 后喂库 API。
 */
struct EsrHypothesisData {
  std::uint64_t hypothesis_id{0U};               /**< 假设记录唯一标识 */
  std::vector<std::string> candidate_classes{};  /**< 候选类别列表（按置信度降序） */
  EventEmitterMode mode{EventEmitterMode::kUnknown}; /**< 工作模式假设（镜像枚举） */
  EventThreatLevel threat_level{EventThreatLevel::kLow}; /**< 威胁等级（镜像枚举） */
  float bearing_az_deg{0.0f};                    /**< 方位线方位角（deg） */
  float bearing_el_deg{0.0f};                    /**< 方位线俯仰角（deg） */
  float bearing_std_deg{0.0f};                   /**< 方位测量标准差（deg） */
  double estimated_center_frequency_hz{0.0};     /**< 估计中心频率（Hz） */
  double estimated_bandwidth_hz{0.0};            /**< 估计占用带宽（Hz） */
  double estimated_pri_s{0.0};                   /**< 估计脉冲重复间隔（s） */
  double estimated_pulse_width_s{0.0};           /**< 估计脉宽（s） */
  double center_frequency_std_hz{0.0};           /**< 中心频率估计标准差（Hz） */
  double bandwidth_std_hz{0.0};                  /**< 带宽估计标准差（Hz） */
  double pri_std_s{0.0};                         /**< PRI 估计标准差（s） */
  double pulse_width_std_s{0.0};                 /**< 脉宽估计标准差（s） */
  float confidence{0.0f};                        /**< 假设置信度 [0,1] */
  std::uint32_t last_seen_cycle{0U};             /**< 最近命中周期号 */
  std::uint8_t waveform_class{0U};               /**< 波形类别整数值（同 EsrWaveformClass：0 脉冲/1 连续/2 扫频/3 噪声） */
};

/**
 * @brief ESR 假设集快照事件：每成功周期发布一次全量假设列表。
 *
 * sensor-driven 消费方（ECM）订阅本事件取当前假设集与批次号（fresh-frame
 * provenance），替代对 ESR 组件方法的直接调用。
 */
struct EsrScanUpdatedEvent {
  std::uint64_t cycle{0U};   /**< 完成周期号 */
  std::uint64_t batch_id{0U}; /**< 成功批次号（非 0；fresh-frame 溯源） */
  std::vector<EsrHypothesisData> hypotheses{}; /**< 全量假设（去真值化） */
};

/** @brief EOS 探测事件类型（生命周期语义，源为库内 EosDetectionLifecycleRecorder）。 */
enum class EosDetectionEventKind {
  kFirstDetected = 0, /**< 首次被发现 */
  kUpdated = 1,       /**< 持续探测并刷新 */
  kLost = 2,          /**< 此前已发现，本周期丢失 */
};

/** @brief EOS 探测事件：生命周期事件各发布一次（target_id 经归属映射）。 */
struct EosDetectionEvent {
  std::uint64_t cycle{0U};                /**< 世界周期号 */
  EosDetectionEventKind kind{EosDetectionEventKind::kUpdated}; /**< 事件类型 */
  std::uint64_t detection_id{0U};         /**< 本输出帧内探测记录标识（kLost 时为 0） */
  std::uint64_t target_id{0U};            /**< 归属目标 ID（无归属时为 0） */
  double snr_db{0.0};                     /**< 融合 SNR（dB；kLost 携带最后一次下检值） */
  double az_deg{0.0};                     /**< 探测方位（deg，平台局部系；kLost 时为 0） */
};

/**
 * @brief 融合态势更新事件：FusionComponent 每周期更新后逐目标发布。
 *
 * 运动学估计字段展平自 fusion::FusedKinematicEstimate（协方差 6×6 ECEF 行
 * 主序 [x,vx,y,vy,z,vz]）；has_kinematic_estimate 为 false 时估计字段无效。
 */
struct FusionUpdatedEvent {
  std::uint64_t cycle{0U};                  /**< 世界周期号 */
  std::uint64_t key{0U};                    /**< 融合目标键 */
  double confidence{0.0};                   /**< 融合置信度 */
  std::vector<std::pair<std::uint32_t, std::size_t>> channels{}; /**< (源通道, 样本数) 列表 */
  std::size_t new_targets{0U};              /**< 本周期新目标数 */
  std::size_t lost_targets{0U};             /**< 本周期消失目标数 */
  bool has_kinematic_estimate{false};       /**< 是否携带运动学估计（滤波未启用/未起始/坐标回写失败时为 false） */
  double latitude_deg{0.0};                 /**< 位置估计纬度（deg；估计无效时无效） */
  double longitude_deg{0.0};                /**< 位置估计经度（deg） */
  double altitude_m{0.0};                   /**< 位置估计海拔（m） */
  std::array<double, 3U> velocity_ecef_m_per_s{{0.0, 0.0, 0.0}}; /**< ECEF 速度（m/s） */
  std::array<double, 36U> covariance_ecef{}; /**< 6×6 ECEF 协方差（行主序 [x,vx,y,vy,z,vz]） */
};

/**
 * @brief 威胁评估更新事件：ThreatComponent 每周期评估后逐目标发布。
 *
 * 字段展平自 threat_assessment::ThreatResult（等级为镜像枚举、贡献分解为
 * 平铺数值）；威胁分 [0,1]，六项贡献之和 = 威胁分（权重和为 1 时）。
 */
struct ThreatUpdatedEvent {
  std::uint64_t cycle{0U};                        /**< 世界周期号 */
  std::uint64_t key{0U};                          /**< 目标库内键（与融合键对齐） */
  double threat_score{0.0};                       /**< 威胁分 [0,1] */
  EventThreatLevel level{EventThreatLevel::kLow}; /**< 威胁等级（镜像枚举） */
  double contribution_range{0.0};                 /**< 距离贡献 */
  double contribution_speed{0.0};                  /**< 速度贡献 */
  double contribution_acceleration{0.0};          /**< 加速度贡献 */
  double contribution_rcs{0.0};                   /**< RCS 贡献 */
  double contribution_target_probability{0.0};    /**< 类型概率贡献 */
  double contribution_fusion_confidence{0.0};     /**< 融合置信度贡献 */
};

/** @brief SBIRS 探测事件类型（生命周期语义，源为库内 SbirsDetectionLifecycleRecorder）。 */
enum class SbirsDetectionEventKind {
  kFirstDetected = 0, /**< 首次被发现 */
  kUpdated = 1,       /**< 持续探测并刷新 */
  kCoasting = 2,      /**< 暂无有效 NFOV 量测但仍保持锁定 */
  kLost = 3,          /**< 此前已发现，本周期丢失 */
};

/**
 * @brief SBIRS 丢失/未检测原因（示例层枚举，源为库内
 * SbirsDetectionLifecycleReason，组件映射）。kLost 事件携带细分原因，
 * 区分"目标真消失"与"扫描间隙/调度跳过/门失败"——避免消费方把扫描间隙
 * 误读为目标丢失。
 */
enum class SbirsDetectionLossReason {
  kNone = 0,               /**< 无具体原因（非丢失事件） */
  kOutOfFieldOfView,       /**< 目标在视场（FOV）外（扫描相位未覆盖） */
  kBelowSnrThreshold,      /**< IR SNR 低于门限 */
  kTargetMissingFromInput, /**< 目标从输入场景消失 */
  kAcquisitionFailed,      /**< 进入 NFOV 待捕获但捕获失败 */
  kSchedulerSkipped,       /**< WFOV 候选未被调度器选中（通道被占） */
  kNisGateLost,            /**< EKF NIS 连续超限导致释放 NFOV 锁定 */
  kPointingTimeout,        /**< ATP 光轴未在派生等待上限内稳定 */
  kTrackingGateLost,       /**< NFOV 跟踪几何/SNR 门连续失败 */
  kUnknown                 /**< 未知原因 */
};

/** @brief SBIRS 探测事件：生命周期事件各发布一次（target_id 经归属映射）。 */
struct SbirsDetectionEvent {
  std::uint64_t cycle{0U};                     /**< 世界周期号 */
  SbirsDetectionEventKind kind{SbirsDetectionEventKind::kUpdated}; /**< 事件类型 */
  SbirsDetectionLossReason reason{SbirsDetectionLossReason::kNone}; /**< 丢失/未检测细分原因（非丢失事件为 kNone） */
  std::uint64_t detection_id{0U};              /**< 本输出帧内检测记录标识（kLost 时为 0） */
  std::uint64_t target_id{0U};                 /**< 归属目标 ID（无归属时为 0） */
  float infrared_snr_linear{0.0f};             /**< 红外通道线性 IR SNR（kLost 携带最后一次下检值） */
  double az_deg{0.0};                          /**< 探测方位（deg，卫星局部系；kLost 时为 0） */
};

/** @brief SAR 产品事件类型（生命周期语义，源为库内 SarProductLifecycleRecorder）。 */
enum class SarProductEventKind {
  kImageProduced = 0,    /**< 首次产出图像产品 */
  kProductSustained = 1, /**< 产品持续存在 */
  kProductLost = 2,      /**< 已有产品本周期丢失 */
  kProcessingFailed = 3, /**< 处理失败 */
};

/** @brief SAR 产品事件：产品生命周期事件各发布一次（kNoProduct 不转发）。 */
struct SarProductEvent {
  std::uint64_t cycle{0U};                              /**< 世界周期号 */
  SarProductEventKind kind{SarProductEventKind::kProductSustained}; /**< 事件类型 */
  EventSarStage stage{EventSarStage::kNone};            /**< 本周期完成处理阶段（镜像枚举） */
  double estimated_snr_db{0.0};                         /**< 估计 SNR（dB） */
  std::string abort_reason{};                           /**< 中止原因（kProcessingFailed 时非空） */
};

/** @brief 决策指令类型（结构化载荷：路由器按类型分发到传感器运行期接口）。 */
enum class CommandKind {
  kEnableAntiFalseTarget = 0, /**< 开启抗假目标鉴别（纯日志演示，无库接口） */
  kEngageHighThreat = 1,      /**< 交战高威胁目标（触发 AR STT 锁定 + RIR 指定识别） */
  kDesignateTarget = 2,       /**< 指定目标（外部系统下令：AR STT + RIR 限时识别） */
  kClearDesignation = 3,      /**< 清除指定（回到扫描；两传感器 designated=0） */
};

/** @brief 决策指令事件：决策侧（DecisionListener）或外部指令脚本发布。 */
struct CommandIssuedEvent {
  std::uint64_t cycle{0U};      /**< 世界周期号 */
  CommandKind kind{CommandKind::kDesignateTarget}; /**< 指令类型 */
  std::uint64_t target_key{0U}; /**< 目标键（发布方视角的融合键；路由器翻译为外部目标 ID） */
  std::uint32_t duration_cycles{0U}; /**< 限时窗口（周期；0 = 无限期） */
  std::string command{};        /**< 指令描述（可读文本） */
};

/**
 * @brief 融合探测样本（纯数据镜像，字段同库内 fusion::DetectionRecord）。
 *
 * 机载/地基传感器把已适配探测展平后经 DetectionBatchSubmittedEvent 发给地面站；
 * 需要喂 FusionEngine 的消费方在本地重建 DetectionRecord。
 */
struct FusionDetectionSample {
  std::uint64_t key{0U};                 /**< 库内身份键（0 = 无身份） */
  std::uint32_t source_id{0U};           /**< 融合源通道 */
  bool has_position{false};              /**< 是否携带位置量测 */
  double latitude_deg{0.0};              /**< 位置纬度（deg） */
  double longitude_deg{0.0};             /**< 位置经度（deg） */
  double altitude_m{0.0};                /**< 位置海拔（m） */
  bool has_bearing{false};               /**< 是否携带方位量测 */
  double bearing_az_deg{0.0};            /**< 方位角（deg） */
  double bearing_el_deg{0.0};            /**< 俯仰角（deg） */
  bool has_sensor_origin{false};         /**< 是否携带量测原点 */
  double origin_latitude_deg{0.0};       /**< 量测原点纬度（deg） */
  double origin_longitude_deg{0.0};      /**< 量测原点经度（deg） */
  double origin_altitude_m{0.0};         /**< 量测原点海拔（m） */
  bool has_bearing_noise{false};         /**< 是否携带方位噪声 1-σ */
  double bearing_noise_sigma_rad{0.0};   /**< 方位轴 1-σ（rad） */
  double verdict{0.0};                   /**< 判决值 */
  double quality{0.0};                   /**< 探测质量 [0,1] */
};

/** @brief 探测记录批次提交：传感器 → 地面站融合（已适配样本，可空）。 */
struct DetectionBatchSubmittedEvent {
  std::uint64_t cycle{0U}; /**< 世界周期号 */
  std::uint32_t source_id{0U}; /**< 融合源通道 */
  std::vector<FusionDetectionSample> records{}; /**< 本批次探测样本 */
};

/**
 * @brief SBIRS 过门探测样本（纯数据镜像：检测角 + 归属目标键）。
 *
 * 卫星侧把本周期过门检测展平后发给地面站；地面站再建库类型做方位适配。
 * 方位/俯仰为 ECI 极坐标（rad），与 SBIRS 输出约定一致。
 */
struct SbirsBearingSample {
  std::uint64_t detection_id{0U};  /**< 本帧检测记录标识 */
  std::uint64_t target_id{0U};     /**< 归属目标 ID（无归属为 0，地面站跳过） */
  float azimuth_rad{0.0f};         /**< 方位角（ECI 极坐标，rad） */
  float elevation_rad{0.0f};       /**< 仰角（ECI 极坐标，rad） */
  float infrared_snr_linear{0.0f}; /**< 红外通道线性 SNR */
  int observation_stage{0};        /**< 观测阶段平铺（0=宽场搜索 1=窄场捕获 2=窄场跟踪；
                                        SbirsObservationStage 枚举值；双星定位仅收窄场） */
  bool has_focal_plane_offset{false}; /**< 是否携带 NFOV 焦平面脱靶量（验收判定标准 第26项 数据源） */
  float focal_plane_offset_x_m{0.0f}; /**< 焦平面脱靶量 x（m） */
  float focal_plane_offset_y_m{0.0f}; /**< 焦平面脱靶量 y（m） */
  double measurement_time_s{0.0};     /**< 测量时刻（s） */
};

/**
 * @brief 卫星 SBIRS 探测帧提交：任一颗星 → 地面站（星群 N 颗，每星一条）。
 *
 * 不携带库内周期结果；地面站按 source_id 区分卫星，本周期可收任意数量。
 */
struct SbirsFrameSubmittedEvent {
  std::uint64_t cycle{0U};           /**< 世界周期号 */
  std::uint32_t source_id{0U};       /**< 融合源通道（每星互异） */
  double satellite_ecef_x_m{0.0};    /**< 该星 ECEF X（m） */
  double satellite_ecef_y_m{0.0};    /**< 该星 ECEF Y（m） */
  double satellite_ecef_z_m{0.0};    /**< 该星 ECEF Z（m） */
  std::vector<SbirsBearingSample> detections{}; /**< 本周期过门探测（可空） */
};

/**
 * @brief 星间 cross-cue（交叉提示）递话条目：来源星宽场量测（裁定 2 口径）。
 *
 * 只带量测链量（ECI 视线角 + 带误差距离），不携带目标位置类合成量；
 * 受话星按来源星位置 + 距离 × 视线方向三角化后转自己视角。
 */
struct SbirsCrossCueItem {
  std::uint64_t target_id{0U};  /**< 被引导目标 ID */
  float azimuth_rad{0.0f};      /**< 来源星宽场量测方位（ECI 极坐标，带误差，rad） */
  float elevation_rad{0.0f};    /**< 来源星宽场量测俯仰（ECI 极坐标，带误差，rad） */
  double range_m{0.0};          /**< 来源星带误差距离（误差模型，m） */
};

/**
 * @brief 星间 cross-cue 递话：卫星 → 地面站 → 其他卫星（星→地→星，2026-09-01 裁定 1）。
 *
 * 来源星每周期把自星宽场检出（stage-0）打包外发；地面站原样转发，各受话星
 * 忽略自己发的，按"收到周期 < 本周期"节拍在下一周期步进前注入会话。
 */
struct SbirsCrossCueEvent {
  std::uint64_t cycle{0U};       /**< 来源星量测周期号 */
  std::uint32_t source_satellite_entity_id{0U}; /**< 来源星实体/融合源 ID */
  double satellite_ecef_x_m{0.0}; /**< 来源星 ECEF X（m；受话星旋 ECI 三角化用） */
  double satellite_ecef_y_m{0.0}; /**< 来源星 ECEF Y（m） */
  double satellite_ecef_z_m{0.0}; /**< 来源星 ECEF Z（m） */
  std::vector<SbirsCrossCueItem> cues{}; /**< 本周期递话条目（可空） */
};

/**
 * @brief 落点预报外发：地面站融合枢纽 → 订阅方（评审 2026-08-26 条12 真外发通道）。
 *
 * 载荷自包含（展平 LLA + 误差 1-σ），订阅方无需回查库内对象；每周期每有落点解
 * 的估计航迹一条。
 */
struct ImpactForecastEvent {
  std::uint64_t cycle{0U};       /**< 世界周期号 */
  std::uint64_t key{0U};         /**< 目标键 */
  double impact_latitude_deg{0.0};  /**< 预测落点纬度（deg） */
  double impact_longitude_deg{0.0}; /**< 预测落点经度（deg） */
  double impact_altitude_m{0.0};    /**< 预测落点高度（m） */
  double impact_position_sigma_m{0.0}; /**< 落点 1-σ（m） */
  bool has_burnout_sigma{false};  /**< 是否携带关机点 1-σ */
  double burnout_position_sigma_m{0.0}; /**< 关机点 1-σ（m） */
  double confidence{0.68};        /**< 置信度 */
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_CORE_EVENTS_H_
