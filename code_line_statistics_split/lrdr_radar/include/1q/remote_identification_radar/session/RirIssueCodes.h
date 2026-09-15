/**
 * @file RirIssueCodes.h
 * @brief 远程识别雷达 issue code 注册表：本模块全部 code 常量的单一事实来源
 *       （session_contract.md 规则 14c）。
 * @note 仅登记真实产生的 issue code；不产生为 issue code 的兜底串不在此登记。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_ISSUE_CODES_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_ISSUE_CODES_H_

namespace remote_identification_radar {
namespace session {
namespace codes {

/** @brief 周期步长非有限值。 */
constexpr char kNonFiniteCycleDeltaTime[] = "rir.validation.non_finite_cycle_delta_time";

/** @brief 周期步长非法（<= 0）。 */
constexpr char kInvalidCycleDeltaTime[] = "rir.validation.invalid_cycle_delta_time";

/** @brief 周期序号非法（为 0）。 */
constexpr char kInvalidCycleIndex[] = "rir.validation.invalid_cycle_index";

/** @brief 目标含非有限数值字段（位置/RCS/真值样本）。 */
constexpr char kNonFiniteTargetField[] = "rir.validation.non_finite_target_field";

/** @brief 目标无非零笛卡尔位置。 */
constexpr char kMissingRangeAndCartesianPosition[] =
    "rir.validation.missing_range_and_cartesian_position";

/** @brief 场景中外部目标 ID 重复。 */
constexpr char kDuplicateExternalTargetId[] = "rir.validation.duplicate_external_target_id";

/** @brief 目标速度/起伏模型含非有限或非法字段。 */
constexpr char kInvalidTargetMotionField[] = "rir.validation.invalid_target_motion_field";

/** @brief 环境快照字段非法（天气衰减非有限/负值）。 */
constexpr char kInvalidEnvironmentSnapshot[] = "rir.validation.invalid_environment_snapshot";

/** @brief RF 场景帧非法或与周期窗口不一致。 */
constexpr char kInvalidRfSceneFrame[] = "rir.validation.invalid_rf_scene_frame";

/** @brief 平台 ECEF 位置非法（has=true 时分量非有限或模长为 0——地心非法）。 */
constexpr char kInvalidPlatformPosition[] = "rir.validation.invalid_platform_position";

/** @brief 平台位置存在性标志与数据不一致（has=false 但分量非默认值）。 */
constexpr char kInconsistentPlatformPosition[] =
    "rir.validation.inconsistent_platform_position";

/** @brief 识别特征权重非法（须有限、在 [0, 1] 且总和为 1）。 */
constexpr char kRecognitionWeightsInvalid[] = "rir.validation.recognition_weights_invalid";

/** @brief 识别数据库路径缺失（启用识别时须非空）。 */
constexpr char kRecognitionDatabasePathMissing[] =
    "rir.validation.recognition_database_path_missing";

/** @brief 识别门限非法（接受分数/最小裕度须在 [0, 1]）。 */
constexpr char kRecognitionThresholdInvalid[] = "rir.validation.recognition_threshold_invalid";

/** @brief 识别累积计数非法（须至少为 1）。 */
constexpr char kRecognitionAccumulationInvalid[] =
    "rir.validation.recognition_accumulation_invalid";

/** @brief 识别时间范围非法（保持时间须非负；最大距离/驻留/累积窗口须有限且为正）。 */
constexpr char kRecognitionTimeRangeInvalid[] = "rir.validation.recognition_time_range_invalid";

/** @brief 扫描策略非法（步长系数须为正）。 */
constexpr char kScanStrategyInvalid[] = "rir.validation.scan_strategy_invalid";

/** @brief 扫描波位单轴采样数超内核上限（4096），波位表会被静默截断；
 *         需加大步长（step_scale/波束宽）或缩小扫描扇区。 */
constexpr char kScanWaveAxisSamplesTruncated[] =
    "rir.validation.scan_wave_axis_samples_truncated";

/** @brief 可扫描体积非法（orientation 须有限有序且在合法域）。 */
constexpr char kSteerableVolumeInvalid[] = "rir.validation.steerable_volume_invalid";

/** @brief 转台朝向非法（mission.scan_center_deg 须有限且在合法域）。 */
constexpr char kScanCenterInvalid[] = "rir.validation.scan_center_invalid";

/** @brief 信号处理增益偏置非法（四偏置须有限且在 [0, 40] dB；阶段 2-M M3）。 */
constexpr char kSignalProcessingGainsInvalid[] = "rir.validation.signal_processing_gains_invalid";

/** @brief 检测策略非法（Pfa/门限/脉冲数/种子）。 */
constexpr char kDetectionPolicyInvalid[] = "rir.validation.detection_policy_invalid";

/** @brief 关联策略非法（波门 sigma 非正）。 */
constexpr char kAssociationPolicyInvalid[] = "rir.validation.association_policy_invalid";

/** @brief 跟踪策略非法（KF 噪声参数非正）。 */
constexpr char kTrackingPolicyInvalid[] = "rir.validation.tracking_policy_invalid";

/** @brief 生命周期策略非法（confirm/lost 阈值）。 */
constexpr char kLifecyclePolicyInvalid[] = "rir.validation.lifecycle_policy_invalid";

/** @brief 发射机载频非法（须有限且为正）。 */
constexpr char kTransmitterFrequencyInvalid[] = "rir.validation.transmitter_frequency_invalid";

/** @brief 频率计划非法（须含有限正值且包含初始载频）。 */
constexpr char kFrequencyPlanInvalid[] = "rir.validation.frequency_plan_invalid";

/** @brief 发射机工作包络非法（功率/占空比/脉冲能量越界）。 */
constexpr char kTransmitterOperatingEnvelopeInvalid[] =
    "rir.validation.transmitter_operating_envelope_invalid";

/** @brief 接收机 RF 硬件边界非法。 */
constexpr char kReceiverRfHardwareInvalid[] = "rir.validation.receiver_rf_hardware_invalid";

/** @brief 天线几何非法（波束宽度或孔径无效）。 */
constexpr char kAntennaAzGeometryInvalid[] = "rir.validation.antenna_az_geometry_invalid";

/** @brief 天线俯仰几何非法。 */
constexpr char kAntennaElGeometryInvalid[] = "rir.validation.antenna_el_geometry_invalid";

/** @brief 天线峰值增益与名义波束宽度物理不相容（G_dBi ≈ 10·log10(26000/(az°·el°))，
 *         容差 3 dB；2026-08-29 还债 B5：两数互不校验会让回波预算虚高）。 */
constexpr char kAntennaGainBeamwidthInconsistent[] =
    "rir.validation.antenna_gain_beamwidth_inconsistent";

/** @brief 标称波束宽度与孔径 λ/L 推导值交叉矛盾（相对偏差 > 50%；
 *         波束宽与孔径是一枚硬币的两面，矛盾配置使波位步长与检测门口径分叉）。 */
constexpr char kAntennaBeamwidthApertureInconsistent[] =
    "rir.validation.antenna_beamwidth_aperture_inconsistent";

/** @brief RCS 物理参数非法。 */
constexpr char kRcsPhysicsInvalid[] = "rir.validation.rcs_physics_invalid";

/** @brief 设备关机（非执行周期中止）。 */
constexpr char kSensorPoweredOff[] = "rir.sensor_powered_off";

// ===== 周期执行按目标门控排除诊断（"rir.target_<snake_case>"，规则 13b）=====

/** @brief 目标视线被地球圆球遮挡（几何门最早：体积/SNR 之前；具体门 cause=kNone）。 */
constexpr char kTargetEarthOcculted[] = "rir.target_earth_occulted";

/** @brief 目标视线角出可扫描体积，不入搜索候选集（角域裁剪在地球遮挡之后；
 *         az 相对 scan_center、el 绝对；具体门 cause=kNone）。 */
constexpr char kTargetOutsideSearchVolume[] = "rir.target_outside_search_volume";

/** @brief 目标在搜索扇区内但不被本周期驻留主瓣覆盖（"探测⟺波束照到"硬门，
 *         门限=有效波束宽度半功率宽；方向图恒开的配套语义；cause=kNone）。 */
constexpr char kTargetOutsideBeamCoverage[] = "rir.target_outside_beam_coverage";

/** @brief 接收前端饱和周期致盲（周期级门：全部入射链聚合功率超接收线性上限，
 *         本周期全部目标不做检测判决；排在检测准入门之前，cause=kNone）。 */
constexpr char kTargetReceiverFrontEndSaturated[] =
    "rir.target_receiver_front_end_saturated";

/** @brief 检测准入门未过（聚合门：SNR/检测器判决；携带门内归因主因）。 */
constexpr char kTargetDetectionGate[] = "rir.target_detection_gate";

/** @brief 目标斜距超识别最大作用距离（识别链距离门，检测/跟踪不受影响）。 */
constexpr char kTargetBeyondRecognitionRange[] = "rir.target_beyond_recognition_range";

/** @brief 本周期非识别工作模式，不建识别观测（STBY 全局模式门）。 */
constexpr char kTargetModeNotIdentify[] = "rir.target_mode_not_identify";

/** @brief 特征库缺失或加载失败，特征链空（识别积累保持）。 */
constexpr char kTargetNoFeatureDatabase[] = "rir.target_no_feature_database";

}  // namespace codes
}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_ISSUE_CODES_H_
