/**
 * @file ArIssueCodes.h
 * @brief AR issue code 注册表：本模块全部 code 常量的单一事实来源（规则 14c）。
 * @note 仅用于日志映射、不产生为 issue code 的兜底串（如 ar.validation_rejected）不在此登记。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_ISSUE_CODES_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_ISSUE_CODES_H_

namespace airborne_radar {
namespace session {
namespace codes {

// ===== 输入校验问题（"ar.validation.<snake_case>"）=====

/** @brief 目标含非有限数值字段（位置/速度/RCS/斜距）。 */
constexpr char kNonFiniteTargetField[] = "ar.validation.non_finite_target_field";

/** @brief 目标斜距 <= 0 且无笛卡尔位置（二者须至少一为正）。 */
constexpr char kMissingRangeAndCartesianPosition[] =
    "ar.validation.missing_range_and_cartesian_position";

/** @brief 目标外部 ID 未知（为 0，kInfo 级）。 */
constexpr char kUnknownExternalTargetId[] = "ar.validation.unknown_external_target_id";

/** @brief 目标 RCS 为负（kWarning 级）。 */
constexpr char kNegativeRcs[] = "ar.validation.negative_rcs";

/** @brief 周期步长非有限值。 */
constexpr char kNonFiniteCycleDeltaTime[] = "ar.validation.non_finite_cycle_delta_time";

/** @brief 周期步长非法（<= 0）。 */
constexpr char kInvalidCycleDeltaTime[] = "ar.validation.invalid_cycle_delta_time";

/** @brief 周期序号非法（为 0）。 */
constexpr char kInvalidCycleIndex[] = "ar.validation.invalid_cycle_index";

/** @brief 周期起始时间非法（非有限或 < 0）。 */
constexpr char kInvalidCycleStartTime[] = "ar.validation.invalid_cycle_start_time";

/** @brief 平台输入非法（标识为 0 或世界系运动学非有限）。 */
constexpr char kInvalidPlatformInput[] = "ar.validation.invalid_platform_input";

/** @brief 目标输入非法（世界系运动学无法转换为雷达本地系）。 */
constexpr char kInvalidTargetInput[] = "ar.validation.invalid_target_input";

/** @brief 场景中外部目标 ID 重复。 */
constexpr char kDuplicateExternalTargetId[] = "ar.validation.duplicate_external_target_id";

/** @brief 非空干扰帧与 AR 周期窗口不匹配。 */
constexpr char kInterferenceFrameMismatch[] = "ar.validation.interference_frame_mismatch";

/** @brief 干扰输入非法（帧含非法 RF 事实）。 */
constexpr char kInvalidInterferenceInput[] = "ar.validation.invalid_interference_input";

/** @brief 发射机频率非法（须有限且为正）。 */
constexpr char kTransmitterFrequencyInvalid[] = "ar.validation.transmitter_frequency_invalid";

/** @brief 频率计划非法（须含有限正值且包含初始载频）。 */
constexpr char kFrequencyPlanInvalid[] = "ar.validation.frequency_plan_invalid";

/** @brief 发射机工作包络非法（功率/占空比/脉冲能量超出硬件限制）。 */
constexpr char kTransmitterOperatingEnvelopeInvalid[] =
    "ar.validation.transmitter_operating_envelope_invalid";


/** @brief 接收机 RF 硬件非法（隔离度/远场距离/线性输入限/共址路径无效）。 */
constexpr char kReceiverRfHardwareInvalid[] = "ar.validation.receiver_rf_hardware_invalid";

/** @brief 信号处理增益偏置非法（四偏置须有限且在 [0, 40] dB）。 */
constexpr char kSignalProcessingGainsInvalid[] = "ar.validation.signal_processing_gains_invalid";

/** @brief 指令方位波束宽度非正（启用指令波束时须有限且为正）。 */
constexpr char kCommandedBeamwidthAzNotPositive[] =
    "ar.validation.commanded_beamwidth_az_not_positive";

/** @brief 指令俯仰波束宽度非正（启用指令波束时须有限且为正）。 */
constexpr char kCommandedBeamwidthElNotPositive[] =
    "ar.validation.commanded_beamwidth_el_not_positive";

/** @brief 天线方位几何非法（标称波束宽度非正且无有效物理孔径）。 */
constexpr char kAntennaAzGeometryInvalid[] = "ar.validation.antenna_az_geometry_invalid";

/** @brief 天线俯仰几何非法（标称波束宽度非正且无有效物理孔径）。 */
constexpr char kAntennaElGeometryInvalid[] = "ar.validation.antenna_el_geometry_invalid";

/** @brief 机械扫描方位界限颠倒（min > max）。 */
constexpr char kMechanicalScanLimitsSwappedAz[] =
    "ar.validation.mechanical_scan_limits_swapped_az";

/** @brief 机械扫描俯仰界限颠倒（min > max）。 */
constexpr char kMechanicalScanLimitsSwappedEl[] =
    "ar.validation.mechanical_scan_limits_swapped_el";

/** @brief 电扫描方位界限颠倒（min > max）。 */
constexpr char kElectronicScanLimitsSwappedAz[] =
    "ar.validation.electronic_scan_limits_swapped_az";

/** @brief 电扫描俯仰界限颠倒（min > max）。 */
constexpr char kElectronicScanLimitsSwappedEl[] =
    "ar.validation.electronic_scan_limits_swapped_el";

// ===== 执行诊断（规则 13b/14c）=====

/** @brief 生命周期不可用（自动生命周期管理器缺失，非执行周期中止）。 */
constexpr char kLifecycleUnavailable[] = "ar.lifecycle_unavailable";

/** @brief 设备关机（非执行周期中止）。 */
constexpr char kSensorPoweredOff[] = "ar.sensor_powered_off";

/** @brief 环境周期无效（未以正 dt_sec 初始化，非执行周期中止）。 */
constexpr char kInvalidEnvironmentCycle[] = "ar.invalid_environment_cycle";

/** @brief 运行期准备失败（非执行周期中止）。 */
constexpr char kRuntimePreparationFailed[] = "ar.runtime_preparation_failed";

/** @brief 目标信噪比低于门限（按目标排除的 kInfo 诊断，规则 13b）。 */
constexpr char kTargetSnrBelowThreshold[] = "ar.target_snr_below_threshold";

}  // namespace codes
}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_ISSUE_CODES_H_
