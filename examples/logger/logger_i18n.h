/**
 * @file logger_i18n.h
 * @brief 集成端日志中文适配：issue code → 中文名 + 问题列表格式化（查表）。
 *
 * 库内 issue message 为英文人读文本且格式不承诺稳定（规则 13b：机器消费只认
 * code，不得解析 message）——因此示例层不做 message 翻译/解析，只把**稳定
 * code** 映射为中文名；量值一律从 DebugView 结构化字段取（组件组摘要行时
 * 填充）。中文名表为五模块 IssueCodes.h 全量注册表（与 @brief 注释同步，
 * 由脚本提取生成，见 docs/common/issue_codes.md）；未知 code 返回英文
 * message 原文，库内新增 code 自动回退英文。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_I18N_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_I18N_H_

#include <string>
#include <vector>

#include "1q/airborne_radar/session/ArIssueCodes.h"
#include "1q/electro_optical_sensor/session/EosIssueCodes.h"
#include "1q/electronic_surveillance_radar/session/EsrIssueCodes.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "1q/sar/session/SarIssueCodes.h"
#include "1q/sbirs_sensor/session/SbirsIssueCodes.h"

namespace component_attachment {
namespace app {

/// issue code → 中文名（人读日志）。未知 code 返回 nullptr（调用方回退英文原文）。
inline const char* IssueCodeChineseName(const std::string& code) {
  struct Entry {
    const char* code;
    const char* name;
  };
  // 全量注册表中文名（与各模块 IssueCodes.h @brief 同步）；库内新增 code 自动回退英文 message。
  static const Entry kNames[] = {
      // electro_optical_sensor (22)
      {electro_optical_sensor::session::codes::kPipelineContractViolation, "管线输出违反内部契约（kOutputContract 相位）。"},
      {electro_optical_sensor::session::codes::kRuntimeStateRestoreRejected, "运行期状态恢复被拒绝。"},
      {electro_optical_sensor::session::codes::kSensorPoweredOff, "设备关机（非执行周期中止）。"},
      {electro_optical_sensor::session::codes::kTargetOutOfFov, "目标视场外（正常周期按目标排除的 kInfo 诊断，规则 13b）。"},
      {electro_optical_sensor::session::codes::kAtmosphericPhysicsInvalid, "启用物理模型时大气物理参数无效（压力/温度/湿度越界）。"},
      {electro_optical_sensor::session::codes::kCycleDeltaTimeExceedsFramePeriod, "周期步长超过帧周期合理范围（> 10 倍帧周期，即 10 / frame_rate_hz）。"},
      {electro_optical_sensor::session::codes::kEnvironmentPresetInvalid, "环境预设无效。"},
      {electro_optical_sensor::session::codes::kFrameRateNotPositive, "帧率非正。"},
      {electro_optical_sensor::session::codes::kHorizontalFovNotPositive, "水平视场角非正。"},
      {electro_optical_sensor::session::codes::kInconsistentTargetEnergyBalance, "目标能量平衡校验失败（辐射效率 + 反射率 > 1，kInfo 级）。"},
      {electro_optical_sensor::session::codes::kInvalidCycleDeltaTime, "周期步长非法（<= 0）。"},
      {electro_optical_sensor::session::codes::kInvalidTargetEmissivity, "目标辐射效率非法（不在 [0, 1]）。"},
      {electro_optical_sensor::session::codes::kInvalidTargetProjectedArea, "目标投影面积非法（<= 0）。"},
      {electro_optical_sensor::session::codes::kInvalidTargetRange, "目标斜距非法（<= 0）。"},
      {electro_optical_sensor::session::codes::kInvalidTargetReflectance, "目标反射率非法（不在 [0, 1]）。"},
      {electro_optical_sensor::session::codes::kInvalidTargetTemperature, "目标温度非法（<= 0）。"},
      {electro_optical_sensor::session::codes::kNonFiniteCycleDeltaTime, "周期步长非有限值。"},
      {electro_optical_sensor::session::codes::kNonFinitePlatformNumericField, "平台位姿含非有限数值（位置/速度/姿态角）。"},
      {electro_optical_sensor::session::codes::kNonFiniteTargetNumericField, "目标含非有限数值字段。"},
      {electro_optical_sensor::session::codes::kScanRangeAzSwapped, "扫描方位起止颠倒（起始 >= 结束）。"},
      {electro_optical_sensor::session::codes::kScanRateNotPositive, "扫描速率非正。"},
      {electro_optical_sensor::session::codes::kVerticalFovNotPositive, "垂直视场角非正。"},
      // sbirs_sensor (30)
      {sbirs_sensor::session::codes::kSensorPoweredOff, "设备关机（非执行周期中止）。"},
      {sbirs_sensor::session::codes::kTargetOcculted, "目标被地球遮挡（视线被地球遮蔽）。"},
      {sbirs_sensor::session::codes::kTargetOutOfRange, "目标超出作用距离（不在距离门 [min, max] 内）。"},
      {sbirs_sensor::session::codes::kTargetOutOfWfov, "目标宽视场外（不在 WFOV 扫描覆盖内）。"},
      {sbirs_sensor::session::codes::kTargetSnrBelowThreshold, "目标信噪比低于门限（低于 WFOV 最低 SNR）。"},
      {sbirs_sensor::session::codes::kCycleDeltaTimeExceedsFramePeriod, "周期步长超过帧周期合理范围（> 10 倍帧周期，即 10 / frame_rate_hz）。"},
      {sbirs_sensor::session::codes::kFrameRateNotPositive, "帧率非正。"},
      {sbirs_sensor::session::codes::kInvalidCycleDeltaTime, "周期步长非法（非正或非有限值）。"},
      {sbirs_sensor::session::codes::kInvalidDetectionThresholds, "检测门限非法（须非负）。"},
      {sbirs_sensor::session::codes::kInvalidDetectorBandwidth, "探测器带宽非法（须为正且有限）。"},
      {sbirs_sensor::session::codes::kInvalidErrorModelSigmas, "误差模型 sigma 非法（须为非负有限值）。"},
      {sbirs_sensor::session::codes::kInvalidEstimatedTrackingBackend, "估计跟踪后端非法。"},
      {sbirs_sensor::session::codes::kInvalidNarrowPointingSettleTolerance, "窄视场指向沉降容差非法（须为非负有限值）。"},
      {sbirs_sensor::session::codes::kInvalidNarrowPointingSlewRate, "窄视场指向最大转动速率非法（须为正且有限）。"},
      {sbirs_sensor::session::codes::kInvalidPointingDisturbanceCorrelation, "指向扰动相关时间非法（须为正且有限）。"},
      {sbirs_sensor::session::codes::kInvalidPointingDisturbanceValues, "指向扰动幅值与频率非法（须为非负有限值）。"},
      {sbirs_sensor::session::codes::kInvalidPointingDisturbanceVibrationFrequency, "指向扰动振动频率非法（幅值非零时须为正）。"},
      {sbirs_sensor::session::codes::kInvalidRangeGate, "距离门非法（min/max 未有序或为负）。"},
      {sbirs_sensor::session::codes::kInvalidSatellitePosition, "卫星位置缺失、非有限或为零向量。"},
      {sbirs_sensor::session::codes::kInvalidScanDirection, "扫描方向非法。"},
      {sbirs_sensor::session::codes::kInvalidScanRate, "扫描速率非法（须为非负有限值）。"},
      {sbirs_sensor::session::codes::kInvalidScanSpan, "扫描跨度非法（须为有限值且在 (0, 360]）。"},
      {sbirs_sensor::session::codes::kInvalidScanStartAzimuth, "扫描起始方位角非法（须为有限值且在 [-180, 180)）。"},
      {sbirs_sensor::session::codes::kInvalidTargetPhysical, "目标物理输入非法（ID/位置/辐射强度/速度等未满足有限与非负要求）。"},
      {sbirs_sensor::session::codes::kInvalidTrackingGateLossCycles, "跟踪门丢失周期数非法（须 >= 1）。"},
      {sbirs_sensor::session::codes::kInvalidTrackingMode, "跟踪模式非法。"},
      {sbirs_sensor::session::codes::kMissionFovNotPositive, "任务视场角非正。"},
      {sbirs_sensor::session::codes::kOpticalApertureNotPositive, "硬件光学孔径非正。"},
      {sbirs_sensor::session::codes::kWavelengthBandInvalid, "硬件波长带非法（须为正且有下界小于上界）。"},
      // sar (69)
      {sar::session::codes::kBpPeak, "BP 峰值定位（kInfo）。"},
      {sar::session::codes::kBpTraversal, "BP 遍历顺序（kInfo）。"},
      {sar::session::codes::kDegenerateImagePeak, "退化图像峰值（聚焦图像零峰值功率，管线无信号产出）。"},
      {sar::session::codes::kExternalRawIq, "外部完整孔径 raw IQ 消费诊断（kInfo）。"},
      {sar::session::codes::kExternalRawIqIdealTrajectoryRequired, "外部 raw IQ 需逐行理想脉冲状态（L2 路径）。"},
      {sar::session::codes::kExternalRawIqInvalidIdealTrajectory, "外部理想脉冲状态轨迹非法（须有限、连续且时间严格递增）。"},
      {sar::session::codes::kExternalRawIqInvalidTrajectory, "外部实际脉冲状态轨迹非法（须有限、连续且时间严格递增）。"},
      {sar::session::codes::kExternalRawIqNonFinite, "外部 raw IQ 含非有限采样。"},
      {sar::session::codes::kExternalRawIqRequiresL1Rda, "外部 raw IQ 需要 L1 RDA 或 L3 BP 成像。"},
      {sar::session::codes::kExternalRawIqShapeMismatch, "外部 raw IQ 形状不匹配（须与配置完整孔径精确一致）。"},
      {sar::session::codes::kExternalRawIqSnrUnavailable, "外部 raw IQ 无信噪比元数据，跳过链路预算与最小 SNR 门限（kInfo）。"},
      {sar::session::codes::kExternalRawIqTrajectoryIgnored, "外部脉冲状态被忽略（L1 RDA 且 L2 关闭时，kInfo）。"},
      {sar::session::codes::kExternalRawIqTrajectoryRequired, "外部 raw IQ 需逐行实际脉冲状态（BP/L2 路径）。"},
      {sar::session::codes::kL2TrackGenerationFailed, "L2 轨迹生成失败。"},
      {sar::session::codes::kL2Trajectory, "L2 扰动轨迹生成（kInfo）。"},
      {sar::session::codes::kL2TrajectoryHistoryMismatch, "L2 轨迹历史与最新原始孔径不匹配。"},
      {sar::session::codes::kL3BpFailed, "L3 BP 聚焦失败。"},
      {sar::session::codes::kL3BpPublicImageExportFailed, "L3 BP 公共聚焦图像导出失败。"},
      {sar::session::codes::kL3Trajectory, "L3 航路点轨迹生成（kInfo）。"},
      {sar::session::codes::kL3TrajectoryHistoryMismatch, "L3 轨迹历史与最新原始孔径不匹配。"},
      {sar::session::codes::kL3WaypointCoverage, "L3 航路点不覆盖固定 PRF 脉冲时间范围。"},
      {sar::session::codes::kL3WaypointGeometryFailed, "L3 航路点几何转换失败。"},
      {sar::session::codes::kMotionCompensation, "运动补偿（一阶运动补偿误差诊断，kInfo）。"},
      {sar::session::codes::kMotionCompensationFailed, "运动补偿失败。"},
      {sar::session::codes::kPlatformGeometryFailed, "平台几何转换失败（LLA 无法转为配置的本地几何）。"},
      {sar::session::codes::kPulseBufferPushFailed, "脉冲写入环缓冲失败。"},
      {sar::session::codes::kPulseBufferUnavailable, "脉冲环缓冲不可用。"},
      {sar::session::codes::kPulseHistoryUnavailable, "脉冲历史不可用（无法提供连续最新孔径）。"},
      {sar::session::codes::kPulseRingBuffer, "脉冲环缓冲复用与状态诊断（kInfo）。"},
      {sar::session::codes::kPulseSampleCountMismatch, "脉冲距离采样数与预期不符。"},
      {sar::session::codes::kRawEchoClipping, "回波裁剪（脉冲采样溢出采样窗口，kWarning）。"},
      {sar::session::codes::kRawEchoFailed, "回波生成失败（点目标与地面背景）。"},
      {sar::session::codes::kRdaFailed, "RDA 聚焦失败。"},
      {sar::session::codes::kRdaPeak, "RDA 峰值定位（聚焦图像峰值与多普勒诊断，kInfo）。"},
      {sar::session::codes::kRdaPublicImageExportFailed, "RDA 公共聚焦图像导出失败。"},
      {sar::session::codes::kRuntimeStateRestoreRejected, "运行期状态恢复被拒绝。"},
      {sar::session::codes::kSensorPoweredOff, "设备关机（非执行周期短路）。"},
      {sar::session::codes::kSlantRangeMismatch, "目标实际斜距与标称斜距严重错配（kWarning）。"},
      {sar::session::codes::kSnrBelowMinimum, "估计 SNR 低于配置最小有效值。"},
      {sar::session::codes::kSquintAngleExceedsLimit, "孔径斜视角超出配置成像限制。"},
      {sar::session::codes::kTargetGeometryFailed, "点目标几何转换失败。"},
      {sar::session::codes::kTrackGenerationFailed, "L1 条带航迹生成失败。"},
      {sar::session::codes::kAntennaLengthNotPositive, "天线长度非正。"},
      {sar::session::codes::kAzimuthPulseCountZero, "方位向脉冲数为零。"},
      {sar::session::codes::kBandwidthNotPositive, "带宽非正。"},
      {sar::session::codes::kCarrierFrequencyNotPositive, "载频非正。"},
      {sar::session::codes::kDesiredResolutionNotPositive, "期望地面距离/方位分辨率非正。"},
      {sar::session::codes::kEnvironmentConfigInvalid, "环境标量字段非法（须有限且大气损耗非负）。"},
      {sar::session::codes::kHardwareLinkBudgetInvalid, "硬件链路预算字段非法（功率须为正，噪声系数/损耗须非负且有限）。"},
      {sar::session::codes::kInvalidConfig, "运行期配置含非法硬件/任务字段。"},
      {sar::session::codes::kInvalidCycleDeltaTime, "周期步长非法（<= 0）。"},
      {sar::session::codes::kInvalidL2MotionCompensationConfig, "L2 运动补偿配置非法（需回波/RDA 且速度误差非负）。"},
      {sar::session::codes::kInvalidL3BpConfig, "L3 BP 配置非法（需回波与有效航路点且禁用 L1/L2 路径）。"},
      {sar::session::codes::kInvalidPulseSequence, "脉冲序号不连续或时间非单调递增。"},
      {sar::session::codes::kL3BpSizeGate, "L3 BP 尺寸超出 128x128 批准门限。"},
      {sar::session::codes::kNominalSlantRangeNotPositive, "标称斜距非正。"},
      {sar::session::codes::kNonFiniteCycleDeltaTime, "周期步长非有限。"},
      {sar::session::codes::kNonFinitePlatformField, "平台含非有限数值字段。"},
      {sar::session::codes::kNonFinitePulseField, "外部脉冲状态含非有限数值字段。"},
      {sar::session::codes::kNonFiniteTargetField, "点目标含非有限数值字段。"},
      {sar::session::codes::kPlatformSpeedNotPositive, "平台速度非正。"},
      {sar::session::codes::kPulseRepetitionFrequencyNotPositive, "脉冲重复频率（PRF）非正。"},
      {sar::session::codes::kRangeSampleCountZero, "距离向采样数为零。"},
      {sar::session::codes::kRdaRequiresRawEcho, "RDA 成像需要启用回波生成。"},
      {sar::session::codes::kRdaSizeGate, "RDA 尺寸超出批准运行门限（性能批准前限制场景规模）。"},
      {sar::session::codes::kSampleRateNotPositive, "采样率非正。"},
      {sar::session::codes::kSampleWindowTooSmallForPulse, "距离采样窗口容不下完整 LFM 脉冲。"},
      {sar::session::codes::kSquintAngleInvalid, "最大允许斜视角非法（须有限且在 [0, 90) 度）。"},
      {sar::session::codes::kWaveformGenerationFailed, "LFM 波形生成失败。"},
      // airborne_radar (36)
      {airborne_radar::session::codes::kInvalidEnvironmentCycle, "环境周期无效（未以正 dt_sec 初始化，非执行周期中止）。"},
      {airborne_radar::session::codes::kLifecycleUnavailable, "生命周期不可用（自动生命周期管理器缺失，非执行周期中止）。"},
      {airborne_radar::session::codes::kRuntimePreparationFailed, "运行期准备失败（非执行周期中止）。"},
      {airborne_radar::session::codes::kSensorPoweredOff, "设备关机（非执行周期中止）。"},
      {airborne_radar::session::codes::kTargetSnrBelowThreshold, "目标信噪比低于门限（按目标排除的 kInfo 诊断，规则 13b）。"},
      {airborne_radar::session::codes::kAntennaAzGeometryInvalid, "天线方位几何非法（标称波束宽度非正且无有效物理孔径）。"},
      {airborne_radar::session::codes::kAntennaElGeometryInvalid, "天线俯仰几何非法（标称波束宽度非正且无有效物理孔径）。"},
      {airborne_radar::session::codes::kCommandedBeamwidthAzNotPositive, "指令方位波束宽度非正（启用指令波束时须有限且为正）。"},
      {airborne_radar::session::codes::kCommandedBeamwidthElNotPositive, "指令俯仰波束宽度非正（启用指令波束时须有限且为正）。"},
      {airborne_radar::session::codes::kDuplicateExternalTargetId, "场景中外部目标 ID 重复。"},
      {airborne_radar::session::codes::kElectronicScanLimitsSwappedAz, "电扫描方位界限颠倒（min > max）。"},
      {airborne_radar::session::codes::kElectronicScanLimitsSwappedEl, "电扫描俯仰界限颠倒（min > max）。"},
      {airborne_radar::session::codes::kFrequencyPlanInvalid, "频率计划非法（须含有限正值且包含初始载频）。"},
      {airborne_radar::session::codes::kInterferenceFrameMismatch, "非空干扰帧与 AR 周期窗口不匹配。"},
      {airborne_radar::session::codes::kInvalidCycleDeltaTime, "周期步长非法（<= 0）。"},
      {airborne_radar::session::codes::kInvalidCycleIndex, "周期序号非法（为 0）。"},
      {airborne_radar::session::codes::kInvalidCycleStartTime, "周期起始时间非法（非有限或 < 0）。"},
      {airborne_radar::session::codes::kInvalidInterferenceInput, "干扰输入非法（帧含非法 RF 事实）。"},
      {airborne_radar::session::codes::kInvalidPlatformInput, "平台输入非法（标识为 0 或世界系运动学非有限）。"},
      {airborne_radar::session::codes::kInvalidTargetInput, "目标输入非法（世界系运动学无法转换为雷达本地系）。"},
      {airborne_radar::session::codes::kMechanicalScanLimitsSwappedAz, "机械扫描方位界限颠倒（min > max）。"},
      {airborne_radar::session::codes::kMechanicalScanLimitsSwappedEl, "机械扫描俯仰界限颠倒（min > max）。"},
      {airborne_radar::session::codes::kMissingRangeAndCartesianPosition, "目标斜距 <= 0 且无笛卡尔位置（二者须至少一为正）。"},
      {airborne_radar::session::codes::kNegativeRcs, "目标 RCS 为负（kWarning 级）。"},
      {airborne_radar::session::codes::kNonFiniteCycleDeltaTime, "周期步长非有限值。"},
      {airborne_radar::session::codes::kNonFiniteTargetField, "目标含非有限数值字段（位置/速度/RCS/斜距）。"},
      {airborne_radar::session::codes::kReceiverRfHardwareInvalid, "接收机 RF 硬件非法（隔离度/远场距离/线性输入限/共址路径无效）。"},
      {airborne_radar::session::codes::kTransmitterFrequencyInvalid, "发射机频率非法（须有限且为正）。"},
      {airborne_radar::session::codes::kTransmitterOperatingEnvelopeInvalid, "发射机工作包络非法（功率/占空比/脉冲能量超出硬件限制）。"},
      {airborne_radar::session::codes::kUnknownExternalTargetId, "目标外部 ID 未知（为 0，kInfo 级）。"},
      // remote_identification_radar (12)
      {remote_identification_radar::session::codes::kNonFiniteCycleDeltaTime, "周期步长非有限值。"},
      {remote_identification_radar::session::codes::kInvalidCycleDeltaTime, "周期步长非法（<= 0）。"},
      {remote_identification_radar::session::codes::kInvalidCycleIndex, "周期序号非法（为 0）。"},
      {remote_identification_radar::session::codes::kNonFiniteTargetField, "目标含非有限数值字段（位置/RCS/斜距/真值样本）。"},
      {remote_identification_radar::session::codes::kMissingRangeAndCartesianPosition, "目标斜距 <= 0 且无笛卡尔位置（二者须至少一为正）。"},
      {remote_identification_radar::session::codes::kDuplicateExternalTargetId, "场景中外部目标 ID 重复。"},
      {remote_identification_radar::session::codes::kInvalidTargetMotionField, "目标速度/起伏模型字段非法。"},
      {remote_identification_radar::session::codes::kInvalidEnvironmentSnapshot, "环境快照字段非法（天气衰减非有限/负值）。"},
      {remote_identification_radar::session::codes::kInvalidRfSceneFrame, "RF 场景帧非法或与周期窗口不一致。"},
      {remote_identification_radar::session::codes::kDetectionPolicyInvalid, "检测策略非法。"},
      {remote_identification_radar::session::codes::kAssociationPolicyInvalid, "关联策略非法。"},
      {remote_identification_radar::session::codes::kTrackingPolicyInvalid, "跟踪策略非法。"},
      {remote_identification_radar::session::codes::kLifecyclePolicyInvalid, "生命周期策略非法。"},
      {remote_identification_radar::session::codes::kRecognitionAccumulationInvalid, "识别累积计数非法（须至少为 1）。"},
      {remote_identification_radar::session::codes::kRecognitionDatabasePathMissing, "识别数据库路径缺失（启用识别时须非空）。"},
      {remote_identification_radar::session::codes::kRecognitionThresholdInvalid, "识别门限非法（接受分数/最小裕度须在 [0, 1]）。"},
      {remote_identification_radar::session::codes::kRecognitionTimeRangeInvalid, "识别时间范围非法（保持时间须非负；最大距离/驻留/累积窗口须有限且为正）。"},
      {remote_identification_radar::session::codes::kRecognitionWeightsInvalid, "识别特征权重非法（须有限、在 [0, 1] 且总和为 1）。"},
      {remote_identification_radar::session::codes::kTargetDetectionGate, "检测准入门未过（正常周期按目标排除的 kInfo 诊断，规则 13b；携带门内归因主因）。"},
      {remote_identification_radar::session::codes::kTargetBeyondRecognitionRange, "目标斜距超识别最大作用距离（正常周期按目标排除的 kInfo 诊断，规则 13b）。"},
      {remote_identification_radar::session::codes::kTargetModeNotIdentify, "本周期非识别工作模式，不建识别观测（正常周期按目标排除的 kInfo 诊断，规则 13b）。"},
      {remote_identification_radar::session::codes::kTargetNoFeatureDatabase, "特征库缺失或加载失败，特征链空（正常周期按目标排除的 kInfo 诊断，规则 13b）。"},
      // electronic_surveillance_radar (24)
      {electronic_surveillance_radar::session::codes::kEmissionBelowThreshold, "发射源低于检测门限（正常周期按发射源排除的 kInfo 诊断）。"},
      {electronic_surveillance_radar::session::codes::kEmissionCoSite, "发射源同址干扰（正常周期按发射源排除的 kInfo 诊断，规则 13b）。"},
      {electronic_surveillance_radar::session::codes::kEmissionZeroPower, "发射源接收功率为零（正常周期按发射源排除的 kInfo 诊断）。"},
      {electronic_surveillance_radar::session::codes::kRfReceiverRejected, "射频接收被拒（RF v2 接收机拒绝本周期）。"},
      {electronic_surveillance_radar::session::codes::kSensorPoweredOff, "设备关机（非执行周期中止）。"},
      {electronic_surveillance_radar::session::codes::kBeamAzWidthNotPositive, "方位波束宽度非正。"},
      {electronic_surveillance_radar::session::codes::kBeamElWidthNotPositive, "俯仰波束宽度非正。"},
      {electronic_surveillance_radar::session::codes::kDetectionPolicyInvalid, "检测策略无效（SNR 须有限、Pfa 在 (0,1)、脉冲数与门限倍率须为正）。"},
      {electronic_surveillance_radar::session::codes::kEnvironmentInvalid, "环境预设或启用的大气物理参数无效。"},
      {electronic_surveillance_radar::session::codes::kExplicitScanBoundsAzSwapped, "显式扫描方位起止颠倒（起始 >= 结束）。"},
      {electronic_surveillance_radar::session::codes::kExplicitScanBoundsElSwapped, "显式扫描俯仰起止颠倒（起始 >= 结束）。"},
      {electronic_surveillance_radar::session::codes::kExplicitScanBoundsNotFinite, "显式扫描边界含非有限数值。"},
      {electronic_surveillance_radar::session::codes::kInvalidCycleDeltaTime, "周期步长非法（<= 0）。"},
      {electronic_surveillance_radar::session::codes::kInvalidCycleStartTime, "周期起始时刻非有限值。"},
      {electronic_surveillance_radar::session::codes::kInvalidRfEmissionFrame, "RF 发射帧非法（不匹配周期窗口或平台身份/ECEF 运动学无效）。"},
      {electronic_surveillance_radar::session::codes::kMissionEnumInvalid, "任务枚举无效（工作模式/扫描起始位/扫描序列须为已知值）。"},
      {electronic_surveillance_radar::session::codes::kNonFiniteCycleDeltaTime, "周期步长非有限值。"},
      {electronic_surveillance_radar::session::codes::kNonFinitePlatformNumericField, "平台姿态角含非有限数值。"},
      {electronic_surveillance_radar::session::codes::kReceiverBandLowerAboveUpper, "接收频段下界高于或等于上界。"},
      {electronic_surveillance_radar::session::codes::kReceiverRfHardwareInvalid, "接收机 RF 硬件参数非法（须有限且物理有效，含同址路径）。"},
      {electronic_surveillance_radar::session::codes::kScanCenterNotFinite, "扫描中心角非有限数值。"},
      {electronic_surveillance_radar::session::codes::kScanRateNotPositive, "扫描速率非正（须为有限正值）。"},
      {electronic_surveillance_radar::session::codes::kTuningPlanInvalid, "调谐计划无效（调谐窗口须有限、非空且在硬件频段内）。"},
      {electronic_surveillance_radar::session::codes::kUnlocatablePlatformEcef, "平台 ECEF 不可定位（无法转换为有效 WGS84 LLA）。"},  };
  for (const auto& entry : kNames) {
    if (code == entry.code) {
      return entry.name;
    }
  }
  return nullptr;
}

/// 问题列表 → 人读文本（逗号分隔）：已知 code 输出 "code 中文名"；未知 code
/// 回退英文 message 原文（规则 13b：不翻译/不解析 message，量值走结构化字段）。
/// @tparam TIssue 各模块 *Issue 结构（含 code/message 字段）。
template <typename TIssue>
inline std::string FormatIssueText(const std::vector<TIssue>& issues) {
  std::string text;
  for (const auto& issue : issues) {
    if (!text.empty()) {
      text += ", ";
    }
    const char* zh = IssueCodeChineseName(issue.code);
    if (zh != nullptr) {
      text += issue.code + " " + zh;
    } else if (!issue.message.empty()) {
      text += issue.code + ": " + issue.message;
    } else {
      text += issue.code;
    }
  }
  return text;
}

}  // namespace app
}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_I18N_H_
