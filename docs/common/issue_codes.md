---
Status: active
Last-reviewed: 2026-08-23
---

# Issue Code 目录（六模块总表）

> 本表由各模块 `include/1q/<module>/session/<Module>IssueCodes.h` 的 `@brief` 注释
> 程序化提取生成（2026-08-20），是 code 全集的人读辅助。**机器消费以公开头文件常量
> 为唯一事实来源（规则 14c）**：集成方取 code 全集直接 include 各 `IssueCodes.h`，
> 新增/改名 code 后本表需重新生成，如有出入以头文件为准。

## 使用说明

- **code 是机器消费的稳定契约**（规则 13b/14c）：量值与状态判断一律走结构化字段，不得解析 `message`。
- **排除诊断门内归因**（规则 13b 归因条款）：聚合门（如 AR 的 SNR 门）排除时 `*Issue.cause`
  （各模块 `<Module>IssueCause` 枚举）给出机器可读主因；具体门排除 `cause` 为 `kNone`，
  关键量值进 `message`。`cause` 不替代 `code`，不用于状态判断。
- **中文含义为人读辅助**：集成方日志/告警展示可直接使用；展示语义非库承诺，库内以 `@brief` 注释为准。
- 各模块 issue code 常量定义与中文注释见对应 `<Module>IssueCodes.h`；日志兜底串
  （`*.validation_rejected`、`*.unknown` 等，仅用于 `AbortReasonToDiagnosticCode` 日志映射、
  不产生为 issue code）不在此登记。

## airborne_radar（机载雷达）

| code | 类型 | 中文含义 |
|---|---|---|
| `ar.invalid_environment_cycle` | 执行/外部输入诊断 | 环境周期无效（未以正 dt_sec 初始化，非执行周期中止）。 |
| `ar.lifecycle_unavailable` | 执行/外部输入诊断 | 生命周期不可用（自动生命周期管理器缺失，非执行周期中止）。 |
| `ar.runtime_preparation_failed` | 执行/外部输入诊断 | 运行期准备失败（非执行周期中止）。 |
| `ar.sensor_powered_off` | 执行/外部输入诊断 | 设备关机（非执行周期中止）。 |
| `ar.target_snr_below_threshold` | 执行/外部输入诊断 | 目标信噪比低于门限（按目标排除的 kInfo 诊断，规则 13b）。 |
| `ar.validation.antenna_az_geometry_invalid` | 输入校验 | 天线方位几何非法（标称波束宽度非正且无有效物理孔径）。 |
| `ar.validation.antenna_el_geometry_invalid` | 输入校验 | 天线俯仰几何非法（标称波束宽度非正且无有效物理孔径）。 |
| `ar.validation.commanded_beamwidth_az_not_positive` | 输入校验 | 指令方位波束宽度非正（启用指令波束时须有限且为正）。 |
| `ar.validation.commanded_beamwidth_el_not_positive` | 输入校验 | 指令俯仰波束宽度非正（启用指令波束时须有限且为正）。 |
| `ar.validation.duplicate_external_target_id` | 输入校验 | 场景中外部目标 ID 重复。 |
| `ar.validation.electronic_scan_limits_swapped_az` | 输入校验 | 电扫描方位界限颠倒（min > max）。 |
| `ar.validation.electronic_scan_limits_swapped_el` | 输入校验 | 电扫描俯仰界限颠倒（min > max）。 |
| `ar.validation.frequency_plan_invalid` | 输入校验 | 频率计划非法（须含有限正值且包含初始载频）。 |
| `ar.validation.interference_frame_mismatch` | 输入校验 | 非空干扰帧与 AR 周期窗口不匹配。 |
| `ar.validation.invalid_cycle_delta_time` | 输入校验 | 周期步长非法（<= 0）。 |
| `ar.validation.invalid_cycle_index` | 输入校验 | 周期序号非法（为 0）。 |
| `ar.validation.invalid_cycle_start_time` | 输入校验 | 周期起始时间非法（非有限或 < 0）。 |
| `ar.validation.invalid_interference_input` | 输入校验 | 干扰输入非法（帧含非法 RF 事实）。 |
| `ar.validation.invalid_platform_input` | 输入校验 | 平台输入非法（标识为 0 或世界系运动学非有限）。 |
| `ar.validation.invalid_target_input` | 输入校验 | 目标输入非法（世界系运动学无法转换为雷达本地系）。 |
| `ar.validation.mechanical_scan_limits_swapped_az` | 输入校验 | 机械扫描方位界限颠倒（min > max）。 |
| `ar.validation.mechanical_scan_limits_swapped_el` | 输入校验 | 机械扫描俯仰界限颠倒（min > max）。 |
| `ar.validation.missing_range_and_cartesian_position` | 输入校验 | 目标斜距 <= 0 且无笛卡尔位置（二者须至少一为正）。 |
| `ar.validation.negative_rcs` | 输入校验 | 目标 RCS 为负（kWarning 级）。 |
| `ar.validation.non_finite_cycle_delta_time` | 输入校验 | 周期步长非有限值。 |
| `ar.validation.non_finite_target_field` | 输入校验 | 目标含非有限数值字段（位置/速度/RCS/斜距）。 |
| `ar.validation.receiver_rf_hardware_invalid` | 输入校验 | 接收机 RF 硬件非法（隔离度/远场距离/线性输入限/共址路径无效）。 |
| `ar.validation.signal_processing_gains_invalid` | 输入校验 | 信号处理增益偏置非法（四偏置须有限且在 [0, 40] dB）。 |
| `ar.validation.transmitter_frequency_invalid` | 输入校验 | 发射机频率非法（须有限且为正）。 |
| `ar.validation.transmitter_operating_envelope_invalid` | 输入校验 | 发射机工作包络非法（功率/占空比/脉冲能量超出硬件限制）。 |
| `ar.validation.unknown_external_target_id` | 输入校验 | 目标外部 ID 未知（为 0，kInfo 级）。 |

## electro_optical_sensor（光电传感器）

| code | 类型 | 中文含义 |
|---|---|---|
| `eos.pipeline_contract_violation` | 执行/外部输入诊断 | 管线输出违反内部契约（kOutputContract 相位）。 |
| `eos.runtime_state_restore_rejected` | 执行/外部输入诊断 | 运行期状态恢复被拒绝。 |
| `eos.sensor_powered_off` | 执行/外部输入诊断 | 设备关机（非执行周期中止）。 |
| `eos.target_out_of_fov` | 执行/外部输入诊断 | 目标视场外（正常周期按目标排除的 kInfo 诊断，规则 13b）。 |
| `eos.validation.atmospheric_physics_invalid` | 输入校验 | 启用物理模型时大气物理参数无效（压力/温度/湿度越界）。 |
| `eos.validation.cycle_delta_time_exceeds_frame_period` | 输入校验 | 周期步长超过帧周期合理范围（> 10 倍帧周期，即 10 / frame_rate_hz）。 |
| `eos.validation.environment_preset_invalid` | 输入校验 | 环境预设无效。 |
| `eos.validation.frame_rate_not_positive` | 输入校验 | 帧率非正。 |
| `eos.validation.horizontal_fov_not_positive` | 输入校验 | 水平视场角非正。 |
| `eos.validation.inconsistent_target_energy_balance` | 输入校验 | 目标能量平衡校验失败（辐射效率 + 反射率 > 1，kInfo 级）。 |
| `eos.validation.invalid_cycle_delta_time` | 输入校验 | 周期步长非法（<= 0）。 |
| `eos.validation.invalid_target_emissivity` | 输入校验 | 目标辐射效率非法（不在 [0, 1]）。 |
| `eos.validation.invalid_target_projected_area` | 输入校验 | 目标投影面积非法（<= 0）。 |
| `eos.validation.invalid_target_range` | 输入校验 | 目标斜距非法（<= 0）。 |
| `eos.validation.invalid_target_reflectance` | 输入校验 | 目标反射率非法（不在 [0, 1]）。 |
| `eos.validation.invalid_target_temperature` | 输入校验 | 目标温度非法（<= 0）。 |
| `eos.validation.non_finite_cycle_delta_time` | 输入校验 | 周期步长非有限值。 |
| `eos.validation.non_finite_platform_numeric_field` | 输入校验 | 平台位姿含非有限数值（位置/速度/姿态角）。 |
| `eos.validation.non_finite_target_numeric_field` | 输入校验 | 目标含非有限数值字段。 |
| `eos.validation.scan_range_az_swapped` | 输入校验 | 扫描方位起止颠倒（起始 >= 结束）。 |
| `eos.validation.scan_rate_not_positive` | 输入校验 | 扫描速率非正。 |
| `eos.validation.vertical_fov_not_positive` | 输入校验 | 垂直视场角非正。 |

## electronic_surveillance_radar（电子侦察雷达）

| code | 类型 | 中文含义 |
|---|---|---|
| `esr.emission_below_threshold` | 执行/外部输入诊断 | 发射源低于检测门限（正常周期按发射源排除的 kInfo 诊断）。 |
| `esr.emission_co_site` | 执行/外部输入诊断 | 发射源同址干扰（正常周期按发射源排除的 kInfo 诊断，规则 13b）。 |
| `esr.emission_zero_power` | 执行/外部输入诊断 | 发射源接收功率为零（正常周期按发射源排除的 kInfo 诊断）。 |
| `esr.rf_receiver_rejected` | 执行/外部输入诊断 | 射频接收被拒（RF v2 接收机拒绝本周期）。 |
| `esr.sensor_powered_off` | 执行/外部输入诊断 | 设备关机（非执行周期中止）。 |
| `esr.validation.beam_az_width_not_positive` | 输入校验 | 方位波束宽度非正。 |
| `esr.validation.beam_el_width_not_positive` | 输入校验 | 俯仰波束宽度非正。 |
| `esr.validation.detection_policy_invalid` | 输入校验 | 检测策略无效（SNR 须有限、Pfa 在 (0,1)、脉冲数与门限倍率须为正）。 |
| `esr.validation.environment_invalid` | 输入校验 | 环境预设或启用的大气物理参数无效。 |
| `esr.validation.explicit_scan_bounds_az_swapped` | 输入校验 | 显式扫描方位起止颠倒（起始 >= 结束）。 |
| `esr.validation.explicit_scan_bounds_el_swapped` | 输入校验 | 显式扫描俯仰起止颠倒（起始 >= 结束）。 |
| `esr.validation.explicit_scan_bounds_not_finite` | 输入校验 | 显式扫描边界含非有限数值。 |
| `esr.validation.invalid_cycle_delta_time` | 输入校验 | 周期步长非法（<= 0）。 |
| `esr.validation.invalid_cycle_start_time` | 输入校验 | 周期起始时刻非有限值。 |
| `esr.validation.invalid_platform_kinematics` | 输入校验 | 平台身份/ECEF 运动学无效（实体标识为零或位置/速度含非有限值；2026-08-24 起从 invalid_rf_emission_frame 拆出）。 |
| `esr.validation.invalid_rf_emission_frame` | 输入校验 | RF 发射帧非法（不匹配周期窗口）。 |
| `esr.validation.mission_enum_invalid` | 输入校验 | 任务枚举无效（工作模式/扫描起始位/扫描序列须为已知值）。 |
| `esr.validation.non_finite_cycle_delta_time` | 输入校验 | 周期步长非有限值。 |
| `esr.validation.non_finite_platform_numeric_field` | 输入校验 | 平台姿态角含非有限数值。 |
| `esr.validation.receiver_band_lower_above_upper` | 输入校验 | 接收频段下界高于或等于上界。 |
| `esr.validation.receiver_rf_hardware_invalid` | 输入校验 | 接收机 RF 硬件参数非法（须有限且物理有效，含同址路径）。 |
| `esr.validation.scan_center_not_finite` | 输入校验 | 扫描中心角非有限数值。 |
| `esr.validation.scan_rate_not_positive` | 输入校验 | 扫描速率非正（须为有限正值）。 |
| `esr.validation.tuning_plan_invalid` | 输入校验 | 调谐计划无效（调谐窗口须有限、非空且在硬件频段内）。 |
| `esr.validation.unlocatable_platform_ecef` | 输入校验 | 平台 ECEF 不可定位（无法转换为有效 WGS84 LLA）。 |

## sar（合成孔径雷达）

| code | 类型 | 中文含义 |
|---|---|---|
| `sar.bp_peak` | 执行/外部输入诊断 | BP 峰值定位（kInfo）。 |
| `sar.bp_traversal` | 执行/外部输入诊断 | BP 遍历顺序（kInfo）。 |
| `sar.degenerate_image_peak` | 执行/外部输入诊断 | 退化图像峰值（聚焦图像零峰值功率，管线无信号产出）。 |
| `sar.external_raw_iq` | 执行/外部输入诊断 | 外部完整孔径 raw IQ 消费诊断（kInfo）。 |
| `sar.external_raw_iq_ideal_trajectory_required` | 执行/外部输入诊断 | 外部 raw IQ 需逐行理想脉冲状态（L2 路径）。 |
| `sar.external_raw_iq_invalid_ideal_trajectory` | 执行/外部输入诊断 | 外部理想脉冲状态轨迹非法（须有限、连续且时间严格递增）。 |
| `sar.external_raw_iq_invalid_trajectory` | 执行/外部输入诊断 | 外部实际脉冲状态轨迹非法（须有限、连续且时间严格递增）。 |
| `sar.external_raw_iq_non_finite` | 执行/外部输入诊断 | 外部 raw IQ 含非有限采样。 |
| `sar.external_raw_iq_requires_l1_rda` | 执行/外部输入诊断 | 外部 raw IQ 需要 L1 RDA 或 L3 BP 成像。 |
| `sar.external_raw_iq_shape_mismatch` | 执行/外部输入诊断 | 外部 raw IQ 形状不匹配（须与配置完整孔径精确一致）。 |
| `sar.external_raw_iq_snr_unavailable` | 执行/外部输入诊断 | 外部 raw IQ 无信噪比元数据，跳过链路预算与最小 SNR 门限（kInfo）。 |
| `sar.external_raw_iq_trajectory_ignored` | 执行/外部输入诊断 | 外部脉冲状态被忽略（L1 RDA 且 L2 关闭时，kInfo）。 |
| `sar.external_raw_iq_trajectory_required` | 执行/外部输入诊断 | 外部 raw IQ 需逐行实际脉冲状态（BP/L2 路径）。 |
| `sar.l2_track_generation_failed` | 执行/外部输入诊断 | L2 轨迹生成失败。 |
| `sar.l2_trajectory` | 执行/外部输入诊断 | L2 扰动轨迹生成（kInfo）。 |
| `sar.l2_trajectory_history_mismatch` | 执行/外部输入诊断 | L2 轨迹历史与最新原始孔径不匹配。 |
| `sar.l3_bp_failed` | 执行/外部输入诊断 | L3 BP 聚焦失败。 |
| `sar.l3_bp_public_image_export_failed` | 执行/外部输入诊断 | L3 BP 公共聚焦图像导出失败。 |
| `sar.l3_trajectory` | 执行/外部输入诊断 | L3 航路点轨迹生成（kInfo）。 |
| `sar.l3_trajectory_history_mismatch` | 执行/外部输入诊断 | L3 轨迹历史与最新原始孔径不匹配。 |
| `sar.l3_waypoint_coverage` | 执行/外部输入诊断 | L3 航路点不覆盖固定 PRF 脉冲时间范围。 |
| `sar.l3_waypoint_geometry_failed` | 执行/外部输入诊断 | L3 航路点几何转换失败。 |
| `sar.motion_compensation` | 执行/外部输入诊断 | 运动补偿（一阶运动补偿误差诊断，kInfo）。 |
| `sar.motion_compensation_failed` | 执行/外部输入诊断 | 运动补偿失败。 |
| `sar.platform_geometry_failed` | 执行/外部输入诊断 | 平台几何转换失败（LLA 无法转为配置的本地几何）。 |
| `sar.pulse_buffer_push_failed` | 执行/外部输入诊断 | 脉冲写入环缓冲失败。 |
| `sar.pulse_buffer_unavailable` | 执行/外部输入诊断 | 脉冲环缓冲不可用。 |
| `sar.pulse_history_unavailable` | 执行/外部输入诊断 | 脉冲历史不可用（无法提供连续最新孔径）。 |
| `sar.pulse_ring_buffer` | 执行/外部输入诊断 | 脉冲环缓冲复用与状态诊断（kInfo）。 |
| `sar.pulse_sample_count_mismatch` | 执行/外部输入诊断 | 脉冲距离采样数与预期不符。 |
| `sar.raw_echo_clipping` | 执行/外部输入诊断 | 回波裁剪（脉冲采样溢出采样窗口，kWarning）。 |
| `sar.raw_echo_failed` | 执行/外部输入诊断 | 回波生成失败（点目标与地面背景）。 |
| `sar.rda_failed` | 执行/外部输入诊断 | RDA 聚焦失败。 |
| `sar.rda_peak` | 执行/外部输入诊断 | RDA 峰值定位（聚焦图像峰值与多普勒诊断，kInfo）。 |
| `sar.rda_public_image_export_failed` | 执行/外部输入诊断 | RDA 公共聚焦图像导出失败。 |
| `sar.runtime_state_restore_rejected` | 执行/外部输入诊断 | 运行期状态恢复被拒绝。 |
| `sar.sensor_powered_off` | 执行/外部输入诊断 | 设备关机（非执行周期短路）。 |
| `sar.slant_range_mismatch` | 执行/外部输入诊断 | 目标实际斜距与标称斜距严重错配（kWarning）。 |
| `sar.snr_below_minimum` | 执行/外部输入诊断 | 估计 SNR 低于配置最小有效值。 |
| `sar.squint_angle_exceeds_limit` | 执行/外部输入诊断 | 孔径斜视角超出配置成像限制。 |
| `sar.target_geometry_failed` | 执行/外部输入诊断 | 点目标几何转换失败。 |
| `sar.track_generation_failed` | 执行/外部输入诊断 | L1 条带航迹生成失败。 |
| `sar.validation.antenna_length_not_positive` | 输入校验 | 天线长度非正。 |
| `sar.validation.azimuth_pulse_count_zero` | 输入校验 | 方位向脉冲数为零。 |
| `sar.validation.bandwidth_not_positive` | 输入校验 | 带宽非正。 |
| `sar.validation.carrier_frequency_not_positive` | 输入校验 | 载频非正。 |
| `sar.validation.desired_resolution_not_positive` | 输入校验 | 期望地面距离/方位分辨率非正。 |
| `sar.validation.environment_config_invalid` | 输入校验 | 环境标量字段非法（须有限且大气损耗非负）。 |
| `sar.validation.hardware_link_budget_invalid` | 输入校验 | 硬件链路预算字段非法（功率须为正，噪声系数/损耗须非负且有限）。 |
| `sar.validation.invalid_config` | 输入校验 | 运行期配置含非法硬件/任务字段。 |
| `sar.validation.invalid_cycle_delta_time` | 输入校验 | 周期步长非法（<= 0）。 |
| `sar.validation.invalid_l2_motion_compensation_config` | 输入校验 | L2 运动补偿配置非法（需回波/RDA 且速度误差非负）。 |
| `sar.validation.invalid_l3_bp_config` | 输入校验 | L3 BP 配置非法（需回波与有效航路点且禁用 L1/L2 路径）。 |
| `sar.validation.invalid_pulse_sequence` | 输入校验 | 脉冲序号不连续或时间非单调递增。 |
| `sar.validation.l3_bp_size_gate` | 输入校验 | L3 BP 尺寸超出 128x128 批准门限。 |
| `sar.validation.nominal_slant_range_not_positive` | 输入校验 | 标称斜距非正。 |
| `sar.validation.non_finite_cycle_delta_time` | 输入校验 | 周期步长非有限。 |
| `sar.validation.non_finite_platform_field` | 输入校验 | 平台含非有限数值字段。 |
| `sar.validation.non_finite_pulse_field` | 输入校验 | 外部脉冲状态含非有限数值字段。 |
| `sar.validation.non_finite_target_field` | 输入校验 | 点目标含非有限数值字段。 |
| `sar.validation.platform_speed_not_positive` | 输入校验 | 平台速度非正。 |
| `sar.validation.pulse_repetition_frequency_not_positive` | 输入校验 | 脉冲重复频率（PRF）非正。 |
| `sar.validation.range_sample_count_zero` | 输入校验 | 距离向采样数为零。 |
| `sar.validation.rda_requires_raw_echo` | 输入校验 | RDA 成像需要启用回波生成。 |
| `sar.validation.rda_size_gate` | 输入校验 | RDA 尺寸超出批准运行门限（性能批准前限制场景规模）。 |
| `sar.validation.sample_rate_not_positive` | 输入校验 | 采样率非正。 |
| `sar.validation.sample_window_too_small_for_pulse` | 输入校验 | 距离采样窗口容不下完整 LFM 脉冲。 |
| `sar.validation.squint_angle_invalid` | 输入校验 | 最大允许斜视角非法（须有限且在 [0, 90) 度）。 |
| `sar.waveform_generation_failed` | 执行/外部输入诊断 | LFM 波形生成失败。 |

## sbirs_sensor（天基红外传感器）

| code | 类型 | 中文含义 |
|---|---|---|
| `sbirs.sensor_powered_off` | 执行/外部输入诊断 | 设备关机（非执行周期中止）。 |
| `sbirs.target_occulted` | 执行/外部输入诊断 | 目标被地球遮挡（视线被地球遮蔽）。 |
| `sbirs.target_out_of_range` | 执行/外部输入诊断 | 目标超出作用距离（不在距离门 [min, max] 内）。 |
| `sbirs.target_out_of_wfov` | 执行/外部输入诊断 | 目标宽视场外（不在 WFOV 扫描覆盖内）。 |
| `sbirs.target_snr_below_threshold` | 执行/外部输入诊断 | 目标信噪比低于门限（低于 WFOV 最低 SNR）。 |
| `sbirs.validation.cycle_delta_time_exceeds_frame_period` | 输入校验 | 周期步长超过帧周期合理范围（> 10 倍帧周期，即 10 / frame_rate_hz）。 |
| `sbirs.validation.focal_plane_config_not_positive` | 输入校验 | 焦平面配置非法（焦距/像元间距须为正有限值；焦平面脱靶量映射）。 |
| `sbirs.validation.frame_rate_not_positive` | 输入校验 | 帧率非正。 |
| `sbirs.validation.invalid_cycle_delta_time` | 输入校验 | 周期步长非法（非正或非有限值）。 |
| `sbirs.validation.invalid_detection_thresholds` | 输入校验 | 检测门限非法（须非负）。 |
| `sbirs.validation.invalid_detector_bandwidth` | 输入校验 | 探测器带宽非法（须为正且有限）。 |
| `sbirs.validation.invalid_error_model_sigmas` | 输入校验 | 误差模型 sigma 非法（须为非负有限值）。 |
| `sbirs.validation.invalid_estimated_tracking_backend` | 输入校验 | 估计跟踪后端非法。 |
| `sbirs.validation.invalid_misalignment` | 输入校验 | 安装失准非法（bias 须有限、random sigma 须非负有限）。 |
| `sbirs.validation.invalid_mount_angles` | 输入校验 | 传感器安装欧拉角非法（须为有限值）。 |
| `sbirs.validation.invalid_narrow_pointing_settle_tolerance` | 输入校验 | 窄视场指向沉降容差非法（须为非负有限值）。 |
| `sbirs.validation.invalid_narrow_pointing_slew_rate` | 输入校验 | 窄视场指向最大转动速率非法（须为正且有限）。 |
| `sbirs.validation.invalid_pointing_disturbance_correlation` | 输入校验 | 指向扰动相关时间非法（须为正且有限）。 |
| `sbirs.validation.invalid_pointing_disturbance_values` | 输入校验 | 指向扰动幅值与频率非法（须为非负有限值）。 |
| `sbirs.validation.invalid_pointing_disturbance_vibration_frequency` | 输入校验 | 指向扰动振动频率非法（幅值非零时须为正）。 |
| `sbirs.validation.invalid_range_gate` | 输入校验 | 距离门非法（min/max 未有序或为负）。 |
| `sbirs.validation.invalid_satellite_attitude` | 输入校验 | 卫星姿态非有限（必填；零欧拉合法 = 体轴对齐 ECI）。 |
| `sbirs.validation.invalid_satellite_position` | 输入校验 | 卫星位置非有限或为零向量（必填）。 |
| `sbirs.validation.invalid_satellite_velocity` | 输入校验 | 卫星速度非有限（必填；ECEF 零向量合法，如 GEO 卫星）。 |
| `sbirs.validation.invalid_scan_direction` | 输入校验 | 扫描方向非法。 |
| `sbirs.validation.invalid_scan_elevation_raster` | 输入校验 | 俯仰栅格非法（span 须非负有限、step 须正有限）。 |
| `sbirs.validation.invalid_scan_rate` | 输入校验 | 扫描速率非法（须为非负有限值）。 |
| `sbirs.validation.invalid_scan_span` | 输入校验 | 扫描跨度非法（须为有限值且在 (0, 360]）。 |
| `sbirs.validation.invalid_scan_start_azimuth` | 输入校验 | 扫描起始方位角非法（须为有限值且在 [-180, 180)）。 |
| `sbirs.validation.invalid_scheduler_nfov_locks` | 输入校验 | 调度器最大并发 NFOV 锁定数非法（须 >= 1）。 |
| `sbirs.validation.invalid_stabilization_mode` | 输入校验 | 扫描稳定方式非法。 |
| `sbirs.validation.invalid_target_physical` | 输入校验 | 目标物理输入非法（ID/位置/辐射强度/速度等未满足有限与非负要求）。 |
| `sbirs.validation.invalid_tracking_gate_loss_cycles` | 输入校验 | 跟踪门丢失周期数非法（须 >= 1）。 |
| `sbirs.validation.invalid_tracking_mode` | 输入校验 | 跟踪模式非法。 |
| `sbirs.validation.invalid_utc_julian_day` | 输入校验 | UTC 儒略日缺失、非有限或非正（ECI 输出参考系必需）。 |
| `sbirs.validation.invalid_wide_to_narrow_required_hits` | 输入校验 | 宽窄切换连续命中阈值非法（须 >=1；宽窄切换前置条件）。 |
| `sbirs.validation.mission_fov_not_positive` | 输入校验 | 任务视场角非正。 |
| `sbirs.validation.optical_aperture_not_positive` | 输入校验 | 硬件光学孔径非正。 |
| `sbirs.validation.scan_elevation_step_exceeds_fov` | 输入校验 | 俯仰栅格行间距超过 WFOV 俯仰视场（无隙覆盖预算违反）。 |
| `sbirs.validation.scan_path_outside_sensor_limits` | 输入校验 | 扫描路径超出传感器系扫描限位（方位扫掠区间或中心俯仰不在限位内）。 |
| `sbirs.validation.sensor_scan_limits_out_of_range` | 输入校验 | 传感器系扫描限位超域（az 须在 [-180, 180]、el 须在 [-90, 90]）。 |
| `sbirs.validation.sensor_scan_limits_swapped_azimuth` | 输入校验 | 传感器系扫描限位方位倒置（az_min > az_max）。 |
| `sbirs.validation.sensor_scan_limits_swapped_elevation` | 输入校验 | 传感器系扫描限位俯仰倒置（el_min > el_max）。 |
| `sbirs.validation.wavelength_band_invalid` | 输入校验 | 硬件波长带非法（须为正且有下界小于上界）。 |

## remote_identification_radar（远程识别雷达）

> 输入/配置校验问题（`rir.validation.<snake_case>`）、执行期按目标门控排除诊断
> （`rir.target_<snake_case>`，规则 13b）与执行中止细码（`rir.sensor_powered_off`，规则 9）三类。

| code | 类型 | 中文含义 |
|---|---|---|
| `rir.sensor_powered_off` | 执行/外部输入诊断 | 设备关机（非执行周期中止）。 |
| `rir.target_beyond_recognition_range` | 执行排除 | 目标斜距超识别最大作用距离（识别链距离门，检测/跟踪不受影响）。 |
| `rir.target_detection_gate` | 执行排除 | 检测准入门未过（聚合门：SNR/检测器判决；携带门内归因主因）。 |
| `rir.target_earth_occulted` | 执行排除 | 目标视线被地球圆球遮挡（有限弦穿地，相切算遮挡）。 |
| `rir.target_receiver_front_end_saturated` | 执行排除 | 接收前端饱和周期致盲（周期级门：入射总功率越接收线性上限，本周期全部目标不做检测判决；排在检测准入门之前）。
| `rir.target_mode_not_identify` | 执行排除 | 本周期非识别工作模式，不建识别观测（STBY 全局模式门）。 |
| `rir.target_no_feature_database` | 执行排除 | 特征库缺失或加载失败，特征链空（识别积累保持）。 |
| `rir.target_outside_search_volume` | 执行排除 | 目标视线角出可扫描体积（角域裁剪）。 |
| `rir.target_outside_beam_coverage` | 执行排除 | 目标在搜索扇区内但不被本周期驻留主瓣覆盖（"探测⟺波束照到"硬门，门限=有效波束宽度半功率宽；2026-08-29 方向图恒开配套语义）。 |
| `rir.validation.antenna_az_geometry_invalid` | 输入校验 | 天线方位几何非法（波束宽度或孔径无效）。 |
| `rir.validation.antenna_el_geometry_invalid` | 输入校验 | 天线俯仰几何非法。 |
| `rir.validation.antenna_beamwidth_aperture_inconsistent` | 输入校验 | 标称波束宽度与孔径 λ/L 推导值交叉矛盾（相对偏差 > 50%）。
| `rir.validation.antenna_gain_beamwidth_inconsistent` | 输入校验 | 天线峰值增益与名义波束宽度物理不相容（G≈26000/(az°·el°)，容差 3 dB）。 |
| `rir.validation.association_policy_invalid` | 输入校验 | 关联策略非法（波门 sigma 非正）。 |
| `rir.validation.detection_policy_invalid` | 输入校验 | 检测策略非法（Pfa/门限/脉冲数/种子）。 |
| `rir.validation.duplicate_external_target_id` | 输入校验 | 场景中外部目标 ID 重复。 |
| `rir.validation.frequency_plan_invalid` | 输入校验 | 频率计划非法（须含有限正值且包含初始载频）。 |
| `rir.validation.inconsistent_platform_position` | 输入校验 | 平台位置存在性标志与数据不一致（has=false 但分量非默认值）。 |
| `rir.validation.invalid_cycle_delta_time` | 输入校验 | 周期步长非法（<= 0）。 |
| `rir.validation.invalid_cycle_index` | 输入校验 | 周期序号非法（为 0）。 |
| `rir.validation.invalid_environment_snapshot` | 输入校验 | 环境快照字段非法（天气衰减非有限/负值）。 |
| `rir.validation.invalid_platform_position` | 输入校验 | 平台 ECEF 位置非法（分量非有限或模长为 0——地心非法）。 |
| `rir.validation.invalid_rf_scene_frame` | 输入校验 | RF 场景帧非法或与周期窗口不一致。 |
| `rir.validation.invalid_target_motion_field` | 输入校验 | 目标速度/起伏模型含非有限或非法字段。 |
| `rir.validation.lifecycle_policy_invalid` | 输入校验 | 生命周期策略非法（confirm/lost 阈值）。 |
| `rir.validation.missing_range_and_cartesian_position` | 输入校验 | 目标斜距 <= 0 且无笛卡尔位置（二者须至少一为正）。 |
| `rir.validation.non_finite_cycle_delta_time` | 输入校验 | 周期步长非有限值。 |
| `rir.validation.non_finite_target_field` | 输入校验 | 目标含非有限数值字段（位置/RCS/斜距/真值样本）。 |
| `rir.validation.rcs_physics_invalid` | 输入校验 | RCS 物理参数非法。 |
| `rir.validation.receiver_rf_hardware_invalid` | 输入校验 | 接收机 RF 硬件边界非法。 |
| `rir.validation.recognition_accumulation_invalid` | 输入校验 | 识别累积计数非法（须至少为 1）。 |
| `rir.validation.recognition_database_path_missing` | 输入校验 | 识别数据库路径缺失（启用识别时须非空）。 |
| `rir.validation.recognition_threshold_invalid` | 输入校验 | 识别门限非法（接受分数/最小裕度须在 [0, 1]）。 |
| `rir.validation.recognition_time_range_invalid` | 输入校验 | 识别时间范围非法（保持时间须非负；最大距离/驻留/累积窗口须有限且为正）。 |
| `rir.validation.recognition_weights_invalid` | 输入校验 | 识别特征权重非法（须有限、在 [0, 1] 且总和为 1）。 |
| `rir.validation.scan_strategy_invalid` | 输入校验 | 扫描策略非法（限位须有限有序且在合法域 az∈[-180,180]、el∈[-90,90]；步长系数须为正）。 |
| `rir.validation.scan_wave_axis_samples_truncated` | 输入校验 | 扫描波位单轴采样数超内核上限（4096），波位表会被截断；需加大步长或缩小扇区。
| `rir.validation.signal_processing_gains_invalid` | 输入校验 | 信号处理增益偏置非法（四偏置须有限且在 [0, 40] dB）。 |
| `rir.validation.tracking_policy_invalid` | 输入校验 | 跟踪策略非法（KF 噪声参数非正）。 |
| `rir.validation.transmitter_frequency_invalid` | 输入校验 | 发射机载频非法（须有限且为正）。 |
| `rir.validation.transmitter_operating_envelope_invalid` | 输入校验 | 发射机工作包络非法（功率/占空比/脉冲能量越界）。 |
