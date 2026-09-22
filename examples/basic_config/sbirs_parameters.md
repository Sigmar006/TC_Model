# 天基红外传感器输入参数说明

本文是 `sbirs.json` 的伴随注释文件。JSON 不支持标准注释，因此参数用途在本文集中说明。参数键缺省时，加载器保留 `SbirsSessionConfig` 中对应字段的默认值。

## 硬件参数 `hardware`

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `wavelength_lower_um` | μm | 红外探测工作波段下限。 |
| `wavelength_upper_um` | μm | 红外探测工作波段上限。 |
| `optical_aperture_m` | m | 光学系统有效孔径，影响目标辐射收集能力。 |
| `optical_transmission` | `[0,1]` | 光学系统透过率，用于计算到达探测器的有效辐射功率。 |
| `detector_quantum_efficiency` | `[0,1]` | 探测器量子效率，用于把入射辐射换算为有效响应。 |
| `integration_time_sec` | s | 单帧探测积分时间，影响信号积累和噪声。 |
| `noise_equivalent_power_w` | W | 等效噪声功率 NEP，表示探测器本底噪声能力。 |
| `background_radiance_w_sr_m2` | W/(sr·m²) | 背景辐射亮度，用于背景光子噪声计算。 |
| `detector_temperature_k` | K | 探测器工作温度；大于 0 时计入热噪声。 |
| `readout_noise_rms_w` | W | 探测器读出链路的等效 RMS 噪声。 |
| `focal_length_m` | m | 光学焦距，用于角度偏差到像面脱靶量的换算。 |
| `detector_pixel_pitch_m` | m | 像元间距，用于把像面距离换算为像素偏移。 |

## 任务参数 `mission`

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `work_mode` | 枚举字符串 | 工作模式；当前示例使用搜索与凝视组合模式 `kSearchAndStare`。 |
| `wide_field_fov_az_deg` | deg | 宽视场 WFOV 的方位视场宽度。 |
| `wide_field_fov_el_deg` | deg | 宽视场 WFOV 的俯仰视场宽度。 |
| `narrow_field_fov_az_deg` | deg | 窄视场 NFOV 的方位视场宽度。 |
| `narrow_field_fov_el_deg` | deg | 窄视场 NFOV 的俯仰视场宽度。 |
| `scan_start_az_deg` | deg | WFOV 扫描方位起点。 |
| `scan_span_deg` | deg | 从扫描起点开始的有向扫描跨度。 |
| `scan_direction` | 枚举字符串 | 扫描方位的递增或递减方向。 |
| `scan_azimuth_reference` | 枚举字符串 | 扫描方位角使用的参考系；当前示例为 `kEciAbsolute`。 |
| `scan_center_el_deg` | deg | WFOV 扫描中心俯仰角。 |
| `scan_rate_deg_per_sec` | deg/s | WFOV 方位扫描速率。 |
| `min_range_m` | m | 目标距离门控下限。 |
| `max_range_m` | m | 目标距离门控上限。 |
| `frame_rate_hz` | Hz | 传感器输出帧率。 |
| `narrow_cue_latency_s` | s | WFOV 发现目标到 NFOV 接收引导之间的延迟。 |
| `narrow_pointing_settle_error_deg` | deg | NFOV 指向稳定后的残余角度误差。 |
| `narrow_pointing_max_slew_rate_deg_per_sec` | deg/s | NFOV 光轴最大转动速度。 |
| `narrow_pointing_settle_tolerance_deg` | deg | 判断 NFOV 指向已稳定的角度容差。 |

以下任务参数受代码支持，但当前 `sbirs.json` 未显式填写，因而使用结构体默认值：

| 参数名 | 用途 |
| --- | --- |
| `mission.initial_scan_phase_deg` | 设置相对扫描起点的初始扫描行程偏移。 |
| `mission.scan_el_start_deg` | 设置二维 WFOV 栅格扫描的起始俯仰行中心。 |
| `mission.scan_el_span_deg` | 设置俯仰栅格扫描跨度；0 表示单行扫描。 |
| `mission.scan_el_step_deg` | 设置俯仰栅格行间距。 |

## 算法策略参数 `policy`

### 检测 `policy.detection`

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `wide_min_snr_linear` | 线性比值 | WFOV 目标检测的最低信噪比门限。 |
| `narrow_min_snr_linear` | 线性比值 | NFOV 目标捕获的最低信噪比门限。 |

### 误差模型 `policy.error_model`

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `range_fraction_sigma` | 比例 | 距离乘法误差的 1σ 比例；0 表示不注入该误差。 |
| `random_seed` | 整数 | 误差模型随机种子，用于复现实验。 |
| `orbit_sigma_deg` | deg | 卫星轨道误差对应的角度 1σ。 |
| `attitude_sigma_deg` | deg | 卫星姿态误差的角度 1σ。 |
| `fov_sigma_deg` | deg | 像元或视场畸变误差的角度 1σ。 |
| `detector_bandwidth_hz` | Hz | 探测器动态带宽，用于动态滞后误差计算。 |

代码还支持可选参数 `policy.error_model.nav_position_sigma_m`，用于验收旁路中的卫星导航定位径向误差；当前 JSON 未填写，使用默认值。

### 指向扰动 `policy.pointing_disturbance`

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `common_attitude_sigma_deg` | deg | 多通道共享姿态扰动的稳态 1σ。 |
| `common_attitude_correlation_time_s` | s | 共享姿态扰动的相关时间。 |
| `channel_pointing_sigma_deg` | deg | 各探测通道独立指向扰动的稳态 1σ。 |
| `channel_pointing_correlation_time_s` | s | 通道独立指向扰动的相关时间。 |
| `channel_vibration_amplitude_deg` | deg | 通道确定性振动的角度峰值。 |
| `channel_vibration_frequency_hz` | Hz | 通道确定性振动频率。 |
| `random_seed` | 整数 | 指向扰动随机流基础种子。 |

### 调度与跟踪

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `policy.scheduler.wide_to_narrow_required_consecutive_hits` | 帧数 | 从 WFOV 转入 NFOV 跟踪前所需的连续命中次数。 |
| `policy.tracking.tracking_mode` | 枚举字符串 | 跟踪数据来源和处理模式；当前示例为估计模式 `kEstimated`。 |
| `policy.tracking.estimated_backend` | 枚举字符串 | 估计跟踪后端；当前示例为扩展卡尔曼滤波 `kEkf`。 |
| `policy.tracking.process_noise_diff_coeff` | 模型系数 | 跟踪过程噪声扩散系数，控制机动适应性。 |
| `policy.tracking.initial_position_std_m` | m | 初始航迹位置的 1σ 标准差。 |
| `policy.tracking.initial_velocity_std_m_per_s` | m/s | 初始航迹速度的 1σ 标准差。 |
| `policy.tracking.nis_gate_loss_cycles` | 周期数 | 连续 NIS 超门限后丢锁的周期数；0 表示禁用该丢锁条件。 |
| `policy.tracking.nfov_tracking_gate_loss_cycles` | 周期数 | NFOV 几何或 SNR 门连续失败后丢锁的周期数。 |
| `policy.tracking.imm_model_noise_diff_coeffs` | 数组 | IMM 各运动模型的过程噪声系数；空数组使用库内默认组合。 |

## 环境参数 `environment`

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `weather_type` | 枚举字符串 | 天气类型，用于选择天气衰减条件。 |
| `sea_state` | 枚举字符串 | 海况等级，用于海面背景或环境影响计算。 |
| `temperature_c` | ℃ | 环境温度。 |
| `relative_humidity_percent` | % | 环境相对湿度。 |
| `visibility_km` | km | 大气能见度。 |
| `base_atmospheric_transmittance` | `[0,1]` | 基础大气透过率。 |
| `humidity_visibility_interaction_weight` | 系数 | 湿度与能见度耦合对透过率影响的权重；0 表示关闭。 |
| `rain_humidity_interaction_weight` | 系数 | 雨天湿度耦合影响权重；0 表示关闭。 |

## 会话、安装与日志参数

| 参数名 | 类型 | 用途 |
| --- | --- | --- |
| `sensor_enabled` | bool | 红外传感器会话初始化时的电源/使能状态。 |
| `acceptance_log_enabled` | bool | 可选的运行时验收日志开关；默认 `true`。编译期总开关关闭时不能强制开启日志。 |

`acceptance_log_output_directory` 为可选目录字符串，默认空。非空时优先于日志环境变量，控制会话内的红外验收日志（含接口、融合副本及结束补写记录）；相对路径以进程工作目录为基准。双星联合验收会话应配置相同目录。空值使用环境变量，再回退至编译期默认路径。

例如在完整配置中设置 `"acceptance_log_enabled": true`、`"acceptance_log_output_directory": "D:/acceptance/sbirs"`，使用 `LoadSbirsSessionConfigFromFile()` 读取后传给 `SbirsSession::Create(config)`；每周期调用 `StepWithResult(input)` 即由模型写入验收记录。构建模型库时须开启 `ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG`，融合副本另需 `ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG`。

代码支持可选的顶层 `orientation` 安装指向域。当前 `sbirs.json` 未显式填写，加载时使用 `SbirsOrientationConfig` 默认值；需要配置安装姿态、扫描限位或安装失准时，应按 `include/1q/sbirs_sensor/config/SbirsOrientationConfig.h` 增加对应对象。

## 代码对应关系

- 参数结构：`include/1q/sbirs_sensor/config/`
- JSON 加载器：`examples/common/config_loaders/sbirs_sensor/config_loader.h`
- 参数校验：`src/sbirs_sensor/session/SbirsSessionConfigValidation.cpp`
