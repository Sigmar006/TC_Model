# 远程识别雷达输入参数说明

本文是 `remote_identification_radar.json` 的伴随注释文件。JSON 不支持标准注释，因此参数用途在本文集中说明。参数键缺省时，加载器保留 `RirSessionConfig` 中对应字段的默认值。

## 硬件参数

### 发射机 `hardware.transmitter`

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `peak_power_w` | W | 雷达峰值发射功率，用于雷达方程和接收回波功率计算。 |
| `frequency_hz` | Hz | 工作载频，用于波长、传播、天线孔径和 RCS 相关计算。 |
| `bandwidth_hz` | Hz | 发射信号带宽，用于距离分辨率等计算，理论距离分辨率约为 `c/(2B)`。 |
| `pulse_width_s` | s | 单脉冲持续时间，影响脉冲能量和回波积累。 |
| `prf_hz` | Hz | 脉冲重复频率，决定单位时间内发射脉冲数量和驻留积累节奏。 |
| `transmit_loss_db` | dB | 发射链路损耗，从有效发射功率中扣除。 |

### 天线 `hardware.antenna`

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `main_beam_gain_db` | dB | 波束中心的天线峰值增益。 |
| `nominal_az_beamwidth_deg` | deg | 名义方位波束宽度；为 0 时可根据工作波长和物理孔径推导。 |
| `nominal_el_beamwidth_deg` | deg | 名义俯仰波束宽度；为 0 时可根据工作波长和物理孔径推导。 |
| `antenna_length_m` | m | 方位向物理孔径，用于孔径波束宽度和 sinc 平方方向图计算。 |
| `antenna_width_m` | m | 俯仰向物理孔径，用于孔径波束宽度和 sinc 平方方向图计算。 |
| `model_type` | 枚举字符串 | 天线方向图类型：`kGaussianMainLobe`、`kCosinePower` 或 `kSincPattern`。 |
| `max_sidelobe_level_db` | dB | 方向图最大旁瓣电平上限。 |
| `backlobe_level_db` | dB | 天线后瓣电平。 |
| `scan_loss_coeff_db_per_deg2` | dB/deg² | 波束偏离安装轴时的扫描损失系数；0 表示不增加扫描损失。 |
| `max_scan_loss_db` | dB | 扫描损失的最大封顶值。 |
| `boresight_offset_deg.az_deg` | deg | 天线方向图相对安装轴的方位偏置。 |
| `boresight_offset_deg.el_deg` | deg | 天线方向图相对安装轴的俯仰偏置。 |

### 接收机 `hardware.receiver`

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `noise_figure_db` | dB | 接收机噪声系数，用于计算接收噪声底。 |
| `receive_loss_db` | dB | 接收链路损耗，从接收信号功率中扣除。 |
| `cross_polarization_isolation_db` | dB | 正交极化隔离度，用于极化失配和通道泄漏计算。 |
| `maximum_linear_input_power_w` | W | 接收机保持线性工作的最大输入功率，用于判断饱和。 |
| `scene_polarization` | 枚举字符串 | 接收极化方式，例如 `kHorizontal`；用于射频场景极化匹配。 |

### 信号处理 `hardware.signal_processing`

这一组参数是效能级信号处理增益或代价，不是原始回波采样参数。它们进入检测链路的有效 SINR 计算。

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `target_processing_gain_db` | dB | 目标信号处理增益；正值提高有效目标 SINR。 |
| `noise_processing_gain_db` | dB | 噪声处理代价；正值抬高等效噪声底。 |
| `clutter_suppression_gain_db` | dB | 杂波抑制改善量；正值降低等效杂波影响。 |
| `jamming_suppression_gain_db` | dB | 干扰抑制改善量；正值降低等效干扰影响。 |

### RCS 物理模型 `hardware.rcs_physics`

这一组参数控制雷达是否根据目标几何和观测条件估计有效 RCS。关闭物理模型时，检测链路使用外部输入的目标 RCS。

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `enable_physical_rcs` | bool | 是否启用物理 RCS 估计。 |
| `physics_mix_ratio` | `[0,1]` | 物理估计 RCS 的混合比例；0 仅用外部 RCS，1 仅用物理估计值。 |
| `cylinder_weight` | `[0,1]` | 组合散射模型中圆柱散射分量的权重。 |
| `min_equivalent_radius_m` | m | 物理估计使用的目标等效半径下限。 |
| `max_equivalent_radius_m` | m | 物理估计使用的目标等效半径上限。 |
| `min_rcs_m2` | m² | 物理估计结果的 RCS 裁剪下限。 |
| `max_rcs_m2` | m² | 物理估计结果的 RCS 裁剪上限。 |
| `bistatic_psi_offset_deg` | deg | 双站角计算使用的角度偏移补偿。 |

## 转向与任务参数

### 转向限位 `orientation`

| 参数名 | 单位 | 用途 |
| --- | --- | --- |
| `az_min_deg` / `az_max_deg` | deg | 阵面或转台允许的最小、最大方位扫描角。 |
| `el_min_deg` / `el_max_deg` | deg | 阵面或转台允许的最小、最大俯仰扫描角。 |

### 任务配置 `mission`

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `work_mode` | 枚举字符串 | 工作模式；`kStby` 为待机，`kIdentify` 为识别工作。 |
| `scan_center_deg.az_deg` | deg | 当前扫描中心的方位角。 |
| `scan_center_deg.el_deg` | deg | 当前扫描中心的俯仰角。 |
| `max_range_m` | m | 识别任务最大作用距离。 |
| `recognition_dwell_sec` | s | 单个识别波位或目标的驻留时间。 |
| `scan_start_position` | 枚举字符串 | 二维波位扫描的起始位置，例如 `kLeftTop`。 |
| `scan_sequence` | 枚举字符串 | 二维扫描推进顺序，例如 `kAzimuthFirst`。 |
| `step_scale` | 倍数 | 波位步长相对波束宽度的比例。 |
| `scan_window_deg.az_min_deg` / `az_max_deg` | deg | 当前任务扫描子窗的方位范围。 |
| `scan_window_deg.el_min_deg` / `el_max_deg` | deg | 当前任务扫描子窗的俯仰范围。 |

## 算法策略参数

### 检测 `policy.detection`

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `cfar_pfa` | 概率 | CFAR 期望虚警概率。 |
| `min_snr_db` | dB | 目标进入检测处理的最低信噪比门限。 |
| `min_detection_margin_db` | dB | 检测统计量相对判决门限所需的最小余量。 |
| `pulse_count` | 个 | 单次检测积累的脉冲数量。 |
| `random_seed` | 整数 | 检测随机过程种子，用于结果复现。 |
| `gate_mode` | 枚举字符串 | 检测门控方式；当前示例 `kSnrFallback` 允许按 SNR 后备判决。 |

### 关联、跟踪与生命周期

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `policy.association.distance_gate_sigma` | σ 倍数 | 量测与航迹关联的距离门限尺度。 |
| `policy.tracking.kalman_noise_diff_coeff` | 模型系数 | 卡尔曼滤波过程噪声扩散系数，控制机动跟随与平滑程度。 |
| `policy.lifecycle.confirm_hits` | 周期数 | 航迹转为确认状态所需命中次数。 |
| `policy.lifecycle.max_miss_before_lost` | 周期数 | 确认航迹转为丢失前允许的连续漏检次数。 |
| `policy.lifecycle.max_lost_cycles` | 周期数 | 丢失航迹在删除前允许保留的周期数。 |
| `policy.lifecycle.enable_imm_lifecycle` | bool | 是否启用 IMM 相关的航迹生命周期处理。 |
| `policy.lifecycle.model_count_hint` | 个 | IMM 运动模型数量提示。 |

### 目标识别 `policy.recognition`

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `enabled` | bool | 是否启用目标特征识别。 |
| `min_confirmed_hits` | 次 | 航迹满足识别处理所需的最少确认命中数。 |
| `accumulation_window_sec` | s | 多周期识别特征积累时间窗。 |
| `min_observation_count` | 个 | 输出识别结论所需的最少有效观测数。 |
| `acceptance_score` | `[0,1]` | 候选类别被接受为识别结果的最低综合得分。 |
| `minimum_margin` | `[0,1]` | 最优候选相对次优候选所需的最小得分差。 |
| `result_hold_sec` | s | 最新识别结论在无新有效观测时的保持时间。 |
| `feature_weights.rcs_weight` | 权重 | RCS 特征在综合识别中的基础权重。 |
| `feature_weights.motion_weight` | 权重 | 运动特征在综合识别中的基础权重。 |
| `feature_weights.polarization_weight` | 权重 | 极化特征在综合识别中的基础权重。 |
| `feature_weights.range_profile_weight` | 权重 | HRRP 宽带一维距离像统计特征的基础权重。 |
| `database_path` | 路径字符串 | 目标特征数据库文件路径。相对路径由集成程序按配置根目录解析。 |

四项特征权重应为非负值，并且总和为 1。运行时还会结合各特征的有效性和质量重新归一化。

## 环境参数 `environment`

| 参数名 | 单位/类型 | 用途 |
| --- | --- | --- |
| `weather_attenuation_db` | dB | 天气引起的附加双程传播损耗。 |
| `vegetation_cover_profile` | 枚举字符串 | 地表植被覆盖档位；`kDisabled` 表示关闭植被散射建模。 |
| `atmospheric_physics.enable_physical_model` | bool | 是否根据气压、温度和湿度启用物理大气模型。 |
| `atmospheric_physics.pressure_hpa` | hPa | 大气压观测值。 |
| `atmospheric_physics.temperature_k` | K | 大气温度观测值。 |
| `atmospheric_physics.relative_humidity` | `[0,1]` | 相对湿度比例。 |

## 会话与日志参数

| 参数名 | 类型 | 用途 |
| --- | --- | --- |
| `sensor_enabled` | bool | 雷达会话初始化时的电源/使能状态。 |
| `acceptance_log_enabled` | bool | 运行时验收日志开关；编译期日志总开关关闭时，本参数不能强制开启日志。 |
| `acceptance_log_output_directory` | 路径字符串 | 雷达验收 `.log` 与 `.csv` 文件的输出目录；空字符串时使用环境变量或编译期默认目录。 |

## 代码对应关系

- 参数结构：`include/1q/remote_identification_radar/config/`
- JSON 加载器：`examples/common/config_loaders/remote_identification_radar/config_loader.h`
- 参数校验：`src/remote_identification_radar/session/RirSessionConfigValidation.cpp`
