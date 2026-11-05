# 雷达（LRDR/RIR）与红外（SBIRS）参数详解：单位 · 坐标系 · 影响的模型

> 本文逐参数给出**含义、单位、所属坐标系/参考面、影响哪些模型链路**，并标注当前取值。
> 取值来源：`ModelSource/CyModel/Equipment/Sensors/OPIR_SENSOR/sbirs.json`、
> `LRDR_SENSOR/remote_identification_radar.json`，以及库内固定常量（见各表"来源"列）。
> 参数值随配置变化，**单位与坐标系不随配置变化**。

---

# 第一部分 坐标系统一约定（先看这张表，后面所有角度都据此理解）

| 代号 | 名称 | 定义 | 用于 |
|---|---|---|---|
| **ECEF** | 地心地固系 | 原点地心，x 轴指向本初子午线与赤道交点，z 轴指向北极；随地球自转 | 位置/速度交换、雷达全部几何、融合滤波状态 |
| **ECI** | 地心惯性系（近似）| 由 ECEF 绕 z 轴旋转 GMST 得到（`r_ECI=R3(θ_GMST)·r_ECEF`），忽略岁差/章动/极移 | 红外全部角度、红外 EKF 状态 |
| **ENU** | 站心东北天 | 以雷达为原点，e 东、n 北、u 天 | 雷达扫描方位、波束指向、方位/俯仰量测 |
| **传感器系** | 本体/仪器系 | 姿态(→Body) → 安装角(Body→Sensor) → 失准角 复合后的仪器坐标 | 红外视场门、扫描栅格、ATP 指向、焦面脱靶量 |
| **ECI 极坐标** | 极角对 (az, el) | `az=atan2(y,x)`（+X 起、向 +Y）、`el=asin(z/\|r\|)`（赤道面起、向上为正）| 红外量测/真值/验收行（标 `(ECI)`）|
| **ENU 方位** | 罗盘方位 | 正北起、顺时针；内部另有 `90° − 方位` 换算 | 雷达方位输出、雷达角度误差 |

**通用单位约定**：长度 m；速度 m/s；角度内部 rad、配置与日志 deg；功率 W（对数 dBW/dB）；增益/损耗 dB；概率无量纲；时间 s；频率 Hz；温度 K/℃（配置里 `temperature_c` 为 ℃，探测器温度为 K）。

---

# 第二部分 雷达（LRDR / RIR）

## 2.1 发射机 `hardware.transmitter`

| 参数 | 当前值 | 单位 | 坐标系/参考 | 影响模型 |
|---|---|---|---|---|
| `peak_power_w` | 3 000 000 | W | 标量 | 雷达方程 `Pr ∝ Pt`；回波功率、SNR、最大探测距离、检测概率 |
| `frequency_hz` | 3.0e9 | Hz | 标量（决定 λ）| 雷达方程 `λ²`；天线有效面积 `Ae=Gλ²/4π`；多普勒/一维像分辨率 |
| `bandwidth_hz` | 30e6 | Hz | 标量 | 脉压增益 `B·τ`；距离分辨率 `c/2B`；测距误差 `σ_R=c/(2B√SNR)`；热噪声 `kTB` |
| `pulse_width_s` | 0.016 | s | 标量 | 脉压增益 `B·τ`；占空比与驻留；发射能量 |
| `prf_hz` | 10 | Hz | 标量 | 脉冲积累数 `N=pulse_count`；最大不模糊距离 `c/2·PRF⁻¹`；MTD 多普勒通道 |
| `transmit_loss_db` | 3.5 | dB | 标量（链路损耗）| 回波功率（扣减）；SNR |

## 2.2 天线 `hardware.antenna`

| 参数 | 当前值 | 单位 | 坐标系/参考 | 影响模型 |
|---|---|---|---|---|
| `main_beam_gain_db` | 52 | dBi | 增益定义于**波束指向**（ENU 内）| 雷达方程 `G²`；回波功率；最大探测距离 |
| `nominal_az_beamwidth_deg` / `nominal_el_beamwidth_deg` | 0.8 / 0.8 | deg | ENU 方位/俯仰切面 | 方向图取样；**测角误差** `σ_θ=0.317·θ_bw/√SNR`；波束驻留与扫描步长 |
| `antenna_length_m` / `antenna_width_m` | 17.5 / 17.5 | m | 天线孔径（物理尺寸）| 有效面积 `Ae≈Gλ²/4π`=125.95 m²；增益与波束宽度自洽性校核 |
| `model_type` | `kGaussianMainLobe` | 枚举 | — | 方向图模型（高斯主瓣）；旁瓣/后瓣电平定义 |
| `max_sidelobe_level_db` | −20 | dB（相对主瓣峰值）| 主瓣参考 | 离轴增益；干扰/杂波从旁瓣进入的功率 |
| `backlobe_level_db` | −35 | dB（相对主瓣峰值）| 主瓣参考 | 后向干扰/地物回波 |
| `scan_loss_coeff_db_per_deg2` / `max_scan_loss_db` | 0 / 6 | dB/deg² / dB | 相对**扫描中心**的离轴角 | 扫描损耗；大角度指向时增益下调 |
| `boresight_offset_deg{az,el}` | 0 / 0 | deg | ENU | 光轴安装偏差；指向误差与量测系统偏差 |

## 2.3 接收机 `hardware.receiver`

| 参数 | 当前值 | 单位 | 坐标系/参考 | 影响模型 |
|---|---|---|---|---|
| `noise_figure_db` | 4.0 | dB | 标量（接收链路）| 热噪声功率 `N=kT₀BF·NF`；SNR；检测概率 |
| `receive_loss_db` | 2.0 | dB | 标量（接收链路）| 回波功率扣减；SNR |
| `cross_polarization_isolation_db` | 30 | dB | 极化基（H/V）| 极化特征信噪比；交叉极化串扰下限 |
| `maximum_linear_input_power_w` | 0.001 | W | 标量（接收前端）| 饱和限幅；近距离强回波/干扰时的压缩失真 |
| `scene_polarization` | `kHorizontal` | 枚举 | 极化基 | 回波极化分量选取；极化特征解算起点 |

## 2.4 信号处理 `hardware.signal_processing`

| 参数 | 当前值 | 单位 | 坐标系/参考 | 影响模型 |
|---|---|---|---|---|
| `target_processing_gain_db` | 0 | dB | 标量 | 目标支路处理增益（叠加到 SINR 分子）|
| `noise_processing_gain_db` | 1.5 | dB | 标量 | 噪声支路增益（叠加到分母）；净处理改善 = 目标 − 噪声 |
| `clutter_suppression_gain_db` | 0 | dB | 标量 | 杂波支路抑制；当前为 0 ⇒ 第 34 项整条省略 |
| `jamming_suppression_gain_db` | 8.0 | dB | 标量 | 干扰支路抑制；第 35 项总干扰抑制增益 |
| `enable_mti_mtd_effects` | true | 布尔 | — | 是否启用 MTI/MTD 增益与多普勒通道选择 |

## 2.5 RCS 物理模型 `hardware.rcs_physics`

| 参数 | 当前值 | 单位 | 坐标系/参考 | 影响模型 |
|---|---|---|---|---|
| `enable_physical_rcs` | false | 布尔 | — | 关闭时 `rcs_m2` = 场景标称 RCS；开启时按视角/几何解算 |
| `physics_mix_ratio` | 0.25 | 无量纲 [0,1] | — | 物理 RCS 与标称 RCS 的混合权重 |
| `cylinder_weight` | 0.5 | 无量纲 | 目标本体系（圆柱近似）| 姿态相关 RCS 起伏形状 |
| `min/max_equivalent_radius_m` | 0.05 / 5.0 | m | 目标本体系 | 物理 RCS 几何范围 |
| `min/max_rcs_m2` | 0.01 / 1000 | m² | 标量 | RCS 钳位；影响回波功率与识别特征 |
| `bistatic_psi_offset_deg` | 5 | deg | 相对视线夹角 | 双站/多站 RCS 修正 |

## 2.6 指向范围 `orientation`

| 参数 | 当前值 | 单位 | 坐标系/参考 | 影响模型 |
|---|---|---|---|---|
| `az_min_deg` / `az_max_deg` | −110 / +130 | deg | **ENU 罗盘方位**（正北起、顺时针）| 扫描/指向可达范围；越界即拒绝或钳位 |
| `el_min_deg` / `el_max_deg` | 0 / 85 | deg | ENU 俯仰（地平起）| 同上；低仰角杂波与遮挡 |

## 2.7 任务与扫描 `mission`

| 参数 | 当前值 | 单位 | 坐标系/参考 | 影响模型 |
|---|---|---|---|---|
| `work_mode` | `kIdentify` | 枚举 | — | 搜索/识别/跟踪调度策略 |
| `scan_center_deg{az,el}` | 147.5 / 0 | deg | ENU | 扫描窗中心；扫描损耗参考点 |
| `max_range_m` | 9e6 | m | 斜距 | 距离门；`Pr ∝ R⁻⁴` 的可用域 |
| `recognition_dwell_sec` | 0.1 | s | 驻留时间 | 一维像/极化特征的积累时长与帧数 |
| `scan_start_position` / `scan_sequence` | `kLeftTop` / `kAzimuthFirst` | 枚举 | ENU 栅格 | 扫描顺序与覆盖 |
| `step_scale` | 1.0 | 无量纲 | 相对波束宽度 | 步进 = 波束宽度 × step_scale |
| `scan_window_deg{az_min/az_max,el_min/el_max}` | ±17.5, 0–55 | deg | ENU | 扫描窗（波束驻留采样范围）|

## 2.8 策略 `policy`

| 参数 | 当前值 | 单位 | 坐标系/参考 | 影响模型 |
|---|---|---|---|---|
| `detection.cfar_pfa` | 1e-6 | 无量纲概率 | — | CFAR 门限 `T=Q⁻¹(N,Pfa)`；检测概率曲线；第 36 项门限 |
| `detection.min_snr_db` | −10 | dB | 标量（SNR）| 最小可检门限，低于则 Pd=0 |
| `detection.min_detection_margin_db` | −2 | dB | 标量（SNR）| 检出后的余量门（抑制边缘虚警）|
| `detection.pulse_count` | 20 | 个 | 积累脉冲数 | `T=Q⁻¹(N,Pfa)`；脉压/积累增益；PD 曲线 |
| `detection.random_seed` | 42 | 整数 | — | 检测抽签的可复现性 |
| `detection.gate_mode` | `kSnrFallback` | 枚举 | — | 量测来源：真实量测 / SNR 回退 |
| `association.distance_gate_sigma` | 6.0 | σ（无量纲）| 滤波新息空间（ENU/ECEF 混合）| 关联门宽；误关联与漏关联 |
| `tracking.kalman_noise_diff_coeff` | 30 | m²/s³（CWNA q）| 状态所在系（ENU）| 过程噪声 Q；机动跟随 vs 平滑 |
| `lifecycle.confirm_hits` | 1 | 拍 | 时间轴（拍）| 航迹确认时延 |
| `lifecycle.max_miss_before_lost` | 418 | 拍 | 时间轴 | 丢批判定（保持预测的时长）|
| `lifecycle.max_lost_cycles` | 2090 | 拍 | 时间轴 | 丢失航迹保留时长（重捕窗口）|
| `lifecycle.enable_imm_lifecycle` / `model_count_hint` | true / 2 | 布尔 / 个 | — | IMM 模型切换与数量 |
| `recognition.enabled` | true | 布尔 | — | 识别链开关 |
| `recognition.min_confirmed_hits` | 1 | 拍 | 时间轴 | 参与识别的确认门槛 |
| `recognition.accumulation_window_sec` | 60 | s | 时间轴 | 特征积累窗 |
| `recognition.min_observation_count` | 1 | 次 | — | 最少观测数 |
| `recognition.acceptance_score` | 0.6 | 无量纲 [0,1] | — | 识别通过分（综合相似度门限）|
| `recognition.minimum_margin` | 0.05 | 无量纲 | — | 与次优类别的裕度门 |
| `recognition.result_hold_sec` | 120 | s | 时间轴 | 结果保持时长（防抖）|
| `recognition.feature_weights{rcs,motion,polarization,range_profile}` | 0.25 ×4 | 无量纲（和为 1）| — | 综合识别加权 |
| `recognition.database_path` | `target_feature_database_v1.1.db` | 路径 | — | 特征库版本 |

## 2.9 环境 `environment`

| 参数 | 当前值 | 单位 | 坐标系/参考 | 影响模型 |
|---|---|---|---|---|
| `weather_attenuation_db` | 0 | dB（单程）| 传播路径 | 回波功率扣减；SNR |
| `vegetation_cover_profile` | `kOpenGrassland` | 枚举 | 地表面 | 面杂波 σ⁰；杂波功率/CNR（决定第 34 项是否有输出）|
| `atmospheric_physics{enable, pressure_hpa, temperature_k, relative_humidity}` | false, 1013.25, 288.15, 0.5 | 布尔 / hPa / K / [0,1] | 大气剖面 | 物理大气衰减（开启时）|

---

# 第三部分 红外（SBIRS）

## 3.1 硬件 `hardware`

| 参数 | 当前值 | 单位 | 坐标系/参考 | 影响模型 |
|---|---|---|---|---|
| `wavelength_lower_um` / `wavelength_upper_um` | 3.0 / 5.0 | µm | 波长带 | 辐射亮度积分带；衍射极限 `λ/D`；背景辐射 |
| `optical_aperture_m` | 0.5 | m | 入瞳直径（传感器系）| 集光面积 `A=πD²/4`；衍射极限；接收功率 |
| `optical_transmission` | 0.8 | 无量纲 [0,1] | 光学链路 | 入瞳→焦面能量；SNR |
| `detector_quantum_efficiency` | 0.7 | 无量纲 | 探测器 | 光子→电子转换效率；信号能量/SNR |
| `integration_time_sec` | 0.1 | s | 时间轴（积分窗）| 信号能量 `E=P·t`；驻留匹配；SNR |
| `noise_equivalent_power_w` | 1e-14 | W | 入瞳等效 | 噪声底；SNR |
| `background_radiance_w_sr_m2` | 1.2 | W/(sr·m²) | 视场背景（传感器系）| 背景光子噪声；SNR 分母 |
| `detector_temperature_k` | 80 | K | 探测器 | 暗电流/热噪声（随温度）|
| `readout_noise_rms_w` | 2e-15 | W（RMS）| 读出电路 | 噪声 RSS 项；SNR |
| `focal_length_m` | 2.0 | m | 传感器系 | IFOV = 像元间距/焦距；焦面脱靶量 `f·tanΔθ` |
| `detector_pixel_pitch_m` | 30e-6 | m | 传感器系焦平面 | **IFOV=15 µrad**；质心噪声 `IFOV/√SNR`；脱靶量像素换算 |

## 3.2 任务/视场 `mission`

| 参数 | 当前值 | 单位 | 坐标系/参考 | 影响模型 |
|---|---|---|---|---|
| `work_mode` | `kSearchAndStare` | 枚举 | — | 输出哪些波束日志（联合/宽扫/大幅面，见 §3.6）|
| `wide_field_fov_az_deg` / `_el_deg` | 24 / 24 | deg | **传感器系**矩形视场 | 宽场四门（视场判定）、覆盖区/驻留、`d_max` 的扫描窗 |
| `narrow_field_fov_az_deg` / `_el_deg` | 2.8 / 2.8 | deg | 传感器系 | 窄场捕获/跟踪的视场门、ATP 指向容差 |
| `scan_start_az_deg` | 180 | deg | 取决于 `scan_azimuth_reference`（ECI 绝对 / 相对星下点）| 扫描相位起点 |
| `scan_span_deg` | 360 | deg | 同上 | 扫描跨度（会按地球可见窗收敛）|
| `initial_scan_phase_deg` | 0 | deg | 相对起点 | 首拍相位 |
| `scan_direction` | `kIncreasingAzimuth` | 枚举 | 方位轴 | 扫描方向（往复）|
| `scan_azimuth_reference` | `kNadirRelative` | 枚举 | **天底参考**：星下点方位 `atan2(−y,−x)` | 扫描起点语义（预警扇区对准）|
| `scan_center_el_deg` | 0 | deg | **ECI 赤道面**（单行模式行中心）| 俯仰指向中心；GEO 下 = 天底 |
| `scan_el_start_deg` / `_span_deg` / `_step_deg` | −8 / 24 / 8 | deg | ECI 赤道面 | 2-D 俯仰栅格；行数与无隙覆盖预算（step ≤ FOV_el）|
| `scan_rate_deg_per_sec` | 36 | deg/s | 方位轴 | 扫描速率；**驻留 = FOV_az/rate**；与积分时间匹配 |
| `min_range_m` / `max_range_m` | 1e3 / 5e7 | m | 斜距 | 距离门（近/远限）；`d_max` 上限 |
| `frame_rate_hz` | 10 | Hz | 时间轴 | 帧计数；驻留帧数 = 驻留×帧率；滤波步长仍是周期 dt |
| `narrow_cue_latency_s` | 0 | s | 时间轴 | 宽→窄引导时延（外推目标位置）|
| `narrow_pointing_settle_error_deg` | 0.0002 | deg | 传感器系 | 稳定后残余指向误差（脱靶量偏差）|
| `narrow_pointing_max_slew_rate_deg_per_sec` | 60 | deg/s | 传感器系 | 转镜可达时间（决定能否在本拍捕获）|
| `narrow_pointing_settle_tolerance_deg` | 0.01（运行期 0.02）| deg | 传感器系 | 稳定判据容差 |

## 3.3 大幅面预警扫描 `mission.wide_area_scan`（2026-11-05 新增，独立参数块）

| 参数 | 当前值 | 单位 | 坐标系/参考 | 影响模型 |
|---|---|---|---|---|
| `enabled` | true | 布尔 | — | 是否启用第三模式；**优先级高于 work_mode** |
| `fov_az_deg` / `fov_el_deg` | 8 / 8 | deg | 传感器系矩形视场 | 该模式视场门、覆盖区、驻留 |
| `scan_rate_deg_per_sec` | 18 | deg/s | 方位轴 | 驻留 = 8/18 ≈ 0.44 s |
| `scan_start_az_deg` / `scan_span_deg` | 180 / 360 | deg | 取决于基准 | 扫描相位 |
| `scan_el_start_deg` / `_span_deg` / `_step_deg` | −8 / 24 / 4 | deg | ECI 赤道面 | 6 行栅格（无隙：step ≤ 8）|
| `min_snr_linear` | 4.5 | 线性 SNR | 标量 | 该模式检测门限（宽 4.0 / 窄 5.0 之间）|
| `sigma_at_35db_urad` / `sigma_at_10db_urad` | 1.6 / 3.2 | µrad | 视线角（ECI 极坐标的 az/el 合成）| **该模式测角 1σ**（作用域覆盖线性锚点）；精度落在宽窄之间 |
| `scan_azimuth_reference` | `kNadirRelative` | 枚举 | 天底参考 | 起点语义 |

## 3.4 策略 `policy`

| 参数 | 当前值 | 单位 | 坐标系/参考 | 影响模型 |
|---|---|---|---|---|
| `detection.wide_min_snr_linear` | 4.0 | 线性 SNR | 标量 | 宽场 SNR 门（第 12 项、`d_max`）|
| `detection.narrow_min_snr_linear` | 5.0 | 线性 SNR | 标量 | 窄场捕获/跟踪 SNR 门（第 13 项）|
| `error_model.range_fraction_sigma` | 0.001 | 无量纲（相对距离）| 斜距 | 距离误差（被动红外无真实测距，仅误差模型）|
| `error_model.random_seed` | 1 | 整数 | — | 误差抽样可复现 |
| `error_model.orbit_sigma_deg` | 0 | deg | 平台轨道确定误差（折算角）| 仪器地板 RSS 项；第 8 项误差分量 |
| `error_model.attitude_sigma_deg` | 0.0001 | deg（0.36″）| 姿态（本体→ECI）| 仪器地板 RSS；指向误差 |
| `error_model.fov_sigma_deg` | 5e-5 | deg（0.18″）| 传感器系视场标定 | 仪器地板 RSS |
| `error_model.detector_bandwidth_hz` | 100 | Hz | 时间轴 | 动态滞后项 `σ_lag=ω/(2πB)`；R 矩阵 |
| `error_model.nav_position_sigma_m` | **2.0** | m | ECEF 径向 | 第 8 项 `|定位误差|`（1σ，最大值约 9 m）；速度误差取其 1% |
| `pointing_disturbance.common_attitude_sigma_deg` | 0.0005 | deg（1.8″）| 共模姿态（→ECI）| 指向扰动；RSS 合成总指向误差；角误差 |
| `…common_attitude_correlation_time_s` | 1.0 | s | 时间轴 | ORN 过程相关时间（扰动平滑度）|
| `pointing_disturbance.channel_pointing_sigma_deg` | 0.0003 | deg（1.08″）| 通道指向（传感器系）| 通道间偏差；指向预算 |
| `…channel_pointing_correlation_time_s` | 1.0 | s | 时间轴 | 同上 |
| `pointing_disturbance.channel_vibration_amplitude_deg` | 0.0001 | deg | 传感器系（正弦）| 高频抖动；瞬时指向误差 |
| `…channel_vibration_frequency_hz` | 0.3 | Hz | 时间轴 | 抖动频谱 |
| `scheduler.wide_to_narrow_required_consecutive_hits` | 1 | 拍 | 时间轴 | 宽→窄交接的连续命中门 |
| `tracking.tracking_mode` | `kEstimated` | 枚举 | — | 估计跟踪 vs 真值辅助跟踪 |
| `tracking.estimated_backend` | `kEkf` | 枚举 | — | EKF / IMM / 角度 CV-KF |
| `tracking.process_noise_diff_coeff` | 20 | m²/s³（CWNA q）| **ECI**（状态系）| 过程噪声 Q；跟踪机动跟随 |
| `tracking.initial_position_std_m` | 1000 | m | ECI | 初值 P；收敛快慢 |
| `tracking.initial_velocity_std_m_per_s` | 100 | m/s | ECI | 同上 |
| `tracking.nis_gate_loss_cycles` | 0 | 拍 | 时间轴 | NIS 连续超门释放（0=不释放）|
| `tracking.nfov_tracking_gate_loss_cycles` | 2 | 拍 | 时间轴 | 窄场几何/SNR 门连续失败释放锁定 |

## 3.5 环境 `environment`

| 参数 | 当前值 | 单位 | 坐标系/参考 | 影响模型 |
|---|---|---|---|---|
| `weather_type` / `sea_state` | `kClear` / `kLow` | 枚举 | 地气背景 | 背景辐亮度；杂散辐射 |
| `temperature_c` | 15 | ℃ | 大气 | 透过率/背景（经环境模型）|
| `relative_humidity_percent` | 50 | % | 大气 | 水汽吸收；透过率 |
| `visibility_km` | 20 | km | 水平能见度 | 气溶胶/雾衰减；透过率 |
| `base_atmospheric_transmittance` | 0.8 | 无量纲 [0,1] | 视线路径 | 入瞳辐照度 `E=I·τ/R²`；SNR |
| `humidity_visibility_interaction_weight` / `rain_humidity_interaction_weight` | 0 / 0 | 无量纲 | — | 环境耦合项（当前关闭）|

## 3.6 各参数在"三种模式"下的可用性

| 参数组 | 联合模式 | 宽扫模式 | 大幅面模式 |
|---|---|---|---|
| `wide_field_fov_*`、`scan_*`、`scan_el_*` | ✅ 生效 | ✅ 生效 | ❌ 被 `wide_area_scan.*` 覆盖 |
| `wide_area_scan.fov/rate/el_*` | ❌ 不生效 | ❌ 不生效 | ✅ 生效 |
| `detection.wide_min_snr_linear` | ✅ | ✅ | ❌ 被 `min_snr_linear` 覆盖 |
| 线性 σ 锚点（35 dB/10 dB）| 库默认 1.0/5.0 | 库默认 | **覆盖为 1.6/3.2** |
| 波束日志（第 12/13/14/11 项）| 12+13+14 | 仅 12 | 仅 11 |

---

# 第四部分 参数 → 模型链路 总览

## 4.1 雷达链路

```text
Pt, G, λ, σ, R, L  →  回波功率 Pr            （参数：peak_power_w、main_beam_gain_db、frequency_hz、RCS、损耗）
NF, B, T0          →  热噪声 N               （noise_figure_db、bandwidth_hz）
σ0, 擦地角, 分辨率 →  杂波 C                 （vegetation_cover_profile）
G_j, 抑制比        →  干扰 J                 （jamming_suppression_gain_db、旁瓣电平）
B·τ, MTI/MTD       →  处理增益               （bandwidth_hz、pulse_width_s、enable_mti_mtd_effects）
Pr/N/C/J           →  SINR                  （以上全部）
Pfa, N, 余量门      →  Pd → 抽签 → 检测       （cfar_pfa、pulse_count、min_detection_margin_db）
θ_bw, SNR          →  测角误差 σ_θ；B, SNR → 测距误差 σ_R
6σ 门 + LAPJV      →  关联                   （distance_gate_sigma）
q=30, R            →  KF/IMM 状态与协方差     （kalman_noise_diff_coeff）
特征权重 0.25×4    →  综合识别得分与类别      （feature_weights、acceptance_score、minimum_margin）
```

## 4.2 红外链路

```text
I_target, τ_atm, R  →  入瞳辐照度 E = I·τ/R²  （wavelength_*、base_atmospheric_transmittance）
D, τ_opt, η, t_int  →  信号能量 / 接收功率    （optical_aperture_m、optical_transmission、QE、integration_time_sec）
NEP, 背景, 读出     →  噪声                   （noise_equivalent_power_w、background_radiance、readout_noise）
信号/噪声           →  线性 SNR               （以上全部）
SNR 门（4.0/5.0/4.5）→  检测（宽场四门/窄场帧门/大幅面门）
五类误差 RSS 或 SNR 线性表 → 测角 σ（az/el，ECI 极坐标）   （error_model.*、pointing_disturbance.*、wide_area.sigma_*）
FOV/step/rate       →  扫描栅格与驻留          （mission.scan_* / wide_area.*）
EKF(q=20, R)        →  目标状态与协方差（ECI） （tracking.*）
2 星视线交会        →  位置/落点/发射点推演
```

---

# 第五部分 单位换算与易错点

| 易错点 | 正确做法 |
|---|---|
| 角度 deg ↔ rad | 库内常量 `π/180`；日志 `(ECI)` 行是 **rad**、`(传感器系)` 行是 **deg** |
| 角度 ↔ 弧秒/微弧 | `1″=4.848 µrad`；`1 µrad=0.2063″`；判据 360″=1745 µrad、10″=48.5 µrad、3 µrad=0.619″ |
| dB ↔ 线性 | SNR 门限配置为**线性**（4.0/5.0/4.5），日志写 dB；`SNR_dB=10log10(SNR_lin)` |
| 功率/能量 | `P[W]`、`E=P·t_int[J]`；`NEP` 与 `readout_noise` 均为 W |
| 概率 | `cfar_pfa`、`acceptance_score`、`minimum_margin`、`optical_transmission` 都是 [0,1] 无量纲 |
| 扫描速率→驻留 | `dwell = FOV_az / rate`；宽 24/36=0.67 s，大幅面 8/18=0.44 s；要求 ≥ 积分时间 0.1 s |
| 栅格无隙 | `scan_el_step_deg ≤ fov_el_deg`（宽 8≤24 ✅；大幅面 4≤8 ✅）|
| 雷达方位 | ENU 罗盘方位（正北起、顺时针），内部有 `90°−az` 换算；**不要**与红外 ECI 极坐标混用 |
| 融合配对窗 | 融合 `pair_window_cycles=1`（1 拍），精度评估 `dual_sat_pair_window_cycles=0`（严格同拍）|
