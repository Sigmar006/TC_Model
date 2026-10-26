# 外部集成"发现不了目标"排查清单（2026-10-26）

> 面向把 1q 模型库接到外部仿真引擎的场景。按**从症状到根因**的次序排列：先分诊（§1），
> 再按层往下钻（§2 传感器 / §3 投递与融合 / §4 外部引擎特有坑 / §5 雷达），最后附最小自检包（§6）。
> 文中字段名、门限值、日志标签均取自当前代码与宿主实跑日志，可直接 grep。

## 1. 三分钟分诊：先看三个数

| 检查 | 在哪看 | 读到什么 ⇒ 结论 |
|---|---|---|
| ① 模型有没有被调用 | 传感器诊断日志 `[detect-entry] call=…` / `[cycle] entered` | 没有 ⇒ 模型未挂载/未启用/周期没驱动 |
| ② 帧到没到融合 | 融合诊断 `[receive] accepted frame total=…`、`[cycle] fusion-in idx=… targets=…` | `total` 不涨 ⇒ 投递链断（§3）；`targets=0` ⇒ 帧到了但被阶段/归属门全丢 |
| ③ 传感器有没有检测 | `SatelliteTarget.log` 里 `目标可探测性…红外可探测=是` | 一条都没有 ⇒ 传感器检测门没过（§2） |

> 三种"发现不了"要分开治：**A 传感器零检测**（§2）、**B 检测了但没送到融合**（§3）、
> **C 有量但位置不对**（§4 坐标系/时间基准）。别在 B 里查 §2 的门限，会白耗时间。

## 2. 传感器层（A：零检测）

按命中率从高到低：

1. **使能位**：`sensor_enabled` 是否为 true；模型有没有被平台 `EMPLOY SENSOR` 挂上。
2. **平台输入非法被直接跳过**：胶水层对平台位置做 `!finite || 全零` 检查后 `return`。
   外部引擎常见坑是**首周期位置为 0**（还没 Update 就先调模型）⇒ 看 `[cycle] invalid site/position`。
3. **场景目标装配**：目标必须 ① `active` ② 不是本星 ③ `target_id≠0` ④ 辐射强度 >0。
   我们踩过：场景从未给 `irar_` 赋值 → 辐射恒为回退常量（不致命，但若回退值配成 0 就永远探测不到）。
4. **几何门**：目标是否在本星视场内。
   - 宽场 FOV：`wide_field_fov_az_deg/el`（当前 8°/8°）；
   - **扫描窗口口径**：`scan_window.az` 是相对 `scan_center` 的偏移、`el` 是绝对；SBIRS 侧
     `scan_start_az_deg` 是**起点**、`scan_span_deg` 是**相对跨度的**（不是上下限），
     方位基准由 `scan_azimuth_reference` 二选一（`kEciAbsolute` 绝对方位 / `kNadirRelative` 相对星下点）。
     口径搬错 = 视场整体平移，表现为"目标明明在天上却一直 miss"。
   - 俯仰栅格：`scan_el_start_deg/span/step`（当前 −8°/24°/8°，4 行）；`span=0` 时才用 `scan_center_el_deg`。
5. **距离门**：`min_range_m` / `max_range_m`（当前 1 km / 5e7 m）。
6. **SNR 门**：`policy.detection.wide_min_snr_linear` / `narrow_min_snr_linear`（**线性倍数，不是 dB**）。
   注意本项目历史坑：早先把门限写成 0.001，导致 `Pfa≈0.5`、探测概率恒 0.5、检测恒过门。
7. **辐照/噪声链**：`noise_equivalent_power_w`、`readout_noise_rms_w`、
   **`background_radiance_w_sr_m2` 必须 >0**（为 0 时光子噪声分支整段跳过，噪声结构失真）、
   `detector_quantum_efficiency`、`optical_transmission`、`optical_aperture_m`、`focal_length_m`、`detector_pixel_pitch_m`。
   `integration_time_sec` 要与驻留自洽：驻留 ≈ `FOV_az/scan_rate`（当前 8/78=0.103 s，积分 0.10 s）。
8. **姿态/安装**：姿态是 `Psi/Theta/Phi`（yaw/pitch/roll）**度**；安装角与失配角的默认集**非零**
   （`mount=(1.0,0.5,−0.5)°`、`bias=(0.10,−0.05,0.02)°`），别按"恒等"去推几何。
9. **误差模型全 0 的陷阱**：三项角误差配 0 ⇒ 量测协方差落到下限 ⇒ 下游 EKF 的 NIS 门拒掉大部分量测，
   表现为"检测有、航迹停更"。这是历史事故，务必给 `error_model.*` 与 `pointing_disturbance.*` 配物理值。
10. **编译期宏**：验收日志由 `ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG`（SBIRS）/`…_RIR_…`（雷达）门控，
   非 acceptance 预设整段不编译 ⇒ **"日志里没有字段"先确认构建预设，再怀疑代码**。

## 3. 投递与融合层（B：检测到了但融合没量）

1. **融合 DP 没定位到**：看 `[fusion-resolve] found mounted …` / `auto-mounted …` / `FAILED`。
   三级定位顺序：配置单元 → 全单元扫描 → 在"融合枢纽"单元上克隆挂载；都没命中就永远收不到帧。
2. **MAL 载荷**：帧通过 `Query` 的**指针载荷**投递（不拷贝）⇒ 帧对象必须活到 `Query` 返回之后；
   栈对象出作用域后再被读就是脏数据。
3. **帧结构版本**：`StructSize` 字段用于接收侧校验；外部引擎自己拼帧时**必须 memset 全零再填**，
   否则 `HasTruthPosition/TruthEci*` 之类的脏位会把目标当"无归属"或"无真值"。
4. **阶段门（最常见）**：融合只收 `ObservationStage ≥ narrow_field_stage_min`（缺省 **1**）。
   生产者若只发 stage=0（宽场搜索）⇒ **全部被丢**，日志表现为 `frames` 涨而 `detect=0 / anchor_ok=0`。
5. **归属 ID**：`TargetID = 0` 的探测**不参与交会**（只累计丢弃计数 `drop_no_id`）。
6. **时间基准**：帧里的 `UtcJulianDay` 必须与喂给传感器的 `utc_julian_day` 同值，否则 DP 的
   ECI→ECEF 旋转与本星测角不同刻，交会解会系统性偏。
7. **双星可用性**：`dual_targets=0` ⇒ 没有任何目标被 ≥2 颗星同时看到（几何/时窗问题）；
   `dual_targets>0` 而 `dual_fix=0` ⇒ 配对后被残差门拒（`residual_gate_m`，缺省 2500 m）。
8. **配对窗**：`pair_window_cycles`（缺省 1 周期）、`strict_pair_time_delta_sec`（1.5 s）；
   外部引擎周期率不同（10 Hz vs 1 Hz）会让"1 个周期"的物理含义差 10 倍。
9. **键的口径**：跨组件一律用 **TrackKey**（上游带身份时等于场景目标 ID），
   **不要用 `UnitID`**——那是 DP 宿主单元的实体 ID（历史坑：误用会把所有航迹并成一条，
   并把引导指向地面站）。

## 4. 外部引擎特有坑（C：有量但不对）

| 坑 | 表现 | 检查 |
|---|---|---|
| 周期率/`dt` 不一致 | 首周期 `dt` 回退 1 s；扫描驻留、滤波 dt 全错 | 打印每周期 `dt_sec`；确认 `track_cycle_period_sec` 与真实调用周期一致 |
| 调用顺序 | 先 Query DP 再 Update 场景 ⇒ 帧用的是上一拍状态 | 固定"先 Update 后 Query"，并核对帧里 `CycleIndex` |
| 单位混用 | deg/rad、m/km、W/sr 与 W/(m²·sr) | 逐字段核对表；SNR 用**线性**，日志里的 dB 只是显示 |
| 参数文件路径 | 模型读不到 config ⇒ 走缺省（多为零/占位） | 环境变量/相对 cwd 打印实际路径；**改配置必须重启进程**（库只在路径变化时重载） |
| 三件套版本不一致 | 1q.lib / include 头 / DLL 三者不同版本 ⇒ 未定义行为或静默错值 | 核对 `1q.lib` 大小与时间、头目录、DLL 时间戳 |
| 跨线程写日志不可见 | 库内线程写的验收行宿主日志里没有 | 只从引擎线程写（我们踩过：库算行、宿主在引擎线程落盘） |
| 日志目录权限 | 写不进去且无报错 | 用探针文件先验证可写；acceptance 目录要显式给 |

## 5. 雷达（LRDR/RIR）侧额外检查

1. **检测门模式**：`policy.detection.gate_mode = kSnrFallback` 时门是**硬编码 6 dB**
   （`kSnrFallbackGateDb`），**不是**配置里的 `min_snr_db`；两者别混。
2. **几何**：`orientation.*` 是相对阵面的可扫体积、`scan_window_deg` 的 az 相对 `scan_center`、
   el 绝对；`scan_center.el` **不参与几何**（只做回显），别指望改它改变指向。
3. **Cue 语义**：外部引导走 `SubmitExternalTrack`（照射中心），**不是**改 `scan_center`；
   只改 `scan_center` 时目标只在波位游标转过去那一下被照到（0/25 周期检出且静默失败）。
4. **干扰压制**：`jamming_suppression_gain_db` 与干扰功率若差 6 个量级，SINR 被干扰主导，
   雷达"有回波但检不出"，属配置而非算法。
5. **驻留预算**：外部引导到位时全部波位给外部指向（`external-first`），
   若 `recognition_dwell_sec` 太少或周期太短，会"看着目标却没驻留"。

## 6. 最小自检包（建议外部集成第一步就跑）

1. **固定几何探针**：把一个目标放在卫星星下点、恒定辐射、无扫描（`scan_span` 设 L 值或
   `work_mode=kStandby` + 手动指向），确认能出 1 条检测；
2. **门限归零**：`wide/narrow_min_snr_linear = 0`、`min/max_range_m` 放宽，看是否出检测
   ⇒ 能出说明几何/装配没问题，问题在门限；
3. **装配计数**：在 glue 里按周期打印 `targets_in / fov_in / snr_pass / detected` 四个数
   （一次打印足够定位是哪一道门）；
4. **单帧回放**：直接调 `StepWithResult` 打印 `output_frame.detections.size()`，
   绕开 DP/MAL，先把传感器与融合解耦；
5. **确定性**：固定 `utc_julian_day`、固定周期序列、关随机（`random_seed`），保证问题可复现。

## 7. 症状 → 最可能原因速查

| 症状 | 先查 |
|---|---|
| 一条 `目标可探测性` 都没有 | 使能位、平台输入全零、辐射强度为 0、FOV/扫描窗口口径 |
| 有 `可探测性` 但无 `检测标志=是` | SNR 门（线性 vs dB）、积分/驻留、NEP/背景辐射 |
| `frames` 涨但 `detect=0` | 阶段门（stage≥1）、`TargetID=0`、帧结构未清零 |
| `dual_targets=0` | 目标从未被 ≥2 星同周期看到：星位几何、扫描窗、时窗 |
| `dual_targets>0` 但 `dual_fix=0` | 残差门 2500 m、配对窗、时间基准不一致 |
| 融合有航迹但引导指向地面站 | 用了 `UnitID` 而非 `TrackKey` |
| 雷达有回波无检测 | `kSnrFallback` 的 6 dB 硬门、干扰压制、`scan_window` 与体积交集为空 |
| 日志缺字段 | 编译期 acceptance 宏、DLL 版本、跨线程写文件 |
