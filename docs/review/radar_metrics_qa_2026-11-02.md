# 雷达功率 / 误差 / 门限口径问答汇总（2026-11-02）

> 本文汇总 2026-11-02 当天关于 **回波功率、功率密度、干扰功率、探测距离、RCS 来源、信号处理增益、
> 量测误差、SNR 口径、检测门限、滤波平滑** 的问答，并把每条公式里**每个变量的取值**摊开写清。
>
> 目的：让外部复算者不必再翻代码，也不必在"同一个字段不同口径"上反复踩坑。
>
> 代码基准：1q `fosim_2013` 分支，宿主 BJ_10Y_WorkSpace `master`。

---

## 0. 本机参数总表（复算时直接取这些值）

| 符号 | 含义 | 取值 | 位置 |
|---|---|---|---|
| `f_c` | 载频 | **3.0e9 Hz（3 GHz）** | `remote_identification_radar.json → hardware.transmitter.frequency_hz` |
| `λ` | 波长 = c/f | **0.09993082 m** | `common/numerics/Constants.h → kLightSpeed = 299792458` |
| `c` | 光速 | **299 792 458 m/s** | 同上 |
| `B` | 信号带宽 | **30e6 Hz** | `hardware.transmitter.bandwidth_hz` |
| `τ` | 脉宽 | **0.002 s（2 ms）** | `hardware.transmitter.pulse_width_s` |
| `B·τ` | 时间带宽积（=脉压增益） | **60 000（+47.78 dB）** | `DetectionCellResolver.cpp:275-276` |
| `P_t` | 峰值功率 | **3.0e6 W** | `hardware.transmitter.peak_power_w` |
| `L_tx` | 发射损耗 | **3.5 dB**（已先扣进辐射功率）| `hardware.transmitter.transmit_loss_db` |
| `L_rx` | 接收损耗 | **2.0 dB** | `hardware.receiver.receive_loss_db` |
| `G` | 单向增益（峰值） | **52.0 dBi** | `hardware.antenna.main_beam_gain_db` |
| `θ_bw` | 半功率波束宽 | **0.4° × 0.4°** | `hardware.antenna.nominal_az/el_beamwidth_deg` |
| 旁瓣 / 后瓣 | 相对峰值偏置 | **−20 dB / −35 dB** | `hardware.antenna.max_sidelobe_level_db / backlobe_level_db` |
| `N` | 检测积累脉冲数 | **20**（`policy.detection.pulse_count`）| 运行期有效值另受接收窗截断 |
| `P_fa` | CFAR 虚警概率 | **1e-6** | `policy.detection.cfar_pfa` |
| `gate_mode` | 检测准入门模式 | **`kSnrFallback`** | `policy.detection.gate_mode` |
| `confirm_hits` | 起批命中数 | **1** | `policy.lifecycle.confirm_hits` |
| `q` | 过程噪声扩散系数 | **30** | `policy.tracking.kalman_noise_diff_coeff` |
| 关联波门 | 马氏距离 σ 倍数 | **6** | `policy.association.distance_gate_sigma` |
| `target_processing_gain_db` | 目标信号处理增益 | **0.0 dB** | `hardware.signal_processing` |
| `noise_processing_gain_db` | 噪声代价 | **0.0 dB** | 同上 |
| `clutter_suppression_gain_db` | 杂波抑制 | **0.0 dB** | 同上（无杂波输入）|
| `jamming_suppression_gain_db` | 干扰抑制 | **8.0 dB** | 同上 |
| `enable_mti_mtd_effects` | MTI/MTD 联算 | **true** | 同上 |
| `nav_position_sigma_m` | 卫星导航定位径向 1-σ | **0.1 m（10 cm）** | `sbirs.json → policy.error_model` |

---

## 1. 回波功率：乘了天线增益，而且是双程 G²

### 1.1 公式（`src/common/radar/DetectionCellResolver.cpp:262-270`）

```text
P_echo = P_t,radiated · G_t · G_r · λ² · σ / ( (4π)³ · R⁴ · L )
```

| 变量 | 含义 | 取值 / 说明 |
|---|---|---|
| `P_t,radiated` | **辐射**峰值功率 | `3.0e6 × 10^(−3.5/10) = 1.339e6 W`（`RirEmissionFactory.cpp:79-81` 先扣发射损耗）|
| `G_t`、`G_r` | 发/收单向增益 | **同一个值**（收发共用天线），取**当前波位指向下的离轴增益** |
| `λ` | 波长 | 0.09993082 m |
| `σ` | 目标 RCS | 场景值或物理模型混合值（见第 5 节）|
| `R` | 斜距 | 米 |
| `L` | 接收侧损耗 | `10^((2.0 + 双程附加传播损耗)/10)` |

### 1.2 增益从哪来（关键：不是恒定的 52 dBi）

1. `RirResolveBeamStateForPointing`（`src/remote_identification_radar/dwell/RirBeamControl.h:75-101`）
   调用 `ResolveFrozenBeamState`；
2. `ResolveFrozenBeamState`（`src/common/radar/FrozenBeamResolve.h:46-73`）用
   `EvaluateAntennaPattern(main_beam_gain, pattern, 波束宽, 指向与视线偏角, …).gain_dbi`；
3. 结果经 `RirController.cpp:668` 写入 `cell_config.one_way_antenna_gain_dbi`。

⇒ **G 随指向/扫描变化**：主瓣中心 52 dBi，偏出主瓣按方向图下降。

### 1.3 反证：不计增益差 97.5 dB

实测样例：`斜距=5,848,044.5 m，RCS=1.654 m²，回波功率=3.385e-17 W`

| 假设 | 计算值 | 与实测比 |
|---|---|---|
| 不计增益（0 dBi）| 6.0e-27 W | **−97.5 dB**（不可能）|
| 峰值 52 dBi + 接收 2 dB + 发射 3.5 dB + 传播 4.5 dB | 5.37e-17 W | +2.0 dB ✅ |

---

## 2. 功率密度：`S = P / Ae`，其中 `Ae = G·λ²/(4π)`

### 2.1 Ae 逐步计算（每个变量的值）

```text
Ae = G · λ² / (4π)
λ  = c / f
```

| 步 | 变量 | 代入值 | 结果 |
|---|---|---|---|
| 1 | `f` | 3.0e9 Hz | — |
| 2 | `c` | 299 792 458 m/s | — |
| 3 | `λ = c/f` | 299792458 / 3.0e9 | **0.09993082 m** |
| 4 | `λ²` | 0.09993082² | **9.986169e-3 m²** |
| 5 | `G_dBi` | 52.0 dBi | `G = 10^(52/10)` = **158 489.32** |
| 6 | `4π` | — | **12.56637061** |
| 7 | `λ²/(4π)` | 9.986169e-3 / 12.56637061 | **7.946741e-4** |
| 8 | **`Ae`** | 158489.32 × 7.946741e-4 | **125.9473 m²** |

### 2.2 实测对账（本轮 2,598 行同时含两字段）

| 回波功率 P (W) | 回波功率密度 S (W/m²) | P/S = Ae (m²) |
|---|---|---|
| 3.385e-17 | 2.687e-19 | 125.98 |
| 3.389e-17 | 2.691e-19 | 125.94 |
| 全部 2,598 行 | — | 中位 **125.94**，范围 125.94–126.04 |

`3.166e-19 × 125.9473 = 3.9875e-17 W`，与日志 `3.987e-17 W` 差 **0.012%**（打印位数）。

> ⚠️ 若把 λ 近似成 0.1（而不是精确 `c/f`），Ae 会得 126.12 m²，与实测差 0.14%。
> 复算时请用精确 λ。

---

## 3. 干扰功率与干扰功率密度：与回波**不同**（单程）

### 3.1 功率公式（`src/common/electromagnetics/RfLinkBudget.cpp:303-345`）

```text
P_jam,link = P_j · G_j(指向雷达) · G_r(干扰来向) · λ² / ( (4π)² · R² · L_prop )
             / (接收机侧损耗、极化失配)
```

| 变量 | 含义 | 说明 |
|---|---|---|
| `P_j` | 干扰源发射功率 | 来自场景干扰机 |
| `G_j` | 干扰机天线增益（指向雷达方向）| `TryEvaluateGain` 求 |
| `G_r` | **雷达**接收增益（干扰来向）| 见 3.3：对同址干扰机即主瓣增益 |
| `R` | 单程距离 | **R²**（不是 R⁴）|
| `L_prop` | 传播损耗 | 不含接收机内部损耗 |

### 3.2 检测等效干扰功率（写进第 29 项 `干扰功率=`，`DetectionCellResolver.cpp:150-219`）

```text
P_jam,eff = Σ_links [ P_link,before_overlap × (抗RGPO 0.5 若脉冲串) × (Σ 时频重叠占比) ]
```

三层处理：

| 层 | 处理 | 代码 |
|---|---|---|
| ① 到达功率 | 上面 3.1 的式子 | `:205` |
| ② 时频重叠 | 逐脉冲 × 逐采样点判"是否在接收窗 + 频谱是否与匹配滤波带重合" | `:166-201` |
| ③ 抗 RGPO 前拖 | 脉冲串干扰功率 ×0.5 | `:206-210` |
| ④ 多源聚合 | 各链路相加（排除自身发射）| `:161-212` |

### 3.3 干扰功率密度（2026-11-02 修正后）

```text
S_link = P_link,before_overlap / [ Ae(G_r) · 10^(−(L_rx_sys + L_pol [+L_cosite])/10) ]
S_jam  = Σ_links S_link
```

**修正前**（有缺陷）：`S = P_received_w / Ae`，把接收机侧损耗也除进了口径，同一几何下密度偏低 2–7 dB；
且按"聚合功率/逐链路功率和"等比缩放，把时频重叠与抗拖因子混进物理量。

**判据依据**：自防护干扰机在弹药上 ⇒ 来向 = 目标方向 ⇒ `G_r` 就是**主瓣增益**。

**反证**：

| 体制 | 增益 | 等效口径 Ae |
|---|---|---|
| 主瓣峰值 | 52.0 dBi | 125.95 m² |
| **实测（修正前，92 行全部 24.7–26.2 m²）** | ≈45.1 dBi | **25.8 m²** |
| 旁瓣 | 32 dBi | 1.26 m² |
| 后瓣 | 17 dBi | 0.04 m² |

⇒ 25.8 m² 既不可能是旁瓣（1.26）也不可能是后瓣（0.04），**只能在主瓣内**；
差的 7 dB 是"接收机损耗被摊进口径"，不是几何。

### 3.4 四种功率的换算口径对照

| 量 | 链路 | 距离幂 | 增益次数 | 密度换算用的 G |
|---|---|---|---|---|
| 回波功率 | 双程 | R⁴ | G_t·G_r | 主瓣/波位增益 |
| 干扰功率 | 单程 | R² | G_j·G_r | 每链路自身接收增益 |
| 噪声功率 | 内部 kTBF | 无 | 无 | 主瓣增益（等效输入口径）|
| 信号总功率 | — | — | — | 主瓣增益 |

---

## 4. 探测距离：为什么"估计 1.2 万公里"而"首次发现 5,800 公里"

### 4.1 1.2 万公里是**辐射学外推**

第 36 项（`RirAcceptanceRecords.cpp:933-947`）：

```text
R_90 = R_now · (SNR_now / SNR_req)^(1/4)          SNR_req：Marcum Q 反解 Pd=90%
```

| SNR 余量 | 距离可再远 | 5,800 km → |
|---|---|---|
| 6 dB | 1.41× | 8,193 km |
| 10 dB | 1.78× | 10,314 km |
| **13 dB** | **2.11×** | **12,258 km** ← 与"1.2 万公里"吻合 |
| 20 dB | 3.16× | 18,341 km |

### 4.2 5,800 公里是**几何可见极限**

4/3 地球半径（标准大气折射等效）下，地面雷达可见最远斜距：

| 目标高度 h | 可见最远斜距 |
|---|---|
| 500 km | 2,957 km |
| 1,000 km | 4,241 km |
| 1,500 km | 5,266 km |
| **约 2,000 km** | **6,163 km** |
| 2,900 km | 7,595 km |

⇒ 由 5,800 km 反解目标高度 ≈ **1,980 km**，正处弹道中段——**超过约 6,200 km 目标落到地平线以下，SNR 再高也照不到**。

### 4.3 五个绑定条件（按影响排序）

| # | 因素 | 量化 |
|---|---|---|
| 1 | **地平线遮蔽** | 见 4.2 |
| 2 | 搜索波位覆盖 | 波束没扫到 = 零检测，与 SNR 无关 |
| 3 | RCS 姿态相关性 | RCS 变 10 dB ⇒ 距离变 1.78× |
| 4 | 离轴增益 | G 降 ΔG ⇒ 距离缩 `10^(ΔG/40)` |
| 5 | MTI 盲速 | `mti = 4sin²(πf/PRF)`，顶点处径向速度≈0 ⇒ SINR 归零 |

---

## 5. RCS 来源：检测链**不读**"视角 RCS 表"

### 5.1 唯一开关

`hardware.rcs_physics.enable_physical_rcs`（+ `physics_mix_ratio`）

| 参数 | 当前值 |
|---|---|
| `enable_physical_rcs` | **true** |
| `physics_mix_ratio` | **0.25** |
| `cylinder_weight` | 0.5 |
| `min_rcs_m2` / `max_rcs_m2` | 0.01 / 1000.0 |
| `bistatic_psi_offset_deg` | 5.0 |

```text
RCS_检测 = (1 − mix) × 场景输入 target.rcs + mix × 物理估计(视角, 载频)
当前     = 0.75 × 场景 RCS + 0.25 × 物理估计
```

### 5.2 视角 RCS 表的用途（明确不接检测）

`src/remote_identification_radar/dwell/RirEffectiveRcs.cpp:25-30`：

> 场景给的 `视角RCS=[…]` 是**识别特征表**（第 45 项用它算 RCS 均值/标准差），不是检测回波用的 RCS。
> 要得到随视角变化的**检测** RCS，应打开 `hardware.rcs_physics.enable_physical_rcs`。

| 用途 | 消费方 | 影响检测 |
|---|---|---|
| 识别特征（第 45 项）| `RecognitionObservationBuilder.cpp:56` | ❌ |
| 日志展示 | `RirAcceptanceRecords.cpp:1138-1143` | ❌ |
| 输入校验 | `RirInputValidation.cpp:73-77` | ❌ |

---

## 6. `signal_processing` 五个参数如何影响内部

### 6.1 结构（`RirHardwareConfig.h:125-137`）

| 参数 | 缺省 | **运行值** | 语义 |
|---|---|---|---|
| `target_processing_gain_db` | 3.0 | **0.0** | 目标信号额外增益 |
| `noise_processing_gain_db` | 1.0 | **0.0** | 噪声代价 |
| `clutter_suppression_gain_db` | 10.0 | **0.0** | 杂波抑制改善因子 |
| `jamming_suppression_gain_db` | 8.0 | **8.0** | 干扰抑制 |
| `enable_mti_mtd_effects` | false | **true** | MTI/MTD 联算开关 |

### 6.2 进入 SINR 的方式（`DetectionCellResolver.cpp:293-346`）

```text
分母 = 热噪声×noise_gain×mti_noise(2.0)
     + 干扰×mti_linear / jamming_suppression
     + 杂波×mti_linear / clutter_suppression

SINR = 回波 × 脉压增益 × target_gain × mti_linear × mtd_bin_linear × 相干积累N / 分母
```

MTI/MTD 三个附加量（仅当 `enable_mti_mtd_effects && N≥2 && PRI>0`）：

| 量 | 值 | 作用 |
|---|---|---|
| `mti_linear` | `4sin²(πf/PRF)` | 对目标、干扰、杂波**同时**作用；盲速→0 |
| `mtd_bin_linear` | ≤1 | 跨多普勒 bin 损耗 |
| `coherent_linear` | **=N（20，+13 dB）** | 只抬信号 |
| `mti_noise_linear` | **2.0（+3.01 dB）** | 对消器噪声增益，只抬噪声 |

### 6.3 是否真的起作用

| 参数 | 当前是否有效 | 证据 |
|---|---|---|
| `target_processing_gain_db` | ✅（=0 ⇒ ×1）| 第 32 项 12,990 行，`总增益=60.559dB` |
| `noise_processing_gain_db` | ✅（=0 ⇒ ×1）| 第 33 项 10,392 行 |
| `clutter_suppression_gain_db` | ⚠️ **无效果**：`clutter_power_w=0` | 第 34 项 **0 行**（无输入不出日志）|
| `jamming_suppression_gain_db` | ⚠️ **无效果**：无外部干扰单音 | 第 35 项 **0 行** |
| `enable_mti_mtd_effects` | ✅ | 第 32 项有 `MTI增益=`/`MTD增益=` 子项 |

> 想让 34/35 项出数，需要先喂输入（植被/地杂波模型、场景干扰源）。

---

## 7. 第 38 项「角距测量」误差：算的是**滤波后 − 真值**

### 7.1 计算位置

```cpp
// RirAcceptanceRecords.cpp:1489-1494
TryLookPolarFromEnuM(track.position..., &range_m, &az_deg, &el_deg);
// :1594-1604
" 目标量测误差统计=斜距误差=" + FormatF(range_m - truth->truth_range_m, 1) + "m ..."
```

⇒ `斜距误差 = 航迹滤波后斜距 − 真值斜距`（不是单次量测误差）。

### 7.2 三层控制

#### ① 随机项（CRLB，`RadarEquations.cpp:82-93`）

```text
ΔR  = c / (2B)                                  ← 距离分辨率
σ_R = 0.5 · ΔR / √SNR_积累                       ← SNR<−10 dB 时退化为 1.5777·ΔR

本机 B = 30 MHz ⇒ ΔR = 5.00 m
```

| 单脉冲 SNR | σ_R |
|---|---|
| 0 dB | 2.498 m |
| 10 dB | 0.790 m |
| 20 dB | 0.250 m |
| 10 dB + N=20 积累 | 0.177 m |

#### ② 固定系统偏置（`RadarEquations.h`）

```text
kRangeMeasurementBiasM = 20.0 m                  ← 加在量测均值侧（沿视线 +20 m）
角度偏置 = 波束宽 / 30 = 0.4°/30 = 0.0133°
```

施加位置：`RirController.cpp:844-862`（仅检测器门控模式）。

#### ③ 滤波平滑（见第 11 节）

### 7.3 为什么日志里只有 0.5 m 中位

**当前 `gate_mode = kSnrFallback`**：量测位置**直接取真值**（`RirController.cpp:835-838`），
噪声与偏置都不采样 ⇒ 斜距误差只剩滤波残差。

本轮 98 行分布：

| 误差区间 | 行数 | 解释 |
|---|---|---|
| 恰好 0.0 m | 41 | 量测=真值，航迹已收敛 |
| < 1 m | 23 | 收敛期残差 |
| 1–10 m | 4 | 同上 |
| 10–50 m | 22 | 起批/重置后收敛中 |
| ≥ 50 m | 8 | 同上（最大 145 m）|

---

## 8. 「回退模式」是什么：`kSnrFallback`

### 8.1 定义（`RirPolicyConfig.h:20-23`）

```cpp
enum class RirDetectionGateMode {
  kDetectorGate = 0,   // 统计级 CFAR 判决驱动目标进入关联/滤波
  kSnrFallback  = 1    // 回退模式：SNR ≥ 6 dB 即进入（旧识别门控口径）
};
```

### 8.2 两种模式的三处差别

| 环节 | `kDetectorGate` | **`kSnrFallback`（当前）** |
|---|---|---|
| 准入门 | `detection.detected`（CFAR + 蒙特卡洛 + 边距）| `snr_db >= 6.0`（硬编码 `kSnrFallbackGateDb`）|
| 量测位置 | +20 m 距离偏置 + `bw/30` 角度偏置 + 采样噪声 | **直接取真值** |
| 误差协方差 | 与采样一致 | 照填，但均值无噪声 |

### 8.3 为什么存在

产品基线的旧识别门控口径；保留它可以让既有验收数据与标定逐位不变。
`lrdr_sensor_1q.cpp:36-39` 记录：**回退模式下 6 dB 是硬编码常量，不是 `min_snr_db`**。

### 8.4 由此"看不到"的东西

| 现象 | 原因 |
|---|---|
| 第 38 项斜距误差中位 0.5 m | 量测=真值 |
| 角度误差 `(0.000,0.000)°` | 同上 |
| 20 m 偏置、`bw/30` 角度偏置从不出现 | 回退分支不施加 |
| 第 36 项 `统计检测结果` 与"被跟踪"可能不一致 | 该项写 CFAR 判决，准入用 6 dB |

---

## 9. 第 36 项 `信噪比=` 的口径：**已含相干积累 N**

### 9.1 字段链路

| 层 | 字段 | 位置 |
|---|---|---|
| 检测单元 | `processed_single_pulse_sinr_db` | `DetectionCellResolver.cpp:335-346` |
| CFAR | `result.snr_db = processed_single_pulse_sinr_db` | `StatisticalCfarDetector.cpp:66` |
| 第 36 项 | `信噪比=…dB` | `RirAcceptanceRecords.cpp:929` |

名字里的 "single_pulse" 指**噪声基准取 1 个脉冲**，分子已乘全部处理增益（含 N）。

### 9.2 用实测反推（周期 887，目标 101）

| 项 | 数值 |
|---|---|
| 回波/噪声 = 3.393e-17 / 3.017e-13 | **−39.49 dB** |
| + 脉压 `B·τ = 60 000` | **+47.78 dB** |
| **+ 相干积累 N=20** | **+13.01 dB** |
| 小计（未计干扰/对消器）| **21.30 dB** |
| − 干扰项（干扰/噪声 +6.54 dB，抑制 8 dB）与对消器噪声 ×2 | −3.0 ~ −4.3 dB |
| = 日志信噪比 | **16.0 ~ 18.3 dB** ✅ |

⇒ 若不含 N，小计只有 8.29 dB，任何分母处理都到不了 18 dB。**N 确定已计入。**

---

## 10. 门限：起批门与 CFAR 门是两回事

### 10.1 起批最小过门限（当前回退模式）

`kSnrFallbackGateDb = 6.0`（`RirController.cpp:44`），起批另需 `confirm_hits = 1`（一次即可）。

| 口径 | 理论最小 | 换算 |
|---|---|---|
| **积累后 SINR**（判决与第 36 项印的口径）| **6.00 dB** | 直接等于门限 |
| 处理后单脉冲（含脉压、不含 N）| **−7.01 dB** | 6.00 − 13.01 |
| **原始回波/噪声**（处理前）| **−51.78 dB** | 6.00 − 47.78 − 13.01 + 3.01 |

### 10.2 CFAR 门限（检测器模式）

| 门限 | 值 | 性质 |
|---|---|---|
| `min_snr_db` | −10.0 dB | 低于它连 Pd 都不算 |
| **`min_detection_margin_db`** | **−2.0 dB** | 低于它即使抽签通过也判不检测（绝对下限）|
| CFAR 统计门槛 | `T = Q⁻¹(N, Pfa)`，`Pfa=1e-6` | 概然门槛，通常高于 −2 dB |

### 10.3 `统计检测门限=15.7719dB` 是什么

`RirAcceptanceRecords.cpp:933-939`：

```text
T = ComputeThreshold(Pfa, N_eff) = Q⁻¹(N_eff, Pfa)      // 正则化上不完全伽马逆函数
输出 10·log10(T) 并带单位 "dB"
```

| 项 | 值 |
|---|---|
| `Pfa` | 1e-6 |
| 有效脉冲数 `N_eff`（接收窗截断后）| **13**（由日志反解：`Q(13, 37.774) = 1.000e-6`）|
| `T` | **37.774** |
| 输出 | `10·log10(37.774) = 15.7719 dB` ✅ |

**它不是与 `信噪比=` 同轴的 SNR 门槛**：

```text
T 定义在 N 脉冲总能量上 ⇒ 单脉冲 SNR 门槛 = T / N
10·log10(T/N) = 15.7719 − 10·log10(13) = 4.63 dB
```

| 口径 | 值 |
|---|---|
| `统计检测门限`（N 脉冲总能量）| **15.7719 dB** |
| 换算成单脉冲 SNR | **4.63 dB** |
| `信噪比=`（已含积累）| 18.259 dB |

---

## 11. 滤波平滑在哪个文件算

| # | 环节 | 文件:行 |
|---|---|---|
| ① | 构造量测协方差 R | `RirController.cpp:833-834` |
| ② | 每周期驱动 | `RirController.cpp:1230`（`lifecycle_->Update`）|
| ③ | 关联门（马氏距离 σ=6）| `tracking/RirTrackAssociator.cpp` |
| ④ | **预测** `x⁻=Fx`，`P⁻=FPFᵀ+Q` | `tracking/RirTrackFilter.cpp:73-75` → `common/estimation/KalmanPredictor.h`（Q 单轴 `q·[[dt³/3, dt²/2],[dt²/2, dt]]`）|
| ⑤ | **更新** `K=P⁻Hᵀ(HP⁻Hᵀ+R)⁻¹`，Joseph 形式 | `tracking/RirTrackFilter.cpp:77-87` → **`common/estimation/KalmanUpdater.h:59-104`** |
| ⑥ | 后验写回航迹 | `tracking/RirTrackLifecycle.cpp:664-666`（`ApplyGaussianState`）← 第 38 项读的就是它 |
| 漏检 | 只预测不更新 | `RirTrackLifecycle.cpp:669-679` |
| IMM | confirmed 航迹多模型 | `tracking/RirImmFilter.cpp` |

### 11.1 更正：不是 `σ_R/√N_hits`

那个式子只在**批量最小二乘**时成立。递归卡尔曼在 `q>0` 时进入**稳态**，不再随命中数下降。

本机数值（q=30、dt=1 s、量测噪声取横向角度主导 640 m）：

| 命中数 | 卡尔曼后验 σ | 理想 `σ_z/√N` |
|---|---|---|
| 1 | 638.8 m | 640.0 m |
| 3 | 579.7 m | 369.5 m |
| 10 | 376.2 m | 202.4 m |
| 50 | 224.4 m | 90.5 m |
| **稳态** | **224.1 m** | 10.1 m（永不达到）|

⇒ 正确表述：**初期按 1/√N 下降，随后被过程噪声托住，稳态约为 σ_meas 的 0.35 倍**。

---

## 12. 卫星定位误差：已设为 10 cm

| 字段 | 原值 | 现值 | 作用 |
|---|---|---|---|
| `policy.error_model.nav_position_sigma_m` | 0.0 | **0.1 m** | 第 8 项"定位误差"**径向**分量 1-σ（`SbirsAcceptanceRecords.cpp:176`）|

**只影响验收行，不影响物理链路**（`SbirsPolicyConfig.h:38`：仅验收旁路 ECEF 误差行使用）。

真正进测角误差模型的量（`SbirsErrorModel.cpp:60-65`）：

```text
σ_θ,RSS = √(orbit_sigma_deg² + attitude_sigma_deg² + fov_sigma_deg²)
当前     = √(0² + (1e-4)² + (5e-5)²) = 1.118e-4 deg = 0.402″
⇒ GEO 处横向 ≈ 78 m
```

| 想要的效果 | 应改 |
|---|---|
| 仅第 8 项显示 10 cm | `nav_position_sigma_m = 0.1`（已完成）|
| 让卫星位置误差按 10 cm 进链路 | `orbit_sigma_deg = 1.43e-7°`（≈0.0005″），对总 RSS 贡献 0.12%，实质无影响 |
| 让探测精度整体到 10 cm | 需把 `attitude_sigma_deg` 降到 ~1.3e-7°（相当于换高 3 个数量级的星，不现实）|

---

## 13. 待决问题与已修缺陷

| # | 问题 | 状态 |
|---|---|---|
| 1 | **误差模型重复计入 N**：`RirController.cpp:805-810` 的 `detection.snr_db + 10log10(N)` 与检测单元已含的 `coherent_linear=N` 重复 ⇒ σ 偏小 √20≈4.4 倍 | **待修**（建议：`enable_mti_mtd_effects` 为真时不再叠加）|
| 2 | `Pd` 计算同样按"单脉冲"处理已含 N 的 SINR（`RadarEquations.cpp`），N 被乘两次 | **待修**（与 #1 同源）|
| 3 | `RirController.cpp:672-676` 注释称"当且仅当目标增益与杂波抑制都置 0 才启用 MTI/MTD"，实际判据只看 `enable_mti_mtd_effects && pulse_count>=2` | **注释过期**（行为以代码为准）|
| 4 | `统计检测门限` 与 `信噪比` 不同轴（前者是 N 脉冲总能量门限）| 建议另印"单脉冲 SNR 口径"门槛 |
| 5 | 视角 RCS 表不参与检测 | 设计如此（`RirEffectiveRcs.cpp:25-30`），如需参与须改代码并复核标定用例 |
| 6 | 干扰功率密度曾把接收机损耗摊进口径 | **已修**（1q `e4520ef8`）|

---

## 附录 A：公式速查

```text
# 波长 / 天线
λ  = c / f
Ae = G · λ² / (4π)

# 回波（双程）
P_echo = P_t,rad · G² · λ² · σ / ((4π)³ R⁴ L)
S_echo = P_echo / Ae

# 干扰（单程）
P_jam,link = P_j · G_j · G_r · λ² / ((4π)² R² L_prop) / (接收机侧损耗)
S_jam      = Σ P_link / (Ae(G_r) · 10^(−(R_侧损耗)/10))

# 噪声
P_noise   = k · T · B · F
S_noise   = P_noise / Ae(主瓣)

# 处理链 SINR
SINR = P_echo · G_pc · G_proc · H_mti · H_mtd · N / (P_noise·G_noise·2 + P_jam·H_mti/Γ_j + P_clt·H_mti/Γ_c)

# MTI/MTD
H_mti    = 4 sin²(π · f_fold / PRF)
H_mtd    = |Σ_k exp(−j…)|² 的最优 bin 响应（≤1）
Γ_noise  = 2（kMtiNoiseFactor）

# 检测
T          = Q⁻¹(N, Pfa)                     # N=1 时 T = −ln(Pfa)
Pd         = MarcumQ(N, √(2·N·SNR_p), √(2·T))
起批(回退)  = SNR_积累 ≥ 6.0 dB
起批(检测器) = Pd 抽签 ∧ SNR ≥ min_detection_margin_db(−2 dB)

# 测量误差
ΔR  = c / (2B)
σ_R = 0.5 · ΔR / √SNR
σ_θ = 0.5 · θ_bw / √SNR
偏置：距离 +20 m；角度 θ_bw/30

# 探测距离
R_90 = R_now · (SNR_now / SNR_req)^(1/4)
地平线（4/3 Re）：d ≈ √(2·Re_eff·h_radar) + √(2·Re_eff·h_target)
```

## 附录 B：日志字段 → 口径对照

| 日志字段 | 出现项 | 口径 |
|---|---|---|
| `回波功率=` | 29 | 处理前双程功率 |
| `回波功率密度=` | 29 | `P_echo / Ae(G主瓣)` |
| `噪声功率=` | 29 | `kTBF` 单脉冲热噪声 |
| `噪声等效输入功率密度=` | 29 | `P_noise / Ae(G主瓣)` |
| `干扰功率=` | 29 | 检测等效（含时频重叠、抗 RGPO、多源聚合）|
| `干扰功率密度=` | 29 | 口面入射密度（接收机侧损耗已还原）|
| `到达雷达的干扰功率=` | 41 | 单链路原始到达功率（未处理）|
| `信噪比=` | 36 | **已含相干积累 N** 的 SINR |
| `统计检测门限=` | 36 | `10log10(Q⁻¹(N,Pfa))`，N 脉冲总能量轴 |
| `斜距误差=` | 38 | 滤波后航迹斜距 − 真值 |
| `方位/俯仰误差=` | 38 | 滤波后角 − 真值（最短角差）|
| `检测概率=90%的最大探测距离=` | 36 | R⁻⁴ 外推（假定 RCS/增益不变）|
| `导航定位误差` / `定位误差ECEF` | 第 8 项(SBIRS) | 径向 = `nav_position_sigma_m`；切向 = `orbit_sigma_deg × 斜距` |

---

*本文数字均取自 2026-11-02 的实测日志（`FOSimStudio/log/RadarOutput.log`、`SatelliteTarget.log`）与本机配置，代码行号对应当日 `fosim_2013` 分支。*
