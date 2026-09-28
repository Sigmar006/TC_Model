---
Status: draft
Date: 2026-09-28
Review-Baseline: 日志产物 `build/acceptance-v141/Debug/bin/log/rir_ground_site_recognition/`（Debug 二进制 2026-09-28 19:09，40 周期 740–779）；源码工作树 `main` @ `1ba68de6` + 3 个编码收口提交（`a312bd10`/`b16847f0`/`56888208`，不涉及本文数值链）
Authority: 非规范性数值核对记录。配置权威为 `examples/scenes/rir_ground_site_recognition/rir_ground_site_recognition.json`；字段口径权威为 `src/remote_identification_radar/runtime/RirAcceptanceRecords.cpp`。若本文与库实现冲突，以库为准。
---

# RIR 验收日志数值核对：`rir_ground_site_recognition`（LRDR 口径）

## 0. 结论速览

- 对日志中参与雷达方程的 11 类量逐一独立复算，**全部与日志位级吻合**（回波功率 0.000 dB、
  热噪声 0.005%、SNR 0.001 dB、Pd 1e-5、Pd=90% 距离 0.0006%、MTI/MTD ≤0.022 dB、
  量测协方差 ≤0.3%）。**数值链自洽，未发现物理计算错误。**
- 发现 **9 项**：3 项设计口径（DES）、4 项正确性/表述（Q）、2 项清理（BND）。其中
  Q-1、Q-2 会直接影响甲方读日志得出的结论，DES-1 影响甲方指标点的可达性。
- **口径提醒（评审勿混用）**：甲方两点指标 3400 km@0.025 m² 与 8550 km@1 m² 是 **6 dB 门**
  口径（复算单脉冲 SINR 均 **+9.90 dB**，对应 Pd≈0.23）；第 36 项
  `检测概率=90%的最大探测距离` 是 **Pd=0.9** 口径（本场景 3981 km@0.1 m²，需求 SNR 13.18 dB）。
  两者相差 4.9 dB~7 dB，不能互相印证。

## 1. 判定方法与输入

### 1.1 核对对象

| 文件 | 规模 | 说明 |
|---|---|---|
| `RadarOutput.log` | 2850 行 / 32 个验收项 | 功能项 28/29/31/32/33/34/35/36/48 等 |
| `RadarInternalInterface.log` | 192 行 | 内部接口 70/71（回波→处理、处理→检测） |
| `RadarExternalInterface.log` | 586 行 | 外部接口 + 资源状态告警 |
| `RadarAntennaGainPatterndata.csv` | 406 351 行 | 方向图 az 0–360° × el ±90°，步长 0.4° |
| `Rir_scan_pattern.csv` | 1386 行 | 波位表（231 az × 6 el） |

产生者：`rir_ground_site_recognition.exe`（Debug，构建于 **19:09**），40 周期（740–779），
场景 `start_cycle=740`。注意：该二进制早于 `75a7659b`（19:49，"无输入不出日志"收紧），
对第 36 项行数有影响（见 BND-1）。

### 1.2 配置（`rir_ground_site_recognition.json` 的 `session_config.rir`，场景自持）

| 参数 | 值 | 行 |
|---|---|---|
| 峰值功率 / 频率 / 带宽 / 脉宽 / PRF / 发射损耗 | 582 kW / 3 GHz / 4.5 MHz / **16 ms** / **10 Hz** / 3.5 dB | :285-290 |
| 天线 | **52 dBi** / 0.4°×0.4° / 17.5 m×17.5 m / kGaussianMainLobe / SLL −20 dB / 后瓣 −35 dB | :293-306 |
| 接收机 | NF 4 dB / 收损 2 dB / 交叉极化隔离 30 dB / 线性上限 1 mW | :309-313 |
| 处理增益 | 目标 3 dB / 噪声 1 dB / 杂波抑制 10 dB / 干扰抑制 8 dB | :316-319 |
| 检测策略 | Pfa 1e-6 / min_snr −10 dB / margin −2 dB（绝对下限）/ pulse_count 1 / 门 kSnrFallback | :358-363 |
| 任务 | 驻留 **0.05 s** / max_range **9e6 m** / 扫描窗 az ±92°、el 2–6° / scan_center az −110° / 步长系数 2.0 | :339-353 |
| 环境 | 天气 0 dB / 植被 kDisabled / 大气物理模型 false | :396-399 |
| 派生 | PRI 0.1 s；单驻留脉冲 `ceil(0.05/0.1)=1`；每周期驻留 `floor(1/0.05)=20` | :493-499（RirController.cpp） |

### 1.3 方法

- **独立复算**：Python 从第一性原理实现雷达方程、kTB 噪声、匹配滤波脉压、Marcum Q 检测模型、
  2 脉冲 MTI + 8 点 DFT 滤波器组、ENU 雅可比量测协方差；不复用库代码。
  脚本：`tmp/analysis/verify_numbers.py`、`verify_mti_mtd.py`、`antenna_csv_check.py`、
  `rir_log_inventory.py`（未跟踪，可删）。
- **源码逐行核对**：确认"哪条公式真正落盘"（主路径 `TryResolveDetectionCell` vs
  回退路径 `RirRadarEquations`），以及每个字段的来源与剔除规则。
- **全量异常值扫描**：32 个验收项、3628 行，按字段聚合 min/max/mean 并标记非有限、
  负值、恒定零、跨度异常。
- **未做**：未重新构建、未重跑场景（结论基于已有日志 + 现有源码）。

发现分类：DES（设计口径）/ BND（边界清理与证据卫生）/ Q（正确性与表述）；
严重度：高 / 中 / 低 / 建议。

## 2. 逐项复算（已核查无问题）

样本：周期 751/760/761/775（目标 208/227，斜距 3.92–3.97 Mm，RCS 0.1 m²）。

| 量 | 日志值 | 独立复算 | 偏差 | 口径来源 |
|---|---|---|---|---|
| 回波功率（R=3 947 862.0 m） | 8.536e-18 W | 8.535804e-18 W | **0.000 dB** | `DetectionCellResolver.cpp:215-224`（Pt 已扣 3.5 dB：`RirEmissionFactory.cpp:79-86`） |
| 热噪声 | 4.526e-14 W | k·290·4.5e6·10^0.4 = 4.52578e-14 W | 0.005% | `DetectionCellResolver.cpp:231-233`；T₀=290 K `RadarEquations.cpp:34` |
| 脉压增益 | 48.573 dB | 10lg(B·τ)=10lg(72000) | 0.000 | `DetectionCellResolver.cpp:229-230` |
| 单脉冲 SNR | 13.329 dB | Pr/N + 48.573 + (3−1) dB | 0.001 dB | `DetectionCellResolver.cpp:246-262` |
| 统计门限 | 13.815511 | −ln(1e-6) | 0.000 | `RadarEquations.cpp:168-181` |
| 检测概率 Pd | 0.91769 | MarcumQ₁(√(2χ),√(2T)) | 1e-5 | `RadarEquations.cpp:271-292` |
| Pd=90% 最大距离 | 3 981 030.4 m | R·(SNR/13.1835 dB)^¼ | 0.0006% | `RirAcceptanceRecords.cpp:322-344,709-725` |
| 时延 / 双程多普勒 | 2.634e-2 s / 62 136.6 Hz | 2R/c / 2v_r/λ | 0.000 | `DetectionCellResolver.cpp:225-228` |
| MTI / MTD 增益 | 4.859 / −1.198 dB | 4sin²(πf_d/PRF) / 8 点 DFT | ≤0.022 / ≤0.006 dB（48 样本） | `MtiMtdAcceptanceBank.cpp:35-56,90-107` |
| 量测协方差 diag | 271 240 / 3 275 551 / 3 542 161 m² | σ_R=0.5(c/2B)/√SNR、σ_θ=0.317θ_bw/√SNR 经 ENU 雅可比 | ≤0.3% | `MeasurementErrorModel.h:68-79`；`RirController.cpp:425-458` |
| 天线三件套自洽 | 52 dBi / 0.4° / 17.5 m | η(πD/λ)²→52.6 dBi（η≈0.53）；70λ/D=0.400° | <0.7 dB | `RirSessionConfigValidation.cpp:151-164` |

复算锚点（可对账）：λ=0.0999308 m；Pt_rad=259 969.85 W；G=158 489.3；L_rx=1.5849；
(4π)³=1984.401；(B·τ)=72 000→48.5733 dB；N=4.525784e-14 W；SNR_req(Pd=0.9, Pfa=1e-6,
N=1, Swerling0)=**13.1835 dB**。

**异常值扫描结论（无问题）**：32 项 3628 行内无 NaN/Inf、无负功率/负距离、无越界；
占空比 16%（16 ms/100 ms）与驻留/波位自洽；接收前端饱和余量 42.8 dB
（自耦合 5.25e-8 W vs 线性上限 1e-3 W）；遮蔽告警仅对目标 210/231/249
（`occultation_margin_m` −13 141/−14 407 m，视线穿地，正确地未产生检测）；
RCS 直接用场景声明值（`enable_physical_rcs=false` 时不夹取 min/max，属既定口径）；
识别置信度 ≈0.10、全程无型号确认，与"交付特征库无对应型号"的预期一致。

## 3. 发现

### 3.1 DES：设计口径

| 编号 | 严重度 | 发现 | 证据 |
|---|---|---|---|
| DES-1 | **高** | **接收窗 = 驻留时长（0.05 s），把可探测距离卡在 7 494.8 km**：`TryCountAvailableEchoPulses` 要求 `pulse_start + 回波时延 < 窗末`，而窗 = `recognition_dwell_sec`。8550 km 的时延 57.0 ms > 50 ms → `effective_pulse_count=0` → `DetectResolvedCell` 因 `pulse_count<=0` 直接判无检测。于是**甲方 8550 km@1 m² 指标点在本场景物理上不可达**（尽管链路预算 +9.9 dB），而 `mission.max_range_m=9e6` 反而更大——两个距离门互不一致；3400 km@0.025 m²（时延 22.7 ms）不受影响。 | `DetectionCellResolver.cpp:48-70,234-237`；`RirController.cpp:638-639`；`StatisticalCfarDetector.cpp:59-62`；`json:344,345` |
| DES-2 | 中 | **PRF 10 Hz 与 3 km/s 量级目标不相容**：双程多普勒 62.1 kHz 折叠进 ±5 Hz，MTI/MTD 增益因此逐周期乱跳（MTI −21.7…+6 dB，`总增益` 46–52.5 dB）；且单驻留仅 1 脉冲，所谓 8 通道 MTD 是**单脉冲上的纸面计算**，无相干处理区间。数值自洽但无物理意义。 | `MtiMtdAcceptanceBank.cpp:35-56,90-107`；日志 `目标信号增益功能测试` 行 |
| DES-3 | 中 | **主路径与回退路径回波功率口径差 +30.9 dB**：RF 解析失败时 `RadarEquations` 额外按 `τ/13 µs` 缩放能量（τ=16 ms → +30.9 dB），主路径无此项。本场景走主路径不受影响，但同一目标在两条路径下数值不可比。 | `RadarEquations.cpp:62-67,92-119`；`RirSignalDetector.cpp:70-71`；`internal/RirRadarEquations.h:64-77` |

### 3.2 Q：正确性与表述

| 编号 | 严重度 | 发现 | 证据 |
|---|---|---|---|
| Q-1 | **高** | **第 30 项把雷达自身同址耦合写成"干扰源"**：日志 `干扰源ID=5 到达雷达功率=5.249e-08W`（ID 5 = 本雷达），而同周期第 29 项写 `干扰功率=0.000W`、第 35 项整条省略。复算 = 259 969.85 W ×10^−12.2（120 dB 同址隔离 + 2 dB 收损）×0.32（16 ms/50 ms 时窗重叠）= 5.2489e-8 W，与日志一致。成因：控制器为饱和门**有意**把自发射追加进解析场景，而第 30 项写出**全部**入射链、未做检测链那侧的自身剔除。审计者据此会误判"存在 +60.6 dB 的干扰"。 | `RirController.cpp:538`；`RirAcceptanceRecords.cpp:786-790`；剔除侧 `DetectionCellResolver.cpp:117`、`RirController.cpp:731-737` |
| Q-2 | 中 | **`方位扫描范围=-179.600/179.600°` 是缝合线假象**：波位表 az = normalize(scan_center + 相对 az) = −110° + (−92…92) = −202°…−18°，归一化到 ±180 后跨缝，min/max 打印成 ±179.6°，读起来像全向扫描（实际扇区 184° 宽）。与配置的指向 ±110°／扫描窗 ±92° 都不符。 | `RirAcceptanceRecords.cpp:1680-1711`；`RirSession.cpp:171-177`；`Rir_scan_pattern.csv`（az −179.6…179.6，231 值，步长 0.8°） |
| Q-3 | 中 | **第 33 项四个子项参照系不一致**：`脉冲压缩后的噪声功率`=N；`各多普勒滤波器通道噪声功率`=N/4（kMtiNoiseFactor 2 ÷ 8 通道）；`MTD等效噪声功率`=N/4；`噪声总增益`=+1.000 dB（配置回显）。三者隐含 −6 dB（每通道）／+3.01 dB（8 通道合计）／+1 dB（配置）三种口径，同一行无法自证。 | `MtiMtdAcceptanceBank.cpp:98-99`、`.h:19-20`；`RirAcceptanceRecords.cpp:621-628` |
| Q-4 | 中 | **交付的 `RadarAntennaGainPatterndata.csv` 无法复现 0.4° 波束**：网格步长 = 波束宽度（0.4°），主瓣只落 1 个采样点（(0,0)=52.000 dBi），0.4°–1.2° 平在 32.000 dBi（−20 dB 副瓣地板）；全场 p50=17 dBi（后瓣）、49.6% 点 <0 dBi、极区固定 −68 dBi（两轴 sinc² 各 clamp 1e-6）；另有 ±az 镜像差 2.6 dB（浮点累加导致格点漂移，落在 sinc 零点附近被放大）。第 28 项的校验实际在 0.02° 内部网格上完成，与附件 CSV 不可互证。 | `RirAcceptanceRecords.cpp:1723-1746`；`AntennaPatternRuntime.h:116-123,143-153,303-305`；CSV 抽样 `(0.40,0)=32.000`、`(0,90)=17.000`、`(0,−90)=−5.301`、`(3.60,10)=−41.259` vs `(356.40,10)=−43.862` |

### 3.3 BND：边界清理与证据卫生

| 编号 | 严重度 | 发现 | 证据 |
|---|---|---|---|
| BND-1 | 低 | **日志与源码版本不同步**：本目录日志由 19:09 二进制产生，早于 `75a7659b`（19:49，无输入不出日志）。故日志中仍有 4 行 `统计检测结果=否`；当前源码在 `detected=false` 时**整条不写**（含 Pd=90% 距离行）→ 重跑后这些行消失，"没有行"≠"没目标"。归档证据时应记录构建提交号。 | `RirAcceptanceRecords.cpp:558-561`；commit `75a7659b`；日志 `仿真周期=753/762/772/775` |
| BND-2 | 低 | **场景文档与配置漂移**：`rir_ground_site_recognition.md:37` 仍写"46 dBi + 0.8°/0.8° + 8.75 m 口径"，JSON 已是 52 dBi + 0.4°/0.4° + 17.5 m；同文"每周期驻留数 = 10"与 `dwell 0.05 s → 20` 也不一致。以 JSON 为准。 | `rir_ground_site_recognition.md:37,41-42` vs `json:293-297,345` |

### 3.4 已核查无问题（防止重复审查）

- 回波功率 / 噪声 / 脉压 / SNR / 门限 / Pd / R90 / 时延 / 多普勒 / MTI / MTD / 量测协方差
  共 12 类量已复算到日志精度上限（§2），无需再审。
- 天线 52 dBi ↔ 0.4° ↔ 17.5 m 三件套自洽（70λ/D=0.400°，η≈0.53），校验器口径
  `G≈10lg(26000/(θaz·θel))=52.11 dBi` 通过。
- 甲方两点指标（3400 km@0.025 m²、8550 km@1 m²）在**链路预算**口径下均为 +9.90 dB，
  与设计文档/单测钉值一致；不可达问题属 DES-1 的接收窗口径，不是链路预算错误。
- 遮蔽告警、接收前端饱和、门控与排除原因行为均与代码契约一致。
- 编码侧（验收文件 UTF-8 无 BOM、字面量 GBK 安全）已由 `a312bd10`/`b16847f0` 收口，
  本文不重复。

## 4. 建议修复优先级（报告项，未实施）

| 优先级 | 编号 | 问题 | 修复建议 |
|---|---|---|---|
| 高 | Q-1 | 第 30 项把自身耦合当干扰源 | 写出前按本雷达 `sensor_entity_id` 剔除自身链（或改写为 `自身同址耦合=` 字段），与第 29 项 `干扰功率` 同源同口径 |
| 高 | DES-1 | 接收窗=驻留 → 7494.8 km 上限，8550 km 指标不可达 | 接收窗改为 ≥ PRI（0.1 s → 14 989 km）或 `max(驻留, 2·max_range/c)`；同步核对 `mission.max_range_m=9e6` 与窗的一致性 |
| 中 | Q-2 | 方位扫描范围缝合线 | 按"相对扫描中心的扇区宽 + 中心"打印，或对 min/max 做非缝合归一化（如输出 −202…−18°） |
| 中 | DES-2 | PRF/多普勒/单脉冲 MTD 不相容 | 提高 PRF 或引入中 PRF/CPI 假设；至少在文档标注第 32/33 项为"频谱效能示意" |
| 中 | Q-3 | 第 33 项参照系混用 | 四个子项统一标注参照（每通道/全通道、处理前/后），`噪声总增益` 由滤波器组实算 |
| 中 | Q-4 | 方向图 CSV 无法复现主瓣 | 主瓣区（±3·bw）用 ≤0.05° 步长（或分段网格）；极区/clamp 地板值标注为数值下限；修正格点累加的浮点漂移 |
| 中 | DES-3 | 两条路径回波功率差 30.9 dB | 回退路径去能量缩放，或让两路径共用同一"峰值功率→SINR"口径 |
| 低 | BND-1 | 日志与源码版本不同步 | 证据归档记录 commit；重跑后用新行为重新基线 |
| 低 | BND-2 | 场景文档漂移 | 同步 `rir_ground_site_recognition.md` 的天线与驻留数 |

## 5. 结论

`rir_ground_site_recognition` 日志中的雷达数值**链内自洽、可复现、无计算错误**：
回波功率严格等于教科书雷达方程（含 3.5 dB 发射损耗与 2 dB 接收损耗）、热噪声等于
kT₀BF（T₀=290 K、NF=4 dB）、SNR = Pr/N + 脉压 + 净处理增益、Pd 与门限出自同一 Marcum Q
模型、Pd=90% 距离由该模型反解、MTI/MTD 为确定性的 2 脉冲对消器 + 8 点 DFT 响应、
量测协方差由距离/角度误差经 ENU 雅可比投影——全部与日志位级吻合。

不阻断数值正确性、但会影响甲方判读或指标可达性的问题集中在**口径表述**（Q-1、Q-2、Q-3）
与**波形/接收窗设计口径**（DES-1、DES-2、DES-3）；建议至少修 Q-1 与 DES-1，
其余按 §4 优先级随提交消化。

> **后续状态**：修复落库后在此追加"编号 → commit"映射、实施中额外发现与暂缓项。
