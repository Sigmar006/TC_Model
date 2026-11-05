# 红外（SBIRS）与 LRDR 雷达（RIR）处理链：逐步说明与代码索引

> 本文回答一个问题：**一次仿真周期里，红外与雷达各自经历了哪些处理过程，每一步在哪个文件/函数里，公式与参数是什么，落到哪条验收项。**
> 对应源码已同步加了中文过程注释（文件头 `## 过程：…` 块 + 函数内 `// 过程：…` 标记），本文是它们的索引版。

---

# 第一部分：LRDR 远程识别雷达（RIR）

## 0. 链路总览

```text
场景几何 ──► 天线方向图（单程/双程增益）
   │              │
   ▼              ▼
波形参数 ──► ① 回波功率 Pr ──► ② 传播/大气损耗
                                  │
   热噪声（接收机） ──┐            ▼
   面杂波 ──────────┼──► ③ SINR 账本（信号/噪声/杂波/干扰四账）
   干扰 ────────────┘            │
                                  ▼
                    ④ 信号处理增益：脉压 B·τ、MTI、MTD
                                  │
                                  ▼
                    ⑤ 检测：SNR → Pd（Marcum Q）→ CFAR 抽签
                                  │
                                  ▼
                    ⑥ 量测（距离/方位/高度 + σ）
                                  │
                                  ▼
                    ⑦ 关联（距离门 6σ + LAPJV）──► ⑧ 生命周期状态机
                                  │
                                  ▼
                    ⑨ 滤波（CV-KF / IMM）
                                  │
                                  ▼
                    ⑩ 特征：RCS / 运动 / 极化 / 宽带一维像（HRRP）
                                  │
                                  ▼
                    ⑪ 综合识别（特征库匹配 + 积累判决）──► 输出
```

## 1. 逐步说明

### ① 回波功率
- **文件/函数**：`src/remote_identification_radar/internal/RirRadarEquations.cpp:35,43`
  `ComputeEchoPowerWithGain_dBW` / `ComputeEchoPower_dBW`
- **公式**：
  ```text
  Pr = Pt · G_t · G_r · λ² · σ / ((4π)³ · R⁴ · L)      （单站：G_t = G_r = G）
  ```
- **参数**：`transmitter.peak_power_w`=3 MW、`frequency_hz`=3 GHz、`transmit_loss_db`=3.5、
  `receiver.receive_loss_db`=2、`rcs_m2`（场景或物理 RCS）
- **对应验收**：第 29 项（回波功率计算）

### ② 传播与大气
- **文件**：`src/remote_identification_radar/internal/RirPropagationModel.cpp`
- **内容**：距离平方/四次方律之外的附加损耗、天气衰减（`environment.weather_attenuation_db`=0）、
  可选大气物理模型（当前关闭）
- **对应验收**：第 29 项辅助字段（传播损耗）

### ③ 天线方向图与增益
- **文件**：`src/remote_identification_radar/dwell/RirRfFrontEndResolver.cpp`、
  `src/remote_identification_radar/dwell/RirEffectiveRcs.cpp`
- **内容**：按离轴角取样方向图（高斯主瓣 + 副瓣 −20 dB + 后瓣 −35 dB）得到单程增益；
  双程 = 2×单程；扫描损耗系数 0 dB/deg²（上限 6 dB）
- **对应验收**：第 28 项（天线方向图）

### ④ 噪声 / 杂波 / 干扰（SINR 账本）
- **热噪声**：`RirRadarEquations::ComputeThermalNoisePower_W`（`kTB` + 噪声系数 4 dB）
  ← `RirReceiverStateBuilder.cpp`
- **面杂波**：`internal/RirSurfaceClutterModel.cpp:119-135`
  ```text
  clutter_rcs = σ0(植被) + 擦地角因子 + 分辨率单元面积
  CNR         = 10·log10(P_clutter / P_noise)
  ```
  当前 `vegetation_cover_profile=kOpenGrassland` 但**杂波抑制增益 0 dB**、
  实际杂波功率未进入检测链（验收第 34 项因此整条省略）
- **干扰**：外部链路 `incident_links`（当前无干扰源）→ 第 35 项
- **对应验收**：第 33 项（噪声增益）、第 30/34/35 项

### ⑤ 信号处理增益（脉压 / MTI / MTD）
- **文件**：`src/remote_identification_radar/dwell/RirDetectionCellResolver.cpp`、
  `RirRadarEquations::ComputePulseCompressionGainDb`
- **公式**：
  ```text
  脉压增益 = 10·log10(B · τ) = 10·log10(30 MHz × 16 ms) ≈ 56.8 dB
  MTI/MTD 增益：来自处理组（mti_gain_db / mtd_gain_db），选择性通道输出
  ```
- **口径**：检测消费的是"**处理后单脉冲 SINR**"，第 36 项 `信噪比=` 与该口径一致
- **对应验收**：第 32 项（目标信号增益）、第 33 项

### ⑥ 检测判决（CFAR）
- **文件**：`src/remote_identification_radar/dwell/RirSignalDetector.cpp` +
  `src/common/radar/StatisticalCfarDetector.cpp:55-77`
- **流程**：
  ```text
  1) Pd = MarcumQ(N, √(2N·SNR_lin), √(2T))，T = Q⁻¹(N, Pfa)
  2) 若 SNR < min_snr_db(-10 dB) → Pd = 0（直接不检）
  3) 抽签判 detected = (U < Pd)
  4) 若 detected 且 SNR < min_detection_margin_db(-2 dB) → 强制不检
  ```
- **口径**：第 36 项 `统计检测门限=` 现取"同一模型下 **Pd=50% 所需的单脉冲 SNR**"（实测 11.243 dB）
- **对应验收**：第 36 项

### ⑦ 量测构造
- **内容**：距离 R（含 `ComputeRangeErrorStdDev = c/(2B√SNR)`）、方位/俯仰
  （`ComputeAngleErrorStdDev = 0.317·θ_bw/√SNR`）、高度换算；写入 `RirTargetReturn`
- **对应验收**：第 38 项（角度与距离测量）

### ⑧ 关联
- **文件**：`src/remote_identification_radar/tracking/RirTrackAssociator.cpp`
- **内容**：新息距离门 **6σ** → LAPJV 全局最近邻分配 → 未关联量测起新航迹
- **参数**：`policy.association.distance_gate_sigma = 6.0`
- **对应验收**：第 41 项（航迹关联）

### ⑨ 生命周期
- **文件**：`src/remote_identification_radar/tracking/RirTrackLifecycle.cpp`
- **内容**：确认（`confirm_hits=1`）→ 保持（`max_miss_before_lost=418`）→
  丢失（`max_lost_cycles=2090`）→ 重捕；`enable_imm_lifecycle=true` 时切换 IMM
- **对应验收**：第 39/40 项

### ⑩ 滤波
- **文件**：`tracking/RirTrackFilter.cpp`（CV-KF）、`tracking/RirImmFilter.cpp`（IMM）
- **公式**：
  ```text
  x⁻ = F x ,  P⁻ = F P Fᵀ + Q ,  Q_轴 = q·[[dt³/3, dt²/2],[dt²/2, dt]]
  K  = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹ ,  P⁺ = (I − K H) P⁻
  ```
- **参数**：`policy.tracking.kalman_noise_diff_coeff = 30`；量测 R 由步骤⑦的 σ 构造
- **对应验收**：第 42 项（跟踪滤波）

### ⑪ 特征提取
| 特征 | 文件 | 关键量 |
|---|---|---|
| RCS 统计 | `recognition/RcsFeatureExtractor.cpp` | 均值/起伏/分位数 |
| 运动 | `recognition/MotionFeatureExtractor.cpp` | 速度、加速度、机动强度 |
| 极化 | `recognition/PolarizationFeatureExtractor.cpp` + `PolarizationStatsExtractor.cpp` | HH/VV/HV 比、极化度、极化熵 |
| 宽带一维像（HRRP） | `recognition/RangeProfileFeatureExtractor.cpp` | 长度、散射中心数、相关系数 |
| 观测组装 | `recognition/RecognitionObservationBuilder.cpp` | 四维特征向量 + 质量 |
- **对应验收**：第 44（运动）、45（RCS 统计）、46（极化）、47（一维像）项

### ⑫ 综合识别
- **文件**：`recognition/RecognitionMatcher.cpp`、`RecognitionTracker.cpp`、`RecognitionFeatureDatabase.cpp`
- **公式**：
  ```text
  score = Σ wᵢ · simᵢ      w = (RCS 0.25, 运动 0.25, 极化 0.25, 一维像 0.25)
  通过条件：score ≥ accept_score(0.6) 且 (score − 次优) ≥ minimum_margin(0.05)
  积累窗 60 s / 最少观测 1 / 结果保持 120 s
  ```
- **对应验收**：第 47A 项（综合特征识别）

---

# 第二部分：红外预警卫星（SBIRS）

## 0. 链路总览

```text
场景真值(ECEF) ──► GMST 旋转 ──► ECI
                                   │
        扫描基准（ECI 绝对 / 相对星下点）──► 栅格行/相位 ──► 传感器系指向
                                   │                        │
                                   ▼                        ▼
                    逐目标几何：遮挡/临边、视线 az/el、距离、相对角速度
                                   │
                                   ▼
                    辐射强度 ──► 入瞳辐照度 ──► 大气透过率（壳段 τ_geo）
                                   │
                                   ▼
                    噪声：NEP + 背景光子 + 读出 ──► 线性 SNR
                                   │
              ┌────────────────────┴────────────────────┐
              ▼ 宽场（WFOV）                            ▼ 窄场（NFOV）
   四门：视场→遮挡→SNR(4.0)→驻留              调度→指向(扰动)→稳定→帧门(SNR 5.0)
              │                                          │
              ▼                                          ▼
        候选累积 / 首次探测事件                    跟踪量测 → EKF(6维 ECI CV)
              │                                          │
              └────────────► 宽窄联合输出 ◄───────────────┘
                                   │
                                   ▼
                    测角误差（5 项 RSS / 线性 SNR）+ 跟踪位置误差与协方差
                                   │
                                   ▼
                          状态机迁移 + 验收行
```

## 1. 逐步说明

### ① 扫描基准与指向链
- **文件**：`pipeline/SbirsPipeline.cpp`（「方位基准」段）、`pipeline/SbirsBoresightChain.cpp`
- **内容**：
  - `scan_azimuth_reference`：`kEciAbsolute`（起点=Eci 绝对方位）/ `kNadirRelative`（起点=相对星下点方位 `atan2(−y,−x)`）
  - 栅格：`scan_el_start_deg=-8`、`scan_el_step_deg=8`、跨度 24°（4 行）× 方位 360°（速率 36°/s）
  - 旋转链：姿态(ECI→Body) → 安装角(Body→Sensor) → 失准角
- **对应验收**：第 12 项（宽视场扫描）、第 28 项（安装矩阵/第 10 项）

### ② 场景 ECEF → ECI
- **文件**：`pipeline/SbirsEciScene.cpp`、`SbirsPipeline.cpp`「过程②」标记处
- **内容**：`r_ECI = R3(GMST)·r_ECEF`（仅绕 z；忽略岁差/章动/极移）

### ③ 逐目标几何
- **文件**：`foundation/SbirsGeometry.cpp:50-60`
- **公式**：`az = atan2(y,x)`、`el = asin(z/‖r‖)`（ECI 极坐标）；
  相对角速度 `ω = ‖v_perp‖/‖r‖`；地球遮挡/临边判定用 `ρ = asin(R_e/‖r_sat‖)`
- **对应验收**：第 5 项（探测角度）、第 6 项（最大探测距离）

### ④ 辐射与大气
- **文件**：`foundation/SbirsRadiometry.cpp`、`environment/SbirsEnvironmentModel.cpp`
- **内容**：目标辐射强度 `I [W/sr]` → 入瞳辐照度 `E = I·τ/R²`；
  透过率 `base_atmospheric_transmittance=0.8`（晴/20 km 能见度）+ 壳段气团修正 `τ_geo = τ_eff^X`
- **对应验收**：第 7 项（可探测性与动态参数）、第 58 项（场景输入）

### ⑤ 噪声与 SNR
- **文件**：`foundation/SbirsNoiseModel.cpp`、`SbirsPipeline.cpp::ComputeSnr`
- **内容**：`NEP=1e-14 W`、背景辐亮度 1.2 W/(sr·m²)、探测温度 80 K、读出噪声 2e-15 W，
  积分时间 0.1 s → 信号能量 → 线性 SNR
- **门限**：宽场 **4.0**、窄场 **5.0**（线性）
- **对应验收**：第 6/12/13 项

### ⑥ 最大探测距离 d_max
- **内容**：`d_max = min(辐射受限, 几何受限, 任务上限 50,000 km)`，只出单值
- **对应验收**：第 6 项

### ⑦ 宽场（WFOV）检测四门
- **内容**：① 瞬时视场（传感器系矩形 24°×24°）② 地球遮挡/临边 ③ SNR ≥ 4.0 ④ 驻留时间 ≥ 积分时间
- **输出**：候选累积、`首次探测` 事件、覆盖区与驻留时间（第 12/14 项）
- **对应验收**：第 12 项、第 14 项（宽窄联合）

### ⑧ 窄场调度与指向
- **文件**：`pipeline/SbirsNfovScheduler.cpp`、`SbirsPointingCoordinator.cpp`、
  `SbirsPointingActuator.cpp`、`SbirsPointingDisturbance.cpp`
- **内容**：单镜筒轮转（每周期服务一个锁定目标；转动期间帧作废）；
  指向扰动 = 共模姿态（σ=0.0005°，相关时间 1 s，ORN 过程）+ 通道指向（0.0003°）+ 振动（0.0001°@0.3 Hz）
- **对应验收**：第 13/14 项、第 76 项（宽场候选与窄场调度接口）

### ⑨ 窄场捕获
- **文件**：`pipeline/SbirsNfovAcquisition.cpp`、`SbirsPipeline.cpp`「首次捕获尝试」段
- **内容**：转向 → 稳定（容差 0.01°）→ 帧门（几何门 + SNR 门 5.0）；
  失败原因归因：视场外 / SNR 不足 / 引导超时 / NIS 失锁
- **对应验收**：第 13 项

### ⑩ 窄场跟踪与滤波
- **文件**：`pipeline/SbirsTrackingCoordinator.cpp`
- **模型**：6 维 ECI 恒速 `[x,vx,y,vy,z,vz]` + 2 维角度量测的 **EKF**（可切 IMM / 角度 CV-KF）
- **公式**：
  ```text
  P⁻ = F P Fᵀ + Q ,  Q_轴 = q·[[dt³/3, dt²/2],[dt²/2, dt]] , q = 20.0
  K  = P⁻Hᵀ(HP⁻Hᵀ + R)⁻¹ ,  P⁺ = (I − K H) P⁻
  R  = diag(σ_az², σ_el²)，σ 由 5 项 RSS（见⑪）
  ```
- **输出**：后验位置/协方差（第 13 项 `跟踪位置误差` / `跟踪位置协方差1σ` / `位置误差/σ`）、NIS 与门结果
- **对应验收**：第 13/77 项

### ⑪ 测角误差模型
- **文件**：`foundation/SbirsErrorModel.cpp`、`tracking/SbirsTrackingTypes.h:148-183`
- **公式**：
  ```text
  σ = √(σ_轨道² + σ_姿态² + σ_视场² + σ_折射² + σ_滞后²)
  σ_折射 = 1.5e-6 /(R·|cos el|) ,  σ_滞后 = ω/(2πB) ,  B=100 Hz
  或（当前默认）线性 SNR 标定：35 dB→1.0 µrad、10 dB→5.0 µrad
  ```
- **对应验收**：第 9 项（测角残差 ≤5″）、第 13 项（窄场 ≤10″）、第 55 项（性能）

### ⑫ 状态机与输出
- **内容**：目标状态迁移（`kStrictTruthAssistedTracking` / `kSensorLikeTruthAssistedTracking` /
  `kEstimatedTracking` / 捕获失败 / 丢锁释放），事件写入第 24/24B 项
- **对应验收**：第 16 项（接力）、第 24 项（事件）、第 25 项（分发）

---

# 第三部分：两模型的验收项 ↔ 过程对照

| 过程 | LRDR | 红外 |
|---|---|---|
| 场景/几何 | 第 28 项方向图、第 37A 地平线搜索 | 第 5 项角度、第 6 项 d_max |
| 辐射/回波 | 第 29 项回波功率 | 第 7 项可探测性、第 58 项场景输入 |
| 噪声/杂波/干扰 | 第 33 项噪声、第 30/34/35 项 | 第 6/12/13 项（噪声隐含在 SNR）|
| 信号处理增益 | 第 32 项信号增益 | —（红外无脉压；积分时间等效）|
| SNR / 检测门限 | 第 36 项 CFAR | 宽场 4.0 / 窄场 5.0（第 12/13 项）|
| 量测 | 第 38 项角度距离 | 第 9 项测角残差 |
| 滤波 | 第 42 项跟踪滤波 | 第 13 项跟踪（EKF）、第 77 项接口 |
| 特征 | 第 44/45/46/47 项 | —（红外不做特征识别）|
| 综合识别 | 第 47A 项 | 第 19 项（规模目标识别）|
| 落点/弹道 | 第 21 项（雷达侧引导）| 第 20–23/25 项 |

---

# 附：源码注释锚点怎么找

每个涉及文件头部新增了：

```cpp
/**
 * ## 过程：<阶段名>（LRDR 探测链 · 第 <n> 步）
 * - 上游：<输入从哪来>
 * - 本步：<做什么；关键公式/判据>
 * - 下游：<输出给谁>
 * - 关键参数：<配置项 + 取值来源>
 * - 对应验收项：<第 N 项 …>
 */
```

函数体内关键步骤前有 `// 过程：…` 标记；`SbirsPipeline::RunCycle` 的 `@brief` 里有一张完整的分步表。
快速检索：

```bash
grep -rn "## 过程：" src/remote_identification_radar src/sbirs_sensor
grep -rn "// 过程" src/remote_identification_radar src/sbirs_sensor
```
