# 传感器跟踪位置误差与误差协方差的计算口径（2026-11-04）

> 本文说明两件事：
> 1. 第 13 项（窄视场跟踪）新增的 `跟踪位置误差` / `跟踪位置协方差1σ` / `位置误差/σ` 从哪来；
> 2. **误差协方差是怎么算出来的**——传感器级 EKF 与融合级 UKF 两条链，逐项给出公式、参数与代码位置。

---

## 一、新增的输出字段（第 13 项 `[窄视场跟踪探测功能测试]`）

```text
仿真时间=…s 仿真周期=… [窄视场跟踪探测功能测试] 卫星ID=105 目标ID=99 …
    跟踪位置误差=1234.567m
    跟踪位置误差三轴ECI=(dx,dy,dz)m
    跟踪位置协方差1σ=(σx,σy,σz)m
    位置误差/σ=0.842
```

| 字段 | 口径 |
|---|---|
| `跟踪位置误差=` | `‖x̂ − x_true‖`，**ECI** 三轴欧氏距离（m）。`x̂` 为滤波后验位置（状态索引 0/2/4），`x_true` 取本周期场景真值 `SbirsEciSceneTarget.position_eci_m`（与 `context.azimuth_deg/elevation_deg` 同一真值源）|
| `跟踪位置误差三轴ECI=` | `(x̂−x_true, ŷ−y_true, ẑ−z_true)`（m），保留 3 位小数 |
| `跟踪位置协方差1σ=` | `(√P₀₀, √P₂₂, √P₄₄)`（m），即 EKF 后验协方差位置块对角的三轴标准差 |
| `位置误差/σ=` | `‖Δ‖ / √(σx²+σy²+σz²)`，无量纲。≈1 表示误差与滤波器自报不确定度一致；≫1 表示滤波发散或协方差偏乐观（"不诚实"）|
| 无后验时 | `跟踪位置误差=不适用(无滤波后验)`，不写 0 占位 |

**代码位置**：`src/sbirs_sensor/pipeline/SbirsPipeline.cpp`（第 13 项行拼接处，`帧数=` 之前）；
后验导出：`SbirsTrackingCoordinator::TryGetPosteriorPosition`（`src/sbirs_sensor/pipeline/SbirsTrackingCoordinator.{h,cpp}`）。

---

## 二、传感器级：红外 EKF 的协方差怎么算

### 2.1 状态与模型

| 项 | 内容 |
|---|---|
| 状态 | **6 维 ECI 恒速** `[x, vx, y, vy, z, vz]`（交错布局，位置在索引 0/2/4）|
| 量测 | **2 维球坐标角度** `[az, el]`（被动红外**不测距**）|
| 参考系 | ECI（pipeline 已把真值旋转到 ECI，滤波状态同步演化）|

来源：`src/sbirs_sensor/tracking/SbirsTrackingTypes.h:37-61`。

### 2.2 预测（时间更新）

```text
x⁻ = F · x
P⁻ = F · P · Fᵀ + Q
```

- **F（CV 转移）**：`F(0,1)=F(2,3)=F(4,5)=dt`，其余同单位阵
  （`src/common/estimation/KalmanPredictor.h:73-87`）
- **Q（过程噪声）**：连续白噪声加速度（CWNA）模型，逐轴

```text
        ⎡ dt³/3   dt²/2 ⎤
Q_轴 = q·⎢               ⎥        q = process_noise_diff_coeff
        ⎣ dt²/2    dt   ⎦
```

（`KalmanPredictor.h:93-112`；`q` 当前配置 **20.0**，见 `sbirs.json → policy.tracking.process_noise_diff_coeff`）

### 2.3 量测（观测更新）

```text
h(x) = [atan2(Δy, Δx), asin(Δz/‖Δ‖)]，Δ = x_pos − x_sat（ECI）
H    = ∂h/∂x（解析 Jacobian；速度列恒 0）
K    = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹
x⁺   = x⁻ + K (z − h(x⁻))
P⁺   = (I − K H) P⁻
```

- `h(x)` 与 `H`：`SbirsTrackingTypes.h:86-131`（`SbirsAngleMeasurementModel`）
- 更新：`src/common/estimation/EkfUpdater.h`（EKF；另有 SRIF/UDKF/UKF 实现可切）

### 2.4 R（量测噪声协方差）——**5 项合成**

`BuildMeasurementCovariance`（`SbirsTrackingTypes.h:148-183`）：

```text
σ_gauss²    = (ResolveEffectiveAngularSigmaDeg)²          ← 轨道/姿态/视场 RSS（或线性 SNR 标定）
σ_refraction = 1.5e-6 / (R · |cos(el)|)                  ← 大气折射
σ_lag        = ω_rel / (2π · B)                          ← 探测器带宽动态滞后（B = 100 Hz）
σ_az = σ_el = √(σ_gauss² + σ_refraction² + σ_lag²)
R   = diag(σ_az², σ_el²)                                 （deg² → rad²）
```

要点：
- **R 随距离/俯仰/角速度动态变化**，不是常数；
- 当前 `σ_gauss` 走"线性 SNR 标定"（35 dB→1.0 µrad、10 dB→5.0 µrad），可由
  `SetLinearSnrAngleSigmaEnabled(false)` 切回物理模型；
- 多帧时按有效帧数缩放（融合 σ = 单帧 σ/√N 口径），行内 `单帧σ=` 与 `融合σ=` 并列给出。

### 2.5 初值

```text
x₀ = [真值位置, 真值速度]                ← 以场景真值起步
P₀ = diag((1000 m)², (100 m/s)², …)     ← 三轴位置/速度同方差
```

（`SbirsTrackingCoordinator.cpp:117-136`，`initial_position_std_m=1000`、`initial_velocity_std_m_per_s=100`）

> ⚠️ 因此第 13 项的 `跟踪位置误差` 反映的是**从起始时刻起的传播/更新漂移**，
> 不是"无先验的绝对定位精度"。真正无先验的定位精度看第 7/8/15/26 项（双星交会）。

### 2.6 一致性门与发散保护

| 机制 | 行为 | 参数 |
|---|---|---|
| NIS 门 | 连续超门 → 丢弃该量测（保持预测）或释放锁定 | `nis_gate_loss_cycles`（当前 0 = 不因 NIS 失锁）|
| 窄场跟踪门 | 几何/SNR 连续失败 → 释放 NFOV 锁定 | `nfov_tracking_gate_loss_cycles = 2` |
| 发散检测 | 后验非有限/量级越界 → 丢弃状态并标记 | 行内 `跟踪状态=` 同步反映 |

### 2.7 为什么单星"位置误差"不能当定位精度

被动测角只有 2 个观测量约束 3 个位置自由度 ⇒ **沿视线方向的距离不可观测**：
- 该方向的 P 会随时间持续增长（Q 注入），`跟踪位置协方差1σ` 中沿视线分量最大；
- 误差主要由沿视线分量贡献，`位置误差/σ` 因此通常仍接近 1（说明协方差是"诚实"的，
  但并不表示位置准）。

---

## 三、融合级：宿主航迹协方差怎么算

来源：`src/fusion/FusionEngine.cpp`

| 项 | 内容 | 代码 |
|---|---|---|
| 状态 | **6 维 ECEF** `[x, vx, y, vy, z, vz]` | `filter_mean` / `filter_cov` |
| 预测 | UKF（`UnscentedPredictor<6,2>`）+ `LinearCvTransitionModel`，Q 同为 CWNA | L650-661 |
| Q 系数 | `track_process_noise`（`FusionConfig.h:33`，默认 **1.0**）| L652 |
| 位置量测 | 3 维 ECEF，**R = 各向同性** `σ²`，σ = `default_position_noise_std_m` | L775-795 |
| 方位量测 | 2 维 az/el（ENU 模型），R = `σ_deg²`，σ 来自检测的 bearing noise | L798-827 |
| 初值协方差 | `SetIsotropicCovariance(range_std, velocity_std)` | L642-643 |

更新形式（UKF）：`P⁺ = P⁻ − K S Kᵀ`，`S = P_zz + R`（sigma 点传播后的量测协方差 + R）。

**发布口径**：第 15/26 项的 `位置精度1σ=` 直接取 `sqrt(P_xx/P_yy/P_zz)` 的模长或三轴值；
它回答"滤波器自认为多准"，而第 13 项新增的 `跟踪位置误差` 回答"实际差多少"，
两者配合 `位置误差/σ` 可判协方差是否诚实。

---

## 四、两条链的关系（读日志时注意）

| 层 | 观测量 | 位置可观测性 | 误差/协方差的含义 |
|---|---|---|---|
| 传感器级（第 13 项）| 单星 az/el | **部分**（沿视线不可观测）| 滤波漂移；σ 沿视线增长 |
| 融合级（第 15/16/26 项）| 双星交会位置 + 方位 | **完整** | 真正的定位精度与不确定度 |

所以：
- 要判"模型定位准不准" → 第 7/8/15/26 项（含真值误差与 1σ）；
- 要判"滤波器是否自洽/发散" → 第 13 项新增的 `位置误差/σ`（长期 ≫3 视为发散或协方差偏乐观）；
- 要判"测角精度" → 第 9/12/13 项的 `视线角误差` 与 `线性判定`。

---

## 五、参数一览（与本文相关）

| 参数 | 值 | 来源 |
|---|---|---|
| `process_noise_diff_coeff`（红外 EKF q）| 20.0 | `sbirs.json → policy.tracking` |
| `initial_position_std_m` | 1000 m | 同上 |
| `initial_velocity_std_m_per_s` | 100 m/s | 同上 |
| `detector_bandwidth_hz`（滞后项）| 100 Hz | `sbirs.json → policy.error_model` |
| `nis_gate_loss_cycles` | 0 | 同上 |
| `nfov_tracking_gate_loss_cycles` | 2 | 同上 |
| `estimated_backend` | `kEkf` | 同上 |
| `track_process_noise`（融合 q）| 1.0 | `include/1q/fusion/FusionConfig.h:33` |
| `default_position_noise_std_m`（融合位置 R）| 见 `FusionConfig.h` | 同上 |
