# 滤波器代码位置 · 工作周期 · 弹道预测 · 卫星内部角度坐标系

> 本文回答四个问题：**滤波器代码在哪、每多久跑一次、弹道预测（落点/发射点/关机点）怎么算、
> 卫星内部方位俯仰用的是什么坐标系。**
> 所有条目给出「文件 : 行号」，行号对应 2026-11-05 的 `fosim_2013` 分支。

---

# 一、滤波器代码位置

系统里有**三套独立滤波器**，另有**一套共享的数值内核**。

## 1. 共享内核（预测/更新的数学实现）

| 内容 | 文件 | 说明 |
|---|---|---|
| CV 状态转移 F + 过程噪声 Q（CWNA）| `src/common/estimation/KalmanPredictor.h:55-137` | `F(0,1)=F(2,3)=F(4,5)=dt`；`Q_轴 = q·[[dt³/3, dt²/2],[dt²/2, dt]]` |
| EKF 预测器/更新器 | `src/common/estimation/EkfFilter.h`、`EkfUpdater.h` | 线性化 `H`、`K = P⁻Hᵀ(HP⁻Hᵀ+R)⁻¹`、`P⁺ = (I−KH)P⁻` |
| UKF 预测器/更新器 | `src/common/estimation/UnscentedPredictor.h`、`UnscentedUpdater.h` | sigma 点传播，用于 6 维非线性/线性混合 |
| IMM 混合器 | `src/common/estimation/ImmFilter.h` | 多模型概率混合 |
| 高斯状态容器 | `src/common/estimation/GaussianState.h` | `mean` / `covariance` / 维度模板 |

## 2. 融合滤波器（航迹级，宿主侧唯一的位置估计器）

| 内容 | 位置 |
|---|---|
| 状态定义（6 维 ECEF `[x,vx,y,vy,z,vz]`）| `src/fusion/FusionEngine.cpp`（`Track::filter_mean/filter_cov`）|
| **预测** | `FusionEngine.cpp:650-661` `PredictTrack()`：`UnscentedPredictor<6,2>` + `LinearCvTransitionModel<6>`，`q = config_.track_process_noise`（默认 1.0）|
| **位置量测更新** | `FusionEngine.cpp:775-796`：`UnscentedUpdater<6,3>`，`R = 各向同性 σ²`，`σ = default_position_noise_std_m` |
| **方位量测更新** | `FusionEngine.cpp:798-827`：`UnscentedUpdater<6,2>`（ENU 方位模型），`R = σ_deg²` |
| **滤波器调度/滑行** | `FusionEngine.cpp:830+` `AdvanceFilters(cycle)`：无新量测的航迹只做 predict |
| **初值协方差** | `FusionEngine.cpp:642-643` `SetIsotropicCovariance(range_std, velocity_std)` |
| **两点差分初速** | `FusionEngine.cpp:697-720`：第二个位置量测 `(p₂−p₁)/Δt`（Δt>0.05 s）建立初速 |
| 速度合理性门 | 宿主 `sbirsfusiondp_1q.cpp`：`|v| > 12000 m/s` 不发布运动学估计 |

## 3. 红外跟踪滤波器（传感器级，ECI 角度域）

| 内容 | 位置 |
|---|---|
| 状态（6 维 ECI `[x,vx,y,vy,z,vz]`）+ 2 维角度量测 | `src/sbirs_sensor/tracking/SbirsTrackingTypes.h:37-131` |
| 量测模型 `h(x)` 与 Jacobian `H` | 同上 `SbirsAngleMeasurementModel::Function/Jacobian` |
| 量测噪声 `R`（5 项 RSS）| 同上 `BuildMeasurementCovariance()`:148-183 |
| **预测/更新调度** | `src/sbirs_sensor/pipeline/SbirsTrackingCoordinator.cpp`：`PredictTarget()` / `CorrectTarget()` / `Update()` |
| IMM 后端 | 同上（`SbirsImmFilter`，`estimated_backend = kImm` 时启用）|
| 角度 CV-KF 后端 | 同上（`SbirsAngleCvGaussianState`，`kAngleCvKf`）|
| NIS 门与发散保护 | 同上 + `EkfUpdater` 的新息协方差奇异判定 |

## 4. 雷达跟踪滤波器（传感器级，量测域）

| 内容 | 位置 |
|---|---|
| CV 卡尔曼（距离/方位/俯仰/高度）| `src/remote_identification_radar/tracking/RirTrackFilter.cpp` |
| IMM 多模型 | `src/remote_identification_radar/tracking/RirImmFilter.cpp` |
| 生命周期（确认/保持/丢失/重捕）| `src/remote_identification_radar/tracking/RirTrackLifecycle.cpp` |
| 关联（6σ + LAPJV）| `src/remote_identification_radar/tracking/RirTrackAssociator.cpp` |

---

# 二、滤波器工作周期

## 1. 统一节拍

| 层 | 节拍 | 说明 |
|---|---|---|
| 仿真周期 | **1 s/拍** | 宿主 `track_cycle_period_sec = 1.0`；周期号即秒数 |
| 融合滤波 | **每拍 1 次** | 宿主 `sbirsfusiondp_1q.cpp:1383` 每拍调 `engine->Update(fusion_records, cycle)` |
| 红外跟踪 | **每拍 1 次/锁定目标** | `SbirsPipeline::RunCycle` 内逐锁定目标 `Update()`；`dt = input.dt_sec`（=1 s）|
| 雷达跟踪 | **每拍 1 次/航迹** | `RirSession::Step` → `RirTrackLifecycle` → `RirTrackFilter::Predict/Update` |
| 精度评估推演 | 每拍一次 `Step`，推演按 `inference_interval_cycles`（默认 1）| `PrecisionEvaluationSession::Step` |
| 汇总类验收行 | **每 300 拍** | 第 26/27 项、安装矩阵重发、3 µrad 最优保持 |

## 2. 每拍内的执行顺序（融合侧）

```text
① 收帧（含多拍积压队列）→ ② 组 fusion_records（位置量测 / 方位量测）
→ ③ engine->Update(records, cycle)：
      对每条已有航迹 AdvanceFilters(cycle)：dt = cycle − filter_cycle，先 PredictTrack
      再逐量测 ApplyFilterSample：有位置→UKF(6,3) 更新；仅方位→UKF(6,2) 更新
      无任何量测→只保留预测（滑行）
→ ④ 发布 FusedTarget（位置/速度/是否已建速/协方差）
```

## 3. 时间步的来源（不是常数写死）

- 融合：`dt = (cycle − track->filter_cycle) × track_cycle_period_sec`（跨拍不丢时间）；
- 红外：`input.dt_sec`（每拍 1 s；帧率 10 Hz 只用于帧数/驻留换算，不改变滤波步长）；
- 雷达：`input.dt_sec`。

## 4. 与验收行的时间关系

- 滤波每拍都在跑，但**验收行不是每拍都写**：第 26/27 项 300 拍一次、角误差行每拍、落点行每拍（有解时）；
- 因此"日志里某拍没有位置行"≠"滤波器没工作"，可能只是该拍没有位置量测（走预测）。

---

# 三、弹道预测怎么做的（落点 / 发射点 / 关机点）

实现文件：`src/target_inference/TargetInferenceEngine.cpp`（510 行）+ 配置 `include/1q/target_inference/TargetInferenceConfig.h`。

## 1. 输入与输出

```cpp
// 输入（每条航迹）
struct InferenceTrackState { key; position(ECEF); velocity(ECEF); covariance; sim_time_sec; }
// 输出
struct TrajectoryPrediction {
  bool valid; waypoints[];                  // 每 waypoint_interval_sec 一个
  bool has_impact;  LlaPositionDegM impact_point;  double impact_time_offset_sec;
  bool has_launch;  LlaPositionDegM launch_point;
  bool has_burnout;  ...                    // 关机点
  double impact_position_sigma_m;            // 落点 1-σ
}
```

## 2. 动力学模型

| 项 | 口径 |
|---|---|
| 主项 | **二体引力** `a = −μ·r/‖r‖³` |
| 可选 | **指数大气阻力**（`drag` 开关，按高度分层密度）|
| 积分器 | **定步长 RK4**，`dt = integration_step_sec`（默认 1 s，最小 0.01 s）|
| 时域 | `prediction_horizon_sec`（库默认 **300 s**；宿主放开到 **1800 s**）|
| 停止条件 | 位置模长 ≤ 地球半径（球面地表）、状态非有限、超时域 |

## 3. 前向预测 + 落点（`TargetInferenceEngine.cpp:196-244`）

```text
state = 航迹状态
while t < horizon:
    next = Rk4Step(state, +dt)
    if ‖next.r‖ ≤ R_earth:                       # 穿越地表
        frac = (‖state.r‖ − R_e) / (‖state.r‖ − ‖next.r‖)   # 线性插值比例
        impact = state + frac·(next − state)                # 交点状态
        impact_lla = ToLla(impact); impact_lla.altitude = 0 # 交付口径归零
        has_impact = true; break
    if t+dt ≥ next_waypoint: 记录 waypoint（每 waypoint_interval_sec）
    state = next; t += dt
```
要点：
- **线性插值**求地表交点（步长内视为匀速直线），不是步进到落地那一拍；
- 落点高度强制 **0 m**（球面地表模型，避免 WGS84 椭球高与球面半径混用的 +5 km 伪高差）。

## 4. 发射点回推（`:246-300`）

```text
从当前状态反向积分：next = Rk4Step(state, −dt)
停止门（任一满足）：
  ① 位置模长 ≤ R_earth        → 命中地表（发射点）
  ② 高度 ≥ 大气界面（100 km） → 助推段上界估计（助推段本身不建模）
  ③ 速度方向翻转/速度过小      → 停机
```
⇒ 回推得到的点就是"发射点估计"；`launch_max_backtrack_sec`（宿主 **1800 s**）为防止无限回推的上限。

## 5. 关机点（burnout）

- 由**能量/加速度状态机**判定助推段结束，再用**回溯**给关机时刻/位置估计；
- 未解出时第 23 项写 `关机时刻=不适用(尚未解算出关机点)`（不写 0 占位）。

## 6. 落点误差 1σ

```text
impact_position_sigma_m ← 由当前状态协方差沿"状态→落点"的敏度线性传播
                           （等价于把 P 投影到落点位置雅可比 J 上：σ² = J·P·Jᵀ）
```
第 20 项写误差椭圆半长/半短/方位，第 25 项写落点误差范围与置信度，第 21 项写**实测落点误差**
（估计落点 vs 真值落点，同一引擎两状态各自解算后求三维距离）。

## 7. 候选门（哪些航迹才进推演）

宿主 `sbirsfusiondp_1q.cpp:1908-1930`：

| 门 | 阈值 |
|---|---|
| 已建立速度估计 | `has_velocity_estimate` |
| 最近双星交会新鲜 | 最近 **30 拍**内有交会解（`kFixFreshCycles=30`）|
| 真值高度门（空间目标剔除）| 真值高度 > **10,000 km** 不推演 |
| 真实量测更新 | `updated_by_real_measurement` |
| 速度合理性 | `|v| ≤ 12,000 m/s` |

---

# 四、卫星内部方位/俯仰所用的坐标系

## 1. 对外（量测、真值、滤波、验收行）= **ECI 极坐标**

```cpp
// src/sbirs_sensor/foundation/SbirsGeometry.cpp:50-60
az = atan2(los.y, los.x)      // 自 ECI +X 轴量向 +Y，范围 (−180°, 180°]
el = asin(los.z / ‖los‖)      // 自 ECI 赤道面向上为正，范围 [−90°, +90°]
```
- 参考系：**J2000 平赤道惯性系近似** —— 仅绕 z 轴按 GMST 旋转，忽略岁差/章动/极移
  （`include/1q/coordinate/inertial_transform.h:3-14`）；
- `los = r_target − r_satellite`，两者都在 ECI；
- 单位：滤波内部为 **rad**；`ComputeAzimuthDeg` 等几何函数输出 **deg**；日志 `(ECI)` 行写 **rad**。

## 2. 对内（视场门、扫描栅格、指向/ATP、第 76 项）= **传感器系**

```cpp
// src/common/geometry/BoresightChain.cpp:83-96
先旋到本体：姿态(ECI→Body) → 安装角(Body→Sensor) → 失准角
再取 az = atan2(y_s, x_s)、el = asin(z_s/‖·‖)
```
日志里标 `(传感器系)` 的字段（如 `实际指向方位/俯仰(传感器系)=`）用的是这一套，单位 deg。

## 3. 扫描方位还有第三层语义（只影响"起点"）

| 配置 | 语义 |
|---|---|
| `kEciAbsolute` | `scan_start_az_deg` = ECI 绝对方位 |
| `kNadirRelative`（当前 `sbirs.json` 用的）| 起点 = **相对星下点方位**的带符号偏移；星下点方位 = `atan2(−y, −x)`（GEO 恒定）|

## 4. 容易踩的三个坑

1. **俯仰的参考面是赤道面**，不是天底。GEO 下两者重合（星下点在赤道面内），倾斜轨道下不等价；
2. **方位有两套取值范围**：`atan2` 给 (−180°,180°]，扫描输出用 `WrapAzimuthPositive` 给 [0°,360°)；
   比对角必须用最短角差 `AzimuthDelta`，不能直接相减；
3. **与雷达不要混**：雷达用 ENU 方位（正北起、顺时针；内部还有 `90° − ENU 方位` 换算），
   红外是 ECI 极坐标（+X 起、逆时针向 +Y）。

---

# 五、速查表

| 想找 | 去哪 |
|---|---|
| 融合滤波器（航迹 UKF）| `src/fusion/FusionEngine.cpp:650-830` |
| 红外 EKF（ECI 角度域）| `src/sbirs_sensor/pipeline/SbirsTrackingCoordinator.cpp` + `tracking/SbirsTrackingTypes.h` |
| 雷达 KF/IMM | `src/remote_identification_radar/tracking/RirTrackFilter.cpp`、`RirImmFilter.cpp` |
| 滤波数学内核（F/Q/K/P）| `src/common/estimation/KalmanPredictor.h`、`EkfFilter.h`、`UnscentedUpdater.h` |
| 弹道预测（落点/发射点/关机点）| `src/target_inference/TargetInferenceEngine.cpp:175-300` |
| 推演配置（时域/步长/回推上限）| `include/1q/target_inference/TargetInferenceConfig.h:27-30` |
| 红外方位/俯仰定义 | `src/sbirs_sensor/foundation/SbirsGeometry.cpp:50-60` |
| ECI 参考系约定 | `include/1q/coordinate/inertial_transform.h:3-14` |
| 传感器系旋转链 | `src/common/geometry/BoresightChain.cpp:83-96`、`src/sbirs_sensor/pipeline/SbirsBoresightChain.cpp` |
