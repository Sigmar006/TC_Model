---
Status: active
Last-reviewed: 2026-09-03
Authority: SBIRS 算法登记与实现边界
Answers: SBIRS 用了哪些算法、各自实现到什么地步、边界在哪、哪些反直觉、哪些刻意不实现
---

# SBIRS 算法登记

本文是天基红外系统（Space-Based Infrared System, SBIRS）模块算法与部件清单及实现边界的权威。算法本身的逐步代码逻辑读 `src/sbirs_sensor/`；本文回答“用没用 / 到哪步 / 边界在哪 / 哪些反直觉 / 哪些刻意不实现”。模块级边界（输出归属、电源、能力决策）见 [boundaries.md](boundaries.md)，数据流与时序见 [data-flow.md](data-flow.md)。

SBIRS 模拟天基红外监视卫星的宽视场（WFOV）扫描探测、星下点/惯性稳定指向、单镜筒分时轮转窄视场（NFOV）精跟、星间交叉引导（Cross-Cue）以及 OPIR 验收日志派生。

---

## 算法登记表

| 算法/部件 | 意图 / 核心转换 | 实现状态 | 证据与单测 |
|---|---|---|---|
| ECEF $\to$ ECI 旋转（GMST） | UTC 儒略日算 GMST，卫星/目标位置及含自转输运速度转 ECI | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_eci_transform_test.cpp] |
| 地球遮挡门控 | 有限线段射线与球体求交，剔除穿地视线（不入 WFOV/SNR） | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_foundation_test.cpp] |
| 公共安装指向与稳定链 | 姿态(Body$\to$ECI) $\circ$ 安装 $\circ$ 扫描合成光轴；体/惯性双稳定 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_boresight_chain_test.cpp] |
| 安装失准误差模型 | 运行内常值微扰合成入光轴链；与时变指向扰动和量测白噪谱正交 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_boresight_chain_test.cpp] |
| Foundation 标量辐射传输 | 辐射强度 $I$ + 几何距离 $\to$ 接收功率 $P_{\text{sig}}$ 与标量链路 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_foundation_test.cpp] |
| 现实噪声能量分母口径 | 积分时间内噪声能量分母，NEP + 像元 IFOV 背景光子起伏 RSS | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_noise_model_test.cpp] |
| 壳段气团几何大气衰减 | 视线穿球对称大气壳（100 km 壳顶）弦长比 $\to \tau_{geo} = \tau_{\text{eff}}^X$ | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_environment_model_test.cpp] |
| 当前时刻最大探测距离 | WFOV 门限闭式反解 $d_{\max}(t)$，逐目标随气团与快照变化进归属 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_radiative_transfer_test.cpp] |
| WFOV 宽视场多目标搜索 | 推进扫描相位，执行几何/SNR 门控，输出带误差观测位置 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test.cpp] |
| 星下点相对与往复栅格扫描 | nadir 相对基准 + 2-D 俯仰网格往复牛耕推进（到边反射折叠） | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test.cpp] |
| 观测误差与时变指向扰动 | 5 类量测误差（RSS合成）+ 一阶高斯-马尔可夫实际光轴抖动 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_error_model_test.cpp] |
| 单镜筒分时轮转与无界集合 | 窄场单执行器轮转服务；同帧免费多跟；精跟容量由物理涌现 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_scheduler_test.cpp] |
| 单镜筒 ATP 光轴动力学 | 球面最短路径限速推进，依角转速折算当拍稳定时长与有效帧数 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_pointing_coordinator_test.cpp] |
| NFOV 高刷新率多帧融合 | ATP 帧数 $N$ 的独立角误差抽样均值 $\to$ 融合量测（随机 1-σ 按 $1/\sqrt{N}$ 收敛） | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_error_model_test.cpp] |
| 星间 Cross-Cue 交叉提示 | 三角化他星宽场测角/距离后注入本星调度池，闭合双星时间断链 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test.cpp] |
| NFOV 首次捕获 | WFOV cue 两点 CV 提前量外推，延迟视线地心/卫星位移评估 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_cue_predictor_test.cpp] |
| 目标 7 状态机 | Undetected $\to$ Wide $\to$ Awaiting $\to$ 3类跟踪 $\to$ Lost 转移闭环 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_state_machine_test.cpp] |
| NFOV 持续闭环跟踪 | 统一闭环：命令 LOS $\to$ ATP 推进 $\to$ 窗口/SNR 门 $\to$ 滤波更新 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test.cpp] |
| 6D ECI CV 状态/2D 测角 EKF | 6 维 ECI 状态 / 纯 2 维角度量测卡尔曼滤波，被动红外不测距 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_ekf_baseline_test.cpp] |
| IMM(EKF) 交互多模型滤波 | 多模型自适应交互，机动目标全场景 RMSE 显著改善 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_imm_evaluation_test.cpp] |
| 角度域线性 KF（实验后端） | 4 维角度及变化率线性卡尔曼滤波（$kAngleCvKf$，显式选配） | experimental | [evidence: tests/unit/sbirs_sensor/sbirs_angle_cv_kf_test.cpp] |
| 连续命中门控计数 | 连续通过 WFOV 四门计数达标方可入 NFOV，进跟踪后清零 | session-wired | [evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test.cpp] |
| OPIR 验收派生量输出 | 视场∩地球盘交集经纬度包络映射 4 角（2026-09-06 裁剪包络化）、驻留时间、脱靶量与角误差写入验收日志 | internal/受控 | [evidence: tests/unit/sbirs_sensor/sbirs_foundation_test.cpp]、[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test.cpp] |

实现状态说明：
- **session-wired**：已接入 `SbirsPipeline` / `SbirsSession` 主链路，覆盖配置、执行周期、重放与集成。
- **experimental**：可编译、有单元测试覆盖，显式选配且默认不启用，尚未完成全场景标定。
- **internal/受控**：库内私有支撑能力或仅验收日志通道消费。

---

## 核心算法详述

## 1. 几何前序与安装指向合成

### 1.1 ECEF $\to$ ECI 惯性旋转（GMST）
- **算法意图与调用时机**：SBIRS 统一在 ECI 惯性参考系中演化与输出（避免地球自转引起的科里奥利与离心加速度虚假耦合）。在每周期入口由 `SbirsPipeline` 读取 UTC 儒略日并执行坐标与速度变换。
- **数学与物理模型**：
  - 格林尼治平恒星时（GMST，IAU 1982 近似 / Vallado 式 3-47 的度数形式，与代码逐项一致）：
    $$\theta_{\text{GMST}} = 280.46061837 + 360.98564736629 \cdot d + 0.000387933 \cdot T^2 - \frac{T^3}{38710000} \pmod{360^\circ}$$
    其中 $d$ 为 J2000 历元（2451545.0）起的天数，$T = d / 36525$ 为儒略世纪数。
  - 位置旋转：$\mathbf{r}_{\text{ECI}} = R_z(\theta_{\text{GMST}}) \mathbf{r}_{\text{ECEF}}$。
  - 速度变换（含自转输运项）：
    $$\mathbf{v}_{\text{ECI}} = R_z(\theta_{\text{GMST}}) \mathbf{v}_{\text{ECEF}} + \boldsymbol{\omega}_e \times \mathbf{r}_{\text{ECI}}$$
    其中地面静止目标在赤道处的 ECI 输运速度达 $\sim 465\text{ m/s}$，不可忽略。
- **实现边界**：变换保模长，球体相交与遮挡距离判定与帧无关；输出角度为 ECI 极坐标弧度（$\text{az} \in [0, 2\pi), \text{el} \in [-\pi/2, \pi/2]$）；共享域 API 位于 `include/1q/coordinate/inertial_transform.h`（仅 SBIRS 消费）。
- **边界保护**：儒略日非法（非有限或 $\le 0$）时变换返回失败，不产生坐标。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_eci_transform_test.cpp]

---

### 1.2 地球遮挡门控 (Earth Occultation)
- **算法意图与调用时机**：视线穿过地球时目标不可见。排在距离、视场与 SNR 门之前，短路穿地视线。
- **数学与物理模型**：有限线段与地球均值圆球（$R_e = 6371\text{ km}$）求交。判定核委托公共域 `oneq::common::geometry::IsEarthOcculted`。
- **实现边界**：纯几何通视判定，不含低空云层、大气折射弯曲与 DEM 地形。全链统一使用 ECI 几何坐标。不存在低空硬排除门——低空路径损耗只经透过率进 SNR，不在此处拦截。
- **边界保护**：退化几何（线段外最近点）视为无遮挡——对地目标不因线段端点判交而误判。
- **反直觉点**：**相切（余量 $= 0$）即算遮挡**——擦地视线被判不可见，排障时优先核对遮挡余量符号（负值 = 穿深）。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_foundation_test.cpp]、[evidence: tests/unit/common/common_earth_occultation_test.cpp]

---

### 1.3 公共域安装指向与稳定链 (Boresight Chain)
- **算法意图与调用时机**：将卫星本体姿态、传感器机械安装角与扫描/跟踪指向合成为 ECI 下的实际物理光轴。
- **数学与物理模型**：
  $$\mathbf{u}_{\text{boresight}} = R_{\text{Body}\to\text{ECI}}(\text{attitude}) \circ R_{\text{Sensor}\to\text{Body}}(\text{mount}) \circ \mathbf{u}_{\text{sensor}}(\text{scan/track})$$
  纯几何内核委托公共单源 `oneq::common::geometry::BoresightChain`（按 `ScanScheduleRuntime.h` 先例于 2026-08-20 收敛提取；ESR 2026-08-21 经 `EsrBoresightChain` 接入——安装偏置取反入链，EOS 经 `EosLookAngles` 委托仅姿态链）。**identity 链回归承诺**：零姿态 + 零安装角 + 零失准下全部数值与提取前历史逐位一致。
- **稳定方式**：
  - `kBodyStabilized`（默认）：扫描角在传感器系定义，姿态转动带动光轴足迹漂移。
  - `kInertialStabilized`：扫描角在 ECI 惯性系定义，反解到传感器系施加转向（$R^T$ 旋转），物理保持惯性指向固定。
- **反直觉点（参考系分化）**：门控在传感器系执行（受物理限位约束），输出报告在 ECI 惯性系（外部接口协议一致性）。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_boresight_chain_test.cpp]、[evidence: tests/unit/common/common_boresight_chain_test.cpp]

---

### 1.4 安装失准误差模型 (Misalignment Error Model)
- **算法意图与调用时机**：模拟静态安装失准（常值偏置 + 随机抽取），作用于光轴链内部。
- **数学与物理模型**：
  $$\mathbf{u}_{\text{actual}} = R_{\text{Body}\to\text{ECI}}(\text{attitude}) \circ R_{\text{Sensor}\to\text{Body}}(\text{mount}) \circ R(\text{misalignment})^{-1} \circ \mathbf{u}(\text{scan})$$
  失准由常值偏置 $\text{bias}$ 与一次性随机微扰 $\mathcal{N}(0, \sigma_{\text{misal}}^2)$ 构成。
- **谱正交设计（杜绝双重计模）**：

| 误差域 | 时间结构 | 作用点 | 流/种子 |
|---|---|---|---|
| 量测域（orbit/attitude/fov/range sigma） | 每周期白噪声重抽 | 量测输出 | `error_model.random_seed` 派生 3 流 |
| 指向扰动（共模/通道） | 时变 GM（tau>0）+ 确定性振动 | 传感器系实际指向（加法叠加） | `pointing_disturbance.random_seed` 派生 |
| 安装失准 | 运行内常值（一次抽取） | boresight 链合成（旋转合成） | `orientation.misalignment.random_seed` |

- **实现边界**：失准在 pipeline 构造/ApplyConfig 时抽取一次，运行内为常值；不进量测协方差 `BuildMeasurementCovariance`；默认全零 = 既有行为逐位不变。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_boresight_chain_test.cpp]

---

## 2. 探测物理链与环境模型

### 2.1 标量红外辐射传输与现实噪声模型 (Radiative Transfer & Noise Model)
- **算法意图与调用时机**：由目标红外辐射强度 $I_t$（W/sr）计算光学孔径接收功率与噪声能量，求解检测 SNR。由 `foundation` 命名空间自由函数（`SbirsRadiometry` / `SbirsNoiseModel`，如 `ComputeReceivedPowerW` / `ResolveEffectiveNoiseW`）驱动。
- **数学与物理模型**：
  1. 信号接收功率：
     $$P_{\text{sig}} = \frac{I_t \cdot A_{\text{ap}} \cdot \tau_{\text{opt}} \cdot \tau_{geo} \cdot \eta}{d^2}$$
  2. **现实噪声能量分母口径（2026-09-02 契约）**：
     $$\text{SNR} = \frac{P_{\text{sig}} \cdot t_{\text{int}}}{N_{\text{eff}}}$$
     $$N_{\text{eff}} = \sqrt{ (NEP \cdot t_{\text{int}})^2 + N_{\text{photon}}^2 + N_{\text{thermal}}^2 + (N_{\text{readout}} \cdot t_{\text{int}})^2 }$$
     其中背景光子起伏能量 $N_{\text{photon}} = \sqrt{P_{\text{bg}} \cdot t_{\text{int}} \cdot E_{\text{ph}}}$，
     像元视场接收背景功率 $P_{\text{bg}} = L_{\text{bg}} \cdot A_{\text{ap}} \cdot \tau_{\text{opt}} \cdot \Omega_{\text{pixel}}$，像元立体角 $\Omega_{\text{pixel}} = (d_{\text{pitch}} / f)^2$，单光子能量 $E_{\text{ph}} = hc / \lambda_{\text{center}}$。
- **实现边界与工程简化**：背景辐射亮度默认 $2.0\text{ W/(sr}\cdot\text{m}^2\text{)}$（典型地球 MWIR 背景）；探测器热项仅在温度 $>0\text{ K}$ 时激活。标量链路不开展二维 PSF 衍射卷积。
- **边界保护**：全部分量退化（NEP=0 且背景/热/读出全关，或积分时间 0）时，分母回退 NEP 标量下限 $\max(10^{-18}, \text{NEP})$，保证分母恒正。
- **反直觉点**：废弃了历史 1 sr 全向背景视场口径（历史曾导致 SNR 虚低 4 个数量级），改用精确单个像元 IFOV 计算背景光子起伏。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_noise_model_test.cpp]

---

### 2.2 壳段气团几何大气衰减 (Shell-Segment Airmass Attenuation)
- **算法意图与调用时机**：计算斜穿大气层时的有效几何透过率，在 `SbirsEnvironmentModel` 中实现。
- **数学与物理模型**：
  - 气团因子 $X$：视线在球对称大气壳（地表至 100 km 壳顶）内的穿壳弦长 $L_{\text{chord}}$ 与垂直壳厚 $H_{\text{shell}}$ 之比：
    $$X = \text{clamp}\left( \frac{L_{\text{chord}}}{H_{\text{shell}}}, 0, 10 \right)$$
  - 几何路径透过率：
    $$\tau_{geo} = \tau_{\text{eff}}^X$$
    其中 $\tau_{\text{eff}} = \tau_{\text{base}} \cdot (1 - A_{\text{total}})$ 为经天气/温湿度折减后的基准垂直透过率。
- **边界行为**：纯空间视线（两端均在 100 km 壳顶外且不穿壳）$X = 0 \implies \tau_{geo} = 1.0$；地面目标垂直天顶观测 $X = 1 \implies \tau_{geo} = \tau_{\text{eff}}$。穿地路径由遮挡门先行阻断。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_environment_model_test.cpp]

---

### 2.3 当前时刻最大探测距离 (Maximum Detection Range)
- **算法意图与调用时机**：根据本周期快照逆解探测距离极限，随归属帧输出供诊断。
- **闭式解公式**：
  $$d_{\max}(t) = \sqrt{ \frac{I_t \cdot A_{\text{ap}} \cdot \tau_{\text{opt}} \cdot \tau_{geo} \cdot \eta \cdot t_{\text{int}}}{N_{\text{eff}} \cdot \text{SNR}_{\text{th}}} }$$
- **实现边界**：因 $\tau_{geo}$ 与 $N_{\text{eff}}$ 逐目标、逐周期变化，$d_{\max}(t)$ 呈现动态响应。
- **边界保护**：任一输入（辐射强度、孔径、透过率、积分时间等）非正或非法时返回 0，不产生虚高距离。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_radiative_transfer_test.cpp]

---

## 3. 宽视场多目标搜索与指向扰动

### 3.1 WFOV 扫描搜索与星下点相对基准
- **算法意图与调用时机**：每周期常数时间推进宽场扫描相位，执行多目标捕获判断。
- **几何门控顺序（代码实际顺序）**：地球遮挡 $\to$ 距离门（min/max range）$\to$ WFOV 视场门 $\to$ SNR 门。穿地/超距/出视场/弱信号逐门短路并写排除诊断。
- **扫描模式**：
  - **方位基准**：`kEciAbsolute`（ECI 绝对角）与 `kNadirRelative`（相对星下点方位 atan2(-y, -x) 的动态偏移）。nadir 模式每周期按「配置扇区 $\cap$ 地球可见窗」裁剪有效跨度（可见窗 = 星下点方位 $\pm(\arcsin(R/\lvert r\rvert) + \text{wfov}_{az}/2)$），相位原点恒为配置起点、只裁远端。
  - **往复栅格推进**：2-D 栅格采用往复牛耕式折返（到边反射），消除锯齿大跳变；单行步进由 `scan_direction` 初始方向动态翻转。
  - **初始扫描相位与多星异步**：任务配置支持 `initial_scan_phase_deg`（范围 $[0, \text{span})$ deg，默认 0.0f）；多星部署时赋予不同初始相位，实现各星错峰巡视，消除“同相广播体操”伪对称。
  - **波束穿越时刻反解与量测时间戳**：往复扫描时，波束中心在当周期内扫过目标视线方位的时刻由扫描角速度闭式反解：
    $$\Delta t = \text{clamp}\left( \frac{\lvert az_{\text{target}} - az_{\text{start}} \rvert}{\omega_{\text{scan}}}, 0, dt \right)$$
    输出绝对物理量测时刻 $t_{\text{meas}} = t_{\text{cycle\_start}} + \Delta t$ 进 `SbirsDetectionRecord.measurement_time_s`。
  - **管线内部不外推原则（方案 A 否决）**：传感器管线如实输出波束穿越瞬间的观测视线与时间戳，严禁在管线内依速度平移目标 ECI 位置（ECI 系内地转速度使静止目标失真，且与探测器动态滞后重叠）；多星异步时刻（如 A 星 0.5s vs B 星 0.8s）引起的空间位移补偿由下游融合滤波状态方程承担（FUSION-OQ-1）。
- **实现边界与工程简化**：
  - 扫描相位用 `std::fmod` 常数时间推进；`span=360°` 跨 ±180° 不产生数轴断点。
  - `kWideSearch` 只产生 channel=`-1` 的 WFOV 观测，不分配 NFOV；WFOV 带误差位置是仿真观测/cue，不是目标真值。
- **反直觉点（扇区 patch 的相位处理）**：扇区 patch 后，当前绝对方位仍位于新有向半开区间时重算 phase 并保持指向（腿方向保留），否则 phase 归零转到新起点且腿复位初始方向；2-D 栅格 patch 下旧行中心 el 映射到新栅格最近行；从 SearchAndStare 切入时释放 NFOV/pointing/filter 绑定但**保留** scan phase、测量随机流与已有 WFOV cue 历史。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test.cpp::WfovCrossingTimeCalculatesSubCycleMeasurementTimestamp]、[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test.cpp::InitialScanPhaseOffsetsSweep]

---

### 3.2 观测误差模型与指向抖动 (Error & Disturbance Model)
- **误差合成**：轨道、姿态与视场三项高斯误差按 RSS 合成，再与折射、动态滞后两项确定性误差 RSS 进量测协方差：
  $$\sigma_{\text{angle}} = \sqrt{\sigma_{\text{orbit}}^2 + \sigma_{\text{attitude}}^2 + \sigma_{\text{fov}}^2 + \sigma_{\text{refraction}}^2 + \sigma_{\text{lag}}^2}$$
- **动态滞后**：随相对视线角速度产生确定性滞后偏差，角速度取视线垂直分量投影 $\omega = \lvert \mathbf{v}_\perp \rvert / R$（目标速度未提供时取 0、卫星速度必填；卫星运动本身扫过视场也产生滞后）。
- **指向扰动**：共模项（WFOV/NFOV 同步抖动）与通道项（NFOV 单镜筒抖动）通过一阶 Gauss-Markov 过程推进（相关时间 $\tau > 0$），叠加通道确定性振动项，作用于传感器系物理指向。单镜筒下通道扰动仅 0 号一份；**全部幅值默认 0**（无可追溯设备参数时不提供仓库级非零"真实 SBIRS"常数）；无目标时仍随仿真时间推进，普通 release/rebind 与无关字段 patch 不重置。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_error_model_test.cpp]、[evidence: tests/unit/sbirs_sensor/sbirs_pointing_disturbance_test.cpp]

---

## 4. 窄视场执行、调度与多星引导

### 4.1 单镜筒分时轮转与无界锁定集合 (Single-Telescope NFOV Scheduler)
- **算法意图与调用时机**：窄视场全星共享唯一光学镜筒。彻底废除 `max_concurrent_nfov_locks`，锁定集合无容量上限。由 `SbirsNfovScheduler` 仲裁。
- **调度机制**：
  1. 轮转推进：每周期固定服务集合中的一个目标：$\text{idx} = \text{rotation\_step} \pmod{\text{size}}$，步长单调递增。
  2. **同帧免费多跟**：镜筒对准当前服务目标时，所有碰巧落入当前瞬时窄视场内的其他已锁定目标，**共享本周期的稳定测量帧**，帧数不发生摊薄。
  3. 视场内候选免转动捕获：处于等待捕获状态的目标若已在当前瞬时视场内，直接执行捕获判决，无需镜筒重定向。
  4. 物理容量涌现：分离目标在轮空周期记为门失败，连续轮空超过 `nfov_tracking_gate_loss_cycles`（默认 2）自动丢锁释放。
- **反直觉点**：并发精跟目标数不是配置出来的，而是由执行器摆速、目标角间距与丢锁周期数共同涌现的物理结果。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_scheduler_test.cpp]

---

### 4.2 单镜筒 ATP 光轴动力学 (Pointing Actuator Dynamics)
- **算法意图与调用时机**：`SbirsPointingActuator` 模拟执行器沿球面最短大圆路径以最大角速度转向。
- **稳定时长与帧数折算**：
  $$\Delta\theta = \arccos(\mathbf{u}_{\text{current}} \cdot \mathbf{u}_{\text{cmd}})$$
  $$t_{\text{settled}} = \max\left(0.0,\; dt - \frac{\max(0.0, \Delta\theta - \theta_{\text{tol}})}{\omega_{\max}}\right)$$
  $$\text{Frames} = \text{round}(f_{\text{frame}} \cdot t_{\text{settled}})$$
  一步转不到位时，$t_{\text{settled}} = 0$，本周期有效测量帧数为 0。
- **NFOV 高刷新率多帧融合（2026-08-31 架构裁定：宽场只引导 / 窄场多帧融合 / 双星定位仅窄场）**：当周期有效帧数 $N > 1$ 时，$N$ 帧独立角误差抽样取均值融合为单周期量测，随机分量按 $1/\sqrt{N}$ 收敛：
  $$\sigma_{\text{fused}} = \frac{\sigma_{\text{frame}}}{\sqrt{N}}$$
  方位走最短角差累加（防 0°/360° 环绕）；折射/动态滞后为确定性公共偏差，**不参与**衰减；距离乘法误差保持单次抽样口径（融合只针对角量测）。
- **边界保护**：帧数 $\le 1$ 时退化为单帧路径（行为与既有口径逐位一致）；帧数为 0 时融合量测为空。
- **反直觉点**：多跟目标共享同一批稳定帧时，各目标按自己的 $N$ 独立融合——帧数是逐目标的稳定时长折算结果，不是全局常数。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_pointing_coordinator_test.cpp]、[evidence: tests/unit/sbirs_sensor/sbirs_error_model_test.cpp]

---

### 4.3 星间 Cross-Cue 交叉提示 (Cross-Cue Triangulation)
- **算法意图与调用时机**：解决大椭圆/高轨下双星无法在窄视场中同时搜索到弱小目标的断链问题。由 `SbirsSession::SubmitExternalCue` 注入。
- **几何三角化解算**：
  受话星接收来源星测角视线与测距结果，利用来源星 ECEF 位置推导目标空间三维坐标，再反解受话星本星视线角，直接将目标作为外部候选注入本机调度池。
- **拓扑与口径**：星 $\to$ 地 $\to$ 星（地面站存储转发，固定滞后一周期）；递话内容为来源星宽场量测，由编排层经 `SbirsSession::SubmitExternalCue` 运行期注入。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test.cpp]

---

## 5. 目标状态机与闭环滤波跟踪

### 5.1 目标 7 状态机与转移规则
```
               [Undetected]
                    │ (WFOV 首次命中四门)
                    ▼
             [WideCandidate] ◄────────────────┐
                    │                         │
                    │ (连续命中达标 → 入单镜筒  │ (ATP超时 /
                    │  轮转集合)               │  连续门失败超限)
                    ▼                         │
        [AwaitingNfovAcquisition] ────────────┤
                    │                         │
                    │ (ATP稳定 + 延迟视线命中)  │
                    ▼                         │
   ┌────────────────┴────────────────┐        │
   ▼                                 ▼        │
[Strict/SensorLike]          [EstimatedTracking] ┘
```
- **转移条件**：
  - `Undetected` $\to$ `WideCandidate`：WFOV **首次**命中（遮挡/距离/视场/SNR 四门通过）即无条件成立。
  - `WideCandidate` $\to$ `AwaitingNfovAcquisition`：连续命中计数达 `wide_to_narrow_required_consecutive_hits`（默认 1 = 单次命中即入 NFOV 调度）才允许进入锁定集合。阈值取 3 时，目标第 1 次命中就已变为 `WideCandidate`，只是入 NFOV 推迟到第 3 拍——**连续命中门守卫的是进 NFOV 调度，不是进候选态**。
  - `Awaiting` $\to$ `EstimatedTracking`：actuator settled 且视场覆盖延迟真值 LOS，且 SNR 达标。
  - 跟踪丢锁：连续失败达 `nfov_tracking_gate_loss_cycles`（默认 2）退化回 `WideCandidate`。
  - 任意 $\to$ `Lost`：目标消失或传感器关闭。
- **逐状态输出语义（消费方契约）**：

| 状态 | 含义 | 该状态下输出 |
|---|---|---|
| `Undetected` | 初始或未被任何视场发现 | 不输出 |
| `WideCandidate` | WFOV 已发现，等待 NFOV 调度 | WFOV 检测记录 |
| `AwaitingNfovAcquisition` | 已入 NFOV 锁定集合，等待轮转窗口 | 轮空周期输出 WFOV；本窗口 settled 后视捕获结果 |
| `StrictTruthAssistedTracking` | 真值 LOS 驱动闭环 ATP（受物理门约束的 oracle） | 捕获成功起输出真值；失视时 coasting |
| `SensorLikeTruthAssistedTracking` | 真值 LOS 驱动，成功观测用独立误差子流 | 门通过时输出带误差角度/诊断距离 |
| `EstimatedTracking` | EKF/IMM 预测 LOS 驱动，门通过才消费量测 | 门通过时输出滤波估计；失视时预测 coasting |
| `Lost` | 目标从场景消失或传感器关闭 | 不输出 |

- **TruthAssisted 边界**：Sensor-like 带噪输出不参与指向、几何/SNR 判门或状态转移（无反馈）；同一 measurement seed 经固定混合与 domain tag 派生 WFOV/cue、Estimated 量测与 Sensor-like 输出三个独立子流，snapshot 分别持久化——一个模式的采样不改变另一链路的未来样本。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_state_machine_test.cpp]

---

### 5.2 6 维 ECI CV 状态 / 2 维角度量测扩展卡尔曼滤波 (EKF)
- **状态空间**：6 维 ECI 恒速模型 $\mathbf{x} = [x, v_x, y, v_y, z, v_z]^T$。
- **量测模型**：纯 2 维角度 $\mathbf{z} = [\text{az}, \text{el}]^T$（弧度），拒绝虚构不可观测的距离大协方差。
  $$\hat{\text{az}} = \text{atan2}(y_{\text{rel}}, x_{\text{rel}}), \quad \hat{\text{el}} = \text{atan2}(z_{\text{rel}}, \sqrt{x_{\text{rel}}^2 + y_{\text{rel}}^2})$$
- **因果执行闭环**：
  $$\text{Predict} \to \text{Actuator Slew} \to \text{Physical Gate} \to \text{Correct (Dynamic R)}$$
  门失败时滤波器只预测、不采样量测、不产生 NIS，并清零连续 NIS 超限计数。
- **实现边界**：输出角度用滤波估计的 ECI 位置反解视线，但 SNR/range 仍走真值物理链；runtime patch 对已存在滤波器只让后续周期读取新 R/Q，**不重置**协方差与状态向量。
- **反直觉点**：初始化均值采用 ECI 真值位置，后续滤波完全闭环于带噪声角度量测（仿真初始化工程简化）。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_ekf_baseline_test.cpp]

---

### 5.3 滤波后端选型与实验性角度滤波

| 后端 | 状态 | 状态空间 | 适用场景与限制 |
|---|---|---|---|
| **EKF** (默认) | session-wired | 6D ECI 位置/速度 | 标称 CV 跟踪基线 |
| **IMM(EKF)** | session-wired | 6D ECI 双模型交互 | 高机动助推段；全场景 RMSE 改善 28-55%（场景级评估结论，单测断言 imm < ekf） |
| **AngleCvKf** | experimental | 4D 方位/俯仰及变化率 | 单星纯角度跟踪实验后端（用例 16）；公开检测记录不含变化率 |
| SRIF | 未接线 | — | 线性 H helper 对 6D/2D 量测不兼容；先支持非线性量测再评估 |
| UDKF | 未接线 | — | 绑定线性 H；UD 分解不解决非线性量测 |
| CKF | 未实现 | — | 仓库无实现；需先提供 Jacobian 近似误差证据矩阵 |

- **不做在线自动后端切换的理由**：① 可复现性优先——在线选型使同一想定因阈值微调走不同后端，结果不可比；② 选型判据（"目标是否机动"）仿真期真值已知，泄露到选型逻辑等同作弊；③ 可解释性——固定后端的误差谱可归因，自动切换引入不可归因的跳变。

- **NIS 诊断**：$\text{NIS} = \boldsymbol{\nu}^T S^{-1} \boldsymbol{\nu} \sim \chi^2(2)$，门限 5.99。超限持续超设定期数可触发丢锁。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_imm_evaluation_test.cpp]、[evidence: tests/unit/sbirs_sensor/sbirs_angle_cv_kf_test.cpp]

---

## 6. 验收派生量（OPIR 验收日志）

当启用 `ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG`（CMake 开关，**默认 OFF**）时，普通功能项逐周期向
`SatelliteTarget.log` 写入，性能项写入 `opir_acceptance.log`：
1. **WFOV 地面覆盖区投影**（2026-09-06 裁剪包络化）：实际扫描中心（传感器系，含共模扰动与限位钳制）的视场矩形 4 边缘在传感器系采样，地平圈（Limb）圆弧在 ECI 绕星-地心方向按角半径 $0.999\rho$（$\rho=\arcsin(R_{\oplus}/r_{\text{sat}})$，收缩因子避免切线掠射求交失败）采样，经 boresight 链双向变换后取落盘（视场∩地球盘）交集采样点，与地球圆球求交（`kEarthRadiusM`，与遮挡门控同口径）旋回 ECEF 提取地心经纬度，输出交集的经纬度包络 [lat_min, lat_max]×[lon_min, lon_max] 映射为规范 4 角（西北、东北、东南、西南）；视场完全脱靶记 `miss`。圆球地球模型（椭球为已登记非目标，见 boundaries.md）。
2. **驻留时间**：$T_{\text{dwell}} = \text{FOV}_{\text{az}} / \omega_{\text{scan}}$（rate=0 退化配置记 0）。
3. **焦平面脱靶量**：逐轴小角投影 $\Delta x = f \cdot \tan(\Delta\text{az}), \Delta y = f \cdot \tan(\Delta\text{el})$（非畸变光学模型）。
4. **目标信号能量**：$E = P_{\text{received}} \cdot t_{\text{int}}$（`ComputeSnr` 出参透出，不改 SNR 数值）。
5. **宽窄切换连续命中计数**：逐目标累计连续通过 WFOV 四门的周期数；命中计数在候选创建点自增（当周期即计 1）；任一门失败/目标消失清零，捕获晋级进跟踪时清零（丢锁回宽场需重新积累）。
6. **角定位误差（需求映射 3.2.1.6.3）**：宽场候选写测量角 $-$ 真值角；窄场跟踪写最终输出角（滤波/误差注入全部完成后）$-$ 真值角；方位差按最短角差回绕（wrap-aware）。
- **边界**：全部派生量仅走日志，不进入公共输出帧；覆盖区为经纬度范围框（非交集多边形）、反子午线经度环绕不处理（均为已裁定非目标，见 boundaries.md）。
- **证据链**：[evidence: tests/unit/sbirs_sensor/sbirs_foundation_test.cpp]

---

## 核心反直觉点与工程陷阱

> [!IMPORTANT]
> 1. **参考系分化**：几何门控与物理限位在**传感器系**执行；但 raw output 测角与 EKF 估计严格报告于 **ECI 惯性极坐标系**。
> 2. **单镜筒轮转释放语义**：普通目标释放仅清理该目标的状态簿记，镜筒当前 LOS 不复位（下一目标从当前视线续转）；仅进入 Standby 时光轴才归零。
> 3. **同帧免费多跟不摊薄**：落入当前视场内的所有锁定目标共享同一批稳定采样帧，帧数不因目标数量增加而被摊薄。
> 4. **真值辅助跟踪依旧受物理门控约束**：`StrictTruthAssisted` 与 `SensorLikeTruthAssisted` 的指向虽然由真值引导，但仍然要经过执行器动力学转向、视场窗口覆盖与物理 SNR 判门，绝非无条件直通的理想通道。
> 5. **现实噪声分母采用能量基准**：噪声分母为积分时间内的噪声能量开方，不是功率相加；背景光子起伏严格受像元立体角 $\Omega_{\text{pixel}}$ 约束，背景亮度不乘以 $4\pi$ 或 $1\text{ sr}$。
> 6. **纯 2 维角度 EKF 量测**：被动红外不可测距，直接采用 2 维量测 Jacobian，不采用人为设定超大协方差屏蔽伪距离。
> 7. **首次捕获延迟视线评估**：捕获窗口是在延迟时间线（$\tau_{\text{latency}}$）上评估目标真实视线，卫星位移与目标位移均参与平移，不会因中心取测量值而必然捕获。
> 8. **覆盖区包络必须联合地平圈采样**：俯仰视场 ≥ 盘高（如 24°）时视场矩形 4 边缘全部落在盘外、边缘采样恒空，但地球视盘完全落在视场内部——此时交集包络由地平圈圆弧采样（含盘内星下点）决定，只用视场边缘采样会恒 miss。视场边缘走传感器系采样经链正向变换；地平圈在 ECI 构造后经链逆向变换（ECI→传感器系）做视场内判定。

---

## 非目标（刻意不实现的算法）

1. **二维红外成像级仿真与多色分类器**：定位在标量辐射探测与光轴动力学，不生成图像阵列，不建模星载点源探测算法（如三帧差分/空域滤波）。
2. **MODTRAN 高光谱三维大气剖面**：采用标量气象折减与壳段气团弦长近似，不进行逐波数辐射传输积分。
3. **在线后端自动切换与参数在线自适应**：保持系统确定性与想定回放严格复现。
4. **刚体反作用轮与星体姿态控制动力学**：光轴扰动采用一阶 Gauss-Markov 小角近似，不仿真卫星姿控飞轮力矩与控制回路。
5. **CA cue predictor（已完成标定、刻意拒绝接线）**：五样本角度二次最小二乘 CA 在 108 个无噪声持续加速组合中聚合 RMS 优于 CV，但在标称噪声恒速场景（dt=0.1 s、latency=0.5 s、σ=0.01°）RMS/P95 为 0.141/0.279°，**劣于** CV 的 0.074/0.142°，捕获率从 73.91% 降至 41.30%——未通过标称噪声零回退门，拒绝生产接线，不新增 CV/CA 配置或自动切换。[evidence: tests/unit/sbirs_sensor/sbirs_cue_ca_characterization_test.cpp]
