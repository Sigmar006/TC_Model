---
Status: active
Last-reviewed: 2026-09-03
Authority: RIR 算法登记与实现边界
Answers: RIR 每个算法做什么、实现边界在哪、哪些反直觉、哪些刻意不做
---

# Remote Identification Radar 算法登记

本文是远程识别雷达（Remote Identification Radar, RIR）算法与部件清单及实现边界的权威。算法本身的逐步代码逻辑见 `src/remote_identification_radar/`；本文回答“用没用 / 到哪步 / 边界在哪 / 哪些反直觉 / 哪些刻意不实现”。模块级边界（dt_sec、环境/RF 事实、输出/失败语义、公共 API 边界）见 [boundaries.md](boundaries.md)，数据流与周期管线见 [data-flow.md](data-flow.md)。

RIR 识别积累全量挂接于自持雷达链路（自发射 $\to$ 入射 $\to$ 检测 $\to$ 关联 $\to$ 滤波）生产的内部航迹，不再消费外部航迹供给。

---

## 算法登记表

| 算法/部件 | 意图 / 核心转换 | 实现状态 | 证据与单测 |
|---|---|---|---|
| 地球遮挡门控 | 视线穿地检测，穿地目标在可扫描体积与 SNR 前硬排除 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_earth_occultation_test.cpp] |
| 波束指向与增益解析 | 调度指向 + 目标视线角 $\to$ 有效宽度/天线方向性增益 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_search_sector_test.cpp] |
| 自发射构建 | 硬件配置 + 周期上下文 $\to$ 逐驻留 `RfSceneEmission` | session-wired | [evidence: tests/unit/remote_identification_radar/rir_emission_factory_test.cpp] |
| 接收机工作状态构建 | 自发射 + 硬件 $\to$ `RirReceiverOperatingState`，供前端与检测单元 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_receiver_state_builder_test.cpp] |
| RF 前端求解 | 合并外部射频场景与自发射 $\to$ incident links 与致盲饱和判定 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_rf_front_end_resolver_test.cpp] |
| 有效物理 RCS | 场景目标 + 视线角 + 载频 $\to$ 混合物理 RCS（$\text{m}^2$） | session-wired | [evidence: tests/unit/remote_identification_radar/rir_effective_rcs_test.cpp] |
| 逐目标大气损耗 | 周期载频 + 几何 + 气象观测 $\to$ 大气附加损耗（dB） | session-wired | [evidence: tests/unit/remote_identification_radar/rir_atmospheric_physics_test.cpp] |
| 逐目标主瓣地杂波模型 | 擦地角 + 距离向脉压单元 + 植被档位 $\to$ 主瓣杂波等效噪声（W） | session-wired | [evidence: tests/unit/remote_identification_radar/rir_surface_clutter_model_test.cpp] |
| 检测单元求解 | 回波事实 + 外部干扰 + 杂波噪声 $\to$ 分项 SINR 账本 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_self_contained_pipeline_test.cpp] |
| 验收旁路 MTI/MTD | 单元功率 + 脉冲时序 $\to$ 8 滤波多普勒通道（仅日志） | internal/受控 | [evidence: tests/unit/common/common_mti_mtd_acceptance_bank_test.cpp] |
| 统计级 CFAR 判决 | SINR + Swerling + Pfa $\to$ 截断积累数 $P_d \to$ 蒙特卡洛判决 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_signal_detector_test.cpp] |
| 量测误差模型（Bias/Std 拆分） | 积累 SNR + 宽度 + 带宽 $\to$ 随机 std + 独立系统偏置 bias | session-wired | [evidence: tests/unit/remote_identification_radar/rir_measurement_error_test.cpp] |
| LAPJV 最优数据关联 | 检测量测 + 航迹种子 $\to$ 全局最优增广方阵线性指派 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_track_associator_test.cpp] |
| 单目标卡尔曼滤波 | 诚实速度先验 + 动态量测协方差 $R \to$ 6 维 CV 后验状态 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_track_filter_test.cpp] |
| 物理加速度周期间差分 | 连续更新周期后验速度差分 $\div$ 实际经过时间 $\to$ 真实物理加速度 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_track_filter_test.cpp] |
| IMM 双模型交互多模型滤波 | 双模型 CV 对数等距过程噪声 $\to$ 机动自适应组合后验 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_track_filter_test.cpp] |
| 航迹池与生命周期状态机 | 关联命中/失配 $\to$ 内部航迹晋级/回收（无单周期回收复用） | session-wired | [evidence: tests/unit/remote_identification_radar/rir_track_lifecycle_test.cpp] |
| 驻留候选排序 | 内部航迹结论 + 场景目标 $\to$ 未识别优先 + 斜距次近候选序 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_self_contained_pipeline_test.cpp] |
| 实际有效目标最大斜距 | 本周期入候选并成航迹的目标最大输入几何斜距 $\to$ `max_detected_slant_range_m` | session-wired | [evidence: tests/unit/remote_identification_radar/rir_search_sector_test.cpp] |
| 驻留调度与波位构建 | 实际搜索扇区（子窗 $\cap$ 体积）$\to$ 统一扫描波位序列 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_search_sector_test.cpp] |
| 指定识别任务管理 | 目标 ID + 限时窗口 $\to$ 任务生命周期（识别即完成，超时作废） | session-wired | [evidence: tests/unit/remote_identification_radar/rir_self_contained_validation_test.cpp] |
| RCS 特征提取 | 视角样本 + 视线角 + SNR $\to$ `RirRcsObservation` | session-wired | [evidence: tests/unit/remote_identification_radar/rir_recognition_feature_test.cpp] |
| 运动特征提取 | 内部已确认航迹 + 不确定度 $\to$ 横向加速度与转弯半径 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_recognition_feature_test.cpp] |
| 极化特征提取 | 双通道样本 + 视线角 + SNR $\to$ `RirPolarizationObservation` | session-wired | [evidence: tests/unit/remote_identification_radar/rir_recognition_feature_test.cpp] |
| 距离像特征提取 | 散射中心 + 带宽 + SNR $\to$ `RirRangeProfileObservation` | session-wired | [evidence: tests/unit/remote_identification_radar/rir_recognition_feature_test.cpp] |
| 观测构造 | 场景真值 + 内部航迹 + 观测上下文 $\to$ `RirFeatureSet`（驻留质量因子作用于 RCS/极化/距离像） | session-wired | [evidence: tests/unit/remote_identification_radar/rir_recognition_feature_test.cpp] |
| 极化验收旁路（L2） | 最近邻极化样本 $\to$ Span / $\lvert\det(S)\rvert$ / 去极化 / Graves $\psi\tau$（仅日志，不进识别） | internal/受控 | [evidence: tests/unit/remote_identification_radar/rir_polarization_acceptance_s_test.cpp] |
| SQLite 特征数据库加载 | SQLite 文件 $\to$ 内存全量模板缓存，schema 自描述校验 | internal/受控 | [evidence: tests/unit/remote_identification_radar/rir_recognition_database_test.cpp] |
| 特征概率匹配与大类归约 | 特征集 + 适用条件 $\to$ 似然度得分，类别取得分最高型号 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_recognition_database_test.cpp] |
| 多周期多假设积累判定 | 逐周期观测得分 $\to$ 阈值门 + margin 门判定识别状态机 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_self_contained_pipeline_test.cpp] |
| 特征量测帧组装（出口①） | 有效特征观测 + 平台位置 $\to$ `RirFeatureMeasurementRecord` | session-wired | [evidence: tests/unit/remote_identification_radar/rir_feature_measurement_test.cpp] |
| 航迹归属视图组装（出口②） | 航迹全快照 $\to$ 键 $\leftrightarrow$ 真值映射与 hit/位置/速度诊断 | session-wired | [evidence: tests/unit/remote_identification_radar/rir_track_attribution_test.cpp] |
| 逐驻留 RF 发射帧组装 | 识别周期逐驻留自发射 $\to$ `RfEmissionFrame` | session-wired | [evidence: tests/unit/remote_identification_radar/rir_emission_frame_test.cpp] |

实现状态说明：
- **session-wired**：已接入 `RirController` / `RirSession` 主链路，覆盖配置、执行周期、重放与集成。
- **internal/受控**：库内私有支撑能力或仅验收日志通道消费。

---

## 核心算法详述

## 1. RF 物理探测与信号链

### 1.1 地球遮挡门控 (Earth Occultation Hard Gate)
- **算法意图与调用时机**：在 WFOV 搜索、天线增益与 SNR 计算之前，判定平台与目标间的视线（LOS）是否受地球圆球几何遮挡。穿地目标立即剔除，避免浪费计算开销。由 `RirController::RunCycle` 初始几何阶段调用。
- **数学与物理模型**：
  - 输入：平台 ECEF 位置 $\mathbf{r}_p$、目标 ECEF 位置 $\mathbf{r}_t$。
  - 模型：线段射线求交。地球视为均值圆球（$R_e = 6371\text{ km}$）。若视线有限线段与地球相交或相切，则判定遮挡。
- **实现边界与工程简化**：
  - 判定核委托公共域 `common/geometry/EarthOccultation` 单源。
  - 纯几何通视判定，不计入对流层 $4/3$ 等效地球半径（$k$ 因子仅在标量大气损耗中体现）。
- **边界保护**：若 ENU 到 ECEF 坐标转换失败，保守跳过该门（不报错、不遮挡）。
- **反直觉点**：相切即算遮挡；本门排在扫描体积与 SNR 计算前，穿地目标在 `detection_cell` 之前即被短路。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_earth_occultation_test.cpp]

---

### 1.2 天线波束指向合成与有效增益 (Beam Pointing & Effective Gain)
- **算法意图与调用时机**：根据驻留调度器输出的波位中心与目标视线角，计算目标偏离主瓣轴线的离轴衰减，求解天线单程与双程方向性增益。
- **数学与物理模型**：
  - 输入：波位中心 $(\theta_{\text{beam}}, \phi_{\text{beam}})$，目标相对雷达视线角 $(\theta_t, \phi_t)$。
  - 核心公式（高斯波束展开近似，**仅主瓣段成立**；主瓣外公共核走 $\text{sinc}^2$ 副瓣包络连续延拓并以副瓣电平为下限，另有扫描损耗项）：
    $$G(\theta_t, \phi_t) = G_0 \cdot \exp\left( -2.776 \cdot \left[ \left(\frac{\Delta\theta}{\theta_{3\text{dB}}}\right)^2 + \left(\frac{\Delta\phi}{\phi_{3\text{dB}}}\right)^2 \right] \right)$$
    其中 $\Delta\theta = \text{wrap}_{180}(\theta_t - \theta_{\text{beam}})$ 为归一化方位差。
- **主瓣模型类型（可选）**：`RirAntennaPatternModelType` 静态会话配置——`kGaussianMainLobe`（默认，上文公式）、`kCosinePower`（余弦幂近似）、`kSincPattern`（sinc² 均匀孔径理论解，**必须填物理孔径尺寸**）；类型只换主瓣衰减曲线，不改变波束中心指向与轴心增益能力锚，运行期不可切换；非法取值静默回退高斯主瓣。
- **实现边界与工程简化**：
  - 波束状态解析委托公共单源 `common/radar/FrozenBeamResolve.h`。
  - 调度器给指向、RIR 信任指向：RIR 不会把给定波束中心吸附到目标位置。指向偏离目标即按实际离轴角真实衰减。
- **边界保护**：目标位置退化（范数 $\le 0.1\text{ m}$）时视线角无效，增益安全回退为主瓣峰值 $G_0$（az=0/el=0 兜底角不做离轴衰减，AR 同口径）。
- **反直觉点**：高斯公式只在主瓣内有效——深离轴目标的实际增益由副瓣电平下限决定，不会按高斯尾无限衰减。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_search_sector_test.cpp]、[evidence: tests/unit/remote_identification_radar/rir_antenna_pattern_test.cpp]

---

### 1.3 逐目标主瓣地杂波物理模型 (Per-Target Surface Clutter Model)
- **算法意图与调用时机**：当环境配置启用植被地表杂波时，由 `RirController` 调用 `RirSurfaceClutterModel` 逐目标求解主瓣地面杂波等效噪声功率（W），替代旧版恒定 CNR 口径。
- **数学与物理模型**：
  - 输入：目标斜距 $R$、目标俯仰视线角 $\phi$、发射波长 $\lambda$、天线波束宽度、脉冲宽度 $\tau$ 及信号带宽 $B$。
  - 核心公式：
    1. 擦地角：$\psi = \text{clamp}\left( \frac{\phi_{3\text{dB}}}{2} - \phi,\; 1^\circ,\; 89^\circ \right)$（原始值 $\le 0$ 即目标仰角高出半俯仰波束宽、主瓣完全离地，杂波功率直接为 0；下限 1° 避开多径干涉区，上限 89° 防超宽波束配置下 $\cos\psi$ 变号产生负面积）。
    2. 杂波散射系数（按参考擦地角归一）：$\sigma_0(\psi) = \sigma_{0,\text{ref}} \cdot \dfrac{\sin\psi}{\sin 10^\circ}$，$\sigma_{0,\text{ref}}$ 为植被档位表值（按参考擦地角 $10^\circ$ 声明的 S 波段量级值，非实测标定）。
    3. 杂波照射面积（取较小者；脉压后距离单元 $\delta_R = c/2B$ 与测距误差模型同源）：
       $$A_c = \min\left( \frac{R \cdot \theta_{3\text{dB}} \cdot \delta_R}{\cos\psi},\; \frac{R^2 \cdot \theta_{3\text{dB}} \cdot \phi_{3\text{dB}}}{\sin\psi} \right)$$
    4. 杂波等效噪声功率：杂波 RCS $= \sigma_0 \cdot A_c$ 代入与目标同一雷达方程（公共核参考脉宽 $13\,\mu\text{s}$ 能量基准 + 主瓣峰值增益 $G_0$），再叠加脉压能量增益 $\max(1, B\tau)$：
       $$P_{c} = \frac{P_t G_0^2 \lambda^2 \sigma_0 A_c}{(4\pi)^3 R^4 L_{\text{prop}}}\Big|_{13\,\mu\text{s 基准}} \cdot \max(1, B\cdot\tau)$$
       相对参考脉宽基准的净修正量为 $+10\log_{10}(B \cdot 13\,\mu\text{s})$（与 $\tau$ 无关，默认 $B = 4.5\text{ MHz}$ 时约 $+17.7\text{ dB}$）；最后按 CNR 经统一单源换算回等效噪声瓦（保留 $\pm 120\text{ dB}$ 相对钳制口径）。
- **实现边界与工程简化**：
  - 杂波回波按主瓣峰值增益 $G_0$ 近似，斜视驻留时的该口径差为已知工程简化。
  - $\sigma_0$ 档位表为 S 波段量级声明值，非外场实测标定数据。
- **边界保护**：斜距、载频、带宽或波束宽度非正时，返回 0 功率；关闭杂波时完全不生成杂波增益账本记录。
- **反直觉点**：同一 SINR 账本内，杂波回波按主瓣峰值增益估计而目标按逐目标离轴增益估计——斜视驻留时两者口径不同，该已知近似使杂波相对偏高。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_surface_clutter_model_test.cpp]

---

### 1.4 统一物理探测与统计 CFAR 判决 (Detection Cell & Statistical CFAR)
- **算法意图与调用时机**：在 `RirSignalDetector` 中汇集目标回波、入射干扰和杂波噪声，建立分项 SINR 账本并执行 CFAR 检测。
- **数学与物理模型**：
  - 分项 SINR 账本（公共核 `common/radar/DetectionCellResolver`，四项处理增益均经 $[0, 40]\text{ dB}$ 钳位）：
    $$\text{SINR} = \frac{P_{\text{echo}} \cdot G_{\text{pc}} \cdot G_{\text{target}}}{P_{\text{thermal}} \cdot G_{\text{noise}} + P_{\text{interference}} / G_{\text{jam\_sup}} + P_{\text{clutter}} / G_{\text{clutter\_sup}}}$$
    其中 $G_{\text{pc}} = \max(1, B\tau)$ 为脉压能量增益。四增益方向**不对称**：目标增益乘分子、噪声增益乘性抬高热噪底进分母、干扰/杂波抑制各自除分母对应项（配置按「正值 = 劣化」理解噪声增益）。RIR 默认注入 target/noise/jamming/clutter $= 3/1/8/10\text{ dB}$（非零）。
  - 生效积累脉冲数：检测单元路径 $N_{\text{eff}} = \min(N_{\text{policy}}, N_{\text{window}})$，其中 $N_{\text{window}}$ 为接收窗内按**该目标回波时延**截断的可容脉冲数（随目标距离变化，非恒定 $\lfloor T_{\text{dwell}} \cdot \text{PRF} \rfloor$）；RF 回退路径取 $N_{\text{eff}} = N_{\text{policy}}$ 不做窗截断。默认驻留 $0.05\text{ s}$（PRF 300 Hz，窗容量 15）× policy 缺省 10 $\to$ 生效 10，相对驻留积累能力损失约 $1.8\text{ dB}$。
  - 判决模型：基于 Swerling 模型的非起伏/起伏目标检测概率 $P_d(\text{SINR}, P_{\text{fa}}, N_{\text{eff}})$，并做蒙特卡洛随机数判决。
- **实现边界与工程简化**：
  - 判决编排委托公共域 `common/radar/StatisticalCfarDetector`，非单元滑窗 CA-CFAR。
  - **RF 链回退模式**：RF 链解析失败时，回退至阶段 1 简化口径（回退分支不注入外部干扰，`env.jam_noise_w` 恒为 0）。
- **边界保护**：接收机饱和（总功率超限）为致盲语义：整周期目标不产生检测，报告 `kTargetReceiverFrontEndSaturated`。
- **反直觉点**：
  - **四增益方向不对称**：`noise_processing_gain_db` 抬高噪声底（进分母），与目标增益（乘分子）、干扰/杂波抑制（除分母项）方向不同——配置时按「噪声项正值 = 劣化、抑制项正值 = 改善」理解。
  - **回波能量基准统一为 $B\cdot\tau$ 脉压口径**：目标回波与杂波噪声等效均统一叠加 $10\log_{10}(\max(1, B\tau))$。
  - **6 dB 回退门控**：`kSnrFallback` 模式以 $\text{SNR} \ge 6\text{ dB}$ 代替蒙特卡洛判决，位置取真值；`kDetectorGate` 才进行采样。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_signal_detector_test.cpp]

---

### 1.5 量测误差模型（Bias / Std 显式拆分）
- **算法意图与调用时机**：由 `RirMeasurementErrorModel` 计算探测点迹的观测随机标准差与固定系统偏置，为航迹关联与滤波提供自适应协方差 $R$。
- **数学与物理模型**：
  - 输入：积累信噪比 $\text{SNR}_{\text{eff}} = \text{SINR} + 10\log_{10}(N_{\text{eff}})$、波束宽度与带宽 $B$。
  - 随机误差（标准差，公共核 `RadarEquations` 工程近似系数，非教科书 $1/\sqrt{2\,\text{SNR}}$ 形式；SNR 均为线性值）：
    $$\sigma_r = \frac{0.5 \cdot \delta_R}{\sqrt{\text{SNR}_{\text{eff}}}} = \frac{c}{4B\sqrt{\text{SNR}_{\text{eff}}}}, \quad \sigma_\theta = \frac{0.317 \cdot \theta_{3\text{dB}}}{\sqrt{\text{SNR}_{\text{eff}}}}$$
    其中 $\delta_R = c/2B$ 为距离分辨率。低 SNR 保护分支：$\text{SNR} < -10\text{ dB}$ 时 $\sigma_r \to 1.5777\,\delta_R$、$\sigma_\theta \to \theta_{3\text{dB}}$。
  - 固定系统偏置（2026-08-30 显式拆分）：
    $$\text{bias}_r = 20\text{ m (沿视线方向)}, \quad \text{bias}_\theta = \frac{\theta_{3\text{dB}}}{30} \text{ (正偏置)}$$
- **实现边界与工程简化**：
  - `std` 字段纯粹表达高斯白噪声；`bias` 字段施加在量测均值侧（先偏移再加噪）。
  - 角度两轴误差经 RMS 合成，供内部轻量跟踪使用，不向外部暴露点迹。
- **反直觉点**：`kSnrFallback` 真值口径既不采样随机噪声也不施加 bias（只携带误差协方差）——bias 与噪声同属检测器门控模式的量测生成侧简化。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_measurement_error_test.cpp]

---

## 2. 航迹关联与自持滤波跟踪

### 2.1 LAPJV 全局最优数据关联 (LAPJV Square Assignment)
- **算法意图与调用时机**：每周期将检测量测分配至既有内部航迹，解决密集目标下的多对多指派冲突。由 `RirTrackAssociator` 调用。
- **数学与物理模型**：
  - 关联代价矩阵构建：马氏距离门控 $d_M^2 = (\mathbf{z} - \hat{\mathbf{z}})^T S^{-1} (\mathbf{z} - \hat{\mathbf{z}}) \le \gamma_{\text{gate}}$（默认波门 9.0）。
  - 求解器：增广方阵 Jonker-Volgenant 算法（LAPJV），委托公共单源 `common/tracking/GatedSquareAssignment.h`。
- **实现边界与工程简化**：
  - 关联键不回收复用：航迹回收只删除内部状态，`next_key` 保持全局单调自增。
- **反直觉点**：关联键单调不回收（详见文末反直觉点 7）——识别积累不需要检测 `hit_count` 回落判定，新键天然等于新目标。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_track_associator_test.cpp]

---

### 2.2 单目标卡尔曼滤波与诚实速度先验 (Single-Target KF & Honest Velocity Prior)
- **算法意图与调用时机**：对单目标执行运动学平滑滤波，输出 6 维**平台局部 ENU** 状态估计 `[x, vx, y, vy, z, vz]`。由 `RirTrackFilter` 逐周期推进。滤波、关联与量测全程使用场景目标自带的平台局部 ENU 位置（ECEF 仅进入地球遮挡门，不进滤波状态）。
- **数学与物理模型**：
  - 状态转移：标准恒速（CV）模型，时间步长 $dt$。
  - **诚实初始化（去真值化）**：
    $$\mathbf{x}_0 = [\mathbf{p}_{\text{meas}}, \mathbf{0}]^T, \quad P_0 = \text{diag}(\sigma_{p}^2, \sigma_{v0}^2, \sigma_{p}^2, \sigma_{v0}^2, \sigma_{p}^2, \sigma_{v0}^2)$$
    其中速度无知方差 $\sigma_{v0} = \text{initial\_velocity\_std\_mps}$（默认 $3000\text{ m/s}$）。
- **实现边界与工程简化**：
  - 动态 $R$ 更新：量测协方差取自探测时的实际误差估计。
  - 过程噪声与量测噪声钳制下限为 $0.001$（防止协方差数值下溢退化）。
- **反直觉点**：初始化时不使用任何真值速度；大的先验方差使建轨初期波门宽阔，收敛后自然收窄，这是波门自适应收敛的物理来源。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_track_filter_test.cpp]

---

### 2.3 物理加速度周期间差分 (Acceleration Posterior Velocity Difference)
- **算法意图与调用时机**：计算目标运动学加速度，为识别模块中的机动特征提取提供真实物理加速度输入。由 `RirTrackLifecycle::ApplyHitFilter` 计算。
- **数学与物理模型**：
  - 核心公式：
    $$\mathbf{a}_k = \frac{\hat{\mathbf{v}}_{k,\text{posterior}} - \hat{\mathbf{v}}_{k-1,\text{posterior}}}{\Delta t_{\text{hit}}}$$
    其中 $\Delta t_{\text{hit}} = (k - k_{\text{last\_hit}}) \cdot dt$ 为距上次命中经过的实际物理时间跨度。
- **实现边界与工程简化**：
  - 滑行（失配）周期滤波速度保持 CV 外推，重命中差分按实际经过的 $\Delta t_{\text{hit}}$ 折算，机动目标加速度不会因跨周期而虚假放大。
  - **置零规则**：新航迹初始化周期或失跟重捕周期，无上周期后验速度可供差分，加速度严格置零；miss 周期加速度归零。
- **反直觉点**：加速度只来源于滤波后验速度的差分，场景速度种子完全不参与差分，避免产生反向滞后伪加速度。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_track_filter_test.cpp]

---

### 2.4 IMM 双模型交互滤波 (IMM Dual-Model Filtering)
- **算法意图与调用时机**：在目标可能出现大幅机动时，自适应调整过程噪声，改善跟踪收敛性。由 `RirImmFilter` 驱动。
- **数学与物理模型**：
  - 双 CV 模型：过程噪声按对数等距分布 $\{q, 10q\}$，锚定于 `kalman_noise_diff_coeff`。
  - 马尔可夫转移矩阵：对角优势转移（对角元素 0.95）。
- **实现边界与工程简化**：
  - 仅对已确认（`kConfirmed`）航迹激活；未确认阶段退化为单模型 CV。
  - `UpdateConfig` 在线热同步既有运行态（每模型 q/转移矩阵，AR SyncRuntimeTuning 同口径）；模型数变化丢弃运行态、下次 confirmed 命中惰性重建。
  - 数值核委托公共域 `oneq::common::estimation::ImmFilter<6, 3>`。
- **反直觉点**：`kalman_noise_diff_coeff` 是低端锚点而非间距——$q = 1$ 时模型对对数等距退化为 $\{1.0, 10.0\}$，并非 $q$ 与 $10q$ 以外的第三种取值。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_track_filter_test.cpp]

---

## 3. 驻留调度与任务管理

### 3.1 实际搜索扇区与波位调度 (Scan Sector & Dwell Scheduling)
- **算法意图与调用时机**：由 `RirSession` 与 `ScanScheduleRuntime` 将任务搜索扇区解析为离散波位，驱动空闲驻留与指定跟踪。
- **数学与物理模型**：
  - 实际搜索扇区：$\Omega_{\text{actual}} = \Omega_{\text{mission}} \cap \Omega_{\text{steerable}}$（任务子窗与机械/电扫物理体积求交；子窗缺省无界时交集退化为体积，默认体积 $\pm60/\pm30$ + center $(0,0)$ 与重构前绝对限位逐位等价）。
  - 波位生成：在相对限位上按步长 $\theta_{3\text{dB}} \times \text{step\_scale}$（缺省 1.0）采样，再经 `scan_center_deg` 平移并归一化；公共核 `BuildScanPattern` 每轴采样上限 4096，超出截断。
- **实现边界与工程简化**：
  - 有效波束宽度按 nominal 两级回退（nominal 为 0 时按 $\lambda = c/f$ 与天线尺寸 $\lambda/L$ 物理推导）——回退触发条件是**名义波束宽度缺失**，避免配置静默产生空波位表。
  - 指定识别任务如果超出物理转向体积，立即回退扫描并报错 `kOutsideSteerableVolume`；指定目标在硬件体积内豁免任务子窗裁剪（确认航迹目标同豁免）。
  - 非法体积/步长/空交集回退 `scan_center`。
- **反直觉点**：子窗裁剪只作用于搜索候选——指定目标与已确认航迹在体积内可越出子窗被服务（角域豁免），这是 TAS 边搜边跟的几何前提。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_search_sector_test.cpp]

---

### 3.2 驻留候选排序与指定识别任务 (Candidate Ordering & Designation)
- **算法意图与调用时机**：在每个周期初决定雷达波束服务的目标顺序。
- **执行规则**：
  1. 优先级规则：未识别目标优先于已识别目标，同等状态下斜距较近者优先；威胁等级不参与排序。
  2. 指定识别任务生命周期：外部指定目标 ID 及限时窗口周期数。识别达成（`kCategoryConfirmed` 或 `kModelConfirmed`）时任务立即圆满完成并恢复空闲扫描；限时窗口耗尽仍未识别则超时作废（`kAcquisitionTimeout`）。
  3. **检测候选主瓣覆盖门**：检测候选须落在本周期某驻留指向的半功率宽内，否则不入候选（候选生成前置几何门）。
  4. **TAS 边搜边跟**：已确认航迹每周期保留专用跟踪驻留，与搜索驻留并存。
- **反直觉点**：识别是离散结论，识别完成即释放指向，不持续独占波束跟随（与 AR 捕获后持续跟随航迹不同）。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_self_contained_validation_test.cpp]

---

## 4. 特征提取与目标识别

### 4.1 四维特征提取 (Feature Extraction)
- **算法意图与调用时机**：在 `recognition/` 中由已确认航迹提取多维物理特征：
  - **RCS 特征**：视角样本**最近邻匹配**（多样本时取欧氏最近单样本值，非插值）；$\text{SNR} < 6\text{ dB}$ 或视角覆盖不足（低于 profile 声明的 `minimum_aspect_coverage_deg`）时维度标记为无效。
  - **运动特征**：根据后验速度差分计算横向加速度与转弯半径；观测质量因子 $Q = 10000 / (10000 + \text{Tr}(P_v)/3)$。
  - **极化特征**：双极化通道比率；强干扰压低 SNR 时该维失效。
  - **距离像特征**：高分辨距离像峰值数与宽度；分辨率 $c/2B$ 超限时失效。
- **反直觉点**：RCS 最近邻**不强制视角覆盖**——样本网格非空即产生观测，覆盖下限判定由 `minimum_aspect_coverage_deg` 承担（维度无效化而非不产生观测）。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_recognition_feature_test.cpp]

---

### 4.2 概率匹配与大类归约 (Matching & Category Reduction)
- **算法意图与调用时机**：将提取到的有效特征与 SQLite 模板库（全量内存加载）进行高斯似然度比对。
- **数学与物理模型**：
  - 单特征得分：$s_i = \exp\left(-0.5 \cdot \left(\frac{f_i - \mu_i}{\sigma_i}\right)^2\right)$。
  - 综合得分（**质量加权**平均，非等权）：$S = \dfrac{\sum_d w_d \, q_d \, s_d}{\sum_d w_d \, q_d}$，质量为 0 的维度不进分子也不进分母；型号最终得分乘以 `model.prior` 先验因子。
  - **大类得分归约（2026-08-30 裁定）**：
    $$S_{\text{category}} = \max_{m \in \text{category}} S(m)$$
    大类得分取类内所有型号的最高分，同类别型号数量多不会获得累加加分。
- **实现边界与工程简化**：
  - 单候选且无竞争者时，次优分记为 0，margin 门限检查天然恒过。
- **反直觉点**：`feature_scores` 分项相似度按判定**实际命中**的 profile 报告（`best_profile_index` 传递，非恒取首个；库热替换致下标越界时回退首 profile）。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_recognition_database_test.cpp]

---

### 4.3 多周期多假设积累状态机 (Recognition Tracker FSM)
- **算法意图与调用时机**：跨周期平滑识别置信度，防范单拍噪声跳变。
- **状态转移规则**：
  - 得分 $\ge \text{acceptance\_score}$ 且竞争余量（margin）充足，且有效特征维数 $\ge 2$，晋级为 `kModelConfirmed`。
  - 纯运动特征只允许确认到大类（`kCategoryConfirmed`），不得单独用于确认型号。
- **反直觉点**：运动学特征跨型号共性太强（型号间共享飞行品质），单独确认型号被刻意禁止——这是识别语义边界而非实现缺陷。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_self_contained_pipeline_test.cpp]

---

## 5. 结果输出与双产品出口

### 5.1 特征量测帧组装（出口①） (Feature Measurement Frame)
- **业务意图**：将本周期雷达构建的有效特征观测直接透出给融合系统（Stage B 契约）。
- **实现边界**：
  - **透出点在积累质量门之前**：观测质量低于识别积累门限但掩码非零的特征记录**照常透出**；但在识别最大距离之外的未构建目标不生成记录。
  - **方位参考系**：出口采用内部 ENU 约定（自东向逆时针量），下游融合适配器负责转换为北向极坐标。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_feature_measurement_test.cpp]

---

### 5.2 逐驻留 RF 发射帧组装 (Per-Dwell RF Emission Frame)
- **业务意图**：当处于识别模式且 RF 链求解成功时，经 `RirCycleResult::emission_frame` 输出本周期全部实际驻留的物理发射记录，供外部构建全局 RF 场景。
- **实现边界**：每个驻留波位一条记录，发射指向对应各驻留的实际 boresight，`emission_id` 在帧内唯一化。
- **证据链**：[evidence: tests/unit/remote_identification_radar/rir_emission_frame_test.cpp]

---

## 辅助/轻量算法（紧凑登记）

### A.1 自发射构建 (RirEmissionFactory)
- **意图**：hardware + 周期上下文 $\to$ `RfSceneEmission`（ECEF 波束指向）。
- **规则**：功率包络钳制；驻留窗脉冲数按 $\lceil \text{窗}/\text{PRI} \rceil$ 计（AR PrepareRfCycle 同口径），下限 1。
- **边界与反直觉**：载频固定取 `transmitter.frequency_hz`（`ResolveCarrierHz` 忽略周期索引）——频率计划/跳频未实现；无 ECCM（烧穿/ECCM 为 AR 专属能力）；**可提取核心，阶段 3b 未迁**。
- **证据**：[evidence: tests/unit/remote_identification_radar/rir_emission_factory_test.cpp]

### A.2 有效物理 RCS (RirEffectiveRcs $\to$ common/rcs/RcsPhysics)
- **意图**：场景目标 + 视线角 + `rcs_physics` $\to$ m²，写入 detection cell 目标 `rcs_m2`。
- **规则**：混合编排 common 单源 `ComputeMixedPhysicalRcsM2`。
- **边界与反直觉**：默认开启且 mix=1（随视线角/载频变，非扫描量）；`enable=false` 或 mix=0 或 `carrier_hz <= 0` 回退 input RCS。
- **证据**：[evidence: tests/unit/remote_identification_radar/rir_effective_rcs_test.cpp]

### A.3 逐目标大气物理损耗 (common/atmosphere 标量胶水)
- **意图**：周期载频 + 平台/目标几何 + 气象观测 $\to$ 大气附加损耗 dB，叠加进全局植被/天气损耗。
- **规则**：`ComputeTargetAtmosphericPhysicsLossDb`；enable/k_factor 留模块侧。
- **边界与反直觉**：`enable_physical_model = false`（**默认**）时恒为 0——默认配置不产生该项损耗。
- **证据**：[evidence: tests/unit/remote_identification_radar/rir_atmospheric_physics_test.cpp]

### A.4 接收机工作状态构建 (RirReceiverStateBuilder)
- **意图**：自发射 + hardware $\to$ `RirReceiverOperatingState`，供前端聚合与 detection cell。
- **规则**：与 AR 同口径 RF 接收机参数。
- **边界与反直觉**：**可提取核心，阶段 3b 未迁**。
- **证据**：[evidence: tests/unit/remote_identification_radar/rir_receiver_state_builder_test.cpp]

### A.5 验收旁路 MTI/MTD (common/radar/MtiMtdAcceptanceBank)
- **意图**：cell 功率 + PRF/载频 + 可选干扰单音 $\to$ 8 路派生与 MTI/MTD 增益。
- **规则**：N=8 通道、2 脉冲 MTI、$\sigma_v = 0.25\text{ m/s}$ 为核内常量。
- **边界与反直觉**：不进 SINR/Pd/航迹；无链路多普勒则干扰通道写 `无`；关验收开关时不求值。
- **证据**：[evidence: tests/unit/common/common_mti_mtd_acceptance_bank_test.cpp]

### A.6 验收旁路输出物组装 (RirAcceptanceRecords)
- **意图**：验收旁路产物落盘：`log/RadarOutput.log`（普通功能与外部接口，含 MTI/MTD 8 路派生、极化 L2 派生、运动特征/RCS 统计等列）、`log/rir_acceptance.log`（性能项）、`log/RadarAntennaGainPatterndata.csv`（全向网格方向图导出，体积随波束宽变化）、`log/Rir_scan_pattern.csv`（本周期波位序列）。
- **规则**：CMake 验收开关门控，逐周期追加。
- **边界与反直觉**：全部产物纯旁路——不进指向、不进识别、不进 SINR；`Rir_scan_pattern.csv` 是调度结果导出而非指向输入。
- **证据**：[evidence: tests/unit/remote_identification_radar/rir_acceptance_look_polar_test.cpp]

---

## 核心反直觉点与工程陷阱

> [!IMPORTANT]
> 1. **6 dB 回退门控的双轨语义**：`kSnrFallback` 模式以 $\text{SNR} \ge 6\text{ dB}$ 代替蒙特卡洛随机判决，且位置取真值（仅附带误差协方差）；`kDetectorGate` 才真正执行 CFAR 抽样与量测位置扰动。
> 2. **KF 速度种子已彻底去真值化（2026-08-31）**：场景速度种子任何环节都不消费——量测 velocity 字段填零、（重）初始化用零速均值 + 速度无知先验；加速度差分只取滤波后验速度按实际经过时间折算。旧口径（种子作差分基准/首拍起效）已废弃，勿据其排障。
> 3. **杂波脉压能量增益对齐**：目标与杂波回波均经同一匹配滤波，杂波噪声计算必须乘以 $\max(1, B\tau)$ 能量增益，否则会导致信杂比系统性虚高 $17.6\text{ dB}$（对于 $4.5\text{ MHz}$ 带宽）。
> 4. **大类得分非累加**：类别得分为类内最高型号分（$\text{argmax}$），避免数据库中某一类型号数量过多虚假抬高识别概率。
> 5. **波束指向不追目标**：方向图永远开启，调度器指向与目标视线角的差值直接导致增益跌落，RIR 绝不擅自将波束轴线吸附至目标。
> 6. **出口①透出原则**：透出点在质量门前，即低质量观测照常输出给外部融合，但库内不用于确认型号。
> 7. **关联键单调不回收**：内部航迹回收销毁时，全局键值不重置，新键天然代表新目标，无需防范回落碰撞。
> 8. **天线系统偏置施加在均值侧**：距离 20 m 与角度 $\text{bw}/30$ 属于系统残差偏置，必须在量测生成侧偏移均值，严禁与随机误差 std 简单平方和相加。

---

## 非目标（刻意不实现的算法）

1. **在线残差驱动的自适应学习与自动后端切换**：保持模型确定性与评测可复现性，严禁在线自适应调参。
2. **微动特征提取与 ISAR 图像重构**：本库定位在窄带/中等带宽宏观特征识别，不包含毫米波级微动成像。
3. **滑窗单元级 CA-CFAR**：采用物理参数化统计 CFAR 判决，不实现距离-多普勒二维参考单元滑窗。
4. **外部雷达波束交互控制**：驻留调度器派生的波位仅在库内自闭环消费，不反向控制外部雷达。
5. **电波视距与复杂地形衍射**：遮挡门采用圆球几何硬判定，不建模数字高程地形（DEM）与临边多径。
6. **ECCM 决策、反欺骗与战术决策**：RIR 定位为识别传感器，无烧穿/ECCM 决策链路（AR 专属能力），威胁结论与战术协调也不在本模块产生。
