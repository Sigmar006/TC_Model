---
Status: active
Last-reviewed: 2026-11-01
Authority: OPIR 与 RIR 的单周期探测主流程、关键公式、跨周期状态流转与下游链路
Answers: 两个模型一个周期各做什么、每步用什么公式、变量从哪来到哪去、哪些状态跨周期保存、检测记录之后如何融合推演评估
---

# 探测周期逐段拆解：OPIR 与 RIR

本文把 `sbirs_sensor`（实际建模对象是 OPIR 天基红外卫星）与 `remote_identification_radar`（远程识别雷达）的
**探测主流程**按周期顺序拆开：每一步做什么、用哪条公式、输入变量是什么、算完写回哪里。逐步实现读代码；
本文只做"流程 + 公式 + 变量流转"的串联。探测之后还有融合、推演、精度评估三段，也在第 5 节一并串起来。

阅读约定：

- **卫星称 OPIR**（过顶持续红外）。源码与配置沿用历史命名——模块路径 `sbirs_sensor`、类名前缀 `Sbirs*`、
  配置文件 `sbirs.json`——这些标识符在本文原样保留，正文一律称 OPIR。
- 标识符一律配中文：正文遇到不熟的类名、字段名、配置键，查 **0.3 标识符怎么读**；那里按命名规则、
  词根、类与函数、枚举、配置键五类给出中文。
- 公式用 LaTeX 书写，Typora 内置 MathJax 直接渲染；变量名、字段、枚举、函数仍用行内代码字体。
- 数学符号用行内公式（如 $\sigma_\theta$ 对应字段 `sigma_theta`），代码量用行内代码，两者不混。
- 每个公式后面跟一张符号表：符号 / 含义 / 代码或配置来源。来源写 `—` 表示该量在本模块内由公式直接算出，
  没有单独命名的字段。
- 含三元表达式、数组下标、函数调用的流程逻辑保留为伪代码块，不强行改写成公式。
- 标 `[common]` 的公式不在模块内实现，委托 `src/common/` 单源，模块只做参数装配。
- "前值 → 后值"指**同一变量在本周期开始与结束时的取值**，跨周期部分集中在第 3 节。

## 0. 两个模型的位置

| | OPIR | RIR |
|---|---|---|
| 周期入口 | `SbirsPipeline::RunCycle`（主周期入口） | `RirController::RunCycle`（主周期入口） |
| 一句话链路 | 扫描 → WFOV 四道门限 → 候选 → 调度 → 首次捕获 → NFOV 跟踪 | 驻留计划 → 候选裁剪（遮挡/角域）→ 主瓣覆盖 → 接收机饱和判定 → SINR/CFAR → 量测 → 关联 → 航迹 → 识别 |
| 门限所在参考系 | 传感器系（输出仍是 ECI 极坐标） | 平台 ENU（输出 ENU/ECEF） |
| 跨周期状态容器 | `SbirsPipeline`（主流程类）成员 + `SbirsPipelineSnapshot`（状态快照） | `RirTrackLifecycle`（航迹生命周期）/ `RirTracker`（识别）/ `RirController`（调度） |
| 探测对象 | 目标**辐射强度**（W/sr，外部给定） | 目标**雷达截面积（RCS）**（m²，可经物理模型按视角修正） |

两者共同的取舍：**物理量在 common，模块只做编排与标定**。OPIR 的辐射/噪声/误差、遮挡；RIR 的雷达方程、
检测单元、恒虚警率（CFAR）、量测误差、跟踪器、航迹提升，全部来自 `common/`，模块侧文件是薄适配。

### 0.1 用词依据

本文用词向下列公开标准对齐。只统一**名称与缩写**，不转抄定义原文；定义以标准正文为准。

| 代号 | 标准 | 本文用到的部分 |
|---|---|---|
| R1 | GB/T 3784-2009《电工术语 雷达》（代替 GB/T 3784-1983《雷达名词术语》） | 雷达检测、测量、跟踪类术语 |
| R2 | IEEE Std 686-2024《IEEE Standard for Radar Definitions》 | 英文对应词 |
| I1 | GJB 4238《军用目标特性和环境特性术语》 | 红外辐射特性、目标与环境 |
| I2 | GB 3102.6《光及有关电磁辐射的量和单位》 | 辐射量名称与单位 |
| I3 | GB/T 17444-2013《红外焦平面阵列参数测试方法》 | 探测器参数 |
| T1 | GJB 545-88《基本的卡尔曼滤波》 | 滤波与状态估计 |
| D1 | GJB 438C-2021《军用软件开发文档通用要求》 | 文档体例（仓库既有约定） |

### 0.2 术语对照

| 本文用语 | 英文 | 依据 | 说明 |
|---|---|---|---|
| 过顶持续红外（OPIR） | overhead persistent infrared | 通用 | 卫星名称；源码沿用 `sbirs` 历史命名 |
| 雷达截面积（RCS） | radar cross section | R1 | 不写"有效面积" |
| 检测概率 `P_d` / 虚警概率 `P_fa` | probability of detection / of false alarm | R1 | |
| 恒虚警率（CFAR） | constant false alarm rate | R1 | |
| 信噪比（SNR）/ 信号干扰噪声比（SINR） | signal-to-noise / signal-to-interference-plus-noise ratio | R1 | 分母含干扰、杂波时用 SINR |
| 脉冲重复频率（PRF）/ 脉冲重复间隔（PRI） | pulse repetition frequency / interval | R1 | |
| 脉冲压缩、相干积累 | pulse compression, coherent integration | R1 | |
| 动目标显示（MTI）/ 动目标检测（MTD） | moving target indication / detection | R1 | |
| 驻留（dwell） | dwell | R1 | 一次波束照射的时间段 |
| 半功率波束宽度（HPBW） | half-power beamwidth | R1 | 本文记 `θ_bw` |
| 主瓣 / 旁瓣 | mainlobe / sidelobe | R1 | |
| 杂波 / 干扰 | clutter / jamming | R1 | |
| 接收机饱和 | receiver saturation | R1 | 不属于候选裁剪门限 |
| 检测单元 | detection cell | R1 | 参与判决的最小单元 |
| 关联波门、新息、新息协方差 | association gate, innovation, innovation covariance | R1/T1 | 数据关联用 |
| 归一化新息平方（NIS） | normalized innovation squared | T1 | 滤波一致性检验量 |
| 航迹起始 / 确认 / 丢失 | track initiation / confirmation / loss | R1 | 对应代码 `tentative/confirmed/lost` |
| 卡尔曼滤波（KF）、扩展卡尔曼滤波（EKF）、交互式多模型（IMM）、无迹卡尔曼滤波（UKF） | KF, EKF, IMM, unscented KF | T1 | |
| 层次分析法（AHP）、判断矩阵、一致性比例（CR） | analytic hierarchy process, judgment matrix, consistency ratio | T1 | 五项精度指标定权 |
| 双星交会（双视线交会） | dual-satellite LOS fix | 通用 | 两条视线最近点中点定目标位置 |
| 落点 / 发射点 | impact point / launch point | R1 | 弹道外推与回推的端点 |
| 目标类型推演 | target type inference | 通用 | 运动学先验 + 外部证据加权 |
| 外推（coast） | coast | R1 | 无新量测时只做预测 |
| 辐射强度（W/sr） | radiant intensity | I1/I2 | OPIR 的输入量 |
| 辐射亮度（W/(m²·sr)）、辐照度（W/m²） | radiance, irradiance | I2 | 本文未直接使用，勿与辐射强度混称 |
| 大气透过率 | atmospheric transmittance | I1 | 本文记 `τ_geo` |
| 噪声等效功率（NEP） | noise equivalent power | I3 | |
| 积分时间、量子效率、光学透过率 | integration time, quantum efficiency, optical transmittance | I3 | |
| 视场（FOV）、瞬时视场（IFOV） | field of view, instantaneous FOV | 通用 | 宽/窄视场即 WFOV/NFOV |
| 视线（LOS）、视轴（boresight） | line of sight, boresight | R1 | |
| 地球遮挡 | Earth occultation | 通用 | 轨道几何术语，非雷达/红外专有 |
| 星下点 | subsatellite point / nadir | 通用 | 航天器几何术语 |
| 引导（cue）、跨星引导（cross-cue） | cue, cross-cue | R1 | |
| 捕获（acquisition） | acquisition | R1 | 雷达文献亦作"截获" |

"依据"列只表示**该名称可追溯到哪一类标准**；标"通用"的是航天器几何或传感器通用词，未在任何一份雷达/红外
术语标准中单独收录，本文按行业惯用写法使用。

已统一替换的口语写法：

| 旧写法 | 现写法 | 旧写法 | 现写法 |
|---|---|---|---|
| 拍 | 周期 | 四门 / 三门 | 四道门限 / 三道门限 |
| 丢锁 | 失锁 | 钳制 | 限幅 |
| 滑行 | 外推（coast） | 起批 | 起始 |
| 窄场 / 宽场 | 窄视场 / 宽视场 | 重捕 | 重新捕获 |
| 饱和闸 | 接收机饱和判定 | 免费多跟 | 同帧并行跟踪 |
| SINR 账本 | SINR 计算 | 物理涌现 | 自然形成 |
| 抽签命中 | 按检测概率抽样 | 门控 | 门限 |

### 0.3 标识符怎么读

正文保留源码标识符（改了就对不上代码），含义在这一节给全。几条命名规则看懂，长名字基本能自己拆：

| 形式 | 含义 | 例子 |
|---|---|---|
| `k` + 驼峰 | 枚举值或常量 | `kUndetected`、`kRangeMeasurementBiasM` |
| 末尾带 `_` | 类的成员变量 | `scan_phase_deg_`、`cue_predictor_` |
| `Is…` / `Has…` / `Can…` 开头 | 返回布尔 | `IsEarthOcculted`、`has_confirmed_track` |
| `Compute…` / `Resolve…` / `Build…` / `Make…` / `Sample…` / `Try…` 开头 | 计算 / 解析 / 构造 / 抽样 / 尝试 | `ComputeMaxDetectionRangeM` |
| 单位后缀 | `_deg` 度、`_m` 米、`_m2` 平方米、`_sec` 秒、`_hz` 赫兹、`_db` 分贝、`_w` 瓦、`_mps` 米每秒、`_rad` 弧度、`_linear` 线性值（非 dB） | `dwell_sec`、`rcs_m2`、`narrow_min_snr_linear` |

常见词根（按首字母大致分组，读到长名字逐个替换即可）：

| 词根 | 中文 | 词根 | 中文 |
|---|---|---|---|
| `wfov` / `nfov` | 宽视场 / 窄视场 | `scan` | 扫描 |
| `snr` / `sinr` | 信噪比 / 信号干扰噪声比 | `slew` | 限速转向 |
| `gate` | 门限、波门 | `steerable` | 可转向的 |
| `hit` / `miss` | 命中 / 失配 | `dwell` | 驻留 |
| `consecutive` | 连续 | `cue` | 引导 |
| `phase` | 相位 | `boresight` | 视轴 |
| `span` | 跨度 | `los` | 视线（line of sight） |
| `leg` | 往返扫描的一程 | `frame` | 帧 |
| `row` | 行 | `disturbance` | 扰动 |
| `wrap` / `fold` | 折回、环绕 | `track` | 航迹；跟踪 |
| `filter` | 滤波器 | `prior` / `posterior` | 先验 / 后验 |
| `innovation` | 新息 | `covariance` / `variance` | 协方差 / 方差 |
| `std` | 标准差 | `bias` | 偏置 |
| `noise` | 噪声 | `gain` / `loss` | 增益 / 损耗 |
| `occulted` | 被地球遮挡 | `eci` / `ecef` / `enu` | 地心惯性系 / 地心地固系 / 东-北-天系 |
| `rcs` | 雷达截面积 | `cfar` | 恒虚警率 |
| `mti` / `mtd` | 动目标显示 / 动目标检测 | `pulse` | 脉冲 |
| `prf` / `pri` | 脉冲重复频率 / 间隔 | `clutter` | 杂波 |
| `jam` | 干扰 | `tentative` / `confirmed` / `lost` | 候选 / 已确认 / 丢失 |
| `recognition` | 识别 | `profile` | 模板；距离像 |
| `quality` / `weight` | 质量 / 权重 | `margin` | 分差（第一名与第二名的差） |
| `lifecycle` | 生命周期 | `associate` / `assignment` | 关联 / 分配 |
| `augmented` | 扩充的（方阵补出漏检、新目标出口） | `aperture` | 孔径 |
| `radiant` | 辐射 | `transmittance` | 透过率 |
| `integration` | 积分（时间） | `readout` | 读出 |
| `quantum` | 量子 | `peak` | 峰值 |
| `saturation` | 饱和 | `plausible` | 合理的（限幅用） |

OPIR 的类与函数：

| 标识符 | 中文 | 作用 |
|---|---|---|
| `SbirsPipeline::RunCycle` | 主周期入口 | 一个探测周期从扫描到跟踪的全过程 |
| `SbirsPipelineSnapshot` | 跨周期状态快照 | 导出/恢复全部运行态，供回放与续算 |
| `SbirsTargetState` | 目标状态枚举 | 七态见 1.9 |
| `SbirsNfovScheduler` | 窄视场调度器 | 候选入列、排序、锁定集合 |
| `SbirsPointingActuator` | 指向执行器 | 按限速把镜筒转向指令方向 |
| `SbirsCuePredictor` | 引导预测器 | 用量测角差分外推指令延迟 |
| `SbirsTrackingCoordinator` | 跟踪协调器 | 分发 EKF / IMM / 角度 CV 三种滤波 |
| `SbirsBoresightChain` | 视轴链 | 传感器系指向 ↔ ECI 指向 |
| `SbirsRadiometry` | 辐射计算 | 接收功率、SNR、最大探测距离 |
| `SbirsNoiseModel` | 噪声模型 | 噪声能量合成（无热噪声项） |
| `SbirsErrorModel` | 误差模型 | 测角/测距误差与多帧融合 |
| `SbirsEnvironmentModel` | 环境模型 | 大气透过率（壳段弦长衰减） |
| `IsEarthOcculted` | 地球遮挡判定 | 视线是否被地球挡住 |
| `InRectangularFov` | 矩形视场判定 | 视线是否落在矩形视场内 |
| `IsNfovAcquisitionEligible` | 窄视场捕获资格判定 | 视场残差 + SNR 两条判据 |
| `AdvanceDisturbance` | 推进扰动 | 生成该帧共模扰动样本 |
| `SensorPointingForDesiredEciLos` | 由期望 ECI 视线反解传感器指向 | 惯性稳定模式用 |
| `PromoteToTracking` | 升级为跟踪 | 捕获成功后切跟踪态 |
| `RecordTrackingGateResult` | 记录跟踪门结果 | 通过清零、失败累加 |
| `CaptureRuntimeState` / `RestoreRuntimeState` | 导出 / 恢复运行态 | 快照存取 |
| `ComputeMaxDetectionRangeM` | 最大探测距离 | 由 SNR 门限反解 |
| `ComputeMaxVisibleRangeInFovM` | 视场内最大可见距离 | 视场几何能压到多远 |
| `ComputeRelativeAngularRateDegPerSec` | 相对视线角速度 | 用于动态滞后偏差 |

RIR 的类与函数：

| 标识符 | 中文 | 作用 |
|---|---|---|
| `RirController::RunCycle` | 主周期入口 | 一个识别周期的全链路 |
| `HoldCycle` | 维持周期 | 非识别模式下只维护航迹 |
| `ResolveRfCycle` | RF 周期解析 | PRF、PRI、脉冲数、接收链 |
| `RirEmissionFactory::ResolveCarrierHz` | 载频解析 | 支持跳频 |
| `RirResolveEffectiveBeamwidth` | 有效波束宽度 | 方向图与覆盖门限共用 |
| `RirTrackAssociator` | 航迹关联器 | 马氏距离 + 全局最优分配 |
| `RirTrackLifecycle` | 航迹生命周期 | 命中/失配、提升、丢弃 |
| `RirTrackFilter` | 航迹滤波器 | CV 卡尔曼 / IMM |
| `RirTrackState` | 航迹状态 | 位置、速度、协方差、计数 |
| `RirTrackStatus` | 航迹状态枚举 | 候选 / 确认 / 丢失 |
| `RecognitionMatcher` | 特征匹配器 | 高斯相似度与加权打分 |
| `RecognitionTracker` | 识别状态机 | 滑窗聚合与结论判定 |
| `DetectionCellResolver` | 检测单元解算 | SINR 的唯一实现（`[common]`） |
| `StatisticalCfarDetector` | 统计恒虚警检测器 | 按检测概率判决 |
| `BuildAugmentedSquareCostMatrix` | 扩充方阵代价矩阵 | 给漏检、新目标留出口 |
| `BuildMeasurementCovariance` | 构造量测协方差 | OPIR 侧多帧融合用 |
| `MakeCartesianMeasurementCovariance` | 极坐标 → 直角协方差 | 雅可比投影到 ENU |
| `SampleMeasurementPosition` | 量测位置采样 | 按协方差做 Cholesky 采样 |
| `TryGetLeastSquaresVelocity` / `TryGetLeastSquaresAcceleration` | 最小二乘速度 / 加速度 | 只进识别特征 |
| `DropTrack` | 丢弃航迹 | 发散航迹直接删，不标丢失 |
| `GaussianSimilarity` | 高斯相似度 | 逐维 $s_d$ |
| `MtiPowerResponse` | MTI 功率响应 | 2 脉冲对消器对目标的响应 |
| `RirDwellKind` | 驻留类型枚举 | 搜索 / 验证 / 跟踪 / 指定 |
| `RirDesignationPhase` | 指定任务阶段枚举 | 待生效 / 生效 / 已回扫 |
| `AntennaPatternRuntime` | 方向图运行时 | 三模型、分区增益、扫描损耗 |
| `ScanScheduleRuntime` | 波位运行时 | 波位栅格、驻留数、优先级、游标 |
| `ComputeMixedPhysicalRcsM2` | 混合物理 RCS | 光学区与谐振区按权重混合 |

下游链路的类与函数：

| 标识符 | 中文 | 作用 |
|---|---|---|
| `FusionEngine::Update` | 融合周期入口 | 关联 → 置信度 → 滤波 → 输出 |
| `FusedTarget` | 融合航迹 | 多源通道、置信度、生命周期、运动学估计 |
| `FusedTrackLifecycle` | 融合航迹状态 | 候选 / 已确认 / 外推 |
| `AssociateUnidentified` | 无身份记录关联 | 位置 / 方位 / 特征三级 |
| `UnscentedPredictor` / `UnscentedUpdater` | 无迹预测 / 更新 | 六维状态 UKF |
| `WriteFusionAcceptance` | 融合验收行 | 接力三项、精度、标准 KF 行 |
| `TargetInferenceEngine::Infer` | 推演入口 | 弹道、落点、发射点、类型 |
| `Rk4Step` | 四阶龙格–库塔单步 | 定步长积分，负步长即回推 |
| `InferenceTargetCategory` | 目标类别枚举 | 弹道 / 导弹 / 战斗机 / 轰炸机 / 临近空间 / 其他 |
| `PositionSigmaAt` | 任意时刻位置 1σ | 六扰动对角简化 |
| `PrecisionEvaluationSession` | 精度评估会话 | 双星交会、五项误差、综合分 |
| `TryComputeDualLosFixM` | 双视线交会 | 异面直线最近点中点 |
| `AhpEvaluator` / `TryEvaluateAhp` | AHP 求权 | 幂迭代 + 一致性检验 |
| `ComposePrecisionScore` | 综合分 | 权重 × 归一化得分 |
| `SummarizeErrorSeries` | 误差序列汇总 | 均值 / RMSE / P95 / 最大值 |

枚举与常量：

| 标识符 | 中文 |
|---|---|
| `kUndetected` / `kWideCandidate` / `kAwaitingNfovAcquisition` | 未探测 / 宽视场候选 / 等待窄视场捕获 |
| `kStrictTruthAssistedTracking` | 严格真值辅助跟踪（直接给真值视线） |
| `kSensorLikeTruthAssistedTracking` | 类传感器真值辅助跟踪（真值视线，按传感器口径出量测） |
| `kEstimatedTracking` | 估计跟踪（滤波输出） |
| `kEkf` / `kImm` / `kAngleCvKf` | 扩展卡尔曼 / 交互式多模型 / 角度匀速卡尔曼 |
| `kDetector` / `kSnrFallback` | 检测器判决 / SNR 兜底门 |
| `kStandby` / `kIdentify` | 待机 / 识别工作模式 |
| `kTentative` / `kConfirmed` / `kLost` | 候选 / 确认 / 丢失 |
| `kTentative` / `kConfirmed` / `kCoasting`（融合层） | 候选 / 已确认 / 外推中 |
| `kAccumulating` / `kUnknown` | 识别累积中 / 未知 |
| `kModelConfirmed` / `kCategoryConfirmed` | 型号确认 / 大类确认 |
| `kTargetEarthOcculted` | 排除原因：地球遮挡 |
| `kTargetOutsideSearchVolume` | 排除原因：在搜索扇区/可扫描体积外 |
| `kTargetOutsideBeamCoverage` | 排除原因：没被任何驻留波束覆盖 |
| `kTargetReceiverFrontEndSaturated` | 排除原因：接收机饱和 |
| `kTargetModeNotIdentify` | 排除原因：非识别工作模式 |
| `kMtiNoiseFactor` / `kCovarianceFloorM2` | MTI 噪声系数（=2）/ 协方差正则项（$10^{-6}$） |
| `kRangeMeasurementBiasM` / `kMaxPlausibleAccelerationMps2` | 距离量测偏置 20 m / 加速度合理上限 100 m/s² |

主要配置键：

| 配置键 | 中文 |
|---|---|
| `narrow_min_snr_linear` / `wide_min_snr_linear` | 窄视场 / 宽视场最低 SNR（线性值） |
| `wide_to_narrow_required_consecutive_hits` | 宽视场转窄视场所需连续命中数 |
| `nfov_tracking_gate_loss_cycles` | 跟踪门连续失败多少周期判失锁 |
| `nfov_rotation_step_` | 窄视场轮转步数（取模选目标） |
| `nis_gate_loss_cycles` | NIS 连续超限多少周期判失锁 |
| `distance_gate_sigma2` / `gate_threshold` | 关联波门（以 σ² 计）/ 实际门限值 |
| `cfar_pfa` / `swerling_type` | 恒虚警率 / 目标起伏模型类型 |
| `min_snr_db` / `min_detection_margin_db` | 最低 SNR 门 / 最小检测余量（dB） |
| `accumulation_window_sec` | 识别累积窗时长（缺省 10 s） |
| `confirm_hits` / `max_miss_before_lost` | 确认所需命中数 / 判丢失前允许的连续失配数 |
| `initial_state_variance` / `initial_velocity_std_mps2` | 航迹初始位置方差 / 初始速度标准差 |
| `sensor_scan_limits_deg` / `scan_window_deg` / `steerable_volume_deg` | 传感器扫描限位 / 任务搜索子窗 / 可转向体积 |
| `enable_mti_mtd_effects` | MTI/MTD 联算开关 |
| `position_radius_m` / `bearing_beamwidth_deg` | 融合空间门限 1000 m / 方位相干门限 5° |
| `window_size` / `max_missed_cycles` | 每源滑窗 10 拍 / 失跟删除门 5 周期 |
| `source_weights` | 按 `source_id` 索引的源权重，缺项按 1.0 |
| `track_cycle_period_sec` | 融合周期时长（滤波 `dt` 与验收行时间基准） |
| `relay_fov_width_deg` | 接力覆盖估算用的视场角宽度（默认 20°） |
| `prediction_horizon_sec` / `integration_step_sec` | 推演时域 300 s / 积分步长 1 s |
| `launch_atmosphere_interface_altitude_m` | 发射点回推的大气界面停机高（100 km） |
| `dual_sat_pair_window_cycles` | 双星交会配对窗（0 = 严格同周期） |
| `reference_error_*` | 五项指标的评分参考误差 |

---

## 1. OPIR：一个周期

### 1.1 周期骨架

`SbirsPipeline::RunCycle`（`pipeline/SbirsPipeline.cpp`）按固定顺序执行：

1. **时间与坐标**：`sim_time_sec = cycle_index · dt_sec`；ECEF → ECI（含 `ω×r` 输运项）。
2. **待机短路**：`work_mode == kStandby` 或 `sensor_enabled == false` → 清空全部目标状态与调度器，只回扫描中心。
3. **扫描基准与跨度**：星下点相对模式下按"配置扇区 ∩ 地球可见窗"裁剪有效跨度 `scan_wrap_span_deg_`。
4. **指向扰动推进**：`AdvanceDisturbance(dt)` → 本帧共模扰动样本 `frame_disturbance`。
5. **扫描推进**：相位/腿方向/行索引三件套更新（见 1.2）。
6. **逐目标 WFOV 段**：遮挡 → 距离带 → SNR 计算 → （锁定目标只缓存上下文）→ 视场门限 → SNR 门限 → 构造候选。
7. **候选调度**：连续命中数达标者入锁定集合，按 SNR↓/距离↑/ID↑ 排序。
8. **单镜筒轮转**：`served = engaged[step % size]`，限速转向 + 稳定时长 → 帧数。
9. **首次捕获**：视场残差 + SNR 门限（候选 SNR 比窄视场门限）双判据；成功则进入三种跟踪模式之一。
10. **跟踪段**：几何/SNR/帧数三道门限 → 多帧融合量测 → 扩展卡尔曼滤波（EKF）/交互式多模型（IMM）/角度 CV（匀速）滤波 → 归一化新息平方（NIS）门限 → 失锁判定。

### 1.2 扫描推进：往复腿 + 俯仰行栅格

`ScanRowCount` / `RowCenterEl` 定义 2-D 栅格（$\Delta_{el}$ 即 `span_el`，$\delta_{el}$ 即 `step_el`）：

$$
N_{row} = 1 + \left\lfloor \frac{\Delta_{el}}{\delta_{el}} \right\rfloor ,
\qquad
\theta_{el}(r) = \theta_{el,start} + r\,\delta_{el}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $N_{row}$ | 俯仰方向的行数 | 由 `ScanRowCount` 算出 |
| $\Delta_{el}$ | 俯仰扫描总跨度 | `span_el` |
| $\delta_{el}$ | 行间俯仰步进 | `step_el` |
| $\theta_{el}(r)$ | 第 $r$ 行中心的俯仰角 | `RowCenterEl(r)` |
| $\theta_{el,start}$ | 首行俯仰角 | `scan_el_start` |
| $r$ | 行索引，$0 \le r < N_{row}$ | `scan_row_index_` |

`span_el = 0` 退化为单行，中心恒为 `scan_center_el_deg`。

扫描相位在"行程坐标"上推进，到边反射而不是跳回：

```text
travel_prev   = forward ? phase : 2·span − phase
travel_new    = travel_prev + rate · max(0, dt)
row_advance   = ⌊travel_new/span⌋ − ⌊travel_prev/span⌋
travel_folded = fmod(travel_new, 2·span)
forward_new   = travel_folded < span
phase_new     = forward_new ? travel_folded : 2·span − travel_folded
phase_azimuth = NormalizeAzimuth(base + start + sign · phase_new)
```

逐行读法：

- `span`：本周期有效跨度（`scan_wrap_span_deg_`）；`phase`、`forward`：相位与腿方向（`scan_phase_deg_`、`scan_leg_forward_`）。
- `travel_*`：把往复折叠展开成单调前进的"行程坐标"，`travel_prev` 先按当前腿方向折算。
- `row_advance`：本周期跨过几条腿，用来推进行号。
- `travel_folded`：折回 $[0,\,2\,span)$；`forward_new`、`phase_new`：再折成"腿方向 + 相位"。
- `phase_azimuth`：把扇区基准、起始偏置和方向符号叠到相位上，得到本周期输出的方位角。

| 变量（中文） | 前值 | 后值 |
|---|---|---|
| `scan_phase_deg_`（扫描相位） | 上一周期末相位，`[0, span)` | `phase_new`，仍在 `[0, span)` |
| `scan_leg_forward_`（扫描腿方向） | 上一周期腿方向 | `travel_folded < span` |
| `scan_row_index_`（扫描行号） | 当前行 | `(row + row_advance) mod row_count`（`row_count=1` 不变） |
| `scan_wrap_span_deg_`（有效扫描跨度） | 上一周期有效跨度 | 本周期裁剪结果（nadir 模式随位置漂移；absolute 恒等于配置值） |

要点：`rate = 0` 时 `travel ≡ 0`，相位与腿方向都不动，等价于凝视。

### 1.3 指向合成与扰动

- 名义扫描中心：传感器系 `(scan_azimuth_deg, RowCenterEl(row))`；输出侧再经 `SbirsBoresightChain`
  转成 ECI 方位/俯仰（恒等链下逐位不变）。
- 实际扫描中心：叠加本帧**共模扰动** `frame_disturbance.common`，再按 `sensor_scan_limits_deg` 限幅。
- 惯性稳定模式下先把期望 ECI 视线反解到传感器系（`SensorPointingForDesiredEciLos`），再叠扰动。

| 变量（中文） | 前值 | 后值 |
|---|---|---|
| `actual_scan_azimuth_sensor_deg`（实际扫描方位角，传感器系） | — | 名义相位角 + 共模扰动，限位后 |
| `nominal_scan_azimuth_sensor_deg`（名义扫描方位角，传感器系） | — | **扰动前**名义角（NFOV 执行器初值用） |

#### 1.3.1 视轴合成链（安装 ∘ 姿态 ∘ 失准；扫描角为链的输入）

传感器系的角要先经过"星体姿态 + 安装角 + 安装失准"三级旋转，才能变成 ECI 方向。`SbirsBoresightChain`
只做纯几何，随机量在 `SbirsPipeline` 里抽：

$$
\mathbf u_s=\bigl(\cos el\,\cos az,\ \cos el\,\sin az,\ \sin el\bigr),
\qquad
\mathbf u_{eci}=R_{b\to eci}\,R_{s\to b}\,R_{mis}^{\top}\,\mathbf u_s
$$

$$
\mathbf u_{s}=R_{mis}\,R_{s\to b}^{\top}\,R_{b\to eci}^{\top}\,\mathbf u_{eci}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\mathbf u_s$ | 传感器系单位视线向量 | `SbirsVector3M` |
| $az$、$el$ | 传感器系方位角、俯仰角（deg） | `actual_scan_azimuth_sensor_deg` 等 |
| $R_{b\to eci}$ | 星体 → ECI 的姿态旋转（Z-Y-X 欧拉角） | `attitude_eci_body_deg` |
| $R_{s\to b}$ | 传感器 → 星体的安装旋转 | `mount_angles_deg` |
| $R_{mis}$ | 安装失准旋转，作用于传感器系内 | `misalignment_total_deg_` |
| $\mathbf u_{eci}$ | ECI 单位视线向量，输出侧使用 | — |

第二式是**反解**：惯性稳定模式下先给期望 ECI 光轴，再退回传感器系去指挥执行器（`SensorPointingForDesiredEciLos`）。
零姿态、零安装、零失准时两式都是恒等变换，历史结果逐位不变。

安装失准是**运行期一次**的常值抽样（`DrawMisalignmentTotal`）：

$$
\theta_{mis}=\theta_{bias}+\sigma_{mis}\,\mathcal N(0,1)
\quad\text{（偏航/俯仰/横滚各抽一次）}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\theta_{bias}$ | 安装失准常值偏置（三轴） | `misalignment.bias_deg` |
| $\sigma_{mis}$ | 失准随机微扰标准差 | `misalignment.random_sigma_deg` |
| $\mathcal N(0,1)$ | 标准正态抽样，用配置种子构造独立随机流 | `misalignment.random_seed` |
| $\theta_{mis}$ | 三轴失准角总量 | `misalignment_total_deg_` |

$\sigma_{mis}=0$ 时直接取偏置、不消耗随机流；否则每次调用都用**同一种子**重抽，所以配置不变时整场恒定，
回放与续算逐位一致。

**失准只进几何，不进量测**：它改变实际光轴足迹与门控（目标是否真在视场内），但不叠加到输出角上，
也不进 `BuildMeasurementCovariance`。也就是说"量测误差"和"指向偏差"是两条独立的账。

| 变量（中文） | 前值 | 后值 |
|---|---|---|
| `misalignment_total_deg_`（三轴失准总量） | — | 构造/改配置时一次抽取，运行期不变 |
| 链上实际光轴（ECI） | 上周期指向 | 姿态∘安装∘失准∘本周期扫描角 |

### 1.4 WFOV 四道门限

四道门限按固定顺序，任何一道失败即清该目标的连续命中、释放调度/指向/预测/滤波，状态转 `kUndetected`：

1. **地球遮挡**（`IsEarthOcculted`，`[common]`；OPIR 侧包了一层 `foundation::`）。
2. **距离带**：`min_range_m` $\le R \le$ `max_range_m`。
3. **WFOV 视场**：目标视线转入传感器系后在矩形视场内（`InRectangularFov`，与**实际**扫描中心比）。
4. **WFOV SNR**：$\mathrm{SNR} \ge$ `wide_min_snr_linear`。

例外是**跨星引导（cross-cue）**：视场门限或 SNR 门限失败、但该目标带外部引导时，不走排除/释放，
而是在同一候选池里构造"外部引导候选"——用来源星的带误差量测（测角 + 距离 + 来源星位置）
三角化出本星视线，连续命中计数与排序规则与自星候选完全一致。

**辐射链（前两式在 `foundation/SbirsRadiometry.cpp`，噪声在 `foundation/SbirsNoiseModel.cpp`）**：

$$
\begin{aligned}
A &= \pi d^{2}/4 \\
P_r &= I\,A\,\tau_{opt}\,\tau_{geo}\,\eta\,/\,R^{2} \\
E_s &= P_r\,t_{int} \\
N_{eff} &= \sqrt{(NEP\cdot t_{int})^{2}+N_{photon}^{2}+N_{readout}^{2}} \\
\mathrm{SNR} &= E_s / N_{eff}
\end{aligned}
$$

单位：$P_r$ 取 W，$E_s$、$N_{eff}$ 取 J。

| 符号 | 代码变量 | 说明 |
|---|---|---|
| $A$ | — | 接收孔径面积，$A=\pi d^{2}/4$，由 $d$ 算出 |
| $I$ | `target.radiant_intensity_w_per_sr` | 目标辐射强度，外部输入，OPIR 不做 Planck 换算 |
| $d$、$\tau_{opt}$、$\eta$、$t_{int}$ | `hardware.optical_aperture_m` / `optical_transmission` / `detector_quantum_efficiency` / `integration_time_sec` | 孔径直径、光学透过率、量子效率、积分时间 |
| $\tau_{geo}$ | `path_transmittance` | 大气透过率，见下 |
| $P_r$ | — | 入瞳接收功率，W |
| $E_s$ | — | 积分时间内的信号能量，J |
| $N_{eff}$ | — | 有效噪声能量，三项噪声平方和开方，J |
| $NEP$ | `hardware.noise_equivalent_power_w` | 探测器噪声等效功率，承担全部探测器噪声 |
| $N_{readout}$ | `readout_noise_rms_w` $\times\, t_{int}$ | 读出噪声折算到能量 |
| $N_{photon}$ | $\sqrt{P_{bg}\,t_{int}\,E_{ph}}$ | 背景光子噪声，$E_{ph} = hc/\lambda_c$ |
| $\mathrm{SNR}$ | — | 信号噪声比，$E_s / N_{eff}$，写入候选供门限使用 |

大气透过率按"穿过大气壳的弦长"做指数衰减（`environment/SbirsEnvironmentModel.cpp`）：

$$
X = \min\!\left(\frac{L_{chord}}{100\ \mathrm{km}},\, X_{max}\right),
\qquad
\tau_{geo} = \tau_{eff}^{\,X}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $L_{chord}$ | 视线与大气壳两交点之间的弦长 | 由平台与目标几何算出 |
| $X$ | 归一化弦长：每 100 km 记 1 个"壳厚"，并被 $X_{max}$ 限幅 | — |
| $X_{max}$ | 归一化弦长上限 | 环境配置 |
| $\tau_{eff}$ | 单个壳厚的等效透过率，按 $X$ 次幂外推 | 环境配置 |
| $\tau_{geo}$ | 整条路径的大气透过率 | `path_transmittance` |

纯空间路径 $X = 0$，因此 $\tau_{geo} = 1$。

**热噪声显式不建模**：`SbirsNoiseModel.cpp` 里没有 `kTB` 项，探测器噪声由 NEP 承担。日志里写"未建模(由NEP承担)"，
不要读成"热噪声为 0 J"。

**最大探测距离**（`ComputeMaxDetectionRangeM`，由 SNR 门限反解）：

$$
d_{max}^{rad} = \sqrt{\frac{I\,A\,\tau_{opt}\,\tau_{geo}\,\eta\,t_{int}}{N_{eff}\,\mathrm{SNR}_{min}}},
\qquad
d_{max} = \min\!\left(d_{max}^{rad},\ d_{max}^{mission},\ d_{max}^{fov}\right)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $d_{max}^{rad}$ | 由 SNR 门限反解出的辐射口径最大距离 | `ComputeMaxDetectionRangeM` |
| $\mathrm{SNR}_{min}$ | 判"能探测"所需的最低 SNR | 探测门限配置 |
| $d_{max}^{mission}$ | 任务允许的最大斜距 | `mission.max_range_m` |
| $d_{max}^{fov}$ | 视场几何能覆盖的最大斜距 | `ComputeMaxVisibleRangeInFovM` |
| $d_{max}$ | 三者最小值，即对外报告的最大探测距离 | — |

`d_max_fov` 由"视场矩形 17×17 采样点与目标地心半径球面的交点"取最大斜距（`ComputeMaxVisibleRangeInFovM`），
表示"这个视场最远能压到哪"。三者取最小，避免把辐射够远但视场盖不到的距离报成可探测。

**目标可探测性与动态参数生成**（需求 3.2.1.2.2）：本节所述四道门限并非固定常量——SNR 门限随目标辐射强度和大气透过率动态变化，最大探测距离三门（辐射/任务/视场几何）取最小，视场门限依赖当前扫描中心。窄视场调度优先级（§1.6）也会参考 WFOV 阶段的 SNR 作为可探测性排序依据。这些共同构成 OPIR 对"每个周期、每个目标的可探测性评估与参数动态调整"。

### 1.5 带误差量测与引导预测

通过四道门限后构造候选，位置由**带误差量测**给出（`foundation/SbirsErrorModel.cpp`）：

$$
\begin{aligned}
\sigma_{\theta} &= \sqrt{\sigma_{orbit}^{2} + \sigma_{attitude}^{2} + \sigma_{fov}^{2}} \\
\delta_{ref} &= \frac{1.5\times10^{-6}}{R\,\lvert\cos\beta\rvert} \\
\delta_{lag} &= \frac{\omega}{2\pi B_{det}} \\
\mathrm{az}_{meas} &= \mathrm{az}_{true} + \mathcal{N}(0,\sigma_{\theta}) + \delta_{ref} + \delta_{lag} \\
\mathrm{el}_{meas} &= \mathrm{el}_{true} + \mathcal{N}(0,\sigma_{\theta}) + \delta_{ref} + \delta_{lag} \\
R_{meas} &= R_{true}\left(1 + \mathcal{N}(0,\sigma_{range\_frac})\right)
\end{aligned}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\sigma_{orbit}$ | 轨道/星历误差引起的测角标准差 | 误差配置 |
| $\sigma_{attitude}$ | 姿态确定误差 | 误差配置 |
| $\sigma_{fov}$ | 视场与像元量化误差 | 误差配置 |
| $\sigma_{\theta}$ | 三类误差的 RSS 合成，单轴测角标准差 | — |
| $\delta_{ref}$ | 大气折射引入的角偏差 | — |
| $\delta_{lag}$ | 指向动态滞后引入的角偏差 | — |
| $\beta$ | 视线天顶角，越小折射路径越长 | 由几何算出 |
| $B_{det}$ | 决定动态滞后时间常数的带宽参数 | 硬件配置 |
| $\omega$ | 目标相对卫星的视线角速度 | 见下方 |
| $\mathcal{N}(0,\sigma)$ | 零均值、标准差 $\sigma$ 的高斯抽样 | 随机源 |
| $\sigma_{range\_frac}$ | 距离量测的相对误差比例 | 误差配置 |
| $\mathrm{az}_{meas}$、$\mathrm{el}_{meas}$ | 加噪后的方位/俯仰量测角 | 写入候选 |
| $R_{meas}$ | 加噪后的斜距量测 | 写入候选 |

候选携带的 `snr` 是宽视场链路算出的物理 SNR；1.7 的捕获门限拿它比窄视场门限 `narrow_min_snr_linear`，
并不重算窄视场 SNR。外部引导候选同样沿用本星物理链路 SNR，只有指向角来自三角化。

相对视线角速度 $\omega$ 用垂直分量算：$\omega = \lvert v_{\perp}\rvert / R$（`ComputeRelativeAngularRateDegPerSec`），
$v_{\perp} = v_{rel} - (v_{rel}\cdot\hat{u})\,\hat{u}$，$v_{rel} = v_{target} - v_{sat}$（目标无速度时取 0，
卫星运动本身也会产生 $\omega$）。

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $v_{rel}$ | 目标相对卫星的速度差 | 目标状态减卫星状态 |
| $\hat{u}$ | 卫星指向目标的单位视线向量 | 由位置差归一化 |
| $v_{\perp}$ | $v_{rel}$ 中垂直于视线的分量，只有它产生角运动 | — |
| $R$ | 目标斜距 | — |
| $\omega$ | 视线角速度，$\delta_{lag}$ 的输入 | — |

引导预测器 `SbirsCuePredictor::Update` 用连续两个周期量测角差分出角速率，再外推指令延迟：

$$
\dot a = \frac{\Delta a_{meas}}{\Delta t},
\qquad
a_{cmd} = \operatorname{normalize}\!\left(a_{meas} + \dot a\,t_{lat}\right)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\Delta a_{meas}$ | 相邻两周期量测角之差 | — |
| $\Delta t$ | 两周期的时间间隔 | `dt_sec` |
| $\dot a$ | 由差分得到的角速率 | — |
| $a_{meas}$ | 本周期量测角 | `measured_azimuth_deg` 等 |
| $t_{lat}$ | 引导指令延迟，需要提前外推的时间 | `narrow_cue_latency_s` |
| $a_{cmd}$ | 外推并归一化后的指向指令 | `command_azimuth_deg` |

本周期这一步写入的跨周期量：

| 变量（中文） | 前值 | 后值 |
|---|---|---|
| `cue_predictor_` 内该目标 `measured_azimuth_deg`（引导预测器记录的量测方位角） | 上周期量测角 | 本周期量测角 |
| `candidate.command_azimuth_deg`（候选的指向指令角） | — | 外推后的指向指令 |
| `wfov_consecutive_hits_[id]`（该目标宽视场连续命中数） | `n` | `n+1`（四道门限全过或外部引导候选） |

### 1.6 调度与单镜筒轮转

- **入列条件**：未被锁定，且 `wfov_consecutive_hits ≥ wide_to_narrow_required_consecutive_hits`（默认 1）。
- **排序**：SNR 降序 → 距离升序 → `target_id` 升序（`SbirsNfovScheduler::SelectForAcquisition`），保证回放可复现。
- **锁定集合无配置上限**：能同时精跟几条由轮转机制自然形成，不由通道数决定。

轮转（`RunCycle` 中段）：被服务目标取 `engaged_ids[nfov_rotation_step_ % engaged_ids.size()]`，
随后 `nfov_rotation_step_` 自增 1。

只有一个镜筒（单一窄视场指向通道），所以每周期只服务一个方向；**当前视场内的其他锁定目标同帧并行跟踪**（共享同一批稳定帧）。

执行器（`SbirsPointingActuator::Step`）按限速转向，并给出"本周期内进入容差后的剩余时长"：

```text
angle   = acos( current_los · command_los )
max_step= slew_rate · dt
next_los= angle > max_step ? rotate(current, axis, max_step) : command
slew_s  = max(0, (angle − tol) / slew_rate)
settled_duration = max(0, dt − slew_s)
settled_frames   = round( frame_rate_hz · settled_duration )
```

- `slew_rate`：执行器最大角速率；`dt`：本周期时长；`tol`：到位容差角。
- `max_step`：本周期最多能转过的角度；超过就只转这么多，否则直接对正。
- `slew_s`：本周期内"还在转"的时间；`settled_duration`：转到位后剩余时间。
- `settled_frames`：把剩余时间折算成帧数，供 1.8 的"稳定帧"门限使用。

| 变量（中文） | 前值 | 后值 |
|---|---|---|
| `pointing_coordinator_` 执行器 `current_los`（当前视线） | 上周期末指向 | `next_los` |
| `nfov_rotation_step_`（窄视场轮转步数） | `s` | `s + 1`（锁定集合非空才推进） |

### 1.7 首次捕获门限

`IsNfovAcquisitionEligible` 只有两条判据（都在传感器系）：

$$
\begin{aligned}
\Delta a_{az} &= \operatorname{wrap}_{180}\!\left(a_{az}^{delayed} - \left(a_{az}^{cmd} + e_{settle}\right)\right) \\
\Delta a_{el} &= a_{el}^{delayed} - a_{el}^{cmd} \\
\text{in\_fov} &= \left(\lvert\Delta a_{az}\rvert \le \mathrm{FOV}_{az}/2\right) \wedge \left(\lvert\Delta a_{el}\rvert \le \mathrm{FOV}_{el}/2\right) \\
\text{captured} &= \text{in\_fov} \wedge \left(\mathrm{SNR} \ge \mathrm{SNR}_{narrow,min}\right)
\end{aligned}
$$

其中 $\mathrm{SNR}_{narrow,min}$ 即 `narrow_min_snr_linear`。

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\Delta a_{az}$、$\Delta a_{el}$ | 目标真值角与指令角的视场残差 | — |
| $a^{delayed}$ | 按引导延迟外推后的真值角 | `delayed_truth_*` |
| $a^{cmd}$ | 本周期指向指令角 | `command_azimuth_deg` 等 |
| $e_{settle}$ | 执行器到位后的稳定误差 | 执行器模型 |
| $\mathrm{FOV}_{az}$、$\mathrm{FOV}_{el}$ | 窄视场矩形的方位/俯仰尺寸 | 硬件配置 |
| $\mathrm{SNR}_{narrow,min}$ | 窄视场捕获所需的 SNR 门限 | `narrow_min_snr_linear` |
| `in_fov` | 目标是否落在窄视场内的布尔量 | — |
| `captured` | 捕获成功与否的布尔量 | — |

`delayed_truth_*` 是"按引导延迟外推后的真值角"（目标与卫星各自前进 `narrow_cue_latency_s`），
用于回答"等镜头转到位时，目标还在不在窄视场内"。

捕获成功后：`cue_predictor_.Release` → `PromoteToTracking` → **清零连续命中** → 按 `tracking_mode` 进入
`kStrictTruthAssistedTracking` / `kSensorLikeTruthAssistedTracking` / `kEstimatedTracking`。

### 1.8 跟踪段：三道门限 + 多帧融合 + NIS 门限

每个锁定目标在跟踪段过三道门限：

| 门限 | 判据 | 变量 |
|---|---|---|
| 几何 | 目标传感器系角在实际指向的窄视场内 | `geometry_gate_passed` |
| SNR | $\mathrm{SNR} \ge$ `narrow_min_snr_linear` | `snr_gate_passed` |
| 稳定帧 | `settled_frames` $\ge 1$ | `nfov_frame_count` |

`nfov_frame_count = (几何 && SNR) ? settled_frames : 0`；`0` 表示本周期无窄视场帧 → **外推**，不产生融合量测。
连续门限失败计数由 `RecordTrackingGateResult` 维护（通过清零、失败 +1），达到
`nfov_tracking_gate_loss_cycles` 即失锁。

多帧融合把 $N$ 帧独立抽样平均，噪声按 $1/\sqrt{N}$ 收缩：

$$
\sigma_{single} = \sqrt{\sigma_{orbit}^{2} + \sigma_{attitude}^{2} + \sigma_{fov}^{2}},
\qquad
\sigma_{fused} = \frac{\sigma_{single}}{\sqrt{N}},
\qquad
R_{meas} = \frac{1}{N}\operatorname{BuildMeasurementCovariance}\!\left(\sigma_{g},\delta_{ref},\delta_{lag}\right)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\sigma_{single}$ | 单帧测角标准差，三类误差 RSS | — |
| $\sigma_{fused}$ | $N$ 帧平均后的测角标准差，按 $1/\sqrt{N}$ 收缩 | — |
| $N$ | 参与融合的稳定帧数 | `nfov_frame_count` |
| $\sigma_{g}$ | 高斯测角误差项（已含轨道/姿态/视场合成） | 误差模型 |
| $R_{meas}$ | 融合后的量测协方差，直接进滤波器 | `BuildMeasurementCovariance` |

跟踪器三后端（`SbirsTrackingCoordinator`）：

- `kEkf`：ECI 六维状态扩展卡尔曼滤波（EKF），量测为方位/俯仰两维角度。
- `kImm`：交互式多模型（IMM），每个模型各自算 NIS，取**最大模型 NIS** 作本周期 NIS。
- `kAngleCvKf`：只用量测角的线性 CV 卡尔曼，禁止三维真值入状态。

归一化新息平方（NIS）双门限（阈值按**卡方分布 2 自由度**取）：

| 门限 | 阈值 | 动作 |
|---|---|---|
| 统计门限 | $\mathrm{NIS} > 5.99$（95%） | 计数 +1，达到失锁周期数则失锁 |
| 离群门限 | $\mathrm{NIS} > 1000$（约 $31\sigma$）或协方差奇异 | EKF / 角度 CV：丢弃本周期量测，后验保持预测；IMM：整条滤波状态作废，退回宽视场重新捕获 |
| 兜底 | 配置 `nis_gate_loss_cycles = 0` | 仍按连续 3 个周期失锁 |

失锁动作统一为：状态回 `kWideCandidate`、释放调度/指向/滤波，下一周期需重新积累连续命中。

### 1.9 状态机

`SbirsTargetState` 七态：未探测（`kUndetected`）→ 宽视场候选（`kWideCandidate`）→ 等待窄视场捕获
（`kAwaitingNfovAcquisition`）→ 三种跟踪态（`kStrictTruthAssistedTracking` 严格真值辅助 /
`kSensorLikeTruthAssistedTracking` 类传感器真值辅助 / `kEstimatedTracking` 估计跟踪）。
任一道门限失败或目标消失落回未探测或丢失；失锁回宽视场候选。目标从场景消失（`!active` 或不在
`present_target_ids`）直接 `kLost` 并释放全部资源。

### 1.10 输出层与事件层

一个周期对外交三样东西：**原生检测记录**（raw output，不含真值）、**归属/诊断记录**（调试层，含真值
归属）、**问题清单**（校验/执行问题）。三者刻意分层，混读会把仿真信息当成传感器能力。

周期状态（`SbirsCycleStatus`）与中止原因（`SbirsPipelineAbortReason`）：

| 枚举 | 含义 | 典型触发 |
|---|---|---|
| `kCompleted` | 周期正常完成 | 主流程跑完 |
| `kPoweredOff` | 设备关机/待机，核心 pipeline 未执行 | `sensor_enabled == false` 或 `kStandby` |
| `kRejectedInvalidInput` | 输入校验失败 | 配置/场景校验拒绝 |
| `kRejectedExecution` | 执行失败 | 运行期异常 |
| `kNone` / `kValidationRejected` / `kSensorPoweredOff` | 中止原因：无 / 校验拒绝 / 关机 | 与上面三态一一对应 |

原生检测记录（`SbirsDetectionRecord`，每目标每阶段一条）：

| 字段 | 中文 | 说明 |
|---|---|---|
| `detection_id` | 检测编号 | 帧内唯一，`next_detection_id_` 递增 |
| `measurement_time_s` | 量测时刻 | 绝对仿真时间，不是周期号 |
| `azimuth_rad` | 方位角 | ECI 极坐标 $\operatorname{atan2}(y,x)$，$[0,2\pi)$ |
| `elevation_rad` | 仰角 | ECI 极坐标 $\arcsin(z/r)$，$[-\pi/2,\pi/2]$ |
| `infrared_snr_linear` | 红外线性 SNR | 物理链路算出的值，非 dB |
| `observation_stage` | 观测阶段 | 宽场搜索 / 窄场捕获 / 窄场跟踪 |
| `detected` | 是否检出 | 门限判决结果 |

注意输出角是**地心惯性系极坐标**，不是卫星局部地平系，也不是传感器系；与视场门限用的传感器系角
差着整条 1.3.1 的旋转链。

归属记录（`SbirsDetectionAttributionRecord`）把检测编号接回场景目标，并带全部诊断量：

| 字段组 | 字段 | 中文 |
|---|---|---|
| 归属 | `target_id`、`target_name` | 场景目标 ID / 名称 |
| 距离 | `estimated_range_m` | 估计距离（宽场取真值、窄场取融合估计；**不是**被动红外测距能力） |
| 距离 | `measured_range_m` | 误差模型给出的带误差距离（宽场段即候选量测） |
| 距离 | `max_detection_range_m` | 当前时刻有效最大探测距离 $d_{max}(t)$，逐周期变 |
| 跟踪 | `tracking_source` | 未建立 / 估计 / 严格真值辅助 / 类传感器真值辅助 |
| 跟踪 | `capture_failure_reason` | 捕获/跟踪失败原因（六选一） |
| 滤波 | `estimation_nis`、`estimation_nis_gate_exceeded` | EKF 的 NIS 及其是否超 2 维 95% 门限 |
| 窄场 | `nfov_channel_id`、`nfov_frame_count` | 通道号（`-1` = 未占用）、本周期帧数 |
| 窄场 | `nfov_frame_sigma_deg`、`nfov_fused_sigma_deg` | 单帧角噪声、多帧融合后角噪声 |
| 门限 | `nfov_geometry_gate_passed`、`nfov_snr_gate_passed` | 几何门、SNR 门是否通过 |
| 门限 | `nfov_tracking_gate_failure_count`、`nfov_tracking_coasting` | 连续失败数、是否外推中 |
| 光轴 | `nfov_pointing_error_deg` | 有效光轴中心与目标真值视线的角距 |
| 脱靶量 | `focal_plane_offset_x_m`、`focal_plane_offset_y_m` | 焦平面脱靶量（米） |
| 引导 | `cue_source_satellite_entity_id` | 引导来源星；`-1` = 自星宽场引导 |

`max_detection_range_m` 就是 1.4 的 $d_{max}$，随目标辐射、环境/噪声快照与扫描指向逐周期变化，
读历史行时不要当成常数。

问题清单（`SbirsIssue`）四要素：严重等级 `severity`、阶段 `phase`、代码 `code`、文本 `message`，
外加可选定位 `location` / `field` 与归因 `cause`。它记录的是"这次为什么没按预期跑"，
与"这次有没有探到"（检测记录）是两回事。

事件层（`SbirsDetectionLifecycleRecorder`）只缓存最近周期、不落盘、不进 raw output。5 类事件：

| 事件 | 中文 | 语义 |
|---|---|---|
| `kFirstDetected` | 首次检测 | 该目标第一次被检出 |
| `kUpdated` | 更新 | 持续检测，状态推进 |
| `kCoasting` | 外推保持 | 本周期无有效窄场量测但未失锁 |
| `kLost` | 丢失 | 锁定释放 |
| `kNotDetected` | 未检测 | 本周期没探到（受 `emit_not_detected_events` 开关控制） |

9 种具名原因（另有 `kNone`、`kUnknown` 兜底）：视场外、低于 SNR 门限、目标从输入消失、窄场捕获失败、
调度跳过、EKF NIS 连续超限、ATP 指向超时、窄场跟踪门连续失败。事件本身不带等级与空间位置字段——
排障要看归属记录里的诊断量，别指望事件行给坐标。

### 1.11 大幅面几何与覆盖验收量

宽视场每一拍要回答"这拍扫到了地面哪一块、目标在里面待多久"，这两个量由几何直接算出，不经过滤波。

**地面覆盖包络**（`ComputeGroundCoverageEnvelope`）：把视场矩形边界的采样点、视场中心、星下点和
地平圈采样点统统打到地球球面上，再取经纬度包围盒：

$$
\mathcal P=\{\text{中心}\}\cup\{\text{矩形四边各 }16+1\ \text{点}\}\cup\{\text{星下点}\}\cup\{\text{地平圈采样}\},
\qquad
\mathcal B=\bigl[\min_{\mathcal P}\varphi,\ \max_{\mathcal P}\varphi\bigr]\times\bigl[\min_{\mathcal P}\lambda,\ \max_{\mathcal P}\lambda\bigr]
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\mathcal P$ | 视线与地球球面交点的集合 | `ComputeGroundCoverageEnvelope` 内采样 |
| $16$ | 每条边的等分段数 | `kEdgeSteps` |
| $\varphi$、$\lambda$ | 纬度、经度（deg） | `foundation::TryComputeGroundIntersectionLatLonDeg` |
| $\mathcal B$ | 经纬度包围盒 | `min_lat_deg` / `max_lat_deg` / `min_lon_deg` / `max_lon_deg` |
| NW/NE/SE/SW | 包围盒四角，规范顺序输出 | 覆盖行"覆盖四角"字段 |

视场完全脱靶（所有采样射线都不与地球相交）时记 `miss`，不给伪造坐标；中心射线指向太空时中心也记
`miss`。地球按**球**处理（椭球口径是已登记的非目标，见 `boundaries.md`）。

**驻留时间**——目标被方位向扫描"扫过"视场所需的时间：

$$
t_{dwell}=\frac{\theta_{wfov,az}}{\omega_{scan}}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\theta_{wfov,az}$ | 宽视场方位向视场角 | `wide_field_fov_az_deg` |
| $\omega_{scan}$ | 扫描角速率 | `scan_rate_deg_per_sec` |
| $t_{dwell}$ | 单次穿越视场的驻留时间（s） | 验收日志"驻留=" |

$\omega_{scan}=0$（凝视）时按 0 记，不做除零兜底。

### 1.12 多传感器协同与能力跟踪

需求 3.2.1.4.1–3 要求 OPIR 内部具备多传感器目标跟踪、能力跟踪和协同探测信息融合能力。

**跨星引导与双星交会**：当多颗 OPIR 卫星同时覆盖同一目标时，一颗星的窄视场跟踪结果可作为另一颗星的 cue 输入（跨星引导，cross-cue）。两颗星各自提供一条视线（LOS），`DualLosFix` 取两条异面直线最近点对的中点作为目标 ECEF 位置解——这是 OPIR 从"纯角度"到"有位置"的关键桥梁。

$$
\begin{aligned}
\mathbf{p}_1 &= \mathbf{r}_1 + t_1 \mathbf{u}_1,\quad \mathbf{p}_2 = \mathbf{r}_2 + t_2 \mathbf{u}_2 \\
\mathbf{p}_{mid} &= (\mathbf{p}_1 + \mathbf{p}_2) / 2,\quad \text{残差} = \|\mathbf{p}_1 - \mathbf{p}_2\|
\end{aligned}
$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $\mathbf{r}_1, \mathbf{r}_2$ | 两颗卫星的 ECEF 位置 | 轨道外推 |
| $\mathbf{u}_1, \mathbf{u}_2$ | 两颗卫星的视线方向单位向量 | `SbirsDetectionRecord.line_of_sight_eci` |
| $t_1, t_2$ | 使 $\mathbf{p}_1, \mathbf{p}_2$ 距离最小的标量参数 | 异面直线最近点求解 |
| $\mathbf{p}_{mid}$ | 双星交会定位解 | `DualLosFix::ComputeFix` |

**能力跟踪与接力**（需求 3.2.1.4.2）：融合验收旁路估算剩余覆盖时间和接力交接对象，但这不是 OPIR 内部的完整产品——缺少传感器任务下发、交接确认和接力状态机。详见 §5.5。

### 1.13 规模目标识别

需求 3.2.1.5.1.2 要求 OPIR 对目标群进行规模识别。当前实现仅为确认航迹数写入验收日志（`WriteRirClusterCount`），作为目标数量下界。缺少三个关键能力：

1. **点迹聚类**：同一宽视场区域内多个检测点属于一个集群还是多个目标？
2. **不可分辨集群分解**：多个目标在角度上无法分辨时，如何估计数量？
3. **独立规模识别产品**：当前只有日志行，没有结构化的规模识别结论（如 `target_count`、`confidence`、`cluster_type`）。

**焦平面脱靶量**：窄场跟踪段把目标像点相对视轴中心的位移按米输出（`focal_plane_offset_x_m/y_m`），
这是"光轴还差多少对准"的直接量，精度评估的脱靶量指标就取自这里，不再从角度误差反推。

---

## 2. RIR：一个周期

### 2.1 周期骨架

`RirController::RunCycle` 只在 `work_mode == kIdentify` 下做完整链路，否则只维护航迹（`HoldCycle`）并逐目标
记 `kTargetModeNotIdentify`。

1. 平台经纬高 LLA（ECEF → LLA，失败则跳过需要高度的门）。
2. 环境：植被传播损耗、逐目标大气附加损耗、主瓣地杂波功率（见 2.9）。
3. RF 周期解析：载频、脉冲重复间隔（PRI）、脉冲数、本机发射、接收链、**接收机饱和**判定。
4. 上一周期航迹快照（供子窗豁免与候选排序）。
5. 本周期的驻留计划与预算：指定 → 外部跟踪 → 库内跟踪 → 搜索（见 2.5）。
6. 逐场景目标候选裁剪（四道门限，见 2.3）。
7. 逐驻留照射 + 天线增益 + 主瓣覆盖门限（见 2.4、2.6）。
8. 检测链路：SINR → 起伏目标检测概率 → 判决（见 2.7、2.8）。
9. 全局关联（马氏距离 + LAPJV）→ 航迹生命周期更新（见 2.12、2.13）。
10. 包络门限 + 同目标去重（见 2.14）。
11. 识别特征采集（四维提取器）→ 匹配 → 状态机（见 2.15）。
12. 指定任务生命周期推进与回扫（见 2.16）。
13. 输出帧组装（航迹、识别结论、特征量测、归属视图、驻留预算）。

**S 波段约束**（需求 3.2.2.3）：RIR 工作在 S 波段（2–4 GHz），默认中心频率 3 GHz（波长 ≈ 0.1 m）。配置字段 `operating_frequency_hz` 可对此频段建模；当前未对配置值做 2–4 GHz 硬范围校验。S 波段的"全天候"能力通过可选大气损耗（§2.9）和环境杂波表达，不是独立的天气/降水模型；实际全天候探测需要在 SINR 链路中计入降雨衰减和云层损耗。

### 2.2 RF（射频）与接收链

`ResolveRfCycle` 每周期解析一次：

$$
\mathrm{PRI} = \frac{1}{\mathrm{PRF}},
\qquad
N_{pulse} = \max\!\left(1,\ \left\lceil \frac{t_{dwell}}{\mathrm{PRI}} \right\rceil\right)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\mathrm{PRF}$ | 脉冲重复频率 | `prf_hz` |
| $\mathrm{PRI}$ | 脉冲重复间隔，PRF 的倒数 | — |
| $t_{dwell}$ | 单次驻留时长 | `dwell_sec` |
| $N_{pulse}$ | 本次驻留的脉冲数，向上取整且至少 1 | `pulse_cnt` |

载频由 `RirEmissionFactory::ResolveCarrierHz` 解析，支持跳频。

接收机前端把本接收窗内**全部入射链**（含自发射同平台耦合）聚合：

$$
P_{inc,total} > P_{in,max} \;\Longrightarrow\; \text{receiver\_saturated}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $P_{inc,total}$ | 本接收窗内全部入射功率之和（含自发射同平台耦合） | — |
| $P_{in,max}$ | 接收机线性输入上限 | `receiver.maximum_linear_input_power_w` |
| `receiver_saturated` | 本周期是否饱和的布尔量 | — |

饱和是**周期级致盲**：本周期不再做任何检测判决；已过遮挡/角域/覆盖的目标各记一条
`kTargetReceiverFrontEndSaturated`。它在检测链里排在 SNR 判决之前。

### 2.3 候选裁剪与主瓣覆盖

按代码执行顺序，一个目标要过四道筛选（前三道在候选池构建，第四道在检测链入口）：

| 顺序 | 门限 | 位置 | 判据 | 失败码（中文含义） |
|---|---|---|---|---|
| 1 | 地球遮挡 | 候选循环 | `IsEarthOcculted(platform, target)` | `kTargetEarthOcculted`（地球遮挡） |
| 2 | 角域 | 候选循环 | 搜索扇区 ∩ 可扫描体积；**指定目标/已有确认航迹**在硬件体积内豁免子窗 | `kTargetOutsideSearchVolume`（在搜索范围外） |
| 3 | 主瓣覆盖 | 逐驻留 | 见 2.6 | `kTargetOutsideBeamCoverage`（没被波束照到） |
| 4 | 接收机饱和 | 测量构建开头 | 周期级饱和，见 2.2 | `kTargetReceiverFrontEndSaturated`（前端饱和） |

饱和不是候选裁剪的一道，而是检测链的头一道判定：饱和周期里，已过前三道的目标全部记
`kTargetReceiverFrontEndSaturated`，不再做 SNR 判决。

角域两层的区别很关键：

```text
search_sector = IntersectScanSector(scan_window_deg, steerable_volume_deg)
in_hardware   = TargetWithinSteerableVolume(look, steerable_volume_deg, scan_center)
in_search     = TargetWithinSteerableVolume(look, search_sector, scan_center)
admit         = in_search || ( (is_designated || has_confirmed_track) && in_hardware )
```

即"搜索按任务子窗裁剪，但一旦已经指定或确认，跟踪连续性优先于子窗"。

| 伪代码变量 | 含义 | 来源 |
|---|---|---|
| `scan_window_deg` | 任务给定的搜索子窗 | 任务/策略配置 |
| `steerable_volume_deg` | 硬件可扫描体积 | 雷达配置 |
| `search_sector` | 两者交集，实际用于搜索 | — |
| `is_designated` | 是否被外部指定 | 输入 |
| `has_confirmed_track` | 是否已有确认航迹 | 上一周期航迹快照 |

候选排序：**未识别优先 → 斜距升序**（`recognition_rank = recognized ? 1 : 0` 升序，
让还没识别的目标先拿到驻留以积累特征）。

### 2.4 天线方向图与增益

`[common]` `AntennaPatternRuntime.h` 是方向图的唯一实现，RIR 只传波束宽度、离轴角与指向。三类模型
（`kGaussianMainLobe` 高斯主瓣 / `kCosinePower` 余弦幂 / `kSincPattern` 均匀孔径 sinc²）都先算主瓣
离轴衰减 $A_{main}$，再按"主瓣 / 副瓣 / 后瓣"分区给增益。

归一化离轴角，三模型共用：

$$
u_{az}=\frac{\lvert \Delta_{az}\rvert}{0.5\,\theta_{bw,az}},
\qquad
u_{el}=\frac{\lvert \Delta_{el}\rvert}{0.5\,\theta_{bw,el}}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\Delta_{az}$、$\Delta_{el}$ | 目标相对波束中心的离轴角（deg，方位先折到 $[-180,180]$） | `AntennaLookOffsetDeg` |
| $\theta_{bw,az}$、$\theta_{bw,el}$ | 有效波束宽度（deg） | `RirResolveEffectiveBeamwidth` |
| $u_{az}$、$u_{el}$ | 归一化离轴角，$u=1$ 恰为主瓣边界（−3 dB） | — |

$$
A_{main}=
\begin{cases}
3\left(u_{az}^{2}+u_{el}^{2}\right), & \text{高斯}\\[4pt]
-10\log_{10}\!\left[\cos^{p_{az}}\!\Delta_{az}\cdot\cos^{p_{el}}\!\Delta_{el}\right],\ p=-\dfrac{\ln 2}{\ln\cos\left(\theta_{bw}/2\right)}, & \text{余弦幂}\\[6pt]
-20\log_{10}\!\left|\operatorname{sinc}\!\left(\dfrac{\pi L_{az}\sin\Delta_{az}}{\lambda}\right)\right|-20\log_{10}\!\left|\operatorname{sinc}\!\left(\dfrac{\pi L_{el}\sin\Delta_{el}}{\lambda}\right)\right|, & \text{sinc}^2
\end{cases}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $A_{main}$ | 主瓣离轴衰减（dB，非负） | `ComputeMainLobeAttenuationDb` |
| $p_{az}$、$p_{el}$ | 余弦幂指数，由半功率宽度反解，保证边界处同为 3 dB | — |
| $L_{az}$、$L_{el}$ | 孔径方位长度、俯仰宽度（m） | 天线硬件配置 |
| $\lambda$ | 载波波长（m） | 由载频换算 |
| $\operatorname{sinc}(x)$ | $\sin x/x$，$x\to0$ 时取 1 | — |

增益按分区给出（后瓣 / 主瓣 / 副瓣）：

$$
G=
\begin{cases}
G_{peak}+L_{back}-L_{scan}, & \text{任一轴}\ \lvert\Delta\rvert>90^\circ\\[4pt]
G_{peak}-A_{main}-L_{scan}, & \text{主瓣内}\\[4pt]
G_{peak}-\max\!\left\{A_{sinc^2},\ -L_{sl}\right\}-L_{scan}, & \text{副瓣区}
\end{cases}
$$

$$
A_{sinc^2}(u)=-10\log_{10}\!\left[\operatorname{sinc}^{2}\!\left(1.3915587\,u\right)\right]
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $G$ | 该方向上的天线增益（dBi） | `AntennaPatternSample.gain_dbi` |
| $G_{peak}$ | 波束中心峰值增益（dBi） | `one_way_antenna_gain_dbi` |
| $L_{back}$ | 后瓣电平（dB，负值） | `backlobe_level_db` |
| $L_{sl}$ | 副瓣包络下限（dB 电平，代码里取反作衰减下限） | `max_sidelobe_level_db` |
| $A_{sinc^2}$ | 副瓣区连续延拓衰减，$u=1$ 处为 3 dB、第一零点 $u\approx2.26$ | `SincSquaredContinuationDb` |
| $1.3915587$ | $\sin x/x=1/\sqrt2$ 的解，用于把延拓对齐到半功率宽 | 常量 |
| 主瓣判据 | $\lvert\Delta_{az}\rvert\le0.5\theta_{bw,az}\wedge\lvert\Delta_{el}\rvert\le0.5\theta_{bw,el}$ | `IsInsideMainLobe` |
| 后瓣判据 | 任一轴 $\lvert\Delta\rvert>90^\circ$ | `IsInsideBackLobe` |

副瓣区不再取常数电平：先按 sinc² 包络连续衰减，再以 $L_{sl}$ 封底，避免主瓣边界出现台阶。
sinc² 模型配了物理孔径时，副瓣区直接沿用孔径公式全域延拓。

扫描损失（波束偏离天线视轴越远，增益越低）：

$$
L_{scan}=\operatorname{clamp}\!\left(k_{scan}\left(\Delta_{scan,az}^{2}+\Delta_{scan,el}^{2}\right),\ 0,\ L_{scan,max}\right)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\Delta_{scan,az}$、$\Delta_{scan,el}$ | 波束指向相对天线视轴偏置的角差（deg） | `boresight_offset_deg` |
| $k_{scan}$ | 单位角平方的扫描损耗系数（dB/deg²） | `scan_loss_coeff_db_per_deg2` |
| $L_{scan,max}$ | 扫描损耗上限（dB） | `max_scan_loss_db` |
| $L_{scan}$ | 本次指向的扫描损耗（dB） | `scan_loss_db` |

方向图的增益直接进 2.7 的雷达方程（$G^2$ 一项），也决定 2.6 的覆盖门限——两处同源，不会出现
"照到了却按 0 增益算"的矛盾。

### 2.5 扫描波位与驻留计划

**波位栅格**（`[common]` `ScanScheduleRuntime.h::BuildScanPattern`）：给定方位/俯仰范围、步长、
起点与扫描顺序，铺一张二维波位表，蛇形往返：

$$
N_{axis}=\left\lfloor \frac{span}{\Delta_{step}}\right\rfloor+1\ (\le 4096),
\qquad
\Delta_{step}=\frac{span}{N_{hint}-1}\ \ (N_{hint}>1)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $span$ | 该轴扫描跨度（deg） | `az_max - az_min` 等 |
| $\Delta_{step}$ | 该轴步长（deg）；给了步进数提示则按提示反算 | `ResolveAxisStepDeg` |
| $N_{axis}$ | 该轴波位数，上限 4096（超出截断） | 常量 `kMaxAxisSamples` |
| $N_{hint}$ | 步进数提示；$\le1$ 时用默认步长 | 扫描配置 |

范围或步长非法（非有限、跨度倒挂、步长 $\le 0$）直接返回空表，不做猜测性填充。

**本周期能排几个驻留**（`RirSession`）：

$$
N_{dwell}=\max\!\left(1,\ \left\lfloor \frac{\Delta t}{t_{dwell}}+10^{-6}\right\rfloor\right)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\Delta t$ | 本周期时长（s） | `input.dt_sec` |
| $t_{dwell}$ | 单次驻留时长（s） | `mission.recognition_dwell_sec` |
| $10^{-6}$ | 浮点容差，避免 $0.5/0.05$ 算成 9.9999 少排一拍 | 常量 |
| $N_{dwell}$ | 本周期驻留槽位数，至少 1 | — |

**槽位分配有严格优先级**（`RirDwellKind`）：

| 优先级 | 驻留类型 | 指向来源 | 说明 |
|---|---|---|---|
| 1 | `kDesignate` | 指定目标当前视线角 | 算子导引，语义最硬 |
| 2 | `kExternalTrack` | 外部请求位置 + 速度外推中心 | 实时目标指令，优先于库内自产状态 |
| 3 | `kTrack` | 库内航迹 `position + velocity·Δt` | 雷达自己的闭环跟踪 |
| 4 | `kSearch` | 固定波位表按游标取 | 兜底搜索 |

第 3 类只有在"剩余槽位 > 4"时才排——`kMinSearchDwells = 4` 保证跟踪再忙也留 4 个搜索槽，
不会把时间全喂给已有航迹。第 4 类按运行态游标 $p\leftarrow(p+1)\bmod N$ 逐周期推进；没有任何指定、
外部请求与航迹时，退化行为就是"第 $k$ 周期取第 $(k-1)\bmod N$ 个波位"。

**外部跟踪窗口**（`BuildExternalTrackWindowDwells`）：以"位置 + 速度 × 时长"外推出的中心为原点，
按波束宽度铺方形窗口，波位按到中心的归一化距离升序（近的先照）：

$$
\Delta_{grid}=\theta_{bw}\cdot s_{step},
\qquad
\rho=\sqrt{\left(\frac{\delta_{az}}{w_{az}}\right)^{2}+\left(\frac{\delta_{el}}{w_{el}}\right)^{2}}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\theta_{bw}$ | 该轴有效波束宽度（deg） | `RirResolveEffectiveBeamwidth` |
| $s_{step}$ | 步长系数 | `mission.step_scale` |
| $\delta_{az}$、$\delta_{el}$ | 波位相对窗口中心的角偏（deg），取 $0,\pm k\Delta_{grid}$ | — |
| $w_{az}$、$w_{el}$ | 窗口两轴半宽（deg） | `RirTrackWindowDeg` |
| $N_{cut}$ | 窗口波位上限 128，两轴各 512 | `kExternalWindowMaxPoints` |
| $\rho$ | 归一化距离，排序键 | — |

外部窗口有波位时**吃满整周期剩余槽位**，本周期不再排库内跟踪与固定光栅：来袭方向随目标变化，
按预设光栅补扫既浪费槽位也照不到目标。窗口跨周期续扫靠游标：

$$
p_{next}=\left(p_{take}+N_{take}\right)\bmod N_{window}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $p_{take}$ | 本周起照的窗口序号 | `external_window_cursor` |
| $N_{take}$ | 本周期窗口可取点数 | `min(剩余槽位, 窗口点数)` |
| $p_{next}$ | 下周期起点，新 cue 到来时归零（先照中心） | — |

**cue 合理性门**：外部引导偶发大幅跳变（实测有 4000 km 以上），跳变超限只标注不可信：

$$
d_{jump}>v_{max}\,\Delta N+s_{slack}\ \Longrightarrow\ \text{cue 降级}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $d_{jump}$ | 相邻两次请求位置的欧氏距离（m） | — |
| $v_{max}$ | 目标可达最大速度 10000 m/s | `kCueGateMaxSpeedMps` |
| $\Delta N$ | 两次请求相隔的周期数（至少按 1 计） | `external_track_last_request_cycle` |
| $s_{slack}$ | 跳变容差 50000 m | `kCueGateSlackM` |

降级**只作标注、不改变照射范围**：槽位照旧给外部窗口，因为本周期唯一的指向来源就是这条引导，
退回光栅既找不回目标、又白扔 8~9 拍。

| 变量（中文） | 前值 | 后值 |
|---|---|---|
| `scan_wave_cursor_`（搜索波位游标） | $p$ | $(p+1)\bmod N$ |
| `external_window_cursor`（外部窗口游标） | $p$ | $(p+N_{take})\bmod N_{window}$ |
| `external_track_phase`（外部跟踪阶段） | `kFresh` / `kCoasting` | 本周期有无新请求决定 |
| `external_track_coast_cycles`（外推保持周期数） | $c$ | 无新请求 $c+1$ |
| `external_cue_degraded_this_cycle`（本周期 cue 是否降级） | — | 由跳变判据给出 |
| `cue_gate_last_position`（上次可信 cue 位置） | — | 可信时更新为本周期位置 |
| `external_track_sweep_index`（外部扫描序号） | $s$ | $s+1$ |

### 2.6 驻留覆盖门限

一条驻留只覆盖以指向为中心、半宽为半功率波束宽度的一块区域：

$$
\begin{aligned}
\Delta_{az} &= 0.5 \cdot 1.0 \cdot \theta_{bw,az} \\
\Delta_{el} &= 0.5 \cdot 1.0 \cdot \theta_{bw,el} \\
\text{covered} &= \left(\lvert \delta_{az} \rvert \le \Delta_{az}\right) \wedge \left(\lvert \delta_{el} \rvert \le \Delta_{el}\right)
\end{aligned}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\Delta_{az}$、$\Delta_{el}$ | 驻留覆盖区的方位/俯仰半宽 | — |
| $\theta_{bw,az}$、$\theta_{bw,el}$ | 方位/俯仰有效波束宽度 | `RirResolveEffectiveBeamwidth` |
| $\delta_{az}$、$\delta_{el}$ | 目标相对驻留中心的角偏差 | — |
| `covered` | 目标是否被该驻留覆盖 | — |

其中 $1.0$ 是原式自带的系数，按原样保留。

$\theta_{bw}$ 取**有效波束宽度**（`RirResolveEffectiveBeamwidth`，nominal 配置或 λ/L 物理推导），与方向图同一来源。
同一目标**首驻留胜出**，同周期不重复量测；没被任何驻留照到的候选在循环后发一条覆盖排除诊断。

### 2.7 检测链路：信号干扰噪声比

`[common]` `DetectionCellResolver` 是 SINR 计算的唯一实现，RIR 只填参数：

$$
\begin{aligned}
P_r &= \frac{P_t\,G^{2}\lambda^{2}\sigma}{(4\pi)^{3}R^{4}L} \\[4pt]
P_n &= k\,T_0\,B\,F \quad (T_0 = 290\ \mathrm{K}) \\[4pt]
\mathrm{SINR} &= \frac{P_r\,G_{pc}\,G_{tgt}\,
\underbrace{H_{mti}}_{\text{MTI 增益}}\,
\underbrace{H_{bin}\,N}_{\text{MTD 增益}}}
{\max\left(P_n\,G_{noise}\,F_{mti} + I\,H_{mti}/G_{jam} + C\,H_{mti}/G_{clut},\ 10^{-300}\right)}
\end{aligned}
$$

| 符号 | 代码变量 | 说明 |
|---|---|---|
| $P_t$ | `own_waveform.transmit_power_w` | 发射峰值功率 |
| $G$ | `one_way_antenna_gain_dbi`（线性化） | 目标离轴增益（方向图按"视线角 − 驻留中心"衰减） |
| $\sigma$ | `cell_target.rcs_m2` | 有效雷达截面积（物理模型开启时按视角/频率修正） |
| $\lambda$ | — | 载波波长，由载频换算 |
| $R$ | — | 目标斜距 |
| $L$ | `receiver_loss_db + two_way_additional_propagation_loss_db` | 接收损耗 + 双向传播损耗 |
| $P_r$ | — | 目标回波功率（雷达方程） |
| $k$ | — | 玻尔兹曼常数 |
| $T_0$ | — | 参考噪声温度 290 K |
| $B$ | — | 接收带宽 |
| $F$ | — | 接收机噪声系数（线性） |
| $P_n$ | — | 接收机热噪声功率 |
| $G_{pc}$ | `max(1, B·τ)` | 脉冲压缩增益，$\tau$ 为脉冲宽度 |
| $G_{tgt}$、$G_{noise}$、$G_{jam}$、$G_{clut}$ | `signal_processing.*_gain_db` | 四个标量处理增益（统一钳位到 [0, 40] dB） |
| $H_{mti}$ | `MtiPowerResponse` = $4\sin^{2}\!\left(\pi f_{fold}/\mathrm{PRF}\right)$，输出字段 `mti_response_db` | **MTI 增益**：2 脉冲对消器对目标的响应（$+6$ dB ~ 盲速处 → 0） |
| $F_{mti}$ | `kMtiNoiseFactor` = 2 | MTI 噪声代价（**不是增益**）：对消器对白噪声 $+3$ dB，必须与 $H_{mti}$ 配对进分母 |
| $H_{bin}$ | MTD 各通道 DFT 功率取最大，输出字段 `mtd_bin_response_db` | **MTD 跨 bin 损耗**：≤ 1，恒为损耗 |
| $N$ | `effective_pulse_count`，输出字段 `coherent_integration_db` | 相干积累增益（**MTD 增益的另一半**，接收窗截断后） |
| $I$ | — | 干扰功率 |
| $C$ | — | 主瓣地杂波功率 |
| $\mathrm{SINR}$ | — | 检测链输出，送 CFAR 判决 |

俗称对照：**MTI 增益** = $H_{mti}$（`mti_response_db`）；**MTD 增益** = $H_{bin}\cdot N$（跨 bin 损耗 × 相干积累，SINR 通道拆成 `mtd_bin_response_db` 与 `coherent_integration_db` 两个字段）；验收旁路 `MtiMtdAcceptanceResult` 才把两者合成为 `mtd_gain_db = mtd_coherent_gain_db + mtd_bin_loss_db`（`MtiMtdAcceptanceBank.cpp` L109–115），与 SINR 通道的 $N\cdot H_{bin}$ 同源。

源码的执行顺序（同一函数内，无交叉重算）：$P_r$（L268–270）→ $\tau_{echo}$、$f_d$（L271–274）→ $G_{pc}$（L275–276）→ $P_n$（L277–279）→ $N$（L280–283）→ 干扰 $J$（L284–288，**用 $N$** 决定代表脉冲数）→ 杂波 $C$（L291）→ 四个增益线性化并计入 [0, 40] dB 钳位（L292–299）→ MTI 块 $H_{mti}$（L317）→ $H_{bin}$（L322）→ $N$ 乘入（L323）→ $F_{mti}=2$（L328）→ 分母三项相加、分子相乘（L330–338）。

动目标显示（MTI）/动目标检测（MTD）联算只在"`enable_mti_mtd_effects` 且 `pulse_count ≥ 2`"时启用；否则三者取 1，与旧口径逐位一致。

### 2.8 起伏目标检测概率与判决

`[common]` `StatisticalCfarDetector` 把 SINR 换成检测概率 $P_d$，再按 $P_d$ 抽一次签。判决门限由虚警率
反解（`RadarEquations::ComputeThreshold`）：

$$
T=
\begin{cases}
-\ln p_{fa}, & N=1\\[4pt]
Q^{-1}\!\left(N,\ p_{fa}\right), & N>1
\end{cases}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $p_{fa}$ | 恒虚警率，越界时回退 $10^{-6}$ | `cfar_pfa` |
| $N$ | 参与相干积累的脉冲数，至少 1 | `effective_pulse_count` |
| $T$ | 归一化检测门限（噪声功率为 1 时的能量门限） | — |
| $Q(\cdot,\cdot)$ | 上不完全伽马函数（正则化）；$Q^{-1}$ 为其反函数 | `boost::math::gamma_q` |

记单脉冲线性信噪比 $\chi=10^{\mathrm{SNR}/10}$，五种起伏模型分别给 $P_d$（$Q(a,x)$ 为上不完全伽马函数，
$\mathcal Q_M$ 为 Marcum Q 函数）：

$$
P_d=
\begin{cases}
\mathcal Q_{N}\!\left(\sqrt{2N\chi},\ \sqrt{2T}\right), & \text{Swerling 0 / 默认}\\[4pt]
e^{-T/(1+\chi)}, & \text{Swerling 1},\ N=1\\[4pt]
Q(a,V)+C^{a}\left[Q\!\left(a,\tfrac{V}{C}\right)-Q(a,V)\right], & \text{Swerling 1},\ N>1\\[4pt]
Q\!\left(N,\ \dfrac{T}{1+\chi}\right), & \text{Swerling 2}\\[4pt]
(1+u)e^{-u},\ u=\dfrac{2T}{2+\chi}, & \text{Swerling 3},\ N=1\\[4pt]
Q(M,V)+C^{M}\left[Q\!\left(M,\tfrac{V}{C}\right)-Q(M,V)\right]+M C^{M-1}(1-C)\left[Q\!\left(M-1,\tfrac{V}{C}\right)-Q\!\left(M-1,V\right)\right], & \text{Swerling 3},\ N>1\\[4pt]
Q\!\left(2N,\ \dfrac{2T}{2+\chi}\right), & \text{Swerling 4}
\end{cases}
$$

其中（Swerling 1 与 3 的中间量）：

$$
V=\frac{T}{1+N\chi},\quad C=\frac{N\chi}{1+N\chi},\quad a=N-1
\qquad\text{（1 型）}
$$

$$
V=\frac{2T}{2+N\chi},\quad C=\frac{N\chi}{2+N\chi},\quad M=2(N-1)
\qquad\text{（3 型）}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\chi$ | 单脉冲线性信噪比 | 由 2.7 的 SINR 换算 |
| $\mathrm{Swerling}\ 0$ | 不起伏（恒定 RCS）目标 | `kSwerling0` |
| $\mathrm{Swerling}\ 1/2$ | 慢/快起伏，扫描间/脉冲间独立 | `kSwerling1`、`kSwerling2` |
| $\mathrm{Swerling}\ 3/4$ | 慢/快起伏，含主散射体 | `kSwerling3`、`kSwerling4` |
| $\mathcal Q_M(a,b)$ | Marcum Q 函数，从 Poisson 峰值附近向两侧累加保证数值稳定 | `MarcumQ` |
| $V$、$C$、$a$、$M$ | 起伏模型中间量，$V$ 是门限折算、$C$ 是能量占比 | — |
| $P_d$ | 检测概率，最后钳到 $[0,1]$ | `detection_prob` |

注意 $N\chi$ 是**总信噪比**：慢起伏模型按整帧能量起伏，快起伏模型按脉冲独立起伏，同一 $N$ 下两者
差别很大——换 `swerling_type` 而不改其余配置，检出率会明显变。

**判决链是三步串联**（`DetectFromEchoBudget` / `DetectResolvedCell` 同构）：

```text
若 SNR < min_snr_db：       detected = false，Pd 清零（硬门优先，连抽签都不做）
否则：                      detected = (uniform[0,1] ≤ Pd)
若 detected 且 SNR < min_detection_margin_db：detected = false（余量门）
```

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| `min_snr_db` | 绝对 SNR 下限，低于它直接判不检出 | `min_snr_db`（默认 −10 dB） |
| `min_detection_margin_db` | 检出后的最小余量门（默认 −2 dB） | `min_detection_margin_db` |
| `uniform[0,1]` | 均匀抽样，随机流种子可设 | `ThresholdDecision`、`SetRandomSeed` |

`DetectFromEchoBudget` 与 `DetectResolvedCell` 的区别只在噪声账：前者把热噪声、杂波、干扰**相加**后
再算 SNR，后者直接消费已经算好的单脉冲 SINR（不再重复算回波/噪声/脉压）。

`gate_mode` 有两种：`kDetector`（检测器判决）走上面整套统计判决；`kSnrFallback`（SNR 兜底）退化为
$\mathrm{SNR}\ge6\ \mathrm{dB}$ 的固定门，不抽签、不算 $P_d$。

### 2.9 环境层：传播损耗与植被杂波

`[common]` `VegetationClutterModel` + RIR 薄适配（`internal/RirPropagationModel.cpp`）。无论植被档位如何，
传播损耗先给一条固定基线：

$$
L_{prop}=L_{base}+L_{atm}+L_{terrain}=4.0+1.5+1.0=6.5\ \mathrm{dB}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $L_{base}$ | 基础传播损耗 4.0 dB | `kInternalBasePropagationLossDb` |
| $L_{atm}$ | 大气衰减 1.5 dB | `kInternalAtmosphericAttenuationDb` |
| $L_{terrain}$ | 地形反射/绕射余量 1.0 dB | `kInternalTerrainReflectionDb` |
| $L_{prop}$ | 该档位的传播损耗（dB） | `propagation_loss_db` |

杂波以"相对热噪底的 dB"（CNR 口径）给出，基线 3 dB；打开物理模型后按植被几何修正再按 0.7 混合：

$$
P_{clut,dB}=3+10\log_{10}\!\left[(1-\mu)+\mu\,M_{veg}\right],
\qquad
M_{veg}=\operatorname{clamp}\!\left(1+0.05\,E_{\phi}\ln\!\left(1+n_{leaf}\right)\left(1+\rho_{norm}\right),\ 1,\ 30\right)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\mu$ | 物理模型混合比，固定 0.7 | `mix_ratio`（源码常量） |
| $M_{veg}$ | 植被散射倍乘因子 | `ComputeVegetationClutterMultiplier` |
| $E_{\phi}$ | 叶片相位矩阵能量 $m_{11}+m_{22}+\lvert m_{12}\rvert+\lvert m_{21}\rvert$ | `compute_leaf_phase_matrices` |
| $n_{leaf}$ | 该档位叶片数（24 / 56 / 96 / 120 / 180） | 档位参数表 |
| $\rho_{norm}$ | 叶片平均展布 / 冠层半径（归一化） | `average_extent / canopy_radius_m` |
| $P_{clut,dB}$ | 等效杂波功率，**相对热噪底**的 dB | `clutter_power_db` |

杂波要换成瓦才能进 2.7 的分母：

$$
P_{clut}=P_{n}\cdot10^{P_{clut,dB}/10},\qquad P_{clut,dB}\in[-120,\,120]\ \mathrm{dB}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $P_{n}$ | 接收机热噪声功率（W） | 2.7 的 $P_n$ |
| $P_{clut}$ | 等效杂波噪声功率（W） | `ComputeEquivalentClutterNoiseW` |
| $\pm120$ | 防溢出的相对 dB 钳位 | 常量 |

**读法要点**：`clutter_power_db` 是相对量，不是绝对 dBW。把它直接当功率用会差出整条热噪底。

五个档位（`kOpenGrassland` 开阔草地 / `kSparseWoodland` 疏林 / `kDeciduousForest` 落叶林 /
`kConiferousForest` 针叶林 / `kTropicalDense` 热带密林）只改植被参数，$L_{prop}$ 不变；`kDisabled` 时
杂波恒为基线 3 dB。

### 2.10 有效 RCS

`[common]` `RcsPhysics` 提供三种物理近似，RIR 侧 `dwell/RirEffectiveRcs.cpp` 只填参数。先把输入 RCS
等效成一个圆盘半径，再由半径和波数算三种散射体的 RCS：

$$
a_{eq}=\operatorname{clamp}\!\left(\sqrt{\frac{\sigma_{in}}{\pi}},\ a_{min},\ a_{max}\right),
\qquad
k_0=\frac{2\pi f}{c}
$$

$$
\sigma_{cyl}=\pi a_{eq}^{2}\cdot\frac{(k_0a_{eq})^{2}}{1+(k_0a_{eq})^{2}},
\qquad
\sigma_{bi}=\sigma_{cyl}\max(0,\cos\psi_i)\max(0,\cos\psi_s)\cdot\tfrac12\left(1+\cos\phi\right)
$$

$$
\sigma_{pl}=\frac{4\pi A^{2}}{\lambda^{2}}\cos^{4}\theta,\qquad A=\pi a_{eq}^{2}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\sigma_{in}$ | 场景输入的 RCS（m²） | `target.rcs` |
| $a_{eq}$ | 等效圆盘半径（m），钳在 $[0.05,5]$ | `min/max_equivalent_radius_m` |
| $k_0$ | 自由空间波数（rad/m） | 由载频算出 |
| $\sigma_{cyl}$ | 圆柱（单站）RCS | `ComputeCylinderRcs` |
| $\psi_i$、$\psi_s$ | 入射角、散射角（deg），后者的偏置由 `bistatic_psi_offset_deg` 加 | 视线俯仰角 |
| $\phi$ | 方位角差（deg） | 视线方位角 |
| $\sigma_{bi}$ | 双站圆柱 RCS | `ComputeBistaticCylinderRcs` |
| $\sigma_{pl}$ | 平板 RCS，$\theta$ 为入射角 | `ComputePlanarPlateRcs` |
| $\lambda$ | 波长（m） | $2\pi/k_0$ |

物理估计与输入值线性混合（`cylinder_weight` 在圆柱与平板之间调配）：

$$
\sigma_{phys}=\operatorname{clamp}\!\left(w\,\tfrac12\left(\sigma_{cyl}+\sigma_{bi}\right)+(1-w)\,\sigma_{pl},\ \sigma_{min},\ \sigma_{max}\right)
$$

$$
\sigma_{eff}=(1-r_{mix})\,\sigma_{in}+r_{mix}\,\sigma_{phys}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $w$ | 圆柱族权重 | `cylinder_weight`（默认 0.7） |
| $r_{mix}$ | 物理估计占比，0 = 只用输入值、1 = 完全物理 | `physics_mix_ratio` |
| $\sigma_{min}$、$\sigma_{max}$ | 物理 RCS 钳位区间 | `min_rcs_m2`（$10^{-4}$）、`max_rcs_m2`（100） |
| $\sigma_{eff}$ | 进 2.7 雷达方程的有效 RCS | `cell_target.rcs_m2` |

关闭物理模型或 $r_{mix}\le0$、载频非正时，直接返回输入 RCS，不做任何近似。

**视角 RCS 表不进检测链**：场景给的 `视角RCS=[…]` 是**识别特征表**（2.15 的 RCS 均值/标准差用它），
不是回波用的 RCS。想得到随视角变化的检测 RCS，要打开 `hardware.rcs_physics.enable_physical_rcs`。
把两者接反会整体改变能量预算，量测误差标定随之失效。

### 2.11 量测误差、系统偏置与采样

误差模型输入是**等效积累后**的信噪比，不是单脉冲值：

$$
\mathrm{SNR}_{int} = \mathrm{SNR}_{single} + 10\log_{10}N
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\mathrm{SNR}_{single}$ | 单脉冲信噪比，dB | 2.7 节 SINR 换算 | 
| $N$ | 相干积累脉冲数 | `effective_pulse_count` |
| $\mathrm{SNR}_{int}$ | 等效积累后信噪比，dB，误差模型的输入 | — |

`[common]` `MeasurementErrorModel.h` + `RadarEquations`：

$$
\Delta R = \frac{c}{2B}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $c$ | 光速 | 常量 |
| $B$ | 接收带宽，与 2.7 节同源 | 波形配置 |
| $\Delta R$ | 距离分辨力，米 | — |

$$
\sigma_R =
\begin{cases}
\dfrac{0.5\,\Delta R}{\sqrt{\mathrm{SNR}_{int}}}, & \mathrm{SNR}_{int} \ge -10\ \mathrm{dB} \\[6pt]
1.5777\,\Delta R, & \text{otherwise}
\end{cases}
\qquad
\sigma_{\theta} =
\begin{cases}
\dfrac{0.317\,\theta_{bw}}{\sqrt{\mathrm{SNR}_{int}}}, & \mathrm{SNR}_{int} \ge -10\ \mathrm{dB} \\[6pt]
\theta_{bw}, & \text{otherwise}
\end{cases}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\sigma_R$ | 测距标准差 | — |
| $\sigma_{\theta}$ | 单轴测角标准差 | — |
| $\theta_{bw}$ | 该轴有效波束宽度 | `RirResolveEffectiveBeamwidth` |
| $0.5$、$0.317$、$1.5777$ | 经验系数；低信噪比分支退化为"一个分辨单元" | 常量 |

$$
\sigma_{\theta,eq} = \sqrt{0.5\left(\sigma_{az}^{2} + \sigma_{el}^{2}\right)},
\qquad
bias_R = 20\ \mathrm{m},
\qquad
bias_{\theta} = \frac{\theta_{bw}}{30}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\sigma_{az}$、$\sigma_{el}$ | 方位、俯仰两个轴的测角标准差 | — |
| $\sigma_{\theta,eq}$ | 两轴合成的等效测角标准差 | — |
| $bias_R$ | 距离**系统偏置**（加在均值侧，不是随机项） | `kRangeMeasurementBiasM` |
| $bias_{\theta}$ | 单个角轴的系统偏置 | `ComputeAngleMeasurementBiasRad` |

量测协方差由极坐标方差经雅可比投到 ENU 直角系：

$$
R_{cart} = J\,\mathrm{diag}\!\left(\sigma_R^{2},\ \sigma_{\theta}^{2},\ \sigma_{\theta}^{2}\right)J^{\top} + 10^{-6}I
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $J$ | 极坐标 $(R,\theta,\theta)$ → ENU 直角坐标的雅可比矩阵 | — |
| $\mathrm{diag}(\cdot)$ | 三轴方差组成的对角阵 | — |
| $10^{-6}I$ | 防止协方差奇异的微小正则项 | 常量 |
| $R_{cart}$ | ENU 直角系下的量测协方差，送关联与滤波 | `MakeCartesianMeasurementCovariance` → `measurement_covariance` |
| `kCovarianceFloorM2` | 正则项 $10^{-6}$ 的常量名 | `1.0e-6` |

标准化流程（`kDetector` 模式）：

1. 位置先施加**系统偏置**（均值侧）：沿视线 $+20\ \mathrm{m}$，两轴横向各偏 $R\cos(el)\cdot bias_{az}$、$R\cdot bias_{el}$。
2. 再按 $R_{cart}$ 做 Cholesky（乔尔斯基）分解采样（`SampleMeasurementPosition`）。

`kSnrFallback` 模式**不施偏置、不采样**，量测位置直接取真值，只携带协方差——这是刻意的简化口径，
不要把两条路径的误差统计混在一起比较。

`built.velocity` 恒置零：雷达单次检测没有测速通道，速度只能来自滤波后验。

### 2.12 关联

`RirTrackAssociator`：关联波门内用马氏距离平方，波门外设为不可行，再做全局最优分配（LAPJV，线性分配求解器）：

$$
\nu = z - Hx_{pred},
\qquad
S = HP_{pred}H^{\top} + R,
\qquad
d^{2} = \nu^{\top}S^{-1}\nu
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $z$ | 本周期量测位置 | `measurement.position` |
| $x_{pred}$ | 航迹预测位置 | `prior.predicted_position` |
| $H$ | 量测矩阵，位置量测时取位置投影 | — |
| $P_{pred}$ | 预测状态协方差 | `predicted.covariance` |
| $R$ | 量测协方差，即 2.11 节的 $R_{cart}$ | `measurements[i].measurement_covariance` |
| $\nu$ | 新息（量测减去预测） | — |
| $S$ | 新息协方差，等于投影协方差加量测协方差 | `innovation_covariance` |
| $d^{2}$ | 马氏距离平方 | — |
| `gate_threshold` | 关联波门门限，取值同 `distance_gate_sigma²` | 关联配置 |

只有 $d^{2} \le$ `gate_threshold` 的配对可行。分配在"扩充方阵"上做（`BuildAugmentedSquareCostMatrix`），保证漏检/新目标都有合法出口。

### 2.13 航迹生命周期与滤波

生命周期（`RirTrackLifecycle`，走 `common/tracking/TrackLifecyclePromote`）：

| 事件 | 效果 |
|---|---|
| 命中 | `hit_count+1`（命中数）、`miss_count=0`（失配清零）；候选 `tentative` 累计到 `confirm_hits`（默认 3）转确认 `confirmed` |
| 失配 | `miss_count+1`；连续超过 `max_miss_before_lost`（默认 2）转丢失 `lost` |
| 丢失 `lost` 再命中 | 立即回确认 `confirmed`，滤波按零速无知先验重置 |

滤波器（`RirTrackFilter`）：六维 CV 卡尔曼（`x, vx, y, vy, z, vz`），初始化位置方差 `initial_state_variance`、
速度方差 `initial_velocity_std_mps²`；IMM 路径按关联键各持一份，模型噪声按 $q\cdot10^{\,i/(M-1)}$ 对数等距排列。

量测本身不带速度（去真值化），所以初速只有一条来源：新建航迹的**第二次命中**用两点位置差补初速
$\mathbf{v} = \left(\mathbf{p}_k - \mathbf{p}_{k-1}\right)/\Delta t$（`track->hit_count == 2` 时生效）；航迹丢失后重新命中不走这条，仍按零速无知先验重置。

**两条速度/加速度通道**，用途不同，不能混：

1. **滤波后验速度**：写进航迹状态，供指向预测（`position + velocity · Δt`）与显示。
2. **量测位置最小二乘拟合**：只进识别特征，不写航迹：
   - **速度**：最近 $\le 20$ 个周期的位置对时间做直线拟合，斜率即 $v$（样本 $\ge 5$）。
   - **加速度**：最近 $\le 40$ 个周期的"拟合速度"对时间做直线拟合，斜率即 $a$（样本 $\ge 10$）。

滤波后验的单周期加速度做限幅 + 一阶指数平滑：

$$
\mathbf a_{raw} = \frac{\mathbf v_k - \mathbf v_{k-1}}{\Delta t_{last}},
\qquad
\mathbf a_{raw} \leftarrow \mathbf a_{raw}\cdot\min\!\left(1,\ \frac{100}{\lVert \mathbf a_{raw}\rVert}\right),
\qquad
\mathbf a_k = (1-\alpha)\,\mathbf a_{k-1} + \alpha\,\mathbf a_{raw},\quad \alpha = \frac{1}{30}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\mathbf v_k$、$\mathbf v_{k-1}$ | 本周期与上周期滤波后验速度（三维矢量） | `track->velocity`、`previous_velocity` |
| $\Delta t_{last}$ | **距上次命中的实际经过时间**，不是固定 `dt` | `acceleration_dt_sec` |
| $\mathbf a_{raw}$ | 单周期差分得到的原始加速度矢量 | `instantaneous_raw` |
| $100$ | 加速度合理上限，超限按比例缩回，**不丢弃样本** | `kMaxPlausibleAccelerationMps2` |
| $\alpha$ | 一阶指数平滑系数 | `1 / kAccelerationSmoothingCycles` |
| $\mathbf a_k$ | 平滑后的加速度矢量 | `track->acceleration` |
| $\lVert\mathbf a_k\rVert$ | 加速度模长，对外输出 | `acceleration_mps2` |

第一式是逐轴差分，第二式按模长限幅（方向不变），第三式是逐轴 EMA。识别用的最小二乘拟合通道另有定义：

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $v$ | 最近 $\le 20$ 个周期位置对时间的拟合斜率 | `TryGetLeastSquaresVelocity` |
| $a$ | 最近 $\le 40$ 个周期"拟合速度"对时间的拟合斜率 | `TryGetLeastSquaresAcceleration` |

发生建轨或航迹丢失后重新命中这类重置时，`acceleration` 清零、`acceleration_smoothed` 复位为 false，下一次命中重新起始；
分母用**距上次命中的实际经过时间**，外推后重新命中不会按单周期 `dt` 把加速度放大。

### 2.14 包络门限与去重

- 斜距 $> 2 \times$ `mission.max_range_m`：判发散。
- 高度 $> 4500\ \mathrm{km}$ 或 $< -1\ \mathrm{km}$：判发散。
- 判发散 → `lifecycle_->DropTrack(key)`，直接丢弃，不标记 `lost`。

去重：同一 `external_target_id` 只留一条（`id = 0` 不参与），比较顺序为
**状态枚举值升序 → 位置不确定度（迹）小者 → 命中数多者**。
（源码注释写的是"已确认优先"，实际比较按 `RirTrackStatus` 枚举值升序，即 `kTentative(0) < kConfirmed(1) < kLost(2)`；
以代码行为为准。）

### 2.15 识别

特征四维：雷达截面积、运动、极化、距离像（range profile）。每维先由独立提取器算出"观测值 + 质量"，
质量决定它进场后有多大话语权。

#### 2.15.1 四维特征的提取

**运动维**（`RirMotionFeatureExtractor`）：只对**已确认航迹**取值，未确认航迹只积累不输出。

$$
\mathbf v=
\begin{cases}
\mathbf v_{fit}, & \text{有多点拟合速度}\\
\mathbf v_{post},\ \lVert\mathbf v_{post}\rVert, & \text{否则用滤波后验}
\end{cases}
\qquad
h=h_{platform}+z
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\mathbf v_{fit}$ | 最近 $\le20$ 拍位置对时间的直线拟合速度（ENU） | `velocity_lsq_mps` |
| $\mathbf v_{post}$ | 滤波后验速度 | `snapshot.velocity` |
| $h$ | 目标绝对高度（m） | `platform_altitude_m + position.z()` |
| 取值优先级 | 拟合速度优先，因滤波后验在长斜距/低角精度下只有真值约 1/3 | 注释 B3 |

加速度有两道门（`kMaxVelocityVarianceForAcceleration = 1e5`、量级 1–50 m/s²），防的是"滤波重置伪值"：

$$
a=
\begin{cases}
a_{fit}, & \sigma_{v,max}^{2}\le10^{5}\ \wedge\ 1\le a_{fit}\le50\\
0, & \text{否则}
\end{cases}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\sigma_{v,max}^{2}$ | 单轴速度方差最大值（速度是否收敛） | `MaxVelocityVariance()` |
| $a_{fit}$ | 逐拍拟合速度的斜率（物理加速度） | `acceleration_lsq_mps2` |
| $10^{5}$ | 加速度可信门限（m²/s²），远低于重置后的无知先验 $9\times10^{6}$ | `kMaxVelocityVarianceForAcceleration` |
| $50$ | 加速度硬上限，超过一律置 0（不写特征） | 常量 |

转弯半径由横向加速度分解得到，横向分量过小记直线飞行：

$$
a_{lat}=\left\|\mathbf a-\frac{\mathbf a\cdot\mathbf v}{\lVert\mathbf v\rVert^{2}}\,\mathbf v\right\|,
\qquad
R_{turn}=\frac{\lVert\mathbf v\rVert^{2}}{a_{lat}}\ \ (a_{lat}\ge0.01)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\mathbf a$ | 航迹加速度矢量（ENU） | `snapshot.acceleration` |
| $\mathbf v$ | 航迹速度矢量（ENU） | `snapshot.velocity` |
| $a_{lat}$ | 横向（垂直于速度）加速度分量 | — |
| $R_{turn}$ | 转弯半径（m）；$a_{lat}<0.01$ 时置直线标志 | `turn_radius_m`、`is_straight` |

质量因子由估计不确定度归一化（不确定度越小质量越高）：

$$
q=\operatorname{clamp}\!\left(\frac{10^{4}}{10^{4}+\operatorname{tr}(P_{pos})/3},\ 0,\ 1\right)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\operatorname{tr}(P_{pos})$ | 预测协方差位置分块的迹（三轴方差之和，m²） | `estimation_uncertainty_trace` |
| $10^{4}$ | 参考方差 $(100\ \mathrm{m})^2$，量纲对齐 | 常量 |
| $q$ | 该维质量 | `observation.quality` |

**RCS 维**（`RirRcsFeatureExtractor`）：先过 SNR 门，再做视角插值，最后看视角覆盖够不够。

$$
\sigma_{meas}=\sigma(\theta_{look})\ \text{最近样本},\qquad
\sigma_{dB}=\frac{3}{\sqrt{\mathrm{SNR}_{lin}}}
$$

$$
q_{rcs}=0.5\min\!\left(1,\frac{\mathrm{SNR}}{30}\right)+0.5\min\!\left(1,\frac{\Omega_{asp}}{60}\right)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\sigma(\theta_{look})$ | 视线角处的 RCS 表插值（dBsm） | `InterpolateAt` |
| $\Omega_{asp}$ | 视角覆盖 = $\min(\text{方位跨距},\text{俯仰跨距})$（deg） | `aspect_coverage_deg` |
| $\mathrm{SNR}_{lin}$ | 线性信噪比 | 本周期检测 SNR |
| $q_{rcs}$ | RCS 维质量 | `features.rcs.quality` |
| 门 1 | $\mathrm{SNR}<6\ \mathrm{dB}$ → 维度无效 | `kMinimumObservationSnrDb` |
| 门 2 | 多样本且 $\Omega_{asp}<$ `minimum_aspect_coverage_deg` → 维度无效 | 识别配置 |
| 单样本 | 视线角与唯一样本各轴差 $\le1^\circ$ 才算命中，不设覆盖门 | — |

**极化维**（`RirPolarizationFeatureExtractor`）：双通道（HH/VV）幅度，噪声按第一通道 SNR 折算。

$$
e_1=\sigma_1+n,\quad e_2=\sigma_2+n,\quad n=\frac{\sigma_1}{10^{\mathrm{SNR}/10}}
$$

$$
\Delta_{E}=10\log_{10}\frac{e_1}{e_2},
\qquad
\Delta_{rel}=10\log_{10}\frac{\lvert e_1-e_2\rvert}{\max(10^{-6},\,e_1+e_2)}
$$

$$
\Sigma_{E}=10\log_{10}\!\left[(\sigma_1+\sigma_2)\left(\frac{R}{10^{5}}\right)^{4}\right],
\qquad
q_{pol}=\min\!\left(1,\frac{\mathrm{SNR}}{30}\right)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\sigma_1$、$\sigma_2$ | 两通道 RCS 线性值（dBsm 反算） | `hh_amp_db`、`vv_amp_db` |
| $e_1$、$e_2$ | 叠加噪声后的通道能量 | — |
| $\Delta_E$ | 能量差（通道功率比，dB） | `energy_difference_db` |
| $\Delta_{rel}$ | 相对差（对数化，接近相等时以下限保护） | `relative_difference_db` |
| $\Sigma_E$ | 能量和，补偿到统一参考距离 100 km，去掉体制相关项 | `energy_sum_db` |
| $R$ | 目标斜距（m） | 本周期量测 |
| $q_{pol}$ | 极化维质量；$\mathrm{SNR}<6\ \mathrm{dB}$ 或 $R\le0$ 时维度无效 | `features.polarization.quality` |

**距离像维**（`RirRangeProfileFeatureExtractor`）：一维距离像只取散射中心级的峰。

$$
\Delta R=\frac{c}{2B}\ \ (\le \Delta R_{max}),
\qquad
P_{gate}=\frac{P_{peak}}{10^{\mathrm{SNR}/10}}
$$

$$
L=o_{max}-o_{min},
\qquad
\eta_{conc}=\frac{\sum_{i=1}^{3}P_{(i)}}{\sum P_{peak}}
$$

$$
q_{rp}=0.5\min\!\left(1,\frac{\mathrm{SNR}}{30}\right)+0.5\min\!\left(1,\frac{50}{\Delta R}\right)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $B$ | 带宽（Hz） | 波形配置 |
| $\Delta R$ | 距离分辨力（m）；超过数据库允许值即维度无效 | `max_range_resolution_m` |
| $P_{peak}$ | 最强散射中心的线性功率 | — |
| $P_{gate}$ | 噪声门限，弱于它不算散射中心 | — |
| $o_{min}$、$o_{max}$ | 有效峰的最近/最远距离偏移（m） | `range_offset_m` |
| $L$ | 目标长度 = 首末有效峰跨距 | `length_m` |
| $\eta_{conc}$ | 能量集中率：前 3 峰能量 / 有效峰总能量 | `peak_energy_concentration` |
| $q_{rp}$ | 距离像维质量 | `features.range_profile.quality` |
| 合并规则 | 相邻峰间距 $<0.5\ \mathrm{m}$ 视为同一散射中心 | `kMinimumPeakSeparationM` |

#### 2.15.2 相似度与打分

逐维相似度是高斯核（`RecognitionMatcher`）：

$$
z_d = \frac{\lvert x_d - \mu_d\rvert}{\sigma_d},
\qquad
s_d = \exp\!\left(-\tfrac{1}{2}z_d^{2}\right)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $d$ | 特征维索引：RCS / 运动 / 极化 / 距离像 | `rcs`、`motion`、`polarization`、`range_profile` |
| $x_d$ | 本周期该维的观测值 | `features.rcs.mean_dbsm`、`motion.speed_mps` 等 |
| $\mu_d$ | 模板中该维的标称值 | `profile.rcs.mean_dbsm`、`tpl.speed_mps.mean` 等 |
| $\sigma_d$ | 模板中该维的标准差 | `profile.rcs.std_db` 等 |
| $z_d$ | 标准化偏差（几个 $\sigma$） | — |
| $s_d$ | 该维高斯相似度，取值 $(0,1]$ | `GaussianSimilarity` |

$$
score = \frac{\sum_d w_d q_d s_d}{\sum_d w_d q_d},
\qquad
\mathrm{model\_score} = \max_{profile}(score)\cdot prior_{model},
\qquad
\mathrm{category\_score} = \max_{model \in cat}\left(\mathrm{model\_score}\right)
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $w_d$ | 该维基础权重 | `rcs_weight`、`motion_weight`、`polarization_weight`、`range_profile_weight` |
| $q_d$ | 该维**质量**（本周期该特征可信度），为 0 时该维不入分母 | `features.*.quality` |
| $score$ | 单个型号（单条模板）的加权相似度 | — |
| $prior_{model}$ | 型号先验概率 | `model.prior` |
| $\mathrm{model\_score}$ | 同一型号多条模板取最大，再乘先验 | — |
| $\mathrm{category\_score}$ | 同一大类内各型号取最大 | — |

$$
confidence = \frac{\mathrm{best\_model\_score}}{\sum \mathrm{model\_score}}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $\mathrm{best\_model\_score}$ | 排序后第一名型号的得分 | — |
| $\sum \mathrm{model\_score}$ | 全部型号得分之和 | — |
| $confidence$ | 最佳型号在总分中的占比 | `result.confidence` |

滑窗聚合（`RecognitionTracker`）：`accumulation_window_sec` 内的观测（缺省 10 s）；**运动子特征取中位数**，
其余取均值。状态机（阈值为 `RecognitionTracker::Options` 缺省值，可由策略配置覆盖）：

| 条件 | 结论 |
|---|---|
| `confirmed_hit_count < 5` 或 `observation_count < 3` | `kAccumulating`（累积中） |
| `best_score < 0.70` | `kUnknown`（未知） |
| `margin ≥ 0.10` 且有效维数 ≥ 2 且非"仅运动维" | `kModelConfirmed`（型号确认） |
| 其余 | `kCategoryConfirmed`（大类确认） |

单周期"无可用特征/无适用模板"不覆盖已有结论（保持上一次判定），避免结论抖动。

### 2.16 指定任务与回扫

外部可以指定一个目标 ID，要求雷达在限时窗口内把它识别出来（`RirDesignationPhase`，会话级跨周期
状态，随 patch 原子提交）：

| 阶段 | 含义 | 驻留行为 |
|---|---|---|
| `kNone` | 未指定 / 指令未消费 | 正常扫描 |
| `kPending` | 窗口内等待识别 | 若目标在可扫描体积内，**第一槽位对准它** |
| `kAcquired` | 识别达成（终态） | 回扫，直到外部重新指定 |
| `kExpired` | 窗口耗尽未识别（终态） | 回扫，直到外部重新指定 |

窗口截止周期与状态转移：

$$
c_{dl}=
\begin{cases}
\min\!\left(c+N_{dur},\ 2^{32}-1\right), & N_{dur}>0\\
0\ \text{（无限期）}, & N_{dur}=0
\end{cases}
$$

$$
\text{phase}_{k+1}=
\begin{cases}
kPending, & \text{phase}_k=kNone\\[3pt]
kAcquired, & \text{phase}_k=kPending\ \wedge\ \text{recognized}\\[3pt]
kExpired, & \text{phase}_k=kPending\ \wedge\ c_{dl}\neq0\ \wedge\ k\ge c_{dl}\\[3pt]
\text{phase}_k, & \text{其余}
\end{cases}
$$

| 符号 | 含义 | 代码/配置 |
|---|---|---|
| $c$ | 开窗时的周期号 | `input_cycle_index` |
| $N_{dur}$ | 指定任务窗口长度（周期数），0 = 无限期 | `designation_duration_cycles` |
| $c_{dl}$ | 截止周期；饱和加法避免接近 $2^{32}$ 时回绕成 0（0 语义是"无限期"） | `deadline_cycle_index` |
| `recognized` | 上一周期识别结论是否为 `kModelConfirmed` / `kCategoryConfirmed` | `RecognitionTracker` 结论 |
| $\text{phase}_{k+1}$ | 本周期推进后的阶段；`kAcquired`/`kExpired` 不再转移 | — |

识别判定与驻留**同源**，都用"上一周期航迹快照"口径——所以识别达成与回扫之间天然差一拍，这是
设计口径，不是漏拍。

驻留是否对准指定目标，还要过"目标在不在、在不在硬件体积内"两问：

| 情形 | 驻留 | 反馈码 |
|---|---|---|
| 目标在场景且在 `scan_center + steerable_volume` 内 | 对准目标视线角（`kDesignate`） | — |
| 目标不在场景（`external_target_id` 找不到 / 为 0） | 回扫描波位 | `kNotRecognized` |
| 目标在场景但视线越出可扫描体积 | 回扫描波位 | `kOutsideSteerableVolume` |

`kOutsideSteerableVolume` 不是作废：转台重新瞄准后同一任务还能恢复；只有窗口耗尽才转 `kExpired`。
回扫沿（`kAcquired` 或 `kExpired` 发生的那个周期）会由生命周期记录器发一条 `kDesignationDropped`，
镜像信封里的 `designation_revert_reason`，供上层知道"这条指定任务结束了、原因是什么"。

| 变量（中文） | 前值 | 后值 |
|---|---|---|
| `designation_phase`（指定任务阶段） | 上周期阶段 | 按上表转移 |
| `deadline_cycle_index`（窗口截止周期） | — | 开窗时算出，终态不再变 |
| `designation_active`（本周期驻留是否生效） | 上周期是否生效 | `kPending ∧ 在体积内` |
| `designation_reverted_to_scan`（是否已回扫） | `false` | 目标缺席/越界/终态时 `true` |

---

## 3. 跨周期状态：前值 → 后值

### 3.1 OPIR（`SbirsPipelineSnapshot`）

| 变量（中文） | 何时改 | 前值 → 后值 |
|---|---|---|
| `scan_phase_deg_`（扫描相位） | 每执行周期 | 上一周期相位 → 往复折叠后的相位 |
| `scan_leg_forward_`（扫描腿是否正向） | 每执行周期 | 腿方向 → `travel_folded < span` |
| `scan_row_index_`（当前扫描行） | 跨界时 | 行号 → 行号 + 越界次数，模 `row_count` |
| `scan_wrap_span_deg_`（有效扫描跨度） | 每执行周期 | 上一周期有效跨度 → 本周期裁剪值；相位越界则相位归零、腿复位 |
| `nfov_rotation_step_`（窄视场轮转步数） | 锁定集合非空 | `s` → `s+1` |
| `target_states_`（各目标状态） | 各门/捕获/失锁 | 七态迁移，见 1.9 |
| `wfov_consecutive_hits_`（宽视场连续命中数） | 四道门限全过 +1；任一道失败/消失/捕获成功清零 | `n` → `n+1` / `0` / 删除键 |
| `nfov_scheduler_`（窄视场调度器） | 入列（`Acquire`）、失锁（`Release`） | 锁定集合成员变化 |
| `pointing_coordinator_`（指向协调器） | 每周期转向 + 逐目标门限失败计数 | `current_los`、`tracking_gate_failure_counts_` |
| `cue_predictor_`（引导预测器） | 每候选更新一次（`Update`） | 该目标上一周期量测角 → 本周期量测角 |
| `tracking_coordinator_.filter_states_` / `angle_kf_states_` / `imm_snapshots`（跟踪协调器的滤波状态） | 跟踪命中时先预测后更新；外推只预测 | 后验均值/协方差递推 |
| `nis_gate_exceeded_counts_`（NIS 超限计数） | NIS 超统计门限 +1，通过清零，失锁清除 | 连续超门限的周期数 |
| `next_detection_id_`（下一个检测编号） | 每条检测记录 | `k` → `k+1` |

回放/断点续算靠这套快照：`CaptureRuntimeState` 导出、`RestoreRuntimeState` 校验后恢复（含随机源状态，
因此续算逐位一致）。

### 3.2 RIR

| 变量（中文） | 何时改 | 前值 → 后值 |
|---|---|---|
| `next_track_id_`（下一个航迹编号） | 新建航迹 | `k` → `k+1` |
| `RirTrackState.position / velocity`（航迹位置 / 速度） | 每次命中 | 预测 → 后验 |
| `RirTrackState.gaussian_state`（高斯状态：均值 + 协方差） | 命中（KF/IMM 更新） | 先验 → 后验 |
| `RirTrackState.hit_count / miss_count`（命中数 / 失配数） | 每周期 | 命中 +1 清零失配；失配 +1 |
| `RirTrackState.status`（航迹状态） | 达到阈值 | 候选 → 确认 → 丢失 → 再确认 |
| `RirTrackState.acceleration`（加速度矢量） | 命中且 `dt` 有效 | EMA 递推；重置周期清零 |
| `acceleration_smoothed`（加速度是否已平滑） | 首个样本 / 重置 | `false → true` / `true → false` |
| `velocity_history_by_key_`（按关联键的位置历史） | 每次命中 | 追加本周期位置，只保留最近 20 条 |
| `velocity_trend_by_key_`（按关联键的拟合速度序列） | 每次记录拟合速度 | 追加，只保留最近 40 条 |
| 识别 `window` / `window_timestamps_sec`（观测窗与时间戳） | 每周期有效观测 | 追加并按 10 s 裁剪（至少保留 1 条） |
| `observation_count` / `confirmed_hit_count`（观测数 / 确认命中数） | 每周期观测 / 确认航迹命中 | 单调累加；航迹丢失后重新命中不清零，仅槽位复用（`hit_count` 回退）时随窗口清零 |
| IMM 运行态（每航迹一份） | 确认航迹命中时更新；失配只预测 | 模型权重与各模型状态递推 |
| `last_track_snapshots_`（上一周期航迹快照） | 每周期末 | 本周期快照 → 下周期"上一周期航迹" |
| `last_cycle_snr_by_target_id_`（上一周期逐目标 SNR） | 每周期重置后按检测填充 | 供识别上下文与归属输出 |

RIR 的跨周期状态分三处持有：航迹与滤波在 `RirTrackLifecycle`，识别窗口在 `RirTracker`，调度/环境/会话态在
`RirController`。三者互不越界——识别只读航迹快照，不写滤波状态。

### 3.3 下游三段

| 变量（中文） | 何时改 | 前值 → 后值 |
|---|---|---|
| `tracks_`（融合航迹表） | 每周期关联/新建/删除 | 上一周期航迹集合 → 归并、扩展、裁剪后的集合 |
| `missed_cycles`（失跟计数） | 周期开始统一 +1；有量测清零 | `n` → `n+1` / `0`；超门删除 |
| `hits`（累计命中数） | 每次挂上样本 | `h` → `h+1`（跨周期只增不减，删除时随航迹消失） |
| `has_real_update` / `last_real_update_cycle`（是否吃过真实量测） | 非合成量测挂上时 | `false → true`；周期号刷新 |
| `synthetic_key_counter_`（合成键计数） | 新建无身份航迹 | `k` → `k+1` |
| `filter_mean` / `filter_cov`（滤波均值 / 协方差） | 每拍预测 + 更新 | 先验 → 后验 |
| `filter_cycle` / `filter_timestamp_s`（滤波时间基准） | 更新成功 | 上一拍 → 本拍 |
| `RelaySights()`（逐航迹逐源视线采样） | 每拍有方位时 | 追加滑窗（最多 8 条）、累计解缠方位 |
| `AngleKfStates()`（标准 KF 状态） | 每拍有方位时 | 预测 → 更新 |
| `KnownSources()`（会话已知源集合） | 每周期登记 | 只增不减 |
| `truth_keypoints`（真值关键点缓存） | 首次推演后置 `computed` | 未算 → 已算 |
| `cycles_since_inference`（距上次关键点推演的周期数） | 每周期 +1，推演后清零 | `n` → `n+1` / `0` |
| 五条误差序列 | 每周期有效样本 | 追加；`Summarize` 只读不清 |

这三段的状态都不进 `SbirsPipelineSnapshot`：回放要重建时，靠同一批输入记录重跑即可得到相同结果。

---

## 4. 容易读错的地方

| 现象 | 正确读法 |
|---|---|
| OPIR 日志"热噪声=未建模" | 不是 0 J；探测器噪声由 NEP 承担，SNR 分母里没有 `kTB` 项 |
| OPIR 视场门用 ECI 角还是传感器系角 | **传感器系**；输出角仍是 ECI 极坐标 |
| OPIR 跟踪段"帧数=0" | 本周期无窄视场帧（轮空/转动中）→ 外推，不是失锁 |
| OPIR `nfov_frame_count` 与 `settled_frames` | 前者已叠了几何/SNR 门；后者只是执行器稳定时长折算 |
| RIR 候选"过了 SNR 门限却没量测" | 可能没过**主瓣覆盖门限**（波束没照到），与 SNR 无关 |
| RIR `kSnrFallback` 量测 | 位置取真值、不施偏置、不采样；与 `kDetector` 的误差统计不可比 |
| RIR 加速度为什么不用滤波差分 | 滤波差分噪声被 $1/\Delta t$ 放大；识别用最小二乘斜率，航迹用限幅 + EMA |
| RIR 同一目标出现多条航迹 | 输出前有包络门限 + 去重；发散航迹是 `DropTrack` 直接丢弃 |
| RIR 视角 RCS 表 | 只进识别特征与打分，不进 SINR；SINR 用的是物理 RCS |
| RIR `clutter_power_db` | 是**相对热噪声底**的比值，不是绝对功率；所以按倍乘进噪声，而不是按瓦相加 |
| RIR 驻留"用满了预算" | 外部窗口与优先级都会抢驻留，预算先到先用，剩余归零；不是每波位固定时长 |
| 融合"同周期新建的航迹没被关联" | 有意为之：周期起始快照冻结，新航迹从下周期起才是候选 |
| 融合置信度看绝对值 | 只做滑窗内精确累加、不归一化，所以只适合同场比大小，不能跨场景比 |
| 融合"交接对象是算出来的" | 库内无轨道/视场模型，只是从已知源里挑样本最多（或编号最小）的那个，不是几何判定 |
| 精度评估"全程没检测也能及格" | 零样本项 RMSE 置 $+\infty$、得 0 分，不允许靠缺证据拿分 |
| 失准角进不进量测 | 只进几何（视轴指向），不进量测误差：量测误差由噪声模型单列 |

---

## 5. 下游链路：融合 → 推演 → 评估

前两节到"检测记录"为止。检测记录之后还有三段，各自是独立库，靠数据结构接力，不共享内部状态：

1. **融合**（`fusion`）：多星/多源检测记录 → 航迹（`FusedTarget`），含关联、置信度、滤波。
2. **推演**（`target_inference`）：航迹的运动学状态 → 弹道、落点、发射点、目标类型。
3. **精度评估**（`precision_evaluation`）：两星周期结果 + 真值 + 融合航迹 → 五项误差与综合分。

三段都可以单独用；接在一起时，上一段的输出就是下一段的输入，中间没有隐式耦合。

### 5.1 融合：三级关联与置信度

每个融合周期固定走六步：失跟计数统一 +1 → 身份键直挂 → 无身份记录分层关联 → 滑窗裁剪与失跟删除 → 失跟航迹外推 → 输出。

**身份键直挂**先分流：`key != kUnidentifiedKey` 的记录按键归并到同名航迹（跨源一致性由调用方保证）；键为 `kUnidentifiedKey` 的才进空间关联。归并前按时间戳稳定排序，保证同周期多拍按物理时序进滤波。

**快照冻结**：周期开始时先记下已有航迹键集合 `cycle_start_keys`，关联（含 KD-tree 建树）只看这批航迹。本周期新建的航迹不参与本周期关联，从下周期起才当候选——否则同一目标的两条记录会在同周期内互相抢答。

无身份记录按可用量测分三条路，优先级由高到低：

| 可用量测 | 关联方式 | 门限 | 代价 |
|---|---|---|---|
| 有位置 | KD-tree 半径搜索（位置先转 ECEF） | `position_radius_m` = 1000 m | ECEF 欧氏距离，取最小 |
| 无位置、有方位 | 遍历航迹比方位相干 | `bearing_beamwidth_deg` = 5.0° | 方位最短差，取最小 |
| 位置方位都没有 | 取首个过特征门的航迹 | `feature_threshold`（≤0 关闭） | 无 |

位置记录转 ECEF 失败（LLA 越界等）不丢弃，降级走方位通道；方位也没有时再降一级走特征通道。

半径搜索里每条候选还要过**特征门**：两侧特征维度不一致时视为"异构传感器不可比"，特征门不构成约束；维度一致才比欧氏距离。门限 `feature_threshold` ≤ 0 即整体关闭。

$$d=\bigl\lVert \mathbf{p}_{det}-\mathbf{p}_{track}\bigr\rVert_2\le R_{pos}$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $d$ | 记录与航迹锚点的 ECEF 距离，m | — |
| $\mathbf{p}_{det}$ | 本拍位置量测转 ECEF | `detection.position` → `TryLlaToEcef` |
| $\mathbf{p}_{track}$ | 航迹锚点（该源最近一拍）转 ECEF | `track.anchor.position` |
| $R_{pos}$ | 空间门限半径，m | `FusionConfig::position_radius_m`（默认 1000） |

方位通道用同一套代价：

$$\bigl|\Delta az\bigr|\le \theta_{bw},\qquad \Delta az=\operatorname{wrap}_{180}(az_{det}-az_{track})$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $\Delta az$ | 方位最短差（折算到 ±180°），deg | `AzimuthShortestDifferenceDeg` |
| $az_{det}$ / $az_{track}$ | 本拍方位 / 锚点方位，deg | `bearing_az_deg` |
| $\theta_{bw}$ | 方位相干门限，deg | `FusionConfig::bearing_beamwidth_deg`（默认 5.0） |

都没命中的记录新建航迹，键从 `kSyntheticKeyBase` 起递增——合成键只用于内部区分，不代表真实身份。

**置信度**在滑窗内精确求和，不归一化，因此随证据单调累积，不同航迹之间可直接比大小：

$$c_k=\sum_{s\in\mathcal{S}_k}\ \sum_{i\in W_{k,s}} v_i\,q_i\,w_s$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $c_k$ | 航迹 $k$ 的置信度 | — |
| $\mathcal{S}_k$ | 航迹 $k$ 有量测的源通道集合 | `track.channel_samples` 的键 |
| $W_{k,s}$ | 源 $s$ 在该航迹上的滑窗样本 | `window_size`（默认 10） |
| $v_i$ | 该拍判决值 | `DetectionRecord.verdict` |
| $q_i$ | 该拍质量因子 | `DetectionRecord.quality` |
| $w_s$ | 源权重 | `source_weights[source_id]`，缺项按 1.0 |

**生命周期**只有三档：累计命中数 `hits` 未达 `confirm_hits`（默认 3）为候选（`kTentative`）；达到即已确认（`kConfirmed`）；已确认但本周期无量测则转外推（`kCoasting`）。连续失跟超过 `max_missed_cycles`（默认 5）直接删除航迹。滑窗裁剪和失跟删除在同一趟里做，避免"先删除后裁剪"的悬空引用。

### 5.2 融合：无迹滤波与输出

滤波状态是六维 $[\mathbf{p},\mathbf{v}]$（ECEF 位置 + 速度），运动模型为匀速（CV），估计器为无迹卡尔曼滤波（UKF：`UnscentedPredictor` / `UnscentedUpdater`）。方位/俯仰到位置的映射是非线性的，UKF 直接用 sigma 点传播，不受线性化误差影响。

预测步：

$$\hat{\mathbf{x}}_k^-=\Phi(\Delta t)\,\hat{\mathbf{x}}_{k-1},\qquad \Delta t=\max\bigl(0,\ t_k-t_{k-1}\bigr)$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $\hat{\mathbf{x}}$ | 六维状态 $[x,v_x,y,v_y,z,v_z]$ | `track.filter_mean` |
| $\Phi$ | 匀速转移矩阵 | `LinearCvTransitionModel<6>` |
| $\Delta t$ | 距上一拍的时间差，s | 有时间戳取时间戳差；否则周期差 × `track_cycle_period_sec`（默认 1.0） |
| 过程噪声 | 扩散系数 $q$ | `track_process_noise`（默认 1.0） |

滤波器起始分两种：

- **位置起始**：均值取该拍位置、速度置 0，协方差各向同性（位置 1σ = `track_initial_position_std_m` 2000 m，速度 1σ = `track_initial_velocity_std_m_per_s` 300 m/s）。首拍不做量测更新（均值本来就是它）。
- **方位 + 传感器原点起始**：沿视线在 `track_bearing_init_range_m`（100 km）处放初值，距离不确定度 `track_bearing_init_range_std_m`（300 km）。首拍照常做量测更新——纯角度航迹的斜距收敛全靠这一步。

**两点差分初速**是建轨期最容易被忽略的一步。位置起始的航迹如果按零速先验硬推，头几拍速度新息会大幅超调；源码在第二个位置量测到来时直接差分定速：

$$\mathbf{v}_2=\frac{\mathbf{p}_2-\mathbf{p}_1}{\Delta t},\qquad \sigma_v^2=\frac{2\sigma_p^2}{\Delta t^{2}}+\bigl(100\ \mathrm{m/s}\bigr)^2$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $\mathbf{v}_2$ | 差分得到的初速，m/s | — |
| $\mathbf{p}_1,\mathbf{p}_2$ | 首拍 / 第二拍位置，m | `filter_mean` 位置轴 |
| $\Delta t$ | 两拍物理间隔，s | 时间戳差或周期差 |
| $\sigma_p$ | 位置量测 1σ，m | `default_position_noise_std_m`（默认 50） |
| $100$ m/s | 速度方差地板 | 硬编码，挡住长间隔差分方差过小 |

间隔不足 50 ms 不做差分：位置噪声除以极小 $\Delta t$ 会被放大成假高速。

量测更新按可用量测二选一：有位置用三维位置模型（量测噪声 1σ = `default_position_noise_std_m` 50 m）；只有方位用二维方位/俯仰模型（1σ 取记录自带 `bearing_noise_sigma_rad`，缺省 `default_bearing_noise_sigma_rad` 2.0e-4 rad）。

**失跟外推**：本周期没更新到的航迹只预测不更新，`dt` 为 0 的（刚更新过）自然跳过。

输出把状态还原成业务量：位置由 ECEF 转 LLA，速度直接给 ECEF 三分量，6×6 协方差原样透出，同时给一句可读的 1σ：

$$\sigma_{pos}=\sqrt{\sigma_x^2+\sigma_y^2+\sigma_z^2},\qquad \sigma_{vel}=\sqrt{\sigma_{vx}^2+\sigma_{vy}^2+\sigma_{vz}^2}$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $\sigma_x,\sigma_y,\sigma_z$ | 位置三轴 1σ（协方差对角 0/2/4 开方），m | `kinematic_estimate.covariance_ecef` |
| $\sigma_{vx},\sigma_{vy},\sigma_{vz}$ | 速度三轴 1σ（对角 1/3/5 开方），m/s | 同上 |
| $\sigma_{pos}$ / $\sigma_{vel}$ | 三维 RSS 1σ | — |

开方前先取 $\max(0,\cdot)$：滤波数值噪声会产生极小负方差，直接开方就是 NaN。

**合成量测不进滤波**是这一层最容易读错的规则：单星保活记录（`synthetic = true`）只参与命中计数、最近周期和锚点更新，不写滤波状态、不置 `has_real_update`。否则冻结距离会把滤波状态拖偏，精度评估也会把非独立量测当成真实观测给自己打分。

### 5.3 弹道推演：外推、落点、发射点

推演引擎吃的是融合航迹的位置、速度、协方差，做四件事：前向弹道、落点、发射点回推、目标类型。动力学是球面地球的中心引力 + 可选指数大气阻力（弹道系数 ≤ 0 时关闭阻力）：

$$\mathbf{a}=-\frac{\mu}{r^{3}}\,\mathbf{r}-\frac{1}{2}\,\frac{\rho\,\lVert\mathbf{v}\rVert}{B}\,\mathbf{v},\qquad \rho=1.225\,e^{-h/8500}$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $\mathbf{a}$ | 加速度，m/s² | — |
| $\mu$ | 地球引力参数，m³/s² | `earth_mu_m3_per_s2`（3.986004418e14） |
| $r$ | 地心距，m | $\lVert\mathbf{r}\rVert$ |
| $\rho$ | 大气密度，kg/m³ | 指数模型，标高 8500 m |
| $h$ | 地心高，m | $r-R_\oplus$ |
| $B$ | 弹道系数，m²/kg | `drag_ballistic_coefficient_m2_per_kg`（默认 0 = 关闭） |

数值积分用定步长四阶龙格–库塔（RK4），步长 `integration_step_sec`（默认 1 s，下限 0.01 s）：

$$\mathbf{y}_{n+1}=\mathbf{y}_n+\frac{\Delta t}{6}\bigl(\mathbf{k}_1+2\mathbf{k}_2+2\mathbf{k}_3+\mathbf{k}_4\bigr)$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $\mathbf{y}$ | 状态 $[\mathbf{r},\mathbf{v}]$ | `RvState` |
| $\mathbf{k}_1\ldots\mathbf{k}_4$ | 四段斜率（前三段取半点、末段取整点） | — |
| $\Delta t$ | 步长，s（可为负，负值即回推） | `integration_step_sec` |

前向预测跑 `prediction_horizon_sec`（默认 300 s），每 `waypoint_interval_sec`（默认 10 s）存一个航迹点。一旦某步地心距落到地球半径以内，就在这一步内做线性插值定落点，而不是把步长缩小重跑：

$$\mathbf{p}_{hit}=\mathbf{p}_n+\frac{r_n-R_\oplus}{r_n-r_{n+1}}\bigl(\mathbf{p}_{n+1}-\mathbf{p}_n\bigr)$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $\mathbf{p}_{hit}$ | 地表穿越点，m | — |
| $\mathbf{p}_n,\mathbf{p}_{n+1}$ | 穿越前 / 后一步的位置 | — |
| $r_n,r_{n+1}$ | 对应地心距，m | — |
| $R_\oplus$ | 地球半径，m | `earth_radius_m`（6378137） |

落点高度强制归零：引擎地表是球面（赤道半径），而 LLA 换算用 WGS84 椭球，两者混用在中纬度会造出约 5 km 的伪高差，交付口径直接写 0。

**发射点回推**用同一个 RK4，只把步长取负。回推有三个停机门，命中任一即停，停机点就是发射点估计：

| 停机门 | 判据 | 含义 |
|---|---|---|
| 地表 | $r\le R_\oplus$ | 弹道真的回到地面 |
| 大气界面 | $h\le$ `launch_atmosphere_interface_altitude_m`（100 km） | 再往回就进助推段（推力未建模），停在这里 |
| 低速 | $\lVert\mathbf{v}\rVert\le$ `launch_speed_threshold_m_per_s`（30 m/s） | 速度已接近静止 |

回推上限 `launch_max_backtrack_sec`（默认 900 s）；超限仍无交点则 `has_launch=false` 丢弃，不给假值。助推段没有建模，所以"发射点"严格说是**助推段上界**，不是发射台坐标——这是模型边界，不是精度问题。

**误差预算**用线性化敏度传播。对六个状态分量各加固定扰动（位置 100 m、速度 1 m/s），分别重跑回推，得到 $6\times6$ 敏度矩阵 $J$，再左乘右乘协方差：

$$P_{launch}=J\,P\,J^{\mathsf T}+\sigma_{model}^{2}\,I_{pos},\qquad \sigma_{model}=\dot{\sigma}_{model}\,\bigl|t_{back}\bigr|$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $P$ | 输入状态协方差（ECEF 6×6） | `track.covariance_ecef` |
| $J$ | 敏度矩阵 $J_{ij}=\partial y_i/\partial x_j$ | 六次扰动重推差分 |
| $\dot{\sigma}_{model}$ | 模型误差增长率，m/s | `launch_model_error_rate_m_per_s`（默认 50） |
| $t_{back}$ | 回推时长，s | 停机时的 `launch_elapsed` |
| $\sigma_{model}^2 I_{pos}$ | 只叠在位置对角上的模型误差 | — |

落点侧用同一套扰动的对角简化：$P$ 的非对角项在高维耦合下贡献是二阶小量，交付口径只取 $\sum_j\sum_i h_{ij}^2 P_{jj}$。

**类型评估**是运动学先验与外部证据的加权和，权重 `kinematic_type_weight`（默认 0.5）：

$$\mathbf{s}=w\,\mathbf{p}_{kin}+(1-w)\,\mathbf{e}_{ext},\qquad \hat{c}=\arg\max_i s_i,\qquad P(\hat{c})=\frac{s_{\hat{c}}}{\sum_i s_i}$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $\mathbf{p}_{kin}$ | 运动学先验（速度/高度门判出的向量） | `KinematicPrior` |
| $\mathbf{e}_{ext}$ | 外部类型证据（识别模块给出） | `track.type_evidence` |
| $w$ | 运动学权重 | `kinematic_type_weight`（默认 0.5） |
| $\hat{c}$ | 取得分最高的类别 | `InferenceTargetCategory` |
| $P(\hat{c})$ | 归一化后的置信度 | — |

运动学先验的门是速度 1200 m/s、高度 30 km：双门全过给弹道目标 0.55 / 导弹 0.30 / 临近空间 0.10；高速低空给导弹 0.45 / 战斗机 0.20；500 m/s 以上给战斗机 0.35 / 轰炸机 0.25；更低速统一归"其他"（原无人机倾向已按甲方裁定删除）。

#### 5.3.1 关机点判断

需求 3.2.1.5.3.3 要求判断助推段结束、发动机关机的时刻。`InferenceAcceptanceRecords::UpdateBurnoutTracker` 通过加速度和机械能双通道状态机实现：

**加速度通道**：从当前状态扣除引力加速度后，剩"非引力加速度"。当此残差连续 $N_{burnout}$ 拍低于阈值 $a_{thresh}$ 时，认为推力已消失。阈值内代码维护为 `kBurnoutAccelThreshold`（4 m/s²），连续拍数 `kBurnoutPersistence`（3 拍）。

**机械能通道**：计算单位质量的机械能 $E = v^2/2 - \mu/r$，当 $E$ 变化率连续低于阈值时，认为已进入自由飞行段。两通道同时满足时输出关机点标志、时间戳和对应 ECEF 位置。

> 当前关机点仅在验收旁路形成，未进入 `TargetInferenceResult` 正式产品；且目标动力学本身不建模助推段——回推在高度回到 100 km 时即停机（再往上就进入不可建模的燃烧段）。

### 5.4 精度评估：五项误差与综合分

评估会话每周期吃一次双星结果 + 真值 + 融合航迹，产出五项误差序列。五项各自独立累积，互不补齐——某项本周期没有有效样本就是没有，不拿别的项顶。

| 序 | 指标 | 怎么算 | 参考误差（默认） |
|---|---|---|---|
| 1 | 红外定位角度误差 | $(\Delta az,\Delta el)$ 的 $L_2$ 模，逐星逐拍 | 0.05° |
| 2 | 双星交会位置误差 | 两条视线最近点中点与真值距离 | 10 km |
| 3 | 融合速度误差 | 融合速度与真值速度的差模 | 100 m/s |
| 4 | 落点预测误差 | 估计状态与真值状态各自推演后的落点距离 | 10 km |
| 5 | 发射点预测误差 | 同上，取发射点 | 20 km |

**双星交会**是异面直线最近点问题：两条视线一般不严格相交（量测有误差、两星时刻有别），取最近点对的中点作定位解，两点间距作视线残差：

$$t=\frac{be-cd}{ac-b^{2}},\qquad s=\frac{ae-bd}{ac-b^{2}},\qquad \operatorname{res}=\bigl\lVert \mathbf{c}_a-\mathbf{c}_b\bigr\rVert_2$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $\mathbf{d}_a,\mathbf{d}_b$ | 两条视线的 ECEF 单位方向 | 角度 → ECI 方向 → 按 GMST 转 ECEF |
| $a=\mathbf{d}_a\!\cdot\!\mathbf{d}_a$，$c=\mathbf{d}_b\!\cdot\!\mathbf{d}_b$，$b=\mathbf{d}_a\!\cdot\!\mathbf{d}_b$ | 方向自/互点积 | — |
| $d=\mathbf{d}_a\!\cdot\!\mathbf{w}_0$，$e=\mathbf{d}_b\!\cdot\!\mathbf{w}_0$，$\mathbf{w}_0=\mathbf{O}_a-\mathbf{O}_b$ | 原点偏移投影 | 两星 ECEF 位置 |
| $t,s$ | 两线上的最近点参数 | — |
| $\operatorname{res}$ | 视线交会残差，m | — |

分母 $ac-b^2=|\mathbf{d}_a\times\mathbf{d}_b|^2$，近零即两线平行，返回失败而不是给一个发散的定位点。

配对窗 `dual_sat_pair_window_cycles` 默认 0，即严格同周期双检出才交会；设成正数则允许周期号相差不超过窗宽的两拍配对，两条视线各锚定各自测量时刻的卫星位置与 GMST——这就是"地面按时刻融合"的时间对齐环节。跨周期样本的误差天然含两拍间隔内的目标运动偏差，读数时不能和同周期样本混着比。

**偏差口径**：落点/发射点误差是"估计状态推演"与"真值状态推演"两条结果之差，用的是同一个推演引擎，所以衡量的是状态误差如何传播，不含引擎自身的模型误差。

**汇总**对每项序列给均值、RMSE、P95、最大值，其中 P95 用最近秩法（升序第 $\lceil 0.95n\rceil$ 个）：

$$\mathrm{RMSE}=\sqrt{\frac{1}{n}\sum_{i=1}^{n}e_i^{2}},\qquad P95=e_{(\lceil 0.95n\rceil)}$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $e_i$ | 第 $i$ 个误差样本 | 五项序列之一 |
| $n$ | 样本数 | `metrics[i].count` |
| $e_{(j)}$ | 升序第 $j$ 个样本（1 基） | — |

**零样本按 0 分**处理：样本数为 0 时把 RMSE 置 $+\infty$ 再进打分，避免"全程没检出反而满分"。

**打分与综合**先逐项归一化，再按 AHP 权重点乘：

$$s_i=\frac{\varepsilon_i^{ref}}{\varepsilon_i^{ref}+\mathrm{RMSE}_i},\qquad S=\sum_{i=1}^{5}w_i\,s_i$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $\varepsilon_i^{ref}$ | 第 $i$ 项参考误差 | `reference_error_*`（见上表） |
| $s_i$ | 第 $i$ 项得分，取值 $(0,1]$ | — |
| $w_i$ | AHP 权重，$\sum w_i=1$ | `AhpEvaluator` |
| $S$ | 综合分 | `report.composite_score` |

$\mathrm{RMSE}=\varepsilon^{ref}$ 时恰好 0.5 分；RMSE 越小越接近 1。参考误差默认值是演示口径，正式验收前要按装备指标替换——它不是性能门限，只是评分标尺。

**AHP 权重**由 5×5 正互反判断矩阵求出（默认全 1，即五指标等权、完全一致）。求解用幂迭代逼近 Perron 主特征向量，再用 Rayleigh 商求主特征值：

$$\lambda_{max}=\frac{(\mathbf{A}\mathbf{w})\cdot\mathbf{w}}{\mathbf{w}\cdot\mathbf{w}},\qquad CI=\frac{\lambda_{max}-n}{n-1},\qquad CR=\frac{CI}{RI}$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $\mathbf{A}$ | 判断矩阵，$a_{ij}\in[1/9,9]$，$a_{ij}a_{ji}=1$，对角为 1 | `AhpJudgmentMatrix` |
| $\mathbf{w}$ | 主特征向量（L1 归一化后即权重） | 幂迭代，收敛阈 1e-12 |
| $n$ | 指标数，恒为 5 | `kPrecisionMetricCount` |
| $RI$ | 随机一致性指标，$n=5$ 取 1.12 | `kRandomIndexTable[4]` |
| $CR$ | 一致性比例，≤ 0.1 才认为矩阵可用 | — |

矩阵非法（元素越界、对角非 1、互反不成立）或幂迭代不收敛时，`ahp_valid=false`、综合分保持 0，并显式写一行说明——不静默退化成等权。

### 5.5 接力三项与标准卡尔曼滤波行

融合输出还要回答"这颗星还能看多久、下一颗是谁"。库内没有接力协议，也没有轨道/视场模型，所以用几何外推给三个数：剩余覆盖时间、接力计划、交接指令。

$$\Theta_{swept}=\sqrt{\bigl(\Delta az_{unw}\bigr)^{2}+\bigl(\Delta el\bigr)^{2}},\qquad T_{rem}=\frac{\Theta_{fov}-\Theta_{swept}}{\omega}$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $\Delta az_{unw}$ | 自该源首次见到本航迹起累计的解缠方位变化，deg | 逐拍最短差累加 |
| $\Delta el$ | 首见俯仰与本拍俯仰之差，deg | — |
| $\Theta_{swept}$ | 已扫过角，deg | — |
| $\Theta_{fov}$ | 视场角宽度，deg | `relay_fov_width_deg`（默认 20） |
| $\omega$ | 视线角速率，deg/s | 8 拍滑窗最小二乘 |
| $T_{rem}$ | 剩余覆盖时间，s；≤0 记 0 | — |

方位必须先解缠再累加，否则跨 ±180° 会凭空多出 360°。角速率用最近 8 拍滑窗分别对解缠方位、俯仰做最小二乘斜率，再取两分量模：

$$\omega=\sqrt{\dot{a}^{2}+\dot{e}^{2}},\qquad \dot{a}=\frac{n\sum t_i a_i-\sum t_i\sum a_i}{n\sum t_i^{2}-\bigl(\sum t_i\bigr)^{2}}$$

| 符号 | 含义 | 来源 |
|---|---|---|
| $t_i$ | 相对窗首的时刻，s | 周期号 × `track_cycle_period_sec` |
| $a_i$ / $e_i$ | 解缠方位 / 俯仰，deg | 该源最近 8 拍 |
| $\dot{a}$ | 方位角速率，deg/s；俯仰同式 | — |
| $n$ | 窗内样本数，需 ≥2 且时间跨度非零 | `kRelayWindowSamples` = 8 |

剩余时间最小的源就是"最早离场者"（`exit_source`），其余源里取本航迹样本数最多的作接力对象；单星场景下从会话已知源全集里挑，优先挑曾测过本航迹的（接力链成员），否则取编号最小的其他源。库内不做视场归属几何判定——这是已知边界。

**标准卡尔曼滤波行**是验收要求单列的一路，与融合层的 UKF 并行存在、互不影响：对每个（航迹, 源）的视线角做四维线性 KF，状态 $[az,\omega_{az},el,\omega_{el}]$，匀速模型，量测噪声取 `default_bearing_noise_sigma_rad` 的平方。第一拍只初始化为量测角、变化率置 0；之后每拍先预测再更新。它输出滤波后的方位/俯仰与两个变化率，逐周期一行，用来看视线角是否平滑——不是融合结果的替代品。

### 5.6 特殊事件监测与信息分发

需求 3.2.1.6.1–2 要求 OPIR 产生特殊事件提示并分发落点预报。当前通过 OPIR 侧生命周期记录器产生事件，通过目标推演层输出落点预报。

**事件层**（OPIR 侧 `SbirsDetectionLifecycleRecorder` + `SbirsExclusionCauseRecorder`）：

| 事件 | 触发条件 | 持久化产物 |
|---|---|---|
| 目标发现 (`kDetected`) | 宽视场首次命中 | `detection_lifecycle.log` 行 |
| 目标更新 (`kUpdated`) | 已有 track 本周期命中 | 同上，追加 `cycle_no` |
| 滑行 (`kCoasting`) | 确认目标连续无命中 | 同上 |
| 丢失 (`kLost`) | 连续丢失超 `max_coast_cycles` | 同上，含最后已知位置 |
| 未检测 (`kNotDetected`) | 受 `emit_not_detected_events` 控制 | 同上 |
| 排除原因变更 | `exclusion_cause` 字段变化 | `exclusion_cause.log` 行，记录旧→新原因 |

**当前局限**：

1. **事件不带等级**：没有"重要/一般/信息"分级，无法按紧急程度过滤。
2. **事件不含关键点空间位置**：丢失事件不含目标当前推算的落点经纬度，接收方需要自行回查。
3. **未形成跨模块持久化产品**：事件仅写入 OPIR 侧的周期日志，没有统一的系统事件总线或结构化事件文档（如 JSON 事件流）。
4. **信息分发未实现**：落点预报通过 `TargetInferenceEngine::Infer` 同步返回，但没有发布/订阅机制、订阅者路由或任务级信息分发——下游必须主动拉取结果，无法被动接收推送。

## 6. 源码索引

| 文件 | 职责 |
|---|---|
| `src/sbirs_sensor/pipeline/SbirsPipeline.cpp` | 主流程、四道门限、轮转、捕获、跟踪段门限 |
| `src/sbirs_sensor/pipeline/SbirsPipeline.h` | 七态枚举、跨周期状态清单 |
| `src/sbirs_sensor/foundation/SbirsRadiometry.cpp` | 接收功率、SNR、$d_{max}$ 反解 |
| `src/sbirs_sensor/foundation/SbirsNoiseModel.cpp` | 噪声能量合成（无热噪声项） |
| `src/sbirs_sensor/foundation/SbirsErrorModel.cpp` | 五类角度误差、多帧 $1/\sqrt{N}$ 融合 |
| `src/sbirs_sensor/environment/SbirsEnvironmentModel.cpp` | 壳段气团 $\tau_{geo} = \tau_{eff}^{\,X}$ |
| `src/sbirs_sensor/pipeline/SbirsBoresightChain.h` | 视轴合成（姿态∘安装∘失准∘扫描）、反解、失准抽样 |
| `src/sbirs_sensor/pipeline/SbirsPointingActuator.cpp` | 限速转向、稳定时长 |
| `src/sbirs_sensor/pipeline/SbirsTrackingCoordinator.cpp` | EKF/IMM/角度 CV、NIS 双门限 |
| `include/1q/sbirs_sensor/session/SbirsOutputTypes.h` | 输出层字段全集（检测/航迹/覆盖包络） |
| `include/1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h` | 事件层：检测生命周期记录 |
| `src/remote_identification_radar/runtime/RirController.cpp` | 候选四道门限、覆盖门限、量测采样、包络门限与去重 |
| `src/remote_identification_radar/session/RirSession.cpp` | 驻留预算、外部窗口、cue 合理性门、指定任务状态机 |
| `src/remote_identification_radar/tracking/RirTrackLifecycle.cpp` | 生命周期、最小二乘速度/加速度、EMA |
| `src/remote_identification_radar/tracking/RirTrackAssociator.cpp` | 关联波门（马氏距离）+ LAPJV |
| `src/remote_identification_radar/recognition/MotionFeatureExtractor.cpp` | 运动学特征（速度/加速度/航向变化） |
| `src/remote_identification_radar/recognition/RcsFeatureExtractor.cpp` | RCS 特征（均值/起伏/极值） |
| `src/remote_identification_radar/recognition/RangeProfileFeatureExtractor.cpp` | 距离像特征（长度/散射中心） |
| `src/remote_identification_radar/recognition/PolarizationFeatureExtractor.cpp` | 极化特征（比/熵） |
| `src/remote_identification_radar/recognition/RecognitionMatcher.cpp` | 高斯相似度、加权打分、confidence |
| `src/remote_identification_radar/recognition/RecognitionTracker.cpp` | 滑窗聚合、识别状态机 |
| `src/common/radar/DetectionCellResolver.cpp` | SINR 计算（四增益 + MTI/MTD） |
| `src/common/radar/AntennaPatternRuntime.h` | 天线方向图三模型、分区增益、副瓣延拓、扫描损耗 |
| `src/common/radar/ScanScheduleRuntime.h` | 波位栅格、驻留数、优先级、外部窗口 |
| `src/common/radar/StatisticalCfarDetector.cpp` | CFAR 门限反解与五类起伏目标 $P_d$ |
| `src/common/radar/VegetationClutterModel.cpp` | 植被杂波倍乘与传播损耗 |
| `src/common/radar/RadarEquations.cpp` | 回波功率、热噪声、测距/测角误差 |
| `src/common/radar/MeasurementErrorModel.h` | 误差合成与系统偏置 |
| `src/common/rcs/RcsPhysics.cpp` | 等效半径、圆柱/双站/平板、混合 RCS |
| `src/fusion/FusionEngine.cpp` | 三级关联、置信度、UKF、合成量测隔离 |
| `src/fusion/FusionAcceptanceRecords.cpp` | 接力三项、精度输出、标准 KF 验收行 |
| `src/target_inference/TargetInferenceEngine.cpp` | RK4 弹道、落点/发射点、敏度误差预算、类型融合 |
| `src/precision_evaluation/PrecisionEvaluationSession.cpp` | 双星交会、五指标序列与汇总 |
| `src/precision_evaluation/DualLosFix.cpp` | 异面直线最近点交会 |
| `src/precision_evaluation/AhpEvaluator.cpp` | 幂迭代权重、CR 检验、综合分 |
| `src/precision_evaluation/PrecisionEvaluationMetrics.cpp` | 均值/RMSE/P95、归一化打分 |

> **公式出处与偏差**：本文只回答"源码怎么算"。若要查每条公式对应的权威定义、与实现的偏差量级（例如 Swerling 1 闭合式在 $N\ge2$ 的偏差、CFAR 损失、MTI 改善因子未实现），看 `detection_pipeline_algorithm_reference.md`。
