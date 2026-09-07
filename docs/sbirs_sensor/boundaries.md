---
Status: active
Last-reviewed: 2026-08-23
Authority: sbirs_sensor 模块级边界、非目标、能力决策与变更规则
Answers: SBIRS 有哪些模块级边界、哪些能力刻意不实现及为什么、输出归属规则、变更规则
---

# SBIRS 模块边界

本文承载 SBIRS 的模块级边界、输出归属、能力决策（刻意不实现什么及为什么）、非目标和变更规则。
算法级边界（状态机转移、EKF 参数、ATP 速率等）见 [algorithms.md](algorithms.md)。

## 电源状态单源边界

电源状态单源（COMMON-OQ-4 字段提升）：`SbirsSessionConfig::sensor_enabled` 是唯一权威（mission 域无
电源字段）；运行时唯一入口为 `SbirsRuntimeConfigPatch::has_sensor_enabled`，builder 为 `WithSensorEnabled`，
replay schema 的 session config 表承载 `sensor_enabled`。旧名 `power_on`/`has_power_on`/`WithPowerOn`
不得回流（见 `tests/contract/check_cross_domain_naming.cmake` 阻断 7 与 contract.md §电源状态单源契约）。

[evidence: tests/contract/sbirs_sensor/sbirs_public_api_convenience_test]
[evidence: tests/unit/sbirs_sensor/sbirs_runtime_config_resolver_test]
[evidence: tests/replay/sbirs_sensor/sbirs_replay_codec_roundtrip_test]

## 输出与仿真归属（分层周期记录 + 可选投影）

SBIRS 遵守分层周期记录 + 可选投影（`session_contract.md` 规则 15；旧称两通道/三层）。落地前仍为扁平 `SbirsCycleResult`。

| 层/投影 | 入口 | 责任 |
|---|---|---|
| 产品 | `Step()` → `SbirsOutputFrame` | 检测记录 |
| 周期记录 | `StepWithResult()` → `SbirsCycleResult` | 执行状态、诊断、仿真附件（归属） |
| 观测投影 | DebugView / Lifecycle / ExclusionCause | 人读快照、生命周期、排除差分 |

非执行周期（`status != kCompleted`）表示本周期没有产生新的目标观测事实。所有 recorder（
`SbirsDetectionLifecycleRecorder` 与 `SbirsExclusionCauseRecorder`）在该边界返回空事件
列表并保持全部累积状态；validation rejection 不得虚构 `Lost`、`NotDetected` 或 `TargetMissingFromInput`。
下一合法检测继续按拒绝前状态产生 `Updated`。

### 非执行周期统一不复用（五模块统一规则）

SBIRS 非执行周期（校验失败/执行 abort）的产品层一律为空载荷，**永不复用**上一有效输出（规则 15d）。
调用方用执行层 `status` / `abort_reason` 判断周期状态。落地前空帧 `cycle_index` 仍可能为 0。

[evidence: tests/contract/sbirs_sensor/sbirs_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailureAfterSuccess]

### 测量物理时刻与离散步调调度边界（连续物理时刻 vs 离散批次号）

`SbirsDetectionRecord` 纯粹表达单次物理红外曝光事实，携带 `measurement_time_s`（绝对仿真时刻，单位秒），
不设周期号字段。外层帧 `SbirsOutputFrame.cycle_index` 承载离散步调调度（如 1s 步长）批次标识。
调用方步调步长属于外层调度与批量通信（Frame Batching）；多星异步扫描下单周期内各星检出时刻
（如 A 星 0.5s、B 星 0.8s）由各自 `measurement_time_s` 准确记录，时空语义严格解耦。

[evidence: include/1q/sbirs_sensor/session/SbirsOutputTypes.h::SbirsDetectionRecord]
[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test.cpp::WfovCrossingTimeCalculatesSubCycleMeasurementTimestamp]

### 三写约束（abort_reason + issues + 日志）

SBIRS 所有中止路径遵守 `session_contract.md` 规则 9 的三写模式与规则 14 的统一问题列表模型：

1. **结构化信号**：`SbirsCycleResult.abort_reason`（粗粒度枚举：`kValidationRejected`、`kSensorPoweredOff`）。
2. **结构化诊断**：`SbirsCycleResult.issues`（`SbirsIssueList`，细粒度 code 如
   `"sbirs.sensor_powered_off"`、`"sbirs.validation.invalid_satellite_position"`；
   条目携带 `phase` 来源标签与可选定位）。
   本模块 code 全集单一事实来源：`include/1q/sbirs_sensor/session/SbirsIssueCodes.h`（规则 14c）。
3. **人读日志**：`PROJECT_LOG_ERROR`。

`SbirsCycleResult` 只承载单一问题列表 `issues`：输入校验问题（`phase=kInputValidation`）与执行诊断
（`phase=kExecution`）同列表承载，不设 `validation_issues`/`has_validation_error` 平行字段。
校验拒绝时校验问题本身就是 error 级诊断（规则 9 写二），不再附加粗粒度条目（COMMON-OQ-9
收敛：校验拒绝路径不再调用 `RecordAbort`，显式补齐 `abort_reason` 与日志）。

三写由 `SbirsDiagnosticUtils::RecordAbort` 统一执行（phase 由中止原因推导）；校验拒绝路径
不调用 RecordAbort（校验问题本身即写二）。

正常执行周期（`status == kCompleted`）的按目标排除诊断（kInfo，code 如
`"sbirs.target_out_of_wfov"`）**不属于三写**（session_contract.md 规则 13b）：仅承载排查信息，
由 pipeline 内联写入、controller 层并入 `SbirsCycleResult.issues`，调用方按规则 12
落盘 DebugView 时自然携带；不改变周期状态与 DebugView 状态语义。周期级 `PROJECT_LOG_INFO`
执行摘要（规则 13a，格式 `[SbirsPipeline] cycle_index=… scan_az=… detections=… excluded=…`）
在 pipeline `RunCycle` 每次实际执行后输出，仅人读，不用于状态判断（规则 3）。
**实体机器可读关联（规则 14e/13b）**：排除诊断结构化携带 `location = {kSceneEntity, target_index}`
（`MakeExclusionIssue` 由 pipeline 主循环索引赋值），供 `SbirsExclusionCauseRecorder` 按实体
关联消费。SBIRS 排除诊断涵盖 4 个 code：遮挡/距离带（具体门，cause 恒 kNone）、视场/SNR
（聚合门，有细分 cause）。
**排除原因跨周期差分（规则 13e）**：`SbirsExclusionCauseRecorder` 对持续被排除目标做
`(code, cause)` 对差分，产出 A2 进入/A3 原因变化/A4 退出事件。差分键为组合对（非纯 cause），
正确捕获遮挡↔距离带切换（同为 kNone、code 不同）的 A3 变化。纯观测只读 `result.issues`
（仅消费 `phase == kExecution` 且 `location.kind == kSceneEntity` 的条目——输入校验
问题也可能用 kSceneEntity 定位，混入会误当排除诊断），与 `SbirsDetectionLifecycleRecorder` 并列
（独立 Attach/驱动/GetLastEvents），注册与否不影响执行语义（规则 11c）。**消失目标边界**：
recorder 只遍历当前周期 `input.scene`，目标从输入消失时其排除状态条目保留（不会被 A4 清除，
与既有 `SbirsDetectionLifecycleRecorder` 的"消失目标状态保留"行为一致）；重现为 A3 而非 A2。

**验收信息日志（`opir_acceptance.log`，2026-08-22）**：CMake 开关
`ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG`（默认 OFF）门控。开启后按验收项写入运行目录
`opir_acceptance.log`（四段同一行：仿真时间 / 仿真周期 / 验收项 / 验收内容），
不进 `1q_library.log`，也不把融合/推演/精度抄进本文件。关闭时宏与派生计算一并
编译剪除，零开销、行为逐位不变。项表与派生公式见
`docs/review/acceptance_item_catalog_2026-08-22.md`。边界：仅人读验收材料，
不属于三写、不进 raw output/attribution/DebugView。地面覆盖区投影使用与遮挡判定
同口径的圆球地球模型（`kEarthRadiusM`）；焦平面脱靶量由
`focal_length_m`/`detector_pixel_pitch_m` 配置映射（仅日志消费，不进标量探测链路）；宽窄切换连续命中计数器
（`wide_to_narrow_required_consecutive_hits`，默认 1 = 既有单次命中调度行为）进快照。

### 输出规则（WFOV/NFOV 状态仅决定当前周期哪些目标输出检测记录，不进 raw output 字段）

1. WFOV 阶段（`WideCandidate`）：输出 WFOV 检测成功目标的检测记录，位置为带误差值。
2. NFOV 指向等待周期（ATP 未 settled）：继续输出 WFOV 检测记录；attribution 携带已预留的
   `nfov_channel_id`。
3. NFOV 首次捕获成功：Estimated 输出带误差角度，Strict 输出真值，Sensor-like 用独立子流产生带误差角度
   和诊断距离；三者均记录明确 `tracking_source`。
4. NFOV 持续跟踪且门通过：Estimated 输出滤波后验，Strict 输出真值，Sensor-like 输出带误差观测。
5. NFOV 单周期门失败但未达丢锁阈值：raw output 无记录；attribution/debug/lifecycle 标记 `Coasting`
   并保持锁定（单镜筒轮转下分离目标轮空周期即此态，帧数记 0）。
6. NFOV 门连续失败达阈值：raw output 无记录；result attribution 携带 `kNfovTrackingGateLost` 并释放锁定。
7. NFOV 首次捕获失败或 pointing timeout：raw output 不含失败记录；result attribution 携带
   `kNfovAcquisitionFailed` 或 `kNfovPointingTimeout`。
8. WFOV 级几何/SNR 排除（地球遮挡、距离门、视场外、WFOV SNR 低于 `wide_min_snr_linear`）：
   raw output 无记录、无 attribution；`SbirsCycleResult.issues` 携带 kInfo 排除码
   （`sbirs.target_occulted` / `sbirs.target_out_of_range` / `sbirs.target_out_of_wfov` /
   `sbirs.target_snr_below_threshold`，message 含 `target_id` 与关键量值）；DebugView 状态仍为
   `kNotInOutput`（规则 13b/c）。
9. **门内归因（规则 13b 归因条款）**：`SbirsIssueCause` 给出机器可读主因——视场门按越界轴细分
   （`kAzOutside` / `kElOutside` / `kBothAxesOutside`，与 `InRectangularFov` 同基准）；
   SNR 门为聚合门（距离²/大气透过率/目标签名折入单一门限），反事实判定主因（距离参考
   1000 km、大气全透过、目标签名取"使 SNR 恰达门限的签名"；目标签名即输入辐射强度
   `radiant_intensity_w_per_sr`，W/sr），损失最大者为
   `kDistanceLimited` / `kAttenuationLimited` / `kSignatureLimited`；遮挡与距离门为具体门
   （cause 恒 `kNone`），message 分别补遮挡余量（`ComputeEarthOccultationMarginM`，负值 =
   遮挡深度）与距带边余量。

仿真归属（detection id → 输入 target id/name）、debug view、lifecycle（found/lost）、replay 仅进
`SbirsCycleResult` 和调试视图层，不得混入 `SbirsOutputFrame`。状态机内部状态如需调试，通过稳定的
status/stage/reason/coasting/gate 语义派生，不直接公开 internal `SbirsTargetState` 枚举。

[evidence: tests/unit/sbirs_sensor/sbirs_cycle_output_builder_test]

### 归属字段边界（不进 raw output）

1. `SbirsSceneTarget.velocity_ecef_m_per_s`：目标速度真值，驱动 cue 延迟外推与动态滞后误差；进 replay。
1b. `SbirsDetectionAttributionRecord.max_detection_range_m`：当前时刻最大探测距离
    d_max(t)（WFOV 门限反解，依赖该目标辐射强度、逐目标路径透过率 τ_geo=τ_eff^X（壳段
    气团，2026-09-02 起）与本周期噪声快照，逐目标、逐周期变化；合同指标 4，
    2026-08-17 起）；进 attribution/debug/lifecycle，进 replay；不进 raw output。
    SNR 门失败目标不产生归属记录，其 d_max 数值写入 `sbirs.target_snr_below_threshold`
    issue 消息（人读诊断）。
1c. `SbirsCycleInput.satellite_attitude_eci_body_deg`：卫星姿态（Body→ECI，Z-Y-X 欧拉，
    必填，零欧拉合法 = 体轴对齐 ECI；2026-08-17 阶段 2 起）；与 `SbirsOrientationConfig` 安装角
    合成指向链，只影响内部光轴几何，不进输出；进 replay。
1d. `SbirsOrientationConfig`（安装指向域，静态初始化配置，不进 RuntimeConfigPatch）：
    `mount_angles_deg`（Body→Sensor）、`sensor_scan_limits_deg`（传感器系扫描限位）、
    `stabilization_mode`（体/惯性稳定）、`misalignment`（安装失准误差模型，阶段 3：
    常值偏置 `bias_deg` + 随机微扰 1-σ `random_sigma_deg` + 种子 `random_seed`，默认全零）；
    进 session replay。
2. `SbirsCaptureFailureReason` + `capture_failure_reason`：捕获失败/调度跳过/NIS 丢锁诊断；进 attribution
   与 lifecycle reason，进 replay。
3. 闭环跟踪诊断（实际光轴误差、几何/SNR 门状态、连续失败计数、coasting 标志）：进 attribution/debug/
   lifecycle/replay。
4. `nfov_channel_id`：单镜筒编号（2026-09-02 单镜筒化后窄场相关行恒 0，-1 表示 WFOV/未占用；
   字段保留以稳定 replay 表与既有消费者）；进 attribution/lifecycle/debug，进 replay。
5. cross-cue 诊断字段（2026-09-01）：
   `cue_source_satellite_entity_id`（引导来源星，-1=自星宽场，随调度器通道持久、释放清除）、
   `measured_range_m`（误差模型距离，宽场段=candidate.measured_range_m，cross-cue 递话数据源）、
   `SbirsCycleResult.wide_cue_measurements`（每周期宽场候选带误差量测，组件据此构造递话）；
   三者均仅归属/调试面，不进 raw output、不进验收日志行；外部引导消息经运行时注入
   `SbirsSession::SubmitExternalCue`，不进每帧输入、不进回放（回放从不注入，快照不含该队列）。

`SbirsCycleResult.abort_reason` 解码只接受当前枚举中的 `kNone` 与 `kValidationRejected`；未知数值在修改
输出前拒绝，已删除的 reason 不提供 replay 数值兼容路径。

[evidence: tests/replay/sbirs_sensor/sbirs_replay_codec_roundtrip_test]

## 专项序列验证边界

`batch_validation::sbirs_sensor` 覆盖双目标同帧双锁（单镜筒窗口+同帧搭车）、三目标轮转交接、
持续机动引发 NIS 丢锁与重捕获、带横向速度的 cue latency、地球遮挡再现、standby 任务重定向和
无效输入恢复。

影响退出码的硬检查：
1. 预期未执行周期数。
2. 单周期 NFOV 服务目标集合一致性。
3. 场景特定的轮转/目标数量与中断-恢复结果。
4. NIS 超门/丢锁/重捕获事件。
5. `replay_complete` 和 `failure_marker_count`。

batch 未直接证明锁定集合跨周期稳定映射、无效输入前零 mutation、恢复后滤波/锁定连续性或与 clean
session 等价；这些性质只能由对应的 unit/integration/replay 测试作为证据。红外链路物理趋势仍为 warning。

## 能力决策与重新进入门

当前定位是**系统级、可解释、可确定性 replay 的 SBIRS-inspired 仿真**。下表不是 backlog 或优先级；
没有可复现失败、误差预算和验收门的候选不计入当前架构债务，也不得仅凭"真实性可能提高"进入生产。

| 能力 | 当前决策 | 证据或重新进入 Stage A 的必要条件 |
|---|---|---|
| 捕获后闭环 ATP 跟踪 | implemented | 单镜筒共享执行器接线（2026-09-02 单镜筒化）；predict→advance→gate→correct、coasting、丢锁、snapshot/replay 均有测试证据 |
| 时间相关姿态抖动与指向误差 | implemented | 已接入共模 WFOV/NFOV 与单通道 NFOV（0 号，单镜筒）；零幅默认，不等同完整整星控制器 |
| CA cue predictor | reject for wiring | 标称噪声和较长 latency 下放大误差、降低捕获率（73.91%→41.30%），未通过零回退门；不得接入 config/schema/pipeline |
| 简化整星姿态动力学与执行机构约束 | defer | 必须先给出当前角度域模型无法满足的可复现失败和误差预算，再冻结共享平台姿态与逐通道光轴所有权 |
| 多通道机械耦合与共享姿态资源 | implemented（单镜筒化，2026-09-02） | 独立 LOS 假设的可复现错误已坐实（三星场景分离 17.2° 双目标被同星双锁，几何+日志双探针；冻结契约已删档、见 git 历史）后收口：窄场收敛为单执行器分时轮转 + 同帧免费多跟，`max_concurrent_nfov_locks` 删除，可保持精跟条数由跟踪门物理涌现；确定性轮转/失败归属/snapshot/replay 均有测试证据 |
| 探测器像元、背景杂波与图像帧 | defer outside current product boundary | 仅在产品目标转为图像检测/TBD/NCC 且具备 PSF/MTF、焦距、像元几何、背景和独立物理真值时重开；归属独立 imaging 子系统 |
| 验收信息日志（`opir_acceptance.log`） | implemented（编译期门控） | `ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG` 默认 OFF 零开销；开启后写四段中文验收行，不进 `1q_library.log` |
| WGS84 椭球地面投影 | reject（保持圆球单口径） | 覆盖区投影与遮挡判定共用 `kEarthRadiusM` 圆球；引入椭球须先给出双地球模型口径分叉的可复现验收失败 |
| 覆盖区交集多边形 | reject（范围框口径，2026-09-06 裁剪包络化） | 覆盖区输出为视场∩地球盘交集的经纬度范围框，映射西北/东北/东南/西南 4 角；不做交集多边形顶点序列（盘缘弧线区域被范围框放大为已知失真）；重开须给出按多边形消费该字段的外部判据 |
| 反子午线经度环绕处理 | reject | 覆盖区跨 ±180° 时范围框经度取大圈方向（三星场景 104 星 118/480 行显现，数值偏大失真）；按字段消费 4 角数值的现行验收判据可读但须知此失真；重开须给出按最短弧口径消费经度的外部判据 |
| 威胁等级/航迹质量/任务优先级联合评分 | reject in sensor ownership | 分层契约归决策层（threat_assessment）；传感器侧只输出通道调度/资源跳过/失败回退事件 |
| 高精度轨道传播 | reject in sensor ownership | cycle input 已提供同一时标下的平台/目标状态；默认归属场景或平台动力学模块 |
| 地面任务规划、区域重访与星座协同 | reject in sensor ownership | 属于任务规划/星座资源域，应通过 session config/input 驱动传感器 |
| 传感器管线内目标运动几何外推（方案 A） | reject | 在 ECI 惯性系下引入地转假位移（静止目标失真），且与 `ApplyAngularErrorModel` 中探测器积分动态滞后误差模型重复叠加，破坏高速目标视线几何；时空对齐与运动补偿严格归属下游融合层（FUSION-OQ-1） |
| 复刻真实 SBIRS 保密参数或处理链 | reject | 不可审计、不可验证；只使用可追溯公开资料、仓库内模型假设和独立测试证据 |

[evidence: tests/unit/sbirs_sensor/sbirs_cue_predictor_test]
[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test]

## 非目标

1. **图像级 TBD（Track-Before-Detect）**：第一版用 WFOV 单帧 SNR 门控判定可探测性，不做管道滤波、
   帧间能量累积或动态规划 TBD。理由：TBD 需要多帧图像缓存和速度空间搜索，是独立的图像处理子系统。
2. **模板匹配 NCC 窄视场捕获**：用 WFOV cue + 几何窗口 + SNR 门判捕获，不实现归一化互相关。理由：
   NCC 依赖图像级数据，当前几何+SNR 判定已覆盖捕获语义。
3. **在线残差驱动的自动滤波后端切换**：后端选择由显式配置决定，保证 replay 可复现。可接受的"智能"
   形态为只读 NIS 诊断 + 人工看报告改配置。
4. **Otsu/DBSCAN 多目标聚类**：按 `target_id` 独立维护状态机，不做自适应阈值分割或聚类。理由：
   聚类针对图像级检测点，当前目标来自输入场景的显式目标列表。
5. **完整整星姿态系统 / CA / 6D-9D 搜索 / 轨道预测**：捕获后 tracking 已使用实际
   actuator LOS 门控，但不扩展为完整整星姿态动力学（见能力决策表 defer 项）。多通道机械耦合已由
   单镜筒化消解（2026-09-02，窄场只有一个镜筒，不再存在"多镜筒耦合"问题）。
6. 不暴露用户自定义 pipeline、controller、状态机、环境模型或 foundation algorithm 类型。
7. 不把仿真目标 ID/name 混入 `SbirsOutputFrame` 的 raw detection。
8. 不把 debug view、lifecycle 或 replay 当作 1q 仿真传感器主输出。
9. 不为测试 mock 便利新增 public 扩展点。
10. **验收量不进公开输出**：地面覆盖区/驻留时间/焦平面脱靶量/信号能量/连续命中计数只走
    `opir_acceptance.log`（人读验收材料），不扩展 `SbirsOutputFrame`、attribution 或
    DebugView 字段；公开输出结构变化须走输出边界评审。

## 设计变更规则

1. `SbirsOutputFrame`、检测记录字段、`SbirsCycleResult` 或 attribution/debug/lifecycle 语义变化，必须同步
   本文和输出边界测试；不得为复用 EOS consumer 而把 range、visible/fused SNR 塞进 raw output。
2. 状态机状态集合、转移条件、优先级规则变化，必须同步 algorithms.md 的状态转移图/转移条件表和
   `sbirs_state_machine_test`、`sbirs_scheduler_test`。
3. 气象影响列表、加权叠加公式 `A_total`、衰减进入 SNR 链路的方式变化，必须同步 algorithms.md 和
   `sbirs_environment_model_test`、`sbirs_radiative_transfer_test`。
4. 误差模型（5 类误差、加法/乘法合成、折射角与滞后公式）变化，必须同步 algorithms.md 和
   `sbirs_error_model_test`；同时检查对 NFOV cue 指向、首次捕获成功率的影响。
5. runtime patch 的可变字段、立即提交策略或状态机 capture/restore 规则变化，必须同步 data-flow.md、
   contract.md 运行期配置提交策略表和 runtime resolver 测试。
6. foundation 算法如果从 internal 变成 public API，必须在 `[evidence: ...]` 标注中记录扩展理由、稳定性
   约束和迁移影响，并检查与 EOS 对应算法的偏离是否有意。
7. 新增 debug/replay 字段时，必须保持真实输出、结构化结果和仿真辅助视图三层分离。
