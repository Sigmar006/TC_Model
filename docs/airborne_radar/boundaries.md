---
Status: active
Last-reviewed: 2026-08-23
Authority: AR 模块级边界、非目标与设计变更规则
Answers: AR 有哪些模块级禁令与边界、哪些非目标、配置/环境/校验/滤波的特殊语义、文档变更规则
---

# Airborne Radar 模块边界

本文承载 AR 的模块级边界、非目标、反直觉配置语义和设计变更规则。算法级边界（统一物理探测链的两
噪声基准、饱和降级、检测门限归属、欺骗 ECCM 路由等）见 [algorithms.md](algorithms.md)。

## 与 common 契约的关系

AR 遵守 `docs/common/contract.md`：

1. public API 只暴露稳定 session/config/input/output/Recording/Replay/decision DTO 门面。`EnvironmentService`、
   `SignalPipeline`、`ArController`、`MutableArContext`、tracking lifecycle 和战术决策部件不通过 public
   header 暴露。
2. 会话配置直接赋值 `ArSessionConfig`（语义档位是 `ArProfileConstants.h`
   中的预定义结构体常量）；运行期热更新直接写 `ArRuntimeConfigPatch`（显式 `has_*`）。
   不提供 ConfigBuilder，不承担 leaf setter 或隐式 validation。
3. AR 输出遵守分层周期记录 + 可选投影（`session_contract.md` 规则 15；旧称两通道/三层）。
   产品层是 `TrackOutputFrame`；执行/副作用/仿真附件/闭环 provenance 在周期记录里分层，
   **不**把 `DecisionObservation` 放进记录（15f 已落地：观测袋与 public 头已删除）。
   仿真真值归属挂仿真附件层（`ArCycleResult.track_attributions`，权威路径）；产品航迹内嵌的
   `external_target_id`/`target_name` 为注册 deprecated 遗留（sim-only），回收由后续独立工作处理。
4. decision 内部闭环是特征驱动的下一拍控制；唯一 public 缝是 `SubmitExternalDecision()`
   （规则 15f）。`DecisionObservation` 不得进入周期记录。

## dt_sec 校验边界（反直觉，勿按"四模块一致"补齐）

`ValidateArCycleDeltaTime`（`double` 类型）对 `dt_sec` 仅校验有限性 + 正值，**故意不含** EOS/SBIRS 的
`dt_sec ≤ 10/frame_rate_hz` 上界。

1. AR 配置中没有 `frame_rate_hz` 概念——主动雷达的周期节拍由 PRF、驻留时间、航迹更新率等雷达域量在
   各自子链路里约束，而非成像帧率。
2. dt 的合理性由 PRI/rejitter、emission 调度和 signal pipeline 消费链把关，不适用一个全局 frame_rate 上界。
3. 该差异已由 `ArCycleInput.dt_sec` 为 `double`（其余模块 `float`）、AR 无 frame_rate 字段、本校验链实测
   三方共同固化。

不得为"四模块一致"给 AR 加 frame_rate 上界。

[evidence: src/airborne_radar/session/ArInputValidation.cpp]

## 环境/RF 事实边界

`EnvironmentScenarioConfig`（`ArSessionConfig.environment`）与 `EnvironmentSnapshot` 只承载自然环境事实：

1. **不包含** jammer、J/S、J/N、干扰检测布尔值或预计算接收功率。AR 的外部 RF 输入是独立的
   `oneq::electromagnetics::RfEmissionFrame`，通过 `ArCycleInput::interference` 直接传入。空 frame 表示无
   外部 RF；非空 frame 必须与 AR 周期号、绝对窗口起点和时长完全一致，否则整个周期拒绝。
2. AR **不从环境场景推导干扰**。调用方只提供实际发射事实（发射 platform/equipment/emission 身份、ECEF
   运动学、天线、极化和参数化波形）；接收功率、PSD、J/N、饱和和观测质量全部由 AR 当前接收机链计算。
3. 大气物理附加损耗由信号层 `ComputeTargetSpecificAtmosphericLossDb` 用**每个目标的真实几何**计算，
   不在环境层重复计算；环境层硬编码几何的死计算已移除，`EnvironmentSnapshot` 不再承载
   `atmospheric_physics_loss_db` 字段。
4. AR **不保留** legacy jammer DTO、技术类别、J/S 摘要、欺骗/转发适配层，也不直接获取 ECM 的
   `EcmDeceptionMode` 真值。外部欺骗发射与压制发射经同一 `RfEmissionFrame` 路径进入接收前端。

拒绝暴露 `SpaceWeatherContext`（年积日、F10.7、地磁 Ap、仿真 Unix 时间戳）：当前 GTD7 大气模型退化为
ISA 标准大气，这些字段全部未被消费，属未接入的死输入。仿真时间统一以 `ArCycleInput::cycle_start_time_s`
为唯一来源。

[evidence: tests/unit/airborne_radar/ar_environment_config_contract_test.cpp]
[evidence: tests/unit/airborne_radar/ar_environment_service_test.cpp]

## 输出/输入校验与失败行为

`ArSession` 和 `ArController` 都有明确的失败语义：

1. **校验层归属（COMMON-OQ-9 收敛，2026-08）**：公共路径入口校验在 `ArSession`
   （`ValidateArCycleInput` 含 ENU→雷达体系旋转，控制器输入面不含 platform/targets 原始
   数据，无法下移）；运行期校验唯一化在 `ArController::RunOnce`（会话层对同一输入的二次
   校验已删除），拒绝时明细经出参直通并装配进最终周期结果；运行期执行失败透传真实
   `abort_reason`（校验拒绝为 `kRejectedInvalidInput` + 细粒度明细），不写死替换。
   场景目标输入为平台锚点 radar-local ENU（`ArTargetInput`，契约见
   docs/common/contract.md「场景目标平台锚点 ENU 输入契约」）；集成侧以公共
   `TryMakeEnuSceneState` 直填。库内经 `TryMakeArTargetFromEnu`（`ArRadarFrameTransform`）
   旋入雷达体系（平台姿态∘安装角复合）后供检测/关联/跟踪使用。平台位姿类型为
   `ArPlatformInput`（ECEF），不是场景目标适配器。
2. cycle input 校验失败时不执行 pipeline，`ArCycleResult` 携带 validation issues 与显式 abort
   reason `kValidationRejected`（保留 replay/trace 数值语义）。
3. **非执行周期统一不复用**：`Step()` 与产品层返回空载荷，不论是否存在上一有效输出（规则 15d）。
   调用方仅凭 `Step()` 即可判定本轮无新航迹。状态判断走 `StepWithResult()` 的执行层
   （`kRejectedInvalidInput`/`kPoweredOff`/`kRejectedExecution`）。落地前空帧 `cycle_index` 仍可能为 0。
4. controller 内部 `last_cycle_reused_previous_output` 仅是 RF 接收/检测侧跨周期状态机的簿记标志
   （捕获/恢复 snapshot 用），不进入 public `ArCycleResult`，也不等于公开输出帧被复用；不得据此推断
   `Step()` 会回传历史航迹。
5. signal pipeline abort 时不会发布合成的最新输出。
6. 电源状态单源：`ArSessionConfig::sensor_enabled` 是唯一来源（mission 域无电源字段），运行时电源唯一入口
   为 `ArRuntimeConfigPatch::has_sensor_enabled`。
7. 设备关机是已接受的非执行配置边界：撤销周期副作用后 finalize 关机配置，并保留外部决策等待下一成功周期。

[evidence: tests/contract/airborne_radar/ar_public_api_convenience_test.cpp::RejectedCycleDoesNotReusePreviousOutput]
[evidence: tests/contract/airborne_radar/ar_public_api_convenience_test.cpp::StepReturnsEmptyCurrentFrameOnRejectedInput]

## 输出边界要求

1. track output 保持系统侧航迹语义；名称、仿真便利信息或调试归因不能替代 stable association key/status。
2. `ArCycleResult` 由 `ArSession` 汇总 controller、context 和 pipeline 状态。按规则 15
   分层：执行状态与 `issues`、产品航迹、`emission_frame` 副作用、归属、闭环 provenance。
   不携带 `decision_observation`（15f 已删除）。完整 proposal、待消费响应与 reducer 计数
   只属于内部 replay 状态，不进入 public 业务结果。
3. query/debug/lifecycle/replay 是诊断辅助，不是用户扩展 signal pipeline 的入口；决策 SPI 不拥有输出结构，
   也不能绕过内部 output adapter 写系统输出。

### 三写约束（abort_reason + issues + 日志）

AR 所有中止路径遵守 `session_contract.md` 规则 9 的三写模式与规则 14 的统一问题列表模型：

1. **结构化信号**：`ArCycleResult.abort_reason`（粗粒度枚举）。
2. **结构化诊断**：`ArCycleResult.issues`（`ArIssueList`，细粒度 code 如 `"ar.sensor_powered_off"`、
   `"ar.validation.invalid_cycle_delta_time"`；条目携带 `phase` 来源标签与可选定位）。
   本模块 code 全集单一事实来源：`include/1q/airborne_radar/session/ArIssueCodes.h`（规则 14c）。
3. **人读日志**：`PROJECT_LOG_ERROR`。

`ArCycleResult` 只承载单一问题列表 `issues`：输入校验问题（`phase=kInputValidation`）与执行诊断
（`phase=kExecution`/`kOutputContract`）同列表承载，不设 `validation_issues`/`has_validation_error`
平行字段。校验拒绝时校验问题本身就是 error 级诊断（规则 9 写二），不再附加粗粒度条目。

三写由 `ArDiagnosticUtils::RecordAbort` 统一执行（phase 由中止原因推导），在 `ArSession` 的周期
装配路径中调用。

**正常周期的按目标排除诊断（规则 13b）**：正常执行周期（`status == kCompleted`）中被 SNR 检测门
排除的目标（`min_snr_db` / `min_detection_margin_db` 任一未过；距离/方向图衰减隐式并入 SNR）写
`kInfo` 级 `ArIssue`（code `"ar.target_snr_below_threshold"`，message 携带 `target_id` 与
`snr_db`/`range_m`/门值/偏轴角，phase=`kExecution`），**不属于三写**（三写仅约束中止路径，规则 9）。
**门内归因（规则 13b 归因条款）**：SNR 门为聚合门（距离/波束偏轴/噪声底/RCS 折入单一门限），
`ArIssueCause` 给出机器可读主因——按各因素相对参考状态（1 km 距离、主瓣中心增益、1 m² RCS、
热噪声底、零传播损耗）的损失 dB 判定，损失最大者为 `kDistanceLimited` / `kBeamLimited` /
`kNoiseLimited` / `kRcsLimited`（传播损耗并入距离项）；无法判定为 `kUnknown`。
诊断不改变 `ArCycleStatus`
与 DebugView 状态语义（排除目标仍为 `kNotInOutput`，规则 13c）；生命周期失效（miss 积累 → `kLost`）
不产生排除诊断（规则 13d）。周期摘要日志（`[SignalPipeline] … excluded={{snr=…}}`）仅人读（规则 13a）。
**实体机器可读关联（规则 14e/13b）**：排除诊断结构化携带 `location = {kSceneEntity, target_index}`
（`MakeExclusionIssue` 由 `RunPhysicalDetectionPass` 循环索引赋值），供 `ArExclusionCauseRecorder`
按实体关联消费。**位置对齐**：`entity_index` 是 `RunPhysicalDetectionPass` 内 `ArSceneTargetList`
索引，recorder 按目标在 `ArTargetInputList` 中的位置 find；两者对齐依赖 `ArSession` 适配器
"按序无过滤"地把 `input.targets` 映射到局部场景目标表（过滤/跳过即整周期拒绝）。若未来
适配器改为可跳过单目标的过滤逻辑，此对齐会被无声破坏，须同步评估 recorder 实体关联。
**排除原因跨周期差分（规则 13e）**：`ArExclusionCauseRecorder` 对持续被排除目标做
`(code, cause)` 对差分，产出 A2 进入/A3 原因变化/A4 退出事件；纯观测只读 `result.issues`
（仅消费 `phase == kExecution` 且 `location.kind == kSceneEntity` 的条目——输入校验
问题也可能用 kSceneEntity 定位，混入会误当排除诊断），与 `ArTrackLifecycleRecorder` 并列（独立 Attach/
驱动/GetLastEvents），注册与否不影响执行语义（规则 11c）。**消失目标边界**：recorder 只遍历
当前周期输入目标表，目标从输入消失时其排除状态条目保留（不会被 A4 清除，与既有
`ArTrackLifecycleRecorder` 的"消失目标状态保留"行为一致）；重现为 A3 而非 A2。

### 周期记录分层（规则 15）：不要把闭环和副作用摊进产品航迹

`TrackOutputFrame` 只包含 track 快照（`cycle_index`、`batch_id`、`tracks`）。
其余事实按层挂在周期记录上，而不是继续把 `ArCycleResult` 当成厨房水槽：

| 层 | 字段 | 为何不进产品航迹 |
|---|---|---|
| 副作用 | `emission_frame`、`receiver_impairment`、`interference_observations` | 物理已发生，产品可空 |
| 闭环 | `control_profile`、`applied_decision_*`、指定回退 | 下一拍控制，不是本拍航迹 |
| 仿真附件 | `track_attributions` | 真值对照 |
| 禁止 | ~~`decision_observation`~~ | 已删除（15f）：外部决策读产品层即可 |

`DecisionInputFrame` 仍是内部 `TacticalCoordinator` 的输入（头文件已移
`src/airborne_radar/decision/`），不外发。

[evidence: tests/integration/cross_domain/multi_model_scenario_test.cpp — ArRfTestCycleResult 字段选择]
[evidence: src/airborne_radar/runtime/ArController.cpp]

## STT 指定航迹跟随与自动回退（方案 A，冻结）

STT 模式不再要求外部提供目标角度：外部通过
`ArRuntimeConfigPatch::has_designated_target_id` 只指定目标（`external_target_id`，
`0` = 清除），波束指向由 AR 用自身航迹推导。

1. **指向来源优先级（冻结，不得改序）**：
   1. 显式 `dwell_center_deg` 非零 → 最终指向 = `scan_center + dwell`（现状语义，最高优先）；
   2. `work_mode == kStt` 且指定目标航迹 confirmed → 最终指向 = 指定航迹位置换算的
      az/el（雷达局部系，`TryTrackPositionToLookAnglesDeg` 口径），dwell 视为零偏移；
   3. 其余（未指定/航迹未确认/丢失/非 STT）→ 最终指向 = `scan_center`（现状行为）。
2. **指定状态是会话级状态**：挂在 `RuntimeConfigState::designated_external_target_id`，
   随 patch 原子暂存/提交/回滚；不进 pipeline 执行配置（pipeline 不消费指向来源）。
3. **生效模式派生（latch-free，无跨周期记忆）**：`effective_work_mode` = 已提交 STT 且
   指定航迹 confirmed 时为 `kStt`；指定航迹未确认/丢失时回退 `kTws`；未指定目标的 STT
   保持现状 `scan_center` 驻留语义（仍为 `kStt`，不视为回退）。回退不修改已提交配置。
4. **自动丢跟踪暴露（三层）**：`ArCycleResult`（L2）新增 `effective_work_mode`、
   `designation_active`、`designated_target_id`、`designation_reverted_to_tws`（每周期状态
   指示，非转换沿，跨周期差分由调用方承担）、`designation_revert_reason`；L3
   `ArTrackOutputDebugView` 转写同名字段；`ArTrackLifecycleRecorder` 在回退转换沿为指定
   目标产生 `kDesignationDropped` 事件。replay 周期记录与 patch 记录均保留新字段。
5. **显式 dwell 覆盖不构成回退**：`designation_active == false` 但
   `designation_reverted_to_tws == false` 表示指向被显式覆盖，不是丢跟踪。
6. **扫描动画接线（session 级，修复原已知限制）**：生效模式为 TWS/TAS 且无显式
   dwell 覆盖、无 STT 航迹跟随时，`ArSession` prepare 指向 = 扫描表当前周期波位
   （`ResolveScheduledBeamPointingFromExecutionConfig`，与 pipeline 内
   `ApplyScanScheduleToRuntimeConfig` 同一扫描相位），经
   `RfV2DetectionContext::beam_pointing_deg` 逐周期推进发射 boresight / 接收状态 /
   增益 / 检测单元——"回 TWS"（STT 指定航迹丢失/未确认回退）恢复扫描动画，不再
   回到静态指向。静态语义保留：显式 dwell（优先级 1）钉住 `scan_center + dwell`；
   STT 航迹跟随（优先级 2）跟随航迹；未指定目标的 STT 驻留（生效模式仍为 `kStt`）
   保持 `scan_center`。扫描范围由 `mechanical/electronic_scan_limits_deg` 交集决定，
   `scan_center` 仅作非法限位时的回退中心（patch 移动 scan_center 不移动扫描范围）；
   pipeline 本地 `ApplyScanScheduleToRuntimeConfig` 保留，供 RF v1 回退路径
   （`rf_v2_detection_context == nullptr`）使用。
7. **限时锁定指令（`designation_duration_cycles`）**：指定指令可带捕获窗口
   （周期数；`0` = 无限期，旧行为）。生命周期阶段（`RuntimeConfigState`：
   `kPending` → `kAcquired` | `kExpired`，终态）由 `AdvanceDesignationPhase`
   每周期推进，推进结果仅在本周期成功完成后落定（失败/关机周期不消耗窗口）：
   - 窗口自指令生效后首个成功周期起算（deadline = 首周期 + duration）；窗口内
     每周期等待指定目标 confirmed 航迹，未捕获则继续扫描（回退报告
     `kTrackNotConfirmed`）；
   - 窗口内捕获 → `kAcquired`：跟随航迹且**不再受窗口限制**（后续丢失按既有
     回退语义 `kTrackLost`/`kTrackNotConfirmed`，不重新开窗口）；
   - 窗口耗尽仍未捕获 → `kExpired`：**指令作废**。作废沿为 `kPending` →
     `kExpired` 转移沿（截止周期被拒时在截止后首个成功周期报告），沿周期
     保留目标 ID 并报告 `designation_revert_reason =
     kAcquisitionTimeout`（L2 结果 + L3 视图 + `kDesignationDropped` 事件）；
     其后指定清零（`designated_target_id == 0`）、无回退报告、生效模式按扫描
     处理（已提交 `kStt` 时生效为 `kTws`，回到扫描），直到外部重新下达指定。
   - 捕获判定与指向同源（上一周期航迹帧，滞后一周期）；窗口独立于指向优先级
     运行（显式 dwell 覆盖不暂停窗口）；任一指定相关 patch 变更（含仅改时长）
     视为新指令，窗口重新起算；`kExpired` 不修改已提交配置（作废后生效模式
     持续按扫描派生）。replay 的 patch 记录保留时长字段，作废行为由
     cycle_index 驱动可复现。

[evidence: tests/unit/airborne_radar/ar_stt_track_follow_test.cpp]
[evidence: tests/unit/airborne_radar/ar_track_output_debug_view_test.cpp]
[evidence: tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp]

## 滤波后端选型（人工配置为主，不做在线自动切换）

AR 使用标准 Joseph 形式 Kalman 滤波器（KF）作为生产后端。IMM 生命周期（`enable_imm_lifecycle`）是包裹 KF
的多模型融合层，**不是**独立后端。选型决策人工配置在先，理由三点：

1. **可复现性优先于智能性**。在线自动选型会使同一想定因阈值微调走不同后端，结果不可比。
2. **选型决策依赖外部真知**。"目标是否机动"等判据，仿真期真值已知，泄露到选型逻辑等同作弊。
3. **可解释性**。工程评审需能追溯到具体后端与参数。

明确不做：在线残差驱动的自动后端切换。EKF/SRIF/UDKF 的否决依据见 [algorithms.md](algorithms.md) 滤波后端
评估表；可接受的"智能"形态只有只读 NIS 诊断和基于任务剖面先验的 IMM 配置。

## 传感器实例身份注入边界（SetSensorEntityId）

`ArSession::SetSensorEntityId(std::uint32_t)` 是标注域设备标识的注入面，供会话构造后注入，区分同型多实例：

1. **注入面特征**：构造后注入，不进入 `ArSessionConfig`，不进入 `ArRuntimeConfigPatch`，不参与电源/事务模型与运行期重置。
2. **时序与缺省**：任意时刻可调，下一周期起生效；未注入或显式传入 `0` 时恢复默认哨兵，周期摘要日志回退使用融合常量 `fusion::kArSourceId`（1U）。
3. **消费面与物理隔离**：仅用于周期摘要日志行 `[SignalPipeline]` 携带 `sensor_entity_id={}` 供排查区分；不进入信号与物理计算，不改变 `TrackOutputFrame` 或 `ArCycleResult` 输出通道，与每拍经 `ArPlatformInput::platform_entity_id` 注入的 RF 平台记账号完全解耦。

[evidence: include/1q/airborne_radar/session/ArSession.h]::SetSensorEntityId
[evidence: tests/unit/airborne_radar/ar_rf_session_test.cpp]

## 非目标

1. 不恢复宽 public customization surface。
2. 不把 `EnvironmentService`、`SignalPipeline`、`ArController`、`MutableArContext`、tracking lifecycle 或
   foundation 工程算法暴露为用户可替换 API。
3. 不把单一默认 association 路径包装成 public algorithm family；只有存在多个生产实现时，才通过受控配置
   暴露选择。
4. 不做在线残差驱动的自动滤波后端切换。
5. 不把测试 mock 便利接口升级为 public SPI。
6. 不让外部 decision engine 绕过内部 control reducer 和 command mapper。
7. 不把 debug/lifecycle/replay 字段混入系统输出语义。

上述边界由文档结构守护和 public API contract 测试守护。

[evidence: tests/contract/airborne_radar/ar_public_api_convenience_test.cpp]
[evidence: tests/contract/check_public_api_boundary.cmake]

## 设计变更规则

1. 新增、删除或改变 public SPI 时，必须同步本文档集、consumer tests 和 `ar_public_api_convenience_test`。
2. 任何新增 runtime patch 字段，必须明确是否影响 pipeline config、自然环境 scenario 或 RF operating state，
   并接入提交/回滚流程。
3. 探测路径如改变 RCS、大气、干扰、波束、SNR、检测概率或量测协方差语义，必须同步 algorithms.md 和相关
   signal/detection tests。
4. 数据关联和 lifecycle 行为变化，必须同步 association quality metrics、decision frame 说明和对应测试。
5. 战术决策或控制归约策略变化，必须补充 LPI/ECCM/ControlReducer 测试，并在 `[evidence: ...]` 标注中记录
   决策依据。
6. 输出字段变化必须保持 `TrackOutputFrame`、`ArCycleResult`、debug/lifecycle/replay 三层分离。
7. 验证优先使用 `unit::airborne_radar`、`contract::airborne_radar`、`replay::airborne_radar`、
   `batch_validation::airborne_radar`、`integration::cross_domain` 以及 AR guards。
