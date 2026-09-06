---
Status: active
Last-reviewed: 2026-08-23
Authority: EOS 模块级边界、非目标与设计变更规则
Answers: EOS 有哪些模块级禁令与边界、哪些非目标、frame_rate/dt 耦合与帧级 config 的特殊语义、文档变更规则
---

# EOS 模块边界

本文承载 EOS 的模块级边界、非目标、反直觉配置语义和设计变更规则。算法级边界（辐射传输、噪声·NEP、
空间频谱、杂散光等 foundation 算法）见 [algorithms.md](algorithms.md)。

## 与 common 契约的关系

EOS 遵守 `docs/common/contract.md`：

1. public API 只暴露稳定 session/config/input/output/Recording/Replay 门面。`EosSession` 是对外门面，
   只委托内部 `EosController`；Controller、Pipeline、CompositionRoot、foundation 算法不通过 public
   header 暴露。
2. 会话配置直接赋值 `EosSessionConfig`；语义档位是
   `EosProfileConstants.h` 中的预定义结构体常量（如 `profiles::kLongRangeSurveillanceMission` +
   `profiles::kLongRangeSurveillanceDetection`）。旧"Mission
   Profile 跨域覆写 `policy.detection.minimum_snr_db`"语义已消除：配置不再有隐式优先级，任何字段的赋值即
   最终决定（档位在前、微调在后时微调胜出）。运行期热更新直接写 `EosRuntimeConfigPatch`（显式 `has_*`）；
   不提供 ConfigBuilder。
3. EOS 输出遵守分层周期记录 + 可选投影（规则 15；旧称两通道/三层）。落地前仍为扁平 `EosCycleResult`。
4. `EosSession::StepWithResult` 在执行 pipeline 前调用 `ValidateEosCycleInput`；存在 error 级问题时
   不执行 pipeline、返回默认空帧并记录校验失败状态（符合 contract.md §实现安全与失败语义规则 3）。
5. EOS runtime config 属于先完整校验、后一次提交的原子语义；`EosRuntimeConfigResolver` 解析 patch，
   失败时不替换当前配置。

## dt_sec 校验边界（反直觉，勿按"五模块一致"补齐）

`ValidateEosCycleInput` 对 `dt_sec` 的校验链为：有限性 → 正值 → 上界 `dt_sec ≤ 10 / frame_rate_hz`
（默认 30 Hz → 上限 ≈ 0.333 s）。`frame_rate_hz` 从 pipeline 当前配置动态读取，runtime patch 热更新
帧率后校验阈值自动跟随。

该上界**故意仅适用于 EOS 与 SBIRS**——两者都是有 `frame_rate_hz` 概念的成像/凝视传感器。SAR/ESR/AR
**故意不含**此上界，因其配置无 `frame_rate` 字段、节拍由各自域量（孔径几何 / scan_rate×dt / PRF）
决定。不得为"五模块一致"给那些模块强加 frame_rate 上界。

[evidence: tests/unit/electro_optical_sensor/eos_input_validation_test]

## 帧级上下文 config 语义（反直觉）

`FrameContext` 是 EOS 帧级物理状态（光学孔径/FOV、波段、NEP/噪声、环境模型、工作模式、探测范围）
的权威容器，但它的几个语义容易误读：

1. **FOV 是记录成员门，不是 SNR 前置过滤器**：视场外目标不生成 detection/attribution（写 `kInfo`
   排除诊断 `eos.target_out_of_fov`，规则 13b）；视场内但超出 `dmin_m/dmax_m` 的目标仍计算通道
   SNR、保留记录，最终 `detected=false`。范围只参与最终检测资格——视场内未过门目标产出
   `detected=false` 记录，**不属于** 13b"被排除"（有记录、有 attribution）。
2. **扫描相位是否重置由 resolver 显式给出**：runtime patch 中的扫描速率或工作模式变化经 resolver
   校验后更新内部配置并决定是否重置扫描相位，不能由调用方隐式假设。
3. **环境 preset 不是 flat 参数**：`EosEnvironmentScenarioConfig` 只含一个 preset 和标准
   `atmospheric_physics` 观测；preset → 物理参数（辐射算法、气溶胶/湍流因子）的映射是设计内容，
   调用方不选择具体辐射算法，也不填写 custom 因子。replay 的 session-config payload 只记录
   `preset + atmospheric_physics`，内部派生数值不进入 schema，也不形成第二套可配置状态。
4. **硬件配置只保留当前生产链实际消费的字段**：波段、孔径、探测器和俯仰边界为 public/session/replay
   配置；焦距不再是 public/session/replay 配置。

[evidence: tests/unit/electro_optical_sensor/eos_pipeline_test]
[evidence: tests/unit/electro_optical_sensor/eos_runtime_config_resolver_test]
[evidence: tests/unit/electro_optical_sensor/eos_environment_model_test]

## 输入校验、失败输出与运行期状态

无效输入不会直接污染 pipeline 状态：

1. 首个周期输入无效时，返回默认空帧（`cycle_index=0`、空检测）。
2. 已有成功周期后再遇到无效输入，返回默认空帧（不复用），同时在 result 中记录校验失败状态。
3. 设备关机返回 `kSensorPoweredOff` 合法非执行状态；返回默认空帧，不得把关机映射为 `kOutputContractViolation`。
4. runtime patch 必须原子校验；任一字段无效时整个 patch 被拒绝。
5. **电源状态单源（COMMON-OQ-4 字段提升）**：`EosSessionConfig::sensor_enabled` 是唯一来源，整块
   mission patch 只更新扫描任务，不触碰电源；`has_sensor_enabled` 叶子是运行期电源唯一入口。
6. controller runtime state 支持 capture/restore，但必须拒绝不兼容的 pipeline snapshot 或其他
   controller 实例的 snapshot。

周期输入校验问题编码统一为 `"eos.validation.<snake_case>"` 字符串；本模块 code 全集
单一事实来源为 `include/1q/electro_optical_sensor/session/EosIssueCodes.h`（规则 14c，库内
调用点与集成方均引用其常量；`eos_input_validation_test` 等测试对 code 值的断言仍锁定既有
语义）。`ValidationCode` 枚举已随规则 14 对齐删除。
环境观测字段（太阳辐照度、云量、风速、背景温度、太阳角、昼夜类型）已迁入
`config::EosEnvironmentScenarioConfig`，不再属于周期输入域，故不声明对应校验编码，以免误导
调用方以为可以在 `CycleInput` 上校验这些字段。

`target.range_m` 的权威校验在 `EosInputValidation`（`<= 0` 为 error），controller 在校验失败时不执行
pipeline，故正常 Session 路径不会把非法 `range_m` 传入 pipeline。pipeline 内部的
`SafePositive(target.range_m, 1000.0f)` 仅为深度防御，兜底值 1000m 不构成合法输入约定。

**场景目标输入为平台锚点 radar-local ENU**（契约见 docs/common/contract.md「场景目标平台锚点
ENU 输入契约」）：`EosSceneTarget::position_x/y/z` 为锚点 ENU 位置（x=东/y=北/z=天，原点=当周期
平台 ECEF 位置）；体系球坐标（斜距/方位/仰角）是库内量测几何，由控制器经
`foundation::TryResolveEosLookAngles` 从 ENU 位置 + `platform_attitude_deg`（Body->ENU）派生后供
pipeline 消费（`EosPipelineSceneTarget`），不进入公开输入契约。旋转与取角委托公共域
`common/geometry/BoresightChain`（2026-08-21 收敛；EOS 无安装角/失准配置，链路仅含姿态，与 SBIRS/
ESR 同引擎），退化判定（模长下限）与斜距输出留在 `EosLookAngles` 模块层。斜距权威校验等价于 ENU
位置模长非退化（`EosLookAngleNormFloorM` 下限）。速度字段保留 ENU 契约统一形状，当前仅校验
有限性，不参与探测计算。

**集成入口**：调用方以公共 `TryEcefToLla` + `TryMakeEnuSceneState` 直填 `EosSceneTarget`，
再手填 `EosCycleInput`（海拔/姿态/`dt_sec`/`cycle_index`）。已删除模块级
`EosCycleInputAdapter` / `TryMakeEosSceneTargetFromExternalInput` /
`EosExternalTargetInput` 平行入口。平台 ECEF 位姿类型为 `EosPlatformEcefPose`
（供输出反算等使用，不是目标 ENU 适配器）。

[evidence: tests/unit/electro_optical_sensor/eos_look_angles_test]
[evidence: tests/unit/electro_optical_sensor/eos_input_validation_test]
[evidence: tests/unit/electro_optical_sensor/eos_controller_runtime_state_test]
[evidence: tests/contract/electro_optical_sensor/eos_public_api_convenience_test]
[evidence: tests/replay/electro_optical_sensor/eos_replay_session_test]

### 非执行周期统一不复用（五模块统一规则）

EOS 非执行周期（校验失败/关机/执行 abort）的产品层一律为空载荷，**永不复用**上一有效输出
（规则 15d）。调用方用执行层 `status` / `abort_reason` 判断周期状态。落地前空帧
`cycle_index` 仍可能为 0。`reused_previous_output` 字段已删除。

[evidence: tests/contract/electro_optical_sensor/eos_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailureAfterSuccess]

### 三写约束（abort_reason + issues + 日志）

EOS 所有中止路径遵守 `session_contract.md` 规则 9 的三写模式与规则 14 的统一问题列表模型：

1. **结构化信号**：`EosCycleResult.abort_reason`（粗粒度枚举）。
2. **结构化诊断**：`EosCycleResult.issues`（`EosIssueList`，细粒度 code 如 `"eos.sensor_powered_off"`、
   `"eos.validation.invalid_target_range"`；条目携带 `phase` 来源标签与可选定位）。
3. **人读日志**：`PROJECT_LOG_ERROR`。

`EosCycleResult` 只承载单一问题列表 `issues`：输入校验问题（`phase=kInputValidation`）与执行诊断
（`phase=kExecution`/`kOutputContract`）同列表承载，不设 `validation_issues`/`has_validation_error`
平行字段。校验拒绝时校验问题本身就是 error 级诊断（规则 9 写二），不再附加粗粒度条目。

三写由 `EosDiagnosticUtils::RecordAbort` 统一执行（phase 由中止原因推导），在
`EosController::AssembleResult`（RunOnce 内装配路径）中调用。周期结果装配在 RunOnce 内
完成并缓存（COMMON-OQ-9：issues 直通），`BuildCycleResult` 仅返回缓存；校验缓存字段与
`GetLastValidationIssues` 查询 API 已删除。

**正常周期的按目标排除诊断（规则 13b）**：正常执行周期（`status == kCompleted`）中视场外目标
（无 detection/attribution）写 `kInfo` 级 `EosIssue`（code `"eos.target_out_of_fov"`，
message 携带 `target_id` 与目标方位/扫描中心/FOV 尺寸/差值），**不属于三写**（三写仅约束中止路径，
规则 9）；不改变 `EosCycleStatus` 与 DebugView 状态语义（视场外目标仍为 `kNotInOutput`，规则 13c）。
**门内归因（规则 13b 归因条款）**：视场门按越界轴细分——`EosIssueCause::kAzOutside` /
`kElOutside` / `kBothAxesOutside`（与 `IsTargetInCurrentFov` 同基准：半视场为门限），
message 补相对扫描中心的差值。
输入中消失的目标由生命周期 recorder 承载（`kLost` 事件），不产生排除诊断（规则 13d）。
周期摘要日志（`[EosPipeline] … excluded={{fov=…}}`）仅人读（规则 13a）。
**实体机器可读关联（规则 14e/13b）**：排除诊断结构化携带 `location = {kSceneEntity, target_index}`
（`MakeExclusionIssue` 由 pipeline 主循环索引 `i` 赋值），供 `EosExclusionCauseRecorder` 按实体
关联消费。
**排除原因跨周期差分（规则 13e）**：`EosExclusionCauseRecorder` 对持续被排除目标做
`(code, cause)` 对差分，产出 A2 进入/A3 原因变化（越界轴变化）/A4 退出事件；纯观测只读
`result.issues`（仅消费 `phase == kExecution` 且 `location.kind == kSceneEntity` 的排除诊断条目；
同样用 `kSceneEntity` 定位的输入校验 issue 属 `kInputValidation` 阶段，不被记录器消费），与
`EosDetectionLifecycleRecorder` 并列（独立 Attach/驱动/GetLastEvents），注册与否不影响执行语义
（规则 11c）。**消失目标边界**：
recorder 只遍历当前周期 `input.scene`，目标从输入消失时其排除状态条目保留（不会被 A4 清除，
与既有 `EosDetectionLifecycleRecorder` 行为一致）；重现为 A3 而非 A2。

## 专项序列验证边界

`batch_validation::electro_optical_sensor` 覆盖双目标焦面交叉、昼/黄昏/夜间、融合/红外/可见光
通道切换、扫描速率重定向、关机恢复和无效输入恢复。EOS 不因此声明跨周期目标跟踪身份。

影响退出码的硬检查只有：replay 完成、预期非执行周期数、failure marker 数、逐周期 attribution
完整、帧内 detection ID 唯一。

属于 warning/error 观测项（不影响退出码）：FOV/lifecycle 行为、非法 runtime patch 原子性、真正的
扫描扇区边界热更、通道 SNR 的昼夜物理趋势。batch 没有直接读取 lifecycle recorder，因此不得把
场景名扩大为这些内部状态的硬契约。场景 ID 与运行方式由 `tests/consumer/batch_validation/README.md` 维护。

## 传感器实例身份注入边界（SetSensorEntityId）

`EosSession::SetSensorEntityId(std::uint32_t)` 是标注域设备标识的注入面，供会话构造后注入，区分同型多实例：

1. **注入面特征**：构造后注入，不进入 `EosSessionConfig`，不进入 `EosRuntimeConfigPatch`，不参与电源/事务模型。
2. **时序与缺省**：任意时刻可调，下一周期起生效；未注入或显式传入 `0` 时恢复默认哨兵，周期摘要日志回退使用融合常量 `fusion::kEosSourceId`（3U）。
3. **消费面与物理隔离**：仅用于周期摘要日志行 `[EosPipeline]` 携带 `sensor_entity_id={}` 供排查区分；不进入辐射传输与检测物理计算，不改变 `EosOutputFrame` 或 `EosCycleResult` 结构。

[evidence: include/1q/electro_optical_sensor/session/EosSession.h]::SetSensorEntityId
[evidence: tests/unit/electro_optical_sensor/eos_session_create_test.cpp]

## 非目标

1. 不暴露用户自定义 pipeline、controller、环境模型或 foundation algorithm 类型。
2. 不把仿真目标 ID/name 混入 `EosOutputFrame` 的 raw detection。
3. 不把 debug view、lifecycle 或 replay 当作真实传感器输出。
4. 不把环境 preset 简化为无语义 flat 参数；preset 到物理参数的映射是设计内容。
5. 不为测试 mock 便利新增 public 扩展点。
6. 不恢复旧"Mission Profile 跨域覆写 policy"的隐式优先级语义。

上述边界由文档结构守护和 public API 契约测试守护。

[evidence: tests/contract/check_docs_structure]
[evidence: tests/contract/electro_optical_sensor/eos_public_api_convenience_test]

## 设计变更规则

1. `EosOutputFrame`、`EosDetectionRecord`、`EosCycleResult` 或 attribution/debug/lifecycle 语义变化，
   必须同步本文档集和输出边界测试。
2. 环境 preset、radiative transfer model、aerosol/turbulence 默认值变化，必须同步 algorithms.md 和
   `eos_environment_model_test`、`eos_pipeline_test`。
3. runtime patch 的可变字段、原子性或 scan reset 规则变化，必须同步本文档集和 runtime resolver 测试。
4. foundation 算法如果从 internal 变成 public API，必须在 algorithms.md 的 `[evidence: ...]` 标注中
   记录扩展理由和兼容策略。
5. 新增 debug/replay 字段时，必须保持真实输出、结构化结果和仿真辅助视图三层分离。
6. 验证优先使用 `unit::electro_optical_sensor`、`contract::electro_optical_sensor`、
   `replay::electro_optical_sensor`、`batch_validation::electro_optical_sensor` 以及 EOS guards。
