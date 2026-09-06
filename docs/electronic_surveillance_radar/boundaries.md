---
Status: active
Last-reviewed: 2026-08-23
Authority: ESR 模块级边界、非目标与设计变更规则
Answers: ESR 有哪些模块级禁令与边界、哪些非目标、配置/扫描/输出/校验的特殊语义、文档变更规则
---

# ESR 模块边界

本文承载 ESR 的模块级边界、非目标、反直觉配置语义和设计变更规则。算法级边界（拦截检测链、聚类、
假设关联）见 [algorithms.md](algorithms.md)。

## 与 common 契约的关系

ESR 遵守 `docs/common/contract.md`：

1. public API 只暴露稳定 session/config/input/output/Recording/Replay 门面；`EsrSession` 是对外门面，
   pipeline/controller/environment service、runtime snapshot 和 generated replay header 不通过 public header 暴露。
2. runtime patch 经 `ApplyRuntimeConfigWithResult()` → `ResolveEsrRuntimeConfigPatch()` 解析：整域值先合并，
   leaf override 后应用，再对最终 mission 枚举、scan policy、基础 detection policy 和 environment 统一做
   一次领域校验。通过校验的 patch 立即写入 `resolved_config` 并同步到 pipeline/environment；被拒绝的 patch
   原子无污染。ESR 属立即提交类，配置单向落定，不提供 session 层回滚。
3. 输出遵守分层周期记录 + 可选投影（规则 15）：`EsrOutputFrame` 发布两个去真值化产品
   （observation_output、emitter_output）与设备状态标量 `scan_azimuth_deg`，
   执行层 `status` 承载本周期执行真相。ESR 为非目标列表型，可不提供 DebugView 同形态投影。

## scan_rate_hz 校验边界（反直觉，勿按"波束更新率"理解）

`scan_rate_hz` 的单位不是波束更新率或角速度，而是**每秒完成的完整二维 scan pattern 循环数**。pipeline
持有归一化扫描相位 `[0, 1)`：本周期先用 `floor(phase × pattern_size)` 选择波束，再累加
`scan_rate_hz × dt` 并回绕。因此变步长不会改变物理扫描速度；运行期只改速率会保留当前相位，改变
窗口边界、顺序或起始位置则重置到起始波束。设备关闭时扫描相位冻结。静态配置和 runtime patch 均拒绝
非有限或非正速率。

该标量 pipeline 每 `Step` 只判定当前相位对应的一个波束，不在单周期内积分连续扫过的全部驻留。
`scan_rate_hz × dt` 为整数时，相位会按物理周期回到同一点；需要观察完整扫描覆盖的场景必须选择能够
解析扫描相位的步长/速率组合，不能依赖 cycle index 隐式轮转波束。

波束序列（`pattern_size` 个波束指向）由扫描窗口按 az/el 步进生成：采样按**整数步数计数**（`start + k×step`），
不使用浮点累加，极小步进不会因增量被舍入吞掉而陷入无限循环；单轴采样点数上限 131072
（`ScanPatternGenerator.h` 的 `kMaxScanPointsPerAxis`），超出时序列截断到前 131072 个点。

[evidence: tests/unit/electronic_surveillance_radar/esr_controller_runtime_state_test]
[evidence: tests/unit/electronic_surveillance_radar/esr_session_config_builder_test]
[evidence: tests/integration/cross_domain/multi_model_scenario_test]

## dt_sec 校验边界（反直觉，勿按"四模块一致"补齐）

`ValidateEsrCycleInput` 对 `dt_sec` 仅校验有限性 + 正值，**故意不含** EOS/SBIRS 的
`dt_sec ≤ 10/frame_rate_hz` 上界。ESR 是被动侦察接收机，配置中没有 `frame_rate_hz` 概念——其节拍由
`scan_rate_hz × dt` 决定（见上文扫描相位模型），变步长不改变物理扫描速度。dt 的合理性由
`scan_rate_hz × dt` 能否解析扫描相位、RF emission 帧窗口一致性（`window_duration_s == dt_sec`）等
ESR 域量约束，不适用一个全局 frame_rate 上界。该差异已由校验链实测、`EsrCycleInput` 无 frame_rate
字段、本节扫描相位文档三方共同固化。

不得为"四模块一致"给 ESR 加 frame_rate 上界。

[evidence: include/1q/electronic_surveillance_radar/session/EsrInputValidation.h]

## 扫描窗口与坐标系语义（反直觉）

### 扫描窗口两种互斥解释

- `use_explicit_scan_bounds=true` 时，四个显式起止角必须全部有限，并分别满足
  `scan_start_az_deg < scan_end_az_deg`、`scan_start_el_deg < scan_end_el_deg`；显式边界生效，中心角字段被忽略。
- `use_explicit_scan_bounds=false` 时，中心角结合硬件扫描范围推导窗口；即使显式字段为 NaN/Inf，也因未被选择而忽略。

静态 session validation 按所选模式验证，不能把非法的显式模式静默退化成中心模式。运行期 resolver 先合并
full-domain mission，再应用 leaf override，最后只对合并后的 scan policy 做一次统一校验和解析；因此
full-domain 中的非法中间值可被合法 leaf 覆盖，但任何留在最终策略中的非法值都会原子拒绝整份 patch。
提交中心角时关闭显式模式，提交显式起止角时开启显式模式；显式提交 `enabled=false` 时忽略该 inactive
payload 中的四个边界字段，并按中心角、硬件扫描范围和天线安装角重建窗口；将被启用的中心角若非有限
则原子拒绝，不能保留旧显式执行态窗口。runtime 开启显式模式与静态 validation 一致，严格要求两轴
`start < end`，不接受 equal/swapped 输入。因此最近一次被明确选择的合法表达拥有窗口语义。

[evidence: tests/unit/electronic_surveillance_radar/esr_session_config_builder_test]
[evidence: tests/unit/electronic_surveillance_radar/esr_runtime_config_resolver_test]

### 扫描配置的坐标系语义（天线坐标系）

`EsrScanPolicyConfig`（mission 域）中的 `scan_center_az_deg` / `scan_center_el_deg`，以及显式模式下的
`scan_start_az_deg` / `scan_end_az_deg` / `scan_start_el_deg` / `scan_end_el_deg`，均定义在**天线坐标系**中，
即默认假定天线 boresight 与接收机参考轴对齐。`EsrOrientationConfig::antenna_mount_az_deg` /
`antenna_mount_el_deg`（会话 `orientation` 域）描述天线相对平台参考轴的安装偏置；`ApplyScanPolicy` 在解算实际接收
波束指向时，会从 mission 域扫描角中**减去** orientation 域安装偏置。因此 mission 配置给出的扫描角与最终
平台系指向之间相差一个 orientation mount 偏移：消费方只看 mission 配置无法推断实际平台系扫描方向。这是
当前固化语义，不接受在 mission 域直接填写平台系角度。

前端 boresight 的物理实现（2026-08-21 公共域收敛）：`EsrRfV2FrontEnd` 经 `EsrBoresightChain`
（`src/common/geometry/BoresightChain` 的 ESR 薄适配）把天线系波束指向按"平台姿态（Body->ENU）∘
天线安装偏置"做**旋转复合**后旋入 ENU，再经 geodetic 步骤转 ECEF；不再使用历史的"波束角 + 安装偏置"
角度加法。正安装偏置仍表示光轴偏向机体系正方位/正仰角（语义与 AR 前向链一致；公共链 mount 参数为
Body->Sensor 坐标旋转，方向相反，取反入链，模块侧不可见）。零姿态 + 单轴安装偏置下与历史加法严格
一致；非零姿态 + 非零安装偏置下几何由近似升级为严格旋转合成。波束角度合法域校验也随之回到天线系
波束角本身（历史校验的是加法后的体系角）。`ApplyScanPolicy` 的减法语义与 `scan_azimuth_deg` 输出
算式不变。

[evidence: tests/unit/electronic_surveillance_radar/esr_boresight_chain_test]
[evidence: tests/unit/electronic_surveillance_radar/esr_rf_v2_front_end_test]

## 电源状态单源（COMMON-OQ-4 字段提升）

`EsrSessionConfig::sensor_enabled` 是电源唯一来源（mission 域无电源字段），`has_sensor_enabled` 叶子是
运行时电源唯一入口。运行期关闭传感器不重建会话，只通过 patch 立即生效。

[evidence: tests/integration/electronic_surveillance_radar/esr_session_test]

## 输出与可观测性边界（模块级）

`EsrController` 是输出帧装配和最近有效帧缓存的唯一 runtime owner；它直接写入 cycle/batch header，
移动 pipeline 的观测/假设结果，并只在成功执行后推进 batch。模块不维护第二个 output-manager 状态或
装配路径。

`batch_id` 在 public `EsrOutputFrame`、controller 累积状态、FlatBuffers replay schema、codec 和 comparator
中统一为 64 位无符号值；codec 不得把它缩窄到 32 位，大于 `UINT32_MAX` 的值必须无损 roundtrip。

`scan_azimuth_deg` 是输出帧的设备状态标量：pipeline 在检测阶段按当前扫描相位选中波束后，以
"波束中心方位 + 天线安装偏置"（即 mission 域扫描角所在参考系的实际指向，与 RF 前端接收求解同算式）
写入检测输出，经 `InterceptPipelineResult` 由 controller 装配进 `EsrOutputFrame`。该字段已纳入 FlatBuffers
replay schema、codec 与 comparator；非执行周期（校验失败/关机）随默认空帧输出 0，消费方须以
`status == kCompleted` 守卫读取，不能把 0 当成真实方位。

truth identity、外部坐标适配输出与 debug view 不属于 ESR 公共输出合同；消费者只使用观测和 hypothesis
的估计字段。不通过日志文本判断状态；调用方应使用 `EsrCycleResult`。

### 非执行周期统一不复用（五模块统一规则）

ESR 非执行周期（校验失败/设备关机）的 `Step()` 与 `EsrCycleResult.output_frame` 一律返回**默认空帧**
（`cycle_index=0`、`scan_azimuth_deg=0`、空 observation/emitter 输出），**永不复用**上一有效输出。
调用方仅凭 `Step()` 返回值即可判定本轮无新观测；执行真相（rejected vs powered-off）须走
`StepWithResult().status` / `abort_reason`。

注意 controller 内部在 validation reject 时确实会保留 `GetLatestInterceptOutputFrame()`（旧帧），
但 `EsrController::BuildCycleResult`（装配在 RunOnce 内完成并缓存，COMMON-OQ-9 收敛，issues
直通无校验缓存）只在 `status == kCompleted` 时把它写入 public `EsrCycleResult.output_frame`，
因此 controller 的旧帧缓存是内部状态机行为，不构成 public `Step()` 的输出回退路径。

[evidence: tests/contract/electronic_surveillance_radar/esr_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailure]
[evidence: tests/unit/electronic_surveillance_radar/esr_controller_runtime_state_test]

### 三写约束（abort_reason + issues + 日志）

ESR 所有中止路径遵守 `session_contract.md` 规则 9 的三写模式与规则 14 的统一问题列表模型：

1. **结构化信号**：`EsrCycleResult.abort_reason`（粗粒度枚举）。
2. **结构化诊断**：`EsrCycleResult.issues`（`EsrIssueList`，细粒度 code 如 `"esr.rf_receiver_rejected"`、
   `"esr.validation.invalid_cycle_delta_time"`；条目携带 `phase` 来源标签与可选定位）。
   本模块 code 全集单一事实来源：`include/1q/electronic_surveillance_radar/session/EsrIssueCodes.h`（规则 14c）。
3. **人读日志**：`PROJECT_LOG_ERROR`。

`EsrCycleResult` 只承载单一问题列表 `issues`：输入校验问题（`phase=kInputValidation`）与执行诊断
（`phase=kExecution`/`kOutputContract`）同列表承载，不设 `validation_issues`/`has_validation_error`
平行字段。校验拒绝时校验问题本身就是 error 级诊断（规则 9 写二），不再附加粗粒度条目。

三写由 `EsrDiagnosticUtils::RecordAbort` 统一执行（phase 由中止原因推导），在
`EsrController::AssembleResult`（RunOnce 内装配路径）中调用。周期结果装配在 RunOnce 内
完成并缓存（COMMON-OQ-9：issues 直通），`BuildCycleResult` 仅返回缓存；校验缓存字段与
`GetLastValidationIssues` 查询 API 已删除。

**正常周期的按发射源排除诊断（规则 13b）**：正常执行周期（`status == kCompleted`）中被门控排除的
发射源（同址干扰 / 零功率 / SNR-统计检测门）写 `kInfo` 级 `EsrIssue`（code 如
`"esr.emission_below_threshold"`，message 携带发射源标识 platform/equipment/emission id 与关键量值），
**不属于三写**（三写仅约束中止路径，规则 9）。ESR 无目标概念（按发射源处理，无 target_id），
排除诊断以发射源标识为载体（对应契约规则 13b 措辞）。
**门内归因（规则 13b 归因条款）**：`EsrIssueCause` 给出机器可读主因——零功率排除按成因细分
（时频重叠窗口为零 `kOverlapWindow` / 发射静默 `kTransmitSilent` / 传播损耗 `kPropagationLoss`），
低于门限按门型细分（硬门 `kHardGateFailed` / 统计门 `kStatisticalGateFailed`）；同址排除为
具体门（cause 恒 `kNone`），message 补隔离度与路径量值。诊断不改变 `EsrCycleExecutionStatus` 与
输出帧语义（规则 13c）；周期摘要日志（`[InterceptPipeline] … excluded=…`）仅人读（规则 13a）。
**实体机器可读关联（规则 14e/13b）**：排除诊断结构化携带 `location = {kSceneEntity, emission_index}`
（`emission_index` = 发射源在 identity 排序后数组中的下标，与 `InterceptDetectionExecutor` 排序序
一致；三发射点分别由循环 A 的 `index` / 循环 B 的 `signal_index` 赋值）。
**排除原因跨周期差分（规则 13e）**：`EsrExclusionCauseRecorder` 对持续被排除发射源做
`(code, cause)` 对差分，产出 A2/A3/A4 事件。**实体键为发射源标识三元组**（platform/equipment/
emission id，非 entity_index 下标）：记录器 Update 时按同一 identity 排序序重排
`input.rf_emissions.emissions` 把 entity_index 解析回 identity 三元组，内部状态以 identity 为键
——免疫跨周期发射源集合变化时的下标移位（源消失后其余源下标变化不会误判为原因变化）。纯观测
只读 `result.issues`（仅消费 `phase == kExecution` 且 `location.kind == kSceneEntity` 的排除诊断
条目；同样用 `kSceneEntity` 定位的输入校验 issue 属 `kInputValidation` 阶段，不被记录器消费），
不改变执行语义（规则 11c/13c）。ESR 当前仅本排除原因差分记录器（无既有
生命周期 recorder），`EsrSession::AttachExclusionCauseRecorder` 为首个 recorder 注册点（注册后
`Step()`/`StepWithResult()` 在周期结果装配完成后自动驱动 `Update`）。

## 专项序列验证边界

`batch_validation::electronic_surveillance_radar` 覆盖近同频辐射源角度交叉、密集辐射源静默、
ESM/RWR/HGESM 切换、显式扫描边界重定向、关机恢复和无效输入恢复。所有场景的 trace replay 失败、输出
分叉或比较数量不一致都会使批量验证失败。

影响退出码的硬检查：各场景预期的非执行周期数；无效输入场景的 failure marker 数；无效显式边界 patch
的原子拒绝；角度交叉场景的建立/最终 hypothesis id 集合连续性；关机恢复和无效输入恢复场景在第一个
重新执行周期立即保持建立阶段 hypothesis id 集合。

batch 不含 truth matching、legacy lifecycle recorder 或旧输入适配器；每个场景显式设置与载频匹配的窄带
tuning window，因此 sweep 与 sequence 都必须产生真实观测——零观测不再被当作可接受的空验证。场景 ID、
结构化 check 和运行方式由 `tests/consumer/batch_validation/README.md` 维护。

性能验收分为两个不可互相替代的 Release 场景：稀疏检测场景以 64 个外部 RF 发射、1000 个 AR 目标和
1000 个 ESR 发射验证 RF 前端/分辨账本；密集检测场景要求每周期 1000 条 raw observation，并至少保留
90% 的预处理观测、cluster 和 hypothesis，同时限制活跃 hypothesis 不超过输入规模的 5 倍。两者都在
20 个 lifecycle 预热周期后连续测量 100 周期并要求 P95 `< 100 ms`，拒绝周期不能计入性能样本。密集场景
还逐周期采样活跃堆分配量，以最初和最后 20 个测量周期的中位数比较稳态增长，允许上限为 4 MiB；不得用
进程历史峰值 RSS 或 hypothesis 数量替代活跃模型状态的内存验收。

[evidence: tests/performance/cross_domain/rf_interference_performance_test]

## 传感器实例身份注入边界（SetSensorEntityId）

`EsrSession::SetSensorEntityId(std::uint32_t)` 是标注域设备标识的注入面，供会话构造后注入，区分同型多实例：

1. **注入面特征**：构造后注入，不进入 `EsrSessionConfig`，不进入 `EsrRuntimeConfigPatch`，不参与电源/事务模型。
2. **时序与缺省**：任意时刻可调，下一周期起生效；未注入或显式传入 `0` 时恢复默认哨兵，周期摘要日志回退使用融合常量 `fusion::kEsrSourceId`（2U）。
3. **消费面与物理隔离**：仅用于周期摘要日志行 `[InterceptPipeline]` 携带 `sensor_entity_id={}` 供排查区分；不进入物理计算与截获检测逻辑，不改变 `EsrOutputFrame` 或 `EsrCycleResult` 结构。

[evidence: include/1q/electronic_surveillance_radar/session/EsrSession.h]::SetSensorEntityId
[evidence: tests/contract/electronic_surveillance_radar/esr_public_api_convenience_test.cpp]

## 非目标

1. 不暴露用户自定义 pipeline/controller/environment service。
2. 不把 truth identity 或预计算受扰结论合并进真实 observation/hypothesis 输出。
3. 不把 pipeline internal context、runtime snapshot 或 generated replay header 当成 public API。
4. 不通过日志文本判断状态；调用方应使用 `EsrCycleResult`。
5. 不以 `legacy / engineering` 标签改变接收物理链；旧 public jammer 摘要和 adapter 已删除，不属于公共合同。

上述边界由文档结构守护和 public API 契约测试守护。

[evidence: tests/contract/check_public_api_boundary]

## 设计变更规则

1. Observation/hypothesis 或 cycle-input 位姿字段变化必须同步精确 replay roundtrip 测试。
2. Gate、preprocess、cluster、association 语义变化必须同步 algorithms.md 和对应 focused tests。
3. Runtime patch、snapshot 或状态边界变化必须同步控制器状态测试与 data-flow.md 状态所有权。
4. 输出通道边界变化必须同步 output boundary contract 测试。
5. 验证优先使用 `unit::electronic_surveillance_radar`、`integration::electronic_surveillance_radar`、
   `replay::electronic_surveillance_radar`、`batch_validation::electronic_surveillance_radar` 与 ESR guards。
