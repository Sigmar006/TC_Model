---
Status: frozen
Date: 2026-09-07
Review-Baseline: `evidence/sbirs-measurement-timestamp-and-phase` @ `68295515`
Authority: 过程脚手架记录（非耐久）；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准；
  权威回写完成、合并进 main 前移除本文档。
---

# sbirs-measurement-timestamp-and-phase：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

- **触发来源**：开放议题 SBIRS-OQ-6（量测时间戳与扫描初始相位缺失）与 FUSION-OQ-1（亚周期时间对齐缺失）长期作为技术债务共存；当前三星场景已正式运行，粗颗粒度周期号驱动在多星异步与高速目标场景下精度瓶颈显现。
- **证据**：[evidence: docs/common/open_questions.md]

各待裁定项四问分析（冻结什么 / 证明证据 / 否定证据 / 最小范围）：

1. **待裁定项 1（检出时间戳字段需求）**：
   - 冻结什么：冻结“SBIRS 检出记录必须包含绝对测量时刻字段”的需求，废除仅由周期号推定时序的粗暴近似。
   - 证明证据：高速机动目标在整周期（如 1~5 秒）内空间位移显著，波束穿越目标的实际时刻与周期采样时刻存在秒级偏差，导致下游交叉定位失真；且扫描方位与目标视线几何允许闭式解析求出波束穿越时刻。
   - 否定证据：所有下游消费方（融合/评估）均证明整周期采样误差在容忍度内，或无法在源头解析出亚周期时刻。
   - 最小改动：在 `SbirsOutputTypes.h` 的 `SbirsDetectionRecord` 中增加 `double measurement_time_s`，并在 `SbirsPipeline.cpp` 中根据扫描几何解析计算。
2. **待裁定项 2（多星异步初始扫描相位配置需求）**：
   - 冻结什么：冻结“SBIRS 任务配置必须支持设置初始扫描相位”的需求，废除所有卫星开机一律从 0° 起扫的同步假象。
   - 证明证据：真实星座各卫星独立开机且需要错峰交替巡视；目前全仓代码硬编码初始相位为 0，三颗星运动步调完全重合（广播体操效应）。
   - 否定证据：多星协同作战推演明确要求各星必须绝对同相起跑，或初始相位可由现有其他字段无损表达。
   - 最小改动：在 `SbirsMissionConfig.h` 增加 `initial_scan_phase_deg` 并在 `SbirsPipeline.cpp` 构造/重锚时作为初始值。
3. **待裁定项 3（下游融合层 `DetectionRecord` 与适配层时间戳贯通）**：
   - 冻结什么：冻结“融合通用探测记录扩充时间戳字段，并在 SBIRS 适配层完成数据透传”的需求。
   - 证明证据：时间戳必须进入跨模块消费链路，否则传感器源头输出的时间戳成为死数据，无法形成精度评估闭环。
   - 否定证据：跨模块改动破坏现有版本稳定性，或融合算法当前无法承受时间戳字段引入。
   - 最小改动：在 `DetectionRecord.h` 增加可选 `has_timestamp` / `timestamp_s`，并在 `SbirsBearingAdapter.cpp` 中映射赋值。
4. **待裁定项 4（FlatBuffers 回放 Schema 升级与标识符演进至 SBI5）**：
   - 冻结什么：冻结“公共输出结构增加字段必须按契约规则同步升级 FlatBuffers schema 并晋升文件标识符为 SBI5”的工程约束。
   - 证明证据：契约规定破坏性 schema 变更随之中段整体升位，标识符升级防静默误解码，项目未上线不维护向后兼容分支。
   - 否定证据：发现可在现有保留槽位无损填充时间戳且不改变 binary layout 的合法途径。
   - 最小改动：修改 `sbirs_replay.fbs` 增加 `measurement_time_s` 槽位并将标识符升为 `SBI5`，适配对应 Codec。
5. **待裁定项 5（检出记录与外层帧的周期号职责划分）**：
   - 冻结什么：冻结“单条点目标检出记录 `SbirsDetectionRecord` 不设周期号字段，周期号留在外层输出帧 `output_frame.cycle_index`”的设计。
   - 证明证据：单条点目标记录代表一次物理曝光事实，仅需知道绝对物理时刻 `measurement_time_s`；周期号是离散仿真步调（如 1s 步长）调度与批次打包的标识，属于帧级元数据。
   - 否定证据：下游单目标跟踪或状态机明确需要从单条记录直接逆解调用方步调步长。
   - 最小改动：保持 `SbirsDetectionRecord` 纯粹承载物理测量，不添加冗余周期号。
6. **待裁定项 6（底层传感器管线内部目标动力学外推——方案 A 评估）**：
   - 冻结什么：冻结“传感器管线内部是否应该按相对速度将目标位置推演至波束穿越时刻”的设计路线。
   - 证明证据：若外推，必须证明该外推在惯性系下物理无歧义、不与现有误差模型冲突且不引起失锁。
   - 否定证据（已证实反证）：
     1. 地球自转假位移：在 ECI 惯性系下，地面静止目标具有 $\sim 460\text{ m/s}$ 的 ECI 输运速度，盲目按速度外推导致地面固定目标几何角严重失真；
     2. 物理误差模型重复计模：`ApplyAngularErrorModel` 中已包含探测器动态滞后 $\frac{\omega}{2\pi f_{\text{det}}}$，几何外推导致相对运动效应被计算两次；
     3. 高速目标脱靶死锁：高速交汇场景（如 $5\text{ km/s}$）下，外推使目标脱离当前瞬时视场造成虚假失锁。
   - 最小改动：明确否决管线内目标动力学外推；传感器仅如实测量视线角并标定物理时刻，时空对齐严格归属下游融合层（FUSION-OQ-1）。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| 1. 检出记录增加绝对测量时刻字段 | 检出记录缺失时间信息导致多星交会无法对齐；且波束穿越目标时刻可在源头闭式反解 | - **证据**：[evidence: include/1q/sbirs_sensor/session/SbirsOutputTypes.h::SbirsDetectionRecord]<br>- **证据**：[evidence: src/sbirs_sensor/pipeline/SbirsPipeline.cpp] | 执行探针：核验 `SbirsDetectionRecord` 确无时间字段；核验 `SbirsPipeline.cpp` 中扫描中心由 `scan_rate_deg_per_sec` 匀速推进，目标视线方位已知，穿越时间差 $\Delta t = \|az_{\text{tgt}} - az_{\text{prev}}\| / \omega$ 可解析求出 | 证明时间缺失导致下游产生等效空间失配，且穿越时刻可解析求出 | 证明整周期误差无影响，或穿越时刻无法在传感器层确定 | pass |
| 2. 任务配置支持初始扫描相位配置 | 真实星座多星异步开机；当前硬编码初始相位为 0 导致多星扫描步调完全同步，无法模拟错峰巡视 | - **证据**：[evidence: include/1q/sbirs_sensor/config/SbirsMissionConfig.h::SbirsMissionConfig]<br>- **证据**：[evidence: src/sbirs_sensor/pipeline/SbirsPipeline.cpp::ApplyConfig] | 执行探针：检查 `SbirsPipeline.h` 中 `scan_phase_deg_{0.0f}` 硬编码为 0；在三星场景中三颗星在 $t=0$ 均从各自扇区起点同时向右推进，折返点完全一致 | 证明当前配置无法表达异步相位，且新增参数不破坏现有单星回归基线（默认 0 逐位不变） | 证明真实场景要求多星强行同步起跑，或现有字段已能表达 | pass |
| 3. 融合输入 `DetectionRecord` 与适配层透传时间戳 | 传感器输出的时间戳必须能经由通用适配层流入多源融合引擎，形成端到端闭环 | - **证据**：[evidence: include/1q/fusion/DetectionRecord.h::DetectionRecord]<br>- **证据**：[evidence: src/precision_evaluation/SbirsBearingAdapter.cpp] | 执行探针：检查 `AdaptSbirsResultToDetectionRecords`，目前仅提取方位角与原点 LLA，无法向下游传递时间；`DetectionRecord` 亦无时间槽位 | 证明下游需要该字段作为时间对齐输入，且可选字段方案完全向后兼容其他传感器（如 AR/ESR） | 证明融合引擎拒绝接收时间戳，或必须将跨模块改造推迟到独立专项 | narrow<br>（仅先打通结构与透传，融合复杂滤波算法可分期） |
| 4. FlatBuffers 回放 Schema 升级为 SBI5 | 依契约规则，输出表增加核心字段属于破坏性变更，必须升级文件标识符为 SBI5 防静默误读 | - **证据**：[evidence: schemas/replay/sbirs_replay.fbs]<br>- **证据**：[evidence: src/sbirs_sensor/session/SbirsReplayFlatbufferCodec.cpp] | 执行探针：检查 `sbirs_replay.fbs` 历史注释（SBI3/SBI4 升位记录明确声明无兼容分支，破坏性变更整体升位）；运行现有回放测试 100% 通过 | 证明字段变更影响 schema 布局，升位符合既有工程契约 | 证明可以在不改变 schema 标识符的情况下无缝兼容旧录像 | pass |
| 5. 检出记录与输出帧周期号职责分离 | 单条检出记录代表物理测量事实，外层帧承载离散步调调度批次；单条记录无需冗余周期号 | - **证据**：[evidence: include/1q/sbirs_sensor/session/SbirsOutputTypes.h::SbirsDetectionRecord]<br>- **证据**：[evidence: include/1q/sbirs_sensor/session/SbirsOutputTypes.h::SbirsOutputFrame] | 执行探针：检查 `SbirsDetectionRecord` 历史字段（无周期号）；外层 `SbirsOutputFrame` 已有 `cycle_index`；仿真步长 1s 下单条记录携带精确物理时间戳 `measurement_time_s` 即可满足下游消费 | 证明单条记录仅需物理时间戳，外层帧承载离散打包，概念清晰无冗余 | 证明单条记录缺失周期号会导致下游无解 | pass |
| 6. 传感器管线内目标动力学外推（方案 A） | 假设在传感器内将目标位置外推至穿越时刻可提升精度 | - **证据**：[evidence: src/sbirs_sensor/pipeline/SbirsPipeline.cpp]<br>- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test.cpp] | 执行探针：在管线中依相对速度推演目标位置，导致 5 个单测失败；证实 ECI 地转速度污染静态目标、重复叠加探测器动态滞后、高速目标视线脱离波束 | 证明外推在惯性系自洽且不破坏物理模型与单测基线 | 证实引入地转失真、重复计模与虚假脱靶（已证实） | reject |

## §2 判定汇总与待裁定问题

### 1. 判定汇总
1. **项 1（检出时间戳，建议 pass）**：传感器输出层必须提供真实测量物理时刻，消除源头信息丢弃。
2. **项 2（初始扫描相位，建议 pass）**：任务配置提供 `initial_scan_phase_deg`，默认 0.0f 保持历史单星测试逐位回归，同时赋能多星错峰场景。
3. **项 3（融合输入结构与透传，建议 narrow）**：本次范围收窄为“在 `DetectionRecord` 增补可选时间戳，适配层透传”，融合引擎内部复杂的“非等间隔亚周期滤波推演”作为下阶段演进或由评估层先行消费，避免单次爆炸半径过大。
4. **项 4（回放 Schema 升至 SBI5，建议 pass）**：严格遵循既有回放契约，升标识符并同步升级 Codec 与单元测试。
5. **项 5（周期号职责划分，建议 pass）**：`SbirsDetectionRecord` 仅保留绝对物理时刻 `measurement_time_s`，不添加冗余周期号；`cycle_index` 保持在外层帧结构。
6. **项 6（管线内部目标动力学外推，建议 reject）**：坚决否决方案 A（在底层传感器内部篡改目标几何位置）；传感器如实输出瞬时观测视线，时空对齐归属融合层（FUSION-OQ-1）。

### 2. 需用户裁定的核心问题
1. **裁定点 1（时间戳语义与精度）**：
   `measurement_time_s` 建议采用**绝对仿真时间（double，单位秒）**，即 $t_{\text{cycle\_start}} + \Delta t_{\text{crossing}}$。是否批准此口径？（替代方案为相对本周期的偏移 `float time_offset_s`）。
2. **裁定点 2（2-D 俯仰栅格模式的初始行配置）**：
   在多行牛耕式扫描模式下，除了水平初始相位 `initial_scan_phase_deg`，是否需要同步开放垂直初始行号 `initial_scan_row_index`？（建议：先做 `initial_scan_phase_deg`，栅格行若暂无强诉求可默认从第 0 行起，避免过度设计）。
3. **裁定点 3（融合层实施深度）**：
   是否同意将融合层范围收窄为“`DetectionRecord` 增加字段 + 适配层透传 + 评估层消费”，融合引擎内部的连续时间卡尔曼滤波算法是否在本次一并改造，还是留待独立阶段实施？
4. **裁定点 4（时空自洽与管线纯粹性）**：
   是否同意否决“底层传感器管线内部目标动力学外推（方案 A）”，维持传感器“如实测量视线角并精准标定时间戳”的纯粹性，将多星异步观测间的时空补偿明确划归下游融合层状态滤波职责？

## §3 冻结契约（用户讨论结束后填写）

### 1. 已证明的需求
1. **需求 1（检出时间戳）**：SBIRS 检出输出结构 `SbirsDetectionRecord` 增加绝对测量时刻 `double measurement_time_s`（单位秒，绝对仿真时刻），在 `SbirsPipeline.cpp` 中根据扫描角速度解析计算波束中心穿越目标的精确时刻（NFOV 稳定帧采用当帧权威仿真时间）。
2. **需求 2（初始扫描相位）**：SBIRS 任务配置 `SbirsMissionConfig` 增加 `float initial_scan_phase_deg{0.0f}`（[0, scan_span) deg），在 `SbirsPipeline` 初始化与重锚时作为初始扫描相位生效，默认 0.0f 保持历史单星单测逐位回归，赋能多星错峰场景。
3. **需求 3（融合输入与适配层透传，收窄）**：通用探测记录 `DetectionRecord` 增加可选时间戳字段（`bool has_timestamp{false}`, `double timestamp_s{0.0}`），并在 `SbirsBearingAdapter.cpp` 中完成数据透传赋值，由评估层及地面站消费；`FusionEngine` 内部非等间隔亚周期滤波推演独立作为下阶段专项演进。
4. **需求 4（FlatBuffers 回放升位）**：`schemas/replay/sbirs_replay.fbs` 增加 `measurement_time_s: double;` 槽位，文件标识符由 `SBI4` 升级为 `SBI5`；更新 `SbirsReplayFlatbufferCodec.cpp` 并验证旧标识符拒绝机制。

### 2. 允许范围
- **模块/目录**：
  - `include/1q/sbirs_sensor/` & `src/sbirs_sensor/`
  - `schemas/replay/sbirs_replay.fbs`
  - `include/1q/fusion/DetectionRecord.h`
  - `include/1q/precision_evaluation/` & `src/precision_evaluation/`
  - `examples/components/ground_station_fusion_component.cpp` & `examples/scenes/sbirs_triple_sat_fix_messages/`
  - `tests/unit/sbirs_sensor/`, `tests/unit/fusion/`, `tests/unit/precision_evaluation/`
- **类/函数**：
  - `SbirsDetectionRecord`（增加 `measurement_time_s`）
  - `SbirsMissionConfig`（增加 `initial_scan_phase_deg` 与配置校验）
  - `SbirsPipeline`（初始相位装配、穿越时刻解析计算、调试视图与生命周期记录同步适配）
  - `DetectionRecord`（增加 `has_timestamp` / `timestamp_s`）
  - `AdaptSbirsResultToDetectionRecords`（透传时间戳与存在标志）
  - `SbirsReplayFlatbufferCodec`（编解码 `measurement_time_s`，升级为 `SBI5`）
- **测试/文档**：
  - 增加时间戳解析计算单测、多星初始相位错峰测试、回放 SBI5 编解码及 SBI4 拒绝测试、适配层时间戳透传测试。

### 3. 明确禁止范围
- **公开头文件**：除 `SbirsOutputTypes.h`、`SbirsMissionConfig.h`、`DetectionRecord.h` 外，禁止修改任何其他公共头文件。
- **跨模块类型**：除 `DetectionRecord` 增补可选字段外，禁止引入新的跨模块结构体或侵入性强耦合类型。
- **schema/trace/replay**：禁止保留 SBI4 兼容分支（契约规定整体升位为 SBI5）；禁止触碰其他传感器的 replay schema。
- **测试阈值/skip**：禁止放宽任何现有单元测试与验收测试的门限值，禁止新增 skip。
- **兼容层**：禁止为旧版无时间戳写分支兼容胶水代码；未交付阶段坚持干净边界。
- **管线动力学外推（方案 A 否决）**：禁止在底层传感器管线 `SbirsPipeline.cpp` 内部对输入场景目标做基于速度的几何外推与视线篡改；原始测量必须反映波束掠过视线时的真实物理观测，跨星异步观测的时空补偿严禁由传感器越权承担。

### 4. 行为边界
- **输入**：`initial_scan_phase_deg` 必须在合法范围 $[0, \text{scan\_span})$ 内（配置校验防御越界），默认 0.0f。
- **输出**：`measurement_time_s` 保证在当前周期窗口 $[t_{\text{cycle\_start}}, t_{\text{cycle\_start}} + dt]$ 之间，单调可信。
- **数据流模型（离散调度与连续物理时刻分离）**：仿真引擎按整周期步长（如 1s）调度推进并打包返回输出帧，输出帧携带批次周期号 `output_frame.cycle_index`；帧内单条检出记录 `SbirsDetectionRecord` 携带连续物理测量时刻 `measurement_time_s`（如 A 星 0.5s、B 星 0.8s），二者语义分明、严格自洽。
- **错误/回退**：输入非法初始相位返回校验 `kInvalidScanPhase` 或相应 IssueCode；回放解析遇到旧版 SBI4 返回解码失败。
- **生命周期/调试/trace**：DebugView 与 LifecycleRecorder 同步增加 `measurement_time_s` 字段输出，与现有调试体系无缝兼容。

### 5. 爆炸半径与回滚
- **下游消费方影响**：所有构造或消费 `SbirsDetectionRecord`、`DetectionRecord` 的地方需要更新（主要在 SBIRS 内部、回放、适配器、示例地面站）。
- **回退难度**：破坏性 schema 升级，若回退需切回 SBI4 并在 git 树上回滚对应提交。

### 6. 验收门
- **构建**：`cmake --build --preset llvm-ninja-release-local` 零告警编译通过。
- **聚焦测试**：
  - `ctest --preset llvm-ninja-release-local -R "unit::sbirs" --output-on-failure` 100% 通过。
  - `ctest --preset llvm-ninja-release-local -R "unit::fusion" --output-on-failure` 100% 通过。
  - `ctest --preset llvm-ninja-release-local -R "unit::precision_evaluation" --output-on-failure` 100% 通过。
- **契约测试**：回放 SBI5 测试与旧标识符拒测 100% 通过。
- **探针转正**：
  - 探针 1 转正：单目标在 WFOV 往复扫中穿越时刻解析精度验证测试。
  - 探针 2 转正：多星不同 `initial_scan_phase_deg` 错峰扫描几何验证测试。
  - 探针 3 转正：`AdaptSbirsResultToDetectionRecords` 时间戳透传测试。

### 7. 非目标
- **非目标 1**：`FusionEngine` 内部非等间隔卡尔曼滤波推演（收窄裁定已定，留待后续专项）。
- **非目标 2**：2-D 俯仰栅格初始行号 `initial_scan_row_index` 配置（本次仅开放水平初始相位）。
- **非目标 3**：AR、ESR、EOS、RIR 等其他传感器的时间戳改造（本次仅针对 SBIRS 与通用探测记录）。
- **非目标 4**：传感器管线内目标动力学外推（方案 A 已否决；时空对齐归属融合层 FUSION-OQ-1）。

## 修订记录

- **修订 1（2026-09-07，用户裁定）**：
  1. 裁定点 1：批准采用绝对仿真时间 `measurement_time_s`（`double`，单位秒）口径。
  2. 裁定点 2：批准 2-D 俯仰栅格模式仅做水平方位初始相位 `initial_scan_phase_deg`，垂直行号暂默认从第 0 行起步。
  3. 裁定点 3：批准融合层实施范围收窄为“`DetectionRecord` 增补可选时间戳 + `SbirsBearingAdapter` 透传 + 评估层消费”，融合算法内部亚周期卡尔曼滤波作为后续专项演进。
  4. 状态由 draft 推进为 frozen，附 §3 冻结契约。
- **修订 2（2026-09-07，时空自洽与管线纯粹性裁定）**：
  1. 裁定点 4：明确否决底层传感器管线内部目标动力学外推（方案 A）；传感器如实测量视线角并精准标定时间戳，多星异步观测间的时空补偿由下游融合层状态滤波（FUSION-OQ-1）承担。
  2. 明确 `SbirsDetectionRecord` 保持纯粹物理量测语义，不冗余设置周期号字段；周期号保留在外层帧结构。
  3. 冻结“离散批量打包调度 (Frame Batching) + 连续物理测量时刻 (Continuous Physical Measurement Timestamp)”的数据流模型。

## §4 运行记录（Stage C 后填写）

<!-- 对照强制回写清单勾项，全部完成才允许拆脚手架（见 SKILL.md 收尾规则）：
1、正向边界：docs/<module>/ 对应文档
2、否决记录：docs/<module>/design.md 架构裁定与否决记录专节
3、开放议题：docs/common/open_questions.md 登记新议题
4、证据锁：[evidence: ...] 指向代码/测试永久锚点
-->
