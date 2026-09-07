---
Status: active
Last-reviewed: 2026-09-04
Authority: AR/ESR/ECM/RIR 公共 RF 工程架构
Answers: 公共 RF 事实域如何设计、发射事实四级 provenance、单周期数据交换时序、接收机四层影响分层
---

# 工程射频架构

`oneq::electromagnetics` 是 AR、ESR、ECM 与 RIR 的公共 RF 事实域，只共享值类型、校验和链路预算纯函数，
不共享传感器检测、受扰判决、ECCM 或资源规划算法。跨模块的契约规则（函数不得因名称相似而合并、
fail-closed 策略、saturated 语义等）见 `docs/common/contract.md` §跨模块物理基元复用；本文描述这些
规则背后的**设计结构**。

## 公共 RF 事实域

1. 公共坐标统一为 ECEF 米/米每秒，频率为 Hz、带宽为 Hz、时间为 s、功率在线性域使用 W，损耗和增益
   使用 dB/dBi。
2. 工程精度冻结为**检测单元/接收通道级统计模型**：高于"整个周期只有一个总接收功率"的原型，但不
   生成复数 IQ、不模拟未标定的射频器件压缩曲线，也不实现新的欺骗/转发算法。
3. 公共 DTO：`RfEmissionFrame` / `RfSceneFrame`、`RfSceneEmission` 和参数化 `RfWaveformSchedule`；
   旧 `RfEmissionSegment` 仅为尚未迁移模块保留的 v1 兼容类型，AR 不得使用。

## 发射事实三级 provenance

发射事实必须区分 world cycle、sensor entity 和 emission 三级 provenance：

1. **world cycle**：绑定世界周期号。
2. **sensor entity**：标识传感器/干扰设备实体（`sensor_entity_id`）。
3. **emission**：标识一次实际发射（`emission_id`）。

co-site isolation 是"发射实体 → 接收实体"的有向硬件隔离路径（`RfCoSiteIsolationPath`）。接收方隔离表命中发射方实体 ID 时按共址处理（查隔离值、时延置零）；未显式配置隔离路径的链路按远场空间传播求解，同载体近距离链路漏配由近场门（`path_length_m < minimum_far_field_range_m`）物理拒解兜底。连续、参数化脉冲列和扫频必须能在不逐 IQ 采样、不逐脉冲无界展开的前提下确定性计算时间/频率占用。

## 单程链路边界

公共单程链路只到达**接收设备输入端**：

1. 自由空间损耗、附加传播损耗、收发方向增益、极化损耗和入射功率/功率谱密度属于公共纯函数结果。
2. 大气公共层只提供路径附加损耗。
3. 匹配滤波、脉冲压缩、通道化、处理增益、热噪声账本、Pd/Pfa、receiver impairment、观测误差和 ECCM
   均由具体传感器拥有。

极化失配损耗的配对规则（`RfPolarization` 与 `RfScenePolarization` 两条枚举链同构实现，2026-09-03 冻结）：

1. 固定极化配固定极化：同值 0 dB；同为线极化互垂或同为圆极化旋向相反取 `cross_polarization_isolation_db`；
   线圆互配 3.0103 dB。
2. 全极化（`kFullPolarization`，两条相互垂直的极化通道同时工作）：全极化接收对任意固定极化发射
   合并收满 0 dB；单极化接收全极化发射只收一半 3.0103 dB；全极化对全极化 0 dB；全极化配对不取
   交叉极化隔离。
3. 任一侧非极化固定 3.0103 dB，该公理优先于全极化规则（全极化对非极化同为 3.0103 dB）。
4. 模块放行范围：仅 RIR 的 `scene_polarization` 允许填全极化（一个旋钮填收发两侧，全极化下自配对
   full/full＝0 dB）；AR/ESR/ECM 冻结不放行。

识别雷达的个体极化散射字典走**架构 B**（2026-09-03 冻结）：全量字典（四路复数
`RirPolSMatrixSample`，按视角逐行）由**场景层开机一次性载入**，每周期只把当前
视线角 ±5° 窗口行放进 `RirSceneTarget::polarization_samples`——大表零重复进库，
库输入契约形状不变；类型手册（SQLite 特征库）仍为库内一次性载入。**类型手册归库、
个体字典归场景**。回放随 RIR3 升版显式拒绝旧录制。
- **证据**：[evidence: examples/scenes/scene_script.cpp]::MakeRirSceneTargets（窗口裁剪，视线角与库内 ComputeLookAngles 同口径）
- **证据**：[evidence: schemas/replay/rir_replay.fbs]（root_type RirCycleReplayRecordV3）
- **证据**：[evidence: tests/unit/common/common_rf_link_budget_test.cpp]::FullPolarizationPairingAppliesMergeAndHalfRules
- **证据**：[evidence: tests/unit/common/common_rf_scene_test.cpp]::FullPolarizationPairingAppliesMergeAndHalfRules
- **证据**：[evidence: tests/replay/remote_identification_radar/rir_replay_session_test.cpp]::FullPolarizationSessionConfigRoundtripsByteExact
- **证据**：[evidence: src/electronic_surveillance_radar/session/EsrSessionConfigValidation.cpp]（ESR 0..4 上限天然拒绝全极化）

AR/RIR 目标回波使用模块自有的双程雷达方程与 RCS，外部雷达、ECM 和其他 RF 源才走公共单程链路。ESR 接收面
不预先区分"目标发射"和"干扰发射"，所有实际发射先进入同一个冻结 RF scene，意图/阵营只允许进入仿真
truth-evaluation 或 attribution。RIR 同形态接入：外部 RF 经 `RirCycleInput.rf_scene` 注入（仅非本机
emission，空表示无外部干扰），库内构建自发射并合并本帧求解 incident links；成功识别周期经
`RirCycleResult::emission_frame` 发布实际发射（与 AR `ArCycleResult::emission_frame` 同契约）。

## 单周期 RF 数据交换

普通调用方以各模块既有的 `Step()` / `StepWithResult()` 形状推进一个世界周期。ECM 发布的
`RfEmissionFrame(N)` 可以直接写入 AR 周期输入的独立 `interference` 字段；调用方不需要创建 RF scene、
回填 AR 自身发射、管理 token，或调用 prepare/complete 状态机：

```mermaid
sequenceDiagram
  participant Caller as Simulation loop
  participant ECM as ECM
  participant AR as AR
  participant ESR as ESR
  Caller->>ECM: StepWithResult ECM(N), optionally using ESR(N-1)
  ECM-->>Caller: RfEmissionFrame(N)
  Caller->>AR: StepWithResult AR(N, interference=frame)
  AR-->>Caller: tracks, impairment, observations, AR emission
  Caller->>ESR: StepWithResult ESR(N), optionally using AR emission
  ESR-->>Caller: hypotheses for a later ECM cycle
```

时序设计要点：

1. ESR 的成功观测最早驱动下一成功 ECM 周期；ECM 当周期发布的干扰帧可以由同周期 AR 直接消费。AR 的
   内部/外部 LPI/ECCM proposal 最早驱动下一成功 AR 周期。
2. 同周期接收波束、调谐/预选器、通道/检测窗口、噪声参数和最大线性输入功率构成不可变 receiver
   operating state，但由传感器会话内部冻结。
3. `PrepareCycle` / `CompleteCycle` / `AbandonCycle` 和 opaque token 不属于公共传感器合同；需要的发射
   准备与接收分层只作为模块内部事务步骤存在。
4. 多装备发射汇集（AR/RIR/ECM 等共同在场的编排）由调用方在编排层完成：汇集各模块
   `emission_frame` 构建世界 RF scene，再按消费方派生外部 rf_scene/interference（排除本机发射）。
   库内不提供全局 RfScene 总线或跨模块总线组件（参考实现：`examples/component_attachment` 的
   rf-world 编排，`rf_world_broker.h`）。

## 接收机影响分层

AR/ESR/RIR 必须遵守同一物理分层，但各自拥有不同算法：

1. **incident link**：公共单程链路得到每个 emission 到接收设备输入端的方向、功率/PSD 和重叠事实。
2. **front-end ledger**：传感器按实际预选器/接收方向图聚合宽带输入并判断最大线性输入边界。
3. **resolution-cell/channel ledger**：AR 按 range/Doppler/beam/time-frequency detection cell，ESR 按
   tuner/channelizer/time-frequency-angle resolution cell，RIR 按 range/Doppler/beam/time-frequency
   detection cell（副本自 AR，无 ECCM 前沿跟踪分支）计算可分辨信号和未分辨干扰。
4. **sensor decision**：AR 使用处理后 SINR/Pfa/Pd 生成量测，ESR 使用截获概率、SINR 和碰撞/掩蔽结果
   生成脉冲或能量观测，RIR 使用处理后 SINR/Pd 驱动自持检测链。只有这一层可以产生 receiver impairment
   和测量不确定度。

公共 RF v2 characterization 已覆盖自由空间、参数化 pulse/sweep 占用、传播时延、Doppler、设备级 co-site
和 W/PSD 域聚合。它仍不替代各传感器的前端/通道账本、检测统计或资源调度验收；AR↔RIR RF 链路（基线/
干扰/杂波/饱和）有跨模块 parity 测试锚定。

## 证据

[evidence: tests/unit/common/common_rf_link_budget_test]
[evidence: tests/unit/common/common_rf_scene_test]
[evidence: tests/unit/electronic_surveillance_radar/esr_rf_v2_front_end_test]
[evidence: tests/unit/electronic_surveillance_radar/esr_resolution_cell_ledger_test]
[evidence: tests/unit/remote_identification_radar/rir_rf_physical_parity_test]
