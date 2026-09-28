# 输出视图与日志体系使用教程（component_attachment）

Status: active
Last-reviewed: 2026-08-23
Authority: 可观测性单源四件事（docs/common/session_contract.md）+ 分层周期记录与投影输出模型 +
  集成端日志设施（examples/logger/README.md）
适用读者: 对"输出视图 / 两种日志 / 三模式"感到困惑的仓库开发者与外部集成方

> 本教程是**教学性导航**，不是契约文档。契约权威仍是 `docs/common/session_contract.md`
> （可观测性单源 / 规则 12/13b/13e）与 `examples/logger/README.md`；
> 源码事实以 `examples/` 为准，本文只负责把这些概念串成一条
> 可理解的主线。

## 0. 调用方只记四件事

库可观测性只有四件事（`session_contract.md`「可观测性单源」）。示例层的视图/事件日志是第五件
教学示范，不是库 API。

| 你要记住的 | 入口 | 一句话 |
| --- | --- | --- |
| **周期记录** | `StepWithResult()` → 分层 `*CycleResult` | 一次生产：执行 / 产品 / 副作用 / 仿真附件（规则 15） |
| **投影** | DebugView / Lifecycle / ExclusionCause | 记录的视图，不是第二套事实；Attach 与否零行为改变 |
| **Replay** | `*RecordingSession` + `ReplayTraceWriter` | 库内唯一周期落盘；按层编码；回放入口 `ReplayXxxTrace()` |
| **`PROJECT_LOG`** | 库内人读旁路 | 默认关闭；状态判断禁止解析日志 |

需要复现实验时包一层 Recording 包装器。人读摘要由调用方从 `CycleResult` 自己写；示例中文事件
只活在 `examples/`。验收文件（`*acceptance.log`）也不是库可观测性 API。

下面三个词是**示例教程**用语，用来解释 `component_attachment` 怎么把信封/投影转成自己的日志：

| 你感到困惑的词 | 它到底指什么 | 一句话 |
| --- | --- | --- |
| **输出视图** | 可选观测投影里的 DebugView（旧称 L3） | 传感器每周期把"探测/航迹结果 + 逐目标状态 + 排除诊断"整理成一份人读友好的快照（`*OutputDebugView`），由调用方决定怎么落盘 |
| **两种日志** | 库内部日志 vs 集成端日志 | 一个写给"调试库算法"的人看（英文、带级别），一个写给"检查仿真行为"的人看（中文、事件/视图分文件） |
| **三个模式** | 视图三模式 + 事件三模式 | 每周期数据都会产生，**落盘多少、怎么落**由编译期宏决定：视图控制密度（异常/变化/摘要），事件控制粒度（关键/聚合/全量） |

三者关系一句话：**库每次仿真周期生产一份分层周期记录；目标列表型再拼可选投影
（DebugView / 生命周期 / 排除差分）；示例示范"集成方怎么把投影转成自己的日志"。**

---

## 1. 分层记录 + 投影：先搞清"输出视图"指什么

有 Session 的传感器模块每次 `StepWithResult` 产出**一份分层周期记录**，并按产品形态选装观测投影
（`docs/common/session_contract.md` 规则 15；旧称两通道 / L1/L2/L3）：

| 层/投影 | 入口 | 责任 | 谁消费 |
| --- | --- | --- | --- |
| **产品** | `Step()`（糖） | 真实传感器/产品输出（SAR 含聚焦图像） | 业务逻辑（融合、跟踪、成像） |
| **周期记录** | `StepWithResult()` | 执行状态 + 产品 + 副作用 + 仿真附件 | 状态判断（`status == kCompleted`） |
| **观测投影** | DebugView / Lifecycle / ExclusionCause | 人读快照、生命周期事件、排除差分 | 调用方落盘/观测 |

落地前 `*CycleResult` 仍扁平，且 SAR `Step()` 还拿不到图像——以规则 15 为准，不以当前头文件为准。

**"输出视图" = DebugView 投影**（不是仿真附件层的归属对照表）。它每周期由
`*OutputDebugViewBuilder::Build(input, result)` 构造——无状态快照：只把本周期"逐目标状态
（AR/RIR 航迹态；EOS/SBIRS 检测态）+ 结构化量值 + 规则 13b 排除诊断"装进一个 struct。

三个容易混淆的点：

1. **视图每周期都会产生**（有 Builder 的模块），与落盘无关。组件 `Step` 里总是 `Build` 一个
   `last_debug_view_`，只是"写不写日志、写几行"由三模式决定。
2. **视图是"本周期快照"**，没有跨周期记忆。想看"目标从什么时候开始丢失"，用 Lifecycle
   投影或由调用方落盘自累积（规则 12）。
3. **视图 ≠ 探测/识别产品**。产品进融合；视图只给人看/落盘。

示例中每个有 DebugView 的组件（AR/EOS/SBIRS 目标列表型 + SAR 阶段型）把视图暴露为
`LastDebugView()`（最近周期快照，关机清零），并在 `Step` 内直写中文人读行（详见下节）。
RIR 三类投影已落地（2026-08-22，`RirOutputDebugViewBuilder` / `RirTrackLifecycleRecorder` /
`RirExclusionCauseRecorder`），组件同样经 `LastDebugView()` + recorder 事件写视图/事件行；
ESR 库内无 DebugView，不适用视图落盘；ECM 同形自拼；推演组件读
融合运动学估计直写关键点摘要行（详见 §3 通道 B）。

---

## 2. 两种日志：为什么有两个、各是什么

示例同时存在**两套独立的日志体系**，这是初学者最容易混的地方。它们互相不替代：

| | **库内部日志** | **集成端日志** |
| --- | --- | --- |
| 谁写 | 库内 `PROJECT_LOG_*` 宏（src/ 内部） | 示例组件源文件（`CA_LOG_EVENT*` / `CA_LOG_VIEW` 宏） |
| 后端 | spdlog **默认 logger**（`InitIntegrationLog` 装配为文件 sink）；**Windows 上 spdlog 关闭，改由库内内置文件后端**（`ProjectFileLog`，`ONEQ_ENABLE_FILE_LOG` 门控） | spdlog **命名 logger** `"integration_events"` / `"integration_views"`（stdout + 文件）；**Windows 上为示例自带 `std::ofstream` 文件后端**（仅落文件，不打 stdout） |
| 输出文件 | `1q_library.log` | `integration_events.log` + `integration_views.log` |
| 行格式 | 时间戳 + 级别 + 消息（英文） | 仅消息体（中文人读） |
| 给谁看 | 调试库算法本身（信号处理/跟踪内部） | 检查仿真行为是否符合预期（事件链/目标状态） |
| 语言 | 英文（项目约束） | 中文（示例业务层） |
| 生命周期 | 宿主拥有（`InitIntegrationLog` 装配） | 宿主拥有（同一函数） |

**一句话区分**：`1q_library.log` 回答"**库内部发生了什么**"（如 `[SignalPipeline] cycle=...`、
SAR squint 拒绝等）；`integration_*.log` 回答"**仿真世界里发生了什么**"（目标确认了、丢失了、
命令下发了、本周期的目标状态明细）。

> 重要：`PROJECT_LOG_*` 的日志消息必须保持英文（CLAUDE.md 约束），而集成端日志是示例
> 业务层，用中文。改 `1q_library.log` 的内容不是集成方职责；集成方看的是 `integration_*.log`。

> **Windows 差异**：Windows 不安装 spdlog/fmt（当前主线为 v141 预设
> `VisualStudio.15.0-amd64`，Conan；VS2015 C++14 与 no-Conan 为备选脚手架），库内
> `PROJECT_LOG_*` 由 `src/common/logging/ProjectFileLog`（纯 C++11、无第三方依赖）
> 承载，行为同构：同样产出 `1q_library.log`（默认 CWD，可用 `OpenFileLog(path)` /
> 环境变量 `ONEQ_FILE_LOG_PATH` / 宏 `ONEQ_FILE_LOG_PATH` 覆盖），行格式
> 时间戳 + 级别 + 英文消息；默认最低级别 info（debug 需 `SetFileLogLevel(kDebug)`）。
> 编译期总开关 `ONEQ_ENABLE_FILE_LOG`（默认 ON）关闭后宏回到空操作。示例在
> Windows 上同样构建（2026-08-19 起双后端）：集成端日志宏经
> `CA_LOG_BACKEND_SPDLOG=0` 走示例自带的 `std::ofstream` 文件后端
> （`logger_format.h` 迷你格式化，`{}` / `{:.Nf}`，与库内 `ProjectFileLog`
> 口径一致），行文案与 spdlog 分支逐字一致；仅落文件不打 stdout（Windows
> 控制台代码页可能非 UTF-8）。`InitIntegrationLog` 在 Windows 分支经
> `ONEQ_FILE_LOG_PATH` 把库日志指到同一输出目录——三个日志文件
> （`integration_events.log` / `integration_views.log` / `1q_library.log`）
> 双平台均落在 demo 的 `--output-dir` 下。

---

## 3. 集成端日志的两个通道：事件 & 视图

集成端日志拆成**两个文件、两个通道**，对应两种不同粒度的信息：

### 通道 A：事件日志 → `integration_events.log`

记录**"发生了什么"**：跨周期边界事件（目标确认/丢失、首发现/更新/丢失、SAR 产品、航点到达、
指令下发、排除原因变化等）。由 `CA_LOG_EVENT` / `CA_LOG_EVENT_DUP` 宏在组件源文件事件产生处
就地记录。事件行自含周期号与时间：

```
[事件:target_confirmed] 周期=101 时间=101.00s 目标=1001 位置=(30.00123,120.00123)
[事件:waypoint_reached] 周期=182 时间=182.00s 航点=1 到达距离=...
[事件:command_issued]   周期=215 时间=215.00s 指令=ENGAGE_HIGH_THREAT 键=1001 威胁分=7.20
[事件:exclusion_cause]  周期=220 时间=220.00s 目标=1002 类型=进入排除 排除码=sbirs.target_out_of_wfov 主因=无归因
```

事件类型（`type` 稳定字符串）一览，来源见 `components/` 与 `demo_output.cpp`：

| type | 含义 | 宏 | 发布信号 |
| --- | --- | --- | --- |
| `target_confirmed` / `target_lost` | AR 首确认/失跟 | `CA_LOG_EVENT` | 是 |
| `eos_detection` | EOS 首发现/更新/丢失 | `CA_LOG_EVENT`（更新用 `_DUP`） | 是 |
| `sbirs_detection` | SBIRS 首发现/更新/coasting/丢失 | 同上 | 是 |
| `sar_product` | SAR 产出/持续/丢失/失败 | `_DUP`（持续类） | 是 |
| `emitter_hypothesis` | ESR 辐射源假设 | `CA_LOG_EVENT_DUP` | 是 |
| `fusion_updated` | 融合态势更新 | `CA_LOG_EVENT_DUP` | 是 |
| `waypoint_reached` / `platform_state` | 航点到达 / 平台状态 | `EVENT` / `_DUP` | 是 |
| `threat_level_up` | 威胁等级升级 | `CA_LOG_EVENT` | 是 |
| `command_issued` | 决策指令下发 | `CA_LOG_EVENT` | 是 |
| `ecm_jamming` | ECM 干扰决策下发（技术/决策数/功率；干扰发射经 rf-world 传播，不发 World 信号） | `CA_LOG_EVENT` | 否 |
| `rir_recognition` | RIR 识别结论确认态沿（状态迁移到确认时逐条） | `CA_LOG_EVENT` | 否 |
| `rir_designation` | RIR 指定任务终态沿（识别达成完成/窗口耗尽作废） | `CA_LOG_EVENT` | 否 |
| `rir_track_confirmed` / `rir_track_lost` | RIR 航迹首确认/丢失（生命周期 recorder） | `CA_LOG_EVENT` | 否 |
| `rir_designation_dropped` | RIR 指定任务作废/回扫终态（生命周期 recorder，镜像 revert_reason） | `CA_LOG_EVENT` | 否 |
| `patrol_loop_restart` | 巡逻循环重启（**纯日志，无信号**） | `CA_LOG_EVENT` | 否 |
| `exclusion_cause` | 排除原因跨周期变化（**纯诊断，无信号**） | `CA_LOG_EVENT` | 否 |

> **`CA_LOG_EVENT` vs `CA_LOG_EVENT_DUP` 的区别只有一处**：`_DUP`（周期性重复事件，
> 每周期都有的 `platform_state`、`fusion_updated`、`kUpdated`/`kProductSustained` 更新类）
> 在事件模式一（KEY）下**不落盘**，但信号照常发布。模式二（聚合）/模式三（全量）下两者
> 行为相同。**不是**"DUP = 重复刷屏"的意思。

### 3.1 记录"探测不到的原因变更"：排除原因跨周期差分（规则 13e）

你在 example 里看到的"记录探测不到的原因变更"，就是规则 13e 的**排除原因跨周期差分
记录器**（库内 `*ExclusionCauseRecorder`，AR/EOS/SBIRS/ESR 四模块各一个）。它和
生命周期事件（首发现/丢失）是**并列的独立观测通道**，容易混，拆开看：

**背景：排除原因（13b）**。目标被门控排除（探测不到）时，库在每周期问题列表 `issues`
里写一条 kInfo 诊断，携带 `code`（机器键，如 `sbirs.target_out_of_wfov`）和 `cause`
（门内归因主因枚举，如 `ArIssueCause::kDistanceLimited`）。**但这是每周期瞬时快照**——
只能回答"本周期它为什么探测不到"，回答不了"原因从什么变成了什么"。

**差分记录器（13e）**：库内 `*ExclusionCauseRecorder` 挂到 Session 后，由
`StepWithResult()` 内部自动驱动（调用方不手动调 `Update()`），对**持续被排除**的实体
做跨周期差分，产出四类事件（`GetLastEvents()` 取）：

| 事件 | 含义 | 何时产 |
| --- | --- | --- |
| **A1 原因稳定** | (code,cause) 与上周期相同 | **不产事件**（没有变化就不刷屏） |
| **A2 进入排除** | 从未被排除 → 被排除 | 产 `kEntered` |
| **A3 原因变化**（核心） | 被排除实体的 (code,cause) 变了 | 产 `kChanged` |
| **A4 退出排除** | 从被排除 → 恢复 | 产 `kExited` |

差分键是 **(code, cause) 组合对**（不是纯 cause）：避免"同为 `kNone` 的具体门切换盲区"
（如 SBIRS 遮挡↔距离带 cause 都 `kNone` 但 code 不同）。组件拿到事件后逐条写
`CA_LOG_EVENT(world, "exclusion_cause", ...)`——**纯诊断观测，不发 World 信号**
（不驱动融合/威胁等下游），`type` 统一为 `exclusion_cause`：

```
[事件:exclusion_cause] 周期=220 时间=220.00s 目标=1002 类型=进入排除 排除码=sbirs.target_out_of_wfov 主因=无归因
[事件:exclusion_cause] 周期=235 时间=235.00s 目标=1002 类型=原因变化 旧码=sbirs.target_out_of_wfov 旧主因=无归因 新码=sbirs.target_snr_below_threshold 新主因=距离受限
[事件:exclusion_cause] 周期=250 时间=250.00s 目标=1002 类型=退出排除 旧码=sbirs.target_snr_below_threshold 旧主因=距离受限
```

**"旧码/旧主因、新码/新主因"分别指哪个周期**（记不住就看事件结构字段名）：

- **code** = 排除**门**的机器键（"哪个门排除的"，如 `sbirs.target_out_of_wfov`）；
  **cause** = 门内归因**主因**（"聚合门失败主要是哪条物理链路"，如 `kDistanceLimited`）。
- **旧码/旧主因** = `previous_code`/`previous_cause`：**上一执行周期**该实体的 (code,cause)；
  **新码/新主因** = `current_code`/`current_cause`：**本执行周期**该实体的 (code,cause)。
- A3 事件 = 组合对从"旧"变到"新"；A2/A4 只有单侧字段有值（见下表），所以日志行
  里 A2 只有"新"（`进入排除 排除码=...`）、A4 只有"旧"（`退出排除 旧码=...`）。

| 事件 | 旧码/旧主因 | 新码/新主因 | 日志行形态 |
| --- | --- | --- | --- |
| A2 进入排除 | 空 / `kNone`（上周未被排除） | 有效 | 只写 `排除码=… 主因=…` |
| A3 原因变化 | 有效（≠ 新） | 有效 | `旧码=… 旧主因=… 新码=… 新主因=…` 四段齐全 |
| A4 退出排除 | 有效 | 空 / `kNone`（本周恢复） | 只写 `旧码=… 旧主因=…` |

`kNone` 本身是合法值：**具体门**（遮挡/视场外/距离带等本身可定位的门）不强制细分
主因，`cause` 保持 `kNone`；只有**聚合门**（如 SNR 门折入距离/波束/噪声底/RCS 多因素）
才填主因。

四个模块的差别（决定日志行里的实体标识长什么样）：

- **AR / EOS / SBIRS**：以 `target_id` 为键，行首 `目标={}`（同上示例）；
- **ESR**：无 target_id 概念，以**发射源标识三元组**（platform/equipment/emission id）
  为键，行首 `辐射源=(platform=...,equipment=...,emission=...)`；
- **EOS**：单一视场门（`eos.target_out_of_fov`），A3 由越界轴变化（az/el/both）驱动。

为什么它**天然适配默认的事件模式一（KEY）**：A1 稳定不产事件、A2/A3/A4 都是边界事件，
逐条落盘不刷屏——这正是"周期聚合/逐条全量"模式之外最省日志量的形态。四模块示例实现见
各组件 `Step` 末尾的 `for (const auto& event : exclusion_.GetLastEvents())` 循环
（ESR 无 lifecycle recorder，exclusion 是它组件里**首个** recorder）。

> 一句话区分：**生命周期事件**（`target_confirmed`/`eos_detection`/`sar_product`）回答
> "探测状态在边界上变了（发现/丢失/产出）"；**排除原因事件**（`exclusion_cause`）回答
> "探测不到的原因在变（进入/换因/恢复）"。前者来自 `*LifecycleRecorder`，后者来自
> `*ExclusionCauseRecorder`，两条通道完全独立。

### 通道 B：视图日志 → `integration_views.log`

记录**"每周期目标状态长什么样"**：AR/EOS/SBIRS 各组件每周期一行（或几行，取决于视图模式）
目标状态明细 + 排除诊断；SAR 为阶段型摘要行；Threat 组件也有每周期视图行；RIR（标准投影
DebugView，航迹状态枚举 + 识别诊断）、ECM（干扰发射状态）与推演（关键点/类型概率，读融合运动学估计）
组件各每周期直写自有摘要行。由 `CA_LOG_VIEW` 宏在组件 `Step` 内直写（字符串归属组件）：

```
[视图:ar] 周期=5 完成=是 目标=[1001 已确认(RCS 2.20m^2), 1002 候选(RCS 1.40m^2)] 问题=[ar.target_snr_below_threshold 目标信噪比低于门限]
[视图:eos] 周期=5 执行=是 目标=[1001 已检测(方位90.0° 俯仰1.0° 距离12.5km)] 问题=[无]
[视图:sbirs] 周期=5 执行=是 目标=[1001 已检测(方位120.0° 俯仰-89.0°)] 问题=[sbirs.target_out_of_wfov 目标宽视场外（不在 WFOV 扫描覆盖内）。]
[视图:sar] 周期=5 执行=是 阶段=L1 RDA 图像 L1图像=有 L3图像=无 聚焦=有 信噪比=2.3dB 目标数=2 问题=[无]
[视图:threat] 周期=5 目标=2 高=1 中=0 低=1 最高=键1001:7.20 升级=是
[视图:rir] 航迹=1 确认=是 指定=执行中 驻留中心=(359.8°,45.2°) [目标=1001(F-16C) 位置LLA=(29.89680,119.88000,400) 速度=250.0]
[视图:ecm] 周期=5 状态=已执行 发射数=1 ESR批次=4
[视图:inference] 键=1001 类型=2 p=0.65 发射=(30.102,120.015) t=-320s σ=850m 落点=时域外
```

模块稳定名：`ar` / `eos` / `sbirs` / `sar` / `threat` / `rir` / `ecm` / `inference`
（ESR 库内无 DebugView，也没有视图行；`rir` / `ecm` 场景未启用或组件未挂载时自然没有视图行，
`inference` 恒挂载在融合之后）。

---

## 4. 视图三模式：控制"每周期状态写多少行"

DebugView 每周期**都会**构建（`LastDebugView()` 恒有值），但**落盘多少行**由编译期宏门控。
三种模式三选一（未选中的模式**不参与编译**）：

| 模式 | 宏 | 行为 | 日志量 | 典型用途 |
| --- | --- | --- | --- | --- |
| **模式一：只落非标称行** | `CA_VIEW_LOG_MODE_NONNOMINAL` | 只写"非标称"目标（AR 非 `kConfirmed`；EOS/SBIRS 非 `kDetected`），全标称时写一行"全部正常" | ∝ 异常数 | 只想盯"谁出问题了" |
| **模式二：跨周期增量（默认）** | `CA_VIEW_LOG_MODE_DELTA` | 只写状态与**上一周期不同**的目标行（组件持有 `prev_track_status_` 表），无变化时写一行"无状态变化" | ∝ 变化数 | 想看状态迁移过程，日志量可控 |
| **模式三：每周期摘要** | `CA_VIEW_LOG_MODE_SUMMARY` | 每周期**恰好一行**中文摘要：目标状态明细带结构化量值 + 问题 code+中文名（i18n 查表） | 恒定（=周期数） | 逐周期完整记录，量可预测 |

> **⚠ 源码注释陷阱（易踩点）**：组件 `.cpp` 里三模式分支写成
> `#if NONNOMINAL → #elif DELTA → #else  /* 模式三（默认） */`——`#else` 分支的注释
> 写着"（默认）"，但**实际的编译期默认是 `CA_VIEW_LOG_MODE_DELTA`**（`logger_modes.h`
> 的兜底宏），`#else` 只是"剩余情况 = SUMMARY"的兜底写法。所以不传 CMake 变量时走的是
> **模式二 DELTA**，不是模式三。读源码时别被"（默认）"注释带偏：**默认 = DELTA + KEY**。

三模式同一周期下的**行数对比**（假设 2 个目标、本周期 1 个状态变化、1 个非标称）：

```
模式一（NONNOMINAL）:  只写非标称的那 1 行（若全标称则写 1 行"全部正常"）
模式二（DELTA）:       只写状态变化的那 1 行（若无变化则写 1 行"无状态变化"）
模式三（SUMMARY）:     写 1 行完整摘要（恒 1 行，内容最全）
```

> 想看**同一个 5 周期场景在六种模式下的完整逐行日志**，见 §5.1 的实例对照。

> **模式二的状态表只增不减**（`prev_track_status_` 按 target_id 记录上周期状态，首次出现
> 视为变化）。目标集长期收缩时调用方可按需清理——示例保持简单，不清理。

三种模式都是**纯观测**：不影响会话执行、不影响信号、不影响冒烟断言（见 §8）。

---

## 5. 事件三模式：控制"事件写多细"

事件三选一（同一套编译期门控）：

| 模式 | 宏 | 行为 | 日志量 | 典型用途 |
| --- | --- | --- | --- | --- |
| **模式一：只记关键事件（默认）** | `CA_EVENT_LOG_MODE_KEY` | `CA_LOG_EVENT` 逐条落盘，`CA_LOG_EVENT_DUP`（周期性重复事件）不落盘 | ∝ 关键事件数 | 默认：`platform_state`/`fusion_updated` 每周期刷屏，但你不关心 |
| **模式二：周期聚合** | `CA_EVENT_LOG_MODE_AGGREGATE` | 每周期把全部事件聚合成**一行** | 恒定（=周期数） | 想每周期看一个"事件统计" |
| **模式三：逐条全量** | `CA_EVENT_LOG_MODE_ALL` | 事件逐条落盘（含重复事件） | ∝ 全部事件 | 排查"每周期到底发了什么" |

模式二的行格式（`logger.cpp` 的 `[事件聚合]`）：

```
[事件聚合] 周期=101 事件数=4 [目标确认×1, 光电探测×2, 平台状态×1]
```

模式三下 `CA_LOG_EVENT_DUP` 也逐条落盘，行格式同 `CA_LOG_EVENT`。

> 事件计数（`EventCount()`/`SbirsEventCount()`/`SarProductEventCount()`）**与模式无关**：
> 宏背后设施无论哪种模式都会 `++g_event_count`，只是落不落盘的区别。冒烟断言读的是计数，
> 所以任意模式组合下断言都成立。

### 5.1 六种模式逐行实例：同一个 5 周期迷你场景

光看定义不如直接看日志。下面用**同一个仿真场景**跑 6 种模式，逐行列出各自写出的文件内容
（视图侧以 AR 组件为例，EOS/SBIRS 与之同构；事件侧覆盖多类事件）。所有模式跑的是**同一个
世界**，唯一区别是"落盘密度"。

**迷你场景时间线**：

| 周期 | AR 目标 1001 | AR 目标 1002 | 事件 |
| --- | --- | --- | --- |
| 1 | 候选 | 候选 | platform_state（重复） |
| 2 | 候选 → **已确认** | 候选 | **target_confirmed**(1001)、platform_state |
| 3 | 已确认 | 候选（**SNR 排除诊断**） | platform_state |
| 4 | 已确认 | 候选 → **丢失** | **target_lost**(1002)、**waypoint_reached**、platform_state |
| 5 | 已确认 | 丢失 | platform_state |

---

**① 视图模式一（`nonnominal`）→ `integration_views.log`**——只写非标称目标行
（跳过 `kConfirmed`），日志量 ∝ 异常数：

```
[视图:ar] 周期=1 目标=1001 状态=候选 位置=(0.0,11400.0,400.0)m 速度=47.0m/s
[视图:ar] 周期=1 目标=1002 状态=候选 位置=(0.0,14000.0,400.0)m 速度=45.0m/s
[视图:ar] 周期=2 目标=1002 状态=候选 位置=(0.0,14000.0,400.0)m 速度=45.0m/s
[视图:ar] 周期=3 目标=1002 状态=候选 位置=(0.0,14000.0,400.0)m 速度=45.0m/s
[视图:ar] 周期=4 目标=1002 状态=丢失 位置=(0.0,14000.0,400.0)m 速度=45.0m/s
[视图:ar] 周期=5 目标=1002 状态=丢失 位置=(0.0,14000.0,400.0)m 速度=45.0m/s
```

注意：周期 2 起 1001 已确认（标称）被跳过，只写 1002。5 个周期共 6 行。

**② 视图模式二（`delta`，默认）→ `integration_views.log`**——只写状态与上一周期**不同**
的目标行（首次出现也算变化），无变化写一行"无状态变化"：

```
[视图:ar] 周期=1 目标=1001 状态=候选     ← 首次出现（视为变化）
[视图:ar] 周期=1 目标=1002 状态=候选     ← 首次出现
[视图:ar] 周期=2 目标=1001 状态=已确认   ← 1001 变了（候选→已确认）
[视图:ar] 周期=3 无状态变化              ← 两个目标状态都没变
[视图:ar] 周期=4 目标=1002 状态=丢失     ← 1002 变了（候选→丢失）
[视图:ar] 周期=5 无状态变化
```

5 个周期共 6 行，但**行内容**与模式一完全不同：这里留下的是"状态迁移时间点"。

**③ 视图模式三（`summary`）→ `integration_views.log`**——每周期**恰好一行**完整摘要，
带结构化量值（RCS/角度）与问题 code+中文名：

```
[视图:ar] 周期=1 完成=是 目标=[1001 候选(RCS 2.20m^2), 1002 候选(RCS 1.40m^2)] 问题=[无]
[视图:ar] 周期=2 完成=是 目标=[1001 已确认(RCS 2.20m^2), 1002 候选(RCS 1.40m^2)] 问题=[无]
[视图:ar] 周期=3 完成=是 目标=[1001 已确认(RCS 2.20m^2), 1002 候选(RCS 1.40m^2)] 问题=[ar.target_snr_below_threshold 目标信噪比低于门限]
[视图:ar] 周期=4 完成=是 目标=[1001 已确认(RCS 2.20m^2), 1002 丢失(RCS 1.40m^2)] 问题=[无]
[视图:ar] 周期=5 完成=是 目标=[1001 已确认(RCS 2.20m^2), 1002 丢失(RCS 1.40m^2)] 问题=[无]
```

5 个周期恒定 5 行，量可预测；每周期都能看到"所有目标现在什么状态、本周有什么诊断"。

**三种视图模式一句话对比**：① 只回答"谁非标称"，② 只回答"谁变了"，③ 每周期都回答
"所有目标现状 + 本周诊断"。

---

**④ 事件模式一（`key`，默认）→ `integration_events.log`**——只落关键事件
（`CA_LOG_EVENT`），DUP 重复事件不落盘（信号照常发布）：

```
[事件:target_confirmed] 周期=2 时间=2.00s 目标=1001 位置=(30.00123,120.00123)
[事件:target_lost]      周期=4 时间=4.00s 目标=1002 原因=失跟
[事件:waypoint_reached] 周期=4 时间=4.00s 航点=1 到达距离=12.3m
```

5 个周期只有 3 行——每周期刷屏的 platform_state / fusion_updated 全被过滤。

**⑤ 事件模式二（`aggregate`）→ `integration_events.log`**——每周期把全部事件聚合成
**一行**（周期边界落上一周期，结束时落最后一周期；中文名查 `logger.cpp` 映射表）：

```
[事件聚合] 周期=1 事件数=1 [平台状态×1]
[事件聚合] 周期=2 事件数=2 [目标确认×1, 平台状态×1]
[事件聚合] 周期=3 事件数=1 [平台状态×1]
[事件聚合] 周期=4 事件数=3 [目标丢失×1, 航点到达×1, 平台状态×1]
[事件聚合] 周期=5 事件数=1 [平台状态×1]
```

5 个周期恒定 5 行；看到的是"每周期各类事件各发生了几次"的统计，而非逐条明细。

**⑥ 事件模式三（`all`）→ `integration_events.log`**——事件逐条全量落盘，DUP 也落：

```
[事件:platform_state]   周期=1 时间=1.00s 位置=(30.00000,120.00000) 高度=400.0m …
[事件:target_confirmed] 周期=2 时间=2.00s 目标=1001 位置=(30.00123,120.00123)
[事件:platform_state]   周期=2 时间=2.00s 位置=(30.00021,120.00045) 高度=400.0m …
[事件:platform_state]   周期=3 时间=3.00s 位置=(30.00042,120.00091) 高度=400.0m …
[事件:target_lost]      周期=4 时间=4.00s 目标=1002 原因=失跟
[事件:waypoint_reached] 周期=4 时间=4.00s 航点=1 到达距离=12.3m
[事件:platform_state]   周期=4 时间=4.00s 位置=(30.00084,120.00136) 高度=400.0m …
[事件:platform_state]   周期=5 时间=5.00s 位置=(30.00105,120.00182) 高度=400.0m …
```

5 个周期 8 行——比 key 多出来的 5 行全是周期性重复事件，这就是默认模式要过滤掉的刷屏。

**三种事件模式一句话对比**：④ 只留"世界状态的边界变化"，⑤ 每周期一个"事件统计"，⑥
"每一件发生的事都在"。

> **共同要点**：无论选哪种模式，仿真世界、信号发布、`LastDebugView()` 内容、事件计数都
> **完全一样**——变的只有 `integration_*.log` 里写多少行、长什么样。对照着看：默认组合
> （②+④）6 行 + 3 行，最省；`summary+all`（③+⑥）5 行 + 8 行，最全。

---

## 6. SAR 例外：阶段型视图，不适用目标级三模式

SAR 是**集体成像模型**，没有逐目标状态（探测/跟踪语义），因此：

- SAR 的 `*OutputDebugView` 是**阶段型**（执行状态/完成阶段/L1 RDA/L3 BP 成像标志/SNR/点目标数/问题列表）；
- 不适用 AR/EOS/SBIRS 那种目标级三模式落盘，`LogDebugView` 没有 `#if` 分支，**每周期恒写一行摘要**；
- 因此冒烟断言里 `sar_views == 周期数`（严格相等），而 AR/EOS/SBIRS 是 `≥ 周期数`
  （默认 delta 模式下状态变化周期会写多行）。

同理，SAR **无排除诊断**（规则 13b 空洞条款），问题列表多为阶段诊断。

ECM / 推演（inference）与威胁（threat）的视图行同属**摘要型**（每周期恒写、无目标级
三模式分支）：ECM 库内无 DebugView，组件以自有结果结构直写；推演行读融合运动学估计、
威胁行读融合态势，均为每周期单行摘要。RIR 自 2026-08-22 起为目标列表型（库内
`RirOutputDebugView`，无航迹目标回填输入斜距/视线角），与 AR/EOS/SBIRS 同样适用目标级
三模式落盘；其摘要行保留 航迹=/确认=/指定=/驻留中心= 汇总令牌。

---

## 7. 模式怎么切换（两条互斥途径，均编译期生效）

### 途径一：CMake 构建时控制（推荐，无需改源码）

```bash
cmake --preset llvm-ninja-release-local -DENABLE_EXAMPLES=ON \
    -DCA_VIEW_LOG_MODE=summary -DCA_EVENT_LOG_MODE=aggregate
cmake --build --preset llvm-ninja-release-local --target component_attachment_demo
```

- `CA_VIEW_LOG_MODE`：`summary` | `nonnominal` | `delta`（空 = 用源码默认 `delta`）
- `CA_EVENT_LOG_MODE`：`all` | `key` | `aggregate`（空 = 用源码默认 `key`）
- **非法值 configure 时 `FATAL_ERROR`**，不会悄悄编译出一个怪行为

### 途径二：源码调试时（与 CMake 途径互斥）

编辑 `examples/logger/logger_modes.h` 底部的注释区，每次只取消注释
一个视图模式 + 一个事件模式，重新编译：

```cpp
// #define CA_VIEW_LOG_MODE_SUMMARY         // 模式三：每周期一行人读摘要
// #define CA_VIEW_LOG_MODE_NONNOMINAL      // 模式一：只落非标称目标行
// #define CA_VIEW_LOG_MODE_DELTA           // 模式二：只落跨周期状态变化行
// #define CA_EVENT_LOG_MODE_ALL            // 模式三：逐条全量
// #define CA_EVENT_LOG_MODE_KEY            // 模式一：只记关键事件
// #define CA_EVENT_LOG_MODE_AGGREGATE      // 模式二：周期聚合
```

规则：**每次只启用一个视图模式 + 一个事件模式**（二选一），否则模式选择区自身的
`#if !defined(...)` 兜底逻辑可能产生歧义。

---

## 8. 产出文件一览与运行

```bash
cmake --preset llvm-ninja-release-local -DENABLE_EXAMPLES=ON
cmake --build --preset llvm-ninja-release-local --target component_attachment_demo
./build/llvm-ninja-release-local/bin/component_attachment_demo \
    [--scene <path>] [--cycles <n>] [--output-dir <dir>]
# 每场景可执行（主开发路径；日志默认落 examples/log/<场景 log_dir>/）：
cmake --build --preset llvm-ninja-release-local --target <scene_name>
./build/llvm-ninja-release-local/bin/<scene_name> [--cycles <n>]
```

默认输出到 `examples/log/`：

| 文件 | 内容 | 属于哪套 |
| --- | --- | --- |
| `integration_events.log` | 事件行 / 事件聚合行（§3 通道 A） | 集成端 |
| `integration_views.log` | 各组件每周期视图行（§3 通道 B） | 集成端 |
| `1q_library.log` | 库内部 `PROJECT_LOG_*` 日志（时间戳+级别+英文消息） | 库内部 |
| `platform_track.csv` / `target_truth.csv` / `route_plan.csv` / `zones.csv` | 可视化 CSV（统一契约 v2，`build_viewer.py` 消费） | **不是日志** |

> **最容易混的一点**：CSV（`platform_track.csv` 等）是**结构化可视化数据**，给
> `examples/common/viz/build_viewer.py` 画图用；`*.log` 是**人读日志**。两者都落盘，
> 但目的完全不同，三模式只作用于两个 `integration_*.log`。

---

## 9. 该用哪个模式：决策表

| 你的目的 | 视图模式 | 事件模式 |
| --- | --- | --- |
| 什么都不配，跑通就行 | `delta`（默认） | `key`（默认） |
| 只想盯"谁出问题/被排除" | `nonnominal` | `key` |
| 想要逐周期完整可回溯记录，日志量可预测 | `summary` | `aggregate` |
| 排查"每个周期到底发了什么事件" | 任意 | `all` |
| 输出归档给后续分析，逐周期都要 | `summary` | `aggregate` |

调试建议路径：先 `summary + all` 跑短周期看全貌 → 锁定问题周期 → 换 `nonnominal/delta + key`
盯迁移与异常 → 恢复默认（`delta + key`）。

**日常推荐（结论）**：什么都不配置就对了——**视图 `delta` + 事件 `key`（默认组合）**。
原因：① DUP 事件（平台状态/融合更新/持续类）在 key 下不落盘但信号照常发布，日志不会被
周期性刷屏淹没；② 规则 13e 排除原因差分（A1 稳定不产、A2/A3/A4 边界事件）天然就是这个
粒度；③ delta 的日志量 ∝ 状态变化数，巡航稳定段每周期只有一行"无状态变化"，状态迁移
一眼可扫。专项排查再临时切换（`summary+all` 看全貌、`nonnominal+key` 盯异常、
`summary+aggregate` 归档），看完换回默认——三模式纯观测，切换不影响仿真结果与断言。

---

## 10. 常见误区（FAQ）

**Q1：改了模式会影响仿真结果吗？**
不会。三模式只控制"集成端日志写多少行"，编译期裁剪；不影响会话执行、信号发布、
`LastDebugView()` 内容或冒烟断言（断言按计数且与模式无关）。

**Q2：`1q_library.log` 和 `integration_events.log` 都有"事件"，是不是重复？**
不是。库日志是算法内部过程（英文、带级别），事件日志是业务级跨周期边界事件（中文人读）。
同一件事可能两边都有（如 SAR squint 拒绝），但视角不同、格式不同、消费方式不同。

**Q3："输出视图"是不是就是 `integration_views.log`？**
不是。视图是 `*OutputDebugView` 这个**库内结构化快照**（`LastDebugView()` 可拿到）；
`integration_views.log` 只是示例**示范的一种落盘形式**（中文人读行）。外部集成方可以
按自己的结构化格式（JSON/FlatBuffers）落盘 `LastDebugView()`，不一定用示例的人读行方式。

**Q4：`CA_LOG_EVENT_DUP` 是"重复事件会写多行"吗？**
不是。`_DUP` 只在默认事件模式（KEY）下**不落盘**；模式二/三下与 `CA_LOG_EVENT` 行为一致。
它标记的是"周期性重复事件"这一**性质**（每周期都有的平台状态、融合更新、持续类产品），
默认模式下省去刷屏。

**Q5：问题列表里的中文名是哪来的？会不会翻译错？**
`logger_i18n.h` 是 **issue code → 中文名**的纯查表适配（六模块 `*IssueCodes.h` 全量注册表：
AR/EOS/ESR/RIR/SAR/SBIRS）。
它**不翻译、不解析 message**（规则 13b：message 不承诺稳定），量值一律走 DebugView 结构化
字段；未知 code 自动回退英文 message 原文。

**Q6：视图模式下"周期=N 无状态变化"算一行吗？**
算。`LogViewSummary` 每调用一次计一行，冒烟断言按"每周期 ≥ 1 行"（AR/EOS/SBIRS）、SAR
"每周期 == 1 行"。所以 delta 模式下变化多的周期行数 > 周期数，全静态周期也至少 1 行。

**Q7：排除原因变化（`exclusion_cause`）和视图里的"问题列表"是什么关系？**
视图问题列表（`FormatIssueText`）是**本周期**瞬时诊断（13b，`LastDebugView()` 里那份）；
`exclusion_cause` 事件是**跨周期差分**诊断（13e，`*ExclusionCauseRecorder`）。前者回答
"这周期它为什么探测不到"，后者回答"原因从什么变成了什么"。差分记录器正是靠视图同源的
`result.issues`（按 `location.kind == kSceneEntity` 关联实体）做原料，但两者一静一动，
分别落在 `integration_views.log`（视图行）与 `integration_events.log`（事件行）两个文件。

**Q8：库会不会再写一份 JSON / FlexBuffers 观测文件？**
不会。周期落盘只有 Replay（`*RecordingSession` + `ReplayTraceWriter`）。调用方若要把
DebugView 存成自己的 JSON，那是集成方文件，不是库可观测性 API。

---

## 11. 关键文件索引（想深入时按此查）

| 想看什么 | 看哪 |
| --- | --- |
| 可观测性单源（周期记录 / 投影 / Replay / PROJECT_LOG） | `docs/common/session_contract.md` §可观测性单源 |
| 分层周期记录（规则 15） | `docs/common/session_contract.md` §分层周期记录 + 可选投影 |
| 日志设施主体与宏定义 | `examples/logger/logger.h`（`CA_LOG_EVENT*`/`CA_LOG_VIEW`） |
| 模式选择区（宏兜底默认值） | `examples/logger/logger_modes.h` |
| issue code → 中文名查表 | `examples/logger/logger_i18n.h` |
| 视图三模式落盘分支实现 | 各组件 `LogDebugView()`（`ar/eos/sbirs_sensor_component.cpp`） |
| 排除原因差分事件（13e）库头与示例 | `include/1q/<module>/session/*ExclusionCauseRecorder.h` + 各组件 `Step` 末尾的 `exclusion_.GetLastEvents()` 循环 |
| 事件三模式实现（聚合/计数） | `examples/logger/logger.cpp` |
| 模式 CMake 门控 | `examples/CMakeLists.txt` |
| 场景数据与预期事件表 | `examples/scenes/README.md` |

## 变更规则

- 本教程是导航文档：三模式语义、宏名、文件布局变更时须同步本文与 `logger/README.md`。
- 新增视图/事件类型（如新组件）时，同步 §3 事件类型表与 §6 例外说明。
- 教程不复制契约原文，只做指引；契约改动以 `session_contract.md` 为准。
