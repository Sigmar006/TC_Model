---
Status: draft
Date: 2026-10-04
Review-Baseline: `fosim_2013` @ `3ca6fa7b`
Authority: 过程脚手架记录（非耐久）；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准；
  权威回写完成、合并进 main 前移除本文档。
---

# rir-external-target-position-input：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

触发来源：需求方提出「雷达从外部获取目标大概位置接口，在集成层面把位置信息录入，雷达据此调整波束中心指向该位置」；明确不做模块内部推导，只由雷达提供对外接口。

- **证据**：[evidence: docs/remote_identification_radar/interfaces.md]（现有三组对外关系的权威表：干扰机 / 外部参数设置系统 / 场景目标）

**前置实测结论（本 Stage A 的输入，已执行）**

1、双星交会位置精度足够引导 4° 波束：σ=3 μrad、交会角 21.8° 时位置误差均值 375 m / P95 841 m，对应 300 km 上指向误差 0.655°（判据 <1.4°）。
- **证据**：[evidence: docs/review/ir_dual_sat_fix_accuracy_measurement_2026-10-04.md]
2、双星「每周期至少一条新视线」成立（80/80 周期），交会刷新率不构成瓶颈。
- **证据**：[evidence: docs/review/ir_cross_cue_link_measurement_2026-10-04.md]
3、主要误差源是测量时间偏斜而非测角精度：目标 1827 m/s，1 周期偏斜 1827 m，大于测角贡献 375 m。
- **证据**：[evidence: docs/review/ir_dual_sat_fix_accuracy_measurement_2026-10-04.md]

**待裁定项（每项回答：冻结什么 / 什么证据能证明 / 什么证据能否定 / 通过后最小改动范围）**

1、**现有公共 API 是否已足够表达「按位置改波束中心」**——若足够则需求降级为文档与集成示例，不新增接口。
2、**是否需要新增外部位置录入接口**——若不足，冻结新增接口的最小形状（坐标系、是否带时刻与不确定度）。
3、**主瓣覆盖门语义是否保持确定性硬门**——即指向误差导致目标出半功率宽时如实报排除、不做吸附。
4、**外部位置是否禁止取自仿真真值**——即引导位置不得来自归属层真值距离或真值播种的跟踪状态。
5、**外部指向是否必须进 replay 落盘**——即它是否属于改变检测结果的仿真输入。
6、**跨系换算的职责归属**——ENU 换算放库内还是集成层。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| 1、现有 API 已足够 | 运行期补丁 `has_scan_center` 可让波束中心指向外部给的位置，无需新接口 | include/1q/remote_identification_radar/config/RirRuntimeConfigPatch.h；include/1q/remote_identification_radar/config/RirMissionConfig.h | 读码核对：`has_scan_center` + `scan_center_deg` 直接改转台朝向；`scan_window_deg` 以 az 相对 `scan_center`、el 绝对定义 | 改中心后目标落入实际搜索扇区且不被裁掉 | `scan_window_deg` 相对中心定义 ⇒ 改中心会整体平移作战搜索扇区，牺牲原有覆盖 | **reject**（语义不满足：需求是"额外照一下"而非"改整机搜索基准"） |
| 2、需要新增接口 | 现有入口无法同时表达"指向某位置"且"保留原搜索扇区" | 同上；src/remote_identification_radar/runtime/RirController.h（`RirDwellPlan` 仅内部可见） | 读码核对：驻留计划 `RirDwellPlan`（指向 + 种类）为 `src/` 私有头，公共面无对应类型 | 公共面确无"独立指向且不影响搜索扇区"的入口 | 公共面已存在等价入口 | **pass**（缺口成立） |
| 3、主瓣覆盖门保持确定性硬门 | 外部指向照不到目标时应如实报排除，不吸附、不静默修正 | docs/remote_identification_radar/boundaries.md；include/1q/remote_identification_radar/session/RirIssueCodes.h | 读码核对：候选须落在本周期驻留指向半功率宽内才入候选，否则报 `rir.target_outside_beam_coverage` | 规则文本与 issue code 均已存在且非本次改动 | 库内存在"按目标位置重算/吸附指向"的路径 | **pass**（保持不动） |
| 4、禁止取自仿真真值 | 引导位置若来自归属层真值距离或真值播种跟踪状态，会构成仿真真值外泄 | docs/remote_identification_radar/boundaries.md；include/1q/sbirs_sensor/session/SbirsOutputTypes.h；src/sbirs_sensor/pipeline/SbirsTrackingCoordinator.cpp；docs/common/open_questions.md | 读码核对：`estimated_range_m` / `measured_range_m` 标注 sim-only；`InitializeTarget` 用场景真值位置/速度初始化滤波均值（SBIRS-OQ-4） | 契约层已禁止真值混入真实输出通道，新增接口须显式继承该禁令 | 真值字段被允许作为引导来源 | **pass**（写为显式禁止项） |
| 5、外部指向必须进 replay | 外部指向改变波束 ⇒ 改变检测结果 ⇒ 漏记会导致回放分叉 | include/1q/sbirs_sensor/session/SbirsExternalCue.h（反例）；schemas/replay/rir_session_replay.fbs | 读码核对：SBIRS 的 cross-cue 刻意不进快照；RIR 当前 session replay schema 存在但未含外部指向 | 存在明确的加性扩展点且不破坏字节兼容 | schema 无法加性扩展或需破坏性升级 | **defer**（需 Stage B 先冻结 schema 槽位与往返测试，见 §2） |
| 6、跨系换算归属 | ENU 换算放库内可实现且不违反既有纪律 | docs/common/contract.md；include/1q/remote_identification_radar/session/RirCycleInput.h；include/1q/remote_identification_radar/session/RirSceneTypes.h | 读码核对：`platform_position` 已以 ECEF 米制进入周期输入；场景目标为雷达局部 ENU | 库内已消费 ECEF 平台位置，入库时换系不引入新的外部状态 | 既有纪律明令禁止模块级换系入口 | **narrow**（可库内换系，但仅限"全局→本机 ENU"这一种，禁止扩为通用坐标适配器） |


## §2 判定汇总与待裁定问题

**建议判定（逐项一句话理由）**

1、待裁定项 1 —— **reject**：`has_scan_center` 是整机转台基准，改它会让 `scan_window_deg` 相对定义整体平移，属"改搜索基准"而非"额外照一下"，语义不满足需求。
2、待裁定项 2 —— **pass**：公共面确无"独立指向且不影响搜索扇区"的入口，`RirDwellPlan` 缝只在 `src/` 内可见，缺口成立。
3、待裁定项 3 —— **pass**：主瓣覆盖门与 `rir.target_outside_beam_coverage` 已存在且语义正确，保持不动即可。
4、待裁定项 4 —— **pass**：真值外泄禁令已在契约层，且库内确有 sim-only 真值字段（`estimated_range_m`）与真值播种跟踪链（SBIRS-OQ-4），必须显式继承该禁令。
5、待裁定项 5 —— **defer**：需先冻结 replay schema 的加性槽位与往返测试，本轮不预设结论。
6、待裁定项 6 —— **narrow**：允许库内做"全局坐标 → 本机 ENU"换系（`platform_position` 已以 ECEF 入库，先例成立），但明令禁止扩为通用坐标适配器或第二套位置入口。

**defer 项（待裁定项 5）下一步需要的探针**

1、读 `schemas/replay/rir_session_replay.fbs` 与 `rir_replay.fbs`，确认外部指向可作为加性字段落在哪个表与槽位。
2、构造"注入外部指向 → 录制 → 回放"探针，验证不加字段时是否真的分叉（若不分叉，则该待裁定项可降为 reject，无需落盘）。
- **推理**：若波束因外部指向改变而检测结果改变，则不加字段必然分叉；但分叉是否在现有比较口径（逐周期输出比较）中被捕获，尚未验证。

**需要用户拍板的问题**

1、**坐标系形态**：接口收"雷达局部 ENU 位置"（与 `RirSceneTarget::position_x/y/z` 同帧），还是收 ECEF 位置由库内换系（建议后者，见待裁定项 6）？
2、**是否携带时刻与不确定度**：只收三坐标，还是同时收"该位置对应的物理时刻 + 1σ"？实测表明时间偏斜是主导误差源，**不含时刻则无法评估可用性**。
3、**默认有效性窗口**：仅下个执行周期有效（逐周期重发），还是允许一次给 N 拍的显式窗口？
4、**与人工指定的优先级**：外部位置与 `designated_external_target_id` 冲突时谁优先（建议人工指定优先）？
5、**是否同时改转台朝向**：默认只改波位指向、保留搜索扇区；是否需要可选的"同时设 `scan_center`"开关？
6、**跨平台对齐职责**：确认"红外平台 → 雷达"的方向换算归集成层/融合层，库内不做（与 `contract.md` 的"跨源/跨平台坐标对齐归调用方"一致）？
7、**单镜筒容量是否需在接口层表达**：实测单星同时只能窄场跟踪一个目标；外部一次录入多个位置时的语义（全部接受但时间片轮转 / 拒绝超出容量）需裁定。

## §3 冻结契约（用户讨论结束后填写）

<!-- 用户裁定后填写，本轮未填。参照骨架注释的六项：允许范围 / 明确禁止范围 / 行为边界 /
     爆炸半径与回滚 / 验收门 / 非目标。 -->

## 修订记录

<!-- 编号条目（修订 1、修订 2……）：日期、裁定内容、来源（用户指令/新证据）；不静默改写既有条目。 -->

## §4 运行记录（Stage C 后填写）

<!-- 对照强制回写清单勾项，全部完成才允许拆脚手架（见 SKILL.md 收尾规则）：
1、实现范围：改动文件与接口。
2、验证命令与结果：命令: pass/fail（含转正的探针测试）。
3、权威回写去向：
   1、正向边界：docs/<module>/design.md（boundaries/data-flow/algorithms 按归属选）。
   2、否决记录：docs/<module>/design.md 的"架构裁定与否决记录"专节（体裁见 docs-governance-standard）。
   3、开放议题：docs/common/open_questions.md 登记（编号）。
   4、证据锁：新增/修订规则后附 - **证据**：[evidence: 路径]。
4、残留风险。
5、后续冻结项。
-->
