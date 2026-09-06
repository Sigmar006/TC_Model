---
Status: draft
Date: 2026-09-06
Review-Baseline: `evidence/rf-identity-unification` @ `1320364e`
Authority: 过程脚手架记录（非耐久）；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准；
  权威回写完成、合并进 main 前移除本文档。
---

# rf-identity-unification：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

触发来源：用户 2026-09-06 指令——四个传感器/对抗模块经共享射频基础设施互相感知与博弈，RF 标识号使用混乱，要求重新设计模块交互逻辑中的身份体系；其中纯接收与纯发射模块的 RF 标识就是其设备号。
- **证据**：[evidence: include/1q/electromagnetics/RfScene.h]（共享 RF 基础设施）
- **证据**：[evidence: examples/core/rf_world_broker.h]（模块间汇集/分发的编排层）

术语白话（全文沿用）：
1、RF 空间：`oneq::electromagnetics` 提供的共享射频世界——各模块把自己的发射事实（波形、功率、位置）放进去，也从里面算"别人打到自己接收机上的信号有多强"。
2、平台号（platform_id）：同一物理载体（一架飞机/一个地面站）的编号。两个设备平台号相同，它们之间的信号按"共址耦合"记账（载体内部的泄漏隔离）；不同平台按空间传播记账。
3、设备号（equipment_id）：一台具体发射机或接收机的编号。共址隔离路径按"发射设备号 → 接收设备号"配对查表。
4、发射号（emission_id）：一次具体发射的流水号，同一帧内必须唯一。
5、共址（co-site）：同平台设备间的电磁耦合，判定条件就是"发射方平台号 == 接收方平台号"。

模块角色（用户给定模型）：
| 模块 | 角色 | RF 身份需求 |
|---|---|---|
| ECM（干扰机） | 纯发射 | 一个发射设备号 |
| ESR（电子侦察） | 纯接收 | 一个接收设备号 |
| AR（机载雷达） | 收发一体 | 平台号 + 发射设备号 + 接收设备号 |
| RIR（识别雷达） | 收发一体 | 平台号 + 发射设备号 + 接收设备号 |

当前填号现状（乱象全景，证据见矩阵）：
| 模块 | 平台号来源 | 发射设备号 | 接收设备号 |
|---|---|---|---|
| AR | 示例常量 kPlatformEntityId=1，每拍注入 | 硬件配置默认 1 | 硬件配置默认 2 |
| ESR | 组件内字面量 1U | —（不发射） | 硬件配置默认 1 |
| ECM | 组件内常量 1 | 库配置默认 1；示例层强制改 101 | —（不接收） |
| RIR | 会话配置字段（场景写 2） | 硬件配置默认 1 | 硬件配置默认 2 |

待裁定项（需求/风险是否成立；四问由 §1 矩阵列承载）：
1、ECM 库默认发射设备号与 AR 发射设备号同平台撞号，AR 自剔除会静默吞掉 ECM 干扰，是否成立。
2、ESR 接收设备默认号与 AR 发射设备号同平台撞号（违反"同平台内设备号唯一"的注释契约），是否成立。
3、"同平台设备号唯一"没有任何机制校验（库校验只查非零），是否成立。
4、平台号四种挂法（配置字段/常量/字面量/每拍参数）、"同机必须同号"无机制保障，是否成立。
5、跨模块身份接线已散装存在（示例层强制 ECM=101、runner 手拼共址路径），集中为场景级身份清单是否成立。
6、共享 RF 层的身份数据模型（四级身份+共址判定+设备号对隔离）本身自洽、无需改动，是否成立（范围收窄裁定）。
7、纯收/纯发模块"RF 标识=其唯一设备号"现状已满足、无需新增字段，是否成立。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| 1、ECM 默认发射设备号与 AR 发射设备号同平台撞号，AR 静默吞掉 ECM 干扰 | 库默认值下（ECM=1、AR 发射=1、同平台 1），AR 的干扰派生过滤条件同时命中 ECM 发射，干扰被静默剔除 | include/1q/electronic_countermeasure/EcmTypes.h::EcmSessionConfig::transmitter_equipment_id（默认 1）；include/1q/airborne_radar/config/ArHardwareConfig.h::TransmitterConfig::equipment_id（默认 1）；examples/core/rf_world_broker.h::BuildArInterferenceFromRfWorld（过滤条件=平台相等且设备号相等即剔除）；src/electronic_countermeasure/EcmWaveformFactory.cpp（发射身份设备号取自 EcmSessionConfig） | grep 全链（2026-09-06 执行）：ECM 发射身份=(输入平台号, 配置设备号)；broker 过滤双匹配；示例层 scene_data 把 ECM 缺省强制为 101 避开库默认 1（绷带证据）；库默认组合下过滤必命中 | 撞号链完整且当前仅靠示例层覆写避开 → 统一编号+查重需求成立 | 库默认不撞号，或过滤条件含其他区分键 | pass |
| 2、ESR 接收设备默认号与 AR 发射设备号同平台撞号 | 同平台两设备（AR 发射链与 ESR 接收链）默认同为 1 号，违反两处配置注释声明的"同平台内必须唯一"契约 | include/1q/electronic_surveillance_radar/config/EsrHardwareConfig.h::receiver_equipment_id（默认 1）；include/1q/airborne_radar/config/ArHardwareConfig.h::TransmitterConfig::equipment_id（默认 1，注释"同平台内必须唯一"）；include/1q/electronic_surveillance_radar/session/EsrExclusionCauseRecorder.h（实体关联键=(platform,equipment,emission) 三元组） | grep（2026-09-06 执行）：ESR 接收设备号进 RfSceneReceiverState；ESR 差分记录器以三元组为实体键——同平台同号使两个物理设备在键空间不可区分 | 同平台同号成立 → 需重新分配默认号并查重 | 两设备不同平台或键空间无碰撞消费方 | pass |
| 3、"同平台设备号唯一"无机制校验 | 库校验只查单设备非零/合法，任何一层都不做跨设备查重，撞号只能靠人眼 | src/electronic_surveillance_radar/session/EsrSessionConfigValidation.cpp（仅校验路径端点非零/不与自身接收号相同）；src/airborne_radar/session/ArSessionConfigValidation.cpp（仅校验共址路径结构合法）；src/electronic_countermeasure/EcmConfigValidator.cpp（仅校验设备号非零）；examples/scenes/scene_data.cpp（场景装载无设备号查重） | grep（2026-09-06 执行）：四个模块校验器均不感知兄弟模块；场景装载层无设备号查重逻辑 | 无任何查重点 → 需新增编排层查重 | 任一层已有同平台查重 | pass |
| 4、平台号四种挂法，同机同号无机制保障 | 换任何一处平台号其他处不跟随；共址判定=平台相等，失配使同机信号静默走错账（共址↔空间传播） | include/1q/remote_identification_radar/config/RirSessionConfig.h::platform_id（配置字段）；examples/components/ecm_sensor_component.h::kPlatformEntityId（常量，AR/ECM 共用）；examples/components/esr_sensor_component.cpp（字面量 1U）；include/1q/airborne_radar/session/ArPlatformInput.h::platform_entity_id（每拍参数）；src/common/electromagnetics/RfScene.cpp::TryEvaluateRfIncidentLink（is_co_site=平台号相等） | grep（2026-09-06 执行）：四种来源互不引用、无一致性校验；共址判定唯一依据平台号相等 | 四源分立成立 → 平台号需单源化 | 存在单一权威来源或一致性校验 | pass |
| 5、跨模块身份接线已散装存在，集中为场景级清单成立 | 示例层已经在做跨模块号码接线（强制 ECM 缺省 101、runner 用 ECM 号+AR 号手拼共址路径），只是散在两处且不在场景文件里 | examples/scenes/scene_data.cpp::LoadSessionConfigs（ECM 缺省强制 101U）；examples/app/runner.cpp（ecm_enabled 时向 AR 接收配置 push 共址路径{ECM 发射号, AR 接收号, 100dB}）；examples/scenes/README.md（ECM 基线 101 已成文档口径） | grep（2026-09-06 执行）：runner 是唯一同时看见全部模块配置的位置；共址路径端点号取自两处不同配置盒 | 跨模块接线需求真实存在 → 场景级身份清单（单源）有落点雏形 | 接线逻辑不存在或可各模块独立解决 | pass |
| 6、共享 RF 层身份数据模型自洽，无需改动 | 四级 provenance（周期/平台/设备/发射）+共址判定（平台相等）+设备号对隔离已覆盖模块交互需要；乱在填号侧不在模型侧 | include/1q/electromagnetics/RfScene.h::RfEmissionIdentity/RfSceneReceiverState/RfCoSiteIsolationPath；src/common/electromagnetics/RfScene.cpp::TryEvaluateRfIncidentLink；docs/common/rf_architecture.md（四级 provenance 规则已成文） | 通读 RF 层结构与求解器（2026-09-06 执行）：身份字段语义闭环，四个模块均按该模型填号 | 模型无缺陷证据 → 重设计范围收窄到"谁填号、号从哪来、怎么查重"，不动 RfScene 数据模型 | 发现模型本身表达不了交互需要（如缺平台分组字段） | reject（不改共享 RF 层） |
| 7、纯收/纯发模块"RF 标识=其唯一设备号"现状已满足 | ESR 只持 receiver_equipment_id、ECM 只持 transmitter_equipment_id，各一个号，与用户原则一致，无需新增字段 | include/1q/electronic_surveillance_radar/config/EsrHardwareConfig.h（仅 receiver_equipment_id）；include/1q/electronic_countermeasure/EcmTypes.h::EcmSessionConfig（仅 transmitter_equipment_id） | grep（2026-09-06 执行）：两模块各自只有单角色设备号字段，无死字段 | 现状满足 → 统一方案对纯功能模块保持单号 | 发现多角色字段或死字段 | pass（现状保持） |

## §2 判定汇总与待裁定问题

判定汇总：
1、项 1-5 建议 pass：撞号、无查重、平台号四源分立、散装接线均为实证事实，统一编号体系需求成立。
2、项 6 建议 reject：共享 RF 层数据模型不动，重设计落在"号的分配、来源与查重"侧。
3、项 7 建议 pass：纯收/纯发模块单设备号现状已符合设计原则，保持。
4、全部判定依据的探针已于 2026-09-06 实际执行，结果写入 §1。

需要用户拍板的问题（处理状态）：
1、身份清单形态：场景级「平台/设备身份清单」单源（模块配置从清单接线）vs 模块配置保持现状、场景层仅新增跨设备查重校验——待拍板。
2、库默认撞号值的处置：废弃库默认、强制场景显式给号 vs 重新分配库默认使其天然不撞——待拍板。
3、RIR platform_id 是否随本轮从 session_config 迁入统一清单（上一轮 COMMON-OQ-13 遗留项）——待拍板。
4、runner 手拼的共址路径（ECM→AR 100dB）是否随清单一并收编进场景文件——待拍板。

## §3 冻结契约（用户讨论结束后填写）

<!-- 一行一项：
1、允许范围：模块/目录、类/函数、测试与文档。
2、明确禁止范围：公开头文件、跨模块类型、schema/回放、测试阈值、兼容层。
3、行为边界：输入、输出、错误回退、生命周期。
4、爆炸半径与回滚：下游消费方影响、回退难度（无损/破坏性/回滚注意点）。
5、验收门：构建、聚焦测试、契约测试、特征化测试、探针转正（有回归价值的探针转正式测试）。
6、非目标。
-->

## 修订记录

- 修订 1（2026-09-06，用户指令）：四模块经共享 RF 基础设施互相感知与博弈（ECM 纯发/ESR 纯收/AR 与 RIR 收发一体）；每个模块以平台号+设备号在 RF 空间标识；纯收与纯发模块的 RF 标识即其设备号；当前 RF 标识号使用混乱，要求重新设计模块交互逻辑的身份体系。用户同时否决「平台号/设备号粒度天然不对齐、维持现状」的辩护方向。

## §4 运行记录（Stage C 后填写）

<!-- 对照强制回写清单勾项，全部完成才允许拆脚手架（见 SKILL.md 收尾规则）：
1、实现范围：改动文件与接口。
2、验证命令与结果：命令: pass/fail（含转正的探针测试）。
3、权威回写去向：
   1、正向边界：docs/<module>/design.md（boundaries/data-flow/algorithms 按归属选）。
   2、否决记录：docs/<module>/design.md 的"架构裁定与否决记录"专节（体裁见 docs-governance-standard）。
   3、开放议题：docs/common/open_questions.md 登记（编号）。
   4、证据锁：新增/修订的规则后附 - **证据**：[evidence: 路径]。
4、残留风险。
5、后续冻结项。
-->
