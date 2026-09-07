---
Status: final
Date: 2026-09-06
Completed: 2026-09-07
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
8、设备号层级整体废除（每模块唯一 platform_id；发射身份=(模块号, 发射流水号)；共址改显式隔离配对表），是否成立。

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
| 8、设备号层级整体废除（每模块唯一 platform_id）是否成立 | 本模型无任何模块建模多发射机/多接收机，设备号层无可归职责：其三项现职（自剔除区分、共址配对、实体关联键）均可由"每模块唯一号"等价或更好地承担 | include/1q/airborne_radar/config/ArHardwareConfig.h（transmitter/receiver 均为单实例结构体，无多设备容器）；include/1q/remote_identification_radar/config/RirHardwareConfig.h（同）；include/1q/electronic_countermeasure/EcmTypes.h::EcmSessionConfig（单发射机，channel_count 为单机的频通道数非设备数）；include/1q/electronic_surveillance_radar/config/EsrHardwareConfig.h（单接收机）；examples/core/rf_world_broker.h::BuildArInterferenceFromRfWorld（自剔除改为模块号精确匹配即天然区分同机 ECM，雷一根除）；src/common/electromagnetics/RfScene.cpp（非共址且 path_length_m < minimum_far_field_range_m 时拒解——同载体漏配隔离对的物理兜底存在） | grep 配置结构（2026-09-06 执行）：四模块收/发配置均为单实例结构体；矩阵项 1/2 的撞号雷全部发生在设备号层 | 无多设备消费方 + 三项职责均有等价承担 + 撞号雷随层消失 → 层级可废 | 发现任一模块真实建模多发射机/多接收机，或发射/接收同体模块必须区分自发自收链路的机制性需求 | pass |

## §2 判定汇总与待裁定问题

判定汇总：
1、项 1-5 建议 pass：撞号、无查重、平台号四源分立、散装接线均为实证事实，统一编号体系需求成立。
2、项 6 原"不动共享 RF 层数据模型"的范围收窄结论已被修订 2 推翻（模型自洽不等于层级必要），设备号层入废除范围。
3、项 7 建议 pass：纯收/纯发模块单设备号现状已符合设计原则——设备号层废除后此原则自然升级为"每模块单 platform_id"。
4、项 8 建议 pass：设备号层级无可归职责，废除后撞号雷（项 1/2）根除、自剔除变精确匹配、共址配对更直观。
5、全部判定依据的探针已于 2026-09-06 实际执行，结果写入 §1。

需要用户拍板的问题（处理状态）：
1、设备号层整体废除的方向确认（项 8）：每模块唯一 platform_id；发射身份=(模块号, 发射流水号)；四模块配置删设备号字段——待拍板。
2、共址触发方式：隔离配对表存在即共址（同载体漏配时由近场门物理兜底、链路拒解）vs 保留"同载体"分组仅作校验（漏配显式报错而非静默拒解）——待拍板。
3、每模块 platform_id 的配置面：场景级身份清单单源（模块配置从清单接线）vs 各模块会话配置保留单字段+场景层查重——待拍板。
4、RIR platform_id 迁移与 runner 手拼共址路径（ECM→AR 100dB）收编场景文件——并入上述拍板后随本轮统一处理——待拍板。

## §3 冻结契约（用户讨论结束后填写）

已证明的需求（修订 4 后的最终口径）：
- 设备号层级无可归职责（§1 项 8），且是同平台撞号雷的载体（§1 项 1/2）。
- RF 身份与标注身份合一：传感器实体 ID（SetSensorEntityId 注入，uint32）同时承担发射身份、自剔除、共址配对与日志/验收/融合标注——全系统每个模块一个号（修订 4）。
- 共址触发 = 接收方隔离表命中发射方实体 ID；漏配走远场求解、近场门拒解兜底（修订 3 拍板 2，维持）。
- 不新增任何身份类配置字段：注入面 = 既有 SetSensorEntityId + 场景顶层 device_ids 块（修订 4）。

允许范围：
- 共享 RF 层（include/1q/electromagnetics/RfScene.h、src/common/electromagnetics/RfScene.cpp）：
  1、RfEmissionIdentity 收缩为 {sensor_entity_id(uint32), emission_id}。
  2、RfSceneReceiverState 删 equipment_id，platform 字段改 sensor_entity_id(uint32)。
  3、RfCoSiteIsolationPath 收缩为 {transmitter_sensor_entity_id, isolation_db}。
  4、TryEvaluateRfIncidentLink 共址判定 = 接收方隔离表命中发射方实体 ID；未命中走远场；近场门保留；共址时延置零保留。
- 四模块公共配置头（只删不增）：
  1、AR：ArHardwareConfig 发射/接收 equipment_id 删；ReceiverConfig::co_site_paths 端点改对方实体 ID。
  2、ESR：EsrHardwareConfig::receiver_equipment_id 删；EsrCoSiteIsolationPath 端点改对方实体 ID。
  3、ECM：EcmSessionConfig 删 transmitter_equipment_id。
  4、RIR：RirSessionConfig::platform_id 字段删；RirHardwareConfig 发射/接收 equipment_id 删。
  5、无任何新增配置字段。
- 周期输入：ArPlatformInput、EsrCycleInput、EcmTypes.h 周期输入结构删 platform_entity_id（位置/速度每拍注入保持）。
- 身份注入面（唯一）：
  1、AR/ESR/EOS/RIR 既有 SetSensorEntityId 语义扩展：有效值（非 0）同时供 RF 组装（发射身份、接收状态、自剔除、共址表归属）与标注面；0=恢复缺省哨兵对两域同义；下一周期生效对两域同义。
  2、ECM 新增 EcmSession::SetSensorEntityId（同语义；库内 WaveformFactory/校验随迁）。
  3、session/controller 已持的 sensor_entity_id 升格为身份源（ArSession::Impl、EsrSession::Impl、RirController::sensor_entity_id_ 等既有成员复用）。
- issue codes：删 ar.validation.equipment_identity_invalid、rir.validation.equipment_identity_invalid、rir.validation.platform_id_invalid；不新增（0 哨兵不报错、有效值恒非零）。
- 回放 schema（字段增删/类型变更 → 标识符升位 + 旧标识符拒绝测试）：airborne_radar_replay（ARC3）、airborne_radar_session_replay（ARSC）、ecm_replay（ECMC）、esr_replay（ESRC，实体键三元组→二元组）、esr_session_replay（ESRS）、rir_session_replay（RIRF，含 platform_id 删除）、rir_replay（RIR3 随周期输入结构核查）。
- 示例层：
  1、场景顶层 device_ids 块新增 "ecm" 键（缺省 101）；查重覆盖全部键（含 satellites 撞号，既有机制扩展）。
  2、runner：ECM 组件接 device_ids.ecm 并调 setter；RIR 组件删 platform_id 入参；AR/ESR 组件平台号常量/字面量删除；ECM 启用时共址路径 push 端点 = ECM 实体 ID。
  3、rf_world_broker：自剔除改实体 ID 单匹配（删 equipment 参数）。
  4、场景 JSON：删 platform_id / transmitter_equipment_id / receiver_equipment_id 等键。
  5、删 kPlatformEntityId / kEcmTransmitterEquipmentId 常量。
- 测试与文档：
  1、上述变更的单元/契约/回放测试；共址命中/未命中+近场门拒解、device_ids 查重、升位标识符拒绝测试转正。
  2、上一轮五模块 boundaries 的「不进入物理计算」边界改写为「同号贯通标注与 RF 身份」。
  3、rf_architecture.md 四级 provenance 改三级（周期/实体/发射）；docs/common/contract.md RF 契约段；issue_codes.md；COMMON-OQ-13 按合一口径收敛。

明确禁止范围：
- 不新增任何身份类配置字段（修订 4）。
- 不引入"同载体/编组"概念、跨模块 ID 基类（修订 3 拍板 2）。
- 不动 EOS/SBIRS 的身份机制（非 RF 参与方，标注面不变）与其 schema；不动 SAR。
- 不做字段过渡/deprecated 兼容层（未交付项目）。
- 链路物理语义除身份字段与共址触发外零改动（极化/波形/损耗/近场门阈值不动）。
- 融合 source_weights 数组机制不动（缺省权重 1.0 兜底，大号差异化配权由集成侧映射——既有裁定）。

行为边界：
- 输入：传感器实体 ID = 非零 uint32；0=恢复缺省；缺省 AR=1/ESR=2/EOS=3/SBIRS=4/RIR=5/ECM=101；场景 device_ids 查重（重复报错，含与 satellites[].source_id 撞号）。
- 输出：发射身份=(实体 ID, 发射流水号)；ESR 实体关联键与差分键=(实体 ID, 发射流水号)；验收行标注沿用有效实体 ID（RIR 雷达ID=5 口径既有）。
- 共址语义：隔离表命中=共址（查隔离值、时延置零）；未命中=远场求解；同载体漏配由近场门拒解兜底。
- 运行期改号：setter 任意时刻可调、下一周期生效（对 RF 与标注同义）；改号后历史发射（旧 ID）不再被自剔除——集成方责任，文档明示。
- 回放：标识符升位，旧录制显式拒绝；无兼容负担。

爆炸半径与回滚：
- 下游消费方：集成方配置面纯删除（设备号/platform_id 字段消失，无新增）；全部旧回放录制失效。
- 回退难度：破坏性（公共字段删除+schema 升位+身份语义合一）；回滚=整分支回退。

验收门：
- 构建：llvm-ninja-release-local 全量通过。
- 聚焦测试：unit::airborne_radar、ESR 契约与单测、unit::remote_identification_radar、unit::electro_optical_sensor、unit::examples、全部 replay 套件、guard 全过。
- 场景冒烟：ecm_esr_driven_jamming（ECM 干扰进 AR、受扰指标不回归）、rir_ground_site_recognition（764 确认基线）、baseline_takeoff_east。
- 探针转正：共址命中/未命中+近场门拒解语义测试；device_ids 查重（含 ecm 键）测试；各升位标识符拒绝测试。

非目标：
- 多发射机/多接收机建模能力（裁定不存在该需求）。
- EOS/SBIRS/SAR 的 RF 身份（非参与方）。
- 融合 source_weights 容器化（既有否决）。
- 近场门阈值与隔离值标定。

## 修订记录

- 修订 1（2026-09-06，用户指令）：四模块经共享 RF 基础设施互相感知与博弈（ECM 纯发/ESR 纯收/AR 与 RIR 收发一体）；每个模块以平台号+设备号在 RF 空间标识；纯收与纯发模块的 RF 标识即其设备号；当前 RF 标识号使用混乱，要求重新设计模块交互逻辑的身份体系。用户同时否决「平台号/设备号粒度天然不对齐、维持现状」的辩护方向。
- 修订 2（2026-09-06，用户质询）：用户指出本模型不可能存在每模块多发射机/多接收机，质疑设备号层级的意义，方向裁定为"全部使用 platform_id（每模块单号）"。据此新增项 8（设备号层整体废除，建议 pass）；项 6 的范围收窄结论（不动 RfScene 数据模型）被推翻——模型自洽不等于层级必要，设备号层属未使用通用性且是项 1/2 撞号雷的载体；§2 拍板问题随之重构。
- 修订 3（2026-09-06，用户拍板）：三问裁定——1、设备号层整体废除，每模块唯一 platform_id（项 8 方向确认）；2、共址触发=接收方隔离配对表命中发射方模块号，漏配走远场求解并由近场门拒解兜底，不引入"同载体"分组概念；3、配置面=各模块会话配置各留一个 platform_id 字段（不建场景级清单；场景装载层做 RF 参与方模块号查重）。随之：RIR platform_id 保持 session_config 原地即目标形态（不再迁移）；runner 共址路径 push 机制保留、端点改模块号；runner/scene_data 中的设备号补丁（强制 101 等）随设备号层一并删除。
- 修订 4（2026-09-06，用户拍板，推翻修订 3 拍板 3 与上一轮"两域正交"裁定）：**RF 身份直接复用 SetSensorEntityId 的传感器实体 ID（uint32）**——同一个号用于发射身份、自剔除、共址配对与全部标注面（日志/验收/融合通道）。不新增任何 platform_id 配置字段；RIR 的 RirSessionConfig.platform_id 字段删除（身份=SetSensorEntityId 有效值）；每拍输入 platform_entity_id 删除；ECM 补设 setter 并纳入场景 device_ids 块（新增 "ecm" 键，缺省 101）。缺省=融合常量（AR1/ESR2/EOS3/SBIRS4/RIR5）+ ECM=101；场景 device_ids 查重覆盖全部键。修订 3 拍板 1/2（设备号层废除、隔离表命中制）不变。

## §4 运行记录

1. **实现范围**：
   - 共享 RF 层：`include/1q/electromagnetics/RfScene.h`、`src/common/electromagnetics/RfScene.cpp`（`RfEmissionIdentity` 收缩为 `{sensor_entity_id, emission_id}`；`RfCoSiteIsolationPath` 收缩为 `{transmitter_sensor_entity_id, isolation_db}`；`RfSceneReceiverState` 使用 `sensor_entity_id`；`TryEvaluateRfIncidentLink` 共址判定改为隔离表精确命中发射方实体 ID，未命中走空间远场，近场门保留）。
   - 四模块配置与输入面：
     - AR：`ArHardwareConfig.h`（删除发射/接收 `equipment_id`，`co_site_paths` 端点改对方实体 ID 并以 1U 为自隔离缺省）；`ArPlatformInput.h`（删除 `platform_entity_id`）；`ArSession.h` / `ArSession.cpp`（`SetSensorEntityId` 升格为 RF 发射、接收与自剔除单一权威源）；`ArReceiverStateBuilder.cpp`（自隔离路径随 `sensor_entity_id` 自动归属）。
     - ESR：`EsrHardwareConfig.h`（删除 `receiver_equipment_id`，`EsrCoSiteIsolationPath` 端点改对方实体 ID）；`EsrCycleInput.h`（删除 `platform_entity_id`）；`EsrSession.h` / `EsrSession.cpp`（`SetSensorEntityId` 贯通接收状态与同平台自剔除）；`EsrExclusionCauseRecorder.h`（实体键收缩为 `{sensor_entity_id, emission_id}`）。
     - ECM：`EcmTypes.h` / `EcmSession.h`（删除 `EcmSessionConfig::transmitter_equipment_id` 与 `EcmCycleInput::platform_entity_id`，新增 `EcmSession::SetSensorEntityId`；库内 `EcmWaveformFactory` 与校验随迁）。
     - RIR：`RirSessionConfig.h`（删除 `platform_id` 字段）；`RirHardwareConfig.h`（删除发射/接收 `equipment_id`）；`RirSession.h` / `RirSession.cpp`（`SetSensorEntityId` 贯通 RF 发射、接收状态与同平台排除）；`RirReceiverStateBuilder.cpp`（自隔离路径自动绑定 `sensor_entity_id`）。
   - 校验代码（Issue Codes）：
     - 删除 `ar.validation.equipment_identity_invalid`、`rir.validation.equipment_identity_invalid`、`rir.validation.platform_id_invalid`。
   - 回放 Schema 升位与旧标识符拒绝测试：
     - `airborne_radar_replay.fbs`（ARC4）
     - `airborne_radar_session_replay.fbs`（ARSD）
     - `ecm_replay.fbs`（ECMD）
     - `esr_replay.fbs`（ESRD）
     - `esr_session_replay.fbs`（ESRT）
     - `rir_replay.fbs`（RIR4）
     - `rir_session_replay.fbs`（RIRG）
     - 各套件旧 identifier 显式拒绝测试全部转正。
   - 示例与场景装配层：
     - 场景 JSON 顶层 `device_ids` 块统一支持 `ecm` 键（缺省 101），`scene_data.cpp` 覆盖全量键查重（含与 satellites 撞号）；
     - `runner.cpp` / 组件装配层移除所有平台号/设备号常量（`kPlatformEntityId` 等），统一通过 `SetSensorEntityId` 注入；
     - `rf_world_broker.h` 简化为实体 ID 单匹配自剔除。
2. **验证命令与结果**：
   - 全量编译构建：`cmake --build --preset examples-release-local`（pass，55/55 目标成功）
   - 全量自动化测试：`ctest --preset examples-release-local --output-on-failure -j 4`（pass，65/65 测试通过，100%）
   - 场景冒烟验证：
     - `build/examples-release-local/bin/ecm_esr_driven_jamming`（pass，120 周期完整执行，ECM 干扰正常驱动 AR）
     - `build/examples-release-local/bin/rir_ground_site_recognition`（pass，1560 周期完整执行，`rir_confirmed_cycles=764` 与基线严格吻合）
     - `build/examples-release-local/bin/baseline_takeoff_east`（pass，400 周期完整执行）
   - 编码规范与 BOM 检查：
     - `python3 scripts/utf8_bom.py check`（pass，全部 C/C++ 源文件均带 UTF-8 BOM 且为合法 UTF-8）
3. **权威回写去向**：
   - 1、正向边界：
     - `docs/airborne_radar/boundaries.md`（SetSensorEntityId 边界更新为同号贯通标注与 RF 身份，废除每拍 `platform_entity_id`）
     - `docs/electronic_surveillance_radar/boundaries.md`（SetSensorEntityId 边界更新为同号贯通标注与 RF 身份）
     - `docs/remote_identification_radar/boundaries.md`（SetSensorEntityId 边界更新为同号贯通标注与 RF 身份，废除静态 `platform_id` 配置）
     - `docs/common/rf_architecture.md`（四级 provenance 改为三级：world cycle, sensor entity, emission；隔离表命中与近场门规则更新）
   - 2、否决记录：
     - `docs/common/open_questions.md`::COMMON-OQ-13（登记闭合，明确设备号层废除与单一实体 ID 贯通全域）
     - `docs/common/issue_codes.md`（删除已废止的三项 equipment/platform identity issue codes）
   - 3、开放议题：
     - `COMMON-OQ-13` 登记已闭合（Closed）。
   - 4、证据锁：
     - [evidence: include/1q/electromagnetics/RfScene.h]::RfEmissionIdentity
     - [evidence: include/1q/electromagnetics/RfScene.h]::RfCoSiteIsolationPath
     - [evidence: include/1q/airborne_radar/session/ArSession.h]::SetSensorEntityId
     - [evidence: include/1q/electronic_surveillance_radar/session/EsrSession.h]::SetSensorEntityId
     - [evidence: include/1q/electronic_countermeasure/EcmSession.h]::SetSensorEntityId
     - [evidence: include/1q/remote_identification_radar/session/RirSession.h]::SetSensorEntityId
     - [evidence: tests/unit/common/common_rf_scene_test.cpp]::CoSiteExactHitBypassesNearFieldAndAppliesIsolation
     - [evidence: tests/unit/common/common_rf_scene_test.cpp]::CoSiteMissFallsBackToFarFieldAndTriggersNearFieldRejection
     - [evidence: tests/unit/examples/scene_data_test.cpp]::DuplicateDeviceIdsAreRejected
     - [evidence: tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp]::CycleRecordRejectsRetiredARC3FileIdentifier
4. **残留风险**：
   - 历史 FlatBuffers 回放录制文件因 schema 升位不可向前兼容（按契约显式拒绝旧标识符，属预期内非兼容升级）。
5. **后续冻结项**：
   - 无。本主题契约已全闭合。
