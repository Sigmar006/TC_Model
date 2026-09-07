# 跨模块契约

Status: active
Last-reviewed: 2026-09-03
Authority: common contract for all modules
RF-Interference-Architecture: frozen target; AR/ESR/ECM/RIR RF v2 implemented (per-module status in each design.md)

本文合并原顶层 public API customization、session config builder、分层周期记录+可选投影可观测性
（旧称两通道/三层）和文档治理契约。模块级文档不得与本文冲突。

## 证据优先开发模式

对算法、架构、模块内部优化、输出语义、配置语义、测试阈值和 public API 相关改动，默认采用
`.claude/skills/evidence-first-freeze-contract` 定义的证据优先模式。

强制规则：

1. 先判定，再契约，再实现——任何级别都不得省略"证据先于实现"。
2. 全流程门禁范围内的改动，Stage A 未得到 `pass` 或 `narrow` 判定时，不进入生产代码实现。
3. 全流程门禁范围内的改动，Stage B 前必须冻结实现契约，明确允许范围、禁止范围、行为边界、验收条件和非目标。
4. 实现只能覆盖已被证据证明的最小边界；不得借机扩大 public API、跨模块抽象、schema、Replay 落盘或兼容层。
5. 验收失败时回到证据矩阵重新拆分原因；不得通过放宽阈值、扩大 skip 或弱化测试制造通过。
6. 分级门禁（细则由 repo skill 维护）：
   a. 全流程门禁：算法、架构、模块内部优化、输出语义、配置语义、public API、测试阈值类改动——证据分支 + Stage A 文档 + 冻结契约 + feat 分支。
   b. 轻量内联门禁：零公共接口、零行为边界、零 schema、零测试阈值变更的模块内部小型改动（如私有辅助函数的局部整理）——不开独立文档，在提交信息内嵌迷你证据块（假设、探针结果、允许范围），证据仍须先于实现。
   c. 无门禁：机械格式化；不改变行为的措辞修正；失败行为与验收条件均已在用户请求中明确的小修复。
   d. 范围归属存疑时，一律按全流程门禁处理（就高不就低）。

具体 evidence matrix、契约模板、输出格式和回写要求由 repo skill 维护；公共契约规定分级门禁的存在与边界，防止轻量级被滥用为绕门通道。

## Public API 边界

默认 public API 只允许稳定门面和稳定 DTO：

- 模块聚合入口头。
- `*Session`，包括 `Create` / `CreateWithDiagnostics` 等静态创建入口。
- `*SessionConfig` 配置域（条件五域，见下节）和运行期 patch。

### 会话创建入口的非阻断语义

`*Session::Create` 与 `*Session::CreateWithDiagnostics` 的校验/构造语义必须遵守以下非阻断契约（AR/ESR/EOS/SAR/SBIRS 五模块已实现并经契约测试覆盖；RIR 于 2026-08 按同语义并入，`RirSession::CreateWithDiagnostics` 同为非阻断）：

1. `Create(config)` 是信任路径，不做配置校验。
2. `CreateWithDiagnostics(config, issues)` 是校验路径——**无论 @p issues 是否含有 error 都会构造并返回会话（非阻断）**；`issues` 为模块 `*IssueList`（统一问题列表模型，见 `docs/common/session_contract.md` 规则 14，config 域校验问题 code 为 `"<module>.validation.<snake_case>"`），仅为咨询性诊断输出，传入 `nullptr` 时仅构造会话、不写回。
3. 两入口均不会因校验失败而拒绝构造；当前不存在"校验失败即不构造"语义。
4. 调用方须据 `issues->empty()`（或 `HasValidationError`）自行决定后续处置。

不得在文档或实现中宣称校验失败会阻断会话创建，也不得让任一模块私自把 `CreateWithDiagnostics` 改为门禁语义（校验失败即不返回会话）；若未来确实需要门禁语义，应新增独立入口点承载，不得复用现有咨询性入口名。
[evidence: tests/contract/electronic_surveillance_radar/esr_public_api_convenience_test.cpp::CreateWithDiagnosticsReportsIssuesButStillConstructsSession]
[evidence: tests/contract/electro_optical_sensor/eos_public_api_convenience_test.cpp::CreateWithDiagnosticsReportsIssuesButStillConstructsSession]
[evidence: tests/contract/sbirs_sensor/sbirs_public_api_convenience_test.cpp::CreateWithDiagnosticsReportsIssues]
- `*CycleInput`、scene target/emitter/point target 等单周期输入 DTO。
- `*OutputFrame`、`*CycleResult` 等输出和结构化执行结果 DTO。
- Replay 落盘、debug view、lifecycle recorder 等已经形成外部消费合同的工具。

业务模块 public 类型使用模块所有权前缀：`Ar*`、`Eos*`、`Esr*`、`Rir*`、`Sar*`、`Sbirs*`。领域术语不受该规则机械约束，例如 `radar_cross_section`、`RadarEquations` 这类物理概念可保留领域名；但 session/config/cycle/result/adapter/replay/debug/lifecycle 等 public DTO 和门面不得把通用领域词误用为模块前缀。

默认禁止公开：

- pipeline/controller/context/environment service 等内部装配 seam。
- algorithm executor、focusing selector、calibration/focusing/truth oracle 等内部阶段。
- generated replay headers、内部 execution config、测试专用 mock 接口。
- 仅有单一生产实现且没有外部替换需求的虚接口。

唯一允许的用户自定义 SPI 是 `airborne_radar` 的 decision engine。其它模块默认只提供稳定 session 门面。

### 条件五域配置所有权

`*SessionConfig` 的配置域按“参数表达的用户意图”划分，而不是按当前内部消费类所在目录划分。

**基线四域**（所有有会话的传感器模块均具备）：

- hardware：发射机、接收机、天线、探测器、光学和波形等物理能力（不含静态安装欧拉/失准）。
- mission：工作模式、运行期扫描/驻留指向（含转台朝向 `scan_center_deg`）、任务目标与可热更新的扫描任务参数。
- policy：最低 SNR、虚警概率、脉冲积累数、检测 margin、关联门限等可由任务策略调整的判决规则。
- environment：场景介质、传播、地表和大气观测等外部环境语义。

**第五域 orientation**（仅当模块有静态安装/指向几何时）：

- 语义：Body→Sensor（或等效）安装角、扫描限位、稳定方式、安装失准（若建模）。
- 生命周期：初始化静态配置；**不得**进入 `*RuntimeConfigPatch`（与 SBIRS 范本一致）。
- 禁止空壳：无静态安装指向几何的模块不得添加仅占位的 `orientation{}`。

| 模块 | 配置域数 | orientation |
|---|---|---|
| SBIRS / AR / ESR | 五域 | 有（范本：SBIRS；AR 拆分静态/运行期后上提；ESR 瘦 mount az/el） |
| EOS / SAR | 四域 | 无（EOS 仅复用瞄准引擎姿态链；SAR 无该子域需求） |
| RIR | 五域 | 有（阵面相对可扫描体积 `steerable_volume_deg`；转台朝向在 mission.scan_center_deg） |

因此最低 SNR、Pfa 等探测门限不得因内部 signal detector 与 hardware 相邻而放入 hardware。跨模块的
同类判决参数应归 policy；静态 mount 不得因历史嵌在 mission/hardware 而继续错域。
模块可以保留不同物理算法和字段集合，但不能改变所有权含义。
[evidence: tests/unit/airborne_radar/ar_session_config_builder_test.cpp]
[evidence: tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp]
[evidence: include/1q/sbirs_sensor/config/SbirsSessionConfig.h]

### 物理量单位命名

公开标量字段必须在名称后缀中表达调用方实际提交的单位。跨模块同物理量只有在公式输入量纲和消费语义
一致时才统一单位；不得为了表面一致，在 public 边界隐藏换算或让字段退化为无单位名。

EOS 的 `detector_area_cm2` 与 `detector_detectivity_cm_sqrt_hz_per_w` 共同进入厘米制 D* / NEP 公式，
因此必须保留显式单位后缀，禁止退化成无单位 `detector_area`。SBIRS 当前没有具备焦距、像元几何与
视场立体角映射的成像链，因此不公开未消费的 detector-area 参数。
[evidence: tests/contract/check_cross_domain_naming.cmake]

## 跨模块物理基元复用

跨模块函数不得因名称或量纲相似而合并。只有输入单位、数值精度、有效/非法输入策略、几何归一化、环境/效率因子和输出失败语义完全同义时，才允许复用无状态纯函数；复用前必须有跨模块 characterization 测试。

当前 EOS/SBIRS 的冻结结论：

| 函数族 | 结论 | 原因 |
|---|---|---|
| Planck 光谱辐亮度 | 仅 EOS 保留，SBIRS 已删除 | SBIRS 目标签名改为调用方提供辐射强度（W/sr），无温度输入、不做 Planck 换算 |
| 接收功率 | 不合并 | EOS 接收孔径面积并除以 `4πr²`；SBIRS 接收孔径直径、使用 `P = I_t·A_ap·τ/d²` 口径和量子效率 |
| 噪声/NEP | 不合并 | EOS 是背景抑制与 NEP 链；SBIRS 是 photon/thermal/readout RMS 与兼容 NEP 回退 |
| 输出方位参考系 | 不合并 | SBIRS 输出 ECI 极坐标弧度（2026-08 正式变更：输入 ECEF + UTC 儒略日，周期入口按 GMST 旋转到 ECI；az∈[0, 2π)、el∈[-π/2, π/2]）；EOS/AR/ESR 保持各自平台局部系约定 |
| 距离输出 | 不输出 | SBIRS 被动红外不测距：raw output 无距离字段；`estimated_range_m` 仅为内部诊断且仅对归属目标回填；示例层不展示距离 |

这不是未来共享 foundation 的禁止令：新的候选必须先证明上述语义完全相同，且不得以转换器、默认值或兼容层掩盖差异。

### 工程 RF 契约

以下为 AR/ESR/ECM/RIR 公共 RF 事实域（`oneq::electromagnetics`）的跨模块契约条款。设计描述（provenance
三级、单周期交换时序、接收机影响分层）见 [rf_architecture.md](rf_architecture.md)。

1. 发射事实身份 = 传感器实体 ID（`sensor_entity_id`，各模块会话经 `SetSensorEntityId` 注入，uint32）+
   发射流水号（`emission_id`）；无设备子层。同一实体 ID 统一承担设备身份、待处理信号自剔除（按实体 ID
   精确匹配）与共址判定（接收方隔离表命中，不再由任何号码相等关系推导）。
2. 发射事实禁止携带 `received_power`、J/S、J/N、receiver impairment、`jamming_detected` 或成功概率。
3. AR/RIR 目标回波不得伪装成外部 `RfEmission` 后复用单程公式；外部雷达、ECM 和其他 RF 源才走公共单程链路。
4. 有效带外发射或零时频重叠产生零贡献，不是错误。非有限值、非法活动区间/波形、负功率、重复
   (sensor_entity_id, emission_id) 身份和不支持的近场路径必须 fail closed，且不得部分写回；未命中接收方
   隔离表的链路按远场空间传播求解，同载体近距离漏配由近场门（`path_length_m <
   minimum_far_field_range_m`）拒解兜底。
5. 超过已标定最大线性输入功率是合法物理结果：周期仍视为已执行并输出结构化 saturated impairment，
   但不得伪造观测；它与输入/配置非法导致的整周期拒绝严格分离。
6. 业务模块之间不直接调用；调用方只转交公共值类型，不需要创建 RF scene、回填 AR 自身发射、管理 token
   或调用 prepare/complete 状态机。
7. 非空 RF frame 必须与消费者周期窗口完全一致；空 frame 表示没有外部 RF 干扰，不要求虚构身份或 mode。
8. 输入拒绝不消费 emission ID、hop/PRI phase、随机流、待应用控制或跟踪状态；设备关机只推进世界 chronology。
9. 模块在返回成功结果时原子提交本周期发射、接收、检测和累积状态。

[evidence: tests/unit/common/common_rf_link_budget_test]
[evidence: tests/unit/common/common_rf_scene_test]

### RIR 驻留指向输入契约

RIR 的波束中心由**库内驻留调度器**（`RirSession`）每周期派生：common 内核在
`orientation.steerable_volume_deg`（阵面相对 az、绝对 el）上建波位，再经
`mission.scan_center_deg`（转台 ENU 朝向）平移并将方位归一化到 `(-180, 180]`；
无指定任务时按该序列逐周期推进；指定识别任务窗口内对准指定目标，但目标视线越出
「center + 体积」时驻留回扫描并报告 `kOutsideSteerableVolume`（任务保持
`kPending`，转台重新瞄准后可恢复）。RIR 消费侧只信任并消费给定值。指向角类型为
`RirAzimuthElevationDeg`（deg），雷达局部 ENU 右手系（与
`RirSceneTarget::position_x/y/z` 同帧）：`az_deg ∈ (-180, 180]`、`el_deg ∈ [-90, 90]`。
离轴方位差由调度/增益链路经 `NormalizeAzimuthDeltaDeg` 折算。指定识别任务见
`docs/remote_identification_radar/boundaries.md`。

### 场景目标平台锚点 ENU 输入契约

AR/EOS/RIR 的场景目标输入统一为平台锚点 radar-local ENU。SAR 为文档化例外（地面场景以
LLA 输入、库内使用 scene-center ENU 几何）；SBIRS 输入保持 ECEF/ECI；ESR 与公共 RF 帧保持
ECEF 全局几何，不适用本契约：

1. ENU 原点 = 当周期平台 ECEF 位置（逐周期重锚）；轴 = 锚点 ENU（x=东、y=北、z=天）。
2. 目标速度 = 目标 ECEF 速度旋入锚点 ENU 轴（固定锚点旋转，无传输率修正）。
3. ECEF/LLA→ENU 转换由公共入口一站式承担：`TryEcefToLla` 求锚点（每周期一次）+
   `TryMakeEnuSceneState` 逐目标转换后直填 `ArTargetInput` / `EosSceneTarget` /
   `RirSceneTarget`。**禁止**模块级场景目标 ECEF→ENU `*CycleInputAdapter` /
   `TryMake*FromExternal*` 平行入口；官方入口仅 `include/1q/coordinate/scene_transform.h`。
4. 各模块继续拥有自己的量测几何：AR 在 ENU 之后再旋入雷达体系（平台姿态∘安装角复合，
   见 `ArRadarFrameTransform`），EOS 由 ENU 位置 + 平台姿态派生体系球坐标（range/az/el），
   RIR 直接在 ENU 轴上计算。
5. ENU 原点逐周期随平台移动，跨周期不构成惯性参考系；跨周期状态（航迹/滤波）的帧语义由
   各模块内部拥有，不通过输入面传递。

集成样板（AR/EOS/RIR 相同）：

```text
TryEcefToLla(platform_ecef) → anchor_lla   // 每周期一次
for each target:
  TryMakeEnuSceneState(kinematics, anchor_lla) → 直填场景目标
Session::Step(CycleInput)
```

[evidence: tests/unit/common/coordinate_scene_transform_test]

### 折射率温标输入迁移

公开折射率入口只提供 `RefractivityInputs` + `RefractivityTemperaturePair` +
`TryRefractivityIndex`。摄氏与开氏字段必须满足
`kelvin = celsius + 273.15`（允许 0.05 K 浮点容差）；温标错配、非有限/越界标量或空输出必须
fail closed，且失败不得修改调用方输出。

[evidence: tests/unit/airborne_radar/ar_atmosphere_physics_test.cpp::TypedPublicRefractivityMatchesPhysicalKernel]
[evidence: tests/unit/airborne_radar/ar_atmosphere_physics_test.cpp::TypedPublicRefractivityRejectsMismatchedTemperaturePairAtomically]

### 核心运行面与观测工具面

public API 分为两类，二者都受 public boundary、install manifest 和 consumer 测试保护：

1. **核心运行面**：模块聚合入口、config、input、session、raw output 与 cycle result。它定义调用方驱动模型和消费仿真结果的稳定语义。
2. **观测工具面**：Replay 落盘、周期记录派生投影（debug view / lifecycle recorder /
   exclusion cause）及其结果 DTO。它用于诊断、复现和人读归属，不能反向改变核心运行面、
   raw output 或控制行为。库内周期持久化只允许 `ReplayTraceWriter`。

观测工具面的新增字段或事件必须保持分层周期记录与可选投影分离（规则 15），并同步
schema/codec、对应 replay 测试和 consumer 测试；删除或重命名已公开工具仍属于
public API 变更，必须先冻结兼容迁移契约。详见 `session_contract.md`「分层周期记录 + 可选投影」
与「可观测性单源」。

## 目标处理分层契约

传感器量测生产与目标域处理（估计/推演/决策）是两条职责线："探测到目标"（量测事实生产）与
"目标是什么/在哪/要去哪"（估计、推演、决策）分属不同层，禁止互相寄生。库内目标域能力按五层归属：

| 层 | 当前归属 | 职责 | 产品 |
|---|---|---|---|
| 传感器层 | AR / ESR / EOS / SAR / SBIRS / RIR | 量测事实生产；传感器自身行为的内部建模（指向闭环、检测门控、驻留调度） | `*OutputFrame` 量测记录 |
| 适配层 | `fusion::SensorAdapters` | 传感器输出 → 泛型 `fusion::DetectionRecord` | `DetectionRecord` |
| 估计层 | `fusion` + `src/common/estimation` | 多目标关联、航迹滤波、航迹管理（轨迹滤波为 fusion 冻结的预留演进项） | `FusedTarget` |
| 推演层 | `target_inference` | 轨迹预测、发射点/落点回推、目标类型识别分类 | 带误差预算的推演产品 |
| 决策层 | `threat_assessment` | 威胁评分与等级 | `ThreatResult` |
| 评估层 | `precision_evaluation`（2026-08-19） | 真值对照的定位精度误差提取（角度/双星交会/速度/落点/发射点）与 AHP 指标聚合 | 评估报告 + `[PrecisionEval]` 验收日志 |

规则：

1. **依赖方向单向**：传感器层不得引用估计/推演/决策层类型；算法面（fusion、
   threat_assessment、未来推演面）不得引用传感器具体类型，唯一允许依赖传感器具体类型的
   公共触点是 `SensorAdapters`。跨源/跨平台坐标对齐归调用方（session_contract.md 既有立场），
   适配层只做字段映射与单位换算，不做跨系转换。
2. **传感器产品边界**：威胁评分、目标类型识别结论、轨迹/发射点预测不得作为传感器 public
   输出字段（raw output、`*CycleResult`、public DTO 一致适用）。量测质量（SNR、quality）与
   量测特征（辐射强度、RCS 量测值、频段标注）不属此列。
   **识别类传感器双产品条款**（2026-08-18 Stage B 写入，文案冻结于
   `docs/review/rir_dual_product_stage_a_2026-08-18.md` §5）：以目标识别为装备使命的传感器
   （当前：remote_identification_radar）采用双产品形态——识别结论可作为装备使命产品出口，
   但**必须同时提供特征量测出口**（带库内键、单位后缀命名、逐维质量与有效掩码、并明示
   仿真保真度语义）；只输出结论不输出特征量测的形态不得新增。
3. **传感器内部目标处理的豁免**：为驱动自身闭环（波束/光轴指向、驻留调度、检测门控、丢锁
   判定）而维护的内部滤波、关联与生命周期状态是合法的传感器行为建模，须同时满足：不外发
   航迹/状态族估计产品（协方差、速度、生命周期语义的状态族），raw output 记录保持量测形态。
   SBIRS ATP 闭环、RIR 内部航迹链、AR STT 指向属此类。
4. **滤波原语单源**：航迹/状态滤波数值原语单源在 `src/common/estimation`；消费模块以
   facade / using 别名实例化，不得另写滤波原语。
5. **去真值化**（并入分层纪律）：目标域跨层记录只携带调用方关联键或引擎合成键，不携带场景
   真值标识；真值只允许进入仿真归属/调试层。
6. **误差预算出口**：估计层运动学产品与推演层产品必须携带不确定度（协方差/误差椭圆），只
   出口点估计不出口误差的产品不得进入 public API。被动体制（角度-only）的可观测性限制必须
   由产品如实承载，不得以真值辅助字段掩盖。
7. **存量偏离登记**：本契约生效前已冻结的公共 API 偏离登记于 `docs/common/open_questions.md`
   （TARGET-OQ-*），按证据优先模式逐项处置；处置完成前不要求追溯回改，但任何新能力不得新增
   同类偏离。
8. **变更规则**：估计层引入轨迹滤波或新关联度量按 fusion boundaries 变更规则 2 冻结实现边界；
   本分层规则变化必须同步本文与受影响模块 design 文档集，并评估是否需要 include 方向纯净度
   守护（当前 `tests/contract/` 无该守护，为已知空白）。
9. **评估层真值边界**（2026-08-19）：评估层（`precision_evaluation`）是去真值化规则 5 的唯一
   合法真值出口——可消费场景真值与各层产品（含仿真归属层）做误差对照，但**只产评估报告与
   验收日志，不回写、不下发任何产品层**；评估失败（如 AHP 矩阵非法）显式返回无效标志，
   不得静默退化。评估层自上而下单向依赖各层公开头，不进入任何传感器/估计/推演模块的依赖
   闭包；其验收日志由编译期开关 `ONEQ_ENABLE_PRECISION_EVALUATION_LOG`（默认 OFF）门控。

新增目标域需求的归属裁定与需求术语对齐背景见
[../review/target_domain_requirements_alignment_2026-08-17.md](../review/target_domain_requirements_alignment_2026-08-17.md)
（非权威草案，不得替代本文）。

## 内部共享命名空间

`src/common/` 同时容纳两类实现，目录位置本身不决定 C++ 命名空间：

- 已由 `include/1q/<domain>/` 公开的领域 API 实现使用对应 public 命名空间，例如
  `oneq::coordinate`、`oneq::replay`。
- 只在库内部跨模块复用的设施使用 `oneq::common::<domain>`，不构成 public API。

规则：

1. `src/common/` 下的类型必须能追溯到 public 领域头或明确的 `oneq::common::<domain>` 所有权；
   不得仅因目录名把 public-domain implementation 改入 `oneq::common`。
2. 跨模块共享工具不得放在 `oneq::internal::*` 或
   `oneq::common::internal` 这类模糊内部命名空间中。
3. 不得为 `oneq::common::*` 工具新增 `oneq::internal::*` dual-alias 或
   兼容 using 块；迁移期 alias 只能作为同一批次内的临时编译过渡，最终提交前必须删除。
4. `namespace internal` 只可用于测试或翻译单元局部辅助语义；跨文件、跨模块消费的
   `src/common/` 设施必须有明确的 `oneq::common::<domain>` 所属域。
5. 若某工具需要成为外部消费者合同，应通过 `include/1q/` 公开并补充 public API
   边界测试，而不是从 `src/common/` 泄漏。

## 实现安全与失败语义

下列规则源自 `src/` 架构与安全审查，是所有模块共享的规定性约束。

1. **项目失败语义不得依赖 C++ 异常。** `src/` 与 `include/` 不得新增 `throw`，也不得用
   `std::runtime_error`、`std::invalid_argument` 等异常作为项目 API 的失败通道。I/O、构造和解析失败
   必须转换为错误状态、空 reader 或诊断字段。对 HighFive、JSBSim 等可能抛异常的第三方边界，允许在
   最窄调用点使用既有 `try/catch`，但 catch 后必须转换为项目状态且不得让异常穿透 session/adapter
   边界。当前构建没有全局启用 `-fno-exceptions`，因此本文不承诺该编译模式已经成立；若要建立该门禁，
   必须先替换或隔离所有第三方异常边界并增加真实构建验证。

2. **存在性标志必须与数据一致，且由校验层断言。** `has_xxx` 若表达“调用方是否提供本周期可选
   数据”，当 `has_xxx=false` 但对应数据非默认值时，输入校验必须报 error 级问题并 abort，不得让
   数据静默跳过。典型反例是 AR 的 `has_environment`：环境快照已写入但因漏置 flag 不被消费，会让
   杂波/干扰/大气数据完全不进入信号链且无任何信号。若布尔量明确是配置选择器而非数据存在性标志，
   可以定义关闭时整组候选参数不生效且不校验，但必须在 public Doxygen 和模块 design 明确优先级，并以
   启用/关闭对照测试锁定。反之，**必填字段不设在场标志**——缺失语义由值校验直接承担（非有限/零向量
   即拒），`has_xxx` 仅保留给真正的可选数据（2026-08-24 起 SBIRS 卫星位置/速度/姿态与 ESR 平台
   ECEF 运动学按此收敛）。

3. **非执行周期必须产生准确的结构化 reason，不得静默或伪造故障。**
   a. **校验失败必须显式 reason**：校验权威层（控制器 `RunOnce`；AR 公共路径入口为 session，
      见 session_contract.md 规则 14 校验层归属条款）设置显式 abort reason（如
      `kValidationRejected`），不执行 pipeline，不合成空输出帧，不把非法输入记作新的有效
      batch/帧。
   b. **关机等合法非执行状态用独立 reason**（如 `kSensorPoweredOff`），不得映射成 output
      contract violation。
   c. **统一不复用**：非执行周期（校验失败/关机/执行 abort）的产品层一律为空载荷，永不复用上一有效输出。
      执行层保留本次输入周期号（`session_contract.md` 规则 15d）。
      `reused_previous_output` 字段、以及支撑复用的 `latest_output`/`previous_output`/
      `has_latest_output`/`has_previous_output` cache 字段已全部删除。
      落地前现状：空产品帧的 `cycle_index` 仍可能为 0，属待迁移实现。
   d. **reason 数值向后兼容**：新增 reason 以显式数值追加，保留已有 Replay 记录中既有数值语义。
   e. **Lifecycle recorder 边界**：不得把非执行周期（`status != kCompleted`）解释为目标丢失或未检测；
      非执行周期不产生 lifecycle 事件，也不推进其累积状态。
   [evidence: tests/contract/airborne_radar/ar_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailure]
   [evidence: tests/contract/electro_optical_sensor/eos_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailureAfterSuccess]
   [evidence: tests/contract/electronic_surveillance_radar/esr_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailure]
   [evidence: tests/contract/sar/sar_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailureAfterSuccess]
   [evidence: tests/contract/sbirs_sensor/sbirs_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailureAfterSuccess]

4. **外部输入解析与 Replay 记录读取必须有上限与完整性校验。** 自研解析器（如 JSON）必须有最大嵌套深度限制、顶层 value 后的 EOF 校验与转义完整性校验。Replay 目录读取必须在读入前检查大小上限（与写入侧守卫对齐）。磁盘写失败必须检查流状态并记录，不得静默丢失。

   failure marker 是 Replay 记录中一个可报告的失败边界，不是回放的终止符。回放必须记录 marker，
   继续应用并比较其后的有效输入/输出，使"有效 -> 拒绝 -> 恢复"整段都进入确定性比较；只有
   divergence、损坏记录或不兼容模块等真正无法继续解释记录的错误才终止回放。
   [evidence: tests/replay/airborne_radar/ar_rf_trace_session_test.cpp::ArRfTraceSessionTest.RejectedCycleAndSameCycleRetryReplayExactly]
   [evidence: tests/replay/electro_optical_sensor/eos_replay_session_test.cpp::EosReplaySessionTest.ReplayEosTraceContinuesAfterFailureMarker]
   [evidence: tests/replay/electronic_surveillance_radar/esr_replay_session_test.cpp::EsrReplaySessionTest.ReplayEsrTraceContinuesAfterFailureMarker]
   [evidence: tests/replay/sar/sar_replay_session_test.cpp::SarReplaySessionTest.ReplayContinuesAfterFailureMarker]
   [evidence: tests/replay/sbirs_sensor/sbirs_replay_session_test.cpp::SbirsReplaySessionTest.ReplayContinuesAfterFailureMarker]

5. **数值归一化必须是常数时间。** 角度/周期归一化等可能接受无界输入的工具函数必须用 `std::fmod` 等常数时间实现，不得用 `while` 循环减/加周期，避免极大输入近似死循环。

## 数值下限语义

数值下限常量不得只因命名相似而合并。当前允许三类边界：

1. **通用数值防护下限**：防除零、对数域、阈值归一化等纯数值保护使用
   `oneq::common::numerics::kNumericFloor` 或更专门的 common numerics helper。
2. **坐标/姿态退化阈值**：ECEF 原点、方向向量零范数、接近姿态奇异点等几何退化判断保留在
   `common/coordinate` 局部实现内，阈值应按坐标算法精度选择，不与功率/概率数值下限共享。
3. **模块局部几何阈值**：例如 EOS 外部输入适配中目标与平台几乎重合的 range gate，属于模块输入几何退化判断，应保留模块局部阈值和状态码。

新增 floor 常量前必须先归入上述语义桶；不能把物理/几何阈值机械改为 `kNumericFloor`，也不能把通用除零保护散落成模块私有常量。

## 会话相关模块契约（指针）

以下契约只对"有 `*Session` 会话模型的传感器模块"（AR/ESR/EOS/SAR/SBIRS/RIR）有效，不是所有模块的跨模块契约。完整内容见 [session_contract.md](session_contract.md)：

- SessionConfig 直接赋值规则（无 ConfigBuilder / 无 dirty flag / 无隐式覆写；运行期写 RuntimeConfigPatch + has_*）
- Session composition ownership（`Impl` 所有权边界、AR 决策 seam）
- 运行期配置提交策略（事务性提交 vs 立即提交的分类表 + 各模块归属判定规则）
- 电源状态单源契约（`sensor_enabled` 唯一来源、`has_sensor_enabled` 唯一入口，六模块统一，RIR 建模即遵守）
- 分层周期记录 + 可选投影（规则 15：一次生产、按层组合；`Step()` 是产品糖；失败时产品空载荷；旧称两通道/三层）
- 可观测性单源（生产者唯一、投影只读周期记录、Replay 按层落盘、`PROJECT_LOG` 人读旁路；产品形态齐套表）
- 统一问题列表模型（`*IssueList` 单一列表 + `phase` 来源标签 + 可选定位；输入校验不设平行字段，见 session_contract.md 规则 14）
- Replay 持久化语义（结构化比较状态、`ReplayTraceWriter` 单落盘、codec 边界、runtime patch 记录）

## 工程治理规则（指针）

CMake 工程边界（target 作用域、Windows 验收）和测试架构（type×domain 组织、CTest label、partition 注册）是工程基础设施规则，见 [docs/practice/build_and_test_governance.md](../practice/build_and_test_governance.md)。

## 文档结构

`docs/` 只允许以下一级目录：

- `common`
- `review`
- `practice`
- `airborne_radar`
- `electro_optical_sensor`
- `electronic_countermeasure`
- `electronic_surveillance_radar`
- `flight_dynamic`
- `fusion`
- `navigation`
- `precision_evaluation`
- `remote_identification_radar`
- `sar`
- `sbirs_sensor`
- `target_inference`
- `threat_assessment`

`docs/` 顶层不保留散落的 Markdown 文件。所有文档必须落在上述某个一级目录内。

`review/` 是唯一允许的评审和迁移草案目录，只能存放扁平 Markdown 草案文件。每个草案必须在文件头声明 `Status: draft`，不得作为当前权威文档引用；结论落定后，应迁入 `common/contract.md`、`common/open_questions.md` 或对应模块 `design.md`，再删除草案。

`practice/` 存放工程实践与基础设施类设计文档（非业务模块设计）：构建、测试策略、覆盖率、示例程序、批量验证框架等跨模块工程产物。每份文档为扁平 Markdown，文件头声明 `Status: active` 与 `Authority:`（如 `build infrastructure`、`test infrastructure`、`examples`）。`practice/` 不存放业务模块设计——模块设计归各自 `design.md`；也不存放契约规则——规定性规则归 `common/contract.md`。

每个业务模块以 `design.md` 为设计权威**入口**，另允许 `boundaries.md`、`data-flow.md`、`algorithms.md` 三个设计文档。`design.md` 承载模块定位与文档导航；`boundaries.md` 承载模块级边界、非目标与设计变更规则；`data-flow.md` 承载数据流、输入输出与状态所有权；`algorithms.md` 承载算法登记表与每算法的实现边界、反直觉点。切分原则：模块级边界（主语是"模块/API/输出"）归 `boundaries.md`，算法级边界（主语是"某算法/某计算路径"）归 `algorithms.md`。文档写代码读不出来的内容（定位/边界/禁令/反直觉点/否决理由），算法逐步逻辑归代码。历史决策记录（旧版 `decisions.md`、`history.md`、`contract.md`）和模块入口（`README.md`）的内容已内聚到该文档集中。

`common/` 只允许保留六份文档：

- `contract.md` —— 公共契约（规定性：所有模块必须遵守的规则）。
- `session_contract.md` —— 有 Session 的传感器模块的统一会话契约（会话配置直接赋值、Session 组合所有权、运行期配置提交、电源单源、分层周期记录+可选投影、可观测性单源、Replay 持久化语义）。
- `open_questions.md` —— 跨模块架构观察与待决项（非规定性：记录调查中发现但尚未定论的议题，不构成契约约束）。条目推进到有结论时，应回写为契约规则（进 contract.md）或模块设计（进对应 design.md），并从 open_questions.md 移除。
- `rf_architecture.md` —— AR/ESR/ECM/RIR 公共 RF 工程架构设计描述（provenance、单周期交换时序、接收机影响分层）。
- `issue_codes.md` —— 各模块 issue code 注册表的人读辅助目录（由各模块 `<Module>IssueCodes.h` 的 `@brief` 提取生成；机器消费以公开头文件常量为唯一事实来源）。

模块目录内不保留 `archive/`、`audits/`、`contracts/`、`design/`、`decisions/`、`workflow/`、`migration/` 等展开式历史目录。历史细节需要追溯时从 git 历史读取。

各模块以 `design.md` 为设计权威入口，配合 `boundaries.md`、`data-flow.md`、`algorithms.md`。限制条件与否决方向的证据引用直接嵌入对应文档的 `[evidence: ...]` 标注，指向对应测试文件和 git 历史。
