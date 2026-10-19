---
Status: draft
Date: 2026-10
Baseline: 1q 工作树 `include/1q/**` + `docs/**`；BJ 工作树 `C:\software\workspace\1qfosim\BJ_10Y_WorkSpace\ModelSource\CyModel\**`（两仓库当时工作树，未打 tag）
Authority: 表**注册机制与结构约束**权威为 `ModelSource/CyModel/Record/OutPutRecord/OutputRecord.hpp`；本文是评估数据表的**定义建议与能力核对**，字段口径若与库实现冲突以库为准。
Answers: 评估用六大类数据表怎么定义、每列从哪个模型的哪个字段来、现在能填多少、缺的部分缺在哪
---

# BJ_10Y_WorkSpace 评估数据基础信息表（仿真评估用）

## 0. 结论速览

- **六大类拆成 12 张表**（含 1 张空域点表），全部按 `OutputRecord.hpp` 同一套机制注册：POD 结构 + `RegisterRelevanceTable/RegisterTableColumn/RegisterOneTableEnd` + 运行时 `GenerateRecorder`，落盘后由宿主效能评估的 SQL 抽取算子直接查（`AssessmentSource/algorithm.ois` 就是 `SELECT … FROM <表名>`）。
- **能立即落盘（4 张，只差写表代码）**：实体基础信息（T1-1）、雷达航迹（T2-2）、空域定义与点（T5-1/T5-2）。
- **需要少量补充（6 张）**：雷达探测（AR 无量测 DTO、J/S 库内不产）、电子侦察观测与假设（ESR 无归属通道 + BJ 尚无 ESR 胶水模型；只有"告警机"路径能先跑起来）、实体状态（毁伤等级依赖毁伤模型维护）、空域事件（需新增几何判定）、开关机（需新建事件点）。
- **需要新增能力（2 张）**：毁伤信息表（脱靶量/PK/抽样值/毁伤等级事件化）、武器发射表的部分列（齐射序号、制导方式、对应毁伤事件回填）。
- **三条最关键的判断**：
  1. **雷达探测与电子侦察的数据源在 1q，不在宿主**：`EmitterObservation`/`EmitterHypothesis`（ESR）、`TrackStateSnapshot`（AR）、`RirFeatureMeasurementRecord`（RIR）字段密度足够支撑评估，当前的缺口只是"把会话输出写成表"，不是算法能力。**但 1q 自带的评估模块（`precision_evaluation`）只认 SBIRS 双星，接收不了这些产品**——因此表要交给宿主效能评估与离线复盘，不能指望 1q 侧现成算指标。
  2. **毁伤、武器发射、空域属于宿主引擎与 BJ 业务模型**，1q 无对应模型。但老表比想象的更不能用：**`AF_Launch`/`AF_Hit`/`AAMMissedRecord` 已注册却全仓零写入（死表），`Stealth*` 三张连注册都没有**；真正带发射/命中标志的是 `DaiWaDengLuRecord`（4 个弹药/平台模型写，含价格），`SAMRecord` 主要由认知 FU 与弹模型写（语义是交战事件，不是发射事件）；毁伤只有布尔/文本事件，且三个效果模型都不维护 HP、平台 `Damage()` 恒返回 `UNKNOWN`。
  3. **开关机是"有状态、无事件"**：1q 六个传感器模块全部有 `sensor_enabled` 与 `kSensorPoweredOff`（但都是静态配置 + 周期状态，胶水未接运行期开关机指令）；宿主侧只有 EWSIM 系平台有真开关状态 `R_Power_On`/`R_WorkState` 与 `RadarStatusReport` 消息；`CYBER_SENSOR_EMISSION_EVENT_RADAR_TURNON` 只有枚举定义、**全仓库无生产者**（实际只有导弹模型发的 `RADAR_LOCK`）。**没有任何表记录"谁、什么时刻、开了哪台设备、为什么"**，必须新表 + 新建事件点。

---

## 1. 范围、用途与不做什么

### 1.1 三条消费链

表本身不产生指标，它只做一件事：**把仿真过程中散落在各模型内的原始量变成可 SQL 联结的平表**，供三条链消费：

| 消费链 | 消费者 | 消费方式 | 证据 |
|---|---|---|---|
| 效能评估 | 宿主 `效能评估` | 抽取算子 `SELECT SUM/AVG(列) FROM 表` → `predefine.pis` 归一化 → 指标体系计分 | `AssessmentSource/algorithm.ois:5-9`、`AssessmentSource/predefine.pis:3-13` |
| 精度评估 | 1q `precision_evaluation` | 五指标（角度/双星交会/速度/落点/发射点）+ AHP 综合分；**仅覆盖 SBIRS 双星链路** | `include/1q/precision_evaluation/PrecisionEvaluationTypes.h:50-56`、`docs/precision_evaluation/boundaries.md:8-38` |
| 复盘与证据链 | 人工 / 离线脚本 | 按 `RunNum + CycleIndex` 拉时间线，做误判归因 | `docs/review/functional_acceptance_test_criteria_ocr.md:324-367` |

### 1.2 与既有记录表的关系：**新增，不改老表**

老表（`OutputRecord.hpp` 内共声明 28 个结构，其中 **24 个**在 `InitlizeRecord` 注册、可落盘；`StealthMissileLaunchRecord`/`StealthMissileLandingRecord`/`StealthMissileResultRecord`/`GG_Plane_Dead` 只声明未注册，属沉睡表）服务的是**历史想定的效能评估**，字段按当时的指标体系长出来的，且已被 `resultheader.istruct` 登记、被 `algorithm.ois` 的算子引用。**改列名会打断既有评估工程**，因此：

- 老表**一律不动**（列名、顺序、语义冻结）；
- 新表按"评估需要什么"重新设计，允许与老表语义重叠（例如新的雷达探测表与 `DetectRecord` 重叠，但粒度从"事件"改为"每周期每目标一行"，且带真值几何列）；
- 新表名不得与老表重名，注册时追加到 `InitlizeRecord` 尾部。

### 1.3 不在本文范围

1. 不改 1q 任何公开产品结构（表是集成侧记录，不是产品通道）；
2. 不定义指标权重、归一化方法、通过判据——那是评估工程（`.pis`/`.ois`/指标体系）的事；
3. 不覆盖第 70–77 项"内部接口"证据（那是 1q 验收日志的职责）；
4. 不覆盖通信/指挥（C4I）报文类数据。

---

## 2. 建表约定（12 张表共用，必须遵守）

### 2.1 注册机制（与 `OutputRecord.hpp` 完全一致）

三步，缺一不可：

```cpp
// ① 在 Record/OutPutRecord/OutputRecord.hpp 中声明 POD 结构（含 Register 静态函数）
struct XxxRecord { ... static void Register(CyberDataRecorderFactory *_fact){...} };
// ② 在 InitlizeRecord() 末尾追加一行（同一文件，OutputRecord.hpp:995-1025）
XxxRecord::Register(_fact);
// ③ 模型侧运行时写行
XxxRecord r;                                   // 构造函数 memset 全零
r.RunNum = GetRunNumber();                     // 现有模型均通过该接口取仿真次数
r.UnitID = Unit->GetEntityId();
r.UnitName = Unit->GetEntityName();            // 注意 CyberString 的 operator= 语义（§2.2）
GenerateRecorder("XxxRecord", &r, sizeof(r), CYBER_RECORDER_RECORD_ONLY);
```

模型 DLL 侧的注册入口是 `fosim_model_manager.cpp` 里的 `InitlizeRecord(global_engine_factory->GetDataRecorderFactory())`（16 个模型工程各一份，`ModelSource/CyModel/Equipment/Sensors/init/fosim_model_manager.cpp:42`），因此**只要老表文件被新增结构，所有模型同时获得该表**，无需逐工程改注册。

### 2.2 POD 硬约束（违者无法落盘）

`memset(this, 0, sizeof(*this))` + `sizeof(结构)` 直接落盘 + SQL 列按偏移读取，三者共同决定了：

| 允许 | 禁止 |
|---|---|
| `CyberInt`(int32)、`CyberLong`(int64)、`CyberDouble`、`CyberBool`、`CyberChar32/64/128` | `std::string`、`std::vector`、`bool`(C++)、指针、虚函数、继承 |
| 固定长度数组（如 `CyberDouble Points[8][3]`） | 变长容器、可选字段用 `std::optional` |

`CyberChar*` 是 `CyberString<N>`（`ModelSource/include/Utils/CyberDataRecorder/CyberBaseType.h:29-30`），注意两点：

- `operator=(const char*)` **超长时静默不赋值**（保留旧值），赋值前必须自行截断；老代码用 `strcpy`（经 `operator char*()`）同样不设边界，**新表代码统一用 `snprintf(buf, sizeof(buf), "%s", src)` 后赋值**；
- `operator=(const std::string&)` 超长时 `return CyberString();` —— **返回的是临时对象的引用（悬垂，属 UB），不只是丢值**；禁止用标准库字符串直接赋值（`CyberBaseType.h:77-86`）。

### 2.3 列类型映射

`RecordDataType` 只有五种（`ModelSource/include/Utils/CyberDataRecorder/CyberDataRecorder.h:5-11`），映射固定：

| 结构成员 | 注册类型 | 备注 |
|---|---|---|
| `CyberInt` / `CyberLong` | `RECORD_INTEGER` | ID、计数、枚举、标志位 |
| `CyberDouble` / `CyberFloat` | `RECORD_DOUBLE` | 物理量一律 `double` |
| `CyberChar64/128` | `RECORD_STRING` | 名称、枚举文本、原因 |
| `CyberBool` | `RECORD_BOOL` | 二值状态 |

> **老表已知坑**：`RecognizedMALDInfo` 把三个不同成员都注册成同名 `"RecognizedMALD_1"` 列且类型错配（`OutputRecord.hpp:384-386`），`AF_Launch.LaunchDistance` 注册成 `RECORD_BOOL`（`:917`）。新表禁止复制这两类错误：**列名必须与成员一一对应，类型必须与成员宽度匹配**。

### 2.4 单位约定

- 位置/距离：m；速度：m/s；加速度：m/s²；角度：**deg**（老表 `OpirDetectRecord` 用 rad，新表统一 deg 以免评估阶段反复换算，rad 源量在写入时换算）；
- 时间：`TimeSec` = 仿真时刻（s，自仿真起点）；事件表另有 `CycleIndex`；
- 功率/幅度：dB 或 W，按列名后缀显式标注（`SnrDb`、`JnDb`、`PowerW`）；
- 频率：Hz；脉冲间隔/脉宽：**s**（沿用 1q `EmitterObservation` 口径，导出报表时可换算 µs）；
- 概率/置信度：无量纲 [0,1]；
- **每个列的中文描述必须带单位**（注册时的第三个参数），与 `OpirDetectRecord` 的写法一致（`OutputRecord.hpp:70-74`）。

### 2.5 缺值约定

`memset` 会让所有数值列为 0，而 **0 在评估里是合法值**（距离 0、速度 0、概率 0）。因此：

- 数值列缺值写 `kNoData = -999999.0`，整数/枚举列缺值写 `kNoDataInt = -999999`；
- 字符串列缺值写空串 `""`；
- 每个可缺列在字段表中标注「可缺」；常量用结构体内的 `static const double`/`static const int`，在 `.cpp` 里给出定义（VS2015 不支持内联 `static constexpr`，静态成员不影响 POD 布局）；
- **评估算子必须显式过滤 `kNoData`**，不得把缺值当 0 参与均值。

### 2.6 关联键与"真值归属"写入规则（本文最关键的约定）

1q 的公开产品是**去真值化**的：产品层默认不携带仿真真值归属，归属挂在"信封层"（`ArCycleResult.track_attributions`、`RirCycleResult.track_attributions`、`Eos/SbirsCycleResult.detection_attributions`；见 `docs/common/session_contract.md:408-431`）。

但**记录表不是产品通道**，它是仿真系统自己的评估数据面，评估没有真值就无法算误差。因此本设计采取**双层列**：

| 层 | 列前缀 | 写入者 | 用途 |
|---|---|---|---|
| 产品层 | 无前缀（`RangeM`、`AzDeg`、`SnrDb`…） | 模型会话输出 | 评估对象 |
| 真值层 | `Truth*`（`TruthUnitID`、`TruthRangeM`…） | 集成侧从场景真值 + 归属对照表回填 | 评估基准 |

这**不违反 1q 分层契约**：真值只进评估层、只用于对照、不回写产品（先例：`EvaluationTruthTarget` 就是"真值只进评估层"的合法出口，`PrecisionEvaluationTypes.h:23-30`；`docs/common/session_contract.md:422-431` 约束的是**公共头中的产品 DTO**，不约束仿真系统的结果表）。

归属来源按模块取：

| 模块 | 真值归属来源 | 状态 |
|---|---|---|
| RIR / AR | `track_attributions`（`association_key` ↔ `external_target_id`/`target_name`） | ✅ 已有 |
| SBIRS / EOS | `detection_attributions` | ✅ 已有 |
| ESR | **无归属通道**（`include/1q/electronic_surveillance_radar/**` 中无 `attribution`） | ✘ 需集成侧按 AOA/频率自配对，或推动 1q 增补 |

### 2.7 采样与体量

仿真步长当前 **1.0 s/周期**（`docs/practice/fosimstudio_host_and_integration_guide.md:188`），一次完整跑数千周期、目标十余批。若 12 张表全量写入，单次运行行数约：

| 表 | 粒度 | 单次运行量级 |
|---|---|---|
| 实体基础信息 | 每实体一次 | 10² |
| 实体状态 | 每实体每周期 | 10⁴–10⁵ |
| 雷达探测 | 每雷达×目标×周期 | 10³–10⁴ |
| 雷达航迹 | 每航迹×周期 | 10³–10⁴ |
| 毁伤 | 每判定事件 | 10¹–10² |
| 武器发射 | 每次发射 | 10¹–10² |
| 空域定义 / 点 | 每空域一次 | 10⁰–10¹ |
| 空域事件 | 每次进出/越界 | 10¹–10² |
| 电子侦察观测 | 每辐射源×周期 | 10⁴–10⁵ |
| 电子侦察假设 | 每辐射源×周期 | 10³–10⁴ |
| 开关机 | 每次状态变化 | 10¹–10² |

因此：**状态类表必须支持采样降频**（每 N 周期写一行，N 由模型参数配置，默认 1 的只有探测/航迹表，实体状态默认 5）；**事件类表不降频**。降频参数放模型 `.mdata`，不硬编码。

### 2.8 命名规则

- 表名 = 结构名（`RegisterRelevanceTable` 两个参数一致），PascalCase，后缀 `Record`；
- 列名 = 成员名，PascalCase；单位后缀固定：`M`/`Mps`/`Mps2`/`Deg`/`Sec`/`Hz`/`Db`/`W`/`Flag`/`Id`/`Count`/`Ratio`；
- 阵营列固定叫 `Team`（老表惯例），目标阵营固定 `TargetTeam`；
- 键列固定：`RunNum`（仿真次数）、`CycleIndex`、`TimeSec`、`EventID`。

---

## 3. 六类 12 张表定义

标记：**✅ 现有模型可填**（字段与来源都在，只差写表代码）；**▲ 需补充**（有源但缺列/缺透出）；**✘ 无来源**（需新增模型能力）。

### 3.1 第一类：实体基础信息（2 张）

#### T1-1 `EntityBaseInfoRecord` — 实体编成与静态属性

用途：评估的"分母表"。任何按实体聚合的指标（战损率、发现率、单位价值交换比）都要先能按 `UnitID` 关联到实体身份、类型、阵营、价值。**每次仿真每实体写一行，写于初始化期**。

| 列名 | 类型 | 单位 | 含义 | 写入来源 | 状态 |
|---|---|---|---|---|---|
| RunNum | Integer | — | 仿真次数 | `GetRunNumber()` | ✅ |
| EntityID | Integer | — | 实体（单元）ID | `Unit->GetEntityId()` | ✅ |
| EntityName | String64 | — | 实体名称 | `Unit->GetEntityName()` | ✅ |
| Team | String64 | — | 阵营（红方/蓝方/中立） | `Unit->GetTeamName()` | ✅ |
| EntityType | String64 | — | 单元类型（预警雷达/卫星/地导阵地/指挥所/陆基洲际弹道导弹…） | 想定 `<TypeOfUnit>` | ✅ |
| PlatformName | String64 | — | 主平台名称 | 平台实例 | ✅ |
| PlatformType | String64 | — | 主平台模型类名 | `.dscript` EMPLOY PLATFORM | ✅ |
| SuperiorUnitID | Integer | — | 上级单元 ID（指挥关系） | `CyberUnitInfoType.commander_id_`（`cyber_struct_type.h:253-303`）或想定 `Relations/OrganicSuperiorHandle` | ✅ |
| SuperiorUnitName | String64 | — | 上级单元名称 | `commander_name_` | ✅ |
| SensorCount | Integer | — | 编成内传感器数量 | `.dscript` EMPLOY SENSOR | ✅ |
| SensorNames | String128 | — | 传感器名称（`;` 分隔，≤4） | 同上 | ✅ |
| JammerCount | Integer | — | 编成内干扰机数量 | `.dscript` EMPLOY JAMMER | ✅ |
| JammerNames | String128 | — | 干扰机名称（`;` 分隔，≤4） | 同上 | ✅ |
| WeaponCount | Integer | — | 编成内武器系统数量 | `.dscript` EMPLOY WEAPONSYSTEM | ✅ |
| UnitPrice | Double | — | 实体价值（评估用，无量纲或万元，想定口径） | 型号库/想定 | ▲ 型号库无统一价格字段，需加参数 |
| InitLat / InitLon | Double | deg | 初始经纬度 | 想定 `<Disposition>` | ✅ |
| InitAltMSL | Double | m | 初始海拔 | 同上 | ✅ |
| InitHeadingDeg | Double | deg | 初始航向 | 同上 | ✅ |
| StartTimeSec | Double | s | 参战时刻 | 想定 `START n` | ✅ |
| EndTimeSec | Double | s | 退出/结束时刻（可缺） | 事件推导 | ▲ |

要点：`SensorNames` 这类"列表折叠"是 POD 约束下的妥协；若评估要按传感器粒度 join，用 T6-3 的 `SensorIndex` 展开而不是解析字符串。

#### T1-2 `EntityStateRecord` — 实体状态快照（周期）

用途：态势复盘基线、按高度/速度分层统计、毁伤前后对照、空域越界判定的输入。**默认每 5 周期一行**。

| 列名 | 类型 | 单位 | 含义 | 写入来源 | 状态 |
|---|---|---|---|---|---|
| RunNum / CycleIndex / TimeSec | Integer/Integer/Double | —/—/s | 运行号、周期号、时刻 | 引擎 | ✅ |
| EntityID / EntityName / Team / EntityType | Integer/String64×3 | — | 实体身份 | 引擎 | ✅ |
| PlatformName | String64 | — | 平台名 | 引擎 | ✅ |
| Lat / Lon / AltMSL | Double×3 | deg/deg/m | 地心→地理位置 | `GetPosition` + 坐标库 | ✅ |
| EcefX / EcefY / EcefZ | Double×3 | m | ECEF 位置 | 同上 | ✅ |
| EcefVx / EcefVy / EcefVz | Double×3 | m/s | ECEF 速度 | `GetVelocity` | ✅ |
| SpeedMps | Double | m/s | 速度模长 | 派生 | ✅ |
| HeadingDeg / PitchDeg / RollDeg | Double×3 | deg | 姿态 | `GetOrientation` | ✅ |
| Alive | Integer | — | 存活标志（1/0） | 引擎实体状态 | ✅ |
| DamageState | Integer | — | 毁伤等级：0未知/1未受影响/2摧毁/3损伤（`CyberDMGRC` 原值） | 毁伤模型维护 | ▲ 仅 `StdEffectPK` 路径会置，普通模型不维护 |
| HpRatio | Double | % | 剩余生命值比例（可缺） | 仅个别模型有 HP（如浮桥 100 满血） | ▲ 非通用 |
| RelationName | String64 | — | 敌我关系（我方/敌方/中立，`CyberUnitRelationshipType`） | 引擎 | ✅ |
| ActiveSensorCount | Integer | — | 当前开机传感器数 | 开关机状态聚合 | ✅ |
| PositionSource | Integer | — | 位置来源：0 真值/1 模型输出 | 集成侧标注 | ✅ |

> **引擎其实给得比这张表要的更多**：`CyberUnitInfoType` 里还有 `rcs_`、`irar_`（红外辐射）、`acceleration_`、`dictionary_name_`（型号字典名），`ICyberUnit` 还提供 `GetKillTime()`（死亡时刻）。若评估要算 RCS 特征利用或红外可探测性，直接加列即可，不需要新模型。角度/位置是 ECR 地心系，经纬高必须用 `CyberEarthECRToLLA` 换算（`fosim_algorithm_impl.h:184`），现成样板见 `zFZ6h01bpltfw_util.cpp:571-621`。

### 3.2 第二类：雷达探测信息（2 张）

#### T2-1 `RadarDetectionRecord` — 逐雷达×目标×周期的探测与量测

用途：探测概率、发现距离、测量精度（角度/距离误差）、受扰降效——**这是雷达能力评估的主表**。粒度定在"每雷达每目标每周期一行"，因为这是 1q 当前公开 DTO 能给出的最细粒度（点迹级 Pulse/Plot 不在公开输出里，见 §5.4）。

| 列名 | 类型 | 单位 | 含义 | 写入来源 | 状态 |
|---|---|---|---|---|---|
| RunNum / CycleIndex / TimeSec | 同上 | — | 键 | 引擎 | ✅ |
| Team / UnitID / UnitName / PlatformName | — | — | 雷达所属实体 | 引擎 | ✅ |
| SensorName / SensorType / WorkMode | String64×3 | — | 传感器名、型号、工作模式（搜索/跟踪/制导/识别） | `ArCycleResult.effective_work_mode`、RIR `pointing_source` | ▲ 仅 AR/RIR 有"本周期生效模式"；ESR/EOS/SBIRS/SAR 的周期结果里没有该字段 |
| TargetUnitID / TargetUnitName / TargetTeam / TargetPlatformName | — | — | 真值目标身份 | 归属对照 `track_attributions` | ✅（RIR/AR）；ESR 雷达模式需自配对 ▲ |
| DetectionID | Long | — | 探测/量测记录 ID | 产品键 | ✅ |
| Detected | Integer | — | 探测结论：0 未探测/1 探测/2 受扰（`CYBER_SENSOR_DETECTRC`） | 会话输出 + `receiver_impairment` | ✅ |
| RangeM / RangeRateMps | Double×2 | m / m/s | 斜距、径向速度 | `RirFeatureMeasurementRecord.range_m`；径向速度由航迹速度投影或逐周期差分 | ✅ RangeM；▲ RangeRateMps 需派生 |
| AzDeg / ElDeg | Double×2 | deg | 视线方位/俯仰（雷达局部 ENU） | `look_az_deg`/`look_el_deg` | ✅ |
| SnrDb | Double | dB | 周期信噪比（单脉冲口径） | `cycle_detection_snr_db`／`snr_db` | ✅ |
| Pd | Double | — | 检测概率 | 1q 信号处理 | ✅ |
| RcsM2 | Double | m² | 估计 RCS | `TrackStateSnapshot.rcs`／RCS 特征量测 | ✅ |
| RangeSigmaM / AzSigmaDeg / ElSigmaDeg | Double×3 | m / deg / deg | 量测标准差（协方差对角开方） | 量测协方差 | ✅ |
| Jammed | Integer | — | 是否受扰 | `ArCycleResult.receiver_impairment` + J/N 门限 | ▲ 1q 不出统一"受扰"结论，RIR 路径无该量 |
| JnDb | Double | dB | 干扰/噪声比 | `ArInterferenceObservation.jammer_to_noise_db`（门限 `interference_observation_jn_gate_db`，默认 6 dB） | ▲ 仅 AR 侧有；**J/S 库内不产出**（ECM 硬性非目标，见 §5.1） |
| DeceptionClass | Integer | — | 欺骗分类（0 无/1 疑似假目标） | `DeceptionClass` | ▲ 仅 AR |
| TruthRangeM / TruthAzDeg / TruthElDeg | Double×3 | m / deg / deg | 真值几何对照 | 场景真值 | ✅ |
| BlockedCause | String64 | — | 未探测主因（地平线/遮挡/门限/静默/校验失败） | 排除诊断（1q exclusion cause recorder） | ✅ |

> **粒度现实**：RIR 能给出"每航迹每周期"的 `look_az/el + range + snr + dwell + bandwidth`（`RirFeatureMeasurementTypes.h:112-117`），这是本表在 RIR 链路上的真实粒度；**AR 完全没有量测 DTO**（`include/1q/airborne_radar/**` 无 detection 记录类型，`grep [Dd]etection` 只命中配置字符串），所以"AR 雷达探测表"要么等 1q 增补量测行，要么退化为 `RadarTrackRecord` + 由航迹反算几何。RIR 另有单周期标量 `max_detected_slant_range_m`（`RirCycleResult.h:103-110`，给外部判断"实际探测距离"的观测口径），可作"发现距离"指标的旁证。

#### T2-2 `RadarTrackRecord` — 航迹输出（每航迹每周期一行）

用途：跟踪精度、航迹连续性/起批时间、关联正确率、识别结论准确率。

| 列名 | 类型 | 单位 | 含义 | 写入来源 | 状态 |
|---|---|---|---|---|---|
| RunNum / CycleIndex / TimeSec | — | — | 键 | 引擎 | ✅ |
| Team / UnitID / UnitName / PlatformName / SensorName | — | — | 航迹归属雷达 | 引擎 | ✅ |
| TrackID | Long | — | 库内航迹关联键 | `association_key` | ✅ |
| TrackStatus / TrackStatusName | Integer/String64 | — | 候选/确认/丢失 | `TrackStatus`、`RirTrackLifecycleStatus` | ✅ |
| TruthUnitID / TruthUnitName | Integer/String64 | — | 真值目标（评估对照） | `track_attributions` | ✅ |
| PosEnuXM / PosEnuYM / PosEnuZM | Double×3 | m | 滤波位置（雷达局部 ENU） | `position_x/y/z`、`position_enu_*_m` | ✅ |
| VelXMps / VelYMps / VelZMps / SpeedMps | Double×4 | m/s | 滤波速度 | `velocity_*`、`speed_m_per_s` | ✅ |
| AccXMps2 / AccYMps2 / AccZMps2 | Double×3 | m/s² | 滤波加速度 | `acceleration_*`（AR） | ✅ |
| RangeM / AzDeg / ElDeg | Double×3 | m / deg | 由位置导出的极坐标 | 派生 | ✅ |
| PosUncertaintyM | Double | m | 位置不确定度（协方差迹开方） | `estimation_uncertainty_trace` | ✅ |
| HitCount / MissCount | Integer×2 | — | 累计命中/连续失配 | 同名 | ✅ |
| TrackSnrDb | Double | dB | 本周期航迹信噪比 | `cycle_detection_snr_db` | ✅ |
| RecogCategory / RecogModel / RecogConfidence | Integer/String64/Double | — | 识别类别/型号/置信度 | `RirTrackRecognitionOutput`、`LrdrRecognitionRecord` 同源 | ✅ |
| FirstDetectTimeSec / LastUpdateTimeSec | Double×2 | s | 航迹起批/最近更新时刻 | 生命周期记录器 | ✅ |

> **可选的周期汇总列**：AR 还给出每周期一次的关联质量 11 项（`prior_track_count`/`detection_count`/`matched_count`/`new_track_count`/`missed_track_count`/`match_rate`/`new_track_rate`/`missed_track_rate`/`mean_match_cost`/`p95_match_cost`/`association_stress`，`ArOutputTypes.h:92-103`）。它是"每雷达每周期一行"的聚合量，塞进"每航迹一行"的表会重复写入；若评估要算关联正确率/航迹连续性，建议**单列一张 3 列的轻表**（或直接用 §2.7 的降频约束挂在同表首航迹行）。

### 3.3 第三类：毁伤信息（1 张）

#### T3-1 `DamageRecord` — 毁伤判定事件

用途：毁伤率、命中率、脱靶量分布、单发毁伤概率校核、战损比。**每次毁伤判定写一行**（含未毁伤判定，否则算不出毁伤率）。

| 列名 | 类型 | 单位 | 含义 | 写入来源 | 状态 |
|---|---|---|---|---|---|
| RunNum / CycleIndex / TimeSec / EventID | — | — | 键 | 引擎 | ✅ |
| AttackerTeam / AttackerUnitID / AttackerUnitName / AttackerPlatformName | — | — | 攻击方 | 弹药/发射平台 | ✅ |
| MunitionID / MunitionName / MunitionType | Integer/String64×2 | — | 弹药身份与类型 | 弹药实体 | ✅ |
| TargetTeam / TargetUnitID / TargetUnitName / TargetPlatformName / TargetEntityType | — | — | 被毁伤目标 | 引擎 | ✅ |
| HitFlag | Integer | — | 是否命中 | 引擎引信/碰撞 | ✅ |
| MissDistanceM | Double | m | 脱靶量（未命中时为最近距离） | 弹药与目标几何 | ▲ 需在判定点采样，当前无表 |
| KillRadiusM | Double | m | 战斗部杀伤半径 | `StdEffectPKData.LethalRadius` | ✅（有源） |
| Pk | Double | — | 本次判定使用的毁伤概率 | `StdEffectPKData.PKImpact/PKLethalRadius` | ✅（有源） |
| PkDraw | Double | — | PK 判定的随机抽样值（复核用） | 效果模型内部随机数 | ▲ 需透出 |
| DamageStateBefore / DamageStateAfter | Integer×2 | — | 判定前后毁伤等级（`CyberDMGRC`） | 效果模型判定结果 | ▲ 枚举在，但平台 `Damage()` 多数恒返回 `UNKNOWN`，需先修或改读存活位 |
| DamageLevelName | String64 | — | 未受影响/损伤/摧毁 | 同上映射 | ✅ |
| HpBefore / HpAfter | Double×2 | — | 目标生命值前后（可缺） | 仅 `FZGroundPlt` 维护 HP | ▲ |
| EffectModelName / EffectDescriptor | String64×2 | — | 判定模型（StdEffectPK/CyEFPK/CyEFAgentEffect）与效果描述符 | 引擎效果模型 | ✅ |
| CumulativeHitCount | Integer | — | 同目标累计被命中级数 | 集成侧累计 | ✅ |
| RelatedLaunchEventID | Integer | — | 对应发射事件 ID（回填） | 关联 T4-1 | ✅（需两表同时落地） |
| Remark | String64 | — | 备注（异常/人工判定） | — | ✅ |

> **毁伤链的三个实现细节（写表时必须照顾）**：
> 1. `StdEffectPK::Process()` 等弹的 `descriptor.time_+50 s` 才判定，并对 `MaxRadius+15 m` 内所有**敌方**单元施加（`stdeffectpk_exec.cpp:11-43`）；无命中时向发起方回 `EVALUATE_DAMAGE(UNAFFECTED)`（`:45-57`）。所以"毁伤事件时刻"天然比"命中时刻"晚约 50 s，`TimeSec` 要写**判定时刻**并另留一列或备注写命中时刻，否则时间轴会错位。
> 2. `CyEFPK` 不调 `unit->Damage()`，只发 `EVALUATE_DAMAGE` MAL（`cyefpk_exec.cpp:159-172,213-231`）；真正改目标状态的是认知侧处理器（`SetActive(CYBER_FALSE)`，`zFZ6h01caamfu_util.cpp:148-178`）。**因此"毁伤是否生效"取决于场景里挂了哪个 FU**，写表方要同时记录 `EffectModelName` 与生效路径。
> 3. `CyEFAgentEffect` 能返回 `DAMAGED`（1~5 级映射），但其 `Process()` 里的伤害施加被注释（`cyefagenteffect_exec.cpp:149-152`）→ **目前没有模型真正产生"损伤（DAMAGED）"状态**，三级毁伤评估实际上只有"摧毁/未受影响"两端可用。

### 3.4 第四类：武器发射信息（1 张）

#### T4-1 `WeaponLaunchRecord` — 发射事件

用途：发射数量/时机/距离、齐射组织、反应时间、发射即毁伤概率链路、弹药消耗。

| 列名 | 类型 | 单位 | 含义 | 写入来源 | 状态 |
|---|---|---|---|---|---|
| RunNum / CycleIndex / TimeSec / EventID | — | — | 键 | 引擎 | ✅ |
| LauncherTeam / LauncherUnitID / LauncherUnitName / LauncherPlatformName / LauncherEntityType | — | — | 发射平台/单元 | 引擎 | ✅ |
| WeaponID / WeaponName / WeaponType / WeaponModel | Integer/String64×3 | — | 武器/弹药身份、类型、型号 | 弹药实体 + 型号库 | ✅ |
| TargetTeam / TargetUnitID / TargetUnitName / TargetPlatformName | — | — | 指定目标 | 火控 | ✅ |
| LaunchLat / LaunchLon / LaunchAltMSL | Double×3 | deg/deg/m | 发射点 | 弹药初始位置 | ✅ |
| LaunchEcefX / Y / Z | Double×3 | m | 发射点 ECEF | 同上 | ✅ |
| LaunchRangeM / LaunchAzDeg / LaunchElDeg | Double×3 | m / deg | 发射瞬间目标相对几何 | 派生 | ✅ |
| LaunchAngleDeg | Double | deg | 发射角（离轴/俯仰） | 发射姿态 | ✅ |
| SalvoIndex / SalvoSize | Integer×2 | — | 本次齐射第几发 / 齐射总发数 | 火控/发射事件序号 | ▲ 序号目前只拼在弹单元名里（`"…00"+std::to_string(++CountMissle)`，`zFZ6h01baamws_process.cpp:291`），需显式化；总发数可用 `MunitionNums` 余弹反推 |
| GuidanceMode | String64 | — | 制导方式（指令/半主动/被动/惯导/复合） | 弹药型号数据 | ▲ 无统一字段（`AF_MidGuidance.IsMidGuidance` 零写入） |
| FireControlUnitID / FireControlUnitName | Integer/String64 | — | 制导/火控单元 | `Cognition/*FU` | ✅ |
| LaunchEventName | String64 | — | 事件名（发射/离架/抛放/投放） | 模型事件 | ✅ |
| RelatedDamageEventID | Integer | — | 对应毁伤事件（回填） | 关联 T3-1 | ✅（需两表同时落地） |
| HitFlag | Integer | — | 最终是否命中（回填） | 同 T3-1 | ▲ |
| FlightTimeSec | Double | s | 飞行时间（回填） | 弹药生命周期 | ✅ |

### 3.5 第五类：空域信息（3 张）

#### T5-1 `AirspaceInfoRecord` — 空域定义（静态）

用途：空域资源使用率、进入/退出合规性、责任区（杀伤区）覆盖、越界告警判定的边界数据源。
数据源是宿主的 `CySZone`（`ScenarioSource/**/ScenarioDatabase/Airspaces/*.mdata`），一个空域一份。

| 列名 | 类型 | 单位 | 含义 | 写入来源 | 状态 |
|---|---|---|---|---|---|
| RunNum | Integer | — | 仿真次数 | 引擎 | ✅ |
| AirspaceID / AirspaceName | Integer/String64 | — | 空域 ID 与名称 | `CySZone/Name`（宿主无数字 ID，**ID 由集成侧按名称序或哈希分配**，一次运行内稳定） | ✅ |
| AirspaceType | Integer | — | 宿主 `Type` 原值：1 责任区/2 导弹交战区/3 监视区 | `CySZone/Type`（`CySZone.mdef:83-110`） | ✅ |
| AreaType / AreaTypeName | Integer/String64 | — | 区域几何类型：1 多边形/2 圆形/3 扇形 | `CySZone/AreaType`（`CySZone.mdef:111-138`） | ▲ 扇形(3) **未实现**（`Inside()` 只处理圆/多边形） |
| FloorMSL / CeilingMSL | Double×2 | m | 下界/上界高度 | `FloorMSL`/`CeilingMSL` | ✅ |
| IsBound | Integer | — | 是否绑定到单元 | `IsBound` | ✅ |
| CenterLat / CenterLon | Double×2 | deg | 圆心（圆形区域） | `CenterLatitude/Longitude` | ✅ |
| RangeM | Double | m | 半径（圆形区域） | `Range` | ✅ |
| OrientationDeg / SectorAngleDeg | Double×2 | deg | 扇区朝向/张角 | `Orientation`/`SectorAngle` | ✅ |
| PointCount | Integer | — | 多边形顶点数（点坐标见 T5-2） | `PointCount` | ✅ |
| ForceSide | String64 | — | 归属阵营 | **宿主模型无此参数**（`.mdata` 里出现的 `<ForceSide>` 不在 `CySZone.mdef` schema 内、模型不读） | ▲ 需按 `BelongUnit` 的阵营推导 |
| BelongUnitID / BelongUnitName | Integer/String64 | — | 绑定单元（责任区常见） | `BelongUnit`（**存的是单元名，ID 需按名字解析**） | ▲ |
| UsageType | String64 | — | 用途：巡逻空域/责任区/禁飞区/拦截区 | 空域命名/想定 | ▲ 宿主无显式枚举，需约定 |
| ValidFromSec / ValidToSec | Double×2 | s | 生效时间窗（可缺） | 宿主无此字段 | ✘ |

#### T5-2 `AirspacePointRecord` — 空域多边形顶点

用途：`AirspaceInfoRecord` 的点集不能塞进定长列，拆表后仍可 SQL 直接算几何。

| 列名 | 类型 | 单位 | 含义 | 写入来源 | 状态 |
|---|---|---|---|---|---|
| RunNum / AirspaceID / AirspaceName | — | — | 键 | 引擎 | ✅ |
| PointIndex | Integer | — | 顶点序号（0 基） | 点序 | ✅ |
| Lat / Lon / AltMSL | Double×3 | deg/deg/m | 顶点坐标与高度 | `Points/Lat,Lon,Alt` | ✅ |
| SpeedMps | Double | m/s | 顶点附带的规划速度 | `Points/Speed` | ✅ |

#### T5-3 `AirspaceEventRecord` — 空域使用事件

用途：进出时刻、滞空时长、越界次数、高度越限——空域管制类评估的主表。**每次状态变化写一行**。

| 列名 | 类型 | 单位 | 含义 | 写入来源 | 状态 |
|---|---|---|---|---|---|
| RunNum / CycleIndex / TimeSec / EventID | — | — | 键 | 引擎 | ✅ |
| AirspaceID / AirspaceName / UsageType | — | — | 空域 | 同 T5-1 | ✅ |
| EntityID / EntityName / Team / EntityType / PlatformName | — | — | 进入方 | 引擎 | ✅ |
| MissionName | String64 | — | 任务名（想定 `Mission/*.mdata`） | 任务实体 | ✅ |
| EventType / EventName | Integer/String64 | — | 进入/离开/越界/高度越限/滞留超时 | 几何判定 | ▲ 无现成模型，需集成侧判定 |
| Lat / Lon / AltMSL | Double×3 | deg/deg/m | 事件时刻位置 | 引擎 | ✅ |
| DistanceToBoundaryM | Double | m | 距边界距离（负值=已越界） | 几何计算 | ▲ 同上 |
| AltMarginM | Double | m | 距上下界余量（负值=越限） | 几何计算 | ▲ 同上 |
| ViolationFlag | Integer | — | 是否违规 | 判定 | ▲ 同上 |
| DwellTimeSec | Double | s | 本次在空域内持续时间（离开时回填） | 时间差 | ✅ |

### 3.6 第六类：电子侦察探测结果与开关机（3 张）

#### T6-1 `EsrDetectionRecord` — 电子侦察观测（每辐射源观测一行）

用途：侦察截获率、测向精度、参数测量精度（载频/PRI/脉宽）、分选正确率、受扰与饱和。字段几乎与 1q `EmitterObservation` 一一对应（`include/1q/electronic_surveillance_radar/session/EmitterObservation.h:54-72`）。

| 列名 | 类型 | 单位 | 含义 | 写入来源 | 状态 |
|---|---|---|---|---|---|
| RunNum / CycleIndex / TimeSec | — | — | 键 | 引擎 | ✅ |
| Team / UnitID / UnitName / PlatformName | — | — | 侦察设备所属实体 | 引擎 | ✅ |
| SensorName / SensorType | String64×2 | — | 侦察传感器名/型号 | 会话配置 | ✅ |
| ObservationID | Long | — | 观测唯一标识 | `observation_id` | ✅ |
| AoaAzDeg / AoaElDeg | Double×2 | deg | 测向方位/俯仰 | `aoa_az_deg`/`aoa_el_deg` | ✅ |
| RfHz / BandwidthHz | Double×2 | Hz | 测得载频/占用带宽 | `rf_hz`/`bandwidth_hz` | ✅ |
| PriS / PulseWidthS | Double×2 | s | 估计 PRI / 脉宽 | `pri_s`/`pulse_width_s` | ✅ |
| RfStdHz / BandwidthStdHz / PriStdS / PulseWidthStdS | Double×4 | Hz/Hz/s/s | 四项参数估计标准差 | 同名 | ✅ |
| AmplitudeDb / SnrDb | Double×2 | dB | 接收幅度/观测信噪比 | `amplitude_db`/`snr_db` | ✅ |
| Quality / QualityName | Integer/String64 | — | 观测质量（低/中/高） | `quality` | ✅ |
| WaveformClass / WaveformName | Integer/String64 | — | 波形类别（脉冲/连续/扫频/噪声） | `waveform_class` | ✅ |
| DeceptionClass | Integer | — | 欺骗分类（0 无/1 疑似假目标） | `deception_class` | ✅ |
| ReceiverCenterFreqHz / ReceiverBandwidthHz / ReceiverSaturated | Double/Double/Integer | Hz/Hz/— | 本周期接收调谐中心/带宽/是否饱和 | `ObservationOutputFrame` | ✅ |
| TruthUnitID / TruthUnitName / TruthPlatformName | Integer/String64×2 | — | 真值辐射源归属（评估对照） | 1q ESR：无归属通道，需 AOA/频率自配对；告警机路径：`CyBearingDpDataRecord.UnitID` 天然是辐射源真值 ID | ✘（ESR 路径）/ ✅（告警机路径），见 §5.4 |
| EmitterTypeName | String64 | — | 辐射源型号（真值或推断） | 型号库 / 告警机按 `UnitID` 反查 | ▲ |

> **两条路径的字段边界**：`RfHz`~`PulseWidthStdS`、`Quality`、`WaveformClass`、`DeceptionClass`、`Receiver*` 这些**参数域**列**只有 1q ESR 能填**；告警机路径（`CyQSRWR`/`ZFZ6H01BRWR` + RWR DP）只有 `AoaAzDeg`/`AoaElDeg`/`TruthUnitID`/敌我/是否弹药，且 DP 里功率密度是**硬置 1**（`zfZ6h01brwrdp_process.cpp:113,128`）。写表代码应按"路径"置位，缺的列写 `kNoData`，不要把硬置 1 当成实测幅度。

#### T6-2 `EsrEmitterRecord` — 辐射源假设与威胁等级（每假设每周期一行）

用途：辐射源识别正确率、威胁等级判定、模式推断、方位交叉定位。

| 列名 | 类型 | 单位 | 含义 | 写入来源 | 状态 |
|---|---|---|---|---|---|
| RunNum / CycleIndex / TimeSec | — | — | 键 | 引擎 | ✅ |
| Team / UnitID / UnitName / PlatformName / SensorName | — | — | 侦察设备 | 引擎 | ✅ |
| HypothesisID | Long | — | 假设唯一标识 | `hypothesis_id` | ✅ |
| EmitterClass | String64 | — | 首选候选类别 | `candidate_classes[0]` | ✅ |
| CandidateClasses | String128 | — | 候选类别（按置信度降序，`;` 分隔 ≤4） | `candidate_classes` | ✅ |
| Mode / ModeName | Integer/String64 | — | 工作模式假设（搜索/跟踪/制导/连续照射） | `mode` | ✅ |
| ThreatLevel / ThreatName | Integer/String64 | — | 威胁等级（低/中/高） | `threat_level` | ✅ |
| BearingAzDeg / BearingElDeg / BearingStdDeg | Double×3 | deg | 方位线及标准差 | `bearing_*` | ✅ |
| EstCenterFreqHz / EstBandwidthHz / EstPriS / EstPulseWidthS | Double×4 | Hz/Hz/s/s | 估计参数 | `estimated_*` | ✅ |
| CenterFreqStdHz / BandwidthStdHz / PriStdS / PulseWidthStdS | Double×4 | — | 估计标准差 | `*_std_*` | ✅ |
| Confidence | Double | — | 假设置信度 [0,1] | `confidence` | ✅ |
| LastSeenCycle / WaveformClass | Integer/Integer | — | 最近命中周期/波形类别 | `last_seen_cycle`/`waveform_class` | ✅ |
| TruthEmitterUnitID / TruthEmitterName | Integer/String64 | — | 真值辐射源（评估对照） | 同 T6-1 | ✘ |

#### T6-3 `SensorPowerRecord` — 传感器与雷达开关机事件

用途：开机率、辐射暴露时间、静默有效性、开关机与探测效果的时间关联——**电子对抗评估的核心时间轴**。**每次状态变化写一行**（含初始开机行）。

| 列名 | 类型 | 单位 | 含义 | 写入来源 | 状态 |
|---|---|---|---|---|---|
| RunNum / CycleIndex / TimeSec / EventID | — | — | 键 | 引擎 | ✅ |
| Team / UnitID / UnitName / PlatformName | — | — | 设备所属实体 | 引擎 | ✅ |
| SensorName / SensorType / SensorIndex | String64×2/Integer | — | 设备名、型号、编成内序号 | 装备编成 | ✅ |
| EventType / EventName | Integer/String64 | — | 1 开机/2 关机/3 模式切换（4 静默/5 恢复见下方说明） | 状态机 | ✅ 1–3；✘ 4–5 |
| PowerBefore / PowerAfter | Integer×2 | — | 事件前后电源状态（0/1） | `sensor_enabled` | ✅ |
| ModeBefore / ModeAfter | String64×2 | — | 事件前后工作模式 | 会话配置 `work_mode` | ✅ 配置侧；▲ 运行期生效模式仅 AR/RIR 有输出 |
| RadiationFlag | Integer | — | 事件后是否处于辐射状态 | 派生（`sensor_enabled` ∧ 周期 `status != kPoweredOff`） | ✅ |
| ReasonCode / ReasonName | Integer/String64 | — | 指令/自动/任务/毁伤/故障 | 集成侧 | ▲ 需约定枚举 |
| OriginUnitID / OriginUnitName | Integer/String64 | — | 下令方（指挥单元） | 指令来源 | ✅ |
| DurationSec | Double | s | 上一状态持续时长（切换时回填） | 时间差 | ✅ |

> **口径注意（三条，容易踩）**：
> 1. **"模式"不等于"关机"**：SBIRS 的 `work_mode == kStandby` 等价关机（`src/sbirs_sensor/pipeline/SbirsPipeline.cpp:925`），但 AR 的 `kStby` **不停发**（`ScanScheduleResolver.cpp:109-115` 只把指向钉到 boresight，pipeline 照常发射）。写表时必须同时取 `sensor_enabled` 与周期 `status`，不能只看模式。
> 2. **"静默/辐射管制"没有模型**：`EventType=4/5` 目前**没有任何模型能产生**。1q 明确不实现 T/R 消隐与扇区禁射（`docs/airborne_radar/algorithms.md:143,331`），`src/` 中 `blank|silent` 仅两处且都是 ESR 对"发射端零功率"的诊断归因（`EsrIssueCause::kTransmitSilent`）。要评估"静默有效性"，只能靠 `PowerAfter=0` 的时间段反推。
> 3. **周期状态也是状态源**：七个模块的周期状态都有 `kPoweredOff`（`ArCycleStatus`/`EsrCycleExecutionStatus`/`EosCycleStatus`/`SbirsCycleStatus`/`RirCycleStatus`/`SarCycleStatus`/`EcmCycleStatus`），因此"设备当周期是否在工作"可以不查这张事件表，直接看周期结果。

写入触发点（三条，取其一即可，建议都接）：

1. **1q 侧**：`TryApplyRuntimeConfig` 带 `has_sensor_enabled` 的补丁提交时（`EsrRuntimeConfigPatch.h:72-73`，其余五个模块同构），由集成侧在同一个补丁点写表；
2. **宿主侧（注意：这条目前是空的）**：`CySensorEmissionEventRecord` 声明了 `RADAR_TURNON=4`，但**全仓库没有任何模型发出 TURNON 事件**；实际只在导弹/武器模型里发出 `RADAR_LOCK`（`zFZ6h01baamws_process.cpp:1057-1058`、`FZmphqwsamissile_process.cpp:517-518`、`cosimfcplatform_samisslileprocess.cpp:551-552`），供告警机消费。**不要把 TURNON 当作现成开关机源**；
3. **业务侧**：指挥模型（`Cognition/*FU`）下达开关机指令时，或由实体状态变化（毁伤/故障）触发。

---

## 4. 表 ↔ 数据来源对照矩阵

| 表 | 1q 模型库 | BJ 胶水/业务模型 | 宿主引擎 | 现有老表可复用 |
|---|---|---|---|---|
| T1-1 实体基础信息 | — | 型号库/编成 | 想定 `.sdata` + `.dscript` | — |
| T1-2 实体状态 | — | 平台模型 | `GetUnitInfo()`（`position_`/`velocity_`/`orientation_`/`rcs_`/`active_`/`alive_`/`unit_type_`） | `FixWingPlatformInfo`（单模型、3 列恒 0、有跨结构 offset 缺陷）、`TargetRecord`（无阵营/速度/类型） |
| T2-1 雷达探测 | RIR（航迹级量测）；AR ✘ 无量测 DTO | `LRDR_SENSOR`、各 `*Radar` | 平台/目标几何 | `DetectRecord`、`LOVRecord`、`DetectProbilityRecord` |
| T2-2 雷达航迹 | RIR / AR | `LRDR_SENSOR`、`FZ9Y_GRadar` | — | `TrackInfoFlowRecord`、`LrdrRecognitionRecord`；`SbirsFusionTrackRecord`（**写了但没注册**，见 G11） |
| T3-1 毁伤 | ✘ 无 | 弹药/引信模型 | `CyberEffectImpl::DeterminePlatformDamage`、`StdEffectPK`/`CyEFPK`/`CyEFAgentEffect` | `DaiWaDengLuFuQiaoRecord`（HP，仅 FZGroundPlt）、`SAMRecord.EventName`（文本"目标已命中/未命中"）、`TACPRecord.DestroyedTargetNum`；**`AF_Hit` 注册但零写入** |
| T4-1 武器发射 | ✘ 无 | `WeaponSystems`、`Munitions`、`*FU` | `MunitionModeType`、`CyberEngagementInfoType` | `SAMRecord`（5 个 FU + 弹模型写，交战事件）、`DaiWaDengLuRecord`（4 个模型写发射/命中+价格）、`MunitionInfo`（弹道采样）、`FireControlInfoRecord`/`MidCourseInfoRecord`/`AssignTargetInfoRecord`/`FZMPHRecord`；**`AF_Launch`/`AAMMissedRecord` 注册但零写入，`Stealth*` 未注册** |
| T5-1/5-2 空域 | ✘ 无 | — | `CySZone` + `CyberAirspaceImpl` | — |
| T5-3 空域事件 | ✘ 无 | 平台机动/任务 | 任务 `EnterAirspaceInfo` + `CySZone::Inside` 查询 | — |
| T6-1/6-2 电子侦察 | ESR（参数级：载频/PRI/脉宽 + 假设） | 告警机 `CyQSRWR`/`ZFZ6H01BRWR` + `*RWRDP`（仅方位/敌我/弹标识） | `CyBearingDpDataRecord`（内存传参，不落表）、`CySensorEmissionEventRecord` | `CyBearingDPRecord`、`CyBearingDpDataRecord`（被动方位） |
| T6-3 开关机 | 全模块 `sensor_enabled`（静态配置） | 各传感器胶水；EWSIM 平台 `R_Power_On` | `RadarStatusReport` 消息链 + `CyberDMGRC` | — |

---

## 5. 当前模型能力分析

### 5.1 1q 模型库：传感器侧字段够，评估侧只认红外双星

| 模块 | 评估相关能力 | 关键结构与证据 | 对本表的贡献 |
|---|---|---|---|
| RIR 远程识别雷达 | 检测（SNR/Pd/协方差）、跟踪（IMM、位置/速度/协方差）、四维特征识别（RCS/运动/极化/距离像）、识别效能摘要（含正确率，需 `has_ground_truth`） | `RirFeatureMeasurementRecord`（`RirFeatureMeasurementTypes.h:108-122`）、`RirTrackAttributionRecord`（`RirOutputTypes.h:103-115`）、`RirRecognitionCycleSummary`（`RirRecognitionResult.h:115-131`）、`LrdrRecognitionRecord` 已落表 | T2-1、T2-2 |
| AR 机载雷达 | 航迹状态、速度/加速度、RCS、不确定度、关联质量 11 项、干扰观测（J/N、欺骗分类、RGPO/VGPO 特征）、生效工作模式 | `TrackStateSnapshot`（`TrackStateSnapshot.h:30-68`）、`ArInterferenceObservation`（`ArInterferenceObservation.h:37-59`）、`AssociationQualityMetrics`（`ArOutputTypes.h:92-103`）、`effective_work_mode`（`ArCycleResult.h:90`） | T2-2、T6-3；**T2-1 无量测可写** |
| ESR 电子侦察 | 观测（AOA、载频、带宽、PRI、脉宽、幅度、SNR、质量、波形、欺骗分类）、辐射源假设（候选类、模式、威胁等级、置信度）、工作模式 kEsm/kHgesm/kRwr | `EmitterObservation`（`EmitterObservation.h:54-72`）、`EmitterHypothesis`（`EmitterHypothesis.h:43-62`）、`EsrMissionConfig.h:24` | T6-1、T6-2 |
| SBIRS 红外预警 | 星上探测（ECI 方位/俯仰/IR SNR/观测阶段，**无距离**）、归属诊断（估计距离、最大探测距离、焦平面脱靶量、捕获失败原因、NIS） | `SbirsDetectionRecord`（`SbirsOutputTypes.h:33-41`）、`SbirsDetectionAttributionRecord`（`:76-109`） | （相邻；T6-3 的关机源） |
| EOS 光电 | 探测（距离/方位/俯仰/三路 SNR）、检测生命周期、归属 | `EosDetectionRecord`（`EosOutputTypes.h:22-31`） | （相邻） |
| SAR | 成像产品、分辨率/熵/对比度元数据，无探测报告 | `SarOutputFrame`（`SarCycleResult.h:134-155`） | （相邻） |
| ECM 电子对抗 | 干扰技术（spot/barrage/sweep/deception）、欺骗拖引状态机、资源决策（通道/功率）、实际发射帧；功率开关 `power_on` | `EcmTypes.h:26-53,152-188`、`EcmEsrAdapter.h:24-25` | T6-3（开关机源之一） |
| fusion / threat / target_inference | 融合航迹（LLA+ECEF 速度+6×6 协方差）、威胁分与 3 级等级、推演关键点与不确定度 | `FusedTarget.h:66`、`ThreatResult.h:40`、`InferenceResult.h:32-63` | （相邻，支撑评估取数） |
| precision_evaluation | **五项精度指标 + AHP 综合分**（角度/双星交会/速度/落点/发射点），逐周期样本 + 报告 | `PrecisionEvaluationTypes.h:50-56`、`:62-154` | 评估侧消费方，但**只覆盖 SBIRS 双星链路** |

**共性优势**：所有传感器模块的会话都是"每周期输入 → 每周期输出"，`cycle_index`/`dt_sec` 与调用方给的时间齐备（`docs/review/functional_acceptance_test_criteria_ocr.md:349`），因此**周期表可以按统一节奏落盘**，无需额外对齐机制；`sim_time` 的唯一权威来源是调用方输入，库内无时钟。

**共性缺口（三条，直接影响本设计）**：

1. **1q 的评估模块只认红外双星**：`PrecisionEvaluationSession::Step` 的入参类型直接是两路 `SbirsCycleResult`，唯一适配器是 `SbirsBearingAdapter`（`PrecisionEvaluationSession.h:62-68`、`SbirsBearingAdapter.h:33`）。**AR/ESR/EOS/SAR/RIR/ECM 的产品进不了这个评估模块**，其指标集也不含探测率、虚警率、航迹连续性/ID 保持、识别正确率、时延。→ 本设计的表不是"给 1q 评估模块喂数据"，而是**给宿主效能评估与离线复盘喂数据**；两者互补不重叠。
2. **AR 没有逐探测/量测 DTO**：`include/1q/airborne_radar/**` 内无 detection 记录类型（`grep [Dd]etection` 只命中配置/文档字符串），只有"航迹行"。
3. **1q 无毁伤 / 武器发射 / 空域概念**（§1.3 已排除在 1q 边界外，属设计如此）；ECM 还明确**不产出 J/S、烧穿距离、压制比、干扰成功概率**（`docs/electronic_countermeasure/design.md:147-149`），只有 AR 侧实测 `jammer_to_noise_db`（J/N）。所以干扰效能评估必须由评估层用"有/无干扰两次运行对比"来做。

### 5.2 BJ 集成胶水与业务模型：写表路径最顺，覆盖最不齐

- **已接 1q 的三处**：`Equipment/Sensors/LRDR_SENSOR/lrdr_sensor_1q.cpp`、`Equipment/Sensors/OPIR_SENSOR/opir_sensor_1q.cpp`、`Equipment/DataProcessors/SBIRSFusionDp/sbirsfusiondp_1q.cpp`（均被 `#ifdef CYMODEL_WITH_1Q` 包住）；`LRDRJammer` **并未接 1q**——它只是参数容器，`Process()` 空实现（`lrdrjammer_exec.cpp:3-6`），由 LRDR 侧 Query `"GETDATA"` 取参数后装配成 1q 的 RF 场景帧（`lrdr_sensor_1q.cpp:681-1040`）。
- **已有 1q 落表先例（一个成功、一个失败的）**：成功的是 `OpirDetectRecord`（`opir_sensor_1q.cpp:486-500`）与 `LrdrRecognitionRecord`（`lrdr_sensor_1q.cpp:1676-1692`）。**失败的是融合航迹**：`sbirsfusiondp_1q.cpp:893` 用 `GenerateRecorder("SbirsFusionTrackRecord", …)` 写出了 100 多字节的融合航迹行，但该表名**只在 `SBIRSFusionDpData.hpp:223` 用 `#define` 定义过、从未 `RegisterRelevanceTable`** → 表不存在，数据白写。**这是本文 §2.1"三步缺一不可"最现实的教训**。
- **ESR / AR / EOS / SAR 尚未接入 BJ**：`ModelSource/include/1q/**` 只是头文件部署副本，业务侧没有对应胶水模型。因此 **T6-1/T6-2 的当前状态是"表能定义、字段有源、但没有实体在跑 ESR 会话"**——要么先加一个 `ESR_SENSOR` 胶水模型，要么这两张表在场景里长期空表。
- **宿主自带一条"被动告警"链路（不是 1q ESR）**：告警接收机模型 `Sensors/CyQSRWR` 与 `Sensors/ZFZ6H01BRWR`，各配一个 RWR 数据处理器（`DataProcessors/CyQDSimpleRWRDP`、`DataProcessors/ZFZ6H01BRWRDP`）。以引擎 `SENSOR_EVENT` 为输入，算本机体系下的辐射源方位/俯仰，置敌我/弹药标志，经 `CYBER_BEARINGDP_DATA_RECORD` 交给 DP（`cyqsrwr_process.cpp:37-110`、`zFZ6h01brwr_process.cpp:40-102`）。四个坑：①事件类型过滤被注释，**任何发射事件都当告警**（`cyqsrwr_process.cpp:62-71` 的 "TOTO"）；②`R_Power_On`/频率/PRI/脉宽在这条链上不存在，DP 里的功率密度被**硬置 1**（`zfZ6h01brwrdp_process.cpp:113,128`）；③RWR 与 DP **都不写任何记录表**，`CyBearingDpDataRecord` 只是内存传参；④`Record/Sensor/` 下确实声明了完整 ESM 接口（`ElecReconProcess`、`RADIATEPARA` 带 `m_RadioFreqency`/`m_SignalPRF`/`m_PulseWidth`，`model_typedef.h:441-466`），但**没有实现、也没有调用方**。→ T6-1 在"只有告警机"的场景里可作过渡源（方位 + `TruthUnitID` 天然可填），**参数域（载频/PRI/脉宽）和 T6-2 完全无源**。
- **老雷达模型的探测表**：`FZBSCRadar`/`FZ9Y_GRadar`/`CyQSRadar` 等已在写 `DetectRecord`/`LOVRecord`/`DetectProbilityRecord`（如 `zFZ6h01bacradar_detect.cpp:643-673`、`FZ9y_gradar_detect.cpp:566-594`），可作为 T2-1 的**过渡数据源**（字段少一半，真值几何要靠 `LOVRecord` 补）。

### 5.3 宿主引擎：记录机制与交战/空间事实都在它手上，但多数是零件不是成品

| 能力 | 证据 | 结论 |
|---|---|---|
| 记录注册/落盘/SQL 导出 | `OutputRecord.hpp` + `InitlizeRecord` + `GenerateRecorder`（`cyber_mount_model_impl.h:427-431`） | ✅ 新表零成本接入；✘ 未注册表名会静默丢数据（G11）；✘ `Environment/Airspaces/init/fosim_model_manager.cpp:44-49` **不调用** `InitlizeRecord`，空域插件自身没有表 |
| 空域模型 | `CySZone`（`Type` = 1 责任区/2 导弹交战区/3 监视区；`AreaType` = 1 多边形/2 圆形/3 扇形；`FloorMSL/CeilingMSL/BelongUnit/IsBound/Points`，`CySZone.mdef:83-138`）、任务侧 `EnterAirspaceInfo`（`Mission/*.mdata:55-58`）、`CySZone::Inside()`（`cyszone_inside.cpp:15-47`，射线法/圆内含高度门限） | ✅ 定义与"点是否在区内"可用；✘ 无阵营、无有效期（基类有 `DTGBegin/End` 但 `CySZone` 从不读写）、**扇形(3) 未实现**、`Inside()` 的 0/1 与 mdef 的 1/2 枚举不一致、无进出事件记录 |
| 毁伤判定 | `CyberDMGRC`（4 值，`cyber_enum_type.h:47-53`）、`CyberEffectImpl::DeterminePlatformDamage`（`cyber_effect_impl.h:108-125`）、`StdEffectPK`（`LethalRadius/PKImpact/PKLethalRadius/MaxRadius`，`StdEffectPKData.hpp:17-48`） | ▲ 判定链在，但**三个效果模型都不维护 HP**：`StdEffectPK` 把判定交给目标 `unit->Damage()`（`stdeffectpk_exec.cpp:41`）、`CyEFPK` 只发 `EVALUATE_DAMAGE` MAL（`cyefpk_exec.cpp:159-172`）、`CyEFAgentEffect` 的 `DAMAGED` 分支因 Process 里伤害调用被注释而**不生效**（`cyefagenteffect_exec.cpp:149-152`）；且平台侧 `Damage()` 一律 `return CYBER_DMGRC_UNKNOWN`（如 `zFZ6h01bpltfw_damage.cpp:5-9`）；真正"杀死"目标的是认知侧 `SetActive(CYBER_FALSE)`（`zFZ6h01caamfu_util.cpp:148-178`）。✘ 结果没有事件表 |
| 武器发射 | `Equipment/WeaponSystems`（11 个注册模型）、`Equipment/Munitions`（28 个注册模型）、`MunitionModeType`（`quantity_`/`launcher_unit_id`/`target_unit_id_`/`weapon_system_name_`）、`CyberEngagementInfoType{MunitionID,TargetID,TargetPosition,WeaponSystemID,EngagementID}`（`cyber_struct_type.h:191-206`）、WS 侧齐射序号拼进单元名 + `MunitionNums` 余弹 | ✅ 发射事实可得（发射者/目标/弹单元/距离/齐射序号/余弹/`Mode`）；✘ **`AF_Launch`/`AF_Hit`/`AAMMissedRecord` 注册了但全仓零写入，`Stealth*` 未注册**；`CyberLaunchRecord`（ENGINE RECORD_ID 11015）声明了但从未填充；制导方式无统一字段 |
| 传感器开机/工作状态 | EWSIM 风格平台：`cosimsensor.h:146-148` 的 `R_Power_On`(0/1) 与 `R_WorkState`(0/1)；Alter MAL `"Power_On"` 可改（`cosimtardetplatform_attr.cpp:51-53`）；每次扫描前发 `RadarStatusReport`（`cosimtardetsensor_process.cpp:56-83`，关机直接不扫描）；指挥侧汇总并下发开机指令（`cosimcsamcontroller_Rank.cpp:148-175`） | ▲ **真实开关机链路存在，但只在 CoSimTarDet/CoSimGuiRad 两类平台**；其余雷达（`ZFZ6H01BACRadar` 等）的 `TurnOn` 只是显示开关；1q 侧 `sensor_enabled` 是 JSON 静态值，现胶水**没有整机开关机运行期指令**（只有"指定识别"补丁） |
| 被动告警链路 | `CyQSRWR`/`ZFZ6H01BRWR` + `CyBearingDPRecord`/`CyBearingDpDataRecord`（`cyber_sensor_record.h:257-324`） | ▲ 可作 T6-1 过渡源（方位/敌我/弹标识 + 辐射源真值 ID），无参数域、不落表 |
| 敌我关系/存活/任务状态 | `CyberUnitRelationshipType`、`IsAlive()/IsActive()/GetKillTime()`、`TaskExecuteStatusEnum`（`ICyberUnit.h:195-219`、`cyber_enum_type.h:55-61,175-184`） | ✅ |

### 5.4 缺口清单（按表）

| # | 表 | 缺口 | 性质 | 处理 |
|---|---|---|---|---|
| G1 | T2-1 | 无"点迹级量测"：RIR 只有航迹级（方位/俯仰/斜距/SNR），**AR 完全没有量测 DTO**；无 J/S（库内只产 J/N） | 1q 输出粒度 | 表定在"每雷达×目标×周期"粒度；AR 场景退化为 T2-2 + 航迹反算几何；J/S 由评估层用两次运行对比算 |
| G2 | T2-1/T6-1 | ESR 无归属对照通道（`track/detection_attributions` 缺席） | 1q 契约 | 集成侧按 AOA+频率门限自配对，写 `Truth*` 列；长期建议 1q 给 ESR 增补 `EmitterAttributionRecord` |
| G3 | T3-1 | 无毁伤事件表；`AF_Hit` 零写入；无脱靶量/命中点/PK 抽样值；三个效果模型都不维护 HP，平台 `Damage()` 恒返回 `UNKNOWN` | 宿主能力 | 新增表 + 在 `StdEffectPK`/`CyEFPK` 判定点写行；`PkDraw` 需效果模型透出随机数；`DamageState` 需先修平台 `Damage()` 或改用存活位 |
| G4 | T3-1/T1-2 | 全仓唯一 HP 是 `DaiWaDengLuFuQiaoRecord.HP`（仅 `FZGroundPlt` 写），与效果模型无关 | 模型能力 | 列为"可缺"，评估用 `DamageState`/存活位替代 |
| G5 | T4-1 | 齐射序号/规模/制导方式未成字段（齐射序号只拼在弹单元名里，`"…00"+CountMissle`） | 业务模型 | WS 侧新增显式字段；制导方式从弹药型号数据带出 |
| G6 | T5-1 | `Type` 有语义（1 责任区/2 导弹交战区/3 监视区）但**无阵营、无有效期**；`AreaType=3`（扇形）**未实现**；`Inside()` 的 0/1 与 mdef 的 1/2 枚举不一致 | 宿主数据 | 表里保留宿主原值并加 `UsageTypeName` 映射；有效期列先留空 |
| G7 | T5-3 | 无进出/越界事件记录，需几何判定（`CySZone::Inside` 可复用，但扇形区域要自己算或等宿主修） | 集成侧 | 新增判定组件（按 T5-1/5-2 几何 + T1-2 位置） |
| G8 | T6-3 | 开关机**只有状态、没有事件行**：EWSIM 平台有 `R_Power_On`+`RadarStatusReport` 消息链，其余雷达只有显示开关；`RADAR_TURNON` 枚举无生产者；1q 胶水未接整机开关机指令 | 1q/宿主 | 新表 + 在 `Power_On` 变更点、`RadarStatusReport` 收发点写行 |
| G9 | 全部 | 老表与新表语义重叠，且**老表里有一批零写入的死表**（`AF_Launch`/`AF_Hit`/`Stealth*`/`AAMMissedRecord`/`GG_Plane_Dead`） | 治理 | 新算子只用新表；死表不复活，避免"名字像有数据"的错觉 |
| G10 | 评估侧 | 1q `precision_evaluation` 硬绑 SBIRS 双星，指标集不含探测率/虚警/连续性/识别率/时延 | 1q 评估能力 | 本设计的表交由宿主效能评估消费；若要 1q 侧评估雷达/侦察，需另立指标与适配器，不能复用现有 `PrecisionEvaluationSession` |
| G11 | 全部 | **未注册表名的 `GenerateRecorder` 会静默丢数据**（现成实例：融合航迹 `SbirsFusionTrackRecord` 写了但从未注册；`Radar_List` 注册入口无人调用） | 记录机制 | 每张新表落地时必须同批提交"注册 + 写入 + 一条非空行"的自检（见 §6 验收列） |
| G12 | T5-1/5-3 | 空域插件 `Airspaces` 不调用 `InitlizeRecord` | 插件缺陷 | 在 `Environment/Airspaces/init/fosim_model_manager.cpp` 补一行，否则只加载空域插件时无表 |

### 5.5 一句话能力结论

**探测、侦察、实体状态这三类"传感器算法产物"1q 已经很强，缺的是出口；毁伤、发射、空域这三类"交战与空间管理事实"宿主有零件但没成品表。** 更具体地说：发射事实在火控/武器系统里齐全，却写进了语义不对的表（`SAMRecord` 是交战事件、`MunitionInfo` 是弹道采样），真正叫"发射表"的 `AF_Launch` 是零写入的死表；毁伤只有布尔/文本事件，连"目标现在是损伤还是摧毁"都取决于平台模型有没有 override `Damage()`。因此 12 张表里 10 张能在现有能力范围内解决（4 张纯接线 + 6 张补列补判定），只有 2 张需要新增模型能力。

---

## 6. 落地建议

| 阶段 | 内容 | 前置 | 验收 |
|---|---|---|---|
| P0（接线） | T1-1、T2-2、T5-1、T5-2 落地 + `InitlizeRecord` 注册（**`Environment/Airspaces` 插件同时补上 `InitlizeRecord` 调用**）；同一批把 T6-1 接到告警机路径 | 无（字段全有源） | 跑一遍现有想定，`ResultData.db` 中出现 5 张**非空**表（空表视为失败，见 G11） |
| P1（补列补判定） | T1-2、T2-1（含 `Truth*`）、T6-2、T6-3、T5-3 | P0 完成；ESR 胶水模型或过渡源选定（T6-2）；开关机事件点选定（EWSIM `Power_On` 或指挥指令） | 按 `RunNum+CycleIndex` 能还原雷达受扰与开关机时间线 |
| P2（补能力） | T3-1、T4-1 | 毁伤/发射判定点改造（效果模型透出 PK 抽样值；WS 侧显式写发射行） | 毁伤率、命中率、齐射规模可由 SQL 单表算出 |

工程约定（三仓库分工）：

- **BJ_10Y_WorkSpace**：`OutputRecord.hpp` 追加 12 个结构 + `InitlizeRecord` 追加 12 行；`Environment/Airspaces/init/fosim_model_manager.cpp` 补 `InitlizeRecord(...)`（G12）；模型侧写行代码；关闭宿主后按 `build_x64_v120.cmd` 重编（`docs/practice/fosimstudio_host_and_integration_guide.md:99-109`）；
  顺带把 `SbirsFusionTrackRecord` 的注册补上（G11），否则融合航迹一直是空表；
- **1q**：本节不改代码；仅当决定增补 ESR 归属通道（G2）或 RIR 点迹输出（G1）时才动；
- **评估工程**：在 `AssessmentSource/algorithm.ois` 增算子、`resultheader.istruct` 登记新表头。**每张表上线前先跑一条"计数为非零"的算子**，把"注册漏了/写入没触发"这种静默失败变成显式失败。

风险提示：

1. 12 张表全开会让单次运行的 `ResultData.db` 显著变大，**P0 上线前先按 §2.7 配好采样降频**；
2. `Truth*` 列只允许评估侧写，**禁止回灌给模型**（违反 `docs/common/session_contract.md:177` 的真值隔离）；
3. 新表列名一旦被 `.ois` 算子引用即成为事实契约，**改名成本等同改评估工程**。

---

## 附录 A：可直接并入 `OutputRecord.hpp` 的结构定义

> 说明：为控制篇幅，`Register` 内每列的中文描述只写要点；正式提交时按 §2.4 补全单位。所有成员均为 POD，构造函数 `memset` 清零。`kNoData`/`kNoDataInt` 建议以 `static const` 在结构体内声明（VS2015 下需在 cpp 中给出定义）。

```cpp
// ============ 第一类：实体基础信息 ============
struct EntityBaseInfoRecord
{
	EntityBaseInfoRecord(){ memset(this, 0, sizeof(*this)); }
	CyberBaseType::CyberInt RunNum;
	CyberBaseType::CyberInt EntityID;
	CyberBaseType::CyberChar64 EntityName;
	CyberBaseType::CyberChar64 Team;
	CyberBaseType::CyberChar64 EntityType;
	CyberBaseType::CyberChar64 PlatformName;
	CyberBaseType::CyberChar64 PlatformType;
	CyberBaseType::CyberInt SuperiorUnitID;
	CyberBaseType::CyberChar64 SuperiorUnitName;
	CyberBaseType::CyberInt SensorCount;
	CyberBaseType::CyberChar128 SensorNames;
	CyberBaseType::CyberInt JammerCount;
	CyberBaseType::CyberChar128 JammerNames;
	CyberBaseType::CyberInt WeaponCount;
	CyberBaseType::CyberDouble UnitPrice;
	CyberBaseType::CyberDouble InitLat;
	CyberBaseType::CyberDouble InitLon;
	CyberBaseType::CyberDouble InitAltMSL;
	CyberBaseType::CyberDouble InitHeadingDeg;
	CyberBaseType::CyberDouble StartTimeSec;
	CyberBaseType::CyberDouble EndTimeSec;

	static void Register(CyberDataRecorderFactory *_fact)
	{
		_fact->RegisterRelevanceTable("EntityBaseInfoRecord", "EntityBaseInfoRecord");
		_fact->RegisterTableColumn("RunNum", RECORD_INTEGER, "仿真次数", GETOFFSET(EntityBaseInfoRecord, RunNum));
		_fact->RegisterTableColumn("EntityID", RECORD_INTEGER, "实体单元ID", GETOFFSET(EntityBaseInfoRecord, EntityID));
		_fact->RegisterTableColumn("EntityName", RECORD_STRING, "实体名称", GETOFFSET(EntityBaseInfoRecord, EntityName));
		_fact->RegisterTableColumn("Team", RECORD_STRING, "阵营", GETOFFSET(EntityBaseInfoRecord, Team));
		_fact->RegisterTableColumn("EntityType", RECORD_STRING, "单元类型", GETOFFSET(EntityBaseInfoRecord, EntityType));
		_fact->RegisterTableColumn("PlatformName", RECORD_STRING, "主平台名称", GETOFFSET(EntityBaseInfoRecord, PlatformName));
		_fact->RegisterTableColumn("PlatformType", RECORD_STRING, "主平台模型类名", GETOFFSET(EntityBaseInfoRecord, PlatformType));
		_fact->RegisterTableColumn("SuperiorUnitID", RECORD_INTEGER, "上级单元ID", GETOFFSET(EntityBaseInfoRecord, SuperiorUnitID));
		_fact->RegisterTableColumn("SuperiorUnitName", RECORD_STRING, "上级单元名称", GETOFFSET(EntityBaseInfoRecord, SuperiorUnitName));
		_fact->RegisterTableColumn("SensorCount", RECORD_INTEGER, "传感器数量", GETOFFSET(EntityBaseInfoRecord, SensorCount));
		_fact->RegisterTableColumn("SensorNames", RECORD_STRING, "传感器名称(分号分隔)", GETOFFSET(EntityBaseInfoRecord, SensorNames));
		_fact->RegisterTableColumn("JammerCount", RECORD_INTEGER, "干扰机数量", GETOFFSET(EntityBaseInfoRecord, JammerCount));
		_fact->RegisterTableColumn("JammerNames", RECORD_STRING, "干扰机名称(分号分隔)", GETOFFSET(EntityBaseInfoRecord, JammerNames));
		_fact->RegisterTableColumn("WeaponCount", RECORD_INTEGER, "武器系统数量", GETOFFSET(EntityBaseInfoRecord, WeaponCount));
		_fact->RegisterTableColumn("UnitPrice", RECORD_DOUBLE, "实体价值", GETOFFSET(EntityBaseInfoRecord, UnitPrice));
		_fact->RegisterTableColumn("InitLat", RECORD_DOUBLE, "初始纬度(deg)", GETOFFSET(EntityBaseInfoRecord, InitLat));
		_fact->RegisterTableColumn("InitLon", RECORD_DOUBLE, "初始经度(deg)", GETOFFSET(EntityBaseInfoRecord, InitLon));
		_fact->RegisterTableColumn("InitAltMSL", RECORD_DOUBLE, "初始海拔(m)", GETOFFSET(EntityBaseInfoRecord, InitAltMSL));
		_fact->RegisterTableColumn("InitHeadingDeg", RECORD_DOUBLE, "初始航向(deg)", GETOFFSET(EntityBaseInfoRecord, InitHeadingDeg));
		_fact->RegisterTableColumn("StartTimeSec", RECORD_DOUBLE, "参战时刻(s)", GETOFFSET(EntityBaseInfoRecord, StartTimeSec));
		_fact->RegisterTableColumn("EndTimeSec", RECORD_DOUBLE, "结束时刻(s)", GETOFFSET(EntityBaseInfoRecord, EndTimeSec));
		_fact->RegisterOneTableEnd();
	}
};

struct EntityStateRecord
{
	EntityStateRecord(){ memset(this, 0, sizeof(*this)); }
	CyberBaseType::CyberInt RunNum;
	CyberBaseType::CyberInt CycleIndex;
	CyberBaseType::CyberDouble TimeSec;
	CyberBaseType::CyberInt EntityID;
	CyberBaseType::CyberChar64 EntityName;
	CyberBaseType::CyberChar64 Team;
	CyberBaseType::CyberChar64 EntityType;
	CyberBaseType::CyberChar64 PlatformName;
	CyberBaseType::CyberDouble Lat;
	CyberBaseType::CyberDouble Lon;
	CyberBaseType::CyberDouble AltMSL;
	CyberBaseType::CyberDouble EcefX;
	CyberBaseType::CyberDouble EcefY;
	CyberBaseType::CyberDouble EcefZ;
	CyberBaseType::CyberDouble EcefVx;
	CyberBaseType::CyberDouble EcefVy;
	CyberBaseType::CyberDouble EcefVz;
	CyberBaseType::CyberDouble SpeedMps;
	CyberBaseType::CyberDouble HeadingDeg;
	CyberBaseType::CyberDouble PitchDeg;
	CyberBaseType::CyberDouble RollDeg;
	CyberBaseType::CyberInt Alive;
	CyberBaseType::CyberInt DamageState;
	CyberBaseType::CyberDouble HpRatio;
	CyberBaseType::CyberChar64 RelationName;
	CyberBaseType::CyberInt ActiveSensorCount;
	CyberBaseType::CyberInt PositionSource;

	static void Register(CyberDataRecorderFactory *_fact)
	{
		_fact->RegisterRelevanceTable("EntityStateRecord", "EntityStateRecord");
		_fact->RegisterTableColumn("RunNum", RECORD_INTEGER, "仿真次数", GETOFFSET(EntityStateRecord, RunNum));
		_fact->RegisterTableColumn("CycleIndex", RECORD_INTEGER, "周期号", GETOFFSET(EntityStateRecord, CycleIndex));
		_fact->RegisterTableColumn("TimeSec", RECORD_DOUBLE, "仿真时刻(s)", GETOFFSET(EntityStateRecord, TimeSec));
		_fact->RegisterTableColumn("EntityID", RECORD_INTEGER, "实体单元ID", GETOFFSET(EntityStateRecord, EntityID));
		_fact->RegisterTableColumn("EntityName", RECORD_STRING, "实体名称", GETOFFSET(EntityStateRecord, EntityName));
		_fact->RegisterTableColumn("Team", RECORD_STRING, "阵营", GETOFFSET(EntityStateRecord, Team));
		_fact->RegisterTableColumn("EntityType", RECORD_STRING, "单元类型", GETOFFSET(EntityStateRecord, EntityType));
		_fact->RegisterTableColumn("PlatformName", RECORD_STRING, "平台名称", GETOFFSET(EntityStateRecord, PlatformName));
		_fact->RegisterTableColumn("Lat", RECORD_DOUBLE, "纬度(deg)", GETOFFSET(EntityStateRecord, Lat));
		_fact->RegisterTableColumn("Lon", RECORD_DOUBLE, "经度(deg)", GETOFFSET(EntityStateRecord, Lon));
		_fact->RegisterTableColumn("AltMSL", RECORD_DOUBLE, "海拔(m)", GETOFFSET(EntityStateRecord, AltMSL));
		_fact->RegisterTableColumn("EcefX", RECORD_DOUBLE, "ECEF X(m)", GETOFFSET(EntityStateRecord, EcefX));
		_fact->RegisterTableColumn("EcefY", RECORD_DOUBLE, "ECEF Y(m)", GETOFFSET(EntityStateRecord, EcefY));
		_fact->RegisterTableColumn("EcefZ", RECORD_DOUBLE, "ECEF Z(m)", GETOFFSET(EntityStateRecord, EcefZ));
		_fact->RegisterTableColumn("EcefVx", RECORD_DOUBLE, "ECEF Vx(m/s)", GETOFFSET(EntityStateRecord, EcefVx));
		_fact->RegisterTableColumn("EcefVy", RECORD_DOUBLE, "ECEF Vy(m/s)", GETOFFSET(EntityStateRecord, EcefVy));
		_fact->RegisterTableColumn("EcefVz", RECORD_DOUBLE, "ECEF Vz(m/s)", GETOFFSET(EntityStateRecord, EcefVz));
		_fact->RegisterTableColumn("SpeedMps", RECORD_DOUBLE, "速度模长(m/s)", GETOFFSET(EntityStateRecord, SpeedMps));
		_fact->RegisterTableColumn("HeadingDeg", RECORD_DOUBLE, "航向(deg)", GETOFFSET(EntityStateRecord, HeadingDeg));
		_fact->RegisterTableColumn("PitchDeg", RECORD_DOUBLE, "俯仰(deg)", GETOFFSET(EntityStateRecord, PitchDeg));
		_fact->RegisterTableColumn("RollDeg", RECORD_DOUBLE, "横滚(deg)", GETOFFSET(EntityStateRecord, RollDeg));
		_fact->RegisterTableColumn("Alive", RECORD_INTEGER, "存活标志", GETOFFSET(EntityStateRecord, Alive));
		_fact->RegisterTableColumn("DamageState", RECORD_INTEGER, "毁伤等级(0未知/1未受影响/2摧毁/3损伤)", GETOFFSET(EntityStateRecord, DamageState));
		_fact->RegisterTableColumn("HpRatio", RECORD_DOUBLE, "剩余生命值比例", GETOFFSET(EntityStateRecord, HpRatio));
		_fact->RegisterTableColumn("RelationName", RECORD_STRING, "敌我关系", GETOFFSET(EntityStateRecord, RelationName));
		_fact->RegisterTableColumn("ActiveSensorCount", RECORD_INTEGER, "开机传感器数", GETOFFSET(EntityStateRecord, ActiveSensorCount));
		_fact->RegisterTableColumn("PositionSource", RECORD_INTEGER, "位置来源(0真值/1模型)", GETOFFSET(EntityStateRecord, PositionSource));
		_fact->RegisterOneTableEnd();
	}
};

// ============ 第二类：雷达探测信息 ============
struct RadarDetectionRecord
{
	RadarDetectionRecord(){ memset(this, 0, sizeof(*this)); }
	CyberBaseType::CyberInt RunNum;
	CyberBaseType::CyberInt CycleIndex;
	CyberBaseType::CyberDouble TimeSec;
	CyberBaseType::CyberChar64 Team;
	CyberBaseType::CyberInt UnitID;
	CyberBaseType::CyberChar64 UnitName;
	CyberBaseType::CyberChar64 PlatformName;
	CyberBaseType::CyberChar64 SensorName;
	CyberBaseType::CyberChar64 SensorType;
	CyberBaseType::CyberChar64 WorkMode;
	CyberBaseType::CyberChar64 TargetTeam;
	CyberBaseType::CyberInt TargetUnitID;
	CyberBaseType::CyberChar64 TargetUnitName;
	CyberBaseType::CyberChar64 TargetPlatformName;
	CyberBaseType::CyberLong DetectionID;
	CyberBaseType::CyberInt Detected;
	CyberBaseType::CyberDouble RangeM;
	CyberBaseType::CyberDouble RangeRateMps;
	CyberBaseType::CyberDouble AzDeg;
	CyberBaseType::CyberDouble ElDeg;
	CyberBaseType::CyberDouble SnrDb;
	CyberBaseType::CyberDouble Pd;
	CyberBaseType::CyberDouble RcsM2;
	CyberBaseType::CyberDouble RangeSigmaM;
	CyberBaseType::CyberDouble AzSigmaDeg;
	CyberBaseType::CyberDouble ElSigmaDeg;
	CyberBaseType::CyberInt Jammed;
	CyberBaseType::CyberDouble JnDb;
	CyberBaseType::CyberInt DeceptionClass;
	CyberBaseType::CyberDouble TruthRangeM;
	CyberBaseType::CyberDouble TruthAzDeg;
	CyberBaseType::CyberDouble TruthElDeg;
	CyberBaseType::CyberChar64 BlockedCause;

	static void Register(CyberDataRecorderFactory *_fact)
	{
		_fact->RegisterRelevanceTable("RadarDetectionRecord", "RadarDetectionRecord");
		_fact->RegisterTableColumn("RunNum", RECORD_INTEGER, "仿真次数", GETOFFSET(RadarDetectionRecord, RunNum));
		_fact->RegisterTableColumn("CycleIndex", RECORD_INTEGER, "周期号", GETOFFSET(RadarDetectionRecord, CycleIndex));
		_fact->RegisterTableColumn("TimeSec", RECORD_DOUBLE, "仿真时刻(s)", GETOFFSET(RadarDetectionRecord, TimeSec));
		_fact->RegisterTableColumn("Team", RECORD_STRING, "雷达阵营", GETOFFSET(RadarDetectionRecord, Team));
		_fact->RegisterTableColumn("UnitID", RECORD_INTEGER, "雷达单元ID", GETOFFSET(RadarDetectionRecord, UnitID));
		_fact->RegisterTableColumn("UnitName", RECORD_STRING, "雷达单元名称", GETOFFSET(RadarDetectionRecord, UnitName));
		_fact->RegisterTableColumn("PlatformName", RECORD_STRING, "雷达平台名称", GETOFFSET(RadarDetectionRecord, PlatformName));
		_fact->RegisterTableColumn("SensorName", RECORD_STRING, "传感器名称", GETOFFSET(RadarDetectionRecord, SensorName));
		_fact->RegisterTableColumn("SensorType", RECORD_STRING, "传感器型号", GETOFFSET(RadarDetectionRecord, SensorType));
		_fact->RegisterTableColumn("WorkMode", RECORD_STRING, "工作模式", GETOFFSET(RadarDetectionRecord, WorkMode));
		_fact->RegisterTableColumn("TargetTeam", RECORD_STRING, "目标阵营", GETOFFSET(RadarDetectionRecord, TargetTeam));
		_fact->RegisterTableColumn("TargetUnitID", RECORD_INTEGER, "目标单元ID", GETOFFSET(RadarDetectionRecord, TargetUnitID));
		_fact->RegisterTableColumn("TargetUnitName", RECORD_STRING, "目标单元名称", GETOFFSET(RadarDetectionRecord, TargetUnitName));
		_fact->RegisterTableColumn("TargetPlatformName", RECORD_STRING, "目标平台名称", GETOFFSET(RadarDetectionRecord, TargetPlatformName));
		_fact->RegisterTableColumn("DetectionID", RECORD_INTEGER, "探测记录ID", GETOFFSET(RadarDetectionRecord, DetectionID));
		_fact->RegisterTableColumn("Detected", RECORD_INTEGER, "探测结论(0未探测/1探测/2受扰)", GETOFFSET(RadarDetectionRecord, Detected));
		_fact->RegisterTableColumn("RangeM", RECORD_DOUBLE, "斜距(m)", GETOFFSET(RadarDetectionRecord, RangeM));
		_fact->RegisterTableColumn("RangeRateMps", RECORD_DOUBLE, "径向速度(m/s)", GETOFFSET(RadarDetectionRecord, RangeRateMps));
		_fact->RegisterTableColumn("AzDeg", RECORD_DOUBLE, "方位角(deg)", GETOFFSET(RadarDetectionRecord, AzDeg));
		_fact->RegisterTableColumn("ElDeg", RECORD_DOUBLE, "俯仰角(deg)", GETOFFSET(RadarDetectionRecord, ElDeg));
		_fact->RegisterTableColumn("SnrDb", RECORD_DOUBLE, "信噪比(dB)", GETOFFSET(RadarDetectionRecord, SnrDb));
		_fact->RegisterTableColumn("Pd", RECORD_DOUBLE, "检测概率", GETOFFSET(RadarDetectionRecord, Pd));
		_fact->RegisterTableColumn("RcsM2", RECORD_DOUBLE, "RCS(m^2)", GETOFFSET(RadarDetectionRecord, RcsM2));
		_fact->RegisterTableColumn("RangeSigmaM", RECORD_DOUBLE, "测距标准差(m)", GETOFFSET(RadarDetectionRecord, RangeSigmaM));
		_fact->RegisterTableColumn("AzSigmaDeg", RECORD_DOUBLE, "测角方位标准差(deg)", GETOFFSET(RadarDetectionRecord, AzSigmaDeg));
		_fact->RegisterTableColumn("ElSigmaDeg", RECORD_DOUBLE, "测角俯仰标准差(deg)", GETOFFSET(RadarDetectionRecord, ElSigmaDeg));
		_fact->RegisterTableColumn("Jammed", RECORD_INTEGER, "是否受扰", GETOFFSET(RadarDetectionRecord, Jammed));
		_fact->RegisterTableColumn("JnDb", RECORD_DOUBLE, "干噪比(dB)", GETOFFSET(RadarDetectionRecord, JnDb));
		_fact->RegisterTableColumn("DeceptionClass", RECORD_INTEGER, "欺骗分类", GETOFFSET(RadarDetectionRecord, DeceptionClass));
		_fact->RegisterTableColumn("TruthRangeM", RECORD_DOUBLE, "真值斜距(m)", GETOFFSET(RadarDetectionRecord, TruthRangeM));
		_fact->RegisterTableColumn("TruthAzDeg", RECORD_DOUBLE, "真值方位角(deg)", GETOFFSET(RadarDetectionRecord, TruthAzDeg));
		_fact->RegisterTableColumn("TruthElDeg", RECORD_DOUBLE, "真值俯仰角(deg)", GETOFFSET(RadarDetectionRecord, TruthElDeg));
		_fact->RegisterTableColumn("BlockedCause", RECORD_STRING, "未探测主因", GETOFFSET(RadarDetectionRecord, BlockedCause));
		_fact->RegisterOneTableEnd();
	}
};

struct RadarTrackRecord
{
	RadarTrackRecord(){ memset(this, 0, sizeof(*this)); }
	CyberBaseType::CyberInt RunNum;
	CyberBaseType::CyberInt CycleIndex;
	CyberBaseType::CyberDouble TimeSec;
	CyberBaseType::CyberChar64 Team;
	CyberBaseType::CyberInt UnitID;
	CyberBaseType::CyberChar64 UnitName;
	CyberBaseType::CyberChar64 PlatformName;
	CyberBaseType::CyberChar64 SensorName;
	CyberBaseType::CyberLong TrackID;
	CyberBaseType::CyberInt TrackStatus;
	CyberBaseType::CyberChar64 TrackStatusName;
	CyberBaseType::CyberInt TruthUnitID;
	CyberBaseType::CyberChar64 TruthUnitName;
	CyberBaseType::CyberDouble PosEnuXM;
	CyberBaseType::CyberDouble PosEnuYM;
	CyberBaseType::CyberDouble PosEnuZM;
	CyberBaseType::CyberDouble VelXMps;
	CyberBaseType::CyberDouble VelYMps;
	CyberBaseType::CyberDouble VelZMps;
	CyberBaseType::CyberDouble SpeedMps;
	CyberBaseType::CyberDouble AccXMps2;
	CyberBaseType::CyberDouble AccYMps2;
	CyberBaseType::CyberDouble AccZMps2;
	CyberBaseType::CyberDouble RangeM;
	CyberBaseType::CyberDouble AzDeg;
	CyberBaseType::CyberDouble ElDeg;
	CyberBaseType::CyberDouble PosUncertaintyM;
	CyberBaseType::CyberInt HitCount;
	CyberBaseType::CyberInt MissCount;
	CyberBaseType::CyberDouble TrackSnrDb;
	CyberBaseType::CyberInt RecogCategory;
	CyberBaseType::CyberChar64 RecogModel;
	CyberBaseType::CyberDouble RecogConfidence;
	CyberBaseType::CyberDouble FirstDetectTimeSec;
	CyberBaseType::CyberDouble LastUpdateTimeSec;

	static void Register(CyberDataRecorderFactory *_fact)
	{
		_fact->RegisterRelevanceTable("RadarTrackRecord", "RadarTrackRecord");
		_fact->RegisterTableColumn("RunNum", RECORD_INTEGER, "仿真次数", GETOFFSET(RadarTrackRecord, RunNum));
		_fact->RegisterTableColumn("CycleIndex", RECORD_INTEGER, "周期号", GETOFFSET(RadarTrackRecord, CycleIndex));
		_fact->RegisterTableColumn("TimeSec", RECORD_DOUBLE, "仿真时刻(s)", GETOFFSET(RadarTrackRecord, TimeSec));
		_fact->RegisterTableColumn("Team", RECORD_STRING, "雷达阵营", GETOFFSET(RadarTrackRecord, Team));
		_fact->RegisterTableColumn("UnitID", RECORD_INTEGER, "雷达单元ID", GETOFFSET(RadarTrackRecord, UnitID));
		_fact->RegisterTableColumn("UnitName", RECORD_STRING, "雷达单元名称", GETOFFSET(RadarTrackRecord, UnitName));
		_fact->RegisterTableColumn("PlatformName", RECORD_STRING, "雷达平台名称", GETOFFSET(RadarTrackRecord, PlatformName));
		_fact->RegisterTableColumn("SensorName", RECORD_STRING, "传感器名称", GETOFFSET(RadarTrackRecord, SensorName));
		_fact->RegisterTableColumn("TrackID", RECORD_INTEGER, "库内航迹关联键", GETOFFSET(RadarTrackRecord, TrackID));
		_fact->RegisterTableColumn("TrackStatus", RECORD_INTEGER, "航迹状态(0候选/1确认/2丢失)", GETOFFSET(RadarTrackRecord, TrackStatus));
		_fact->RegisterTableColumn("TrackStatusName", RECORD_STRING, "航迹状态名", GETOFFSET(RadarTrackRecord, TrackStatusName));
		_fact->RegisterTableColumn("TruthUnitID", RECORD_INTEGER, "真值目标单元ID", GETOFFSET(RadarTrackRecord, TruthUnitID));
		_fact->RegisterTableColumn("TruthUnitName", RECORD_STRING, "真值目标名称", GETOFFSET(RadarTrackRecord, TruthUnitName));
		_fact->RegisterTableColumn("PosEnuXM", RECORD_DOUBLE, "滤波位置ENU X(m)", GETOFFSET(RadarTrackRecord, PosEnuXM));
		_fact->RegisterTableColumn("PosEnuYM", RECORD_DOUBLE, "滤波位置ENU Y(m)", GETOFFSET(RadarTrackRecord, PosEnuYM));
		_fact->RegisterTableColumn("PosEnuZM", RECORD_DOUBLE, "滤波位置ENU Z(m)", GETOFFSET(RadarTrackRecord, PosEnuZM));
		_fact->RegisterTableColumn("VelXMps", RECORD_DOUBLE, "滤波速度X(m/s)", GETOFFSET(RadarTrackRecord, VelXMps));
		_fact->RegisterTableColumn("VelYMps", RECORD_DOUBLE, "滤波速度Y(m/s)", GETOFFSET(RadarTrackRecord, VelYMps));
		_fact->RegisterTableColumn("VelZMps", RECORD_DOUBLE, "滤波速度Z(m/s)", GETOFFSET(RadarTrackRecord, VelZMps));
		_fact->RegisterTableColumn("SpeedMps", RECORD_DOUBLE, "速度模长(m/s)", GETOFFSET(RadarTrackRecord, SpeedMps));
		_fact->RegisterTableColumn("AccXMps2", RECORD_DOUBLE, "滤波加速度X(m/s^2)", GETOFFSET(RadarTrackRecord, AccXMps2));
		_fact->RegisterTableColumn("AccYMps2", RECORD_DOUBLE, "滤波加速度Y(m/s^2)", GETOFFSET(RadarTrackRecord, AccYMps2));
		_fact->RegisterTableColumn("AccZMps2", RECORD_DOUBLE, "滤波加速度Z(m/s^2)", GETOFFSET(RadarTrackRecord, AccZMps2));
		_fact->RegisterTableColumn("RangeM", RECORD_DOUBLE, "斜距(m)", GETOFFSET(RadarTrackRecord, RangeM));
		_fact->RegisterTableColumn("AzDeg", RECORD_DOUBLE, "方位角(deg)", GETOFFSET(RadarTrackRecord, AzDeg));
		_fact->RegisterTableColumn("ElDeg", RECORD_DOUBLE, "俯仰角(deg)", GETOFFSET(RadarTrackRecord, ElDeg));
		_fact->RegisterTableColumn("PosUncertaintyM", RECORD_DOUBLE, "位置不确定度(m)", GETOFFSET(RadarTrackRecord, PosUncertaintyM));
		_fact->RegisterTableColumn("HitCount", RECORD_INTEGER, "累计命中数", GETOFFSET(RadarTrackRecord, HitCount));
		_fact->RegisterTableColumn("MissCount", RECORD_INTEGER, "连续失配数", GETOFFSET(RadarTrackRecord, MissCount));
		_fact->RegisterTableColumn("TrackSnrDb", RECORD_DOUBLE, "航迹信噪比(dB)", GETOFFSET(RadarTrackRecord, TrackSnrDb));
		_fact->RegisterTableColumn("RecogCategory", RECORD_INTEGER, "识别类别", GETOFFSET(RadarTrackRecord, RecogCategory));
		_fact->RegisterTableColumn("RecogModel", RECORD_STRING, "识别型号", GETOFFSET(RadarTrackRecord, RecogModel));
		_fact->RegisterTableColumn("RecogConfidence", RECORD_DOUBLE, "识别置信度", GETOFFSET(RadarTrackRecord, RecogConfidence));
		_fact->RegisterTableColumn("FirstDetectTimeSec", RECORD_DOUBLE, "起批时刻(s)", GETOFFSET(RadarTrackRecord, FirstDetectTimeSec));
		_fact->RegisterTableColumn("LastUpdateTimeSec", RECORD_DOUBLE, "最近更新时刻(s)", GETOFFSET(RadarTrackRecord, LastUpdateTimeSec));
		_fact->RegisterOneTableEnd();
	}
};
```

```cpp
// ============ 第三类：毁伤信息 ============
struct DamageRecord
{
	DamageRecord(){ memset(this, 0, sizeof(*this)); }
	CyberBaseType::CyberInt RunNum;
	CyberBaseType::CyberInt CycleIndex;
	CyberBaseType::CyberDouble TimeSec;
	CyberBaseType::CyberInt EventID;
	CyberBaseType::CyberChar64 AttackerTeam;
	CyberBaseType::CyberInt AttackerUnitID;
	CyberBaseType::CyberChar64 AttackerUnitName;
	CyberBaseType::CyberChar64 AttackerPlatformName;
	CyberBaseType::CyberChar64 MunitionName;
	CyberBaseType::CyberChar64 MunitionType;
	CyberBaseType::CyberInt MunitionID;
	CyberBaseType::CyberChar64 TargetTeam;
	CyberBaseType::CyberInt TargetUnitID;
	CyberBaseType::CyberChar64 TargetUnitName;
	CyberBaseType::CyberChar64 TargetPlatformName;
	CyberBaseType::CyberChar64 TargetEntityType;
	CyberBaseType::CyberInt HitFlag;
	CyberBaseType::CyberDouble MissDistanceM;
	CyberBaseType::CyberDouble KillRadiusM;
	CyberBaseType::CyberDouble Pk;
	CyberBaseType::CyberDouble PkDraw;
	CyberBaseType::CyberInt DamageStateBefore;
	CyberBaseType::CyberInt DamageStateAfter;
	CyberBaseType::CyberChar64 DamageLevelName;
	CyberBaseType::CyberDouble HpBefore;
	CyberBaseType::CyberDouble HpAfter;
	CyberBaseType::CyberChar64 EffectModelName;
	CyberBaseType::CyberChar64 EffectDescriptor;
	CyberBaseType::CyberInt CumulativeHitCount;
	CyberBaseType::CyberInt RelatedLaunchEventID;
	CyberBaseType::CyberChar64 Remark;

	static void Register(CyberDataRecorderFactory *_fact)
	{
		_fact->RegisterRelevanceTable("DamageRecord", "DamageRecord");
		_fact->RegisterTableColumn("RunNum", RECORD_INTEGER, "仿真次数", GETOFFSET(DamageRecord, RunNum));
		_fact->RegisterTableColumn("CycleIndex", RECORD_INTEGER, "周期号", GETOFFSET(DamageRecord, CycleIndex));
		_fact->RegisterTableColumn("TimeSec", RECORD_DOUBLE, "仿真时刻(s)", GETOFFSET(DamageRecord, TimeSec));
		_fact->RegisterTableColumn("EventID", RECORD_INTEGER, "事件ID", GETOFFSET(DamageRecord, EventID));
		_fact->RegisterTableColumn("AttackerTeam", RECORD_STRING, "攻击方阵营", GETOFFSET(DamageRecord, AttackerTeam));
		_fact->RegisterTableColumn("AttackerUnitID", RECORD_INTEGER, "攻击方单元ID", GETOFFSET(DamageRecord, AttackerUnitID));
		_fact->RegisterTableColumn("AttackerUnitName", RECORD_STRING, "攻击方单元名称", GETOFFSET(DamageRecord, AttackerUnitName));
		_fact->RegisterTableColumn("AttackerPlatformName", RECORD_STRING, "攻击方平台名称", GETOFFSET(DamageRecord, AttackerPlatformName));
		_fact->RegisterTableColumn("MunitionName", RECORD_STRING, "弹药名称", GETOFFSET(DamageRecord, MunitionName));
		_fact->RegisterTableColumn("MunitionType", RECORD_STRING, "弹药类型", GETOFFSET(DamageRecord, MunitionType));
		_fact->RegisterTableColumn("MunitionID", RECORD_INTEGER, "弹药ID", GETOFFSET(DamageRecord, MunitionID));
		_fact->RegisterTableColumn("TargetTeam", RECORD_STRING, "目标阵营", GETOFFSET(DamageRecord, TargetTeam));
		_fact->RegisterTableColumn("TargetUnitID", RECORD_INTEGER, "目标单元ID", GETOFFSET(DamageRecord, TargetUnitID));
		_fact->RegisterTableColumn("TargetUnitName", RECORD_STRING, "目标单元名称", GETOFFSET(DamageRecord, TargetUnitName));
		_fact->RegisterTableColumn("TargetPlatformName", RECORD_STRING, "目标平台名称", GETOFFSET(DamageRecord, TargetPlatformName));
		_fact->RegisterTableColumn("TargetEntityType", RECORD_STRING, "目标类型", GETOFFSET(DamageRecord, TargetEntityType));
		_fact->RegisterTableColumn("HitFlag", RECORD_INTEGER, "是否命中", GETOFFSET(DamageRecord, HitFlag));
		_fact->RegisterTableColumn("MissDistanceM", RECORD_DOUBLE, "脱靶量(m)", GETOFFSET(DamageRecord, MissDistanceM));
		_fact->RegisterTableColumn("KillRadiusM", RECORD_DOUBLE, "杀伤半径(m)", GETOFFSET(DamageRecord, KillRadiusM));
		_fact->RegisterTableColumn("Pk", RECORD_DOUBLE, "毁伤概率", GETOFFSET(DamageRecord, Pk));
		_fact->RegisterTableColumn("PkDraw", RECORD_DOUBLE, "PK抽样值", GETOFFSET(DamageRecord, PkDraw));
		_fact->RegisterTableColumn("DamageStateBefore", RECORD_INTEGER, "毁伤前等级", GETOFFSET(DamageRecord, DamageStateBefore));
		_fact->RegisterTableColumn("DamageStateAfter", RECORD_INTEGER, "毁伤后等级", GETOFFSET(DamageRecord, DamageStateAfter));
		_fact->RegisterTableColumn("DamageLevelName", RECORD_STRING, "毁伤等级名", GETOFFSET(DamageRecord, DamageLevelName));
		_fact->RegisterTableColumn("HpBefore", RECORD_DOUBLE, "生命值前", GETOFFSET(DamageRecord, HpBefore));
		_fact->RegisterTableColumn("HpAfter", RECORD_DOUBLE, "生命值后", GETOFFSET(DamageRecord, HpAfter));
		_fact->RegisterTableColumn("EffectModelName", RECORD_STRING, "毁伤判定模型", GETOFFSET(DamageRecord, EffectModelName));
		_fact->RegisterTableColumn("EffectDescriptor", RECORD_STRING, "效果描述符", GETOFFSET(DamageRecord, EffectDescriptor));
		_fact->RegisterTableColumn("CumulativeHitCount", RECORD_INTEGER, "累计命中数", GETOFFSET(DamageRecord, CumulativeHitCount));
		_fact->RegisterTableColumn("RelatedLaunchEventID", RECORD_INTEGER, "对应发射事件ID", GETOFFSET(DamageRecord, RelatedLaunchEventID));
		_fact->RegisterTableColumn("Remark", RECORD_STRING, "备注", GETOFFSET(DamageRecord, Remark));
		_fact->RegisterOneTableEnd();
	}
};

// ============ 第四类：武器发射信息 ============
struct WeaponLaunchRecord
{
	WeaponLaunchRecord(){ memset(this, 0, sizeof(*this)); }
	CyberBaseType::CyberInt RunNum;
	CyberBaseType::CyberInt CycleIndex;
	CyberBaseType::CyberDouble TimeSec;
	CyberBaseType::CyberInt EventID;
	CyberBaseType::CyberChar64 LauncherTeam;
	CyberBaseType::CyberInt LauncherUnitID;
	CyberBaseType::CyberChar64 LauncherUnitName;
	CyberBaseType::CyberChar64 LauncherPlatformName;
	CyberBaseType::CyberChar64 LauncherEntityType;
	CyberBaseType::CyberInt WeaponID;
	CyberBaseType::CyberChar64 WeaponName;
	CyberBaseType::CyberChar64 WeaponType;
	CyberBaseType::CyberChar64 WeaponModel;
	CyberBaseType::CyberChar64 TargetTeam;
	CyberBaseType::CyberInt TargetUnitID;
	CyberBaseType::CyberChar64 TargetUnitName;
	CyberBaseType::CyberChar64 TargetPlatformName;
	CyberBaseType::CyberDouble LaunchLat;
	CyberBaseType::CyberDouble LaunchLon;
	CyberBaseType::CyberDouble LaunchAltMSL;
	CyberBaseType::CyberDouble LaunchEcefX;
	CyberBaseType::CyberDouble LaunchEcefY;
	CyberBaseType::CyberDouble LaunchEcefZ;
	CyberBaseType::CyberDouble LaunchRangeM;
	CyberBaseType::CyberDouble LaunchAzDeg;
	CyberBaseType::CyberDouble LaunchElDeg;
	CyberBaseType::CyberDouble LaunchAngleDeg;
	CyberBaseType::CyberInt SalvoIndex;
	CyberBaseType::CyberInt SalvoSize;
	CyberBaseType::CyberChar64 GuidanceMode;
	CyberBaseType::CyberInt FireControlUnitID;
	CyberBaseType::CyberChar64 FireControlUnitName;
	CyberBaseType::CyberChar64 LaunchEventName;
	CyberBaseType::CyberInt RelatedDamageEventID;
	CyberBaseType::CyberInt HitFlag;
	CyberBaseType::CyberDouble FlightTimeSec;

	static void Register(CyberDataRecorderFactory *_fact)
	{
		_fact->RegisterRelevanceTable("WeaponLaunchRecord", "WeaponLaunchRecord");
		_fact->RegisterTableColumn("RunNum", RECORD_INTEGER, "仿真次数", GETOFFSET(WeaponLaunchRecord, RunNum));
		_fact->RegisterTableColumn("CycleIndex", RECORD_INTEGER, "周期号", GETOFFSET(WeaponLaunchRecord, CycleIndex));
		_fact->RegisterTableColumn("TimeSec", RECORD_DOUBLE, "仿真时刻(s)", GETOFFSET(WeaponLaunchRecord, TimeSec));
		_fact->RegisterTableColumn("EventID", RECORD_INTEGER, "事件ID", GETOFFSET(WeaponLaunchRecord, EventID));
		_fact->RegisterTableColumn("LauncherTeam", RECORD_STRING, "发射方阵营", GETOFFSET(WeaponLaunchRecord, LauncherTeam));
		_fact->RegisterTableColumn("LauncherUnitID", RECORD_INTEGER, "发射单元ID", GETOFFSET(WeaponLaunchRecord, LauncherUnitID));
		_fact->RegisterTableColumn("LauncherUnitName", RECORD_STRING, "发射单元名称", GETOFFSET(WeaponLaunchRecord, LauncherUnitName));
		_fact->RegisterTableColumn("LauncherPlatformName", RECORD_STRING, "发射平台名称", GETOFFSET(WeaponLaunchRecord, LauncherPlatformName));
		_fact->RegisterTableColumn("LauncherEntityType", RECORD_STRING, "发射单元类型", GETOFFSET(WeaponLaunchRecord, LauncherEntityType));
		_fact->RegisterTableColumn("WeaponID", RECORD_INTEGER, "武器/弹药ID", GETOFFSET(WeaponLaunchRecord, WeaponID));
		_fact->RegisterTableColumn("WeaponName", RECORD_STRING, "武器名称", GETOFFSET(WeaponLaunchRecord, WeaponName));
		_fact->RegisterTableColumn("WeaponType", RECORD_STRING, "武器类型", GETOFFSET(WeaponLaunchRecord, WeaponType));
		_fact->RegisterTableColumn("WeaponModel", RECORD_STRING, "武器型号", GETOFFSET(WeaponLaunchRecord, WeaponModel));
		_fact->RegisterTableColumn("TargetTeam", RECORD_STRING, "目标阵营", GETOFFSET(WeaponLaunchRecord, TargetTeam));
		_fact->RegisterTableColumn("TargetUnitID", RECORD_INTEGER, "目标单元ID", GETOFFSET(WeaponLaunchRecord, TargetUnitID));
		_fact->RegisterTableColumn("TargetUnitName", RECORD_STRING, "目标单元名称", GETOFFSET(WeaponLaunchRecord, TargetUnitName));
		_fact->RegisterTableColumn("TargetPlatformName", RECORD_STRING, "目标平台名称", GETOFFSET(WeaponLaunchRecord, TargetPlatformName));
		_fact->RegisterTableColumn("LaunchLat", RECORD_DOUBLE, "发射点纬度(deg)", GETOFFSET(WeaponLaunchRecord, LaunchLat));
		_fact->RegisterTableColumn("LaunchLon", RECORD_DOUBLE, "发射点经度(deg)", GETOFFSET(WeaponLaunchRecord, LaunchLon));
		_fact->RegisterTableColumn("LaunchAltMSL", RECORD_DOUBLE, "发射点海拔(m)", GETOFFSET(WeaponLaunchRecord, LaunchAltMSL));
		_fact->RegisterTableColumn("LaunchEcefX", RECORD_DOUBLE, "发射点ECEF X(m)", GETOFFSET(WeaponLaunchRecord, LaunchEcefX));
		_fact->RegisterTableColumn("LaunchEcefY", RECORD_DOUBLE, "发射点ECEF Y(m)", GETOFFSET(WeaponLaunchRecord, LaunchEcefY));
		_fact->RegisterTableColumn("LaunchEcefZ", RECORD_DOUBLE, "发射点ECEF Z(m)", GETOFFSET(WeaponLaunchRecord, LaunchEcefZ));
		_fact->RegisterTableColumn("LaunchRangeM", RECORD_DOUBLE, "发射距离(m)", GETOFFSET(WeaponLaunchRecord, LaunchRangeM));
		_fact->RegisterTableColumn("LaunchAzDeg", RECORD_DOUBLE, "发射方位(deg)", GETOFFSET(WeaponLaunchRecord, LaunchAzDeg));
		_fact->RegisterTableColumn("LaunchElDeg", RECORD_DOUBLE, "发射俯仰(deg)", GETOFFSET(WeaponLaunchRecord, LaunchElDeg));
		_fact->RegisterTableColumn("LaunchAngleDeg", RECORD_DOUBLE, "发射角(deg)", GETOFFSET(WeaponLaunchRecord, LaunchAngleDeg));
		_fact->RegisterTableColumn("SalvoIndex", RECORD_INTEGER, "齐射序号", GETOFFSET(WeaponLaunchRecord, SalvoIndex));
		_fact->RegisterTableColumn("SalvoSize", RECORD_INTEGER, "齐射规模", GETOFFSET(WeaponLaunchRecord, SalvoSize));
		_fact->RegisterTableColumn("GuidanceMode", RECORD_STRING, "制导方式", GETOFFSET(WeaponLaunchRecord, GuidanceMode));
		_fact->RegisterTableColumn("FireControlUnitID", RECORD_INTEGER, "火控单元ID", GETOFFSET(WeaponLaunchRecord, FireControlUnitID));
		_fact->RegisterTableColumn("FireControlUnitName", RECORD_STRING, "火控单元名称", GETOFFSET(WeaponLaunchRecord, FireControlUnitName));
		_fact->RegisterTableColumn("LaunchEventName", RECORD_STRING, "发射事件名", GETOFFSET(WeaponLaunchRecord, LaunchEventName));
		_fact->RegisterTableColumn("RelatedDamageEventID", RECORD_INTEGER, "对应毁伤事件ID", GETOFFSET(WeaponLaunchRecord, RelatedDamageEventID));
		_fact->RegisterTableColumn("HitFlag", RECORD_INTEGER, "最终命中标志", GETOFFSET(WeaponLaunchRecord, HitFlag));
		_fact->RegisterTableColumn("FlightTimeSec", RECORD_DOUBLE, "飞行时间(s)", GETOFFSET(WeaponLaunchRecord, FlightTimeSec));
		_fact->RegisterOneTableEnd();
	}
};
```

```cpp
// ============ 第五类：空域信息 ============
struct AirspaceInfoRecord
{
	AirspaceInfoRecord(){ memset(this, 0, sizeof(*this)); }
	CyberBaseType::CyberInt RunNum;
	CyberBaseType::CyberInt AirspaceID;
	CyberBaseType::CyberChar64 AirspaceName;
	CyberBaseType::CyberInt AirspaceType;
	CyberBaseType::CyberInt AreaType;
	CyberBaseType::CyberChar64 AreaTypeName;
	CyberBaseType::CyberDouble FloorMSL;
	CyberBaseType::CyberDouble CeilingMSL;
	CyberBaseType::CyberInt IsBound;
	CyberBaseType::CyberDouble CenterLat;
	CyberBaseType::CyberDouble CenterLon;
	CyberBaseType::CyberDouble RangeM;
	CyberBaseType::CyberDouble OrientationDeg;
	CyberBaseType::CyberDouble SectorAngleDeg;
	CyberBaseType::CyberInt PointCount;
	CyberBaseType::CyberChar64 ForceSide;
	CyberBaseType::CyberInt BelongUnitID;
	CyberBaseType::CyberChar64 BelongUnitName;
	CyberBaseType::CyberChar64 UsageType;
	CyberBaseType::CyberDouble ValidFromSec;
	CyberBaseType::CyberDouble ValidToSec;

	static void Register(CyberDataRecorderFactory *_fact)
	{
		_fact->RegisterRelevanceTable("AirspaceInfoRecord", "AirspaceInfoRecord");
		_fact->RegisterTableColumn("RunNum", RECORD_INTEGER, "仿真次数", GETOFFSET(AirspaceInfoRecord, RunNum));
		_fact->RegisterTableColumn("AirspaceID", RECORD_INTEGER, "空域ID", GETOFFSET(AirspaceInfoRecord, AirspaceID));
		_fact->RegisterTableColumn("AirspaceName", RECORD_STRING, "空域名称", GETOFFSET(AirspaceInfoRecord, AirspaceName));
		_fact->RegisterTableColumn("AirspaceType", RECORD_INTEGER, "空域类型(宿主Type)", GETOFFSET(AirspaceInfoRecord, AirspaceType));
		_fact->RegisterTableColumn("AreaType", RECORD_INTEGER, "几何类型(1多边形/2圆形)", GETOFFSET(AirspaceInfoRecord, AreaType));
		_fact->RegisterTableColumn("AreaTypeName", RECORD_STRING, "几何类型名", GETOFFSET(AirspaceInfoRecord, AreaTypeName));
		_fact->RegisterTableColumn("FloorMSL", RECORD_DOUBLE, "下界高度(m)", GETOFFSET(AirspaceInfoRecord, FloorMSL));
		_fact->RegisterTableColumn("CeilingMSL", RECORD_DOUBLE, "上界高度(m)", GETOFFSET(AirspaceInfoRecord, CeilingMSL));
		_fact->RegisterTableColumn("IsBound", RECORD_INTEGER, "是否绑定单元", GETOFFSET(AirspaceInfoRecord, IsBound));
		_fact->RegisterTableColumn("CenterLat", RECORD_DOUBLE, "圆心纬度(deg)", GETOFFSET(AirspaceInfoRecord, CenterLat));
		_fact->RegisterTableColumn("CenterLon", RECORD_DOUBLE, "圆心经度(deg)", GETOFFSET(AirspaceInfoRecord, CenterLon));
		_fact->RegisterTableColumn("RangeM", RECORD_DOUBLE, "半径(m)", GETOFFSET(AirspaceInfoRecord, RangeM));
		_fact->RegisterTableColumn("OrientationDeg", RECORD_DOUBLE, "朝向(deg)", GETOFFSET(AirspaceInfoRecord, OrientationDeg));
		_fact->RegisterTableColumn("SectorAngleDeg", RECORD_DOUBLE, "扇区张角(deg)", GETOFFSET(AirspaceInfoRecord, SectorAngleDeg));
		_fact->RegisterTableColumn("PointCount", RECORD_INTEGER, "多边形顶点数", GETOFFSET(AirspaceInfoRecord, PointCount));
		_fact->RegisterTableColumn("ForceSide", RECORD_STRING, "归属阵营", GETOFFSET(AirspaceInfoRecord, ForceSide));
		_fact->RegisterTableColumn("BelongUnitID", RECORD_INTEGER, "绑定单元ID", GETOFFSET(AirspaceInfoRecord, BelongUnitID));
		_fact->RegisterTableColumn("BelongUnitName", RECORD_STRING, "绑定单元名称", GETOFFSET(AirspaceInfoRecord, BelongUnitName));
		_fact->RegisterTableColumn("UsageType", RECORD_STRING, "用途(巡逻/责任区/禁飞/拦截)", GETOFFSET(AirspaceInfoRecord, UsageType));
		_fact->RegisterTableColumn("ValidFromSec", RECORD_DOUBLE, "生效时刻(s)", GETOFFSET(AirspaceInfoRecord, ValidFromSec));
		_fact->RegisterTableColumn("ValidToSec", RECORD_DOUBLE, "失效时刻(s)", GETOFFSET(AirspaceInfoRecord, ValidToSec));
		_fact->RegisterOneTableEnd();
	}
};

struct AirspacePointRecord
{
	AirspacePointRecord(){ memset(this, 0, sizeof(*this)); }
	CyberBaseType::CyberInt RunNum;
	CyberBaseType::CyberInt AirspaceID;
	CyberBaseType::CyberChar64 AirspaceName;
	CyberBaseType::CyberInt PointIndex;
	CyberBaseType::CyberDouble Lat;
	CyberBaseType::CyberDouble Lon;
	CyberBaseType::CyberDouble AltMSL;
	CyberBaseType::CyberDouble SpeedMps;

	static void Register(CyberDataRecorderFactory *_fact)
	{
		_fact->RegisterRelevanceTable("AirspacePointRecord", "AirspacePointRecord");
		_fact->RegisterTableColumn("RunNum", RECORD_INTEGER, "仿真次数", GETOFFSET(AirspacePointRecord, RunNum));
		_fact->RegisterTableColumn("AirspaceID", RECORD_INTEGER, "空域ID", GETOFFSET(AirspacePointRecord, AirspaceID));
		_fact->RegisterTableColumn("AirspaceName", RECORD_STRING, "空域名称", GETOFFSET(AirspacePointRecord, AirspaceName));
		_fact->RegisterTableColumn("PointIndex", RECORD_INTEGER, "顶点序号", GETOFFSET(AirspacePointRecord, PointIndex));
		_fact->RegisterTableColumn("Lat", RECORD_DOUBLE, "纬度(deg)", GETOFFSET(AirspacePointRecord, Lat));
		_fact->RegisterTableColumn("Lon", RECORD_DOUBLE, "经度(deg)", GETOFFSET(AirspacePointRecord, Lon));
		_fact->RegisterTableColumn("AltMSL", RECORD_DOUBLE, "高度(m)", GETOFFSET(AirspacePointRecord, AltMSL));
		_fact->RegisterTableColumn("SpeedMps", RECORD_DOUBLE, "规划速度(m/s)", GETOFFSET(AirspacePointRecord, SpeedMps));
		_fact->RegisterOneTableEnd();
	}
};

struct AirspaceEventRecord
{
	AirspaceEventRecord(){ memset(this, 0, sizeof(*this)); }
	CyberBaseType::CyberInt RunNum;
	CyberBaseType::CyberInt CycleIndex;
	CyberBaseType::CyberDouble TimeSec;
	CyberBaseType::CyberInt EventID;
	CyberBaseType::CyberInt AirspaceID;
	CyberBaseType::CyberChar64 AirspaceName;
	CyberBaseType::CyberChar64 UsageType;
	CyberBaseType::CyberInt EntityID;
	CyberBaseType::CyberChar64 EntityName;
	CyberBaseType::CyberChar64 Team;
	CyberBaseType::CyberChar64 EntityType;
	CyberBaseType::CyberChar64 PlatformName;
	CyberBaseType::CyberChar64 MissionName;
	CyberBaseType::CyberInt EventType;
	CyberBaseType::CyberChar64 EventName;
	CyberBaseType::CyberDouble Lat;
	CyberBaseType::CyberDouble Lon;
	CyberBaseType::CyberDouble AltMSL;
	CyberBaseType::CyberDouble DistanceToBoundaryM;
	CyberBaseType::CyberDouble AltMarginM;
	CyberBaseType::CyberInt ViolationFlag;
	CyberBaseType::CyberDouble DwellTimeSec;

	static void Register(CyberDataRecorderFactory *_fact)
	{
		_fact->RegisterRelevanceTable("AirspaceEventRecord", "AirspaceEventRecord");
		_fact->RegisterTableColumn("RunNum", RECORD_INTEGER, "仿真次数", GETOFFSET(AirspaceEventRecord, RunNum));
		_fact->RegisterTableColumn("CycleIndex", RECORD_INTEGER, "周期号", GETOFFSET(AirspaceEventRecord, CycleIndex));
		_fact->RegisterTableColumn("TimeSec", RECORD_DOUBLE, "仿真时刻(s)", GETOFFSET(AirspaceEventRecord, TimeSec));
		_fact->RegisterTableColumn("EventID", RECORD_INTEGER, "事件ID", GETOFFSET(AirspaceEventRecord, EventID));
		_fact->RegisterTableColumn("AirspaceID", RECORD_INTEGER, "空域ID", GETOFFSET(AirspaceEventRecord, AirspaceID));
		_fact->RegisterTableColumn("AirspaceName", RECORD_STRING, "空域名称", GETOFFSET(AirspaceEventRecord, AirspaceName));
		_fact->RegisterTableColumn("UsageType", RECORD_STRING, "空域用途", GETOFFSET(AirspaceEventRecord, UsageType));
		_fact->RegisterTableColumn("EntityID", RECORD_INTEGER, "实体单元ID", GETOFFSET(AirspaceEventRecord, EntityID));
		_fact->RegisterTableColumn("EntityName", RECORD_STRING, "实体名称", GETOFFSET(AirspaceEventRecord, EntityName));
		_fact->RegisterTableColumn("Team", RECORD_STRING, "阵营", GETOFFSET(AirspaceEventRecord, Team));
		_fact->RegisterTableColumn("EntityType", RECORD_STRING, "单元类型", GETOFFSET(AirspaceEventRecord, EntityType));
		_fact->RegisterTableColumn("PlatformName", RECORD_STRING, "平台名称", GETOFFSET(AirspaceEventRecord, PlatformName));
		_fact->RegisterTableColumn("MissionName", RECORD_STRING, "任务名称", GETOFFSET(AirspaceEventRecord, MissionName));
		_fact->RegisterTableColumn("EventType", RECORD_INTEGER, "事件类型(1进入/2离开/3越界/4高度越限/5滞留超时)", GETOFFSET(AirspaceEventRecord, EventType));
		_fact->RegisterTableColumn("EventName", RECORD_STRING, "事件名称", GETOFFSET(AirspaceEventRecord, EventName));
		_fact->RegisterTableColumn("Lat", RECORD_DOUBLE, "纬度(deg)", GETOFFSET(AirspaceEventRecord, Lat));
		_fact->RegisterTableColumn("Lon", RECORD_DOUBLE, "经度(deg)", GETOFFSET(AirspaceEventRecord, Lon));
		_fact->RegisterTableColumn("AltMSL", RECORD_DOUBLE, "海拔(m)", GETOFFSET(AirspaceEventRecord, AltMSL));
		_fact->RegisterTableColumn("DistanceToBoundaryM", RECORD_DOUBLE, "距边界距离(m)", GETOFFSET(AirspaceEventRecord, DistanceToBoundaryM));
		_fact->RegisterTableColumn("AltMarginM", RECORD_DOUBLE, "距高度界余量(m)", GETOFFSET(AirspaceEventRecord, AltMarginM));
		_fact->RegisterTableColumn("ViolationFlag", RECORD_INTEGER, "是否违规", GETOFFSET(AirspaceEventRecord, ViolationFlag));
		_fact->RegisterTableColumn("DwellTimeSec", RECORD_DOUBLE, "滞留时长(s)", GETOFFSET(AirspaceEventRecord, DwellTimeSec));
		_fact->RegisterOneTableEnd();
	}
};
```

```cpp
// ============ 第六类：电子侦察探测结果与开关机 ============
struct EsrDetectionRecord
{
	EsrDetectionRecord(){ memset(this, 0, sizeof(*this)); }
	CyberBaseType::CyberInt RunNum;
	CyberBaseType::CyberInt CycleIndex;
	CyberBaseType::CyberDouble TimeSec;
	CyberBaseType::CyberChar64 Team;
	CyberBaseType::CyberInt UnitID;
	CyberBaseType::CyberChar64 UnitName;
	CyberBaseType::CyberChar64 PlatformName;
	CyberBaseType::CyberChar64 SensorName;
	CyberBaseType::CyberChar64 SensorType;
	CyberBaseType::CyberLong ObservationID;
	CyberBaseType::CyberDouble AoaAzDeg;
	CyberBaseType::CyberDouble AoaElDeg;
	CyberBaseType::CyberDouble RfHz;
	CyberBaseType::CyberDouble BandwidthHz;
	CyberBaseType::CyberDouble PriS;
	CyberBaseType::CyberDouble PulseWidthS;
	CyberBaseType::CyberDouble RfStdHz;
	CyberBaseType::CyberDouble BandwidthStdHz;
	CyberBaseType::CyberDouble PriStdS;
	CyberBaseType::CyberDouble PulseWidthStdS;
	CyberBaseType::CyberDouble AmplitudeDb;
	CyberBaseType::CyberDouble SnrDb;
	CyberBaseType::CyberInt Quality;
	CyberBaseType::CyberChar64 QualityName;
	CyberBaseType::CyberInt WaveformClass;
	CyberBaseType::CyberChar64 WaveformName;
	CyberBaseType::CyberInt DeceptionClass;
	CyberBaseType::CyberDouble ReceiverCenterFreqHz;
	CyberBaseType::CyberDouble ReceiverBandwidthHz;
	CyberBaseType::CyberInt ReceiverSaturated;
	CyberBaseType::CyberInt TruthUnitID;
	CyberBaseType::CyberChar64 TruthUnitName;
	CyberBaseType::CyberChar64 TruthPlatformName;
	CyberBaseType::CyberChar64 EmitterTypeName;

	static void Register(CyberDataRecorderFactory *_fact)
	{
		_fact->RegisterRelevanceTable("EsrDetectionRecord", "EsrDetectionRecord");
		_fact->RegisterTableColumn("RunNum", RECORD_INTEGER, "仿真次数", GETOFFSET(EsrDetectionRecord, RunNum));
		_fact->RegisterTableColumn("CycleIndex", RECORD_INTEGER, "周期号", GETOFFSET(EsrDetectionRecord, CycleIndex));
		_fact->RegisterTableColumn("TimeSec", RECORD_DOUBLE, "观测时刻(s)", GETOFFSET(EsrDetectionRecord, TimeSec));
		_fact->RegisterTableColumn("Team", RECORD_STRING, "侦察方阵营", GETOFFSET(EsrDetectionRecord, Team));
		_fact->RegisterTableColumn("UnitID", RECORD_INTEGER, "侦察单元ID", GETOFFSET(EsrDetectionRecord, UnitID));
		_fact->RegisterTableColumn("UnitName", RECORD_STRING, "侦察单元名称", GETOFFSET(EsrDetectionRecord, UnitName));
		_fact->RegisterTableColumn("PlatformName", RECORD_STRING, "侦察平台名称", GETOFFSET(EsrDetectionRecord, PlatformName));
		_fact->RegisterTableColumn("SensorName", RECORD_STRING, "侦察传感器名称", GETOFFSET(EsrDetectionRecord, SensorName));
		_fact->RegisterTableColumn("SensorType", RECORD_STRING, "侦察传感器型号", GETOFFSET(EsrDetectionRecord, SensorType));
		_fact->RegisterTableColumn("ObservationID", RECORD_INTEGER, "观测唯一标识", GETOFFSET(EsrDetectionRecord, ObservationID));
		_fact->RegisterTableColumn("AoaAzDeg", RECORD_DOUBLE, "到达角方位(deg)", GETOFFSET(EsrDetectionRecord, AoaAzDeg));
		_fact->RegisterTableColumn("AoaElDeg", RECORD_DOUBLE, "到达角俯仰(deg)", GETOFFSET(EsrDetectionRecord, AoaElDeg));
		_fact->RegisterTableColumn("RfHz", RECORD_DOUBLE, "载频(Hz)", GETOFFSET(EsrDetectionRecord, RfHz));
		_fact->RegisterTableColumn("BandwidthHz", RECORD_DOUBLE, "占用带宽(Hz)", GETOFFSET(EsrDetectionRecord, BandwidthHz));
		_fact->RegisterTableColumn("PriS", RECORD_DOUBLE, "脉冲重复间隔(s)", GETOFFSET(EsrDetectionRecord, PriS));
		_fact->RegisterTableColumn("PulseWidthS", RECORD_DOUBLE, "脉宽(s)", GETOFFSET(EsrDetectionRecord, PulseWidthS));
		_fact->RegisterTableColumn("RfStdHz", RECORD_DOUBLE, "载频估计标准差(Hz)", GETOFFSET(EsrDetectionRecord, RfStdHz));
		_fact->RegisterTableColumn("BandwidthStdHz", RECORD_DOUBLE, "带宽估计标准差(Hz)", GETOFFSET(EsrDetectionRecord, BandwidthStdHz));
		_fact->RegisterTableColumn("PriStdS", RECORD_DOUBLE, "PRI估计标准差(s)", GETOFFSET(EsrDetectionRecord, PriStdS));
		_fact->RegisterTableColumn("PulseWidthStdS", RECORD_DOUBLE, "脉宽估计标准差(s)", GETOFFSET(EsrDetectionRecord, PulseWidthStdS));
		_fact->RegisterTableColumn("AmplitudeDb", RECORD_DOUBLE, "接收幅度(dB)", GETOFFSET(EsrDetectionRecord, AmplitudeDb));
		_fact->RegisterTableColumn("SnrDb", RECORD_DOUBLE, "观测信噪比(dB)", GETOFFSET(EsrDetectionRecord, SnrDb));
		_fact->RegisterTableColumn("Quality", RECORD_INTEGER, "观测质量(0低/1中/2高)", GETOFFSET(EsrDetectionRecord, Quality));
		_fact->RegisterTableColumn("QualityName", RECORD_STRING, "观测质量名", GETOFFSET(EsrDetectionRecord, QualityName));
		_fact->RegisterTableColumn("WaveformClass", RECORD_INTEGER, "波形类别(0脉冲/1连续/2扫频/3噪声)", GETOFFSET(EsrDetectionRecord, WaveformClass));
		_fact->RegisterTableColumn("WaveformName", RECORD_STRING, "波形类别名", GETOFFSET(EsrDetectionRecord, WaveformName));
		_fact->RegisterTableColumn("DeceptionClass", RECORD_INTEGER, "欺骗分类(0无/1疑似假目标)", GETOFFSET(EsrDetectionRecord, DeceptionClass));
		_fact->RegisterTableColumn("ReceiverCenterFreqHz", RECORD_DOUBLE, "接收调谐中心频率(Hz)", GETOFFSET(EsrDetectionRecord, ReceiverCenterFreqHz));
		_fact->RegisterTableColumn("ReceiverBandwidthHz", RECORD_DOUBLE, "接收调谐带宽(Hz)", GETOFFSET(EsrDetectionRecord, ReceiverBandwidthHz));
		_fact->RegisterTableColumn("ReceiverSaturated", RECORD_INTEGER, "接收机饱和", GETOFFSET(EsrDetectionRecord, ReceiverSaturated));
		_fact->RegisterTableColumn("TruthUnitID", RECORD_INTEGER, "真值辐射源ID", GETOFFSET(EsrDetectionRecord, TruthUnitID));
		_fact->RegisterTableColumn("TruthUnitName", RECORD_STRING, "真值辐射源名称", GETOFFSET(EsrDetectionRecord, TruthUnitName));
		_fact->RegisterTableColumn("TruthPlatformName", RECORD_STRING, "真值辐射源平台名", GETOFFSET(EsrDetectionRecord, TruthPlatformName));
		_fact->RegisterTableColumn("EmitterTypeName", RECORD_STRING, "辐射源型号", GETOFFSET(EsrDetectionRecord, EmitterTypeName));
		_fact->RegisterOneTableEnd();
	}
};

struct EsrEmitterRecord
{
	EsrEmitterRecord(){ memset(this, 0, sizeof(*this)); }
	CyberBaseType::CyberInt RunNum;
	CyberBaseType::CyberInt CycleIndex;
	CyberBaseType::CyberDouble TimeSec;
	CyberBaseType::CyberChar64 Team;
	CyberBaseType::CyberInt UnitID;
	CyberBaseType::CyberChar64 UnitName;
	CyberBaseType::CyberChar64 PlatformName;
	CyberBaseType::CyberChar64 SensorName;
	CyberBaseType::CyberLong HypothesisID;
	CyberBaseType::CyberChar64 EmitterClass;
	CyberBaseType::CyberChar128 CandidateClasses;
	CyberBaseType::CyberInt Mode;
	CyberBaseType::CyberChar64 ModeName;
	CyberBaseType::CyberInt ThreatLevel;
	CyberBaseType::CyberChar64 ThreatName;
	CyberBaseType::CyberDouble BearingAzDeg;
	CyberBaseType::CyberDouble BearingElDeg;
	CyberBaseType::CyberDouble BearingStdDeg;
	CyberBaseType::CyberDouble EstCenterFreqHz;
	CyberBaseType::CyberDouble EstBandwidthHz;
	CyberBaseType::CyberDouble EstPriS;
	CyberBaseType::CyberDouble EstPulseWidthS;
	CyberBaseType::CyberDouble CenterFreqStdHz;
	CyberBaseType::CyberDouble BandwidthStdHz;
	CyberBaseType::CyberDouble PriStdS;
	CyberBaseType::CyberDouble PulseWidthStdS;
	CyberBaseType::CyberDouble Confidence;
	CyberBaseType::CyberInt LastSeenCycle;
	CyberBaseType::CyberInt WaveformClass;
	CyberBaseType::CyberInt TruthEmitterUnitID;
	CyberBaseType::CyberChar64 TruthEmitterName;

	static void Register(CyberDataRecorderFactory *_fact)
	{
		_fact->RegisterRelevanceTable("EsrEmitterRecord", "EsrEmitterRecord");
		_fact->RegisterTableColumn("RunNum", RECORD_INTEGER, "仿真次数", GETOFFSET(EsrEmitterRecord, RunNum));
		_fact->RegisterTableColumn("CycleIndex", RECORD_INTEGER, "周期号", GETOFFSET(EsrEmitterRecord, CycleIndex));
		_fact->RegisterTableColumn("TimeSec", RECORD_DOUBLE, "仿真时刻(s)", GETOFFSET(EsrEmitterRecord, TimeSec));
		_fact->RegisterTableColumn("Team", RECORD_STRING, "侦察方阵营", GETOFFSET(EsrEmitterRecord, Team));
		_fact->RegisterTableColumn("UnitID", RECORD_INTEGER, "侦察单元ID", GETOFFSET(EsrEmitterRecord, UnitID));
		_fact->RegisterTableColumn("UnitName", RECORD_STRING, "侦察单元名称", GETOFFSET(EsrEmitterRecord, UnitName));
		_fact->RegisterTableColumn("PlatformName", RECORD_STRING, "侦察平台名称", GETOFFSET(EsrEmitterRecord, PlatformName));
		_fact->RegisterTableColumn("SensorName", RECORD_STRING, "侦察传感器名称", GETOFFSET(EsrEmitterRecord, SensorName));
		_fact->RegisterTableColumn("HypothesisID", RECORD_INTEGER, "假设唯一标识", GETOFFSET(EsrEmitterRecord, HypothesisID));
		_fact->RegisterTableColumn("EmitterClass", RECORD_STRING, "首选候选类别", GETOFFSET(EsrEmitterRecord, EmitterClass));
		_fact->RegisterTableColumn("CandidateClasses", RECORD_STRING, "候选类别(分号分隔)", GETOFFSET(EsrEmitterRecord, CandidateClasses));
		_fact->RegisterTableColumn("Mode", RECORD_INTEGER, "工作模式假设", GETOFFSET(EsrEmitterRecord, Mode));
		_fact->RegisterTableColumn("ModeName", RECORD_STRING, "工作模式名", GETOFFSET(EsrEmitterRecord, ModeName));
		_fact->RegisterTableColumn("ThreatLevel", RECORD_INTEGER, "威胁等级(0低/1中/2高)", GETOFFSET(EsrEmitterRecord, ThreatLevel));
		_fact->RegisterTableColumn("ThreatName", RECORD_STRING, "威胁等级名", GETOFFSET(EsrEmitterRecord, ThreatName));
		_fact->RegisterTableColumn("BearingAzDeg", RECORD_DOUBLE, "方位线方位(deg)", GETOFFSET(EsrEmitterRecord, BearingAzDeg));
		_fact->RegisterTableColumn("BearingElDeg", RECORD_DOUBLE, "方位线俯仰(deg)", GETOFFSET(EsrEmitterRecord, BearingElDeg));
		_fact->RegisterTableColumn("BearingStdDeg", RECORD_DOUBLE, "方位标准差(deg)", GETOFFSET(EsrEmitterRecord, BearingStdDeg));
		_fact->RegisterTableColumn("EstCenterFreqHz", RECORD_DOUBLE, "估计中心频率(Hz)", GETOFFSET(EsrEmitterRecord, EstCenterFreqHz));
		_fact->RegisterTableColumn("EstBandwidthHz", RECORD_DOUBLE, "估计带宽(Hz)", GETOFFSET(EsrEmitterRecord, EstBandwidthHz));
		_fact->RegisterTableColumn("EstPriS", RECORD_DOUBLE, "估计PRI(s)", GETOFFSET(EsrEmitterRecord, EstPriS));
		_fact->RegisterTableColumn("EstPulseWidthS", RECORD_DOUBLE, "估计脉宽(s)", GETOFFSET(EsrEmitterRecord, EstPulseWidthS));
		_fact->RegisterTableColumn("CenterFreqStdHz", RECORD_DOUBLE, "中心频率标准差(Hz)", GETOFFSET(EsrEmitterRecord, CenterFreqStdHz));
		_fact->RegisterTableColumn("BandwidthStdHz", RECORD_DOUBLE, "带宽标准差(Hz)", GETOFFSET(EsrEmitterRecord, BandwidthStdHz));
		_fact->RegisterTableColumn("PriStdS", RECORD_DOUBLE, "PRI标准差(s)", GETOFFSET(EsrEmitterRecord, PriStdS));
		_fact->RegisterTableColumn("PulseWidthStdS", RECORD_DOUBLE, "脉宽标准差(s)", GETOFFSET(EsrEmitterRecord, PulseWidthStdS));
		_fact->RegisterTableColumn("Confidence", RECORD_DOUBLE, "假设置信度", GETOFFSET(EsrEmitterRecord, Confidence));
		_fact->RegisterTableColumn("LastSeenCycle", RECORD_INTEGER, "最近命中周期", GETOFFSET(EsrEmitterRecord, LastSeenCycle));
		_fact->RegisterTableColumn("WaveformClass", RECORD_INTEGER, "波形类别", GETOFFSET(EsrEmitterRecord, WaveformClass));
		_fact->RegisterTableColumn("TruthEmitterUnitID", RECORD_INTEGER, "真值辐射源ID", GETOFFSET(EsrEmitterRecord, TruthEmitterUnitID));
		_fact->RegisterTableColumn("TruthEmitterName", RECORD_STRING, "真值辐射源名称", GETOFFSET(EsrEmitterRecord, TruthEmitterName));
		_fact->RegisterOneTableEnd();
	}
};

struct SensorPowerRecord
{
	SensorPowerRecord(){ memset(this, 0, sizeof(*this)); }
	CyberBaseType::CyberInt RunNum;
	CyberBaseType::CyberInt CycleIndex;
	CyberBaseType::CyberDouble TimeSec;
	CyberBaseType::CyberInt EventID;
	CyberBaseType::CyberChar64 Team;
	CyberBaseType::CyberInt UnitID;
	CyberBaseType::CyberChar64 UnitName;
	CyberBaseType::CyberChar64 PlatformName;
	CyberBaseType::CyberChar64 SensorName;
	CyberBaseType::CyberChar64 SensorType;
	CyberBaseType::CyberInt SensorIndex;
	CyberBaseType::CyberInt EventType;
	CyberBaseType::CyberChar64 EventName;
	CyberBaseType::CyberInt PowerBefore;
	CyberBaseType::CyberInt PowerAfter;
	CyberBaseType::CyberChar64 ModeBefore;
	CyberBaseType::CyberChar64 ModeAfter;
	CyberBaseType::CyberInt RadiationFlag;
	CyberBaseType::CyberInt ReasonCode;
	CyberBaseType::CyberChar64 ReasonName;
	CyberBaseType::CyberInt OriginUnitID;
	CyberBaseType::CyberChar64 OriginUnitName;
	CyberBaseType::CyberDouble DurationSec;

	static void Register(CyberDataRecorderFactory *_fact)
	{
		_fact->RegisterRelevanceTable("SensorPowerRecord", "SensorPowerRecord");
		_fact->RegisterTableColumn("RunNum", RECORD_INTEGER, "仿真次数", GETOFFSET(SensorPowerRecord, RunNum));
		_fact->RegisterTableColumn("CycleIndex", RECORD_INTEGER, "周期号", GETOFFSET(SensorPowerRecord, CycleIndex));
		_fact->RegisterTableColumn("TimeSec", RECORD_DOUBLE, "仿真时刻(s)", GETOFFSET(SensorPowerRecord, TimeSec));
		_fact->RegisterTableColumn("EventID", RECORD_INTEGER, "事件ID", GETOFFSET(SensorPowerRecord, EventID));
		_fact->RegisterTableColumn("Team", RECORD_STRING, "阵营", GETOFFSET(SensorPowerRecord, Team));
		_fact->RegisterTableColumn("UnitID", RECORD_INTEGER, "单元ID", GETOFFSET(SensorPowerRecord, UnitID));
		_fact->RegisterTableColumn("UnitName", RECORD_STRING, "单元名称", GETOFFSET(SensorPowerRecord, UnitName));
		_fact->RegisterTableColumn("PlatformName", RECORD_STRING, "平台名称", GETOFFSET(SensorPowerRecord, PlatformName));
		_fact->RegisterTableColumn("SensorName", RECORD_STRING, "传感器名称", GETOFFSET(SensorPowerRecord, SensorName));
		_fact->RegisterTableColumn("SensorType", RECORD_STRING, "传感器型号", GETOFFSET(SensorPowerRecord, SensorType));
		_fact->RegisterTableColumn("SensorIndex", RECORD_INTEGER, "编成内传感器序号", GETOFFSET(SensorPowerRecord, SensorIndex));
		_fact->RegisterTableColumn("EventType", RECORD_INTEGER, "事件类型(1开机/2关机/3模式切换/4静默/5恢复)", GETOFFSET(SensorPowerRecord, EventType));
		_fact->RegisterTableColumn("EventName", RECORD_STRING, "事件名称", GETOFFSET(SensorPowerRecord, EventName));
		_fact->RegisterTableColumn("PowerBefore", RECORD_INTEGER, "事件前电源状态", GETOFFSET(SensorPowerRecord, PowerBefore));
		_fact->RegisterTableColumn("PowerAfter", RECORD_INTEGER, "事件后电源状态", GETOFFSET(SensorPowerRecord, PowerAfter));
		_fact->RegisterTableColumn("ModeBefore", RECORD_STRING, "事件前工作模式", GETOFFSET(SensorPowerRecord, ModeBefore));
		_fact->RegisterTableColumn("ModeAfter", RECORD_STRING, "事件后工作模式", GETOFFSET(SensorPowerRecord, ModeAfter));
		_fact->RegisterTableColumn("RadiationFlag", RECORD_INTEGER, "是否辐射", GETOFFSET(SensorPowerRecord, RadiationFlag));
		_fact->RegisterTableColumn("ReasonCode", RECORD_INTEGER, "原因编码", GETOFFSET(SensorPowerRecord, ReasonCode));
		_fact->RegisterTableColumn("ReasonName", RECORD_STRING, "原因(指令/自动/任务/毁伤/故障)", GETOFFSET(SensorPowerRecord, ReasonName));
		_fact->RegisterTableColumn("OriginUnitID", RECORD_INTEGER, "下令单元ID", GETOFFSET(SensorPowerRecord, OriginUnitID));
		_fact->RegisterTableColumn("OriginUnitName", RECORD_STRING, "下令单元名称", GETOFFSET(SensorPowerRecord, OriginUnitName));
		_fact->RegisterTableColumn("DurationSec", RECORD_DOUBLE, "上一状态持续时长(s)", GETOFFSET(SensorPowerRecord, DurationSec));
		_fact->RegisterOneTableEnd();
	}
};

// 追加到 InitlizeRecord() 末尾（OutputRecord.hpp）：12 行
// EntityBaseInfoRecord::Register(_fact);
// EntityStateRecord::Register(_fact);
// RadarDetectionRecord::Register(_fact);
// RadarTrackRecord::Register(_fact);
// DamageRecord::Register(_fact);
// WeaponLaunchRecord::Register(_fact);
// AirspaceInfoRecord::Register(_fact);
// AirspacePointRecord::Register(_fact);
// AirspaceEventRecord::Register(_fact);
// EsrDetectionRecord::Register(_fact);
// EsrEmitterRecord::Register(_fact);
// SensorPowerRecord::Register(_fact);
```

---

## 附录 B：字段来源速查（写入方视角）

| 写入方 | 负责写哪些列 | 触发点 |
|---|---|---|
| 引擎（记录框架） | `RunNum`/`CycleIndex`/`TimeSec`/`EventID` | 每次写行时统一填 |
| 实体/平台模型 | T1-1 静态属性；T1-2 位置/姿态/速度/存活 | 初始化一次 / 每 N 周期 |
| 雷达会话胶水（RIR/AR/老雷达） | T2-1、T2-2 全部产品列 | 会话 `Step` 返回后逐条写 |
| 电子侦察会话胶水（ESR） | T6-1、T6-2 全部产品列 | 同上 |
| 弹药/引信模型 + 效果模型 | T3-1 | 毁伤判定点（`DeterminePlatformDamage` 调用处） |
| 发射/火控模型 | T4-1 | 发射事件 |
| 指挥/任务模型 + 几何判定组件 | T5-3 | 状态变化 |
| 传感器胶水（`TryApplyRuntimeConfig` 处） | T6-3 | `has_sensor_enabled` 补丁提交时 |
| **集成侧统一回填** | 各表 `Truth*` 列；T3-1↔T4-1 的 `Related*EventID`；T2-2 的 `TruthUnit*` | 周期末批处理 |

## 附录 C：本文引用的证据索引

| 主题 | 文件:行 |
|---|---|
| 记录结构与注册示例 | `ModelSource/CyModel/Record/OutPutRecord/OutputRecord.hpp:3-42,44-77,995-1025` |
| 老表字段缺陷 | `.../OutputRecord.hpp:384-386,917` |
| 注册入口（模型 DLL） | `ModelSource/CyModel/Equipment/Sensors/init/fosim_model_manager.cpp:42` |
| 写行示例 | `ModelSource/CyModel/Equipment/Sensors/FZ9Y_GRadar/FZ9y_gradar_detect.cpp:184-212,566-594`；`.../OPIR_SENSOR/opir_sensor_1q.cpp:500`；`.../LRDR_SENSOR/lrdr_sensor_1q.cpp:1692` |
| 记录类型/列类型 | `ModelSource/include/Utils/CyberDataRecorder/CyberBaseType.h:17-30`；`CyberDataRecorder.h:5-11`；`CyberDataRecorderFactory.h:5-7,40-46` |
| 评估算子与指标 | `AssessmentSource/algorithm.ois:5-9`；`AssessmentSource/predefine.pis:3-13`；`AssessmentSource/resultheader.istruct` |
| 空域定义 | `ScenarioSource/模型调测想定_空空拦截/ScenarioDatabase/Airspaces/红方巡逻空域.mdata:1-41`；`ScenarioSource/模型调测想定_单地导阵地拦截目标/ScenarioDatabase/Airspaces/RCA1.mdata:1-17` |
| 任务进入空域 | `ScenarioSource/模型调测想定_空空拦截/ScenarioDatabase/Mission/红方飞机任务1-1.mdata:55-58` |
| 毁伤枚举与判定链 | `ModelSource/include/FOSim/Engine/cyber_enum_type.h:47-53`；`cyber_effect_impl.h:108-125`；`Environment/Effects/StdEffectPK/StdEffectPKData.hpp:88-91`；`stdeffectpk.h:16-17` |
| 传感器事件与被动记录 | `ModelSource/CyModel/Record/EquipmentRecord/cyber_sensor_record.h:257-324,392-408` |
| 弹药/发射参数 | `ModelSource/CyModel/Record/EquipmentRecord/cyber_equipment_record.h:224-239` |
| 1q 传感器电源与关机码 | `include/1q/electronic_surveillance_radar/session/EsrOutputTypes.h:99-104`；`config/EsrRuntimeConfigPatch.h:72-73`；`.../session/EsrIssueCodes.h:93`（AR/RIR/SBIRS/EOS/SAR 同构） |
| 1q ESR 观测/假设 | `include/1q/electronic_surveillance_radar/session/EmitterObservation.h:54-72`；`EmitterHypothesis.h:43-62` |
| 1q AR 航迹与干扰观测 | `include/1q/airborne_radar/session/TrackStateSnapshot.h:30-68`；`ArInterferenceObservation.h:37-59` |
| 1q RIR 归属与特征 | `include/1q/remote_identification_radar/session/RirOutputTypes.h:103-115`；`RirFeatureMeasurementTypes.h:108-122` |
| 1q 真值只进评估层 | `include/1q/precision_evaluation/PrecisionEvaluationTypes.h:23-30,50-56,145-154` |
| 归属挂载契约 | `docs/common/session_contract.md:408-431` |
| 模型可输出/不可输出边界 | `docs/review/functional_acceptance_test_criteria_ocr.md:324-367` |
| 集成与构建流程 | `docs/practice/fosimstudio_host_and_integration_guide.md:76-121,188` |
| 告警机（被动侦察）链路 | `ModelSource/CyModel/Equipment/Sensors/CyQSRWR/cyqsrwr_process.cpp:37-110`（事件类型过滤被注释，见 `:62-71`） |
| 发射类事件实际生产者 | `ModelSource/CyModel/Equipment/WeaponSystems/ZFZ6H01BAAMWS/zFZ6h01baamws_process.cpp:1057-1058`（`RADAR_LOCK`）；`RADAR_TURNON` 无生产者 |
| AR 关联质量 | `include/1q/airborne_radar/session/ArOutputTypes.h:92-103` |
| AR 无逐探测量测 | `include/1q/airborne_radar/**`（无 detection 记录类型；`ArCycleResult.h:56-107` 只含航迹/干扰/控制/关联质量） |
| ESR 工作模式 | `include/1q/electronic_surveillance_radar/config/EsrMissionConfig.h:24` |
| RIR 识别效能摘要 | `include/1q/remote_identification_radar/session/RirRecognitionResult.h:115-131` |
| 1q 评估模块硬绑双星 | `include/1q/precision_evaluation/PrecisionEvaluationSession.h:62-68`；`SbirsBearingAdapter.h:33` |
| "模式≠关机"两个反例 | `src/sbirs_sensor/pipeline/SbirsPipeline.cpp:925`（kStandby 等价关机）；`src/airborne_radar/signal/pipeline/ScanScheduleResolver.cpp:109-115`（kStby 仍发射） |
| ECM 不产 J/S / 烧穿 / 压制比 | `docs/electronic_countermeasure/design.md:147-149` |
| 死表实证（零写入） | `AF_Launch`/`AF_Hit`/`StealthMissile*`/`AAMMissedRecord`（`OutputRecord.hpp:892,913,737,776,814,619` 注册；全仓 `GenerateRecorder` 无对应写入）；`AF_Launch` 在 `InitlizeRecord` 中 `:1018` |
| 未注册表名（静默丢数据） | `ModelSource/CyModel/Equipment/DataProcessors/SBIRSFusionDp/SBIRSFusionDpData.hpp:223`（`#define … "SbirsFusionTrackRecord"`，无 `RegisterRelevanceTable`）；写入 `sbirsfusiondp_1q.cpp:893` |
| 发射/命中真正落表的表 | `DaiWaDengLuRecord`（`zFZ6h01semiactiveaam_move.cpp:528`、`cosimqmcruisemissile_move.cpp:960`、`cosimqmarm_move.cpp:555`、`ht8511pltcwptbm_move.cpp:579`）；`SAMRecord`（`zFZ6h01caamfu_util.cpp:178` 等 5 个 FU）；`MunitionInfo`（`cosimballisticmissile_move.cpp:428` 等 5 个弹药） |
| 发射瞬间可取数据 | `Cognition/ZFZ6H01CAAMFU/zFZ6h01caamfu_Launch.cpp:33-67`（`CyberEngagementInfoType`）；`Equipment/WeaponSystems/ZFZ6H01BAAMWS/zFZ6h01baamws_process.cpp:238-311`（`LaunchMunition`/齐射序号/余弹/`Mode`） |
| 毁伤判定细节（+50 s / DAMAGED 不生效） | `Environment/Effects/StdEffectPK/stdeffectpk_exec.cpp:11-43,45-57`；`CyEFPK/cyefpk_exec.cpp:159-172`；`CyEFAgentEffect/cyefagenteffect_exec.cpp:59-73,149-152`；认知侧 `SetActive` `zFZ6h01caamfu_util.cpp:148-178` |
| 空域枚举与几何 | `Environment/Airspaces/CySZone/CySZone.mdef:83-138`（Type/AreaType）；`cyszone_inside.cpp:15-47`；`CySZoneData.hpp:240-249`（无 ForceSide 成员） |
| 空域插件不注册记录表 | `Environment/Airspaces/init/fosim_model_manager.cpp:44-49` |
| 开关机真实状态与上报链 | `ModelUtil/share/cosim/cosimsensor.h:146-148`（`R_Power_On`/`R_WorkState`）；`Platforms/CoSimTarDetPlatform/cosimtardetsensor_process.cpp:56-83`；`CoSimCSAMController/cosimcsamcontroller_Rank.cpp:148-175` |
| 告警机 DP 硬置幅度 | `DataProcessors/ZFZ6H01BRWRDP/zfZ6h01brwrdp_process.cpp:113,128`；`CyQDSimpleRWRDP/cyqdsimplerwrdp_process.cpp:99,114` |
| 实体状态源（含 RCS/IR/指挥关系） | `include/FOSim/Engine/cyber_struct_type.h:253-303`；`include/FOSim/Engine/models/unit/ICyberUnit.h:195-219`；`fosim_algorithm_impl.h:144,178,184`（LLA/ENU 换算） |
