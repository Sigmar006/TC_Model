---
Status: active
Date: 2026-08-22
Authority: 甲方集成验证清单；两个保留场景（`rir_ground_site_recognition` / `sbirs_triple_sat_fix_messages`）的组件挂载、调用序、事件接线与验收核验。场景几何与配置权威见各场景 `.md`；库 API 权威见 `include/1q/`。
---

# 甲方集成验证清单（两个核心场景）

本文写给**第一次接触 1q 库的集成工程师**：拿到交付物后，要复现本分支两个核心场景
的效果，需要集成哪些组件、按什么顺序调用、接哪些事件、最后核对什么。假设甲方的
宿主工程**也是实体-组件模式、也有自己的事件机制**——本文所有"组件/事件"都给出
「我们示例怎么做 → 甲方框架里对应怎么做」的映射，不要求照搬示例的 World/Boost.Signals2。

> **2026-08-28 场景集精简注**：原两个核心场景（`rir_long_range_scan` /
> `sbirs_dual_sat_fix`）已从场景集移除，本文改为以保留场景
> `rir_ground_site_recognition` / `sbirs_triple_sat_fix_messages` 为参照。
> §4–§6 的组件挂载/调用序/事件接线通用；§7 的行数基线为精简前的实测值，
> 保留场景的逐项行数基线待重跑后更新。

## 0. 先建立三个概念

1. **库会话（session/engine）是真正的产品**。`1q.lib` 里每个仿真模块暴露一个会话
   或引擎类（如 `RirSession`、`FusionEngine`）。它们是纯 C++ 对象，不依赖任何
   框架——你每周期喂数据、调一次 `Step`，它返回结果。验收文件也由库内写出。
2. **示例组件只是"会话的壳"**。`examples/`（core + components）用一个 50 行的
   `Component` 基类 + World + Boost.Signals2 演示"怎么把会话挂进实体-组件框架"。
   甲方有自己的框架和事件机制，**壳可以自己写，会话直接用**。
3. **验收文件是交付判据**。每层一份中文日志（四段同一行，见 §7），甲方按原文
   指标名搜索即可核对。两个核心场景合计覆盖验收目录 77/78 项。

## 1. 交付物清单（拿到什么）

| 交付物 | 位置（消费工程布局） | 作用 |
| --- | --- | --- |
| 公共头 | `include/1q/` | 全部库 API；带 UTF-8 BOM，VS2015 直接编 |
| 静态库 | `lib/1q.lib` | 全部模块（Release，五层验收开关已 ON） |
| 二进制依赖 | `lib/sqlite3_vendor.lib`、`lib/zlibstatic.lib` | 链接 `1q.lib` 时一并链上 |
| boost 头 | `third_party/boost/` | **仅示例组件层的事件库（signals2）需要**；只用会话（场景 B / 自写组件壳）不需要 |
| 识别特征库 | `src/basic_config/remote_identification_radar/target_feature_database_v1.1.db` | RIR 型号识别交付库（SQLite） |
| 配置模板 | `src/basic_config/*.json` | 六域基础 session 模板（含 `remote_identification_radar.json`，甲方链路参数已填；场景自持配置的拷贝源） |
| 示例源码 | `src/`（core/components/app/scenes/logger/basic_config/common 七目录） | 零改动可编译的集成参考（场景可执行集 + 通用 runner 的完整实现） |

**编译口径**（VS2015 交付档）：C++11、x64、**全链不 `/utf-8`**（头文件自带 BOM，
cl 自动按 UTF-8 解码；这正是交付验证要证明的约束）。工程定义参考
`D:\1q\1q_consumer\CMakeLists.txt`：`ONEQ_STATIC_DEFINE` 必须定义（关 dllimport）；
链接顺序 `1q.lib + sqlite3_vendor.lib + zlibstatic.lib`。

## 2. 两个核心场景一览

| | 场景 A：`rir_ground_site_recognition` | 场景 B：`sbirs_triple_sat_fix_messages` |
| --- | --- | --- |
| 宿主 | `rir_ground_site_recognition`（场景可执行；同源也可用 `component_attachment_demo --scene`） | `sbirs_triple_sat_fix_messages` / `precision_evaluation_demo`（三星实体 + 消息机制地面站融合组件） |
| 被测 | RIR 地基站点识别链（RCS+运动真值四维特征）+ 运行期指令指定任务 + 特征量测进五源融合 + 推演 + 威胁 | 三星 GEO 红外交会 + 消息投递地面站融合 + 推演 + **精度评估（五项误差 + AHP）** |
| 周期 | 400 × 1 s（场景 JSON 里改） | 80 × 1 s |
| 集成形态 | 挂 5 类组件（§4） | 三卫星实体挂 SBIRS（`kMessage` 投递）；地面站挂 `GroundStationFusionComponent`（内含 `FusionEngine` + `PrecisionEvaluationSession`）（§6） |
| 验收文件 | `rir_acceptance.log` + `rir_antenna_pattern.csv` + `rir_scan_pattern.csv` + `fusion/inference_acceptance.log` | `precision/opir/fusion/inference_acceptance.log` 四份 |
| 集成端日志 | `integration_events.log` + `integration_views.log`（rir/inference/threat 视图行） | `integration_events.log` / `integration_views.log`（消息投递与回执事件） |
| 冒烟判据 | demo 退出码 0（航迹≥2、融合≥1、视图/CSV 行数达标） | demo 退出码 0（五指标有样本、AHP 合法、`dual_sat_cycles>0`） |

场景 B 走三卫星实体 + 消息机制地面站融合组件；场景 A 才需要机载传感器挂载与事件接线。

## 3. 验收开关与日志路径（先打通这个）

五个验收文件开关是**库编译期**的（交付的 `1q.lib` 已全开；自建库时用
`1q_log_vs2015` preset 或按下表传 `-D`）：

| CMake 开关（编译库时） | 控制文件 |
| --- | --- |
| `ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG` | `opir_acceptance.log` + `IRInternalInterface.log` |
| `ONEQ_ENABLE_RIR_ACCEPTANCE_LOG` | `rir_acceptance.log` + `RadarInterface.log` + `RadarInternalInterface.log`（+ 两份雷达 CSV） |
| `ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG` | `fusion_acceptance.log` |
| `ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG` | `inference_acceptance.log` |
| `ONEQ_ENABLE_PRECISION_EVALUATION_LOG` | `precision_acceptance.log` |

**运行时路径**：默认落在进程当前目录 `log/` 下；集成方可用以下环境变量把
单文件钉到自己的输出目录（示例的 `InitIntegrationLog` 就是这么做的）：

```
ONEQ_OPIR_ACCEPTANCE_LOG_PATH / ONEQ_RIR_ACCEPTANCE_LOG_PATH /
ONEQ_IR_INTERNAL_INTERFACE_LOG_PATH / ONEQ_RADAR_INTERNAL_INTERFACE_LOG_PATH /
ONEQ_RADAR_INTERFACE_LOG_PATH /
ONEQ_FUSION_ACCEPTANCE_LOG_PATH / ONEQ_INFERENCE_ACCEPTANCE_LOG_PATH /
ONEQ_PRECISION_ACCEPTANCE_LOG_PATH = <目录>/<文件名>
```

场景 B 的 demo 还有一个**示例编译期**开关 `PE_ACCEPTANCE_LOG_ENABLED=1`（钉盘
分支），消费工程 CMakeLists 里照抄即可。

## 4. 场景 A：需要集成哪些组件

### 4.1 组件清单（必须挂的 5 类）

| # | 实体 | 组件 | 封装的库对象 | 每周期输入 | 产出 |
| --- | --- | --- | --- | --- | --- |
| 1 | `rir_ground_site`（**独立地基实体，先于平台创建**） | `RirSensorComponent` | `RirSession::Create(configs.rir)` | 站点局部 ENU 场景目标（`RirSceneTarget`，含识别特征真值）+ 站点 ECEF | `RirOutputFrame`（航迹/识别/量测）；量测经库适配器 `AdaptRirFeatureMeasurementsToDetectionRecords`（源通道 `kRirSourceId=5`）供融合；写 `rir_acceptance.log` |
| 2 | `platform` | `FlightComponent` | flight_dynamic（FD 门控；Windows 走运动学回退） | 场景航路（LLA 航点） | 平台位姿推进；发布 `on_platform_state` / `on_waypoint_reached` |
| 3 | `platform` | `FusionComponent` | `fusion::FusionEngine(scene_data.fusion)` | 本周期各源 `DetectionRecord`（含 RIR 源 5） | `FusedTarget` 态势（含逐航迹 UKF 运动学估计，库默认开）；发布 `on_fusion_updated`；写 `fusion_acceptance.log` |
| 4 | `platform` | `InferenceComponent` | `target_inference::TargetInferenceEngine`（纯函数式） | 融合目标的运动学估计 | 轨迹预报/发射点/落点/类型概率；写 `inference_acceptance.log`；`[视图:inference]` 输出发射/落点 LLA |
| 5 | `platform` | `ThreatComponent` | threat_assessment（`ThreatEvaluator`） | 融合态势置信度 | 威胁分/等级；等级升级发 `on_threat_updated`；`[视图:threat]` |

### 4.2 明确**不挂**的（挂了反而坏）

AR / ESR / EOS / SBIRS / SAR / ECM 六个机载组件**全部不挂**（场景 JSON
`sensors.* = false`）。原因：本场景只验 RIR 链路；不挂的通道不写视图行、不写
排除原因（日志干净，验收计数不被机载通道灌满）。融合的 `source_weights`
按场景配六项（源 0–5），本场景只有源 5（RIR）有量测进来，其余源的权重不起
作用，无需改动。

### 4.3 顺序约束（三条，违反即行为变化）

1. **`rir_ground_site` 实体先于 `platform` 创建**：实体按创建序步进，站点先步进，
   平台上的融合组件才能在本周期内读到 RIR 同周期量测（无跨周期滞后）。
2. **平台内挂载序 = 步进序**：`Flight → Fusion → Inference → Threat`
   （传感器位在 Flight 后、Fusion 前；本场景只有 RIR 在另一实体）。
3. **每周期先更新共享场景状态再步进**：周期号、仿真时间、世界真值（ECEF）由
   集成方注入一次，各组件从中取数。

### 4.4 数据流（每周期）

```
集成方注入：cycle/t_sec + 两目标 ECEF 真值（场景脚本推进）
  ↓ (ECEF → 站点局部 ENU + 识别特征铺样 = 集成方职责，示例 MakeRirSceneTargets)
rir_ground_site.Step → RirSession::Step(RirCycleInput) → 航迹/识别/量测
  ↓ 量测适配（库适配器，源 5）
platform: Flight.Step（位姿）
        Fusion.Step（聚合各源量测 → FusionEngine::Update）
        Inference.Step（融合运动学 → TargetInferenceEngine::Infer）
        Threat.Step（融合置信度 → ThreatEvaluator）
  ↓ 视图目标位置 ENU → LLA（集成方用库公开函数 TryEnuToEcef + TryEcefToLla，
    示例 sensor_utils.h 的 TryEnuMetersToLla 可照抄）
integration_views.log：[视图:rir]（航迹+位置LLA）→ [视图:inference]（发射/落点LLA）
                    → [视图:threat]
```

## 5. 场景 A：怎么集成到甲方框架（七步）

示例的 `core/`（component.h / entity.h / world.h / signals.h / events.h）总共
约几百行，是"怎么包会话"的参考实现。甲方框架里等价做法：

1. **建两个实体**：地基站点实体 + 平台实体（创建序：站点在前）。甲方实体容器
   只要保证"按挂载/创建顺序每周期调一次 Step"即可，接口形态随意。
2. **站点实体上放一个"RIR 组件"**：内部 `RirSession rir = RirSession::Create(cfg);`
   （cfg 从 `configs/remote_identification_radar.json` 加载，识别库路径指到
   `target_feature_database_v1.1.db`）；每周期把目标真值转成站点局部 ENU 的
   `RirCycleInput` 调 `StepWithResult`。识别结论/指定任务的事件**只进日志**
   （示例没有为 RIR 定义信号），甲方要事件就监听自己组件的输出 diff。
3. **平台实体上放四个组件**，各包一个库对象（§4.1 表第 2–5 行）。融合聚合
   RIR 量测：直接函数调用/查组件引用都行（示例用同实体类型化访问
   `host.Find<T>()` + 跨实体 `FindEntity`），**关键是融合在步进时能拿到本周期
   各源记录**——用甲方的事件机制做"本周期量测就绪"通知亦可。
4. **事件接线**：示例 11 个信号里本场景实际发布 4 个——`on_platform_state`、
   `on_waypoint_reached`、`on_fusion_updated`、`on_threat_updated`（其余 7 个
   属于不挂的机载通道）。甲方的等价事件通道照此四个映射即可；不接也不影响
   验收文件（信号只做通知/记录，不承载数据流）。
5. **LLA 视图**：库会话内部坐标是 ENU/ECEF；人读视图里的经纬高由**集成方**换算
   （ENU→ECEF→LLA，库公开函数直接可用，照抄 `sensor_utils.h`）。不做这步，
   验收文件仍完整，缺的只是 `integration_views.log` 里的 LLA 列。
6. **日志落盘**：会话创建前设 §3 的五个环境变量（或 `InitIntegrationLog` 的
   等价物）把验收文件钉到输出目录；集成端自己的日志可参考示例每个
   `CA_LOG_*` 写入点上方**已备好的同内容 `std::string`**（纯
   `std::string`/`std::to_string` 拼接，无任何示例依赖）——把该串写入甲方
   日志系统即完成对接，不需要示例的格式化门面。
7. **收尾断言**（示例 main 的做法，可照抄当冒烟）：退出码 0；融合目标 ≥1；
   RIR 航迹 ≥2；平台轨迹 CSV 行数 = 周期数；视图行数按 `--view-every` 求余达标。

**场景参数从哪来**：目标/航路/开关/融合权重全在
`scenes/rir_ground_site_recognition/rir_ground_site_recognition.json`（改 JSON 不改代码）；
雷达链路参数在 `configs/remote_identification_radar.json`（甲方参数与配套上限
对照表见场景 `.md`——峰值功率/脉宽/PRF/最大距离/驻留窗六项必须一起改）。

## 6. 场景 B：需要集成什么（三星实体 + 消息机制地面站）

三颗卫星各自 `SbirsSession::StepWithResult`，探测帧经 `on_sbirs_frame_submitted`
消息投递（`kMessage` 投递模式）写入地面站收件箱；地面站
`GroundStationFusionComponent` 内做适配、`FusionEngine::Update`，并挂
`PrecisionEvaluationSession` 对照真值打分。集成方：

```cpp
// 1) 三卫星实体 + 地面站实体（消息投递；融合引擎强制开逐航迹滤波）
world.CreateEntity("satellite_a").Attach(sbirs_a);  // 投递模式 kMessage
world.CreateEntity("satellite_b").Attach(sbirs_b);
world.CreateEntity("satellite_c").Attach(sbirs_c);
ground_station.Attach(GroundStationFusionComponent(engine, evaluation_session, src_a, src_b));

// 2) 每周期：调用方自己推进真值，写入星历/真值后 World::Step
for (cycle = 1..80) {
  for (auto& t : truth) t.position += t.velocity * dt;   // p += v·dt
  fusion->SetEvaluationInputs(ephemeris, truth);
  world.Step(dt);
  const auto& result = fusion->last_evaluation();
  // result.dual_sat / angular / velocity / keypoints 直接读
}
// 3) 结束：一次 SummarizeEvaluation → 五指标 mean/RMSE/P95/max + AHP 综合分
auto report = fusion->SummarizeEvaluation();
```

验收文件（precision/opir/fusion/inference 四份）由库会话写出，路径钉法同 §3。
冒烟判据照 demo：`all_metrics_sampled && ahp_valid && 0 < composite ≤ 1 &&
dual_sat_cycles > 0`（精度评估交会只用前两颗星——库 API 双视线边界，第三星
进融合不进交会指标）。

## 7. 验收核验清单（跑完逐项勾）

**四段行格式**（一行一条原文指标，直接按项名搜索）：

```
仿真时间=1.000s 仿真周期=1 [验收项：MTD增益] 验收内容：目标ID=1001 6.021dB
```

场景 A（400 周期；行数基线为 2026-08-28 场景精简前的实测，保留场景逐项
数值待重跑后更新）：

- [ ] demo 退出码 0
- [ ] `rir_acceptance.log` 存在且全为 `[验收项：` 行（含 `[规模目标识别功能测试]` 等改名后项名）
- [ ] `[验收项：目标测量角度与距离]` 有行，**无一行 `斜距=无`**
- [ ] `[验收项：IMM模型权重]`、`[验收项：独立目标识别器结论]`、`[验收项：极化特征解算]`、`[验收项：散射中心和轮廓特征]` 均有行
- [ ] `rir_antenna_pattern.csv`、`rir_scan_pattern.csv` 存在
- [ ] `integration_events.log` 含三行计时：`初始化时间`/`单步执行时间`/`单个模型加载时间`（模块=RIR）
- [ ] `integration_views.log`：`[视图:rir]`≈400 行量级、`[视图:inference]`/`[视图:threat]` 同步
- [ ] `platform_track.csv` 行数 = 周期数；`target_truth/route_plan/zones.csv` 存在
- [ ] `fusion_acceptance.log`、`inference_acceptance.log` 有四段行（RIR 源通道 5 并键）

场景 B（80 周期；基线同理待重跑更新）：

- [ ] demo 退出码 0；stdout 末行 `dual_sat_cycles=N/80`（N>0 即过冒烟）
- [ ] 四份验收文件齐：`precision`/`opir`/`fusion`/`inference`
- [ ] `[验收项：层次分析法]` 一行：综合分/等级/CR 有数；`[验收项：关键精度指标]` 有逐周期行 + 汇总行
- [ ] `[验收项：关机点预测]` = `暂无`（按设计，无推力模型）；`集群目标识别` 不落盘（按设计）
- [ ] 行数不是硬门控——判据是**文件齐 + 项名齐 + 冒烟过**

搜索示例（Windows，验收文件为 ANSI/GBK 编码）：

```bat
findstr /C:"[验收项：UKF滤波]" log\sbirs_triple_sat_fix_messages\fusion_acceptance.log
```

## 8. 已知限制与坑（集成前读一遍）

1. **验收文件编码 = 编译机的 ANSI 代码页**（交付链路无 `/utf-8`，中文 Windows
   上即 GBK）：记事本/`findstr` 直接可读；用强制 UTF-8 的工具会乱码。已知显示
   缺陷：单位 `m/s²` 在 GBK 落盘成 `m/s?`（GBK 无此字符，读数不受影响）。
2. **五个验收开关编进库**：拿交付的 `1q.lib` 不用管；**自建库时忘开开关 =
   文件一个都不出**（也不报错）。先跑场景 B 验通路。
3. **`rir_ground_site` 必须先于平台创建**（§4.3），顺序反了融合读不到同周期量测。
4. **融合参数会话级不可变**：`FusionEngine` 只在构造时接受 `FusionConfig`，
   没有运行时补丁接口（要热改参数须先提库需求）。
5. **别挂机载六组件**（§4.2）：不是"多挂更全"，而是视图/事件被无关通道灌满、
   验收计数失真。
6. **Windows 路径**：demo 的 `--output-dir` 传 Windows 路径（别从 WSL 传
   `/mnt/d/...`，Windows exe 会吃掉该路径）。
7. **RIR 型号确认在本场景不是门控**：交付特征库无 `RCS-0.025`/`RCS-1.0` 型号，
   综合分 ~0.10、全程无型号确认是**预期行为**（验识别确认用
   `rir_ground_site_recognition` 专项场景）。
8. **场景 B 的几何是评估配方**（SNR 门 0.001、三星静止 7000 km）：宽视场四角
   `miss`、最大探测距离 1e10 m 量级是配方产物，**不要当装备指标**；宽窄交接
   专项（原 `sbirs_wfov_nfov_handover`）已随 2026-08-28 场景集精简移除。

## 附：事件信号全集（示例层，供甲方映射自己的事件机制）

| 信号（Boost.Signals2） | 事件 | 发布方 | 场景 A 是否发布 |
| --- | --- | --- | --- |
| `on_platform_state` | 平台状态（每周期） | FlightComponent | 是 |
| `on_waypoint_reached` | 航点到达 | FlightComponent | 是 |
| `on_target_confirmed` / `on_target_lost` | AR 航迹首确认/失跟 | ArSensorComponent | 否（未挂） |
| `on_emitter_hypothesis` | ESR 辐射源假设 | EsrSensorComponent | 否（未挂） |
| `on_eos_detection` | EOS 探测生命周期 | EosSensorComponent | 否（未挂） |
| `on_sbirs_detection` | SBIRS 探测生命周期 | SbirsSensorComponent | 否（未挂） |
| `on_sar_product` | SAR 产品生命周期 | SarSensorComponent | 否（未挂） |
| `on_fusion_updated` | 融合态势更新 | FusionComponent | 是 |
| `on_threat_updated` | 威胁等级升级 | ThreatComponent | 是 |
| `on_command_issued` | 决策指令（事件链末梢） | DecisionListener | 视门限 |

RIR 的识别结论/指定任务变化**只写集成端事件日志**（`[事件:rir_recognition]`、
`[事件:rir_designation]`），无信号——甲方需要事件时在自己的组件壳里做状态差分。
