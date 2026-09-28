# 自定义实体-组件示例（core/ + components/）

消费方业务层的集成参考示例：**自定义实体-组件**开发——不依赖 ECS 开源库，
用组件基类 + 挂载 + 事件库实现完整的感知→决策链路。

## 快速上手

**先读这几个文件**就能理解示例骨架：

1. `core/component.h` — 组件基类（虚接口，50 行）
2. `core/world.h` — 世界（实体注册表 + 步进 + 事件信号）
3. `app/runner.cpp` — 运行体（装配 + 编排 + 收尾，RunScene 为全部场景可执行共用）
4. `scenes/baseline_takeoff_east/` — 基线场景三件套（JSON + MD 期望表 + main.cpp 薄入口，改 JSON 就能跑不同参数）

**运行**：

```bash
cmake --preset llvm-ninja-release-local -DENABLE_EXAMPLES=ON [-DONEQ_ENABLE_FLIGHT_DYNAMIC=ON]
cmake --build --preset examples-release-local --target baseline_takeoff_east
./build/examples-release-local/bin/baseline_takeoff_east [--cycles <n>]   # 日志落 examples/log/<log_dir>/
# 通用 runner（任意场景 JSON，兼容入口）：
./build/examples-release-local/bin/component_attachment_demo [--scene <path>] [--cycles <n>] [--output-dir <dir>]
```

默认跑基线场景（400 周期），产物落在 `log/`（见"输出"节）。

## 定位

- **组件基类**：`core/component.h` 定义虚接口（Name / OnAttach / OnDetach / Step），
  组件**携带逻辑**（每周期 `Step` 驱动封装的一个库会话/引擎）；
- **模块 → 组件**：每个仿真模块对应一个 `Component` 子类（飞行 / AR / ESR / EOS /
  SBIRS / SAR / RIR 地基站点 / 融合 / 推演 / 威胁评估），**挂载**到实体上参与仿真；
- **事件机制**：使用 C++ 常见开源事件库 **Boost.Signals2**（零自定义分发层），
  提供跨周期通知/记录的事件通道。

单平台精简场景：1 个 `platform` 实体挂载 8 个组件，400 周期 × 1 s。四传感器
（AR / ESR / EOS / SBIRS）端到端：探测 → 融合 → 威胁评估 → 高置信威胁 → 决策指令（事件链）；
SAR 为图像产品通道（无探测输出，不入融合，发布产品生命周期事件）。场景 `rir`
块 enabled 时额外创建**独立地基站点实体**（先于平台创建保证融合读同周期量测）
挂载 RIR 识别雷达组件：识别结论/指定任务事件 + 特征量测经归属键重写进融合
（第 5 源通道）；未启用场景行为不变。

## 目录结构

```
examples/
├── core/                            自定义 ECS 核心（纯头文件）
│   ├── component.h                  组件基类（虚接口 + 生命周期钩子）
│   ├── entity.h                     实体（组件挂载容器，挂载序 = 步进序）
│   ├── world.h                      世界（实体注册表 + Step + 共享场景状态）
│   ├── signals.h                    事件信号集合（Boost.Signals2 具名信号）
│   └── events.h                     组件间通信的事件类型（纯数据结构）
├── logger/                          集成端日志设施（详见 logger/README.md）
│   ├── logger.h/.cpp                日志主体（InitIntegrationLog + CA_LOG_* 宏）
│   ├── logger_modes.h               日志模式选择区（视图/事件各三模式，编译期宏门控）
│   └── logger_i18n.h                issue code → 中文名适配表（纯查表零依赖）
├── components/                      模块组件（每个模块对应一个组件）
│   ├── flight_component.h/.cpp      FlightComponent：六自由度机动（起飞→航点→降落）
│   ├── ar_sensor_component.h/.cpp   ArSensorComponent：机载雷达会话
│   ├── esr_sensor_component.h/.cpp  EsrSensorComponent：电子侦察会话
│   ├── eos_sensor_component.h/.cpp  EosSensorComponent：光电会话
│   ├── sbirs_sensor_component.h/.cpp SbirsSensorComponent：天基红外会话（第 4 融合通道）
│   ├── sar_sensor_component.h/.cpp  SarSensorComponent：合成孔径雷达产品（不入融合）
│   ├── rir_sensor_component.h/.cpp  RirSensorComponent：地基识别雷达（场景可选，第 5 融合通道）
│   ├── fusion_component.h/.cpp      FusionComponent：多源融合引擎
│   ├── threat_component.h/.cpp      ThreatComponent：威胁评估
│   └── sensor_utils.h               平台坐标转换（ECEF 解析）
├── app/                             装配层
│   ├── runner.h/.cpp                RunScene 运行体（场景加载→装配→周期循环→摘要+冒烟，
│   │                                 全部场景可执行与通用 runner 共用）
│   ├── main.cpp                     通用 runner 入口（component_attachment_demo，--scene 任意场景）
│   ├── outputs.h/.cpp               可视化 CSV 落盘（AppOutputs：轨迹/真值/航路/区域）
│   ├── command_routing.h/.cpp       事件消费（DecisionListener 决策链 + CommandRouter 指令路由）
│   ├── fs_compat.h / output_dir.h   filesystem 别名 / 输出目录策略（临时区硬拒）
│   └── README.md → 见上级 examples/README.md
├── scenes/                          场景集（schema 权威见 scenes/README.md）
│   ├── scene_data.h/.cpp            场景描述加载（scenes/*.json → SceneData +
│   │                                 SceneSessionConfigs：session_config 挂载即全量；
│   │                                 coverage 块经 AreaCoveragePlanner 规划巡逻航路；
│   │                                 mission_area 块经 area_division 切分后逐机规划）
│   ├── scene_script.h/.cpp          世界模型目标真值脚本（场景目标脚本 → ECEF 状态 → 四通道周期真值 + 推进）
│   ├── area_division.h/.cpp         编队区域切分算法（example 业务层：单个覆盖区域 →
│   │                                 每机子区域，多边形 = 等宽条带、圆形 = 同心环）
│   └── <name>/{<name>.json + <name>.md + main.cpp}   每场景目录（薄入口 → 场景可执行）
├── core/                            本目录（ECS 内核）+ scene_types.h（AppSceneState 共享场景状态）
├── basic_config/                    六域 session 配置模板 + RIR 识别库资产
└── CMakeLists.txt（examples 根：examples_core 静态库 + 场景可执行自动发现 + 兼容目标）

> 传感器输出 → 融合探测记录的边界适配由库内官方适配器
> `fusion/SensorAdapters.h` 承担（四个 `Adapt*ToDetectionRecords` 函数 + 源通道
> 常量 `kArSourceId`..`kSbirsSourceId`，与 `FusionConfig::source_weights` 对齐）。

## ECS 核心设计

### 组件基类（core/component.h）

```cpp
class Component {
 public:
  virtual ~Component() = default;
  virtual const char* Name() const = 0;          // 类型标识（日志/卸载）
  virtual void OnAttach(Entity& host) {}         // 挂载钩子：保存宿主引用
  virtual void OnDetach() {}                     // 卸载钩子
  virtual void Step(World& world, double dt_sec) = 0;  // 周期推进
};
```

组件携带逻辑：每个模块组件封装一个库会话/引擎，`Step` 内驱动会话并产出状态。

### 实体与挂载（core/entity.h）

- 实体是组件的**挂载容器**：`Attach(std::unique_ptr<Component>)` 按序追加并触发
  `OnAttach`；`DetachByName` 触发 `OnDetach`；实体销毁时逐个 `OnDetach`（钩子对称）。
- **挂载序 = 周期执行序**：本示例
  `Flight → AR → ESR → EOS → SBIRS → SAR → Fusion`——传感器读到推进后的
  平台位姿，融合读到本周期探测（SAR 无探测，位置在传感器之后、融合之前）。
- **同实体组件通信**（周期内同步数据聚合）：`host.Find<T>()`（dynamic_cast
  类型化访问）。例如 `FusionComponent::Step` 聚合四个传感器组件的探测记录。

### 世界与共享上下文（core/world.h）

- `World` 是实体注册表：`CreateEntity(name)`（创建序 = 步进序）、
  `FindEntity(name)`、`Step(dt)` 顺序步进全部实体、`signals()` 事件通道。
- **共享场景状态**（跨实体上下文）：`SceneState` 基类（cycle/t_sec），
  消费方继承扩展（`AppSceneState` 增加四通道世界真值 + 天基平台位置 + SAR
  点目标），每周期更新、组件读取。

### 事件机制（core/signals.h + events.h）

事件机制直接使用 **Boost.Signals2**（C++ 常见开源事件库，boost 已在 conanfile
基础依赖，header-only）：

```cpp
// 发布（组件内）
world.signals().on_target_confirmed(event);
// 订阅（消费方）
boost::signals2::scoped_connection conn =
    world.signals().on_target_confirmed.connect(handler);
```

10 个具名信号（信号 = 事件通道，仅组织库对象，零自定义分发逻辑）：

| 信号 | 事件 | 发布方 |
| --- | --- | --- |
| `on_platform_state` | `PlatformStateEvent` | FlightComponent（每周期） |
| `on_waypoint_reached` | `WaypointReachedEvent` | FlightComponent（航点完成） |
| `on_target_confirmed` | `TargetConfirmedEvent` | ArSensorComponent（首确认，源：库内 ArTrackLifecycleRecorder） |
| `on_target_lost` | `TargetLostEvent` | ArSensorComponent（失跟，源：库内 ArTrackLifecycleRecorder） |
| `on_emitter_hypothesis` | `EmitterHypothesisEvent` | EsrSensorComponent（假设） |
| `on_eos_detection` | `EosDetectionEvent` | EosSensorComponent（首发现/更新/丢失，源：库内 EosDetectionLifecycleRecorder） |
| `on_sbirs_detection` | `SbirsDetectionEvent` | SbirsSensorComponent（首发现/更新/coasting/丢失，源：库内 SbirsDetectionLifecycleRecorder） |
| `on_sar_product` | `SarProductEvent` | SarSensorComponent（产出/持续/丢失/失败，源：库内 SarProductLifecycleRecorder） |
| `on_fusion_updated` | `FusionUpdatedEvent` | FusionComponent（态势更新） |
| `on_command_issued` | `CommandIssuedEvent` | DecisionListener（决策，事件链） |

两种通信形态同时演示：**周期内同步数据聚合**用组件类型化访问；**跨周期通知 /
记录**走信号（事件）。

> 日志体系（事件日志与调试视图落盘、三模式宏门控）见 [`logger/README.md`](logger/README.md)；
> 想快速理解"输出视图 / 两种日志 / 三模式"三者关系，读
> [`docs/practice/output_view_and_logging_guide.md`](../../docs/practice/output_view_and_logging_guide.md)（使用教程）。

## 模块 → 组件映射

| 组件 | 封装库模块 | 周期行为 | 发布信号 |
| --- | --- | --- | --- |
| `FlightComponent` | flight_dynamic（FD 门控 + 运动学回退） | 六自由度机动推进：起飞→航点巡航→降落；航点完成判定；**循环巡逻**（coverage 场景：FD 模式航点簿记消费库完成事件、kCompleted 后以当前状态 Reset 重建续飞；运动学回退路径带航点寻的 + 索引回绕，段间瞬时转向） | on_platform_state、on_waypoint_reached |
| `ArSensorComponent` | airborne_radar（ArSession + ArCycleOutputAdapter + ArTrackLifecycleRecorder） | 探测 → `DetectionRecord`（key=关联键，含位置）；首确认/失跟事件由库内 recorder 差分产生 | on_target_confirmed / on_target_lost |
| `EsrSensorComponent` | electronic_surveillance_radar（EsrSession） | 假设 → `DetectionRecord`（key=假设键，方位+射频特征） | on_emitter_hypothesis |
| `EosSensorComponent` | electro_optical_sensor（EosSession + TryMakeEnuSceneState 手填 CycleInput + EosDetectionLifecycleRecorder） | 探测 → `DetectionRecord`（key=0，仅方位）；首发现/更新/丢失事件由库内 recorder 差分产生 | on_eos_detection |
| `SbirsSensorComponent` | sbirs_sensor（SbirsSession + SbirsDetectionLifecycleRecorder） | 探测 → `DetectionRecord`（key=0，仅方位，与 EOS 同构）；首发现/更新/coasting/丢失事件由库内 recorder 差分产生 | on_sbirs_detection |
| `SarSensorComponent` | sar（SarSession + SarProductLifecycleRecorder） | 孔径积累成像；产品生命周期事件由库内 recorder 差分产生（**无探测输出，不入融合**，契约见 docs/review/Behavior.md）；阶段型调试视图每周期直写摘要行 | on_sar_product |
| `FusionComponent` | fusion（FusionEngine） | 聚合四传感器探测一次 `Update`；新/消失差分 | on_fusion_updated |
| `ThreatComponent` | threat_assessment（ThreatEvaluator） | 融合态势 + AR 调试视图按键组装输入 → 威胁分/等级；等级升级（含首见）→ 升级事件；每周期视图摘要行 | on_threat_updated |

## 运行时修改接口

每个组件把库模块的「运行时修改」设计暴露为组件公开方法（未来外部调用入口，
薄包装库 API，不改库行为）。提交语义随模块而异（权威定义：
`docs/common/contract.md`「运行期配置提交策略」）：

| 组件 | 接口 | 提交语义 |
| --- | --- | --- |
| `ArSensorComponent` | `bool TryApplyRuntimeConfig(const ArRuntimeConfigPatch&)` | **事务性提交**：补丁先暂存，下次成功周期边界统一生效（失败由库内快照完整回滚）；与现有配置冲突的非法补丁入口即原子拒绝 |
| `EsrSensorComponent` | `bool TryApplyRuntimeConfig(const EsrRuntimeConfigPatch&)`；`ApplyRuntimeConfigWithResult(...)` → 结构化结果（拒绝原因枚举） | **立即提交**：调用即生效、单向落定（session 层无回滚）；结构化结果供外部决策/诊断 |
| `EosSensorComponent` | `bool TryApplyRuntimeConfig(const EosRuntimeConfigPatch&)` | **立即提交**：补丁经 resolver 原子校验后一次生效；frame_rate_hz 热更新经 resolver 校验（非法值整补丁拒绝） |
| `SbirsSensorComponent` | `bool TryApplyRuntimeConfig(const SbirsRuntimeConfigPatch&)` | **立即提交**：补丁经配置校验后一次生效（如 scan_rate < 0 整补丁拒绝） |
| `SarSensorComponent` | `bool TryApplyRuntimeConfig(const SarRuntimeConfigPatch&)` | **立即提交**：补丁经配置校验后一次生效（如 enable_l1_rda_imaging 依赖 raw echo，依赖不满足时整补丁拒绝） |
| `FlightComponent` | `bool PushManeuver(const ManeuverCommand&)`；`bool ClearManeuvers()`；`bool Abort()` | **命令式**（FD 无 patch 范式）：追加机动队列/清空/中止；FD 未启用或初始化失败（运动学回退）时返回 false（指令被丢弃） |
| `FusionComponent` | 无 | fusion 模块参数为**会话级不可变**（库内无 RuntimeConfigPatch 设计，FusionEngine 仅构造时接受 FusionConfig）；需要运行时修改须先补库 API，不在示例层包装 |

补丁字段统一用 `has_*` 位标志选择（未设置的字段不覆盖现值）；传感器组件的
生命周期 recorder 事件源与运行时修改入口互不影响（recorder 只管差分事件，
patch 只改会话配置）。

## 实体状态查询接口（外置查询数据源）

选定实体后按实体名/ID 拉取设备状态的查询数据源：各传感器组件把**最近周期
执行真相**暴露为 const getter（每周期 `Step` 随会话结果刷新，外部系统读取
即"当前状态"；未步进前默认 `powered_on=true`，与 `sensor_enabled=true` 配置
默认一致）。有对应库语义的组件才提供函数：AR/SAR 无扫描方位概念（跟踪/
成像雷达），不提供角度函数。

| 组件 | 开关机 | 扫描方位 |
| --- | --- | --- |
| `ArSensorComponent` | `bool powered_on()` | —（无扫描方位概念） |
| `EsrSensorComponent` | `bool powered_on()` | `float scan_azimuth_deg()`（deg，平台系） |
| `EosSensorComponent` | `bool powered_on()` | `float scan_azimuth_deg()`（deg，平台系） |
| `SbirsSensorComponent` | `bool powered_on()` | `float scan_azimuth_deg()`（deg，ECI 极坐标参考——库内为弧度，组件转度） |
| `SarSensorComponent` | `bool powered_on()` | —（无扫描方位概念） |

有 DebugView 的四个组件（AR/EOS/SBIRS 目标列表型 + SAR 阶段型）另暴露
`const *OutputDebugView& LastDebugView()`——最近周期调试视图快照（规则 12 落盘示范：
per-target 状态 + 规则 13b kInfo 排除诊断；关机周期清零，拒绝周期为
`kCycleNotCompleted`/`kCycleNotExecuted` 快照）。组件在 `Step` 内取该视图
直写中文人读行到集成端视图日志 `integration_views.log`（每周期一行，如
`[视图:ar] 周期=5 完成=是 目标=[1001 已确认(RCS 2.20m^2)] 问题=[ar.target_snr_below_threshold 目标信噪比低于门限]`；
SAR 为阶段型摘要行）——日志给人读，结构化持久化由外部集成方接入自己的
日志/事件系统实现（示例不内置 JSON 序列化器）。落盘密度（三模式）由
`logger/logger_modes.h` 模式选择区宏控制。ESR 库内无 DebugView，不适用
视图落盘。

**组件层电源门控**：电源状态由 `sensor_enabled` 补丁唯一维护（`TryApplyRuntimeConfig`
成功且带 `has_sensor_enabled` 时更新，拒绝的补丁不改状态）。关机时组件**不驱动
会话**（设备不工作，会话扫描相位冻结），角度主动清零表示无有效扫描方位；重新
上电后组件恢复驱动，会话从冻结状态继续推进。角度字段在开机周期随会话输出帧
刷新（被拒绝周期为空帧 → 0）。消费方一律以 `powered_on()` 守卫：仅开机且最近
周期 `kCompleted` 时角度才代表有效扫描方位。demo 末尾
"Platform Sensor States (entity query)" 块演示按实体查询（`Entity::Find<T>()`）。

## 周期调用序

```
每周期（World::Step(1.0)）：
  platform 实体按挂载序步进：
    1. Flight    → 推进平台位姿（FD 子步进 10 ms × 100；或运动学回退）
                  → 发布 PlatformStateEvent / WaypointReachedEvent
    2. ArSensor  → 读 Flight 状态构建周期输入 → 驱动 ArSession
                  → 探测存自身 detections_，发布轨迹事件
    3. EsrSensor → 同上（EsrSession）
    4. EosSensor → 同上（EosSession）
    5. SbirsSensor → 读共享场景天基平台位置与红外目标真值 → 驱动 SbirsSession
                  → 探测存自身 detections_，发布探测生命周期事件
    6. SarSensor → 读 Flight 状态（LLA/NED）与 SAR 点目标真值 → 驱动 SarSession
                  → 发布产品生命周期事件（无 detections_，不入融合）
    7. Fusion    → 类型化聚合四传感器 detections_ → FusionEngine::Update
                  → 发布 FusionUpdatedEvent
    8. Threat    → 融合输出（置信度）+ AR 调试视图（运动学/RCS）按键组装
                  → ThreatEvaluator::Evaluate → 威胁分/等级
                  → 等级升级 → 升级事件，逐目标发布 ThreatUpdatedEvent
  订阅者（demo 侧）：
    DecisionListener → 置信度 ≥ 3.0 或威胁等级 HIGH → 发布 CommandIssuedEvent（事件链）
    组件宏（CA_LOG_EVENT / CA_LOG_VIEW）→ 事件与视图摘要就地记录 → integration_events.log / integration_views.log（人读行）
```

## 构建与运行

```bash
cmake --preset llvm-ninja-release-local -DENABLE_EXAMPLES=ON [-DONEQ_ENABLE_FLIGHT_DYNAMIC=ON]
cmake --build --preset examples-release-local --target baseline_takeoff_east
./build/examples-release-local/bin/baseline_takeoff_east [--cycles <n>]   # 日志落 examples/log/<log_dir>/
# 通用 runner（任意场景 JSON，兼容入口）：
./build/examples-release-local/bin/component_attachment_demo [--scene <path>] [--cycles <n>] [--output-dir <dir>]
```

- `--scene <path>`：场景描述文件（默认 `scenes/baseline_takeoff_east/baseline_takeoff_east.json`，路径由
  CMake 注入 `CA_SCENE_DIR`）；场景文件即配置记录，见 [`scenes/README.md`](scenes/README.md)；
- `--cycles <n>`：仿真周期数（覆盖场景文件，默认场景文件值）；
- `--view-every <n>`：视图摘要间隔（`周期 % n == 0` 才写；覆盖场景
  `view_log_every_cycles`，默认 1 = 每周期一行）；
- `--output-dir <dir>`：输出目录（日志 + CSV + 各层验收文件）。默认
  `examples/log/<场景名>/`（例如 `rir_ground_site_recognition`），
  不传则按场景钉死，避免 `RadarOutput.log` 等验收文件落到运行目录。显式传入则用该路径。
  运行时产物不入版本控制，见 .gitignore；
- 视图默认摘要模式；密度用 `--view-every` / 场景 `view_log_every_cycles`。
  编译期模式一般不用改（不传则：视图摘要 + 事件只记关键）：
  ```bash
  cmake --preset llvm-ninja-release-local -DENABLE_EXAMPLES=ON \
      -DCA_VIEW_LOG_MODE=summary -DCA_EVENT_LOG_MODE=aggregate
  ```
- FD 开启时输出 `FlightComponent` 的六自由度机动日志（JSBSim），关闭/失败时
  打印回退告警并走运动学路径。
- 本示例依赖 spdlog（conanfile 非 Windows 依赖），Windows 构建不纳入
  （examples/CMakeLists.txt 门控）。

## 输出

| 文件 | 内容 | 说明 |
| --- | --- | --- |
| `integration_events.log` | 每行一条人读记录 | **集成端事件日志**（spdlog 命名 logger `"integration_events"`）：事件行 `[事件:type] 周期=... 时间=...s 中文详情`（10 类信号事件 + 纯日志事件 `patrol_loop_restart`（巡逻循环重启，无信号，KEY 模式落盘），字符串归属组件源文件，`CA_LOG_EVENT` / `CA_LOG_EVENT_DUP` 宏；事件模式二为 `[事件聚合]` 行） |
| `integration_views.log` | 每行一条人读记录 | **集成端视图日志**（spdlog 命名 logger `"integration_views"`）：AR/EOS/SBIRS/SAR 四组件每周期调试视图行 `[视图:module] 中文摘要`（`CA_LOG_VIEW` 宏；日志给人读，落盘密度三模式由宏门控，见 [`logger/README.md`](logger/README.md)） |
| `1q_library.log` | 人读日志行 | **库内部日志**：库内 `PROJECT_LOG_*` 宏 → spdlog 默认 logger（时间戳 + 级别 + 消息），`InitIntegrationLog` 装配 |
| `platform_track.csv` | cycle,t_sec,lat_deg,lon_deg,alt_m,heading_deg,speed_mps,wp_index | 平台轨迹（每周期一行；FD 模式含起飞爬升段） |

控制台输出每周期一行摘要（平台高度/航向/速度/航点进度 + 融合目标数），事件
逐条打印；结束打印汇总（实体数/组件数/事件数/指令是否下发）。

## 测试

- `tests/unit/examples/ecs_core_test.cpp`：ECS 核心单测（实体挂载/卸载生命周期
  钩子与调用序、按名精确卸载、类型化访问与宿主兄弟组件通路、挂载序与创建序
  步进、共享场景状态、信号发布→订阅接线、事件类型隔离）；
- `tests/unit/examples/ecs_component_runtime_test.cpp`：组件运行时修改接口单测
  （AR 合法/非法 patch 的接受与原子拒绝、ESR 立即提交 + 结构化拒绝状态码、
  EOS 立即提交 + 整补丁拒绝、SBIRS/SAR 立即提交 + 整补丁拒绝、FlightComponent
  机动入口在 FD 可用/不可用时的返回语义）+ 状态查询与调试视图单测（开关机/
  扫描方位、AR/EOS/SBIRS/SAR 四通道 `LastDebugView()` 逐目标/阶段型状态与
  13b kInfo 排除诊断、关机清零、SBIRS 关机冻结相位）；
- 示例冒烟不进 ctest（场景可执行自带冒烟断言，exit code 即结果；400 周期 + 日志/CSV
  落盘 + 最小产出断言：关键事件 ≥ 1、SBIRS 关键探测事件 ≥ 1、SAR 关键产品事件
  ≥ 1、融合目标 ≥ 1、平台轨迹行数 = 周期数、视图行数按 `view_log_every_cycles`
  求余后的拍数断言；未挂传感器不计入视图/排除原因）。
