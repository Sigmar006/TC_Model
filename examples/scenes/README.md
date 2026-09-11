# 场景系统（scenes/）

每个场景一个子目录 `<name>/<name>.json`（场景描述）+ `<name>.md`（预期事件表归档）。
场景文件即配置记录（git 跟踪），新场景只加子目录 + JSON + 预期表，不改代码。

## 场景设计：六自由度机动从起飞开始

**场景由场景描述文件驱动**：平台飞行脚本、目标脚本、ESR 波形、天基平台、EOS 扫描、
SAR 任务几何/链路、融合配置与冒烟下限全部数据化——新场景 = 新 JSON 文件 + 重跑，
无需改代码；`scene_script` 只保留"目标脚本 → ECEF 状态 → 各通道真值"的纯转换。下述
基线行为对应 `baseline_takeoff_east/baseline_takeoff_east.json`。

`FlightComponent` 的 FD 路径遵循 `FlightManager.h` `Step(dt)` @note 的集成契约
（同款用法见 `tests/unit/flight_dynamic/fd_tools/takeoff_land_csv.cpp` 开发期工具；
**不做空中配平**——空中配平虽允许但存在不稳定问题）：

- **初始条件**：机场地面（alt 0）、零速度、姿态水平、`do_trim = false`；
- **机动队列**：`kTakeoff`（滑跑→抬轮→爬升到巡航高度）→ 航路点巡航
  （kFlyToWaypoint）→ `kLand`（降落目标 = 航路终点，场景简化）；
- **子步进 10 ms**：地面滑跑/起落架为快动态，100 ms 步长会失控发散
  （实测起飞段 roll 达 180° 量级后数值崩溃；10 ms 与集成契约一致）；
- **运动学回退**（FD 关闭/初始化失败）：模拟起飞爬升（5 m/s 到巡航高度）+
  巡航直线，行为语义一致。

演示场景（400 周期）：机场 (30°N, 120°E) → 起飞东飞 → 巡航 400 m / ~50 m/s →
3 个航点（正东，间距约 5 km）。目标真值 2 个空中目标固定 400 m 高度、**正北**
12/14 km（东速 47 m/s = FD 巡航实际地速，保持目标恒在平台正北侧方；北速
±5 m/s 提供两目标分离与 AR 径向速度）。目标方位（正北）落在 EOS 扫描覆盖内
（平台局部系 az 0 = 东，扫描 50°~130°，覆盖正北 az 90°）。EOS 探测距离窗
≈ [11.5, 22.9] km（400 m 高度 × 俯仰角 2°/1°），目标斜距全程稳定在窗内
（起飞爬升期平台高度不足、窗口窄，探测从爬升后期开始）。FD 模式实测（目标
位置修复后）：起飞 ~157 s 完成、航点 0 在演示窗口内到达（`waypoint_reached`
事件）、EOS 生命周期事件 ~300 条（首发现/丢失各 ~150 次，cycle 101 起——
起飞段平台向西北爬升（FD 实际航迹 hdg 293°→358°），目标相对平台方位从 90°
快速扫到 ~59°，cycle 1-100 恰好不在扫描波束窗内）；运动学模式下 3 个航点
全部到达、EOS 探测记录 ~177 次。事件目标 ID 为外部原始目标标识（1001/1002），
无外部标识时回退 AR 内部关联键。

### 区域巡逻（coverage）

场景 `platform` 块可选 `coverage` 字段声明巡逻区域与规划参数，加载时经
`navigation::AreaCoveragePlanner`（多边形牛耕式扫描 / 圆形盘旋，独立中立算法面）
生成巡逻航路，平台沿航路**循环巡逻**。`coverage` 与显式 `platform.waypoints`
互斥（航路来源歧义报错）；规划失败（几何非法/模式-区域不匹配）报错退出，
不允许静默退化为直飞。

### 多机编队（platforms[]）

场景可选顶层 `platforms[]` 数组声明**从机**（多机编队）：每条目同 `platform`
块字段（原点/航向/巡航 + `waypoints` 或 `coverage` 区域任务），另加可选
`name`（实体名，缺省 `wingman_<N>`）。主平台（`platform` 块）挂载五传感器
与融合（感知资产）；从机**纯飞行**（只挂 FlightComponent），各自航路/区域
任务 = "不同指令"。飞行器编号（aircraft_id） = 1（主）+ 2..N（按数组序），
落盘到共享可视化契约（platform_track/route_plan 的 aircraft_id 列）。

### 编队区域切分（mission_area）

场景可选顶层 `mission_area` 块声明**单个覆盖区域**（多边形 / 圆形）+ 规划参数
（字段同 `coverage` 块）——"主机收到一个区域"的任务语义：加载时由 example
业务层切分算法（`area_division.*`）自动把区域分为每架飞机一份（**分工覆盖**，
合起来恰好覆盖整个区域，非多机重复同航路）：

- **多边形**（scan）：沿扫描航向切成**等宽条带**（条带法向 = 扫描线法向），
  逐条带经 `AreaCoveragePlanner` 牛耕扫描——条带边界共用（裁剪含等号），
  相邻机子区域无缝无重叠；每机仍独立半间距偏移布线。**一般简单多边形通用**
  （不规则/凹多边形无需特判）：凹口跨条带边界时该条带子区域为多顶点多边形
  （如 L 形切 3 条带 → 矩形/六边形/矩形），扫描线按子区域自身几何生成；
  轴对齐条带对简单连通多边形恒非空（连通像区间），仅退化（近共线/零宽接触）
  报错；
- **圆形**（orbit）：切成**同心环**，环半径外 → 内算术均匀
  （r·(N−j)/N，主机最外环），每机单环盘旋（`orbit_rings` 由切分接管，
  强制 1）——环覆盖为线覆盖近似（圆心小盘与环间空隙不扫，同 planner
  多环语义，已知限制）。

切分结果逐机填入各自 `coverage`（子区域 + 规划参数）并生成巡逻航路，与
手工分配（各平台自带 `coverage`）在装配后完全等价（FlightComponent 不变）。
`mission_area` 与各平台 `waypoints`/`coverage` 块**互斥**（区域来源歧义报错，
同 waypoints/coverage 互斥先例）；编队数 = 1 + `platforms[]`.size()，无
从机报错（切分需 ≥ 2 架）；空子区域（退化多边形）报错退出，不静默降级。
参考场景：`fleet_area_division`（多边形 3 机条带）/ `fleet_area_division_circle`
（圆形 2 机同心环）。

**循环巡逻的执行语义（按 flight_dynamic 权威语义实现）**：

- **运动学回退路径**：航点寻的（每周期航向指向下一航点，段间瞬时转向），航路
  耗尽后索引回绕首个航点——几何干净，直线段序列可手算先验；
- **FD 模式**：航点簿记**消费库完成事件**（`FlightManager::GetWaypointEvents()`，
  每航点 1 条，含完成门/距离快照）——库的航点完成是双层语义（中间航点法平面
  穿越或到达圈、最终航点转弯量级捕获圈），组件自研几何判定与库语义必然错位，
  不参与 FD 模式；航路完成后 kCompleted → **以当前载机状态 Reset 重建续飞**
  （库状态机契约：kCompleted 必须 Reset 恢复 kReady；初始运动学取自
  VehicleState，do_trim=false 不做空中配平）——DIAG 仿真时间重置即循环证据。

`patrol_area_scan` 场景：巡逻区域 lat 29.9905~30.0045（约 1.56 km 南北）×
lon 120.0~120.02（约 1.93 km 东西），扫描航向 0（线沿东西）、间距 500 m →
3 条扫描线 = 6 航点（线纬度 29.99275/29.99725/30.00175），牛耕式交替方向；
到达半径 200 m（小于扫描间距，运动学路径防过渡航点连续到达合并）。
**cycles=600**（FD 模式起飞段 ~157 s 为硬开销，400 周期窗口装不下首轮+循环；
600 内 FD 首轮 ~423 s 完成 + 第二轮 4 航点可见，运动学模式 4 轮完整循环）。
目标相对基线**仅把 `v_east_mps` 47 归零**（保持正北 12/14 km 静止：扫描线
东西向飞行时目标恒在侧方，SAR squint ≈ 0 成像成立、EOS 方位恒在扫描扇区内；
`v_north_mps` ±5 保留，提供 AR 径向速度）——基线目标随平台同速东飞的设计
与"小区域往复巡逻"不兼容，属单变量控制调整（与 sbirs 场景归零 v_north 同理）。
南北过渡段（约 6 s/段）SAR 侧视几何不成立，被 squint 门控拒绝（**按设计
拒绝**，预期出现）。注意：SAR 首产出在 cycle 1（跑道低速假成像，SNR −2 dB
级，几何成立下的物理真实——基线同窗口是 squint 拒绝，因场景中心方位不同）；
产品事件 KEY 模式只落首条（持续类门控），sar_products=1 与基线一致。

### 天基通道（SBIRS / SAR）场景设计

- **SBIRS 天基平台（卫星）**：位置由消费方每周期注入共享场景状态（世界模型
  驱动），凝视模式固定于目标群中心正上方 +500 km（ECEF z 轴）；UTC 儒略日
  由场景提供（缺省 2024-01-01 00:00 UTC，`sbirs_satellite.utc_julian_day`
  可覆写）。SBIRS 的 az/el 为 **ECI 极坐标**（2026-08 正式变更：输入仍为
  ECEF，库内按 GMST 旋转到 ECI；`az=atan2(y,x)`、`el=asin(z/r)`，见
  SbirsGeometry 与 1q/coordinate/inertial_transform.h），因此扫描配置为全向
  （span 360°）+ 下视（scan_center_el −90°），目标始终位于星下点附近
  （FOV 20° 覆盖，GMST 引起的 az 平移不影响全向覆盖）。**示例简化声明**：
  星载方位参考系（ECI 极坐标）与机载通道方位（平台局部系）不同，融合方位
  相干门限（8°）无法跨参考系关联，SBIRS 探测在融合中独立成目标（通道 4）——
  演示"异构通道聚合"而非跨源关联。
- **SAR 侧视几何**：SAR 的 squint 门控（库内 `max_allowed_squint_angle_deg`
  = 10°，覆写自 sar.json 的 5°）要求平台视线垂直航迹。目标与平台同速东飞
  使目标恒在平台正北侧方，场景中心配置在 FD 巡航段（~cycle 331 起正东直线）
  的侧方（30.117°N, 120.06°E），巡航段飞越时 squint ≈ 0°、成像窗口覆盖
  ~cycle 19-397（产品事件 ~200 条）；起飞/转弯段（cycle 1-330 的绕飞与转向）
  侧视几何不成立，被库内 squint 门控拒绝（`PROJECT_LOG_ERROR` 每失败周期
  一条，**预期行为**：真实系统在几何不满足时同样不成像）。
- **SAR 链路预算**：目标 RCS 仅 2.2/1.4 m²，在 sar.json 的 10 kW 峰值功率 +
  30 dBi 天线下 13 km 斜距链路 SNR ≈ −29 dB（低于 minimum_snr_db）→ demo
  覆写峰值功率 1 MW、天线增益 40 dBi（SAR 常用量级）；目标位置修复后点目标
  回到场景中心附近真实位置（正北 12/14 km、400 m 高，斜距 ≈ 13 km），实测
  成像期 SNR ≈ 1.5~5.3 dB（高于 minimum_snr_db，L1 成像成立）；
  孔径 1024 脉冲 @ PRF 100 Hz ≈ 10.24 s 积累，逐周期滚动成像。
- **SAR 平台状态简化**：组件从 FlightComponent 读 LLA 位置与航向/速度，航向
  分解为 NED 速度（北/东分量），**姿态零假设**（roll/pitch 0、yaw = 航向）
  ——FD 模式未对外暴露姿态，示例简化。

> 生命周期事件语义说明：AR/EOS/SBIRS 传感器组件的"首确认/失跟"、"首发现/更新/丢失"
> 事件与 SAR 的"产出/持续/丢失/失败"事件由**库内 LifecycleRecorder**
> （`ArTrackLifecycleRecorder` / `EosDetectionLifecycleRecorder` /
> `SbirsDetectionLifecycleRecorder` / `SarProductLifecycleRecorder`）承担——
> Attach 到会话后 `StepWithResult` 内部自动驱动跨周期状态差分，集成侧不再
> 自研状态集合判定（消除掉轨漏报/集合无界等自研 bug）。`EosDetectionEvent.kind`
> / `SbirsDetectionEvent.kind` / `SarProductEvent.kind` 标注生命周期类型；
> 诊断类事件（`kNotDetected` / `kNoProduct`）未开启不转发。本场景 EOS 扫描
> 步进（20°/周期）大于波束宽度，目标约每 4 周期被探测 1 周期，故事件流以
> "首发现→丢失"交替为主、`kUpdated` 稀少——这是场景物理（扫描断续）在
> 生命周期语义下的自然表现。SAR 的 squint 门控失败周期（起飞/转弯段）不产生
> 生命周期事件（recorder 对非执行周期静默），属库内契约。

## 场景描述文件（数据驱动，schema v2：session_config 自持）

场景 = 消费方世界模型 + 业务调参的数据化载体。每场景目录自带三件套：
`<name>.json`（场景描述）+ `<name>.md`（期望表）+ `main.cpp`（薄入口，编译为
同名可执行——`ONEQ_SCENE_JSON` 钉死本场景，装配与执行共用 `app/runner.h` 的
`RunScene`；调试参数 `--cycles/--view-every/--output-dir`）。

加载见本目录 `scene_data.h/.cpp`（`LoadSceneData` 四参重载 = 场景层 +
`session_config`；解析复用 `examples/common/json_reader.h` 与各域
config_loader，遵循惯例：块内缺省字段静默默认、语法错误与必填几何字段缺失
报错）。

顶层结构（`baseline_takeoff_east/baseline_takeoff_east.json` 为基线样例）：

| 块 | 必填 | 字段（缺省值） |
| --- | --- | --- |
| `name` / `cycles` / `dt_sec` | 否 | 场景名 / 周期数（400）/ 步长 s（1.0） |
| `log_dir` | **是** | 本场景日志输出目录（相对 `examples/log/` 的路径，如 `"baseline_takeoff_east"`；须相对且不含 `..`。运行期对最终目录做临时区硬拒——`--output-dir` 覆盖同样受约束） |
| `platform` | **是** | `origin_lat_deg`/`origin_lon_deg`（**必填**）、`origin_alt_m`（0）、`initial_heading_deg`（90）、`cruise_altitude_m`（400）、`cruise_speed_mps`（50）、`waypoints[]`（lat/lon 必填，alt/speed 缺省回退巡航参数、radius 500；**与块内 `coverage` 互斥**）、`coverage`（可选区域巡逻任务，字段见下） |
| `platforms[]` | 否 | 从机数组（多机编队，纯飞行不挂传感器）：每条目同 `platform` 块字段 + `name`（缺省 `wingman_<N>`）；巡航参数缺省回退主平台值 |
| `coverage`（platform/platforms[] 条目内） | 否 | 区域巡逻任务：`kind`（polygon/circle）、`mode`（scan/orbit，须与 kind 匹配）、polygon `vertices[]`（lat/lon 必填）或 circle `center` + `radius_m`、`scan_heading_deg`（0 = 扫描线沿正东）、`scan_spacing_m`（须 > 0）、`altitude_m`/`speed_mps`（缺省回退巡航参数）、`arrival_radius_m`（500）、`orbit_segments`（8）/`orbit_rings`（1）。加载时经 `navigation::AreaCoveragePlanner` 生成巡逻航路（填入该平台的 `waypoints`），**循环巡逻**（航路飞完回绕首航点）；规划失败（顶点 < 3/间距非正/模式-区域不匹配等）报错退出 |
| `mission_area`（顶层） | 否 | 编队区域切分任务（**与各平台 `waypoints`/`coverage` 互斥**；需 `platforms[]` ≥ 1）：字段同 `coverage` 块。加载时经 example 层 `area_division` 自动切分为每机子区域（多边形 = 沿扫描航向等宽条带；圆形 = 同心环，外 → 内算术均匀，`orbit_rings` 强制 1），再逐机经 `AreaCoveragePlanner` 生成巡逻航路并循环巡逻；切分失败（退化多边形/空条带/无从机）报错退出 |
| `targets[]` | **是**（可为空 = 无目标场景） | `id`/`rcs_m2`（**必填**）、`type`（`air`/`ground`/`ballistic`，缺省 air；ground = 地面目标，静止近地运动学点，可视化以不同线型标注；ballistic = 二体椭圆弹道弧线，见下）、`azimuth_deg`/`range_m`/`altitude_m`（air/ground **必填**；ballistic **禁止**）、`v_east_mps`/`v_north_mps`（0；ballistic 禁止）、`temperature_k`（0，EOS 外观）、`projected_area_m2`（0，EOS 外观）、`radiant_intensity_w_per_sr`（0，SBIRS 外观，W/sr——已折算温度/发射率/投影面积）、`emitter_center_frequency_hz`（0 = 不配辐射源）、`maneuvers[]`（可选变速机动表：`start_cycle` 必填且严格递增，`v_east_mps`/`v_north_mps` 缺省 0——**绝对速度分段匀速**，未指定分量 = 0，须写全；ballistic 禁止）、`rir`（可选识别特征真值块：`rcs_dbsm` 视角网格值 / `pol_ch1_dbsm`+`pol_ch2_dbsm` 极化双通道（**显式给值才铺极化样本**，0 dBsm 是合法值不能当缺省）/ `pol_cross_dbsm`+`pol_phase_vv_deg`（**显式键才置 has_***，供验收旁路构造 S）/ `truth_model` 真值型号名 / `scatterers[{offset_m,rcs_dbsm}]` 距离向散射中心） |
| `targets[]` `type: "ballistic"`（弹道条目专用字段） | — | 与 ENU 几何字段（`azimuth_deg`/`range_m`/`altitude_m`/`v_east_mps`/`v_north_mps`/`maneuvers`）**互斥**（出现即报错）。必填：`start_lla_deg_m`/`end_lla_deg_m`（`[lat_deg, lon_deg, alt_m]` 数组）、`max_alt_m`（>0，弹道顶高）、`max_alt_time_s`（>0，数据顶高时刻）。加载时闭式解二体椭圆（三约束：过起/落点 WGS84 ECEF + 顶点 6371 km+max_alt，见 `scenes/ballistic_trajectory.h`）；两段式时间基准：助推段 [0, t_bo) 保持发射点（速度 0→v_bo 线性爬升），t ≥ t_bo 按 Kepler 方程解析推进，场景顶高时刻精确重建 `max_alt_time_s`；推进按绝对时间 t = cycle·dt 求值，无累积漂移。起落点重合/对跖、顶点不高于端点 → 加载期报错 |
| `esr` | 否 | **辐射源波形真值**（目标侧发射机参数，非 ESR 传感器配置）：`peak_gain_dbi`（30）、`bandwidth_hz`（2e6）、`peak_power_w`（5e7）、`pulse_width_s`（1e-6）、`pri_s`（1e-3）、`pulse_count`（200）、`timing_seed`（42） |
| `sbirs_satellite` | 否 | 天基平台几何：`altitude_m`（500000，凝视目标群质心正上方）、`utc_julian_day`（2460310.5 = 2024-01-01 00:00 UTC；SBIRS ECI 输出参考系必需，GMST 平移 az 不影响全向覆盖）。焦平面/命中门等会话量在 `session_config.sbirs` |
| `sensors` | 否 | 机载传感器挂载：`ar`/`esr`/`eos`/`sbirs`/`sar`（全 true）。`false` 则不挂该组件，不写该通道视图/排除原因。RIR/ECM 仍用各自 `enabled`；ECM 还要求 `esr=true` |
| `rir` | 否 | RIR 地基识别雷达站点块：`enabled`（false）、`site{lat_deg,lon_deg,alt_m}`（站点 LLA = 雷达局部 ENU 原点，缺省 30/120/0）；enabled 时整机挂载独立地基实体（先于平台创建），识别库经编译定义注入。指定识别任务不再随场景配置——经 `commands[]` 指令或威胁闭环运行期下发 |
| `ecm` | 否 | ECM 挂载开关：`enabled`（false；须 `esr=true` 挂载序） |
| `commands[]` | 否 | 运行期指令脚本（外部指挥系统代理）：`cycle`（**必填** ≥ 1，下发周期）、`kind`（`designate` 指定识别+锁定 / `engage` 交战 / `clear_designation` 清除指定，缺省 designate）、`target_id`（**designate/engage 必填**，须命中 `targets[].id`）、`duration_cycles`（0 = 无限期窗口）。主循环每周期 world.Step 前派发 → 指令事件 → CommandRouter 键解析 → 传感器运行期补丁（AR 切 STT 指定 + RIR 指定识别）；与威胁自动闭环（DecisionListener 的 `ENGAGE_HIGH_THREAT`）共用同一指令入口 |
| `high_threat_confidence` | 否 | 决策门限（3.0） |
| `smoke` | 否 | 冒烟下限：`min_key_events`/`min_sbirs_events`/`min_sar_products`/`min_fused_targets`（全 1；零产出场景显式置 0）、`min_rir_recognition_outputs`（0；RIR 确认态周期数下限） |
| `session_config` | **是** | **场景自持会话配置（挂载即全量）**：挂载的通道必带对应子块，未挂载通道**禁止携带**（loader 校验，"有配置 = 有挂载"一一对应）。子块：`ar`/`esr`/`eos`/`sbirs`/`sar`/`rir`（= `examples/basic_config/<域>.json` 模板整份拷贝后按场景改；`esr` 源文件为 `electronic_warfare.json`；结构见各模板与 `examples/common/config_loaders/`，缺省字段 = 库结构体默认）；`ecm`（仅 `ecm.enabled` 场景：`channel_count`/`maximum_total_transmit_power_w`/`maximum_channel_transmit_power_w`/`default_technique`（kSpot/kBarrage/kSweep/kDeception），基线 1/1000/1000/kSpot；设备 ID 统一由顶层 `device_ids.ecm` 配置，默认 101）；`fusion`（**恒必带**，可为空对象：`position_radius_m`（1000）/`bearing_beamwidth_deg`（5）/`feature_threshold`（0）/`window_size`（10）/`max_missed_cycles`（5）/`source_weights[]`（空 = 全 1.0））；`threat`（**恒必带**，可为空对象：权重/断点/阈值字段集见 loader 映射，缺省 = 库默认）。RIR 识别库路径按编译宏 `CA_RIR_DATABASE_PATH` 钉定。**新建场景的推荐做法：从 `examples/basic_config/` 拷模板 → 只留挂载通道 → 按场景改参** |

历史注：v1 场景的顶层 `eos_scan`/`sar`/`fusion`/`threat` 覆写块与
`sbirs_satellite` 的焦平面/命中门键已并入 `session_config` 对应子块（生效值 =
模板 ⊕ 场景覆写）；ECM 由代码默认（MakeDefaultEcmConfig）升为 `session_config.ecm`
数据。

## 现有场景集

每场景一份预期表归档（`<name>.md`）：

| 场景 | 被测行为 | 预期表结论 |
| --- | --- | --- |
| `baseline_takeoff_east` | 基线：全通道端到端（探测→融合→决策） | 通过（含 SBIRS 穿越质心注） |
| `no_targets_clean_airspace` | 空域清净：零假警（SAR 产品与点目标解耦） | 通过（1 项预期修正） |
| `target_maneuver_evasion` | 目标大机动：跟踪保持（AR 失跟需探测断链） | 通过（1 项预期修正） |
| `rir_ground_site_recognition` | RIR 地基站点专项：四维特征识别链（RCS+运动真值）+ 运行期指令下发的指定任务（cycle 30 designate 1001 → 识别达成回扫）+ 特征量测进五源融合 | 通过 |
| `patrol_area_scan` | 区域巡逻专项：coverage 块规划航路 + 循环巡逻（巡逻中四通道探测保持） | 通过 |
| `fleet_patrol_multi_zone` | 多机区域巡逻专项：3 机各自区域任务（platforms[]）+ 区域内空中/地面目标 + 契约 v2 多机可视化 | 通过（运动学冒烟 + **FD 600 周期复核**：三机 jsbsim、循环重启 5 次、SAR 起飞段 1 产品） |
| `fleet_area_division` | 编队切分专项（多边形）：顶层单个 `mission_area` 自动切 3 条等宽条带 + 逐机自动航路 + **同机场起飞**分工覆盖 | 通过（**FD 600 周期**：条带边界 29.990/29.999/30.008/30.017 无缝、扫描线纬度与手算逐点吻合、循环重启 9 次） |
| `fleet_area_division_circle` | 编队切分专项（圆形）：单个圆形 `mission_area` 自动切同心环（主机 2000 m / 僚机 1000 m）+ **同机场起飞**逐机盘旋 | 通过（**FD 400 周期**：环半径与手算吻合、`orbit_rings` 由切分接管、SAR 起飞段 1 产品） |
| `fleet_area_division_irregular` | 编队切分专项（不规则凹多边形）：单个 L 形 `mission_area` 自动切 3 条带（矩形/六边形/矩形）+ 逐机自动航路 + **同机场起飞** | 通过（**FD 600 周期**：六边形条带与手算逐点吻合、浮点边界伪重复顶点已修复、循环重启 7 次） |
| `threat_multi_target` | 威胁评估专项：三目标威胁分排序 + 等级映射 + 威胁→指令→AR STT 锁定闭环（cycle 1 指令 → cycle 2 锁定生效） | 通过 |
