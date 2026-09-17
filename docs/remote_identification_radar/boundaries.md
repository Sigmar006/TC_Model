---
Status: active
Last-reviewed: 2026-08-27
Authority: RIR 模块级边界、非目标与设计变更规则
Answers: RIR 有哪些模块级禁令与边界、哪些非目标、单位纪律与失败降级契约
---

# Remote Identification Radar 模块边界

本文承载 RIR 的模块级边界、非目标与设计变更规则。算法级边界见
[algorithms.md](algorithms.md)。

## 装备前提与模块定位

RIR 是与机载雷达（AR）**相互独立的另一部雷达装备**，不是 AR 的工作模式或子能力
（2026-08-15 审计定案）。本模块由 AR 内被耦合的远程识别子系统（kLrr）解耦而来：

- **独立硬件**：自带 hardware 域（`RirHardwareConfig`：发射机/天线/接收机/RCS
  物理/信号处理增益），效能级 SNR 由模块内 `RirRadarEquations` 自算，不引用 AR
  内部实现。
- **独立输入面（阶段 2-S 已落地；RF 物理链 2026-08-19）**：与 AR 无任何模块间
  接口。输入为场景目标（含速度/名称/Swerling 起伏/识别特征真值）+ 必填平台 ECEF
  + 可选外部 `rf_scene`（非本机 emission；空表示无外部干扰）；自发射与 incident
  links 由库内 RF 链求解，集成方不再预算入射链路。环境事实经
  `RirSessionConfig.environment` / 运行期补丁注入，禁止周期输入；
  内部航迹由 RIR 自持检测与轻量跟踪生产。`RirTrackFeed` 公开供给已删除。
- **驻留指向（阶段 2-S；调度器入库 2026-08-17）**：RIR 自管的是“驻留候选排序”
  （消费内部航迹，语义为“未识别优先 + 斜距次近”；威胁等级输入随独立性消失）。
  每周期实际波束中心由**库内驻留调度器**（`RirSession`：相对可扫描体积 +
  转台朝向平移归一化，或指定识别任务限位执行）派生，控制器只消费并信任
  给定值（见下方驻留指向契约）。
  RIR 不驱动任何外部雷达波束。
- **搜索角域裁剪（2026-08-22 甲方批注「设定方位俯仰进行扫描」+「任务范围由用户指定」）**：
  检测候选集按**实际搜索扇区** = `mission.scan_window_deg`（任务扫描子窗，用户指定的
  作战搜索扇区）∩ `orientation.steerable_volume_deg`（硬件最大可扫描体积）裁剪——视线角
  出扇区（az 相对 `scan_center`、el 绝对，交集单源 `internal/IntersectScanSector`；
  角域判定单源 `internal/RirScanVolume.h`）的场景目标不入检测候选集；此前“遍历全部
  场景目标、波位只改方向图”的软门控行为废弃。**分层语义**：`steerable_volume` 为硬件
  最大界限（也是扫描波位与指定目标驻留的最终边界）；`scan_window` 为其内的作战搜索
  扇区，缺省无界 [-180,180]×[-90,90] 时交集退化为体积（与既有行为兼容）。扫描波位在
  实际搜索扇区内推进（`RirSession::BuildAbsoluteScanWaves`）。**指定豁免**：被
  `designated_external_target_id` 指定的目标即便落在子窗外，只要仍在 `steerable_volume`
  内即豁免子窗裁剪（算子显式指派高于常规搜索扇区）。`scan_center` 运行期补丁即
  「设定方位俯仰」；直连 `RirController::RunCycle` 的调用方默认不裁剪（子窗/体积均缺省
  无界）。出界目标逐周期落 `rir.target_outside_search_volume` kInfo 排除诊断（规则 13b，
  区分「出硬件体积」与「在体积内但出任务子窗」两类门，携带视线角与角域窗口），目标
  消失可归因、不静默；航迹按既有失跟语义自然消退。
- **地球遮挡门控（2026-08-27）**：检测候选在角域裁剪之前做有限弦-地球圆球判定
  （半径 6371 km，相切算遮挡，与 SBIRS 同口径；判定核 `common/geometry/EarthOccultation`）。
  平台 ECEF + 目标雷达局部 ENU 经 `TryEnuToEcef` 还原目标 ECEF；穿地目标不入检测
  候选，落 `rir.target_earth_occulted` kInfo（`cause=kNone`，message 带遮挡余量）。
  k 因子不进本门（只服务大气损耗）。ENU 还原失败则跳过本门，不伪装成遮挡。
  不负责地形、椭球或电波视距。
- **实际有效目标最大斜距（外部探测距离观测口径）**：`RirCycleResult` 新增
  `max_detected_slant_range_m`——本周期持有航迹的目标里最大输入几何斜距。它与
  `mission.max_range_m`（径向粗筛门，仅过滤超远候选）语义不同，反映 SNR 链路预算下真正
  成航迹的目标能达到的最远斜距，供外部集成方判断“实际探测距离”。本周期无航迹为 0；
  仅统计入检测候选（在实际搜索扇区内或被指定）且成航迹的目标，出扇区目标不计入。
- **自持检测链（阶段 2-S 已接线；跟踪升级 2-T N1-N7 已完成）**：需求所列九项
  信号链能力（天线方向图仿真、回波/干扰/噪声功率计算、四项处理增益、恒虚警
  检测）界定为 **RIR 自持检测链**（检测 → LAPJV 全局最优关联 → CV KF/IMM
  双路径滤波 → 池化生命周期 → 内部航迹 → 识别积累）；检测量测仅内部消费，
  不对外发布点迹；战术决策/ECCM/对外点迹仍为非目标。逐项归属与阶段切分的
  能力边界审计（2026-08-15）已删档，见 git 历史。
- **replay 会话配置**：阶段 1 只提供周期结果记录编解码（`rir_replay.fbs`）；
  会话配置 replay 与 trace 事件流列为阶段 2 评估项。

## 与 common 契约的关系

RIR 遵守 `docs/common/contract.md` 与 `docs/common/session_contract.md`：

1. public API 只暴露稳定 session/config/input/output/validation/replay DTO 门面；
   `RirController`、识别内部类型不通过 public header 暴露。
2. 会话配置直接赋值 `RirSessionConfig`；运行期热更新直接写 `RirRuntimeConfigPatch`
   （显式 `has_*`）；不提供 ConfigBuilder。
3. 输出遵守分层周期记录 + 可选投影（规则 15）：产品层 `RirOutputFrame`；副作用
   `emission_frame`；仿真附件 `track_attributions`。落地前仍为扁平 `RirCycleResult`。
   目标列表型三类投影为观测完备必选；字段冻结见
   `docs/review/rir_observability_projections_freeze_2026-08-21.md`。
4. 周期语义：非执行周期不复用上一帧；校验拒绝 `kRejectedInvalidInput` +
   明细 issues；关机 `kPoweredOff` 只推进世界时间；统一问题列表
   （规则 14，`RirIssueList`，code 前缀 `rir.validation.*`）。
5. 电源状态单源：`RirSessionConfig::sensor_enabled`，补丁入口
   `RirRuntimeConfigPatch::has_sensor_enabled`（COMMON-OQ-4 对齐）。
6. 跨域形状契约：`ONEQ_SENSOR_SESSION_CONTRACT` 锚定
   `RirSession::Step/StepWithResult` 签名。
7. 阶段 3 / 3b common 化：LAPJV / 雷达方程 / 天线方向图 / 植被传播损耗 / 大气胶水 /
   RCS 混合 / 检测单元账本 / 统计级 CFAR 编排 / 冻结波束 / 航迹池·关联核·生命周期
   计数已收敛到 `src/common/`，RIR 保留薄适配层，不引入 AR 头。发射/接收「可提取
   核心」与 6 dB 真值回退门留模块侧（提取审计 2026-08-21 已删档，见 git 历史）。

## 非目标（否决项）

1. ISAR / 二维距离-多普勒像、微动特征、在线学习/自适应权重、实时外部数据库联网、
   信号级 IQ/全波散射求解。
2. 非 `kIdentify` 模式激活识别链路；威胁分类混入识别输出；以场景真值直接产生结论。
3. 暴露内部识别类型为 public SPI；process-wide 识别全局状态。
4. 战斗级跟踪之外的关联决策/战术决策：RIR 自持跟踪为 **LAPJV 全局最优关联 +
   CV KF/IMM 双路径滤波 + 池化生命周期**（2026-08-15 跟踪升级 N1-N7 已落地；
   IMM 为 confirmed 命中激活，`enable_imm_lifecycle` 缺省开启）。战术决策、
   ECCM、对外点迹/航迹输出仍否决；检测判决不解释为对外
   "目标发现"事件，检测量测不出 public 面。
5. 波束控制：RIR 消费库内驻留调度器派生的波束指向，但不通过本模块 API 控制、
   生成或输出任何外部雷达波束。
6. 出口①的保真度升级（特征物理化：加噪/电磁散射链，RIR-OQ-1）、fusion 特征门
   mask-aware 升级、识别链模块内解耦——均非本次范围，各自为独立后续冻结项
   （登记见 `docs/review/rir_dual_product_stage_a_2026-08-18.md` §6）。
7. 地球椭球遮挡、地形通视、电波视距（4/3 k 因子）——几何门为圆球有限弦，k 只进大气损耗。

## 单位纪律

- `RirSceneTarget::rcs`（m²，探测链标量）与识别 RCS 特征/数据库（dBsm）显式区分，
  不得混用；数据库 units 表 `rcs == 'dBsm'`，声明其他单位即拒绝。
- 特征单位：速度 m/s、高度 m、加速度 m/s²、转弯半径 log10(m)、极化 dB、距离 m。

## ENU 帧约定

本节即「场景目标平台锚点 ENU 输入契约」（docs/common/contract.md）的库级范式：AR/EOS 的
场景目标输入已对齐本模块的 ENU 形态，公共一站式转换入口为
`oneq::coordinate::TryEcefToLla`（锚点）+ `TryMakeEnuSceneState`（逐目标）。

- 识别高度观测 = 平台绝对海拔 + 内部航迹 `position_z`；绝对海拔由必填平台 ECEF
  经 `TryEcefToLla` 库内派生；`position_z` 为雷达局部 ENU 切平面上向分量。
- 场景目标 `position_x/y/z` 同帧（公共 API 为 ENU；集成层以公共
  `TryMakeEnuSceneState` 完成 ECEF→ENU 后直填）；`range_m` 为斜距（>0 或带非零位置）。
- 视角样本网格（`aspect_az_deg`/`aspect_el_deg`）为雷达局部视线角；RCS 插值为
  最近邻（不强制覆盖），覆盖下限属数据库 profile 级适用条件，由匹配阶段判定。

## 双产品输出边界（出口①特征量测 + 出口②识别结论，2026-08-18 Stage B）

契约依据：`docs/common/contract.md` 规则 2 识别类传感器双产品条款；字段级冻结契约见
`docs/review/rir_dual_product_stage_a_2026-08-18.md` §3。

1. **出口②（识别结论，形态不变）**：`RirRecognitionResult` 逐航迹输出，装备使命
   产品；出口①上线不改变其任何字段与语义。
2. **出口①（特征量测帧，`RirOutputFrame.feature_measurements`）**：逐周期逐
   `association_key` 一条；只携带库内键（去真值化，规则 5——
   `external_target_id`/`target_name` 不得出口）；**仿真保真度语义 = 场景真值特征
   经效能约束（SNR/视角覆盖/带宽/驻留）转换的仿真量测，非加噪量测**——角度无噪声、
   RCS 均值无偏（std_db 为 SNR 推定不确定度），公共头 Doxygen 明示，保真度升级
   （特征物理化）为独立后续冻结项（RIR-OQ-1）。
3. **透出原则（裁定）**：出口①只透出识别链本周期实际构建且至少一维有效的观测
   （`RirObservationBuilder::Build` 后 mask ≠ 0）；积累质量门只挡积累不挡量测出口；
   无特征库（HoldCycle）、超识别最大距离、非识别模式的周期特征帧为**空**——
   不虚构。全维无效记录不产生。
4. **方位角参考系**：出口① `look_az_deg` 自 +x（东）起量（雷达局部 ENU），与
   fusion 自北约定不同——east→north 换算归 fusion 适配器，库内不做跨系转换。
   斜距+视线角+平台原点在适配器内还原 `has_position`（失败维持仅方位+原点）。
5. **平台位置输入**（`oneq::coordinate::EcefPositionM`，ECEF 米制，**必填**）：
   fail-closed——分量须有限且模长 > 0（地心非法，否则
   `rir.validation.invalid_platform_position` 整周期拒绝），且须可转换为合法 LLA。
   语义为场景 radar-local ENU 的绝对锚点（不改变场景目标 ENU 语义），透传到
   出口①记录（成功执行周期 `has_platform_position=true`），fusion 适配器换算
   LLA 填 sensor_origin，斜距有效时再填位置量测。`batch_id` 由 `RirSession` 内部自增分配（成功执行周期
   后递增），输出帧/特征量测/replay 仍暴露批号供 fusion 溯源。
6. **环境事实**（`RirEnvironmentConfig`）：会话初始化 + `RirRuntimeConfigPatch.has_environment`
   整域覆盖；**禁止**经 `RirCycleInput` 周期携带。`vegetation_cover_profile=kDisabled`
   且 `weather_attenuation_db=0`（默认）时传播/杂波退化到阶段 1 旧 SNR 口径。
   植被档非 `kDisabled` 时：传播损耗沿用 common 恒定基线（AR 同口径，损耗链
   物理化为后续冻结项）；杂波为模块内逐目标主瓣地杂波物理模型
   （`internal/RirSurfaceClutterModel`，σ₀ 档位表+杂波区几何+雷达方程，
   响应载频/斜距/擦地角），不再使用 common 杂波 stub。
   `atmospheric_physics`（气象观测，复用 `oneq::environment::AtmosphericObservation`）
   为场景事实输入：`enable_physical_model=true` 时驻留链路预算按每目标真实几何计算
   大气物理附加损耗（common 大气单源）；默认关闭零回归。k 因子为运行期派生量
   （`ResolveEffectiveKFactor`），不进配置。
7. **归属视图（`RirCycleResult.track_attributions`，仿真附件层）**：库内键 ↔ 场景
   真值目标对照（`external_target_id`/`target_name`）+ 最小航迹诊断
   （hit_count/滤波 ENU 位置/速度）；覆盖本周期全部航迹快照
   （tentative/confirmed/lost，与出口②同循环）；**不进 `RirOutputFrame` 产品层**
   （规则 15，与 SBIRS detection_attributions 同纪律）；非执行周期（校验
   拒绝/关机/中止）返回空列表且不推进状态。归属为航迹级（RIR 产品粒度即航迹级，
   不做逐检测级归属）。
8. **replay 加性扩展**：输出帧加特征量测向量、结果表加归属向量与
   `emission_frame`（V2 表加可选字段，`RIR2` 标识不变；旧记录缺新字段解码为空）。

## 驻留指向跨模块契约（库内驻留调度器：相对体积 + 转台朝向 + 指定任务限位）

波束指向的来源是**库内驻留调度器**（`RirSession` 每周期派生）：common 内核在**实际搜索
扇区** = `mission.scan_window_deg` ∩ `orientation.steerable_volume_deg`（阵面相对 az、
绝对 el）上建波位，再经 `mission.scan_center_deg` 平移并方位归一化；无指定任务时按该序列
推进；指定识别任务窗口内对准指定目标（目标在场景且在 `steerable_volume` 内，指定豁免
子窗）。RIR 消费侧只信任并消费给定波束中心。

1. **来源与所有权**：`RirSession::StepWithResult` 每周期解析驻留中心
   （`RirCycleResult::dwell_center_deg`）：
   - 无任务 / 任务间隙 / 越界回扫：绝对波位 = `NormalizeAzimuthDeg(scan_center.az +
     relative_wave.az)` + 绝对 el（相对体积 el 轴）；
   - 指定识别任务窗口内（`kPending`）且目标在场景且在体积内：驻留中心 = 目标视线角；
   - 非法体积/步长：扫描波位回退 `scan_center`（转台指向基准）。
2. **信任边界**：RIR 不判断给定指向是否朝向目标。方向图恒开（2026-08-29 还债：
   `enable_directional_pattern` 开关已删除，探测与波束指向耦合）：有有效目标视线角时，
   方位离轴差经 `NormalizeAzimuthDeltaDeg` 折算后求方向图增益；指向偏离目标就按实际
   离轴衰减执行，不静默修正。无有效视线角（位置范数 ≤ 0.1 m 的退化输入）回退主瓣
   峰值增益。配套主瓣覆盖门：扇区内候选须落在本周期驻留指向半功率宽内才入检测候选集
   （"探测⟺波束照到"，未照到发 `rir.target_outside_beam_coverage`）；确认航迹目标与
   指定目标在硬件体积内豁免扫描子窗（TAS 跟踪连续性，跟踪驻留指向为库内航迹预测）。
3. **角度含义与范围**：波束中心类型为 `config::RirAzimuthElevationDeg`（单位：度），
   定义在雷达局部 ENU 右手坐标系，与 `RirSceneTarget::position_x/y/z` 同帧：
   `az_deg = atan2(y, x)`，调度器输出前归一化到 `(-180, 180]`；
   `el_deg = atan2(z, sqrt(x²+y²))`，合法域 [-90, 90]。
4. **可扫描体积 / 任务子窗语义**：`steerable_volume_deg.az_*` 相对 `scan_center_deg.az`；
   `el_*` 为绝对俯仰域（ENU）。跨界扇区（如朝南 ±110°）通过 center 平移 + 归一化表达。
   `mission.scan_window_deg` 同口径（相对 az、绝对 el），是体积内的作战搜索扇区，缺省
   无界=不收窄；实际搜索扇区取二者逐轴交集，指定目标在体积内豁免子窗。
5. **目标视线角同帧**：RIR 计算目标视线角必须使用与指向角完全相同的坐标系和公式。

## 指定识别任务（限时锁定，镜像 AR designation 语义）

外部经 `RirRuntimeConfigPatch` 下达"识别目标 X"指令（`designated_external_target_id`
+ `designation_duration_cycles`，0 = 无限期），仅在 `work_mode == kIdentify` 时被
消费；`kStby` 下忽略。

1. **生命周期（会话级跨周期状态，镜像 AR 骨架）**：`kNone → kPending → kAcquired |
   kExpired`（kAcquired/kExpired 为终态）。窗口自指令生效后首个处理周期起算
   （deadline = 首周期 + duration）；任一指定相关 patch 变更（含仅改时长）视为
   新指令，窗口重新起算。
2. **任务窗口内（kPending）**：驻留中心对准指定目标（目标在场景且在可扫描体积内），
   识别积累照常进行；目标缺席时驻留回扫描波位并报告 `designation_reverted_to_scan =
   kNotRecognized`；目标在场景但视线越出 `scan_center + steerable_volume` 时回扫描并
   报告 `kOutsideSteerableVolume`（阶段保持 `kPending`，转台重新瞄准后可恢复对准）。
3. **识别达成（kAcquired）**：指定目标识别状态达 `kCategoryConfirmed`/`kModelConfirmed`
   （上一周期航迹快照口径，滞后一周期）→ **任务完成**：指定清零、回到扫描。
   与 AR 不同（AR 捕获后持续跟随），识别是离散结论，确认即任务结束。
4. **窗口耗尽未识别（kExpired）**：作废沿周期（cycle == deadline）报告
   `designation_revert_reason = kAcquisitionTimeout`（ID 保留）；其后指定清零、
   回到扫描，直到外部重新指定。
5. **结果暴露**：`RirCycleResult` 新增 `designated_target_id` / `designation_active` /
   `designation_reverted_to_scan` / `designation_revert_reason` / `dwell_center_deg`；
   replay 周期记录保留同名字段。

## 失败降级与状态机

1. 库未加载/版本不兼容 → `kDisabled`（不影响自持检测跟踪与其余能力）。
2. 分数/分差不足 → `kUnknown` 或仅大类；运动维度不能单独确认型号。
3. 内部航迹 lost 回收/无观测保持期 → 结论按 `result_hold_sec` 后置 `kStale`；
   关联键单调分配且不回收复用，键重分配天然等于新目标。
4. 退出 `kIdentify` → 清空积累，结论进入保持期；再次进入从零积累。
5. 关机 → 不触碰识别状态，结论在恢复后按保持期过期；校验拒绝不推进积累。
6. 数据库加载失败保持原库（路径变更时按需加载，失败仅降级并记录日志）。

## 传感器实例身份边界（SetSensorEntityId）

RIR 全面废止平台号字段与设备号字段，统一以 `SetSensorEntityId` 实体身份同号贯通标注与 RF 身份：

1. **注入面特征**：`RirSession::SetSensorEntityId(std::uint32_t)` 为会话构造后注入接口，不进入 `RirSessionConfig` 与 `RirRuntimeConfigPatch`，不参与配置重置。
2. **时序与缺省**：任意时刻可调，下一周期生效；未注入或显式传入 `0` 时恢复默认哨兵，周期摘要与融合标识回退使用融合常量 `fusion::kRirSourceId`（5U）。
3. **消费面（同号贯通标注与 RF 身份）**：有效值（非 0）同时供 RF 组装（发射身份 `identity.sensor_entity_id`、接收状态 `receiver_state.sensor_entity_id`、共址隔离表自路径归属、同平台自发射与自干扰剔除）与标注面（周期摘要日志 `[RirPipeline]` 携带 `sensor_entity_id={}`、验收行 `radar_id` 记录与下游融合通道标识）。完全废止 `RirSessionConfig::platform_id`。

[evidence: include/1q/remote_identification_radar/session/RirSession.h]::SetSensorEntityId
[evidence: tests/unit/remote_identification_radar/rir_projection_session_test.cpp]::SetSensorEntityIdContract

## 接口不变式

- 识别配置经 `RirRuntimeConfigPatch::has_policy` 整域提交（无叶子级 recognition
  patch 字段）；任务域经 `has_mission` 整域提交，`has_work_mode` 叶子可覆盖
  mission 内工作模式。
- 公共枚举加性扩展（不重排既有值，replay 字节兼容）；新增类别须同步
  `RirTracker::CategoryToPublic` 映射与 `RirRecognitionCategory`。
- replay 逐周期比较识别结果（浮点容差 `1e-5f`），`database_version` 与检测随机
  种子入 `RirSessionReplayState`，不一致即 failure；replay schema 为 V2 破坏性
  版本，旧 V1 记录显式拒绝。
- 识别配置校验：`rir.validation.recognition_*` 五码（权重/路径/门限/计数/时间范围）；
  其中任务域作用距离/驻留字段为 `mission.max_range_m`/`recognition_dwell_sec`（四域归位后）。

## 识别子模型的物理保真度边界（F1/F2）

识别链路在效能级观测之上引入两条**识别专用更高保真观测路径**，与自持检测链
物理口径**不逐项对账**（阶段 2-S 后本模块已有探测链，F1/F2 仍只服务于识别
维度有效性与质量）：

1. **F1 极化散射**（2026-09-03 四类判决重构）：场景目标 `polarization_samples`
   携带**四路复数窗口行**（`RirPolSMatrixSample`：HH/HV/VH/VV 幅度 dBsm+相位 deg，
   当前视线角 ±5° 裁剪；全量字典由场景层开机一次性载入，架构 B——类型手册归库、
   个体字典归场景）。识别链两路消费：双通道能量特征（HH/VV 幅度，经同一雷达方程
   与 SNR 噪声底派生，通道定义由数据库 meta `polarization_channels` 固定）与
   **五量统计特征**（|det S|/Span/去极化/Graves ψτ ×(均值, 标准差)；ψ 为圆统计，
   周期 180°）。细分类型（弹头/重诱饵/轻诱饵/碎片）判据未冻结，恒 kUnknown 不判决。
   - **证据**：[evidence: src/remote_identification_radar/recognition/PolarizationStatsExtractor.cpp]::RirPolarizationStatsExtractor
   - **证据**：[evidence: tests/unit/remote_identification_radar/rir_polarization_stats_test.cpp]::RirPolarizationStatsTest
2. **F2 距离像相干叠加**：仅消费场景侧 `range_rcs_scatterers` 真值列表；散射中心级
   峰值判定为效能级简化（粗距离单元下不合并峰标识，仅投影能量）。

上述两条仅存在于 `src/remote_identification_radar/recognition/`。

## 识别特征数据库契约（schema v1.1）

- 自描述只读基线：meta 必填六键、units 必填七量纲且 `rcs == 'dBsm'`。
- 权威 DDL 单源：`schemas/remote_identification_radar/recognition_feature_database.sql`，
  C++ 加载器、测试生成头（`recognition_feature_database_schema.h`）、建库工具
  （`tools/remote_identification_radar_db_builder.py`）共用；禁止第二份 DDL。
- 加载期只读读取器，运行期不持有 SQLite 连接。
- 版本策略：`schema_version` 为 `major.minor`；major 破坏性变更加载器拒绝。

## 验收信息日志（`RadarOutput.log`，2026-08-22）

CMake 开关 `ONEQ_ENABLE_RIR_ACCEPTANCE_LOG`（默认 OFF）门控。开启后按验收项写入
`RadarOutput.log`（四段同一行），性能项单独写入 `rir_acceptance.log`，三维方向图另写
`rir_antenna_pattern.csv`。
不进 `1q_library.log`，也不把融合/推演/精度抄进本文件。关闭时宏与派生计算一并
编译剪除，零开销、行为逐位不变。项表与「不能输出」字段见
`docs/review/acceptance_item_catalog_2026-08-22.md`。边界：仅人读验收材料，
不属于三写、不进公开输出/replay。交付日志一条原文指标一行，`[验收项：]` 用原文
指标名；内容只写数值或 `无`/`暂无`，不标注派生/未进/占位。公式与旁路边界见
`docs/review/rir_mti_mtd_acceptance_sidecar_freeze_2026-08-22.md`、
`docs/review/rir_polarization_l2_acceptance_sidecar_freeze_2026-08-22.md`；
剩余指标冻结记录已删档，见 git 历史。
缺交叉极化或 HH–VV 相位、无链路干扰通道写 `无`，不回退对角实矩阵、不均分冒充。

## 设计变更规则

1. 新增/删除/改变 public SPI 时，同步本文档集、consumer tests 与
   `check_public_api_boundary.cmake` 白名单。
2. 任何新增 runtime patch 字段，明确整域/叶子归属并接入提交语义。
3. 观测构造/提取/匹配/判定语义变化，必须同步 algorithms.md 与
   `tests/unit|integration/remote_identification_radar/` 对应测试。
4. 输出字段变化保持分层周期记录与可选投影分离（规则 15）；replay 表变更评估字节兼容。
5. 输入面/RF/环境字段变化须同步 `RirInputValidation` 新 issue code、契约白名单
   与场景测试；不再存在 AR 航迹供给契约。
6. 验证范围：`unit::remote_identification_radar`、`integration::remote_identification_radar`、
   `replay::remote_identification_radar`（AR↔RIR RF 物理链对账测试位于 unit 分区）。

[evidence: tests/unit/remote_identification_radar/]
[evidence: tests/integration/remote_identification_radar/]
[evidence: tests/unit/remote_identification_radar/rir_rf_physical_parity_test.cpp]
