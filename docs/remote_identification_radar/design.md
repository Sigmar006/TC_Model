---
Status: active
Last-reviewed: 2026-08-23
Authority: RIR 设计权威入口
Answers: 远程识别雷达模块是什么、和谁交互、设计文档怎么导航
---

# Remote Identification Radar 设计

`remote_identification_radar`（RIR）提供远程目标识别雷达的独立仿真模型：对自持检测-跟踪形成的内部航迹
执行驻留观测，经效能化观测（RCS/运动/双通道极化/宽带一维距离像）与预设特征数据库
加权匹配，输出目标类别与最可能型号，并携带周期识别效能摘要。

RIR 是与机载雷达（AR）**相互独立的另一部雷达装备**，不是 AR 的工作模式或子能力。
本模块由 AR 内被耦合的远程识别子系统（kLrr）解耦而来（2026-08-15 审计；迁移主体已完成，
余暂缓议题。两份审计记录已删档，见 git 历史）。

## 模块定位要点

1. **独立装备**：自带 hardware 域（发射机/天线/接收机/RCS 物理/信号处理增益），
   自持检测（方向图恒开+主瓣覆盖门/分项 SINR 账本/统计级 CFAR/TAS 跟踪驻留调度，
   2026-08-29 还债）、LAPJV 全局最优关联、CV KF/IMM
   双路径滤波与池化生命周期（跟踪升级 N1-N7 已落地）；
   独立输入输出与 Recording/Replay 落盘。与 AR 无任何模块间
   协作接口，不 include 任何 AR 头，不消费任何 AR 输出（阶段 2-S 已删除
   RirTrackFeed 供给面）。
2. **输入面**：`RirCycleInput` 提供周期戳、必填平台 ECEF（
   `oneq::coordinate::EcefPositionM`，fail-closed 校验）、场景目标
   （含识别特征真值 `aspect_rcs_samples`/`polarization_rcs_samples`/
   `range_rcs_scatterers`；公共 API 为 radar-local ENU，集成层用户侧以 ECEF
   描述目标，适配层边界转换）与可选外部 `rf_scene`；环境事实经
   `RirSessionConfig.environment` / 运行期补丁注入。场景目标速度/名称/Swerling
   起伏为自持链路事实；识别只消费效能化观测，场景真值不得直接产生结论。每周期
   波束中心由**库内驻留调度器**派生（相对可扫描体积 + 转台朝向 `scan_center_deg`
   平移归一化，或指定识别任务限位执行，见 boundaries.md 驻留指向契约），
   RIR 消费侧只信任并消费给定指向。
3. **输出面（双产品，2026-08-18 Stage B）**：出口②识别结论（`RirRecognitionResult`
   与效能摘要 `RirRecognitionCycleSummary`，形态不变）+ 出口①特征量测帧
   （`RirFeatureMeasurementRecord`：四维特征 + 逐维质量 + 有效掩码 + 库内键 +
   视线角/效能上下文 + 平台位置，语义=真值×效能约束转换的仿真量测）。
   归属视图（`RirTrackAttributionRecord`：库内键 ↔ 真值目标对照 + 最小航迹诊断）
   经 `RirCycleResult.track_attributions` 暴露（仿真附件层，不进产品层）。与 AR 威胁
   分类相互独立，不进任何决策帧；指定识别任务状态（`designated_target_id`/
   `designation_*`/`dwell_center_deg`）与**实际有效目标最大斜距**
   （`max_detected_slant_range_m`：本周期持航迹目标最大输入斜距，供外部判断实际探测距离，
   区别于 `mission.max_range_m` 径向粗筛门）经 `RirCycleResult` 逐周期暴露。
   fusion 侧由 `AdaptRirFeatureMeasurementsToDetectionRecords` 消费出口①。
4. **配置**：五域（hardware/orientation/mission/policy/environment）；policy 域承载
   检测/关联/跟踪/生命周期/识别策略，运行期补丁整域提交；识别作用距离/驻留
   四域归位（任务域）；`orientation.steerable_volume_deg` 承载阵面相对可扫描体积（硬件
   最大界限），`mission.scan_center_deg` 承载转台朝向（可补丁），`mission.scan_window_deg`
   承载用户指定的任务扫描子窗（作战搜索扇区，缺省无界；实际搜索扇区 = 子窗 ∩ 体积）；
   扫描步进/起点/顺序与指定识别任务（目标 ID + 限时窗口）随 mission/patch 配置，库内
   驻留调度器消费。
5. **数据**：特征数据库为只读 SQLite 基线（schema v1.1，权威 DDL 单源随迁），
   运行期不持有连接；单位纪律：场景 `rcs` 为 m²（SNR 门控），识别 RCS 特征与
   数据库一律 dBsm。
6. **命名**：public 前缀 `Rir*`；issue code 前缀 `rir.validation.*`；
   `ArRecognition*` 前缀废弃，不保留 compat 层。

## 文档导航

- 模块边界、非目标、单位纪律、失败降级、接口不变式、F1/F2 物理保真度边界、
  设计变更规则 → [boundaries.md](boundaries.md)
- 数据流图与状态所有权（数据库/积累/内部航迹/回滚）→
  [data-flow.md](data-flow.md)
- 对外交互接口与数据方向（与干扰机 / 外部参数设置系统 / 场景目标三组关系、
  单周期交互顺序）→ [../practice/rir_lrdr_external_interfaces.md](../practice/rir_lrdr_external_interfaces.md)
- 算法登记表（观测构造/四提取器/积累/匹配/判定）、反直觉点、刻意不实现清单 →
  [algorithms.md](algorithms.md)

跨模块公共规则见 `docs/common/contract.md`。

## 架构裁定与否决记录

1、**AR/ESR/ECM 放行全极化**（2026-09-03 否决）：Stage A 曾评估四模块全部放开
   `kFullPolarization`；裁定仅 RIR 的 `scene_polarization` 放行，其余模块冻结——配置不放行、
   加载器不加拼写、回放不接。ESR 会话校验 0..4 上限天然拒绝值 5；AR/ECM 无放行路径。
   解冻登记为开放议题 COMMON-OQ-11。
   - **证据**：[evidence: src/electronic_surveillance_radar/session/EsrSessionConfigValidation.cpp]
   - **证据**：[evidence: docs/common/rf_architecture.md]（配对规则与模块放行范围）
2、**dual_channel 能力字段化建模**（2026-09-03 否决）：提议不新增枚举值、另开双通道能力
   字段表达全极化；被否——四个回放 schema 的极化字段为 int 槽位，加字段破坏 schema，而枚举值
   零 schema 变更即可字节精确往返；且 `kUnpolarized` 已确立"极化工作状态"类别先例。
   - **证据**：[evidence: schemas/replay/rir_session_replay.fbs]::scene_polarization
   - **证据**：[evidence: tests/replay/remote_identification_radar/rir_replay_session_test.cpp]::FullPolarizationSessionConfigRoundtripsByteExact
3、**全极化对非极化 0 dB 功率守恒口径**（2026-09-03 否决）：物理上双通道功率相加可回收全部
   非极化功率（0 dB）；被否——"任一侧非极化固定 3.0103 dB"公理保持不动（保守、改动最小），
   守恒口径登记为开放议题 COMMON-OQ-12。
   - **证据**：[evidence: src/common/electromagnetics/RfLinkBudget.cpp]::TryPolarizationLoss
   - **证据**：[evidence: docs/common/open_questions.md]（COMMON-OQ-12）

4、**旧 dBsm 极化输入与新四路复数结构并存**（2026-09-03 否决）：Stage A 建议
   新旧并存保旧验收口径；用户裁定**拆旧**——旧 `RirPolarizationRcsSample`
   （3 功率+1 相位+has_* 开关）、旧回放样本表、旧场景键一并移除，回放按
   V1→V2 先例升 RIR3 显式拒绝旧录制。
   - **证据**：[evidence: include/1q/remote_identification_radar/session/RirSceneTypes.h]::RirPolSMatrixSample
   - **证据**：[evidence: schemas/replay/rir_replay.fbs]（root_type RirCycleReplayRecordV3 / "RIR3"）
5、**方向角 ψ 算术平均**（2026-09-03 否决）：ψ 周期 180°，+89° 与 −89° 物理同向，
   算术平均产生假 0°——冻结为圆统计（倍角向量平均，均值角+角度散布）；椭圆率 τ
   无缠绕（±45° 为两个不同物理态），维持算术平均。
   - **证据**：[evidence: src/remote_identification_radar/recognition/PolarizationStatsExtractor.cpp]::CircularMeanStdDeg
   - **证据**：[evidence: tests/unit/remote_identification_radar/rir_polarization_stats_test.cpp]::RirPolarizationStatsTest.CircularStatsWraparoundGoldenValues
