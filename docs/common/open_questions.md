---
Status: active
Authority: 非规定性记录（不构成契约约束）
Lifecycle: 条目有结论后回写 contract.md 或 design.md 并从本文删除；不保留已收敛条目
Last-reviewed: 2026-09-07
---

# 跨模块开放议题

登记调查中发现但尚未定论的跨模块架构议题。每条仅记录现状、后果、待决问题、当前边界与再进入条件，
**不构成**已批准的实现要求或契约规则。已定论条目必须迁出本文——契约规则进 `docs/common/contract.md`，
模块设计进对应 `design.md`。

- 何时读本文：评估某项反直觉/非阻断行为是否为已知边界、查某议题的再进入门槛、为审查中的
  Class D 发现（有用但未证实的能力）登记一条开放议题。
- 何时不读本文：查必须遵守的规则（去 `contract.md`）、查模块设计细节（去对应 `design.md`）、查工程实践
  （去 `docs/practice/`）。

## 议题索引

| ID | 域 | 主题 | 一句话 | Status |
|---|---|---|---|---|
| COMMON-OQ-1 | common | Windows 全链 CI 验收缺口 | 本机构建/交付已成治理事实；CI 仅 macOS、依赖引导无校验 | open |
| COMMON-OQ-8 | common | 周期时间/窗口静默拒绝 | 三类不同性质的拒绝门被并列；空帧 envelope 语义 AR/ESR 分裂 | open |
| COMMON-OQ-9 | common | RIR 补丁提交策略归类 | 暂存+下周期批量提交、无 resolver 校验、恒 true，不符现有二分类 | open |
| COMMON-OQ-10 | common | examples 会话配置三处同构维护 | 模板 + 12 场景 JSON + loader 键映射靠人工同步，字段变更漏一处即静默失配 | open |
| COMMON-OQ-11 | common | AR/ESR/ECM 全极化放行冻结 | kFullPolarization 仅 RIR 放行；共享计算层已支持值 5 | open |
| COMMON-OQ-12 | common | 全极化对非极化功率守恒口径 | 冻结保守 3.0103 dB；守恒 0 dB 口径待干扰/杂波定量验收触发 | open |
| SAR-OQ-1 | sar | RDA 性能数字是否重测 | /O2 实测数字来自旧档位，需确认是否随真发布档更新 | open |
| SAR-OQ-2 | sar | 脉冲环缓冲 O(1) 去重契约 | 依赖严格递增不变量，文档无 push 复杂度断言 | open |
| AR-OQ-2 | airborne_radar | 集成层 rf-world 干扰接线 | examples 从共享 rf-world 派生 interference，模块文档未述 | open |
| ESR-OQ-4 | electronic_surveillance_radar | 扫描采样点数上限截断 | 超 131072 点截断丢弃窗口末端，是否产品认可 | open |
| RIR-OQ-3 | remote_identification_radar | inconsistent_platform_position 疑似死码 | 头文件有、代码无产生点，是否删除 | open |
| RIR-OQ-4 | remote_identification_radar | 极化四类判决判据缺位 | 五统计量已量测，RV/HD/LD/DB 映射判据等甲方原件；场景字典为合成占位 | open |
| AR-OQ-1 | airborne_radar | 假目标鉴别跨域命名双轨 | 观测域枚举 vs 量测域 bool | open |
| ESR-OQ-1 | electronic_surveillance_radar | 压制干扰感知与 ECCM 链路缺失 | 死字段 + 无结构化观测 + 无 ECCM | open |
| ESR-OQ-2 | electronic_surveillance_radar | 运行时补丁扫描中心静默关边界 | scan center 补丁隐式切扫描模式 | open |
| ESR-OQ-3 | electronic_surveillance_radar | 扫描策略跨域耦合 | mission 值被 orientation mount 静默偏移 | open |
| SBIRS-OQ-1 | sbirs_sensor | 诊断距离的物理语义 | 仅 cue/诊断层，易被误读为测距输出 | open |
| SBIRS-OQ-2 | sbirs_sensor | 分阶段误差统计共享参数 | 三用途共享一组角度/距离统计 | open |
| SBIRS-OQ-3 | sbirs_sensor | 多目标随机样本与输入顺序 | 全局用途流，无 target 不变性 | open |
| SBIRS-OQ-4 | sbirs_sensor | Estimated 航迹真值初始化 | 首捕用真值位置/速度初始化滤波均值 | open |
| SBIRS-OQ-5 | sbirs_sensor | 俯仰中心手工配置与基准 nadir 化 | scan_center_el 仍需手填；非 GEO 轨道星下点俯仰漂移会配错且无告警 | open |
| SBIRS-OQ-6 | sbirs_sensor | 量测时间戳与扫描初始相位缺失 | 检出无时间字段、初始相位硬 0；扫描"穿越时刻"信息在源头丢弃 | open |
| FUSION-OQ-1 | fusion | 亚周期时间对齐缺失 | 融合按周期号驱动，无测量时间字段；跨周期对齐暂由评估层配对窗承担 | open |
| RIR-OQ-1 | remote_identification_radar | 特征量测保真度边界 | 真值×效能约束转换，非加噪量测 | open |
| TARGET-OQ-1 | target-layer | AR 估计/推演职责前置 | 消费级航迹产品 + 识别结论回填 public 输出 | open |
| TARGET-OQ-2 | target-layer | ESR 威胁等级进 public 输出 | threat_level 由传感器生产并被 ECM 下游消费 | open |

## Common 非阻塞边界

### COMMON-OQ-1：Windows 全链支持的 CI 验收缺口

- **现状**：Windows 构建与交付已是治理事实（2026-08 核验）：
  1. v141 preset（`VisualStudio.15.0-amd64`，Conan）为 Windows 本机构建主线（AGENTS.md/README 记载）。
  2. VS2015 no-Conan（`VisualStudio.14.0-amd64-none` / `1q_log_vs2015`）为交付层真实路径：
     运行手册 `docs/practice/windows_local_build.md`（零修改 consumer 镜像 + 零 C4819 +
     验收行为判据）。
  3. 依赖引导 `scripts/fetch_third_party.bat` 存在两个风险点：下载来源含 GitLab/archives.boost.io
     （非锁定 GitHub）；未对下载内容做 hash 校验。
  4. CI（ci.yml / nightly.yml）仍仅 macOS runner。
  [evidence: scripts/fetch_third_party.bat]
  [evidence: docs/practice/windows_local_build.md]
- **后果**：
  1. 本机交付流程已受治理，但依赖→configure→build→install→consumer 全链未在任何 Windows
     CI runner 上证明，回归靠人工。
  2. 无校验的下载链不可重复——同一脚本在不同时间/网络可能拉到不同内容。
- **待决问题**：是否为 Windows 引入 CI runner（或 nightly 一档）固化全链验收；
  `fetch_third_party.bat` 是否升级为锁定来源 + hash 校验矩阵。
- **当前边界**：Windows 支持按现有治理文档表述（v141 本机主线、VS2015 交付层）；
  不得宣称 Windows 全链有 CI 验收，也不得宣称依赖引导可重复。
- **再进入条件 (Stage A)**：Windows 交付出现一次因依赖漂移或环境差异导致的不可复现失败，
  或交付节奏要求自动化回归时，先提交锁定版本/哈希矩阵与 runner 方案，再决定 presets 与
  `.bat` 入口的保留形态。

### COMMON-OQ-8：周期输入时间/窗口字段无统一契约，违反时静默拒绝

- **现状**：原议题把三类不同性质的拒绝机制并列，经 Stage A 证据复核，性质不同不可统一：
  1. **AR 编年史门**：拒绝 `cycle_start_time_s < 上一周期窗口结束`，是单调推进校验。
  2. **AR+ESR envelope-equality 门**：非空 RF frame 的 envelope（`world_cycle_index`/`window_start_time_s`/`window_duration_s`）须与周期 input 精确相等。此判断已提取共享谓词 `RfFrameMatchesCycleWindow`（冻结于 contract.md §工程 RF 契约 条款 7 非空部分）。
  3. **EOS dt/frame-rate 门**：拒绝 `dt_sec > 10/frame_rate_hz`，是物理采样门（NEP/积分时间依赖），与 AR/ESR 时间门不同类。
  此外，RF 物理层 `TryResolveOverlap` 已对每条 emission 的 `activity_start_time_s + propagation_delay_s` 与 receiver 窗口做重叠判断，零重叠即零功率。
  [evidence: include/1q/electromagnetics/RfScene.h]
  [evidence: src/airborne_radar/session/ArInputValidation.cpp]
  RIR（2026-08-19 rf_scene-only 契约）为第三个消费方：空 `rf_scene` 豁免（同 AR）；非空须
  `TryValidateRfSceneFrame` 通过且窗口精确等于 `(sim_time_sec, recognition_dwell_sec)`。
  [evidence: src/remote_identification_radar/session/RirInputValidation.cpp]
- **后果**：调用方违反时整周期在决策消费点之前被拒绝，无显式错误可供察觉。但 Stage A 证明正常装配不触发：ECM 同步返回 `RfEmissionFrame` 并用同一周期权威时间盖戳，无事件总线、无延迟。触发的真实场景是调用方 bug（只递增 `cycle_index` 忘推进时间戳、缓存复用旧帧），对此显式拒绝是正确行为——若放宽为重叠判断，旧帧会过校验却被 RF 层判零功率静默消失，反而恶化静默问题。
- **待决问题**：空帧 envelope 语义是否统一——AR 豁免空帧（envelope 全无效也接受），ESR 不豁免（含空帧亦须填齐）。双方均有测试锁定，contract 条款 7 措辞只明确"身份或 mode"可豁免（emission 级），对 envelope（frame 级）未明确，无证据裁定优劣。
- **当前边界**：非空 envelope-equality 已提取共享谓词，两侧非空行为一致。空帧策略各模块保持现状：AR 豁免、ESR 严格。不得放宽任何时间门为重叠判断，不得宣称周期时间戳可任意重复。
- **再进入条件 (Stage A)**：出现真实 ESR 消费方因空帧 envelope 未填被静默拒绝的踩坑，或 contract 条款 7 envelope 含义需正式裁定时，先评估空帧豁免语义统一对 contract 条款 7 的修订成本。

### COMMON-OQ-9：RIR 会话运行时补丁提交策略归类的缺口

- **现状**：`session_contract.md` §运行期配置提交策略 将各模块归类为"事务性提交"或"立即提交"，
  二者均要求 patch 经 resolver 校验（`is_valid`/`has_requested_update`）。RIR（2026-08-20 并入六模块
  清单后）的 `RirSession::TryApplyRuntimeConfig` 为"暂存 + 下一成功周期边界整批提交"，且不做 resolver
  校验、恒返回 `true`——既不属于事务性提交，也不属于立即提交，与"patch 必须经 resolver 校验"的条款
  冲突。分类表现无 RIR 行。
  [evidence: src/remote_identification_radar/session/RirSession.cpp]
  [evidence: docs/common/session_contract.md]
- **后果**：会话契约的分类表缺 RIR 一行；RIR 补丁仅暂存不校验，无效补丁会静默排入下一成功周期，
  调用方无即时反馈。分类规则（"归属由状态空间决定"）未表述 RIR 这种"仅暂存、无校验、无失败路径"的形态。
- **待决问题**：RIR 补丁提交策略应归入哪一类——是否增设第三类"暂存提交"（仅暂存、无 resolver 校验、
  下周期边界整批落定），或将 RIR 收敛到现有某类（如为补丁引入 resolver 校验并入立即提交）。
- **当前边界**：RIR 按"暂存 + 下周期边界整批提交、恒 true"现状运行；契约分类表未登记 RIR，
  不得据契约宣称 RIR 属于现有两类之一。
- **再进入条件 (Stage A)**：会话契约下一轮修订（或 RIR 补丁路径新增真实校验/失败处理）时，
  先冻结 RIR 分类与校验契约，再更新 `session_contract.md` 与本文条目。

### COMMON-OQ-10：examples 会话配置三处同构维护（模板 / 场景 JSON / loader）

- **现状**：session_config"挂载即全量"口径落地后，同一份传感器配置结构存在三处同构拷贝，靠人工同步。
  1. `examples/basic_config/` 六域模板（batch_validation 夹具同源引用同一批文件）。
  2. 14 个场景 JSON 中的 12 份 `session_config` 子块（模板整份拷贝后按场景改值；
     两个 SBIRS 场景无该子块）。
  3. `examples/common/config_loaders/` 的叶子键映射（决定读哪些键、缺键回落库默认）。
  [evidence: examples/basic_config]
- **后果**：传感器配置字段增删改需要人工同步模板 + 12 份场景 JSON + loader 映射；
  漏一处要么静默失配（缺键回落库默认值），要么场景间隐性漂移；副本完备性目前只能靠人工审计确认。
- **待决问题**：是否将"场景自持全量副本"收敛为单源机制——运行期模板 ⊕ 场景覆写、
  脚本生成场景 JSON、CI 键树校验三者的取舍或组合。
- **当前边界**：保持 schema v2 挂载即全量口径与现有全量副本不变；不得在机制定论前
  引入"部分字段块"混用形态。
- **再进入条件 (Stage A)**：下一次传感器配置字段增删改需触及超过 5 份 JSON，
  或发现任一场景 `session_config` 键树与模板漂移。

### COMMON-OQ-11：AR/ESR/ECM 全极化放行冻结

- **现状**：`kFullPolarization`（2026-09-03 落地）仅 RIR 的 `scene_polarization` 放行；
  AR/ESR/ECM 冻结——配置不放行、加载器不加拼写、回放不接。公共纯函数层
  （两条枚举链的失配计算）已支持值 5，模块差异只在配置/加载器/回放放行面。
  [evidence: docs/common/rf_architecture.md]
  [evidence: src/electronic_surveillance_radar/session/EsrSessionConfigValidation.cpp]
- **后果**：三模块的仿真无法表达全极化收/发；程序化直接填值 5 时共享计算层会按全极化规则求解
  （ESR 经校验拒绝，AR/ECM 无校验拦截）。
- **待决问题**：某模块需要全极化时按何口径解冻（加载器拼写、校验上限、回放往返断言一次配齐）。
- **当前边界**：RIR 之外不得放开 `kFullPolarization`；解冻前不得为 AR/ESR/ECM 加放行代码。
- **再进入条件 (Stage A)**：AR/ESR/ECM 任一模块立项需要全极化工作模式。

### COMMON-OQ-12：全极化对非极化的功率守恒口径

- **现状**：全极化对非极化冻结为保守 3.0103 dB（"任一侧非极化固定 3.0103 dB"公理优先于
  全极化规则）；物理上双通道功率相加可回收全部非极化功率（0 dB）。
  [evidence: src/common/electromagnetics/RfLinkBudget.cpp]
  [evidence: tests/unit/common/common_rf_scene_test.cpp]
- **后果**：全极化接收对非极化干扰/杂波按半功率计入——对接收方偏乐观（漏计一半干扰功率）；
  对信号检测偏保守。
- **待决问题**：是否改按功率守恒 0 dB 口径，或按"信号/干扰分路"区分口径。
- **当前边界**：维持 3.0103 dB 单一口径；不得在一处改口径而不动另一条枚举链。
- **再进入条件 (Stage A)**：出现以非极化干扰/杂波为对象的定量验收或标定需求。

## Airborne Radar 非阻塞边界

### AR-OQ-2：集成层 rf-world 干扰接线是否补入模块文档

- **现状**：`examples/components` 集成层把 ECM 等发射设备发布的 `RfEmissionFrame` 汇入共享
  rf-world（`rf_world_broker.h`），再经 `BuildArInterferenceFromRfWorld` 组装 `ArCycleInput::interference`
  （排除自身平台发射设备）。AR 模块契约本身仍以 `ArCycleInput::interference` 接收 `RfEmissionFrame`，未变。
  [evidence: examples/components/ar_sensor_component.cpp]
- **后果**：AR 的四个模块设计文档均不引用 examples 层，该集成先例只存在于演示层；未来集成方从模块文档
  无法得知"干扰可从共享 rf-world 派生"的现成编排路径。
- **待决问题**：是否在 AR 的 data-flow.md（或 common/rf_architecture.md）补一段集成先例说明，把
  "共享 rf-world → AR interference"记为推荐编排方式；还是维持"模块文档只述模块契约、集成先例归 examples"。
- **当前边界**：模块文档不引用 examples 层；AR 干扰输入契约（`ArCycleInput::interference` =
  `RfEmissionFrame`，自发发射设备不计干扰）不受影响。
- **再进入条件 (Stage A)**：出现外部需求要求标准化 AR 干扰供给（或 ECM/RIR 同名先例增多）时，
  先在第一处编排语义（module contract vs examples）上裁定，再决定是否写入 rf_architecture.md。

### AR-OQ-1：假目标鉴别跨域命名双轨

- **现状**：反假目标鉴别判据（同方向多脉冲列）跨两个域表达：
  1. **接收机观测域**：`ArInterferenceObservation.deception_class`（枚举），判据天然归属于此。
  2. **航迹生命周期域**：`TrackLifecycleManager::PromoteState` 消费该标注，量测域以
     `RawTrackMeasurement.classified_as_false_target`（bool）表达。
  `SignalCycleInput` 周期输入端口已收敛（旁路 mutable setter 已删除），不构成公开契约。
  [evidence: include/1q/airborne_radar/session/ArInterferenceObservation.h]
- **后果**：
  1. 同一"疑似假目标"概念跨域用了两套命名（枚举 vs bool），跨域阅读增加认知负担。
  2. 内部 `ArDeceptionMeasurementCandidate` 不进入 public result 或 replay，公开观测仍是唯一持久化事实。
- **待决问题**：是否在跨域标注契约收敛时统一两套命名。
- **当前边界**：`SignalCycleInput` 是内部 `ISignalPipeline` API，不构成公开契约；内部
  `ArDeceptionMeasurementCandidate` 不进入 public result 或 replay。两套命名保留，待跨域标注契约收敛时统一。
- **再进入条件 (Stage A)**：跨域标注契约进入冻结时，一并评估命名的统一成本与对 trace/replay 的影响。

## Electronic Surveillance Radar 非阻塞边界

### ESR-OQ-1：压制干扰感知与 ECCM 决策链路缺失

- **现状**：ECM 输出 `RfEmissionFrame` 经 `EsrCycleInput.rf_emissions` 进入 ESR，物理层干扰功率计算已完成
  （`EsrResolutionCellLedger` 将非最强信号功率作为 `interference_power_w` 叠加到 SNR 分母）。但 ESR 存在
  两条未消费的配置链路和三处能力缺口：
  1. `InterceptSuppressionModelConfig` 填充到 `InterceptPipelineConfig.suppression_model`，但
     `InterceptDetectionExecutor` 从未读取。
  2. `EsrEnvironmentSnapshot.spectrum_occupancy_ratio` 注释声称参与噪声计算，但代码不存在。
  3. 无结构化干扰观测输出（对比 AR 的 `ArInterferenceObservation`）。
  4. 无 ECCM 决策引擎（对比 AR 的 `EccmEvaluator` 评分-提案-执行架构）。
  5. 无工作模式自适应切换。
  [evidence: src/electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.cpp]
- **后果**：
  1. `suppression_model` 和 `spectrum_occupancy_ratio` 为死字段，外部据配置名可能误以为 ESR 具备压制干扰
     感知或自适应抗干扰能力。
  2. 无结构化观测输出使下游消费方无法获得干扰态势感知。
- **待决问题**：
  1. 是否激活死字段消费，将压制干扰对等效噪声底的影响纳入 SNR 和检测门限计算。
  2. 是否定义 `EsrInterferenceObservation` 结构化输出（bearing、频谱、J/N、deception_class）。
  3. 是否实现 ESR 特有的 ECCM 决策措施（接收机重调谐、扫描优先级调整、检测门限自适应、工作模式降级、
     欺骗标记与置信度降级）。
- **当前边界**：ECM 压制干扰在 ESR 接收端仅以通用 RF emission 身份参与分辨单元竞争和 SNR 计算。不产生
  结构化干扰观测输出，不触发 ECCM 反制措施。`suppression_model` 和 `spectrum_occupancy_ratio` 为死字段，
  不得在文档中声称 ESR 具备压制干扰感知或自适应抗干扰能力。
- **再进入条件 (Stage A)**：
  1. 先激活死字段消费（修改 `InterceptDetectionExecutor` 噪声计算），并提供 unit test 证明
     `suppression_noise_scale` 和 `spectrum_occupancy_ratio` 的变化可被检测结果观测到。
  2. 定义 `EsrInterferenceObservation` 公开类型并提供 unit test 覆盖。
  3. 实现 ECCM 评分与提案机制，提供集成测试覆盖"ECM 发射 → ESR 感知 → ECCM 反制 → 检测效果变化"全链路。

### ESR-OQ-2：运行时补丁扫描中心静默关闭显式扫描边界

- **现状**：`EsrRuntimeConfigResolver` 在应用 `has_scan_center_az_deg` 或 `has_scan_center_el_deg` 补丁时，
  会同时设置 `use_explicit_scan_bounds = false`，静默将扫描模式从显式边界切换为中心驱动。三个事实：
  1. 用户仅调整扫描中心，不会预期丢失之前配置的四个扫描边界角。
  2. 单独补丁 azimuth center 也会导致 elevation 侧跟着切模式（副作用跨轴传播）。
  3. 显式边界的补丁路径已存在（`has_explicit_scan_bounds`，含 enabled 与四边界角），
     可显式开/关该模式——但 center 补丁的隐式关闭仍与之并存。
  [evidence: src/electronic_surveillance_radar/session/EsrRuntimeConfigResolver.cpp]
- **后果**：用户难以预判 scan center 补丁会清空显式边界配置且跨轴生效，配置结果与意图偏离。
- **待决问题**：
  1. center 补丁是否不再隐式关闭显式边界模式（模式切换只经 `has_explicit_scan_bounds`）。
  2. 或保留当前行为但增加返回值/日志提示。
- **当前边界**：当前行为为 scan center 补丁隐式关闭显式边界模式。消费方必须知晓此副作用。
- **再进入条件 (Stage A)**：出现真实场景要求"调整扫描中心但保留显式边界模式"，先评估移除
  隐式触发对既有补丁序列的兼容成本。

### ESR-OQ-4：扫描采样点数上限的截断策略

- **现状**：`ScanPatternGenerator` 单轴波位序列改为按整数步数计数采样（b869da22）后，
  `kMaxScanPointsPerAxis = 131072U`；超出上限时序列**截断保留前 131072 个点**（丢弃扫描窗口末端，
  不做加密重采样）。
  [evidence: src/electronic_surveillance_radar/pipeline/ScanPatternGenerator.h]
- **后果**：极小步进 + 大窗口时消费方会静默丢失窗口末端的波位覆盖，且无显式告警；文档已按现状
  如实描述截断语义（boundaries.md scan_rate_hz 节）。
- **待决问题**：上限值与"截断丢弃末端"是否为产品认可行为——是否需要显式告警，或改为对超限配置报错拒绝。
- **当前边界**：按现状截断（保留前 131072 点），文档如实描述；不得宣称窗口全覆盖，也不得称超限报错。
- **再进入条件 (Stage A)**：出现因截断导致覆盖缺失的验收/集成用例时，先评估告警或报错的 API 成本再决定。

### ESR-OQ-3：扫描策略跨域耦合（mission.scan + orientation mount 偏移）

- **现状**：`EsrScanPolicyConfig`（mission 域）的扫描字段经 `ApplyScanPolicy` 解算时会被 orientation 域偏移：
  1. `scan_center_az_deg` 减去 `EsrOrientationConfig::antenna_mount_az_deg`。
  2. `scan_start_az_deg` / `scan_end_az_deg` 在 `use_explicit_scan_bounds` 模式下也被 mount 偏移。
  mission 域的值被 orientation 域静默偏移。[evidence: src/electronic_surveillance_radar/session/EsrResolutionRules.cpp]
- **后果**：用户只看 mission 配置无法推断实际扫描方向；mount 偏移在内部解算时扣除但文档未明确，集成时易误配。
- **待决问题**：是否在公开 API 中明确扫描中心的坐标系语义，有三个备选：
  1. 定义为"天线坐标系"（已含 mount 偏移）。
  2. 定义为"平台坐标系"（需显式减去 mount）。
  3. 提供查询实际解算扫描几何的 API。
- **当前边界**：扫描配置语义为"天线坐标系"，mount 偏移在内部解算时扣除。文档未明确说明此语义。
- **再进入条件 (Stage A)**：出现因 mount 偏移导致的集成问题或用户误配，先明确公开 API 的坐标系语义并在
  design.md 中固化，再评估是否需要查询 API。

## SBIRS 非阻塞边界

### SBIRS-OQ-1：诊断距离的物理语义

- **现状**：`SbirsDetectionAttributionRecord.estimated_range_m` 明确只属于 cue/诊断层，不代表被动红外测距
  能力。三模式取值来源不同：
  1. Strict/Estimated 使用真值距离。
  2. Sensor-like 使用真值距离叠加比例误差。
  [evidence: include/1q/sbirs_sensor/session/SbirsOutputTypes.h]
- **后果**：字段名含 `estimated_range` 易被调用方误解为正式传感器测距输出，误用于滤波或定位。
- **待决问题**：字段名称和三模式取值来源是否足以防止调用方把它误解为正式传感器测距输出。
- **当前边界**：不得进入 `SbirsOutputFrame` raw output；消费方只能把它当作仿真归属与诊断辅助量。
- **再进入条件 (Stage A)**：出现真实下游消费者需要区分 truth-derived、filter-derived 或不可用距离，先盘点
  消费路径，再评估重命名、增加来源枚举或显式有效性字段。

### SBIRS-OQ-2：WFOV、Estimated 与 Sensor-like 的分阶段误差统计

- **现状**：WFOV 搜索、Estimated 校正量测、NFOV Sensor-like 输出三条用途使用独立随机子流，
  但共同读取 `SbirsErrorModelConfig` 的同一组角度/距离统计参数。单镜筒 NFOV 落地（2026-09-02）
  后，调度与指向扰动已有独立配置块（`SbirsSchedulerConfig`/`SbirsPointingDisturbanceConfig`），
  误差统计参数本身仍三用途共享。
  [evidence: include/1q/sbirs_sensor/config/SbirsPolicyConfig.h]
- **后果**：共享参数掩盖了三条链路在真实载荷上的精度差异，仿真结果可能高估某条链路的精度。
- **待决问题**：是否需要分别表达三用途的精度等级（WFOV 搜索、Estimated 校正量测、NFOV Sensor-like 输出）。
- **当前边界**：共享参数是当前确定性简化，不得宣称代表真实 WFOV/NFOV 载荷精度差异。
- **再进入条件 (Stage A)**：取得可追溯的分阶段参数依据，或构造出共享参数无法满足的 SBIRS 场景验收矩阵后，
  再评估拆分配置；不得仅为形式完整扩大 public API。

### SBIRS-OQ-3：多目标随机样本与 scene 输入顺序

- **现状**：WFOV、Estimated、Sensor-like 各自是一条全局用途随机流。两个确定性边界：
  1. 同一 trace 可确定性 replay（相同输入字节和顺序）。
  2. 多目标在同一周期获得哪个样本取决于 `scene` 遍历顺序。
  [evidence: src/sbirs_sensor/pipeline/SbirsPipeline.cpp]
- **后果**：目标列表置换后，同一 `target_id` 可能获得不同的量测随机序列，跨场景对比或批量验证时结果不可复现。
- **待决问题**：SBIRS 是否需要保证目标列表置换后，每个 `target_id` 仍获得相同的量测随机序列。
- **当前边界**：replay 只保证相同输入字节和顺序的确定性，不承诺 scene permutation invariance。
- **再进入条件 (Stage A)**：外部场景源无法稳定排序，或批量验证明确要求按 target 不受输入顺序影响时，比较
  按 target/channel 派生子流与现有全局用途子流的 snapshot、热更和目标生命周期成本。

### SBIRS-OQ-4：Estimated 航迹的真值初始化

- **现状**：Estimated 首捕初始化按后端分两轨：
  1. 默认 EKF/IMM 后端：首次捕获后用输入场景真值 ECEF 位置和速度初始化滤波均值，
     后续才使用带误差角度量测。
  2. 实验后端 kAngleCvKf（非默认）：4 维角度 CV 线性 KF，首捕仅用当前量测角度点迹
     初始化、变化率置 0，禁止真值三维。
  [evidence: src/sbirs_sensor/pipeline/SbirsTrackingCoordinator.h]
  [evidence: src/sbirs_sensor/tracking/SbirsAngleCvKalman.h]
- **后果**：默认链的 `Estimated` 描述为生产仿真链，但首捕阶段含 truth-seeded 简化，若被当作
  完全无真值辅助的真实载荷跟踪器使用，会高估其无辅助条件下的起始性能；无真值初始化路径
  目前仅实验后端可用。
- **待决问题**：kAngleCvKf 的纯量测初始化是否转正为默认航迹起始链；EKF/IMM 后端是否同样
  去真值化，或维持"真值种子 + 实验对照"格局。
- **当前边界**：默认 `Estimated`（EKF/IMM）仍包含 truth-seeded track initiation 简化，不得描述为完全
  无真值辅助的真实载荷跟踪器；kAngleCvKf 为实验后端，不作为验收默认口径。
- **再进入条件 (Stage A)**：转正评估时提供 kAngleCvKf 与 EKF/IMM 的捕获率、位置协方差、丢锁率及
  replay 对比证据，并定义角度-only（距离不可观测）时的初始化先验与失败判据。

### SBIRS-OQ-5：俯仰中心手工配置与基准 nadir 化

- **现状**：扫描方位已有双基准（kEciAbsolute 绝对 ECI / kNadirRelative 相对星下点偏移，2026-08-31），
  但俯仰中心 `scan_center_el_deg` 仍只能手工填绝对值；config 校验无"视场是否对着地球"的几何告警
  （方位基准裁决时方案 1 告警被 defer）。
  [evidence: include/1q/sbirs_sensor/config/SbirsMissionConfig.h]
- **后果**：GEO 赤道轨道下星下点俯仰恒 ≈0，手填 0 恰为正确值，无痛点；非 GEO（倾斜/低轨）轨道下
  星下点俯仰 = −asin(z/|r|) 随位置漂移（可达数十度），手填值会持续偏离且无告警，复现"方位抄数配错"
  的同型故障。
- **待决问题**：俯仰是否引入与方位同款的 nadir 基准（管线已有卫星 ECI 位置，公式现成）；是否一并补
  方案 1 的"扇区不含地球盘"启动告警（绝对模式无锚，收敛不适用，告警是唯一防护）。
- **当前边界**：nadir 模式跨度收敛的可见窗公式按 el=0（赤道）近似；行 el 远离星下点俯仰时窗宽偏宽
  （行内仍可能扫空天），属本条登记范围内的已知近似。
- **再进入条件 (Stage A)**：出现非 GEO 轨道的 SBIRS 集成需求，或 el 误配导致"看不到目标"的真实案例。

### SBIRS-OQ-6：量测时间戳与扫描初始相位缺失

- **现状**：检出记录无时间字段（只有 az/el/SNR/阶段），扫描相位每周期按 rate×dt 抽帧推进，
  波束"何时穿越目标"的信息在源头丢弃；多星初始扫描相位一律硬置 0（配置无相位字段），
  现实中各星独立开机、相位任意。
  [evidence: include/1q/sbirs_sensor/session/SbirsOutputTypes.h]
  [evidence: src/sbirs_sensor/pipeline/SbirsPipeline.cpp]
- **后果**：扫描模式下"哪一周期检出"是抽帧相位的人为产物；下游（融合/评估）拿不到测量时刻，
  只能以周期号为时间基准。当前由评估层配对窗（`dual_sat_pair_window_cycles`）在消费侧补偿，
  属权宜口径。
- **待决问题**：扫描模式是否输出"穿越时刻"量测（ping-pong 相位确定，穿越时刻可解析求出）；
  是否给配置加扫描初始相位字段以表达多星异步。
- **当前边界**：kSearchAndStare 下 WFOV 扫描检测无独立外发通道（量测产品被 NFOV 交接门控），
  时间戳补位若立项须一并考虑外发通道形态。
- **再进入条件 (Stage A)**：扫描模式进入正式验收且需要亚周期量测精度，或出现非 GEO/多星
  异步扫描集成需求。

## Fusion 非阻塞边界

### FUSION-OQ-1：亚周期时间对齐缺失

- **现状**：`DetectionRecord` 无时间字段，`FusionEngine::Update` 按周期号驱动；同周期多源
  量测被视为同一时刻。跨周期的时间对齐目前由评估层配对窗承担（精度评估模块，2026-08-31）。
  [evidence: include/1q/fusion/DetectionRecord.h]
  [evidence: include/1q/fusion/FusionEngine.h]
- **后果**：传感器若将来携带测量时间戳（见 SBIRS-OQ-6），融合滤波器无法利用其做
  "外推到测量时刻再更新"的时间对齐；现实地面融合"按时刻对齐零点几秒观测差"的环节在库内
  没有对应物。
- **待决问题**：`DetectionRecord` 是否加可选测量时间字段；逐航迹滤波是否支持按量测时刻
  推进（现有周期驱动 + coasting 是其自然推广）。
- **当前边界**：身份键跨源关联不受影响（双星量测经归属键进同一航迹已被场景证据覆盖）。
- **再进入条件 (Stage A)**：SBIRS-OQ-6 立项后随时间戳进融合，或出现对融合产品有亚周期
  时间精度要求的下游。

## SAR 非阻塞边界

### SAR-OQ-1：RDA 性能数字在新 Release 档位下是否重测

- **现状**：`docs/sar/algorithms.md` 性能注记记录 Windows 消费工程实测（1024×1024 孔径 / 20 点滤波器，
  RDA 总耗时约 0.29-0.39 s），数字来自 e1484476 时期 /O2 实测。bfb53867 后 MSVC Release 已为真发布档
  （/O2 /Ob2 /Oi），该数字仍代表 /O2 量级、语义未失效，但不是当前构建档位的重新实测。
  [evidence: docs/sar/algorithms.md]
- **后果**：若消费方按"当前构建档位权威数字"引用会有轻微偏差；但无需阻止任何使用。
- **待决问题**：是否在新 Release 档位重新实测并更新注记中的数字与修订日期，还是维持现值并在下一次
  SAR 性能变更时一并刷新。
- **当前边界**：文档按现有数字如实记录，并标注 2026-08-20 说明档位变化；不冒称是当前档位重新实测。
- **再进入条件 (Stage A)**：SAR 算法或构建档位下一次实质变化时，随变更重测并更新性能注记。

### SAR-OQ-2：脉冲环缓冲 O(1) 去重是否作为复杂度契约写入文档

- **现状**：脉冲环缓冲 `PulseRingBuffer` 的推入重复检查依赖"严格递增脉冲 ID"不变量
  实现 O(1) 拒绝（Push 拒绝 id <= back，RestoreRuntimeState 校验 IsStrictlyIncreasing）；
  现有文档（含 data-flow.md 状态所有权）对 push 复杂度无任何断言。
  [evidence: src/sar/runtime/PulseRingBuffer.cpp]
- **后果**：O(1) 去重依赖输入严格递增不变量，若未来调用方以乱序/重复脉冲输入，该复杂度契约不再成立；
  文档未记录该依赖。
- **待决问题**：是否在 data-flow.md（或 algorithms.md 登记表）把"脉冲 ID 严格递增、push 重复拒绝 O(1)"
  记为复杂度契约，还是维持现状（契约只存在于代码不变量中）。
- **当前边界**：代码按严格递增不变量 O(1) 去重；文档无 push 复杂度断言（按"只改错的"原则未新增）。
- **再进入条件 (Stage A)**：出现以非递增脉冲序列输入的消费方，或文档化复杂度契约的需求时，
  先冻结不变量边界再写入文档。

## Remote Identification Radar 非阻塞边界

### RIR-OQ-4：极化四类判决判据缺位（RV/重诱饵/轻诱饵/碎片）

- **现状**：极化散射矩阵统计特征（五量×均值/标准差）已量测进特征链与验收日志
  （2026-09-03 落地），`RirBallisticSubclass` 枚举与回放字段就位但无生产者——
  五统计量→四类的映射判据（阈值表/特征库维度/规则）依赖甲方表格与判据文本，
  原件未到手。新弹道场景字典为合成占位值（类间区分有、绝对数值无实测依据）。
  [evidence: src/remote_identification_radar/recognition/PolarizationStatsExtractor.cpp]
  [evidence: examples/scenes/rir_ballistic_discrimination/rir_ballistic_discrimination.md]
- **后果**：验收日志"目标类型"行无细分输出（恒仅粗类）；占位字典不能用于
  定量验收。
- **待决问题**：判据形式（甲方阈值表 / SQLite 特征库新维度 / 演示规则）与
  占位字典替换。
- **当前边界**：不得臆造判决阈值；细分字段恒 kUnknown。
- **再进入条件 (Stage A)**：甲方表格原件+判据文本到手。

### RIR-OQ-3：inconsistent_platform_position 疑似死码

- **现状**：`RirIssueCodes.h` 登记 `rir.validation.inconsistent_platform_position`（平台位置一致性校验），
  但 28ca2edf 后 `platform_position` 必填、`has_platform_position = false` 语义消失，全仓无该码产生点，
  @brief 亦陈旧。issue_codes.md 按"头文件为单一事实来源"如实收录。
  [evidence: include/1q/remote_identification_radar/session/RirIssueCodes.h]
- **后果**：issue 码表收录了代码永不产生的码；保留死码会让阅读者误以为存在平台位置一致性校验路径。
- **待决问题**：该常量与注释是否在代码侧删除（或修订 @brief 为预留），以及 issue_codes.md 是否同步
  移除对应行。
- **当前边界**：头文件为单一事实来源时该码被登记，但代码无产生路径、不产生该码。
- **再进入条件 (Stage A)**：RIR 校验层下一轮修订（增删平台一致性校验）时，先删除/修订死码及文档对应行，
  避免实质校验与登记再次脱节。

### RIR-OQ-1：特征量测的保真度边界（真值×效能约束，非加噪量测）

- **现状**：RIR 四维特征（RCS/运动/极化/距离像）由场景真值特征经效能约束转换产生：
  视线角无噪声；RCS 均值无偏、仅以 SNR 推定 `std_db = 3/√snr`；极化加 SNR 决定的
  确定性噪声底；距离像按噪声门删峰；笛卡尔位置是唯一被采样扰动的量测。2026-08-18
  双产品 Stage A 已裁定：出口①按此语义如实标注（公共字段注释明示"仿真保真度：真值×
  效能约束转换，非加噪量测"），物理化列为独立后续冻结项。
  [evidence: src/remote_identification_radar/recognition/RcsFeatureExtractor.cpp]
  [evidence: src/remote_identification_radar/recognition/PolarizationFeatureExtractor.cpp]
  [evidence: docs/review/rir_dual_product_stage_a_2026-08-18.md]
  [evidence: include/1q/remote_identification_radar/session/RirFeatureMeasurementTypes.h]
- **后果**：
  1. 统计类评估（蒙特卡洛识别率、融合收益）中特征不含测量随机性，结果不能代表真实
     量测分布下的性能。
  2. 下游若未读语义注释，会把特征当加噪量测使用（噪声/协方差口径错误）。
  3. 特征级多源融合的噪声建模无基础。
- **待决问题**：是否以及何时立"特征物理化"冻结项——各维噪声模型（RCS 幅度起伏与
  Swerling 模型的关系、极化通道噪声、距离像加噪/失真）与验收门。
- **当前边界**：出口①按"真值×效能约束"语义出口并在公共字段注释明示；不得宣称特征为
  统计真实量测；识别真值输入侧（角度-RCS 网格等）不变。
- **再进入条件 (Stage A)**：出现依赖特征统计分布的下游需求（识别率蒙特卡洛评估、特征级
  多源融合噪声建模）时，先冻结各维噪声模型与验收门，再实施。

## Target Layer 非阻塞边界

登记目标处理分层契约（contract.md §目标处理分层契约）生效前的存量偏离。处置完成前不要求
追溯回改，但不得新增同类偏离。

### TARGET-OQ-1：AR 估计/推演职责前置（航迹产品 + 识别结论回填）

- **现状**：AR 在传感器内持有完整估计层三件套——LAPJV 关联（`signal/association`）、KF/IMM 滤波
  （common/estimation 的向后兼容外观）、`TrackLifecycleManager` 生命周期——并以
  `TrackOutputFrame` / `ArExternalTrackOutputFrame` 对 fusion 发布消费级航迹产品（滤波后 ECEF
  运动学 + 协方差迹）；同时保留决策层启发式识别 `ThreatAssessmentEvaluator::IdentifyTarget`
  （速度/RCS 最近邻 + `FeatureRepository`），其结论经 `ArController` 回填 public
  `TrackStateSnapshot.target_type/target_probability`，内部威胁分驱动 LPI/ECCM。注意：kLrr
  识别子系统已按 2026-08-15 耦合审计全量迁出（迁移 2-C `1ac346ca`），本条指的是**保留的决策层
  启发式**，与 kLrr 迁移无关。
  [evidence: include/1q/airborne_radar/session/TrackStateSnapshot.h]
  [evidence: src/airborne_radar/signal/association/DataAssociation.h]
  [evidence: src/airborne_radar/signal/tracking/TrackLifecycleManager.h]
  [evidence: src/airborne_radar/decision/ThreatAssessmentEvaluator.cpp]
- **后果**：同一目标在 AR 内（LAPJV）与 fusion 内（按 association_key 再关联）被关联两次；
  传感器 public 输出同时承载估计产品（滤波航迹）与推演结论（识别标签），下游
  threat_assessment 的类型概率输入实际消费传感器回填的识别结论；估计层轨迹滤波立项时存在
  职责重叠。
- **待决问题**：估计层轨迹滤波立项时 AR 航迹帧的语义定位（AR 内部信号处理产品，还是估计层
  在传感器内的前置实现）；识别结论回填 public 输出是否退出，改由推演层识别面供给。
- **当前边界**：AR 航迹输出是冻结公共 API（fusion SensorAdapters 消费），不因分层契约追溯
  回改；AR 内部威胁分仅驱动 LPI/ECCM 资源管理，不外发威胁产品；`target_type`/
  `target_probability` 维持现状直至推演层识别面立项。仿真真值归属（`external_target_id`/
  `target_name`）已于 2026-08-21 补齐信封对照表 `ArCycleResult.track_attributions`（权威
  路径，见 session_contract.md Attribution 挂载表与 docs/review/ar_track_attribution_2026-08-21.md），
  产品字段降级为 deprecated 遗留（sim-only）；字段回收（去真值化收回）仍归本条 Stage A 处理。
- **再进入条件 (Stage A)**：估计层轨迹滤波（fusion 演进）立项时，按证据优先模式提交 AR↔估计层
  职责划分方案（关联单源化、滤波原语单源化、识别结论出口迁移）与 replay/公共 API 迁移契约；
  不得零碎单独修改。

### TARGET-OQ-2：ESR 威胁等级进 public 输出（ECM 下游消费）

- **现状**：`EmitterHypothesis.threat_level`（public DTO 字段）由
  `HypothesisAssociator::InferThreatFromCluster`（模式+SNR 启发式）计算并对外发布；ECM 侧
  `EcmEsrAdapter` 消费该字段二次计分为调度排序输入。同一关联器内的 `InferModeFromCluster`
  （PRI/脉宽→工作模式）与假设关联/生命周期（最小费用流指派 + 指数混合平滑）也在传感器内。
  [evidence: include/1q/electronic_surveillance_radar/session/EmitterHypothesis.h]
  [evidence: src/electronic_surveillance_radar/pipeline/HypothesisAssociator.cpp]
  [evidence: src/electronic_countermeasure/EcmEsrAdapter.cpp]
- **后果**：决策层产品（威胁等级）由传感器生产并进入 public 输出，违反分层契约规则 2；ECM
  调度输入依赖传感器的越层字段，形成越层消费链。模式推断与假设平滑的归属存在语义争议（可
  辩护为 ESM 量测语义标注 / ESM 产品形态）。
- **待决问题**：`threat_level` 是否从 public DTO 退出（破坏性公共 API 变更，需冻结迁移契约；
  ECM 调度输入改由 threat_assessment 或调用方供给）；ESM 假设管理（关联/生命周期/平滑）是否
  作为合法 ESM 产品形态保留。
- **当前边界**：`threat_level` 为冻结公共 API，不追溯回改；处置前 ESM 假设管理保留现状，
  不得新增同类威胁语义字段。
- **再进入条件 (Stage A)**：ESR 公共 API 修订立项时，提交 `threat_level` 迁移契约（含 ECM
  消费路径改造与 replay 兼容性裁定）。
