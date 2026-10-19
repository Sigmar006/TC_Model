# 下一步工作计划（2026-10-19 晚起草）

Status: proposal（待批准后按 P0→P5 推进）
依据：`docs/review/session_handoff_2026-10-19.md`、`docs/review/acceptance_log_physical_audit_2026-10-19.md`、
`docs/review/acceptance_log_fix_plan_2026-10-19.md`、`docs/practice/integration_boundary_1q_vs_host_2026-10-19.md`、
`docs/practice/fosimstudio_host_and_integration_guide.md`，以及 2026-10-19 18:55–19:05 对两个仓库 / 部署目录 / 配置的逐项核对。
原则沿用修改方案：**先让读数成为证据（A 类）→ 再改物理量级（B/C 类）→ 最后重标定门限**；每轮只改一个变量。

---

## 0. 核对结论：交接文档需要修正的 5 点

| # | 交接文档说法 | 本次核对（证据） | 影响 |
|---|---|---|---|
| 1 | §3.1「A1 只差角速率派生」，§2.2 认为日志改动未提交 | 日志改动已随 `1df6f724` 入库（`SbirsPipeline.cpp:2430-2448`）。但 `has_angle_rate=true` **只在 `kAngleCvKf` 分支**（`SbirsTrackingCoordinator.cpp:185-187`），EKF/IMM 分支不置位；`SbirsPipeline.cpp:2431-2432` 注释声称「kEkf/kImm 由后验视线角跨拍差分派生（见 `SbirsTrackingCoordinator::CorrectTarget`）」——**代码里没有这段**。 | 当前 backend=`kEkf` ⇒ 第 77 项 `角速度` 将**永久**输出 `不适用(首拍无历史)`，且理由与真实原因不符。A1 实质未完成，且注释必须先改真话。 |
| 2 | §2.4 部署状态「含验收字段补齐」 | v141 测试 exe 18:48、v120 `1q.lib` 09:35、`ModelSource\lib\...\1q.lib` 15:17、`Sensors.dll`/`DataProcessors.dll` 15:26；而 A2/A3/A4 与 A1 日志改动是 **13:xx–18:52** 才进源码/提交。审计轮运行时间 16:22–16:26。 | **16:22 那轮日志不包含今天的 A 类改动**，A 类修复目前**没有任何运行证据**。必须先「重编 v120 库 → 重编宿主 → 重跑」再判读。 |
| 3 | §6 表只列"当前配置值" | 部署副本与**源副本已漂移**（新发现，见 §0.1） | BJ 仓库是交付仓库、提交的是**源副本** ⇒ 换机/重装/他人复现时，今天全部调参**静默回退**。 |
| 4 | §4 第 10 项「应用 `cue_max_age_cycles`」 | 1q **已有**外部跟踪保持机制（`RirExternalTrackPhase::kCoasting/kExpired`、`RirExternalTrackState`）。真正缺陷有两处：① `RirTrackRequest.h:79` 承诺"`valid_until_cycle_index=0` ⇒ 库内按速度与位置误差**推导保持期**"，但 `RirSession.cpp:463-465` 只在**非 0** 时判到期 ⇒ **0 = 永不到期**，承诺未实现；② `RirTrackRequest.h:95` 的 `extrapolated_age_sec` **全仓库从未被赋值**（`GetExternalTrackState` 只填 3 个字段）。 | 宿主之所以在第一个无引导周期就 `ClearExternalTrack()`，是因为库内没有"到期"概念、不清就会永久保持。修法应是**库内补推导保持期 + 宿主改看 phase**，不是加一个宿主侧超时参数。 |
| 5 | §2.2 未提交改动清单 | 5 个改动 + 3 个未跟踪文件仍在（`code_line_statistics_split/.../1q.code-workspace` 明显误入）。其中 `docs/README.md` 的改动引用的 `docs/review/evaluation_data_foundation_tables_2026-10.md` 目前**未跟踪** ⇒ 提交 README 会留下死链。 | 需先分流，否则每轮 diff 都掺噪声。 |

其它已复核基线：1q 仓库分支 `fosim_2013`、HEAD `1df6f724`；`1q_sbirs_sensor_unit_tests.exe` 实跑 **281/281 PASS**；
部署配置实测 `background_radiance=1.2`、`wide/narrow_min_snr_linear=0.001`、`process_noise_diff_coeff=20`、
`orbit/attitude/fov_sigma=5e-5/1e-4/5e-5`、`readout_noise_rms_w=0.0`、`estimated_backend=kEkf`、RIR `peak_power_w=4.8e6`、
`accumulation_window_sec=60`、`result_hold_sec=120`。

### 0.1 配置源副本 ↔ 部署副本漂移（本轮新增证据）

| 文件 | 部署副本（生效） | 源副本（BJ 仓库提交对象） | 差异 |
|---|---|---|---|
| `sbirs.json` | bg **1.2** / σ 5e-5·1e-4·5e-5 / pn **20.0** | bg **0.0** / σ **0** / pn **1.0** | 5 项（含 `_tuning_note`） |
| `remote_identification_radar.json` | 4.8e6 / 60 / 120 / `scan_center` 40° / 窗口 az110–130 el36–48 | 582000 / 10 / 30 / `scan_center` 0° / 窗口 az113–120 el0–12 | 6 项 |
| `remote_identification_radar_target_features.json` | 全周向 171 行 RCS 网格 | 旧版 | **1032 行** |

> 部署副本路径：`C:\software\workspace\1qfosim\FOSimStudio\bin\App\config\`；
> 源副本路径：`BJ_10Y_WorkSpace\ModelSource\CyModel\Equipment\Sensors\{OPIR_SENSOR,LRDR_SENSOR}\`。
> 该风险在 `fosimstudio_host_and_integration_guide.md` §2.2 / §7 坑 7 已有提示，但缺一个可执行的同步/自检动作。

---

## 1. 排期总览

| 相 | 内容 | 改在哪 | 风险 | 依赖 |
|---|---|---|---|---|
| **P0** | 配置源副本同步 + 工作树分流 + 重部署重跑 | BJ / 文档 | 零（不动物理） | — |
| **P1** | A 类收口：A1 派生、A2/A3/A4、B9/B8 日志口径 | 1q | 低（只动日志与派生量） | P0.3 基线 |
| **P2** | 噪声链 B1–B3 + 辐射强度 C1 | 1q + 宿主 + 配置 | **高**（改变 SNR 量级） | P1 |
| **P3** | 门限按 Pfa/Pd 重标定 B4 | 配置/模型 | 中 | **必须紧跟 P2** |
| **P4** | 跟踪/滤波：B5 诊断 → B6/B7 | 1q | 中 | P2/P3（用新 SNR 判读） |
| **P5** | 引导连续性（D1 库内保持期 + D2 宿主按 phase） | 1q + 宿主 | 中 | P4（越界与滑行先归零） |
| **P6** | 集成迁移准备（4 类漂移 + 4 个 API 缺口） | 集成侧 + 1q | 低 | 可与 P2/P5 并行 |

---

## 2. P0：先把仓库与部署摆平（今天可完成）

### P0.1 配置源副本同步 + 单一真相动作
1. 把三个文件的**部署副本**复制回**源副本**（`sbirs.json`、`remote_identification_radar.json`、
   `remote_identification_radar_target_features.json`）；`target_feature_database_v1.1.db` 无源副本，只在部署侧，需在指南里写明"库只在 `bin\App\config`，必须单独备份/提交到 LFS 或发布包"。
2. 新增一次性自检（放 BJ 仓库 `ModelSource\sync_config_check.cmd` 或 1q `tools/`，二选一，**先定归属再写**），
   行为：对三对文件做逐字比较，不一致时打印 diff 摘要并以非 0 退出。
3. 指南补一条：改配置 = 改**源副本** → 跑自检 → 同步到 `bin\App\config`；`sbirs.json` 与 RIR 库**必须在宿主重启后**才生效。

**验收**：三对 `Compare-Object` 为空；BJ 仓库一笔提交（只含三份配置）；指南有可复制的一行命令。

### P0.2 工作树分流（1q）
| 文件 | 建议动作 |
|---|---|
| `docs/README.md` + `docs/review/evaluation_data_foundation_tables_2026-10.md` | **一起提交**（先 `git add` 未跟踪文档，否则死链） |
| `tests/unit/remote_identification_radar/rir_designation_task_test.cpp`（+26） | 提交（随文档提交或单独一笔，视测试是否通过） |
| `tools/db_probe/db_probe.cpp`（仅补 BOM） | 提交（BOM 是硬要求，属修正噪声） |
| `.vscode/settings.json` | **回退**（个人编辑器设置，不入库） |
| `code_line_statistics_split/lrdr_radar/.../1q.code-workspace` | **删除**（误入产物） |
| 未跟踪的 `tests/integration/.../rir_dual_satellite_lock_test.cpp` | 先看是否能编过；能过则随 P5 一起提交，不能过则删 |

### P0.3 重建 + 重部署 + 跑一轮（A 类的唯一判据来源）

按 `session_handoff_2026-10-19.md` §8 执行：**BOM 转换 → v141 编 `sbirs_engine` + 聚合库 `1q_lib` + 测试 → 281 项回归
→ v120 编 `1q.lib` → 复制到 `ModelSource\lib\x64\Release\` → 重编 `Sensors.dll`/`DataProcessors.dll` → 关宿主 → 重跑想定**。
> ⚠ 聚合库 `1q_lib` 必须重建（测试链接的是它）；强杀编译留下的 0 字节 obj 会报 `LNK1136`，先扫一遍删掉。

跑完先填这张**基线表**（后续每轮只允许与它有一处差异）：

| 指标 | 本轮值 | 备注 |
|---|---|---|
| 第 77 项 `角速度` | 期望 `不适用`，**不得**出现 `(0.000000,0.000000)` | A1 日志部分 |
| 第 77 项 `角速度` 行数 / 总行数 | | 100% 不适用即说明派生仍未做 |
| 第 67 项 `观测性质/检测标志` | | A2 后字段名应变化 |
| 第 76 项 `连续命中` 分布 | | 期望出现 >1 |
| 第 76 项 `剩余角` 为 0 的占比 | 上轮 91.86% | B8 后应显著下降 |
| 第 77 项 `帧数=0` 且判"跟踪"行数 | 上轮 326 | B7 后应为 0 |
| 四分量噪声 / 有效噪声能量 / SNR 区间 | 上轮 1.451e-15 / 6.647e-11 / 0 / −31~−19 dB | P2 目标 |
| 门限与 SNR 门通过率 | 0.001 / 99.24% | P3 目标 |
| NIS p50/p99/max 与"超门"行占比 | 1 / 9.62 / 2.55e9 / 26,990 行（96.8% 不自洽） | P4 目标 |
| 第 65 项 Up>2293 km 行占比 | 59.02% | P4/P5 目标 |
| 第 47A 项 空输出占比 / 置信度上限 | 98.43% / 0.1164 | P5 目标 |

---

## 3. P1：A 类收口（只改日志与派生量，零物理影响）

### P1.1 A1 — 角速率派生（推荐"解析法"，不引入历史状态）

| 方案 | 做法 | 评价 |
|---|---|---|
| ① 协调器加 `std::map` 历史 + `last_dt` | 交接 §3.1 曾试 | ❌ 已实测堆损坏 `0xC0000374`，**不再尝试** |
| ② pipeline 内加历史 map 做差分 | 交接推荐 | ⚠ 可行但引入逐目标历史状态，且需处理重捕/清零 |
| ③ **解析法（推荐）** | 用**已有的** EKF/IMM 后验状态解析求角速率，不加成员 | ✅ 无需历史、无布局风险、与滤波器自洽 |

方案 ③ 细节：
- 后验状态取 `combined`：`r_est=(mean(0),mean(2),mean(4))`、`v_est=(mean(1),mean(3),mean(5))`（ECI）；
  卫星位置/速度：`CorrectTarget` 已收 `satellite_position_eci_m`，**再补一个 `satellite_velocity_eci_m` 入参**
  （pipeline `SbirsPipeline.cpp:957` 已有现成局部量 `satellite_velocity_eci_m`）。
- 公式（与交接 §3.1 一致，但用状态而非差分）：
  ```
  r_rel = r_est − r_sat,  v_rel = v_est − v_sat
  ω_az = (x·v_y − y·v_x)/(x²+y²)
  ρ = √(x²+y²)
  ω_el = (ρ·v_z − z·(x·v_x + y·v_y)/ρ)/(ρ²+z²)
  ```
- 置位：EKF/IMM 分支在填 `output_*` 的同时填 `has_angle_rate/azimuth_rate_rad_per_s/elevation_rate_rad_per_s`；
  `kAngleCvKf` 分支保持原值**不覆盖**。
- 日志：把 `SbirsPipeline.cpp:2431-2432` 的假注释改为「EKF/IMM 由后验状态解析派生」，`else` 分支理由改为
  `不适用(无滤波后验)`；`kAngleCvKf` 首拍仍为 `不适用(首拍无历史)`。
- 交叉校验：同一行同时打 `角速度` 与 `视线角速率`，两者应同量级（实测 0.0039–0.0134 deg/s）、同符号；
  差异大说明后端状态发散，是 P4 的输入证据。
- 测试（`tests/unit/sbirs_sensor/`）：①静止目标 + GEO 卫星 ⇒ 派生 ω ≈ 卫星运动引起的相对角速率（解析值对照 ±1%）；
  ②卫星速度置零后同一场景 ω 偏差 >50%（证明**确实减掉了卫星速度**，这条正反例最关键）；
  ③无后验（首拍）时 `has_angle_rate=false`。

### P1.2 A2/A3/A4（原样推进，附**实测现状**）
| # | 现状核对 | 动作 |
|---|---|---|
| A2 | `SbirsAcceptanceRecords.cpp:578` 仍输出 `检测标志=YesNo(...)` | 改为 `观测性质=检测`（去掉二值语义）；报告写明"仅检测行入日志" |
| A3 | 全库无 `候选数` 字样（未做） | 捕获行补 `候选数=N 通过=M`，使"从未失败"可与"没有拒绝逻辑"区分 |
| A4 | 全库无极化诊断标记（未做） | 先在 `PolarizationFeatureExtractor` 打印参与计算的 hh/hv/vh/vv 幅相，**确认相位是否被丢弃**再改公式；三项特征必须随目标/周期变化 |

### P1.3 B9 / B8（与 A 类同批，因为都只是口径）
- B9 `连续命中`：`SbirsPipeline.cpp:2004` 打印 `wfov_consecutive_hits_[id]`，但审计见恒 1 ⇒ 先确认
  L1745 附近的累加点是否在**每周期**执行、是否被 L785/2201 的重置误覆盖；修好后应出现 1,2,3… 递增，丢检归零。
- B8 `剩余角`：`SbirsPipeline.cpp:2105` 打印的量应为 `|实际指向 − 指令指向|`；同时让 `候选状态=已捕获跟踪`
  引用同一量（当前残差 149.85° 仍报已捕获）。

**P1 验收**：重跑一轮，逐条对比基线表；`角速度` 不再是空壳；恒定/同义反复字段清零；281 项回归全绿；
1q 仓库一笔提交（日志 + 派生），BJ 不需要改动。

---

## 4. P2：噪声链与辐射强度（本方案核心，先改模型再谈门限）

| # | 改动 | 文件 |
|---|---|---|
| B1 | 背景光子功率乘 `τ_atm`：`ComputeBackgroundNoiseStatistics` **加 `atmospheric_transmittance` 入参**；调用点两处 `SbirsPipeline.cpp:633`、`1232` | `src/sbirs_sensor/foundation/SbirsNoiseModel.{h,cpp}` |
| B2 | 换掉 `sqrt(4·k_B·T·t_int)` 代理式：NEP 主导（`NEP·t_int`），热/暗电流项按 `sqrt(2·q·I_dark·t_int)` 或 NEP 折算；`detector_temperature_k` 退回**诊断量** | 同上 |
| B3 | `readout_noise_rms_w: 0.0 → 1.0e-14`（与 NEP 同量级） | `bin\App\config\sbirs.json`（**改源副本后同步**，见 P0.1） |
| C1 | 辐射强度：宿主恢复 `cyairportData.cpp:58` / `CyQPEGVehicleData.cpp:58` 的 `irar_`，并按类别查表（助推 1e5–1e6 / 中段 RV ~1 / 地面站 1e4，表放 `sbirs.json`）；1q 侧保留常量回退但**在代码注释与文档标注非物理** | 宿主 + `sbirs.json` |

**验证**（同帧打印四分量）：① `光子噪声` 随 `大气透过率`（0.0996–1.0）变化 ≥3 倍；② 四分量无一项独占 >20 dB；
③ `有效噪声能量 = √Σ²` 自洽；④ 第 75 项 `判决模块输入SNR` 回到 O(10 dB)。

**决策点（P2 完成后立即做）**：按新 SNR 判定 **4.8 MW 是否回退 582 kW**。理由：当初提功率是为了让 −28 dB 通过，
前提被 B2 推翻；既修噪声又提功率属**重复计数**。回退后 RIR 检测距离应从 5,038 km 回落到 ~1,970 km 量级，
需在场景里确认仍覆盖 ICBM 中段。

---

## 5. P3：门限重标定（紧跟 P2，不可提前）

- `wide/narrow_min_snr_linear` 由 Pfa/Pd 反推：`SNR_min = σ·(Q⁻¹(Pfa) − Q⁻¹(Pd))`；WFOV 搜索与 NFOV 精跟取不同
  Pfa（如 1e-5 / 1e-8），**门限不再相等**。
- 门限值写入日志（可复核），而不是只留在 JSON。
- **验收**：SNR 门通过率落在 20%–80%；`SNR门结果` 与比较式 0 处不一致；WFOV ≠ NFOV。

---

## 6. P4：跟踪 / 滤波（每项单独验证）

| # | 动作 | 关键点 |
|---|---|---|
| B5 | **先加诊断**：每次判决打印 `参与判决NIS / 门限(5.991) / 模型数 / gate来源(ekf\|imm\|other) / 量测来源(真值\|滤波预测)` | 当前 26,990 行"超门"里 96.8% 其 NIS 未超门；22 行 `NIS=0` 的量测与预测仅差 1.5e-5° ⇒ 疑似某分支把预测当量测。**定位前不要改 5.99** |
| B6 | RIR 包络门：离地高度或 \|r\| 超上限 ⇒ 直接置 `lost`，不得以确认航迹输出 | 65 项 59% 越界（max 7,424 km）；字段在 `RirAcceptanceRecords.cpp:1488`，状态机在 `RirTrackLifecycle.cpp` |
| B7 | `frame_count==0` ⇒ 输出 `滑行`（≤ max_miss）或 `丢失` | 77 项 326 行"帧数=0 仍判跟踪"；SBIRS 侧 `SbirsPipeline.cpp:2319-2322` 的 `nfov_tracking_coasting` 已具备该语义，缺陷在 RIR 侧 |

---

## 7. P5：引导连续性（识别能否落到 ICBM 的前提）

### P5.1（1q）补齐"库内推导保持期"
- 实现 `RirTrackRequest.h:79` 的承诺：`valid_until_cycle_index == 0` 时由库内按
  `position_sigma_m`/`velocity_sigma_mps`/跟踪窗口导出到期周期（无 sigma 时用**配置默认上限**，如 5 周期），
  `RirSession.cpp:463-465` 的判据改为使用该推导值。
- 顺带补 `RirExternalTrackState::extrapolated_age_sec`（现已声明但**从未赋值**）。
- 测试扩 `tests/unit/remote_identification_radar/rir_external_track_test.cpp`：
  ①连续不提交时 `coasting_cycles` 递增并在到期转 `kExpired`；②到期后重新提交恢复 `kFresh`；
  ③显式 `valid_until_cycle_index` 覆盖推导值；④`extrapolated_age_sec` 数值正确。

### P5.2（宿主）按 phase 行为，而不是"第一个无引导周期就清"
- `lrdr_sensor_1q.cpp:1493` 的 `ClearExternalTrack()` 改为：仅在 `phase == kExpired` 时释放并回搜索；
  `kCoasting` 期间**保留**并从 `GetExternalTrackState()` 打印外推周期数。
- 引导速度非零保障（`lrdr_sensor_1q.cpp:1438-1445`）：位置不变时差分为 0 ⇒ 用最近两拍位置差 / `dt`，
  仍为 0 则回填上一次有效速度，避免库内外推退化为"钉在原地"。

**验收**：第 65 项越界行归零；第 67 项中段观测连续性（周期覆盖率）显著上升；
第 47A 项空输出占比 <50%、置信度上限 >0.6（能判出 ICBM）；`[cue]` 日志出现完整的 `kFresh → kCoasting → kExpired` 序列。

---

## 8. P6：集成迁移准备（可与 P2/P5 并行）

按 `integration_boundary_1q_vs_host_2026-10-19.md` §8 顺序：
1. **先修 4 类语义漂移**——尤其 `SimSecondsToJulianDay` 兜底分支缺 `2451545.0` 纪元偏移：兜底应**返回 false**，
   由上层跳过该周期并计数告警（否则 GMST 错位数千年且不报错）；
2. 收敛 `GregorianToJulianDay` / `SimSecondsToJulianDay` / `dt_sec` / `oneq_diag_log.h` 到 `1q_integration/` 单一实现；
3. 三连坐标变换改 `TryMakeEnuSceneState`，删 `TryRotateEciDirectionToEcef` / `IsFinite*` / boresight 归一化；
4. 4 个 API 缺口：`TryGregorianToJulianDay`、`TryEciToEcefDirection`、Body→NED↔Body→ECI、`numerics` 公开
   （其中**极化 100 km 参考距离**与 **`ComputeLookAngles`** 正好与 P1.2 的 A4 同源，建议合并做一次）；
5. 抽宿主接口层（7 类），再删临时诊断块。

---

## 9. 需要拍板的 4 件事

| # | 决策 | 建议 |
|---|---|---|
| 1 | A1 角速率派生口径 | **解析法（P1.1 方案 ③）**：无历史状态、无堆风险、与滤波自洽 |
| 2 | 引导保持期归属 | **库内推导**（兑现头文件承诺）+ 宿主只读 phase；不要新增宿主侧超时参数 |
| 3 | 4.8 MW 是否回退 | **等 P2 数据**再定，先别回退（避免同时改两个变量） |
| 4 | 配置自检脚本放哪个仓库 | 建议放 **BJ 仓库 `ModelSource\`**（与配置同仓，交付时一起走） |

---

## 10. 每轮纪律（沿用既有教训）

1. 一次只改一个变量；改完**完整重启宿主**（识别库只在路径变化时重载 ⇒ 换 `.db` 必须重启）。
2. 每轮记录基线表（§2 P0.3），与上一轮**只允许一处差异**。
3. 改完 1q 源码必做：`python scripts\utf8_bom.py convert`（否则 MSVC 按 GBK 解析中文注释报 C2059）。
4. 重建顺序：`sbirs_engine` → **聚合库 `1q_lib`** → 测试 → v120 `1q.lib` → BJ 重编 → 重跑。
5. 提交口径：1q 一笔（源码/文档）、BJ 一笔（配置/集成代码/`1q.lib`）；`git checkout -- ModelSource/library` 清构建噪声。
6. 已知崩溃模式：给 `SbirsTrackingCoordinator` 加成员曾导致堆损坏 `0xC0000374`——任何类布局变更都必须
   **全量重编 + 281 项全跑**，不能只增量编。

---

## 11. 暂不做 / 已证伪（避免重试）

- ❌ 把门限（0.001）先调高：P2 未完成时调门限只会把 SNR −28 dB 的问题转移到"全拒"。
- ❌ 引导键改用 `UnitID`；❌ 单纯提功率识别 ICBM；❌ 识别窗口停在早期判 ICBM（物理不可分）；
  ❌ 加卫星/改雷达位置解决调度（瓶颈是每星窄视场只锁一个目标）；❌ 辐射强度统一调到 1e5–1e6（只对助推段成立）。
- ⏸ 第 68 项跨模块聚合边界、第 57/61 项配置回显字段：仍需需求方冻结，不列入本轮。
