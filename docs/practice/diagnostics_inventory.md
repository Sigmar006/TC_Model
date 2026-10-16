# 诊断代码与排查物登记表（待统一清理）

Status: active（临时登记；全部清理后可整体删除本文件）
Last-updated: 2026-10-16
Authority: **本表是清理诊断代码/临时排查物的唯一依据。** 新增诊断必须在此登记；
         标注 `[DIAG-<日期>]` 的代码块按本表删除，未登记的诊断视为遗漏。

## 0. 怎么用

1. 搜索标记 `DIAG-` 即可定位全部待删诊断代码。
2. 按本表逐条删除「可删」项；「**保留**」项是产品功能或契约说明，**不得误删**。
3. 删完把本文件一并删除（或改为已清理记录）。

---

## 1. 集成侧（BJ_10Y_WorkSpace）

仓库：`C:\software\workspace\1qfosim\BJ_10Y_WorkSpace`
文件：`ModelSource\CyModel\Equipment\Sensors\LRDR_SENSOR\lrdr_sensor_1q.cpp`

| 标记 | 位置（2026-10-16 行号，即标记所在行） | 大致内容 | 处置 |
|---|---|---|---|
| `[DIAG-2026-10-16-MEAS]` | `RirRuntime` 字段块 **L470–L486**（起止标记各一行）；构造初始化列表里对应的 **L514–L520** 共 7 行（`meas_last_east()` … `meas_last_cycle()`） | 逐目标"上一拍喂进 1q 的位置 + 周期号 + 历史最大跳变"共 6 个 map（`meas_last_east/north/up/time`、`meas_max_jump`、`meas_max_jump_cycle`、`meas_last_cycle`） | **可删**（字段块 + 构造列表 7 行） |
| `[DIAG-2026-10-16-MEAS]` | 量测诊断块 **L1213–L1271**（起止标记各一行，块体连大括号整删） | 每拍算 `|Δp|` 与真实 `dt`，只在刷新该目标历史最大跳变时打印。产出 `[meas-first]`、`[meas-gap]`、`[meas-jump]` 三种行 | **可删**（整块） |
| `[DIAG-2026-10-16-CUE]` | **L1410–L1427** | 外部跟踪提交结果诊断 `[cue-track]`：打印 `submitted/phase/中心ECEF/速度/窗口/source_time` | **可删**（含 `if` 判断与 `LrdrDiagLog` 调用） |
| `[DIAG-2026-10-16-CUE]` | **L1442–L1446** | 引导消失诊断行 `[cue-track] … 引导消失，已清除外部跟踪` | **可删**（仅该 `LrdrDiagLog` 调用） |
| `[DIAG-2026-10-16-RECOG]` | `StepWithResult` 之后紧邻一段（搜 `DIAG-recog`） | 逐航迹打印识别 `best_score` / `runner_up_score` / `confidence` / 型号 / 四维相似度 / 掩码 / 观测数，每 100 周期一条。**为什么不复用库内那条**：`RecognitionTracker` 的 `fell below acceptance` 走 `PROJECT_LOG_INFO`，而 Windows 上 `PROJECT_LOG` 默认关闭、宿主也不捕获 stdout，集成实跑看不到；而这两个分数量是判「得分不够 vs 可分性不够」的唯一入口 | **可删**（整段）；删除时建议顺带评估把库内那条改走验收通道 |
| — | `cue_center_valid = true;`（**L1142**）一带 | 引导中心收集（**不是诊断**，是产品逻辑） | **保留** |
| — | **L1349**–L1447 外部跟踪提交整块 | `SubmitExternalTrack` 的**产品实现**（引导只改指向不改位置）；`cue_prev_ecef_*` / `cue_active_id` 等运行态（**L487–L500** 声明）同为产品状态 | **保留** |
| — | `LRDR_SENSORData.hpp` 的 `cue_track_az_half_deg` / `cue_track_el_half_deg` | 邻域窗口参数（产品配置） | **保留** |

> 说明：`lrdr_sensor_1q.cpp` 里**原有的**诊断体系（`[init]` / `[cycle]` / `[view]` / `[issue]` /
> `[scene]` / `[scene-vel]` / `[cue]` / `[feature-pick]` / `[designation]`）是**上一轮及更早**加的，
> **不在本表范围**，需要时另行登记。

### 集成侧一次性产物（可直接删）

| 物 | 位置 | 说明 |
|---|---|---|
| 旧库备份 | ~~`ModelSource\lib\x64\Release\1q.lib.bak-20261015-1724`~~ | **已于 2026-10-16 删除** |
| 诊断日志（历史） | `FOSimStudio\bin\App\lrdr_rir_diag_<时间戳>.log` 等 | 每轮新建、带时间戳；确认结论后可按时间批量清理 |

---

## 2. 1q 库侧

仓库：`C:\software\workspace\1qfosim\1q`

| 物 | 位置 | 大致内容 | 处置 |
|---|---|---|---|
| 量测快照缓存（已回滚） | `src/remote_identification_radar/runtime/RirController.{h,cpp}` | 两点起批速度种子：`RirPriorMeasurement` + 两张 map + `SeedVelocityFromPriorMeasurement()`。**已由 `ea43e9a7` 整体回滚**，当前工作区不存在 | 无需处置（核对：源码里应无 `SeedVelocityFromPriorMeasurement`） |
| 探针工具 | `build/_imm_probe/imm_probe.cpp` + `build.cmd`（约 150 行） | 链接集成那份 v120 `1q.lib`，按集成配置与 LRDR 部署几何复刻跟踪场景，用 `--argv[1]=IMM 开关`、`argv[2]=周期数`、`argv[3]=可见性间隔` 三参数跑 A/B；曾用它排除 IMM 与跟踪层 | **建议保留**（可复现工具，重跑成本低）；若要删，连同 `build/_imm_probe` 整目录删除 |
| 验收日志副本 | ~~`build/_ab_new`、`build/_ab_new_diag`~~ | 新库那轮的 RadarOutput/诊断日志拷贝 | **已于 2026-10-16 删除** |

> 注：`build/` 被 `.gitignore` 忽略，上述探针不进版本库，只在本地存在。

---

## 4. 追加登记：2026-10-15 一轮的集成侧诊断与"不可删"清单

> 依据 §1 注（"`[scene-vel]` / `[feature-pick]` 等是上一轮及更早加的，不在本表范围，需要时另行登记"）
> 由另一会话补登记。行号为 2026-10-15 当时值，**定位请用搜索标记**。

### 4.1 可删诊断

| 仓库/文件 | 搜索标记 | 大致行号 | 内容 | 处置 |
|---|---|---|---|---|
| BJ `…\Sensors\LRDR_SENSOR\lrdr_sensor_1q.cpp` | `feature_pick_logged` | L460–463（成员声明）、L511（构造初始化列表） | `RirRuntime` 里 `std::map<unsigned int, std::string> feature_pick_logged` + 注释 | **可删** |
| 同上 | `[feature-pick]` | ~L1186–1213 | 特征集选择结果：首次/变化时输出 `idx=/target=/id=/rcs_in=/set='/set_rcs=/out_rcs=/aspect=/pol=/scatterers=` | **可删**（整块含注释） |
| 同上 | `[scene-vel]` | ~L1178–1204 | 场景目标真值速度与步长：`truth_vel_enu=(…)m/s abs_v=… has_vel=… dt_from_last=…`，节流 `idx<=5 \|\| idx%500==0` | **可删**（整块含注释） |
| BJ `…\DataProcessors\SBIRSFusionDp\sbirsfusiondp_1q.cpp` | `[init] target inference engine created` / `disabled` | L348 / L352 | 推演引擎创建/关闭确认行 | **可删** |
| 同上 | `[predict]` | L1012 | 每周期推演概况 `cycle=/tracks=/results=` | **可删** |
| 同上 | `det_total` / `det_drop_no_id` / `det_drop_stage` / `det_anchor_ok` | 声明 L549–552、自增 L578/583、输出 L1035–1044 | 观测锚点提取层的**丢弃分类计数**；`[verbose]` 行尾追加 `det=/drop_no_id=/drop_stage=/anchor_ok=/anchors=/targets_multi_sat=` 与配套统计循环 | **可删** |

> `verbose_log` 开关（`SBIRSFusionDpData.hpp`，默认 1）只控制上述 `[verbose]`/`[predict]` 输出；
> 删诊断后可保留参数（无输出）或连参数一起删——**后者需同步四处**：`.hpp` / `.mdef` / `.mui` /
> `ModelDatabase\Equipment\DataProcessors\红外地面站.mdata`。

### 4.2 **不可删**（产品功能 / 契约 / 配置）

| 位置 | 内容 | 原因 |
|---|---|---|
| BJ `lrdr_sensor_features.{h,cpp}` + `lrdr_sensor_1q.cpp` 调用点 | `SelectFeatureSet` 新增入参 `_scene_rcs_m2`（用场景真值 RCS 兜底"就近选档"） | 功能修复：修"导弹被装配成 `air_fighter` ⇒ 识别成 F-16C" |
| BJ `sbirsfusiondp_1q.cpp` | 三处 `printf` → `FUSION_DIAG_LOG`（`first dual-LOS fix ok` / `first fused track created` / 逐周期概况） | 宿主不落盘 stdout，不改则此类信息永远不可见 |
| BJ `sbirsfusiondp_1q.cpp` | 推演引擎接入（`TargetInferenceEngine::Infer`） | 驱动验收第 20/21/22/23/25 项 |
| BJ `SBIRSFusionDpData.hpp` | 11 个参数的 `Serialize`/`Deserialize`（原为空实现） | 参数可配的前提 |
| BJ `SBIRSFusionDpData.hpp` | `inference_enabled`（默认 1） | 推演开关（功能） |
| BJ 想定 `20261006-LRDR和SBIRS测试想定.sdata` | 预警雷达站点 300 m AGL | 第 34 项杂波几何窗口 |
| BJ `bin\App\config\sbirs.json`（+源副本） | 8° 视场 + 3 行俯仰栅格 + `span 360` / `rate 78` | 双星交会成立的前提（配对窗口 35 配套） |
| BJ `ModelDatabase\…\红外地面站.mdata` | `pair_window_cycles=35`、`narrow_field_stage_min=1` | 同上 |
| 1q `src/common/logging/AcceptanceFileLog.cpp` | 逐行 `flush()` + `ClaimFirstOpenForPath`（跨模块"首次打开"判定） | 分别修"运行期读不到日志"与"跨 DLL 截断抹掉第 10 项" |
| 1q `src/remote_identification_radar/internal/RirSurfaceClutterModel.{h,cpp}` + `runtime/RirController.{h,cpp}` | 杂波区面积/回波改用"波束下沿地面交点距离"；输入加 `platform_altitude_m` | 修远距目标杂波 ∝R² 非物理放大 |
| 1q `src/remote_identification_radar/recognition/MotionFeatureExtractor.cpp` | 标量模长兜底（`speed`/`acceleration_mps2` 为 0 时取向量模长） | 修复；注意其与 §3.1"两点起批已回滚"是两件事 |

> **1q 侧本会话未新增任何纯诊断代码**（避免走 `PROJECT_LOG_*`——Windows 上默认关闭 ⇒ 写了看不到）。

### 4.3 删除操作清单

```
1) 按 4.1 逐项删除（搜索标记定位，勿按行号硬删）
2) 重编集成工程（先关宿主：ModelSource\build_x64_v120.cmd）
3) 跑一轮：验收条目数与内容不应变化；[cue] 引导/推演输出不受影响
4) 应消失：[feature-pick] / [scene-vel] / [predict] / [verbose] 行尾的 det=… 字段
5) 提交 BJ 仓库一笔
```

---

## 3. 已完成的排查结论（供后续判读，勿重复走）

1. **两点起批速度种子已在 BJ 集成实证为负**：让航迹持续存活后发散，最好航迹由 8.8 km 劣化到 1119 km。已回滚（`ea43e9a7`）。
2. **IMM 与跟踪层不是发散源**：探针 A/B 显示 IMM 开/关逐拍几乎逐位相同；静止目标、量测噪声、间歇可见性下均不发散。
3. **发散主因是"融合定位被写进场景目标位置"**：`id=102` 在 cycle=3230 单拍跳 6835.9 m（真值速度 0）。已改为外部跟踪指向（BJ `ce3b35d`）。
4. **`id=102`（指挥所-1）真值位置永久在地平线以下**：全轮仅 `rir.target_earth_occulted`。引导决定往哪看，但不能把地平线以下目标变可见——若要它可探测，须改场景几何或融合定位高度。
5. **目标表在 cycle≈2972 整体左移一位**（id=101 弹道导弹离场），`entity_index` 不再稳定；`max_lost_cycles=2090` 会让离场目标的航迹长期残留外推，判读误差须按航迹号分组看年龄。
6. **第 42 项已达标**：新口径下 726 个样本全属 id=101，平均 286 m、最大 2341 m，真值斜距 198.7 km 起 ⇒ 最坏相对误差 1.18%（判据 ±5%）。
