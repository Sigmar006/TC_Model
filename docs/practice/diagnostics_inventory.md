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

## 3. 已完成的排查结论（供后续判读，勿重复走）

1. **两点起批速度种子已在 BJ 集成实证为负**：让航迹持续存活后发散，最好航迹由 8.8 km 劣化到 1119 km。已回滚（`ea43e9a7`）。
2. **IMM 与跟踪层不是发散源**：探针 A/B 显示 IMM 开/关逐拍几乎逐位相同；静止目标、量测噪声、间歇可见性下均不发散。
3. **发散主因是"融合定位被写进场景目标位置"**：`id=102` 在 cycle=3230 单拍跳 6835.9 m（真值速度 0）。已改为外部跟踪指向（BJ `ce3b35d`）。
4. **`id=102`（指挥所-1）真值位置永久在地平线以下**：全轮仅 `rir.target_earth_occulted`。引导决定往哪看，但不能把地平线以下目标变可见——若要它可探测，须改场景几何或融合定位高度。
5. **目标表在 cycle≈2972 整体左移一位**（id=101 弹道导弹离场），`entity_index` 不再稳定；`max_lost_cycles=2090` 会让离场目标的航迹长期残留外推，判读误差须按航迹号分组看年龄。
6. **第 42 项已达标**：新口径下 726 个样本全属 id=101，平均 286 m、最大 2341 m，真值斜距 198.7 km 起 ⇒ 最坏相对误差 1.18%（判据 ±5%）。
