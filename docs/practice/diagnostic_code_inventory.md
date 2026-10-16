# 诊断代码清单（便于日后统一删除）

> 记录 2026-10-15 这一轮为排查 BJ 集成问题而**新增的诊断代码**：位置、内容、删除方式。
> 另标注**不是诊断、不可删**的修复代码，以及**既有诊断**（本次仅改输出通道）。
> 行号为 2026-10-15 当时值，删除前请按"搜索标记"重新定位。

---

## A. 本会话新增的诊断代码（**可整块删除**）

### A1. `BJ_10Y_WorkSpace\ModelSource\CyModel\Equipment\Sensors\LRDR_SENSOR\lrdr_sensor_1q.cpp`

| # | 搜索标记 | 大致行号 | 内容 | 删除方式 |
|---|---|---|---|---|
| 1 | `feature_pick_logged` | 460–463 | `RirRuntime` 里的成员声明：`std::map<unsigned int, std::string> feature_pick_logged;` + 其上方 3 行注释 | 删成员与注释 |
| 2 | `, feature_pick_logged()` | 511 | 构造函数初始化列表中的一项 | 删该行 |
| 3 | `[feature-pick]` | ~1186–1213 | `ApplyFeatureSet` 之后的一整块：查 `feature_pick_logged`，首次/变化时输出<br>`[feature-pick] idx=… target=… id=… rcs_in=… set='…' set_rcs=… out_rcs=… aspect=… pol=… scatterers=…` | 删整个 `if` 块（含上方注释） |
| 4 | `[scene-vel]` | ~1178–1204 | 场景目标速度写入之后的一整块：`std::sqrt` 求 `abs_v` + `LrdrDiagLog("[scene-vel] … truth_vel_enu=(…)m/s abs_v=… has_vel=… dt_from_last=…")`，节流条件 `idx <= 5 \|\| idx % 500 == 0` | 删整个 `if` 块（含上方注释） |

> 该文件里 `[cue]` / `[cue-resolve]`（367/385/390/1156 一带）来自**双星融合引导功能**（提交 `e1c723f`），
> **属功能诊断，不是本次排查新增** —— 若该功能保留则不要删。

### A2. `BJ_10Y_WorkSpace\ModelSource\CyModel\Equipment\DataProcessors\SBIRSFusionDp\sbirsfusiondp_1q.cpp`

| # | 搜索标记 | 大致行号 | 内容 | 删除方式 |
|---|---|---|---|---|
| 1 | `[init] target inference engine created` | 348 | 推演引擎创建确认行 | 删该 `FUSION_DIAG_LOG` 调用 |
| 2 | `[init] target inference disabled` | 352 | 推演关闭提示行 | 删该 `FUSION_DIAG_LOG` 调用 |
| 3 | `det_drop_no_id` / `det_drop_stage` / `det_anchor_ok` / `det_total` | 549–552（声明）、578/583（自增）、1035–1044（输出） | 观测锚点提取层的**丢弃分类计数**（本次为定位"帧很多但 fix_records=0"而加）：4 个 `std::size_t` 声明 + 3 处 `++` + `[verbose]` 行尾追加 `det=/drop_no_id=/drop_stage=/anchor_ok=/anchors=/targets_multi_sat=` | 删 4 个声明、3 处自增、以及 `[verbose]` 里追加的那一段（含 `anchors_total`/`targets_multi_sat` 的统计循环） |
| 4 | `[predict]` | 1012 | 每周期推演概况行（`cycle=/tracks=/results=`） | 删该 `FUSION_DIAG_LOG` 调用（注意其外层 `if (data_.verbose_log != 0)` 可一并删） |

> `[init] fusion runtime created`(356)、`[event] first …`(918/924/932)、`[cycle] …`、`[receive] …`、
> `[process-enter] …` 等为**既有诊断**（本会话仅把其中三个 `printf` 改为 `FUSION_DIAG_LOG`，见 B3）。

### A3. 集成侧诊断开关（保留或删除均可）

| 位置 | 内容 | 说明 |
|---|---|---|
| `SBIRSFusionDpData.hpp` | `verbose_log`（默认 1）、`inference_enabled`（默认 1） | **同时是功能开关**：`inference_enabled` 控制推演（第 20/21/22/23/25 项），**不要删**；`verbose_log` 只控制 `[verbose]`/`[predict]` 行，可在删 A2 时一并清理 |
| `SBIRSFusionDp.mdef` / `.mui` / `ModelDatabase\Equipment\DataProcessors\红外地面站.mdata` | 上述两个参数的 schema/界面/实例值 | 与上面配套；删参数需四处同时改 |

---

## B. **不是诊断，不要删**（本次同时做的修复，删了会回退功能）

| # | 位置 | 内容 |
|---|---|---|
| B1 | `LRDR_SENSOR\lrdr_sensor_features.h` / `.cpp` | `SelectFeatureSet` 新增入参 `_scene_rcs_m2`（用场景真值 RCS 做"就近选档"兜底）——**功能修复**，修"导弹被装配成 air_fighter 导致识别成 F-16C" |
| B2 | `LRDR_SENSOR\lrdr_sensor_1q.cpp` | 调用点传入 `scene_target.rcs`（B1 的配套） |
| B3 | `SBIRSFusionDp\sbirsfusiondp_1q.cpp` | 三处 `printf` 改为 `FUSION_DIAG_LOG`（`first dual-LOS fix ok` / `first fused track created` / 逐周期概况）——宿主不落盘 stdout，不改则永远看不到 |
| B4 | `SBIRSFusionDp\sbirsfusiondp_1q.cpp` | 推演引擎接入（`TargetInferenceEngine::Infer`）——驱动验收第 20/21/22/23/25 项，**核心功能** |
| B5 | `SBIRSFusionDp\SBIRSFusionDpData.hpp` | `Serialize`/`Deserialize` 补全 11 个参数（原为空实现） |
| B6 | 想定 `20261006-LRDR和SBIRS测试想定.sdata` | 预警雷达站点抬到 300 m AGL（第 34 项杂波几何窗口）——**场景配置** |
| B7 | `bin\App\config\sbirs.json` + 源副本 | 8° 视场 + 3 行俯仰栅格 + 全盘覆盖（span 360 / rate 78）——**配置** |
| B8 | `ModelDatabase\Equipment\DataProcessors\红外地面站.mdata` | `pair_window_cycles=35`、`narrow_field_stage_min=1` ——**配置** |

### B9. 1q 仓库内的改动（全部是修复，**无诊断**）

| 位置 | 内容 |
|---|---|
| `src/common/logging/AcceptanceFileLog.cpp` | ① 每行 `flush()`（`Write` 内）；② `ClaimFirstOpenForPath` 跨模块"首次打开"判定（修跨 DLL 截断） |
| `src/remote_identification_radar/internal/RirSurfaceClutterModel.{h,cpp}` | 杂波区面积/回波改用"波束下沿地面交点距离"（修 ∝R² 非物理放大）；输入结构加 `platform_altitude_m` |
| `src/remote_identification_radar/runtime/RirController.{h,cpp}` | 上述 `platform_altitude_m` 的透传（入参 + 填充 + 调用点） |
| `src/remote_identification_radar/recognition/MotionFeatureExtractor.cpp` | 运动特征标量模长兜底（`speed`/`acceleration_mps2` 为 0 时取向量模长） |

> 1q 侧**没有**新增任何纯诊断代码（避免走 `PROJECT_LOG_*`——它在 Windows 上默认关闭，写了也看不到）。

---

## C. 既有诊断（他人/早先添加，本次未新增，**保留**）

| 位置 | 内容 | 备注 |
|---|---|---|
| `LRDR_SENSOR\lrdr_sensor_1q.cpp` | `[init]` / `[cycle]` / `[view]` / `[issue]` / `[scene]` / `[rf-scene]` / `[detect-entry]` / `[designation]` / `[meas-first]` / `[meas-gap]` / `[meas-jump]` | 排查主入口；`[meas-*]` 非本次新增 |
| `LRDR_SENSOR\lrdr_sensor_1q.cpp` | `LrdrDiagLog` 本体（第 160 行） | 诊断基础设施，删了所有 `LrdrDiagLog` 调用都编不过 |
| `OPIR_SENSOR\opir_sensor_1q.cpp` | `[init]` / `[push]` / `[cycle-result]` / `[issue]` / `[push-skip]` | 本次未改动 |
| `1q\src\remote_identification_radar\recognition\RecognitionTracker.cpp:362-374` | `sims=(rcs=…,motion=…,pol=…,rp=…)` 打分报告行 | **走项目日志、Windows 上默认关闭 ⇒ 实际看不到**；要用得改到验收行 |
| `1q\src\common\logging\*` | 项目日志框架 | — |

---

## D. 统一删除的操作清单（建议顺序）

```
1) 删 A1 的 4 处（LRDR：[feature-pick] 块与成员、[scene-vel] 块）
2) 删 A2 的 4 处（融合：[predict]、推演 init 两行、丢弃分类计数）
3) 若 A2 全删 → 决定 verbose_log 开关去留（保留参数但不产生输出，或连参数一起删：需同步
   SBIRSFusionDpData.hpp / .mdef / .mui / 红外地面站.mdata 四处）
4) 重编集成工程（ModelSource\build_x64_v120.cmd，先关宿主）
5) 跑一轮确认：六份验收日志条目数不变（诊断删除不应影响验收输出）
6) 提交：BJ 仓库一笔；若同时清了 verbose_log 参数，一并说明
```

**删除后不应改变的东西**：验收条目数量与内容、`[cue]` 引导行为、推演输出（第 20/21/22/23/25 项）。
**删除后应消失的东西**：`[feature-pick]` / `[scene-vel]` / `[predict]` / `[verbose]` 行尾的 `det=/drop_*` 字段。

---

## E. 相关文档

| 文档 | 用途 |
|---|---|
| [fosimstudio_host_and_integration_guide.md](fosimstudio_host_and_integration_guide.md) | 宿主/测试工程怎么用：数据与想定位置、集成代码位置、编译与运行、日志输出路径 |
| [rir_tracking_two_point_seed_and_acceptance_guide.md](rir_tracking_two_point_seed_and_acceptance_guide.md) | "跟踪速度恒 0"缺陷的排查与验收指南（含已证伪假设清单） |
