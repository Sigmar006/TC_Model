# 会话交接（2026-10-19）

Status: handoff
用途：新会话继承本会话的结论、未完成工作与集成调用逻辑。**先读本文，再读 §9 的专题文档。**

---

## 1. 本会话做了什么（按时间）

| 阶段 | 内容 | 结果 |
|---|---|---|
| 验收字段补齐 | 第 74/75/76/77 项交换数据字段 | ✅ 六项 100% 覆盖（`0574ff17`/`13b7e203`/`4964556d`）|
| 背景辐射 | `0 → 1.2 W/(m²·sr)`，光子噪声由 0 变 1.451e-15 J | ✅ `d225e57d` |
| 四类弹道导弹入库 | SRBM/MRBM/IRBM/ICBM | ✅ `a31435a7` |
| 引导交接修复 | 角误差模型复原、过程噪声 20、单星保活 | ✅ 检测窗口 202→1313，斜距至 5038 km |
| 功率提升 | 582 kW → 4.8 MW | ✅ 1970 km 处 SNR 精确到 16.19 dB（预测 16.20）|
| 特征/模板修正 | RCS 网格扩到全周向；ICBM 高度模板 1.15→1.5 Mm | ✅ 但识别仍卡在稀疏观测 |
| 审计 | 验收日志物理合理性 + 集成边界 | ✅ `1dbd9d5d` |
| 修改方案 | A/B/C 三批 | ✅ `7a52d28b`（**未实施**）|

---

## 2. 当前工程状态

### 2.1 仓库

| 仓库 | 分支 | HEAD | 说明 |
|---|---|---|---|
| `C:\software\workspace\1qfosim\1q` | `fosim_2013` | `7a52d28b` | 静态库源码 |
| `C:\software\workspace\1qfosim\BJ_10Y_WorkSpace` | `master` | `5eb5e4b` | FOSimStudio 集成 |

### 2.2 未提交改动（1q）

| 文件 | 内容 | 状态 |
|---|---|---|
| `src/sbirs_sensor/pipeline/SbirsPipeline.cpp` | 77 项角速度日志改为"不适用(首拍无历史)"；第 16 项验收行改为按后端门控 | ✅ 测试 281/281 通过 |
| `.vscode/settings.json`、`docs/README.md`、`docs/review/ir_interface_log_review_2026-10-17.md`、`tests/unit/.../rir_designation_task_test.cpp`、`tools/db_probe/db_probe.cpp` | 非本会话主线改动（子代理/编辑残留） | ⚠ 提交前需人工确认 |

> ⚠ 未跟踪文件里 `_rad.txt` 是临时产物，可删。

### 2.3 测试基线

```
1q_sbirs_sensor_unit_tests : 281/281 PASS
```

### 2.4 部署状态（BJ）

| 产物 | 时间 | 说明 |
|---|---|---|
| `ModelSource\lib\x64\Release\1q.lib` | 15:17 | 含验收字段补齐；**不含**之后的模板/网格修正（那些是 DB/JSON，不需重编）|
| `ModelSource\bin_x64_Release\Sensors.dll` | 15:26 | 同上 |
| `DataProcessors.dll` | 15:26 | 含单星保活 |

---

## 3. 立刻要做的（第一优先）

### 3.1 完成 A1：角速率派生 —— **必须避开协调器**

**已试过的错误做法**：在 `SbirsTrackingCoordinator` 里加 `std::map<...> angle_rate_history_` 成员与 `last_dt_sec_`，
编译通过但**运行期堆损坏**（`0xC0000374`），7 个测试崩溃。已回退。

**正确做法（建议）**：在 **`SbirsPipeline`** 内派生，不动协调器的类布局：
1. 在 `SbirsPipeline` 私有区加 `std::map<std::uint64_t, struct { double az; double el; double t; bool has; }>`；
2. 在拿到 `tracking_result` 之后（`SbirsPipeline.cpp` 约 L2337 处）用
   `tracking_result.output_azimuth_deg / output_elevation_deg` 与 `input.dt_sec` 差分；
3. 方位差要回卷到 `(−π, π]`；
4. 写入 `tracking_result.has_angle_rate / azimuth_rate_rad_per_s / elevation_rate_rad_per_s`；
5. 若后端是 `kAngleCvKf` 则保留协调器给的原值（不要覆盖）。

**派生公式（供交叉校验）**

```
r_rel = r_target − r_sat,  v_rel = v_target − v_sat
u = r_rel/|r_rel|,  v_trans = v_rel − (v_rel·u)u
|ω| = |v_trans| / |r_rel|
ω_az = (x·v_y − y·v_x)/(x²+y²)
ω_el = (ρ·v_z − z·(x·v_x + y·v_y)/ρ)/(ρ²+z²),  ρ=√(x²+y²)
```

⚠ 必须减掉**卫星自身速度**（GEO 在 ECI 约 3 km/s），否则派生值被卫星运动主导。

### 3.2 验收日志修改方案尚未实施

按 `docs/review/acceptance_log_fix_plan_2026-10-19.md` 批次推进。**批次 1 只剩 A2/A3/A4**，批次 2 起全是模型/配置改动。

---

## 4. 之前保留、仍未完成的待办

| # | 待办 | 位置 | 备注 |
|---|---|---|---|
| 1 | 恢复 `irar_` 赋值 | `cyairportData.cpp:58`、`CyQPEGVehicleData.cpp:58`（均被注释） | 与"辐射强度恒 10388"同源 |
| 2 | 噪声模型重标定 | `src/sbirs_sensor/foundation/SbirsNoiseModel.cpp` | 光子功率缺 τ_atm；热噪声 `√(4kTt)` 为代理式、压过一切 |
| 3 | 门限重标定 | `sbirs.json → policy.detection.{wide,narrow}_min_snr_linear` | 现 0.001，通过率 99.24%，无判别力 |
| 4 | NIS 门与数值不自洽 | `SbirsTrackingCoordinator.cpp` IMM 分支 | **先加诊断**（打印参与判决的 NIS/门限/模型数/gate 来源）再改 |
| 5 | RIR 滤波发散未复位 | `src/remote_identification_radar/tracking/RirTrackLifecycle.cpp` | 65 项 59% 航迹高度 >2293 km（max 7424 km）|
| 6 | 帧数=0 仍判"跟踪" | 同上 | 326 行 |
| 7 | `剩余角` 语义 | `SbirsPointingActuator` | 应为 \|实际−指令\|，现 91.86% 为 0 |
| 8 | `连续命中` 计数器不累加 | `SbirsPipeline.cpp` WFOV 命中计数 | 恒 1 |
| 9 | 46 项极化三特征恒定 | `PolarizationFeatureExtractor.cpp` | 方向角 100% 为 0 |
| 10 | 引导容限期 | `lrdr_sensor_1q.cpp:1493` | 第一个无引导周期就 `ClearExternalTrack()`，丢掉了库内的外推/保持能力；应用 `cue_max_age_cycles` |
| 11 | 引导速度非零保障 | `lrdr_sensor_1q.cpp:1438-1445` | 位置不变时差分为 0，库内无法外推 |
| 12 | 诊断清理 | `[DIAG-2026-10-16-*]` 标记 | 登记于 `docs/practice/diagnostics_inventory.md` |
| 13 | 集成迁移准备 | 见 `docs/practice/integration_boundary_1q_vs_host_2026-10-19.md` | 4 类语义漂移 + JD 兜底 bug，**迁移前必须先修** |

---

## 5. 集成调用逻辑（三个工程）

### 5.1 LRDR（`Sensors\LRDR_SENSOR\lrdr_sensor_1q.cpp`）

```
RunRirCycle
  ├ 站点 ECEF→LLA                  TryEcefToLla
  ├ 目标 ECEF→ENU(位置/速度)        TryEcefToEnu / TryEcefToEnuVelocity
  │   └ 填 RirSceneTarget（含 aspect_rcs_samples / polarization_samples / range_rcs_scatterers）
  ├ 引导：PullFusionCues（读 SBIRSFusionDp 记录，**按 TrackKey** 索引）
  │   ├ SubmitExternalTrack(position/velocity/source_time/window)   ← 照射中心，逐周期重发
  │   │    库内：position + velocity × (sim_time − source_time) 线性外推
  │   │    valid_until_cycle_index=0 → 库内按速度与位置误差自动推导保持期
  │   └ TryApplyRuntimeConfig(has_designated_target_id)            ← 指定识别，仅目标变化时下发
  └ RirSession::StepWithResult(RirCycleInput) → RirCycleResult
```

**关键口径**
- 引导只驱动**指向**，不改写场景目标位置（旧做法会让量测等于融合定位，产生数公里阶跃）。
- `scan_window` 的 az 是**相对 `scan_center`**、el 是**绝对**；两者必须一起改。
- 波束半功率门仅 **±0.2°**。
- 检测准入在 `kSnrFallback` 模式下是**硬编码 6.0 dB**，不是配置里的 `min_snr_db`。
- 识别库只在**路径字符串变化**时重载 ⇒ 改 `.db` 必须重启宿主。

### 5.2 OPIR（`Sensors\OPIR_SENSOR\opir_sensor_1q.cpp`）

```
RunSbirsCycle
  ├ SimSecondsToJulianDay(宿主儒略秒) → input.utc_julian_day
  ├ 场景目标 → SbirsSceneTarget（radiant_intensity 目前恒为回退常量 10388.114607）
  ├ SbirsSession::StepWithResult(SbirsCycleInput) → 测角 + 归属
  └ 组 SbirsFusionDpFrameRecord，经 CyberMalImpl 投递给 SBIRSFusionDp
      · UtcJulianDay 必须与本星同一 utc_julian_day
      · ObservationStage 原样透传（<2 会被 DP 当未稳光轴过滤）
      · TargetID = 归属真值目标 ID（0 = 不入交会）
```

### 5.3 SBIRSFusionDp（`DataProcessors\SBIRSFusionDp\sbirsfusiondp_1q.cpp`）

```
Process → RunSbirsFusionCycle
  ├ AcceptFusionFramesFromMal → inbox
  ├ 锚点缓存（仅 NFOV 精跟，stage ≥ narrow_field_stage_min）
  ├ 逐目标择优两星交会：TryComputeDualLosFixM → 残差门 residual_gate_m
  │   └ 【本会话改动】单星/配对失败 → emit_single_satellite_keepalive()
  │        用「观测方向 + last_fix_range_m 缓存距离」合成位置，key=target_id
  ├ fusion::FusionEngine::Update(records, cycle) → FusedTarget[]
  ├ 逐航迹发 SbirsFusionDpTrackRecord（TrackKey / UnitID / LastUpdateCycle / HasKinematic…）
  └ TargetInferenceEngine::Infer(tracks) → 弹道外推（waypoints/落点/关机点/发射点）
        ⚠ 目前只写验收日志，**未回喂给引导**
```

**关键语义（易错）**
- `TrackKey` = `fusion::FusedTarget.key`，**上游带身份时等于场景目标 ID**；身份丢失时引擎会**合成新键**。
- `UnitID` = **DP 宿主单元**（本场景恒 104 = 指挥所-2），**不可用于目标匹配**（已实测证伪）。
- `HasKinematic=0` 的航迹位置/速度无效，消费侧必须跳过。
- `max_missed_cycles` 默认 5；配置里 `max_miss_before_lost` 已是 418、`max_lost_cycles` 2090。

---

## 6. 当前配置现状（改之前先看这张表）

| 文件 | 项 | 现值 | 备份 |
|---|---|---|---|
| `config\sbirs.json` | `background_radiance_w_sr_m2` | **1.2** | `.bak` |
| | `error_model.{orbit,attitude,fov}_sigma_deg` | **5e-5 / 1e-4 / 5e-5** | `.bak2` |
| | `tracking.process_noise_diff_coeff` | **20.0** | `.bak3` |
| | `detection.{wide,narrow}_min_snr_linear` | 0.001 / 0.001（**无物理依据，待重标定**） | — |
| `config\remote_identification_radar.json` | `peak_power_w` | **4.8e6** | `.bak-power` |
| | `recognition.accumulation_window_sec` / `result_hold_sec` | **60 / 120** | `.bak-win` |
| | `recognition.acceptance_score` / `minimum_margin` | 0.6 / 0.05 | — |
| `config\target_feature_database_v1.1.db` | 弹道四类 | SRBM/MRBM/IRBM/ICBM | `.bak-four-class` |
| | ICBM 高度 | **1 500 000 ± 900 000** | `.bak-alt` |
| `config\remote_identification_radar_target_features.json` | 弹道 `aspect_rcs.rows` | **171 行**（az −180…180 step20） | `.bak-aspect` |

**四类弹道参数（DB）**

| model | v (m/s) | h (m) | RCS (dBsm) |
|---|---|---|---|
| SRBM | 1500±400 | 150k±70k | −6±2.5 |
| MRBM | 3000±500 | 450k±200k | −8±3 |
| IRBM | 4300±600 | 900k±300k | −9.5±3.5 |
| ICBM | 5000±800 | **1500k±900k** | −10±4.5 |

---

## 7. 已证伪的假设（别再试）

| 假设 | 结论 |
|---|---|
| 引导键用 `UnitID` | ❌ 实测把雷达指向地面站，检测归零 |
| 单纯提功率就能识别 ICBM | ❌ 功率够（SNR 16 dB）但特征掩码仍为 8/10 |
| 识别窗口停在早期也能判对 | ❌ 145 km/2.66 km/s 与 MRBM 不可分，物理限制 |
| 加卫星/改雷达位置能解决调度 | ❌ 星座已 60° 均匀无盲区；瓶颈是"每星窄视场只锁一个目标" |
| 把辐射强度统一调高到 1e5–1e6 | ❌ 只对助推段成立；中段 RV 仅 ~1 W/sr |
| 在协调器里加成员实现角速率 | ❌ 堆损坏 0xC0000374 |
| 改完库只重编模块库就够 | ❌ 测试链接**聚合库** `Release\lib\1q.lib`，必须一并重建 |

---

## 8. 复现步骤（构建/测试/部署）

### 8.1 BOM（每次用编辑工具改完 1q 源码后必做）

```powershell
cd C:\software\workspace\1qfosim\1q
python scripts\utf8_bom.py convert
```
> `edit` 工具会剥掉 BOM，MSVC 会按 GBK 解析 UTF-8 中文注释，导致**编译报奇怪的 C2059**。

### 8.2 v141 开发构建（测试用）

```powershell
$mb='C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise\MSBuild\15.0\Bin\MSBuild.exe'
$a=@('/p:Configuration=Release','/p:Platform=x64','/p:BuildProjectReferences=false','/m','/v:minimal','/nologo')
& $mb build\VisualStudio.15.0-amd64\src\sbirs_sensor\sbirs_engine.vcxproj @a
& $mb build\VisualStudio.15.0-amd64\src\1q_lib.vcxproj @a                       # 聚合库，必须重建
& $mb build\VisualStudio.15.0-amd64\tests\1q_sbirs_sensor_unit_tests.vcxproj /t:Rebuild @a
& 'build\VisualStudio.15.0-amd64\Release\bin\1q_sbirs_sensor_unit_tests.exe'
```
> ⚠ 若曾强杀过编译进程，会留下 0 字节 obj → `LNK1136`。扫描并删除：
> `Get-ChildItem build\... -Recurse -Filter *.obj | Where-Object { $_.Length -lt 64 } | Remove-Item`

### 8.3 v120 集成库 + 部署（宿主必须关闭）

```powershell
$mb12='C:\Program Files (x86)\MSBuild\12.0\Bin\MSBuild.exe'
& $mb12 build\acceptance-v120-fosim\src\sbirs_sensor\sbirs_engine.vcxproj @a
& $mb12 build\acceptance-v120-fosim\src\1q_lib.vcxproj @a
Copy-Item build\acceptance-v120-fosim\Release\lib\1q.lib ...\BJ_10Y_WorkSpace\ModelSource\lib\x64\Release\1q.lib -Force
Get-ChildItem ...\bin_x64_Release -Filter 'Sensors.*'        | Remove-Item -Force
Get-ChildItem ...\bin_x64_Release -Filter 'DataProcessors.*' | Remove-Item -Force
cd ...\BJ_10Y_WorkSpace; cmd /c "ModelSource\build_x64_v120.cmd"
```

### 8.4 日志位置与判读

| 日志 | 内容 |
|---|---|
| `FOSimStudio\log\IRInternalInterface.log` | 第 74/75/76/77 项 |
| `FOSimStudio\log\IRExternalInterface.log` | 第 58/67 项 |
| `FOSimStudio\log\RadarOutput.log` | 第 29/40/42/44/46/47A |
| `FOSimStudio\log\RadarExternalInterface.log` | 第 62/63/64/65/66 |
| `FOSimStudio\bin\App\lrdr_rir_diag_*.log` | `[cue] / [cue-track] / [view] / [DIAG-recog]` |
| `FOSimStudio\bin\App\sbirsfusion_diag_*.log` | `[verbose]`（fix_records/tracks/drop_stage/targets_multi_sat）|

> 查 DLL 内字符串：**1q 源码带 BOM → 字面量是 GBK；集成源码无 BOM → 字面量是 UTF-8**，两种解码都要试。
> 正则里 `[cue]` 是字符类，匹配字面量必须用 `-SimpleMatch`。

---

## 9. 专题文档索引

| 文档 | 内容 |
|---|---|
| `docs/review/acceptance_log_physical_audit_2026-10-19.md` | 验收日志不合理值审计（S1×13 / S2×9 / S3×6）|
| `docs/review/acceptance_log_fix_plan_2026-10-19.md` | A/B/C 三批修改方案（**未实施**）|
| `docs/practice/integration_boundary_1q_vs_host_2026-10-19.md` | 集成边界：什么取 1q、什么留宿主、4 类漂移、4 个 API 缺口 |
| `docs/review/rir_recognition_assessment_2026-10-16.md` | RIR 识别评估 §3p.1–3p.4.10（四类库、引导修复全过程、模板标定）|
| `docs/review/ir_interface_log_review_2026-10-17.md` | 第 58/67/74/75/76/77 项字段覆盖结论 |
| `docs/practice/integration_experience.md` | 库面缺位历史经验（F1/F2…）|
| `docs/practice/diagnostics_inventory.md` | 诊断标记清理清单 |

---

## 10. 下一步建议顺序

1. **A1 角速率在 `SbirsPipeline` 侧实现**（§3.1），跑 281 项测试确认无回归；
2. **批次 1 剩余 A2/A3/A4**（日志口径 + 极化诊断）；
3. **批次 2 噪声模型 + C1 辐射强度**；出来数据后**决策 4.8 MW 是否回退**；
4. **批次 3 门限重标定**（必须紧跟批次 2）；
5. 引导容限期（§4 第 10/11 项）——这一步决定中段观测能否连续，是识别能否落到 ICBM 的前提；
6. 集成迁移准备（§4 第 13 项）。
