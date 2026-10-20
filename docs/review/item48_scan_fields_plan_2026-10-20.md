# 第 48 项「波束扫描功能测试」输出改造方案（2026-10-20）

Status: **已实施**（2026-10-20，1q `35e8d8ba` / BJ `0cde9a1`）——评审结论：R1 文档同步已做、
R2 按转台朝向、R4 可反算量不落日志（实施时只回显 8 项静态配置）
范围：`src/remote_identification_radar/runtime/RirAcceptanceRecords.{h,cpp}`、`src/remote_identification_radar/session/RirSession.cpp`
依据：`docs/review/functional_acceptance_test_criteria_ocr.md` 第 48 项、
`include/1q/foundation/scan_schedule_types.h`、`include/1q/remote_identification_radar/config/RirMissionConfig.h`

---

## 0. 需求（用户 2026-10-20 提出）

1. **删除** 第 48 项驻留行里的 `目标ID=`；
2. **增加**：阵面方位朝向、阵面俯仰朝向、扫描起始象限 `kLeftTop(0)`、二维扫描推进顺序
   `kAzimuthFirst(0)`、波位步长系数，以及"等等"（本方案给出完整建议清单）。

---

## 1. 现状

第 48 项现在有三类行：

| 行型 | 频率 | 实测（14:46 轮） | 内容 |
|---|---|---|---|
| **扫描范围摘要** | **每轮 1 行**（`acceptance_scan_pattern_logged` 一次性守卫） | 1 行 | `方位扫描范围=140.000/155.000° 俯仰扫描范围=5.000/55.000° 波位排列表已输出至 …csv` |
| **搜索波位序号行** | 每个搜索驻留 1 行 | — | `本周期序号=N 波位排列表已输出至 …csv` |
| **驻留种类行** | 每个非搜索驻留 1 行 | 外部跟踪 37,717 + 指定 36 + 库内跟踪 58 | `驻留种类=外部跟踪驻留 目标ID=101` |

**问题**：`目标ID` 来自**外部 cue 或指定决策**，不是探测结果；而该行在"排波位时"就写，
所以"外部引导到位"与"雷达扫到目标"在日志里长得一样（37,811 行驻留 vs 仅 67 条检测判决）。

---

## 2. 设计决定（需你确认）

| # | 决定 | 理由 |
|---|---|---|
| **D1** | 静态配置**只回显一次**（挂在"扫描范围摘要"那一行），不逐驻留重复 | 现 37,811 行若每行加 ~120 字节，日志会长约 4.5 MB/轮；而这些配置整轮不变 |
| **D2** | **删除 `目标ID`**；归因交给第 66 项「波束任务队列」——它已按 `种类@(az,el)deg/目标ID` 输出（本轮实测：`外部跟踪@(-169.995,0.012)deg/目标101`） | 第 48 项本来只回答"扫了哪些波位"；把"为谁扫"留在 66 项，避免两个位置都写、口径不一致 |
| **D3** | 驻留行删 ID 后**补 `指向=(az,el)°`**（建议） | 否则驻留行只剩 `驻留种类=外部跟踪驻留` 一个词，几乎不含信息；指向本就在 `dwell_plan` 里，零成本 |

---

## 3. 新增字段清单（回显行）

取值全部来自**当前生效配置**（运行期 patch 后应反映新值），单位与库内口径一致：

| # | 日志字段 | 取值来源 | 单位 / 编码 | 本轮示例 |
|---|---|---|---|---|
| 1 | `阵面方位朝向` | `mission.scan_center_deg.az_deg` | deg（自东逆时针） | `147.500` |
| 2 | `阵面俯仰朝向` | `mission.scan_center_deg.el_deg` | deg | `0.000` |
| 3 | `可扫描方位范围` | `hardware.orientation.az_min_deg / az_max_deg` | deg（**相对朝向**） | `-110.000/130.000` |
| 4 | `可扫描俯仰范围` | `hardware.orientation.el_min_deg / el_max_deg` | deg（**绝对**） | `0.000/85.000` |
| 5 | `任务扫描子窗` | `mission.scan_window_deg` | deg（az 相对 / el 绝对） | `az[-7.500,7.500]el[5.000,55.000]` |
| 6 | `扫描起始象限` | `mission.scan_start_position` | `枚举名(数值)` | `kLeftTop(0)` |
| 7 | `二维扫描推进顺序` | `mission.scan_sequence` | `枚举名(数值)` | `kAzimuthFirst(0)` |
| 8 | `波位步长系数` | `mission.step_scale` | 无量纲 | `1.000` |
| 9 | `波位间距` | `step_scale × 有效波束宽度` | deg | `0.400/0.400` |
| 10 | `波位总数` | `scan_pattern.size()`（与 CSV 同源实算） | 个 | `4750` |
| 11 | `单帧扫描时间` | `波位总数 × mission.recognition_dwell_sec` | s | `237.500` |

> 第 9–11 项是我建议的"等等"：它们是**反算量**，能自证"配置 → 波位表 → 扫描节奏"三者一致，
> 也是判断"自主搜索能否在目标穿越前覆盖"的唯一日志依据（见 `round_analysis_2026-10-20_1006.md` §7.1）。
> 如不需要可去掉 9–11。

### 3.1 枚举映射（新增两个文本函数）

`oneq::foundation::ScanStartPosition`（`scan_schedule_types.h:17-22`）：

| 值 | 名称 |
|---|---|
| 0 | `kLeftTop` |
| 1 | `kRightTop` |
| 2 | `kRightBottom` |
| 3 | `kLeftBottom` |

`oneq::foundation::ScanSequence`（同文件 `:27-29`）：

| 值 | 名称 |
|---|---|
| 0 | `kAzimuthFirst` |
| 1 | `kElevationFirst` |

输出格式统一 `名称(数值)`，例如 `kLeftTop(0)`——既人读又机器可解析，未知值输出 `未知(-1)`。

---

## 4. 改造后的输出样例

```text
# ① 每轮一次：扫描范围 + 配置回显（新增 11 项）
[波束扫描功能测试] 雷达ID=103 方位扫描范围=140.000/155.000° 俯仰扫描范围=5.000/55.000° \
  阵面方位朝向=147.500° 阵面俯仰朝向=0.000° \
  可扫描方位范围=-110.000/130.000°(相对) 可扫描俯仰范围=0.000/85.000°(绝对) \
  任务扫描子窗=az[-7.500,7.500]el[5.000,55.000]° \
  扫描起始象限=kLeftTop(0) 二维扫描推进顺序=kAzimuthFirst(0) \
  波位步长系数=1.000 波位间距=0.400/0.400° 波位总数=4750 单帧扫描时间=237.500s \
  波位排列表已输出至log/Rir_scan_pattern.csv

# ② 每个搜索驻留一行（不变）
[波束扫描功能测试] 雷达ID=103 本周期序号=0 波位排列表已输出至log/Rir_scan_pattern.csv

# ③ 每个非搜索驻留一行：删 目标ID，补 指向
[波束扫描功能测试] 雷达ID=103 驻留种类=外部跟踪驻留 指向=(-169.995,0.012)°
```

**行数不变**（除摘要行每轮多约 200 字节）。

---

## 5. 代码改动清单

| 文件 | 函数 | 改动 |
|---|---|---|
| `RirAcceptanceRecords.h` | `WriteRirScanRangeSummary` | 增加两个入参：`const config::RirMissionConfig& mission`、`const config::RirHardwareConfig& hardware` |
| `RirAcceptanceRecords.cpp` | `WriteRirScanRangeSummary` | 追加 §3 的 11 个字段；新增 `ScanStartPositionText` / `ScanSequenceText` 两个映射函数（放在现有 `DwellKindText` 旁） |
| `RirAcceptanceRecords.cpp` | `WriteRirDwellScan` | 驻留行删除 `目标ID`，追加 `指向=(az,el)` |
| `RirSession.cpp:747` | 调用点 | 传入 `config.mission` / `config.hardware` |

无新增文件、无 CMake/vcxproj 变更、无回放格式变更。

---

## 6. 回归与验收

| 项 | 判据 |
|---|---|
| 数值不变 | 除第 48 项外，其余五项日志逐位不变 |
| 摘要行 | 11 个新字段齐全，且 `方位/俯仰扫描范围` 与 `任务扫描子窗` 不矛盾（前者是实算波位表上下限，后者是配置输入） |
| 反算自洽 | `波位间距 = 步长系数 × 波束宽度`；`单帧扫描时间 = 波位总数 × 0.05 s` |
| 驻留行 | 不再出现 `目标ID=`；`指向=` 与第 66 项队列同周期的波位坐标一致 |
| 归因可追 | 第 66 项仍能按 `/目标101` 归因（已有能力） |

---

## 7. 风险与待确认

| # | 事项 | 影响 / 建议 |
|---|---|---|
| **R1** | 删除 `目标ID` 与判定标准第 48 项示例（`驻留种类=跟踪驻留 目标ID=1001`）**不一致** | 需同步改 `functional_acceptance_test_criteria_ocr.md` 第 48 项示例，并在验收报告注明"目标归因见第 66 项" |
| **R2** | `阵面方位/俯仰朝向` 打的是 `scan_center_deg`（转台朝向）还是 `antenna.boresight_offset_deg`（安装偏置）？ | 本方案按**转台朝向**（`scan_center_deg`）理解；若你要的是"阵面在弹体/平台坐标系里的安装朝向"，那是 `boresight_offset_deg`，字段名需改为"阵面安装方位/俯仰偏置" |
| **R3** | 摘要行目前是**每轮一次**（`acceptance_scan_pattern_logged` 守卫，实测整轮仅 1 行） | 若运行期 patch 改了 `scan_center`，回显不会更新；如需反映，改成"配置变化时补写一行" |
| **R4** | 第 9–11 项（波位间距/总数/单帧时间）是否保留 | 保留可自证扫描节奏；如只要"输入回显"，可去掉 |

---

## 8. 实施顺序（确认后）

1. `WriteRirScanRangeSummary` 增参 + 11 字段 + 两个枚举映射（约 80 行）；
2. `WriteRirDwellScan` 删 ID 补指向（约 5 行）；
3. 改调用点（2 行）；
4. 同步判定标准第 48 项示例；
5. v141 编译 + RIR 单测（基线 290 通过）+ v120 编译 + 部署；
6. 重跑一轮，核 §6 的四条判据。
