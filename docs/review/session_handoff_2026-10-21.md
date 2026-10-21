# 会话交接（2026-10-21）

Status: handoff
用途：新会话继承本轮结论、未完成工作与可复跑命令。**先读本文，再读 §6 的专题文档。**

---

## 1. 本会话做了什么（按提交）

### 1q 仓库（`fosim_2013`）

| 提交 | 内容 | 状态 |
|---|---|---|
| `252c04df` | **B8 修复**：第 76 项 `剩余角` 改为 **实际指向 − 指令指向**（实际指向含扰动/settle 误差）；`候选状态` 随指向稳定状态（转动中 → `指向未稳定`） | ✅ 已验证（286/286 单测；现场 180 行变为 `指向未稳定`） |
| `d89354a4` / `b213cf7b` | 第 48 项：`本周期序号` → **`波位序号`**（=波位表游标），每周期新增 **`本周期搜索驻留数=N`** | ✅ 现场生效 |
| `b3b43d1d` | 文档更正：阿拉斯加扫描窗波位 **4,914**（原写 4,750）、单帧 **245.7 s**（原 238 s） | ✅ |
| `8784b985` / `7a113def` | 工具 `tools/acceptance_item48_dwell_check.py`：逐周期核对第 48 项驻留数 vs 第 37 项调度（含四分类），退出码即判据 | ✅ 本轮实测全一致 |
| `fd6f36a9` | `PrecisionEvaluationInputs.h`：检测级输入 → `SbirsCycleResult` 装配（未归属检测不进交会） | ✅ 43/43 |
| `996d18f5` | `PrecisionPairingSelector.h`：**可切换选星策略**（默认 `kLongestBaseline`，`kFixedSources` 一行切回固定 A/B） | ✅ |
| `1af2eac2` | `DebugLatestErrorCount()` 只读诊断访问器 | ✅ |
| `d7efb412` / `6e2d22a2` / `a1cbbbb1` / `2fcfd9e0` | 第 26/27 项写盘尝试：无闸门接口 → 直达通道 → 借道推演通道（头文件内联） | ⚠ 全部无效，根因见 §3；**当晚已按 §3 定案修复**（精度模块直写通道 + 回归用例） |

### BJ 宿主仓库（`master`）

| 提交 | 内容 |
|---|---|
| `9991af6` | 融合 DP 挂 `PrecisionEvaluationSession`；逐周期选星 + `Step`；每 300 周期 `Summarize()`；OPIR 透传焦平面脱靶量与**目标真值 ECEF**；检测记录加 `HasTruthPosition/TruthEcefXm/Ym/Zm` |
| `02b41ac` | 汇总时打印 `[precision] summary: … ahp_valid=… angles=… dualsat=… errors=… pair=… baseline_km=…` |

---

## 2. 本轮实测结论（10-21 17:16 / 17:54 两轮）

| 项 | 结论 |
|---|---|
| 第 48 项 ↔ 第 37 项驻留数 | **完全一致**：73,029 = 73,029；指定 5、外部 8,023、库内 1,903 逐项相等；每周期恒 20；波位表 4,914 |
| 第 76 项 `候选状态` | ✅ 已分层（17,562 `已捕获跟踪` + 180 `指向未稳定`） |
| 第 76 项 `剩余角` | 98.9% 仍为 0 —— **因部署配置指向扰动/settle 误差都是 0**（`pointing.channel_pointing_sigma_deg`、`narrow_pointing_settle_error_deg`）。要非零读数需配置物理量级 |
| 第 26/27 项 | ❌ **0 行**（会话跑通：`ahp_valid=1`、角度样本 2,386、双星交会 959、`errors=6`；写盘不落地） |
| SBIRS 跟踪/识别 | 稳定段为 `NFOV跟踪`（28,133 行 + 1,629 行滑行）；识别"弹道目标"仅周期 852–2333；6 颗星接力（85 次集合切换），目标 101 覆盖周期 69–2,971（助推→中段→再入全程） |
| 融合出点时间 | 首帧 46 → 首条融合航迹 69 ⇒ **冷启动 23 s**；稳定后 95.2% 周期每周期出定位（1 s） |

---

## 3. 第 26/27 项写盘（10-21 晚已定案并修复，待下轮现场复核行数）

### 3.1 根因（已证实，原判定"宏闸门已排除"是错的）

精度模块借用了推演模块的**头文件内联**包装 `target_inference::WriteSatelliteTargetAcceptanceLine`，
而那个内联体的宏闸门 `ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG` 只由
`src/target_inference/CMakeLists.txt` 加在 `target_inference_core` 上（`PRIVATE`），
**在 `precision_evaluation_core` 的编译单元里恒为关** ⇒ 两个写点被编译成 `((void)0)`。
跨模块"借"来的不只是函数，还借来了对模块编译选项求值的宏——所以无论会话、AHP、样本多正常都是 0 行。

证据（同一库内 A/B 对照）：

| 对象（从交付 `1q.lib` 抽出） | `WriteAcceptanceLog` / `FormatAcceptanceLine` 引用 |
|---|---|
| `InferenceAcceptanceRecords.obj`（推演，宏=1） | 有 |
| `PrecisionAcceptanceRecords.obj`（精度，宏未定义） | **无** |

辅证：生成的 `precision_evaluation_core.vcxproj` 只带 `ONEQ_ENABLE_PRECISION_EVALUATION_LOG=1`，
不带 `ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG`；会话侧一切正常（17:53 现场 `ahp_valid=1`、
`angles=1235…4783`、`dualsat=482…1766`、`errors=4…6`）。

### 3.2 修复

- `src/precision_evaluation/PrecisionAcceptanceRecords.cpp`：改为本模块直接写 SatelliteTarget 通道
  （`oneq::logging::WriteAcceptanceLog` + `FormatAcceptanceLine`），不再跨模块借用；
  行格式、字段口径、`等级`/权重格式均未变。
- `src/target_inference/InferenceAcceptanceRecords.h`：删掉借用包装及其顺带引入的
  `InferenceAcceptanceLog.h`、`<string>`（该头不再承担别家写盘职责）。
- 新增回归用例 `tests/unit/precision_evaluation/precision_acceptance_records_test.cpp`：
  打开通道 → 调用两个写接口 → 回读断言两行存在、字段齐全、AHP 非法时整行省略。
  修复前该用例必然红（文件为空），是本问题的守门用例。

### 3.3 §3.1 旧口径的两处纠正

1. 旧命令 `1q_lib.vcxproj /p:BuildProjectReferences=false` 在**对象目录被清空**时救不回来——
   1q_lib 按对象文件归档 20+ 个模块工程的 `.obj`，必须编整棵 `1q.sln`（或
   `cmake --build build/acceptance-v120-fosim --config Release --target 1q_lib`，已含工程引用）。
2. `build\acceptance-v120-fosim\sqlite3_vendor.dir\Release\sqlite3.obj` 是 17:55 清理留下的
   **0 字节残留**，归档直接报 `LNK1136 无效或损坏的文件`——这也是"1q.lib 删掉后一直没再产出"的
   直接原因。删掉该对象目录重编即可（已处置）。

### 3.4 本轮验证

- v141 单测：precision / target_inference 全绿（新增 2 例通过）；
- v120 交付库重建：`build\acceptance-v120-fosim\Release\lib\1q.lib` 295,322,568 B（21:30:36），
  抽 obj 已引用 `WriteAcceptanceLog`；
- 部署：已拷到宿主 `ModelSource\lib\x64\Release\1q.lib`，
  `build_x64_v120.cmd` 0 错误产出 `Sensors.dll` / `DataProcessors.dll`（21:30）；
- **尚待现场**：下一轮跑完确认 `SatelliteTarget.log` 出现
  `[关键精度指标提取功能测试]`、`[层次分析法功能测试]` 行，并复核数值口径（§4 第 2 条）。

---

## 4. 下一步建议顺序

1. ~~恢复 `1q.lib` 构建 + 符号检查~~（已完成，见 §3）；下轮跑完先数第 26/27 项行数；
2. 核对数值：角度误差 ~1e-3°、双星交会误差 ≤ 参考值 10 km、脱靶量 m 量级；
3. 第 76 项：按验收要求把指向扰动/settle 误差配置成物理量级，再复核 `剩余角` 分布；
4. 精度评估的**正式配置**：AHP 判断矩阵与五项参考误差目前是演示口径（`PrecisionEvaluationConfig` 注释已写明），验收前按装备指标替换；
5. 第 37A 项判定标准示例仍写三分类（`跟踪=0`），与第 37 项四分类不一致，可顺手同步；
6. 卫星侧 11 个零行项（1/2/3/4/18/19A/24A/24B/24C/26/27）：26/27 本轮已修（§3），其余仍未输出——
   排查口径同本轮：先确认**该模块自己的编译单元**里有写点引用，再看通道。

---

## 5. 口径与命令备忘

```bash
# 1q 构建（v141 单测 / v120 交付）
powershell scripts\win\build_rir_libs.ps1            # rir_engine + rir_core 两个工程 + 聚合库 1q_lib
# 改过任一模块 *_core 后必须重编对应工程再做 1q_lib（1q_lib 按对象文件归档，不吃工程引用）
# 改完 1q 源码必做 BOM 转换（否则 MSVC 按 GBK 解析中文注释）
python scripts\utf8_bom.py convert
```

- 宿主重建：`ModelSource\build_x64_v120.cmd`（需宿主进程关闭）；
- 第 48 项核对：`python tools/acceptance_item48_dwell_check.py`（退出码 1 = 不一致）；
- 验收日志：`FOSimStudio\log\{SatelliteTarget,RadarOutput,RadarExternalInterface,RadarInternalInterface,IRExternalInterface,IRInternalInterface}.log` + `Rir_scan_pattern.csv`；
- 融合诊断：`FOSimStudio\bin\App\sbirsfusion_diag_*.log`（`[precision] summary:` 可判读数）。

### 5.1 顺带发现（本轮未改，供排期）

1. **验收日志行被并发写撕裂**：`SatelliteTarget.log` 350,250 行里有 25 行是两行粘连或半字符截断
   （例：`仿真周期=74 [宽窄视场联合探测功能测试] … 信仿真时间=74.000s …`）。原因是每个模块 DLL
   各自静态链接一份 `AcceptanceFileLog.cpp`，各有独立 `ChannelSink`/ofstream 写同一路径，
   跨模块只防御了"首开截断"、没防御写交错。整改方向：单写者（host 侧统一 sink）或行级两段写改一次 `write`。
2. **`unit::remote_identification_radar` 在 HEAD 上红**：7 条面杂波模型用例返回 0 / −inf
   （`RirSurfaceClutterModelTest.*`、`RirRfPhysicalParityTest.BallisticPodJammerAndSurfaceClutterShareDetectionBudget`），
   与本轮精度/推演改动无关（改动只碰 precision_evaluation + target_inference 头），属既有红。
3. 开发树 `build\VisualStudio.15.0-amd64` 现由 **CMake 4.3.3 + Visual Studio 18 2026 生成（`1q.slnx`、工具集 v141）**，
   该树 `ONEQ_ENABLE_PRECISION_EVALUATION_LOG=OFF`；交付树 `acceptance-v120-fosim` 仍是 VS14 生成器 + v120、六个验收开关全 ON。
   重建任何一套都以 `cmake --build <tree> --config Release --target <target>` 为准（两棵树共用同一份源码目录）。

---

## 6. 专题文档索引

| 文档 | 内容 |
|---|---|
| `docs/review/cue_geometry_check_2026-10-20.md` | 外部引导几何核查（cue 位置跳变 / 速度恒 0） |
| `docs/review/round_analysis_2026-10-20_1006.md` | 阿拉斯加部署首轮分析（B6/B7/去重/扫描窗） |
| `docs/review/item48_scan_fields_plan_2026-10-20.md` | 第 48 项字段改造（已实施） |
| `docs/review/todo_item29_power_evidence_2026-10-19.md` | T-01…T-04 日志口径待办与状态 |
| `docs/review/functional_acceptance_test_criteria_ocr.md` | 判定标准主文档（第 48/76 项本轮已同步口径） |
| `docs/review/session_handoff_2026-10-19.md` | 上一份交接（已过期，仅保留集成链路与已证伪清单） |
