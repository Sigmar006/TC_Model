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
| `d7efb412` / `6e2d22a2` / `a1cbbbb1` / `2fcfd9e0` | 第 26/27 项写盘尝试：无闸门接口 → 直达通道 → 借道推演通道（头文件内联） | ⚠ **仍未落盘**，见 §3 |

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

## 3. 未完成：第 26/27 项写盘（**下个会话第一优先**）

已排除：会话未跑、AHP 非法、无样本、宏闸门、函数闸门、直达通道、借道推演通道。

### 3.1 构建现场（重要）

本轮最后一次"清理重建"把 **`build\acceptance-v120-fosim\Release\lib\1q.lib` 删掉后未重新产出**。**先恢复**：

```powershell
cd C:\software\workspace\1qfosim\1q
& 'C:\Program Files (x86)\MSBuild\2017\Enterprise\MSBuild\15.0\Bin\MSBuild.exe' `  # 路径按本机
  build\acceptance-v120-fosim\src\1q_lib.vcxproj `
  /p:Configuration=Release /p:Platform=x64 /p:BuildProjectReferences=false /m /v:minimal /nologo
```

宿主 `ModelSource\lib\x64\Release\1q.lib`（17:50:15）仍在，运行侧未坏。

### 3.2 判定步骤

1. 恢复构建后，在 `1q.lib` 里查符号 `WriteSatelliteTargetAcceptanceLine`：
   - **查不到** ⇒ 精度对象没进聚合库（对象目录曾被删，归档清单引用失效）→ 核对 `1q_lib.vcxproj` 里 `PrecisionAcceptanceRecords.obj` 的引用路径；
2. 符号在、日志仍空 ⇒ **该 DLL 里只有推演模块那份静态验收通道是活的**，需宿主启动时对精度模块显式初始化验收日志（宿主侧一次性修复）。

### 3.3 已部署但未验证的最终形态

写入器改为调用推演模块内联接口：

```cpp
target_inference::WriteSatelliteTargetAcceptanceLine("关键精度指标提取功能测试", content);
```

---

## 4. 下一步建议顺序

1. **恢复 `1q.lib` 构建 + 符号检查**（§3.1/3.2），把第 26/27 项落盘打通；
2. 打通后核对数值：角度误差 ~1e-3°、双星交会误差 ≤ 参考值 10 km、脱靶量 m 量级；
3. 第 76 项：按验收要求把指向扰动/settle 误差配置成物理量级，再复核 `剩余角` 分布；
4. 精度评估的**正式配置**：AHP 判断矩阵与五项参考误差目前是演示口径（`PrecisionEvaluationConfig` 注释已写明），验收前按装备指标替换；
5. 第 37A 项判定标准示例仍写三分类（`跟踪=0`），与第 37 项四分类不一致，可顺手同步；
6. 卫星侧 11 个零行项（1/2/3/4/18/19A/24A/24B/24C/26/27）中，26/27 由本会话接线，其余仍未输出。

---

## 5. 口径与命令备忘

```bash
# 1q 构建（v141 单测 / v120 交付）
powershell scripts\win\build_rir_libs.ps1            # RIR 三工程 + 聚合库
# SBIRS / 精度模块改动后必须重编对应 *_core 工程再做 1q_lib
# 改完 1q 源码必做 BOM 转换（否则 MSVC 按 GBK 解析中文注释）
python scripts\utf8_bom.py convert
```

- 宿主重建：`ModelSource\build_x64_v120.cmd`（需宿主进程关闭）；
- 第 48 项核对：`python tools/acceptance_item48_dwell_check.py`（退出码 1 = 不一致）；
- 验收日志：`FOSimStudio\log\{SatelliteTarget,RadarOutput,RadarExternalInterface,RadarInternalInterface,IRExternalInterface,IRInternalInterface}.log` + `Rir_scan_pattern.csv`；
- 融合诊断：`FOSimStudio\bin\App\sbirsfusion_diag_*.log`（`[precision] summary:` 可判读数）。

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
