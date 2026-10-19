# 宿主/测试工程使用指南（LRDR + SBIRS + 地面融合站）

> 面向需要**跑仿真、改集成代码、看日志**的人（含下一个会话）。
> 2026-10-15 编写。仅描述实际使用的这一套宿主与工程，不涉及其它宿主。

---

## 1. 测试工程是什么

| 角色 | 名称 | 绝对路径 |
|---|---|---|
| **宿主（仿真支撑软件）** | `仿真支撑软件.exe` | `C:\software\workspace\1qfosim\FOSimStudio\bin\App\仿真支撑软件.exe` |
| **模型工程（改代码的地方，git 仓库）** | `FOSimWorkSpace.wsp` | `C:\software\workspace\1qfosim\BJ_10Y_WorkSpace\` |
| **模型库源码（1q，另一个 git 仓库）** | 1q static library | `C:\software\workspace\1qfosim\1q\` |
| 场景编辑器 / 运行器（宿主自带） | `CyScenEditorRunner.exe` / `CySimulatorRunner.exe` / `CySituationRunner.exe` / `CyberStandaloneEngine.exe` | 与宿主同目录 |

> ⚠️ **宿主的工作目录是 `C:\software\workspace\1qfosim\FOSimStudio`**（不是 `bin\App`）。
> 所有**相对路径**（尤其配置里的 `acceptance_log_output_directory`）都相对这个目录解析。

**模型工程 BJ_10Y_WorkSpace 的顶层：**

```
ModelSource\      模型 C++ 源码 + 工程文件（.vcxproj）+ 构建脚本 + 产物 bin_x64_Release\
ModelDatabase\    型号数据库：.dscript（单元编成）+ Equipment\...\*.mdata（实例参数）
ScenarioSource\   想定（每个想定一个子目录，内含同名 .sdata）
Project\          宿主工程文件
AssessmentSource\ 效能评估
ExprimentDesigner\实验设计
doc\ csv杀伤区表格\
FOSimWorkSpace.wsp  工作区文件（工程入口）
```

---

## 2. 测试工程的数据存在哪

### 2.1 型号数据（"编成"与"实例参数"）

| 内容 | 位置 | 说明 |
|---|---|---|
| **单元编成脚本** `.dscript` | `ModelDatabase\*.dscript`（如 `远程识别雷达.dscript`、`OPIR卫星.dscript`、`红外地面融合站.dscript`、`干扰机.dscript`、`弹道弹.dscript`） | 写 "这个单元由哪些平台/传感器/处理器拼成"：`EMPLOY PLATFORM/SENSOR/DATAPROCESSOR/JAMMER "…";` |
| **模型实例参数** `.mdata` | `ModelDatabase\Equipment\<类别>\<文件夹>\<实例名>.mdata`（例：`ModelDatabase\Equipment\DataProcessors\红外地面站.mdata`） | 型号库里**具体那一个**模型实例的参数（可在宿主界面里改） |
| **参数 schema / 界面** `.mdef` / `.mui` | 与模型源码同目录（如 `ModelSource\CyModel\Equipment\Sensors\LRDRJammer\LRDRJammer.mdef`） | 定义参数类型与编辑器控件；`.mdata` 必须与它们一致 |
| **型号库索引** | `ModelDatabase\database.dbpath` | 宿主加载型号库的入口 |

### 2.2 运行期配置（JSON）

**运行期真正读取的是宿主侧副本**，源副本在模型目录里（改完要同步）。
同步与自检统一走 `BJ_10Y_WorkSpace\ModelSource\sync_config_check.cmd`：
无参数 = 逐字节比三对文件（漂移则非 0 退出），`sync` = 源副本 → 部署副本（需先关宿主）。

| 模型 | 运行期读取（生效） | 源副本（提交用） |
|---|---|---|
| LRDR 雷达 | `FOSimStudio\bin\App\config\remote_identification_radar.json`（另有一份同名嵌套副本 `…\config\remote_identification_radar\remote_identification_radar.json`） | `ModelSource\CyModel\Equipment\Sensors\LRDR_SENSOR\remote_identification_radar.json` |
| 雷达识别特征真值表 | `FOSimStudio\bin\App\config\remote_identification_radar_target_features.json` | 同目录源副本 |
| 雷达识别库（SQLite） | `FOSimStudio\bin\App\config\target_feature_database_v1.1.db` | — |
| SBIRS 卫星 | `FOSimStudio\bin\App\config\sbirs.json` | `ModelSource\CyModel\Equipment\Sensors\OPIR_SENSOR\sbirs.json` |

> 诊断日志里会打印实际读取路径，例：
> `[init] resolved_config='C:\…\FOSimStudio\bin\App\config\remote_identification_radar.json' loaded=1`

### 2.3 想定放在哪

```
ScenarioSource\<想定名>\<想定名>.sdata          ← 本工程用的是：
ScenarioSource\20261006-LRDR和SBIRS测试想定\20261006-LRDR和SBIRS测试想定.sdata
```
- `.sdata` 是 XML：`<Unit>` 列表（每个单元有 `<Name>`、`<TypeOfUnit>`、`<Disposition>`（经纬高）、`<UnitScript>`、`<Dictionary>`）
- **单元行为写在 `<UnitScript>` 里**（`INCLUDE "<型号>.dscript"; LOCATE …; ASSIGN …; START 0;`），改想定优先改这里
- 想定目录里可能还有 `.sdata` 之外的配套文件，一并保留

---

## 3. LRDR / SBIRS / 地面融合站的集成代码在哪

**两处集成**：① 1q 静态库（算法）；② BJ 模型工程里的"胶水"模型（把 1q 接到宿主引擎上）。

| 模型 | 胶水代码（BJ，要改的是这些） | 关键文件 |
|---|---|---|
| **LRDR 远程识别雷达** | `ModelSource\CyModel\Equipment\Sensors\LRDR_SENSOR\` | **`lrdr_sensor_1q.cpp`**（主：场景装配、驱动 1q 会话、诊断）；`lrdr_sensor_features.{h,cpp}`（**识别特征选档**）；`LRDR_SENSORData.hpp`（参数与序列化） |
| **SBIRS 红外预警卫星** | `ModelSource\CyModel\Equipment\Sensors\OPIR_SENSOR\` | **`opir_sensor_1q.cpp`**（主：驱动 1q SBIRS 会话、把星上探测帧推给融合处理器） |
| **地面融合站**（双星交会/融合/推演） | `ModelSource\CyModel\Equipment\DataProcessors\SBIRSFusionDp\` | **`sbirsfusiondp_1q.cpp`**（主：收帧、双星交会配对、调 `FusionEngine`，并驱动 `TargetInferenceEngine` 推演）；`SBIRSFusionDpData.hpp`（11 个可配参数） |
| 干扰机（配合 LRDR 干扰验收） | `ModelSource\CyModel\Equipment\Jammers\LRDRJammer\` | `lrdrjammer.cpp`、`LRDRJammerData.hpp` |
| **共用诊断日志** | `ModelSource\CyModel\Equipment\Sensors\1q_integration\oneq_diag_log.h` 与 `…\DataProcessors\1q_integration\oneq_diag_log.h`（**同名两份，别改错**） | 提供 `LrdrDiagLog` / `FUSION_DIAG_LOG` |
| **1q 静态库** | `ModelSource\lib\x64\Release\1q.lib`（**git-lfs 管理，约 293 MB**） | 只被 **`Sensors.vcxproj`** 与 **`DataProcessors.vcxproj`** 链接；1q 源码在 `C:\software\workspace\1qfosim\1q\` |

**数据流（一句话）**：
```
想定 .sdata → 宿主引擎 → LRDR/OPIR 胶水模型 → 1q 会话（探测/跟踪/识别）
                                   ↘ OPIR 把星上测角帧推给 地面融合站(SBIRSFusionDp)
                                   ↘ 融合站双星交会 → 融合航迹 → 推演
                                   ↘ 融合定位可经 [cue] 回灌给 LRDR 做指定识别
```

---

## 4. 怎么编译

### 4.1 编模型工程（改集成代码后必做）

```powershell
# 先关掉宿主，否则 LNK1104（DLL 被占用）
Get-Process | Where-Object { $_.ProcessName -match '仿真支撑软件' }   # 应为空

cmd /c "C:\software\workspace\1qfosim\BJ_10Y_WorkSpace\ModelSource\build_x64_v120.cmd"
```
- 工具链：**VS2013 / v120 / x64**
- 产物：`ModelSource\bin_x64_Release\{Sensors.dll, DataProcessors.dll, Jammers.dll, …}`
- 构建日志：`ModelSource\_buildtest\build_v120_last.log`（看 `0 个错误`）
- 另有 `build_x64_v141.cmd`（v141 工具链，**验收产线用 v120**）
- 收尾：`git checkout -- ModelSource/library`（构建会改写该目录，避免把噪声提交进去）

### 4.2 编 1q.lib（改 1q 源码后必做，约 3 分钟）

```powershell
cd C:\software\workspace\1qfosim\1q\build\acceptance-v120-fosim
cmd /c 'call "C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat" amd64 >nul 2>&1 && "C:\Program Files (x86)\MSBuild\12.0\Bin\MSBuild.exe" 1q.sln /t:Build /m:8 /p:Configuration=Release /p:Platform=x64 /v:m /nologo'
# 产物：…\acceptance-v120-fosim\Release\lib\1q.lib  → 复制到 →
#       BJ_10Y_WorkSpace\ModelSource\lib\x64\Release\1q.lib
# 然后按 4.1 重编集成工程
```
> 该构建目录已开启验收日志（`ONEQ_ENABLE_*_ACCEPTANCE_LOG=ON`），**不要改配置**。
> 1q 单元/集成测试走另一条路径：Git Bash 里 `scripts/1q.sh build/test VisualStudio.15.0-amd64-release`（依赖机器本地 `CMakeUserPresets.json`）。

---

## 5. 怎么运行仿真

1. 启动 `FOSimStudio\bin\App\仿真支撑软件.exe`（**工作目录务必是 `…\FOSimStudio`**）
2. 在宿主里**打开想定**：`ScenarioSource\20261006-LRDR和SBIRS测试想定\20261006-LRDR和SBIRS测试想定.sdata`
3. 运行仿真；结束/中断后引擎会打印"卸载旧想定完成！"
4. 需要重跑时：直接再运行一次即可（新日志按**时间戳**另存，不会覆盖旧诊断日志；六份验收日志会被新进程**截断重写**）

**注意**：
- 六份验收日志是**逐行 flush**的，运行期间即可读取（若发现恒为 0 字节，说明宿主还没被改过或程序未退出——见 §7 坑 4）
- 一次完整跑（到导弹再入落地）约 **2–12 分钟真实时间**，对应仿真时间 **数千秒**

---

## 6. 日志输出路径（全部）

### 6.1 验收日志（**交付判据**，六份 + 两个 CSV）

```
C:\software\workspace\1qfosim\FOSimStudio\log\
  SatelliteTarget.log            第 1–18、19A、20–27 项（含 24A/B/C）
  RadarOutput.log                第 19、21(引导段)、28–49 项（含 37A/47A/48）
  RadarExternalInterface.log     第 61–66 项
  RadarInternalInterface.log     第 70–73 项
  IRExternalInterface.log        第 57、58、67、68 项
  IRInternalInterface.log        第 74–77 项
  RadarAntennaGainPatterndata.csv  第 28 项（天线方向图）
  Rir_scan_pattern.csv             第 48 项（扫描波位表）
  仿真支撑软件.log                 宿主自身日志（启动/装载/卸载、模型加载告警）
```

> 物理文件与条目的对应关系是权威口径，见 `1q\docs\review\functional_acceptance_test_criteria_ocr.md` 附录。

### 6.2 模型诊断日志（排查问题的主要入口）

```
C:\software\workspace\1qfosim\FOSimStudio\bin\App\
  lrdr_rir_diag_<yyyyMMdd_HHmmss>.log      LRDR：  [init] [cycle] [view] [issue] [scene]
                                                   [feature-pick] [scene-vel] [cue] [designation]
  opir_sbirs_diag_<…>.log                  SBIRS： [init] [push] [cycle-result] [issue]
  sbirsfusion_diag_<…>.log                 融合站：[init] [event] [verbose] [predict]
```
每次运行**新建**一个，带时间戳，历史可追溯。

### 6.3 历史遗留路径（不要再看）

```
FOSimStudio\bin\App\log\        ← 旧落点，仅存历史文件（最后写入 2026-10-14）
```
当前六份验收日志的落点由三处共同决定，依次为：
**编译期默认**（`ONEQ_*_LOG_PATH`）→ **用户级环境变量**（`ONEQ_SATELLITE_TARGET_LOG_PATH` 等三个）→ **会话配置 `acceptance_log_output_directory`**（雷达用 `"log"`，卫星用 `""` 表示走默认解析）。

---

## 7. 本机踩过的坑（省时间）

1. **源文件必须保留 UTF-8 BOM**（`EF BB BF`）。BOM 被剥掉后 MSVC 按 GBK 解析中文注释 ⇒ 报莫名的语法错。改完用字节检查补回；仓库有 `scripts/check_source_bom.py`、`utf8_bom.py`。
2. **`PROJECT_LOG_*` 在 Windows 上默认关闭**（`ONEQ_ENABLE_FILE_LOG` 默认 OFF）⇒ **不要用项目日志做诊断**，否则永远看不到。要可见就写**验收行**或**集成侧诊断**（`oneq_diag_log.h`）。
3. **宿主不落盘模型的 `printf`/stdout** ⇒ 调试输出必须走诊断日志或验收行。
4. **验收日志已逐行 flush**；若改成缓冲，运行期读到的会是 0 字节（曾误判为"日志丢失"）。另：宿主持有文件句柄，**未退出时同名文件可能显示为上次内容或 0 字节**。
5. **编集成前必须关宿主**，否则 `LNK1104`。
6. **构建会改写 `ModelSource\library\`** ⇒ 提交前 `git checkout -- ModelSource/library`。
7. **配置有源/部署两份**：改源副本（提交用）**必须同步**部署副本（`bin\App\config\`），否则改了不生效。
   反向同理：直接在部署副本上调参，仓库里看不到，重装/换机即**静默回退**（2026-10-19 实测漂移：
   `sbirs.json` 5 项、`remote_identification_radar.json` 6 项、特征表 1032 行）。改完跑
   `ModelSource\sync_config_check.cmd` 自检，退出码非 0 就不许往下走。
8. **`1q.lib` 是静态库，被 `Sensors.dll` 与 `DataProcessors.dll` 各链一份** ⇒ 文件级 statics 有两份（历史缺陷"跨 DLL 截断"即由此而来，已修）。
9. **仿真步长当前为 1.0 s/周期**（验收行时间戳可自证）；弹道目标每周期位移约 5 km，涉及关联/外推的改动都要按此复核。
10. **`0.000m/s` 这类"恒零"值**往往意味着对应链路根本没接上（运动特征速度、干扰剩余功率等都出现过），别急着调参数，先确认数据源。

---

## 8. 最短上手路径（给下一个会话）

```
1) 读本指南第 1–6 节 → 知道东西在哪、怎么编、怎么跑、日志在哪
2) 改集成代码 → §4.1 编模型工程（v120）
3) 改 1q 源码 → §4.2 编 1q.lib → 复制 → 再 §4.1
4) 关宿主 → 重编 → 启动宿主 → 打开 20261006-LRDR和SBIRS测试想定 → 运行
5) 看 bin\App\*_diag_<新时间戳>.log 定位，看 log\*.log 出验收证据
6) 提交：1q 仓库一笔（源码）+ BJ 仓库一笔（1q.lib / 集成代码 / 配置）
```
