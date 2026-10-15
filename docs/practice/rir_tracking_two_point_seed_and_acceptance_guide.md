# RIR 跟踪"速度恒 0"修复与验证指南（交接给下一个会话）

> 本文件面向**下一个会话/另一位工程师**，用于修复 BJ_10Y_WorkSpace 集成中
> LRDR 雷达**航迹滤波后验速度恒为 0** 的缺陷，并沿本机已验证的链路做验收。
> 编写时间：2026-10-15。所有结论均有实测证据，证据来源与命令都写在文中。

---

## 0. 一句话结论（先读这一段）

**根因**：雷达量测结构 `RirTrackMeasurement::velocity`（速度种子）被**刻意填零**
（契约禁止真值泄漏），新航迹因此以**零速**初值起批 → 外推位置原地不动 →
高速弹道目标每周期位移约 5 km → 关联门（马氏距离 `gate_threshold`）**拒绝匹配** →
`matched_existing_track=false` → **每周期重建航迹** → 速度永远是初值 0（死循环）。

**实测铁证**（`log/RadarOutput.log` 第 42 项「跟踪滤波功能测试」）：

```
[跟踪滤波功能测试] 航迹=1 目标ID=101 位置（ECEF）=(-2615267.2,-2640945.2,7006722.1)m 速度=(0.000,0.000,0.000)m/s
[跟踪滤波功能测试] 航迹=1 目标ID=101 估计误差指标=位置误差ECEF=(0.0,0.0,0.0)m 速度误差=(-1306.4,3594.4,3916.2)m/s
…同一周期同时存在…
[跟踪滤波功能测试] 航迹=32 目标ID=101 估计误差指标=位置误差ECEF=(-234120.9,307125.6,694306.3)m …
[跟踪滤波功能测试] 航迹=33 目标ID=101 估计误差指标=位置误差ECEF=(-14681.3,17022.2,44091.7)m …
[跟踪滤波功能测试] 航迹=34 目标ID=101 估计误差指标=位置误差ECEF=(0.0,-0.0,0.0)m …
```

- `位置误差ECEF=(0,0,0)` ⇒ 航迹位置 = 量测位置（**刚初始化的新航迹**，不是滤波输出）
- `速度误差 = 真值速度本身` ⇒ **估计速度 = 0**
- 同一周期出现航迹 32/33/34 ⇒ **每周期都在新建航迹**

**影响**：第 44 项「运动特征」无判别力；第 47A 项「综合特征识别」置信度恒 ≈0.09（判据 0.60），
型号始终"未识别" —— 两个失分点同源。

---

## 1. 要改的文件与落点

### 1.1 主改点（唯一必须改的地方）

| 项 | 内容 |
|---|---|
| 文件 | `src/remote_identification_radar/runtime/RirController.cpp` |
| 位置 | **783–791 行**，`RirTrackMeasurement built;` 的填充段 |
| 现状 | `built.velocity = Eigen::Vector3f::Zero();`（附有契约注释，见 1.2） |

### 1.2 契约约束（**必须先读，否则会做出违约改动**）

```cpp
// RirController.cpp:788-791 原文
// 去真值化（rir-tracking-realism 契约）：雷达单次检测无速度量测通道，旧口径的
// 真值速度种子属泄漏；滤波速度唯一来源为后验收敛。字段保留填零（公开面与回放
// 兼容，契约修订 1 第 4 条）。
built.velocity = Eigen::Vector3f::Zero();
```

⇒ **禁止**用场景真值速度去填 `built.velocity`（那是被契约修订 1 第 4 条明令禁止的泄漏）。
**合法修法只有"两点起批"**：用雷达**自己的**相邻两次量测做差分。

### 1.3 ⚠️ 先查这个：1q 里已有两点起批的测试

```
tests/unit/remote_identification_radar/rir_two_point_velocity_seed_test.cpp
```

**这是本次交接的第一入口。** 它说明"两点速度种子"在 1q 里**已有测试（很可能已有规格甚至实现）**。
动手前必须先读它 + 同目录下这几个：

```
tests/unit/remote_identification_radar/rir_track_associator_test.cpp
tests/unit/remote_identification_radar/rir_track_filter_test.cpp
tests/unit/remote_identification_radar/rir_track_lifecycle_test.cpp
tests/unit/remote_identification_radar/rir_imm_tracking_test.cpp
tests/integration/remote_identification_radar/rir_dual_satellite_lock_test.cpp
```

若其中已定义两点起批的 API/行为，**按它实现**，不要另起一套。

### 1.4 建议的最小实现（若测试未给出既有实现）

在 `RirController` 增加一个私有缓存成员（`std::map` 默认构造，无需改构造函数）：

```cpp
// RirController.h（私有区）
// 每目标的上一周期量测快照：外目标ID -> {位置(ENU, m), 仿真时刻(s)}
std::map<std::uint64_t, std::pair<Eigen::Vector3f, float>> last_measurement_by_target_;
```

在 `RirController.cpp` 的 `built.position = PositionOf(target);` 之后填入种子：

```cpp
const auto prev = last_measurement_by_target_.find(target.external_target_id);
const float t_now = static_cast<float>(input.sim_time_sec);
if (prev != last_measurement_by_target_.end()) {
  const float dt = t_now - prev->second.second;
  built.velocity = (dt > 1.0e-3f && std::isfinite(dt))
                       ? (built.position - prev->second.first) / dt   // 两点差分，雷达自有信息
                       : Eigen::Vector3f::Zero();
} else {
  built.velocity = Eigen::Vector3f::Zero();   // 首次检测仍填零，与现状逐位一致
}
last_measurement_by_target_[target.external_target_id] =
    std::make_pair(built.position, t_now);
```

要点：
- **不引入真值**，只用雷达自己的量测序列（两点差分是航迹起始的标准工程做法）；
- 首次检测仍为零速 ⇒ 无副作用；
- 正好覆盖死循环的触发点（第二次量测起才有种子）。

### 1.5 兜底方案（若两点起批后仍重建航迹）

给新航迹前 1~2 个周期用与速度无关的宽松门限（物理上界 `v_max·dt`）：
`src/remote_identification_radar/tracking/RirTrackAssociator.cpp:167`
```cpp
if (cost <= config_.gate_threshold) { gated_costs(s, m) = cost; }
```

---

## 2. 两条测试/验证路径

### 2.1 路径 A：1q 单元/集成测试（**规格优先，但本机未验证**）

按仓库 `CLAUDE.md` 的口径（Windows 用 Git Bash / WSL）：

```bash
scripts/1q.sh configure VisualStudio.15.0-amd64        # 一次性（会自动 bootstrap Conan 依赖）
scripts/1q.sh build     VisualStudio.15.0-amd64-release
scripts/1q.sh test      VisualStudio.15.0-amd64-release -R "unit::remote_identification_radar"
```

针对性：
```bash
scripts/1q.sh test VisualStudio.15.0-amd64-release -R "rir_two_point_velocity_seed"
scripts/1q.sh test VisualStudio.15.0-amd64-release -R "rir_track_associator"
scripts/1q.sh test VisualStudio.15.0-amd64-release -R "rir_track_lifecycle"
```

> ⚠️ **本会话没有验证过这条路径**（未在本机跑过 `scripts/1q.sh`，也未确认 VS2015 工具链与
> Conan 依赖是否齐备）。它依赖 `CMakeUserPresets.json`（机器本地、不在 git）。
> 若不可用，直接走路径 B —— 路径 B 是本会话**完整跑通并验证过**的。

### 2.2 路径 B：Windows 验收链路（**本会话已验证可用**）

这是本机唯一验证过的端到端链路，修复的最终验收必须走它。

#### 步骤 1：重编 `1q.lib`（验收配置，VS2013 / MSBuild 12.0）

```powershell
cd C:\software\workspace\1qfosim\1q\build\acceptance-v120-fosim
cmd /c 'call "C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat" amd64 >nul 2>&1 && "C:\Program Files (x86)\MSBuild\12.0\Bin\MSBuild.exe" 1q.sln /t:Build /m:8 /p:Configuration=Release /p:Platform=x64 /v:m /nologo'
```
- 产物：`…\build\acceptance-v120-fosim\Release\lib\1q.lib`（约 **293 MB**）
- 该构建目录已配置好（`ONEQ_ENABLE_*_ACCEPTANCE_LOG=ON`），**验证配置不要改**
- 该目录**没有测试目标**（测试需走路径 A）

#### 步骤 2：部署到集成工程

```powershell
Copy-Item 'C:\software\workspace\1qfosim\1q\build\acceptance-v120-fosim\Release\lib\1q.lib' `
          'C:\software\workspace\1qfosim\BJ_10Y_WorkSpace\ModelSource\lib\x64\Release\1q.lib' -Force
```
> `1q.lib` 在 BJ 仓库里由 **git-lfs** 管理（293 MB），提交前确认 lfs 已安装。

#### 步骤 3：重编集成工程（必须先关掉仿真程序，否则 LNK1104）

```powershell
# 先确认：Get-Process | Where-Object { $_.ProcessName -match '仿真支撑软件' }  —— 应为空
cmd /c "C:\software\workspace\1qfosim\BJ_10Y_WorkSpace\ModelSource\build_x64_v120.cmd"
```
- 产物：`ModelSource\bin_x64_Release\{Sensors.dll, DataProcessors.dll}`
- 构建日志：`ModelSource\_buildtest\build_v120_last.log`（看 `0 个错误`）
- 只链接 `1q.lib` 的工程是 **`Sensors.vcxproj` 与 `DataProcessors.vcxproj`**（其它工程与 1q 无关）

#### 步骤 4：跑仿真并读日志

运行 `FOSimStudio\bin\App\仿真支撑软件.exe`（**工作目录 = `C:\software\workspace\1qfosim\FOSimStudio`**）。
日志落点：

| 文件 | 内容 |
|---|---|
| `FOSimStudio\log\RadarOutput.log` | 第 19/28–49 项（含**第 42 跟踪滤波、第 44 运动特征、第 47A 综合识别**）|
| `FOSimStudio\log\SatelliteTarget.log` | 第 5–18/20–27 项 |
| `FOSimStudio\log\{Radar,IR}{External,Internal}Interface.log` | 第 57–77 项 |
| `FOSimStudio\bin\App\lrdr_rir_diag_*.log` | LRDR 诊断（`[view]` / `[issue]` / `[feature-pick]` / `[scene-vel]` / `[cue]`）|
| `FOSimStudio\bin\App\sbirsfusion_diag_*.log` | 融合诊断（`[verbose]` / `[event]` / `[predict]`）|
| `FOSimStudio\bin\App\opir_sbirs_diag_*.log` | SBIRS 诊断 |

---

## 3. 验收判据（按顺序看，一步不对就不必往下）

| # | 检查项 | 命令 / 位置 | 期望 |
|---|---|---|---|
| 1 | 航迹不再每周期重建 | 第 42 项行里的 `航迹=N` | 同一目标长时间保持**同一个 N**（不再出现 32/33/34 同周期并存） |
| 2 | 滤波速度收敛 | 第 42 项 `速度=(vx,vy,vz)m/s` | 弹道目标应为 **km/s 量级**（真值实测 3877~5458 m/s） |
| 3 | 速度误差 | 第 42 项 `速度误差=…` | 显著小于真值速度（收敛后应为百米/s 量级） |
| 4 | 位置误差非零 | 第 42 项 `位置误差ECEF=…` | **不再恒为 (0,0,0)**（非零才说明是滤波输出而非量测直通） |
| 5 | 第 44 项运动特征 | `[运动特征处理功能测试] … 速度=` | 非零、km/s 量级 |
| 6 | 第 47A 项识别 | `[综合特征识别功能测试] … 置信度=` | 由 ≈0.09 上升；理想情况出现弹道型号且 `置信度 > 0.60`（判据 `acceptance_score`） |
| 7 | 跟踪占空比 | `[view] … has_track=1` 与 `snr(valid=1)` 计数 | 探测占空比由 47% 提升 |

参考命令（PowerShell）：
```powershell
$p='C:\software\workspace\1qfosim\FOSimStudio\log\RadarOutput.log'
Select-String -Path $p -Pattern '\[跟踪滤波功能测试\].*目标ID=101' -Encoding UTF8 | Select-Object -First 4 | % {$_.Line}
Select-String -Path $p -Pattern '\[运动特征处理功能测试\].*目标ID=101' -Encoding UTF8 | Select-Object -First 4 | % {$_.Line}
Select-String -Path $p -Pattern '\[综合特征识别功能测试\].*目标ID=101' -Encoding UTF8 | Select-Object -First 4 | % {$_.Line}
```

---

## 4. 本机踩过的坑（务必先看，能省数小时）

1. **UTF-8 BOM 必须保留**：1q 与 BJ 的源文件多为 **UTF-8 with BOM**；用脚本/编辑器改动后若
   BOM 被剥掉，MSVC 会按 GBK 解析中文注释 ⇒ 编译报**莫名其妙的语法错**（曾报在第 52 行）。
   改动后用字节检查 `EF BB BF`，缺失就补回。仓库有 `scripts/check_source_bom.py` / `utf8_bom.py`。
2. **`PROJECT_LOG_*` 在 Windows 上默认关闭**（`ONEQ_ENABLE_FILE_LOG` 默认 OFF）⇒ **不要把新诊断
   写进项目日志**，否则等于没写（`recognition/RecognitionTracker.cpp:362-374` 的 `sims=…` 就是这么
   白写的，Windows 上永远看不到）。要可见就写：**验收行**（`runtime/RirAcceptanceRecords.cpp`）
   或**集成侧诊断**（`ModelSource\CyModel\...\1q_integration\oneq_diag_log.h`）。
3. **1q.lib 是静态库，被 `Sensors.dll` 与 `DataProcessors.dll` 各链一份** ⇒ `AcceptanceFileLog.cpp`
   的 statics 有两份；"首次打开截断"的判定必须跨模块共享（已修，见 `ClaimFirstOpenForPath`）。
   改 `AcceptanceFileLog` 时要意识到这一点。
4. **验收日志已改为逐行 flush**；改回缓冲会导致**运行期读不到日志**（曾误判为"日志全丢失"）。
5. **集成构建前必须关闭 `仿真支撑软件.exe`**，否则 `LNK1104`（DLL 被占用）。
6. **`ModelSource\library\` 里的 DLL** 会被构建顺带改写 ⇒ 提交前用
   `git checkout -- ModelSource/library` 还原，避免把构建噪声混进提交。
7. **不要把 stdout 当日志**：宿主的模型 `printf` 不落盘（曾因此看不到融合的 `first dual-LOS fix ok`）。
8. **仿真步长**：当前为 **1.0 s/周期**（验收行时间戳 `2504→2505…` 可自证）；弹道目标每周期位移
   约 5 km —— 这正是零速初值必然失败的原因，任何"关联门/外推"相关改动都要按 1.0 s 复核。

---

## 5. 提交规范（两个仓库）

| 仓库 | 路径 | 分支 | 说明 |
|---|---|---|---|
| 1q | `C:\software\workspace\1qfosim\1q` | `fosim_2013` | 源码改动提交在这里 |
| BJ | `C:\software\workspace\1qfosim\BJ_10Y_WorkSpace` | `master` | `1q.lib` 更新 + 集成侧改动 |

惯例：1q 源码一笔、BJ 的 `1q.lib` 一笔，提交信息里写清"改了什么 / 为什么 / 实测证据 / 验证方式"。

---

## 6. 已完成的修复（本会话，供判断基线——不要重复做）

| 内容 | 1q 提交 | BJ 提交 | 状态 |
|---|---|---|---|
| 验收文件"首次打开"跨模块判定（修跨 DLL 截断，第 10 项恢复） | `ad1717a5` | `e87f597` | ✅ 已验证 |
| 验收日志逐行落盘（运行期可读） | `d119ee49` | `bbeda0e` | ✅ 已验证 |
| 主瓣地杂波改用"波束下沿地面交点距离"（修 ∝R² 非物理放大） | `a5773902` | `b10fa02` | ✅ 已部署 |
| 运动特征标量模长兜底（`speed`/`acceleration_mps2` 标量为 0 时取向量模长） | `04c49531` | `8b9b0e5` | ⚠️ 生效但不解决根因 |
| 特征集选档修复（导弹改用 `ballistic_icbm`，用场景真值 RCS 兜底） | — | `9af4f34` | ✅ 已验证 |
| 场景/配置：8° 视场 + 3 行栅格 + 全盘覆盖 + 配对窗口 35 | — | `133471c` `0adaae4` | ✅ 双星交会已成立 |
| 推演引擎接入（第 20/21/22/23/25 项） | — | `b2feddd` | ✅ `[predict]` 出数 |
| 雷达站点 300 m AGL（第 34 项杂波几何窗口） | — | `b8e313d` | ✅ 已部署 |
| 诊断：`[feature-pick]` / `[scene-vel]` | — | `910c8ee` `6509bc2` `481a1b3` | ✅ 可用 |

**已排除的 9 条假设**（不要重复排查）：真值速度未填、`Initialize(measurement)` 误用、
快照未拷贝速度、识别层拿到未确认航迹、取快照时机、集成侧步长错、dt 口径错、
滤波增益配置趴 0、双星引导替换位置。详见各提交信息。
