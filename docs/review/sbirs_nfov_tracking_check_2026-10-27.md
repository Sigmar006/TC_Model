# 红外（SBIRS）进不了窄视场（NFOV）跟踪——检查路径与代码定位（2026-10-27）

> 面向外部检查：**§2** 是逐层排查路径，**§3** 是"每个判据由哪个函数算"，
> **§4** 是根因与证据，**§5** 是修法，**§6** 是现场 grep 清单。

---

## 1. 结论先行

宽场（WFOV）探测正常、但**始终进不了窄场跟踪**。代码走查后的判断（**2026-10-27 复核修订**）：

| 优先级 | 原因 | 一句话 |
|---|---|---|
| **①** | 单镜筒轮转：每周期只推进**一个**目标 3°，其余目标"命令每拍更新、指向跟不上" | 目标数 N 越大，收敛越慢；`kTimedOut`（6 s 服务时长）先到就释放 |
| **②** | 失败**立即释放**（无重试预算），形成"释放→重调度→再失败" | 每次重来都从当前指向重新爬 |
| **③** | 捕获失败**只记结果、不记子判据**（视场外 or SNR 不足），无法定位 | 目前日志只有 `捕获判决输入SNR` 与 `捕获结果` |
| ④（次要） | 窄场门限 5.0 **严于**宽场门限 4.0 | SNR∈[4,5) 的目标"能进宽场、永远捕不到" |

> **修订说明（2026-10-27）**：此前把"不等稳定就判捕获"列为原因②，**该判断有误**。
> 复核 `SbirsPipeline.cpp:2704-2766` 后确认：捕获只在 `pointing_result.status == kSettled`
> 的 `else` 分支里调用（`kSlewing` / `kTimedOut` / `kRejected` 各自先行处理），
> 即**代码确实等稳定**。真正可疑的是单镜筒轮转下的收敛速度与超时释放策略。

> 注意：④ 在本场景多半不是主因——调参注记记录 GEO 4e7 m、1e4 W/sr 工况 SNR ≈ +16 dB（≈40 线性），
> 远高于 5.0。

---

## 2. 检查路径（按顺序，每步都有可判读字段）

### 第 1 步：确认宽场已有候选（否则无需谈窄场）

| 看什么 | 字段/命令 | 正常 | 异常含义 |
|---|---|---|---|
| 捕获判决 | `grep "捕获判决输入SNR" $LOG` | 有 SNR 与结果 | 无行 → 从未走到捕获判定（一直在 kSlewing）|
| **指向状态** | `grep "指向状态=" $LOG` | 出现"已稳定" | **全是"转动中"→ 根因①**（从不稳定）|
| 剩余角 | 同上行 `剩余角=` | 逐渐减小到 <0.01° | 长期几十度 → 轮转跟不上 |
| 失败循环 | `grep "目标ID=<id>" $LOG \| grep -E "捕获结果\|调度结果"` | 少量失败后成功 | 长期"失败→重调度"→ 根因② |
| 成功计数 | `grep -c "捕获结果=成功" $LOG` | >0 | =0 → 从未捕获 |

### 修复后的预期

| 指标 | 修复前 | 修复后 |
|---|---|---|
| `捕获结果=成功` 行数 | 0 | > 0（每目标首次稳定后 1 行）|
| 同一目标的"失败"次数 | 长期循环 | ≤ 设定重试预算（如 5 次）或 6 s |
| 跟踪状态 | 始终 `kWideCandidate` | `kEstimatedTracking`（日志 `候选状态=已捕获跟踪`）|
| 第 76 项候选状态 | 只有"指向未稳定" | 出现"已捕获跟踪" |
| 融合侧 `ObservationStage` | 恒为 0（宽场）| 出现 1/2，`drop_stage` 下降 |

### 第 2 步：看捕获判决的 SNR 是否够

日志行（`SbirsPipeline.cpp:2218-2229`）：

```text
… 卫星ID=… 目标ID=… 捕获判决输入SNR=<snr> 捕获门限=5.000000 捕获结果=成功/失败 候选状态=已捕获/捕获失败
```

| 结果 | 含义 | 下一步 |
|---|---|---|
| `输入SNR` **≥ 5** 且 `捕获结果=失败` | **不是 SNR 问题**，是几何/时序 → 第 3 步 | 修 ①②③ |
| `输入SNR` **< 5** | 门限倒挂（④）| 对齐门限到 4.0，或对 cross-cue 放宽 |

### 第 3 步：看"指向串了没有"（根因 ①）

日志行（`SbirsPipeline.cpp:2170-2180`）：

```text
… 当前服务目标=<A> 指令方位/俯仰=(az,el)deg 指向状态=已稳定/转动中 剩余角=<r>deg 候选状态=…
```

判读：

| 现象 | 结论 |
|---|---|
| `当前服务目标` 与当拍被判捕获的 `目标ID` 不同 | 只可能出现在"同帧免转动捕获"路径（`:2770-2800`，条件是该目标已落在当前视场内）；若该目标并不在视场内却进入该路径，即为缺陷 |
| 只有 1 个目标时也可能失败 | 看第 4 步 |

### 第 4 步：看指向收敛与超时（根因 ①）

| 参数 | 值 | 推导 |
|---|---|---|
| `frame_rate_hz` | 10.0 | dt = 0.1 s |
| `narrow_pointing_max_slew_rate_deg_per_sec` | 30.0 | **每周期最多转 3.0°** |
| `narrow_field_fov_az/el_deg` | 5.0 | 捕获窗口 = **±2.5°** |
| `narrow_pointing_settle_tolerance_deg` | 0.01 | 稳定判据（`SbirsPointingActuator.cpp` 的 `settled = 剩余角 ≤ 容差`）|
| 轮转服务 | 每周期只服务 1 个目标 | `SbirsPipeline.cpp:2085` |
| 捕获超时释放 | `180 / max_slew_rate` = **6 s（按"被服务时长"累计）** | `SbirsPointingCoordinator::AdvanceAcquisition` |

⇒ 目标数 N 时，每个目标每 N 拍才被推进 3°；若在累计 6 s 服务时长内没进到
`剩余角 ≤ 0.01°`，`AdvanceAcquisition` 返回 `kTimedOut`，`SbirsPipeline.cpp:2731-2746`
立即释放并回宽场候选——下一拍重新调度、从当前位置重爬。

### 第 5 步：看失败后有没有重试（根因 ②）

日志行（`SbirsPipeline.cpp:1986-1993`）：

```text
… 目标ID=… 捕获结果=失败 失败原因枚举=1 捕获失败原因=nfov-acquisition-failed
```

失败分支（`:2244-2252`）会 `Release` + 状态回 `kWideCandidate`：
下一拍重新调度、重新判定。判据：同一 `目标ID` 的"失败 → 已选中 → 失败"是否**长期循环**
（`grep 目标ID=<id>` 看行序）。

### 第 6 步：确认物理前提

| 检查 | 期望 | 说明 |
|---|---|---|
| 目标在窄场视场内 | 真值角与指令指向差 < 2.5° | `in_nfov` 判据 |
| 扫描限位未钳死 | `az ±180° / el ±90°`（默认，配置未设 `orientation`）| 不是本次原因 |
| cue 延迟 | `narrow_cue_latency_s = 0` | 不是本次原因 |

---

## 3. 计算值在哪个函数（外部检查直接按此定位）

| 判据/量 | 函数 | 文件:行 |
|---|---|---|
| 连续命中门（宽→窄） | 候选循环内 `consecutive_hits < required` | `src/sbirs_sensor/pipeline/SbirsPipeline.cpp:2003-2031` |
| 调度器选中/锁定 | `nfov_scheduler_.SelectForAcquisition` / `Acquire` | `SbirsPipeline.cpp:2034-2039` |
| 镜筒初始化 | `pointing_coordinator_.EnsureActuatorInitialized` | `SbirsPipeline.cpp:2082-2084` |
| **轮转服务目标**（每拍只服务 1 个） | `served_target_id = engaged_ids[nfov_rotation_step_ % size]` | `SbirsPipeline.cpp:2085` |
| 指向推进（限速 3°/周期） | `SbirsPointingCoordinator::AdvanceAcquisition` | `SbirsPipeline.cpp:2127-2133`，实现 `SbirsPointingCoordinator.cpp` |
| **稳定判据** | `SbirsPointingActuator::Step`：`settled = 剩余角 ≤ settle_tolerance(0.01°)` | `src/sbirs_sensor/pipeline/SbirsPointingActuator.cpp` |
| **超时释放** | `AdvanceAcquisition`：`elapsed ≥ 180/max_slew_rate` → `kTimedOut` | `SbirsPointingCoordinator.cpp` |
| 稳定帧数（跟踪积分用） | `settled_frames` | `SbirsPipeline.cpp:2138-2141` |
| **捕获判定** | `IsNfovAcquisitionEligible` | 调用 `SbirsPipeline.cpp:2217`，实现 `SbirsNfovAcquisition.cpp:19-27` |
| 捕获分支（稳定后才判） | `kSlewing / kTimedOut / kRejected / else(settled→attempt_capture)` | `SbirsPipeline.cpp:2704-2766` |
| 捕获失败处理 | `if (!captured) { Release… }` | `SbirsPipeline.cpp:2244-2252` |
| 同帧免转动捕获 | 已落在当前视场内的非服务候选 | `SbirsPipeline.cpp:2770-2800` |
| 进入跟踪 | `PromoteToTracking` + 状态置 `kEstimatedTracking` | `SbirsPipeline.cpp:2270-2278` |

捕获判据本体（`SbirsNfovAcquisition.cpp:19-27`）：

```cpp
in_nfov = |delayed_truth_az − (command_az + settle_error)| <= fov_az/2
       && |delayed_truth_el − command_el|              <= fov_el/2;
return in_nfov && snr >= minimum_snr_linear;
```

---

## 4. 根因与证据（2026-10-27 复核版）

### ① 单镜筒轮转导致收敛慢 + 超时释放

```cpp
// SbirsPipeline.cpp:2085  每周期只推进一个目标（最多 3.0°）
served_target_id = engaged_ids[nfov_rotation_step_ % engaged_ids.size()];
pointing_result = pointing_coordinator_.AdvanceAcquisition(served_target_id, command_los,
                                                           input.dt_sec, pointing_config);
// SbirsPointingCoordinator.cpp:  next_elapsed = acquisition_wait_sec_[id] + dt_sec;
//   if (next_elapsed >= 180.0 / max_slew_rate) → kTimedOut 并清空计时
```

目标数 N 时每个目标每 N 拍才前进 3°；6 s（服务时长）内没进到
`剩余角 ≤ settle_tolerance(0.01°)` 就 `kTimedOut`，随后 `SbirsPipeline.cpp:2731-2746`
释放并回 `kWideCandidate`，下一拍重新调度、从当前位置重爬。

### ② 失败即释放（无重试预算）

`attempt_capture` 的 `!captured` 分支（`:2244-2252`）直接 `Release` + 回宽场候选，
没有"本目标再等几拍"的预算；与 ① 叠加就成了"爬一半被释放"。

### ③ 捕获失败只记结果、不记子判据

`:2218-2229` 只写 `捕获判决输入SNR` 与 `捕获结果`，**不写**是 `in_nfov` 失败还是 SNR 失败，
也不写方位/俯仰残差——这正是现在无法定位的直接原因。

### ④ 门限倒挂（次要）

`wide_min_snr_linear = 4.0` < `narrow_min_snr_linear = 5.0`：
SNR∈[4,5) 的目标宽场可见、窄场永远不可捕；cross-cue 旁路进来的低 SNR 候选同理。

---

## 5. 改法（按"先诊断、再行为"两阶段；函数级清单）

### 阶段 A：加诊断（不改行为，先定位）

| # | 函数 | 位置 | 改动 |
|---|---|---|---|
| A1 | `IsNfovAcquisitionEligible` | `src/sbirs_sensor/pipeline/SbirsNfovAcquisition.cpp:19` | 增加一个带输出的重载：返回 `{kOk, kOutsideFov, kSnrBelow}`，并输出 `az_residual_deg` / `el_residual_deg`；原 bool 版本保留转发 |
| A2 | `SbirsPipeline::RunCycle`（`attempt_capture` lambda）| `SbirsPipeline.cpp:2187-2253` | 捕获日志行追加 `子判据=`、`方位残差=`、`俯仰残差=`、`视场半宽=`、`指向状态=` |
| A3 | `SbirsPipeline::RunCycle`（`kSlewing` 分支）| `SbirsPipeline.cpp:2710-2730` | "转向中"日志追加 `剩余角=` 与 `已服务时长=`（`pointing_result.remaining_angle_deg` / `elapsed_wait_sec`）|

### 阶段 B：改行为（三处，逐条可回退）

| # | 函数 | 位置 | 改动 |
|---|---|---|---|
| B1 | `SbirsPipeline::RunCycle`（`attempt_capture`）| `:2244-2252` | `!captured` 时**保留** `kAwaitingNfovAcquisition`，累计失败次数；仅当 `kTimedOut` 或失败次数 ≥ N（建议 5）才 `Release` |
| B2 | `SbirsPointingCoordinator::AdvanceAcquisition` | `SbirsPointingCoordinator.cpp` | 命令角相对上次变化 > `settle_tolerance` 时**重置** `acquisition_wait_sec_`；或把超时改为"距上次命令更新"计时（避免命令刷新把计时器"拖死"）|
| B3 | `SbirsPipeline::RunCycle`（轮转选择）| `:2085` | 让"正在捕获中的目标"**连续占用镜筒**若干拍（直到 settled / 超时）再轮转；避免 3°/N 爬行 |
| B4（可选）| 配置 | `sbirs.json` / 宿主 | `narrow_min_snr_linear` 与 `wide_min_snr_linear` 对齐（4.0）；宿主 `kCueMinChannels` 1→2 |

> 建议顺序：**先做 A1–A3**，跑一轮拿到"子判据 + 残差"，再决定 B1–B3 里哪几条是必要的——
> 如果残差本来就是几十度，B3 是主因；如果残差 <2.5° 而 SNR 不足，则是 ④。

---

## 6. 现场确认命令（30 秒）

```bash
LOG=FOSimStudio/log/IRInternalInterface.log

# 1) 捕获判决的 SNR 与结果
grep "捕获判决输入SNR" $LOG | head
#    期望：SNR 约 30~40；若"输入SNR≥5 且 捕获结果=失败" → 几何/时序问题

# 2) 当前服务目标 vs 被判捕获目标（根因①）
grep "当前服务目标=" $LOG | head
#    服务目标 ≠ 目标ID → 指向串了

# 3) 指向是否稳定（根因②）
grep "指向状态=" $LOG | head
#    全是"转动中"、剩余角几十度 → 没等稳定就判

# 4) 失败是否无限循环（根因③）
grep "目标ID=103" $LOG | grep -E "捕获结果|调度结果" | head -20

# 5) 是否成功过
grep -c "捕获结果=成功" $LOG
```

### 修复后的预期

| 指标 | 修复前 | 修复后 |
|---|---|---|
| `捕获结果=成功` 行数 | 0 | > 0（每目标首次稳定后 1 行）|
| 同一目标的"失败"次数 | 无限 | ≤ 转向周期数（剩余角/3°，上限 6 s）|
| 跟踪状态 | 始终 `kWideCandidate` | `kEstimatedTracking`（日志 `候选状态=已捕获跟踪`）|
| 第 76 项候选状态 | 只有"指向未稳定" | 出现"已捕获跟踪" |
