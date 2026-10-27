# 红外（SBIRS）进不了窄视场（NFOV）跟踪——检查路径与代码定位（2026-10-27）

> 面向外部检查：**§2** 是逐层排查路径，**§3** 是"每个判据由哪个函数算"，
> **§4** 是根因与证据，**§5** 是修法，**§6** 是现场 grep 清单。

---

## 1. 结论先行

宽场（WFOV）探测正常、但**始终进不了窄场跟踪**。代码走查后的判断：

| 优先级 | 原因 | 一句话 |
|---|---|---|
| **①** | 捕获判定用了**共享镜筒当前的指向**，而不是**本目标的指向** | 多目标轮转时，新候选拿别的目标的指向去比，必然"视场外" |
| **②** | 判捕获**不等指向稳定**，转向途中每拍都判失败 | 每周期只转 3°，而捕获要求落在 2.5° 内 |
| **③** | 失败**立即释放**（无重试），形成活锁 | 释放 → 下拍重新调度 → 再失败 |
| ④（次要） | 窄场门限 5.0 **严于**宽场门限 4.0 | SNR∈[4,5) 的目标"能进宽场、永远捕不到" |

> 注意：④ 在本场景多半不是主因——调参注记记录 GEO 4e7 m、1e4 W/sr 工况 SNR ≈ +16 dB（≈40 线性），
> 远高于 5.0。**主要矛盾是 ①②③ 的几何/时序问题。**

---

## 2. 检查路径（按顺序，每步都有可判读字段）

### 第 1 步：确认宽场已有候选（否则无需谈窄场）

| 看什么 | 字段/命令 | 正常 | 异常含义 |
|---|---|---|---|
| 宽场探测行 | `grep 红外宽场扫描探测功能测试 IRInternalInterface.log` | 有目标、有 SNR | 无行 → 目标没进宽场，先查辐射/距离门 |
| 候选入列 | `grep 红外宽场候选与窄场调度交互接口测试` | 有"调度结果=已选中" | 无 → 连续命中/调度器问题 |

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
| `当前服务目标` ≠ 当拍被判捕获的 `目标ID` | **根因①确认**：用别的目标的指向判本目标 |
| 只有 1 个目标时也可能失败 | 看第 4 步 |

### 第 4 步：看是否"没等稳定就判"（根因 ②）

| 参数 | 值 | 推导 |
|---|---|---|
| `frame_rate_hz` | 10.0 | dt = 0.1 s |
| `narrow_pointing_max_slew_rate_deg_per_sec` | 30.0 | **每周期最多转 3.0°** |
| `narrow_field_fov_az/el_deg` | 5.0 | 捕获窗口 = **±2.5°** |
| `narrow_pointing_settle_tolerance_deg` | 0.01 | 稳定判据 |
| 稳定判定上限 | `180 / max_slew_rate` = **6 s** | `SbirsPointingCoordinator.cpp` 的 `AdvanceAcquisition` |

⇒ 若 `指向状态=转动中` 且 `剩余角 > 2.5°`，本拍判捕获**必然失败**，
但代码并未检查 `kSettled`（`SbirsPipeline.cpp:2138-2141` 算了 `settled_frames` 只用于日志）。

### 第 5 步：看失败后有没有重试（根因 ③）

日志行（`SbirsPipeline.cpp:1986-1993`）：

```text
… 目标ID=… 捕获结果=失败 失败原因枚举=1 捕获失败原因=nfov-acquisition-failed
```

失败分支（`:2244-2252`）会 `Release` + 状态回 `kWideCandidate`：
下一拍重新调度、重新用当前指向判定 → **活锁**。判据：同一 `目标ID` 的
"失败 → 已选中 → 失败"是否**无限循环**（`grep 目标ID=103` 看行序）。

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
| 轮转服务目标（**问题源**） | `served_target_id = engaged_ids[nfov_rotation_step_ % size]` | `SbirsPipeline.cpp:2085` |
| 指向推进（限速 3°/周期） | `SbirsPointingCoordinator::AdvanceAcquisition` / `AdvanceTracking` | `SbirsPipeline.cpp:2127-2133`，实现 `SbirsPointingCoordinator.cpp` |
| 稳定帧数 | `settled_frames`（**只进日志**） | `SbirsPipeline.cpp:2138-2141` |
| **捕获判定** | `IsNfovAcquisitionEligible` | 调用 `SbirsPipeline.cpp:2217`，实现 `src/sbirs_sensor/pipeline/SbirsNfovAcquisition.cpp:19-27` |
| 捕获失败处理（活锁） | `if (!captured) { Release… }` | `SbirsPipeline.cpp:2244-2252` |
| 进入跟踪 | `PromoteToTracking` + 状态置 `kEstimatedTracking` | `SbirsPipeline.cpp:2270-2278` |
| 跟踪门丢锁 | `nfov_tracking_gate_loss_cycles = 2` | `SbirsPipeline.cpp` 跟踪段 |

捕获判据本体（`SbirsNfovAcquisition.cpp:19-27`）：

```cpp
in_nfov = |delayed_truth_az − (command_az + settle_error)| <= fov_az/2
       && |delayed_truth_el − command_el|              <= fov_el/2;
return in_nfov && snr >= minimum_snr_linear;
```

---

## 4. 根因与证据

### ① 指向串了（结构性）

```cpp
// SbirsPipeline.cpp:2085  每周期只轮转服务一个目标
served_target_id = engaged_ids[nfov_rotation_step_ % engaged_ids.size()];
...
pointing_result = pointing_coordinator_.AdvanceAcquisition(served_target_id, ...);  // 只推这一个
...
// :2217 却对**所有**当拍被调度的候选判捕获，用的是上面这一个目标的指向
const bool captured = IsNfovAcquisitionEligible(acquisition_request);
```

⇒ 多目标时，新候选的 `delayed_truth` 与“当前指向”相差几十度 → `in_nfov = false`。

### ② 不等稳定

`AdvanceAcquisition` 每周期最多转 3°，而捕获窗口只有 ±2.5°；
代码不检查 `status == kSettled`，转到一半就判，必然失败。

### ③ 失败即释放（活锁）

`:2244-2252` 释放锁定、状态回宽场候选，下一拍重新调度 → 同一目标反复"失败"。

### ④ 门限倒挂（次要）

`wide_min_snr_linear = 4.0` < `narrow_min_snr_linear = 5.0`：
SNR∈[4,5) 的目标宽场可见、窄场永远不可捕；cross-cue 旁路进来的低 SNR 候选同理。

---

## 5. 修法（`SbirsPipeline.cpp` 的 `attempt_capture`，最小改动）

```cpp
// 1) 只对"本拍被服务 且 指向已稳定"的目标判捕获；否则保持等待，不释放
if (served_target_id != target_id ||
    pointing_result.status != SbirsPointingAdvanceStatus::kSettled) {
  target_states_[target_id] = SbirsTargetState::kAwaitingNfovAcquisition;
  return;                        // 下一拍再判
}

// 2) 失败只在超时后释放（AdvanceAcquisition 已给 elapsed_wait_sec / kTimedOut）
const bool timed_out =
    (pointing_result.status == SbirsPointingAdvanceStatus::kTimedOut ||
     pointing_result.elapsed_wait_sec >= 180.0 / max_slew_rate_deg_per_sec);
if (!captured && timed_out) {
  nfov_scheduler_.Release(target_id);
  append_acquisition_failure(selected, SbirsCaptureFailureReason::kNfovPointingTimeout);
  return;
}
```

3) 门限对齐：`narrow_min_snr_linear` 取 `wide_min_snr_linear`（4.0），
   或对 `from_external_cue` 候选沿用宽场门限，避免"能进不能捕"。

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
