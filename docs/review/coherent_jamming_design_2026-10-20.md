# 相干干扰建模与雷达对抗措施 —— 技术方案（2026-10-20）

Status: proposal（待评审；**当前技术要求未要求相干干扰**，属能力扩展，不作为缺陷修复）
范围：RIR（远程识别雷达）接收/检测链；`RfScene` 干扰输入 → 检测预算 → 航迹输出
依据：`docs/review/functional_acceptance_test_criteria_ocr.md` 第 29/30/35/63 项、
`src/common/radar/DetectionCellResolver.cpp:254-260`、`src/common/electromagnetics/RfScene.cpp:282-357`

---

## 0. 结论摘要

| 项 | 现状 | 本方案 |
|---|---|---|
| 干扰处理 | 一律按**非相干**：只乘时频重叠、除 8 dB 抑制 | 按相干占比 ρ **分解**为相干/非相干两部分 |
| 脉压增益 | 只有回波乘 B·τ（本雷达 **48.573 dB**） | 相干干扰同样乘 B·τ，并落成**假目标** |
| 对抗手段 | 仅 `jamming_suppression_gain_db=8 dB` 一个平坦常数 | 7 类 ECCM，按可量化程度排列，全部可配置、可关闭 |
| 兼容性 | — | **ρ=0 时与现状逐位一致**（缺省关闭，可作回归基线） |

---

## 1. 现状与问题（数值）

本雷达发射 `kPulseTrain`：载频 3 GHz、占用带宽 **4.5 MHz**、脉宽 **16 ms**、PRI **0.1 s**
（`RirEmissionFactory.cpp:82-86`），因此

```
脉压增益 G_pc = B·τ = 4.5e6 × 0.016 = 72,000 = 48.573 dB
距离分辨   = c/(2B) = 33.3 m
无模糊距离 = c·PRI/2 = 15 000 km
```

现行判决预算（`DetectionCellResolver.cpp:254-260`）：

```
SINR = S · G_pc · G_target / ( N · G_noise + J / G_jam + C / G_clutter )
```

**干扰项没有 G_pc。** 用 2026-10-20 轮实测值（回波 1.06e-17 W、干扰 1.06e-14 W、G_jam=8 dB）：

| 处理假设 | 有效干扰 | 处理后 SINR | 判定 |
|---|---|---|---|
| 非相干（现状） | 1.06e-14 / 6.31 = 1.68e-15 W | **+23 dB** | 可检测 |
| 相干（同波形） | 1.06e-14 × 72 000 / 6.31 = 1.21e-10 W | **−23 dB** | 不可检测 |

**差 46~48 dB**，即现状对"同波形/转发式干扰"的抗干扰能力高估了一整个脉压增益。

---

## 2. 相干干扰分类与数学描述

| 类型 | 物理过程 | 匹配滤波后 | 对雷达的后果 |
|---|---|---|---|
| **同波形独立** | 同型号雷达各自触发，PRF/时标不对齐 | 只有与本振副本相干的部分被压缩，其余按失配衰减 | 部分相干（ρ 小），近似噪声底抬高 |
| **转发式（Repeater）** | 接收我方脉冲、放大后按固定时延转发 | **完整压缩**，落在 τ_d 对应距离单元 | 生成**假目标**（距离 = c·τ_d/2 + 干扰机视距） |
| **距离/速度拖引（RGPO/RGPI）** | 转发时延/频移随时间渐变 | 压缩后目标"移动" | 把跟踪波门拖走，导致丢批 |
| **部分相干** | 宽带噪声 + 相干分量混合 | 相干分量压缩、其余进噪声底 | ρ 加权 |

### 2.1 计算模型（本方案核心公式）

```
给定一条入射链路：到达功率 J、相干占比 ρ∈[0,1]、转发增益 G_rep、时延 τ_d、频偏 f_off、场景重叠系数 f_ov

J_coh   = J · ρ · G_pc · G_rep            // 相干部分：获得完整脉压增益
J_incoh = J · (1−ρ) · f_ov / G_jam        // 非相干部分：维持现状

若 |τ_d − τ_target| ≤ 1/B（同距离分辨单元）：
    J_eff = J_coh + J_incoh               // 作为"信号竞争"直接进分母（与目标抢同一单元）
否则：
    J_eff = J_incoh                       // 不抬高目标单元噪声底
    另在 τ_d 单元生成一条检测候选（假点迹，见 §4 S2）
```

SINR 与判决不变：

```
SINR = S · G_pc · G_target / ( N · G_noise + J_eff + C / G_clutter )
```

**关键设计**：相干部分不再"除以抑制"，而是与其自身增益一起作为**信号级功率**参与竞争；
这与物理一致——匹配滤波对相干干扰是相干积累，对热噪声是非相干积累。

### 2.2 数值示例（本场景，ρ=1、G_rep=0 dB）

| 输入 | 值 |
|---|---|
| 到达功率 J | 1.06e-14 W |
| τ_d | 1 µs → 假目标距离偏移 150 m（≈4.5 个分辨单元） |
| J_coh | 1.06e-14 × 72 000 = 7.6e-10 W |
| J_eff（异单元） | 0（目标单元不受影响）；假目标在其自身单元以 7.6e-10 W 出现 |
| 同单元情形 | SINR = 1.06e-17×72 000×2 / (4.5e-14×1.26 + 7.6e-10) ≈ 2.0e-3 ⇒ **−27 dB，目标被压制** |

---

## 3. 参数设计（三层）

### 3.1 外部干扰输入（`include/1q/electromagnetics/RfScene.h`）

`RfSceneWaveformKind` 已有 `kContinuous / kPulseTrain / kLinearSweep / kBandLimitedNoise`，
但**没有"与我方同波形"的表达**。新增：

| 字段 | 类型 | 含义 | 缺省 |
|---|---|---|---|
| `coherence_mode` | enum `kNone/kSameWaveformIndependent/kRepeater/kDeceptive` | 相干方式 | kNone |
| `coherence_fraction` | double [0,1] | 相干能量占比 ρ | 0 |
| `repeater_delay_s` | double | 转发时延（→ 假目标距离 c·τ/2） | 0 |
| `repeater_gain_db` | double | 转发增益（相对入射功率） | 0 |
| `frequency_offset_hz` | double | 相对我方载频的频偏 | 0 |
| `timing_alignment` | bool | 是否与本雷达发射时标（PRF）对齐 | false |
| `waveform_signature` | struct{kind, center_hz, bandwidth_hz, pulse_width_s, pri_s, chirp_rate_hz_per_s, code_id} | 与本地副本比对 | 由现有字段补齐 |

宿主 `LRDRJammer` 的 `.mdef/.mdata` 同步增加：`CoherenceMode`、`CoherenceFraction`、
`RepeaterDelayUs`、`RepeaterGainDb`、`WaveformSignature*`（现只有 `TransPowerW/BandWidthHz/Polarization` + 波形类别）。

### 3.2 雷达侧配置（`RirHardwareConfig` / `RirPolicyConfig`）

| 参数 | 含义 | 建议缺省 | 备注 |
|---|---|---|---|
| `coherent_jamming_enabled` | 总开关 | **false** | 关闭时与现状逐位一致 |
| `coherence_match_threshold` | 判定"同波形"的签名匹配度阈值 | 0.9 | 带宽/脉宽/PRI/中心频容差 |
| `anti_coherent_jamming_improvement_db` | 抗相干干扰改善因子（捷变/旁瓣/极化合计） | 0 | 从 G_pc 中扣减 |
| `false_target_gate_sigma` | 假点迹与航迹外推的距离一致性门 | 3.0 | 假目标剔除 |
| `range_pull_rate_limit_mps` | 距离拖引速率上限 | 0（不限） | 超限判假 |
| `max_false_targets_per_cycle` | 单周期假点迹上限 | 32 | 防航迹爆炸 |

### 3.3 日志字段

| 项 | 新增字段 |
|---|---|
| 30 干扰功率 | `相干方式=`、`相干占比=`、`波形匹配度=`、`转发时延=`、`假目标距离=` |
| 29 回波功率 | `相干干扰=`、`非相干干扰=`、`处理后SINR=`（§T-01 同批） |
| 35 干扰处理增益 | `相干干扰抑制比=`、`假目标剔除数=` |

---

## 4. 处理流程与代码落点

```
RF 场景（RfScene.cpp）
  └─ 新增：波形签名匹配 → 相干占比 ρ、时延 τ_d、假目标距离       ← 新增
        ↓
单元级干扰（DetectionCellResolver::TryResolveCellInterference）
  ├─ 非相干部分：现状（时频重叠 × 到达功率 / G_jam）              ← 保留
  └─ 相干部分：J·ρ·G_pc·G_rep（不除抑制）                        ← 新增
        ↓
判决预算（DetectionCellResolver:254-260）
  └─ 分母：N·G_noise + J_eff + C/G_clutter                       ← 改一处
        ↓
假目标注入（RirController 检测链）                                ← S2 新增
  └─ τ_d 单元生成检测候选 → 点迹（64 项）→ 关联 → 航迹
        ↓
关联/航迹（RirTrackAssociator / RirTrackLifecycle）
  └─ 假目标鉴别：距离变化率、与既有航迹冲突、拖引速率门            ← S3 新增
        ↓
验收日志（RirAcceptanceRecords.cpp）
  └─ 29/30/35 新增字段
```

| 阶段 | 文件 | 改动性质 |
|---|---|---|
| S1 | `RfScene.h`、`RfScene.cpp`、`DetectionCellResolver.cpp`、`RirDetectionCellResolver.cpp`、`RirHardwareConfig.h`、`RirAcceptanceRecords.cpp` | 纯计算 + 参数 + 日志 |
| S2 | `RirController.cpp`（检测候选注入 `g_acceptance_found_targets` 相邻路径）、`RirSignalDetector` | 新增目标来源 |
| S3 | `RirTrackAssociator.cpp`、`RirTrackLifecycle.cpp`、配置 | 判别与抑制 |

---

## 5. 雷达对抗措施（ECCM）与性能保障

技术要求未要求，但"保证雷达性能"必须给出可量化的对抗路径。按**模型可实现度**排序：

| # | 措施 | 机理 | 在本模型中如何体现 | 典型改善 | 代价 |
|---|---|---|---|---|---|
| 1 | **脉间波形捷变**（chirp 斜率/方向随机、PRF 参差） | 使转发信号与副本失配，ρ 与 G_pc 同时下降 | `waveform_signature` 随周期变体；匹配失败 ⇒ ρ→0，失配损耗项 | 10–20 dB | 处理复杂度、距离旁瓣 |
| 2 | **频率捷变** | 逃出干扰机调谐带宽 | `frequency_offset_hz` 与占用带宽重叠率下降 ⇒ f_ov↓ | 5–15 dB | 本振切换时间 |
| 3 | **旁瓣消隐/抑制（SLB/SLC）** | 从旁瓣进入的干扰被辅天线对消 | `jamming_suppression_gain_db` 分主/旁瓣两档（主瓣 0 dB、旁瓣 +20 dB） | 15–25 dB（旁瓣） | 辅通道硬件 |
| 4 | **极化鉴别** | 转发干扰通常保持我方极化，目标回波极化随姿态变 | 复用现有极化特征链路（第 46 项），对极化不符者降权/剔除 | 3–10 dB | 需双极化通道 |
| 5 | **多帧一致性/多假设检验** | 假目标距离跳变与角度/速度不自洽 | `false_target_gate_sigma`：与历史外推偏差超门即剔除 | 剔除率 80–95% | 航迹延迟 |
| 6 | **拖引判别（RGPO/RGPI）** | 拖引的距离变化率异常（远超物理加速度） | `range_pull_rate_limit_mps`：速率超限即冻结波门 | 拖引成功率↓ | 需速率估计 |
| 7 | **前沿跟踪** | 对 RGPO 只跟脉冲前沿 | 已有先例：`enable_anti_rgpo_leading_edge`（AR 在用，RIR 恒 false），可直接接线 | 抗拖引 | 波形需前沿可辨 |

### 5.1 性能保障指标（可写进验收/内部评估）

| 指标 | 定义 | 目标 |
|---|---|---|
| **抗干扰改善因子 A** | `A = 10log10(SINR_对抗后 / SINR_对抗前)`，同场景同干扰 | 措施 1+3+5 合计 **≥ 25 dB** |
| 相干干扰抑制比 | `10log10(J_coh_raw / J_coh_residual)`（第 35 项已有同口径字段） | ≥ 20 dB |
| 假目标剔除率 | `1 − 假航迹确认数 / 假点迹数` | ≥ 90% |
| 检测概率恢复 | 干扰存在时 Pd 相对无干扰 Pd 的下降 | ≤ 10 个百分点 |
| 航迹连续性 | 拖引场景下目标航迹中断次数 | 0 次 |

> 这些指标与现有验收项**不冲突**：第 35 项的四个子项本身就是"抑制比/抑制增益"，
> 只需在加了相干项后按新口径一并输出。

---

## 6. 实施计划

| 步 | 内容 | 交付 | 回归判据 |
|---|---|---|---|
| **S1** | 参数 + 预算式分段 + 日志字段（相干部分不落假目标） | 可在 ρ=1 但 τ_d=0（同单元）条件下验证"目标被压制" | **ρ=0 时六份日志与当前逐位一致**；RIR 单测 290/297 不变 |
| **S2** | 假目标注入（τ_d 异单元生成检测/点迹） | 可复现"距目标 150 m 处出现一条假航迹" | 假点迹数符合输入配置；无假目标时输出不变 |
| **S3** | ECCM：捷变/旁瓣分档/极化降权/多帧一致性/拖引门 | 改善因子 A 可测 | 假目标剔除率 ≥90%；目标航迹不断 |

S1 预计 1 个工作日（改动集中在 4 个文件 + 1 处公式 + 日志）；
S2 需要新增"检测候选来源"概念，约 2–3 天；S3 与关联/航迹耦合，建议单独立项。

---

## 7. 风险与边界

| 风险 | 处置 |
|---|---|
| 改变检测判决影响既有验收 | `coherent_jamming_enabled` 缺省 false，ρ=0 逐位兼容；开关状态写进日志首行 |
| J_coh 量级跨 7 个数量级（1e-14 → 1e-7） | 内部用 double 累计，日志用科学计数法（现有 `FormatSci` 已支持） |
| 假目标引发航迹爆炸 | `max_false_targets_per_cycle` 上限 + 假目标标记（不进识别积累） |
| 与技术要求不一致 | 判定标准第 30/35 项补一句"当前按非相干处理，不适用于转发式干扰"；本方案作为扩展项登记 |
| 极化相干处理 | **不做**：模型只有极化统计量，没有复散射矩阵，无法做极化域相干对消 |

**暂不做**：多基地/分布式相干干扰、干扰源无源定位、相干干扰下的识别性能评估（需 S2 完成后才有假目标样本）。

---

## 8. 与现有待办的关系

本方案登记为 **T-08**（见 `docs/review/todo_item29_power_evidence_2026-10-19.md`），
与 T-01（处理后 SINR 落日志）同批实施最省事：两者都要改 `DetectionCellResolver` 的预算与
`RirAcceptanceRecords` 的第 29/35 项输出，一次改动、一次回归。
