---
Status: active
Last-reviewed: 2026-11-06
Authority: RIR 主瓣地杂波的计算链路与「杂波信号处理增益」日志字段的推导
Answers: 杂波日志什么时候出现、每个字段由什么算出来、现在为什么没有、哪些口径要复核
---

# 杂波日志是怎么算出来的（RIR 主瓣地杂波 → MTI/MTD → 日志）

日志行归属验收第 34 项 `[杂波信号处理增益功能测试]`，共四类字段：

```text
MTI处理后杂波剩余功率=<W>
MTD各多普勒通道杂波剩余功率分布=[…]W
MTD处理后的杂波抑制比=<dB>
总杂波抑制增益=<dB>
```

四者出自**同一条链路的两段**：前段是"主瓣地杂波物理量"（`RirSurfaceClutterModel`），
后段是"MTI/MTD 频谱处理旁路"（`MtiMtdAcceptanceBank`）。中间用**杂波等效噪声功率**衔接。

## 0. 速览

```text
配置(植被档位/波形/天线) + 目标几何(视线俯仰、斜距) + 平台高度 + 总传播损耗
        │
        ├─① 擦地角 ψ = clamp(θ_el/2 − φ, 1°, 89°)        raw ≤ 0 → 杂波 0（主瓣离地）
        ├─② 地面照射距离 s（4/3 地球半径的射线-球面交点）
        ├─③ 距离单元 δR = c/(2B) → 擦地面积 A_c = min(脉冲限制, 波束限制)
        ├─④ σ₀(ψ) = σ₀_ref + 10lg(sinψ/sin10°) → 杂波 RCS σ_c = σ₀·A_c
        ├─⑤ 同一雷达方程（主瓣增益）+ 脉压增益 → 杂波回波功率 P_c
        └─⑥ CNR = P_c − N_thermal → 等效杂波噪声功率 N_c（±120 dB 钳位）
                │
                ├─► 检测链：SINR 分母的杂波项（实际参与判决）
                └─► 验收旁路：MTI/MTD 8 通道频谱核 → 第 34 项四类字段
```

## 1. 触发条件（决定"有没有这一行"）

| 条件 | 不满足时 |
|---|---|
| `environment.vegetation_cover_profile ≠ kDisabled` | 杂波恒 0，**整条日志省略**（不写"缺失"占位） |
| 目标视线俯仰 `φ < θ_el,3dB/2`（主瓣下沿擦地） | 杂波恒 0，同样不写行 |
| 斜距、热噪声、载频、带宽均 > 0 | 杂波 0 |
| 该目标本周期真的有检测单元（`has_cell`） | 无日志（旁路输入不全） |

宿主当前配置 `vegetation_cover_profile = kOpenGrassland`、俯仰波束 0.8° ⇒ **φ 必须 < 0.4°**
才可能有杂波。实战里被探测目标多在 20°~60° 仰角（导弹中段），主瓣完全离地，所以
**日志里一条杂波行都没有是正常的**——不是模型没跑，是几何条件不成立。

## 2. 前段：主瓣地杂波（`RirSurfaceClutterModel::EvaluateClutterNoiseW`）

### ① 擦地角

```text
ψ_raw = θ_el/2 − φ                      θ_el = 有效俯仰波束宽，φ = 目标视线俯仰
ψ_raw ≤ 0            → 杂波 0（主瓣下沿已在地平线以上）
ψ = clamp(ψ_raw, 1°, 89°)              下限进多径区不可信、上限防 cosψ 变号
```

有效波束宽由 `nominal_az/el_beamwidth_deg` 回退到 `λ/L` 孔径推导（`ResolveEffectiveBeamwidth`）。

### ② 主瓣下沿的地面照射距离

只判"擦地"不够，还要知道照到多远（否则远距离目标会算出面积 ∝R² 的非物理杂波区）：

```text
s²/(2·Re_eff) + s·sinδ − h = 0   ⇒   s = Re_eff·( √(sin²δ + 2h/Re_eff) − sinδ )
Re_eff = 4/3 · 6371 km（标准大气折射等效地球半径），δ = ψ_raw，h = 平台海拔
s > 目标斜距时按目标斜距取（杂波只可能来自目标之前的距离）
```

### ③ 擦地面积

```text
δR = c/(2B)                            匹配滤波带宽 B 决定距离单元（与测距误差同源）
A_pulse = s · θ_az · δR / cosψ         脉冲限制：距离向单元 × 方位向波束
A_beam  = s² · θ_az · θ_el / sinψ      波束限制：方位×俯仰足印
A_c     = min(A_pulse, A_beam)         低擦地走脉冲限制、高擦地走波束限制
```

### ④ 杂波 RCS

```text
σ₀(ψ) = σ₀_ref(档位) + 10·lg( sinψ / sin10° )     一阶擦地角折算，参考角 10°
σ_c   = σ₀(ψ) · A_c                               dB → m² 后乘面积
```

档位表（S 波段量级声明值，非实测标定）：开阔草地 −28 dB、稀疏林地 −22 dB、
落叶林 −18 dB、针叶林 −16 dB、热带密林 −14 dB。

### ⑤ 杂波回波

```text
P_c = 雷达方程(主瓣增益 G₀, σ_c, 距离 s, 传播损耗) + 脉压增益 10·lg(max(1, Bτ))
```

### ⑥ 等效杂波噪声功率（本段唯一出口）

```text
CNR   = P_c(dBW) − 10·lg(N_thermal)       N_thermal = k·T₀·B·F
N_c   = N_thermal · 10^(CNR/10)           公共单源换算，±120 dB 相对钳位
```

`N_c` 进两处：检测链 SINR 分母的杂波项（真正影响判决），以及下面的验收旁路。

## 3. 后段：MTI/MTD 验收旁路（`MtiMtdAcceptanceBank`）

这一段是**独立频谱核**，不改变检测链的 SINR，只用来产出第 34 项日志。固定口径：
8 个多普勒通道（`kMtiMtdChannelCount`）、2 脉冲 MTI、杂波谱宽 `σ_v = 0.25 m/s`。

```text
f_d      = fold(目标双程多普勒, PRF)          折进 ±PRF/2
H_mti(f) = 4·sin²(πf/PRF)                    两脉冲对消器功率响应（DC 处为 0）
H_k(f)   = |sin(Nθ/2)/(N·sin(θ/2))|²          N=8 点 DFT 第 k 通道功率响应
             θ = 2π·fold(f − k·PRF/N, PRF)/PRF
S(f)     = 高斯谱，σ_f = 2σ_v/λ              杂波谱宽（λ = c/f_c）
```

在 `[−PRF/2, PRF/2)` 上取 `kClutterGridCount` 个格点做数值积分：

```text
MTI 剩余杂波      = C · ∫S(f)·H_mti(f)df / ∫S(f)df
第 k 通道剩余杂波  = C · ∫S(f)·H_mti(f)·H_k(f)df / ∫S(f)df
总剩余            = Σ_k 第 k 通道
```

日志四个字段的对应关系：

| 日志字段 | 取值 | 备注 |
|---|---|---|
| `MTI处理后杂波剩余功率` | `mti_residual_clutter_w` | 直接落盘 |
| `MTD各多普勒通道杂波剩余功率分布` | `clutter_w[0..7]` | 直接落盘 |
| `MTD处理后的杂波抑制比` | `10·lg( C/N · 1/clutter_w[k_sel] )`，`N=8` | 先按通道数均分输入，再比目标通道 |
| `总杂波抑制增益` | `10·lg( C / Σ_k clutter_w[k] )` | 全通道合计，可能为负（见 §5） |

补充：`目标信号增益` / `噪声增益` 两类行用的是同一个 bank 的 `mti_gain_db`、`mtd_gain_db`，
与本项共用旁路，但**不参与**检测判决。

## 4. 宿主工作点的实测数值（可复算）

用宿主 RIR 配置（3 MW、3 GHz、B=30 MHz、τ=16 ms、PRF=10 Hz、G₀=52 dBi、波束 0.8°、
平台 300 m、目标斜距 80 km、开阔草地、无传播损耗）直接调用两个模型得到的值：

```text
thermal_noise = 3.0172e-13 W (−125.20 dBW)
look_el = 0.00° → clutter = 3.0172e-01 W   CNR = +120.00 dB
look_el = 0.20° → clutter = 3.0172e-01 W   CNR = +120.00 dB
look_el = 0.30° → clutter = 3.0172e-01 W   CNR = +120.00 dB
look_el = 0.40° → clutter = 0               ——主瓣下沿刚好离地
look_el = 1.00° → clutter = 0
look_el = 5.00° → clutter = 0

MTI 剩余 = 5.4440e-01 W（相对输入 −2.56 dB，即"抑制"为负）
通道分布 = [9.44e-03, 3.06e-02, 7.56e-02, 1.07e-01, 1.09e-01, 1.07e-01, 7.56e-02, 3.06e-02] W
总剩余 5.4440e-01 W，mtd_gain = +13.01 dB，mti_gain = −3000 dB（目标多普勒折回 DC，正好落在 MTI 陷波）
```

复算方式：写一个只调这两个内部模型的探针程序，链 `1q.lib` 即可（本仓库 `build/scratch/` 下
留有一份一次性探针源码；不随库交付）。

## 5. 三个需要复核的口径（本轮分析发现）

1. **杂波 CNR 顶到 ±120 dB 钳位**：`3.0172e-01 W` 正好等于 `N_thermal × 10^12`，说明真实
   CNR 已超过钳位。原因可追到第 ⑤ 步：杂波面积 `A_c` 已经按**压缩后**距离单元 `δR=c/(2B)`
   算过一遍，再乘一次脉压增益 `10·lg(Bτ)=56.8 dB`，两处都含"压缩"语义。若确认重复，
   量级会低约 57 dB（`3e-1 W → 6.3e-7 W`），杂波对 SINR 的影响也应同步复核。
2. **低 PRF 下"抑制增益"可能为负**：固定 `σ_v=0.25 m/s` 在 S 波段折合 `σ_f = 5 Hz`，
   而 PRF 只有 10 Hz——杂波谱铺满整个 PRF，MTI 陷波（DC 处为 0、半 PRF 处为 4）
   平均下来对杂波没有净抑制；同时 8 个通道的响应之和可以 >1，于是 `总杂波抑制增益` 出现
   −2.56 dB。这是"固定杂波谱宽 + 低 PRF"的组合后果，用低 PRF 的地平线模式时要预期到。
3. **当前日志 0 条杂波行**：宿主被探测目标仰角 26.6°~59.4°（导弹中段），远大于 0.4° 的
   擦地门，主瓣离地。要让第 34 项出数据，必须让雷达在**低仰角**照到目标
   （地平线扫描模式 + 近地平线目标），或把俯仰波束加宽——这正是"地平线扫描"需要单独
   成一个模式的原因。

## 6. 代码位置

| 环节 | 位置 |
|---|---|
| 逐目标杂波入口（装配输入、口径注释） | `src/remote_identification_radar/runtime/RirController.cpp:391` `ResolveTargetClutterPowerW` |
| 主瓣地杂波模型（①~⑥ 全部公式） | `src/remote_identification_radar/internal/RirSurfaceClutterModel.cpp` |
| 雷达方程 / 脉压增益 / 热噪声 | `src/remote_identification_radar/internal/RirRadarEquations.cpp` |
| MTI/MTD 验收旁路 | `src/common/radar/MtiMtdAcceptanceBank.cpp` |
| 第 34 项日志组装 | `src/remote_identification_radar/runtime/RirAcceptanceRecords.cpp:868`（杂波行） |
| 环境配置（植被档位） | `include/1q/remote_identification_radar/config/RirEnvironmentConfig.h` |
