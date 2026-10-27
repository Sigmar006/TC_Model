# 目标类别/型号判不出来——检查路径与代码定位（2026-10-27）

> 面向外部检查：**第 1 章**给出按日志逐层排查的步骤；**第 2 章**给出"每个计算值由哪个函数算的"；
> **第 3 章**列出最可能的原因与修法；**第 4 章**用一个 4,700 m/s 的实例演示怎么算。

---

## 0. 结论先行

类别判不出来只有三种可能，检查时先分清属于哪一种：

| 现象 | 含义 | 后续动作 |
|---|---|---|
| **综合分 < 0.6** | 所有模板的相似度都低 → 特征与模板系统性失配 | 走 §1.4，找"偏差最大的一维" |
| **综合分 ≥ 0.6 但边际 < 0.05** | 候选模板太接近（典型 ICBM vs IRBM） | 属模板重叠，需增判别力（一维像/高度） |
| **有效维度数 < 3** | 特征本身不足 | 走 §1.2，看哪一维无效及为什么 |

判据（`policy.recognition`）：`acceptance_score = 0.6`、`minimum_margin = 0.05`、
`feature_weights = rcs 0.25 / motion 0.25 / polarization 0.25 / range_profile 0.25`、
`accumulation_window_sec = 60`。

---

## 1. 检查路径（按顺序做，每步都有可判读字段）

### 1.1 确认识别在跑

| 检查 | 命令 | 正常 | 异常含义 |
|---|---|---|---|
| 47A 有输出 | `grep 综合特征识别功能测试 log/RadarOutput.log \| wc -l` | 每航迹每周期 1 行 | 0 行 → 识别未启用或航迹未确认 |
| 73 项有输出 | `grep 雷达航迹与特征识别交互接口测试 log/RadarInternalInterface.log` | 每航迹每周期 1 行 | 缺 → 内部接口未写 |
| 航迹状态 | 73 项 `航迹状态=` | `确认` | `候选/丢失` → 识别不参与，先修跟踪 |

### 1.2 看特征有效掩码（**最关键**）

73 项 `特征有效掩码=` 是位图（`RirRecognitionFeatureDimension`，`RirRecognitionResult.h:59-64`）：

| 位 | 维度 | 值 |
|---|---|---|
| bit0 | RCS | 1 |
| bit1 | 运动 | 2 |
| **bit2** | **极化** | **4** |
| **bit3** | **一维像（距离像）** | **8** |

全有效 = **15**。置位条件是 `valid == true` **且** `quality > 0`
（`RecognitionTracker.cpp:174-190`）。

| 掩码 | 组成 | 缺哪一维 | 常见原因 |
|---|---|---|---|
| 15 | 1+2+4+8 | 无 | 特征齐全，问题在打分（走 §1.4）|
| **11** | 1+2+8 | **极化** | SNR 低于极化门限 / 有效 patch 不足 |
| **7** | 1+2+4 | **一维像** | 带宽不足 / 散射中心塌成 1 个（长度=0）|
| 13 | 1+4+8 | **运动** | 不确定性过大（协方差迹超门）|
| 14 | 2+4+8 | **RCS** | 视角覆盖 < `minimum_aspect_coverage_deg` 或 SNR 低 |
| 3 | 1+2 | 极化+一维像 | 只剩 2 维，基本判不准 |

> 掩码由 `RecognitionTracker.cpp` 的聚合函数置位（每维 `valid && quality>0` 才置位），
> `valid`/`quality` 在对应提取器里计算（见 §2.3）。

### 1.3 看"谁在主导"（第 44/45/46/47A 项的加权贡献）

每项都带 `综合识别加权贡献=`，四项之和 = 综合分。判读：

| 现象 | 结论 |
|---|---|
| 某一维 ≈0.9，其余 ≈0.02 | 该维一票主导（其余维质量低被归一化掉）|
| 四项都 < 0.15 | 分母不小但相似度都低 → 所有模板都不匹配（模板/量纲问题）|
| 某项缺行 | 该维无效（与掩码对应）|

贡献由 `TryRecognitionContribution`（`RirAcceptanceRecords.cpp:133`）计算：
`contribution = base_weight × quality × similarity / Σ(base_weight × quality)`。

### 1.4 把各维实测量与模板逐项比对

弹道族模板（`target_feature_database_v1.1.db`）：

| 型号 | RCS 均值±σ (dBsm) | 速度 (m/s) | 高度 (m) | 加速度 (m/s²) | 一维像长度 (m) |
|---|---|---|---|---|---|
| BALLISTIC_SRBM | −6.0±2.5 | 1,500±400 | 1.5e5±7e4 | 7.5±3.0 | 9.0±1.5 |
| BALLISTIC_MRBM | −8.0±3.0 | 3,000±500 | 4.5e5±2e5 | 7.5±3.0 | 12.0±2.0 |
| BALLISTIC_IRBM | −9.5±3.5 | 4,300±600 | 9.0e5±3e5 | 7.5±3.0 | 15.0±2.5 |
| BALLISTIC_ICBM | −10.0±4.5 | 5,000±800 | 1.5e6±9e5 | 7.5±3.0 | 18.0±3.0 |
| BALLISTIC_EXAMPLE_A | −3.0±2.0 | 1,800±300 | 5.0e4±1.2e4 | 12.0±6.0 | 8.0±2.0 |

> **极化维对弹道族零区分度**：五个弹道模板的极化参数完全相同
> （energy_difference 2.0±1.5、relative_difference −6.0±2.0、energy_sum 5.0±4.0），
> 但它仍占 0.25 权重——相当于给所有弹道型号加同一个常数项，只稀释差异、不提供判别力。

计算每维偏差：`z = |实测 − 均值| / σ`；**z > 1.5 的那一维就是把分数拉下来的维**。

### 1.5 看综合分与边际（新增诊断字段）

73 项已加入（需重编译后生效）：

```text
… 类别=未知 型号=未识别 综合分=0.4821 次优分=0.4703 边际=0.0118 有效维度数=3 积累观测数=412
```

| 字段 | 来源 | 判读 |
|---|---|---|
| `综合分` | `RirRecognitionResult::best_score` | < 0.6 → 模板全不匹配 |
| `次优分` | `runner_up_score` | — |
| `边际` | 两者之差 | < 0.05 → 型号不能确认 |
| `有效维度数` | `valid_feature_mask` 置位数 | < 3 → 特征不足 |
| `积累观测数` | `observation_count` | 反复归零 → 航迹/窗口重置 |

### 1.6 看时间维度是否被打断

| 字段 | 判读 |
|---|---|
| `积累观测数` | 应单调增长；反复归零 = 航迹丢弃重建或窗口清空 |
| `特征来源周期` vs `识别来源周期` | 差值过大 → 识别吃陈旧特征 |
| `恒虚警` 行缺口 | 检测缺口会让 60 s 窗口数据变稀 |

---

## 2. 计算值在哪个函数（外部检查直接按此定位）

### 2.1 打分与判决（核心）

| 计算 | 函数 | 文件:行 |
|---|---|---|
| 单维高斯相似度 | `GaussianSimilarity` | `src/remote_identification_radar/recognition/RecognitionMatcher.cpp:19` |
| 运动维相似度（速度/高度/加速度等权 + 转弯半径） | `MotionSimilarity` | `RecognitionMatcher.cpp:27` |
| 极化维相似度（三个参数等权） | `PolarizationSimilarity` | `RecognitionMatcher.cpp:54` |
| 一维像相似度（长度/峰数/能量集中度等权） | `RangeProfileSimilarity` | `RecognitionMatcher.cpp:76` |
| profile 适用性（带宽/视角覆盖门限） | `Applicable` | `RecognitionMatcher.cpp:98` |
| 四维相似度向量 | `RirMatcher::ComputeFeatureSimilarities` | `RecognitionMatcher.cpp:120` |
| **加权综合分** | `RirMatcher::QueryBestMatch` | `RecognitionMatcher.cpp:139`，加权在 **:189-197**（`Σ(w·q·sim)/Σ(w·q)`），**型号先验**在 **:210**（`score = profile_score × model.prior`）|
| 大类分数（取大类内最佳型号） | `QueryBestMatch` | `RecognitionMatcher.cpp:226` 起 |
| **状态判决（未知/类别确认/型号确认）** | `RirRecognitionTracker::Update...` | `src/remote_identification_radar/recognition/RecognitionTracker.cpp:374-420` |

判决逻辑（`RecognitionTracker.cpp:377-410`）：

```cpp
margin   = best_score - runner_up_score;
margin_ok = margin >= options_.minimum_margin;      // 0.05
if (best_score < options_.acceptance_score) {       // 0.6
    state = kUnknown;                                // ← 类别判不出来就是走这里
} else if (margin_ok && 维度足够) {
    state = kModelConfirmed;                         // 型号确认
} else {
    state = kCategoryConfirmed;                      // 只确认类别
}
```

> **外部检查只需在这两处下断点/加日志**：`RecognitionMatcher.cpp:197`（每个 profile 的分数）
> 与 `RecognitionTracker.cpp:377`（边际与状态）。

### 2.2 特征聚合（窗口 → 特征集）

| 内容 | 函数 | 文件 |
|---|---|---|
| 每周期观测构建 | `BuildRecognitionObservation`（`RecognitionObservationBuilder.cpp:56` 起） | `src/remote_identification_radar/recognition/RecognitionObservationBuilder.cpp` |
| 窗口聚合与状态机 | `RirRecognitionTracker` 的周期更新 | `RecognitionTracker.cpp`（`UpdateCycle` 一路）|

### 2.3 各维特征提取（`valid` / `quality` 在这里决定）

| 维度 | 提取器 | 关键门限 |
|---|---|---|
| RCS | `RirRcsFeatureExtractor::Extract` | `minimum_aspect_coverage_deg`（视角覆盖）、SNR |
| 运动 | `RirMotionFeatureExtractor::Extract` | 速度/高度/加速度不确定性（协方差迹）|
| 极化 | `RirPolarizationFeatureExtractor::Extract` | SNR、有效 patch 数 |
| 一维像 | `RirRangeProfileFeatureExtractor::Extract` | `minimum_bandwidth_hz`（长度 15 m ≈ 需 ≥10 MHz）|

文件：`src/remote_identification_radar/recognition/{Rcs,Motion,Polarization,RangeProfile}FeatureExtractor.cpp`。

### 2.4 验收行写点（日志字段就在这里拼）

| 项 | 函数 | 文件:行 |
|---|---|---|
| 第 44/45/46/47/47A 项 + 73 项 | `WriteRirTrackAndId` | `RirAcceptanceRecords.cpp:1312`（73 项在 **:1376 / :1421**） |
| `综合识别加权贡献` | `TryRecognitionContribution` | `RirAcceptanceRecords.cpp:133` |
| 第 29–35 项（检测链） | `WriteRirDetectionChain` | `RirAcceptanceRecords.cpp:619` |

---

## 3. 最可能的问题（按概率排序）

| # | 问题 | 判别证据 | 修法 |
|---|---|---|---|
| **①** | **一维像长度恒 0**（散射中心塌成 1 个）| 掩码缺 bit3（值 7）；47A 项 `长度=0.000m 峰数=1` | 修距离像多峰分辨，恢复该维 |
| **②** | **极化维对弹道族零区分度**却占 25% 权重 | 五个弹道模板极化参数相同 | 极化权重置 0 或补差异化模板 |
| **③** | 速度落在 ICBM/IRBM 之间（如 4,700 m/s）| 综合分高、边际 <0.05 | 靠高度/一维像区分，速度不承担 |
| **④** | 高度维失配（升降段被判 MRBM）| 高度 z>1 | 说明阶段，或用中段高度做判据 |
| **⑤** | RCS 维无效（视角覆盖 <15°）| 掩码缺 bit0（值 14）| 延长驻留/放宽覆盖门限 |
| **⑥** | 窗口被重置（积累观测数归零）| `积累观测数` 反复归零 | 修航迹连续性（检测缺口）|

---

## 4. 实例：4,700 m/s 弹道弹能不能判出型号

按 §2.1 的公式实算（四维权重 0.25，一维像/极化按中性 0.5 参与）：

| 场景 | 最佳 | 次优 | 边际 | 判定 |
|---|---|---|---|---|
| 高度 1,500 km、RCS −10 dBsm、像长 18 m | ICBM 0.860 | IRBM 0.646 | **0.214** | ✅ 可判 ICBM |
| 高度 1,000 km、像长 15 m | IRBM 0.842 | ICBM 0.749 | **0.092** | ✅ 可判 IRBM |
| 高度 300 km（升降段）、像长 12 m | MRBM 0.712 | IRBM 0.646 | **0.066** | ✅ 可判 MRBM |
| **同一 4,700 m/s，但一维像失效**（3 维） | ICBM 0.797 | IRBM 0.789 | **0.008** | ❌ **停"类别确认"** |

结论：

1. **单靠速度判不出来**——4,700 m/s 同时落在 ICBM（0.375σ）与 IRBM（0.667σ）内，
   速度维只贡献约 0.03 的综合分差；
2. **能判型号的前提是"高度 + 一维像"有效**；
3. 一旦一维像失效（当前常态），ICBM 与 IRBM 的边际只有 **0.008**，必然停在"类别"。

---

## 5. 复核用命令

```bash
LOG=FOSimStudio/log
# 1) 掩码分布
grep -o "特征有效掩码=[0-9]*" $LOG/RadarInternalInterface.log | sort | uniq -c
# 2) 综合分/边际分布（新版本才有）
grep -o "综合分=[0-9.]*" $LOG/RadarInternalInterface.log | sort | uniq -c | head
# 3) 各维实测量
grep "雷达目标RCS测量功能测试\|航迹运动参数测量功能测试\|极化特征提取功能测试\|一维像特征提取功能测试" $LOG/RadarOutput.log | head
# 4) 加权贡献
grep -o "综合识别加权贡献=[0-9.]*" $LOG/RadarOutput.log | sort | uniq -c | head
```

> 注：`综合分/次优分/边际/有效维度数` 四个字段本次已加入 73 项写入点
> （`RirAcceptanceRecords.cpp` 的 `WriteRirTrackAndId`），但 `1q\build\` 构建树已被清理，
> 需重新 `cmake` 配置后才能编译生效。
