---
Status: active
Last-reviewed: 2026-08-27
Authority: fusion 算法清单与实现边界
Answers: 每个融合算法怎么实现、边界在哪、反直觉点是什么
---

# Fusion 算法

## 算法清单

| 算法 | 输入 | 输出 | 证据 |
|---|---|---|---|
| 关联分层（键直挂 / 位置 / 方位 / 特征） | 探测记录 + 既有航迹 | 探测 → 航迹归属 | `src/fusion/FusionEngine.cpp`；`tests/unit/fusion/fusion_association_test.cpp` |
| 置信度滑窗融合 | 滑窗内量测 | 每航迹融合置信度 | 同上；`tests/unit/fusion/fusion_confidence_test.cpp` |
| 传感器输出 → 泛型探测记录（官方适配器） | AR 轨迹帧 / ESR 假设 / EOS 探测 / SBIRS 探测 / RIR 特征量测帧 | `DetectionRecord` 列表 | `src/fusion/SensorAdapters.cpp`；`tests/unit/fusion/sensor_adapters_test.cpp` |
| 逐航迹无迹滤波 + 航迹管理（P2，默认关） | 关联后量测（位置 / 方位+原点） | 运动学估计（ECEF 状态 + 6×6 协方差）+ 生命周期 | `src/fusion/FusionEngine.cpp`；`tests/unit/fusion/track_filtering_test.cpp` |

## 1. 关联分层

每周期 `Update` 处理顺序：

1. **身份键直挂**：探测按键（≠ 0）分组，同键归并到同航迹（跨源一致性由调用方
   保证）；无对应航迹时创建。
2. **位置关联**：对带位置锚点的航迹建 nanoflann KD-tree（ECEF 三维，double，
   `L2_Simple_Adaptor`，先例复制自 ESR `KdTreeClusterer` 的 adaptor/radiusSearch
   模式，含 1.3.x / 1.5+ 返回类型兼容垫片），无身份探测做半径搜索
   （`position_radius_m`，平方半径语义）。
3. **方位关联**：仅方位探测与带方位锚点的航迹做 `AreBearingsCoherent`
   （`bearing_beamwidth_deg`，az 按最短圆差、el 线性比较）。
4. **特征门限**：`feature_threshold > 0` 且双方都有特征且维度一致时，要求欧氏
   距离 ≤ 门限；否则不构成约束。
5. 候选择优：位置取最近、方位取最短圆差、纯特征取首个通过者；
   未关联 → 新建航迹（合成键）。

### 实现边界

- 每周期成本 **O(N log N + M)**：KD-tree 建树 O(T log T) + 每次半径搜索 O(log T)，
  无全量两两比对。
- **同周期新建航迹不参与本周期后续关联**（自下周期起成为候选）；
  **无身份探测之间本周期互不合并**（合并依赖身份键或后续周期航迹关联）——
  两源对同一新目标的首见周期会各自成航迹，属已知限制（业务层可用身份键规避）。
- 锚点为航迹最近一次量测；KD-tree 使用周期开始时的锚点快照。

## 2. 置信度滑窗融合

- 公式（冻结）：**confidence = Σ 判决值 × 质量归一化 × 权重**，对滑窗内全部
  量测精确求和。
- 权重按 `source_id` 索引 `FusionConfig::source_weights`，缺失按 1.0。
- 每源每航迹 `std::deque` 滑窗（`window_size`），超出驱逐并重算置信度。
- 失跟：航迹每周期未收到量测则 `missed_cycles` +1，超过 `max_missed_cycles` 删除。

### 反直觉点

- **置信度不归一化**：随窗口内证据（探测次数/源数）单调累积，可超过 1.0——
  这是冻结公式的字面语义（更多证据 → 更高置信），不是归一化概率。
- 滑窗按**量测条数**而非周期数截断；同一源高频率探测会更快驱逐旧样本。
- 输出按航迹键升序（`std::map` 迭代序），确定性可复现。

## 3. 传感器输出 → 泛型探测记录（官方适配器）

`fusion/SensorAdapters.h` 为五传感器输出提供默认映射（`Adapt*ToDetectionRecords`，
跳过规则与质量归一化基准如下），集成方即开即用；业务层可自行适配（不修改
本文件）。源通道常量 `kArSourceId=1..kSbirsSourceId=4`、`kRirSourceId=5` 与
`FusionConfig::source_weights` 索引对齐（索引 0 未用；source_weights 缺项按 1.0 计）。

| 适配函数 | 跳过规则 | 身份键 | 质量归一化基准 |
|---|---|---|---|
| `AdaptArTracksToDetectionRecords` | `kLost` 轨迹 | `association_key` | `target_probability`，无识别置信度时按状态（`kConfirmed`→1.0 / 其余→0.5） |
| `AdaptEsrHypothesesToDetectionRecords` | `hypothesis_id == 0` | `hypothesis_id` | `confidence` |
| `AdaptEosDetectionsToDetectionRecords` | `detected == false` | 0（方位相干） | `fused_snr_db / 10`（10 dB → 1.0）夹取 [0,1] |
| `AdaptSbirsDetectionsToDetectionRecords` | `detected == false` | 0（方位相干） | `infrared_snr_linear / 4`（WFOV 门限 4.0 → 1.0）夹取 [0,1] |
| `AdaptRirFeatureMeasurementsToDetectionRecords`（2026-08-18 Stage B 落地，冻结契约见 `docs/review/rir_dual_product_stage_a_2026-08-18.md` §3.2/§3.3） | 全维无效或键 0 记录 | `association_key`（库内键透传，ESR 先例） | 有效维质量等权均值（feature_weights 口径，缺省等权） |

**RIR 特征量测映射（已落地，冻结要点）**：11 维固定布局（RCS dBsm / 速度 km/s / 高度 km /
加速度 m/s² / log10 转弯半径 / 极化三量 dB / 距离像长度 m / 峰数 / 峰能集中度），
**无效维填 0**（NaN 禁止——毒化欧氏门）、有效性以帧内 valid_feature_mask 为权威；
方位通道做 east→north 参考换算（az = wrap(90° − look_az)）。**观测原点**（2026-08-18
修订 1/1a）：输入周期携带平台位置（ECEF 米制）时，适配器经 TryEcefToLla 换算填
has_sensor_origin + origin（失败退化为无原点记录）→ 记录可参与三维方位滤波。
**位置通道**（2026-08-27 裁定，Stage A 记录已删档）：
斜距有限且 >0 且原点换算成功时，按 `ComputeLookAngles` 逆运算（自东 az、出地平 el）
还原东-北-天，再 TryEnuToEcef→TryEcefToLla 填 has_position；失败维持仅方位+原点。
位置与方位并存（滤波位置优先，方位留给跨源仅方位关联）；量测噪声走
`default_position_noise_std_m`（默认 50 m，与 AR 同路径）。未携带原点时仅关联 + 特征门。
已知近似：0 填对欧氏特征门引入失真，mask-aware 门升级为后续冻结项；跨源维度不一致
（ESR 4 维 vs RIR 11 维）维持"门不约束"既有语义。

### 实现边界

- AR 位置 ECEF→LLA（`TryEcefToLla`），转换失败退化为仅身份键记录（`has_position=false`）。
- RIR 位置由斜距+自东视线角+平台原点还原（`TryEnuToEcef`→`TryEcefToLla`），失败维持仅方位+原点。
- ESR 射频特征归一化到可比尺度（GHz/MHz/ms/µs），供特征门限启用。
- 归一化基准固化为库默认（示例场景验证过的业务决策），不做配置参数（YAGNI）。
  [evidence: tests/unit/fusion/sensor_adapters_test.cpp]

## 输出语义

`FusedTarget`：库内键、各源通道状态（量测数、最近判决/质量/位置/方位）、
融合置信度、最近更新周期、航迹生命周期（tentative/confirmed/coasting）、
可选运动学估计（LLA 位置 + ECEF 速度 + 6×6 ECEF 协方差，行主序 [x,vx,y,vy,z,vz]）。
报告节奏（配置化周期 + 事件触发）属 example 业务层，
引擎不感知周期语义，`cycle` 用于失跟统计、时间戳与滤波 dt 推导（×
`track_cycle_period_sec`）。

## 4. 逐航迹无迹滤波 + 航迹管理（P2，2026-08-17 冻结）

### 实现边界

- **开关**：`enable_track_filtering` 默认 true。置 false 时引擎行为与 P2 前
  完全一致（关联/置信度/删除语义不变，仅输出追加 lifecycle 字段，无运动学估计）。
- **状态与时间对齐**：每航迹 6 维 ECEF CV [x,vx,y,vy,z,vz]；预测 = 无迹 + `LinearCvTransitionModel`
  （`track_process_noise`）。时间推进支持双轨（FUSION-OQ-1 闭环）：若量测携带有效物理时间戳 `has_timestamp`，dt 为前后量测物理时间差 dt = t_meas − t_last；若为纯周期驱动，回退为 dt = (cycle − 上次滤波周期) × `track_cycle_period_sec`。同周期内多源异步量测按物理时间戳升序排序依次推进，消除跨传感器观测的时间偏差；失跟（coasting）按周期步长同步推进物理与周期基准。
  [evidence: tests/unit/fusion/track_filtering_test.cpp::SubCycleTimestampPropagatesMovingTrackBetweenAsyncDetections]
  [evidence: tests/unit/fusion/track_filtering_test.cpp::SubCycleBearingDualSatResolvesRange]
- **位置通道更新**：z = LLA→ECEF 位置，`UnscentedUpdater<6,3>` +
  `LinearPositionMeasurementModel`；R = `default_position_noise_std_m`²（记录级
  位置噪声通道为后续冻结项，当前仅配置默认）。
- **方位通道更新**：记录携带 `has_sensor_origin` 时经 `UnscentedUpdater<6,2>` +
  库内 ENU 方位模型（h(x) = 目标 ECEF 相对原点的 ENU az/el，deg）；R 取记录
  `bearing_noise_sigma_rad`（换 deg），缺省 `default_bearing_noise_sigma_rad`。
  无原点的方位记录只关联不滤波。
- **起始**：位置记录 → ECEF 位置 + 零速 + 配置先验；方位+原点记录 → 沿 LOS 配置
  距离先验（`track_bearing_init_range_m`）+ 各向同性
  `track_bearing_init_range_std_m`。
- **失跟外推（coasting）**：已起始航迹无本周期量测时按周期外推一次，生命周期置
  kCoasting；`missed_cycles > max_missed_cycles` 删除（既有语义不变）。
- **确认门**：累计命中数 ≥ `confirm_hits`（默认 3）转 kConfirmed。
- **数值**：float 滤波态（同 common/estimation 家族）；P0 证据表明 σ≲10 µrad
  角度场景触及 float 精度边缘（target_domain_p0_p1_decision §4.2 结论 2），
  double 中间量为后续冻结项。
- **AR 关联复用评估**（变更规则 2 要求）：`airborne_radar::signal::association` 的
  LAPJV 关联与 fusion 关联单源化延后到 TARGET-OQ-1 债务处置立项（本交付范围外），
  结论登记，不在 P2 内动 AR。

### 反直觉点

- **角度-only 距离弱可观测是产品语义不是缺陷**：单原点方位航迹的距离方差收敛远慢
  于横向方差（可达性证据：地板公里级）；消费方必须读协方差而非点估计（契约规则 6）。
- **方位+原点记录与无原点方位记录语义不同**：前者参与三维滤波（ENU 契约），后者
  只参与方位相干关联——同一传感器若要进估计层必须由调用方补原点（库不做跨系转换）。
- 滤波不改变关联：关联仍用锚点量测（原始 LLA/方位），不用滤波后验——避免关联与
  滤波互馈发散。

[evidence: tests/unit/fusion/track_filtering_test.cpp]
