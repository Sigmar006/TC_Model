---
Status: active
Last-reviewed: 2026-08-20
Authority: fusion 模块级边界、非目标与设计变更规则
Answers: fusion 有哪些模块级禁令、关联键边界、变更规则
---

# Fusion 模块边界

本文承载 fusion 的模块级边界、非目标和变更规则。算法级边界与反直觉点见
[algorithms.md](algorithms.md)。

## 关联键边界（冻结）

1. **无外部身份通道**：探测记录的身份键由调用方提供（库内键，如 AR
   `association_key`、ESR `hypothesis_id`），跨源一致性由调用方保证；
   融合引擎不解析、不生成任何场景真值标识（守去真值化纪律）。
2. 身份键 **0 约定为无身份**：仅无身份探测参与空间/方位/特征关联；
   有身份探测直接按键归并。
3. 无身份探测新建的航迹使用**引擎合成键（≥ 2^63）**，与调用方身份键
   （约定 < 2^63）不冲突；合成键仅引擎内部保证稳定，业务层不应跨 Reset 依赖。

## 探测记录边界

- `DetectionRecord` 为**泛型**记录（位置/方位/特征向量/判决值/质量/库内键），
  算法不感知 ESR/EOS/AR/RIR 具体类型；库提供官方适配器 `fusion/SensorAdapters.h`
  （五传感器输出——AR 轨迹帧 / ESR 假设 / EOS 探测 / SBIRS 探测 / RIR 特征量测帧
  → 泛型记录，可选便利层），业务层也可自行适配。
- 位置/方位/特征三通道可独立存在；特征向量维度跨源不一致时特征门不构成约束。
- **合成量测隔离（2026-10-26）**：`DetectionRecord::synthetic=true` 的记录是调用方
  由"上次真实几何 + 单源视线"推导出的**非独立**量测（如单星保活位置）。引擎对它只做
  保活语义（累计命中、更新锚点/最近周期、生命周期），**不进入运动学滤波更新**；
  `FusedTarget::updated_by_real_measurement` / `last_real_measurement_cycle` 如实
  透出该航迹是否吃过真实量测。消费方（精度评估、推演）必须跳过纯合成航迹，
  不得把预测外推值当实测数据打分。

## SAR 输入约束（冻结）

库内 SAR 无探测/spot-report 输出（`SarOutputFrame` 仅为图像质量元数据），
首期 SAR 不作为融合输入，可作为使命状态旁路信息；SAR 探测能力属未来扩展
（见 `docs/review/Behavior.md` §4.3）。

## 非目标

1. ~~不做轨迹滤波~~（**2026-08-17 P2 解除**：逐航迹无迹滤波已落地，实现边界见
   algorithms.md §4；`src/common/estimation/` 滤波器为唯一原语来源，契约规则 4）。
2. 不提供 ECM 逐威胁 tasking SPI、报告节奏策略或事件模型（属 example 业务层）。
3. 不建事件总线、不引入外部身份通道（与去真值化纪律冲突）。
4. 不提供 Session/Cycle 会话形态。
5. 航迹滤波非目标（P2 冻结）：不做在线残差驱动滤波器切换、不做 IMM（后续冻结项）、
   不做 Mahalanobis 关联门（现有空间/方位门沿用；引入前须按变更规则 2 冻结）、
   不做单源角度-only 的动力学收敛起始器（以配置距离先验起始，弱可观测性由协方差
   如实承载——见 algorithms.md §4 反直觉点）。
6. 量测原点契约：`DetectionRecord.has_sensor_origin` 为真时，方位角定义为该原点
   （度制 LLA）传感器局部 ENU（az 自北向东、el 出地平）；无原点的方位记录只参与
   关联不参与三维滤波。跨系对齐（如 SBIRS ECI 角度 → ENU）归调用方——库内不做
   跨系转换（session_contract.md 既有立场）。

## 设计变更规则

1. 关联键策略、置信度公式或滑窗语义变化必须同步本文档集与
   `docs/review/Behavior.md` §4 冻结项。
2. 若引入轨迹滤波或新的关联度量（如 Mahalanobis 门限），必须先在
   algorithms.md 冻结实现边界，并评估 `airborne_radar::signal::association`
   既有实现的复用/提升路径。
3. 空间索引实现（当前 nanoflann KD-tree）变更必须说明复杂度与确定性输出影响。
