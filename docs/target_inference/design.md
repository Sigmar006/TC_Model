---
Status: active
Last-reviewed: 2026-08-18
Authority: target_inference 设计权威入口
Answers: target_inference 是什么、和 fusion/threat_assessment 怎么分工、设计文档怎么导航
---

# Target Inference 设计

`target_inference` 是**目标域推演层算法面**（contract.md §目标处理分层契约；模式照抄
threat_assessment 先例：泛型输入帧、无状态纯函数、算法不感知传感器与坐标系来源、
无 Session 形态）。它把航迹状态帧（ECEF 位置/速度/协方差 + 可选类型证据）推演为
**带误差预算的目标结论**：轨迹预测、落点、发射点回推、类型概率。

心智模型：**态势 → 推演**。每次调用 `TargetInferenceEngine::Infer(tracks)` 对每条
航迹独立执行"前向外推 + 反向积分 + 证据融合"，返回与输入顺序一致的结果；引擎无
跨调用状态，同输入同输出。

## 关键定位

- **JDL 推演层落点**：估计层（fusion，Level 1-2 关联+滤波）之后的态势精化——
  "目标是什么/要去哪/从哪来"；威胁评分（"多危险"）仍归 threat_assessment（决策层）。
- **误差预算是产品语义不是附件**（契约规则 6）：发射点/落点必须携带 1-σ 与协方差；
  无协方差输入的产品显式 `has_uncertainty=false`，消费方不得把 σ=0 读作零误差。
- **输入对接**：调用方从 `fusion::FusedTarget` 组装 `InferenceTrackState`（值级传递，
  不引用 fusion 类型——分层契约规则 1）；RIR 识别结论经调用方键映射转成类型证据
  （TARGET-OQ-4 方案 a，零库内改动）。**多源证据预留**（2026-08-18 OQ-4 修订；
  出口①已于同日 Stage B 落地）：RIR 特征量测帧（`RirFeatureMeasurementRecord`，
  经 fusion 适配器带库内键可自动关联）上线后，类型证据可由多源特征证据组合——
  本模块仅预留证据通道语义（输入帧的 type_evidence 向量），不在模块内实现特征级
  识别算法；组合逻辑归调用方/未来识别算法面。
- 弹道模型私有于本模块（中心引力 + 可选指数大气阻力 RK4），不依赖
  JSBSim/flight_dynamic。

## 文档导航

- 模块边界、非目标、变更规则 → [boundaries.md](boundaries.md)
- 算法清单（弹道积分/回推/敏度误差预算/类型融合）、实现边界与反直觉点 →
  [algorithms.md](algorithms.md)
- 四类预测（轨迹/落点/发射点/关机点）的**方法、参数表与计算流程**（对接与验收查阅入口）
  → [prediction_methods.md](prediction_methods.md)

注：target_inference 不含 data-flow.md——无状态函数式算法面（同 threat_assessment
先例），没有数据流管线。

跨模块公共规则见 `docs/common/contract.md`（不参与三层输出模型与传感器周期语义）。
