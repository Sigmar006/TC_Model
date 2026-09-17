---
Status: active
Last-reviewed: 2026-08-23
Authority: sbirs_sensor 设计权威入口
Answers: SBIRS 模块是什么、和 EOS 有何不同、设计文档怎么导航
---

# SBIRS 设计

`sbirs_sensor` 提供天基红外预警仿真传感器（SBIRS-inspired）的配置、单周期输入、环境与大气建模、
扫描搜索发现、凝视捕获/跟踪、搜索→凝视交接（cueing & handover）、多目标状态管理、结构化结果聚合、
Recording/Replay、调试视图和生命周期事件。

SBIRS 的心智模型是**状态机驱动的双视场传感器**：
1. **WFOV 宽视场扫描**发现目标（带误差位置）；连续命中达到阈值（默认 1）才允许切换窄场。
2. **NFOV 首次捕获**：用 WFOV 带误差 cue 生成凝视指向，限速 ATP 稳定后做几何窗口 + SNR 门判定。
3. **NFOV 持续跟踪**：捕获成功后进入三种互斥模式之一（Estimated/Strict/Sensor-like），闭环 ATP 跟踪。

与 EOS 的核心差异：EOS 是单视场扫描探测器，对 FOV 内目标做一次性 SNR 判定；SBIRS 用跨周期状态机
管理每个目标的 WFOV 发现 → NFOV 首次捕获 → 持续跟踪全过程。

验收信息（需求映射 3.2.1.3 章节的覆盖区/驻留时间/焦平面脱靶量/信号能量/连续命中计数，
以及 3.2.1.6.3 的角定位误差）走独立文件 `SatelliteTarget.log`（CMake 开关
`ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG`，默认 OFF；四段同一行），不进公开输出结构、
也不进 `1q_library.log`；性能项单独写入 `opir_acceptance.log`。见 [boundaries.md](boundaries.md) 与
[algorithms.md](algorithms.md) 的验收派生量节。项表见
`docs/review/acceptance_item_catalog_2026-08-22.md`。

本文以公开 SBIRS/OPIR 资料为真实系统校准点，但不声称复刻真实 SBIRS 设备、保密载荷或地面处理链路。
WFOV/NFOV 是面向仿真实现的宽域搜索/窄域凝视抽象。

## 文档导航

- 模块边界、非目标、能力决策与重新进入门、电源命名边界、输出与仿真归属、设计变更规则 →
  [boundaries.md](boundaries.md)
- 数据流图、Public API 边界、与 EOS 的关系、时序（含 runtime patch 状态迁移表）、状态所有权 →
  [data-flow.md](data-flow.md)
- 算法登记表（状态机/WFOV 搜索/NFOV 捕获/ATP/跟踪 EKF+IMM/调度/地球遮挡/foundation/气象/误差模型）、
  每算法的实现边界与反直觉点、刻意不实现的算法 → [algorithms.md](algorithms.md)

跨模块公共规则见 `docs/common/contract.md`。
