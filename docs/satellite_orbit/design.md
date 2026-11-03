---
Status: active
Last-reviewed: 2026-11-03
Authority: satellite_orbit 设计权威入口
Answers: satellite_orbit 是什么、为谁提供卫星几何基准、与其他模块怎么协作
---

# Satellite Orbit 设计

`satellite_orbit` 是 **OPIR 卫星轨道运动模型**：由轨道六根数与历元推进卫星状态，
逐周期输出 ECEF 位置与速度，为探测链提供时变几何基准。

设计依据：《探测仿真系统软件设计说明》5.1.1「模拟 OPIR 轨道运动能力」——
轨道传播由 `OrbitPropagator` 类承担，该类持有轨道根数与积分步长作为内部状态，
逐周期接收 UTC 儒略日，从历元状态推进至目标时刻并输出 ECEF 位置与速度。

## 定位

- **有状态模型**：与坐标变换等无状态函数不同，历元与当前积分状态跨周期保持，
  仅在重新初始化或配置变更时重置。
- **只负责动力学与坐标输出**：不感知传感器类型、探测链或场景目标；ECEF→ECI 的
  目标旋转、探测几何与辐射计算由消费方（如 `sbirs_sensor`）完成。
- **共用时间基准**：输出 ECEF 使用 IAU 1982 GMST（UT1 ≈ UTC），与
  `1q/coordinate/inertial_transform.h` 同一约定，保证与探测链 GMST 一致。

## 与 SBIRS 的协作

`SbirsSessionConfig` 的轨道根数与历元字段为传播器提供初始基准，
`SbirsCycleInput` 的卫星 ECEF 状态字段承载传播器输出
（设计说明 5.1.1「卫星轨道传播」）。

**实现状态**：本模块当前只交付传播器本身；`sbirs_sensor` 的配置字段与会话内
传播器接线尚未完成，属后续独立变更。在该接线完成前，调用方需自行调用
`OrbitPropagator::PropagateToTime` 并把结果填入周期输入。

## 公共接口

| 符号 | 用途 |
|---|---|
| `satellite_orbit::OrbitalElements` | 轨道六根数（a, e, i, Ω, ω, M） |
| `satellite_orbit::EciState` / `EcefState` | ECI / ECEF 位置与速度状态 |
| `satellite_orbit::OrbitPropagator` | 传播器：`Initialize` / `PropagateToTime` / `SetStepSize` / `SetJ2Enabled` |

详见 [boundaries.md](boundaries.md) 与 [algorithms.md](algorithms.md)。
