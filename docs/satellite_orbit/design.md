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

`SbirsSessionConfig::orbit`（`SbirsOrbitConfig`）的轨道根数与历元字段为传播器提供初始基准，
`SbirsCycleInput` 的卫星 ECEF 状态字段承载传播器输出
（设计说明 5.1.1「卫星轨道传播」）。

**实现状态**：已接线。启用 `SbirsSessionConfig::orbit.propagation_enabled` 后，
`SbirsSession` 内部持有本模块的 `OrbitPropagator`，调用方每周期调用
`SbirsSession::PropagateSatelliteState(utc_julian_day, &input)` 推进卫星状态并写入
`SbirsCycleInput` 的 `utc_julian_day` / `satellite_position_ecef_m` /
`satellite_velocity_ecef_m_per_s`。未启用（默认）时传播器不存在，卫星状态仍由调用方
逐周期提供，既有必填契约不变。配置非法时会话不建立传播器（fail-closed），
`CreateWithDiagnostics` 以 `sbirs.validation.invalid_orbit_*` 报告。

## 公共接口

| 符号 | 用途 |
|---|---|
| `satellite_orbit::OrbitalElements` | 轨道六根数（a, e, i, Ω, ω, M） |
| `satellite_orbit::EciState` / `EcefState` | ECI / ECEF 位置与速度状态 |
| `satellite_orbit::OrbitPropagator` | 传播器：`Initialize` / `PropagateToTime` / `SetStepSize` / `SetJ2Enabled` |

详见 [boundaries.md](boundaries.md) 与 [algorithms.md](algorithms.md)。
