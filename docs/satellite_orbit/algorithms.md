---
Status: active
Last-reviewed: 2026-11-03
Authority: satellite_orbit 算法清单与参数表
Answers: 轨道怎么由根数推进、J2 与 GMST 怎么算、参数取值来自哪里
---

# Satellite Orbit 算法

## 算法清单

| 算法 | 输入 | 输出 | 证据 |
|---|---|---|---|
| 根数 → 历元 ECI 状态 | 轨道六根数 | ECI 位置/速度 | `src/satellite_orbit/OrbitPropagator.cpp`；`tests/unit/satellite_orbit/orbit_propagator_test.cpp` |
| 二体 + J2 加速度 | ECI 位置 | ECI 加速度 | 同上 |
| 经典 RK4 积分 | 状态、步长、J2 开关 | 下一时刻状态 | 同上 |
| GMST 旋转 ECI → ECEF | ECI 状态、UTC 儒略日 | ECEF 位置/速度 | 同上 |

## 1. 根数 → 历元 ECI 状态

平近点角按 $M \bmod 2\pi$ 归一化后，用牛顿迭代解开普勒方程
$E - e\sin E = M$（收敛阈值 $10^{-12}$ rad，迭代上限 100）。真近点角

$$
\nu = \operatorname{atan2}\left(\sqrt{1-e^2}\sin E,\ \cos E - e\right)
$$

半径 $r = a(1 - e\cos E)$；近焦点系下位置 $\mathbf r = (r\cos\nu, r\sin\nu, 0)$，
速度按 $p = a(1-e^2)$ 与 $\sqrt{\mu/p}$ 分解为切向与径向分量。最后按
$R_3(\Omega)\,R_1(i)\,R_3(\omega)$ 旋转到 ECI。

## 2. 加速度

$$
\ddot{\mathbf r} = -\frac{\mu}{r^3}\mathbf r + \mathbf a_{J2},\qquad
\mathbf a_{J2} = -\frac{3}{2}J_2\mu\frac{R_e^2}{r^5}
\begin{bmatrix} x\left(1 - 5z^2/r^2\right) \\ y\left(1 - 5z^2/r^2\right) \\ z\left(3 - 5z^2/r^2\right) \end{bmatrix}
$$

J2 项可在配置中关闭（`SetJ2Enabled(false)`），仅保留二体中心引力。

## 3. 数值积分

经典四阶龙格-库塔；单步步长取 `min(step_size_sec, 剩余时长)`，
剩余时长不足 $10^{-9}$ s 时结束。每步后校验状态有限，溢出即失败返回。

## 4. ECEF 输出

先按目标时刻求 GMST，位置经 $R_3(-\theta)$ 旋转；速度补偿地球自转输运项：

$$
\mathbf v_{ECEF} = R_3(-\theta)\left(\mathbf v_{ECI} - \boldsymbol\omega_e \times \mathbf r_{ECI}\right)
$$

该输出与 `TryEcefVelocityToEci` 互逆，保证 ECEF/ECI 往返一致。

## 参数表

| 参数 | 取值 | 来源 |
|---|---|---|
| 地球引力常数 $\mu$ | $3.986004418\times10^{14}$ m³/s² | WGS-84 |
| J2 带谐系数 | $1.08262668\times10^{-3}$ | 无量纲 |
| 地球赤道半径 $R_e$ | 6 378 137.0 m | WGS-84 |
| 默认积分步长 | 10 s | `step_size_s_` 初值 |
| 步长正下限 | $10^{-3}$ s | `kMinStepSizeSec` |
| GMST 模型 | IAU 1982（UT1 ≈ UTC） | `1q/coordinate/inertial_transform.h` |
