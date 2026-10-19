# 集成边界划分：1q 库 vs 宿主引擎（2026-10-19）

Status: active
目的：为"把本集成迁移到另一个完全不同的宿主引擎"确定边界——
**宿主只提供环境量；坐标/时间/几何等通用能力一律取自 1q。**
Authority: 本次对 LRDR / OPIR / SBIRSFusionDp 三工程的逐文件审计（约 4,000 行）

## 0. 一句话结论

三工程当前**已经大量使用 `oneq::coordinate::*`**（LRDR 侧基本到位），真正的风险不在
"重复实现太多"，而在**4 处语义漂移点**与**7 类必须随引擎重写的宿主耦合**；
另有 **4 个 1q API 缺口**导致每个集成方都要再抄一遍。

---

## 1. 分层原则（本次确定）

| 层 | 归属 | 内容 |
|---|---|---|
| **环境量** | 宿主引擎 | 平台/目标的 ECEF 位置速度姿态、真值属性（`irar_`/`rcs_`）、仿真时刻、实体 ID、跨组件消息、配置路径 |
| **物理与几何** | **1q** | LLA↔ECEF↔ENU↔NED、ECI↔ECEF、GMST、欧拉角↔旋转矩阵、视线角、双视线交会、雷达方程、噪声与检测 |
| **业务降级策略** | 集成侧 | 单星保活合成位置、引导速度差分、`dt_sec` 差分、严格配对时窗（贴着本场景的判定，1q 无等价物） |

判定基准：**"换一个宿主后这段逻辑还一样吗？"** 一样 → 应是 1q 或集成侧共享头；
不一样 → 宿主接口层。

---

## 2. 集成侧重复实现清单（表 A）

| 文件:行号 | 代码 | 做了什么 |
|---|---|---|
| `sbirsfusiondp_1q.cpp:169` | `GregorianToJulianDay(int,int,int)` | 公历→儒略日（Meeus 简化式） |
| `opir_sensor_1q.cpp:95` | 同上 | **与上一行逐行相同的副本**（已实测确认仅缩进差异） |
| `sbirsfusiondp_1q.cpp:185` / `opir_sensor_1q.cpp:111` | `SimSecondsToJulianDay(CyberJulianType)` | 宿主儒略秒→JD（含 `CyberJulianGetDateTime`） |
| `sbirsfusiondp_1q.cpp:219-235` | `TryRotateEciDirectionToEcef` | ECI 单位方向→ECEF：借 `TryEciToEcef` 中转，因 1q 无"方向版" |
| `opir_sensor_1q.cpp:93` | `kRadToDeg = 57.295779513082320876…` | 弧度→角度（写法 1） |
| `lrdr_sensor_1q.cpp:1185` | `atan2(...) * 57.29577951308232` | 弧度→角度（写法 2，精度位不同） |
| `lrdr_sensor_features.cpp:21,500` | `* 180.0 / kPi` | 弧度→角度（写法 3） |
| `lrdr_sensor_features.cpp:489-502` | `ComputeLookAnglesDeg` | ENU→视线角（`az=atan2(north,east)`、`el=atan2(up,ground)`） |
| `lrdr_sensor_1q.cpp:1181-1186` | 内联同一公式 | **视线角第 2 份**（引导地平线门） |
| `lrdr_sensor_1q.cpp:1141-1157, 1219-1235` | 三连坐标变换 | ECEF→ENU 位置/速度逐字段填 `RirSceneTarget` |
| `sbirsfusiondp_1q.cpp:209` / `opir_sensor_1q.cpp:129` / `lrdr_sensor_1q.cpp:85` | `IsFinite*` 家族 3 份 | 分量有限性检查 |
| `sbirsfusiondp_1q.cpp:927,946` / `lrdr_sensor_1q.cpp:868,895` | 手写 clamp 4 处 | 限幅 |
| `lrdr_sensor_1q.cpp:859-861` | boresight 三次除法归一化 | 1q 注释已写明"求解时会归一化" |
| `lrdr_sensor_features.cpp:548-573` | 极化 100 km 参考距离补偿 | 复刻 1q 私有常量 `kReferenceRangeM=100000` |
| `lrdr_sensor_features.cpp:240-272` | `Hash32/Uniform01/ZeroMeanGaussian/PerturbDb` | 确定性伪随机 RCS 起伏（重跑可复现） |
| `Sensors/` 与 `DataProcessors/` 下 `1q_integration/oneq_diag_log.h` | 126 行 × 2 副本 | Win32 诊断日志 |
| `lrdr_sensor_1q.cpp:237-315` | 6 个 `*Name()` switch（≈78 行） | 1q 枚举→中文字符串，纯诊断 |
| `lrdr_sensor_1q.cpp` 内 `[DIAG-MEAS]`(59 行)/`[DIAG-CUE]`/`[DIAG-RECOG]` | 临时诊断块（≈100 行） | 已登记待删 |

---

## 3. 1q 对应 API（表 B，已核实的公开面）

### 3.1 位置

| API | 单位/参考系 |
|---|---|
| `TryLlaToEcef` / `TryEcefToLla` | lat/lon **deg**，alt 椭球高 **m**，ECEF **m**，WGS-84 |
| `TryEcefToEnu/EcefToNed/EcefToNue` | x=东/y=北/z=天（ENU）等 |
| `TryLlaToEnu/LlaToNed/LlaToNue` | 内部 LLA→ECEF→局部 |
| `TryEnuToEcef` | `TryEcefToEnu` 的逆 |
| `TryBearingRangeToEnuOffset(az_deg, range_m, *)` | az **北偏东**（0=北，90=东） |
| `TryEnuToEcefDirection(dir, origin_lla, *)` | 方向量，**不含原点平移** |
| `ToNed/ToEnu/ToNue` | 纯轴重排 |

### 3.2 速度

| API | 说明 |
|---|---|
| `TryEcefToEnuVelocity` / `TryEnuToEcefVelocity` | 旋转仅由 origin 经纬决定 |
| `TryMakeEcefVelocityFromHeading(heading_deg, speed_mps, lla, *)` | heading 北偏东 |
| `ToNedVelocity/ToEnuVelocity/ToNueVelocity` | 轴重排 |

### 3.3 惯性系与时间

| API | 约定 |
|---|---|
| `TryComputeGmstRad(utc_julian_day, *)` | 入 **JD_UTC>0**，出 **rad∈[0,2π)**，IAU1982（UT1≈UTC） |
| `TryEcefToEci` / `TryEciToEcef` | **纯 R3(θ) 绕 z**；1q 内部已用它处理单位方向（`SbirsBearingAdapter`、`SbirsAcceptanceRecords`） |
| `TryEcefVelocityToEci` | 含输运项 ω×r（漏项 = 0.5 km/s 级误差） |
| `kEarthRotationRateRadPerSec` | 7.292115146706979e-5 |

### 3.4 姿态

| API | 约定 |
|---|---|
| `BuildRotationMatrix(EulerAnglesDeg)` | **Z-Y-X**，deg |
| `ToEulerAnglesDeg(RotationMatrix3d)` | 万向节锁时 roll=0 |
| `Inverse` / `Compose` / `ComposeAttitudeDeg` | deg；复合走矩阵乘 |
| `RotateEnuToLocal` / `RotateLocalToEnu` | deg |
| `ToEnuAttitude` / `ToNedAttitude` | ⚠ 头文件明文警告**不适用于飞机姿态** |

### 3.5 场景一站式

| API | 说明 |
|---|---|
| `TryMakeEnuSceneState(const ExternalKinematics&, anchor_lla, EnuSceneState*)` | 位置支持 ECEF 或 LLA，**速度固定 ECEF**；锚点逐周期重锚 |
| `ExternalKinematics` | `PositionFrame` 选择 + ECEF/LLA + ECEF 速度 + Body→ENU 姿态 |

### 3.6 其它

| API | 说明 |
|---|---|
| `TryComputeDualLosFixM(a_origin, a_dir, b_origin, b_dir, *fix, *residual)` | 双视线交会，残差=公垂线长 |
| `EciDirectionFromAzimuthElevationRad(az, el)` | az∈[0,2π)、el∈[−π/2,π/2] **rad** |

---

## 4. 迁移判定（表 C）

| 项 | 判定 | 理由 / 风险 |
|---|---|---|
| LRDR 站点 ECEF→LLA、目标 ECEF→ENU、引导 LLA→ECEF→ENU | **已调 1q** | 无需改 |
| 三连坐标变换 → `TryMakeEnuSceneState` | **应改为调 1q** | 低；需保留"变换失败置零速度"的降级语义 |
| `TryRotateEciDirectionToEcef` → 直接 `TryEciToEcef` | **应改为调 1q** | 数学逐位等价；1q 内部已直接这么用 |
| `IsFinite*` ×3 | **应改为调 1q** | 1q 有 5 个重载 |
| boresight 三次除法 | **可删**（保留退化门） | 1q 内部会归一化 |
| 限幅 4 处 | **建议等 1q 公开 `Clamp`** | 当前 `src/common/numerics/ClampUtils.h` 私有 |
| `GregorianToJulianDay` ×2 | **必须保留**（1q 无此 API） | 两份逐字副本已有漂移风险 → 先收敛到 `1q_integration/oneq_time_utils.h` |
| `SimSecondsToJulianDay` ×2 | **保留宿主依赖** | ⚠ **兜底分支有缺陷**，见 §5 |
| 姿态 rad→deg | **必须人工确认** | 1q 要 **Body→ECI**；宿主 `orientation_` 参考系未注明，极可能 Body→NED → 存在绕 z 轴 (GMST−经度) 的系统性错位，且**不报错** |
| `ComputeLookAnglesDeg` | **待 1q 公开** | 与 1q 内部 `RirController::ComputeLookAngles` 逐行等价但未导出 |
| 极化 100 km 参考距离补偿 | **待 1q 公开常量** | 1q 改常量会静默失配 |
| `Hash32/Uniform01/...` | **必须保留** | 1q 无公开确定性 PRNG；重跑可复现是本工程诉求 |
| `dt_sec` 差分 ×2 | **保留（工程内去重）** | 1q 的 `dt_sec` 是纯输入 |
| 引导速度差分 | **必须保留** | 融合记录不带速度；1q 无"位置序列→速度"原语 |
| 单星保活合成位置 | **必须保留** | 业务降级策略，贴着本场景 |
| `oneq_diag_log.h` ×2 | **建议换 1q logging 或新引擎日志** | Win32+SEH，非 Windows 直接编不过 |
| 1q 枚举→中文字符串 ×6 | **建议 1q 补 `ToString`** | 纯诊断，每个集成方都要重写 78 行 |
| 临时诊断块 | **迁移时整块删** | 已登记于 `docs/practice/diagnostics_inventory.md` |

**可减行数**：立即可换 ≈65 行；收敛重复（JD/dt_sec/日志头）再并 ≈45 行；
删临时诊断 ≈100 行 ⇒ **约 190–210 行（占四个文件 5%）**。
**但行数不是重点**——重点是消掉下列 4 类语义漂移。

---

## 5. 4 类语义漂移点（迁移前必须先处理）

1. **弧度↔角度有 3 种写法**（`57.295779513082320876…` / `57.29577951308232` / `*180/kPi`）；
2. **公历→JD 有 2 份逐字副本**，且**兜底分支是错的**：

   ```cpp
   if (CyberJulianGetDateTime(...) == CYBER_SUCCESS) { ... }
   return _seconds / 86400.0;      // ← 这不是 JD（缺 2451545.0 纪元偏移）
   ```

   一旦宿主拆时失败，`TryComputeGmstRad` 收到 ≈1.7e-5 的"儒略日"（仍 >0，不会被拒），
   会算出与真实时刻差数千年的 GMST ⇒ **ECI↔ECEF 旋转整体错位且不报错**。
   **建议**：兜底改为返回 `false`，由上层跳过该周期并计数告警。

3. **视线角公式有 2 份**（`features:489-502` 与 `lrdr:1181-1186`），
   1q 内部还有第 3 份；且 az 基准是"东起逆时针"，与融合通道"北起顺时针"不同；
4. **极化 100 km 参考距离**复刻了 1q 私有常量，跨版本脆弱（改了就静默偏移）。

---

## 6. 必须随引擎重写的宿主耦合（7 类）

| # | 类别 | 具体接口 |
|---|---|---|
| 1 | 平台状态 | `GetParentUnit()->GetUnitInfo()` 的 `position_/velocity_/orientation_`（ECEF m、m/s、**rad**）、`GetCurrentSimTime()`、`GetEntityId/GetUnitNameCString` |
| 2 | 目标真值 | `ObjectMGR()->GetUnitInfoInExtrapolate()`（含 `irar_`/`rcs_`/`active_`）、`GetRelationship()` |
| 3 | 跨组件通信 | `CyberMalImpl`（指针载荷）、`CyberDataProcessorImpl`（Query/Clone/EmployModel/GetRecordCount/GetRecord）、`GenerateRecorder` |
| 4 | 时间换算 | `CyberJulianUtils::CyberJulianGetDateTime`（JD 的唯一来源） |
| 5 | 配置路径 | `oneq_config_path.h`（exe 目录+环境变量+CWD）、`sbirs/rir config_loader.h` |
| 6 | 诊断日志 | `oneq_diag_log.h`（Win32+SEH）、`LrdrDiagLog` |
| 7 | 业务降级 | 单星保活、引导速度差分、`dt_sec` 差分、严格配对时窗 |

每工程约 **150–400 行**需重写。

---

## 7. 建议向 1q 提的 4 个 API 缺口

| # | 缺口 | 建议 |
|---|---|---|
| 1 | 无公历→JD | `coordinate/time_transform.h::TryGregorianToJulianDay(y,m,d,h,mi,s,double*)`——已产生 2 份副本 |
| 2 | 无 ECI→ECEF **方向版** | `inertial_transform.h::TryEciToEcefDirection(const Vector3d&, gmst, Vector3d*)`，对标已有的 `TryEnuToEcefDirection` |
| 3 | 无 Body→NED ↔ Body→ECI | `attitude_transform.h` 补该变换（需 lla+gmst），或在 `satellite_attitude_eci_body_deg` 旁写明换算方法 |
| 4 | `numerics` 私有 | 把 `src/common/numerics/{Constants,ClampUtils,NumericGuard}.h` 提升到 `include/1q/common/numerics/`；并公开 `ComputeLookAngles`、`Vector3d::Norm/Distance`、`PolarizationFeatureExtractor::kReferenceRangeM` |

---

## 8. 迁移清单（建议顺序）

1. **先修 §5 的 4 类漂移**（尤其 JD 兜底），再动结构——否则迁移会把隐性错误一并带走；
2. 把 `GregorianToJulianDay` / `SimSecondsToJulianDay` / `dt_sec` / `oneq_diag_log.h` 收敛到
   `1q_integration/` 下的单一实现；
3. 三连坐标变换改 `TryMakeEnuSceneState`，删 `TryRotateEciDirectionToEcef`、`IsFinite*`、boresight 归一化；
4. 抽宿主接口层（§6 的 7 类）为一组纯虚接口，三工程只依赖接口
   —— 新引擎只需实现该接口，模型代码零改动；
5. 删临时诊断块；
6. 向上游提 §7 的 4 个 API 缺口。
