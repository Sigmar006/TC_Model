---
Status: active
Last-reviewed: 2026-09-15
Authority: RIR/LRDR 对外交互接口及数据方向
Answers: 雷达如何与干扰机、外部参数设置系统和场景目标交互
---

# Remote Identification Radar 对外交互接口

本文按 LRDR/RIR 的三个外部关系整理接口。仓库中的实现模块名为
`remote_identification_radar`（RIR），下文“雷达”均指该模块。

## 总体关系

```text
干扰机/其他射频设备
  -- RfEmissionFrame -> RF WORLD -> RfSceneFrame --> 雷达

外部配置系统
  -- RirSessionConfig / RirRuntimeConfigPatch -----> 雷达
  <-- bool + RirCycleResult.status/issues ---------- 雷达

场景目标系统
  -- RirSceneTargetList ---------------------------> 雷达
  <-- recognition_outputs / feature_measurements -- 雷达
```

雷达统一通过 `RirSession` 对外。初始化调用 `Create`，运行期设置调用
`TryApplyRuntimeConfig`，逐周期计算调用 `Step` 或 `StepWithResult`。

## 1. 雷达与干扰机

雷达和干扰机不直接调用对方模块接口，也不互相 include 私有头文件。二者通过公共 RF
场景契约和编排层 RF WORLD 间接交换发射事实。

### 干扰机到雷达

| 数据方向 | 外部接口 | 主要内容 | 雷达用途 |
|---|---|---|---|
| 干扰机输出 | `EcmCycleResult::emission_frame` | 干扰发射身份、位置、指向、天线方向图、波形、频率、带宽、功率和时间窗 | 发布到 RF WORLD |
| 编排层转换 | `BuildExternalRfScene` | 汇集当前可见发射，并排除雷达本机 `sensor_entity_id` | 构造雷达外部 RF 场景 |
| 雷达输入 | `RirCycleInput::rf_scene`（`RfSceneFrame`） | `world_cycle_index`、时间窗及 `emissions[]` | 求解频率/时间/空间重叠、接收功率、干扰功率和接收机饱和 |

主数据流：

```text
EcmSession::StepWithResult
  -> EcmCycleResult.emission_frame
  -> PublishEquipmentEmissions
  -> RF WORLD
  -> BuildExternalRfScene
  -> RirCycleInput.rf_scene
  -> TryResolveRirRfFrontEnd
  -> RIR detection cell 的 interference_power_w
```

### 雷达到干扰机/射频世界

| 数据方向 | 外部接口 | 主要内容 | 用途 |
|---|---|---|---|
| 雷达输出 | `RirCycleResult::emission_frame`（`RfEmissionFrame`） | 本周期各驻留实际发射；包含雷达身份、波位、波形、频率、带宽、功率和时间窗 | 发布到 RF WORLD，供其他射频设备按编排顺序消费 |

当前实现中，ECM 的决策输入来自 ESR 观测，不直接消费 RIR 的识别结果或控制命令。因此：

- 存在“ECM 发射影响 RIR 探测”的 RF 物理接口；
- 存在“RIR 发射发布到 RF WORLD”的公共输出接口；
- 不存在 `RirSession -> EcmSession` 的直接控制、任务或识别结果接口。

相关公共类型：`include/1q/electromagnetics/RfScene.h`。示例编排：
`examples/core/rf_world_broker.h`、`examples/components/ecm_sensor_component.cpp`、
`examples/components/rir_sensor_component.cpp`。

## 2. 雷达与外部参数设置系统

外部设置分为“初始化全量配置”和“运行期增量补丁”两条接口。

### 初始化配置

调用接口：

```cpp
RirSession RirSession::Create(const RirSessionConfig& config);
RirSession RirSession::CreateWithDiagnostics(const RirSessionConfig& config,
                                             RirIssueList* issues);
```

`RirSessionConfig` 包含：

| 配置域 | 数据类型 | 设置内容 |
|---|---|---|
| 硬件 | `RirHardwareConfig` | 发射机、天线、接收机、RCS 物理和信号处理增益 |
| 安装/转向 | `RirOrientationConfig` | 雷达可扫描方位和俯仰体积 |
| 任务 | `RirMissionConfig` | 工作模式、扫描中心、最大距离、驻留时间、扫描顺序与任务子窗 |
| 策略 | `RirPolicyConfig` | 检测、关联、滤波、生命周期、识别门限及特征权重 |
| 环境 | `RirEnvironmentConfig` | 天气衰减、大气物理和植被地表条件 |
| 电源 | `sensor_enabled` | 雷达初始开关机状态 |

### 运行期设置

调用接口：

```cpp
bool RirSession::TryApplyRuntimeConfig(const RirRuntimeConfigPatch& patch);
```

| 补丁字段 | 外部设置含义 |
|---|---|
| `has_mission + mission` | 整体覆盖任务配置 |
| `has_work_mode + work_mode` | 切换待机/识别模式 |
| `has_scan_center + scan_center_deg` | 设置转台扫描中心 |
| `has_policy + policy` | 整体覆盖检测、跟踪和识别策略 |
| `has_environment + environment` | 整体覆盖环境事实 |
| `has_sensor_enabled + sensor_enabled` | 开关雷达电源 |
| `has_designated_target_id + designated_external_target_id` | 指定或清除识别目标 |
| `has_designation_duration_cycles + designation_duration_cycles` | 设置指定识别任务窗口，0 表示无限期 |

`TryApplyRuntimeConfig` 返回 `true` 只表示补丁已通过校验并暂存；补丁在下一个成功执行
周期边界生效。外部系统应继续检查 `RirCycleResult::status`、`abort_reason` 和 `issues`，
不能仅依赖补丁调用返回值判断雷达是否完成处理。

逐周期调用接口：

```cpp
RirOutputFrame RirSession::Step(const RirCycleInput& input);
RirCycleResult RirSession::StepWithResult(const RirCycleInput& input);
```

需要诊断、RF 发射、指定任务状态或实际探测距离时使用 `StepWithResult`；只消费双产品
输出时可使用 `Step`。

相关公共类型位于 `include/1q/remote_identification_radar/config/` 和
`include/1q/remote_identification_radar/session/RirSession.h`。

## 3. 雷达与目标

### 目标到雷达

目标系统每周期通过 `RirCycleInput::scene_targets` 传入 `RirSceneTargetList`。

| 输入字段 | 含义 | 消费环节 |
|---|---|---|
| `external_target_id` | 外部目标标识；也用于指定识别任务回联 | 场景回联、任务管理和仿真归属 |
| `target_name` | 可选人读名称 | 调试和真值准确率统计 |
| `position_x/y/z` | 雷达局部 ENU 位置（m） | 斜距/视线角、遮挡、扫描门、回波和量测 |
| `velocity_x/y/z` | ENU 速度（m/s） | 多普勒、建轨速度初值和运动特征 |
| `rcs` | 探测用标量 RCS（m2） | 回波功率和检测概率 |
| `target_swerling_type` | Swerling 0 至 4 起伏类型 | CFAR 检测概率 |
| `aspect_rcs_samples` | 视角 RCS 样本（dBsm） | RCS 识别特征 |
| `polarization_samples` | HH/HV/VH/VV 极化散射样本 | 极化识别特征 |
| `range_rcs_scatterers` | 距离向散射中心 | 一维距离像识别特征 |

雷达平台绝对位置不放在目标记录中，而由 `RirCycleInput::platform_position` 以 ECEF 米制
单独传入，作为目标 ENU 坐标的绝对锚点。

### 雷达到外部目标消费者

| 输出接口 | 标识方式 | 主要结果 | 边界 |
|---|---|---|---|
| `RirOutputFrame::recognition_outputs` | `association_key` | 识别状态、目标类别/型号、置信度和特征得分 | 正式产品出口 |
| `RirOutputFrame::feature_measurements` | `association_key` | RCS、运动、极化、距离像特征及有效掩码 | 正式特征量测出口；不含目标真值 ID/名称 |
| `RirCycleResult::track_attributions` | `association_key <-> external_target_id` | 航迹状态、命中数、滤波位置/速度和周期 SNR | 仿真归属附件，不属于正式产品层 |
| `RirCycleResult::max_detected_slant_range_m` | 周期聚合值 | 本周期已成航迹目标的最大实际斜距 | 效能观测，不是单目标结果 |
| `designation_*` | 外部目标 ID | 指定目标、任务是否激活、是否回扫及回退原因 | 指定识别任务反馈 |

关键边界：

- 雷达不向目标实体发送控制量，目标只是场景事实输入；
- 内部检测点迹和内部航迹不对外发布；
- 正式识别与特征产品只使用稳定的 `association_key`；
- `external_target_id` 和 `target_name` 只通过仿真附件 `track_attributions` 回联，不能混入正式产品；
- 只有 `RirCycleResult::status == kCompleted` 时，本周期产品和归属结果才有效。

## 单周期交互顺序

```text
1. 外部设置系统提交 RirRuntimeConfigPatch（可选）
2. 场景系统更新目标 RirSceneTargetList
3. ECM/其他射频设备发布 RfEmissionFrame 到 RF WORLD
4. 编排层构造 RirCycleInput：平台 ECEF + 目标列表 + 外部 RfSceneFrame
5. 调用 RirSession::StepWithResult
6. 检查 status / abort_reason / issues
7. 发布 RIR emission_frame 到 RF WORLD
8. 消费 recognition_outputs、feature_measurements 和 designation_* 反馈
```
