# 验收日志打印后续开发指导手册

状态：待开发任务清单（2026-09-28）。本文是交接依据，不表示代码已经完成。用例字段、判据和示例以 [验收用例主文档](functional_acceptance_test_criteria_ocr.md) 为准；截图转写见 [输出路径与日志要求](acceptance_output_paths_and_log_requirements_from_screenshots.md)。开发时先核对主文档中每项的“目标输出文件”，不要把旧代码的实际文件名当作目标文件名。

## 1. 目标输出矩阵

| 用例 | 目标物理文件 | 注意事项 |
|---|---|---|
| 1–18、19A、20–27（含 24A–24C） | `log/SatelliteTarget.log` | Fusion、Inference、Precision 的独立旧日志不再作为验收文件；第 11–18 项不双写 |
| 19、21 的雷达引导段、28–49（含 37A、47A） | `log/RadarOutput.log` | 第 28 项同时输出方向图 CSV；第 48 项同时输出扫描 CSV |
| 28 | `log/RadarAntennaGainPatterndata.csv` | 文件名大小写和 `Patterndata` 拼写按主文档 |
| 48 | `log/Rir_scan_pattern.csv` | 与 `RadarOutput.log` 中的扫描摘要共同核对 |
| 57、58、67、68 | `log/IRExternalInterface.log` | 红外相关外部接口；第 68 项的聚合边界待实现 |
| 61–66 | `log/RadarExternalInterface.log` | 雷达相关外部接口；不得用 `RadarOutput.log` 代替 |
| 70–73 | `log/RadarInternalInterface.log` | 雷达内部接口 |
| 74–77 | `log/IRInternalInterface.log` | 红外内部接口 |
| 50–55、56、60 | 无专用验收日志 | 性能项和仿真控制/时钟项由测试夹具及报告判定 |

以上是**目标状态**。不要新增 `Fusion_acceptance.log`、`inference_acceptance.log`、`precision_acceptance.log`、`Opir_acceptance.log`、`SatellitePredict.log` 等独立验收文件。截图中 `MissileBallisticData.log` 等辅助文件不属于本轮库内落盘目标。

## 2. 当前代码与待办

> **2026-09-28 进展登记**：下表 P0/P1 中「代码入口」一列的原始描述保留为任务定义；
> 实际完成情况见本表末列「状态」。

| 优先级 | 工作项 | 代码入口与完成标准 | 状态 |
|---|---|---|---|
| P0 | 统一物理通道和路径 | `src/common/logging/AcceptanceFileLog.h/.cpp` 当前仍有 Fusion、Inference、Precision 独立通道与旧文件名，缺少雷达/红外外部接口通道。增加目标通道、路径和目录写入测试；保留编译开关的功能控制，但不能因多通道并写造成覆盖或重复。 | **已完成**：通道收敛为 6 个（`kSatelliteTarget`/`kRadarOutput`/`kRadarExternalInterface`/`kIrExternalInterface`/`kRadarInternalInterface`/`kIrInternalInterface`）；旧环境变量作为兼容回退保留；`acceptance_record_format_test.cpp` 增加六文件目录作用域与接口分文件用例。验证：`1q_lib` 构建通过；测试文件单独编译通过（common 测试 target 因 `rapidcheck` 依赖缺失、生成头需 C++17 而无法整体构建，属既有环境问题）。 |
| P0 | 61–66 雷达外部接口分流 | `src/remote_identification_radar/runtime/RirAcceptanceLog.h` 的现有宏路由到 `kRadarOutput`；将外部接口项单独路由到 `RadarExternalInterface.log`。逐项核对 62 场景、63 干扰、64 点迹、65 航迹、66 资源字段；61 的完整配置清单和应答规则须先冻结。 | **路由已完成**：`RIR_EXTERNAL_INTERFACE_ITEM` 已改指 `kRadarExternalInterface`（62–66 现场生效）。**61 待冻结**：配置回显字段与应答规则未定义，不能凭现有字段片段判整项通过。 |
| P0 | 57、58、67、68 红外外部接口分流 | `src/sbirs_sensor/pipeline/SbirsAcceptanceLog.h` 当前支持 `SatelliteTarget`、融合和内部接口，但没有红外外部接口目标通道。57、58、67 分层投影；68 需要明确跨模块聚合责任与稳定目标键。57 的配置回显字段须先冻结。 | **58、67 已完成**：新增 `SBIRS_IR_EXTERNAL_INTERFACE_ITEM` → `IRExternalInterface.log`；58 投影目标数量/状态数组/辐射特征/背景辐射/大气透过率；67 分三层输出原始观测、归属诊断（显式标注非被动红外测距）与跟踪状态（IMM/EKF/角度 CV-KF 状态估计与协方差）。**57 待冻结**、**68 待定聚合边界**。 |
| P0 | 融合/推演/精度统一落盘 | `src/fusion/FusionAcceptanceLog.h`、`src/target_inference/InferenceAcceptanceLog.h`、`src/precision_evaluation/PrecisionEvaluationLog.h` 仍路由到旧通道；`SbirsAcceptanceLog.h` 仍可能双写融合通道。统一写 `SatelliteTarget.log`，验证同一目录、并发写入、单条记录完整性、无重复。 | **已完成**：三个模块宏改指 `kSatelliteTarget`；`SBIRS_DUAL_ACCEPTANCE_ITEM` 由“两通道各写一次”改为“任一开关开启只写一次”，消除归并后同文件重复行；`examples/logger/acceptance_paths.h` 同步为六文件。 |
| P1 | 第 29 项三种功率 | `src/remote_identification_radar/runtime/RirAcceptanceRecords.cpp` 当前第 29 项只写回波功率。逐目标同一行补目标 ID、斜距、RCS、回波功率、噪声功率、干扰功率；用同距离 `1 m²` 与 `0.025 m²` 样本验证主文档中的差值判据。 | **代码已完成**：同一行写六项。差值判据（`1 m²` vs `0.025 m²` → 约 `16 dB`）属测试夹具/外部验证程序口径，待固定场景样本核对。 |
| P1 | 第 28 项内部方向图一致性 | 从**实际生成的方向图采样**反算可配置特征并与外部参数逐项比较，按主文档的 `5%` 判据给出各项及整体结果。不要把“打印峰值增益值”当作验收要求，也不要用配置输入自身冒充反算值。保留方向图 CSV 供追溯。 | **代码已完成**：新增 `WriteRirAntennaPatternDerived`，从实际采样反算峰值增益、方位/俯仰 -3 dB 宽度、最大旁瓣电平、后瓣电平，逐项输出相对误差与整体判据；配置输入不参与冒充。 |
| P1 | 第 36、37/37A、39/40 项 | 第 36 项记录检测概率 90% 最大探测距离及依据的 RCS，并用物理关系/数值用例核对计算方向；第 37A 项独立记录地平线搜索；第 39/40 项定位字段为距离、方位、**俯仰**，不能以高度替代。 | **第 36、37/37A、39/40 项已完成**（第 36 项由 Marcum Q 反解所需 SNR 再按四次方根距离定律缩放；37A 以地平线波位独立成行；39/40 已删除 `高度=`）。数值用例核对待补。 |
| P1 | 其他用例字段补齐 | 按主文档逐项清点 1–27、28–49、57–77 的“能够记录”字段；对缺失项补实现和断言，不用示例文本代替实际测试。特别检查第 8 项速度、第 12–14 项视线角误差与驻留时间、第 20 项误差椭圆、第 21 项探测角度。 | **第 8 项已完成**（补真值/实际速度 ECEF 与速度误差）、**第 9 项已完成**（补 arcsec 残差）、**第 20 项已覆盖**（误差椭圆与落点距离误差已写出）；**第 21 项维持 §0B.3 外部依赖**（RIR 输入无 SBIRS→RIR 引导消息与时序字段）；**第 12–14 项**驻留时间与 WFOV/NFOV 视线角已写出，误差量由外部验证程序计算。 |

## 3. 推荐实施与验收节奏

1. 先完成公共日志通道，构造临时输出目录测试六类目标文件的路径、文件名、开关和并发写入；确认没有旧独立文件或重复记录。
2. 按仓库模块节奏逐个推进 RIR、SBIRS、Fusion、Inference、Precision；每完成一个模块先构建并运行对应单元测试，再继续下一个模块。外部编排项单独验收。
3. 使用同一固定场景分别核对“模型产出值—日志字段—验证程序判据”。每项至少有正例、缺字段反例和边界值反例；日志首部、周期、目标 ID、单位必须可机器解析。
4. 最后对照主文档的用例目录和附录索引逐项验收：目标文件存在、字段齐全、文件名与大小写一致；无日志项不误生成专用日志。对尚未冻结的 57、61 及 68 聚合边界记录明确的待决问题，不以现有字段片段判定整项通过。

## 4. 尚未完成 / 需先冻结的事项（2026-09-28 收口）

| 事项 | 阻塞原因 | 需要谁先决定 |
|---|---|---|
| 第 57 项 卫星平台与载荷配置接口 | 主文档未给出必测配置字段、单位、方向与应答规则；缺全量配置回显契约 | 需求方冻结字段清单 |
| 第 61 项 雷达平台与工作参数配置接口 | 同上（配置字段、可热改范围、应答规则未定义） | 需求方冻结字段清单 |
| 第 68 项 预警预测与精度评估接口 | 字段跨 inference/precision/threat 多个模型与日志，缺统一聚合责任方与稳定目标键；主文档要求“新增稳定预警消息 DTO 或由编排层聚合”后方可验收 | 需求方/架构确定聚合边界 |
| 第 21 项 卫星探测引导段 | RIR 当前周期输入没有 SBIRS→RIR 引导消息及投递时序字段（§0B.3 已登记为模型不可见） | 仿真消息总线/编排层提供 |
| 第 11 项 相邻探测时间间隔方向 | `≥1 s` 与截图 `≤1 s` 冲突（§0C.2 裁定 2） | 需求方复核 |
| 数值用例与反例 | 第 29 项 `16 dB` 差值、第 28 项 `5%` 反算、第 36 项距离方向核对均需固定场景样本与外部判据 | 测试夹具/验证程序 |
| 单元测试执行 | 本 build tree 的测试 target 缺 `rapidcheck` 依赖、生成头需 `/std:c++17`（工程为 C++14），改动前即不可构建 | 构建环境负责人 |

## 5. 本次改动的验证方式

1. `cmake --build build/acceptance-v141 --config Release --target 1q_lib` 全库构建通过
   （接受 RIR/OPIR 开关为 ON；融合/推演/精度开关在本 tree 为 OFF，故其代码路径按宏剪除编译）。
2. `tests/unit/common/acceptance_record_format_test.cpp` 用工程同款包含路径与 `cl /std:c++14`
   单独编译通过（含六文件目录作用域、外部/内部接口分文件、首开截断重开追加用例）。
3. **未执行**：验收日志的端到端运行核对（需固定场景样本）与单元测试运行（见上表阻塞项）。

本指南的「状态」列记录已完成范围；未标注完成的事项不得视为已实现。
