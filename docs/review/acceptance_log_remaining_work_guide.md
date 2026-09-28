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

| 优先级 | 工作项 | 代码入口与完成标准 |
|---|---|---|
| P0 | 统一物理通道和路径 | `src/common/logging/AcceptanceFileLog.h/.cpp` 当前仍有 Fusion、Inference、Precision 独立通道与旧文件名，缺少雷达/红外外部接口通道。增加目标通道、路径和目录写入测试；保留编译开关的功能控制，但不能因多通道并写造成覆盖或重复。 |
| P0 | 61–66 雷达外部接口分流 | `src/remote_identification_radar/runtime/RirAcceptanceLog.h` 的现有宏路由到 `kRadarOutput`；将外部接口项单独路由到 `RadarExternalInterface.log`。逐项核对 62 场景、63 干扰、64 点迹、65 航迹、66 资源字段；61 的完整配置清单和应答规则须先冻结。 |
| P0 | 57、58、67、68 红外外部接口分流 | `src/sbirs_sensor/pipeline/SbirsAcceptanceLog.h` 当前支持 `SatelliteTarget`、融合和内部接口，但没有红外外部接口目标通道。57、58、67 分层投影；68 需要明确跨模块聚合责任与稳定目标键。57 的配置回显字段须先冻结。 |
| P0 | 融合/推演/精度统一落盘 | `src/fusion/FusionAcceptanceLog.h`、`src/target_inference/InferenceAcceptanceLog.h`、`src/precision_evaluation/PrecisionEvaluationLog.h` 仍路由到旧通道；`SbirsAcceptanceLog.h` 仍可能双写融合通道。统一写 `SatelliteTarget.log`，验证同一目录、并发写入、单条记录完整性、无重复。 |
| P1 | 第 29 项三种功率 | `src/remote_identification_radar/runtime/RirAcceptanceRecords.cpp` 当前第 29 项只写回波功率。逐目标同一行补目标 ID、斜距、RCS、回波功率、噪声功率、干扰功率；用同距离 `1 m²` 与 `0.025 m²` 样本验证主文档中的差值判据。 |
| P1 | 第 28 项内部方向图一致性 | 从**实际生成的方向图采样**反算可配置特征并与外部参数逐项比较，按主文档的 `5%` 判据给出各项及整体结果。不要把“打印峰值增益值”当作验收要求，也不要用配置输入自身冒充反算值。保留方向图 CSV 供追溯。 |
| P1 | 第 36、37/37A、39/40 项 | 第 36 项记录检测概率 90% 最大探测距离及依据的 RCS，并用物理关系/数值用例核对计算方向；第 37A 项独立记录地平线搜索；第 39/40 项定位字段为距离、方位、**俯仰**，不能以高度替代。 |
| P1 | 其他用例字段补齐 | 按主文档逐项清点 1–27、28–49、57–77 的“能够记录”字段；对缺失项补实现和断言，不用示例文本代替实际测试。特别检查第 8 项速度、第 12–14 项视线角误差与驻留时间、第 20 项误差椭圆、第 21 项探测角度。 |

## 3. 推荐实施与验收节奏

1. 先完成公共日志通道，构造临时输出目录测试六类目标文件的路径、文件名、开关和并发写入；确认没有旧独立文件或重复记录。
2. 按仓库模块节奏逐个推进 RIR、SBIRS、Fusion、Inference、Precision；每完成一个模块先构建并运行对应单元测试，再继续下一个模块。外部编排项单独验收。
3. 使用同一固定场景分别核对“模型产出值—日志字段—验证程序判据”。每项至少有正例、缺字段反例和边界值反例；日志首部、周期、目标 ID、单位必须可机器解析。
4. 最后对照主文档的用例目录和附录索引逐项验收：目标文件存在、字段齐全、文件名与大小写一致；无日志项不误生成专用日志。对尚未冻结的 57、61 及 68 聚合边界记录明确的待决问题，不以现有字段片段判定整项通过。

本指南仅整理待办；没有修改代码或证明当前输出已经满足这些要求。
