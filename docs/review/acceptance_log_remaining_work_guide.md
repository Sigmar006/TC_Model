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
>
> **复核修正**：状态按“代码存在 + 计算口径正确 + 必需字段已落盘 + 有针对性测试”四项判断。
> 只有代码入口存在、但公式错误、字段不全或没有验证的，不再标为“已完成”。

| 优先级 | 工作项 | 代码入口与完成标准 | 状态 |
|---|---|---|---|
| P0 | 统一物理通道和路径 | `src/common/logging/AcceptanceFileLog.h/.cpp` 当前仍有 Fusion、Inference、Precision 独立通道与旧文件名，缺少雷达/红外外部接口通道。增加目标通道、路径和目录写入测试；保留编译开关的功能控制，但不能因多通道并写造成覆盖或重复。 | **已完成**：通道收敛为 6 个（`kSatelliteTarget`/`kRadarOutput`/`kRadarExternalInterface`/`kIrExternalInterface`/`kRadarInternalInterface`/`kIrInternalInterface`）；旧环境变量作为兼容回退保留；`acceptance_record_format_test.cpp` 增加六文件目录作用域与接口分文件用例。验证：`1q_lib` 构建通过；测试文件单独编译通过（common 测试 target 因 `rapidcheck` 依赖缺失、生成头需 C++17 而无法整体构建，属既有环境问题）。 |
| P0 | 61–66 雷达外部接口分流 | `src/remote_identification_radar/runtime/RirAcceptanceLog.h` 的现有宏路由到 `kRadarOutput`；将外部接口项单独路由到 `RadarExternalInterface.log`。逐项核对 62 场景、63 干扰、64 点迹、65 航迹、66 资源字段；61 的完整配置清单和应答规则须先冻结。 | **路由已完成**：`RIR_EXTERNAL_INTERFACE_ITEM` 已改指 `kRadarExternalInterface`（62–66 现场生效）。**61 待冻结**：配置回显字段与应答规则未定义，不能凭现有字段片段判整项通过。 |
| P0 | 57、58、67、68 红外外部接口分流 | `src/sbirs_sensor/pipeline/SbirsAcceptanceLog.h` 当前支持 `SatelliteTarget`、融合和内部接口，但没有红外外部接口目标通道。57、58、67 分层投影；68 需要明确跨模块聚合责任与稳定目标键。57 的配置回显字段须先冻结。 | **58、67 已完成**：新增 `SBIRS_IR_EXTERNAL_INTERFACE_ITEM` → `IRExternalInterface.log`；58 投影目标数量/状态数组/辐射特征/背景辐射/大气透过率；67 分三层输出原始观测、归属诊断（显式标注非被动红外测距）与跟踪状态（IMM/EKF/角度 CV-KF 状态估计与协方差）。**57 待冻结**、**68 待定聚合边界**。 |
| P0 | 融合/推演/精度统一落盘 | `src/fusion/FusionAcceptanceLog.h`、`src/target_inference/InferenceAcceptanceLog.h`、`src/precision_evaluation/PrecisionEvaluationLog.h` 仍路由到旧通道；`SbirsAcceptanceLog.h` 仍可能双写融合通道。统一写 `SatelliteTarget.log`，验证同一目录、并发写入、单条记录完整性、无重复。 | **已完成**：三个模块宏改指 `kSatelliteTarget`；`SBIRS_DUAL_ACCEPTANCE_ITEM` 由“两通道各写一次”改为“任一开关开启只写一次”，消除归并后同文件重复行；`examples/logger/acceptance_paths.h` 同步为六文件。 |
| P1 | 第 29 项三种功率 | `src/remote_identification_radar/runtime/RirAcceptanceRecords.cpp` 当前第 29 项只写回波功率。逐目标同一行补目标 ID、斜距、RCS、回波功率、噪声功率、干扰功率；用同距离 `1 m²` 与 `0.025 m²` 样本验证主文档中的差值判据。 | **代码已完成**：同一行写六项。差值判据（`1 m²` vs `0.025 m²` → 约 `16 dB`）属测试夹具/外部验证程序口径，待固定场景样本核对。 |
| P1 | 第 28 项内部方向图一致性 | 从**实际生成的方向图采样**反算可配置特征并与外部参数逐项比较，按主文档的 `5%` 判据给出各项及整体结果。不要把“打印峰值增益值”当作验收要求，也不要用配置输入自身冒充反算值。保留方向图 CSV 供追溯。 | **已完成（2026-09-28 修复）**：删除旧摘要输出 `WriteRirAntennaPatternSummary`（配置值直读，函数与声明均已移除）；峰值增益改为主瓣范围二维细扫取采样最大值，-3 dB 宽度以采样峰值为基准；日志只保留 雷达ID + 各项「反算值/配置值/判据」+ 整体判据。验证：`rir_acceptance_calculations_test.cpp` 覆盖采样峰值一致性、5% 内宽度、配置变窄时宽度随之变小（证明宽度来自采样）；并独立链接 `1q.lib` 实跑通过（采样峰值 52.0000 dBi、az/el 宽度均 4.000°）。 |
| P1 | 第 36、37/37A、39/40 项 | 第 36 项记录检测概率 90% 最大探测距离及依据的 RCS，并用物理关系/数值用例核对计算方向；第 37A 项独立记录地平线搜索；第 39/40 项定位字段为距离、方位、**俯仰**，不能以高度替代。 | **已完成（2026-09-28 修复）**：距离公式方向修正为 `R_now·(SNR_now/SNR_req)^(1/4)`，并抽出可测函数 `TryComputeDetectionRangeAtPd`。验证（独立链接 `1q.lib` 实跑，全部 PASS）：所需 SNR 上距离等于当前距离；SNR 升高距离单调不减；低于所需时距离变小；+8 dB 距离比 = `10^0.2` = 1.584893；非法入参原子拒绝。37/37A 分项、39/40 距离/方位/俯仰均已就位。 |
| P1 | 其他用例字段补齐 | 按主文档逐项清点 1–27、28–49、57–77 的“能够记录”字段；对缺失项补实现和断言，不用示例文本代替实际测试。特别检查第 8 项速度、第 12–14 项视线角误差与驻留时间、第 20 项误差椭圆、第 21 项探测角度。 | **字段部分已补，驻留判据仍未完成**：第 12 项已补宽场角误差，第 13 项已补脱靶量模和窄场角误差，第 14 项已补镜头转向时间；但现有样例日志出现 `驻留=1.450s`，部分运行出现 `20.000s`，均不满足 `0.5 s<驻留时间<1 s`。必须修正场景参数或计算/校验逻辑并重跑，不得以文档中的 `0.750s` 示例认定通过。第 8、9、20 项此前已覆盖；**第 21 项仍依赖外部消息总线**。 |

## 3. 推荐实施与验收节奏

1. 先完成公共日志通道，构造临时输出目录测试六类目标文件的路径、文件名、开关和并发写入；确认没有旧独立文件或重复记录。
2. 按仓库模块节奏逐个推进 RIR、SBIRS、Fusion、Inference、Precision；每完成一个模块先构建并运行对应单元测试，再继续下一个模块。外部编排项单独验收。
3. 使用同一固定场景分别核对“模型产出值—日志字段—验证程序判据”。每项至少有正例、缺字段反例和边界值反例；日志首部、周期、目标 ID、单位必须可机器解析。
4. 最后对照主文档的用例目录和附录索引逐项验收：目标文件存在、字段齐全、文件名与大小写一致；无日志项不误生成专用日志。对尚未冻结的 57、61 及 68 聚合边界记录明确的待决问题，不以现有字段片段判定整项通过。

## 4. 尚未完成 / 需先冻结的事项（2026-09-28 第三次收口）

> 第 28、36 项已修复并验证；第 12–14 项字段已补，驻留时间按用户裁定（方案 A：保留公式、
> 调场景配置）已收口，详见下方「第 12、14 项驻留时间收口」。下表只保留**尚未完成**或
> **必须由他人先决定**的事项。

**第 12、14 项驻留时间收口（2026-09-28，用户裁定方案 A）**

- 判据：`0.5 s < 驻留 < 1 s`（开区间）；实现口径：`驻留 = 宽场方位视场 / 扫描速率`。
- 缺陷：所有 SBIRS 场景都算不出合规值——`8.7°/6.0°/s = 1.450 s`（与实际日志一致），
  另有 `20°/10°/s = 2.000 s`、`20°/20°/s = 1.000 s`（恰好 1 s 也不在开区间）。
- 修复：**公式不改**（与主文档写法一致），改场景扫描速率取区间中值——
  `sbirs_triple_sat_fix_messages`、`sbirs_inframe_dual_target` 三颗星各 `6.0 → 12.0 °/s`
  （8.7/12 = `0.725 s`）；`examples/basic_config/sbirs.json` `10.0 → 26.0 °/s`
  （20/26 = `0.769 s`）。
- 未改：`lrdr_multi_sbirs_acceptance` 的 `scan_rate = 0`（`scan_span = 0`，即不扫描的固定
  指向验收配置，本判据不适用）；测试已明确「速率为 0 不算合规」。
- 回归保护：新增 `tests/unit/sbirs_sensor/sbirs_scan_dwell_test.cpp`——锁定三处已交付配置
  落在开区间内，把 `8.7/6.0 = 1.45 s` 记为**必须被拒**的反例，并覆盖过慢、过快、恰好 1 s、
  速率为 0 四种边界；该文件用工程同款包含路径与 `cl /std:c++14` 单独编译通过。

**本轮端到端核对发现并修复（2026-09-28）**

| 缺陷 | 现象与根因 | 修复 |
|---|---|---|
| 会话配置留空时 `--output-dir` / 环境变量钉扎失效 | `ScopedAcceptanceLogOutputDirectory` 对空字符串也执行覆盖，把线程内目录清空，验收日志退回进程工作目录的编译期默认路径；表现为「同一场景的 integration 日志在 `--output-dir`，验收日志却落到会话配置目录」 | 空字符串改为「不指定目录」：构造与析构都不改动当前作用域，继续按 环境变量 → 编译期默认路径 解析 |
| 字符串常量里的 `²`（U+00B2）在日志里变成 `?` | 该字符不在 GBK 码页内，MSVC 把它转写成 `?` 后编入库中，落盘即 `目标RCS=0.100m?` | 所有**字符串常量**中的 `U+00B2` 换成 `^2`（`目标RCS=0.100m^2`）；注释中的非 GBK 字符无害可保留（源文件带 BOM 时按 UTF-8 解码） |
| 运行器完成提示仍列旧文件名 | 提示写 `fusion/inference/precision_acceptance.log`，与实际六文件模型不符 | 提示改为六个目标文件 + 两份 CSV |

**端到端核对结果（`rir_ground_site_recognition`，120 周期，本次构建）**

- 产物集合精确为 `RadarOutput.log`、`RadarExternalInterface.log`、`RadarInternalInterface.log`、
  `RadarAntennaGainPatterndata.csv`、`Rir_scan_pattern.csv`；**未生成** `fusion_acceptance.log`、
  `inference_acceptance.log`、`precision_acceptance.log`、`opir_acceptance.log`、
  `rir_acceptance.log`、`RadarInterface.log`。
- 路由正确：`RadarOutput.log` 只含普通功能项（含 `空域自主搜索功能测试`、`天线方向图仿真功能测试`、
  `回波功率计算功能测试`、`恒虚警检测功能测试` 等，**无**旧名 `对指定空域进行搜索功能测试`）；
  62–66 项只出现在 `RadarExternalInterface.log`。
- 字段实测：第 29 项 `目标ID=208 斜距=3947862.0m 目标RCS=0.100m^2 回波功率=8.536e-18W
  噪声功率=4.526e-14W 干扰功率=0.000W`；第 36 项 `检测概率=90%的最大探测距离=3981030.4m
  依据RCS=0.100m^2`（>当前斜距 3947862.0 m，与 SNR 高于所需一致）；第 28 项反算/配置
  峰值 52.000 dBi、宽度 0.400°、四项判据 + 整体判据；第 40 项 `本周期航迹数=`；
  第 34/35 项无杂波/无干扰时写 `…=缺失`（本次 666 行）。

| 事项 | 阻塞原因 | 需要谁先决定 |
|---|---|---|
| 第 57 项 卫星平台与载荷配置接口 | 主文档未给出必测配置字段、单位、方向与应答规则；缺全量配置回显契约 | 需求方冻结字段清单 |
| 第 61 项 雷达平台与工作参数配置接口 | 同上（配置字段、可热改范围、应答规则未定义） | 需求方冻结字段清单 |
| 第 68 项 预警预测与精度评估接口 | 字段跨 inference/precision/threat 多个模型与日志，缺统一聚合责任方与稳定目标键；主文档要求“新增稳定预警消息 DTO 或由编排层聚合”后方可验收 | 需求方/架构确定聚合边界 |
| 第 21 项 卫星探测引导段 | RIR 当前周期输入没有 SBIRS→RIR 引导消息及投递时序语义字段（§0B.3 已登记为模型不可见） | 仿真消息总线/编排层提供 |
| 第 11 项 相邻探测时间间隔方向 | `≥1 s` 与截图 `≤1 s` 冲突（§0C.2 裁定 2） | 需求方复核 |
| SBIRS 侧端到端核对 | **已完成**（`sbirs_triple_sat_fix_messages`，`--output-dir log/verify_sbirs_20260928`）：产物为 `SatelliteTarget.log`、`IRExternalInterface.log`、`IRInternalInterface.log` + 6 份场景 CSV，旧独立文件全无；48→75 内部接口项分文件正确；第 5/8/9/12/13/14/58/67 项字段实测见下。`sbirs_inframe_dual_target` 未在本 tree 构建（无对应 target） | — |
| 统一 `SatelliteTarget.log` 的并发与无重复 | 融合/推演/精度同时写入时的并发与去重仍需多模块同场景验证（本 tree 的 FUSION/INFERENCE 开关为 OFF，其路径按宏剪除，无法在本 tree 取证） | 开发 + 需开启全部五个开关的构建 |
| 单元测试**运行** | 本 build tree 的测试 target 缺 `rapidcheck` 依赖、生成头需 `/std:c++17`（工程为 C++14），改动前即不可构建；本轮改用「独立链接 `1q.lib` 的验算程序」与「单文件编译」替代 | 构建环境负责人 |

**SBIRS 侧端到端核对结果（`sbirs_triple_sat_fix_messages`，本 tree Release 构建）**

- 产物：`SatelliteTarget.log`、`IRExternalInterface.log`、`IRInternalInterface.log` +
  `sbirs_dual_fix/scan/sats/los/fused/truth.csv`、`integration_*`；**未生成**
  `fusion_acceptance.log`、`inference_acceptance.log`、`precision_acceptance.log`、
  `opir_acceptance.log`、`SatellitePredict.log`。
- 第 5 项（实测）：`相对卫星ID=4 目标ID=1 量测方位/俯仰(ECI)=(2.85073738,0.04572722)rad
  瞬时视场方位/俯仰范围=(0.075922,0.075922)rad 位于瞬时视场内=是`
- 第 8 项：`卫星ID=4 真值位置ECEF=… 实际位置ECEF=… 定位误差ECEF=(-46.2,83.1,-38.1)m
  真值速度ECEF=(0.0,0.0,0.0)m/s 实际速度ECEF=(-0.1,0.3,0.3)m/s
  速度误差ECEF=(-0.110,0.330,0.279)m/s |定位误差|=102.4m`
  （注：本场景 `orbit_sigma_deg` 配置偏大，模长未落在 `[47.5,52.5] m`；属场景参数问题，见下）
- 第 9 项：`测角残差方位/俯仰(ECI)=(-0.00000027,0.00000136)rad 测角残差方位/俯仰=(-0.05,0.28)arcsec`
- 第 12/14 项（驻留时间收口后实测）：`驻留=0.725s` —— 落在 `0.5–1 s` 开区间
- 第 13 项：`脱靶量m=(0.617990,0.033404) 脱靶量模=0.61889m 窄视场目标视线方位/俯仰(ECI)=…
  窄视场视线角误差方位/俯仰=(-0.93,0.11)arcsec 跟踪状态=… 信号能量=7.141e-13J SNR=18.463dB`
- 第 58 项：`卫星ID=4 目标数量=2 背景辐射亮度=2.0000W/(sr·m^2) 基础大气透过率=0.800000
  环境温度=15.00C 相对湿度=50.00%`
- 第 67 项：`卫星ID=4 观测数量=1 目标数量=1` + 逐观测行（观测ID/目标ID/检测标志/ECI 方位俯仰/
  IR_SNR/观测阶段）+ 归属诊断距离（显式标注非被动红外直接测距）+ 跟踪状态行

**新发现的待办（由 SBIRS 端到端核对暴露）**

| 事项 | 现象 | 需要谁决定 |
|---|---|---|
| 第 8 项定位误差模长超出 `[47.5, 52.5] m` | `sbirs_triple_sat_fix_messages` 实测 `|定位误差|=102.4m`，超出判据；逐轴误差 `(-46.2, 83.1, -38.1)` 说明场景 `error_model.orbit_sigma_deg` / `nav_position_sigma_m` 配置与 `50 m` 口径不匹配 | 需求方确认目标噪声量级；若要判据成立需调场景误差模型参数（属场景配置，不是模型公式） |

## 5. 本次改动的验证方式

1. `cmake --build build/acceptance-v141 --config Release --target 1q_lib` 全库构建通过
   （接受 RIR/OPIR 开关为 ON；融合/推演/精度开关在本 tree 为 OFF，故其代码路径按宏剪除编译）。
2. `tests/unit/common/acceptance_record_format_test.cpp` 用工程同款包含路径与 `cl /std:c++14`
   单独编译通过（含六文件目录作用域、外部/内部接口分文件、首开截断重开追加用例）。
3. **端到端运行核对已执行**：`rir_ground_site_recognition`（120 周期，Release 构建）产物与字段
   逐项核对见 §4；**仍未执行** SBIRS 场景核对与单元测试整体运行（见上表阻塞项）。

以上构建记录只能证明代码可编译，不能单独证明计算口径和验收字段正确。**2026-09-28 第二次
收口新增的验证**：

1. 新增 `tests/unit/remote_identification_radar/rir_acceptance_calculations_test.cpp`：
   第 29 项 `16 dB` 差值（含反向反例）、第 36 项等值/单调/低于所需/四次方根闭合/非法入参、
   第 28 项采样峰值一致性与 5% 宽度及“配置变窄宽度随之变小”。该文件用工程同款包含路径与
   `cl /std:c++14` 单独编译通过，并由 RIR 单测分区 glob 自动收录。
2. 由于本 build tree 的测试 target 不可构建（见 §4），另用一个**独立链接
   `build/acceptance-v141/Release/lib/1q.lib` 的验算程序**实跑同一批断言，结果全部通过：
   `item29 gap_db=16.0206`；`item36 required_snr_db=13.1835`、`range@required=3400000.000 m`、
   `+8 dB 距离比=1.584893`；`item28 采样峰值=52.0000 dBi、az/el 宽度=4.000°/4.000°`。
3. **仍未执行**：完整 acceptance 场景的端到端核对（六个目标文件存在性、旧独立文件不存在、
   58/62–67 项字段齐全、统一 `SatelliteTarget.log` 的并发与无重复）。

## 6. 当前问题与建议执行顺序

日志分流框架与逐项计算口径已收口，剩余风险集中在“契约未冻结”与“端到端场景未跑”。

已完成（本轮）：
1. 第 36 项 SNR 比值方向修正 + 等值/高于/低于/单调性/闭合测试 —— 已完成并实跑通过。
2. 第 28 项重做：删除旧摘要、峰值与宽度均从实际采样反算、日志只保留标识+分项+整体 —— 已完成并实跑通过。
3. 第 12–14 项补 `arcsec` 视线角误差与 `镜头转向时间=s` —— 字段已落盘；驻留时间
   尚未完成，现有 `1.450s/20.000s` 输出须修正并增加严格范围断言。
4. 第 29 项 `16 dB` 差值测试 —— 已完成并实跑通过。

待办（按顺序）：
5. 先修正第 12、14 项驻留计算或验收场景配置，确保每条记录满足
   `0.5 s<驻留时间<1 s`；日志行首仿真时间不参与该判定。
6. 增加 58、62–67 项实际物理文件内容测试，以及统一 `SatelliteTarget.log` 的并发与重复测试。
7. 第 57、61、68 项在契约冻结前维持未完成；第 21 项在消息总线提供引导输入前维持外部阻塞。
8. 最后运行一次完整 acceptance 场景，逐项核对目标文件存在、旧独立文件不存在、字段齐全且无重复。

本指南的「状态」列记录每次收口的完成范围；未同时满足代码、口径、字段和测试四项的，不得视为已完成。
