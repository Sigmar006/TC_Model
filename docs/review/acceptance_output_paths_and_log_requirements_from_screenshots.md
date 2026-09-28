---
Status: draft
Role: 用户截图独立 OCR 存档及与现有验收用例的对比（不合并）
Date: 2026-09-27
Source: 用户提供的 5 张电子表格截图
Compared-With: docs/review/functional_acceptance_test_criteria_ocr.md
---

# 验收测试输出路径、日志文件名及记录要求（截图转写）

> 本文件是截图内容的独立转写，不修改、不替代，也不合并到现有验收用例。
>
> 截图存在摩尔纹、列被裁切及少量文字模糊。能够从截图确认的内容直接转写；不能可靠确认的编号、名称或文字以“待复核”标记。路径和文件名保留截图中的大小写及拼写。

## 1. 截图表头与通用信息

截图表格的主要列为：

| 列 | 含义 |
|---|---|
| A/B | 测试编号、功能测试名称 |
| C | 文件路径 |
| D | 文件名 |
| E | 涉及的数据/记录要求 |
| 右侧扩展列 | 测试输入、操作步骤或通过判据（仅部分截图可见） |

### 1.1 输出目录

| 适用范围 | 截图中的目录 |
|---|---|
| 天基红外/OPIR、融合、预测等 | `C:/SosimPlatform/SosimApplication/SimHelper/` |
| 雷达普通功能项 | `C:/SosimPlatform/SosimApplication/SimHelper/` |
| 雷达天线方向图仿真 | `C:.../ModelOutput`（截图为省略路径） |

## 2. 天基红外及相关功能测试

### 2.1 轨道、探测及融合

| 编号（截图） | 功能测试 | 文件名 | 涉及的数据/要求 |
|---|---|---|---|
| 5.3.1.1.1 | 实时计算卫星当前轨道位置功能测试（`GN_GDCX_TCPT_GDWZ`） | `Opir_acceptance.log` | 卫星位置信息 |
| 5.3.1.1.2.1 | 坐标系转换功能测试（`GN_GDCX_TCPT_ZBZH`） | `Opir_acceptance.log` | 卫星 ECI、ECEF、WGS-84 经纬度坐标 |
| 5.3.1.1.2.2 | 卫星轨道动力学计算功能测试（`GN_GDCX_TCPT_DLJS`） | `Opir_acceptance.log` | 卫星轨道高度、地心系坐标、速度、经纬度坐标 |
| 5.3.1.1.2.3 | 龙格库塔积分功能测试（`GN_GDCX_TCPT_LGKT`） | `Opir_acceptance.log` | 卫星轨道高度、地心系坐标、速度 |
| 5.3.1.2.1.1 | 探测角度计算功能测试（`GN_GDCX_TCXX_TCJD`） | `SatelliteTarget.log` | 卫星传感器时视场信息，以及目标是否位于传感器时视场内的判定结果 |
| 5.3.1.2.1.2 | 最大探测距离计算功能测试（`GN_GDCX_TCXX_TCJL`） | `SatelliteTarget.log` | 卫星对目标实体的最大探测距离 |
| 5.3.1.2.1.3 | 根据目标识别功能测试（`GN_GDCX_TCXX_MBSB`，名称按截图，待复核） | `SatelliteTarget.log`、`MissileBallisticData.log` | 目标群具体规模数量；文件中查看实际目标数据；仿真数量与实际数量误差不大于 30% 视为通过 |
| 5.3.1.2.2 | 目标可探测性与动态参数生成功能测试（`GN_GDCX_TCXX_KTCX`） | `SatelliteTarget.log` | 目标位置、距离；对应测角误差；宽视场误差范围约为 `-360` 角秒，窄视场约为 `±10` 角秒（截图措辞待原表复核） |
| 5.3.1.2.3.1 | 卫星自身定位误差模拟功能测试（`GN_GDCX_TCXX_DWWC`） | `SatelliteTarget.log` | 卫星位置、速度等动态参数及自身定位误差统计；误差向量模长小于 `50 m`（`1σ，约 68%`）视为通过 |
| 5.3.1.2.3.2 | 红外载荷角误差模拟功能测试（`GN_GDCX_TCXX_CJWC`） | `SatelliteTarget.log` | 红外探测器对高速动态目标进行方位角和俯仰角测量时的测角误差；不大于 `5` 角秒视为通过 |
| 5.3.1.2.3.3 | 安装矩阵误差模拟功能测试（`GN_GDCX_TCXX_JZWC`） | `SatelliteTarget.log` | 传感器理想指向、实际指向及安装误差矩阵 |
| 5.3.1.3.1 | 红外预警卫星大幅面扫描与探测功能测试（`GN_GDCX_TCPT_SMTC`） | `SatelliteTarget.log` | 扫描幅宽、探测目标辐射、目标状态、探测 SNR、探测概率；连续目标探测数据时间间隔不大于 `1 s` 视为通过 |
| 5.3.1.3.2.1 | 宽视场扫描探测功能测试（`GN_GDCX_TCPT_KCSM`） | `SatelliteTarget.log` | 扫描周期地面覆盖范围坐标；覆盖区域驻留时间且驻留时间大于 `0.5 s`、小于 `1 s`；目标视线角及对应误差，误差范围约 `±360` 角秒 |
| 5.3.1.3.2.2 | 窄视场跟踪探测功能测试（`GN_GDCX_TCPT_ZCSM`） | `SatelliteTarget.log` | 采样时目标在焦平面上的脱靶量、跟踪状态、目标信号能量、信噪比；输出高精度目标视线角及误差，误差范围约 `±10` 角秒 |
| 5.3.1.3.2.3 | 宽窄视场联合探测功能测试（`GN_GDCX_TCPT_KZLH`） | `SatelliteTarget.log` | 宽视场扫描覆盖区域、疑似目标列表、扫描疑似目标的精确状态序列；宽视场视线角误差约 `±360` 角秒，窄视场约 `±10` 角秒；记录窄视场镜头转向时间 |
| 5.3.1.4.1 | 多传感器目标跟踪功能测试（`GN_GDCX_MNTCQ_MBGZ`） | `SatelliteTarget.log`、`Fusion_acceptance.log` | 目标角度及红外可探测性；目标定位信息应包含传感器信息源名称、信息源测角数据、测角信息融合卫星地面站名称、目标地心地固系三维位置坐标及误差协方差矩阵；距离误差小于 `50 m` 视为通过 |
| 5.3.1.4.2 | 多传感器接力跟踪功能测试（`GN_GDCX_MNTCQ_JLGZ`） | `Fusion_acceptance.log` | 记录跟踪目标的位置信息、速度、加速度、初始卫星对目标的剩余覆盖时间、预测目标运动轨迹、接力计划、交接指令及多源数据融合后的跟踪结果 |

### 2.2 预测、精度和事件（截图下半部分）

| 功能测试 | 文件名 | 涉及的数据/要求 |
|---|---|---|
| 标准卡尔曼滤波功能测试（`GN_GDCX_DPT_BZLB`） | `SatellitePredict.log`、`Fusion_acceptance.log` | 目标方位角、俯仰角及其变化率；连续时刻目标视线方向的平滑估计结果 |
| 目标轨迹预报功能测试（`GN_GDCX_DPT_GJYB`） | `SatellitePredict.txt`、`Inference_acceptance.log`、`MissileBallisticData.log` | 目标从当前时刻到 `10 s` 后的轨迹；预测落点位置（经度、纬度、高程）；落点误差椭圆参数；实际目标与预测落点误差距离值 |
| 落点预测功能测试（`GN_GDCX_DPT_LDYC`） | `SatelliteTarget.log`、`RadarSingalProcessingDateFlow.log`、`Inference_acceptance.log`、`MissileBallisticData.log` | DD 目标角度数据；在测角数据仿真时间之后检查雷达收到的卫星探测引导信息和 DD 目标得到的距离、方位和高度等探测结果；查看 DD 目标初步落点预测位置及弹道再提取数据 |
| 发射点预测功能测试（`GN_GDCX_DPT_FSDYC`） | `Inference_acceptance.log`、`MissileBallisticData.log` | 预测发射时刻和地理位置并记录误差范围；查看弹道再提取数据 |
| 关机点判断功能测试（`GN_GDCX_DPT_GJDYC`） | `Inference_acceptance.log`、`MissileBallisticData.log` | 预测关机时刻和关机点位置并记录误差范围；查看实际关机点时刻数据 |
| 特殊事件监测与提示功能测试（`GN_GDCX_MNSC_TSSJ`） | `Opir_acceptance.log` | 识别首次探测、跟踪中断、轨迹突变等关键事件，包含事件等级、目标 ID、时间和位置 |
| 目标异常事件监测与提示功能测试（名称/编号待复核） | `Opir_acceptance.log` | 识别跟踪中断关键事件信息，包含事件等级、目标 ID、时间和位置 |
| 预警中断事件监测与提示功能测试（名称/编号待复核） | `Opir_acceptance.log` | 识别轨迹突变等关键事件信息，包含事件等级、目标 ID、时间和位置 |
| 轨迹突变事件监测与提示功能测试（名称/编号待复核） | `Opir_acceptance.log` | 同上；截图此处文字有重叠，需原表复核 |
| 落点预报与信息分发功能测试（`GN_GDCX_MNSC_LDYB`） | `SatelliteTarget.log`、`MissileBallisticData.log` | 记录 DD 模型预测落点、落点误差范围/标准比例数据和分发跟踪数据信息；文件中查看弹道再提取数据 |
| 关键精度指标提取功能测试（`GN_GDCX_MNSC_ZBTQ`） | `SatelliteTarget.log` | 目标三轴位置误差、距离误差、方位角和俯仰角误差、目标脱靶量 |
| 层次分析法功能测试（`GN_GDCX_MNSC_CCFX`） | `SatelliteTarget.log` | 各层级要素权重、各层指标评分值、综合效能总分及等级评定 |

## 3. 雷达功能测试

截图中除天线方向图条目外，以下条目的目录均为：
`C:/SosimPlatform/SosimApplication/SimHelper/`。

| 编号（截图） | 功能测试 | 文件名 | 涉及的数据/要求 |
|---|---|---|---|
| 5.3.2.1.1.1 | 天线方向图仿真功能（高斯主瓣、余弦幂、辛克天线方向图） | `RadarAntennaGainPatterndata.csv`、`RadarOutput.log` | 记录三维天线增益方向图；由方向图数据计算天线增益、方位/俯仰波束宽度、最大旁瓣电平、后瓣电平，与设置值误差不超过 `5%` 视为通过；记录扫描波束宽度数据。目录为 `C:.../ModelOutput` |
| 5.3.2.1.1.2 | 回波功率计算功能测试（`GN_YCSB_MNTC_HBGL`） | `RadarOutput.log` | 查看控制台打印的目标回波功率、噪声功率和干扰功率信息 |
| 5.3.2.1.1.3 | 干扰功率计算功能测试（`GN_YCSB_MNTC_GRGL`） | `RadarOutput.log` | 打印各干扰源到达雷达接收端的干扰功率 |
| 5.3.2.1.1.4 | 接收机噪声功率计算功能测试（`GN_YCSB_MNTC_ZSGL`） | `RadarOutput.log` | 记录接收机噪声功率信息 |
| 5.3.2.1.1.5 | 目标信号增益功能测试（`GN_YCSB_MNTC_MBXH`） | `RadarOutput.log` | 记录经处理后雷达回波信号、总增益、脉压增益、MTI 增益、MTD 增益 |
| 5.3.2.1.1.6 | 噪声增益功能测试（`GN_YCSB_MNTC_ZSZY`） | `RadarOutput.log` | 记录脉冲压缩后的噪声功率、各多普勒滤波器通道的噪声功率、MTD 处理后的等效噪声功率、噪声总增益 |
| 5.3.2.1.1.7 | 杂波信号处理增益功能测试（`GN_YCSB_MNTC_ZBZY`） | `RadarOutput.log` | 记录 MTI 处理后杂波剩余功率、MTD 各多普勒通道杂波剩余功率分布、MTD 处理后的杂波总抑制比、总杂波抑制增益 |
| 5.3.2.1.1.8 | 干扰信号处理增益功能测试（`GN_YCSB_MNTC_GRZY`） | `RadarOutput.log` | 记录各多普勒通道干扰功率分布、MTD 处理后的干扰总抑制比、总干扰抑制增益 |
| 5.3.2.1.1.9 | 恒虚警检测功能测试（`GN_YCSB_MNTC_HXJ`） | `RadarOutput.log` | 记录统计检测门限、统计检测结果；按检测概率为 90% 的最大探测距离、目标 RCS 和雷达方程换算当前目标最大探测距离，与定界报告数值对比，误差在 `±5%`；记录统计检测概率 |
| 5.3.2.1.2 | 空域自主搜索功能测试 | `RadarOutput.log` | 记录本周期方位/俯仰指向信息、扫描调度信息 |
| 5.3.2.1.2（子项） | 地平线搜索功能测试 | `RadarOutput.log` | 记录检测、量测信息，及本周期方位/俯仰指向信息、扫描调度信息 |
| 5.3.2.1.3 | 对目标的角度和距离进行测量功能测试（`GN_YCSB_MNTC_MBCL`） | `RadarOutput.log`、`MissileBallisticData.log` | 记录目标测量角度与距离、目标测量误差统计；文件中查看实际目标数据 |
| 5.3.2.2.1 | 对典型目标/再入目标进行跟踪功能测试（`GN_YCSB_MNTQGZ_MBGZ`） | `RadarOutput.log` | 记录目标定位（距离、方位、俯仰）和运动参数（速度、航向、加速度） |
| 5.3.2.2.2 | 多目标跟踪功能测试（`GN_YCSB_MNTQGZ_DMBGZ`） | `RadarOutput.log` | 记录多个目标定位（距离、方位、高度）和运动参数（速度、航向、加速度） |
| 5.3.2.3.1.1 | 航迹关联功能测试（`GN_YCSB_MNSBD_HJGL`） | `RadarOutput.log` | 记录检测结果、目标位置、检测概率 |
| 5.3.2.3.1.2 | 跟踪滤波功能测试（`GN_YCSB_MNSBD_GZLB`） | `RadarOutput.log`、`MissileBallisticData.log` | 当前时刻 ECEF 速度和加速度估计值、当前时刻目标状态协方差；当前时刻的位置误差指标；文件中查看实际目标数据 |
| 5.3.2.3.2 | 根据目标 RCS 实时计算目标探测结果功能测试（`GN_YCSB_MNSBD_SSJS`） | `RadarOutput.log` | 记录对已探测目标保持持续、精确定位（距离、方位、俯仰）和运动参数（速度、航向、加速度）；目标距离小于 `3400 km` 时能够记录到命中概率大于目标进行持续、精确定位（原句局部模糊）；与前项相比定位和运动参数误差增大视为通过，否则不通过（原句逻辑待复核） |
| 5.3.2.3.3.1 | 运动特征处理功能测试（`GN_YCSB_MNSBD_YDTZ`） | `RadarOutput.log` | 航迹、目标 ID、目标速度、目标高度、加速度、近似直线判定和识别类型 |
| 5.3.2.3.3.2 | RCS 统计特征处理功能测试（`GN_YCSB_MNSBD_RCS`） | `RadarOutput.log` | 航迹、目标 ID、RCS 均值/标准差/平均归一化值/RCS 波动系数和识别类型；有结果视为通过 |
| 5.3.2.3.3.3 | 极化特征解算功能测试（`GN_YCSB_MNSBD_JHTZ`） | `RadarOutput.log` | 极化散射矩阵、极化系数、本征极化方向角、本征极化椭圆率、目标类型 |
| 5.3.2.3.3.4 | 宽带一维像特征解算功能测试（`GN_YCSB_MNSBD_KDYWX`） | `RadarOutput.log` | 航迹、目标 ID、散射中心距离与 RCS、目标长度、峰值、识别类型；形成匹配滤波后记录 HRRP 综合识别加权贡献，数值有限且位于规定区间（截图下限/上限未完整显示） |
| 5.3.2.3.3.5 | 综合特征识别功能测试（`GN_YCSB_MNSBD_ZHTZ`） | `RadarOutput.log` | 综合识别结果的统计；识别正确率大于 `60%` |
| 5.3.2.4.1 | 波束扫描功能测试（`GN_YCSB_MNSD_BSSM`） | `Rir_scan_pattern.csv` | 记录波位排列、扫描轨迹序列；方位/俯仰扫描范围满足设定范围要求（下限、阵面方向扫描上限等，截图局部模糊） |
| 5.3.2.4.2 | 调度策略功能测试（`GN_YCSB_MNSD_DDCL`） | `RadarOutput.log` | 上限、Pitch 方向、AzimuthFirst/LeftTop 等扫描起始设置；记录各类事件的实际执行数量列表（截图末行被裁切） |

## 4. 截图中可确认的特殊测试输入与判据

### 4.1 回波功率计算

- 在模型属性参数界面输入峰值发射功率 `582 kW`、载波频率 `3e9 Hz`、发射带宽 `4.5e6 Hz`、发射损耗 `3.5 dB`、接收损耗 `2.0 dB`、天线附加损耗 `0.0 dB`、大气压 `1013.25 hPa`、大气温度 `288.15 K`、相对湿度 `50%`、目标 RCS `1 m²`。
- 将目标 RCS 改为 `0.025 m²`；在相同距离下，与基准判据相比，目标回波功率降低约 `16 dB` 视为通过。

### 4.2 干扰功率计算

- 干扰源参数：干扰功率 `50 W`、干扰载波频率 `3e9 Hz`、干扰带宽 `4e7 Hz`、干扰天线增益 `20 dB`、极化方式 `kVertical`、波束方位指向 `0 deg`、波束俯仰指向 `-20 deg`。
- 将干扰功率修改为 `150 W`、天线增益修改为 `30 dB` 后保存并执行相应步骤。
- 非国产和国产化系统中同名 `RadarOutput.log` 应记录各干扰源到达雷达接收端的干扰功率；与基准判据相比干扰功率数值增大视为通过，否则视为不通过。

## 5. 与现有验收用例的对比（不合并）

对比对象：`docs/review/functional_acceptance_test_criteria_ocr.md`。

### 5.1 总体结论

| 对比项 | 截图转写 | 现有验收文档 | 结论 |
|---|---|---|---|
| 覆盖范围 | 天基红外/融合/预测/雷达功能项，约对应现有文档 1–49 | 含 1–77，另有外部/内部接口、性能项和责任边界 | 截图是现有文档早期功能项的主要来源之一，但覆盖面更窄 |
| 输出目录 | 固定绝对目录 `C:/SosimPlatform/SosimApplication/SimHelper/`，方向图另用 `C:.../ModelOutput` | 统一映射到仓库运行目录 `log/` | **路径不一致**；现有文档已做工程化映射 |
| OPIR 文件名 | `Opir_acceptance.log`、`SatelliteTarget.log` 等 | 普通 OPIR 功能主要落到 `log/SatelliteTarget.log` | **存在文件归并/映射差异** |
| Fusion 文件名 | `Fusion_acceptance.log` | `log/fusion_acceptance.log` | 语义一致，大小写及目录不同 |
| Inference 文件名 | 截图同时出现 `Inference_acceptance.log` 和其它预测文件 | `log/inference_acceptance.log` | 大小写/目录不同；现有文档明确了仓库实际落盘名 |
| 雷达文件名 | `RadarOutput.log`、`RadarAntennaGainPatterndata.csv`、`Rir_scan_pattern.csv` | 同名文件落在 `log/` | 文件名基本一致，目录不同 |
| 判定方式 | 多数以“日志中记录到指定字段”为通过；部分附数值阈值 | 保留同类字段和阈值，并增加通用日志格式、身份字段、缺失项/存疑项规则 | 现有文档更结构化、更严格 |
| 多文件判定 | 截图中部分用例指定多个文件 | 现有文档强调每个指定文件独立判定，不得跨文件替代 | 现有文档增加了明确边界 |

### 5.2 可确认的一致项

- 雷达第 28–49 项的测试名称、主要日志文件及核心记录字段总体一致。
- `RadarAntennaGainPatterndata.csv` 用于天线方向图，`Rir_scan_pattern.csv` 用于波束扫描。
- 雷达回波功率、干扰功率、噪声、增益、恒虚警、搜索、量测、跟踪、识别等均使用 `RadarOutput.log` 作为主要证据。
- 天基红外普通探测项主要使用 `SatelliteTarget.log`；融合项使用 `Fusion_acceptance.log`；预测项使用 `Inference_acceptance.log`。
- 现有文档中的关键阈值（例如方向图误差 `5%`、卫星定位误差 `50 m`、角误差 `5` 角秒、多传感器定位误差 `50 m`、综合识别正确率 `60%`）与截图可辨内容一致。

### 5.3 需要保留差异、不应直接覆盖现有文档的项

1. 截图使用固定 Windows 绝对路径；现有实现按工程配置写入 `log/`，不能直接以截图路径覆盖。
2. 截图的 `Opir_acceptance.log` 在现有文档中被映射/拆分到 `SatelliteTarget.log` 等文件，不能仅凭截图恢复旧文件名。
3. 截图同时出现 `SatellitePredict.txt`、`SatellitePredict.log`、`MissileBallisticData.log`、`RadarSingalProcessingDateFlow.log`；现有文档将其中一部分视为外部数据或辅助证据，不能未经接口核对就纳入当前模型正式输出。
4. 截图部分阈值文字被裁切或存在疑似原表笔误（尤其 RCS 实时探测、波束扫描范围、HRRP 加权贡献区间）；本文件保持“待复核”，不反向修订现有验收判据。
5. 现有文档包含日志开关、身份字段、文件独立判定、模型数据责任、内部接口及性能项；这些是截图中没有的后续工程化补充。

## 6. OCR 待复核清单

- “根据目标识别功能测试”一行的完整名称及编号。
- 目标可探测性条目中宽/窄视场角误差范围的正负号和单位原文。
- 特殊事件监测下三个子项的完整名称、编号及是否共用同一条记录要求。
- RCS 实时计算目标探测结果条目的完整通过判据。
- 宽带一维像特征解算中“综合识别加权贡献”的上下限。
- 波束扫描的方位/俯仰上下限，以及调度策略最后一行完整文字。
- 截图中的 `RadarSingalProcessingDateFlow.log` 拼写是否为原表正式名称（`Singal`/`Signal`、`Date`/`Data`）。

