# 远程识别雷达目标特征库——权威参考资料清单

- 下载日期：2026-11-08
- 目录：`TC_Model/examples/basic_config/remote_identification_radar/`
- 用途：核对 `target_feature_database_v1.1.db` 中弹头 / 诱饵 / 残骸 / 碎片四类型特征值的量级合理性
- 共 14 份，均为公开可获取文档；每份的“提供了什么”只列与本次核对直接相关的内容

> 说明：本清单只做资料归档与出处标注，不改变数据库内容。文中的数值均按原文单位摘录，dBsm 与 m² 的换算以 σ(dBsm)=10·lg σ(m²) 为准。

## 一、清单总表

| # | 文件名 | 标题 | 作者 / 机构 | 年份 | 页数 | 来源 | 许可 / 获取方式 |
|---|--------|------|-------------|------|------|------|------------------|
| 1 | `ref_ucs_countermeasures_2000.pdf` | Countermeasures: A Technical Evaluation of the Operational Effectiveness of the Planned US National Missile Defense System | A. Sessler 等 / Union of Concerned Scientists + MIT Security Studies Program | 2000 | 200 | https://hdl.handle.net/1903/4333 （UMD DRUM 镜像） | © UCS，公开全文 |
| 2 | `ref_ucs_physics_of_space_security_2005.pdf` | The Physics of Space Security: A Reference Manual | D. Wright, L. Grego, L. Gronlund / UCS + American Academy of Arts and Sciences | 2005 | 192 | https://www.ucsusa.org/sites/default/files/2019-09/physics-space-security.pdf | © 作者，公开全文 |
| 3 | `ref_ucs_decoys_in_intercept_tests_2019.pdf` | Decoys Used in Missile Defense Intercept Tests | D. Wright / UCS | 2019 | 11 | https://www.ucsusa.org/sites/default/files/attach/2019/01/Missile-Defense-Intercept-Test-Decoys-white-paper.pdf | © UCS，公开白皮书 |
| 4 | `ref_tno_missile_defence_overview_2012.pdf` | Missile Defence: An Overview | TNO Defence, Safety and Security | 2012 | 90 | https://publications.tno.nl/publication/100675/VLDsVa/2012-missile-defence.pdf | © 2012 TNO，公开 PDF（保留所有权利，仅作内部参考） |
| 5 | `ref_nasa_orbital_debris_quarterly_news_2020_v24n1.pdf` | Orbital Debris Quarterly News, Vol. 24, Issue 1 | NASA Orbital Debris Program Office | 2020 | 16 | https://orbitaldebris.jsc.nasa.gov/quarterly-news/pdfs/odqnv24i1.pdf | NASA 公开出版物 |
| 6 | `ref_mdpi_overview_sensors_long_range_missile_defense_2022.pdf` | An Overview of Sensors for Long Range Missile Defense | S. Fontana, F. Di Lauro / Sensors (MDPI) | 2022 | 19 | https://mdpi-res.com/d_attachment/sensors/sensors-22-09871/article_deploy/sensors-22-09871.pdf | CC BY 4.0 |
| 7 | `ref_mdpi_micro_motion_ballistic_missile_wideband_radar_2021.pdf` | Micro-Motion Parameter Extraction for Ballistic Missile with Wideband Radar Using Improved Ensemble EMD Method | N. Zhu 等 / Remote Sensing (MDPI) | 2021 | 15 | https://mdpi-res.com/d_attachment/remotesensing/remotesensing-13-03545/article_deploy/remotesensing-13-03545.pdf | CC BY 4.0 |
| 8 | `ref_mdpi_dynamic_electromagnetic_scattering_2020.pdf` | Simulation and Feature Extraction of the Dynamic Electromagnetic Scattering of a Hypersonic Vehicle Covered with Plasma Sheath | Z. Bian, J. Li, L. Guo / Remote Sensing (MDPI) | 2020 | 21 | https://mdpi-res.com/d_attachment/remotesensing/remotesensing-12-02740/article_deploy/remotesensing-12-02740.pdf | CC BY 4.0 |
| 9 | `ref_eurasip_precession_missile_feature_2012.pdf` | Precession missile feature extraction using sparse component analysis of radar measurements | L. Liu 等 / EURASIP J. Adv. Signal Process. | 2012 | 10 | https://asp-eurasipjournals.springeropen.com/counter/pdf/10.1186/1687-6180-2012-24 | 开放获取（SpringerOpen） |
| 10 | `ref_wpi_linking_simulated_radar_data_2009.pdf` | Linking Radar Data to Physical Models（MQP 报告，MIT Lincoln Laboratory 资助） | C. Fancher 等 / Worcester Polytechnic Institute | 2009 | 50 | http://digital.wpi.edu/downloads/79407z75b | 机构库公开 |
| 11 | `ref_arxiv_exkf_tbm_reentry_2007.pdf` | Application of Extended Kalman Filter to Tactical Ballistic Missile Re-entry Problem | S. Bhowmik 等 / arXiv | 2007 | 5 | https://arxiv.org/pdf/0707.1918 | arXiv 许可 |
| 12 | `ref_arxiv_polarimetric_rotation_target_2024.pdf` | Target Characteristics on Rotation in Euclidean Space Using Full Polarimetric SAR Data | S. Dey, I. Hajnsek / DLR，arXiv | 2024 | 4 | https://arxiv.org/pdf/2406.16060 | arXiv 许可 |
| 13 | `ref_illinois_bmd_future_security_1987.pdf` | Ballistic Missile Defense and Future U.S. Security（本科毕业论文） | J. S. Kennell / University of Illinois | 1987 | 74 | https://www.ideals.illinois.edu/items/96280/bitstreams/310139/data.pdf | 机构库公开 |
| 14 | `ref_uea_thaad_deterrence_2018.pdf` | Deterrence under Nuclear Asymmetry: THAAD and the Prospects for Missile Defense on the Korean Peninsula | I. Kim, S. Park / 接受稿（UEA 机构库） | 2018 | 39 | https://ueaeprints.uea.ac.uk/id/eprint/81383/1/Accepted_Manuscript.pdf | 作者接受稿，机构库公开 |

## 二、每份资料提供了什么

### 1. UCS《Countermeasures》(2000)——本次核对的主证据源

与库内特征直接相关的部分：

- **附录 C“The Radar Cross Section of Warheads and Other Objects”（p.131–134）**
  - 球体：直径 1 m → σ=0.8 m²；半径 10 cm 的子弹药 → σ=0.03 m²。
  - 尖锥-球（cone-sphere）正鼻向：σ≈0.1λ²，X 波段（λ=3 cm）约 **9×10⁻⁵ m²（−40 dBsm）**；平底锥正鼻向约大 10⁴ 倍（≈0.9 m²，0 dBsm 量级）。
  - X 波段 15 cm 三面角反射器最大 σ 可达 **21 m²**。
  - 美国早期预警雷达 420–450 MHz 下，1 m 球与平底锥 σ≈**1.0 m²**；同样尺寸 cone-sphere 正鼻向约 **0.1 m²**。
  - 图 C-1：底半径 0.22 m、半锥角 15° 的 cone-sphere，在 9.6 GHz 下 RCS 随视角变化可达数个数量级（60° 以外明显抬升）。
- **附录 D（p.135–144）**
  - X 波段雷达 1 GHz 带宽 → 距离分辨率 **0.15 m**；UHF 早期预警雷达 30 MHz → 5 m；PAVE PAWS 搜索模式仅 150 m。
  - 目标角分辨率、横向分辨率、Doppler/ISAR 成像条件（转一圈 10 s、积累 160 ms → 横向 0.15 m）。
- **第 8 章（p.59–80）**：抗模拟气球诱饵、红外/雷达判别机理，气球与弹头的热/运动差异。
- **p.129**：典型弹头“长约 2 m、底径约 1 m”；p.54–55：20 cm 级子弹药（球/锥）尺寸与质量。

### 2. UCS《The Physics of Space Security》(2005)——轨道/弹道运动学基准

- 圆轨道速度-高度表（Table 4.1）：200 km→7.8 km/s，500 km→7.6，1,000 km→7.4，5,000 km→5.9 km/s。
- vis-viva 方程与椭圆轨道近/远地点速度关系（§4）；可用于检验“高度-速度”组合是否落在弹道可达范围内。
- 弹道中段“重弹头与轻诱饵轨迹相同、无法靠轨迹区分”的判据（§4）。
- 空间目标环境：SSN 跟踪约 8,500 个 ≥10 cm 目标（§7），可作为“残骸/碎片”尺寸-数量背景。

### 3. UCS《Decoys Used in Missile Defense Intercept Tests》(2019)——诱饵与弹头几何

- IFT-3 中再入体（mock RV）：长约 **1.8 m**、底径约 **0.75 m**（p.5）。
- 气球诱饵直径 **2.2 m**（IFT-3），后续改为 **1.7 m**（IFT-6/7）；气球红外亮度约为 RV 的 6 倍（p.5–6）。
- 说明早期拦截试验的诱饵与弹头在红外/雷达特征上差异过大，不能代表真实对抗条件。

### 4. TNO《Missile Defence: An Overview》(2012)——威胁与弹道阶段

- 弹道三阶段划分与时间占比：中段约占全程 75%；远地点高度约为最大射程的 **1/3～1/5**（§2.2，p.19）。
- RV 再入速度：ICBM 射程量级为 **6–8 km/s**（§2.2，p.25）。
- 中段对抗手段：chaff、假弹头/气球诱饵、RV 再定向、RAM、助推器爆炸解体、干扰机、机动 RV（§3）。
- “残骸判别（debris discrimination）”的定义：区分 RV、残骸（含级间段/解体产物）与诱饵（§3.5）。
- 传感器体系：SBX（X 波段，主职判别）、UEWR、GMD 拦截链。

### 5. NASA ODQN 24(1) (2020)——空间碎片环境与雷达测量

- NASA ODPO 用 HUSIR（Haystack 超宽带成像雷达）、HAX、Goldstone 雷达测量**小至约 5 mm** 的轨道碎片；SSN 跟踪 ≥10 cm 目标。
- 碎片特征尺寸-数量/质量分布、面质比分布；10 cm 以上碰撞视为灾难性；再入幸存碎片 casualty area 限值 7 m²。
- 会议摘要“Radar Cross Section of Orbital Debris Objects”：非球形体 RCS 随姿态变化，NASA SEM 给出 RCS 概率密度分布。

### 6. MDPI Sensors (2022)——远程反导传感器综述

- X 波段用于区分 RV 与诱饵（THAAD AN/TPY-2、SBX-1）；分辨率与天线尺寸的权衡；各系统（Aegis、THAAD、GMD、Don-2N）频段与部署。
- 可作为“LRDR 类 X 波段宽带雷达承担判别任务”的体系背景。

### 7. MDPI Remote Sensing (2021)——弹道目标微动与宽带特征

- 锥形弹道目标几何：质心到锥顶 **h=0.64 m**、底半径 **r=0.25 m**；自旋 4 Hz、锥旋 2 Hz、进动角 10°；雷达 10 GHz、带宽 **2 GHz**、脉宽 10 µs、PRF 1 kHz、驻留 1 s（p.9）。
- 进动/章动引起的散射中心距离-时间轨迹，可用于检验“距离像长度/峰值数/能量集中率”的建模假设。

### 8. MDPI Remote Sensing (2020)——再入等离子体鞘套的动态 RCS

- 高超声速（15–25 Ma、30–70 km）等离子体鞘套使 RCS 在 **−60～+30 dBsm** 大范围起伏，且 HH/VV 极化响应不同（1.25/3.25/8.25 GHz）。
- 说明再入段 RCS 与极化特征会因等离子体发生数量级变化；中段（本库适用段）不涉及该效应，可作为“段的选择”依据。

### 9. EURASIP (2012)——进动锥体雷达信号模型

- 进动锥体多散射中心回波模型（锥顶/锥底/滑动散射中心），5 GHz 弹道导弹预警雷达仿真；散射中心数目与锥体几何的关系。
- 支撑“弹头距离像为少峰值、强集中”的定性假设。

### 10. WPI/MIT LL (2009)——雷达数据与物理模型关联

- RCS 定义、dBsm 表示、姿态相关 RCS 极图；翻滚/分离双体火箭的距离-多普勒回波仿真。
- 支撑“姿态变化导致 RCS 起伏、可用 RCS 时序/距离像判别”的方法论。

### 11. arXiv 0707.1918 (2007)——TBM 再入段 EKF

- 再入段六自由度/三自由度运动方程、阻力与弹道系数模型；位置/速度估计误差随弹道系数变化。
- 用于理解“再入段加速度主要来自气动阻力”，中段自由飞行段则接近零非引力加速度。

### 12. arXiv 2406.16060 (2024)——极化特征随目标旋转的变化

- 全极化 SAR 下目标旋转对极化特征的影响；提出旋转域特征参数，说明极化特征非旋转不变。
- 支撑“ψ/τ 需按 180° 周期处理、极化特征应带姿态不确定度”的设计。

### 13. Illinois 1987（本科论文）——历史背景，仅作背景

- 1980 年代 BMD 技术路线、定向能/动能拦截、对抗手段的政策性讨论；无可用特征数值。

### 14. UEA 2018——THAAD 对抗的定性讨论，仅作背景

- 对抗 THAAD 的手段（低红外/低 RCS、干扰、chaff、机动等）及其战略效果；无特征数值。

## 三、未能下载的权威资料（受访问限制）

以下资料与本次核对高度相关，但站点对本机网络返回 403/202（Cloudflare/AWS WAF 或地域限制），未能取得全文：

| 资料 | 期望提供的内容 | 受阻情况 |
|------|----------------|----------|
| Army Science Board, *Midcourse Discrimination for the Phase One Strategic Defense System* (1989), `spp.fas.org/military/asbmidcourse.pdf` | 中段判别体系、诱饵/碎片判别概率 | AWS WAF challenge（HTTP 202） |
| JASON, *MDA Discrimination* JSR-10-620, `irp.fas.org/agency/dod/jason/mda-discrimination.pdf` | 判别技术成熟度与物理极限 | 同上 |
| OTA, *Ballistic Missile Defense Technologies* (1985) | 弹道目标 RCS/判别/对抗的经典定量章节 | Princeton OTA 镜像与 Georgetown 镜像均 403/Cloudflare |
| NASA NTRS：Trailblazer IIa 再入 RCS（1966）、Apollo 再入雷达（1967/1968） | 再入体 RCS 实测 | ntrs.nasa.gov 403 |
| MIT Lincoln Laboratory Journal：宽带雷达判别专刊 | X 波段宽带距离像判别实测 | ll.mit.edu 403 |
| National Academies, *Making Sense of Ballistic Missile Defense* (2012) | 判别能力与对抗评估 | nap.edu 地域重定向 403 |
| NASA HUSIR CY22 轨道碎片测量报告（TP-20260000227） | 碎片 RCS-尺寸统计 | NTRS 403 |

> 若需要补齐上述资料，可在可访问这些站点的网络环境下重试；当前 14 份已足以支撑 RCS、几何尺寸、运动量级与传感器分辨率的核对，但**极化五量的绝对值仍缺公开实测**。

## 四、引用与许可注意

- MDPI 两篇为 CC BY 4.0，可直接引用/再分发（保留出处）。
- NASA ODQN 为美国政府出版物，通常不受版权保护。
- UCS、TNO 文档为“公开可下载但保留版权”，仅作内部技术参考，不随产品分发。
- arXiv、EURASIP、WPI、Illinois、UEA 为开放获取/机构库文稿，引用时注明版本（尤其 UEA 为 accepted manuscript，非最终排版版）。
