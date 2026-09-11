# 场景预期表：sbirs_triple_sat_fix_messages（三星 GEO + 消息机制地面站）

## 场景意图

| 项 | 值 |
| --- | --- |
| 被测通道 | SBIRS × 3 + Fusion + 精度评估（双星交会叠加） |
| 被测行为 | 赤道静止轨道三星覆盖地球盘；两两重叠区双星测角确认目标位置 |
| 验证深度 | L2 预期表（周期窗 + 通道组成）+ L3 几何先验（遮挡 / 星下点视场） |
| 构建模式 | Windows v141 release；FD 无关 |
| 日志模式 | 默认 summary / key |
| 输出目录 | `examples/log/sbirs_triple_sat_fix_messages/` |

## 几何

三颗地球静止轨道卫星（赤道、高度约 35788 km、经度间隔 120°，ECEF 速度为 0）：

| 星 | source_id | LLA（lat, lon, alt m） | 扫描扇区 | 覆盖角色 |
| --- | ---: | --- | --- | --- |
| A | 4 | (0, −25.212457, 35778820.868343) | 星下点相对 [0°,30°) | 大西洋 / 非洲西岸 |
| B | 104 | (0, −145.212457, 35778821.146480) | 星下点相对 [0°,30°) | 太平洋 |
| C | 204 | (0, 94.787543, 35778821.003790) | 星下点相对 [0°,30°) | 印度洋 / 东亚 |

从 GEO 看地球角半径约 8.7°。WFOV 24°×24°（DSP 式"方位扫 + 俯仰一次盖满全盘"：俯仰
视场 ≥ 盘高 17.4° 是该架构的物理要求；2026-08-31 试验过 8.7° 窄视场 + 俯仰栅格，因伴
星引导断链否定，见下"窄视场试验记录"）；`scan_rate=6°/s`、`scan_span=30°`。星下点相对扇区 [0°,30°)
远端超出 GEO 可见窗边界 +20.69°（可见窗 ±(盘半径 8.69° + 视场半宽 12°)，总宽 41.38°），触发跨度收敛
（只裁远端，相位原点仍在起点 0）：有效扫程 [0°,+20.69°]（见 SbirsPipeline.cpp 星下点跨度收敛），往复式（到边反向、牛耕式行进）。

**双星定位架构（2026-08-31 裁定落地）**：宽场只做粗测向 + 引导窄场（4/4 交接捕获成功），
窄场跟踪输出高刷新率高精度数据（`frame_rate_hz=10` → 每周期 10 帧独立量测融合，
随机 1-σ 按 1/√N 衰减；单帧 σ=`fov_sigma_deg` 1e-4°≈17.5 μrad，融合 σ≈5.5 μrad；
第 13 项行带 `帧数/单帧σ/融合σ` 字段），双星交会**仅收窄场阶段量测**（宽场粗角
在评估会话聚合处剔除）。实测（vs 当日宽场口径基线）：定位 74 行，位置误差
均值 72.7→43.7 m（-40%）、max 213→113 m——且新口径含真实单帧噪声（旧基线 σ=0）。

**窄视场试验记录（2026-08-31，已回退）**：WFOV 8.7°×8.7° + scan_span 16°/30° 两轮 +
俯仰栅格（库内阶段 4，场景加载器已补 `scan_el_start/span/step` 键解析）——四角可落地
（60/240 全落地）但每目标仅单星可见（伴星俯仰偏移 4.35°~12° 出窄带），双星配对 0/80、
冒烟失败。教训：宽场窄到"一次盖不满盘"时引导链断（无相位协同的两星稀疏可见撞不上
配对窗）。

**cross-cue 重启（2026-09-01，已实现；契约记录已删档，见 git 历史）**：
三星 `cross_cue=true`（星→地→星递话：发现星宽场量测经地面站转发，受话星下一周期窄场
直接转头），宽场 8.7°×8.7° 常开——双星配对恢复 79/80（对照 0/80），composite 0.226→0.408，
冒烟通过；24° 基线（cross_cue 默认关）逐位不回归（74/80、0.431）。
扫描配置（2026-08-31 起）：三星均用 `scan_azimuth_reference=kNadirRelative` + 起始偏移 0，
各星独立往复扫（**无星间相位协同**——现实即如此，重叠靠几何布局）；双星交会配对窗
`precision.dual_sat_pair_window_cycles=1`（检出错开 ≤1 周期可配对，对应"按时刻融合"
口径；时间基准=周期号抽帧时刻，量测本身无时间戳属已知缺口，见
`docs/common/open_questions.md` SBIRS-OQ-6/FUSION-OQ-1）。

**安装指向配置（2026-09-01 起，验收第 10 项数据源）**：每星可选键
`mount_angles_deg=[yaw,pitch,roll]`（Body→Sensor 安装角）、
`misalignment_bias_deg=[yaw,pitch,roll]`（常值失准偏置）、
`misalignment_random_sigma_deg`（运行期一次抽取微扰 1-σ）、
`misalignment_random_seed`（微扰种子）、`stabilization_mode`
（`kBodyStabilized`/`kInertialStabilized`）、
`sensor_scan_limits_deg=[az_min,az_max,el_min,el_max]`（传感器系扫描限位）。
**缺段/缺键继承加载器默认集 `SbirsOrientationDefaults()`**（config_loader_detail.h）：
名义安装角 (1.0, 0.5, −0.5)° + 失准偏置 (0.10, −0.05, 0.02)° + 微扰 σ 0.02°（种子 7）
——即本场景验收演示那套误差参数，后续场景不写键直接继承；写了的键逐一覆盖。
库结构体 `SbirsOrientationConfig` 默认仍为全零（单测/库内几何不受影响，默认集只在
JSON 加载层生效）。本场景 A 星零键纯继承默认；B/C 星互异安装角覆盖用于核对卫星 ID
装配（失准三星同批）。失准为会话级常值、进光轴链不进量测，覆盖四角经纬随之偏移
≤1° 量级属预期。

**两两重叠、不要求三星同视同一目标。** 120° 静止轨道对地球表面没有三星公共视场（地球遮挡）；相邻两星重叠约 42° 经度。双星测角交会已经能确认位置。

| 目标 | LLA | 可见星 | 80 s 内 |
| --- | --- | --- | --- |
| key=1 | 15°N, 85°W, 600 km | A+B（C 被地球挡住） | 仍在 A/B 视场 |
| key=2 | 10°N, 35°E, 500 km | A+C（B 被地球挡住） | 仍在 A/C 视场 |

精度评估交会只吃 A/B，因此 key=1 出 `dual_sat` 样本；key=2 只走融合通道 4+204。

## 算法边界（做不到的不要装做得到）

1. **精度评估仍是双视线 API**（`TryComputeDualLosFixM`）。第三星进融合、不进 `dual_sat_fix` 指标。这是库边界，本场景不改交会 API。
2. **融合默认沿视线起步 100 km**，GEO 真距约 36000 km。不覆写时 UKF 位置会停在卫星附近，不能当「确认位置」。本场景 JSON `fusion.track_bearing_init_range_m=3.6e7`（σ=8e6）是场景层补丁，不是库默认已按 GEO 标定。
3. **无身份探测不能靠 5° 方位门跨星关联**（不同卫星的局部方位本来就不同）。本场景靠 SBIRS 归属键直挂，所以双星能并到同一航迹。
4. **测角误差到 GEO 斜距会放大**：姿态 1-σ 0.01° × 约 39200 km ≈ 7 km 是上界口径；
   实际交会样本来自 NFOV 跟踪段（建链后每周期供数），扫描态实测交会 RMSE 约 85 m。
   早期"数公里"预期按 WFOV 单帧量测估计，已被跟踪段实测取代。
5. **NFOV 8° 若不转到目标上，离星下点约 8.5° 的目标只在 WFOV 里。** 确认位置走 WFOV 测角 + 双星交会；宽窄交接失败不算本场景失败。

## 预期事件表

| 通道 | 行为 | 周期窗 | 预期量级 | 实测（2026-08-31，扫描态） | 判定 |
| --- | --- | --- | --- | --- | --- |
| SBIRS A/B/C | 每周期交出一帧 | 1–80 | `inbox_sats=3` | 80/80 周期 `inbox_sats=3` | 通过 |
| SBIRS A | WFOV 命中 key=1 与 key=2 | 1–80 | 每周期两目标 | PE 角度样本 228 条（角度样本自周期 7 起每周期 3 条） | 通过 |
| SBIRS B | 只命中 key=1；key=2 地球遮挡 | 7–80（扫描捕获起始） | `sbirs.target_occulted` | 视图：目标 2「未检测」+ `sbirs.target_occulted` | 通过（按设计拒绝） |
| SBIRS C | 只命中 key=2；key=1 地球遮挡 | 1–80 | 同上 | 视图：目标 1「未检测」+ `sbirs.target_occulted` | 通过（按设计拒绝） |
| 融合 | key=1 双源 A+B；key=2 双源 A+C | 1–80 | `4,104` 与 `4,204` | `k1[4:10,104:10] \| k2[4:10,204:10]` | 通过 |
| 精度评估 | A/B 对 key=1 双星交会 | 7–80 | `dual_sat>0`，误差百米级 | `dual_sat_cycles=74/80`，交会 RMSE 85 m | 通过 |

冒烟：`all_metrics_sampled`、AHP 合法、综合分 0.430377 ∈ (0,1]、双星交会 74/80 周期。
缺的 6 个周期是 B 星扫描捕获 key=1 的**起始几何延迟**（期间 B 星无任何检出，
配对窗不制造量测）；建链后跟踪段每周期供数，扫描速率（6/30/60°/s 探针）不影响样本率。

落点/发射点 RMSE 约 200 km 量级：**不是交会失败**。本场景目标是中段高空平飞（ENU 水平速度），推演按弹道落点/发射点模型外推，几何对不上。位置确认看 `dual_sat_fix`，不要拿落点分当 GEO 交会指标。

每周期 stdout：`inbox_sats` 应为 3；融合航迹 key=1 带 4/104、key=2 带 4/204。

## 消息流（每周期）

```
main: app_scene.cycle = N
main: fusion->BeginCycle(world, N)
World::Step:
  各卫星 Sbirs (kMessage) → on_sbirs_frame_submitted → fusion.sbirs_inbox_
  ground_station GroundStationFusion → 重建 + 适配 + FusionEngine::Update
                                     → 从 inbox 按评估源通道挑 A/B 做 PE
```

## 构建与运行

```bash
source scripts/win/activate_git_bash.sh
scripts/1q.sh build VisualStudio.15.0-amd64-release --target sbirs_triple_sat_fix_messages
./build/VisualStudio.15.0-amd64/Release/bin/sbirs_triple_sat_fix_messages.exe
```

输出：`examples/log/sbirs_triple_sat_fix_messages/`

## 三维可视化

场景 exe 运行时同步落盘六份几何 CSV（与验收日志同目录）：`sbirs_sats`
（卫星 ECEF + 视场角 + 扫描参数 + 星下点方位）、`sbirs_scan`（每周期每星
WFOV 扫描中心方位，供扫描条带动画）、`sbirs_truth`（真值轨迹）、
`sbirs_los`（每星逐目标视线状态/测角/SNR，取组件调试视图快照）、
`sbirs_fused`（融合航迹，LLA 后验已转 ECEF）、`sbirs_dual_fix`（双星交会
误差样本）。

```bash
python3 examples/common/viz/sbirs_orbit_viewer.py examples/log/sbirs_triple_sat_fix_messages
# 生成的 HTML 离线自包含：顶部「星下点相对方位扫描」条带（橙框 = WFOV 足印随周期移动）+
# 下方 3D 视场锥随扫描方位旋转；拖拽旋转 / 滚轮缩放 / 时间轴播放 / 图层开关。
# --check 封口测角与交会复算同库一致：
python3 examples/common/viz/sbirs_orbit_viewer.py --check examples/log/sbirs_triple_sat_fix_messages
```

## 结论

**判定：通过（位置确认成立）。** 三星 GEO 覆盖 + 两两重叠区双星交会按设计工作；地球遮挡导致的「第三星看不到」是预期。综合分被落点/发射点拖低，不代表交会失败。
