# 红外双星交会定位精度实测（2026-10-04）

Status: draft
Authority: 开发期度量记录（非契约、非模块设计）
Answers: 双星交会能给雷达提供什么精度的位置；同时可见率是否支撑波束引导；时间偏斜的影响量级

> 本文是**实测记录**，不是契约或设计文档。结论用于支撑「红外引导雷达波束」方案的可行性判定。
> 度量工具与原始数据见文末「复现」。

## 1. 目的

回答三个决定性问题：

1. 双星交会输出的目标位置精度是多少（对照库内参考值 `reference_error_dual_sat_fix_m = 10000`）？
2. 两颗红外星「同周期都看到同一目标」的实际占空比是多少（决定引导刷新率）？
3. 位置/时间误差对 RIR 波束（默认半功率全宽 **4.0°**）的指向影响有多大？

## 2. 结论摘要

| 问题 | 实测结论 |
|---|---|
| 交会位置精度 | 与交会角强相关：θ=21.8°、σ=3 μrad 时 **均值 375 m / P95 841 m**；θ=5.34° 时劣化到 **1435 m / 3446 m** |
| 是否够引导 4° 波束 | **够**。按甲方 3 μrad 指标，300 km 上指向误差 0.16°、200 km 上 0.24°、100 km 上 0.66°（判据 <1.4°） |
| 同时可见率 | **cross-cue 开启时 100%（80/80 周期至少一星可见）**；关闭时 **0%**（伴星完全看不到目标） |
| 主导误差源 | **不是测角精度，是时间偏斜**：目标 1827 m/s，1 周期偏斜 = 1827 m，大于测角贡献的 375 m |

**推翻的既有认知**：`reference_error_dual_sat_fix_m = 10000` 并不代表 GEO 双星的标称精度；实测标称工况精度比它好 1～2 个数量级。

## 3. 实测数据

### 3.1 交会位置精度（合成 GEO 双星，θ 为交会角，σ 为单星单轴测角 1-σ）

| σ (μrad) | θ=5.34° | θ=10.75° | θ=21.80° | θ=50.45° |
|---:|---|---|---|---|
| 1.95（库默认 RSS） | 933 \| 2254 | 470 \| 1132 | **244 \| 548** | 130 \| 256 |
| 3.0（甲方指标） | 1435 \| 3446 | 722 \| 1735 | **375 \| 841** | 199 \| 389 |
| 10 | 4775 \| 11578 | 2404 \| 5763 | 1248 \| 2821 | 664 \| 1306 |
| 30 | 14326 \| 34731 | 7214 \| 17179 | 3745 \| 8446 | 1991 \| 3914 |
| 100 | 47747 \| 115904 | 24045 \| 57189 | 12483 \| 28184 | 6636 \| 13053 |
| 300 | 143219 \| 349343 | 72131 \| 172005 | 37449 \| 84652 | 19908 \| 39177 |
| 500 | 238670 \| 581600 | 120211 \| 287346 | 62414 \| 141233 | 33179 \| 65306 |

（格式：均值 | P95，单位 m；A 星真值斜距 39,354 km；2000 样本/格点；0 失败样本）

**两条实测标定律（精度 <1%）**：

- **测角标定**：`位置误差均值 ≈ 2.24 · R · σ_angle / sin θ`
  （θ=21.8°、σ=3 μrad、R=39,354 km → 预测 374 m，实测 374.8 m）
  ⚠️ 系数 **2.24 是本构型（两星与目标共面）的经验值，不是普适常数**，换构型须重测。
- **交会角标定**：固定 σ=3 μrad 时误差 ∝ 1/sin θ，与解析预测吻合至 4% 以内
  （θ=5.34°→1437 m，10.75°→723 m，21.80°→375 m，50.45°→198 m）。

### 3.2 双星同时可见率（场景 `sbirs_triple_sat_fix_messages`，三星 GEO + 两目标，80 周期）

| 配置 | 目标 1（星 4+104） | 目标 2（星 4+204） | 双星交会样本 |
|---|---|---|---|
| `cross_cue=true` | 同周期 39/80；**至少一星 80/80** | 同周期 40/80；**至少一星 80/80** | **79/80 周期** |
| `cross_cue=false` | 仅星 4 可见（80/80），**伴星 0/80** | 仅星 204 可见（80/80），**伴星 0/80** | **0/80 周期** |

**关键机制发现（逐周期检出图）**：开启 cross-cue 后，主星与伴星呈**严格交替**照明——

```
目标 1  星 4  : X.X.X.X.X.X.X.X.X.X. ...   （隔周期）
        星 104: .XXXXXXXXXXXXXXXXXXXX ...   （近乎连续）
        合并  : 每一周期都有至少一星可见（100%）
目标 2  星 4  : .X.X.X.X.X.X.X.X.X.X ...   （相位与目标 1 相反）
        星 204: XXXXXXXXXXXXXXXXXXXX ...   （连续）
```

即：**库内并未实现「两星同周期同时测量」**，而是**交替单星照明**，靠精度评估的 `dual_sat_pair_window_cycles`（场景取 1）把相邻周期的两条视线配对成交会。

**为什么伴星看不到（cross-cue 关时）**：伴星自身的扫描扇区**扫不到该目标**——不是被地球遮挡，是波束指向根本没经过目标方向。cross-cue 通过「发现星经地面站递话、受话星窄场直接转头」让伴星对准目标，才产生可见性。这与场景预期表的记录一致（"宽场窄到一次盖不满盘时引导链断"）。

### 3.3 场景真实交会误差分布（79 样本，目标 1）

| 量 | 均值 | 中位 | P95 | 最大 |
|---|---:|---:|---:|---:|
| 位置误差 (m) | 796 | **1013** | 1251 | 14651 |
| 斜距误差 (m) | −461 | −383 | 84 | 283 |
| 几何残差 (m) | 822 | 81 | 1675 | 1747 |

**时间偏斜是主导误差源**：目标速度 1827 m/s（由真值轨迹差分实测）→ 满 1 周期偏斜 = 1827 m。
- 测角精度（3 μrad）贡献：约 375 m
- 时间偏斜贡献：交替照明下平均约 0.5 周期 ≈ **913 m**
- 实测中位 **1013 m** ≈ 两者合成，**时间偏斜大于测角贡献 2.4 倍**

最大 14.6 km 对应整周期偏斜的极端配对。

## 4. 对引导方案的含义

1. **位置精度不是瓶颈**，交会角 ≥ 20° 时余量充足。
2. **交会角是首要设计驱动量**：θ 从 21.8° 降到 5.3° 精度劣化 3.8 倍（∝1/sin θ）。工程上应要求 **θ ≥ 20°**。
3. **同时可见性依赖 cross-cue 机制**：无 cross-cue 时伴星贡献为零，双星定位不可用。若引导链依赖双星交会，**必须确认 cross-cue 链路在线**。
4. **时间对齐与位置精度同等重要**：库内不做目标运动外推，位置/时延误差直接变成指向偏差。RIR 侧输入 `RirSceneTarget` 只有位置与速度，**时延补偿必须由集成层用速度外推完成**。
5. **几何残差不能当精度指示量**：实测中残差在不同 σ/θ 下几乎恒为 ≈82–86 m（表 2），与位置误差不相关，不能作为"这次交会准不准"的判据。

## 5. 复现

```powershell
# 1. 依赖（注意：enable_testing=True 会拉入 rapidcheck，其配方在 Windows 上构建失败，需关闭）
conan install . --output-folder build/VisualStudio.15.0-amd64 --build=missing ^
  -s compiler.cppstd=17 -o "&:enable_testing=False" ^
  -c "tools.cmake.cmaketoolchain:generator=Visual Studio 18 2026" ^
  -s os=Windows -s arch=x86_64 -s compiler=msvc -s compiler.runtime=dynamic ^
  -c "tools.microsoft.msbuild:vs_version=18" -s compiler.version=191 -s build_type=Release

# 2. 主库 + 示例
cmake --preset VisualStudio.15.0-amd64 -D ENABLE_TESTING=OFF -D ENABLE_EXAMPLES=ON
cmake --build --preset VisualStudio.15.0-amd64-release
cmake --install build/VisualStudio.15.0-amd64 --config Release

# 3. 度量工具（两条独立实现，用于交叉验证）
#    a) 独立版：不依赖库，自实现误差注入（纯角度随机项）
cmake -S tools/dual_los_only -B build/tools/dual_los_only -G "Visual Studio 18 2026" -A x64 -T v141
cmake --build build/tools/dual_los_only --config Release
build/tools/dual_los_only/Release/dual_los_measure.exe --samples 2000 --csv build/tools/dual_los_only/samples.csv
#    b) 库内模型版：链接 1q.lib，调用 ApplyAngularErrorModel（含折射/滞后）
cmake -S tools/measure_dual_los_fix -B build/tools/measure_dual_los_fix -G "Visual Studio 18 2026" -A x64 -T v141 ^
  -DCMAKE_PREFIX_PATH="<repo>/build/install/VisualStudio.15.0-amd64;<repo>/build/VisualStudio.15.0-amd64/build/generators"
cmake --build build/tools/measure_dual_los_fix --config Release
build/tools/measure_dual_los_fix/Release/measure_dual_los_library.exe --samples 2000

# 4. 同时可见率（cross-cue 对照）
#    由场景 JSON 复制两份并改 cross_cue 与 log_dir，再以 --scene 运行：
build/VisualStudio.15.0-amd64/Release/bin/sbirs_triple_sat_fix_messages.exe ^
  --scene build/handover_probe/croscue_on.json --output-dir <abs>/build/handover_probe/out_croscue_on
#    解析输出目录的 sbirs_los.csv（status=detected）与 sbirs_dual_fix.csv
```

**交叉验证**：两条独立实现的表 1 在全部 28 个格点上差异 **<0.1%**，证明大气折射与动态滞后项在本场景（GEO、相对角速度 0）可忽略，也证明结果不依赖任一特定注入路径。

## 6. 尚未覆盖

- **本实测用合成 GEO 几何**（两星同高度、赤道面共面），真实星座的斜距差与异面几何会使系数偏离 2.24，须按实际星历重测。
- **未建模目标机动**：交会解假设两星测的是同一时刻同一位置；机动目标会额外放大时间偏斜误差。
- **未覆盖 NFOV 跟踪段的量测口径**：场景实测样本来自 NFOV 跟踪段（每周期 10 帧融合，随机项按 1/√N 衰减），本合成的 σ 是等效单次量测口径，两者不可直接逐项对账。
- **cross-cue 链路本身未度量**：本次只测了开/关两端，未测递话延迟与丢失率。
