# 修改目标标称 RCS 操作手册

> 适用：LRDR 远程识别雷达（RIR）链路里"目标标称 RCS"的调整。
> 结论先说：**只改模板里的 `rcs_m2` 与 `rcs_dbsm` 两行（三份文件同步）**，
> 它们同时决定【识别特征中心】与【雷达方程输入】；再按需检查两个钳位上限。

---

## 1. 为什么改模板就够

`lrdr_sensor_features.cpp::ApplyFeatureSet()` 里有这一句：

```cpp
if (_set.rcs_m2 > 0.0f) {
  _target->rcs = _set.rcs_m2;   // 模板值直接覆盖场景实体 RCS
}
```

⇒ 模板 RCS 会在每拍**覆盖**想定里给的目标 RCS，因此：

| 你想影响的东西 | 改模板是否够 |
|---|---|
| 回波功率 / SNR / 检测概率 / 最大探测距离（第 36/38/43 项）| ✅ 够（`target.rcs` 被覆盖）|
| 识别特征中心与判型（第 44/45/47A 项）| ✅ 够 |
| 想定里显示的实体属性 | ❌ 不改（但会被覆盖，无实际影响）|

---

## 2. 要改的文件（三份，内容必须一致）

| # | 路径 | 作用 |
|---|---|---|
| 1 | `ModelSource\CyModel\Equipment\Sensors\LRDR_SENSOR\remote_identification_radar_target_features.json` | 源文件（进 git）|
| 2 | `FOSimStudio\bin\App\config\remote_identification_radar_target_features.json` | **运行期实际加载** |
| 3 | `FOSimStudio\bin\App\config\remote_identification_radar\remote_identification_radar_target_features.json` | 备用副本 |

哪一份真正生效，看启动日志：

```text
[init] target_features path='…' loaded=1 sets=N
```

---

## 3. 改哪两行（必须成对改）

```json
{
  "id": "ballistic_icbm",
  "rcs_dbsm": -6.02,      // ① 对数口径（识别特征、库模板比对用）
  "rcs_m2": 0.25          // ② 线性口径（雷达方程用，单位 m²）
}
```

**换算**：`rcs_dbsm = 10 × log10(rcs_m2)`

| 目标 m² | dBsm | 典型含义 |
|---|---|---|
| 0.02 | −16.99 | 小碎片 / 极隐身 |
| 0.05 | −13.01 | 典型再入弹头（头向）|
| **0.1** | **−10.00** | 典型 RV 标称值（推荐）|
| **0.25** | **−6.02** | 略偏大的 RV / 带凸起 |
| 0.5 | −3.01 | 弹头 + 部分弹体 |
| 1.0 | 0.00 | 母舱级 |
| 2.0 | +3.01 | 整弹（含助推段）|

---

## 4. 三个连带项（按需）

| # | 位置 | 何时需要动 |
|---|---|---|
| **A** | `remote_identification_radar.json → hardware.rcs_physics.max_equivalent_radius_m` | 物理模型用 `r = sqrt(σ/π)` **反推半径**再钳到该上限。σ=0.25 ⇒ r=0.28（被钳到 0.15）；σ=0.02 ⇒ r=0.08（不钳）。**上限 ≤ 0.2 时通常不用改** |
| **B** | 同处 `max_rcs_m2` | 若 `0.7·σ_nom + 0.3·σ_phys` 超过它，输出会撞顶变常数（表现为"RCS 恒定"）。经验：`max_rcs_m2 ≥ 3 × σ_nom` |
| **C** | 特征库 `target_feature_database_v1.1.db → rcs_templates.mean_dbsm` | 库模板是识别比对中心。若标称改到 −6 dBsm，弹道四型库中心（−6…−10）仍相容；若标称改到 −13，建议同步下调库中心 |

---

## 5. 验证（三步）

```text
① 重启仿真（配置在传感器初始化时读入，热改无效）
② 看启动日志 [init] target_features path='…'   ← 确认读到改过的那份
③ 看第 43 项「本周期RCS」与第 47A 项「目标类别」
```

命令行快速核对三份文件是否一致：

```powershell
$files = @(
 'C:\software\workspace\1qfosim\BJ_10Y_WorkSpace\ModelSource\CyModel\Equipment\Sensors\LRDR_SENSOR\remote_identification_radar_target_features.json',
 'C:\software\workspace\1qfosim\FOSimStudio\bin\App\config\remote_identification_radar_target_features.json',
 'C:\software\workspace\1qfosim\FOSimStudio\bin\App\config\remote_identification_radar\remote_identification_radar_target_features.json')
$files | ForEach-Object { "$_"; Select-String -Path $_ -Pattern '"rcs_m2"|"rcs_dbsm"' | Select-Object -First 2 | ForEach-Object { "   $($_.Line.Trim())" } }
```

统计日志里的实际 RCS 分布：

```powershell
python - <<'PY'
import re, statistics, math
p=r'C:\software\workspace\1qfosim\FOSimStudio\log\RadarOutput.log'
v=sorted(float(m.group(1)) for m in (re.search(r'本周期RCS=([\d.]+)',l) for l in open(p,encoding='utf-8-sig',errors='replace')) if m)
print('n=%d 不同值=%d min=%.3f(%.1fdBsm) 中位=%.3f(%.1fdBsm) max=%.3f(%.1fdBsm)' % (
 len(v), len(set(round(x,4) for x in v)), v[0],10*math.log10(v[0]),
 v[len(v)//2],10*math.log10(v[len(v)//2]), v[-1],10*math.log10(v[-1])))
PY
```

判定标准：

| 现象 | 含义 |
|---|---|
| 中位接近你设的 `rcs_m2`、且有多个不同取值 | ✅ 正常（角度调制在起作用）|
| 所有行都是同一个值且等于 `0.7·σ + 0.3·max_rcs_m2` | ⚠️ 撞 `max_rcs_m2` 上限，见 §4-B |
| 所有行都是同一个值且等于 `rcs_m2` | 物理项未生效（`enable_physical_rcs=false`）|
| 第 47A 项大量 `未知` | 标称与库模板中心偏离过大，见 §4-C |

---

## 6. 推荐取值

| 目标类型 | 建议 `rcs_m2` | 依据 |
|---|---|---|
| **洲际弹头（RV）** | **0.05–0.1** | 真实锥体头向 −20…−10 dBsm |
| 弹头 + 少量凸起 | 0.1–0.25 | 当前工程值 0.25，略偏大但可用 |
| 母舱 / 整弹 | 1.0–10 | 侧向大面积散射体 |

---

## 7. 一次改动的完整检查清单

```text
□ 三份 target_features JSON 的 rcs_m2 / rcs_dbsm 已同步（10·lg 关系自洽）
□ max_equivalent_radius_m ≤ 0.2（除非标称很大）
□ max_rcs_m2 ≥ 3 × rcs_m2
□ 需要时同步了 rcs_templates.mean_dbsm
□ 重启仿真
□ 启动日志确认 path 指向 config 目录那份
□ 第 43 项 RCS 有分布、量级符合预期
□ 第 47A 项目标类别稳定（不大量"未知"）
```
