# 计划：TrackFilter 滑动窗口 + Reset

## 行动项

- [ ] 把 `TrackFilter::Push` 改成滑动窗口：窗口未满时累加，满后淘汰最旧的样本
- [ ] 新增 `TrackFilter::Reset()`，清空窗口状态并复位累加器
- [ ] `docs/airborne_radar/algorithms.md` 补上窗口语义与 Reset 的说明
