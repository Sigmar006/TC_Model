# 计划：TrackFilter 滑动窗口

## 行动项

- [ ] 把 `TrackFilter::Push` 改成滑动窗口：窗口未满时累加，满后淘汰最旧的样本
- [ ] 头文件补上窗口状态成员
