# 计划：把 TrackFilter 改成真正的滑动窗口

## 背景

`TrackFilter` 目前只是把全部输入累加后求平均，`window_` 参数被存下来却从未使用。
窗口语义是模块对外承诺的一部分，必须落实。

## 行动项

- [ ] 把 `TrackFilter::Push` 改成滑动窗口：窗口未满时累加，满后淘汰最旧的样本
- [ ] 新增 `TrackFilter::Reset()`，清空窗口状态
- [ ] 头文件补上窗口状态成员，并保持 C++11 可编译
- [ ] `docs/airborne_radar/algorithms.md` 补上窗口语义说明
