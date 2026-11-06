# 离线部署说明（内网机器：无网络、无 Python、无管理员权限）

本目录是"把绘图程序搬到内网就能跑"的自带运行时包：内网机器上不需要装 Python、
不需要联网 pip、不改注册表、不装系统服务，全部安装内容只写在本程序的 `.runtime\` 目录里。

---

## 一、包内组成

```text
log_print\
  plot_all.py                一键入口
  viz\                       绘图代码（7 个模块）
  figures\                   已生成的 43 张参考图
  offline\
    runtime\python-3.12.8-embed-amd64.zip   Python 3.12.8 免安装运行时（官方 embeddable）
    wheels\                                全部 wheel（numpy/pandas/matplotlib/scipy + 依赖 + pip）
    requirements.txt                       依赖清单（版本与开发环境一致）
    install.bat                            安装脚本
    run.bat                                启动脚本
    README.md                              本文件
```

安装后新增：

```text
log_print\.runtime\        Python 运行时 + 依赖（约 350 MB，可随时整个删掉）
log_print\viz\.cache\      日志解析缓存（自动生成）
log_print\figures\         出图目录（自动生成/覆盖）
```

---

## 二、内网部署（3 步）

1. **拷贝**：把整个 `log_print` 目录拷到内网机器任意可写位置。
   不需要拷 `.venv\`（开发机虚拟环境）、`viz\.cache\`、`log.zip`、`dist\`；
   若用 `offline\pack_delivery.ps1` 打包，这些已经自动排除。

2. **安装**：双击 `offline\install.bat`（或在 cmd 里执行）。
   脚本会解压 Python、装好 pip、从 `wheels\` 本地装齐依赖，并自检版本。
   看到 `Install OK.` 即成功；重复执行是安全的（已装则复用）。

3. **放日志并出图**：
   - 把 6 个日志文件和天线增益 CSV 放进 `log_print\log\`（或放任意目录，用 `--log-dir` 指过去）；
   - 先自检：`offline\run.bat --doctor`
   - 再出图：`offline\run.bat`

---

## 三、常用命令

```bat
offline\run.bat                          :: 生成全部 43 张图 -> figures\
offline\run.bat --only 1-3               :: 只画 1~3 号图
offline\run.bat --only 1,5,9             :: 只画指定编号
offline\run.bat --refresh                :: 忽略缓存，重新解析日志
offline\run.bat --list                   :: 列出全部图目录
offline\run.bat --dpi 300                :: 改出图分辨率（默认 200）
offline\run.bat --doctor                 :: 环境/数据自检（不画图）
offline\run.bat --log-dir D:\data\run1   :: 指定日志目录
offline\run.bat --out-dir  D:\out\run1   :: 指定出图目录
```

等价的三个环境变量（不写死任何路径）：

| 变量 | 作用 | 默认 |
| --- | --- | --- |
| `LOG_PRINT_LOG_DIR` | 日志目录 | 程序目录下 `log\` |
| `LOG_PRINT_OUT_DIR` | 出图目录 | 程序目录下 `figures\` |
| `LOG_PRINT_CACHE_DIR` | 解析缓存目录 | `viz\.cache\`；不可写时自动用 `%TEMP%\log_print_cache` |

---

## 四、内网环境的适配点（已处理）

| 风险 | 处理方式 |
| --- | --- |
| 日志目录不同 / 按日期分子目录 | `--log-dir` 指定；同名文件会在子目录里递归查找，多份时取最新并提示 |
| 日志编码是 GBK/GB18030 | 按错误密度自动识别 UTF-8 / GB18030，自检里打印识别结果 |
| 缺中文字体，中文变方框 | 按 Microsoft YaHei → 仿宋/黑体 → … 顺序挑可用字体；一个都没有时启动即告警 |
| 程序目录只读（受控目录/刻盘） | 缓存自动退化到 `%TEMP%`；出图用 `--out-dir` 指到可写目录 |
| 系统没有 Python | 自带 3.12.8 embeddable 运行时，只依赖本目录 |
| 不能联网 | pip 全程 `--no-index --find-links wheels`，并屏蔽用户 pip 配置 |
| 控制台编码导致输出报错 | 启动脚本 `chcp 65001` + `PYTHONIOENCODING=utf-8`，重定向也不中断 |
| 无管理员权限 | 不写注册表、不写 Program Files、不改 PATH |

---

## 五、耗时参考

- 首次解析 640 MB 日志：约 10 秒（结果缓存到 `viz\.cache\`，约 170 MB）。
- 缓存就绪后出 43 张图：约 35~40 秒。
- 换一批同格式日志后，缓存键（文件大小+mtime）变化会自动重解析，无需手动清理；
  `--refresh` 可强制清缓存重解析。

---

## 六、卸载 / 重装

- 卸载：删掉 `log_print\.runtime\` 即可，系统其它位置没有任何残留。
- 重装：删掉 `.runtime\` 后重新执行 `offline\install.bat`。
- 无网络也能重装：`wheels\` 与 `runtime\` 里的 zip 都被保留。

---

## 七、开发机重建离线包（需要联网，仅打包者执行）

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File offline\build_offline.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File offline\pack_delivery.ps1   # 可选：打成交付 zip
```

`build_offline.ps1` 会下载 Python embeddable 运行时与 `win_amd64 / cp312` 全部 wheel；
`pack_delivery.ps1` 只打包内网真正需要的文件（排除 `.venv`、缓存、日志、`dist`）。
