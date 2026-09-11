# Windows 本机构建注记（v141）

Status: active
Last-reviewed: 2026-08-27
Authority: local build environment, delivery tier runbook

CLAUDE.md 只保留命令流；本文件承载 Windows 本机开发的环境说明、排障顺序、
VS2015 交付档手册与历史事件。会话门禁的权威简版见
`.cursor/rules/windows-git-bash-build.mdc`。

入口是 `scripts/1q.sh`：包装脚本 exec Windows 的 `cmake.exe` / `ctest.exe`，
因此 **Git Bash 与 Cursor/WSL bash 的日常编测命令相同**。二者并不等价——WSL
里的 Linux 原生 cmake/ctest 不能拿去配 `VisualStudio.*` preset。

## 平台差异总览（macOS 主线 vs Windows v141）

macOS 是开发/CI 主线（完整 Conan 依赖集）；本机 Windows 使用 legacy v141 工具集
（裁剪依赖 + 内置文件日志）。Preset、生成器与依赖集差异：

| Aspect | macOS (mainline) | Windows (local v141) |
|---|---|---|
| preset | `llvm-ninja-debug(-local)` / `llvm-ninja-release(-local)` / `llvm-ninja-coverage` | `VisualStudio.15.0-amd64` |
| Generator | Ninja, single-config（build_type 随 preset 固定） | VS2026 generator + v141 (14.16) toolset, multi-config（`--config Debug/Release` 选档） |
| conan install | 每 preset 一次，单一 build_type | bootstrap 一次装 Debug + Release |
| Logging | spdlog/fmt (Conan)；库调试日志默认关，排库时才 `-DONEQ_ENABLE_FILE_LOG=ON` | 库调试日志默认关（不要开 `ONEQ_ENABLE_FILE_LOG`）；`component_attachment` 示例层自带 std::ofstream 后端（`CA_LOG_BACKEND_SPDLOG=0`） |
| HighFive / JSBSim | Conan 预装 | 均未装；FD=ON 时 JSBSim 需 third_party 源码或预编译树 |
| Coverage | `llvm-ninja-coverage` | 无 |

日常开发用 `*-local` 变体（机器本地 `CMakeUserPresets.json`，不进 git）；CI 用
`CMakePresets.json` 基础 preset。Trap：repo preset `llvm-ninja-release` 的
binaryDir 是 `build/llvm-ninja-release`（无 `-local` 后缀的历史遗留），而
`llvm-ninja-debug` 指向 `build/llvm-ninja-debug-local`——安装路径以实际 binaryDir 为准。

## 一次性环境准备

1. **壳 / `1q.sh`**：Git Bash（MINGW）或 Cursor 落到的 WSL bash 均可，前提是
   `source scripts/activate_1q_git_bash.sh` 后 `scripts/1q.sh doctor` 能解析到
   Windows `cmake`/`ctest`（`scripts/bin` 包装或 `*.exe`）。不要用 Linux 自带
   cmake 去配 `VisualStudio.*`。Cursor 打开本仓库时，用户集成终端默认仍是
   Git Bash（`.vscode/settings.json`）；Agent 若在 WSL 里，同样走 `1q.sh`。
2. `source scripts/activate_1q_git_bash.sh`（Git Bash 可写入 `~/.bashrc`；activate
   会补 cmake/ctest PATH、自愈 `core.hooksPath=.githooks`）。机器本地 cmake 路径
   覆盖：`scripts/1q_env.local.sh`（模板见 `.example`，已 gitignore）。
3. 建议把仓库目录加入 Windows Defender 排除（管理员 PowerShell：
   `Add-MpPreference -ExclusionPath 'D:\1q\1q'`），减少 `.obj` 文件锁并提速构建。

自检：`scripts/1q.sh doctor`——`cmake=`/`ctest=` 有路径。WSL 下 `uname=Linux` 且
出现 `SHELL_NOTE` 不算失败；`SHELL_WARNING` 或 `ctest=NOT FOUND` 才需要修 PATH。

## 排障顺序（避免打转）

按序归因，**禁止跳过前几步去改业务代码碰运气**：

1. **壳/PATH**：先 `source scripts/activate_1q_git_bash.sh && scripts/1q.sh doctor`。
   `cmake=`/`ctest=` 必须有路径。WSL（`uname=Linux` + `/mnt/d/...`）可继续，只要
   `1q.sh` 能 exec 到 Windows `cmake.exe`/`ctest.exe`。真正的失败态是 `ctest=NOT FOUND`
   或误用 Linux `/usr/bin/ctest` 去跑 VisualStudio 测试树。
2. **doctor**：见上；失败先修 PATH / `ONEQ_CMAKE_ROOT`，不要换业务代码碰运气。
3. **Eigen C4127 / SolveTriangular「卡住」**：v141 + `/W4` 会把 Eigen 模板里的常量
   `if` 打成 C4127，并附带 `SolveTriangular.h` / `GeneralBlockPanelKernel.h` 的模板
   实例化 note。日志可到数十万行，MSBuild 看起来像死锁，其实是警告洪流。
   `cmake/compilers/CompilerMSVC.cmake` 已对走 `oneq_apply_target_configuration`
   的 MSVC 目标（库组件、测试、示例）关 C4127 并定义
   `EIGEN_PERMANENTLY_DISABLE_STUPID_WARNINGS`。改完 CMake 后需一次 reconfigure，
   不必改业务代码。改编译选项会触发全量重编；且 VS preset 的 `jobs` 与 cl `/MP`
   叠乘会过订阅（12 核上 jobs=12 再每个工程 `/MP` 打满核，表现为构建极慢）。
   `VisualStudio.15.0-amd64-*` 的 jobs 已改为 4。日常只改业务 cpp 走增量，不要反复 configure。
4. **BOM**：带中文注释的 `.h/.cpp` 缺 UTF-8 BOM → C4819/C1083 类问题。
   pre-commit 钩子会在提交时自动补齐；**禁止**删中文注释来消 C4819。
5. **陈旧产物/文件锁**：见下节。
6. 最后才是编译错误本身。

## 陈旧增量产物 / 文件锁

- 症状：public 头布局变更后，模块测试批量 SEH `0xc0000005` / 无关 `bad_alloc`。
  原因：VS 生成器 `.tlog` 增量依赖跟踪对头文件布局移动不敏感，未全量重编，
  Debug 加固检查放大为崩溃。处理：`scripts/1q.sh clean-stale VisualStudio.15.0-amd64 <module_slug>`
  后重建（先于逻辑排查执行）。
- 历史教训（2026-08-17 erratum，2026-08-21 更正）：`SbirsExclusionCauseRecorderTest`
  整批 bad_alloc 曾被记为"已知基线失败"，实为上述陈旧产物；删除
  `build/.../src/sbirs_sensor` 产物强制全量重编后 203 个 sbirs 单测全绿。
  早期记录将其归因于 "Unity builds" 是错的——Unity 构建助手存在于
  `cmake/features/UnityBuild.cmake` 但从未启用。
- 写 `.obj` 报 `Permission denied`：先解除 IDE/杀毒占用（Defender 排除见上），
  不要当逻辑 bug。
- `integration::airborne_radar` 0xc0000409 是独立的既有问题，与本节无关。

## Eigen 编译耗时与日常优化

Eigen 是 header-only：没有 `.lib` 可链接，**每个包含 estimation 模板的 `.cpp`
都要在 TU 内解析并实例化 Eigen 模板**（LLT / SolveTriangular 等）。日志里路径
总是 `conan2/.../eigen3/...`，看起来像「在编 Eigen 库」，实际是在编你的 TU。

### 日常命令（短期）

1. **Release preset**：日常验证用 `VisualStudio.15.0-amd64-release`，不用 Debug
   （Debug 的 `/Od` + `/RTC1` 会显著拖慢 Eigen 模板编译）。
2. **精准 `--target`**：只编当前模块测试/示例，不要裸 `cmake --build`（默认
   `ALL_BUILD` ≈ 全解决方案）：
   ```bash
   scripts/1q.sh build VisualStudio.15.0-amd64-release --target 1q_fusion_unit_tests
   scripts/1q.sh test VisualStudio.15.0-amd64-release -R "unit::fusion"
   ```
3. **少动 `src/common/` 与 `include/1q/` 头**：`common/estimation/*.h`（如
   `EkfFilter.h`）被 fusion / sbirs / rir / airborne_radar 等多模块引用；改一行
   会级联重编所有 include 链上的 TU。业务改动优先留在模块 `src/<module>/*.cpp`。
4. **不要反复 `configure`**：仅依赖/CMake 变更后跑；改 CMake 编译选项会触发一次
   全量重编。`1q.sh build` **禁止隐式 reconfigure**——若 CMake 输入比
   `generate.stamp` 新，build 会失败并提示先 `scripts/1q.sh configure <preset>`；
   切换 preset 时必须 configure 对应的 configure_preset（例：build
   `VisualStudio.15.0-amd64-release` 对应 configure `VisualStudio.15.0-amd64`）。
   基座 preset 设 `CMAKE_SUPPRESS_REGENERATION=ON`，避免 VS/MSBuild 在构建期自行
   重跑 cmake。
5. **区分「检查」与「重编」**：增量有效时，第二次 build 应在数秒内结束，日志无
   `编译源文件 xxx.cpp`；若仍出现大量 cl 输出，先查是否改了 common 头或刚 configure。

### 已启用的构建加速（中 / 长期）

| 措施 | 作用 |
|---|---|
| `ENABLE_PCH=ON`（`VisualStudio.15.0-amd64` preset 默认） | 预编译 `<Eigen/Core>` + 常用 STL，减少重复头解析 |
| `src/common/estimation/EstimationInstantiations.cpp` | 对 **6×3 / 6×2** 的 Kalman/EKF/UKF/IMM 做显式实例化，`extern template` 阻止其它 TU 重复实例化 LLT |
| `/wd4127` + `EIGEN_PERMANENTLY_DISABLE_STUPID_WARNINGS` | 避免 Eigen 模板 warning/note 洪流拖慢 MSBuild |

显式实例化**不覆盖**单测里的其它维度（如 4×2）；那些 TU 仍本地实例化，不影响测试。

PCH 或显式实例化清单变更后需一次 reconfigure + 全量重编；之后恢复增量。

## VS2015 交付档（客户集成）

**存在原因**：客户的 VS2015 (v140, any Update) 环境在自己 TU 里编译我们的头文件，
无法要求加编译选项。无 BOM 的 UTF-8 会被按系统代码页（GBK）误读，中文注释错乱。
因此交付 = C++11 + 全部受控 C/C++ 文件带 UTF-8 BOM + 全链**无 `/utf-8`**（含
INTERFACE 传播）。BOM 使 cl.exe 无条件按 UTF-8 解码。维护链：
`scripts/utf8_bom.py`（convert/check/strip）+ pre-commit 自动补齐 + CI `check`
守卫 + `.editorconfig` 声明 `utf-8-bom`。

**Preset 选择**：

| Preset | 用途 |
|---|---|
| `1q_log_vs2015` | **交付验证**：v140 x64，无 Conan（先跑 `fetch_third_party.bat`），C++11，五层验收文件（sbirs/rir/fusion/inference/precision）ON |
| `VisualStudio.14.0-amd64-none` | 交付基座（同上去掉验收日志） |
| `VisualStudio.15.0-amd64` | Windows 开发主线：VS2026 + v141，C++17，Conan |
| `llvm-ninja-*` | macOS CI/开发主线 |
| `VisualStudio.14.0-amd64` | VS2015 + Conan——本机不可用（CMake 4.3.1 vs v140 toolset） |

**交付验证流程**（端到端约 15 分钟）：

1. `cmake --preset 1q_log_vs2015 && cmake --build --preset 1q_log_vs2015-release`
2. `cmake --install build/1q_log_vs2015 --config Release`
3. 同步 install → consumer（`D:\1q\1q_consumer`）：`include/1q/*`、`lib/1q.lib`、
   仓库 `examples/` → consumer `src/`（BOM 零改动镜像）。
4. consumer 以 **C++11、无 `/utf-8`** 构建（模拟客户 vcxproj）：
   `cmake -G "Visual Studio 14 2015" -A x64 && cmake --build --config Release`
5. 跑 demo 3 周期；构建日志零 C4819；运行目录 `log/` 下有验收文件
   （四段中文验收行）。库内部 `1q_library.log` 仍只承载 `PROJECT_LOG_*`，不再写验收项。

## 历史事件：UCRT 注册表死路径（已修复 2026-08-21）

独立安装的 x64 调试工具（WinDbg）曾把 64 位注册表视图
`HKLM\SOFTWARE\Microsoft\Windows Kits\Installed Roots\KitsRoot10` 覆盖为仅含
Debuggers/Catalogs 的目录，v141 的 ucrt.props 解析到死路径——裸目录构建报
`corecrt.h` not found (C1083) / LNK1104 (ucrtd.lib)（2026-08-19 复现）。当时的
规避是 preset/脚本注入 `UCRTContentRoot` + 禁止裸目录构建 + `scripts/bin/cmake`
拦截。2026-08-21 机器级修复：两个注册表视图均指向真实 SDK
（`C:\Program Files (x86)\Windows Kits\10\`），全部规避已退役（preset 不再注入
环境变量、拦截已移除），裸目录构建与 IDE MSBuild 恢复可用。

复发识别：裸构建再次批量 C1083/LNK1104（多为 SDK/调试器重装改写注册表）→
重新修注册表值，**不要**恢复环境变量管道。
