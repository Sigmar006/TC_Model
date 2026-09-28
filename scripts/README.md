# scripts/ 平台矩阵

一张表说清哪个脚本属于哪个平台。日常构建/测试入口一律是 `scripts/1q.sh`。

| 脚本 | 平台 | 用途 | 谁调用它 |
|---|---|---|---|
| `1q.sh` | 跨平台 | 构建/测试统一入口（bootstrap / configure / build / test / install / clean-stale / doctor） | 人、`docs/practice/*`、示例说明 |
| `bootstrap_conan.sh` | 跨平台 | 按 preset 派生参数执行 conan install，生成 toolchain 与 CMakeDeps | 人；CI（`ci.yml` ×2、`nightly.yml` ×2）；`tests/contract/check_preset_provider_contract.cmake`（**按路径 file(READ)，移动即断 contract 门禁**） |
| `utf8_bom.py` | 跨平台 | UTF-8 BOM 维护（convert / check / strip） | CI `check` 作业；`.githooks/pre-commit` 的提示文案 |
| `check_source_literals.py` | 跨平台 | 字面量字符集守卫：字符串/字符/raw 字面量内不得出现 ANSI 代码页（GBK）表示不了的字符（MSVC 会静默编成 `?`；`u8"..."`/`L"..."` 前缀跳过） | CI `check` 作业 |
| `1q_clean_stale.sh` | 跨平台 | 删指定 preset 下某模块的增量产物，强制该模块全量重编（治头文件布局变更后的脏 TU） | `1q.sh clean-stale` |
| `activate_1q_git_bash.sh` | 转发壳（无逻辑） | 仅为已写入用户 `~/.bashrc` 的老路径保留；迁移完成后删除 | 用户 `~/.bashrc` |
| `lib/1q_env.sh` | 跨平台 | 环境解析：`ONEQ_ROOT`、cmake/ctest PATH（内含 Windows 兜底搜索） | 上述所有 shell 脚本 source 它 |
| `lib/1q_preset_query.py` | 跨平台 | 解析 `CMakePresets.json`/`CMakeUserPresets.json`，给出 configurePreset 与 binaryDir | `1q.sh install`、`lib/1q_build_guard.py`、`1q_clean_stale.sh` |
| `lib/1q_build_guard.py` / `.sh` | 跨平台 | 禁止 build 隐式 reconfigure（Ninja 树无 `generate.stamp`，放行） | `1q.sh build` |
| `win/activate_git_bash.sh` | **Windows 专用** | Git Bash/WSL 会话激活：补 PATH、`1q` 命令、自愈 `core.hooksPath` | 用户 `source` |
| `win/fetch_third_party.bat` | **Windows 专用** | 无 Conan（VS2015 交付链）时把第三方源码拉进 `third_party/` | 人、交付流程 |
| `win/env.local.sh.example` | **Windows 专用** | 机器本地 cmake 路径覆盖模板 → 复制为 `scripts/1q_env.local.sh`（已 gitignore） | 人 |
| `win/bin/cmake`、`win/bin/ctest` | **Windows 专用** | 包装器：exec Windows 的 `cmake.exe` / `ctest.exe` | `lib/1q_env.sh` 前插到 PATH |

## 约定

- 新脚本按平台落位：Windows 专用进 `scripts/win/`；跨平台留在 `scripts/` 或 `scripts/lib/`。目前没有 macOS 专用的构建脚本（`tools/` 下的分析/绘图工具不属此列）。
- `scripts/1q_env.local.sh` 是**路径契约**（由 `lib/1q_env.sh` 读取、被 `.gitignore` 忽略），不要移动；模板在 `win/env.local.sh.example`。
- `lib/1q_env.sh` 用 `dirname(BASH_SOURCE)/../..` 推导 `ONEQ_ROOT`，因此**它必须始终位于 `scripts/lib/` 这一层**。
- Windows 环境、排障顺序与交付档手册：`docs/practice/windows_local_build.md`；会话门禁简版：`.cursor/rules/windows-git-bash-build.mdc`。
