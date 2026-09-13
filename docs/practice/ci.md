# CI 持续集成

Status: active
Last-reviewed: 2026-09-13
Authority: build & test infrastructure

本仓库的持续集成（CI）由两个 GitHub Actions workflow 组成，按"快速门禁 vs 慢任务"分层。

## Job 矩阵总览

| Workflow | 触发 | Job | 内容 | 阻断 PR |
|---|---|---|---|---|
| `ci.yml` | push / PR | **guard** | 编码守卫（`scripts/utf8_bom.py check`：C/C++ 源文件必须 UTF-8 with BOM）+ `-L contract`：运行 live CTest inventory 中的 compiled contract 与 script guards | ✅ |
| `ci.yml` | push / PR | **build-test** | Debug 库/第一方示例构建 + 安装 consumer + `ci_required` + 全量 `unit` 分区 | ✅ |
| `nightly.yml` | cron 02:00 + 手动 | **performance** | Release 构建 + 性能/cxx11 兼容 gate | ❌ |
| `nightly.yml` | cron + 手动 | **flight-dynamic** | FD=ON + JSBSim 数据 + `unit::flight_dynamic` | ❌ |

设计原则：**CI 一上线就必须是绿的。** `ci_required` 覆盖跨层关键路径与架构守护；完整 `unit` 分区覆盖可重复的局部回归，二者均为 PR 阻断门禁。

## 平台范围

当前 CI 仅覆盖 macOS（`macos-14` arm64 runner）。理由：
- macOS 路径下 Conan 提供全部依赖（含 JSBSim 预编译包 `jsbsim/1.3.1`），最干净可靠。
- Windows v141 预设（`VisualStudio.15.0-amd64`）是本机 Windows 开发主线，但 CI 尚无
  Windows runner job；VS2015 C++14 预设、no-Conan 模式和 `scripts/win/fetch_third_party.bat`
  仍属未验收脚手架，不是支持声明（见 `build_and_test_governance.md` 与"扩展指南"）。

Contract 测试数量不在本文硬编码；以配置后执行
`ctest --preset <preset> -N -L contract` 的结果为准。当前集合同时包含按 domain 编译的 contract tests
与源代码/script guards，新增或删除注册项时无需人工维护一个容易漂移的总数。

## 本地复现

CI 跑的每条命令都可在本地复现，便于调试失败：

```bash
# === guard job（contract 守护）===
python3 scripts/utf8_bom.py check  # 编码守卫：C/C++ 源文件必须 UTF-8 with BOM
bash scripts/bootstrap_conan.sh llvm-ninja-debug
cmake --preset llvm-ninja-debug
ctest --test-dir build/llvm-ninja-debug -L contract --output-on-failure -j 4

# === build-test job（绿色门禁）===
bash scripts/bootstrap_conan.sh llvm-ninja-debug
cmake --preset llvm-ninja-debug -D ENABLE_EXAMPLES=ON
cmake --build --preset llvm-ninja-debug -j 4
cmake --install build/llvm-ninja-debug
cmake -S tests/consumer -B build/consumer/llvm-ninja-debug -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/build/llvm-ninja-debug/build/Debug/generators/conan_toolchain.cmake" \
  -DCMAKE_PREFIX_PATH="$PWD/build/install/llvm-ninja-debug"
cmake --build build/consumer/llvm-ninja-debug -j 4
# 运行安装 consumer 可执行程序（CI 同款 find 逐个执行）
find build/consumer/llvm-ninja-debug -maxdepth 1 -type f -perm -111 -exec {} \;
ctest --test-dir build/llvm-ninja-debug -L ci_required --output-on-failure -j 4

# === build-test 的 unit 分区 ===
ctest --test-dir build/llvm-ninja-debug -L unit --output-on-failure -j 4

# === nightly: performance ===
bash scripts/bootstrap_conan.sh llvm-ninja-release
cmake --preset llvm-ninja-release
cmake --build --preset llvm-ninja-release -j 4
ctest --test-dir build/llvm-ninja-release -L 'performance|sar_performance' --output-on-failure -j 4
ctest --test-dir build/llvm-ninja-release -R sar_cxx11_compat --output-on-failure -j 4

# === nightly: flight-dynamic（需先获取 JSBSim 数据）===
git clone --depth 1 https://github.com/JSBSim-Team/jsbsim.git third_party/jsbsim
bash scripts/bootstrap_conan.sh llvm-ninja-debug
cmake --preset llvm-ninja-debug -D ONEQ_ENABLE_FLIGHT_DYNAMIC=ON -D ENABLE_EXAMPLES=ON
cmake --build --preset llvm-ninja-debug -j 4
ctest --test-dir build/llvm-ninja-debug -R '^unit::flight_dynamic$' --output-on-failure -j 4
```

## 手动触发 nightly

GitHub 仓库 → **Actions** 标签页 → 左侧选 **Nightly** → 右上角 **Run workflow**。

适用于：修复后想立即验证、不想等到凌晨 2 点自动触发。

## 扩展指南

**新增 Windows job**：先实现 `docs/common/contract.md` 冻结的 shell/GitHub 依赖 bootstrap，锁定版本和
提交并校验下载内容；再在真实 Windows runner 上验证 configure、Debug/Release build、install 与独立
consumer build/run。Windows 主线 v141 Conan preset（`VisualStudio.15.0-amd64`）是候选主入口（JSBSim
来源待核实：Windows Conan 不装 jsbsim 包，FD=ON 需源码/预编译树路径）；VS2015 C++14 与 no-Conan
preset 仅属未验收脚手架，是否保留由全链证据决定。

**新增 lint job**（clang-format/tidy）：在 `ci.yml` 加独立 job 跑 `cmake --build --target format-check`（需 runner 装 clang-format）。与功能测试解耦，格式问题不阻塞代码验证。

## 故障排查

**guard job 失败**：通常是某个 `check_*.cmake` 守护脚本发现架构违规（如 include 方向、命名规约、公共 API 边界）。看失败信息里的 `VIOLATIONS:` 清单，对应修源码。这类失败最快修——纯文本规则，无需理解编译。

**Source encoding guard 失败**：git 追踪的 C/C++ 文件缺 UTF-8 BOM（多为新增文件由编辑器存成无 BOM）。修复：`python scripts/utf8_bom.py convert`（幂等，只补 BOM 不动其它字节）。背景：VS2015 集成端对无 BOM 文件按系统代码页（GBK）误读，中文注释错位截断；`.editorconfig` 已对 C/C++ 声明 `utf-8-bom`。

**Conan 缓存导致依赖旧**：改了 `conanfile.py` 后 CI 应自动重建缓存（key 含文件哈希）。若仍用旧包，检查 `CONAN_CACHE_KEY` 是否正确更新。

**JSBSim 数据 clone 失败**（flight-dynamic job）：网络问题或上游仓库变动。`git clone --depth 1` 应该很稳定；若失败重试即可。注意 JSBSim 源码库较大，浅克隆仍需数十秒。
