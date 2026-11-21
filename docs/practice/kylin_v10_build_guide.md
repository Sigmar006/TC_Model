# 银河麒麟 V10（飞腾 / 鲲鹏 aarch64）编译说明

> 适用对象：在**银河麒麟 V10 国防版 / 桌面版 SP1~SP3**（aarch64，飞腾 S2500/FT-2000+、鲲鹏 920）
> 上从源码构建 `1q` 静态库与测试的开发者。
> 本文只讲**怎么把它编出来**；每处兼容性改动的原因与证据见
> [`kylin_v10_aarch64_build_compat.md`](kylin_v10_aarch64_build_compat.md)。
>
> 默认产出：`lib1q.a`（静态）+ 头文件 + `1qTargets.cmake`。默认**不**产出 `.so`。

---

## 0. 一键脚本（推荐）

```bash
cd <1q 根目录>
bash scripts/build_kylin.sh --release --tests          # 体检 → 依赖 → 配置 → 编译 → 报告
bash scripts/build_kylin.sh --selfcheck                # 只体检，不编译
```

脚本做的事：

| 阶段 | 行为 |
|---|---|
| 体检 | 打印 aarch64 / GCC / CMake / glibc / Conan / Ninja / flatc；缺 CMake 只在**构建阶段**报错，`--selfcheck` 照样出报告 |
| 能力探针 | 实测 `<filesystem>` 是否可用、`mallinfo2` 是否存在、`long double` 尾数位（aarch64=113 / x86-64=64）、`-ffp-contract=off` 是否支持 |
| 选路线 | CMake ≥ 3.21 且有可读的 preset → `--preset`；否则自动改走显式 `-D`（CMake 3.16 可用） |
| 依赖 | `conan install` 带齐 `enable_testing` / `enable_hdf5` / `enable_flight_dynamic`，`--offline` 时加 `--no-remote` |
| 编译 | `cmake --build --parallel $(nproc)`，全量输出落 `<构建目录>/oneq_kylin_build.log` |
| 失败诊断 | 按已知错误模式翻译成「改什么」，而不是丢一屏 ninja 报错（覆盖 HighFive 裸名、`-z stack-size`、`<filesystem>`、`mallinfo2`、pthread、LTO、flatc、离线 remote、glibc 版本、presets version 等） |

常用开关：

```bash
bash scripts/build_kylin.sh --offline                  # 完全离线（国防版常态）
bash scripts/build_kylin.sh --debug --ctest            # 调试构建 + 跑 ctest
bash scripts/build_kylin.sh --install --install-dir /opt/1q
bash scripts/build_kylin.sh --hdf5                     # 打开 HighFive/HDF5（aarch64 需源码构建 HDF5 全家桶）
bash scripts/build_kylin.sh --flight-dynamic           # 打开 JSBSim 模块
bash scripts/build_kylin.sh --dry-run                  # 只打印将执行的命令
```

> `--dry-run` 不需要本机装 Conan/CMake 也能看命令组成，适合先对一遍参数再上机。

## 0.1 手工四步（CMake ≥ 3.21，联网或已预热 Conan 缓存）

```bash
cd <1q 根目录>
bash scripts/bootstrap_conan.sh linux-aarch64-release     # 生成 conan_toolchain.cmake
cmake --preset linux-aarch64-release
cmake --build build/linux-aarch64-release -j"$(nproc)"
ctest --test-dir build/linux-aarch64-release --output-on-failure -j4
```

完全离线（国防版常态）在第一条命令前加一行：

```bash
export ONEQ_CONAN_OFFLINE=1     # 让 conan 只读本地缓存，不尝试访问 remote
```

麒麟 V10 出厂的 CMake 版本随 SP 不同，可能低至 **3.16**（无 `--preset`），遇此情况请直接走
**§4 路线 B**，其余步骤完全等价。

> **preset 读不出来时先看 `CMakeUserPresets.json`**：Conan 2 生成的该文件用 presets `version: 4`，
> 而 CMake 3.21/3.22 只支持到 version 3，报错形如
> `CMake Error: Could not read presets from <目录>: Unrecognized "version" field`——
> **报错只给目录、不给文件名**，极易误判成工程的 `CMakePresets.json` 坏了。
> 处理：`rm CMakeUserPresets.json` 后重跑，或用 `--route manual` 走显式 `-D`（脚本已自动规避）。

---

## 1. 环境要求与体检

| 组件 | 最低 | 说明 |
|---|---|---|
| CPU / OS | aarch64 / 麒麟 V10 | 飞腾、鲲鹏均按 `armv8` 处理 |
| GCC / G++ | **7.3** | 麒麟 V10 自带。C++17 必需；`<filesystem>` 不存在，构建系统已自动切 `experimental` |
| CMake | **3.16** | 3.16~3.20 无 `--preset`（走路线 B）；≥3.21 可用 preset |
| Ninja 或 Make | 任一 | 推荐 Ninja（`apt install ninja-build` 或本地 rpm） |
| Conan | **2.x** | 仅路线 A/B 的 conan 依赖路径需要 |
| Python 3 | 3.6+ | Conan 依赖；`scripts/lib/1q_preset_query.py` 也需要 |
| glibc | 2.28（系统自带） | 决定 `mallinfo2` 是否可用（< 2.33 不存在，代码已分流） |

一次体检：

```bash
uname -m                                  # 期望 aarch64
gcc --version | head -1                   # 期望 7.3.x
cmake --version | head -1                 # 3.16.x 或更高
ninja --version 2>/dev/null || make --version | head -1
conan --version 2>/dev/null || echo "no conan"
python3 --version
ldd --version | head -1                   # 期望 2.28
getconf LONG_BIT                          # 期望 64      —— 32 位环境不支持
```

> 若 `gcc` 指向 4.8/5.x，先 `export CC=gcc-7 CXX=g++-7`（或 `update-alternatives`）再继续。

---

## 2. 依赖准备

工程依赖只从 `conanfile.py` 来（Eigen 3.4、Boost 1.85、nanoflann、flatbuffers、zlib、sqlite3；
测试再加 gtest）。**`PACKAGE_MANAGER=none`（third_party 源码路径）当前不可用**——
`third_party/` 只有 `lib/`、`win/` 两个目录，没有源码；见 §9 最后一行。

### 2.1 有网 / 内网有 Conan 私服

直接跑 §0，`--build=missing` 会把 CCI 上没有 armv8 预编译包的依赖（Eigen、Boost、zlib、
sqlite3、flatbuffers、spdlog、fmt、gtest 等，共十余个）从源码编出来。**首次会很久**（30~90 分钟，
视 CPU 核数），属正常。

### 2.2 完全离线：先在能上网的机器上把缓存搬过来

在**同为 aarch64、同 GCC 大版本**的机器上：

```bash
conan install . --output-folder build/linux-aarch64-release \
  --build=missing -s compiler.cppstd=17 -s arch=armv8 \
  -o "&:enable_testing=True" -o "&:enable_hdf5=False" -o "&:enable_flight_dynamic=False"
conan cache path --folder . > /dev/null 2>&1 || true
# 打包整份缓存（按本机 conan home 实际路径替换，Conan 2 默认 ~/.conan2）
tar -czf conan_cache_armv8.tar.gz -C ~/.conan2 .
```

拷到麒麟后：

```bash
tar -xzf conan_cache_armv8.tar.gz -C ~/.conan2
export ONEQ_CONAN_OFFLINE=1
bash scripts/bootstrap_conan.sh linux-aarch64-release
```

> 缓存里的包记录包含 `arch=armv8`、`compiler.version=7.x` 等 profile 指纹。
> 若目标机 GCC 小版本不同（7.3 vs 7.5），Conan 会认为 `package_id` 不同而重新构建，
> 此时要么统一编译器版本，要么接受源码重建。

### 2.3 flatc（FlatBuffers 编译器）

7 个模块（airborne_radar / esr / eos / rir / sar / sbirs / ecm）在**构建期**用 `flatc` 把
`schemas/replay/*.fbs` 生成 C++ 头。因此：

* **本机编译（aarch64 上编 aarch64）**：`flatbuffers` 的 Conan 包会连 `flatc` 一起构建，
  无需额外操作。前提是 `conanfile.py` 里保持 `flatbuffers/*:header_only=False`（已改好；
  设成 `True` 的包里没有 `flatc`，会在配置期直接失败）。
* **交叉编译（x86_64 主机编 aarch64）**：`flatc` 必须是**能在这台主机上运行的**版本，显式指定：

  ```bash
  -DONEQ_FLATC_EXECUTABLE=/usr/local/bin/flatc
  ```

---

## 3. 构建路线 A：preset（CMake ≥ 3.21）

路线 A 用仓库自带的 `linux-aarch64-release` / `linux-aarch64-debug` preset，
它们继承隐藏的 `linux-base`（`PACKAGE_MANAGER=conan`、`ONEQ_ENABLE_HDF5=OFF`、`ONEQ_STRICT_FP=ON`、
`BUILD_SHARED_LIBS=OFF`）。

```bash
cd <1q 根目录>

# 1) 生成 toolchain（bootstrap 内部会 conan install；离线时先 export ONEQ_CONAN_OFFLINE=1）
bash scripts/bootstrap_conan.sh linux-aarch64-release

# 2) 配置（preset 里已指向 build/linux-aarch64-release/build/Release/generators/conan_toolchain.cmake）
cmake --preset linux-aarch64-release

# 3) 编译
cmake --build build/linux-aarch64-release -j"$(nproc)"

# 4) 测试（preset 已开 ENABLE_TESTING）
ctest --test-dir build/linux-aarch64-release --output-on-failure -j4
```

Debug（含 `examples`，便于排查）：

```bash
bash scripts/bootstrap_conan.sh linux-aarch64-debug
cmake --preset linux-aarch64-debug
cmake --build build/linux-aarch64-debug -j"$(nproc)"
```

也可以统一用封装脚本（preset 名在 configure/build/test 三处一致）：

```bash
bash scripts/1q.sh bootstrap linux-aarch64-release
bash scripts/1q.sh configure linux-aarch64-release
bash scripts/1q.sh build     linux-aarch64-release -j"$(nproc)"
bash scripts/1q.sh test      linux-aarch64-release -j4
```

---

## 4. 构建路线 B：CMake 3.16 / 显式 `-D`

麒麟出厂 CMake 常为 3.16（无 `--preset`），或现场不允许改 CMake 版本时用这条。
**不需要** `CMakePresets.json`，但需要先自己跑一次 `conan install` 拿到 toolchain。

```bash
cd <1q 根目录>
export ONEQ_CONAN_OFFLINE=1                 # 离线才加

# 1) conan install（输出目录自己定，这里用 local/）
conan install . --output-folder local \
  --build=missing -s compiler.cppstd=17 -s arch=armv8 \
  -o "&:enable_testing=False" \
  -o "&:enable_hdf5=False" \
  -o "&:enable_flight_dynamic=False"

# 2) 配置：toolchain 在 local/build/Release/generators/conan_toolchain.cmake
cmake -S . -B local -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/local/build/Release/generators/conan_toolchain.cmake" \
  -DPACKAGE_MANAGER=conan \
  -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_CXX_STANDARD=17 \
  -DONEQ_ENABLE_HDF5=OFF \
  -DONEQ_STRICT_FP=ON \
  -DENABLE_TESTING=OFF -DENABLE_EXAMPLES=OFF -DENABLE_INSTALL=ON

# 3) 编译
cmake --build local -j"$(nproc)"
```

要点：

* `CMAKE_TOOLCHAIN_FILE` **必须**在第一次配置时就给（`ProjectSetup.cmake` 会校验其存在，
  否则直接 `FATAL_ERROR` 提示先跑 bootstrap）。
* `-DCMAKE_CXX_STANDARD=17`：`if constexpr` 等用法需要 C++17；不要退回 11/14。
* 想要测试就把 `-o "&:enable_testing=True"` 与 `-DENABLE_TESTING=ON` 同时打开
  （两处必须一致，否则要么缺 gtest、要么装了 gtest 却不用）。

---

## 5. 构建路线 C：x86_64 主机交叉编译 aarch64

需要交叉工具链（如 `aarch64-linux-gnu-g++`）与 **aarch64 版的依赖包**（Conan 侧用
`-s arch=armv8` 构建，或现场提供 aarch64 sysroot）。仓库未内置交叉工具链文件，需自备，
例如 `cmake/toolchains/linux-aarch64.cmake`（按现场路径填写）：

```cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
```

```bash
conan install . --output-folder xbuild --build=missing \
  -s arch=armv8 -s compiler.cppstd=17 -o "&:enable_testing=False" -o "&:enable_hdf5=False"

cmake -S . -B xbuild -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/xbuild/build/Release/generators/conan_toolchain.cmake" \
  -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
  -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
  -DONEQ_FLATC_EXECUTABLE=/usr/local/bin/flatc \     # 必须是 x86_64 原生 flatc
  -DONEQ_ENABLE_HDF5=OFF -DENABLE_TESTING=OFF -DENABLE_EXAMPLES=OFF
cmake --build xbuild -j"$(nproc)"
```

> 路线 C 未在本轮容器环境里实测（本机无 aarch64 交叉工具链 + aarch64 sysroot），
> 仅按配置项验证过语义；首次使用请预留调试时间。

---

## 6. 开关速查

| CMake 选项 | 默认 | 麒麟建议 | 作用 |
|---|---|---|---|
| `BUILD_SHARED_LIBS` | `OFF` | 保持 `OFF` | 静态库。改 `ON` 会因导出宏覆盖不足 + hidden 可见性产出不可用 `.so` |
| `ONEQ_ENABLE_HDF5` | `ON` | **`OFF`** | HighFive/HDF5。aarch64 无预编译包，要连 libaec/szip 源码构建，离线最易失败 |
| `ONEQ_ENABLE_FLIGHT_DYNAMIC` | `OFF` | 保持 `OFF` | 打开需要 JSBSim（且代码用 C++17 `std::clamp`） |
| `ONEQ_ENABLE_LTO` | `ON` | 保持 `ON` | Release 启用 LTO；已同时加 `-ffat-lto-objects`，静态库可被第三方消费 |
| `ONEQ_STRICT_FP` | `OFF` | **`ON`** | 加 `-ffp-contract=off`，使飞腾与 x86-64 的点积/判据结果位级一致 |
| `ONEQ_LINKER_STACK_OPTION` | `OFF` | 保持 `OFF` | `-Wl,-z,stack-size` 只对 Solaris ld/LLD 有效，GNU ld 会报错。深栈请用 `ulimit -s` |
| `PACKAGE_MANAGER` | `conan` | 保持 `conan` | `none` 当前不可用（§9 末行） |
| `ENABLE_TESTING` | `OFF` | release 可 `OFF` | 开则编译 gtest 单测 |
| `ENABLE_EXAMPLES` | `OFF` | 通常 `OFF` | 开则同时编 `examples/` |
| `ENABLE_INSTALL` | `OFF` | 交付时 `ON` | 生成 `1q::core` 包与 `1qTargets.cmake` |
| `ENABLE_WARNINGS` | `ON` | 保持 `ON` | 项目不开 `-Werror`，告警不阻断 |

对应的 Conan 侧选项（**必须与 CMake 侧一致**）：

```bash
-o "&:enable_testing=<同 ENABLE_TESTING>"
-o "&:enable_hdf5=<同 ONEQ_ENABLE_HDF5>"
-o "&:enable_flight_dynamic=<同 ONEQ_ENABLE_FLIGHT_DYNAMIC>"
```

环境变量：

| 变量 | 用途 |
|---|---|
| `ONEQ_CONAN_OFFLINE=1` | bootstrap 追加 `--no-remote`，避免每次 install 都等网络超时 |
| `ONEQ_CONAN_ARCH=armv8` | 交叉编译或 profile 探测不准时显式指定 Conan 架构 |
| `ONEQ_ENABLE_HDF5=True/False` | 透传给 `bootstrap_conan.sh`（与 CMake 选项语义相同） |
| `ONEQ_ENABLE_FLIGHT_DYNAMIC=True/False` | 同上 |
| `ONEQ_FLATC_EXECUTABLE=/path/flatc` | CMake 参数，交叉编译时指定宿主 flatc |
| `ONEQ_JSBSIM_PREBUILT_ROOT_DIR=/opt/jsbsim` | 仅在开启 flight_dynamic 且用预编译 JSBSim 时需要 |
| `ONEQ_CMAKE_ROOT=/opt/cmake` | `scripts/lib/1q_env.sh` 会把它加进 PATH（自装新版 CMake 时用） |

---

## 7. 编译与测试

```bash
cmake --build <build-dir> -j"$(nproc)"          # 或 --target 1q_lib / 1q_tests / 1q_unit_tests

# CMake >= 3.20
ctest --test-dir <build-dir> --output-on-failure -j4
# CMake 3.16~3.19 没有 --test-dir，进目录跑
(cd <build-dir> && ctest --output-on-failure -j4)
```

* 单测目标聚合：`1q_unit_tests`（单元）、`1q_tests`（全量）、`1q_tests_fast`（跳过慢测）。
* 测试临时目录默认 `/tmp`（`ONEQ_TEST_TEMP_DIR`），若 `/tmp` 挂载为 `noexec`，
  改造 `-DCMAKE_...` 或挂载方式，否则测试会被拒绝执行。
* 验收日志默认关闭。需要产物日志时打开对应开关（`ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG`、
  `ONEQ_ENABLE_RIR_ACCEPTANCE_LOG`、`ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG`、
  `ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG`、`ONEQ_ENABLE_PRECISION_EVALUATION_LOG`），
  并用 `-DONEQ_LOG_DIR=<可写目录>` 指定输出位置（默认是当前工作目录下的 `log/`）。

---

## 8. 安装与交付

```bash
cmake --install <build-dir> --prefix /opt/1q
```

产出 `lib/lib1q.a`、`include/1q/**`、`lib/cmake/1q/1qTargets.cmake` 与 `1qConfig.cmake`。
宿主工程侧：

```cmake
find_package(1q CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE 1q::core)
```

宿主侧需要能 `find_package` 到 **Eigen3 / Boost / nanoflann / flatbuffers / ZLIB / SQLite3 /
Threads**（`1qConfig.cmake` 会 `find_dependency` 它们）；离线交付时把这几个包一并交付，
否则会在宿主配置期报 `The link interface of target "1q::core" contains: SQLite::SQLite3 ...`。

---

## 9. 常见报错对照表

| 报错（原文关键片段） | 原因 | 处理 |
|---|---|---|
| `CMake Error: Could not read presets ... Unrecognized "version" field` | ① 旧版 `CMakePresets.json`（version 8 需 CMake ≥3.28）；② Conan 生成的 `CMakeUserPresets.json` 是 version 4，而 CMake 3.21/3.22 只支持 3 | ① 仓库已改为 `version 3`；② `rm CMakeUserPresets.json` 或 `--route manual`。**报错只给目录、不给文件名**，先查 user presets |
| `CMake Error: Unknown argument --preset` | CMake 3.16~3.18 无 preset 支持 | 走 **§4 路线 B** |
| `CMAKE_TOOLCHAIN_FILE ... does not exist` / `请先运行 scripts/bootstrap_conan.sh` | 没生成过 toolchain | 先跑 `bootstrap_conan.sh <preset>`；路线 B 则先手工 `conan install` |
| `错误: 不支持的 preset 'xxx'` | bootstrap 脚本不认识该 preset | 用 `linux-aarch64-debug` / `linux-aarch64-release`；或 `git pull` 拿最新脚本 |
| `cannot find -lHighFive` / `HighFive ... not found` | HighFive 被当裸库名链接；或开了 HDF5 但没装 | 用 `-DONEQ_ENABLE_HDF5=OFF`（推荐）；确需 HDF5 则确保 conan 装了 highfive |
| `setup_flatc ... 无法定位 flatc` / `flatc: command not found` | `flatbuffers` 被当 header-only 安装（无 flatc），或交叉编译未指定宿主 flatc | 确认 `conanfile.py` 里 `flatbuffers/*:header_only=False`；交叉编译加 `-DONEQ_FLATC_EXECUTABLE=` |
| `fatal error: airborne_radar/session/generated/*_generated.h: No such file` | 同上：flatc 没跑起来，生成头不存在 | 先修 flatc，再重新配置 |
| `ld: unrecognized -z option: stack-size` | 传了 `-Wl,-z,stack-size=`，GNU ld 不认 | 保持 `ONEQ_LINKER_STACK_OPTION=OFF`（默认）；深栈用 `ulimit -s unlimited` 或线程栈 API |
| `undefined reference to std::experimental::filesystem::v1::create_directories` | GCC < 9.1 需显式 `-lstdc++fs` | 构建系统已自动补；自建工程需自己加 `-lstdc++fs` |
| `fatal error: filesystem: No such file or directory` | GCC 7/8 没有 `<filesystem>` | 用 `#include <oneq/FsCompat.h>`（本仓库已改），不要直接 include |
| `undefined reference to 'pthread_mutex_lock'` | 用了 `std::mutex` 但没链 libpthread | 构建系统已加 `Threads::Threads`；自建工程需 `-pthread` 或 `find_package(Threads)` |
| `undefined reference to 'mallinfo2'` | glibc < 2.33 无该符号 | 已按 `__GLIBC_PREREQ(2,33)` 分流到 `mallinfo`；升级/替换测试文件即可 |
| `cc1plus: error: unrecognized command-line option '-Wuseless-cast'` | 老 GCC 不认识该警告旗标（GCC 对未知 `-W` 报 error） | 构建系统已用 `check_cxx_compiler_flag` 探测后再下发；自建工程照抄该写法 |
| `plugin needed to handle lto object` | 静态库带 LTO 对象，消费方工具链缺 LTO 插件 | 保持 `-ffat-lto-objects`（默认 ON）；或 `-DONEQ_ENABLE_LTO=OFF` 重新构建 |
| `ERROR: Package '...' not resolved ... no binary for armv8` | CCI 无 aarch64 预编译包 | `--build=missing`（需联网）或按 §2.2 预热缓存 + `--no-remote` |
| `ERROR: ... Connection to remote ... timed out`（离线） | `--build=missing` 仍会先访问 remote | `export ONEQ_CONAN_OFFLINE=1` |
| 配置期 `third_party/eigen ... does not exist` | 用了 `-DPACKAGE_MANAGER=none`，但仓库没有 third_party 源码 | 改回 `conan`（默认）。**`PACKAGE_MANAGER=none` 当前不可用**，见下 |

> **关于 `PACKAGE_MANAGER=none`**：`docs/practice/offline_build_no_conan.md` 描述的
> 「不装 Conan 直接用系统包」路线**目前跑不通**——`third_party/` 下没有源码，
> `VendorPackages.cmake` 也没有 spdlog/fmt/jsbsim/highfive/gtest 的 vendor 分支，
> 且 C++17 下 HighFive 缺省会硬 FATAL。离线场景请走 §2.2 的缓存迁移。

---

## 10. 数值一致性与已知限制

1. **`long double` 在两个 ISA 上不等价**：x86-64 尾数 64 位，aarch64（含飞腾）是 binary128、
   尾数 113 位且走软件浮点（`__addtf3`）。三处热路径
   （`RfLinkBudget.cpp`、`RfScene.cpp`、`DetectionCellResolver.cpp`）目前仍用 `long double`，
   因此**跨 ISA 的中间和可能有 1~2 ULP 差异**，极端情况下翻转阈值判据。
   本轮的处置是**不改变既有平台的数值结果**（仅提供 `ONEQ_STRICT_FP=ON` 消除 FMA 收缩差异）；
   是否统一改回 `double` 需业务侧确认（改动会改变 x86 现有黄金数据）。
2. **`ONEQ_STRICT_FP=ON` 是麒麟侧的默认值**（`linux-base` preset 内置），目的是让飞腾与
   x86-64 基线在 FMA 收缩这一项上对齐。手工配置（路线 B）请自己加 `-DONEQ_STRICT_FP=ON`。
3. **`std::filesystem` 在 GCC 7 上是 `experimental`**：本仓库通过 `include/oneq/FsCompat.h`
   统一别名 `oneq_fs`，并按 `__has_include` 自动选择；链接期 `stdc++fs` 由构建系统自动补。
4. **`flight_dynamic` 依赖 JSBSim 且用到 C++17 算法**：默认关闭，不建议在麒麟上打开；
   确需打开时确保 GCC ≥ 7 且 JSBSim 有 aarch64 构建。
5. **`ENABLE_TESTING=ON` 下性能测试读 `/proc/self/statm` 与 `mallinfo`**：容器/受限内核
   若隐藏 `/proc/self/statm`，该测试会失败，属环境限制而非程序问题。

---

## 11. 验收清单

构建完成后逐项确认：

- [ ] `cmake --build` 全程 **0 error**（有告警正常，项目不开 `-Werror`）。
- [ ] `ls build/<preset>/lib/lib1q.a` 存在（或 `local/lib1q.a`）。
- [ ] 静态库里含 `pthread_*` 未定义符号是**正常的**（实测 28 处），关键看**消费方**能链上：
      宿主工程用 `find_package(Threads)` + `Threads::Threads`（`1qConfig.cmake` 已代为 `find_dependency`）。
      自建工程若链接报 `undefined reference to 'pthread_mutex_lock'`，加 `-pthread` 即可。
- [ ] `ctest --output-on-failure` 全绿（开了 `ENABLE_TESTING` 才有）。
- [ ] 生成头齐全：`ls build/<preset>/generated/*/session/generated/*_generated.h` 应有 7 个模块的产物。
- [ ] 若交付：`cmake --install` 后 `find_package(1q)` 能在宿主空工程里通过配置。
- [ ] 数值抽查：用同一份输入分别在麒麟与 x86 上跑，确认关键指标差异在容差内（见 §10.1）。

---

## 12. 相关文件

| 文件 | 作用 |
|---|---|
| `conanfile.py` | 依赖清单与 `enable_testing` / `enable_hdf5` / `enable_flight_dynamic` 选项 |
| `cmake/project/ProjectOptions.cmake` | 所有 `ONEQ_*` / `ENABLE_*` 开关的默认值 |
| `cmake/compilers/CompilerClangGCC.cmake` | 告警旗标探测、LTO、`stdc++fs`、FP 收缩、链接选项 |
| `cmake/project/dependencies/ConanPackages.cmake` | `find_package` 的实际调用点 |
| `CMakePresets.json` | `linux-base` / `linux-aarch64-debug` / `linux-aarch64-release` |
| `scripts/bootstrap_conan.sh` | preset → conan install 参数映射（含离线/架构/业务开关） |
| `scripts/build_kylin.sh` | **一键构建**：体检 + 能力探针 + 选路线 + 依赖 + 编译 + 失败诊断 |
| `scripts/1q.sh` | `bootstrap` / `configure` / `build` / `test` / `install` 封装 |
| `docs/practice/kylin_v10_aarch64_build_compat.md` | 兼容性问题清单、修改说明与容器实测记录 |