# 银河麒麟 V10（国防版 / 飞腾·华芯 aarch64）编译兼容性分析与整改报告

> 目标：让本仓库源码在**国产化 aarch64 环境**（银河麒麟 V10 国防版 + 飞腾/华芯 CPU，离线、无外网）上
> 从「配置即失败」变成「可配置、可编译、可安装」。
> 本文记录**问题清单 → 根因 → 证据（文件:行）→ 处理状态**，以及已经落地的代码修改与整改后的构建步骤。
> 未修改、只给建议的条目单独列在第 5 节，请业务方确认后再动。

---

## 0. 靶环境假设与判定依据

| 项 | 假设值 | 依据 |
|---|---|---|
| CPU | 飞腾 FT-2000+/64、S2500 等（华芯即飞腾，**aarch64/armv8-a**） | 用户明确「华芯 CPU」 |
| OS | 银河麒麟 V10 国防版（Kylin V10 SP1/SP2 同源） | 用户明确 |
| 编译器 | **GCC 7.3.0**（麒麟 V10 出厂 gcc/g++ 基线），可升到 8.3/9.3 | 麒麟 V10 默认 toolchain |
| C 库 | **glibc 2.28**（pthread 尚未并入 libc，需 glibc ≥ 2.34） | 麒麟 V10 基线 |
| 二进制工具 | GNU ld **2.30**、binutils 无 `gcc-ar` LTO 插件默认路径问题 | 同上 |
| CMake | **3.16.3**（出厂），可升到 3.22 | 麒麟 V10 仓库 |
| 网络 | 完全离线（国防版内网，不允许访问 conancenter） | 用户场景 |
| 构建开关 | 默认配置：`ONEQ_ENABLE_FLIGHT_DYNAMIC=OFF`、`ENABLE_TESTING/EXAMPLES=OFF`；另有 `ENABLE_TESTING=ON` 的验收构建 | 仓库默认值 |

> **一句话结论**：主库源码本身（`src/common` 等 14 个模块）在 GCC 7.3 下**没有 C++ 语言层阻塞**；
> 真正让人「一条命令都跑不起来」的是**构建系统与依赖引导**（preset 版本、preset 平台、bootstrap 白名单、
> HighFive 裸名链接、flatbuffers header_only 缺 flatc、ZLIB 绕过 conan），
> 其次是 **glibc 版本相关 API**（`mallinfo2`）、**`<filesystem>` 可用性判定**与**浮点收缩/`long double` 跨 ISA 一致性**。

---

## 1. 结论摘要

### P0（阻断：配置或编译直接失败，必须改）

| # | 问题 | 证据 | 状态 |
|---|---|---|---|
| P0-1 | `CMakePresets.json` 写 `"version": 8`，需 **CMake ≥ 3.28**；麒麟 3.16/3.22 直接 `Unrecognized "version" field` | `CMakePresets.json:2` | **已修**：降到 `version 3`（CMake 3.21+），`cmakeMinimumRequired` 3.25 → 3.21 |
| P0-2 | 5 个 configure preset 全部挂在 `macos-base`(condition Darwin) / `windows-base`(condition Windows) 之下，**aarch64 Linux 无任何 preset 可用** | `CMakePresets.json` | **已修**：新增 `linux-base` / `linux-aarch64-debug` / `linux-aarch64-release` + 对应 build/test preset |
| P0-3 | `scripts/bootstrap_conan.sh` 的 `case "${PRESET}"` 只认 6 个 preset，其它 `exit 2`；而 `ProjectSetup.cmake:9-17` 硬性要求 toolchain 文件已存在 → **「不 bootstrap 无法 configure，bootstrap 又不认 Linux preset」死锁** | `scripts/bootstrap_conan.sh:69`、`cmake/project/ProjectSetup.cmake:9-17` | **已修**：新增两个 Linux 分支；Linux 侧补 `os/arch/compiler` 与业务选项；支持 `--no-remote` |
| P0-4 | **HighFive 以裸名链接**：`PRIVATE HighFive` → 链接器解析成 `-lHighFive` → `cannot find -lHighFive`（CMakeDeps 提供的是 `HighFive::HighFive`）。且 Linux 默认 C++17 非 Windows 时 HDF5 分支**强制开启**，必然命中 | `src/CMakeLists.txt:70`、`src/sar/CMakeLists.txt:29`、`cmake/project/dependencies/ConanPackages.cmake:38-45` | **已修**：改为 `${ONEQ_HIGHFIVE_TARGET}`（自动探测 `HighFive::HighFive` / 裸名）+ 新增 `ONEQ_ENABLE_HDF5` 开关 |
| P0-5 | `flatbuffers/*:header_only=True` → conan 包内**没有 flatc**；而 7 个模块在构建期用 flatc 生成头（仓库内 `*_generated.h` 数量为 0）→ 配置/构建期 FATAL | `conanfile.py:49`、`cmake/project/codegen/FlatBuffers.cmake`、7 处 `oneq_add_flatbuffers_schemas` | **已修**：`header_only=False`；`setup_flatc` 新增 `-DONEQ_FLATC_EXECUTABLE=`（交叉编译必需） |
| P0-6 | `find_package(ZLIB REQUIRED)` 不带 `CONFIG` → 走 CMake 自带 MODULE 查找，**绕过 conan**：麒麟缺 `zlib1g-dev` 即失败，或与 conan 版本混链两份 libz | `ConanPackages.cmake:29`、`tests/cmake/TestSupport.cmake:19` | **已修**：先 `CONFIG QUIET`，未命中再回退 MODULE 并告警 |
| P0-7 | **`mallinfo2()` 需 glibc ≥ 2.33**，麒麟 glibc 2.28 无此符号 → 该测试 TU 编译失败（`ENABLE_TESTING=ON` 时非 Windows 一律注册） | `tests/performance/cross_domain/rf_interference_performance_test.cpp:100` | **已修**：按 `__GLIBC_PREREQ` 分流到 `mallinfo2` / `struct mallinfo`（含负值钳位）/ `0U` |
| P0-8 | `examples/app/fs_compat.h` 按 **语言标准**（`__cplusplus < 201703L`）猜 `<filesystem>`，而 GCC 7.3 在 `-std=c++17` 下 `__cplusplus` 已是 201703L 却**没有**该头 → `fatal error: filesystem: No such file or directory` | `examples/app/fs_compat.h:12-22` | **已修**：改用 `__has_include` 探测；抽出内部头 `include/oneq/FsCompat.h` 统一供 examples/tests 引用 |
| P0-9 | 全仓 **`stdc++fs` 零处**：GCC 7/8 的 `<experimental/filesystem>` / `<filesystem>` 必须显式链接 `libstdc++fs`，GCC 9.1 才免 | 全树 grep `stdc++fs` = 0 | **已修**：`CompilerClangGCC.cmake` 对 GNU < 9.1 自动为所有非 OBJECT/INTERFACE 目标补 `stdc++fs` |
| P0-10 | GNU 专属警告旗标（`-Wduplicated-branches`、`-Wuseless-cast` 等 6 个）**无版本门**；GCC 对不认识的 `-W` 是 **error**（`cc1plus: error: unrecognized command-line option`），不是 warning | `cmake/compilers/CompilerClangGCC.cmake:51-59` | **已修**：改为 `check_cxx_compiler_flag` 逐条探测后再下发 |

### P1（严重：能配出来但编/链/跑不对，必须改）

| # | 问题 | 证据 | 状态 |
|---|---|---|---|
| P1-1 | `-Wl,-z,stack-size=N` 是 **Solaris/LLD** 关键字，GNU ld 2.30 的 `-z` 表里没有 → `ld: unrecognized -z option: stack-size`；且 Linux 主线程栈由 `RLIMIT_STACK` 决定，改 PT_GNU_STACK 无效。原默认档位 `RECOMMENDED` 使**每个**可执行都吃到这个链接项 | `CompilerClangGCC.cmake:89-95`、`ProjectOptions.cmake:47` | **已修**：新增 `ONEQ_LINKER_STACK_OPTION`（默认 OFF），且仅非 Linux/非 APPLE 生效 |
| P1-2 | Release/MinSizeRel 无条件 `-flto`，但**无 `-ffat-lto-objects`、未切 `gcc-ar/gcc-ranlib`**：交付静态库给第三方（用非 LTO 编译器链接）会报 `plugin needed to handle lto object` | `CompilerClangGCC.cmake:100-102` | **已修**：LTO 收进 `ONEQ_ENABLE_LTO`（默认 ON 保持行为），GNU 下追加 `-ffat-lto-objects`，Clang 不下发该旗标 |
| P1-3 | `-Wl,--gc-sections` 与 `-ffunction-sections/-fdata-sections` **不配对**（全树无后者），gc-sections 实际空转 | `CompilerClangGCC.cmake:110-112` | **已修**：Release/MinSizeRel 补 `-ffunction-sections -fdata-sections` |
| P1-4 | **`BUILD_SHARED_LIBS` 默认 ON**，同时 `-fvisibility=hidden` + `CXX_VISIBILITY_PRESET hidden`，而公共头只有 `include/1q/api.hpp` 一个 `ONEQ_EXPORT` → 默认产出**不可用的 .so**（下游大面积 `undefined reference`） | `ProjectOptions.cmake:12`、`ProjectTargets.cmake:66-70`、`include/1q/api.hpp:19-22` | **已修**：默认改 OFF（静态库，符合国产化交付习惯）；显式开 ON 时打 WARNING 说明风险 |
| P1-5 | 库内 `std::mutex` **37 处**，但全树**没有 `find_package(Threads)`/`-pthread`**；glibc 2.28 上 pthread 尚未并入 libc，静态链接或 `-static-libstdc++` 会 `undefined reference to 'pthread_mutex_lock'` | `src/common/logging/*`、`src/airborne_radar/signal/tracking/SynchronizedTrackPool.*`、`src/sbirs_sensor/pipeline/SbirsAcceptanceRecords.cpp` | **已修**：`ProjectDependencies.cmake` 无条件 `find_package(Threads)` 并入 `ONEQ_LINK_DEPENDENCIES`；组件加 `Threads::Threads`；安装配置补 `find_dependency(Threads)` |
| P1-6 | 导出的 `1qTargets.cmake` **缺 `find_dependency(SQLite3)`**：静态库的 PRIVATE 依赖以 `$<LINK_ONLY:SQLite::SQLite3>` 进入链接接口，下游 `find_package(1q)` 报 `The link interface of target "1q::core" contains: SQLite::SQLite3 but the target was not found` | `cmake/project/ProjectInstall.cmake` | **已修** |
| P1-7 | `-ffp-contract=fast` 是 GCC 默认：aarch64 **恒有 FMA**，x86-64 SSE2 基线没有 → 同一点积差 1 ULP，可能翻转 `snr >= threshold` 之类的整数判据 | 代表点积 `src/common/electromagnetics/RfLinkBudget.cpp:62,102,277-279` | **已修**（默认不变，麒麟 preset 内开启）：新增 `ONEQ_STRICT_FP`，`linux-aarch64-*` preset 置 ON 下发 `-ffp-contract=off` |
| P1-8 | **`long double` 在 aarch64 是 16 字节 binary128 软浮点**（macOS arm64 上却等同 double，CI 恰好测不出）；三处累加用它，最内层 40 次/链路 | `src/common/electromagnetics/RfLinkBudget.cpp:382,387`、`RfScene.cpp:683,688`、`src/common/radar/DetectionCellResolver.cpp:160,166,203-205,209,211` | **未改**（见 §5.1，需业务确认数值基线） |
| P1-9 | 非 Windows 的 `IsValidUtf8` **恒返回 true**，`NormalizeExternalText` 恒原样返回 —— 与公共头注释「绝不返回坏字节」矛盾，GBK/乱码文件名会直通 FlatBuffers 与验收日志 | `src/common/text/TextEncoding.cpp:29-32,118-120`、`include/1q/foundation/TextEncoding.h:33,44` | **已修**：完整 RFC 3629 校验 + 非 ASCII 降级替换 `?` |
| P1-10 | 非 MSVC 的 `ToAbsolutePath` 直接 `return path`（保留相对路径），cwd 不可写时六个验收文件**静默消失**；`ProcessStartStamp` 用**不可重入**的 `std::localtime` 且不判空 | `src/common/logging/AcceptanceFileLog.cpp:283-285,323` | **已修**：`getcwd` 绝对化 + `localtime_r` |
| P1-11 | `examples/app/runner.cpp:138` 用**会抛异常**的 `create_directories(path)` 重载，违反仓库「禁止 C++ 异常」铁律，目录不可写时直接 `std::terminate` | `examples/app/runner.cpp:138` | **未改**（见 §5.3） |
| P1-12 | `PACKAGE_MANAGER=none`（`docs/practice/offline_build_no_conan.md` 主推的离线路线）**当前不可用**：`third_party/` 只有 `lib/`、`win/`，无 eigen/nanoflann/boost 源码 → 第 25 行 FATAL；且该文件无 spdlog/highfive/gtest 的 vendor 分支 | `cmake/project/dependencies/VendorPackages.cmake:20-29,372-378` | **未改**（见 §5.2） |

### P2（建议 / 稳健性，不阻断编译）

| # | 内容 | 状态 |
|---|---|---|
| P2-1 | `examples/CMakeLists.txt`、`docs/practice/offline_build_no_conan.md` **不是 UTF-8**（GBK），在中文 locale 的构建机/grep/CI 上读出来是乱码；`scripts/check_source_bom.py` 未覆盖 | 未改 |
| P2-2 | `ReplayTrace.cpp` 写 double 走 ostream 默认 6 位有效数字，读用 `strtod`（受 `LC_NUMERIC` 影响），且多处流未开 `std::ios::binary` → 跨平台 trace 不可逐字节互校验 | 未改 |
| P2-3 | `SbirsPipeline.cpp:859-865` **每个仿真周期** `setenv("ONEQ_SBIRS_WIDE_AREA", ...)`：glibc `setenv` 可能 realloc `environ`，与并发 `getenv` 构成数据竞争，且每周期一次堆分配 | 未改 |
| P2-4 | `examples/logger/acceptance_paths.h:14` 缺 `#include <cstdlib>`（`setenv` 声明） | 未改 |
| P2-5 | `src/sar/imaging/SarScanSarFocusing.cpp:206` 裸 `long`（Windows 4B / Linux aarch64 8B），建议 `std::int64_t` | 未改 |
| P2-6 | `src/sar/output/ImageFormatter.cpp:29-40` 用 float↔uint32 union 双关（UB），建议 `memcpy` | 未改 |
| P2-7 | `docs/practice/offline_build_no_conan.md` 主推 `-DCMAKE_CXX_STANDARD=11`，但 `flight_dynamic/guidance/Maneuver.cpp` 有 17 处 `std::clamp`（C++17）、tests/examples 有 62 处 `std::make_unique`、12 处结构化绑定 → **「C++11 交付口径」只在默认 OFF 开关下成立** | 未改（见 §5.4） |

### 已核实「无风险」清单（缩小后续复核面）

以下在国产化 aarch64 上**常见但本仓库确实不存在**，已逐一核查，不必再翻：

- **无 SIMD/x86 内联汇编**：`immintrin.h`、`__m128/__m256`、`_mm_*`、`__asm__`、`__builtin_cpu_supports`、`__cpuid`、`__rdtsc`、`_Interlocked*` 命中 **0**。
- **无架构宏分支**：`__x86_64__`/`_M_X64`/`__aarch64__` 命中 **0**。
- **无字节序/对齐隐患**：`__attribute__((packed))`、`#pragma pack`、`alignas`、位域、`htonl/ntohl`、裸结构体流读写、`volatile`、`static_assert(sizeof...)` 命中 **0**（早期"20 个文件命中 packed"是 `aligned_power` 子串误报）。
- **无 `-Werror`**：全树仅 `CompilerMSVC.cmake:73` 的 `/WX-` 与 ClangFormat 的 `--Werror`（格式检查，不进构建）。
- **Eigen 在 aarch64 无需任何 `EIGEN_*` 宏干预**：仅 6 处 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`，无 `Eigen::aligned_allocator`、无 `-msse*`。
- **`char` 符号性**：所有字节级代码均已 `static_cast<unsigned char>`（ctype 调用 14 处全对）；`-Wold-style-cast` 的 199 处命中全是 `(void)expr`，GCC 对 cast-to-void 不告警 → 零噪声。
- **`if constexpr` 4 处均带 C++11 回退**（`EkfFilter.h:114-122`、`KalmanPredictor.h:91-99,111-124`、`IKalmanUpdater.h:90`），GCC 7 下走回退分支。
- **`std::optional/variant/string_view/charconv/bit/byte`、`[[likely]]`、`__int128`、`asm(`、两阶段查找缺陷** 命中 **0**。
- **printf 格式**：`%I64d/%Iu` 等 MSVC 私有格式 **0**，`%zu` 52 / `%llu` 23 / `%lld` 1，Linux 全部合法。
- **`-Wcast-align` 预期温和**：aarch64 ABI 允许非对齐访问，一级 `-Wcast-align` 主要在 `STRICT_ALIGNMENT=1` 目标告警；src 内 71 处 `reinterpret_cast` 基本是 `char*/void*` → `uint8_t*`。

---

## 2. 已实施修改清单（逐文件）

### 2.1 构建系统 / 预设

| 文件 | 修改 |
|---|---|
| `CMakePresets.json` | `version` 8 → **3**；`cmakeMinimumRequired` 3.25 → **3.21**；新增 hidden `linux-base`（`condition` 判定 Linux、Ninja、`PACKAGE_MANAGER=conan`、`ONEQ_ENABLE_HDF5=OFF`、`ONEQ_STRICT_FP=ON`）；新增 `linux-aarch64-debug` / `linux-aarch64-release`（Debug/Release、`ENABLE_TESTING=ON`、`BUILD_SHARED_LIBS=OFF`、单配置 toolchain 路径 `build/<preset>/build/<Config>/generators/conan_toolchain.cmake`）；build/testPresets 各补两条 |
| `cmake/project/ProjectOptions.cmake` | `BUILD_SHARED_LIBS` 默认 ON → **OFF**（开 ON 时 WARNING）；新增 `ONEQ_ENABLE_HDF5`(ON)、`ONEQ_ENABLE_LTO`(ON)、`ONEQ_STRICT_FP`(OFF)、`ONEQ_LINKER_STACK_OPTION`(OFF) |
| `cmake/compilers/CompilerClangGCC.cmake` | GNU 6 个警告旗标改 `check_cxx_compiler_flag` 探测；Release/MinSizeRel 补 `-ffunction-sections -fdata-sections`；`-flto` 收进 `ONEQ_ENABLE_LTO` + GNU 才加 `-ffat-lto-objects`；`ONEQ_STRICT_FP` → `-ffp-contract=off`；GNU < 9.1 自动链 `stdc++fs`（跳过 OBJECT/INTERFACE 库）；`-Wl,-z,stack-size=*` 收进 `ONEQ_LINKER_STACK_OPTION` 且排除 Linux |
| `cmake/project/dependencies/ConanPackages.cmake` | ZLIB 先 `CONFIG QUIET` 再回退 MODULE；Boost 缺 `Boost::boost` 时用 `Boost::headers` 造别名；HDF5 分支加 `AND ONEQ_ENABLE_HDF5`，并解析 `ONEQ_HIGHFIVE_TARGET` |
| `cmake/project/dependencies/JsbsimProvider.cmake` | 策略三改 `find_package(jsbsim CONFIG QUIET)` + 目标探测（`jsbsim::jsbsim` / `JSBSim` / `JSBSim::JSBSim`）再建别名；失败时给出 conan 选项与预编译/源码两条出路 |
| `cmake/project/dependencies/VendorPackages.cmake` | 未改（见 §5.2） |
| `cmake/project/codegen/FlatBuffers.cmake` | `setup_flatc` 支持 `-DONEQ_FLATC_EXECUTABLE=`（交叉编译必需），FATAL 文案补病因 |
| `cmake/project/ProjectDependencies.cmake` | 无条件 `find_package(Threads REQUIRED)` 并加入 `ONEQ_LINK_DEPENDENCIES`；透传 `ONEQ_HIGHFIVE_TARGET` |
| `cmake/project/ProjectInstall.cmake` | 补 `find_dependency(SQLite3 REQUIRED CONFIG)`（conan 分支内）与 `find_dependency(Threads REQUIRED)`（provider 无关） |
| `cmake/project/ProjectTargets.cmake` | 组件加 `Threads::Threads`（PRIVATE） |
| `src/CMakeLists.txt` / `src/sar/CMakeLists.txt` | `PRIVATE HighFive` → `PRIVATE ${ONEQ_HIGHFIVE_TARGET}` |
| `conanfile.py` | 新增选项 `enable_flight_dynamic`、`enable_hdf5`（默认 False/True）；`requirements()` 中 jsbsim 受 `enable_flight_dynamic` 门控、`highfive/2.10.0` 受 `enable_hdf5` 门控（原先**即使不用也强制拉 hdf5 + jsbsim**，aarch64 上要源码编 hdf5）；`flatbuffers/*:header_only` True → **False** |
| `scripts/bootstrap_conan.sh` | 新增 `linux-aarch64-debug/release` 两分支（原先 `exit 2`）；Linux 侧传 `enable_flight_dynamic`/`enable_hdf5`/可选 `-s arch=`；`ONEQ_CONAN_OFFLINE=1` → `--no-remote`（离线时 `--build=missing` 仍会先访问 remote 然后超时） |
| `scripts/lib/1q_env.sh` | 不再无条件注入 Windows 盘符候选与 `scripts/win/bin`（原 `:38-54,94` 会把 `/d/environment/CMake/bin` 塞进 PATH） |

### 2.2 源码 / 测试 / 示例

| 文件 | 修改 |
|---|---|
| **新增** `include/oneq/FsCompat.h` | `__has_include(<filesystem>)` → `std`，否则 `__has_include(<experimental/filesystem>)` → `std::experimental`，都没有则 `#error`；提供 `namespace oneq_fs`。**刻意放在 `include/oneq/`（内部头、不安装）**，避免触碰公共头 C++11 禁令 |
| `examples/app/fs_compat.h` | 重写为 `#include <oneq/FsCompat.h>` + `namespace app_fs = oneq_fs;`（4 处使用点无需改） |
| `src/flight_dynamic/adapter/JsbsimAdapter.cpp` | 去 `<filesystem>`；改用 `LogDirectory.h` 的 `EnsureParentDirectory` + `SGPath`；新增 `TempJsbsimOutputDirectory()`（`TEMP`/`TMPDIR`，缺省 `/tmp`、`C:/Windows/Temp`）与 `PathExists()`（`access`/`_access`）；**去掉 `try/catch`**（违反禁异常铁律） |
| `tests/unit/examples/scene_data_test.cpp`、`tests/consumer/batch_validation/{ar,eos,esr,sar,sbirs}_batch_validation.cpp` | `<filesystem>` → `<oneq/FsCompat.h>`，`std::filesystem::` → `oneq_fs::` |
| `tests/performance/cross_domain/rf_interference_performance_test.cpp` | `mallinfo2()` 按 glibc 版本分流（≥2.33 用 `mallinfo2`；否则 `struct mallinfo` + 负值钳位；再否则 0） |
| `src/common/text/TextEncoding.cpp` | 非 Windows `IsValidUtf8` 实现完整 RFC 3629 校验（拒绝非法首字节/截断续字节/超长编码/代理区/`>0x10FFFF`）；`NormalizeExternalText` 非 Windows 降级为替换 `?` |
| `src/common/logging/AcceptanceFileLog.cpp` | 非 MSVC 加 `<unistd.h>`；`ToAbsolutePath` 用 `getcwd` 绝对化；`ProcessStartStamp` 改 `localtime_r` 并判空 |

> 全部修改均**保持 Windows/macOS 现有行为不变**为第一约束：所有平台分支都保留了原路径，
> 新增行为只在「非 Windows / GNU / 特定 glibc」条件下生效。

---

## 3. 麒麟 aarch64 上的构建步骤（整改后）

### 3.1 前置（离线机器）

```bash
# 系统工具链（麒麟 V10 自带即可，建议确认版本）
gcc --version        # 期望 >= 7.3
g++ --version
cmake --version      # 3.16.3 可用；但 --preset 需 3.19+，见 3.2
ninja --version      # 若无 ninja，可换 -G "Unix Makefiles"
python3 --version    # scripts/lib/1q_preset_query.py 需要
```

离线依赖（二选一）：

- **路线 A（推荐）**：在联网 x86/同架构机器上 `conan create`/`conan install` 预取全部依赖到本地缓存，
  打包 `~/.conan2`（或 `CONAN_HOME`）与 recipe 一起拷到麒麟机器，然后 `ONEQ_CONAN_OFFLINE=1` 构建。
- **路线 B**：把 `third_party/`（eigen、nanoflann、boost 源码、flatbuffers、zlib、spdlog）补齐后
  用 `-DPACKAGE_MANAGER=none`。**注意路线 B 当前不可用**，见 §5.2。

### 3.2 用 preset（CMake ≥ 3.21）

```bash
cd <1q>
export ONEQ_CONAN_OFFLINE=1        # 离线：禁止访问 remote
export ONEQ_CONAN_ARCH=armv8       # 可选，显式指定 conan 架构
bash scripts/bootstrap_conan.sh linux-aarch64-release
cmake --preset linux-aarch64-release
cmake --build build/linux-aarch64-release --target 1q_lib -j"$(nproc)"
ctest --preset linux-aarch64-release          # 需要 ENABLE_TESTING
```

Debug 版把 `release` 换成 `debug`（额外开 `ENABLE_EXAMPLES=ON`）。

### 3.3 不用 preset（CMake 3.16 出厂版本）

`--preset` 自 CMake 3.19 才有，麒麟自带 3.16 必须显式传参：

```bash
cd <1q>
bash scripts/bootstrap_conan.sh linux-aarch64-release      # 仍用它生成 toolchain
cmake -S . -B build/kylin -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/build/linux-aarch64-release/build/Release/generators/conan_toolchain.cmake" \
  -DCMAKE_CXX_STANDARD=17 \
  -DBUILD_SHARED_LIBS=OFF \
  -DPACKAGE_MANAGER=conan \
  -DONEQ_ENABLE_HDF5=OFF \
  -DONEQ_STRICT_FP=ON \
  -DENABLE_TESTING=ON -DENABLE_EXAMPLES=OFF -DENABLE_INSTALL=ON
cmake --build build/kylin --target 1q_lib -j"$(nproc)"
```

> `-DCMAKE_CXX_STANDARD=17` 是硬要求：源码用了 `if constexpr`/`std::clamp` 等 C++17 特性
> （`ProjectSetup.cmake` 默认值本来就是 17）。**不要**照旧文档传 11。

### 3.4 交叉编译（x86 主机 → aarch64 目标）

```bash
cmake -S . -B build/x \
  -DCMAKE_TOOLCHAIN_FILE=<aarch64-toolchain>.cmake \
  -DONEQ_FLATC_EXECUTABLE=/usr/bin/flatc     # 宿主机 flatc，交叉编译必需
  -DPACKAGE_MANAGER=conan -DONEQ_ENABLE_HDF5=OFF
```

---

## 4. 验收自检（本轮已做）

| 检查 | 结果 |
|---|---|
| `CMakePresets.json` JSON 合法性 | 通过（`json.load`） |
| `cmake --list-presets`（原 version 8 直接报错） | 修复后可用；`linux-aarch64-debug/release` 已出现在 build/test preset 列表 |
| 全量 CMake 解析冒烟（`-DPACKAGE_MANAGER=none` + 假 toolchain） | ProjectOptions / CCache / ClangTidy / ClangFormat / FlatBuffers / CompilerClangGCC / Threads 全部解析通过，只在 `VendorPackages.cmake:25`（缺 `third_party/eigen`，既有问题）停下 |
| `<filesystem>` 引用收敛 | 全树仅 `include/oneq/FsCompat.h` 直接包含；`JsbsimAdapter.cpp` 已无 filesystem 依赖 |
| `stdc++fs` 覆盖 | `CompilerClangGCC.cmake` 对 GNU < 9.1 自动补，无需各模块手写 |

> **未能在本机完成的验证**：本机为 Windows/MSVC，无法真实执行 aarch64 编译，也无法跑 GCC 7.3。
> 因此「GCC 7.3 是否还有其它告警/错误」需要到麒麟机器上实跑一次 `ENABLE_TESTING=ON` 的全量构建确认。

---

## 5. 未修改项与后续建议（需业务确认）

### 5.1 `long double` → `double`（P1-8，**建议改，但会动数值基线**）

- 事实：`RfLinkBudget.cpp:382,387`、`RfScene.cpp:683,688,690`、`DetectionCellResolver.cpp:160,166,200-205,211,213`
  用 `long double` 做干扰功率累加。x86-64 上 `long double` 是 80 位扩展精度（尾数 64 位），
  aarch64 上是 128 位 binary128（尾数 113 位）且**无硬件指令**，走 `__addtf3` 软浮点。
- 影响：同一输入的累加结果尾数不同 → `argmax` 选中的链路可能翻转；而三处累加前都已 `std::sort`
  （`RfLinkBudget.cpp:379`、`RfScene.cpp:680`、`DetectionCellResolver.cpp:141`），求和顺序本就确定，
  **用 `long double` 并没有换来"顺序无关"**，只换来"更慢 + 跨 ISA 不一致"。
- 建议：统一改 `double`。风险点是 `tests/unit/airborne_radar/ar_detection_cell_resolver_test.cpp:240`
  这类 `EXPECT_DOUBLE_EQ(forward.interference_power_w, reverse.interference_power_w)` 与既有黄金数据需要重录。
  **这属于数值行为变更，本轮不动，请确认后再改。**

### 5.2 `PACKAGE_MANAGER=none` 不可用（P1-12）

- `VendorPackages.cmake:20-29` 要求 `third_party/{eigen,nanoflann,flatbuffers,zlib}` 就绪，而仓库里
  `third_party/` 只有 `lib/`、`win/` 两个二进制目录；`:372-378` 在 C++17 下对 HighFive 直接 FATAL；
  该文件也没有 spdlog / fmt / jsbsim / gtest 的 vendor 分支。
- 建议：若要继续支持"零 conan 离线构建"，需要
  1) 补 `scripts/fetch_third_party.sh` 的 Linux 分支并维护 `third_party/` 清单；
  2) 给 `VendorPackages.cmake` 加 spdlog/fmt/highfive/gtest 分支，或在这些依赖缺失时降级为
     `ONEQ_ENABLE_FILE_LOG=OFF` 的纯文件日志；
  3) 同步修正 `docs/practice/offline_build_no_conan.md`（该文目前主推这条不可用的路线）。

### 5.3 `examples/app/runner.cpp:138` 抛异常重载（P1-11）

`app_fs::create_directories(output_dir)` 用的是会抛 `filesystem_error` 的重载，违反「禁止 C++ 异常」；
同仓库正例在同文件邻居里：`examples/scenes/sbirs_triple_sat_fix_messages/main.cpp:618` 与
`sbirs_inframe_dual_target/main.cpp:572` 都用 `std::error_code` 版本。
本轮未改是因为 examples 只在 `-DENABLE_EXAMPLES=ON` 时编译（默认 OFF），
但麒麟 Debug preset 会打开它 —— **建议合并前改掉**。

### 5.4 "C++11 交付口径"与源码实际标准错位（P2-7）

`docs/practice/offline_build_no_conan.md:17,34,90` 推荐 `-DCMAKE_CXX_STANDARD=11`，
但 `flight_dynamic/guidance/Maneuver.cpp` 有 17 处 `std::clamp`（C++17），tests/examples 有
62 处 `std::make_unique`、12 处结构化绑定。默认配置（flight_dynamic OFF、tests/examples OFF）下能过，
一旦打开就必然失败。建议：
1) `src/flight_dynamic/CMakeLists.txt` 增加 `if(PROJECT_CXX_STANDARD LESS 17) FATAL_ERROR`；
2) 把公共头 C++11 禁令（`tests/compatibility/public_api`）与"实现可用 C++17"这两件事在文档里写清楚。

### 5.5 其它 P2

见 §1 表格 P2-1 ~ P2-6：源文件编码（GBK）统一、`ReplayTrace` 数值序列化精度与 locale、
`SbirsPipeline` 每周期 `setenv`、`ImageFormatter` union 双关、`SarScanSarFocusing` 裸 `long`、
第三方 include 目录加 `SYSTEM`（可减少 Eigen 头里的告警噪声）。

---

## 6. 一页速查：这次到底改了什么

1. **让 preset 能被麒麟上的 CMake 读出来**：`version 8 → 3`，并补 `linux-base` / `linux-aarch64-*`。
2. **让 bootstrap 认 Linux preset**：补两个 case 分支 + `--no-remote` + 业务选项透传。
3. **修必然链接失败的两处**：HighFive 裸名 → imported target；ZLIB 走 conan CONFIG。
4. **修必然配置失败的两处**：flatbuffers `header_only` 关掉（要 flatc）；GNU 警告旗标先探测再下发。
5. **修 GNU ld / 静态库交付隐患**：`-z stack-size` 默认关、LTO 加 `-ffat-lto-objects`、
   `--gc-sections` 补配对、GNU<9.1 自动 `stdc++fs`、`Threads::Threads`、`find_dependency(SQLite3)`。
6. **修 glibc 2.28 与 GCC 7.3 的 API 依赖**：`mallinfo2` 分流、`<filesystem>` 用 `__has_include` 探测。
7. **修 Linux 上"静默失效"的行为**：UTF-8 校验、日志绝对路径、`localtime_r`。
8. **默认改成静态库**（`BUILD_SHARED_LIBS=OFF`），避免 hidden 可见性 + 单导出宏产出不可用 .so。

---

## 7. 实测验证：没有麒麟虚机时怎么验

本文档第 1~6 节的结论最初来自静态审计。为了不只停留在「读代码推断」，本轮又在**容器里做了等价环境实测**。
必须写清楚：**这不是银河麒麟 V10 真机**，而是「GCC 版本档 + glibc 版本档」都能卡住同类问题的逼近环境。

### 7.1 为什么用容器，以及选了哪两个镜像

先确认本机确实没有麒麟虚机：Hyper-V `Get-VM` 返回空、未安装 VMware（无 `vmrun`）、未安装 VirtualBox、
PATH 里没有 QEMU；WSL 只有 `Ubuntu 24.04`（x86_64、GCC 13、glibc 2.39，版本太高，测不出问题）。
本机可用的是 Docker Desktop，且 `github.com:443` 不可达（`objects.githubusercontent.com` 通），
因此走「Ubuntu 18.04 基线 + 阿里/清华镜像源」的路子：

| 镜像 | 架构 | 编译器 | glibc | CMake | 逼近点 |
|---|---|---|---|---|---|
| `kylin-amd64-gcc7` | x86_64 原生 | GCC 7.5.0 | 2.27 | 3.22.6 | GCC 7.x 档、glibc < 2.33（无 `mallinfo2`）、GCC < 8（无 `<filesystem>`） |
| `kylin-arm64-gcc7` | aarch64（qemu-user 模拟） | GCC 7.5.0 (Linaro) | 2.27 | —（只做源码级验证） | 上述全部 + **aarch64 ABI**（`long double` = binary128、参数/返回约定、对齐） |

麒麟 V10 SP 的实际组合是 GCC 7.3 + glibc 2.28。两个镜像的编译器是 7.5 而非 7.3、glibc 是 2.27 而非 2.28，
差异方向一致（都在 `<8` / `<2.33` 这一侧），所以**能暴露**本文列出的这些问题，但不能替代真机验收。

### 7.2 实测结果

平台事实（同一份探针源码在两镜像里各编一次）：

| 探针项 | x86_64 / GCC 7.5 | aarch64 / GCC 7.5 | 结论 |
|---|---|---|---|
| `sizeof(long double)` | 16 | 16 | **两边一样，`sizeof` 查不出差异** |
| `LDBL_MANT_DIG` | **64** | **113** | 尾数位 64 vs 113：aarch64 是 binary128 软浮点，见 §1 P1-4 |
| `mallinfo2` | ABSENT | ABSENT | 证明 `rf_interference_performance_test.cpp` 的 `__GLIBC_PREREQ(2,33)` 回退分支是**必需**的 |
| `<filesystem>` | ABSENT | ABSENT | 证明 `FsCompat.h` 必须走 `<experimental/filesystem>` 分支 |
| `-ffp-contract=off` | 接受 | 接受 | 交叉 ISA 数值一致性开关可用 |
| 12 个 GNU 警告旗标 | 12/12 接受 | 12/12 接受 | GCC 7.5 已支持全部；`check_cxx_compiler_flag` 版本门对更老的 7.x 是保护 |
| `libstdc++fs` | 需要（`-lstdc++fs`） | 需要（`-lstdc++fs`） | 印证 §1 P0-5：GCC 9.1 前必须显式链接 |
| `FsCompat` 端到端 | `create_directories`+`exists` 运行 OK（`ec=Success`） | 同（`ec=Success`） | 头文件分派与实现都正确 |

**全树语法扫描**（`src/**/*.cpp` 逐个 `-fsyntax-only`，`-std=c++17`，只需 Eigen/Boost）：

| 环境 | TU 总数 | 通过 | 缺第三方头跳过 | **编译失败** |
|---|---|---|---|---|
| x86_64 / GCC 7.5 | 298 | **282** | 16 | **0** |
| aarch64 / GCC 7.5 | 298 | **282** | 16 | **0** |

被跳过（缺 `flatbuffers` / `nanoflann` / `FGFDMExec.h` / 生成的 `*_generated.h`）的 16 个 TU 都是第三方依赖明确未安装的，
不是代码问题。**没有任何一个 TU 因为 C++ 语言特性、标准库缺失、平台 API 或 ABI 原因编译失败**——
这与 §1 的判断一致：主库的 C++ 源码在 GCC 7.x 下没有语言层阻塞。

**CMake 模块级验证**（`CompilerClangGCC.cmake` + `ProjectOptions.cmake` 真实 include，并真实编译链接）：

```
PASS: Linux 上不发射 -Wl,-z,stack-size（GNU ld 不认该关键字）
PASS: GCC7.5.0 的 STATIC 库自动链接 stdc++fs
PASS: OBJECT 库不加 stdc++fs（无链接步骤）
PASS: Release 启用 -ffunction-sections（与 --gc-sections 配对）
PASS: Release/MinSizeRel 启用 --gc-sections
PASS: ONEQ_STRICT_FP=ON 时下发 -ffp-contract=off
PASS: Release 启用 -flto
PASS: GCC 下追加 -ffat-lto-objects
probe: fs=1 obj=41          # Release + LTO 下真实链接并运行成功
```

这条最关键：它不只是「变量值对了」，而是**真的在 GCC 7.5 + glibc 2.27 上把一个用到
`std::experimental::filesystem` 的静态库链接成了可执行文件并跑通**。若没有 `stdc++fs` 自动链接，
此处必然是 `undefined reference to std::experimental::filesystem::v1::create_directories(...)`。

**静态库与线程符号**（282 个目标文件归档成 8.0 MB 静态库后做 `nm -u`）：

```
pthread_undef_count=28      # pthread_mutex_lock / unlock / key_create ...
```

28 处未定义的 `pthread_*` 就是 §P0-10 里「`std::mutex` 用了却没有任何 `find_package(Threads)`」
的实证：消费方必须拿到 `-pthread`（`Threads::Threads`）。

### 7.3 这次实测**没有**覆盖的部分（请在真机上补）

1. **完整 conan install 与全库链接**：容器内无法访问 Conan 官方 remote（且 CCI 上 flatbuffers/nanoflann/spdlog
   在 aarch64 需从源码构建，需联网与较长构建时间）。因此 16 个依赖第三方头的 TU 只验证了「缺头即跳过」。
2. **glibc 2.28 与 GCC 7.3 的精确行为**：本环境是 2.27/7.5。若麒麟实际是 glibc 2.28，`ReplayTrace` 的
   locale 行为、`setenv` 并发语义可能有细微差别。
3. **飞腾（Phytium）硬件特性**：`long double` 软浮点（`__addtf3`）的性能损耗、FMA 收缩对判据的影响，
   在 qemu 上只有功能语义、没有真实性能与位级差异的完整分布。
4. **`ENABLE_TESTING=ON` 的全量测试**：容器里没有 gtest/flatbuffers，未跑 ctest。

### 7.4 复现命令

```bash
# 1) 构建逼近镜像（仓库外目录，勿提交）
cd TC_Model/build/_kylintest
docker build -f Dockerfile.amd64 -t kylin-amd64-gcc7 .
docker run --privileged --rm tonistiigi/binfmt --install arm64     # 首次需要
docker build --platform linux/arm64 -f Dockerfile.arm64 -t kylin-arm64-gcc7 .

# 2) 平台事实 + 本轮改动文件的编译/运行检查（x86 与 arm64 各跑一次）
docker run --rm -v "$PWD/../..:/work" kylin-amd64-gcc7 bash /work/build/_kylintest/verify.sh
docker run --rm --platform linux/arm64 -v "$PWD/../..:/work" kylin-arm64-gcc7 bash /work/build/_kylintest/verify.sh

# 3) 全树语法扫描
docker run --rm -v "$PWD/../..:/work" kylin-amd64-gcc7 bash /work/build/_kylintest/verify_wide.sh

# 4) CMake 模块级断言（真实 configure + build + run）
docker run --rm -v "$PWD/../..:/work" kylin-amd64-gcc7 bash -c \
  "cmake -S /work/build/_kylintest/cmakeprobe -B /tmp/cm -DCMAKE_BUILD_TYPE=Release -DONEQ_STRICT_FP=ON && cmake --build /tmp/cm && /tmp/cm/probe_exe"

# 5) 部分库归档与 pthread 符号统计
docker run --rm -v "$PWD/../..:/work" kylin-amd64-gcc7 bash /work/build/_kylintest/build_partial.sh
```

> `build/_kylintest/` 是本轮的临时验证工程（Dockerfile、verify 脚本、CMake 探针），**不进入版本控制**，
> 也不作为交付物；需要复现时按上面命令重建即可。麒麟真机上仍应走 §4 的 preset 路线做一次完整
> `conan install → configure → build → ctest`。