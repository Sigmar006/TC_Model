# 无 Conan 环境下用 CMakeGUI 构建 1q（Windows / VS2013·VS2015）

> 适用场景：外部集成方**没有 Conan**，需要用 CMakeGUI 直接配置并构建 1q 静态库，
> 再把 `1q.lib` 交给 BJ 集成工程（`ModelSource`）链接。
> 本文所有取值均来自**当前交付树的实测 CMake 缓存**，不是推测：
> `C:\software\workspace\1qfosim\build\acceptance-v120-fosim\CMakeCache.txt`。

---

## 0. 结论（先看这三条）

1. **1q 支持完全不装 Conan 的构建**：设置 `PACKAGE_MANAGER=none`，依赖从 `third_party/` 源码消费
   （`cmake/project/ProjectDependencies.cmake:12-16`）。
2. **当前交付给宿主的 `1q.lib` 就是这样构建的**——缓存里明确是
   `PACKAGE_MANAGER:STRING=none` + `CMAKE_GENERATOR=Visual Studio 12 2013`，与 Conan 无关。
3. **`third_party/` 不在 Git 仓库里**（被 `.gitignore` 忽略），外部必须先把它准备好，
   否则 CMake 配置阶段直接 `FATAL_ERROR`（见 §2）。

---

## 1. 依赖来源：`third_party/`（vendored）而不是 Conan

### 1.1 两个 provider 的等价性

| provider | 触发条件 | 依赖来源 | 对下游的影响 |
|---|---|---|---|
| `conan` | `PACKAGE_MANAGER=conan`（默认）| `conanfile.py` + `conan_toolchain.cmake` | — |
| **`none`** | **`PACKAGE_MANAGER=none`** | **`third_party/` 源码** | **无**：产出的 imported target 名与输出变量完全一致（`Eigen3::Eigen`、`Boost::boost`、`ZLIB::ZLIB`、`SQLite::SQLite3`、`nanoflann::nanoflann`、`flatbuffers::flatbuffers`）|

即 `src/`、`tests/`、`install` 等消费点**不感知**用的是哪一个。

### 1.2 版本矩阵（与 `conanfile.py` 锁定一致）

| 库 | 通用（VS2015+）| **VS2013（MSVC<1900）专用** | 消费方式 |
|---|---|---|---|
| eigen | 3.4.0 | **3.3.9**（`eigen_vs2013`）| 仅头文件 |
| nanoflann | v1.3.2 | **v1.2.3**（`nanoflann_vs2013`）| 仅头文件 |
| boost | 1.85.0 | **1.68.0**（`boost_vs2013`）| 仅头文件（`boost/math`、`boost/pool`、`boost/multiprecision`）|
| flatbuffers | v1.12.0 | 同 | 头 + `add_subdirectory` 构建 `flatc` |
| zlib | v1.3.1 | 同 | `add_subdirectory` 编静态库（`gzopen` 运行时 API）|
| sqlite3 | 3.53.4 | 同 | amalgamation `sqlite3.c` 编静态库 |

**为什么要 VS2013 专用集**（`VendorPackages.cmake` L128 / L176 / L307 按 `MSVC_VERSION LESS 1900` 自动切换）：

- Eigen 3.4 在 MSVC 1800 上做矩阵乘积会 **ICE（C1001 内部编译器错误）**；
- boost 1.85 用了 C++14 `constexpr`，VS2013 直接语法报错；
- nanoflann 1.3.x 的表达式 SFINAE 在 MSVC 2013 上不通过。

### 1.3 `third_party/` 就绪检查清单

| 路径 | 就绪判据 |
|---|---|
| `third_party/eigen/Eigen/Core` | VS2015+ 需要 |
| `third_party/eigen_vs2013/Eigen/Core` | **VS2013 需要** |
| `third_party/boost/boost/math/special_functions/gamma.hpp` | VS2015+ 需要 |
| `third_party/boost_vs2013/boost/math/.../gamma.hpp` | **VS2013 需要** |
| `third_party/nanoflann/include/nanoflann.hpp` | VS2015+ |
| `third_party/nanoflann_vs2013/include/nanoflann.hpp` | **VS2013** |
| `third_party/flatbuffers/CMakeLists.txt` | 必需 |
| `third_party/zlib/CMakeLists.txt` | 必需 |
| `third_party/sqlite/sqlite3.c` | 必需（amalgamation 单文件）|

### 1.4 两种获取方式

**方式 A：跑拉取脚本**（需 `git`、`curl`、外网）

```bat
cd <包>\1q
scripts\win\fetch_third_party.bat
```

脚本会：`git clone --depth 1 --branch <tag>` 拉 eigen / nanoflann / flatbuffers / zlib（+ 三个 `_vs2013` 集），
并用 `curl` 下载 `boost_1_85_0.zip`、`boost_1_68_0.zip`（archives.boost.io）与
`sqlite-amalgamation-3530400.zip`（sqlite.org）。幂等：已存在的目录会跳过。

**方式 B：直接拷贝 vendored 目录**（无网络时）

| 打包粒度 | 体积（本机实测）| 说明 |
|---|---|---|
| 全量 `third_party/` | **892 MB** | 含 359 MB boost zip 缓存与各库 `.git` |
| 精简（只留 CMake 消费路径）| **≈303 MB** | 双工具链 |
| **仅 VS2013**（`eigen_vs2013/Eigen` + `boost_vs2013/boost` + `nanoflann_vs2013` + flatbuffers + zlib + sqlite）| **≈145 MB** | 只重建交付库时用这个 |
| 仅 VS2015+（`eigen/Eigen` + `boost/boost` + …）| **≈170 MB** | 开发/新工具链 |

> 体积大头是 boost 头（1.85 ≈ 142 MB / 1.68 ≈ 121 MB）。

---

## 2. CMakeGUI 逐步配置

### 2.1 界面字段

| CMakeGUI 字段 | 填写值（交付 v120）|
|---|---|
| **Where is the source code** | `<包>\1q` |
| **Where to build the binaries** | `<包>\build\acceptance-v120-fosim`（任意空目录）|
| **Generator** | **Visual Studio 12 2013** |
| **Platform**（下拉）| **x64** |
| **Optional toolset** | `v120`（VS2013 可留空）|

> 开发工具链：Generator 选 **Visual Studio 15 2017**，Platform `x64`，toolset `v141`。
> **两棵树不能混用同一份 `1q.lib`**：`_MSC_VER` 不一致会链接失败（`FAILIFMISMATCH`）。

### 2.2 Cache 变量（点 `Add Entry` 添加，Type 见括号）

| 变量（Type）| 值 | 为什么必须 |
|---|---|---|
| `PACKAGE_MANAGER`（STRING）| **none** | 指定走 `third_party/`，不找 Conan。拼错会 `FATAL_ERROR: Unsupported PACKAGE_MANAGER`（只接受 `conan` / `none`）|
| `CMAKE_CXX_STANDARD`（STRING）| **11** | 全库按 C++11 编写（VS2015 交付下限），设 17 也能编但非交付口径 |
| `BUILD_SHARED_LIBS`（BOOL）| **OFF** | 产出静态 `1q.lib` 供宿主链接；默认 ON 会出 DLL |
| `ENABLE_PCH`（BOOL）| **ON** | 预编译头，显著缩短编译时间 |
| `ENABLE_INSTALL`（BOOL）| **ON** | 生成安装规则（按需）|
| `ENABLE_TESTING`（BOOL）| **OFF** | 需要单测改 ON；gtest 版本见 §5 |
| `ENABLE_EXAMPLES`（BOOL）| **OFF** | |
| `ENABLE_UNITY_BUILD`（BOOL）| **OFF** | 交付构建未用 |
| `ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG`（BOOL）| **ON** | ★ 关掉则**红外验收日志整体为空** |
| `ONEQ_ENABLE_RIR_ACCEPTANCE_LOG`（BOOL）| **ON** | ★ 雷达验收日志 |
| `ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG`（BOOL）| **ON** | ★ 第 15/16 项融合行 |
| `ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG`（BOOL）| **ON** | ★ 第 20–25 项推演行 |
| `ONEQ_ENABLE_PRECISION_EVALUATION_LOG`（BOOL）| **ON** | ★ 第 26/27 项精度行 |
| `ONEQ_ENABLE_FILE_LOG`（BOOL）| **OFF** | 库内部调试日志，验收不走它 |
| `ONEQ_ENABLE_FLIGHT_DYNAMIC`（BOOL）| **OFF** | 机动模块依赖 JSBSim，Windows 不构建 |
| `ONEQ_MSVC_RUNTIME_LIBRARY`（STRING）| **留空** | 空 = 默认动态 CRT（`/MD`、Debug 为 `/MDd`），与宿主工程一致；填 `MultiThreaded` 会与宿主链接冲突 |

### 2.3 命令行等价（不想点 GUI）

```bat
:: VS2013（与上表完全等价，preset 已内置这些 cache 值）
cmake --preset VisualStudio.12.0-amd64-none
cmake --build --preset VisualStudio.12.0-amd64-none-release

:: VS2015
cmake --preset VisualStudio.14.0-amd64-none
cmake --build --preset VisualStudio.14.0-amd64-none-release
```

两个 preset 的实际内容（`CMakePresets.json`）：

```text
windows-base-none         : PACKAGE_MANAGER=none, ENABLE_PCH=ON
VisualStudio.12.0-amd64-none : inherits windows-base-none
                               CMAKE_CXX_STANDARD=11, BUILD_SHARED_LIBS=OFF,
                               ENABLE_TESTING=OFF, ENABLE_INSTALL=ON
                               生成器 = Visual Studio 12 2013
binaryDir 约定            : ${sourceDir}/build/${presetName}
```

### 2.4 构建与产物

- GUI 里：`Configure` → `Generate` → `Open Project` → 选 **Release | x64** → 生成 `1q_lib`
- 命令行：`cmake --build --preset <...>-release`
- **产物**：`<build>\Release\lib\1q.lib`（交付树实测 147.9 MB / 约 3.1 万符号）

---

## 3. 把 `1q.lib` 接进集成工程

```bat
rem ① 放库（目录需自建）
copy <build>\Release\lib\1q.lib  ModelSource\lib\x64\Release\1q.lib

rem ② 同步公共头（宿主是按 include\1q 的副本编译的，不同步会 ABI 错位甚至崩溃）
robocopy <1q>\include\1q  ModelSource\include\1q  /MIR

rem ③ 全量重建（脚本会顺手删掉旧 obj，避免增量残留）
ModelSource\rebuild_with_1q.cmd
```

> **②不可省**：踩过一次现场崩溃——只换了 `1q.lib` 没同步头，宿主按旧布局构造
> `PrecisionEvaluationConfig`，库按新布局读栈，崩在 `FusionConfig` 拷贝构造的 `std::string::_Copy`。
> 现在宿主有 ABI 自检，日志会打印 `[init][ABI] precision config size ok: 1496 bytes`。

---

## 4. 配置/构建失败对照表

| 报错 | 原因 | 处理 |
|---|---|---|
| `Unsupported PACKAGE_MANAGER: 'xxx'` | 值不是 `conan`/`none`（大小写敏感）| 改成 `none` |
| `PACKAGE_MANAGER=none 要求 third_party/ 已就绪，但缺失：…\third_party/eigen` | `third_party` 未准备 | §1.4 方式 A/B |
| `PACKAGE_MANAGER=none 缺失 boost 头：…\third_party/boost/boost` | boost zip 没解压到位（modular superproject 无法 clone 出预合并头）| 重跑 `fetch_third_party.bat` 的 boost 步骤，或直接拷 `boost/boost` |
| `PACKAGE_MANAGER=none 缺失 sqlite3 amalgamation：…\third_party/sqlite/sqlite3.c` | 只拷了 GitHub 镜像（`src/*.c` 拆分源）| 用 sqlite.org 发布包 |
| VS2013 报 **C1001（ICE）** 且栈在 Eigen | 用了 `eigen`（3.4）而非 `eigen_vs2013`（3.3.9）| 确认编译器确为 MSVC<1900（`MSVC_VERSION` 判定），必要时删 `CMakeCache.txt` 重配 |
| VS2013 报 `constexpr` 相关语法错 | 用了 boost 1.85 | 换 `boost_vs2013`（1.68）|
| 链接期 `LNK2038 / mismatch detected for '_MSC_VER'` | 1q.lib 与宿主工具链不一致 | 用同一工具链重建（交付 v120）|
| 链接期 CRT 冲突（`_ITERATOR_DEBUG_LEVEL` / `/MT` vs `/MD`）| `ONEQ_MSVC_RUNTIME_LIBRARY` 被改成静态 CRT | 留空；宿主工程也用默认 `/MD` |
| 构建成功但日志缺整块验收项 | `ONEQ_ENABLE_*_ACCEPTANCE_LOG` 关着 | 见 §2.2 中带 ★ 的五项 |

---

## 5. 单元测试（可选）

- `ENABLE_TESTING=ON` 需要的 gtest **必须用 1.8.1**：仓库 `conanfile.py` 固定的 1.12.x 在 MSVC 19.00（VS2015）
  上因 `FlatTupleBase`/`C2535` 编译失败；1.10 同样失败，**1.8.1 可过**。
- 无 Conan 时 gtest 不在 `third_party/` 里，需自行提供或跳过测试目标。

---

## 6. 交付树实测缓存对照（ground truth）

以下为本机交付构建 `build\acceptance-v120-fosim\CMakeCache.txt` 的原始取值，
可直接用它核对你的 CMakeGUI 是否配成同一口径：

```text
CMAKE_GENERATOR            = Visual Studio 12 2013
CMAKE_GENERATOR_PLATFORM   = x64
CMAKE_GENERATOR_TOOLSET    = v120
CMAKE_CXX_STANDARD         = 11
CMAKE_CONFIGURATION_TYPES  = Debug;Release;MinSizeRel;RelWithDebInfo
PACKAGE_MANAGER            = none
BUILD_SHARED_LIBS          = OFF
ENABLE_INSTALL             = ON
ENABLE_PCH                 = ON
ENABLE_TESTING             = OFF
ENABLE_EXAMPLES            = OFF
ENABLE_UNITY_BUILD         = OFF
USE_CCACHE                 = OFF            # 交付机未装 ccache
ONEQ_MSVC_RUNTIME_LIBRARY  = （空，默认 /MD）
ONEQ_ENABLE_FILE_LOG              = OFF
ONEQ_ENABLE_FLIGHT_DYNAMIC        = OFF
ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG   = ON
ONEQ_ENABLE_RIR_ACCEPTANCE_LOG    = ON
ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG = ON
ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG = ON
ONEQ_ENABLE_PRECISION_EVALUATION_LOG = ON
CMAKE_CXX_FLAGS            = /DWIN32 /D_WINDOWS /EHsc
CMAKE_CXX_FLAGS_RELEASE    = /O2 /Ob2 /DNDEBUG
```

---

## 7. 构建后自查清单

| 检查 | 期望 |
|---|---|
| `1q.lib` 是否生成 | `<build>\Release\lib\1q.lib`，约 148 MB |
| 是否静态库 | `BUILD_SHARED_LIBS=OFF`，无 `1q.dll` |
| 关键模块是否在库里 | `dumpbin /symbols 1q.lib \| findstr /i "sbirs_sensor remote_identification_radar target_inference precision_evaluation fusion"` 均有命中 |
| 验收日志开关生效 | 宿主启动日志出现 `[init][ABI] precision config size ok: …`；运行后 `log\SatelliteTarget.log`、`log\RadarOutput.log` 有行 |
| 工具链一致 | 宿主与库同为 v120（交付）或 v141（开发），不混用 |

---

## 8. 相关文件索引

| 内容 | 文件 |
|---|---|
| 依赖分派 | `cmake/project/ProjectDependencies.cmake` |
| vendored provider | `cmake/project/dependencies/VendorPackages.cmake` |
| Conan provider | `cmake/project/dependencies/ConanPackages.cmake` |
| 选项与 `PACKAGE_MANAGER` | `cmake/project/ProjectOptions.cmake` |
| 运行库/MSVC 策略 | `cmake/project/ProjectTargets.cmake`（`ONEQ_MSVC_RUNTIME_LIBRARY`）|
| 拉取脚本 | `scripts/win/fetch_third_party.bat` |
| preset | `CMakePresets.json`（`windows-base-none`、`VisualStudio.12.0-amd64-none`、`VisualStudio.14.0-amd64-none`）|
| 宿主侧同步与重建 | `ModelSource/rebuild_with_1q.cmd`、`build_x64_v120.cmd`、`build_x64_v141.cmd` |
| 当前生效的红外/雷达参数 | `docs/review/ir_radar_parameters_2026-11-04.md` |
