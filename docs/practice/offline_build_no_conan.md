# 国产化 / 离线 / 无 Conan 构建指南（Windows 与 Linux）

> 目标：在**不安装 Conan、构建期不联网**的环境（国产化内网 / 麒麟 / UOS / Windows 内网机）中，
> 用 CMake 配置并编译出**静态库**（Windows 下为 `1q.lib`，Linux 下为 `lib1q.a`），
> 并且**验收日志开关全部打开**（否则宿主跑起来日志整段为空）。
> 构建目录固定为源码目录下的 **`local/`**（即 `<1q>/local`），不污染 `build/`。

---

## 0. 一句话命令（复制即用）

### Windows（VS2015/2017/2019/2022，任选其一）
```bat
cd <1q 根目录>
cmake -S . -B local -G "Visual Studio 15 2017" -A x64 -T v141 ^
  -DPACKAGE_MANAGER=none ^
  -DCMAKE_CXX_STANDARD=11 ^
  -DBUILD_SHARED_LIBS=OFF ^
  -DENABLE_TESTING=OFF -DENABLE_EXAMPLES=OFF -DENABLE_INSTALL=ON -DENABLE_PCH=ON ^
  -DONEQ_ENABLE_OPIR_ACCEPTANCE_LOG=ON ^
  -DONEQ_ENABLE_RIR_ACCEPTANCE_LOG=ON ^
  -DONEQ_ENABLE_FUSION_ACCEPTANCE_LOG=ON ^
  -DONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG=ON ^
  -DONEQ_ENABLE_PRECISION_EVALUATION_LOG=ON
cmake --build local --config Release --target 1q_lib
```

### Linux（GCC / Clang + Ninja 或 Make）
```bash
cd <1q 根目录>
cmake -S . -B local -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DPACKAGE_MANAGER=none \
  -DCMAKE_CXX_STANDARD=11 \
  -DBUILD_SHARED_LIBS=OFF \
  -DENABLE_TESTING=OFF -DENABLE_EXAMPLES=OFF -DENABLE_INSTALL=ON -DENABLE_PCH=ON \
  -DONEQ_ENABLE_OPIR_ACCEPTANCE_LOG=ON \
  -DONEQ_ENABLE_RIR_ACCEPTANCE_LOG=ON \
  -DONEQ_ENABLE_FUSION_ACCEPTANCE_LOG=ON \
  -DONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG=ON \
  -DONEQ_ENABLE_PRECISION_EVALUATION_LOG=ON
cmake --build local --target 1q_lib -j
```

### Linux 实测补充（WSL2 / Ubuntu 24.04 / gcc 13.3 / CMake 3.28 / Ninja 1.11，2026-11-07）

按上面的 Linux 命令**原样执行会在 `flatc` 处失败**：第三方 `third_party/flatbuffers` 的
`src/reflection.cpp` 在 gcc 13 下触发 `-Wstringop-overflow` 类告警，而工程开了 `-Werror`，
于是"告警即错误"。这是**工具链版本差**（老 flatbuffers + 新 gcc），不是 1q 代码问题。

实测可用的做法（把第三方告警降级、不影响本库代码的告警）：

```bash
cmake -S . -B local-wsl -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DPACKAGE_MANAGER=none -DCMAKE_CXX_STANDARD=11 \
  -DBUILD_SHARED_LIBS=OFF -DENABLE_TESTING=OFF -DENABLE_EXAMPLES=OFF \
  -DENABLE_INSTALL=ON -DENABLE_PCH=ON \
  -DCMAKE_CXX_FLAGS="-Wno-error -Wno-stringop-overflow -Wno-array-bounds -Wno-restrict" \
  -DONEQ_ENABLE_OPIR_ACCEPTANCE_LOG=ON -DONEQ_ENABLE_RIR_ACCEPTANCE_LOG=ON \
  -DONEQ_ENABLE_FUSION_ACCEPTANCE_LOG=ON -DONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG=ON \
  -DONEQ_ENABLE_PRECISION_EVALUATION_LOG=ON
cmake --build local-wsl --target 1q_lib -j 8
```

实测结果：**367 个目标全部编译通过**，产物 `local-wsl/lib/lib1q.a`（48 MB，含
`MtiMtdAcceptanceBank.cpp.o` / `RirAcceptanceRecords.cpp.o` / `TextEncoding.cpp.o` /
`AcceptanceFileLog.cpp.o`）。构建目录约 1.7 GB，验证完可整目录删除（不要提交）。

另外：本节脚本若带 UTF-8 BOM，`bash script.sh` 会报 `#!/usr/bin/env: No such file or directory`
（BOM 落在 shebang 前），但脚本仍会继续执行——想要干净的话，`.sh` 文件不要带 BOM。
（C/C++ 源文件必须带 BOM，`.sh` 相反。）

**产物位置（实测）**
- Windows 多配置生成器：`local/Release/lib/1q.lib` ← **这就是"类似 .lib 的静态库"**
- Linux 单配置（Ninja/Make）：`local/lib/lib1q.a`
- 找不到时：`cmake --build local --target 1q_lib --verbose | findstr /i "\.lib"`（Windows）
  或 `find local -name 'lib1q.a' -o -name '1q.lib'`（Linux）

---

## 1. 前置条件

| 项 | 要求 |
|---|---|
| CMake | ≥ 3.16（实测 4.3.3 可用）|
| 编译器 | Windows：VS2015(v140)/2017(v141)/2019/2022；**VS2013(v120) 需用 `Visual Studio 12 2013` 生成器**（CMake≥4 已移除该生成器，须用 CMake 3.x）<br>Linux：GCC ≥ 5（C++11）/ Clang ≥ 3.6 |
| **`third_party/`** | **必须存在**（见 §2），这是"无 Conan"的唯一前提 |
| 网络 | 构建期**不需要**；`third_party/` 就绪后全程离线 |
| Conan | **不需要**（`PACKAGE_MANAGER=none`）|

---

## 2. 准备 `third_party/`（二选一）

### 方式 A：有一次性联网能力
```bat
:: Windows
scripts\win\fetch_third_party.bat
```
脚本按锁定版本拉取：
`eigen 3.4.0`、`nanoflann v1.3.2`、`flatbuffers v1.12.0`、`zlib v1.3.1`、`sqlite 3.53.4`（amalgamation）、
`boost 1.85.0`，以及 VS2013 降级集 `eigen_vs2013 3.3.9`、`boost_vs2013 1.68.0`、`nanoflann_vs2013 v1.2.3`。

### 方式 B：完全离线（国产化内网推荐）
把已就绪的 `third_party/` 目录整体拷到目标机源码根下。**只需保留以下路径**（精简后约 145–170 MB）：

```text
third_party/
├── eigen/                     # VS2015+ 用（Eigen/ 子目录即可）
├── eigen_vs2013/              # VS2013 用（3.3.9）
├── boost/boost/               # VS2015+ 用（只需 boost/ 头目录）
├── boost_vs2013/boost/        # VS2013 用（1.68.0）
├── nanoflann/include/         # VS2015+
├── nanoflann_vs2013/include/  # VS2013
├── flatbuffers/               # 需 CMakeLists.txt + include/ + src/（要构 flatc）
├── zlib/                      # 需 CMakeLists.txt 全套
└── sqlite/                    # 需 sqlite3.c + sqlite3.h（amalgamation）
```

**就绪自检（9 条全 True 才能配置成功）**
```bat
for %d in (eigen\Eigen\Core eigen_vs2013\Eigen\Core boost\boost\math\special_functions\gamma.hpp boost_vs2013\boost\math\special_functions\gamma.hpp nanoflann\include\nanoflann.hpp nanoflann_vs2013\include\nanoflann.hpp flatbuffers\CMakeLists.txt zlib\CMakeLists.txt sqlite\sqlite3.c) do @if exist third_party\%d (echo OK %d) else (echo MISSING %d)
```

> **Linux 注意**：`fetch_third_party.bat` 是 Windows 脚本；Linux 上按上表**手工 `git clone` 同版本 tag**
> 并解压 boost 源码包即可，`VendorPackages.cmake` 本身跨平台（不含 `add_subdirectory(boost)`，
> boost 只作头文件使用）。

---

## 3. 关键选项解释（为什么必须这么配）

| 选项 | 值 | 作用 / 不这么配的后果 |
|---|---|---|
| `PACKAGE_MANAGER` | **`none`** | 走 `third_party/`；写错会 `FATAL_ERROR: Unsupported PACKAGE_MANAGER` |
| `CMAKE_CXX_STANDARD` | **`11`** | 全库按 C++11 编写（VS2015 交付下限）；17 能编但非交付口径 |
| `BUILD_SHARED_LIBS` | **`OFF`** | 产出**静态库** `1q.lib` / `lib1q.a`；ON 会出 DLL/so |
| `ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG` | **ON** | 红外验收行（`SatelliteTarget.log`）；关了红外日志整段空 |
| `ONEQ_ENABLE_RIR_ACCEPTANCE_LOG` | **ON** | 雷达验收行（`RadarOutput.log`）|
| `ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG` | **ON** | 第 15/16 项融合行 |
| `ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG` | **ON** | 第 20–25 项推演行（落点/发射点/关机点/落点误差）|
| `ONEQ_ENABLE_PRECISION_EVALUATION_LOG` | **ON** | 第 26/27 项精度行 |
| `ENABLE_TESTING` | **OFF** | gtest 不在 `third_party/`，开了会找不到包直接配置失败 |
| `ENABLE_PCH` | ON | 预编译头，显著加快 |
| `ENABLE_INSTALL` | ON | 生成安装规则（按需）|
| `ONEQ_MSVC_RUNTIME_LIBRARY` | 留空 | 空 = 默认 `/MD`；宿主工程也是默认，改静态 CRT 会链接冲突 |

**验收日志路径**（运行期，宿主工作目录下）：`log/SatelliteTarget.log`、`log/RadarOutput.log`、
`log/IRExternalInterface.log`、`log/IRInternalInterface.log`、`log/RadarExternalInterface.log`、`log/RadarInternalInterface.log`。
库侧另有 `ONEQ_LOG_DIR` 只用于**调试日志**，与验收日志无关。

---

## 4. 构建与验证

### 4.1 构建
```bat
:: Windows
cmake --build local --config Release --target 1q_lib
:: 只要更多产物（examples 等）可去掉 --target
cmake --build local --config Release
```
```bash
# Linux
cmake --build local --target 1q_lib -j"$(nproc)"
```

### 4.2 验证产物是静态库且含全部模块
```bat
dir local\Release\lib\1q.lib
dumpbin /symbols local\Release\lib\1q.lib | findstr /i "sbirs_sensor remote_identification_radar target_inference precision_evaluation fusion"
```
```bash
ls -lh local/lib/lib1q.a
nm -C local/lib/lib1q.a | grep -E "sbirs_sensor|remote_identification_radar|target_inference" | head
```

### 4.3 验证验收日志开关真的生效
最直接的办法：把库接进宿主跑一轮，检查日志文件里**五类行都在**：

```text
log/SatelliteTarget.log        含 [探测角度计算功能测试] [卫星自身定位误差功能测试] [落点预测功能测试] …
log/RadarOutput.log            含 [恒虚警检测功能测试] [目标信号增益功能测试] …
log/IRExternalInterface.log    含 [目标与红外环境场景输入接口测试]
```
缺少整块 → 对应 `ONEQ_ENABLE_*` 没打开。

---

## 5. 接进宿主（Windows 交付链路）

```bat
rem ① 库放到宿主要求的位置
copy local\Release\lib\1q.lib  <ModelSource>\lib\x64\Release\1q.lib

rem ② 同步公共头（宿主按 include\1q 副本编译，不同步会 ABI 错位甚至崩溃）
robocopy <1q>\include\1q  <ModelSource>\include\1q  /MIR

rem ③ 全量重建宿主（脚本会先删旧 obj）
<ModelSource>\rebuild_with_1q.cmd
```

> ②不可省：曾因只换 `1q.lib` 不同步头，宿主按旧结构布局构造配置对象，崩在 `std::string::_Copy`。
> 现在宿主启动会打印 ABI 自检：`[init][ABI] precision config size ok: 1496 bytes`。

---

## 6. 常见错误对照

| 报错 | 原因 | 处理 |
|---|---|---|
| `Unsupported PACKAGE_MANAGER: 'xxx'` | 值不是 `conan`/`none`（大小写敏感）| 改成 `none` |
| `PACKAGE_MANAGER=none 要求 third_party/ 已就绪，但缺失：…/eigen` | `third_party` 未准备 | §2 |
| `PACKAGE_MANAGER=none 缺失 boost 头：…/boost/boost` | boost 源码包没解压到位 | 重跑拉取脚本的 boost 步骤 |
| `PACKAGE_MANAGER=none 缺失 sqlite3 amalgamation` | 只拷了 GitHub 镜像（无 `sqlite3.c` 单文件）| 用 sqlite.org 发布包 |
| `Could not find Eigen3Config.cmake` | 用了 `conan` 模式但没装 conan | 改 `PACKAGE_MANAGER=none` |
| `Could not create named generator Visual Studio 12 2013` | CMake ≥ 4 已移除 VS2013 生成器 | 用 CMake 3.x，或改用 VS2015+ |
| VS2013 `C1001`（栈在 Eigen）| 用了 `eigen` 而非 `eigen_vs2013` | 确认编译器确为 MSVC<1900；删缓存重配 |
| `LNK2038 _MSC_VER mismatch` | 库与宿主工具链不一致 | 同一工具链重建（交付 v120 就用 v120）|
| 构建成功但日志缺整块 | `ONEQ_ENABLE_*` 关着 | §3 表中五个开关全 ON |

---

## 7. 与 `local/` 构建目录相关的清理

```bat
rmdir /s /q local          :: Windows
```
```bash
rm -rf local               # Linux
```

> `local/` 未加入 `.gitignore` 前请勿提交；若需要，可自行在 `.gitignore` 追加一行 `local/`。

---

## 8. 已验证记录（2026-11-05）

| 检查 | 结果 |
|---|---|
| 全新空目录、`PACKAGE_MANAGER=none`、无 Conan | `Configuring done` / `Generating done`，退出码 0 |
| 实际编译 `1q_lib` | 产出 `1q.lib`（VS2017/v141，33.8 MB），退出码 0 |
| 交付树（VS2013/v120）| `CMakeCache.txt` 中 `PACKAGE_MANAGER:STRING=none`，交付 `1q.lib` 147.9 MB |
| `third_party/` 依赖 | 9/9 就绪 |

---

## 9. Linux 构建（CMake 3.16 + g++ 7，已实测）

> 本节命令已用 **Ubuntu 18.04 + g++ 7.5.0 + CMake 3.16.9** 容器实测通过：
> `configure` 成功、`cmake --build --target 1q_lib` 成功，产出 **`lib/lib1q.a`（46 MB）**。

### 9.1 依赖

| 项 | 要求 | 说明 |
|---|---|---|
| 编译器 | **GCC ≥ 5**（实测 7.5 通过；13.3 亦通过）| 需支持 C++11 |
| CMake | **≥ 3.16**（实测 3.16.9 / 4.3.3 通过）| 老版本 3.10 未验证 |
| make / ninja | 任一 | 示例用 Unix Makefiles |
| `third_party/` | 同 §2 | 构建期不联网 |

```bash
sudo apt-get update && sudo apt-get install -y g++ make cmake
# 若仓库 cmake 版本低于 3.16，用官方二进制：
# curl -sSL -o cmake.tgz https://github.com/Kitware/CMake/releases/download/v3.16.9/cmake-3.16.9-Linux-x86_64.tar.gz
# sudo mkdir -p /opt && sudo tar xzf cmake.tgz -C /opt && export PATH=/opt/cmake-3.16.9-Linux-x86_64/bin:$PATH
```

### 9.2 配置（产物落到 `build/`，不污染源码树）

```bash
cd <1q 根目录>
cmake -S . -B build -G "Unix Makefiles" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=11 \
  -DPACKAGE_MANAGER=none \
  -DBUILD_SHARED_LIBS=OFF \
  -DENABLE_TESTING=OFF -DENABLE_EXAMPLES=OFF -DENABLE_INSTALL=OFF -DENABLE_PCH=OFF \
  -DONEQ_ENABLE_OPIR_ACCEPTANCE_LOG=ON \
  -DONEQ_ENABLE_RIR_ACCEPTANCE_LOG=ON \
  -DONEQ_ENABLE_FUSION_ACCEPTANCE_LOG=ON \
  -DONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG=ON \
  -DONEQ_ENABLE_PRECISION_EVALUATION_LOG=ON
```

> **注意**：`ENABLE_PCH=OFF`。g++ 7 下 PCH 非必需，关掉可避免 `.gch` 与 `-std=c++11` 的偶发不匹配。

### 9.3 编译

```bash
cmake --build build --target 1q_lib -j"$(nproc)"
# 产物：build/lib/lib1q.a
ls -lh build/lib/lib1q.a
```

### 9.4 一次性兼容修复（已入库，勿回退）

Linux/老工具链曾卡在两处，已在仓库修好（提交 `3ba804e5`），回顾如下：

| 现象 | 根因 | 现状 |
|---|---|---|
| `add_library cannot create ALIAS target "nanoflann::nanoflann" because target "nanoflann_vendor" is imported but not globally visible` | CMake **3.16 不允许**给"非 GLOBAL 的 IMPORTED 目标"建 ALIAS（3.18+ 才放宽）| 6 个 vendor 目标已加 `GLOBAL`（`cmake/project/dependencies/VendorPackages.cmake`）|
| `'make_unique' is not a member of 'std'` | `std::make_unique` 属 **C++14**，工程下限是 C++11 | 5 处改为 `std::unique_ptr<T>(new T(...))`（`FusionEngine.cpp`、`SbirsTrackingCoordinator.cpp`）|

### 9.5 已验证矩阵

| 工具链 | 配置 | 编译 | 产物 |
|---|---|---|---|
| MSVC v120（VS2013 工具集）+ CMake 4.3.3 | ✅ | ✅ | `1q.lib` 148 MB |
| **g++ 7.5.0 + CMake 3.16.9**（Ubuntu 18.04）| ✅ | ✅ | **`lib1q.a` 46 MB** |
| g++ 13.3 + CMake 4.x（Ubuntu 24.04）| ✅ | 头文件级验证通过 | — |

### 9.6 常见问题（Linux 专有）

| 报错 | 处理 |
|---|---|
| `Could NOT find Boost` / `Eigen3` | 确认 `third_party/` 已就绪；本工程只用头文件（`boost/`、`Eigen/`），无需系统安装 |
| `sqlite3.c: No such file` | `third_party/sqlite/` 缺 amalgamation，见 §2 |
| `fatal error: flatbuffers/flatbuffers.h` | `third_party/flatbuffers` 不全（需 `include/` + `CMakeLists.txt`）|
| 链接时 `lib1q.a` 找不到 zlib/sqlite 符号 | `PACKAGE_MANAGER=none` 下这两个由源码编译；不要同时用系统 `-lz` |
