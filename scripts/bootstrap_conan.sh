#!/usr/bin/env bash
# Conan 引导脚本：按 preset 名派生参数并执行 conan install，生成 toolchain 与 CMakeDeps。
#
# 用法：
#   scripts/bootstrap_conan.sh <preset>
#
# 迁移自 ConanBootstrapToolchain.cmake（toolchain 内部触发 conan install）的两步式方案。
# 本脚本只负责"装依赖、生成 toolchain"，配置仍由 cmake --preset 完成：
#
#   scripts/bootstrap_conan.sh llvm-ninja-debug     # 1. 装依赖、生成 conan_toolchain.cmake
#   cmake --preset llvm-ninja-debug                 # 2. 配置（自动发现生成的 toolchain）
#   cmake --build --preset llvm-ninja-debug         # 3. 构建
#
# Windows (VS2015) 同样两步：
#   scripts/bootstrap_conan.sh VisualStudio.14.0-amd64
#   cmake --preset VisualStudio.14.0-amd64
#   cmake --build --preset VisualStudio.14.0-amd64-debug
#
# Windows (VS2015 + 验收日志开关默认 ON)：无 Conan 变体，不走本脚本——
#   scripts\win\fetch_third_party.bat      # 1. 拉取第三方源码到 third_party/（一次性）
#   cmake --preset 1q_log_vs2015           # 2. 配置（验收日志开关默认 ON）
#   cmake --build --preset 1q_log_vs2015-release
#
# Windows (v141 老工具集)：
#   scripts/bootstrap_conan.sh VisualStudio.15.0-amd64
#   cmake --preset VisualStudio.15.0-amd64
#   cmake --build --preset VisualStudio.15.0-amd64-debug
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/1q_env.sh
source "${SCRIPT_DIR}/lib/1q_env.sh"

# --- 参数校验 ---
if [[ $# -ne 1 ]]; then
    echo "用法: $0 <preset>" >&2
    echo "支持的 preset: llvm-ninja-debug | llvm-ninja-release | VisualStudio.14.0-amd64 | VisualStudio.15.0-amd64" >&2
    exit 2
fi

PRESET="$1"

# 源码根目录（脚本位于 <root>/scripts/ 下）。
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONANFILE="${SOURCE_DIR}/conanfile.py"

if [[ ! -f "${CONANFILE}" ]]; then
    echo "错误: 未找到 conanfile.py: ${CONANFILE}" >&2
    exit 1
fi

# --- 定位 conan 可执行文件 ---
if ! command -v conan >/dev/null 2>&1; then
    echo "错误: 未找到 conan 可执行文件，请先安装 Conan 2.x（pip install conan）" >&2
    exit 1
fi

# C++ 标准默认 17（与 cmake/project/ProjectSetup.cmake 的 PROJECT_DEFAULT_CXX_STANDARD 一致）。
# VS2015 (msvc 190) 在 Conan 支持表里只接受 cppstd=14（MSVC 无 C++11 档位，/std:c++14 覆盖 11+14）；
# flight_dynamic 模块默认 OFF 且 Windows 不拉 JSBSim，14 不触发其 C++17 第三方依赖。
CPPSTD=17
# 非 Windows 平台标记：为 1 时走 Linux/国产化分支（自动探测 profile + 可选依赖开关）。
PLATFORM_LINUX=0

# --- preset → 参数映射 -------------------------------------------------------
# 每个 preset 派生：binaryDir、build_type（单配置才有）、enable_testing、generator。
# binaryDir 必须与 CMakePresets.json 中各 preset 的 binaryDir 完全一致，否则 cmake 找不到
# 生成的 toolchain。macOS 两族 preset 均继承 base 的 ${sourceDir}/build/${presetName}；
# 用户级 -local 变体（CMakeUserPresets.json）的目录才带 -local 后缀。
case "${PRESET}" in
    llvm-ninja-debug)
        BINARY_DIR="${SOURCE_DIR}/build/llvm-ninja-debug"
        BUILD_TYPE="Debug"
        ENABLE_TESTING="True"
        GENERATOR=""            # 单配置 Ninja，无需告知 Conan 生成器
        ;;
    llvm-ninja-release)
        BINARY_DIR="${SOURCE_DIR}/build/llvm-ninja-release"
        BUILD_TYPE="Release"
        ENABLE_TESTING="True"
        GENERATOR=""
        ;;
    # 本地用户 preset 变体（CMakeUserPresets.json，不进 git，binaryDir 带 -local）。
    llvm-ninja-debug-local)
        BINARY_DIR="${SOURCE_DIR}/build/llvm-ninja-debug-local"
        BUILD_TYPE="Debug"
        ENABLE_TESTING="True"
        GENERATOR=""
        ;;
    llvm-ninja-release-local)
        BINARY_DIR="${SOURCE_DIR}/build/llvm-ninja-release-local"
        BUILD_TYPE="Release"
        ENABLE_TESTING="True"
        GENERATOR=""
        ;;
    VisualStudio.14.0-amd64)
        # VS2015 多配置：binaryDir 与 CMakePresets.json 的 windows-base 一致
        # （${sourceDir}/build/${presetName}），conan 在其下生成 build/generators/。
        BINARY_DIR="${SOURCE_DIR}/build/VisualStudio.14.0-amd64"
        BUILD_TYPE=""            # 多配置生成器，不固定 build_type
        ENABLE_TESTING="False"   # 与 preset ENABLE_TESTING 缺省(OFF)一致
        GENERATOR="Visual Studio 14 2015"
        CPPSTD=14                # msvc 190 在 Conan 仅支持 14（覆盖 11/14）
        ;;
    VisualStudio.15.0-amd64)
        # v141 老工具集（14.16.27023，VS2017 时代）。CMake 4.3.1 的 "Visual Studio 15 2017"
        # 生成器按版本范围找实例（找不到 VS2026），因此生成器用 "Visual Studio 18 2026"，
        # 工具集显式 v141。编译器检测与链接均由 64 位 MSBuild(amd64) 执行。
        #
        # 历史注：本机 64 位注册表 KitsRoot10 死路径曾致 LNK1104(ucrtd.lib)，靠
        # UCRTContentRoot 环境变量绕过；2026-08-21 注册表已机器级修复，注入已退役
        # （详见 docs/practice/windows_local_build.md）。
        BINARY_DIR="${SOURCE_DIR}/build/VisualStudio.15.0-amd64"
        BUILD_TYPE=""            # 多配置生成器，不固定 build_type
        ENABLE_TESTING="True"
        GENERATOR="Visual Studio 18 2026"
        CPPSTD=17
        ;;
    # 国产化平台（银河麒麟 V10 国防版 / 统信 UOS + 飞腾、鲲鹏 aarch64）。
    # 与 windows-base 不同，这里**不写死** os/arch/compiler：Conan 2 在目标机上按
    # profiles/default 自动探测（Linux/armv8/gcc），写死反而会在交叉编译或不同
    # 麒麟小版本上出错。需要交叉编译时用 ONEQ_CONAN_ARCH=armv8 覆盖。
    linux-aarch64-debug)
        BINARY_DIR="${SOURCE_DIR}/build/linux-aarch64-debug"
        BUILD_TYPE="Debug"
        ENABLE_TESTING="True"
        GENERATOR=""
        PLATFORM_LINUX=1
        ;;
    linux-aarch64-release)
        BINARY_DIR="${SOURCE_DIR}/build/linux-aarch64-release"
        BUILD_TYPE="Release"
        ENABLE_TESTING="True"
        GENERATOR=""
        PLATFORM_LINUX=1
        ;;
    *)
        echo "错误: 不支持的 preset '${PRESET}'" >&2
        echo "支持: llvm-ninja-debug | llvm-ninja-release | linux-aarch64-debug | linux-aarch64-release | VisualStudio.14.0-amd64 | VisualStudio.15.0-amd64" >&2
        exit 2
        ;;
esac

# --- 组装 conan install 公共参数 ---------------------------------------------
# 单配置 preset（BUILD_TYPE 非空）只装该配置；多配置 preset（BUILD_TYPE 为空，
# 如 VS 多配置生成器）需对 Debug + Release 各装一次，CMakeDeps 才会为两套配置
# 都生成 *-Target-debug.cmake / *-Target-release.cmake，否则缺配置的 build 找不到 include。
if [[ -n "${BUILD_TYPE}" ]]; then
    CONAN_BUILD_TYPES=("${BUILD_TYPE}")
else
    CONAN_BUILD_TYPES=("Debug" "Release")
fi

# 公共参数：输出目录、构建策略、cppstd、testing 开关。
CONAN_COMMON_ARGS=(
    install "${SOURCE_DIR}"
    --output-folder "${BINARY_DIR}"
    --build=missing
    -s "compiler.cppstd=${CPPSTD}"
    -o "&:enable_testing=${ENABLE_TESTING}"
)

# Windows (VS2015) preset 设置 GENERATOR 后走多配置 MSVC 分支。
if [[ -n "${GENERATOR}" ]]; then
    CONAN_COMMON_ARGS+=(
        -c "tools.cmake.cmaketoolchain:generator=${GENERATOR}"
        -s "os=Windows"
        -s "arch=x86_64"
        -s "compiler=msvc"
        -s "compiler.runtime=dynamic"
    )
    case "${PRESET}" in
        VisualStudio.14.0-amd64)
            CONAN_COMMON_ARGS+=(
                -c "tools.microsoft.msbuild:vs_version=14"
                -s "compiler.version=190"
            )
            ;;
        VisualStudio.15.0-amd64)
            CONAN_COMMON_ARGS+=(
                -c "tools.microsoft.msbuild:vs_version=18"
                -s "compiler.version=191"
            )
            ;;
    esac
fi

# 国产化/离线平台：显式声明可选组件开关，与 CMake 的 ONEQ_ENABLE_FLIGHT_DYNAMIC /
# ONEQ_ENABLE_HDF5 一一对应（两处必须同步，否则会出现「CMake 要 HighFive，Conan 没装」
# 或反之的配置期失败）。
#
# 默认把 HDF5 关掉：HighFive→HDF5 在 aarch64 没有 ConanCenter 预编译包，要连
# libaec/szip 一起源码构建，是离线环境里最容易失败的一环；需要 SAR HDF5 输出时
# 用 ONEQ_ENABLE_HDF5=True 打开。
if [[ "${PLATFORM_LINUX}" == "1" ]]; then
    CONAN_COMMON_ARGS+=(
        -o "&:enable_flight_dynamic=${ONEQ_ENABLE_FLIGHT_DYNAMIC:-False}"
        -o "&:enable_hdf5=${ONEQ_ENABLE_HDF5:-False}"
    )
    if [[ -n "${ONEQ_CONAN_ARCH:-}" ]]; then
        CONAN_COMMON_ARGS+=(-s "arch=${ONEQ_CONAN_ARCH}")
    fi
    # 完全离线（国防版常见）：--build=missing 仍会先尝试访问 remote，加 --no-remote
    # 强制只从本地缓存/源码构建，避免每次安装都卡在网络超时上。
    if [[ "${ONEQ_CONAN_OFFLINE:-0}" == "1" ]]; then
        CONAN_COMMON_ARGS+=(--no-remote)
    fi
fi

# --- 执行（按配置列表循环）---------------------------------------------------
echo "[bootstrap_conan] preset=${PRESET}"
echo "[bootstrap_conan] output-folder=${BINARY_DIR}"
echo "[bootstrap_conan] build_type=${BUILD_TYPE:-multi-config(Debug+Release)}"
echo "[bootstrap_conan] enable_testing=${ENABLE_TESTING}"
echo

for _build_type in "${CONAN_BUILD_TYPES[@]}"; do
    echo "[bootstrap_conan] --- build_type=${_build_type} ---"
    echo "[bootstrap_conan] 运行: conan ${CONAN_COMMON_ARGS[*]} -s build_type=${_build_type}"
    echo
    conan "${CONAN_COMMON_ARGS[@]}" -s "build_type=${_build_type}"
done

# --- 报告生成的 toolchain 路径 ----------------------------------------------
if [[ -n "${GENERATOR}" ]]; then
    # VS multi-config：toolchain 在 <binaryDir>/build/generators/（不带 build_type）。
    TOOLCHAIN="${BINARY_DIR}/build/generators/conan_toolchain.cmake"
else
    # 单配置：toolchain 在 <binaryDir>/build/<build_type>/generators/。
    TOOLCHAIN="${BINARY_DIR}/build/${BUILD_TYPE}/generators/conan_toolchain.cmake"
fi

echo
echo "[bootstrap_conan] 成功。生成的 toolchain:"
echo "  ${TOOLCHAIN}"
if [[ ! -f "${TOOLCHAIN}" ]]; then
    echo "警告: 预期路径未找到 toolchain 文件，请检查 conan 输出。" >&2
fi
echo
echo "下一步："
echo "  cmake --preset ${PRESET}"
