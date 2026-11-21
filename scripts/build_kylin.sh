#!/usr/bin/env bash
# 银河麒麟 V10（国防版/桌面版）/ 统信 UOS + 飞腾(Phytium)、鲲鹏(Kunpeng) aarch64 一键构建。
#
# 一条命令走完「环境体检 → 依赖（Conan）→ 配置 → 编译 →（可选）测试/安装」，
# 中途失败会按已知错误模式给出**具体修复建议**，而不是丢一个 ninja 报错给现场。
#
# 快速开始：
#   bash scripts/build_kylin.sh                 # Release + 单元测试，CPU 全核并行
#   bash scripts/build_kylin.sh --offline       # 完全离线（国防版常见）
#   bash scripts/build_kylin.sh --debug --ctest # 调试构建并跑 ctest
#   bash scripts/build_kylin.sh --selfcheck     # 只做环境体检，不编译
#
# 常用选项：
#   --debug | --release     构建类型（默认 release）
#   --tests | --no-tests    单元测试（默认 --tests）
#   --examples               编译示例（默认关闭；注意 examples 要求 GCC >= 8 之外无额外依赖）
#   --install                编译后安装
#   --offline                Conan 加 --no-remote，不访问 remote
#   --hdf5 | --no-hdf5       HighFive/HDF5 输出（默认关闭：aarch64 无预编译包，需源码构建）
#   --flight-dynamic         JSBSim 飞行动力学模块（默认关闭）
#   --jobs N                 并行度（默认 CPU 核数）
#   --build-dir DIR          自定义构建目录（会强制走「显式 -D」路线）
#   --install-dir DIR        安装前缀（默认 <build-dir>/install）
#   --route auto|preset|manual
#                            auto（默认）：CMake >= 3.21 用 preset，否则显式 -D
#                            preset：强制 cmake --preset（CMake 3.16 会失败）
#                            manual：强制显式 -D，兼容 CMake 3.16（麒麟出厂版本）
#   --no-conan               跳过 conan install（toolchain 已存在时复用）
#   --ctest                  构建后跑 ctest
#   --dry-run                只打印将执行的命令
#   -h | --help              帮助
#
# 环境变量（与文档 docs/practice/kylin_v10_build_guide.md §6 一致）：
#   ONEQ_CONAN_OFFLINE=1  ONEQ_CONAN_ARCH=armv8  ONEQ_ENABLE_HDF5=True
#   ONEQ_ENABLE_FLIGHT_DYNAMIC=True  ONEQ_FLATC_EXECUTABLE=/path/to/flatc
#   ONEQ_STRICT_FP=ON|OFF  ONEQ_ENABLE_LTO=ON|OFF  ONEQ_CXX=g++-7
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck source=lib/1q_env.sh
source "${SCRIPT_DIR}/lib/1q_env.sh"

# ---------------------------------------------------------------------------
# 默认值（全部可用命令行覆盖）
# ---------------------------------------------------------------------------
BUILD_TYPE="Release"
ENABLE_TESTING="ON"
ENABLE_EXAMPLES="OFF"
ENABLE_INSTALL="OFF"
ENABLE_HDF5="${ONEQ_ENABLE_HDF5:-OFF}"
ENABLE_FLIGHT_DYNAMIC="${ONEQ_ENABLE_FLIGHT_DYNAMIC:-OFF}"
STRICT_FP="${ONEQ_STRICT_FP:-ON}"
ENABLE_LTO="${ONEQ_ENABLE_LTO:-ON}"
OFFLINE="${ONEQ_CONAN_OFFLINE:-0}"
JOBS=""
BUILD_DIR=""
INSTALL_DIR=""
ROUTE="auto"
RUN_CTEST="0"
DO_INSTALL="0"
NO_CONAN="0"
DRY_RUN="0"
SELFCHECK_ONLY="0"
CXX_BIN="${ONEQ_CXX:-c++}"
CMAKE_BIN="${ONEQ_REAL_CMAKE:-cmake}"

# ---------------------------------------------------------------------------
# 基础工具
# ---------------------------------------------------------------------------
_oneq_log()  { printf '[kylin] %s\n' "$*"; }
_oneq_warn() { printf '[kylin] 警告: %s\n' "$*" >&2; }
_oneq_die()  { printf '[kylin] 错误: %s\n' "$*" >&2; exit 1; }

_oneq_usage() {
  awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "${BASH_SOURCE[0]}"
}

# 版本比较：_oneq_version_ge 3.21 3.20.5 → 0/1
_oneq_version_ge() {
  local IFS=.
  local i ver1=($1) ver2=($2)
  for ((i = 0; i < ${#ver2[@]}; i++)); do
    local v1="${ver1[i]:-0}" v2="${ver2[i]:-0}"
    [[ -z "${v1}" ]] && v1=0
    if ((10#${v1} > 10#${v2})); then return 0; fi
    if ((10#${v1} < 10#${v2})); then return 1; fi
  done
  return 0
}

_oneq_nproc() {
  if command -v nproc >/dev/null 2>&1; then
    nproc
  elif command -v getconf >/dev/null 2>&1; then
    getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4
  else
    echo 4
  fi
}

# ---------------------------------------------------------------------------
# 参数解析
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --debug)            BUILD_TYPE="Debug" ;;
    --release)          BUILD_TYPE="Release" ;;
    --tests)            ENABLE_TESTING="ON" ;;
    --no-tests)         ENABLE_TESTING="OFF" ;;
    --examples)         ENABLE_EXAMPLES="ON" ;;
    --no-examples)      ENABLE_EXAMPLES="OFF" ;;
    --install)          DO_INSTALL="1"; ENABLE_INSTALL="ON" ;;
    --offline)          OFFLINE="1" ;;
    --hdf5)             ENABLE_HDF5="ON" ;;
    --no-hdf5)          ENABLE_HDF5="OFF" ;;
    --flight-dynamic)   ENABLE_FLIGHT_DYNAMIC="ON" ;;
    --no-flight-dynamic) ENABLE_FLIGHT_DYNAMIC="OFF" ;;
    --jobs)             JOBS="${2:?--jobs 需要一个数字}"; shift ;;
    --build-dir)        BUILD_DIR="${2:?--build-dir 需要一个路径}"; shift ;;
    --install-dir)      INSTALL_DIR="${2:?--install-dir 需要一个路径}"; shift ;;
    --route)            ROUTE="${2:?--route 需要 auto|preset|manual}"; shift ;;
    --no-conan)         NO_CONAN="1" ;;
    --ctest)            RUN_CTEST="1" ;;
    --dry-run)          DRY_RUN="1" ;;
    --selfcheck)        SELFCHECK_ONLY="1" ;;
    -h|--help)          _oneq_usage; exit 0 ;;
    *)                  _oneq_die "未知参数: $1（用 --help 查看用法）" ;;
  esac
  shift
done

# 构建类型大小写归一（容错 --build-type 风格输入）
case "$(printf '%s' "${BUILD_TYPE}" | tr '[:upper:]' '[:lower:]')" in
  debug)   BUILD_TYPE="Debug" ;;
  release) BUILD_TYPE="Release" ;;
  *)       _oneq_die "构建类型只支持 Debug/Release，收到: ${BUILD_TYPE}" ;;
esac

# --build-dir 与 preset 互斥：preset 的 binaryDir 在 CMakePresets.json 里写死。
if [[ -n "${BUILD_DIR}" && "${ROUTE}" == "preset" ]]; then
  _oneq_die "--build-dir 与 --route preset 不能同时使用（preset 的构建目录不可改）"
fi
if [[ -n "${BUILD_DIR}" && "${ROUTE}" == "auto" ]]; then
  ROUTE="manual"
  _oneq_log "指定了 --build-dir，自动改走显式 -D 路线"
fi
[[ -z "${JOBS}" ]] && JOBS="$(_oneq_nproc)"

# ---------------------------------------------------------------------------
# 1. 环境体检
# ---------------------------------------------------------------------------
_ONEQ_CMAKE_VERSION=""
_ONEQ_CXX_VERSION=""
_ONEQ_GLIBC_VERSION=""
_ONEQ_ARCH=""

_oneq_detect_env() {
  _ONEQ_ARCH="$(uname -m 2>/dev/null || echo unknown)"

  # CMake 缺失不在这里退出：--selfcheck 只做环境体检，没有 CMake 也应该能出报告
  # （现场先体检、后装工具是常见顺序）。真正构建前由 _oneq_require_cmake 拦。
  if "${CMAKE_BIN}" --version >/dev/null 2>&1; then
    _ONEQ_CMAKE_VERSION="$("${CMAKE_BIN}" --version | head -1 | sed 's/.*version[[:space:]]*//')"
  else
    _ONEQ_CMAKE_VERSION=""
  fi

  if ! command -v "${CXX_BIN}" >/dev/null 2>&1; then
    _oneq_die "找不到 C++ 编译器 '${CXX_BIN}'。用 ONEQ_CXX=g++-7 指定，或确认 g++ 已安装"
  fi
  _ONEQ_CXX_VERSION="$("${CXX_BIN}" -dumpversion 2>/dev/null || "${CXX_BIN}" --version | head -1)"

  if command -v getconf >/dev/null 2>&1; then
    _ONEQ_GLIBC_VERSION="$(getconf GNU_LIBC_VERSION 2>/dev/null | awk '{print $2}' || true)"
  fi
}

# 构建前的硬性门槛（--selfcheck 不会走到这里）。
_oneq_require_cmake() {
  if [[ -z "${_ONEQ_CMAKE_VERSION}" ]]; then
    _oneq_die "找不到 cmake（>= 3.16）。麒麟上: sudo apt install cmake，或 ONEQ_CMAKE_ROOT 指向自备 CMake"
  fi
  if ! _oneq_version_ge "${_ONEQ_CMAKE_VERSION}" "3.16"; then
    _oneq_die "CMake ${_ONEQ_CMAKE_VERSION} 太旧，工程要求 >= 3.16"
  fi
}

_oneq_print_env() {
  local conan_ver="-"
  command -v conan >/dev/null 2>&1 && conan_ver="$(conan --version 2>/dev/null | awk '{print $NF}')"
  local ninja_ver="-"
  command -v ninja >/dev/null 2>&1 && ninja_ver="$(ninja --version 2>/dev/null | head -1)"
  local flatc_path="${ONEQ_FLATC_EXECUTABLE:-}"
  [[ -z "${flatc_path}" ]] && command -v flatc >/dev/null 2>&1 && flatc_path="$(command -v flatc)"
  [[ -z "${flatc_path}" ]] && flatc_path="（未装，将由 Conan 的 flatbuffers 包提供）"

  _oneq_log "================ 环境体检 ================"
  printf '  架构          : %s\n' "${_ONEQ_ARCH}"
  printf '  编译器        : %s (%s)\n' "${CXX_BIN}" "${_ONEQ_CXX_VERSION}"
  printf '  CMake         : %s\n' "${_ONEQ_CMAKE_VERSION:-未找到（构建前必须安装 >= 3.16）}"
  printf '  glibc         : %s\n' "${_ONEQ_GLIBC_VERSION:-未知（非 glibc 系统）}"
  printf '  Conan         : %s\n' "${conan_ver}"
  printf '  Ninja         : %s\n' "${ninja_ver}"
  printf '  flatc         : %s\n' "${flatc_path}"
  _oneq_log "=========================================="

  case "${_ONEQ_ARCH}" in
    aarch64|arm64) ;;
    *) _oneq_warn "当前架构是 ${_ONEQ_ARCH}，本脚本面向 aarch64；在 x86 上也可用于冒烟验证" ;;
  esac
  if command -v ninja >/dev/null 2>&1; then :; else
    _oneq_warn "未找到 ninja，将用 Makefile 生成器（慢一些）。麒麟上: sudo apt install ninja-build"
  fi
}

# ---------------------------------------------------------------------------
# 2. 能力探针：直白地告诉现场「这台机器的库到底有没有某个符号」
#    这三项正是麒麟 V10（GCC 7.3 / glibc 2.28）与开发者本机的差异所在。
# ---------------------------------------------------------------------------
# 探针用独立临时目录：--selfcheck 时不该在源码树里留下构建目录。
_ONEQ_PROBE_DIR="${TMPDIR:-/tmp}/oneq_probe_$$"
_oneq_probe_dir() { printf '%s' "${_ONEQ_PROBE_DIR}"; }

_probe_compile_ok() { # $1=源码内容 $2=额外旗标
  local dir src
  dir="$(_oneq_probe_dir)"
  mkdir -p "${dir}"
  src="${dir}/probe.cpp"
  printf '%s\n' "$1" > "${src}"
  # shellcheck disable=SC2086
  "${CXX_BIN}" -std=c++17 $2 -fsyntax-only "${src}" >/dev/null 2>&1
}

_probe_link_run() { # $1=源码内容 $2=额外旗标，成功则打印运行输出
  local dir src out
  dir="$(_oneq_probe_dir)"
  mkdir -p "${dir}"
  src="${dir}/probe_run.cpp"
  out="${dir}/probe_run.bin"
  printf '%s\n' "$1" > "${src}"
  # shellcheck disable=SC2086
  if "${CXX_BIN}" -std=c++17 $2 "${src}" -o "${out}" >/dev/null 2>&1; then
    "${out}" 2>/dev/null || true
  fi
}

_oneq_run_probes() {
  local fs_state="缺失" mallinfo2_state="缺失" mantissa="?" contract_state="不支持"
  local fs_lib_note="" exp_note=""

  if _probe_compile_ok '#include <filesystem>
int main() { return 0; }' ""; then
    fs_state="可用（无需 -lstdc++fs）"
  elif _probe_compile_ok '#include <experimental/filesystem>
int main() { return 0; }' ""; then
    fs_state="仅 experimental"
    if [[ -n "$("${CXX_BIN}" -print-file-name=libstdc++fs.a 2>/dev/null | grep -v '^libstdc++fs\.a$' || true)" ]]; then
      exp_note="需 -lstdc++fs（本工程已按 GCC 版本自动链接）"
    fi
  fi

  # mallinfo2 只在 glibc >= 2.33 声明，用「能否编译」判定即可，不必链接运行。
  if _probe_compile_ok '#include <malloc.h>
int main() { struct mallinfo2 m = mallinfo2(); (void)m; return 0; }' ""; then
    mallinfo2_state="可用"
  else
    mallinfo2_state="缺失（本工程回退 mallinfo）"
  fi

  mantissa="$(_probe_link_run '#include <cfloat>
#include <cstdio>
int main() { std::printf("%d", LDBL_MANT_DIG); return 0; }' "" 2>/dev/null || true)"
  [[ -z "${mantissa}" ]] && mantissa="?"

  if _probe_compile_ok 'int main() { return 0; }' "-ffp-contract=off"; then
    contract_state="支持"
  fi

  _oneq_log "---------------- 能力探针 -----------------"
  printf '  <filesystem>          : %s\n' "${fs_state}"
  [[ -n "${exp_note}" ]] && printf '                          %s\n' "${exp_note}"
  printf '  mallinfo2()           : %s\n' "${mallinfo2_state}"
  printf '  long double 尾数位    : %s (aarch64=113, x86-64=64；sizeof 都是 16)\n' "${mantissa}"
  printf '  -ffp-contract=off     : %s\n' "${contract_state}"
  _oneq_log "------------------------------------------"

  if [[ "${mantissa}" == "113" && "${_ONEQ_ARCH}" == "aarch64" ]]; then
    _oneq_log "提示: aarch64 的 long double 是 binary128 软浮点，与 x86-64 的 64 位尾数不同；"
    _oneq_log "      工程已用 ONEQ_STRICT_FP=${STRICT_FP} 关掉 FP 收缩以消除点积差异。"
  fi
  rm -rf "$(_oneq_probe_dir)"
}

# ---------------------------------------------------------------------------
# 3. 生成器与路线
# ---------------------------------------------------------------------------
_oneq_generator_args() {
  if command -v ninja >/dev/null 2>&1; then
    printf '%s\n' "-G" "Ninja"
  else
    printf '%s\n' "-G" "Unix Makefiles"
  fi
}

# 当前 CMake 能读的 CMakePresets 最高 version（映射见 CMake 文档：3.19→1、3.20→2、
# 3.21/3.22→3、3.23→4、3.24→5、3.25→6、3.27→7、3.28→8）。
_oneq_max_presets_version() {
  local v="${_ONEQ_CMAKE_VERSION}"
  if [[ -z "${v}" ]]; then echo 0; return 0; fi
  if   _oneq_version_ge "${v}" "3.28"; then echo 8
  elif _oneq_version_ge "${v}" "3.27"; then echo 7
  elif _oneq_version_ge "${v}" "3.25"; then echo 6
  elif _oneq_version_ge "${v}" "3.24"; then echo 5
  elif _oneq_version_ge "${v}" "3.23"; then echo 4
  elif _oneq_version_ge "${v}" "3.21"; then echo 3
  elif _oneq_version_ge "${v}" "3.20"; then echo 2
  elif _oneq_version_ge "${v}" "3.19"; then echo 1
  else echo 0
  fi
}

# 取 presets 文件里的顶层 version（文件不存在则输出空）。
_oneq_presets_file_version() {
  local file="$1"
  [[ -f "${file}" ]] || return 0
  grep -m1 -o '"version"[[:space:]]*:[[:space:]]*[0-9]\+' "${file}" | grep -o '[0-9]\+' || true
}

_oneq_decide_route() {
  local preset="linux-aarch64-$(printf '%s' "${BUILD_TYPE}" | tr '[:upper:]' '[:lower:]')"
  PRESET_NAME="${preset}"
  local maxv
  maxv="$(_oneq_max_presets_version)"

  if [[ "${ROUTE}" == "auto" ]]; then
    if [[ -z "${_ONEQ_CMAKE_VERSION}" ]]; then
      ROUTE="manual"
      _oneq_log "未检测到 CMake，按显式 -D 路线准备（构建前需先安装 CMake >= 3.16）"
    elif (( maxv < 3 )); then
      ROUTE="manual"
      _oneq_log "CMake ${_ONEQ_CMAKE_VERSION} 只支持 presets version ${maxv}（工程要求 3，即 CMake >= 3.21），改走显式 -D 路线"
    else
      ROUTE="preset"
      # Conan 生成的 CMakeUserPresets.json 用 version 4；CMake 3.21/3.22 读不了，
      # 会报 "Could not read presets from <dir>: Unrecognized version field"——
      # 而这个报错指向目录、不指向文件，现场极易误判成工程的 CMakePresets.json 坏了。
      local upv
      upv="$(_oneq_presets_file_version "${SOURCE_DIR}/CMakeUserPresets.json")"
      if [[ -n "${upv}" ]] && (( upv > maxv )); then
        ROUTE="manual"
        _oneq_warn "检测到 ${SOURCE_DIR}/CMakeUserPresets.json 为 version ${upv}，而 CMake ${_ONEQ_CMAKE_VERSION} 最多支持 ${maxv}"
        _oneq_warn "（Conan 生成的 user presets 用 version 4，CMake 3.21/3.22 会报 Unrecognized version field）"
        _oneq_log "改走显式 -D 路线以绕开 user presets；也可手动删除该文件后重跑"
      fi
    fi
  fi
  if [[ -z "${BUILD_DIR}" ]]; then
    BUILD_DIR="${SOURCE_DIR}/build/${PRESET_NAME}"
  fi
  [[ -z "${INSTALL_DIR}" ]] && INSTALL_DIR="${BUILD_DIR}/install"
  LOG_FILE="${BUILD_DIR}/oneq_kylin_build.log"
  TOOLCHAIN_FILE="${BUILD_DIR}/build/${BUILD_TYPE}/generators/conan_toolchain.cmake"
}

# ---------------------------------------------------------------------------
# 4. 依赖（Conan）
# ---------------------------------------------------------------------------
_oneq_conan_install() {
  [[ "${DRY_RUN}" == "1" ]] || command -v conan >/dev/null 2>&1 \
    || _oneq_die "找不到 conan。安装: pip3 install 'conan>=2.0'（离线环境用离线包）"

  local args=(
    install "${SOURCE_DIR}"
    --output-folder "${BUILD_DIR}"
    --build=missing
    -s "compiler.cppstd=17"
    -s "build_type=${BUILD_TYPE}"
    -o "&:enable_testing=${ENABLE_TESTING}"
    -o "&:enable_flight_dynamic=${ENABLE_FLIGHT_DYNAMIC}"
    -o "&:enable_hdf5=${ENABLE_HDF5}"
  )
  if [[ -n "${ONEQ_CONAN_ARCH:-}" ]]; then
    args+=(-s "arch=${ONEQ_CONAN_ARCH}")
  fi
  if [[ "${OFFLINE}" == "1" ]]; then
    args+=(--no-remote)
  fi

  _oneq_log "Conan 依赖（HDF5=${ENABLE_HDF5} JSBSim=${ENABLE_FLIGHT_DYNAMIC} 离线=${OFFLINE}）"
  _oneq_run conan "${args[@]}"

  if [[ ! -f "${TOOLCHAIN_FILE}" ]]; then
    _oneq_warn "预期 toolchain 不存在: ${TOOLCHAIN_FILE}"
    _oneq_warn "检查 conan 是否用了别的 build_type 目录（多配置生成器路径不含 build_type）"
  fi
}

# ---------------------------------------------------------------------------
# 5. 配置 / 编译 / 测试 / 安装
# ---------------------------------------------------------------------------
_oneq_configure() {
  if [[ "${ROUTE}" == "preset" ]]; then
    _oneq_log "配置（preset ${PRESET_NAME}）"
    _oneq_run "${CMAKE_BIN}" --preset "${PRESET_NAME}" \
      -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
      -DENABLE_TESTING="${ENABLE_TESTING}" \
      -DENABLE_EXAMPLES="${ENABLE_EXAMPLES}" \
      -DENABLE_INSTALL="${ENABLE_INSTALL}" \
      -DBUILD_SHARED_LIBS=OFF \
      -DONEQ_ENABLE_HDF5="${ENABLE_HDF5}" \
      -DONEQ_ENABLE_FLIGHT_DYNAMIC="${ENABLE_FLIGHT_DYNAMIC}" \
      -DONEQ_STRICT_FP="${STRICT_FP}" \
      -DONEQ_ENABLE_LTO="${ENABLE_LTO}"
    return 0
  fi

  [[ "${DRY_RUN}" == "1" || -f "${TOOLCHAIN_FILE}" ]] \
    || _oneq_die "找不到 toolchain: ${TOOLCHAIN_FILE}（去掉 --no-conan 或先跑 conan install）"
  local gen_args=()
  while IFS= read -r line; do gen_args+=("${line}"); done < <(_oneq_generator_args)

  _oneq_log "配置（显式 -D，兼容 CMake 3.16）"
  _oneq_run "${CMAKE_BIN}" \
    -S "${SOURCE_DIR}" -B "${BUILD_DIR}" \
    "${gen_args[@]}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
    -DPACKAGE_MANAGER=conan \
    -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
    -DENABLE_TESTING="${ENABLE_TESTING}" \
    -DENABLE_EXAMPLES="${ENABLE_EXAMPLES}" \
    -DENABLE_INSTALL="${ENABLE_INSTALL}" \
    -DONEQ_ENABLE_HDF5="${ENABLE_HDF5}" \
    -DONEQ_ENABLE_FLIGHT_DYNAMIC="${ENABLE_FLIGHT_DYNAMIC}" \
    -DONEQ_STRICT_FP="${STRICT_FP}" \
    -DONEQ_ENABLE_LTO="${ENABLE_LTO}" \
    -DONEQ_LINKER_STACK_OPTION=OFF
}

_oneq_build() {
  _oneq_log "编译（-j ${JOBS}）"
  _oneq_run "${CMAKE_BIN}" --build "${BUILD_DIR}" --parallel "${JOBS}"
}

_oneq_test() {
  local ctest_args=(-j "${JOBS}" --output-on-failure)
  if _oneq_version_ge "${_ONEQ_CMAKE_VERSION}" "3.20"; then
    _oneq_log "运行 ctest"
    _oneq_run ctest --test-dir "${BUILD_DIR}" "${ctest_args[@]}"
  else
    _oneq_log "运行 ctest（CMake < 3.20，需先进入构建目录）"
    _oneq_run bash -c "cd '${BUILD_DIR}' && ctest -j '${JOBS}' --output-on-failure"
  fi
}

_oneq_install() {
  _oneq_log "安装到 ${INSTALL_DIR}"
  _oneq_run "${CMAKE_BIN}" --install "${BUILD_DIR}" --prefix "${INSTALL_DIR}"
}

# ---------------------------------------------------------------------------
# 6. 失败诊断：把已知错误模式翻译成「改什么」
# ---------------------------------------------------------------------------
_oneq_diagnose() {
  local log="$1"
  [[ -f "${log}" ]] || return 0
  local hit=0
  _diag() { printf '  → %s\n' "$1"; hit=1; }

  echo
  _oneq_log "================ 失败诊断 ================"

  if grep -qF 'cannot find -lHighFive' "${log}"; then
    _diag "HighFive 被当裸库名链接（-lHighFive）。用 -DONEQ_ENABLE_HDF5=OFF 关闭 HDF5 输出，"
    _diag "  或确认 cmake/project/dependencies/ConanPackages.cmake 已解析 ONEQ_HIGHFIVE_TARGET。"
  fi
  if grep -qF 'contains: SQLite::SQLite3 but the target was not found' "${log}"; then
    _diag "下游 find_package(1q) 缺少 find_dependency(SQLite3)：确认 cmake/project/ProjectInstall.cmake。"
  fi
  if grep -qF 'Unrecognized "version" field' "${log}"; then
    _diag "CMake 读不了某个 presets 文件（报错只给目录、不给文件名）。最常见是 Conan 生成的"
    _diag "  CMakeUserPresets.json 用了 version 4，而 CMake 3.21/3.22 只支持 3。"
    _diag "  处理：删掉 CMakeUserPresets.json，或改用本脚本的显式 -D 路线（--route manual）。"
  fi
  if grep -qF 'Could not read presets' "${log}"; then
    _diag "presets 无法解析：确认 CMake 版本 >= presets 文件声明的 version（工程为 3，需 CMake >= 3.21），"
    _diag "  且没有同目录的 CMakeUserPresets.json 用了更高 version。"
  fi
    if grep -qE "unrecognized command-line option '-W" "${log}"; then
    _diag "编译器不认识某个 -W 旗标（GCC 对未知 -W 是 error）。本工程已改为探测式下发，"
    _diag "  若仍出现说明有外部 -W 注入：检查 CMAKE_CXX_FLAGS 或 toolchain 文件。"
  fi
  if grep -qE "unrecognized -z option|unknown -z option" "${log}"; then
    _diag "链接器不认识 -z stack-size（GNU ld 不支持）。加 -DONEQ_LINKER_STACK_OPTION=OFF。"
  fi
  if grep -qE "fatal error: filesystem: No such file|filesystem: No such file or directory" "${log}"; then
    _diag "GCC < 8 没有 <filesystem>。工程统一走 include/oneq/FsCompat.h；若报错在第三方"
    _diag "  或 examples 里，确认它 #include <oneq/FsCompat.h> 而不是 <filesystem>。"
  fi
  if grep -qF "'mallinfo2' was not declared" "${log}" || grep -qF 'mallinfo2 was not declared' "${log}"; then
    _diag "glibc < 2.33 没有 mallinfo2。tests/performance/.../rf_interference_performance_test.cpp"
    _diag "  已按 __GLIBC_PREREQ(2,33) 回退到 struct mallinfo。"
  fi
  if grep -qE "undefined reference to .pthread_" "${log}"; then
    _diag "缺少 pthread 链接：确认 cmake/project/ProjectDependencies.cmake 里的 Threads::Threads，"
    _diag "  或在配置阶段加 -DCMAKE_CXX_FLAGS=-pthread 临时绕过。"
  fi
  if grep -qF 'plugin needed to handle lto object' "${log}"; then
    _diag "静态库带 LTO 对象但消费方没有 lto 插件：加 -DONEQ_ENABLE_LTO=OFF（默认已加 -ffat-lto-objects）。"
  fi
  if grep -qE "undefined reference to .std::experimental::filesystem" "${log}"; then
    _diag "GCC 7/8 需要显式链接 stdc++fs：本工程已按 GCC 版本自动链 libstdc++fs.a，"
    _diag "  若报错在自建目标上，检查是否绕过了 oneq_apply_target_configuration。"
  fi
  if grep -qF 'third_party/eigen' "${log}"; then
    _diag "PACKAGE_MANAGER=none 需要 third_party 源码，当前仓库没有：改用 Conan（不加 -DPACKAGE_MANAGER=none）。"
  fi
  if grep -qiE 'flatc.*not found|flatc.*No such file' "${log}"; then
    _diag "找不到 flatc。两种解法：1) 保持 conanfile.py 里 flatbuffers/*:header_only=False；"
    _diag "  2) 交叉编译时用 -DONEQ_FLATC_EXECUTABLE=/usr/bin/flatc 指定宿主 flatc。"
  fi
  if grep -qiE 'Could not connect|Failed to connect|timed out|Connection.*refused' "${log}"; then
    _diag "Conan 访问 remote 失败（离线环境常见）。加 --offline（等价 ONEQ_CONAN_OFFLINE=1），"
    _diag "  或先把依赖缓存打包迁到本机 ~/.conan2。"
  fi
  if grep -qE "GLIBC_2\.[0-9]+.{0,20}not found|version .GLIBCXX_.* not found" "${log}"; then
    _diag "构建机 glibc/libstdc++ 高于运行机：在低版本机器上编译，或用容器做构建环境。"
  fi
  if grep -qF 'ninja: command not found' "${log}"; then
    _diag "没装 ninja：sudo apt install ninja-build，或改用 -G 'Unix Makefiles'。"
  fi

  if [[ "${hit}" == "0" ]]; then
    printf '  （未匹配到已知模式，请查看完整日志）\n'
  fi
  printf '  完整日志: %s\n' "${log}"
  _oneq_log "=========================================="
}

# ---------------------------------------------------------------------------
# 7. 执行包装：记录日志 + 失败即诊断
# ---------------------------------------------------------------------------
_on_error() {
  local code=$?
  [[ "${code}" -eq 0 ]] && return 0
  echo
  _oneq_log "命令失败（退出码 ${code}）"
  [[ -n "${LOG_FILE:-}" ]] && _oneq_diagnose "${LOG_FILE}"
  exit "${code}"
}

_oneq_run() {
  if [[ "${DRY_RUN}" == "1" ]]; then
    printf '  [dry-run]'
    printf ' %q' "$@"
    printf '\n'
    return 0
  fi

  mkdir -p "$(dirname "${LOG_FILE}")"
  {
    printf '\n===== %s =====\n' "$(date '+%Y-%m-%d %H:%M:%S')"
    printf '$'
    printf ' %q' "$@"
    printf '\n'
  } >> "${LOG_FILE}"

  # 注意：pipefail 会把 tee 之后的管道状态带回，命令失败时返回非 0，由 ERR trap 处理。
  if ! "$@" 2>&1 | tee -a "${LOG_FILE}"; then
    return 1
  fi
  return 0
}

# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
trap _on_error ERR

_oneq_detect_env
_oneq_print_env
_oneq_decide_route
_oneq_run_probes

if [[ "${SELFCHECK_ONLY}" == "1" ]]; then
  _oneq_log "自检完成（未编译）。去掉 --selfcheck 即可开始构建。"
  exit 0
fi

_oneq_require_cmake

_oneq_log "路线=${ROUTE}  构建类型=${BUILD_TYPE}  构建目录=${BUILD_DIR}"
[[ "${ROUTE}" == "preset" ]] && _oneq_log "preset=${PRESET_NAME}"
_oneq_log "开关: TESTING=${ENABLE_TESTING} EXAMPLES=${ENABLE_EXAMPLES} INSTALL=${ENABLE_INSTALL} HDF5=${ENABLE_HDF5} JSBSim=${ENABLE_FLIGHT_DYNAMIC} STRICT_FP=${STRICT_FP} LTO=${ENABLE_LTO}"

if [[ "${NO_CONAN}" == "1" ]]; then
  _oneq_log "跳过 conan install（--no-conan）"
else
  _oneq_conan_install
fi

[[ "${DRY_RUN}" == "1" ]] || mkdir -p "${BUILD_DIR}"

_oneq_configure
_oneq_build

if [[ "${RUN_CTEST}" == "1" ]]; then
  _oneq_test
fi

if [[ "${DO_INSTALL}" == "1" ]]; then
  _oneq_install
fi

echo
_oneq_log "================ 构建完成 ================"
printf '  构建目录 : %s\n' "${BUILD_DIR}"
printf '  日志     : %s\n' "${LOG_FILE}"
if [[ -f "${BUILD_DIR}/lib/lib1q.a" ]]; then
  printf '  静态库   : %s\n' "${BUILD_DIR}/lib/lib1q.a"
elif [[ -f "${BUILD_DIR}/lib/lib1q.so" ]]; then
  printf '  动态库   : %s\n' "${BUILD_DIR}/lib/lib1q.so"
fi
if [[ "${DO_INSTALL}" == "1" ]]; then
  printf '  安装前缀 : %s\n' "${INSTALL_DIR}"
fi
printf '  下一步   : ctest --test-dir %s -j %s --output-on-failure\n' "${BUILD_DIR}" "${JOBS}"
_oneq_log "=========================================="