#!/usr/bin/env bash
# 1Q 构建/测试统一入口（跨平台：macOS / Linux 直接用原生 cmake+ctest；
# Windows Git Bash/WSL 经 scripts/win/ 的包装器 exec cmake.exe/ctest.exe）。
# 平台矩阵与各脚本归属见 scripts/README.md。
#
# 用法：
#   source scripts/win/activate_git_bash.sh    # 仅 Windows：每个 Git Bash 会话一次（或写入 ~/.bashrc）
#   scripts/1q.sh bootstrap llvm-ninja-release-local
#   scripts/1q.sh configure llvm-ninja-release-local
#   scripts/1q.sh build llvm-ninja-release-local [--target T ...]
#   scripts/1q.sh test llvm-ninja-release-local [-R regex] [-j N]
#   scripts/1q.sh install llvm-ninja-release-local [--prefix DIR]
#   scripts/1q.sh clean-stale VisualStudio.15.0-amd64 remote_identification_radar
#   scripts/1q.sh doctor
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/1q_env.sh
source "${SCRIPT_DIR}/lib/1q_env.sh"

usage() {
  cat <<'EOF'
1Q 构建/测试工具（跨平台；平台矩阵见 scripts/README.md）

  scripts/1q.sh bootstrap <configure_preset>
  scripts/1q.sh configure <configure_preset>
  scripts/1q.sh build <build_preset> [--target T ...] [-j N]
  scripts/1q.sh test <test_preset> [-R regex] [-j N]
  scripts/1q.sh install <configure_preset> [--prefix DIR]   # 打包安装（不构建）
  scripts/1q.sh clean-stale <configure_preset> <module_slug>
  scripts/1q.sh doctor

configure_preset 例：llvm-ninja-release-local | VisualStudio.15.0-amd64
build/test preset 例：llvm-ninja-release-local | VisualStudio.15.0-amd64-release

build 禁止隐式 reconfigure：CMake/依赖变更后须先 configure 对应 preset，再 build。
切换 preset 时 configure/build/test 三者的 preset 族必须一致（例：15.0-amd64 / 15.0-amd64-release）。

Windows（Git Bash/WSL）首次使用：source scripts/win/activate_git_bash.sh
EOF
}

doctor() {
  echo "ONEQ_ROOT=${ONEQ_ROOT}"
  echo "ONEQ_CMAKE_ROOT=${ONEQ_CMAKE_ROOT:-<unset>}"
  echo "uname=$(uname -s 2>/dev/null || echo unknown)"
  _wsl_windows_repo=0
  case "$(uname -s 2>/dev/null || true)" in
    Linux*)
      if [[ "${ONEQ_ROOT}" == /mnt/?/* || "${ONEQ_ROOT}" == /d/* || "${ONEQ_ROOT}" == /D/* ]]; then
        _wsl_windows_repo=1
      fi
      ;;
  esac
  if command -v cmake >/dev/null 2>&1; then
    echo "cmake=$(command -v cmake)"
    cmake --version | head -n 1
  else
    echo "cmake=NOT FOUND"
  fi
  _ctest="$(_oneq_resolve_ctest || true)"
  if [[ -n "${_ctest}" ]]; then
    echo "ctest=${_ctest}"
  else
    echo "ctest=NOT FOUND"
  fi
  if [[ "${_wsl_windows_repo}" -eq 1 ]]; then
    if [[ -n "${_ctest}" ]]; then
      echo "SHELL_NOTE=WSL bash on a Windows repo path."
      echo "  Daily build/test: scripts/1q.sh (wrappers exec Windows cmake.exe/ctest.exe)."
      echo "  Do not use Linux-native cmake/ctest against VisualStudio.* presets."
    else
      echo "SHELL_WARNING=WSL bash cannot resolve ctest.exe."
      echo "  source scripts/win/activate_git_bash.sh so scripts/win/bin/ctest is on PATH."
      echo "  Do not invoke Linux /usr/bin/ctest against a VisualStudio build tree."
    fi
  fi
  unset _ctest _wsl_windows_repo
  echo "ONEQ_REAL_CMAKE=${ONEQ_REAL_CMAKE:-<unset>}"
  echo "ONEQ_GIT_BASH_ACTIVATED=${ONEQ_GIT_BASH_ACTIVATED:-0}"
}

# Resolve ctest even when the shell only finds Windows *.exe via full path
# (WSL: command -v ctest fails; Git Bash: finds ctest.exe as ctest).
_oneq_resolve_ctest() {
  if command -v ctest >/dev/null 2>&1; then
    command -v ctest
    return 0
  fi
  if [[ -n "${ONEQ_REAL_CMAKE:-}" ]]; then
    local dir candidate
    dir="$(dirname "${ONEQ_REAL_CMAKE}")"
    for candidate in "${dir}/ctest.exe" "${dir}/ctest"; do
      if [[ -x "${candidate}" || -f "${candidate}" ]]; then
        printf '%s' "${candidate}"
        return 0
      fi
    done
  fi
  return 1
}

# 解析 preset 的 binaryDir：以 1q_preset_query.py 解析为准，勿硬编码——binaryDir
# 由 preset 定义决定（两族 macOS preset 均继承 base 的 ${sourceDir}/build/${presetName}）。
_oneq_binary_dir() {
  local preset="$1" py
  py="$(command -v python3 || command -v python || true)"
  if [[ -z "${py}" ]]; then
    echo "[1q.sh] 需要 python3 解析 preset 的 binaryDir" >&2
    return 1
  fi
  "${py}" "${SCRIPT_DIR}/lib/1q_preset_query.py" "${preset}" binaryDir
}

# 解析 preset 的 conan toolchain 路径（无 Conan 的 preset 输出为空，如 1q_log_vs2015）。
_oneq_toolchain_file() {
  local preset="$1" py
  py="$(command -v python3 || command -v python || true)"
  if [[ -z "${py}" ]]; then
    echo "[1q.sh] 需要 python3 解析 preset 的 toolchain" >&2
    return 1
  fi
  "${py}" "${SCRIPT_DIR}/lib/1q_preset_query.py" "${preset}" toolchain
}

cmd="${1:-}"
shift || true

case "${cmd}" in
  bootstrap)
    [[ $# -ge 1 ]] || { usage; exit 2; }
    exec bash "${SCRIPT_DIR}/bootstrap_conan.sh" "$1"
    ;;
  configure)
    [[ $# -ge 1 ]] || { usage; exit 2; }
    preset="$1"
    shift || true
    # 首次配置自动引导：preset 的 conan toolchain 不存在时先跑 bootstrap（一次性；
    # 无 Conan 的 preset 无 toolchain 字段，不触发）。
    _tc="$(_oneq_toolchain_file "${preset}" 2>/dev/null || true)"
    if [[ -n "${_tc}" ]] && [[ ! -f "${_tc}" ]]; then
      echo "[1q.sh configure] conan toolchain 缺失，自动 bootstrap: ${preset}"
      bash "${SCRIPT_DIR}/bootstrap_conan.sh" "${preset}"
    fi
    unset _tc
    exec cmake --preset "${preset}" "$@"
    ;;
  build)
    [[ $# -ge 1 ]] || { usage; exit 2; }
    preset="$1"
    shift
    # shellcheck source=lib/1q_build_guard.sh
    source "${SCRIPT_DIR}/lib/1q_build_guard.sh"
    _oneq_run_build_guard "${preset}" || exit 1
    # shift 后 "$@" 即额外参数；macOS bash 3.2 下 set -u 对空数组
    # "${extra[@]}" 报 unbound，故不经过中间数组。
    exec cmake --build --preset "${preset}" "$@"
    ;;
  test)
    [[ $# -ge 1 ]] || { usage; exit 2; }
    preset="$1"
    shift
    extra=(--output-on-failure)
    while [[ $# -gt 0 ]]; do
      extra+=("$1")
      shift
    done
    if ! printf '%s\n' "${extra[@]}" | grep -q -- '-j'; then
      extra+=(-j 4)
    fi
    _ctest="$(_oneq_resolve_ctest || true)"
    if [[ -z "${_ctest}" ]]; then
      echo "[1q.sh test] ctest not found. Run: source scripts/win/activate_git_bash.sh && scripts/1q.sh doctor" >&2
      echo "  Wrappers exec Windows ctest.exe from Git Bash or WSL. Do not use Linux /usr/bin/ctest." >&2
      exit 127
    fi
    exec "${_ctest}" --preset "${preset}" "${extra[@]}"
    ;;
  install)
    [[ $# -ge 1 ]] || { usage; exit 2; }
    preset="$1"
    shift
    _bin="$(_oneq_binary_dir "${preset}")" || exit 1
    exec cmake --install "${_bin}" "$@"
    ;;
  clean-stale)
    [[ $# -eq 2 ]] || { usage; exit 2; }
    exec bash "${SCRIPT_DIR}/1q_clean_stale.sh" "$1" "$2"
    ;;
  doctor)
    doctor
    ;;
  ""|help|-h|--help)
    usage
    ;;
  *)
    echo "未知子命令: ${cmd}" >&2
    usage
    exit 2
    ;;
esac
