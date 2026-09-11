#!/usr/bin/env bash
# 1Q Git Bash 环境：补 PATH（cmake/ctest）。
# 由 scripts/win/activate_git_bash.sh、scripts/1q.sh、scripts/win/bin/cmake 等 source。
# 跨平台：Windows 侧负责补 cmake/ctest PATH，macOS/Linux 侧只在 cmake 不在 PATH 时兜底。
# 幂等：重复 source 安全。
#
# 机器级覆盖（可选，不进 git）：scripts/1q_env.local.sh
#   export ONEQ_CMAKE_ROOT='D:/environment/CMake'
#
# 历史：2026-08-21 前本文件还向 MINGW 会话注入 UCRTContentRoot，以绕开本机
# 64 位注册表 KitsRoot10 死路径；该注册表值已在机器级修复（两个视图均指向
# C:\Program Files (x86)\Windows Kits\10\），注入已退役，裸目录构建可用。

if [[ -n "${ONEQ_ENV_LOADED:-}" ]]; then
  return 0 2>/dev/null || exit 0
fi
ONEQ_ENV_LOADED=1

if [[ -z "${ONEQ_ROOT:-}" ]]; then
  ONEQ_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
fi
export ONEQ_ROOT

if [[ -f "${ONEQ_ROOT}/scripts/1q_env.local.sh" ]]; then
  # shellcheck source=/dev/null
  source "${ONEQ_ROOT}/scripts/1q_env.local.sh"
fi

_oneq_prepend_path_if_dir() {
  local dir="$1"
  [[ -d "$dir" ]] || return 0
  case ":${PATH}:" in
    *":${dir}:"*) ;;
    *) PATH="${dir}:${PATH}" ;;
  esac
}

_oneq_cmake_candidates() {
  local root="${ONEQ_ROOT}"
  local drive=""
  if [[ "${root}" == /mnt/?/* ]]; then
    drive="/mnt/${root:5:1}"
  elif [[ "${root}" == /?/* ]]; then
    drive="/${root:1:1}"
  fi

  cat <<EOF
${ONEQ_CMAKE_ROOT:+$ONEQ_CMAKE_ROOT/bin}
${drive}/environment/CMake/bin
/d/environment/CMake/bin
/mnt/d/environment/CMake/bin
/c/Program Files/CMake/bin
/c/Program Files (x86)/CMake/bin
EOF
}

_oneq_resolve_cmake_executable() {
  local dir candidate
  while IFS= read -r dir; do
    [[ -n "$dir" ]] || continue
    for candidate in "${dir}/cmake.exe" "${dir}/cmake"; do
      if [[ -x "$candidate" ]]; then
        export ONEQ_CMAKE_ROOT="${dir%/bin}"
        printf '%s' "$candidate"
        return 0
      fi
    done
  done < <(_oneq_cmake_candidates)
  return 1
}

if [[ -z "${ONEQ_REAL_CMAKE:-}" ]]; then
  if command -v cmake >/dev/null 2>&1; then
    _existing_cmake="$(command -v cmake)"
    case "${_existing_cmake}" in
      "${ONEQ_ROOT}/scripts/win/bin/cmake"|"${ONEQ_ROOT}/scripts/win/bin/cmake.exe")
        _resolved="$(_oneq_resolve_cmake_executable || true)"
        [[ -n "${_resolved:-}" ]] && export ONEQ_REAL_CMAKE="${_resolved}"
        ;;
      *)
        export ONEQ_REAL_CMAKE="${_existing_cmake}"
        ;;
    esac
  else
    _resolved="$(_oneq_resolve_cmake_executable || true)"
    [[ -n "${_resolved:-}" ]] && export ONEQ_REAL_CMAKE="${_resolved}"
  fi
fi

if [[ -n "${ONEQ_REAL_CMAKE:-}" ]]; then
  _oneq_prepend_path_if_dir "$(dirname "${ONEQ_REAL_CMAKE}")"
fi

_oneq_prepend_path_if_dir "${ONEQ_ROOT}/scripts/win/bin"

unset _existing_cmake _resolved _oneq_prepend_path_if_dir _oneq_resolve_cmake_executable _oneq_cmake_candidates
