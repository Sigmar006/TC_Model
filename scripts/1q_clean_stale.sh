#!/usr/bin/env bash
# 删除指定 preset 下某模块的增量构建产物，强制下次全量重编该模块（修复 SEH/头文件布局变更后的脏 TU）。
#
# 用法：
#   scripts/1q_clean_stale.sh <configure_preset> <module_slug>
#
# module_slug 示例：remote_identification_radar | sbirs_sensor | airborne_radar | common
#
# 典型触发：public 头布局变更后 Unity/增量未重建全部 TU，Release/Debug 测试出现
# 0xc0000005 / bad_alloc 等与逻辑无关的崩溃（见 CLAUDE.md erratum 2026-08-17）。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/1q_env.sh
source "${SCRIPT_DIR}/lib/1q_env.sh"

usage() {
  cat >&2 <<'EOF'
用法: scripts/1q_clean_stale.sh <configure_preset> <module_slug>

示例:
  scripts/1q_clean_stale.sh VisualStudio.15.0-amd64 remote_identification_radar
  scripts/1q_clean_stale.sh VisualStudio.15.0-amd64 sbirs_sensor

module_slug: remote_identification_radar | airborne_radar | sbirs_sensor | common | ...
EOF
  exit 2
}

[[ $# -eq 2 ]] || usage

PRESET="$1"
MODULE="$2"

# binaryDir 由 preset 定义决定（两族 macOS preset 均继承 base 的
# ${sourceDir}/build/${presetName}），权威解析器是 lib/1q_preset_query.py。
# **不要在这里维护第二份 preset→目录映射表**，两份表迟早漂移。
_resolve_binary_dir() {
  local preset="$1" py out
  py="$(command -v python3 || command -v python || true)"
  if [[ -n "${py}" ]]; then
    if out="$("${py}" "${SCRIPT_DIR}/lib/1q_preset_query.py" "${preset}" binaryDir 2>/dev/null)" \
      && [[ -n "${out}" ]]; then
      printf '%s' "${out}"
      return 0
    fi
  fi
  echo "[1q_clean_stale] 提示：无法解析 preset「${preset}」的 binaryDir，回退到 build/${preset}" >&2
  printf '%s' "${ONEQ_ROOT}/build/${preset}"
}

BINARY_DIR="$(_resolve_binary_dir "${PRESET}")"

if [[ ! -d "${BINARY_DIR}" ]]; then
  echo "[1q_clean_stale] 跳过：binaryDir 不存在 ${BINARY_DIR}" >&2
  exit 0
fi

PRUNE_PATHS=()
if [[ -d "${BINARY_DIR}/src/${MODULE}" ]]; then
  PRUNE_PATHS+=("${BINARY_DIR}/src/${MODULE}")
fi
while IFS= read -r test_dir; do
  [[ -n "$test_dir" ]] && PRUNE_PATHS+=("$test_dir")
done < <(find "${BINARY_DIR}/tests" -maxdepth 2 -type d -iname "*${MODULE}*" 2>/dev/null || true)

if [[ ${#PRUNE_PATHS[@]} -eq 0 ]]; then
  echo "[1q_clean_stale] 未找到模块 '${MODULE}' 在 ${BINARY_DIR} 下的构建树" >&2
  exit 1
fi

echo "[1q_clean_stale] preset=${PRESET} module=${MODULE}"
for path in "${PRUNE_PATHS[@]}"; do
  echo "[1q_clean_stale] 删除 ${path}"
  rm -rf "${path}"
done

echo "[1q_clean_stale] 完成。请重新：scripts/1q.sh build ${PRESET}-release --target <tests>"
