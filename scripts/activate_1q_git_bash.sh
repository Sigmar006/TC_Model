#!/usr/bin/env bash
# 转发壳（历史路径）——保留原因：用户 Windows 机器的 ~/.bashrc 里写着
#   source "/d/1q/1q/scripts/activate_1q_git_bash.sh" 2>/dev/null
# 那句带 2>/dev/null，路径失效是**静默**的：新终端会没有 cmake PATH、没有 1q 命令、
# core.hooksPath 也不自愈。真实实现已移到 scripts/win/activate_git_bash.sh。
#
# 迁移：各 Windows 机器把 ~/.bashrc 里那行改成 .../scripts/win/activate_git_bash.sh，
# 确认全部迁移完成后删除本文件（它是唯一的兼容遗留，不承载任何逻辑）。
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_target="${_SCRIPT_DIR}/win/activate_git_bash.sh"
if [[ -r "${_target}" ]]; then
  # shellcheck source=win/activate_git_bash.sh
  source "${_target}" "$@"
else
  # 不用 `source` 直接失败：那样只会打一行 bash 原生错误，且退出码仍为 0。
  echo "[1q] 转发目标缺失：${_target}（本壳应已随 scripts/win/ 迁移一并删除）" >&2
fi
unset _target
