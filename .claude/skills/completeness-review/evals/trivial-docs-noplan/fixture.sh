#!/usr/bin/env bash
#
# case: trivial-docs-noplan —— 纯文档改动、且工作区里没有计划文件。
#
# 中译：一箭双雕——既验证 no-C++ 改动被降为 trivial 档且不触发构建，又验证
# 没有计划文件时 skill 如何取到评审锚点（Step 0 要求向用户要一句意图说明）。
# 标识：eval-case-trivial-docs

set -euo pipefail

CASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$CASE_DIR/../lib/mini_repo.sh"

mini_repo_init

cat >> docs/airborne_radar/algorithms.md <<'EOF'

## 术语

- **窗口（window）**：参与滑动平均的最近测量个数。
- **平滑值（smoothed value）**：当前窗口内样本的算术平均。
EOF

# 改动照常 stage，把"未 stage"这个变量留给 unstaged-only 那个 case 单独测。
stage_changes
