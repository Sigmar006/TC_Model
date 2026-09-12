#!/usr/bin/env bash
#
# case: minor-examples —— 改动只在示例层，应判 tier = minor、走轻审。
#
# 中译：造出"只动 examples/、不碰核心库"的改动集，验证 skill 是否按风险降档，
# 而不是对示例代码也上一整套三车道评审。
# 标识：eval-case-minor-examples

set -euo pipefail

CASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$CASE_DIR/../lib/mini_repo.sh"

mini_repo_init
install_plan "$CASE_DIR/fixtures/plan.md"
stage_example_layer_change
