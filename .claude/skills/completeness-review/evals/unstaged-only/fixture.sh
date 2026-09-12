#!/usr/bin/env bash
#
# case: unstaged-only —— 改动只在工作区，一行都没 stage。
#
# 中译：Step 0 用 `git diff --stat HEAD` 看改动（含未 stage 的工作区），而
# Step 0.5 的分类器只读 `git diff --cached`。两者口径不一致时，分类器必然返回
# {"tier": "empty"}，而 skill 通篇没有 empty 分支——本 case 就是来钉这个缺口的。
# 标识：eval-case-unstaged-only

set -euo pipefail

CASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$CASE_DIR/../lib/mini_repo.sh"

mini_repo_init
install_plan "$CASE_DIR/fixtures/plan.md"

# 一次实打实的核心库改动……但刻意不 stage。
cat > include/1q/airborne_radar/track_filter.h <<'EOF'
#pragma once

#include <cstddef>
#include <vector>

namespace oneq {
namespace airborne_radar {

class TrackFilter {
 public:
  explicit TrackFilter(std::size_t window);

  void Push(double value);
  double Smoothed() const;
  std::size_t Size() const;

 private:
  std::size_t window_;
  std::vector<double> samples_;
  std::size_t head_;
  std::size_t count_;
  double sum_;
};

}  // namespace airborne_radar
}  // namespace oneq
EOF

git add -A
git restore --staged .
