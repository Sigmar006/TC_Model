#!/usr/bin/env bash
#
# completeness-review 评测套件的共享脚手架。
#
# 每个 case 的 fixture.sh 先 source 本文件，再调用下面的函数，在评测沙箱的
# 空工作区里搭一个 mini 版 1q 仓库：真 git 仓库 + 真分类器 + 一条基线提交，
# 之后该 case 只在基线上叠加自己的改动集。
#
# 只在 `claude plugin eval --scaffold` 时执行；运行身份是你本人、位置在 agent
# 的沙箱之外，所以 git init / git commit 都能正常落盘。
#
# 中译：本文件是评测用的假仓库搭建脚本，不参与产品构建，也不被库代码引用。
# 标识：eval-fixture-only

set -euo pipefail

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 建立基线仓库：一个已存在的机载雷达模块 + 分类器 + 工程约定文件。
# 中译：造出"改动之前"的状态，并提交一次，让后续 git diff 有东西可比。
mini_repo_init() {
  git init -q .
  git config user.email "eval@example.invalid"
  git config user.name "completeness-review eval"
  git config commit.gpgsign false

  mkdir -p src/airborne_radar include/1q/airborne_radar docs/airborne_radar \
           tests/unit/airborne_radar scripts .claude/plans

  # 计划文件不进 diff：真实仓库里 .claude/ 是本地目录。
  printf '.claude/\n' > .gitignore

  cat > CLAUDE.md <<'MD'
# Mini 1Q (eval fixture)

## Engineering Conventions

**Style**
- Write C++11-compatible C++.
- Follow the Google C++ Style Guide.
- Never introduce C++ exceptions.
- Keep changes in `src/` unless a public API change is unavoidable.

**Tests**
- New public API or significant logic changes come with new or updated tests under `tests/`.
MD
  ln -sf CLAUDE.md AGENTS.md

  cp "$LIB_DIR/pre-commit-review.py" scripts/pre-commit-review.py
  chmod +x scripts/pre-commit-review.py

  cat > include/1q/airborne_radar/track_filter.h <<'EOF'
#pragma once

#include <cstddef>

namespace oneq {
namespace airborne_radar {

// 航迹平滑滤波器：对最近的测量做滑动平均。
class TrackFilter {
 public:
  explicit TrackFilter(std::size_t window);

  void Push(double value);
  double Smoothed() const;
  std::size_t Size() const;

 private:
  std::size_t window_;
  double sum_;
  std::size_t count_;
};

}  // namespace airborne_radar
}  // namespace oneq
EOF

  cat > src/airborne_radar/track_filter.cpp <<'EOF'
#include "1q/airborne_radar/track_filter.h"

namespace oneq {
namespace airborne_radar {

TrackFilter::TrackFilter(std::size_t window)
    : window_(window), sum_(0.0), count_(0) {}

void TrackFilter::Push(double value) {
  sum_ += value;
  ++count_;
}

double TrackFilter::Smoothed() const {
  if (count_ == 0) {
    return 0.0;
  }
  return sum_ / static_cast<double>(count_);
}

std::size_t TrackFilter::Size() const { return count_; }

}  // namespace airborne_radar
}  // namespace oneq
EOF

  cat > tests/unit/airborne_radar/track_filter_test.cpp <<'EOF'
#include "1q/airborne_radar/track_filter.h"

#include <cassert>

int main() {
  oneq::airborne_radar::TrackFilter filter(3);
  assert(filter.Size() == 0);
  filter.Push(1.0);
  filter.Push(3.0);
  assert(filter.Smoothed() == 2.0);
  return 0;
}
EOF

  cat > docs/airborne_radar/algorithms.md <<'EOF'
# 机载雷达算法

## 航迹平滑

`TrackFilter` 对输入的测量序列做滑动平均，窗口大小由构造参数给定。
EOF

  git add -A
  git commit -qm "baseline: mini 1q airborne_radar slice"
}

# 把计划文件装进工作区。分类器只读索引区、计划文件被 .gitignore 排除，
# 所以计划不进改动集，只在 Step 0/2 被 skill 读到。
# 中译：把"这次改动的意图"交给被测 skill，让它有评审锚点。
install_plan() {
  mkdir -p .claude/plans
  cp "$1" ".claude/plans/$(basename "$1")"
}

# 把改动标记为待评审。分类器读 `git diff --cached`，未 stage 的改动它看不见——
# 这正是本套件要钉的口径差异之一，所以这里显式 stage。
stage_changes() {
  git add -A
}

# 落到非核心层的改动（examples/ tests/ docs/ tools/ cmake/）。
# 中译：制造"轻审"场景所需的改动集。
stage_example_layer_change() {
  mkdir -p examples/components
  cat > examples/components/track_filter_component.cpp <<'EOF'
#include "1q/airborne_radar/track_filter.h"

#include <cstdio>

// 集成方示例：把 TrackFilter 包成一个可调用的组件。
int main() {
  oneq::airborne_radar::TrackFilter filter(5);
  filter.Push(10.0);
  filter.Push(20.0);
  std::printf("smoothed=%.3f\n", filter.Smoothed());
  return 0;
}
EOF
  git add -A
}
