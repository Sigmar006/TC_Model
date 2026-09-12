#!/usr/bin/env bash
#
# case: blocked-no-merge —— 计划未完成，Gate A 应判 BLOCKED 并停住。
#
# 中译：改动集里只有滑动窗口是真正做完的，Reset 是空壳、文档一行没动，
# 所以计划覆盖必然是 一项完成 / 一项残缺 / 一项缺失，门禁必须拦下。
# 标识：eval-case-blocked-no-merge

set -euo pipefail

CASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$CASE_DIR/../lib/mini_repo.sh"

mini_repo_init
install_plan "$CASE_DIR/fixtures/plan.md"

cat > include/1q/airborne_radar/track_filter.h <<'EOF'
#pragma once

#include <cstddef>
#include <vector>

namespace oneq {
namespace airborne_radar {

// 航迹平滑滤波器：对最近 window 次测量做滑动平均。
class TrackFilter {
 public:
  explicit TrackFilter(std::size_t window);

  void Push(double value);
  double Smoothed() const;
  std::size_t Size() const;
  void Reset();

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

cat > src/airborne_radar/track_filter.cpp <<'EOF'
#include "1q/airborne_radar/track_filter.h"

namespace oneq {
namespace airborne_radar {

TrackFilter::TrackFilter(std::size_t window)
    : window_(window),
      samples_(window, 0.0),
      head_(0),
      count_(0),
      sum_(0.0) {}

void TrackFilter::Push(double value) {
  if (window_ == 0) {
    return;
  }
  if (count_ == window_) {
    // 窗口已满：先淘汰最旧的样本，再写入新样本。
    sum_ -= samples_[head_];
  } else {
    ++count_;
  }
  samples_[head_] = value;
  sum_ += value;
  head_ = (head_ + 1) % window_;
}

double TrackFilter::Smoothed() const {
  if (count_ == 0) {
    return 0.0;
  }
  return sum_ / static_cast<double>(count_);
}

std::size_t TrackFilter::Size() const { return count_; }

void TrackFilter::Reset() {
  // TODO: 清空窗口状态并复位累加器。
}

}  // namespace airborne_radar
}  // namespace oneq
EOF

# 计划第 3 项（补文档）完全没有动。
stage_changes
