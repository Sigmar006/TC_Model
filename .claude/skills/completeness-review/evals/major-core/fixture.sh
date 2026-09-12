#!/usr/bin/env bash
#
# case: major-core —— 核心库改动，应判 tier = major。
#
# 中译：造出"src/ 与 include/1q/ 下的实质改动"，用来验证 skill 的风险分级与
# 深度选择是否正确落到 major / 全量评审。
# 标识：eval-case-major-core

set -euo pipefail

CASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$CASE_DIR/../lib/mini_repo.sh"

mini_repo_init
install_plan "$CASE_DIR/fixtures/plan.md"

# 核心库改动：include/1q + src/ + 新增一个核心头文件。
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

cat > include/1q/airborne_radar/track_filter_config.h <<'EOF'
#pragma once

#include <cstddef>

namespace oneq {
namespace airborne_radar {

// 航迹滤波器的默认参数。
struct TrackFilterConfig {
  std::size_t window;
  bool reject_non_finite;

  TrackFilterConfig() : window(5), reject_non_finite(true) {}
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
  samples_.assign(window_, 0.0);
  head_ = 0;
  count_ = 0;
  sum_ = 0.0;
}

}  // namespace airborne_radar
}  // namespace oneq
EOF

cat > tests/unit/airborne_radar/track_filter_test.cpp <<'EOF'
#include "1q/airborne_radar/track_filter.h"

#include <cassert>

int main() {
  oneq::airborne_radar::TrackFilter filter(3);

  // 窗口未满：按已收到的样本数求平均。
  filter.Push(1.0);
  filter.Push(3.0);
  assert(filter.Size() == 2);
  assert(filter.Smoothed() == 2.0);

  // 窗口满：最旧的样本被淘汰。
  filter.Push(6.0);
  filter.Push(10.0);
  assert(filter.Size() == 3);
  assert(filter.Smoothed() == (3.0 + 6.0 + 10.0) / 3.0);

  filter.Reset();
  assert(filter.Size() == 0);
  assert(filter.Smoothed() == 0.0);
  return 0;
}
EOF

cat >> docs/airborne_radar/algorithms.md <<'EOF'

## 窗口语义

`TrackFilter` 只保留最近 `window` 次测量：窗口未满时对已收到的样本求平均，
满后每来一次新测量就淘汰最旧的样本。`Reset()` 清空窗口状态。
EOF

stage_changes
