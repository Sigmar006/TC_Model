/**
 * @file common_acceptance_source_stage_test.cpp
 * @brief 验收行"来源"标注的阶段词表：词面固定，且滤波前后由标志位单点决定。
 */

#include <gtest/gtest.h>

#include <cstring>

#include "common/acceptance/AcceptanceSourceStage.h"

namespace oneq {
namespace common {
namespace acceptance {
namespace {

/// @brief 词表就是验收报告里要出现的四个阶段词，改动即破坏既有验收行口径。
TEST(AcceptanceSourceStageTest, UsesTheAgreedStageVocabulary) {
  EXPECT_STREQ(kStageBeforeDetection, "检测前");
  EXPECT_STREQ(kStageAfterDetection, "检测后");
  EXPECT_STREQ(kStageBeforeFilter, "滤波前");
  EXPECT_STREQ(kStageAfterFilter, "滤波后");
  EXPECT_STREQ(kStageAfterRecognition, "识别后");
}

/// @brief 运动量来源只有两种取值：滤波前（检测后点迹统计）或滤波后（滤波后验）。
TEST(AcceptanceSourceStageTest, FilterStageFollowsTheSourceFlag) {
  EXPECT_STREQ(FilterStageText(true), kStageBeforeFilter);
  EXPECT_STREQ(FilterStageText(false), kStageAfterFilter);
  EXPECT_STRNE(FilterStageText(true), FilterStageText(false));
}

}  // namespace
}  // namespace acceptance
}  // namespace common
}  // namespace oneq
