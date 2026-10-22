// 红外验收行语义守门用例（2026-10-21 红外日志体检）。
//
// 现场三类"0 值"其实不是同一个事实，却被写成同一个数：
//   - 最大探测距离 66% 写 0 m（目标不在本星宽场视场内）、6.4% 写死 mission.max_range_m；
//   - 检测标志在 SbirsPipeline.cpp 里写死"是"，与 SNR/视场门无关；
//   - 探测概率因门限 k=0.001 被当成归一化门限算 Pfa（≈0.4996）而恒 ≈0.5。
// 本用例锁住"如何区分"的判定，防止语义回退。

#include <gtest/gtest.h>

#include "sbirs_sensor/pipeline/SbirsAcceptanceSemantics.h"

namespace {

using sbirs_sensor::pipeline::ClassifyDetectionRange;
using sbirs_sensor::pipeline::DetectionFlagText;
using sbirs_sensor::pipeline::DetectionThresholdText;
using sbirs_sensor::pipeline::IsNoiseComponentModeled;
using sbirs_sensor::pipeline::SbirsDetectionRangeKind;

constexpr double kMissionMaxM = 5.0e7;

// 视场内且未被钳位 → 写物理值。
TEST(SbirsAcceptanceSemanticsTest, InFieldUnclampedIsApplicable) {
  EXPECT_EQ(ClassifyDetectionRange(1.2e7, 3.0e7, kMissionMaxM),
            SbirsDetectionRangeKind::kApplicable);
}

// 视场内几何为 0（目标不在宽场视场内）→ 不适用，不能写 0 m。
TEST(SbirsAcceptanceSemanticsTest, ZeroFovRangeIsOutOfFieldOfView) {
  EXPECT_EQ(ClassifyDetectionRange(1.2e7, 0.0, kMissionMaxM),
            SbirsDetectionRangeKind::kOutOfFieldOfView);
}

// 辐射链路不可解（τ_geo=0 等）→ 同样不适用，而不是"探测距离为 0"。
TEST(SbirsAcceptanceSemanticsTest, ZeroRadiometricRangeIsOutOfFieldOfView) {
  EXPECT_EQ(ClassifyDetectionRange(0.0, 8.0e7, kMissionMaxM),
            SbirsDetectionRangeKind::kOutOfFieldOfView);
}

// 反解值超过任务上限 → 标注"上限钳位"（现场 8,754 行恰为 5.0e7 m）。
TEST(SbirsAcceptanceSemanticsTest, RangeBeyondMissionCapIsClamped) {
  EXPECT_EQ(ClassifyDetectionRange(9.0e7, 5.0e7, kMissionMaxM),
            SbirsDetectionRangeKind::kClampedToMissionMax);
}

// 检测标志必须由两道门派生；任一不过即"否"。
TEST(SbirsAcceptanceSemanticsTest, DetectionFlagFollowsBothGates) {
  EXPECT_STREQ(DetectionFlagText(true, true), "是");
  EXPECT_STREQ(DetectionFlagText(true, false), "否");
  EXPECT_STREQ(DetectionFlagText(false, true), "否");
  EXPECT_STREQ(DetectionFlagText(false, false), "否");
}

// 门限 k<1 不是物理检测门限：必须显式标注，避免"门限=0.001 却全部通过"被当成正常。
TEST(SbirsAcceptanceSemanticsTest, SubUnityThresholdIsFlagged) {
  EXPECT_EQ(DetectionThresholdText(4.0f), "4.000000");
  const std::string low = DetectionThresholdText(0.001f);
  EXPECT_EQ(low.find("0.001000"), 0U);
  EXPECT_NE(low.find("低于k=1"), std::string::npos);
}

// 未配置的噪声分量必须写"未建模"，不能被当成 0 J。
TEST(SbirsAcceptanceSemanticsTest, UnconfiguredNoiseIsNotModeled) {
  EXPECT_FALSE(IsNoiseComponentModeled(0.0f));
  EXPECT_FALSE(IsNoiseComponentModeled(-1.0f));
  EXPECT_TRUE(IsNoiseComponentModeled(1.0e-15f));
}

}  // namespace
