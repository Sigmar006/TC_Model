/**
 * @file precision_evaluation_metrics_boundary_test.cpp
 * @brief 精度评估纯函数面的对抗性边界测试：误差序列汇总、归一化评分、综合分合成。
 *
 * 断言依据（全部来自规格与公开头注释，未参考实现）：
 * - docs/precision_evaluation/algorithms.md「公式与口径」「编排会话与日志事件」
 * - docs/precision_evaluation/boundaries.md「输入输出边界」（纯算法面无状态、确定性）
 * - include/1q/precision_evaluation/PrecisionEvaluationMetrics.h
 * - include/1q/precision_evaluation/AhpEvaluator.h
 * 每个期望值均由上式手工推导，注释给出算术过程。
 */
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "rapidcheck.h"

#include "1q/precision_evaluation/AhpEvaluator.h"
#include "1q/precision_evaluation/PrecisionEvaluationMetrics.h"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"
#include "support/rapidcheck_gtest.h"

namespace {

namespace pe = precision_evaluation;
using pe::kPrecisionMetricCount;

// 容差依据：误差样本量级 ≤1e4、样本数 ≤100，朴素求和在 double 下的绝对误差上界约
// (n−1)·eps·Σx ≈ 100·2.2e-16·1e6 ≈ 2.2e-8；取 1e-6 留约 45 倍余量。
// 该容差只用于「同一多重集、不同求和次序」的比较，不用于掩盖内容错误。
constexpr double kOrderInvarianceTolerance = 1.0e-6;

// ---------------------------------------------------------------------------
// 误差序列汇总 SummarizeErrorSeries
// ---------------------------------------------------------------------------

// P95 必须是「排序后」的最近秩，与输入次序无关。
// 依据：PrecisionEvaluationMetrics.h「P95 取升序排序后的 ceil(0.95·n)−1 下标（最近秩法）」；
// algorithms.md「P95：升序第 ceil(0.95·n) 个样本（1 基最近秩）」。
// 输入刻意乱序：若实现按下标直取（漏排序），升序输入无法暴露，乱序输入会立刻失败。
TEST(PrecisionMetricsBoundaryTest, P95IsOrderInvariantNearestRank) {
  const std::vector<double> errors = {20.0, 1.0, 19.0, 2.0, 18.0, 3.0, 17.0, 4.0, 16.0, 5.0,
                                      15.0, 6.0, 14.0, 7.0, 13.0, 8.0, 12.0, 9.0, 11.0, 10.0};
  const pe::ErrorMetricSummary first = pe::SummarizeErrorSeries(errors);

  // n=20 ⇒ ceil(0.95·20)=19 ⇒ 0 基下标 18 ⇒ 升序 1..20 的第 19 个样本 = 19（不是最大值 20）。
  EXPECT_EQ(first.count, 20U);
  // 均值：Σk(k=1..20)=210，210/20=10.5（精确）。
  EXPECT_DOUBLE_EQ(first.mean, 10.5);
  // RMSE：Σk²(k=1..20)=2870，2870/20=143.5；sqrt(143.5) 无理，
  // 容差依据：被开方数量级 1e2，double 相对精度 2.2e-16 ⇒ 结果绝对误差 ~2e-14，取 1e-12。
  EXPECT_NEAR(first.rmse, std::sqrt(143.5), 1.0e-12);
  EXPECT_DOUBLE_EQ(first.p95, 19.0);
  EXPECT_DOUBLE_EQ(first.max, 20.0);

  // 变异探针已知 survivor（等价变异体，非测试缺口，勿重复调查）：
  //   arith_sub @ PrecisionEvaluationMetrics.cpp:26  `size() - 1U` -> `size() + 1U`
  //   p95_index = min(n-1, ceil(0.95·n)-1)，而 ceil(0.95·n) ≤ n 恒成立，
  //   故第二参恒 ≤ 第一参，min 从不取到 size()-1U；改大它结果不变。
  //   唯一可能区分的 n=0 已被函数开头的 errors.empty() 提前返回挡掉。

  // 确定性：boundaries.md 声明纯算法面「无状态、确定性」⇒ 同输入两次调用逐位一致。
  const pe::ErrorMetricSummary second = pe::SummarizeErrorSeries(errors);
  EXPECT_EQ(second.count, first.count);
  EXPECT_DOUBLE_EQ(second.mean, first.mean);
  EXPECT_DOUBLE_EQ(second.rmse, first.rmse);
  EXPECT_DOUBLE_EQ(second.p95, first.p95);
  EXPECT_DOUBLE_EQ(second.max, first.max);
}

// 空序列的四个统计量必须全零——不得给出任何「看起来像满分」的默认值。
// 依据：PrecisionEvaluationMetrics.h「空序列返回全零（count=0）」；algorithms.md
// 「零证据=零分」：空序列汇总若给出非零默认值，编排层按 rmse 归一化时会静默得满分。
TEST(PrecisionMetricsBoundaryTest, EmptySeriesIsAllZero) {
  const pe::ErrorMetricSummary summary = pe::SummarizeErrorSeries(std::vector<double>{});
  EXPECT_EQ(summary.count, 0U);
  EXPECT_DOUBLE_EQ(summary.mean, 0.0);
  EXPECT_DOUBLE_EQ(summary.rmse, 0.0);
  EXPECT_DOUBLE_EQ(summary.p95, 0.0);
  EXPECT_DOUBLE_EQ(summary.max, 0.0);
}

// 单样本退化：n=1 ⇒ ceil(0.95·1)=1 ⇒ 下标 0 ⇒ P95 即该样本。
// 依据：PrecisionEvaluationMetrics.h「序列短时退化为最大值」；rmse=sqrt(x²/1)=|x|。
TEST(PrecisionMetricsBoundaryTest, SingleSampleDegeneratesToThatSample) {
  const pe::ErrorMetricSummary summary = pe::SummarizeErrorSeries(std::vector<double>{7.5});
  EXPECT_EQ(summary.count, 1U);
  EXPECT_DOUBLE_EQ(summary.mean, 7.5);
  // 7.5 = 15/2 精确可表示 ⇒ 7.5² = 56.25 精确，sqrt(56.25) = 7.5 精确（正确舍入）。
  EXPECT_DOUBLE_EQ(summary.rmse, 7.5);
  EXPECT_DOUBLE_EQ(summary.p95, 7.5);
  EXPECT_DOUBLE_EQ(summary.max, 7.5);
}

// 偏斜批次：P95 退化为最大值，且 RMSE > 均值（离群样本对 RMSE 的放大）。
// 依据：mean=Σx/n、rmse=sqrt(Σx²/n)（algorithms.md 误差序列汇总）；
// 9 个 0 + 1 个 100 ⇒ mean=100/10=10，rmse=sqrt(10000/10)=sqrt(1000)。
// 该用例说明 P95 在此形状下不可当作「典型误差」，必须与 mean/RMSE 分开看。
TEST(PrecisionMetricsBoundaryTest, SkewedBatchPutsP95AtMaximum) {
  std::vector<double> errors(9U, 0.0);
  errors.push_back(100.0);
  const pe::ErrorMetricSummary summary = pe::SummarizeErrorSeries(errors);
  EXPECT_EQ(summary.count, 10U);
  EXPECT_DOUBLE_EQ(summary.mean, 10.0);
  // sqrt(1000) = 31.622776601683793…；容差依据：量级 3e1，相对精度 2.2e-16 ⇒ 绝对误差 ~7e-15。
  EXPECT_NEAR(summary.rmse, 31.622776601683793, 1.0e-12);
  EXPECT_DOUBLE_EQ(summary.p95, 100.0);  // ceil(0.95·10)=10 ⇒ 下标 9
  EXPECT_DOUBLE_EQ(summary.max, 100.0);
  EXPECT_GT(summary.rmse, summary.mean);  // 方差非负 ⇒ rmse ≥ |mean|
}

// ---------------------------------------------------------------------------
// 归一化评分 NormalizeErrorScore
// ---------------------------------------------------------------------------

// 非有限 / 负数 / 非正参考误差一律记 0；有效区间内对 rmse 严格单调递减。
// 依据：PrecisionEvaluationMetrics.h「score = ref/(ref+rmse)」「ref≤0、rmse<0 或 rmse
// 非有限（含 +∞，评估侧以 +∞ 表示零证据指标）时返回 0」；algorithms.md「零证据=零分」。
TEST(PrecisionNormalizeBoundaryTest, NonFiniteNegativeAndMonotonicity) {
  const double kReferenceDeg = 0.05;
  // +∞ 是零证据指标的入口值（count=0 的指标 rmse 按 +∞ 进评分），必须得 0 而不是 NaN。
  EXPECT_DOUBLE_EQ(
      pe::NormalizeErrorScore(std::numeric_limits<double>::infinity(), kReferenceDeg), 0.0);
  // NaN 属「非有限」，同样必须显式记 0：否则 NaN 会经 Σw·score 污染综合分。
  EXPECT_DOUBLE_EQ(
      pe::NormalizeErrorScore(std::numeric_limits<double>::quiet_NaN(), kReferenceDeg), 0.0);
  EXPECT_DOUBLE_EQ(
      pe::NormalizeErrorScore(-std::numeric_limits<double>::infinity(), kReferenceDeg), 0.0);
  EXPECT_DOUBLE_EQ(pe::NormalizeErrorScore(-1.0, kReferenceDeg), 0.0);
  // ref ≤ 0：含 0 与负值（避免 ref/(ref+rmse) 在 ref=0 时退化为 1 或 NaN）。
  EXPECT_DOUBLE_EQ(pe::NormalizeErrorScore(1.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(pe::NormalizeErrorScore(1.0, -2.0), 0.0);

  // 单调性手算（ref=2）：score(0.5)=2/2.5=0.8、score(2)=2/4=0.5、score(8)=2/10=0.2。
  const double kRmse[] = {0.5, 2.0, 8.0};
  const double kExpected[] = {0.8, 0.5, 0.2};
  for (std::size_t index = 0U; index < 3U; ++index) {
    SCOPED_TRACE(::testing::Message() << "index=" << index << " rmse=" << kRmse[index]);
    const double score = pe::NormalizeErrorScore(kRmse[index], 2.0);
    EXPECT_DOUBLE_EQ(score, kExpected[index]);
    // 头文件声明返回 ∈ [0,1)；此处仅对有限有效输入断言该区间（rmse=0 的端点语义见报告）。
    EXPECT_GE(score, 0.0);
    EXPECT_LT(score, 1.0);
    if (index > 0U) {
      EXPECT_LT(score, pe::NormalizeErrorScore(kRmse[index - 1U], 2.0));
    }
  }
}

// ---------------------------------------------------------------------------
// 综合分合成 ComposePrecisionScore
// ---------------------------------------------------------------------------

// 零证据指标（rmse=+∞）贡献 0 分、拖低综合分，且不污染其余指标。
// 依据：algorithms.md「零证据=零分：count=0 的指标 rmse 按 +∞ 进评分，得 0 分拖低综合，
// 不因空序列 rmse=0 而得满分」；AhpEvaluator.h「composite = Σ w_i·ref_i/(ref_i+rmse_i)」
// 「贡献分解 w_i·score_i 之和恒等于综合分」。
// 手算：w_i=0.2、ref_i=10，前四项 rmse=10 ⇒ 10/20=0.5，末项 rmse=+∞ ⇒ 0；
// composite = 4×0.2×0.5 = 0.4（若末项被当作 0 误差则会是 0.5，差异可辨）。
TEST(PrecisionScoreComposeBoundaryTest, ZeroEvidenceMetricContributesZero) {
  const double weights[kPrecisionMetricCount] = {0.2, 0.2, 0.2, 0.2, 0.2};
  const double rmse[kPrecisionMetricCount] = {10.0, 10.0, 10.0, 10.0,
                                              std::numeric_limits<double>::infinity()};
  const double references[kPrecisionMetricCount] = {10.0, 10.0, 10.0, 10.0, 10.0};
  pe::PrecisionEvaluationReport report;
  pe::ComposePrecisionScore(weights, rmse, references, &report);

  // 零证据指标的贡献必须为 0（+∞ 若被当作 0 误差，这里会是 0.1）。
  EXPECT_DOUBLE_EQ(report.metric_scores[4U], 0.0);
  EXPECT_DOUBLE_EQ(report.metric_contributions[4U], 0.0);
  double contribution_sum = 0.0;
  for (std::size_t index = 0U; index < kPrecisionMetricCount; ++index) {
    SCOPED_TRACE(::testing::Message() << "index=" << index);
    if (index < 4U) {
      EXPECT_DOUBLE_EQ(report.metric_scores[index], 0.5);  // 10/(10+10)
    }
    // 贡献分解定义：contribution_i = w_i · score_i（AhpEvaluator.h「贡献分解」）。
    EXPECT_DOUBLE_EQ(report.metric_contributions[index],
                     weights[index] * report.metric_scores[index]);
    // 参考误差必须透传（配置口径可在报告里复核）。
    EXPECT_DOUBLE_EQ(report.reference_errors[index], references[index]);
    contribution_sum += report.metric_contributions[index];
  }
  // 容差依据：5 项各 ~0.1 量级求和，double 相对精度 2.2e-16 ⇒ 绝对误差 ~1e-16，取 1e-12。
  EXPECT_NEAR(report.composite_score, 0.4, 1.0e-12);
  EXPECT_NEAR(contribution_sum, report.composite_score, 1.0e-12);
  EXPECT_GE(report.composite_score, 0.0);
  EXPECT_LE(report.composite_score, 1.0);
}

// 非正参考误差按 0 分计，且不得以 0/0 的 NaN 污染综合分与其余指标。
// 依据：AhpEvaluator.h「reference_errors …（≤0 的项得分记 0）」。
// 手算：w_i=0.2，ref=(0,10,10,10,10)，rmse=(0,10,10,10,10) ⇒ 第 0 项 0 分、其余 10/20=0.5；
// composite = 4×0.2×0.5 = 0.4。
TEST(PrecisionScoreComposeBoundaryTest, NonPositiveReferenceScoresZeroWithoutNaN) {
  const double weights[kPrecisionMetricCount] = {0.2, 0.2, 0.2, 0.2, 0.2};
  const double rmse[kPrecisionMetricCount] = {0.0, 10.0, 10.0, 10.0, 10.0};
  const double references[kPrecisionMetricCount] = {0.0, 10.0, 10.0, 10.0, 10.0};
  pe::PrecisionEvaluationReport report;
  pe::ComposePrecisionScore(weights, rmse, references, &report);

  EXPECT_DOUBLE_EQ(report.metric_scores[0U], 0.0);
  EXPECT_TRUE(std::isfinite(report.composite_score));
  for (std::size_t index = 1U; index < kPrecisionMetricCount; ++index) {
    SCOPED_TRACE(::testing::Message() << "index=" << index);
    EXPECT_DOUBLE_EQ(report.metric_scores[index], 0.5);
  }
  EXPECT_NEAR(report.composite_score, 0.4, 1.0e-12);
}

// 单个非有限参考误差不得静默污染综合分与其余指标。
// 依据：PrecisionEvaluationMetrics.h 明确规定「ref 非正或非有限（NaN/±∞）…
// 时返回 0」，PrecisionEvaluationTypes.h 声明 composite_score「∈ [0,1]（越大越好）」
// 为无条件字段契约，boundaries.md「非目标 4」要求不静默退化；AhpEvaluator.h
// 「≤0 的项得分记 0」给出同类不可用参考误差的既有处理方式。
// 手算：w_i=0.2、ref=(NaN,10,10,10,10)、rmse 全 10 ⇒ 第 0 项 0 分、其余 10/20=0.5；
// composite = 4×0.2×0.5 = 0.4（若第 0 项静默变 NaN，综合分即 NaN）。
TEST(PrecisionScoreComposeBoundaryTest, NonFiniteReferenceDoesNotPoisonCompositeRange) {
  const double weights[kPrecisionMetricCount] = {0.2, 0.2, 0.2, 0.2, 0.2};
  const double rmse[kPrecisionMetricCount] = {10.0, 10.0, 10.0, 10.0, 10.0};
  const double references[kPrecisionMetricCount] = {std::numeric_limits<double>::quiet_NaN(),
                                                    10.0, 10.0, 10.0, 10.0};
  pe::PrecisionEvaluationReport report;
  pe::ComposePrecisionScore(weights, rmse, references, &report);

  // 参考误差不可用 ⇒ 该项记 0，与 ref≤0 同类；不得因 NaN 比较恒假而漏网。
  EXPECT_DOUBLE_EQ(report.metric_scores[0U], 0.0);
  for (std::size_t index = 1U; index < kPrecisionMetricCount; ++index) {
    SCOPED_TRACE(::testing::Message() << "index=" << index);
    // 这些指标的量测与参考误差均有效，得分必须是 10/(10+10)=0.5，不受第 0 项影响。
    EXPECT_DOUBLE_EQ(report.metric_scores[index], 0.5);
  }
  EXPECT_NEAR(report.composite_score, 0.4, 1.0e-12);
  // 文档区间契约：越界或 NaN 都意味着一个配置项静默毁掉了整份报告。
  EXPECT_TRUE(std::isfinite(report.composite_score));
  EXPECT_GE(report.composite_score, 0.0);
  EXPECT_LE(report.composite_score, 1.0);
}

// ---------------------------------------------------------------------------
// 属性：汇总统计在非负误差序列上的不变量
// ---------------------------------------------------------------------------

// 属性：非负有限误差序列（量级 ≤1e4）的汇总满足四条不变量——与输入次序无关、count 等于
// 样本数、各统计量被最大值界定、RMSE ≥ 均值。
// 依据：PrecisionEvaluationMetrics.h「errors 误差样本序列（非负）」「count 样本数」；
// mean=Σx/n、rmse=sqrt(Σx²/n) ⇒ rmse² − mean² = Σ(x−mean)²/n ≥ 0（方差非负），
// 且 x ≤ max ⇒ rmse = sqrt(mean of x²) ≤ max。
// 输入构造：把生成的 double 映射到 [0, 1e4) 的非负值，而不是用 RC_PRE 过滤——
// 过滤会让丢弃率超过 max_discard_ratio 而报「Gave up」而非检验不变量。
// 量级上界 1e4 是刻意的：不设界时 Σx² 会在 double 下溢出为 +∞，
// 让不变量平凡成立而不是被真正检验。
RC_GTEST_PROP(PrecisionSummaryPropertyTest, SummaryIsOrderInvariantAndMaxBounded,
              (const std::vector<double>& raw)) {
  std::vector<double> errors;
  errors.reserve(raw.size());
  for (const double value : raw) {
    // fmod(|v|, 1e4) 有限且落在 [0, 1e4)；非有限输入（arbitrary<double> 不产生，
    // 但保证映射是全域的）映射为 0。误差样本按定义非负，故此处取模长。
    const double magnitude = std::fmod(std::abs(value), 1.0e4);
    errors.push_back(std::isfinite(magnitude) ? magnitude : 0.0);
  }

  const pe::ErrorMetricSummary forward = pe::SummarizeErrorSeries(errors);
  const std::vector<double> reversed(errors.rbegin(), errors.rend());
  const pe::ErrorMetricSummary backward = pe::SummarizeErrorSeries(reversed);

  RC_ASSERT(forward.count == errors.size());
  RC_ASSERT(backward.count == errors.size());
  RC_ASSERT(std::abs(forward.mean - backward.mean) <= kOrderInvarianceTolerance);
  RC_ASSERT(std::abs(forward.rmse - backward.rmse) <= kOrderInvarianceTolerance);
  RC_ASSERT(std::abs(forward.p95 - backward.p95) <= kOrderInvarianceTolerance);
  RC_ASSERT(std::abs(forward.max - backward.max) <= kOrderInvarianceTolerance);

  if (!errors.empty()) {
    RC_ASSERT(forward.mean <= forward.max);
    RC_ASSERT(forward.rmse <= forward.max);
    RC_ASSERT(forward.p95 <= forward.max);
    RC_ASSERT(forward.rmse >= forward.mean);
  }
}

}  // namespace
