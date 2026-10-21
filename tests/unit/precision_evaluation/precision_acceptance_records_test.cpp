// 第26/27 项落盘回归：精度模块必须能独立写 SatelliteTarget 通道。
//
// 2026-10-21 现场：第26/27 项日志恒 0 行，而会话已跑通（ahp_valid=1、角度样本 2,386、
// 双星交会 959）。根因是精度模块借用推演模块 `InferenceAcceptanceRecords.h` 的头文件
// 内联包装，那层包装的宏闸门 `ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG` 只定义在
// target_inference_core 上，在 precision_evaluation_core 的编译单元里恒为关，两个写点
// 被编译成空语句（1q.lib 内 PrecisionAcceptanceRecords.obj 无 WriteAcceptanceLog 引用）。
// 本用例直接验证"调用写出接口后文件里确实有这一行"，通道闸门配置错误时必然失败。

#include "precision_evaluation/PrecisionAcceptanceRecords.h"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <map>
#include <string>

#include "common/logging/AcceptanceFileLog.h"
#include "gtest/gtest.h"

namespace {

const char* kPath = "oneq_precision_acceptance_test.log";

const oneq::logging::AcceptanceChannel kChannel =
    oneq::logging::AcceptanceChannel::kSatelliteTarget;

std::string ReadAll(const std::string& path) {
  std::ifstream in(path.c_str(), std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

class PrecisionAcceptanceRecordsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    oneq::logging::CloseAcceptanceLog(kChannel);
    std::remove(kPath);
    oneq::logging::OpenAcceptanceLog(kChannel, kPath);
  }

  void TearDown() override {
    oneq::logging::CloseAcceptanceLog(kChannel);
    std::remove(kPath);
  }

  std::string FlushAndRead() {
    oneq::logging::FlushAcceptanceLog(kChannel);
    oneq::logging::CloseAcceptanceLog(kChannel);
    return ReadAll(kPath);
  }
};

}  // namespace

// 第26 项：逐目标一行，字段按样本可用性省略（验收判定标准 第26项）。
TEST_F(PrecisionAcceptanceRecordsTest, KeyMetricsRowLandsOnSatelliteTargetChannel) {
  std::map<std::uint64_t, precision_evaluation::TargetKeyErrorSnapshot> latest;
  precision_evaluation::TargetKeyErrorSnapshot full;
  full.has_ecef = true;
  full.ecef_error_m[0] = 90.0;
  full.ecef_error_m[1] = -80.0;
  full.ecef_error_m[2] = 150.0;
  full.has_slant_range = true;
  full.slant_range_error_m = 180.0;
  full.has_angular = true;
  full.az_error_deg = 0.01;
  full.el_error_deg = 0.012;
  full.has_focal = true;
  full.focal_x_m = 0.0012;
  full.focal_y_m = -0.0008;
  latest[1001U] = full;

  precision_evaluation::TargetKeyErrorSnapshot partial;
  partial.has_slant_range = true;
  partial.slant_range_error_m = 12.5;
  latest[1002U] = partial;

  precision_evaluation::WritePrecisionKeyMetricsUnconditional(latest);
  const std::string content = FlushAndRead();

  ASSERT_FALSE(content.empty()) << "第26 项未写出任何行：写盘通道被编译期闸门吃掉了";
#if defined(_MSC_VER)
  EXPECT_NE(content.find(u8"[关键精度指标提取功能测试]"), std::string::npos);
  EXPECT_NE(content.find(u8"目标ID=1001"), std::string::npos);
  EXPECT_NE(content.find(u8"三轴位置误差ECEF=(90.0,-80.0,150.0)m"), std::string::npos);
  EXPECT_NE(content.find(u8"方位/俯仰误差=(0.0100,0.0120)°"), std::string::npos);
  EXPECT_NE(content.find(u8"脱靶量m=(0.001200,-0.000800)"), std::string::npos);
  EXPECT_NE(content.find(u8"目标ID=1002"), std::string::npos);
  EXPECT_NE(content.find(u8"距离误差=12.5m"), std::string::npos);
  // 无样本的字段整段省略：第二个目标没有 ECEF/角度/脱靶量样本。
  EXPECT_EQ(content.find(u8"目标ID=1002 三轴位置误差ECEF"), std::string::npos);
#else
  EXPECT_NE(content.find("[关键精度指标提取功能测试]"), std::string::npos);
  EXPECT_NE(content.find("目标ID=1001"), std::string::npos);
  EXPECT_NE(content.find("三轴位置误差ECEF=(90.0,-80.0,150.0)m"), std::string::npos);
  EXPECT_NE(content.find("方位/俯仰误差=(0.0100,0.0120)°"), std::string::npos);
  EXPECT_NE(content.find("脱靶量m=(0.001200,-0.000800)"), std::string::npos);
  EXPECT_NE(content.find("目标ID=1002"), std::string::npos);
  EXPECT_NE(content.find("距离误差=12.5m"), std::string::npos);
  EXPECT_EQ(content.find("目标ID=1002 三轴位置误差ECEF"), std::string::npos);
#endif
  // 第26 项只写误差本身，不写 RMSE/CEP 等派生统计。
  EXPECT_EQ(content.find("RMSE"), std::string::npos);
  EXPECT_EQ(content.find("CEP"), std::string::npos);
}

// 第27 项：权重/底层得分/综合分/等级；AHP 非法时整行省略。
TEST_F(PrecisionAcceptanceRecordsTest, AhpRowLandsOnSatelliteTargetChannel) {
  precision_evaluation::PrecisionEvaluationReport report;
  report.ahp_valid = true;
  report.ahp.weights[0] = 0.2;
  report.ahp.weights[1] = 0.2;
  report.ahp.weights[2] = 0.2;
  report.ahp.weights[3] = 0.2;
  report.ahp.weights[4] = 0.2;
  report.metric_scores[0] = 0.85;
  report.metric_scores[1] = 0.78;
  report.metric_scores[2] = 0.90;
  report.metric_scores[3] = 0.72;
  report.metric_scores[4] = 0.70;
  report.composite_score = 0.79;

  precision_evaluation::WritePrecisionAhpUnconditional(report);
  const std::string content = FlushAndRead();

  ASSERT_FALSE(content.empty()) << "第27 项未写出任何行：写盘通道被编译期闸门吃掉了";
#if defined(_MSC_VER)
  EXPECT_NE(content.find(u8"[层次分析法功能测试]"), std::string::npos);
  EXPECT_NE(content.find(u8"准则层权重=(0.200,0.200,0.200,0.200,0.200)"), std::string::npos);
  EXPECT_NE(content.find(u8"底层分=(0.850,0.780,0.900,0.720,0.700)"), std::string::npos);
  EXPECT_NE(content.find(u8"综合分=0.790"), std::string::npos);
  EXPECT_NE(content.find(u8"等级=B"), std::string::npos);
#else
  EXPECT_NE(content.find("[层次分析法功能测试]"), std::string::npos);
  EXPECT_NE(content.find("准则层权重=(0.200,0.200,0.200,0.200,0.200)"), std::string::npos);
  EXPECT_NE(content.find("底层分=(0.850,0.780,0.900,0.720,0.700)"), std::string::npos);
  EXPECT_NE(content.find("综合分=0.790"), std::string::npos);
  EXPECT_NE(content.find("等级=B"), std::string::npos);
#endif

  // AHP 未求解（判断矩阵非法）时不得写出半成品行：清掉上一行后重写应保持无文件。
  std::remove(kPath);
  precision_evaluation::PrecisionEvaluationReport invalid;
  invalid.ahp_valid = false;
  precision_evaluation::WritePrecisionAhpUnconditional(invalid);
  EXPECT_TRUE(FlushAndRead().empty());
}
