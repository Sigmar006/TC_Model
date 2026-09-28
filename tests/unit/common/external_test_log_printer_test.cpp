#include "1q/logging/ExternalTestLogPrinter.h"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>

#include "gtest/gtest.h"

namespace {

std::string ReadExternalLog(const char* path) {
  std::ifstream stream(path, std::ios::in | std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(stream),
                     std::istreambuf_iterator<char>());
}

TEST(ExternalTestLogPrinterTest, PrintsThreePortableRecordKinds) {
  const char* path = "oneq_external_test_log_printer.log";
  std::remove(path);
  oneq::logging::ExternalTestLogPrinter printer(path);

  EXPECT_TRUE(printer.PrintBallisticData(1.25, 2U, "目标ID=1001 位置=(1,2,3)"));
  EXPECT_TRUE(printer.PrintSimulationControlClock(
      2.5, 3U, "期望执行时刻=2.500s 仿真步长=0.100s 应答状态=成功"));
  EXPECT_TRUE(printer.PrintRadarSimulationControlClock(
      3.75, 4U, "命令=步进 错误码=0 错误描述=无"));

  const std::string text = ReadExternalLog(path);
  EXPECT_NE(text.find("仿真时间=1.250s 仿真周期=2"), std::string::npos);
  EXPECT_NE(text.find("目标ID=1001 位置=(1,2,3)"), std::string::npos);
  EXPECT_NE(text.find("仿真时间=2.500s 仿真周期=3"), std::string::npos);
  EXPECT_NE(text.find("仿真步长=0.100s"), std::string::npos);
  EXPECT_NE(text.find("仿真时间=3.750s 仿真周期=4"), std::string::npos);
  EXPECT_NE(text.find("错误码=0"), std::string::npos);
  std::remove(path);
}

TEST(ExternalTestLogPrinterTest, RejectsEmptyPathAndKeepsEachRecordOnOneLine) {
  oneq::logging::ExternalTestLogPrinter no_path("");
  EXPECT_FALSE(no_path.PrintBallisticData(0.0, 0U, "content"));

  const char* path = "oneq_external_test_log_printer_single_line.log";
  std::remove(path);
  oneq::logging::ExternalTestLogPrinter printer(path);
  ASSERT_TRUE(printer.PrintBallisticData(0.0, 0U, "第一段\n第二段\r\n第三段"));

  const std::string text = ReadExternalLog(path);
  ASSERT_FALSE(text.empty());
  EXPECT_EQ(text.find('\r'), std::string::npos);
  EXPECT_EQ(text.find('\n'), text.size() - 1U);
  std::remove(path);
}

}  // namespace
