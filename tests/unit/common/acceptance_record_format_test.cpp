// 验收日志一行四段格式与文件 sink。

#include "common/logging/AcceptanceFileLog.h"
#include "common/logging/AcceptanceRecordFormat.h"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>

#include "gtest/gtest.h"

namespace {

const char* kTestPath = "oneq_acceptance_format_test.log";

class AcceptanceRecordFormatTest : public ::testing::Test {
 protected:
  void TearDown() override {
    oneq::logging::CloseAcceptanceLog(oneq::logging::AcceptanceChannel::kOpir);
    oneq::logging::CloseAcceptanceLog(
        oneq::logging::AcceptanceChannel::kIrInternalInterface);
    oneq::logging::CloseAcceptanceLog(
        oneq::logging::AcceptanceChannel::kRadarInternalInterface);
    oneq::logging::CloseAcceptanceLog(oneq::logging::AcceptanceChannel::kRadarOutput);
    std::remove(kTestPath);
    std::remove("oneq_ir_internal_interface_test.log");
    std::remove("oneq_radar_internal_interface_test.log");
    std::remove("oneq_radar_output_test.log");
  }
};

TEST_F(AcceptanceRecordFormatTest, FourFieldLineShape) {
  const std::string line =
      oneq::logging::FormatAcceptanceLine(1.0f, 1U, "最大探测距离", "目标ID=1001 相对卫星最大探测距离=4500000.0m");
  EXPECT_EQ(line,
            "仿真时间=1.000s 仿真周期=1 [最大探测距离] 目标ID=1001 "
            "相对卫星最大探测距离=4500000.0m");
}

TEST_F(AcceptanceRecordFormatTest, WriteAndReadFile) {
  oneq::logging::OpenAcceptanceLog(oneq::logging::AcceptanceChannel::kOpir, kTestPath);
  ASSERT_TRUE(oneq::logging::IsAcceptanceLogOpen(oneq::logging::AcceptanceChannel::kOpir));
  oneq::logging::WriteAcceptanceLog(
      oneq::logging::AcceptanceChannel::kOpir,
      oneq::logging::FormatAcceptanceLine(0.0f, 0U, "初始化时间", "见integration_events.log"));
  oneq::logging::FlushAcceptanceLog(oneq::logging::AcceptanceChannel::kOpir);
  oneq::logging::CloseAcceptanceLog(oneq::logging::AcceptanceChannel::kOpir);

  std::ifstream in(kTestPath, std::ios::binary);
  std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
#if defined(_MSC_VER)
  EXPECT_NE(content.find(u8"[初始化时间]"), std::string::npos);
  EXPECT_NE(content.find(u8"integration_events"), std::string::npos);
#else
  EXPECT_NE(content.find("[初始化时间]"), std::string::npos);
  EXPECT_NE(content.find("integration_events"), std::string::npos);
#endif
}

// 覆写语义：进程内首开截断旧文件（重跑进程不向归档证据追加重复行），
// 进程内重开追加（会话重启不丢已写行）。用独立通道 kFusion 保证本用例
// 拥有该 sink 的进程内首开。
TEST_F(AcceptanceRecordFormatTest, FirstOpenTruncatesThenReopenAppends) {
  const char* rerun_path = "oneq_acceptance_rerun_test.log";
  {
    std::ofstream stale(rerun_path, std::ios::binary);
    stale << "STALE_ROW_FROM_PREVIOUS_PROCESS\n";
  }
  const oneq::logging::AcceptanceChannel ch = oneq::logging::AcceptanceChannel::kFusion;
  oneq::logging::OpenAcceptanceLog(ch, rerun_path);
  oneq::logging::WriteAcceptanceLog(
      ch, oneq::logging::FormatAcceptanceLine(1.0f, 1U, "首开行", "A"));
  oneq::logging::CloseAcceptanceLog(ch);

  std::ifstream first(rerun_path, std::ios::binary);
  std::string after_first((std::istreambuf_iterator<char>(first)),
                          std::istreambuf_iterator<char>());
  EXPECT_EQ(after_first.find("STALE_ROW_FROM_PREVIOUS_PROCESS"), std::string::npos);
#if defined(_MSC_VER)
  EXPECT_NE(after_first.find(u8"[首开行]"), std::string::npos);
#else
  EXPECT_NE(after_first.find("[首开行]"), std::string::npos);
#endif

  oneq::logging::OpenAcceptanceLog(ch, rerun_path);
  oneq::logging::WriteAcceptanceLog(
      ch, oneq::logging::FormatAcceptanceLine(2.0f, 2U, "重开行", "B"));
  oneq::logging::CloseAcceptanceLog(ch);

  std::ifstream second(rerun_path, std::ios::binary);
  std::string after_reopen((std::istreambuf_iterator<char>(second)),
                           std::istreambuf_iterator<char>());
#if defined(_MSC_VER)
  EXPECT_NE(after_reopen.find(u8"[首开行]"), std::string::npos);
  EXPECT_NE(after_reopen.find(u8"[重开行]"), std::string::npos);
#else
  EXPECT_NE(after_reopen.find("[首开行]"), std::string::npos);
  EXPECT_NE(after_reopen.find("[重开行]"), std::string::npos);
#endif
  std::remove(rerun_path);
}

TEST_F(AcceptanceRecordFormatTest, InternalInterfaceChannelsWriteSeparateFiles) {
  const char* ir_path = "oneq_ir_internal_interface_test.log";
  const char* radar_path = "oneq_radar_internal_interface_test.log";
  const oneq::logging::AcceptanceChannel ir_channel =
      oneq::logging::AcceptanceChannel::kIrInternalInterface;
  const oneq::logging::AcceptanceChannel radar_channel =
      oneq::logging::AcceptanceChannel::kRadarInternalInterface;

  oneq::logging::OpenAcceptanceLog(ir_channel, ir_path);
  oneq::logging::OpenAcceptanceLog(radar_channel, radar_path);
  oneq::logging::WriteAcceptanceLog(
      ir_channel, oneq::logging::FormatAcceptanceLine(
                      1.0f, 1U, "红外内部接口", "目标ID=1001"));
  oneq::logging::WriteAcceptanceLog(
      radar_channel, oneq::logging::FormatAcceptanceLine(
                         1.0f, 1U, "雷达内部接口", "目标ID=2001"));
  oneq::logging::CloseAcceptanceLog(ir_channel);
  oneq::logging::CloseAcceptanceLog(radar_channel);

  std::ifstream ir_stream(ir_path, std::ios::binary);
  std::ifstream radar_stream(radar_path, std::ios::binary);
  const std::string ir_text((std::istreambuf_iterator<char>(ir_stream)),
                            std::istreambuf_iterator<char>());
  const std::string radar_text((std::istreambuf_iterator<char>(radar_stream)),
                               std::istreambuf_iterator<char>());
#if defined(_MSC_VER)
  EXPECT_NE(ir_text.find(u8"[红外内部接口]"), std::string::npos);
  EXPECT_EQ(ir_text.find(u8"[雷达内部接口]"), std::string::npos);
  EXPECT_NE(radar_text.find(u8"[雷达内部接口]"), std::string::npos);
  EXPECT_EQ(radar_text.find(u8"[红外内部接口]"), std::string::npos);
#else
  EXPECT_NE(ir_text.find("[红外内部接口]"), std::string::npos);
  EXPECT_EQ(ir_text.find("[雷达内部接口]"), std::string::npos);
  EXPECT_NE(radar_text.find("[雷达内部接口]"), std::string::npos);
  EXPECT_EQ(radar_text.find("[红外内部接口]"), std::string::npos);
#endif
}

TEST_F(AcceptanceRecordFormatTest, RadarOutputUsesDedicatedFile) {
  const char* path = "oneq_radar_output_test.log";
  const oneq::logging::AcceptanceChannel channel =
      oneq::logging::AcceptanceChannel::kRadarOutput;
  oneq::logging::OpenAcceptanceLog(channel, path);
  oneq::logging::WriteAcceptanceLog(
      channel, oneq::logging::FormatAcceptanceLine(
                   2.0f, 2U, "雷达检测与量测输出接口测试", "量测数量=1"));
  oneq::logging::CloseAcceptanceLog(channel);

  std::ifstream stream(path, std::ios::binary);
  const std::string content((std::istreambuf_iterator<char>(stream)),
                            std::istreambuf_iterator<char>());
#if defined(_MSC_VER)
  EXPECT_NE(content.find(u8"[雷达检测与量测输出接口测试]"), std::string::npos);
#else
  EXPECT_NE(content.find("[雷达检测与量测输出接口测试]"), std::string::npos);
#endif
  EXPECT_NE(content.find("量测数量=1"), std::string::npos);
}

}  // namespace
