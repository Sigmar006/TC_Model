// 验收日志一行四段格式、文件 sink 与六类目标文件落盘。

#include "common/logging/AcceptanceFileLog.h"
#include "common/logging/AcceptanceRecordFormat.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>

#include "gtest/gtest.h"

namespace {

const char* kTestPath = "oneq_acceptance_format_test.log";

/**
 * @brief 测试临时目录（末尾带分隔符）。
 *
 * 与 `tests/support/oneq_test_temp_dir.h` 同源口径，但本文件不额外依赖该头：
 * common 分区未把 tests/support 加入包含目录，内联可避免耦合测试构建配置。
 * CMake 注入 `ONEQ_TEST_TEMP_DIR` 时优先使用，否则回退 `%TEMP%` 或 `/tmp`。
 */
std::string TestTempDir() {
#ifdef ONEQ_TEST_TEMP_DIR
  std::string dir = ONEQ_TEST_TEMP_DIR;
#else
  const char* env = std::getenv("TEMP");
  std::string dir = (env != nullptr && *env != '\0') ? std::string(env) : std::string("/tmp");
#endif
  if (dir.empty()) {
    dir = ".";
  }
  const char last = dir[dir.size() - 1U];
  if (last != '/' && last != '\\') {
    dir.push_back('/');
  }
  return dir;
}

std::string ReadAll(const std::string& path) {
  std::ifstream in(path.c_str(), std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

class AcceptanceRecordFormatTest : public ::testing::Test {
 protected:
  void TearDown() override {
    oneq::logging::CloseAcceptanceLog(oneq::logging::AcceptanceChannel::kSatelliteTarget);
    oneq::logging::CloseAcceptanceLog(oneq::logging::AcceptanceChannel::kRadarOutput);
    oneq::logging::CloseAcceptanceLog(
        oneq::logging::AcceptanceChannel::kRadarExternalInterface);
    oneq::logging::CloseAcceptanceLog(oneq::logging::AcceptanceChannel::kIrExternalInterface);
    oneq::logging::CloseAcceptanceLog(
        oneq::logging::AcceptanceChannel::kRadarInternalInterface);
    oneq::logging::CloseAcceptanceLog(
        oneq::logging::AcceptanceChannel::kIrInternalInterface);
    std::remove(kTestPath);
    std::remove("oneq_ir_external_interface_test.log");
    std::remove("oneq_ir_internal_interface_test.log");
    std::remove("oneq_radar_external_interface_test.log");
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
  oneq::logging::OpenAcceptanceLog(oneq::logging::AcceptanceChannel::kSatelliteTarget,
                                   kTestPath);
  ASSERT_TRUE(
      oneq::logging::IsAcceptanceLogOpen(oneq::logging::AcceptanceChannel::kSatelliteTarget));
  oneq::logging::WriteAcceptanceLog(
      oneq::logging::AcceptanceChannel::kSatelliteTarget,
      oneq::logging::FormatAcceptanceLine(0.0f, 0U, "初始化时间", "见integration_events.log"));
  oneq::logging::FlushAcceptanceLog(oneq::logging::AcceptanceChannel::kSatelliteTarget);
  oneq::logging::CloseAcceptanceLog(oneq::logging::AcceptanceChannel::kSatelliteTarget);

  const std::string content = ReadAll(kTestPath);
#if defined(_MSC_VER)
  EXPECT_NE(content.find(u8"[初始化时间]"), std::string::npos);
  EXPECT_NE(content.find(u8"integration_events"), std::string::npos);
#else
  EXPECT_NE(content.find("[初始化时间]"), std::string::npos);
  EXPECT_NE(content.find("integration_events"), std::string::npos);
#endif
}

// 覆写语义：进程内首开截断旧文件（重跑进程不向归档证据追加重复行），
// 进程内重开追加（会话重启不丢已写行）。用独立通道 kIrExternalInterface
// 保证本用例拥有该 sink 的进程内首开。
TEST_F(AcceptanceRecordFormatTest, FirstOpenTruncatesThenReopenAppends) {
  const char* rerun_path = "oneq_acceptance_rerun_test.log";
  {
    std::ofstream stale(rerun_path, std::ios::binary);
    stale << "STALE_ROW_FROM_PREVIOUS_PROCESS\n";
  }
  const oneq::logging::AcceptanceChannel ch =
      oneq::logging::AcceptanceChannel::kIrExternalInterface;
  oneq::logging::OpenAcceptanceLog(ch, rerun_path);
  oneq::logging::WriteAcceptanceLog(
      ch, oneq::logging::FormatAcceptanceLine(1.0f, 1U, "首开行", "A"));
  oneq::logging::CloseAcceptanceLog(ch);

  const std::string after_first = ReadAll(rerun_path);
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

  const std::string after_reopen = ReadAll(rerun_path);
#if defined(_MSC_VER)
  EXPECT_NE(after_reopen.find(u8"[首开行]"), std::string::npos);
  EXPECT_NE(after_reopen.find(u8"[重开行]"), std::string::npos);
#else
  EXPECT_NE(after_reopen.find("[首开行]"), std::string::npos);
  EXPECT_NE(after_reopen.find("[重开行]"), std::string::npos);
#endif
  std::remove(rerun_path);
}

// 内部接口与外部接口必须各自独立文件，互不串写。
TEST_F(AcceptanceRecordFormatTest, InterfaceChannelsWriteSeparateFiles) {
  const char* ir_external_path = "oneq_ir_external_interface_test.log";
  const char* ir_internal_path = "oneq_ir_internal_interface_test.log";
  const char* radar_external_path = "oneq_radar_external_interface_test.log";
  const char* radar_internal_path = "oneq_radar_internal_interface_test.log";
  const oneq::logging::AcceptanceChannel ir_external =
      oneq::logging::AcceptanceChannel::kIrExternalInterface;
  const oneq::logging::AcceptanceChannel ir_internal =
      oneq::logging::AcceptanceChannel::kIrInternalInterface;
  const oneq::logging::AcceptanceChannel radar_external =
      oneq::logging::AcceptanceChannel::kRadarExternalInterface;
  const oneq::logging::AcceptanceChannel radar_internal =
      oneq::logging::AcceptanceChannel::kRadarInternalInterface;

  oneq::logging::OpenAcceptanceLog(ir_external, ir_external_path);
  oneq::logging::OpenAcceptanceLog(ir_internal, ir_internal_path);
  oneq::logging::OpenAcceptanceLog(radar_external, radar_external_path);
  oneq::logging::OpenAcceptanceLog(radar_internal, radar_internal_path);
  oneq::logging::WriteAcceptanceLog(
      ir_external,
      oneq::logging::FormatAcceptanceLine(1.0f, 1U, "红外观测输出接口测试", "观测数量=1"));
  oneq::logging::WriteAcceptanceLog(
      ir_internal,
      oneq::logging::FormatAcceptanceLine(1.0f, 1U, "红外内部接口", "目标ID=1001"));
  oneq::logging::WriteAcceptanceLog(
      radar_external,
      oneq::logging::FormatAcceptanceLine(1.0f, 1U, "目标场景接口测试", "目标数量=5"));
  oneq::logging::WriteAcceptanceLog(
      radar_internal,
      oneq::logging::FormatAcceptanceLine(1.0f, 1U, "雷达内部接口", "目标ID=2001"));
  oneq::logging::CloseAcceptanceLog(ir_external);
  oneq::logging::CloseAcceptanceLog(ir_internal);
  oneq::logging::CloseAcceptanceLog(radar_external);
  oneq::logging::CloseAcceptanceLog(radar_internal);

  const std::string ir_external_text = ReadAll(ir_external_path);
  const std::string ir_internal_text = ReadAll(ir_internal_path);
  const std::string radar_external_text = ReadAll(radar_external_path);
  const std::string radar_internal_text = ReadAll(radar_internal_path);
#if defined(_MSC_VER)
  EXPECT_NE(ir_external_text.find(u8"[红外观测输出接口测试]"), std::string::npos);
  EXPECT_EQ(ir_external_text.find(u8"[红外内部接口]"), std::string::npos);
  EXPECT_NE(ir_internal_text.find(u8"[红外内部接口]"), std::string::npos);
  EXPECT_EQ(ir_internal_text.find(u8"[红外观测输出接口测试]"), std::string::npos);
  EXPECT_NE(radar_external_text.find(u8"[目标场景接口测试]"), std::string::npos);
  EXPECT_EQ(radar_external_text.find(u8"[雷达内部接口]"), std::string::npos);
  EXPECT_NE(radar_internal_text.find(u8"[雷达内部接口]"), std::string::npos);
  EXPECT_EQ(radar_internal_text.find(u8"[目标场景接口测试]"), std::string::npos);
#else
  EXPECT_NE(ir_external_text.find("[红外观测输出接口测试]"), std::string::npos);
  EXPECT_EQ(ir_external_text.find("[红外内部接口]"), std::string::npos);
  EXPECT_NE(ir_internal_text.find("[红外内部接口]"), std::string::npos);
  EXPECT_EQ(ir_internal_text.find("[红外观测输出接口测试]"), std::string::npos);
  EXPECT_NE(radar_external_text.find("[目标场景接口测试]"), std::string::npos);
  EXPECT_EQ(radar_external_text.find("[雷达内部接口]"), std::string::npos);
  EXPECT_NE(radar_internal_text.find("[雷达内部接口]"), std::string::npos);
  EXPECT_EQ(radar_internal_text.find("[目标场景接口测试]"), std::string::npos);
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

  const std::string content = ReadAll(path);
#if defined(_MSC_VER)
  EXPECT_NE(content.find(u8"[雷达检测与量测输出接口测试]"), std::string::npos);
#else
  EXPECT_NE(content.find("[雷达检测与量测输出接口测试]"), std::string::npos);
#endif
  EXPECT_NE(content.find(u8"量测数量=1"), std::string::npos);
}

// 目录作用域：六类目标文件必须落在同一输出目录，文件名与大小写按验收判定标准 §0。
TEST_F(AcceptanceRecordFormatTest, OutputDirectoryScopeWritesSixTargetFiles) {
  const std::string dir = TestTempDir() + "oneq_acceptance_targets";
  const std::string target_dir = dir + "/";
  oneq::logging::CloseAcceptanceLog(oneq::logging::AcceptanceChannel::kSatelliteTarget);
  oneq::logging::CloseAcceptanceLog(oneq::logging::AcceptanceChannel::kRadarOutput);
  oneq::logging::CloseAcceptanceLog(
      oneq::logging::AcceptanceChannel::kRadarExternalInterface);
  oneq::logging::CloseAcceptanceLog(oneq::logging::AcceptanceChannel::kIrExternalInterface);
  oneq::logging::CloseAcceptanceLog(
      oneq::logging::AcceptanceChannel::kRadarInternalInterface);
  oneq::logging::CloseAcceptanceLog(
      oneq::logging::AcceptanceChannel::kIrInternalInterface);

  const char* expected[] = {"SatelliteTarget.log", "RadarOutput.log",
                            "RadarExternalInterface.log", "IRExternalInterface.log",
                            "RadarInternalInterface.log", "IRInternalInterface.log"};
  const oneq::logging::AcceptanceChannel channels[] = {
      oneq::logging::AcceptanceChannel::kSatelliteTarget,
      oneq::logging::AcceptanceChannel::kRadarOutput,
      oneq::logging::AcceptanceChannel::kRadarExternalInterface,
      oneq::logging::AcceptanceChannel::kIrExternalInterface,
      oneq::logging::AcceptanceChannel::kRadarInternalInterface,
      oneq::logging::AcceptanceChannel::kIrInternalInterface};
  {
    oneq::logging::ScopedAcceptanceLogOutputDirectory scope(dir);
    for (int i = 0; i < 6; ++i) {
      const std::string line = oneq::logging::FormatAcceptanceLine(
          1.0f, 1U, "目录作用域测试", "通道=" + std::to_string(i));
      oneq::logging::WriteAcceptanceLog(channels[i], line);
      oneq::logging::FlushAcceptanceLog(channels[i]);
      const std::string path = target_dir + expected[i];
      const std::string content = ReadAll(path);
      EXPECT_NE(content.find(u8"通道="), std::string::npos) << path;
      std::remove(path.c_str());
    }
    for (int i = 0; i < 6; ++i) {
      oneq::logging::CloseAcceptanceLog(channels[i]);
    }
  }
}

}  // namespace
