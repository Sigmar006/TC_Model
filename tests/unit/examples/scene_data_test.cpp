/**
 * @file scene_data_test.cpp
 * @brief 场景描述加载器（examples/scenes/scene_data.*）单元测试。
 *
 * 覆盖：基线场景解析（全字段）、缺省块回退、必填几何字段校验、畸形 JSON
 * 报错、空目标数组（"无目标"场景）、session_config 挂载即全量校验。
 * 场景内容以原始字符串内嵌（与 scenes/baseline_takeoff_east/baseline_takeoff_east.json 同值），
 * 写入临时目录后加载——测试不依赖仓库路径。
 */

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "scenes/scene_data.h"

namespace ca = component_attachment;
namespace app = component_attachment::app;

namespace {

/// 临时场景文件（析构清理），内容来自构造参数。
class ScopedSceneFile {
 public:
  explicit ScopedSceneFile(const std::string& content) {
    static std::atomic<std::uint64_t> counter{0U};
    const auto stamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
            ("scene_data_test_" + std::to_string(stamp) + "_" +
             std::to_string(counter.fetch_add(1U)) + ".json");
    std::ofstream out(path_);
    out << content;
  }

  ~ScopedSceneFile() { std::filesystem::remove(path_); }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

/// 加载临时文件内容，返回场景与错误串（断言加载成功）。
app::SceneData LoadOk(const std::string& content) {
  ScopedSceneFile file(content);
  app::SceneData scene;
  std::string error;
  EXPECT_TRUE(app::LoadSceneData(file.path().string().c_str(), &scene, &error)) << error;
  return scene;
}

/// 与 scenes/baseline_takeoff_east/baseline_takeoff_east.json 同值的基线场景内容。
constexpr char kBaselineSceneJson[] = R"json({
  "name": "baseline_takeoff_east",
  "log_dir": "baseline_takeoff_east",
  "cycles": 400,
  "dt_sec": 1.0,
  "platform": {
    "origin_lat_deg": 30.0,
    "origin_lon_deg": 120.0,
    "origin_alt_m": 0.0,
    "initial_heading_deg": 90.0,
    "cruise_altitude_m": 400.0,
    "cruise_speed_mps": 50.0,
    "waypoints": [
      {"lat_deg": 30.0, "lon_deg": 120.05, "alt_m": 400.0, "speed_mps": 50.0, "radius_m": 500.0},
      {"lat_deg": 30.0, "lon_deg": 120.10, "alt_m": 400.0, "speed_mps": 50.0, "radius_m": 500.0},
      {"lat_deg": 30.0, "lon_deg": 120.15, "alt_m": 400.0, "speed_mps": 50.0, "radius_m": 500.0}
    ]
  },
  "targets": [
    {"id": 1001, "azimuth_deg": 0.0, "range_m": 12000.0, "altitude_m": 400.0,
     "v_east_mps": 47.0, "v_north_mps": 5.0, "temperature_k": 520.0,
     "rcs_m2": 2.2, "projected_area_m2": 18.0, "radiant_intensity_w_per_sr": 3819.864, "emitter_center_frequency_hz": 9.5e9},
    {"id": 1002, "azimuth_deg": 0.0, "range_m": 14000.0, "altitude_m": 400.0,
     "v_east_mps": 47.0, "v_north_mps": -5.0, "temperature_k": 540.0,
     "rcs_m2": 1.4, "projected_area_m2": 15.0, "radiant_intensity_w_per_sr": 4113.916, "emitter_center_frequency_hz": 10.0e9}
  ],
  "esr": {
    "peak_gain_dbi": 30.0,
    "bandwidth_hz": 2.0e6,
    "peak_power_w": 5.0e7,
    "pulse_width_s": 1.0e-6,
    "pri_s": 1.0e-3,
    "pulse_count": 200,
    "timing_seed": 42
  },
  "sbirs_satellite": {
    "altitude_m": 500000.0
  },
  "sensors": {"ar": false, "esr": false, "eos": false, "sbirs": false, "sar": false},
  "session_config": {
    "fusion": {
      "position_radius_m": 1000.0,
      "bearing_beamwidth_deg": 8.0,
      "feature_threshold": 0.0,
      "window_size": 10,
      "max_missed_cycles": 5,
      "source_weights": [0.0, 1.0, 0.8, 0.6, 0.5]
    },
    "threat": {}
  },
  "high_threat_confidence": 3.0,
  "smoke": {
    "min_key_events": 1,
    "min_sbirs_events": 1,
    "min_sar_products": 1,
    "min_fused_targets": 1
  }
})json";

}  // namespace

TEST(SceneDataTest, LoadsBaselineScene) {
  const app::SceneData scene = LoadOk(kBaselineSceneJson);

  EXPECT_EQ(scene.name, "baseline_takeoff_east");
  EXPECT_EQ(scene.cycles, 400U);
  EXPECT_DOUBLE_EQ(scene.dt_sec, 1.0);

  EXPECT_DOUBLE_EQ(scene.platform_origin.latitude_deg, 30.0);
  EXPECT_DOUBLE_EQ(scene.platform_origin.longitude_deg, 120.0);
  EXPECT_DOUBLE_EQ(scene.platform_origin.altitude_m, 0.0);
  EXPECT_DOUBLE_EQ(scene.initial_heading_deg, 90.0);
  EXPECT_DOUBLE_EQ(scene.cruise_altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.cruise_speed_mps, 50.0);

  ASSERT_EQ(scene.waypoints.size(), 3U);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].position.latitude_deg, 30.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].position.longitude_deg, 120.05);
  EXPECT_DOUBLE_EQ(scene.waypoints[2].position.longitude_deg, 120.15);
  EXPECT_DOUBLE_EQ(scene.waypoints[1].position.altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[1].speed_mps, 50.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[1].radius_m, 500.0);
  // 无 coverage 块：显式航路、非巡逻。
  EXPECT_FALSE(scene.coverage.planned);

  ASSERT_EQ(scene.targets.size(), 2U);
  EXPECT_EQ(scene.targets[0].id, 1001U);
  EXPECT_DOUBLE_EQ(scene.targets[0].azimuth_deg, 0.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].range_m, 12000.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].v_east_mps, 47.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].v_north_mps, 5.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].temperature_k, 520.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].rcs, 2.2);
  EXPECT_DOUBLE_EQ(scene.targets[0].projected_area_m2, 18.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].radiant_intensity_w_per_sr, 3819.864);
  EXPECT_DOUBLE_EQ(scene.targets[0].emitter_center_frequency_hz, 9.5e9);
  EXPECT_EQ(scene.targets[1].id, 1002U);
  EXPECT_DOUBLE_EQ(scene.targets[1].v_north_mps, -5.0);
  EXPECT_DOUBLE_EQ(scene.targets[1].rcs, 1.4);
  EXPECT_DOUBLE_EQ(scene.targets[1].emitter_center_frequency_hz, 10.0e9);

  EXPECT_DOUBLE_EQ(scene.esr.peak_gain_dbi, 30.0);
  EXPECT_DOUBLE_EQ(scene.esr.bandwidth_hz, 2.0e6);
  EXPECT_DOUBLE_EQ(scene.esr.peak_power_w, 5.0e7);
  EXPECT_DOUBLE_EQ(scene.esr.pulse_width_s, 1.0e-6);
  EXPECT_DOUBLE_EQ(scene.esr.pri_s, 1.0e-3);
  EXPECT_EQ(scene.esr.pulse_count, 200U);
  EXPECT_EQ(scene.esr.timing_seed, 42U);

  EXPECT_DOUBLE_EQ(scene.sbirs_satellite_altitude_m, 500000.0);

  EXPECT_DOUBLE_EQ(scene.high_threat_confidence, 3.0);
  EXPECT_EQ(scene.smoke.min_key_events, 1U);
  EXPECT_EQ(scene.smoke.min_sbirs_events, 1U);
  EXPECT_EQ(scene.smoke.min_sar_products, 1U);
  EXPECT_EQ(scene.smoke.min_fused_targets, 1U);
}

TEST(SceneDataTest, SessionConfigFusionParsedFromBaseline) {
  // 基线 fixture 的 session_config.fusion 经四参加载进 SceneSessionConfigs。
  ScopedSceneFile file(kBaselineSceneJson);
  app::SceneData scene;
  app::SceneSessionConfigs configs;
  std::string error;
  ASSERT_TRUE(app::LoadSceneData(file.path().string().c_str(), &scene, &configs, &error))
      << error;
  EXPECT_DOUBLE_EQ(configs.fusion.position_radius_m, 1000.0);
  EXPECT_DOUBLE_EQ(configs.fusion.bearing_beamwidth_deg, 8.0);
  EXPECT_DOUBLE_EQ(configs.fusion.feature_threshold, 0.0);
  EXPECT_EQ(configs.fusion.window_size, 10U);
  EXPECT_EQ(configs.fusion.max_missed_cycles, 5U);
  ASSERT_EQ(configs.fusion.source_weights.size(), 5U);
  EXPECT_DOUBLE_EQ(configs.fusion.source_weights[1], 1.0);
  EXPECT_DOUBLE_EQ(configs.fusion.source_weights[4], 0.5);
}

TEST(SceneDataTest, DefaultsWhenBlocksMissing) {
  // 仅平台 + 空目标：全部可缺省块回退默认值。
  const app::SceneData scene = LoadOk(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": []
  })json");

  EXPECT_EQ(scene.name, "unnamed_scene");
  EXPECT_EQ(scene.cycles, 400U);
  EXPECT_DOUBLE_EQ(scene.dt_sec, 1.0);
  EXPECT_DOUBLE_EQ(scene.initial_heading_deg, 90.0);
  EXPECT_DOUBLE_EQ(scene.cruise_altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.cruise_speed_mps, 50.0);
  EXPECT_TRUE(scene.waypoints.empty());
  EXPECT_TRUE(scene.targets.empty());
  EXPECT_EQ(scene.esr.pulse_count, 200U);
  EXPECT_EQ(scene.esr.timing_seed, 42U);
  EXPECT_DOUBLE_EQ(scene.sbirs_satellite_altitude_m, 500000.0);
  EXPECT_DOUBLE_EQ(scene.high_threat_confidence, 3.0);
  EXPECT_EQ(scene.smoke.min_key_events, 1U);
  EXPECT_EQ(scene.smoke.min_fused_targets, 1U);
}

TEST(SceneDataTest, WaypointDefaultsToCruiseParameters) {
  const app::SceneData scene = LoadOk(R"json({
    "platform": {
      "origin_lat_deg": 30.0, "origin_lon_deg": 120.0,
      "cruise_altitude_m": 800.0, "cruise_speed_mps": 60.0,
      "waypoints": [{"lat_deg": 31.0, "lon_deg": 121.0}]
    },
    "targets": []
  })json");

  ASSERT_EQ(scene.waypoints.size(), 1U);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].position.latitude_deg, 31.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].position.longitude_deg, 121.0);
  // alt/speed 缺省回退巡航参数，radius 缺省 500 m。
  EXPECT_DOUBLE_EQ(scene.waypoints[0].position.altitude_m, 800.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].speed_mps, 60.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].radius_m, 500.0);
}

TEST(SceneDataTest, FusionAndSmokeOverrides) {
  // 融合覆写位于 session_config.fusion（四参加载）；smoke 仍在场景层。
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "log_dir": "test",
    "sensors": {"ar": false, "esr": false, "eos": false, "sbirs": false, "sar": false},
    "session_config": {"fusion": {"bearing_beamwidth_deg": 12.0, "source_weights": [0.0, 0.5]},
                       "threat": {}},
    "smoke": {"min_fused_targets": 0}
  })json");
  app::SceneData scene;
  app::SceneSessionConfigs configs;
  std::string error;
  ASSERT_TRUE(app::LoadSceneData(file.path().string().c_str(), &scene, &configs, &error))
      << error;

  EXPECT_DOUBLE_EQ(configs.fusion.bearing_beamwidth_deg, 12.0);
  ASSERT_EQ(configs.fusion.source_weights.size(), 2U);
  EXPECT_DOUBLE_EQ(configs.fusion.source_weights[1], 0.5);
  // 未覆写字段保持默认。
  EXPECT_DOUBLE_EQ(configs.fusion.position_radius_m, 1000.0);
  EXPECT_EQ(scene.smoke.min_fused_targets, 0U);
  EXPECT_EQ(scene.smoke.min_key_events, 1U);
}

TEST(SceneDataTest, MissingPlatformBlockFails) {
  ScopedSceneFile file(R"json({"targets": []})json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("platform"), std::string::npos);
}

TEST(SceneDataTest, MissingTargetsBlockFails) {
  ScopedSceneFile file(R"json({"platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0}})json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("targets"), std::string::npos);
}

TEST(SceneDataTest, MissingGeometryFieldFails) {
  // 目标缺 altitude_m（必填几何字段）→ 报错并点名缺失字段。
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [{"id": 1001, "azimuth_deg": 0.0, "range_m": 1000.0, "rcs_m2": 1.0}]
  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("altitude_m"), std::string::npos);
}

TEST(SceneDataTest, InvalidCyclesFails) {
  ScopedSceneFile file(R"json({
    "cycles": 0,
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": []
  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("cycles"), std::string::npos);
}

TEST(SceneDataTest, ParsesTargetManeuvers) {
  const app::SceneData scene = LoadOk(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [{
      "id": 1001, "azimuth_deg": 0.0, "range_m": 12000.0, "altitude_m": 400.0,
      "rcs_m2": 2.2, "v_east_mps": 47.0, "v_north_mps": 5.0,
      "maneuvers": [
        {"start_cycle": 200, "v_east_mps": 47.0, "v_north_mps": -30.0},
        {"start_cycle": 260, "v_north_mps": 0.0}
      ]
    }]
  })json");

  ASSERT_EQ(scene.targets.size(), 1U);
  ASSERT_EQ(scene.targets[0].maneuvers.size(), 2U);
  EXPECT_EQ(scene.targets[0].maneuvers[0].start_cycle, 200U);
  EXPECT_DOUBLE_EQ(scene.targets[0].maneuvers[0].v_east_mps, 47.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].maneuvers[0].v_north_mps, -30.0);
  EXPECT_EQ(scene.targets[0].maneuvers[1].start_cycle, 260U);
  // v_east 缺省 0（机动条目字段级缺省）。
  EXPECT_DOUBLE_EQ(scene.targets[0].maneuvers[1].v_east_mps, 0.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].maneuvers[1].v_north_mps, 0.0);
}

TEST(SceneDataTest, ManeuverMissingStartCycleFails) {
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [{
      "id": 1001, "azimuth_deg": 0.0, "range_m": 12000.0, "altitude_m": 400.0,
      "rcs_m2": 2.2, "maneuvers": [{"v_north_mps": -30.0}]
    }]
  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("start_cycle"), std::string::npos);
}

TEST(SceneDataTest, ManeuverStartCycleMustIncrease) {
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [{
      "id": 1001, "azimuth_deg": 0.0, "range_m": 12000.0, "altitude_m": 400.0,
      "rcs_m2": 2.2,
      "maneuvers": [
        {"start_cycle": 260, "v_north_mps": 0.0},
        {"start_cycle": 200, "v_north_mps": -30.0}
      ]
    }]
  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("increasing"), std::string::npos);
}

TEST(SceneDataTest, MalformedJsonFails) {
  ScopedSceneFile file(R"json({"platform": {"origin_lat_deg": )json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_FALSE(error.empty());
}

TEST(SceneDataTest, SessionConfigRequiredForFullLoad) {
  // 四参重载（场景可执行/通用 runner 的加载路径）：session_config 缺失 →
  // 报错；挂载即全量——挂载通道缺子块、未挂载通道携带子块均报错。
  ScopedSceneFile file(R"json({"platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [], "log_dir": "test"})json");
  app::SceneData scene;
  app::SceneSessionConfigs configs;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &configs, &error));
  EXPECT_NE(error.find("session_config"), std::string::npos);
}

TEST(SceneDataTest, SessionConfigUnmountedSensorBlockRejected) {
  // 全通道不挂载（sensors 全 false、rir/ecm 关）：session_config 只带恒挂载
  // 的 fusion/threat（可为空对象）→ 通过；额外携带未挂载通道的 ar 块 → 报错。
  const char* ok_json = R"json({"platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "log_dir": "test",
    "sensors": {"ar": false, "esr": false, "eos": false, "sbirs": false, "sar": false},
    "session_config": {"fusion": {}, "threat": {}}})json";
  app::SceneData scene;
  app::SceneSessionConfigs configs;
  std::string error;
  {
    ScopedSceneFile file(ok_json);
    EXPECT_TRUE(app::LoadSceneData(file.path().string().c_str(), &scene, &configs, &error))
        << error;
  }
  const std::string with_ar = std::string(R"json({"platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "log_dir": "test",
    "sensors": {"ar": false, "esr": false, "eos": false, "sbirs": false, "sar": false},
    "session_config": {"fusion": {}, "threat": {}, "ar": {"hardware": {}}}})json");
  ScopedSceneFile file(with_ar);
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &configs, &error))
      << error;
  EXPECT_NE(error.find("session_config.ar"), std::string::npos) << error;
}

TEST(SceneDataTest, SessionConfigMountedSensorBlockRequiredAndParsed) {
  // 挂载 EOS（默认 true）：session_config 缺 eos 子块 → 报错；携带子块后
  // 字段按模板结构解析（帧率来自 mission.frame_rate_hz）。
  const std::string missing = std::string(R"json({"platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [], "log_dir": "test", "sensors": {"ar": false, "esr": false, "eos": true, "sbirs": false, "sar": false},
    "session_config": {"fusion": {}, "threat": {}}})json");
  app::SceneData scene;
  app::SceneSessionConfigs configs;
  std::string error;
  {
    ScopedSceneFile file(missing);
    EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &configs, &error));
    EXPECT_NE(error.find("session_config.eos"), std::string::npos);
  }
  const std::string with_eos = std::string(R"json({"platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [], "log_dir": "test", "sensors": {"ar": false, "esr": false, "eos": true, "sbirs": false, "sar": false},
    "session_config": {"fusion": {}, "threat": {},
      "eos": {"mission": {"frame_rate_hz": 12.5}}}})json");
  ScopedSceneFile file(with_eos);
  EXPECT_TRUE(app::LoadSceneData(file.path().string().c_str(), &scene, &configs, &error))
      << error;
  EXPECT_FLOAT_EQ(configs.eos.mission.frame_rate_hz, 12.5f);
}

TEST(SceneDataTest, LogDirRequiredForFullLoad) {
  // 四参加载：log_dir 必填（场景自带日志落点，禁止漂到运行目录/临时目录）。
  ScopedSceneFile file(R"json({"platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "sensors": {"ar": false, "esr": false, "eos": false, "sbirs": false, "sar": false},
    "session_config": {"fusion": {}, "threat": {}}})json");
  app::SceneData scene;
  app::SceneSessionConfigs configs;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &configs, &error));
  EXPECT_NE(error.find("log_dir"), std::string::npos) << error;
}

TEST(SceneDataTest, LogDirRejectsTraversalAndAbsolute) {
  // log_dir 须为相对路径且不含 ..（解析基点 examples/log/，防逃逸日志根）。
  app::SceneData scene;
  app::SceneSessionConfigs configs;
  std::string error;
  for (const char* bad : {"../escape", "/abs/dir", "a/../.."}) {
    const std::string json = std::string(R"json({"platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [], "log_dir": ")json") + bad +
        R"json("})json";
    ScopedSceneFile file(json);
    EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &configs, &error))
        << bad;
    EXPECT_NE(error.find("log_dir"), std::string::npos) << bad;
  }
}

TEST(SceneDataTest, UnknownTopLevelKeyRejected) {
  // schema 正式化：未知顶层键报错（拼写保护——错键名静默忽略会让调参失效）。
  ScopedSceneFile file(R"json({"platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [], "log_dir": "test", "fusio_typo": 1})json");
  app::SceneData scene;
  app::SceneSessionConfigs configs;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &configs, &error));
  EXPECT_NE(error.find("fusio_typo"), std::string::npos) << error;
}

TEST(SceneDataTest, SessionConfigEcmParsed) {
  // ecm.enabled 场景携带 session_config.ecm（原 MakeDefaultEcmConfig 代码默认
  // 数据化）：通道数/功率上限/默认技术解析入库配置。
  ScopedSceneFile file(R"json({"platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [], "log_dir": "test", "sensors": {"ar": false, "esr": false, "eos": false, "sbirs": false, "sar": false},
    "ecm": {"enabled": true},
    "session_config": {"fusion": {}, "threat": {},
      "ecm": {"channel_count": 1,
              "maximum_total_transmit_power_w": 1000.0,
              "maximum_channel_transmit_power_w": 1000.0,
              "default_technique": "kSpot"}}})json");
  app::SceneData scene;
  app::SceneSessionConfigs configs;
  std::string error;
  EXPECT_TRUE(app::LoadSceneData(file.path().string().c_str(), &scene, &configs, &error))
      << error;
  EXPECT_EQ(configs.ecm.channel_count, 1U);
  EXPECT_DOUBLE_EQ(configs.ecm.maximum_total_transmit_power_w, 1000.0);
  EXPECT_EQ(configs.ecm.default_technique,
            electronic_countermeasure::EcmTechnique::kSpot);
}

TEST(SceneDataTest, PlansPolygonScanPatrolRoute) {
  // coverage 多边形 + 扫描模式：加载时经 AreaCoveragePlanner 生成巡逻航路。
  // 区域 lat 29.989..30.001（约 1334 m 南北跨度）× lon 120.0..120.03，间距
  // 500 m → 扫描线 v = 250/750/1250（3 条 × 2 端点 = 6 航点），牛耕式交替
  // 方向（线 1 东向 → 线 2 西向 → 线 3 东向）。
  const app::SceneData scene = LoadOk(R"json({

    "platform": {
        "origin_lat_deg": 30.0,
        "origin_lon_deg": 120.0,
        "cruise_altitude_m": 400.0,
        "cruise_speed_mps": 50.0,
        "coverage": {
            "kind": "polygon",
            "mode": "scan",
            "vertices": [
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.0
                },
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.0
                }
            ],
            "scan_heading_deg": 0.0,
            "scan_spacing_m": 500.0,
            "arrival_radius_m": 200.0
        }
    },
    "targets": []

  })json");

  EXPECT_TRUE(scene.coverage.planned);
  EXPECT_EQ(scene.coverage.area.kind, navigation::CoverageAreaKind::kPolygon);
  EXPECT_EQ(scene.coverage.config.mode, navigation::CoverageMode::kScan);
  // 高度/速度缺省回退巡航参数。
  EXPECT_DOUBLE_EQ(scene.coverage.config.altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.coverage.config.speed_mps, 50.0);

  ASSERT_EQ(scene.waypoints.size(), 6U);
  // 线 1 东向：同纬度、lon 递增。
  EXPECT_DOUBLE_EQ(scene.waypoints[0].position.latitude_deg,
                   scene.waypoints[1].position.latitude_deg);
  EXPECT_LT(scene.waypoints[0].position.longitude_deg,
            scene.waypoints[1].position.longitude_deg);
  // 线 2 西向：北移一条扫描间距、lon 递减。
  EXPECT_GT(scene.waypoints[2].position.latitude_deg,
            scene.waypoints[1].position.latitude_deg);
  EXPECT_GT(scene.waypoints[2].position.longitude_deg,
            scene.waypoints[3].position.longitude_deg);
  // 线 3 东向：北移一条扫描间距。
  EXPECT_GT(scene.waypoints[4].position.latitude_deg,
            scene.waypoints[3].position.latitude_deg);
  EXPECT_LT(scene.waypoints[4].position.longitude_deg,
            scene.waypoints[5].position.longitude_deg);
  // 相邻扫描线纬度差 ≈ 500 m（1° 纬度 ≈ 111190 m），容差覆盖 ENU 回变换误差。
  const double line_delta_deg = scene.waypoints[2].position.latitude_deg -
                                scene.waypoints[0].position.latitude_deg;
  EXPECT_NEAR(line_delta_deg, 500.0 / 111190.0, 1e-4);
  // 航点属性来自规划配置。
  EXPECT_DOUBLE_EQ(scene.waypoints[0].position.altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].speed_mps, 50.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].radius_m, 200.0);
}

TEST(SceneDataTest, PlansCircleOrbitPatrolRoute) {
  // coverage 圆形 + 盘旋模式：8 段 × 1 环 = 8 航点，均匀分布在半径 5000 m
  // 的圆周上（逆时针，自正北起）。
  const app::SceneData scene = LoadOk(R"json({

    "platform": {
        "origin_lat_deg": 30.0,
        "origin_lon_deg": 120.0,
        "coverage": {
            "kind": "circle",
            "mode": "orbit",
            "center": {
                "lat_deg": 30.0,
                "lon_deg": 120.0
            },
            "radius_m": 5000.0,
            "altitude_m": 400.0,
            "speed_mps": 50.0,
            "arrival_radius_m": 200.0,
            "orbit_segments": 8,
            "orbit_rings": 1
        }
    },
    "targets": []

  })json");

  EXPECT_TRUE(scene.coverage.planned);
  EXPECT_EQ(scene.coverage.area.kind, navigation::CoverageAreaKind::kCircle);
  EXPECT_EQ(scene.coverage.config.mode, navigation::CoverageMode::kOrbit);
  ASSERT_EQ(scene.waypoints.size(), 8U);
  for (const auto& wp : scene.waypoints) {
    // 每个航点都在圆心 5000 m 半径附近：纬度偏移 ≈ 5000/110850 ≈ 0.04511°
    // （WGS84 30°N 子午线弧长），经度偏移 ≈ 5000/(110850·cos30°) ≈ 0.0521°
    // （圆周几何，容差覆盖 ENU 回变换误差）。
    EXPECT_NEAR(wp.position.latitude_deg - 30.0, 0.0, 0.0452);
    EXPECT_NEAR(wp.position.longitude_deg - 120.0, 0.0, 0.0522);
    EXPECT_DOUBLE_EQ(wp.position.altitude_m, 400.0);
    EXPECT_DOUBLE_EQ(wp.speed_mps, 50.0);
  }
}

TEST(SceneDataTest, CoverageAndExplicitWaypointsConflictFails) {
  ScopedSceneFile file(R"json({

    "platform": {
        "origin_lat_deg": 30.0,
        "origin_lon_deg": 120.0,
        "waypoints": [
            {
                "lat_deg": 30.0,
                "lon_deg": 120.05
            }
        ],
        "coverage": {
            "kind": "polygon",
            "mode": "scan",
            "vertices": [
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.0
                },
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.0
                }
            ],
            "scan_spacing_m": 500.0
        }
    },
    "targets": []

  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("mutually exclusive"), std::string::npos);
}

TEST(SceneDataTest, InvalidCoverageGeometryFails) {
  // 扫描间距非正 → 规划失败（空计划）→ 加载报错，不允许静默直飞。
  ScopedSceneFile file(R"json({

    "platform": {
        "origin_lat_deg": 30.0,
        "origin_lon_deg": 120.0,
        "coverage": {
            "kind": "polygon",
            "mode": "scan",
            "vertices": [
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.0
                },
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.0
                }
            ],
            "scan_spacing_m": 0.0
        }
    },
    "targets": []

  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("planning failed"), std::string::npos);
}

TEST(SceneDataTest, CoverageModeKindMismatchFails) {
  // 盘旋模式 + 多边形区域：模式-区域不匹配 → 规划失败。
  ScopedSceneFile file(R"json({

    "platform": {
        "origin_lat_deg": 30.0,
        "origin_lon_deg": 120.0,
        "coverage": {
            "kind": "polygon",
            "mode": "orbit",
            "vertices": [
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.0
                },
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.0
                }
            ]
        }
    },
    "targets": []

  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("planning failed"), std::string::npos);
}

TEST(SceneDataTest, MissingCoverageVertexFails) {
  ScopedSceneFile file(R"json({

    "platform": {
        "origin_lat_deg": 30.0,
        "origin_lon_deg": 120.0,
        "coverage": {
            "kind": "polygon",
            "mode": "scan",
            "vertices": [
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.0
                },
                {
                    "lon_deg": 120.03
                }
            ],
            "scan_spacing_m": 500.0
        }
    },
    "targets": []

  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("lat_deg"), std::string::npos);
}

TEST(SceneDataTest, ParsesWingmanPlatforms) {
  // platforms[] 数组：每条目同 platform 块（原点/航向/巡航 + waypoints 或
  // coverage 区域巡逻）；巡航参数缺省回退主平台值。
  const app::SceneData scene = LoadOk(R"json({
    "platform": {
      "origin_lat_deg": 30.0,
      "origin_lon_deg": 120.0,
      "cruise_altitude_m": 400.0,
      "cruise_speed_mps": 50.0
    },
    "targets": [],
    "platforms": [
      {
        "name": "wing_a",
        "origin_lat_deg": 30.01,
        "origin_lon_deg": 120.01,
        "initial_heading_deg": 270.0,
        "waypoints": [
          {"lat_deg": 30.01, "lon_deg": 120.02, "speed_mps": 45.0}
        ]
      },
      {
        "name": "wing_b",
        "origin_lat_deg": 30.02,
        "origin_lon_deg": 120.02,
        "coverage": {
          "kind": "circle",
          "mode": "orbit",
          "center": {"lat_deg": 30.02, "lon_deg": 120.05},
          "radius_m": 2000.0
        }
      }
    ]
  })json");

  ASSERT_EQ(scene.platforms.size(), 2U);
  // 从机 A：显式航路 + 名称/航向/巡航缺省回退主平台。
  EXPECT_EQ(scene.platforms[0].name, "wing_a");
  EXPECT_DOUBLE_EQ(scene.platforms[0].origin.latitude_deg, 30.01);
  EXPECT_DOUBLE_EQ(scene.platforms[0].initial_heading_deg, 270.0);
  ASSERT_EQ(scene.platforms[0].waypoints.size(), 1U);
  EXPECT_DOUBLE_EQ(scene.platforms[0].waypoints[0].speed_mps, 45.0);
  EXPECT_FALSE(scene.platforms[0].coverage.planned);
  // 从机 B：coverage 圆形盘旋 → 规划航路（8 段 × 1 环）。
  EXPECT_EQ(scene.platforms[1].name, "wing_b");
  EXPECT_TRUE(scene.platforms[1].coverage.planned);
  EXPECT_EQ(scene.platforms[1].coverage.area.kind, navigation::CoverageAreaKind::kCircle);
  EXPECT_EQ(scene.platforms[1].coverage.config.mode, navigation::CoverageMode::kOrbit);
  ASSERT_EQ(scene.platforms[1].waypoints.size(), 8U);
  // 巡航缺省回退主平台（400/50），而非 ScenePlatform 默认值。
  EXPECT_DOUBLE_EQ(scene.platforms[1].coverage.config.altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.platforms[1].coverage.config.speed_mps, 50.0);
  // 主平台不受 platforms[] 影响（单平台语义保留）。
  EXPECT_DOUBLE_EQ(scene.platform_origin.latitude_deg, 30.0);
}

TEST(SceneDataTest, WingmanWithoutNameDefaultsToWingmanIndex) {
  const app::SceneData scene = LoadOk(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "platforms": [
      {"origin_lat_deg": 30.01, "origin_lon_deg": 120.01},
      {"origin_lat_deg": 30.02, "origin_lon_deg": 120.02}
    ]
  })json");
  ASSERT_EQ(scene.platforms.size(), 2U);
  EXPECT_EQ(scene.platforms[0].name, "wingman_1");
  EXPECT_EQ(scene.platforms[1].name, "wingman_2");
}

TEST(SceneDataTest, PlatformsNotArrayFails) {
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "platforms": {"origin_lat_deg": 30.01, "origin_lon_deg": 120.01}
  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("platforms"), std::string::npos);
}

TEST(SceneDataTest, WingmanCoverageAndWaypointsConflictFails) {
  // 从机同主平台：waypoints 与 coverage 互斥报错（航路来源歧义）。
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "platforms": [
      {
        "origin_lat_deg": 30.01,
        "origin_lon_deg": 120.01,
        "waypoints": [{"lat_deg": 30.01, "lon_deg": 120.02}],
        "coverage": {
          "kind": "polygon",
          "mode": "scan",
          "vertices": [
            {"lat_deg": 29.989, "lon_deg": 120.0},
            {"lat_deg": 29.989, "lon_deg": 120.03},
            {"lat_deg": 30.001, "lon_deg": 120.03}
          ],
          "scan_spacing_m": 500.0
        }
      }
    ]
  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("mutually exclusive"), std::string::npos);
}

TEST(SceneDataTest, ParsesTargetType) {
  // targets[].type：缺省 air；ground 显式（地面目标 = 静止近地运动学点）。
  const app::SceneData scene = LoadOk(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [
      {"id": 1001, "azimuth_deg": 0.0, "range_m": 12000.0, "altitude_m": 400.0,
       "rcs_m2": 2.2},
      {"id": 1002, "azimuth_deg": 0.0, "range_m": 12000.0, "altitude_m": 0.0,
       "rcs_m2": 1.0, "type": "ground"}
    ]
  })json");
  ASSERT_EQ(scene.targets.size(), 2U);
  EXPECT_EQ(scene.targets[0].type, "air");
  EXPECT_EQ(scene.targets[1].type, "ground");
}

TEST(SceneDataTest, InvalidTargetTypeFails) {
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [
      {"id": 1001, "azimuth_deg": 0.0, "range_m": 12000.0, "altitude_m": 400.0,
       "rcs_m2": 2.2, "type": "sea"}
    ]
  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("type"), std::string::npos);
}

TEST(SceneDataTest, PlansFormationDivisionPolygonRoutes) {
  // 顶层 mission_area（多边形 + 扫描）：加载时自动切分为 3 条等宽条带
  // （主机 = 最南条带，wingman_1/2 依次向北），逐机生成覆盖航路。
  // 区域 lat 29.989..30.001（约 1334 m）× lon 120.0..120.03，条带宽
  // ≈ 444.8 m；间距 250 m → 每机条带内 2 条扫描线 × 2 端点 = 4 航点。
  const app::SceneData scene = LoadOk(R"json({
    "platform": {
        "origin_lat_deg": 30.0,
        "origin_lon_deg": 120.0,
        "cruise_altitude_m": 400.0,
        "cruise_speed_mps": 50.0
    },
    "platforms": [
        {"name": "wingman_1", "origin_lat_deg": 30.0, "origin_lon_deg": 120.06},
        {"name": "wingman_2", "origin_lat_deg": 30.0, "origin_lon_deg": 120.12}
    ],
    "mission_area": {
        "kind": "polygon",
        "mode": "scan",
        "vertices": [
            {"lat_deg": 29.989, "lon_deg": 120.0},
            {"lat_deg": 29.989, "lon_deg": 120.03},
            {"lat_deg": 30.001, "lon_deg": 120.03},
            {"lat_deg": 30.001, "lon_deg": 120.0}
        ],
        "scan_heading_deg": 0.0,
        "scan_spacing_m": 250.0,
        "altitude_m": 400.0,
        "speed_mps": 50.0,
        "arrival_radius_m": 200.0
    },
    "targets": []
  })json");

  // 主机：最南条带（lat ∈ [29.989, 29.989 + 444.8/111190 ≈ 29.9930]），
  // 4 航点；从机 2 架各 4 航点，条带依次向北且边界无缝共享。
  ASSERT_EQ(scene.platforms.size(), 2U);
  EXPECT_TRUE(scene.coverage.planned);
  EXPECT_EQ(scene.coverage.area.kind, navigation::CoverageAreaKind::kPolygon);
  ASSERT_EQ(scene.coverage.area.polygon.vertices.size(), 4U);
  EXPECT_NEAR(scene.coverage.area.polygon.vertices[0].latitude_deg, 29.989, 1e-5);
  double host_lat_min = 1e9, host_lat_max = -1e9;
  for (const auto& vertex : scene.coverage.area.polygon.vertices) {
    host_lat_min = std::min(host_lat_min, vertex.latitude_deg);
    host_lat_max = std::max(host_lat_max, vertex.latitude_deg);
  }
  EXPECT_NEAR(host_lat_max - host_lat_min, 444.8 / 111190.0, 1e-4);
  EXPECT_EQ(scene.waypoints.size(), 4U);  // 每机 2 线 × 2 端点
  EXPECT_DOUBLE_EQ(scene.coverage.config.scan_spacing_m, 250.0);
  EXPECT_DOUBLE_EQ(scene.coverage.config.arrival_radius_m, 200.0);

  double prev_strip_max_lat = host_lat_max;
  for (std::size_t i = 0U; i < scene.platforms.size(); ++i) {
    const auto& platform = scene.platforms[i];
    EXPECT_TRUE(platform.coverage.planned);
    EXPECT_EQ(platform.coverage.area.kind, navigation::CoverageAreaKind::kPolygon);
    ASSERT_EQ(platform.waypoints.size(), 4U);
    double lat_min = 1e9, lat_max = -1e9;
    for (const auto& vertex : platform.coverage.area.polygon.vertices) {
      lat_min = std::min(lat_min, vertex.latitude_deg);
      lat_max = std::max(lat_max, vertex.latitude_deg);
    }
    // 与上一条带共享边界（无缝覆盖，容差覆盖 ENU 回变换误差）。
    EXPECT_NEAR(lat_min, prev_strip_max_lat, 1e-5);
    EXPECT_NEAR(lat_max - lat_min, 444.8 / 111190.0, 1e-4);
    prev_strip_max_lat = lat_max;
  }
}

TEST(SceneDataTest, PlansFormationDivisionCircleRoutes) {
  // 顶层 mission_area（圆形 + 盘旋）+ 1 架从机 → 2 个同心环：主机最外环
  // （半径 2000 m）、wingman_1 内环（1000 m）；每机单环（orbit_rings 由
  // 切分强制 1，JSON 传入的 2 被覆盖），8 段 = 8 航点。
  const app::SceneData scene = LoadOk(R"json({
    "platform": {
        "origin_lat_deg": 30.0,
        "origin_lon_deg": 120.0
    },
    "platforms": [
        {"name": "wingman_1", "origin_lat_deg": 30.0, "origin_lon_deg": 120.12}
    ],
    "mission_area": {
        "kind": "circle",
        "mode": "orbit",
        "center": {"lat_deg": 30.0, "lon_deg": 120.0},
        "radius_m": 2000.0,
        "altitude_m": 400.0,
        "speed_mps": 50.0,
        "arrival_radius_m": 200.0,
        "orbit_segments": 8,
        "orbit_rings": 2
    },
    "targets": []
  })json");

  ASSERT_EQ(scene.platforms.size(), 1U);
  EXPECT_TRUE(scene.coverage.planned);
  EXPECT_EQ(scene.coverage.area.kind, navigation::CoverageAreaKind::kCircle);
  EXPECT_NEAR(scene.coverage.area.circle.radius_m, 2000.0, 1e-6);
  EXPECT_EQ(scene.coverage.config.orbit_rings, 1U);
  ASSERT_EQ(scene.waypoints.size(), 8U);

  EXPECT_TRUE(scene.platforms[0].coverage.planned);
  EXPECT_EQ(scene.platforms[0].coverage.area.kind, navigation::CoverageAreaKind::kCircle);
  EXPECT_NEAR(scene.platforms[0].coverage.area.circle.radius_m, 1000.0, 1e-6);
  EXPECT_EQ(scene.platforms[0].coverage.config.orbit_rings, 1U);
  ASSERT_EQ(scene.platforms[0].waypoints.size(), 8U);
}

TEST(SceneDataTest, MissionAreaConflictsWithPerPlatformCoverageFail) {
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "platforms": [
        {"name": "wingman_1", "origin_lat_deg": 30.0, "origin_lon_deg": 120.06,
         "coverage": {
             "kind": "polygon",
             "mode": "scan",
             "vertices": [
                 {"lat_deg": 29.99, "lon_deg": 120.0},
                 {"lat_deg": 29.99, "lon_deg": 120.02},
                 {"lat_deg": 30.0, "lon_deg": 120.02},
                 {"lat_deg": 30.0, "lon_deg": 120.0}
             ],
             "scan_spacing_m": 500.0
         }}
    ],
    "mission_area": {
        "kind": "polygon",
        "mode": "scan",
        "vertices": [
            {"lat_deg": 29.99, "lon_deg": 120.0},
            {"lat_deg": 29.99, "lon_deg": 120.02},
            {"lat_deg": 30.0, "lon_deg": 120.02},
            {"lat_deg": 30.0, "lon_deg": 120.0}
        ],
        "scan_spacing_m": 500.0
    },
    "targets": []
  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("mutually exclusive"), std::string::npos);
}

TEST(SceneDataTest, MissionAreaWithoutWingmenFails) {
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "mission_area": {
        "kind": "polygon",
        "mode": "scan",
        "vertices": [
            {"lat_deg": 29.99, "lon_deg": 120.0},
            {"lat_deg": 29.99, "lon_deg": 120.02},
            {"lat_deg": 30.0, "lon_deg": 120.02},
            {"lat_deg": 30.0, "lon_deg": 120.0}
        ],
        "scan_spacing_m": 500.0
    },
    "targets": []
  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("requires"), std::string::npos);
}

TEST(SceneDataTest, ParsesBallisticTarget) {
  // 弹道条目：LLA 数组 [lat, lon, alt] + 顶高/顶高时刻；与 208 号目标同值
  // （设计文档 §4.2 schema 示例）。
  const app::SceneData scene = LoadOk(R"json({
    "platform": {"origin_lat_deg": 64.2878, "origin_lon_deg": -149.1746},
    "targets": [
      {"id": 208, "type": "ballistic",
       "start_lla_deg_m": [43.195964, 122.728481, 0.0],
       "end_lla_deg_m": [45.311723, -121.221318, 0.0],
       "max_alt_m": 1663185.661, "max_alt_time_s": 1247.0,
       "rcs_m2": 0.1, "projected_area_m2": 1.0, "temperature_k": 600.0,
       "radiant_intensity_w_per_sr": 300.0, "emitter_center_frequency_hz": 0.0,
       "rir": {"rcs_dbsm": -10.0, "truth_model": "BALLISTIC_EXAMPLE_B"}}
    ]
  })json");
  ASSERT_EQ(scene.targets.size(), 1U);
  const app::ScriptedTarget& target = scene.targets[0];
  EXPECT_TRUE(target.is_ballistic);
  EXPECT_EQ(target.type, "ballistic");
  EXPECT_EQ(target.id, 208U);
  EXPECT_DOUBLE_EQ(target.start_lla.latitude_deg, 43.195964);
  EXPECT_DOUBLE_EQ(target.start_lla.longitude_deg, 122.728481);
  EXPECT_DOUBLE_EQ(target.end_lla.latitude_deg, 45.311723);
  EXPECT_DOUBLE_EQ(target.end_lla.longitude_deg, -121.221318);
  EXPECT_DOUBLE_EQ(target.max_alt_m, 1663185.661);
  EXPECT_DOUBLE_EQ(target.max_alt_time_s, 1247.0);
  EXPECT_DOUBLE_EQ(target.rcs, 0.1);
  EXPECT_TRUE(target.has_rir_features);
  EXPECT_EQ(target.rir_truth_model, "BALLISTIC_EXAMPLE_B");
}

TEST(SceneDataTest, BallisticMutuallyExclusiveFieldsFail) {
  app::SceneData scene;
  std::string error;
  // ENU 几何字段与弹道互斥：azimuth_deg 出现即报错。
  ScopedSceneFile azimuth(R"json({
    "platform": {"origin_lat_deg": 64.2878, "origin_lon_deg": -149.1746},
    "targets": [{"id": 208, "type": "ballistic", "azimuth_deg": 90.0,
                 "start_lla_deg_m": [43.195964, 122.728481, 0.0],
                 "end_lla_deg_m": [45.311723, -121.221318, 0.0],
                 "max_alt_m": 1663185.661, "max_alt_time_s": 1247.0, "rcs_m2": 0.1}]
  })json");
  EXPECT_FALSE(app::LoadSceneData(azimuth.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("azimuth_deg"), std::string::npos);
  // 机动表与弹道互斥。
  ScopedSceneFile maneuvers(R"json({
    "platform": {"origin_lat_deg": 64.2878, "origin_lon_deg": -149.1746},
    "targets": [{"id": 208, "type": "ballistic",
                 "start_lla_deg_m": [43.195964, 122.728481, 0.0],
                 "end_lla_deg_m": [45.311723, -121.221318, 0.0],
                 "max_alt_m": 1663185.661, "max_alt_time_s": 1247.0, "rcs_m2": 0.1,
                 "maneuvers": [{"start_cycle": 10, "v_east_mps": 5.0}]}]
  })json");
  EXPECT_FALSE(app::LoadSceneData(maneuvers.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("maneuvers"), std::string::npos);
}

TEST(SceneDataTest, BallisticMissingOrMalformedFieldsFail) {
  app::SceneData scene;
  std::string error;
  // 缺顶高时刻（必填弹道几何）。
  ScopedSceneFile missing(R"json({
    "platform": {"origin_lat_deg": 64.2878, "origin_lon_deg": -149.1746},
    "targets": [{"id": 208, "type": "ballistic",
                 "start_lla_deg_m": [43.195964, 122.728481, 0.0],
                 "end_lla_deg_m": [45.311723, -121.221318, 0.0],
                 "max_alt_m": 1663185.661, "rcs_m2": 0.1}]
  })json");
  EXPECT_FALSE(app::LoadSceneData(missing.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("max_alt_time_s"), std::string::npos);
  // LLA 数组长度不为 3。
  ScopedSceneFile short_lla(R"json({
    "platform": {"origin_lat_deg": 64.2878, "origin_lon_deg": -149.1746},
    "targets": [{"id": 208, "type": "ballistic",
                 "start_lla_deg_m": [43.195964, 122.728481],
                 "end_lla_deg_m": [45.311723, -121.221318, 0.0],
                 "max_alt_m": 1663185.661, "max_alt_time_s": 1247.0, "rcs_m2": 0.1}]
  })json");
  EXPECT_FALSE(app::LoadSceneData(short_lla.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("start_lla_deg_m"), std::string::npos);
  // 起落点重合：轨道不可解，加载期报错。
  ScopedSceneFile degenerate(R"json({
    "platform": {"origin_lat_deg": 64.2878, "origin_lon_deg": -149.1746},
    "targets": [{"id": 208, "type": "ballistic",
                 "start_lla_deg_m": [43.195964, 122.728481, 0.0],
                 "end_lla_deg_m": [43.195964, 122.728481, 0.0],
                 "max_alt_m": 1663185.661, "max_alt_time_s": 1247.0, "rcs_m2": 0.1}]
  })json");
  EXPECT_FALSE(app::LoadSceneData(degenerate.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("ballistic"), std::string::npos);
}

TEST(SceneDataTest, StartCycleDefaultsAndParses) {
  const app::SceneData baseline = LoadOk(kBaselineSceneJson);
  EXPECT_EQ(baseline.start_cycle, 1U);

  ScopedSceneFile with_start(R"json({
    "log_dir": "t",
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "session_config": {"fusion": {}, "threat": {}},
    "start_cycle": 740,
    "cycles": 100
  })json");
  app::SceneData scene;
  std::string error;
  ASSERT_TRUE(app::LoadSceneData(with_start.path().string().c_str(), &scene, &error)) << error;
  EXPECT_EQ(scene.start_cycle, 740U);
  EXPECT_EQ(scene.cycles, 100U);

  ScopedSceneFile invalid_start(R"json({
    "log_dir": "t",
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "session_config": {"fusion": {}, "threat": {}},
    "start_cycle": 0
  })json");
  EXPECT_FALSE(app::LoadSceneData(invalid_start.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("start_cycle"), std::string::npos);
}

TEST(SceneDataTest, CustomDeviceIdsParsedSuccessfully) {
  ScopedSceneFile file(R"json({
    "log_dir": "t",
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "device_ids": {
      "ar": 11,
      "esr": 12,
      "eos": 13,
      "sbirs": 14,
      "rir": 15,
      "ecm": 16
    }
  })json");
  app::SceneData scene;
  std::string error;
  ASSERT_TRUE(app::LoadSceneData(file.path().string().c_str(), &scene, &error)) << error;
  EXPECT_EQ(scene.device_ids.ar, 11U);
  EXPECT_EQ(scene.device_ids.esr, 12U);
  EXPECT_EQ(scene.device_ids.eos, 13U);
  EXPECT_EQ(scene.device_ids.sbirs, 14U);
  EXPECT_EQ(scene.device_ids.rir, 15U);
  EXPECT_EQ(scene.device_ids.ecm, 16U);
}

TEST(SceneDataTest, DeviceIdsDefaultWhenOmitted) {
  ScopedSceneFile file(R"json({
    "log_dir": "t",
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "session_config": {"fusion": {}, "threat": {}}
  })json");
  app::SceneData scene;
  std::string error;
  ASSERT_TRUE(app::LoadSceneData(file.path().string().c_str(), &scene, &error)) << error;
  EXPECT_EQ(scene.device_ids.ar, fusion::kArSourceId);
  EXPECT_EQ(scene.device_ids.esr, fusion::kEsrSourceId);
  EXPECT_EQ(scene.device_ids.eos, fusion::kEosSourceId);
  EXPECT_EQ(scene.device_ids.sbirs, fusion::kSbirsSourceId);
  EXPECT_EQ(scene.device_ids.rir, fusion::kRirSourceId);
  EXPECT_EQ(scene.device_ids.ecm, 101U);
}

TEST(SceneDataTest, DuplicateDeviceIdsRejected) {
  ScopedSceneFile file(R"json({
    "log_dir": "t",
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "device_ids": {
      "ar": 10,
      "esr": 10
    }
  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("duplicate device_id"), std::string::npos) << error;
}

TEST(SceneDataTest, NonPositiveDeviceIdRejected) {
  ScopedSceneFile file(R"json({
    "log_dir": "t",
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "device_ids": {
      "ar": 0
    }
  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("must be a positive integer"), std::string::npos) << error;
}

TEST(SceneDataTest, UnknownKeyInDeviceIdsRejected) {
  ScopedSceneFile file(R"json({
    "log_dir": "t",
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "device_ids": {
      "unknown_sensor": 10
    }
  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("unknown device_ids key"), std::string::npos) << error;
}

TEST(SceneDataTest, DeviceIdCollidesWithSatelliteSourceId) {
  ScopedSceneFile file(R"json({
    "log_dir": "t",
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "device_ids": {
      "ar": 4
    },
    "satellites": [
      {"source_id": 4}
    ]
  })json");
  app::SceneData scene;
  std::string error;
  EXPECT_FALSE(app::LoadSceneData(file.path().string().c_str(), &scene, &error));
  EXPECT_NE(error.find("collides with satellites[0].source_id"), std::string::npos) << error;
}
