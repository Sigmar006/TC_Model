/**
 * @file scenes/scene_data.cpp
 * @brief 场景描述加载实现（见 scenes/scene_data.h）。
 */

#include "scenes/scene_data.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>

#include "1q/navigation/AreaCoveragePlanner.h"
#include "scenes/area_division.h"
#include "scenes/ballistic_trajectory.h"
#include "json_reader.h"
#include "config_loaders/airborne_radar/config_loader.h"
#include "config_loaders/electro_optical/config_loader.h"
#include "config_loaders/electronic_warfare/config_loader.h"
#include "config_loaders/remote_identification_radar/config_loader.h"
#include "config_loaders/sar/config_loader.h"
#include "config_loaders/sbirs_sensor/config_loader.h"

namespace component_attachment {
namespace app {
namespace {

/// 读取浮点字段：缺省返回默认值（遵循 config_loader 静默默认惯例）。
double ReadDouble(const examples::JsonValue& value, const char* key, double default_value) {
  return value[key].IsNull() ? default_value : value[key].AsDouble();
}

/// 读取整数字段：缺省返回默认值。
std::int64_t ReadInt(const examples::JsonValue& value, const char* key,
                     std::int64_t default_value) {
  return value[key].IsNull() ? default_value : value[key].AsInt();
}

/// 指令类型字符串 → 枚举（commands[].kind；非法值报错而非静默默认——
/// 指令名拼错会让场景悄然缺失指定/锁定演示）。
bool ParseCommandKind(const std::string& text, CommandKind* kind, std::string* error) {
  if (text == "designate") {
    *kind = CommandKind::kDesignateTarget;
  } else if (text == "engage") {
    *kind = CommandKind::kEngageHighThreat;
  } else if (text == "clear_designation") {
    *kind = CommandKind::kClearDesignation;
  } else {
    *error = "invalid \"commands[].kind\" (must be \"designate\", \"engage\" or "
             "\"clear_designation\")";
    return false;
  }
  return true;
}

/// 必填几何字段校验：缺失时置 error 并返回 false（几何字段静默为 0 会让
/// 场景在错误位置"合理"运行，浪费整轮验证——此处严格于调参字段）。
bool RequireGeometry(const examples::JsonValue& value, const std::string& block,
                     const char* key, std::string* error) {
  if (value[key].IsNull()) {
    *error = "missing required field \"" + std::string(key) + "\" in \"" + block + "\"";
    return false;
  }
  return true;
}

/// 弹道 LLA 数组 [纬度 deg, 经度 deg, 高度 m]（targets[].start/end_lla_deg_m）：
/// 长度 3 且元素全为数值，否则置 error。
bool ParseLlaArray(const examples::JsonValue& value, const std::string& block,
                   oneq::coordinate::LlaPositionDegM* lla, std::string* error) {
  if (value.IsNull() || value.type() != examples::JsonValue::kArray) {
    *error = "\"" + block + "\" must be an array [lat_deg, lon_deg, alt_m]";
    return false;
  }
  if (value.Size() != 3U) {
    *error = "\"" + block + "\" must have exactly 3 elements [lat_deg, lon_deg, alt_m]";
    return false;
  }
  double elements[3] = {0.0, 0.0, 0.0};
  for (std::size_t k = 0U; k < 3U; ++k) {
    const examples::JsonValue::Type type = value[k].type();
    if (type != examples::JsonValue::kInt && type != examples::JsonValue::kDouble) {
      *error = "\"" + block + "\" element " + std::to_string(k) + " must be a number";
      return false;
    }
    elements[k] = value[k].AsDouble();
  }
  *lla = oneq::coordinate::LlaPositionDegM(elements[0], elements[1], elements[2]);
  return true;
}

/// 解析可选 coverage 块（区域巡逻任务）/ mission_area 块（编队切分任务）：
/// 区域（多边形/圆形）+ 规划参数，block_name 用于报错消息定位（"coverage"
/// 或 "mission_area"）。区域形态/模式不匹配等结构性错误在此报出；几何深度
/// 校验（顶点 < 3、间距非正、坐标非法等）由 AreaCoveragePlanner 承担
/// （空计划 → 调用方报错）。
bool ParseCoverageTask(const examples::JsonValue& block, const std::string& block_name,
                       double cruise_altitude_m, double cruise_speed_mps,
                       navigation::CoverageArea* area, navigation::CoveragePlanConfig* config,
                       std::string* error) {
  if (block["kind"].IsString()) {
    const std::string kind = block["kind"].AsString();
    if (kind == "polygon") {
      area->kind = navigation::CoverageAreaKind::kPolygon;
    } else if (kind == "circle") {
      area->kind = navigation::CoverageAreaKind::kCircle;
    } else {
      *error = "invalid \"" + block_name + ".kind\" (must be \"polygon\" or \"circle\")";
      return false;
    }
  }
  if (block["mode"].IsString()) {
    const std::string mode = block["mode"].AsString();
    if (mode == "scan") {
      config->mode = navigation::CoverageMode::kScan;
    } else if (mode == "orbit") {
      config->mode = navigation::CoverageMode::kOrbit;
    } else {
      *error = "invalid \"" + block_name + ".mode\" (must be \"scan\" or \"orbit\")";
      return false;
    }
  }

  if (area->kind == navigation::CoverageAreaKind::kPolygon) {
    const examples::JsonValue& vertices = block["vertices"];
    if (vertices.IsNull() || vertices.type() != examples::JsonValue::kArray) {
      *error = "\"" + block_name + ".vertices\" must be an array (polygon region)";
      return false;
    }
    for (std::size_t i = 0U; i < vertices.Size(); ++i) {
      const examples::JsonValue& vertex = vertices[i];
      const std::string vertex_block = block_name + ".vertices[" + std::to_string(i) + "]";
      if (!RequireGeometry(vertex, vertex_block, "lat_deg", error) ||
          !RequireGeometry(vertex, vertex_block, "lon_deg", error)) {
        return false;
      }
      oneq::coordinate::LlaPositionDegM point;
      point.latitude_deg = vertex["lat_deg"].AsDouble();
      point.longitude_deg = vertex["lon_deg"].AsDouble();
      point.altitude_m = ReadDouble(vertex, "alt_m", 0.0);
      area->polygon.vertices.push_back(point);
    }
  } else {
    const examples::JsonValue& center = block["center"];
    if (center.IsNull() || center.type() != examples::JsonValue::kObject) {
      *error = "\"" + block_name + ".center\" must be an object (circle region)";
      return false;
    }
    if (!RequireGeometry(center, block_name + ".center", "lat_deg", error) ||
        !RequireGeometry(center, block_name + ".center", "lon_deg", error)) {
      return false;
    }
    area->circle.center.latitude_deg = center["lat_deg"].AsDouble();
    area->circle.center.longitude_deg = center["lon_deg"].AsDouble();
    area->circle.center.altitude_m = ReadDouble(center, "alt_m", 0.0);
    area->circle.radius_m = ReadDouble(block, "radius_m", 0.0);
  }

  config->scan_heading_deg = ReadDouble(block, "scan_heading_deg", config->scan_heading_deg);
  config->scan_spacing_m = ReadDouble(block, "scan_spacing_m", config->scan_spacing_m);
  // 高度/速度缺省回退巡航参数（与 platform.waypoints 条目缺省语义一致）。
  config->altitude_m = ReadDouble(block, "altitude_m", cruise_altitude_m);
  config->speed_mps = ReadDouble(block, "speed_mps", cruise_speed_mps);
  config->arrival_radius_m = ReadDouble(block, "arrival_radius_m", 500.0);
  config->orbit_segments = static_cast<std::size_t>(
      ReadInt(block, "orbit_segments", static_cast<std::int64_t>(config->orbit_segments)));
  config->orbit_rings = static_cast<std::size_t>(
      ReadInt(block, "orbit_rings", static_cast<std::int64_t>(config->orbit_rings)));
  return true;
}

/// 用规划器把区域任务展开为航路（coverage 块与编队切分共用）：规划失败
/// （空计划 = 顶点不足/间距非正/模式-区域不匹配等）报错退出，不允许静默
/// 退化为直飞。
bool PlanCoverageRoute(const CoverageTask& task, const std::string& block_name,
                       std::vector<navigation::RoutePoint>* waypoints, std::string* error) {
  navigation::AreaCoveragePlanner planner;
  *waypoints = planner.Plan(task.area, task.config);
  if (waypoints->empty()) {
    *error = "\"" + block_name + "\" planning failed: invalid region or "
             "mode-kind mismatch (need >= 3 vertices / positive scan_spacing_m / "
             "positive radius_m)";
    return false;
  }
  return true;
}

/// 解析平台块（platform 块 / platforms[] 数组条目共用）：原点/起飞航向/
/// 巡航参数 + 显式航路或区域巡逻 coverage 块（互斥）。coverage 规划成功时
/// waypoints 为规划器输出的巡逻航路（planned=true，平台循环巡逻）；规划失败
/// （顶点不足/间距非正/模式-区域不匹配等 → 空计划）报错退出，不允许静默
/// 退化为直飞。
bool ParsePlatformBlock(const examples::JsonValue& block, const std::string& block_name,
                        ScenePlatform* platform, std::string* error) {
  for (const char* key : {"origin_lat_deg", "origin_lon_deg"}) {
    if (!RequireGeometry(block, block_name, key, error)) {
      return false;
    }
  }
  platform->origin.latitude_deg = block["origin_lat_deg"].AsDouble();
  platform->origin.longitude_deg = block["origin_lon_deg"].AsDouble();
  platform->origin.altitude_m = ReadDouble(block, "origin_alt_m", 0.0);
  platform->initial_heading_deg =
      ReadDouble(block, "initial_heading_deg", platform->initial_heading_deg);
  platform->cruise_altitude_m =
      ReadDouble(block, "cruise_altitude_m", platform->cruise_altitude_m);
  platform->cruise_speed_mps =
      ReadDouble(block, "cruise_speed_mps", platform->cruise_speed_mps);

  const examples::JsonValue& waypoints = block["waypoints"];
  if (!waypoints.IsNull()) {
    if (waypoints.type() != examples::JsonValue::kArray) {
      *error = "\"" + block_name + ".waypoints\" must be an array";
      return false;
    }
    for (std::size_t i = 0U; i < waypoints.Size(); ++i) {
      const examples::JsonValue& wp = waypoints[i];
      if (wp["lat_deg"].IsNull() || wp["lon_deg"].IsNull()) {
        *error = block_name + " waypoint[" + std::to_string(i) + "] missing lat_deg/lon_deg";
        return false;
      }
      navigation::RoutePoint point;
      point.position.latitude_deg = wp["lat_deg"].AsDouble();
      point.position.longitude_deg = wp["lon_deg"].AsDouble();
      point.position.altitude_m = ReadDouble(wp, "alt_m", platform->cruise_altitude_m);
      point.speed_mps = ReadDouble(wp, "speed_mps", platform->cruise_speed_mps);
      point.radius_m = ReadDouble(wp, "radius_m", 500.0);
      platform->waypoints.push_back(point);
    }
  }

  // 区域巡逻块（可选）：与显式航路互斥（航路来源歧义直接报错）。
  const examples::JsonValue& coverage = block["coverage"];
  if (!coverage.IsNull()) {
    if (coverage.type() != examples::JsonValue::kObject) {
      *error = "\"" + block_name + ".coverage\" must be an object";
      return false;
    }
    if (!waypoints.IsNull()) {
      *error = "\"" + block_name + ".waypoints\" and \"coverage\" are mutually exclusive "
               "(explicit waypoints vs planned patrol route)";
      return false;
    }
    if (!ParseCoverageTask(coverage, "coverage", platform->cruise_altitude_m,
                           platform->cruise_speed_mps, &platform->coverage.area,
                           &platform->coverage.config, error)) {
      return false;
    }
    if (!PlanCoverageRoute(platform->coverage, block_name + ".coverage",
                           &platform->waypoints, error)) {
      return false;
    }
    platform->coverage.planned = true;
  }
  return true;
}

}  // namespace

namespace {

// 场景体与会话配置解析（定义于本文件尾部；场景加载入口先行声明）。
bool ParseSceneBody(const examples::JsonValue& root, SceneData* scene, std::string* error);
bool LoadSessionConfigs(const examples::JsonValue& root, const SceneData& scene,
                        SceneSessionConfigs* configs, std::string* error);
bool ValidateFullLoadExtras(const examples::JsonValue& root, SceneData* scene,
                            std::string* error);

}  // namespace

bool LoadSceneData(const char* path, SceneData* scene, std::string* error) {
  if (scene == nullptr || error == nullptr) {
    return false;
  }
  error->clear();

  examples::JsonValue root;
  if (!examples::JsonReader::ParseFile(path, &root, error)) {
    return false;
  }
  if (root.IsNull() || root.type() != examples::JsonValue::kObject) {
    *error = "scene root must be a JSON object";
    return false;
  }
  return ParseSceneBody(root, scene, error);
}

bool LoadSceneData(const char* path, SceneData* scene, SceneSessionConfigs* configs,
                   std::string* error) {
  if (scene == nullptr || configs == nullptr || error == nullptr) {
    return false;
  }
  error->clear();

  examples::JsonValue root;
  if (!examples::JsonReader::ParseFile(path, &root, error)) {
    return false;
  }
  if (root.IsNull() || root.type() != examples::JsonValue::kObject) {
    *error = "scene root must be a JSON object";
    return false;
  }
  if (!ParseSceneBody(root, scene, error)) {
    return false;
  }
  if (!ValidateFullLoadExtras(root, scene, error)) {
    return false;
  }
  return LoadSessionConfigs(root, *scene, configs, error);
}

namespace {

bool ParseSceneBody(const examples::JsonValue& root, SceneData* scene,
                    std::string* error) {
  if (scene == nullptr || error == nullptr) {
    return false;
  }
  error->clear();

  SceneData out;  // 成员初始化值 = 缺省字段值
  if (root["name"].IsString()) {
    out.name = root["name"].AsString();
  }
  if (root["cycles"].IsInt()) {
    const std::int64_t cycles = root["cycles"].AsInt();
    if (cycles <= 0) {
      *error = "invalid \"cycles\": must be > 0";
      return false;
    }
    out.cycles = static_cast<std::uint32_t>(cycles);
  }
  if (root["start_cycle"].IsInt()) {
    const std::int64_t start = root["start_cycle"].AsInt();
    if (start <= 0) {
      *error = "invalid \"start_cycle\": must be > 0";
      return false;
    }
    out.start_cycle = static_cast<std::uint32_t>(start);
  }
  if (root["view_log_every_cycles"].IsInt()) {
    const std::int64_t every = root["view_log_every_cycles"].AsInt();
    if (every <= 0) {
      *error = "invalid \"view_log_every_cycles\": must be > 0";
      return false;
    }
    out.view_log_every_cycles = static_cast<std::uint32_t>(every);
  }
  out.dt_sec = ReadDouble(root, "dt_sec", out.dt_sec);

  const examples::JsonValue& sensors = root["sensors"];
  if (!sensors.IsNull() && sensors.type() == examples::JsonValue::kObject) {
    if (sensors.Has("ar")) {
      out.ar_enabled = sensors["ar"].AsBool();
    }
    if (sensors.Has("esr")) {
      out.esr_enabled = sensors["esr"].AsBool();
    }
    if (sensors.Has("eos")) {
      out.eos_enabled = sensors["eos"].AsBool();
    }
    if (sensors.Has("sbirs")) {
      out.sbirs_enabled = sensors["sbirs"].AsBool();
    }
    if (sensors.Has("sar")) {
      out.sar_enabled = sensors["sar"].AsBool();
    }
  }

  // 主平台块（必填）：原点/起飞航向/巡航参数 + 可选航路/区域巡逻。
  const examples::JsonValue& platform = root["platform"];
  if (platform.IsNull() || platform.type() != examples::JsonValue::kObject) {
    *error = "missing required block \"platform\"";
    return false;
  }
  ScenePlatform main_platform;
  if (!ParsePlatformBlock(platform, "platform", &main_platform, error)) {
    return false;
  }
  out.platform_origin = main_platform.origin;
  out.initial_heading_deg = main_platform.initial_heading_deg;
  out.cruise_altitude_m = main_platform.cruise_altitude_m;
  out.cruise_speed_mps = main_platform.cruise_speed_mps;
  out.waypoints = std::move(main_platform.waypoints);
  out.coverage = main_platform.coverage;

  // 从机（可选顶层 platforms[] 数组）：每条目同 platform 块（各自航路/区域 =
  // "不同指令"；纯飞行，不挂传感器）。巡航参数缺省回退主平台值（同编队量级）。
  const examples::JsonValue& platforms = root["platforms"];
  if (!platforms.IsNull()) {
    if (platforms.type() != examples::JsonValue::kArray) {
      *error = "\"platforms\" must be an array";
      return false;
    }
    for (std::size_t i = 0U; i < platforms.Size(); ++i) {
      const examples::JsonValue& entry = platforms[i];
      const std::string block_name = "platforms[" + std::to_string(i) + "]";
      if (entry.IsNull() || entry.type() != examples::JsonValue::kObject) {
        *error = "\"" + block_name + "\" must be an object";
        return false;
      }
      ScenePlatform wing;
      wing.name = "wingman_" + std::to_string(i + 1U);  // 缺省名（可被 name 覆盖）
      if (entry["name"].IsString()) {
        wing.name = entry["name"].AsString();
      }
      wing.cruise_altitude_m = out.cruise_altitude_m;
      wing.cruise_speed_mps = out.cruise_speed_mps;
      if (!ParsePlatformBlock(entry, block_name, &wing, error)) {
        return false;
      }
      out.platforms.push_back(std::move(wing));
    }
  }

  // 编队切分块（可选顶层 mission_area）：主机收到单个覆盖区域 → 加载时自动
  // 切分为每架飞机（主机 + platforms[] 从机）的子区域（分工覆盖：多边形 =
  // 沿扫描航向等宽条带，圆形 = 同心环），再逐机经 AreaCoveragePlanner 自动
  // 生成覆盖航路。与各平台 coverage/waypoints 块互斥（区域来源歧义报错，
  // 同 waypoints/coverage 互斥先例）；编队数 = 1 + platforms.size()，无从机
  // 报错（切分需 >= 2 架飞机）。
  const examples::JsonValue& mission_area = root["mission_area"];
  if (!mission_area.IsNull()) {
    if (mission_area.type() != examples::JsonValue::kObject) {
      *error = "\"mission_area\" must be an object";
      return false;
    }
    FormationMissionArea mission;
    if (!ParseCoverageTask(mission_area, "mission_area", out.cruise_altitude_m,
                           out.cruise_speed_mps, &mission.area, &mission.config, error)) {
      return false;
    }
    if (out.platforms.empty()) {
      *error = "\"mission_area\" requires \"platforms\" (formation division needs "
               ">= 2 aircraft)";
      return false;
    }
    if (!out.waypoints.empty() || out.coverage.planned) {
      *error = "\"mission_area\" and \"platform.waypoints\"/\"platform.coverage\" are "
               "mutually exclusive (single mission area vs per-platform route)";
      return false;
    }
    for (std::size_t i = 0U; i < out.platforms.size(); ++i) {
      if (!out.platforms[i].waypoints.empty() || out.platforms[i].coverage.planned) {
        *error = "\"mission_area\" and \"platforms[" + std::to_string(i) +
                 "].waypoints\"/\".coverage\" are mutually exclusive (single mission "
                 "area vs per-platform route)";
        return false;
      }
    }
    const FormationDivisionResult division =
        DivideArea(mission.area, mission.config, 1U + out.platforms.size());
    if (!division.ok) {
      *error = "\"mission_area\" division failed: " + division.error;
      return false;
    }
    // 主机（下标 0）+ 从机（1..N-1）逐机填入子区域并规划航路（planned=true
    // → FlightComponent 循环巡逻，与 coverage 块语义一致）。
    out.coverage.area = division.sub_areas[0];
    out.coverage.config = division.sub_configs[0];
    if (!PlanCoverageRoute(out.coverage, "mission_area sub-area (host)", &out.waypoints,
                           error)) {
      return false;
    }
    out.coverage.planned = true;
    for (std::size_t i = 0U; i < out.platforms.size(); ++i) {
      out.platforms[i].coverage.area = division.sub_areas[i + 1U];
      out.platforms[i].coverage.config = division.sub_configs[i + 1U];
      const std::string sub_block =
          "mission_area sub-area (" + out.platforms[i].name + ")";
      if (!PlanCoverageRoute(out.platforms[i].coverage, sub_block,
                             &out.platforms[i].waypoints, error)) {
        return false;
      }
      out.platforms[i].coverage.planned = true;
    }
  }

  // 目标块（必填数组；可为空 = "无目标"场景，冒烟下限由 smoke 块置 0）。
  const examples::JsonValue& targets = root["targets"];
  if (targets.IsNull() || targets.type() != examples::JsonValue::kArray) {
    *error = "missing required block \"targets\" (array)";
    return false;
  }
  for (std::size_t i = 0U; i < targets.Size(); ++i) {
    const examples::JsonValue& t = targets[i];
    for (const char* key : {"id", "rcs_m2"}) {
      if (!RequireGeometry(t, "targets[" + std::to_string(i) + "]", key, error)) {
        return false;
      }
    }
    ScriptedTarget target;
    target.id = static_cast<std::uint32_t>(t["id"].AsInt());
    // 实体类型（可选）："air"（缺省）/"ground"（地面目标 = 静止近地运动学点）/
    // "ballistic"（二体椭圆弹道弧线：起止 LLA + 顶高 + 顶高时刻，见
    // scenes/ballistic_trajectory.h）。
    if (t["type"].IsString()) {
      const std::string type = t["type"].AsString();
      if (type != "air" && type != "ground" && type != "ballistic") {
        *error = "targets[" + std::to_string(i) +
                 "].type must be \"air\", \"ground\" or \"ballistic\"";
        return false;
      }
      target.type = type;
    }
    target.is_ballistic = target.type == "ballistic";
    if (target.is_ballistic) {
      // 弹道条目：四键必填；与 ENU 几何/机动表字段互斥（同时出现必是脚本
      // 语义冲突，报错而非静默忽略，避免"写了但不生效"）。
      for (const char* key : {"start_lla_deg_m", "end_lla_deg_m", "max_alt_m",
                              "max_alt_time_s"}) {
        if (!RequireGeometry(t, "targets[" + std::to_string(i) + "]", key, error)) {
          return false;
        }
      }
      for (const char* key : {"azimuth_deg", "range_m", "altitude_m", "v_east_mps",
                              "v_north_mps", "maneuvers"}) {
        if (t.Has(key)) {
          *error = "targets[" + std::to_string(i) + "] (ballistic) must not set \"" +
                   key + "\" (mutually exclusive with the LLA trajectory fields)";
          return false;
        }
      }
      if (!ParseLlaArray(t["start_lla_deg_m"],
                         "targets[" + std::to_string(i) + "].start_lla_deg_m",
                         &target.start_lla, error) ||
          !ParseLlaArray(t["end_lla_deg_m"],
                         "targets[" + std::to_string(i) + "].end_lla_deg_m",
                         &target.end_lla, error)) {
        return false;
      }
      target.max_alt_m = t["max_alt_m"].AsDouble();
      target.max_alt_time_s = t["max_alt_time_s"].AsDouble();
      if (!(target.max_alt_m > 0.0) || !(target.max_alt_time_s > 0.0)) {
        *error = "targets[" + std::to_string(i) +
                 "] (ballistic) max_alt_m and max_alt_time_s must be > 0";
        return false;
      }
      // 加载期可解性校验（起落点可转 ECEF、互不重合/对跖、顶点高于两端点）：
      // 把轨道数据错误拦在场景加载报错通道，不留到运行时静默降级。
      BallisticTrajectory probe;
      if (!SolveBallisticTrajectory(target.start_lla, target.end_lla, target.max_alt_m,
                                    target.max_alt_time_s, &probe)) {
        *error = "targets[" + std::to_string(i) +
                 "] (ballistic): trajectory solve failed (distinct, non-antipodal "
                 "start/end with apogee above both endpoints is required)";
        return false;
      }
    } else {
      for (const char* key : {"azimuth_deg", "range_m", "altitude_m"}) {
        if (!RequireGeometry(t, "targets[" + std::to_string(i) + "]", key, error)) {
          return false;
        }
      }
      target.azimuth_deg = t["azimuth_deg"].AsDouble();
      target.range_m = t["range_m"].AsDouble();
      target.altitude_m = t["altitude_m"].AsDouble();
      target.v_east_mps = ReadDouble(t, "v_east_mps", 0.0);
      target.v_north_mps = ReadDouble(t, "v_north_mps", 0.0);
    }
    target.temperature_k = ReadDouble(t, "temperature_k", 0.0);
    target.rcs = t["rcs_m2"].AsDouble();
    target.projected_area_m2 = ReadDouble(t, "projected_area_m2", 0.0);
    target.radiant_intensity_w_per_sr = ReadDouble(t, "radiant_intensity_w_per_sr", 0.0);
    target.emitter_center_frequency_hz = ReadDouble(t, "emitter_center_frequency_hz", 0.0);
    // 变速机动表（可选）：start_cycle 必填且严格递增（乱序语义混乱，直接报错）。
    const examples::JsonValue& maneuvers = t["maneuvers"];
    if (!maneuvers.IsNull()) {
      if (maneuvers.type() != examples::JsonValue::kArray) {
        *error = "targets[" + std::to_string(i) + "].maneuvers must be an array";
        return false;
      }
      for (std::size_t m = 0U; m < maneuvers.Size(); ++m) {
        const examples::JsonValue& mv = maneuvers[m];
        if (mv["start_cycle"].IsNull()) {
          *error = "targets[" + std::to_string(i) + "].maneuvers[" +
                   std::to_string(m) + "] missing start_cycle";
          return false;
        }
        TargetManeuver maneuver;
        maneuver.start_cycle = static_cast<std::uint32_t>(mv["start_cycle"].AsInt());
        if (maneuver.start_cycle == 0U ||
            (!target.maneuvers.empty() &&
             maneuver.start_cycle <= target.maneuvers.back().start_cycle)) {
          *error = "targets[" + std::to_string(i) + "].maneuvers[" +
                   std::to_string(m) + "] start_cycle must be > 0 and strictly increasing";
          return false;
        }
        maneuver.v_east_mps = ReadDouble(mv, "v_east_mps", 0.0);
        maneuver.v_north_mps = ReadDouble(mv, "v_north_mps", 0.0);
        target.maneuvers.push_back(maneuver);
      }
    }
    // RIR 特征真值块（可选，rir 字段存在即视为携带）：标量 + 散射器脚本。
    if (t.Has("rir")) {
      const examples::JsonValue& rir = t["rir"];
      if (rir.type() != examples::JsonValue::kObject) {
        *error = "targets[" + std::to_string(i) + "].rir must be an object";
        return false;
      }
      target.has_rir_features = true;
      target.rir_rcs_dbsm = ReadDouble(rir, "rcs_dbsm", 0.0);
      // 极化通道显式给值才铺样（0 dBsm 合法，不能按零值判缺省）。
      // 极化散射字典（全量，开机一次载入）：行对象键 az/el/hh_amp/hh_phase/
      // hv_amp/hv_phase/vh_amp/vh_phase/vv_amp/vv_phase；非对象行跳过。
      if (rir.Has("polarization_dictionary") &&
          rir["polarization_dictionary"].type() == examples::JsonValue::kArray) {
        const examples::JsonValue& dictionary = rir["polarization_dictionary"];
        for (std::size_t k = 0U; k < dictionary.Size(); ++k) {
          const examples::JsonValue& row = dictionary[k];
          if (row.type() != examples::JsonValue::kObject) {
            continue;
          }
          remote_identification_radar::session::RirPolSMatrixSample sample;
          sample.aspect_az_deg = static_cast<float>(ReadDouble(row, "az", 0.0));
          sample.aspect_el_deg = static_cast<float>(ReadDouble(row, "el", 0.0));
          sample.hh_amp_db = static_cast<float>(ReadDouble(row, "hh_amp", 0.0));
          sample.hh_phase_deg = static_cast<float>(ReadDouble(row, "hh_phase", 0.0));
          sample.hv_amp_db = static_cast<float>(ReadDouble(row, "hv_amp", 0.0));
          sample.hv_phase_deg = static_cast<float>(ReadDouble(row, "hv_phase", 0.0));
          sample.vh_amp_db = static_cast<float>(ReadDouble(row, "vh_amp", 0.0));
          sample.vh_phase_deg = static_cast<float>(ReadDouble(row, "vh_phase", 0.0));
          sample.vv_amp_db = static_cast<float>(ReadDouble(row, "vv_amp", 0.0));
          sample.vv_phase_deg = static_cast<float>(ReadDouble(row, "vv_phase", 0.0));
          target.rir_pol_dictionary.push_back(sample);
        }
      }
      target.rir_pol_window_deg = ReadDouble(rir, "pol_window_deg", 5.0);
      if (rir["truth_model"].IsString()) {
        target.rir_truth_model = rir["truth_model"].AsString();
      }
      const examples::JsonValue& scatterers = rir["scatterers"];
      if (!scatterers.IsNull()) {
        if (scatterers.type() != examples::JsonValue::kArray) {
          *error = "targets[" + std::to_string(i) + "].rir.scatterers must be an array";
          return false;
        }
        for (std::size_t s = 0U; s < scatterers.Size(); ++s) {
          const examples::JsonValue& sc = scatterers[s];
          RirScattererScript scatterer;
          scatterer.offset_m = ReadDouble(sc, "offset_m", 0.0);
          scatterer.rcs_dbsm = ReadDouble(sc, "rcs_dbsm", 0.0);
          target.rir_scatterers.push_back(scatterer);
        }
      }
    }
    out.targets.push_back(target);
  }

  // ESR 波形块（可选，字段级缺省）。
  const examples::JsonValue& esr = root["esr"];
  if (!esr.IsNull() && esr.type() == examples::JsonValue::kObject) {
    out.esr.peak_gain_dbi = ReadDouble(esr, "peak_gain_dbi", out.esr.peak_gain_dbi);
    out.esr.bandwidth_hz = ReadDouble(esr, "bandwidth_hz", out.esr.bandwidth_hz);
    out.esr.peak_power_w = ReadDouble(esr, "peak_power_w", out.esr.peak_power_w);
    out.esr.pulse_width_s = ReadDouble(esr, "pulse_width_s", out.esr.pulse_width_s);
    out.esr.pri_s = ReadDouble(esr, "pri_s", out.esr.pri_s);
    out.esr.pulse_count =
        static_cast<std::uint32_t>(ReadInt(esr, "pulse_count", out.esr.pulse_count));
    out.esr.timing_seed =
        static_cast<std::uint32_t>(ReadInt(esr, "timing_seed", out.esr.timing_seed));
  }

  // 天基平台块（可选）：凝视目标群质心正上方，高度由场景控制；UTC 儒略日
  // 可选覆写（缺省 = 2024-01-01 00:00 UTC，SBIRS ECI 输出参考系必需）。
  // 焦平面几何与宽→窄命中门属 SBIRS 会话配置（session_config.sbirs）。
  const examples::JsonValue& satellite = root["sbirs_satellite"];
  if (!satellite.IsNull() && satellite.type() == examples::JsonValue::kObject) {
    out.sbirs_satellite_altitude_m =
        ReadDouble(satellite, "altitude_m", out.sbirs_satellite_altitude_m);
    out.sbirs_utc_julian_day =
        ReadDouble(satellite, "utc_julian_day", out.sbirs_utc_julian_day);
  }


  out.high_threat_confidence =
      ReadDouble(root, "high_threat_confidence", out.high_threat_confidence);

  // RIR 地基站点块（可选，enabled=true 时挂载识别雷达组件；站点 LLA 为雷达
  // 局部 ENU 原点；指定识别任务经运行期指令 commands[] 下发）。
  const examples::JsonValue& rir = root["rir"];
  if (!rir.IsNull() && rir.type() == examples::JsonValue::kObject) {
    out.rir_enabled = rir.Has("enabled") ? rir["enabled"].AsBool() : false;
    const examples::JsonValue& site = rir["site"];
    if (!site.IsNull() && site.type() == examples::JsonValue::kObject) {
      out.rir_site_origin.latitude_deg = ReadDouble(site, "lat_deg", out.rir_site_origin.latitude_deg);
      out.rir_site_origin.longitude_deg =
          ReadDouble(site, "lon_deg", out.rir_site_origin.longitude_deg);
      out.rir_site_origin.altitude_m = ReadDouble(site, "alt_m", out.rir_site_origin.altitude_m);
    }
  }

  const examples::JsonValue& ecm = root["ecm"];
  if (!ecm.IsNull() && ecm.type() == examples::JsonValue::kObject) {
    out.ecm_enabled = ecm.Has("enabled") ? ecm["enabled"].AsBool() : false;
  }

  // 运行期指令脚本块（顶层可选 commands[]，缺省空）：按周期派发的外部指令。
  // target_id 必须命中 targets[].id（下发前校验——指令指向不存在的目标属于
  // 场景文件错误，静默跳过会让指定/锁定演示悄然缺失）。
  const examples::JsonValue& commands = root["commands"];
  if (!commands.IsNull()) {
    if (commands.type() != examples::JsonValue::kArray) {
      *error = "\"commands\" must be an array";
      return false;
    }
    for (std::size_t i = 0U; i < commands.Size(); ++i) {
      const examples::JsonValue& entry = commands[i];
      ScriptedCommand command;
      command.start_cycle = static_cast<std::uint32_t>(
          ReadInt(entry, "cycle", static_cast<std::int64_t>(command.start_cycle)));
      const std::string kind_text =
          entry["kind"].IsString() ? entry["kind"].AsString() : "designate";
      if (!ParseCommandKind(kind_text, &command.kind, error)) {
        return false;
      }
      command.target_id = static_cast<std::uint64_t>(
          ReadInt(entry, "target_id", static_cast<std::int64_t>(command.target_id)));
      command.duration_cycles = static_cast<std::uint32_t>(ReadInt(
          entry, "duration_cycles", static_cast<std::int64_t>(command.duration_cycles)));
      if (command.start_cycle == 0U) {
        *error = "\"commands[].cycle\" must be >= 1";
        return false;
      }
      if (command.kind != CommandKind::kClearDesignation) {
        bool target_found = false;
        for (const auto& target : out.targets) {
          if (static_cast<std::uint64_t>(target.id) == command.target_id) {
            target_found = true;
            break;
          }
        }
        if (!target_found) {
          *error = "\"commands[].target_id\" does not match any targets[].id";
          return false;
        }
      }
      out.commands.push_back(command);
    }
  }

  // 冒烟块（可选，缺省全部下限 = 1）。
  const examples::JsonValue& smoke = root["smoke"];
  if (!smoke.IsNull() && smoke.type() == examples::JsonValue::kObject) {
    out.smoke.min_key_events = static_cast<std::uint32_t>(
        ReadInt(smoke, "min_key_events", out.smoke.min_key_events));
    out.smoke.min_sbirs_events = static_cast<std::uint32_t>(
        ReadInt(smoke, "min_sbirs_events", out.smoke.min_sbirs_events));
    out.smoke.min_sar_products = static_cast<std::uint32_t>(
        ReadInt(smoke, "min_sar_products", out.smoke.min_sar_products));
    out.smoke.min_fused_targets = static_cast<std::uint32_t>(
        ReadInt(smoke, "min_fused_targets", out.smoke.min_fused_targets));
    out.smoke.min_rir_recognition_outputs = static_cast<std::uint32_t>(
        ReadInt(smoke, "min_rir_recognition_outputs", out.smoke.min_rir_recognition_outputs));
  }

  // 统一传感器设备实体标识（可选顶层 device_ids 块，缺省各自 fusion::k*SourceId）。
  const examples::JsonValue& dev_ids = root["device_ids"];
  if (!dev_ids.IsNull()) {
    if (dev_ids.type() != examples::JsonValue::kObject) {
      *error = "\"device_ids\" must be an object";
      return false;
    }
    static const char* kKnownDeviceKeys[] = {"ar", "esr", "eos", "sbirs", "rir", "ecm"};
    for (const std::string& key : dev_ids.Keys()) {
      bool known = false;
      for (const char* allowed : kKnownDeviceKeys) {
        if (key == allowed) {
          known = true;
          break;
        }
      }
      if (!known) {
        *error = "unknown device_ids key \"" + key + "\"";
        return false;
      }
    }
    auto parse_positive_u32 = [&](const char* key, std::uint32_t* target) -> bool {
      if (dev_ids.Has(key)) {
        const examples::JsonValue& v = dev_ids[key];
        if (!v.IsInt() || v.AsInt() <= 0 || v.AsInt() > 0xFFFFFFFFLL) {
          *error = std::string("\"device_ids.") + key + "\" must be a positive integer";
          return false;
        }
        *target = static_cast<std::uint32_t>(v.AsInt());
      }
      return true;
    };
    if (!parse_positive_u32("ar", &out.device_ids.ar) ||
        !parse_positive_u32("esr", &out.device_ids.esr) ||
        !parse_positive_u32("eos", &out.device_ids.eos) ||
        !parse_positive_u32("sbirs", &out.device_ids.sbirs) ||
        !parse_positive_u32("rir", &out.device_ids.rir) ||
        !parse_positive_u32("ecm", &out.device_ids.ecm)) {
      return false;
    }

    // 内部冲突校验：device_ids 显式给出的键之间不能重复
    std::unordered_map<std::uint32_t, std::string> seen_ids;
    for (const char* key : kKnownDeviceKeys) {
      if (dev_ids.Has(key)) {
        std::uint32_t val = 0U;
        if (std::string(key) == "ar") val = out.device_ids.ar;
        else if (std::string(key) == "esr") val = out.device_ids.esr;
        else if (std::string(key) == "eos") val = out.device_ids.eos;
        else if (std::string(key) == "sbirs") val = out.device_ids.sbirs;
        else if (std::string(key) == "rir") val = out.device_ids.rir;
        else if (std::string(key) == "ecm") val = out.device_ids.ecm;

        const auto it = seen_ids.find(val);
        if (it != seen_ids.end()) {
          *error = "duplicate device_id " + std::to_string(val) + " between \"" +
                   it->second + "\" and \"" + key + "\"";
          return false;
        }
        seen_ids[val] = key;
      }
    }

    // 与 satellites[].source_id 冲突校验。防御性守卫：现网带 satellites[] 的
    // SBIRS 场景走各自 main 的专属装载（不经本函数），runner 全量装载的允许键
    // 清单也不含 "satellites"——本校验兜的是未来 runner 支持卫星数组时
    // device_ids 与卫星融合通道静默撞号的口子，当前无真实触发路径。
    const examples::JsonValue& satellites = root["satellites"];
    if (!satellites.IsNull() && satellites.type() == examples::JsonValue::kArray) {
      for (std::size_t s = 0U; s < satellites.Size(); ++s) {
        const examples::JsonValue& sat = satellites[s];
        if (sat.Has("source_id") && sat["source_id"].IsInt()) {
          const std::uint32_t sat_source_id = static_cast<std::uint32_t>(sat["source_id"].AsInt());
          const auto it = seen_ids.find(sat_source_id);
          if (it != seen_ids.end()) {
            *error = "device_ids." + it->second + " (" + std::to_string(sat_source_id) +
                     ") collides with satellites[" + std::to_string(s) + "].source_id";
            return false;
          }
        }
      }
    }
  }

  *scene = std::move(out);
  return true;
}

/// 四参加载的额外校验：log_dir 必填 + 顶层未知键拒绝（schema 正式化后的
/// 拼写保护——错键名静默忽略会让调参悄悄失效）。三参重载（单测场景层路径）
/// 不做这两项，保持夹具最小。
bool ValidateFullLoadExtras(const examples::JsonValue& root, SceneData* scene,
                            std::string* error) {
  static const char* kAllowedKeys[] = {
      "name",  "cycles", "start_cycle", "dt_sec", "view_log_every_cycles", "platform",
      "platforms", "mission_area", "targets", "esr", "sbirs_satellite",
      "sensors", "rir", "ecm", "commands", "high_threat_confidence",
      "smoke", "session_config", "log_dir", "device_ids"};
  for (const std::string& key : root.Keys()) {
    bool known = false;
    for (const char* allowed : kAllowedKeys) {
      if (key == allowed) {
        known = true;
        break;
      }
    }
    if (!known) {
      *error = "unknown scene key \"" + key +
               "\"（schema v2 顶层键集见 scenes/README.md；调参多在 "
               "session_config 子块内）";
      return false;
    }
  }
  const examples::JsonValue& log_dir = root["log_dir"];
  if (!log_dir.IsString() || log_dir.AsString().empty()) {
    *error = "missing required \"log_dir\"（场景自带日志输出目录：相对 "
             "examples/log/ 的路径，如 \"baseline_takeoff_east\"）";
    return false;
  }
  const std::string& dir = log_dir.AsString();
  const bool absolute = !dir.empty() && (dir[0] == '/' || dir[1] == ':');
  if (absolute || dir.find("..") != std::string::npos) {
    *error = "invalid \"log_dir\" \"" + dir +
             "\"：须为相对路径且不含 ..（解析基点 = examples/log/）";
    return false;
  }
  scene->log_dir = dir;
  return true;
}

/// RIR 识别库路径解析：编译宏 CA_RIR_DATABASE_PATH 钉定（CMake 注入仓库
/// 绝对路径，Windows 交付免相对路径歧义）；无宏时相对值按 SCENE_CONFIG_DIR
/// （examples/basic_config/）解析。
void ResolveRirDatabasePath(remote_identification_radar::config::RirSessionConfig* config) {
  if (config == nullptr) {
    return;
  }
#if defined(CA_RIR_DATABASE_PATH)
  config->policy.recognition.database_path = CA_RIR_DATABASE_PATH;
#else
  std::string& path = config->policy.recognition.database_path;
  if (path.empty()) {
    return;
  }
  const bool absolute =
      path[0] == '/' ||
      (path.size() > 1U && path[1] == ':');
  if (!absolute) {
    path = std::string(SCENE_CONFIG_DIR) + "/" + path;
  }
#endif
}

/// session_config{} 解析（挂载即全量）：挂载通道必带子块、未挂载通道禁止
/// 携带；子块结构 = examples/basic_config/<域>.json 模板（config_loaders
/// 原样解析，字段缺省 = 库结构体默认）。fusion/threat 恒挂载恒必带（可为
/// 空对象 {}，字段级缺省）；ecm 仅 ecm.enabled 场景携带（原代码默认值
/// MakeDefaultEcmConfig 数据化）。
bool LoadSessionConfigs(const examples::JsonValue& root, const SceneData& scene,
                        SceneSessionConfigs* configs, std::string* error) {
  // ECM 字段级缺省基线（scenes/README 口径 1/1000/1000/kSpot）：仅作下方
  // session_config.ecm 部分字段的回落值。
  configs->ecm.channel_count = 1U;
  configs->ecm.maximum_total_transmit_power_w = 1000.0;
  configs->ecm.maximum_channel_transmit_power_w = 1000.0;
  configs->ecm.default_technique = electronic_countermeasure::EcmTechnique::kSpot;

  const examples::JsonValue& sc = root["session_config"];
  if (sc.IsNull() || sc.type() != examples::JsonValue::kObject) {
    *error =
        "missing required \"session_config\" block（挂载即全量：挂载通道必带"
        "子块，模板见 examples/basic_config/）";
    return false;
  }

  // 挂载 ⇔ 携带一一对应的六域传感器块。
  const struct {
    const char* key;
    bool mounted;
  } sensor_blocks[] = {
      {"ar", scene.ar_enabled},     {"esr", scene.esr_enabled},
      {"eos", scene.eos_enabled},   {"sbirs", scene.sbirs_enabled},
      {"sar", scene.sar_enabled},   {"rir", scene.rir_enabled},
      {"ecm", scene.ecm_enabled},
  };
  for (const auto& block : sensor_blocks) {
    const examples::JsonValue& value = sc[block.key];
    const bool present = !value.IsNull();
    if (block.mounted && (!present || value.type() != examples::JsonValue::kObject)) {
      *error = std::string("session_config.") + block.key +
               " 缺失或非对象：挂载通道必带全量配置"
               "（模板 examples/basic_config/，键名对应 sensors.*/rir.enabled/ecm.enabled）";
      return false;
    }
    if (!block.mounted && present) {
      *error = std::string("session_config.") + block.key +
               " 不应存在：未挂载通道禁止携带配置（挂载开关见 sensors.*/rir/ecm 块）";
      return false;
    }
    if (!present) {
      continue;
    }
    if (std::string(block.key) == "ar") {
      examples::LoadArSessionConfig(value, &configs->ar);
    } else if (std::string(block.key) == "esr") {
      examples::LoadEsrSessionConfig(value, &configs->esr);
    } else if (std::string(block.key) == "eos") {
      examples::LoadEosSessionConfig(value, &configs->eos);
    } else if (std::string(block.key) == "sbirs") {
      examples::LoadSbirsSessionConfig(value, &configs->sbirs);
    } else if (std::string(block.key) == "sar") {
      examples::LoadSarSessionConfig(value, &configs->sar);
    } else if (std::string(block.key) == "rir") {
      examples::LoadRirSessionConfig(value, &configs->rir);
      ResolveRirDatabasePath(&configs->rir);
    } else if (std::string(block.key) == "ecm") {
      configs->ecm.channel_count = static_cast<std::uint32_t>(
          ReadInt(value, "channel_count", static_cast<std::int64_t>(configs->ecm.channel_count)));
      configs->ecm.maximum_total_transmit_power_w =
          ReadDouble(value, "maximum_total_transmit_power_w",
                     configs->ecm.maximum_total_transmit_power_w);
      configs->ecm.maximum_channel_transmit_power_w =
          ReadDouble(value, "maximum_channel_transmit_power_w",
                     configs->ecm.maximum_channel_transmit_power_w);
      if (value["default_technique"].IsString()) {
        const std::string technique = value["default_technique"].AsString();
        if (technique == "kSpot") {
          configs->ecm.default_technique = electronic_countermeasure::EcmTechnique::kSpot;
        } else if (technique == "kBarrage") {
          configs->ecm.default_technique = electronic_countermeasure::EcmTechnique::kBarrage;
        } else if (technique == "kSweep") {
          configs->ecm.default_technique = electronic_countermeasure::EcmTechnique::kSweep;
        } else if (technique == "kDeception") {
          configs->ecm.default_technique = electronic_countermeasure::EcmTechnique::kDeception;
        } else {
          *error = "session_config.ecm.default_technique 非法（kSpot|kBarrage|kSweep|kDeception）";
          return false;
        }
      }
    }
  }

  // 融合/威胁（恒挂载恒必带；字段级缺省 = 库结构体默认）。
  const examples::JsonValue& fusion = sc["fusion"];
  if (fusion.IsNull() || fusion.type() != examples::JsonValue::kObject) {
    *error = "session_config.fusion 缺失（融合恒挂载，可为空对象 {}）";
    return false;
  }
  configs->fusion.position_radius_m =
      ReadDouble(fusion, "position_radius_m", configs->fusion.position_radius_m);
  configs->fusion.bearing_beamwidth_deg =
      ReadDouble(fusion, "bearing_beamwidth_deg", configs->fusion.bearing_beamwidth_deg);
  configs->fusion.feature_threshold =
      ReadDouble(fusion, "feature_threshold", configs->fusion.feature_threshold);
  configs->fusion.window_size = static_cast<std::size_t>(
      ReadInt(fusion, "window_size", static_cast<std::int64_t>(configs->fusion.window_size)));
  configs->fusion.max_missed_cycles = static_cast<std::size_t>(
      ReadInt(fusion, "max_missed_cycles",
              static_cast<std::int64_t>(configs->fusion.max_missed_cycles)));
  const examples::JsonValue& weights = fusion["source_weights"];
  if (!weights.IsNull()) {
    if (weights.type() != examples::JsonValue::kArray) {
      *error = "session_config.fusion.source_weights must be an array";
      return false;
    }
    configs->fusion.source_weights.clear();
    for (std::size_t i = 0U; i < weights.Size(); ++i) {
      configs->fusion.source_weights.push_back(weights[i].AsDouble());
    }
  }

  const examples::JsonValue& threat = sc["threat"];
  if (threat.IsNull() || threat.type() != examples::JsonValue::kObject) {
    *error = "session_config.threat 缺失（威胁评估恒挂载，可为空对象 {}）";
    return false;
  }
  configs->threat.weight_range =
      ReadDouble(threat, "weight_range", configs->threat.weight_range);
  configs->threat.weight_speed =
      ReadDouble(threat, "weight_speed", configs->threat.weight_speed);
  configs->threat.weight_acceleration =
      ReadDouble(threat, "weight_acceleration", configs->threat.weight_acceleration);
  configs->threat.weight_rcs =
      ReadDouble(threat, "weight_rcs", configs->threat.weight_rcs);
  configs->threat.weight_target_probability =
      ReadDouble(threat, "weight_target_probability",
                 configs->threat.weight_target_probability);
  configs->threat.weight_fusion_confidence =
      ReadDouble(threat, "weight_fusion_confidence",
                 configs->threat.weight_fusion_confidence);
  configs->threat.range_near_m =
      ReadDouble(threat, "range_near_m", configs->threat.range_near_m);
  configs->threat.range_far_m =
      ReadDouble(threat, "range_far_m", configs->threat.range_far_m);
  configs->threat.speed_min_mps =
      ReadDouble(threat, "speed_min_mps", configs->threat.speed_min_mps);
  configs->threat.speed_max_mps =
      ReadDouble(threat, "speed_max_mps", configs->threat.speed_max_mps);
  configs->threat.acceleration_max_mps2 =
      ReadDouble(threat, "acceleration_max_mps2",
                 configs->threat.acceleration_max_mps2);
  configs->threat.rcs_min_sqm =
      ReadDouble(threat, "rcs_min_sqm", configs->threat.rcs_min_sqm);
  configs->threat.rcs_max_sqm =
      ReadDouble(threat, "rcs_max_sqm", configs->threat.rcs_max_sqm);
  configs->threat.high_threshold =
      ReadDouble(threat, "high_threshold", configs->threat.high_threshold);
  configs->threat.medium_threshold =
      ReadDouble(threat, "medium_threshold", configs->threat.medium_threshold);
  return true;
}

}  // namespace

}  // namespace app
}  // namespace component_attachment
